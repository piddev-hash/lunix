/* Copyright (c) 2026
 *
 * Permission to use, copy, modify, and/or distribute this software for any
 * purpose with or without fee is hereby granted, provided that the above
 * copyright notice and this permission notice appear in all copies.
 *
 * THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
 * WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
 * ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
 * WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
 * ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
 * OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.
 */

/* kernel/src/xhci.cpp
 * Minimal xHCI USB host controller with HID boot keyboard/mouse support.
 */

#include <sched.h>
#include <string.h>
#include <time.h>
#include <lunix/kbkeys.h>
#include <lunix/kernel/addressspace.h>
#include <lunix/kernel/clock.h>
#include <lunix/kernel/console.h>
#include <lunix/kernel/interrupts.h>
#include <lunix/kernel/keyboard.h>
#include <lunix/kernel/log.h>
#include <lunix/kernel/mouse.h>
#include <lunix/kernel/panic.h>
#include <lunix/kernel/pci.h>
#include <lunix/kernel/physicalmemory.h>
#include <lunix/kernel/thread.h>
#include <lunix/kernel/worker.h>
#include <lunix/kernel/xhci.h>

#define XHCI_MAX_PORTS 127
#define XHCI_MAX_SLOTS 255
#define XHCI_CONTEXT_ENTRIES 32
#define XHCI_RING_TRBS 256
#define XHCI_EVENT_RING_TRBS 256
#define XHCI_CONTROL_BUFFER_SIZE 512
#define XHCI_MAX_REPORT_SIZE 64
#define XHCI_REPORT_QUEUE_DEPTH 16
#define XHCI_PORT_SCAN_INTERVAL_YIELDS 2048
#define USB_KEYBOARD_REPEAT_DELAY_MS 300
#define USB_KEYBOARD_REPEAT_INTERVAL_MS 33

#define XHCI_USBCMD 0x00
#define XHCI_USBCMD_RUN (1 << 0)
#define XHCI_USBCMD_RESET (1 << 1)
#define XHCI_USBCMD_EIE (1 << 2)

#define XHCI_USBSTS 0x04
#define XHCI_USBSTS_HCHALTED (1 << 0)
#define XHCI_USBSTS_EINT (1 << 3)
#define XHCI_USBSTS_CNR (1 << 11)
#define XHCI_USBSTS_HCE (1 << 12)

#define XHCI_CRCR 0x18
#define XHCI_DCBAAP 0x30
#define XHCI_CONFIG 0x38
#define XHCI_PORTSC_BASE 0x400
#define XHCI_PORTSC_STRIDE 0x10

#define XHCI_RUNTIME_IR0 0x20
#define XHCI_IMAN 0x00
#define XHCI_IMOD 0x04
#define XHCI_ERSTSZ 0x08
#define XHCI_ERSTBA 0x10
#define XHCI_ERDP 0x18

#define XHCI_CAP_HCSPARAMS1 0x04
#define XHCI_CAP_HCCPARAMS1 0x10
#define XHCI_CAP_DBOFF 0x14
#define XHCI_CAP_RTSOFF 0x18

#define XHCI_HCS_MAX_SLOTS(x) (((x) >> 0) & 0xFF)
#define XHCI_HCS_MAX_PORTS(x) (((x) >> 24) & 0xFF)

#define XHCI_HCC_64BYTE_CONTEXT (1 << 2)
#define XHCI_HCC_PPC (1 << 3)
#define XHCI_HCC_EXT_CAPS(x) (((x) >> 16) & 0xFFFF)

#define XHCI_DBOFF_MASK 0xFFFFFFFCU
#define XHCI_RTSOFF_MASK (~0x1FU)

#define XHCI_EXT_CAP_LEGACY 1
#define XHCI_EXT_CAP_ID(x) (((x) >> 0) & 0xFF)
#define XHCI_EXT_CAP_NEXT(x) (((x) >> 8) & 0xFF)
#define XHCI_HC_BIOS_OWNED (1 << 16)
#define XHCI_HC_OS_OWNED (1 << 24)
#define XHCI_LEGACY_DISABLE_SMI (((0x7 << 1) | (0xFF << 5) | (0x7 << 17)))
#define XHCI_LEGACY_SMI_EVENTS (0x7 << 29)

#define XHCI_IMAN_IP (1 << 0)
#define XHCI_IMAN_IE (1 << 1)

#define XHCI_PORT_CONNECT (1 << 0)
#define XHCI_PORT_PE (1 << 1)
#define XHCI_PORT_OC (1 << 3)
#define XHCI_PORT_RESET (1 << 4)
#define XHCI_PORT_POWER (1 << 9)
#define XHCI_PORT_SPEED_MASK (0xF << 10)
#define XHCI_PORT_CSC (1 << 17)
#define XHCI_PORT_PEC (1 << 18)
#define XHCI_PORT_WRC (1 << 19)
#define XHCI_PORT_OCC (1 << 20)
#define XHCI_PORT_RC (1 << 21)
#define XHCI_PORT_PLC (1 << 22)
#define XHCI_PORT_CEC (1 << 23)
#define XHCI_PORT_CHANGE_MASK (XHCI_PORT_CSC | XHCI_PORT_PEC | XHCI_PORT_WRC | \
        XHCI_PORT_OCC | XHCI_PORT_RC | XHCI_PORT_PLC | XHCI_PORT_CEC)
#define XHCI_PORT_SPEED(p) (((p) >> 10) & 0xF)

#define XHCI_PORT_RO ((1 << 0) | (1 << 3) | (0xF << 10) | (1 << 30))
#define XHCI_PORT_RWS ((0xF << 5) | (1 << 9) | (0x3 << 14) | (0x7 << 25))

#define XHCI_SLOT_SPEED_SHIFT 20
#define XHCI_SLOT_SPEED_MASK (0xF << XHCI_SLOT_SPEED_SHIFT)
#define XHCI_SLOT_LAST_CTX_SHIFT 27
#define XHCI_SLOT_LAST_CTX_MASK (0x1F << XHCI_SLOT_LAST_CTX_SHIFT)
#define XHCI_SLOT_ROOT_PORT_SHIFT 16

#define XHCI_EP_TYPE_SHIFT 3
#define XHCI_EP_MULT_SHIFT 8
#define XHCI_EP_INTERVAL_SHIFT 16
#define XHCI_EP_MAX_ESIT_PAYLOAD_HI_SHIFT 24
#define XHCI_EP_MAX_BURST_SHIFT 8
#define XHCI_EP_MAX_PACKET_SHIFT 16
#define XHCI_EP_ERROR_COUNT_SHIFT 1
#define XHCI_EP_AVG_TRB_LENGTH_MASK 0xFFFF
#define XHCI_EP_MAX_ESIT_PAYLOAD_LO_SHIFT 16

#define XHCI_EP_CTRL 4
#define XHCI_EP_INT_IN 7

#define XHCI_TRB_CYCLE (1 << 0)
#define XHCI_TRB_TC (1 << 1)
#define XHCI_TRB_ISP (1 << 2)
#define XHCI_TRB_CHAIN (1 << 4)
#define XHCI_TRB_IOC (1 << 5)
#define XHCI_TRB_IDT (1 << 6)
#define XHCI_TRB_DIR_IN (1 << 16)
#define XHCI_TRB_INTR_TARGET(x) (((x) & 0x3FF) << 22)
#define XHCI_TRB_LEN(x) ((x) & 0x1FFFF)
#define XHCI_TRB_TD_SIZE(x) (((x) & 0x1F) << 17)
#define XHCI_TRB_TYPE(x) ((x) << 10)
#define XHCI_TRB_TYPE_MASK (0x3F << 10)
#define XHCI_TRB_GET_TYPE(x) (((x) & XHCI_TRB_TYPE_MASK) >> 10)

#define XHCI_TRB_NORMAL 1
#define XHCI_TRB_SETUP 2
#define XHCI_TRB_DATA 3
#define XHCI_TRB_STATUS 4
#define XHCI_TRB_LINK 6
#define XHCI_TRB_ENABLE_SLOT 9
#define XHCI_TRB_ADDRESS_DEVICE 11
#define XHCI_TRB_CONFIGURE_ENDPOINT 12
#define XHCI_TRB_EVALUATE_CONTEXT 13
#define XHCI_TRB_TRANSFER_EVENT 32
#define XHCI_TRB_COMMAND_COMPLETION 33

#define XHCI_TRB_TX_TYPE_SHIFT 16
#define XHCI_TRB_DATA_OUT 2
#define XHCI_TRB_DATA_IN 3

#define XHCI_COMP_SUCCESS 1
#define XHCI_COMP_SHORT_PACKET 13

#define XHCI_ERST_EHB (1 << 3)

#define USB_DIR_IN 0x80
#define USB_TYPE_STANDARD 0x00
#define USB_TYPE_CLASS 0x20
#define USB_RECIP_DEVICE 0x00
#define USB_RECIP_INTERFACE 0x01

#define USB_REQUEST_GET_DESCRIPTOR 0x06
#define USB_REQUEST_SET_CONFIGURATION 0x09

#define USB_DESCRIPTOR_DEVICE 0x01
#define USB_DESCRIPTOR_CONFIGURATION 0x02
#define USB_DESCRIPTOR_INTERFACE 0x04
#define USB_DESCRIPTOR_ENDPOINT 0x05

#define USB_CLASS_HID 0x03
#define USB_SUBCLASS_BOOT 0x01
#define USB_PROTOCOL_KEYBOARD 0x01
#define USB_PROTOCOL_MOUSE 0x02

#define USB_ENDPOINT_INTERRUPT 0x03
#define USB_ENDPOINT_IN 0x80

#define HID_REQUEST_SET_REPORT 0x09
#define HID_REQUEST_SET_PROTOCOL 0x0B
#define HID_REPORT_TYPE_OUTPUT 0x02

#define PCI_COMMAND_MEMSPACE (1 << 1)
#define PCI_COMMAND_BUSMASTER (1 << 2)

struct PACKED UsbSetupPacket {
    uint8_t requestType;
    uint8_t request;
    uint16_t value;
    uint16_t index;
    uint16_t length;
};

struct PACKED UsbDeviceDescriptor {
    uint8_t length;
    uint8_t descriptorType;
    uint16_t usbVersion;
    uint8_t deviceClass;
    uint8_t deviceSubClass;
    uint8_t deviceProtocol;
    uint8_t maxPacketSize0;
    uint16_t vendorId;
    uint16_t productId;
    uint16_t deviceVersion;
    uint8_t manufacturer;
    uint8_t product;
    uint8_t serialNumber;
    uint8_t numConfigurations;
};

struct PACKED UsbConfigurationDescriptor {
    uint8_t length;
    uint8_t descriptorType;
    uint16_t totalLength;
    uint8_t numInterfaces;
    uint8_t configurationValue;
    uint8_t configuration;
    uint8_t attributes;
    uint8_t maxPower;
};

struct PACKED UsbInterfaceDescriptor {
    uint8_t length;
    uint8_t descriptorType;
    uint8_t interfaceNumber;
    uint8_t alternateSetting;
    uint8_t numEndpoints;
    uint8_t interfaceClass;
    uint8_t interfaceSubClass;
    uint8_t interfaceProtocol;
    uint8_t interface;
};

struct PACKED UsbEndpointDescriptor {
    uint8_t length;
    uint8_t descriptorType;
    uint8_t endpointAddress;
    uint8_t attributes;
    uint16_t maxPacketSize;
    uint8_t interval;
};

struct PACKED XhciTrb {
    volatile uint32_t parameterLo;
    volatile uint32_t parameterHi;
    volatile uint32_t status;
    volatile uint32_t control;
} ALIGNED(16);

struct PACKED XhciErstEntry {
    volatile uint64_t segmentBase;
    volatile uint32_t segmentSize;
    volatile uint32_t reserved;
} ALIGNED(64);

struct PACKED XhciInputControlContext {
    volatile uint32_t dropFlags;
    volatile uint32_t addFlags;
    volatile uint32_t reserved[6];
};

struct PACKED XhciSlotContext {
    volatile uint32_t devInfo;
    volatile uint32_t devInfo2;
    volatile uint32_t ttInfo;
    volatile uint32_t devState;
    volatile uint32_t reserved[4];
};

struct PACKED XhciEndpointContext {
    volatile uint32_t epInfo;
    volatile uint32_t epInfo2;
    volatile uint64_t dequeuePointer;
    volatile uint32_t txInfo;
    volatile uint32_t reserved[3];
};

struct DmaPage {
    DmaPage* next;
    paddr_t phys;
    vaddr_t virt;
    size_t offset;
};

struct BootInterface {
    uint8_t interfaceNumber;
    uint8_t protocol;
    uint8_t endpointAddress;
    uint16_t maxPacketSize;
    uint8_t interval;
};

struct XhciRing {
    XhciTrb* trbs;
    paddr_t phys;
    size_t enqueueIndex;
    bool cycleState;
};

struct XhciEventRing {
    XhciTrb* trbs;
    paddr_t phys;
    size_t dequeueIndex;
    bool cycleState;
};

struct XhciDevice {
    uint8_t slotId;
    uint8_t portNumber;
    uint8_t speed;
    uint16_t controlPacketSize;
    void* inputContext;
    paddr_t inputContextPhys;
    void* outputContext;
    paddr_t outputContextPhys;
    XhciRing ep0Ring;
};

class XhciController;

struct XhciInterruptDevice {
    XhciInterruptDevice* next;
    XhciController* controller;
    XhciDevice* usbDevice;
    XhciRing ring;
    uint8_t* reportBuffer;
    paddr_t reportBufferPhys;
    uint8_t interfaceNumber;
    uint8_t endpointAddress;
    uint8_t epIndex;
    uint8_t protocol;
    uint16_t maxPacketSize;
    KeyboardListener* listener;
    uint8_t oldReport[8];
    uint8_t ledState;
    bool transferPending;
    paddr_t currentTrbPhys;
    size_t currentLength;
    uint8_t reportQueue[XHCI_REPORT_QUEUE_DEPTH][XHCI_MAX_REPORT_SIZE];
    uint8_t reportLengths[XHCI_REPORT_QUEUE_DEPTH];
    uint8_t reportHead;
    uint8_t reportCount;
    uint8_t errorLogsRemaining;
    uint8_t repeatUsage;
    struct timespec repeatDeadline;
};

enum WaitKind {
    WAIT_COMMAND,
    WAIT_TRANSFER,
};

struct WaitState {
    WaitKind kind;
    paddr_t trbPhys;
    uint8_t slotId;
    uint8_t epId;
    bool completed;
    uint32_t completionCode;
    uint32_t completionParam;
    uint32_t remainingLength;
    uint8_t completionSlotId;
};

class XhciController {
public:
    XhciController(vaddr_t mmioBase, size_t mmioSize, int irq);
    NOT_COPYABLE(XhciController);
    NOT_MOVABLE(XhciController);

    void onIrq();
    void processIrqWork();
    bool needsPolling() const { return true; }
    void scanPorts();
    void processInterruptDevices();

    XhciController* nextController;
private:
    void acknowledgePortChanges(unsigned int port);
    void allocateDma(size_t size, size_t alignment, void*& virtualAddress,
            paddr_t& physicalAddress);
    bool armInterruptTransfer(XhciInterruptDevice* device);
    void copyOutputContextToInput(XhciDevice* device);
    bool configureBootInterfaces(XhciDevice* device);
    bool configureInterruptEndpoints(XhciDevice* device,
            BootInterface* interfaces, size_t interfaceCount);
    bool controlTransfer(XhciDevice* device, uint8_t requestType,
            uint8_t request, uint16_t value, uint16_t index, void* data,
            size_t length);
    void drainEventsLocked(WaitState* wait);
    bool enableSlot(uint8_t& slotId);
    bool enumeratePort(unsigned int port);
    size_t eventActualLength(size_t requestedLength, uint32_t remainingLength);
    int findExtendedCapability(uint8_t capabilityId);
    bool getDescriptor(XhciDevice* device, uint8_t type, uint8_t index,
            void* data, size_t length);
    XhciEndpointContext* getInputEndpointContext(XhciDevice* device,
            unsigned int epIndex);
    XhciInputControlContext* getInputControlContext(XhciDevice* device);
    XhciSlotContext* getInputSlotContext(XhciDevice* device);
    XhciSlotContext* getOutputSlotContext(XhciDevice* device);
    void handleInterruptTransferEvent(paddr_t trbPhys, uint8_t slotId,
            uint8_t epId, uint32_t completionCode, uint32_t remainingLength);
    void handleKeyboardReport(XhciInterruptDevice* device,
            const uint8_t* report, size_t length);
    void handleMouseReport(const uint8_t* report, size_t length);
    void initializeController();
    void initializeEventRing();
    void initializeRing(XhciRing& ring);
    bool queueCommand(uint32_t parameterLo, uint32_t parameterHi,
            uint32_t status, uint32_t control, paddr_t& trbPhys);
    paddr_t queueTrb(XhciRing& ring, uint32_t parameterLo, uint32_t parameterHi,
            uint32_t status, uint32_t control);
    uint32_t readCapability(size_t offset);
    uint32_t readOperational(size_t offset);
    uint32_t readPortStatus(unsigned int port);
    uint32_t readRuntime(size_t offset);
    uint64_t readRuntime64(size_t offset);
    void registerInterruptDevice(XhciInterruptDevice* device);
    bool resetPort(unsigned int port, uint8_t& speed);
    void ringCommandDoorbell();
    void ringEndpointDoorbell(uint8_t slotId, unsigned int epIndex);
    void handleKeyboardRepeat(XhciInterruptDevice* device);
    bool setBootProtocol(XhciDevice* device, uint8_t interfaceNumber);
    bool setConfiguration(XhciDevice* device, uint8_t configurationValue);
    bool setKeyboardLeds(XhciInterruptDevice* device, uint8_t leds);
    uint8_t slotSpeedFromPortSpeed(uint8_t speed);
    bool updateControlPacketSize(XhciDevice* device, uint16_t packetSize);
    bool waitForCommand(paddr_t trbPhys, uint32_t timeoutMs, WaitState& result);
    bool waitForTransfer(paddr_t trbPhys, uint8_t slotId, uint8_t epId,
            uint32_t timeoutMs, WaitState& result);
    void writeOperational(size_t offset, uint32_t value);
    void writeOperational64(size_t offset, uint64_t value);
    void writePortStatus(unsigned int port, uint32_t value);
    void writeRuntime(size_t offset, uint32_t value);
    void writeRuntime64(size_t offset, uint64_t value);
private:
    int irq;
    vaddr_t mmioBase;
    size_t mmioSize;
    size_t operationalBase;
    size_t runtimeBase;
    size_t doorbellBase;
    unsigned int numPorts;
    unsigned int maxSlots;
    size_t contextSize;
    bool portPowerControl;
    kthread_mutex_t mutex;
    DmaPage* dmaPages;
    uint64_t* dcbaa;
    paddr_t dcbaaPhys;
    XhciErstEntry* erst;
    paddr_t erstPhys;
    XhciRing commandRing;
    XhciEventRing eventRing;
    uint8_t* controlBuffer;
    paddr_t controlBufferPhys;
    XhciDevice* devicesBySlot[XHCI_MAX_SLOTS + 1];
    XhciInterruptDevice* interruptDevices;
    bool portInitialized[XHCI_MAX_PORTS];
    bool portProbed[XHCI_MAX_PORTS];
    bool running;
    uint16_t hciVersion;
    IrqHandler irqHandler;
    WorkerJob irqJob;
    bool irqJobQueued;
    bool irqWorkPending;
};

static XhciController* firstController;
static bool xhciPollThreadStarted;

static void onXhciIrq(void* user, const InterruptContext*) {
    XhciController* controller = (XhciController*) user;
    controller->onIrq();
}

static void xhciIrqWork(void* context) {
    ((XhciController*) context)->processIrqWork();
}

static int hidUsageToKey(uint8_t usage);

static size_t alignUp(size_t value, size_t alignment) {
    return (value + alignment - 1) & ~(alignment - 1);
}

static bool containsUsage(const uint8_t* usages, uint8_t usage) {
    for (size_t i = 0; i < 6; i++) {
        if (usages[i] == usage) return true;
    }
    return false;
}

static paddr_t combineAddress(uint32_t low, uint32_t high) {
    return ((uint64_t) high << 32) | low;
}

static void splitAddress(paddr_t address, uint32_t& low, uint32_t& high) {
    low = (uint32_t) address;
    high = (uint32_t) (address >> 32);
}

static bool getMonotonicTime(struct timespec& now) {
    Clock* clock = Clock::get(CLOCK_MONOTONIC);
    return clock && clock->getTime(&now) == 0;
}

static bool isRepeatableKey(int key) {
    return key && key != KB_NUMLOCK && key != KB_CAPSLOCK &&
            key != KB_SCROLLLOCK;
}

static uint8_t selectRepeatUsage(const uint8_t* oldReport,
        const uint8_t* current, uint8_t currentRepeatUsage) {
    for (size_t i = 2; i < 8; i++) {
        uint8_t usage = current[i];
        int key = hidUsageToKey(usage);
        if (usage <= 3 || !isRepeatableKey(key) ||
                containsUsage(oldReport + 2, usage)) {
            continue;
        }
        return usage;
    }

    if (currentRepeatUsage > 3 &&
            containsUsage(current + 2, currentRepeatUsage) &&
            isRepeatableKey(hidUsageToKey(currentRepeatUsage))) {
        return currentRepeatUsage;
    }

    for (size_t i = 2; i < 8; i++) {
        uint8_t usage = current[i];
        if (usage <= 3) continue;

        int key = hidUsageToKey(usage);
        if (isRepeatableKey(key)) return usage;
    }

    return 0;
}

static void sleepMilliseconds(unsigned int milliseconds) {
    Clock* clock = Clock::get(CLOCK_MONOTONIC);
    if (!clock) return;

    struct timespec duration;
    duration.tv_sec = milliseconds / 1000;
    duration.tv_nsec = (milliseconds % 1000) * 1000000L;
    clock->nanosleep(0, &duration, nullptr);
}

static int hidUsageToKey(uint8_t usage) {
    switch (usage) {
    case 0x04: return KB_A;
    case 0x05: return KB_B;
    case 0x06: return KB_C;
    case 0x07: return KB_D;
    case 0x08: return KB_E;
    case 0x09: return KB_F;
    case 0x0A: return KB_G;
    case 0x0B: return KB_H;
    case 0x0C: return KB_I;
    case 0x0D: return KB_J;
    case 0x0E: return KB_K;
    case 0x0F: return KB_L;
    case 0x10: return KB_M;
    case 0x11: return KB_N;
    case 0x12: return KB_O;
    case 0x13: return KB_P;
    case 0x14: return KB_Q;
    case 0x15: return KB_R;
    case 0x16: return KB_S;
    case 0x17: return KB_T;
    case 0x18: return KB_U;
    case 0x19: return KB_V;
    case 0x1A: return KB_W;
    case 0x1B: return KB_X;
    case 0x1C: return KB_Y;
    case 0x1D: return KB_Z;
    case 0x1E: return KB_1;
    case 0x1F: return KB_2;
    case 0x20: return KB_3;
    case 0x21: return KB_4;
    case 0x22: return KB_5;
    case 0x23: return KB_6;
    case 0x24: return KB_7;
    case 0x25: return KB_8;
    case 0x26: return KB_9;
    case 0x27: return KB_0;
    case 0x28: return KB_ENTER;
    case 0x29: return KB_ESCAPE;
    case 0x2A: return KB_BACKSPACE;
    case 0x2B: return KB_TAB;
    case 0x2C: return KB_SPACE;
    case 0x2D: return KB_SYMBOL1;
    case 0x2E: return KB_SYMBOL2;
    case 0x2F: return KB_SYMBOL3;
    case 0x30: return KB_SYMBOL4;
    case 0x31: return KB_SYMBOL8;
    case 0x33: return KB_SYMBOL5;
    case 0x34: return KB_SYMBOL6;
    case 0x35: return KB_SYMBOL7;
    case 0x36: return KB_SYMBOL9;
    case 0x37: return KB_SYMBOL10;
    case 0x38: return KB_SYMBOL11;
    case 0x39: return KB_CAPSLOCK;
    case 0x3A: return KB_F1;
    case 0x3B: return KB_F2;
    case 0x3C: return KB_F3;
    case 0x3D: return KB_F4;
    case 0x3E: return KB_F5;
    case 0x3F: return KB_F6;
    case 0x40: return KB_F7;
    case 0x41: return KB_F8;
    case 0x42: return KB_F9;
    case 0x43: return KB_F10;
    case 0x44: return KB_F11;
    case 0x45: return KB_F12;
    case 0x47: return KB_SCROLLLOCK;
    case 0x49: return KB_INSERT;
    case 0x4A: return KB_HOME;
    case 0x4B: return KB_PAGEUP;
    case 0x4C: return KB_DELETE;
    case 0x4D: return KB_END;
    case 0x4E: return KB_PAGEDOWN;
    case 0x4F: return KB_RIGHT;
    case 0x50: return KB_LEFT;
    case 0x51: return KB_DOWN;
    case 0x52: return KB_UP;
    case 0x53: return KB_NUMLOCK;
    case 0x54: return KB_NUMPAD_DIV;
    case 0x55: return KB_NUMPAD_MULT;
    case 0x56: return KB_NUMPAD_MINUS;
    case 0x57: return KB_NUMPAD_PLUS;
    case 0x58: return KB_NUMPAD_ENTER;
    case 0x59: return KB_NUMPAD1;
    case 0x5A: return KB_NUMPAD2;
    case 0x5B: return KB_NUMPAD3;
    case 0x5C: return KB_NUMPAD4;
    case 0x5D: return KB_NUMPAD5;
    case 0x5E: return KB_NUMPAD6;
    case 0x5F: return KB_NUMPAD7;
    case 0x60: return KB_NUMPAD8;
    case 0x61: return KB_NUMPAD9;
    case 0x62: return KB_NUMPAD0;
    case 0x63: return KB_NUMPAD_DOT;
    case 0x64: return KB_SYMBOL12;
    default:
        return 0;
    }
}

static int modifierBitToKey(size_t bit) {
    static const int keys[] = {
        KB_LCONTROL,
        KB_LSHIFT,
        KB_LALT,
        KB_LGUI,
        KB_RCONTROL,
        KB_RSHIFT,
        KB_ALTGR,
        KB_RGUI,
    };
    return keys[bit];
}

static unsigned int endpointIndexFromAddress(uint8_t endpointAddress) {
    unsigned int endpointNumber = endpointAddress & 0xF;
    bool in = endpointAddress & USB_ENDPOINT_IN;
    return endpointNumber * 2 + (in ? 1 : 0) - 1;
}

static uint16_t defaultControlPacketSize(uint8_t speed) {
    switch (speed) {
    case 1:
        return 64;
    case 2:
        return 8;
    case 3:
        return 64;
    case 4:
    case 5:
        return 512;
    default:
        return 8;
    }
}

static unsigned int highestSetBit(unsigned int value) {
    unsigned int bit = 0;
    while (value) {
        bit++;
        value >>= 1;
    }
    return bit;
}

static uint8_t normalizeBootInterruptInterval(uint8_t speed,
        uint8_t interval) {
    if (speed == 3 || speed == 4 || speed == 5) {
        if (interval < 1 || interval > 16) {
            return 7;
        }

        /* Linux usbhid has a quirk path for HS HID devices that still
         * report bInterval in full-speed milliseconds. For boot HID,
         * treat obviously too-large HS values the same way.
         */
        if (speed == 3 && interval > 8) {
            unsigned int fixed = highestSetBit((unsigned int) interval * 8);
            return fixed ? fixed : 7;
        }

        return interval;
    }

    if (interval == 0) {
        return 10;
    }

    return interval;
}

static unsigned int interruptIntervalForDevice(uint8_t speed,
        uint8_t interval) {
    if (interval == 0) return 0;

    if (speed == 3 || speed == 4 || speed == 5) {
        if (interval > 16) interval = 16;
        return interval - 1;
    }

    unsigned int microframes = (unsigned int) interval * 8;
    unsigned int exponent = 0;
    while (exponent < 10 && (1U << (exponent + 1)) <= microframes) {
        exponent++;
    }
    if (exponent < 3) exponent = 3;
    return exponent;
}

static NORETURN void xhciPollLoop() {
    unsigned int scanCountdown = 0;

    while (true) {
        bool rescanPorts = scanCountdown == 0;
        for (XhciController* controller = firstController; controller;
                controller = controller->nextController) {
            if (rescanPorts) {
                controller->scanPorts();
            }
            controller->processInterruptDevices();
        }

        if (rescanPorts) {
            scanCountdown = XHCI_PORT_SCAN_INTERVAL_YIELDS - 1;
        } else {
            scanCountdown--;
        }

        /* Keep fallback polling time-based instead of relying on scheduler
         * fairness, which caused multi-second HID latency on real hardware.
         */
        sleepMilliseconds(1);
    }
}

static void startXhciPollThread(void*) {
    xhciPollLoop();
}

static void ensureXhciPollThread() {
    if (xhciPollThreadStarted) return;
    xhciPollThreadStarted = true;
    Thread::createKernelThread(startXhciPollThread, nullptr);
}

XhciController::XhciController(vaddr_t mmioBase, size_t mmioSize, int irq)
        : irq(irq), mmioBase(mmioBase), mmioSize(mmioSize) {
    uint32_t capLengthVersion = readCapability(0x00);
    uint32_t hcsParams1 = readCapability(XHCI_CAP_HCSPARAMS1);
    uint32_t hccParams1 = readCapability(XHCI_CAP_HCCPARAMS1);
    uint32_t dbOff = readCapability(XHCI_CAP_DBOFF);
    uint32_t rtOff = readCapability(XHCI_CAP_RTSOFF);

    operationalBase = capLengthVersion & 0xFF;
    runtimeBase = rtOff & XHCI_RTSOFF_MASK;
    doorbellBase = dbOff & XHCI_DBOFF_MASK;
    numPorts = XHCI_HCS_MAX_PORTS(hcsParams1);
    if (numPorts > XHCI_MAX_PORTS) numPorts = XHCI_MAX_PORTS;
    maxSlots = XHCI_HCS_MAX_SLOTS(hcsParams1);
    if (maxSlots > XHCI_MAX_SLOTS) maxSlots = XHCI_MAX_SLOTS;
    contextSize = (hccParams1 & XHCI_HCC_64BYTE_CONTEXT) ? 64 : 32;
    portPowerControl = hccParams1 & XHCI_HCC_PPC;
    hciVersion = (capLengthVersion >> 16) & 0xFFFF;

    mutex = KTHREAD_MUTEX_INITIALIZER;
    dmaPages = nullptr;
    dcbaa = nullptr;
    dcbaaPhys = 0;
    erst = nullptr;
    erstPhys = 0;
    commandRing = {};
    eventRing = {};
    controlBuffer = nullptr;
    controlBufferPhys = 0;
    memset(devicesBySlot, 0, sizeof(devicesBySlot));
    interruptDevices = nullptr;
    memset(portInitialized, 0, sizeof(portInitialized));
    memset(portProbed, 0, sizeof(portProbed));
    running = false;
    nextController = nullptr;
    irqHandler = {};
    irqJob = {};
    irqJob.func = xhciIrqWork;
    irqJob.context = this;
    irqJobQueued = false;
    irqWorkPending = false;

    void* virt = nullptr;
    allocateDma((maxSlots + 1) * sizeof(uint64_t), 64, virt, dcbaaPhys);
    dcbaa = (uint64_t*) virt;
    allocateDma(XHCI_EVENT_RING_TRBS * sizeof(XhciTrb), 64, virt,
            eventRing.phys);
    eventRing.trbs = (XhciTrb*) virt;
    allocateDma(sizeof(XhciErstEntry), 64, virt, erstPhys);
    erst = (XhciErstEntry*) virt;
    allocateDma(XHCI_RING_TRBS * sizeof(XhciTrb), 64, virt, commandRing.phys);
    commandRing.trbs = (XhciTrb*) virt;
    allocateDma(XHCI_CONTROL_BUFFER_SIZE, 64, virt, controlBufferPhys);
    controlBuffer = (uint8_t*) virt;

    memset(dcbaa, 0, (maxSlots + 1) * sizeof(uint64_t));
    memset(eventRing.trbs, 0, XHCI_EVENT_RING_TRBS * sizeof(XhciTrb));
    memset(erst, 0, sizeof(XhciErstEntry));
    memset(controlBuffer, 0, XHCI_CONTROL_BUFFER_SIZE);
    initializeRing(commandRing);
    initializeEventRing();

    uint32_t hcsParams2 = readCapability(0x08);
    unsigned int scratchpads = (((hcsParams2 >> 21) & 0x1F) << 5) |
            ((hcsParams2 >> 27) & 0x1F);
    if (scratchpads) {
        uint64_t* scratchpadArray = nullptr;
        paddr_t scratchpadArrayPhys = 0;
        allocateDma(scratchpads * sizeof(uint64_t), 64,
                (void*&) scratchpadArray, scratchpadArrayPhys);
        memset(scratchpadArray, 0, scratchpads * sizeof(uint64_t));
        for (unsigned int i = 0; i < scratchpads; i++) {
            paddr_t scratchpad = PhysicalMemory::popPageFrame32();
            if (!scratchpad) PANIC("Failed to allocate xHCI scratchpad");
            vaddr_t mapped = kernelSpace->mapPhysical(scratchpad, PAGESIZE,
                    PROT_READ | PROT_WRITE);
            if (!mapped) PANIC("Failed to map xHCI scratchpad");
            memset((void*) mapped, 0, PAGESIZE);
            scratchpadArray[i] = scratchpad;
        }
        dcbaa[0] = scratchpadArrayPhys;
    }

    initializeController();
    if (!running) return;

    Log::printf("xHCI controller initialized at 0x%X\n", (unsigned int) mmioBase);

    scanPorts();

    nextController = firstController;
    firstController = this;
    ensureXhciPollThread();
}

void XhciController::acknowledgePortChanges(unsigned int port) {
    uint32_t status = readPortStatus(port);
    writePortStatus(port, (status & (XHCI_PORT_RO | XHCI_PORT_RWS)) |
            (status & XHCI_PORT_CHANGE_MASK));
}

void XhciController::allocateDma(size_t size, size_t alignment,
        void*& virtualAddress, paddr_t& physicalAddress) {
    if (size > PAGESIZE || alignment == 0 || (alignment & (alignment - 1))) {
        PANIC("Invalid xHCI DMA allocation request");
    }

    if (!dmaPages || alignUp(dmaPages->offset, alignment) + size > PAGESIZE) {
        DmaPage* page = xnew DmaPage;
        page->next = dmaPages;
        page->offset = 0;
        page->phys = PhysicalMemory::popPageFrame32();
        if (!page->phys) PANIC("Failed to allocate xHCI DMA page");

        page->virt = kernelSpace->mapPhysical(page->phys, PAGESIZE,
                PROT_READ | PROT_WRITE);
        if (!page->virt) PANIC("Failed to map xHCI DMA page");

        memset((void*) page->virt, 0, PAGESIZE);
        dmaPages = page;
    }

    size_t offset = alignUp(dmaPages->offset, alignment);
    virtualAddress = (void*) (dmaPages->virt + offset);
    physicalAddress = dmaPages->phys + offset;
    memset(virtualAddress, 0, size);
    dmaPages->offset = offset + size;
}

void XhciController::initializeRing(XhciRing& ring) {
    memset(ring.trbs, 0, XHCI_RING_TRBS * sizeof(XhciTrb));
    ring.enqueueIndex = 0;
    ring.cycleState = true;

    uint32_t low = 0;
    uint32_t high = 0;
    splitAddress(ring.phys, low, high);
    ring.trbs[XHCI_RING_TRBS - 1].parameterLo = low;
    ring.trbs[XHCI_RING_TRBS - 1].parameterHi = high;
    ring.trbs[XHCI_RING_TRBS - 1].status = 0;
    ring.trbs[XHCI_RING_TRBS - 1].control = XHCI_TRB_TYPE(XHCI_TRB_LINK) |
            XHCI_TRB_TC | XHCI_TRB_CYCLE;
}

void XhciController::initializeEventRing() {
    memset(eventRing.trbs, 0, XHCI_EVENT_RING_TRBS * sizeof(XhciTrb));
    eventRing.dequeueIndex = 0;
    eventRing.cycleState = true;
    erst->segmentBase = eventRing.phys;
    erst->segmentSize = XHCI_EVENT_RING_TRBS;
    erst->reserved = 0;
}

bool XhciController::armInterruptTransfer(XhciInterruptDevice* device) {
    memset(device->reportBuffer, 0, device->maxPacketSize);

    uint32_t low = 0;
    uint32_t high = 0;
    splitAddress(device->reportBufferPhys, low, high);
    uint32_t control = XHCI_TRB_TYPE(XHCI_TRB_NORMAL) | XHCI_TRB_IOC |
            XHCI_TRB_ISP | XHCI_TRB_DIR_IN;
    device->currentTrbPhys = queueTrb(device->ring, low, high,
            XHCI_TRB_LEN(device->maxPacketSize) | XHCI_TRB_TD_SIZE(0) |
            XHCI_TRB_INTR_TARGET(0), control);
    device->currentLength = device->maxPacketSize;
    device->transferPending = true;
    ringEndpointDoorbell(device->usbDevice->slotId, device->epIndex);
    return true;
}

void XhciController::copyOutputContextToInput(XhciDevice* device) {
    memset(device->inputContext, 0, contextSize * (XHCI_CONTEXT_ENTRIES + 1));
    memcpy((uint8_t*) device->inputContext + contextSize, device->outputContext,
            contextSize * XHCI_CONTEXT_ENTRIES);
}

bool XhciController::configureBootInterfaces(XhciDevice* device) {
    uint8_t configBuffer[XHCI_CONTROL_BUFFER_SIZE];
    BootInterface interfaces[4];
    size_t interfaceCount = 0;

    if (!getDescriptor(device, USB_DESCRIPTOR_CONFIGURATION, 0, configBuffer,
            sizeof(UsbConfigurationDescriptor))) {
        return false;
    }

    UsbConfigurationDescriptor* config =
            (UsbConfigurationDescriptor*) configBuffer;
    if (config->length < sizeof(UsbConfigurationDescriptor)) {
        return false;
    }

    size_t totalLength = config->totalLength;
    if (totalLength > sizeof(configBuffer)) {
        Log::printf("xHCI device on port %u ignored: configuration descriptor "
                "too large\n", device->portNumber);
        return false;
    }

    if (!getDescriptor(device, USB_DESCRIPTOR_CONFIGURATION, 0, configBuffer,
            totalLength)) {
        return false;
    }

    const UsbInterfaceDescriptor* currentInterface = nullptr;
    for (size_t offset = 0; offset + 2 <= totalLength;) {
        uint8_t length = configBuffer[offset];
        uint8_t type = configBuffer[offset + 1];
        if (length == 0 || offset + length > totalLength) break;

        if (type == USB_DESCRIPTOR_INTERFACE &&
                length >= sizeof(UsbInterfaceDescriptor)) {
            currentInterface = (const UsbInterfaceDescriptor*)
                    (configBuffer + offset);
        } else if (type == USB_DESCRIPTOR_ENDPOINT &&
                currentInterface &&
                length >= sizeof(UsbEndpointDescriptor) &&
                currentInterface->interfaceClass == USB_CLASS_HID &&
                currentInterface->interfaceSubClass == USB_SUBCLASS_BOOT &&
                (currentInterface->interfaceProtocol == USB_PROTOCOL_KEYBOARD ||
                currentInterface->interfaceProtocol == USB_PROTOCOL_MOUSE)) {
            const UsbEndpointDescriptor* endpoint =
                    (const UsbEndpointDescriptor*) (configBuffer + offset);
            if ((endpoint->endpointAddress & USB_ENDPOINT_IN) &&
                    (endpoint->attributes & 0x3) == USB_ENDPOINT_INTERRUPT &&
                    interfaceCount < sizeof(interfaces) / sizeof(interfaces[0])) {
                uint8_t interval = normalizeBootInterruptInterval(device->speed,
                        endpoint->interval);
                if (interval != endpoint->interval) {
                    Log::printf("xHCI HID interval adjusted on port %u: %u -> %u\n",
                            device->portNumber, endpoint->interval, interval);
                }
                interfaces[interfaceCount].interfaceNumber =
                        currentInterface->interfaceNumber;
                interfaces[interfaceCount].protocol =
                        currentInterface->interfaceProtocol;
                interfaces[interfaceCount].endpointAddress =
                        endpoint->endpointAddress;
                interfaces[interfaceCount].maxPacketSize =
                        endpoint->maxPacketSize & 0x7FF;
                interfaces[interfaceCount].interval = interval;
                interfaceCount++;
                currentInterface = nullptr;
            }
        }

        offset += length;
    }

    if (interfaceCount == 0) return false;
    if (!setConfiguration(device, config->configurationValue)) {
        return false;
    }

    for (size_t i = 0; i < interfaceCount; i++) {
        if (!setBootProtocol(device, interfaces[i].interfaceNumber)) {
            interfaces[i].protocol = 0;
        }
    }

    return configureInterruptEndpoints(device, interfaces, interfaceCount);
}

bool XhciController::configureInterruptEndpoints(XhciDevice* device,
        BootInterface* interfaces, size_t interfaceCount) {
    XhciInterruptDevice* createdDevices[4] = {};
    size_t createdCount = 0;
    unsigned int lastContext = 1;

    copyOutputContextToInput(device);
    XhciInputControlContext* control = getInputControlContext(device);
    XhciSlotContext* slot = getInputSlotContext(device);
    control->dropFlags = 0;
    control->addFlags = 1;

    for (size_t i = 0; i < interfaceCount; i++) {
        if (!interfaces[i].protocol || interfaces[i].maxPacketSize == 0) {
            continue;
        }

        unsigned int epIndex = endpointIndexFromAddress(
                interfaces[i].endpointAddress);
        if (epIndex >= 31) continue;

        XhciInterruptDevice* interruptDevice = xnew XhciInterruptDevice;
        memset(interruptDevice, 0, sizeof(*interruptDevice));
        interruptDevice->controller = this;
        interruptDevice->usbDevice = device;
        interruptDevice->interfaceNumber = interfaces[i].interfaceNumber;
        interruptDevice->endpointAddress = interfaces[i].endpointAddress;
        interruptDevice->epIndex = epIndex;
        interruptDevice->protocol = interfaces[i].protocol;
        interruptDevice->maxPacketSize = interfaces[i].maxPacketSize;
        interruptDevice->listener = console ? console.operator->() : nullptr;
        interruptDevice->errorLogsRemaining = 4;

        void* virt = nullptr;
        allocateDma(XHCI_RING_TRBS * sizeof(XhciTrb), 64, virt,
                interruptDevice->ring.phys);
        interruptDevice->ring.trbs = (XhciTrb*) virt;
        initializeRing(interruptDevice->ring);

        allocateDma(interruptDevice->maxPacketSize, 64, virt,
                interruptDevice->reportBufferPhys);
        interruptDevice->reportBuffer = (uint8_t*) virt;
        memset(interruptDevice->reportBuffer, 0, interruptDevice->maxPacketSize);

        XhciEndpointContext* endpoint = getInputEndpointContext(device, epIndex);
        memset((void*) endpoint, 0, sizeof(*endpoint));
        uint32_t maxEsitPayload = interfaces[i].maxPacketSize;
        uint32_t interval = interruptIntervalForDevice(device->speed,
                interfaces[i].interval);
        endpoint->epInfo =
                (((maxEsitPayload >> 16) & 0xFF) <<
                XHCI_EP_MAX_ESIT_PAYLOAD_HI_SHIFT) |
                (interval << XHCI_EP_INTERVAL_SHIFT) |
                (0U << XHCI_EP_MULT_SHIFT);
        endpoint->epInfo2 =
                (XHCI_EP_INT_IN << XHCI_EP_TYPE_SHIFT) |
                (3U << XHCI_EP_ERROR_COUNT_SHIFT) |
                ((uint32_t) interfaces[i].maxPacketSize <<
                XHCI_EP_MAX_PACKET_SHIFT);
        endpoint->dequeuePointer = interruptDevice->ring.phys |
                (interruptDevice->ring.cycleState ? 1U : 0U);
        endpoint->txInfo =
                ((maxEsitPayload & 0xFFFF) << XHCI_EP_MAX_ESIT_PAYLOAD_LO_SHIFT) |
                (interfaces[i].maxPacketSize & XHCI_EP_AVG_TRB_LENGTH_MASK);

        control->addFlags |= 1U << (epIndex + 1);
        if (epIndex + 1 > lastContext) lastContext = epIndex + 1;
        createdDevices[createdCount++] = interruptDevice;
    }

    if (createdCount == 0) return false;

    slot->devInfo &= ~XHCI_SLOT_LAST_CTX_MASK;
    slot->devInfo |= lastContext << XHCI_SLOT_LAST_CTX_SHIFT;

    paddr_t commandTrb = 0;
    if (!queueCommand((uint32_t) device->inputContextPhys,
            (uint32_t) (device->inputContextPhys >> 32), 0,
            XHCI_TRB_TYPE(XHCI_TRB_CONFIGURE_ENDPOINT) |
            ((uint32_t) device->slotId << 24), commandTrb)) {
        return false;
    }

    ringCommandDoorbell();

    WaitState wait = {};
    if (!waitForCommand(commandTrb, 1000, wait) ||
            wait.completionCode != XHCI_COMP_SUCCESS) {
        return false;
    }

    for (size_t i = 0; i < createdCount; i++) {
        XhciInterruptDevice* interruptDevice = createdDevices[i];
        if (interruptDevice->protocol == USB_PROTOCOL_KEYBOARD) {
            Log::printf("USB keyboard found on xHCI port\n");
        } else if (interruptDevice->protocol == USB_PROTOCOL_MOUSE) {
            Log::printf("USB mouse found on xHCI port\n");
        }
        registerInterruptDevice(interruptDevice);
    }

    return true;
}

bool XhciController::controlTransfer(XhciDevice* device, uint8_t requestType,
        uint8_t request, uint16_t value, uint16_t index, void* data,
        size_t length) {
    if (!running || length > XHCI_CONTROL_BUFFER_SIZE ||
            device->controlPacketSize == 0) {
        return false;
    }

    AutoLock lock(&mutex);

    if (length > 0) {
        if (requestType & USB_DIR_IN) {
            memset(controlBuffer, 0, length);
        } else {
            memcpy(controlBuffer, data, length);
        }
    }

    uint32_t setupControl = XHCI_TRB_IDT | XHCI_TRB_TYPE(XHCI_TRB_SETUP);
    if (hciVersion >= 0x100 && length > 0) {
        setupControl |= ((requestType & USB_DIR_IN) ? XHCI_TRB_DATA_IN :
                XHCI_TRB_DATA_OUT) << XHCI_TRB_TX_TYPE_SHIFT;
    }

    paddr_t setupTrb = queueTrb(device->ep0Ring,
            requestType | ((uint32_t) request << 8) |
            ((uint32_t) value << 16),
            index | ((uint32_t) length << 16),
            XHCI_TRB_LEN(sizeof(UsbSetupPacket)) | XHCI_TRB_INTR_TARGET(0),
            setupControl);
    (void) setupTrb;

    if (length > 0) {
        uint32_t low = 0;
        uint32_t high = 0;
        splitAddress(controlBufferPhys, low, high);
        uint32_t dataControl = XHCI_TRB_TYPE(XHCI_TRB_DATA) |
                ((requestType & USB_DIR_IN) ? XHCI_TRB_ISP | XHCI_TRB_DIR_IN : 0);
        queueTrb(device->ep0Ring, low, high,
                XHCI_TRB_LEN(length) | XHCI_TRB_TD_SIZE(0) |
                XHCI_TRB_INTR_TARGET(0), dataControl);
    }

    uint32_t statusControl = (length > 0 && (requestType & USB_DIR_IN)) ? 0 :
            XHCI_TRB_DIR_IN;
    paddr_t statusTrb = queueTrb(device->ep0Ring, 0, 0,
            XHCI_TRB_INTR_TARGET(0), statusControl | XHCI_TRB_IOC |
            XHCI_TRB_TYPE(XHCI_TRB_STATUS));

    ringEndpointDoorbell(device->slotId, 0);

    WaitState wait = {};
    if (!waitForTransfer(statusTrb, device->slotId, 1, 1000, wait)) {
        return false;
    }

    if (wait.completionCode != XHCI_COMP_SUCCESS &&
            wait.completionCode != XHCI_COMP_SHORT_PACKET) {
        return false;
    }

    if ((requestType & USB_DIR_IN) && length > 0) {
        memcpy(data, controlBuffer, length);
    }

    return true;
}

void XhciController::drainEventsLocked(WaitState* wait) {
    bool processed = false;

    while (true) {
        XhciTrb* trb = &eventRing.trbs[eventRing.dequeueIndex];
        bool cycle = trb->control & XHCI_TRB_CYCLE;
        if (cycle != eventRing.cycleState) break;

        XhciTrb event = *trb;
        processed = true;

        eventRing.dequeueIndex++;
        if (eventRing.dequeueIndex == XHCI_EVENT_RING_TRBS) {
            eventRing.dequeueIndex = 0;
            eventRing.cycleState = !eventRing.cycleState;
        }

        uint32_t type = XHCI_TRB_GET_TYPE(event.control);
        if (type == XHCI_TRB_COMMAND_COMPLETION) {
            paddr_t trbPhys = combineAddress(event.parameterLo,
                    event.parameterHi);
            if (wait && wait->kind == WAIT_COMMAND &&
                    wait->trbPhys == trbPhys) {
                wait->completed = true;
                wait->completionCode = (event.status >> 24) & 0xFF;
                wait->completionParam = event.status & 0xFFFFFF;
                wait->completionSlotId = (event.control >> 24) & 0xFF;
            }
        } else if (type == XHCI_TRB_TRANSFER_EVENT) {
            paddr_t trbPhys = combineAddress(event.parameterLo,
                    event.parameterHi);
            uint8_t slotId = (event.control >> 24) & 0xFF;
            uint8_t epId = (event.control >> 16) & 0x1F;
            uint32_t completionCode = (event.status >> 24) & 0xFF;
            uint32_t remainingLength = event.status & 0xFFFFFF;

            if (wait && wait->kind == WAIT_TRANSFER &&
                    wait->trbPhys == trbPhys && wait->slotId == slotId &&
                    wait->epId == epId) {
                wait->completed = true;
                wait->completionCode = completionCode;
                wait->remainingLength = remainingLength;
            } else {
                handleInterruptTransferEvent(trbPhys, slotId, epId,
                        completionCode, remainingLength);
            }
        }
    }

    if (processed) {
        paddr_t dequeue = eventRing.phys +
                eventRing.dequeueIndex * sizeof(XhciTrb);
        writeRuntime64(XHCI_RUNTIME_IR0 + XHCI_ERDP, dequeue | XHCI_ERST_EHB);
        writeRuntime(XHCI_RUNTIME_IR0 + XHCI_IMAN, XHCI_IMAN_IP);
    }
}

bool XhciController::enableSlot(uint8_t& slotId) {
    AutoLock lock(&mutex);

    paddr_t commandTrb = 0;
    if (!queueCommand(0, 0, 0, XHCI_TRB_TYPE(XHCI_TRB_ENABLE_SLOT),
            commandTrb)) {
        return false;
    }

    ringCommandDoorbell();

    WaitState wait = {};
    if (!waitForCommand(commandTrb, 1000, wait)) return false;
    if (wait.completionCode != XHCI_COMP_SUCCESS) return false;

    slotId = wait.completionSlotId;
    return slotId != 0;
}

bool XhciController::enumeratePort(unsigned int port) {
    uint8_t speed = 0;
    if (!resetPort(port, speed)) return false;

    uint8_t slotId = 0;
    if (!enableSlot(slotId)) {
        return false;
    }

    XhciDevice* device = xnew XhciDevice;
    memset(device, 0, sizeof(*device));
    device->slotId = slotId;
    device->portNumber = port + 1;
    device->speed = speed;
    device->controlPacketSize = defaultControlPacketSize(speed);

    void* virt = nullptr;
    allocateDma(contextSize * (XHCI_CONTEXT_ENTRIES + 1), 64, virt,
            device->inputContextPhys);
    device->inputContext = virt;
    allocateDma(contextSize * XHCI_CONTEXT_ENTRIES, 64, virt,
            device->outputContextPhys);
    device->outputContext = virt;
    allocateDma(XHCI_RING_TRBS * sizeof(XhciTrb), 64, virt,
            device->ep0Ring.phys);
    device->ep0Ring.trbs = (XhciTrb*) virt;

    memset(device->inputContext, 0, contextSize * (XHCI_CONTEXT_ENTRIES + 1));
    memset(device->outputContext, 0, contextSize * XHCI_CONTEXT_ENTRIES);
    initializeRing(device->ep0Ring);
    devicesBySlot[slotId] = device;
    dcbaa[slotId] = device->outputContextPhys;

    {
        AutoLock lock(&mutex);
        XhciInputControlContext* control = getInputControlContext(device);
        XhciSlotContext* slot = getInputSlotContext(device);
        XhciEndpointContext* ep0 = getInputEndpointContext(device, 0);

        control->dropFlags = 0;
        control->addFlags = 0x3;
        slot->devInfo =
                (1U << XHCI_SLOT_LAST_CTX_SHIFT) |
                ((uint32_t) slotSpeedFromPortSpeed(speed) <<
                XHCI_SLOT_SPEED_SHIFT);
        slot->devInfo2 = ((uint32_t) (port + 1) << XHCI_SLOT_ROOT_PORT_SHIFT);
        ep0->epInfo2 =
                (XHCI_EP_CTRL << XHCI_EP_TYPE_SHIFT) |
                (3U << XHCI_EP_ERROR_COUNT_SHIFT) |
                ((uint32_t) device->controlPacketSize <<
                XHCI_EP_MAX_PACKET_SHIFT);
        ep0->dequeuePointer = device->ep0Ring.phys |
                (device->ep0Ring.cycleState ? 1U : 0U);
        ep0->txInfo = 8;

        paddr_t commandTrb = 0;
        if (!queueCommand((uint32_t) device->inputContextPhys,
                (uint32_t) (device->inputContextPhys >> 32), 0,
                XHCI_TRB_TYPE(XHCI_TRB_ADDRESS_DEVICE) |
                ((uint32_t) slotId << 24), commandTrb)) {
            return false;
        }
        ringCommandDoorbell();

        WaitState wait = {};
        if (!waitForCommand(commandTrb, 1000, wait) ||
                wait.completionCode != XHCI_COMP_SUCCESS) {
            return false;
        }
    }

    sleepMilliseconds(10);

    UsbDeviceDescriptor descriptor = {};
    if (!getDescriptor(device, USB_DESCRIPTOR_DEVICE, 0, &descriptor, 8)) {
        return false;
    }

    if (descriptor.maxPacketSize0 &&
            descriptor.maxPacketSize0 != device->controlPacketSize) {
        if (!updateControlPacketSize(device, descriptor.maxPacketSize0)) {
            return false;
        }
    }

    if (!getDescriptor(device, USB_DESCRIPTOR_DEVICE, 0, &descriptor,
            sizeof(descriptor))) {
        return false;
    }

    if (!configureBootInterfaces(device)) {
        return false;
    }

    return true;
}

size_t XhciController::eventActualLength(size_t requestedLength,
        uint32_t remainingLength) {
    if (remainingLength > requestedLength) return 0;
    return requestedLength - remainingLength;
}

int XhciController::findExtendedCapability(uint8_t capabilityId) {
    uint32_t hccParams = readCapability(XHCI_CAP_HCCPARAMS1);
    uint32_t offset = XHCI_HCC_EXT_CAPS(hccParams) << 2;
    unsigned int loops = 0;

    while (offset && loops < 64) {
        uint32_t value = *(volatile uint32_t*) (mmioBase + offset);
        if (XHCI_EXT_CAP_ID(value) == capabilityId) {
            return offset;
        }

        uint32_t next = XHCI_EXT_CAP_NEXT(value);
        if (!next) break;
        offset += next << 2;
        loops++;
    }

    return 0;
}

bool XhciController::getDescriptor(XhciDevice* device, uint8_t type,
        uint8_t index, void* data, size_t length) {
    return controlTransfer(device,
            USB_DIR_IN | USB_TYPE_STANDARD | USB_RECIP_DEVICE,
            USB_REQUEST_GET_DESCRIPTOR, (type << 8) | index, 0, data, length);
}

XhciEndpointContext* XhciController::getInputEndpointContext(XhciDevice* device,
        unsigned int epIndex) {
    return (XhciEndpointContext*) ((uint8_t*) device->inputContext +
            (epIndex + 2) * contextSize);
}

XhciInputControlContext* XhciController::getInputControlContext(
        XhciDevice* device) {
    return (XhciInputControlContext*) device->inputContext;
}

XhciSlotContext* XhciController::getInputSlotContext(XhciDevice* device) {
    return (XhciSlotContext*) ((uint8_t*) device->inputContext + contextSize);
}

XhciSlotContext* XhciController::getOutputSlotContext(XhciDevice* device) {
    return (XhciSlotContext*) device->outputContext;
}

void XhciController::handleInterruptTransferEvent(paddr_t trbPhys,
        uint8_t slotId, uint8_t epId, uint32_t completionCode,
        uint32_t remainingLength) {
    for (XhciInterruptDevice* device = interruptDevices; device;
            device = device->next) {
        if (!device->transferPending) continue;
        if (device->usbDevice->slotId != slotId) continue;
        if (device->epIndex + 1 != epId) continue;
        if (device->currentTrbPhys != trbPhys) continue;

        device->transferPending = false;
        if (completionCode == XHCI_COMP_SUCCESS ||
                completionCode == XHCI_COMP_SHORT_PACKET) {
            size_t length = eventActualLength(device->currentLength,
                    remainingLength);
            if (length > XHCI_MAX_REPORT_SIZE) {
                length = XHCI_MAX_REPORT_SIZE;
            }

            if (device->reportCount == XHCI_REPORT_QUEUE_DEPTH) {
                device->reportHead = (device->reportHead + 1) %
                        XHCI_REPORT_QUEUE_DEPTH;
                device->reportCount--;
            }

            uint8_t queueIndex = (device->reportHead + device->reportCount) %
                    XHCI_REPORT_QUEUE_DEPTH;
            memcpy(device->reportQueue[queueIndex], device->reportBuffer, length);
            device->reportLengths[queueIndex] = length;
            device->reportCount++;
        } else if (device->errorLogsRemaining) {
            device->errorLogsRemaining--;
            Log::printf("xHCI interrupt transfer error on port %u: code=%u ep=%u\n",
                    device->usbDevice->portNumber, completionCode,
                    device->epIndex + 1);
        }

        armInterruptTransfer(device);
        return;
    }
}

void XhciController::handleKeyboardReport(XhciInterruptDevice* device,
        const uint8_t* report, size_t length) {
    static const struct timespec repeatDelay = {
        USB_KEYBOARD_REPEAT_DELAY_MS / 1000,
        (USB_KEYBOARD_REPEAT_DELAY_MS % 1000) * 1000000L
    };
    uint8_t current[8] = {};
    size_t copyLength = length < sizeof(current) ? length : sizeof(current);
    memcpy(current, report, copyLength);

    for (size_t i = 0; i < 8; i++) {
        bool oldPressed = device->oldReport[0] & (1 << i);
        bool newPressed = current[0] & (1 << i);
        if (oldPressed == newPressed) continue;

        int key = modifierBitToKey(i);
        if (device->listener && key) {
            device->listener->onKeyboardEvent(newPressed ? key : -key);
        }
    }

    for (size_t i = 2; i < 8; i++) {
        uint8_t usage = device->oldReport[i];
        if (usage <= 3 || containsUsage(current + 2, usage)) continue;

        int key = hidUsageToKey(usage);
        if (device->listener && key) device->listener->onKeyboardEvent(-key);
    }

    for (size_t i = 2; i < 8; i++) {
        uint8_t usage = current[i];
        if (usage <= 3 || containsUsage(device->oldReport + 2, usage)) continue;

        int key = hidUsageToKey(usage);
        if (!key) continue;

        if (key == KB_NUMLOCK) {
            device->ledState ^= 0x01;
            setKeyboardLeds(device, device->ledState);
        } else if (key == KB_CAPSLOCK) {
            device->ledState ^= 0x02;
            setKeyboardLeds(device, device->ledState);
        } else if (key == KB_SCROLLLOCK) {
            device->ledState ^= 0x04;
            setKeyboardLeds(device, device->ledState);
        }

        if (device->listener) device->listener->onKeyboardEvent(key);
    }

    uint8_t repeatUsage = selectRepeatUsage(device->oldReport, current,
            device->repeatUsage);
    if (repeatUsage != device->repeatUsage) {
        device->repeatUsage = repeatUsage;
        if (repeatUsage && getMonotonicTime(device->repeatDeadline)) {
            device->repeatDeadline = timespecPlus(device->repeatDeadline,
                    repeatDelay);
        } else if (repeatUsage) {
            device->repeatUsage = 0;
        }
    }

    memcpy(device->oldReport, current, sizeof(device->oldReport));
}

void XhciController::handleKeyboardRepeat(XhciInterruptDevice* device) {
    static const struct timespec repeatInterval = {
        USB_KEYBOARD_REPEAT_INTERVAL_MS / 1000,
        (USB_KEYBOARD_REPEAT_INTERVAL_MS % 1000) * 1000000L
    };
    if (!device->repeatUsage || !device->listener) return;

    struct timespec now;
    if (!getMonotonicTime(now) || timespecLess(now, device->repeatDeadline)) {
        return;
    }

    int key = hidUsageToKey(device->repeatUsage);
    if (!isRepeatableKey(key)) {
        device->repeatUsage = 0;
        return;
    }

    device->listener->onKeyboardEvent(key);
    do {
        device->repeatDeadline = timespecPlus(device->repeatDeadline,
                repeatInterval);
    } while (!timespecLess(now, device->repeatDeadline));
}

void XhciController::handleMouseReport(const uint8_t* report, size_t length) {
    if (length < 3 || !mouseDevice) return;

    mouse_data data = {};
    if (report[0] & 0x01) data.mouse_flags |= MOUSE_LEFT;
    if (report[0] & 0x02) data.mouse_flags |= MOUSE_RIGHT;
    if (report[0] & 0x04) data.mouse_flags |= MOUSE_MIDDLE;

    data.mouse_x = (int8_t) report[1];
    data.mouse_y = (int8_t) report[2];

    if (length >= 4) {
        int8_t wheel = (int8_t) report[3];
        if (wheel > 0) {
            data.mouse_flags |= MOUSE_SCROLL_UP;
        } else if (wheel < 0) {
            data.mouse_flags |= MOUSE_SCROLL_DOWN;
        }
    }

    mouseDevice->addPacket(data);
}

void XhciController::initializeController() {
    if (irq >= 0) {
        irqHandler.func = onXhciIrq;
        irqHandler.user = this;
        Interrupts::addIrqHandler(irq, &irqHandler);
    }

    uint32_t command = readOperational(XHCI_USBCMD);
    if (command & XHCI_USBCMD_RUN) {
        writeOperational(XHCI_USBCMD, command & ~XHCI_USBCMD_RUN);
        for (unsigned int i = 0; i < 1000; i++) {
            if (readOperational(XHCI_USBSTS) & XHCI_USBSTS_HCHALTED) break;
            sleepMilliseconds(1);
        }
    }

    writeOperational(XHCI_USBCMD, readOperational(XHCI_USBCMD) |
            XHCI_USBCMD_RESET);
    for (unsigned int i = 0; i < 1000; i++) {
        if (!(readOperational(XHCI_USBCMD) & XHCI_USBCMD_RESET)) break;
        sleepMilliseconds(1);
    }
    for (unsigned int i = 0; i < 5000; i++) {
        if (!(readOperational(XHCI_USBSTS) & XHCI_USBSTS_CNR)) break;
        sleepMilliseconds(1);
    }

    if (readOperational(XHCI_USBSTS) & XHCI_USBSTS_CNR) {
        Log::printf("xHCI controller at 0x%X never became ready\n",
                (unsigned int) mmioBase);
        return;
    }

    writeOperational64(XHCI_DCBAAP, dcbaaPhys);
    writeOperational64(XHCI_CRCR, commandRing.phys |
            (commandRing.cycleState ? 1U : 0U));
    writeRuntime(XHCI_RUNTIME_IR0 + XHCI_IMAN, XHCI_IMAN_IP);
    writeRuntime(XHCI_RUNTIME_IR0 + XHCI_IMOD, 0);
    writeRuntime(XHCI_RUNTIME_IR0 + XHCI_ERSTSZ, 1);
    writeRuntime64(XHCI_RUNTIME_IR0 + XHCI_ERSTBA, erstPhys);
    writeRuntime64(XHCI_RUNTIME_IR0 + XHCI_ERDP, eventRing.phys | XHCI_ERST_EHB);
    writeOperational(XHCI_CONFIG, maxSlots);

    if (portPowerControl) {
        for (unsigned int port = 0; port < numPorts; port++) {
            uint32_t status = readPortStatus(port);
            writePortStatus(port, (status & (XHCI_PORT_RO | XHCI_PORT_RWS)) |
                    XHCI_PORT_POWER | (status & XHCI_PORT_CHANGE_MASK));
        }
        sleepMilliseconds(20);
    }

    command = readOperational(XHCI_USBCMD) | XHCI_USBCMD_RUN;
    if (irq >= 0) command |= XHCI_USBCMD_EIE;
    writeOperational(XHCI_USBCMD, command);
    if (irq >= 0) {
        writeRuntime(XHCI_RUNTIME_IR0 + XHCI_IMAN,
                XHCI_IMAN_IP | XHCI_IMAN_IE);
    }
    for (unsigned int i = 0; i < 1000; i++) {
        uint32_t status = readOperational(XHCI_USBSTS);
        if (!(status & XHCI_USBSTS_HCHALTED) &&
                !(status & XHCI_USBSTS_CNR)) {
            running = true;
            break;
        }
        sleepMilliseconds(1);
    }

    if (!running) {
        Log::printf("xHCI controller at 0x%X failed to start\n",
                (unsigned int) mmioBase);
    }
}

void XhciController::onIrq() {
    if (!running || irq < 0) return;

    uint32_t status = readOperational(XHCI_USBSTS);
    uint32_t iman = readRuntime(XHCI_RUNTIME_IR0 + XHCI_IMAN);
    if (!(status & (XHCI_USBSTS_EINT | XHCI_USBSTS_HCE)) &&
            !(iman & XHCI_IMAN_IP)) {
        return;
    }

    if (status & XHCI_USBSTS_EINT) {
        writeOperational(XHCI_USBSTS, XHCI_USBSTS_EINT);
    }
    if (iman & XHCI_IMAN_IP) {
        writeRuntime(XHCI_RUNTIME_IR0 + XHCI_IMAN, iman | XHCI_IMAN_IP);
    }

    if (status & XHCI_USBSTS_HCE) {
        Log::printf("xHCI controller error 0x%X\n", status);
    }

    irqWorkPending = true;
    if (!irqJobQueued) {
        irqJobQueued = true;
        WorkerThread::addJob(&irqJob);
    }
}

void XhciController::processIrqWork() {
    while (true) {
        Interrupts::disable();
        irqWorkPending = false;
        Interrupts::enable();

        processInterruptDevices();

        Interrupts::disable();
        if (!irqWorkPending) {
            irqJobQueued = false;
            Interrupts::enable();
            return;
        }
        Interrupts::enable();
    }
}

bool XhciController::queueCommand(uint32_t parameterLo, uint32_t parameterHi,
        uint32_t status, uint32_t control, paddr_t& trbPhys) {
    trbPhys = queueTrb(commandRing, parameterLo, parameterHi, status, control);
    return true;
}

paddr_t XhciController::queueTrb(XhciRing& ring, uint32_t parameterLo,
        uint32_t parameterHi, uint32_t status, uint32_t control) {
    size_t index = ring.enqueueIndex;
    XhciTrb* trb = &ring.trbs[index];
    paddr_t trbPhys = ring.phys + index * sizeof(XhciTrb);

    trb->parameterLo = parameterLo;
    trb->parameterHi = parameterHi;
    trb->status = status;
    trb->control = control | (ring.cycleState ? XHCI_TRB_CYCLE : 0);

    ring.enqueueIndex++;
    if (ring.enqueueIndex == XHCI_RING_TRBS - 1) {
        uint32_t low = 0;
        uint32_t high = 0;
        splitAddress(ring.phys, low, high);
        ring.trbs[XHCI_RING_TRBS - 1].parameterLo = low;
        ring.trbs[XHCI_RING_TRBS - 1].parameterHi = high;
        ring.trbs[XHCI_RING_TRBS - 1].status = 0;
        ring.trbs[XHCI_RING_TRBS - 1].control = XHCI_TRB_TYPE(XHCI_TRB_LINK) |
                XHCI_TRB_TC | (ring.cycleState ? XHCI_TRB_CYCLE : 0);
        ring.enqueueIndex = 0;
        ring.cycleState = !ring.cycleState;
    }

    return trbPhys;
}

uint32_t XhciController::readCapability(size_t offset) {
    return *(volatile uint32_t*) (mmioBase + offset);
}

uint32_t XhciController::readOperational(size_t offset) {
    return *(volatile uint32_t*) (mmioBase + operationalBase + offset);
}

uint32_t XhciController::readPortStatus(unsigned int port) {
    return readOperational(XHCI_PORTSC_BASE + port * XHCI_PORTSC_STRIDE);
}

uint32_t XhciController::readRuntime(size_t offset) {
    return *(volatile uint32_t*) (mmioBase + runtimeBase + offset);
}

uint64_t XhciController::readRuntime64(size_t offset) {
    return *(volatile uint64_t*) (mmioBase + runtimeBase + offset);
}

void XhciController::registerInterruptDevice(XhciInterruptDevice* device) {
    AutoLock lock(&mutex);
    device->next = interruptDevices;
    interruptDevices = device;
    armInterruptTransfer(device);
}

bool XhciController::resetPort(unsigned int port, uint8_t& speed) {
    uint32_t status = readPortStatus(port);
    if (!(status & XHCI_PORT_CONNECT)) return false;

    if (portPowerControl && !(status & XHCI_PORT_POWER)) {
        writePortStatus(port, (status & (XHCI_PORT_RO | XHCI_PORT_RWS)) |
                XHCI_PORT_POWER | (status & XHCI_PORT_CHANGE_MASK));
        sleepMilliseconds(20);
        status = readPortStatus(port);
    }

    acknowledgePortChanges(port);
    status = readPortStatus(port);

    writePortStatus(port, (status & (XHCI_PORT_RO | XHCI_PORT_RWS)) |
            XHCI_PORT_RESET | (status & XHCI_PORT_CHANGE_MASK));

    for (unsigned int i = 0; i < 100; i++) {
        sleepMilliseconds(1);
        status = readPortStatus(port);
        if (!(status & XHCI_PORT_RESET)) break;
    }

    acknowledgePortChanges(port);
    status = readPortStatus(port);

    if (!(status & XHCI_PORT_CONNECT) || !(status & XHCI_PORT_PE)) {
        return false;
    }

    speed = XHCI_PORT_SPEED(status);
    return speed != 0;
}

void XhciController::ringCommandDoorbell() {
    *(volatile uint32_t*) (mmioBase + doorbellBase) = 0;
}

void XhciController::ringEndpointDoorbell(uint8_t slotId,
        unsigned int epIndex) {
    *(volatile uint32_t*) (mmioBase + doorbellBase + slotId * sizeof(uint32_t)) =
            (epIndex + 1) & 0xFF;
}

bool XhciController::setBootProtocol(XhciDevice* device,
        uint8_t interfaceNumber) {
    uint8_t dummy = 0;
    return controlTransfer(device, USB_TYPE_CLASS | USB_RECIP_INTERFACE,
            HID_REQUEST_SET_PROTOCOL, 0, interfaceNumber, &dummy, 0);
}

bool XhciController::setConfiguration(XhciDevice* device,
        uint8_t configurationValue) {
    uint8_t dummy = 0;
    return controlTransfer(device,
            USB_TYPE_STANDARD | USB_RECIP_DEVICE,
            USB_REQUEST_SET_CONFIGURATION, configurationValue, 0, &dummy, 0);
}

bool XhciController::setKeyboardLeds(XhciInterruptDevice* device,
        uint8_t leds) {
    return controlTransfer(device->usbDevice,
            USB_TYPE_CLASS | USB_RECIP_INTERFACE, HID_REQUEST_SET_REPORT,
            HID_REPORT_TYPE_OUTPUT << 8, device->interfaceNumber, &leds,
            sizeof(leds));
}

uint8_t XhciController::slotSpeedFromPortSpeed(uint8_t speed) {
    switch (speed) {
    case 1: return 1;
    case 2: return 2;
    case 3: return 3;
    case 4: return 4;
    case 5: return 5;
    default: return 0;
    }
}

bool XhciController::updateControlPacketSize(XhciDevice* device,
        uint16_t packetSize) {
    AutoLock lock(&mutex);

    copyOutputContextToInput(device);
    XhciInputControlContext* control = getInputControlContext(device);
    XhciEndpointContext* ep0 = getInputEndpointContext(device, 0);
    control->dropFlags = 0;
    control->addFlags = 0x2;
    ep0->epInfo2 &= ~(0xFFFFU << XHCI_EP_MAX_PACKET_SHIFT);
    ep0->epInfo2 |= (uint32_t) packetSize << XHCI_EP_MAX_PACKET_SHIFT;

    paddr_t commandTrb = 0;
    if (!queueCommand((uint32_t) device->inputContextPhys,
            (uint32_t) (device->inputContextPhys >> 32), 0,
            XHCI_TRB_TYPE(XHCI_TRB_EVALUATE_CONTEXT) |
            ((uint32_t) device->slotId << 24), commandTrb)) {
        return false;
    }

    ringCommandDoorbell();

    WaitState wait = {};
    if (!waitForCommand(commandTrb, 1000, wait) ||
            wait.completionCode != XHCI_COMP_SUCCESS) {
        return false;
    }

    device->controlPacketSize = packetSize;
    return true;
}

bool XhciController::waitForCommand(paddr_t trbPhys, uint32_t timeoutMs,
        WaitState& result) {
    WaitState wait = {};
    wait.kind = WAIT_COMMAND;
    wait.trbPhys = trbPhys;

    Clock* clock = Clock::get(CLOCK_MONOTONIC);
    struct timespec now;
    struct timespec timeout;
    timeout.tv_sec = timeoutMs / 1000;
    timeout.tv_nsec = (timeoutMs % 1000) * 1000000L;
    if (!clock || clock->getTime(&now) < 0) return false;
    struct timespec deadline = timespecPlus(now, timeout);

    while (!wait.completed) {
        drainEventsLocked(&wait);
        if (wait.completed) break;

        clock->getTime(&now);
        if (!timespecLess(now, deadline)) return false;
        sched_yield();
    }

    result = wait;
    return true;
}

bool XhciController::waitForTransfer(paddr_t trbPhys, uint8_t slotId,
        uint8_t epId, uint32_t timeoutMs, WaitState& result) {
    WaitState wait = {};
    wait.kind = WAIT_TRANSFER;
    wait.trbPhys = trbPhys;
    wait.slotId = slotId;
    wait.epId = epId;

    Clock* clock = Clock::get(CLOCK_MONOTONIC);
    struct timespec now;
    struct timespec timeout;
    timeout.tv_sec = timeoutMs / 1000;
    timeout.tv_nsec = (timeoutMs % 1000) * 1000000L;
    if (!clock || clock->getTime(&now) < 0) return false;
    struct timespec deadline = timespecPlus(now, timeout);

    while (!wait.completed) {
        drainEventsLocked(&wait);
        if (wait.completed) break;

        clock->getTime(&now);
        if (!timespecLess(now, deadline)) return false;
        sched_yield();
    }

    result = wait;
    return true;
}

void XhciController::writeOperational(size_t offset, uint32_t value) {
    *(volatile uint32_t*) (mmioBase + operationalBase + offset) = value;
}

void XhciController::writeOperational64(size_t offset, uint64_t value) {
    *(volatile uint64_t*) (mmioBase + operationalBase + offset) = value;
}

void XhciController::writePortStatus(unsigned int port, uint32_t value) {
    writeOperational(XHCI_PORTSC_BASE + port * XHCI_PORTSC_STRIDE, value);
}

void XhciController::writeRuntime(size_t offset, uint32_t value) {
    *(volatile uint32_t*) (mmioBase + runtimeBase + offset) = value;
}

void XhciController::writeRuntime64(size_t offset, uint64_t value) {
    *(volatile uint64_t*) (mmioBase + runtimeBase + offset) = value;
}

void XhciController::processInterruptDevices() {
    if (!running) return;

    {
        AutoLock lock(&mutex);
        drainEventsLocked(nullptr);
    }

    for (XhciInterruptDevice* device = interruptDevices; device;
            device = device->next) {
        while (true) {
            uint8_t report[XHCI_MAX_REPORT_SIZE];
            size_t length = 0;

            {
                AutoLock lock(&mutex);
                if (!device->reportCount) break;

                uint8_t queueIndex = device->reportHead;
                length = device->reportLengths[queueIndex];
                memcpy(report, device->reportQueue[queueIndex], length);
                device->reportHead = (device->reportHead + 1) %
                        XHCI_REPORT_QUEUE_DEPTH;
                device->reportCount--;
            }

            if (device->protocol == USB_PROTOCOL_KEYBOARD) {
                handleKeyboardReport(device, report, length);
            } else if (device->protocol == USB_PROTOCOL_MOUSE) {
                handleMouseReport(report, length);
            }
        }

        if (device->protocol == USB_PROTOCOL_KEYBOARD) {
            handleKeyboardRepeat(device);
        }
    }
}

void XhciController::scanPorts() {
    if (!running) return;

    for (unsigned int port = 0; port < numPorts; port++) {
        uint32_t status = readPortStatus(port);
        bool connected = status & XHCI_PORT_CONNECT;

        if (!connected) {
            portInitialized[port] = false;
            portProbed[port] = false;
            continue;
        }

        if (portProbed[port]) continue;

        portProbed[port] = true;
        if (enumeratePort(port)) {
            portInitialized[port] = true;
            Log::printf("xHCI port %u enumerated\n", port + 1);
        } else {
            Log::printf("xHCI port %u probe failed, waiting for disconnect "
                    "before retry\n", port + 1);
        }
    }
}

static bool readMemoryBar(unsigned int bus, unsigned int device,
        unsigned int function, unsigned int barIndex, paddr_t& baseAddress,
        size_t& size) {
    unsigned int offset = offsetof(PciHeader, bar0) + barIndex * sizeof(uint32_t);
    uint32_t originalLow = Pci::readConfig(bus, device, function, offset);
    if (originalLow & 0x1) return false;

    bool is64Bit = (originalLow & 0x6) == 0x4;
    uint32_t originalHigh = 0;
    if (is64Bit) {
        originalHigh = Pci::readConfig(bus, device, function, offset + 4);
    }

    Pci::writeConfig(bus, device, function, offset, 0xFFFFFFFF);
    if (is64Bit) {
        Pci::writeConfig(bus, device, function, offset + 4, 0xFFFFFFFF);
    }

    uint32_t sizeLow = Pci::readConfig(bus, device, function, offset);
    uint32_t sizeHigh = is64Bit ? Pci::readConfig(bus, device, function,
            offset + 4) : 0;

    Pci::writeConfig(bus, device, function, offset, originalLow);
    if (is64Bit) {
        Pci::writeConfig(bus, device, function, offset + 4, originalHigh);
    }

    uint64_t base = ((uint64_t) originalHigh << 32) | (originalLow & ~0xFULL);
    uint64_t mask = ((uint64_t) sizeHigh << 32) | (sizeLow & ~0xFULL);
    uint64_t barSize = (~mask) + 1;
    if (!barSize) return false;

    baseAddress = base;
    size = barSize;
    return true;
}

void Xhci::initialize(uint8_t bus, uint8_t device, uint8_t function) {
    paddr_t baseAddress = 0;
    size_t barSize = 0;
    if (!readMemoryBar(bus, device, function, 0, baseAddress, barSize)) {
        Log::printf("xHCI controller unsupported: BAR0 is not a memory BAR\n");
        return;
    }

    uint32_t command = Pci::readConfig(bus, device, function,
            offsetof(PciHeader, command));
    command |= PCI_COMMAND_MEMSPACE | PCI_COMMAND_BUSMASTER;
    Pci::writeConfig(bus, device, function, offsetof(PciHeader, command),
            command);

    size_t mapSize = alignUp(barSize, PAGESIZE);
    if (mapSize < 4 * PAGESIZE) mapSize = 4 * PAGESIZE;

    vaddr_t mmioBase = kernelSpace->mapPhysical(baseAddress, mapSize,
            PROT_READ | PROT_WRITE);
    if (!mmioBase) PANIC("Failed to map xHCI registers");

    uint32_t capLengthVersion = *(volatile uint32_t*) mmioBase;
    size_t operationalBase = capLengthVersion & 0xFF;
    uint32_t usbSts = *(volatile uint32_t*) (mmioBase + operationalBase +
            XHCI_USBSTS);
    if (usbSts & XHCI_USBSTS_CNR) {
        for (unsigned int i = 0; i < 5000; i++) {
            usbSts = *(volatile uint32_t*) (mmioBase + operationalBase +
                    XHCI_USBSTS);
            if (!(usbSts & XHCI_USBSTS_CNR)) break;
            sleepMilliseconds(1);
        }
    }

    uint32_t hccParams = *(volatile uint32_t*) (mmioBase + XHCI_CAP_HCCPARAMS1);
    uint32_t offset = XHCI_HCC_EXT_CAPS(hccParams) << 2;
    unsigned int loops = 0;
    while (offset && loops < 64) {
        uint32_t value = *(volatile uint32_t*) (mmioBase + offset);
        if (XHCI_EXT_CAP_ID(value) == XHCI_EXT_CAP_LEGACY) {
            if (value & XHCI_HC_BIOS_OWNED) {
                *(volatile uint32_t*) (mmioBase + offset) =
                        value | XHCI_HC_OS_OWNED;
                for (unsigned int i = 0; i < 1000; i++) {
                    value = *(volatile uint32_t*) (mmioBase + offset);
                    if (!(value & XHCI_HC_BIOS_OWNED)) break;
                    sleepMilliseconds(1);
                }
                if (value & XHCI_HC_BIOS_OWNED) {
                    *(volatile uint32_t*) (mmioBase + offset) =
                            (value | XHCI_HC_OS_OWNED) &
                            ~XHCI_HC_BIOS_OWNED;
                }
            }

            uint32_t legacyControl = *(volatile uint32_t*) (mmioBase + offset + 4);
            legacyControl &= XHCI_LEGACY_DISABLE_SMI;
            legacyControl |= XHCI_LEGACY_SMI_EVENTS;
            *(volatile uint32_t*) (mmioBase + offset + 4) = legacyControl;
            break;
        }

        uint32_t next = XHCI_EXT_CAP_NEXT(value);
        if (!next) break;
        offset += next << 2;
        loops++;
    }

    int irq = Pci::getIrq(bus, device, function);
    if (irq < 0) {
        irq = Pci::readConfig(bus, device, function,
                offsetof(PciHeader, interruptLine)) & 0xFF;
        if (irq == 0 || irq == 0xFF) irq = -1;
    }
    if (irq < 0) {
        Log::printf("xHCI controller at 0x%X is using polling mode\n",
                (unsigned int) baseAddress);
    }
    xnew XhciController(mmioBase, mapSize, irq);
}
