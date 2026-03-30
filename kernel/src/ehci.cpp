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

/* kernel/src/ehci.cpp
 * Minimal EHCI USB host controller with HID boot keyboard/mouse support.
 */

#include <sched.h>
#include <string.h>
#include <time.h>
#include <lunix/kbkeys.h>
#include <lunix/kernel/addressspace.h>
#include <lunix/kernel/clock.h>
#include <lunix/kernel/console.h>
#include <lunix/kernel/ehci.h>
#include <lunix/kernel/interrupts.h>
#include <lunix/kernel/keyboard.h>
#include <lunix/kernel/log.h>
#include <lunix/kernel/mouse.h>
#include <lunix/kernel/panic.h>
#include <lunix/kernel/pci.h>
#include <lunix/kernel/physicalmemory.h>
#include <lunix/kernel/thread.h>
#include <lunix/kernel/worker.h>

#define EHCI_MAX_PORTS 15
#define CONTROL_BUFFER_SIZE 512
#define CONTROL_QTD_COUNT 16
#define FRAME_LIST_ENTRIES 1024
#define EHCI_PORT_SCAN_INTERVAL_YIELDS 2048
#define USB_KEYBOARD_REPEAT_DELAY_MS 300
#define USB_KEYBOARD_REPEAT_INTERVAL_MS 33

#define EHCI_USBCMD 0x00
#define EHCI_USBCMD_RUN (1 << 0)
#define EHCI_USBCMD_RESET (1 << 1)
#define EHCI_USBCMD_PSE (1 << 4)
#define EHCI_USBCMD_ASE (1 << 5)

#define EHCI_USBSTS 0x04
#define EHCI_USBSTS_INT (1 << 0)
#define EHCI_USBSTS_ERR (1 << 1)
#define EHCI_USBSTS_PCD (1 << 2)
#define EHCI_USBSTS_FATAL (1 << 4)
#define EHCI_USBSTS_IAA (1 << 5)
#define EHCI_USBSTS_HCHALTED (1 << 12)
#define EHCI_USBINTR_MASK (EHCI_USBSTS_IAA | EHCI_USBSTS_FATAL | \
        EHCI_USBSTS_PCD | EHCI_USBSTS_ERR | EHCI_USBSTS_INT)

#define EHCI_USBINTR 0x08
#define EHCI_FRINDEX 0x0C
#define EHCI_CTRLDSSEGMENT 0x10
#define EHCI_PERIODICLISTBASE 0x14
#define EHCI_ASYNCLISTADDR 0x18
#define EHCI_CONFIGFLAG 0x40
#define EHCI_PORTSC_BASE 0x44
#define EHCI_PORTSC_STRIDE 0x04

#define EHCI_PORTSC_CCS (1 << 0)
#define EHCI_PORTSC_CSC (1 << 1)
#define EHCI_PORTSC_PE (1 << 2)
#define EHCI_PORTSC_PEC (1 << 3)
#define EHCI_PORTSC_OCA (1 << 4)
#define EHCI_PORTSC_OCC (1 << 5)
#define EHCI_PORTSC_FPR (1 << 6)
#define EHCI_PORTSC_SUSPEND (1 << 7)
#define EHCI_PORTSC_PR (1 << 8)
#define EHCI_PORTSC_PP (1 << 12)
#define EHCI_PORTSC_OWNER (1 << 13)

#define EHCI_QH_LINK_TERMINATE 0x00000001U
#define EHCI_QH_LINK_TYPE_QH 0x00000002U

#define EHCI_QTD_STATUS_PING (1 << 0)
#define EHCI_QTD_STATUS_SPLIT (1 << 1)
#define EHCI_QTD_STATUS_MMF (1 << 2)
#define EHCI_QTD_STATUS_XACTERR (1 << 3)
#define EHCI_QTD_STATUS_BABBLE (1 << 4)
#define EHCI_QTD_STATUS_DBUFERR (1 << 5)
#define EHCI_QTD_STATUS_HALTED (1 << 6)
#define EHCI_QTD_STATUS_ACTIVE (1 << 7)
#define EHCI_QTD_PID_OUT (0U << 8)
#define EHCI_QTD_PID_IN (1U << 8)
#define EHCI_QTD_PID_SETUP (2U << 8)
#define EHCI_QTD_CERR_SHIFT 10
#define EHCI_QTD_IOC (1 << 15)
#define EHCI_QTD_BYTES_SHIFT 16
#define EHCI_QTD_BYTES_MASK 0x7FFFU
#define EHCI_QTD_DT (1U << 31)

#define EHCI_QH_EPCHAR_DEVADDR_SHIFT 0
#define EHCI_QH_EPCHAR_ENDPOINT_SHIFT 8
#define EHCI_QH_EPCHAR_EPS_SHIFT 12
#define EHCI_QH_EPCHAR_DTC (1 << 14)
#define EHCI_QH_EPCHAR_HEAD (1 << 15)
#define EHCI_QH_EPCHAR_MPS_SHIFT 16
#define EHCI_QH_EPCHAR_CONTROL (1 << 27)

#define EHCI_QH_EPCAPS_SMASK_SHIFT 0
#define EHCI_QH_EPCAPS_CMASK_SHIFT 8
#define EHCI_QH_EPCAPS_MULT_SHIFT 30

#define EHCI_EPS_FULLSPEED 0
#define EHCI_EPS_LOWSPEED 1
#define EHCI_EPS_HIGHSPEED 2

#define USB_DIR_IN 0x80
#define USB_TYPE_STANDARD 0x00
#define USB_TYPE_CLASS 0x20
#define USB_RECIP_DEVICE 0x00
#define USB_RECIP_INTERFACE 0x01

#define USB_REQUEST_GET_DESCRIPTOR 0x06
#define USB_REQUEST_SET_ADDRESS 0x05
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

struct PACKED EhciQtd {
    volatile uint32_t nextQtd;
    volatile uint32_t altNextQtd;
    volatile uint32_t token;
    volatile uint32_t buffer[5];
    volatile uint32_t bufferHi[5];
} ALIGNED(32);

struct PACKED EhciQh {
    volatile uint32_t horizLink;
    volatile uint32_t epChars;
    volatile uint32_t epCaps;
    volatile uint32_t currentQtd;
    volatile uint32_t nextQtd;
    volatile uint32_t altNextQtd;
    volatile uint32_t token;
    volatile uint32_t buffer[5];
    volatile uint32_t bufferHi[5];
} ALIGNED(32);

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
};

class EhciController;

struct EhciInterruptDevice {
    EhciInterruptDevice* next;
    EhciController* controller;
    EhciQh* qh;
    paddr_t qhPhys;
    EhciQtd* qtd;
    paddr_t qtdPhys;
    uint8_t* reportBuffer;
    paddr_t reportBufferPhys;
    uint8_t address;
    uint8_t controlPacketSize;
    uint8_t interfaceNumber;
    uint8_t endpointAddress;
    uint16_t maxPacketSize;
    uint8_t protocol;
    uint8_t toggle;
    uint8_t oldReport[8];
    uint8_t ledState;
    uint8_t repeatUsage;
    struct timespec repeatDeadline;
};

class EhciController {
public:
    EhciController(vaddr_t mmioBase, int irq);
    NOT_COPYABLE(EhciController);
    NOT_MOVABLE(EhciController);

    void scanPorts();
    void processInterruptDevices();
    void onIrq();
    void processIrqWork();
    bool needsPolling() const { return true; }

    EhciController* nextController;
private:
    void allocateDma(size_t size, size_t alignment, void*& virtualAddress,
            paddr_t& physicalAddress);
    bool armInterruptTransfer(EhciInterruptDevice* device, bool advanceToggle);
    bool configureBootInterfaces(uint8_t address, uint8_t packetSize0);
    bool controlTransfer(uint8_t address, uint8_t packetSize0,
            uint8_t requestType, uint8_t request, uint16_t value,
            uint16_t index, void* data, size_t length);
    bool enumeratePort(unsigned int port);
    bool getDescriptor(uint8_t address, uint8_t packetSize0, uint8_t type,
            uint8_t index, void* data, size_t length);
    void handleKeyboardReport(EhciInterruptDevice* device,
            const uint8_t* report, size_t length);
    void handleMouseReport(const uint8_t* report, size_t length);
    void initializeController();
    uint32_t readRegister(size_t offset);
    uint32_t readPortStatus(unsigned int port);
    void registerInterruptDevice(EhciInterruptDevice* device);
    bool resetPort(unsigned int port);
    bool setAddress(uint8_t oldAddress, uint8_t packetSize0,
            uint8_t newAddress);
    bool setBootProtocol(uint8_t address, uint8_t packetSize0,
            uint8_t interfaceNumber);
    bool setConfiguration(uint8_t address, uint8_t packetSize0,
            uint8_t configurationValue);
    bool setKeyboardLeds(EhciInterruptDevice* device, uint8_t leds);
    void writeRegister(size_t offset, uint32_t value);
private:
    int irq;
    vaddr_t mmioBase;
    size_t operationalBase;
    unsigned int numPorts;
    kthread_mutex_t mutex;
    DmaPage* dmaPages;
    uint32_t* frameList;
    paddr_t frameListPhys;
    EhciQh* asyncHeadQh;
    paddr_t asyncHeadQhPhys;
    EhciQh* controlQh;
    paddr_t controlQhPhys;
    EhciQh* periodicHeadQh;
    paddr_t periodicHeadQhPhys;
    EhciQtd* controlQtds;
    paddr_t controlQtdsPhys;
    UsbSetupPacket* controlSetup;
    paddr_t controlSetupPhys;
    uint8_t* controlBuffer;
    paddr_t controlBufferPhys;
    EhciInterruptDevice* interruptDevices;
    uint8_t nextAddress;
    bool portInitialized[EHCI_MAX_PORTS];
    bool portProbed[EHCI_MAX_PORTS];
    bool running;
    IrqHandler irqHandler;
    WorkerJob irqJob;
    bool irqJobQueued;
    bool irqWorkPending;
};

static EhciController* firstController;
static bool ehciPollThreadStarted;

static void onEhciIrq(void* user, const InterruptContext*) {
    EhciController* controller = (EhciController*) user;
    controller->onIrq();
}

static void ehciIrqWork(void* context) {
    ((EhciController*) context)->processIrqWork();
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

static uint32_t makeQhLink(paddr_t physicalAddress) {
    return (uint32_t) physicalAddress | EHCI_QH_LINK_TYPE_QH;
}

static uint32_t makeQtdLink(paddr_t physicalAddress) {
    return (uint32_t) physicalAddress;
}

static void fillQtdBuffers(EhciQtd* qtd, paddr_t physicalAddress, size_t size) {
    memset((void*) qtd->buffer, 0, sizeof(qtd->buffer));
    memset((void*) qtd->bufferHi, 0, sizeof(qtd->bufferHi));

    if (size == 0) return;

    qtd->buffer[0] = physicalAddress;
    paddr_t pageBase = physicalAddress & ~0xFFFULL;
    for (size_t i = 1; i < 5; i++) {
        pageBase += 0x1000;
        qtd->buffer[i] = pageBase;
    }
}

static size_t qtdActualLength(size_t requestedLength, uint32_t token) {
    size_t remaining = (token >> EHCI_QTD_BYTES_SHIFT) & EHCI_QTD_BYTES_MASK;
    if (remaining > requestedLength) return 0;
    return requestedLength - remaining;
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

static NORETURN void ehciPollLoop() {
    unsigned int scanCountdown = 0;

    while (true) {
        bool rescanPorts = scanCountdown == 0;
        for (EhciController* controller = firstController; controller;
                controller = controller->nextController) {
            if (rescanPorts) {
                controller->scanPorts();
            }
            controller->processInterruptDevices();
        }

        if (rescanPorts) {
            scanCountdown = EHCI_PORT_SCAN_INTERVAL_YIELDS - 1;
        } else {
            scanCountdown--;
        }

        /* Keep fallback polling time-based instead of relying on scheduler
         * fairness, which caused multi-second HID latency on real hardware.
         */
        sleepMilliseconds(1);
    }
}

static void startEhciPollThread(void*) {
    ehciPollLoop();
}

static void ensureEhciPollThread() {
    if (ehciPollThreadStarted) return;
    ehciPollThreadStarted = true;
    Thread::createKernelThread(startEhciPollThread, nullptr);
}

EhciController::EhciController(vaddr_t mmioBase, int irq)
        : irq(irq), mmioBase(mmioBase) {
    uint8_t capLength = *(volatile uint8_t*) mmioBase;
    uint32_t hcsParams = *(volatile uint32_t*) (mmioBase + 0x04);

    operationalBase = capLength;
    numPorts = hcsParams & 0xF;
    if (numPorts > EHCI_MAX_PORTS) numPorts = EHCI_MAX_PORTS;

    mutex = KTHREAD_MUTEX_INITIALIZER;
    dmaPages = nullptr;
    frameList = nullptr;
    frameListPhys = 0;
    asyncHeadQh = nullptr;
    asyncHeadQhPhys = 0;
    controlQh = nullptr;
    controlQhPhys = 0;
    periodicHeadQh = nullptr;
    periodicHeadQhPhys = 0;
    controlQtds = nullptr;
    controlQtdsPhys = 0;
    controlSetup = nullptr;
    controlSetupPhys = 0;
    controlBuffer = nullptr;
    controlBufferPhys = 0;
    interruptDevices = nullptr;
    nextAddress = 1;
    memset(portInitialized, 0, sizeof(portInitialized));
    memset(portProbed, 0, sizeof(portProbed));
    running = false;
    nextController = nullptr;
    irqHandler = {};
    irqJob = {};
    irqJob.func = ehciIrqWork;
    irqJob.context = this;
    irqJobQueued = false;
    irqWorkPending = false;

    void* virt = nullptr;
    allocateDma(FRAME_LIST_ENTRIES * sizeof(uint32_t), PAGESIZE, virt,
            frameListPhys);
    frameList = (uint32_t*) virt;
    allocateDma(sizeof(EhciQh), 32, virt, asyncHeadQhPhys);
    asyncHeadQh = (EhciQh*) virt;
    allocateDma(sizeof(EhciQh), 32, virt, controlQhPhys);
    controlQh = (EhciQh*) virt;
    allocateDma(sizeof(EhciQh), 32, virt, periodicHeadQhPhys);
    periodicHeadQh = (EhciQh*) virt;
    allocateDma(CONTROL_QTD_COUNT * sizeof(EhciQtd), 32, virt, controlQtdsPhys);
    controlQtds = (EhciQtd*) virt;
    allocateDma(sizeof(UsbSetupPacket), 8, virt, controlSetupPhys);
    controlSetup = (UsbSetupPacket*) virt;
    allocateDma(CONTROL_BUFFER_SIZE, 32, virt, controlBufferPhys);
    controlBuffer = (uint8_t*) virt;

    memset(frameList, 0, FRAME_LIST_ENTRIES * sizeof(uint32_t));
    memset(asyncHeadQh, 0, sizeof(*asyncHeadQh));
    memset(controlQh, 0, sizeof(*controlQh));
    memset(periodicHeadQh, 0, sizeof(*periodicHeadQh));

    asyncHeadQh->horizLink = makeQhLink(controlQhPhys);
    asyncHeadQh->epChars = EHCI_QH_EPCHAR_HEAD;
    asyncHeadQh->nextQtd = EHCI_QH_LINK_TERMINATE;
    asyncHeadQh->altNextQtd = EHCI_QH_LINK_TERMINATE;

    controlQh->horizLink = makeQhLink(asyncHeadQhPhys);
    controlQh->nextQtd = EHCI_QH_LINK_TERMINATE;
    controlQh->altNextQtd = EHCI_QH_LINK_TERMINATE;

    periodicHeadQh->horizLink = EHCI_QH_LINK_TERMINATE;
    periodicHeadQh->nextQtd = EHCI_QH_LINK_TERMINATE;
    periodicHeadQh->altNextQtd = EHCI_QH_LINK_TERMINATE;

    // Always point the frame list at a valid inactive QH, which also matches
    // the AMD SB700/SB800 dummy-QH workaround used by Linux.
    for (size_t i = 0; i < FRAME_LIST_ENTRIES; i++) {
        frameList[i] = makeQhLink(periodicHeadQhPhys);
    }

    initializeController();
    if (!running) return;

    Log::printf("EHCI controller initialized at 0x%X\n", (unsigned int) mmioBase);

    scanPorts();

    nextController = firstController;
    firstController = this;
    ensureEhciPollThread();
}

void EhciController::allocateDma(size_t size, size_t alignment,
        void*& virtualAddress, paddr_t& physicalAddress) {
    if (size > PAGESIZE || alignment == 0 || (alignment & (alignment - 1))) {
        PANIC("Invalid EHCI DMA allocation request");
    }

    if (!dmaPages || alignUp(dmaPages->offset, alignment) + size > PAGESIZE) {
        DmaPage* page = xnew DmaPage;
        page->next = dmaPages;
        page->offset = 0;
        page->phys = PhysicalMemory::popPageFrame32();
        if (!page->phys) PANIC("Failed to allocate EHCI DMA page");

        page->virt = kernelSpace->mapPhysical(page->phys, PAGESIZE,
                PROT_READ | PROT_WRITE);
        if (!page->virt) PANIC("Failed to map EHCI DMA page");

        memset((void*) page->virt, 0, PAGESIZE);
        dmaPages = page;
    }

    size_t offset = alignUp(dmaPages->offset, alignment);
    virtualAddress = (void*) (dmaPages->virt + offset);
    physicalAddress = dmaPages->phys + offset;
    memset(virtualAddress, 0, size);
    dmaPages->offset = offset + size;
}

bool EhciController::armInterruptTransfer(EhciInterruptDevice* device,
        bool advanceToggle) {
    if (advanceToggle) {
        device->toggle ^= 1;
    }

    memset(device->reportBuffer, 0, device->maxPacketSize);
    device->qtd->nextQtd = EHCI_QH_LINK_TERMINATE;
    device->qtd->altNextQtd = EHCI_QH_LINK_TERMINATE;
    device->qtd->token = EHCI_QTD_STATUS_ACTIVE |
            (3U << EHCI_QTD_CERR_SHIFT) |
            EHCI_QTD_IOC |
            ((uint32_t) device->maxPacketSize << EHCI_QTD_BYTES_SHIFT) |
            EHCI_QTD_PID_IN |
            (device->toggle ? EHCI_QTD_DT : 0);
    fillQtdBuffers(device->qtd, device->reportBufferPhys, device->maxPacketSize);

    device->qh->nextQtd = makeQtdLink(device->qtdPhys);
    device->qh->altNextQtd = EHCI_QH_LINK_TERMINATE;
    device->qh->token = 0;
    return true;
}

bool EhciController::configureBootInterfaces(uint8_t address,
        uint8_t packetSize0) {
    uint8_t configBuffer[CONTROL_BUFFER_SIZE];
    BootInterface interfaces[4];
    size_t interfaceCount = 0;

    if (!getDescriptor(address, packetSize0, USB_DESCRIPTOR_CONFIGURATION, 0,
            configBuffer, sizeof(UsbConfigurationDescriptor))) {
        return false;
    }

    UsbConfigurationDescriptor* config =
            (UsbConfigurationDescriptor*) configBuffer;
    if (config->length < sizeof(UsbConfigurationDescriptor)) {
        return false;
    }

    size_t totalLength = config->totalLength;
    if (totalLength > sizeof(configBuffer)) {
        Log::printf("EHCI device %u ignored: configuration descriptor too "
                "large\n", address);
        return false;
    }

    if (!getDescriptor(address, packetSize0, USB_DESCRIPTOR_CONFIGURATION, 0,
            configBuffer, totalLength)) {
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
                interfaces[interfaceCount].interfaceNumber =
                        currentInterface->interfaceNumber;
                interfaces[interfaceCount].protocol =
                        currentInterface->interfaceProtocol;
                interfaces[interfaceCount].endpointAddress =
                        endpoint->endpointAddress;
                interfaces[interfaceCount].maxPacketSize =
                        endpoint->maxPacketSize & 0x7FF;
                interfaceCount++;
                currentInterface = nullptr;
            }
        }

        offset += length;
    }

    if (interfaceCount == 0) return false;
    if (!setConfiguration(address, packetSize0, config->configurationValue)) {
        return false;
    }

    for (size_t i = 0; i < interfaceCount; i++) {
        if (!setBootProtocol(address, packetSize0,
                interfaces[i].interfaceNumber)) {
            continue;
        }

        void* virt = nullptr;
        EhciInterruptDevice* device = xnew EhciInterruptDevice;
        memset(device, 0, sizeof(*device));
        device->next = nullptr;
        device->controller = this;
        device->address = address;
        device->controlPacketSize = packetSize0;
        device->interfaceNumber = interfaces[i].interfaceNumber;
        device->endpointAddress = interfaces[i].endpointAddress;
        device->maxPacketSize = interfaces[i].maxPacketSize;
        device->protocol = interfaces[i].protocol;
        device->toggle = 0;

        allocateDma(sizeof(EhciQh), 32, virt, device->qhPhys);
        device->qh = (EhciQh*) virt;
        allocateDma(sizeof(EhciQtd), 32, virt, device->qtdPhys);
        device->qtd = (EhciQtd*) virt;
        allocateDma(device->maxPacketSize, 32, virt, device->reportBufferPhys);
        device->reportBuffer = (uint8_t*) virt;

        memset(device->qh, 0, sizeof(EhciQh));
        memset(device->qtd, 0, sizeof(EhciQtd));
        memset(device->reportBuffer, 0, device->maxPacketSize);

        device->qh->horizLink = EHCI_QH_LINK_TERMINATE;
        device->qh->epChars =
                (address << EHCI_QH_EPCHAR_DEVADDR_SHIFT) |
                ((device->endpointAddress & 0xF) <<
                EHCI_QH_EPCHAR_ENDPOINT_SHIFT) |
                (EHCI_EPS_HIGHSPEED << EHCI_QH_EPCHAR_EPS_SHIFT) |
                EHCI_QH_EPCHAR_DTC |
                ((uint32_t) device->maxPacketSize << EHCI_QH_EPCHAR_MPS_SHIFT);
        device->qh->epCaps =
                (1U << EHCI_QH_EPCAPS_SMASK_SHIFT) |
                (1U << EHCI_QH_EPCAPS_MULT_SHIFT);
        device->qh->nextQtd = EHCI_QH_LINK_TERMINATE;
        device->qh->altNextQtd = EHCI_QH_LINK_TERMINATE;

        if (device->protocol == USB_PROTOCOL_KEYBOARD) {
            Log::printf("USB keyboard found on EHCI port\n");
        } else if (device->protocol == USB_PROTOCOL_MOUSE) {
            Log::printf("USB mouse found on EHCI port\n");
        }

        registerInterruptDevice(device);
    }

    return true;
}

bool EhciController::controlTransfer(uint8_t address, uint8_t packetSize0,
        uint8_t requestType, uint8_t request, uint16_t value, uint16_t index,
        void* data, size_t length) {
    if (!running || length > CONTROL_BUFFER_SIZE || packetSize0 == 0) {
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

    controlSetup->requestType = requestType;
    controlSetup->request = request;
    controlSetup->value = value;
    controlSetup->index = index;
    controlSetup->length = length;

    size_t qtdCount = 0;
    controlQtds[qtdCount].nextQtd = EHCI_QH_LINK_TERMINATE;
    controlQtds[qtdCount].altNextQtd = EHCI_QH_LINK_TERMINATE;
    controlQtds[qtdCount].token = EHCI_QTD_STATUS_ACTIVE |
            (3U << EHCI_QTD_CERR_SHIFT) |
            EHCI_QTD_PID_SETUP |
            (sizeof(UsbSetupPacket) << EHCI_QTD_BYTES_SHIFT);
    fillQtdBuffers(&controlQtds[qtdCount], controlSetupPhys,
            sizeof(UsbSetupPacket));
    qtdCount++;

    bool toggle = true;
    if (length > 0) {
        size_t transferred = 0;
        while (transferred < length) {
            size_t packetLength = length - transferred;
            if (packetLength > packetSize0) packetLength = packetSize0;

            controlQtds[qtdCount].nextQtd = EHCI_QH_LINK_TERMINATE;
            controlQtds[qtdCount].altNextQtd = EHCI_QH_LINK_TERMINATE;
            controlQtds[qtdCount].token = EHCI_QTD_STATUS_ACTIVE |
                    (3U << EHCI_QTD_CERR_SHIFT) |
                    ((uint32_t) packetLength << EHCI_QTD_BYTES_SHIFT) |
                    ((requestType & USB_DIR_IN) ? EHCI_QTD_PID_IN :
                    EHCI_QTD_PID_OUT) |
                    (toggle ? EHCI_QTD_DT : 0);
            fillQtdBuffers(&controlQtds[qtdCount], controlBufferPhys + transferred,
                    packetLength);
            qtdCount++;

            transferred += packetLength;
            toggle = !toggle;
            if (qtdCount >= CONTROL_QTD_COUNT - 1) return false;
        }
    }

    controlQtds[qtdCount].nextQtd = EHCI_QH_LINK_TERMINATE;
    controlQtds[qtdCount].altNextQtd = EHCI_QH_LINK_TERMINATE;
    controlQtds[qtdCount].token = EHCI_QTD_STATUS_ACTIVE |
            (3U << EHCI_QTD_CERR_SHIFT) |
            EHCI_QTD_IOC |
            ((requestType & USB_DIR_IN) ? EHCI_QTD_PID_OUT : EHCI_QTD_PID_IN) |
            EHCI_QTD_DT;
    qtdCount++;

    for (size_t i = 0; i < qtdCount; i++) {
        if (i + 1 < qtdCount) {
            controlQtds[i].nextQtd = makeQtdLink(controlQtdsPhys +
                    (i + 1) * sizeof(EhciQtd));
        }
    }

    controlQh->epChars =
            (address << EHCI_QH_EPCHAR_DEVADDR_SHIFT) |
            (0U << EHCI_QH_EPCHAR_ENDPOINT_SHIFT) |
            (EHCI_EPS_HIGHSPEED << EHCI_QH_EPCHAR_EPS_SHIFT) |
            EHCI_QH_EPCHAR_DTC |
            ((uint32_t) packetSize0 << EHCI_QH_EPCHAR_MPS_SHIFT);
    controlQh->epCaps = 0;
    controlQh->nextQtd = makeQtdLink(controlQtdsPhys);
    controlQh->altNextQtd = EHCI_QH_LINK_TERMINATE;
    controlQh->token = 0;

    Clock* clock = Clock::get(CLOCK_MONOTONIC);
    struct timespec now;
    struct timespec timeout;
    timeout.tv_sec = 0;
    timeout.tv_nsec = 100000000L;
    if (!clock || clock->getTime(&now) < 0) {
        controlQh->nextQtd = EHCI_QH_LINK_TERMINATE;
        controlQh->altNextQtd = EHCI_QH_LINK_TERMINATE;
        return false;
    }
    struct timespec deadline = timespecPlus(now, timeout);

    while (true) {
        bool active = false;
        bool failed = false;

        for (size_t i = 0; i < qtdCount; i++) {
            uint32_t token = controlQtds[i].token;
            if (token & EHCI_QTD_STATUS_ACTIVE) {
                active = true;
            } else if (token & (EHCI_QTD_STATUS_HALTED |
                    EHCI_QTD_STATUS_DBUFERR | EHCI_QTD_STATUS_BABBLE |
                    EHCI_QTD_STATUS_XACTERR)) {
                failed = true;
                break;
            }
        }

        if (!active || failed) break;

        clock->getTime(&now);
        if (!timespecLess(now, deadline)) {
            controlQh->nextQtd = EHCI_QH_LINK_TERMINATE;
            controlQh->altNextQtd = EHCI_QH_LINK_TERMINATE;
            return false;
        }

        sched_yield();
    }

    controlQh->nextQtd = EHCI_QH_LINK_TERMINATE;
    controlQh->altNextQtd = EHCI_QH_LINK_TERMINATE;

    for (size_t i = 0; i < qtdCount; i++) {
        uint32_t token = controlQtds[i].token;
        if (token & EHCI_QTD_STATUS_ACTIVE) return false;
        if (token & (EHCI_QTD_STATUS_HALTED | EHCI_QTD_STATUS_DBUFERR |
                EHCI_QTD_STATUS_BABBLE | EHCI_QTD_STATUS_XACTERR)) {
            return false;
        }
    }

    if ((requestType & USB_DIR_IN) && length > 0) {
        memcpy(data, controlBuffer, length);
    }

    return true;
}

bool EhciController::enumeratePort(unsigned int port) {
    if (!resetPort(port)) return false;

    UsbDeviceDescriptor descriptor = {};
    uint8_t packetSize0 = 64;

    if (!getDescriptor(0, packetSize0, USB_DESCRIPTOR_DEVICE, 0,
            &descriptor, 8)) {
        return false;
    }

    packetSize0 = descriptor.maxPacketSize0;
    if (!getDescriptor(0, packetSize0, USB_DESCRIPTOR_DEVICE, 0,
            &descriptor, sizeof(descriptor))) {
        return false;
    }

    uint8_t address = nextAddress++;
    if (!setAddress(0, packetSize0, address)) {
        return false;
    }

    sleepMilliseconds(10);
    if (!configureBootInterfaces(address, packetSize0)) {
        return false;
    }

    return true;
}

bool EhciController::getDescriptor(uint8_t address, uint8_t packetSize0,
        uint8_t type, uint8_t index, void* data, size_t length) {
    return controlTransfer(address, packetSize0,
            USB_DIR_IN | USB_TYPE_STANDARD | USB_RECIP_DEVICE,
            USB_REQUEST_GET_DESCRIPTOR, (type << 8) | index, 0, data, length);
}

void EhciController::handleKeyboardReport(EhciInterruptDevice* device,
        const uint8_t* report, size_t length) {
    static const struct timespec repeatDelay = {
        USB_KEYBOARD_REPEAT_DELAY_MS / 1000,
        (USB_KEYBOARD_REPEAT_DELAY_MS % 1000) * 1000000L
    };
    KeyboardListener* listener = console ? console.operator->() : nullptr;
    uint8_t current[8] = {};
    size_t copyLength = length < sizeof(current) ? length : sizeof(current);
    memcpy(current, report, copyLength);

    for (size_t i = 0; i < 8; i++) {
        bool oldPressed = device->oldReport[0] & (1 << i);
        bool newPressed = current[0] & (1 << i);
        if (oldPressed == newPressed) continue;

        int key = modifierBitToKey(i);
        if (listener && key) {
            listener->onKeyboardEvent(newPressed ? key : -key);
        }
    }

    for (size_t i = 2; i < 8; i++) {
        uint8_t usage = device->oldReport[i];
        if (usage <= 3 || containsUsage(current + 2, usage)) continue;

        int key = hidUsageToKey(usage);
        if (listener && key) listener->onKeyboardEvent(-key);
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

        if (listener) listener->onKeyboardEvent(key);
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

static void handleKeyboardRepeat(EhciInterruptDevice* device) {
    static const struct timespec repeatInterval = {
        USB_KEYBOARD_REPEAT_INTERVAL_MS / 1000,
        (USB_KEYBOARD_REPEAT_INTERVAL_MS % 1000) * 1000000L
    };
    KeyboardListener* listener = console ? console.operator->() : nullptr;
    if (!device->repeatUsage || !listener) return;

    struct timespec now;
    if (!getMonotonicTime(now) || timespecLess(now, device->repeatDeadline)) {
        return;
    }

    int key = hidUsageToKey(device->repeatUsage);
    if (!isRepeatableKey(key)) {
        device->repeatUsage = 0;
        return;
    }

    listener->onKeyboardEvent(key);
    do {
        device->repeatDeadline = timespecPlus(device->repeatDeadline,
                repeatInterval);
    } while (!timespecLess(now, device->repeatDeadline));
}

void EhciController::handleMouseReport(const uint8_t* report, size_t length) {
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

void EhciController::initializeController() {
    if (irq >= 0) {
        irqHandler.func = onEhciIrq;
        irqHandler.user = this;
        Interrupts::addIrqHandler(irq, &irqHandler);
    }

    writeRegister(EHCI_USBCMD, 0);
    writeRegister(EHCI_USBINTR, 0);
    sleepMilliseconds(1);

    writeRegister(EHCI_USBCMD, EHCI_USBCMD_RESET);
    sleepMilliseconds(50);
    while (readRegister(EHCI_USBCMD) & EHCI_USBCMD_RESET) {
        sched_yield();
    }

    writeRegister(EHCI_USBSTS, readRegister(EHCI_USBSTS));
    writeRegister(EHCI_CTRLDSSEGMENT, 0);
    writeRegister(EHCI_PERIODICLISTBASE, frameListPhys);
    writeRegister(EHCI_ASYNCLISTADDR, asyncHeadQhPhys);
    writeRegister(EHCI_FRINDEX, 0);
    writeRegister(EHCI_CONFIGFLAG, 1);

    for (unsigned int port = 0; port < numPorts; port++) {
        uint32_t status = readPortStatus(port);
        writeRegister(EHCI_PORTSC_BASE + port * EHCI_PORTSC_STRIDE,
                status | EHCI_PORTSC_PP);
    }

    writeRegister(EHCI_USBCMD, EHCI_USBCMD_RUN | EHCI_USBCMD_PSE |
            EHCI_USBCMD_ASE);
    writeRegister(EHCI_USBINTR, irq >= 0 ? EHCI_USBINTR_MASK : 0);
    sleepMilliseconds(10);

    if (readRegister(EHCI_USBSTS) & EHCI_USBSTS_HCHALTED) {
        Log::printf("EHCI controller at 0x%X failed to start\n",
                (unsigned int) mmioBase);
        return;
    }

    running = true;
}

void EhciController::onIrq() {
    if (!running || irq < 0) return;

    uint32_t status = readRegister(EHCI_USBSTS);
    uint32_t handled = status & EHCI_USBINTR_MASK;
    if (!handled) return;

    writeRegister(EHCI_USBSTS, handled);

    if (status & EHCI_USBSTS_FATAL) {
        Log::printf("EHCI controller error 0x%X\n", status);
    }

    irqWorkPending = true;
    if (!irqJobQueued) {
        irqJobQueued = true;
        WorkerThread::addJob(&irqJob);
    }
}

void EhciController::processIrqWork() {
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

void EhciController::processInterruptDevices() {
    for (EhciInterruptDevice* device = interruptDevices; device;
            device = device->next) {
        uint8_t report[64];
        size_t length = 0;
        bool haveReport = false;

        {
            AutoLock lock(&mutex);

            uint32_t token = device->qtd->token;
            if (token & EHCI_QTD_STATUS_ACTIVE) continue;

            if (token & (EHCI_QTD_STATUS_HALTED | EHCI_QTD_STATUS_DBUFERR |
                    EHCI_QTD_STATUS_BABBLE | EHCI_QTD_STATUS_XACTERR)) {
                armInterruptTransfer(device, false);
                continue;
            }

            length = qtdActualLength(device->maxPacketSize, token);
            if (length > device->maxPacketSize) {
                length = device->maxPacketSize;
            }
            if (length > sizeof(report)) {
                length = sizeof(report);
            }

            memcpy(report, device->reportBuffer, length);
            armInterruptTransfer(device, true);
            haveReport = true;
        }

        if (haveReport) {
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

uint32_t EhciController::readRegister(size_t offset) {
    return *(volatile uint32_t*) (mmioBase + operationalBase + offset);
}

uint32_t EhciController::readPortStatus(unsigned int port) {
    return readRegister(EHCI_PORTSC_BASE + port * EHCI_PORTSC_STRIDE);
}

void EhciController::registerInterruptDevice(EhciInterruptDevice* device) {
    AutoLock lock(&mutex);
    device->next = interruptDevices;
    interruptDevices = device;
    device->qh->horizLink = periodicHeadQh->horizLink;
    periodicHeadQh->horizLink = makeQhLink(device->qhPhys);
    armInterruptTransfer(device, false);
}

bool EhciController::resetPort(unsigned int port) {
    uint32_t status = readPortStatus(port);
    if (!(status & EHCI_PORTSC_CCS)) return false;

    status &= ~(EHCI_PORTSC_CSC | EHCI_PORTSC_PEC | EHCI_PORTSC_OCC);
    status |= EHCI_PORTSC_PP;
    writeRegister(EHCI_PORTSC_BASE + port * EHCI_PORTSC_STRIDE, status);

    status &= ~EHCI_PORTSC_OWNER;
    status |= EHCI_PORTSC_PR;
    writeRegister(EHCI_PORTSC_BASE + port * EHCI_PORTSC_STRIDE, status);
    sleepMilliseconds(50);

    status &= ~EHCI_PORTSC_PR;
    writeRegister(EHCI_PORTSC_BASE + port * EHCI_PORTSC_STRIDE, status);
    sleepMilliseconds(10);

    status = readPortStatus(port);
    writeRegister(EHCI_PORTSC_BASE + port * EHCI_PORTSC_STRIDE,
            status | EHCI_PORTSC_CSC | EHCI_PORTSC_PEC | EHCI_PORTSC_OCC);

    status = readPortStatus(port);
    if (!(status & EHCI_PORTSC_CCS) || !(status & EHCI_PORTSC_PE) ||
            (status & EHCI_PORTSC_OWNER)) {
        return false;
    }

    return true;
}

bool EhciController::setAddress(uint8_t oldAddress, uint8_t packetSize0,
        uint8_t newAddress) {
    uint8_t dummy = 0;
    return controlTransfer(oldAddress, packetSize0,
            USB_TYPE_STANDARD | USB_RECIP_DEVICE, USB_REQUEST_SET_ADDRESS,
            newAddress, 0, &dummy, 0);
}

bool EhciController::setBootProtocol(uint8_t address, uint8_t packetSize0,
        uint8_t interfaceNumber) {
    uint8_t dummy = 0;
    return controlTransfer(address, packetSize0,
            USB_TYPE_CLASS | USB_RECIP_INTERFACE, HID_REQUEST_SET_PROTOCOL, 0,
            interfaceNumber, &dummy, 0);
}

bool EhciController::setConfiguration(uint8_t address, uint8_t packetSize0,
        uint8_t configurationValue) {
    uint8_t dummy = 0;
    return controlTransfer(address, packetSize0,
            USB_TYPE_STANDARD | USB_RECIP_DEVICE,
            USB_REQUEST_SET_CONFIGURATION, configurationValue, 0, &dummy, 0);
}

bool EhciController::setKeyboardLeds(EhciInterruptDevice* device, uint8_t leds) {
    return controlTransfer(device->address, device->controlPacketSize,
            USB_TYPE_CLASS | USB_RECIP_INTERFACE, HID_REQUEST_SET_REPORT,
            HID_REPORT_TYPE_OUTPUT << 8, device->interfaceNumber, &leds,
            sizeof(leds));
}

void EhciController::scanPorts() {
    if (!running) return;

    for (unsigned int port = 0; port < numPorts; port++) {
        uint32_t status = readPortStatus(port);
        bool connected = status & EHCI_PORTSC_CCS;

        if (!connected) {
            portInitialized[port] = false;
            portProbed[port] = false;
            continue;
        }

        if (portProbed[port]) continue;

        portProbed[port] = true;
        if (enumeratePort(port)) {
            portInitialized[port] = true;
            Log::printf("EHCI port %u enumerated\n", port + 1);
        } else {
            Log::printf("EHCI port %u probe failed, waiting for disconnect "
                    "before retry\n", port + 1);
        }
    }
}

void EhciController::writeRegister(size_t offset, uint32_t value) {
    *(volatile uint32_t*) (mmioBase + operationalBase + offset) = value;
}

void Ehci::initialize(uint8_t bus, uint8_t device, uint8_t function) {
    uint32_t command = Pci::readConfig(bus, device, function,
            offsetof(PciHeader, command));
    command |= PCI_COMMAND_MEMSPACE | PCI_COMMAND_BUSMASTER;
    Pci::writeConfig(bus, device, function, offsetof(PciHeader, command),
            command);

    uint32_t bar0 = Pci::readConfig(bus, device, function,
            offsetof(PciHeader, bar0));
    if (bar0 & 0x1) {
        Log::printf("EHCI controller unsupported: BAR0 is not a memory BAR\n");
        return;
    }

    paddr_t baseAddress = bar0 & ~0xFULL;
    vaddr_t mmioBase = kernelSpace->mapPhysical(baseAddress, PAGESIZE,
            PROT_READ | PROT_WRITE);
    if (!mmioBase) PANIC("Failed to map EHCI registers");

    uint32_t hccParams = *(volatile uint32_t*) (mmioBase + 0x08);
    uint8_t eecp = (hccParams >> 8) & 0xFF;
    if (eecp) {
        uint32_t legacy = Pci::readConfig(bus, device, function, eecp);
        if (legacy & (1U << 16)) {
            Pci::writeConfig(bus, device, function, eecp, legacy | (1U << 24));
            for (size_t i = 0; i < 100; i++) {
                legacy = Pci::readConfig(bus, device, function, eecp);
                if (!(legacy & (1U << 16))) break;
                sleepMilliseconds(1);
            }
        }
        Pci::writeConfig(bus, device, function, eecp + 4, 0);
    }

    int irq = Pci::getIrq(bus, device, function);
    if (irq < 0) {
        irq = Pci::readConfig(bus, device, function,
                offsetof(PciHeader, interruptLine)) & 0xFF;
        if (irq == 0 || irq == 0xFF) irq = -1;
    }
    if (irq < 0) {
        Log::printf("EHCI controller at 0x%X is using polling mode\n",
                (unsigned int) baseAddress);
    }
    xnew EhciController(mmioBase, irq);
}
