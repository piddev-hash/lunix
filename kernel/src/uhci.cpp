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

/* kernel/src/uhci.cpp
 * Minimal UHCI USB host controller with HID boot keyboard/mouse support.
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
#include <lunix/kernel/portio.h>
#include <lunix/kernel/thread.h>
#include <lunix/kernel/uhci.h>
#include <lunix/kernel/worker.h>

#define USBCMD 0
#define USBCMD_RS (1 << 0)
#define USBCMD_HCRESET (1 << 1)
#define USBCMD_CF (1 << 6)
#define USBCMD_MAXP (1 << 7)

#define USBSTS 2
#define USBSTS_USBINT (1 << 0)
#define USBSTS_ERROR (1 << 1)
#define USBSTS_HSE (1 << 3)
#define USBSTS_HCPE (1 << 4)
#define USBSTS_HCH (1 << 5)

#define USBINTR 4
#define USBINTR_TIMEOUT (1 << 0)
#define USBINTR_IOC (1 << 2)

#define USBFRNUM 6
#define USBFLBASEADD 8
#define USBSOF 12
#define USBSOF_DEFAULT 64

#define USBPORTSC1 16
#define USBPORTSC_CCS (1 << 0)
#define USBPORTSC_CSC (1 << 1)
#define USBPORTSC_PE (1 << 2)
#define USBPORTSC_PEC (1 << 3)
#define USBPORTSC_RD (1 << 6)
#define USBPORTSC_RES1 (1 << 7)
#define USBPORTSC_LSDA (1 << 8)
#define USBPORTSC_PR (1 << 9)
#define USBPORTSC_OCC (1 << 11)

#define UHCI_PORT_SCAN_INTERVAL_YIELDS 2048
#define USBPORTSC_SUSP (1 << 12)
#define USBPORTSC_RES2 (1 << 13)
#define USBPORTSC_RES3 (1 << 14)
#define USBPORTSC_RES4 (1 << 15)

#define UHCI_LINK_TERM 0x00000001U
#define UHCI_LINK_QH 0x00000002U
#define UHCI_LINK_DEPTH 0x00000004U

#define TD_CTRL_SPD (1U << 29)
#define TD_CTRL_C_ERR_SHIFT 27
#define TD_CTRL_LS (1U << 26)
#define TD_CTRL_IOC (1U << 24)
#define TD_CTRL_ACTIVE (1U << 23)
#define TD_CTRL_STALLED (1U << 22)
#define TD_CTRL_DBUFERR (1U << 21)
#define TD_CTRL_BABBLE (1U << 20)
#define TD_CTRL_NAK (1U << 19)
#define TD_CTRL_CRCTIMEO (1U << 18)
#define TD_CTRL_BITSTUFF (1U << 17)
#define TD_CTRL_ACTLEN_MASK 0x7FF

#define TD_TOKEN_DEVADDR_SHIFT 8
#define TD_TOKEN_ENDPOINT_SHIFT 15
#define TD_TOKEN_TOGGLE_SHIFT 19
#define TD_TOKEN_EXPLEN_SHIFT 21
#define TD_TOKEN_EXPLEN_MASK 0x7FF

#define USB_PID_OUT 0xE1
#define USB_PID_IN 0x69
#define USB_PID_SETUP 0x2D

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

#define CONTROL_BUFFER_SIZE 512
#define CONTROL_TD_COUNT 16
#define UHCI_PORTS 2
#define UHCI_FAST_POLL_INTERVAL_NS 1000000L
#define UHCI_IDLE_POLL_INTERVAL_NS 10000000L
#define UHCI_PORT_SCAN_INTERVAL_TICKS 10

#define PORT_RWC_BITS (USBPORTSC_OCC | USBPORTSC_PEC | USBPORTSC_CSC)
#define PORT_WZ_BITS (USBPORTSC_RES2 | USBPORTSC_RES3 | USBPORTSC_RES4)

#define USBLEGSUP 0xC0
#define USBLEGSUP_DEFAULT 0x2000
#define USBLEGSUP_RWC 0x8F00
#define USBRES_INTEL 0xC4

#define PCI_VENDOR_INTEL 0x8086

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

struct PACKED UhciQh {
    volatile uint32_t link;
    volatile uint32_t element;
} ALIGNED(16);

struct PACKED UhciTd {
    volatile uint32_t link;
    volatile uint32_t status;
    volatile uint32_t token;
    volatile uint32_t buffer;
} ALIGNED(16);

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

class UhciController;

class UsbHidDevice {
public:
    UsbHidDevice(UhciController* controller, uint8_t address,
            uint8_t controlPacketSize, uint8_t interfaceNumber,
            uint8_t endpointAddress, uint16_t maxPacketSize, bool lowSpeed);
    virtual ~UsbHidDevice() = default;
    NOT_COPYABLE(UsbHidDevice);
    NOT_MOVABLE(UsbHidDevice);

    virtual void handleReport(const uint8_t* report, size_t length) = 0;
protected:
    UhciController* controller;
    uint8_t address;
    uint8_t controlPacketSize;
    uint8_t interfaceNumber;
    uint8_t endpointAddress;
    uint16_t maxPacketSize;
    bool lowSpeed;
private:
    friend class UhciController;
    UsbHidDevice* next;
    UhciQh* qh;
    paddr_t qhPhys;
    UhciTd* td;
    paddr_t tdPhys;
    uint8_t* reportBuffer;
    paddr_t reportBufferPhys;
    uint8_t toggle;
};

class UsbKeyboard : public UsbHidDevice {
public:
    UsbKeyboard(UhciController* controller, uint8_t address,
            uint8_t controlPacketSize, uint8_t interfaceNumber,
            uint8_t endpointAddress, uint16_t maxPacketSize, bool lowSpeed);
    void handleReport(const uint8_t* report, size_t length) override;
private:
    void emitKeyEvent(int key);
    void updateLeds();
private:
    KeyboardListener* listener;
    uint8_t oldReport[8];
    uint8_t ledState;
};

class UsbMouse : public UsbHidDevice {
public:
    UsbMouse(UhciController* controller, uint8_t address,
            uint8_t controlPacketSize, uint8_t interfaceNumber,
            uint8_t endpointAddress, uint16_t maxPacketSize, bool lowSpeed);
    void handleReport(const uint8_t* report, size_t length) override;
};

class UhciController {
public:
    UhciController(uint16_t ioBase, int irq);
    ~UhciController() = default;
    NOT_COPYABLE(UhciController);
    NOT_MOVABLE(UhciController);

    bool controlTransfer(uint8_t address, bool lowSpeed, uint8_t packetSize0,
            uint8_t requestType, uint8_t request, uint16_t value,
            uint16_t index, void* data, size_t length);
    bool setBootProtocol(uint8_t address, bool lowSpeed, uint8_t packetSize0,
            uint8_t interfaceNumber);
    bool setKeyboardLeds(uint8_t address, bool lowSpeed, uint8_t packetSize0,
            uint8_t interfaceNumber, uint8_t leds);
    void registerInterruptDevice(UsbHidDevice* device);
    void onIrq();
    void processInterruptDevices();
    void processIrqWork();
    bool needsPolling() const { return true; }
private:
    friend class UsbHidDevice;
    void acknowledgePortChanges(unsigned int port);
    void allocateDma(size_t size, size_t alignment, void*& virtualAddress,
            paddr_t& physicalAddress);
    bool armInterruptTransfer(UsbHidDevice* device, bool advanceToggle);
    bool configureBootInterfaces(uint8_t address, bool lowSpeed,
            uint8_t packetSize0);
    bool enumeratePort(unsigned int port);
    bool getDescriptor(uint8_t address, bool lowSpeed, uint8_t packetSize0,
            uint8_t type, uint8_t index, void* data, size_t length);
    void initializeController(int irq);
    uint16_t readRegister(uint16_t reg);
    uint16_t readPortStatus(unsigned int port);
    bool resetPort(unsigned int port, bool& lowSpeed);
    bool setAddress(uint8_t oldAddress, bool lowSpeed, uint8_t packetSize0,
            uint8_t newAddress);
    bool setConfiguration(uint8_t address, bool lowSpeed, uint8_t packetSize0,
            uint8_t configurationValue);
    void setPortBits(unsigned int port, uint16_t bits);
    void clearPortBits(unsigned int port, uint16_t bits);
    void writeRegister(uint16_t reg, uint16_t value);
    void writeRegister32(uint16_t reg, uint32_t value);
private:
    uint16_t ioBase;
    int irq;
    kthread_mutex_t mutex;
    DmaPage* dmaPages;
    uint32_t* frameList;
    paddr_t frameListPhys;
    UhciQh* rootQh;
    paddr_t rootQhPhys;
    UhciQh* controlQh;
    paddr_t controlQhPhys;
    UhciTd* controlTds;
    paddr_t controlTdsPhys;
    UsbSetupPacket* controlSetup;
    paddr_t controlSetupPhys;
    uint8_t* controlBuffer;
    paddr_t controlBufferPhys;
    UsbHidDevice* interruptDevices;
    uint8_t nextAddress;
    bool portInitialized[UHCI_PORTS];
    bool running;
    IrqHandler irqHandler;
    WorkerJob irqJob;
    bool irqJobQueued;
    bool irqWorkPending;
public:
    void scanPorts();
    UhciController* nextController;
};

static UhciController* firstController;
static bool uhciPollThreadStarted;

static void uhciIrqWork(void* context) {
    ((UhciController*) context)->processIrqWork();
}

static NORETURN void uhciPollLoop() {
    unsigned int scanCountdown = 0;

    while (true) {
        bool rescanPorts = scanCountdown == 0;
        for (UhciController* controller = firstController; controller;
                controller = controller->nextController) {
            if (rescanPorts) {
                controller->scanPorts();
            }
            controller->processInterruptDevices();
        }

        if (rescanPorts) {
            scanCountdown = UHCI_PORT_SCAN_INTERVAL_YIELDS - 1;
        } else {
            scanCountdown--;
        }

        sched_yield();
    }
}

static void startUhciPollThread(void*) {
    uhciPollLoop();
}

static void ensureUhciPollThread() {
    if (uhciPollThreadStarted) return;
    uhciPollThreadStarted = true;
    Thread::createKernelThread(startUhciPollThread, nullptr);
}

static void onUhciIrq(void* user, const InterruptContext* /*context*/) {
    UhciController* controller = (UhciController*) user;
    controller->onIrq();
}

static size_t alignUp(size_t value, size_t alignment) {
    return (value + alignment - 1) & ~(alignment - 1);
}

static bool containsUsage(const uint8_t* usages, uint8_t usage) {
    for (size_t i = 0; i < 6; i++) {
        if (usages[i] == usage) return true;
    }
    return false;
}

static uint32_t makeQhLink(paddr_t physicalAddress) {
    return (uint32_t) physicalAddress | UHCI_LINK_QH;
}

static uint32_t makeTdLink(paddr_t physicalAddress) {
    return (uint32_t) physicalAddress | UHCI_LINK_DEPTH;
}

static uint32_t encodeLength(size_t length) {
    if (length == 0) {
        return TD_TOKEN_EXPLEN_MASK << TD_TOKEN_EXPLEN_SHIFT;
    }
    return (((uint32_t) length - 1) & TD_TOKEN_EXPLEN_MASK) <<
            TD_TOKEN_EXPLEN_SHIFT;
}

static uint32_t makeToken(uint8_t pid, uint8_t address, uint8_t endpoint,
        size_t length, bool toggle) {
    return pid | ((uint32_t) address << TD_TOKEN_DEVADDR_SHIFT) |
            ((uint32_t) endpoint << TD_TOKEN_ENDPOINT_SHIFT) |
            ((uint32_t) toggle << TD_TOKEN_TOGGLE_SHIFT) |
            encodeLength(length);
}

static size_t actualLength(uint32_t status) {
    return (status + 1) & TD_CTRL_ACTLEN_MASK;
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

UsbHidDevice::UsbHidDevice(UhciController* controller, uint8_t address,
        uint8_t controlPacketSize, uint8_t interfaceNumber,
        uint8_t endpointAddress, uint16_t maxPacketSize, bool lowSpeed)
        : controller(controller), address(address),
        controlPacketSize(controlPacketSize), interfaceNumber(interfaceNumber),
        endpointAddress(endpointAddress), maxPacketSize(maxPacketSize),
        lowSpeed(lowSpeed) {
    void* virt = nullptr;

    next = nullptr;
    toggle = 0;

    controller->allocateDma(sizeof(UhciQh), 16, virt, qhPhys);
    qh = (UhciQh*) virt;
    controller->allocateDma(sizeof(UhciTd), 16, virt, tdPhys);
    td = (UhciTd*) virt;
    controller->allocateDma(maxPacketSize, 16, virt, reportBufferPhys);
    reportBuffer = (uint8_t*) virt;

    qh->link = UHCI_LINK_TERM;
    qh->element = UHCI_LINK_TERM;
    td->link = UHCI_LINK_TERM;
    td->status = 0;
    td->token = 0;
    td->buffer = 0;
}

UsbKeyboard::UsbKeyboard(UhciController* controller, uint8_t address,
        uint8_t controlPacketSize, uint8_t interfaceNumber,
        uint8_t endpointAddress, uint16_t maxPacketSize, bool lowSpeed)
        : UsbHidDevice(controller, address, controlPacketSize, interfaceNumber,
        endpointAddress, maxPacketSize, lowSpeed) {
    listener = console ? console.operator->() : nullptr;
    memset(oldReport, 0, sizeof(oldReport));
    ledState = 0;
}

void UsbKeyboard::emitKeyEvent(int key) {
    if (key == 0) return;

    if (key > 0) {
        if (key == KB_NUMLOCK) {
            ledState ^= 0x01;
            updateLeds();
        } else if (key == KB_CAPSLOCK) {
            ledState ^= 0x02;
            updateLeds();
        } else if (key == KB_SCROLLLOCK) {
            ledState ^= 0x04;
            updateLeds();
        }
    }

    if (listener) {
        listener->onKeyboardEvent(key);
    }
}

void UsbKeyboard::handleReport(const uint8_t* report, size_t length) {
    uint8_t current[8] = {};
    size_t copyLength = length < sizeof(current) ? length : sizeof(current);
    memcpy(current, report, copyLength);

    for (size_t i = 0; i < 8; i++) {
        bool oldPressed = oldReport[0] & (1 << i);
        bool newPressed = current[0] & (1 << i);
        if (oldPressed == newPressed) continue;

        int key = modifierBitToKey(i);
        emitKeyEvent(newPressed ? key : -key);
    }

    for (size_t i = 2; i < 8; i++) {
        uint8_t usage = oldReport[i];
        if (usage <= 3 || containsUsage(current + 2, usage)) continue;

        int key = hidUsageToKey(usage);
        if (key) emitKeyEvent(-key);
    }

    for (size_t i = 2; i < 8; i++) {
        uint8_t usage = current[i];
        if (usage <= 3 || containsUsage(oldReport + 2, usage)) continue;

        int key = hidUsageToKey(usage);
        if (key) emitKeyEvent(key);
    }

    memcpy(oldReport, current, sizeof(oldReport));
}

void UsbKeyboard::updateLeds() {
    controller->setKeyboardLeds(address, lowSpeed, controlPacketSize,
            interfaceNumber, ledState);
}

UsbMouse::UsbMouse(UhciController* controller, uint8_t address,
        uint8_t controlPacketSize, uint8_t interfaceNumber,
        uint8_t endpointAddress, uint16_t maxPacketSize, bool lowSpeed)
        : UsbHidDevice(controller, address, controlPacketSize, interfaceNumber,
        endpointAddress, maxPacketSize, lowSpeed) {
}

void UsbMouse::handleReport(const uint8_t* report, size_t length) {
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

UhciController::UhciController(uint16_t ioBase, int irq)
        : ioBase(ioBase), irq(irq) {
    mutex = KTHREAD_MUTEX_INITIALIZER;
    dmaPages = nullptr;
    frameList = nullptr;
    frameListPhys = 0;
    rootQh = nullptr;
    rootQhPhys = 0;
    controlQh = nullptr;
    controlQhPhys = 0;
    controlTds = nullptr;
    controlTdsPhys = 0;
    controlSetup = nullptr;
    controlSetupPhys = 0;
    controlBuffer = nullptr;
    controlBufferPhys = 0;
    interruptDevices = nullptr;
    nextAddress = 1;
    memset(portInitialized, 0, sizeof(portInitialized));
    running = false;
    nextController = nullptr;
    irqHandler = {};
    irqJob = {};
    irqJob.func = uhciIrqWork;
    irqJob.context = this;
    irqJobQueued = false;
    irqWorkPending = false;

    void* virt = nullptr;
    allocateDma(PAGESIZE, PAGESIZE, virt, frameListPhys);
    frameList = (uint32_t*) virt;
    allocateDma(sizeof(UhciQh), 16, virt, rootQhPhys);
    rootQh = (UhciQh*) virt;
    allocateDma(sizeof(UhciQh), 16, virt, controlQhPhys);
    controlQh = (UhciQh*) virt;
    allocateDma(CONTROL_TD_COUNT * sizeof(UhciTd), 16, virt, controlTdsPhys);
    controlTds = (UhciTd*) virt;
    allocateDma(sizeof(UsbSetupPacket), 16, virt, controlSetupPhys);
    controlSetup = (UsbSetupPacket*) virt;
    allocateDma(CONTROL_BUFFER_SIZE, 16, virt, controlBufferPhys);
    controlBuffer = (uint8_t*) virt;

    rootQh->link = makeQhLink(controlQhPhys);
    rootQh->element = UHCI_LINK_TERM;
    controlQh->link = UHCI_LINK_TERM;
    controlQh->element = UHCI_LINK_TERM;

    for (size_t i = 0; i < PAGESIZE / sizeof(uint32_t); i++) {
        frameList[i] = makeQhLink(rootQhPhys);
    }

    initializeController(irq);
    if (!running) return;

    Log::printf("UHCI controller initialized at 0x%X\n", ioBase);

    scanPorts();

    nextController = firstController;
    firstController = this;
    ensureUhciPollThread();
}

void UhciController::acknowledgePortChanges(unsigned int port) {
    uint16_t status = readPortStatus(port);
    writeRegister(USBPORTSC1 + port * 2,
            (status & ~PORT_WZ_BITS & ~PORT_RWC_BITS) | (status & PORT_RWC_BITS));
}

void UhciController::allocateDma(size_t size, size_t alignment,
        void*& virtualAddress, paddr_t& physicalAddress) {
    if (size > PAGESIZE || alignment == 0 || (alignment & (alignment - 1))) {
        PANIC("Invalid UHCI DMA allocation request");
    }

    if (!dmaPages || alignUp(dmaPages->offset, alignment) + size > PAGESIZE) {
        DmaPage* page = xnew DmaPage;
        page->next = dmaPages;
        page->offset = 0;
        page->phys = PhysicalMemory::popPageFrame32();
        if (!page->phys) PANIC("Failed to allocate UHCI DMA page");

        page->virt = kernelSpace->mapPhysical(page->phys, PAGESIZE,
                PROT_READ | PROT_WRITE);
        if (!page->virt) PANIC("Failed to map UHCI DMA page");

        memset((void*) page->virt, 0, PAGESIZE);
        dmaPages = page;
    }

    size_t offset = alignUp(dmaPages->offset, alignment);
    virtualAddress = (void*) (dmaPages->virt + offset);
    physicalAddress = dmaPages->phys + offset;
    memset(virtualAddress, 0, size);
    dmaPages->offset = offset + size;
}

bool UhciController::armInterruptTransfer(UsbHidDevice* device,
        bool advanceToggle) {
    if (advanceToggle) {
        device->toggle ^= 1;
    }

    memset(device->reportBuffer, 0, device->maxPacketSize);
    device->td->link = UHCI_LINK_TERM;
    device->td->status = TD_CTRL_ACTIVE | TD_CTRL_IOC | TD_CTRL_SPD |
            (3U << TD_CTRL_C_ERR_SHIFT) |
            (device->lowSpeed ? TD_CTRL_LS : 0);
    device->td->token = makeToken(USB_PID_IN, device->address,
            device->endpointAddress & 0xF, device->maxPacketSize,
            device->toggle);
    device->td->buffer = device->reportBufferPhys;
    device->qh->element = device->tdPhys;
    return true;
}

bool UhciController::configureBootInterfaces(uint8_t address, bool lowSpeed,
        uint8_t packetSize0) {
    uint8_t configBuffer[CONTROL_BUFFER_SIZE];
    BootInterface interfaces[4];
    size_t interfaceCount = 0;

    if (!getDescriptor(address, lowSpeed, packetSize0,
            USB_DESCRIPTOR_CONFIGURATION, 0, configBuffer,
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
        Log::printf("UHCI device %u ignored: configuration descriptor too "
                "large\n", address);
        return false;
    }

    if (!getDescriptor(address, lowSpeed, packetSize0,
            USB_DESCRIPTOR_CONFIGURATION, 0, configBuffer, totalLength)) {
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
    if (!setConfiguration(address, lowSpeed, packetSize0,
            config->configurationValue)) {
        return false;
    }

    for (size_t i = 0; i < interfaceCount; i++) {
        if (!setBootProtocol(address, lowSpeed, packetSize0,
                interfaces[i].interfaceNumber)) {
            continue;
        }

        UsbHidDevice* device = nullptr;
        if (interfaces[i].protocol == USB_PROTOCOL_KEYBOARD) {
            device = xnew UsbKeyboard(this, address, packetSize0,
                    interfaces[i].interfaceNumber,
                    interfaces[i].endpointAddress,
                    interfaces[i].maxPacketSize, lowSpeed);
            Log::printf("USB keyboard found on UHCI port\n");
        } else if (interfaces[i].protocol == USB_PROTOCOL_MOUSE) {
            device = xnew UsbMouse(this, address, packetSize0,
                    interfaces[i].interfaceNumber,
                    interfaces[i].endpointAddress,
                    interfaces[i].maxPacketSize, lowSpeed);
            Log::printf("USB mouse found on UHCI port\n");
        }

        if (device) {
            registerInterruptDevice(device);
        }
    }

    return true;
}

bool UhciController::controlTransfer(uint8_t address, bool lowSpeed,
        uint8_t packetSize0, uint8_t requestType, uint8_t request,
        uint16_t value, uint16_t index, void* data, size_t length) {
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

    size_t tdCount = 0;
    controlTds[tdCount].status = TD_CTRL_ACTIVE |
            (3U << TD_CTRL_C_ERR_SHIFT) | (lowSpeed ? TD_CTRL_LS : 0);
    controlTds[tdCount].token = makeToken(USB_PID_SETUP, address, 0,
            sizeof(UsbSetupPacket), false);
    controlTds[tdCount].buffer = controlSetupPhys;
    tdCount++;

    bool toggle = true;
    if (length > 0) {
        size_t transferred = 0;
        while (transferred < length) {
            size_t packetLength = length - transferred;
            if (packetLength > packetSize0) packetLength = packetSize0;

            controlTds[tdCount].status = TD_CTRL_ACTIVE | TD_CTRL_SPD |
                    (3U << TD_CTRL_C_ERR_SHIFT) |
                    (lowSpeed ? TD_CTRL_LS : 0);
            controlTds[tdCount].token = makeToken(
                    (requestType & USB_DIR_IN) ? USB_PID_IN : USB_PID_OUT,
                    address, 0, packetLength, toggle);
            controlTds[tdCount].buffer = controlBufferPhys + transferred;
            tdCount++;

            transferred += packetLength;
            toggle = !toggle;
            if (tdCount >= CONTROL_TD_COUNT - 1) return false;
        }
    }

    controlTds[tdCount].status = TD_CTRL_ACTIVE |
            (3U << TD_CTRL_C_ERR_SHIFT) | (lowSpeed ? TD_CTRL_LS : 0);
    controlTds[tdCount].token = makeToken(
            (requestType & USB_DIR_IN) ? USB_PID_OUT : USB_PID_IN,
            address, 0, 0, true);
    controlTds[tdCount].buffer = 0;
    tdCount++;

    for (size_t i = 0; i < tdCount; i++) {
        if (i + 1 < tdCount) {
            controlTds[i].link = makeTdLink(controlTdsPhys +
                    (i + 1) * sizeof(UhciTd));
        } else {
            controlTds[i].link = UHCI_LINK_TERM;
        }
    }

    controlQh->element = controlTdsPhys;

    Clock* clock = Clock::get(CLOCK_MONOTONIC);
    struct timespec now;
    struct timespec timeout;
    timeout.tv_sec = 0;
    timeout.tv_nsec = 100000000L;
    if (!clock || clock->getTime(&now) < 0) {
        controlQh->element = UHCI_LINK_TERM;
        return false;
    }
    struct timespec deadline = timespecPlus(now, timeout);

    while (true) {
        bool done = controlQh->element == UHCI_LINK_TERM;
        bool failed = false;

        for (size_t i = 0; i < tdCount; i++) {
            uint32_t status = controlTds[i].status;
            if (!(status & TD_CTRL_ACTIVE) &&
                    (status & (TD_CTRL_STALLED | TD_CTRL_DBUFERR |
                    TD_CTRL_BABBLE | TD_CTRL_CRCTIMEO |
                    TD_CTRL_BITSTUFF))) {
                failed = true;
                break;
            }
        }

        if (done || failed) break;

        clock->getTime(&now);
        if (!timespecLess(now, deadline)) {
            controlQh->element = UHCI_LINK_TERM;
            return false;
        }

        sched_yield();
    }

    controlQh->element = UHCI_LINK_TERM;

    for (size_t i = 0; i < tdCount; i++) {
        uint32_t status = controlTds[i].status;
        if (status & TD_CTRL_ACTIVE) return false;
        if (status & (TD_CTRL_STALLED | TD_CTRL_DBUFERR | TD_CTRL_BABBLE |
                TD_CTRL_CRCTIMEO | TD_CTRL_BITSTUFF)) {
            return false;
        }
    }

    if ((requestType & USB_DIR_IN) && length > 0) {
        memcpy(data, controlBuffer, length);
    }

    return true;
}

bool UhciController::enumeratePort(unsigned int port) {
    bool lowSpeed = false;
    if (!resetPort(port, lowSpeed)) return false;

    UsbDeviceDescriptor descriptor = {};
    uint8_t packetSize0 = 8;

    if (!getDescriptor(0, lowSpeed, packetSize0, USB_DESCRIPTOR_DEVICE, 0,
            &descriptor, 8)) {
        return false;
    }

    packetSize0 = descriptor.maxPacketSize0;
    if (!getDescriptor(0, lowSpeed, packetSize0, USB_DESCRIPTOR_DEVICE, 0,
            &descriptor, sizeof(descriptor))) {
        return false;
    }

    uint8_t address = nextAddress++;
    if (!setAddress(0, lowSpeed, packetSize0, address)) {
        return false;
    }

    sleepMilliseconds(10);
    if (!configureBootInterfaces(address, lowSpeed, packetSize0)) {
        return false;
    }

    return true;
}

bool UhciController::getDescriptor(uint8_t address, bool lowSpeed,
        uint8_t packetSize0, uint8_t type, uint8_t index, void* data,
        size_t length) {
    return controlTransfer(address, lowSpeed, packetSize0,
            USB_DIR_IN | USB_TYPE_STANDARD | USB_RECIP_DEVICE,
            USB_REQUEST_GET_DESCRIPTOR, (type << 8) | index, 0, data, length);
}

void UhciController::initializeController(int irq) {
    outw(ioBase + USBINTR, 0);
    outw(ioBase + USBCMD, 0);
    sleepMilliseconds(1);

    outw(ioBase + USBCMD, USBCMD_HCRESET);
    sleepMilliseconds(10);

    for (unsigned int port = 0; port < UHCI_PORTS; port++) {
        outw(ioBase + USBPORTSC1 + port * 2, 0);
    }

    if (irq >= 0) {
        irqHandler.func = onUhciIrq;
        irqHandler.user = this;
        Interrupts::addIrqHandler(irq, &irqHandler);
    }

    outw(ioBase + USBSTS, 0xFFFF);
    outw(ioBase + USBFRNUM, 0);
    outl(ioBase + USBFLBASEADD, frameListPhys);
    outb(ioBase + USBSOF, USBSOF_DEFAULT);
    outw(ioBase + USBINTR, irq >= 0 ? USBINTR_IOC | USBINTR_TIMEOUT : 0);
    outw(ioBase + USBCMD, USBCMD_MAXP | USBCMD_CF | USBCMD_RS);
    sleepMilliseconds(10);

    if (inw(ioBase + USBSTS) & USBSTS_HCH) {
        Log::printf("UHCI controller at 0x%X failed to start\n", ioBase);
        return;
    }

    running = true;
}

void UhciController::onIrq() {
    if (!running) return;

    uint16_t status = readRegister(USBSTS);
    if (!status) return;

    writeRegister(USBSTS, status);

    if (!(status & (USBSTS_USBINT | USBSTS_ERROR | USBSTS_HSE |
            USBSTS_HCPE))) {
        return;
    }

    if (status & (USBSTS_HSE | USBSTS_HCPE)) {
        Log::printf("UHCI controller error 0x%X\n", status);
    }
}

void UhciController::processIrqWork() {
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

void UhciController::processInterruptDevices() {
    for (UsbHidDevice* device = interruptDevices; device; device = device->next) {
        uint8_t report[64];
        size_t length = 0;
        bool haveReport = false;

        {
            AutoLock lock(&mutex);

            if (device->qh->element != UHCI_LINK_TERM ||
                    (device->td->status & TD_CTRL_ACTIVE)) {
                continue;
            }

            uint32_t status = device->td->status;
            if (status & (TD_CTRL_STALLED | TD_CTRL_DBUFERR | TD_CTRL_BABBLE |
                    TD_CTRL_CRCTIMEO | TD_CTRL_BITSTUFF)) {
                armInterruptTransfer(device, false);
                continue;
            }

            length = actualLength(status);
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
            device->handleReport(report, length);
        }
    }
}

uint16_t UhciController::readRegister(uint16_t reg) {
    return inw(ioBase + reg);
}

uint16_t UhciController::readPortStatus(unsigned int port) {
    return readRegister(USBPORTSC1 + port * 2);
}

bool UhciController::resetPort(unsigned int port, bool& lowSpeed) {
    uint16_t status = readPortStatus(port);
    if (!(status & USBPORTSC_CCS)) return false;

    acknowledgePortChanges(port);
    setPortBits(port, USBPORTSC_PR);
    sleepMilliseconds(50);
    clearPortBits(port, USBPORTSC_PR);
    sleepMilliseconds(10);
    acknowledgePortChanges(port);
    setPortBits(port, USBPORTSC_PE);
    sleepMilliseconds(10);

    status = readPortStatus(port);
    if (!(status & USBPORTSC_CCS) || !(status & USBPORTSC_PE)) {
        acknowledgePortChanges(port);
        return false;
    }

    lowSpeed = status & USBPORTSC_LSDA;
    acknowledgePortChanges(port);
    return true;
}

void UhciController::registerInterruptDevice(UsbHidDevice* device) {
    AutoLock lock(&mutex);
    device->next = interruptDevices;
    interruptDevices = device;
    device->qh->link = rootQh->link;
    device->qh->element = UHCI_LINK_TERM;
    rootQh->link = makeQhLink(device->qhPhys);
    armInterruptTransfer(device, false);
}

bool UhciController::setAddress(uint8_t oldAddress, bool lowSpeed,
        uint8_t packetSize0, uint8_t newAddress) {
    uint8_t dummy = 0;
    return controlTransfer(oldAddress, lowSpeed, packetSize0,
            USB_TYPE_STANDARD | USB_RECIP_DEVICE, USB_REQUEST_SET_ADDRESS,
            newAddress, 0, &dummy, 0);
}

bool UhciController::setBootProtocol(uint8_t address, bool lowSpeed,
        uint8_t packetSize0, uint8_t interfaceNumber) {
    uint8_t dummy = 0;
    return controlTransfer(address, lowSpeed, packetSize0,
            USB_TYPE_CLASS | USB_RECIP_INTERFACE, HID_REQUEST_SET_PROTOCOL,
            0, interfaceNumber, &dummy, 0);
}

bool UhciController::setConfiguration(uint8_t address, bool lowSpeed,
        uint8_t packetSize0, uint8_t configurationValue) {
    uint8_t dummy = 0;
    return controlTransfer(address, lowSpeed, packetSize0,
            USB_TYPE_STANDARD | USB_RECIP_DEVICE,
            USB_REQUEST_SET_CONFIGURATION, configurationValue, 0, &dummy, 0);
}

bool UhciController::setKeyboardLeds(uint8_t address, bool lowSpeed,
        uint8_t packetSize0, uint8_t interfaceNumber, uint8_t leds) {
    return controlTransfer(address, lowSpeed, packetSize0,
            USB_TYPE_CLASS | USB_RECIP_INTERFACE, HID_REQUEST_SET_REPORT,
            HID_REPORT_TYPE_OUTPUT << 8, interfaceNumber, &leds, sizeof(leds));
}

void UhciController::scanPorts() {
    if (!running) return;

    for (unsigned int port = 0; port < UHCI_PORTS; port++) {
        uint16_t status = readPortStatus(port);
        bool connected = status & USBPORTSC_CCS;

        if (!connected) {
            portInitialized[port] = false;
            continue;
        }

        if (portInitialized[port]) continue;

        if (enumeratePort(port)) {
            portInitialized[port] = true;
            Log::printf("UHCI port %u enumerated\n", port + 1);
        }
    }
}

void UhciController::setPortBits(unsigned int port, uint16_t bits) {
    uint16_t status = readPortStatus(port);
    status |= bits;
    status &= ~PORT_RWC_BITS;
    status &= ~PORT_WZ_BITS;
    writeRegister(USBPORTSC1 + port * 2, status);
}

void UhciController::clearPortBits(unsigned int port, uint16_t bits) {
    uint16_t status = readPortStatus(port);
    status &= ~bits;
    status &= ~PORT_RWC_BITS;
    status &= ~PORT_WZ_BITS;
    status |= bits & PORT_RWC_BITS;
    writeRegister(USBPORTSC1 + port * 2, status);
}

void UhciController::writeRegister(uint16_t reg, uint16_t value) {
    outw(ioBase + reg, value);
}

void UhciController::writeRegister32(uint16_t reg, uint32_t value) {
    outl(ioBase + reg, value);
}

void Uhci::initialize(uint8_t bus, uint8_t device, uint8_t function) {
    uint32_t bar4 = Pci::readConfig(bus, device, function,
            offsetof(PciHeader, bar4));
    if (!(bar4 & 0x1)) {
        Log::printf("UHCI controller unsupported: BAR4 is not an I/O BAR\n");
        return;
    }

    uint16_t ioBase = bar4 & 0xFFFC;
    uint16_t vendor = Pci::readConfig(bus, device, function,
            offsetof(PciHeader, vendorId));
    uint32_t command = Pci::readConfig(bus, device, function,
            offsetof(PciHeader, command));
    command |= (1 << 0) | (1 << 2);
    command &= ~(1 << 10);
    Pci::writeConfig(bus, device, function, offsetof(PciHeader, command),
            command);

    uint32_t legsup = Pci::readConfig(bus, device, function, USBLEGSUP);
    legsup = (legsup & 0xFFFF0000U) | USBLEGSUP_RWC;
    Pci::writeConfig(bus, device, function, USBLEGSUP, legsup);

    if (vendor == PCI_VENDOR_INTEL) {
        uint32_t usbres = Pci::readConfig(bus, device, function, USBRES_INTEL);
        usbres &= 0xFFFFFF00U;
        Pci::writeConfig(bus, device, function, USBRES_INTEL, usbres);
    }

    int irq = Pci::getIrq(bus, device, function);
    if (irq < 0) {
        irq = Pci::readConfig(bus, device, function,
                offsetof(PciHeader, interruptLine)) & 0xFF;
        if (irq == 0xFF) irq = -1;
    }

    if (irq < 0) {
        Log::printf("UHCI controller at 0x%X is using polling mode\n",
                ioBase);
    }

    xnew UhciController(ioBase, irq);
}
