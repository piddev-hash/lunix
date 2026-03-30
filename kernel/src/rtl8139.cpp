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

/* kernel/src/rtl8139.cpp
 * Realtek RTL8139 ethernet driver.
 */

#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <lunix/net.h>
#include <lunix/kernel/addressspace.h>
#include <lunix/kernel/clock.h>
#include <lunix/kernel/interrupts.h>
#include <lunix/kernel/log.h>
#include <lunix/kernel/netdevice.h>
#include <lunix/kernel/pci.h>
#include <lunix/kernel/physicalmemory.h>
#include <lunix/kernel/portio.h>
#include <lunix/kernel/rtl8139.h>
#include <lunix/kernel/thread.h>
#include <lunix/kernel/worker.h>

namespace {

enum RegisterOffsets {
    MAC0 = 0x00,
    TxStatus0 = 0x10,
    TxAddr0 = 0x20,
    RxBuf = 0x30,
    ChipCmd = 0x37,
    RxBufPtr = 0x38,
    IntrMask = 0x3C,
    IntrStatus = 0x3E,
    TxConfig = 0x40,
    RxConfig = 0x44,
    RxMissed = 0x4C,
    Cfg9346 = 0x50,
    Config1 = 0x52,
    MultiIntr = 0x5C,
    BasicModeStatus = 0x64,
    HltClk = 0x5B,
};

enum ChipCommandBits {
    CmdReset = 0x10,
    CmdRxEnable = 0x08,
    CmdTxEnable = 0x04,
    RxBufferEmpty = 0x01,
};

enum InterruptBits {
    PCIErr = 0x8000,
    PCSTimeout = 0x4000,
    RxFIFOOver = 0x0040,
    RxUnderrun = 0x0020,
    RxOverflow = 0x0010,
    TxErr = 0x0008,
    TxOK = 0x0004,
    RxErr = 0x0002,
    RxOK = 0x0001,
};

enum TxStatusBits {
    TxUnderrun = 0x00004000,
    TxStatOK = 0x00008000,
    TxOutOfWindow = 0x20000000,
    TxAborted = 0x40000000,
};

enum RxStatusBits {
    RxTooLong = 0x0008,
    RxCrcErr = 0x0004,
    RxBadAlign = 0x0002,
    RxStatusOK = 0x0001,
};

enum ReceiveModeBits {
    AcceptBroadcast = 0x08,
    AcceptMulticast = 0x04,
    AcceptMyPhys = 0x02,
    AcceptAllPhys = 0x01,
};

enum RxConfigBits {
    RxCfgDmaUnlimited = (7 << 8),
    RxCfgFifoNone = (7 << 13),
    RxCfgReceive32K = (1 << 12),
    RxNoWrap = (1 << 7),
};

enum TxConfigBits {
    TxDmaBurst = (6 << 8),
    TxRetry = (8 << 4),
    TxIfg96 = (3 << 24),
    TxClearAbort = 1 << 0,
};

static const uint16_t rtl8139InterruptMask = PCIErr | PCSTimeout |
        RxUnderrun | RxOverflow | RxFIFOOver | TxErr | TxOK | RxErr | RxOK;
static const uint32_t rtl8139RxConfig = RxCfgReceive32K | RxNoWrap |
        RxCfgFifoNone | RxCfgDmaUnlimited;
static const uint32_t rtl8139TxConfig = TxIfg96 | TxDmaBurst | TxRetry;
static const uint32_t rtl8139TxStatusFlags = 0x00080000;

struct DmaArea {
    paddr_t physical;
    vaddr_t virtualAddress;
    size_t size;
};

static bool allocateContiguousDma32(size_t size, DmaArea& area) {
    size_t pages = ALIGNUP(size, PAGESIZE) / PAGESIZE;
    size_t searchPages = pages + 255;
    paddr_t* frames = new paddr_t[searchPages];
    if (!frames) return false;

    size_t count = 0;
    while (count < searchPages) {
        paddr_t frame = PhysicalMemory::popPageFrame32();
        if (!frame) break;
        frames[count++] = frame;
    }

    for (size_t i = 1; i < count; i++) {
        paddr_t frame = frames[i];
        size_t j = i;
        while (j > 0 && frames[j - 1] > frame) {
            frames[j] = frames[j - 1];
            j--;
        }
        frames[j] = frame;
    }

    bool found = false;
    size_t first = 0;
    if (count >= pages) {
        size_t runLength = 1;
        for (size_t i = 1; i < count; i++) {
            if (frames[i] == frames[i - 1] + PAGESIZE) {
                runLength++;
                if (runLength == pages) {
                    first = i + 1 - pages;
                    found = true;
                    break;
                }
            } else if (frames[i] != frames[i - 1]) {
                runLength = 1;
            }
        }
    }

    if (!found) {
        for (size_t i = 0; i < count; i++) {
            PhysicalMemory::pushPageFrame(frames[i]);
        }
        delete[] frames;
        return false;
    }

    for (size_t i = 0; i < count; i++) {
        if (i < first || i >= first + pages) {
            PhysicalMemory::pushPageFrame(frames[i]);
        }
    }

    area.physical = frames[first];
    area.size = pages * PAGESIZE;
    area.virtualAddress = kernelSpace->mapPhysical(area.physical, area.size,
            PROT_READ | PROT_WRITE);
    if (!area.virtualAddress) {
        for (size_t i = 0; i < pages; i++) {
            PhysicalMemory::pushPageFrame(area.physical + i * PAGESIZE);
        }
        delete[] frames;
        return false;
    }

    memset((void*) area.virtualAddress, 0, area.size);
    delete[] frames;
    return true;
}

class Rtl8139Device : public NetworkDevice, public ConstructorMayFail {
public:
    Rtl8139Device(uint8_t bus, uint8_t device, uint8_t function, int irq);
    ~Rtl8139Device() = default;
    NOT_COPYABLE(Rtl8139Device);
    NOT_MOVABLE(Rtl8139Device);

    void onIrq();
    void pollHardware();
    void processIrqWork();
public:
    Rtl8139Device* nextDevice;
private:
    struct TxBuffer {
        paddr_t physical;
        vaddr_t virtualAddress;
    };

    uint8_t read8(size_t offset) const;
    uint16_t read16(size_t offset) const;
    uint32_t read32(size_t offset) const;
    void write8(size_t offset, uint8_t value);
    void write16(size_t offset, uint16_t value);
    void write32(size_t offset, uint32_t value);
    void write8Flush(size_t offset, uint8_t value);
    void write32Flush(size_t offset, uint32_t value);

    bool canTransmitLocked() const override;
    void fillInfoLocked(struct net_info& info) const override;
    bool setPromiscuousModeLocked(bool enabled) override;
    bool transmitFrameLocked(const void* frame, size_t size) override;

    bool initializeMemory();
    bool initializeRegisters();
    void powerOn();
    bool resetChip();
    bool updateLinkStateLocked();
    bool reapTransmittedLocked();
    bool receiveFramesLocked();
    void restartHardwareLocked();
    void programReceiveFilterLocked();
private:
    static constexpr size_t rxBufferLength = 8192U << 2;
    static constexpr size_t rxBufferSize = rxBufferLength + 16 + 2048;
    static constexpr size_t txBufferCount = 4;

    bool hasMmio;
    uint8_t bus;
    uint8_t device;
    uint8_t function;
    int irq;
    uint16_t ioBase;
    vaddr_t mmioBase;
    DmaArea rxRing;
    TxBuffer txBuffers[txBufferCount];
    unsigned int curRx;
    unsigned int curTx;
    unsigned int dirtyTx;
    bool promiscuous;
    bool linkUp;
    bool irqJobQueued;
    uint16_t pendingInterrupts;
    uint8_t mac[6];
    IrqHandler irqHandler;
    WorkerJob irqJob;
};

static Rtl8139Device* firstRtl8139Device;
static bool rtl8139PollThreadStarted;

static void sleepMilliseconds(unsigned int milliseconds) {
    Clock* clock = Clock::get(CLOCK_MONOTONIC);
    if (!clock) return;

    struct timespec duration;
    duration.tv_sec = milliseconds / 1000;
    duration.tv_nsec = (milliseconds % 1000) * 1000000L;
    clock->nanosleep(0, &duration, nullptr);
}

void onRtl8139Irq(void* user, const InterruptContext* /*context*/) {
    ((Rtl8139Device*) user)->onIrq();
}

void rtl8139IrqWork(void* user) {
    ((Rtl8139Device*) user)->processIrqWork();
}

static NORETURN void rtl8139PollLoop() {
    while (true) {
        for (Rtl8139Device* device = firstRtl8139Device; device;
                device = device->nextDevice) {
            device->pollHardware();
        }

        sleepMilliseconds(1);
    }
}

static void startRtl8139PollThread(void*) {
    rtl8139PollLoop();
}

static void ensureRtl8139PollThread() {
    if (rtl8139PollThreadStarted) return;
    rtl8139PollThreadStarted = true;
    Log::printf("rtl8139: polling fallback enabled\n");
    Thread::createKernelThread(startRtl8139PollThread, nullptr);
}

Rtl8139Device::Rtl8139Device(uint8_t bus, uint8_t device, uint8_t function,
        int irq) : nextDevice(nullptr), hasMmio(false), bus(bus),
        device(device), function(function), irq(irq), ioBase(0), mmioBase(0),
        rxRing(), txBuffers(), curRx(0), curTx(0), dirtyTx(0),
        promiscuous(false), linkUp(false), irqJobQueued(false),
        pendingInterrupts(0), mac(), irqHandler(), irqJob() {
    uint32_t command = Pci::readConfig(bus, device, function,
            offsetof(PciHeader, command));
    uint32_t bar0 = Pci::readConfig(bus, device, function,
            offsetof(PciHeader, bar0));
    uint32_t bar1 = Pci::readConfig(bus, device, function,
            offsetof(PciHeader, bar1));

    if (bar0 & 0x1) {
        ioBase = bar0 & 0xFFFC;
        command |= 0x1;
    } else {
        paddr_t physical = bar1 & ~0xFU;
        mmioBase = kernelSpace->mapPhysical(physical, PAGESIZE,
                PROT_READ | PROT_WRITE);
        if (!mmioBase) FAIL_CONSTRUCTOR;
        hasMmio = true;
        command |= 0x2;
    }

    command |= 0x4;
    Pci::writeConfig(bus, device, function, offsetof(PciHeader, command),
            command);

    if (read32(TxConfig) == 0xFFFFFFFFU) {
        FAIL_CONSTRUCTOR;
    }

    if (!initializeMemory()) FAIL_CONSTRUCTOR;
    if (!initializeRegisters()) FAIL_CONSTRUCTOR;

    irqJob.func = rtl8139IrqWork;
    irqJob.context = this;
    irqHandler.func = onRtl8139Irq;
    irqHandler.user = this;
    Interrupts::addIrqHandler(irq, &irqHandler);

    nextDevice = firstRtl8139Device;
    firstRtl8139Device = this;
    ensureRtl8139PollThread();
}

uint8_t Rtl8139Device::read8(size_t offset) const {
    if (hasMmio) {
        return *(volatile uint8_t*) (mmioBase + offset);
    }
    return inb(ioBase + offset);
}

uint16_t Rtl8139Device::read16(size_t offset) const {
    if (hasMmio) {
        return *(volatile uint16_t*) (mmioBase + offset);
    }
    return inw(ioBase + offset);
}

uint32_t Rtl8139Device::read32(size_t offset) const {
    if (hasMmio) {
        return *(volatile uint32_t*) (mmioBase + offset);
    }
    return inl(ioBase + offset);
}

void Rtl8139Device::write8(size_t offset, uint8_t value) {
    if (hasMmio) {
        *(volatile uint8_t*) (mmioBase + offset) = value;
    } else {
        outb(ioBase + offset, value);
    }
}

void Rtl8139Device::write16(size_t offset, uint16_t value) {
    if (hasMmio) {
        *(volatile uint16_t*) (mmioBase + offset) = value;
    } else {
        outw(ioBase + offset, value);
    }
}

void Rtl8139Device::write32(size_t offset, uint32_t value) {
    if (hasMmio) {
        *(volatile uint32_t*) (mmioBase + offset) = value;
    } else {
        outl(ioBase + offset, value);
    }
}

void Rtl8139Device::write8Flush(size_t offset, uint8_t value) {
    write8(offset, value);
    (void) read8(offset);
}

void Rtl8139Device::write32Flush(size_t offset, uint32_t value) {
    write32(offset, value);
    (void) read32(offset);
}

bool Rtl8139Device::canTransmitLocked() const {
    return linkUp && curTx - dirtyTx < txBufferCount;
}

void Rtl8139Device::fillInfoLocked(struct net_info& info) const {
    memcpy(info.mac, mac, sizeof(mac));
    info.flags = (linkUp ? NET_INFO_LINK_UP : 0) |
            (promiscuous ? NET_INFO_PROMISCUOUS : 0);
    info.mtu = NET_DEFAULT_MTU;
    info.max_frame_size = NET_MAX_FRAME_SIZE;
}

bool Rtl8139Device::setPromiscuousModeLocked(bool enabled) {
    if (promiscuous == enabled) return true;
    promiscuous = enabled;
    programReceiveFilterLocked();
    notifyStateChangedLocked();
    return true;
}

bool Rtl8139Device::transmitFrameLocked(const void* frame, size_t size) {
    if (!canTransmitLocked()) return false;

    unsigned int entry = curTx % txBufferCount;
    memcpy((void*) txBuffers[entry].virtualAddress, frame, size);
    __atomic_thread_fence(__ATOMIC_RELEASE);
    write32Flush(TxStatus0 + entry * sizeof(uint32_t),
            rtl8139TxStatusFlags | size);
    curTx++;
    return true;
}

bool Rtl8139Device::initializeMemory() {
    if (!allocateContiguousDma32(rxBufferSize, rxRing)) return false;

    for (size_t i = 0; i < txBufferCount; i++) {
        paddr_t frame = PhysicalMemory::popPageFrame32();
        if (!frame) return false;
        txBuffers[i].physical = frame;
        txBuffers[i].virtualAddress = kernelSpace->mapPhysical(frame, PAGESIZE,
                PROT_READ | PROT_WRITE);
        if (!txBuffers[i].virtualAddress) return false;
        memset((void*) txBuffers[i].virtualAddress, 0, PAGESIZE);
    }
    return true;
}

void Rtl8139Device::powerOn() {
    write8(HltClk, 'R');
    write8Flush(Cfg9346, 0xC0);
    uint8_t config1 = read8(Config1);
    config1 &= ~0x3;
    write8(Config1, config1);
    write8(Cfg9346, 0x00);
}

bool Rtl8139Device::resetChip() {
    write8(ChipCmd, CmdReset);
    for (unsigned int i = 0; i < 100000; i++) {
        if (!(read8(ChipCmd) & CmdReset)) {
            return true;
        }
    }
    return false;
}

void Rtl8139Device::programReceiveFilterLocked() {
    uint32_t filter = rtl8139RxConfig | AcceptBroadcast | AcceptMulticast |
            AcceptMyPhys;
    if (promiscuous) filter |= AcceptAllPhys;
    write32(RxConfig, filter);
}

bool Rtl8139Device::initializeRegisters() {
    powerOn();
    if (!resetChip()) return false;

    for (size_t i = 0; i < sizeof(mac); i++) {
        mac[i] = read8(MAC0 + i);
    }

    curRx = 0;
    curTx = 0;
    dirtyTx = 0;
    pendingInterrupts = 0;
    memset((void*) rxRing.virtualAddress, 0, rxRing.size);

    write8Flush(Cfg9346, 0xC0);
    write32Flush(RxBuf, rxRing.physical);
    write8(ChipCmd, CmdRxEnable | CmdTxEnable);
    programReceiveFilterLocked();
    write32(TxConfig, rtl8139TxConfig);
    for (size_t i = 0; i < txBufferCount; i++) {
        write32Flush(TxAddr0 + i * sizeof(uint32_t), txBuffers[i].physical);
    }
    write32(RxMissed, 0);
    write16(MultiIntr, read16(MultiIntr) & 0xF000);
    write16(IntrStatus, 0xFFFF);
    write16(IntrMask, rtl8139InterruptMask);
    write8(Cfg9346, 0x00);

    linkUp = updateLinkStateLocked();
    return true;
}

bool Rtl8139Device::updateLinkStateLocked() {
    (void) read16(BasicModeStatus);
    bool newLink = read16(BasicModeStatus) & (1 << 2);
    bool changed = newLink != linkUp;
    linkUp = newLink;
    return changed;
}

bool Rtl8139Device::reapTransmittedLocked() {
    bool reclaimed = false;

    while (dirtyTx < curTx) {
        unsigned int entry = dirtyTx % txBufferCount;
        uint32_t status = read32(TxStatus0 + entry * sizeof(uint32_t));
        if (!(status & (TxStatOK | TxUnderrun | TxAborted))) break;

        if (status & TxAborted) {
            Log::printf("rtl8139 %u/%u/%u: transmit aborted (0x%X)\n", bus,
                    device, function, status);
            write32(TxConfig, TxClearAbort);
        } else if (status & TxOutOfWindow) {
            Log::printf("rtl8139 %u/%u/%u: transmit window error (0x%X)\n",
                    bus, device, function, status);
        }

        dirtyTx++;
        reclaimed = true;
    }

    if (reclaimed) {
        kthread_cond_broadcast(&writeCond);
    }
    return reclaimed;
}

bool Rtl8139Device::receiveFramesLocked() {
    bool received = false;

    while (!(read8(ChipCmd) & RxBufferEmpty)) {
        uint32_t ringOffset = curRx % rxBufferLength;
        uint32_t descriptor = *(volatile uint32_t*)
                (rxRing.virtualAddress + ringOffset);
        uint16_t status = descriptor & 0xFFFF;
        uint16_t frameSize = descriptor >> 16;

        if (frameSize < 8 || frameSize > NET_MAX_FRAME_SIZE + 4 ||
                !(status & RxStatusOK) || (status & (RxTooLong | RxCrcErr |
                RxBadAlign))) {
            return false;
        }

        size_t payloadSize = frameSize - 4;
        queueReceivedFrameLocked((const void*) (rxRing.virtualAddress +
                ringOffset + 4), payloadSize);
        received = true;

        curRx = (curRx + frameSize + 4 + 3) & ~3U;
        write16(RxBufPtr, (uint16_t) (curRx - 16));
    }

    return received;
}

void Rtl8139Device::restartHardwareLocked() {
    write16(IntrMask, 0);
    if (!initializeRegisters()) {
        Log::printf("rtl8139 %u/%u/%u: failed to restart device\n", bus,
                device, function);
        linkUp = false;
    }
    dirtyTx = curTx;
    notifyStateChangedLocked();
}

void Rtl8139Device::onIrq() {
    uint16_t status = read16(IntrStatus);
    if (status == 0xFFFF || !(status & rtl8139InterruptMask)) return;

    write16(IntrStatus, status);
    pendingInterrupts |= status;
    if (!irqJobQueued) {
        irqJobQueued = true;
        WorkerThread::addJob(&irqJob);
    }
}

void Rtl8139Device::pollHardware() {
    uint16_t status = read16(IntrStatus);
    if (status == 0xFFFF) return;
    if (status & rtl8139InterruptMask) {
        write16(IntrStatus, status);
    }

    AutoLock lock(&mutex);
    bool notify = false;
    bool restart = false;

    if (status & (PCIErr | PCSTimeout | RxOverflow | RxFIFOOver | RxErr)) {
        Log::printf("rtl8139 %u/%u/%u: polled receive error 0x%X\n", bus,
                device, function, status);
        restart = true;
    }

    bool rxPending = !(read8(ChipCmd) & RxBufferEmpty);
    if (!restart && rxPending) {
        restart = !receiveFramesLocked();
        notify = !restart;
    }

    if (restart) {
        restartHardwareLocked();
        return;
    }

    notify |= reapTransmittedLocked();
    notify |= updateLinkStateLocked();
    if (notify) notifyStateChangedLocked();
}

void Rtl8139Device::processIrqWork() {
    while (true) {
        Interrupts::disable();
        uint16_t status = pendingInterrupts;
        pendingInterrupts = 0;
        Interrupts::enable();

        AutoLock lock(&mutex);
        bool notify = false;
        bool restart = false;

        if (status & (PCIErr | PCSTimeout | RxOverflow | RxFIFOOver | RxErr)) {
            Log::printf("rtl8139 %u/%u/%u: receive error 0x%X\n", bus, device,
                    function, status);
            restart = true;
        } else if (status & RxOK) {
            restart = !receiveFramesLocked();
        }

        if (restart) {
            restartHardwareLocked();
            notify = true;
        } else {
            if (status & (TxOK | TxErr)) {
                notify |= reapTransmittedLocked();
            }
            notify |= updateLinkStateLocked();
            if (notify) notifyStateChangedLocked();
        }

        Interrupts::disable();
        if (!pendingInterrupts) {
            irqJobQueued = false;
            Interrupts::enable();
            return;
        }
        Interrupts::enable();
    }
}

}

void Rtl8139::initialize(uint8_t bus, uint8_t device, uint8_t function) {
    int irq = Pci::getIrq(bus, device, function);
    if (irq < 0) {
        irq = Pci::readConfig(bus, device, function,
                offsetof(PciHeader, interruptLine)) & 0xFF;
        if (irq == 0 || irq == 0xFF) irq = -1;
    }
    if (irq < 0) {
        Log::printf("rtl8139 %u/%u/%u unsupported: cannot allocate IRQ\n",
                bus, device, function);
        return;
    }

    Reference<Rtl8139Device> nic = new Rtl8139Device(bus, device, function,
            irq);
    if (!nic) {
        Log::printf("rtl8139 %u/%u/%u initialization failed\n", bus, device,
                function);
        return;
    }

    NetworkDevice::registerDevice(nic);
    Log::printf("rtl8139 %u/%u/%u initialized as raw Ethernet device\n", bus,
            device, function);
}
