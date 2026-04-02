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

/* kernel/src/rtl8169.cpp
 * Realtek RTL8111/8168/8411 ethernet driver.
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
#include <lunix/kernel/rtl8169.h>
#include <lunix/kernel/thread.h>
#include <lunix/kernel/worker.h>

namespace {

enum RegisterOffsets {
    MAC0 = 0x00,
    MAC4 = 0x04,
    MAR0 = 0x08,
    TxDescStartAddrLow = 0x20,
    TxDescStartAddrHigh = 0x24,
    ChipCmd = 0x37,
    TxPoll = 0x38,
    IntrMask = 0x3C,
    IntrStatus = 0x3E,
    TxConfig = 0x40,
    RxConfig = 0x44,
    Cfg9346 = 0x50,
    PHYstatus = 0x6C,
    RxMaxSize = 0xDA,
    CPlusCmd = 0xE0,
    IntrMitigate = 0xE2,
    RxDescAddrLow = 0xE4,
    RxDescAddrHigh = 0xE8,
    MaxTxPacketSize = 0xEC,
    MISC = 0xF0,
};

enum RegisterContent {
    SYSErr = 0x8000,
    TxDescUnavail = 0x0080,
    RxFIFOOver = 0x0040,
    LinkChg = 0x0020,
    RxOverflow = 0x0010,
    TxErr = 0x0008,
    TxOK = 0x0004,
    RxErr = 0x0002,
    RxOK = 0x0001,

    CmdReset = 0x10,
    CmdRxEnable = 0x08,
    CmdTxEnable = 0x04,

    NPQ = 0x40,

    Cfg9346Lock = 0x00,
    Cfg9346Unlock = 0xC0,

    AcceptBroadcast = 0x08,
    AcceptMulticast = 0x04,
    AcceptMyPhys = 0x02,
    AcceptAllPhys = 0x01,

    RX128_INT_EN = (1 << 15),
    RX_MULTI_EN = (1 << 14),
    RX_EARLY_OFF = (1 << 11),
    RX_DMA_BURST = (7 << 8),

    TxInterFrameGap = (3 << 24),
    TxDmaBurst = (7 << 8),

    LinkStatus = 0x02,

    NormalMode = (1 << 13),
    RxVlan = (1 << 6),
    RxChecksum = (1 << 5),
    CPlusCmdMask = NormalMode | RxVlan | RxChecksum | 0x3,

    DescOwn = (1U << 31),
    RingEnd = (1U << 30),
    FirstFrag = (1U << 29),
    LastFrag = (1U << 28),

    RxRWT = (1 << 22),
    RxRES = (1 << 21),
    RxRUNT = (1 << 20),
    RxCRC = (1 << 19),
    RxPacketSizeMask = 0x3FFF,

    TxPacketMax = (8064 >> 7),

    RXDV_GATED_EN = (1 << 19),
};

static const uint16_t rtl8169InterruptMask = SYSErr | TxDescUnavail |
        RxFIFOOver | LinkChg | RxOverflow | TxErr | TxOK | RxErr | RxOK;
static const uint32_t rtl8169TxConfig = TxInterFrameGap | TxDmaBurst;
static const uint32_t rtl8169BaseRxConfig = RX128_INT_EN | RX_MULTI_EN |
        RX_EARLY_OFF | RX_DMA_BURST;

struct PACKED TxDesc {
    uint32_t opts1;
    uint32_t opts2;
    uint64_t addr;
};

struct PACKED RxDesc {
    uint32_t opts1;
    uint32_t opts2;
    uint64_t addr;
};

struct DmaArea {
    paddr_t physical;
    vaddr_t virtualAddress;
    size_t size;
};

static bool allocateDmaPage32(DmaArea& area) {
    area.physical = PhysicalMemory::popPageFrame32();
    if (!area.physical) return false;

    area.size = PAGESIZE;
    area.virtualAddress = kernelSpace->mapPhysical(area.physical, area.size,
            PROT_READ | PROT_WRITE);
    if (!area.virtualAddress) return false;

    memset((void*) area.virtualAddress, 0, area.size);
    return true;
}

class Rtl8169Device : public NetworkDevice, public ConstructorMayFail {
public:
    Rtl8169Device(uint8_t bus, uint8_t device, uint8_t function, int irq);
    ~Rtl8169Device() = default;
    NOT_COPYABLE(Rtl8169Device);
    NOT_MOVABLE(Rtl8169Device);

    void onIrq();
    void pollHardware();
    void processIrqWork();
public:
    Rtl8169Device* nextDevice;
private:
    uint8_t read8(size_t offset) const;
    uint16_t read16(size_t offset) const;
    uint32_t read32(size_t offset) const;
    void write8(size_t offset, uint8_t value);
    void write16(size_t offset, uint16_t value);
    void write32(size_t offset, uint32_t value);
    void pciCommit();

    bool canTransmitLocked() const override;
    void fillInfoLocked(struct net_info& info) const override;
    bool setPromiscuousModeLocked(bool enabled) override;
    bool transmitFrameLocked(const void* frame, size_t size) override;

    bool initializeMemory();
    bool initializeRegisters();
    bool mapMmioBar();
    bool resetChip();
    bool updateLinkStateLocked();
    bool reapTransmittedLocked();
    bool receiveFramesLocked();
    void initializeDescriptorRingsLocked();
    void markRxDescriptorAvailable(size_t index);
    void programReceiveFilterLocked();
    void logHardwareStateLocked(const char* reason, uint16_t status);
    void restartHardwareLocked();
private:
    static constexpr size_t txDescCount = 64;
    static constexpr size_t rxDescCount = 64;
    static constexpr size_t dmaBufferSize = PAGESIZE;

    uint8_t bus;
    uint8_t device;
    uint8_t function;
    int irq;
    vaddr_t mmioBase;
    DmaArea txDescArea;
    DmaArea rxDescArea;
    DmaArea txBuffers[txDescCount];
    DmaArea rxBuffers[rxDescCount];
    TxDesc* txDescs;
    RxDesc* rxDescs;
    uint32_t curRx;
    uint32_t curTx;
    uint32_t dirtyTx;
    uint16_t cpCmd;
    uint16_t xid;
    uint8_t pciRevision;
    bool promiscuous;
    bool linkUp;
    bool irqJobQueued;
    uint16_t pendingStatus;
    uint32_t txPacketsSubmitted;
    uint32_t txPacketsCompleted;
    uint32_t rxPacketsAccepted;
    uint32_t rxPacketsDropped;
    uint32_t debugPollCountdown;
    bool loggedFirstTxComplete;
    bool loggedFirstRxFrame;
    bool loggedFirstRxDrop;
    uint8_t mac[6];
    IrqHandler irqHandler;
    WorkerJob irqJob;
};

static Rtl8169Device* firstRtl8169Device;
static bool rtl8169PollThreadStarted;

static void sleepMilliseconds(unsigned int milliseconds) {
    Clock* clock = Clock::get(CLOCK_MONOTONIC);
    if (!clock) return;

    struct timespec duration;
    duration.tv_sec = milliseconds / 1000;
    duration.tv_nsec = (milliseconds % 1000) * 1000000L;
    clock->nanosleep(0, &duration, nullptr);
}

void onRtl8169Irq(void* user, const InterruptContext* /*context*/) {
    ((Rtl8169Device*) user)->onIrq();
}

void rtl8169IrqWork(void* user) {
    ((Rtl8169Device*) user)->processIrqWork();
}

static NORETURN void rtl8169PollLoop() {
    while (true) {
        for (Rtl8169Device* device = firstRtl8169Device; device;
                device = device->nextDevice) {
            device->pollHardware();
        }

        sleepMilliseconds(1);
    }
}

static void startRtl8169PollThread(void*) {
    rtl8169PollLoop();
}

static void ensureRtl8169PollThread() {
    if (rtl8169PollThreadStarted) return;
    rtl8169PollThreadStarted = true;
    Thread::createKernelThread(startRtl8169PollThread, nullptr);
}

Rtl8169Device::Rtl8169Device(uint8_t bus, uint8_t device, uint8_t function,
        int irq) : nextDevice(nullptr), bus(bus), device(device),
        function(function), irq(irq), mmioBase(0), txDescArea(), rxDescArea(),
        txBuffers(), rxBuffers(), txDescs(nullptr), rxDescs(nullptr), curRx(0),
        curTx(0), dirtyTx(0), cpCmd(0), xid(0), pciRevision(0),
        promiscuous(false), linkUp(false), irqJobQueued(false),
        pendingStatus(0), txPacketsSubmitted(0), txPacketsCompleted(0),
        rxPacketsAccepted(0), rxPacketsDropped(0), debugPollCountdown(1000),
        loggedFirstTxComplete(false), loggedFirstRxFrame(false),
        loggedFirstRxDrop(false), mac(), irqHandler(), irqJob() {
    if (!mapMmioBar()) FAIL_CONSTRUCTOR;

    uint32_t command = Pci::readConfig(bus, device, function,
            offsetof(PciHeader, command));
    command |= 0x2 | 0x4;
    Pci::writeConfig(bus, device, function, offsetof(PciHeader, command),
            command);

    if (read32(TxConfig) == 0xFFFFFFFFU) FAIL_CONSTRUCTOR;

    xid = (read32(TxConfig) >> 20) & 0xFCF;
    cpCmd = read16(CPlusCmd) & CPlusCmdMask;
    pciRevision = Pci::readConfig(bus, device, function,
            offsetof(PciHeader, revisionId)) & 0xFF;

    if (!initializeMemory()) FAIL_CONSTRUCTOR;
    if (!initializeRegisters()) FAIL_CONSTRUCTOR;

    irqJob.func = rtl8169IrqWork;
    irqJob.context = this;
    irqHandler.func = onRtl8169Irq;
    irqHandler.user = this;
    Interrupts::addIrqHandler(irq, &irqHandler);

    nextDevice = firstRtl8169Device;
    firstRtl8169Device = this;
    ensureRtl8169PollThread();
}

bool Rtl8169Device::mapMmioBar() {
    for (size_t i = 0; i < 6; i++) {
        size_t offset = offsetof(PciHeader, bar0) + i * sizeof(uint32_t);
        uint32_t bar = Pci::readConfig(bus, device, function, offset);
        if (bar == 0 || bar == 0xFFFFFFFFU || (bar & 0x1)) continue;

        paddr_t physical = bar & ~0xFULL;
        if ((bar & 0x6) == 0x4 && i + 1 < 6) {
            uint32_t high = Pci::readConfig(bus, device, function,
                    offset + sizeof(uint32_t));
            physical |= (paddr_t) high << 32;
            i++;
        }

        mmioBase = kernelSpace->mapPhysical(physical, PAGESIZE,
                PROT_READ | PROT_WRITE);
        if (mmioBase) return true;
    }

    return false;
}

uint8_t Rtl8169Device::read8(size_t offset) const {
    return *(volatile uint8_t*) (mmioBase + offset);
}

uint16_t Rtl8169Device::read16(size_t offset) const {
    return *(volatile uint16_t*) (mmioBase + offset);
}

uint32_t Rtl8169Device::read32(size_t offset) const {
    return *(volatile uint32_t*) (mmioBase + offset);
}

void Rtl8169Device::write8(size_t offset, uint8_t value) {
    *(volatile uint8_t*) (mmioBase + offset) = value;
}

void Rtl8169Device::write16(size_t offset, uint16_t value) {
    *(volatile uint16_t*) (mmioBase + offset) = value;
}

void Rtl8169Device::write32(size_t offset, uint32_t value) {
    *(volatile uint32_t*) (mmioBase + offset) = value;
}

void Rtl8169Device::pciCommit() {
    (void) read8(ChipCmd);
}

bool Rtl8169Device::initializeMemory() {
    if (!allocateDmaPage32(txDescArea) || !allocateDmaPage32(rxDescArea)) {
        return false;
    }

    txDescs = (TxDesc*) txDescArea.virtualAddress;
    rxDescs = (RxDesc*) rxDescArea.virtualAddress;

    for (size_t i = 0; i < txDescCount; i++) {
        if (!allocateDmaPage32(txBuffers[i])) return false;
    }

    for (size_t i = 0; i < rxDescCount; i++) {
        if (!allocateDmaPage32(rxBuffers[i])) return false;
    }

    return true;
}

bool Rtl8169Device::resetChip() {
    write8(ChipCmd, CmdReset);
    for (unsigned int i = 0; i < 100000; i++) {
        if (!(read8(ChipCmd) & CmdReset)) return true;
    }
    return false;
}

void Rtl8169Device::markRxDescriptorAvailable(size_t index) {
    uint32_t eor = index == rxDescCount - 1 ? (uint32_t) RingEnd : 0;
    rxDescs[index].opts2 = 0;
    __atomic_thread_fence(__ATOMIC_RELEASE);
    rxDescs[index].opts1 = DescOwn | eor | dmaBufferSize;
}

void Rtl8169Device::initializeDescriptorRingsLocked() {
    memset((void*) txDescArea.virtualAddress, 0, txDescArea.size);
    memset((void*) rxDescArea.virtualAddress, 0, rxDescArea.size);

    for (size_t i = 0; i < txDescCount; i++) {
        txDescs[i].addr = txBuffers[i].physical;
        txDescs[i].opts1 = i == txDescCount - 1 ? (uint32_t) RingEnd : 0;
    }

    for (size_t i = 0; i < rxDescCount; i++) {
        rxDescs[i].addr = rxBuffers[i].physical;
        markRxDescriptorAvailable(i);
    }

    curRx = 0;
    curTx = 0;
    dirtyTx = 0;
}

void Rtl8169Device::programReceiveFilterLocked() {
    uint32_t rxMode = AcceptBroadcast | AcceptMulticast | AcceptMyPhys;
    if (promiscuous) rxMode |= AcceptAllPhys;

    write32(MAR0, 0xFFFFFFFFU);
    write32(MAR0 + 4, 0xFFFFFFFFU);
    write32(RxConfig, (read32(RxConfig) & ~0x0FU) | rxMode);
}

bool Rtl8169Device::initializeRegisters() {
    AutoLock lock(&mutex);

    if (!resetChip()) return false;
    initializeDescriptorRingsLocked();

    write8(Cfg9346, Cfg9346Unlock);
    write16(CPlusCmd, cpCmd);
    write16(IntrMitigate, 0);
    write8(MaxTxPacketSize, TxPacketMax);
    write16(RxMaxSize, NET_MAX_FRAME_SIZE + 4);
    write32(TxDescStartAddrHigh, txDescArea.physical >> 32);
    write32(TxDescStartAddrLow, txDescArea.physical & 0xFFFFFFFFU);
    write32(RxDescAddrHigh, rxDescArea.physical >> 32);
    write32(RxDescAddrLow, rxDescArea.physical & 0xFFFFFFFFU);
    // Linux clears RXDV gating on newer RTL8111/8168/8411 revisions to avoid
    // receive path stalls after link-up.
    write32(MISC, read32(MISC) & ~RXDV_GATED_EN);
    write8(ChipCmd, CmdTxEnable | CmdRxEnable);
    write32(RxConfig, rtl8169BaseRxConfig);
    write32(TxConfig, rtl8169TxConfig);
    programReceiveFilterLocked();
    write16(IntrMask, 0);
    write16(IntrStatus, 0xFFFF);
    write8(Cfg9346, Cfg9346Lock);
    pciCommit();
    write16(IntrMask, rtl8169InterruptMask);

    uint32_t mac0 = read32(MAC0);
    uint32_t mac4 = read32(MAC4);
    mac[0] = mac0 & 0xFF;
    mac[1] = (mac0 >> 8) & 0xFF;
    mac[2] = (mac0 >> 16) & 0xFF;
    mac[3] = (mac0 >> 24) & 0xFF;
    mac[4] = mac4 & 0xFF;
    mac[5] = (mac4 >> 8) & 0xFF;

    linkUp = read8(PHYstatus) & LinkStatus;
    return true;
}

bool Rtl8169Device::canTransmitLocked() const {
    return linkUp && curTx - dirtyTx < txDescCount;
}

void Rtl8169Device::fillInfoLocked(struct net_info& info) const {
    memcpy(info.mac, mac, sizeof(mac));
    info.flags = (linkUp ? NET_INFO_LINK_UP : 0) |
            (promiscuous ? NET_INFO_PROMISCUOUS : 0);
    info.mtu = NET_DEFAULT_MTU;
    info.max_frame_size = NET_MAX_FRAME_SIZE;
}

bool Rtl8169Device::setPromiscuousModeLocked(bool enabled) {
    if (promiscuous == enabled) return true;
    promiscuous = enabled;
    programReceiveFilterLocked();
    notifyStateChangedLocked();
    return true;
}

bool Rtl8169Device::transmitFrameLocked(const void* frame, size_t size) {
    if (!canTransmitLocked()) return false;

    unsigned int entry = curTx % txDescCount;
    memcpy((void*) txBuffers[entry].virtualAddress, frame, size);

    txDescs[entry].opts2 = 0;
    uint32_t opts1 = size | FirstFrag | LastFrag;
    if (entry == txDescCount - 1) opts1 |= RingEnd;

    __atomic_thread_fence(__ATOMIC_RELEASE);
    txDescs[entry].opts1 = opts1 | DescOwn;
    write8(TxPoll, NPQ);
    curTx++;
    txPacketsSubmitted++;
    return true;
}

bool Rtl8169Device::updateLinkStateLocked() {
    bool newLink = read8(PHYstatus) & LinkStatus;
    bool changed = newLink != linkUp;
    linkUp = newLink;
    return changed;
}

bool Rtl8169Device::reapTransmittedLocked() {
    bool reclaimed = false;

    while (dirtyTx < curTx) {
        unsigned int entry = dirtyTx % txDescCount;
        if (txDescs[entry].opts1 & DescOwn) break;

        txDescs[entry].opts2 = 0;
        txDescs[entry].opts1 = entry == txDescCount - 1 ?
                (uint32_t) RingEnd : 0;
        dirtyTx++;
        txPacketsCompleted++;
        reclaimed = true;
    }

    if (reclaimed && !loggedFirstTxComplete) {
        loggedFirstTxComplete = true;
    }

    if (reclaimed) kthread_cond_broadcast(&writeCond);
    return reclaimed;
}

bool Rtl8169Device::receiveFramesLocked() {
    bool received = false;

    while (true) {
        unsigned int entry = curRx % rxDescCount;
        uint32_t status = rxDescs[entry].opts1;
        if (status & DescOwn) break;

        __atomic_thread_fence(__ATOMIC_ACQUIRE);
        size_t packetSize = status & RxPacketSizeMask;

        if ((status & (FirstFrag | LastFrag)) !=
                (FirstFrag | LastFrag) ||
                (status & (RxRES | RxRWT | RxRUNT | RxCRC)) ||
                packetSize < 4 || packetSize > NET_MAX_FRAME_SIZE + 4) {
            rxPacketsDropped++;
            if (!loggedFirstRxDrop) {
                loggedFirstRxDrop = true;
            }
            markRxDescriptorAvailable(entry);
            curRx++;
            continue;
        }

        queueReceivedFrameLocked((void*) rxBuffers[entry].virtualAddress,
                packetSize - 4);
        rxPacketsAccepted++;
        if (!loggedFirstRxFrame) {
            loggedFirstRxFrame = true;
        }
        markRxDescriptorAvailable(entry);
        curRx++;
        received = true;
    }

    return received;
}

void Rtl8169Device::logHardwareStateLocked(const char* reason, uint16_t status) {
    (void) reason;
    (void) status;
}

void Rtl8169Device::restartHardwareLocked() {
    write16(IntrMask, 0);
    if (!resetChip()) {
        linkUp = false;
        notifyStateChangedLocked();
        return;
    }

    initializeDescriptorRingsLocked();
    write8(Cfg9346, Cfg9346Unlock);
    write16(CPlusCmd, cpCmd);
    write16(IntrMitigate, 0);
    write8(MaxTxPacketSize, TxPacketMax);
    write16(RxMaxSize, NET_MAX_FRAME_SIZE + 4);
    write32(TxDescStartAddrHigh, txDescArea.physical >> 32);
    write32(TxDescStartAddrLow, txDescArea.physical & 0xFFFFFFFFU);
    write32(RxDescAddrHigh, rxDescArea.physical >> 32);
    write32(RxDescAddrLow, rxDescArea.physical & 0xFFFFFFFFU);
    write32(MISC, read32(MISC) & ~RXDV_GATED_EN);
    write8(ChipCmd, CmdTxEnable | CmdRxEnable);
    write32(RxConfig, rtl8169BaseRxConfig);
    write32(TxConfig, rtl8169TxConfig);
    programReceiveFilterLocked();
    write16(IntrStatus, 0xFFFF);
    write8(Cfg9346, Cfg9346Lock);
    pciCommit();
    write16(IntrMask, rtl8169InterruptMask);

    dirtyTx = 0;
    curTx = 0;
    curRx = 0;
    txPacketsCompleted = 0;
    txPacketsSubmitted = 0;
    rxPacketsAccepted = 0;
    rxPacketsDropped = 0;
    debugPollCountdown = 1000;
    loggedFirstTxComplete = false;
    loggedFirstRxFrame = false;
    loggedFirstRxDrop = false;
    updateLinkStateLocked();
    notifyStateChangedLocked();
}

void Rtl8169Device::onIrq() {
    uint16_t status = read16(IntrStatus);
    if (status == 0xFFFF || !(status & rtl8169InterruptMask)) return;

    write16(IntrStatus, status);
    pendingStatus |= status;
    if (!irqJobQueued) {
        irqJobQueued = true;
        WorkerThread::addJob(&irqJob);
    }
}

void Rtl8169Device::pollHardware() {
    uint16_t status = read16(IntrStatus);
    if (status == 0xFFFF) return;
    if (status & rtl8169InterruptMask) {
        write16(IntrStatus, status);
    }

    AutoLock lock(&mutex);
    if (status & SYSErr) {
        Log::printf("rtl8169 %u/%u/%u: polled system error 0x%X\n", bus,
                device, function, status);
        restartHardwareLocked();
        return;
    }

    bool notify = false;
    notify |= reapTransmittedLocked();
    notify |= receiveFramesLocked();
    notify |= updateLinkStateLocked();
    if (debugPollCountdown > 0) {
        debugPollCountdown--;
    } else {
        if (txPacketsSubmitted != txPacketsCompleted ||
                rxPacketsAccepted == 0 || rxPacketsDropped != 0) {
            logHardwareStateLocked("poll", status);
        }
        debugPollCountdown = 1000;
    }
    if (notify) notifyStateChangedLocked();
}

void Rtl8169Device::processIrqWork() {
    while (true) {
        Interrupts::disable();
        uint16_t status = pendingStatus;
        pendingStatus = 0;
        Interrupts::enable();

        AutoLock lock(&mutex);
        bool notify = false;

        if (status & SYSErr) {
            Log::printf("rtl8169 %u/%u/%u: system error 0x%X\n", bus, device,
                    function, status);
            restartHardwareLocked();
        } else {
            if (status & (TxOK | TxErr | TxDescUnavail)) {
                notify |= reapTransmittedLocked();
            }
            if (status & (RxOK | RxErr | RxOverflow | RxFIFOOver)) {
                notify |= receiveFramesLocked();
            }
            if (status & LinkChg) {
                notify |= updateLinkStateLocked();
            }
            if (notify) notifyStateChangedLocked();
        }

        Interrupts::disable();
        if (!pendingStatus) {
            irqJobQueued = false;
            Interrupts::enable();
            return;
        }
        Interrupts::enable();
    }
}

}

void Rtl8169::initialize(uint8_t bus, uint8_t device, uint8_t function) {
    int irq = Pci::getIrq(bus, device, function);
    if (irq < 0) {
        irq = Pci::readConfig(bus, device, function,
                offsetof(PciHeader, interruptLine)) & 0xFF;
        if (irq == 0 || irq == 0xFF) irq = -1;
    }
    if (irq < 0) {
        Log::printf("rtl8169 %u/%u/%u unsupported: cannot allocate IRQ\n",
                bus, device, function);
        return;
    }

    Reference<Rtl8169Device> nic = new Rtl8169Device(bus, device, function,
            irq);
    if (!nic) {
        Log::printf("rtl8169 %u/%u/%u initialization failed\n", bus, device,
                function);
        return;
    }

    NetworkDevice::registerDevice(nic);
}
