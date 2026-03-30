// SPDX-License-Identifier: GPL-2.0-or-later

/* kernel/src/hda.cpp
 * Intel HD Audio controller.
 *
 * This is a minimal playback-only HDA path. It uses immediate commands for
 * codec setup and a kernel thread that refills a cyclic DMA buffer by polling
 * LPIB. The goal is to get a practical PCM sink for /dev/audio first.
 */

#include <errno.h>
#include <sched.h>
#include <string.h>
#include <lunix/kernel/addressspace.h>
#include <lunix/kernel/audio.h>
#include <lunix/kernel/clock.h>
#include <lunix/kernel/hda.h>
#include <lunix/kernel/log.h>
#include <lunix/kernel/panic.h>
#include <lunix/kernel/pci.h>
#include <lunix/kernel/physicalmemory.h>
#include <lunix/kernel/thread.h>

#define PCI_COMMAND_IO 0x1
#define PCI_COMMAND_MEMORY 0x2
#define PCI_COMMAND_BUS_MASTER 0x4

#define HDA_REG_GCAP 0x00
#define HDA_REG_GCTL 0x08
#define HDA_REG_STATESTS 0x0E
#define HDA_REG_INTCTL 0x20
#define HDA_REG_INTSTS 0x24
#define HDA_REG_IC 0x60
#define HDA_REG_IR 0x64
#define HDA_REG_IRS 0x68

#define HDA_REG_SD_BASE 0x80
#define HDA_SD_STRIDE 0x20
#define HDA_REG_SD_CTL 0x00
#define HDA_REG_SD_STS 0x03
#define HDA_REG_SD_LPIB 0x04
#define HDA_REG_SD_CBL 0x08
#define HDA_REG_SD_LVI 0x0C
#define HDA_REG_SD_FIFOSIZE 0x10
#define HDA_REG_SD_FORMAT 0x12
#define HDA_REG_SD_BDLPL 0x18
#define HDA_REG_SD_BDLPU 0x1C

#define HDA_GCTL_RESET (1 << 0)

#define HDA_IRS_VALID (1 << 1)
#define HDA_IRS_BUSY (1 << 0)

#define HDA_SD_CTL_SRST 0x01
#define HDA_SD_CTL_RUN 0x02
#define HDA_SD_CTL_STREAM_TAG_SHIFT 20
#define HDA_SD_CTL_STREAM_TAG_MASK (0xF << HDA_SD_CTL_STREAM_TAG_SHIFT)

#define HDA_SD_INT_MASK 0x1C

#define HDA_INT_CTRL_EN 0x40000000U
#define HDA_INT_GLOBAL_EN 0x80000000U

#define HDA_GCAP_ISS_MASK 0x0F00
#define HDA_GCAP_OSS_MASK 0xF000

#define HDA_MAX_CODECS 8
#define HDA_STATESTS_CODEC_MASK ((1U << HDA_MAX_CODECS) - 1)

#define HDA_VERB_GET_PARAMETERS 0x0F00
#define HDA_VERB_GET_CONNECT_LIST 0x0F02
#define HDA_VERB_GET_PIN_SENSE 0x0F09
#define HDA_VERB_GET_CONFIG_DEFAULT 0x0F1C
#define HDA_VERB_SET_STREAM_FORMAT 0x0200
#define HDA_VERB_SET_AMP_GAIN_MUTE 0x0300
#define HDA_VERB_SET_CONNECT_SEL 0x0701
#define HDA_VERB_SET_POWER_STATE 0x0705
#define HDA_VERB_SET_CHANNEL_STREAMID 0x0706
#define HDA_VERB_SET_PIN_WIDGET_CONTROL 0x0707
#define HDA_VERB_SET_PIN_SENSE 0x0709
#define HDA_VERB_SET_EAPD_BTLENABLE 0x070C

#define HDA_PAR_NODE_COUNT 0x04
#define HDA_PAR_FUNCTION_TYPE 0x05
#define HDA_PAR_AUDIO_WIDGET_CAP 0x09
#define HDA_PAR_PIN_CAP 0x0C
#define HDA_PAR_AMP_IN_CAP 0x0D
#define HDA_PAR_CONNLIST_LEN 0x0E
#define HDA_PAR_AMP_OUT_CAP 0x12

#define HDA_GRP_AUDIO_FUNCTION 0x01

#define HDA_WCAP_IN_AMP (1 << 1)
#define HDA_WCAP_OUT_AMP (1 << 2)
#define HDA_WCAP_AMP_OVRD (1 << 3)
#define HDA_WCAP_CONN_LIST (1 << 8)
#define HDA_WCAP_DIGITAL (1 << 9)
#define HDA_WCAP_TYPE_SHIFT 20
#define HDA_WCAP_TYPE_MASK (0xF << HDA_WCAP_TYPE_SHIFT)

#define HDA_WID_AUD_OUT 0x0
#define HDA_WID_AUD_MIX 0x2
#define HDA_WID_AUD_SEL 0x3
#define HDA_WID_PIN 0x4

#define HDA_PINCAP_TRIG_REQ (1 << 1)
#define HDA_PINCAP_PRES_DETECT (1 << 2)
#define HDA_PINCAP_OUT (1 << 4)
#define HDA_PINCAP_HP_DRV (1 << 3)

#define HDA_PINSENSE_PRESENCE (1U << 31)

#define HDA_CLIST_LENGTH 0x7F
#define HDA_CLIST_LONG (1 << 7)

#define HDA_PWR_D0 0x00

#define HDA_PINCTL_OUT_EN (1 << 6)
#define HDA_PINCTL_HP_EN (1 << 7)
#define HDA_PIN_OUT HDA_PINCTL_OUT_EN
#define HDA_PIN_HP (HDA_PINCTL_OUT_EN | HDA_PINCTL_HP_EN)

#define HDA_EAPD_ENABLE (1 << 1)

#define HDA_AMPCAP_OFFSET_MASK (0x7F << 0)
#define HDA_AMPCAP_OFFSET_SHIFT 0
#define HDA_AMPCAP_NUM_STEPS_MASK (0x7F << 8)
#define HDA_AMPCAP_NUM_STEPS_SHIFT 8

#define HDA_AMP_IN_UNMUTE_BASE 0x7000
#define HDA_AMP_OUT_UNMUTE_BASE 0xB000

#define HDA_DEFCFG_DEVICE_SHIFT 20
#define HDA_DEFCFG_DEVICE_MASK (0xF << HDA_DEFCFG_DEVICE_SHIFT)
#define HDA_DEFCFG_PORT_CONN_SHIFT 30
#define HDA_DEFCFG_PORT_CONN_MASK (0x3U << HDA_DEFCFG_PORT_CONN_SHIFT)

#define HDA_JACK_LINE_OUT 0x0
#define HDA_JACK_SPEAKER 0x1
#define HDA_JACK_HP_OUT 0x2
#define HDA_JACK_SPDIF_OUT 0x4
#define HDA_JACK_DIG_OTHER_OUT 0x5

#define HDA_JACK_PORT_COMPLEX 0x0
#define HDA_JACK_PORT_NONE 0x1
#define HDA_JACK_PORT_FIXED 0x2
#define HDA_JACK_PORT_BOTH 0x3

#define HDA_STREAM_FORMAT_BITS_16 (1 << 4)
#define HDA_STREAM_FORMAT_CHAN_SHIFT 0
#define HDA_STREAM_FORMAT_DIV_SHIFT 8
#define HDA_STREAM_FORMAT_MULT_SHIFT 11
#define HDA_STREAM_FORMAT_BASE_48K (0 << 14)
#define HDA_STREAM_FORMAT_BASE_44K (1 << 14)

#define HDA_PERIOD_BYTES PAGESIZE
#define HDA_PERIODS 4
#define HDA_BUFFER_BYTES (HDA_PERIOD_BYTES * HDA_PERIODS)
#define HDA_POLL_NS 1000000L
#define HDA_MAX_CONNECTIONS 32
#define HDA_MAX_PATH 16

struct HdaBdlEntry {
    uint32_t addressLow;
    uint32_t addressHigh;
    uint32_t length;
    uint32_t flags;
};

struct HdaWidget {
    bool present;
    uint8_t type;
    bool digital;
    bool jackPresent;
    bool jackPresentKnown;
    uint32_t caps;
    uint32_t pinCaps;
    uint32_t configDefault;
    uint8_t connectionCount;
    uint8_t connections[HDA_MAX_CONNECTIONS];
};

class HdaController : public AudioOutput {
public:
    friend void refreshActiveControllerSelection();

    HdaController(vaddr_t mmioBase, uint8_t bus, uint8_t device,
            uint8_t function);
    ~HdaController() = default;
    NOT_COPYABLE(HdaController);
    NOT_MOVABLE(HdaController);

    const audio_format& getFormat() const override;
    bool setFormat(const audio_format& format) override;
    void onAudioDataAvailable() override;
    void pumpPlayback();
public:
    HdaController* nextController;
private:
    bool configureCodec();
    bool configurePath();
    bool execImmediateCommand(uint32_t command, uint32_t* response);
    bool findBestOutputPath(bool allowDigital);
    bool findFallbackOutputPath(bool allowDigital);
    bool findPathFrom(uint8_t nid, bool allowDigital, bool visited[],
            size_t depth);
    uint16_t ampInputUnmuteValue(uint8_t nid, uint8_t index);
    uint16_t ampOutputUnmuteValue(uint8_t nid);
    bool getConnections(uint8_t nid, uint8_t* connections,
            uint8_t& count);
    bool getParameter(uint8_t codecAddress, uint8_t nid, uint8_t parameter,
            uint32_t& response);
    bool initializeDmaStream();
    bool initializeController();
    bool initializeStreamSelection();
    bool isSupportedFormat(const audio_format& format) const;
    int pinScore(const HdaWidget& widget) const;
    bool programCurrentFormat();
    bool probeCodec(uint8_t codecAddress);
    bool queryAmpCaps(uint8_t nid, bool output, uint32_t& caps);
public:
    void refreshOutputRouting();
private:
    bool refreshPinPresence(uint8_t nid);
    bool selectOutputPath();
    uint32_t read32(size_t offset) const;
    uint16_t read16(size_t offset) const;
    uint8_t read8(size_t offset) const;
    uint8_t readCodecCommand8(uint8_t nid, uint16_t verb) const;
    uint32_t currentLpiB() const;
    uint16_t streamFormatValue() const;
    size_t streamBase() const;
    bool waitForCodecPresence(uint16_t& state);
    bool waitForControllerReset(bool asserted);
    bool waitForImmediateResponse();
    bool waitForStreamReset(bool asserted);
    uint8_t currentHardwarePeriod() const;
    size_t refillPeriod(size_t index, bool fillSilence);
    void opportunisticPrimePlayback();
    void write32(size_t offset, uint32_t value);
    void write16(size_t offset, uint16_t value);
    void write8(size_t offset, uint8_t value);
    bool writeVerb(uint8_t nid, uint16_t verb, uint16_t parameter);
private:
    vaddr_t mmioBase;
    uint8_t bus;
    uint8_t device;
    uint8_t function;
    uint8_t codecAddress;
    uint8_t afgNode;
    uint8_t pinNode;
    uint8_t converterNode;
    uint8_t pathNodes[HDA_MAX_PATH];
    uint8_t pathSelectors[HDA_MAX_PATH];
    size_t pathLength;
    HdaWidget widgets[256];
    HdaBdlEntry* bdl;
    paddr_t bdlPhys;
    vaddr_t periodVirt[HDA_PERIODS];
    paddr_t periodPhys[HDA_PERIODS];
    unsigned int outputStreamIndex;
    unsigned int outputStreamTag;
    bool playbackStarted;
    bool attachedToAudioDevice;
    kthread_mutex_t mutex;
    int outputScore;
    bool periodContainsAudio[HDA_PERIODS];
    uint32_t lastLpiB;
    uint32_t pendingLpiB;
    uint64_t completedPeriods;
    uint64_t filledPeriods;
    audio_format format;
};

static HdaController* firstController;
static HdaController* activeController;
static bool hdaPollThreadStarted;

void refreshActiveControllerSelection();

static void sleepMilliseconds(unsigned int milliseconds) {
    Clock* clock = Clock::get(CLOCK_MONOTONIC);
    if (!clock) {
        sched_yield();
        return;
    }

    struct timespec duration;
    duration.tv_sec = milliseconds / 1000;
    duration.tv_nsec = (milliseconds % 1000) * 1000000L;
    clock->nanosleep(0, &duration, nullptr);
}

static NORETURN void hdaPollLoop() {
    Clock* clock = Clock::get(CLOCK_MONOTONIC);
    struct timespec delay = { .tv_sec = 0, .tv_nsec = HDA_POLL_NS };
    unsigned int routingTicks = 0;

    while (true) {
        for (HdaController* controller = firstController; controller;
                controller = controller->nextController) {
            controller->pumpPlayback();
        }

        if (++routingTicks >= 250) {
            routingTicks = 0;
            for (HdaController* controller = firstController; controller;
                    controller = controller->nextController) {
                controller->refreshOutputRouting();
            }
            refreshActiveControllerSelection();
        }

        if (clock) {
            clock->nanosleep(0, &delay, nullptr);
        } else {
            sched_yield();
        }
    }
}

static void startHdaPollThread(void*) {
    hdaPollLoop();
}

static void ensureHdaPollThread() {
    if (hdaPollThreadStarted) return;
    hdaPollThreadStarted = true;
    Thread::createKernelThread(startHdaPollThread, nullptr);
}

void refreshActiveControllerSelection() {
    if (!audioDevice) return;

    HdaController* best = nullptr;
    int bestScore = -1;
    for (HdaController* controller = firstController; controller;
            controller = controller->nextController) {
        if (controller->outputScore <= bestScore) continue;
        best = controller;
        bestScore = controller->outputScore;
    }

    if (activeController == best) return;

    if (activeController && activeController->attachedToAudioDevice) {
        audioDevice->detachOutput(activeController);
        activeController->attachedToAudioDevice = false;
    }

    activeController = best;
    if (activeController && !activeController->attachedToAudioDevice) {
        audioDevice->attachOutput(activeController);
        activeController->attachedToAudioDevice = true;
        Log::printf("HDA: using controller 0x%X pin 0x%X (score %d)\n",
                (unsigned int) activeController->mmioBase,
                activeController->pinNode, activeController->outputScore);
    }
}

void Hda::initialize(uint8_t bus, uint8_t device, uint8_t function) {
    uint32_t command = Pci::readConfig(bus, device, function,
            offsetof(PciHeader, command));
    command |= PCI_COMMAND_MEMORY | PCI_COMMAND_BUS_MASTER;
    command &= ~PCI_COMMAND_IO;
    Pci::writeConfig(bus, device, function, offsetof(PciHeader, command),
            command);

    uint64_t bar = Pci::readConfig(bus, device, function,
            offsetof(PciHeader, bar0));
    if (!(bar & 0x1) && (bar & 0x6) == 0x4) {
        uint64_t upper = Pci::readConfig(bus, device, function,
                offsetof(PciHeader, bar1));
        bar |= upper << 32;
    }

    paddr_t mmioPhys = (paddr_t) (bar & ~0xFULL);
    if (!mmioPhys) return;

    vaddr_t mmioBase = kernelSpace->mapPhysical(mmioPhys, 4 * PAGESIZE,
            PROT_READ | PROT_WRITE);
    if (!mmioBase) PANIC("Failed to map HDA registers");

    HdaController* controller = xnew HdaController(mmioBase, bus, device,
            function);
    if (!controller->nextController && firstController != controller) {
        return;
    }
    refreshActiveControllerSelection();
    ensureHdaPollThread();
}

HdaController::HdaController(vaddr_t mmioBase, uint8_t bus, uint8_t device,
        uint8_t function) : nextController(nullptr), mmioBase(mmioBase),
        bus(bus), device(device), function(function), codecAddress(0),
        afgNode(0), pinNode(0),
        converterNode(0), pathNodes(), pathSelectors(), pathLength(0),
        widgets(), bdl(nullptr), bdlPhys(0), periodVirt(), periodPhys(),
        outputStreamIndex(0), outputStreamTag(1), playbackStarted(false),
        attachedToAudioDevice(false), mutex(KTHREAD_MUTEX_INITIALIZER),
        outputScore(-1), periodContainsAudio(), lastLpiB(0),
        pendingLpiB(0), completedPeriods(0), filledPeriods(0), format() {
    format.sample_rate = 48000;
    format.channels = 2;
    format.encoding = AUDIO_ENCODING_S16_LE;
    format.bits_per_sample = 16;
    format.frame_size = 4;

    if (!initializeController()) {
        return;
    }

    nextController = firstController;
    firstController = this;
    Log::printf("HDA controller initialized at 0x%X, codec %u, pin 0x%X, "
            "converter 0x%X\n", (unsigned int) mmioBase, codecAddress,
            pinNode, converterNode);
}

const audio_format& HdaController::getFormat() const {
    return format;
}

bool HdaController::setFormat(const audio_format& requestedFormat) {
    if (!isSupportedFormat(requestedFormat)) {
        return false;
    }

    AutoLock lock(&mutex);
    if (format.sample_rate == requestedFormat.sample_rate &&
            format.channels == requestedFormat.channels &&
            format.encoding == requestedFormat.encoding &&
            format.bits_per_sample == requestedFormat.bits_per_sample &&
            format.frame_size == requestedFormat.frame_size) {
        return true;
    }

    audio_format oldFormat = format;
    format = requestedFormat;
    if (configurePath() && initializeDmaStream()) {
        return true;
    }

    format = oldFormat;
    configurePath();
    initializeDmaStream();
    return false;
}

void HdaController::onAudioDataAvailable() {
    AutoLock lock(&mutex);
    opportunisticPrimePlayback();
}

uint32_t HdaController::read32(size_t offset) const {
    return *(volatile uint32_t*) (mmioBase + offset);
}

uint16_t HdaController::read16(size_t offset) const {
    return *(volatile uint16_t*) (mmioBase + offset);
}

uint8_t HdaController::read8(size_t offset) const {
    return *(volatile uint8_t*) (mmioBase + offset);
}

void HdaController::write32(size_t offset, uint32_t value) {
    *(volatile uint32_t*) (mmioBase + offset) = value;
}

void HdaController::write16(size_t offset, uint16_t value) {
    *(volatile uint16_t*) (mmioBase + offset) = value;
}

void HdaController::write8(size_t offset, uint8_t value) {
    *(volatile uint8_t*) (mmioBase + offset) = value;
}

size_t HdaController::streamBase() const {
    return HDA_REG_SD_BASE + outputStreamIndex * HDA_SD_STRIDE;
}

bool HdaController::waitForControllerReset(bool asserted) {
    for (int i = 0; i < 100; i++) {
        bool state = !!(read8(HDA_REG_GCTL) & HDA_GCTL_RESET);
        if (state == asserted) return true;
        sleepMilliseconds(1);
    }
    return false;
}

bool HdaController::waitForCodecPresence(uint16_t& state) {
    for (int i = 0; i < 200; i++) {
        state = read16(HDA_REG_STATESTS) & HDA_STATESTS_CODEC_MASK;
        if (state) return true;
        sleepMilliseconds(1);
    }
    state = 0;
    return false;
}

bool HdaController::waitForImmediateResponse() {
    for (int i = 0; i < 100; i++) {
        uint16_t irs = read16(HDA_REG_IRS);
        if (irs & HDA_IRS_VALID) return true;
        sleepMilliseconds(1);
    }
    return false;
}

bool HdaController::waitForStreamReset(bool asserted) {
    size_t base = streamBase();
    for (int i = 0; i < 100; i++) {
        bool state = !!(read8(base + HDA_REG_SD_CTL) & HDA_SD_CTL_SRST);
        if (state == asserted) return true;
        sleepMilliseconds(1);
    }
    return false;
}

uint32_t HdaController::currentLpiB() const {
    uint32_t lpib = read32(streamBase() + HDA_REG_SD_LPIB);
    if (lpib >= HDA_BUFFER_BYTES) {
        lpib %= HDA_BUFFER_BYTES;
    }
    return lpib;
}

uint8_t HdaController::currentHardwarePeriod() const {
    uint32_t lpib = currentLpiB();
    return lpib / HDA_PERIOD_BYTES;
}

size_t HdaController::refillPeriod(size_t index, bool fillSilence) {
    size_t copied = 0;
    if (audioDevice) {
        copied = audioDevice->dequeuePlaybackData((void*) periodVirt[index],
                HDA_PERIOD_BYTES, fillSilence);
    } else if (fillSilence) {
        memset((void*) periodVirt[index], 0, HDA_PERIOD_BYTES);
    }

    periodContainsAudio[index] = copied > 0;
    return copied;
}

void HdaController::opportunisticPrimePlayback() {
    if (!playbackStarted || !audioDevice) return;

    uint8_t hardwarePeriod = currentHardwarePeriod();
    for (size_t offset = 1; offset < HDA_PERIODS; offset++) {
        size_t index = (hardwarePeriod + offset) % HDA_PERIODS;
        if (periodContainsAudio[index]) continue;
        if (refillPeriod(index, false) == 0) break;
    }
}

bool HdaController::execImmediateCommand(uint32_t command, uint32_t* response) {
    for (int i = 0; i < 100; i++) {
        if (!(read16(HDA_REG_IRS) & HDA_IRS_BUSY)) break;
        sleepMilliseconds(1);
    }
    if (read16(HDA_REG_IRS) & HDA_IRS_BUSY) return false;

    write16(HDA_REG_IRS, read16(HDA_REG_IRS) | HDA_IRS_VALID);
    write32(HDA_REG_IC, command);
    write16(HDA_REG_IRS, read16(HDA_REG_IRS) | HDA_IRS_BUSY);

    if (!waitForImmediateResponse()) return false;
    if (response) *response = read32(HDA_REG_IR);
    write16(HDA_REG_IRS, read16(HDA_REG_IRS) | HDA_IRS_VALID);
    return true;
}

bool HdaController::writeVerb(uint8_t nid, uint16_t verb, uint16_t parameter) {
    uint32_t unused;
    uint32_t command = ((uint32_t) codecAddress << 28) |
            ((uint32_t) nid << 20) | ((uint32_t) verb << 8) | parameter;
    return execImmediateCommand(command, &unused);
}

bool HdaController::getParameter(uint8_t codecAddress, uint8_t nid,
        uint8_t parameter, uint32_t& response) {
    uint32_t command = ((uint32_t) codecAddress << 28) |
            ((uint32_t) nid << 20) | ((uint32_t) HDA_VERB_GET_PARAMETERS << 8) |
            parameter;
    return execImmediateCommand(command, &response);
}

bool HdaController::queryAmpCaps(uint8_t nid, bool output, uint32_t& caps) {
    const HdaWidget& widget = widgets[nid];
    uint32_t ampFlag = output ? HDA_WCAP_OUT_AMP : HDA_WCAP_IN_AMP;
    if (!(widget.caps & ampFlag)) return false;

    uint8_t ampNid = nid;
    if (!(widget.caps & HDA_WCAP_AMP_OVRD)) ampNid = afgNode;
    return getParameter(codecAddress, ampNid,
            output ? HDA_PAR_AMP_OUT_CAP : HDA_PAR_AMP_IN_CAP, caps);
}

uint16_t HdaController::ampInputUnmuteValue(uint8_t nid, uint8_t index) {
    uint32_t caps;
    unsigned int gain = 0;
    if (queryAmpCaps(nid, false, caps)) {
        gain = (caps & HDA_AMPCAP_OFFSET_MASK) >> HDA_AMPCAP_OFFSET_SHIFT;
        unsigned int steps = (caps & HDA_AMPCAP_NUM_STEPS_MASK) >>
                HDA_AMPCAP_NUM_STEPS_SHIFT;
        if (gain > steps) gain = steps;
    }

    return HDA_AMP_IN_UNMUTE_BASE | ((uint16_t) index << 8) | gain;
}

uint16_t HdaController::ampOutputUnmuteValue(uint8_t nid) {
    uint32_t caps;
    unsigned int gain = 0;
    if (queryAmpCaps(nid, true, caps)) {
        gain = (caps & HDA_AMPCAP_OFFSET_MASK) >> HDA_AMPCAP_OFFSET_SHIFT;
        unsigned int steps = (caps & HDA_AMPCAP_NUM_STEPS_MASK) >>
                HDA_AMPCAP_NUM_STEPS_SHIFT;
        if (gain > steps) gain = steps;
    }

    return HDA_AMP_OUT_UNMUTE_BASE | gain;
}

uint8_t HdaController::readCodecCommand8(uint8_t nid, uint16_t verb) const {
    (void) nid;
    (void) verb;
    return 0;
}

bool HdaController::getConnections(uint8_t nid, uint8_t* connections,
        uint8_t& count) {
    count = 0;
    const HdaWidget& widget = widgets[nid];
    if (!widget.present || !(widget.caps & HDA_WCAP_CONN_LIST)) return false;

    uint32_t parameter = widget.connectionCount;
    if (parameter == 0) return false;

    bool longForm = !!(parameter & HDA_CLIST_LONG);
    unsigned int shift = longForm ? 16 : 8;
    unsigned int numElems = longForm ? 2 : 4;
    unsigned int mask = (1U << (shift - 1)) - 1;
    unsigned int connLen = parameter & HDA_CLIST_LENGTH;
    if (!connLen) return false;

    uint8_t previous = 0;
    for (unsigned int i = 0; i < connLen; i++) {
        uint32_t value;
        if (!execImmediateCommand(((uint32_t) codecAddress << 28) |
                ((uint32_t) nid << 20) |
                ((uint32_t) HDA_VERB_GET_CONNECT_LIST << 8) |
                (i - (i % numElems)), &value)) {
            return false;
        }

        value >>= (i % numElems) * shift;
        bool range = !!(value & (1U << (shift - 1)));
        uint8_t connection = value & mask;

        if (range) {
            if (!previous || previous >= connection) continue;
            for (uint8_t n = previous + 1; n <= connection; n++) {
                if (count >= HDA_MAX_CONNECTIONS) return true;
                connections[count++] = n;
            }
        } else {
            if (count >= HDA_MAX_CONNECTIONS) return true;
            connections[count++] = connection;
        }
        previous = connection;
    }

    return count > 0;
}

int HdaController::pinScore(const HdaWidget& widget) const {
    if (((widget.configDefault & HDA_DEFCFG_PORT_CONN_MASK) >>
            HDA_DEFCFG_PORT_CONN_SHIFT) == HDA_JACK_PORT_NONE) {
        return -1;
    }

    int score = widget.digital ? 0 : 100;
    switch ((widget.configDefault & HDA_DEFCFG_DEVICE_MASK) >>
            HDA_DEFCFG_DEVICE_SHIFT) {
    case HDA_JACK_SPEAKER: score += 60; break;
    case HDA_JACK_HP_OUT: score += 50; break;
    case HDA_JACK_LINE_OUT: score += 40; break;
    case HDA_JACK_SPDIF_OUT:
    case HDA_JACK_DIG_OTHER_OUT:
        score += widget.digital ? 30 : -100;
        break;
    default:
        score += widget.digital ? 10 : 20;
        break;
    }

    uint32_t portConnection = (widget.configDefault & HDA_DEFCFG_PORT_CONN_MASK)
            >> HDA_DEFCFG_PORT_CONN_SHIFT;
    switch (portConnection) {
    case HDA_JACK_PORT_FIXED: score += 10; break;
    case HDA_JACK_PORT_BOTH: score += 5; break;
    default: break;
    }

    if (widget.pinCaps & HDA_PINCAP_PRES_DETECT) {
        if (widget.jackPresentKnown && widget.jackPresent) {
            score += 400;
        } else if (portConnection != HDA_JACK_PORT_FIXED) {
            score -= 200;
        }
    }

    return score;
}

bool HdaController::findPathFrom(uint8_t nid, bool allowDigital,
        bool visited[], size_t depth) {
    if (depth >= HDA_MAX_PATH || visited[nid] || !widgets[nid].present) {
        return false;
    }

    const HdaWidget& widget = widgets[nid];
    if (!allowDigital && widget.digital) return false;

    visited[nid] = true;
    if (widget.type == HDA_WID_AUD_OUT) {
        pathNodes[depth] = nid;
        pathSelectors[depth] = 0xFF;
        pathLength = depth + 1;
        visited[nid] = false;
        return true;
    }

    for (uint8_t i = 0; i < widget.connectionCount; i++) {
        uint8_t connection = widget.connections[i];
        if (!widgets[connection].present) continue;
        if (findPathFrom(connection, allowDigital, visited, depth + 1)) {
            pathNodes[depth] = nid;
            pathSelectors[depth] = i;
            visited[nid] = false;
            return true;
        }
    }

    visited[nid] = false;
    return false;
}

bool HdaController::findBestOutputPath(bool allowDigital) {
    int bestScore = -1;
    uint8_t bestPin = 0;
    uint8_t bestPathNodes[HDA_MAX_PATH] = {};
    uint8_t bestPathSelectors[HDA_MAX_PATH] = {};
    size_t bestPathLength = 0;

    for (unsigned int nid = 0; nid < 256; nid++) {
        const HdaWidget& widget = widgets[nid];
        if (!widget.present || widget.type != HDA_WID_PIN) continue;
        if (!(widget.pinCaps & HDA_PINCAP_OUT)) continue;

        int score = pinScore(widget);
        if (score < 0) continue;
        if (!allowDigital && widget.digital) continue;

        bool visited[256] = {};
        if (!findPathFrom((uint8_t) nid, allowDigital, visited, 0)) continue;
        if (score <= bestScore) continue;

        bestScore = score;
        bestPin = (uint8_t) nid;
        bestPathLength = pathLength;
        memcpy(bestPathNodes, pathNodes, sizeof(bestPathNodes));
        memcpy(bestPathSelectors, pathSelectors, sizeof(bestPathSelectors));
    }

    if (bestScore < 0) return false;

    pinNode = bestPin;
    pathLength = bestPathLength;
    memcpy(pathNodes, bestPathNodes, sizeof(pathNodes));
    memcpy(pathSelectors, bestPathSelectors, sizeof(pathSelectors));
    converterNode = pathNodes[pathLength - 1];
    outputScore = bestScore;
    return true;
}

bool HdaController::findFallbackOutputPath(bool allowDigital) {
    int bestScore = -1;
    uint8_t bestPin = 0;
    uint8_t bestConverter = 0;
    uint8_t bestPathNodes[HDA_MAX_PATH] = {};
    uint8_t bestPathSelectors[HDA_MAX_PATH];
    memset(bestPathSelectors, 0xFF, sizeof(bestPathSelectors));
    size_t bestPathLength = 0;

    for (unsigned int pin = 0; pin < 256; pin++) {
        const HdaWidget& pinWidget = widgets[pin];
        if (!pinWidget.present || pinWidget.type != HDA_WID_PIN) continue;
        if (!(pinWidget.pinCaps & HDA_PINCAP_OUT)) continue;
        if (!allowDigital && pinWidget.digital) continue;

        int score = pinScore(pinWidget);
        if (score < 0) continue;

        for (unsigned int converter = 0; converter < 256; converter++) {
            const HdaWidget& converterWidget = widgets[converter];
            if (!converterWidget.present ||
                    converterWidget.type != HDA_WID_AUD_OUT) {
                continue;
            }
            if (!allowDigital && converterWidget.digital) continue;

            uint8_t pathNodesCandidate[HDA_MAX_PATH] = {};
            uint8_t pathSelectorsCandidate[HDA_MAX_PATH];
            memset(pathSelectorsCandidate, 0xFF,
                    sizeof(pathSelectorsCandidate));
            size_t pathLengthCandidate = 2;
            int pairScore = score;

            pathNodesCandidate[0] = pin;
            pathNodesCandidate[1] = converter;
            if (!converterWidget.digital) pairScore += 20;
            if (converterWidget.digital == pinWidget.digital) pairScore += 40;

            bool foundExplicitRoute = false;
            for (uint8_t i = 0; i < pinWidget.connectionCount; i++) {
                uint8_t connection = pinWidget.connections[i];
                if (connection == converter) {
                    pathSelectorsCandidate[0] = i;
                    pairScore += 200;
                    foundExplicitRoute = true;
                    break;
                }

                const HdaWidget& midWidget = widgets[connection];
                if (!midWidget.present) continue;
                if (!allowDigital && midWidget.digital) continue;

                for (uint8_t j = 0; j < midWidget.connectionCount; j++) {
                    if (midWidget.connections[j] != converter) continue;
                    pathNodesCandidate[1] = connection;
                    pathNodesCandidate[2] = converter;
                    pathSelectorsCandidate[0] = i;
                    pathSelectorsCandidate[1] = j;
                    pathLengthCandidate = 3;
                    pairScore += 160;
                    foundExplicitRoute = true;
                    break;
                }

                if (foundExplicitRoute) break;
            }

            if (!foundExplicitRoute && pinWidget.connectionCount == 0) {
                pairScore += 30;
            }

            if (pairScore <= bestScore) continue;
            bestScore = pairScore;
            bestPin = pin;
            bestConverter = converter;
            bestPathLength = pathLengthCandidate;
            memcpy(bestPathNodes, pathNodesCandidate, sizeof(bestPathNodes));
            memcpy(bestPathSelectors, pathSelectorsCandidate,
                    sizeof(bestPathSelectors));
        }
    }

    if (bestScore < 0) return false;

    pinNode = bestPin;
    converterNode = bestConverter;
    pathLength = bestPathLength;
    memcpy(pathNodes, bestPathNodes, sizeof(pathNodes));
    memcpy(pathSelectors, bestPathSelectors, sizeof(pathSelectors));
    outputScore = bestScore;
    return true;
}

bool HdaController::refreshPinPresence(uint8_t nid) {
    HdaWidget& widget = widgets[nid];
    if (!widget.present || widget.type != HDA_WID_PIN) return false;
    if (!(widget.pinCaps & HDA_PINCAP_PRES_DETECT)) {
        widget.jackPresentKnown = false;
        widget.jackPresent = false;
        return false;
    }

    if (widget.pinCaps & HDA_PINCAP_TRIG_REQ) {
        writeVerb(nid, HDA_VERB_SET_PIN_SENSE, 0);
    }

    uint32_t response;
    uint32_t command = ((uint32_t) codecAddress << 28) |
            ((uint32_t) nid << 20) | ((uint32_t) HDA_VERB_GET_PIN_SENSE << 8);
    if (!execImmediateCommand(command, &response)) {
        widget.jackPresentKnown = false;
        widget.jackPresent = false;
        return false;
    }

    widget.jackPresentKnown = true;
    widget.jackPresent = !!(response & HDA_PINSENSE_PRESENCE);
    return true;
}

bool HdaController::selectOutputPath() {
    outputScore = -1;
    if (findBestOutputPath(false) || findBestOutputPath(true)) {
        return true;
    }

    return findFallbackOutputPath(false) || findFallbackOutputPath(true);
}

void HdaController::refreshOutputRouting() {
    AutoLock lock(&mutex);

    uint8_t oldPin = pinNode;
    uint8_t oldPathNodes[HDA_MAX_PATH];
    uint8_t oldPathSelectors[HDA_MAX_PATH];
    size_t oldPathLength = pathLength;
    int oldScore = outputScore;
    memcpy(oldPathNodes, pathNodes, sizeof(pathNodes));
    memcpy(oldPathSelectors, pathSelectors, sizeof(pathSelectors));

    for (unsigned int nid = 0; nid < 256; nid++) {
        refreshPinPresence((uint8_t) nid);
    }

    if (!selectOutputPath()) {
        pinNode = oldPin;
        pathLength = oldPathLength;
        outputScore = oldScore;
        memcpy(pathNodes, oldPathNodes, sizeof(pathNodes));
        memcpy(pathSelectors, oldPathSelectors, sizeof(pathSelectors));
        return;
    }

    bool pathChanged = oldPin != pinNode || oldPathLength != pathLength ||
            memcmp(oldPathNodes, pathNodes, sizeof(pathNodes)) != 0 ||
            memcmp(oldPathSelectors, pathSelectors,
                    sizeof(pathSelectors)) != 0;
    if (pathChanged && attachedToAudioDevice) {
        configurePath();
    }
}

bool HdaController::probeCodec(uint8_t codecAddress) {
    this->codecAddress = codecAddress;

    uint32_t rootNodes;
    if (!getParameter(codecAddress, 0, HDA_PAR_NODE_COUNT, rootNodes)) {
        return false;
    }

    uint8_t startNid = (rootNodes >> 16) & 0x7F;
    uint8_t totalNodes = rootNodes & 0x7F;
    uint8_t selectedAfg = 0;

    for (uint8_t i = 0; i < totalNodes; i++) {
        uint8_t nid = startNid + i;
        uint32_t functionType;
        if (!getParameter(codecAddress, nid, HDA_PAR_FUNCTION_TYPE,
                functionType)) {
            continue;
        }
        if ((functionType & 0xFF) == HDA_GRP_AUDIO_FUNCTION) {
            selectedAfg = nid;
            break;
        }
    }

    if (!selectedAfg) return false;

    uint32_t widgetNodes;
    if (!getParameter(codecAddress, selectedAfg, HDA_PAR_NODE_COUNT,
            widgetNodes)) {
        return false;
    }

    memset(widgets, 0, sizeof(widgets));
    uint8_t widgetStart = (widgetNodes >> 16) & 0x7F;
    uint8_t widgetCount = widgetNodes & 0x7F;
    if (!widgetStart || !widgetCount) return false;

    for (uint8_t i = 0; i < widgetCount; i++) {
        uint8_t nid = widgetStart + i;
        HdaWidget& widget = widgets[nid];
        uint32_t caps;
        if (!getParameter(codecAddress, nid, HDA_PAR_AUDIO_WIDGET_CAP, caps)) {
            continue;
        }
        widget.present = true;
        widget.caps = caps;
        widget.type = (caps & HDA_WCAP_TYPE_MASK) >> HDA_WCAP_TYPE_SHIFT;
        widget.digital = !!(caps & HDA_WCAP_DIGITAL);
        widget.jackPresent = false;
        widget.jackPresentKnown = false;
        widget.connectionCount = 0;

        if (widget.type == HDA_WID_PIN) {
            getParameter(codecAddress, nid, HDA_PAR_PIN_CAP, widget.pinCaps);
            uint32_t response;
            uint32_t command = ((uint32_t) codecAddress << 28) |
                    ((uint32_t) nid << 20) |
                    ((uint32_t) HDA_VERB_GET_CONFIG_DEFAULT << 8);
            if (execImmediateCommand(command, &response)) {
                widget.configDefault = response;
            }
        }

        if (caps & HDA_WCAP_CONN_LIST) {
            uint32_t connParameter;
            if (getParameter(codecAddress, nid, HDA_PAR_CONNLIST_LEN,
                    connParameter)) {
                widget.connectionCount = connParameter;
                getConnections(nid, widget.connections, widget.connectionCount);
            }
        }
    }

    this->afgNode = selectedAfg;
    for (unsigned int nid = 0; nid < 256; nid++) {
        refreshPinPresence((uint8_t) nid);
    }
    return selectOutputPath();
}

bool HdaController::configurePath() {
    uint16_t streamFormat = streamFormatValue();
    if (!streamFormat) return false;

    if (!writeVerb(afgNode, HDA_VERB_SET_POWER_STATE, HDA_PWR_D0)) {
        return false;
    }

    for (size_t i = 0; i < pathLength; i++) {
        uint8_t nid = pathNodes[i];
        const HdaWidget& widget = widgets[nid];

        writeVerb(nid, HDA_VERB_SET_POWER_STATE, HDA_PWR_D0);

        if (i + 1 < pathLength && pathSelectors[i] != 0xFF &&
                widget.connectionCount > 0 &&
                widget.type != HDA_WID_AUD_MIX) {
            writeVerb(nid, HDA_VERB_SET_CONNECT_SEL, pathSelectors[i]);
        }
        if (i + 1 < pathLength && pathSelectors[i] != 0xFF &&
                widget.connectionCount > 0 &&
                (widget.caps & HDA_WCAP_IN_AMP)) {
            writeVerb(nid, HDA_VERB_SET_AMP_GAIN_MUTE,
                    ampInputUnmuteValue(nid, pathSelectors[i]));
        }
        if (widget.caps & HDA_WCAP_OUT_AMP) {
            writeVerb(nid, HDA_VERB_SET_AMP_GAIN_MUTE,
                    ampOutputUnmuteValue(nid));
        }
    }

    uint16_t pinControl = (widgets[pinNode].pinCaps & HDA_PINCAP_HP_DRV) ?
            HDA_PIN_HP : HDA_PIN_OUT;
    writeVerb(pinNode, HDA_VERB_SET_PIN_WIDGET_CONTROL, pinControl);
    writeVerb(pinNode, HDA_VERB_SET_EAPD_BTLENABLE, HDA_EAPD_ENABLE);

    writeVerb(converterNode, HDA_VERB_SET_CHANNEL_STREAMID,
            (outputStreamTag << 4) | 0);
    sleepMilliseconds(1);
    return writeVerb(converterNode, HDA_VERB_SET_STREAM_FORMAT, streamFormat);
}

bool HdaController::configureCodec() {
    uint16_t state = 0;
    waitForCodecPresence(state);
    if (state) {
        write16(HDA_REG_STATESTS, state);
    } else {
        Log::printf("HDA: STATESTS empty, probing codec slots directly\n");
    }

    for (uint8_t codec = 0; codec < HDA_MAX_CODECS; codec++) {
        if (state && !(state & (1U << codec))) continue;
        if (probeCodec(codec)) {
            return configurePath();
        }
    }

    for (uint8_t codec = 0; codec < HDA_MAX_CODECS; codec++) {
        if (state & (1U << codec)) continue;
        if (probeCodec(codec)) {
            Log::printf("HDA: codec %u found via fallback probe\n", codec);
            return configurePath();
        }
    }

    return false;
}

bool HdaController::initializeStreamSelection() {
    outputStreamIndex = (read16(HDA_REG_GCAP) & HDA_GCAP_ISS_MASK) >> 8;
    unsigned int playbackStreams =
            (read16(HDA_REG_GCAP) & HDA_GCAP_OSS_MASK) >> 12;
    if (playbackStreams == 0) {
        Log::printf("HDA: controller exposes no playback streams\n");
        return false;
    }

    // Most Intel-style controllers use stream tags that match the stream
    // descriptor index + 1. Programming tag 1 unconditionally can leave the
    // DMA engine running while the codec stays disconnected and silent.
    outputStreamTag = outputStreamIndex + 1;
    return true;
}

bool HdaController::isSupportedFormat(const audio_format& requestedFormat) const {
    if (requestedFormat.channels != 2) return false;
    if (requestedFormat.encoding != AUDIO_ENCODING_S16_LE) return false;
    if (requestedFormat.bits_per_sample != 16) return false;
    if (requestedFormat.frame_size != 4) return false;
    return requestedFormat.sample_rate == 44100 ||
            requestedFormat.sample_rate == 48000;
}

uint16_t HdaController::streamFormatValue() const {
    if (!isSupportedFormat(format)) return 0;

    uint16_t value = HDA_STREAM_FORMAT_BITS_16;
    value |= (format.channels - 1) << HDA_STREAM_FORMAT_CHAN_SHIFT;
    if (format.sample_rate == 44100) {
        value |= HDA_STREAM_FORMAT_BASE_44K;
    } else if (format.sample_rate == 48000) {
        value |= HDA_STREAM_FORMAT_BASE_48K;
    } else {
        return 0;
    }

    value |= 0 << HDA_STREAM_FORMAT_MULT_SHIFT;
    value |= 0 << HDA_STREAM_FORMAT_DIV_SHIFT;
    return value;
}

bool HdaController::programCurrentFormat() {
    uint16_t streamFormat = streamFormatValue();
    if (!streamFormat) return false;

    size_t base = streamBase();
    write8(base + HDA_REG_SD_CTL, 0);
    write8(base + HDA_REG_SD_STS, HDA_SD_INT_MASK);

    write8(base + HDA_REG_SD_CTL, HDA_SD_CTL_SRST);
    if (!waitForStreamReset(true)) return false;
    write8(base + HDA_REG_SD_CTL, 0);
    if (!waitForStreamReset(false)) return false;

    uint32_t control = read32(base + HDA_REG_SD_CTL);
    control &= ~HDA_SD_CTL_STREAM_TAG_MASK;
    control |= outputStreamTag << HDA_SD_CTL_STREAM_TAG_SHIFT;
    write32(base + HDA_REG_SD_CTL, control);

    write32(base + HDA_REG_SD_CBL, HDA_BUFFER_BYTES);
    write16(base + HDA_REG_SD_FORMAT, streamFormat);
    for (int i = 0; i < 20; i++) {
        if (read16(base + HDA_REG_SD_FIFOSIZE)) break;
        sleepMilliseconds(1);
    }
    write16(base + HDA_REG_SD_LVI, HDA_PERIODS - 1);
    write32(base + HDA_REG_SD_BDLPL, bdlPhys);
    write32(base + HDA_REG_SD_BDLPU, 0);

    for (size_t i = 0; i < HDA_PERIODS; i++) {
        refillPeriod(i, true);
    }
    filledPeriods = HDA_PERIODS;
    completedPeriods = 0;
    lastLpiB = 0;
    pendingLpiB = 0;

    write8(base + HDA_REG_SD_STS, HDA_SD_INT_MASK);
    write32(HDA_REG_INTCTL, read32(HDA_REG_INTCTL) | HDA_INT_CTRL_EN |
            HDA_INT_GLOBAL_EN | (1U << outputStreamIndex));
    write8(base + HDA_REG_SD_CTL, read8(base + HDA_REG_SD_CTL) |
            HDA_SD_CTL_RUN | HDA_SD_INT_MASK);
    lastLpiB = currentLpiB();
    playbackStarted = true;
    return true;
}

bool HdaController::initializeDmaStream() {
    if (!initializeStreamSelection()) return false;

    if (!bdl) {
        bdlPhys = PhysicalMemory::popPageFrame32();
        if (!bdlPhys) PANIC("Failed to allocate HDA BDL");
        vaddr_t bdlVirt = kernelSpace->mapPhysical(bdlPhys, PAGESIZE,
                PROT_READ | PROT_WRITE);
        if (!bdlVirt) PANIC("Failed to map HDA BDL");
        memset((void*) bdlVirt, 0, PAGESIZE);
        bdl = (HdaBdlEntry*) bdlVirt;

        for (size_t i = 0; i < HDA_PERIODS; i++) {
            periodPhys[i] = PhysicalMemory::popPageFrame32();
            if (!periodPhys[i]) PANIC("Failed to allocate HDA DMA period");
            periodVirt[i] = kernelSpace->mapPhysical(periodPhys[i], PAGESIZE,
                    PROT_READ | PROT_WRITE);
            if (!periodVirt[i]) PANIC("Failed to map HDA DMA period");
            memset((void*) periodVirt[i], 0, PAGESIZE);
            bdl[i].addressLow = periodPhys[i];
            bdl[i].addressHigh = 0;
            bdl[i].length = HDA_PERIOD_BYTES;
            bdl[i].flags = 0;
        }
    }

    return programCurrentFormat();
}

bool HdaController::initializeController() {
    write32(HDA_REG_INTCTL, 0);
    write32(HDA_REG_INTSTS, 0xFFFFFFFFU);
    if (read8(HDA_REG_GCTL) & HDA_GCTL_RESET) {
        write16(HDA_REG_STATESTS, HDA_STATESTS_CODEC_MASK);
    }

    write32(HDA_REG_GCTL, read32(HDA_REG_GCTL) & ~HDA_GCTL_RESET);
    if (!waitForControllerReset(false)) {
        Log::printf("HDA reset assert timed out\n");
        return false;
    }
    sleepMilliseconds(1);

    write32(HDA_REG_GCTL, read32(HDA_REG_GCTL) | HDA_GCTL_RESET);
    if (!waitForControllerReset(true)) {
        Log::printf("HDA reset deassert timed out\n");
        return false;
    }
    sleepMilliseconds(2);

    if (!initializeStreamSelection()) {
        return false;
    }

    if (!configureCodec()) {
        Log::printf("HDA: no usable codec path found\n");
        return false;
    }
    if (!initializeDmaStream()) {
        Log::printf("HDA: failed to initialize DMA stream\n");
        return false;
    }
    return true;
}

void HdaController::pumpPlayback() {
    AutoLock lock(&mutex);
    if (!playbackStarted) return;

    uint32_t lpib = currentLpiB();
    uint32_t advancedBytes;
    if (lpib >= lastLpiB) {
        advancedBytes = lpib - lastLpiB;
    } else {
        advancedBytes = HDA_BUFFER_BYTES - lastLpiB + lpib;
    }
    lastLpiB = lpib;

    pendingLpiB += advancedBytes;
    if (pendingLpiB >= HDA_PERIOD_BYTES) {
        completedPeriods += pendingLpiB / HDA_PERIOD_BYTES;
        pendingLpiB %= HDA_PERIOD_BYTES;
    }

    while (filledPeriods < completedPeriods + HDA_PERIODS) {
        size_t index = filledPeriods % HDA_PERIODS;
        refillPeriod(index, true);
        filledPeriods++;
    }

    opportunisticPrimePlayback();
}
