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

/* kernel/include/lunix/kernel/polaris.h
 * AMD Polaris (GFX8/VI) GPU driver — internal kernel header.
 *
 * Additions over the original scaffold:
 *   - PolarisSdmaRing : per-engine ring buffer state
 *   - SDMA0 / SDMA1 bring-up, fence, fill/copy operations
 *   - gpuFillRect() / gpuCopyRect() hooks for the display driver
 */

#ifndef KERNEL_POLARIS_H
#define KERNEL_POLARIS_H

#include <lunix/kernel/display.h>
#include <lunix/kernel/kernel.h>
#include <lunix/kernel/kthread.h>

enum PolarisAsic {
    POLARIS10,
    POLARIS11,
    POLARIS12,
};

/* --------------------------------------------------------------------------
 * SDMA ring state (one per engine instance, VI has SDMA0 + SDMA1)
 * -------------------------------------------------------------------------- */

/* Ring buffer size in DWORDs (must be a power-of-two, 256 … 8 192). */
static constexpr size_t polarisSdmaRingDwords = 1024; /* 4 KiB */

struct PolarisSdmaRing {
    /* Ring buffer in VRAM aperture */
    paddr_t ringPhysical;        /* GPU-side physical address */
    vaddr_t ringVirtual;         /* kernel-side virtual address */

    /* CPU writeback location where the GPU writes the current RPTR */
    paddr_t rptrWbPhysical;
    volatile uint32_t* rptrWbVirtual;

    /* Fence location — GPU writes fence_seq here on FENCE packets */
    paddr_t fencePhysical;
    volatile uint32_t* fenceVirtual;
    uint32_t fenceSeq;           /* next fence value to emit */

    /* Write pointer tracked in software (in DWORDs, wraps at 2^32) */
    uint32_t wptr;

    bool initialized;
};

/* --------------------------------------------------------------------------
 * Forward declarations
 * -------------------------------------------------------------------------- */

struct PolarisBufferObject;
struct PolarisFirmwareBlob;
class  PolarisControlDevice;

/* --------------------------------------------------------------------------
 * PolarisDevice — main driver object
 * -------------------------------------------------------------------------- */

class PolarisDevice : public GraphicsDriver, public ConstructorMayFail {
public:
    PolarisDevice(PolarisAsic asicType, uint8_t bus, uint8_t device,
            uint8_t function);
    ~PolarisDevice() = default;
    NOT_COPYABLE(PolarisDevice);
    NOT_MOVABLE(PolarisDevice);
    friend class PolarisControlDevice;

    /* ---------- GraphicsDriver interface ---------- */
    bool isSupportedMode(video_mode mode) override;
    vaddr_t setVideoMode(video_mode* mode) override;

    /**
     * gpuFillRect — fill a rectangle in the current framebuffer with a
     * solid RGBA colour using an SDMA CONST_FILL packet.
     * Returns true on success, false if SDMA is not ready.
     */
    bool gpuFillRect(uint32_t rgbaColor,
                     unsigned x, unsigned y,
                     unsigned w, unsigned h) override;

    /**
     * gpuBlit — copy rectangle within framebuffer using SDMA.
     */
    bool gpuBlit(unsigned srcX, unsigned srcY,
                 unsigned dstX, unsigned dstY,
                 unsigned w, unsigned h) override;

    /* ---------- Static helpers ---------- */
    static void initialize(uint8_t bus, uint8_t device, uint8_t function,
            PolarisAsic asicType);
    static bool matchesDevice(uint16_t deviceId, PolarisAsic& asicType);

private:
    /* ---- PCI / BAR init ---- */
    bool enableDevice();
    bool initializeBars();

    /* ---- ATOM BIOS ---- */
    bool loadAtomBios();
    bool tryAdoptBootFramebuffer();

    /* ---- EDID ---- */

    /**
     * Primary path: read 128 bytes of EDID from the monitor via the DCE
     * hardware I2C controller (DDC bus on connector index 0 / DDC1).
     * Returns true and fills edidBuf[0..127] on success.
     */
    bool readEdidViaDdc(uint8_t* edidBuf);

    /**
     * Fallback path: scan the loaded VBIOS image for an embedded EDID
     * signature and copy the first valid 128-byte block found.
     * Returns true and fills edidBuf[0..127] on success.
     */
    bool findEdidInVbios(uint8_t* edidBuf);

    /**
     * Try to read one additional CEA-861 extension block (block 1) via DDC.
     * edidBuf must already contain a valid base block (128 bytes).
     * On success appends 128 bytes at edidBuf+128 and returns true.
     */
    bool readEdidExtensionViaDdc(uint8_t* edidBuf);

    /**
     * Parse a validated 128-byte EDID base block and populate
     * nativeWidth / nativeHeight / nativeRefresh / hasNativeMode.
     * Also logs manufacturer ID and monitor name if available.
     */
    void parseEdidBlock(const uint8_t* edid);

    /**
     * Top-level entry point called from the constructor after VBIOS load.
     * Tries DDC first, then VBIOS scan, then calls parseEdidBlock().
     */
    void readAndParseEdid();

    /* ---- Firmware blobs ---- */
    PolarisFirmwareBlob* getFirmwareBlob(uint32_t firmwareBit) const;
    bool loadFirmwareBlob(uint32_t firmwareBit);

    /* ---- SDMA ---- */

    /**
     * Allocate a 16-KiB block from the VRAM aperture for ring buffers,
     * writeback pages and fence pages.  Called once from the constructor.
     */
    bool initializeSdmaMemory();

    /**
     * Set up and start SDMA instance i (0 or 1) using already-loaded
     * firmware.  Caller must hold controlMutex.
     */
    bool initializeSdmaInstance(int i);

    /* Halt / un-halt SDMA microcontroller */
    void sdmaHalt(int i);
    void sdmaUnhalt(int i);

    /* Upload SDMA microcode to the GPU.  Returns true on success. */
    bool sdmaLoadMicrocode(int i);

    /* MMIO helpers — always use the 32-bit register aperture */
    inline uint32_t readReg(uint32_t dwOffset) const {
        return ((volatile uint32_t*)mmioBase)[dwOffset];
    }
    inline void writeReg(uint32_t dwOffset, uint32_t value) {
        ((volatile uint32_t*)mmioBase)[dwOffset] = value;
    }

    /* Write one packet DWord and advance the software write pointer */
    inline void sdmaWrite(int i, uint32_t dw) {
        volatile uint32_t* ring =
            (volatile uint32_t*) sdmaRings[i].ringVirtual;
        ring[sdmaRings[i].wptr & (polarisSdmaRingDwords - 1)] = dw;
        sdmaRings[i].wptr++;
    }

    /**
     * Commit wptr to hardware after writing packets.
     * Must be called with controlMutex held.
     */
    void sdmaCommit(int i);

    /**
     * Append a 4-DW FENCE packet and wait (spin) for it to complete.
     * Returns true on success, false on timeout (~100 ms).
     */
    bool sdmaFenceAndWait(int i);

    /**
     * Fill byteCount bytes starting at dstPhysical with fillData.
     * Uses SDMA0.  byteCount must be a multiple of 4, max 0x3FFFE0.
     */
    bool sdmaFill(paddr_t dstPhysical, uint32_t fillData, uint32_t byteCount);

    /**
     * Copy byteCount bytes from srcPhysical to dstPhysical.
     * Uses SDMA0.  byteCount must be <= 0x3FFFE0.
     */
    bool sdmaCopy(paddr_t dstPhysical, paddr_t srcPhysical, uint32_t byteCount);

    /**
     * Blit a rectangle from a BO to the current framebuffer.
     */
    bool sdmaBlitBoToFb(const PolarisBufferObject* srcBo,
                        size_t srcOffset, uint32_t srcPitch,
                        unsigned srcX, unsigned srcY,
                        unsigned dstX, unsigned dstY,
                        unsigned w, unsigned h,
                        unsigned bytesPerPixel);

    /* ---- Constants ---- */
    static constexpr size_t maxBufferObjects   = 256;
    static constexpr size_t maxFirmwareBlobs   = 8;
    static constexpr size_t numSdmaInstances   = 2;
    /* SDMA private VRAM allocation: 4 KiB ring × 2 + 4 KiB writeback + 4 KiB fence */
    static constexpr size_t sdmaVramSize = 4 * PAGESIZE; /* 16 KiB */

    /* ---- Data members ---- */
    uint8_t* bios;
    size_t biosSize;

    paddr_t mmioPhysical;
    vaddr_t mmioBase;
    size_t  mmioSize;

    paddr_t aperBase;
    size_t  aperSize;

    paddr_t framebufferPhysical;
    vaddr_t framebufferBase;
    size_t  framebufferSize;
    size_t  framebufferPitch;
    video_mode framebufferMode;
    bool hasBootFramebuffer;

    /* Serialises control-device devctl / mmap lookups */
    kthread_mutex_t controlMutex;

    /* BO allocator cursor in the aperture */
    paddr_t boCursor;
    size_t  boBytesAvailable;

    PolarisBufferObject* bufferObjects[maxBufferObjects];

    PolarisFirmwareBlob* firmwareBlobs[maxFirmwareBlobs];
    uint32_t firmwarePresentMask;
    uint32_t firmwareLoadAttemptedMask;
    uint32_t firmwareLoadedMask;

    /* SDMA per-instance state */
    PolarisSdmaRing sdmaRings[numSdmaInstances];
    /* Base physical address of the VRAM block we carved for SDMA */
    paddr_t sdmaVramBase;
    bool    sdmaMemoryReady;

    /* Raw EDID blob (up to 256 bytes: base block + one CEA extension) */
    uint8_t  edidData[256];
    uint32_t edidSize;      /* 0 = not available, 128 or 256 */
    uint32_t edidSource;    /* POLARIS_EDID_SOURCE_* */

    /* Native resolution parsed from EDID */
    uint32_t nativeWidth;
    uint32_t nativeHeight;
    uint32_t nativeRefresh; /* Hz */
    bool hasNativeMode;

    PolarisAsic asicType;
    uint8_t bus;
    uint8_t device;
    uint8_t function;
};

#endif