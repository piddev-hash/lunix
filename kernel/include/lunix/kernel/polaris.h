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
 * Minimal AMD Polaris GPU driver scaffold.
 */

#ifndef KERNEL_POLARIS_H
#define KERNEL_POLARIS_H

#include <lunix/kernel/display.h>
#include <lunix/kernel/kernel.h>

enum PolarisAsic {
    POLARIS10,
    POLARIS11,
    POLARIS12,
};

class PolarisDevice : public GraphicsDriver, public ConstructorMayFail {
public:
    PolarisDevice(PolarisAsic asicType, uint8_t bus, uint8_t device,
            uint8_t function);
    ~PolarisDevice() = default;
    NOT_COPYABLE(PolarisDevice);
    NOT_MOVABLE(PolarisDevice);

    bool isSupportedMode(video_mode mode) override;
    vaddr_t setVideoMode(video_mode* mode) override;

    static void initialize(uint8_t bus, uint8_t device, uint8_t function,
            PolarisAsic asicType);
    static bool matchesDevice(uint16_t deviceId, PolarisAsic& asicType);
private:
    bool enableDevice();
    bool initializeBars();
    bool loadAtomBios();
private:
    uint8_t* bios;
    size_t biosSize;
    paddr_t mmioPhysical;
    vaddr_t mmioBase;
    size_t mmioSize;
    paddr_t aperBase;
    size_t aperSize;
    PolarisAsic asicType;
    uint8_t bus;
    uint8_t device;
    uint8_t function;
};

#endif
