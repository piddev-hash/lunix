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

/* kernel/include/lunix/polaris.h
 * AMD Polaris GPU device interface.
 */

#ifndef _DENNIX_POLARIS_H
#define _DENNIX_POLARIS_H

#include <stdint.h>
#include <lunix/devctl.h>

#define POLARIS_GET_INFO              _DEVCTL(_IOCTL_PTR, 13)
#define POLARIS_GET_REGION            _DEVCTL(_IOCTL_PTR, 14)
#define POLARIS_CREATE_BO             _DEVCTL(_IOCTL_PTR, 15)
#define POLARIS_DESTROY_BO            _DEVCTL(_IOCTL_PTR, 16)
#define POLARIS_GET_FIRMWARE_STATUS   _DEVCTL(_IOCTL_PTR, 17)
#define POLARIS_LOAD_FIRMWARE         _DEVCTL(_IOCTL_PTR, 18)
#define POLARIS_GET_RING_STATUS       _DEVCTL(_IOCTL_PTR, 19)
#define POLARIS_SDMA_FILL             _DEVCTL(_IOCTL_PTR, 20)
#define POLARIS_SDMA_COPY             _DEVCTL(_IOCTL_PTR, 21)
#define POLARIS_SDMA_BLIT_BO_TO_FB    _DEVCTL(_IOCTL_PTR, 22)
#define POLARIS_GET_ATOM_BIOS         _DEVCTL(_IOCTL_PTR, 23)
#define POLARIS_GET_EDID              _DEVCTL(_IOCTL_PTR, 24)

#define POLARIS_INFO_HAS_MMIO             (1U << 0)
#define POLARIS_INFO_HAS_APERTURE         (1U << 1)
#define POLARIS_INFO_HAS_BOOT_FRAMEBUFFER (1U << 2)
#define POLARIS_INFO_HAS_ATOM_BIOS        (1U << 3)
#define POLARIS_INFO_HAS_SDMA             (1U << 4)
#define POLARIS_INFO_HAS_NATIVE_MODE      (1U << 5)

#define POLARIS_REGION_MMIO             1U
#define POLARIS_REGION_APERTURE         2U
#define POLARIS_REGION_BOOT_FRAMEBUFFER 3U
#define POLARIS_REGION_BO               4U

#define POLARIS_FIRMWARE_GFX_ME   (1U << 0)
#define POLARIS_FIRMWARE_GFX_PFP  (1U << 1)
#define POLARIS_FIRMWARE_GFX_CE   (1U << 2)
#define POLARIS_FIRMWARE_GFX_MEC  (1U << 3)
#define POLARIS_FIRMWARE_GFX_MEC2 (1U << 4)
#define POLARIS_FIRMWARE_RLC      (1U << 5)
#define POLARIS_FIRMWARE_SDMA0    (1U << 6)
#define POLARIS_FIRMWARE_SDMA1    (1U << 7)

#define POLARIS_RING_GFX   1U
#define POLARIS_RING_SDMA0 2U
#define POLARIS_RING_SDMA1 3U

struct polaris_info {
    uint16_t vendor_id;
    uint16_t device_id;
    uint8_t bus;
    uint8_t device;
    uint8_t function;
    uint8_t asic_type;
    uint32_t flags;
    uint64_t mmio_physical;
    uint64_t mmio_size;
    uint64_t aperture_physical;
    uint64_t aperture_size;
    uint64_t framebuffer_physical;
    uint64_t framebuffer_size;
    uint32_t framebuffer_width;
    uint32_t framebuffer_height;
    uint32_t framebuffer_bpp;
    uint32_t framebuffer_pitch;
    uint32_t atom_bios_size;
    /* Native resolution from EDID embedded in ATOM BIOS (if POLARIS_INFO_HAS_NATIVE_MODE) */
    uint32_t native_width;
    uint32_t native_height;
    uint32_t native_refresh_hz;  /* e.g. 60 */
    uint32_t reserved;
};

struct polaris_region {
    uint32_t region;
    uint32_t handle;
    uint64_t mmap_offset;
    uint64_t physical;
    uint64_t size;
    uint64_t suboffset;
    uint32_t protection;
    uint32_t reserved1;
};

struct polaris_create_bo {
    uint32_t size;
    uint32_t protection;
    uint32_t handle;
    uint32_t reserved0;
    uint64_t mmap_offset;
    uint64_t mapped_size;
    uint64_t suboffset;
};

struct polaris_destroy_bo {
    uint32_t handle;
    uint32_t reserved0;
};

struct polaris_firmware_status {
    uint32_t available_mask;
    uint32_t loaded_mask;
    uint32_t failed_mask;
    uint32_t reserved0;
    uint32_t gfx_me_size;
    uint32_t gfx_pfp_size;
    uint32_t gfx_ce_size;
    uint32_t gfx_mec_size;
    uint32_t gfx_mec2_size;
    uint32_t rlc_size;
    uint32_t sdma0_size;
    uint32_t sdma1_size;
};

struct polaris_load_firmware {
    uint32_t requested_mask;
    uint32_t loaded_mask;
    uint32_t failed_mask;
    uint32_t reserved0;
};

struct polaris_ring_status {
    uint32_t ring;
    uint32_t flags;
    uint32_t reserved0;
    uint32_t reserved1;
    uint64_t wptr;
    uint64_t rptr;
    uint64_t ring_size;
};

/* sdmaFill: fill byteCount bytes at dstBoHandle+dstOffset with fillData */
struct polaris_sdma_fill {
    uint32_t dst_bo_handle;   /* BO handle (destination VRAM buffer) */
    uint64_t dst_offset;      /* byte offset within BO */
    uint64_t byte_count;      /* number of bytes to fill (must be multiple of 4) */
    uint32_t fill_data;       /* 32-bit value to fill with */
    uint32_t reserved0;
};

/* sdmaCopy: copy byteCount bytes from src BO to dst BO */
struct polaris_sdma_copy {
    uint32_t src_bo_handle;   /* source BO handle */
    uint32_t dst_bo_handle;   /* destination BO handle */
    uint64_t src_offset;      /* byte offset in source BO */
    uint64_t dst_offset;      /* byte offset in destination BO */
    uint64_t byte_count;      /* number of bytes to copy */
};

/* sdmaBlitBoToFb: blit rectangle from a BO (src_pitch in bytes) to framebuffer */
struct polaris_sdma_blit_bo_to_fb {
    uint32_t src_bo_handle;   /* source BO handle */
    uint32_t src_pitch;       /* source pitch in bytes */
    uint32_t src_x;           /* source X in pixels */
    uint32_t src_y;           /* source Y in pixels */
    uint32_t dst_x;           /* destination X in framebuffer */
    uint32_t dst_y;           /* destination Y in framebuffer */
    uint32_t width;           /* width in pixels */
    uint32_t height;          /* height in pixels */
    uint32_t bytes_per_pixel; /* 3 or 4 */
    uint32_t reserved0;
};

/* polaris_get_atom_bios: read raw ATOM VBIOS bytes */
struct polaris_get_atom_bios {
    void* data;            /* caller-allocated buffer */
    uint32_t offset;       /* byte offset within BIOS image */
    uint32_t size;         /* bytes to read */
    uint32_t reserved0;
};

/*
 * polaris_get_edid: retrieve the raw 128-byte (or 256-byte with one CEA
 * extension) EDID blob that the driver obtained from the display.
 *
 * The driver tries two sources in order:
 *   1. DCE hardware I2C (DDC) — reads directly from the monitor over the
 *      display engine's built-in I2C controller (connector 0 / DDC1).
 *   2. ATOM VBIOS scan — searches the loaded VBIOS image for an embedded
 *      EDID signature (00 FF FF FF FF FF FF 00) and copies the first valid
 *      128-byte block found.
 *
 * Fields:
 *   data   — caller-allocated buffer of at least `size` bytes.
 *   size   — on input: buffer capacity (must be >= 128).
 *             on output: actual number of bytes written (128 or 256).
 *   source — on output: 1 = DDC hardware, 2 = VBIOS scan, 0 = not found.
 */
#define POLARIS_EDID_SOURCE_NONE  0
#define POLARIS_EDID_SOURCE_DDC   1
#define POLARIS_EDID_SOURCE_VBIOS 2

struct polaris_get_edid {
    void*    data;         /* caller-allocated buffer */
    uint32_t size;         /* in: buffer size; out: bytes written */
    uint32_t source;       /* out: POLARIS_EDID_SOURCE_* */
};

#endif