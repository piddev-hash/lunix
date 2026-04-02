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

/* kernel/src/polaris.cpp
 * Minimal AMD Polaris GPU driver scaffold.
 *
 * This ports the practical low-level pieces from Linux/amdgpu that are small
 * enough to fit this kernel today:
 * - Polaris10/11/12 PCI ID matching
 * - MMIO BAR discovery (BAR5 with BAR2 fallback)
 * - VRAM aperture discovery (BAR0)
 * - ATOM VBIOS validation and loading from VRAM/ROM BAR
 *
 * Full modesetting for real AMD display hardware still requires much more of
 * Linux's amdgpu + atom/dce stack than this kernel currently has.
 */

#include <errno.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <sys/stat.h>
#include <stdlib.h>
#include <string.h>
#include <lunix/polaris.h>
#include <lunix/kernel/addressspace.h>
#include <lunix/kernel/devices.h>
#include <lunix/kernel/log.h>
#include <lunix/kernel/pci.h>
#include <lunix/kernel/console.h>
#include <lunix/kernel/polaris.h>
#include <lunix/kernel/process.h>

#define PCI_COMMAND_MEMORY_SPACE 0x2
#define PCI_COMMAND_BUS_MASTER 0x4
#define PCI_ROM_ADDRESS_ENABLE 0x1

struct PolarisBufferObject {
    uint32_t handle;
    paddr_t physical;
    size_t size;
    size_t mappedSize;
    uint64_t mmapOffset;
};

struct PolarisFirmwareBlob {
    uint32_t bit;
    const char* name;
    uint8_t* data;
    size_t size;
};




class PolarisControlDevice : public Vnode {
public:
    explicit PolarisControlDevice(PolarisDevice* device)
            : Vnode(S_IFCHR | 0666, DevFS::dev), device(device) {}

    int devctl(int command, void* restrict data, size_t size,
            int* restrict info) override;
    void* mmap(size_t size, int protection, int flags, off_t offset) override;
private:
    PolarisDevice* device;
};

namespace {

static size_t numPolarisDevices;
static uint32_t nextPolarisBoHandle = 1;
static const uint64_t polarisMmapRegionShift = 56;
static const uint64_t polarisMmapHandleShift = 32;
static const uint64_t polarisMmapHandleMask =
        ((1ULL << (polarisMmapRegionShift - polarisMmapHandleShift)) - 1) <<
        polarisMmapHandleShift;
static const uint64_t polarisMmapOffsetMask =
        (1ULL << polarisMmapHandleShift) - 1;

static const char* getAsicName(PolarisAsic asicType) {
    switch (asicType) {
    case POLARIS10: return "Polaris10";
    case POLARIS11: return "Polaris11";
    case POLARIS12: return "Polaris12";
    }

    return "Polaris";
}

static const char* getFirmwareName(PolarisAsic asicType, uint32_t firmwareBit) {
    switch (asicType) {
    case POLARIS10:
        switch (firmwareBit) {
        case POLARIS_FIRMWARE_GFX_ME: return "polaris10_me.bin";
        case POLARIS_FIRMWARE_GFX_PFP: return "polaris10_pfp.bin";
        case POLARIS_FIRMWARE_GFX_CE: return "polaris10_ce.bin";
        case POLARIS_FIRMWARE_GFX_MEC: return "polaris10_mec.bin";
        case POLARIS_FIRMWARE_GFX_MEC2: return "polaris10_mec2.bin";
        case POLARIS_FIRMWARE_RLC: return "polaris10_rlc.bin";
        case POLARIS_FIRMWARE_SDMA0: return "polaris10_sdma.bin";
        case POLARIS_FIRMWARE_SDMA1: return "polaris10_sdma1.bin";
        default: return nullptr;
        }
    case POLARIS11:
        switch (firmwareBit) {
        case POLARIS_FIRMWARE_GFX_ME: return "polaris11_me.bin";
        case POLARIS_FIRMWARE_GFX_PFP: return "polaris11_pfp.bin";
        case POLARIS_FIRMWARE_GFX_CE: return "polaris11_ce.bin";
        case POLARIS_FIRMWARE_GFX_MEC: return "polaris11_mec.bin";
        case POLARIS_FIRMWARE_GFX_MEC2: return "polaris11_mec2.bin";
        case POLARIS_FIRMWARE_RLC: return "polaris11_rlc.bin";
        case POLARIS_FIRMWARE_SDMA0: return "polaris11_sdma.bin";
        case POLARIS_FIRMWARE_SDMA1: return "polaris11_sdma1.bin";
        default: return nullptr;
        }
    case POLARIS12:
        switch (firmwareBit) {
        case POLARIS_FIRMWARE_GFX_ME: return "polaris12_me.bin";
        case POLARIS_FIRMWARE_GFX_PFP: return "polaris12_pfp.bin";
        case POLARIS_FIRMWARE_GFX_CE: return "polaris12_ce.bin";
        case POLARIS_FIRMWARE_GFX_MEC: return "polaris12_mec.bin";
        case POLARIS_FIRMWARE_GFX_MEC2: return "polaris12_mec2.bin";
        case POLARIS_FIRMWARE_RLC: return "polaris12_rlc.bin";
        case POLARIS_FIRMWARE_SDMA0: return "polaris12_sdma.bin";
        case POLARIS_FIRMWARE_SDMA1: return "polaris12_sdma1.bin";
        default: return nullptr;
        }
    }

    return nullptr;
}

static bool isValidAtomBios(const uint8_t* bios, size_t size) {
    if (!bios || size < 0x4A) return false;
    if (bios[0] != 0x55 || bios[1] != 0xAA) return false;

    uint16_t biosHeaderStart = bios[0x48] | (bios[0x49] << 8);
    if (!biosHeaderStart) return false;

    size_t atomSignatureOffset = biosHeaderStart + 4;
    if (atomSignatureOffset + 4 > size) return false;

    return memcmp(bios + atomSignatureOffset, "ATOM", 4) == 0 ||
            memcmp(bios + atomSignatureOffset, "MOTA", 4) == 0;
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

    uint64_t base;
    uint64_t barSize;
    if (is64Bit) {
        uint64_t mask = ((uint64_t) sizeHigh << 32) | (sizeLow & ~0xFULL);
        base = ((uint64_t) originalHigh << 32) | (originalLow & ~0xFULL);
        barSize = (~mask) + 1;
    } else {
        uint32_t mask = sizeLow & ~0xFULL;
        base = originalLow & ~0xFULL;
        // Keep 32-bit BAR size calculation in 32 bits. Extending the
        // mask to 64 bits on x86_64 would turn normal MMIO BARs into huge
        // bogus sizes and make the mapping fail.
        barSize = (uint32_t) ((~mask) + 1);
    }

    if (!base || !barSize) return false;
    if ((uint64_t) (paddr_t) base != base) return false;
    if ((uint64_t) (size_t) barSize != barSize) return false;

    baseAddress = (paddr_t) base;
    size = (size_t) barSize;
    return true;
}

static bool readExpansionRomBar(unsigned int bus, unsigned int device,
        unsigned int function, paddr_t& baseAddress, size_t& size) {
    unsigned int offset = offsetof(PciHeader, expansionRomBaseAddress);
    uint32_t original = Pci::readConfig(bus, device, function, offset);

    Pci::writeConfig(bus, device, function, offset, 0xFFFFFFFF);
    uint32_t sizeMask = Pci::readConfig(bus, device, function, offset);
    Pci::writeConfig(bus, device, function, offset, original);

    uint32_t base = original & ~0x7FFU;
    uint32_t mask = sizeMask & ~0x7FFU;
    uint32_t romSize = (~mask) + 1;
    if (!base || !romSize) return false;

    baseAddress = base;
    size = romSize;
    return true;
}

static bool copyAtomBiosFromMapping(paddr_t physicalAddress, size_t size,
        uint8_t** biosData, size_t* biosSize) {
    size_t mapSize = ALIGNUP(size, PAGESIZE);
    vaddr_t mapping = kernelSpace->mapPhysical(physicalAddress, mapSize,
            PROT_READ);
    if (!mapping) return false;

    const uint8_t* src = (const uint8_t*) mapping;
    uint8_t* copy = (uint8_t*) malloc(size);
    if (!copy) {
        kernelSpace->unmapPhysical(mapping, mapSize);
        return false;
    }

    memcpy(copy, src, size);
    kernelSpace->unmapPhysical(mapping, mapSize);

    if (!isValidAtomBios(copy, size)) {
        free(copy);
        return false;
    }

    *biosData = copy;
    *biosSize = size;
    return true;
}

static bool readAtomBiosFromVram(unsigned int bus, unsigned int device,
        unsigned int function, uint8_t** biosData, size_t* biosSize) {
    paddr_t barBase;
    size_t barSize;
    if (!readMemoryBar(bus, device, function, 0, barBase, barSize)) {
        return false;
    }

    size_t probeSize = 256 * 1024;
    if (barSize < probeSize) return false;
    return copyAtomBiosFromMapping(barBase, probeSize, biosData, biosSize);
}

static bool readAtomBiosFromExpansionRom(unsigned int bus, unsigned int device,
        unsigned int function, uint8_t** biosData, size_t* biosSize) {
    paddr_t romBase;
    size_t romSize;
    if (!readExpansionRomBar(bus, device, function, romBase, romSize)) {
        return false;
    }

    unsigned int offset = offsetof(PciHeader, expansionRomBaseAddress);
    uint32_t original = Pci::readConfig(bus, device, function, offset);
    uint32_t enabled = (uint32_t) romBase | PCI_ROM_ADDRESS_ENABLE;
    Pci::writeConfig(bus, device, function, offset, enabled);

    size_t headerMapSize = PAGESIZE;
    vaddr_t headerMapping = kernelSpace->mapPhysical(romBase, headerMapSize,
            PROT_READ);
    if (!headerMapping) {
        Pci::writeConfig(bus, device, function, offset, original);
        return false;
    }

    const uint8_t* header = (const uint8_t*) headerMapping;
    size_t imageSize = 0;
    if (header[0] == 0x55 && header[1] == 0xAA) {
        imageSize = header[2] << 9;
    }
    kernelSpace->unmapPhysical(headerMapping, headerMapSize);

    if (!imageSize || imageSize > romSize) {
        Pci::writeConfig(bus, device, function, offset, original);
        return false;
    }

    bool success = copyAtomBiosFromMapping(romBase, imageSize, biosData,
            biosSize);
    Pci::writeConfig(bus, device, function, offset, original);
    return success;
}

}

PolarisDevice::PolarisDevice(PolarisAsic asicType, uint8_t bus, uint8_t device,
        uint8_t function) : bios(nullptr), biosSize(0), mmioPhysical(0),
        mmioBase(0), mmioSize(0), aperBase(0), aperSize(0),
        framebufferPhysical(0), framebufferBase(0), framebufferSize(0),
        framebufferPitch(0), framebufferMode(), hasBootFramebuffer(false),
        controlMutex(KTHREAD_MUTEX_INITIALIZER), boCursor(0),
        boBytesAvailable(0), bufferObjects(), firmwareBlobs(),
        firmwarePresentMask(0), firmwareLoadAttemptedMask(0),
        firmwareLoadedMask(0), edidData(), edidSize(0),
        edidSource(POLARIS_EDID_SOURCE_NONE), nativeWidth(0), nativeHeight(0),
        nativeRefresh(0), hasNativeMode(false),
        asicType(asicType), bus(bus), device(device),
        function(function) {
    if (!enableDevice() || !initializeBars()) {
        FAIL_CONSTRUCTOR;
    }

    tryAdoptBootFramebuffer();

    if (loadAtomBios()) {
        Log::printf("%s GPU at %u/%u/%u: loaded %zu bytes of ATOM VBIOS\n",
                getAsicName(asicType), bus, device, function, biosSize);
    } else {
        Log::printf("%s GPU at %u/%u/%u: MMIO ready, but ATOM VBIOS was not "
                "found via VRAM/ROM BAR\n", getAsicName(asicType), bus,
                device, function);
    }

    readAndParseEdid();
}

bool PolarisDevice::matchesDevice(uint16_t deviceId, PolarisAsic& asicType) {
    switch (deviceId) {
    case 0x67C0:
    case 0x67C1:
    case 0x67C2:
    case 0x67C4:
    case 0x67C7:
    case 0x67C8:
    case 0x67C9:
    case 0x67CA:
    case 0x67CC:
    case 0x67CF:
    case 0x67D0:
    case 0x67DF:
    case 0x6FDF:
        asicType = POLARIS10;
        return true;
    case 0x67E0:
    case 0x67E1:
    case 0x67E3:
    case 0x67E7:
    case 0x67E8:
    case 0x67E9:
    case 0x67EB:
    case 0x67EF:
    case 0x67FF:
        asicType = POLARIS11;
        return true;
    case 0x6980:
    case 0x6981:
    case 0x6985:
    case 0x6986:
    case 0x6987:
    case 0x6995:
    case 0x6997:
    case 0x699F:
        asicType = POLARIS12;
        return true;
    default:
        return false;
    }
}

void PolarisDevice::initialize(uint8_t bus, uint8_t device, uint8_t function,
        PolarisAsic asicType) {
    bool graphicsAlreadyActive = graphicsDriver != nullptr;
    if (graphicsAlreadyActive) {
        Log::printf("%s GPU at %u/%u/%u found, but another graphics driver is "
                "already active\n", getAsicName(asicType), bus, device,
                function);
    }

    PolarisDevice* deviceObject = new PolarisDevice(asicType, bus, device,
            function);
    if (!deviceObject) {
        Log::printf("%s GPU at %u/%u/%u detected, but initialization failed\n",
                getAsicName(asicType), bus, device, function);
        return;
    }

    char deviceName[16];
    snprintf(deviceName, sizeof(deviceName), "polaris%zu", numPolarisDevices++);
    Reference<Vnode> controlDevice = new PolarisControlDevice(deviceObject);
    if (!controlDevice) {
        Log::printf("%s GPU at %u/%u/%u: failed to create control device\n",
                getAsicName(asicType), bus, device, function);
    } else {
        devFS.addDevice(deviceName, controlDevice);
    }

    if (!deviceObject->hasBootFramebuffer) {
        Log::printf("%s GPU at %u/%u/%u: MMIO/VBIOS ready, but no boot "
                "framebuffer was handed off\n", getAsicName(asicType), bus,
                device, function);
    }

    if (!graphicsAlreadyActive) {
        graphicsDriver = deviceObject;
        /*
         * We do not attempt to switch to the EDID native resolution here.
         * PolarisDevice has no DCE modesetting stack — it can only serve
         * the boot framebuffer at its original dimensions.  Calling
         * setVideoMode() with a different resolution would make the display
         * subsystem render at the wrong stride and produce a black screen.
         *
         * The native resolution is exposed via POLARIS_INFO_HAS_NATIVE_MODE
         * in POLARIS_GET_INFO for userspace to use as a hint.
         */
    }
}

bool PolarisDevice::enableDevice() {
    uint16_t command = Pci::readConfig(bus, device, function,
            offsetof(PciHeader, command));
    command |= PCI_COMMAND_MEMORY_SPACE | PCI_COMMAND_BUS_MASTER;
    Pci::writeConfig(bus, device, function, offsetof(PciHeader, command),
            command);
    return true;
}

bool PolarisDevice::initializeBars() {
    if (!readMemoryBar(bus, device, function, 5, mmioPhysical, mmioSize) &&
            !readMemoryBar(bus, device, function, 2, mmioPhysical, mmioSize)) {
        Log::printf("%s GPU at %u/%u/%u has no usable MMIO BAR\n",
                getAsicName(asicType), bus, device, function);
        return false;
    }

    size_t mapSize = ALIGNUP(mmioSize, PAGESIZE);
    mmioBase = kernelSpace->mapPhysical(mmioPhysical, mapSize,
            PROT_READ | PROT_WRITE);
    if (!mmioBase) {
        Log::printf("%s GPU at %u/%u/%u: failed to map MMIO BAR\n",
                getAsicName(asicType), bus, device, function);
        return false;
    }
    mmioSize = mapSize;

    if (!readMemoryBar(bus, device, function, 0, aperBase, aperSize)) {
        aperBase = 0;
        aperSize = 0;
    } else {
        boCursor = aperBase;
        boBytesAvailable = aperSize;
    }

    Log::printf("%s GPU at %u/%u/%u: MMIO %#lX (%zu bytes), aperture %#lX "
            "(%zu bytes)\n", getAsicName(asicType), bus, device, function,
            (unsigned long) mmioPhysical, mmioSize, (unsigned long) aperBase,
            aperSize);
    return true;
}

bool PolarisDevice::loadAtomBios() {
    if (readAtomBiosFromVram(bus, device, function, &bios, &biosSize)) {
        return true;
    }

    return readAtomBiosFromExpansionRom(bus, device, function, &bios,
            &biosSize);
}

bool PolarisDevice::tryAdoptBootFramebuffer() {
    BootFramebufferInfo info;
    if (!getBootFramebufferInfo(info)) return false;
    if (info.mode.video_bpp == 0) return false;
    if (!aperBase || !aperSize) return false;
    if (info.physicalAddress < aperBase) return false;

    uint64_t offset = (uint64_t) info.physicalAddress - (uint64_t) aperBase;
    uint64_t end = offset + info.size;
    if (end > aperSize) return false;

    framebufferPhysical = info.physicalAddress;
    framebufferBase = info.virtualAddress;
    framebufferSize = info.size;
    framebufferPitch = info.pitch;
    framebufferMode = info.mode;
    hasBootFramebuffer = true;

    paddr_t framebufferStart = framebufferPhysical & ~PAGE_MISALIGN;
    size_t framebufferMappedSize = ALIGNUP(
            (framebufferPhysical - framebufferStart) + framebufferSize,
            PAGESIZE);

    if (boCursor < framebufferStart) {
        size_t prefixSize = framebufferStart - boCursor;
        if (prefixSize < boBytesAvailable) {
            boBytesAvailable = prefixSize;
        }
    } else if (boCursor >= framebufferStart &&
            boCursor < framebufferStart + framebufferMappedSize) {
        paddr_t newCursor = framebufferStart + framebufferMappedSize;
        size_t consumed = newCursor - boCursor;
        boCursor = newCursor;
        boBytesAvailable = consumed < boBytesAvailable ?
                boBytesAvailable - consumed : 0;
    }

    Log::printf("%s GPU at %u/%u/%u: adopted boot framebuffer %#lX "
            "(%ux%ux%u pitch %zu)\n", getAsicName(asicType), bus, device,
            function, (unsigned long) framebufferPhysical,
            framebufferMode.video_width, framebufferMode.video_height,
            framebufferMode.video_bpp, framebufferPitch);
    return true;
}

int PolarisControlDevice::devctl(int command, void* restrict data, size_t size,
        int* restrict info) {
    AutoLock lock(&device->controlMutex);

    switch (command) {
    case POLARIS_GET_INFO: {
        if (size != 0 && size != sizeof(struct polaris_info)) {
            *info = -1;
            return EINVAL;
        }
        if (!data) {
            *info = -1;
            return EINVAL;
        }

        struct polaris_info* result = (struct polaris_info*) data;
        memset(result, 0, sizeof(*result));
        result->vendor_id = 0x1002;
        result->device_id = Pci::readConfig(device->bus, device->device,
                device->function, offsetof(PciHeader, deviceId));
        result->bus = device->bus;
        result->device = device->device;
        result->function = device->function;
        result->asic_type = device->asicType;
        if (device->mmioPhysical && device->mmioSize) {
            result->flags |= POLARIS_INFO_HAS_MMIO;
            result->mmio_physical = device->mmioPhysical;
            result->mmio_size = device->mmioSize;
        }
        if (device->aperBase && device->aperSize) {
            result->flags |= POLARIS_INFO_HAS_APERTURE;
            result->aperture_physical = device->aperBase;
            result->aperture_size = device->aperSize;
        }
        if (device->hasBootFramebuffer) {
            result->flags |= POLARIS_INFO_HAS_BOOT_FRAMEBUFFER;
            result->framebuffer_physical = device->framebufferPhysical;
            result->framebuffer_size = device->framebufferSize;
            result->framebuffer_width = device->framebufferMode.video_width;
            result->framebuffer_height = device->framebufferMode.video_height;
            result->framebuffer_bpp = device->framebufferMode.video_bpp;
            result->framebuffer_pitch = device->framebufferPitch;
        }
        if (device->bios && device->biosSize) {
            result->flags |= POLARIS_INFO_HAS_ATOM_BIOS;
            result->atom_bios_size = device->biosSize;
        }
        if (device->hasNativeMode) {
            result->flags |= POLARIS_INFO_HAS_NATIVE_MODE;
            result->native_width = device->nativeWidth;
            result->native_height = device->nativeHeight;
            result->native_refresh_hz = device->nativeRefresh;
        }

        *info = 0;
        return 0;
    } break;
    case POLARIS_GET_FIRMWARE_STATUS: {
        if (size != 0 && size != sizeof(struct polaris_firmware_status)) {
            *info = -1;
            return EINVAL;
        }
        if (!data) {
            *info = -1;
            return EINVAL;
        }

        struct polaris_firmware_status* status =
                (struct polaris_firmware_status*) data;
        memset(status, 0, sizeof(*status));
        status->available_mask = device->firmwarePresentMask;
        status->loaded_mask = device->firmwareLoadedMask;
        status->failed_mask = device->firmwareLoadAttemptedMask &
                ~device->firmwareLoadedMask;

        PolarisFirmwareBlob* blob;
        blob = device->getFirmwareBlob(POLARIS_FIRMWARE_GFX_ME);
        if (blob) status->gfx_me_size = blob->size;
        blob = device->getFirmwareBlob(POLARIS_FIRMWARE_GFX_PFP);
        if (blob) status->gfx_pfp_size = blob->size;
        blob = device->getFirmwareBlob(POLARIS_FIRMWARE_GFX_CE);
        if (blob) status->gfx_ce_size = blob->size;
        blob = device->getFirmwareBlob(POLARIS_FIRMWARE_GFX_MEC);
        if (blob) status->gfx_mec_size = blob->size;
        blob = device->getFirmwareBlob(POLARIS_FIRMWARE_GFX_MEC2);
        if (blob) status->gfx_mec2_size = blob->size;
        blob = device->getFirmwareBlob(POLARIS_FIRMWARE_RLC);
        if (blob) status->rlc_size = blob->size;
        blob = device->getFirmwareBlob(POLARIS_FIRMWARE_SDMA0);
        if (blob) status->sdma0_size = blob->size;
        blob = device->getFirmwareBlob(POLARIS_FIRMWARE_SDMA1);
        if (blob) status->sdma1_size = blob->size;

        *info = 0;
        return 0;
    } break;
    case POLARIS_LOAD_FIRMWARE: {
        if (size != 0 && size != sizeof(struct polaris_load_firmware)) {
            *info = -1;
            return EINVAL;
        }
        if (!data) {
            *info = -1;
            return EINVAL;
        }

        struct polaris_load_firmware* request =
                (struct polaris_load_firmware*) data;
        if (request->requested_mask == 0) {
            *info = -1;
            return EINVAL;
        }

        uint32_t loadedMask = 0;
        uint32_t failedMask = 0;
        uint32_t requestedMask = request->requested_mask;
        for (uint32_t bit = 1; bit != 0; bit <<= 1) {
            if (!(requestedMask & bit)) continue;
            if (device->loadFirmwareBlob(bit)) {
                loadedMask |= bit;
            } else {
                failedMask |= bit;
            }
        }

        request->loaded_mask = loadedMask;
        request->failed_mask = failedMask;
        *info = failedMask ? -1 : 0;
        return failedMask ? ENOENT : 0;
    } break;
    case POLARIS_GET_RING_STATUS: {
        if (size != 0 && size != sizeof(struct polaris_ring_status)) {
            *info = -1;
            return EINVAL;
        }
        if (!data) {
            *info = -1;
            return EINVAL;
        }

        struct polaris_ring_status* status =
                (struct polaris_ring_status*) data;
        uint32_t requestedRing = status->ring;
        memset(status, 0, sizeof(*status));
        status->ring = requestedRing;

        switch (requestedRing) {
        case POLARIS_RING_GFX:
            status->flags = device->firmwareLoadedMask &
                    (POLARIS_FIRMWARE_GFX_ME | POLARIS_FIRMWARE_GFX_PFP |
                    POLARIS_FIRMWARE_GFX_CE | POLARIS_FIRMWARE_GFX_MEC) ?
                    0 : 0;
            break;
        case POLARIS_RING_SDMA0:
            status->flags = device->firmwareLoadedMask &
                    POLARIS_FIRMWARE_SDMA0 ? 0 : 0;
            break;
        case POLARIS_RING_SDMA1:
            status->flags = device->firmwareLoadedMask &
                    POLARIS_FIRMWARE_SDMA1 ? 0 : 0;
            break;
        default:
            *info = -1;
            return EINVAL;
        }

        *info = 0;
        return 0;
    } break;
    case POLARIS_GET_REGION: {
        if (size != 0 && size != sizeof(struct polaris_region)) {
            *info = -1;
            return EINVAL;
        }
        if (!data) {
            *info = -1;
            return EINVAL;
        }

        struct polaris_region* region = (struct polaris_region*) data;
        uint32_t requestedRegion = region->region;
        uint32_t requestedHandle = region->handle;
        memset(region, 0, sizeof(*region));
        region->region = requestedRegion;
        region->handle = requestedHandle;
        region->mmap_offset = (uint64_t) requestedRegion <<
                polarisMmapRegionShift;

        switch (requestedRegion) {
        case POLARIS_REGION_MMIO:
            if (!device->mmioPhysical || !device->mmioSize) {
                *info = -1;
                return ENODEV;
            }
            region->physical = device->mmioPhysical;
            region->size = device->mmioSize;
            region->protection = PROT_READ | PROT_WRITE;
            break;
        case POLARIS_REGION_APERTURE:
            if (!device->aperBase || !device->aperSize) {
                *info = -1;
                return ENODEV;
            }
            region->physical = device->aperBase;
            region->size = device->aperSize;
            region->protection = PROT_READ | PROT_WRITE;
            break;
        case POLARIS_REGION_BOOT_FRAMEBUFFER:
            if (!device->hasBootFramebuffer || !device->framebufferSize) {
                *info = -1;
                return ENODEV;
            }
            region->physical = device->framebufferPhysical & ~PAGE_MISALIGN;
            region->suboffset = device->framebufferPhysical - region->physical;
            region->size = ALIGNUP(region->suboffset + device->framebufferSize,
                    PAGESIZE);
            region->protection = PROT_READ | PROT_WRITE;
            break;
        case POLARIS_REGION_BO: {
            PolarisBufferObject* bo = nullptr;
            for (size_t i = 0; i < PolarisDevice::maxBufferObjects; i++) {
                if (device->bufferObjects[i] &&
                        device->bufferObjects[i]->handle == requestedHandle) {
                    bo = device->bufferObjects[i];
                    break;
                }
            }
            if (!bo) {
                *info = -1;
                return ENODEV;
            }
            region->physical = bo->physical;
            region->size = bo->mappedSize;
            region->protection = PROT_READ | PROT_WRITE;
            region->mmap_offset = bo->mmapOffset;
            break;
        }
        default:
            *info = -1;
            return EINVAL;
        }

        *info = 0;
        return 0;
    } break;
    case POLARIS_CREATE_BO: {
        if (size != 0 && size != sizeof(struct polaris_create_bo)) {
            *info = -1;
            return EINVAL;
        }
        if (!data) {
            *info = -1;
            return EINVAL;
        }
        if (!device->aperBase || device->boBytesAvailable == 0) {
            *info = -1;
            return ENODEV;
        }

        struct polaris_create_bo* request = (struct polaris_create_bo*) data;
        if (request->size == 0) {
            *info = -1;
            return EINVAL;
        }

        size_t requestedSize = ALIGNUP(request->size, PAGESIZE);
        if (requestedSize > device->boBytesAvailable) {
            *info = -1;
            return ENOMEM;
        }

        size_t slot = PolarisDevice::maxBufferObjects;
        for (size_t i = 0; i < PolarisDevice::maxBufferObjects; i++) {
            if (!device->bufferObjects[i]) {
                slot = i;
                break;
            }
        }
        if (slot == PolarisDevice::maxBufferObjects) {
            *info = -1;
            return ENOMEM;
        }

        PolarisBufferObject* bo = new PolarisBufferObject;
        if (!bo) {
            *info = -1;
            return ENOMEM;
        }

        bo->handle = nextPolarisBoHandle++;
        if (bo->handle == 0) bo->handle = nextPolarisBoHandle++;
        bo->physical = device->boCursor;
        bo->size = request->size;
        bo->mappedSize = requestedSize;
        bo->mmapOffset = ((uint64_t) POLARIS_REGION_BO <<
                polarisMmapRegionShift) |
                ((uint64_t) bo->handle << polarisMmapHandleShift);

        device->bufferObjects[slot] = bo;
        device->boCursor += requestedSize;
        device->boBytesAvailable -= requestedSize;

        request->handle = bo->handle;
        request->mmap_offset = bo->mmapOffset;
        request->mapped_size = bo->mappedSize;
        request->suboffset = 0;

        *info = 0;
        return 0;
    } break;
    case POLARIS_DESTROY_BO: {
        if (size != 0 && size != sizeof(struct polaris_destroy_bo)) {
            *info = -1;
            return EINVAL;
        }
        if (!data) {
            *info = -1;
            return EINVAL;
        }

        const struct polaris_destroy_bo* request =
                (const struct polaris_destroy_bo*) data;
        for (size_t i = 0; i < PolarisDevice::maxBufferObjects; i++) {
            if (device->bufferObjects[i] &&
                    device->bufferObjects[i]->handle == request->handle) {
                PolarisBufferObject* bo = device->bufferObjects[i];
                if (bo->physical + bo->mappedSize == device->boCursor) {
                    device->boCursor = bo->physical;
                    device->boBytesAvailable += bo->mappedSize;
                }
                delete bo;
                device->bufferObjects[i] = nullptr;
                *info = 0;
                return 0;
            }
        }

        *info = -1;
        return ENODEV;
    } break;
    case POLARIS_GET_ATOM_BIOS: {
        if (size != 0 && size != sizeof(struct polaris_get_atom_bios)) {
            *info = -1;
            return EINVAL;
        }
        if (!data) {
            *info = -1;
            return EINVAL;
        }

        struct polaris_get_atom_bios* req =
                (struct polaris_get_atom_bios*) data;
        if (!device->bios || !device->biosSize) {
            *info = -1;
            return ENOENT;
        }
        if (req->offset >= device->biosSize) {
            *info = -1;
            return EINVAL;
        }
        if (!req->data) {
            /* Query-only: just report total size */
            req->size = device->biosSize;
            *info = 0;
            return 0;
        }

        size_t avail = device->biosSize - req->offset;
        if (req->size > avail) req->size = (uint32_t) avail;
        memcpy(req->data, device->bios + req->offset, req->size);
        *info = 0;
        return 0;
    } break;
    case POLARIS_GET_EDID: {
        if (size != 0 && size != sizeof(struct polaris_get_edid)) {
            *info = -1;
            return EINVAL;
        }
        if (!data) {
            *info = -1;
            return EINVAL;
        }

        struct polaris_get_edid* req = (struct polaris_get_edid*) data;

        /*
         * If no EDID was found during boot (VBIOS scan found nothing),
         * try the DDC hardware path now — the display engine should be
         * fully initialised by the time userspace calls this ioctl.
         */
        if (device->edidSize == 0) {
            uint8_t buf[256];
            memset(buf, 0, sizeof(buf));
            if (device->readEdidViaDdc(buf)) {
                device->edidSource = POLARIS_EDID_SOURCE_DDC;
                device->edidSize   = 128;
                memcpy(device->edidData, buf, 128);
                /* Try extension block */
                if (buf[0x7E] > 0 && device->readEdidExtensionViaDdc(buf)) {
                    device->edidSize = 256;
                    memcpy(device->edidData + 128, buf + 128, 128);
                }
                Log::printf("%s GPU at %u/%u/%u: EDID read via DDC on demand "
                            "(%u bytes)\n",
                            getAsicName(device->asicType),
                            device->bus, device->device, device->function,
                            device->edidSize);
                device->parseEdidBlock(device->edidData);
            }
        }

        if (device->edidSize == 0) {
            req->size   = 0;
            req->source = POLARIS_EDID_SOURCE_NONE;
            *info = -1;
            return ENOENT;
        }

        uint32_t toWrite = device->edidSize;
        if (req->size < toWrite) toWrite = req->size;

        if (req->data && toWrite > 0) {
            memcpy(req->data, device->edidData, toWrite);
        }

        req->size   = device->edidSize; /* report actual EDID size */
        req->source = device->edidSource;
        *info = 0;
        return 0;
    } break;
    default:
        *info = -1;
        return EINVAL;
    }
}

void* PolarisControlDevice::mmap(size_t size, int protection, int /*flags*/,
        off_t offset) {
    AutoLock lock(&device->controlMutex);

    if (!size || offset < 0 || !PAGE_ALIGNED((uint64_t) offset)) {
        errno = EINVAL;
        return MAP_FAILED;
    }
    if (protection & PROT_EXEC) {
        errno = ENOTSUP;
        return MAP_FAILED;
    }

    uint64_t rawOffset = (uint64_t) offset;
    uint32_t region = rawOffset >> polarisMmapRegionShift;
    uint32_t handle = (rawOffset & polarisMmapHandleMask) >>
            polarisMmapHandleShift;
    uint64_t regionOffset = rawOffset & polarisMmapOffsetMask;

    paddr_t physical = 0;
    size_t regionSize = 0;
    int mapProtection = protection;

    switch (region) {
    case POLARIS_REGION_MMIO:
        physical = device->mmioPhysical;
        regionSize = device->mmioSize;
        break;
    case POLARIS_REGION_APERTURE:
        physical = device->aperBase;
        regionSize = device->aperSize;
        mapProtection |= PROT_WRITE_COMBINING;
        break;
    case POLARIS_REGION_BOOT_FRAMEBUFFER:
        if (!device->hasBootFramebuffer || !device->framebufferSize) {
            errno = ENODEV;
            return MAP_FAILED;
        }
        physical = device->framebufferPhysical & ~PAGE_MISALIGN;
        regionSize = ALIGNUP((device->framebufferPhysical - physical) +
                device->framebufferSize, PAGESIZE);
        mapProtection |= PROT_WRITE_COMBINING;
        break;
    case POLARIS_REGION_BO: {
        PolarisBufferObject* bo = nullptr;
        for (size_t i = 0; i < PolarisDevice::maxBufferObjects; i++) {
            if (device->bufferObjects[i] &&
                    device->bufferObjects[i]->handle == handle) {
                bo = device->bufferObjects[i];
                break;
            }
        }
        if (!bo) {
            errno = ENODEV;
            return MAP_FAILED;
        }
        physical = bo->physical;
        regionSize = bo->mappedSize;
        mapProtection |= PROT_WRITE_COMBINING;
        break;
    }
    default:
        errno = EINVAL;
        return MAP_FAILED;
    }

    if (!physical || !regionSize || regionOffset > regionSize ||
            size > regionSize - regionOffset) {
        errno = EINVAL;
        return MAP_FAILED;
    }

    return (void*) Process::current()->addressSpace->mapPhysical(
            physical + regionOffset, size, mapProtection);
}

PolarisFirmwareBlob* PolarisDevice::getFirmwareBlob(uint32_t firmwareBit) const {
    for (size_t i = 0; i < maxFirmwareBlobs; i++) {
        if (firmwareBlobs[i] && firmwareBlobs[i]->bit == firmwareBit) {
            return firmwareBlobs[i];
        }
    }

    return nullptr;
}

bool PolarisDevice::loadFirmwareBlob(uint32_t firmwareBit) {
    firmwareLoadAttemptedMask |= firmwareBit;
    if (firmwareLoadedMask & firmwareBit) return true;

    const char* firmwareName = getFirmwareName(asicType, firmwareBit);
    if (!firmwareName) return false;

    char firmwarePath[128];
    snprintf(firmwarePath, sizeof(firmwarePath), "/lib/firmware/amdgpu/%s",
            firmwareName);

    Reference<FileDescription> rootFd = Process::current()->rootFd;
    if (!rootFd) return false;

    Reference<Vnode> vnode = resolvePath(rootFd->vnode, firmwarePath);
    if (!vnode) return false;

    struct stat st;
    if (vnode->stat(&st) < 0) return false;
    if (!S_ISREG(st.st_mode) || st.st_size <= 0) return false;
    if (sizeof(off_t) > sizeof(size_t) && st.st_size > (off_t) SIZE_MAX) {
        return false;
    }

    size_t blobSize = st.st_size;
    uint8_t* blobData = (uint8_t*) malloc(blobSize);
    if (!blobData) return false;

    ssize_t bytesRead = vnode->pread(blobData, blobSize, 0, 0);
    if (bytesRead != (ssize_t) blobSize) {
        free(blobData);
        return false;
    }

    size_t slot = maxFirmwareBlobs;
    for (size_t i = 0; i < maxFirmwareBlobs; i++) {
        if (!firmwareBlobs[i]) {
            slot = i;
            break;
        }
    }
    if (slot == maxFirmwareBlobs) {
        free(blobData);
        return false;
    }

    PolarisFirmwareBlob* blob = new PolarisFirmwareBlob;
    if (!blob) {
        free(blobData);
        return false;
    }

    blob->bit = firmwareBit;
    blob->name = firmwareName;
    blob->data = blobData;
    blob->size = blobSize;
    firmwareBlobs[slot] = blob;
    firmwarePresentMask |= firmwareBit;
    firmwareLoadedMask |= firmwareBit;
    return true;
}


bool PolarisDevice::isSupportedMode(video_mode mode) {
    if (!hasBootFramebuffer) return false;
    /*
     * We can only serve the boot framebuffer as-is — we have no DCE
     * modesetting stack to reprogram the display engine.  The native
     * resolution from EDID is informational only; claiming we support it
     * when it differs from the boot framebuffer would cause the display
     * subsystem to render at the wrong stride and produce a black screen.
     */
    return mode.video_width == framebufferMode.video_width &&
            mode.video_height == framebufferMode.video_height &&
            mode.video_bpp == framebufferMode.video_bpp;
}

vaddr_t PolarisDevice::setVideoMode(video_mode* mode) {
    if (!hasBootFramebuffer || !mode) return 0;
    if (!isSupportedMode(*mode)) return 0;

    *mode = framebufferMode;
    return framebufferBase;
}

/* --------------------------------------------------------------------------
 * SDMA Implementation
 * -------------------------------------------------------------------------- */

#define SDMA0_BASE_OFFSET 0x10A0
#define SDMA1_BASE_OFFSET 0x12A0

/* Relative offsets from SDMA base */
#define SDMA_GFX_RB_CNTL 0x0
#define SDMA_GFX_RB_BASE 0x1
#define SDMA_GFX_RB_BASE_HI 0x2
#define SDMA_GFX_RB_RPTR 0x3
#define SDMA_GFX_RB_WPTR 0x4
#define SDMA_GFX_RB_WPTR_POLL_CNTL 0x5
#define SDMA_GFX_RB_WPTR_POLL_ADDR_HI 0x6
#define SDMA_GFX_RB_WPTR_POLL_ADDR_LO 0x7
#define SDMA_GFX_RB_RPTR_ADDR_HI 0x8
#define SDMA_GFX_RB_RPTR_ADDR_LO 0x9
#define SDMA_GFX_IB_CNTL 0xA
#define SDMA_GFX_IB_RPTR 0xB
#define SDMA_F32_CNTL 0x2A

#define SDMA_UCODE_ADDR 0x58
#define SDMA_UCODE_DATA 0x59

#define SDMA_OPCODE_NOP             0
#define SDMA_OPCODE_COPY            1
#define SDMA_OPCODE_FENCE           5
#define SDMA_OPCODE_CONSTANT_FILL   11

#define SDMA_PACKET(op, sub_op, e)      ((((e) & 0xFFFF) << 16) | (((sub_op) & 0xFF) << 8) | ((op) & 0xFF))

static inline uint32_t sdmaReg(int i, uint32_t relOffset) {
    return (i == 0 ? SDMA0_BASE_OFFSET : SDMA1_BASE_OFFSET) + relOffset;
}

bool PolarisDevice::initializeSdmaMemory() {
    if (!aperBase || boBytesAvailable < sdmaVramSize) {
        return false;
    }
    
    sdmaVramBase = boCursor;
    boCursor += sdmaVramSize;
    boBytesAvailable -= sdmaVramSize;
    
    vaddr_t vramMapping = kernelSpace->mapPhysical(sdmaVramBase, sdmaVramSize, PROT_READ | PROT_WRITE);
    if (!vramMapping) return false;
    
    memset((void*)vramMapping, 0, sdmaVramSize);
    
    /* 4 pages: ring0, ring1, writeback pages, fence pages */
    for (size_t i = 0; i < numSdmaInstances; i++) {
        sdmaRings[i].ringPhysical = sdmaVramBase + i * PAGESIZE;
        sdmaRings[i].ringVirtual = vramMapping + i * PAGESIZE;
        
        sdmaRings[i].rptrWbPhysical = sdmaVramBase + 2 * PAGESIZE + i * 256;
        sdmaRings[i].rptrWbVirtual = (volatile uint32_t*)(vramMapping + 2 * PAGESIZE + i * 256);
        
        sdmaRings[i].fencePhysical = sdmaVramBase + 3 * PAGESIZE + i * 256;
        sdmaRings[i].fenceVirtual = (volatile uint32_t*)(vramMapping + 3 * PAGESIZE + i * 256);
        *sdmaRings[i].fenceVirtual = 0;
        
        sdmaRings[i].fenceSeq = 1;
        sdmaRings[i].wptr = 0;
        sdmaRings[i].initialized = false;
    }
    
    sdmaMemoryReady = true;
    return true;
}

void PolarisDevice::sdmaHalt(int i) {
    uint32_t cntl = readReg(sdmaReg(i, SDMA_F32_CNTL));
    cntl |= 1; /* HALT bit */
    writeReg(sdmaReg(i, SDMA_F32_CNTL), cntl);
}

void PolarisDevice::sdmaUnhalt(int i) {
    uint32_t cntl = readReg(sdmaReg(i, SDMA_F32_CNTL));
    cntl &= ~1U; /* HALT bit */
    writeReg(sdmaReg(i, SDMA_F32_CNTL), cntl);
}

bool PolarisDevice::sdmaLoadMicrocode(int i) {
    uint32_t firmwareBit = (i == 0) ? POLARIS_FIRMWARE_SDMA0 : POLARIS_FIRMWARE_SDMA1;
    PolarisFirmwareBlob* blob = getFirmwareBlob(firmwareBit);
    if (!blob || blob->size < 256) return false;
    
    sdmaHalt(i);
    writeReg(sdmaReg(i, SDMA_UCODE_ADDR), 0);
    
    /* Skip 256-byte amdgpu firmware header if present.  Some dumps might not have it, 
       but standard linux fw does. Usually fw is multiple of 4 bytes. 
       Let's assume the first 256 bytes is header. */
    size_t offset = 0;
    if (blob->size > 256 && *(uint32_t*)blob->data != 0) {
        offset = 256;
    }
    
    const uint32_t* ucode = (const uint32_t*)(blob->data + offset);
    size_t ucodeDwords = (blob->size - offset) / 4;
    
    for (size_t j = 0; j < ucodeDwords; j++) {
        writeReg(sdmaReg(i, SDMA_UCODE_DATA), ucode[j]);
    }
    
    writeReg(sdmaReg(i, SDMA_UCODE_ADDR), 0);
    return true;
}

bool PolarisDevice::initializeSdmaInstance(int i) {
    if (!sdmaMemoryReady) return false;
    if (!sdmaLoadMicrocode(i)) return false;
    
    writeReg(sdmaReg(i, SDMA_GFX_RB_CNTL), 0);
    
    uint64_t ringBase = sdmaRings[i].ringPhysical >> 8;
    writeReg(sdmaReg(i, SDMA_GFX_RB_BASE), (uint32_t)ringBase);
    writeReg(sdmaReg(i, SDMA_GFX_RB_BASE_HI), (uint32_t)(ringBase >> 32));
    
    writeReg(sdmaReg(i, SDMA_GFX_RB_RPTR), 0);
    writeReg(sdmaReg(i, SDMA_GFX_RB_WPTR), 0);
    
    uint64_t rptrAddr = sdmaRings[i].rptrWbPhysical;
    writeReg(sdmaReg(i, SDMA_GFX_RB_RPTR_ADDR_LO), (uint32_t)rptrAddr);
    writeReg(sdmaReg(i, SDMA_GFX_RB_RPTR_ADDR_HI), (uint32_t)(rptrAddr >> 32));
    
    /* Enable ring, size = 12 (2^12 dwords = 16 KiB? No, our ring is 4 KiB, so 2^10 dwords, meaning size = 10) */
    uint32_t rbCntl = 10 | (1 << 12) | (1 << 14); /* RB_SIZE | RB_RPTR_WRITEBACK_ENABLE | RPTR_WRITEBACK_TIMER */
    writeReg(sdmaReg(i, SDMA_GFX_RB_CNTL), rbCntl);
    
    sdmaUnhalt(i);
    
    uint32_t rbCntlEnable = rbCntl | 1; /* RB_ENABLE */
    writeReg(sdmaReg(i, SDMA_GFX_RB_CNTL), rbCntlEnable);
    
    /* Fill with NOPs */
    volatile uint32_t* ring = (volatile uint32_t*) sdmaRings[i].ringVirtual;
    for (size_t j = 0; j < polarisSdmaRingDwords; j++) {
        ring[j] = SDMA_PACKET(SDMA_OPCODE_NOP, 0, 0);
    }
    
    sdmaRings[i].initialized = true;
    return true;
}

void PolarisDevice::sdmaCommit(int i) {
    uint32_t wptr = sdmaRings[i].wptr;
    /* AMD SDMA wptr is in dwords, shifted by 2 bytes */
    writeReg(sdmaReg(i, SDMA_GFX_RB_WPTR), (wptr * 4) & 0x3FFFFC);
}

bool PolarisDevice::sdmaFenceAndWait(int i) {
    if (!sdmaRings[i].initialized) return false;
    
    uint32_t fenceSeq = sdmaRings[i].fenceSeq++;
    
    sdmaWrite(i, SDMA_PACKET(SDMA_OPCODE_FENCE, 0, 0));
    sdmaWrite(i, (uint32_t)sdmaRings[i].fencePhysical);
    sdmaWrite(i, (uint32_t)(sdmaRings[i].fencePhysical >> 32));
    sdmaWrite(i, fenceSeq);
    
    sdmaCommit(i);
    
    /* Wait for fence to complete (timeout ~1 second) */
    for (int timeout = 0; timeout < 1000000; timeout++) {
        if (*sdmaRings[i].fenceVirtual >= fenceSeq) {
            return true;
        }
        asm volatile("pause" ::: "memory");
    }
    
    return false;
}

bool PolarisDevice::sdmaFill(paddr_t dstPhysical, uint32_t fillData, uint32_t byteCount) {
    if (!sdmaRings[0].initialized) return false;
    
    sdmaWrite(0, SDMA_PACKET(SDMA_OPCODE_CONSTANT_FILL, 0, 0));
    sdmaWrite(0, (uint32_t)dstPhysical);
    sdmaWrite(0, (uint32_t)(dstPhysical >> 32));
    sdmaWrite(0, fillData);
    sdmaWrite(0, byteCount - 1);
    
    return sdmaFenceAndWait(0);
}

bool PolarisDevice::sdmaCopy(paddr_t dstPhysical, paddr_t srcPhysical, uint32_t byteCount) {
    if (!sdmaRings[0].initialized) return false;
    
    /* Sub-op 0: linear copy */
    sdmaWrite(0, SDMA_PACKET(SDMA_OPCODE_COPY, 0, 0));
    sdmaWrite(0, byteCount - 1);
    sdmaWrite(0, 0); /* src/dst endian swaps (0) */
    sdmaWrite(0, (uint32_t)srcPhysical);
    sdmaWrite(0, (uint32_t)(srcPhysical >> 32));
    sdmaWrite(0, (uint32_t)dstPhysical);
    sdmaWrite(0, (uint32_t)(dstPhysical >> 32));
    
    return sdmaFenceAndWait(0);
}

bool PolarisDevice::sdmaBlitBoToFb(const PolarisBufferObject* srcBo,
                    size_t srcOffset, uint32_t srcPitch,
                    unsigned srcX, unsigned srcY,
                    unsigned dstX, unsigned dstY,
                    unsigned w, unsigned h,
                    unsigned bytesPerPixel) {
    if (!srcBo || !hasBootFramebuffer || !sdmaRings[0].initialized) return false;
    
    for (unsigned line = 0; line < h; line++) {
        paddr_t srcRow = srcBo->physical + srcOffset + (srcY + line) * srcPitch + srcX * bytesPerPixel;
        paddr_t dstRow = framebufferPhysical + (dstY + line) * framebufferPitch + dstX * bytesPerPixel;
        
        uint32_t rowBytes = w * bytesPerPixel;
        
        sdmaWrite(0, SDMA_PACKET(SDMA_OPCODE_COPY, 0, 0));
        sdmaWrite(0, rowBytes - 1);
        sdmaWrite(0, 0);
        sdmaWrite(0, (uint32_t)srcRow);
        sdmaWrite(0, (uint32_t)(srcRow >> 32));
        sdmaWrite(0, (uint32_t)dstRow);
        sdmaWrite(0, (uint32_t)(dstRow >> 32));
    }
    
    return sdmaFenceAndWait(0);
}

bool PolarisDevice::gpuFillRect(uint32_t rgbaColor, unsigned x, unsigned y, unsigned w, unsigned h) {
    if (!hasBootFramebuffer || !sdmaRings[0].initialized) return false;

    unsigned bytesPerPixel = framebufferMode.video_bpp / 8;

    /* Row-by-row SDMA CONSTANT_FILL (max 0x3FFFE0 bytes per operation) */
    for (unsigned line = 0; line < h; line++) {
        paddr_t dstRow = framebufferPhysical + (y + line) * framebufferPitch +
                x * bytesPerPixel;
        uint32_t rowBytes = w * bytesPerPixel;
        if (!rowBytes) continue;

        sdmaWrite(0, SDMA_PACKET(SDMA_OPCODE_CONSTANT_FILL, 0, 0));
        sdmaWrite(0, (uint32_t)dstRow);
        sdmaWrite(0, (uint32_t)(dstRow >> 32));
        sdmaWrite(0, rgbaColor);
        sdmaWrite(0, rowBytes - 1);
    }

    return sdmaFenceAndWait(0);
}

bool PolarisDevice::gpuBlit(unsigned srcX, unsigned srcY,
                             unsigned dstX, unsigned dstY,
                             unsigned w, unsigned h) {
    if (!hasBootFramebuffer || !sdmaRings[0].initialized) return false;

    unsigned bytesPerPixel = framebufferMode.video_bpp / 8;
    uint32_t rowBytes = w * bytesPerPixel;
    if (!rowBytes || !h) return true;

    /* Determine copy direction to avoid overlap corruption.
     * If dst comes after src in memory and they overlap, copy from bottom. */
    bool bottomUp = (dstY > srcY) || (dstY == srcY && dstX > srcX);
    if (bottomUp) {
        for (unsigned line = h; line > 0; line--) {
            unsigned l = line - 1;
            paddr_t src = framebufferPhysical + (srcY + l) * framebufferPitch +
                    srcX * bytesPerPixel;
            paddr_t dst = framebufferPhysical + (dstY + l) * framebufferPitch +
                    dstX * bytesPerPixel;
            sdmaWrite(0, SDMA_PACKET(SDMA_OPCODE_COPY, 0, 0));
            sdmaWrite(0, rowBytes - 1);
            sdmaWrite(0, 0);
            sdmaWrite(0, (uint32_t)src);
            sdmaWrite(0, (uint32_t)(src >> 32));
            sdmaWrite(0, (uint32_t)dst);
            sdmaWrite(0, (uint32_t)(dst >> 32));
        }
    } else {
        for (unsigned line = 0; line < h; line++) {
            paddr_t src = framebufferPhysical + (srcY + line) * framebufferPitch +
                    srcX * bytesPerPixel;
            paddr_t dst = framebufferPhysical + (dstY + line) * framebufferPitch +
                    dstX * bytesPerPixel;
            sdmaWrite(0, SDMA_PACKET(SDMA_OPCODE_COPY, 0, 0));
            sdmaWrite(0, rowBytes - 1);
            sdmaWrite(0, 0);
            sdmaWrite(0, (uint32_t)src);
            sdmaWrite(0, (uint32_t)(src >> 32));
            sdmaWrite(0, (uint32_t)dst);
            sdmaWrite(0, (uint32_t)(dst >> 32));
        }
    }

    return sdmaFenceAndWait(0);
}

/* ==========================================================================
 * EDID reading and parsing
 *
 * Two source paths are tried in order:
 *
 *  1. DCE hardware I2C (DDC) — Polaris uses the DCE 11.2 display engine.
 *     The I2C controller for DDC1 (connector 0) is accessed via the
 *     DC_I2C_* registers in the MMIO aperture.  We perform a standard
 *     I2C read of the DDC slave (address 0x50) to fetch 128 bytes of EDID.
 *
 *  2. ATOM VBIOS scan — many AMD VBIOSes embed the monitor's EDID somewhere
 *     inside the image.  We scan for the EDID signature and copy the first
 *     valid block found.
 *
 * After obtaining the raw bytes the block is fully validated (checksum,
 * version) and then parsed:
 *   - All four Detailed Timing Descriptors (DTDs) at 0x36/0x48/0x5A/0x6C
 *   - Monitor Name descriptor (tag 0xFC) — logged
 *   - Monitor Range Limits descriptor (tag 0xFD) — logged
 *   - CEA-861 extension block (if present and readable via DDC)
 *
 * The preferred (native) resolution is taken from the first DTD with a
 * non-zero pixel clock, which per the EDID specification is always the
 * display's preferred timing.
 * ========================================================================== */

/* --------------------------------------------------------------------------
 * DCE 11.2 I2C register offsets (relative to MMIO base, in DWORD units).
 *
 * These match the register map used by amdgpu/dce_v11_0.c in Linux for
 * Polaris (GFX8 / VI family, DCE 11.2).
 *
 * DC_I2C_CONTROL         0x1A00  — engine control / transaction trigger
 * DC_I2C_ARBITRATION     0x1A01  — SW/HW arbitration
 * DC_I2C_STATUS          0x1A02  — transaction status
 * DC_I2C_SPEED           0x1A03  — SCL speed (100 kHz standard)
 * DC_I2C_SETUP           0x1A04  — timing setup
 * DC_I2C_TRANSACTION0    0x1A10  — first transaction descriptor
 * DC_I2C_DATA            0x1A20  — data FIFO (write address, then read data)
 * -------------------------------------------------------------------------- */

#define DC_I2C_CONTROL      0x1A00U
#define DC_I2C_ARBITRATION  0x1A01U
#define DC_I2C_STATUS       0x1A02U
#define DC_I2C_SPEED        0x1A03U
#define DC_I2C_SETUP        0x1A04U
#define DC_I2C_TRANSACTION0 0x1A10U
#define DC_I2C_DATA         0x1A20U

/* DC_I2C_CONTROL bits */
#define DC_I2C_GO           (1U << 0)   /* start transaction */
#define DC_I2C_SOFT_RESET   (1U << 1)   /* soft-reset engine */
#define DC_I2C_SEND_RESET   (1U << 2)   /* send 9 SCL pulses */
#define DC_I2C_SW_STATUS_RESET (1U << 3)
#define DC_I2C_TRANSACTION_COUNT_SHIFT 20
#define DC_I2C_DDC_SELECT_SHIFT 8       /* bits [10:8] = DDC line select */

/* DC_I2C_STATUS bits */
#define DC_I2C_STATUS_DONE  (1U << 0)
#define DC_I2C_STATUS_ABORTED (1U << 1)
#define DC_I2C_STATUS_TIMEOUT (1U << 2)
#define DC_I2C_STATUS_STOPPED (1U << 4)
#define DC_I2C_STATUS_BUSY  (1U << 8)

/* DC_I2C_TRANSACTION bits */
#define DC_I2C_RW           (1U << 0)   /* 0=write, 1=read */
#define DC_I2C_STOP_ON_NACK (1U << 8)
#define DC_I2C_START        (1U << 12)
#define DC_I2C_STOP         (1U << 13)
#define DC_I2C_COUNT_SHIFT  16          /* byte count in bits [23:16] */

/* DC_I2C_DATA bits */
#define DC_I2C_DATA_RW      (1U << 0)   /* 0=write to FIFO, 1=read from FIFO */
#define DC_I2C_DATA_INDEX_WRITE (1U << 31) /* auto-increment index on write */
#define DC_I2C_INDEX_SHIFT  8

/* DDC slave address for EDID */
#define DDC_EDID_ADDR       0x50U

/* Maximum spin iterations waiting for DC_I2C_STATUS_DONE (~5 ms at 1 GHz) */
#define DC_I2C_TIMEOUT_ITERS 5000000U

bool PolarisDevice::readEdidViaDdc(uint8_t* edidBuf) {
    if (!mmioBase || mmioSize < (DC_I2C_DATA + 1) * 4) return false;

    /* --- Reset the I2C engine --- */
    writeReg(DC_I2C_CONTROL,
             DC_I2C_SOFT_RESET | DC_I2C_SW_STATUS_RESET);
    /* Small delay: spin a few iterations */
    for (volatile int d = 0; d < 1000; d++) {}
    writeReg(DC_I2C_CONTROL, DC_I2C_SW_STATUS_RESET);

    /* --- Set speed to 100 kHz (standard DDC) ---
     * The field encoding is GPU-clock-dependent; Linux uses 0x11 for 100 kHz
     * on most DCE 11.x parts.  We write the same value. */
    writeReg(DC_I2C_SPEED, 0x11U);

    /* --- Arbitration: request SW control --- */
    writeReg(DC_I2C_ARBITRATION, 0x00000001U);

    /* --- Build the two-transaction sequence:
     *   Tx0: WRITE 1 byte (register address 0x00) to slave 0x50
     *   Tx1: READ  128 bytes from slave 0x51 (read address = write addr | 1)
     * --- */

    /* Transaction 0: write the EDID register offset (0x00) */
    uint32_t tx0 = (0U << DC_I2C_COUNT_SHIFT) | /* count=1 written via DATA */
                   DC_I2C_START |
                   DC_I2C_STOP_ON_NACK;
    /* count field is actually byte count - 1 for writes; Linux uses count=1
     * meaning 1 data byte (the register address byte). */
    tx0 = (1U << DC_I2C_COUNT_SHIFT) | DC_I2C_START | DC_I2C_STOP_ON_NACK;
    writeReg(DC_I2C_TRANSACTION0, tx0);

    /* Transaction 1: read 128 bytes */
    uint32_t tx1 = (128U << DC_I2C_COUNT_SHIFT) |
                   DC_I2C_RW |
                   DC_I2C_START |
                   DC_I2C_STOP |
                   DC_I2C_STOP_ON_NACK;
    writeReg(DC_I2C_TRANSACTION0 + 1, tx1);

    /* --- Fill the write FIFO ---
     * Index 0: slave write address (DDC_EDID_ADDR << 1 | 0)
     * Index 1: EDID register offset = 0x00
     * Index 2: slave read address  (DDC_EDID_ADDR << 1 | 1)
     */
    writeReg(DC_I2C_DATA,
             DC_I2C_DATA_INDEX_WRITE | (0U << DC_I2C_INDEX_SHIFT) | 0U);
    writeReg(DC_I2C_DATA, (DDC_EDID_ADDR << 1) & 0xFEU);  /* write addr */
    writeReg(DC_I2C_DATA, 0x00U);                           /* register 0 */
    writeReg(DC_I2C_DATA, ((DDC_EDID_ADDR << 1) | 0x01U)); /* read addr */

    /* --- Trigger: 2 transactions, DDC line 0 --- */
    uint32_t ctrl = DC_I2C_GO |
                    (1U << DC_I2C_TRANSACTION_COUNT_SHIFT) | /* 0-based: 1 = 2 txns */
                    (0U << DC_I2C_DDC_SELECT_SHIFT);         /* DDC1 */
    writeReg(DC_I2C_CONTROL, ctrl);

    /* --- Wait for completion --- */
    uint32_t status = 0;
    for (uint32_t iter = 0; iter < DC_I2C_TIMEOUT_ITERS; iter++) {
        status = readReg(DC_I2C_STATUS);
        if (status & DC_I2C_STATUS_DONE) break;
        if (status & (DC_I2C_STATUS_ABORTED | DC_I2C_STATUS_TIMEOUT |
                      DC_I2C_STATUS_STOPPED)) break;
    }

    if (!(status & DC_I2C_STATUS_DONE) ||
         (status & (DC_I2C_STATUS_ABORTED | DC_I2C_STATUS_TIMEOUT))) {
        /* Reset engine before returning */
        writeReg(DC_I2C_CONTROL,
                 DC_I2C_SOFT_RESET | DC_I2C_SW_STATUS_RESET);
        writeReg(DC_I2C_ARBITRATION, 0U);
        return false;
    }

    /* --- Drain the read FIFO ---
     * Set index to 0 in read mode and read 128 bytes. */
    writeReg(DC_I2C_DATA,
             DC_I2C_DATA_RW | DC_I2C_DATA_INDEX_WRITE |
             (0U << DC_I2C_INDEX_SHIFT));
    for (int b = 0; b < 128; b++) {
        uint32_t word = readReg(DC_I2C_DATA);
        edidBuf[b] = (uint8_t)(word & 0xFFU);
    }

    /* Release arbitration */
    writeReg(DC_I2C_ARBITRATION, 0U);

    /* Validate checksum */
    uint8_t sum = 0;
    for (int k = 0; k < 128; k++) sum += edidBuf[k];
    if (sum != 0) return false;

    /* Validate EDID header signature */
    static const uint8_t edidSig[8] = {
        0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00
    };
    if (memcmp(edidBuf, edidSig, 8) != 0) return false;

    return true;
}

bool PolarisDevice::readEdidExtensionViaDdc(uint8_t* edidBuf) {
    /* edidBuf[0..127] is already a valid base block.
     * Extension count is at byte 0x7E. */
    if (edidBuf[0x7E] == 0) return false;
    if (!mmioBase || mmioSize < (DC_I2C_DATA + 1) * 4) return false;

    /* Reset */
    writeReg(DC_I2C_CONTROL,
             DC_I2C_SOFT_RESET | DC_I2C_SW_STATUS_RESET);
    for (volatile int d = 0; d < 1000; d++) {}
    writeReg(DC_I2C_CONTROL, DC_I2C_SW_STATUS_RESET);
    writeReg(DC_I2C_SPEED, 0x11U);
    writeReg(DC_I2C_ARBITRATION, 0x00000001U);

    /* Write register offset = 0x80 (block 1 starts at byte 128) */
    uint32_t tx0 = (1U << DC_I2C_COUNT_SHIFT) | DC_I2C_START | DC_I2C_STOP_ON_NACK;
    writeReg(DC_I2C_TRANSACTION0, tx0);
    uint32_t tx1 = (128U << DC_I2C_COUNT_SHIFT) |
                   DC_I2C_RW | DC_I2C_START | DC_I2C_STOP | DC_I2C_STOP_ON_NACK;
    writeReg(DC_I2C_TRANSACTION0 + 1, tx1);

    writeReg(DC_I2C_DATA,
             DC_I2C_DATA_INDEX_WRITE | (0U << DC_I2C_INDEX_SHIFT) | 0U);
    writeReg(DC_I2C_DATA, (DDC_EDID_ADDR << 1) & 0xFEU);
    writeReg(DC_I2C_DATA, 0x80U); /* block 1 offset */
    writeReg(DC_I2C_DATA, ((DDC_EDID_ADDR << 1) | 0x01U));

    uint32_t ctrl = DC_I2C_GO |
                    (1U << DC_I2C_TRANSACTION_COUNT_SHIFT) |
                    (0U << DC_I2C_DDC_SELECT_SHIFT);
    writeReg(DC_I2C_CONTROL, ctrl);

    uint32_t status = 0;
    for (uint32_t iter = 0; iter < DC_I2C_TIMEOUT_ITERS; iter++) {
        status = readReg(DC_I2C_STATUS);
        if (status & DC_I2C_STATUS_DONE) break;
        if (status & (DC_I2C_STATUS_ABORTED | DC_I2C_STATUS_TIMEOUT |
                      DC_I2C_STATUS_STOPPED)) break;
    }

    if (!(status & DC_I2C_STATUS_DONE) ||
         (status & (DC_I2C_STATUS_ABORTED | DC_I2C_STATUS_TIMEOUT))) {
        writeReg(DC_I2C_CONTROL,
                 DC_I2C_SOFT_RESET | DC_I2C_SW_STATUS_RESET);
        writeReg(DC_I2C_ARBITRATION, 0U);
        return false;
    }

    writeReg(DC_I2C_DATA,
             DC_I2C_DATA_RW | DC_I2C_DATA_INDEX_WRITE |
             (0U << DC_I2C_INDEX_SHIFT));
    for (int b = 0; b < 128; b++) {
        uint32_t word = readReg(DC_I2C_DATA);
        edidBuf[128 + b] = (uint8_t)(word & 0xFFU);
    }

    writeReg(DC_I2C_ARBITRATION, 0U);

    /* Validate extension block checksum */
    uint8_t sum = 0;
    for (int k = 0; k < 128; k++) sum += edidBuf[128 + k];
    return sum == 0;
}

bool PolarisDevice::findEdidInVbios(uint8_t* edidBuf) {
    if (!bios || biosSize < 128) return false;

    static const uint8_t edidSig[8] = {
        0x00, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00
    };

    for (size_t i = 0; i + 128 <= biosSize; i++) {
        if (memcmp(bios + i, edidSig, 8) != 0) continue;

        const uint8_t* candidate = bios + i;

        /* Verify checksum */
        uint8_t sum = 0;
        for (int k = 0; k < 128; k++) sum += candidate[k];
        if (sum != 0) continue;

        /* Require EDID version 1 */
        if (candidate[0x12] != 1) continue;

        memcpy(edidBuf, candidate, 128);
        return true;
    }
    return false;
}

/* --------------------------------------------------------------------------
 * Parse a single 18-byte Detailed Timing Descriptor.
 * Returns true and fills out *w, *h, *refresh if it is a valid timing DTD.
 * -------------------------------------------------------------------------- */
static bool parseDtd(const uint8_t* dtd,
                     uint16_t* w, uint16_t* h, uint32_t* refresh) {
    uint16_t pixelClock10kHz = (uint16_t)dtd[0] | ((uint16_t)dtd[1] << 8);
    if (pixelClock10kHz == 0) return false; /* descriptor, not timing */

    uint16_t hActive   = (uint16_t)dtd[2] | (((uint16_t)(dtd[4] >> 4)) << 8);
    uint16_t hBlanking = (uint16_t)dtd[3] | (((uint16_t)(dtd[4] & 0xF)) << 8);
    uint16_t vActive   = (uint16_t)dtd[5] | (((uint16_t)(dtd[7] >> 4)) << 8);
    uint16_t vBlanking = (uint16_t)dtd[6] | (((uint16_t)(dtd[7] & 0xF)) << 8);

    if (hActive == 0 || vActive == 0) return false;

    unsigned totalH = hActive + hBlanking;
    unsigned totalV = vActive + vBlanking;
    uint32_t pclk   = (uint32_t)pixelClock10kHz * 10000U; /* Hz */
    uint32_t denom  = (uint32_t)totalH * (uint32_t)totalV;
    uint32_t ref    = denom ? (pclk + denom / 2) / denom : 60U;
    if (ref == 0) ref = 60U;

    *w = hActive;
    *h = vActive;
    *refresh = ref;
    return true;
}

void PolarisDevice::parseEdidBlock(const uint8_t* edid) {
    /* Decode manufacturer ID (3 letters packed into 2 bytes, big-endian).
     * Bits [14:10] = letter1-'A'+1, [9:5] = letter2, [4:0] = letter3. */
    uint16_t mfgRaw = ((uint16_t)edid[8] << 8) | edid[9];
    char mfg[4];
    mfg[0] = (char)('@' + ((mfgRaw >> 10) & 0x1F));
    mfg[1] = (char)('@' + ((mfgRaw >>  5) & 0x1F));
    mfg[2] = (char)('@' + ((mfgRaw >>  0) & 0x1F));
    mfg[3] = '\0';

    uint16_t productCode = (uint16_t)edid[10] | ((uint16_t)edid[11] << 8);
    uint8_t  edidVersion  = edid[0x12];
    uint8_t  edidRevision = edid[0x13];

    Log::printf("%s GPU at %u/%u/%u: EDID v%u.%u, manufacturer %s "
                "product 0x%04X\n",
                getAsicName(asicType), bus, device, function,
                edidVersion, edidRevision, mfg, (unsigned)productCode);

    /* Scan the four 18-byte descriptor slots at 0x36, 0x48, 0x5A, 0x6C */
    char monitorName[14] = {};
    bool foundName = false;

    for (int slot = 0; slot < 4; slot++) {
        const uint8_t* desc = edid + 0x36 + slot * 18;

        /* Check if this is a timing descriptor (pixel clock != 0) */
        uint16_t pclkField = (uint16_t)desc[0] | ((uint16_t)desc[1] << 8);
        if (pclkField != 0) {
            /* Timing descriptor — only use slot 0 (preferred timing) */
            if (slot == 0 && !hasNativeMode) {
                uint16_t w, h;
                uint32_t ref;
                if (parseDtd(desc, &w, &h, &ref)) {
                    nativeWidth  = w;
                    nativeHeight = h;
                    nativeRefresh = ref;
                    hasNativeMode = true;
                }
            }
            continue;
        }

        /* Non-timing descriptor: bytes [0..3] == 0x00 0x00 0x00 <tag> 0x00 */
        uint8_t tag = desc[3];

        if (tag == 0xFC && !foundName) {
            /* Monitor Name descriptor */
            foundName = true;
            int len = 0;
            for (int c = 0; c < 13; c++) {
                uint8_t ch = desc[5 + c];
                if (ch == 0x0A) break; /* newline terminates */
                monitorName[len++] = (char)ch;
            }
            monitorName[len] = '\0';
        } else if (tag == 0xFD) {
            /* Monitor Range Limits descriptor */
            uint8_t minVHz  = desc[5];
            uint8_t maxVHz  = desc[6];
            uint8_t minHkHz = desc[7];
            uint8_t maxHkHz = desc[8];
            uint8_t maxPclkMHz = (uint8_t)(desc[9] * 10);
            Log::printf("%s GPU at %u/%u/%u: EDID range: V %u-%u Hz, "
                        "H %u-%u kHz, max pclk %u MHz\n",
                        getAsicName(asicType), bus, device, function,
                        minVHz, maxVHz, minHkHz, maxHkHz, maxPclkMHz);
        }
    }

    if (foundName && monitorName[0]) {
        Log::printf("%s GPU at %u/%u/%u: monitor name: \"%s\"\n",
                    getAsicName(asicType), bus, device, function, monitorName);
    }

    if (hasNativeMode) {
        Log::printf("%s GPU at %u/%u/%u: native resolution %ux%u@%uHz "
                    "(from EDID)\n",
                    getAsicName(asicType), bus, device, function,
                    nativeWidth, nativeHeight, nativeRefresh);
    } else {
        Log::printf("%s GPU at %u/%u/%u: EDID found but no valid preferred "
                    "timing descriptor\n",
                    getAsicName(asicType), bus, device, function);
    }
}

void PolarisDevice::readAndParseEdid() {
    uint8_t buf[256];
    memset(buf, 0, sizeof(buf));

    /*
     * During driver initialisation we only use the safe VBIOS scan path.
     * The DCE hardware I2C (DDC) path is available on demand via the
     * POLARIS_GET_EDID devctl — calling it here during early boot would
     * spin on DC_I2C_STATUS before the display engine is fully initialised,
     * causing a multi-second hang and a black screen.
     */

    /* --- VBIOS scan --- */
    if (findEdidInVbios(buf)) {
        edidSource = POLARIS_EDID_SOURCE_VBIOS;
        edidSize   = 128;
        memcpy(edidData, buf, 128);

        Log::printf("%s GPU at %u/%u/%u: EDID found in VBIOS image "
                    "(%u bytes)\n",
                    getAsicName(asicType), bus, device, function, edidSize);
        parseEdidBlock(edidData);
        return;
    }

    Log::printf("%s GPU at %u/%u/%u: no EDID in VBIOS; use POLARIS_GET_EDID "
                "devctl after boot to read via DDC\n",
                getAsicName(asicType), bus, device, function);
}
