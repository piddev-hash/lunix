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

#include <stddef.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <lunix/kernel/addressspace.h>
#include <lunix/kernel/log.h>
#include <lunix/kernel/pci.h>
#include <lunix/kernel/polaris.h>

#define PCI_COMMAND_MEMORY_SPACE 0x2
#define PCI_COMMAND_BUS_MASTER 0x4
#define PCI_ROM_ADDRESS_ENABLE 0x1

namespace {

static const char* getAsicName(PolarisAsic asicType) {
    switch (asicType) {
    case POLARIS10: return "Polaris10";
    case POLARIS11: return "Polaris11";
    case POLARIS12: return "Polaris12";
    }

    return "Polaris";
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

    uint64_t base = ((uint64_t) originalHigh << 32) | (originalLow & ~0xFULL);
    uint64_t mask = ((uint64_t) sizeHigh << 32) | (sizeLow & ~0xFULL);
    uint64_t barSize = (~mask) + 1;
    if (!base || !barSize) return false;

    baseAddress = base;
    size = barSize;
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
        asicType(asicType), bus(bus), device(device), function(function) {
    if (!enableDevice() || !initializeBars()) {
        FAIL_CONSTRUCTOR;
    }

    if (loadAtomBios()) {
        Log::printf("%s GPU at %u/%u/%u: loaded %zu bytes of ATOM VBIOS\n",
                getAsicName(asicType), bus, device, function, biosSize);
    } else {
        Log::printf("%s GPU at %u/%u/%u: MMIO ready, but ATOM VBIOS was not "
                "found via VRAM/ROM BAR\n", getAsicName(asicType), bus,
                device, function);
    }
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
    if (graphicsDriver) {
        Log::printf("%s GPU at %u/%u/%u found, but another graphics driver is "
                "already active\n", getAsicName(asicType), bus, device,
                function);
        return;
    }

    PolarisDevice* deviceObject = new PolarisDevice(asicType, bus, device,
            function);
    if (!deviceObject) {
        Log::printf("%s GPU at %u/%u/%u detected, but initialization failed\n",
                getAsicName(asicType), bus, device, function);
        return;
    }

    graphicsDriver = deviceObject;
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

bool PolarisDevice::isSupportedMode(video_mode /*mode*/) {
    // Linux's Polaris modesetting stack depends on much more infrastructure
    // than this kernel currently has, so for now we only initialize the GPU
    // and VBIOS access paths.
    return false;
}

vaddr_t PolarisDevice::setVideoMode(video_mode* /*mode*/) {
    return 0;
}
