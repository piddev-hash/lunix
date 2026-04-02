/* Copyright (c) 2018, 2019, 2020, 2021, 2022 Dennis Wölfing
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

/* kernel/src/display.cpp
 * Display class.
 */

#include <errno.h>
#include <stdlib.h>
#include <string.h>
#include <lunix/kernel/addressspace.h>
#include <lunix/kernel/console.h>
#include <lunix/kernel/devices.h>
#include <lunix/kernel/display.h>
#include <lunix/kernel/panic.h>
#include <lunix/kernel/portio.h>
#include <lunix/kernel/process.h>
#include "../../libdxui/src/cp437.h"

// Classical VGA font but with the Unicode replacement character at 0xFF.
asm(".pushsection .rodata\n"
".global vgafont\n"
"vgafont:\n\t"
    ".incbin \"../vgafont\"\n"
".popsection");
extern const uint8_t vgafont[];
GraphicsDriver* graphicsDriver;
static BootFramebufferInfo bootFramebufferInfo;
static bool haveBootFramebufferInfo;

static const size_t charHeight = 16;
static const size_t charWidth = 9;

void setBootFramebufferInfo(paddr_t physicalAddress, vaddr_t virtualAddress,
        size_t size, size_t pitch, video_mode mode) {
    bootFramebufferInfo.physicalAddress = physicalAddress;
    bootFramebufferInfo.virtualAddress = virtualAddress;
    bootFramebufferInfo.size = size;
    bootFramebufferInfo.pitch = pitch;
    bootFramebufferInfo.mode = mode;
    haveBootFramebufferInfo = true;
}

bool getBootFramebufferInfo(BootFramebufferInfo& info) {
    if (!haveBootFramebufferInfo) return false;
    info = bootFramebufferInfo;
    return true;
}

Display::Display(video_mode mode, char* buffer, size_t pitch)
        : Vnode(S_IFCHR | 0666, DevFS::dev) {
    this->buffer = buffer;
    this->mode = mode;
    if (mode.video_bpp == 0) {
        rows = mode.video_height;
        columns = mode.video_width;
    } else {
        rows = mode.video_height / charHeight;
        columns = (mode.video_width + 1) / charWidth;
    }
    this->pitch = pitch;
    cursorPos = {0, 0};
    cursorVisible = true;
    doubleBuffer = nullptr;
    primaryBuffer = nullptr;
    alternateBuffer = nullptr;
    invalidated = false;
    renderingText = true;
    haveOldBuffer = true;
    changingResolution = false;
    displayOwner = nullptr;
}

ALWAYS_INLINE char* Display::charAddress(CharPos position) {
    return buffer + position.y * charHeight * pitch +
            position.x * charWidth * mode.video_bpp / 8;
}

ALWAYS_INLINE void Display::setPixelColor(char* addr,
        uint32_t rgbColor) {
    if ((rgbColor & 0xFF000000) == 0) return;
    if (mode.video_bpp == 32) {
        uint32_t* pixel = (uint32_t*) addr;
        *pixel = rgbColor;
    } else {
        addr[0] = (rgbColor >> 0) & 0xFF;
        addr[1] = (rgbColor >> 8) & 0xFF;
        addr[2] = (rgbColor >> 16) & 0xFF;
    }
}

static ALWAYS_INLINE bool hasVisiblePixels(const uint32_t* src, size_t width) {
    for (size_t i = 0; i < width; i++) {
        if (src[i] & 0xFF000000U) return true;
    }
    return false;
}

void Display::clear(CharPos from, CharPos to, Color color) {
    size_t bufferStart = from.x + columns * from.y;
    size_t bufferEnd = to.x + columns * to.y;
    for (size_t i = bufferStart; i <= bufferEnd; i++) {
        if (doubleBuffer[i].wc != L'\0' || doubleBuffer[i].color != color) {
            doubleBuffer[i].wc = L'\0';
            doubleBuffer[i].color = color;
            doubleBuffer[i].modified = true;
        }
    }
}

video_mode Display::getVideoMode() {
    AutoLock lock(&mutex);
    return mode;
}

void Display::initialize() {
    primaryBuffer = (CharBufferEntry*) malloc(rows * columns *
            sizeof(CharBufferEntry));
    if (!primaryBuffer) PANIC("Allocation failure");
    alternateBuffer = (CharBufferEntry*) malloc(rows * columns *
            sizeof(CharBufferEntry));
    if (!alternateBuffer) PANIC("Allocation failure");
    doubleBuffer = primaryBuffer;
    Color defaultColor = { RGB(170, 170, 170), RGB(0, 0, 0), 0x07 };
    clear({0, 0}, {columns - 1, rows - 1}, defaultColor);
}

void Display::onPanic() {
    renderingText = true;
    update();
}

void Display::putCharacter(CharPos position, wchar_t wc, Color color) {
    if (unlikely(!doubleBuffer)) {
        CharBufferEntry entry = { wc, color.fgColor, color.bgColor, true };
        redraw(position, &entry);
        return;
    }

    doubleBuffer[position.x + columns * position.y].wc = wc;
    doubleBuffer[position.x + columns * position.y].color = color;
    doubleBuffer[position.x + columns * position.y].modified = true;
}

void Display::redraw(CharPos position) {
    CharBufferEntry* entry = &doubleBuffer[position.x + columns * position.y];
    redraw(position, entry);
}

void Display::redraw(CharPos position, CharBufferEntry* entry) {
    entry->modified = false;
    wchar_t wc = entry->wc;

    if (mode.video_bpp == 0) {
        uint8_t vgaColor = entry->color.vgaColor;
        uint8_t cp437 = unicodeToCp437(wc);
        if (cp437 == 0xFF) {
            // Print unrepresentable characters as ? with inverted colors.
            cp437 = '?';
            vgaColor = ((vgaColor & 0x0F) << 4) | ((vgaColor & 0xF0) >> 4);
        }
        char* addr = buffer + 2 * position.y * 80 + 2 * position.x;
        addr[0] = cp437;
        addr[1] = vgaColor;
        return;
    }

    uint32_t foreground = entry->color.fgColor;
    uint32_t background = entry->color.bgColor;
    uint8_t cp437 = unicodeToCp437(wc);
    const uint8_t* charFont = vgafont + cp437 * 16;
    char* addr = charAddress(position);

    for (size_t i = 0; i < 16; i++) {
        for (size_t j = 0; j < 8; j++) {
            bool pixelFg = charFont[i] & (1 << (7 - j)) ||
                    (cursorVisible && position == cursorPos && i >= 14);
            uint32_t rgbColor = pixelFg ? foreground : background;
            setPixelColor(&addr[j * mode.video_bpp / 8], rgbColor);
        }

        if (likely((position.x + 1) * charWidth <= mode.video_width)) {
            bool pixelFg = cp437 >= 0xB0 && cp437 <= 0xDF && charFont[i] & 1;
            uint32_t rgbColor = pixelFg ? foreground : background;
            setPixelColor(&addr[8 * mode.video_bpp / 8], rgbColor);
        }
        addr += pitch;
    }
}

void Display::releaseDisplay() {
    AutoLock lock(&mutex);
    assert(displayOwner);
    displayOwner->ownsDisplay = false;
    displayOwner = nullptr;
    renderingText = true;
    invalidated = true;
}

void Display::scroll(unsigned int lines, Color color, bool up /*= true*/) {
    CharBufferEntry empty;
    empty.wc = L'\0';
    empty.color = color;

    if (up) {
        for (unsigned int y = 0; y < rows; y++) {
            for (unsigned int x = 0; x < columns; x++) {
                CharBufferEntry& entry = y < rows - lines ?
                        doubleBuffer[x + (y + lines) * columns] : empty;
                if (doubleBuffer[x + y * columns] != entry) {
                    doubleBuffer[x + y * columns].wc = entry.wc;
                    doubleBuffer[x + y * columns].color = entry.color;
                    doubleBuffer[x + y * columns].modified = true;
                }
            }
        }

        /* GPU-accelerated scroll: blit framebuffer content up, then GPU-fill
         * the newly blank strip at the bottom. */
        if (graphicsDriver && mode.video_bpp != 0) {
            unsigned scrollPx = lines * charHeight;
            unsigned keepH = mode.video_height > scrollPx ?
                    mode.video_height - scrollPx : 0;
            if (keepH && graphicsDriver->gpuBlit(0, scrollPx, 0, 0,
                    mode.video_width, keepH)) {
                /* Clear the newly exposed strip */
                uint32_t bgRgba = color.bgColor | 0xFF000000U;
                graphicsDriver->gpuFillRect(bgRgba,
                        0, keepH,
                        mode.video_width, scrollPx);
                /* Mark all character cells that were moved as clean
                 * (the framebuffer already reflects them via GPU). */
                for (unsigned int y = 0; y < rows - lines; y++) {
                    for (unsigned int x = 0; x < columns; x++) {
                        doubleBuffer[x + y * columns].modified = false;
                    }
                }
            }
        }
    } else {
        for (unsigned int y = rows - 1; y < rows; y--) {
            for (unsigned int x = 0; x < columns; x++) {
                CharBufferEntry& entry = y >= lines ?
                        doubleBuffer[x + (y - lines) * columns] : empty;
                if (doubleBuffer[x + y * columns] != entry) {
                    doubleBuffer[x + y * columns].wc = entry.wc;
                    doubleBuffer[x + y * columns].color = entry.color;
                    doubleBuffer[x + y * columns].modified = true;
                }
            }
        }

        if (graphicsDriver && mode.video_bpp != 0) {
            unsigned scrollPx = lines * charHeight;
            unsigned keepH = mode.video_height > scrollPx ?
                    mode.video_height - scrollPx : 0;
            if (keepH && graphicsDriver->gpuBlit(0, 0, 0, scrollPx,
                    mode.video_width, keepH)) {
                uint32_t bgRgba = color.bgColor | 0xFF000000U;
                graphicsDriver->gpuFillRect(bgRgba,
                        0, 0,
                        mode.video_width, scrollPx);
                for (unsigned int y = lines; y < rows; y++) {
                    for (unsigned int x = 0; x < columns; x++) {
                        doubleBuffer[x + y * columns].modified = false;
                    }
                }
            }
        }
    }
}

void Display::setCursorPos(CharPos position) {
    if (mode.video_bpp == 0) {
        uint16_t value = position.x + position.y * 80;
        outb(0x3D4, 0x0E);
        outb(0x3D5, (value >> 8) & 0xFF);
        outb(0x3D4, 0x0F);
        outb(0x3D5, value & 0xFF);
    } else {
        if (unlikely(!doubleBuffer)) return;
        CharPos oldPos = cursorPos;
        cursorPos = position;
        doubleBuffer[oldPos.x + oldPos.y * columns].modified = true;
        doubleBuffer[cursorPos.x + cursorPos.y * columns].modified = true;
    }
}

void Display::setCursorVisibility(bool visible) {
    cursorVisible = visible;
    if (mode.video_bpp == 0) {
        if (visible) {
            outb(0x3D4, 0x0A);
            outb(0x3D5, 14);
        } else {
            outb(0x3D4, 0x0A);
            outb(0x3D5, 0x20);
        }
    } else {
        if (unlikely(!doubleBuffer)) return;
        doubleBuffer[cursorPos.x + cursorPos.y * columns].modified = true;
    }
}

int Display::setVideoMode(video_mode* videoMode) {
    AutoLock lock(&mutex);
    return setVideoModeUnlocked(videoMode);
}

int Display::setVideoModeUnlocked(video_mode* videoMode) {
    if (videoMode->video_width == mode.video_width &&
            videoMode->video_height == mode.video_height &&
            videoMode->video_bpp == mode.video_bpp) {
        return 0;
    }
    if (!graphicsDriver->isSupportedMode(*videoMode)) return ENOTSUP;

    console->lock();
    changingResolution = true;

    size_t newRows = videoMode->video_height / charHeight;
    size_t newColumns = (videoMode->video_width + 1) / charWidth;
    size_t newSize = newRows * newColumns;
    size_t oldSize = rows * columns;

    if (newSize > oldSize) {
        bool primary = doubleBuffer == primaryBuffer;
        CharBufferEntry* newPrimaryBuffer = (CharBufferEntry*)
                reallocarray(primaryBuffer, newSize, sizeof(CharBufferEntry));
        if (!newPrimaryBuffer) {
            changingResolution = false;
            console->unlock();
            return ENOMEM;
        }
        primaryBuffer = newPrimaryBuffer;
        CharBufferEntry* newAlternateBuffer = (CharBufferEntry*)
                reallocarray(alternateBuffer, newSize, sizeof(CharBufferEntry));
        if (!newAlternateBuffer) {
            changingResolution = false;
            console->unlock();
            return ENOMEM;
        }
        alternateBuffer = newAlternateBuffer;
        doubleBuffer = primary ? primaryBuffer : alternateBuffer;
    }

    vaddr_t framebuffer = graphicsDriver->setVideoMode(videoMode);
    if (!framebuffer) {
        changingResolution = false;
        console->unlock();
        return EIO;
    }

    if (haveOldBuffer) {
        size_t oldBufferSize = ALIGNUP(mode.video_height * pitch, PAGESIZE);
        kernelSpace->unmapPhysical((vaddr_t) buffer, oldBufferSize);
        haveOldBuffer = false;
    }
    buffer = (char*) framebuffer;
    mode = *videoMode;
    pitch = mode.video_width * (mode.video_bpp / 8);

    size_t oldRows = rows;
    size_t oldColumns = columns;

    newRows = videoMode->video_height / charHeight;
    newColumns = (videoMode->video_width + 1) / charWidth;

    CharBufferEntry blank;
    blank.wc = L'\0';
    blank.color.bgColor = RGB(0, 0, 0);
    blank.color.fgColor = RGB(170, 170, 170);
    blank.color.vgaColor = 0x07;
    blank.modified = true;

    if (cursorPos.y >= newRows) {
        scroll(cursorPos.y - newRows + 1, blank.color);
    }
    rows = newRows;
    columns = newColumns;
    console->updateDisplaySize();

    if (displayOwner) {
        siginfo_t siginfo = {};
        siginfo.si_signo = SIGWINCH;
        siginfo.si_code = SI_KERNEL;
        displayOwner->raiseSignal(siginfo);
    }

    if (newColumns <= oldColumns) {
        for (size_t i = 0; i < newRows; i++) {
            if (i < oldRows) {
                memmove(&doubleBuffer[i * newColumns],
                        &doubleBuffer[i * oldColumns],
                        newColumns * sizeof(CharBufferEntry));
            } else {
                for (size_t j = 0; j < newColumns; j++) {
                    doubleBuffer[i * newColumns + j] = blank;
                }
            }
        }
    } else {
        for (size_t i = newRows; i > 0; i--) {
            if (i < oldRows) {
                memmove(&doubleBuffer[(i - 1) * newColumns],
                        &doubleBuffer[(i - 1) * oldColumns],
                        oldColumns * sizeof(CharBufferEntry));
                for (size_t j = oldColumns; j < newColumns; j++) {
                    doubleBuffer[(i - 1) * newColumns + j] = blank;
                }
            } else {
                for (size_t j = 0; j < newColumns; j++) {
                    doubleBuffer[(i - 1) * newColumns + j] = blank;
                }
            }
        }
    }

    invalidated = true;
    changingResolution = false;
    console->unlock();
    return 0;
}

void Display::switchBuffer(Color color) {
    if (doubleBuffer == primaryBuffer) {
        doubleBuffer = alternateBuffer;
        clear({0, 0}, {columns - 1, rows - 1}, color);
    } else {
        doubleBuffer = primaryBuffer;
    }
    invalidated = true;
}

void Display::update() {
    if (!renderingText || !doubleBuffer || changingResolution) return;
    bool redrawAll = invalidated;
    invalidated = false;

    for (unsigned int y = 0; y < rows; y++) {
        /* Optimisation: if all cells in this row are blank (wc==0) with the
         * same background colour and they are all marked modified, use a
         * single GPU fill for the entire row instead of per-cell redraws. */
        if (graphicsDriver && mode.video_bpp != 0 && !redrawAll) {
            bool allBlank = true;
            uint32_t rowBg = doubleBuffer[0 + y * columns].color.bgColor;
            for (unsigned int x = 0; x < columns && allBlank; x++) {
                CharBufferEntry& e = doubleBuffer[x + y * columns];
                if (!e.modified || e.wc != L'\0' || e.color.bgColor != rowBg)
                    allBlank = false;
            }
            if (allBlank) {
                unsigned py = y * charHeight;
                if (graphicsDriver->gpuFillRect(
                        rowBg | 0xFF000000U,
                        0, py, mode.video_width, charHeight)) {
                    for (unsigned int x = 0; x < columns; x++)
                        doubleBuffer[x + y * columns].modified = false;
                    continue;
                }
            }
        }

        for (unsigned int x = 0; x < columns; x++) {
            if (redrawAll || doubleBuffer[x + y * columns].modified) {
                redraw({x, y});
            }
        }
    }
}

int Display::devctl(int command, void* restrict data, size_t size,
        int* restrict info) {
    AutoLock lock(&mutex);

    switch (command) {
    case DISPLAY_SET_MODE: {
        if (size != 0 && size != sizeof(int)) {
            *info = -1;
            return EINVAL;
        }

        const int* displayMode = (const int*) data;

        if (*displayMode == DISPLAY_MODE_QUERY) {
            *info = renderingText ? DISPLAY_MODE_TEXT : DISPLAY_MODE_LFB;
            return 0;
        } else if (*displayMode == DISPLAY_MODE_TEXT) {
            if (!renderingText) {
                invalidated = true;
            }
            renderingText = true;
            *info = DISPLAY_MODE_TEXT;
            return 0;
        } else if (*displayMode == DISPLAY_MODE_LFB &&
                this->mode.video_bpp != 0) {
            renderingText = false;
            *info = DISPLAY_MODE_LFB;
            return 0;
        } else {
            *info = renderingText ? DISPLAY_MODE_TEXT : DISPLAY_MODE_LFB;
            return ENOTSUP;
        }
    } break;
    case DISPLAY_GET_RESOLUTION: {
        if (size != 0 && size != sizeof(struct display_resolution)) {
            *info = -1;
            return EINVAL;
        }

        if (mode.video_bpp == 0) {
            *info = -1;
            return ENOTSUP;
        }

        struct display_resolution* res = (struct display_resolution*) data;
        res->width = mode.video_width;
        res->height = mode.video_height;
        *info = 0;
        return 0;
    } break;
    case DISPLAY_DRAW: {
        if (size != 0 && size != sizeof(struct display_draw)) {
            *info = -1;
            return EINVAL;
        }

        if (renderingText) {
            *info = -1;
            return ENOTSUP;
        }

        const struct display_draw* draw = (const struct display_draw*) data;
        size_t bytesPerPixel = mode.video_bpp / 8;
        size_t destX = draw->lfb_x + draw->draw_x;
        size_t destY = draw->lfb_y + draw->draw_y;

        if (destX >= mode.video_width || destY >= mode.video_height ||
                draw->draw_width == 0 || draw->draw_height == 0) {
            *info = 0;
            return 0;
        }

        size_t drawWidth = draw->draw_width;
        size_t drawHeight = draw->draw_height;
        if (drawWidth > mode.video_width - destX) {
            drawWidth = mode.video_width - destX;
        }
        if (drawHeight > mode.video_height - destY) {
            drawHeight = mode.video_height - destY;
        }

        for (size_t y = 0; y < drawHeight; y++) {
            const uint32_t* row = (const uint32_t*) ((uintptr_t) draw->lfb +
                    (draw->draw_y + y) * draw->lfb_pitch) + draw->draw_x;
            char* dest = buffer + (destY + y) * pitch + destX * bytesPerPixel;

            if (mode.video_bpp == 32) {
                size_t x = 0;
                while (x < drawWidth) {
                    while (x < drawWidth && (row[x] & 0xFF000000U) == 0) {
                        x++;
                    }
                    size_t runStart = x;
                    while (x < drawWidth && (row[x] & 0xFF000000U) != 0) {
                        x++;
                    }
                    if (runStart < x) {
                        memcpy(dest + runStart * sizeof(uint32_t),
                                row + runStart,
                                (x - runStart) * sizeof(uint32_t));
                    }
                }
            } else if (mode.video_bpp == 24) {
                if (!hasVisiblePixels(row, drawWidth)) continue;

                for (size_t x = 0; x < drawWidth; x++) {
                    uint32_t rgbColor = row[x];
                    if ((rgbColor & 0xFF000000U) == 0) continue;

                    char* addr = dest + x * 3;
                    addr[0] = (rgbColor >> 0) & 0xFF;
                    addr[1] = (rgbColor >> 8) & 0xFF;
                    addr[2] = (rgbColor >> 16) & 0xFF;
                }
            } else {
                for (size_t x = 0; x < drawWidth; x++) {
                    setPixelColor(dest + x * bytesPerPixel, row[x]);
                }
            }
        }

        *info = 0;
        return 0;
    } break;
    case DISPLAY_GET_VIDEO_MODE: {
        if (size != 0 && size != sizeof(struct video_mode)) {
            *info = -1;
            return EINVAL;
        }

        video_mode* videoMode = (video_mode*) data;
        *videoMode = mode;
        *info = 0;
        return 0;
    }
    case DISPLAY_SET_VIDEO_MODE: {
        if (size != 0 && size != sizeof(struct video_mode)) {
            *info = -1;
            return EINVAL;
        }

        if (!graphicsDriver) {
            *info = -1;
            return ENOTSUP;
        }

        video_mode* videoMode = (video_mode*) data;

        int errnum = setVideoModeUnlocked(videoMode);
        *info = errnum ? -1 : 0;
        return errnum;
    }
    case DISPLAY_ACQUIRE: {
        if (data || size) {
            *info = -1;
            return EINVAL;
        }

        if (displayOwner) {
            *info = -1;
            return EBUSY;
        }

        if (mode.video_bpp == 0) {
            *info = -1;
            return ENOTSUP;
        }

        displayOwner = Process::current();
        displayOwner->ownsDisplay = true;
        renderingText = false;

        *info = 0;
        return 0;
    } break;
    case DISPLAY_RELEASE: {
        if (data || size) {
            *info = -1;
            return EINVAL;
        }

        if (displayOwner != Process::current()) {
            *info = -1;
            return EINVAL;
        }

        displayOwner->ownsDisplay = false;
        displayOwner = nullptr;
        renderingText = true;
        invalidated = true;

        *info = 0;
        return 0;
    } break;
    default:
        *info = -1;
        return EINVAL;
    }
}
