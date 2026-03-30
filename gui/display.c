/* Copyright (c) 2020, 2021 Dennis Wölfing
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

/* gui/display.c
 * Display.
 */

#include <stdlib.h>
#include "window.h"

enum {
    desktopTextBrand,
    desktopTextTerminal,
    desktopTextTheme,
    desktopTextExit,

    numDesktopTexts
};

static dxui_rect damageRect;
static dxui_rect desktopBarRect;
static dxui_color* desktopTextFrameBuffers[numDesktopTexts];
static dxui_rect desktopTextRects[numDesktopTexts];

static dxui_color blend(dxui_color fg, dxui_color bg);
static void freeDesktopText(void);
static int getDesktopBarHeight(void);
static bool getTaskbarButtonRect(const struct Window* target, dxui_rect* rect,
        dxui_rect* textRect);
static int getTaskbarButtonLeft(void);
static int getTaskbarButtonRight(void);
static dxui_color renderTaskbarButtonPixel(struct Window* window, dxui_pos pos,
        dxui_rect rect, dxui_rect textRect);
static dxui_color renderDesktopPixel(dxui_pos pos);
static dxui_color renderPixel(dxui_pos pos);
static void rebuildDesktopChrome(void);

#define min(x, y) ((x) < (y) ? (x) : (y))
#define max(x, y) ((x) > (y) ? (x) : (y))
#define RED_PART(rgba) (((rgba) >> 16) & 0xFF)
#define GREEN_PART(rgba) (((rgba) >> 8) & 0xFF)
#define BLUE_PART(rgba) (((rgba) >> 0) & 0xFF)
#define ALPHA_PART(rgba) (((rgba) >> 24) & 0xFF)

void addDamageRect(dxui_rect rect) {
    // TODO: This is a rather primitive implementation that causes us to redraw
    // too much.
    if (damageRect.width == 0) {
        damageRect = rect;
        return;
    }
    if (rect.width == 0) return;

    dxui_rect newRect;
    newRect.x = min(damageRect.x, rect.x);
    newRect.y = min(damageRect.y, rect.y);
    int xEnd = max(damageRect.x + damageRect.width, rect.x + rect.width);
    int yEnd = max(damageRect.y + damageRect.height, rect.y + rect.height);
    newRect.width = xEnd - newRect.x;
    newRect.height = yEnd - newRect.y;
    damageRect = newRect;
}

void damageDesktopBar(void) {
    if (desktopBarRect.width > 0 && desktopBarRect.height > 0) {
        addDamageRect(desktopBarRect);
    }
}

static dxui_color blend(dxui_color fg, dxui_color bg) {
    if (ALPHA_PART(fg) == 0) return bg;
    if (ALPHA_PART(bg) == 0) return fg;

    dxui_color r = RED_PART(fg);
    dxui_color g = GREEN_PART(fg);
    dxui_color b = BLUE_PART(fg);
    dxui_color a = ALPHA_PART(fg);

    r = r * a * 255 + RED_PART(bg) * ALPHA_PART(bg) * (255 - a);
    g = g * a * 255 + GREEN_PART(bg) * ALPHA_PART(bg) * (255 - a);
    b = b * a * 255 + BLUE_PART(bg) * ALPHA_PART(bg) * (255 - a);
    a = a * 255 + ALPHA_PART(bg) * (255 - a);
    return RGBA(r / 255 / 255, g / 255 / 255, b / 255 / 255, a / 255);
}

void composit(void) {
    dxui_rect rect = dxui_rect_crop(damageRect, guiDim);
    if (rect.width == 0) return;

    for (int y = rect.y; y < rect.y + rect.height; y++) {
        for (int x = rect.x; x < rect.x + rect.width; x++) {
            lfb[y * guiDim.width + x] = renderPixel((dxui_pos) {x, y});
        }
    }

    dxui_update_framebuffer(compositorWindow, rect);
    damageRect.width = 0;
}

static void freeDesktopText(void) {
    for (size_t i = 0; i < numDesktopTexts; i++) {
        free(desktopTextFrameBuffers[i]);
        desktopTextFrameBuffers[i] = NULL;
        desktopTextRects[i] = (dxui_rect) {
            .x = 0,
            .y = 0,
            .width = 0,
            .height = 0,
        };
    }
}

static int getDesktopBarHeight(void) {
    return 28;
}

static int getTaskbarButtonLeft(void) {
    return desktopTextRects[desktopTextBrand].x +
            desktopTextRects[desktopTextBrand].width + 24;
}

static int getTaskbarButtonRight(void) {
    return desktopTextRects[desktopTextTerminal].x - 20;
}

static bool getTaskbarButtonRect(const struct Window* target, dxui_rect* rect,
        dxui_rect* textRect) {
    int left = getTaskbarButtonLeft();
    int right = getTaskbarButtonRight();
    if (right - left < 32) return false;

    size_t count = 0;
    for (struct Window* window = topWindow; window; window = window->below) {
        if (window->showInTaskbar) count++;
    }
    if (count == 0) return false;

    int slotWidth = (right - left) / (int) count;
    if (slotWidth < 36) slotWidth = 36;
    if (slotWidth > 156) slotWidth = 156;

    int x = left;
    for (struct Window* window = topWindow; window; window = window->below) {
        if (!window->showInTaskbar) continue;

        dxui_rect buttonRect = {
            .x = x + 2,
            .y = desktopBarRect.y + 4,
            .width = slotWidth - 4,
            .height = desktopBarRect.height - 8,
        };
        if (buttonRect.x >= right) break;
        if (buttonRect.x + buttonRect.width > right) {
            buttonRect.width = right - buttonRect.x;
        }
        if (buttonRect.width < 24) break;

        if (window == target) {
            if (rect) {
                *rect = buttonRect;
            }

            if (textRect) {
                int textOffset = window == getTopVisibleWindow() ? 1 : 0;
                *textRect = (dxui_rect) {
                    .x = buttonRect.x + 6 + textOffset,
                    .y = buttonRect.y + (buttonRect.height -
                            window->taskTitleDim.height) / 2 + textOffset,
                    .width = buttonRect.width - 12,
                    .height = window->taskTitleDim.height,
                };
            }
            return true;
        }

        x += slotWidth;
    }

    return false;
}

struct Window* getTaskbarWindowAt(dxui_pos pos) {
    for (struct Window* window = topWindow; window; window = window->below) {
        if (!window->showInTaskbar) continue;

        dxui_rect rect;
        if (!getTaskbarButtonRect(window, &rect, NULL)) continue;
        if (dxui_rect_contains_pos(rect, pos)) return window;
    }

    return NULL;
}

void handleResize(dxui_window* window, dxui_resize_event* event) {
    lfb = dxui_get_framebuffer(window, event->dim);
    if (!lfb) dxui_panic(context, "Failed to create window framebuffer");
    guiDim = event->dim;

    dxui_rect rect = { .pos = {0, 0}, .dim = guiDim };
    addDamageRect(rect);

    for (struct Window* win = topWindow; win; win = win->below) {
        if (win->rect.x > guiDim.width - 10) {
            win->rect.x = guiDim.width - 50;
        }

        if (win->rect.y > guiDim.height - 10) {
            win->rect.y = guiDim.height - 50;
        }
    }

    rebuildDesktopChrome();
    broadcastStatusEvent();
}

void initializeDisplay(void) {
    rebuildDesktopChrome();
}

void refreshDesktopTheme(void) {
    rebuildDesktopChrome();
}

static dxui_color renderDesktopPixel(dxui_pos pos) {
    unsigned int flags = guiThemeFlags;
    dxui_color rgba;

    if (dxui_rect_contains_pos(desktopBarRect, pos)) {
        int localX = pos.x - desktopBarRect.x;
        int localY = pos.y - desktopBarRect.y;
        int lastX = desktopBarRect.width - 1;
        int lastY = desktopBarRect.height - 1;

        if (localY == 0) {
            rgba = gui_theme_highlight(flags);
        } else if (localY == 1) {
            rgba = gui_theme_light_shadow(flags);
        } else if (localY == lastY) {
            rgba = gui_theme_dark_shadow(flags);
        } else if (localY == lastY - 1 || localX == 0 || localX == lastX) {
            rgba = gui_theme_shadow(flags);
        } else {
            rgba = gui_theme_face(flags);
        }

        struct Window* taskWindow = getTaskbarWindowAt(pos);
        if (taskWindow) {
            dxui_rect rect;
            dxui_rect textRect;
            if (getTaskbarButtonRect(taskWindow, &rect, &textRect)) {
                rgba = renderTaskbarButtonPixel(taskWindow, pos, rect, textRect);
            }
        }
    } else {
        int patternX = pos.x & 7;
        int patternY = pos.y & 7;
        if (patternX == patternY || patternX == ((patternY + 4) & 7)) {
            rgba = gui_theme_desktop_pattern(flags);
        } else {
            rgba = gui_theme_desktop_background(flags);
        }
    }

    for (size_t i = 0; i < numDesktopTexts; i++) {
        if (!dxui_rect_contains_pos(desktopTextRects[i], pos)) continue;

        dxui_color color = desktopTextFrameBuffers[i][pos.x -
                desktopTextRects[i].x + desktopTextRects[i].width *
                (pos.y - desktopTextRects[i].y)];
        rgba = blend(color, rgba);
    }

    return rgba;
}

static dxui_color renderTaskbarButtonPixel(struct Window* window, dxui_pos pos,
        dxui_rect rect, dxui_rect textRect) {
    unsigned int flags = guiThemeFlags;
    bool active = window == getTopVisibleWindow();
    bool hidden = !window->visible;
    int localX = pos.x - rect.x;
    int localY = pos.y - rect.y;
    int lastX = rect.width - 1;
    int lastY = rect.height - 1;
    dxui_color face = hidden ? gui_theme_window_background(flags) :
            gui_theme_face(flags);
    if (active) {
        face = gui_theme_light_shadow(flags);
    }

    if (!active) {
        if (localX == 0 || localY == 0) {
            return gui_theme_highlight(flags);
        }
        if (localX == 1 || localY == 1) {
            return gui_theme_light_shadow(flags);
        }
        if (localX == lastX || localY == lastY) {
            return gui_theme_dark_shadow(flags);
        }
        if (localX == lastX - 1 || localY == lastY - 1) {
            return gui_theme_shadow(flags);
        }
    } else {
        if (localX == 0 || localY == 0) {
            return gui_theme_shadow(flags);
        }
        if (localX == 1 || localY == 1) {
            return gui_theme_dark_shadow(flags);
        }
        if (localX == lastX || localY == lastY) {
            return gui_theme_highlight(flags);
        }
        if (localX == lastX - 1 || localY == lastY - 1) {
            return gui_theme_light_shadow(flags);
        }
    }

    if (window->taskTitleLfb && dxui_rect_contains_pos(textRect, pos)) {
        int textX = pos.x - textRect.x;
        int textY = pos.y - textRect.y;
        if (textX >= 0 && textY >= 0 && textX < textRect.width &&
                textY < textRect.height &&
                textX < window->taskTitleDim.width &&
                textY < window->taskTitleDim.height) {
            dxui_color textColor = window->taskTitleLfb[textY *
                    window->taskTitleDim.width + textX];
            if (textColor) return textColor;
        }
    }

    return face;
}

static dxui_color renderPixel(dxui_pos pos) {
    dxui_color rgba = 0;

    for (struct Window* window = topWindow; window; window = window->below) {
        if (!window->visible) continue;
        if (!dxui_rect_contains_pos(window->rect, pos)) continue;
        dxui_rect clientRect = getClientRect(window);

        dxui_color color;
        if (dxui_rect_contains_pos(clientRect, pos)) {
            color = renderClientArea(window, pos.x - clientRect.x,
                    pos.y - clientRect.y);
        } else {
            color = renderWindowDecoration(window, pos.x - window->rect.x,
                    pos.y - window->rect.y);
        }
        rgba = blend(rgba, color);
        if (ALPHA_PART(rgba) == 255) return rgba;
    }

    return blend(rgba, renderDesktopPixel(pos));
}

static void rebuildDesktopChrome(void) {
    static const char* labels[numDesktopTexts] = {
        "Lunix " DENNIX_VERSION " Program Manager",
        "GUI+T Terminal",
        "GUI+D Theme",
        "GUI+Q Exit",
    };

    freeDesktopText();

    int barHeight = getDesktopBarHeight();
    desktopBarRect = (dxui_rect) {
        .x = 0,
        .y = guiDim.height - barHeight,
        .width = guiDim.width,
        .height = barHeight,
    };
    dxui_color textColor = gui_theme_text(guiThemeFlags);
    int barCenterY = desktopBarRect.y + (barHeight - 16) / 2;

    dxui_rect rect = {{0, 0, 0, 0}};
    rect = dxui_get_text_rect(labels[desktopTextBrand], rect, 0);
    desktopTextFrameBuffers[desktopTextBrand] = calloc(rect.width,
            rect.height * sizeof(dxui_color));
    if (!desktopTextFrameBuffers[desktopTextBrand]) dxui_panic(context,
            "malloc");
    dxui_draw_text_in_rect(context, desktopTextFrameBuffers[desktopTextBrand],
            labels[desktopTextBrand], textColor, rect.pos, rect, rect.width);
    desktopTextRects[desktopTextBrand] = (dxui_rect) {
        .x = 10,
        .y = barCenterY,
        .width = rect.width,
        .height = rect.height,
    };

    int rightEdge = guiDim.width - 10;
    for (int i = desktopTextExit; i >= desktopTextTerminal; i--) {
        rect = dxui_get_text_rect(labels[i], rect, 0);
        desktopTextFrameBuffers[i] = calloc(rect.width,
                rect.height * sizeof(dxui_color));
        if (!desktopTextFrameBuffers[i]) dxui_panic(context, "malloc");
        dxui_draw_text_in_rect(context, desktopTextFrameBuffers[i], labels[i],
                textColor, rect.pos, rect, rect.width);

        desktopTextRects[i] = (dxui_rect) {
            .x = rightEdge - rect.width,
            .y = barCenterY,
            .width = rect.width,
            .height = rect.height,
        };
        rightEdge = desktopTextRects[i].x - 18;
    }
}
