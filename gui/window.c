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

/* gui/window.c
 * Window.
 */

#include <stdlib.h>
#include <string.h>

#include "connection.h"
#include "window.h"

static const int windowBorderSize = 3;
static const int windowCloseButtonSize = 14;
static const int windowMinimizeButtonSize = 14;
static const int windowTitleAreaHeight = 18;
static const int windowTitleBarSize = windowBorderSize + windowTitleAreaHeight +
        1;

struct Window* changingWindow;
struct Window* mouseWindow;
struct Window* topWindow;

static void addWindowOnTop(struct Window* window);
static dxui_rect chooseWindowRect(int x, int y, int width, int height);
static void damageWindowTitle(struct Window* window);
static dxui_rect getCloseButtonRectLocal(struct Window* window);
static dxui_rect getMinimizeButtonRectLocal(struct Window* window);
static dxui_rect getCloseButtonRect(struct Window* window);
static dxui_rect getMinimizeButtonRect(struct Window* window);
static void refreshTaskButtonTitle(struct Window* window);
static void refreshWindowTitle(struct Window* window);
static void updateActiveWindow(struct Window* oldActive, struct Window* newActive);
static void removeWindow(struct Window* window);
static dxui_color renderCloseButton(int x, int y);
static dxui_color renderMinimizeButton(int x, int y);
static dxui_color renderTitleBarPixel(int x, int y, bool active);

struct Window* getTopVisibleWindow(void) {
    for (struct Window* window = topWindow; window; window = window->below) {
        if (window->visible) return window;
    }

    return NULL;
}

static void updateActiveWindow(struct Window* oldActive, struct Window* newActive) {
    if (oldActive && oldActive != newActive) {
        refreshWindowTitle(oldActive);
        damageWindowTitle(oldActive);
    }

    if (newActive && newActive != oldActive) {
        refreshWindowTitle(newActive);
        damageWindowTitle(newActive);
    }

    bool oldRelative = oldActive && oldActive->relativeMouse;
    bool newRelative = newActive && newActive->relativeMouse;
    if (oldRelative != newRelative) {
        dxui_set_relative_mouse(compositorWindow, newRelative);
    }

    if (oldActive != newActive) {
        damageDesktopBar();
    }
}

static void addWindowOnTop(struct Window* window) {
    struct Window* oldActive = getTopVisibleWindow();

    if (topWindow) {
        topWindow->above = window;
    }
    window->below = topWindow;
    window->above = NULL;
    topWindow = window;

    updateActiveWindow(oldActive, getTopVisibleWindow());

    if (window->visible) {
        addDamageRect(window->rect);
    }
}

struct Window* addWindow(int x, int y, int width, int height, const char* title,
        int flags, struct Connection* connection) {
    struct Window* window = malloc(sizeof(struct Window));
    if (!window) dxui_panic(context, "malloc");

    window->connection = connection;
    window->background = gui_theme_window_background(guiThemeFlags);
    window->cursor = DXUI_CURSOR_ARROW;
    window->flags = flags;
    window->rect = chooseWindowRect(x, y, width, height);
    window->restoreRect = window->rect;
    window->titleText = NULL;
    window->titleLfb = NULL;
    window->taskTitleLfb = NULL;
    window->lfb = NULL;
    window->clientDim = (dxui_dim) {0, 0};
    window->relativeMouse = false;
    window->showInTaskbar = false;
    window->visible = false;

    setWindowTitle(window, title);
    addWindowOnTop(window);
    return window;
}

int checkMouseInteraction(struct Window* window, dxui_pos pos) {
    if (dxui_rect_contains_pos(window->rect, pos)) {
        if (dxui_rect_contains_pos(getClientRect(window), pos)) {
            return CLIENT_AREA;
        } else if (dxui_rect_contains_pos(getMinimizeButtonRect(window), pos)) {
            return MINIMIZE_BUTTON;
        } else if (dxui_rect_contains_pos(getCloseButtonRect(window), pos)) {
            return CLOSE_BUTTON;
        } else {
            int result = 0;
            if (!(window->flags & GUI_WINDOW_NO_RESIZE)) {
                if (pos.x - window->rect.x < windowBorderSize) {
                    result |= RESIZE_LEFT;
                }
                if (pos.x - window->rect.x >= window->rect.width -
                        windowBorderSize) {
                    result |= RESIZE_RIGHT;
                }
                if (pos.y - window->rect.y < windowBorderSize) {
                    result |= RESIZE_TOP;
                }
                if (pos.y - window->rect.y >= window->rect.height -
                        windowBorderSize) {
                    result |= RESIZE_BOTTOM;
                }
            }

            if (!result) {
                return TITLE_BAR;
            }

            return result;
        }
    }

    return 0;
}

static dxui_rect chooseWindowRect(int x, int y, int width, int height) {
    dxui_rect rect;
    rect.x = x;
    rect.y = y;
    rect.width = width + 2 * windowBorderSize;
    rect.height = height + windowTitleBarSize + windowBorderSize;

    if (x < 0) {
        int maxX = guiDim.width - rect.width;
        if (maxX < 0) maxX = 50;
        rect.x = arc4random_uniform(maxX + 1);
    }

    if (y < 0) {
        int maxY = guiDim.height - rect.height;
        if (maxY < 0) maxY = 50;
        rect.y = arc4random_uniform(maxY + 1);
    }

    return rect;
}

static void damageWindowTitle(struct Window* window) {
    if (!window || !window->visible) return;

    dxui_rect titleBar = window->rect;
    titleBar.height = windowTitleBarSize;
    addDamageRect(titleBar);
}

void closeWindow(struct Window* window) {
    struct Window* oldActive = getTopVisibleWindow();

    if (changingWindow == window) {
        changingWindow = NULL;
    }
    if (mouseWindow == window) {
        mouseWindow = NULL;
    }

    removeWindow(window);
    updateActiveWindow(oldActive, getTopVisibleWindow());
    if (window->visible) {
        addDamageRect(window->rect);
    }
    window->connection->windows[window->id] = NULL;
    free(window->titleText);
    free(window->titleLfb);
    free(window->taskTitleLfb);
    free(window->lfb);
    damageDesktopBar();
    free(window);
}

dxui_rect getClientRect(struct Window* window) {
    dxui_rect result;
    result.x = window->rect.x + windowBorderSize;
    result.y = window->rect.y + windowTitleBarSize;
    result.width = window->rect.width - 2 * windowBorderSize;
    result.height = window->rect.height - windowTitleBarSize - windowBorderSize;
    return result;
}

static dxui_rect getCloseButtonRectLocal(struct Window* window) {
    (void) window;
    dxui_rect result;
    result.x = windowBorderSize + 2;
    result.y = windowBorderSize +
            (windowTitleAreaHeight - windowCloseButtonSize) / 2;
    result.width = windowCloseButtonSize;
    result.height = windowCloseButtonSize;
    return result;
}

static dxui_rect getMinimizeButtonRectLocal(struct Window* window) {
    dxui_rect result = getCloseButtonRectLocal(window);
    result.x += result.width + 3;
    result.width = windowMinimizeButtonSize;
    result.height = windowMinimizeButtonSize;
    return result;
}

static dxui_rect getCloseButtonRect(struct Window* window) {
    dxui_rect result = getCloseButtonRectLocal(window);
    result.x += window->rect.x;
    result.y += window->rect.y;
    return result;
}

static dxui_rect getMinimizeButtonRect(struct Window* window) {
    dxui_rect result = getMinimizeButtonRectLocal(window);
    result.x += window->rect.x;
    result.y += window->rect.y;
    return result;
}

void hideWindow(struct Window* window) {
    if (!window->visible) return;
    struct Window* oldActive = getTopVisibleWindow();
    window->visible = false;
    updateActiveWindow(oldActive, getTopVisibleWindow());
    addDamageRect(window->rect);
    damageDesktopBar();
}

void minimizeWindow(struct Window* window) {
    window->restoreRect = window->rect;
    hideWindow(window);
}

void moveWindowToTop(struct Window* window) {
    if (window == topWindow) return;
    removeWindow(window);
    addWindowOnTop(window);
}

void redrawWindow(struct Window* window, int width, int height,
        dxui_color* lfb) {
    if (window->clientDim.width != width ||
            window->clientDim.height != height) {
        free(window->lfb);
        window->lfb = malloc(width * height * sizeof(dxui_color));
        if (!window->lfb) dxui_panic(context, "malloc");
        window->clientDim.width = width;
        window->clientDim.height = height;
    }
    memcpy(window->lfb, lfb, width * height * sizeof(dxui_color));
    if (window->visible) {
        addDamageRect(getClientRect(window));
    }
}

void redrawWindowPart(struct Window* window, int x, int y, int width,
        int height, size_t pitch, dxui_color* lfb) {
    if (x + width > window->clientDim.width ||
            y + height > window->clientDim.height) {
        return;
    }

    for (int yPos = 0; yPos < height; yPos++) {
        for (int xPos = 0; xPos < width; xPos++) {
            size_t index = (y + yPos) * window->clientDim.width + x + xPos;
            window->lfb[index] = lfb[yPos * pitch + xPos];
        }
    }

    if (window->visible) {
        dxui_rect rect;
        rect.x = getClientRect(window).x + x;
        rect.y = getClientRect(window).y + y;
        rect.width = width;
        rect.height = height;
        addDamageRect(rect);
    }
}

static void removeWindow(struct Window* window) {
    if (window->below) {
        window->below->above = window->above;
    }
    if (window->above) {
        window->above->below = window->below;
    } else {
        topWindow = window->below;
    }
}

dxui_color renderClientArea(struct Window* window, int x, int y) {
    if (x >= 0 && x < window->clientDim.width && y >= 0 &&
            y < window->clientDim.height) {
        return window->lfb[y * window->clientDim.width + x];
    } else {
        return window->background;
    }
}

static dxui_color renderCloseButton(int x, int y) {
    unsigned int flags = guiThemeFlags;
    int last = windowCloseButtonSize - 1;

    if (x == 0 || y == 0) {
        return gui_theme_highlight(flags);
    }
    if (x == 1 || y == 1) {
        return gui_theme_light_shadow(flags);
    }
    if (x == last || y == last) {
        return gui_theme_dark_shadow(flags);
    }
    if (x == last - 1 || y == last - 1) {
        return gui_theme_shadow(flags);
    }
    if (((x == y) || (x == last - y)) && x >= 4 && x <= last - 4) {
        return gui_theme_dark_shadow(flags);
    }

    return gui_theme_button_background(flags);
}

static dxui_color renderMinimizeButton(int x, int y) {
    unsigned int flags = guiThemeFlags;
    int last = windowMinimizeButtonSize - 1;

    if (x == 0 || y == 0) {
        return gui_theme_highlight(flags);
    }
    if (x == 1 || y == 1) {
        return gui_theme_light_shadow(flags);
    }
    if (x == last || y == last) {
        return gui_theme_dark_shadow(flags);
    }
    if (x == last - 1 || y == last - 1) {
        return gui_theme_shadow(flags);
    }
    if (y >= last - 4 && y <= last - 3 && x >= 4 && x <= last - 4) {
        return gui_theme_dark_shadow(flags);
    }

    return gui_theme_button_background(flags);
}

static dxui_color renderTitleBarPixel(int x, int y, bool active) {
    (void) y;
    dxui_color base = active ? gui_theme_title_active(guiThemeFlags) :
            gui_theme_title_inactive(guiThemeFlags);
    dxui_color stripe = gui_theme_title_stripe(guiThemeFlags, active);
    return (x & 3) == 1 ? stripe : base;
}

dxui_color renderWindowDecoration(struct Window* window, int x, int y) {
    unsigned int flags = guiThemeFlags;
    bool active = window == getTopVisibleWindow();
    int width = window->rect.width;
    int height = window->rect.height;
    dxui_rect closeRect = getCloseButtonRectLocal(window);
    dxui_rect minimizeRect = getMinimizeButtonRectLocal(window);
    int titleX = minimizeRect.x + minimizeRect.width + 6;
    int titleY = windowBorderSize +
            (windowTitleAreaHeight - window->titleDim.height) / 2;

    if (x == 0 || y == 0 || x == width - 1 || y == height - 1) {
        return gui_theme_dark_shadow(flags);
    }
    if (x == 1 || y == 1) {
        return gui_theme_highlight(flags);
    }
    if (x == width - 2 || y == height - 2) {
        return gui_theme_shadow(flags);
    }
    if (y == windowTitleBarSize - 2 && x >= windowBorderSize &&
            x < width - windowBorderSize) {
        return gui_theme_highlight(flags);
    }
    if (y == windowTitleBarSize - 1 && x >= windowBorderSize &&
            x < width - windowBorderSize) {
        return gui_theme_dark_shadow(flags);
    }
    if (x >= closeRect.x && x < closeRect.x + closeRect.width &&
            y >= closeRect.y && y < closeRect.y + closeRect.height) {
        return renderCloseButton(x - closeRect.x, y - closeRect.y);
    }
    if (x >= minimizeRect.x && x < minimizeRect.x + minimizeRect.width &&
            y >= minimizeRect.y && y < minimizeRect.y + minimizeRect.height) {
        return renderMinimizeButton(x - minimizeRect.x, y - minimizeRect.y);
    }
    if (y >= windowBorderSize && y < windowTitleBarSize - 1 &&
            x >= windowBorderSize && x < width - windowBorderSize) {
        if (window->titleLfb && x >= titleX &&
                x < titleX + window->titleDim.width && y >= titleY &&
                y < titleY + window->titleDim.height) {
            dxui_color color = window->titleLfb[(y - titleY) *
                    window->titleDim.width + x - titleX];
            if (color) return color;
        }
        return renderTitleBarPixel(x - windowBorderSize, y - windowBorderSize,
                active);
    }

    return gui_theme_face(flags);
}

void resizeClientRect(struct Window* window, dxui_dim dim) {
    dxui_rect rect = window->rect;
    rect.width = dim.width + 2 * windowBorderSize;
    rect.height = dim.height + windowTitleBarSize + windowBorderSize;
    resizeWindow(window, rect);
}

void resizeWindow(struct Window* window, dxui_rect rect) {
    if (window->visible) {
        addDamageRect(window->rect);
        addDamageRect(rect);
    }
    window->rect = rect;

    struct gui_event_window_resized msg;
    msg.window_id = window->id;
    msg.width = getClientRect(window).width;
    msg.height = getClientRect(window).height;
    sendEvent(window->connection, GUI_EVENT_WINDOW_RESIZED, sizeof(msg), &msg);
}

void setWindowBackground(struct Window* window, dxui_color color) {
    window->background = color;
    addDamageRect(window->rect);
}

void setWindowCursor(struct Window* window, int cursor) {
    window->cursor = cursor;
}

static void refreshWindowTitle(struct Window* window) {
    free(window->titleLfb);
    window->titleLfb = NULL;

    const char* title = window->titleText ? window->titleText : "";
    dxui_rect rect = {{0, 0, 0, 0}};
    rect = dxui_get_text_rect(title, rect, 0);
    window->titleDim = rect.dim;

    if (rect.width == 0 || rect.height == 0) return;

    window->titleLfb = calloc(rect.width * rect.height, sizeof(dxui_color));
    if (!window->titleLfb) dxui_panic(context, "malloc");

    dxui_draw_text_in_rect(context, window->titleLfb, title,
            gui_theme_title_text(guiThemeFlags, window == getTopVisibleWindow()),
            rect.pos, rect, rect.width);
}

static void refreshTaskButtonTitle(struct Window* window) {
    free(window->taskTitleLfb);
    window->taskTitleLfb = NULL;

    const char* title = window->titleText ? window->titleText : "";
    dxui_rect rect = {{0, 0, 0, 0}};
    rect = dxui_get_text_rect(title, rect, 0);
    window->taskTitleDim = rect.dim;

    if (rect.width == 0 || rect.height == 0) return;

    window->taskTitleLfb = calloc(rect.width * rect.height, sizeof(dxui_color));
    if (!window->taskTitleLfb) dxui_panic(context, "malloc");

    dxui_draw_text_in_rect(context, window->taskTitleLfb, title,
            gui_theme_text(guiThemeFlags), rect.pos, rect, rect.width);
}

void setWindowTitle(struct Window* window, const char* title) {
    char* copy = strdup(title ? title : "");
    if (!copy) dxui_panic(context, "malloc");

    free(window->titleText);
    window->titleText = copy;
    refreshWindowTitle(window);
    refreshTaskButtonTitle(window);
    damageWindowTitle(window);
    if (window->showInTaskbar) {
        damageDesktopBar();
    }
}

void showWindow(struct Window* window) {
    if (window->visible) return;
    struct Window* oldActive = getTopVisibleWindow();
    window->showInTaskbar = true;
    window->visible = true;
    updateActiveWindow(oldActive, getTopVisibleWindow());
    addDamageRect(window->rect);
    damageDesktopBar();
}

void refreshWindowTheme(void) {
    for (struct Window* window = topWindow; window; window = window->below) {
        refreshWindowTitle(window);
        refreshTaskButtonTitle(window);
        if (window->visible) {
            addDamageRect(window->rect);
        }
    }
    damageDesktopBar();
}
