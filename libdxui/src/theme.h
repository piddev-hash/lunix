#ifndef DXUI_THEME_H
#define DXUI_THEME_H

#include <sys/guitheme.h>
#include "context.h"

static inline unsigned int dxui_theme_flags(dxui_context* context) {
    return context ? context->themeFlags : 0;
}

static inline dxui_color dxui_theme_window_background(dxui_context* context) {
    return gui_theme_window_background(dxui_theme_flags(context));
}

static inline dxui_color dxui_theme_surface_background(dxui_context* context) {
    return gui_theme_window_background(dxui_theme_flags(context));
}

static inline dxui_color dxui_theme_button_background(dxui_context* context) {
    return gui_theme_button_background(dxui_theme_flags(context));
}

static inline dxui_color dxui_theme_highlight(dxui_context* context) {
    return gui_theme_highlight(dxui_theme_flags(context));
}

static inline dxui_color dxui_theme_light_shadow(dxui_context* context) {
    return gui_theme_light_shadow(dxui_theme_flags(context));
}

static inline dxui_color dxui_theme_shadow(dxui_context* context) {
    return gui_theme_shadow(dxui_theme_flags(context));
}

static inline dxui_color dxui_theme_dark_shadow(dxui_context* context) {
    return gui_theme_dark_shadow(dxui_theme_flags(context));
}

static inline dxui_color dxui_theme_text_primary(dxui_context* context) {
    return gui_theme_text(dxui_theme_flags(context));
}

#endif
