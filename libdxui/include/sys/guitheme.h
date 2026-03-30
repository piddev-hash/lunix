#ifndef _SYS_GUITHEME_H
#define _SYS_GUITHEME_H

#include <stdbool.h>
#include <stdint.h>
#include <sys/colors.h>
#include <sys/guimsg.h>

static inline bool gui_theme_is_dark(unsigned int flags) {
    return flags & GUI_STATUS_DARK_THEME;
}

static inline uint32_t gui_theme_desktop_background(unsigned int flags) {
    return gui_theme_is_dark(flags) ? RGB(17, 41, 47) : RGB(0, 128, 128);
}

static inline uint32_t gui_theme_desktop_pattern(unsigned int flags) {
    return gui_theme_is_dark(flags) ? RGB(10, 25, 29) : RGB(0, 96, 96);
}

static inline uint32_t gui_theme_face(unsigned int flags) {
    return gui_theme_is_dark(flags) ? RGB(62, 66, 74) : RGB(192, 192, 192);
}

static inline uint32_t gui_theme_window_background(unsigned int flags) {
    return gui_theme_is_dark(flags) ? RGB(56, 60, 68) : RGB(192, 192, 192);
}

static inline uint32_t gui_theme_button_background(unsigned int flags) {
    return gui_theme_face(flags);
}

static inline uint32_t gui_theme_text(unsigned int flags) {
    return gui_theme_is_dark(flags) ? RGB(242, 242, 242) : COLOR_BLACK;
}

static inline uint32_t gui_theme_muted_text(unsigned int flags) {
    return gui_theme_is_dark(flags) ? RGB(194, 198, 205) : RGB(64, 64, 64);
}

static inline uint32_t gui_theme_highlight(unsigned int flags) {
    return gui_theme_is_dark(flags) ? RGB(150, 156, 166) : COLOR_WHITE;
}

static inline uint32_t gui_theme_light_shadow(unsigned int flags) {
    return gui_theme_is_dark(flags) ? RGB(96, 102, 112) : RGB(223, 223, 223);
}

static inline uint32_t gui_theme_shadow(unsigned int flags) {
    return gui_theme_is_dark(flags) ? RGB(26, 29, 35) : RGB(128, 128, 128);
}

static inline uint32_t gui_theme_dark_shadow(unsigned int flags) {
    return gui_theme_is_dark(flags) ? RGB(8, 10, 13) : COLOR_BLACK;
}

static inline uint32_t gui_theme_title_active(unsigned int flags) {
    return gui_theme_is_dark(flags) ? RGB(17, 82, 123) : RGB(0, 0, 128);
}

static inline uint32_t gui_theme_title_inactive(unsigned int flags) {
    return gui_theme_is_dark(flags) ? RGB(72, 77, 88) : RGB(128, 128, 128);
}

static inline uint32_t gui_theme_title_stripe(unsigned int flags, bool active) {
    if (active) {
        return gui_theme_is_dark(flags) ? RGB(38, 108, 152) : RGB(16, 40, 160);
    }

    return gui_theme_is_dark(flags) ? RGB(94, 99, 111) : RGB(160, 160, 160);
}

static inline uint32_t gui_theme_title_text(unsigned int flags, bool active) {
    (void) active;
    return gui_theme_is_dark(flags) ? RGB(244, 244, 244) : COLOR_WHITE;
}

#endif
