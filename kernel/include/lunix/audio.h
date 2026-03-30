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

/* kernel/include/lunix/audio.h
 * Audio device interface.
 */

#ifndef _DENNIX_AUDIO_H
#define _DENNIX_AUDIO_H

#include <stdint.h>
#include <lunix/devctl.h>

#define AUDIO_ENCODING_S16_LE 1

#define AUDIO_DRAIN _DEVCTL(_IOCTL_VOID, 3)
#define AUDIO_GET_FORMAT _DEVCTL(_IOCTL_PTR, 9)
#define AUDIO_SET_FORMAT _DEVCTL(_IOCTL_PTR, 10)
#define AUDIO_GET_BUFFER_INFO _DEVCTL(_IOCTL_PTR, 11)

struct audio_format {
    uint32_t sample_rate;
    uint16_t channels;
    uint16_t encoding;
    uint16_t bits_per_sample;
    uint16_t frame_size;
};

struct audio_buffer_info {
    uint32_t capacity;
    uint32_t queued;
    uint32_t free_bytes;
};

#endif
