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

/* kernel/include/lunix/net.h
 * Network device control.
 */

#ifndef _DENNIX_NET_H
#define _DENNIX_NET_H

#include <stdint.h>
#include <lunix/devctl.h>

#define NET_SET_PROMISCUOUS _DEVCTL(_IOCTL_INT, 3)
#define NET_GET_INFO _DEVCTL(_IOCTL_PTR, 12)

#define NET_INFO_LINK_UP (1U << 0)
#define NET_INFO_PROMISCUOUS (1U << 1)

#define NET_DEFAULT_MTU 1500U
#define NET_MIN_FRAME_SIZE 60U
#define NET_MAX_FRAME_SIZE 1518U

struct net_info {
    uint8_t mac[6];
    uint8_t reserved[2];
    uint32_t flags;
    uint32_t mtu;
    uint32_t max_frame_size;
};

#endif
