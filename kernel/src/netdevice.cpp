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

/* kernel/src/netdevice.cpp
 * Raw Ethernet network device.
 */

#include <errno.h>
#include <fcntl.h>
#include <stdio.h>
#include <string.h>
#include <lunix/poll.h>
#include <lunix/kernel/devices.h>
#include <lunix/kernel/netdevice.h>

static size_t numNetworkDevices = 0;

NetworkDevice::NetworkDevice() : Vnode(S_IFCHR | 0666, DevFS::dev) {
    queuedFrames = 0;
    readIndex = 0;
    readCond = KTHREAD_COND_INITIALIZER;
    writeCond = KTHREAD_COND_INITIALIZER;
}

int NetworkDevice::devctl(int command, void* restrict data, size_t size,
        int* restrict info) {
    AutoLock lock(&mutex);

    switch (command) {
    case NET_GET_INFO: {
        if (size != 0 && size != sizeof(net_info)) {
            *info = -1;
            return EINVAL;
        }
        if (!data) {
            *info = -1;
            return EINVAL;
        }

        net_info* result = (net_info*) data;
        memset(result, 0, sizeof(*result));
        fillInfoLocked(*result);
        *info = 0;
        return 0;
    } break;
    case NET_SET_PROMISCUOUS: {
        if (size != 0 && size != sizeof(int)) {
            *info = -1;
            return EINVAL;
        }
        if (!data) {
            *info = -1;
            return EINVAL;
        }

        const int* enabled = (const int*) data;
        if (!setPromiscuousModeLocked(*enabled != 0)) {
            *info = -1;
            return EIO;
        }

        *info = 0;
        return 0;
    } break;
    default:
        *info = -1;
        return EINVAL;
    }
}

short NetworkDevice::poll() {
    AutoLock lock(&mutex);
    short result = 0;
    if (queuedFrames > 0) result |= POLLIN | POLLRDNORM;
    if (canTransmitLocked()) result |= POLLOUT | POLLWRNORM;
    return result;
}

ssize_t NetworkDevice::read(void* buffer, size_t size, int flags) {
    AutoLock lock(&mutex);

    while (queuedFrames == 0) {
        if (flags & O_NONBLOCK) {
            errno = EAGAIN;
            return -1;
        }

        if (kthread_cond_sigwait(&readCond, &mutex) == EINTR) {
            errno = EINTR;
            return -1;
        }
    }

    const QueuedFrame& frame = receiveQueue[readIndex];
    if (size < frame.size) {
        errno = EMSGSIZE;
        return -1;
    }

    memcpy(buffer, frame.data, frame.size);
    ssize_t result = frame.size;
    readIndex = (readIndex + 1) % (sizeof(receiveQueue) / sizeof(receiveQueue[0]));
    queuedFrames--;
    return result;
}

ssize_t NetworkDevice::write(const void* buffer, size_t size, int flags) {
    if (size == 0) return 0;
    if (size < 14) {
        errno = EINVAL;
        return -1;
    }
    if (size > NET_MAX_FRAME_SIZE) {
        errno = EMSGSIZE;
        return -1;
    }

    uint8_t paddedFrame[NET_MAX_FRAME_SIZE];
    const void* frame = buffer;
    size_t frameSize = size;
    if (size < NET_MIN_FRAME_SIZE) {
        memcpy(paddedFrame, buffer, size);
        memset(paddedFrame + size, 0, NET_MIN_FRAME_SIZE - size);
        frame = paddedFrame;
        frameSize = NET_MIN_FRAME_SIZE;
    }

    AutoLock lock(&mutex);
    while (!canTransmitLocked()) {
        if (flags & O_NONBLOCK) {
            errno = EAGAIN;
            return -1;
        }

        if (kthread_cond_sigwait(&writeCond, &mutex) == EINTR) {
            errno = EINTR;
            return -1;
        }
    }

    if (!transmitFrameLocked(frame, frameSize)) {
        errno = EIO;
        return -1;
    }

    return size;
}

void NetworkDevice::notifyStateChangedLocked() {
    kthread_cond_broadcast(&readCond);
    kthread_cond_broadcast(&writeCond);
}

void NetworkDevice::queueReceivedFrameLocked(const void* frame, size_t size) {
    if (size > NET_MAX_FRAME_SIZE) return;

    if (queuedFrames == sizeof(receiveQueue) / sizeof(receiveQueue[0])) {
        queuedFrames--;
        readIndex = (readIndex + 1) % (sizeof(receiveQueue) /
                sizeof(receiveQueue[0]));
    }

    size_t writeIndex = (readIndex + queuedFrames) %
            (sizeof(receiveQueue) / sizeof(receiveQueue[0]));
    receiveQueue[writeIndex].size = size;
    memcpy(receiveQueue[writeIndex].data, frame, size);
    queuedFrames++;
    kthread_cond_broadcast(&readCond);
}

void NetworkDevice::registerDevice(const Reference<NetworkDevice>& device) {
    char name[16];
    snprintf(name, sizeof(name), "eth%zu", numNetworkDevices++);
    devFS.addDevice(name, device);
}
