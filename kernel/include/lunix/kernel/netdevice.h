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

/* kernel/include/lunix/kernel/netdevice.h
 * Raw Ethernet network device.
 */

#ifndef KERNEL_NETDEVICE_H
#define KERNEL_NETDEVICE_H

#include <lunix/net.h>
#include <lunix/kernel/vnode.h>

class NetworkDevice : public Vnode {
public:
    NetworkDevice();
    virtual ~NetworkDevice() = default;
    NOT_COPYABLE(NetworkDevice);
    NOT_MOVABLE(NetworkDevice);

    int devctl(int command, void* restrict data, size_t size,
            int* restrict info) override;
    short poll() override;
    ssize_t read(void* buffer, size_t size, int flags) override;
    ssize_t write(const void* buffer, size_t size, int flags) override;

    static void registerDevice(const Reference<NetworkDevice>& device);
protected:
    struct QueuedFrame {
        size_t size;
        uint8_t data[NET_MAX_FRAME_SIZE];
    };

    void notifyStateChangedLocked();
    void queueReceivedFrameLocked(const void* frame, size_t size);
    virtual bool canTransmitLocked() const = 0;
    virtual void fillInfoLocked(struct net_info& info) const = 0;
    virtual bool setPromiscuousModeLocked(bool enabled) = 0;
    virtual bool transmitFrameLocked(const void* frame, size_t size) = 0;
protected:
    kthread_cond_t readCond;
    kthread_cond_t writeCond;
private:
    QueuedFrame receiveQueue[32];
    size_t queuedFrames;
    size_t readIndex;
};

#endif
