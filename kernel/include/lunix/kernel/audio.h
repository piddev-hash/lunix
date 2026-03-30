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

/* kernel/include/lunix/kernel/audio.h
 * Audio device.
 */

#ifndef KERNEL_AUDIO_H
#define KERNEL_AUDIO_H

#include <lunix/audio.h>
#include <lunix/kernel/vnode.h>

class AudioOutput {
public:
    virtual const audio_format& getFormat() const = 0;
    virtual bool setFormat(const audio_format& format) = 0;
    virtual void onAudioDataAvailable() = 0;
    virtual ~AudioOutput() = default;
};

class AudioDevice : public Vnode {
public:
    AudioDevice();
    ~AudioDevice() = default;
    NOT_COPYABLE(AudioDevice);
    NOT_MOVABLE(AudioDevice);

    void attachOutput(AudioOutput* output);
    void detachOutput(AudioOutput* output);
    size_t dequeuePlaybackData(void* buffer, size_t size, bool fillSilence);
    int devctl(int command, void* restrict data, size_t size,
            int* restrict info) override;
    short poll() override;
    ssize_t write(const void* buffer, size_t size, int flags) override;
private:
    uint8_t pcmBuffer[16 * 1024];
    size_t readIndex;
    bool reconfiguring;
    size_t writeIndex;
    size_t queuedBytes;
    audio_format format;
    AudioOutput* output;
    kthread_cond_t drainCond;
    kthread_cond_t writeCond;
};

extern Reference<AudioDevice> audioDevice;

#endif
