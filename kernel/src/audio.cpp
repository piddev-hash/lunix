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

/* kernel/src/audio.cpp
 * Audio device.
 */

#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <lunix/poll.h>
#include <lunix/kernel/audio.h>
#include <lunix/kernel/devices.h>

Reference<AudioDevice> audioDevice;

static inline size_t minSize(size_t a, size_t b) {
    return a < b ? a : b;
}

static bool formatsEqual(const audio_format& a, const audio_format& b) {
    return a.sample_rate == b.sample_rate &&
            a.channels == b.channels &&
            a.encoding == b.encoding &&
            a.bits_per_sample == b.bits_per_sample &&
            a.frame_size == b.frame_size;
}

AudioDevice::AudioDevice() : Vnode(S_IFCHR | 0666, DevFS::dev) {
    readIndex = 0;
    reconfiguring = false;
    writeIndex = 0;
    queuedBytes = 0;
    output = nullptr;
    drainCond = KTHREAD_COND_INITIALIZER;
    writeCond = KTHREAD_COND_INITIALIZER;
    format.sample_rate = 48000;
    format.channels = 2;
    format.encoding = AUDIO_ENCODING_S16_LE;
    format.bits_per_sample = 16;
    format.frame_size = 4;
}

void AudioDevice::attachOutput(AudioOutput* output) {
    bool notify;
    audio_format newFormat = output->getFormat();
    kthread_mutex_lock(&mutex);
    reconfiguring = false;
    this->output = output;
    format = newFormat;
    kthread_cond_broadcast(&writeCond);
    notify = queuedBytes > 0;
    kthread_mutex_unlock(&mutex);
    if (notify) {
        output->onAudioDataAvailable();
    }
}

void AudioDevice::detachOutput(AudioOutput* output) {
    AutoLock lock(&mutex);
    if (this->output != output) return;
    reconfiguring = false;
    this->output = nullptr;
    kthread_cond_broadcast(&writeCond);
}

size_t AudioDevice::dequeuePlaybackData(void* buffer, size_t size,
        bool fillSilence) {
    AutoLock lock(&mutex);

    if (format.frame_size == 0) return 0;
    size -= size % format.frame_size;
    size_t copied = 0;
    uint8_t* dest = (uint8_t*) buffer;

    while (copied < size && queuedBytes > 0) {
        size_t chunk = minSize(size - copied, queuedBytes);
        size_t untilWrap = sizeof(pcmBuffer) - readIndex;
        chunk = minSize(chunk, untilWrap);
        memcpy(dest + copied, pcmBuffer + readIndex, chunk);
        copied += chunk;
        readIndex = (readIndex + chunk) % sizeof(pcmBuffer);
        queuedBytes -= chunk;
    }

    if (fillSilence && copied < size) {
        memset(dest + copied, 0, size - copied);
    }

    if (copied > 0 || queuedBytes < sizeof(pcmBuffer)) {
        kthread_cond_broadcast(&writeCond);
    }
    if (queuedBytes == 0) {
        kthread_cond_broadcast(&drainCond);
    }

    return copied;
}

int AudioDevice::devctl(int command, void* restrict data, size_t size,
        int* restrict info) {
    if (command == AUDIO_SET_FORMAT) {
        if (size != 0 && size != sizeof(audio_format)) {
            *info = -1;
            return EINVAL;
        }

        const audio_format* requested = (const audio_format*) data;
        AudioOutput* currentOutput;
        audio_format currentFormat;

        kthread_mutex_lock(&mutex);
        if (!output) {
            kthread_mutex_unlock(&mutex);
            *info = -1;
            return ENODEV;
        }

        currentFormat = format;
        if (formatsEqual(*requested, currentFormat)) {
            kthread_mutex_unlock(&mutex);
            *info = 0;
            return 0;
        }

        if (queuedBytes > 0 || reconfiguring) {
            kthread_mutex_unlock(&mutex);
            *info = -1;
            return EBUSY;
        }

        reconfiguring = true;
        currentOutput = output;
        kthread_cond_broadcast(&writeCond);
        kthread_mutex_unlock(&mutex);

        bool success = currentOutput->setFormat(*requested);

        kthread_mutex_lock(&mutex);
        bool outputStillPresent = output == currentOutput;
        reconfiguring = false;
        if (success && outputStillPresent) {
            format = currentOutput->getFormat();
            *info = 0;
            kthread_cond_broadcast(&writeCond);
            kthread_mutex_unlock(&mutex);
            return 0;
        }

        if (output) {
            format = output->getFormat();
        } else {
            format = currentFormat;
        }
        kthread_cond_broadcast(&writeCond);
        kthread_mutex_unlock(&mutex);
        *info = -1;
        return outputStillPresent ? ENOTSUP : ENODEV;
    }

    AutoLock lock(&mutex);

    switch (command) {
    case AUDIO_DRAIN:
        if (!output) {
            *info = -1;
            return ENODEV;
        }
        while (queuedBytes > 0) {
            if (kthread_cond_sigwait(&drainCond, &mutex) == EINTR) {
                *info = -1;
                return EINTR;
            }
            if (!output) {
                *info = -1;
                return ENODEV;
            }
        }
        *info = 0;
        return 0;
    case AUDIO_GET_BUFFER_INFO: {
        if (size != 0 && size != sizeof(audio_buffer_info)) {
            *info = -1;
            return EINVAL;
        }
        audio_buffer_info* bufferInfo = (audio_buffer_info*) data;
        bufferInfo->capacity = sizeof(pcmBuffer);
        bufferInfo->queued = queuedBytes;
        bufferInfo->free_bytes = sizeof(pcmBuffer) - queuedBytes;
        *info = 0;
        return 0;
    } break;
    case AUDIO_GET_FORMAT: {
        if (size != 0 && size != sizeof(audio_format)) {
            *info = -1;
            return EINVAL;
        }
        audio_format* deviceFormat = (audio_format*) data;
        *deviceFormat = format;
        *info = 0;
        return 0;
    } break;
    default:
        *info = -1;
        return EINVAL;
    }
}

short AudioDevice::poll() {
    AutoLock lock(&mutex);
    if (output && !reconfiguring && queuedBytes < sizeof(pcmBuffer)) {
        return POLLOUT | POLLWRNORM;
    }
    return 0;
}

ssize_t AudioDevice::write(const void* buffer, size_t size, int flags) {
    if (size == 0) return 0;

    AudioOutput* readyOutput = nullptr;
    const uint8_t* src = (const uint8_t*) buffer;
    size_t written = 0;

    AutoLock lock(&mutex);
    if (!output) {
        errno = ENODEV;
        return -1;
    }

    if (format.frame_size == 0 || size % format.frame_size != 0) {
        errno = EINVAL;
        return -1;
    }

    while (written < size) {
        while (reconfiguring) {
            if (flags & O_NONBLOCK) {
                if (written > 0) return written;
                errno = EAGAIN;
                return -1;
            }

            if (kthread_cond_sigwait(&writeCond, &mutex) == EINTR) {
                if (written > 0) return written;
                errno = EINTR;
                return -1;
            }

            if (!output) {
                if (written > 0) return written;
                errno = ENODEV;
                return -1;
            }
        }

        while (queuedBytes == sizeof(pcmBuffer)) {
            if (flags & O_NONBLOCK) {
                if (written > 0) return written;
                errno = EAGAIN;
                return -1;
            }

            if (kthread_cond_sigwait(&writeCond, &mutex) == EINTR) {
                if (written > 0) return written;
                errno = EINTR;
                return -1;
            }

            if (!output) {
                if (written > 0) return written;
                errno = ENODEV;
                return -1;
            }
        }

        size_t freeBytes = sizeof(pcmBuffer) - queuedBytes;
        size_t chunk = minSize(size - written, freeBytes);
        size_t untilWrap = sizeof(pcmBuffer) - writeIndex;
        chunk = minSize(chunk, untilWrap);
        memcpy(pcmBuffer + writeIndex, src + written, chunk);
        written += chunk;
        writeIndex = (writeIndex + chunk) % sizeof(pcmBuffer);
        queuedBytes += chunk;
        readyOutput = output;
    }

    if (readyOutput) {
        kthread_mutex_unlock(&mutex);
        readyOutput->onAudioDataAvailable();
        kthread_mutex_lock(&mutex);
    }

    return written;
}
