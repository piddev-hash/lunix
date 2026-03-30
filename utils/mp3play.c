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

/* utils/mp3play.c
 * Simple MP3 player for /dev/audio.
 */

#include "utils.h"
#include <devctl.h>
#include <errno.h>
#include <err.h>
#include <fcntl.h>
#include <getopt.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <lunix/audio.h>
#include <mpg123.h>

#define AUDIO_PATH "/dev/audio"
#define DECODE_BUFFER_SIZE (16 * 1024)

static const int expectedEncoding = MPG123_ENC_SIGNED_16;

static void checkMpg123(int status, mpg123_handle* handle, const char* path,
        const char* action) {
    if (status == MPG123_OK) return;

    const char* error;
    if (handle && status == MPG123_ERR) {
        error = mpg123_strerror(handle);
    } else {
        error = mpg123_plain_strerror(status);
    }

    errx(1, "%s: %s failed: %s", path, action, error);
}

static void checkOutputFormat(const char* path, long rate, int channels,
        int encoding) {
    if ((rate != 44100 && rate != 48000) || channels != 2 ||
            encoding != expectedEncoding) {
        errx(1, "%s: decoder produced unsupported format %ld Hz, %d ch, enc 0x%X",
                path, rate, channels, encoding);
    }
}

static int openAudioDevice(void) {
    int fd = open(AUDIO_PATH, O_WRONLY);
    if (fd < 0) err(1, "'%s'", AUDIO_PATH);
    return fd;
}

static void configureAudioDevice(int fd, const char* path, long rate,
        int channels, int encoding) {
    checkOutputFormat(path, rate, channels, encoding);

    struct audio_format format;
    format.sample_rate = (uint32_t) rate;
    format.channels = (uint16_t) channels;
    format.encoding = AUDIO_ENCODING_S16_LE;
    format.bits_per_sample = 16;
    format.frame_size = 4;

    int info;
    errno = posix_devctl(fd, AUDIO_SET_FORMAT, &format, sizeof(format), &info);
    if (errno) {
        errx(1, "%s: AUDIO_SET_FORMAT failed for %ld Hz, %d ch: %s", path,
                rate, channels, strerror(errno));
    }
}

static void drainAudioDevice(int fd, const char* path) {
    int info;
    errno = posix_devctl(fd, AUDIO_DRAIN, NULL, 0, &info);
    if (errno) warn("%s: AUDIO_DRAIN", path);
}

static void writeAll(int fd, const unsigned char* buffer, size_t size) {
    size_t written = 0;
    while (written < size) {
        ssize_t bytes = write(fd, buffer + written, size - written);
        if (bytes < 0) {
            if (errno == EINTR) continue;
            err(1, "write");
        }
        written += (size_t) bytes;
    }
}

static int openInput(const char* path) {
    if (strcmp(path, "-") == 0) {
        int fd = dup(STDIN_FILENO);
        if (fd < 0) err(1, "dup");
        return fd;
    }

    int fd = open(path, O_RDONLY);
    if (fd < 0) err(1, "'%s'", path);
    return fd;
}

static void playFile(int audioFd, const char* path) {
    int mpgError = MPG123_OK;
    mpg123_handle* handle = mpg123_new(NULL, &mpgError);
    if (!handle) {
        errx(1, "%s: mpg123_new failed: %s", path,
                mpg123_plain_strerror(mpgError));
    }

    checkMpg123(mpg123_param(handle, MPG123_ADD_FLAGS,
            MPG123_FORCE_STEREO | MPG123_QUIET, 0.0), handle, path,
            "mpg123_param");
    checkMpg123(mpg123_format_none(handle), handle, path,
            "mpg123_format_none");
    checkMpg123(mpg123_format(handle, 44100, 2, expectedEncoding), handle,
            path, "mpg123_format");
    checkMpg123(mpg123_format(handle, 48000, 2, expectedEncoding), handle,
            path, "mpg123_format");

    int inputFd = openInput(path);
    checkMpg123(mpg123_open_fd(handle, inputFd), handle, path,
            "mpg123_open_fd");

    long rate;
    int channels;
    int encoding;
    checkMpg123(mpg123_getformat(handle, &rate, &channels, &encoding), handle,
            path, "mpg123_getformat");
    configureAudioDevice(audioFd, path, rate, channels, encoding);

    unsigned char buffer[DECODE_BUFFER_SIZE];
    while (true) {
        size_t done = 0;
        int status = mpg123_read(handle, buffer, sizeof(buffer), &done);
        if (done > 0) {
            writeAll(audioFd, buffer, done);
        }

        if (status == MPG123_OK) continue;
        if (status == MPG123_DONE) break;
        if (status == MPG123_NEW_FORMAT) {
            checkMpg123(mpg123_getformat(handle, &rate, &channels, &encoding),
                    handle, path, "mpg123_getformat");
            configureAudioDevice(audioFd, path, rate, channels, encoding);
            continue;
        }

        checkMpg123(status, handle, path, "mpg123_read");
    }

    drainAudioDevice(audioFd, path);

    mpg123_close(handle);
    mpg123_delete(handle);
    close(inputFd);
}

int main(int argc, char* argv[]) {
    struct option longopts[] = {
        { "help", no_argument, 0, 0 },
        { "version", no_argument, 0, 1 },
        { 0, 0, 0, 0 }
    };

    int c;
    while ((c = getopt_long(argc, argv, "", longopts, NULL)) != -1) {
        switch (c) {
        case 0:
            return help(argv[0], "[OPTIONS] [FILE...]\n"
                    "      --help               display this help\n"
                    "      --version            display version info\n"
                    "\n"
                    "Plays MP3 files through /dev/audio. Use '-' for stdin.");
        case 1:
            return version(argv[0]);
        case '?':
            return 1;
        }
    }

    checkMpg123(mpg123_init(), NULL, "mp3play", "mpg123_init");

    int audioFd = openAudioDevice();

    if (optind < argc) {
        for (int i = optind; i < argc; i++) {
            playFile(audioFd, argv[i]);
        }
    } else {
        playFile(audioFd, "-");
    }

    close(audioFd);
    mpg123_exit();
    return 0;
}
