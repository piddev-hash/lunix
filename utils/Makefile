# Copyright (c) 2016, 2017, 2018, 2019, 2020, 2021, 2022, 2023 Dennis Wölfing
#
# Permission to use, copy, modify, and/or distribute this software for any
# purpose with or without fee is hereby granted, provided that the above
# copyright notice and this permission notice appear in all copies.
#
# THE SOFTWARE IS PROVIDED "AS IS" AND THE AUTHOR DISCLAIMS ALL WARRANTIES
# WITH REGARD TO THIS SOFTWARE INCLUDING ALL IMPLIED WARRANTIES OF
# MERCHANTABILITY AND FITNESS. IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR
# ANY SPECIAL, DIRECT, INDIRECT, OR CONSEQUENTIAL DAMAGES OR ANY DAMAGES
# WHATSOEVER RESULTING FROM LOSS OF USE, DATA OR PROFITS, WHETHER IN AN
# ACTION OF CONTRACT, NEGLIGENCE OR OTHER TORTIOUS ACTION, ARISING OUT OF
# OR IN CONNECTION WITH THE USE OR PERFORMANCE OF THIS SOFTWARE.

REPO_ROOT = ..

include $(REPO_ROOT)/build-aux/arch.mk
include $(REPO_ROOT)/build-aux/paths.mk
include $(REPO_ROOT)/build-aux/toolchain.mk
include $(REPO_ROOT)/build-aux/version.mk

BUILD = $(BUILD_DIR)/utils

ARFLAGS = -crs
CFLAGS ?= -O2 -g
CFLAGS += --sysroot=$(SYSROOT) -std=c11 -fstack-protector-strong -Wall -Wextra
CPPFLAGS += -D_DENNIX_SOURCE -DDENNIX_VERSION=\"$(VERSION)\"

MPG123_DIR = $(REPO_ROOT)/third_party/mpg123
MPG123_BUILD = $(BUILD)/third_party/mpg123
MPG123_CPPFLAGS = -I$(MPG123_DIR) -I$(MPG123_DIR)/compat -I$(MPG123_DIR)/libmpg123
MPG123_LIB = $(BUILD)/libmpg123.a
MPG123_OBJ = \
	compat/compat.o \
	compat/compat_str.o \
	libmpg123/dct64.o \
	libmpg123/dither.o \
	libmpg123/equalizer.o \
	libmpg123/feature.o \
	libmpg123/format.o \
	libmpg123/frame.o \
	libmpg123/icy.o \
	libmpg123/icy2utf8.o \
	libmpg123/id3.o \
	libmpg123/index.o \
	libmpg123/layer1.o \
	libmpg123/layer2.o \
	libmpg123/layer3.o \
	libmpg123/libmpg123.o \
	libmpg123/ntom.o \
	libmpg123/optimize.o \
	libmpg123/parse.o \
	libmpg123/readers.o \
	libmpg123/stringbuf.o \
	libmpg123/synth.o \
	libmpg123/synth_8bit.o \
	libmpg123/synth_real.o \
	libmpg123/synth_s32.o \
	libmpg123/tabinit.o

BIN_PROGRAMS = \
	basename \
	cat \
	chmod \
	chvideomode \
	clear \
	cp \
	date \
	dirname \
	echo \
	editor \
	expr \
	false \
	head \
	kill \
	ln \
	ls \
	meminfo \
	mkdir \
	mv \
	ping \
	printf \
	pwd \
	rm \
	rmdir \
	seq \
	sleep \
	snake \
	sort \
	sync \
	tail \
	test \
	time \
	touch \
	tr \
	true \
	uname \
	mp3play

SBIN_PROGRAMS = \
	dhcp \
	init \
	mount \
	umount

PROGRAMS = $(BIN_PROGRAMS) $(SBIN_PROGRAMS)
MPG123_OBJ := $(addprefix $(MPG123_BUILD)/, $(MPG123_OBJ))

.DEFAULT_GOAL := all
-include $(MPG123_OBJ:.o=.d)

all: $(addprefix $(BUILD)/, $(PROGRAMS))

install: install-bin install-sbin

install-bin: $(addprefix $(BUILD)/, $(BIN_PROGRAMS))
	@mkdir -p $(BIN_DIR)
	cp -f $^ $(BIN_DIR)
	ln -f $(BIN_DIR)/test $(BIN_DIR)/[
	touch $(SYSROOT)

install-sbin: $(addprefix $(BUILD)/, $(SBIN_PROGRAMS))
	@mkdir -p $(SBIN_DIR)
	cp -f $^ $(SBIN_DIR)
	touch $(SYSROOT)

$(MPG123_LIB): $(MPG123_OBJ)
	$(AR) $(ARFLAGS) $@ $^

$(BUILD)/mp3play: mp3play.c $(MPG123_LIB)
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) $(CPPFLAGS) $(MPG123_CPPFLAGS) -o $@ $< $(MPG123_LIB)

$(BUILD)/%: %.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) $(CPPFLAGS) -o $@ $<

$(MPG123_BUILD)/%.o: $(MPG123_DIR)/%.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) $(CPPFLAGS) $(MPG123_CPPFLAGS) -MD -MP -c -o $@ $<

$(BUILD)/mv: cp.c rm.c
$(BUILD)/tail: head.c

clean:
	rm -rf $(BUILD)

.PHONY: all install install-bin install-sbin clean
