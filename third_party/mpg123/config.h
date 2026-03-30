/*
 * Local mpg123 configuration for Lunix/x86_64.
 *
 * This is derived from upstream mpg123 platform configs and trimmed down for
 * the decoder-only static build that powers utils/mp3play.
 */

#ifndef LUNIX_MPG123_CONFIG_H
#define LUNIX_MPG123_CONFIG_H

#define DEFAULT_OUTPUT_MODULE "dummy"

#define ASMALIGN_BALIGN 1
#define CCALIGN 1

#define FRAME_INDEX 1
#define GAPLESS 1

#define HAVE_ATOLL 1
#define HAVE_DIRENT_H 1
#define HAVE_INTTYPES_H 1
#define HAVE_LANGINFO_H 1
#define HAVE_LIBM 1
#define HAVE_LIMITS_H 1
#define HAVE_LOCALE_H 1
#define HAVE_SIGNAL_H 1
#define HAVE_STDINT_H 1
#define HAVE_STDIO_H 1
#define HAVE_STDLIB_H 1
#define HAVE_STRDUP 1
#define HAVE_STRERROR 1
#define HAVE_STRINGS_H 1
#define HAVE_STRING_H 1
#define HAVE_SYS_IOCTL_H 1
#define HAVE_SYS_RESOURCE_H 1
#define HAVE_SYS_SOCKET_H 1
#define HAVE_SYS_STAT_H 1
#define HAVE_SYS_TIME_H 1
#define HAVE_SYS_TYPES_H 1
#define HAVE_SYS_WAIT_H 1
#define HAVE_TERMIOS 1
#define HAVE_UNISTD_H 1

#define IEEE_FLOAT 1

#define INDEX_SIZE 1000
#define LFS_ALIAS_BITS 64

#define PACKAGE "mpg123"
#define PACKAGE_BUGREPORT "mpg123-devel@lists.sourceforge.net"
#define PACKAGE_NAME "mpg123"
#define PACKAGE_STRING "mpg123 1.25.6"
#define PACKAGE_TARNAME "mpg123"
#define PACKAGE_URL ""
#define PACKAGE_VERSION "1.25.6"

#define SIZEOF_INT32_T 4
#define SIZEOF_LONG 8
#define SIZEOF_OFF_T 8
#define SIZEOF_SIZE_T 8
#define SIZEOF_SSIZE_T 8

#define STDC_HEADERS 1
#define VERSION "1.25.6"

#define lfs_alias_t off_t

#define OPT_GENERIC
#define REAL_IS_FLOAT

#endif
