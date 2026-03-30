# Lunix

Lunix is a small Unix-like operating system project with its own kernel, C library, shell, GUI stack, userland utilities, and sample applications.

The repository builds a bootable ISO image and runs in QEMU.

## Highlights

- Monolithic kernel for `i686` and `x86_64`
- POSIX-style syscall surface and userspace ABI (`<lunix/...>` headers)
- In-tree C library (`libc`) and math library integration (`libm` sources)
- Built-in shell (`sh`) and core userland tools (`utils`)
- Graphical stack:
  - `gui`: compositor/server process
  - `libdxui`: UI toolkit for client apps
  - GUI apps: `terminal`, `calculator`, `bricks`
- Initrd-based root filesystem image generated from `sysroot`

## Repository Layout

- `kernel/`: kernel sources, architecture startup code, drivers, VFS, networking, scheduler, IPC, and syscall handling.
- `libc/`: C standard library and system interfaces for Lunix.
- `libm/`: math implementation (integrated into `libc` build).
- `libdxui/`: user interface library for windowed applications.
- `gui/`: graphical compositor and GUI socket server.
- `sh/`: shell implementation.
- `utils/`: base command-line programs and system binaries (`/bin`, `/sbin`).
- `apps/`: GUI applications.
- `build-aux/`: architecture/toolchain/path config, GRUB config, helper scripts.
- `third_party/mpg123/`: vendored decoder used by `utils/mp3play`.

## Supported Architectures

Set `ARCH` when invoking `make`:

- `x86_64` (default)
- `i686`

Examples:

```sh
make ARCH=x86_64
make ARCH=i686
```

## Prerequisites

Host tools expected by the build:

- `make`
- `tar`
- `xz`
- `grub-mkrescue` (or compatible tool via `MKRESCUE=...`)
- `qemu-system-x86_64` / `qemu-system-i386` for emulation

Cross toolchain expected in `PATH`:

- `${ARCH}-lunix-gcc`
- `${ARCH}-lunix-g++`
- `${ARCH}-lunix-ar`

By default, the build system does **not** use host `gcc/clang`; it expects the Lunix-targeted cross toolchain.

## Building

Build everything (libc, kernel, GUI stack, shell, apps, utils, ISO):

```sh
make -j$(nproc)
```

Common targets:

- `make libc`
- `make kernel`
- `make libdxui`
- `make gui`
- `make sh`
- `make utils`
- `make apps`
- `make iso`

Output artifacts:

- `lunix.iso`
- `build/<arch>/kernel/kernel`
- `build/<arch>/initrd.tar.xz`

## Running in QEMU

Run the generated ISO:

```sh
make qemu
```

Default QEMU options include:

- 1 GB RAM
- KVM enabled (`-enable-kvm`)
- RTL8139 network device (`-netdev user ... -device rtl8139 ...`)

If you need custom invocation, run `qemu-system-<arch>` manually with `lunix.iso`.

## Sysroot and Initrd

The root filesystem image is assembled from `sysroot/`:

- binaries go to `/bin` and `/sbin`
- libraries and headers installed under `/lib` and `/include`
- license copied to `/share/licenses/lunix/LICENSE`

`make iso` packs `sysroot` into `build/<arch>/initrd.tar.xz`, then creates a GRUB bootable ISO.

## Toolchain Bootstrap

A helper script is provided to build a Lunix-targeted toolchain:

```sh
make install-toolchain
```

Under the hood this runs `build-aux/install-toolchain.sh`, which clones and builds:

- `lunix-binutils`
- `lunix-gcc`

You can control installation paths via environment variables such as `PREFIX`, `SRCDIR`, `BUILDDIR`, `ARCH`, `TARGET`, and `SYSROOT`.

## Userland Programs

### Shell

- `/bin/sh`: Lunix shell with interactive mode, scripting, builtins, parsing, expansion, and job-control-related options.

### Core utilities (`utils/`)

Installed commands include (non-exhaustive categories):

- File tools: `cat`, `cp`, `mv`, `rm`, `ln`, `mkdir`, `rmdir`, `touch`, `ls`
- Text tools: `echo`, `printf`, `head`, `tail`, `sort`, `tr`, `expr`, `test`
- System tools: `uname`, `date`, `time`, `meminfo`, `kill`, `sync`, `chmod`
- Games/tools: `snake`, `editor`, `mp3play`
- System binaries in `/sbin`: `init`, `mount`, `umount`, `dhcp`

### GUI applications (`apps/`)

- `terminal`: GUI terminal emulator backed by a PTY and `/bin/sh`
- `calculator`: integer calculator UI
- `bricks`: brick-breaker style game

## Kernel Notes

From the current source tree, kernel subsystems include:

- Memory management and virtual address spaces
- Process/thread scheduling and signals
- VFS and filesystem support
- Device and bus support (PCI, PS/2, AHCI/ATA, HDA, HPET/PIT/RTC, network NIC drivers)
- Networking and sockets
- Graphics/display support

Filesystem support includes an ext2/ext3/ext4-style driver (`ext234fs`) with documented partial ext4 feature coverage.

## Cleaning

```sh
make clean
make distclean
```

- `clean`: removes build outputs
- `distclean`: removes `build`, `sysroot`, and generated ISOs

## Licensing

- Top-level project license: see [LICENSE](LICENSE).
- Source file headers include permissive notices for upstream/original components.
- `libm/` is based on musl math sources (see `libm/README`).
- `third_party/mpg123/` contains vendored third-party code with its own license files.
