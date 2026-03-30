// SPDX-License-Identifier: GPL-2.0-or-later

/* kernel/include/lunix/kernel/hda.h
 * Intel HD Audio controller.
 */

#ifndef KERNEL_HDA_H
#define KERNEL_HDA_H

#include <stdint.h>

namespace Hda {
void initialize(uint8_t bus, uint8_t device, uint8_t function);
}

#endif
