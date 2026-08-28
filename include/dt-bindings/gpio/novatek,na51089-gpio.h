/* SPDX-License-Identifier: (GPL-2.0-only OR BSD-2-Clause) */
/*
 * Copyright (C) 2026 Yu-Tung Chang <mtwget@gmail.com>
 */

#ifndef _DT_BINDINGS_GPIO_NOVATEK_NA51089_GPIO_H
#define _DT_BINDINGS_GPIO_NOVATEK_NA51089_GPIO_H

#include <dt-bindings/gpio/gpio.h>

#define NA51089_CGPIO(pin)		(pin)
#define NA51089_PGPIO(pin)		((pin) + 0x20)
#define NA51089_SGPIO(pin)		((pin) + 0x40)
#define NA51089_LGPIO(pin)		((pin) + 0x60)
#define NA51089_DGPIO(pin)		((pin) + 0x80)
#define NA51089_HGPIO(pin)		((pin) + 0xa0)
#define NA51089_AGPIO(pin)		((pin) + 0xc0)
#define NA51089_DSIGPIO(pin)		((pin) + 0xe0)

#define NA51089_CGPIO_NUM_PINS		23
#define NA51089_PGPIO_NUM_PINS		26
#define NA51089_SGPIO_NUM_PINS		9
#define NA51089_LGPIO_NUM_PINS		10
#define NA51089_DGPIO_NUM_PINS		8
#define NA51089_HGPIO_NUM_PINS		12
#define NA51089_AGPIO_NUM_PINS		3
#define NA51089_DSIGPIO_NUM_PINS	11

#endif
