/* SPDX-License-Identifier: (GPL-2.0-only OR BSD-2-Clause) */
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

#endif
