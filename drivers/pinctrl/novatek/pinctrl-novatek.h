/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __PINCTRL_NOVATEK_H
#define __PINCTRL_NOVATEK_H

#include <linux/io.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/types.h>

struct platform_device;

struct novatek_mux_setting {
	u32 reg;
	u32 mask;
	u32 val;
};

struct novatek_pinctrl_group {
	const char *name;
	const uint *pins;
	uint npins;
	const struct novatek_mux_setting *settings;
	uint nsettings;
	const ulong *gpio_modes;
};

struct novatek_pinctrl_function {
	const char *name;
	const char * const *groups;
	uint ngroups;
};

struct novatek_mux_bank {
	uint pin_base;
	u32 valid_mask;
	u32 reg;
};

struct novatek_pad_field {
	u32 reg;
	u32 mask;
	u8 shift;
	u8 drive_type;
};

struct novatek_pinctrl_soc_data {
	const char *name;
	const char *top_resource;
	const char *pad_resource;

	const struct pinctrl_pin_desc *pins;
	uint npins;
	const struct novatek_pinctrl_group *groups;
	uint ngroups;
	const struct novatek_pinctrl_function *functions;
	uint nfunctions;
	const struct novatek_mux_bank *pinmux_banks;
	uint npinmux_banks;
	u32 mux_pin_mask;
	u32 mux_peripheral;
	u32 mux_gpio;

	u32 pull_none;
	u32 pull_down;
	u32 pull_up;
	u32 pull_keeper;

	int (*get_pull_field)(uint pin,
			      struct novatek_pad_field *field);
	int (*get_drive_field)(uint pin,
			       struct novatek_pad_field *field);
	int (*drive_to_raw)(const struct novatek_pad_field *field,
			    uint drive_ma, u32 *raw);
	uint (*raw_to_drive)(const struct novatek_pad_field *field, u32 raw);
};

int novatek_pinctrl_probe(struct platform_device *pdev);

#endif /* __PINCTRL_NOVATEK_H */
