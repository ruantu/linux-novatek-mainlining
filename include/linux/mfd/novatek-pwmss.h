/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Copyright (C) 2026 Yu-Tung Chang <mtwget@gmail.com>
 */

#ifndef __LINUX_MFD_NOVATEK_PWMSS_H
#define __LINUX_MFD_NOVATEK_PWMSS_H

#include <linux/bits.h>
#include <linux/clk.h>

struct regmap;
struct regmap_irq_chip_data;
struct reset_control;

#define PWM_CCNT_STATUS_REG			0x0f8

/* PWM_CCNT_STATUS bit fields */
#define PWM_CCNT_STATUS_THRESHOLD(ch)		BIT(1 + (ch) * 4)

struct novatek_pwmss {
	struct regmap *regmap;
	struct regmap_irq_chip_data *irq_data;
	int irq;
	struct reset_control *reset;
	struct clk_bulk_data *pwm_clks;
	struct clk **ccnt_clks;
	uint num_pwm;
	uint num_ccnt;
};

#endif /* __LINUX_MFD_NOVATEK_PWMSS_H */
