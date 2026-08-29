// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2026 Yu-Tung Chang <mtwget@gmail.com>
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/clk-provider.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/math64.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include <dt-bindings/clock/novatek,na51089-clock.h>
#include <dt-bindings/reset/novatek,na51089-reset.h>

#include "clk-novatek.h"

#define NA51089_PLL_ENABLE_REG			0x0000
#define NA51089_PLL_STATUS_REG			0x0004
#define NA51089_SYS_CLK_RATE_REG		0x0010
#define NA51089_CODEC_CLK_RATE_REG		0x001c
#define NA51089_PERI_CLK_RATE0_REG		0x0020
#define NA51089_PERI_CLK_RATE1_REG		0x0024
#define NA51089_VIDEO_CLK_DIV_REG		0x0034
#define NA51089_AUDIO_CLK_DIV_REG		0x0038
#define NA51089_SDIO_CLK_DIV_REG		0x003c
#define NA51089_PERI_CLK_DIV1_REG		0x0040
#define NA51089_SPI_CLK_DIV0_REG		0x0044
#define NA51089_SPI_CLK_DIV1_REG		0x0048
#define NA51089_UART_CLK_DIV_REG		0x004c
#define NA51089_PWM_CLK_DIV0_REG		0x0050
#define NA51089_PWM_CLK_DIV1_REG		0x0054
#define NA51089_PWM_CLK_DIV2_REG		0x0058
#define NA51089_PERI_CLK_DIV2_REG		0x0060
#define NA51089_CLK_EN0_REG			0x0070
#define NA51089_CLK_EN1_REG			0x0074
#define NA51089_CLK_EN2_REG			0x0078
#define NA51089_CLK_EN3_REG			0x007c
#define NA51089_RESET0_REG			0x0080
#define NA51089_RESET1_REG			0x0084
#define NA51089_RESET2_REG			0x0088
#define NA51089_SLEEP_REG			0x00a0
#define NA51089_CLK_AUTO_GATE0_REG		0x00b0
#define NA51089_CLK_AUTO_GATE1_REG		0x00b4
#define NA51089_PCLK_AUTO_GATE0_REG		0x00c0
#define NA51089_PCLK_AUTO_GATE1_REG		0x00c4
#define NA51089_PLL8_RATE0_REG			0x4420
#define NA51089_PLL6_RATE0_REG			0x44e0
#define NA51089_PLL9_RATE0_REG			0x4520
#define NA51089_PLL7_RATE0_REG			0x45e0
#define NA51089_CGU_PLL_RATE0_REG		0x0
#define NA51089_CGU_PLL_RATE1_REG		0x4
#define NA51089_CGU_PLL_RATE2_REG		0x8

/* NA51089_PLL_ENABLE bit fields */
#define NA51089_PLL_ENABLE_BIT(bit)		BIT(bit)

/* NA51089_PLL_STATUS bit fields */
#define NA51089_PLL_STATUS_READY(bit)		BIT(bit)

/* NA51089_SYS_CLK_RATE bit fields */
#define NA51089_SYS_CLK_RATE_CPU		GENMASK(1, 0)

/* NA51089_RESET0 bit fields */
#define NA51089_RESET0_BIT(bit)			BIT(bit)

/* NA51089_RESET2 bit fields */
#define NA51089_RESET2_ETH_PHY			GENMASK(31, 30)
#define NA51089_RESET2_ETH_PHY_ASSERT		0x0
#define NA51089_RESET2_ETH_PHY_DEASSERT		0x3

/* NA51089_SLEEP bit fields */
#define NA51089_SLEEP_MODE			GENMASK(1, 0)
#define NA51089_SLEEP_MODE_CLOCK_SCALING	0x0
#define NA51089_SLEEP_ENABLE			BIT(2)
#define NA51089_SLEEP_DONE			BIT(3)
#define NA51089_SLEEP_INT_ENABLE		BIT(4)
#define NA51089_SLEEP_SKIP_CPU_SLEEP		BIT(5)
#define NA51089_SLEEP_SWITCH_CONFIG \
	(FIELD_PREP_CONST(NA51089_SLEEP_MODE, NA51089_SLEEP_MODE_CLOCK_SCALING) | \
	 FIELD_PREP_CONST(NA51089_SLEEP_ENABLE, 1) | \
	 FIELD_PREP_CONST(NA51089_SLEEP_DONE, 0) | \
	 FIELD_PREP_CONST(NA51089_SLEEP_INT_ENABLE, 1) | \
	 FIELD_PREP_CONST(NA51089_SLEEP_SKIP_CPU_SLEEP, 1))
#define NA51089_SLEEP_COMPLETE_CONFIG \
	(FIELD_PREP_CONST(NA51089_SLEEP_MODE, NA51089_SLEEP_MODE_CLOCK_SCALING) | \
	 FIELD_PREP_CONST(NA51089_SLEEP_ENABLE, 0) | \
	 FIELD_PREP_CONST(NA51089_SLEEP_DONE, 1) | \
	 FIELD_PREP_CONST(NA51089_SLEEP_INT_ENABLE, 0) | \
	 FIELD_PREP_CONST(NA51089_SLEEP_SKIP_CPU_SLEEP, 1))

/* NA51089_CLK_AUTO_GATE0 bit fields */
#define NA51089_CLK_AUTO_GATE0_BIT(bit)		BIT(bit)

/* NA51089_CGU_PLL_RATE0 bit fields */
#define NA51089_CGU_PLL_RATE0_RATIO_BYTE	GENMASK(7, 0)

#define NA51089_CGU_PLL_RATIO_DIV		131072
#define NA51089_CGU_PLL_RATIO_MAX		0x00ffffff

#define NA51089_PLL_POLL_DELAY_US		1
#define NA51089_PLL_TIMEOUT_US			3000
#define NA51089_RESET_PULSE_US			10
#define NA51089_CPU_POLL_DELAY_US		1
#define NA51089_CPU_TIMEOUT_US			3000

#define NA51089_CPU_PLL_MAX_RATE		960000000

struct na51089_cpu_clock {
	struct clk_hw hw;
	struct novatek_cgu *cgu;
};

enum na51089_clock_hw {
	NA51089_SOURCE_OSC,
	NA51089_SOURCE_FIX480M,
	NA51089_SOURCE_FIX240M,
	NA51089_SOURCE_FIX192M,
	NA51089_SOURCE_PLLF320,
	NA51089_SOURCE_FIX160M,
	NA51089_SOURCE_FIX120M,
	NA51089_SOURCE_FIX96M,
	NA51089_SOURCE_FIX80M,
	NA51089_SOURCE_FIX60M,
	NA51089_SOURCE_FIX48M,
	NA51089_SOURCE_FIX24M,
	NA51089_SOURCE_FIX16M,
	NA51089_SOURCE_FIX3M,
	NA51089_SOURCE_FIX32768,
	NA51089_SOURCE_FIX32K,
	NA51089_SOURCE_FIX25M,
	NA51089_SOURCE_RESERVED,
	NA51089_SOURCE_PLL6,
	NA51089_SOURCE_ETH_REF,
	NA51089_SOURCE_PLL7,
	NA51089_SOURCE_PLL8,
	NA51089_SOURCE_PLL9,
	NA51089_HW_CPU,
	NA51089_HW_PWM0_3_DIVIDER,
	NA51089_HW_PWM4_7_DIVIDER,
	NA51089_HW_I2S_MCLK,
	NA51089_HW_NR,
};

static struct novatek_cgu *na51089_cgu;

static struct clk_hw *na51089_register_cpu_clock(struct novatek_cgu *cgu,
						 const struct novatek_clk_data *cfg,
						 const struct clk_hw **parents);

static const struct novatek_clk_ops na51089_cpu_clock_ops = {
	.register_hw = na51089_register_cpu_clock,
};

static const struct novatek_clk_data na51089_early_clocks[] = {
	{
		.name = "na51089-osc",
		.type = NOVATEK_CLK_FIXED_FACTOR,
		.id = NOVATEK_CLK_NO_ID,
		.hw_id = NA51089_SOURCE_OSC,
		.num_parents = 1,
		.parent_fw_name = "osc",
		.data.factor.mult = 1,
		.data.factor.div = 1,
	}, {
		.name = "na51089-fix480m",
		.type = NOVATEK_CLK_FIXED_RATE,
		.id = NA51089_CLK_FIXED480M,
		.hw_id = NA51089_SOURCE_FIX480M,
		.data.rate.fixed_rate = 480000000,
	}, {
		.name = "na51089-fix240m",
		.type = NOVATEK_CLK_FIXED_RATE,
		.id = NOVATEK_CLK_NO_ID,
		.hw_id = NA51089_SOURCE_FIX240M,
		.data.rate.fixed_rate = 240000000,
	}, {
		.name = "na51089-fix192m",
		.type = NOVATEK_CLK_FIXED_RATE,
		.id = NOVATEK_CLK_NO_ID,
		.hw_id = NA51089_SOURCE_FIX192M,
		.data.rate.fixed_rate = 192000000,
	}, {
		.name = "na51089-pllf320",
		.type = NOVATEK_CLK_FIXED_RATE,
		.id = NOVATEK_CLK_NO_ID,
		.hw_id = NA51089_SOURCE_PLLF320,
		.data.rate.fixed_rate = 320000000,
	}, {
		.name = "na51089-fix160m",
		.type = NOVATEK_CLK_FIXED_RATE,
		.id = NOVATEK_CLK_NO_ID,
		.hw_id = NA51089_SOURCE_FIX160M,
		.data.rate.fixed_rate = 160000000,
	}, {
		.name = "na51089-fix120m",
		.type = NOVATEK_CLK_FIXED_RATE,
		.id = NOVATEK_CLK_NO_ID,
		.hw_id = NA51089_SOURCE_FIX120M,
		.data.rate.fixed_rate = 120000000,
	}, {
		.name = "na51089-fix96m",
		.type = NOVATEK_CLK_FIXED_RATE,
		.id = NOVATEK_CLK_NO_ID,
		.hw_id = NA51089_SOURCE_FIX96M,
		.data.rate.fixed_rate = 96000000,
	}, {
		.name = "na51089-fix80m",
		.type = NOVATEK_CLK_FIXED_RATE,
		.id = NOVATEK_CLK_NO_ID,
		.hw_id = NA51089_SOURCE_FIX80M,
		.data.rate.fixed_rate = 80000000,
	}, {
		.name = "na51089-fix60m",
		.type = NOVATEK_CLK_FIXED_RATE,
		.id = NOVATEK_CLK_NO_ID,
		.hw_id = NA51089_SOURCE_FIX60M,
		.data.rate.fixed_rate = 60000000,
	}, {
		.name = "na51089-fix48m",
		.type = NOVATEK_CLK_FIXED_RATE,
		.id = NOVATEK_CLK_NO_ID,
		.hw_id = NA51089_SOURCE_FIX48M,
		.data.rate.fixed_rate = 48000000,
	}, {
		.name = "na51089-fix24m",
		.type = NOVATEK_CLK_FIXED_RATE,
		.id = NOVATEK_CLK_NO_ID,
		.hw_id = NA51089_SOURCE_FIX24M,
		.data.rate.fixed_rate = 24000000,
	}, {
		.name = "na51089-fix16m",
		.type = NOVATEK_CLK_FIXED_RATE,
		.id = NOVATEK_CLK_NO_ID,
		.hw_id = NA51089_SOURCE_FIX16M,
		.data.rate.fixed_rate = 16000000,
	}, {
		.name = "na51089-fix3m",
		.type = NOVATEK_CLK_FIXED_RATE,
		.id = NOVATEK_CLK_NO_ID,
		.hw_id = NA51089_SOURCE_FIX3M,
		.data.rate.fixed_rate = 3000000,
	}, {
		.name = "na51089-fix32768",
		.type = NOVATEK_CLK_FIXED_RATE,
		.id = NOVATEK_CLK_NO_ID,
		.hw_id = NA51089_SOURCE_FIX32768,
		.data.rate.fixed_rate = 32768,
	}, {
		.name = "na51089-fix32k",
		.type = NOVATEK_CLK_FIXED_RATE,
		.id = NOVATEK_CLK_NO_ID,
		.hw_id = NA51089_SOURCE_FIX32K,
		.data.rate.fixed_rate = 32000,
	}, {
		.name = "na51089-fix25m",
		.type = NOVATEK_CLK_FIXED_RATE,
		.id = NOVATEK_CLK_NO_ID,
		.hw_id = NA51089_SOURCE_FIX25M,
		.data.rate.fixed_rate = 25000000,
	}, {
		.name = "na51089-reserved",
		.type = NOVATEK_CLK_FIXED_RATE,
		.id = NOVATEK_CLK_NO_ID,
		.hw_id = NA51089_SOURCE_RESERVED,
		.data.rate.fixed_rate = 0,
	}, {
		.name = "na51089-pll6",
		.type = NOVATEK_CLK_PLL,
		.id = NOVATEK_CLK_NO_ID,
		.hw_id = NA51089_SOURCE_PLL6,
		.parents = { NA51089_SOURCE_OSC },
		.num_parents = 1,
		.data.pll.rate0 = NA51089_PLL6_RATE0_REG,
		.data.pll.enable_reg = NA51089_PLL_ENABLE_REG,
		.data.pll.status_reg = NA51089_PLL_STATUS_REG,
		.data.pll.enable_mask = NA51089_PLL_ENABLE_BIT(6),
		.data.pll.enable_value = FIELD_PREP_CONST(NA51089_PLL_ENABLE_BIT(6), 1),
		.data.pll.ready_mask = NA51089_PLL_STATUS_READY(6),
		.data.pll.ready_value = FIELD_PREP_CONST(NA51089_PLL_STATUS_READY(6), 1),
	}, {
		.name = "na51089-eth-ref",
		.type = NOVATEK_CLK_FIXED_FACTOR,
		.id = NOVATEK_CLK_NO_ID,
		.hw_id = NA51089_SOURCE_ETH_REF,
		.parents = { NA51089_SOURCE_PLL6 },
		.num_parents = 1,
		.data.factor.mult = 1,
		.data.factor.div = 6,
	}, {
		.name = "na51089-pll7",
		.type = NOVATEK_CLK_PLL,
		.id = NOVATEK_CLK_NO_ID,
		.hw_id = NA51089_SOURCE_PLL7,
		.parents = { NA51089_SOURCE_OSC },
		.num_parents = 1,
		.data.pll.rate0 = NA51089_PLL7_RATE0_REG,
		.data.pll.enable_reg = NA51089_PLL_ENABLE_REG,
		.data.pll.status_reg = NA51089_PLL_STATUS_REG,
		.data.pll.enable_mask = NA51089_PLL_ENABLE_BIT(7),
		.data.pll.enable_value = FIELD_PREP_CONST(NA51089_PLL_ENABLE_BIT(7), 1),
		.data.pll.ready_mask = NA51089_PLL_STATUS_READY(7),
		.data.pll.ready_value = FIELD_PREP_CONST(NA51089_PLL_STATUS_READY(7), 1),
	}, {
		.name = "na51089-pll8",
		.type = NOVATEK_CLK_PLL,
		.id = NA51089_CLK_PLL8,
		.hw_id = NA51089_SOURCE_PLL8,
		.parents = { NA51089_SOURCE_OSC },
		.num_parents = 1,
		.ops = &na51089_cpu_clock_ops,
	}, {
		.name = "na51089-pll9",
		.type = NOVATEK_CLK_PLL,
		.id = NOVATEK_CLK_NO_ID,
		.hw_id = NA51089_SOURCE_PLL9,
		.parents = { NA51089_SOURCE_OSC },
		.num_parents = 1,
		.data.pll.rate0 = NA51089_PLL9_RATE0_REG,
		.data.pll.enable_reg = NA51089_PLL_ENABLE_REG,
		.data.pll.status_reg = NA51089_PLL_STATUS_REG,
		.data.pll.enable_mask = NA51089_PLL_ENABLE_BIT(9),
		.data.pll.enable_value = FIELD_PREP_CONST(NA51089_PLL_ENABLE_BIT(9), 1),
		.data.pll.ready_mask = NA51089_PLL_STATUS_READY(9),
		.data.pll.ready_value = FIELD_PREP_CONST(NA51089_PLL_STATUS_READY(9), 1),
	}, {
		.name = "na51089-cpu",
		.type = NOVATEK_CLK_MUX,
		.id = NA51089_CLK_CPU,
		.hw_id = NA51089_HW_CPU,
		.flags = CLK_IS_CRITICAL | CLK_SET_RATE_PARENT,
		.parents = { NA51089_SOURCE_FIX80M, NA51089_SOURCE_PLL8,
			     NA51089_SOURCE_FIX480M, NA51089_SOURCE_RESERVED },
		.num_parents = 4,
		.ops = &na51089_cpu_clock_ops,
	}, {
		.name = "na51089-periph",
		.type = NOVATEK_CLK_FIXED_FACTOR,
		.id = NA51089_CLK_PERIPH,
		.hw_id = NOVATEK_CLK_NO_ID,
		.parents = { NA51089_HW_CPU },
		.num_parents = 1,
		.data.factor.mult = 1,
		.data.factor.div = 8,
	}, {
		.name = "na51089-apb",
		.type = NOVATEK_CLK_MUX,
		.id = NA51089_CLK_APB,
		.hw_id = NOVATEK_CLK_NO_ID,
		.parents = { NA51089_SOURCE_FIX48M, NA51089_SOURCE_FIX60M,
			     NA51089_SOURCE_FIX80M, NA51089_SOURCE_FIX120M },
		.num_parents = 4,
		.data.mux.reg = NA51089_SYS_CLK_RATE_REG,
		.data.mux.shift = 8,
		.data.mux.width = 2,
		.data.mux.flags = CLK_MUX_READ_ONLY,
	},
};

static const struct novatek_clk_data na51089_late_clocks[] = {
	{
		.name = "na51089-pwm0-3-divider",
		.type = NOVATEK_CLK_DIVIDER,
		.id = NOVATEK_CLK_NO_ID,
		.hw_id = NA51089_HW_PWM0_3_DIVIDER,
		.parents = { NA51089_SOURCE_FIX120M },
		.num_parents = 1,
		.data.divider.reg = NA51089_PWM_CLK_DIV0_REG,
		.data.divider.shift = 0,
		.data.divider.width = 14,
		.data.divider.flags = 0,
	}, {
		.name = "na51089-pwm4-7-divider",
		.type = NOVATEK_CLK_DIVIDER,
		.id = NOVATEK_CLK_NO_ID,
		.hw_id = NA51089_HW_PWM4_7_DIVIDER,
		.parents = { NA51089_SOURCE_FIX120M },
		.num_parents = 1,
		.data.divider.reg = NA51089_PWM_CLK_DIV0_REG,
		.data.divider.shift = 16,
		.data.divider.width = 14,
		.data.divider.flags = 0,
	}, {
		.name = "na51089-i2s-mclk",
		.type = NOVATEK_CLK_COMPOSITE,
		.id = NOVATEK_CLK_NO_ID,
		.hw_id = NA51089_HW_I2S_MCLK,
		.parents = { NA51089_SOURCE_PLL7 },
		.num_parents = 1,
		.data.composite.div_reg = NA51089_AUDIO_CLK_DIV_REG,
		.data.composite.div_shift = 24,
		.data.composite.div_width = 8,
		.data.composite.gate_reg = NA51089_CLK_EN0_REG,
		.data.composite.gate_bit = 31,
	}, {
		.name = "na51089-timer",
		.type = NOVATEK_CLK_GATE,
		.id = NA51089_CLK_TIMER,
		.hw_id = NOVATEK_CLK_NO_ID,
		.parents = { NA51089_SOURCE_FIX3M },
		.num_parents = 1,
		.data.gate.reg = NA51089_CLK_EN1_REG,
		.data.gate.bit = 18,
	}, {
		.name = "na51089-wdt",
		.type = NOVATEK_CLK_GATE,
		.id = NA51089_CLK_WDT,
		.hw_id = NOVATEK_CLK_NO_ID,
		.parents = { NA51089_SOURCE_OSC },
		.num_parents = 1,
		.data.gate.reg = NA51089_CLK_EN1_REG,
		.data.gate.bit = 17,
	}, {
		.name = "na51089-drtc",
		.type = NOVATEK_CLK_GATE,
		.id = NA51089_CLK_DRTC,
		.hw_id = NOVATEK_CLK_NO_ID,
		.parents = { NA51089_SOURCE_OSC },
		.num_parents = 1,
		.data.gate.reg = NA51089_CLK_EN2_REG,
		.data.gate.bit = 22,
	}, {
		.name = "na51089-i2c0",
		.type = NOVATEK_CLK_GATE,
		.id = NA51089_CLK_I2C0,
		.hw_id = NOVATEK_CLK_NO_ID,
		.parents = { NA51089_SOURCE_FIX48M },
		.num_parents = 1,
		.data.gate.reg = NA51089_CLK_EN1_REG,
		.data.gate.bit = 4,
	}, {
		.name = "na51089-i2c1",
		.type = NOVATEK_CLK_GATE,
		.id = NA51089_CLK_I2C1,
		.hw_id = NOVATEK_CLK_NO_ID,
		.parents = { NA51089_SOURCE_FIX48M },
		.num_parents = 1,
		.data.gate.reg = NA51089_CLK_EN1_REG,
		.data.gate.bit = 5,
	}, {
		.name = "na51089-i2c2",
		.type = NOVATEK_CLK_GATE,
		.id = NA51089_CLK_I2C2,
		.hw_id = NOVATEK_CLK_NO_ID,
		.parents = { NA51089_SOURCE_FIX48M },
		.num_parents = 1,
		.data.gate.reg = NA51089_CLK_EN1_REG,
		.data.gate.bit = 31,
	}, {
		.name = "na51089-uart0",
		.type = NOVATEK_CLK_GATE,
		.id = NA51089_CLK_UART0,
		.hw_id = NOVATEK_CLK_NO_ID,
		.parents = { NA51089_SOURCE_FIX24M },
		.num_parents = 1,
		.data.gate.reg = NA51089_CLK_EN1_REG,
		.data.gate.bit = 10,
	}, {
		.name = "na51089-adc",
		.type = NOVATEK_CLK_GATE,
		.id = NA51089_CLK_ADC,
		.hw_id = NOVATEK_CLK_NO_ID,
		.parents = { NA51089_SOURCE_FIX16M },
		.num_parents = 1,
		.data.gate.reg = NA51089_CLK_EN1_REG,
		.data.gate.bit = 13,
	}, {
		.name = "na51089-usb2",
		.type = NOVATEK_CLK_GATE,
		.id = NA51089_CLK_USB2,
		.hw_id = NOVATEK_CLK_NO_ID,
		.parents = { NA51089_SOURCE_FIX48M },
		.num_parents = 1,
		.data.gate.reg = NA51089_CLK_EN1_REG,
		.data.gate.bit = 19,
	}, {
		.name = "na51089-efuse",
		.type = NOVATEK_CLK_GATE,
		.id = NA51089_CLK_EFUSE,
		.hw_id = NOVATEK_CLK_NO_ID,
		.data.gate.reg = NA51089_CLK_EN1_REG,
		.data.gate.bit = 28,
	}, {
		.name = "na51089-picnt0",
		.type = NOVATEK_CLK_GATE,
		.id = NA51089_CLK_PICNT0,
		.hw_id = NOVATEK_CLK_NO_ID,
		.parents = { NA51089_SOURCE_FIX3M },
		.num_parents = 1,
		.data.gate.reg = NA51089_CLK_EN2_REG,
		.data.gate.bit = 8,
	}, {
		.name = "na51089-picnt1",
		.type = NOVATEK_CLK_GATE,
		.id = NA51089_CLK_PICNT1,
		.hw_id = NOVATEK_CLK_NO_ID,
		.parents = { NA51089_SOURCE_FIX3M },
		.num_parents = 1,
		.data.gate.reg = NA51089_CLK_EN2_REG,
		.data.gate.bit = 9,
	}, {
		.name = "na51089-picnt2",
		.type = NOVATEK_CLK_GATE,
		.id = NA51089_CLK_PICNT2,
		.hw_id = NOVATEK_CLK_NO_ID,
		.parents = { NA51089_SOURCE_FIX3M },
		.num_parents = 1,
		.data.gate.reg = NA51089_CLK_EN2_REG,
		.data.gate.bit = 10,
	}, {
		.name = "na51089-eth",
		.type = NOVATEK_CLK_GATE,
		.id = NA51089_CLK_ETH,
		.hw_id = NOVATEK_CLK_NO_ID,
		.parents = { NA51089_SOURCE_ETH_REF },
		.num_parents = 1,
		.data.gate.reg = NA51089_CLK_EN1_REG,
		.data.gate.bit = 29,
	}, {
		.name = "na51089-eth-ext-phy",
		.type = NOVATEK_CLK_GATE,
		.id = NA51089_CLK_ETH_EXT_PHY,
		.hw_id = NOVATEK_CLK_NO_ID,
		.parents = { NA51089_SOURCE_FIX25M },
		.num_parents = 1,
		.data.gate.reg = NA51089_CLK_EN2_REG,
		.data.gate.bit = 24,
	}, {
		.name = "na51089-pwm0",
		.type = NOVATEK_CLK_GATE,
		.id = NA51089_CLK_PWM0,
		.hw_id = NOVATEK_CLK_NO_ID,
		.flags = CLK_SET_RATE_PARENT,
		.parents = { NA51089_HW_PWM0_3_DIVIDER },
		.num_parents = 1,
		.data.gate.reg = NA51089_CLK_EN3_REG,
		.data.gate.bit = 0,
	}, {
		.name = "na51089-pwm1",
		.type = NOVATEK_CLK_GATE,
		.id = NA51089_CLK_PWM1,
		.hw_id = NOVATEK_CLK_NO_ID,
		.flags = CLK_SET_RATE_PARENT,
		.parents = { NA51089_HW_PWM0_3_DIVIDER },
		.num_parents = 1,
		.data.gate.reg = NA51089_CLK_EN3_REG,
		.data.gate.bit = 1,
	}, {
		.name = "na51089-pwm2",
		.type = NOVATEK_CLK_GATE,
		.id = NA51089_CLK_PWM2,
		.hw_id = NOVATEK_CLK_NO_ID,
		.flags = CLK_SET_RATE_PARENT,
		.parents = { NA51089_HW_PWM0_3_DIVIDER },
		.num_parents = 1,
		.data.gate.reg = NA51089_CLK_EN3_REG,
		.data.gate.bit = 2,
	}, {
		.name = "na51089-pwm3",
		.type = NOVATEK_CLK_GATE,
		.id = NA51089_CLK_PWM3,
		.hw_id = NOVATEK_CLK_NO_ID,
		.flags = CLK_SET_RATE_PARENT,
		.parents = { NA51089_HW_PWM0_3_DIVIDER },
		.num_parents = 1,
		.data.gate.reg = NA51089_CLK_EN3_REG,
		.data.gate.bit = 3,
	}, {
		.name = "na51089-pwm4",
		.type = NOVATEK_CLK_GATE,
		.id = NA51089_CLK_PWM4,
		.hw_id = NOVATEK_CLK_NO_ID,
		.flags = CLK_SET_RATE_PARENT,
		.parents = { NA51089_HW_PWM4_7_DIVIDER },
		.num_parents = 1,
		.data.gate.reg = NA51089_CLK_EN3_REG,
		.data.gate.bit = 4,
	}, {
		.name = "na51089-pwm5",
		.type = NOVATEK_CLK_GATE,
		.id = NA51089_CLK_PWM5,
		.hw_id = NOVATEK_CLK_NO_ID,
		.flags = CLK_SET_RATE_PARENT,
		.parents = { NA51089_HW_PWM4_7_DIVIDER },
		.num_parents = 1,
		.data.gate.reg = NA51089_CLK_EN3_REG,
		.data.gate.bit = 5,
	}, {
		.name = "na51089-pwm6",
		.type = NOVATEK_CLK_GATE,
		.id = NA51089_CLK_PWM6,
		.hw_id = NOVATEK_CLK_NO_ID,
		.flags = CLK_SET_RATE_PARENT,
		.parents = { NA51089_HW_PWM4_7_DIVIDER },
		.num_parents = 1,
		.data.gate.reg = NA51089_CLK_EN3_REG,
		.data.gate.bit = 6,
	}, {
		.name = "na51089-pwm7",
		.type = NOVATEK_CLK_GATE,
		.id = NA51089_CLK_PWM7,
		.hw_id = NOVATEK_CLK_NO_ID,
		.flags = CLK_SET_RATE_PARENT,
		.parents = { NA51089_HW_PWM4_7_DIVIDER },
		.num_parents = 1,
		.data.gate.reg = NA51089_CLK_EN3_REG,
		.data.gate.bit = 7,
	}, {
		.name = "na51089-uart1",
		.type = NOVATEK_CLK_COMPOSITE,
		.id = NA51089_CLK_UART1,
		.hw_id = NOVATEK_CLK_NO_ID,
		.parents = { NA51089_SOURCE_FIX480M },
		.num_parents = 1,
		.data.composite.div_reg = NA51089_UART_CLK_DIV_REG,
		.data.composite.div_width = 8,
		.data.composite.gate_reg = NA51089_CLK_EN1_REG,
		.data.composite.gate_bit = 11,
	}, {
		.name = "na51089-uart2",
		.type = NOVATEK_CLK_COMPOSITE,
		.id = NA51089_CLK_UART2,
		.hw_id = NOVATEK_CLK_NO_ID,
		.parents = { NA51089_SOURCE_FIX480M },
		.num_parents = 1,
		.data.composite.div_reg = NA51089_UART_CLK_DIV_REG,
		.data.composite.div_shift = 8,
		.data.composite.div_width = 8,
		.data.composite.gate_reg = NA51089_CLK_EN1_REG,
		.data.composite.gate_bit = 22,
	}, {
		.name = "na51089-spi0",
		.type = NOVATEK_CLK_COMPOSITE,
		.id = NA51089_CLK_SPI0,
		.hw_id = NOVATEK_CLK_NO_ID,
		.parents = { NA51089_SOURCE_FIX192M },
		.num_parents = 1,
		.data.composite.div_reg = NA51089_SPI_CLK_DIV0_REG,
		.data.composite.div_width = 11,
		.data.composite.gate_reg = NA51089_CLK_EN1_REG,
		.data.composite.gate_bit = 6,
	}, {
		.name = "na51089-spi1",
		.type = NOVATEK_CLK_COMPOSITE,
		.id = NA51089_CLK_SPI1,
		.hw_id = NOVATEK_CLK_NO_ID,
		.parents = { NA51089_SOURCE_FIX192M },
		.num_parents = 1,
		.data.composite.div_reg = NA51089_SPI_CLK_DIV0_REG,
		.data.composite.div_shift = 16,
		.data.composite.div_width = 11,
		.data.composite.gate_reg = NA51089_CLK_EN1_REG,
		.data.composite.gate_bit = 7,
	}, {
		.name = "na51089-spi2",
		.type = NOVATEK_CLK_COMPOSITE,
		.id = NA51089_CLK_SPI2,
		.hw_id = NOVATEK_CLK_NO_ID,
		.parents = { NA51089_SOURCE_FIX192M },
		.num_parents = 1,
		.data.composite.div_reg = NA51089_SPI_CLK_DIV1_REG,
		.data.composite.div_width = 11,
		.data.composite.gate_reg = NA51089_CLK_EN1_REG,
		.data.composite.gate_bit = 8,
	}, {
		.name = "na51089-sdio0",
		.type = NOVATEK_CLK_COMPOSITE,
		.id = NA51089_CLK_SDIO0,
		.hw_id = NOVATEK_CLK_NO_ID,
		.parents = { NA51089_SOURCE_FIX192M, NA51089_SOURCE_FIX480M,
			     NA51089_SOURCE_PLLF320, NA51089_SOURCE_PLL9 },
		.num_parents = 4,
		.data.composite.mux_reg = NA51089_PERI_CLK_RATE0_REG,
		.data.composite.mux_shift = 4,
		.data.composite.mux_width = 2,
		.data.composite.div_reg = NA51089_SDIO_CLK_DIV_REG,
		.data.composite.div_width = 11,
		.data.composite.gate_reg = NA51089_CLK_EN1_REG,
		.data.composite.gate_bit = 2,
	}, {
		.name = "na51089-sdio1",
		.type = NOVATEK_CLK_COMPOSITE,
		.id = NA51089_CLK_SDIO1,
		.hw_id = NOVATEK_CLK_NO_ID,
		.parents = { NA51089_SOURCE_FIX192M, NA51089_SOURCE_FIX480M,
			     NA51089_SOURCE_PLLF320, NA51089_SOURCE_PLL9 },
		.num_parents = 4,
		.data.composite.mux_reg = NA51089_PERI_CLK_RATE0_REG,
		.data.composite.mux_shift = 8,
		.data.composite.mux_width = 2,
		.data.composite.div_reg = NA51089_SDIO_CLK_DIV_REG,
		.data.composite.div_shift = 16,
		.data.composite.div_width = 11,
		.data.composite.gate_reg = NA51089_CLK_EN1_REG,
		.data.composite.gate_bit = 3,
	}, {
		.name = "na51089-sdio2",
		.type = NOVATEK_CLK_COMPOSITE,
		.id = NA51089_CLK_SDIO2,
		.hw_id = NOVATEK_CLK_NO_ID,
		.parents = { NA51089_SOURCE_FIX192M, NA51089_SOURCE_FIX480M,
			     NA51089_SOURCE_PLLF320, NA51089_SOURCE_PLL9 },
		.num_parents = 4,
		.data.composite.mux_reg = NA51089_PERI_CLK_RATE1_REG,
		.data.composite.mux_width = 2,
		.data.composite.div_reg = NA51089_PERI_CLK_DIV1_REG,
		.data.composite.div_width = 11,
		.data.composite.gate_reg = NA51089_CLK_EN1_REG,
		.data.composite.gate_bit = 14,
	}, {
		.name = "na51089-fspi",
		.type = NOVATEK_CLK_COMPOSITE,
		.id = NA51089_CLK_FSPI,
		.hw_id = NOVATEK_CLK_NO_ID,
		.parents = { NA51089_SOURCE_FIX480M },
		.num_parents = 1,
		.data.composite.div_reg = NA51089_PERI_CLK_DIV1_REG,
		.data.composite.div_shift = 12,
		.data.composite.div_width = 6,
		.data.composite.gate_reg = NA51089_CLK_EN1_REG,
	}, {
		.name = "na51089-remote",
		.type = NOVATEK_CLK_COMPOSITE,
		.id = NA51089_CLK_REMOTE,
		.hw_id = NOVATEK_CLK_NO_ID,
		.parents = { NA51089_SOURCE_FIX32768, NA51089_SOURCE_FIX32K },
		.num_parents = 2,
		.data.composite.mux_reg = NA51089_PERI_CLK_RATE1_REG,
		.data.composite.mux_shift = 21,
		.data.composite.mux_width = 1,
		.data.composite.gate_reg = NA51089_CLK_EN1_REG,
		.data.composite.gate_bit = 12,
	}, {
		.name = "na51089-crypto",
		.type = NOVATEK_CLK_COMPOSITE,
		.id = NA51089_CLK_CRYPTO,
		.hw_id = NOVATEK_CLK_NO_ID,
		.parents = { NA51089_SOURCE_FIX240M, NA51089_SOURCE_PLLF320,
			     NA51089_SOURCE_RESERVED, NA51089_SOURCE_PLL9 },
		.num_parents = 4,
		.data.composite.mux_reg = NA51089_CODEC_CLK_RATE_REG,
		.data.composite.mux_shift = 20,
		.data.composite.mux_width = 2,
		.data.composite.gate_reg = NA51089_CLK_EN0_REG,
		.data.composite.gate_bit = 23,
	}, {
		.name = "na51089-trng",
		.type = NOVATEK_CLK_COMPOSITE,
		.id = NA51089_CLK_TRNG,
		.hw_id = NOVATEK_CLK_NO_ID,
		.parents = { NA51089_SOURCE_FIX160M, NA51089_SOURCE_FIX240M },
		.num_parents = 2,
		.data.composite.mux_reg = NA51089_PERI_CLK_RATE1_REG,
		.data.composite.mux_shift = 18,
		.data.composite.mux_width = 1,
		.data.composite.div_reg = NA51089_VIDEO_CLK_DIV_REG,
		.data.composite.div_shift = 24,
		.data.composite.div_width = 8,
		.data.composite.gate_reg = NA51089_CLK_EN2_REG,
		.data.composite.gate_bit = 25,
	}, {
		.name = "na51089-trng-ro",
		.type = NOVATEK_CLK_COMPOSITE,
		.id = NA51089_CLK_TRNG_RO,
		.hw_id = NOVATEK_CLK_NO_ID,
		.parents = { NA51089_SOURCE_OSC, NA51089_SOURCE_RESERVED },
		.num_parents = 2,
		.data.composite.mux_reg = NA51089_PERI_CLK_RATE1_REG,
		.data.composite.mux_shift = 19,
		.data.composite.mux_width = 1,
		.data.composite.mux_flags = CLK_MUX_READ_ONLY,
		.data.composite.div_reg = NA51089_PERI_CLK_DIV2_REG,
		.data.composite.div_width = 8,
		.data.composite.gate_reg = NA51089_CLK_EN2_REG,
		.data.composite.gate_bit = 28,
	}, {
		.name = "na51089-rsa",
		.type = NOVATEK_CLK_COMPOSITE,
		.id = NA51089_CLK_RSA,
		.hw_id = NOVATEK_CLK_NO_ID,
		.parents = { NA51089_SOURCE_FIX240M, NA51089_SOURCE_PLLF320,
			     NA51089_SOURCE_RESERVED, NA51089_SOURCE_PLL9 },
		.num_parents = 4,
		.data.composite.mux_reg = NA51089_CODEC_CLK_RATE_REG,
		.data.composite.mux_shift = 22,
		.data.composite.mux_width = 2,
		.data.composite.gate_reg = NA51089_CLK_EN2_REG,
		.data.composite.gate_bit = 26,
	}, {
		.name = "na51089-hash",
		.type = NOVATEK_CLK_COMPOSITE,
		.id = NA51089_CLK_HASH,
		.hw_id = NOVATEK_CLK_NO_ID,
		.parents = { NA51089_SOURCE_FIX240M, NA51089_SOURCE_PLLF320,
			     NA51089_SOURCE_RESERVED, NA51089_SOURCE_PLL9 },
		.num_parents = 4,
		.data.composite.mux_reg = NA51089_PERI_CLK_RATE1_REG,
		.data.composite.mux_shift = 16,
		.data.composite.mux_width = 2,
		.data.composite.gate_reg = NA51089_CLK_EN2_REG,
		.data.composite.gate_bit = 27,
	}, {
		.name = "na51089-pwm8",
		.type = NOVATEK_CLK_COMPOSITE,
		.id = NA51089_CLK_PWM8,
		.hw_id = NOVATEK_CLK_NO_ID,
		.parents = { NA51089_SOURCE_FIX120M },
		.num_parents = 1,
		.data.composite.div_reg = NA51089_PWM_CLK_DIV1_REG,
		.data.composite.div_width = 14,
		.data.composite.gate_reg = NA51089_CLK_EN3_REG,
		.data.composite.gate_bit = 8,
	}, {
		.name = "na51089-pwm9",
		.type = NOVATEK_CLK_COMPOSITE,
		.id = NA51089_CLK_PWM9,
		.hw_id = NOVATEK_CLK_NO_ID,
		.parents = { NA51089_SOURCE_FIX120M },
		.num_parents = 1,
		.data.composite.div_reg = NA51089_PWM_CLK_DIV1_REG,
		.data.composite.div_shift = 16,
		.data.composite.div_width = 14,
		.data.composite.gate_reg = NA51089_CLK_EN3_REG,
		.data.composite.gate_bit = 9,
	}, {
		.name = "na51089-pwm10",
		.type = NOVATEK_CLK_COMPOSITE,
		.id = NA51089_CLK_PWM10,
		.hw_id = NOVATEK_CLK_NO_ID,
		.parents = { NA51089_SOURCE_FIX120M },
		.num_parents = 1,
		.data.composite.div_reg = NA51089_PWM_CLK_DIV2_REG,
		.data.composite.div_width = 14,
		.data.composite.gate_reg = NA51089_CLK_EN3_REG,
		.data.composite.gate_bit = 10,
	}, {
		.name = "na51089-pwm11",
		.type = NOVATEK_CLK_COMPOSITE,
		.id = NA51089_CLK_PWM11,
		.hw_id = NOVATEK_CLK_NO_ID,
		.parents = { NA51089_SOURCE_FIX120M },
		.num_parents = 1,
		.data.composite.div_reg = NA51089_PWM_CLK_DIV2_REG,
		.data.composite.div_shift = 16,
		.data.composite.div_width = 14,
		.data.composite.gate_reg = NA51089_CLK_EN3_REG,
		.data.composite.gate_bit = 11,
	},
};

static const struct novatek_auto_gate_data na51089_auto_gates[] = {
	{
		.reg = NA51089_PCLK_AUTO_GATE1_REG,
		.mask = NA51089_CLK_AUTO_GATE0_BIT(0),
		.value = FIELD_PREP_CONST(NA51089_CLK_AUTO_GATE0_BIT(0), 0),
	}, {
		.reg = NA51089_PCLK_AUTO_GATE1_REG,
		.mask = NA51089_CLK_AUTO_GATE0_BIT(1),
		.value = FIELD_PREP_CONST(NA51089_CLK_AUTO_GATE0_BIT(1), 0),
	}, {
		.reg = NA51089_PCLK_AUTO_GATE0_REG,
		.mask = NA51089_CLK_AUTO_GATE0_BIT(22),
		.value = FIELD_PREP_CONST(NA51089_CLK_AUTO_GATE0_BIT(22), 0),
	}, {
		.reg = NA51089_PCLK_AUTO_GATE0_REG,
		.mask = NA51089_CLK_AUTO_GATE0_BIT(23),
		.value = FIELD_PREP_CONST(NA51089_CLK_AUTO_GATE0_BIT(23), 0),
	}, {
		.reg = NA51089_PCLK_AUTO_GATE1_REG,
		.mask = NA51089_CLK_AUTO_GATE0_BIT(24),
		.value = FIELD_PREP_CONST(NA51089_CLK_AUTO_GATE0_BIT(24), 0),
	}, {
		.reg = NA51089_PCLK_AUTO_GATE0_REG,
		.mask = NA51089_CLK_AUTO_GATE0_BIT(28),
		.value = FIELD_PREP_CONST(NA51089_CLK_AUTO_GATE0_BIT(28), 0),
	}, {
		.reg = NA51089_PCLK_AUTO_GATE0_REG,
		.mask = NA51089_CLK_AUTO_GATE0_BIT(29),
		.value = FIELD_PREP_CONST(NA51089_CLK_AUTO_GATE0_BIT(29), 0),
	}, {
		.reg = NA51089_PCLK_AUTO_GATE1_REG,
		.mask = NA51089_CLK_AUTO_GATE0_BIT(16),
		.value = FIELD_PREP_CONST(NA51089_CLK_AUTO_GATE0_BIT(16), 0),
	}, {
		.reg = NA51089_CLK_AUTO_GATE0_REG,
		.mask = NA51089_CLK_AUTO_GATE0_BIT(24),
		.value = FIELD_PREP_CONST(NA51089_CLK_AUTO_GATE0_BIT(24), 0),
	}, {
		.reg = NA51089_CLK_AUTO_GATE0_REG,
		.mask = NA51089_CLK_AUTO_GATE0_BIT(25),
		.value = FIELD_PREP_CONST(NA51089_CLK_AUTO_GATE0_BIT(25), 0),
	}, {
		.reg = NA51089_CLK_AUTO_GATE0_REG,
		.mask = NA51089_CLK_AUTO_GATE0_BIT(26),
		.value = FIELD_PREP_CONST(NA51089_CLK_AUTO_GATE0_BIT(26), 0),
	}, {
		.reg = NA51089_CLK_AUTO_GATE0_REG,
		.mask = NA51089_CLK_AUTO_GATE0_BIT(20),
		.value = FIELD_PREP_CONST(NA51089_CLK_AUTO_GATE0_BIT(20), 0),
	}, {
		.reg = NA51089_CLK_AUTO_GATE0_REG,
		.mask = NA51089_CLK_AUTO_GATE0_BIT(21),
		.value = FIELD_PREP_CONST(NA51089_CLK_AUTO_GATE0_BIT(21), 0),
	}, {
		.reg = NA51089_CLK_AUTO_GATE1_REG,
		.mask = NA51089_CLK_AUTO_GATE0_BIT(15),
		.value = FIELD_PREP_CONST(NA51089_CLK_AUTO_GATE0_BIT(15), 0),
	}, {
		.reg = NA51089_CLK_AUTO_GATE0_REG,
		.mask = NA51089_CLK_AUTO_GATE0_BIT(19),
		.value = FIELD_PREP_CONST(NA51089_CLK_AUTO_GATE0_BIT(19), 0),
	}, {
		.reg = NA51089_PCLK_AUTO_GATE0_REG,
		.mask = NA51089_CLK_AUTO_GATE0_BIT(31),
		.value = FIELD_PREP_CONST(NA51089_CLK_AUTO_GATE0_BIT(31), 0),
	}, {
		.reg = NA51089_PCLK_AUTO_GATE0_REG,
		.mask = NA51089_CLK_AUTO_GATE0_BIT(30),
		.value = FIELD_PREP_CONST(NA51089_CLK_AUTO_GATE0_BIT(30), 0),
	}, {
		.reg = NA51089_CLK_AUTO_GATE1_REG,
		.mask = NA51089_CLK_AUTO_GATE0_BIT(27),
		.value = FIELD_PREP_CONST(NA51089_CLK_AUTO_GATE0_BIT(27), 0),
	}, {
		.reg = NA51089_PCLK_AUTO_GATE0_REG,
		.mask = NA51089_CLK_AUTO_GATE0_BIT(17),
		.value = FIELD_PREP_CONST(NA51089_CLK_AUTO_GATE0_BIT(17), 0),
	},
};

static const struct novatek_reset_map na51089_reset_maps[] = {
	[NA51089_RESET_TIMER] = {
		.reg = NA51089_RESET1_REG,
		.mask = NA51089_RESET0_BIT(18),
		.assert_value = FIELD_PREP_CONST(NA51089_RESET0_BIT(18), 0),
		.deassert_value = FIELD_PREP_CONST(NA51089_RESET0_BIT(18), 1),
	},
	[NA51089_RESET_WDT] = {
		.reg = NA51089_RESET1_REG,
		.mask = NA51089_RESET0_BIT(17),
		.assert_value = FIELD_PREP_CONST(NA51089_RESET0_BIT(17), 0),
		.deassert_value = FIELD_PREP_CONST(NA51089_RESET0_BIT(17), 1),
	},
	[NA51089_RESET_RTC] = {
		.reg = NA51089_RESET1_REG,
		.mask = NA51089_RESET0_BIT(16),
		.assert_value = FIELD_PREP_CONST(NA51089_RESET0_BIT(16), 0),
		.deassert_value = FIELD_PREP_CONST(NA51089_RESET0_BIT(16), 1),
	},
	[NA51089_RESET_DRTC] = {
		.reg = NA51089_RESET2_REG,
		.mask = NA51089_RESET0_BIT(22),
		.assert_value = FIELD_PREP_CONST(NA51089_RESET0_BIT(22), 0),
		.deassert_value = FIELD_PREP_CONST(NA51089_RESET0_BIT(22), 1),
	},
	[NA51089_RESET_I2C0] = {
		.reg = NA51089_RESET1_REG,
		.mask = NA51089_RESET0_BIT(4),
		.assert_value = FIELD_PREP_CONST(NA51089_RESET0_BIT(4), 0),
		.deassert_value = FIELD_PREP_CONST(NA51089_RESET0_BIT(4), 1),
	},
	[NA51089_RESET_I2C1] = {
		.reg = NA51089_RESET1_REG,
		.mask = NA51089_RESET0_BIT(5),
		.assert_value = FIELD_PREP_CONST(NA51089_RESET0_BIT(5), 0),
		.deassert_value = FIELD_PREP_CONST(NA51089_RESET0_BIT(5), 1),
	},
	[NA51089_RESET_I2C2] = {
		.reg = NA51089_RESET1_REG,
		.mask = NA51089_RESET0_BIT(31),
		.assert_value = FIELD_PREP_CONST(NA51089_RESET0_BIT(31), 0),
		.deassert_value = FIELD_PREP_CONST(NA51089_RESET0_BIT(31), 1),
	},
	[NA51089_RESET_UART0] = {
		.reg = NA51089_RESET1_REG,
		.mask = NA51089_RESET0_BIT(10),
		.assert_value = FIELD_PREP_CONST(NA51089_RESET0_BIT(10), 0),
		.deassert_value = FIELD_PREP_CONST(NA51089_RESET0_BIT(10), 1),
	},
	[NA51089_RESET_UART1] = {
		.reg = NA51089_RESET1_REG,
		.mask = NA51089_RESET0_BIT(11),
		.assert_value = FIELD_PREP_CONST(NA51089_RESET0_BIT(11), 0),
		.deassert_value = FIELD_PREP_CONST(NA51089_RESET0_BIT(11), 1),
	},
	[NA51089_RESET_UART2] = {
		.reg = NA51089_RESET1_REG,
		.mask = NA51089_RESET0_BIT(22),
		.assert_value = FIELD_PREP_CONST(NA51089_RESET0_BIT(22), 0),
		.deassert_value = FIELD_PREP_CONST(NA51089_RESET0_BIT(22), 1),
	},
	[NA51089_RESET_SPI0] = {
		.reg = NA51089_RESET1_REG,
		.mask = NA51089_RESET0_BIT(6),
		.assert_value = FIELD_PREP_CONST(NA51089_RESET0_BIT(6), 0),
		.deassert_value = FIELD_PREP_CONST(NA51089_RESET0_BIT(6), 1),
	},
	[NA51089_RESET_SPI1] = {
		.reg = NA51089_RESET1_REG,
		.mask = NA51089_RESET0_BIT(7),
		.assert_value = FIELD_PREP_CONST(NA51089_RESET0_BIT(7), 0),
		.deassert_value = FIELD_PREP_CONST(NA51089_RESET0_BIT(7), 1),
	},
	[NA51089_RESET_SPI2] = {
		.reg = NA51089_RESET1_REG,
		.mask = NA51089_RESET0_BIT(8),
		.assert_value = FIELD_PREP_CONST(NA51089_RESET0_BIT(8), 0),
		.deassert_value = FIELD_PREP_CONST(NA51089_RESET0_BIT(8), 1),
	},
	[NA51089_RESET_SDIO0] = {
		.reg = NA51089_RESET1_REG,
		.mask = NA51089_RESET0_BIT(2),
		.assert_value = FIELD_PREP_CONST(NA51089_RESET0_BIT(2), 0),
		.deassert_value = FIELD_PREP_CONST(NA51089_RESET0_BIT(2), 1),
	},
	[NA51089_RESET_SDIO1] = {
		.reg = NA51089_RESET1_REG,
		.mask = NA51089_RESET0_BIT(3),
		.assert_value = FIELD_PREP_CONST(NA51089_RESET0_BIT(3), 0),
		.deassert_value = FIELD_PREP_CONST(NA51089_RESET0_BIT(3), 1),
	},
	[NA51089_RESET_SDIO2] = {
		.reg = NA51089_RESET1_REG,
		.mask = NA51089_RESET0_BIT(14),
		.assert_value = FIELD_PREP_CONST(NA51089_RESET0_BIT(14), 0),
		.deassert_value = FIELD_PREP_CONST(NA51089_RESET0_BIT(14), 1),
	},
	[NA51089_RESET_FSPI] = {
		.reg = NA51089_RESET1_REG,
		.mask = NA51089_RESET0_BIT(0),
		.assert_value = FIELD_PREP_CONST(NA51089_RESET0_BIT(0), 0),
		.deassert_value = FIELD_PREP_CONST(NA51089_RESET0_BIT(0), 1),
	},
	[NA51089_RESET_ADC] = {
		.reg = NA51089_RESET1_REG,
		.mask = NA51089_RESET0_BIT(13),
		.assert_value = FIELD_PREP_CONST(NA51089_RESET0_BIT(13), 0),
		.deassert_value = FIELD_PREP_CONST(NA51089_RESET0_BIT(13), 1),
	},
	[NA51089_RESET_REMOTE] = {
		.reg = NA51089_RESET1_REG,
		.mask = NA51089_RESET0_BIT(12),
		.assert_value = FIELD_PREP_CONST(NA51089_RESET0_BIT(12), 0),
		.deassert_value = FIELD_PREP_CONST(NA51089_RESET0_BIT(12), 1),
	},
	[NA51089_RESET_USB2] = {
		.reg = NA51089_RESET1_REG,
		.mask = NA51089_RESET0_BIT(19),
		.assert_value = FIELD_PREP_CONST(NA51089_RESET0_BIT(19), 0),
		.deassert_value = FIELD_PREP_CONST(NA51089_RESET0_BIT(19), 1),
	},
	[NA51089_RESET_EFUSE] = {
		.reg = NA51089_RESET1_REG,
		.mask = NA51089_RESET0_BIT(28),
		.assert_value = FIELD_PREP_CONST(NA51089_RESET0_BIT(28), 0),
		.deassert_value = FIELD_PREP_CONST(NA51089_RESET0_BIT(28), 1),
	},
	[NA51089_RESET_PWM] = {
		.reg = NA51089_RESET2_REG,
		.mask = NA51089_RESET0_BIT(8),
		.assert_value = FIELD_PREP_CONST(NA51089_RESET0_BIT(8), 0),
		.deassert_value = FIELD_PREP_CONST(NA51089_RESET0_BIT(8), 1),
	},
	[NA51089_RESET_ETH] = {
		.reg = NA51089_RESET1_REG,
		.mask = NA51089_RESET0_BIT(29),
		.assert_value = FIELD_PREP_CONST(NA51089_RESET0_BIT(29), 0),
		.deassert_value = FIELD_PREP_CONST(NA51089_RESET0_BIT(29), 1),
	},
	[NA51089_RESET_ETH_PHY] = {
		.reg = NA51089_RESET2_REG,
		.mask = NA51089_RESET2_ETH_PHY,
		.assert_value = FIELD_PREP_CONST(NA51089_RESET2_ETH_PHY,
						 NA51089_RESET2_ETH_PHY_ASSERT),
		.deassert_value = FIELD_PREP_CONST(NA51089_RESET2_ETH_PHY,
						   NA51089_RESET2_ETH_PHY_DEASSERT),
	},
	[NA51089_RESET_CRYPTO] = {
		.reg = NA51089_RESET0_REG,
		.mask = NA51089_RESET0_BIT(23),
		.assert_value = FIELD_PREP_CONST(NA51089_RESET0_BIT(23), 0),
		.deassert_value = FIELD_PREP_CONST(NA51089_RESET0_BIT(23), 1),
	},
	[NA51089_RESET_TRNG] = {
		.reg = NA51089_RESET2_REG,
		.mask = NA51089_RESET0_BIT(25),
		.assert_value = FIELD_PREP_CONST(NA51089_RESET0_BIT(25), 0),
		.deassert_value = FIELD_PREP_CONST(NA51089_RESET0_BIT(25), 1),
	},
	[NA51089_RESET_RSA] = {
		.reg = NA51089_RESET2_REG,
		.mask = NA51089_RESET0_BIT(26),
		.assert_value = FIELD_PREP_CONST(NA51089_RESET0_BIT(26), 0),
		.deassert_value = FIELD_PREP_CONST(NA51089_RESET0_BIT(26), 1),
	},
	[NA51089_RESET_HASH] = {
		.reg = NA51089_RESET2_REG,
		.mask = NA51089_RESET0_BIT(27),
		.assert_value = FIELD_PREP_CONST(NA51089_RESET0_BIT(27), 0),
		.deassert_value = FIELD_PREP_CONST(NA51089_RESET0_BIT(27), 1),
	},
	[NA51089_RESET_DAI] = {
		.reg = NA51089_RESET0_REG,
		.mask = NA51089_RESET0_BIT(29),
		.assert_value = FIELD_PREP_CONST(NA51089_RESET0_BIT(29), 0),
		.deassert_value = FIELD_PREP_CONST(NA51089_RESET0_BIT(29), 1),
	},
};

static ulong na51089_pll_recalc_rate(void __iomem *rate0, ulong parent_rate)
{
	u32 ratio;

	ratio = FIELD_GET(NA51089_CGU_PLL_RATE0_RATIO_BYTE,
			  readl(rate0 + NA51089_CGU_PLL_RATE0_REG));
	ratio |= FIELD_GET(NA51089_CGU_PLL_RATE0_RATIO_BYTE,
			   readl(rate0 + NA51089_CGU_PLL_RATE1_REG)) << 8;
	ratio |= FIELD_GET(NA51089_CGU_PLL_RATE0_RATIO_BYTE,
			   readl(rate0 + NA51089_CGU_PLL_RATE2_REG)) << 16;
	return div_u64((u64)parent_rate * ratio, NA51089_CGU_PLL_RATIO_DIV);
}

static int na51089_cpu_pll_enable(struct clk_hw *hw)
{
	struct na51089_cpu_clock *clock = container_of(hw, struct na51089_cpu_clock, hw);
	struct novatek_cgu *cgu = clock->cgu;
	ulong flags;
	u32 value;

	spin_lock_irqsave(&cgu->lock, flags);
	value = readl(cgu->early_base + NA51089_PLL_ENABLE_REG);
	value |= NA51089_PLL_ENABLE_BIT(8);
	writel(value, cgu->early_base + NA51089_PLL_ENABLE_REG);
	spin_unlock_irqrestore(&cgu->lock, flags);

	return readl_poll_timeout_atomic(cgu->early_base + NA51089_PLL_STATUS_REG,
					 value, value & NA51089_PLL_STATUS_READY(8),
					 NA51089_PLL_POLL_DELAY_US, NA51089_PLL_TIMEOUT_US);
}

static int na51089_cpu_pll_is_enabled(struct clk_hw *hw)
{
	struct na51089_cpu_clock *clock = container_of(hw, struct na51089_cpu_clock, hw);

	return !!(readl(clock->cgu->early_base + NA51089_PLL_ENABLE_REG) & NA51089_PLL_ENABLE_BIT(8));
}

static unsigned long na51089_cpu_pll_recalc_rate(struct clk_hw *hw, unsigned long parent_rate)
{
	struct na51089_cpu_clock *clock = container_of(hw, struct na51089_cpu_clock, hw);

	return na51089_pll_recalc_rate(clock->cgu->early_base + NA51089_PLL8_RATE0_REG,
				       parent_rate);
}

static int na51089_cpu_pll_determine_rate(struct clk_hw *hw, struct clk_rate_request *req)
{
	ulong parent_rate = req->best_parent_rate;
	u64 ratio;

	if (!parent_rate || req->rate > NA51089_CPU_PLL_MAX_RATE)
		return -EINVAL;

	ratio = DIV_ROUND_CLOSEST_ULL((u64)req->rate * NA51089_CGU_PLL_RATIO_DIV, parent_rate);
	if (!ratio || ratio > NA51089_CGU_PLL_RATIO_MAX)
		return -EINVAL;
	req->rate = div_u64(ratio * parent_rate, NA51089_CGU_PLL_RATIO_DIV);
	return 0;
}

static int na51089_cpu_pll_set_rate(struct clk_hw *hw, unsigned long rate,
				    unsigned long parent_rate)
{
	struct na51089_cpu_clock *clock = container_of(hw, struct na51089_cpu_clock, hw);
	struct novatek_cgu *cgu = clock->cgu;
	void __iomem *rate0 = cgu->early_base + NA51089_PLL8_RATE0_REG;
	ulong flags;
	u64 ratio;

	if (!parent_rate || rate > NA51089_CPU_PLL_MAX_RATE)
		return -EINVAL;

	ratio = DIV_ROUND_CLOSEST_ULL((u64)rate * NA51089_CGU_PLL_RATIO_DIV, parent_rate);
	if (!ratio || ratio > NA51089_CGU_PLL_RATIO_MAX)
		return -EINVAL;

	spin_lock_irqsave(&cgu->lock, flags);
	writel(FIELD_GET(NA51089_CGU_PLL_RATE0_RATIO_BYTE, ratio),
	       rate0 + NA51089_CGU_PLL_RATE0_REG);
	writel(FIELD_GET(NA51089_CGU_PLL_RATE0_RATIO_BYTE, ratio >> 8),
	       rate0 + NA51089_CGU_PLL_RATE1_REG);
	writel(FIELD_GET(NA51089_CGU_PLL_RATE0_RATIO_BYTE, ratio >> 16),
	       rate0 + NA51089_CGU_PLL_RATE2_REG);
	spin_unlock_irqrestore(&cgu->lock, flags);

	return 0;
}

static const struct clk_ops na51089_cpu_pll_ops = {
	.enable = na51089_cpu_pll_enable,
	.is_enabled = na51089_cpu_pll_is_enabled,
	.recalc_rate = na51089_cpu_pll_recalc_rate,
	.determine_rate = na51089_cpu_pll_determine_rate,
	.set_rate = na51089_cpu_pll_set_rate,
};

static u8 na51089_cpu_get_parent(struct clk_hw *hw)
{
	struct na51089_cpu_clock *clock = container_of(hw, struct na51089_cpu_clock, hw);
	u32 value;

	value = readl(clock->cgu->early_base + NA51089_SYS_CLK_RATE_REG);
	return FIELD_GET(NA51089_SYS_CLK_RATE_CPU, value);
}

static int na51089_cpu_set_parent(struct clk_hw *hw, u8 index)
{
	struct na51089_cpu_clock *clock = container_of(hw, struct na51089_cpu_clock, hw);
	struct novatek_cgu *cgu = clock->cgu;
	ulong flags;
	u32 value;
	int ret;

	spin_lock_irqsave(&cgu->lock, flags);
	value = readl(cgu->early_base + NA51089_SYS_CLK_RATE_REG);
	value &= ~NA51089_SYS_CLK_RATE_CPU;
	value |= FIELD_PREP(NA51089_SYS_CLK_RATE_CPU, index);
	writel(value, cgu->early_base + NA51089_SYS_CLK_RATE_REG);

	writel(NA51089_SLEEP_SWITCH_CONFIG, cgu->early_base + NA51089_SLEEP_REG);
	ret = readl_poll_timeout_atomic(cgu->early_base + NA51089_SLEEP_REG,
					value, value & NA51089_SLEEP_DONE,
					NA51089_CPU_POLL_DELAY_US, NA51089_CPU_TIMEOUT_US);
	if (ret)
		goto out_unlock;

	writel(NA51089_SLEEP_COMPLETE_CONFIG, cgu->early_base + NA51089_SLEEP_REG);

out_unlock:
	spin_unlock_irqrestore(&cgu->lock, flags);
	return ret;
}

static const struct clk_ops na51089_cpu_mux_ops = {
	.get_parent = na51089_cpu_get_parent,
	.set_parent = na51089_cpu_set_parent,
	.determine_rate = __clk_mux_determine_rate,
};

static struct clk_hw *na51089_register_cpu_clock(struct novatek_cgu *cgu,
						 const struct novatek_clk_data *cfg,
						 const struct clk_hw **parents)
{
	struct na51089_cpu_clock *clock;
	struct clk_init_data init = {
		.name = cfg->name,
		.parent_hws = parents,
		.num_parents = cfg->num_parents,
		.flags = cfg->flags,
	};
	int ret;

	clock = kzalloc_obj(*clock);
	if (!clock)
		return ERR_PTR(-ENOMEM);

	init.ops = cfg->type == NOVATEK_CLK_PLL ? &na51089_cpu_pll_ops : &na51089_cpu_mux_ops;
	clock->cgu = cgu;
	clock->hw.init = &init;

	ret = clk_hw_register(NULL, &clock->hw);
	if (ret) {
		kfree(clock);
		return ERR_PTR(ret);
	}
	return &clock->hw;
}

static const struct novatek_cgu_match_data na51089_cgu_data = {
	.early_clocks = na51089_early_clocks,
	.num_early_clocks = ARRAY_SIZE(na51089_early_clocks),
	.late_clocks = na51089_late_clocks,
	.num_late_clocks = ARRAY_SIZE(na51089_late_clocks),
	.num_clks = NA51089_CLK_NR_CLKS,
	.num_hws = NA51089_HW_NR,
	.resets = na51089_reset_maps,
	.num_resets = NA51089_RESET_NR_RESETS,
	.reset_pulse_us = NA51089_RESET_PULSE_US,
	.auto_gates = na51089_auto_gates,
	.num_auto_gates = ARRAY_SIZE(na51089_auto_gates),
	.pll_poll_delay_us = NA51089_PLL_POLL_DELAY_US,
	.pll_timeout_us = NA51089_PLL_TIMEOUT_US,
	.pll_recalc_rate = na51089_pll_recalc_rate,
};

static void __init na51089_cgu_early_init(struct device_node *np)
{
	na51089_cgu = novatek_cgu_early_init(np, &na51089_cgu_data);
}

CLK_OF_DECLARE_DRIVER(na51089_cgu, "novatek,na51089-cgu", na51089_cgu_early_init);

static int na51089_cgu_probe(struct platform_device *pdev)
{
	return novatek_cgu_probe(pdev, na51089_cgu);
}

static const struct of_device_id na51089_cgu_of_match[] = {
	{ .compatible = "novatek,na51089-cgu", .data = &na51089_cgu_data },
	{ }
};

static struct platform_driver na51089_cgu_driver = {
	.probe = na51089_cgu_probe,
	.driver = {
		.name = "na51089-cgu",
		.of_match_table = na51089_cgu_of_match,
		.suppress_bind_attrs = true,
	},
};
builtin_platform_driver(na51089_cgu_driver);
