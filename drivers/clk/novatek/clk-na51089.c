// SPDX-License-Identifier: GPL-2.0-only

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/clk-provider.h>
#include <linux/io.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#include <dt-bindings/clock/novatek,na51089-clock.h>
#include <dt-bindings/reset/novatek,na51089-reset.h>

#include "clk-novatek.h"

#define NA51089_PLL_ENABLE			0x0000
#define NA51089_CGU_PLL_ENABLE_MASK(bit)	BIT(bit)
#define NA51089_CGU_PLL_DISABLE			0x0
#define NA51089_CGU_PLL_ENABLE			0x1

#define NA51089_PLL_STATUS			0x0004
#define NA51089_CGU_PLL_STATUS_MASK(bit)	BIT(bit)
#define NA51089_CGU_PLL_NOT_READY		0x0
#define NA51089_CGU_PLL_READY			0x1

#define NA51089_SYS_CLK_RATE			0x0010

#define NA51089_CODEC_CLK_RATE			0x001c

#define NA51089_PERI_CLK_RATE0			0x0020
#define NA51089_PERI_CLK_RATE1			0x0024

#define NA51089_VIDEO_CLK_DIV			0x0034

#define NA51089_AUDIO_CLK_DIV			0x0038

#define NA51089_SDIO_CLK_DIV			0x003c

#define NA51089_PERI_CLK_DIV1			0x0040
#define NA51089_PERI_CLK_DIV2			0x0060

#define NA51089_SPI_CLK_DIV0			0x0044
#define NA51089_SPI_CLK_DIV1			0x0048

#define NA51089_UART_CLK_DIV			0x004c

#define NA51089_PWM_CLK_DIV0			0x0050
#define NA51089_PWM_CLK_DIV1			0x0054
#define NA51089_PWM_CLK_DIV2			0x0058

#define NA51089_CLK_EN0				0x0070
#define NA51089_CLK_EN1				0x0074
#define NA51089_CLK_EN2				0x0078
#define NA51089_CLK_EN3				0x007c

#define NA51089_RESET0				0x0080
#define NA51089_RESET1				0x0084
#define NA51089_RESET2				0x0088
#define NA51089_CGU_RESET_MASK(bit)		BIT(bit)
#define NA51089_CGU_RESET_ASSERT		0x0
#define NA51089_CGU_RESET_DEASSERT		0x1

#define NA51089_CLK_AUTO_GATE0			0x00b0
#define NA51089_CLK_AUTO_GATE1			0x00b4

#define NA51089_PCLK_AUTO_GATE0			0x00c0
#define NA51089_PCLK_AUTO_GATE1			0x00c4
#define NA51089_CGU_AUTO_GATE_MASK(bit)		BIT(bit)
#define NA51089_CGU_AUTO_GATE_DISABLE		0x0
#define NA51089_CGU_AUTO_GATE_ENABLE		0x1

#define NA51089_PLL8_RATE0			0x4420
#define NA51089_PLL9_RATE0			0x4520
#define NA51089_PLL7_RATE0			0x45e0
#define NA51089_CGU_PLL_RATE0			0x0
#define NA51089_CGU_PLL_RATE1			0x4
#define NA51089_CGU_PLL_RATE2			0x8
#define NA51089_CGU_PLL_RATIO_BYTE_MASK		GENMASK(7, 0)
#define NA51089_CGU_PLL_RATIO_DIV		131072

#define NA51089_PLL_POLL_DELAY_US		1
#define NA51089_PLL_TIMEOUT_US			3000

#define NA51089_RESET_PULSE_US			10

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
	NA51089_SOURCE_FIX50M,
	NA51089_SOURCE_FIX25M,
	NA51089_SOURCE_RESERVED,
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

static const struct novatek_clk_data na51089_early_clocks[] = {
	{
		.name = "na51089-osc",
		.type = NOVATEK_CLK_FIXED_FACTOR,
		.id = NOVATEK_CLK_NO_ID,
		.hw_id = NA51089_SOURCE_OSC,
		.num_parents = 1,
		.parent_fw_name = "osc",
		.factor.mult = 1,
		.factor.div = 1,
	}, {
		.name = "na51089-fix480m",
		.type = NOVATEK_CLK_FIXED_RATE,
		.id = NOVATEK_CLK_NO_ID,
		.hw_id = NA51089_SOURCE_FIX480M,
		.fixed_rate = 480000000,
	}, {
		.name = "na51089-fix240m",
		.type = NOVATEK_CLK_FIXED_RATE,
		.id = NOVATEK_CLK_NO_ID,
		.hw_id = NA51089_SOURCE_FIX240M,
		.fixed_rate = 240000000,
	}, {
		.name = "na51089-fix192m",
		.type = NOVATEK_CLK_FIXED_RATE,
		.id = NOVATEK_CLK_NO_ID,
		.hw_id = NA51089_SOURCE_FIX192M,
		.fixed_rate = 192000000,
	}, {
		.name = "na51089-pllf320",
		.type = NOVATEK_CLK_FIXED_RATE,
		.id = NOVATEK_CLK_NO_ID,
		.hw_id = NA51089_SOURCE_PLLF320,
		.fixed_rate = 320000000,
	}, {
		.name = "na51089-fix160m",
		.type = NOVATEK_CLK_FIXED_RATE,
		.id = NOVATEK_CLK_NO_ID,
		.hw_id = NA51089_SOURCE_FIX160M,
		.fixed_rate = 160000000,
	}, {
		.name = "na51089-fix120m",
		.type = NOVATEK_CLK_FIXED_RATE,
		.id = NOVATEK_CLK_NO_ID,
		.hw_id = NA51089_SOURCE_FIX120M,
		.fixed_rate = 120000000,
	}, {
		.name = "na51089-fix96m",
		.type = NOVATEK_CLK_FIXED_RATE,
		.id = NOVATEK_CLK_NO_ID,
		.hw_id = NA51089_SOURCE_FIX96M,
		.fixed_rate = 96000000,
	}, {
		.name = "na51089-fix80m",
		.type = NOVATEK_CLK_FIXED_RATE,
		.id = NOVATEK_CLK_NO_ID,
		.hw_id = NA51089_SOURCE_FIX80M,
		.fixed_rate = 80000000,
	}, {
		.name = "na51089-fix60m",
		.type = NOVATEK_CLK_FIXED_RATE,
		.id = NOVATEK_CLK_NO_ID,
		.hw_id = NA51089_SOURCE_FIX60M,
		.fixed_rate = 60000000,
	}, {
		.name = "na51089-fix48m",
		.type = NOVATEK_CLK_FIXED_RATE,
		.id = NOVATEK_CLK_NO_ID,
		.hw_id = NA51089_SOURCE_FIX48M,
		.fixed_rate = 48000000,
	}, {
		.name = "na51089-fix24m",
		.type = NOVATEK_CLK_FIXED_RATE,
		.id = NOVATEK_CLK_NO_ID,
		.hw_id = NA51089_SOURCE_FIX24M,
		.fixed_rate = 24000000,
	}, {
		.name = "na51089-fix16m",
		.type = NOVATEK_CLK_FIXED_RATE,
		.id = NOVATEK_CLK_NO_ID,
		.hw_id = NA51089_SOURCE_FIX16M,
		.fixed_rate = 16000000,
	}, {
		.name = "na51089-fix3m",
		.type = NOVATEK_CLK_FIXED_RATE,
		.id = NOVATEK_CLK_NO_ID,
		.hw_id = NA51089_SOURCE_FIX3M,
		.fixed_rate = 3000000,
	}, {
		.name = "na51089-fix32768",
		.type = NOVATEK_CLK_FIXED_RATE,
		.id = NOVATEK_CLK_NO_ID,
		.hw_id = NA51089_SOURCE_FIX32768,
		.fixed_rate = 32768,
	}, {
		.name = "na51089-fix32k",
		.type = NOVATEK_CLK_FIXED_RATE,
		.id = NOVATEK_CLK_NO_ID,
		.hw_id = NA51089_SOURCE_FIX32K,
		.fixed_rate = 32000,
	}, {
		.name = "na51089-fix50m",
		.type = NOVATEK_CLK_FIXED_RATE,
		.id = NOVATEK_CLK_NO_ID,
		.hw_id = NA51089_SOURCE_FIX50M,
		.fixed_rate = 50000000,
	}, {
		.name = "na51089-fix25m",
		.type = NOVATEK_CLK_FIXED_RATE,
		.id = NOVATEK_CLK_NO_ID,
		.hw_id = NA51089_SOURCE_FIX25M,
		.fixed_rate = 25000000,
	}, {
		.name = "na51089-reserved",
		.type = NOVATEK_CLK_FIXED_RATE,
		.id = NOVATEK_CLK_NO_ID,
		.hw_id = NA51089_SOURCE_RESERVED,
		.fixed_rate = 0,
	}, {
		.name = "na51089-pll7",
		.type = NOVATEK_CLK_PLL,
		.id = NOVATEK_CLK_NO_ID,
		.hw_id = NA51089_SOURCE_PLL7,
		.parents = { NA51089_SOURCE_OSC },
		.num_parents = 1,
		.pll.rate0 = NA51089_PLL7_RATE0,
		.pll.enable_reg = NA51089_PLL_ENABLE,
		.pll.status_reg = NA51089_PLL_STATUS,
		.pll.enable_mask = NA51089_CGU_PLL_ENABLE_MASK(7),
		.pll.enable_value = FIELD_PREP_CONST(
			NA51089_CGU_PLL_ENABLE_MASK(7),
			NA51089_CGU_PLL_ENABLE),
		.pll.ready_mask = NA51089_CGU_PLL_STATUS_MASK(7),
		.pll.ready_value = FIELD_PREP_CONST(
			NA51089_CGU_PLL_STATUS_MASK(7),
			NA51089_CGU_PLL_READY),
	}, {
		.name = "na51089-pll8",
		.type = NOVATEK_CLK_PLL,
		.id = NOVATEK_CLK_NO_ID,
		.hw_id = NA51089_SOURCE_PLL8,
		.parents = { NA51089_SOURCE_OSC },
		.num_parents = 1,
		.pll.rate0 = NA51089_PLL8_RATE0,
		.pll.enable_reg = NA51089_PLL_ENABLE,
		.pll.status_reg = NA51089_PLL_STATUS,
		.pll.enable_mask = NA51089_CGU_PLL_ENABLE_MASK(8),
		.pll.enable_value = FIELD_PREP_CONST(
			NA51089_CGU_PLL_ENABLE_MASK(8),
			NA51089_CGU_PLL_ENABLE),
		.pll.ready_mask = NA51089_CGU_PLL_STATUS_MASK(8),
		.pll.ready_value = FIELD_PREP_CONST(
			NA51089_CGU_PLL_STATUS_MASK(8),
			NA51089_CGU_PLL_READY),
	}, {
		.name = "na51089-pll9",
		.type = NOVATEK_CLK_PLL,
		.id = NOVATEK_CLK_NO_ID,
		.hw_id = NA51089_SOURCE_PLL9,
		.parents = { NA51089_SOURCE_OSC },
		.num_parents = 1,
		.pll.rate0 = NA51089_PLL9_RATE0,
		.pll.enable_reg = NA51089_PLL_ENABLE,
		.pll.status_reg = NA51089_PLL_STATUS,
		.pll.enable_mask = NA51089_CGU_PLL_ENABLE_MASK(9),
		.pll.enable_value = FIELD_PREP_CONST(
			NA51089_CGU_PLL_ENABLE_MASK(9),
			NA51089_CGU_PLL_ENABLE),
		.pll.ready_mask = NA51089_CGU_PLL_STATUS_MASK(9),
		.pll.ready_value = FIELD_PREP_CONST(
			NA51089_CGU_PLL_STATUS_MASK(9),
			NA51089_CGU_PLL_READY),
	}, {
		.name = "na51089-cpu",
		.type = NOVATEK_CLK_MUX,
		.id = NA51089_CLK_CPU,
		.hw_id = NA51089_HW_CPU,
		.parents = { NA51089_SOURCE_FIX80M, NA51089_SOURCE_PLL8,
			     NA51089_SOURCE_FIX480M, NA51089_SOURCE_RESERVED },
		.num_parents = 4,
		.mux.reg = NA51089_SYS_CLK_RATE,
		.mux.shift = 0,
		.mux.width = 2,
		.mux.flags = CLK_MUX_READ_ONLY,
	}, {
		.name = "na51089-periph",
		.type = NOVATEK_CLK_FIXED_FACTOR,
		.id = NA51089_CLK_PERIPH,
		.hw_id = NOVATEK_CLK_NO_ID,
		.parents = { NA51089_HW_CPU },
		.num_parents = 1,
		.factor.mult = 1,
		.factor.div = 8,
	}, {
		.name = "na51089-apb",
		.type = NOVATEK_CLK_MUX,
		.id = NA51089_CLK_APB,
		.hw_id = NOVATEK_CLK_NO_ID,
		.parents = { NA51089_SOURCE_FIX48M, NA51089_SOURCE_FIX60M,
			     NA51089_SOURCE_FIX80M, NA51089_SOURCE_FIX120M },
		.num_parents = 4,
		.mux.reg = NA51089_SYS_CLK_RATE,
		.mux.shift = 8,
		.mux.width = 2,
		.mux.flags = CLK_MUX_READ_ONLY,
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
		.divider.reg = NA51089_PWM_CLK_DIV0,
		.divider.shift = 0,
		.divider.width = 14,
		.divider.flags = 0,
	}, {
		.name = "na51089-pwm4-7-divider",
		.type = NOVATEK_CLK_DIVIDER,
		.id = NOVATEK_CLK_NO_ID,
		.hw_id = NA51089_HW_PWM4_7_DIVIDER,
		.parents = { NA51089_SOURCE_FIX120M },
		.num_parents = 1,
		.divider.reg = NA51089_PWM_CLK_DIV0,
		.divider.shift = 16,
		.divider.width = 14,
		.divider.flags = 0,
	}, {
		.name = "na51089-i2s-mclk",
		.type = NOVATEK_CLK_COMPOSITE,
		.id = NOVATEK_CLK_NO_ID,
		.hw_id = NA51089_HW_I2S_MCLK,
		.parents = { NA51089_SOURCE_PLL7 },
		.num_parents = 1,
		.composite.div_reg = NA51089_AUDIO_CLK_DIV,
		.composite.div_shift = 24,
		.composite.div_width = 8,
		.composite.gate_reg = NA51089_CLK_EN0,
		.composite.gate_bit = 31,
	}, {
		.name = "na51089-timer",
		.type = NOVATEK_CLK_GATE,
		.id = NA51089_CLK_TIMER,
		.hw_id = NOVATEK_CLK_NO_ID,
		.parents = { NA51089_SOURCE_FIX3M },
		.num_parents = 1,
		.gate.reg = NA51089_CLK_EN1,
		.gate.bit = 18,
	}, {
		.name = "na51089-wdt",
		.type = NOVATEK_CLK_GATE,
		.id = NA51089_CLK_WDT,
		.hw_id = NOVATEK_CLK_NO_ID,
		.parents = { NA51089_SOURCE_OSC },
		.num_parents = 1,
		.gate.reg = NA51089_CLK_EN1,
		.gate.bit = 17,
	}, {
		.name = "na51089-drtc",
		.type = NOVATEK_CLK_GATE,
		.id = NA51089_CLK_DRTC,
		.hw_id = NOVATEK_CLK_NO_ID,
		.parents = { NA51089_SOURCE_OSC },
		.num_parents = 1,
		.gate.reg = NA51089_CLK_EN2,
		.gate.bit = 22,
	}, {
		.name = "na51089-i2c0",
		.type = NOVATEK_CLK_GATE,
		.id = NA51089_CLK_I2C0,
		.hw_id = NOVATEK_CLK_NO_ID,
		.parents = { NA51089_SOURCE_FIX48M },
		.num_parents = 1,
		.gate.reg = NA51089_CLK_EN1,
		.gate.bit = 4,
	}, {
		.name = "na51089-i2c1",
		.type = NOVATEK_CLK_GATE,
		.id = NA51089_CLK_I2C1,
		.hw_id = NOVATEK_CLK_NO_ID,
		.parents = { NA51089_SOURCE_FIX48M },
		.num_parents = 1,
		.gate.reg = NA51089_CLK_EN1,
		.gate.bit = 5,
	}, {
		.name = "na51089-i2c2",
		.type = NOVATEK_CLK_GATE,
		.id = NA51089_CLK_I2C2,
		.hw_id = NOVATEK_CLK_NO_ID,
		.parents = { NA51089_SOURCE_FIX48M },
		.num_parents = 1,
		.gate.reg = NA51089_CLK_EN1,
		.gate.bit = 31,
	}, {
		.name = "na51089-uart0",
		.type = NOVATEK_CLK_GATE,
		.id = NA51089_CLK_UART0,
		.hw_id = NOVATEK_CLK_NO_ID,
		.parents = { NA51089_SOURCE_FIX24M },
		.num_parents = 1,
		.gate.reg = NA51089_CLK_EN1,
		.gate.bit = 10,
	}, {
		.name = "na51089-adc",
		.type = NOVATEK_CLK_GATE,
		.id = NA51089_CLK_ADC,
		.hw_id = NOVATEK_CLK_NO_ID,
		.parents = { NA51089_SOURCE_FIX16M },
		.num_parents = 1,
		.gate.reg = NA51089_CLK_EN1,
		.gate.bit = 13,
	}, {
		.name = "na51089-usb2",
		.type = NOVATEK_CLK_GATE,
		.id = NA51089_CLK_USB2,
		.hw_id = NOVATEK_CLK_NO_ID,
		.parents = { NA51089_SOURCE_FIX48M },
		.num_parents = 1,
		.gate.reg = NA51089_CLK_EN1,
		.gate.bit = 19,
	}, {
		.name = "na51089-efuse",
		.type = NOVATEK_CLK_GATE,
		.id = NA51089_CLK_EFUSE,
		.hw_id = NOVATEK_CLK_NO_ID,
		.gate.reg = NA51089_CLK_EN1,
		.gate.bit = 28,
	}, {
		.name = "na51089-picnt0",
		.type = NOVATEK_CLK_GATE,
		.id = NA51089_CLK_PICNT0,
		.hw_id = NOVATEK_CLK_NO_ID,
		.parents = { NA51089_SOURCE_FIX3M },
		.num_parents = 1,
		.gate.reg = NA51089_CLK_EN2,
		.gate.bit = 8,
	}, {
		.name = "na51089-picnt1",
		.type = NOVATEK_CLK_GATE,
		.id = NA51089_CLK_PICNT1,
		.hw_id = NOVATEK_CLK_NO_ID,
		.parents = { NA51089_SOURCE_FIX3M },
		.num_parents = 1,
		.gate.reg = NA51089_CLK_EN2,
		.gate.bit = 9,
	}, {
		.name = "na51089-picnt2",
		.type = NOVATEK_CLK_GATE,
		.id = NA51089_CLK_PICNT2,
		.hw_id = NOVATEK_CLK_NO_ID,
		.parents = { NA51089_SOURCE_FIX3M },
		.num_parents = 1,
		.gate.reg = NA51089_CLK_EN2,
		.gate.bit = 10,
	}, {
		.name = "na51089-eth",
		.type = NOVATEK_CLK_GATE,
		.id = NA51089_CLK_ETH,
		.hw_id = NOVATEK_CLK_NO_ID,
		.parents = { NA51089_SOURCE_FIX50M },
		.num_parents = 1,
		.gate.reg = NA51089_CLK_EN1,
		.gate.bit = 29,
	}, {
		.name = "na51089-eth-phy",
		.type = NOVATEK_CLK_GATE,
		.id = NA51089_CLK_ETH_PHY,
		.hw_id = NOVATEK_CLK_NO_ID,
		.parents = { NA51089_SOURCE_FIX25M },
		.num_parents = 1,
		.gate.reg = NA51089_CLK_EN2,
		.gate.bit = 24,
	}, {
		.name = "na51089-pwm0",
		.type = NOVATEK_CLK_GATE,
		.id = NA51089_CLK_PWM0,
		.hw_id = NOVATEK_CLK_NO_ID,
		.flags = CLK_SET_RATE_PARENT,
		.parents = { NA51089_HW_PWM0_3_DIVIDER },
		.num_parents = 1,
		.gate.reg = NA51089_CLK_EN3,
		.gate.bit = 0,
	}, {
		.name = "na51089-pwm1",
		.type = NOVATEK_CLK_GATE,
		.id = NA51089_CLK_PWM1,
		.hw_id = NOVATEK_CLK_NO_ID,
		.flags = CLK_SET_RATE_PARENT,
		.parents = { NA51089_HW_PWM0_3_DIVIDER },
		.num_parents = 1,
		.gate.reg = NA51089_CLK_EN3,
		.gate.bit = 1,
	}, {
		.name = "na51089-pwm2",
		.type = NOVATEK_CLK_GATE,
		.id = NA51089_CLK_PWM2,
		.hw_id = NOVATEK_CLK_NO_ID,
		.flags = CLK_SET_RATE_PARENT,
		.parents = { NA51089_HW_PWM0_3_DIVIDER },
		.num_parents = 1,
		.gate.reg = NA51089_CLK_EN3,
		.gate.bit = 2,
	}, {
		.name = "na51089-pwm3",
		.type = NOVATEK_CLK_GATE,
		.id = NA51089_CLK_PWM3,
		.hw_id = NOVATEK_CLK_NO_ID,
		.flags = CLK_SET_RATE_PARENT,
		.parents = { NA51089_HW_PWM0_3_DIVIDER },
		.num_parents = 1,
		.gate.reg = NA51089_CLK_EN3,
		.gate.bit = 3,
	}, {
		.name = "na51089-pwm4",
		.type = NOVATEK_CLK_GATE,
		.id = NA51089_CLK_PWM4,
		.hw_id = NOVATEK_CLK_NO_ID,
		.flags = CLK_SET_RATE_PARENT,
		.parents = { NA51089_HW_PWM4_7_DIVIDER },
		.num_parents = 1,
		.gate.reg = NA51089_CLK_EN3,
		.gate.bit = 4,
	}, {
		.name = "na51089-pwm5",
		.type = NOVATEK_CLK_GATE,
		.id = NA51089_CLK_PWM5,
		.hw_id = NOVATEK_CLK_NO_ID,
		.flags = CLK_SET_RATE_PARENT,
		.parents = { NA51089_HW_PWM4_7_DIVIDER },
		.num_parents = 1,
		.gate.reg = NA51089_CLK_EN3,
		.gate.bit = 5,
	}, {
		.name = "na51089-pwm6",
		.type = NOVATEK_CLK_GATE,
		.id = NA51089_CLK_PWM6,
		.hw_id = NOVATEK_CLK_NO_ID,
		.flags = CLK_SET_RATE_PARENT,
		.parents = { NA51089_HW_PWM4_7_DIVIDER },
		.num_parents = 1,
		.gate.reg = NA51089_CLK_EN3,
		.gate.bit = 6,
	}, {
		.name = "na51089-pwm7",
		.type = NOVATEK_CLK_GATE,
		.id = NA51089_CLK_PWM7,
		.hw_id = NOVATEK_CLK_NO_ID,
		.flags = CLK_SET_RATE_PARENT,
		.parents = { NA51089_HW_PWM4_7_DIVIDER },
		.num_parents = 1,
		.gate.reg = NA51089_CLK_EN3,
		.gate.bit = 7,
	}, {
		.name = "na51089-dai",
		.type = NOVATEK_CLK_COMPOSITE,
		.id = NA51089_CLK_DAI,
		.hw_id = NOVATEK_CLK_NO_ID,
		.parents = { NA51089_HW_I2S_MCLK },
		.num_parents = 1,
		.composite.div_reg = NA51089_AUDIO_CLK_DIV,
		.composite.div_shift = 16,
		.composite.div_width = 8,
		.composite.gate_reg = NA51089_CLK_EN0,
		.composite.gate_bit = 29,
	}, {
		.name = "na51089-uart1",
		.type = NOVATEK_CLK_COMPOSITE,
		.id = NA51089_CLK_UART1,
		.hw_id = NOVATEK_CLK_NO_ID,
		.parents = { NA51089_SOURCE_FIX480M },
		.num_parents = 1,
		.composite.div_reg = NA51089_UART_CLK_DIV,
		.composite.div_width = 8,
		.composite.gate_reg = NA51089_CLK_EN1,
		.composite.gate_bit = 11,
	}, {
		.name = "na51089-uart2",
		.type = NOVATEK_CLK_COMPOSITE,
		.id = NA51089_CLK_UART2,
		.hw_id = NOVATEK_CLK_NO_ID,
		.parents = { NA51089_SOURCE_FIX480M },
		.num_parents = 1,
		.composite.div_reg = NA51089_UART_CLK_DIV,
		.composite.div_shift = 8,
		.composite.div_width = 8,
		.composite.gate_reg = NA51089_CLK_EN1,
		.composite.gate_bit = 22,
	}, {
		.name = "na51089-spi0",
		.type = NOVATEK_CLK_COMPOSITE,
		.id = NA51089_CLK_SPI0,
		.hw_id = NOVATEK_CLK_NO_ID,
		.parents = { NA51089_SOURCE_FIX192M },
		.num_parents = 1,
		.composite.div_reg = NA51089_SPI_CLK_DIV0,
		.composite.div_width = 11,
		.composite.gate_reg = NA51089_CLK_EN1,
		.composite.gate_bit = 6,
	}, {
		.name = "na51089-spi1",
		.type = NOVATEK_CLK_COMPOSITE,
		.id = NA51089_CLK_SPI1,
		.hw_id = NOVATEK_CLK_NO_ID,
		.parents = { NA51089_SOURCE_FIX192M },
		.num_parents = 1,
		.composite.div_reg = NA51089_SPI_CLK_DIV0,
		.composite.div_shift = 16,
		.composite.div_width = 11,
		.composite.gate_reg = NA51089_CLK_EN1,
		.composite.gate_bit = 7,
	}, {
		.name = "na51089-spi2",
		.type = NOVATEK_CLK_COMPOSITE,
		.id = NA51089_CLK_SPI2,
		.hw_id = NOVATEK_CLK_NO_ID,
		.parents = { NA51089_SOURCE_FIX192M },
		.num_parents = 1,
		.composite.div_reg = NA51089_SPI_CLK_DIV1,
		.composite.div_width = 11,
		.composite.gate_reg = NA51089_CLK_EN1,
		.composite.gate_bit = 8,
	}, {
		.name = "na51089-sdio0",
		.type = NOVATEK_CLK_COMPOSITE,
		.id = NA51089_CLK_SDIO0,
		.hw_id = NOVATEK_CLK_NO_ID,
		.parents = { NA51089_SOURCE_FIX192M, NA51089_SOURCE_FIX480M,
			     NA51089_SOURCE_PLLF320, NA51089_SOURCE_PLL9 },
		.num_parents = 4,
		.composite.mux_reg = NA51089_PERI_CLK_RATE0,
		.composite.mux_shift = 4,
		.composite.mux_width = 2,
		.composite.div_reg = NA51089_SDIO_CLK_DIV,
		.composite.div_width = 11,
		.composite.gate_reg = NA51089_CLK_EN1,
		.composite.gate_bit = 2,
	}, {
		.name = "na51089-sdio1",
		.type = NOVATEK_CLK_COMPOSITE,
		.id = NA51089_CLK_SDIO1,
		.hw_id = NOVATEK_CLK_NO_ID,
		.parents = { NA51089_SOURCE_FIX192M, NA51089_SOURCE_FIX480M,
			     NA51089_SOURCE_PLLF320, NA51089_SOURCE_PLL9 },
		.num_parents = 4,
		.composite.mux_reg = NA51089_PERI_CLK_RATE0,
		.composite.mux_shift = 8,
		.composite.mux_width = 2,
		.composite.div_reg = NA51089_SDIO_CLK_DIV,
		.composite.div_shift = 16,
		.composite.div_width = 11,
		.composite.gate_reg = NA51089_CLK_EN1,
		.composite.gate_bit = 3,
	}, {
		.name = "na51089-sdio2",
		.type = NOVATEK_CLK_COMPOSITE,
		.id = NA51089_CLK_SDIO2,
		.hw_id = NOVATEK_CLK_NO_ID,
		.parents = { NA51089_SOURCE_FIX192M, NA51089_SOURCE_FIX480M,
			     NA51089_SOURCE_PLLF320, NA51089_SOURCE_PLL9 },
		.num_parents = 4,
		.composite.mux_reg = NA51089_PERI_CLK_RATE1,
		.composite.mux_width = 2,
		.composite.div_reg = NA51089_PERI_CLK_DIV1,
		.composite.div_width = 11,
		.composite.gate_reg = NA51089_CLK_EN1,
		.composite.gate_bit = 14,
	}, {
		.name = "na51089-fspi",
		.type = NOVATEK_CLK_COMPOSITE,
		.id = NA51089_CLK_FSPI,
		.hw_id = NOVATEK_CLK_NO_ID,
		.parents = { NA51089_SOURCE_FIX480M },
		.num_parents = 1,
		.composite.div_reg = NA51089_PERI_CLK_DIV1,
		.composite.div_shift = 12,
		.composite.div_width = 6,
		.composite.gate_reg = NA51089_CLK_EN1,
	}, {
		.name = "na51089-remote",
		.type = NOVATEK_CLK_COMPOSITE,
		.id = NA51089_CLK_REMOTE,
		.hw_id = NOVATEK_CLK_NO_ID,
		.parents = { NA51089_SOURCE_FIX32768, NA51089_SOURCE_FIX32K },
		.num_parents = 2,
		.composite.mux_reg = NA51089_PERI_CLK_RATE1,
		.composite.mux_shift = 21,
		.composite.mux_width = 1,
		.composite.gate_reg = NA51089_CLK_EN1,
		.composite.gate_bit = 12,
	}, {
		.name = "na51089-crypto",
		.type = NOVATEK_CLK_COMPOSITE,
		.id = NA51089_CLK_CRYPTO,
		.hw_id = NOVATEK_CLK_NO_ID,
		.parents = { NA51089_SOURCE_FIX240M, NA51089_SOURCE_PLLF320,
			     NA51089_SOURCE_RESERVED, NA51089_SOURCE_PLL9 },
		.num_parents = 4,
		.composite.mux_reg = NA51089_CODEC_CLK_RATE,
		.composite.mux_shift = 20,
		.composite.mux_width = 2,
		.composite.gate_reg = NA51089_CLK_EN0,
		.composite.gate_bit = 23,
	}, {
		.name = "na51089-trng",
		.type = NOVATEK_CLK_COMPOSITE,
		.id = NA51089_CLK_TRNG,
		.hw_id = NOVATEK_CLK_NO_ID,
		.parents = { NA51089_SOURCE_FIX160M, NA51089_SOURCE_FIX240M },
		.num_parents = 2,
		.composite.mux_reg = NA51089_PERI_CLK_RATE1,
		.composite.mux_shift = 18,
		.composite.mux_width = 1,
		.composite.div_reg = NA51089_VIDEO_CLK_DIV,
		.composite.div_shift = 24,
		.composite.div_width = 8,
		.composite.gate_reg = NA51089_CLK_EN2,
		.composite.gate_bit = 25,
	}, {
		.name = "na51089-trng-ro",
		.type = NOVATEK_CLK_COMPOSITE,
		.id = NA51089_CLK_TRNG_RO,
		.hw_id = NOVATEK_CLK_NO_ID,
		.parents = { NA51089_SOURCE_OSC, NA51089_SOURCE_RESERVED },
		.num_parents = 2,
		.composite.mux_reg = NA51089_PERI_CLK_RATE1,
		.composite.mux_shift = 19,
		.composite.mux_width = 1,
		.composite.mux_flags = CLK_MUX_READ_ONLY,
		.composite.div_reg = NA51089_PERI_CLK_DIV2,
		.composite.div_width = 8,
		.composite.gate_reg = NA51089_CLK_EN2,
		.composite.gate_bit = 28,
	}, {
		.name = "na51089-rsa",
		.type = NOVATEK_CLK_COMPOSITE,
		.id = NA51089_CLK_RSA,
		.hw_id = NOVATEK_CLK_NO_ID,
		.parents = { NA51089_SOURCE_FIX240M, NA51089_SOURCE_PLLF320,
			     NA51089_SOURCE_RESERVED, NA51089_SOURCE_PLL9 },
		.num_parents = 4,
		.composite.mux_reg = NA51089_CODEC_CLK_RATE,
		.composite.mux_shift = 22,
		.composite.mux_width = 2,
		.composite.gate_reg = NA51089_CLK_EN2,
		.composite.gate_bit = 26,
	}, {
		.name = "na51089-hash",
		.type = NOVATEK_CLK_COMPOSITE,
		.id = NA51089_CLK_HASH,
		.hw_id = NOVATEK_CLK_NO_ID,
		.parents = { NA51089_SOURCE_FIX240M, NA51089_SOURCE_PLLF320,
			     NA51089_SOURCE_RESERVED, NA51089_SOURCE_PLL9 },
		.num_parents = 4,
		.composite.mux_reg = NA51089_PERI_CLK_RATE1,
		.composite.mux_shift = 16,
		.composite.mux_width = 2,
		.composite.gate_reg = NA51089_CLK_EN2,
		.composite.gate_bit = 27,
	}, {
		.name = "na51089-pwm8",
		.type = NOVATEK_CLK_COMPOSITE,
		.id = NA51089_CLK_PWM8,
		.hw_id = NOVATEK_CLK_NO_ID,
		.parents = { NA51089_SOURCE_FIX120M },
		.num_parents = 1,
		.composite.div_reg = NA51089_PWM_CLK_DIV1,
		.composite.div_width = 14,
		.composite.gate_reg = NA51089_CLK_EN3,
		.composite.gate_bit = 8,
	}, {
		.name = "na51089-pwm9",
		.type = NOVATEK_CLK_COMPOSITE,
		.id = NA51089_CLK_PWM9,
		.hw_id = NOVATEK_CLK_NO_ID,
		.parents = { NA51089_SOURCE_FIX120M },
		.num_parents = 1,
		.composite.div_reg = NA51089_PWM_CLK_DIV1,
		.composite.div_shift = 16,
		.composite.div_width = 14,
		.composite.gate_reg = NA51089_CLK_EN3,
		.composite.gate_bit = 9,
	}, {
		.name = "na51089-pwm10",
		.type = NOVATEK_CLK_COMPOSITE,
		.id = NA51089_CLK_PWM10,
		.hw_id = NOVATEK_CLK_NO_ID,
		.parents = { NA51089_SOURCE_FIX120M },
		.num_parents = 1,
		.composite.div_reg = NA51089_PWM_CLK_DIV2,
		.composite.div_width = 14,
		.composite.gate_reg = NA51089_CLK_EN3,
		.composite.gate_bit = 10,
	}, {
		.name = "na51089-pwm11",
		.type = NOVATEK_CLK_COMPOSITE,
		.id = NA51089_CLK_PWM11,
		.hw_id = NOVATEK_CLK_NO_ID,
		.parents = { NA51089_SOURCE_FIX120M },
		.num_parents = 1,
		.composite.div_reg = NA51089_PWM_CLK_DIV2,
		.composite.div_shift = 16,
		.composite.div_width = 14,
		.composite.gate_reg = NA51089_CLK_EN3,
		.composite.gate_bit = 11,
	},
};

static const struct novatek_auto_gate_data na51089_auto_gates[] = {
	{
		.reg = NA51089_PCLK_AUTO_GATE1,
		.mask = NA51089_CGU_AUTO_GATE_MASK(0),
		.value = FIELD_PREP_CONST(NA51089_CGU_AUTO_GATE_MASK(0),
					  NA51089_CGU_AUTO_GATE_DISABLE),
	}, {
		.reg = NA51089_PCLK_AUTO_GATE1,
		.mask = NA51089_CGU_AUTO_GATE_MASK(1),
		.value = FIELD_PREP_CONST(NA51089_CGU_AUTO_GATE_MASK(1),
					  NA51089_CGU_AUTO_GATE_DISABLE),
	}, {
		.reg = NA51089_PCLK_AUTO_GATE0,
		.mask = NA51089_CGU_AUTO_GATE_MASK(22),
		.value = FIELD_PREP_CONST(NA51089_CGU_AUTO_GATE_MASK(22),
					  NA51089_CGU_AUTO_GATE_DISABLE),
	}, {
		.reg = NA51089_PCLK_AUTO_GATE0,
		.mask = NA51089_CGU_AUTO_GATE_MASK(23),
		.value = FIELD_PREP_CONST(NA51089_CGU_AUTO_GATE_MASK(23),
					  NA51089_CGU_AUTO_GATE_DISABLE),
	}, {
		.reg = NA51089_PCLK_AUTO_GATE1,
		.mask = NA51089_CGU_AUTO_GATE_MASK(24),
		.value = FIELD_PREP_CONST(NA51089_CGU_AUTO_GATE_MASK(24),
					  NA51089_CGU_AUTO_GATE_DISABLE),
	}, {
		.reg = NA51089_PCLK_AUTO_GATE0,
		.mask = NA51089_CGU_AUTO_GATE_MASK(28),
		.value = FIELD_PREP_CONST(NA51089_CGU_AUTO_GATE_MASK(28),
					  NA51089_CGU_AUTO_GATE_DISABLE),
	}, {
		.reg = NA51089_PCLK_AUTO_GATE0,
		.mask = NA51089_CGU_AUTO_GATE_MASK(29),
		.value = FIELD_PREP_CONST(NA51089_CGU_AUTO_GATE_MASK(29),
					  NA51089_CGU_AUTO_GATE_DISABLE),
	}, {
		.reg = NA51089_PCLK_AUTO_GATE1,
		.mask = NA51089_CGU_AUTO_GATE_MASK(16),
		.value = FIELD_PREP_CONST(NA51089_CGU_AUTO_GATE_MASK(16),
					  NA51089_CGU_AUTO_GATE_DISABLE),
	}, {
		.reg = NA51089_CLK_AUTO_GATE0,
		.mask = NA51089_CGU_AUTO_GATE_MASK(24),
		.value = FIELD_PREP_CONST(NA51089_CGU_AUTO_GATE_MASK(24),
					  NA51089_CGU_AUTO_GATE_DISABLE),
	}, {
		.reg = NA51089_CLK_AUTO_GATE0,
		.mask = NA51089_CGU_AUTO_GATE_MASK(25),
		.value = FIELD_PREP_CONST(NA51089_CGU_AUTO_GATE_MASK(25),
					  NA51089_CGU_AUTO_GATE_DISABLE),
	}, {
		.reg = NA51089_CLK_AUTO_GATE0,
		.mask = NA51089_CGU_AUTO_GATE_MASK(26),
		.value = FIELD_PREP_CONST(NA51089_CGU_AUTO_GATE_MASK(26),
					  NA51089_CGU_AUTO_GATE_DISABLE),
	}, {
		.reg = NA51089_CLK_AUTO_GATE0,
		.mask = NA51089_CGU_AUTO_GATE_MASK(20),
		.value = FIELD_PREP_CONST(NA51089_CGU_AUTO_GATE_MASK(20),
					  NA51089_CGU_AUTO_GATE_DISABLE),
	}, {
		.reg = NA51089_CLK_AUTO_GATE0,
		.mask = NA51089_CGU_AUTO_GATE_MASK(21),
		.value = FIELD_PREP_CONST(NA51089_CGU_AUTO_GATE_MASK(21),
					  NA51089_CGU_AUTO_GATE_DISABLE),
	}, {
		.reg = NA51089_CLK_AUTO_GATE1,
		.mask = NA51089_CGU_AUTO_GATE_MASK(15),
		.value = FIELD_PREP_CONST(NA51089_CGU_AUTO_GATE_MASK(15),
					  NA51089_CGU_AUTO_GATE_DISABLE),
	}, {
		.reg = NA51089_CLK_AUTO_GATE0,
		.mask = NA51089_CGU_AUTO_GATE_MASK(19),
		.value = FIELD_PREP_CONST(NA51089_CGU_AUTO_GATE_MASK(19),
					  NA51089_CGU_AUTO_GATE_DISABLE),
	}, {
		.reg = NA51089_PCLK_AUTO_GATE0,
		.mask = NA51089_CGU_AUTO_GATE_MASK(31),
		.value = FIELD_PREP_CONST(NA51089_CGU_AUTO_GATE_MASK(31),
					  NA51089_CGU_AUTO_GATE_DISABLE),
	}, {
		.reg = NA51089_PCLK_AUTO_GATE0,
		.mask = NA51089_CGU_AUTO_GATE_MASK(30),
		.value = FIELD_PREP_CONST(NA51089_CGU_AUTO_GATE_MASK(30),
					  NA51089_CGU_AUTO_GATE_DISABLE),
	}, {
		.reg = NA51089_CLK_AUTO_GATE1,
		.mask = NA51089_CGU_AUTO_GATE_MASK(27),
		.value = FIELD_PREP_CONST(NA51089_CGU_AUTO_GATE_MASK(27),
					  NA51089_CGU_AUTO_GATE_DISABLE),
	}, {
		.reg = NA51089_PCLK_AUTO_GATE0,
		.mask = NA51089_CGU_AUTO_GATE_MASK(17),
		.value = FIELD_PREP_CONST(NA51089_CGU_AUTO_GATE_MASK(17),
					  NA51089_CGU_AUTO_GATE_DISABLE),
	},
};

static const struct novatek_reset_map na51089_reset_maps[] = {
	[NA51089_RESET_TIMER] = {
		.reg = NA51089_RESET1,
		.mask = NA51089_CGU_RESET_MASK(18),
		.assert_value = FIELD_PREP_CONST(NA51089_CGU_RESET_MASK(18),
						 NA51089_CGU_RESET_ASSERT),
		.deassert_value = FIELD_PREP_CONST(NA51089_CGU_RESET_MASK(18),
						   NA51089_CGU_RESET_DEASSERT),
	},
	[NA51089_RESET_WDT] = {
		.reg = NA51089_RESET1,
		.mask = NA51089_CGU_RESET_MASK(17),
		.assert_value = FIELD_PREP_CONST(NA51089_CGU_RESET_MASK(17),
						 NA51089_CGU_RESET_ASSERT),
		.deassert_value = FIELD_PREP_CONST(NA51089_CGU_RESET_MASK(17),
						   NA51089_CGU_RESET_DEASSERT),
	},
	[NA51089_RESET_RTC] = {
		.reg = NA51089_RESET1,
		.mask = NA51089_CGU_RESET_MASK(16),
		.assert_value = FIELD_PREP_CONST(NA51089_CGU_RESET_MASK(16),
						 NA51089_CGU_RESET_ASSERT),
		.deassert_value = FIELD_PREP_CONST(NA51089_CGU_RESET_MASK(16),
						   NA51089_CGU_RESET_DEASSERT),
	},
	[NA51089_RESET_DRTC] = {
		.reg = NA51089_RESET2,
		.mask = NA51089_CGU_RESET_MASK(22),
		.assert_value = FIELD_PREP_CONST(NA51089_CGU_RESET_MASK(22),
						 NA51089_CGU_RESET_ASSERT),
		.deassert_value = FIELD_PREP_CONST(NA51089_CGU_RESET_MASK(22),
						   NA51089_CGU_RESET_DEASSERT),
	},
	[NA51089_RESET_I2C0] = {
		.reg = NA51089_RESET1,
		.mask = NA51089_CGU_RESET_MASK(4),
		.assert_value = FIELD_PREP_CONST(NA51089_CGU_RESET_MASK(4),
						 NA51089_CGU_RESET_ASSERT),
		.deassert_value = FIELD_PREP_CONST(NA51089_CGU_RESET_MASK(4),
						   NA51089_CGU_RESET_DEASSERT),
	},
	[NA51089_RESET_I2C1] = {
		.reg = NA51089_RESET1,
		.mask = NA51089_CGU_RESET_MASK(5),
		.assert_value = FIELD_PREP_CONST(NA51089_CGU_RESET_MASK(5),
						 NA51089_CGU_RESET_ASSERT),
		.deassert_value = FIELD_PREP_CONST(NA51089_CGU_RESET_MASK(5),
						   NA51089_CGU_RESET_DEASSERT),
	},
	[NA51089_RESET_I2C2] = {
		.reg = NA51089_RESET1,
		.mask = NA51089_CGU_RESET_MASK(31),
		.assert_value = FIELD_PREP_CONST(NA51089_CGU_RESET_MASK(31),
						 NA51089_CGU_RESET_ASSERT),
		.deassert_value = FIELD_PREP_CONST(NA51089_CGU_RESET_MASK(31),
						   NA51089_CGU_RESET_DEASSERT),
	},
	[NA51089_RESET_UART0] = {
		.reg = NA51089_RESET1,
		.mask = NA51089_CGU_RESET_MASK(10),
		.assert_value = FIELD_PREP_CONST(NA51089_CGU_RESET_MASK(10),
						 NA51089_CGU_RESET_ASSERT),
		.deassert_value = FIELD_PREP_CONST(NA51089_CGU_RESET_MASK(10),
						   NA51089_CGU_RESET_DEASSERT),
	},
	[NA51089_RESET_UART1] = {
		.reg = NA51089_RESET1,
		.mask = NA51089_CGU_RESET_MASK(11),
		.assert_value = FIELD_PREP_CONST(NA51089_CGU_RESET_MASK(11),
						 NA51089_CGU_RESET_ASSERT),
		.deassert_value = FIELD_PREP_CONST(NA51089_CGU_RESET_MASK(11),
						   NA51089_CGU_RESET_DEASSERT),
	},
	[NA51089_RESET_UART2] = {
		.reg = NA51089_RESET1,
		.mask = NA51089_CGU_RESET_MASK(22),
		.assert_value = FIELD_PREP_CONST(NA51089_CGU_RESET_MASK(22),
						 NA51089_CGU_RESET_ASSERT),
		.deassert_value = FIELD_PREP_CONST(NA51089_CGU_RESET_MASK(22),
						   NA51089_CGU_RESET_DEASSERT),
	},
	[NA51089_RESET_SPI0] = {
		.reg = NA51089_RESET1,
		.mask = NA51089_CGU_RESET_MASK(6),
		.assert_value = FIELD_PREP_CONST(NA51089_CGU_RESET_MASK(6),
						 NA51089_CGU_RESET_ASSERT),
		.deassert_value = FIELD_PREP_CONST(NA51089_CGU_RESET_MASK(6),
						   NA51089_CGU_RESET_DEASSERT),
	},
	[NA51089_RESET_SPI1] = {
		.reg = NA51089_RESET1,
		.mask = NA51089_CGU_RESET_MASK(7),
		.assert_value = FIELD_PREP_CONST(NA51089_CGU_RESET_MASK(7),
						 NA51089_CGU_RESET_ASSERT),
		.deassert_value = FIELD_PREP_CONST(NA51089_CGU_RESET_MASK(7),
						   NA51089_CGU_RESET_DEASSERT),
	},
	[NA51089_RESET_SPI2] = {
		.reg = NA51089_RESET1,
		.mask = NA51089_CGU_RESET_MASK(8),
		.assert_value = FIELD_PREP_CONST(NA51089_CGU_RESET_MASK(8),
						 NA51089_CGU_RESET_ASSERT),
		.deassert_value = FIELD_PREP_CONST(NA51089_CGU_RESET_MASK(8),
						   NA51089_CGU_RESET_DEASSERT),
	},
	[NA51089_RESET_SDIO0] = {
		.reg = NA51089_RESET1,
		.mask = NA51089_CGU_RESET_MASK(2),
		.assert_value = FIELD_PREP_CONST(NA51089_CGU_RESET_MASK(2),
						 NA51089_CGU_RESET_ASSERT),
		.deassert_value = FIELD_PREP_CONST(NA51089_CGU_RESET_MASK(2),
						   NA51089_CGU_RESET_DEASSERT),
	},
	[NA51089_RESET_SDIO1] = {
		.reg = NA51089_RESET1,
		.mask = NA51089_CGU_RESET_MASK(3),
		.assert_value = FIELD_PREP_CONST(NA51089_CGU_RESET_MASK(3),
						 NA51089_CGU_RESET_ASSERT),
		.deassert_value = FIELD_PREP_CONST(NA51089_CGU_RESET_MASK(3),
						   NA51089_CGU_RESET_DEASSERT),
	},
	[NA51089_RESET_SDIO2] = {
		.reg = NA51089_RESET1,
		.mask = NA51089_CGU_RESET_MASK(14),
		.assert_value = FIELD_PREP_CONST(NA51089_CGU_RESET_MASK(14),
						 NA51089_CGU_RESET_ASSERT),
		.deassert_value = FIELD_PREP_CONST(NA51089_CGU_RESET_MASK(14),
						   NA51089_CGU_RESET_DEASSERT),
	},
	[NA51089_RESET_FSPI] = {
		.reg = NA51089_RESET1,
		.mask = NA51089_CGU_RESET_MASK(0),
		.assert_value = FIELD_PREP_CONST(NA51089_CGU_RESET_MASK(0),
						 NA51089_CGU_RESET_ASSERT),
		.deassert_value = FIELD_PREP_CONST(NA51089_CGU_RESET_MASK(0),
						   NA51089_CGU_RESET_DEASSERT),
	},
	[NA51089_RESET_ADC] = {
		.reg = NA51089_RESET1,
		.mask = NA51089_CGU_RESET_MASK(13),
		.assert_value = FIELD_PREP_CONST(NA51089_CGU_RESET_MASK(13),
						 NA51089_CGU_RESET_ASSERT),
		.deassert_value = FIELD_PREP_CONST(NA51089_CGU_RESET_MASK(13),
						   NA51089_CGU_RESET_DEASSERT),
	},
	[NA51089_RESET_REMOTE] = {
		.reg = NA51089_RESET1,
		.mask = NA51089_CGU_RESET_MASK(12),
		.assert_value = FIELD_PREP_CONST(NA51089_CGU_RESET_MASK(12),
						 NA51089_CGU_RESET_ASSERT),
		.deassert_value = FIELD_PREP_CONST(NA51089_CGU_RESET_MASK(12),
						   NA51089_CGU_RESET_DEASSERT),
	},
	[NA51089_RESET_USB2] = {
		.reg = NA51089_RESET1,
		.mask = NA51089_CGU_RESET_MASK(19),
		.assert_value = FIELD_PREP_CONST(NA51089_CGU_RESET_MASK(19),
						 NA51089_CGU_RESET_ASSERT),
		.deassert_value = FIELD_PREP_CONST(NA51089_CGU_RESET_MASK(19),
						   NA51089_CGU_RESET_DEASSERT),
	},
	[NA51089_RESET_EFUSE] = {
		.reg = NA51089_RESET1,
		.mask = NA51089_CGU_RESET_MASK(28),
		.assert_value = FIELD_PREP_CONST(NA51089_CGU_RESET_MASK(28),
						 NA51089_CGU_RESET_ASSERT),
		.deassert_value = FIELD_PREP_CONST(NA51089_CGU_RESET_MASK(28),
						   NA51089_CGU_RESET_DEASSERT),
	},
	[NA51089_RESET_PWM] = {
		.reg = NA51089_RESET2,
		.mask = NA51089_CGU_RESET_MASK(8),
		.assert_value = FIELD_PREP_CONST(NA51089_CGU_RESET_MASK(8),
						 NA51089_CGU_RESET_ASSERT),
		.deassert_value = FIELD_PREP_CONST(NA51089_CGU_RESET_MASK(8),
						   NA51089_CGU_RESET_DEASSERT),
	},
	[NA51089_RESET_ETH] = {
		.reg = NA51089_RESET1,
		.mask = NA51089_CGU_RESET_MASK(29),
		.assert_value = FIELD_PREP_CONST(NA51089_CGU_RESET_MASK(29),
						 NA51089_CGU_RESET_ASSERT),
		.deassert_value = FIELD_PREP_CONST(NA51089_CGU_RESET_MASK(29),
						   NA51089_CGU_RESET_DEASSERT),
	},
	[NA51089_RESET_ETH_GLUE] = {
		.reg = NA51089_RESET1,
		.mask = NA51089_CGU_RESET_MASK(30),
		.assert_value = FIELD_PREP_CONST(NA51089_CGU_RESET_MASK(30),
						 NA51089_CGU_RESET_ASSERT),
		.deassert_value = FIELD_PREP_CONST(NA51089_CGU_RESET_MASK(30),
						   NA51089_CGU_RESET_DEASSERT),
	},
	[NA51089_RESET_ETH_PHY] = {
		.reg = NA51089_RESET2,
		.mask = NA51089_CGU_RESET_MASK(30),
		.assert_value = FIELD_PREP_CONST(NA51089_CGU_RESET_MASK(30),
						 NA51089_CGU_RESET_ASSERT),
		.deassert_value = FIELD_PREP_CONST(NA51089_CGU_RESET_MASK(30),
						   NA51089_CGU_RESET_DEASSERT),
	},
	[NA51089_RESET_ETH_PHY_HI] = {
		.reg = NA51089_RESET2,
		.mask = NA51089_CGU_RESET_MASK(31),
		.assert_value = FIELD_PREP_CONST(NA51089_CGU_RESET_MASK(31),
						 NA51089_CGU_RESET_ASSERT),
		.deassert_value = FIELD_PREP_CONST(NA51089_CGU_RESET_MASK(31),
						   NA51089_CGU_RESET_DEASSERT),
	},
	[NA51089_RESET_CRYPTO] = {
		.reg = NA51089_RESET0,
		.mask = NA51089_CGU_RESET_MASK(23),
		.assert_value = FIELD_PREP_CONST(NA51089_CGU_RESET_MASK(23),
						 NA51089_CGU_RESET_ASSERT),
		.deassert_value = FIELD_PREP_CONST(NA51089_CGU_RESET_MASK(23),
						   NA51089_CGU_RESET_DEASSERT),
	},
	[NA51089_RESET_TRNG] = {
		.reg = NA51089_RESET2,
		.mask = NA51089_CGU_RESET_MASK(25),
		.assert_value = FIELD_PREP_CONST(NA51089_CGU_RESET_MASK(25),
						 NA51089_CGU_RESET_ASSERT),
		.deassert_value = FIELD_PREP_CONST(NA51089_CGU_RESET_MASK(25),
						   NA51089_CGU_RESET_DEASSERT),
	},
	[NA51089_RESET_RSA] = {
		.reg = NA51089_RESET2,
		.mask = NA51089_CGU_RESET_MASK(26),
		.assert_value = FIELD_PREP_CONST(NA51089_CGU_RESET_MASK(26),
						 NA51089_CGU_RESET_ASSERT),
		.deassert_value = FIELD_PREP_CONST(NA51089_CGU_RESET_MASK(26),
						   NA51089_CGU_RESET_DEASSERT),
	},
	[NA51089_RESET_HASH] = {
		.reg = NA51089_RESET2,
		.mask = NA51089_CGU_RESET_MASK(27),
		.assert_value = FIELD_PREP_CONST(NA51089_CGU_RESET_MASK(27),
						 NA51089_CGU_RESET_ASSERT),
		.deassert_value = FIELD_PREP_CONST(NA51089_CGU_RESET_MASK(27),
						   NA51089_CGU_RESET_DEASSERT),
	},
	[NA51089_RESET_DAI] = {
		.reg = NA51089_RESET0,
		.mask = NA51089_CGU_RESET_MASK(29),
		.assert_value = FIELD_PREP_CONST(NA51089_CGU_RESET_MASK(29),
						 NA51089_CGU_RESET_ASSERT),
		.deassert_value = FIELD_PREP_CONST(NA51089_CGU_RESET_MASK(29),
						   NA51089_CGU_RESET_DEASSERT),
	},
};

static ulong na51089_pll_recalc_rate(void __iomem *rate0, ulong parent_rate)
{
	u32 ratio;

	ratio = readl(rate0 + NA51089_CGU_PLL_RATE0) &
		NA51089_CGU_PLL_RATIO_BYTE_MASK;
	ratio |= (readl(rate0 + NA51089_CGU_PLL_RATE1) &
		  NA51089_CGU_PLL_RATIO_BYTE_MASK) << 8;
	ratio |= (readl(rate0 + NA51089_CGU_PLL_RATE2) &
		  NA51089_CGU_PLL_RATIO_BYTE_MASK) << 16;
	return div_u64((u64)parent_rate * ratio, NA51089_CGU_PLL_RATIO_DIV);
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

CLK_OF_DECLARE_DRIVER(na51089_cgu, "novatek,na51089-cgu",
		      na51089_cgu_early_init);

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

MODULE_DESCRIPTION("Novatek NA51089 clock and reset controller");
MODULE_LICENSE("GPL");
