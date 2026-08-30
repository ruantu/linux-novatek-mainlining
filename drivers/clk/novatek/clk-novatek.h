/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef _CLK_NOVATEK_H
#define _CLK_NOVATEK_H

#include <linux/clk-provider.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/spinlock.h>
#include <linux/types.h>

struct device_node;
struct platform_device;
struct novatek_cgu_match_data;

struct novatek_cgu {
	void __iomem *early_base;
	void __iomem *base;
	spinlock_t lock;
	struct clk_hw_onecell_data *hw_data;
	struct clk_hw **clks;
	const struct novatek_cgu_match_data *data;
};

enum novatek_clk_type {
	NOVATEK_CLK_FIXED_RATE,
	NOVATEK_CLK_FIXED_FACTOR,
	NOVATEK_CLK_PLL,
	NOVATEK_CLK_MUX,
	NOVATEK_CLK_DIVIDER,
	NOVATEK_CLK_GATE,
	NOVATEK_CLK_COMPOSITE,
};

enum {
	NOVATEK_CLK_NO_ID = -1,
};

struct novatek_pll_data {
	u16 rate0;
	u16 enable_reg;
	u16 status_reg;
	u32 enable_mask;
	u32 enable_value;
	u32 ready_mask;
	u32 ready_value;
};

struct novatek_mux_data {
	u16 reg;
	u8 shift;
	u8 width;
	u8 flags;
};

struct novatek_divider_data {
	u16 reg;
	u8 shift;
	u8 width;
	u8 flags;
};

struct novatek_gate_data {
	u16 reg;
	u8 bit;
	u8 flags;
};

struct novatek_composite_data {
	u16 mux_reg;
	u16 div_reg;
	u16 gate_reg;
	u8 mux_shift;
	u8 mux_width;
	u8 mux_flags;
	u8 div_shift;
	u8 div_width;
	u8 gate_bit;
};

struct novatek_clk_data {
	const char *name;
	enum novatek_clk_type type;
	int id;
	int hw_id;
	ulong flags;
	uint num_parents;
	uint parents[4];
	const char *parent_fw_name;
	union {
		ulong fixed_rate;
		struct {
			uint mult;
			uint div;
		} factor;
		struct novatek_pll_data pll;
		struct novatek_mux_data mux;
		struct novatek_divider_data divider;
		struct novatek_gate_data gate;
		struct novatek_composite_data composite;
	};
};

struct novatek_auto_gate_data {
	u16 reg;
	u32 mask;
	u32 value;
};

struct novatek_reset_map {
	u16 reg;
	u32 mask;
	u32 assert_value;
	u32 deassert_value;
};

struct novatek_cgu_match_data {
	const struct novatek_clk_data *early_clocks;
	uint num_early_clocks;
	const struct novatek_clk_data *late_clocks;
	uint num_late_clocks;
	uint num_clks;
	uint num_hws;
	const struct novatek_reset_map *resets;
	uint num_resets;
	uint reset_pulse_us;
	const struct novatek_auto_gate_data *auto_gates;
	uint num_auto_gates;
	uint pll_poll_delay_us;
	uint pll_timeout_us;
	ulong (*pll_recalc_rate)(void __iomem *rate0, ulong parent_rate);
};

struct novatek_cgu *__init novatek_cgu_early_init(
	struct device_node *np, const struct novatek_cgu_match_data *data);
int novatek_cgu_probe(struct platform_device *pdev, struct novatek_cgu *cgu);

#endif /* _CLK_NOVATEK_H */
