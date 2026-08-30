// SPDX-License-Identifier: GPL-2.0-only

#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/reset-controller.h>
#include <linux/slab.h>

#include "clk-novatek.h"

struct novatek_pll {
	struct clk_hw hw;
	struct novatek_cgu *cgu;
	const struct novatek_pll_data *data;
};

struct novatek_composite {
	struct clk_mux mux;
	struct clk_divider divider;
	struct clk_gate gate;
};

struct novatek_reset {
	struct reset_controller_dev rcdev;
	struct novatek_cgu *cgu;
};

static int novatek_pll_enable(struct clk_hw *hw)
{
	struct novatek_pll *pll = container_of(hw, struct novatek_pll, hw);
	const struct novatek_pll_data *data = pll->data;
	struct novatek_cgu *cgu = pll->cgu;
	ulong flags;
	u32 value;

	spin_lock_irqsave(&cgu->lock, flags);
	value = readl(cgu->early_base + data->enable_reg);
	value &= ~data->enable_mask;
	value |= data->enable_value;
	writel(value, cgu->early_base + data->enable_reg);
	spin_unlock_irqrestore(&cgu->lock, flags);

	return readl_poll_timeout_atomic(
		cgu->early_base + data->status_reg, value,
		(value & data->ready_mask) == data->ready_value,
		cgu->data->pll_poll_delay_us, cgu->data->pll_timeout_us);
}

static int novatek_pll_is_enabled(struct clk_hw *hw)
{
	struct novatek_pll *pll = container_of(hw, struct novatek_pll, hw);
	const struct novatek_pll_data *data = pll->data;

	return (readl(pll->cgu->early_base + data->enable_reg) &
		data->enable_mask) == data->enable_value;
}

static unsigned long novatek_pll_recalc_rate(struct clk_hw *hw,
					     unsigned long parent_rate)
{
	struct novatek_pll *pll = container_of(hw, struct novatek_pll, hw);
	struct novatek_cgu *cgu = pll->cgu;

	return cgu->data->pll_recalc_rate(cgu->early_base + pll->data->rate0,
					parent_rate);
}

static const struct clk_ops novatek_pll_ops = {
	.enable = novatek_pll_enable,
	.is_enabled = novatek_pll_is_enabled,
	.recalc_rate = novatek_pll_recalc_rate,
};

static struct clk_hw *__init novatek_clk_register_pll(
	struct novatek_cgu *cgu, const struct novatek_clk_data *cfg,
	const struct clk_hw *parent)
{
	struct novatek_pll *pll;
	struct clk_init_data init = {
		.name = cfg->name,
		.ops = &novatek_pll_ops,
		.parent_hws = &parent,
		.num_parents = 1,
		.flags = cfg->flags,
	};
	int ret;

	pll = kzalloc_obj(*pll);
	if (!pll)
		return ERR_PTR(-ENOMEM);

	pll->cgu = cgu;
	pll->data = &cfg->pll;
	pll->hw.init = &init;
	ret = clk_hw_register(NULL, &pll->hw);
	if (ret) {
		kfree(pll);
		return ERR_PTR(ret);
	}
	return &pll->hw;
}

static struct clk_hw *__init novatek_clk_register_early(
	struct device_node *np, struct novatek_cgu *cgu,
	const struct novatek_clk_data *cfg)
{
	const struct clk_hw *parents[ARRAY_SIZE(cfg->parents)] = {};
	uint i;

	if (!cfg->parent_fw_name) {
		for (i = 0; i < cfg->num_parents; i++)
			parents[i] = cgu->clks[cfg->parents[i]];
	}

	switch (cfg->type) {
	case NOVATEK_CLK_FIXED_RATE:
		return clk_hw_register_fixed_rate(NULL, cfg->name, NULL,
						  cfg->flags,
						  cfg->fixed_rate);
	case NOVATEK_CLK_FIXED_FACTOR:
		if (cfg->parent_fw_name)
			return clk_hw_register_fixed_factor_fwname(
				NULL, np, cfg->name, cfg->parent_fw_name,
				cfg->flags, cfg->factor.mult, cfg->factor.div);
		return clk_hw_register_fixed_factor_parent_hw(
			NULL, cfg->name, parents[0], cfg->flags,
			cfg->factor.mult, cfg->factor.div);
	case NOVATEK_CLK_PLL:
		return novatek_clk_register_pll(cgu, cfg, parents[0]);
	case NOVATEK_CLK_MUX:
		return clk_hw_register_mux_hws(
			NULL, cfg->name, parents, cfg->num_parents, cfg->flags,
			cgu->early_base + cfg->mux.reg, cfg->mux.shift,
			cfg->mux.width, cfg->mux.flags, &cgu->lock);
	default:
		return ERR_PTR(-EINVAL);
	}
}

struct novatek_cgu *__init novatek_cgu_early_init(
	struct device_node *np, const struct novatek_cgu_match_data *data)
{
	struct clk_hw_onecell_data *hw_data;
	struct novatek_cgu *cgu;
	struct clk_hw *hw;
	uint i;
	int ret = -ENOMEM;

	cgu = kzalloc_obj(*cgu);
	if (!cgu)
		goto err;

	cgu->early_base = of_iomap(np, 0);
	if (!cgu->early_base)
		goto err_cgu;

	hw_data = kzalloc_flex(*hw_data, hws, data->num_clks);
	if (!hw_data)
		goto err_unmap;

	cgu->clks = kcalloc(data->num_hws, sizeof(*cgu->clks), GFP_KERNEL);
	if (!cgu->clks)
		goto err_hw_data;

	spin_lock_init(&cgu->lock);
	cgu->data = data;
	cgu->hw_data = hw_data;
	hw_data->num = data->num_clks;
	for (i = 0; i < data->num_clks; i++)
		hw_data->hws[i] = ERR_PTR(-EPROBE_DEFER);
	for (i = 0; i < data->num_hws; i++)
		cgu->clks[i] = ERR_PTR(-ENOENT);

	for (i = 0; i < data->num_early_clocks; i++) {
		const struct novatek_clk_data *cfg = &data->early_clocks[i];

		hw = novatek_clk_register_early(np, cgu, cfg);
		if (IS_ERR(hw)) {
			ret = PTR_ERR(hw);
			goto err;
		}
		if (cfg->hw_id != NOVATEK_CLK_NO_ID)
			cgu->clks[cfg->hw_id] = hw;
		if (cfg->id != NOVATEK_CLK_NO_ID)
			hw_data->hws[cfg->id] = hw;
	}

	ret = of_clk_add_hw_provider(np, of_clk_hw_onecell_get, hw_data);
	if (ret)
		goto err;
	return cgu;

err_hw_data:
	kfree(hw_data);
err_unmap:
	iounmap(cgu->early_base);
err_cgu:
	kfree(cgu);
err:
	pr_err("%pOF: failed to register clocks: %d\n", np, ret);
	return NULL;
}

static struct clk_hw *novatek_clk_register_composite(
	struct device *dev, struct novatek_cgu *cgu,
	const struct novatek_clk_data *cfg, struct clk_hw **clks)
{
	struct clk_parent_data parents[ARRAY_SIZE(cfg->parents)] = {};
	const struct novatek_composite_data *data = &cfg->composite;
	const struct clk_ops *divider_ops = NULL;
	const struct clk_ops *mux_ops = NULL;
	struct novatek_composite *composite;
	struct clk_hw *divider_hw = NULL;
	struct clk_hw *mux_hw = NULL;
	uint i;

	composite = devm_kzalloc(dev, sizeof(*composite), GFP_KERNEL);
	if (!composite)
		return ERR_PTR(-ENOMEM);

	for (i = 0; i < cfg->num_parents; i++)
		parents[i].hw = clks[cfg->parents[i]];

	if (data->mux_width) {
		composite->mux.reg = cgu->base + data->mux_reg;
		composite->mux.shift = data->mux_shift;
		composite->mux.mask = BIT(data->mux_width) - 1;
		composite->mux.flags = data->mux_flags;
		composite->mux.lock = &cgu->lock;
		mux_hw = &composite->mux.hw;
		mux_ops = data->mux_flags & CLK_MUX_READ_ONLY ?
			  &clk_mux_ro_ops : &clk_mux_ops;
	}

	if (data->div_width) {
		composite->divider.reg = cgu->base + data->div_reg;
		composite->divider.shift = data->div_shift;
		composite->divider.width = data->div_width;
		composite->divider.lock = &cgu->lock;
		divider_hw = &composite->divider.hw;
		divider_ops = &clk_divider_ops;
	}

	composite->gate.reg = cgu->base + data->gate_reg;
	composite->gate.bit_idx = data->gate_bit;
	composite->gate.lock = &cgu->lock;
	return devm_clk_hw_register_composite_pdata(
		dev, cfg->name, parents, cfg->num_parents, mux_hw, mux_ops,
		divider_hw, divider_ops, &composite->gate.hw, &clk_gate_ops,
		cfg->flags);
}

static struct clk_hw *novatek_clk_register_late(
	struct device *dev, struct novatek_cgu *cgu,
	const struct novatek_clk_data *cfg, struct clk_hw **clks)
{
	const struct clk_hw *parent = NULL;

	if (cfg->num_parents)
		parent = clks[cfg->parents[0]];

	switch (cfg->type) {
	case NOVATEK_CLK_DIVIDER:
		return devm_clk_hw_register_divider_parent_hw(
			dev, cfg->name, parent, cfg->flags,
			cgu->base + cfg->divider.reg, cfg->divider.shift,
			cfg->divider.width, cfg->divider.flags, &cgu->lock);
	case NOVATEK_CLK_GATE:
		if (!cfg->num_parents)
			return devm_clk_hw_register_gate(
				dev, cfg->name, NULL, cfg->flags,
				cgu->base + cfg->gate.reg, cfg->gate.bit,
				cfg->gate.flags, &cgu->lock);
		return devm_clk_hw_register_gate_parent_hw(
			dev, cfg->name, parent, cfg->flags,
			cgu->base + cfg->gate.reg, cfg->gate.bit,
			cfg->gate.flags, &cgu->lock);
	case NOVATEK_CLK_COMPOSITE:
		return novatek_clk_register_composite(dev, cgu, cfg, clks);
	default:
		return ERR_PTR(-EINVAL);
	}
}

static void novatek_clk_disable_auto_gates(struct novatek_cgu *cgu)
{
	const struct novatek_auto_gate_data *data = cgu->data->auto_gates;
	uint num_gates = cgu->data->num_auto_gates;
	ulong flags;
	uint i;

	spin_lock_irqsave(&cgu->lock, flags);
	for (i = 0; i < num_gates; i++) {
		u32 value = readl(cgu->base + data[i].reg);

		value &= ~data[i].mask;
		value |= data[i].value;
		writel(value, cgu->base + data[i].reg);
	}

	if (num_gates)
		readl(cgu->base + data[num_gates - 1].reg);
	spin_unlock_irqrestore(&cgu->lock, flags);
}

static int novatek_reset_update(struct reset_controller_dev *rcdev, ulong id,
				bool assert)
{
	struct novatek_reset *reset;
	struct novatek_cgu *cgu;
	const struct novatek_reset_map *map;
	ulong flags;
	u32 value;

	reset = container_of(rcdev, struct novatek_reset, rcdev);
	cgu = reset->cgu;
	map = &cgu->data->resets[id];
	spin_lock_irqsave(&cgu->lock, flags);
	value = readl(cgu->base + map->reg);
	value &= ~map->mask;
	value |= assert ? map->assert_value : map->deassert_value;
	writel(value, cgu->base + map->reg);
	spin_unlock_irqrestore(&cgu->lock, flags);

	return 0;
}

static int novatek_reset_assert(struct reset_controller_dev *rcdev,
				unsigned long id)
{
	return novatek_reset_update(rcdev, id, true);
}

static int novatek_reset_deassert(struct reset_controller_dev *rcdev,
				  unsigned long id)
{
	return novatek_reset_update(rcdev, id, false);
}

static int novatek_reset_status(struct reset_controller_dev *rcdev,
				unsigned long id)
{
	struct novatek_reset *reset;
	const struct novatek_reset_map *map;

	reset = container_of(rcdev, struct novatek_reset, rcdev);
	map = &reset->cgu->data->resets[id];
	return (readl(reset->cgu->base + map->reg) & map->mask) ==
		map->assert_value;
}

static int novatek_reset_reset(struct reset_controller_dev *rcdev,
			       unsigned long id)
{
	struct novatek_reset *reset;

	reset = container_of(rcdev, struct novatek_reset, rcdev);
	novatek_reset_assert(rcdev, id);
	udelay(reset->cgu->data->reset_pulse_us);

	return novatek_reset_deassert(rcdev, id);
}

static const struct reset_control_ops novatek_reset_ops = {
	.assert = novatek_reset_assert,
	.deassert = novatek_reset_deassert,
	.reset = novatek_reset_reset,
	.status = novatek_reset_status,
};

static int novatek_reset_controller_register(struct device *dev,
					     struct novatek_cgu *cgu)
{
	struct novatek_reset *reset;

	reset = devm_kzalloc(dev, sizeof(*reset), GFP_KERNEL);
	if (!reset)
		return -ENOMEM;

	reset->cgu = cgu;
	reset->rcdev.owner = THIS_MODULE;
	reset->rcdev.nr_resets = cgu->data->num_resets;
	reset->rcdev.ops = &novatek_reset_ops;
	reset->rcdev.of_node = dev->of_node;
	return devm_reset_controller_register(dev, &reset->rcdev);
}

int novatek_cgu_probe(struct platform_device *pdev, struct novatek_cgu *cgu)
{
	struct device *dev = &pdev->dev;
	const struct novatek_cgu_match_data *data;
	struct clk_hw **clks;
	struct clk_hw **hws;
	struct clk_hw *hw;
	uint i;
	int ret;

	if (!cgu)
		return dev_err_probe(dev, -ENODEV,
				     "early clock provider is unavailable\n");

	data = of_device_get_match_data(dev);
	if (data != cgu->data)
		return dev_err_probe(
			dev, -EINVAL,
			"clock match data differs from early provider\n");

	cgu->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(cgu->base))
		return PTR_ERR(cgu->base);

	hws = kcalloc(data->num_clks, sizeof(*hws), GFP_KERNEL);
	if (!hws)
		return -ENOMEM;

	clks = kmemdup_array(cgu->clks, data->num_hws, sizeof(*clks),
			     GFP_KERNEL);
	if (!clks) {
		ret = -ENOMEM;
		goto out_hws;
	}

	for (i = 0; i < data->num_clks; i++) {
		if (cgu->hw_data->hws[i] == ERR_PTR(-EPROBE_DEFER))
			hws[i] = ERR_PTR(-ENOENT);
		else
			hws[i] = cgu->hw_data->hws[i];
	}

	for (i = 0; i < data->num_late_clocks; i++) {
		const struct novatek_clk_data *cfg = &data->late_clocks[i];

		hw = novatek_clk_register_late(dev, cgu, cfg, clks);
		if (IS_ERR(hw)) {
			ret = dev_err_probe(dev, PTR_ERR(hw),
					    "failed to register %s\n",
					    cfg->name);
			goto out_clks;
		}
		if (cfg->hw_id != NOVATEK_CLK_NO_ID)
			clks[cfg->hw_id] = hw;
		if (cfg->id != NOVATEK_CLK_NO_ID)
			hws[cfg->id] = hw;
	}

	novatek_clk_disable_auto_gates(cgu);

	ret = novatek_reset_controller_register(dev, cgu);
	if (ret) {
		dev_err_probe(dev, ret,
			      "failed to register reset controller\n");
		goto out_clks;
	}

	for (i = 0; i < data->num_clks; i++)
		cgu->hw_data->hws[i] = hws[i];
	for (i = 0; i < data->num_hws; i++)
		cgu->clks[i] = clks[i];

out_clks:
	kfree(clks);
out_hws:
	kfree(hws);
	return ret;
}
