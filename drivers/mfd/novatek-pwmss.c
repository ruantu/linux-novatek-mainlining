// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2026 Yu-Tung Chang <mtwget@gmail.com>
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/mfd/core.h>
#include <linux/mfd/novatek-pwmss.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/reset.h>
#include <linux/slab.h>

#define PWM_INT_ENABLE_REG				0x0e0
#define PWM_MS_INT_ENABLE_REG				0x0e4
#define PWM_CCNT_INT_ENABLE_REG				0x0e8
#define PWM_CLK_INT_ENABLE_REG				0x0ec
#define PWM_PERIOD_EXT_LAST_REG				0x24c

/* PWM_INT_ENABLE bit fields */
#define PWM_INT_ENABLE_DONE(ch)				BIT(ch)
#define PWM_INT_ENABLE_DISABLED \
	(FIELD_PREP_CONST(PWM_INT_ENABLE_DONE(0), 0) | \
	 FIELD_PREP_CONST(PWM_INT_ENABLE_DONE(1), 0) | \
	 FIELD_PREP_CONST(PWM_INT_ENABLE_DONE(2), 0) | \
	 FIELD_PREP_CONST(PWM_INT_ENABLE_DONE(3), 0) | \
	 FIELD_PREP_CONST(PWM_INT_ENABLE_DONE(4), 0) | \
	 FIELD_PREP_CONST(PWM_INT_ENABLE_DONE(5), 0) | \
	 FIELD_PREP_CONST(PWM_INT_ENABLE_DONE(6), 0) | \
	 FIELD_PREP_CONST(PWM_INT_ENABLE_DONE(7), 0) | \
	 FIELD_PREP_CONST(PWM_INT_ENABLE_DONE(8), 0) | \
	 FIELD_PREP_CONST(PWM_INT_ENABLE_DONE(9), 0) | \
	 FIELD_PREP_CONST(PWM_INT_ENABLE_DONE(10), 0) | \
	 FIELD_PREP_CONST(PWM_INT_ENABLE_DONE(11), 0))

/* PWM_MS_INT_ENABLE bit fields */
#define PWM_MS_INT_ENABLE_DONE(ch)			BIT(ch)
#define PWM_MS_INT_ENABLE_DISABLED \
	(FIELD_PREP_CONST(PWM_MS_INT_ENABLE_DONE(0), 0) | \
	 FIELD_PREP_CONST(PWM_MS_INT_ENABLE_DONE(1), 0) | \
	 FIELD_PREP_CONST(PWM_MS_INT_ENABLE_DONE(2), 0) | \
	 FIELD_PREP_CONST(PWM_MS_INT_ENABLE_DONE(3), 0) | \
	 FIELD_PREP_CONST(PWM_MS_INT_ENABLE_DONE(4), 0) | \
	 FIELD_PREP_CONST(PWM_MS_INT_ENABLE_DONE(5), 0) | \
	 FIELD_PREP_CONST(PWM_MS_INT_ENABLE_DONE(6), 0) | \
	 FIELD_PREP_CONST(PWM_MS_INT_ENABLE_DONE(7), 0))

/* PWM_CCNT_INT_ENABLE bit fields */
#define PWM_CCNT_INT_ENABLE_EDGE(ch)			BIT((ch) * 4)
#define PWM_CCNT_INT_ENABLE_THRESHOLD(ch)		BIT(1 + (ch) * 4)
#define PWM_CCNT_INT_ENABLE_TIMEOUT(ch)			BIT(2 + (ch) * 4)
#define PWM_CCNT_INT_ENABLE_DISABLED \
	(FIELD_PREP_CONST(PWM_CCNT_INT_ENABLE_EDGE(0), 0) | \
	 FIELD_PREP_CONST(PWM_CCNT_INT_ENABLE_THRESHOLD(0), 0) | \
	 FIELD_PREP_CONST(PWM_CCNT_INT_ENABLE_TIMEOUT(0), 0) | \
	 FIELD_PREP_CONST(PWM_CCNT_INT_ENABLE_EDGE(1), 0) | \
	 FIELD_PREP_CONST(PWM_CCNT_INT_ENABLE_THRESHOLD(1), 0) | \
	 FIELD_PREP_CONST(PWM_CCNT_INT_ENABLE_TIMEOUT(1), 0) | \
	 FIELD_PREP_CONST(PWM_CCNT_INT_ENABLE_EDGE(2), 0) | \
	 FIELD_PREP_CONST(PWM_CCNT_INT_ENABLE_THRESHOLD(2), 0) | \
	 FIELD_PREP_CONST(PWM_CCNT_INT_ENABLE_TIMEOUT(2), 0))

/* PWM_CLK_INT_ENABLE bit fields */
#define PWM_CLK_INT_ENABLE_LOAD_DONE_0_3		BIT(0)
#define PWM_CLK_INT_ENABLE_LOAD_DONE_4_7		BIT(1)
#define PWM_CLK_INT_ENABLE_TARGET_DONE_0_3		BIT(16)
#define PWM_CLK_INT_ENABLE_TARGET_DONE_4_7		BIT(17)
#define PWM_CLK_INT_ENABLE_DISABLED \
	(FIELD_PREP_CONST(PWM_CLK_INT_ENABLE_LOAD_DONE_0_3, 0) | \
	 FIELD_PREP_CONST(PWM_CLK_INT_ENABLE_LOAD_DONE_4_7, 0) | \
	 FIELD_PREP_CONST(PWM_CLK_INT_ENABLE_TARGET_DONE_0_3, 0) | \
	 FIELD_PREP_CONST(PWM_CLK_INT_ENABLE_TARGET_DONE_4_7, 0))

static const struct regmap_config na51089_pwmss_regmap_config = {
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
	.max_register = PWM_PERIOD_EXT_LAST_REG,
	.cache_type = REGCACHE_NONE,
	.fast_io = true,
};

static const struct regmap_irq novatek_pwmss_irqs[] = {
	{ .mask = PWM_CCNT_STATUS_THRESHOLD(0) },
	{ .mask = PWM_CCNT_STATUS_THRESHOLD(1) },
	{ .mask = PWM_CCNT_STATUS_THRESHOLD(2) },
};

static const struct regmap_irq_chip novatek_pwmss_irq_chip = {
	.name = "novatek-pwm",
	.status_base = PWM_CCNT_STATUS_REG,
	.unmask_base = PWM_CCNT_INT_ENABLE_REG,
	.ack_base = PWM_CCNT_STATUS_REG,
	.init_ack_masked = true,
	.num_regs = 1,
	.irqs = novatek_pwmss_irqs,
	.num_irqs = ARRAY_SIZE(novatek_pwmss_irqs),
};

static const struct resource novatek_ccnt_resources[] = {
	DEFINE_RES_IRQ_NAMED(0, "ccnt0"),
	DEFINE_RES_IRQ_NAMED(1, "ccnt1"),
	DEFINE_RES_IRQ_NAMED(2, "ccnt2"),
};

struct novatek_pwmss_soc_data {
	uint num_pwm;
	uint num_ccnt;
};

static const struct novatek_pwmss_soc_data na51089_pwmss_data = {
	.num_pwm = 12,
	.num_ccnt = 3,
};

static const struct mfd_cell novatek_pwmss_cells[] = {
	{
		.name = "novatek-pwm",
		.of_compatible = "novatek,na51089-pwm",
	}, {
		.name = "novatek-ccnt",
		.of_compatible = "novatek,na51089-ccnt",
		.resources = novatek_ccnt_resources,
		.num_resources = ARRAY_SIZE(novatek_ccnt_resources),
	},
};

static void novatek_pwmss_reset(void *data)
{
	struct reset_control *reset = data;

	reset_control_assert(reset);
}

static int novatek_pwmss_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct novatek_pwmss_soc_data *data;
	struct novatek_pwmss *core;
	void __iomem *base;
	char *name;
	uint i;
	int irq, ret;

	data = device_get_match_data(dev);

	core = devm_kzalloc(dev, sizeof(*core), GFP_KERNEL);
	if (!core)
		return -ENOMEM;

	core->num_pwm = data->num_pwm;
	core->num_ccnt = data->num_ccnt;

	core->pwm_clks = devm_kcalloc(dev, core->num_pwm, sizeof(*core->pwm_clks), GFP_KERNEL);
	if (!core->pwm_clks)
		return -ENOMEM;

	core->ccnt_clks = devm_kcalloc(dev, core->num_ccnt, sizeof(*core->ccnt_clks), GFP_KERNEL);
	if (!core->ccnt_clks)
		return -ENOMEM;

	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base))
		return PTR_ERR(base);

	core->regmap = devm_regmap_init_mmio(dev, base, &na51089_pwmss_regmap_config);
	if (IS_ERR(core->regmap))
		return dev_err_probe(dev, PTR_ERR(core->regmap),
				     "Failed to initialize register map\n");

	for (i = 0; i < core->num_pwm; i++) {
		name = devm_kasprintf(dev, GFP_KERNEL, "pwm%u", i);
		if (!name)
			return -ENOMEM;
		core->pwm_clks[i].id = name;
	}

	ret = devm_clk_bulk_get(dev, core->num_pwm, core->pwm_clks);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to get PWM clocks\n");

	for (i = 0; i < core->num_ccnt; i++) {
		name = devm_kasprintf(dev, GFP_KERNEL, "ccnt%u", i);
		if (!name)
			return -ENOMEM;

		core->ccnt_clks[i] = devm_clk_get(dev, name);
		if (IS_ERR(core->ccnt_clks[i])) {
			ret = PTR_ERR(core->ccnt_clks[i]);
			return dev_err_probe(dev, ret, "Failed to get counter clock\n");
		}
	}

	core->reset = devm_reset_control_get_exclusive(dev, NULL);
	if (IS_ERR(core->reset))
		return dev_err_probe(dev, PTR_ERR(core->reset), "Failed to get reset\n");

	ret = clk_bulk_prepare_enable(core->num_pwm, core->pwm_clks);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to enable PWM clocks\n");

	ret = reset_control_reset(core->reset);
	clk_bulk_disable_unprepare(core->num_pwm, core->pwm_clks);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to reset controller\n");

	ret = devm_add_action_or_reset(dev, novatek_pwmss_reset, core->reset);
	if (ret)
		return ret;

	ret = regmap_write(core->regmap, PWM_INT_ENABLE_REG, PWM_INT_ENABLE_DISABLED);
	if (ret)
		return ret;

	ret = regmap_write(core->regmap, PWM_MS_INT_ENABLE_REG, PWM_MS_INT_ENABLE_DISABLED);
	if (ret)
		return ret;

	ret = regmap_write(core->regmap, PWM_CCNT_INT_ENABLE_REG, PWM_CCNT_INT_ENABLE_DISABLED);
	if (ret)
		return ret;

	ret = regmap_write(core->regmap, PWM_CLK_INT_ENABLE_REG, PWM_CLK_INT_ENABLE_DISABLED);
	if (ret)
		return ret;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;
	core->irq = irq;

	ret = devm_regmap_add_irq_chip(dev, core->regmap,
				       irq, 0, 0,
				       &novatek_pwmss_irq_chip, &core->irq_data);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to add IRQ chip\n");

	platform_set_drvdata(pdev, core);

	return devm_mfd_add_devices(dev, PLATFORM_DEVID_AUTO,
				    novatek_pwmss_cells, ARRAY_SIZE(novatek_pwmss_cells),
				    NULL, 0, regmap_irq_get_domain(core->irq_data));
}

static void novatek_pwmss_shutdown(struct platform_device *pdev)
{
	struct novatek_pwmss *core = platform_get_drvdata(pdev);

	reset_control_assert(core->reset);
}

static const struct of_device_id novatek_pwmss_of_match[] = {
	{
		.compatible = "novatek,na51089-pwmss",
		.data = &na51089_pwmss_data,
	},
	{ }
};
MODULE_DEVICE_TABLE(of, novatek_pwmss_of_match);

static struct platform_driver novatek_pwmss_driver = {
	.probe = novatek_pwmss_probe,
	.shutdown = novatek_pwmss_shutdown,
	.driver = {
		.name = "novatek-pwmss",
		.of_match_table = novatek_pwmss_of_match,
	},
};
module_platform_driver(novatek_pwmss_driver);

MODULE_AUTHOR("Yu-Tung Chang <mtwget@gmail.com>");
MODULE_DESCRIPTION("Novatek PWM and counter controller");
MODULE_LICENSE("GPL");
