// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2026 Yu-Tung Chang <mtwget@gmail.com>
 */

#include <linux/clk.h>
#include <linux/cpu.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_opp.h>
#include <linux/slab.h>

#include "cpufreq-dt.h"

#define CPUFREQ_FIXED_RATE	480000000

struct novatek_cpufreq {
	struct clk *cpu_clk;
	struct clk *pll_clk;
	struct clk *fixed_clk;
	struct platform_device *cpufreq_dt;
	int opp_token;
};

static struct platform_device *novatek_cpufreq_pdev;
static struct novatek_cpufreq *novatek_cpufreq_data;

static int novatek_cpufreq_config_clks(struct device *dev, struct opp_table *table,
				       struct dev_pm_opp *opp, void *data, bool scaling_down)
{
	struct novatek_cpufreq *priv = novatek_cpufreq_data;
	ulong rate = dev_pm_opp_get_freq(opp);
	struct clk *parent;
	int ret;

	parent = priv->fixed_clk;

	if (rate != CPUFREQ_FIXED_RATE) {
		ret = clk_set_rate(priv->pll_clk, rate);
		if (ret)
			return ret;

		parent = priv->pll_clk;
	}

	ret = clk_set_parent(priv->cpu_clk, parent);
	if (ret) {
		if (ret > 0)
			ret = -EBUSY;
		dev_err(dev, "Failed to switch CPU clock source: %d\n", ret);
	}
	return ret;
}

static const char * const novatek_cpufreq_clk_names[] = { "cpu", NULL };

static int novatek_cpufreq_probe(struct platform_device *pdev)
{
	struct dev_pm_opp_config config = {
		.clk_names = novatek_cpufreq_clk_names,
		.config_clks = novatek_cpufreq_config_clks,
	};
	struct device *dev = &pdev->dev;
	struct novatek_cpufreq *priv;
	struct device *cpu_dev;
	int ret;

	cpu_dev = get_cpu_device(0);
	if (!cpu_dev)
		return -EPROBE_DEFER;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->cpu_clk = devm_get_clk_from_child(dev, cpu_dev->of_node, "cpu");
	if (IS_ERR(priv->cpu_clk))
		return dev_err_probe(dev, PTR_ERR(priv->cpu_clk), "Failed to get CPU clock\n");

	priv->pll_clk = devm_get_clk_from_child(dev, cpu_dev->of_node, "pll8");
	if (IS_ERR(priv->pll_clk))
		return dev_err_probe(dev, PTR_ERR(priv->pll_clk), "Failed to get CPU PLL\n");

	priv->fixed_clk = devm_get_clk_from_child(dev, cpu_dev->of_node, "fixed480");
	if (IS_ERR(priv->fixed_clk))
		return dev_err_probe(dev, PTR_ERR(priv->fixed_clk), "Failed to get fixed clock\n");

	priv->opp_token = dev_pm_opp_set_config(cpu_dev, &config);
	if (priv->opp_token < 0)
		return dev_err_probe(dev, priv->opp_token, "Failed to configure OPP clocks\n");

	ret = clk_prepare_enable(priv->pll_clk);
	if (ret)
		goto out_clear_config;

	novatek_cpufreq_data = priv;
	priv->cpufreq_dt = cpufreq_dt_pdev_register(dev);
	if (IS_ERR(priv->cpufreq_dt)) {
		ret = PTR_ERR(priv->cpufreq_dt);
		novatek_cpufreq_data = NULL;
		clk_disable_unprepare(priv->pll_clk);
		goto out_clear_config;
	}

	platform_set_drvdata(pdev, priv);
	return 0;

out_clear_config:
	dev_pm_opp_clear_config(priv->opp_token);
	return dev_err_probe(dev, ret, "Failed to register CPUFreq\n");
}

static void novatek_cpufreq_remove(struct platform_device *pdev)
{
	struct novatek_cpufreq *priv = platform_get_drvdata(pdev);

	platform_device_unregister(priv->cpufreq_dt);

	dev_pm_opp_clear_config(priv->opp_token);
	novatek_cpufreq_data = NULL;

	clk_disable_unprepare(priv->pll_clk);
}

static struct platform_driver novatek_cpufreq_platform_driver = {
	.probe = novatek_cpufreq_probe,
	.remove = novatek_cpufreq_remove,
	.driver = {
		.name = "novatek-cpufreq",
	},
};

static const struct of_device_id novatek_cpufreq_match[] = {
	{ .compatible = "novatek,na51089" },
	{ }
};
MODULE_DEVICE_TABLE(of, novatek_cpufreq_match);

static int __init novatek_cpufreq_init(void)
{
	int ret;

	if (!of_machine_device_match(novatek_cpufreq_match))
		return -ENODEV;

	ret = platform_driver_register(&novatek_cpufreq_platform_driver);
	if (ret)
		return ret;

	novatek_cpufreq_pdev = platform_device_register_simple("novatek-cpufreq", -1, NULL, 0);
	if (IS_ERR(novatek_cpufreq_pdev)) {
		platform_driver_unregister(&novatek_cpufreq_platform_driver);
		return PTR_ERR(novatek_cpufreq_pdev);
	}
	return 0;
}
module_init(novatek_cpufreq_init);

static void __exit novatek_cpufreq_exit(void)
{
	platform_device_unregister(novatek_cpufreq_pdev);

	platform_driver_unregister(&novatek_cpufreq_platform_driver);
}
module_exit(novatek_cpufreq_exit);

MODULE_AUTHOR("Yu-Tung Chang <mtwget@gmail.com>");
MODULE_DESCRIPTION("Novatek CPU frequency scaling driver");
MODULE_LICENSE("GPL");
