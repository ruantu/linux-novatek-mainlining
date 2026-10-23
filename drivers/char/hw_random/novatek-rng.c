// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2026 Yu-Tung Chang <mtwget@gmail.com>
 */

#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/hw_random.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/reset.h>

#define RNG_CTRL_REG			0x00
#define RNG_DATA_REG			0x0c

/* RNG_CTRL bit fields */
#define RNG_CTRL_CASR			BIT(8)
#define RNG_CTRL_POST			BIT(9)

#define RNG_RESET_DELAY_US		10

struct novatek_rng {
	struct hwrng rng;
	void __iomem *base;
	struct clk *clk;
	struct clk *ro_clk;
	struct reset_control *reset;
	struct mutex lock;
	bool active;
};

static int novatek_rng_start(struct novatek_rng *rng)
{
	u32 value;
	int ret;

	ret = clk_prepare_enable(rng->ro_clk);
	if (ret)
		return ret;

	ret = reset_control_deassert(rng->reset);
	if (ret)
		goto out_disable_ro_clk;
	udelay(RNG_RESET_DELAY_US);

	ret = clk_prepare_enable(rng->clk);
	if (ret) {
		reset_control_assert(rng->reset);
		goto out_disable_ro_clk;
	}

	value = readl(rng->base + RNG_CTRL_REG);
	value |= RNG_CTRL_CASR | RNG_CTRL_POST;
	writel(value, rng->base + RNG_CTRL_REG);

	rng->active = true;
	return 0;

out_disable_ro_clk:
	clk_disable_unprepare(rng->ro_clk);
	return ret;
}

static void novatek_rng_stop(void *data)
{
	struct novatek_rng *rng = data;

	if (!rng->active)
		return;
	rng->active = false;

	clk_disable_unprepare(rng->clk);
	reset_control_assert(rng->reset);
	clk_disable_unprepare(rng->ro_clk);
}

static int novatek_rng_read(struct hwrng *hwrng, void *buf, size_t max, bool wait)
{
	struct novatek_rng *rng = container_of(hwrng, struct novatek_rng, rng);
	int ret = -EIO;

	mutex_lock(&rng->lock);
	if (rng->active) {
		*(u32 *)buf = readl(rng->base + RNG_DATA_REG);
		ret = sizeof(u32);
	}
	mutex_unlock(&rng->lock);

	return ret;
}

static int novatek_rng_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct novatek_rng *rng;
	int ret;

	rng = devm_kzalloc(dev, sizeof(*rng), GFP_KERNEL);
	if (!rng)
		return -ENOMEM;

	rng->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(rng->base))
		return PTR_ERR(rng->base);

	rng->clk = devm_clk_get(dev, "core");
	if (IS_ERR(rng->clk))
		return dev_err_probe(dev, PTR_ERR(rng->clk), "Failed to get core clock\n");

	rng->ro_clk = devm_clk_get(dev, "ro");
	if (IS_ERR(rng->ro_clk))
		return dev_err_probe(dev, PTR_ERR(rng->ro_clk),
				     "Failed to get ring oscillator clock\n");

	rng->reset = devm_reset_control_get_exclusive(dev, NULL);
	if (IS_ERR(rng->reset))
		return dev_err_probe(dev, PTR_ERR(rng->reset), "Failed to get reset\n");

	mutex_init(&rng->lock);
	platform_set_drvdata(pdev, rng);

	ret = novatek_rng_start(rng);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to start RNG\n");

	ret = devm_add_action_or_reset(dev, novatek_rng_stop, rng);
	if (ret)
		return ret;

	rng->rng.name = dev_name(dev);
	rng->rng.read = novatek_rng_read;

	return devm_hwrng_register(dev, &rng->rng);
}

#ifdef CONFIG_PM_SLEEP
static int novatek_rng_suspend(struct device *dev)
{
	struct novatek_rng *rng = dev_get_drvdata(dev);

	mutex_lock(&rng->lock);
	novatek_rng_stop(rng);
	mutex_unlock(&rng->lock);

	return 0;
}

static int novatek_rng_resume(struct device *dev)
{
	struct novatek_rng *rng = dev_get_drvdata(dev);
	int ret;

	mutex_lock(&rng->lock);
	ret = novatek_rng_start(rng);
	mutex_unlock(&rng->lock);

	return ret;
}
#endif

static const struct dev_pm_ops novatek_rng_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(novatek_rng_suspend, novatek_rng_resume)
};

static const struct of_device_id novatek_rng_of_match[] = {
	{ .compatible = "novatek,na51089-rng" },
	{ }
};
MODULE_DEVICE_TABLE(of, novatek_rng_of_match);

static struct platform_driver novatek_rng_driver = {
	.probe = novatek_rng_probe,
	.driver = {
		.name = "novatek-rng",
		.of_match_table = novatek_rng_of_match,
		.pm = &novatek_rng_pm_ops,
	},
};
module_platform_driver(novatek_rng_driver);

MODULE_AUTHOR("Yu-Tung Chang <mtwget@gmail.com>");
MODULE_DESCRIPTION("Novatek true random number generator driver");
MODULE_LICENSE("GPL");
