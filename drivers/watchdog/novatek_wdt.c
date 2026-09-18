// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2026 Yu-Tung Chang <mtwget@gmail.com>
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/reset.h>
#include <linux/spinlock.h>
#include <linux/time64.h>
#include <linux/watchdog.h>

#define WDT_CTRL_REG			0x00
#define WDT_STATUS_REG			0x04
#define WDT_TRIGGER_REG			0x08
#define WDT_MANUAL_RESET_REG		0x0c

/* WDT_CTRL bit fields */
#define WDT_CTRL_ENABLE			BIT(0)
#define WDT_CTRL_MODE			BIT(1)
#define WDT_CTRL_EXT_RESET		BIT(4)
#define WDT_CTRL_RESET_NUM0		BIT(5)
#define WDT_CTRL_MSB			GENMASK(15, 8)
#define WDT_CTRL_KEY			GENMASK(31, 16)
#define WDT_CTRL_KEY_VALUE		0x5a96

/* WDT_STATUS bit fields */
#define WDT_STATUS_COUNT		GENMASK(19, 0)
#define WDT_STATUS_ENABLED		BIT(30)

/* WDT_TRIGGER bit fields */
#define WDT_TRIGGER_RELOAD		BIT(0)
#define WDT_TRIGGER_RELOAD_CONFIG \
	(FIELD_PREP_CONST(WDT_TRIGGER_RELOAD, 1))

/* WDT_MANUAL_RESET bit fields */
#define WDT_MANUAL_RESET_ASSERT		BIT(0)
#define WDT_MANUAL_RESET_CONFIG \
	(FIELD_PREP_CONST(WDT_MANUAL_RESET_ASSERT, 1))

#define WDT_MSB_TICKS			4096

#define WDT_DEFAULT_TIMEOUT		80U
#define WDT_POLL_DELAY_US		1
#define WDT_POLL_TIMEOUT_US		100

struct novatek_wdt_soc_data {
	uint prescale;
};

struct novatek_wdt {
	struct watchdog_device wdd;
	const struct novatek_wdt_soc_data *soc;
	void __iomem *base;
	spinlock_t lock;
	ulong rate;
	uint enable_delay_us;
};

static int novatek_wdt_ping(struct watchdog_device *wdd)
{
	struct novatek_wdt *wdt = watchdog_get_drvdata(wdd);
	unsigned long flags;
	u32 reg;
	int ret;

	spin_lock_irqsave(&wdt->lock, flags);
	ret = readl_poll_timeout_atomic(wdt->base + WDT_TRIGGER_REG,
					reg, !(reg & WDT_TRIGGER_RELOAD),
					WDT_POLL_DELAY_US, WDT_POLL_TIMEOUT_US);
	if (!ret)
		writel(WDT_TRIGGER_RELOAD_CONFIG, wdt->base + WDT_TRIGGER_REG);
	spin_unlock_irqrestore(&wdt->lock, flags);

	return ret;
}

static int novatek_wdt_set_timeout(struct watchdog_device *wdd, unsigned int timeout)
{
	struct novatek_wdt *wdt = watchdog_get_drvdata(wdd);
	unsigned long flags;
	u32 msb;
	u32 reg;

	msb = div64_u64((u64)timeout * wdt->rate, (u64)wdt->soc->prescale * WDT_MSB_TICKS);
	if (msb > FIELD_MAX(WDT_CTRL_MSB))
		return -EINVAL;

	spin_lock_irqsave(&wdt->lock, flags);
	reg = readl(wdt->base + WDT_CTRL_REG);
	reg &= ~(WDT_CTRL_MSB | WDT_CTRL_KEY);
	reg |= FIELD_PREP(WDT_CTRL_MSB, msb) |
	       FIELD_PREP(WDT_CTRL_KEY, WDT_CTRL_KEY_VALUE);
	writel(reg, wdt->base + WDT_CTRL_REG);
	spin_unlock_irqrestore(&wdt->lock, flags);
	wdd->timeout = timeout;

	if (readl(wdt->base + WDT_STATUS_REG) & WDT_STATUS_ENABLED)
		return novatek_wdt_ping(wdd);

	return 0;
}

static int novatek_wdt_start(struct watchdog_device *wdd)
{
	struct novatek_wdt *wdt = watchdog_get_drvdata(wdd);
	unsigned long flags;
	u32 reg;
	u32 status;
	int ret;

	ret = novatek_wdt_set_timeout(wdd, wdd->timeout);
	if (ret)
		return ret;

	spin_lock_irqsave(&wdt->lock, flags);
	reg = readl(wdt->base + WDT_CTRL_REG);
	reg &= ~(WDT_CTRL_ENABLE | WDT_CTRL_MODE | WDT_CTRL_EXT_RESET | WDT_CTRL_KEY);
	reg |= WDT_CTRL_MODE |
	       WDT_CTRL_RESET_NUM0 |
	       FIELD_PREP(WDT_CTRL_KEY, WDT_CTRL_KEY_VALUE);
	writel(reg, wdt->base + WDT_CTRL_REG);

	ret = readl_poll_timeout_atomic(wdt->base + WDT_STATUS_REG,
					status, !(status & WDT_STATUS_ENABLED),
					WDT_POLL_DELAY_US, WDT_POLL_TIMEOUT_US);
	if (!ret) {
		udelay(wdt->enable_delay_us);
		reg |= WDT_CTRL_ENABLE;
		writel(reg, wdt->base + WDT_CTRL_REG);
		ret = readl_poll_timeout_atomic(wdt->base + WDT_STATUS_REG,
						status, status & WDT_STATUS_ENABLED,
						WDT_POLL_DELAY_US, WDT_POLL_TIMEOUT_US);
	}
	spin_unlock_irqrestore(&wdt->lock, flags);

	return ret;
}

static int novatek_wdt_stop(struct watchdog_device *wdd)
{
	struct novatek_wdt *wdt = watchdog_get_drvdata(wdd);
	unsigned long flags;
	u32 reg;
	u32 status;
	int ret;

	spin_lock_irqsave(&wdt->lock, flags);
	reg = readl(wdt->base + WDT_CTRL_REG);
	reg &= ~(WDT_CTRL_ENABLE | WDT_CTRL_KEY);
	reg |= FIELD_PREP(WDT_CTRL_KEY, WDT_CTRL_KEY_VALUE);
	writel(reg, wdt->base + WDT_CTRL_REG);
	ret = readl_poll_timeout_atomic(wdt->base + WDT_STATUS_REG,
					status, !(status & WDT_STATUS_ENABLED),
					WDT_POLL_DELAY_US, WDT_POLL_TIMEOUT_US);
	spin_unlock_irqrestore(&wdt->lock, flags);

	return ret;
}

static unsigned int novatek_wdt_get_timeleft(struct watchdog_device *wdd)
{
	struct novatek_wdt *wdt = watchdog_get_drvdata(wdd);
	u32 count;

	count = FIELD_GET(WDT_STATUS_COUNT, readl(wdt->base + WDT_STATUS_REG));
	return div64_u64((u64)count * wdt->soc->prescale, wdt->rate);
}

static int novatek_wdt_restart(struct watchdog_device *wdd, unsigned long action, void *data)
{
	struct novatek_wdt *wdt = watchdog_get_drvdata(wdd);
	unsigned long flags;
	u32 reg;
	u32 status;
	int ret;

	spin_lock_irqsave(&wdt->lock, flags);
	reg = readl(wdt->base + WDT_CTRL_REG);
	reg &= ~(WDT_CTRL_ENABLE | WDT_CTRL_KEY);
	reg |= FIELD_PREP(WDT_CTRL_KEY, WDT_CTRL_KEY_VALUE);
	writel(reg, wdt->base + WDT_CTRL_REG);
	ret = readl_poll_timeout_atomic(wdt->base + WDT_STATUS_REG,
					status, !(status & WDT_STATUS_ENABLED),
					WDT_POLL_DELAY_US, WDT_POLL_TIMEOUT_US);
	if (ret)
		goto out_unlock;

	reg &= ~(WDT_CTRL_MODE | WDT_CTRL_EXT_RESET | WDT_CTRL_MSB);
	reg |= WDT_CTRL_MODE |
	       WDT_CTRL_RESET_NUM0 |
	       FIELD_PREP(WDT_CTRL_MSB, 1);
	writel(reg, wdt->base + WDT_CTRL_REG);
	udelay(wdt->enable_delay_us);

	reg |= WDT_CTRL_ENABLE;
	writel(reg, wdt->base + WDT_CTRL_REG);
	ret = readl_poll_timeout_atomic(wdt->base + WDT_STATUS_REG,
					status, status & WDT_STATUS_ENABLED,
					WDT_POLL_DELAY_US, WDT_POLL_TIMEOUT_US);
	if (ret)
		goto out_unlock;

	writel(WDT_MANUAL_RESET_CONFIG, wdt->base + WDT_MANUAL_RESET_REG);
	readl(wdt->base + WDT_MANUAL_RESET_REG);
	mdelay(100);
	ret = -ETIMEDOUT;

out_unlock:
	spin_unlock_irqrestore(&wdt->lock, flags);

	return ret;
}

static const struct watchdog_info novatek_wdt_info = {
	.identity = "Novatek watchdog",
	.options = WDIOF_SETTIMEOUT | WDIOF_KEEPALIVEPING | WDIOF_MAGICCLOSE,
};

static const struct watchdog_ops novatek_wdt_ops = {
	.owner = THIS_MODULE,
	.start = novatek_wdt_start,
	.stop = novatek_wdt_stop,
	.ping = novatek_wdt_ping,
	.set_timeout = novatek_wdt_set_timeout,
	.get_timeleft = novatek_wdt_get_timeleft,
	.restart = novatek_wdt_restart,
};

static int novatek_wdt_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct novatek_wdt *wdt;
	struct reset_control *reset;
	struct clk *clk;
	int ret;

	wdt = devm_kzalloc(dev, sizeof(*wdt), GFP_KERNEL);
	if (!wdt)
		return -ENOMEM;

	wdt->soc = of_device_get_match_data(dev);
	if (!wdt->soc)
		return -ENODEV;

	wdt->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(wdt->base))
		return PTR_ERR(wdt->base);

	clk = devm_clk_get_enabled(dev, NULL);
	if (IS_ERR(clk))
		return dev_err_probe(dev, PTR_ERR(clk), "failed to enable watchdog clock\n");

	reset = devm_reset_control_get_exclusive_deasserted(dev, NULL);
	if (IS_ERR(reset))
		return dev_err_probe(dev, PTR_ERR(reset), "failed to deassert watchdog reset\n");

	wdt->rate = clk_get_rate(clk);
	if (!wdt->rate)
		return dev_err_probe(dev, -EINVAL, "invalid watchdog clock\n");
	wdt->enable_delay_us = DIV_ROUND_UP_ULL((u64)wdt->soc->prescale * USEC_PER_SEC,
						wdt->rate) + 1;

	spin_lock_init(&wdt->lock);
	wdt->wdd.info = &novatek_wdt_info;
	wdt->wdd.ops = &novatek_wdt_ops;
	wdt->wdd.parent = dev;

	wdt->wdd.min_timeout = 1;
	wdt->wdd.max_timeout = div64_u64((u64)(FIELD_MAX(WDT_CTRL_MSB) + 1) *
					 wdt->soc->prescale * WDT_MSB_TICKS,
					 wdt->rate);
	if (!wdt->wdd.max_timeout)
		return dev_err_probe(dev, -EINVAL, "watchdog clock exceeds timeout range\n");
	wdt->wdd.timeout = min(wdt->wdd.max_timeout, WDT_DEFAULT_TIMEOUT);

	watchdog_set_drvdata(&wdt->wdd, wdt);
	watchdog_init_timeout(&wdt->wdd, 0, dev);

	watchdog_set_nowayout(&wdt->wdd, WATCHDOG_NOWAYOUT);
	watchdog_stop_on_reboot(&wdt->wdd);
	watchdog_stop_on_unregister(&wdt->wdd);
	watchdog_stop_ping_on_suspend(&wdt->wdd);
	watchdog_set_restart_priority(&wdt->wdd, 128);

	platform_set_drvdata(pdev, wdt);

	if (readl(wdt->base + WDT_STATUS_REG) & WDT_STATUS_ENABLED) {
		set_bit(WDOG_HW_RUNNING, &wdt->wdd.status);
		ret = novatek_wdt_start(&wdt->wdd);
		if (ret)
			return dev_err_probe(dev, ret, "boot watchdog takeover failed\n");
	}

	return devm_watchdog_register_device(dev, &wdt->wdd);
}

#ifdef CONFIG_PM_SLEEP
static int novatek_wdt_suspend(struct device *dev)
{
	struct novatek_wdt *wdt = dev_get_drvdata(dev);

	if (watchdog_active(&wdt->wdd) || watchdog_hw_running(&wdt->wdd))
		return novatek_wdt_stop(&wdt->wdd);

	return 0;
}

static int novatek_wdt_resume(struct device *dev)
{
	struct novatek_wdt *wdt = dev_get_drvdata(dev);

	if (watchdog_active(&wdt->wdd) || watchdog_hw_running(&wdt->wdd))
		return novatek_wdt_start(&wdt->wdd);

	return 0;
}
#endif

static const struct dev_pm_ops novatek_wdt_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(novatek_wdt_suspend, novatek_wdt_resume)
};

static const struct novatek_wdt_soc_data na51089_wdt_data = {
	.prescale = 1024,
};

static const struct of_device_id novatek_wdt_of_match[] = {
	{ .compatible = "novatek,na51089-wdt", .data = &na51089_wdt_data },
	{ }
};
MODULE_DEVICE_TABLE(of, novatek_wdt_of_match);

static struct platform_driver novatek_wdt_driver = {
	.probe = novatek_wdt_probe,
	.driver = {
		.name = "novatek-wdt",
		.of_match_table = novatek_wdt_of_match,
		.pm = pm_sleep_ptr(&novatek_wdt_pm_ops),
		.suppress_bind_attrs = true,
	},
};
module_platform_driver(novatek_wdt_driver);

MODULE_AUTHOR("Yu-Tung Chang <mtwget@gmail.com>");
MODULE_DESCRIPTION("Novatek watchdog driver");
MODULE_LICENSE("GPL");
