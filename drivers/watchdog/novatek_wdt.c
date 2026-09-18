// SPDX-License-Identifier: GPL-2.0-only

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

#define NOVATEK_WDT_CTRL			0x00
#define NOVATEK_WDT_CTRL_ENABLE_MASK		BIT(0)
#define NOVATEK_WDT_CTRL_ENABLE			1
#define NOVATEK_WDT_CTRL_DISABLE		0
#define NOVATEK_WDT_CTRL_MODE_MASK		BIT(1)
#define NOVATEK_WDT_CTRL_MODE_RESET		1
#define NOVATEK_WDT_CTRL_EXT_RESET_MASK		BIT(4)
#define NOVATEK_WDT_CTRL_EXT_RESET_DISABLE	0
#define NOVATEK_WDT_CTRL_EXT_RESET_ENABLE	1
#define NOVATEK_WDT_CTRL_RESET_NUM0_MASK	BIT(5)
#define NOVATEK_WDT_CTRL_RESET_NUM0_DISABLE	0
#define NOVATEK_WDT_CTRL_RESET_NUM0_ENABLE	1
#define NOVATEK_WDT_CTRL_MSB_MASK		GENMASK(15, 8)
#define NOVATEK_WDT_CTRL_KEY_MASK		GENMASK(31, 16)
#define NOVATEK_WDT_CTRL_KEY_VALUE		0x5a96

#define NOVATEK_WDT_STATUS			0x04
#define NOVATEK_WDT_STATUS_COUNT_MASK		GENMASK(19, 0)
#define NOVATEK_WDT_STATUS_ENABLED_MASK		BIT(30)

#define NOVATEK_WDT_TRIGGER			0x08
#define NOVATEK_WDT_TRIGGER_MASK		BIT(0)
#define NOVATEK_WDT_TRIGGER_RELOAD		1

#define NOVATEK_WDT_MANUAL_RESET		0x0c
#define NOVATEK_WDT_MANUAL_RESET_MASK		BIT(0)
#define NOVATEK_WDT_MANUAL_RESET_ASSERT		1

#define NOVATEK_WDT_DEFAULT_TIMEOUT		80U
#define NOVATEK_WDT_PRESCALE			1024
#define NOVATEK_WDT_MSB_TICKS			4096
#define NOVATEK_WDT_POLL_DELAY_US		1
#define NOVATEK_WDT_POLL_TIMEOUT_US		100

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
	ret = readl_poll_timeout_atomic(wdt->base + NOVATEK_WDT_TRIGGER,
					reg,
					!(reg & NOVATEK_WDT_TRIGGER_MASK),
					NOVATEK_WDT_POLL_DELAY_US,
					NOVATEK_WDT_POLL_TIMEOUT_US);
	if (!ret)
		writel(FIELD_PREP(NOVATEK_WDT_TRIGGER_MASK,
				  NOVATEK_WDT_TRIGGER_RELOAD),
		       wdt->base + NOVATEK_WDT_TRIGGER);
	spin_unlock_irqrestore(&wdt->lock, flags);

	return ret;
}

static int novatek_wdt_set_timeout(struct watchdog_device *wdd,
				   unsigned int timeout)
{
	struct novatek_wdt *wdt = watchdog_get_drvdata(wdd);
	unsigned long flags;
	u32 msb;
	u32 reg;

	msb = div64_u64((u64)timeout * wdt->rate,
			(u64)wdt->soc->prescale * NOVATEK_WDT_MSB_TICKS);
	if (msb > FIELD_MAX(NOVATEK_WDT_CTRL_MSB_MASK))
		return -EINVAL;

	spin_lock_irqsave(&wdt->lock, flags);
	reg = readl(wdt->base + NOVATEK_WDT_CTRL);
	reg &= ~(NOVATEK_WDT_CTRL_MSB_MASK | NOVATEK_WDT_CTRL_KEY_MASK);
	reg |= FIELD_PREP(NOVATEK_WDT_CTRL_MSB_MASK, msb) |
	       FIELD_PREP(NOVATEK_WDT_CTRL_KEY_MASK,
			  NOVATEK_WDT_CTRL_KEY_VALUE);
	writel(reg, wdt->base + NOVATEK_WDT_CTRL);
	spin_unlock_irqrestore(&wdt->lock, flags);

	wdd->timeout = timeout;
	if (readl(wdt->base + NOVATEK_WDT_STATUS) &
	    NOVATEK_WDT_STATUS_ENABLED_MASK)
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
	reg = readl(wdt->base + NOVATEK_WDT_CTRL);
	reg &= ~(NOVATEK_WDT_CTRL_ENABLE_MASK |
		 NOVATEK_WDT_CTRL_MODE_MASK |
		 NOVATEK_WDT_CTRL_EXT_RESET_MASK |
		 NOVATEK_WDT_CTRL_KEY_MASK);
	reg |= FIELD_PREP(NOVATEK_WDT_CTRL_ENABLE_MASK,
			  NOVATEK_WDT_CTRL_DISABLE) |
	       FIELD_PREP(NOVATEK_WDT_CTRL_MODE_MASK,
			  NOVATEK_WDT_CTRL_MODE_RESET) |
	       FIELD_PREP(NOVATEK_WDT_CTRL_EXT_RESET_MASK,
			  NOVATEK_WDT_CTRL_EXT_RESET_DISABLE) |
	       FIELD_PREP(NOVATEK_WDT_CTRL_RESET_NUM0_MASK,
			  NOVATEK_WDT_CTRL_RESET_NUM0_ENABLE) |
	       FIELD_PREP(NOVATEK_WDT_CTRL_KEY_MASK,
			  NOVATEK_WDT_CTRL_KEY_VALUE);
	writel(reg, wdt->base + NOVATEK_WDT_CTRL);
	ret = readl_poll_timeout_atomic(wdt->base + NOVATEK_WDT_STATUS,
					status,
					!(status &
					  NOVATEK_WDT_STATUS_ENABLED_MASK),
					NOVATEK_WDT_POLL_DELAY_US,
					NOVATEK_WDT_POLL_TIMEOUT_US);
	if (!ret) {
		udelay(wdt->enable_delay_us);
		reg |= FIELD_PREP(NOVATEK_WDT_CTRL_ENABLE_MASK,
				  NOVATEK_WDT_CTRL_ENABLE);
		writel(reg, wdt->base + NOVATEK_WDT_CTRL);
		ret = readl_poll_timeout_atomic(
			wdt->base + NOVATEK_WDT_STATUS, status,
			status & NOVATEK_WDT_STATUS_ENABLED_MASK,
			NOVATEK_WDT_POLL_DELAY_US,
			NOVATEK_WDT_POLL_TIMEOUT_US);
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
	reg = readl(wdt->base + NOVATEK_WDT_CTRL);
	reg &= ~(NOVATEK_WDT_CTRL_ENABLE_MASK | NOVATEK_WDT_CTRL_KEY_MASK);
	reg |= FIELD_PREP(NOVATEK_WDT_CTRL_ENABLE_MASK,
			  NOVATEK_WDT_CTRL_DISABLE) |
	       FIELD_PREP(NOVATEK_WDT_CTRL_KEY_MASK,
			  NOVATEK_WDT_CTRL_KEY_VALUE);
	writel(reg, wdt->base + NOVATEK_WDT_CTRL);
	ret = readl_poll_timeout_atomic(wdt->base + NOVATEK_WDT_STATUS,
					status,
					!(status &
					  NOVATEK_WDT_STATUS_ENABLED_MASK),
					NOVATEK_WDT_POLL_DELAY_US,
					NOVATEK_WDT_POLL_TIMEOUT_US);
	spin_unlock_irqrestore(&wdt->lock, flags);

	return ret;
}

static unsigned int novatek_wdt_get_timeleft(struct watchdog_device *wdd)
{
	struct novatek_wdt *wdt = watchdog_get_drvdata(wdd);
	u32 count;

	count = readl(wdt->base + NOVATEK_WDT_STATUS) &
		NOVATEK_WDT_STATUS_COUNT_MASK;

	return div64_u64((u64)count * wdt->soc->prescale,
			 wdt->rate);
}

static int novatek_wdt_restart(struct watchdog_device *wdd,
			       unsigned long action, void *data)
{
	struct novatek_wdt *wdt = watchdog_get_drvdata(wdd);
	unsigned long flags;
	u32 reg;
	u32 status;
	int ret;

	spin_lock_irqsave(&wdt->lock, flags);
	reg = readl(wdt->base + NOVATEK_WDT_CTRL);
	reg &= ~(NOVATEK_WDT_CTRL_ENABLE_MASK | NOVATEK_WDT_CTRL_KEY_MASK);
	reg |= FIELD_PREP(NOVATEK_WDT_CTRL_KEY_MASK,
			  NOVATEK_WDT_CTRL_KEY_VALUE);
	writel(reg, wdt->base + NOVATEK_WDT_CTRL);
	ret = readl_poll_timeout_atomic(wdt->base + NOVATEK_WDT_STATUS,
					status,
					!(status &
					  NOVATEK_WDT_STATUS_ENABLED_MASK),
					NOVATEK_WDT_POLL_DELAY_US,
					NOVATEK_WDT_POLL_TIMEOUT_US);
	if (ret)
		goto unlock;

	reg &= ~(NOVATEK_WDT_CTRL_MODE_MASK |
		 NOVATEK_WDT_CTRL_EXT_RESET_MASK |
		 NOVATEK_WDT_CTRL_MSB_MASK);
	reg |= FIELD_PREP(NOVATEK_WDT_CTRL_MODE_MASK,
			  NOVATEK_WDT_CTRL_MODE_RESET) |
	       FIELD_PREP(NOVATEK_WDT_CTRL_EXT_RESET_MASK,
			  NOVATEK_WDT_CTRL_EXT_RESET_DISABLE) |
	       FIELD_PREP(NOVATEK_WDT_CTRL_RESET_NUM0_MASK,
			  NOVATEK_WDT_CTRL_RESET_NUM0_ENABLE) |
	       FIELD_PREP(NOVATEK_WDT_CTRL_MSB_MASK, 1);
	writel(reg, wdt->base + NOVATEK_WDT_CTRL);
	udelay(wdt->enable_delay_us);

	reg |= FIELD_PREP(NOVATEK_WDT_CTRL_ENABLE_MASK,
			  NOVATEK_WDT_CTRL_ENABLE);
	writel(reg, wdt->base + NOVATEK_WDT_CTRL);
	ret = readl_poll_timeout_atomic(wdt->base + NOVATEK_WDT_STATUS,
					status,
					status &
					NOVATEK_WDT_STATUS_ENABLED_MASK,
					NOVATEK_WDT_POLL_DELAY_US,
					NOVATEK_WDT_POLL_TIMEOUT_US);
	if (ret)
		goto unlock;

	writel(FIELD_PREP(NOVATEK_WDT_MANUAL_RESET_MASK,
			  NOVATEK_WDT_MANUAL_RESET_ASSERT),
	       wdt->base + NOVATEK_WDT_MANUAL_RESET);
	readl(wdt->base + NOVATEK_WDT_MANUAL_RESET);
	mdelay(100);
	ret = -ETIMEDOUT;

unlock:
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
		return dev_err_probe(dev, PTR_ERR(clk),
				     "failed to enable watchdog clock\n");

	reset = devm_reset_control_get_exclusive_deasserted(dev, NULL);
	if (IS_ERR(reset))
		return dev_err_probe(dev, PTR_ERR(reset),
				     "failed to deassert watchdog reset\n");

	wdt->rate = clk_get_rate(clk);
	if (!wdt->rate)
		return dev_err_probe(dev, -EINVAL, "invalid watchdog clock\n");
	wdt->enable_delay_us = DIV_ROUND_UP_ULL(
		(u64)wdt->soc->prescale * USEC_PER_SEC, wdt->rate) + 1;

	spin_lock_init(&wdt->lock);
	wdt->wdd.info = &novatek_wdt_info;
	wdt->wdd.ops = &novatek_wdt_ops;
	wdt->wdd.parent = dev;
	wdt->wdd.min_timeout = 1;
	wdt->wdd.max_timeout = div64_u64(
		(u64)(FIELD_MAX(NOVATEK_WDT_CTRL_MSB_MASK) + 1) *
		wdt->soc->prescale * NOVATEK_WDT_MSB_TICKS,
		wdt->rate);
	if (!wdt->wdd.max_timeout)
		return dev_err_probe(dev, -EINVAL,
				     "watchdog clock exceeds timeout range\n");
	wdt->wdd.timeout = min(wdt->wdd.max_timeout,
			       NOVATEK_WDT_DEFAULT_TIMEOUT);
	watchdog_set_drvdata(&wdt->wdd, wdt);
	watchdog_init_timeout(&wdt->wdd, 0, dev);
	watchdog_set_nowayout(&wdt->wdd, WATCHDOG_NOWAYOUT);
	watchdog_stop_on_reboot(&wdt->wdd);
	watchdog_stop_on_unregister(&wdt->wdd);
	watchdog_stop_ping_on_suspend(&wdt->wdd);
	watchdog_set_restart_priority(&wdt->wdd, 128);
	platform_set_drvdata(pdev, wdt);

	if (readl(wdt->base + NOVATEK_WDT_STATUS) &
	    NOVATEK_WDT_STATUS_ENABLED_MASK) {
		set_bit(WDOG_HW_RUNNING, &wdt->wdd.status);
		ret = novatek_wdt_start(&wdt->wdd);
		if (ret)
			return dev_err_probe(dev, ret,
					     "boot watchdog takeover failed\n");
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
	.prescale = NOVATEK_WDT_PRESCALE,
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

MODULE_DESCRIPTION("Novatek watchdog driver");
MODULE_LICENSE("GPL");
