// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2026 Yu-Tung Chang <mtwget@gmail.com>
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/clockchips.h>
#include <linux/clocksource.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/spinlock.h>

#define TIMER_DESTINATION_REG			0x00
#define TIMER_STATUS_REG			0x10
#define TIMER_INT_ENABLE_REG			0x20
#define TIMER_CLKDIV_REG			0x30
#define TIMER_CTRL_REG(ch)			(0x100 + (ch) * 0x10)
#define TIMER_TARGET_REG(ch)			(0x104 + (ch) * 0x10)
#define TIMER_COUNTER_REG(ch)			(0x108 + (ch) * 0x10)

/* TIMER_STATUS bit fields */
#define TIMER_STATUS_CHANNEL(ch)		BIT(ch)

/* TIMER_INT_ENABLE bit fields */
#define TIMER_INT_ENABLE_CHANNEL(ch)		BIT(ch)

/* TIMER_CLKDIV bit fields */
#define TIMER_CLKDIV_DIV1			GENMASK(15, 8)
#define TIMER_CLKDIV_DIV1_BYPASS		0x00
#define TIMER_CLKDIV_DIV1_ENABLE		BIT(17)

/* TIMER_CTRL bit fields */
#define TIMER_CTRL_ENABLE			BIT(0)
#define TIMER_CTRL_MODE				BIT(1)
#define TIMER_CTRL_SOURCE			BIT(2)

#define TIMER_CLKSRC_CHANNEL			18
#define TIMER_CLKEVT_CHANNEL			19

#define TIMER_MIN_DELTA				1
#define TIMER_MAX_DELTA				U32_MAX

#define TIMER_POLL_DELAY_US			1
#define TIMER_POLL_TIMEOUT_US			100

struct novatek_timer {
	struct clocksource cs;
	struct clock_event_device ce;
	struct device *dev;
	void __iomem *base;
	raw_spinlock_t lock;
	ulong period;
};

static int novatek_timer_stop(struct novatek_timer *timer, uint channel)
{
	void __iomem *ctrl = timer->base + TIMER_CTRL_REG(channel);
	u32 value;

	value = readl(ctrl);
	value &= ~TIMER_CTRL_ENABLE;
	writel(value, ctrl);

	return readl_poll_timeout_atomic(ctrl,
					 value, !(value & TIMER_CTRL_ENABLE),
					 TIMER_POLL_DELAY_US, TIMER_POLL_TIMEOUT_US);
}

static int novatek_timer_disable(struct novatek_timer *timer, uint channel)
{
	u32 mask = TIMER_INT_ENABLE_CHANNEL(channel);
	ulong flags;
	u32 value;
	int ret;

	raw_spin_lock_irqsave(&timer->lock, flags);
	value = readl(timer->base + TIMER_INT_ENABLE_REG);
	value &= ~mask;
	writel(value, timer->base + TIMER_INT_ENABLE_REG);

	ret = novatek_timer_stop(timer, channel);

	writel(mask, timer->base + TIMER_STATUS_REG);
	raw_spin_unlock_irqrestore(&timer->lock, flags);

	return ret;
}

static int novatek_timer_program(struct novatek_timer *timer, uint channel, u32 cycles, u32 mode)
{
	u32 mask = TIMER_INT_ENABLE_CHANNEL(channel);
	void __iomem *ctrl = timer->base + TIMER_CTRL_REG(channel);
	void __iomem *status = timer->base + TIMER_STATUS_REG;
	ulong flags;
	u32 value;
	int ret;

	raw_spin_lock_irqsave(&timer->lock, flags);
	value = readl(timer->base + TIMER_INT_ENABLE_REG);
	value &= ~mask;
	writel(value, timer->base + TIMER_INT_ENABLE_REG);

	ret = novatek_timer_stop(timer, channel);
	if (ret)
		goto out_unlock;

	writel(mask, status);

	writel(cycles, timer->base + TIMER_TARGET_REG(channel));

	if (channel == TIMER_CLKEVT_CHANNEL) {
		value |= mask;
		writel(value, timer->base + TIMER_INT_ENABLE_REG);
	}

	value = readl(ctrl);
	value &= ~(TIMER_CTRL_ENABLE | TIMER_CTRL_MODE | TIMER_CTRL_SOURCE);
	value |= TIMER_CTRL_ENABLE | mode | TIMER_CTRL_SOURCE;
	writel(value, ctrl);

	ret = readl_poll_timeout_atomic(ctrl,
					value, (value & TIMER_CTRL_ENABLE) ||
					       (readl(status) & mask),
					TIMER_POLL_DELAY_US, TIMER_POLL_TIMEOUT_US);
	if (ret) {
		value = readl(timer->base + TIMER_INT_ENABLE_REG);
		value &= ~mask;
		writel(value, timer->base + TIMER_INT_ENABLE_REG);
		novatek_timer_stop(timer, channel);
		writel(mask, status);
	}

out_unlock:
	raw_spin_unlock_irqrestore(&timer->lock, flags);
	return ret;
}

static void novatek_timer_hw_init(struct novatek_timer *timer)
{
	u32 mask;
	u32 value;

	mask = TIMER_INT_ENABLE_CHANNEL(TIMER_CLKSRC_CHANNEL) |
	       TIMER_INT_ENABLE_CHANNEL(TIMER_CLKEVT_CHANNEL);
	value = readl(timer->base + TIMER_INT_ENABLE_REG);
	value &= ~mask;
	writel(value, timer->base + TIMER_INT_ENABLE_REG);

	writel(mask, timer->base + TIMER_STATUS_REG);

	value = readl(timer->base + TIMER_DESTINATION_REG);
	value |= mask;
	writel(value, timer->base + TIMER_DESTINATION_REG);

	value = readl(timer->base + TIMER_CLKDIV_REG);
	value &= ~(TIMER_CLKDIV_DIV1 | TIMER_CLKDIV_DIV1_ENABLE);
	value |= FIELD_PREP(TIMER_CLKDIV_DIV1, TIMER_CLKDIV_DIV1_BYPASS) | TIMER_CLKDIV_DIV1_ENABLE;
	writel(value, timer->base + TIMER_CLKDIV_REG);
}

static int novatek_timer_shutdown(struct clock_event_device *ce)
{
	struct novatek_timer *timer;

	timer = container_of(ce, struct novatek_timer, ce);
	return novatek_timer_disable(timer, TIMER_CLKEVT_CHANNEL);
}

static int novatek_timer_set_periodic(struct clock_event_device *ce)
{
	struct novatek_timer *timer;

	timer = container_of(ce, struct novatek_timer, ce);
	return novatek_timer_program(timer, TIMER_CLKEVT_CHANNEL, timer->period, TIMER_CTRL_MODE);
}

static int novatek_timer_next_event(unsigned long cycles, struct clock_event_device *ce)
{
	struct novatek_timer *timer;

	timer = container_of(ce, struct novatek_timer, ce);
	return novatek_timer_program(timer, TIMER_CLKEVT_CHANNEL, cycles, 0);
}

static irqreturn_t novatek_timer_interrupt(int irq, void *data)
{
	struct novatek_timer *timer = data;
	ulong flags;
	u32 status;

	raw_spin_lock_irqsave(&timer->lock, flags);
	status = readl(timer->base + TIMER_STATUS_REG);
	status &= readl(timer->base + TIMER_INT_ENABLE_REG);
	status &= TIMER_STATUS_CHANNEL(TIMER_CLKEVT_CHANNEL);
	if (!status) {
		raw_spin_unlock_irqrestore(&timer->lock, flags);
		return IRQ_NONE;
	}
	writel(status, timer->base + TIMER_STATUS_REG);
	raw_spin_unlock_irqrestore(&timer->lock, flags);

	timer->ce.event_handler(&timer->ce);

	return IRQ_HANDLED;
}

static u64 novatek_timer_read(struct clocksource *cs)
{
	struct novatek_timer *timer;
	void __iomem *counter;

	timer = container_of(cs, struct novatek_timer, cs);
	counter = timer->base + TIMER_COUNTER_REG(TIMER_CLKSRC_CHANNEL);
	return readl_relaxed(counter);
}

static void novatek_timer_quiesce(void *data)
{
	struct novatek_timer *timer = data;
	int ret;

	ret = novatek_timer_disable(timer, TIMER_CLKEVT_CHANNEL);
	if (ret)
		dev_err(timer->dev, "Failed to stop clockevent: %d\n", ret);

	ret = novatek_timer_disable(timer, TIMER_CLKSRC_CHANNEL);
	if (ret)
		dev_err(timer->dev, "Failed to stop clocksource: %d\n", ret);
}

static void novatek_timer_suspend(struct clocksource *cs)
{
	struct novatek_timer *timer;

	timer = container_of(cs, struct novatek_timer, cs);
	novatek_timer_quiesce(timer);
}

static void novatek_timer_resume(struct clocksource *cs)
{
	struct novatek_timer *timer;
	int ret;

	timer = container_of(cs, struct novatek_timer, cs);
	novatek_timer_hw_init(timer);

	ret = novatek_timer_program(timer, TIMER_CLKSRC_CHANNEL, TIMER_MAX_DELTA, TIMER_CTRL_MODE);
	if (ret)
		dev_err(timer->dev, "Failed to resume clocksource: %d\n", ret);
}

static int novatek_timer_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct reset_control *reset;
	struct novatek_timer *timer;
	struct clk *clk;
	ulong rate;
	int irq;
	int ret;

	timer = devm_kzalloc(dev, sizeof(*timer), GFP_KERNEL);
	if (!timer)
		return -ENOMEM;
	timer->dev = dev;
	raw_spin_lock_init(&timer->lock);

	timer->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(timer->base))
		return PTR_ERR(timer->base);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	clk = devm_clk_get_enabled(dev, NULL);
	if (IS_ERR(clk))
		return dev_err_probe(dev, PTR_ERR(clk), "Failed to enable clock\n");
	rate = clk_get_rate(clk);
	if (!rate)
		return dev_err_probe(dev, -EINVAL, "Invalid clock rate\n");
	timer->period = DIV_ROUND_CLOSEST(rate, HZ);

	reset = devm_reset_control_get_exclusive_deasserted(dev, NULL);
	if (IS_ERR(reset))
		return dev_err_probe(dev, PTR_ERR(reset), "Failed to get reset\n");

	ret = reset_control_reset(reset);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to reset timer\n");

	novatek_timer_hw_init(timer);

	ret = devm_request_irq(dev, irq, novatek_timer_interrupt, IRQF_TIMER, dev_name(dev), timer);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to request interrupt\n");

	ret = devm_add_action_or_reset(dev, novatek_timer_quiesce, timer);
	if (ret)
		return ret;

	ret = novatek_timer_program(timer, TIMER_CLKSRC_CHANNEL, TIMER_MAX_DELTA, TIMER_CTRL_MODE);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to start clocksource\n");

	timer->cs.name = "novatek-timer";
	timer->cs.rating = 200;
	timer->cs.read = novatek_timer_read;
	timer->cs.mask = CLOCKSOURCE_MASK(32);
	timer->cs.flags = CLOCK_SOURCE_IS_CONTINUOUS;
	timer->cs.suspend = novatek_timer_suspend;
	timer->cs.resume = novatek_timer_resume;
	timer->cs.owner = THIS_MODULE;

	ret = devm_clocksource_register_hz(dev, &timer->cs, rate);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to register clocksource\n");

	timer->ce.name = "novatek-timer";
	timer->ce.rating = 100;
	timer->ce.features = CLOCK_EVT_FEAT_PERIODIC | CLOCK_EVT_FEAT_ONESHOT;

	timer->ce.set_state_shutdown = novatek_timer_shutdown;
	timer->ce.set_state_oneshot = novatek_timer_shutdown;
	timer->ce.set_state_oneshot_stopped = novatek_timer_shutdown;
	timer->ce.set_state_periodic = novatek_timer_set_periodic;
	timer->ce.set_next_event = novatek_timer_next_event;
	timer->ce.tick_resume = novatek_timer_shutdown;

	timer->ce.cpumask = cpu_possible_mask;
	timer->ce.irq = irq;
	timer->ce.owner = THIS_MODULE;

	clockevents_config_and_register(&timer->ce, rate, TIMER_MIN_DELTA, TIMER_MAX_DELTA);

	return 0;
}

static const struct of_device_id novatek_timer_of_match[] = {
	{ .compatible = "novatek,na51089-timer" },
	{ }
};
MODULE_DEVICE_TABLE(of, novatek_timer_of_match);

static struct platform_driver novatek_timer_driver = {
	.probe = novatek_timer_probe,
	.driver = {
		.name = "novatek-timer",
		.of_match_table = novatek_timer_of_match,
		.suppress_bind_attrs = true,
	},
};
module_platform_driver(novatek_timer_driver);

MODULE_AUTHOR("Yu-Tung Chang <mtwget@gmail.com>");
MODULE_DESCRIPTION("Novatek timer driver");
MODULE_LICENSE("GPL");
