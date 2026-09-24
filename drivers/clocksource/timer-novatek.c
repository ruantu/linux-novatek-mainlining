// SPDX-License-Identifier: GPL-2.0-only

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

#define NOVATEK_TIMER_CHANNEL_MASK(ch)	BIT(ch)

#define NOVATEK_TIMER_DESTINATION	0x00

#define NOVATEK_TIMER_STATUS		0x10

#define NOVATEK_TIMER_INT_ENABLE	0x20

#define NOVATEK_TIMER_CLKDIV		0x30
#define NOVATEK_TIMER_DIV1_MASK		GENMASK(15, 8)
#define NOVATEK_TIMER_DIV1_BYPASS	0x00
#define NOVATEK_TIMER_DIV1_ENABLE_MASK	BIT(17)
#define NOVATEK_TIMER_DIV1_ENABLE	1

#define NOVATEK_TIMER_CTRL(ch)		(0x100 + (ch) * 0x10)
#define NOVATEK_TIMER_ENABLE_MASK	BIT(0)
#define NOVATEK_TIMER_ENABLE		1
#define NOVATEK_TIMER_MODE_MASK		BIT(1)
#define NOVATEK_TIMER_MODE_ONESHOT	0
#define NOVATEK_TIMER_MODE_PERIODIC	1
#define NOVATEK_TIMER_SOURCE_MASK	BIT(2)
#define NOVATEK_TIMER_SOURCE_DIV1	1

#define NOVATEK_TIMER_TARGET(ch)	(0x104 + (ch) * 0x10)

#define NOVATEK_TIMER_COUNTER(ch)	(0x108 + (ch) * 0x10)

#define NOVATEK_TIMER_REG_SIZE		0x240
#define NOVATEK_TIMER_CLKSRC_CHANNEL	18
#define NOVATEK_TIMER_CLKEVT_CHANNEL	19
#define NOVATEK_TIMER_POLL_DELAY_US	1
#define NOVATEK_TIMER_POLL_TIMEOUT_US	100
#define NOVATEK_TIMER_MIN_DELTA		1
#define NOVATEK_TIMER_MAX_DELTA		U32_MAX

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
	void __iomem *ctrl = timer->base + NOVATEK_TIMER_CTRL(channel);
	u32 value;

	value = readl(ctrl);
	value &= ~NOVATEK_TIMER_ENABLE_MASK;
	writel(value, ctrl);

	return readl_poll_timeout_atomic(ctrl, value,
					!(value & NOVATEK_TIMER_ENABLE_MASK),
					NOVATEK_TIMER_POLL_DELAY_US,
					NOVATEK_TIMER_POLL_TIMEOUT_US);
}

static int novatek_timer_disable(struct novatek_timer *timer, uint channel)
{
	u32 mask = NOVATEK_TIMER_CHANNEL_MASK(channel);
	ulong flags;
	u32 value;
	int ret;

	raw_spin_lock_irqsave(&timer->lock, flags);
	value = readl(timer->base + NOVATEK_TIMER_INT_ENABLE);
	value &= ~mask;
	writel(value, timer->base + NOVATEK_TIMER_INT_ENABLE);
	ret = novatek_timer_stop(timer, channel);
	writel(mask, timer->base + NOVATEK_TIMER_STATUS);
	raw_spin_unlock_irqrestore(&timer->lock, flags);

	return ret;
}

static int novatek_timer_program(struct novatek_timer *timer, uint channel,
				 u32 cycles, u32 mode)
{
	u32 mask = NOVATEK_TIMER_CHANNEL_MASK(channel);
	void __iomem *ctrl = timer->base + NOVATEK_TIMER_CTRL(channel);
	void __iomem *status = timer->base + NOVATEK_TIMER_STATUS;
	ulong flags;
	u32 value;
	int ret;

	raw_spin_lock_irqsave(&timer->lock, flags);
	value = readl(timer->base + NOVATEK_TIMER_INT_ENABLE);
	value &= ~mask;
	writel(value, timer->base + NOVATEK_TIMER_INT_ENABLE);
	ret = novatek_timer_stop(timer, channel);
	if (ret)
		goto out_unlock;

	writel(mask, status);
	writel(cycles, timer->base + NOVATEK_TIMER_TARGET(channel));
	if (channel == NOVATEK_TIMER_CLKEVT_CHANNEL) {
		value |= mask;
		writel(value, timer->base + NOVATEK_TIMER_INT_ENABLE);
	}

	value = readl(ctrl);
	value &= ~(NOVATEK_TIMER_ENABLE_MASK | NOVATEK_TIMER_MODE_MASK |
		   NOVATEK_TIMER_SOURCE_MASK);
	value |= FIELD_PREP(NOVATEK_TIMER_ENABLE_MASK, NOVATEK_TIMER_ENABLE) |
		 FIELD_PREP(NOVATEK_TIMER_MODE_MASK, mode) |
		 FIELD_PREP(NOVATEK_TIMER_SOURCE_MASK,
			    NOVATEK_TIMER_SOURCE_DIV1);
	writel(value, ctrl);

	ret = readl_poll_timeout_atomic(ctrl, value,
					(value & NOVATEK_TIMER_ENABLE_MASK) ||
					(readl(status) & mask),
					NOVATEK_TIMER_POLL_DELAY_US,
					NOVATEK_TIMER_POLL_TIMEOUT_US);
	if (ret) {
		value = readl(timer->base + NOVATEK_TIMER_INT_ENABLE);
		value &= ~mask;
		writel(value, timer->base + NOVATEK_TIMER_INT_ENABLE);
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

	mask = NOVATEK_TIMER_CHANNEL_MASK(NOVATEK_TIMER_CLKSRC_CHANNEL) |
	       NOVATEK_TIMER_CHANNEL_MASK(NOVATEK_TIMER_CLKEVT_CHANNEL);
	value = readl(timer->base + NOVATEK_TIMER_INT_ENABLE);
	value &= ~mask;
	writel(value, timer->base + NOVATEK_TIMER_INT_ENABLE);
	writel(mask, timer->base + NOVATEK_TIMER_STATUS);

	value = readl(timer->base + NOVATEK_TIMER_DESTINATION);
	value |= mask;
	writel(value, timer->base + NOVATEK_TIMER_DESTINATION);

	value = readl(timer->base + NOVATEK_TIMER_CLKDIV);
	value &= ~(NOVATEK_TIMER_DIV1_MASK |
		   NOVATEK_TIMER_DIV1_ENABLE_MASK);
	value |= FIELD_PREP(NOVATEK_TIMER_DIV1_MASK,
			    NOVATEK_TIMER_DIV1_BYPASS) |
		 FIELD_PREP(NOVATEK_TIMER_DIV1_ENABLE_MASK,
			    NOVATEK_TIMER_DIV1_ENABLE);
	writel(value, timer->base + NOVATEK_TIMER_CLKDIV);
}

static int novatek_timer_shutdown(struct clock_event_device *ce)
{
	struct novatek_timer *timer;

	timer = container_of(ce, struct novatek_timer, ce);
	return novatek_timer_disable(timer, NOVATEK_TIMER_CLKEVT_CHANNEL);
}

static int novatek_timer_set_periodic(struct clock_event_device *ce)
{
	struct novatek_timer *timer;

	timer = container_of(ce, struct novatek_timer, ce);
	return novatek_timer_program(timer, NOVATEK_TIMER_CLKEVT_CHANNEL,
				     timer->period,
				     NOVATEK_TIMER_MODE_PERIODIC);
}

static int novatek_timer_next_event(unsigned long cycles,
				    struct clock_event_device *ce)
{
	struct novatek_timer *timer;

	timer = container_of(ce, struct novatek_timer, ce);
	return novatek_timer_program(timer, NOVATEK_TIMER_CLKEVT_CHANNEL,
				     cycles, NOVATEK_TIMER_MODE_ONESHOT);
}

static irqreturn_t novatek_timer_interrupt(int irq, void *data)
{
	struct novatek_timer *timer = data;
	ulong flags;
	u32 status;

	raw_spin_lock_irqsave(&timer->lock, flags);
	status = readl(timer->base + NOVATEK_TIMER_STATUS);
	status &= readl(timer->base + NOVATEK_TIMER_INT_ENABLE);
	status &= NOVATEK_TIMER_CHANNEL_MASK(NOVATEK_TIMER_CLKEVT_CHANNEL);
	if (!status) {
		raw_spin_unlock_irqrestore(&timer->lock, flags);
		return IRQ_NONE;
	}

	writel(status, timer->base + NOVATEK_TIMER_STATUS);
	raw_spin_unlock_irqrestore(&timer->lock, flags);
	timer->ce.event_handler(&timer->ce);

	return IRQ_HANDLED;
}

static u64 novatek_timer_read(struct clocksource *cs)
{
	struct novatek_timer *timer;
	void __iomem *counter;

	timer = container_of(cs, struct novatek_timer, cs);
	counter = timer->base +
		  NOVATEK_TIMER_COUNTER(NOVATEK_TIMER_CLKSRC_CHANNEL);
	return readl_relaxed(counter);
}

static void novatek_timer_quiesce(void *data)
{
	struct novatek_timer *timer = data;
	int ret;

	ret = novatek_timer_disable(timer, NOVATEK_TIMER_CLKEVT_CHANNEL);
	if (ret)
		dev_err(timer->dev, "Failed to stop clockevent: %d\n", ret);

	ret = novatek_timer_disable(timer, NOVATEK_TIMER_CLKSRC_CHANNEL);
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
	ret = novatek_timer_program(timer, NOVATEK_TIMER_CLKSRC_CHANNEL,
				    NOVATEK_TIMER_MAX_DELTA,
				    NOVATEK_TIMER_MODE_PERIODIC);
	if (ret)
		dev_err(timer->dev, "Failed to resume clocksource: %d\n", ret);
}

static int novatek_timer_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct reset_control *reset;
	struct novatek_timer *timer;
	struct resource *res;
	struct clk *clk;
	ulong rate;
	int irq;
	int ret;

	timer = devm_kzalloc(dev, sizeof(*timer), GFP_KERNEL);
	if (!timer)
		return -ENOMEM;
	timer->dev = dev;
	raw_spin_lock_init(&timer->lock);

	timer->base = devm_platform_get_and_ioremap_resource(pdev, 0, &res);
	if (IS_ERR(timer->base))
		return PTR_ERR(timer->base);
	if (resource_size(res) < NOVATEK_TIMER_REG_SIZE)
		return dev_err_probe(dev, -EINVAL, "Invalid register window\n");

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	clk = devm_clk_get_enabled(dev, NULL);
	if (IS_ERR(clk))
		return dev_err_probe(dev, PTR_ERR(clk),
				     "Failed to enable clock\n");
	rate = clk_get_rate(clk);
	if (!rate)
		return dev_err_probe(dev, -EINVAL, "Invalid clock rate\n");
	timer->period = DIV_ROUND_CLOSEST(rate, HZ);

	reset = devm_reset_control_get_exclusive_deasserted(dev, NULL);
	if (IS_ERR(reset))
		return dev_err_probe(dev, PTR_ERR(reset),
				     "Failed to get reset\n");
	ret = reset_control_reset(reset);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to reset timer\n");

	novatek_timer_hw_init(timer);
	ret = devm_request_irq(dev, irq, novatek_timer_interrupt, IRQF_TIMER,
			       dev_name(dev), timer);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to request interrupt\n");

	ret = devm_add_action_or_reset(dev, novatek_timer_quiesce, timer);
	if (ret)
		return ret;

	ret = novatek_timer_program(timer, NOVATEK_TIMER_CLKSRC_CHANNEL,
				    NOVATEK_TIMER_MAX_DELTA,
				    NOVATEK_TIMER_MODE_PERIODIC);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to start clocksource\n");

	timer->cs.name = "novatek-timer";
	timer->cs.rating = 200;
	timer->cs.read = novatek_timer_read;
	timer->cs.mask = CLOCKSOURCE_MASK(32);
	timer->cs.flags = CLOCK_SOURCE_IS_CONTINUOUS;
	timer->cs.suspend = novatek_timer_suspend;
	timer->cs.resume = novatek_timer_resume;
	ret = devm_clocksource_register_hz(dev, &timer->cs, rate);
	if (ret)
		return dev_err_probe(dev, ret,
				     "Failed to register clocksource\n");

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
	clockevents_config_and_register(&timer->ce, rate,
					NOVATEK_TIMER_MIN_DELTA,
					NOVATEK_TIMER_MAX_DELTA);

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
builtin_platform_driver(novatek_timer_driver);

MODULE_DESCRIPTION("Novatek NA51089 timer driver");
MODULE_LICENSE("GPL");
