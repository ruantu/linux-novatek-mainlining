// SPDX-License-Identifier: GPL-2.0-only

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm_wakeirq.h>
#include <linux/pm_wakeup.h>
#include <linux/property.h>
#include <linux/reset.h>
#include <linux/rtc.h>

#define NOVATEK_DRTC_TIMER			0x00
#define NOVATEK_DRTC_TIMER_SECOND_MASK		GENMASK(5, 0)
#define NOVATEK_DRTC_TIMER_MINUTE_MASK		GENMASK(13, 8)
#define NOVATEK_DRTC_TIMER_HOUR_MASK		GENMASK(20, 16)
#define NOVATEK_DRTC_TIMER_TIME_MASK				\
	(NOVATEK_DRTC_TIMER_SECOND_MASK |			\
	 NOVATEK_DRTC_TIMER_MINUTE_MASK |			\
	 NOVATEK_DRTC_TIMER_HOUR_MASK)

#define NOVATEK_DRTC_DAY			0x04
#define NOVATEK_DRTC_DAY_VALUE_MASK		GENMASK(16, 0)

#define NOVATEK_DRTC_ALARM_TIMER		0x08

#define NOVATEK_DRTC_ALARM_DAY			0x0c

#define NOVATEK_DRTC_CTRL			0x10
#define NOVATEK_DRTC_CTRL_ALARM_INT_MASK	BIT(0)
#define NOVATEK_DRTC_CTRL_ALARM_INT_DISABLE	0
#define NOVATEK_DRTC_CTRL_ALARM_INT_ENABLE	1
#define NOVATEK_DRTC_CTRL_TIME_ENABLE_MASK	BIT(4)
#define NOVATEK_DRTC_CTRL_TIME_DISABLE		0
#define NOVATEK_DRTC_CTRL_TIME_ENABLE		1
#define NOVATEK_DRTC_CTRL_ALARM_PERIOD_MASK	GENMASK(11, 8)
#define NOVATEK_DRTC_CTRL_ALARM_PERIOD_ONCE	0x8

#define NOVATEK_DRTC_STATUS			0x14
#define NOVATEK_DRTC_STATUS_ALARM_MASK		BIT(0)

#define NOVATEK_DRTC_SECONDS_PER_MINUTE		60
#define NOVATEK_DRTC_SECONDS_PER_HOUR		3600
#define NOVATEK_DRTC_SECONDS_PER_DAY		86400
#define NOVATEK_DRTC_TIMESTAMP_END		9115631999LL
#define NOVATEK_DRTC_TM_YEAR_MAX		358
#define NOVATEK_DRTC_READ_RETRIES		5
#define NOVATEK_DRTC_POLL_DELAY_US		10
#define NOVATEK_DRTC_POLL_TIMEOUT_US		10000

struct novatek_drtc {
	void __iomem *base;
	struct rtc_device *rtc;
};

static int novatek_drtc_valid_time(struct rtc_time *tm)
{
	if ((uint)tm->tm_year > NOVATEK_DRTC_TM_YEAR_MAX ||
	    (uint)tm->tm_mon >= 12 || tm->tm_mday < 1 ||
	    tm->tm_mday > rtc_month_days(tm->tm_mon, tm->tm_year + 1900) ||
	    (uint)tm->tm_hour >= 24 ||
	    (uint)tm->tm_min >= NOVATEK_DRTC_SECONDS_PER_MINUTE ||
	    (uint)tm->tm_sec >= NOVATEK_DRTC_SECONDS_PER_MINUTE)
		return -EINVAL;

	return 0;
}

static int novatek_drtc_set_time_enabled(struct novatek_drtc *drtc,
					 bool enabled)
{
	u32 value;
	uint state;

	state = enabled ? NOVATEK_DRTC_CTRL_TIME_ENABLE :
		NOVATEK_DRTC_CTRL_TIME_DISABLE;
	value = readl(drtc->base + NOVATEK_DRTC_CTRL);
	value &= ~NOVATEK_DRTC_CTRL_TIME_ENABLE_MASK;
	value |= FIELD_PREP(NOVATEK_DRTC_CTRL_TIME_ENABLE_MASK, state);
	writel(value, drtc->base + NOVATEK_DRTC_CTRL);

	return readl_poll_timeout(drtc->base + NOVATEK_DRTC_CTRL, value,
				  FIELD_GET(NOVATEK_DRTC_CTRL_TIME_ENABLE_MASK,
					    value) == state,
				  NOVATEK_DRTC_POLL_DELAY_US,
				  NOVATEK_DRTC_POLL_TIMEOUT_US);
}

static int novatek_drtc_decode_time(u32 timer, u32 day,
				    struct rtc_time *tm)
{
	uint second;
	uint minute;
	uint hour;
	time64_t time;

	second = FIELD_GET(NOVATEK_DRTC_TIMER_SECOND_MASK, timer);
	minute = FIELD_GET(NOVATEK_DRTC_TIMER_MINUTE_MASK, timer);
	hour = FIELD_GET(NOVATEK_DRTC_TIMER_HOUR_MASK, timer);
	if (second >= NOVATEK_DRTC_SECONDS_PER_MINUTE ||
	    minute >= NOVATEK_DRTC_SECONDS_PER_MINUTE || hour >= 24)
		return -EINVAL;

	time = RTC_TIMESTAMP_BEGIN_1900 +
	       (time64_t)FIELD_GET(NOVATEK_DRTC_DAY_VALUE_MASK, day) *
	       NOVATEK_DRTC_SECONDS_PER_DAY +
	       hour * NOVATEK_DRTC_SECONDS_PER_HOUR +
	       minute * NOVATEK_DRTC_SECONDS_PER_MINUTE + second;
	rtc_time64_to_tm(time, tm);

	return 0;
}

static int novatek_drtc_encode_time(struct rtc_time *tm, u32 *timer,
				    u32 *day)
{
	time64_t time;
	u64 elapsed;
	u64 days;
	u32 seconds;
	uint hour;
	uint minute;
	int ret;

	ret = novatek_drtc_valid_time(tm);
	if (ret)
		return ret;

	time = rtc_tm_to_time64(tm);
	if (time < RTC_TIMESTAMP_BEGIN_1900 ||
	    time > NOVATEK_DRTC_TIMESTAMP_END)
		return -ERANGE;

	elapsed = time - RTC_TIMESTAMP_BEGIN_1900;
	days = div_u64_rem(elapsed, NOVATEK_DRTC_SECONDS_PER_DAY, &seconds);
	hour = seconds / NOVATEK_DRTC_SECONDS_PER_HOUR;
	seconds %= NOVATEK_DRTC_SECONDS_PER_HOUR;
	minute = seconds / NOVATEK_DRTC_SECONDS_PER_MINUTE;
	seconds %= NOVATEK_DRTC_SECONDS_PER_MINUTE;

	*timer = FIELD_PREP(NOVATEK_DRTC_TIMER_SECOND_MASK, seconds) |
		 FIELD_PREP(NOVATEK_DRTC_TIMER_MINUTE_MASK, minute) |
		 FIELD_PREP(NOVATEK_DRTC_TIMER_HOUR_MASK, hour);
	*day = FIELD_PREP(NOVATEK_DRTC_DAY_VALUE_MASK, (u32)days);

	return 0;
}

static int novatek_drtc_read_stable(struct novatek_drtc *drtc, u32 *timer,
				    u32 *day)
{
	u32 first_timer;
	u32 second_timer;
	u32 first_day;
	u32 second_day;
	uint i;

	for (i = 0; i < NOVATEK_DRTC_READ_RETRIES; i++) {
		first_day = readl(drtc->base + NOVATEK_DRTC_DAY);
		first_timer = readl(drtc->base + NOVATEK_DRTC_TIMER);
		second_timer = readl(drtc->base + NOVATEK_DRTC_TIMER);
		second_day = readl(drtc->base + NOVATEK_DRTC_DAY);
		if ((first_day & NOVATEK_DRTC_DAY_VALUE_MASK) ==
		    (second_day & NOVATEK_DRTC_DAY_VALUE_MASK) &&
		    (first_timer & NOVATEK_DRTC_TIMER_TIME_MASK) ==
		    (second_timer & NOVATEK_DRTC_TIMER_TIME_MASK)) {
			*timer = second_timer;
			*day = second_day;
			return 0;
		}
	}

	return -EIO;
}

static int novatek_drtc_read_time(struct device *dev, struct rtc_time *tm)
{
	struct novatek_drtc *drtc = dev_get_drvdata(dev);
	u32 timer;
	u32 day;
	int ret;

	if (!(readl(drtc->base + NOVATEK_DRTC_CTRL) &
	      NOVATEK_DRTC_CTRL_TIME_ENABLE_MASK))
		return -EINVAL;

	ret = novatek_drtc_read_stable(drtc, &timer, &day);
	if (ret)
		return ret;

	return novatek_drtc_decode_time(timer, day, tm);
}

static int novatek_drtc_set_time(struct device *dev, struct rtc_time *tm)
{
	struct novatek_drtc *drtc = dev_get_drvdata(dev);
	u32 timer;
	u32 day;
	int ret;

	ret = novatek_drtc_encode_time(tm, &timer, &day);
	if (ret)
		return ret;

	ret = novatek_drtc_set_time_enabled(drtc, false);
	if (ret) {
		novatek_drtc_set_time_enabled(drtc, true);
		return ret;
	}

	writel(timer, drtc->base + NOVATEK_DRTC_TIMER);
	writel(day, drtc->base + NOVATEK_DRTC_DAY);

	return novatek_drtc_set_time_enabled(drtc, true);
}

static int novatek_drtc_read_alarm(struct device *dev,
				   struct rtc_wkalrm *alarm)
{
	struct novatek_drtc *drtc = dev_get_drvdata(dev);
	u32 timer;
	u32 day;
	u32 ctrl;
	u32 status;
	int ret;

	timer = readl(drtc->base + NOVATEK_DRTC_ALARM_TIMER);
	day = readl(drtc->base + NOVATEK_DRTC_ALARM_DAY);
	ret = novatek_drtc_decode_time(timer, day, &alarm->time);
	if (ret)
		return ret;

	ctrl = readl(drtc->base + NOVATEK_DRTC_CTRL);
	status = readl(drtc->base + NOVATEK_DRTC_STATUS);
	alarm->enabled = FIELD_GET(NOVATEK_DRTC_CTRL_ALARM_INT_MASK, ctrl) ==
		NOVATEK_DRTC_CTRL_ALARM_INT_ENABLE;
	alarm->pending = !!(status & NOVATEK_DRTC_STATUS_ALARM_MASK);

	return 0;
}

static int novatek_drtc_set_alarm(struct device *dev,
				  struct rtc_wkalrm *alarm)
{
	struct novatek_drtc *drtc = dev_get_drvdata(dev);
	u32 timer;
	u32 day;
	u32 ctrl;
	uint enabled;
	int ret;

	ret = novatek_drtc_encode_time(&alarm->time, &timer, &day);
	if (ret)
		return ret;

	ctrl = readl(drtc->base + NOVATEK_DRTC_CTRL);
	ctrl &= ~NOVATEK_DRTC_CTRL_ALARM_INT_MASK;
	ctrl |= FIELD_PREP(NOVATEK_DRTC_CTRL_ALARM_INT_MASK,
			   NOVATEK_DRTC_CTRL_ALARM_INT_DISABLE);
	writel(ctrl, drtc->base + NOVATEK_DRTC_CTRL);

	writel(timer, drtc->base + NOVATEK_DRTC_ALARM_TIMER);
	writel(day, drtc->base + NOVATEK_DRTC_ALARM_DAY);
	writel(NOVATEK_DRTC_STATUS_ALARM_MASK,
	       drtc->base + NOVATEK_DRTC_STATUS);

	enabled = alarm->enabled ? NOVATEK_DRTC_CTRL_ALARM_INT_ENABLE :
		NOVATEK_DRTC_CTRL_ALARM_INT_DISABLE;
	ctrl &= ~(NOVATEK_DRTC_CTRL_ALARM_PERIOD_MASK |
		  NOVATEK_DRTC_CTRL_ALARM_INT_MASK);
	ctrl |= FIELD_PREP(NOVATEK_DRTC_CTRL_ALARM_PERIOD_MASK,
			   NOVATEK_DRTC_CTRL_ALARM_PERIOD_ONCE) |
		FIELD_PREP(NOVATEK_DRTC_CTRL_ALARM_INT_MASK, enabled);
	writel(ctrl, drtc->base + NOVATEK_DRTC_CTRL);

	return 0;
}

static int novatek_drtc_alarm_irq_enable(struct device *dev,
					 unsigned int enabled)
{
	struct novatek_drtc *drtc = dev_get_drvdata(dev);
	u32 ctrl;
	uint value;

	if (enabled)
		writel(NOVATEK_DRTC_STATUS_ALARM_MASK,
		       drtc->base + NOVATEK_DRTC_STATUS);

	value = enabled ? NOVATEK_DRTC_CTRL_ALARM_INT_ENABLE :
		NOVATEK_DRTC_CTRL_ALARM_INT_DISABLE;
	ctrl = readl(drtc->base + NOVATEK_DRTC_CTRL);
	ctrl &= ~(NOVATEK_DRTC_CTRL_ALARM_PERIOD_MASK |
		  NOVATEK_DRTC_CTRL_ALARM_INT_MASK);
	ctrl |= FIELD_PREP(NOVATEK_DRTC_CTRL_ALARM_PERIOD_MASK,
			   NOVATEK_DRTC_CTRL_ALARM_PERIOD_ONCE) |
		FIELD_PREP(NOVATEK_DRTC_CTRL_ALARM_INT_MASK, value);
	writel(ctrl, drtc->base + NOVATEK_DRTC_CTRL);

	return 0;
}

static irqreturn_t novatek_drtc_irq(int irq, void *data)
{
	struct novatek_drtc *drtc = data;
	u32 status;

	status = readl(drtc->base + NOVATEK_DRTC_STATUS);
	if (!(status & NOVATEK_DRTC_STATUS_ALARM_MASK))
		return IRQ_NONE;

	writel(NOVATEK_DRTC_STATUS_ALARM_MASK,
	       drtc->base + NOVATEK_DRTC_STATUS);
	rtc_update_irq(drtc->rtc, 1, RTC_IRQF | RTC_AF);

	return IRQ_HANDLED;
}

static const struct rtc_class_ops novatek_drtc_ops = {
	.read_time = novatek_drtc_read_time,
	.set_time = novatek_drtc_set_time,
	.read_alarm = novatek_drtc_read_alarm,
	.set_alarm = novatek_drtc_set_alarm,
	.alarm_irq_enable = novatek_drtc_alarm_irq_enable,
};

static int novatek_drtc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct reset_control *reset;
	struct novatek_drtc *drtc;
	struct clk *clk;
	u32 ctrl;
	int irq;
	int ret;

	drtc = devm_kzalloc(dev, sizeof(*drtc), GFP_KERNEL);
	if (!drtc)
		return -ENOMEM;

	drtc->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(drtc->base))
		return PTR_ERR(drtc->base);

	/* Do not pulse reset because it would erase firmware's RTC state. */
	reset = devm_reset_control_get_exclusive(dev, NULL);
	if (IS_ERR(reset))
		return dev_err_probe(dev, PTR_ERR(reset),
				     "failed to get DRTC reset\n");

	clk = devm_clk_get_enabled(dev, NULL);
	if (IS_ERR(clk))
		return dev_err_probe(dev, PTR_ERR(clk),
				     "failed to enable DRTC clock\n");

	ret = reset_control_deassert(reset);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to deassert DRTC reset\n");

	ctrl = readl(drtc->base + NOVATEK_DRTC_CTRL);
	if (FIELD_GET(NOVATEK_DRTC_CTRL_ALARM_PERIOD_MASK, ctrl) !=
	    NOVATEK_DRTC_CTRL_ALARM_PERIOD_ONCE)
		ctrl &= ~NOVATEK_DRTC_CTRL_ALARM_INT_MASK;
	ctrl &= ~NOVATEK_DRTC_CTRL_ALARM_PERIOD_MASK;
	ctrl |= FIELD_PREP(NOVATEK_DRTC_CTRL_ALARM_PERIOD_MASK,
			   NOVATEK_DRTC_CTRL_ALARM_PERIOD_ONCE);
	writel(ctrl, drtc->base + NOVATEK_DRTC_CTRL);

	drtc->rtc = devm_rtc_allocate_device(dev);
	if (IS_ERR(drtc->rtc))
		return PTR_ERR(drtc->rtc);

	drtc->rtc->ops = &novatek_drtc_ops;
	drtc->rtc->range_min = RTC_TIMESTAMP_BEGIN_1900;
	drtc->rtc->range_max = NOVATEK_DRTC_TIMESTAMP_END;
	clear_bit(RTC_FEATURE_UPDATE_INTERRUPT, drtc->rtc->features);
	platform_set_drvdata(pdev, drtc);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	ret = devm_request_irq(dev, irq, novatek_drtc_irq, 0,
			       dev_name(dev), drtc);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to request DRTC IRQ\n");

	if (device_property_read_bool(dev, "wakeup-source")) {
		ret = devm_device_init_wakeup(dev);
		if (ret)
			return ret;

		ret = devm_pm_set_wake_irq(dev, irq);
		if (ret)
			return dev_err_probe(dev, ret,
					     "failed to configure wake IRQ\n");
	}

	return devm_rtc_register_device(drtc->rtc);
}

static const struct of_device_id novatek_drtc_of_match[] = {
	{ .compatible = "novatek,na51089-drtc" },
	{ }
};
MODULE_DEVICE_TABLE(of, novatek_drtc_of_match);

static struct platform_driver novatek_drtc_driver = {
	.probe = novatek_drtc_probe,
	.driver = {
		.name = "novatek-drtc",
		.of_match_table = novatek_drtc_of_match,
	},
};
module_platform_driver(novatek_drtc_driver);

MODULE_DESCRIPTION("Novatek digital real-time clock driver");
MODULE_LICENSE("GPL");
