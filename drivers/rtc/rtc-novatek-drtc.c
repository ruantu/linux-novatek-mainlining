// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2026 Yu-Tung Chang <mtwget@gmail.com>
 */

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

#define DRTC_TIMER_REG				0x00
#define DRTC_DAY_REG				0x04
#define DRTC_ALARM_TIMER_REG			0x08
#define DRTC_ALARM_DAY_REG			0x0c
#define DRTC_CTRL_REG				0x10
#define DRTC_STATUS_REG				0x14

/* DRTC_TIMER bit fields */
#define DRTC_TIMER_SECOND			GENMASK(5, 0)
#define DRTC_TIMER_MINUTE			GENMASK(13, 8)
#define DRTC_TIMER_HOUR				GENMASK(20, 16)
#define DRTC_TIMER_TIME				\
	(DRTC_TIMER_SECOND |			\
	 DRTC_TIMER_MINUTE |			\
	 DRTC_TIMER_HOUR)

/* DRTC_DAY bit fields */
#define DRTC_DAY_VALUE				GENMASK(16, 0)

/* DRTC_CTRL bit fields */
#define DRTC_CTRL_ALARM_INT			BIT(0)
#define DRTC_CTRL_TIME_ENABLE			BIT(4)
#define DRTC_CTRL_ALARM_PERIOD			GENMASK(11, 8)
#define DRTC_CTRL_ALARM_PERIOD_ONCE		0x8

/* DRTC_STATUS bit fields */
#define DRTC_STATUS_ALARM			BIT(0)

#define DRTC_SECONDS_PER_MINUTE			60
#define DRTC_SECONDS_PER_HOUR			3600
#define DRTC_SECONDS_PER_DAY			86400
#define DRTC_TIMESTAMP_END			9115631999LL
#define DRTC_TM_YEAR_MAX			358

#define DRTC_READ_RETRIES			5

#define DRTC_POLL_DELAY_US			10
#define DRTC_POLL_TIMEOUT_US			10000

struct novatek_drtc {
	void __iomem *base;
	struct rtc_device *rtc;
};

static int novatek_drtc_valid_time(struct rtc_time *tm)
{
	if ((uint)tm->tm_year > DRTC_TM_YEAR_MAX ||
	    (uint)tm->tm_mon >= 12 || tm->tm_mday < 1 ||
	    tm->tm_mday > rtc_month_days(tm->tm_mon, tm->tm_year + 1900) ||
	    (uint)tm->tm_hour >= 24 ||
	    (uint)tm->tm_min >= DRTC_SECONDS_PER_MINUTE ||
	    (uint)tm->tm_sec >= DRTC_SECONDS_PER_MINUTE)
		return -EINVAL;

	return 0;
}

static int novatek_drtc_set_time_enabled(struct novatek_drtc *drtc, bool enabled)
{
	u32 value;
	uint state;

	state = enabled ? DRTC_CTRL_TIME_ENABLE : 0;
	value = readl(drtc->base + DRTC_CTRL_REG);
	value &= ~DRTC_CTRL_TIME_ENABLE;
	value |= state;
	writel(value, drtc->base + DRTC_CTRL_REG);

	return readl_poll_timeout(drtc->base + DRTC_CTRL_REG,
				  value, (value & DRTC_CTRL_TIME_ENABLE) == state,
				  DRTC_POLL_DELAY_US, DRTC_POLL_TIMEOUT_US);
}

static int novatek_drtc_decode_time(u32 timer, u32 day, struct rtc_time *tm)
{
	uint second;
	uint minute;
	uint hour;
	time64_t time;

	second = FIELD_GET(DRTC_TIMER_SECOND, timer);
	minute = FIELD_GET(DRTC_TIMER_MINUTE, timer);
	hour = FIELD_GET(DRTC_TIMER_HOUR, timer);
	if (second >= DRTC_SECONDS_PER_MINUTE || minute >= DRTC_SECONDS_PER_MINUTE || hour >= 24)
		return -EINVAL;

	time = RTC_TIMESTAMP_BEGIN_1900 +
	       (time64_t)FIELD_GET(DRTC_DAY_VALUE, day) * DRTC_SECONDS_PER_DAY +
	       hour * DRTC_SECONDS_PER_HOUR +
	       minute * DRTC_SECONDS_PER_MINUTE + second;
	rtc_time64_to_tm(time, tm);

	return 0;
}

static int novatek_drtc_encode_time(struct rtc_time *tm, u32 *timer, u32 *day)
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
	if (time < RTC_TIMESTAMP_BEGIN_1900 || time > DRTC_TIMESTAMP_END)
		return -ERANGE;

	elapsed = time - RTC_TIMESTAMP_BEGIN_1900;
	days = div_u64_rem(elapsed, DRTC_SECONDS_PER_DAY, &seconds);
	hour = seconds / DRTC_SECONDS_PER_HOUR;
	seconds %= DRTC_SECONDS_PER_HOUR;
	minute = seconds / DRTC_SECONDS_PER_MINUTE;
	seconds %= DRTC_SECONDS_PER_MINUTE;

	*timer = FIELD_PREP(DRTC_TIMER_SECOND, seconds) |
		 FIELD_PREP(DRTC_TIMER_MINUTE, minute) |
		 FIELD_PREP(DRTC_TIMER_HOUR, hour);
	*day = FIELD_PREP(DRTC_DAY_VALUE, (u32)days);

	return 0;
}

static int novatek_drtc_read_stable(struct novatek_drtc *drtc, u32 *timer, u32 *day)
{
	u32 first_timer;
	u32 second_timer;
	u32 first_day;
	u32 second_day;
	uint i;

	for (i = 0; i < DRTC_READ_RETRIES; i++) {
		first_day = readl(drtc->base + DRTC_DAY_REG);
		first_timer = readl(drtc->base + DRTC_TIMER_REG);
		second_timer = readl(drtc->base + DRTC_TIMER_REG);
		second_day = readl(drtc->base + DRTC_DAY_REG);
		if ((first_day & DRTC_DAY_VALUE) == (second_day & DRTC_DAY_VALUE) &&
		    (first_timer & DRTC_TIMER_TIME) == (second_timer & DRTC_TIMER_TIME)) {
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

	if (!(readl(drtc->base + DRTC_CTRL_REG) & DRTC_CTRL_TIME_ENABLE))
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
	if (ret)
		return ret;

	writel(timer, drtc->base + DRTC_TIMER_REG);
	writel(day, drtc->base + DRTC_DAY_REG);

	return novatek_drtc_set_time_enabled(drtc, true);
}

static int novatek_drtc_read_alarm(struct device *dev, struct rtc_wkalrm *alarm)
{
	struct novatek_drtc *drtc = dev_get_drvdata(dev);
	u32 timer;
	u32 day;
	u32 ctrl;
	u32 status;
	int ret;

	timer = readl(drtc->base + DRTC_ALARM_TIMER_REG);
	day = readl(drtc->base + DRTC_ALARM_DAY_REG);
	ret = novatek_drtc_decode_time(timer, day, &alarm->time);
	if (ret)
		return ret;

	ctrl = readl(drtc->base + DRTC_CTRL_REG);
	status = readl(drtc->base + DRTC_STATUS_REG);
	alarm->enabled = !!(ctrl & DRTC_CTRL_ALARM_INT);
	alarm->pending = !!(status & DRTC_STATUS_ALARM);

	return 0;
}

static int novatek_drtc_set_alarm(struct device *dev, struct rtc_wkalrm *alarm)
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

	ctrl = readl(drtc->base + DRTC_CTRL_REG);
	ctrl &= ~DRTC_CTRL_ALARM_INT;
	writel(ctrl, drtc->base + DRTC_CTRL_REG);

	writel(timer, drtc->base + DRTC_ALARM_TIMER_REG);
	writel(day, drtc->base + DRTC_ALARM_DAY_REG);

	writel(DRTC_STATUS_ALARM, drtc->base + DRTC_STATUS_REG);

	enabled = alarm->enabled ? DRTC_CTRL_ALARM_INT : 0;
	ctrl &= ~(DRTC_CTRL_ALARM_PERIOD | DRTC_CTRL_ALARM_INT);
	ctrl |= FIELD_PREP(DRTC_CTRL_ALARM_PERIOD, DRTC_CTRL_ALARM_PERIOD_ONCE) | enabled;
	writel(ctrl, drtc->base + DRTC_CTRL_REG);

	return 0;
}

static int novatek_drtc_alarm_irq_enable(struct device *dev, unsigned int enabled)
{
	struct novatek_drtc *drtc = dev_get_drvdata(dev);
	u32 ctrl;
	uint value;

	if (enabled)
		writel(DRTC_STATUS_ALARM, drtc->base + DRTC_STATUS_REG);

	value = enabled ? DRTC_CTRL_ALARM_INT : 0;
	ctrl = readl(drtc->base + DRTC_CTRL_REG);
	ctrl &= ~(DRTC_CTRL_ALARM_PERIOD | DRTC_CTRL_ALARM_INT);
	ctrl |= FIELD_PREP(DRTC_CTRL_ALARM_PERIOD, DRTC_CTRL_ALARM_PERIOD_ONCE) | value;
	writel(ctrl, drtc->base + DRTC_CTRL_REG);

	return 0;
}

static irqreturn_t novatek_drtc_irq(int irq, void *data)
{
	struct novatek_drtc *drtc = data;
	u32 status;

	status = readl(drtc->base + DRTC_STATUS_REG);
	if (!(status & DRTC_STATUS_ALARM))
		return IRQ_NONE;
	writel(DRTC_STATUS_ALARM, drtc->base + DRTC_STATUS_REG);

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

	reset = devm_reset_control_get_exclusive(dev, NULL);
	if (IS_ERR(reset))
		return dev_err_probe(dev, PTR_ERR(reset), "failed to get DRTC reset\n");

	clk = devm_clk_get_enabled(dev, NULL);
	if (IS_ERR(clk))
		return dev_err_probe(dev, PTR_ERR(clk), "failed to enable DRTC clock\n");

	ret = reset_control_deassert(reset);
	if (ret)
		return dev_err_probe(dev, ret, "failed to deassert DRTC reset\n");

	ctrl = readl(drtc->base + DRTC_CTRL_REG);
	if (FIELD_GET(DRTC_CTRL_ALARM_PERIOD, ctrl) != DRTC_CTRL_ALARM_PERIOD_ONCE)
		ctrl &= ~DRTC_CTRL_ALARM_INT;
	ctrl &= ~DRTC_CTRL_ALARM_PERIOD;
	ctrl |= FIELD_PREP(DRTC_CTRL_ALARM_PERIOD, DRTC_CTRL_ALARM_PERIOD_ONCE);
	writel(ctrl, drtc->base + DRTC_CTRL_REG);

	drtc->rtc = devm_rtc_allocate_device(dev);
	if (IS_ERR(drtc->rtc))
		return PTR_ERR(drtc->rtc);

	drtc->rtc->ops = &novatek_drtc_ops;
	drtc->rtc->range_min = RTC_TIMESTAMP_BEGIN_1900;
	drtc->rtc->range_max = DRTC_TIMESTAMP_END;
	clear_bit(RTC_FEATURE_UPDATE_INTERRUPT, drtc->rtc->features);
	platform_set_drvdata(pdev, drtc);

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	ret = devm_request_irq(dev, irq, novatek_drtc_irq, 0, dev_name(dev), drtc);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request DRTC IRQ\n");

	if (device_property_read_bool(dev, "wakeup-source")) {
		ret = devm_device_init_wakeup(dev);
		if (ret)
			return ret;

		ret = devm_pm_set_wake_irq(dev, irq);
		if (ret)
			return dev_err_probe(dev, ret, "failed to configure wake IRQ\n");
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

MODULE_AUTHOR("Yu-Tung Chang <mtwget@gmail.com>");
MODULE_DESCRIPTION("Novatek digital RTC driver");
MODULE_LICENSE("GPL");
