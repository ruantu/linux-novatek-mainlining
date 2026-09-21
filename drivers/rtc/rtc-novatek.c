// SPDX-License-Identifier: GPL-2.0-only

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/math64.h>
#include <linux/mfd/novatek-rtcsys.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm_wakeirq.h>
#include <linux/pm_wakeup.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/rtc.h>
#include <linux/time64.h>

#define NOVATEK_RTC_SECONDS_PER_MINUTE	60
#define NOVATEK_RTC_SECONDS_PER_HOUR	3600
#define NOVATEK_RTC_SECONDS_PER_DAY	86400
#define NOVATEK_RTC_PWRALARM_MAX_DAYS	31
#define NOVATEK_RTC_ALARM_OFFSET_MAX					\
	(NOVATEK_RTC_PWRALARM_MAX_DAYS * NOVATEK_RTC_SECONDS_PER_DAY)
#define NOVATEK_RTC_TM_YEAR_MAX		179
#define NOVATEK_RTC_READ_RETRIES	5
#define NOVATEK_RTC_POLL_DELAY_US	100
#define NOVATEK_RTC_POLL_TIMEOUT_US	1000000
#define NOVATEK_RTC_CSET_SETTLE_US	8000

struct novatek_rtc {
	struct novatek_rtcsys *rtcsys;
	struct rtc_device *rtc;
	int irq;
	bool irq_enabled;
};

static int novatek_rtc_valid_time(struct rtc_time *tm)
{
	if ((uint)tm->tm_year > NOVATEK_RTC_TM_YEAR_MAX ||
	    (uint)tm->tm_mon >= 12 || tm->tm_mday < 1 ||
	    tm->tm_mday > rtc_month_days(tm->tm_mon, tm->tm_year + 1900) ||
	    (uint)tm->tm_hour >= 24 ||
	    (uint)tm->tm_min >= NOVATEK_RTC_SECONDS_PER_MINUTE ||
	    (uint)tm->tm_sec >= NOVATEK_RTC_SECONDS_PER_MINUTE)
		return -EINVAL;

	return 0;
}

static int novatek_rtc_wait_idle(struct novatek_rtc *rtc)
{
	struct regmap *regmap = rtc->rtcsys->regmap;
	u32 value;
	int ret;

	ret = regmap_read_poll_timeout(regmap, NOVATEK_RTC_STATUS, value,
				       !(value & NOVATEK_RTC_STATUS_SRST_MASK),
				       NOVATEK_RTC_POLL_DELAY_US,
				       NOVATEK_RTC_POLL_TIMEOUT_US);
	if (ret)
		return ret;

	return regmap_read_poll_timeout(regmap, NOVATEK_RTC_CTRL, value,
					!(value & NOVATEK_RTC_CTRL_CSET_MASK),
					NOVATEK_RTC_POLL_DELAY_US,
					NOVATEK_RTC_POLL_TIMEOUT_US);
}

static int novatek_rtc_prepare_update(struct novatek_rtc *rtc, u32 select)
{
	int ret;

	ret = novatek_rtc_wait_idle(rtc);
	if (ret)
		return ret;

	return regmap_update_bits(rtc->rtcsys->regmap, NOVATEK_RTC_CTRL,
				  NOVATEK_RTC_CTRL_SRST_MASK |
				  NOVATEK_RTC_CTRL_CSET_MASK |
				  NOVATEK_RTC_CTRL_TIME_SEL_MASK |
				  NOVATEK_RTC_CTRL_DAY_SEL_MASK |
				  NOVATEK_RTC_CTRL_KEY_SEL_MASK |
				  NOVATEK_RTC_CTRL_PWRALARMTIME_SEL_MASK |
				  NOVATEK_RTC_CTRL_PWRALARMDAY_SEL_MASK,
				  select);
}

static int novatek_rtc_commit_update(struct novatek_rtc *rtc)
{
	struct regmap *regmap = rtc->rtcsys->regmap;
	u32 value;
	int ret;

	ret = regmap_write(regmap, NOVATEK_RTC_STATUS,
			   NOVATEK_RTC_STATUS_CSET_MASK);
	if (ret)
		return ret;

	ret = regmap_set_bits(regmap, NOVATEK_RTC_CTRL,
			      NOVATEK_RTC_CTRL_CSET_MASK);
	if (ret)
		return ret;

	ret = regmap_read_poll_timeout(regmap, NOVATEK_RTC_STATUS, value,
				       value & NOVATEK_RTC_STATUS_CSET_MASK,
				       NOVATEK_RTC_POLL_DELAY_US,
				       NOVATEK_RTC_POLL_TIMEOUT_US);
	if (ret)
		return ret;

	ret = regmap_write(regmap, NOVATEK_RTC_STATUS,
			   NOVATEK_RTC_STATUS_CSET_MASK);
	if (ret)
		return ret;

	usleep_range(NOVATEK_RTC_CSET_SETTLE_US,
		     NOVATEK_RTC_CSET_SETTLE_US + 1000);

	return 0;
}

static int novatek_rtc_software_reset(struct novatek_rtc *rtc)
{
	struct regmap *regmap = rtc->rtcsys->regmap;
	u32 value;
	int ret;

	ret = novatek_rtc_wait_idle(rtc);
	if (ret)
		return ret;

	ret = regmap_set_bits(regmap, NOVATEK_RTC_CTRL,
			      NOVATEK_RTC_CTRL_SRST_MASK);
	if (ret)
		return ret;

	return regmap_read_poll_timeout(regmap, NOVATEK_RTC_CTRL, value,
					!(value & NOVATEK_RTC_CTRL_SRST_MASK),
					NOVATEK_RTC_POLL_DELAY_US,
					NOVATEK_RTC_POLL_TIMEOUT_US);
}

static int novatek_rtc_power_lost(struct novatek_rtc *rtc, bool *power_lost)
{
	u32 daykey;
	u32 oscan;
	int ret;

	ret = regmap_read(rtc->rtcsys->regmap, NOVATEK_RTC_DAYKEY, &daykey);
	if (ret)
		return ret;

	ret = regmap_read(rtc->rtcsys->regmap, NOVATEK_RTC_OSCAN, &oscan);
	if (ret)
		return ret;

	*power_lost =
		(FIELD_GET(NOVATEK_RTC_DAYKEY_KEY_MASK, daykey) &
		 NOVATEK_RTC_KEY_POWER_LOST) ||
		FIELD_GET(NOVATEK_RTC_OSCAN_CONFIG_MASK, oscan) !=
		NOVATEK_RTC_OSCAN_CONFIG;

	return 0;
}

static int novatek_rtc_read_raw(struct novatek_rtc *rtc, u32 *day,
				u32 *timer)
{
	struct regmap *regmap = rtc->rtcsys->regmap;
	u32 first_day;
	u32 first_timer;
	u32 second_day;
	u32 second_timer;
	bool power_lost;
	uint i;
	int ret;

	ret = novatek_rtc_wait_idle(rtc);
	if (ret)
		return ret;

	ret = novatek_rtc_power_lost(rtc, &power_lost);
	if (ret)
		return ret;
	if (power_lost)
		return -EINVAL;

	for (i = 0; i < NOVATEK_RTC_READ_RETRIES; i++) {
		ret = regmap_read(regmap, NOVATEK_RTC_DAYKEY, &first_day);
		if (ret)
			return ret;
		ret = regmap_read(regmap, NOVATEK_RTC_TIMER, &first_timer);
		if (ret)
			return ret;
		ret = regmap_read(regmap, NOVATEK_RTC_TIMER, &second_timer);
		if (ret)
			return ret;
		ret = regmap_read(regmap, NOVATEK_RTC_DAYKEY, &second_day);
		if (ret)
			return ret;

		if ((first_day & NOVATEK_RTC_DAYKEY_DAY_MASK) ==
		    (second_day & NOVATEK_RTC_DAYKEY_DAY_MASK) &&
		    (first_timer & NOVATEK_RTC_TIMER_TIME_MASK) ==
		    (second_timer & NOVATEK_RTC_TIMER_TIME_MASK)) {
			*day = FIELD_GET(NOVATEK_RTC_DAYKEY_DAY_MASK,
					 second_day);
			*timer = second_timer;
			return 0;
		}
	}

	return -EIO;
}

static int novatek_rtc_decode_time(u32 day, u32 timer,
				   struct rtc_time *tm)
{
	uint second;
	uint minute;
	uint hour;
	time64_t time;

	second = FIELD_GET(NOVATEK_RTC_TIMER_SECOND_MASK, timer);
	minute = FIELD_GET(NOVATEK_RTC_TIMER_MINUTE_MASK, timer);
	hour = FIELD_GET(NOVATEK_RTC_TIMER_HOUR_MASK, timer);
	if (second > 59 || minute > 59 || hour > 23)
		return -EINVAL;

	time = RTC_TIMESTAMP_BEGIN_1900 +
	       (time64_t)day * NOVATEK_RTC_SECONDS_PER_DAY +
	       hour * NOVATEK_RTC_SECONDS_PER_HOUR +
	       minute * NOVATEK_RTC_SECONDS_PER_MINUTE + second;
	rtc_time64_to_tm(time, tm);

	return 0;
}

static int novatek_rtc_decode_alarm(u32 current_day, u32 pwralarm,
				    struct rtc_time *tm)
{
	u32 alarm_day;

	alarm_day = (current_day & ~NOVATEK_RTC_PWRALARM_MAX_DAYS) |
		FIELD_GET(NOVATEK_RTC_PWRALM_DAY_MASK, pwralarm);
	if (alarm_day < current_day)
		alarm_day += NOVATEK_RTC_PWRALARM_MAX_DAYS + 1;
	if (alarm_day > FIELD_MAX(NOVATEK_RTC_DAYKEY_DAY_MASK))
		return -ERANGE;

	return novatek_rtc_decode_time(alarm_day, pwralarm, tm);
}

static int novatek_rtc_alarm_is_future(struct novatek_rtc *rtc, bool *future)
{
	struct rtc_time alarm_tm;
	struct rtc_time now_tm;
	u32 current_timer;
	u32 current_day;
	u32 pwralarm;
	int ret;

	ret = novatek_rtc_read_raw(rtc, &current_day, &current_timer);
	if (ret)
		return ret;

	ret = regmap_read(rtc->rtcsys->regmap, NOVATEK_RTC_PWRALM,
			  &pwralarm);
	if (ret)
		return ret;

	ret = novatek_rtc_decode_time(current_day, current_timer, &now_tm);
	if (ret)
		return ret;

	ret = novatek_rtc_decode_alarm(current_day, pwralarm, &alarm_tm);
	if (ret)
		return ret;

	*future = rtc_tm_to_time64(&alarm_tm) > rtc_tm_to_time64(&now_tm);

	return 0;
}

static int novatek_rtc_read_time(struct device *dev, struct rtc_time *tm)
{
	struct novatek_rtc *rtc = dev_get_drvdata(dev);
	u32 timer;
	u32 day;
	int ret;

	mutex_lock(&rtc->rtcsys->lock);
	ret = novatek_rtc_read_raw(rtc, &day, &timer);
	if (!ret)
		ret = novatek_rtc_decode_time(day, timer, tm);
	mutex_unlock(&rtc->rtcsys->lock);

	return ret;
}

static int novatek_rtc_set_time(struct device *dev, struct rtc_time *tm)
{
	struct novatek_rtc *rtc = dev_get_drvdata(dev);
	time64_t time;
	u64 elapsed;
	u32 day;
	u32 daykey;
	u32 oscan;
	u32 select;
	u32 timer;
	bool power_lost;
	int ret;

	ret = novatek_rtc_valid_time(tm);
	if (ret)
		return ret;
	time = rtc_tm_to_time64(tm);
	if (time < rtc->rtc->range_min ||
	    time > (time64_t)rtc->rtc->range_max)
		return -ERANGE;

	mutex_lock(&rtc->rtcsys->lock);
	elapsed = time - RTC_TIMESTAMP_BEGIN_1900;
	day = div_u64(elapsed, NOVATEK_RTC_SECONDS_PER_DAY);
	ret = novatek_rtc_power_lost(rtc, &power_lost);
	if (ret)
		goto out_unlock;

	if (power_lost) {
		ret = novatek_rtc_software_reset(rtc);
		if (ret)
			goto out_unlock;
	}

	select = NOVATEK_RTC_CTRL_TIME_SEL_MASK |
		 NOVATEK_RTC_CTRL_DAY_SEL_MASK;
	if (power_lost)
		select |= NOVATEK_RTC_CTRL_KEY_SEL_MASK |
			  NOVATEK_RTC_CTRL_PWRALARMDAY_SEL_MASK;
	ret = novatek_rtc_prepare_update(rtc, select);
	if (ret)
		goto out_unlock;

	timer = FIELD_PREP(NOVATEK_RTC_TIMER_SECOND_MASK, tm->tm_sec) |
		FIELD_PREP(NOVATEK_RTC_TIMER_MINUTE_MASK, tm->tm_min) |
		FIELD_PREP(NOVATEK_RTC_TIMER_HOUR_MASK, tm->tm_hour);
	ret = regmap_write(rtc->rtcsys->regmap, NOVATEK_RTC_TIMER, timer);
	if (ret)
		goto out_unlock;

	ret = regmap_read(rtc->rtcsys->regmap, NOVATEK_RTC_DAYKEY,
			  &daykey);
	if (ret)
		goto out_unlock;
	daykey &= ~NOVATEK_RTC_DAYKEY_DAY_MASK;
	daykey |= FIELD_PREP(NOVATEK_RTC_DAYKEY_DAY_MASK, day);
	if (power_lost) {
		daykey &= ~NOVATEK_RTC_DAYKEY_KEY_MASK;
		daykey |= FIELD_PREP(NOVATEK_RTC_DAYKEY_KEY_MASK,
				     NOVATEK_RTC_KEY_VALID);
	}
	ret = regmap_write(rtc->rtcsys->regmap, NOVATEK_RTC_DAYKEY, daykey);
	if (ret)
		goto out_unlock;

	if (power_lost) {
		oscan = FIELD_PREP(NOVATEK_RTC_OSCAN_CONFIG_MASK,
				   NOVATEK_RTC_OSCAN_CONFIG);
		ret = regmap_update_bits(rtc->rtcsys->regmap,
					 NOVATEK_RTC_OSCAN,
					 NOVATEK_RTC_OSCAN_CONFIG_MASK,
					 oscan);
		if (ret)
			goto out_unlock;
	}

	ret = novatek_rtc_commit_update(rtc);

out_unlock:
	mutex_unlock(&rtc->rtcsys->lock);

	return ret;
}

static int novatek_rtc_wait_pwbc_idle(struct novatek_rtc *rtc)
{
	u32 value;

	return regmap_read_poll_timeout(rtc->rtcsys->regmap,
					NOVATEK_RTC_PWBC, value,
					!(value &
					  NOVATEK_RTC_PWBC_COMMAND_MASK),
					NOVATEK_RTC_POLL_DELAY_US,
					NOVATEK_RTC_POLL_TIMEOUT_US);
}

static int novatek_rtc_set_power_alarm(struct novatek_rtc *rtc,
				       bool enabled)
{
	u32 command;
	int ret;

	ret = novatek_rtc_wait_pwbc_idle(rtc);
	if (ret)
		return ret;

	command = enabled ? NOVATEK_RTC_PWBC_PWRALARM_EN_MASK :
		NOVATEK_RTC_PWBC_PWRALARM_DIS_MASK;
	ret = regmap_update_bits(rtc->rtcsys->regmap, NOVATEK_RTC_PWBC,
				 NOVATEK_RTC_PWBC_PWRALARM_EN_MASK |
				 NOVATEK_RTC_PWBC_PWRALARM_DIS_MASK,
				 command);
	if (ret)
		return ret;

	return novatek_rtc_wait_pwbc_idle(rtc);
}

static int novatek_rtc_set_alarm_enabled(struct novatek_rtc *rtc,
					 bool enabled)
{
	int ret;

	if (!enabled) {
		if (rtc->irq_enabled) {
			disable_irq(rtc->irq);
			rtc->irq_enabled = false;
		}

		return novatek_rtc_set_power_alarm(rtc, false);
	}

	ret = novatek_rtc_set_power_alarm(rtc, true);
	if (ret)
		return ret;

	if (!rtc->irq_enabled) {
		enable_irq(rtc->irq);
		rtc->irq_enabled = true;
	}

	return 0;
}

static int novatek_rtc_read_alarm(struct device *dev,
				  struct rtc_wkalrm *alarm)
{
	struct novatek_rtc *rtc = dev_get_drvdata(dev);
	u32 current_day;
	u32 current_timer;
	u32 pwralarm;
	u32 pwbcsts;
	u32 status;
	u32 ctrl;
	int ret;

	mutex_lock(&rtc->rtcsys->lock);
	ret = novatek_rtc_read_raw(rtc, &current_day, &current_timer);
	if (ret)
		goto out_unlock;
	ret = regmap_read(rtc->rtcsys->regmap, NOVATEK_RTC_PWRALM,
			  &pwralarm);
	if (ret)
		goto out_unlock;
	ret = regmap_read(rtc->rtcsys->regmap, NOVATEK_RTC_CTRL, &ctrl);
	if (ret)
		goto out_unlock;
	ret = regmap_read(rtc->rtcsys->regmap, NOVATEK_RTC_STATUS, &status);
	if (ret)
		goto out_unlock;
	ret = regmap_read(rtc->rtcsys->regmap, NOVATEK_RTC_PWBCSTS,
			  &pwbcsts);
	if (ret)
		goto out_unlock;

	ret = novatek_rtc_decode_alarm(current_day, pwralarm, &alarm->time);
	if (ret)
		goto out_unlock;

	alarm->enabled =
		!!(ctrl & NOVATEK_RTC_CTRL_ALARM_INTEN_MASK) &&
		!!(pwbcsts & NOVATEK_RTC_PWBCSTS_PWRALARM_EN_MASK);
	alarm->pending = !!(status & NOVATEK_RTC_STATUS_ALARM_MASK);

out_unlock:
	mutex_unlock(&rtc->rtcsys->lock);

	return ret;
}

static int novatek_rtc_set_alarm(struct device *dev,
				 struct rtc_wkalrm *alarm)
{
	struct novatek_rtc *rtc = dev_get_drvdata(dev);
	struct rtc_time now_tm;
	time64_t scheduled;
	time64_t now;
	u64 elapsed;
	u32 current_timer;
	u32 current_day;
	u32 alarm_day;
	u32 alarm_timer;
	u32 pwralarm;
	u32 oscan;
	u32 select;
	int ret;

	ret = novatek_rtc_valid_time(&alarm->time);
	if (ret)
		return ret;

	scheduled = rtc_tm_to_time64(&alarm->time);
	if (scheduled < rtc->rtc->range_min ||
	    scheduled > (time64_t)rtc->rtc->range_max)
		return -ERANGE;

	mutex_lock(&rtc->rtcsys->lock);
	ret = novatek_rtc_read_raw(rtc, &current_day, &current_timer);
	if (ret)
		goto out_unlock;
	ret = novatek_rtc_decode_time(current_day, current_timer, &now_tm);
	if (ret)
		goto out_unlock;
	now = rtc_tm_to_time64(&now_tm);
	if (scheduled <= now) {
		ret = -ETIME;
		goto out_unlock;
	}

	elapsed = scheduled - RTC_TIMESTAMP_BEGIN_1900;
	alarm_day = div_u64(elapsed, NOVATEK_RTC_SECONDS_PER_DAY);
	if (alarm_day < current_day ||
	    alarm_day - current_day > NOVATEK_RTC_PWRALARM_MAX_DAYS) {
		ret = -ERANGE;
		goto out_unlock;
	}

	ret = novatek_rtc_set_alarm_enabled(rtc, false);
	if (ret)
		goto out_unlock;

	select = NOVATEK_RTC_CTRL_PWRALARMTIME_SEL_MASK |
		 NOVATEK_RTC_CTRL_PWRALARMDAY_SEL_MASK;
	ret = novatek_rtc_prepare_update(rtc, select);
	if (ret)
		goto out_unlock;

	ret = regmap_write(rtc->rtcsys->regmap, NOVATEK_RTC_STATUS,
			   NOVATEK_RTC_STATUS_ALARM_MASK);
	if (ret)
		goto out_unlock;

	/*
	 * ALARM supplies the running-state IRQ and matches HMS. PWRALM adds
	 * the day qualification needed to turn the system back on. The RTC
	 * core retains the absolute deadline across intermediary ALARM matches.
	 */
	alarm_timer = FIELD_PREP(NOVATEK_RTC_ALARM_SECOND_MASK,
				 alarm->time.tm_sec) |
		FIELD_PREP(NOVATEK_RTC_ALARM_MINUTE_MASK,
			   alarm->time.tm_min) |
		FIELD_PREP(NOVATEK_RTC_ALARM_HOUR_MASK,
			   alarm->time.tm_hour);
	ret = regmap_write(rtc->rtcsys->regmap, NOVATEK_RTC_ALARM,
			   alarm_timer);
	if (ret)
		goto out_unlock;

	pwralarm = FIELD_PREP(NOVATEK_RTC_PWRALM_SECOND_MASK,
			      alarm->time.tm_sec) |
		   FIELD_PREP(NOVATEK_RTC_PWRALM_MINUTE_MASK,
			      alarm->time.tm_min) |
		   FIELD_PREP(NOVATEK_RTC_PWRALM_HOUR_MASK,
			      alarm->time.tm_hour) |
		   FIELD_PREP(NOVATEK_RTC_PWRALM_DAY_MASK, alarm_day);
	ret = regmap_write(rtc->rtcsys->regmap, NOVATEK_RTC_PWRALM,
			   pwralarm);
	if (ret)
		goto out_unlock;

	oscan = FIELD_PREP(NOVATEK_RTC_OSCAN_CONFIG_MASK,
			   NOVATEK_RTC_OSCAN_CONFIG);
	ret = regmap_update_bits(rtc->rtcsys->regmap, NOVATEK_RTC_OSCAN,
				 NOVATEK_RTC_OSCAN_CONFIG_MASK, oscan);
	if (ret)
		goto out_unlock;

	ret = novatek_rtc_commit_update(rtc);
	if (ret)
		goto out_unlock;

	if (alarm->enabled)
		ret = novatek_rtc_set_alarm_enabled(rtc, true);

out_unlock:
	mutex_unlock(&rtc->rtcsys->lock);

	return ret;
}

static int novatek_rtc_alarm_irq_enable(struct device *dev,
					unsigned int enabled)
{
	struct novatek_rtc *rtc = dev_get_drvdata(dev);
	int ret;

	mutex_lock(&rtc->rtcsys->lock);
	if (enabled) {
		ret = regmap_write(rtc->rtcsys->regmap, NOVATEK_RTC_STATUS,
				   NOVATEK_RTC_STATUS_ALARM_MASK);
		if (ret)
			goto out_unlock;
	}

	ret = novatek_rtc_set_alarm_enabled(rtc, enabled);

out_unlock:
	mutex_unlock(&rtc->rtcsys->lock);

	return ret;
}

static irqreturn_t novatek_rtc_irq(int irq, void *data)
{
	struct novatek_rtc *rtc = data;

	rtc_update_irq(rtc->rtc, 1, RTC_IRQF | RTC_AF);

	return IRQ_HANDLED;
}

static const struct rtc_class_ops novatek_rtc_ops = {
	.read_time = novatek_rtc_read_time,
	.set_time = novatek_rtc_set_time,
	.read_alarm = novatek_rtc_read_alarm,
	.set_alarm = novatek_rtc_set_alarm,
	.alarm_irq_enable = novatek_rtc_alarm_irq_enable,
};

static int novatek_rtc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct novatek_rtc *rtc;
	struct novatek_rtcsys *rtcsys;
	u32 pwbcsts;
	bool restore_alarm = false;
	bool power_lost;
	bool alarm_enabled;
	bool alarm_triggered;
	int ret;

	rtcsys = dev_get_drvdata(dev->parent);
	if (!rtcsys || !rtcsys->regmap)
		return dev_err_probe(dev, -ENODEV,
				     "failed to get parent register map\n");

	rtc = devm_kzalloc(dev, sizeof(*rtc), GFP_KERNEL);
	if (!rtc)
		return -ENOMEM;

	rtc->rtcsys = rtcsys;
	rtc->irq = platform_get_irq_byname(pdev, "alarm");
	if (rtc->irq < 0)
		return rtc->irq;

	rtc->rtc = devm_rtc_allocate_device(dev);
	if (IS_ERR(rtc->rtc))
		return PTR_ERR(rtc->rtc);
	rtc->rtc->ops = &novatek_rtc_ops;
	rtc->rtc->range_min = RTC_TIMESTAMP_BEGIN_1900;
	rtc->rtc->range_max = RTC_TIMESTAMP_BEGIN_1900 +
		(timeu64_t)(FIELD_MAX(NOVATEK_RTC_DAYKEY_DAY_MASK) + 1) *
		NOVATEK_RTC_SECONDS_PER_DAY - 1;
	rtc->rtc->alarm_offset_max = NOVATEK_RTC_ALARM_OFFSET_MAX;
	clear_bit(RTC_FEATURE_UPDATE_INTERRUPT, rtc->rtc->features);
	platform_set_drvdata(pdev, rtc);

	ret = devm_request_threaded_irq(dev, rtc->irq, NULL, novatek_rtc_irq,
					IRQF_ONESHOT | IRQF_NO_AUTOEN,
					dev_name(dev), rtc);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request RTC IRQ\n");

	if (device_property_read_bool(dev, "wakeup-source")) {
		ret = devm_device_init_wakeup(dev);
		if (ret)
			return ret;
		ret = devm_pm_set_wake_irq(dev, rtc->irq);
		if (ret)
			return dev_err_probe(dev, ret,
					     "failed to configure wake IRQ\n");
	}

	mutex_lock(&rtcsys->lock);
	ret = novatek_rtc_power_lost(rtc, &power_lost);
	if (!ret)
		ret = regmap_read(rtcsys->regmap, NOVATEK_RTC_PWBCSTS,
				  &pwbcsts);
	alarm_enabled = !ret &&
			(pwbcsts & NOVATEK_RTC_PWBCSTS_PWRALARM_EN_MASK);
	alarm_triggered = !ret &&
			  (pwbcsts & NOVATEK_RTC_PWBCSTS_PWRALARM_SRC_MASK);
	if (!ret && !power_lost && alarm_enabled && !alarm_triggered) {
		ret = novatek_rtc_alarm_is_future(rtc, &restore_alarm);
		if (ret == -EINVAL || ret == -ERANGE)
			ret = 0;
	}
	if (!ret && (power_lost || alarm_triggered ||
		     (alarm_enabled && !restore_alarm)))
		ret = novatek_rtc_set_power_alarm(rtc, false);
	mutex_unlock(&rtcsys->lock);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to initialize RTC state\n");

	if (power_lost)
		dev_warn(dev, "RTC power loss detected\n");
	if (restore_alarm) {
		enable_irq(rtc->irq);
		rtc->irq_enabled = true;
	}

	return devm_rtc_register_device(rtc->rtc);
}

static const struct of_device_id novatek_rtc_of_match[] = {
	{ .compatible = "novatek,na51089-rtc" },
	{ }
};
MODULE_DEVICE_TABLE(of, novatek_rtc_of_match);

static struct platform_driver novatek_rtc_driver = {
	.probe = novatek_rtc_probe,
	.driver = {
		.name = "novatek-rtc",
		.of_match_table = novatek_rtc_of_match,
	},
};
module_platform_driver(novatek_rtc_driver);

MODULE_DESCRIPTION("Novatek real-time clock driver");
MODULE_LICENSE("GPL");
