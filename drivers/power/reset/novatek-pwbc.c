// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2026 Yu-Tung Chang <mtwget@gmail.com>
 */

#include <linux/mfd/novatek-rtcsys.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/reboot.h>
#include <linux/slab.h>

#define PWBC_POLL_DELAY_US		100
#define PWBC_POLL_TIMEOUT_US		1000000

struct novatek_pwbc {
	struct device *dev;
	struct regmap *regmap;
};

static int novatek_pwbc_wait_rtc_idle(struct novatek_pwbc *pwbc)
{
	unsigned int value;
	int ret;

	ret = regmap_read_poll_timeout_atomic(pwbc->regmap, RTC_STATUS_REG,
					      value, !(value & RTC_STATUS_SRST),
					      PWBC_POLL_DELAY_US, PWBC_POLL_TIMEOUT_US);
	if (ret)
		return ret;

	return regmap_read_poll_timeout_atomic(pwbc->regmap, RTC_CTRL_REG,
					       value, !(value & RTC_CTRL_CSET),
					       PWBC_POLL_DELAY_US, PWBC_POLL_TIMEOUT_US);
}

static int novatek_pwbc_wait_command_idle(struct novatek_pwbc *pwbc, unsigned int mask)
{
	unsigned int value;

	return regmap_read_poll_timeout_atomic(pwbc->regmap, RTC_PWBC_REG,
					       value, !(value & mask),
					       PWBC_POLL_DELAY_US, PWBC_POLL_TIMEOUT_US);
}

static int novatek_pwbc_power_off(struct sys_off_data *data)
{
	struct novatek_pwbc *pwbc = data->cb_data;
	unsigned int command;
	int ret;

	ret = novatek_pwbc_wait_rtc_idle(pwbc);
	if (ret) {
		dev_err(pwbc->dev, "RTC did not become idle: %d\n", ret);
		return notifier_from_errno(ret);
	}

	ret = novatek_pwbc_wait_command_idle(pwbc, RTC_PWBC_COMMAND);
	if (ret) {
		dev_err(pwbc->dev, "PWBC did not become idle: %d\n", ret);
		return notifier_from_errno(ret);
	}

	command = RTC_PWBC_RESET_SDT_TIMER_CONFIG;
	ret = regmap_write(pwbc->regmap, RTC_PWBC_REG, command);
	if (ret) {
		dev_err(pwbc->dev, "failed to reset shutdown timer: %d\n", ret);
		return notifier_from_errno(ret);
	}

	ret = novatek_pwbc_wait_command_idle(pwbc, command);
	if (ret) {
		dev_err(pwbc->dev, "shutdown timer reset did not complete: %d\n", ret);
		return notifier_from_errno(ret);
	}

	command = RTC_PWBC_PWR_OFF_CONFIG;
	ret = regmap_write(pwbc->regmap, RTC_PWBC_REG, command);
	if (ret) {
		dev_err(pwbc->dev, "failed to request power-off: %d\n", ret);
		return notifier_from_errno(ret);
	}

	return NOTIFY_DONE;
}

static int novatek_pwbc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct novatek_rtcsys *rtcsys;
	struct novatek_pwbc *pwbc;
	int ret;

	if (!device_property_read_bool(dev, "system-power-controller"))
		return 0;

	rtcsys = dev_get_drvdata(dev->parent);
	if (!rtcsys || !rtcsys->regmap)
		return dev_err_probe(dev, -ENODEV, "failed to get parent register map\n");

	pwbc = devm_kzalloc(dev, sizeof(*pwbc), GFP_KERNEL);
	if (!pwbc)
		return -ENOMEM;

	pwbc->dev = dev;
	pwbc->regmap = rtcsys->regmap;

	ret = devm_register_power_off_handler(dev, novatek_pwbc_power_off, pwbc);
	if (ret)
		return dev_err_probe(dev, ret, "failed to register power-off handler\n");

	return 0;
}

static const struct of_device_id novatek_pwbc_of_match[] = {
	{ .compatible = "novatek,na51089-pwbc" },
	{ }
};
MODULE_DEVICE_TABLE(of, novatek_pwbc_of_match);

static struct platform_driver novatek_pwbc_driver = {
	.probe = novatek_pwbc_probe,
	.driver = {
		.name = "novatek-pwbc",
		.of_match_table = novatek_pwbc_of_match,
	},
};
module_platform_driver(novatek_pwbc_driver);

MODULE_AUTHOR("Yu-Tung Chang <mtwget@gmail.com>");
MODULE_DESCRIPTION("Novatek power button controller driver");
MODULE_LICENSE("GPL");
