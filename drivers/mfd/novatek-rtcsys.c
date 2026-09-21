// SPDX-License-Identifier: GPL-2.0-only

#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/mfd/core.h>
#include <linux/mfd/novatek-rtcsys.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/reset.h>

#define NOVATEK_RTCSYS_POLL_DELAY_US	100
#define NOVATEK_RTCSYS_POLL_TIMEOUT_US	1000000

static const struct regmap_config novatek_rtcsys_regmap_config = {
	.name = "rtcsys",
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
	.max_register = NOVATEK_RTC_OSCAN,
	.cache_type = REGCACHE_NONE,
	.fast_io = true,
};

static const struct regmap_irq novatek_rtcsys_irqs[] = {
	[NOVATEK_RTCSYS_IRQ_ALARM] = {
		.mask = NOVATEK_RTC_STATUS_ALARM_MASK,
	},
	[NOVATEK_RTCSYS_IRQ_CSET] = {
		.mask = NOVATEK_RTC_STATUS_CSET_MASK,
	},
};

static const struct regmap_irq_chip novatek_rtcsys_irq_chip = {
	.name = "novatek-rtcsys",
	.status_base = NOVATEK_RTC_STATUS,
	.unmask_base = NOVATEK_RTC_CTRL,
	.ack_base = NOVATEK_RTC_STATUS,
	.init_ack_masked = true,
	.num_regs = 1,
	.irqs = novatek_rtcsys_irqs,
	.num_irqs = ARRAY_SIZE(novatek_rtcsys_irqs),
};

static const struct resource novatek_rtc_resources[] = {
	DEFINE_RES_IRQ_NAMED(NOVATEK_RTCSYS_IRQ_ALARM, "alarm"),
};

static const struct mfd_cell novatek_rtcsys_cells[] = {
	{
		.name = "novatek-rtc",
		.of_compatible = "novatek,na51089-rtc",
		.resources = novatek_rtc_resources,
		.num_resources = ARRAY_SIZE(novatek_rtc_resources),
	}, {
		.name = "novatek-pwbc",
		.of_compatible = "novatek,na51089-pwbc",
	},
};

static int novatek_rtcsys_disable_pwbc_irqs(struct novatek_rtcsys *rtcsys)
{
	u32 value;
	int ret;

	ret = regmap_read_poll_timeout(rtcsys->regmap, NOVATEK_RTC_PWBC,
				       value,
				       !(value & NOVATEK_RTC_PWBC_COMMAND_MASK),
				       NOVATEK_RTCSYS_POLL_DELAY_US,
				       NOVATEK_RTCSYS_POLL_TIMEOUT_US);
	if (ret)
		return ret;

	ret = regmap_clear_bits(rtcsys->regmap, NOVATEK_RTC_PWBC,
				NOVATEK_RTC_PWBC_PWR_SW1_INTEN_MASK |
				NOVATEK_RTC_PWBC_PWR_SW2_INTEN_MASK);
	if (ret)
		return ret;

	return regmap_write(rtcsys->regmap, NOVATEK_RTC_PWBCSTS,
			    NOVATEK_RTC_PWBCSTS_PWR_SW1_STATUS_MASK |
			    NOVATEK_RTC_PWBCSTS_PWR_SW2_STATUS_MASK);
}

static int novatek_rtcsys_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct regmap_irq_chip_data *irq_data;
	struct novatek_rtcsys *rtcsys;
	struct reset_control *reset;
	void __iomem *base;
	int irq;
	int ret;

	rtcsys = devm_kzalloc(dev, sizeof(*rtcsys), GFP_KERNEL);
	if (!rtcsys)
		return -ENOMEM;

	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base))
		return PTR_ERR(base);

	rtcsys->regmap = devm_regmap_init_mmio(dev, base,
					       &novatek_rtcsys_regmap_config);
	if (IS_ERR(rtcsys->regmap))
		return dev_err_probe(dev, PTR_ERR(rtcsys->regmap),
				     "failed to initialize register map\n");

	/* Keep the battery-backed counter running when the driver unbinds. */
	reset = devm_reset_control_get_exclusive(dev, NULL);
	if (IS_ERR(reset))
		return dev_err_probe(dev, PTR_ERR(reset),
				     "failed to get RTC reset\n");

	ret = reset_control_deassert(reset);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to deassert RTC reset\n");

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;

	mutex_init(&rtcsys->lock);
	platform_set_drvdata(pdev, rtcsys);

	ret = novatek_rtcsys_disable_pwbc_irqs(rtcsys);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to disable PWBC interrupts\n");

	ret = devm_regmap_add_irq_chip(dev, rtcsys->regmap, irq, 0, 0,
				       &novatek_rtcsys_irq_chip, &irq_data);
	if (ret)
		return dev_err_probe(dev, ret, "failed to add IRQ chip\n");

	ret = devm_mfd_add_devices(dev, PLATFORM_DEVID_NONE,
				   novatek_rtcsys_cells,
				   ARRAY_SIZE(novatek_rtcsys_cells),
				   NULL, 0,
				   regmap_irq_get_domain(irq_data));
	if (ret)
		return dev_err_probe(dev, ret, "failed to add child devices\n");

	return 0;
}

static const struct of_device_id novatek_rtcsys_of_match[] = {
	{ .compatible = "novatek,na51089-rtcsys" },
	{ }
};
MODULE_DEVICE_TABLE(of, novatek_rtcsys_of_match);

static struct platform_driver novatek_rtcsys_driver = {
	.probe = novatek_rtcsys_probe,
	.driver = {
		.name = "novatek-rtcsys",
		.of_match_table = novatek_rtcsys_of_match,
	},
};
module_platform_driver(novatek_rtcsys_driver);

MODULE_DESCRIPTION("Novatek RTC subsystem driver");
MODULE_LICENSE("GPL");
