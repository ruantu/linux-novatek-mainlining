// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2026 Yu-Tung Chang <mtwget@gmail.com>
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/iio/iio.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/nvmem-consumer.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/property.h>
#include <linux/reset.h>

#define ADC_CTRL_REG			0x00
#define ADC_ONESHOT_REG			0x04
#define ADC_INT_ENABLE_REG		0x08
#define ADC_STATUS_REG			0x0c
#define ADC_DATA_REG(channel)		(0x30 + (channel) * 4)
#define ADC_THERMAL_REG			0x40

/* ADC_CTRL bit fields */
#define ADC_CTRL_CONTINUOUS(channel)	BIT(channel)
#define ADC_CTRL_CLKDIV			GENMASK(16, 12)
#define ADC_CTRL_SAMPLE_AVERAGE		GENMASK(21, 20)
#define ADC_CTRL_SAMPLE_AVERAGE_ONCE	0
#define ADC_CTRL_ENABLE			BIT(23)
#define ADC_CTRL_EXTRA_SAMPLE_CLOCKS	GENMASK(27, 24)
#define ADC_CTRL_DEFAULT \
	(FIELD_PREP_CONST(ADC_CTRL_CONTINUOUS(0), 0) | \
	 FIELD_PREP_CONST(ADC_CTRL_CONTINUOUS(1), 0) | \
	 FIELD_PREP_CONST(ADC_CTRL_CONTINUOUS(2), 0) | \
	 FIELD_PREP_CONST(ADC_CTRL_CONTINUOUS(3), 1) | \
	 FIELD_PREP_CONST(ADC_CTRL_SAMPLE_AVERAGE, ADC_CTRL_SAMPLE_AVERAGE_ONCE) | \
	 FIELD_PREP_CONST(ADC_CTRL_ENABLE, 0) | \
	 FIELD_PREP_CONST(ADC_CTRL_EXTRA_SAMPLE_CLOCKS, 0))

/* ADC_ONESHOT bit fields */
#define ADC_ONESHOT_CHANNEL(channel)	BIT(channel)

/* ADC_INT_ENABLE bit fields */
#define ADC_INT_ENABLE_READY(channel)	BIT(channel)
#define ADC_INT_ENABLE_TRIGGER(channel)	BIT((channel) + 4)
#define ADC_INT_ENABLE_NONE \
	(FIELD_PREP_CONST(ADC_INT_ENABLE_READY(0), 0) | \
	 FIELD_PREP_CONST(ADC_INT_ENABLE_READY(1), 0) | \
	 FIELD_PREP_CONST(ADC_INT_ENABLE_READY(2), 0) | \
	 FIELD_PREP_CONST(ADC_INT_ENABLE_READY(3), 0) | \
	 FIELD_PREP_CONST(ADC_INT_ENABLE_TRIGGER(0), 0) | \
	 FIELD_PREP_CONST(ADC_INT_ENABLE_TRIGGER(1), 0) | \
	 FIELD_PREP_CONST(ADC_INT_ENABLE_TRIGGER(2), 0))

/* ADC_STATUS bit fields */
#define ADC_STATUS_READY(channel)	BIT(channel)
#define ADC_STATUS_PENDING		GENMASK(6, 0)

/* ADC_DATA bit fields */
#define ADC_DATA_VALUE			GENMASK(8, 0)

/* ADC_THERMAL bit fields */
#define ADC_THERMAL_AVERAGE_COUNT	GENMASK(2, 0)
#define ADC_THERMAL_AVERAGE_COUNT_TWO	0
#define ADC_THERMAL_AVERAGE_CHANNEL	GENMASK(5, 4)
#define ADC_THERMAL_AVERAGE_VALUE	GENMASK(24, 16)

#define ADC_TRIM_VALUE			GENMASK(8, 0)
#define ADC_TRIM_INVALID		BIT(14)
#define ADC_TRIM_VALID			BIT(15)
#define ADC_TRIM_BACKUP			GENMASK(31, 16)

#define ADC_FULL_SCALE_MV		2700
#define ADC_MAX_RAW			511
#define ADC_THERMAL_SAMPLES		2

#define ADC_POLL_US			10
#define ADC_TIMEOUT_US			20000

#define ADC_MAX_CLOCK_HZ		1000000

struct novatek_adc_thermal_data {
	int default_trim;
	int min_trim;
	int max_trim;
	int slope;
	int offset;
};

struct novatek_adc_soc_data {
	const struct iio_chan_spec *channels;
	u32 num_channels;
	const struct novatek_adc_thermal_data *thermal;
	u32 thermal_average_channel;
};

struct novatek_adc {
	void __iomem *base;
	struct clk *clk;
	struct reset_control *reset;
	const struct novatek_adc_soc_data *data;
	struct mutex lock;
	u32 control;
	int thermal_trim;
	bool active;
	bool fault;
};

static int novatek_adc_init_hw(struct novatek_adc *adc)
{
	ulong rate;
	u32 divider;
	int ret;

	rate = clk_get_rate(adc->clk);
	divider = DIV_ROUND_UP(rate, 2 * ADC_MAX_CLOCK_HZ);
	if (!divider || divider - 1 > FIELD_MAX(ADC_CTRL_CLKDIV))
		return -EINVAL;

	ret = reset_control_reset(adc->reset);
	if (ret)
		return ret;

	adc->control = ADC_CTRL_DEFAULT | FIELD_PREP(ADC_CTRL_CLKDIV, divider - 1);
	writel(adc->control, adc->base + ADC_CTRL_REG);

	writel(ADC_INT_ENABLE_NONE, adc->base + ADC_INT_ENABLE_REG);

	writel(ADC_STATUS_PENDING, adc->base + ADC_STATUS_REG);

	writel(FIELD_PREP(ADC_THERMAL_AVERAGE_COUNT, ADC_THERMAL_AVERAGE_COUNT_TWO) |
	       FIELD_PREP(ADC_THERMAL_AVERAGE_CHANNEL, adc->data->thermal_average_channel),
	       adc->base + ADC_THERMAL_REG);

	writel(adc->control | ADC_CTRL_ENABLE, adc->base + ADC_CTRL_REG);
	adc->fault = false;

	return 0;
}

static int novatek_adc_read(struct novatek_adc *adc, uint channel, int *value)
{
	u32 ready = ADC_STATUS_READY(channel);
	uint samples;
	u32 status;
	int ret;

	mutex_lock(&adc->lock);
	if (!adc->active) {
		ret = -EBUSY;
		goto out_unlock;
	}

	if (adc->fault) {
		ret = novatek_adc_init_hw(adc);
		if (ret)
			goto out_unlock;
	}

	/* Disabling the ADC clears the thermal average; wait for a complete pair. */
	samples = channel == adc->data->thermal_average_channel ? ADC_THERMAL_SAMPLES : 1;
	while (samples--) {
		writel(ready, adc->base + ADC_STATUS_REG);

		if (channel != adc->data->thermal_average_channel)
			writel(ADC_ONESHOT_CHANNEL(channel), adc->base + ADC_ONESHOT_REG);

		ret = readl_poll_timeout(adc->base + ADC_STATUS_REG,
					 status, status & ready,
					 ADC_POLL_US, ADC_TIMEOUT_US);
		if (ret) {
			writel(adc->control, adc->base + ADC_CTRL_REG);
			readl(adc->base + ADC_CTRL_REG);
			adc->fault = true;
			goto out_unlock;
		}
	}

	if (channel == adc->data->thermal_average_channel) {
		status = readl(adc->base + ADC_THERMAL_REG);
		*value = FIELD_GET(ADC_THERMAL_AVERAGE_VALUE, status);
	} else {
		status = readl(adc->base + ADC_DATA_REG(channel));
		*value = FIELD_GET(ADC_DATA_VALUE, status);
	}

out_unlock:
	mutex_unlock(&adc->lock);
	return ret;
}

static int novatek_adc_read_raw(struct iio_dev *indio_dev, const struct iio_chan_spec *chan,
				int *val, int *val2, long mask)
{
	struct novatek_adc *adc = iio_priv(indio_dev);
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_RAW:
		ret = novatek_adc_read(adc, chan->channel, val);
		return ret ? ret : IIO_VAL_INT;
	case IIO_CHAN_INFO_SCALE:
		/* The untrimmed transfer function maps codes 0..511 to 0..2700 mV. */
		*val = ADC_FULL_SCALE_MV;
		*val2 = ADC_MAX_RAW;
		return IIO_VAL_FRACTIONAL;

	case IIO_CHAN_INFO_PROCESSED:
		if (chan->type != IIO_TEMP)
			return -EINVAL;

		ret = novatek_adc_read(adc, chan->channel, val);
		if (ret)
			return ret;

		*val = (*val - adc->thermal_trim) * adc->data->thermal->slope +
			adc->data->thermal->offset;
		return IIO_VAL_INT;

	default:
		return -EINVAL;
	}
}

static int novatek_adc_fwnode_xlate(struct iio_dev *indio_dev,
				    const struct fwnode_reference_args *iiospec)
{
	uint i;

	if (iiospec->nargs != 1)
		return -EINVAL;

	for (i = 0; i < indio_dev->num_channels; i++) {
		if (indio_dev->channels[i].channel == iiospec->args[0])
			return i;
	}

	return -EINVAL;
}

static const struct iio_info novatek_adc_info = {
	.read_raw = novatek_adc_read_raw,
	.fwnode_xlate = novatek_adc_fwnode_xlate,
};

#define ADC_CHANNEL(_channel) { \
	.type = IIO_VOLTAGE, \
	.indexed = 1, \
	.channel = (_channel), \
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW), \
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE), \
}

#define ADC_TEMP_CHANNEL(_channel) { \
	.type = IIO_TEMP, \
	.channel = (_channel), \
	.info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED), \
}

static const struct iio_chan_spec na51089_adc_channels[] = {
	ADC_CHANNEL(0),
	ADC_CHANNEL(1),
	ADC_CHANNEL(2),
	ADC_TEMP_CHANNEL(3),
};

static int novatek_adc_read_calibration(struct device *dev, struct novatek_adc *adc)
{
	u32 trim;
	int ret;

	adc->thermal_trim = adc->data->thermal->default_trim;
	if (!device_property_present(dev, "nvmem-cells"))
		return 0;

	ret = nvmem_cell_read_u32(dev, "thermal-calibration", &trim);
	if (ret)
		return ret;

	trim = le32_to_cpu((__force __le32)trim);
	if (!(trim & ADC_TRIM_VALID) || (trim & ADC_TRIM_INVALID))
		trim = FIELD_GET(ADC_TRIM_BACKUP, trim);
	if (!(trim & ADC_TRIM_VALID) || (trim & ADC_TRIM_INVALID))
		return 0;

	trim = FIELD_GET(ADC_TRIM_VALUE, trim);
	if (trim >= adc->data->thermal->min_trim && trim <= adc->data->thermal->max_trim)
		adc->thermal_trim = trim;

	return 0;
}

static void novatek_adc_power_off(void *data)
{
	struct novatek_adc *adc = data;

	if (adc->active) {
		writel(adc->control, adc->base + ADC_CTRL_REG);
		readl(adc->base + ADC_CTRL_REG);

		clk_disable_unprepare(adc->clk);
		adc->active = false;
	}
}

#ifdef CONFIG_PM_SLEEP
static int novatek_adc_suspend(struct device *dev)
{
	struct novatek_adc *adc = dev_get_drvdata(dev);

	mutex_lock(&adc->lock);
	novatek_adc_power_off(adc);
	mutex_unlock(&adc->lock);

	return 0;
}

static int novatek_adc_resume(struct device *dev)
{
	struct novatek_adc *adc = dev_get_drvdata(dev);
	int ret;

	mutex_lock(&adc->lock);
	ret = clk_prepare_enable(adc->clk);
	if (ret)
		goto out_unlock;

	ret = novatek_adc_init_hw(adc);
	if (ret) {
		clk_disable_unprepare(adc->clk);
		goto out_unlock;
	}

	adc->active = true;

out_unlock:
	mutex_unlock(&adc->lock);
	return ret;
}
#endif

static const struct dev_pm_ops novatek_adc_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(novatek_adc_suspend, novatek_adc_resume)
};

static int novatek_adc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct iio_dev *indio_dev;
	struct novatek_adc *adc;
	int ret;

	indio_dev = devm_iio_device_alloc(dev, sizeof(*adc));
	if (!indio_dev)
		return -ENOMEM;

	adc = iio_priv(indio_dev);
	adc->data = device_get_match_data(dev);
	mutex_init(&adc->lock);
	platform_set_drvdata(pdev, adc);

	adc->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(adc->base))
		return PTR_ERR(adc->base);

	adc->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(adc->clk))
		return dev_err_probe(dev, PTR_ERR(adc->clk), "Failed to get clock\n");

	adc->reset = devm_reset_control_get_exclusive(dev, NULL);
	if (IS_ERR(adc->reset))
		return dev_err_probe(dev, PTR_ERR(adc->reset), "Failed to get reset\n");

	ret = novatek_adc_read_calibration(dev, adc);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to read thermal calibration\n");

	ret = clk_prepare_enable(adc->clk);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to enable clock\n");

	ret = novatek_adc_init_hw(adc);
	if (ret) {
		clk_disable_unprepare(adc->clk);
		return dev_err_probe(dev, ret, "Failed to initialize ADC\n");
	}

	adc->active = true;
	ret = devm_add_action_or_reset(dev, novatek_adc_power_off, adc);
	if (ret)
		return ret;

	indio_dev->name = "novatek-adc";
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->info = &novatek_adc_info;
	indio_dev->channels = adc->data->channels;
	indio_dev->num_channels = adc->data->num_channels;

	ret = devm_iio_device_register(dev, indio_dev);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to register ADC\n");

	return 0;
}

static const struct novatek_adc_thermal_data na51089_adc_thermal_data = {
	.default_trim = 231,
	.min_trim = 191,
	.max_trim = 271,
	.slope = 1319,
	.offset = 30000,
};

static const struct novatek_adc_soc_data na51089_adc_data = {
	.channels = na51089_adc_channels,
	.num_channels = ARRAY_SIZE(na51089_adc_channels),
	.thermal = &na51089_adc_thermal_data,
	.thermal_average_channel = 3,
};

static const struct of_device_id novatek_adc_of_match[] = {
	{ .compatible = "novatek,na51089-adc", .data = &na51089_adc_data },
	{ }
};
MODULE_DEVICE_TABLE(of, novatek_adc_of_match);

static struct platform_driver novatek_adc_driver = {
	.probe = novatek_adc_probe,
	.driver = {
		.name = "novatek-adc",
		.of_match_table = novatek_adc_of_match,
		.pm = &novatek_adc_pm_ops,
	},
};
module_platform_driver(novatek_adc_driver);

MODULE_IMPORT_NS("IIO_DRIVER");
MODULE_AUTHOR("Yu-Tung Chang <mtwget@gmail.com>");
MODULE_DESCRIPTION("Novatek ADC and temperature sensor driver");
MODULE_LICENSE("GPL");
