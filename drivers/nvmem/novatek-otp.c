// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2026 Yu-Tung Chang <mtwget@gmail.com>
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/nvmem-provider.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/reset.h>

#define OTP_CONFIG_REG			0x00
#define OTP_ADDRESS_REG			0x04
#define OTP_DATA_REG			0x08
#define OTP_TIMING_REG			0x10
#define OTP_CONTROL_REG			0x20
#define OTP_STATUS_REG			0x30

/* OTP_CONFIG bit fields */
#define OTP_CONFIG_MODE			GENMASK(4, 0)
#define OTP_CONFIG_AUTO_READ		0x00
#define OTP_CONFIG_DEFAULT \
	(FIELD_PREP_CONST(OTP_CONFIG_MODE, OTP_CONFIG_AUTO_READ))

/* OTP_ADDRESS bit fields */
#define OTP_ADDRESS_ROW			GENMASK(4, 0)

/* OTP_TIMING bit fields */
#define OTP_TIMING_48M			0x00692618
#define OTP_TIMING_60M			0x008b3026
#define OTP_TIMING_80M			0x00af4032
#define OTP_TIMING_120M			0x00ff6050

/* OTP_CONTROL bit fields */
#define OTP_CONTROL_SETUP		BIT(0)
#define OTP_CONTROL_OPERATION		BIT(4)

/* OTP_STATUS bit fields */
#define OTP_STATUS_READY		BIT(0)
#define OTP_STATUS_DONE			BIT(1)

#define OTP_POLL_US			10
#define OTP_TIMEOUT_US			10000

struct novatek_otp_soc_data {
	uint size;
};

struct novatek_otp {
	const struct novatek_otp_soc_data *data;
	void __iomem *base;
	struct mutex lock;
	u32 timing;
	bool fault;
};

static const struct nvmem_keepout na51089_otp_keepouts[] = {
	{ .start = 0x20, .end = 0x28 },
	{ .start = 0x30, .end = 0x80 },
};

static int novatek_otp_read_word(struct novatek_otp *otp, uint row, u32 *word)
{
	u32 control, status, data;
	int ret, stop_ret;

	control = readl(otp->base + OTP_CONTROL_REG);
	if (control & (OTP_CONTROL_SETUP | OTP_CONTROL_OPERATION))
		return -EBUSY;

	writel(otp->timing, otp->base + OTP_TIMING_REG);

	status = readl(otp->base + OTP_STATUS_REG);
	writel(status, otp->base + OTP_STATUS_REG);

	control |= OTP_CONTROL_SETUP;
	writel(control, otp->base + OTP_CONTROL_REG);
	ret = readl_poll_timeout(otp->base + OTP_STATUS_REG,
				 status, status & OTP_STATUS_READY,
				 OTP_POLL_US, OTP_TIMEOUT_US);
	if (ret)
		goto out_stop;

	writel(OTP_CONFIG_DEFAULT, otp->base + OTP_CONFIG_REG);

	status = readl(otp->base + OTP_STATUS_REG);
	writel(status, otp->base + OTP_STATUS_REG);

	writel(FIELD_PREP(OTP_ADDRESS_ROW, row), otp->base + OTP_ADDRESS_REG);

	control |= OTP_CONTROL_OPERATION;
	writel(control, otp->base + OTP_CONTROL_REG);
	ret = readl_poll_timeout(otp->base + OTP_CONTROL_REG,
				 control, !(control & OTP_CONTROL_OPERATION),
				 OTP_POLL_US, OTP_TIMEOUT_US);
	if (ret)
		goto out_stop;

	ret = readl_poll_timeout(otp->base + OTP_STATUS_REG,
				 status, status & OTP_STATUS_DONE,
				 OTP_POLL_US, OTP_TIMEOUT_US);
	if (ret)
		goto out_stop;

	status = readl(otp->base + OTP_STATUS_REG);
	writel(status, otp->base + OTP_STATUS_REG);

	data = readl(otp->base + OTP_DATA_REG);

	writel(0, otp->base + OTP_ADDRESS_REG);

out_stop:
	control = readl(otp->base + OTP_CONTROL_REG);
	control &= ~OTP_CONTROL_SETUP;
	writel(control, otp->base + OTP_CONTROL_REG);
	stop_ret = readl_poll_timeout(otp->base + OTP_STATUS_REG,
				      status, !(status & OTP_STATUS_READY),
				      OTP_POLL_US, OTP_TIMEOUT_US);
	if (ret || stop_ret) {
		otp->fault = true;
		return ret ?: stop_ret;
	}

	*word = data;
	return 0;
}

static int novatek_otp_read(void *context, unsigned int offset, void *val, size_t bytes)
{
	struct novatek_otp *otp = context;
	u8 *buf = val;
	__le32 data;
	u32 word;
	size_t count;
	int ret = 0;

	if (offset > otp->data->size || bytes > otp->data->size - offset)
		return -EINVAL;
	if (!bytes)
		return 0;
	if ((offset < 0x28 && offset + bytes > 0x20) || offset + bytes > 0x30)
		return -EACCES;

	mutex_lock(&otp->lock);
	if (otp->fault) {
		ret = -EIO;
		goto out_unlock;
	}

	while (bytes) {
		ret = novatek_otp_read_word(otp, offset / sizeof(word), &word);
		if (ret)
			break;

		data = cpu_to_le32(word);
		count = min(bytes, sizeof(data) - offset % sizeof(data));
		memcpy(buf, (u8 *)&data + offset % sizeof(data), count);
		buf += count;
		offset += count;
		bytes -= count;
	}

out_unlock:
	mutex_unlock(&otp->lock);
	return ret;
}

static int novatek_otp_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct nvmem_config config = {};
	struct reset_control *reset;
	struct nvmem_device *nvmem;
	struct novatek_otp *otp;
	struct clk *clk;
	int ret;

	otp = devm_kzalloc(dev, sizeof(*otp), GFP_KERNEL);
	if (!otp)
		return -ENOMEM;

	otp->data = device_get_match_data(dev);
	if (!otp->data)
		return -ENODEV;

	otp->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(otp->base))
		return PTR_ERR(otp->base);

	reset = devm_reset_control_get_exclusive(dev, NULL);
	if (IS_ERR(reset))
		return dev_err_probe(dev, PTR_ERR(reset), "Failed to get reset\n");

	ret = reset_control_status(reset);
	if (ret < 0)
		return dev_err_probe(dev, ret, "Failed to read reset status\n");
	if (ret)
		return dev_err_probe(dev, -EBUSY, "OTP is held in reset\n");

	clk = devm_clk_get_enabled(dev, "otp");
	if (IS_ERR(clk))
		return dev_err_probe(dev, PTR_ERR(clk), "Failed to enable clock\n");

	clk = devm_clk_get_enabled(dev, "apb");
	if (IS_ERR(clk))
		return dev_err_probe(dev, PTR_ERR(clk), "Failed to get APB clock\n");

	switch (clk_get_rate(clk)) {
	case 48000000:
		otp->timing = OTP_TIMING_48M;
		break;
	case 60000000:
		otp->timing = OTP_TIMING_60M;
		break;
	case 80000000:
		otp->timing = OTP_TIMING_80M;
		break;
	case 120000000:
		otp->timing = OTP_TIMING_120M;
		break;
	default:
		return dev_err_probe(dev, -EINVAL, "Unsupported APB rate\n");
	}

	mutex_init(&otp->lock);

	config.name = "novatek-otp";
	config.dev = dev;
	config.owner = THIS_MODULE;
	config.priv = otp;
	config.type = NVMEM_TYPE_OTP;
	config.read_only = true;
	config.root_only = true;
	config.stride = 1;
	config.word_size = 1;
	config.size = otp->data->size;

	config.reg_read = novatek_otp_read;

	config.keepout = na51089_otp_keepouts;
	config.nkeepout = ARRAY_SIZE(na51089_otp_keepouts);

	nvmem = devm_nvmem_register(dev, &config);
	return PTR_ERR_OR_ZERO(nvmem);
}

static const struct novatek_otp_soc_data na51089_otp_data = {
	.size = 128,
};

static const struct of_device_id novatek_otp_of_match[] = {
	{
		.compatible = "novatek,na51089-otp",
		.data = &na51089_otp_data,
	},
	{ }
};
MODULE_DEVICE_TABLE(of, novatek_otp_of_match);

static struct platform_driver novatek_otp_driver = {
	.probe = novatek_otp_probe,
	.driver = {
		.name = "novatek-otp",
		.of_match_table = novatek_otp_of_match,
	},
};
module_platform_driver(novatek_otp_driver);

MODULE_AUTHOR("Yu-Tung Chang <mtwget@gmail.com>");
MODULE_DESCRIPTION("Novatek OTP driver");
MODULE_LICENSE("GPL");
