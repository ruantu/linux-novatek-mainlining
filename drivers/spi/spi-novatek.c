// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2026 Yudong Zhang <mtwget@gmail.com>
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/compiler.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/spi/spi.h>
#include <linux/unaligned.h>

#define SPI_CTRL			0x00
#define SPI_CTRL_EN_MASK		BIT(0)
#define SPI_CTRL_DISABLE		0x0
#define SPI_CTRL_ENABLE			0x1
#define SPI_CTRL_CS_LEVEL_MASK		BIT(1)
#define SPI_CTRL_CS_LOW			0x0
#define SPI_CTRL_CS_HIGH		0x1

#define SPI_IO				0x04
#define SPI_IO_CPHA_MASK		BIT(2)
#define SPI_IO_CPHA_LEADING		0x0
#define SPI_IO_CPHA_TRAILING		0x1
#define SPI_IO_CPOL_MASK		BIT(3)
#define SPI_IO_CPOL_LOW			0x0
#define SPI_IO_CPOL_HIGH		0x1
#define SPI_IO_OUT_MASK			BIT(12)
#define SPI_IO_OUT_DISABLE		0x0
#define SPI_IO_OUT_ENABLE		0x1
#define SPI_IO_ORDER_MASK		BIT(13)
#define SPI_IO_ORDER_DESCENDING		0x0
#define SPI_IO_ORDER_ASCENDING		0x1
#define SPI_IO_AUTO_OUT_MASK		BIT(16)
#define SPI_IO_AUTO_OUT_DISABLE		0x0
#define SPI_IO_AUTO_OUT_ENABLE		0x1
#define SPI_IO_DEFAULT			\
	(FIELD_PREP(SPI_IO_CPHA_MASK, SPI_IO_CPHA_LEADING) | \
	 FIELD_PREP(SPI_IO_CPOL_MASK, SPI_IO_CPOL_LOW) | \
	 FIELD_PREP(SPI_IO_OUT_MASK, SPI_IO_OUT_ENABLE) | \
	 FIELD_PREP(SPI_IO_ORDER_MASK, SPI_IO_ORDER_ASCENDING) | \
	 FIELD_PREP(SPI_IO_AUTO_OUT_MASK, SPI_IO_AUTO_OUT_ENABLE))

#define SPI_CONFIG			0x08
#define SPI_CONFIG_BIT_ORDER_MASK	BIT(0)
#define SPI_CONFIG_MSB_FIRST		0x0
#define SPI_CONFIG_LSB_FIRST		0x1
#define SPI_CONFIG_WORD_SIZE_MASK	BIT(2)
#define SPI_CONFIG_WORD_8BIT		0x0
#define SPI_CONFIG_WORD_16BIT		0x1
#define SPI_CONFIG_PACKET_COUNT_MASK	GENMASK(5, 4)
#define SPI_PACKET_COUNT_1		0x0
#define SPI_PACKET_COUNT_2		0x1
#define SPI_PACKET_COUNT_4		0x3
#define SPI_CONFIG_DEFAULT		\
	(FIELD_PREP(SPI_CONFIG_BIT_ORDER_MASK, SPI_CONFIG_MSB_FIRST) | \
	 FIELD_PREP(SPI_CONFIG_WORD_SIZE_MASK, SPI_CONFIG_WORD_8BIT) | \
	 FIELD_PREP(SPI_CONFIG_PACKET_COUNT_MASK, SPI_PACKET_COUNT_1))

#define SPI_TIMING			0x0c
#define SPI_TIMING_CS_DELAY_MASK	GENMASK(7, 0)
#define SPI_TIMING_POST_DELAY_MASK	GENMASK(28, 16)
#define SPI_TIMING_DEFAULT		\
	(FIELD_PREP(SPI_TIMING_CS_DELAY_MASK, 0x0) | \
	 FIELD_PREP(SPI_TIMING_POST_DELAY_MASK, 0x0))

#define SPI_DLY_CHAIN			0x14
#define SPI_DLY_LATCH_SHIFT_MASK	GENMASK(17, 16)
#define SPI_DLY_LATCH_NORMAL		0x0
#define SPI_DLY_LATCH_1T		0x1
#define SPI_DLY_LATCH_2T		0x2
#define SPI_DLY_LATCH_EDGE_MASK		BIT(20)
#define SPI_DLY_LATCH_RISING		0x0
#define SPI_DLY_LATCH_FALLING		0x1

#define SPI_STATUS			0x18
#define SPI_STATUS_TDR_EMPTY_MASK	BIT(0)
#define SPI_STATUS_TDR_NOT_EMPTY	0x0
#define SPI_STATUS_TDR_EMPTY		0x1
#define SPI_STATUS_RDR_FULL_MASK	BIT(1)
#define SPI_STATUS_RDR_NOT_FULL		0x0
#define SPI_STATUS_RDR_FULL		0x1

#define SPI_INTEN			0x1c
#define SPI_INTEN_TDR_EMPTY_MASK	BIT(0)
#define SPI_INTEN_TDR_EMPTY_DISABLE	0x0
#define SPI_INTEN_TDR_EMPTY_ENABLE	0x1
#define SPI_INTEN_RDR_FULL_MASK		BIT(1)
#define SPI_INTEN_RDR_FULL_DISABLE	0x0
#define SPI_INTEN_RDR_FULL_ENABLE	0x1
#define SPI_INTEN_DMA_ABORT_MASK	BIT(2)
#define SPI_INTEN_DMA_ABORT_DISABLE	0x0
#define SPI_INTEN_DMA_ABORT_ENABLE	0x1
#define SPI_INTEN_DMA_DONE_MASK		BIT(3)
#define SPI_INTEN_DMA_DONE_DISABLE	0x0
#define SPI_INTEN_DMA_DONE_ENABLE	0x1
#define SPI_INTEN_RDSTS_DONE_MASK	BIT(4)
#define SPI_INTEN_RDSTS_DONE_DISABLE	0x0
#define SPI_INTEN_RDSTS_DONE_ENABLE	0x1
#define SPI_INTEN_GYRO_READY_MASK	BIT(5)
#define SPI_INTEN_GYRO_READY_DISABLE	0x0
#define SPI_INTEN_GYRO_READY_ENABLE	0x1
#define SPI_INTEN_GYRO_OVERRUN_MASK	BIT(6)
#define SPI_INTEN_GYRO_OVERRUN_DISABLE	0x0
#define SPI_INTEN_GYRO_OVERRUN_ENABLE	0x1
#define SPI_INTEN_GYRO_DONE_MASK	BIT(7)
#define SPI_INTEN_GYRO_DONE_DISABLE	0x0
#define SPI_INTEN_GYRO_DONE_ENABLE	0x1
#define SPI_INTEN_GYRO_SEQ_ERR_MASK	BIT(8)
#define SPI_INTEN_GYRO_SEQ_ERR_DISABLE	0x0
#define SPI_INTEN_GYRO_SEQ_ERR_ENABLE	0x1
#define SPI_INTEN_GYRO_TIMEOUT_MASK	BIT(9)
#define SPI_INTEN_GYRO_TIMEOUT_DISABLE	0x0
#define SPI_INTEN_GYRO_TIMEOUT_ENABLE	0x1
#define SPI_INTEN_DISABLE_ALL		\
	(FIELD_PREP(SPI_INTEN_TDR_EMPTY_MASK, SPI_INTEN_TDR_EMPTY_DISABLE) | \
	 FIELD_PREP(SPI_INTEN_RDR_FULL_MASK, SPI_INTEN_RDR_FULL_DISABLE) | \
	 FIELD_PREP(SPI_INTEN_DMA_ABORT_MASK, SPI_INTEN_DMA_ABORT_DISABLE) | \
	 FIELD_PREP(SPI_INTEN_DMA_DONE_MASK, SPI_INTEN_DMA_DONE_DISABLE) | \
	 FIELD_PREP(SPI_INTEN_RDSTS_DONE_MASK, SPI_INTEN_RDSTS_DONE_DISABLE) | \
	 FIELD_PREP(SPI_INTEN_GYRO_READY_MASK, SPI_INTEN_GYRO_READY_DISABLE) | \
	 FIELD_PREP(SPI_INTEN_GYRO_OVERRUN_MASK, \
		    SPI_INTEN_GYRO_OVERRUN_DISABLE) | \
	 FIELD_PREP(SPI_INTEN_GYRO_DONE_MASK, SPI_INTEN_GYRO_DONE_DISABLE) | \
	 FIELD_PREP(SPI_INTEN_GYRO_SEQ_ERR_MASK, \
		    SPI_INTEN_GYRO_SEQ_ERR_DISABLE) | \
	 FIELD_PREP(SPI_INTEN_GYRO_TIMEOUT_MASK, \
		    SPI_INTEN_GYRO_TIMEOUT_DISABLE))

#define SPI_RDR				0x20

#define SPI_TDR				0x24

#define SPI_MIN_SPEED_HZ		100000
#define SPI_MAX_SPEED_HZ		48000000

#define SPI_POLL_DELAY_US		1
#define SPI_POLL_TIMEOUT_US		100000
#define SPI_REG_TIMEOUT_US		1000

struct novatek_spi {
	struct device *dev;
	void __iomem *base;
	struct clk *clk;
	struct reset_control *reset;
	bool fault;
};

static int novatek_spi_hw_init(struct novatek_spi *nspi, bool cs_level)
{
	u32 mask = SPI_CTRL_EN_MASK | SPI_CTRL_CS_LEVEL_MASK;
	u32 control = FIELD_PREP(SPI_CTRL_EN_MASK, SPI_CTRL_ENABLE);
	u32 value;

	if (cs_level)
		control |= FIELD_PREP(SPI_CTRL_CS_LEVEL_MASK, SPI_CTRL_CS_HIGH);

	writel(SPI_INTEN_DISABLE_ALL, nspi->base + SPI_INTEN);

	writel(SPI_IO_DEFAULT, nspi->base + SPI_IO);

	writel(SPI_TIMING_DEFAULT, nspi->base + SPI_TIMING);
	writel(FIELD_PREP(SPI_DLY_LATCH_EDGE_MASK, SPI_DLY_LATCH_FALLING),
	       nspi->base + SPI_DLY_CHAIN);

	writel(SPI_CONFIG_DEFAULT, nspi->base + SPI_CONFIG);

	writel(control, nspi->base + SPI_CTRL);
	return readl_poll_timeout_atomic(nspi->base + SPI_CTRL,
					 value,
					 (value & mask) == control,
					 0, SPI_REG_TIMEOUT_US);
}

static int novatek_spi_set_cs_level(struct novatek_spi *nspi, bool level)
{
	u32 mask = SPI_CTRL_CS_LEVEL_MASK;
	u32 cs_level;
	u32 data;
	u32 value;

	cs_level = FIELD_PREP(SPI_CTRL_CS_LEVEL_MASK,
			      level ? SPI_CTRL_CS_HIGH :
				      SPI_CTRL_CS_LOW);
	data = readl(nspi->base + SPI_CTRL);
	data &= ~mask;
	data |= cs_level & mask;
	writel(data, nspi->base + SPI_CTRL);
	return readl_poll_timeout_atomic(nspi->base + SPI_CTRL,
					 value,
					 (value & mask) == cs_level,
					 0, SPI_REG_TIMEOUT_US);
}

static int novatek_spi_recover(struct novatek_spi *nspi,
			       struct spi_device *spi)
{
	bool inactive_level = !(spi->mode & SPI_CS_HIGH);
	int ret;

	WRITE_ONCE(nspi->fault, true);

	ret = reset_control_reset(nspi->reset);
	if (!ret)
		ret = novatek_spi_hw_init(nspi, inactive_level);
	if (!ret)
		WRITE_ONCE(nspi->fault, false);

	return ret;
}

static int novatek_spi_prepare_message(struct spi_controller *host,
				       struct spi_message *message)
{
	struct novatek_spi *nspi = spi_controller_get_devdata(host);
	struct spi_device *spi = message->spi;
	u32 delay = 0;
	u32 io = SPI_IO_DEFAULT;
	u32 config;
	int ret;

	if (READ_ONCE(nspi->fault)) {
		ret = novatek_spi_recover(nspi, spi);
		if (ret) {
			dev_err_ratelimited(nspi->dev,
					    "controller recovery failed: %d\n",
					    ret);
			return ret;
		}
	}

	switch (spi->mode & (SPI_CPOL | SPI_CPHA)) {
	case SPI_MODE_0:
		delay = FIELD_PREP(SPI_DLY_LATCH_EDGE_MASK,
				   SPI_DLY_LATCH_FALLING);
		break;
	case SPI_MODE_1:
		io |= FIELD_PREP(SPI_IO_CPHA_MASK, SPI_IO_CPHA_TRAILING);
		delay = FIELD_PREP(SPI_DLY_LATCH_SHIFT_MASK, SPI_DLY_LATCH_1T);
		break;
	case SPI_MODE_2:
		io |= FIELD_PREP(SPI_IO_CPOL_MASK, SPI_IO_CPOL_HIGH);
		delay = FIELD_PREP(SPI_DLY_LATCH_EDGE_MASK,
				   SPI_DLY_LATCH_FALLING);
		break;
	case SPI_MODE_3:
		io |= FIELD_PREP(SPI_IO_CPOL_MASK, SPI_IO_CPOL_HIGH) |
		      FIELD_PREP(SPI_IO_CPHA_MASK, SPI_IO_CPHA_TRAILING);
		delay = FIELD_PREP(SPI_DLY_LATCH_SHIFT_MASK, SPI_DLY_LATCH_1T);
		break;
	}
	writel(io, nspi->base + SPI_IO);
	writel(delay, nspi->base + SPI_DLY_CHAIN);

	config = readl(nspi->base + SPI_CONFIG);
	config &= ~(SPI_CONFIG_BIT_ORDER_MASK |
		    SPI_CONFIG_WORD_SIZE_MASK |
		    SPI_CONFIG_PACKET_COUNT_MASK);
	if (spi->mode & SPI_LSB_FIRST)
		config |= FIELD_PREP(SPI_CONFIG_BIT_ORDER_MASK,
				     SPI_CONFIG_LSB_FIRST);
	if (spi->bits_per_word == 16)
		config |= FIELD_PREP(SPI_CONFIG_WORD_SIZE_MASK,
				     SPI_CONFIG_WORD_16BIT);
	writel(config, nspi->base + SPI_CONFIG);

	return 0;
}

static void novatek_spi_set_cs(struct spi_device *spi, bool level)
{
	struct novatek_spi *nspi = spi_controller_get_devdata(spi->controller);
	int ret;

	ret = novatek_spi_set_cs_level(nspi, level);
	if (ret) {
		WRITE_ONCE(nspi->fault, true);
		dev_err_ratelimited(nspi->dev,
				    "timed out setting chip select %s\n",
				    level ? "high" : "low");
	}
}

static uint novatek_spi_packet_count(uint words, uint max_words)
{
	if (words >= max_words)
		return max_words;
	if (words >= 2)
		return 2;

	return 1;
}

static u32 novatek_spi_packet_encoding(uint packets)
{
	switch (packets) {
	case 1:
		return SPI_PACKET_COUNT_1;
	case 2:
		return SPI_PACKET_COUNT_2;
	default:
		return SPI_PACKET_COUNT_4;
	}
}

static u32 novatek_spi_pack(const u8 *tx, uint offset, uint packets,
			    uint bytes_per_word)
{
	u32 value = 0;
	uint i;

	if (!tx)
		return 0;

	for (i = 0; i < packets; i++) {
		const u8 *word = tx + offset + i * bytes_per_word;
		u32 word_value;

		if (bytes_per_word == 1)
			word_value = *word;
		else
			word_value = get_unaligned((const u16 *)word);
		value |= word_value << (i * bytes_per_word * 8);
	}

	return value;
}

static void novatek_spi_unpack(u8 *rx, uint offset, uint packets,
			       uint bytes_per_word, u32 value)
{
	uint i;

	if (!rx)
		return;

	for (i = 0; i < packets; i++) {
		u8 *word = rx + offset + i * bytes_per_word;
		u32 word_value = value >> (i * bytes_per_word * 8);

		if (bytes_per_word == 1)
			*word = word_value;
		else
			put_unaligned((u16)word_value, (u16 *)word);
	}
}

static int novatek_spi_set_rate(struct novatek_spi *nspi,
				struct spi_transfer *transfer)
{
	ulong rate;
	int ret;

	ret = clk_set_rate(nspi->clk, transfer->speed_hz);
	if (ret)
		return ret;

	rate = clk_get_rate(nspi->clk);
	if (!rate || rate > transfer->speed_hz)
		return -EINVAL;
	transfer->effective_speed_hz = rate;

	return 0;
}

static int novatek_spi_transfer_one(struct spi_controller *host,
				    struct spi_device *spi,
				    struct spi_transfer *transfer)
{
	struct novatek_spi *nspi = spi_controller_get_devdata(host);
	const u8 *tx = transfer->tx_buf;
	u8 *rx = transfer->rx_buf;
	uint bytes_per_word = transfer->bits_per_word / 8;
	uint words = transfer->len / bytes_per_word;
	uint max_packets = bytes_per_word == 1 ? 4 : 2;
	uint done = 0;
	u32 config;
	u32 value;
	int ret;

	if (READ_ONCE(nspi->fault))
		return -EIO;

	ret = novatek_spi_set_rate(nspi, transfer);
	if (ret)
		return ret;

	config = readl(nspi->base + SPI_CONFIG);
	config &= ~(SPI_CONFIG_WORD_SIZE_MASK |
		    SPI_CONFIG_PACKET_COUNT_MASK);
	if (bytes_per_word == 2)
		config |= FIELD_PREP(SPI_CONFIG_WORD_SIZE_MASK,
				     SPI_CONFIG_WORD_16BIT);

	while (done < words) {
		uint packets;

		if (transfer->word_delay.value)
			packets = 1;
		else
			packets = novatek_spi_packet_count(words - done,
							   max_packets);
		config &= ~SPI_CONFIG_PACKET_COUNT_MASK;
		config |= FIELD_PREP(SPI_CONFIG_PACKET_COUNT_MASK,
				     novatek_spi_packet_encoding(packets));
		writel(config, nspi->base + SPI_CONFIG);

		ret = readl_poll_timeout(nspi->base + SPI_STATUS,
					 value,
					 value & SPI_STATUS_TDR_EMPTY_MASK,
					 SPI_POLL_DELAY_US,
					 SPI_POLL_TIMEOUT_US);
		if (ret)
			goto out;

		value = novatek_spi_pack(tx, done * bytes_per_word,
					 packets, bytes_per_word);
		writel(value, nspi->base + SPI_TDR);

		ret = readl_poll_timeout(nspi->base + SPI_STATUS,
					 value,
					 value & SPI_STATUS_RDR_FULL_MASK,
					 SPI_POLL_DELAY_US,
					 SPI_POLL_TIMEOUT_US);
		if (ret)
			goto out;

		value = readl(nspi->base + SPI_RDR);
		novatek_spi_unpack(rx, done * bytes_per_word, packets,
				   bytes_per_word, value);

		done += packets;

		if (transfer->word_delay.value) {
			ret = spi_delay_exec(&transfer->word_delay, transfer);
			if (ret)
				goto out;
		}
	}

out:
	if (!ret) {
		config &= ~SPI_CONFIG_PACKET_COUNT_MASK;
		writel(config, nspi->base + SPI_CONFIG);
	}

	return ret;
}

static void novatek_spi_handle_err(struct spi_controller *host,
				   struct spi_message *message)
{
	struct novatek_spi *nspi = spi_controller_get_devdata(host);
	int ret;

	ret = novatek_spi_recover(nspi, message->spi);
	if (ret)
		dev_err_ratelimited(nspi->dev,
				    "failed to recover controller: %d\n", ret);
}

static const struct of_device_id novatek_spi_of_match[] = {
	{ .compatible = "novatek,na51089-spi" },
	{ }
};
MODULE_DEVICE_TABLE(of, novatek_spi_of_match);

static int novatek_spi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct spi_controller *host;
	struct novatek_spi *nspi;
	int ret;

	host = devm_spi_alloc_host(dev, sizeof(*nspi));
	if (!host)
		return -ENOMEM;
	nspi = spi_controller_get_devdata(host);
	nspi->dev = dev;

	nspi->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(nspi->base))
		return PTR_ERR(nspi->base);

	nspi->clk = devm_clk_get_enabled(dev, NULL);
	if (IS_ERR(nspi->clk))
		return dev_err_probe(dev, PTR_ERR(nspi->clk),
				     "failed to enable clock\n");

	nspi->reset = devm_reset_control_get_exclusive_deasserted(dev, NULL);
	if (IS_ERR(nspi->reset))
		return dev_err_probe(dev, PTR_ERR(nspi->reset),
				     "failed to deassert reset\n");
	ret = reset_control_reset(nspi->reset);
	if (ret)
		return dev_err_probe(dev, ret, "failed to reset controller\n");

	ret = novatek_spi_hw_init(nspi, true);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to initialize controller\n");

	host->mode_bits = SPI_CPOL | SPI_CPHA | SPI_CS_HIGH | SPI_LSB_FIRST;
	host->bits_per_word_mask = SPI_BPW_MASK(8) | SPI_BPW_MASK(16);

	host->min_speed_hz = SPI_MIN_SPEED_HZ;
	host->max_speed_hz = SPI_MAX_SPEED_HZ;

	host->num_chipselect = 1;
	host->max_native_cs = 1;
	host->use_gpio_descriptors = true;

	host->prepare_message = novatek_spi_prepare_message;
	host->set_cs = novatek_spi_set_cs;
	host->transfer_one = novatek_spi_transfer_one;
	host->handle_err = novatek_spi_handle_err;

	platform_set_drvdata(pdev, host);

	return devm_spi_register_controller(dev, host);
}

static struct platform_driver novatek_spi_driver = {
	.probe = novatek_spi_probe,
	.driver = {
		.name = "novatek-spi",
		.of_match_table = novatek_spi_of_match,
	},
};
module_platform_driver(novatek_spi_driver);

MODULE_DESCRIPTION("Novatek NA51089 SPI host controller driver");
MODULE_LICENSE("GPL");
