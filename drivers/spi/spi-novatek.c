// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2026 Yu-Tung Chang <mtwget@gmail.com>
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/compiler.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/jiffies.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/property.h>
#include <linux/reset.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/unaligned.h>

#define SPI_CTRL_REG					0x00
#define SPI_IO_REG					0x04
#define SPI_CONFIG_REG					0x08
#define SPI_TIMING_REG					0x0c
#define SPI_DLY_CHAIN_REG				0x14
#define SPI_STATUS_REG					0x18
#define SPI_INTEN_REG					0x1c
#define SPI_RDR_REG					0x20
#define SPI_TDR_REG					0x24
#define SPI_DMA_CTRL_REG				0x30
#define SPI_DMA_SIZE_REG				0x34
#define SPI_DMA_ADDR_REG				0x38

/* SPI_CTRL bit fields */
#define SPI_CTRL_EN					BIT(0)
#define SPI_CTRL_CS_LEVEL				BIT(1)
#define SPI_CTRL_DMA_EN					BIT(2)
#define SPI_CTRL_DMA_DIS				BIT(3)
#define SPI_CTRL_DMA_STS				BIT(16)

/* SPI_IO bit fields */
#define SPI_IO_CPHA					BIT(2)
#define SPI_IO_CPOL					BIT(3)
#define SPI_IO_OUT					BIT(12)
#define SPI_IO_ORDER					BIT(13)
#define SPI_IO_AUTO_OUT					BIT(16)
#define SPI_IO_DEFAULT			\
	(FIELD_PREP_CONST(SPI_IO_CPHA, 0) | \
	 FIELD_PREP_CONST(SPI_IO_CPOL, 0) | \
	 FIELD_PREP_CONST(SPI_IO_OUT, 1) | \
	 FIELD_PREP_CONST(SPI_IO_ORDER, 1) | \
	 FIELD_PREP_CONST(SPI_IO_AUTO_OUT, 1))

/* SPI_CONFIG bit fields */
#define SPI_CONFIG_BIT_ORDER				BIT(0)
#define SPI_CONFIG_WORD_SIZE				BIT(2)
#define SPI_CONFIG_PACKET_COUNT				GENMASK(5, 4)
#define SPI_CONFIG_PACKET_COUNT_1			0x0
#define SPI_CONFIG_PACKET_COUNT_2			0x1
#define SPI_CONFIG_PACKET_COUNT_4			0x3
#define SPI_CONFIG_DEFAULT		\
	(FIELD_PREP_CONST(SPI_CONFIG_BIT_ORDER, 0) | \
	 FIELD_PREP_CONST(SPI_CONFIG_WORD_SIZE, 0) | \
	 FIELD_PREP_CONST(SPI_CONFIG_PACKET_COUNT, SPI_CONFIG_PACKET_COUNT_1))

/* SPI_TIMING bit fields */
#define SPI_TIMING_CS_DELAY				GENMASK(7, 0)
#define SPI_TIMING_POST_DELAY				GENMASK(28, 16)
#define SPI_TIMING_DEFAULT		\
	(FIELD_PREP_CONST(SPI_TIMING_CS_DELAY, 0x00) | \
	 FIELD_PREP_CONST(SPI_TIMING_POST_DELAY, 0x0000))

/* SPI_DLY_CHAIN bit fields */
#define SPI_DLY_CHAIN_LATCH_SHIFT			GENMASK(17, 16)
#define SPI_DLY_CHAIN_LATCH_SHIFT_NORMAL		0x0
#define SPI_DLY_CHAIN_LATCH_SHIFT_1T			0x1
#define SPI_DLY_CHAIN_LATCH_SHIFT_2T			0x2
#define SPI_DLY_CHAIN_LATCH_EDGE			BIT(20)
#define SPI_DLY_CHAIN_DEFAULT \
	(FIELD_PREP_CONST(SPI_DLY_CHAIN_LATCH_SHIFT, SPI_DLY_CHAIN_LATCH_SHIFT_NORMAL) | \
	 FIELD_PREP_CONST(SPI_DLY_CHAIN_LATCH_EDGE, 1))

/* SPI_STATUS bit fields */
#define SPI_STATUS_TDR_EMPTY				BIT(0)
#define SPI_STATUS_RDR_FULL				BIT(1)
#define SPI_STATUS_DMA_ABORT				BIT(2)
#define SPI_STATUS_DMA_DONE				BIT(3)

/* SPI_INTEN bit fields */
#define SPI_INTEN_RDR_FULL				BIT(1)
#define SPI_INTEN_DMA_DONE				BIT(3)
#define SPI_INTEN_RDSTS_DONE				BIT(4)
#define SPI_INTEN_GYRO_READY				BIT(5)
#define SPI_INTEN_GYRO_OVERRUN				BIT(6)
#define SPI_INTEN_GYRO_DONE				BIT(7)
#define SPI_INTEN_GYRO_SEQ_ERR				BIT(8)
#define SPI_INTEN_GYRO_TIMEOUT				BIT(9)
#define SPI_INTEN_DISABLE_ALL		\
	(FIELD_PREP_CONST(SPI_INTEN_RDR_FULL, 0) | \
	 FIELD_PREP_CONST(SPI_INTEN_DMA_DONE, 0) | \
	 FIELD_PREP_CONST(SPI_INTEN_RDSTS_DONE, 0) | \
	 FIELD_PREP_CONST(SPI_INTEN_GYRO_READY, 0) | \
	 FIELD_PREP_CONST(SPI_INTEN_GYRO_OVERRUN, 0) | \
	 FIELD_PREP_CONST(SPI_INTEN_GYRO_DONE, 0) | \
	 FIELD_PREP_CONST(SPI_INTEN_GYRO_SEQ_ERR, 0) | \
	 FIELD_PREP_CONST(SPI_INTEN_GYRO_TIMEOUT, 0))
#define SPI_INTEN_DMA_DONE_ONLY \
	(FIELD_PREP_CONST(SPI_INTEN_RDR_FULL, 0) | \
	 FIELD_PREP_CONST(SPI_INTEN_DMA_DONE, 1) | \
	 FIELD_PREP_CONST(SPI_INTEN_RDSTS_DONE, 0) | \
	 FIELD_PREP_CONST(SPI_INTEN_GYRO_READY, 0) | \
	 FIELD_PREP_CONST(SPI_INTEN_GYRO_OVERRUN, 0) | \
	 FIELD_PREP_CONST(SPI_INTEN_GYRO_DONE, 0) | \
	 FIELD_PREP_CONST(SPI_INTEN_GYRO_SEQ_ERR, 0) | \
	 FIELD_PREP_CONST(SPI_INTEN_GYRO_TIMEOUT, 0))

/* SPI_DMA_CTRL bit fields */
#define SPI_DMA_CTRL_DIR				BIT(0)

/* SPI_DMA_SIZE bit fields */
#define SPI_DMA_SIZE_COUNT				GENMASK(23, 0)

#define SPI_DMA_ALIGN					4
#define SPI_DMA_MIN_BYTES				16

#define SPI_DMA_TIMEOUT_MARGIN_MS			200
#define SPI_RESET_DELAY_US				10
#define SPI_POLL_DELAY_US				1
#define SPI_POLL_TIMEOUT_US				100000
#define SPI_REG_TIMEOUT_US				1000

#define SPI_MIN_SPEED_HZ				100000
#define SPI_MAX_SPEED_HZ				48000000

struct novatek_spi {
	struct device *dev;
	void __iomem *base;
	struct clk *clk;
	struct reset_control *reset;
	struct completion complete;
	void *dma_buf;
	dma_addr_t dma_addr;
	enum dma_data_direction dma_dir;
	uint dma_len;
	u32 dma_status;
	int irq;
	bool dma_enabled;
	bool dma_active;
	bool clock_enabled;
	bool reset_asserted;
	bool cs_level;
	bool fault;
};

static void novatek_spi_mask_irq(struct novatek_spi *nspi)
{
	if (nspi->clock_enabled && !nspi->reset_asserted) {
		writel(SPI_INTEN_DISABLE_ALL, nspi->base + SPI_INTEN_REG);
		readl(nspi->base + SPI_INTEN_REG);
	}
	if (nspi->irq >= 0)
		synchronize_irq(nspi->irq);
}

static int novatek_spi_hw_init(struct novatek_spi *nspi, bool cs_level)
{
	u32 mask = SPI_CTRL_EN | SPI_CTRL_CS_LEVEL;
	u32 control;
	u32 value;
	int ret;

	control = SPI_CTRL_EN;
	if (cs_level)
		control |= SPI_CTRL_CS_LEVEL;

	writel(SPI_INTEN_DISABLE_ALL, nspi->base + SPI_INTEN_REG);

	writel(SPI_IO_DEFAULT, nspi->base + SPI_IO_REG);

	writel(SPI_TIMING_DEFAULT, nspi->base + SPI_TIMING_REG);
	writel(SPI_DLY_CHAIN_DEFAULT, nspi->base + SPI_DLY_CHAIN_REG);

	writel(SPI_CONFIG_DEFAULT, nspi->base + SPI_CONFIG_REG);

	writel(control, nspi->base + SPI_CTRL_REG);
	ret = readl_poll_timeout_atomic(nspi->base + SPI_CTRL_REG,
					value, (value & mask) == control,
					0, SPI_REG_TIMEOUT_US);
	if (!ret)
		nspi->cs_level = cs_level;

	return ret;
}

static int novatek_spi_set_cs_level(struct novatek_spi *nspi, bool level)
{
	u32 mask = SPI_CTRL_CS_LEVEL;
	ulong rate;
	u32 cs_level;
	u32 data;
	u32 value;
	int ret;

	rate = clk_get_rate(nspi->clk);
	if (!rate)
		return -EINVAL;

	cs_level = (level ? SPI_CTRL_CS_LEVEL : 0);
	data = readl(nspi->base + SPI_CTRL_REG);
	data &= SPI_CTRL_EN;
	data |= cs_level & mask;
	writel(data, nspi->base + SPI_CTRL_REG);
	ret = readl_poll_timeout_atomic(nspi->base + SPI_CTRL_REG,
					value, (value & mask) == cs_level,
					0, SPI_REG_TIMEOUT_US);
	if (!ret) {
		/* Readback can complete before the CS output changes. */
		udelay(DIV_ROUND_UP(USEC_PER_SEC, rate));
		nspi->cs_level = level;
	}

	return ret;
}

static void novatek_spi_release_dma(struct novatek_spi *nspi, u8 *rx, uint len)
{
	if (!nspi->dma_buf || nspi->dma_active)
		return;

	dma_unmap_single(nspi->dev, nspi->dma_addr, nspi->dma_len, nspi->dma_dir);
	if (rx)
		memcpy(rx, nspi->dma_buf, len);

	kfree(nspi->dma_buf);
	nspi->dma_buf = NULL;
}

static int novatek_spi_recover(struct novatek_spi *nspi, bool cs_level)
{
	int ret;

	WRITE_ONCE(nspi->fault, true);
	novatek_spi_mask_irq(nspi);
	if (!nspi->reset_asserted) {
		ret = reset_control_assert(nspi->reset);
		if (ret)
			return ret;
		nspi->reset_asserted = true;
		udelay(SPI_RESET_DELAY_US);
	}

	nspi->dma_active = false;
	novatek_spi_release_dma(nspi, NULL, 0);

	ret = reset_control_deassert(nspi->reset);
	if (ret)
		return ret;
	nspi->reset_asserted = false;

	ret = novatek_spi_hw_init(nspi, cs_level);
	if (!ret)
		WRITE_ONCE(nspi->fault, false);

	return ret;
}

static int novatek_spi_prepare_message(struct spi_controller *host, struct spi_message *message)
{
	struct novatek_spi *nspi = spi_controller_get_devdata(host);
	struct spi_device *spi = message->spi;
	u32 delay = 0;
	u32 io = SPI_IO_DEFAULT;
	u32 config;
	int ret;

	if (READ_ONCE(nspi->fault)) {
		ret = novatek_spi_recover(nspi, !(spi->mode & SPI_CS_HIGH));
		if (ret) {
			dev_err_ratelimited(nspi->dev, "controller recovery failed: %d\n", ret);
			return ret;
		}
	}

	switch (spi->mode & (SPI_CPOL | SPI_CPHA)) {
	case SPI_MODE_0:
		delay = SPI_DLY_CHAIN_LATCH_EDGE;
		break;
	case SPI_MODE_1:
		io |= SPI_IO_CPHA;
		delay = FIELD_PREP(SPI_DLY_CHAIN_LATCH_SHIFT, SPI_DLY_CHAIN_LATCH_SHIFT_1T);
		break;
	case SPI_MODE_2:
		io |= SPI_IO_CPOL;
		delay = SPI_DLY_CHAIN_LATCH_EDGE;
		break;
	case SPI_MODE_3:
		io |= SPI_IO_CPOL | SPI_IO_CPHA;
		delay = FIELD_PREP(SPI_DLY_CHAIN_LATCH_SHIFT, SPI_DLY_CHAIN_LATCH_SHIFT_1T);
		break;
	}
	writel(io, nspi->base + SPI_IO_REG);
	writel(delay, nspi->base + SPI_DLY_CHAIN_REG);

	config = readl(nspi->base + SPI_CONFIG_REG);
	config &= ~(SPI_CONFIG_BIT_ORDER | SPI_CONFIG_WORD_SIZE | SPI_CONFIG_PACKET_COUNT);
	if (spi->mode & SPI_LSB_FIRST)
		config |= SPI_CONFIG_BIT_ORDER;
	if (spi->bits_per_word == 16)
		config |= SPI_CONFIG_WORD_SIZE;
	writel(config, nspi->base + SPI_CONFIG_REG);

	return 0;
}

static void novatek_spi_set_cs(struct spi_device *spi, bool level)
{
	struct novatek_spi *nspi = spi_controller_get_devdata(spi->controller);
	int ret;

	if (nspi->reset_asserted) {
		nspi->cs_level = level;
		return;
	}

	ret = novatek_spi_set_cs_level(nspi, level);
	if (ret) {
		WRITE_ONCE(nspi->fault, true);
		dev_err_ratelimited(nspi->dev,
				    "failed to set chip select %s: %d\n",
				    level ? "high" : "low", ret);
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
		return SPI_CONFIG_PACKET_COUNT_1;
	case 2:
		return SPI_CONFIG_PACKET_COUNT_2;
	default:
		return SPI_CONFIG_PACKET_COUNT_4;
	}
}

static u32 novatek_spi_pack(const u8 *tx, uint offset, uint packets, uint bytes_per_word)
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

static void novatek_spi_unpack(u8 *rx, uint offset, uint packets, uint bytes_per_word, u32 value)
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

static int novatek_spi_set_rate(struct novatek_spi *nspi, struct spi_transfer *transfer)
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

static void novatek_spi_prepare_dma(struct novatek_spi *nspi, struct spi_transfer *transfer)
{
	enum dma_data_direction dir;
	dma_addr_t addr;
	void *buf;
	uint len;

	if (!nspi->dma_enabled || transfer->word_delay.value ||
	    (transfer->tx_buf && transfer->rx_buf) ||
	    transfer->len < SPI_DMA_MIN_BYTES)
		return;

	len = round_down(transfer->len, SPI_DMA_ALIGN);
	len = min_t(uint, len, PAGE_SIZE);
	buf = kmalloc(len, GFP_KERNEL);
	if (!buf)
		return;
	if (transfer->tx_buf)
		memcpy(buf, transfer->tx_buf, len);

	dir = transfer->tx_buf ? DMA_TO_DEVICE : DMA_FROM_DEVICE;
	addr = dma_map_single(nspi->dev, buf, len, dir);
	if (dma_mapping_error(nspi->dev, addr))
		goto out_free_buf;
	if (!IS_ALIGNED(addr, SPI_DMA_ALIGN)) {
		dma_unmap_single(nspi->dev, addr, len, dir);
		goto out_free_buf;
	}
	nspi->dma_buf = buf;
	nspi->dma_addr = addr;
	nspi->dma_len = len;
	nspi->dma_dir = dir;

	return;

out_free_buf:
	kfree(buf);
}

static int novatek_spi_stop_dma(struct novatek_spi *nspi)
{
	u32 control;
	u32 value;
	int ret;

	if (!nspi->dma_active)
		return 0;

	novatek_spi_mask_irq(nspi);
	control = readl(nspi->base + SPI_CTRL_REG);
	control &= SPI_CTRL_EN | SPI_CTRL_CS_LEVEL;
	control |= SPI_CTRL_DMA_DIS;
	writel(control, nspi->base + SPI_CTRL_REG);

	ret = readl_poll_timeout(nspi->base + SPI_CTRL_REG,
				 value, !(value & SPI_CTRL_DMA_STS),
				 SPI_POLL_DELAY_US, SPI_POLL_TIMEOUT_US);
	if (ret) {
		WRITE_ONCE(nspi->fault, true);
		ret = reset_control_assert(nspi->reset);
		if (ret) {
			dev_err(nspi->dev, "DMA not stopped, retaining buffer\n");
			return ret;
		}
		nspi->reset_asserted = true;
		udelay(SPI_RESET_DELAY_US);
	}
	nspi->dma_active = false;

	return 0;
}

static irqreturn_t novatek_spi_irq(int irq, void *data)
{
	struct novatek_spi *nspi = data;
	u32 status = readl(nspi->base + SPI_STATUS_REG);
	u32 enabled = readl(nspi->base + SPI_INTEN_REG);

	if (!(status & enabled & SPI_STATUS_DMA_DONE))
		return IRQ_NONE;

	writel(SPI_INTEN_DISABLE_ALL, nspi->base + SPI_INTEN_REG);
	writel(status & (SPI_STATUS_DMA_ABORT | SPI_STATUS_DMA_DONE), nspi->base + SPI_STATUS_REG);
	nspi->dma_status = status;
	complete(&nspi->complete);

	return IRQ_HANDLED;
}

static int novatek_spi_dma_transfer(struct novatek_spi *nspi, struct spi_transfer *transfer,
				    u32 config)
{
	const u8 *tx = transfer->tx_buf;
	u8 *rx = transfer->rx_buf;
	uint body = round_down(transfer->len, SPI_DMA_ALIGN);
	uint done = 0;
	uint len = 0;
	uint timeout;
	ulong completed;
	u32 control;
	u32 value;
	int ret;

	config |= FIELD_PREP(SPI_CONFIG_PACKET_COUNT, transfer->bits_per_word == 8 ? SPI_CONFIG_PACKET_COUNT_4 : SPI_CONFIG_PACKET_COUNT_2);
	writel(config, nspi->base + SPI_CONFIG_REG);

	value = tx ? SPI_DMA_CTRL_DIR : 0;
	writel(value, nspi->base + SPI_DMA_CTRL_REG);

	while (done < body) {
		len = min(body - done, nspi->dma_len);
		reinit_completion(&nspi->complete);
		nspi->dma_status = 0;
		writel(SPI_STATUS_DMA_ABORT | SPI_STATUS_DMA_DONE, nspi->base + SPI_STATUS_REG);

		writel(FIELD_PREP(SPI_DMA_SIZE_COUNT, len), nspi->base + SPI_DMA_SIZE_REG);
		writel(lower_32_bits(nspi->dma_addr), nspi->base + SPI_DMA_ADDR_REG);

		writel(SPI_INTEN_DMA_DONE_ONLY, nspi->base + SPI_INTEN_REG);

		control = readl(nspi->base + SPI_CTRL_REG);
		control &= SPI_CTRL_EN | SPI_CTRL_CS_LEVEL;
		control |= SPI_CTRL_DMA_EN;
		nspi->dma_active = true;
		writel(control, nspi->base + SPI_CTRL_REG);

		timeout = DIV_ROUND_UP_ULL((u64)len * 8 * 2 * MSEC_PER_SEC,
					   transfer->effective_speed_hz);
		timeout += SPI_DMA_TIMEOUT_MARGIN_MS;
		completed = wait_for_completion_timeout(&nspi->complete, msecs_to_jiffies(timeout));
		novatek_spi_mask_irq(nspi);
		if (!completed) {
			ret = -ETIMEDOUT;
			goto out_stop_dma;
		}
		if (nspi->dma_status & SPI_STATUS_DMA_ABORT) {
			ret = -EIO;
			goto out_stop_dma;
		}

		ret = readl_poll_timeout(nspi->base + SPI_CTRL_REG,
					 value, !(value & SPI_CTRL_DMA_STS),
					 SPI_POLL_DELAY_US, SPI_REG_TIMEOUT_US);
		if (ret)
			goto out_stop_dma;
		nspi->dma_active = false;

		done += len;
		if (done == body)
			break;

		dma_sync_single_for_cpu(nspi->dev, nspi->dma_addr, nspi->dma_len, nspi->dma_dir);
		if (rx)
			memcpy(rx + done - len, nspi->dma_buf, len);
		else
			memcpy(nspi->dma_buf, tx + done, min(body - done, nspi->dma_len));
		dma_sync_single_for_device(nspi->dev, nspi->dma_addr, nspi->dma_len, nspi->dma_dir);
	}

	novatek_spi_release_dma(nspi, rx ? rx + done - len : NULL, len);

	return 0;

out_stop_dma:
	novatek_spi_stop_dma(nspi);
	novatek_spi_release_dma(nspi, NULL, 0);
	return ret;
}

static int novatek_spi_transfer_one(struct spi_controller *host, struct spi_device *spi,
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

	config = readl(nspi->base + SPI_CONFIG_REG);
	config &= ~(SPI_CONFIG_WORD_SIZE | SPI_CONFIG_PACKET_COUNT);
	if (bytes_per_word == 2)
		config |= SPI_CONFIG_WORD_SIZE;

	novatek_spi_prepare_dma(nspi, transfer);
	if (nspi->dma_buf) {
		ret = novatek_spi_dma_transfer(nspi, transfer, config);
		if (ret)
			return ret;
		done = round_down(transfer->len, SPI_DMA_ALIGN);
		done /= bytes_per_word;
	}

	while (done < words) {
		uint packets;

		if (transfer->word_delay.value)
			packets = 1;
		else
			packets = novatek_spi_packet_count(words - done, max_packets);
		config &= ~SPI_CONFIG_PACKET_COUNT;
		config |= FIELD_PREP(SPI_CONFIG_PACKET_COUNT, novatek_spi_packet_encoding(packets));
		writel(config, nspi->base + SPI_CONFIG_REG);

		ret = readl_poll_timeout(nspi->base + SPI_STATUS_REG,
					 value, value & SPI_STATUS_TDR_EMPTY,
					 SPI_POLL_DELAY_US, SPI_POLL_TIMEOUT_US);
		if (ret)
			return ret;

		value = novatek_spi_pack(tx, done * bytes_per_word, packets, bytes_per_word);
		writel(value, nspi->base + SPI_TDR_REG);

		ret = readl_poll_timeout(nspi->base + SPI_STATUS_REG,
					 value, value & SPI_STATUS_RDR_FULL,
					 SPI_POLL_DELAY_US, SPI_POLL_TIMEOUT_US);
		if (ret)
			return ret;

		value = readl(nspi->base + SPI_RDR_REG);
		novatek_spi_unpack(rx, done * bytes_per_word, packets, bytes_per_word, value);

		done += packets;

		if (transfer->word_delay.value) {
			ret = spi_delay_exec(&transfer->word_delay, transfer);
			if (ret)
				return ret;
		}
	}

	config &= ~SPI_CONFIG_PACKET_COUNT;
	writel(config, nspi->base + SPI_CONFIG_REG);

	return ret;
}

static void novatek_spi_handle_err(struct spi_controller *host, struct spi_message *message)
{
	struct novatek_spi *nspi = spi_controller_get_devdata(host);
	int ret;

	if (nspi->dma_active)
		return;

	ret = novatek_spi_recover(nspi, !(message->spi->mode & SPI_CS_HIGH));
	if (ret)
		dev_err_ratelimited(nspi->dev, "failed to recover controller: %d\n", ret);
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
	struct resource *res;
	int irq = -1;
	int ret;

	host = devm_spi_alloc_host(dev, sizeof(*nspi));
	if (!host)
		return -ENOMEM;
	nspi = spi_controller_get_devdata(host);
	nspi->dev = dev;
	nspi->irq = -1;
	init_completion(&nspi->complete);

	nspi->base = devm_platform_get_and_ioremap_resource(pdev, 0, &res);
	if (IS_ERR(nspi->base))
		return PTR_ERR(nspi->base);

	nspi->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(nspi->clk))
		return dev_err_probe(dev, PTR_ERR(nspi->clk), "failed to get clock\n");

	nspi->reset = devm_reset_control_get_exclusive(dev, NULL);
	if (IS_ERR(nspi->reset))
		return dev_err_probe(dev, PTR_ERR(nspi->reset), "failed to get reset\n");

	if (!device_property_read_bool(dev, "novatek,no-dma")) {
		if (resource_size(res) < SPI_DMA_ADDR_REG + sizeof(u32))
			return dev_err_probe(dev, -EINVAL, "register resource too small for DMA\n");

		ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
		if (ret) {
			dev_warn(dev, "DMA unavailable, using PIO\n");
		} else {
			irq = platform_get_irq(pdev, 0);
			if (irq < 0)
				return irq;
			nspi->dma_enabled = true;
		}
	}

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

	ret = clk_prepare_enable(nspi->clk);
	if (ret)
		return dev_err_probe(dev, ret, "failed to enable clock\n");
	nspi->clock_enabled = true;

	ret = novatek_spi_recover(nspi, true);
	if (ret) {
		dev_err_probe(dev, ret, "failed to initialize controller\n");
		goto out_assert_reset;
	}

	if (nspi->dma_enabled) {
		ret = devm_request_irq(dev, irq, novatek_spi_irq, 0, dev_name(dev), nspi);
		if (ret)
			goto out_assert_reset;
		nspi->irq = irq;
	}

	ret = spi_register_controller(host);
	if (ret)
		goto out_mask_irq;

	return 0;

out_mask_irq:
	novatek_spi_mask_irq(nspi);
	if (nspi->irq >= 0) {
		devm_free_irq(dev, nspi->irq, nspi);
		nspi->irq = -1;
	}
out_assert_reset:
	reset_control_assert(nspi->reset);
	clk_disable_unprepare(nspi->clk);
	nspi->clock_enabled = false;
	return ret;
}

static void novatek_spi_remove(struct platform_device *pdev)
{
	struct spi_controller *host = platform_get_drvdata(pdev);
	struct novatek_spi *nspi = spi_controller_get_devdata(host);
	int ret;

	spi_unregister_controller(host);

	novatek_spi_stop_dma(nspi);

	novatek_spi_mask_irq(nspi);
	if (nspi->irq >= 0) {
		devm_free_irq(&pdev->dev, nspi->irq, nspi);
		nspi->irq = -1;
	}

	if (!nspi->reset_asserted) {
		ret = reset_control_assert(nspi->reset);
		if (!ret) {
			nspi->reset_asserted = true;
			udelay(SPI_RESET_DELAY_US);
			nspi->dma_active = false;
		}
	}
	if (nspi->dma_active) {
		dev_err(&pdev->dev, "DMA not stopped, retaining buffer\n");
		return;
	}

	novatek_spi_release_dma(nspi, NULL, 0);
	if (nspi->clock_enabled)
		clk_disable_unprepare(nspi->clk);
	nspi->clock_enabled = false;
}

#ifdef CONFIG_PM_SLEEP
static int novatek_spi_suspend(struct device *dev)
{
	struct spi_controller *host = dev_get_drvdata(dev);
	struct novatek_spi *nspi = spi_controller_get_devdata(host);
	u32 control;
	int ret;

	ret = spi_controller_suspend(host);
	if (ret)
		return ret;

	if (nspi->dma_active) {
		spi_controller_resume(host);
		return -EBUSY;
	}

	novatek_spi_mask_irq(nspi);
	if (!nspi->reset_asserted) {
		control = (nspi->cs_level ? SPI_CTRL_CS_LEVEL : 0);
		writel(control, nspi->base + SPI_CTRL_REG);
		readl(nspi->base + SPI_CTRL_REG);
	}

	clk_disable_unprepare(nspi->clk);
	nspi->clock_enabled = false;

	return 0;
}

static int novatek_spi_resume(struct device *dev)
{
	struct spi_controller *host = dev_get_drvdata(dev);
	struct novatek_spi *nspi = spi_controller_get_devdata(host);
	int ret;

	ret = clk_prepare_enable(nspi->clk);
	if (ret)
		return ret;
	nspi->clock_enabled = true;

	ret = novatek_spi_recover(nspi, nspi->cs_level);
	if (ret)
		goto out_disable_clk;

	ret = spi_controller_resume(host);
	if (ret)
		goto out_disable_clk;

	return 0;

out_disable_clk:
	clk_disable_unprepare(nspi->clk);
	nspi->clock_enabled = false;
	return ret;
}
#endif

static const struct dev_pm_ops novatek_spi_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(novatek_spi_suspend, novatek_spi_resume)
};

static struct platform_driver novatek_spi_driver = {
	.probe = novatek_spi_probe,
	.remove = novatek_spi_remove,
	.driver = {
		.name = "novatek-spi",
		.of_match_table = novatek_spi_of_match,
		.pm = pm_sleep_ptr(&novatek_spi_pm_ops),
	},
};
module_platform_driver(novatek_spi_driver);

MODULE_AUTHOR("Yu-Tung Chang <mtwget@gmail.com>");
MODULE_DESCRIPTION("Novatek SPI host controller driver");
MODULE_LICENSE("GPL");
