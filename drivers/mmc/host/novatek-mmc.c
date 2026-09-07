// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2026 Yu-Tung Chang <mtwget@gmail.com>
 */

#include <linux/bitfield.h>
#include <linux/cache.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/jiffies.h>
#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/sdio.h>
#include <linux/mmc/slot-gpio.h>
#include <linux/module.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/pm_wakeup.h>
#include <linux/property.h>
#include <linux/reset.h>
#include <linux/scatterlist.h>
#include <linux/sizes.h>
#include <linux/spinlock.h>
#include <linux/unaligned.h>

#define MMC_CMD_REG					0x000
#define MMC_ARG_REG					0x004
#define MMC_RSP0_REG					0x008
#define MMC_RSP3_REG					0x014
#define MMC_RSP_CMD_REG					0x018
#define MMC_DATA_CTRL_REG				0x01c
#define MMC_DATA_TIMER_REG				0x020
#define MMC_VOLTAGE_TIMER_REG				0x024
#define MMC_STATUS_REG					0x028
#define MMC_INT_EN_REG					0x030
#define MMC_CLOCK_CTRL_REG				0x038
#define MMC_BUS_WIDTH_REG				0x03c
#define MMC_BUS_STATUS_REG				0x040
#define MMC_CLOCK_CTRL2_REG				0x044
#define MMC_PHY_REG					0x04c
#define MMC_DLY0_REG					0x050
#define MMC_DLY1_REG					0x054
#define MMC_DLY5_REG					0x064
#define MMC_DATA_PORT_REG				0x100
#define MMC_DATA_LENGTH_REG				0x104
#define MMC_FIFO_STATUS_REG				0x108
#define MMC_FIFO_CTRL_REG				0x10c
#define MMC_DMA_DESC_ADDR_REG				0x110
#define MMC_DMA_ERROR_REG				0x118
#define MMC_FIFO_SWITCH_REG				0x1b0

/* MMC_CMD bit fields */
#define MMC_CMD_INDEX					GENMASK(5, 0)
#define MMC_CMD_NEED_RSP				BIT(6)
#define MMC_CMD_LONG_RSP				BIT(7)
#define MMC_CMD_RSP_TIMEOUT				BIT(8)
#define MMC_CMD_EN					BIT(9)
#define MMC_CMD_RESET					BIT(10)
#define MMC_CMD_SDIO_DETECT				BIT(11)

/* MMC_RSP_CMD bit fields */
#define MMC_RSP_CMD_INDEX				GENMASK(5, 0)

/* MMC_DATA_CTRL bit fields */
#define MMC_DATA_CTRL_EN				BIT(6)
#define MMC_DATA_CTRL_VOLTAGE_WAIT			BIT(13)
#define MMC_DATA_CTRL_BLOCK_SIZE			GENMASK(31, 16)
#define MMC_DATA_CTRL_DISABLED \
	(FIELD_PREP_CONST(MMC_DATA_CTRL_EN, 0) | \
	 FIELD_PREP_CONST(MMC_DATA_CTRL_VOLTAGE_WAIT, 0) | \
	 FIELD_PREP_CONST(MMC_DATA_CTRL_BLOCK_SIZE, 0))

/* MMC_VOLTAGE_TIMER bit fields */
#define MMC_VOLTAGE_TIMER_COUNT				GENMASK(11, 0)

/* MMC_STATUS bit fields */
#define MMC_STATUS_RSP_CRC_FAIL				BIT(0)
#define MMC_STATUS_DATA_CRC_FAIL			BIT(1)
#define MMC_STATUS_RSP_TIMEOUT				BIT(2)
#define MMC_STATUS_DATA_TIMEOUT				BIT(3)
#define MMC_STATUS_RSP_CRC_OK				BIT(4)
#define MMC_STATUS_DATA_CRC_OK				BIT(5)
#define MMC_STATUS_CMD_SENT				BIT(6)
#define MMC_STATUS_DATA_END				BIT(7)
#define MMC_STATUS_SDIO_IRQ				BIT(8)
#define MMC_STATUS_VOLTAGE_END				BIT(11)
#define MMC_STATUS_VOLTAGE_TIMEOUT			BIT(12)
#define MMC_STATUS_DMA_ERROR				BIT(19)
#define MMC_STATUS_CMD			\
	(MMC_STATUS_RSP_CRC_FAIL | \
	 MMC_STATUS_RSP_TIMEOUT | \
	 MMC_STATUS_RSP_CRC_OK | \
	 MMC_STATUS_CMD_SENT)
#define MMC_STATUS_CLEAR_CMD			\
	(FIELD_PREP_CONST(MMC_STATUS_RSP_CRC_FAIL, 1) | \
	 FIELD_PREP_CONST(MMC_STATUS_RSP_TIMEOUT, 1) | \
	 FIELD_PREP_CONST(MMC_STATUS_RSP_CRC_OK, 1) | \
	 FIELD_PREP_CONST(MMC_STATUS_CMD_SENT, 1))
#define MMC_STATUS_DATA			\
	(MMC_STATUS_DATA_CRC_FAIL | \
	 MMC_STATUS_DATA_TIMEOUT | \
	 MMC_STATUS_DATA_CRC_OK | \
	 MMC_STATUS_DATA_END)
#define MMC_STATUS_CLEAR_DATA			\
	(FIELD_PREP_CONST(MMC_STATUS_DATA_CRC_FAIL, 1) | \
	 FIELD_PREP_CONST(MMC_STATUS_DATA_TIMEOUT, 1) | \
	 FIELD_PREP_CONST(MMC_STATUS_DATA_CRC_OK, 1) | \
	 FIELD_PREP_CONST(MMC_STATUS_DATA_END, 1))
#define MMC_STATUS_DMA_DONE		\
	(MMC_STATUS_DATA_CRC_OK | \
	 MMC_STATUS_DATA_END)
#define MMC_STATUS_DMA_ERRORS		\
	(MMC_STATUS_DATA_CRC_FAIL | \
	 MMC_STATUS_DATA_TIMEOUT | \
	 MMC_STATUS_DMA_ERROR)
#define MMC_STATUS_VOLTAGE			\
	(MMC_STATUS_VOLTAGE_END | \
	 MMC_STATUS_VOLTAGE_TIMEOUT)
#define MMC_STATUS_CLEAR_VOLTAGE			\
	(FIELD_PREP_CONST(MMC_STATUS_VOLTAGE_END, 1) | \
	 FIELD_PREP_CONST(MMC_STATUS_VOLTAGE_TIMEOUT, 1))

/* MMC_INT_EN bit fields */
#define MMC_INT_EN_DATA_CRC_FAIL			BIT(1)
#define MMC_INT_EN_DATA_TIMEOUT				BIT(3)
#define MMC_INT_EN_DATA_CRC_OK				BIT(5)
#define MMC_INT_EN_DATA_END				BIT(7)
#define MMC_INT_EN_SDIO_IRQ				BIT(8)
#define MMC_INT_EN_VOLTAGE_END				BIT(11)
#define MMC_INT_EN_VOLTAGE_TIMEOUT			BIT(12)
#define MMC_INT_EN_DISABLE_ALL \
	(FIELD_PREP_CONST(MMC_INT_EN_DATA_CRC_FAIL, 0) | \
	 FIELD_PREP_CONST(MMC_INT_EN_DATA_TIMEOUT, 0) | \
	 FIELD_PREP_CONST(MMC_INT_EN_DATA_CRC_OK, 0) | \
	 FIELD_PREP_CONST(MMC_INT_EN_DATA_END, 0) | \
	 FIELD_PREP_CONST(MMC_INT_EN_SDIO_IRQ, 0) | \
	 FIELD_PREP_CONST(MMC_INT_EN_VOLTAGE_END, 0) | \
	 FIELD_PREP_CONST(MMC_INT_EN_VOLTAGE_TIMEOUT, 0))
#define MMC_INT_EN_DATA			\
	(MMC_INT_EN_DATA_CRC_FAIL | \
	 MMC_INT_EN_DATA_TIMEOUT | \
	 MMC_INT_EN_DATA_CRC_OK | MMC_INT_EN_DATA_END)
#define MMC_INT_EN_DATA_ENABLE			\
	(FIELD_PREP_CONST(MMC_INT_EN_DATA_CRC_FAIL, 1) | \
	 FIELD_PREP_CONST(MMC_INT_EN_DATA_TIMEOUT, 1) | \
	 FIELD_PREP_CONST(MMC_INT_EN_DATA_CRC_OK, 1) | \
	 FIELD_PREP_CONST(MMC_INT_EN_DATA_END, 1))
#define MMC_INT_EN_VOLTAGE			\
	(MMC_INT_EN_VOLTAGE_END | \
	 MMC_INT_EN_VOLTAGE_TIMEOUT)
#define MMC_INT_EN_VOLTAGE_ENABLE			\
	(FIELD_PREP_CONST(MMC_INT_EN_VOLTAGE_END, 1) | \
	 FIELD_PREP_CONST(MMC_INT_EN_VOLTAGE_TIMEOUT, 1))

/* MMC_CLOCK_CTRL bit fields */
#define MMC_CLOCK_CTRL_OUTPUT				BIT(10)
#define MMC_CLOCK_CTRL_DEFAULT \
	(FIELD_PREP_CONST(MMC_CLOCK_CTRL_OUTPUT, 0))
#define MMC_CLOCK_CTRL_DISABLED \
	(FIELD_PREP_CONST(MMC_CLOCK_CTRL_OUTPUT, 1))

/* MMC_BUS_WIDTH bit fields */
#define MMC_BUS_WIDTH_VALUE				GENMASK(1, 0)
#define MMC_BUS_WIDTH_VALUE_1BIT			0x0
#define MMC_BUS_WIDTH_VALUE_4BIT			0x1
#define MMC_BUS_WIDTH_VALUE_8BIT			0x2

/* MMC_BUS_STATUS bit fields */
#define MMC_BUS_STATUS_READY				BIT(0)

/* MMC_CLOCK_CTRL2 bit fields */
#define MMC_CLOCK_CTRL2_INPUT_DELAY			GENMASK(21, 16)
#define MMC_CLOCK_CTRL2_INPUT_DELAY_NONE		0x00

/* MMC_PHY bit fields */
#define MMC_PHY_RESET					BIT(0)
#define MMC_PHY_BLOCK_FIFO				BIT(4)

/* MMC_DLY0 bit fields */
#define MMC_DLY0_SAMPLE_EDGE				BIT(12)
#define MMC_DLY0_SAMPLE_SOURCE				BIT(13)
#define MMC_DLY0_SAMPLE_PAD_CLK				BIT(14)

/* MMC_DLY1 bit fields */
#define MMC_DLY1_READ_DELAY				GENMASK(29, 28)
#define MMC_DLY1_READ_DELAY_2				0x2

/* MMC_DLY5 bit fields */
#define MMC_DLY5_DATA_OUT_INVERT			BIT(5)
#define MMC_DLY5_CMD_OUT_INVERT				BIT(6)

/* MMC_DATA_LENGTH bit fields */
#define MMC_DATA_LENGTH_COUNT				GENMASK(25, 0)

/* MMC_FIFO_STATUS bit fields */
#define MMC_FIFO_STATUS_COUNT				GENMASK(5, 0)
#define MMC_FIFO_STATUS_EMPTY				BIT(8)
#define MMC_FIFO_STATUS_FULL				BIT(9)

/* MMC_FIFO_CTRL bit fields */
#define MMC_FIFO_CTRL_EN				BIT(0)
#define MMC_FIFO_CTRL_MODE				BIT(1)
#define MMC_FIFO_CTRL_DIR				BIT(2)
#define MMC_FIFO_CTRL_IDLE			\
	(FIELD_PREP_CONST(MMC_FIFO_CTRL_EN, 0) | \
	 FIELD_PREP_CONST(MMC_FIFO_CTRL_MODE, 0) | \
	 FIELD_PREP_CONST(MMC_FIFO_CTRL_DIR, 0))

/* MMC_DMA_DESC_ADDR bit fields */
#define MMC_DMA_DESC_ADDR_VALUE				GENMASK(30, 0)

/* MMC_FIFO_SWITCH bit fields */
#define MMC_FIFO_SWITCH_DELAY				BIT(4)

#define MMC_DESC_VALID					BIT(0)
#define MMC_DESC_END					BIT(1)
#define MMC_DESC_ACTION					GENMASK(5, 4)
#define MMC_DESC_TRANSFER				0x2
#define MMC_DESC_LENGTH					GENMASK(25, 0)

#define MMC_SDIO_ARG_DATA				GENMASK(7, 0)
#define MMC_SDIO_ARG_ADDRESS				GENMASK(25, 9)
#define MMC_SDIO_ARG_FUNCTION				GENMASK(30, 28)
#define MMC_SDIO_ARG_FUNCTION_COMMON			0x0
#define MMC_SDIO_ARG_WRITE				BIT(31)

#define MMC_MAX_SEGS					128

#define MMC_MAX_REQUEST					SZ_64K

#define MMC_RESET_TIMEOUT_US				10000
#define MMC_RESET_DELAY_US				10
#define MMC_DMA_POLL_MS					20
#define MMC_CMD_TIMEOUT_US				100000
#define MMC_BUSY_TIMEOUT_MS				1000
#define MMC_VOLTAGE_SETTLE_MS				400
#define MMC_VOLTAGE_CLOCK_MS				20
#define MMC_VOLTAGE_TIMEOUT_MS				100
#define MMC_TUNING_TIMEOUT_MS				150

#define MMC_MIN_CLOCK					312500
#define MMC_MAX_CLOCK					192000000

struct novatek_mmc_desc {
	__le32 control;
	__le32 length;
	__le32 address;
};

struct novatek_mmc_soc_data {
	uint fifo_bytes;
};

struct novatek_mmc {
	struct mmc_host *mmc;
	const struct novatek_mmc_soc_data *data;
	void __iomem *base;
	struct clk *clk;
	struct reset_control *rst;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_3v3;
	struct pinctrl_state *pins_1v8;
	struct novatek_mmc_desc *descs;
	dma_addr_t descs_dma;
	struct completion dma_complete;
	struct completion voltage_complete;
	spinlock_t irq_lock;
	u32 irq_mask;
	u32 dma_status;
	u32 voltage_status;
	u32 input_delay;
	int irq;
	int ios_error;
	bool dma_active;
	bool reset_asserted;
	bool suspended;
	bool irq_wake_enabled;
	bool tuning;
	bool no_1v8;
	u8 pio_buf[];
};

static void novatek_mmc_update_irq(struct novatek_mmc *host, u32 mask, u32 value)
{
	ulong flags;

	spin_lock_irqsave(&host->irq_lock, flags);
	host->irq_mask = (host->irq_mask & ~mask) | value;
	if (!host->reset_asserted && !host->suspended) {
		writel(host->irq_mask, host->base + MMC_INT_EN_REG);
		readl(host->base + MMC_INT_EN_REG);
	}
	spin_unlock_irqrestore(&host->irq_lock, flags);
}

static int novatek_mmc_reset(struct novatek_mmc *host)
{
	ulong rate;
	u32 cmd, val;
	int ret;

	rate = clk_get_rate(host->clk);
	if (!rate)
		return -EINVAL;

	writel(MMC_STATUS_DATA_END, host->base + MMC_STATUS_REG);

	writel(MMC_FIFO_CTRL_IDLE, host->base + MMC_FIFO_CTRL_REG);
	ret = readl_poll_timeout(host->base + MMC_FIFO_CTRL_REG,
				 val, !(val & MMC_FIFO_CTRL_EN),
				 1, MMC_RESET_TIMEOUT_US);
	if (ret) {
		dev_err(mmc_dev(host->mmc), "FIFO disable timed out\n");
		return ret;
	}

	writel(MMC_DATA_CTRL_DISABLED, host->base + MMC_DATA_CTRL_REG);

	cmd = readl(host->base + MMC_CMD_REG);
	writel(cmd | MMC_CMD_RESET, host->base + MMC_CMD_REG);
	ret = readl_poll_timeout(host->base + MMC_CMD_REG,
				 val, !(val & MMC_CMD_RESET),
				 1, MMC_RESET_TIMEOUT_US);
	if (ret) {
		dev_err(mmc_dev(host->mmc), "controller reset timed out\n");
		goto out_clear_status;
	}
	udelay(USEC_PER_SEC / rate + 1);

out_clear_status:
	writel(MMC_STATUS_CLEAR_CMD | MMC_STATUS_CLEAR_DATA | MMC_STATUS_DMA_ERROR,
	       host->base + MMC_STATUS_REG);

	return ret;
}

static int novatek_mmc_configure_phy(struct novatek_mmc *host)
{
	u32 val;
	int ret;

	val = readl(host->base + MMC_CLOCK_CTRL2_REG);
	val &= ~MMC_CLOCK_CTRL2_INPUT_DELAY;
	val |= FIELD_PREP(MMC_CLOCK_CTRL2_INPUT_DELAY, host->input_delay);
	writel(val, host->base + MMC_CLOCK_CTRL2_REG);

	val = readl(host->base + MMC_PHY_REG);
	val &= ~MMC_PHY_BLOCK_FIFO;
	val |= (host->mmc->actual_clock >= 50000000 ? MMC_PHY_BLOCK_FIFO : 0) | MMC_PHY_RESET;
	writel(val, host->base + MMC_PHY_REG);
	ret = readl_poll_timeout(host->base + MMC_PHY_REG,
				 val, !(val & MMC_PHY_RESET),
				 1, MMC_RESET_TIMEOUT_US);
	if (ret)
		dev_err(mmc_dev(host->mmc), "PHY reset timed out\n");

	return ret;
}

static int novatek_mmc_init(struct novatek_mmc *host)
{
	u32 val;
	int ret;

	ret = novatek_mmc_reset(host);
	if (ret)
		return ret;

	val = readl(host->base + MMC_FIFO_SWITCH_REG);
	val &= ~MMC_FIFO_SWITCH_DELAY;
	val |= MMC_FIFO_SWITCH_DELAY;
	writel(val, host->base + MMC_FIFO_SWITCH_REG);

	val = readl(host->base + MMC_DLY0_REG);
	val &= ~(MMC_DLY0_SAMPLE_EDGE | MMC_DLY0_SAMPLE_SOURCE | MMC_DLY0_SAMPLE_PAD_CLK);
	val |= MMC_DLY0_SAMPLE_PAD_CLK;
	writel(val, host->base + MMC_DLY0_REG);

	val = readl(host->base + MMC_DLY5_REG);
	val &= ~(MMC_DLY5_DATA_OUT_INVERT | MMC_DLY5_CMD_OUT_INVERT);
	writel(val, host->base + MMC_DLY5_REG);

	ret = novatek_mmc_configure_phy(host);
	if (ret)
		return ret;

	val = readl(host->base + MMC_DLY1_REG);
	val &= ~MMC_DLY1_READ_DELAY;
	val |= FIELD_PREP(MMC_DLY1_READ_DELAY, MMC_DLY1_READ_DELAY_2);
	writel(val, host->base + MMC_DLY1_REG);

	return 0;
}

static int novatek_mmc_abort_dma(struct novatek_mmc *host)
{
	struct mmc_host *mmc = host->mmc;
	ulong flags;
	u32 width;
	int ret;

	novatek_mmc_update_irq(host, MMC_INT_EN_DATA, 0);
	synchronize_irq(host->irq);
	ret = novatek_mmc_reset(host);
	if (!ret) {
		host->dma_active = false;
		return 0;
	}

	spin_lock_irqsave(&host->irq_lock, flags);
	host->reset_asserted = true;
	writel(MMC_INT_EN_DISABLE_ALL, host->base + MMC_INT_EN_REG);
	readl(host->base + MMC_INT_EN_REG);
	spin_unlock_irqrestore(&host->irq_lock, flags);
	synchronize_irq(host->irq);

	reset_control_assert(host->rst);
	reset_control_status(host->rst);
	udelay(MMC_RESET_DELAY_US);
	host->dma_active = false;

	ret = reset_control_deassert(host->rst);
	if (ret)
		return ret;
	reset_control_status(host->rst);
	udelay(MMC_RESET_DELAY_US);
	host->reset_asserted = false;

	ret = novatek_mmc_init(host);
	if (ret)
		return ret;

	width = mmc->ios.bus_width == MMC_BUS_WIDTH_8 ?
		MMC_BUS_WIDTH_VALUE_8BIT :
		mmc->ios.bus_width == MMC_BUS_WIDTH_4 ?
		MMC_BUS_WIDTH_VALUE_4BIT :
		MMC_BUS_WIDTH_VALUE_1BIT;
	writel(FIELD_PREP(MMC_BUS_WIDTH_VALUE, width), host->base + MMC_BUS_WIDTH_REG);

	writel(MMC_CLOCK_CTRL_DEFAULT, host->base + MMC_CLOCK_CTRL_REG);

	novatek_mmc_update_irq(host, 0, 0);

	return 0;
}

static void novatek_mmc_command(struct novatek_mmc *host, struct mmc_command *cmd)
{
	u32 control, status, mask;
	uint i, timeout_ms;
	int ret;

	cmd->error = 0;

	control = FIELD_PREP(MMC_CMD_INDEX, cmd->opcode) | MMC_CMD_EN;
	if (cmd->flags & MMC_RSP_PRESENT)
		control |= MMC_CMD_NEED_RSP;
	if (cmd->flags & MMC_RSP_136)
		control |= MMC_CMD_LONG_RSP;
	if (cmd->opcode == MMC_STOP_TRANSMISSION ||
	    (cmd->opcode == SD_IO_RW_DIRECT &&
	     FIELD_GET(MMC_SDIO_ARG_ADDRESS, cmd->arg) == SDIO_CCCR_ABORT))
		control |= MMC_CMD_SDIO_DETECT;

	writel(MMC_STATUS_CLEAR_CMD, host->base + MMC_STATUS_REG);
	writel(cmd->arg, host->base + MMC_ARG_REG);
	writel(control, host->base + MMC_CMD_REG);

	if (cmd->flags & MMC_RSP_PRESENT)
		mask = MMC_STATUS_RSP_CRC_OK | MMC_STATUS_RSP_CRC_FAIL | MMC_STATUS_RSP_TIMEOUT;
	else
		mask = MMC_STATUS_CMD_SENT;
	ret = readl_poll_timeout(host->base + MMC_STATUS_REG,
				 status, status & mask,
				 1, MMC_CMD_TIMEOUT_US);
	if (ret || (status & MMC_STATUS_RSP_TIMEOUT)) {
		cmd->error = -ETIMEDOUT;
		return;
	}
	if ((status & MMC_STATUS_RSP_CRC_FAIL) && (cmd->flags & MMC_RSP_CRC)) {
		cmd->error = -EILSEQ;
		return;
	}

	if (cmd->flags & MMC_RSP_136) {
		for (i = 0; i < 4; i++)
			cmd->resp[i] = readl(host->base + MMC_RSP3_REG - i * 4);
	} else if (cmd->flags & MMC_RSP_PRESENT) {
		cmd->resp[0] = readl(host->base + MMC_RSP0_REG);
	}
	if ((cmd->flags & MMC_RSP_OPCODE) &&
	    FIELD_GET(MMC_RSP_CMD_INDEX, readl(host->base + MMC_RSP_CMD_REG)) != cmd->opcode) {
		cmd->error = -EILSEQ;
		return;
	}

	if (cmd->flags & MMC_RSP_BUSY) {
		timeout_ms = cmd->busy_timeout ?: MMC_BUSY_TIMEOUT_MS;
		ret = readl_poll_timeout(host->base + MMC_BUS_STATUS_REG,
					 status, status & MMC_BUS_STATUS_READY,
					 10, (u64)timeout_ms * USEC_PER_MSEC);
		if (ret)
			cmd->error = ret;
	}
}

static int novatek_mmc_copy(struct sg_mapping_iter *miter, u8 *buf, uint len, bool read)
{
	int ret = 0;
	uint n;

	while (len) {
		if (!sg_miter_next(miter)) {
			ret = -EINVAL;
			break;
		}
		n = min_t(size_t, len, miter->length);
		if (read)
			memcpy(miter->addr, buf, n);
		else
			memcpy(buf, miter->addr, n);
		miter->consumed = n;
		buf += n;
		len -= n;
	}

	sg_miter_stop(miter);

	return ret;
}

static int novatek_mmc_pio(struct novatek_mmc *host, struct mmc_data *data, ulong deadline)
{
	bool read = data->flags & MMC_DATA_READ;
	struct sg_mapping_iter miter;
	u8 *buf = host->pio_buf;
	uint left = data->blksz * data->blocks;
	uint len, words, i;
	u32 status, fifo;
	int ret = 0;

	sg_miter_start(&miter, data->sg, data->sg_len,
		       SG_MITER_LOCAL | (read ? SG_MITER_TO_SG : SG_MITER_FROM_SG));

	while (left) {
		len = min(left, host->data->fifo_bytes);
		words = DIV_ROUND_UP(len, 4);
		if (!read) {
			memset(buf, 0, host->data->fifo_bytes);
			ret = novatek_mmc_copy(&miter, buf, len, false);
			if (ret)
				break;
		}

		for (;;) {
			status = readl(host->base + MMC_STATUS_REG);
			if (status & MMC_STATUS_DATA_TIMEOUT) {
				ret = -ETIMEDOUT;
				goto out_stop_miter;
			}
			if (status & MMC_STATUS_DATA_CRC_FAIL) {
				ret = -EILSEQ;
				goto out_stop_miter;
			}

			fifo = readl(host->base + MMC_FIFO_STATUS_REG);
			if (read ? (len == host->data->fifo_bytes ?
				    fifo & MMC_FIFO_STATUS_FULL :
				    FIELD_GET(MMC_FIFO_STATUS_COUNT, fifo) == words) :
			    fifo & MMC_FIFO_STATUS_EMPTY)
				break;

			if (time_after_eq(jiffies, deadline)) {
				ret = -ETIMEDOUT;
				goto out_stop_miter;
			}

			usleep_range(10, 20);
		}

		for (i = 0; i < words; i++) {
			if (read) {
				put_unaligned_le32(readl(host->base + MMC_DATA_PORT_REG),
						   buf + i * 4);
			} else {
				writel(get_unaligned_le32(buf + i * 4),
				       host->base + MMC_DATA_PORT_REG);
				/* Avoid FIFO stalls at low SD clocks. */
				readl(host->base + MMC_FIFO_STATUS_REG);
			}
		}

		if (read) {
			ret = novatek_mmc_copy(&miter, buf, len, true);
			if (ret)
				break;
		}

		left -= len;
		cond_resched();
	}

	if (ret)
		goto out_stop_miter;

	for (;;) {
		status = readl(host->base + MMC_STATUS_REG);
		if (status & MMC_STATUS_DATA_TIMEOUT) {
			ret = -ETIMEDOUT;
			break;
		}
		if (status & MMC_STATUS_DATA_CRC_FAIL) {
			ret = -EILSEQ;
			break;
		}
		if ((status & (MMC_STATUS_DATA_END | MMC_STATUS_DATA_CRC_OK)) &&
		    (read || (readl(host->base + MMC_BUS_STATUS_REG) &
			      MMC_BUS_STATUS_READY)))
			break;

		if (time_after_eq(jiffies, deadline)) {
			ret = -ETIMEDOUT;
			break;
		}

		usleep_range(10, 20);
	}

out_stop_miter:
	sg_miter_stop(&miter);

	return ret;
}

static bool novatek_mmc_prepare_dma(struct novatek_mmc *host, struct mmc_data *data)
{
	struct device *dev = mmc_dev(host->mmc);
	struct scatterlist *sg;
	u32 control;
	uint i;
	int count;

	if (!host->descs)
		return false;
	for_each_sg(data->sg, sg, data->sg_len, i) {
		if (!IS_ALIGNED(sg->offset, L1_CACHE_BYTES) ||
		    !IS_ALIGNED(sg->length, L1_CACHE_BYTES))
			return false;
	}

	count = dma_map_sg(dev, data->sg, data->sg_len, mmc_get_dma_dir(data));
	if (!count)
		return false;
	if (count > MMC_MAX_SEGS)
		goto out_unmap_sg;

	for_each_sg(data->sg, sg, count, i) {
		if (!IS_ALIGNED(sg_dma_address(sg), L1_CACHE_BYTES) ||
		    !IS_ALIGNED(sg_dma_len(sg), L1_CACHE_BYTES) ||
		    !FIELD_FIT(MMC_DESC_LENGTH, sg_dma_len(sg)))
			goto out_unmap_sg;
		control = MMC_DESC_VALID | FIELD_PREP(MMC_DESC_ACTION, MMC_DESC_TRANSFER);
		if (i == count - 1)
			control |= MMC_DESC_END;
		host->descs[i].control = cpu_to_le32(control);
		host->descs[i].length = cpu_to_le32(sg_dma_len(sg));
		host->descs[i].address = cpu_to_le32(lower_32_bits(sg_dma_address(sg)));
	}
	dma_wmb();

	return true;

out_unmap_sg:
	dma_unmap_sg(dev, data->sg, data->sg_len, mmc_get_dma_dir(data));
	return false;
}

static int novatek_mmc_wait_dma(struct novatek_mmc *host, struct mmc_data *data, ulong deadline)
{
	ulong timeout;
	u32 status, fifo, bus;
	int ret = 0;

	for (;;) {
		status = READ_ONCE(host->dma_status) | readl(host->base + MMC_STATUS_REG);
		if (status & MMC_STATUS_DMA_ERRORS)
			break;
		if ((status & MMC_STATUS_DMA_DONE) == MMC_STATUS_DMA_DONE)
			break;
		if (time_after_eq(jiffies, deadline)) {
			ret = -ETIMEDOUT;
			break;
		}
		timeout = min(deadline - jiffies, msecs_to_jiffies(MMC_DMA_POLL_MS));
		wait_for_completion_timeout(&host->dma_complete, timeout);
	}

	novatek_mmc_update_irq(host, MMC_INT_EN_DATA, 0);
	synchronize_irq(host->irq);

	for (;;) {
		status = host->dma_status | readl(host->base + MMC_STATUS_REG);
		if (status & MMC_STATUS_DMA_ERROR) {
			dev_err_ratelimited(mmc_dev(host->mmc),
					    "DMA error: %#x\n",
					    readl(host->base + MMC_DMA_ERROR_REG));
			return -EIO;
		}
		if (status & MMC_STATUS_DATA_TIMEOUT)
			return -ETIMEDOUT;
		if (status & MMC_STATUS_DATA_CRC_FAIL)
			return -EILSEQ;
		if (ret)
			return ret;

		fifo = readl(host->base + MMC_FIFO_CTRL_REG);
		bus = readl(host->base + MMC_BUS_STATUS_REG);
		if (!(fifo & MMC_FIFO_CTRL_EN) &&
		    ((data->flags & MMC_DATA_READ) ||
		     (bus & MMC_BUS_STATUS_READY)))
			break;
		if (time_after_eq(jiffies, deadline))
			return -ETIMEDOUT;
		usleep_range(10, 20);
	}
	host->dma_active = false;

	return 0;
}

static void novatek_mmc_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct novatek_mmc *host = mmc_priv(mmc);
	struct mmc_data *data = mrq->data;
	struct mmc_command abort = {};
	u64 cycles, timeout_ms;
	ulong deadline = 0;
	u32 fifo, control;
	uint len;
	bool dma = false;
	int ret;

	mrq->cmd->error = 0;
	if (mrq->sbc)
		mrq->sbc->error = 0;
	if (data) {
		data->bytes_xfered = 0;
		data->error = 0;
	}
	if (mrq->stop)
		mrq->stop->error = 0;

	if (host->ios_error || !mmc->actual_clock || host->suspended || host->reset_asserted) {
		mrq->cmd->error = host->ios_error ?: -EIO;
		goto out_complete_request;
	}

	if (data) {
		ret = readl_poll_timeout(host->base + MMC_BUS_STATUS_REG,
					 fifo, fifo & MMC_BUS_STATUS_READY,
					 10, (host->tuning ? MMC_TUNING_TIMEOUT_MS :
					      MMC_BUSY_TIMEOUT_MS) * USEC_PER_MSEC);
		if (ret) {
			mrq->cmd->error = ret;
			goto out_complete_request;
		}
	}

	ret = novatek_mmc_reset(host);
	if (ret) {
		host->ios_error = ret;
		mrq->cmd->error = ret;
		goto out_complete_request;
	}

	if (mrq->sbc) {
		novatek_mmc_command(host, mrq->sbc);
		if (mrq->sbc->error)
			goto out_stop_transfer;
	}

	if (data) {
		dma = novatek_mmc_prepare_dma(host, data);

		control = readl(host->base + MMC_PHY_REG);
		control &= ~MMC_PHY_BLOCK_FIFO;
		control |= (mmc->actual_clock >= 50000000 &&
			    !(dma && (data->flags & MMC_DATA_WRITE)) ?
			    MMC_PHY_BLOCK_FIFO : 0);
		writel(control, host->base + MMC_PHY_REG);

		len = data->blksz * data->blocks;
		cycles = DIV_ROUND_UP_ULL((u64)data->timeout_ns * mmc->actual_clock, NSEC_PER_SEC) +
			 data->timeout_clks;
		writel(clamp_val(cycles, 1, U32_MAX), host->base + MMC_DATA_TIMER_REG);

		/*
		 * Budget each block's access timeout plus wire time and
		 * scheduling.
		 */
		timeout_ms = DIV_ROUND_UP_ULL((cycles * data->blocks + (u64)len * 8) * MSEC_PER_SEC,
					      mmc->actual_clock) + 1000;
		if (host->tuning)
			timeout_ms = MMC_TUNING_TIMEOUT_MS;
		deadline = jiffies + msecs_to_jiffies(min_t(u64, timeout_ms, U32_MAX));

		len = mmc->actual_clock >= 48000000 ? 3 :
		      mmc->actual_clock >= 24000000 ? 6 :
		      mmc->actual_clock >= 12000000 ? 9 : 21;
		for (len += 2; len; len--)
			readl(host->base + MMC_CMD_REG);

		control = FIELD_PREP(MMC_DATA_CTRL_BLOCK_SIZE, data->blksz);
		if (!dma)
			control |= MMC_DATA_CTRL_EN;
		writel(control, host->base + MMC_DATA_CTRL_REG);

		if (dma) {
			reinit_completion(&host->dma_complete);
			WRITE_ONCE(host->dma_status, 0);
			writel(FIELD_PREP(MMC_DMA_DESC_ADDR_VALUE, host->descs_dma), host->base + MMC_DMA_DESC_ADDR_REG);
			novatek_mmc_update_irq(host, MMC_INT_EN_DATA, MMC_INT_EN_DATA_ENABLE);
		}

		writel(FIELD_PREP(MMC_DATA_LENGTH_COUNT, data->blksz * data->blocks), host->base + MMC_DATA_LENGTH_REG);

		fifo = (data->flags & MMC_DATA_WRITE ? MMC_FIFO_CTRL_DIR : 0) |
		       (dma ? MMC_FIFO_CTRL_MODE : 0);
		writel(fifo, host->base + MMC_FIFO_CTRL_REG);

		if (dma) {
			writel(control | MMC_DATA_CTRL_EN, host->base + MMC_DATA_CTRL_REG);
			host->dma_active = true;
		}
		writel(fifo | MMC_FIFO_CTRL_EN, host->base + MMC_FIFO_CTRL_REG);
	}

	novatek_mmc_command(host, mrq->cmd);
	if (mrq->cmd->error)
		goto out_stop_transfer;

	if (data) {
		if (dma)
			data->error = novatek_mmc_wait_dma(host, data, deadline);
		else
			data->error = novatek_mmc_pio(host, data, deadline);
		if (data->error)
			goto out_stop_transfer;

		if (dma) {
			dma_unmap_sg(mmc_dev(mmc), data->sg, data->sg_len, mmc_get_dma_dir(data));
			dma = false;
		}
		data->bytes_xfered = data->blksz * data->blocks;
	}

	if (mrq->stop && !mrq->sbc)
		novatek_mmc_command(host, mrq->stop);
	if (mrq->stop && mrq->stop->error)
		goto out_stop_transfer;

	goto out_complete_request;

out_stop_transfer:
	if (dma) {
		ret = novatek_mmc_abort_dma(host);
		dma_unmap_sg(mmc_dev(mmc), data->sg, data->sg_len, mmc_get_dma_dir(data));
	} else {
		ret = novatek_mmc_reset(host);
	}
	if (ret) {
		host->ios_error = ret;
		goto out_complete_request;
	}

	if (mrq->sbc && mrq->sbc->error)
		goto out_complete_request;

	if (data && mrq->cmd->opcode == SD_IO_RW_EXTENDED) {
		abort.opcode = SD_IO_RW_DIRECT;
		abort.arg = MMC_SDIO_ARG_WRITE |
			    FIELD_PREP(MMC_SDIO_ARG_FUNCTION, MMC_SDIO_ARG_FUNCTION_COMMON) |
			    FIELD_PREP(MMC_SDIO_ARG_ADDRESS, SDIO_CCCR_ABORT) |
			    FIELD_PREP(MMC_SDIO_ARG_DATA,
				       FIELD_GET(MMC_SDIO_ARG_FUNCTION, mrq->cmd->arg));
		abort.flags = MMC_RSP_R5 | MMC_CMD_AC;
		novatek_mmc_command(host, &abort);
	} else if (mrq->stop && !mrq->stop->error) {
		novatek_mmc_command(host, mrq->stop);
	}

out_complete_request:
	mmc_request_done(mmc, mrq);
}

static void novatek_mmc_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct novatek_mmc *host = mmc_priv(mmc);
	ulong rate;
	u32 width;
	int ret;

	mmc->actual_clock = 0;
	if (host->suspended) {
		ret = -EIO;
		goto out_set_ios_error;
	}

	if (host->reset_asserted && ios->power_mode == MMC_POWER_UP) {
		ret = reset_control_deassert(host->rst);
		if (ret)
			goto out_set_ios_error;
		reset_control_status(host->rst);
		udelay(MMC_RESET_DELAY_US);
		host->reset_asserted = false;
	}

	if (!host->reset_asserted)
		writel(MMC_CLOCK_CTRL_DISABLED, host->base + MMC_CLOCK_CTRL_REG);

	if (ios->power_mode == MMC_POWER_OFF) {
		if (!host->reset_asserted)
			host->ios_error = novatek_mmc_reset(host);

		mmc_regulator_disable_vqmmc(mmc);
		ret = mmc_regulator_set_ocr(mmc, mmc->supply.vmmc, 0);
		if (ret)
			host->ios_error = ret;

		return;
	}

	if (host->ios_error && ios->power_mode != MMC_POWER_UP)
		return;

	ret = mmc_regulator_set_ocr(mmc, mmc->supply.vmmc, ios->vdd);
	if (ret)
		goto out_set_ios_error;

	ret = mmc_regulator_enable_vqmmc(mmc);
	if (ret)
		goto out_set_ios_error;

	if (ios->timing != MMC_TIMING_UHS_SDR50 && ios->timing != MMC_TIMING_UHS_SDR104)
		host->input_delay = MMC_CLOCK_CTRL2_INPUT_DELAY_NONE;

	if (ios->power_mode == MMC_POWER_UP) {
		ret = novatek_mmc_init(host);
		if (ret)
			goto out_set_ios_error;
		novatek_mmc_update_irq(host, 0, 0);
	}

	width = ios->bus_width == MMC_BUS_WIDTH_8 ?
		MMC_BUS_WIDTH_VALUE_8BIT :
		ios->bus_width == MMC_BUS_WIDTH_4 ?
		MMC_BUS_WIDTH_VALUE_4BIT :
		MMC_BUS_WIDTH_VALUE_1BIT;
	writel(FIELD_PREP(MMC_BUS_WIDTH_VALUE, width), host->base + MMC_BUS_WIDTH_REG);

	if (ios->clock) {
		ret = clk_set_rate(host->clk, ios->clock);
		if (ret)
			goto out_set_ios_error;

		rate = clk_get_rate(host->clk);
		if (!rate || rate > ios->clock) {
			ret = -EINVAL;
			goto out_set_ios_error;
		}
		mmc->actual_clock = rate;

		ret = novatek_mmc_configure_phy(host);
		if (ret) {
			mmc->actual_clock = 0;
			goto out_set_ios_error;
		}

		writel(MMC_CLOCK_CTRL_DEFAULT, host->base + MMC_CLOCK_CTRL_REG);
	}

	host->ios_error = 0;

	return;

out_set_ios_error:
	host->ios_error = ret;
	dev_err(mmc_dev(mmc), "failed to configure bus: %d\n", ret);
}

static int novatek_mmc_start_signal_voltage_switch(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct novatek_mmc *host = mmc_priv(mmc);
	struct pinctrl_state *pins;
	ulong timeout;
	u32 clock, control, cycles, status;
	bool changed;
	int ret;

	if (host->suspended || host->reset_asserted)
		return -EIO;
	if (ios->signal_voltage != MMC_SIGNAL_VOLTAGE_330 &&
	    ios->signal_voltage != MMC_SIGNAL_VOLTAGE_180)
		return -EINVAL;
	if (ios->signal_voltage == MMC_SIGNAL_VOLTAGE_180 && host->no_1v8)
		return -EINVAL;
	if (IS_ERR(mmc->supply.vqmmc))
		return ios->signal_voltage == MMC_SIGNAL_VOLTAGE_330 ?
		       0 : -EINVAL;
	pins = ios->signal_voltage == MMC_SIGNAL_VOLTAGE_330 ? host->pins_3v3 : host->pins_1v8;

	clock = readl(host->base + MMC_CLOCK_CTRL_REG);
	control = clock & ~MMC_CLOCK_CTRL_OUTPUT;
	writel(control | MMC_CLOCK_CTRL_OUTPUT, host->base + MMC_CLOCK_CTRL_REG);

	novatek_mmc_update_irq(host, MMC_INT_EN_VOLTAGE, 0);
	synchronize_irq(host->irq);

	control = readl(host->base + MMC_DATA_CTRL_REG);
	control &= ~MMC_DATA_CTRL_VOLTAGE_WAIT;
	writel(control, host->base + MMC_DATA_CTRL_REG);
	writel(MMC_STATUS_CLEAR_VOLTAGE, host->base + MMC_STATUS_REG);

	ret = mmc_regulator_set_vqmmc(mmc, ios);
	if (ret < 0)
		goto out_restore_clock;
	changed = !ret;

	if (pins) {
		ret = pinctrl_select_state(host->pinctrl, pins);
		if (ret)
			goto out_restore_clock;
	}
	ret = 0;
	if (!changed || ios->power_mode == MMC_POWER_UP ||
	    ios->signal_voltage == MMC_SIGNAL_VOLTAGE_330)
		goto out_restore_clock;

	cycles = DIV_ROUND_UP(clk_get_rate(host->clk), MSEC_PER_SEC);
	if (!cycles || !FIELD_FIT(MMC_VOLTAGE_TIMER_COUNT, cycles)) {
		ret = -EINVAL;
		goto out_restore_clock;
	}
	msleep(MMC_VOLTAGE_SETTLE_MS);

	/* The detector needs SD clocks while the core keeps ios.clock zero. */
	control = clock & ~MMC_CLOCK_CTRL_OUTPUT;
	writel(control, host->base + MMC_CLOCK_CTRL_REG);
	msleep(MMC_VOLTAGE_CLOCK_MS);

	reinit_completion(&host->voltage_complete);
	WRITE_ONCE(host->voltage_status, 0);
	writel(FIELD_PREP(MMC_VOLTAGE_TIMER_COUNT, cycles), host->base + MMC_VOLTAGE_TIMER_REG);
	novatek_mmc_update_irq(host, MMC_INT_EN_VOLTAGE, MMC_INT_EN_VOLTAGE_ENABLE);

	control = readl(host->base + MMC_DATA_CTRL_REG);
	control &= ~MMC_DATA_CTRL_VOLTAGE_WAIT;
	control |= MMC_DATA_CTRL_VOLTAGE_WAIT;
	writel(control, host->base + MMC_DATA_CTRL_REG);

	timeout = wait_for_completion_timeout(&host->voltage_complete,
					      msecs_to_jiffies(MMC_VOLTAGE_TIMEOUT_MS));
	novatek_mmc_update_irq(host, MMC_INT_EN_VOLTAGE, 0);
	synchronize_irq(host->irq);

	status = host->voltage_status | (readl(host->base + MMC_STATUS_REG) & MMC_STATUS_VOLTAGE);
	control = readl(host->base + MMC_DATA_CTRL_REG);
	control &= ~MMC_DATA_CTRL_VOLTAGE_WAIT;
	writel(control, host->base + MMC_DATA_CTRL_REG);
	writel(MMC_STATUS_CLEAR_VOLTAGE, host->base + MMC_STATUS_REG);
	if (!timeout || (status & MMC_STATUS_VOLTAGE_TIMEOUT))
		ret = -ETIMEDOUT;
	else if (!(status & MMC_STATUS_VOLTAGE_END))
		ret = -EIO;

out_restore_clock:
	writel(clock, host->base + MMC_CLOCK_CTRL_REG);

	return ret;
}

static int novatek_mmc_execute_tuning(struct mmc_host *mmc, u32 opcode)
{
	struct novatek_mmc *host = mmc_priv(mmc);
	u32 delay, length = 0, best_length = 0, best_start = 0, val;
	int ret = -EIO;

	host->tuning = true;
	for (delay = 0; delay <= FIELD_MAX(MMC_CLOCK_CTRL2_INPUT_DELAY); delay++) {
		host->input_delay = delay;
		val = readl(host->base + MMC_CLOCK_CTRL2_REG);
		val &= ~MMC_CLOCK_CTRL2_INPUT_DELAY;
		val |= FIELD_PREP(MMC_CLOCK_CTRL2_INPUT_DELAY, delay);
		writel(val, host->base + MMC_CLOCK_CTRL2_REG);

		ret = mmc_send_tuning(mmc, opcode, NULL);
		if (ret == -ENOMEM || ret == -EINVAL)
			goto out_clear_tuning;
		if (host->ios_error) {
			ret = host->ios_error;
			goto out_clear_tuning;
		}
		if (ret) {
			length = 0;
			continue;
		}
		length++;
		if (length > best_length) {
			best_length = length;
			best_start = delay + 1 - length;
		}
	}

	if (!best_length) {
		ret = -EIO;
		goto out_clear_tuning;
	}

	host->input_delay = best_start + best_length / 2;
	val = readl(host->base + MMC_CLOCK_CTRL2_REG);
	val &= ~MMC_CLOCK_CTRL2_INPUT_DELAY;
	val |= FIELD_PREP(MMC_CLOCK_CTRL2_INPUT_DELAY, host->input_delay);
	writel(val, host->base + MMC_CLOCK_CTRL2_REG);

	ret = mmc_send_tuning(mmc, opcode, NULL);
	if (!ret)
		dev_dbg(mmc_dev(mmc),
			"input delay %u, passing window %u-%u\n",
			host->input_delay, best_start, best_start + best_length - 1);

out_clear_tuning:
	host->tuning = false;

	return ret;
}

static int novatek_mmc_get_cd(struct mmc_host *mmc)
{
	int ret;

	if (mmc->caps & MMC_CAP_NONREMOVABLE)
		return 1;

	ret = mmc_gpio_get_cd(mmc);
	return ret == -ENOSYS ? 1 : ret;
}

static int novatek_mmc_card_busy(struct mmc_host *mmc)
{
	struct novatek_mmc *host = mmc_priv(mmc);

	if (host->suspended || host->reset_asserted)
		return 1;

	return !(readl(host->base + MMC_BUS_STATUS_REG) & MMC_BUS_STATUS_READY);
}

static void novatek_mmc_enable_sdio_irq(struct mmc_host *mmc, int enable)
{
	struct novatek_mmc *host = mmc_priv(mmc);
	u32 value;

	value = (enable ? MMC_INT_EN_SDIO_IRQ : 0);
	novatek_mmc_update_irq(host, MMC_INT_EN_SDIO_IRQ, value);
}

static irqreturn_t novatek_mmc_irq(int irq, void *data)
{
	struct novatek_mmc *host = data;
	bool complete_dma = false;
	ulong flags;
	u32 status, pending;

	spin_lock_irqsave(&host->irq_lock, flags);
	if (host->reset_asserted || host->suspended)
		goto out_unlock;

	status = readl(host->base + MMC_STATUS_REG);
	pending = status & host->irq_mask;
	if (!pending)
		goto out_unlock;
	if (pending & MMC_INT_EN_DATA) {
		host->dma_status |= status & (MMC_STATUS_DATA | MMC_STATUS_DMA_ERROR);
		if ((host->dma_status & MMC_STATUS_DMA_ERRORS) ||
		    (host->dma_status & MMC_STATUS_DMA_DONE) == MMC_STATUS_DMA_DONE) {
			host->irq_mask &= ~MMC_INT_EN_DATA;
			complete_dma = true;
		}
	}
	if (pending & MMC_STATUS_VOLTAGE) {
		host->voltage_status |= pending & MMC_STATUS_VOLTAGE;
		host->irq_mask &= ~MMC_INT_EN_VOLTAGE;
	}
	if (pending & MMC_STATUS_SDIO_IRQ)
		host->irq_mask &= ~MMC_INT_EN_SDIO_IRQ;
	writel(host->irq_mask, host->base + MMC_INT_EN_REG);
	writel(pending, host->base + MMC_STATUS_REG);
	spin_unlock_irqrestore(&host->irq_lock, flags);

	if (complete_dma)
		complete(&host->dma_complete);
	if (pending & MMC_STATUS_VOLTAGE)
		complete(&host->voltage_complete);
	if (pending & MMC_STATUS_SDIO_IRQ)
		mmc_signal_sdio_irq(host->mmc);

	return IRQ_HANDLED;

out_unlock:
	spin_unlock_irqrestore(&host->irq_lock, flags);
	return IRQ_NONE;
}

static const struct mmc_host_ops novatek_mmc_ops = {
	.request = novatek_mmc_request,
	.set_ios = novatek_mmc_set_ios,
	.get_cd = novatek_mmc_get_cd,
	.get_ro = mmc_gpio_get_ro,
	.card_busy = novatek_mmc_card_busy,
	.enable_sdio_irq = novatek_mmc_enable_sdio_irq,
	.start_signal_voltage_switch = novatek_mmc_start_signal_voltage_switch,
	.execute_tuning = novatek_mmc_execute_tuning,
};

static const struct novatek_mmc_soc_data na51089_mmc_data = {
	.fifo_bytes = 64,
};

static const struct of_device_id novatek_mmc_of_match[] = {
	{ .compatible = "novatek,na51089-mmc", .data = &na51089_mmc_data },
	{}
};
MODULE_DEVICE_TABLE(of, novatek_mmc_of_match);

static int novatek_mmc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct novatek_mmc_soc_data *data = device_get_match_data(dev);
	struct novatek_mmc *host;
	struct mmc_host *mmc;
	struct resource *res;
	bool has_pins_3v3, has_pins_1v8;
	int ret;

	if (!data)
		return -ENODEV;

	mmc = mmc_alloc_host(struct_size(host, pio_buf, data->fifo_bytes), dev);
	if (!mmc)
		return -ENOMEM;
	host = mmc_priv(mmc);
	host->mmc = mmc;
	host->data = data;
	spin_lock_init(&host->irq_lock);
	init_completion(&host->dma_complete);
	init_completion(&host->voltage_complete);
	platform_set_drvdata(pdev, host);

	host->base = devm_platform_get_and_ioremap_resource(pdev, 0, &res);
	if (IS_ERR(host->base)) {
		ret = PTR_ERR(host->base);
		goto out_free_host;
	}
	if (resource_size(res) < MMC_FIFO_SWITCH_REG + sizeof(u32)) {
		ret = -EINVAL;
		goto out_free_host;
	}

	host->irq = platform_get_irq(pdev, 0);
	if (host->irq < 0) {
		ret = host->irq;
		goto out_free_host;
	}

	host->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(host->clk)) {
		ret = dev_err_probe(dev, PTR_ERR(host->clk), "failed to get clock\n");
		goto out_free_host;
	}

	host->rst = devm_reset_control_get_exclusive(dev, NULL);
	if (IS_ERR(host->rst)) {
		ret = dev_err_probe(dev, PTR_ERR(host->rst), "failed to get reset\n");
		goto out_free_host;
	}

	mmc->ops = &novatek_mmc_ops;

	mmc->f_min = MMC_MIN_CLOCK;
	mmc->f_max = MMC_MAX_CLOCK;
	mmc->ocr_avail = MMC_VDD_32_33 | MMC_VDD_33_34;
	mmc->caps = MMC_CAP_CMD23 | MMC_CAP_WAIT_WHILE_BUSY | MMC_CAP_SDIO_IRQ;
	mmc->max_busy_timeout = 0;

	mmc->max_req_size = MMC_MAX_REQUEST;
	mmc->max_seg_size = MMC_MAX_REQUEST;
	mmc->max_segs = MMC_MAX_SEGS;
	mmc->max_blk_size = 2048;
	mmc->max_blk_count = MMC_MAX_REQUEST / 512;

	ret = mmc_regulator_get_supply(mmc);
	if (ret)
		goto out_free_host;
	mmc->ocr_avail &= MMC_VDD_32_33 | MMC_VDD_33_34;
	if (!mmc->ocr_avail) {
		ret = dev_err_probe(dev, -EINVAL, "a 3.3 V card supply is required\n");
		goto out_free_host;
	}

	ret = mmc_of_parse(mmc);
	if (ret)
		goto out_free_host;
	mmc->f_max = min(mmc->f_max, MMC_MAX_CLOCK);
	mmc->caps &= ~(MMC_CAP_UHS_DDR50 | MMC_CAP_DDR);
	mmc->caps2 &= ~(MMC_CAP2_HS200 | MMC_CAP2_HS400 | MMC_CAP2_HS400_ES);

	host->no_1v8 = device_property_read_bool(dev, "no-1-8-v");
	if (host->no_1v8)
		mmc->caps &= ~MMC_CAP_UHS;

	has_pins_3v3 = device_property_match_string(dev, "pinctrl-names", "sdmmc-3v3") >= 0;
	has_pins_1v8 = device_property_match_string(dev, "pinctrl-names", "sdmmc-1v8") >= 0;
	if (has_pins_3v3 || has_pins_1v8) {
		host->pinctrl = devm_pinctrl_get(dev);
		if (IS_ERR(host->pinctrl)) {
			ret = dev_err_probe(dev, PTR_ERR(host->pinctrl),
					    "failed to get pin control\n");
			goto out_free_host;
		}
	}

	if (has_pins_3v3) {
		host->pins_3v3 = pinctrl_lookup_state(host->pinctrl, "sdmmc-3v3");
		if (IS_ERR(host->pins_3v3)) {
			ret = dev_err_probe(dev, PTR_ERR(host->pins_3v3),
					    "failed to get 3.3 V pin state\n");
			goto out_free_host;
		}
	}

	if (has_pins_1v8) {
		host->pins_1v8 = pinctrl_lookup_state(host->pinctrl, "sdmmc-1v8");
		if (IS_ERR(host->pins_1v8)) {
			ret = dev_err_probe(dev, PTR_ERR(host->pins_1v8),
					    "failed to get 1.8 V pin state\n");
			goto out_free_host;
		}
	}

	if (!device_property_read_bool(dev, "novatek,no-dma")) {
		ret = dma_set_mask(dev, DMA_BIT_MASK(32));
		if (!ret)
			ret = dma_set_coherent_mask(dev, DMA_BIT_MASK(31));
		if (!ret)
			host->descs = dma_alloc_coherent(dev, MMC_MAX_SEGS * sizeof(*host->descs),
							 &host->descs_dma, GFP_KERNEL);
		if (!host->descs)
			dev_warn(dev, "DMA unavailable, using PIO\n");
	}

	if (mmc->pm_caps & MMC_PM_WAKE_SDIO_IRQ) {
		ret = devm_device_init_wakeup(dev);
		if (ret)
			goto out_free_descs;
	}

	if (!mmc_host_can_gpio_cd(mmc) && !(mmc->caps & MMC_CAP_NONREMOVABLE))
		mmc->caps |= MMC_CAP_NEEDS_POLL;

	ret = reset_control_assert(host->rst);
	if (ret)
		goto out_free_descs;

	ret = clk_set_rate(host->clk, 400000);
	if (ret)
		goto out_free_descs;

	ret = clk_prepare_enable(host->clk);
	if (ret)
		goto out_free_descs;

	ret = reset_control_deassert(host->rst);
	if (ret)
		goto out_disable_clk;

	writel(MMC_INT_EN_DISABLE_ALL, host->base + MMC_INT_EN_REG);

	writel(MMC_CLOCK_CTRL_DISABLED, host->base + MMC_CLOCK_CTRL_REG);

	ret = novatek_mmc_init(host);
	if (ret)
		goto out_assert_reset;

	ret = devm_request_irq(dev, host->irq, novatek_mmc_irq, 0, dev_name(dev), host);
	if (ret)
		goto out_assert_reset;

	ret = mmc_add_host(mmc);
	if (ret)
		goto out_disable_irq;

	return 0;

out_disable_irq:
	writel(MMC_INT_EN_DISABLE_ALL, host->base + MMC_INT_EN_REG);
	devm_free_irq(dev, host->irq, host);
out_assert_reset:
	reset_control_assert(host->rst);
out_disable_clk:
	clk_disable_unprepare(host->clk);
out_free_descs:
	if (host->descs)
		dma_free_coherent(dev, MMC_MAX_SEGS * sizeof(*host->descs),
				  host->descs, host->descs_dma);
out_free_host:
	mmc_free_host(mmc);

	return ret;
}

static void novatek_mmc_remove(struct platform_device *pdev)
{
	struct novatek_mmc *host = platform_get_drvdata(pdev);

	mmc_remove_host(host->mmc);

	novatek_mmc_update_irq(host, U32_MAX, 0);
	devm_free_irq(&pdev->dev, host->irq, host);

	reset_control_assert(host->rst);
	if (!host->suspended)
		clk_disable_unprepare(host->clk);

	if (host->descs)
		dma_free_coherent(&pdev->dev, MMC_MAX_SEGS * sizeof(*host->descs),
				  host->descs, host->descs_dma);
	mmc_free_host(host->mmc);
}

#ifdef CONFIG_PM_SLEEP
static int novatek_mmc_suspend(struct device *dev)
{
	struct novatek_mmc *host = dev_get_drvdata(dev);
	struct mmc_host *mmc = host->mmc;
	int ret;

	if (READ_ONCE(host->dma_active))
		return -EBUSY;
	novatek_mmc_update_irq(host, MMC_INT_EN_DATA, 0);
	synchronize_irq(host->irq);

	if (host->irq_wake_enabled || !device_may_wakeup(dev) ||
	    !mmc_card_keep_power(mmc) || !mmc_card_wake_sdio_irq(mmc))
		return 0;

	ret = enable_irq_wake(host->irq);
	if (ret)
		return ret;
	host->irq_wake_enabled = true;

	return 0;
}

static int novatek_mmc_resume(struct device *dev)
{
	struct novatek_mmc *host = dev_get_drvdata(dev);
	int ret;

	if (!host->irq_wake_enabled)
		return 0;

	ret = disable_irq_wake(host->irq);
	if (ret)
		return ret;
	host->irq_wake_enabled = false;

	return 0;
}

static int novatek_mmc_suspend_noirq(struct device *dev)
{
	struct novatek_mmc *host = dev_get_drvdata(dev);
	struct mmc_host *mmc = host->mmc;

	if (host->suspended)
		return 0;

	if (mmc_card_keep_power(mmc) || mmc_card_wake_sdio_irq(mmc) ||
	    sdio_irq_claimed(mmc) || mmc->ios.power_mode != MMC_POWER_OFF) {
		device_set_awake_path(dev);
		return 0;
	}

	clk_disable_unprepare(host->clk);
	host->suspended = true;

	return 0;
}

static int novatek_mmc_resume_noirq(struct device *dev)
{
	struct novatek_mmc *host = dev_get_drvdata(dev);
	int ret;

	if (!host->suspended)
		return 0;

	ret = clk_prepare_enable(host->clk);
	if (ret)
		return ret;
	host->suspended = false;

	return 0;
}
#endif

static const struct dev_pm_ops novatek_mmc_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(novatek_mmc_suspend, novatek_mmc_resume)
	SET_NOIRQ_SYSTEM_SLEEP_PM_OPS(novatek_mmc_suspend_noirq, novatek_mmc_resume_noirq)
};

static struct platform_driver novatek_mmc_driver = {
	.probe = novatek_mmc_probe,
	.remove = novatek_mmc_remove,
	.driver = {
		.name = "novatek-mmc",
		.of_match_table = novatek_mmc_of_match,
		.pm = pm_sleep_ptr(&novatek_mmc_pm_ops),
	},
};
module_platform_driver(novatek_mmc_driver);

MODULE_AUTHOR("Yu-Tung Chang <mtwget@gmail.com>");
MODULE_DESCRIPTION("Novatek SD/SDIO/eMMC host controller");
MODULE_LICENSE("GPL");
