// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2026 Yudong Zhang <mtwget@gmail.com>
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/jiffies.h>
#include <linux/mfd/syscon.h>
#include <linux/mfd/syscon/novatek-sramctrl.h>
#include <linux/mmc/host.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/sdio.h>
#include <linux/mmc/slot-gpio.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/reset.h>
#include <linux/scatterlist.h>
#include <linux/sizes.h>
#include <linux/unaligned.h>

#define MMC_CMD					0x000
#define MMC_CMD_INDEX_MASK			GENMASK(5, 0)
#define MMC_CMD_NEED_RSP_MASK			BIT(6)
#define MMC_CMD_RSP_NONE			0x0
#define MMC_CMD_RSP_PRESENT			0x1
#define MMC_CMD_LONG_RSP_MASK			BIT(7)
#define MMC_CMD_RSP_SHORT			0x0
#define MMC_CMD_RSP_LONG			0x1
#define MMC_CMD_RSP_TIMEOUT_MASK		BIT(8)
#define MMC_CMD_RSP_TIMEOUT_64			0x0
#define MMC_CMD_RSP_TIMEOUT_5			0x1
#define MMC_CMD_EN_MASK				BIT(9)
#define MMC_CMD_NO_START			0x0
#define MMC_CMD_START				0x1
#define MMC_CMD_RESET_MASK			BIT(10)
#define MMC_CMD_NO_RESET			0x0
#define MMC_CMD_RESET				0x1
#define MMC_CMD_SDIO_DETECT_MASK		BIT(11)
#define MMC_CMD_SDIO_DETECT_NO_CHANGE		0x0
#define MMC_CMD_SDIO_DETECT_ENABLE		0x1

#define MMC_ARG					0x004

#define MMC_RSP0				0x008
#define MMC_RSP3				0x014

#define MMC_RSP_CMD				0x018
#define MMC_RSP_CMD_INDEX_MASK			GENMASK(5, 0)

#define MMC_DATA_CTRL				0x01c
#define MMC_DATA_CTRL_EN_MASK			BIT(6)
#define MMC_DATA_CTRL_DISABLE			0x0
#define MMC_DATA_CTRL_ENABLE			0x1
#define MMC_DATA_CTRL_BLOCK_SIZE_MASK		GENMASK(31, 16)

#define MMC_DATA_TIMER				0x020

#define MMC_STATUS				0x028
#define MMC_STATUS_RSP_CRC_FAIL_MASK		BIT(0)
#define MMC_STATUS_RSP_CRC_FAIL_NONE		0x0
#define MMC_STATUS_RSP_CRC_FAIL_SET		0x1
#define MMC_STATUS_DATA_CRC_FAIL_MASK		BIT(1)
#define MMC_STATUS_DATA_CRC_FAIL_NONE		0x0
#define MMC_STATUS_DATA_CRC_FAIL_SET		0x1
#define MMC_STATUS_RSP_TIMEOUT_MASK		BIT(2)
#define MMC_STATUS_RSP_TIMEOUT_NONE		0x0
#define MMC_STATUS_RSP_TIMEOUT_SET		0x1
#define MMC_STATUS_DATA_TIMEOUT_MASK		BIT(3)
#define MMC_STATUS_DATA_TIMEOUT_NONE		0x0
#define MMC_STATUS_DATA_TIMEOUT_SET		0x1
#define MMC_STATUS_RSP_CRC_OK_MASK		BIT(4)
#define MMC_STATUS_RSP_CRC_OK_NONE		0x0
#define MMC_STATUS_RSP_CRC_OK_SET		0x1
#define MMC_STATUS_DATA_CRC_OK_MASK		BIT(5)
#define MMC_STATUS_DATA_CRC_OK_NONE		0x0
#define MMC_STATUS_DATA_CRC_OK_SET		0x1
#define MMC_STATUS_CMD_SENT_MASK		BIT(6)
#define MMC_STATUS_CMD_SENT_NONE		0x0
#define MMC_STATUS_CMD_SENT_SET			0x1
#define MMC_STATUS_DATA_END_MASK		BIT(7)
#define MMC_STATUS_DATA_END_NONE		0x0
#define MMC_STATUS_DATA_END_SET			0x1
#define MMC_STATUS_SDIO_IRQ_MASK		BIT(8)
#define MMC_STATUS_SDIO_IRQ_NONE		0x0
#define MMC_STATUS_SDIO_IRQ_SET			0x1
#define MMC_STATUS_KEEP				0x0
#define MMC_STATUS_CLEAR			0x1

#define MMC_STATUS_CMD_MASK				\
	(MMC_STATUS_RSP_CRC_FAIL_MASK | \
	 MMC_STATUS_RSP_TIMEOUT_MASK | \
	 MMC_STATUS_RSP_CRC_OK_MASK | \
	 MMC_STATUS_CMD_SENT_MASK)
#define MMC_STATUS_DATA_MASK				\
	(MMC_STATUS_DATA_CRC_FAIL_MASK | \
	 MMC_STATUS_DATA_TIMEOUT_MASK | \
	 MMC_STATUS_DATA_CRC_OK_MASK | \
	 MMC_STATUS_DATA_END_MASK)
#define MMC_STATUS_CLEAR_CMD				\
	(FIELD_PREP(MMC_STATUS_RSP_CRC_FAIL_MASK, MMC_STATUS_CLEAR) | \
	 FIELD_PREP(MMC_STATUS_RSP_TIMEOUT_MASK, MMC_STATUS_CLEAR) | \
	 FIELD_PREP(MMC_STATUS_RSP_CRC_OK_MASK, MMC_STATUS_CLEAR) | \
	 FIELD_PREP(MMC_STATUS_CMD_SENT_MASK, MMC_STATUS_CLEAR))
#define MMC_STATUS_CLEAR_DATA				\
	(FIELD_PREP(MMC_STATUS_DATA_CRC_FAIL_MASK, MMC_STATUS_CLEAR) | \
	 FIELD_PREP(MMC_STATUS_DATA_TIMEOUT_MASK, MMC_STATUS_CLEAR) | \
	 FIELD_PREP(MMC_STATUS_DATA_CRC_OK_MASK, MMC_STATUS_CLEAR) | \
	 FIELD_PREP(MMC_STATUS_DATA_END_MASK, MMC_STATUS_CLEAR))

#define MMC_INT_EN				0x030
#define MMC_INT_SDIO_IRQ_MASK			BIT(8)
#define MMC_INT_SDIO_IRQ_DISABLE		0x0
#define MMC_INT_SDIO_IRQ_ENABLE			0x1
#define MMC_INT_DISABLE_ALL			0x0

#define MMC_CLOCK_CTRL				0x038
#define MMC_CLOCK_CTRL_OUTPUT_MASK		BIT(10)
#define MMC_CLOCK_CTRL_OUTPUT_ENABLE		0x0
#define MMC_CLOCK_CTRL_OUTPUT_DISABLE		0x1

#define MMC_BUS_WIDTH				0x03c
#define MMC_BUS_WIDTH_MASK			GENMASK(1, 0)
#define MMC_BUS_WIDTH_1BIT			0x0
#define MMC_BUS_WIDTH_4BIT			0x1
#define MMC_BUS_WIDTH_8BIT			0x2

#define MMC_BUS_STATUS				0x040
#define MMC_BUS_STATUS_READY_MASK		BIT(0)
#define MMC_BUS_STATUS_BUSY			0x0
#define MMC_BUS_STATUS_READY			0x1

#define MMC_CLOCK_CTRL2				0x044
#define MMC_CLOCK_CTRL2_INPUT_DELAY_MASK	GENMASK(21, 16)
#define MMC_CLOCK_CTRL2_INPUT_DELAY_NONE	0x0

#define MMC_PHY					0x04c
#define MMC_PHY_RESET_MASK			BIT(0)
#define MMC_PHY_NO_RESET			0x0
#define MMC_PHY_RESET				0x1
#define MMC_PHY_BLOCK_FIFO_MASK			BIT(4)
#define MMC_PHY_BLOCK_FIFO_DISABLE		0x0
#define MMC_PHY_BLOCK_FIFO_ENABLE		0x1

#define MMC_DLY0				0x050
#define MMC_DLY0_SAMPLE_EDGE_MASK		BIT(12)
#define MMC_DLY0_SAMPLE_RISING			0x0
#define MMC_DLY0_SAMPLE_FALLING			0x1
#define MMC_DLY0_SAMPLE_SOURCE_MASK		BIT(13)
#define MMC_DLY0_SAMPLE_PAD			0x0
#define MMC_DLY0_SAMPLE_INTERNAL		0x1
#define MMC_DLY0_SAMPLE_PAD_CLK_MASK		BIT(14)
#define MMC_DLY0_SAMPLE_AFTER_PAD		0x0
#define MMC_DLY0_SAMPLE_BEFORE_PAD		0x1

#define MMC_DLY1				0x054
#define MMC_DLY1_READ_DELAY_MASK		GENMASK(29, 28)
#define MMC_DLY1_READ_DELAY_2			0x2

#define MMC_DLY5				0x064
#define MMC_DLY5_DATA_OUT_INVERT_MASK		BIT(5)
#define MMC_DLY5_DATA_OUT_NORMAL		0x0
#define MMC_DLY5_DATA_OUT_INVERT		0x1
#define MMC_DLY5_CMD_OUT_INVERT_MASK		BIT(6)
#define MMC_DLY5_CMD_OUT_NORMAL			0x0
#define MMC_DLY5_CMD_OUT_INVERT			0x1

#define MMC_DATA_PORT				0x100

#define MMC_DATA_LENGTH				0x104
#define MMC_DATA_LENGTH_MASK			GENMASK(25, 0)

#define MMC_FIFO_STATUS				0x108
#define MMC_FIFO_STATUS_COUNT_MASK		GENMASK(5, 0)
#define MMC_FIFO_STATUS_EMPTY_MASK		BIT(8)
#define MMC_FIFO_STATUS_NOT_EMPTY		0x0
#define MMC_FIFO_STATUS_EMPTY			0x1
#define MMC_FIFO_STATUS_FULL_MASK		BIT(9)
#define MMC_FIFO_STATUS_NOT_FULL		0x0
#define MMC_FIFO_STATUS_FULL			0x1

#define MMC_FIFO_CTRL				0x10c
#define MMC_FIFO_CTRL_EN_MASK			BIT(0)
#define MMC_FIFO_CTRL_DISABLE			0x0
#define MMC_FIFO_CTRL_ENABLE			0x1
#define MMC_FIFO_CTRL_MODE_MASK			BIT(1)
#define MMC_FIFO_CTRL_PIO			0x0
#define MMC_FIFO_CTRL_DMA			0x1
#define MMC_FIFO_CTRL_DIR_MASK			BIT(2)
#define MMC_FIFO_CTRL_READ			0x0
#define MMC_FIFO_CTRL_WRITE			0x1
#define MMC_FIFO_CTRL_IDLE				\
	(FIELD_PREP(MMC_FIFO_CTRL_EN_MASK, MMC_FIFO_CTRL_DISABLE) | \
	 FIELD_PREP(MMC_FIFO_CTRL_MODE_MASK, MMC_FIFO_CTRL_PIO) | \
	 FIELD_PREP(MMC_FIFO_CTRL_DIR_MASK, MMC_FIFO_CTRL_READ))

#define MMC_FIFO_SWITCH				0x1b0
#define MMC_FIFO_SWITCH_DELAY_MASK		BIT(4)
#define MMC_FIFO_SWITCH_DELAY_DISABLE		0x0
#define MMC_FIFO_SWITCH_DELAY_ENABLE		0x1

#define MMC_FIFO_BYTES				64

#define MMC_SDIO_ARG_WRITE_MASK			BIT(31)
#define MMC_SDIO_ARG_READ			0x0
#define MMC_SDIO_ARG_WRITE			0x1
#define MMC_SDIO_ARG_FUNCTION_MASK		GENMASK(30, 28)
#define MMC_SDIO_ARG_FUNCTION_COMMON		0x0
#define MMC_SDIO_ARG_ADDRESS_MASK		GENMASK(25, 9)
#define MMC_SDIO_ARG_DATA_MASK			GENMASK(7, 0)

#define MMC_MAX_REQUEST				SZ_64K

#define MMC_RESET_TIMEOUT_US			10000
#define MMC_CMD_TIMEOUT_US			100000
#define MMC_BUSY_TIMEOUT_MS			1000

#define MMC_MIN_CLOCK				312500
#define MMC_MAX_CLOCK				48000000

struct novatek_mmc {
	struct mmc_host *mmc;
	void __iomem *base;
	struct clk *clk;
	struct reset_control *rst;
	struct regmap *sram;
	int irq;
	int ios_error;
};

static int novatek_mmc_reset(struct novatek_mmc *host)
{
	u32 val;
	int ret;

	writel(FIELD_PREP(MMC_STATUS_DATA_END_MASK, MMC_STATUS_CLEAR),
	       host->base + MMC_STATUS);
	writel(MMC_FIFO_CTRL_IDLE, host->base + MMC_FIFO_CTRL);
	ret = readl_poll_timeout(host->base + MMC_FIFO_CTRL, val,
				 !(val & MMC_FIFO_CTRL_EN_MASK),
				 1, MMC_RESET_TIMEOUT_US);
	if (ret)
		return ret;

	writel(FIELD_PREP(MMC_DATA_CTRL_EN_MASK, MMC_DATA_CTRL_DISABLE),
	       host->base + MMC_DATA_CTRL);
	writel(FIELD_PREP(MMC_CMD_RESET_MASK, MMC_CMD_RESET),
	       host->base + MMC_CMD);
	ret = readl_poll_timeout(host->base + MMC_CMD, val,
				 !(val & MMC_CMD_RESET_MASK),
				 1, MMC_RESET_TIMEOUT_US);

	writel(MMC_STATUS_CLEAR_CMD | MMC_STATUS_CLEAR_DATA,
	       host->base + MMC_STATUS);

	return ret;
}

static int novatek_mmc_init(struct novatek_mmc *host)
{
	u32 val;
	int ret;

	ret = novatek_mmc_reset(host);
	if (ret)
		return ret;

	val = readl(host->base + MMC_FIFO_SWITCH);
	val &= ~MMC_FIFO_SWITCH_DELAY_MASK;
	val |= FIELD_PREP(MMC_FIFO_SWITCH_DELAY_MASK,
			  MMC_FIFO_SWITCH_DELAY_ENABLE);
	writel(val, host->base + MMC_FIFO_SWITCH);

	val = readl(host->base + MMC_CLOCK_CTRL2);
	val &= ~MMC_CLOCK_CTRL2_INPUT_DELAY_MASK;
	val |= FIELD_PREP(MMC_CLOCK_CTRL2_INPUT_DELAY_MASK,
			  MMC_CLOCK_CTRL2_INPUT_DELAY_NONE);
	writel(val, host->base + MMC_CLOCK_CTRL2);

	val = readl(host->base + MMC_DLY0);
	val &= ~(MMC_DLY0_SAMPLE_EDGE_MASK | MMC_DLY0_SAMPLE_SOURCE_MASK |
		 MMC_DLY0_SAMPLE_PAD_CLK_MASK);
	val |= FIELD_PREP(MMC_DLY0_SAMPLE_EDGE_MASK, MMC_DLY0_SAMPLE_RISING) |
	       FIELD_PREP(MMC_DLY0_SAMPLE_SOURCE_MASK, MMC_DLY0_SAMPLE_PAD) |
	       FIELD_PREP(MMC_DLY0_SAMPLE_PAD_CLK_MASK,
			  MMC_DLY0_SAMPLE_BEFORE_PAD);
	writel(val, host->base + MMC_DLY0);

	val = readl(host->base + MMC_DLY5);
	val &= ~(MMC_DLY5_DATA_OUT_INVERT_MASK | MMC_DLY5_CMD_OUT_INVERT_MASK);
	val |= FIELD_PREP(MMC_DLY5_DATA_OUT_INVERT_MASK,
			  MMC_DLY5_DATA_OUT_NORMAL) |
	       FIELD_PREP(MMC_DLY5_CMD_OUT_INVERT_MASK,
			  MMC_DLY5_CMD_OUT_NORMAL);
	writel(val, host->base + MMC_DLY5);

	val = readl(host->base + MMC_PHY) & ~MMC_PHY_BLOCK_FIFO_MASK;
	val |= FIELD_PREP(MMC_PHY_BLOCK_FIFO_MASK, MMC_PHY_BLOCK_FIFO_DISABLE) |
	       FIELD_PREP(MMC_PHY_RESET_MASK, MMC_PHY_RESET);
	writel(val, host->base + MMC_PHY);
	ret = readl_poll_timeout(host->base + MMC_PHY, val,
				 !(val & MMC_PHY_RESET_MASK),
				 1, MMC_RESET_TIMEOUT_US);
	if (ret)
		return ret;

	val = readl(host->base + MMC_DLY1);
	val &= ~MMC_DLY1_READ_DELAY_MASK;
	val |= FIELD_PREP(MMC_DLY1_READ_DELAY_MASK, MMC_DLY1_READ_DELAY_2);
	writel(val, host->base + MMC_DLY1);

	return 0;
}

static void novatek_mmc_command(struct novatek_mmc *host,
				struct mmc_command *cmd)
{
	u32 control, status, mask;
	uint i, timeout_ms;
	int ret;

	cmd->error = 0;

	control = FIELD_PREP(MMC_CMD_INDEX_MASK, cmd->opcode) |
		  FIELD_PREP(MMC_CMD_EN_MASK, MMC_CMD_START) |
		  FIELD_PREP(MMC_CMD_RSP_TIMEOUT_MASK, MMC_CMD_RSP_TIMEOUT_64);
	if (cmd->flags & MMC_RSP_PRESENT)
		control |= FIELD_PREP(MMC_CMD_NEED_RSP_MASK,
				      MMC_CMD_RSP_PRESENT);
	if (cmd->flags & MMC_RSP_136)
		control |= FIELD_PREP(MMC_CMD_LONG_RSP_MASK, MMC_CMD_RSP_LONG);
	if (cmd->opcode == MMC_STOP_TRANSMISSION ||
	    (cmd->opcode == SD_IO_RW_DIRECT &&
	     FIELD_GET(MMC_SDIO_ARG_ADDRESS_MASK, cmd->arg) == SDIO_CCCR_ABORT))
		control |= FIELD_PREP(MMC_CMD_SDIO_DETECT_MASK,
				      MMC_CMD_SDIO_DETECT_ENABLE);

	writel(MMC_STATUS_CLEAR_CMD, host->base + MMC_STATUS);
	writel(cmd->arg, host->base + MMC_ARG);
	writel(control, host->base + MMC_CMD);

	if (cmd->flags & MMC_RSP_PRESENT)
		mask = MMC_STATUS_RSP_CRC_OK_MASK |
		       MMC_STATUS_RSP_CRC_FAIL_MASK |
		       MMC_STATUS_RSP_TIMEOUT_MASK;
	else
		mask = MMC_STATUS_CMD_SENT_MASK;
	ret = readl_poll_timeout(host->base + MMC_STATUS, status,
				 status & mask, 1, MMC_CMD_TIMEOUT_US);
	if (ret || (status & MMC_STATUS_RSP_TIMEOUT_MASK)) {
		cmd->error = -ETIMEDOUT;
		return;
	}
	if ((status & MMC_STATUS_RSP_CRC_FAIL_MASK) &&
	    (cmd->flags & MMC_RSP_CRC)) {
		cmd->error = -EILSEQ;
		return;
	}

	if (cmd->flags & MMC_RSP_136) {
		for (i = 0; i < 4; i++)
			cmd->resp[i] = readl(host->base + MMC_RSP3 - i * 4);
	} else if (cmd->flags & MMC_RSP_PRESENT) {
		cmd->resp[0] = readl(host->base + MMC_RSP0);
	}
	if ((cmd->flags & MMC_RSP_OPCODE) &&
	    FIELD_GET(MMC_RSP_CMD_INDEX_MASK,
		      readl(host->base + MMC_RSP_CMD)) != cmd->opcode) {
		cmd->error = -EILSEQ;
		return;
	}
	if (cmd->flags & MMC_RSP_BUSY) {
		timeout_ms = cmd->busy_timeout ?: MMC_BUSY_TIMEOUT_MS;
		ret = readl_poll_timeout(host->base + MMC_BUS_STATUS, status,
					 status & MMC_BUS_STATUS_READY_MASK, 10,
					 (u64)timeout_ms * USEC_PER_MSEC);
		if (ret)
			cmd->error = ret;
	}
}

static int novatek_mmc_copy(struct sg_mapping_iter *miter, u8 *buf,
			    uint len, bool read)
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

static int novatek_mmc_pio(struct novatek_mmc *host, struct mmc_data *data,
			   ulong deadline)
{
	bool read = data->flags & MMC_DATA_READ;
	struct sg_mapping_iter miter;
	u8 buf[MMC_FIFO_BYTES];
	uint left = data->blksz * data->blocks;
	uint len, words, i;
	u32 status, fifo;
	int ret = 0;

	sg_miter_start(&miter, data->sg, data->sg_len, SG_MITER_LOCAL |
		       (read ? SG_MITER_TO_SG : SG_MITER_FROM_SG));

	while (left) {
		len = min(left, MMC_FIFO_BYTES);
		words = DIV_ROUND_UP(len, 4);
		if (!read) {
			memset(buf, 0, sizeof(buf));
			ret = novatek_mmc_copy(&miter, buf, len, false);
			if (ret)
				break;
		}

		for (;;) {
			status = readl(host->base + MMC_STATUS);
			if (status & MMC_STATUS_DATA_TIMEOUT_MASK) {
				ret = -ETIMEDOUT;
				goto out;
			}
			if (status & MMC_STATUS_DATA_CRC_FAIL_MASK) {
				ret = -EILSEQ;
				goto out;
			}

			fifo = readl(host->base + MMC_FIFO_STATUS);
			if (read ? (len == MMC_FIFO_BYTES ?
				    fifo & MMC_FIFO_STATUS_FULL_MASK :
				    FIELD_GET(MMC_FIFO_STATUS_COUNT_MASK,
					      fifo) == words) :
				   fifo & MMC_FIFO_STATUS_EMPTY_MASK)
				break;

			if (time_after_eq(jiffies, deadline)) {
				ret = -ETIMEDOUT;
				goto out;
			}

			usleep_range(10, 20);
		}

		for (i = 0; i < words; i++) {
			if (read)
				put_unaligned_le32(
					readl(host->base + MMC_DATA_PORT),
					buf + i * 4);
			else
				writel(get_unaligned_le32(buf + i * 4),
				       host->base + MMC_DATA_PORT);
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
		goto out;

	for (;;) {
		status = readl(host->base + MMC_STATUS);
		if (status & MMC_STATUS_DATA_TIMEOUT_MASK) {
			ret = -ETIMEDOUT;
			break;
		}
		if (status & MMC_STATUS_DATA_CRC_FAIL_MASK) {
			ret = -EILSEQ;
			break;
		}
		if ((status & (MMC_STATUS_DATA_END_MASK |
			       MMC_STATUS_DATA_CRC_OK_MASK)) &&
		    (read || (readl(host->base + MMC_BUS_STATUS) &
			      MMC_BUS_STATUS_READY_MASK)))
			break;

		if (time_after_eq(jiffies, deadline)) {
			ret = -ETIMEDOUT;
			break;
		}

		usleep_range(10, 20);
	}

out:
	sg_miter_stop(&miter);

	return ret;
}

static void novatek_mmc_request(struct mmc_host *mmc, struct mmc_request *mrq)
{
	struct novatek_mmc *host = mmc_priv(mmc);
	struct mmc_data *data = mrq->data;
	struct mmc_command abort = {};
	u64 cycles, timeout_ms;
	ulong deadline = 0;
	u32 fifo;
	uint len;
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

	if (host->ios_error || !mmc->actual_clock) {
		mrq->cmd->error = host->ios_error ?: -EIO;
		goto done;
	}

	if (data) {
		ret = readl_poll_timeout(host->base + MMC_BUS_STATUS, fifo,
					 fifo & MMC_BUS_STATUS_READY_MASK, 10,
					 MMC_BUSY_TIMEOUT_MS * USEC_PER_MSEC);
		if (ret) {
			mrq->cmd->error = ret;
			goto done;
		}
	}

	ret = novatek_mmc_reset(host);
	if (ret) {
		host->ios_error = ret;
		mrq->cmd->error = ret;
		goto done;
	}

	if (mrq->sbc) {
		novatek_mmc_command(host, mrq->sbc);
		if (mrq->sbc->error)
			goto recover;
	}

	if (data) {
		len = data->blksz * data->blocks;
		cycles = DIV_ROUND_UP_ULL((u64)data->timeout_ns *
					mmc->actual_clock,
					NSEC_PER_SEC) + data->timeout_clks;
		writel(clamp_val(cycles, 1, U32_MAX),
		       host->base + MMC_DATA_TIMER);

		/*
		 * Budget each block's access timeout plus wire time and
		 * scheduling.
		 */
		timeout_ms = DIV_ROUND_UP_ULL((cycles * data->blocks +
					     (u64)len * 8) * MSEC_PER_SEC,
					     mmc->actual_clock) + 1000;
		deadline = jiffies +
			   msecs_to_jiffies(min_t(u64, timeout_ms, U32_MAX));

		len = mmc->actual_clock >= 48000000 ? 3 :
		      mmc->actual_clock >= 24000000 ? 6 :
		      mmc->actual_clock >= 12000000 ? 9 : 21;
		for (len += 2; len; len--)
			readl(host->base + MMC_CMD);

		writel(FIELD_PREP(MMC_DATA_CTRL_BLOCK_SIZE_MASK, data->blksz) |
		       FIELD_PREP(MMC_DATA_CTRL_EN_MASK, MMC_DATA_CTRL_ENABLE),
		       host->base + MMC_DATA_CTRL);
		writel(FIELD_PREP(MMC_DATA_LENGTH_MASK,
				  data->blksz * data->blocks),
		       host->base + MMC_DATA_LENGTH);

		fifo = FIELD_PREP(MMC_FIFO_CTRL_DIR_MASK,
				 data->flags & MMC_DATA_WRITE ?
				 MMC_FIFO_CTRL_WRITE : MMC_FIFO_CTRL_READ) |
		       FIELD_PREP(MMC_FIFO_CTRL_MODE_MASK, MMC_FIFO_CTRL_PIO) |
		       FIELD_PREP(MMC_FIFO_CTRL_EN_MASK, MMC_FIFO_CTRL_DISABLE);
		writel(fifo, host->base + MMC_FIFO_CTRL);
		writel(fifo | FIELD_PREP(MMC_FIFO_CTRL_EN_MASK,
					 MMC_FIFO_CTRL_ENABLE),
		       host->base + MMC_FIFO_CTRL);
	}

	novatek_mmc_command(host, mrq->cmd);
	if (mrq->cmd->error)
		goto recover;

	if (data) {
		data->error = novatek_mmc_pio(host, data, deadline);
		if (data->error)
			goto recover;
		data->bytes_xfered = data->blksz * data->blocks;
	}

	if (mrq->stop && !mrq->sbc)
		novatek_mmc_command(host, mrq->stop);
	if (mrq->stop && mrq->stop->error)
		goto recover;

	goto done;

recover:
	ret = novatek_mmc_reset(host);
	if (ret) {
		host->ios_error = ret;
		goto done;
	}

	if (mrq->sbc && mrq->sbc->error)
		goto done;

	if (data && mrq->cmd->opcode == SD_IO_RW_EXTENDED) {
		abort.opcode = SD_IO_RW_DIRECT;
		abort.arg = FIELD_PREP(MMC_SDIO_ARG_WRITE_MASK,
				       MMC_SDIO_ARG_WRITE) |
			    FIELD_PREP(MMC_SDIO_ARG_FUNCTION_MASK,
				       MMC_SDIO_ARG_FUNCTION_COMMON) |
			    FIELD_PREP(MMC_SDIO_ARG_ADDRESS_MASK,
				       SDIO_CCCR_ABORT) |
			    FIELD_PREP(MMC_SDIO_ARG_DATA_MASK,
				       FIELD_GET(MMC_SDIO_ARG_FUNCTION_MASK,
						 mrq->cmd->arg));
		abort.flags = MMC_RSP_R5 | MMC_CMD_AC;
		novatek_mmc_command(host, &abort);
	} else if (mrq->stop && !mrq->stop->error) {
		novatek_mmc_command(host, mrq->stop);
	}

done:
	mmc_request_done(mmc, mrq);
}

static void novatek_mmc_set_ios(struct mmc_host *mmc, struct mmc_ios *ios)
{
	struct novatek_mmc *host = mmc_priv(mmc);
	ulong rate;
	u32 width;
	int ret;

	writel(FIELD_PREP(MMC_CLOCK_CTRL_OUTPUT_MASK,
			  MMC_CLOCK_CTRL_OUTPUT_DISABLE),
	       host->base + MMC_CLOCK_CTRL);
	mmc->actual_clock = 0;

	if (ios->power_mode == MMC_POWER_OFF) {
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
		goto error;

	if (ios->power_mode == MMC_POWER_UP && !IS_ERR(mmc->supply.vqmmc)) {
		ret = mmc_regulator_set_vqmmc(mmc, ios);
		if (ret < 0)
			goto error;
	}

	ret = mmc_regulator_enable_vqmmc(mmc);
	if (ret)
		goto error;

	if (ios->power_mode == MMC_POWER_UP) {
		ret = novatek_mmc_init(host);
		if (ret)
			goto error;
	}

	width = ios->bus_width == MMC_BUS_WIDTH_8 ? MMC_BUS_WIDTH_8BIT :
		ios->bus_width == MMC_BUS_WIDTH_4 ? MMC_BUS_WIDTH_4BIT :
		MMC_BUS_WIDTH_1BIT;
	writel(FIELD_PREP(MMC_BUS_WIDTH_MASK, width),
	       host->base + MMC_BUS_WIDTH);

	if (ios->clock) {
		ret = clk_set_rate(host->clk, ios->clock);
		if (ret)
			goto error;

		rate = clk_get_rate(host->clk);
		if (!rate || rate > ios->clock) {
			ret = -EINVAL;
			goto error;
		}
		mmc->actual_clock = rate;
		writel(FIELD_PREP(MMC_CLOCK_CTRL_OUTPUT_MASK,
				  MMC_CLOCK_CTRL_OUTPUT_ENABLE),
		       host->base + MMC_CLOCK_CTRL);
	}

	host->ios_error = 0;

	return;

error:
	host->ios_error = ret;
	dev_err(mmc_dev(mmc), "failed to configure bus: %d\n", ret);
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

	return !(readl(host->base + MMC_BUS_STATUS) &
		 MMC_BUS_STATUS_READY_MASK);
}

static void novatek_mmc_enable_sdio_irq(struct mmc_host *mmc, int enable)
{
	struct novatek_mmc *host = mmc_priv(mmc);

	writel(FIELD_PREP(MMC_INT_SDIO_IRQ_MASK,
			  enable ? MMC_INT_SDIO_IRQ_ENABLE :
			  MMC_INT_SDIO_IRQ_DISABLE),
	       host->base + MMC_INT_EN);
}

static irqreturn_t novatek_mmc_irq(int irq, void *data)
{
	struct novatek_mmc *host = data;
	u32 status;

	status = readl(host->base + MMC_STATUS) &
		 readl(host->base + MMC_INT_EN);
	if (!(status & MMC_STATUS_SDIO_IRQ_MASK))
		return IRQ_NONE;

	writel(MMC_INT_DISABLE_ALL, host->base + MMC_INT_EN);
	writel(FIELD_PREP(MMC_STATUS_SDIO_IRQ_MASK, MMC_STATUS_CLEAR),
	       host->base + MMC_STATUS);

	mmc_signal_sdio_irq(host->mmc);

	return IRQ_HANDLED;
}

static const struct mmc_host_ops novatek_mmc_ops = {
	.request = novatek_mmc_request,
	.set_ios = novatek_mmc_set_ios,
	.get_cd = novatek_mmc_get_cd,
	.get_ro = mmc_gpio_get_ro,
	.card_busy = novatek_mmc_card_busy,
	.enable_sdio_irq = novatek_mmc_enable_sdio_irq,
};

static const struct of_device_id novatek_mmc_of_match[] = {
	{ .compatible = "novatek,na51089-mmc" },
	{}
};
MODULE_DEVICE_TABLE(of, novatek_mmc_of_match);

static int novatek_mmc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct novatek_mmc *host;
	struct mmc_host *mmc;
	struct resource *res;
	int ret;

	mmc = mmc_alloc_host(sizeof(*host), dev);
	if (!mmc)
		return -ENOMEM;
	host = mmc_priv(mmc);
	host->mmc = mmc;
	platform_set_drvdata(pdev, host);

	host->base = devm_platform_get_and_ioremap_resource(pdev, 0, &res);
	if (IS_ERR(host->base)) {
		ret = PTR_ERR(host->base);
		goto free_host;
	}
	if (resource_size(res) < MMC_FIFO_SWITCH + sizeof(u32)) {
		ret = -EINVAL;
		goto free_host;
	}

	host->irq = platform_get_irq(pdev, 0);
	if (host->irq < 0) {
		ret = host->irq;
		goto free_host;
	}

	host->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(host->clk)) {
		ret = dev_err_probe(dev, PTR_ERR(host->clk),
				    "failed to get clock\n");
		goto free_host;
	}

	host->rst = devm_reset_control_get_exclusive(dev, NULL);
	if (IS_ERR(host->rst)) {
		ret = dev_err_probe(dev, PTR_ERR(host->rst),
				    "failed to get reset\n");
		goto free_host;
	}

	if (device_property_present(dev, "regmap")) {
		host->sram = syscon_regmap_lookup_by_phandle(dev->of_node,
							   "regmap");
		if (IS_ERR(host->sram)) {
			ret = dev_err_probe(dev, PTR_ERR(host->sram),
					    "failed to get SRAM controller\n");
			goto free_host;
		}
	}

	mmc->ops = &novatek_mmc_ops;
	mmc->f_min = MMC_MIN_CLOCK;
	mmc->f_max = MMC_MAX_CLOCK;
	mmc->ocr_avail = MMC_VDD_32_33 | MMC_VDD_33_34;
	mmc->caps = MMC_CAP_CMD23 | MMC_CAP_WAIT_WHILE_BUSY | MMC_CAP_SDIO_IRQ;
	mmc->max_busy_timeout = 0;

	mmc->max_req_size = MMC_MAX_REQUEST;
	mmc->max_seg_size = MMC_MAX_REQUEST;
	mmc->max_segs = 128;
	mmc->max_blk_size = 2048;
	mmc->max_blk_count = MMC_MAX_REQUEST / 512;

	ret = mmc_regulator_get_supply(mmc);
	if (ret)
		goto free_host;
	mmc->ocr_avail &= MMC_VDD_32_33 | MMC_VDD_33_34;
	if (!mmc->ocr_avail) {
		ret = dev_err_probe(dev, -EINVAL,
				    "a 3.3 V card supply is required\n");
		goto free_host;
	}

	ret = mmc_of_parse(mmc);
	if (ret)
		goto free_host;
	mmc->f_max = min(mmc->f_max, MMC_MAX_CLOCK);
	mmc->caps &= ~(MMC_CAP_UHS | MMC_CAP_DDR);
	mmc->caps2 &= ~(MMC_CAP2_HS200 | MMC_CAP2_HS400 | MMC_CAP2_HS400_ES);

	if (!mmc_host_can_gpio_cd(mmc) && !(mmc->caps & MMC_CAP_NONREMOVABLE))
		mmc->caps |= MMC_CAP_NEEDS_POLL;

	ret = reset_control_assert(host->rst);
	if (ret)
		goto free_host;

	if (host->sram) {
		ret = regmap_update_bits(host->sram, NOVATEK_SRAM_SHUTDOWN,
					 NOVATEK_SRAM_SDIO3_SD_MASK,
					 FIELD_PREP(NOVATEK_SRAM_SDIO3_SD_MASK,
						    NOVATEK_SRAM_SDIO3_ENABLE));
		if (ret)
			goto free_host;
	}

	ret = clk_set_rate(host->clk, 400000);
	if (ret)
		goto disable_sram;

	ret = clk_prepare_enable(host->clk);
	if (ret)
		goto disable_sram;

	ret = reset_control_deassert(host->rst);
	if (ret)
		goto disable_clk;

	writel(MMC_INT_DISABLE_ALL, host->base + MMC_INT_EN);
	writel(FIELD_PREP(MMC_CLOCK_CTRL_OUTPUT_MASK,
			  MMC_CLOCK_CTRL_OUTPUT_DISABLE),
	       host->base + MMC_CLOCK_CTRL);

	ret = novatek_mmc_init(host);
	if (ret)
		goto assert_reset;

	ret = devm_request_irq(dev, host->irq, novatek_mmc_irq, 0,
			       dev_name(dev), host);
	if (ret)
		goto assert_reset;

	ret = mmc_add_host(mmc);
	if (ret)
		goto free_irq;

	return 0;

free_irq:
	writel(MMC_INT_DISABLE_ALL, host->base + MMC_INT_EN);
	devm_free_irq(dev, host->irq, host);
assert_reset:
	reset_control_assert(host->rst);
disable_clk:
	clk_disable_unprepare(host->clk);
disable_sram:
	if (host->sram)
		regmap_update_bits(host->sram, NOVATEK_SRAM_SHUTDOWN,
				   NOVATEK_SRAM_SDIO3_SD_MASK,
				   FIELD_PREP(NOVATEK_SRAM_SDIO3_SD_MASK,
					      NOVATEK_SRAM_SDIO3_DISABLE));
free_host:
	mmc_free_host(mmc);

	return ret;
}

static void novatek_mmc_remove(struct platform_device *pdev)
{
	struct novatek_mmc *host = platform_get_drvdata(pdev);
	int ret;

	mmc_remove_host(host->mmc);

	writel(MMC_INT_DISABLE_ALL, host->base + MMC_INT_EN);
	devm_free_irq(&pdev->dev, host->irq, host);

	reset_control_assert(host->rst);
	clk_disable_unprepare(host->clk);

	if (host->sram) {
		ret = regmap_update_bits(
			host->sram, NOVATEK_SRAM_SHUTDOWN,
			NOVATEK_SRAM_SDIO3_SD_MASK,
			FIELD_PREP(NOVATEK_SRAM_SDIO3_SD_MASK,
				   NOVATEK_SRAM_SDIO3_DISABLE));
		if (ret)
			dev_err(&pdev->dev,
				"failed to shut down SRAM: %d\n", ret);
	}

	mmc_free_host(host->mmc);
}

static struct platform_driver novatek_mmc_driver = {
	.probe = novatek_mmc_probe,
	.remove = novatek_mmc_remove,
	.driver = {
		.name = "novatek-mmc",
		.of_match_table = novatek_mmc_of_match,
	},
};
module_platform_driver(novatek_mmc_driver);

MODULE_DESCRIPTION("Novatek NA51089 SD/SDIO/eMMC host controller");
MODULE_AUTHOR("Yudong Zhang <mtwget@gmail.com>");
MODULE_LICENSE("GPL");
