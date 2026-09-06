// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2026 Yudong Zhang <mtwget@gmail.com>
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/ktime.h>
#include <linux/mfd/syscon.h>
#include <linux/mfd/syscon/novatek-sramctrl.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/regmap.h>
#include <linux/reset.h>
#include <linux/sched.h>
#include <linux/sizes.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi-mem.h>
#include <linux/units.h>

#define NOVATEK_FSPI_MODULE			0x00
#define NOVATEK_FSPI_MODULE_ROW_BYTES_MASK	GENMASK(3, 2)
#define NOVATEK_FSPI_MODULE_TYPE_MASK		BIT(19)
#define NOVATEK_FSPI_MODULE_SERIAL		0x1
#define NOVATEK_FSPI_MODULE_FLASH_MASK		BIT(20)
#define NOVATEK_FSPI_MODULE_NOR			0x1
/*
 * Use NOR mode for split command/address/data transfers, including SPI NAND,
 * without the controller's NAND-specific command sequences.
 */
#define NOVATEK_FSPI_MODULE_RAW			\
	(FIELD_PREP(NOVATEK_FSPI_MODULE_TYPE_MASK, \
		    NOVATEK_FSPI_MODULE_SERIAL) | \
	 FIELD_PREP(NOVATEK_FSPI_MODULE_FLASH_MASK, NOVATEK_FSPI_MODULE_NOR))

#define NOVATEK_FSPI_PHY			0x08
#define NOVATEK_FSPI_PHY_RESET_MASK		BIT(0)
#define NOVATEK_FSPI_PHY_NO_RESET		0x0
#define NOVATEK_FSPI_PHY_RESET			0x1
#define NOVATEK_FSPI_PHY_SAMPLE_INV_MASK	BIT(2)
#define NOVATEK_FSPI_PHY_SAMPLE_NORMAL		0x0
#define NOVATEK_FSPI_PHY_SAMPLE_INVERT		0x1
#define NOVATEK_FSPI_PHY_PAD_CLK_MASK		BIT(20)
#define NOVATEK_FSPI_PHY_PAD_CLK_UNSELECTED	0x0
#define NOVATEK_FSPI_PHY_PAD_CLK_SELECTED	0x1

#define NOVATEK_FSPI_PHY_DELAY			0x0c
#define NOVATEK_FSPI_PHY_INPUT_DELAY_MASK	GENMASK(29, 24)

#define NOVATEK_FSPI_CONFIG			0x14
#define NOVATEK_FSPI_CONFIG_CS_MODE_MASK	BIT(1)
#define NOVATEK_FSPI_CONFIG_AUTO_CS		0x0
#define NOVATEK_FSPI_CONFIG_MANUAL_CS		0x1
#define NOVATEK_FSPI_CONFIG_CS_LEVEL_MASK	BIT(4)
#define NOVATEK_FSPI_CONFIG_CS_LOW		0x0
#define NOVATEK_FSPI_CONFIG_CS_HIGH		0x1
#define NOVATEK_FSPI_CONFIG_WIDTH_MASK		GENMASK(10, 9)
#define NOVATEK_FSPI_CONFIG_SINGLE		0x0
#define NOVATEK_FSPI_CONFIG_DUAL		0x1
#define NOVATEK_FSPI_CONFIG_QUAD		0x2
#define NOVATEK_FSPI_CONFIG_IO_ORDER_MASK	BIT(11)
#define NOVATEK_FSPI_CONFIG_IO_DESCENDING	0x0
#define NOVATEK_FSPI_CONFIG_IO_ASCENDING	0x1
#define NOVATEK_FSPI_CONFIG_PULL_MASK		BIT(12)
#define NOVATEK_FSPI_CONFIG_PULL_DISABLE	0x0
#define NOVATEK_FSPI_CONFIG_PULL_ENABLE		0x1
#define NOVATEK_FSPI_CONFIG_IDLE		\
	(FIELD_PREP(NOVATEK_FSPI_CONFIG_CS_MODE_MASK, \
		    NOVATEK_FSPI_CONFIG_MANUAL_CS) | \
	 FIELD_PREP(NOVATEK_FSPI_CONFIG_CS_LEVEL_MASK, \
		    NOVATEK_FSPI_CONFIG_CS_HIGH) | \
	 FIELD_PREP(NOVATEK_FSPI_CONFIG_WIDTH_MASK, \
		    NOVATEK_FSPI_CONFIG_SINGLE) | \
	 FIELD_PREP(NOVATEK_FSPI_CONFIG_IO_ORDER_MASK, \
		    NOVATEK_FSPI_CONFIG_IO_ASCENDING) | \
	 FIELD_PREP(NOVATEK_FSPI_CONFIG_PULL_MASK, \
		    NOVATEK_FSPI_CONFIG_PULL_ENABLE))

#define NOVATEK_FSPI_CTRL			0x20
#define NOVATEK_FSPI_CTRL_OP_MASK		GENMASK(5, 0)
#define NOVATEK_FSPI_OP_COMMAND			0x07
#define NOVATEK_FSPI_OP_COMMAND_ADDRESS		0x0c
#define NOVATEK_FSPI_OP_READ			0x18
#define NOVATEK_FSPI_OP_WRITE			0x19
#define NOVATEK_FSPI_OP_DUMMY			0x1e
#define NOVATEK_FSPI_CTRL_START_MASK		BIT(12)
#define NOVATEK_FSPI_CTRL_NO_START		0x0
#define NOVATEK_FSPI_CTRL_START			0x1
#define NOVATEK_FSPI_CTRL_RESET_MASK		BIT(15)
#define NOVATEK_FSPI_CTRL_NO_RESET		0x0
#define NOVATEK_FSPI_CTRL_RESET			0x1

#define NOVATEK_FSPI_TIMING			0x24
#define NOVATEK_FSPI_TIMING_TSLCH_MASK		GENMASK(3, 0)
#define NOVATEK_FSPI_TIMING_TSHCH_MASK		GENMASK(7, 4)
#define NOVATEK_FSPI_TIMING_TSHSL_MASK		GENMASK(15, 8)

#define NOVATEK_FSPI_COMMAND			0x34

#define NOVATEK_FSPI_ROW_ADDRESS		0x3c

#define NOVATEK_FSPI_INT_ENABLE			0x44
#define NOVATEK_FSPI_INT_COMPLETE_MASK		BIT(12)
#define NOVATEK_FSPI_INT_COMPLETE_DISABLE	0x0
#define NOVATEK_FSPI_INT_COMPLETE_ENABLE	0x1
#define NOVATEK_FSPI_INT_PRI_ECC_MASK		BIT(13)
#define NOVATEK_FSPI_INT_PRI_ECC_DISABLE	0x0
#define NOVATEK_FSPI_INT_PRI_ECC_ENABLE		0x1
#define NOVATEK_FSPI_INT_STATUS_FAIL_MASK	BIT(14)
#define NOVATEK_FSPI_INT_STATUS_FAIL_DISABLE	0x0
#define NOVATEK_FSPI_INT_STATUS_FAIL_ENABLE	0x1
#define NOVATEK_FSPI_INT_TIMEOUT_MASK		BIT(15)
#define NOVATEK_FSPI_INT_TIMEOUT_DISABLE	0x0
#define NOVATEK_FSPI_INT_TIMEOUT_ENABLE		0x1
#define NOVATEK_FSPI_INT_SEC_ECC_MASK		BIT(16)
#define NOVATEK_FSPI_INT_SEC_ECC_DISABLE	0x0
#define NOVATEK_FSPI_INT_SEC_ECC_ENABLE		0x1
#define NOVATEK_FSPI_INT_PROTECT1_MASK		BIT(17)
#define NOVATEK_FSPI_INT_PROTECT1_DISABLE	0x0
#define NOVATEK_FSPI_INT_PROTECT1_ENABLE	0x1
#define NOVATEK_FSPI_INT_PROTECT2_MASK		BIT(18)
#define NOVATEK_FSPI_INT_PROTECT2_DISABLE	0x0
#define NOVATEK_FSPI_INT_PROTECT2_ENABLE	0x1
#define NOVATEK_FSPI_INT_DISABLE_ALL		\
	(FIELD_PREP(NOVATEK_FSPI_INT_COMPLETE_MASK, \
		    NOVATEK_FSPI_INT_COMPLETE_DISABLE) | \
	 FIELD_PREP(NOVATEK_FSPI_INT_PRI_ECC_MASK, \
		    NOVATEK_FSPI_INT_PRI_ECC_DISABLE) | \
	 FIELD_PREP(NOVATEK_FSPI_INT_STATUS_FAIL_MASK, \
		    NOVATEK_FSPI_INT_STATUS_FAIL_DISABLE) | \
	 FIELD_PREP(NOVATEK_FSPI_INT_TIMEOUT_MASK, \
		    NOVATEK_FSPI_INT_TIMEOUT_DISABLE) | \
	 FIELD_PREP(NOVATEK_FSPI_INT_SEC_ECC_MASK, \
		    NOVATEK_FSPI_INT_SEC_ECC_DISABLE) | \
	 FIELD_PREP(NOVATEK_FSPI_INT_PROTECT1_MASK, \
		    NOVATEK_FSPI_INT_PROTECT1_DISABLE) | \
	 FIELD_PREP(NOVATEK_FSPI_INT_PROTECT2_MASK, \
		    NOVATEK_FSPI_INT_PROTECT2_DISABLE))

#define NOVATEK_FSPI_STATUS			0x48
#define NOVATEK_FSPI_STATUS_COMPLETE_MASK	BIT(12)
#define NOVATEK_FSPI_STATUS_COMPLETE_NONE	0x0
#define NOVATEK_FSPI_STATUS_COMPLETE_SET	0x1
#define NOVATEK_FSPI_STATUS_PRI_ECC_MASK	BIT(13)
#define NOVATEK_FSPI_STATUS_PRI_ECC_NONE	0x0
#define NOVATEK_FSPI_STATUS_PRI_ECC_SET		0x1
#define NOVATEK_FSPI_STATUS_FAIL_MASK		BIT(14)
#define NOVATEK_FSPI_STATUS_FAIL_NONE		0x0
#define NOVATEK_FSPI_STATUS_FAIL_SET		0x1
#define NOVATEK_FSPI_STATUS_TIMEOUT_MASK	BIT(15)
#define NOVATEK_FSPI_STATUS_TIMEOUT_NONE	0x0
#define NOVATEK_FSPI_STATUS_TIMEOUT_SET		0x1
#define NOVATEK_FSPI_STATUS_SEC_ECC_MASK	BIT(16)
#define NOVATEK_FSPI_STATUS_SEC_ECC_NONE	0x0
#define NOVATEK_FSPI_STATUS_SEC_ECC_SET		0x1
#define NOVATEK_FSPI_STATUS_PROTECT1_MASK	BIT(17)
#define NOVATEK_FSPI_STATUS_PROTECT1_NONE	0x0
#define NOVATEK_FSPI_STATUS_PROTECT1_SET	0x1
#define NOVATEK_FSPI_STATUS_PROTECT2_MASK	BIT(18)
#define NOVATEK_FSPI_STATUS_PROTECT2_NONE	0x0
#define NOVATEK_FSPI_STATUS_PROTECT2_SET	0x1
#define NOVATEK_FSPI_STATUS_KEEP		0x0
#define NOVATEK_FSPI_STATUS_CLEAR		0x1
#define NOVATEK_FSPI_STATUS_ALL_MASK		\
	(NOVATEK_FSPI_STATUS_COMPLETE_MASK | \
	 NOVATEK_FSPI_STATUS_PRI_ECC_MASK | \
	 NOVATEK_FSPI_STATUS_FAIL_MASK | \
	 NOVATEK_FSPI_STATUS_TIMEOUT_MASK | \
	 NOVATEK_FSPI_STATUS_SEC_ECC_MASK | \
	 NOVATEK_FSPI_STATUS_PROTECT1_MASK | \
	 NOVATEK_FSPI_STATUS_PROTECT2_MASK)
#define NOVATEK_FSPI_STATUS_CLEAR_ALL		\
	(FIELD_PREP(NOVATEK_FSPI_STATUS_COMPLETE_MASK, \
		    NOVATEK_FSPI_STATUS_CLEAR) | \
	 FIELD_PREP(NOVATEK_FSPI_STATUS_PRI_ECC_MASK, \
		    NOVATEK_FSPI_STATUS_CLEAR) | \
	 FIELD_PREP(NOVATEK_FSPI_STATUS_FAIL_MASK, \
		    NOVATEK_FSPI_STATUS_CLEAR) | \
	 FIELD_PREP(NOVATEK_FSPI_STATUS_TIMEOUT_MASK, \
		    NOVATEK_FSPI_STATUS_CLEAR) | \
	 FIELD_PREP(NOVATEK_FSPI_STATUS_SEC_ECC_MASK, \
		    NOVATEK_FSPI_STATUS_CLEAR) | \
	 FIELD_PREP(NOVATEK_FSPI_STATUS_PROTECT1_MASK, \
		    NOVATEK_FSPI_STATUS_CLEAR) | \
	 FIELD_PREP(NOVATEK_FSPI_STATUS_PROTECT2_MASK, \
		    NOVATEK_FSPI_STATUS_CLEAR))
#define NOVATEK_FSPI_STATUS_ERROR_MASK		\
	(NOVATEK_FSPI_STATUS_FAIL_MASK | \
	 NOVATEK_FSPI_STATUS_TIMEOUT_MASK | \
	 NOVATEK_FSPI_STATUS_PROTECT1_MASK | \
	 NOVATEK_FSPI_STATUS_PROTECT2_MASK)

#define NOVATEK_FSPI_DUMMY			0xc0
#define NOVATEK_FSPI_DUMMY_CLOCK_MASK		GENMASK(2, 0)

#define NOVATEK_FSPI_DATA			0x100
#define NOVATEK_FSPI_DATA_LENGTH		0x104
#define NOVATEK_FSPI_DATA_LENGTH_MASK		GENMASK(25, 0)

#define NOVATEK_FSPI_FIFO_STATUS		0x108
#define NOVATEK_FSPI_FIFO_COUNT_MASK		GENMASK(4, 0)
#define NOVATEK_FSPI_FIFO_EMPTY_MASK		BIT(8)
#define NOVATEK_FSPI_FIFO_NOT_EMPTY		0x0
#define NOVATEK_FSPI_FIFO_EMPTY			0x1
#define NOVATEK_FSPI_FIFO_FULL_MASK		BIT(9)
#define NOVATEK_FSPI_FIFO_NOT_FULL		0x0
#define NOVATEK_FSPI_FIFO_FULL			0x1

#define NOVATEK_FSPI_FIFO_CTRL			0x10c
#define NOVATEK_FSPI_FIFO_EN_MASK		BIT(0)
#define NOVATEK_FSPI_FIFO_DISABLE		0x0
#define NOVATEK_FSPI_FIFO_ENABLE		0x1
#define NOVATEK_FSPI_FIFO_MODE_MASK		BIT(1)
#define NOVATEK_FSPI_FIFO_PIO			0x0
#define NOVATEK_FSPI_FIFO_DMA			0x1
#define NOVATEK_FSPI_FIFO_DIR_MASK		BIT(2)
#define NOVATEK_FSPI_FIFO_READ			0x0
#define NOVATEK_FSPI_FIFO_WRITE			0x1
#define NOVATEK_FSPI_FIFO_IDLE			\
	(FIELD_PREP(NOVATEK_FSPI_FIFO_EN_MASK, NOVATEK_FSPI_FIFO_DISABLE) | \
	 FIELD_PREP(NOVATEK_FSPI_FIFO_MODE_MASK, NOVATEK_FSPI_FIFO_PIO) | \
	 FIELD_PREP(NOVATEK_FSPI_FIFO_DIR_MASK, NOVATEK_FSPI_FIFO_READ))

#define NOVATEK_FSPI_FIFO_BANK_SIZE		64U

#define NOVATEK_FSPI_MAX_TRANSFER		SZ_64K

#define NOVATEK_FSPI_MIN_SPEED_HZ		7500000
#define NOVATEK_FSPI_MAX_SPEED_HZ		48000000

#define NOVATEK_FSPI_POLL_FAST_US		100
#define NOVATEK_FSPI_POLL_SLEEP_US		20
#define NOVATEK_FSPI_TIMEOUT_US			100000

struct novatek_fspi {
	struct device *dev;
	void __iomem *base;
	struct clk *clk;
	struct reset_control *reset;
	struct regmap *sram;
	bool fault;
};

#define novatek_fspi_poll_timeout(addr, val, cond) \
({ \
	const void __iomem *__fspi_addr = (addr); \
	ktime_t __fspi_deadline; \
	s64 __fspi_remaining; \
	int __fspi_ret; \
\
	__fspi_deadline = ktime_add_us(ktime_get(), NOVATEK_FSPI_TIMEOUT_US); \
	__fspi_ret = readl_poll_timeout(__fspi_addr, val, cond, 0, \
					NOVATEK_FSPI_POLL_FAST_US); \
	if (__fspi_ret) { \
		__fspi_remaining = \
			ktime_us_delta(__fspi_deadline, ktime_get()); \
		if (__fspi_remaining > 0) \
			__fspi_ret = readl_poll_timeout( \
				__fspi_addr, val, cond, \
				NOVATEK_FSPI_POLL_SLEEP_US, __fspi_remaining); \
	} \
\
	__fspi_ret; \
})

static int novatek_fspi_enable_sram(struct novatek_fspi *fspi)
{
	return regmap_update_bits(fspi->sram, NOVATEK_SRAM_SHUTDOWN,
				  NOVATEK_SRAM_FSPI_SD_MASK,
				  FIELD_PREP(NOVATEK_SRAM_FSPI_SD_MASK,
					     NOVATEK_SRAM_FSPI_ENABLE));
}

static int novatek_fspi_disable_sram(struct novatek_fspi *fspi)
{
	return regmap_update_bits(fspi->sram, NOVATEK_SRAM_SHUTDOWN,
				  NOVATEK_SRAM_FSPI_SD_MASK,
				  FIELD_PREP(NOVATEK_SRAM_FSPI_SD_MASK,
					     NOVATEK_SRAM_FSPI_DISABLE));
}

static void novatek_fspi_disable_sram_action(void *data)
{
	struct novatek_fspi *fspi = data;
	int ret;

	ret = novatek_fspi_disable_sram(fspi);
	if (ret)
		dev_err(fspi->dev, "failed to shut down SRAM: %d\n", ret);
}

static int novatek_fspi_hw_init(struct novatek_fspi *fspi)
{
	u32 value;
	int ret;

	fspi->fault = true;

	writel(NOVATEK_FSPI_INT_DISABLE_ALL,
	       fspi->base + NOVATEK_FSPI_INT_ENABLE);

	writel(NOVATEK_FSPI_FIFO_IDLE, fspi->base + NOVATEK_FSPI_FIFO_CTRL);
	ret = readl_poll_timeout(fspi->base + NOVATEK_FSPI_FIFO_CTRL, value,
				 !(value & NOVATEK_FSPI_FIFO_EN_MASK), 1,
				 NOVATEK_FSPI_TIMEOUT_US);
	if (!ret) {
		writel(FIELD_PREP(NOVATEK_FSPI_CTRL_RESET_MASK,
				  NOVATEK_FSPI_CTRL_RESET),
		       fspi->base + NOVATEK_FSPI_CTRL);
		ret = readl_poll_timeout(
			fspi->base + NOVATEK_FSPI_CTRL, value,
			!(value & NOVATEK_FSPI_CTRL_RESET_MASK), 1,
			NOVATEK_FSPI_TIMEOUT_US);
	}
	if (ret)
		ret = reset_control_reset(fspi->reset);
	if (ret)
		return ret;

	writel(NOVATEK_FSPI_INT_DISABLE_ALL,
	       fspi->base + NOVATEK_FSPI_INT_ENABLE);
	writel(NOVATEK_FSPI_FIFO_IDLE, fspi->base + NOVATEK_FSPI_FIFO_CTRL);

	writel(NOVATEK_FSPI_MODULE_RAW, fspi->base + NOVATEK_FSPI_MODULE);
	writel(NOVATEK_FSPI_CONFIG_IDLE, fspi->base + NOVATEK_FSPI_CONFIG);

	writel(NOVATEK_FSPI_STATUS_CLEAR_ALL, fspi->base + NOVATEK_FSPI_STATUS);

	writel(FIELD_PREP(NOVATEK_FSPI_TIMING_TSLCH_MASK, 0x1) |
	       FIELD_PREP(NOVATEK_FSPI_TIMING_TSHCH_MASK, 0x5) |
	       FIELD_PREP(NOVATEK_FSPI_TIMING_TSHSL_MASK, 0x5f),
	       fspi->base + NOVATEK_FSPI_TIMING);

	writel(FIELD_PREP(NOVATEK_FSPI_PHY_INPUT_DELAY_MASK, 0x5),
	       fspi->base + NOVATEK_FSPI_PHY_DELAY);
	writel(FIELD_PREP(NOVATEK_FSPI_PHY_SAMPLE_INV_MASK,
			  NOVATEK_FSPI_PHY_SAMPLE_INVERT) |
	       FIELD_PREP(NOVATEK_FSPI_PHY_PAD_CLK_MASK,
			  NOVATEK_FSPI_PHY_PAD_CLK_SELECTED) |
	       FIELD_PREP(NOVATEK_FSPI_PHY_RESET_MASK, NOVATEK_FSPI_PHY_RESET),
	       fspi->base + NOVATEK_FSPI_PHY);
	ret = readl_poll_timeout(fspi->base + NOVATEK_FSPI_PHY, value,
				 !(value & NOVATEK_FSPI_PHY_RESET_MASK), 1,
				 NOVATEK_FSPI_TIMEOUT_US);
	if (!ret)
		fspi->fault = false;

	return ret;
}

static int novatek_fspi_wait_complete(struct novatek_fspi *fspi)
{
	u32 status;
	int ret;

	ret = novatek_fspi_poll_timeout(
		fspi->base + NOVATEK_FSPI_STATUS, status,
		status & (NOVATEK_FSPI_STATUS_COMPLETE_MASK |
			  NOVATEK_FSPI_STATUS_ERROR_MASK));
	writel(status & NOVATEK_FSPI_STATUS_ALL_MASK,
	       fspi->base + NOVATEK_FSPI_STATUS);
	if (ret || (status & NOVATEK_FSPI_STATUS_TIMEOUT_MASK))
		return -ETIMEDOUT;
	if (status & NOVATEK_FSPI_STATUS_ERROR_MASK)
		return -EIO;

	return 0;
}

static bool novatek_fspi_supports_op(struct spi_mem *mem,
				     const struct spi_mem_op *op)
{
	if (!spi_mem_default_supports_op(mem, op))
		return false;
	if (op->cmd.buswidth != 1 || op->addr.nbytes > 4 ||
	    (op->addr.nbytes && op->addr.buswidth != 1))
		return false;

	return true;
}

static int novatek_fspi_adjust_op_size(struct spi_mem *mem,
				       struct spi_mem_op *op)
{
	op->data.nbytes = min_t(uint, op->data.nbytes,
				NOVATEK_FSPI_MAX_TRANSFER);

	return 0;
}

static int novatek_fspi_data(struct novatek_fspi *fspi,
			     const struct spi_mem_op *op, ulong rate)
{
	bool read = op->data.dir == SPI_MEM_DATA_IN;
	u32 fifo_ctrl;
	uint remaining = op->data.nbytes;
	u8 *rx = op->data.buf.in;
	const u8 *tx = op->data.buf.out;
	u32 value;
	int ret;

	fifo_ctrl = FIELD_PREP(NOVATEK_FSPI_FIFO_EN_MASK,
			       NOVATEK_FSPI_FIFO_DISABLE) |
		    FIELD_PREP(NOVATEK_FSPI_FIFO_MODE_MASK,
			       NOVATEK_FSPI_FIFO_PIO) |
		    FIELD_PREP(NOVATEK_FSPI_FIFO_DIR_MASK,
			       read ? NOVATEK_FSPI_FIFO_READ :
				      NOVATEK_FSPI_FIFO_WRITE);

	writel((NOVATEK_FSPI_CONFIG_IDLE & ~NOVATEK_FSPI_CONFIG_CS_LEVEL_MASK) |
	       FIELD_PREP(NOVATEK_FSPI_CONFIG_CS_LEVEL_MASK,
			  NOVATEK_FSPI_CONFIG_CS_LOW) |
	       FIELD_PREP(NOVATEK_FSPI_CONFIG_WIDTH_MASK,
			  ilog2(op->data.buswidth)),
	       fspi->base + NOVATEK_FSPI_CONFIG);

	writel(FIELD_PREP(NOVATEK_FSPI_DATA_LENGTH_MASK, remaining),
	       fspi->base + NOVATEK_FSPI_DATA_LENGTH);

	writel(fifo_ctrl, fspi->base + NOVATEK_FSPI_FIFO_CTRL);
	ret = novatek_fspi_poll_timeout(
		fspi->base + NOVATEK_FSPI_FIFO_CTRL, value,
		!(value & NOVATEK_FSPI_FIFO_EN_MASK));
	if (ret)
		return ret;

	writel(fifo_ctrl | FIELD_PREP(NOVATEK_FSPI_FIFO_EN_MASK,
				      NOVATEK_FSPI_FIFO_ENABLE),
	       fspi->base + NOVATEK_FSPI_FIFO_CTRL);

	writel(FIELD_PREP(NOVATEK_FSPI_CTRL_START_MASK,
			  NOVATEK_FSPI_CTRL_START) |
	       FIELD_PREP(NOVATEK_FSPI_CTRL_OP_MASK,
			  read ? NOVATEK_FSPI_OP_READ : NOVATEK_FSPI_OP_WRITE),
	       fspi->base + NOVATEK_FSPI_CTRL);

	while (remaining) {
		uint count = min(remaining, NOVATEK_FSPI_FIFO_BANK_SIZE);
		uint offset;
		u32 mask = NOVATEK_FSPI_FIFO_EMPTY_MASK;
		u32 ready;

		ready = FIELD_PREP(NOVATEK_FSPI_FIFO_EMPTY_MASK,
				   NOVATEK_FSPI_FIFO_EMPTY);
		if (read) {
			mask = count == NOVATEK_FSPI_FIFO_BANK_SIZE ?
				NOVATEK_FSPI_FIFO_FULL_MASK :
				NOVATEK_FSPI_FIFO_COUNT_MASK;
			ready = count == NOVATEK_FSPI_FIFO_BANK_SIZE ?
				FIELD_PREP(NOVATEK_FSPI_FIFO_FULL_MASK,
					   NOVATEK_FSPI_FIFO_FULL) :
				FIELD_PREP(NOVATEK_FSPI_FIFO_COUNT_MASK,
					   DIV_ROUND_UP(count, 4));
		}
		ret = novatek_fspi_poll_timeout(
			fspi->base + NOVATEK_FSPI_FIFO_STATUS, value,
			(value & mask) == ready);
		if (ret)
			return ret;

		for (offset = 0; offset < count; offset += 4) {
			uint bytes = min(count - offset, 4U);
			uint i;

			if (read) {
				value = readl(fspi->base + NOVATEK_FSPI_DATA);
				for (i = 0; i < bytes; i++)
					rx[i] = value >> (8 * i);
				rx += bytes;
			} else {
				value = 0;
				for (i = 0; i < bytes; i++)
					value |= (u32)tx[i] << (8 * i);
				writel(value, fspi->base + NOVATEK_FSPI_DATA);
				tx += bytes;
			}
		}

		remaining -= count;
		if (remaining)
			cond_resched();
	}

	/*
	 * Padded TX words do not complete automatically. Wait the full
	 * single-lane wire time before stopping the FIFO, including in
	 * dual/quad mode.
	 */
	if (!read && (op->data.nbytes & 3)) {
		uint delay;

		delay = DIV_ROUND_UP_ULL(((u64)op->data.nbytes * 8 + 10) *
					 USEC_PER_SEC, rate);
		usleep_range(delay, delay + 10);

		writel(fifo_ctrl, fspi->base + NOVATEK_FSPI_FIFO_CTRL);
	}

	ret = novatek_fspi_wait_complete(fspi);
	if (ret)
		return ret;

	writel(NOVATEK_FSPI_FIFO_IDLE, fspi->base + NOVATEK_FSPI_FIFO_CTRL);
	return novatek_fspi_poll_timeout(
		fspi->base + NOVATEK_FSPI_FIFO_CTRL, value,
		!(value & NOVATEK_FSPI_FIFO_EN_MASK));
}

static int novatek_fspi_exec_op(struct spi_mem *mem,
				const struct spi_mem_op *op)
{
	struct novatek_fspi *fspi;
	uint dummy_cycles = 0;
	ulong rate;
	u32 module = NOVATEK_FSPI_MODULE_RAW;
	u32 control = NOVATEK_FSPI_OP_COMMAND;
	int ret;

	fspi = spi_controller_get_devdata(mem->spi->controller);

	if (op->data.nbytes > NOVATEK_FSPI_MAX_TRANSFER)
		return -EMSGSIZE;

	if (fspi->fault) {
		ret = novatek_fspi_hw_init(fspi);
		if (ret)
			return ret;
	}

	ret = clk_set_rate(fspi->clk, op->max_freq);
	if (ret)
		return ret;

	rate = clk_get_rate(fspi->clk);
	if (!rate || rate > op->max_freq)
		return -EINVAL;

	if (op->addr.nbytes) {
		module |= FIELD_PREP(NOVATEK_FSPI_MODULE_ROW_BYTES_MASK,
				     op->addr.nbytes - 1);
		control = NOVATEK_FSPI_OP_COMMAND_ADDRESS;
	}
	writel(module, fspi->base + NOVATEK_FSPI_MODULE);
	writel(op->addr.val, fspi->base + NOVATEK_FSPI_ROW_ADDRESS);
	writel(op->cmd.opcode, fspi->base + NOVATEK_FSPI_COMMAND);

	writel(NOVATEK_FSPI_STATUS_CLEAR_ALL, fspi->base + NOVATEK_FSPI_STATUS);

	writel((NOVATEK_FSPI_CONFIG_IDLE & ~NOVATEK_FSPI_CONFIG_CS_LEVEL_MASK) |
	       FIELD_PREP(NOVATEK_FSPI_CONFIG_CS_LEVEL_MASK,
			  NOVATEK_FSPI_CONFIG_CS_LOW),
	       fspi->base + NOVATEK_FSPI_CONFIG);
	writel(FIELD_PREP(NOVATEK_FSPI_CTRL_OP_MASK, control) |
	       FIELD_PREP(NOVATEK_FSPI_CTRL_START_MASK,
			  NOVATEK_FSPI_CTRL_START),
	       fspi->base + NOVATEK_FSPI_CTRL);
	ret = novatek_fspi_wait_complete(fspi);
	if (ret)
		goto out;

	if (op->dummy.nbytes)
		dummy_cycles = op->dummy.nbytes * 8 / op->dummy.buswidth;
	while (dummy_cycles) {
		uint cycles = min(dummy_cycles, 8U);

		writel(FIELD_PREP(NOVATEK_FSPI_DUMMY_CLOCK_MASK, cycles - 1),
		       fspi->base + NOVATEK_FSPI_DUMMY);
		writel(FIELD_PREP(NOVATEK_FSPI_CTRL_OP_MASK,
				  NOVATEK_FSPI_OP_DUMMY) |
		       FIELD_PREP(NOVATEK_FSPI_CTRL_START_MASK,
				  NOVATEK_FSPI_CTRL_START),
		       fspi->base + NOVATEK_FSPI_CTRL);
		ret = novatek_fspi_wait_complete(fspi);
		if (ret)
			goto out;

		dummy_cycles -= cycles;
	}

	if (op->data.nbytes)
		ret = novatek_fspi_data(fspi, op, rate);

out:
	writel(NOVATEK_FSPI_CONFIG_IDLE, fspi->base + NOVATEK_FSPI_CONFIG);
	readl(fspi->base + NOVATEK_FSPI_CONFIG);

	if (ret) {
		int recovery_ret = novatek_fspi_hw_init(fspi);

		if (recovery_ret)
			dev_err_ratelimited(fspi->dev,
					    "controller recovery failed: %d\n",
					    recovery_ret);
	}

	return ret;
}

static const struct spi_controller_mem_ops novatek_fspi_mem_ops = {
	.supports_op = novatek_fspi_supports_op,
	.adjust_op_size = novatek_fspi_adjust_op_size,
	.exec_op = novatek_fspi_exec_op,
};

static const struct spi_controller_mem_caps novatek_fspi_mem_caps = {
	.per_op_freq = true,
};

static int novatek_fspi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct spi_controller *host;
	struct novatek_fspi *fspi;
	int ret;

	host = devm_spi_alloc_host(dev, sizeof(*fspi));
	if (!host)
		return -ENOMEM;
	fspi = spi_controller_get_devdata(host);
	fspi->dev = dev;

	fspi->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(fspi->base))
		return PTR_ERR(fspi->base);

	fspi->sram = syscon_regmap_lookup_by_phandle(dev->of_node, "regmap");
	if (IS_ERR(fspi->sram))
		return dev_err_probe(dev, PTR_ERR(fspi->sram),
				     "failed to get SRAM controller\n");
	ret = novatek_fspi_enable_sram(fspi);
	if (ret)
		return dev_err_probe(dev, ret, "failed to enable SRAM\n");
	ret = devm_add_action_or_reset(dev, novatek_fspi_disable_sram_action,
				       fspi);
	if (ret)
		return ret;

	fspi->clk = devm_clk_get_enabled(dev, NULL);
	if (IS_ERR(fspi->clk))
		return dev_err_probe(dev, PTR_ERR(fspi->clk),
				     "failed to enable clock\n");
	ret = clk_set_rate(fspi->clk, NOVATEK_FSPI_MAX_SPEED_HZ);
	if (ret)
		return dev_err_probe(dev, ret, "failed to set clock rate\n");

	fspi->reset = devm_reset_control_get_exclusive_deasserted(dev, NULL);
	if (IS_ERR(fspi->reset))
		return dev_err_probe(dev, PTR_ERR(fspi->reset),
				     "failed to get reset\n");

	ret = novatek_fspi_hw_init(fspi);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to initialize controller\n");

	host->mode_bits = SPI_RX_DUAL | SPI_RX_QUAD | SPI_TX_DUAL | SPI_TX_QUAD;
	host->bits_per_word_mask = SPI_BPW_MASK(8);

	host->num_chipselect = 1;

	host->min_speed_hz = NOVATEK_FSPI_MIN_SPEED_HZ;
	host->max_speed_hz = NOVATEK_FSPI_MAX_SPEED_HZ;

	host->mem_ops = &novatek_fspi_mem_ops;
	host->mem_caps = &novatek_fspi_mem_caps;

	platform_set_drvdata(pdev, host);

	return devm_spi_register_controller(dev, host);
}

#ifdef CONFIG_PM_SLEEP
static int novatek_fspi_suspend(struct device *dev)
{
	struct spi_controller *host = dev_get_drvdata(dev);
	struct novatek_fspi *fspi = spi_controller_get_devdata(host);
	int ret;

	ret = spi_controller_suspend(host);
	if (ret)
		return ret;

	clk_disable_unprepare(fspi->clk);

	ret = novatek_fspi_disable_sram(fspi);
	if (ret) {
		return ret;
	}

	return 0;
}

static int novatek_fspi_resume(struct device *dev)
{
	struct spi_controller *host = dev_get_drvdata(dev);
	struct novatek_fspi *fspi = spi_controller_get_devdata(host);
	int ret;

	ret = novatek_fspi_enable_sram(fspi);
	if (ret)
		return ret;

	ret = clk_prepare_enable(fspi->clk);
	if (ret)
		return ret;

	ret = novatek_fspi_hw_init(fspi);
	if (ret)
		return ret;

	return spi_controller_resume(host);
}
#endif

static const struct dev_pm_ops novatek_fspi_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(novatek_fspi_suspend, novatek_fspi_resume)
};

static const struct of_device_id novatek_fspi_of_match[] = {
	{ .compatible = "novatek,na51089-fspi" },
	{ }
};
MODULE_DEVICE_TABLE(of, novatek_fspi_of_match);

static struct platform_driver novatek_fspi_driver = {
	.probe = novatek_fspi_probe,
	.driver = {
		.name = "novatek-fspi",
		.of_match_table = novatek_fspi_of_match,
		.pm = pm_sleep_ptr(&novatek_fspi_pm_ops),
	},
};
module_platform_driver(novatek_fspi_driver);

MODULE_DESCRIPTION("Novatek NA51089 flash SPI controller driver");
MODULE_LICENSE("GPL");
