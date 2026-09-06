// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2026 Yu-Tung Chang <mtwget@gmail.com>
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/jiffies.h>
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/mtd/spinand.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/property.h>
#include <linux/reset.h>
#include <linux/sched.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/spi/spi-mem.h>
#include <linux/units.h>

#define FSPI_MODULE_REG					0x00
#define FSPI_PHY_REG					0x08
#define FSPI_PHY_DELAY_REG				0x0c
#define FSPI_CONFIG_REG					0x14
#define FSPI_CTRL_REG					0x20
#define FSPI_TIMING_REG					0x24
#define FSPI_SRAM_ACCESS_REG				0x28
#define FSPI_COMMAND_REG				0x34
#define FSPI_COLUMN_ADDRESS_REG				0x38
#define FSPI_ROW_ADDRESS_REG				0x3c
#define FSPI_INT_ENABLE_REG				0x44
#define FSPI_STATUS_REG					0x48
#define FSPI_ECC_FREE_REG				0x5c
#define FSPI_RS_STATUS_REG				0x70
#define FSPI_DUMMY_REG					0xc0
#define FSPI_DMA_ADDRESS_HIGH_REG			0xc4
#define FSPI_DATA_REG					0x100
#define FSPI_DATA_LENGTH_REG				0x104
#define FSPI_FIFO_STATUS_REG				0x108
#define FSPI_FIFO_CTRL_REG				0x10c
#define FSPI_DMA_ADDRESS_REG				0x110
#define FSPI_SPARE_REG					0x200
#define FSPI_ERROR_REG					0x400

/* FSPI_MODULE bit fields */
#define FSPI_MODULE_COL_BYTES				GENMASK(1, 0)
#define FSPI_MODULE_ROW_BYTES				GENMASK(3, 2)
#define FSPI_MODULE_PAGE_SIZE				GENMASK(7, 4)
#define FSPI_MODULE_PRI_ECC				GENMASK(9, 8)
#define FSPI_MODULE_PRI_ECC_RS				0x2
#define FSPI_MODULE_TYPE				BIT(19)
#define FSPI_MODULE_FLASH				BIT(20)
/*
 * Use NOR mode for split command/address/data transfers, including SPI NAND,
 * without the controller's NAND-specific command sequences.
 */
#define FSPI_MODULE_RAW			\
	(FIELD_PREP_CONST(FSPI_MODULE_TYPE, 1) | \
	 FIELD_PREP_CONST(FSPI_MODULE_FLASH, 1))

/* FSPI_PHY bit fields */
#define FSPI_PHY_RESET					BIT(0)
#define FSPI_PHY_SAMPLE_INV				BIT(2)
#define FSPI_PHY_PAD_CLK				BIT(20)
#define FSPI_PHY_DEFAULT \
	(FIELD_PREP_CONST(FSPI_PHY_RESET, 1) | \
	 FIELD_PREP_CONST(FSPI_PHY_SAMPLE_INV, 1) | \
	 FIELD_PREP_CONST(FSPI_PHY_PAD_CLK, 1))

/* FSPI_PHY_DELAY bit fields */
#define FSPI_PHY_DELAY_INPUT_DELAY			GENMASK(29, 24)
#define FSPI_PHY_DELAY_DEFAULT \
	(FIELD_PREP_CONST(FSPI_PHY_DELAY_INPUT_DELAY, 0x05))

/* FSPI_CONFIG bit fields */
#define FSPI_CONFIG_CS_MODE				BIT(1)
#define FSPI_CONFIG_CS_LEVEL				BIT(4)
#define FSPI_CONFIG_WIDTH				GENMASK(10, 9)
#define FSPI_CONFIG_WIDTH_SINGLE			0x0
#define FSPI_CONFIG_WIDTH_DUAL				0x1
#define FSPI_CONFIG_WIDTH_QUAD				0x2
#define FSPI_CONFIG_IO_ORDER				BIT(11)
#define FSPI_CONFIG_PULL				BIT(12)
#define FSPI_CONFIG_IDLE		\
	(FIELD_PREP_CONST(FSPI_CONFIG_CS_MODE, 1) | \
	 FIELD_PREP_CONST(FSPI_CONFIG_CS_LEVEL, 1) | \
	 FIELD_PREP_CONST(FSPI_CONFIG_WIDTH, FSPI_CONFIG_WIDTH_SINGLE) | \
	 FIELD_PREP_CONST(FSPI_CONFIG_IO_ORDER, 1) | \
	 FIELD_PREP_CONST(FSPI_CONFIG_PULL, 1))

/* FSPI_CTRL bit fields */
#define FSPI_CTRL_OP					GENMASK(5, 0)
#define FSPI_CTRL_OP_READ_PAGE				0x00
#define FSPI_CTRL_OP_COMMAND				0x07
#define FSPI_CTRL_OP_COMMAND_ADDRESS			0x0c
#define FSPI_CTRL_OP_READ				0x18
#define FSPI_CTRL_OP_WRITE				0x19
#define FSPI_CTRL_OP_WRITE_CACHE			0x1c
#define FSPI_CTRL_OP_DUMMY				0x1e
#define FSPI_CTRL_START					BIT(12)
#define FSPI_CTRL_RESET					BIT(15)
#define FSPI_CTRL_RESET_CONFIG \
	(FIELD_PREP_CONST(FSPI_CTRL_OP, FSPI_CTRL_OP_READ_PAGE) | \
	 FIELD_PREP_CONST(FSPI_CTRL_START, 0) | \
	 FIELD_PREP_CONST(FSPI_CTRL_RESET, 1))

/* FSPI_TIMING bit fields */
#define FSPI_TIMING_TSLCH				GENMASK(3, 0)
#define FSPI_TIMING_TSHCH				GENMASK(7, 4)
#define FSPI_TIMING_TSHSL				GENMASK(15, 8)
#define FSPI_TIMING_DEFAULT \
	(FIELD_PREP_CONST(FSPI_TIMING_TSLCH, 0x1) | \
	 FIELD_PREP_CONST(FSPI_TIMING_TSHCH, 0x5) | \
	 FIELD_PREP_CONST(FSPI_TIMING_TSHSL, 0x5f))

/* FSPI_SRAM_ACCESS bit fields */
#define FSPI_SRAM_ACCESS_SPARE				BIT(0)
#define FSPI_SRAM_ACCESS_ERROR				BIT(1)
#define FSPI_SRAM_ACCESS_CONTROLLER \
	(FIELD_PREP_CONST(FSPI_SRAM_ACCESS_SPARE, 0) |		\
	 FIELD_PREP_CONST(FSPI_SRAM_ACCESS_ERROR, 0))
#define FSPI_SRAM_ACCESS_SPARE_ONLY \
	(FIELD_PREP_CONST(FSPI_SRAM_ACCESS_SPARE, 1) | \
	 FIELD_PREP_CONST(FSPI_SRAM_ACCESS_ERROR, 0))

/* FSPI_INT_ENABLE bit fields */
#define FSPI_INT_ENABLE_COMPLETE			BIT(12)
#define FSPI_INT_ENABLE_PRI_ECC				BIT(13)
#define FSPI_INT_ENABLE_STATUS_FAIL			BIT(14)
#define FSPI_INT_ENABLE_TIMEOUT				BIT(15)
#define FSPI_INT_ENABLE_SEC_ECC				BIT(16)
#define FSPI_INT_ENABLE_PROTECT1			BIT(17)
#define FSPI_INT_ENABLE_PROTECT2			BIT(18)
#define FSPI_INT_ENABLE_DISABLE_ALL		\
	(FIELD_PREP_CONST(FSPI_INT_ENABLE_COMPLETE, 0) | \
	 FIELD_PREP_CONST(FSPI_INT_ENABLE_PRI_ECC, 0) | \
	 FIELD_PREP_CONST(FSPI_INT_ENABLE_STATUS_FAIL, 0) | \
	 FIELD_PREP_CONST(FSPI_INT_ENABLE_TIMEOUT, 0) | \
	 FIELD_PREP_CONST(FSPI_INT_ENABLE_SEC_ECC, 0) | \
	 FIELD_PREP_CONST(FSPI_INT_ENABLE_PROTECT1, 0) | \
	 FIELD_PREP_CONST(FSPI_INT_ENABLE_PROTECT2, 0))
#define FSPI_INT_ENABLE_DMA_ENABLE		\
	(FIELD_PREP_CONST(FSPI_INT_ENABLE_COMPLETE, 1) | \
	 FIELD_PREP_CONST(FSPI_INT_ENABLE_STATUS_FAIL, 1) | \
	 FIELD_PREP_CONST(FSPI_INT_ENABLE_TIMEOUT, 1) | \
	 FIELD_PREP_CONST(FSPI_INT_ENABLE_PROTECT1, 1) | \
	 FIELD_PREP_CONST(FSPI_INT_ENABLE_PROTECT2, 1))

/* FSPI_STATUS bit fields */
#define FSPI_STATUS_COMPLETE				BIT(12)
#define FSPI_STATUS_PRI_ECC				BIT(13)
#define FSPI_STATUS_FAIL				BIT(14)
#define FSPI_STATUS_TIMEOUT				BIT(15)
#define FSPI_STATUS_SEC_ECC				BIT(16)
#define FSPI_STATUS_PROTECT1				BIT(17)
#define FSPI_STATUS_PROTECT2				BIT(18)
#define FSPI_STATUS_ALL		\
	(FSPI_STATUS_COMPLETE | \
	 FSPI_STATUS_PRI_ECC | \
	 FSPI_STATUS_FAIL | \
	 FSPI_STATUS_TIMEOUT | \
	 FSPI_STATUS_SEC_ECC | \
	 FSPI_STATUS_PROTECT1 | \
	 FSPI_STATUS_PROTECT2)
#define FSPI_STATUS_CLEAR_ALL		\
	(FIELD_PREP_CONST(FSPI_STATUS_COMPLETE, 1) | \
	 FIELD_PREP_CONST(FSPI_STATUS_PRI_ECC, 1) | \
	 FIELD_PREP_CONST(FSPI_STATUS_FAIL, 1) | \
	 FIELD_PREP_CONST(FSPI_STATUS_TIMEOUT, 1) | \
	 FIELD_PREP_CONST(FSPI_STATUS_SEC_ECC, 1) | \
	 FIELD_PREP_CONST(FSPI_STATUS_PROTECT1, 1) | \
	 FIELD_PREP_CONST(FSPI_STATUS_PROTECT2, 1))
#define FSPI_STATUS_ERROR		\
	(FSPI_STATUS_FAIL | \
	 FSPI_STATUS_TIMEOUT | \
	 FSPI_STATUS_PROTECT1 | \
	 FSPI_STATUS_PROTECT2)

/* FSPI_ECC_FREE bit fields */
#define FSPI_ECC_FREE_VALUE				BIT(0)
#define FSPI_ECC_FREE_DISABLED \
	(FIELD_PREP_CONST(FSPI_ECC_FREE_VALUE, 0))

/* FSPI_RS_STATUS bit fields */
#define FSPI_RS_STATUS_COUNT				GENMASK(2, 0)

/* FSPI_DUMMY bit fields */
#define FSPI_DUMMY_CLOCK				GENMASK(2, 0)

/* FSPI_DMA_ADDRESS_HIGH bit fields */
#define FSPI_DMA_ADDRESS_HIGH_VALUE			GENMASK(3, 0)

/* FSPI_DATA_LENGTH bit fields */
#define FSPI_DATA_LENGTH_COUNT				GENMASK(25, 0)

/* FSPI_FIFO_STATUS bit fields */
#define FSPI_FIFO_STATUS_COUNT				GENMASK(4, 0)
#define FSPI_FIFO_STATUS_EMPTY				BIT(8)
#define FSPI_FIFO_STATUS_FULL				BIT(9)

/* FSPI_FIFO_CTRL bit fields */
#define FSPI_FIFO_CTRL_EN				BIT(0)
#define FSPI_FIFO_CTRL_MODE				BIT(1)
#define FSPI_FIFO_CTRL_DIR				BIT(2)
#define FSPI_FIFO_CTRL_IDLE			\
	(FIELD_PREP_CONST(FSPI_FIFO_CTRL_EN, 0) | \
	 FIELD_PREP_CONST(FSPI_FIFO_CTRL_MODE, 0) | \
	 FIELD_PREP_CONST(FSPI_FIFO_CTRL_DIR, 0))

/* FSPI_ERROR bit fields */
#define FSPI_ERROR_ADDRESS				GENMASK(15, 0)
#define FSPI_ERROR_VALUE				GENMASK(23, 16)
#define FSPI_ERROR_NA51089_RS_PARITY_VALUE		GENMASK(25, 16)

#define FSPI_RS_STATUS_BITS				4U
#define FSPI_RS_ERROR_WORDS_PER_STEP			4U

#define FSPI_MAX_TRANSFER				SZ_64K

#define FSPI_DMA_MIN_BYTES				64U
#define FSPI_DMA_ALIGN					4U

#define FSPI_NAND_UPDATE_CACHE				0x84
#define FSPI_NAND_UPDATE_CACHE_QUAD			0x34

#define FSPI_POLL_FAST_US				100
#define FSPI_POLL_SLEEP_US				20
#define FSPI_TIMEOUT_US					100000
#define FSPI_RESET_DELAY_US				10
#define FSPI_SRAM_DELAY_NS				200

#define FSPI_MIN_SPEED_HZ				7500000
#define FSPI_MAX_SPEED_HZ				48000000

struct novatek_fspi_ecc_caps {
	uint step_size;
	uint strength;
	uint spare_size;
	uint bytes;
	uint max_steps;
	uint symbol_bits;
	uint parity_symbols;
};

struct novatek_fspi_soc_data {
	uint fifo_bank_size;
	const struct novatek_fspi_ecc_caps *ecc;
};

struct novatek_fspi_ecc {
	const struct novatek_fspi_ecc_caps *caps;
	struct spi_mem *mem;
	struct nand_page_io_req req;
	u8 *buf;
	u32 module;
	u32 status;
	uint steps;
	uint page_size;
	uint oob_size;
	bool result_valid;
	u32 errors[];
};

struct novatek_fspi {
	struct device *dev;
	const struct novatek_fspi_soc_data *data;
	void __iomem *base;
	struct clk *clk;
	struct reset_control *reset;
	struct completion dma_complete;
	struct nand_ecc_engine ecc_engine;
	struct novatek_fspi_ecc *ecc;
	void *dma_buf;
	dma_addr_t dma_addr;
	enum dma_data_direction dma_dir;
	uint dma_len;
	u32 dma_status;
	int irq;
	bool dma_enabled;
	bool dma_active;
	bool reset_asserted;
	bool fault;
};

#define novatek_fspi_poll_timeout(addr, val, cond) \
({ \
	const void __iomem *__fspi_addr = (addr); \
	ktime_t __fspi_deadline; \
	s64 __fspi_remaining; \
	int __fspi_ret; \
\
	__fspi_deadline = ktime_add_us(ktime_get(), FSPI_TIMEOUT_US); \
	__fspi_ret = readl_poll_timeout(__fspi_addr, val, cond, 0, FSPI_POLL_FAST_US); \
	if (__fspi_ret) { \
		__fspi_remaining = ktime_us_delta(__fspi_deadline, ktime_get()); \
		if (__fspi_remaining > 0) \
			__fspi_ret = readl_poll_timeout(__fspi_addr, \
							val, cond, \
							FSPI_POLL_SLEEP_US, __fspi_remaining); \
	} \
\
	__fspi_ret; \
})

static void novatek_fspi_mask_irq(struct novatek_fspi *fspi)
{
	if (fspi->reset_asserted)
		return;

	writel(FSPI_INT_ENABLE_DISABLE_ALL, fspi->base + FSPI_INT_ENABLE_REG);
	readl(fspi->base + FSPI_INT_ENABLE_REG);
}

static int novatek_fspi_stop_dma(struct novatek_fspi *fspi)
{
	u32 value;
	int ret;

	if (!fspi->dma_active)
		return 0;

	novatek_fspi_mask_irq(fspi);
	synchronize_irq(fspi->irq);

	value = readl(fspi->base + FSPI_FIFO_CTRL_REG);
	value &= ~FSPI_FIFO_CTRL_EN;
	writel(value, fspi->base + FSPI_FIFO_CTRL_REG);
	ret = readl_poll_timeout(fspi->base + FSPI_FIFO_CTRL_REG,
				 value, !(value & FSPI_FIFO_CTRL_EN),
				 1, FSPI_TIMEOUT_US);
	if (ret) {
		fspi->fault = true;
		ret = reset_control_assert(fspi->reset);
		if (ret)
			return ret;
		fspi->reset_asserted = true;
		reset_control_status(fspi->reset);
		udelay(FSPI_RESET_DELAY_US);
	}
	fspi->dma_active = false;

	return 0;
}

static void novatek_fspi_release_dma(struct novatek_fspi *fspi, void *rx)
{
	if (!fspi->dma_buf || fspi->dma_active)
		return;

	synchronize_irq(fspi->irq);
	dma_unmap_single(fspi->dev, fspi->dma_addr, fspi->dma_len, fspi->dma_dir);
	if (rx)
		memcpy(rx, fspi->dma_buf, fspi->dma_len);

	kfree(fspi->dma_buf);
	fspi->dma_buf = NULL;
}

static int novatek_fspi_hw_init(struct novatek_fspi *fspi)
{
	u32 value;
	int ret;

	fspi->fault = true;

	if (fspi->reset_asserted) {
		ret = reset_control_deassert(fspi->reset);
		if (ret)
			return ret;
		fspi->reset_asserted = false;
		reset_control_status(fspi->reset);
		udelay(FSPI_RESET_DELAY_US);
	}

	writel(FSPI_INT_ENABLE_DISABLE_ALL, fspi->base + FSPI_INT_ENABLE_REG);

	writel(FSPI_FIFO_CTRL_IDLE, fspi->base + FSPI_FIFO_CTRL_REG);
	ret = readl_poll_timeout(fspi->base + FSPI_FIFO_CTRL_REG,
				 value, !(value & FSPI_FIFO_CTRL_EN),
				 1, FSPI_TIMEOUT_US);
	if (!ret) {
		writel(FSPI_CTRL_RESET_CONFIG, fspi->base + FSPI_CTRL_REG);
		ret = readl_poll_timeout(fspi->base + FSPI_CTRL_REG,
					 value, !(value & FSPI_CTRL_RESET),
					 1, FSPI_TIMEOUT_US);
	}
	if (ret)
		ret = reset_control_reset(fspi->reset);
	if (ret)
		return ret;

	writel(FSPI_INT_ENABLE_DISABLE_ALL, fspi->base + FSPI_INT_ENABLE_REG);

	writel(FSPI_FIFO_CTRL_IDLE, fspi->base + FSPI_FIFO_CTRL_REG);

	writel(FSPI_MODULE_RAW, fspi->base + FSPI_MODULE_REG);

	writel(FSPI_CONFIG_IDLE, fspi->base + FSPI_CONFIG_REG);

	writel(FSPI_STATUS_CLEAR_ALL, fspi->base + FSPI_STATUS_REG);

	writel(FSPI_TIMING_DEFAULT, fspi->base + FSPI_TIMING_REG);

	writel(FSPI_PHY_DELAY_DEFAULT, fspi->base + FSPI_PHY_DELAY_REG);
	writel(FSPI_PHY_DEFAULT, fspi->base + FSPI_PHY_REG);
	ret = readl_poll_timeout(fspi->base + FSPI_PHY_REG,
				 value, !(value & FSPI_PHY_RESET),
				 1, FSPI_TIMEOUT_US);
	if (!ret)
		fspi->fault = false;

	return ret;
}

static int novatek_fspi_wait_complete(struct novatek_fspi *fspi)
{
	u32 status;
	int ret;

	ret = novatek_fspi_poll_timeout(fspi->base + FSPI_STATUS_REG,
					status, status & (FSPI_STATUS_COMPLETE |
							  FSPI_STATUS_ERROR));
	writel(status & FSPI_STATUS_ALL, fspi->base + FSPI_STATUS_REG);
	if (ret || (status & FSPI_STATUS_TIMEOUT))
		return -ETIMEDOUT;
	if (status & FSPI_STATUS_ERROR)
		return -EIO;

	return 0;
}

static int novatek_fspi_ooblayout_ecc(struct mtd_info *mtd, int section,
				      struct mtd_oob_region *region)
{
	struct nand_device *nand = mtd_to_nanddev(mtd);
	struct novatek_fspi_ecc *ecc = nand->ecc.ctx.priv;
	const struct novatek_fspi_ecc_caps *caps = ecc->caps;

	if (section >= ecc->steps)
		return -ERANGE;

	region->offset = section * caps->spare_size + caps->spare_size - caps->bytes;
	region->length = caps->bytes;

	return 0;
}

static int novatek_fspi_ooblayout_free(struct mtd_info *mtd, int section,
				       struct mtd_oob_region *region)
{
	struct nand_device *nand = mtd_to_nanddev(mtd);
	struct novatek_fspi_ecc *ecc = nand->ecc.ctx.priv;
	const struct novatek_fspi_ecc_caps *caps = ecc->caps;

	if (section >= ecc->steps)
		return -ERANGE;

	region->offset = section * caps->spare_size + 2;
	region->length = caps->spare_size - caps->bytes - 2;

	return 0;
}

static const struct mtd_ooblayout_ops novatek_fspi_ooblayout_ops = {
	.ecc = novatek_fspi_ooblayout_ecc,
	.free = novatek_fspi_ooblayout_free,
};

static int novatek_fspi_ecc_init_ctx(struct nand_device *nand)
{
	struct novatek_fspi *fspi = nand->ecc.engine->priv;
	const struct novatek_fspi_ecc_caps *caps = fspi->data->ecc;
	const struct nand_ecc_props *user = &nand->ecc.user_conf;
	struct mtd_info *mtd = nanddev_to_mtd(nand);
	struct novatek_fspi_ecc *ecc;
	uint steps;

	nand->ecc.ctx.conf.engine_type = NAND_ECC_ENGINE_TYPE_ON_HOST;
	if (!fspi->dma_enabled)
		return -EOPNOTSUPP;
	if ((user->step_size && user->step_size != caps->step_size) ||
	    user->strength > caps->strength)
		return -EOPNOTSUPP;
	if (mtd->writesize != SZ_2K && mtd->writesize != SZ_4K)
		return -EOPNOTSUPP;
	steps = mtd->writesize / caps->step_size;
	if (steps > caps->max_steps || mtd->oobsize < steps * caps->spare_size)
		return -EOPNOTSUPP;

	ecc = kzalloc(struct_size(ecc, errors, steps * caps->strength), GFP_KERNEL);
	if (!ecc)
		return -ENOMEM;

	ecc->buf = kmalloc(mtd->writesize + mtd->oobsize, GFP_KERNEL);
	if (!ecc->buf) {
		kfree(ecc);
		return -ENOMEM;
	}

	ecc->caps = caps;
	ecc->mem = nand_to_spinand(nand)->spimem;
	ecc->page_size = mtd->writesize;
	ecc->oob_size = mtd->oobsize;
	ecc->steps = steps;

	ecc->module = FIELD_PREP(FSPI_MODULE_PAGE_SIZE, ecc->steps - 1) |
		      FIELD_PREP(FSPI_MODULE_PRI_ECC, FSPI_MODULE_PRI_ECC_RS) |
		      FSPI_MODULE_TYPE;

	nand->ecc.ctx.conf.algo = NAND_ECC_ALGO_RS;
	nand->ecc.ctx.conf.placement = NAND_ECC_PLACEMENT_OOB;
	nand->ecc.ctx.conf.step_size = caps->step_size;
	nand->ecc.ctx.conf.strength = caps->strength;
	nand->ecc.ctx.nsteps = ecc->steps;
	nand->ecc.ctx.total = ecc->steps * caps->bytes;
	nand->ecc.ctx.priv = ecc;
	fspi->ecc = ecc;

	mtd_set_ooblayout(mtd, &novatek_fspi_ooblayout_ops);

	return 0;
}

static void novatek_fspi_ecc_cleanup_ctx(struct nand_device *nand)
{
	struct novatek_fspi *fspi = nand->ecc.engine->priv;
	struct novatek_fspi_ecc *ecc = nand->ecc.ctx.priv;

	fspi->ecc = NULL;
	nand->ecc.ctx.priv = NULL;
	kfree(ecc->buf);
	kfree(ecc);
}

static int novatek_fspi_ecc_prepare_io_req(struct nand_device *nand, struct nand_page_io_req *req)
{
	struct novatek_fspi_ecc *ecc = nand->ecc.ctx.priv;
	struct spinand_device *spinand = nand_to_spinand(nand);
	struct mtd_info *mtd = nanddev_to_mtd(nand);
	u8 *oob = ecc->buf + ecc->page_size;
	int ret;

	ecc->result_valid = false;
	memset(spinand->oobbuf, 0xff, ecc->oob_size);
	if (req->mode == MTD_OPS_RAW)
		return 0;
	if (req->type == NAND_PAGE_WRITE && !req->datalen)
		return -EOPNOTSUPP;

	ecc->req = *req;
	memset(ecc->buf, 0xff, ecc->page_size + ecc->oob_size);
	if (req->type == NAND_PAGE_WRITE) {
		if (req->datalen)
			memcpy(ecc->buf + req->dataoffs, req->databuf.out, req->datalen);

		if (req->ooblen) {
			if (req->mode == MTD_OPS_AUTO_OOB) {
				ret = mtd_ooblayout_set_databytes(mtd,
								  req->oobbuf.out, oob,
								  req->ooboffs, req->ooblen);
				if (ret)
					return ret;
			} else {
				memcpy(oob + req->ooboffs, req->oobbuf.out, req->ooblen);
			}
		}
	}

	req->dataoffs = 0;
	req->datalen = ecc->page_size;
	req->databuf.in = ecc->buf;
	req->ooboffs = 0;
	req->ooblen = ecc->oob_size;
	req->oobbuf.in = oob;
	req->mode = MTD_OPS_PLACE_OOB;

	return 0;
}

static int novatek_fspi_ecc_correct(const struct novatek_fspi_ecc_caps *caps,
				    u8 *data, u8 *oob, u32 error)
{
	uint metadata_size = caps->spare_size - caps->bytes;
	uint addr = FIELD_GET(FSPI_ERROR_ADDRESS, error);
	uint value = FIELD_GET(FSPI_ERROR_VALUE, error);
	uint bitflips;

	if (addr < caps->step_size) {
		data[addr] ^= value;
		return hweight8(value);
	}
	addr -= caps->step_size;
	if (addr < metadata_size) {
		oob[addr] ^= value;
		return hweight8(value);
	}
	addr -= metadata_size;
	if (addr >= caps->parity_symbols)
		return -EBADMSG;

	/* Parity addresses index symbols packed into the OOB. */
	value = FIELD_GET(FSPI_ERROR_NA51089_RS_PARITY_VALUE, error);
	bitflips = hweight16(value);
	addr *= caps->symbol_bits;
	value <<= addr % BITS_PER_BYTE;
	addr = metadata_size + addr / BITS_PER_BYTE;
	oob[addr] ^= value;
	oob[addr + 1] ^= value >> BITS_PER_BYTE;

	return bitflips;
}

static int novatek_fspi_ecc_finish_io_req(struct nand_device *nand, struct nand_page_io_req *req)
{
	struct novatek_fspi_ecc *ecc = nand->ecc.ctx.priv;
	const struct novatek_fspi_ecc_caps *caps = ecc->caps;
	struct mtd_info *mtd = nanddev_to_mtd(nand);
	uint max_bitflips = 0;
	bool failed = false;
	uint step;
	int ret = 0;

	if (req->mode == MTD_OPS_RAW)
		return 0;
	if (req->type == NAND_PAGE_WRITE)
		goto out_restore_request;
	if (!ecc->result_valid) {
		ret = -EIO;
		goto out_restore_request;
	}

	for (step = 0; step < ecc->steps; step++) {
		u8 *data = ecc->buf + step * caps->step_size;
		u8 *oob;
		uint count;
		uint bitflips = 0;
		uint i;

		oob = ecc->buf + ecc->page_size + step * caps->spare_size;
		count = FIELD_GET(FSPI_RS_STATUS_COUNT,
				  ecc->status >> (step * FSPI_RS_STATUS_BITS));
		if (count > caps->strength) {
			ret = nand_check_erased_ecc_chunk(data, caps->step_size,
							  oob, caps->spare_size,
							  NULL, 0, caps->strength);
			if (ret < 0) {
				mtd->ecc_stats.failed++;
				failed = true;
				continue;
			}
			bitflips = ret;
		} else {
			for (i = 0; i < count; i++) {
				ret = novatek_fspi_ecc_correct(caps,
							       data, oob,
							       ecc->errors[step *
									   caps->strength + i]);
				if (ret < 0) {
					mtd->ecc_stats.failed++;
					failed = true;
					break;
				}
				bitflips += ret;
			}
		}
		mtd->ecc_stats.corrected += bitflips;
		max_bitflips = max(max_bitflips, bitflips);
	}

	if (ecc->req.datalen)
		memcpy(ecc->req.databuf.in, ecc->buf + ecc->req.dataoffs, ecc->req.datalen);

	if (ecc->req.ooblen) {
		u8 *oob = ecc->buf + ecc->page_size;

		if (ecc->req.mode == MTD_OPS_AUTO_OOB) {
			ret = mtd_ooblayout_get_databytes(mtd,
							  ecc->req.oobbuf.in, oob,
							  ecc->req.ooboffs, ecc->req.ooblen);
			if (ret)
				goto out_restore_request;
		} else {
			memcpy(ecc->req.oobbuf.in, oob + ecc->req.ooboffs, ecc->req.ooblen);
		}
	}

	ret = failed ? -EBADMSG : max_bitflips;

out_restore_request:
	ecc->result_valid = false;
	*req = ecc->req;
	return ret;
}

static const struct nand_ecc_engine_ops novatek_fspi_ecc_ops = {
	.init_ctx = novatek_fspi_ecc_init_ctx,
	.cleanup_ctx = novatek_fspi_ecc_cleanup_ctx,
	.prepare_io_req = novatek_fspi_ecc_prepare_io_req,
	.finish_io_req = novatek_fspi_ecc_finish_io_req,
};

static bool novatek_fspi_supports_op(struct spi_mem *mem, const struct spi_mem_op *op)
{
	struct novatek_fspi *fspi;

	if (!spi_mem_default_supports_op(mem, op))
		return false;
	if (op->cmd.buswidth != 1 || op->addr.nbytes > 4 ||
	    (op->addr.nbytes && op->addr.buswidth != 1))
		return false;

	if (op->data.ecc) {
		fspi = spi_controller_get_devdata(mem->spi->controller);
		if (!fspi->dma_enabled || op->addr.nbytes != 2)
			return false;
		if (op->data.dir == SPI_MEM_DATA_IN)
			return op->dummy.nbytes && op->dummy.nbytes * 8 / op->dummy.buswidth == 8;
		return op->data.dir == SPI_MEM_DATA_OUT && !op->dummy.nbytes &&
		       (op->cmd.opcode == FSPI_NAND_UPDATE_CACHE ||
			op->cmd.opcode == FSPI_NAND_UPDATE_CACHE_QUAD);
	}

	return true;
}

static int novatek_fspi_adjust_op_size(struct spi_mem *mem, struct spi_mem_op *op)
{
	op->data.nbytes = min_t(uint, op->data.nbytes, FSPI_MAX_TRANSFER);

	return 0;
}

static int novatek_fspi_prepare_dma(struct novatek_fspi *fspi, const struct spi_mem_op *op)
{
	enum dma_data_direction dir;
	dma_addr_t addr;
	void *buf;
	int ret;

	if (!fspi->dma_enabled || op->data.nbytes < FSPI_DMA_MIN_BYTES ||
	    !IS_ALIGNED(op->data.nbytes, FSPI_DMA_ALIGN))
		return -EOPNOTSUPP;

	buf = kmalloc(op->data.nbytes, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;
	if (op->data.dir == SPI_MEM_DATA_OUT)
		memcpy(buf, op->data.buf.out, op->data.nbytes);

	dir = op->data.dir == SPI_MEM_DATA_IN ? DMA_FROM_DEVICE : DMA_TO_DEVICE;
	addr = dma_map_single(fspi->dev, buf, op->data.nbytes, dir);
	if (dma_mapping_error(fspi->dev, addr)) {
		ret = -EIO;
		goto out_free_buffer;
	}
	if (!IS_ALIGNED(addr, FSPI_DMA_ALIGN)) {
		ret = -EINVAL;
		goto out_unmap_dma;
	}
	fspi->dma_buf = buf;
	fspi->dma_addr = addr;
	fspi->dma_len = op->data.nbytes;
	fspi->dma_dir = dir;

	return 0;

out_unmap_dma:
	dma_unmap_single(fspi->dev, addr, op->data.nbytes, dir);
out_free_buffer:
	kfree(buf);
	return ret;
}

static irqreturn_t novatek_fspi_irq(int irq, void *data)
{
	struct novatek_fspi *fspi = data;
	u32 status;
	u32 enabled;

	if (fspi->reset_asserted)
		return IRQ_NONE;

	status = readl(fspi->base + FSPI_STATUS_REG);
	enabled = readl(fspi->base + FSPI_INT_ENABLE_REG);
	if (!(status & enabled))
		return IRQ_NONE;

	writel(FSPI_INT_ENABLE_DISABLE_ALL, fspi->base + FSPI_INT_ENABLE_REG);
	writel(status & FSPI_STATUS_ALL, fspi->base + FSPI_STATUS_REG);
	fspi->dma_status = status;
	complete(&fspi->dma_complete);

	return IRQ_HANDLED;
}

static int novatek_fspi_dma_data(struct novatek_fspi *fspi,
				 const struct spi_mem_op *op, u32 control)
{
	bool read = op->data.dir == SPI_MEM_DATA_IN;
	u32 enabled = FSPI_INT_ENABLE_DMA_ENABLE;
	u32 fifo_ctrl;
	ulong completed;
	int ret;

	fifo_ctrl = FSPI_FIFO_CTRL_MODE | (read ? 0 : FSPI_FIFO_CTRL_DIR);
	writel(fifo_ctrl, fspi->base + FSPI_FIFO_CTRL_REG);

	writel(lower_32_bits(fspi->dma_addr), fspi->base + FSPI_DMA_ADDRESS_REG);
	writel(FIELD_PREP(FSPI_DMA_ADDRESS_HIGH_VALUE, upper_32_bits(fspi->dma_addr)), fspi->base + FSPI_DMA_ADDRESS_HIGH_REG);

	reinit_completion(&fspi->dma_complete);
	fspi->dma_status = 0;
	writel(FSPI_STATUS_CLEAR_ALL, fspi->base + FSPI_STATUS_REG);

	if (op->data.ecc)
		enabled |= FSPI_INT_ENABLE_PRI_ECC;
	writel(enabled, fspi->base + FSPI_INT_ENABLE_REG);

	fspi->dma_active = true;
	writel(fifo_ctrl | FSPI_FIFO_CTRL_EN, fspi->base + FSPI_FIFO_CTRL_REG);
	writel(FSPI_CTRL_START | FIELD_PREP(FSPI_CTRL_OP, control), fspi->base + FSPI_CTRL_REG);

	completed = wait_for_completion_timeout(&fspi->dma_complete,
						usecs_to_jiffies(FSPI_TIMEOUT_US));
	novatek_fspi_mask_irq(fspi);
	synchronize_irq(fspi->irq);
	if (!completed || (fspi->dma_status & FSPI_STATUS_TIMEOUT))
		return -ETIMEDOUT;
	if (fspi->dma_status & FSPI_STATUS_ERROR)
		return -EIO;
	if (!(fspi->dma_status & FSPI_STATUS_COMPLETE) &&
	    !(op->data.ecc &&
	      (fspi->dma_status & FSPI_STATUS_PRI_ECC)))
		return -EIO;

	ret = novatek_fspi_stop_dma(fspi);
	if (ret)
		return ret;
	if (fspi->fault)
		return -EIO;

	writel(FSPI_FIFO_CTRL_IDLE, fspi->base + FSPI_FIFO_CTRL_REG);

	return 0;
}

static int novatek_fspi_ecc_data(struct novatek_fspi *fspi, const struct spi_mem_op *op)
{
	struct novatek_fspi_ecc *ecc = fspi->ecc;
	const struct novatek_fspi_ecc_caps *caps = ecc->caps;
	bool read = op->data.dir == SPI_MEM_DATA_IN;
	uint spare_size = ecc->steps * caps->spare_size;
	u32 access;
	uint step;
	int ret;

	writel(ecc->module | FIELD_PREP(FSPI_MODULE_COL_BYTES, op->addr.nbytes - 1),
	       fspi->base + FSPI_MODULE_REG);

	writel(FSPI_ECC_FREE_DISABLED, fspi->base + FSPI_ECC_FREE_REG);

	writel(op->addr.val, fspi->base + FSPI_COLUMN_ADDRESS_REG);
	writel(op->cmd.opcode, fspi->base + FSPI_COMMAND_REG);
	writel(FIELD_PREP(FSPI_DATA_LENGTH_COUNT, ecc->page_size), fspi->base + FSPI_DATA_LENGTH_REG);

	writel((FSPI_CONFIG_IDLE & ~FSPI_CONFIG_CS_MODE) |
	       FIELD_PREP(FSPI_CONFIG_WIDTH, ilog2(op->data.buswidth)),
	       fspi->base + FSPI_CONFIG_REG);

	if (!read) {
		writel(FSPI_SRAM_ACCESS_SPARE_ONLY, fspi->base + FSPI_SRAM_ACCESS_REG);
		readl(fspi->base + FSPI_SRAM_ACCESS_REG);
		ndelay(FSPI_SRAM_DELAY_NS);
		memcpy_toio(fspi->base + FSPI_SPARE_REG,
			    op->data.buf.out + ecc->page_size, spare_size);
	}
	writel(FSPI_SRAM_ACCESS_CONTROLLER, fspi->base + FSPI_SRAM_ACCESS_REG);

	ret = novatek_fspi_dma_data(fspi, op,
				    read ? FSPI_CTRL_OP_READ_PAGE : FSPI_CTRL_OP_WRITE_CACHE);
	if (ret || !read)
		return ret;

	novatek_fspi_release_dma(fspi, op->data.buf.in);

	access = FSPI_SRAM_ACCESS_SPARE | FSPI_SRAM_ACCESS_ERROR;
	writel(access, fspi->base + FSPI_SRAM_ACCESS_REG);
	readl(fspi->base + FSPI_SRAM_ACCESS_REG);
	ndelay(FSPI_SRAM_DELAY_NS);
	memcpy_fromio(op->data.buf.in + ecc->page_size, fspi->base + FSPI_SPARE_REG, spare_size);

	ecc->status = readl(fspi->base + FSPI_RS_STATUS_REG);
	for (step = 0; step < ecc->steps; step++) {
		uint count;
		uint i;

		count = FIELD_GET(FSPI_RS_STATUS_COUNT,
				  ecc->status >> (step * FSPI_RS_STATUS_BITS));
		if (count > caps->strength)
			continue;
		for (i = 0; i < count; i++) {
			uint offset;

			offset = (step * FSPI_RS_ERROR_WORDS_PER_STEP + i) * sizeof(u32);
			ecc->errors[step * caps->strength + i] =
				readl(fspi->base + FSPI_ERROR_REG + offset);
		}
	}

	writel(FSPI_SRAM_ACCESS_CONTROLLER, fspi->base + FSPI_SRAM_ACCESS_REG);

	return 0;
}

static int novatek_fspi_data(struct novatek_fspi *fspi, const struct spi_mem_op *op, ulong rate)
{
	bool read = op->data.dir == SPI_MEM_DATA_IN;
	u32 fifo_ctrl;
	uint remaining = op->data.nbytes;
	u8 *rx = op->data.buf.in;
	const u8 *tx = op->data.buf.out;
	u32 value;
	int ret;

	fifo_ctrl = (read ? 0 : FSPI_FIFO_CTRL_DIR);

	writel((FSPI_CONFIG_IDLE & ~FSPI_CONFIG_CS_LEVEL) |
	       FIELD_PREP(FSPI_CONFIG_WIDTH, ilog2(op->data.buswidth)),
	       fspi->base + FSPI_CONFIG_REG);

	writel(FIELD_PREP(FSPI_DATA_LENGTH_COUNT, remaining), fspi->base + FSPI_DATA_LENGTH_REG);

	if (fspi->dma_buf)
		return novatek_fspi_dma_data(fspi, op,
					     read ? FSPI_CTRL_OP_READ : FSPI_CTRL_OP_WRITE);

	writel(fifo_ctrl, fspi->base + FSPI_FIFO_CTRL_REG);
	ret = novatek_fspi_poll_timeout(fspi->base + FSPI_FIFO_CTRL_REG,
					value, !(value & FSPI_FIFO_CTRL_EN));
	if (ret)
		return ret;

	writel(fifo_ctrl | FSPI_FIFO_CTRL_EN, fspi->base + FSPI_FIFO_CTRL_REG);

	writel(FSPI_CTRL_START |
	       FIELD_PREP(FSPI_CTRL_OP, read ? FSPI_CTRL_OP_READ : FSPI_CTRL_OP_WRITE),
	       fspi->base + FSPI_CTRL_REG);

	while (remaining) {
		uint count = min(remaining, fspi->data->fifo_bank_size);
		uint offset;
		u32 mask = FSPI_FIFO_STATUS_EMPTY;
		u32 ready;

		ready = FSPI_FIFO_STATUS_EMPTY;
		if (read) {
			mask = count == fspi->data->fifo_bank_size ?
				FSPI_FIFO_STATUS_FULL :
				FSPI_FIFO_STATUS_COUNT;
			ready = count == fspi->data->fifo_bank_size ?
				FSPI_FIFO_STATUS_FULL :
				FIELD_PREP(FSPI_FIFO_STATUS_COUNT, DIV_ROUND_UP(count, 4));
		}
		ret = novatek_fspi_poll_timeout(fspi->base + FSPI_FIFO_STATUS_REG,
						value, (value & mask) == ready);
		if (ret)
			return ret;

		for (offset = 0; offset < count; offset += 4) {
			uint bytes = min(count - offset, 4U);
			uint i;

			if (read) {
				value = readl(fspi->base + FSPI_DATA_REG);
				for (i = 0; i < bytes; i++)
					rx[i] = value >> (8 * i);
				rx += bytes;
			} else {
				value = 0;
				for (i = 0; i < bytes; i++)
					value |= (u32)tx[i] << (8 * i);
				writel(value, fspi->base + FSPI_DATA_REG);
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

		delay = DIV_ROUND_UP_ULL(((u64)op->data.nbytes * 8 + 10) * USEC_PER_SEC, rate);
		usleep_range(delay, delay + 10);

		writel(fifo_ctrl, fspi->base + FSPI_FIFO_CTRL_REG);
	}

	ret = novatek_fspi_wait_complete(fspi);
	if (ret)
		return ret;

	writel(FSPI_FIFO_CTRL_IDLE, fspi->base + FSPI_FIFO_CTRL_REG);
	return novatek_fspi_poll_timeout(fspi->base + FSPI_FIFO_CTRL_REG,
					 value, !(value & FSPI_FIFO_CTRL_EN));
}

static int novatek_fspi_exec_one_op(struct spi_mem *mem, const struct spi_mem_op *op)
{
	struct novatek_fspi *fspi;
	uint dummy_cycles = 0;
	ulong rate;
	u32 module = FSPI_MODULE_RAW;
	u32 control = FSPI_CTRL_OP_COMMAND;
	int ret;

	fspi = spi_controller_get_devdata(mem->spi->controller);

	if (op->data.nbytes > FSPI_MAX_TRANSFER)
		return -EMSGSIZE;

	if (fspi->dma_active) {
		ret = novatek_fspi_stop_dma(fspi);
		if (ret)
			return ret;
		novatek_fspi_release_dma(fspi, NULL);
	}

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

	if (op->data.ecc) {
		struct spi_mem_op data = *op;

		data.data.nbytes = fspi->ecc->page_size;
		ret = novatek_fspi_prepare_dma(fspi, &data);
		if (ret)
			return ret;

		ret = novatek_fspi_ecc_data(fspi, op);
		goto out_stop_dma;
	}

	novatek_fspi_prepare_dma(fspi, op);

	if (op->addr.nbytes) {
		module |= FIELD_PREP(FSPI_MODULE_ROW_BYTES, op->addr.nbytes - 1);
		control = FSPI_CTRL_OP_COMMAND_ADDRESS;
	}
	writel(module, fspi->base + FSPI_MODULE_REG);
	writel(op->addr.val, fspi->base + FSPI_ROW_ADDRESS_REG);
	writel(op->cmd.opcode, fspi->base + FSPI_COMMAND_REG);

	writel(FSPI_STATUS_CLEAR_ALL, fspi->base + FSPI_STATUS_REG);

	writel((FSPI_CONFIG_IDLE & ~FSPI_CONFIG_CS_LEVEL) | 0, fspi->base + FSPI_CONFIG_REG);
	writel(FIELD_PREP(FSPI_CTRL_OP, control) | FSPI_CTRL_START, fspi->base + FSPI_CTRL_REG);
	ret = novatek_fspi_wait_complete(fspi);
	if (ret)
		goto out_stop_dma;

	if (op->dummy.nbytes)
		dummy_cycles = op->dummy.nbytes * 8 / op->dummy.buswidth;
	while (dummy_cycles) {
		uint cycles = min(dummy_cycles, 8U);

		writel(FIELD_PREP(FSPI_DUMMY_CLOCK, cycles - 1), fspi->base + FSPI_DUMMY_REG);
		writel(FIELD_PREP(FSPI_CTRL_OP, FSPI_CTRL_OP_DUMMY) | FSPI_CTRL_START,
		       fspi->base + FSPI_CTRL_REG);
		ret = novatek_fspi_wait_complete(fspi);
		if (ret)
			goto out_stop_dma;

		dummy_cycles -= cycles;
	}

	if (op->data.nbytes)
		ret = novatek_fspi_data(fspi, op, rate);

out_stop_dma:
	if (fspi->dma_active) {
		int stop_ret = novatek_fspi_stop_dma(fspi);

		if (stop_ret) {
			fspi->fault = true;
			ret = stop_ret;
			dev_err_ratelimited(fspi->dev, "DMA still active, retaining buffer\n");
		}
	}

	if (!fspi->reset_asserted) {
		writel(FSPI_CONFIG_IDLE, fspi->base + FSPI_CONFIG_REG);
		readl(fspi->base + FSPI_CONFIG_REG);
	}

	if (ret && !fspi->dma_active) {
		int recovery_ret = novatek_fspi_hw_init(fspi);

		if (recovery_ret)
			dev_err_ratelimited(fspi->dev,
					    "controller recovery failed: %d\n", recovery_ret);
	}

	novatek_fspi_release_dma(fspi,
				 !ret && op->data.dir == SPI_MEM_DATA_IN ? op->data.buf.in : NULL);

	return ret;
}

static int novatek_fspi_exec_op(struct spi_mem *mem, const struct spi_mem_op *op)
{
	struct novatek_fspi *fspi;
	struct novatek_fspi_ecc *ecc;
	struct spi_mem_op tail;
	uint length;
	int ret;

	fspi = spi_controller_get_devdata(mem->spi->controller);
	if (!op->data.ecc)
		return novatek_fspi_exec_one_op(mem, op);

	ecc = fspi->ecc;
	if (!fspi->dma_enabled || !ecc || ecc->mem != mem)
		return -EOPNOTSUPP;
	if (op->data.nbytes != ecc->page_size + ecc->oob_size ||
	    !IS_ALIGNED(op->addr.val, ecc->page_size * 2))
		return -EINVAL;

	ecc->result_valid = false;
	ret = novatek_fspi_exec_one_op(mem, op);
	if (ret)
		return ret;

	length = ecc->page_size + ecc->steps * ecc->caps->spare_size;
	if (length < op->data.nbytes) {
		tail = *op;
		tail.addr.val += length;
		tail.data.ecc = false;
		tail.data.nbytes -= length;
		if (op->data.dir == SPI_MEM_DATA_IN)
			tail.data.buf.in += length;
		else
			tail.data.buf.out += length;
		ret = novatek_fspi_exec_one_op(mem, &tail);
		if (ret)
			return ret;
	}
	ecc->result_valid = true;

	return 0;
}

static const struct spi_controller_mem_ops novatek_fspi_mem_ops = {
	.supports_op = novatek_fspi_supports_op,
	.adjust_op_size = novatek_fspi_adjust_op_size,
	.exec_op = novatek_fspi_exec_op,
};

static const struct spi_controller_mem_caps novatek_fspi_mem_caps = {
	.ecc = true,
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
	fspi->data = device_get_match_data(dev);
	if (!fspi->data)
		return -ENODEV;

	fspi->irq = -1;
	init_completion(&fspi->dma_complete);

	fspi->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(fspi->base))
		return PTR_ERR(fspi->base);

	fspi->clk = devm_clk_get_enabled(dev, NULL);
	if (IS_ERR(fspi->clk))
		return dev_err_probe(dev, PTR_ERR(fspi->clk), "failed to enable clock\n");

	ret = clk_set_rate(fspi->clk, FSPI_MAX_SPEED_HZ);
	if (ret)
		return dev_err_probe(dev, ret, "failed to set clock rate\n");

	fspi->reset = devm_reset_control_get_exclusive_deasserted(dev, NULL);
	if (IS_ERR(fspi->reset))
		return dev_err_probe(dev, PTR_ERR(fspi->reset), "failed to get reset\n");

	ret = novatek_fspi_hw_init(fspi);
	if (ret)
		return dev_err_probe(dev, ret, "failed to initialize controller\n");

	if (!device_property_read_bool(dev, "novatek,no-dma")) {
		ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
		if (ret) {
			dev_warn(dev, "DMA unavailable, using PIO\n");
		} else {
			fspi->irq = platform_get_irq(pdev, 0);
			if (fspi->irq < 0)
				return fspi->irq;

			ret = devm_request_irq(dev, fspi->irq,
					       novatek_fspi_irq, 0, dev_name(dev), fspi);
			if (ret)
				return dev_err_probe(dev, ret, "failed to request IRQ\n");
			fspi->dma_enabled = true;
		}
	}

	host->mode_bits = SPI_RX_DUAL | SPI_RX_QUAD | SPI_TX_DUAL | SPI_TX_QUAD;
	host->bits_per_word_mask = SPI_BPW_MASK(8);

	host->num_chipselect = 1;

	host->min_speed_hz = FSPI_MIN_SPEED_HZ;
	host->max_speed_hz = FSPI_MAX_SPEED_HZ;

	host->mem_ops = &novatek_fspi_mem_ops;
	host->mem_caps = &novatek_fspi_mem_caps;

	platform_set_drvdata(pdev, host);

	fspi->ecc_engine.dev = dev;
	fspi->ecc_engine.integration = NAND_ECC_ENGINE_INTEGRATION_PIPELINED;
	fspi->ecc_engine.ops = &novatek_fspi_ecc_ops;
	fspi->ecc_engine.priv = fspi;

	ret = nand_ecc_register_on_host_hw_engine(&fspi->ecc_engine);
	if (ret)
		return ret;

	ret = spi_register_controller(host);
	if (ret)
		nand_ecc_unregister_on_host_hw_engine(&fspi->ecc_engine);
	return ret;
}

static void novatek_fspi_remove(struct platform_device *pdev)
{
	struct spi_controller *host = platform_get_drvdata(pdev);
	struct novatek_fspi *fspi = spi_controller_get_devdata(host);
	int ret;

	spi_unregister_controller(host);

	nand_ecc_unregister_on_host_hw_engine(&fspi->ecc_engine);

	novatek_fspi_mask_irq(fspi);
	ret = novatek_fspi_stop_dma(fspi);
	if (ret)
		dev_err(fspi->dev, "failed to stop DMA: %d\n", ret);

	if (fspi->irq >= 0)
		synchronize_irq(fspi->irq);
	novatek_fspi_release_dma(fspi, NULL);
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

	if (fspi->dma_active) {
		spi_controller_resume(host);
		return -EBUSY;
	}

	novatek_fspi_mask_irq(fspi);
	if (fspi->irq >= 0)
		synchronize_irq(fspi->irq);

	clk_disable_unprepare(fspi->clk);

	return 0;
}

static int novatek_fspi_resume(struct device *dev)
{
	struct spi_controller *host = dev_get_drvdata(dev);
	struct novatek_fspi *fspi = spi_controller_get_devdata(host);
	int ret;

	ret = clk_prepare_enable(fspi->clk);
	if (ret)
		return ret;

	ret = novatek_fspi_hw_init(fspi);
	if (ret) {
		clk_disable_unprepare(fspi->clk);
		return ret;
	}

	return spi_controller_resume(host);
}
#endif

static const struct dev_pm_ops novatek_fspi_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(novatek_fspi_suspend, novatek_fspi_resume)
};

static const struct novatek_fspi_ecc_caps novatek_fspi_rs4_ecc_caps = {
	.step_size = 512,
	.strength = 4,
	.spare_size = 16,
	.bytes = 10,
	.max_steps = 8,
	.symbol_bits = 10,
	.parity_symbols = 8,
};

static const struct novatek_fspi_soc_data na51089_fspi_data = {
	.fifo_bank_size = 64,
	.ecc = &novatek_fspi_rs4_ecc_caps,
};

static const struct of_device_id novatek_fspi_of_match[] = {
	{ .compatible = "novatek,na51089-fspi", .data = &na51089_fspi_data },
	{ }
};
MODULE_DEVICE_TABLE(of, novatek_fspi_of_match);

static struct platform_driver novatek_fspi_driver = {
	.probe = novatek_fspi_probe,
	.remove = novatek_fspi_remove,
	.driver = {
		.name = "novatek-fspi",
		.of_match_table = novatek_fspi_of_match,
		.pm = pm_sleep_ptr(&novatek_fspi_pm_ops),
	},
};
module_platform_driver(novatek_fspi_driver);

MODULE_AUTHOR("Yu-Tung Chang <mtwget@gmail.com>");
MODULE_DESCRIPTION("Novatek flash SPI controller driver");
MODULE_LICENSE("GPL");
