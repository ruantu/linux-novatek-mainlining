// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2026 Yu-Tung Chang <mtwget@gmail.com>
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/jiffies.h>
#include <linux/math64.h>
#include <linux/minmax.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/property.h>
#include <linux/reset.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/time64.h>

#define I2C_CTRL_REG				0x00
#define I2C_STATUS_REG				0x04
#define I2C_CFG_REG				0x08
#define I2C_BUSCLK_REG				0x0c
#define I2C_DATA_REG				0x10
#define I2C_TIMING_REG				0x18
#define I2C_MONITOR_REG				0x1c
#define I2C_DMA_SIZE_REG			0x20
#define I2C_DMA_ADDR_REG			0x24
#define I2C_BUSFREE_REG				0x28

/* I2C_CTRL bit fields */
#define I2C_CTRL_EN				BIT(0)
#define I2C_CTRL_MODE				BIT(1)
#define I2C_CTRL_TB				BIT(3)
#define I2C_CTRL_DT_IRQ				BIT(4)
#define I2C_CTRL_DR_IRQ				BIT(5)
#define I2C_CTRL_BERR_IRQ			BIT(6)
#define I2C_CTRL_AL_IRQ				BIT(10)
#define I2C_CTRL_DMAED_IRQ			BIT(11)
#define I2C_CTRL_TIMEOUT_IRQ			BIT(12)
#define I2C_CTRL_BC				BIT(16)
#define I2C_CTRL_DMA_DIR			BIT(30)
#define I2C_CTRL_DMA_EN				BIT(31)
#define I2C_CTRL_IRQ			\
	(I2C_CTRL_DT_IRQ | I2C_CTRL_DR_IRQ | \
	 I2C_CTRL_BERR_IRQ | I2C_CTRL_AL_IRQ | \
	 I2C_CTRL_DMAED_IRQ | \
	 I2C_CTRL_TIMEOUT_IRQ)
#define I2C_CTRL_DISABLED \
	(FIELD_PREP_CONST(I2C_CTRL_EN, 0) | \
	 FIELD_PREP_CONST(I2C_CTRL_MODE, 0))
#define I2C_CTRL_MASTER_INIT			\
	(FIELD_PREP_CONST(I2C_CTRL_EN, 0) | \
	 FIELD_PREP_CONST(I2C_CTRL_MODE, 1))
#define I2C_CTRL_IDLE				\
	(FIELD_PREP_CONST(I2C_CTRL_EN, 1) | \
	 FIELD_PREP_CONST(I2C_CTRL_MODE, 1))
#define I2C_CTRL_PIO				\
	(I2C_CTRL_IDLE | \
	 FIELD_PREP_CONST(I2C_CTRL_TB, 1) | \
	 FIELD_PREP_CONST(I2C_CTRL_DT_IRQ, 1) | \
	 FIELD_PREP_CONST(I2C_CTRL_DR_IRQ, 1) | \
	 FIELD_PREP_CONST(I2C_CTRL_BERR_IRQ, 1) | \
	 FIELD_PREP_CONST(I2C_CTRL_AL_IRQ, 1) | \
	 FIELD_PREP_CONST(I2C_CTRL_TIMEOUT_IRQ, 1))
#define I2C_CTRL_DMA				\
	(I2C_CTRL_IDLE | \
	 FIELD_PREP_CONST(I2C_CTRL_DMA_EN, 1) | \
	 FIELD_PREP_CONST(I2C_CTRL_BERR_IRQ, 1) | \
	 FIELD_PREP_CONST(I2C_CTRL_AL_IRQ, 1) | \
	 FIELD_PREP_CONST(I2C_CTRL_DMAED_IRQ, 1) | \
	 FIELD_PREP_CONST(I2C_CTRL_TIMEOUT_IRQ, 1))

/* I2C_STATUS bit fields */
#define I2C_STATUS_BUSY				BIT(2)
#define I2C_STATUS_BB				BIT(3)
#define I2C_STATUS_DT				BIT(4)
#define I2C_STATUS_DR				BIT(5)
#define I2C_STATUS_BERR				BIT(6)
#define I2C_STATUS_AL				BIT(10)
#define I2C_STATUS_DMAED			BIT(11)
#define I2C_STATUS_TIMEOUT			BIT(12)
#define I2C_STATUS_BUSFREE			BIT(28)
#define I2C_STATUS_W1C		\
	(I2C_STATUS_DT | I2C_STATUS_DR | \
	 I2C_STATUS_BERR | I2C_STATUS_AL | \
	 I2C_STATUS_DMAED | \
	 I2C_STATUS_TIMEOUT)

/* I2C_CFG bit fields */
#define I2C_CFG_MODE				BIT(0)
#define I2C_CFG_NACK				GENMASK(7, 4)
#define I2C_CFG_START				GENMASK(11, 8)
#define I2C_CFG_START_FIRST			0x1
#define I2C_CFG_STOP				GENMASK(15, 12)
#define I2C_CFG_STOP_FIRST			0x1
#define I2C_CFG_SIZE				GENMASK(18, 16)
#define I2C_CFG_LAST_BYTE(n)			BIT((n) - 1)
#define I2C_CFG_PIO_DEFAULT \
	(FIELD_PREP_CONST(I2C_CFG_MODE, 0) | \
	 FIELD_PREP_CONST(I2C_CFG_NACK, 0) | \
	 FIELD_PREP_CONST(I2C_CFG_START, 0) | \
	 FIELD_PREP_CONST(I2C_CFG_STOP, 0) | \
	 FIELD_PREP_CONST(I2C_CFG_SIZE, 0))
#define I2C_CFG_DMA_DEFAULT \
	(FIELD_PREP_CONST(I2C_CFG_MODE, 1) | \
	 FIELD_PREP_CONST(I2C_CFG_NACK, 0) | \
	 FIELD_PREP_CONST(I2C_CFG_START, 0) | \
	 FIELD_PREP_CONST(I2C_CFG_STOP, 0) | \
	 FIELD_PREP_CONST(I2C_CFG_SIZE, 0))

/* I2C_BUSCLK bit fields */
#define I2C_BUSCLK_LOW				GENMASK(9, 0)
#define I2C_BUSCLK_HIGH				GENMASK(25, 16)

/* I2C_TIMING bit fields */
#define I2C_TIMING_TSR				GENMASK(9, 0)
#define I2C_TIMING_TSR_DEFAULT			0x001
#define I2C_TIMING_GSR				GENMASK(15, 12)
#define I2C_TIMING_GSR_DEFAULT			0x2
#define I2C_TIMING_TIMEOUT			GENMASK(25, 16)
#define I2C_TIMING_TIMEOUT_DEFAULT		0x3ff
#define I2C_TIMING_DEFAULT \
	(FIELD_PREP_CONST(I2C_TIMING_TSR, I2C_TIMING_TSR_DEFAULT) | \
	 FIELD_PREP_CONST(I2C_TIMING_GSR, I2C_TIMING_GSR_DEFAULT) | \
	 FIELD_PREP_CONST(I2C_TIMING_TIMEOUT, I2C_TIMING_TIMEOUT_DEFAULT))

/* I2C_MONITOR bit fields */
#define I2C_MONITOR_SDA				BIT(0)
#define I2C_MONITOR_SCL				BIT(1)
#define I2C_MONITOR_HIGH			(I2C_MONITOR_SDA | I2C_MONITOR_SCL)

/* I2C_DMA_SIZE bit fields */
#define I2C_DMA_SIZE_COUNT			GENMASK(15, 0)

/* I2C_BUSFREE bit fields */
#define I2C_BUSFREE_COUNT			GENMASK(19, 0)
#define I2C_BUSFREE_COUNT_DEFAULT		0x000e2

#define I2C_PIO_BYTES				4

#define I2C_DMA_ALIGN				4
#define I2C_DMA_MIN_BYTES			32

#define I2C_POLL_US				10
#define I2C_BUS_TIMEOUT_US			200000
#define I2C_XFER_TIMEOUT_MS			1000

#define I2C_SOURCE_RATE				48000000

struct novatek_i2c {
	void __iomem *base;
	struct clk *clk;
	struct reset_control *reset;
	struct i2c_adapter adap;
	struct i2c_bus_recovery_info recovery;
	struct completion complete;
	spinlock_t lock;
	void *dma_buf;
	dma_addr_t dma_addr;
	enum dma_data_direction dma_dir;
	uint dma_len;
	u32 busclk;
	u32 irq_status;
	int irq;
	bool clock_enabled;
	bool initialized;
	bool dma_enabled;
	bool dma_active;
};

static void novatek_i2c_init(struct novatek_i2c *i2c)
{
	u32 timing;

	writel(I2C_CTRL_MASTER_INIT, i2c->base + I2C_CTRL_REG);

	writel(I2C_CFG_PIO_DEFAULT, i2c->base + I2C_CFG_REG);

	writel(i2c->busclk, i2c->base + I2C_BUSCLK_REG);

	timing = I2C_TIMING_DEFAULT;
	writel(timing, i2c->base + I2C_TIMING_REG);

	writel(FIELD_PREP(I2C_BUSFREE_COUNT, I2C_BUSFREE_COUNT_DEFAULT), i2c->base + I2C_BUSFREE_REG);

	writel(I2C_STATUS_W1C, i2c->base + I2C_STATUS_REG);

	writel(I2C_CTRL_IDLE, i2c->base + I2C_CTRL_REG);
	i2c->initialized = true;
}

static void novatek_i2c_disable(struct novatek_i2c *i2c)
{
	ulong flags;

	if (!i2c->initialized)
		return;

	spin_lock_irqsave(&i2c->lock, flags);
	writel(I2C_CTRL_DISABLED, i2c->base + I2C_CTRL_REG);
	readl(i2c->base + I2C_CTRL_REG);
	spin_unlock_irqrestore(&i2c->lock, flags);
	synchronize_irq(i2c->irq);
	i2c->initialized = false;
}

static int novatek_i2c_reset(struct novatek_i2c *i2c)
{
	int ret;

	novatek_i2c_disable(i2c);

	ret = reset_control_assert(i2c->reset);
	if (ret)
		return ret;
	i2c->dma_active = false;
	udelay(10);

	ret = reset_control_deassert(i2c->reset);
	if (ret)
		return ret;

	novatek_i2c_init(i2c);

	return 0;
}

static int novatek_i2c_wait_bus_free(struct novatek_i2c *i2c)
{
	u32 status;
	int ret;

	ret = readl_poll_timeout(i2c->base + I2C_STATUS_REG,
				 status, (status & I2C_STATUS_BUSFREE) &&
					 !(status & (I2C_STATUS_BUSY | I2C_STATUS_BB)),
				 I2C_POLL_US, I2C_BUS_TIMEOUT_US);
	return ret ? -EBUSY : 0;
}

static int novatek_i2c_recover_bus(struct i2c_adapter *adap)
{
	struct novatek_i2c *i2c = i2c_get_adapdata(adap);
	u32 value;
	int ret, reset_ret;

	if (!i2c->clock_enabled)
		return -EHOSTDOWN;
	if (!i2c->initialized)
		return -EIO;

	value = I2C_CTRL_IDLE | I2C_CTRL_BC;
	writel(value, i2c->base + I2C_CTRL_REG);
	ret = readl_poll_timeout(i2c->base + I2C_CTRL_REG,
				 value, !(value & I2C_CTRL_BC),
				 I2C_POLL_US, I2C_BUS_TIMEOUT_US);

	/* Bus clear leaves a state that can add clocks to the next transfer. */
	reset_ret = novatek_i2c_reset(i2c);
	if (reset_ret)
		return reset_ret;
	if (ret)
		return ret;

	ret = readl_poll_timeout(i2c->base + I2C_MONITOR_REG,
				 value, (value & I2C_MONITOR_HIGH) == I2C_MONITOR_HIGH,
				 I2C_POLL_US, I2C_BUS_TIMEOUT_US);
	return ret ? -EBUSY : 0;
}

static irqreturn_t novatek_i2c_irq(int irq, void *data)
{
	struct novatek_i2c *i2c = data;
	irqreturn_t ret = IRQ_NONE;
	u32 ctrl, status;

	spin_lock(&i2c->lock);
	ctrl = readl(i2c->base + I2C_CTRL_REG);
	status = readl(i2c->base + I2C_STATUS_REG);
	status &= ctrl & I2C_CTRL_IRQ;
	if (!status)
		goto out_unlock;
	ctrl &= ~I2C_CTRL_IRQ;
	if (i2c->dma_active)
		ctrl &= ~I2C_CTRL_DMA_EN;
	writel(ctrl, i2c->base + I2C_CTRL_REG);
	writel(status, i2c->base + I2C_STATUS_REG);
	i2c->irq_status = status;
	complete(&i2c->complete);
	ret = IRQ_HANDLED;

out_unlock:
	spin_unlock(&i2c->lock);

	return ret;
}

static int novatek_i2c_packet(struct novatek_i2c *i2c, u32 config, u32 *data, bool read)
{
	ulong flags, time_left;
	u32 status, expected;

	spin_lock_irqsave(&i2c->lock, flags);
	reinit_completion(&i2c->complete);
	i2c->irq_status = 0;

	writel(I2C_STATUS_W1C, i2c->base + I2C_STATUS_REG);

	if (!read)
		writel(*data, i2c->base + I2C_DATA_REG);
	writel(config, i2c->base + I2C_CFG_REG);

	writel(I2C_CTRL_PIO, i2c->base + I2C_CTRL_REG);
	spin_unlock_irqrestore(&i2c->lock, flags);

	time_left = wait_for_completion_timeout(&i2c->complete, i2c->adap.timeout);

	spin_lock_irqsave(&i2c->lock, flags);
	status = i2c->irq_status;
	if (!time_left) {
		writel(I2C_CTRL_IDLE, i2c->base + I2C_CTRL_REG);
		readl(i2c->base + I2C_CTRL_REG);
	}
	spin_unlock_irqrestore(&i2c->lock, flags);
	if (!time_left) {
		synchronize_irq(i2c->irq);
		return -ETIMEDOUT;
	}
	if (status & I2C_STATUS_AL)
		return -EAGAIN;
	if (status & I2C_STATUS_TIMEOUT)
		return -ETIMEDOUT;
	if (status & I2C_STATUS_BERR)
		return -EREMOTEIO;
	expected = read ? I2C_STATUS_DR : I2C_STATUS_DT;
	if (!(status & expected))
		return -EIO;

	if (read)
		*data = readl(i2c->base + I2C_DATA_REG);

	return 0;
}

static void novatek_i2c_prepare_dma(struct novatek_i2c *i2c, struct i2c_msg *msg)
{
	struct device *dev = i2c->adap.dev.parent;
	enum dma_data_direction dir;
	dma_addr_t addr;
	void *buf;
	uint len;

	if (!i2c->dma_enabled || msg->len <= I2C_DMA_MIN_BYTES)
		return;

	len = round_down(msg->len - 1, I2C_DMA_ALIGN);
	buf = kmalloc(len, GFP_KERNEL);
	if (!buf)
		return;
	if (!(msg->flags & I2C_M_RD))
		memcpy(buf, msg->buf, len);

	dir = msg->flags & I2C_M_RD ? DMA_FROM_DEVICE : DMA_TO_DEVICE;
	addr = dma_map_single(dev, buf, len, dir);
	if (dma_mapping_error(dev, addr))
		goto out_free_buf;
	if (!IS_ALIGNED(addr, I2C_DMA_ALIGN)) {
		dma_unmap_single(dev, addr, len, dir);
		goto out_free_buf;
	}
	i2c->dma_buf = buf;
	i2c->dma_addr = addr;
	i2c->dma_len = len;
	i2c->dma_dir = dir;

	return;

out_free_buf:
	kfree(buf);
}

static void novatek_i2c_release_dma(struct novatek_i2c *i2c, struct i2c_msg *msg, bool xferred)
{
	if (!i2c->dma_buf || i2c->dma_active)
		return;

	dma_unmap_single(i2c->adap.dev.parent, i2c->dma_addr, i2c->dma_len, i2c->dma_dir);
	if (xferred && i2c->dma_dir == DMA_FROM_DEVICE)
		memcpy(msg->buf, i2c->dma_buf, i2c->dma_len);

	kfree(i2c->dma_buf);
	i2c->dma_buf = NULL;
	i2c->dma_len = 0;
}

static int novatek_i2c_dma_xfer(struct novatek_i2c *i2c, bool *recover_bus)
{
	ulong flags, time_left, timeout;
	u32 ctrl, period, status, value;
	bool owns_bus;
	int ret;

	*recover_bus = false;

	period = FIELD_GET(I2C_BUSCLK_LOW, i2c->busclk) +
		 FIELD_GET(I2C_BUSCLK_HIGH, i2c->busclk) + I2C_TIMING_GSR_DEFAULT + 4;
	timeout = DIV_ROUND_UP_ULL((u64)i2c->dma_len * 9 * period * HZ,
				   I2C_SOURCE_RATE) + i2c->adap.timeout;

	spin_lock_irqsave(&i2c->lock, flags);
	reinit_completion(&i2c->complete);
	i2c->irq_status = 0;

	writel(I2C_STATUS_W1C, i2c->base + I2C_STATUS_REG);

	writel(I2C_CFG_DMA_DEFAULT, i2c->base + I2C_CFG_REG);

	writel(i2c->dma_addr, i2c->base + I2C_DMA_ADDR_REG);
	writel(FIELD_PREP(I2C_DMA_SIZE_COUNT, i2c->dma_len), i2c->base + I2C_DMA_SIZE_REG);

	i2c->dma_active = true;
	ctrl = I2C_CTRL_DMA | (i2c->dma_dir == DMA_TO_DEVICE ? I2C_CTRL_DMA_DIR : 0);
	writel(ctrl, i2c->base + I2C_CTRL_REG);
	spin_unlock_irqrestore(&i2c->lock, flags);

	time_left = wait_for_completion_timeout(&i2c->complete, timeout);

	spin_lock_irqsave(&i2c->lock, flags);
	status = i2c->irq_status | readl(i2c->base + I2C_STATUS_REG);
	owns_bus = status & I2C_STATUS_BUSY;

	ctrl = readl(i2c->base + I2C_CTRL_REG);
	ctrl &= ~(I2C_CTRL_IRQ | I2C_CTRL_DMA_EN);
	writel(ctrl, i2c->base + I2C_CTRL_REG);
	readl(i2c->base + I2C_CTRL_REG);
	spin_unlock_irqrestore(&i2c->lock, flags);
	synchronize_irq(i2c->irq);

	if (!(status & I2C_STATUS_DMAED)) {
		ret = readl_poll_timeout(i2c->base + I2C_STATUS_REG,
					 value, value & I2C_STATUS_DMAED,
					 I2C_POLL_US, I2C_BUS_TIMEOUT_US);
		status |= value & (I2C_STATUS_BERR | I2C_STATUS_AL |
				   I2C_STATUS_BB | I2C_STATUS_TIMEOUT);
		if (ret) {
			ret = novatek_i2c_reset(i2c);
			if (ret)
				return ret;
			goto out_set_recover_bus;
		}
	}
	i2c->dma_active = false;
	writel(I2C_STATUS_DMAED, i2c->base + I2C_STATUS_REG);

	writel(I2C_CFG_PIO_DEFAULT, i2c->base + I2C_CFG_REG);

	if (!time_left)
		goto out_set_recover_bus;
	if (status & I2C_STATUS_AL)
		return -EAGAIN;
	if (status & I2C_STATUS_TIMEOUT)
		goto out_set_recover_bus;
	if (status & I2C_STATUS_BERR)
		return -EREMOTEIO;
	if (!(status & I2C_STATUS_DMAED))
		return -EIO;

	return 0;

out_set_recover_bus:
	*recover_bus = owns_bus && !(status & (I2C_STATUS_AL | I2C_STATUS_BB));
	return -ETIMEDOUT;
}

static int novatek_i2c_xfer(struct i2c_adapter *adap, struct i2c_msg *msgs, int num)
{
	struct novatek_i2c *i2c = i2c_get_adapdata(adap);
	struct i2c_msg *msg;
	__le32 bytes;
	u32 config, data, status, last_byte;
	uint count, pos;
	bool read, stop;
	bool recover_bus = false;
	int i, ret, stop_ret;

	for (i = 0; i < num; i++) {
		if (msgs[i].flags & I2C_M_TEN)
			return -EAFNOSUPPORT;
		if (msgs[i].flags & ~(I2C_M_RD | I2C_M_DMA_SAFE | I2C_M_STOP))
			return -EOPNOTSUPP;
		if (msgs[i].addr > 0x7f || !msgs[i].buf)
			return -EINVAL;
	}

	if (!i2c->initialized)
		return -EIO;

	ret = novatek_i2c_wait_bus_free(i2c);
	if (ret)
		return ret;

	for (i = 0; i < num; i++) {
		msg = &msgs[i];
		read = msg->flags & I2C_M_RD;
		stop = i == num - 1 || (msg->flags & I2C_M_STOP);
		novatek_i2c_prepare_dma(i2c, msg);

		data = i2c_8bit_addr_from_msg(msg);
		config = FIELD_PREP(I2C_CFG_START, I2C_CFG_START_FIRST) |
			 FIELD_PREP(I2C_CFG_SIZE, 1);
		ret = novatek_i2c_packet(i2c, config, &data, false);
		if (ret) {
			if (ret == -EREMOTEIO)
				ret = -ENXIO;
			goto out_stop_bus;
		}

		if (i2c->dma_buf) {
			ret = novatek_i2c_dma_xfer(i2c, &recover_bus);
			if (ret)
				goto out_stop_bus;
		}

		for (pos = i2c->dma_len; pos < msg->len; pos += count) {
			count = min_t(uint, msg->len - pos, I2C_PIO_BYTES);
			config = FIELD_PREP(I2C_CFG_SIZE, count);
			if (pos + count == msg->len) {
				last_byte = I2C_CFG_LAST_BYTE(count);
				if (read)
					config |= FIELD_PREP(I2C_CFG_NACK, last_byte);
				if (stop)
					config |= FIELD_PREP(I2C_CFG_STOP, last_byte);
			}

			if (!read) {
				bytes = 0;
				memcpy(&bytes, msg->buf + pos, count);
				data = le32_to_cpu(bytes);
			}
			ret = novatek_i2c_packet(i2c, config, &data, read);
			if (ret)
				goto out_stop_bus;
			if (read) {
				bytes = cpu_to_le32(data);
				memcpy(msg->buf + pos, &bytes, count);
			}
		}

		if (stop) {
			ret = novatek_i2c_wait_bus_free(i2c);
			if (ret)
				goto out_stop_bus;
		}

		novatek_i2c_release_dma(i2c, msg, true);
	}

	return num;

out_stop_bus:
	if (i2c->dma_active)
		return ret;
	if (!i2c->initialized)
		goto out_unmap_dma;

	if (ret == -ENXIO || ret == -EREMOTEIO) {
		config = FIELD_PREP(I2C_CFG_STOP, I2C_CFG_STOP_FIRST) | FIELD_PREP(I2C_CFG_SIZE, 1);
		writel(config, i2c->base + I2C_CFG_REG);
		writel(I2C_CTRL_IDLE | I2C_CTRL_TB, i2c->base + I2C_CTRL_REG);

		stop_ret = readl_poll_timeout(i2c->base + I2C_STATUS_REG,
					      status, !(status & I2C_STATUS_BUSY),
					      I2C_POLL_US, I2C_BUS_TIMEOUT_US);
		if (stop_ret)
			ret = stop_ret;
	}

	novatek_i2c_init(i2c);

	if (recover_bus &&
	    (readl(i2c->base + I2C_MONITOR_REG) &
	     I2C_MONITOR_HIGH) == I2C_MONITOR_SCL) {
		stop_ret = i2c_recover_bus(adap);
		if (stop_ret)
			dev_warn_ratelimited(adap->dev.parent,
					     "Failed to recover bus: %d\n", stop_ret);
	}

out_unmap_dma:
	novatek_i2c_release_dma(i2c, msg, false);
	return ret;
}

static u32 novatek_i2c_functionality(struct i2c_adapter *adap)
{
	return I2C_FUNC_I2C | (I2C_FUNC_SMBUS_EMUL & ~I2C_FUNC_SMBUS_QUICK);
}

static const struct i2c_algorithm novatek_i2c_algorithm = {
	.xfer = novatek_i2c_xfer,
	.functionality = novatek_i2c_functionality,
};

static const struct i2c_adapter_quirks novatek_i2c_quirks = {
	.flags = I2C_AQ_NO_ZERO_LEN,
};

static int novatek_i2c_calc_clock(struct novatek_i2c *i2c, u32 frequency)
{
	ulong rate = clk_get_rate(i2c->clk);
	u32 low, high, min_low, min_high, period;
	u32 low_ns, high_ns;

	if (rate != I2C_SOURCE_RATE)
		return -EINVAL;
	if (!frequency)
		return -EINVAL;

	if (frequency <= I2C_MAX_STANDARD_MODE_FREQ) {
		low_ns = 4700;
		high_ns = 4000;
	} else if (frequency <= I2C_MAX_FAST_MODE_FREQ) {
		low_ns = 1300;
		high_ns = 600;
	} else {
		low_ns = 500;
		high_ns = 260;
	}
	min_low = div_u64((u64)rate * low_ns, NSEC_PER_SEC) + 1;
	min_high = div_u64((u64)rate * high_ns, NSEC_PER_SEC) + 1;
	low = (rate / frequency - I2C_TIMING_GSR_DEFAULT) / 2;
	high = low;
	if (low < min_low) {
		high -= min_low - low;
		low = min_low;
	}
	high = max(high, min_high);

	low--;
	high -= 3;
	period = DIV_ROUND_UP(rate, frequency);
	if (low + high + I2C_TIMING_GSR_DEFAULT + 4 < period)
		low = period - high - I2C_TIMING_GSR_DEFAULT - 4;
	if (!FIELD_FIT(I2C_BUSCLK_LOW, low) || !FIELD_FIT(I2C_BUSCLK_HIGH, high))
		return -EINVAL;
	i2c->busclk = FIELD_PREP(I2C_BUSCLK_LOW, low) | FIELD_PREP(I2C_BUSCLK_HIGH, high);

	return 0;
}

static int novatek_i2c_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct novatek_i2c *i2c;
	u32 frequency = I2C_MAX_STANDARD_MODE_FREQ;
	int ret;

	i2c = devm_kzalloc(dev, sizeof(*i2c), GFP_KERNEL);
	if (!i2c)
		return -ENOMEM;

	i2c->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(i2c->base))
		return PTR_ERR(i2c->base);

	i2c->irq = platform_get_irq(pdev, 0);
	if (i2c->irq < 0)
		return i2c->irq;

	i2c->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(i2c->clk))
		return dev_err_probe(dev, PTR_ERR(i2c->clk), "Failed to get clock\n");

	i2c->reset = devm_reset_control_get_exclusive(dev, NULL);
	if (IS_ERR(i2c->reset))
		return dev_err_probe(dev, PTR_ERR(i2c->reset), "Failed to get reset\n");

	if (device_property_present(dev, "clock-frequency")) {
		ret = device_property_read_u32(dev, "clock-frequency", &frequency);
		if (ret)
			return dev_err_probe(dev, ret, "Invalid bus frequency\n");
	}
	ret = novatek_i2c_calc_clock(i2c, frequency);
	if (ret)
		return dev_err_probe(dev, ret, "Unsupported clock configuration\n");

	if (!device_property_read_bool(dev, "novatek,no-dma")) {
		ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
		if (ret)
			dev_warn(dev, "DMA unavailable, using PIO\n");
		else
			i2c->dma_enabled = true;
	}

	spin_lock_init(&i2c->lock);
	init_completion(&i2c->complete);

	platform_set_drvdata(pdev, i2c);
	i2c_set_adapdata(&i2c->adap, i2c);

	i2c->adap.owner = THIS_MODULE;

	i2c->adap.algo = &novatek_i2c_algorithm;
	i2c->adap.quirks = &novatek_i2c_quirks;

	i2c->adap.timeout = msecs_to_jiffies(I2C_XFER_TIMEOUT_MS);
	i2c->adap.dev.parent = dev;
	device_set_node(&i2c->adap.dev, dev_fwnode(dev));
	strscpy(i2c->adap.name, "Novatek I2C controller", sizeof(i2c->adap.name));

	i2c->recovery.recover_bus = novatek_i2c_recover_bus;
	i2c->adap.bus_recovery_info = &i2c->recovery;

	ret = clk_prepare_enable(i2c->clk);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to enable clock\n");
	i2c->clock_enabled = true;

	ret = novatek_i2c_reset(i2c);
	if (ret)
		goto out_disable_clk;

	ret = devm_request_irq(dev, i2c->irq, novatek_i2c_irq, 0, dev_name(dev), i2c);
	if (ret)
		goto out_disable_controller_no_irq;

	ret = i2c_add_adapter(&i2c->adap);
	if (ret)
		goto out_disable_controller;

	return 0;

out_disable_controller:
	novatek_i2c_disable(i2c);
	devm_free_irq(dev, i2c->irq, i2c);
	goto out_assert_reset;
out_disable_controller_no_irq:
	writel(I2C_CTRL_DISABLED, i2c->base + I2C_CTRL_REG);
	i2c->initialized = false;
out_assert_reset:
	reset_control_assert(i2c->reset);
out_disable_clk:
	clk_disable_unprepare(i2c->clk);
	i2c->clock_enabled = false;
	return ret;
}

static void novatek_i2c_remove(struct platform_device *pdev)
{
	struct novatek_i2c *i2c = platform_get_drvdata(pdev);
	int ret;

	i2c_del_adapter(&i2c->adap);

	novatek_i2c_disable(i2c);
	devm_free_irq(&pdev->dev, i2c->irq, i2c);

	ret = reset_control_assert(i2c->reset);
	if (!ret)
		i2c->dma_active = false;
	if (i2c->dma_active) {
		dev_err(&pdev->dev, "DMA not stopped, retaining buffer\n");
		return;
	}

	novatek_i2c_release_dma(i2c, NULL, false);
	if (i2c->clock_enabled)
		clk_disable_unprepare(i2c->clk);
}

#ifdef CONFIG_PM_SLEEP
static int novatek_i2c_suspend(struct device *dev)
{
	struct novatek_i2c *i2c = dev_get_drvdata(dev);

	i2c_mark_adapter_suspended(&i2c->adap);
	if (i2c->dma_active) {
		i2c_mark_adapter_resumed(&i2c->adap);
		return -EBUSY;
	}

	novatek_i2c_disable(i2c);

	if (i2c->clock_enabled)
		clk_disable_unprepare(i2c->clk);
	i2c->clock_enabled = false;

	return 0;
}

static int novatek_i2c_resume(struct device *dev)
{
	struct novatek_i2c *i2c = dev_get_drvdata(dev);
	int ret;

	ret = clk_prepare_enable(i2c->clk);
	if (ret)
		return ret;
	i2c->clock_enabled = true;

	ret = reset_control_deassert(i2c->reset);
	if (ret) {
		clk_disable_unprepare(i2c->clk);
		i2c->clock_enabled = false;
		return ret;
	}

	novatek_i2c_init(i2c);

	i2c_mark_adapter_resumed(&i2c->adap);

	return 0;
}
#endif

static const struct dev_pm_ops novatek_i2c_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(novatek_i2c_suspend, novatek_i2c_resume)
};

static const struct of_device_id novatek_i2c_of_match[] = {
	{ .compatible = "novatek,na51089-i2c" },
	{ }
};
MODULE_DEVICE_TABLE(of, novatek_i2c_of_match);

static struct platform_driver novatek_i2c_driver = {
	.probe = novatek_i2c_probe,
	.remove = novatek_i2c_remove,
	.driver = {
		.name = "i2c-novatek",
		.of_match_table = novatek_i2c_of_match,
		.pm = pm_sleep_ptr(&novatek_i2c_pm_ops),
	},
};
module_platform_driver(novatek_i2c_driver);

MODULE_AUTHOR("Yu-Tung Chang <mtwget@gmail.com>");
MODULE_DESCRIPTION("Novatek I2C controller driver");
MODULE_LICENSE("GPL");
