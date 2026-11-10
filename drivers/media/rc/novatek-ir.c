// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2026 Yu-Tung Chang <mtwget@gmail.com>
 */

#include <linux/bitfield.h>
#include <linux/bitrev.h>
#include <linux/clk.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/math64.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/reset.h>
#include <linux/spinlock.h>

#include <media/rc-core.h>

#define REMOTE_CTRL_REG				0x00
#define REMOTE_INT_REG				0x08
#define REMOTE_STATUS_REG			0x0c
#define REMOTE_DATA_LOW_REG			0x10
#define REMOTE_TH0_REG				0x20
#define REMOTE_TH1_REG				0x24
#define REMOTE_TH2_REG				0x28

/* REMOTE_CTRL bit fields */
#define REMOTE_CTRL_EN				BIT(0)
#define REMOTE_CTRL_INV				BIT(5)
#define REMOTE_CTRL_MSB				BIT(6)
#define REMOTE_CTRL_REPEAT			BIT(7)
#define REMOTE_CTRL_HEADER			GENMASK(9, 8)
#define REMOTE_CTRL_HEADER_FORCE		1
#define REMOTE_CTRL_HEADER_OPTIONAL		2
#define REMOTE_CTRL_LENGTH			GENMASK(22, 16)
#define REMOTE_CTRL_DISABLED \
	FIELD_PREP_CONST(REMOTE_CTRL_EN, 0)
#define REMOTE_CTRL_NEC \
	(FIELD_PREP_CONST(REMOTE_CTRL_INV, 1) | \
	 FIELD_PREP_CONST(REMOTE_CTRL_MSB, 1) | \
	 FIELD_PREP_CONST(REMOTE_CTRL_REPEAT, 1) | \
	 FIELD_PREP_CONST(REMOTE_CTRL_HEADER, REMOTE_CTRL_HEADER_FORCE) | \
	 FIELD_PREP_CONST(REMOTE_CTRL_LENGTH, 32))
#define REMOTE_CTRL_JVC \
	(FIELD_PREP_CONST(REMOTE_CTRL_INV, 1) | \
	 FIELD_PREP_CONST(REMOTE_CTRL_MSB, 1) | \
	 FIELD_PREP_CONST(REMOTE_CTRL_HEADER, REMOTE_CTRL_HEADER_OPTIONAL) | \
	 FIELD_PREP_CONST(REMOTE_CTRL_LENGTH, 16))

/* REMOTE_INT bit fields */
#define REMOTE_INT_READY			BIT(0)
#define REMOTE_INT_ERROR			BIT(1)
#define REMOTE_INT_MATCH			BIT(2)
#define REMOTE_INT_REPEAT			BIT(3)
#define REMOTE_INT_OVERRUN			BIT(4)
#define REMOTE_INT_DISABLED \
	(FIELD_PREP_CONST(REMOTE_INT_READY, 0) | \
	 FIELD_PREP_CONST(REMOTE_INT_ERROR, 0) | \
	 FIELD_PREP_CONST(REMOTE_INT_MATCH, 0) | \
	 FIELD_PREP_CONST(REMOTE_INT_REPEAT, 0) | \
	 FIELD_PREP_CONST(REMOTE_INT_OVERRUN, 0))
#define REMOTE_INT_RECEIVE \
	(FIELD_PREP_CONST(REMOTE_INT_READY, 1) | \
	 FIELD_PREP_CONST(REMOTE_INT_ERROR, 1) | \
	 FIELD_PREP_CONST(REMOTE_INT_REPEAT, 1))
#define REMOTE_INT_CLEAR \
	(FIELD_PREP_CONST(REMOTE_INT_READY, 1) | \
	 FIELD_PREP_CONST(REMOTE_INT_ERROR, 1) | \
	 FIELD_PREP_CONST(REMOTE_INT_MATCH, 1) | \
	 FIELD_PREP_CONST(REMOTE_INT_REPEAT, 1) | \
	 FIELD_PREP_CONST(REMOTE_INT_OVERRUN, 1))

/* REMOTE_TH0 bit fields */
#define REMOTE_TH0_HEADER			GENMASK(15, 0)
#define REMOTE_TH0_REPEAT			GENMASK(31, 16)

/* REMOTE_TH1 bit fields */
#define REMOTE_TH1_LOGIC			GENMASK(10, 0)
#define REMOTE_TH1_GLITCH			GENMASK(23, 16)

/* REMOTE_TH2 bit fields */
#define REMOTE_TH2_ERROR			GENMASK(15, 0)

#define REMOTE_NEC_PROTOCOLS \
	(RC_PROTO_BIT_NEC | RC_PROTO_BIT_NECX | RC_PROTO_BIT_NEC32)

#define REMOTE_HEADER_US			6000
#define REMOTE_LOGIC_US				1100
#define REMOTE_GLITCH_US			62
#define REMOTE_REPEAT_US			3000
#define REMOTE_NEC_ERROR_US			20000
#define REMOTE_JVC_ERROR_US			15000
#define REMOTE_REPEAT_TIMEOUT_MS		150

struct novatek_ir {
	void __iomem *base;
	struct clk *clk;
	struct rc_dev *rc;
	/* IRQs need a private mask: rc-core updates its copy after our callback. */
	spinlock_t lock;
	u64 protocols;
	u32 threshold0;
	u32 threshold1;
	u32 nec_error;
	u32 jvc_error;
	ulong repeat_deadline;
	int irq;
	bool repeat_valid;
	bool running;
	bool clock_enabled;
};

static void novatek_ir_stop(struct novatek_ir *ir)
{
	writel(REMOTE_INT_DISABLED, ir->base + REMOTE_INT_REG);
	writel(REMOTE_CTRL_DISABLED, ir->base + REMOTE_CTRL_REG);
	writel(REMOTE_INT_CLEAR, ir->base + REMOTE_STATUS_REG);
	ir->repeat_valid = false;
}

static void novatek_ir_configure(struct novatek_ir *ir)
{
	u32 control, error;

	novatek_ir_stop(ir);
	if (!ir->protocols)
		return;

	if (ir->protocols & REMOTE_NEC_PROTOCOLS) {
		control = REMOTE_CTRL_NEC;
		error = ir->nec_error;
	} else {
		control = REMOTE_CTRL_JVC;
		error = ir->jvc_error;
	}

	writel(ir->threshold0, ir->base + REMOTE_TH0_REG);
	writel(ir->threshold1, ir->base + REMOTE_TH1_REG);
	writel(FIELD_PREP(REMOTE_TH2_ERROR, error), ir->base + REMOTE_TH2_REG);
	writel(control, ir->base + REMOTE_CTRL_REG);

	writel(REMOTE_INT_RECEIVE, ir->base + REMOTE_INT_REG);
	writel(control | REMOTE_CTRL_EN, ir->base + REMOTE_CTRL_REG);
}

static int novatek_ir_change_protocol(struct rc_dev *rc, u64 *protocols)
{
	struct novatek_ir *ir = rc->priv;
	ulong flags;

	*protocols &= rc->allowed_protocols;
	if (*protocols & REMOTE_NEC_PROTOCOLS)
		*protocols &= REMOTE_NEC_PROTOCOLS;
	if (*protocols & RC_PROTO_BIT_JVC)
		*protocols &= RC_PROTO_BIT_JVC;

	spin_lock_irqsave(&ir->lock, flags);
	ir->protocols = *protocols;
	ir->repeat_valid = false;
	if (ir->running)
		novatek_ir_configure(ir);
	spin_unlock_irqrestore(&ir->lock, flags);

	return 0;
}

static irqreturn_t novatek_ir_irq(int irq, void *data)
{
	struct novatek_ir *ir = data;
	enum rc_proto protocol;
	u32 status, raw, scancode;
	irqreturn_t ret = IRQ_NONE;

	spin_lock(&ir->lock);
	if (!ir->running)
		goto out_unlock;

	status = readl(ir->base + REMOTE_STATUS_REG);
	status &= readl(ir->base + REMOTE_INT_REG);
	if (!status)
		goto out_unlock;

	writel(status, ir->base + REMOTE_STATUS_REG);
	ret = IRQ_HANDLED;
	if (!ir->protocols)
		goto out_unlock;

	if (status & REMOTE_INT_READY) {
		raw = readl(ir->base + REMOTE_DATA_LOW_REG);
		ir->repeat_valid = false;
		if (ir->protocols & REMOTE_NEC_PROTOCOLS) {
			scancode = ir_nec_bytes_to_scancode(bitrev8(raw >> 24), bitrev8(raw >> 16),
							    bitrev8(raw >> 8), bitrev8(raw),
							    &protocol);
			if (!(ir->protocols & BIT_ULL(protocol)))
				goto out_unlock;

			ir->repeat_valid = true;
			ir->repeat_deadline = jiffies + msecs_to_jiffies(REMOTE_REPEAT_TIMEOUT_MS);
		}

		if (ir->protocols & RC_PROTO_BIT_JVC) {
			scancode = bitrev8(raw >> 8) << 8 | bitrev8(raw);
			protocol = RC_PROTO_JVC;
		}

		rc_keydown(ir->rc, protocol, scancode, 0);
	} else if ((status & REMOTE_INT_REPEAT) && ir->repeat_valid) {
		if (time_before(jiffies, ir->repeat_deadline)) {
			rc_repeat(ir->rc);
			ir->repeat_deadline = jiffies + msecs_to_jiffies(REMOTE_REPEAT_TIMEOUT_MS);
		} else {
			ir->repeat_valid = false;
		}
	}

out_unlock:
	spin_unlock(&ir->lock);
	return ret;
}

static void novatek_ir_quiesce(void *data)
{
	struct novatek_ir *ir = data;
	ulong flags;

	spin_lock_irqsave(&ir->lock, flags);
	if (ir->running) {
		novatek_ir_stop(ir);
		ir->running = false;
	}
	spin_unlock_irqrestore(&ir->lock, flags);

	synchronize_irq(ir->irq);
}

static void novatek_ir_disable_clock(void *data)
{
	struct novatek_ir *ir = data;

	if (ir->clock_enabled) {
		clk_disable_unprepare(ir->clk);
		ir->clock_enabled = false;
	}
}

static int novatek_ir_set_timings(struct novatek_ir *ir)
{
	ulong rate = clk_get_rate(ir->clk);
	u64 header, repeat, logic, glitch, nec_error, jvc_error;

	header = div_u64((u64)rate * REMOTE_HEADER_US, USEC_PER_SEC);
	repeat = div_u64((u64)rate * REMOTE_REPEAT_US, USEC_PER_SEC);
	logic = div_u64((u64)rate * REMOTE_LOGIC_US, USEC_PER_SEC);
	glitch = div_u64((u64)rate * REMOTE_GLITCH_US, USEC_PER_SEC);
	nec_error = div_u64((u64)rate * REMOTE_NEC_ERROR_US, USEC_PER_SEC);
	jvc_error = div_u64((u64)rate * REMOTE_JVC_ERROR_US, USEC_PER_SEC);
	if (!glitch || header > FIELD_MAX(REMOTE_TH0_HEADER) ||
	    repeat > FIELD_MAX(REMOTE_TH0_REPEAT) || logic > FIELD_MAX(REMOTE_TH1_LOGIC) ||
	    glitch > FIELD_MAX(REMOTE_TH1_GLITCH) || nec_error > FIELD_MAX(REMOTE_TH2_ERROR))
		return -EINVAL;

	ir->threshold0 = FIELD_PREP(REMOTE_TH0_HEADER, header) |
			 FIELD_PREP(REMOTE_TH0_REPEAT, repeat);
	ir->threshold1 = FIELD_PREP(REMOTE_TH1_LOGIC, logic) |
			 FIELD_PREP(REMOTE_TH1_GLITCH, glitch);
	ir->nec_error = nec_error;
	ir->jvc_error = jvc_error;
	return 0;
}

static int novatek_ir_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct reset_control *reset;
	struct novatek_ir *ir;
	struct rc_dev *rc;
	ulong flags;
	int ret;

	ir = devm_kzalloc(dev, sizeof(*ir), GFP_KERNEL);
	if (!ir)
		return -ENOMEM;

	ir->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(ir->base))
		return PTR_ERR(ir->base);

	ir->clk = devm_clk_get(dev, NULL);
	if (IS_ERR(ir->clk))
		return dev_err_probe(dev, PTR_ERR(ir->clk), "Failed to get clock\n");

	reset = devm_reset_control_get_exclusive(dev, NULL);
	if (IS_ERR(reset))
		return dev_err_probe(dev, PTR_ERR(reset), "Failed to get reset\n");

	ir->irq = platform_get_irq(pdev, 0);
	if (ir->irq < 0)
		return ir->irq;

	rc = devm_rc_allocate_device(dev, RC_DRIVER_SCANCODE);
	if (!rc)
		return -ENOMEM;
	ir->rc = rc;

	spin_lock_init(&ir->lock);
	platform_set_drvdata(pdev, ir);

	ret = clk_prepare_enable(ir->clk);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to enable clock\n");
	ir->clock_enabled = true;
	ret = devm_add_action_or_reset(dev, novatek_ir_disable_clock, ir);
	if (ret)
		return ret;

	ret = reset_control_reset(reset);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to reset receiver\n");

	novatek_ir_stop(ir);

	ret = novatek_ir_set_timings(ir);
	if (ret)
		return dev_err_probe(dev, ret, "Unsupported receiver clock rate\n");

	rc->priv = ir;
	rc->device_name = "Novatek infrared receiver";
	rc->input_phys = "novatek-ir/input0";
	rc->input_id.bustype = BUS_HOST;
	rc->driver_name = "novatek-ir";
	rc->allowed_protocols = REMOTE_NEC_PROTOCOLS | RC_PROTO_BIT_JVC;
	rc->map_name = of_get_property(dev->of_node, "linux,rc-map-name", NULL);
	if (!rc->map_name)
		rc->map_name = RC_MAP_EMPTY;

	rc->change_protocol = novatek_ir_change_protocol;

	ret = devm_request_irq(dev, ir->irq, novatek_ir_irq, 0, dev_name(dev), ir);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to request interrupt\n");

	ret = devm_rc_register_device(dev, rc);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to register receiver\n");

	ret = devm_add_action_or_reset(dev, novatek_ir_quiesce, ir);
	if (ret)
		return ret;

	spin_lock_irqsave(&ir->lock, flags);
	ir->running = true;
	novatek_ir_configure(ir);
	spin_unlock_irqrestore(&ir->lock, flags);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int novatek_ir_suspend(struct device *dev)
{
	struct novatek_ir *ir = dev_get_drvdata(dev);

	novatek_ir_quiesce(ir);
	novatek_ir_disable_clock(ir);
	return 0;
}

static int novatek_ir_resume(struct device *dev)
{
	struct novatek_ir *ir = dev_get_drvdata(dev);
	ulong flags;
	int ret;

	ret = clk_prepare_enable(ir->clk);
	if (ret)
		return ret;
	ir->clock_enabled = true;

	ret = novatek_ir_set_timings(ir);
	if (ret) {
		novatek_ir_disable_clock(ir);
		return ret;
	}

	spin_lock_irqsave(&ir->lock, flags);
	ir->running = true;
	novatek_ir_configure(ir);
	spin_unlock_irqrestore(&ir->lock, flags);

	return 0;
}
#endif

static const struct dev_pm_ops novatek_ir_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(novatek_ir_suspend, novatek_ir_resume)
};

static const struct of_device_id novatek_ir_of_match[] = {
	{ .compatible = "novatek,na51089-ir" },
	{ }
};
MODULE_DEVICE_TABLE(of, novatek_ir_of_match);

static struct platform_driver novatek_ir_driver = {
	.probe = novatek_ir_probe,
	.driver = {
		.name = "novatek-ir",
		.of_match_table = novatek_ir_of_match,
		.pm = &novatek_ir_pm_ops,
	},
};
module_platform_driver(novatek_ir_driver);

MODULE_AUTHOR("Yu-Tung Chang <mtwget@gmail.com>");
MODULE_DESCRIPTION("Novatek infrared receiver driver");
MODULE_LICENSE("GPL");
