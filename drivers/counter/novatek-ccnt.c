// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2026 Yu-Tung Chang <mtwget@gmail.com>
 */

#include <linux/bitfield.h>
#include <linux/cleanup.h>
#include <linux/clk.h>
#include <linux/counter.h>
#include <linux/interrupt.h>
#include <linux/mfd/novatek-pwmss.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/spinlock.h>

#define CCNT_CONFIG_REG(ch)			(0x120 + (ch) * 0x20)
#define CCNT_START_REG(ch)			(0x124 + (ch) * 0x20)
#define CCNT_COMPARE_REG(ch)			(0x128 + (ch) * 0x20)
#define CCNT_TIMEOUT_REG(ch)			(0x12c + (ch) * 0x20)
#define CCNT_CURRENT_REG(ch)			(0x130 + (ch) * 0x20)
#define CCNT_ENABLE_REG				0x200

/* CCNT_CONFIG bit fields */
#define CCNT_CONFIG_FILTER			GENMASK(7, 0)
#define CCNT_CONFIG_FILTER_DEFAULT		0x00
#define CCNT_CONFIG_DECREASE			BIT(16)
#define CCNT_CONFIG_INVERT			BIT(17)
#define CCNT_CONFIG_EDGE			BIT(18)
#define CCNT_CONFIG_SOURCE			BIT(20)
#define CCNT_CONFIG_DEFAULT \
	(FIELD_PREP_CONST(CCNT_CONFIG_FILTER, CCNT_CONFIG_FILTER_DEFAULT) | \
	 FIELD_PREP_CONST(CCNT_CONFIG_DECREASE, 0) | \
	 FIELD_PREP_CONST(CCNT_CONFIG_INVERT, 0) | \
	 FIELD_PREP_CONST(CCNT_CONFIG_EDGE, 0) | \
	 FIELD_PREP_CONST(CCNT_CONFIG_SOURCE, 0))

/* CCNT_TIMEOUT bit fields */
#define CCNT_TIMEOUT_ENABLE			BIT(0)
#define CCNT_TIMEOUT_DISABLED \
	(FIELD_PREP_CONST(CCNT_TIMEOUT_ENABLE, 0))

/* CCNT_CURRENT bit fields */
#define CCNT_CURRENT_VALUE			GENMASK(15, 0)

/* CCNT_ENABLE bit fields */
#define CCNT_ENABLE_CHANNEL(ch)			BIT(ch)

#define CCNT_SYNC_TIMEOUT_US			2000

struct novatek_ccnt;

struct novatek_ccnt_soc_data {
	uint num_channels;
};

struct novatek_ccnt_channel {
	struct novatek_ccnt *priv;
	uint index;
	int irq;
	u32 config;
	u16 value;
	u16 compare;
	bool enabled;
	bool value_valid;
};

struct novatek_ccnt {
	struct novatek_pwmss *core;
	const struct novatek_ccnt_soc_data *data;
	struct counter_device *counter;
	/* IRQ masking sleeps and must be serialized separately from reads. */
	struct mutex lock;
	/* Counter event snapshots invoke reads with interrupts disabled. */
	spinlock_t state_lock;
	struct novatek_ccnt_channel *channels;
	struct counter_signal *signals;
	struct counter_synapse *synapses;
	struct counter_count *counts;
	uint resume_mask;
	bool suspended;
	bool removing;
	bool clocks_enabled;
};

static int novatek_ccnt_stop(struct novatek_ccnt_channel *channel, bool save_count);

static void novatek_ccnt_disable_irq(struct novatek_ccnt_channel *channel)
{
	disable_irq(channel->irq);
	synchronize_irq(channel->priv->core->irq);
}

static int novatek_ccnt_start(struct novatek_ccnt_channel *channel)
{
	struct novatek_ccnt *priv = channel->priv;
	struct regmap *regmap = priv->core->regmap;
	uint index = channel->index;
	u32 mask = CCNT_ENABLE_CHANNEL(index);
	ulong flags;
	u32 value;
	int ret;
	int stop_ret;

	if (!channel->value_valid)
		return -EIO;

	ret = regmap_write(regmap, CCNT_CONFIG_REG(index), channel->config);
	if (ret)
		return ret;

	ret = regmap_write(regmap, CCNT_START_REG(index), channel->value);
	if (ret)
		return ret;

	ret = regmap_write(regmap, CCNT_COMPARE_REG(index), channel->compare);
	if (ret)
		return ret;

	ret = regmap_write(regmap, PWM_CCNT_STATUS_REG, PWM_CCNT_STATUS_THRESHOLD(index));
	if (ret)
		return ret;

	enable_irq(channel->irq);

	spin_lock_irqsave(&priv->state_lock, flags);
	channel->enabled = true;
	ret = regmap_set_bits(regmap, CCNT_ENABLE_REG, mask);
	if (!ret)
		ret = regmap_read_poll_timeout_atomic(regmap, CCNT_ENABLE_REG,
						      value, (value & mask) == mask,
						      1, CCNT_SYNC_TIMEOUT_US);
	spin_unlock_irqrestore(&priv->state_lock, flags);

	if (ret) {
		stop_ret = novatek_ccnt_stop(channel, false);
		if (stop_ret)
			dev_err(priv->counter->parent,
				"Failed to stop counter %u: %d\n", index, stop_ret);
	}
	return ret;
}

static int novatek_ccnt_stop(struct novatek_ccnt_channel *channel, bool save_count)
{
	struct novatek_ccnt *priv = channel->priv;
	struct regmap *regmap = priv->core->regmap;
	u32 mask = CCNT_ENABLE_CHANNEL(channel->index);
	bool stopped = false;
	ulong flags;
	u32 value;
	int ret;

	spin_lock_irqsave(&priv->state_lock, flags);
	ret = regmap_clear_bits(regmap, CCNT_ENABLE_REG, mask);
	if (ret)
		goto out_unlock;
	ret = regmap_read_poll_timeout_atomic(regmap, CCNT_ENABLE_REG,
					      value, !(value & mask),
					      1, CCNT_SYNC_TIMEOUT_US);
	if (ret)
		goto out_unlock;
	channel->enabled = false;
	stopped = true;
	if (!save_count)
		goto out_unlock;

	ret = regmap_read(regmap, CCNT_CURRENT_REG(channel->index), &value);
	channel->value_valid = !ret;
	if (!ret)
		channel->value = FIELD_GET(CCNT_CURRENT_VALUE, value);

out_unlock:
	spin_unlock_irqrestore(&priv->state_lock, flags);

	if (stopped)
		novatek_ccnt_disable_irq(channel);
	return ret;
}

static int novatek_ccnt_count_read(struct counter_device *counter,
				   struct counter_count *count, u64 *value)
{
	struct novatek_ccnt *priv = counter_priv(counter);
	struct novatek_ccnt_channel *channel = &priv->channels[count->id];
	u32 count_val;
	int ret;

	guard(spinlock_irqsave)(&priv->state_lock);
	if (priv->removing)
		return -ENODEV;
	if (!channel->enabled) {
		if (!channel->value_valid)
			return -EIO;
		*value = channel->value;
		return 0;
	}

	ret = regmap_read(priv->core->regmap, CCNT_CURRENT_REG(count->id), &count_val);
	if (ret)
		return ret;
	*value = FIELD_GET(CCNT_CURRENT_VALUE, count_val);
	return 0;
}

static int novatek_ccnt_count_write(struct counter_device *counter,
				    struct counter_count *count, u64 value)
{
	struct novatek_ccnt *priv = counter_priv(counter);
	struct novatek_ccnt_channel *channel = &priv->channels[count->id];

	if (value > U16_MAX)
		return -ERANGE;

	guard(mutex)(&priv->lock);
	guard(spinlock_irqsave)(&priv->state_lock);
	if (priv->suspended || priv->removing || channel->enabled)
		return -EBUSY;
	channel->value = value;
	channel->value_valid = true;
	return 0;
}

static int novatek_ccnt_function_read(struct counter_device *counter, struct counter_count *count,
				      enum counter_function *function)
{
	struct novatek_ccnt *priv = counter_priv(counter);

	guard(spinlock_irqsave)(&priv->state_lock);
	if (priv->channels[count->id].config & CCNT_CONFIG_DECREASE)
		*function = COUNTER_FUNCTION_DECREASE;
	else
		*function = COUNTER_FUNCTION_INCREASE;
	return 0;
}

static int novatek_ccnt_function_write(struct counter_device *counter, struct counter_count *count,
				       enum counter_function function)
{
	struct novatek_ccnt *priv = counter_priv(counter);
	struct novatek_ccnt_channel *channel = &priv->channels[count->id];
	u32 value;

	switch (function) {
	case COUNTER_FUNCTION_INCREASE:
		value = 0;
		break;
	case COUNTER_FUNCTION_DECREASE:
		value = CCNT_CONFIG_DECREASE;
		break;
	default:
		return -EINVAL;
	}

	guard(mutex)(&priv->lock);
	guard(spinlock_irqsave)(&priv->state_lock);
	if (priv->suspended || priv->removing || channel->enabled)
		return -EBUSY;
	channel->config &= ~CCNT_CONFIG_DECREASE;
	channel->config |= value;
	return 0;
}

static int novatek_ccnt_action_read(struct counter_device *counter, struct counter_count *count,
				    struct counter_synapse *synapse,
				    enum counter_synapse_action *action)
{
	struct novatek_ccnt *priv = counter_priv(counter);
	u32 config;

	guard(spinlock_irqsave)(&priv->state_lock);
	config = priv->channels[count->id].config;
	if (config & CCNT_CONFIG_EDGE)
		*action = COUNTER_SYNAPSE_ACTION_BOTH_EDGES;
	else if (config & CCNT_CONFIG_INVERT)
		*action = COUNTER_SYNAPSE_ACTION_FALLING_EDGE;
	else
		*action = COUNTER_SYNAPSE_ACTION_RISING_EDGE;
	return 0;
}

static int novatek_ccnt_action_write(struct counter_device *counter, struct counter_count *count,
				     struct counter_synapse *synapse,
				     enum counter_synapse_action action)
{
	struct novatek_ccnt *priv = counter_priv(counter);
	struct novatek_ccnt_channel *channel = &priv->channels[count->id];
	u32 invert = 0;
	u32 edge = 0;

	switch (action) {
	case COUNTER_SYNAPSE_ACTION_RISING_EDGE:
		break;
	case COUNTER_SYNAPSE_ACTION_FALLING_EDGE:
		invert = CCNT_CONFIG_INVERT;
		break;
	case COUNTER_SYNAPSE_ACTION_BOTH_EDGES:
		edge = CCNT_CONFIG_EDGE;
		break;
	default:
		return -EINVAL;
	}

	guard(mutex)(&priv->lock);
	guard(spinlock_irqsave)(&priv->state_lock);
	if (priv->suspended || priv->removing || channel->enabled)
		return -EBUSY;
	channel->config &= ~(CCNT_CONFIG_INVERT | CCNT_CONFIG_EDGE);
	channel->config |= invert | edge;
	return 0;
}

static int novatek_ccnt_enable_read(struct counter_device *counter,
				    struct counter_count *count, u8 *enable)
{
	struct novatek_ccnt *priv = counter_priv(counter);

	guard(spinlock_irqsave)(&priv->state_lock);
	*enable = priv->channels[count->id].enabled;
	return 0;
}

static int novatek_ccnt_enable_write(struct counter_device *counter,
				     struct counter_count *count, u8 enable)
{
	struct novatek_ccnt *priv = counter_priv(counter);
	struct novatek_ccnt_channel *channel = &priv->channels[count->id];

	guard(mutex)(&priv->lock);
	if (priv->suspended || priv->removing)
		return -EBUSY;
	if (channel->enabled == enable)
		return 0;

	if (enable)
		return novatek_ccnt_start(channel);
	return novatek_ccnt_stop(channel, true);
}

static int novatek_ccnt_compare_read(struct counter_device *counter,
				     struct counter_count *count, u64 *value)
{
	struct novatek_ccnt *priv = counter_priv(counter);

	guard(spinlock_irqsave)(&priv->state_lock);
	*value = priv->channels[count->id].compare;
	return 0;
}

static int novatek_ccnt_compare_write(struct counter_device *counter,
				      struct counter_count *count, u64 value)
{
	struct novatek_ccnt *priv = counter_priv(counter);
	struct novatek_ccnt_channel *channel = &priv->channels[count->id];

	if (value > U16_MAX)
		return -ERANGE;

	guard(mutex)(&priv->lock);
	guard(spinlock_irqsave)(&priv->state_lock);
	if (priv->suspended || priv->removing || channel->enabled)
		return -EBUSY;
	channel->compare = value;
	return 0;
}

static int novatek_ccnt_floor_read(struct counter_device *counter,
				   struct counter_count *count, u64 *value)
{
	*value = 0;
	return 0;
}

static int novatek_ccnt_ceiling_read(struct counter_device *counter,
				     struct counter_count *count, u64 *value)
{
	*value = U16_MAX;
	return 0;
}

static int novatek_ccnt_watch_validate(struct counter_device *counter,
				       const struct counter_watch *watch)
{
	struct novatek_ccnt *priv = counter_priv(counter);

	if (watch->event != COUNTER_EVENT_THRESHOLD || watch->channel >= priv->data->num_channels)
		return -EINVAL;
	return 0;
}

static irqreturn_t novatek_ccnt_irq(int irq, void *data)
{
	struct novatek_ccnt_channel *channel = data;

	counter_push_event(channel->priv->counter, COUNTER_EVENT_THRESHOLD, channel->index);
	return IRQ_HANDLED;
}

static const enum counter_function novatek_ccnt_functions[] = {
	COUNTER_FUNCTION_INCREASE,
	COUNTER_FUNCTION_DECREASE,
};

static const enum counter_synapse_action novatek_ccnt_actions[] = {
	COUNTER_SYNAPSE_ACTION_RISING_EDGE,
	COUNTER_SYNAPSE_ACTION_FALLING_EDGE,
	COUNTER_SYNAPSE_ACTION_BOTH_EDGES,
};

static struct counter_comp novatek_ccnt_ext[] = {
	COUNTER_COMP_ENABLE(novatek_ccnt_enable_read, novatek_ccnt_enable_write),
	COUNTER_COMP_COMPARE(novatek_ccnt_compare_read, novatek_ccnt_compare_write),
	COUNTER_COMP_FLOOR(novatek_ccnt_floor_read, NULL),
	COUNTER_COMP_CEILING(novatek_ccnt_ceiling_read, NULL),
};

static const struct counter_ops novatek_ccnt_ops = {
	.count_read = novatek_ccnt_count_read,
	.count_write = novatek_ccnt_count_write,
	.function_read = novatek_ccnt_function_read,
	.function_write = novatek_ccnt_function_write,
	.action_read = novatek_ccnt_action_read,
	.action_write = novatek_ccnt_action_write,
	.watch_validate = novatek_ccnt_watch_validate,
};

static int novatek_ccnt_enable_clocks(struct novatek_ccnt *priv)
{
	uint i;
	int ret;

	if (priv->clocks_enabled)
		return 0;

	for (i = 0; i < priv->data->num_channels; i++) {
		ret = clk_prepare_enable(priv->core->ccnt_clks[i]);
		if (ret) {
			while (i--)
				clk_disable_unprepare(priv->core->ccnt_clks[i]);
			return ret;
		}
	}
	priv->clocks_enabled = true;
	return 0;
}

static void novatek_ccnt_disable_clocks(struct novatek_ccnt *priv)
{
	uint i = priv->data->num_channels;

	if (!priv->clocks_enabled)
		return;

	while (i--)
		clk_disable_unprepare(priv->core->ccnt_clks[i]);
	priv->clocks_enabled = false;
}

static void novatek_ccnt_quiesce(void *data)
{
	struct novatek_ccnt *priv = data;
	ulong flags;
	uint i;
	int ret;

	guard(mutex)(&priv->lock);
	spin_lock_irqsave(&priv->state_lock, flags);
	priv->removing = true;
	spin_unlock_irqrestore(&priv->state_lock, flags);
	if (!priv->clocks_enabled)
		return;

	for (i = 0; i < priv->data->num_channels; i++) {
		if (!priv->channels[i].enabled)
			continue;
		ret = novatek_ccnt_stop(&priv->channels[i], true);
		if (ret) {
			dev_err(priv->counter->parent, "Failed to stop counter %u: %d\n", i, ret);
			if (priv->channels[i].enabled)
				novatek_ccnt_disable_irq(&priv->channels[i]);
		}
	}

	spin_lock_irqsave(&priv->state_lock, flags);
	for (i = 0; i < priv->data->num_channels; i++)
		priv->channels[i].enabled = false;
	spin_unlock_irqrestore(&priv->state_lock, flags);

	novatek_ccnt_disable_clocks(priv);
}

static int novatek_ccnt_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct novatek_ccnt_soc_data *data;
	struct counter_device *counter;
	struct novatek_ccnt *priv;
	u32 channel_mask;
	u32 value;
	uint i;
	int ret;

	data = device_get_match_data(dev);
	if (!data)
		return -ENODEV;

	counter = devm_counter_alloc(dev, sizeof(*priv));
	if (!counter)
		return -ENOMEM;
	priv = counter_priv(counter);
	priv->core = dev_get_drvdata(dev->parent);
	priv->data = data;
	priv->counter = counter;

	spin_lock_init(&priv->state_lock);

	ret = devm_mutex_init(dev, &priv->lock);
	if (ret)
		return ret;

	priv->channels = devm_kcalloc(dev, data->num_channels, sizeof(*priv->channels), GFP_KERNEL);
	if (!priv->channels)
		return -ENOMEM;

	priv->signals = devm_kcalloc(dev, data->num_channels, sizeof(*priv->signals), GFP_KERNEL);
	if (!priv->signals)
		return -ENOMEM;

	priv->synapses = devm_kcalloc(dev, data->num_channels, sizeof(*priv->synapses), GFP_KERNEL);
	if (!priv->synapses)
		return -ENOMEM;

	priv->counts = devm_kcalloc(dev, data->num_channels, sizeof(*priv->counts), GFP_KERNEL);
	if (!priv->counts)
		return -ENOMEM;

	counter->name = "Novatek cycle counter";
	counter->parent = dev;
	counter->ops = &novatek_ccnt_ops;
	counter->counts = priv->counts;
	counter->num_counts = data->num_channels;
	counter->signals = priv->signals;
	counter->num_signals = data->num_channels;

	platform_set_drvdata(pdev, counter);

	ret = novatek_ccnt_enable_clocks(priv);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to enable clocks\n");

	channel_mask = GENMASK(data->num_channels - 1, 0);
	ret = regmap_clear_bits(priv->core->regmap, CCNT_ENABLE_REG, channel_mask);
	if (ret)
		goto out_disable_clocks;
	ret = regmap_read_poll_timeout(priv->core->regmap, CCNT_ENABLE_REG,
				       value, !(value & channel_mask),
				       1, CCNT_SYNC_TIMEOUT_US);
	if (ret)
		goto out_disable_clocks;

	for (i = 0; i < data->num_channels; i++) {
		struct novatek_ccnt_channel *channel = &priv->channels[i];
		struct counter_count *count = &priv->counts[i];
		struct counter_synapse *synapse = &priv->synapses[i];

		ret = regmap_write(priv->core->regmap, CCNT_TIMEOUT_REG(i), CCNT_TIMEOUT_DISABLED);
		if (ret)
			goto out_disable_clocks;
		ret = regmap_read_poll_timeout(priv->core->regmap, CCNT_TIMEOUT_REG(i),
					       value, !(value & CCNT_TIMEOUT_ENABLE),
					       1, CCNT_SYNC_TIMEOUT_US);
		if (ret)
			goto out_disable_clocks;

		channel->priv = priv;
		channel->index = i;
		channel->config = CCNT_CONFIG_DEFAULT;
		channel->compare = U16_MAX;
		channel->value_valid = true;

		channel->irq = platform_get_irq(pdev, i);
		if (channel->irq < 0) {
			ret = channel->irq;
			goto out_disable_clocks;
		}

		ret = devm_request_threaded_irq(dev, channel->irq,
						NULL, novatek_ccnt_irq,
						IRQF_ONESHOT | IRQF_NO_AUTOEN,
						dev_name(dev), channel);
		if (ret)
			goto out_disable_clocks;

		priv->signals[i].name = devm_kasprintf(dev, GFP_KERNEL, "PI_CNT%u", i);
		if (!priv->signals[i].name) {
			ret = -ENOMEM;
			goto out_disable_clocks;
		}
		priv->signals[i].id = i;

		synapse->actions_list = novatek_ccnt_actions;
		synapse->num_actions = ARRAY_SIZE(novatek_ccnt_actions);
		synapse->signal = &priv->signals[i];

		count->name = devm_kasprintf(dev, GFP_KERNEL, "CCNT%u", i);
		if (!count->name) {
			ret = -ENOMEM;
			goto out_disable_clocks;
		}

		count->id = i;
		count->functions_list = novatek_ccnt_functions;
		count->num_functions = ARRAY_SIZE(novatek_ccnt_functions);
		count->synapses = synapse;
		count->num_synapses = 1;
		count->ext = novatek_ccnt_ext;
		count->num_ext = ARRAY_SIZE(novatek_ccnt_ext);
	}

	ret = devm_counter_add(dev, counter);
	if (ret) {
		novatek_ccnt_quiesce(priv);
		return ret;
	}

	return devm_add_action_or_reset(dev, novatek_ccnt_quiesce, priv);

out_disable_clocks:
	novatek_ccnt_disable_clocks(priv);
	return ret;
}

static void novatek_ccnt_shutdown(struct platform_device *pdev)
{
	struct counter_device *counter = platform_get_drvdata(pdev);
	struct novatek_ccnt *priv = counter_priv(counter);

	novatek_ccnt_quiesce(priv);
}

#ifdef CONFIG_PM_SLEEP
static int novatek_ccnt_suspend(struct device *dev)
{
	struct counter_device *counter = dev_get_drvdata(dev);
	struct novatek_ccnt *priv = counter_priv(counter);
	uint resume_mask = 0;
	uint i;
	int ret;
	int start_ret;

	guard(mutex)(&priv->lock);
	if (priv->suspended)
		return 0;

	for (i = 0; i < priv->data->num_channels; i++) {
		if (!priv->channels[i].enabled)
			continue;
		resume_mask |= CCNT_ENABLE_CHANNEL(i);
		ret = novatek_ccnt_stop(&priv->channels[i], true);
		if (ret) {
			for (i = 0; i < priv->data->num_channels; i++) {
				struct novatek_ccnt_channel *channel;
				uint mask = CCNT_ENABLE_CHANNEL(i);

				channel = &priv->channels[i];
				if (!(resume_mask & mask))
					continue;
				if (channel->enabled)
					continue;

				start_ret = novatek_ccnt_start(channel);
				if (start_ret)
					dev_err(dev, "Cannot restart CCNT%u: %d\n", i, start_ret);
			}
			return ret;
		}
	}

	priv->resume_mask = resume_mask;
	priv->suspended = true;

	novatek_ccnt_disable_clocks(priv);
	return 0;
}

static int novatek_ccnt_resume(struct device *dev)
{
	struct counter_device *counter = dev_get_drvdata(dev);
	struct novatek_ccnt *priv = counter_priv(counter);
	uint i;
	int ret;

	guard(mutex)(&priv->lock);
	if (!priv->suspended)
		return 0;

	ret = novatek_ccnt_enable_clocks(priv);
	if (ret)
		return ret;
	priv->suspended = false;

	for (i = 0; i < priv->data->num_channels; i++) {
		if (!(priv->resume_mask & CCNT_ENABLE_CHANNEL(i)))
			continue;

		ret = novatek_ccnt_start(&priv->channels[i]);
		if (ret)
			return ret;
	}
	return 0;
}
#endif

static const struct dev_pm_ops novatek_ccnt_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(novatek_ccnt_suspend, novatek_ccnt_resume)
};

static const struct novatek_ccnt_soc_data na51089_ccnt_data = {
	.num_channels = 3,
};

static const struct of_device_id novatek_ccnt_of_match[] = {
	{
		.compatible = "novatek,na51089-ccnt",
		.data = &na51089_ccnt_data,
	},
	{ }
};
MODULE_DEVICE_TABLE(of, novatek_ccnt_of_match);

static struct platform_driver novatek_ccnt_driver = {
	.probe = novatek_ccnt_probe,
	.shutdown = novatek_ccnt_shutdown,
	.driver = {
		.name = "novatek-ccnt",
		.of_match_table = novatek_ccnt_of_match,
		.pm = &novatek_ccnt_pm_ops,
	},
};
module_platform_driver(novatek_ccnt_driver);

MODULE_IMPORT_NS("COUNTER");
MODULE_AUTHOR("Yu-Tung Chang <mtwget@gmail.com>");
MODULE_DESCRIPTION("Novatek cycle counter");
MODULE_LICENSE("GPL");
