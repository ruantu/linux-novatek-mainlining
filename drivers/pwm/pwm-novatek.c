// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2026 Yu-Tung Chang <mtwget@gmail.com>
 */

/*
 * Period and duty updates are latched at a period boundary. Polarity and clock
 * dividers cannot be changed while the channel is running, so changing either
 * requires a stop and restart and can introduce a gap. Channels 0-3 and 4-7
 * share dividers; an active sibling restricts the available period range.
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/math64.h>
#include <linux/mfd/novatek-pwmss.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/pwm.h>
#include <linux/regmap.h>

#define PWM_CTRL_REG(ch)			(0x000 + (ch) * 8)
#define PWM_PERIOD_REG(ch)			(0x004 + (ch) * 8)
#define PWM_ENABLE_REG				0x100
#define PWM_DISABLE_REG				0x104
#define PWM_LOAD_REG				0x108
#define PWM_PERIOD_EXT_REG(ch)			(0x230 + (ch) * 4)

/* PWM_CTRL bit fields */
#define PWM_CTRL_CYCLE				GENMASK(15, 0)
#define PWM_CTRL_CYCLE_FREE_RUN			0x0000
#define PWM_CTRL_TYPE				BIT(16)
#define PWM_CTRL_DEFAULT \
	(FIELD_PREP_CONST(PWM_CTRL_CYCLE, PWM_CTRL_CYCLE_FREE_RUN) | \
	 FIELD_PREP_CONST(PWM_CTRL_TYPE, 0))

/* PWM_PERIOD bit fields */
#define PWM_PERIOD_FALL				GENMASK(15, 8)
#define PWM_PERIOD_PERIOD			GENMASK(23, 16)
#define PWM_PERIOD_INVERT			BIT(28)

/* PWM_ENABLE bit fields */
#define PWM_ENABLE_CHANNEL(ch)			BIT(ch)
#define PWM_ENABLE_GROUP(index, size)		(GENMASK((size) - 1, 0) << (index))

/* PWM_DISABLE bit fields */
#define PWM_DISABLE_CHANNEL(ch)			BIT(ch)

/* PWM_LOAD bit fields */
#define PWM_LOAD_CHANNEL(ch)			BIT(ch)

#define PWM_SYNC_TIMEOUT_US			2000

#define PWM_MAX_RATE				30000000

struct novatek_pwm_soc_data {
	uint npwm;
	uint num_ext_channels;
	uint group_size;
};

struct novatek_pwm {
	const struct novatek_pwm_soc_data *data;
	struct regmap *regmap;
	struct clk_bulk_data *clks;
	u32 enabled_clks;
};

struct novatek_pwm_config {
	struct clk *clk;
	ulong rate;
	u32 period;
	u32 period_ext;
};

static int novatek_pwm_get_state(struct pwm_chip *chip, struct pwm_device *pwm,
				 struct pwm_state *state)
{
	struct novatek_pwm *priv = pwmchip_get_drvdata(chip);
	uint ch = pwm->hwpwm;
	ulong rate = clk_get_rate(priv->clks[ch].clk);
	u32 val, period, duty;
	int ret;

	if (!rate)
		return -EINVAL;

	ret = regmap_read(priv->regmap, PWM_ENABLE_REG, &val);
	if (ret)
		return ret;
	state->enabled = val & PWM_ENABLE_CHANNEL(ch);

	ret = regmap_read(priv->regmap, PWM_PERIOD_REG(ch), &val);
	if (ret)
		return ret;
	state->polarity = val & PWM_PERIOD_INVERT ? PWM_POLARITY_INVERSED : PWM_POLARITY_NORMAL;
	period = FIELD_GET(PWM_PERIOD_PERIOD, val);
	duty = FIELD_GET(PWM_PERIOD_FALL, val);
	if (ch < priv->data->num_ext_channels) {
		ret = regmap_read(priv->regmap, PWM_PERIOD_EXT_REG(ch), &val);
		if (ret)
			return ret;
		period |= FIELD_GET(PWM_PERIOD_PERIOD, val) << 8;
		duty |= FIELD_GET(PWM_PERIOD_FALL, val) << 8;
	}

	state->period = DIV64_U64_ROUND_UP((u64)period * NSEC_PER_SEC, rate);
	state->duty_cycle = DIV64_U64_ROUND_UP((u64)duty * NSEC_PER_SEC, rate);

	return 0;
}

static int novatek_pwm_wait_for_load(struct novatek_pwm *priv, uint ch,
				     const struct pwm_state *cstate)
{
	u32 mask = PWM_LOAD_CHANNEL(ch), val;
	u64 timeout;
	int ret;

	timeout = DIV_ROUND_UP_ULL(cstate->period, NSEC_PER_USEC) + PWM_SYNC_TIMEOUT_US;
	ret = regmap_write(priv->regmap, PWM_LOAD_REG, mask);
	if (ret)
		return ret;
	return regmap_read_poll_timeout(priv->regmap, PWM_LOAD_REG,
					val, !(val & mask),
					10, timeout);
}

static int novatek_pwm_stop(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct novatek_pwm *priv = pwmchip_get_drvdata(chip);
	u32 mask = PWM_DISABLE_CHANNEL(pwm->hwpwm);
	struct pwm_state state;
	ulong rate;
	u64 timeout;
	u32 max_period, val;
	int ret;

	ret = novatek_pwm_get_state(chip, pwm, &state);
	if (ret)
		return ret;
	if (!state.enabled)
		return 0;

	/* With LOAD pending, PERIOD can differ from the running waveform. */
	ret = regmap_test_bits(priv->regmap, PWM_LOAD_REG, PWM_LOAD_CHANNEL(pwm->hwpwm));
	if (ret < 0)
		return ret;
	if (ret) {
		max_period = pwm->hwpwm < priv->data->num_ext_channels ? U16_MAX : U8_MAX;
		rate = clk_get_rate(priv->clks[pwm->hwpwm].clk);
		state.period = DIV64_U64_ROUND_UP((u64)max_period * NSEC_PER_SEC, rate);
	}

	timeout = DIV_ROUND_UP_ULL(state.period, NSEC_PER_USEC) + PWM_SYNC_TIMEOUT_US;
	ret = regmap_write(priv->regmap, PWM_DISABLE_REG, mask);
	if (ret)
		return ret;
	return regmap_read_poll_timeout(priv->regmap, PWM_ENABLE_REG,
					val, !(val & mask),
					10, timeout);
}

static int novatek_pwm_start(struct novatek_pwm *priv, uint ch)
{
	u32 mask = PWM_ENABLE_CHANNEL(ch);
	u32 val;
	int ret;

	val = PWM_CTRL_DEFAULT;
	ret = regmap_write(priv->regmap, PWM_CTRL_REG(ch), val);
	if (ret)
		return ret;

	ret = regmap_write(priv->regmap, PWM_ENABLE_REG, mask);
	if (ret)
		return ret;
	return regmap_read_poll_timeout(priv->regmap, PWM_ENABLE_REG,
					val, val & mask,
					10, PWM_SYNC_TIMEOUT_US);
}

static int novatek_pwm_calc_config(struct pwm_chip *chip, struct pwm_device *pwm,
				   const struct pwm_state *state, struct novatek_pwm_config *config)
{
	struct novatek_pwm *priv = pwmchip_get_drvdata(chip);
	uint ch = pwm->hwpwm, clk_index = ch;
	u32 mask = PWM_ENABLE_CHANNEL(ch), group_mask = mask;
	u32 max_period = U8_MAX, enabled, period, duty;
	ulong rate;
	struct clk *clk;
	u64 target, limit;
	long rounded;
	int ret;

	if (ch < priv->data->num_ext_channels) {
		clk_index = round_down(ch, priv->data->group_size);
		group_mask = PWM_ENABLE_GROUP(clk_index, priv->data->group_size);
		max_period = U16_MAX;
	}

	clk = priv->clks[clk_index].clk;
	rate = clk_get_rate(clk);

	ret = regmap_read(priv->regmap, PWM_ENABLE_REG, &enabled);
	if (ret)
		return ret;

	limit = DIV64_U64_ROUND_UP(((u64)max_period + 1) * NSEC_PER_SEC, rate);
	if (state->period < DIV64_U64_ROUND_UP(2ULL * NSEC_PER_SEC, rate) ||
	    state->period >= limit) {
		if (enabled & group_mask & ~mask)
			return -ERANGE;

		target = div64_u64((u64)max_period * NSEC_PER_SEC, state->period);
		if (!target)
			return -ERANGE;
		target = min_t(u64, target, PWM_MAX_RATE);
		rounded = clk_round_rate(clk, target);
		if (rounded < 0)
			return rounded;
		if (!rounded)
			return -EINVAL;
		if (rounded > PWM_MAX_RATE)
			return -ERANGE;
		rate = rounded;
	}

	limit = DIV64_U64_ROUND_UP(((u64)max_period + 1) * NSEC_PER_SEC, rate);
	if (state->period >= limit)
		return -ERANGE;
	period = mul_u64_u64_div_u64(state->period, rate, NSEC_PER_SEC);
	if (period < 2)
		return -ERANGE;

	duty = mul_u64_u64_div_u64(state->duty_cycle, rate, NSEC_PER_SEC);
	duty = min(duty, period);

	config->clk = clk;
	config->rate = rate;
	config->period = FIELD_PREP(PWM_PERIOD_FALL, duty & U8_MAX) |
			 FIELD_PREP(PWM_PERIOD_PERIOD, period & U8_MAX) |
			 (state->polarity == PWM_POLARITY_INVERSED ? PWM_PERIOD_INVERT : 0);
	config->period_ext = FIELD_PREP(PWM_PERIOD_FALL, duty >> 8) |
			     FIELD_PREP(PWM_PERIOD_PERIOD, period >> 8);

	return 0;
}

static int novatek_pwm_apply_config(struct novatek_pwm *priv, uint ch,
				    const struct novatek_pwm_config *config)
{
	int ret;

	if (config->rate != clk_get_rate(config->clk)) {
		ret = clk_set_rate(config->clk, config->rate);
		if (ret)
			return ret;
	}

	if (ch < priv->data->num_ext_channels) {
		ret = regmap_write(priv->regmap, PWM_PERIOD_EXT_REG(ch), config->period_ext);
		if (ret)
			return ret;
	}

	return regmap_write(priv->regmap, PWM_PERIOD_REG(ch), config->period);
}

static int novatek_pwm_apply(struct pwm_chip *chip, struct pwm_device *pwm,
			     const struct pwm_state *state)
{
	struct novatek_pwm *priv = pwmchip_get_drvdata(chip);
	struct novatek_pwm_config config;
	struct pwm_state cstate;
	int ret;

	if (!state->enabled)
		return novatek_pwm_stop(chip, pwm);

	ret = regmap_test_bits(priv->regmap, PWM_LOAD_REG, PWM_LOAD_CHANNEL(pwm->hwpwm));
	if (ret < 0)
		return ret;
	if (ret)
		return -EBUSY;

	ret = novatek_pwm_calc_config(chip, pwm, state, &config);
	if (ret)
		return ret;

	ret = novatek_pwm_get_state(chip, pwm, &cstate);
	if (ret)
		return ret;

	if (cstate.enabled && state->polarity == cstate.polarity &&
	    config.rate == clk_get_rate(config.clk)) {
		ret = novatek_pwm_apply_config(priv, pwm->hwpwm, &config);
		if (ret)
			return ret;

		return novatek_pwm_wait_for_load(priv, pwm->hwpwm, &cstate);
	}

	ret = novatek_pwm_stop(chip, pwm);
	if (ret)
		return ret;

	ret = novatek_pwm_apply_config(priv, pwm->hwpwm, &config);
	if (ret)
		return ret;

	return novatek_pwm_start(priv, pwm->hwpwm);
}

static const struct pwm_ops novatek_pwm_ops = {
	.apply = novatek_pwm_apply,
	.get_state = novatek_pwm_get_state,
};

static void novatek_pwm_disable(void *data)
{
	struct pwm_chip *chip = data;
	struct novatek_pwm *priv = pwmchip_get_drvdata(chip);
	uint i;
	int ret;

	for (i = 0; i < chip->npwm; i++) {
		if (!(priv->enabled_clks & PWM_ENABLE_CHANNEL(i)))
			continue;
		ret = novatek_pwm_stop(chip, &chip->pwms[i]);
		if (ret)
			dev_err(pwmchip_parent(chip),
				"PWM%u stop failed before clock disable: %d\n", i, ret);

		clk_disable_unprepare(priv->clks[i].clk);
		priv->enabled_clks &= ~PWM_ENABLE_CHANNEL(i);
	}
}

static int novatek_pwm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct novatek_pwmss *core = dev_get_drvdata(dev->parent);
	const struct novatek_pwm_soc_data *data;
	struct novatek_pwm *priv;
	struct pwm_chip *chip;
	uint i;
	int ret;

	data = device_get_match_data(dev);
	if (!data)
		return -ENODEV;
	if (data->npwm > core->num_pwm)
		return -EINVAL;

	chip = devm_pwmchip_alloc(dev, data->npwm, sizeof(*priv));
	if (IS_ERR(chip))
		return PTR_ERR(chip);
	priv = pwmchip_get_drvdata(chip);
	chip->ops = &novatek_pwm_ops;

	priv->data = data;
	priv->regmap = core->regmap;
	priv->clks = core->pwm_clks;

	ret = clk_bulk_prepare_enable(chip->npwm, priv->clks);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to enable clocks\n");
	priv->enabled_clks = GENMASK(chip->npwm - 1, 0);

	for (i = 0; i < chip->npwm; i++) {
		if (!clk_get_rate(priv->clks[i].clk)) {
			ret = dev_err_probe(dev, -EINVAL, "Invalid PWM clock\n");
			goto out_disable_clocks;
		}

		if (i < priv->data->num_ext_channels && i % priv->data->group_size)
			continue;
		ret = devm_clk_rate_exclusive_get(dev, priv->clks[i].clk);
		if (ret) {
			dev_err_probe(dev, ret, "Failed to lock clock\n");
			goto out_disable_clocks;
		}
	}

	ret = devm_add_action_or_reset(dev, novatek_pwm_disable, chip);
	if (ret)
		return ret;

	for (i = 0; i < chip->npwm; i++) {
		if (i < priv->data->num_ext_channels && i % priv->data->group_size)
			continue;
		if (clk_get_rate(priv->clks[i].clk) <= PWM_MAX_RATE)
			continue;
		ret = clk_set_rate(priv->clks[i].clk, PWM_MAX_RATE);
		if (ret)
			return dev_err_probe(dev, ret, "Failed to set clock\n");
		if (!clk_get_rate(priv->clks[i].clk) ||
		    clk_get_rate(priv->clks[i].clk) > PWM_MAX_RATE)
			return dev_err_probe(dev, -EINVAL, "Invalid PWM clock rate\n");
	}

	platform_set_drvdata(pdev, chip);

	return devm_pwmchip_add(dev, chip);

out_disable_clocks:
	clk_bulk_disable_unprepare(chip->npwm, priv->clks);
	return ret;
}

static void novatek_pwm_shutdown(struct platform_device *pdev)
{
	novatek_pwm_disable(platform_get_drvdata(pdev));
}

static const struct novatek_pwm_soc_data na51089_pwm_data = {
	.npwm = 12,
	.num_ext_channels = 8,
	.group_size = 4,
};

static const struct of_device_id novatek_pwm_of_match[] = {
	{
		.compatible = "novatek,na51089-pwm",
		.data = &na51089_pwm_data,
	},
	{ }
};
MODULE_DEVICE_TABLE(of, novatek_pwm_of_match);

static struct platform_driver novatek_pwm_driver = {
	.probe = novatek_pwm_probe,
	.shutdown = novatek_pwm_shutdown,
	.driver = {
		.name = "novatek-pwm",
		.of_match_table = novatek_pwm_of_match,
	},
};
module_platform_driver(novatek_pwm_driver);

MODULE_AUTHOR("Yu-Tung Chang <mtwget@gmail.com>");
MODULE_DESCRIPTION("Novatek PWM driver");
MODULE_LICENSE("GPL");
