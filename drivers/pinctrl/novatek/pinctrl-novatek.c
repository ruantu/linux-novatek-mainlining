// SPDX-License-Identifier: GPL-2.0-only

#include <linux/bitops.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/pinctrl/pinconf-generic.h>
#include <linux/pinctrl/pinconf.h>
#include <linux/pinctrl/pinctrl.h>
#include <linux/pinctrl/pinmux.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/slab.h>
#include <linux/spinlock.h>

#include "../pinctrl-utils.h"
#include "pinctrl-novatek.h"

struct novatek_pinctrl {
	void __iomem *top_base;
	void __iomem *pad_base;
	raw_spinlock_t lock;
	const struct novatek_pinctrl_soc_data *soc;
	struct pinctrl_desc desc;
	struct pinctrl_dev *pctldev;
};

static int novatek_get_groups_count(struct pinctrl_dev *pctldev)
{
	struct novatek_pinctrl *npctl = pinctrl_dev_get_drvdata(pctldev);

	return npctl->soc->ngroups;
}

static const char *novatek_get_group_name(struct pinctrl_dev *pctldev,
					  unsigned int selector)
{
	struct novatek_pinctrl *npctl = pinctrl_dev_get_drvdata(pctldev);

	return npctl->soc->groups[selector].name;
}

static int novatek_get_group_pins(struct pinctrl_dev *pctldev,
				  unsigned int selector,
				  const unsigned int **pins,
				  unsigned int *num_pins)
{
	struct novatek_pinctrl *npctl = pinctrl_dev_get_drvdata(pctldev);
	const struct novatek_pinctrl_group *group;

	group = &npctl->soc->groups[selector];
	*pins = group->pins;
	*num_pins = group->npins;

	return 0;
}

static const struct pinctrl_ops novatek_pinctrl_ops = {
	.get_groups_count = novatek_get_groups_count,
	.get_group_name = novatek_get_group_name,
	.get_group_pins = novatek_get_group_pins,
	.dt_node_to_map = pinconf_generic_dt_node_to_map_all,
	.dt_free_map = pinconf_generic_dt_free_map,
};

static int novatek_get_functions_count(struct pinctrl_dev *pctldev)
{
	struct novatek_pinctrl *npctl = pinctrl_dev_get_drvdata(pctldev);

	return npctl->soc->nfunctions;
}

static const char *novatek_get_function_name(struct pinctrl_dev *pctldev,
					     unsigned int selector)
{
	struct novatek_pinctrl *npctl = pinctrl_dev_get_drvdata(pctldev);

	return npctl->soc->functions[selector].name;
}

static int novatek_get_function_groups(struct pinctrl_dev *pctldev,
				       unsigned int selector,
				       const char * const **groups,
				       unsigned int *num_groups)
{
	struct novatek_pinctrl *npctl = pinctrl_dev_get_drvdata(pctldev);
	const struct novatek_pinctrl_function *function;

	function = &npctl->soc->functions[selector];
	*groups = function->groups;
	*num_groups = function->ngroups;

	return 0;
}

static const struct novatek_mux_bank *novatek_find_bank(
	const struct novatek_pinctrl_soc_data *soc, uint pin, uint *bit)
{
	uint i;

	for (i = 0; i < soc->npinmux_banks; i++) {
		const struct novatek_mux_bank *bank = &soc->pinmux_banks[i];
		uint offset;

		if (pin < bank->pin_base)
			continue;
		offset = pin - bank->pin_base;
		if (offset >= 32 || !(bank->valid_mask & BIT(offset)))
			continue;
		*bit = offset;
		return bank;
	}

	return NULL;
}

static int novatek_set_mux(struct pinctrl_dev *pctldev, unsigned int selector,
			   unsigned int group_selector)
{
	struct novatek_pinctrl *npctl = pinctrl_dev_get_drvdata(pctldev);
	const struct novatek_pinctrl_soc_data *soc = npctl->soc;
	const struct novatek_pinctrl_group *group;
	ulong flags;
	uint i;
	int ret = 0;

	group = &soc->groups[group_selector];
	raw_spin_lock_irqsave(&npctl->lock, flags);
	for (i = 0; i < group->nsettings; i++) {
		const struct novatek_mux_setting *setting = &group->settings[i];
		u32 val;

		val = readl(npctl->top_base + setting->reg);
		val &= ~setting->mask;
		val |= setting->val & setting->mask;
		writel(val, npctl->top_base + setting->reg);
	}

	for (i = 0; i < group->npins; i++) {
		const struct novatek_mux_bank *bank;
		u32 bits = soc->mux_peripheral;
		u32 mask, val;
		uint bit;

		bank = novatek_find_bank(soc, group->pins[i], &bit);
		if (!bank) {
			ret = -EINVAL;
			goto out;
		}
		if (group->gpio_modes && test_bit(i, group->gpio_modes))
			bits = soc->mux_gpio;
		mask = soc->mux_pin_mask << bit;
		val = readl(npctl->top_base + bank->reg);
		val &= ~mask;
		val |= (bits << bit) & mask;
		writel(val, npctl->top_base + bank->reg);
	}

out:
	raw_spin_unlock_irqrestore(&npctl->lock, flags);

	return ret;
}

static int novatek_gpio_request_enable(struct pinctrl_dev *pctldev,
				       struct pinctrl_gpio_range *range,
				       unsigned int pin)
{
	struct novatek_pinctrl *npctl = pinctrl_dev_get_drvdata(pctldev);
	const struct novatek_pinctrl_soc_data *soc = npctl->soc;
	const struct novatek_mux_bank *bank;
	ulong flags;
	u32 mask, val;
	uint bit;
	int ret = 0;

	raw_spin_lock_irqsave(&npctl->lock, flags);
	bank = novatek_find_bank(soc, pin, &bit);
	if (!bank) {
		ret = -EINVAL;
		goto out;
	}
	mask = soc->mux_pin_mask << bit;
	val = readl(npctl->top_base + bank->reg);
	val &= ~mask;
	val |= (soc->mux_gpio << bit) & mask;
	writel(val, npctl->top_base + bank->reg);

out:
	raw_spin_unlock_irqrestore(&npctl->lock, flags);

	return ret;
}

static const struct pinmux_ops novatek_pinmux_ops = {
	.get_functions_count = novatek_get_functions_count,
	.get_function_name = novatek_get_function_name,
	.get_function_groups = novatek_get_function_groups,
	.set_mux = novatek_set_mux,
	.gpio_request_enable = novatek_gpio_request_enable,
	.strict = true,
};

static int novatek_pin_config_get(struct pinctrl_dev *pctldev,
				  unsigned int pin, unsigned long *config)
{
	struct novatek_pinctrl *npctl = pinctrl_dev_get_drvdata(pctldev);
	const struct novatek_pinctrl_soc_data *soc = npctl->soc;
	struct novatek_pad_field field;
	enum pin_config_param param = pinconf_to_config_param(*config);
	u32 arg, raw, val;
	int ret;

	switch (param) {
	case PIN_CONFIG_BIAS_DISABLE:
	case PIN_CONFIG_BIAS_PULL_DOWN:
	case PIN_CONFIG_BIAS_PULL_UP:
	case PIN_CONFIG_BIAS_BUS_HOLD:
		ret = soc->get_pull_field(pin, &field);
		if (ret)
			return ret;
		val = readl(npctl->pad_base + field.reg);
		raw = (val & field.mask) >> field.shift;
		if ((param == PIN_CONFIG_BIAS_DISABLE &&
		     raw != soc->pull_none) ||
		    (param == PIN_CONFIG_BIAS_PULL_DOWN &&
		     raw != soc->pull_down) ||
		    (param == PIN_CONFIG_BIAS_PULL_UP &&
		     raw != soc->pull_up) ||
		    (param == PIN_CONFIG_BIAS_BUS_HOLD &&
		     raw != soc->pull_keeper))
			return -EINVAL;
		arg = param == PIN_CONFIG_BIAS_DISABLE ? 0 : 1;
		break;
	case PIN_CONFIG_DRIVE_STRENGTH:
		ret = soc->get_drive_field(pin, &field);
		if (ret)
			return ret;
		val = readl(npctl->pad_base + field.reg);
		raw = (val & field.mask) >> field.shift;
		arg = soc->raw_to_drive(&field, raw);
		if (!arg)
			return -EINVAL;
		break;
	default:
		return -ENOTSUPP;
	}
	*config = pinconf_to_config_packed(param, arg);

	return 0;
}

static int novatek_pin_config_decode(struct novatek_pinctrl *npctl,
				     uint pin, ulong config,
				     struct novatek_pad_field *field,
				     u32 *raw)
{
	const struct novatek_pinctrl_soc_data *soc = npctl->soc;
	enum pin_config_param param = pinconf_to_config_param(config);
	u32 arg = pinconf_to_config_argument(config);
	int ret;

	switch (param) {
	case PIN_CONFIG_BIAS_DISABLE:
		*raw = soc->pull_none;
		return soc->get_pull_field(pin, field);
	case PIN_CONFIG_BIAS_PULL_DOWN:
		if (!arg)
			return -EINVAL;
		*raw = soc->pull_down;
		return soc->get_pull_field(pin, field);
	case PIN_CONFIG_BIAS_PULL_UP:
		if (!arg)
			return -EINVAL;
		*raw = soc->pull_up;
		return soc->get_pull_field(pin, field);
	case PIN_CONFIG_BIAS_BUS_HOLD:
		*raw = soc->pull_keeper;
		return soc->get_pull_field(pin, field);
	case PIN_CONFIG_DRIVE_STRENGTH:
		ret = soc->get_drive_field(pin, field);
		if (ret)
			return ret;
		return soc->drive_to_raw(field, arg, raw);
	default:
		return -ENOTSUPP;
	}
}

static int novatek_pin_config_set(struct pinctrl_dev *pctldev,
				  unsigned int pin, unsigned long *configs,
				  unsigned int num_configs)
{
	struct novatek_pinctrl *npctl = pinctrl_dev_get_drvdata(pctldev);
	struct novatek_pad_field field;
	ulong flags;
	uint i;
	u32 raw, val;
	int ret;

	for (i = 0; i < num_configs; i++) {
		ret = novatek_pin_config_decode(npctl, pin, configs[i], &field,
						&raw);
		if (ret)
			return ret;
	}

	for (i = 0; i < num_configs; i++) {
		ret = novatek_pin_config_decode(npctl, pin, configs[i], &field,
						&raw);
		if (ret)
			return ret;
		raw_spin_lock_irqsave(&npctl->lock, flags);
		val = readl(npctl->pad_base + field.reg);
		val &= ~field.mask;
		val |= raw << field.shift;
		writel(val, npctl->pad_base + field.reg);
		raw_spin_unlock_irqrestore(&npctl->lock, flags);
	}

	return 0;
}

static int novatek_pin_config_group_set(struct pinctrl_dev *pctldev,
					unsigned int selector,
					unsigned long *configs,
					unsigned int num_configs)
{
	struct novatek_pinctrl *npctl = pinctrl_dev_get_drvdata(pctldev);
	const struct novatek_pinctrl_group *group;
	struct novatek_pad_field field;
	uint i, j;
	u32 raw;
	int ret;

	group = &npctl->soc->groups[selector];
	for (i = 0; i < group->npins; i++) {
		for (j = 0; j < num_configs; j++) {
			ret = novatek_pin_config_decode(
				npctl, group->pins[i], configs[j],
				&field, &raw);
			if (ret)
				return ret;
		}
	}

	for (i = 0; i < group->npins; i++) {
		ret = novatek_pin_config_set(pctldev, group->pins[i], configs,
					     num_configs);
		if (ret)
			return ret;
	}

	return 0;
}

static const struct pinconf_ops novatek_pinconf_ops = {
	.is_generic = true,
	.pin_config_get = novatek_pin_config_get,
	.pin_config_set = novatek_pin_config_set,
	.pin_config_group_set = novatek_pin_config_group_set,
};

int novatek_pinctrl_probe(struct platform_device *pdev)
{
	const struct novatek_pinctrl_soc_data *soc;
	struct novatek_pinctrl *npctl;

	soc = device_get_match_data(&pdev->dev);
	if (!soc || !soc->pinmux_banks || !soc->npinmux_banks ||
	    !soc->mux_pin_mask)
		return -EINVAL;

	npctl = devm_kzalloc(&pdev->dev, sizeof(*npctl), GFP_KERNEL);
	if (!npctl)
		return -ENOMEM;

	npctl->top_base =
		devm_platform_ioremap_resource_byname(pdev, soc->top_resource);
	if (IS_ERR(npctl->top_base))
		return PTR_ERR(npctl->top_base);

	npctl->pad_base =
		devm_platform_ioremap_resource_byname(pdev, soc->pad_resource);
	if (IS_ERR(npctl->pad_base))
		return PTR_ERR(npctl->pad_base);

	raw_spin_lock_init(&npctl->lock);
	npctl->soc = soc;

	npctl->desc.name = soc->name;
	npctl->desc.pctlops = &novatek_pinctrl_ops;
	npctl->desc.pmxops = &novatek_pinmux_ops;
	npctl->desc.confops = &novatek_pinconf_ops;
	npctl->desc.pins = soc->pins;
	npctl->desc.npins = soc->npins;
	npctl->desc.owner = pdev->dev.driver->owner;

	npctl->pctldev = devm_pinctrl_register(&pdev->dev, &npctl->desc, npctl);
	if (IS_ERR(npctl->pctldev))
		return dev_err_probe(&pdev->dev, PTR_ERR(npctl->pctldev),
				     "failed to register pinctrl\n");

	platform_set_drvdata(pdev, npctl);

	return 0;
}
EXPORT_SYMBOL_GPL(novatek_pinctrl_probe);

MODULE_DESCRIPTION("Novatek pinctrl core driver");
MODULE_LICENSE("GPL");
