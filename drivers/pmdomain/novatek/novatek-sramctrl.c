// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2026 Yu-Tung Chang <mtwget@gmail.com>
 */

#include <linux/bits.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/slab.h>

#include <dt-bindings/power/novatek,na51089-sramctrl.h>

#define SRAMCTRL_SHUTDOWN_REG			0x00

/* SRAMCTRL_SHUTDOWN bit fields */
#define SRAMCTRL_SHUTDOWN_FSPI_SD		BIT(17)
#define SRAMCTRL_SHUTDOWN_ETH_SD		BIT(18)
#define SRAMCTRL_SHUTDOWN_USB_SD		BIT(19)
#define SRAMCTRL_SHUTDOWN_RSA_SD		BIT(25)
#define SRAMCTRL_SHUTDOWN_SDIO3_SD		BIT(31)

static const struct regmap_config novatek_sramctrl_regmap_config = {
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
	.max_register = SRAMCTRL_SHUTDOWN_REG,
	.max_register_is_0 = true,
	.fast_io = true,
};

struct novatek_sramctrl_domain_data {
	const char *name;
	u32 mask;
};

struct novatek_sramctrl_soc_data {
	const struct novatek_sramctrl_domain_data *domains;
	uint num_domains;
};

static const struct novatek_sramctrl_domain_data na51089_sramctrl_domains[] = {
	[NA51089_SRAM_SW_FSPI] = {
		.name = "fspi-sram",
		.mask = SRAMCTRL_SHUTDOWN_FSPI_SD,
	},
	[NA51089_SRAM_SW_ETH] = {
		.name = "ethernet-sram",
		.mask = SRAMCTRL_SHUTDOWN_ETH_SD,
	},
	[NA51089_SRAM_SW_USB] = {
		.name = "usb-sram",
		.mask = SRAMCTRL_SHUTDOWN_USB_SD,
	},
	[NA51089_SRAM_SW_RSA] = {
		.name = "rsa-sram",
		.mask = SRAMCTRL_SHUTDOWN_RSA_SD,
	},
	[NA51089_SRAM_SW_SDIO3] = {
		.name = "sdio3-sram",
		.mask = SRAMCTRL_SHUTDOWN_SDIO3_SD,
	},
};

static const struct novatek_sramctrl_soc_data na51089_sramctrl_data = {
	.domains = na51089_sramctrl_domains,
	.num_domains = ARRAY_SIZE(na51089_sramctrl_domains),
};

struct novatek_sramctrl_domain {
	struct generic_pm_domain genpd;
	struct regmap *regmap;
	u32 mask;
};

struct novatek_sramctrl {
	struct genpd_onecell_data data;
	struct novatek_sramctrl_domain domains[];
};

static int novatek_sramctrl_power_on(struct generic_pm_domain *genpd)
{
	struct novatek_sramctrl_domain *domain;

	domain = container_of(genpd, struct novatek_sramctrl_domain, genpd);
	return regmap_clear_bits(domain->regmap, SRAMCTRL_SHUTDOWN_REG, domain->mask);
}

static int novatek_sramctrl_power_off(struct generic_pm_domain *genpd)
{
	struct novatek_sramctrl_domain *domain;

	domain = container_of(genpd, struct novatek_sramctrl_domain, genpd);
	return regmap_set_bits(domain->regmap, SRAMCTRL_SHUTDOWN_REG, domain->mask);
}

static int novatek_sramctrl_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	const struct novatek_sramctrl_soc_data *soc;
	struct novatek_sramctrl *sramctrl;
	struct regmap *regmap;
	void __iomem *base;
	uint state;
	uint i;
	int ret;

	soc = device_get_match_data(dev);
	if (!soc)
		return -ENODEV;

	sramctrl = devm_kzalloc(dev, struct_size(sramctrl, domains, soc->num_domains), GFP_KERNEL);
	if (!sramctrl)
		return -ENOMEM;

	sramctrl->data.domains = devm_kcalloc(dev,
					      soc->num_domains, sizeof(*sramctrl->data.domains),
					      GFP_KERNEL);
	if (!sramctrl->data.domains)
		return -ENOMEM;

	base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(base))
		return PTR_ERR(base);

	regmap = devm_regmap_init_mmio(dev, base, &novatek_sramctrl_regmap_config);
	if (IS_ERR(regmap))
		return dev_err_probe(dev, PTR_ERR(regmap), "Failed to initialize SRAM regmap\n");

	ret = regmap_read(regmap, SRAMCTRL_SHUTDOWN_REG, &state);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to read SRAM state\n");

	for (i = 0; i < soc->num_domains; i++) {
		struct novatek_sramctrl_domain *domain = &sramctrl->domains[i];

		domain->regmap = regmap;
		domain->mask = soc->domains[i].mask;
		domain->genpd.name = soc->domains[i].name;

		domain->genpd.power_on = novatek_sramctrl_power_on;
		domain->genpd.power_off = novatek_sramctrl_power_off;

		domain->genpd.flags = GENPD_FLAG_ACTIVE_WAKEUP;

		ret = pm_genpd_init(&domain->genpd, NULL, state & domain->mask);
		if (ret) {
			dev_err_probe(dev, ret, "Failed to initialize SRAM domain\n");
			goto out_remove_domains;
		}
		sramctrl->data.domains[i] = &domain->genpd;
	}

	sramctrl->data.num_domains = soc->num_domains;
	ret = of_genpd_add_provider_onecell(dev->of_node, &sramctrl->data);
	if (ret) {
		dev_err_probe(dev, ret, "Failed to register SRAM domains\n");
		goto out_remove_domains;
	}

	return 0;

out_remove_domains:
	while (i--)
		pm_genpd_remove(&sramctrl->domains[i].genpd);
	return ret;
}

static const struct of_device_id novatek_sramctrl_of_match[] = {
	{
		.compatible = "novatek,na51089-sramctrl",
		.data = &na51089_sramctrl_data,
	},
	{ }
};
MODULE_DEVICE_TABLE(of, novatek_sramctrl_of_match);

static struct platform_driver novatek_sramctrl_driver = {
	.probe = novatek_sramctrl_probe,
	.driver = {
		.name = "novatek-sramctrl",
		.of_match_table = novatek_sramctrl_of_match,
		.suppress_bind_attrs = true,
	},
};
builtin_platform_driver(novatek_sramctrl_driver);

MODULE_AUTHOR("Yu-Tung Chang <mtwget@gmail.com>");
MODULE_DESCRIPTION("Novatek SRAM power domain controller");
MODULE_LICENSE("GPL");
