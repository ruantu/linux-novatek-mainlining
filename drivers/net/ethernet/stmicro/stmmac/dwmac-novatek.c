// SPDX-License-Identifier: GPL-2.0-only

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/etherdevice.h>
#include <linux/mfd/syscon.h>
#include <linux/mfd/syscon/novatek-sramctrl.h>
#include <linux/module.h>
#include <linux/of_mdio.h>
#include <linux/phy.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/reset.h>

#include "stmmac.h"
#include "stmmac_platform.h"

#define NOVATEK_ETH_REF_RATE	50000000

struct novatek_dwmac {
	struct clk *refclk;
	struct reset_control *reset;
	struct regmap *sram;
	bool enabled;
};

static int novatek_dwmac_init(struct device *dev, void *data)
{
	struct novatek_dwmac *dwmac = data;
	int ret;

	if (dwmac->enabled)
		return 0;

	ret = clk_prepare_enable(dwmac->refclk);
	if (ret)
		return ret;
	fsleep(10);

	ret = reset_control_deassert(dwmac->reset);
	if (ret)
		goto disable_clk;
	fsleep(10);

	ret = regmap_update_bits(dwmac->sram, NOVATEK_SRAM_SHUTDOWN,
				 NOVATEK_SRAM_ETH_SD_MASK,
				 FIELD_PREP(NOVATEK_SRAM_ETH_SD_MASK,
					    NOVATEK_SRAM_ETH_ENABLE));
	if (ret)
		goto assert_reset;

	dwmac->enabled = true;
	return 0;

assert_reset:
	reset_control_assert(dwmac->reset);
disable_clk:
	clk_disable_unprepare(dwmac->refclk);
	return ret;
}

static int novatek_dwmac_power_down(struct novatek_dwmac *dwmac)
{
	int ret;

	if (!dwmac->enabled)
		return 0;

	ret = reset_control_assert(dwmac->reset);
	if (ret)
		return ret;

	ret = regmap_update_bits(dwmac->sram, NOVATEK_SRAM_SHUTDOWN,
				 NOVATEK_SRAM_ETH_SD_MASK,
				 FIELD_PREP(NOVATEK_SRAM_ETH_SD_MASK,
					    NOVATEK_SRAM_ETH_DISABLE));
	clk_disable_unprepare(dwmac->refclk);
	dwmac->enabled = false;
	return ret;
}

static void novatek_dwmac_exit(struct device *dev, void *data)
{
	int ret;

	ret = novatek_dwmac_power_down(data);
	if (ret)
		dev_err(dev, "Failed to disable Ethernet: %d\n", ret);
}

static int novatek_dwmac_suspend(struct device *dev, void *data)
{
	struct stmmac_priv *priv = netdev_priv(dev_get_drvdata(dev));

	if (priv->wolopts)
		return 0;

	return novatek_dwmac_power_down(data);
}

static int novatek_dwmac_resume(struct device *dev, void *data)
{
	struct device_node *np;
	struct phy_device *phydev;
	int ret;

	ret = novatek_dwmac_init(dev, data);
	if (ret)
		return ret;

	np = of_parse_phandle(dev->of_node, "phy-handle", 0);
	phydev = of_phy_find_device(np);
	of_node_put(np);
	if (!phydev)
		return 0;

	/* Restore the mux and RMII clock before DMA reset, even if the PHY
	 * has no resume callback or is detached from a closed interface.
	 * BMCR reads do not clear latched status.
	 */
	ret = phy_read(phydev, MII_BMCR);
	put_device(&phydev->mdio.dev);
	if (ret < 0)
		return ret;

	return 0;
}

static void novatek_dwmac_ptp_config(struct stmmac_priv *priv)
{
	/* The CSR clock does not establish the timestamp clock frequency. */
	priv->dma_cap.time_stamp = 0;
	priv->dma_cap.atime_stamp = 0;
}

static int novatek_dwmac_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct plat_stmmacenet_data *plat;
	struct stmmac_resources resources;
	struct novatek_dwmac *dwmac;
	struct clk *clk;
	int ret;

	dwmac = devm_kzalloc(dev, sizeof(*dwmac), GFP_KERNEL);
	if (!dwmac)
		return -ENOMEM;

	clk = devm_clk_get(dev, "stmmaceth");
	if (IS_ERR(clk))
		return dev_err_probe(dev, PTR_ERR(clk),
				     "Failed to get APB clock\n");

	dwmac->refclk = devm_clk_get(dev, "ethref");
	if (IS_ERR(dwmac->refclk))
		return dev_err_probe(dev, PTR_ERR(dwmac->refclk),
				     "Failed to get reference clock\n");

	if (clk_get_rate(dwmac->refclk) != NOVATEK_ETH_REF_RATE)
		return dev_err_probe(dev, -EINVAL,
				     "Reference clock must be 50 MHz\n");

	dwmac->sram = syscon_regmap_lookup_by_phandle(dev->of_node, "regmap");
	if (IS_ERR(dwmac->sram))
		return dev_err_probe(dev, PTR_ERR(dwmac->sram),
				     "Failed to get SRAM controller\n");

	ret = stmmac_get_platform_resources(pdev, &resources);
	if (ret)
		return ret;

	plat = devm_stmmac_probe_config_dt(pdev, resources.mac);
	if (IS_ERR(plat))
		return PTR_ERR(plat);
	if (plat->phy_interface != PHY_INTERFACE_MODE_MII &&
	    plat->phy_interface != PHY_INTERFACE_MODE_RMII)
		return dev_err_probe(dev, -EINVAL,
				     "Unsupported PHY interface\n");
	if (!plat->stmmac_rst)
		return dev_err_probe(dev, -EINVAL, "Missing MAC reset\n");

	/* The reference clock must run before releasing the MAC reset. */
	dwmac->reset = plat->stmmac_rst;
	plat->stmmac_rst = NULL;
	plat->bsp_priv = dwmac;
	plat->init = novatek_dwmac_init;
	plat->exit = novatek_dwmac_exit;
	plat->suspend = novatek_dwmac_suspend;
	plat->resume = novatek_dwmac_resume;
	plat->ptp_clk_freq_config = novatek_dwmac_ptp_config;
	plat->clk_ptp_rate = 0;
	plat->core_type = DWMAC_CORE_GMAC4;
	plat->host_dma_width = 32;
	plat->max_speed = SPEED_100;
	plat->maxmtu = ETH_DATA_LEN;

	return devm_stmmac_pltfr_probe(pdev, plat, &resources);
}

static const struct of_device_id novatek_dwmac_match[] = {
	{ .compatible = "novatek,na51089-dwmac" },
	{ }
};
MODULE_DEVICE_TABLE(of, novatek_dwmac_match);

static struct platform_driver novatek_dwmac_driver = {
	.probe = novatek_dwmac_probe,
	.driver = {
		.name = "novatek-dwmac",
		.pm = &stmmac_pltfr_pm_ops,
		.of_match_table = novatek_dwmac_match,
	},
};
module_platform_driver(novatek_dwmac_driver);

MODULE_AUTHOR("Yu-Tung Chang <mtwget@gmail.com>");
MODULE_DESCRIPTION("Novatek NA51089 DWMAC glue driver");
MODULE_LICENSE("GPL");
