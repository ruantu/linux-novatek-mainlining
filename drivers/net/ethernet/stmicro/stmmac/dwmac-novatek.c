// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2026 Yu-Tung Chang <mtwget@gmail.com>
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/etherdevice.h>
#include <linux/io.h>
#include <linux/mdio-mux.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/phy.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/pm_wakeup.h>
#include <linux/reset.h>
#include <linux/rtnetlink.h>

#include "stmmac.h"
#include "stmmac_platform.h"

#define ETH_DEBUG_REG					0x3000
#define ETH_CONTROL_REG					0x3004
#define ETH_IO_CTRL_REG					0x3014
#define PHY_RESET_REG					0x3800
#define PHY_POWER_9C_REG				0x389c
#define PHY_POWER_C8_REG				0x38c8
#define PHY_POWER_CC_REG				0x38cc
#define PHY_POWER_DC_REG				0x38dc
#define PHY_POWER_F8_REG				0x38f8
#define PHY_LED_TIMING_REG				0x3900
#define PHY_BREAK_TIMER_CTRL_REG			0x3a84
#define PHY_BREAK_TIMER_LOW_REG				0x3a88
#define PHY_BREAK_TIMER_HIGH_REG			0x3a8c
#define PHY_POWER_2E8_REG				0x3ae8
#define PHY_TUNE_2F8_REG				0x3af8
#define PHY_TUNE_354_REG				0x3b54
#define PHY_TUNE_358_REG				0x3b58

/* ETH_DEBUG bit fields */
#define ETH_DEBUG_PHY_ADDR				GENMASK(20, 16)
#define ETH_DEBUG_PHY_ADDR_INTERNAL			0x00

/* ETH_CONTROL bit fields */
#define ETH_CONTROL_NA51089_PHY_SEL			GENMASK(5, 4)
#define ETH_CONTROL_NA51089_PHY_SEL_INTERNAL		0x0
#define ETH_CONTROL_NA51089_PHY_SEL_EXTERNAL		0x1
#define ETH_CONTROL_NA51089_RMII_OUTPUT_PHASE		BIT(31)

/* ETH_IO_CTRL bit fields */
#define ETH_IO_CTRL_REF_SEL				BIT(0)
#define ETH_IO_CTRL_REF_OUT				BIT(4)
#define ETH_IO_CTRL_REF_IN				BIT(5)
#define ETH_IO_CTRL_TXD_SRC				GENMASK(31, 30)
#define ETH_IO_CTRL_TXD_SRC_RMII			0x0

/* PHY_RESET bit fields */
#define PHY_RESET_SW					BIT(1)
#define PHY_RESET_PLL_WR				BIT(3)

/* PHY_POWER_9C bit fields */
#define PHY_POWER_9C_BIT0				BIT(0)

/* PHY_POWER_C8 bit fields */
#define PHY_POWER_C8_STAGE1				BIT(0)
#define PHY_POWER_C8_STAGE2				BIT(1)

/* PHY_POWER_CC bit fields */
#define PHY_POWER_CC_BIT0				BIT(0)

/* PHY_POWER_DC bit fields */
#define PHY_POWER_DC_BIT0				BIT(0)

/* PHY_POWER_F8 bit fields */
#define PHY_POWER_F8_BIT7				BIT(7)

/* PHY_LED_TIMING bit fields */
#define PHY_LED_TIMING_DEFAULT				0x00000040

/* PHY_BREAK_TIMER_CTRL bit fields */
#define PHY_BREAK_TIMER_CTRL_DEFAULT			0x000000c9

/* PHY_BREAK_TIMER_LOW bit fields */
#define PHY_BREAK_TIMER_LOW_DEFAULT			0x00000053

/* PHY_BREAK_TIMER_HIGH bit fields */
#define PHY_BREAK_TIMER_HIGH_DEFAULT			0x00000007

/* PHY_POWER_2E8 bit fields */
#define PHY_POWER_2E8_BIT0				BIT(0)

/* PHY_TUNE_2F8 bit fields */
#define PHY_TUNE_2F8_NA51089_VALUE			GENMASK(2, 0)
#define PHY_TUNE_2F8_NA51089_VALUE_DEFAULT		0x3

/* PHY_TUNE_354 bit fields */
#define PHY_TUNE_354_NA51089_VALUE			BIT(7)
#define PHY_TUNE_354_NA51089_VALUE_DEFAULT \
	(FIELD_PREP_CONST(PHY_TUNE_354_NA51089_VALUE, 1))

/* PHY_TUNE_358 bit fields */
#define PHY_TUNE_358_NA51089_VALUE			GENMASK(5, 4)
#define PHY_TUNE_358_NA51089_VALUE_DEFAULT		0x3

#define ETH_REF_RATE					50000000

#define ETH_PHY_REF_RATE				25000000

struct novatek_priv_data {
	void __iomem *ioaddr;
	void *mux_handle;
	struct clk *refclk;
	struct clk *phy_refclk;
	struct reset_control *reset;
	struct reset_control *rst_ephy;
	phy_interface_t interface;
	bool internal_phy_powered;
	bool use_internal_phy;
	bool refclk_out;
};

static void novatek_dwmac_config_interface(struct novatek_priv_data *gmac)
{
	void __iomem *ioaddr = gmac->ioaddr;
	u32 val;

	if (gmac->interface != PHY_INTERFACE_MODE_RMII)
		return;

	val = readl(ioaddr + ETH_IO_CTRL_REG);
	val &= ~(ETH_IO_CTRL_REF_SEL | ETH_IO_CTRL_REF_OUT |
		 ETH_IO_CTRL_REF_IN | ETH_IO_CTRL_TXD_SRC);
	val |= ETH_IO_CTRL_REF_IN | FIELD_PREP(ETH_IO_CTRL_TXD_SRC, ETH_IO_CTRL_TXD_SRC_RMII);
	if (gmac->refclk_out)
		val |= ETH_IO_CTRL_REF_OUT;
	else
		val |= ETH_IO_CTRL_REF_SEL;
	writel(val, ioaddr + ETH_IO_CTRL_REG);
}

static int novatek_dwmac_power_internal_phy(struct novatek_priv_data *gmac)
{
	void __iomem *ioaddr = gmac->ioaddr;
	u32 val;
	int ret;

	if (gmac->internal_phy_powered)
		return 0;

	if (!gmac->rst_ephy)
		return -ENODEV;

	ret = reset_control_assert(gmac->rst_ephy);
	if (ret)
		return ret;
	fsleep(20000);

	ret = reset_control_deassert(gmac->rst_ephy);
	if (ret) {
		reset_control_assert(gmac->rst_ephy);
		return ret;
	}
	fsleep(20000);

	val = readl(ioaddr + PHY_POWER_F8_REG);
	val &= ~PHY_POWER_F8_BIT7;
	val |= PHY_POWER_F8_BIT7;
	writel(val, ioaddr + PHY_POWER_F8_REG);
	fsleep(20);

	val = readl(ioaddr + PHY_POWER_C8_REG);
	val &= ~PHY_POWER_C8_STAGE1;
	writel(val, ioaddr + PHY_POWER_C8_REG);
	fsleep(200);

	val = readl(ioaddr + PHY_POWER_C8_REG);
	val &= ~PHY_POWER_C8_STAGE2;
	writel(val, ioaddr + PHY_POWER_C8_REG);
	fsleep(250);

	val = readl(ioaddr + PHY_POWER_2E8_REG);
	val &= ~PHY_POWER_2E8_BIT0;
	writel(val, ioaddr + PHY_POWER_2E8_REG);

	val = readl(ioaddr + PHY_POWER_CC_REG);
	val &= ~PHY_POWER_CC_BIT0;
	writel(val, ioaddr + PHY_POWER_CC_REG);

	val = readl(ioaddr + PHY_POWER_DC_REG);
	val &= ~PHY_POWER_DC_BIT0;
	val |= PHY_POWER_DC_BIT0;
	writel(val, ioaddr + PHY_POWER_DC_REG);

	val = readl(ioaddr + PHY_POWER_9C_REG);
	val &= ~PHY_POWER_9C_BIT0;
	writel(val, ioaddr + PHY_POWER_9C_REG);

	val = readl(ioaddr + PHY_TUNE_2F8_REG);
	val &= ~PHY_TUNE_2F8_NA51089_VALUE;
	val |= FIELD_PREP(PHY_TUNE_2F8_NA51089_VALUE, PHY_TUNE_2F8_NA51089_VALUE_DEFAULT);
	writel(val, ioaddr + PHY_TUNE_2F8_REG);

	val = readl(ioaddr + PHY_TUNE_354_REG);
	val &= ~PHY_TUNE_354_NA51089_VALUE;
	val |= PHY_TUNE_354_NA51089_VALUE_DEFAULT;
	writel(val, ioaddr + PHY_TUNE_354_REG);

	val = readl(ioaddr + PHY_TUNE_358_REG);
	val &= ~PHY_TUNE_358_NA51089_VALUE;
	val |= FIELD_PREP(PHY_TUNE_358_NA51089_VALUE, PHY_TUNE_358_NA51089_VALUE_DEFAULT);
	writel(val, ioaddr + PHY_TUNE_358_REG);

	val = readl(ioaddr + PHY_RESET_REG);
	val &= ~PHY_RESET_SW;
	val |= PHY_RESET_SW;
	writel(val, ioaddr + PHY_RESET_REG);

	writel(PHY_BREAK_TIMER_LOW_DEFAULT, ioaddr + PHY_BREAK_TIMER_LOW_REG);
	writel(PHY_BREAK_TIMER_HIGH_DEFAULT, ioaddr + PHY_BREAK_TIMER_HIGH_REG);
	writel(PHY_BREAK_TIMER_CTRL_DEFAULT, ioaddr + PHY_BREAK_TIMER_CTRL_REG);

	writel(PHY_LED_TIMING_DEFAULT, ioaddr + PHY_LED_TIMING_REG);
	fsleep(10000);

	val = readl(ioaddr + PHY_RESET_REG);
	val &= ~(PHY_RESET_SW | PHY_RESET_PLL_WR);
	val |= PHY_RESET_PLL_WR;
	writel(val, ioaddr + PHY_RESET_REG);
	gmac->internal_phy_powered = true;
	return 0;
}

static void novatek_dwmac_unpower_internal_phy(struct novatek_priv_data *gmac)
{
	if (!gmac->internal_phy_powered)
		return;

	reset_control_assert(gmac->rst_ephy);
	gmac->internal_phy_powered = false;
}

static void novatek_dwmac_set_phy(struct novatek_priv_data *gmac)
{
	void __iomem *ioaddr = gmac->ioaddr;
	u32 val;

	if (gmac->use_internal_phy) {
		val = readl(ioaddr + ETH_DEBUG_REG);
		val &= ~ETH_DEBUG_PHY_ADDR;
		val |= FIELD_PREP(ETH_DEBUG_PHY_ADDR, ETH_DEBUG_PHY_ADDR_INTERNAL);
		writel(val, ioaddr + ETH_DEBUG_REG);
	}

	val = readl(ioaddr + ETH_CONTROL_REG);
	val &= ~ETH_CONTROL_NA51089_PHY_SEL;
	if (gmac->use_internal_phy) {
		val |= FIELD_PREP(ETH_CONTROL_NA51089_PHY_SEL, ETH_CONTROL_NA51089_PHY_SEL_INTERNAL);
	} else {
		val |= FIELD_PREP(ETH_CONTROL_NA51089_PHY_SEL, ETH_CONTROL_NA51089_PHY_SEL_EXTERNAL);
		val &= ~ETH_CONTROL_NA51089_RMII_OUTPUT_PHASE;
	}
	writel(val, ioaddr + ETH_CONTROL_REG);
}

static int mdio_mux_switch_fn(int current_child, int desired_child, void *data)
{
	struct stmmac_priv *priv = data;
	struct novatek_priv_data *gmac = priv->plat->bsp_priv;
	int ret;

	switch (desired_child) {
	case ETH_CONTROL_NA51089_PHY_SEL_INTERNAL:
		ret = novatek_dwmac_power_internal_phy(gmac);
		if (ret)
			return ret;
		gmac->use_internal_phy = true;
		break;

	case ETH_CONTROL_NA51089_PHY_SEL_EXTERNAL:
		gmac->use_internal_phy = false;
		break;

	default:
		return -EINVAL;
	}

	novatek_dwmac_set_phy(gmac);
	return 0;
}

static int novatek_dwmac_init(struct device *dev, void *data)
{
	struct novatek_priv_data *gmac = data;
	int ret;

	ret = clk_prepare_enable(gmac->refclk);
	if (ret)
		return ret;

	ret = clk_prepare_enable(gmac->phy_refclk);
	if (ret)
		goto out_disable_clk;
	fsleep(10);

	ret = reset_control_deassert(gmac->reset);
	if (ret)
		goto out_disable_phy_clk;
	fsleep(10);

	novatek_dwmac_config_interface(gmac);

	if (gmac->use_internal_phy) {
		ret = novatek_dwmac_power_internal_phy(gmac);
		if (ret)
			goto out_assert_reset;
	}

	novatek_dwmac_set_phy(gmac);
	return 0;

out_assert_reset:
	reset_control_assert(gmac->reset);
out_disable_phy_clk:
	clk_disable_unprepare(gmac->phy_refclk);
out_disable_clk:
	clk_disable_unprepare(gmac->refclk);
	return ret;
}

static void novatek_dwmac_exit(struct device *dev, void *data)
{
	struct novatek_priv_data *gmac = data;
	int ret;

	novatek_dwmac_unpower_internal_phy(gmac);

	ret = reset_control_assert(gmac->reset);
	if (ret)
		dev_err(dev, "Failed to assert MAC reset: %d\n", ret);

	clk_disable_unprepare(gmac->phy_refclk);
	clk_disable_unprepare(gmac->refclk);
}

static int novatek_dwmac_suspend(struct device *dev, void *data)
{
	struct stmmac_priv *priv = netdev_priv(dev_get_drvdata(dev));

	if (!priv->wolopts)
		novatek_dwmac_exit(dev, data);
	else
		device_set_awake_path(dev);

	return 0;
}

static int novatek_dwmac_resume(struct device *dev, void *data)
{
	struct stmmac_priv *priv = netdev_priv(dev_get_drvdata(dev));

	if (priv->wolopts)
		return 0;

	return novatek_dwmac_init(dev, data);
}

static void novatek_dwmac_ptp_config(struct stmmac_priv *priv)
{
	priv->dma_cap.time_stamp = 0;
	priv->dma_cap.atime_stamp = 0;
}

static int novatek_dwmac_register_mdio_mux(struct stmmac_priv *priv)
{
	struct novatek_priv_data *gmac = priv->plat->bsp_priv;
	struct device *dev = priv->device;
	struct reset_control *reset;

	struct device_node *mdio_mux __free(device_node) =
		of_get_child_by_name(dev->of_node, "mdio-mux");
	if (!mdio_mux)
		return dev_err_probe(dev, -ENODEV, "Missing MDIO mux\n");

	reset = of_reset_control_get_optional_exclusive(mdio_mux, "phy");
	if (IS_ERR(reset))
		return dev_err_probe(dev, PTR_ERR(reset), "Cannot get PHY reset\n");
	gmac->rst_ephy = reset;

	return mdio_mux_init(dev, mdio_mux, mdio_mux_switch_fn, &gmac->mux_handle, priv, priv->mii);
}

static int novatek_dwmac_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct plat_stmmacenet_data *plat_dat;
	struct stmmac_resources stmmac_res;
	struct novatek_priv_data *gmac;
	struct net_device *ndev;
	struct stmmac_priv *priv;
	int ret;

	ret = stmmac_get_platform_resources(pdev, &stmmac_res);
	if (ret)
		return ret;

	gmac = devm_kzalloc(dev, sizeof(*gmac), GFP_KERNEL);
	if (!gmac)
		return -ENOMEM;
	gmac->ioaddr = stmmac_res.addr;

	gmac->refclk = devm_clk_get(dev, "ethref");
	if (IS_ERR(gmac->refclk))
		return dev_err_probe(dev, PTR_ERR(gmac->refclk), "Failed to get reference clock\n");
	if (clk_get_rate(gmac->refclk) != ETH_REF_RATE)
		return dev_err_probe(dev, -EINVAL, "Reference clock must be 50 MHz\n");

	gmac->phy_refclk = devm_clk_get_optional(dev, "phy-ref");
	if (IS_ERR(gmac->phy_refclk))
		return dev_err_probe(dev, PTR_ERR(gmac->phy_refclk),
				     "Failed to get PHY reference clock\n");
	if (gmac->phy_refclk && clk_get_rate(gmac->phy_refclk) != ETH_PHY_REF_RATE)
		return dev_err_probe(dev, -EINVAL, "PHY reference clock must be 25 MHz\n");

	plat_dat = devm_stmmac_probe_config_dt(pdev, stmmac_res.mac);
	if (IS_ERR(plat_dat))
		return PTR_ERR(plat_dat);
	if (plat_dat->phy_interface != PHY_INTERFACE_MODE_MII &&
	    plat_dat->phy_interface != PHY_INTERFACE_MODE_RMII)
		return dev_err_probe(dev, -EINVAL, "Unsupported PHY interface\n");
	gmac->interface = plat_dat->phy_interface;

	gmac->refclk_out = of_property_read_bool(dev->of_node, "novatek,rmii-refclk-out");

	if (!plat_dat->stmmac_rst)
		return dev_err_probe(dev, -EINVAL, "Missing MAC reset\n");
	gmac->reset = plat_dat->stmmac_rst;
	plat_dat->stmmac_rst = NULL;

	plat_dat->bsp_priv = gmac;
	plat_dat->init = novatek_dwmac_init;
	plat_dat->exit = novatek_dwmac_exit;
	plat_dat->suspend = novatek_dwmac_suspend;
	plat_dat->resume = novatek_dwmac_resume;
	plat_dat->ptp_clk_freq_config = novatek_dwmac_ptp_config;

	plat_dat->clk_ptp_rate = 0;
	plat_dat->core_type = DWMAC_CORE_GMAC4;
	plat_dat->host_dma_width = 32;
	plat_dat->multicast_filter_bins = 0;
	plat_dat->max_speed = SPEED_100;
	plat_dat->maxmtu = ETH_DATA_LEN;

	ret = stmmac_pltfr_probe(pdev, plat_dat, &stmmac_res);
	if (ret)
		return ret;

	ret = pm_runtime_resume_and_get(dev);
	if (ret)
		goto out_remove_dwmac;

	ndev = dev_get_drvdata(dev);
	priv = netdev_priv(ndev);
	ret = novatek_dwmac_register_mdio_mux(priv);
	if (ret) {
		dev_err_probe(dev, ret, "Failed to register MDIO mux\n");
		goto out_put_runtime;
	}

	return 0;

out_put_runtime:
	pm_runtime_put(dev);
out_remove_dwmac:
	stmmac_pltfr_remove(pdev);
	reset_control_put(gmac->rst_ephy);
	return ret;
}

static void novatek_dwmac_remove(struct platform_device *pdev)
{
	struct net_device *ndev = platform_get_drvdata(pdev);
	struct stmmac_priv *priv = netdev_priv(ndev);
	struct novatek_priv_data *gmac = priv->plat->bsp_priv;

	rtnl_lock();
	netif_device_detach(ndev);
	dev_close(ndev);
	rtnl_unlock();

	mdio_mux_uninit(gmac->mux_handle);
	pm_runtime_put(&pdev->dev);
	stmmac_pltfr_remove(pdev);
	reset_control_put(gmac->rst_ephy);
}

static const struct of_device_id novatek_dwmac_match[] = {
	{ .compatible = "novatek,na51089-dwmac" },
	{ }
};
MODULE_DEVICE_TABLE(of, novatek_dwmac_match);

static struct platform_driver novatek_dwmac_driver = {
	.probe = novatek_dwmac_probe,
	.remove = novatek_dwmac_remove,
	.driver = {
		.name = "novatek-dwmac",
		.pm = &stmmac_pltfr_pm_ops,
		.of_match_table = novatek_dwmac_match,
	},
};
module_platform_driver(novatek_dwmac_driver);

MODULE_AUTHOR("Yu-Tung Chang <mtwget@gmail.com>");
MODULE_DESCRIPTION("Novatek DWMAC glue driver");
MODULE_LICENSE("GPL");
