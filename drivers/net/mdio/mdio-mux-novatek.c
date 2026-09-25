// SPDX-License-Identifier: GPL-2.0-only
/* Copyright (C) 2026 Yu-Tung Chang <mtwget@gmail.com> */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/ethtool.h>
#include <linux/io.h>
#include <linux/mdio-mux.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/netdevice.h>
#include <linux/notifier.h>
#include <linux/of_mdio.h>
#include <linux/of_net.h>
#include <linux/phy.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/reset.h>
#include <linux/rtnetlink.h>

#define NOVATEK_ETH_DEBUG			0x0
#define NOVATEK_ETH_PHY_ADDR_MASK		GENMASK(20, 16)
#define NOVATEK_ETH_PHY_ADDR_INTERNAL		0x00

#define NOVATEK_ETH_CONTROL			0x4
#define NOVATEK_ETH_PHY_SEL_MASK		GENMASK(5, 4)
#define NOVATEK_ETH_PHY_SEL_INTERNAL		0x0
#define NOVATEK_ETH_PHY_SEL_EXTERNAL		0x1
#define NA51089_ETH_RMII_OUTPUT_PHASE_MASK	BIT(31)
#define NA51089_ETH_RMII_OUTPUT_PHASE_RISING	0x0

#define NOVATEK_ETH_IO_CTRL			0x14
#define NOVATEK_ETH_REF_SEL_MASK		BIT(0)
#define NOVATEK_ETH_REF_INTERNAL		0x0
#define NOVATEK_ETH_REF_EXTERNAL		0x1
#define NOVATEK_ETH_REF_OUT_MASK		BIT(4)
#define NOVATEK_ETH_REF_OUT_DISABLE		0x0
#define NOVATEK_ETH_REF_OUT_ENABLE		0x1
#define NOVATEK_ETH_REF_IN_MASK			BIT(5)
#define NOVATEK_ETH_REF_IN_ENABLE		0x1
#define NA51089_ETH_TXD_SRC_MASK		GENMASK(31, 30)
#define NA51089_ETH_TXD_SRC_RMII		0x0

#define NOVATEK_PHY_RESET			0x000
#define NOVATEK_PHY_RESET_SW_MASK		BIT(1)
#define NOVATEK_PHY_RESET_SW_ASSERT		0x1
#define NOVATEK_PHY_RESET_SW_DEASSERT		0x0
#define NOVATEK_PHY_RESET_PLL_WR_MASK		BIT(3)
#define NOVATEK_PHY_RESET_PLL_WR_ENABLE		0x1

#define NOVATEK_PHY_POWER_9C			0x09c
#define NOVATEK_PHY_POWER_9C_MASK		BIT(0)
#define NOVATEK_PHY_POWER_9C_ON			0x0

#define NOVATEK_PHY_POWER_C8			0x0c8
#define NOVATEK_PHY_POWER_C8_STAGE1_MASK	BIT(0)
#define NOVATEK_PHY_POWER_C8_STAGE1_ON		0x0
#define NOVATEK_PHY_POWER_C8_STAGE2_MASK	BIT(1)
#define NOVATEK_PHY_POWER_C8_STAGE2_ON		0x0

#define NOVATEK_PHY_POWER_CC			0x0cc
#define NOVATEK_PHY_POWER_CC_MASK		BIT(0)
#define NOVATEK_PHY_POWER_CC_ON			0x0

#define NOVATEK_PHY_POWER_DC			0x0dc
#define NOVATEK_PHY_POWER_DC_MASK		BIT(0)
#define NOVATEK_PHY_POWER_DC_ON			0x1

#define NOVATEK_PHY_POWER_F8			0x0f8
#define NOVATEK_PHY_POWER_F8_MASK		BIT(7)
#define NOVATEK_PHY_POWER_F8_ON			0x1

#define NOVATEK_PHY_LED_TIMING			0x100
#define NOVATEK_PHY_LED_TIMING_DEFAULT		0x00000040

#define NOVATEK_PHY_BREAK_TIMER_CTRL		0x284
#define NOVATEK_PHY_BREAK_TIMER_CTRL_DEFAULT	0x000000c9

#define NOVATEK_PHY_BREAK_TIMER_LOW		0x288
#define NOVATEK_PHY_BREAK_TIMER_LOW_DEFAULT	0x00000053

#define NOVATEK_PHY_BREAK_TIMER_HIGH		0x28c
#define NOVATEK_PHY_BREAK_TIMER_HIGH_DEFAULT	0x00000007

#define NOVATEK_PHY_POWER_2E8			0x2e8
#define NOVATEK_PHY_POWER_2E8_MASK		BIT(0)
#define NOVATEK_PHY_POWER_2E8_ON		0x0

#define NOVATEK_PHY_TUNE_2F8			0x2f8
#define NA51089_PHY_TUNE_2F8_MASK		GENMASK(2, 0)
#define NA51089_PHY_TUNE_2F8_DEFAULT		0x3

#define NOVATEK_PHY_TUNE_354			0x354
#define NA51089_PHY_TUNE_354_MASK		BIT(7)
#define NA51089_PHY_TUNE_354_DEFAULT		0x1

#define NOVATEK_PHY_TUNE_358			0x358
#define NA51089_PHY_TUNE_358_MASK		GENMASK(5, 4)
#define NA51089_PHY_TUNE_358_DEFAULT		0x3

#define NOVATEK_ETH_PHY_REF_RATE		25000000

#define NA51089_PHY_REG_MAX			0x3fc

struct novatek_mdio_mux {
	void __iomem *base;
	void *mux_handle;
	struct device *mac;
	struct clk *phy_refclk;
	struct regmap *phy_regmap;
	struct reset_control *phy_reset;
	struct phy_device *phydev;
	struct notifier_block phy_notifier;
	phy_interface_t interface;
	/* System PM invalidates PHY state outside the parent MDIO bus lock. */
	struct mutex lock;
	bool initialized;
	bool removing;
	bool phy_detached;
	bool refclk_out;
};

static const struct regmap_config novatek_phy_regmap_config = {
	.reg_bits = 32,
	.reg_stride = 4,
	.val_bits = 32,
	.max_register = NA51089_PHY_REG_MAX,
	.cache_type = REGCACHE_NONE,
};

static int novatek_mdio_mux_phy_power_on(struct novatek_mdio_mux *mux)
{
	struct regmap *regmap = mux->phy_regmap;
	int ret;

	ret = regmap_update_bits(regmap, NOVATEK_PHY_POWER_F8,
				 NOVATEK_PHY_POWER_F8_MASK,
				 FIELD_PREP(NOVATEK_PHY_POWER_F8_MASK,
					    NOVATEK_PHY_POWER_F8_ON));
	if (ret)
		return ret;
	fsleep(20);

	ret = regmap_update_bits(regmap, NOVATEK_PHY_POWER_C8,
				 NOVATEK_PHY_POWER_C8_STAGE1_MASK,
				 FIELD_PREP(NOVATEK_PHY_POWER_C8_STAGE1_MASK,
					    NOVATEK_PHY_POWER_C8_STAGE1_ON));
	if (ret)
		return ret;
	fsleep(200);

	ret = regmap_update_bits(regmap, NOVATEK_PHY_POWER_C8,
				 NOVATEK_PHY_POWER_C8_STAGE2_MASK,
				 FIELD_PREP(NOVATEK_PHY_POWER_C8_STAGE2_MASK,
					    NOVATEK_PHY_POWER_C8_STAGE2_ON));
	if (ret)
		return ret;
	fsleep(250);

	ret = regmap_update_bits(regmap, NOVATEK_PHY_POWER_2E8,
				 NOVATEK_PHY_POWER_2E8_MASK,
				 FIELD_PREP(NOVATEK_PHY_POWER_2E8_MASK,
					    NOVATEK_PHY_POWER_2E8_ON));
	if (ret)
		return ret;

	ret = regmap_update_bits(regmap, NOVATEK_PHY_POWER_CC,
				 NOVATEK_PHY_POWER_CC_MASK,
				 FIELD_PREP(NOVATEK_PHY_POWER_CC_MASK,
					    NOVATEK_PHY_POWER_CC_ON));
	if (ret)
		return ret;

	ret = regmap_update_bits(regmap, NOVATEK_PHY_POWER_DC,
				 NOVATEK_PHY_POWER_DC_MASK,
				 FIELD_PREP(NOVATEK_PHY_POWER_DC_MASK,
					    NOVATEK_PHY_POWER_DC_ON));
	if (ret)
		return ret;

	return regmap_update_bits(regmap, NOVATEK_PHY_POWER_9C,
				  NOVATEK_PHY_POWER_9C_MASK,
				  FIELD_PREP(NOVATEK_PHY_POWER_9C_MASK,
					     NOVATEK_PHY_POWER_9C_ON));
}

static int novatek_mdio_mux_phy_init(struct novatek_mdio_mux *mux)
{
	static const struct reg_sequence sequence[] = {
		{ NOVATEK_PHY_BREAK_TIMER_LOW,
		  NOVATEK_PHY_BREAK_TIMER_LOW_DEFAULT },
		{ NOVATEK_PHY_BREAK_TIMER_HIGH,
		  NOVATEK_PHY_BREAK_TIMER_HIGH_DEFAULT },
		{ NOVATEK_PHY_BREAK_TIMER_CTRL,
		  NOVATEK_PHY_BREAK_TIMER_CTRL_DEFAULT },
		{ NOVATEK_PHY_LED_TIMING, NOVATEK_PHY_LED_TIMING_DEFAULT },
	};
	struct regmap *regmap = mux->phy_regmap;
	int ret;

	ret = reset_control_assert(mux->phy_reset);
	if (ret)
		return ret;
	fsleep(20000);

	ret = reset_control_deassert(mux->phy_reset);
	if (ret)
		return ret;
	fsleep(20000);

	ret = novatek_mdio_mux_phy_power_on(mux);
	if (ret)
		return ret;

	ret = regmap_update_bits(regmap, NOVATEK_PHY_TUNE_2F8,
				 NA51089_PHY_TUNE_2F8_MASK,
				 FIELD_PREP(NA51089_PHY_TUNE_2F8_MASK,
					    NA51089_PHY_TUNE_2F8_DEFAULT));
	if (ret)
		return ret;

	ret = regmap_update_bits(regmap, NOVATEK_PHY_TUNE_354,
				 NA51089_PHY_TUNE_354_MASK,
				 FIELD_PREP(NA51089_PHY_TUNE_354_MASK,
					    NA51089_PHY_TUNE_354_DEFAULT));
	if (ret)
		return ret;

	ret = regmap_update_bits(regmap, NOVATEK_PHY_TUNE_358,
				 NA51089_PHY_TUNE_358_MASK,
				 FIELD_PREP(NA51089_PHY_TUNE_358_MASK,
					    NA51089_PHY_TUNE_358_DEFAULT));
	if (ret)
		return ret;

	ret = regmap_update_bits(regmap, NOVATEK_PHY_RESET,
				 NOVATEK_PHY_RESET_SW_MASK,
				 FIELD_PREP(NOVATEK_PHY_RESET_SW_MASK,
					    NOVATEK_PHY_RESET_SW_ASSERT));
	if (ret)
		return ret;

	ret = regmap_multi_reg_write(regmap, sequence, ARRAY_SIZE(sequence));
	if (ret)
		return ret;
	fsleep(10000);

	ret = regmap_update_bits(regmap, NOVATEK_PHY_RESET,
				 NOVATEK_PHY_RESET_SW_MASK |
				 NOVATEK_PHY_RESET_PLL_WR_MASK,
				 FIELD_PREP(NOVATEK_PHY_RESET_SW_MASK,
					    NOVATEK_PHY_RESET_SW_DEASSERT) |
				 FIELD_PREP(NOVATEK_PHY_RESET_PLL_WR_MASK,
					    NOVATEK_PHY_RESET_PLL_WR_ENABLE));
	if (ret)
		return ret;
	mux->initialized = true;

	return 0;
}

static void novatek_mdio_mux_rmii_config(struct novatek_mdio_mux *mux)
{
	u32 value;

	if (mux->interface != PHY_INTERFACE_MODE_RMII)
		return;

	value = readl(mux->base + NOVATEK_ETH_IO_CTRL);
	value &= ~(NOVATEK_ETH_REF_SEL_MASK | NOVATEK_ETH_REF_OUT_MASK |
		   NOVATEK_ETH_REF_IN_MASK | NA51089_ETH_TXD_SRC_MASK);
	value |= FIELD_PREP(NOVATEK_ETH_REF_IN_MASK,
			    NOVATEK_ETH_REF_IN_ENABLE) |
		 FIELD_PREP(NA51089_ETH_TXD_SRC_MASK, NA51089_ETH_TXD_SRC_RMII);
	if (mux->refclk_out)
		value |= FIELD_PREP(NOVATEK_ETH_REF_SEL_MASK,
				    NOVATEK_ETH_REF_INTERNAL) |
			 FIELD_PREP(NOVATEK_ETH_REF_OUT_MASK,
				    NOVATEK_ETH_REF_OUT_ENABLE);
	else
		value |= FIELD_PREP(NOVATEK_ETH_REF_SEL_MASK,
				    NOVATEK_ETH_REF_EXTERNAL) |
			 FIELD_PREP(NOVATEK_ETH_REF_OUT_MASK,
				    NOVATEK_ETH_REF_OUT_DISABLE);
	writel(value, mux->base + NOVATEK_ETH_IO_CTRL);
}

static int novatek_mdio_mux_switch(int current_child, int desired_child,
				   void *data)
{
	struct novatek_mdio_mux *mux = data;
	u32 value;
	int ret = 0;

	if (desired_child != NOVATEK_ETH_PHY_SEL_INTERNAL &&
	    desired_child != NOVATEK_ETH_PHY_SEL_EXTERNAL)
		return -EINVAL;

	mutex_lock(&mux->lock);
	/* MAC reset can invalidate the mux core's cached child selection. */
	novatek_mdio_mux_rmii_config(mux);
	if (desired_child == NOVATEK_ETH_PHY_SEL_INTERNAL) {
		if (!mux->initialized) {
			ret = novatek_mdio_mux_phy_init(mux);
			if (ret) {
				reset_control_assert(mux->phy_reset);
				goto unlock;
			}
		}
		value = readl(mux->base + NOVATEK_ETH_DEBUG);
		value &= ~NOVATEK_ETH_PHY_ADDR_MASK;
		value |= FIELD_PREP(NOVATEK_ETH_PHY_ADDR_MASK,
				    NOVATEK_ETH_PHY_ADDR_INTERNAL);
		writel(value, mux->base + NOVATEK_ETH_DEBUG);
	}

	value = readl(mux->base + NOVATEK_ETH_CONTROL);
	value &= ~NOVATEK_ETH_PHY_SEL_MASK;
	value |= FIELD_PREP(NOVATEK_ETH_PHY_SEL_MASK, desired_child);
	if (desired_child == NOVATEK_ETH_PHY_SEL_EXTERNAL) {
		value &= ~NA51089_ETH_RMII_OUTPUT_PHASE_MASK;
		value |= FIELD_PREP(NA51089_ETH_RMII_OUTPUT_PHASE_MASK,
				    NA51089_ETH_RMII_OUTPUT_PHASE_RISING);
	}
	writel(value, mux->base + NOVATEK_ETH_CONTROL);

unlock:
	mutex_unlock(&mux->lock);
	return ret;
}

static int novatek_mdio_mux_phy_notify(struct notifier_block *nb,
				       unsigned long event, void *data)
{
	struct novatek_mdio_mux *mux;
	struct device *dev = data;
	struct net_device *ndev;
	struct phy_device *phydev;

	mux = container_of(nb, struct novatek_mdio_mux, phy_notifier);
	phydev = mux->phydev;
	if (dev != &phydev->mdio.dev || READ_ONCE(mux->removing))
		return NOTIFY_DONE;
	if (event != BUS_NOTIFY_UNBIND_DRIVER &&
	    event != BUS_NOTIFY_BOUND_DRIVER)
		return NOTIFY_DONE;

	/* Generic fallback binding and detach already hold RTNL. */
	if (phydev->is_genphy_driven)
		return NOTIFY_DONE;

	rtnl_lock();
	if (mux->removing)
		goto unlock;
	ndev = dev_get_drvdata(mux->mac);
	if (event == BUS_NOTIFY_UNBIND_DRIVER) {
		mux->phy_detached = true;
		netif_device_detach(ndev);
		dev_close(ndev);
	} else if (mux->phy_detached) {
		mux->phy_detached = false;
		netif_device_attach(ndev);
	}

unlock:
	rtnl_unlock();

	return NOTIFY_OK;
}

static void novatek_mdio_mux_unregister_notifier(void *data)
{
	struct novatek_mdio_mux *mux = data;

	bus_unregister_notifier(mux->phydev->mdio.dev.bus, &mux->phy_notifier);
	put_device(&mux->phydev->mdio.dev);
}

static int novatek_mdio_mux_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct novatek_mdio_mux *mux;
	struct device_node *np;
	struct mii_bus *parent_bus;
	struct net_device *ndev;
	void __iomem *phy_base;
	int ret;

	mux = devm_kzalloc(dev, sizeof(*mux), GFP_KERNEL);
	if (!mux)
		return -ENOMEM;
	mutex_init(&mux->lock);
	platform_set_drvdata(pdev, mux);

	mux->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(mux->base))
		return PTR_ERR(mux->base);

	phy_base = devm_platform_ioremap_resource(pdev, 1);
	if (IS_ERR(phy_base))
		return PTR_ERR(phy_base);
	mux->phy_regmap = devm_regmap_init_mmio(dev, phy_base,
						&novatek_phy_regmap_config);
	if (IS_ERR(mux->phy_regmap))
		return PTR_ERR(mux->phy_regmap);

	mux->phy_reset = devm_reset_control_get_exclusive(dev, "phy");
	if (IS_ERR(mux->phy_reset))
		return dev_err_probe(dev, PTR_ERR(mux->phy_reset),
				     "Failed to get internal PHY reset\n");

	mux->phy_refclk = devm_clk_get_optional(dev, "phy-ref");
	if (IS_ERR(mux->phy_refclk))
		return dev_err_probe(dev, PTR_ERR(mux->phy_refclk),
				     "Failed to get PHY reference clock\n");
	if (mux->phy_refclk &&
	    clk_get_rate(mux->phy_refclk) != NOVATEK_ETH_PHY_REF_RATE)
		return dev_err_probe(dev, -EINVAL,
				     "PHY reference clock must be 25 MHz\n");

	np = of_parse_phandle(dev->of_node, "mdio-parent-bus", 0);
	if (!np)
		return dev_err_probe(dev, -EINVAL, "Missing parent MDIO bus\n");
	parent_bus = of_mdio_find_bus(np);
	of_node_put(np);
	if (!parent_bus)
		return -EPROBE_DEFER;
	mux->mac = parent_bus->parent;
	if (!mux->mac ||
	    !of_device_is_compatible(mux->mac->of_node,
				     "novatek,na51089-dwmac")) {
		ret = -EINVAL;
		goto put_bus;
	}

	ret = of_get_phy_mode(mux->mac->of_node, &mux->interface);
	if (ret)
		goto put_bus;
	if (mux->interface != PHY_INTERFACE_MODE_MII &&
	    mux->interface != PHY_INTERFACE_MODE_RMII) {
		ret = -EINVAL;
		goto put_bus;
	}
	mux->refclk_out = of_property_read_bool(mux->mac->of_node,
						"novatek,rmii-refclk-out");
	if (mux->refclk_out && mux->interface != PHY_INTERFACE_MODE_RMII) {
		ret = -EINVAL;
		goto put_bus;
	}

	device_lock(mux->mac);
	if (!device_is_bound(mux->mac)) {
		ret = -EPROBE_DEFER;
		goto unlock_mac;
	}
	if (!device_link_add(dev, mux->mac, DL_FLAG_AUTOPROBE_CONSUMER)) {
		ret = -EINVAL;
		goto unlock_mac;
	}

	/* Switching precedes the parent MDIO operation's runtime PM get. */
	ret = pm_runtime_resume_and_get(mux->mac);
unlock_mac:
	device_unlock(mux->mac);
	if (ret)
		goto put_bus;

	/* Keep this clock on during sleep: the MAC resumes its PHY first. */
	ret = clk_prepare_enable(mux->phy_refclk);
	if (ret)
		goto put_runtime;

	novatek_mdio_mux_rmii_config(mux);

	ret = mdio_mux_init(dev, dev->of_node, novatek_mdio_mux_switch,
			    &mux->mux_handle, mux, parent_bus);
	if (ret)
		goto disable_clk;

	np = of_parse_phandle(mux->mac->of_node, "phy-handle", 0);
	mux->phydev = of_phy_find_device(np);
	of_node_put(np);
	if (!mux->phydev) {
		ret = -ENODEV;
		goto uninit_mux;
	}
	if (mux->phydev->mdio.bus->parent != dev) {
		ret = -EINVAL;
		put_device(&mux->phydev->mdio.dev);
		goto uninit_mux;
	}
	mux->phy_notifier.notifier_call = novatek_mdio_mux_phy_notify;
	ret = bus_register_notifier(mux->phydev->mdio.dev.bus,
				    &mux->phy_notifier);
	if (ret) {
		put_device(&mux->phydev->mdio.dev);
		goto uninit_mux;
	}
	ret = devm_add_action_or_reset(dev,
				       novatek_mdio_mux_unregister_notifier,
				       mux);
	if (ret)
		goto uninit_mux;

	rtnl_lock();
	ndev = dev_get_drvdata(mux->mac);
	if (!mux->phy_detached)
		netif_device_attach(ndev);
	rtnl_unlock();
	goto put_bus;

uninit_mux:
	mdio_mux_uninit(mux->mux_handle);
disable_clk:
	clk_disable_unprepare(mux->phy_refclk);
put_runtime:
	reset_control_assert(mux->phy_reset);
	pm_runtime_put(mux->mac);
put_bus:
	put_device(&parent_bus->dev);
	if (ret)
		return dev_err_probe(dev, ret,
				     "Failed to initialize MDIO mux\n");

	return 0;
}

static void novatek_mdio_mux_remove(struct platform_device *pdev)
{
	struct novatek_mdio_mux *mux = platform_get_drvdata(pdev);
	struct net_device *ndev = dev_get_drvdata(mux->mac);

	/* Block reopening the MAC while its child PHY devices are removed. */
	rtnl_lock();
	WRITE_ONCE(mux->removing, true);
	netif_device_detach(ndev);
	dev_close(ndev);
	rtnl_unlock();

	mdio_mux_uninit(mux->mux_handle);
	reset_control_assert(mux->phy_reset);
	clk_disable_unprepare(mux->phy_refclk);
	pm_runtime_put(mux->mac);
}

static int novatek_mdio_mux_suspend_late(struct device *dev)
{
	struct novatek_mdio_mux *mux = dev_get_drvdata(dev);
	struct net_device *ndev = dev_get_drvdata(mux->mac);

	/* MAC resume uses MDIO before the mux's normal resume callback. */
	mutex_lock(&mux->lock);
	if (!device_may_wakeup(mux->mac) || !ndev->ethtool->wol_enabled)
		mux->initialized = false;
	mutex_unlock(&mux->lock);

	return 0;
}

static const struct dev_pm_ops novatek_mdio_mux_pm_ops = {
	LATE_SYSTEM_SLEEP_PM_OPS(novatek_mdio_mux_suspend_late, NULL)
};

static const struct of_device_id novatek_mdio_mux_match[] = {
	{ .compatible = "novatek,na51089-mdio-mux" },
	{ }
};
MODULE_DEVICE_TABLE(of, novatek_mdio_mux_match);

static struct platform_driver novatek_mdio_mux_driver = {
	.probe = novatek_mdio_mux_probe,
	.remove = novatek_mdio_mux_remove,
	.driver = {
		.name = "novatek-mdio-mux",
		.of_match_table = novatek_mdio_mux_match,
		.pm = pm_sleep_ptr(&novatek_mdio_mux_pm_ops),
	},
};
module_platform_driver(novatek_mdio_mux_driver);

MODULE_AUTHOR("Yu-Tung Chang <mtwget@gmail.com>");
MODULE_DESCRIPTION("Novatek NA51089 MDIO multiplexer driver");
MODULE_LICENSE("GPL");
