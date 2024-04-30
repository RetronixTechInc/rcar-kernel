// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (C) 2022 Renesas Electronics Corporation
 */

#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/gpio/consumer.h>
#include <linux/gpio/driver.h>
#include <linux/i2c.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of_graph.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>

#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_bridge.h>
#include <drm/drm_dp_helper.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_of.h>
#include <drm/drm_panel.h>
#include <drm/drm_print.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_edid.h>

#include "max96789.h"

static unsigned int DEBUG_COLOR_PATTERN = 1;	/* 0: close, 1: MAX96789 */
module_param_named(patgen, DEBUG_COLOR_PATTERN, int, 0644);

static unsigned int PATTERN_TYPE = 1;			/* 1: gradient, 2: chessboard */
module_param_named(patgen_type, PATTERN_TYPE, int, 0644);

#define bridge_to_max96789_priv(b) \
	container_of(b, struct max96789_priv, bridge)

#define connector_to_max96789_priv(c) \
	container_of(c, struct max96789_priv, connector);

/* -----------------------------------------------------------------------------
 * test pattern
 */
static int max96789_patgen(struct max96789_priv *priv, int pat_flags)
{
	int ret = 0;
	u32 xres = 0, yres = 0;
	u32 hfp = 0, hbp = 0, hsa = 0, vfp = 0, vbp = 0, vsa = 0;
	xres = 1280;
	yres = 768;
	hbp	 = 6;
	hfp	 = 200;
	hsa	 = 14;
	vbp	 = 20;
	vfp	 = 20;
	vsa	 = 16;

	u32 vtotal 	= vfp + vsa + vbp + yres;
	u32 htotal 	= xres + hfp + hbp + hsa;
	u32 vs_high = vsa * htotal;
	u32 vs_low 	= (vfp + yres + vbp) * htotal;
	u32 hs_high = hsa;
	u32 hs_low 	= xres + hfp + hbp;
	u32 de_high = xres;
	u32 de_low 	= hfp + hsa + hbp;
	u32 de_cnt 	= yres;
	u32 v2h 	= (vsa + vbp) * htotal + hfp;		/* Set HS Delay */
	u32 v2d 	= v2h + hsa + hbp;					/* Set DE Delay */
	
	max96789_write_n(priv, MAX96789_VTX_X(2) , 3, 0);			/* VS delay */
	max96789_write_n(priv, MAX96789_VTX_X(11), 3, v2h);			/* HS delay */
	max96789_write_n(priv, MAX96789_VTX_X(20), 3, v2d);			/* DE delay */
	max96789_write_n(priv, MAX96789_VTX_X(5) , 3, vs_high); 
	max96789_write_n(priv, MAX96789_VTX_X(8) , 3, vs_low);
	max96789_write_n(priv, MAX96789_VTX_X(14), 2, hs_high);
	max96789_write_n(priv, MAX96789_VTX_X(16), 2, hs_low);
	max96789_write_n(priv, MAX96789_VTX_X(18), 2, vtotal);	
	max96789_write_n(priv, MAX96789_VTX_X(23), 2, de_high);		
	max96789_write_n(priv, MAX96789_VTX_X(25), 2, de_low);		
	max96789_write_n(priv, MAX96789_VTX_X(27), 2, de_cnt);		
	
	/* Generate VS, HS and DE in free-running mode. */
	max96789_write_reg(priv, MAX96789_VTX_X(0), 0xFB);
	
	/* pclk detect, select VS trigger edge */
	max96789_write_reg(priv, MAX96789_VTX_X(1), 0x01);

	/* Set gradient increment for gradient pattern. */
	max96789_write_reg(priv, MAX96789_VTX_X(30), 0x03);
	
	/* Set color for chessboard pattern. */
	max96789_write_n(priv, MAX96789_VTX_X(31), 3, 0xFF0000);
	max96789_write_n(priv, MAX96789_VTX_X(34), 3, 0x0000FF);
	max96789_write_n(priv, MAX96789_VTX_X(37), 3, 0x505050);
	
	/* Select pattern : 0x10 is gradient, 0x01 is chessboard, 
	 * 0x00 is pattern generator disabled - use video from the serializer input */
	max96789_write_reg(priv, MAX96789_VTX_X(29), pat_flags == 1 ? 0x02 : 0x01);

	return ret;
}

/* -----------------------------------------------------------------------------
 * chip initialize
 */
static void max96789_preinit(struct max96789_priv *priv)
{
	max96789_update_bits(priv, MAX96789_CTRL0, BIT(7), BIT(7));	
	usleep_range(10000, 20000);
	
	max96789_write_reg(priv, MAX96789_GPIO_A(0), 0x12);			// MFP0 TX
	
	max96789_update_bits(priv, MAX96789_REG3, 0x03, 0x00);		// RCLK 25MHz
	max96789_update_bits(priv, MAX96789_REG6, BIT(5), BIT(5));	// EN RCLK output
	
	max96789_write_reg(priv, MAX96789_REG5, 0xC0);				// ERRB output to GPIO
	usleep_range(10000, 20000);
	
	/* enable internal regulator for 1.2V VDD supply */
	max96789_update_bits(priv, MAX96789_CTRL0, BIT(2), BIT(2));	// REG_ENABLE = 1
	max96789_update_bits(priv, MAX96789_CTRL2, BIT(4), BIT(4));	// REG_MNL = 1
	
	/* I2C-to-I2C Slave Timeout Setting */
	// Fast-mode plus speed
	max96789_write_reg(priv, MAX96789_I2C_0, 0x06);
	// Timeout = 1ms, 397Kbps - set for I2C fast or fast-mode plus speed
	max96789_write_reg(priv, MAX96789_I2C_1, 0x76);
	
	
	////
	//max96789_write_reg(priv, 0x0602, BIT(5));
	
	
	// Link AB
	max96789_update_bits(priv, MAX96789_CTRL1, 0x05, 0x05);
	max96789_update_bits(priv, MAX96789_REG4, 0xF0, 0);
	
}

static void max96789_gmsl2_initial_setup(struct max96789_priv *priv)
{
	max96789_update_bits(priv, MAX96789_REG4, 0xF0, 0xF0);
	max96789_write_reg(priv, MAX96789_REG1, 0x08);
}

static int max96789_mipi_setup(struct max96789_priv *priv)
{
	// DSI port A selection for video pipeline X
	max96789_write_reg(priv, MAX96789_FRONTTOP_0, 0x5E);	
	
	// MIPI Rx 2x4 only A
	max96789_update_bits(priv, MAX96789_MIPI_RX0, 0x0F, 0x0C);
	
	// 4 data lanes
	max96789_update_bits(priv, MAX96789_MIPI_RX1, 0x33, 0x33);
	
	// PHY0 D0 is lane 0, PHY0 D1 is lane 1
	// PHY1 D0 is lane 2, PHY1 D1 is lane 3
	max96789_update_bits(priv, MAX96789_MIPI_RX2, 0xFF, 0xE4);
	
	max96789_update_bits(priv, MAX96789_MIPI_RX4, 0xFF, 0x70);
	
	max96789_update_bits(priv, MAX96789_MIPI_RX8, 0xFF, 0xC0);
	
	// ----DSI Controller 0
	// controller 0 dsi video mode
	max96789_update_bits(priv, MAX96789_MIPI_DSI0, 0xF8, 0x0D);
	
	//----------------
	// DE length in number of PCLK cycles, 768
	max96789_update_bits(priv, MAX96789_MIPI_DSI1, 0xFF, 0x00);
	max96789_update_bits(priv, MAX96789_MIPI_DSI2, 0x0F, 0x03);
	
	// HS pulse width in number of PCLK cycles, hsa	 = 14
	max96789_update_bits(priv, MAX96789_MIPI_DSI5, 0xFF, 0x0E);
	max96789_update_bits(priv, MAX96789_MIPI_DSI7, 0x0F, 0x00);
	
	// VS pulse width in number of PCLK cycles, vsa	 = 16
	max96789_update_bits(priv, MAX96789_MIPI_DSI6, 0xFF, 0x10);
	max96789_update_bits(priv, MAX96789_MIPI_DSI7, 0xF0, 0x00);
	
	// high-speed Rx timeout
	max96789_update_bits(priv, MAX96789_MIPI_DSI10, 0xFF, 0x00);
	max96789_update_bits(priv, MAX96789_MIPI_DSI11, 0xFF, 0x00);
	max96789_update_bits(priv, MAX96789_MIPI_DSI12, 0xFF, 0x00);
	
	// Low-power Tx timeout
	max96789_update_bits(priv, MAX96789_MIPI_DSI13, 0xFF, 0x00);
	max96789_update_bits(priv, MAX96789_MIPI_DSI14, 0xFF, 0x00);
	max96789_update_bits(priv, MAX96789_MIPI_DSI15, 0xFF, 0x00);
	//----------------
		
	// vc = 0
	max96789_update_bits(priv, MAX96789_MIPI_DSI8, 0xC4, 0x04);
	
	// DPI deskew
	max96789_update_bits(priv, MAX96789_MIPI_DSI36, 0xC3, 0xC2);
	
	// VFP = 20
	max96789_update_bits(priv, MAX96789_MIPI_DSI37, 0xFF, 0x14);	
	max96789_update_bits(priv, MAX96789_MIPI_DSI38, 0x0F, 0x00);
	
	// VBP = 20
	max96789_update_bits(priv, MAX96789_MIPI_DSI38, 0xF0, 0x14);	
	max96789_update_bits(priv, MAX96789_MIPI_DSI39, 0xFF, 0x00);
	
	// vertical active window size 768
	max96789_update_bits(priv, MAX96789_MIPI_DSI40, 0xFF, 0x00);
	max96789_update_bits(priv, MAX96789_MIPI_DSI41, 0xFF, 0x03);
	
	// HFP = 200
	max96789_update_bits(priv, MAX96789_MIPI_DSI42, 0xFF, 0xC8);
	max96789_update_bits(priv, MAX96789_MIPI_DSI43, 0x0F, 0x00);

	// HBP = 6
	max96789_update_bits(priv, MAX96789_MIPI_DSI43, 0xF0, 0x06);
	max96789_update_bits(priv, MAX96789_MIPI_DSI44, 0xFF, 0x00);
	
	// horizontal active window size 1280
	max96789_update_bits(priv, MAX96789_MIPI_DSI45, 0xFF, 0x00);
	max96789_update_bits(priv, MAX96789_MIPI_DSI46, 0x1F, 0x05);
	
	// Forces the MIPI receiver to start
	max96789_write_reg(priv, MAX96789_FRONTTOP_29, 0x81);
	
	usleep_range(5000, 10000);

	return 0;
}

static void max96789_pipe_override(struct max96789_priv *priv,
								   unsigned int pipe,
								   unsigned int dt, unsigned int vc)
{
	// EN virtual channel, bpp, datatype for video pipeline X
	max96789_update_bits(priv, MAX96789_FRONTTOP_20, 0xFF, 0xB8);
	
	//// EN virtual channel, bpp, datatype for video pipeline Y
	//max96789_update_bits(priv, MAX96789_FRONTTOP_20, 0xFF, 0xB8);
	
	// data type for video channel X
	max96789_update_bits(priv, MAX96789_FRONTTOP_25, 0x3F, 0x30);
	
	//// data type for video channel Y
	//max96789_update_bits(priv, MAX96789_FRONTTOP_26, 0x3F, 0x30);
	
	// video pipe crossbar (X->X, Y->Y, Z->Z, U->U)
	max96789_update_bits(priv, MAX96789_FRONTTOP_30, 0xFF, 0xE4);
}

static void max96789_gmsl2_link_pipe_setup(struct max96789_priv *priv, int link_n)
{
	//struct max96789_link *link = priv->link[link_n];
	int pipe = link_n; /* straight mapping */
	int dt = priv->dt; /* must come from imager */
	int in_vc = 0;
	int i;

	// AUTO_LINK = 0, link A is selected
	max96789_update_bits(priv, MAX96789_CTRL0, BIT(4), BIT(4));	
		
	// packets are transmitted over GMSL A
	max96789_write_reg(priv, MAX96789_TX0(0), 0xB8);
	max96789_write_reg(priv, MAX96789_TX3(0), 0x10);

	// start video pipe X from DSI port A
	max96789_write_reg(priv, MAX96789_FRONTTOP_9, 0x01);
	usleep_range(2000, 5000);
	
	max96789_pipe_override(priv, pipe, dt, in_vc);
	usleep_range(2000, 5000);
	
	// LINK A remote wake-up enable
	max96789_update_bits(priv, MAX96789_PWR4, 0x70, 0x10);
	
	// LINK A is enabled
	max96789_update_bits(priv, MAX96789_REG4, 0x50, 0x50);
	
	// CRC bpp from BPP bitfield
	max96789_write_reg(priv, MAX96789_VIDEO_TX0(0), 0x68);
	// bpp
	max96789_update_bits(priv, MAX96789_VIDEO_TX1(0), 0x3F, 0x18);
	// mask video with DE
	//max96789_update_bits(priv, MAX96789_VIDEO_TX6(0), BIT(6), BIT(6));
	
	//
}

static void max96789_reset_oneshot(struct max96789_priv *priv, int mask)
{
	int timeout;
	u8 val;

	// reset link
	max96789_update_bits(priv, MAX96789_CTRL0, 0x60, 0x60);	

	//mask &= 0x0f;
	//max96712_update_bits(priv, MAX96712_CTRL1, mask, mask);

	///* wait for one-shot bit self-cleared */
	//for (timeout = 0; timeout < 100; timeout++) 
	//{
		//max96712_read(priv, MAX96712_CTRL1, &val);
		//if (!(val & mask))
			//break;

		//mdelay(1);
	//}

	//if (val & mask)
		//dev_err(&priv->client->dev, "Failed reset oneshot 0x%x\n", mask);
}

static int max96789_gmsl2_get_link_lock(struct max96789_priv *priv, int link_n)
{
	u8 val;

	max96789_read(priv, MAX96789_CTRL3, &val);

	return !!(val & BIT(3));
}

static int max96789_gmsl2_reverse_channel_setup(struct max96789_priv *priv, int link_n)
{
	//struct max96712_link *link = priv->link[link_n];
	int des_addrs[] = {0x40, 0x42, 0x60, 0x62};	/* possible MAX9295 addresses on i2c bus */
	int timeout = 100;
	int ret = 0;
	int val = 0, i, j = 0;

	max96789_update_bits(priv, MAX96789_REG4, 0x50, 0x40);
	
	//max96789_reset_oneshot(priv, 0);

	/*
	 * wait the link to be established,
	 * indicated when status bit LOCKED goes high
	 */
	//for (; timeout > 0; timeout--) 
	//{
		//if (max96789_gmsl2_get_link_lock(priv, 0))
			//break;
		//mdelay(1);
	//}

	//if (!timeout) 
	//{
		//ret = -ETIMEDOUT;
		////goto out;
	//}

	for (i = 0; i < ARRAY_SIZE(des_addrs); i++) 
	{
		/* read deserializer ID */
		//__reg16_read(des_addrs[i], 0x000d, &val);
		//if (val == MAX9295A_ID || val == MAX9295B_ID) 
		//{
			//dev_dbg(&priv->client->dev, "ID val:0x%x>\n", val);
			//link->des_id = val;
			 ///* relocate deserizlizer on I2C bus */
			//__reg16_write(des_addrs[i], 0x0000, link->des_addr << 1);
			//usleep_range(2000, 2500);
			//j = i;
		//}
	}


	max96789_write_reg(priv, MAX96789_REG2, BIT(4));
	
	//priv->links_mask |= BIT(link_n);

	return ret;
}

static void max96789_initialize(struct max96789_priv *priv)
{
	printk("[%s (%d)]\n", __FUNCTION__, __LINE__);
	//struct max96789_source *source;
	int ret;
	int link = 0;

	max96789_preinit(priv);
	max96789_gmsl2_initial_setup(priv);
	max96789_mipi_setup(priv);	// MIPI-DSI

	max96789_gmsl2_link_pipe_setup(priv, 0);
	//max96789_gmsl2_reverse_channel_setup(priv, 0);
	
	///* Start all cameras. */
	//for_each_source(priv, source) 
	//{
		//max96712_gmsl2_link_pipe_setup(priv, link);
		//ret = max96712_gmsl2_reverse_channel_setup(priv, link);
		//if (ret < 0)
			//source->linkup = false;
		//else
			//source->linkup = true;
		//link++;
	//}
	//max96712_gmsl2_fsync_setup(priv);
	
	//max967XX_set_regs(priv, 0);
		
	if (DEBUG_COLOR_PATTERN == 1)
	{
		max96789_patgen(priv, PATTERN_TYPE);
	}
	return;
}

/* -----------------------------------------------------------------------------
 * DRM Bridge Operations
 */
static int max96789_bridge_attach(struct drm_bridge *bridge,
				enum drm_bridge_attach_flags flags)
{
	int ret;
	struct max96789_priv *priv = bridge_to_max96789_priv(bridge);
	struct mipi_dsi_host *host;
	struct mipi_dsi_device *dsi;
	const struct mipi_dsi_device_info info = { .type = "max96789_bridge",
						   .channel = 0,
						   .node = NULL,
						 };

	host = of_find_mipi_dsi_host_by_node(priv->host_node);
	if (!host) 
	{
		DRM_ERROR("failed to find dsi host\n");
		return -ENODEV;
	}

	dsi = mipi_dsi_device_register_full(host, &info);
	if (IS_ERR(dsi)) 
	{
		DRM_ERROR("failed to create dsi device\n");
		return PTR_ERR(dsi);
	}

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO;

	ret = mipi_dsi_attach(dsi);
	if (ret < 0) 
	{
		DRM_ERROR("failed to attach dsi to host\n");
		mipi_dsi_device_unregister(dsi);
		return ret;
	}
	
	max96789_initialize(priv);
	
	printk("[%s (%d)]\n", __FUNCTION__, __LINE__);
	return drm_bridge_attach(bridge->encoder, priv->next_bridge, bridge, flags);
}

static void max96789_bridge_enable(struct drm_bridge *bridge)
{
	struct max96789_priv *priv = bridge_to_max96789_priv(bridge);

	gpiod_set_value_cansleep(priv->gpiod_pwdn, 1);
	
	max96789_initialize(priv);
	
	printk("[%s (%d)]\n", __FUNCTION__, __LINE__);
}

static void max96789_bridge_disable(struct drm_bridge *bridge)
{
	struct max96789_priv *priv = bridge_to_max96789_priv(bridge);

	gpiod_set_value_cansleep(priv->gpiod_pwdn, 0);
}

static const struct drm_bridge_funcs max96789_bridge_funcs = {
	.attach = max96789_bridge_attach,
	.enable = max96789_bridge_enable,
	.disable = max96789_bridge_disable,
};

//static const struct regmap_config max96789_i2c_regmap = {
	//.reg_bits = 16,
	//.val_bits = 8,
	//.max_register = 0x1f00,
//};

static int max96789_bridge_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct max96789_priv *priv;
	int ret;

	priv = devm_kzalloc(&client->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	dev_set_drvdata(dev, priv);
	priv->dev = dev;

	//priv->regmap = devm_regmap_init_i2c(client, &max96789_i2c_regmap);
	//if (IS_ERR(priv->regmap))
		//return PTR_ERR(priv->regmap);

	priv->gpiod_pwdn = devm_gpiod_get_optional(&client->dev, "enable",
						   GPIOD_OUT_HIGH);
	if (IS_ERR(priv->gpiod_pwdn))
		return PTR_ERR(priv->gpiod_pwdn);

	gpiod_set_consumer_name(priv->gpiod_pwdn, "max96789-pwdn");

	if (priv->gpiod_pwdn)
		usleep_range(4000, 5000);

	priv->client = client;
	priv->dt = MIPI_DT_RGB888;
	i2c_set_clientdata(client, priv);
	
	//max96789_initialize(priv);

	priv->host_node = of_graph_get_remote_node(priv->dev->of_node
				, 0, 0);
	priv->bridge.driver_private = priv;
	priv->bridge.funcs = &max96789_bridge_funcs;
	priv->bridge.of_node = priv->dev->of_node;
	drm_bridge_add(&priv->bridge);
	
	ret = drm_of_find_panel_or_bridge(priv->dev->of_node, 1, 0,
				NULL, &priv->next_bridge);
	
	if (ret) 
	{
		DRM_ERROR("could not find bridge node\n");
		//return ret;
	}
	
	// ------------- debug
	u8 vddbad = 0;
	max96789_read(priv, MAX96789_PWR0, &vddbad);
	printk("[%s (%d)]: vdd info = %d\n", __FUNCTION__, __LINE__, vddbad);
	
	u8 link_status = 0;
	max96789_read(priv, MAX96789_REG15, &link_status);
	printk("[%s (%d)]: link_status = %d\n", __FUNCTION__, __LINE__, link_status);
	
	u8 pclk_det = 0;
	max96789_read(priv, MAX96789_VTX_X(1), &pclk_det);
	if (pclk_det == 1)
	{
		printk("[%s (%d)]: PCLK is not detected\n", __FUNCTION__, __LINE__);
	}
	else
	{
		printk("[%s (%d)]: pclk_det = %d\n", __FUNCTION__, __LINE__, pclk_det);
	}
	

	u8 hs_vs_det = 0;
	max96789_read(priv, MAX96789_HS_VS_X, &hs_vs_det);
	if (hs_vs_det == 3)
	{
		printk("[%s (%d)]: HS VS DE are not detected\n", __FUNCTION__, __LINE__);
	}
	else
	{
		printk("[%s (%d)]: hs_vs_det = %d\n", __FUNCTION__, __LINE__, hs_vs_det);
	}
	
	u8 dsi_contr_0_status = 0;
	max96789_read(priv, MAX96789_MIPI_DSI32, &dsi_contr_0_status);
	printk("[%s (%d)]: dsi_contr_0_status = %d\n", __FUNCTION__, __LINE__, dsi_contr_0_status);
	
	return 0;
}

static int max96789_bridge_remove(struct i2c_client *client)
{
	return 0;
}

static const struct of_device_id max96789_bridge_match_table[] = {
	{.compatible = "maxim,max96789"},
	{},
};
MODULE_DEVICE_TABLE(of, max96789_bridge_match_table);

static struct i2c_driver max96789_bridge_driver = {
	.driver = {
		.name = "maxim-max96789",
		.of_match_table = max96789_bridge_match_table,
	},
	.probe_new = max96789_bridge_probe,
	.remove = max96789_bridge_remove,
};
module_i2c_driver(max96789_bridge_driver);

MODULE_DESCRIPTION("max96789 driver");
MODULE_LICENSE("GPL v2");
