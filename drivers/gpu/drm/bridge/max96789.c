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
#include <drm/drm_crtc_helper.h>
#include <drm/drm_connector.h>
#include "max96789.h"

static unsigned int SER_PATTERN_SEL = 0x02;		/* 0x02: gradient, 0x01: chessboard, 0x00: OFF */
module_param_named(ser_patgen_select, SER_PATTERN_SEL, int, 0644);

#define bridge_to_max96789_priv(b) \
	container_of(b, struct max96789_priv, bridge)

#define connector_to_max96789_priv(c) \
	container_of(c, struct max96789_priv, connector);

/* -----------------------------------------------------------------------------
 * DEBUG
 */
 
static void DEBUG_INFO(struct max96789_priv *priv)
{
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
}
 
 
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
	
	/* Select pattern : 0x02 is gradient, 0x01 is chessboard, 
	 * 0x00 is pattern generator disabled - use video from the serializer input */
	max96789_write_reg(priv, MAX96789_VTX_X(29), pat_flags == 1 ? 0x02 : 0x01);

	return ret;
}

/* -----------------------------------------------------------------------------
 * chip initialize
 */
static void max96789_preinit(struct max96789_priv *priv)
{
	max96789_write_reg(priv, 0x20F5, 0x00);				// HPD
	max96789_write_reg(priv, MAX96789_CTRL0, 0x11);		// AUTO_LINK = 1
	
	/* I2C-to-I2C Slave Timeout Setting */
	max96789_write_reg(priv, MAX96789_I2C_0, 0x06);		// Fast-mode plus speed
	max96789_write_reg(priv, MAX96789_I2C_1, 0x56);		// Timeout = 32ms, 397Kbps
}

static int max96789_mipi_link_pipe_setup(struct max96789_priv *priv)
{
	// DSI port A selection for video pipeline X, Clock Select
	max96789_write_reg(priv, MAX96789_FRONTTOP_0, 0x5E);	
	
	// Set Stream for DSI Port A
	max96789_write_reg(priv, MAX96789_TX3(0), 0x10);
	
	// start video pipe X from DSI port A
	max96789_write_reg(priv, MAX96789_FRONTTOP_9, 0x01);
	
	// set MIPI port a mapping, PHY01 to port A
	max96789_write_reg(priv, MAX96789_MIPI_RX2, 0x4E);
	
	//// set MIPI port a mapping, PHY23 to port B
	//max96789_write_reg(priv, MAX96789_MIPI_RX3, 0xE4);
	
	// 4 data lanes
	max96789_write_reg(priv, MAX96789_MIPI_RX1, 0x33);
	
	// MIPI Rx 2x4 only A
	max96789_write_reg(priv, MAX96789_MIPI_RX0, 0x04);
	
	// video pipe crossbar (X->X, Y->Y, Z->Z, U->U)
	max96789_write_reg(priv, MAX96789_FRONTTOP_30, 0xE4);
	
	usleep_range(2000, 5000);

	return 0;
}

static void max96789_gmsl2_initial_setup(struct max96789_priv *priv)
{
	// coax drive
	max96789_write_reg(priv, MAX96789_CTRL1, 0x0F);
			
	// TX_RATE 6 Gbps
	max96789_write_reg(priv, MAX96789_REG1, 0x08);
	
	// Link A: enable, link B: disable 
	max96789_write_reg(priv, MAX96789_REG4, 0x50);
	
	usleep_range(2000, 5000);
}

static void max96789_pipe_override(struct max96789_priv *priv)
{
	// EN virtual channel, bpp, datatype for video pipeline X
	max96789_write_reg(priv, MAX96789_FRONTTOP_20, 0x98);
	
	// data type for video channel X
	max96789_write_reg(priv, MAX96789_FRONTTOP_25, 0x24);
	
	// EN virtual channel, bpp, datatype for video pipeline Y
	max96789_write_reg(priv, MAX96789_FRONTTOP_21, 0x98);
	
	// data type for video channel Y
	max96789_write_reg(priv, MAX96789_FRONTTOP_26, 0x24);
	
	usleep_range(2000, 5000);
}

static void max96789_video_timing(struct max96789_priv *priv)
{
	max96789_write_reg(priv, MAX96789_MIPI_DSI5, 	0x0E);	// HSYNC_WIDTH_L
	max96789_write_reg(priv, MAX96789_MIPI_DSI6,	0x10);	// VSYNC_WIDTH_L
	max96789_write_reg(priv, MAX96789_MIPI_DSI7, 	0x00);	// HSYNC_WIDTH_H / VSYNC_WIDTH_H
	max96789_write_reg(priv, MAX96789_MIPI_DSI37, 	0x14);	// VFP_L
	max96789_write_reg(priv, MAX96789_MIPI_DSI38, 	0x40);	// VFP_H / VBP_L
	max96789_write_reg(priv, MAX96789_MIPI_DSI39, 	0x01);	// VBP_H
	max96789_write_reg(priv, MAX96789_MIPI_DSI40, 	0x00);	// VRES_L
	max96789_write_reg(priv, MAX96789_MIPI_DSI41, 	0x03);	// VRES_H
	max96789_write_reg(priv, MAX96789_MIPI_DSI42, 	0xC8);	// HFP_L
	max96789_write_reg(priv, MAX96789_MIPI_DSI43, 	0x60);	// HFP_H / HBP_L
	max96789_write_reg(priv, MAX96789_MIPI_DSI44, 	0x00);	// HBP_H
	max96789_write_reg(priv, MAX96789_MIPI_DSI45, 	0x00);	// HRES_L
	max96789_write_reg(priv, MAX96789_MIPI_DSI46, 	0x05);	// HRES_H
	max96789_write_reg(priv, MAX96789_MIPI_DSI36, 	0xC1);	// FIFO / DESKEW_EN
	
	usleep_range(2000, 5000);
}

static int max96789_gmsl2_get_link_lock(struct max96789_priv *priv)
{
	u8 val;
	max96789_read(priv, MAX96789_CTRL3, &val);

	return !!(val & BIT(3));
}

static void max96789_reset_oneshot(struct max96789_priv *priv, u8 mask, u8 bits)
{
	int timeout;
	u8 val;

	max96789_update_bits(priv, MAX96789_CTRL0, mask, bits);
				       
	/* wait for one-shot bit self-cleared */
	for (timeout = 0; timeout < 100; timeout++) 
	{
		max96789_read(priv, MAX96789_CTRL0, &val);
		//if (!(val & mask))
			//break;

		mdelay(1);
	}

	//if (val & mask)
		//dev_err(&priv->client->dev, "Failed reset oneshot 0x%x\n", mask);
}

static int max96789_gmsl2_reverse_channel_setup(struct max96789_priv *priv, int link_n)
{
	printk("[%s (%d)]\n", __FUNCTION__, __LINE__);
	struct max96789_link *link = priv->link[link_n];
	/* possible MAX96776 addresses on i2c bus */
	int des_addrs[] = {0x3A};	
	int timeout = 100;
	int ret = 0;
	int val = 0, i, j = 0;

	max96789_reset_oneshot(priv, 0x20, BIT(5));

	/*
	 * wait the link to be established,
	 * indicated when status bit LOCKED goes high
	 */
	for (; timeout > 0; timeout--) 
	{
		if (max96789_gmsl2_get_link_lock(priv))
			break;
		mdelay(1);
	}

	if (!timeout) 
	{
		ret = -ETIMEDOUT;
		goto out;
	}

	for (i = 0; i < ARRAY_SIZE(des_addrs); i++) 
	{
		/* read de-serializer ID */
		__reg16_read(des_addrs[i], 0x000d, &val);					
		if (val == MAX96776_ID || val == MAX96778_ID) 
		{
			printk("[%s (%d)]: des_addrs[%d]: 0x%X, chip_id = 0x%X\n", __FUNCTION__, __LINE__, i, des_addrs[i], val);	
			dev_dbg(&priv->client->dev, "ID val:0x%x>\n", val);
			link->des_id = val;
			///* relocate de-serizlizer on I2C bus */
			//__reg16_write(des_addrs[i], 0x0000, link->des_addr << 1);	
			usleep_range(2000, 2500);
			j = i;
		}
	}
	
out:
	dev_info(&priv->client->dev, "link%d %s %sat 0x%X (0x%X) %s\n",
			 link_n, chip_name(link->des_id),
			 ret == -EADDRINUSE ? "already " : "",
			 link->des_addr, des_addrs[j], ret == -ETIMEDOUT ?
			 "not found: timeout GMSL2 link establish" : "");
	return ret;
}

static void max96789_initialize(struct max96789_priv *priv)
{
	printk("[%s (%d)]\n", __FUNCTION__, __LINE__);
	int ret;

	max96789_preinit(priv);
	
	max96789_mipi_link_pipe_setup(priv);
	
	max96789_gmsl2_initial_setup(priv);
	
	max96789_pipe_override(priv);

	//max96789_video_timing(priv);
		
	if (DEBUG_COLOR_PATTERN == 1)
	{
		max96789_patgen(priv, SER_PATTERN_SEL);
	}
	
	usleep_range(2000, 5000);
}

/* -----------------------------------------------------------------------------
 * max96776
 */
static int max96776_sensor_set_regs(struct max96789_priv *priv, u32 link_nr)
{
	int ret;
	struct max96789_link *link;

	link = priv->link[link_nr];

	max96776_write_reg(link, 0x0010, 0x21);	/* SW reset */
	msleep(200);

	/* Program the camera sensor initial configuration. */
	ret = max96776_set_regs(link, configuretable_vc0113,
							ARRAY_SIZE(configuretable_vc0113));
	msleep(200);
	return ret;
}

/* -----------------------------------------------------------------------------
 * DRM Bridge Operations
 */
 static const struct drm_connector_funcs max96789_connector_funcs = {
	.dpms = drm_helper_connector_dpms,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = drm_connector_cleanup,
	.reset = drm_atomic_helper_connector_reset,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static struct drm_connector_helper_funcs max96789_connector_helper_funcs = {
	.get_modes = max96789_connector_get_modes,
	.best_encoder = max96789_connector_best_encoder,
};

static int max96789_connector_get_modes(struct drm_connector *connector)
{
	int count;
	count = drm_add_modes_noedid(connector, 8192, 8192);
	drm_set_preferred_mode(connector, 1280, 768);
	return count;
}

static struct drm_encoder *max96789_connector_best_encoder(struct drm_connector *connector)
{
	struct max96789_priv *priv = connector_to_max96789_priv(connector);
	return priv->bridge.encoder;
}

static int max96789_bridge_attach(struct drm_bridge *bridge,
				enum drm_bridge_attach_flags flags)
{
	printk("[%s (%d)]\n", __FUNCTION__, __LINE__);
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
	
	ret = drm_connector_init(bridge->dev, &priv->connector,
							 &max96789_connector_funcs,
							 DRM_MODE_CONNECTOR_VIRTUAL);
	if (ret) 
	{
		DRM_ERROR("Failed to initialize connector with drm\n");
		return ret;
	}	
	drm_connector_helper_add(&priv->connector, 
							 &max96789_connector_helper_funcs);
	drm_connector_attach_encoder(&priv->connector, bridge->encoder);

	//max96789_initialize(priv);
	
	printk("[%s (%d)]\n", __FUNCTION__, __LINE__);
	return drm_bridge_attach(bridge->encoder, priv->next_bridge, bridge, flags);
}

static void max96789_bridge_enable(struct drm_bridge *bridge)
{
	struct max96789_priv *priv = bridge_to_max96789_priv(bridge);

	gpiod_set_value_cansleep(priv->gpiod_pwdn, 1);
	
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
	struct max96789_priv *priv;
	struct device *dev = &client->dev;
	struct device_node *np = client->dev.of_node;
	int ret;
	int addrs[3];

	priv = devm_kzalloc(&client->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	int i;
	for (i = 0; i < MAX96789_NUM_GMSL; i++) 
	{
		priv->link[i] = devm_kzalloc(&client->dev, sizeof(*priv->link[i]), GFP_KERNEL);
		if (!priv->link[i])
			return -ENOMEM;
	}
	
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
	of_property_read_u32_array(np, "reg", addrs, ARRAY_SIZE(addrs));

	for (i = 0; i < MAX96789_NUM_GMSL; i++) 
	{
		priv->link[i]->des_addr = addrs[i+1];
		//priv->link[i]->out_vc = i;				
		priv->link[i]->client = i2c_new_dummy_device(client->adapter, addrs[i+1]);
	}
	i2c_set_clientdata(client, priv);

	priv->host_node = of_graph_get_remote_node(priv->dev->of_node
				, 0, 0);
	priv->bridge.driver_private = priv;
	priv->bridge.funcs = &max96789_bridge_funcs;
	priv->bridge.of_node = priv->dev->of_node;
	drm_bridge_add(&priv->bridge);
	
	ret = drm_of_find_panel_or_bridge(priv->dev->of_node, 1, 0,
			NULL, &priv->next_bridge);
	
	//ret = drm_of_find_panel_or_bridge(priv->dev->of_node, 1, 0,
				//&priv->next_panel, NULL);
	
	if (ret) 
	{
		DRM_ERROR("could not find bridge node\n");
		//return ret;
	}
	
	max96789_initialize(priv);
	
	for (i = 0; i < MAX96789_NUM_GMSL; i++)
	{
		int link_n = i;
		int ret = 0;
		ret = max96789_gmsl2_reverse_channel_setup(priv, link_n);
		
		max96776_sensor_set_regs(priv, link_n);
		
	}

	// ------------- debug
	DEBUG_INFO(priv);
	
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
