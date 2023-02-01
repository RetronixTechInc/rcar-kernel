/*
 * ON Semiconductor AP020X-AR023X sensor camera driver
 *
 * Copyright (C) 2020 Cogent Embedded, Inc.
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/delay.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of_graph.h>
#include <linux/videodev2.h>

#include <media/soc_camera.h>
#include <media/v4l2-common.h>
#include <media/v4l2-ctrls.h>

#include "ar0233.h"
#include "../gmsl/max9295.h"
#include "../gmsl/common.h"

static const int ar0233_i2c_addr[] = {0x10};

#define AR0233_PID_REG		0x3000
#define AR0233_REV_REG		0x0058

#define AR0233_PID		0x0956


//#define ar0233_MEDIA_BUS_FMT	MEDIA_BUS_FMT_SRGGB10_1X10
#define ar0233_MEDIA_BUS_FMT	MEDIA_BUS_FMT_SRGGB12_1X12

struct ar0233_priv {
	struct v4l2_subdev		sd;
	struct v4l2_ctrl_handler	hdl;
	struct media_pad		pad;
	struct v4l2_rect		rect;
	int				max_width;
	int				max_height;
	int				init_complete;
	u8				id[6];
	int				exposure;
	int				gain;
	int				autogain;
	/* serializer */
	int				ser_addr;
};

static inline struct ar0233_priv *to_ar0233(const struct i2c_client *client)
{
	return container_of(i2c_get_clientdata(client), struct ar0233_priv, sd);
}

static inline struct v4l2_subdev *to_sd(struct v4l2_ctrl *ctrl)
{
	return &container_of(ctrl->handler, struct ar0233_priv, hdl)->sd;
}

static int max9295_set_regs(struct i2c_client *client, const struct max9295_reg *regs, int nr_regs)
{
	printk(KERN_ERR "[ RTX-%s-%d ] nr_regs = %d, ", __func__, __LINE__, nr_regs);
	
	struct ar0233_priv *priv = to_ar0233(client);
	int i = 0;

	for (i = 0; i < nr_regs; i++) {
		if (regs[i].reg == MAX9295_TABLE_END) {
			mdelay(1000);
			continue;
		}
		
		//printk(KERN_ERR "[ RTX-%s-%d ] regs[%d].reg = 0x%X ; regs[%d].val = 0x%X\n", __func__, __LINE__, i, regs[i].reg, i, regs[i].val);
		reg16_write_addr(client, priv->ser_addr, regs[i].reg, regs[i].val);
	}

	return 0;
}

static int ar0233_set_regs(struct i2c_client *client,
			  const struct ar0233_reg *regs, int nr_regs)
{
	int i;

	for (i = 0; i < nr_regs; i++) {
		if (regs[i].reg == AR0233_DELAY) {
			mdelay(regs[i].val);
			continue;
		}
		
		//printk(KERN_ERR "[ RTX-%s-%d ] regs[%d].reg = 0x%X ; regs[%d].val = 0x%X\n", __func__, __LINE__, i, regs[i].reg, i, regs[i].val);
		reg16_write16(client, regs[i].reg, regs[i].val);
	}

	return 0;
}

static u16 ar0233_ar023x_read(struct i2c_client *client, u16 addr)
{
	u16 reg_val = 0;

	reg16_write16(client, 0x0040, 0x8d00);
	usleep_range(1000, 1500); /* wait 1000 us */
	reg16_write16(client, 0xfc00, addr);
	reg16_write16(client, 0xfc02, 0x0200); /* 2 bytes */
	reg16_write16(client, 0x0040, 0x8d05);
	usleep_range(1000, 1500); /* wait 1000 us */
	reg16_write16(client, 0x0040, 0x8d08);
	usleep_range(1000, 1500); /* wait 1000 us */
	reg16_read16(client, 0xfc00, &reg_val);
	reg16_write16(client, 0x0040, 0x8d02);
	usleep_range(1000, 1500); /* wait 1000 us */

	return reg_val;
}

static void ar0233_ar023x_write(struct i2c_client *client, u16 addr, u16 val)
{
	reg16_write16(client, 0x0040, 0x8d00);
	usleep_range(1000, 1500); /* wait 1000 us */
	reg16_write16(client, 0xfc00, addr);
	reg16_write16(client, 0xfc02, 0x0200 | (val >> 8)); /* 2 bytes */
	reg16_write16(client, 0xfc04, (val & 0xff) << 8);
	reg16_write16(client, 0x0040, 0x8d06);
	usleep_range(1000, 1500); /* wait 1000 us */
	reg16_write16(client, 0x0040, 0x8d08);
	usleep_range(1000, 1500); /* wait 1000 us */
	reg16_write16(client, 0x0040, 0x8d02);
	usleep_range(1000, 1500); /* wait 1000 us */
}

static void ar0233_otp_id_read(struct i2c_client *client)
{
	struct ar0233_priv *priv = to_ar0233(client);
	int i;

	/* read camera id from ar023x OTP memory */
	ar0233_ar023x_write(client, 0x3054, 0x400);
	ar0233_ar023x_write(client, 0x304a, 0x110);
	usleep_range(25000, 25500); /* wait 25 ms */

	for (i = 0; i < 6; i += 2) {
		u16 val = 0;
		/* first 4 bytes are equal on all ar023x */
		val = ar0233_ar023x_read(client, 0x3800 + i + 4);
		priv->id[i]     = val >> 8;
		priv->id[i + 1] = val & 0xff;
	}
}

static int ar0233_s_stream(struct v4l2_subdev *sd, int enable)
{
	return 0;
}

static int ar0233_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *mf = &format->format;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ar0233_priv *priv = to_ar0233(client);

	if (format->pad)
		return -EINVAL;

	mf->width = priv->rect.width;
	mf->height = priv->rect.height;
	mf->code = ar0233_MEDIA_BUS_FMT;
	mf->colorspace = V4L2_COLORSPACE_SMPTE170M;
	mf->field = V4L2_FIELD_NONE;

	return 0;
}

static int ar0233_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *mf = &format->format;

	mf->code = ar0233_MEDIA_BUS_FMT;
	mf->colorspace = V4L2_COLORSPACE_SMPTE170M;
	mf->field = V4L2_FIELD_NONE;

	if (format->which == V4L2_SUBDEV_FORMAT_TRY)
		cfg->try_fmt = *mf;

	return 0;
}

static int ar0233_enum_mbus_code(struct v4l2_subdev *sd,
				struct v4l2_subdev_pad_config *cfg,
				struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->pad || code->index > 0)
		return -EINVAL;

	code->code = ar0233_MEDIA_BUS_FMT;

	return 0;
}

static int ar0233_get_edid(struct v4l2_subdev *sd, struct v4l2_edid *edid)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ar0233_priv *priv = to_ar0233(client);

	ar0233_otp_id_read(client);

	memcpy(edid->edid, priv->id, 6);

	edid->edid[6] = 0xff;
	edid->edid[7] = client->addr;
	edid->edid[8] = AR0233_PID >> 8;
	edid->edid[9] = AR0233_PID & 0xff;

	return 0;
}

static int ar0233_set_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_pad_config *cfg,
				struct v4l2_subdev_selection *sel)
{
	struct v4l2_rect *rect = &sel->r;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ar0233_priv *priv = to_ar0233(client);

	if (sel->which != V4L2_SUBDEV_FORMAT_ACTIVE ||
	    sel->target != V4L2_SEL_TGT_CROP)
		return -EINVAL;

	rect->left = ALIGN(rect->left, 2);
	rect->top = ALIGN(rect->top, 2);
	rect->width = ALIGN(rect->width, 2);
	rect->height = ALIGN(rect->height, 2);

	if ((rect->left + rect->width > priv->max_width) ||
	    (rect->top + rect->height > priv->max_height))
		*rect = priv->rect;

	priv->rect.left = rect->left;
	priv->rect.top = rect->top;
	priv->rect.width = rect->width;
	priv->rect.height = rect->height;

	return 0;
}

static int ar0233_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_pad_config *cfg,
				struct v4l2_subdev_selection *sel)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ar0233_priv *priv = to_ar0233(client);

	if (sel->which != V4L2_SUBDEV_FORMAT_ACTIVE)
		return -EINVAL;

	switch (sel->target) {
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = priv->max_width;
		sel->r.height = priv->max_height;
		return 0;
	case V4L2_SEL_TGT_CROP_DEFAULT:
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = priv->max_width;
		sel->r.height = priv->max_height;
		return 0;
	case V4L2_SEL_TGT_CROP:
		sel->r = priv->rect;
		return 0;
	default:
		return -EINVAL;
	}
}

static int ar0233_g_mbus_config(struct v4l2_subdev *sd,
				struct v4l2_mbus_config *cfg)
{
	cfg->flags = V4L2_MBUS_CSI2_1_LANE | V4L2_MBUS_CSI2_CHANNEL_0 |
		     V4L2_MBUS_CSI2_CONTINUOUS_CLOCK;
	cfg->type = V4L2_MBUS_CSI2_DPHY;

	return 0;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int ar0233_g_register(struct v4l2_subdev *sd,
			    struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;
	u16 val = 0;

	ret = reg16_read16(client, (u16)reg->reg, &val);
	if (ret < 0)
		return ret;

	reg->val = val;
	reg->size = sizeof(u16);

	return 0;
}

static int ar0233_s_register(struct v4l2_subdev *sd,
			     const struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	return reg16_write16(client, (u16)reg->reg, (u16)reg->val);
}
#endif

static struct v4l2_subdev_core_ops ar0233_core_ops = {
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register = ar0233_g_register,
	.s_register = ar0233_s_register,
#endif
};

static int ar0233_change_config(struct i2c_client *client)
{
	reg16_write16(client, 0x098e, 0x7c00);
	usleep_range(1000, 1500); /* wait 1 ms */
	reg16_write16(client, 0xfc00, 0x2800);
	reg16_write16(client, 0x0040, 0x8100);

	return 0;
}

static int ar0233_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = to_sd(ctrl);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ar0233_priv *priv = to_ar0233(client);
	int ret = -EINVAL;
	u16 val = 0;

	if (!priv->init_complete)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_BRIGHTNESS:
	case V4L2_CID_CONTRAST:
	case V4L2_CID_SATURATION:
	case V4L2_CID_HUE:
	case V4L2_CID_GAMMA:
	case V4L2_CID_SHARPNESS:
	case V4L2_CID_AUTOGAIN:
	case V4L2_CID_GAIN:
	case V4L2_CID_EXPOSURE:
		break;
	case V4L2_CID_HFLIP:
		reg16_read16(client, 0xc846, &val);
		if (ctrl->val)
			val |= 0x01;
		else
			val &= ~0x01;
		reg16_write16(client, 0xc846, val);
		ret = ar0233_change_config(client);
		break;
	case V4L2_CID_VFLIP:
		reg16_read16(client, 0xc846, &val);
		if (ctrl->val)
			val |= 0x02;
		else
			val &= ~0x02;
		reg16_write16(client, 0xc846, val);
		ret = ar0233_change_config(client);
		break;
	case V4L2_CID_MIN_BUFFERS_FOR_CAPTURE:
		ret = 0;
		break;
	}

	return ret;
}

static const struct v4l2_ctrl_ops ar0233_ctrl_ops = {
	.s_ctrl = ar0233_s_ctrl,
};

static struct v4l2_subdev_video_ops ar0233_video_ops = {
	.s_stream	= ar0233_s_stream,
	.g_mbus_config	= ar0233_g_mbus_config,
};

static const struct v4l2_subdev_pad_ops ar0233_subdev_pad_ops = {
	.get_edid	= ar0233_get_edid,
	.enum_mbus_code	= ar0233_enum_mbus_code,
	.get_selection	= ar0233_get_selection,
	.set_selection	= ar0233_set_selection,
	.get_fmt	= ar0233_get_fmt,
	.set_fmt	= ar0233_set_fmt,
};

static struct v4l2_subdev_ops ar0233_subdev_ops = {
	.core	= &ar0233_core_ops,
	.video	= &ar0233_video_ops,
	.pad	= &ar0233_subdev_pad_ops,
};

static ssize_t ar0233_otp_id_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(to_i2c_client(dev));
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ar0233_priv *priv = to_ar0233(client);

	ar0233_otp_id_read(client);

	return snprintf(buf, 32, "%02x:%02x:%02x:%02x:%02x:%02x\n",
			priv->id[0], priv->id[1], priv->id[2], priv->id[3], priv->id[4], priv->id[5]);
}

static DEVICE_ATTR(otp_id_ar0233, S_IRUGO, ar0233_otp_id_show, NULL);

static int ar0233_initialize(struct i2c_client *client)
{
	struct ar0233_priv *priv = to_ar0233(client);
	u16 pid = 0, rev = 0, val = 0;


	{//Rtx Add
		//printk(KERN_ERR "[ RTX-%s-%d ] priv->ser_addr = 0x%X\n", __func__, __LINE__, priv->ser_addr);		

		//reg16_write_addr(client, priv->ser_addr, MAX9295_REG3, 0x07);
		//reg16_write_addr(client, priv->ser_addr, MAX9max9295_set_regs295_REG6, 0xbf);
		//reg16_write_addr(client, priv->ser_addr, MAX9295_REF_VTG0, 0x53);
		//reg16_write_addr(client, priv->ser_addr, MAX9295_REF_VTG0, 0x51);
		printk(KERN_ERR "[ RTX-%s-%d ] \n", __func__, __LINE__);		

		max9295_set_regs(client, max9295_regs_rtx_1, ARRAY_SIZE(max9295_regs_rtx_1));
	}

	printk(KERN_ERR "[ RTX-%s-%d ] \n", __func__, __LINE__);
	setup_i2c_translator(client, priv->ser_addr, ar0233_i2c_addr[0]);
		
	/* check product ID */
	reg16_read16(client, AR0233_PID_REG, &pid);
	
	if (pid != AR0233_PID) {
		dev_dbg(&client->dev, "Product ID error %x\n", pid);
		return -ENODEV;
	}
	printk(KERN_ERR "[ RTX-%s-%d ] pid = 0x%X\n", __func__, __LINE__, pid);
	{//Rtx Add
		ar0233_set_regs(client, ar0233_2048x1280_crop_30fps_final, ARRAY_SIZE(ar0233_2048x1280_crop_30fps_final)); 
	}
	
	printk(KERN_ERR "[ RTX-%s-%d ] \n", __func__, __LINE__);
	//reg16_read16(client, AR0233_REV_REG, &rev);
	/* Program wizard registers */
	switch (pid) {
	case AR0233_PID:
	//	ar0233_set_regs(client, ar0233_regs_wizard, ARRAY_SIZE(ar0233_regs_wizard));
		break;
	}
	
	/* Read OTP IDs */
	//ar0233_otp_id_read(client);
#if 0
	/* read resolution used by current firmware */
	reg16_read16(client, 0xcae4, &val);
	priv->max_width = val;
	reg16_read16(client, 0xcae6, &val);
	priv->max_height = val;
#else
	priv->max_width = AR0233_MAX_WIDTH;
	priv->max_height = AR0233_MAX_HEIGHT;
#endif
	dev_info(&client->dev, "PID %x (rev%x), res %dx%d, OTP_ID %02x:%02x:%02x:%02x:%02x:%02x\n",
		 pid, rev, priv->max_width, priv->max_height, priv->id[0], priv->id[1], priv->id[2], priv->id[3], priv->id[4], priv->id[5]);

	return 0;
}

static int ar0233_parse_dt(struct device_node *np, struct ar0233_priv *priv)
{
	struct i2c_client *client = v4l2_get_subdevdata(&priv->sd);
	struct fwnode_handle *ep;
	u32 addrs[2], naddrs;

	naddrs = of_property_count_elems_of_size(np, "reg", sizeof(u32));
	if (naddrs != 2) {
		dev_err(&client->dev, "Invalid DT reg property\n");
		return -EINVAL;
	}

	if (of_property_read_u32_array(client->dev.of_node, "reg", addrs, naddrs) < 0) {
		dev_err(&client->dev, "Invalid DT reg property\n");
		return -EINVAL;
	}

	priv->ser_addr = addrs[1];

	ep = fwnode_graph_get_next_endpoint(dev_fwnode(&client->dev), NULL);
	if (!ep) {
		dev_err(&client->dev, "Unable to get endpoint in node %pOF: %ld\n",
				      client->dev.of_node, PTR_ERR(ep));
		return -ENOENT;
	}
	priv->sd.fwnode = ep;

	return 0;
}

static int ar0233_probe(struct i2c_client *client,
			const struct i2c_device_id *did)
{
	printk(KERN_ERR "[ RTX-%s-%d ] Start========================================================\n", __func__, __LINE__);		
	
	struct ar0233_priv *priv;
	int ret;

	priv = devm_kzalloc(&client->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	v4l2_i2c_subdev_init(&priv->sd, client, &ar0233_subdev_ops);
	priv->sd.flags = V4L2_SUBDEV_FL_HAS_DEVNODE;

	priv->exposure = 0x100;
	priv->gain = 0x100;
	priv->autogain = 1;
	v4l2_ctrl_handler_init(&priv->hdl, 4);
	v4l2_ctrl_new_std(&priv->hdl, &ar0233_ctrl_ops,
			  V4L2_CID_BRIGHTNESS, 0, 16, 1, 7);
	v4l2_ctrl_new_std(&priv->hdl, &ar0233_ctrl_ops,
			  V4L2_CID_CONTRAST, 0, 16, 1, 7);
	v4l2_ctrl_new_std(&priv->hdl, &ar0233_ctrl_ops,
			  V4L2_CID_SATURATION, 0, 7, 1, 2);
	v4l2_ctrl_new_std(&priv->hdl, &ar0233_ctrl_ops,
			  V4L2_CID_HUE, 0, 23, 1, 12);
	v4l2_ctrl_new_std(&priv->hdl, &ar0233_ctrl_ops,
			  V4L2_CID_GAMMA, -128, 128, 1, 0);
	v4l2_ctrl_new_std(&priv->hdl, &ar0233_ctrl_ops,
			  V4L2_CID_SHARPNESS, 0, 10, 1, 3);
	v4l2_ctrl_new_std(&priv->hdl, &ar0233_ctrl_ops,
			  V4L2_CID_AUTOGAIN, 0, 1, 1, priv->autogain);
	v4l2_ctrl_new_std(&priv->hdl, &ar0233_ctrl_ops,
			  V4L2_CID_GAIN, 0, 0xffff, 1, priv->gain);
	v4l2_ctrl_new_std(&priv->hdl, &ar0233_ctrl_ops,
			  V4L2_CID_EXPOSURE, 0, 0xffff, 1, priv->exposure);
	v4l2_ctrl_new_std(&priv->hdl, &ar0233_ctrl_ops,
			  V4L2_CID_HFLIP, 0, 1, 1, 0);
	v4l2_ctrl_new_std(&priv->hdl, &ar0233_ctrl_ops,
			  V4L2_CID_VFLIP, 0, 1, 1, 0);
	priv->sd.ctrl_handler = &priv->hdl;

	ret = priv->hdl.error;
	if (ret)
		goto cleanup;

	v4l2_ctrl_handler_setup(&priv->hdl);

	priv->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	priv->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&priv->sd.entity, 1, &priv->pad);
	if (ret < 0)
		goto cleanup;

	ret = ar0233_parse_dt(client->dev.of_node, priv);
	if (ret)
		goto cleanup;

	ret = ar0233_initialize(client);
	if (ret < 0)
		goto cleanup;
	printk(KERN_ERR "[ RTX-%s-%d ] \n", __func__, __LINE__);
	priv->rect.left = 0;
	priv->rect.top = 0;
	priv->rect.width = priv->max_width;
	priv->rect.height = priv->max_height;

	ret = v4l2_async_register_subdev(&priv->sd);
	if (ret)
		goto cleanup;

	if (device_create_file(&client->dev, &dev_attr_otp_id_ar0233) != 0) {
		dev_err(&client->dev, "sysfs otp_id entry creation failed\n");
		goto cleanup;
	}

	priv->init_complete = 1;

	printk(KERN_ERR "[ RTX-%s-%d ] End========================================================\n", __func__, __LINE__);

	return 0;

cleanup:
	media_entity_cleanup(&priv->sd.entity);
	v4l2_ctrl_handler_free(&priv->hdl);
	v4l2_device_unregister_subdev(&priv->sd);
	return ret;
}

static int ar0233_remove(struct i2c_client *client)
{
	struct ar0233_priv *priv = i2c_get_clientdata(client);

	device_remove_file(&client->dev, &dev_attr_otp_id_ar0233);
	v4l2_async_unregister_subdev(&priv->sd);
	media_entity_cleanup(&priv->sd.entity);
	v4l2_ctrl_handler_free(&priv->hdl);
	v4l2_device_unregister_subdev(&priv->sd);

	return 0;
}

static const struct i2c_device_id ar0233_id[] = {
	{ "ar0233", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ar0233_id);

static const struct of_device_id ar0233_of_ids[] = {
	{ .compatible = "onnn,ar0233", },
	{ }
};
MODULE_DEVICE_TABLE(of, ar0233_of_ids);

static struct i2c_driver ar0233_i2c_driver = {
	.driver	= {
		.name = "ar0233",
		.of_match_table = ar0233_of_ids,
	},
	.probe = ar0233_probe,
	.remove = ar0233_remove,
	.id_table = ar0233_id,
};

module_i2c_driver(ar0233_i2c_driver);

MODULE_DESCRIPTION("Camera glue driver for AP020X");
MODULE_AUTHOR("Andrey Gusakov");
MODULE_LICENSE("GPL");
