#include <linux/delay.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of_graph.h>
#include <linux/videodev2.h>

#include <media/soc_camera.h>
#include <media/v4l2-common.h>
#include <media/v4l2-ctrls.h>

#include "ox03f10.h"
//#include "../gmsl/max96717.h"
#include "../gmsl/common.h"

static const int OX03F10_i2c_addr[] = {0x36};

#define OX03F10_PIDA_REG		0x300a
#define OX03F10_PIDB_REG		0x300b
#define OX03F10_PIDC_REG		0x300c
//#define OX03F10_REV_REG		0x0058

#define OX03F10_PID			0x460358 //it should be 0x580346 , but the read_n function reverse the order

#define OX03F10_MEDIA_BUS_FMT	MEDIA_BUS_FMT_SRGGB12_1X12

struct OX03F10_priv {
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

static inline struct OX03F10_priv *to_OX03F10(const struct i2c_client *client)
{
	return container_of(i2c_get_clientdata(client), struct OX03F10_priv, sd);
}

static inline struct v4l2_subdev *to_sd(struct v4l2_ctrl *ctrl)
{
	return &container_of(ctrl->handler, struct OX03F10_priv, hdl)->sd;
}

static int max92717_set_regs(struct i2c_client *client, const struct max92717_reg *regs, int nr_regs)
{
	struct OX03F10_priv *priv = to_OX03F10(client);
	int i = 0;

	for (i = 0; i < nr_regs; i++) {
		if (regs[i].reg == MAX92717_TABLE_DELAY) {
			mdelay(1000);
			continue;
		}
		reg16_write_addr(client, priv->ser_addr, regs[i].reg, regs[i].val);
	}

	return 0;
}

static int OX03F10_set_regs(struct i2c_client *client,
			  const struct OX03F10_reg *regs, int nr_regs)
{
	int i;

	for (i = 0; i < nr_regs; i++) {
		if (regs[i].reg == OX03F10_DELAY) {
			mdelay(regs[i].val);
			continue;
		}
		
		reg16_write(client, regs[i].reg, regs[i].val);
	}

	return 0;
}

static int OX03F10_s_stream(struct v4l2_subdev *sd, int enable)
{
	return 0;
}

static int OX03F10_get_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *mf = &format->format;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct OX03F10_priv *priv = to_OX03F10(client);

	if (format->pad)
		return -EINVAL;

	mf->width = priv->rect.width;
	mf->height = priv->rect.height;
	mf->code = OX03F10_MEDIA_BUS_FMT;
	mf->colorspace = V4L2_COLORSPACE_SMPTE170M;
	mf->field = V4L2_FIELD_NONE;

	return 0;
}

static int OX03F10_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_pad_config *cfg,
			  struct v4l2_subdev_format *format)
{
	struct v4l2_mbus_framefmt *mf = &format->format;

	mf->code = OX03F10_MEDIA_BUS_FMT;
	mf->colorspace = V4L2_COLORSPACE_SMPTE170M;
	mf->field = V4L2_FIELD_NONE;

	if (format->which == V4L2_SUBDEV_FORMAT_TRY)
		cfg->try_fmt = *mf;

	return 0;
}

static int OX03F10_enum_mbus_code(struct v4l2_subdev *sd,
				struct v4l2_subdev_pad_config *cfg,
				struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->pad || code->index > 0)
		return -EINVAL;

	code->code = OX03F10_MEDIA_BUS_FMT;

	return 0;
}

static int OX03F10_get_edid(struct v4l2_subdev *sd, struct v4l2_edid *edid)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct OX03F10_priv *priv = to_OX03F10(client);

	return 0;
}

static int OX03F10_set_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_pad_config *cfg,
				struct v4l2_subdev_selection *sel)
{
	struct v4l2_rect *rect = &sel->r;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct OX03F10_priv *priv = to_OX03F10(client);

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

static int OX03F10_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_pad_config *cfg,
				struct v4l2_subdev_selection *sel)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct OX03F10_priv *priv = to_OX03F10(client);

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

static int OX03F10_g_mbus_config(struct v4l2_subdev *sd,
				struct v4l2_mbus_config *cfg)
{
	cfg->flags = V4L2_MBUS_CSI2_1_LANE | V4L2_MBUS_CSI2_CHANNEL_0 |
		     V4L2_MBUS_CSI2_CONTINUOUS_CLOCK;
	cfg->type = V4L2_MBUS_CSI2_DPHY;

	return 0;
}

#ifdef CONFIG_VIDEO_ADV_DEBUG
static int OX03F10_g_register(struct v4l2_subdev *sd,
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

static int OX03F10_s_register(struct v4l2_subdev *sd,
			     const struct v4l2_dbg_register *reg)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	return reg16_write16(client, (u16)reg->reg, (u16)reg->val);
}
#endif

static struct v4l2_subdev_core_ops OX03F10_core_ops = {
#ifdef CONFIG_VIDEO_ADV_DEBUG
	.g_register = OX03F10_g_register,
	.s_register = OX03F10_s_register,
#endif
};

static int OX03F10_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct v4l2_subdev *sd = to_sd(ctrl);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct OX03F10_priv *priv = to_OX03F10(client);
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
	case V4L2_CID_HFLIP:
	case V4L2_CID_VFLIP:
		break;
	case V4L2_CID_MIN_BUFFERS_FOR_CAPTURE:
		ret = 0;
		break;
	}

	return ret;
}

static const struct v4l2_ctrl_ops OX03F10_ctrl_ops = {
	.s_ctrl = OX03F10_s_ctrl,
};

static struct v4l2_subdev_video_ops OX03F10_video_ops = {
	.s_stream	= OX03F10_s_stream,
	.g_mbus_config	= OX03F10_g_mbus_config,
};

static const struct v4l2_subdev_pad_ops OX03F10_subdev_pad_ops = {
	.get_edid	= OX03F10_get_edid,
	.enum_mbus_code	= OX03F10_enum_mbus_code,
	.get_selection	= OX03F10_get_selection,
	.set_selection	= OX03F10_set_selection,
	.get_fmt	= OX03F10_get_fmt,
	.set_fmt	= OX03F10_set_fmt,
};

static struct v4l2_subdev_ops OX03F10_subdev_ops = {
	.core	= &OX03F10_core_ops,
	.video	= &OX03F10_video_ops,
	.pad	= &OX03F10_subdev_pad_ops,
};

static ssize_t OX03F10_otp_id_show(struct device *dev,
				 struct device_attribute *attr, char *buf)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(to_i2c_client(dev));
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct OX03F10_priv *priv = to_OX03F10(client);

	return snprintf(buf, 32, "%02x:%02x:%02x:%02x:%02x:%02x\n",
			priv->id[0], priv->id[1], priv->id[2], priv->id[3], priv->id[4], priv->id[5]);
}

static DEVICE_ATTR(otp_id_OX03F10, S_IRUGO, OX03F10_otp_id_show, NULL);

static int OX03F10_initialize(struct i2c_client *client)
{
	struct OX03F10_priv *priv = to_OX03F10(client);
	u32 pid = 0;


	{	
		max92717_set_regs(client, max92717_regs_rtx, ARRAY_SIZE(max92717_regs_rtx));
	}

	setup_i2c_translator(client, priv->ser_addr, OX03F10_i2c_addr[0]);
		
	/* check product ID */
	reg16_read_n(client, OX03F10_PIDA_REG, (u8*)&pid, 3);
	if ((pid & 0x00ffffff) != OX03F10_PID) {
		dev_dbg(&client->dev, "Product ID error %x\n", pid);
		return -ENODEV;
	}

	{
		OX03F10_set_regs(client, OX03F10_2048x1280_crop_30fps_final, ARRAY_SIZE(OX03F10_2048x1280_crop_30fps_final)); 
	}
	
	priv->max_width = OX03F10_MAX_WIDTH;
	priv->max_height = OX03F10_MAX_HEIGHT;

	dev_info(&client->dev, "PID %x, res %dx%d, OTP_ID %02x:%02x:%02x:%02x:%02x:%02x\n",
		 pid, priv->max_width, priv->max_height, priv->id[0], priv->id[1], priv->id[2], priv->id[3], priv->id[4], priv->id[5]);

	return 0;
}

static int OX03F10_parse_dt(struct device_node *np, struct OX03F10_priv *priv)
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

static int OX03F10_probe(struct i2c_client *client,
			const struct i2c_device_id *did)
{	
	struct OX03F10_priv *priv;
	int ret;

	priv = devm_kzalloc(&client->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	v4l2_i2c_subdev_init(&priv->sd, client, &OX03F10_subdev_ops);
	priv->sd.flags = V4L2_SUBDEV_FL_HAS_DEVNODE;

	priv->exposure = 0x100;
	priv->gain = 0x100;
	priv->autogain = 1;
	v4l2_ctrl_handler_init(&priv->hdl, 4);
	v4l2_ctrl_new_std(&priv->hdl, &OX03F10_ctrl_ops,
			  V4L2_CID_BRIGHTNESS, 0, 16, 1, 7);
	v4l2_ctrl_new_std(&priv->hdl, &OX03F10_ctrl_ops,
			  V4L2_CID_CONTRAST, 0, 16, 1, 7);
	v4l2_ctrl_new_std(&priv->hdl, &OX03F10_ctrl_ops,
			  V4L2_CID_SATURATION, 0, 7, 1, 2);
	v4l2_ctrl_new_std(&priv->hdl, &OX03F10_ctrl_ops,
			  V4L2_CID_HUE, 0, 23, 1, 12);
	v4l2_ctrl_new_std(&priv->hdl, &OX03F10_ctrl_ops,
			  V4L2_CID_GAMMA, -128, 128, 1, 0);
	v4l2_ctrl_new_std(&priv->hdl, &OX03F10_ctrl_ops,
			  V4L2_CID_SHARPNESS, 0, 10, 1, 3);
	v4l2_ctrl_new_std(&priv->hdl, &OX03F10_ctrl_ops,
			  V4L2_CID_AUTOGAIN, 0, 1, 1, priv->autogain);
	v4l2_ctrl_new_std(&priv->hdl, &OX03F10_ctrl_ops,
			  V4L2_CID_GAIN, 0, 0xffff, 1, priv->gain);
	v4l2_ctrl_new_std(&priv->hdl, &OX03F10_ctrl_ops,
			  V4L2_CID_EXPOSURE, 0, 0xffff, 1, priv->exposure);
	v4l2_ctrl_new_std(&priv->hdl, &OX03F10_ctrl_ops,
			  V4L2_CID_HFLIP, 0, 1, 1, 0);
	v4l2_ctrl_new_std(&priv->hdl, &OX03F10_ctrl_ops,
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

	ret = OX03F10_parse_dt(client->dev.of_node, priv);
	if (ret)
		goto cleanup;

	ret = OX03F10_initialize(client);
	if (ret < 0)
		goto cleanup;
	
	priv->rect.left = 0;
	priv->rect.top = 0;
	priv->rect.width = priv->max_width;
	priv->rect.height = priv->max_height;

	ret = v4l2_async_register_subdev(&priv->sd);
	if (ret)
		goto cleanup;

	if (device_create_file(&client->dev, &dev_attr_otp_id_OX03F10) != 0) {
		dev_err(&client->dev, "sysfs otp_id entry creation failed\n");
		goto cleanup;
	}

	priv->init_complete = 1;

	return 0;

cleanup:
	media_entity_cleanup(&priv->sd.entity);
	v4l2_ctrl_handler_free(&priv->hdl);
	v4l2_device_unregister_subdev(&priv->sd);
	return ret;
}

static int OX03F10_remove(struct i2c_client *client)
{
	struct OX03F10_priv *priv = i2c_get_clientdata(client);

	device_remove_file(&client->dev, &dev_attr_otp_id_OX03F10);
	v4l2_async_unregister_subdev(&priv->sd);
	media_entity_cleanup(&priv->sd.entity);
	v4l2_ctrl_handler_free(&priv->hdl);
	v4l2_device_unregister_subdev(&priv->sd);

	return 0;
}

static const struct i2c_device_id OX03F10_id[] = {
	{ "OX03F10", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, OX03F10_id);

static const struct of_device_id OX03F10_of_ids[] = {
	{ .compatible = "quanta,OX03F10", },
	{ }
};
MODULE_DEVICE_TABLE(of, OX03F10_of_ids);

static struct i2c_driver OX03F10_i2c_driver = {
	.driver	= {
		.name = "OX03F10",
		.of_match_table = OX03F10_of_ids,
	},
	.probe = OX03F10_probe,
	.remove = OX03F10_remove,
	.id_table = OX03F10_id,
};

module_i2c_driver(OX03F10_i2c_driver);

MODULE_DESCRIPTION("Camera glue driver for OX03F10");
MODULE_AUTHOR("Retronix");
MODULE_LICENSE("GPL");
