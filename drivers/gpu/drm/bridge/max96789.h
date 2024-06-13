#define DEBUG_COLOR_PATTERN			0

#define MAX96789_NUM_GMSL			2

#define HI_NIBBLE(b) (((b) >> 4) & 0x0F)
#define LO_NIBBLE(b) ((b) & 0x0F)

#define MAX96789_PWR0  				0x08
#define MAX96789_PWR4  				0x0C
#define MAX96789_REG1				0x01
#define MAX96789_REG2				0x02
#define MAX96789_REG3				0x03
#define MAX96789_REG4				0x04
#define MAX96789_REG5				0x05
#define MAX96789_REG6				0x06
#define MAX96789_REG13				0x0D
#define MAX96789_REG15				0x0F

#define MAX96789_CTRL0 				0x10
#define MAX96789_CTRL1 				0x11
#define MAX96789_CTRL2				0x12
#define MAX96789_CTRL3				0x13

#define MAX96789_GPIO_BASE(n)		(0x2BE + n)
#define MAX96789_GPIO_A(n) 			(MAX96789_GPIO_BASE(0) + (3 * n))
#define MAX96789_GPIO_B(n) 			(MAX96789_GPIO_BASE(1) + (3 * n))
#define MAX96789_GPIO_C(n) 			(MAX96789_GPIO_BASE(2) + (3 * n))

#define MAX96789_I2C_0				0x40
#define MAX96789_I2C_1				0x41
#define MAX96789_I2C_2				0x42
#define MAX96789_I2C_3				0x43
#define MAX96789_I2C_4				0x44
#define MAX96789_I2C_5				0x45

#define MAX96789_FRONTTOP_0			0x308
#define MAX96789_FRONTTOP_9			0x311
#define MAX96789_FRONTTOP_20		0x31C
#define MAX96789_FRONTTOP_21		0x31D
#define MAX96789_FRONTTOP_25		0x321
#define MAX96789_FRONTTOP_26		0x322
#define MAX96789_FRONTTOP_29		0x325
#define MAX96789_FRONTTOP_30		0x326

#define MAX96789_MIPI_RX0			0x330
#define MAX96789_MIPI_RX1			0x331
#define MAX96789_MIPI_RX2			0x332
#define MAX96789_MIPI_RX3			0x333
#define MAX96789_MIPI_RX4			0x334
#define MAX96789_MIPI_RX5			0x335
#define MAX96789_MIPI_RX8			0x338

// Controller 0
#define MAX96789_MIPI_DSI0			0x380
#define MAX96789_MIPI_DSI1			0x381
#define MAX96789_MIPI_DSI2			0x382
#define MAX96789_MIPI_DSI5			0x385
#define MAX96789_MIPI_DSI6			0x386
#define MAX96789_MIPI_DSI7			0x387
#define MAX96789_MIPI_DSI8			0x388
#define MAX96789_MIPI_DSI9			0x389
#define MAX96789_MIPI_DSI10			0x38A
#define MAX96789_MIPI_DSI11			0x38B
#define MAX96789_MIPI_DSI12			0x38C
#define MAX96789_MIPI_DSI13			0x38D
#define MAX96789_MIPI_DSI14			0x38E
#define MAX96789_MIPI_DSI15			0x38F
#define MAX96789_MIPI_DSI32			0x3A0
#define MAX96789_MIPI_DSI36			0x3A4
#define MAX96789_MIPI_DSI37			0x3A5
#define MAX96789_MIPI_DSI38			0x3A6
#define MAX96789_MIPI_DSI39			0x3A7
#define MAX96789_MIPI_DSI40			0x3A8
#define MAX96789_MIPI_DSI41			0x3A9
#define MAX96789_MIPI_DSI42			0x3AA
#define MAX96789_MIPI_DSI43			0x3AB
#define MAX96789_MIPI_DSI44			0x3AC
#define MAX96789_MIPI_DSI45			0x3AD
#define MAX96789_MIPI_DSI46			0x3AE

#define MAX96789_TX_BASE(n)			(0x50 + n * 4)
#define MAX96789_TX0(n)				(MAX96789_TX_BASE(n) + 0)
#define MAX96789_TX1(n)				(MAX96789_TX_BASE(n) + 1)
#define MAX96789_TX3(n)				(MAX96789_TX_BASE(n) + 3)

#define MAX96789_VIDEO_TX_BASE(n)	(0x100 + n * 8)
#define MAX96789_VIDEO_TX0(n)		(MAX96789_VIDEO_TX_BASE(n) + 0)
#define MAX96789_VIDEO_TX1(n)		(MAX96789_VIDEO_TX_BASE(n) + 1)
#define MAX96789_VIDEO_TX2(n)		(MAX96789_VIDEO_TX_BASE(n) + 2)
#define MAX96789_VIDEO_TX3(n)		(MAX96789_VIDEO_TX_BASE(n) + 6)

#define MAX96789_VTX_BASE(n)		(0x1B0 + 0x43 * n)
#define MAX96789_VTX_X(n)			(MAX96789_VTX_BASE(0) + 0x18 + n)
#define MAX96789_VTX_Y(n)			(MAX96789_VTX_BASE(1) + 0x18 + n)
#define MAX96789_VTX_Z(n)			(MAX96789_VTX_BASE(2) + 0x18 + n)
#define MAX96789_VTX_U(n)			(MAX96789_VTX_BASE(3) + 0x18 + n)

#define MIPI_DT_RGB888				0x24

#define MAX96789_HS_VS(n)			(0x55D + n)
#define MAX96789_HS_VS_X			MAX96789_HS_VS(0)
#define MAX96789_HS_VS_Y			MAX96789_HS_VS(1)
#define MAX96789_HS_VS_Z			MAX96789_HS_VS(2)
#define MAX96789_HS_VS_U			MAX96789_HS_VS(3)

#define MAX96776_ID					0x8C
#define MAX96778_ID					0x8D

#define REG8_NUM_RETRIES			1 /* number of read/write retries */
#define REG16_NUM_RETRIES			10 /* number of read/write retries */

struct max96789_source {
	struct fwnode_handle *fwnode;
	bool linkup;
};

struct max96789_link {
	struct fwnode_handle *sd_fwnode;
	struct i2c_client	 *client;
	int des_id;
	int des_addr;
	int pipes_mask;		 /* mask of pipes used by this link */
	int out_mipi;		 /* MIPI# */
	int out_vc;			 /* VC# */
	struct regulator 	 *poc_reg;	/* PoC power supply */
};

struct max96789_priv {
	struct device 		*dev;
	struct i2c_client	*client;
	struct regmap 		*regmap;
	struct gpio_desc 	*gpiod_pwdn;
	struct device_node	*host_node;
	struct max96789_link *link[MAX96789_NUM_GMSL];
	
	struct drm_bridge 	bridge;
	struct drm_bridge 	*next_bridge;
	
	struct drm_panel 	panel;
	struct drm_panel 	*next_panel;
	
	int links_mask;
	int dt;
};

/* -----------------------------------------------------------------------------
 * I2C IO
 */
static int max96789_write_reg(struct max96789_priv *priv, u16 reg, u8 val)
{
	int ret;
	
	u8 regbuf[3];
	regbuf[0] = reg >> 8;
	regbuf[1] = reg & 0xff;
	regbuf[2] = val;
	//regmap_raw_write(link->regmap, reg, &val, 3);
	ret = i2c_master_send(priv->client, regbuf, 3);
	msleep(5);
	if (ret < 0) 
	{
		dev_err(&priv->client->dev, "%s: write reg error %d: reg=%x, val=%x\n", __func__, ret, reg, val);
		return ret;
	}
	return 0;
}


static int max96789_write_n(struct max96789_priv *priv, int reg, int val_count, int val)
{
	int ret;
	int i;
	
	u8 values[3];
	for (i = 0; i < val_count; i++)
	{
		values[i] = (val >> ((val_count - i - 1) * 8)) & 0xff;
		ret = max96789_write_reg(priv, reg, values[i]);
		if (ret)
		{
			dev_dbg(&priv->client->dev, "write register 0x%04x failed (%d)\n", reg, ret);
			return ret;
		}
		reg += 1;
	}
	return 0;
}

static int max96789_read(struct max96789_priv *priv, u16 reg, u8 *val)
{
	int ret;
	u8 regbuf[2];
	regbuf[0] = reg >> 8;
	regbuf[1] = reg & 0xff;

	ret = i2c_master_send(priv->client, regbuf, 2);
	if (ret < 0) 
	{
		dev_err(&priv->client->dev, "%s: write reg error %d: reg=%x\n", __func__, ret, reg);
		return ret;
	}

	ret = i2c_master_recv(priv->client, val, 1);
	if (ret < 0) 
	{
		dev_err(&priv->client->dev, "%s: read reg error %d: reg=%x\n", __func__, ret, reg);
		return ret;
	}
	return 0;
}

static inline int max96789_update_bits(struct max96789_priv *priv, u16 reg,
				       u8 mask, u8 bits)
{
	int ret, tmp;
	u8 val;

	ret = max96789_read(priv, reg, &val);
	if (ret != 0)
		return ret;

	tmp = val & ~mask;
	tmp |= bits & mask;

	if (tmp != val)
		ret = max96789_write_reg(priv, reg, tmp);
	return ret;
}

//------------------------------------------

static inline char *chip_name(int id)
{
	switch (id) {
	case MAX96776_ID:
		return "MAX96776";
	case MAX96778_ID:
		return "MAX96778";
	default:
		return "de-serializer";
	}
}

struct max96776_reg {
	u16	reg;
	u8	val;
};

static const struct max96776_reg configuretable_vc0113[] = {
	//{MAX9295_REG2, 0x03}, /* VideoTX Disable and write 3 to reserved bits */
	//{MAX9295_VIDEO_TX0(0), 0x60}, /* VIDEO_TX) - Line CRC enabled.  Encoding ON. Read back 62. */
	//{MAX9295_VIDEO_TX1(0), 0x0A}, /* VIDEO_TX) - BPP Setting. */

//// GPIO8: FV_OUT in <== Camera-Ser output
	//{MAX9295_GPIO_A(8), 0x63}, /* GPIO_A - for GPIO8 ... MFP8: FV_OUT in <=== ISP(AP0200)[from Camera-Ser(GPIO8)] */
	//{MAX9295_GPIO_B(8), 0x2B}, /* GPIO_B - for GPIO8 ... GPIO_TX_ID=11, push-pull */
	//{MAX9295_GPIO_C(8), 0x0B}, /* GPIO_C(dummy) */

//// GPIO7: FR_SYNC out ==> Camera-Ser input
	//{MAX9295_GPIO_A(7), 0x84}, /* GPIO_A - for GPIO7 ... MFP7: FR_SYNC out==> ISP(AP0200)[to Camera-Ser(GPIO8)] */
	//{MAX9295_GPIO_B(7), 0x2C}, /* GPIO_B(dummy), push-pull */
	//{MAX9295_GPIO_C(7), 0x0C}, /* GPIO_C - for GPIO7 ... GPIO_RX_ID=12 */

	//{MAX9295_REG7, 0xC7}, 	  /* Configure serializer for parallel sensor input */
	//{MAX9295_MIPI_RX2, 0xEE}, /* PHY lane mapping - PHY1 lane 0 is lane 2, PHY1 lane 1 is lane 3 */
	//{MAX9295_MIPI_RX3, 0xE4}, /* PHY lane mapping - PHY2 lane 0 is lane 0, PHY1 lane 1 is lane 1 */
	//{MAX9295_FRONTTOP_12, 0x2B}, /* Select designated datatype to route to Video Pipeline X */
	//{MAX9295_FRONTTOP_14, 0x22}, /* Select designated datatype to route to Video Pipeline Y */
	//{MAX9295_FRONTTOP_16, 0x22}, /* Select designated datatype to route to Video Pipeline Z */
	//{MAX9295_FRONTTOP_18, 0x22}, /* Select designated datatype to route to Video Pipeline U */
	//{MAX9295_FRONTTOP_20, 0x2A}, /* Soft BPP Pipe X - 10 bits */

	//{MAX9295_REG2, 0x13}, 	  /* Video transmit Pipe X enable */
	//{MAX9295_REF_VTG1, 0x89}  /* GPIO4 Output RCLK to Sensor, RCLK f_REF : reference clock output */
};

static int max96776_write_reg(struct max96789_link *link, u16 reg, u8 val)
{
	u8 buf[3] = { reg >> 8, reg & 0xff, val };
	int ret;
	ret = i2c_master_send(link->client, buf, 3);
	return ret < 0 ? ret : 0;
}

static int max96776_write_n(struct max96789_link *link, int reg, int val_count, int val)
{
	int ret;
	
	int i;
	u8 values[3];
	for (i = 0; i < val_count; i++)
	{
		values[i] = (val >> ((val_count - i - 1) * 8)) & 0xff;
		ret = max96776_write_reg(link, reg, values[i]);
		if (ret)
		{
			dev_dbg(&link->client->dev, "write register 0x%04x failed (%d)\n", reg, ret);
			return ret;
		}
		reg += 1;
	}
	return 0;
}

static int max96776_set_regs(struct max96789_link *link,
							 const struct max96776_reg *regs,
							 unsigned int nr_regs)
{
	unsigned int i;
	int ret;

	for (i = 0; i < nr_regs; i++) 
	{
		ret = max96776_write_reg(link, regs[i].reg, regs[i].val);
		msleep(5);
		if (ret)
			return ret;
	}

	return 0;
}

//static inline int reg8_read(struct i2c_client *client, u8 reg, u8 *val)
//{
	//int ret, retries;

	//for (retries = REG8_NUM_RETRIES; retries; retries--) {
		//ret = i2c_smbus_read_byte_data(client, reg);
		//if (!(ret < 0))
			//break;
	//}

	//if (ret < 0) {
		//dev_dbg(&client->dev,
			//"read fail: chip 0x%x register 0x%x: %d\n",
			//client->addr, reg, ret);
	//} else {
		//*val = ret;
	//}

	//return ret < 0 ? ret : 0;
//}

//static inline int reg8_write(struct i2c_client *client, u8 reg, u8 val)
//{
	//int ret, retries;

	//for (retries = REG8_NUM_RETRIES; retries; retries--) {
		//ret = i2c_smbus_write_byte_data(client, reg, val);
		//if (!(ret < 0))
			//break;
	//}

	//if (ret < 0) {
		//dev_dbg(&client->dev,
			//"write fail: chip 0x%x register 0x%x: %d\n",
			//client->addr, reg, ret);
	//} else {
//#ifdef WRITE_VERIFY
		//u8 val2;
		//reg8_read(client, reg, &val2);
		//if (val != val2)
			//dev_err(&client->dev,
				//"write verify mismatch: chip 0x%x reg=0x%x "
				//"0x%x->0x%x\n", client->addr, reg, val, val2);
//#endif
	//}

	//return ret < 0 ? ret : 0;
//}

//static inline int reg16_read(struct i2c_client *client, u16 reg, u8 *val)
//{
	//int ret, retries;
	//u8 buf[2] = {reg >> 8, reg & 0xff};

	//for (retries = REG16_NUM_RETRIES; retries; retries--) {
		//ret = i2c_master_send(client, buf, 2);
		//if (ret == 2) {
			//ret = i2c_master_recv(client, buf, 1);
			//if (ret == 1)
				//break;
		//}
	//}

	//if (ret < 0) {
		//dev_dbg(&client->dev,
			//"read fail: chip 0x%x register 0x%x: %d\n",
			//client->addr, reg, ret);
	//} else {
		//*val = buf[0];
	//}

	//return ret < 0 ? ret : 0;
//}

//static inline int reg16_write(struct i2c_client *client, u16 reg, u8 val)
//{
	//int ret, retries;
	//u8 buf[3] = {reg >> 8, reg & 0xff, val};

	//for (retries = REG16_NUM_RETRIES; retries; retries--) {
		//ret = i2c_master_send(client, buf, 3);
		//if (ret == 3)
			//break;
	//}

	//if (ret < 0) {
		//dev_dbg(&client->dev,
			//"write fail: chip 0x%x register 0x%x: %d\n",
			//client->addr, reg, ret);
	//} else {
//#ifdef WRITE_VERIFY
		//u8 val2;
		//reg16_read(client, reg, &val2);
		//if (val != val2)
			//dev_err(&client->dev,
				//"write verify mismatch: chip 0x%x reg=0x%x "
				//"0x%x->0x%x\n", client->addr, reg, val, val2);
//#endif
	//}

	//return ret < 0 ? ret : 0;
//}

//static inline int reg16_read16(struct i2c_client *client, u16 reg, u16 *val)
//{
	//int ret, retries;
	//u8 buf[2] = {reg >> 8, reg & 0xff};

	//for (retries = REG8_NUM_RETRIES; retries; retries--) {
		//ret = i2c_master_send(client, buf, 2);
		//if (ret == 2) {
			//ret = i2c_master_recv(client, buf, 2);
			//if (ret == 2)
				//break;
		//}
	//}

	//if (ret < 0) {
		//dev_dbg(&client->dev,
			//"read fail: chip 0x%x register 0x%x: %d\n",
			//client->addr, reg, ret);
	//} else {
		//*val = ((u16)buf[0] << 8) | buf[1];
	//}

	//return ret < 0 ? ret : 0;
//}

//static inline int reg16_write16(struct i2c_client *client, u16 reg, u16 val)
//{
	//int ret, retries;
	//u8 buf[4] = {reg >> 8, reg & 0xff, val >> 8, val & 0xff};

	//for (retries = REG8_NUM_RETRIES; retries; retries--) {
		//ret = i2c_master_send(client, buf, 4);
		//if (ret == 4)
			//break;
	//}

	//if (ret < 0) {
		//dev_dbg(&client->dev,
			//"write fail: chip 0x%x register 0x%x: %d\n",
			//client->addr, reg, ret);
	//}

	//return ret < 0 ? ret : 0;
//}

//static inline int reg16_read_n(struct i2c_client *client,
			       //u16 reg, u8 *val, int n)
//{
	//int ret, retries;
	//u8 buf[2] = {reg >> 8, reg & 0xff};

	//for (retries = REG16_NUM_RETRIES; retries; retries--) {
		//ret = i2c_master_send(client, buf, 2);
		//if (ret == 2) {
			//ret = i2c_master_recv(client, val, n);
			//if (ret == n)
				//break;
		//}
	//}

	//if (ret < 0) {
		//dev_dbg(&client->dev,
			//"read fail: chip 0x%x registers 0x%x-0x%x: %d\n",
			//client->addr, reg, reg + n, ret);
	//}

	//return ret < 0 ? ret : 0;
//}

static inline int reg8_read_addr(struct i2c_client *client, int addr,
				 u8 reg, u8 *val)
{
	int ret, retries;
	union i2c_smbus_data data;

	for (retries = REG8_NUM_RETRIES; retries; retries--) {
		ret = i2c_smbus_xfer(client->adapter, addr, client->flags,
				     I2C_SMBUS_READ, reg,
				     I2C_SMBUS_BYTE_DATA, &data);
		if (!(ret < 0))
			break;
	}

	if (ret < 0) {
		dev_dbg(&client->dev,
			"read fail: chip 0x%x register 0x%x: %d\n",
			addr, reg, ret);
	} else {
		*val = data.byte;
	}

	return ret < 0 ? ret : 0;
}

static inline int reg8_write_addr(struct i2c_client *client, u8 addr,
				  u8 reg, u8 val)
{
	int ret, retries;
	union i2c_smbus_data data;

	data.byte = val;

	for (retries = REG8_NUM_RETRIES; retries; retries--) {
		ret = i2c_smbus_xfer(client->adapter, addr, client->flags,
				     I2C_SMBUS_WRITE, reg,
				     I2C_SMBUS_BYTE_DATA, &data);
		if (!(ret < 0))
			break;
	}

	if (ret < 0) {
		dev_dbg(&client->dev,
			"write fail: chip 0x%x register 0x%x value 0x%0x: %d\n",
			addr, reg, val, ret);
	}

	return ret < 0 ? ret : 0;
}


static inline int reg16_write_addr(struct i2c_client *client, int chip,
				   u16 reg, u8 val)
{
	struct i2c_msg msg[1];
	u8 wbuf[3];
	int ret;

	msg->addr = chip;
	msg->flags = 0;
	msg->len = 3;
	msg->buf = wbuf;
	wbuf[0] = reg >> 8;
	wbuf[1] = reg & 0xff;
	wbuf[2] = val;

	ret = i2c_transfer(client->adapter, msg, 1);
	if (ret < 0) {
		dev_dbg(&client->dev,
			"i2c fail: chip 0x%02x wr 0x%04x (0x%02x): %d\n",
			chip, reg, val, ret);
		return ret;
	}

	return 0;
}

static inline int reg16_read_addr(struct i2c_client *client, int chip,
				  u16 reg, int *val)
{
	struct i2c_msg msg[2];
	u8 wbuf[2];
	u8 rbuf[1];
	int ret;

	msg[0].addr = chip;
	msg[0].flags = 0;
	msg[0].len = 2;
	msg[0].buf = wbuf;
	wbuf[0] = reg >> 8;
	wbuf[1] = reg & 0xff;

	msg[1].addr = chip;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 1;
	msg[1].buf = rbuf;

	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret < 0) {
		dev_dbg(&client->dev, "i2c fail: chip 0x%02x rd 0x%04x: %d\n",
			chip, reg, ret);
		return ret;
	}

	*val = rbuf[0];

	return 0;
}

#define __reg8_read(addr, reg, val)		reg8_read_addr(priv->client, addr, reg, val)
#define __reg8_write(addr, reg, val)	reg8_write_addr(priv->client, addr, reg, val)
#define __reg16_read(addr, reg, val)	reg16_read_addr(priv->client, addr, reg, val)
#define __reg16_write(addr, reg, val)	reg16_write_addr(priv->client, addr, reg, val)

//struct i2c_mux_priv {
	//struct i2c_adapter adap;
	//struct i2c_algorithm algo;
	//struct i2c_mux_core *muxc;
	//u32 chan_id;
//};

//static inline int get_des_id(struct i2c_client *client)
//{
	//struct i2c_mux_priv *mux_priv = client->adapter->algo_data;

	//if (!strcmp(mux_priv->muxc->dev->driver->name, "max9286"))
		//return MAX9286_ID;
	//if (!strcmp(mux_priv->muxc->dev->driver->name, "max9288"))
		//return MAX9288_ID;
	//if (!strcmp(mux_priv->muxc->dev->driver->name, "max9296"))
		//return MAX9296A_ID;
	//if (!strcmp(mux_priv->muxc->dev->driver->name, "max96706"))
		//return MAX96706_ID;
	//if (!strcmp(mux_priv->muxc->dev->driver->name, "max96712"))
		//return MAX96712_ID;
	//if (!strcmp(mux_priv->muxc->dev->driver->name, "ti9x4"))
		//return UB960_ID;

	//return -EINVAL;
//}

//static inline int get_des_addr(struct i2c_client *client)
//{
	//struct i2c_mux_priv *mux_priv = client->adapter->algo_data;

	//return to_i2c_client(mux_priv->muxc->dev)->addr;
//}

//static inline void setup_i2c_translator(struct i2c_client *client, int ser_addr,
					//int sensor_addr)
//{
	//int gmsl_mode = MODE_GMSL2;

	//switch (get_des_id(client)) {
	//case MAX9286_ID:
	//case MAX9288_ID:
	//case MAX96706_ID:
		//reg8_write_addr(client, ser_addr, 0x09, client->addr << 1);
		//reg8_write_addr(client, ser_addr, 0x0A, sensor_addr << 1);
		//break;
	//case MAX9296A_ID:
	//case MAX96712_ID:
		///* parse gmsl mode from deserializer */
		//reg16_read_addr(client, get_des_addr(client), 6, &gmsl_mode);
		//gmsl_mode = !!(gmsl_mode & BIT(7)) + 1;

		//if (gmsl_mode == MODE_GMSL1) {
			//reg8_write_addr(client, ser_addr, 0x09,
					//client->addr << 1);
			//reg8_write_addr(client, ser_addr, 0x0A,
					//sensor_addr << 1);
		//}
		//if (gmsl_mode == MODE_GMSL2) {
			//reg16_write_addr(client, ser_addr, MAX9295_I2C2,
					 //client->addr << 1);
			//reg16_write_addr(client, ser_addr, MAX9295_I2C3,
					 //sensor_addr << 1);
		//}
		//break;
	//case UB960_ID:
		//reg8_write_addr(client, get_des_addr(client), 0x65,
				//client->addr << 1);
		//reg8_write_addr(client, get_des_addr(client), 0x5d,
				//sensor_addr << 1);
		//break;
	//}
	//usleep_range(2000, 2500);
//}




//static void max96789_gmsl2_link_pipe_setup(struct max96789_priv *priv, int link_n)
//{
	////struct max96789_link *link = priv->link[link_n];
	//int pipe = link_n; /* straight mapping */
	//int dt = priv->dt; /* must come from imager */
	//int in_vc = 0;
	//int i;

	//// AUTO_LINK = 0, link A is selected
	//max96789_update_bits(priv, MAX96789_CTRL0, BIT(4), BIT(4));	
		
	//// packets are transmitted over GMSL A
	//max96789_write_reg(priv, MAX96789_TX0(0), 0xB8);
	
	//// Stream ID and splitter more
	//max96789_write_reg(priv, MAX96789_TX3(0), 0x10);
	
	////// LINK A remote wake-up enable
	////max96789_update_bits(priv, MAX96789_PWR4, 0x70, 0x10);
	
	//// LINK A is enabled
	//max96789_update_bits(priv, MAX96789_REG4, 0x50, 0x50);
	
	//max96789_pipe_override(priv, pipe, dt, in_vc);
	//usleep_range(2000, 5000);
	
	//// start video pipe X from DSI port A
	//max96789_write_reg(priv, MAX96789_FRONTTOP_9, 0x01);
	//usleep_range(2000, 5000);
	
	////// CRC bpp from BPP bitfield
	////max96789_write_reg(priv, MAX96789_VIDEO_TX0(0), 0x68);
	////// bpp
	////max96789_update_bits(priv, MAX96789_VIDEO_TX1(0), 0x3F, 0x18);
	////// mask video with DE
	////max96789_update_bits(priv, MAX96789_VIDEO_TX6(0), BIT(6), BIT(6));
	
	////
//}

//static void max96789_reset_oneshot(struct max96789_priv *priv, int mask)
//{
	//int timeout;
	//u8 val;

	//// reset link
	//max96789_update_bits(priv, MAX96789_CTRL0, 0x60, 0x60);	

	////mask &= 0x0f;
	////max96712_update_bits(priv, MAX96712_CTRL1, mask, mask);

	/////* wait for one-shot bit self-cleared */
	////for (timeout = 0; timeout < 100; timeout++) 
	////{
		////max96712_read(priv, MAX96712_CTRL1, &val);
		////if (!(val & mask))
			////break;

		////mdelay(1);
	////}

	////if (val & mask)
		////dev_err(&priv->client->dev, "Failed reset oneshot 0x%x\n", mask);
//}

//static int max96789_gmsl2_get_link_lock(struct max96789_priv *priv, int link_n)
//{
	//u8 val;

	//max96789_read(priv, MAX96789_CTRL3, &val);

	//return !!(val & BIT(3));
//}

//static int max96789_gmsl2_reverse_channel_setup(struct max96789_priv *priv, int link_n)
//{
	////struct max96712_link *link = priv->link[link_n];
	//int des_addrs[] = {0x40, 0x42, 0x60, 0x62};	/* possible MAX9295 addresses on i2c bus */
	//int timeout = 100;
	//int ret = 0;
	//int val = 0, i, j = 0;

	//max96789_update_bits(priv, MAX96789_REG4, 0x50, 0x40);
	
	////max96789_reset_oneshot(priv, 0);

	///*
	 //* wait the link to be established,
	 //* indicated when status bit LOCKED goes high
	 //*/
	////for (; timeout > 0; timeout--) 
	////{
		////if (max96789_gmsl2_get_link_lock(priv, 0))
			////break;
		////mdelay(1);
	////}

	////if (!timeout) 
	////{
		////ret = -ETIMEDOUT;
		//////goto out;
	////}

	//for (i = 0; i < ARRAY_SIZE(des_addrs); i++) 
	//{
		///* read deserializer ID */
		////__reg16_read(des_addrs[i], 0x000d, &val);
		////if (val == MAX9295A_ID || val == MAX9295B_ID) 
		////{
			////dev_dbg(&priv->client->dev, "ID val:0x%x>\n", val);
			////link->des_id = val;
			 /////* relocate deserizlizer on I2C bus */
			////__reg16_write(des_addrs[i], 0x0000, link->des_addr << 1);
			////usleep_range(2000, 2500);
			////j = i;
		////}
	//}


	//max96789_write_reg(priv, MAX96789_REG2, BIT(4));
	
	////priv->links_mask |= BIT(link_n);

	//return ret;
//}
