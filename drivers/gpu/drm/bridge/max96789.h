
#define MAX96789_PWR4  				0X0C
#define MAX96789_REG1				0x01
#define MAX96789_REG2				0x02
#define MAX96789_REG4				0x04
#define MAX96789_REG5				0x05

#define MAX96789_CTRL0 				0X10
#define MAX96789_CTRL1 				0X11
#define MAX96789_CTRL2				0X12
#define MAX96789_CTRL3				0x13

#define MAX96789_GPIO_BASE(n)		(0x2BE + n)
#define MAX96789_GPIO_A(n) 			(MAX96789_GPIO_BASE(0) + (3 * n))
#define MAX96789_GPIO_B(n) 			(MAX96789_GPIO_BASE(1) + (3 * n))
#define MAX96789_GPIO_C(n) 			(MAX96789_GPIO_BASE(2) + (3 * n))

#define MAX96789_I2C_0				0x40
#define MAX96789_I2C_1				0x41

#define MAX96789_FRONTTOP_0			0x308
#define MAX96789_FRONTTOP_9			0x311
#define MAX96789_FRONTTOP_20		0x31C
#define MAX96789_FRONTTOP_25		0x321
#define MAX96789_FRONTTOP_29		0x325
#define MAX96789_FRONTTOP_30		0x326

#define MAX96789_MIPI_RX0			0x330
#define MAX96789_MIPI_RX1			0x331
#define MAX96789_MIPI_RX2			0x332
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

#define REG8_NUM_RETRIES		1 /* number of read/write retries */
#define REG16_NUM_RETRIES		10 /* number of read/write retries */

struct max96789_priv {
	struct device *dev;
	struct i2c_client *client;
	struct regmap *regmap;
	struct gpio_desc *gpiod_pwdn;
	struct device_node		*host_node;

	struct drm_bridge bridge;
	struct drm_bridge *next_bridge;
	
	int links_mask;
	int dt;
};

struct max96789_source {
	struct fwnode_handle *fwnode;
	bool linkup;
};

struct max96789_link {
	struct fwnode_handle *sd_fwnode;
	struct i2c_client *client;
	int des_id;
	int des_addr;
	int pipes_mask;			/* mask of pipes used by this link */
	int out_mipi;			/* MIPI# */
	int out_vc;				/* VC# */
	struct regulator *poc_reg;	/* PoC power supply */
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

//static inline char *chip_name(int id)
//{
	//switch (id) {
	//case MAX9271_ID:
		//return "MAX9271";
	//case MAX9286_ID:
		//return "MAX9286";
	//case MAX9288_ID:
		//return "MAX9288";
	//case MAX9290_ID:
		//return "MAX9290";
	//case MAX9295A_ID:
		//return "MAX9295A";
	//case MAX9295B_ID:
		//return "MAX9295B";
	//case MAX9296A_ID:
		//return "MAX9296A";
	//case MAX96705_ID:
		//return "MAX96705";
	//case MAX96706_ID:
		//return "MAX96706";
	//case MAX96707_ID:
		//return "MAX96707";
	//case MAX96712_ID:
		//return "MAX96712";
	//default:
		//return "serializer";
	//}
//}

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


