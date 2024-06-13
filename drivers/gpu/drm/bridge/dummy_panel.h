#ifndef __DUMMY_PANEL_H__
#define __DUMMY_PANEL_H__

struct dummy_panel_reg 
{
	u16	reg;
	u8	val;
};

struct dummy_panel_device 
{
	struct device			*dev;
	struct max96776_device	*deserializer;
	struct i2c_client		*panel_dummy;
	
	//struct drm_bridge 		bridge;
	//struct drm_connector 	connector;
	struct drm_panel 		panel;
	//u32 connector_type;
};



#endif /* __DUMMY_PANEL_H__ */
