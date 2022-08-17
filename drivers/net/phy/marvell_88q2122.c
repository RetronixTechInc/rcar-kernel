/*
 * drivers/net/phy/marvell_88q2122.c
 *
 * Driver for Marvell 88Q2122 PHYs
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 *
 */
#include <linux/kernel.h>
#include <linux/string.h>
#include <linux/ctype.h>
#include <linux/errno.h>
#include <linux/unistd.h>
#include <linux/hwmon.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/netdevice.h>
#include <linux/etherdevice.h>
#include <linux/skbuff.h>
#include <linux/spinlock.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/mii.h>
#include <linux/ethtool.h>
#include <linux/phy.h>
#include <linux/marvell_phy.h>
#include <linux/of.h>

#include <linux/io.h>
#include <asm/irq.h>
#include <linux/uaccess.h>

#define REGISTER13_ADDR		0x0d
#define REGISTER14_ADDR		0x0e
#define	DEVADDR_OFFSET		0x4000

#define MARVELL_PHY_ID_88Q2122			0x002B0980
#define MARVELL_PHY_ID_88Q2122_MASK		0xfffffff0

#define MRVL_Q212X_LPSD_FEATURE_ENABLE 0

MODULE_DESCRIPTION("Marvell 88Q2122 PHY driver");
MODULE_AUTHOR("Layne");
MODULE_LICENSE("GPL");

static int m88Q2122_probe(struct phy_device *phydev)
{
	phydev_info(phydev, "Marvell m88Q2122_probe!\n");
	return 0;
}

// Write a 16-bit value into a register through SMI interface.
// @param phydev bootstrap address of the PHY
// @param devAddr section of register map (Ex: 0x07 : Auto-Neg registers)
// @param regAddr register address within section
// @param data 16-bit to write into register
// @return void
//static void regWrite ( uint16_t phydev, uint16_t devAddr, uint16_t regAddr, uint16_t data );
static void regWrite ( struct phy_device *phydev, u16 devAddr, u16 regAddr, u16 data )
{
	int value = 0;
	int ret = 0;

	//set slave mode of the phy
	ret = mdiobus_write(phydev->mdio.bus, phydev->mdio.addr, REGISTER13_ADDR, devAddr);
	ret |= mdiobus_write(phydev->mdio.bus, phydev->mdio.addr, REGISTER14_ADDR, regAddr);
	ret |= mdiobus_write(phydev->mdio.bus, phydev->mdio.addr, REGISTER13_ADDR, DEVADDR_OFFSET+devAddr);
	ret |= mdiobus_write(phydev->mdio.bus, phydev->mdio.addr, REGISTER14_ADDR, data);
	if(ret < 0)
	{
		phydev_err(phydev, "set slave mdiobus_write ret:0x%x!\n", ret);
	}
}

// Read out a 16-bit value from a register through SMI interface.
// @param phydev bootstrap address of the PHY
// @param devAddr section of register map (Ex: 0x07 : Auto-Neg registers)
// @param regAddr register address within section
// @return value in register if successful
//static int regRead ( uint16_t phydev, uint16_t devAddr, uint16_t regAddr );
static int regRead ( struct phy_device *phydev, u16 devAddr, u16 regAddr )
{
	int value = 0;
	int ret = 0;

	ret = mdiobus_write(phydev->mdio.bus, phydev->mdio.addr, REGISTER13_ADDR, devAddr);
	ret |= mdiobus_write(phydev->mdio.bus, phydev->mdio.addr, REGISTER14_ADDR, regAddr);
	ret |= mdiobus_write(phydev->mdio.bus, phydev->mdio.addr, REGISTER13_ADDR, DEVADDR_OFFSET+devAddr);
	if(ret < 0)
	{
		phydev_err(phydev, "read status mdiobus_write ret:0x%x!\n", ret);
	}
	value = mdiobus_read(phydev->mdio.bus, phydev->mdio.addr, REGISTER14_ADDR);
	//phydev_info(phydev, "Marvell m88Q2122 read mode value:0x%x!\n", value);

	return value;
}

// Set Master/Slave mode of the PHY by software
// @param phydev bootstrap address of the PHY
// @param forceMaster
// @return void
static void setMasterSlave(struct phy_device *phydev, bool forceMaster)
{
	u16 regData = 0;
	regData = regRead(phydev, 1, 0x0834);

	if (forceMaster)
	{
		phydev_info(phydev, "Set Master\n");
		regData |= 0x4000;
	}
	else
	{
		phydev_info(phydev, "Set Slave\n");
		regData &= 0xBFFF;
	}

	regWrite(phydev, 1, 0x0834, regData);
	if (forceMaster && MRVL_Q212X_LPSD_FEATURE_ENABLE)
	{
		regWrite(phydev, 7, 0x8032, 0x005A);
	}
	else
	{
		regWrite(phydev, 7, 0x8032, 0x0064);
	}
	regWrite(phydev, 7, 0x8031, 0x0A01);
	regWrite(phydev, 7, 0x8031, 0x0C01);
}

// Check current master/slave setting
// @param phydev address of the PHY
// @return true if master, false if slave
static bool isMaster(struct phy_device *phydev)
{
	return (0x0 != (regRead(phydev, 7, 0x8001) & 0x4000));
}

// Software Reset procedure
// @param phydev address of the PHY
// @return void
static void softReset(struct phy_device *phydev)
{
	phydev_info(phydev, "SoftReset\n");
	int regData = regRead(phydev, 1, 0x0000);

	regData |= 1 << 11;
	regWrite(phydev, 1, 0x0000, regData);
	regWrite(phydev, 3, 0xFFE4, 0x000C);
	msleep(1000);
	regWrite(phydev, 3, 0xFFE4, 0x06B6);
	regData &= ~(1 << 11);

	regWrite(phydev, 1, 0x0000, regData);
	msleep(1000);
	regWrite(phydev, 3, 0xFC47, 0x0030);
	regWrite(phydev, 3, 0xFC47, 0x0031);
	regWrite(phydev, 3, 0xFC47, 0x0030);
	regWrite(phydev, 3, 0xFC47, 0x0000);
	regWrite(phydev, 3, 0xFC47, 0x0001);
	regWrite(phydev, 3, 0xFC47, 0x0000);
	regWrite(phydev, 3, 0x0900, 0x8000);
	regWrite(phydev, 1, 0x0900, 0x0000);
	regWrite(phydev, 3, 0xFFE4, 0x000C);
}

// Initialize PHY
// @param phydev address of the PHY
// @return void
static int m88Q2122_config_init(struct phy_device *phydev)
{
	setMasterSlave(phydev, 1);

	phydev_info(phydev, "Marvell m88Q2122_config_init!\n");

	int regData = 0;

	regData = regRead(phydev, 1, 0x0002);
	regData = regRead(phydev, 1, 0x0003);
	msleep(2000);
	regWrite(phydev, 1, 0x0900, 0x4000);
	regWrite(phydev, 7, 0x0200, 0x0000);

	regData = regRead(phydev, 1, 0x0834);
	regData = (regData & 0xFFF0) | 0x0001;
	regWrite(phydev, 1, 0x0834, regData);

	regWrite(phydev, 3, 0xFFE4, 0x07B5);
	regWrite(phydev, 3, 0xFFE4, 0x06B6);
	msleep(5000);
	regWrite(phydev, 3, 0xFFDE, 0x402F);
	regWrite(phydev, 3, 0xFE2A, 0x3C3D);
	regWrite(phydev, 3, 0xFE34, 0x4040);
	regWrite(phydev, 3, 0xFE4B, 0x9337);
	regWrite(phydev, 3, 0xFE2A, 0x3C1D);
	regWrite(phydev, 3, 0xFE34, 0x0040);
	regWrite(phydev, 3, 0xFE0F, 0x0000);
	regWrite(phydev, 3, 0xFC00, 0x01C0);
	regWrite(phydev, 3, 0xFC17, 0x0425);
	regWrite(phydev, 3, 0xFC94, 0x5470);
	regWrite(phydev, 3, 0xFC95, 0x0055);
	regWrite(phydev, 3, 0xFC19, 0x08d8);
	regWrite(phydev, 3, 0xFC1A, 0x0110);
	regWrite(phydev, 3, 0xFC1B, 0x0a10);
	regWrite(phydev, 3, 0xFC3A, 0x2725);
	regWrite(phydev, 3, 0xFC61, 0x2627);
	regWrite(phydev, 3, 0xFC3B, 0x1612);
	regWrite(phydev, 3, 0xFC62, 0x1C12);
	regWrite(phydev, 3, 0xFC9D, 0x6367);
	regWrite(phydev, 3, 0xFC9E, 0x8060);
	regWrite(phydev, 3, 0xFC00, 0x01C8);
	regWrite(phydev, 3, 0x8000, 0x0000);
	regWrite(phydev, 3, 0x8016, 0x0011);
	regWrite(phydev, 3, 0xFDA3, 0x1800);
	regWrite(phydev, 3, 0xFE02, 0x00C0);
	regWrite(phydev, 3, 0xFFDB, 0x0010);
	regWrite(phydev, 3, 0xFFF3, 0x0020);
	regWrite(phydev, 3, 0xFE40, 0x00A6);
	regWrite(phydev, 3, 0xFE60, 0x0000);
	regWrite(phydev, 3, 0xFE2A, 0x3C3D);
	regWrite(phydev, 3, 0xFE4B, 0x9334);
	regWrite(phydev, 3, 0xFC10, 0xF600);
	regWrite(phydev, 3, 0xFC11, 0x073D);
	regWrite(phydev, 3, 0xFC12, 0x000D);
	regWrite(phydev, 3, 0xFC13, 0x0010);
	//LPSD feature
	if (MRVL_Q212X_LPSD_FEATURE_ENABLE){
		if (isMaster(phydev)){
			regWrite(phydev, 7, 0x8032, 0x005A);
		}
		else{
			regWrite(phydev, 7, 0x8032, 0x0064);
		}
		regWrite(phydev, 7, 0x8031, 0x0A01);
		regWrite(phydev, 7, 0x8031, 0x0C01);
		regWrite(phydev, 3, 0x800C, 0x0008);
		regWrite(phydev, 7, 0x8032, 0x0001);
		regWrite(phydev, 7, 0x8031, 0x0A1B);
		regWrite(phydev, 7, 0x8031, 0x0C1B);
		regWrite(phydev, 7, 0x8032, 0x000B);
		regWrite(phydev, 7, 0x8031, 0x0A1C);
		regWrite(phydev, 7, 0x8031, 0x0C1C);
		regWrite(phydev, 3, 0xFE04, 0x0016);
	}
	else{
		regWrite(phydev, 7, 0x8032, 0x0064);
		regWrite(phydev, 7, 0x8031, 0x0A01);
		regWrite(phydev, 7, 0x8031, 0x0C01);
		regWrite(phydev, 3, 0x800C, 0x0000);
		regWrite(phydev, 7, 0x8032, 0x0002);
		regWrite(phydev, 7, 0x8031, 0x0A1B);
		regWrite(phydev, 7, 0x8031, 0x0C1B);
		regWrite(phydev, 7, 0x8032, 0x0003);
		regWrite(phydev, 7, 0x8031, 0x0A1C);
		regWrite(phydev, 7, 0x8031, 0x0C1C);
		regWrite(phydev, 3, 0xFE04, 0x0008);
	}

	softReset(phydev);

	return 0;
}

static int m88Q2122_config_aneg(struct phy_device *phydev)
{
	return 0;
}

static int m88Q2122_read_status(struct phy_device *phydev)
{
	phydev->duplex = DUPLEX_FULL;
	phydev->speed = SPEED_1000;
	phydev->pause = 0;
	phydev->asym_pause = 0;

	phydev->link = 1;
	return 0;
}
static int m88Q2122_ack_interrupt(struct phy_device *phydev)
{
	return 0;
}
static int m88Q2122_config_intr(struct phy_device *phydev)
{
	return 0;
}

static int m88Q2122_did_interrupt(struct phy_device *phydev)
{
	return 0;
}

static int m88Q2122_aneg_done(struct phy_device *phydev)
{
	return 1;
}
static struct phy_driver marvell_88Q2122_drivers[] = {
	{
		.phy_id = MARVELL_PHY_ID_88Q2122,
		.phy_id_mask = MARVELL_PHY_ID_88Q2122_MASK,
		.name = "Marvell 88Q2122",
		.features = PHY_BASIC_T1_FEATURES,
		.probe = m88Q2122_probe,
		.config_init = &m88Q2122_config_init,
		.config_aneg = &m88Q2122_config_aneg,
		.read_status = &m88Q2122_read_status,
		.ack_interrupt = &m88Q2122_ack_interrupt,
		.config_intr = &m88Q2122_config_intr,
		.did_interrupt = &m88Q2122_did_interrupt,
		.aneg_done = &m88Q2122_aneg_done,
	},
};

module_phy_driver(marvell_88Q2122_drivers);

static struct mdio_device_id __maybe_unused marvell_88Q2122_tbl[] = {
	{ MARVELL_PHY_ID_88Q2122, MARVELL_PHY_ID_88Q2122_MASK },
	{ }
};

MODULE_DEVICE_TABLE(mdio, marvell_88Q2122_tbl);
