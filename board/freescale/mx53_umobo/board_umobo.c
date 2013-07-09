/*
 * Copyright (C) 2012 U-MoBo Srl
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <common.h>
#include <asm/io.h>
#include <asm/arch/mx53.h>
#include <asm/errno.h>

#ifdef CONFIG_I2C_MXC
#include <i2c.h>

#define BASEBOARD_I2C_ADDRESS  0x2A

#define ID95APM_GLOB_RESET_ID	   0x00
#define ID95APM_GLOB_PAGE_CTRL	  0x01
#define ID95APM_GLOB_LDO_ENABLE	 0x04
#define ID95APM_GLOB_DCDC_ENABLE	0x05
#define ID95APM_LDO_150MA_00		0x60
#define ID95APM_LDO_50MA_05		 0x6a
#define ID95APM_DCDC_LED_BOOST	  0x86
#define ID95APM_DCDC_BUCK1000	   0x84
#define ID95APM_CHGR_IN_CUR_LIMIT   0x90

#define ID95APM_PCON_PLL_CFG		0x34
#define ID95APM_PCON_PLL_STAT	   0x35
#define ID95APM_PCON_CKGEN		  0x3d

#define i2c_read_byte(addr, reg, ptr)	i2c_read(addr, reg, 1, ptr, 1);
#define i2c_write_byte(addr, reg, ptr)	i2c_write(addr, reg, 1, ptr, 1);

static int umobo_board_init(void)
{
	uchar value = 0;
	i2c_read_byte(BASEBOARD_I2C_ADDRESS, ID95APM_GLOB_RESET_ID, &value);

	if (value != 0x55)
	{
		printf("umobo_board_init: invalid P95020 ID (0x%02x)!\n", value);
		return -ENODEV;
	}

	// Set page #0
	value = 0x00;
	i2c_write_byte(BASEBOARD_I2C_ADDRESS, ID95APM_GLOB_PAGE_CTRL, &value);

	// Set input current limit to 2000mA
	value = 0x80 | 0x04;
	i2c_write_byte(BASEBOARD_I2C_ADDRESS, ID95APM_CHGR_IN_CUR_LIMIT, &value);

	// Switch on 3V3_LDO (LDO_150_0 @3.3V) for RS232 interface (among others)
	value = 0x80 | 0x66;
	i2c_write_byte(BASEBOARD_I2C_ADDRESS, ID95APM_LDO_150MA_00, &value);
	i2c_read_byte(BASEBOARD_I2C_ADDRESS, ID95APM_GLOB_LDO_ENABLE, &value);
	value |= 0x10;
	i2c_write_byte(BASEBOARD_I2C_ADDRESS, ID95APM_GLOB_LDO_ENABLE, &value);

	// Set clock output 24MHz on USB_CLK
	value = 0x06;
	i2c_write_byte(BASEBOARD_I2C_ADDRESS, ID95APM_PCON_CKGEN, &value);

	/*
	i2c_read_byte(BASEBOARD_I2C_ADDRESS, ID95APM_PCON_PLL_CFG, &value);
	value &= 0x07;
	value |= 0x83;
	i2c_write_byte(BASEBOARD_I2C_ADDRESS, ID95APM_PCON_PLL_CFG, &value);

	do
	{
		i2c_read_byte(BASEBOARD_I2C_ADDRESS, ID95APM_PCON_PLL_STAT, &value);
	} while (!(value & 1));

	value = 0x02;
	i2c_write_byte(BASEBOARD_I2C_ADDRESS, ID95APM_PCON_CKGEN, &value);
	*/

	return 0;
}

static void umobo_enable_backlight(int on)
{
	uchar value = 0;

	// Set 25mA (Full Scale), Enabled
	value = 0xDF;
	i2c_write_byte(BASEBOARD_I2C_ADDRESS, ID95APM_DCDC_LED_BOOST, &value);

	// Enable LED_BOOST DCDC
	i2c_read_byte(BASEBOARD_I2C_ADDRESS, ID95APM_GLOB_DCDC_ENABLE, &value);
	if (on)
		value |= 0x10;
	else
		value &= ~0x10;
	i2c_write_byte(BASEBOARD_I2C_ADDRESS, ID95APM_GLOB_DCDC_ENABLE, &value);
}

static void umobo_enable_lcd(int on)
{
	uchar value;

	// Enable BUCK1000 DCDC (globally)
	i2c_read_byte(BASEBOARD_I2C_ADDRESS, ID95APM_GLOB_DCDC_ENABLE, &value);
	value |= 0x04;
	i2c_write_byte(BASEBOARD_I2C_ADDRESS, ID95APM_GLOB_DCDC_ENABLE, &value);

	// Enable BUCK1000
	i2c_read_byte(BASEBOARD_I2C_ADDRESS, ID95APM_DCDC_BUCK1000 + 1, &value);
	value = 0x80 | 0x66;
	i2c_write_byte(BASEBOARD_I2C_ADDRESS, ID95APM_DCDC_BUCK1000, &value);

	// Switch on 3V3_LCD_ON (LDO_50_1 @3.3V)
	value = 0x80 | 0x66;
	i2c_write_byte(BASEBOARD_I2C_ADDRESS, ID95APM_LDO_50MA_05, &value);
	i2c_read_byte(BASEBOARD_I2C_ADDRESS, ID95APM_GLOB_LDO_ENABLE, &value);
	if (on)
		value |= 0x04;
	else
		value &= ~0x04;
	i2c_write_byte(BASEBOARD_I2C_ADDRESS, ID95APM_GLOB_LDO_ENABLE, &value);

	i2c_read_byte(BASEBOARD_I2C_ADDRESS, ID95APM_DCDC_BUCK1000 + 1, &value);
}

void setup_umobo_board(void)
{
	if (umobo_board_init() == 0) {
		umobo_enable_backlight(0);
		umobo_enable_lcd(0);
	}
}

#endif /* CONFIG_I2C_MXC */
