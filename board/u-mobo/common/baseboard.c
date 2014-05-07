/*
 * Copyright (C) 2014 U-MoBo
 * Pierluigi Passaro <p.passaro@u-mobo.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <asm/errno.h>
#include <power/pmic.h>
#include <power/id95apm_pmic.h>

#include "baseboard.h"

int baseboard_init(int bus)
{
	int ret;
	unsigned int value;
	struct pmic *p;

	ret = i2c_set_bus_num(bus);
	if (ret) {
		printf("IDT P95020 pmic bus not set!\n");
		return -ENODEV;
	}

	ret = i2c_probe(CONFIG_SYS_ID95APM_PMIC_I2C_ADDR);
	if (ret) {
		printf("IDT P95020 pmic not probed!\n");
		return -ENODEV;
	}

	ret = pmic_id95apm_init(bus, CONFIG_SYS_ID95APM_PMIC_I2C_ADDR);
	if (ret)
		return ret;

	p = pmic_get("ID95APM_PMIC");
	if (!p) {
		printf("IDT P95020 pmic not initialized!\n");
		return -ENODEV;
	}

	pmic_reg_read(p, ID95APM_GLOB_RESET_ID, &value);
	if (value != 0x55) {
		printf("IDT P95020 invalid ID (0x%02x)!\n", value);
		return -ENODEV;
	}

	// Set page #0
	value = 0x00;
	pmic_reg_write(p, ID95APM_GLOB_PAGE_CTRL, value);

	// Set input current limit to 2000mA
	value = 0x80 | 0x04;
	pmic_reg_write(p, ID95APM_CHGR_IN_CUR_LIMIT, value);

	// Switch on 3V3_LDO (LDO_150_0 @3.3V) for RS232 interface (among others)
	value = 0x80 | 0x66;
	pmic_reg_write(p, ID95APM_LDO_150MA_00, value);
	pmic_reg_read(p, ID95APM_GLOB_LDO_ENABLE, &value);
	value |= 0x10;
	pmic_reg_write(p, ID95APM_GLOB_LDO_ENABLE, value);

	// Set clock output 24MHz on USB_CLK
	value = 0x06;
	pmic_reg_write(p, ID95APM_PCON_CKGEN, value);

	/*
	pmic_reg_read(p, ID95APM_PCON_PLL_CFG, &value);
	value &= 0x07;
	value |= 0x83;
	pmic_reg_write(p, ID95APM_PCON_PLL_CFG, value);

	do
	{
		pmic_reg_read(p, ID95APM_PCON_PLL_STAT, &value);
	} while (!(value & 1));

	value = 0x02;
	pmic_reg_write(p, ID95APM_PCON_CKGEN, value);
	*/

	return 0;
}

int baseboard_enable_backlight(int on)
{
	unsigned int value;
	struct pmic *p;

	p = pmic_get("ID95APM_PMIC");
	if (!p) {
		printf("IDT P95020 pmic not initialized!\n");
		return -ENODEV;
	}

	// Set 25mA (Full Scale), Enabled
	value = 0xDF;
	pmic_reg_write(p, ID95APM_DCDC_LED_BOOST, value);

	// Enable LED_BOOST DCDC
	pmic_reg_read(p, ID95APM_GLOB_DCDC_ENABLE, &value);
	if (on)
		value |= 0x10;
	else
		value &= ~0x10;
	pmic_reg_write(p, ID95APM_GLOB_DCDC_ENABLE, value);

	return 0;
}

int baseboard_enable_lcd(int on)
{
	unsigned int value;
	struct pmic *p;

	p = pmic_get("ID95APM_PMIC");
	if (!p) {
		printf("IDT P95020 pmic not initialized!\n");
		return -ENODEV;
	}

	// Enable BUCK1000 DCDC (globally)
	pmic_reg_read(p, ID95APM_GLOB_DCDC_ENABLE, &value);
	if (on)
		value |= 0x04;
	else
		value &= ~0x04;
	pmic_reg_write(p, ID95APM_GLOB_DCDC_ENABLE, value);

	// Enable BUCK1000
	value = 0x66;
	if (on)
		value |= 0x80;
	pmic_reg_write(p, ID95APM_DCDC_BUCK1000, value);

	// Switch on 3V3_LCD_ON (LDO_50_1 @3.3V)
	value = 0x66;
	if (on)
		value = 0x80;
	pmic_reg_write(p, ID95APM_LDO_50MA_05, value);
	pmic_reg_read(p, ID95APM_GLOB_LDO_ENABLE, &value);
	if (on)
		value |= 0x04;
	else
		value &= ~0x04;
	pmic_reg_write(p, ID95APM_GLOB_LDO_ENABLE, value);

	return 0;
}

int baseboard_enable_usb(int on)
{
	unsigned int value;
	struct pmic *p;

	p = pmic_get("ID95APM_PMIC");
	if (!p) {
		printf("IDT P95020 pmic not initialized!\n");
		return -ENODEV;
	}

	// Enable BUCK500_0 DCDC (globally)
	pmic_reg_read(p, ID95APM_GLOB_DCDC_ENABLE, &value);
	if (on)
		value |= 0x01;
	else
		value &= ~0x01;
	pmic_reg_write(p, ID95APM_GLOB_DCDC_ENABLE, value);

	// Enable BUCK500_0 (3V3_HUB)
	value = 0x66;
	if (on)
		value |= 0x80;
	pmic_reg_write(p, ID95APM_DCDC_BUCK500_0, value);

	// Enable BOOST5 DCDC (globally)
	pmic_reg_read(p, ID95APM_GLOB_DCDC_ENABLE, &value);
	if (on)
		value |= 0x08;
	else
		value &= ~0x08;
	pmic_reg_write(p, ID95APM_GLOB_DCDC_ENABLE, value);

	// Enable BOOST5
	value = 0x13;
	if (on)
		value |= 0x80;
	pmic_reg_write(p, ID95APM_DCDC_BOOST5, value);

	return 0;
}
