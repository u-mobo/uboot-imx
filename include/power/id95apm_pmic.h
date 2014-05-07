/*
 * IDT P95020 declarations.
 *
 * Copyright(c) 2014 U-MoBo
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __ID95APM_PMIC_H__
#define __ID95APM_PMIC_H__

#define ID95APM_GLOB_RESET_ID		0x00
#define ID95APM_GLOB_PAGE_CTRL		0x01
#define ID95APM_GLOB_LDO_ENABLE		0x04
#define ID95APM_GLOB_DCDC_ENABLE	0x05
#define ID95APM_PCON_PLL_CFG		0x34
#define ID95APM_PCON_PLL_STAT		0x35
#define ID95APM_PCON_CKGEN		0x3d
#define ID95APM_LDO_150MA_00		0x60
#define ID95APM_LDO_50MA_05		0x6a
#define ID95APM_DCDC_BUCK500_0		0x80
#define ID95APM_DCDC_BUCK500_1		0x82
#define ID95APM_DCDC_BUCK1000		0x84
#define ID95APM_DCDC_LED_BOOST		0x86
#define ID95APM_DCDC_BOOST5		0x88
#define ID95APM_CHGR_IN_CUR_LIMIT	0x90

#define ID95APM_NUM_OF_REGS		0xff

int pmic_id95apm_init(unsigned char bus, unsigned char i2c_addr);

#endif /* __ID95APM_PMIC_H__ */
