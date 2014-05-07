/*
 * Copyright (C) 2014 U-MoBo
 * Pierluigi Passaro <p.passaro@u-mobo.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <power/pmic.h>
#include <power/id95apm_pmic.h>
#include <errno.h>

int pmic_id95apm_init(unsigned char bus, unsigned char i2c_addr)
{
	static const char name[] = "ID95APM_PMIC";
	struct pmic *p = pmic_alloc();

	if (!p) {
		printf("%s: POWER allocation error!\n", __func__);
		return -ENOMEM;
	}

	p->name = name;
	p->number_of_regs = ID95APM_NUM_OF_REGS;
	p->interface = PMIC_I2C;
	p->hw.i2c.addr = i2c_addr;
	p->hw.i2c.tx_num = 1;
	p->bus = bus;

	return 0;
}
