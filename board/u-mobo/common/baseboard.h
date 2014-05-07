/*
 * Copyright (C) 2014 U-MoBo
 * Pierluigi Passaro <p.passaro@u-mobo.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __BASEBOARD_H
#define __BASEBOARD_H

int baseboard_init(int bus);
int baseboard_enable_backlight(int on);
int baseboard_enable_lcd(int on);
int baseboard_enable_usb(int on);

#endif /* __BASEBOARD_H */
