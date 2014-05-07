/*
 * Copyright (C) 2014 U-MoBo
 * Pierluigi Passaro <p.passaro@u-mobo.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <linux/list.h>
#include <asm/gpio.h>
#include <asm/arch/iomux-mx53.h>
#include <asm/imx-common/video.h>
#include <i2c.h>
#include <linux/fb.h>
#include <ipu_pixfmt.h>
#include <pca953x.h>

#include "../common/baseboard.h"

/* IO Expander pins */
#define IO_EXP_VID1_PWREN	(8)
#define IO_EXP_VID1_RESET	(9)
#define IO_EXP_VID2_PWREN	(10)
#define IO_EXP_VID2_RESET	(11)

static int detect_i2c(struct display_info_t const *dev)
{
	return ((0 == i2c_set_bus_num(dev->bus))
		&&
		(0 == i2c_probe(dev->addr)));
}

static void enable_rgb(struct display_info_t const *dev)
{
	/* power on LCD */
	pca953x_set_val(CONFIG_SYS_I2C_PCA953X_ADDR, 1 << IO_EXP_VID1_PWREN, PCA953X_OUT_HIGH << IO_EXP_VID1_PWREN);
	pca953x_set_val(CONFIG_SYS_I2C_PCA953X_ADDR, 1 << IO_EXP_VID1_RESET, PCA953X_OUT_HIGH << IO_EXP_VID1_RESET);

#ifdef CONFIG_UMOBO_BASEBOARD
	baseboard_enable_backlight(1);	/* Turn on backlight */
	baseboard_enable_lcd(1);	/* Turn on display */
#endif
}

struct display_info_t const displays[] = {{
	.bus	= -1,
	.addr	= 0,
	.pixfmt	= IPU_PIX_FMT_RGB666,
	.detect	= NULL,
	.enable	= enable_rgb,
	.mode	= {
		.name		= "CLAA07LC0ACW",
		.refresh	= 57,
		.xres		= 800,
		.yres		= 480,
		.pixclock	= 37037,
		.left_margin	= 40,
		.right_margin	= 60,
		.upper_margin	= 10,
		.lower_margin	= 10,
		.hsync_len	= 20,
		.vsync_len	= 10,
		.sync		= 0,
		.vmode		= FB_VMODE_NONINTERLACED
} }, {
	.bus	= -1,
	.addr	= 0,
	.pixfmt	= IPU_PIX_FMT_RGB666,
	.detect	= NULL,
	.enable	= enable_rgb,
	.mode	= {
		.name		= "Seiko-43WVF1G",
		.refresh	= 60,
		.xres		= 800,
		.yres		= 480,
		.pixclock	= 29851, /* picosecond (33.5 MHz) */
		.left_margin	= 89,
		.right_margin	= 164,
		.upper_margin	= 23,
		.lower_margin	= 10,
		.hsync_len	= 10,
		.vsync_len	= 10,
		.sync		= 0,
		.vmode		= FB_VMODE_NONINTERLACED
} }, {
	.bus	= 0,
	.addr	= 0x2a,	/* hack: actually ID95APM addr */
	.pixfmt	= IPU_PIX_FMT_RGB24,
	.detect	= detect_i2c,
	.enable	= enable_rgb,
	.mode	= {
		.name		= "SAMSUNG-LMS700",
		.refresh	= 60,
		.xres		= 800,
		.yres		= 480,
		.pixclock	= 40816, /* picosecond (24.5 MHz) */
		.left_margin	= 16,
		.right_margin	= 8,
		.upper_margin	= 4,
		.lower_margin	= 9,
		.hsync_len	= 8,
		.vsync_len	= 4,
		.sync		= 0,
		.vmode		= FB_VMODE_NONINTERLACED
} } };
size_t display_count = ARRAY_SIZE(displays);

void setup_iomux_lcd(void)
{
	static const iomux_v3_cfg_t lcd_pads[] = {
		MX53_PAD_DI0_DISP_CLK__IPU_DI0_DISP_CLK,
		MX53_PAD_DI0_PIN15__IPU_DI0_PIN15,
		MX53_PAD_DI0_PIN2__IPU_DI0_PIN2,
		MX53_PAD_DI0_PIN3__IPU_DI0_PIN3,
		MX53_PAD_DISP0_DAT0__IPU_DISP0_DAT_0,
		MX53_PAD_DISP0_DAT1__IPU_DISP0_DAT_1,
		MX53_PAD_DISP0_DAT2__IPU_DISP0_DAT_2,
		MX53_PAD_DISP0_DAT3__IPU_DISP0_DAT_3,
		MX53_PAD_DISP0_DAT4__IPU_DISP0_DAT_4,
		MX53_PAD_DISP0_DAT5__IPU_DISP0_DAT_5,
		MX53_PAD_DISP0_DAT6__IPU_DISP0_DAT_6,
		MX53_PAD_DISP0_DAT7__IPU_DISP0_DAT_7,
		MX53_PAD_DISP0_DAT8__IPU_DISP0_DAT_8,
		MX53_PAD_DISP0_DAT9__IPU_DISP0_DAT_9,
		MX53_PAD_DISP0_DAT10__IPU_DISP0_DAT_10,
		MX53_PAD_DISP0_DAT11__IPU_DISP0_DAT_11,
		MX53_PAD_DISP0_DAT12__IPU_DISP0_DAT_12,
		MX53_PAD_DISP0_DAT13__IPU_DISP0_DAT_13,
		MX53_PAD_DISP0_DAT14__IPU_DISP0_DAT_14,
		MX53_PAD_DISP0_DAT15__IPU_DISP0_DAT_15,
		MX53_PAD_DISP0_DAT16__IPU_DISP0_DAT_16,
		MX53_PAD_DISP0_DAT17__IPU_DISP0_DAT_17,
		MX53_PAD_DISP0_DAT18__IPU_DISP0_DAT_18,
		MX53_PAD_DISP0_DAT19__IPU_DISP0_DAT_19,
		MX53_PAD_DISP0_DAT20__IPU_DISP0_DAT_20,
		MX53_PAD_DISP0_DAT21__IPU_DISP0_DAT_21,
		MX53_PAD_DISP0_DAT22__IPU_DISP0_DAT_22,
		MX53_PAD_DISP0_DAT23__IPU_DISP0_DAT_23,
	};

	imx_iomux_v3_setup_multiple_pads(lcd_pads, ARRAY_SIZE(lcd_pads));
}
