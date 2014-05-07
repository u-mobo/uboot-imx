/*
 * Copyright (C) 2014 U-MoBo
 * Pierluigi Passaro <p.passaro@u-mobo.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <asm/io.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/sys_proto.h>
#include <asm/arch/crm_regs.h>
#include <asm/arch/clock.h>
#include <asm/arch/iomux-mx53.h>
#include <asm/arch/clock.h>
#include <asm/errno.h>
#include <asm/imx-common/mx5_video.h>
#include <asm/imx-common/mxc_i2c.h>
#include <netdev.h>
#include <i2c.h>
#include <mmc.h>
#include <fsl_esdhc.h>
#include <asm/gpio.h>
#include <power/pmic.h>
#include <dialog_pmic.h>
#include <pca953x.h>

#include "../common/baseboard.h"

/* IO Expander pins */
#define IO_EXP_VID1_PWREN	(8)
#define IO_EXP_VID1_RESET	(9)
#define IO_EXP_VID2_PWREN	(10)
#define IO_EXP_VID2_RESET	(11)
#define IO_EXP_CIS_RESET	(12)
#define IO_EXP_CIS_CONTROL	(13)
#define IO_EXP_OTG_VBUSEN	(14)
#define IO_EXP_USBH_VBUSEN	(15)

DECLARE_GLOBAL_DATA_PTR;

static uint32_t mx53_dram_size[2];

phys_size_t get_effective_memsize(void)
{
	/*
	 * WARNING: We must override get_effective_memsize() function here
	 * to report only the size of the first DRAM bank. This is to make
	 * U-Boot relocator place U-Boot into valid memory, that is, at the
	 * end of the first DRAM bank. If we did not override this function
	 * like so, U-Boot would be placed at the address of the first DRAM
	 * bank + total DRAM size - sizeof(uboot), which in the setup where
	 * each DRAM bank contains 512MiB of DRAM would result in placing
	 * U-Boot into invalid memory area close to the end of the first
	 * DRAM bank.
	 */
	return mx53_dram_size[0];
}

int dram_init(void)
{
	mx53_dram_size[0] = get_ram_size((void *)PHYS_SDRAM_1, 1 << 30);
	mx53_dram_size[1] = get_ram_size((void *)PHYS_SDRAM_2, 1 << 30);

	gd->ram_size = mx53_dram_size[0] + mx53_dram_size[1];

	return 0;
}

void dram_init_banksize(void)
{
	gd->bd->bi_dram[0].start = PHYS_SDRAM_1;
	gd->bd->bi_dram[0].size = mx53_dram_size[0];

	gd->bd->bi_dram[1].start = PHYS_SDRAM_2;
	gd->bd->bi_dram[1].size = mx53_dram_size[1];
}

u32 get_board_rev(void)
{
	int rev = 0;

	return (get_cpu_rev() & ~(0xF << 8)) | (rev & 0xF) << 8;
}

#define UART_PAD_CTRL	(PAD_CTL_HYS | PAD_CTL_DSE_HIGH | \
			 PAD_CTL_PUS_100K_UP | PAD_CTL_ODE)

static void setup_iomux_uart(void)
{
	static const iomux_v3_cfg_t uart_pads[] = {
		NEW_PAD_CTRL(MX53_PAD_KEY_ROW0__UART4_RXD_MUX, UART_PAD_CTRL),
		NEW_PAD_CTRL(MX53_PAD_KEY_COL0__UART4_TXD_MUX, UART_PAD_CTRL),
	};

	imx_iomux_v3_setup_multiple_pads(uart_pads, ARRAY_SIZE(uart_pads));
}

#ifdef CONFIG_USB_EHCI_MX5
int board_ehci_hcd_init(int port)
{
#ifdef CONFIG_UMOBO_BASEBOARD
	baseboard_enable_usb(1);	/* Turn on USB */
#endif
	/* reset USBH and OTG via VBUSEN pins */
	pca953x_set_dir(CONFIG_SYS_I2C_PCA953X_ADDR, 1 << IO_EXP_OTG_VBUSEN, PCA953X_DIR_OUT << IO_EXP_OTG_VBUSEN);
	pca953x_set_dir(CONFIG_SYS_I2C_PCA953X_ADDR, 1 << IO_EXP_USBH_VBUSEN, PCA953X_DIR_OUT << IO_EXP_USBH_VBUSEN);
	pca953x_set_val(CONFIG_SYS_I2C_PCA953X_ADDR, 1 << IO_EXP_OTG_VBUSEN, PCA953X_OUT_LOW << IO_EXP_OTG_VBUSEN);
	pca953x_set_val(CONFIG_SYS_I2C_PCA953X_ADDR, 1 << IO_EXP_USBH_VBUSEN, PCA953X_OUT_LOW << IO_EXP_USBH_VBUSEN);
	mdelay(2);
	pca953x_set_val(CONFIG_SYS_I2C_PCA953X_ADDR, 1 << IO_EXP_OTG_VBUSEN, PCA953X_OUT_HIGH << IO_EXP_OTG_VBUSEN);
	pca953x_set_val(CONFIG_SYS_I2C_PCA953X_ADDR, 1 << IO_EXP_USBH_VBUSEN, PCA953X_OUT_HIGH << IO_EXP_USBH_VBUSEN);
	mdelay(20);

	return 0;
}
#endif

#define FEC_RESET IMX_GPIO_NR(5, 20)

static void setup_iomux_fec(void)
{
	static const iomux_v3_cfg_t fec_pads[] = {
		NEW_PAD_CTRL(MX53_PAD_FEC_MDIO__FEC_MDIO, PAD_CTL_HYS |
			PAD_CTL_DSE_HIGH | PAD_CTL_PUS_22K_UP | PAD_CTL_ODE),
		NEW_PAD_CTRL(MX53_PAD_FEC_MDC__FEC_MDC, PAD_CTL_DSE_HIGH),
		NEW_PAD_CTRL(MX53_PAD_FEC_RXD1__FEC_RDATA_1,
				PAD_CTL_HYS | PAD_CTL_PKE),
		NEW_PAD_CTRL(MX53_PAD_FEC_RXD0__FEC_RDATA_0,
				PAD_CTL_HYS | PAD_CTL_PKE),
		NEW_PAD_CTRL(MX53_PAD_FEC_TXD1__FEC_TDATA_1, PAD_CTL_DSE_HIGH),
		NEW_PAD_CTRL(MX53_PAD_FEC_TXD0__FEC_TDATA_0, PAD_CTL_DSE_HIGH),
		NEW_PAD_CTRL(MX53_PAD_FEC_TX_EN__FEC_TX_EN, PAD_CTL_DSE_HIGH),
		NEW_PAD_CTRL(MX53_PAD_FEC_REF_CLK__FEC_TX_CLK,
				PAD_CTL_HYS | PAD_CTL_PKE),
		NEW_PAD_CTRL(MX53_PAD_FEC_RX_ER__FEC_RX_ER,
				PAD_CTL_HYS | PAD_CTL_PKE),
		NEW_PAD_CTRL(MX53_PAD_FEC_CRS_DV__FEC_RX_DV,
				PAD_CTL_HYS | PAD_CTL_PKE),
		MX53_PAD_CSI0_DATA_EN__GPIO5_20,
	};

	imx_iomux_v3_setup_multiple_pads(fec_pads, ARRAY_SIZE(fec_pads));
}

#ifdef NEED_A_TEST_GPIO
static void test_loop(unsigned int period_us, unsigned int cycles)
{
	int i;

	period_us = period_us >> 1;
	for (i = 0; i < cycles; i++) {
		gpio_direction_output(FEC_RESET, 0);
		udelay(period_us);
		gpio_direction_output(FEC_RESET, 1);
		udelay(period_us);
	}
}
#endif

void reset_phy(void)
{
	/* called after board_late_init, when VDDIO_FEC is already setup */
	gpio_direction_output(FEC_RESET, 0);
	udelay(500);
	gpio_direction_output(FEC_RESET, 1);
}

#ifdef CONFIG_FSL_ESDHC
#define SDHC2_CD IMX_GPIO_NR(1, 4)
struct fsl_esdhc_cfg esdhc_cfg[2] = {
	{MMC_SDHC1_BASE_ADDR},
	{MMC_SDHC2_BASE_ADDR},
};

int board_mmc_getcd(struct mmc *mmc)
{
	struct fsl_esdhc_cfg *cfg = (struct fsl_esdhc_cfg *)mmc->priv;
	int ret;

	if (cfg->esdhc_base == MMC_SDHC1_BASE_ADDR)
		ret = 1;
	else if (cfg->esdhc_base == MMC_SDHC2_BASE_ADDR)
		ret = !gpio_get_value(SDHC2_CD);
	else
		ret = 0;

	return ret;
}

#define SD_CMD_PAD_CTRL		(PAD_CTL_HYS | PAD_CTL_DSE_HIGH | \
				 PAD_CTL_PUS_100K_UP)
#define SD_PAD_CTRL		(PAD_CTL_HYS | PAD_CTL_PUS_47K_UP | \
				 PAD_CTL_DSE_HIGH)

int board_mmc_init(bd_t *bis)
{
	static const iomux_v3_cfg_t sd1_pads[] = {
		NEW_PAD_CTRL(MX53_PAD_SD1_CMD__ESDHC1_CMD, SD_CMD_PAD_CTRL),
		NEW_PAD_CTRL(MX53_PAD_SD1_CLK__ESDHC1_CLK, SD_PAD_CTRL),
		NEW_PAD_CTRL(MX53_PAD_SD1_DATA0__ESDHC1_DAT0, SD_PAD_CTRL),
		NEW_PAD_CTRL(MX53_PAD_SD1_DATA1__ESDHC1_DAT1, SD_PAD_CTRL),
		NEW_PAD_CTRL(MX53_PAD_SD1_DATA2__ESDHC1_DAT2, SD_PAD_CTRL),
		NEW_PAD_CTRL(MX53_PAD_SD1_DATA3__ESDHC1_DAT3, SD_PAD_CTRL),
	};

	static const iomux_v3_cfg_t sd2_pads[] = {
		NEW_PAD_CTRL(MX53_PAD_SD2_CMD__ESDHC2_CMD, SD_CMD_PAD_CTRL),
		NEW_PAD_CTRL(MX53_PAD_SD2_CLK__ESDHC2_CLK, SD_PAD_CTRL),
		NEW_PAD_CTRL(MX53_PAD_SD2_DATA0__ESDHC2_DAT0, SD_PAD_CTRL),
		NEW_PAD_CTRL(MX53_PAD_SD2_DATA1__ESDHC2_DAT1, SD_PAD_CTRL),
		NEW_PAD_CTRL(MX53_PAD_SD2_DATA2__ESDHC2_DAT2, SD_PAD_CTRL),
		NEW_PAD_CTRL(MX53_PAD_SD2_DATA3__ESDHC2_DAT3, SD_PAD_CTRL),
		MX53_PAD_GPIO_4__GPIO1_4,
	};

	u32 index;
	s32 status = 0;

	esdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC_CLK);
	esdhc_cfg[1].sdhc_clk = mxc_get_clock(MXC_ESDHC2_CLK);

	for (index = 0; index < CONFIG_SYS_FSL_ESDHC_NUM; index++) {
		switch (index) {
		case 0:
			imx_iomux_v3_setup_multiple_pads(sd1_pads,
							 ARRAY_SIZE(sd1_pads));
			break;
		case 1:
			imx_iomux_v3_setup_multiple_pads(sd2_pads,
							 ARRAY_SIZE(sd2_pads));
			gpio_direction_input(SDHC2_CD);
			break;
		default:
			printf("Warning: you configured more ESDHC controller"
				"(%d) as supported by the board(2)\n",
				CONFIG_SYS_FSL_ESDHC_NUM);
			return status;
		}
		status |= fsl_esdhc_initialize(bis, &esdhc_cfg[index]);
	}

	return status;
}
#endif

#define I2C_PAD_CTRL	(PAD_CTL_SRE_FAST | PAD_CTL_DSE_HIGH | \
			 PAD_CTL_PUS_100K_UP | PAD_CTL_ODE)

struct i2c_pads_info i2c_pad_info0 = {
	.scl = {
		.i2c_mode  = NEW_PAD_CTRL(MX53_PAD_CSI0_DAT9__I2C1_SCL, I2C_PAD_CTRL),
		.gpio_mode = NEW_PAD_CTRL(MX53_PAD_CSI0_DAT9__GPIO5_27, I2C_PAD_CTRL),
		.gp = IMX_GPIO_NR(5, 27)
	},
	.sda = {
		 .i2c_mode = NEW_PAD_CTRL(MX53_PAD_CSI0_DAT8__I2C1_SDA, I2C_PAD_CTRL),
		 .gpio_mode = NEW_PAD_CTRL(MX53_PAD_CSI0_DAT8__GPIO5_26, I2C_PAD_CTRL),
		 .gp = IMX_GPIO_NR(5, 26)
	 }
};

struct i2c_pads_info i2c_pad_info1 = {
	.scl = {
		.i2c_mode = NEW_PAD_CTRL(MX53_PAD_KEY_COL3__I2C2_SCL, I2C_PAD_CTRL),
		.gpio_mode = NEW_PAD_CTRL(MX53_PAD_KEY_COL3__GPIO4_12, I2C_PAD_CTRL),
		.gp = IMX_GPIO_NR(4, 12)
	},
	.sda = {
		.i2c_mode = NEW_PAD_CTRL(MX53_PAD_KEY_ROW3__I2C2_SDA, I2C_PAD_CTRL),
		.gpio_mode = NEW_PAD_CTRL(MX53_PAD_KEY_ROW3__GPIO4_13, I2C_PAD_CTRL),
		.gp = IMX_GPIO_NR(4, 13)
	}
};

struct i2c_pads_info i2c_pad_info2 = {
	.scl = {
		.i2c_mode = NEW_PAD_CTRL(MX53_PAD_GPIO_3__I2C3_SCL, I2C_PAD_CTRL),
		.gpio_mode = NEW_PAD_CTRL(MX53_PAD_GPIO_3__GPIO1_3, I2C_PAD_CTRL),
		.gp = IMX_GPIO_NR(1, 3)
	},
	.sda = {
		.i2c_mode = NEW_PAD_CTRL(MX53_PAD_GPIO_6__I2C3_SDA, I2C_PAD_CTRL),
		.gpio_mode = NEW_PAD_CTRL(MX53_PAD_GPIO_6__GPIO1_6, I2C_PAD_CTRL),
		.gp = IMX_GPIO_NR(1, 6)
	}
};

static int power_init(void)
{
	unsigned int val;
	int ret;
	struct pmic *p;
	int da9053_i2c_addr;
	char const *e = getenv("da9053_i2c_addr");

	da9053_i2c_addr = e ? (int)simple_strtol(e, NULL, 0) : CONFIG_SYS_DIALOG_PMIC_I2C_ADDR;
	if (!(da9053_i2c_addr & 0x0008) ||	/* bit 3 to one */
	     (da9053_i2c_addr & 0xff87)) {	/* only bits 3 to 6 non zero */
		printf("invalid DA9053 address 0x%x!\n", da9053_i2c_addr);
		da9053_i2c_addr = CONFIG_SYS_DIALOG_PMIC_I2C_ADDR;
	}
	i2c_set_bus_num(I2C_PMIC);
	if (i2c_probe(CONFIG_SYS_DIALOG_PMIC_I2C_ADDR)) {
		printf("DA9053 standard address 0x%x probe failed!\n", CONFIG_SYS_DIALOG_PMIC_I2C_ADDR);
		if (da9053_i2c_addr == CONFIG_SYS_DIALOG_PMIC_I2C_ADDR)
			return -ENODEV;
		if (i2c_probe(da9053_i2c_addr)) {
			printf("DA9053 custom address 0x%x probe failed!\n", da9053_i2c_addr);
			return -ENODEV;
		} else {
			printf("DA9053 custom address 0x%x probed successfully!\n", da9053_i2c_addr);
		}
	} else {
		printf("DA9053 standard address 0x%x probed successfully!\n", CONFIG_SYS_DIALOG_PMIC_I2C_ADDR);
		if (da9053_i2c_addr != CONFIG_SYS_DIALOG_PMIC_I2C_ADDR) {
			uchar value;
			i2c_read(CONFIG_SYS_DIALOG_PMIC_I2C_ADDR, DA9053_INTERFACE_REG, 1, &value, 1);
			value &= 0x1f;
			value |= ((da9053_i2c_addr & 0x0070) << 1);
			i2c_write(CONFIG_SYS_DIALOG_PMIC_I2C_ADDR, DA9053_INTERFACE_REG, 1, &value, 1);
			if (i2c_probe(da9053_i2c_addr)) {
				printf("DA9053 custom address 0x%x probe failed!\n", da9053_i2c_addr);
				return -ENODEV;
			} else {
				printf("DA9053 custom address 0x%x probed successfully!\n", da9053_i2c_addr);
			}
		}
	}

	ret = pmic_dialog_init(I2C_PMIC, da9053_i2c_addr);
	if (ret)
		return ret;

	p = pmic_get("DIALOG_PMIC");
	if (!p)
		return -ENODEV;

#ifdef CONFIG_CPU_1_2G
	/* Set VDDA to 1.30V for 1.2GHZ */
	val = DA9052_BUCKCORE_BCOREEN | DA_BUCKCORE_VBCORE_1_300V;
#else
	/* Set VDDA to 1.25V for 1GHZ */
	val = DA9052_BUCKCORE_BCOREEN | DA_BUCKCORE_VBCORE_1_250V;
#endif
	ret = pmic_reg_write(p, DA9053_BUCKCORE_REG, val);
	if (ret) {
		printf("Writing to BUCKCORE_REG failed: %d\n", ret);
		return ret;
	}

	pmic_reg_read(p, DA9053_SUPPLY_REG, &val);
	val |= DA9052_SUPPLY_VBCOREGO;
	ret = pmic_reg_write(p, DA9053_SUPPLY_REG, val);
	if (ret) {
		printf("Writing to DA9053_SUPPLY_REG failed: %d\n", ret);
		return ret;
	}

	/* Set Vcc peripheral to 1.30V */
	ret = pmic_reg_write(p, DA9053_BUCKPRO_REG, 0x62);
	if (ret) {
		printf("Writing to DA9053_BUCKPRO_REG failed: %d\n", ret);
		return ret;
	}

	val |= DA9052_SUPPLY_VBPROGO;
	ret = pmic_reg_write(p, DA9053_SUPPLY_REG, val);
	if (ret) {
		printf("Writing to DA9053_SUPPLY_REG failed: %d\n", ret);
		return ret;
	}

	/* set LDO6 to 3.3V for VDDIO_3V3_EXT */
	ret = pmic_reg_write(p, DA9053_LDO6_REG, 0x6a);
	if (ret) {
		printf("Writing to DA9053_LDO6_REG failed: %d\n", ret);
		return ret;
	}

	/* set LDO9 to 3.3V for VDDIO_FEC */
	ret = pmic_reg_write(p, DA9053_LDO9_REG, 0x69);
	if (ret) {
		printf("Writing to DA9053_LDO9_REG failed: %d\n", ret);
		return ret;
	}

	return ret;
}

static void clock_1GHz(void)
{
	int ret;
	u32 ref_clk = MXC_HCLK;
	int megaherts;
#ifdef CONFIG_CPU_1_2G
	megaherts = 1200;
#else
	megaherts = 1000;
#endif
	/*
	 * After increasing voltage to 1.25V/1.30V, we can switch
	 * CPU clock to 1GHz/1.2GHz and DDR to 400MHz safely
	 */
	ret = mxc_set_clock(ref_clk, megaherts, MXC_ARM_CLK);
	if (ret)
		printf("CPU:   Switch CPU clock to %dMHz failed\n", megaherts);

	ret = mxc_set_clock(ref_clk, 400, MXC_PERIPH_CLK);
	ret |= mxc_set_clock(ref_clk, 400, MXC_DDR_CLK);
	if (ret)
		printf("CPU:   Switch DDR clock to 400MHz failed\n");
}

int board_early_init_f(void)
{
	setup_iomux_uart();
	setup_iomux_fec();
	setup_iomux_lcd();

	return 0;
}

/*
 * Do not overwrite the console
 * Use always serial for U-Boot console
 */
int overwrite_console(void)
{
	return 1;
}

int board_init(void)
{
	gd->bd->bi_boot_params = PHYS_SDRAM_1 + 0x100;

	mxc_set_sata_internal_clock();
	setup_i2c(0, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info0);
	setup_i2c(1, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info1);
	setup_i2c(2, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info2);

	return 0;
}

int power_init_board(void)
{
	/* power off LCDs */
	pca953x_set_dir(CONFIG_SYS_I2C_PCA953X_ADDR, 1 << IO_EXP_VID1_PWREN, PCA953X_DIR_OUT << IO_EXP_VID1_PWREN);
	pca953x_set_dir(CONFIG_SYS_I2C_PCA953X_ADDR, 1 << IO_EXP_VID1_RESET, PCA953X_DIR_OUT << IO_EXP_VID1_RESET);
	pca953x_set_dir(CONFIG_SYS_I2C_PCA953X_ADDR, 1 << IO_EXP_VID2_PWREN, PCA953X_DIR_OUT << IO_EXP_VID2_PWREN);
	pca953x_set_dir(CONFIG_SYS_I2C_PCA953X_ADDR, 1 << IO_EXP_VID2_RESET, PCA953X_DIR_OUT << IO_EXP_VID2_RESET);
	pca953x_set_val(CONFIG_SYS_I2C_PCA953X_ADDR, 1 << IO_EXP_VID1_PWREN, PCA953X_OUT_LOW << IO_EXP_VID1_PWREN);
	pca953x_set_val(CONFIG_SYS_I2C_PCA953X_ADDR, 1 << IO_EXP_VID1_RESET, PCA953X_OUT_LOW << IO_EXP_VID1_RESET);
	pca953x_set_val(CONFIG_SYS_I2C_PCA953X_ADDR, 1 << IO_EXP_VID2_PWREN, PCA953X_OUT_LOW << IO_EXP_VID2_PWREN);
	pca953x_set_val(CONFIG_SYS_I2C_PCA953X_ADDR, 1 << IO_EXP_VID2_RESET, PCA953X_OUT_LOW << IO_EXP_VID2_RESET);

	if (!power_init())
		clock_1GHz();
	print_cpuinfo();
#ifdef CONFIG_UMOBO_BASEBOARD
	baseboard_init(I2C_PMIC);
#endif
	return 0;
}

int checkboard(void)
{
	puts("Board: BlueLightning\n");

	return 0;
}
