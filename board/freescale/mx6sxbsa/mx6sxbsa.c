/*
 * Copyright (C) 2016 NXP Semiconductor
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <asm/arch/clock.h>
#include <asm/arch/iomux.h>
#include <asm/arch/imx-regs.h>
#include <asm/arch/mx6-pins.h>
#include <asm/arch/sys_proto.h>
#include <asm/gpio.h>
#include <asm/imx-common/iomux-v3.h>
#include <asm/imx-common/boot_mode.h>
#include <asm/io.h>
#include <linux/sizes.h>
#include <common.h>
#include <fsl_esdhc.h>
#include <mmc.h>
#include <miiphy.h>
#include <netdev.h>
#include <asm/arch/crm_regs.h>
#ifdef CONFIG_SYS_I2C_MXC
#include <i2c.h>
#include <asm/imx-common/mxc_i2c.h>
#endif
#include <power/pmic.h>
#include <power/pfuze300_pmic.h>
#include "../common/pfuze.h"
#include <usb.h>
#include <usb/ehci-fsl.h>

#ifdef CONFIG_MXC_RDC
#include <asm/imx-common/rdc-sema.h>
#include <asm/arch/imx-rdc.h>
#endif

#ifdef CONFIG_VIDEO_MXS
#include <linux/fb.h>
#include <mxsfb.h>
#endif

#ifdef CONFIG_MAX7310_IOEXP
#include <gpio_exp.h>
#endif

#ifdef CONFIG_FSL_FASTBOOT
#include <fsl_fastboot.h>
#ifdef CONFIG_ANDROID_RECOVERY
#include <recovery.h>
#endif
#endif /*CONFIG_FSL_FASTBOOT*/

DECLARE_GLOBAL_DATA_PTR;

#define UART_PAD_CTRL  (PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED |		\
	PAD_CTL_DSE_40ohm   | PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define USDHC_PAD_CTRL (PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_22K_UP  | PAD_CTL_SPEED_LOW |		\
	PAD_CTL_DSE_80ohm   | PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define ENET_PAD_CTRL  (PAD_CTL_PUS_100K_UP | PAD_CTL_PUE |     \
	PAD_CTL_SPEED_HIGH   |                                   \
	PAD_CTL_DSE_48ohm   | PAD_CTL_SRE_FAST)

#define ENET_CLK_PAD_CTRL  (PAD_CTL_SPEED_MED | \
	PAD_CTL_DSE_120ohm   | PAD_CTL_SRE_FAST)

#define ENET_RX_PAD_CTRL  (PAD_CTL_PKE | PAD_CTL_PUE |          \
	PAD_CTL_SPEED_HIGH   | PAD_CTL_SRE_FAST)

#define I2C_PAD_CTRL    (PAD_CTL_PKE | PAD_CTL_PUE |            \
	PAD_CTL_PUS_100K_UP | PAD_CTL_SPEED_MED |               \
	PAD_CTL_DSE_40ohm | PAD_CTL_HYS |			\
	PAD_CTL_ODE | PAD_CTL_SRE_FAST)

#define LCD_PAD_CTRL    (PAD_CTL_HYS | PAD_CTL_PUS_100K_UP | PAD_CTL_PUE | \
	PAD_CTL_PKE | PAD_CTL_SPEED_MED | PAD_CTL_DSE_40ohm)

#define BUTTON_PAD_CTRL    (PAD_CTL_PKE | PAD_CTL_PUE | \
	PAD_CTL_PUS_22K_UP | PAD_CTL_DSE_40ohm)

#define GPMI_PAD_CTRL0 (PAD_CTL_PKE | PAD_CTL_PUE | PAD_CTL_PUS_100K_UP)
#define GPMI_PAD_CTRL1 (PAD_CTL_DSE_40ohm | PAD_CTL_SPEED_MED | \
			PAD_CTL_SRE_FAST)
#define GPMI_PAD_CTRL2 (GPMI_PAD_CTRL0 | GPMI_PAD_CTRL1)

#define OTG_ID_PAD_CTRL (PAD_CTL_PKE | PAD_CTL_PUE |		\
	PAD_CTL_PUS_47K_UP  | PAD_CTL_SPEED_LOW |		\
	PAD_CTL_DSE_80ohm   | PAD_CTL_SRE_FAST  | PAD_CTL_HYS)

#define I2C_PMIC 0

#ifdef CONFIG_SYS_I2C_MXC
#define PC MUX_PAD_CTRL(I2C_PAD_CTRL)
/* I2C2 for PMIC */
static struct i2c_pads_info i2c_pad_info1 = {
	.scl = {
		.i2c_mode = MX6_PAD_GPIO1_IO00__I2C1_SCL | PC,
		.gpio_mode = MX6_PAD_GPIO1_IO00__GPIO1_IO_0 | PC,
		.gp = IMX_GPIO_NR(1, 0),
	},
	.sda = {
		.i2c_mode = MX6_PAD_GPIO1_IO01__I2C1_SDA | PC,
		.gpio_mode = MX6_PAD_GPIO1_IO01__GPIO1_IO_1 | PC,
		.gp = IMX_GPIO_NR(1, 1),
	},
};


static struct pmic *pfuze;
int power_init_board(void)
{
	unsigned int reg, rev_id;
	int ret;

	ret = power_pfuze300_init(I2C_PMIC);
	if (ret)
		return ret;

	pfuze = pmic_get("PFUZE300");
	ret = pmic_probe(pfuze);
	if (ret)
		return ret;

	pmic_reg_read(pfuze, PFUZE300_DEVICEID, &reg);
	pmic_reg_read(pfuze, PFUZE300_REVID, &rev_id);
	printf("PMIC: PFUZE300 DEV_ID=0x%x REV_ID=0x%x\n", reg, rev_id);

	/* set SW1A standby voltage 0.975V */
	pmic_reg_read(pfuze, PFUZE300_SW1ASTBY, &reg);
	reg &= ~0x1f;
	reg |= PFUZE300_SW1AB_SETP(9750);
	pmic_reg_write(pfuze, PFUZE300_SW1ASTBY, reg);

	/* set SW1A step ramp up time from 2us to 4us/25mV */
	pmic_reg_read(pfuze, PFUZE300_SW1ACONF, &reg);
	reg |= 0x40;
	pmic_reg_write(pfuze, PFUZE300_SW1ACONF, reg);

	/* set SW1B standby voltage 1.10V */
	pmic_reg_read(pfuze, PFUZE300_SW1BSTBY, &reg);
	reg &= ~0x1f;
	reg |= PFUZE300_SW1AB_SETP(11000);
	pmic_reg_write(pfuze, PFUZE300_SW1BSTBY, reg);

	/* set SW1C/VDDSOC step ramp up time to from 16us to 4us/25mV */
	pmic_reg_read(pfuze, PFUZE300_SW1BCONF, &reg);
	reg &= ~0xc0;
	reg |= 0x40;
	pmic_reg_write(pfuze, PFUZE300_SW1BCONF, reg);

	return 0;
}

#ifdef CONFIG_LDO_BYPASS_CHECK
void ldo_mode_set(int ldo_bypass)
{
	unsigned int value;

	struct pmic *p = pfuze;

	if (!p) {
		printf("No PMIC found!\n");
		return;
	}

	/* switch to ldo_bypass mode */
	if (ldo_bypass) {
		/* decrease VDDARM to 1.275V */
		pmic_reg_read(p, PFUZE300_SW1AVOLT, &value);
		value &= ~0x1f;
		value |= PFUZE300_SW1AB_SETP(12750);
		pmic_reg_write(p, PFUZE300_SW1AVOLT, value);

		/* decrease VDDSOC to 1.15V */
		pmic_reg_read(p, PFUZE300_SW1BVOLT, &value);
		value &= ~0x1f;
		value |= PFUZE300_SW1AB_SETP(13000);
		pmic_reg_write(p, PFUZE300_SW1BVOLT, value);

		set_anatop_bypass(1);
		printf("switch to ldo_bypass mode!\n");
	}

}
#endif
#endif

int dram_init(void)
{
	gd->ram_size = PHYS_SDRAM_SIZE;

	return 0;
}

static iomux_v3_cfg_t const uart1_pads[] = {
	MX6_PAD_GPIO1_IO04__UART1_TX | MUX_PAD_CTRL(UART_PAD_CTRL),
	MX6_PAD_GPIO1_IO05__UART1_RX | MUX_PAD_CTRL(UART_PAD_CTRL),
};

static iomux_v3_cfg_t const usdhc2_pads[] = {
	MX6_PAD_SD2_CLK__USDHC2_CLK | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_CMD__USDHC2_CMD | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DATA0__USDHC2_DATA0 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DATA1__USDHC2_DATA1 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DATA2__USDHC2_DATA2 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
	MX6_PAD_SD2_DATA3__USDHC2_DATA3 | MUX_PAD_CTRL(USDHC_PAD_CTRL),
};


#ifdef CONFIG_FEC_MXC

static iomux_v3_cfg_t const fec2_pads[] = {
	MX6_PAD_GPIO1_IO06__ENET2_MDC | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_GPIO1_IO07__ENET2_MDIO | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII2_RX_CTL__ENET2_RX_EN | MUX_PAD_CTRL(ENET_RX_PAD_CTRL),
	MX6_PAD_RGMII2_RXC__ENET2_RX_ER | MUX_PAD_CTRL(ENET_RX_PAD_CTRL),
	MX6_PAD_RGMII2_RD0__ENET2_RX_DATA_0 | MUX_PAD_CTRL(ENET_RX_PAD_CTRL),
	MX6_PAD_RGMII2_RD1__ENET2_RX_DATA_1 | MUX_PAD_CTRL(ENET_RX_PAD_CTRL),
	MX6_PAD_RGMII2_RD2__ENET2_RX_DATA_2 | MUX_PAD_CTRL(ENET_RX_PAD_CTRL),
	MX6_PAD_RGMII2_RD3__ENET2_RX_DATA_3 | MUX_PAD_CTRL(ENET_RX_PAD_CTRL),
	MX6_PAD_ENET2_RX_CLK__ENET2_RX_CLK | MUX_PAD_CTRL(ENET_RX_PAD_CTRL),
	MX6_PAD_RGMII2_TX_CTL__ENET2_TX_EN | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII2_TXC__ENET2_TX_ER | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII2_TD0__ENET2_TX_DATA_0 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII2_TD1__ENET2_TX_DATA_1 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII2_TD2__ENET2_TX_DATA_2 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_RGMII2_TD3__ENET2_TX_DATA_3 | MUX_PAD_CTRL(ENET_PAD_CTRL),
	MX6_PAD_ENET2_TX_CLK__ENET2_TX_CLK | MUX_PAD_CTRL(ENET_PAD_CTRL),
};

static iomux_v3_cfg_t const phy_control_pads[] = {
	/* TJA1100 PHY Enable */
	MX6_PAD_GPIO1_IO03__GPIO1_IO_3 | MUX_PAD_CTRL(NO_PAD_CTRL),

	/* TJA1100 PHY Reset */
	MX6_PAD_ENET2_CRS__GPIO2_IO_7 | MUX_PAD_CTRL(NO_PAD_CTRL),
};

#endif

static void setup_iomux_uart(void)
{
	imx_iomux_v3_setup_multiple_pads(uart1_pads, ARRAY_SIZE(uart1_pads));
}

#ifdef CONFIG_FSL_QSPI

#define QSPI_PAD_CTRL1	\
		(PAD_CTL_SRE_FAST | PAD_CTL_SPEED_MED | \
		 PAD_CTL_PKE | PAD_CTL_PUE | PAD_CTL_PUS_47K_UP | PAD_CTL_DSE_60ohm)

static iomux_v3_cfg_t const quadspi_pads[] = {
	MX6_PAD_QSPI1A_SS0_B__QSPI1_A_SS0_B   | MUX_PAD_CTRL(QSPI_PAD_CTRL1),
	MX6_PAD_QSPI1A_SCLK__QSPI1_A_SCLK     | MUX_PAD_CTRL(QSPI_PAD_CTRL1),
	MX6_PAD_QSPI1A_DATA0__QSPI1_A_DATA_0  | MUX_PAD_CTRL(QSPI_PAD_CTRL1),
	MX6_PAD_QSPI1A_DATA1__QSPI1_A_DATA_1	| MUX_PAD_CTRL(QSPI_PAD_CTRL1),
	MX6_PAD_QSPI1A_DATA2__QSPI1_A_DATA_2  | MUX_PAD_CTRL(QSPI_PAD_CTRL1),
	MX6_PAD_QSPI1A_DATA3__QSPI1_A_DATA_3  | MUX_PAD_CTRL(QSPI_PAD_CTRL1),
	MX6_PAD_QSPI1B_SS0_B__QSPI1_B_SS0_B   | MUX_PAD_CTRL(QSPI_PAD_CTRL1),
	MX6_PAD_QSPI1B_SCLK__QSPI1_B_SCLK     | MUX_PAD_CTRL(QSPI_PAD_CTRL1),
	MX6_PAD_QSPI1B_DATA0__QSPI1_B_DATA_0  | MUX_PAD_CTRL(QSPI_PAD_CTRL1),
	MX6_PAD_QSPI1B_DATA1__QSPI1_B_DATA_1  | MUX_PAD_CTRL(QSPI_PAD_CTRL1),
	MX6_PAD_QSPI1B_DATA2__QSPI1_B_DATA_2  | MUX_PAD_CTRL(QSPI_PAD_CTRL1),
	MX6_PAD_QSPI1B_DATA3__QSPI1_B_DATA_3  | MUX_PAD_CTRL(QSPI_PAD_CTRL1),
};

int board_qspi_init(void)
{
	/* Set the iomux */
	imx_iomux_v3_setup_multiple_pads(quadspi_pads, ARRAY_SIZE(quadspi_pads));

	/* Set the clock */
	enable_qspi_clk(0);

	return 0;
}
#endif

#ifdef CONFIG_FSL_ESDHC
static struct fsl_esdhc_cfg usdhc_cfg[1] = {
	{USDHC2_BASE_ADDR},
};

int mmc_get_env_devno(void)
{
	u32 soc_sbmr = readl(SRC_BASE_ADDR + 0x4);
	int dev_no;
	u32 bootsel;

	bootsel = (soc_sbmr & 0x000000FF) >> 6 ;

	/* If not boot from sd/mmc, use default value */
	if (bootsel != 1)
		return CONFIG_SYS_MMC_ENV_DEV;

	/* BOOT_CFG2[3] and BOOT_CFG2[4] */
	dev_no = (soc_sbmr & 0x00001800) >> 11;

	/* need ubstract 1 to map to the mmc device id
	 * see the comments in board_mmc_init function
	 */

	dev_no -= 1;

	return dev_no;
}

int mmc_map_to_kernel_blk(int dev_no)
{
	return dev_no + 1;
}

int board_mmc_getcd(struct mmc *mmc)
{
	/* SD2 I/F has no card detect */
	return 1;
}

int board_mmc_init(bd_t *bis)
{
	imx_iomux_v3_setup_multiple_pads(
			usdhc2_pads, ARRAY_SIZE(usdhc2_pads));
	usdhc_cfg[0].sdhc_clk = mxc_get_clock(MXC_ESDHC2_CLK);

	if (fsl_esdhc_initialize(bis, &usdhc_cfg[0]))
		printf("Warning: failed to initialize mmc dev %d\n", 0);

	return 0;
}

int check_mmc_autodetect(void)
{
	char *autodetect_str = getenv("mmcautodetect");

	if ((autodetect_str != NULL) &&
		(strcmp(autodetect_str, "yes") == 0)) {
		return 1;
	}

	return 0;
}

void board_late_mmc_init(void)
{
	char cmd[32];
	char mmcblk[32];
	u32 dev_no = mmc_get_env_devno();

	if (!check_mmc_autodetect())
		return;

	setenv_ulong("mmcdev", dev_no);

	/* Set mmcblk env */
	sprintf(mmcblk, "/dev/mmcblk%dp2 rootwait rw",
		mmc_map_to_kernel_blk(dev_no));
	setenv("mmcroot", mmcblk);

	sprintf(cmd, "mmc dev %d", dev_no);
	run_command(cmd, 0);
}
#endif

#ifdef CONFIG_FEC_MXC

static int setup_fec(void)
{
	struct iomuxc_gpr_base_regs *const iomuxc_gpr_regs
		= (struct iomuxc_gpr_base_regs *) IOMUXC_GPR_BASE_ADDR;

	/* Use 125M anatop REF_CLK1 for ENET2, clear gpr1[14], gpr1[18]*/
	clrsetbits_le32(&iomuxc_gpr_regs->gpr[1], IOMUX_GPR1_FEC2_MASK, 0);

	imx_iomux_v3_setup_multiple_pads(phy_control_pads,
					 ARRAY_SIZE(phy_control_pads));

	/* Reset TJA1100 PHY */
	gpio_direction_output(IMX_GPIO_NR(2, 7) , 0);
	udelay(500);
	gpio_set_value(IMX_GPIO_NR(2, 7), 1);

	/* Enable TJA1100 PHY */
	gpio_direction_output(IMX_GPIO_NR(1, 3) , 1);

	return enable_fec_anatop_clock(1, ENET_125MHZ);
}

int board_eth_init(bd_t *bis)
{
	int ret;

	imx_iomux_v3_setup_multiple_pads(fec2_pads, ARRAY_SIZE(fec2_pads));

	setup_fec();

	ret = fecmxc_initialize_multi(bis, CONFIG_FEC_ENET_DEV,
		CONFIG_FEC_MXC_PHYADDR, IMX_FEC_BASE);
	if (ret)
		printf("FEC%d MXC: %s:failed\n", CONFIG_FEC_ENET_DEV, __func__);

	return 0;
}

int board_phy_config(struct phy_device *phydev)
{
	/* Enable 1.8V(SEL_1P5_1P8_POS_REG) on Phy control debug reg 0 */
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1d, 0x1f);
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1e, 0x8);

	/* rgmii tx clock delay enable */
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1d, 0x05);
	phy_write(phydev, MDIO_DEVAD_NONE, 0x1e, 0x100);

	if (phydev->drv->config)
		phydev->drv->config(phydev);

	return 0;
}
#endif

#ifdef CONFIG_MXC_RDC
static rdc_peri_cfg_t const shared_resources[] = {
	(RDC_PER_GPIO1 | RDC_DOMAIN(0) | RDC_DOMAIN(1)),
};
#endif

int board_early_init_f(void)
{
#ifdef CONFIG_MXC_RDC
	imx_rdc_setup_peripherals(shared_resources, ARRAY_SIZE(shared_resources));
#endif

#ifdef CONFIG_SYS_AUXCORE_FASTUP
	arch_auxiliary_core_up(0, CONFIG_SYS_AUXCORE_BOOTDATA);
#endif

	setup_iomux_uart();
	return 0;
}

#ifdef CONFIG_USB_EHCI_MX6
#define USB_OTHERREGS_OFFSET	0x800
#define UCTRL_PWR_POL		(1 << 9)

iomux_v3_cfg_t const usb_otg_pads[] = {
	/* OTG1 */
	MX6_PAD_GPIO1_IO09__USB_OTG1_PWR | MUX_PAD_CTRL(NO_PAD_CTRL),
	MX6_PAD_GPIO1_IO10__ANATOP_OTG1_ID | MUX_PAD_CTRL(OTG_ID_PAD_CTRL),
	/* OTG2 */
	MX6_PAD_GPIO1_IO12__USB_OTG2_PWR | MUX_PAD_CTRL(NO_PAD_CTRL),
};

static void setup_usb(void)
{
	imx_iomux_v3_setup_multiple_pads(usb_otg_pads,
					 ARRAY_SIZE(usb_otg_pads));
}

int board_usb_phy_mode(int port)
{
	if (port == 1)
		return USB_INIT_HOST;
	else
		return usb_phy_mode(port);
}

int board_ehci_hcd_init(int port)
{
	u32 *usbnc_usb_ctrl;

	if (port > 1)
		return -EINVAL;

	usbnc_usb_ctrl = (u32 *)(USB_BASE_ADDR + USB_OTHERREGS_OFFSET +
				 port * 4);

	/* Set Power polarity */
	setbits_le32(usbnc_usb_ctrl, UCTRL_PWR_POL);

	return 0;
}
#endif

int board_init(void)
{
	/* Address of boot parameters */
	gd->bd->bi_boot_params = PHYS_SDRAM + 0x100;

#ifdef CONFIG_SYS_I2C_MXC
	setup_i2c(1, CONFIG_SYS_I2C_SPEED, 0x7f, &i2c_pad_info1);
#endif

#ifdef CONFIG_USB_EHCI_MX6
	setup_usb();
#endif

#ifdef CONFIG_FSL_QSPI
	board_qspi_init();
#endif

	return 0;
}

#ifdef CONFIG_CMD_BMODE
static const struct boot_mode board_boot_modes[] = {
	{"sd2", MAKE_CFGVAL(0x40, 0x28, 0x00, 0x00)},
	{NULL,	 0},
};
#endif

int board_late_init(void)
{
#ifdef CONFIG_CMD_BMODE
	add_board_boot_modes(board_boot_modes);
#endif

#ifdef CONFIG_ENV_IS_IN_MMC
	board_late_mmc_init();
#endif
	/* set WDOG_B to reset whole system */
	set_wdog_reset((struct wdog_regs *)WDOG1_BASE_ADDR);

	return 0;
}

u32 get_board_rev(void)
{
	return get_cpu_rev();
}

int checkboard(void)
{
	puts("Board: MX6SX BROADCAST SMART ANTENNA\n");

	return 0;
}

#ifdef CONFIG_FSL_FASTBOOT

void board_fastboot_setup(void)
{
	switch (get_boot_device()) {
#if defined(CONFIG_FASTBOOT_STORAGE_MMC)
	case SD3_BOOT:
	case MMC3_BOOT:
		if (!getenv("fastboot_dev"))
			setenv("fastboot_dev", "mmc0");
		if (!getenv("bootcmd"))
			setenv("bootcmd", "boota mmc0");
		break;
	case SD4_BOOT:
	case MMC4_BOOT:
		if (!getenv("fastboot_dev"))
			setenv("fastboot_dev", "mmc1");
		if (!getenv("bootcmd"))
			setenv("bootcmd", "boota mmc1");
		break;
#endif /*CONFIG_FASTBOOT_STORAGE_MMC*/
#if defined(CONFIG_FASTBOOT_STORAGE_NAND)
	case NAND_BOOT:
		if (!getenv("fastboot_dev"))
			setenv("fastboot_dev", "nand");
		if (!getenv("fbparts"))
			setenv("fbparts", ANDROID_FASTBOOT_NAND_PARTS);
		if (!getenv("bootcmd"))
			setenv("bootcmd",
				"nand read ${loadaddr} ${boot_nand_offset} "
				"${boot_nand_size};boota ${loadaddr}");
		break;
#endif /*CONFIG_FASTBOOT_STORAGE_NAND*/

	default:
		printf("unsupported boot devices\n");
		break;
	}
}

#ifdef CONFIG_ANDROID_RECOVERY
int check_recovery_cmd_file(void)
{
	int recovery_mode = 0;

	recovery_mode = recovery_check_and_clean_flag();

	return recovery_mode;
}

void board_recovery_setup(void)
{
	int bootdev = get_boot_device();

	switch (bootdev) {
#if defined(CONFIG_FASTBOOT_STORAGE_MMC)
	case SD3_BOOT:
	case MMC3_BOOT:
		if (!getenv("bootcmd_android_recovery"))
			setenv("bootcmd_android_recovery", "boota mmc0 recovery");
		break;
	case SD4_BOOT:
	case MMC4_BOOT:
		if (!getenv("bootcmd_android_recovery"))
			setenv("bootcmd_android_recovery", "boota mmc1 recovery");
		break;
#endif /*CONFIG_FASTBOOT_STORAGE_MMC*/
#if defined(CONFIG_FASTBOOT_STORAGE_NAND)
	case NAND_BOOT:
		if (!getenv("bootcmd_android_recovery"))
			setenv("bootcmd_android_recovery",
				"nand read ${loadaddr} ${recovery_nand_offset} "
				"${recovery_nand_size};boota ${loadaddr}");
		break;
#endif /*CONFIG_FASTBOOT_STORAGE_NAND*/
	default:
		printf("Unsupported bootup device for recovery: dev: %d\n",
			bootdev);
		return;
	}

	printf("setup env for recovery..\n");
	setenv("bootcmd", "run bootcmd_android_recovery");
}
#endif /*CONFIG_ANDROID_RECOVERY*/

#endif /*CONFIG_FSL_FASTBOOT*/
