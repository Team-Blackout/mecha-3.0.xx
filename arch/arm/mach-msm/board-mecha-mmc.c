/* linux/arch/arm/mach-msm/board-mecha-mmc.c
 *
 * Copyright (C) 2008 HTC Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/mmc/host.h>
#include <linux/mmc/sdio_ids.h>
#include <linux/err.h>
#include <linux/debugfs.h>
#include <linux/gpio.h>
#include <linux/mfd/pmic8058.h>
#include <mach/htc_fast_clk.h>
#include <linux/gpio.h>
#include <linux/io.h>

#include <mach/vreg.h>
#include <mach/htc_pwrsink.h>

#include <asm/mach/mmc.h>

#include "devices.h"
#include "board-mecha.h"
#include "proc_comm.h"

/*#define MECHA_SDMC_CD_N_TO_SYS PM8058_GPIO_PM_TO_SYS(MECHA_GPIO_SDMC_CD_N)*/

#if 0
/* ---- SDCARD ---- */

static uint32_t sdcard_on_gpio_table[] = {
	PCOM_GPIO_CFG(58, 1, GPIO_OUTPUT, GPIO_PULL_DOWN, GPIO_16MA), /* CLK */
	PCOM_GPIO_CFG(59, 1, GPIO_INPUT, GPIO_NO_PULL, GPIO_10MA), /* CMD */
	PCOM_GPIO_CFG(60, 1, GPIO_INPUT, GPIO_NO_PULL, GPIO_10MA), /* DAT3 */
	PCOM_GPIO_CFG(61, 1, GPIO_INPUT, GPIO_NO_PULL, GPIO_10MA), /* DAT2 */
	PCOM_GPIO_CFG(62, 1, GPIO_INPUT, GPIO_NO_PULL, GPIO_10MA), /* DAT1 */
	PCOM_GPIO_CFG(63, 1, GPIO_INPUT, GPIO_NO_PULL, GPIO_10MA), /* DAT0 */
};

static uint32_t sdcard_off_gpio_table[] = {
	PCOM_GPIO_CFG(58, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_16MA), /* CLK */
	PCOM_GPIO_CFG(59, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_10MA), /* CMD */
	PCOM_GPIO_CFG(60, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_10MA), /* DAT3 */
	PCOM_GPIO_CFG(61, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_10MA), /* DAT2 */
	PCOM_GPIO_CFG(62, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_10MA), /* DAT1 */
	PCOM_GPIO_CFG(63, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_10MA), /* DAT0 */
};

static uint opt_disable_sdcard;
#endif

static uint32_t movinand_on_gpio_table[] = {
	PCOM_GPIO_CFG(38, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_16MA), /* CLK */
	PCOM_GPIO_CFG(39, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_10MA), /* CMD */
	PCOM_GPIO_CFG(40, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_10MA), /* DAT3 */
	PCOM_GPIO_CFG(41, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_10MA), /* DAT2 */
	PCOM_GPIO_CFG(42, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_10MA), /* DAT1 */
	PCOM_GPIO_CFG(43, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_10MA), /* DAT0 */
};

static void config_gpio_table(uint32_t *table, int len)
{
	int n, rc;
	for (n = 0; n < len; n++) {
		rc = gpio_tlmm_config(table[n], GPIO_CFG_ENABLE);
		if (rc) {
			pr_err("%s: gpio_tlmm_config(%#x)=%d\n", __func__, table[n], rc);
			break;
		}
	}
}

#if 0
static int __init mecha_disablesdcard_setup(char *str)
{
	/* int cal = simple_strtol(str, NULL, 0); */

	opt_disable_sdcard = cal;
	return 1;
}

static int __init mecha_disablesdcard_setup(char *str)
{
	unsigned long cal;
	if (strict_strtoul(str, 0, &cal) < 0)
		return -EINVAL;

	opt_disable_sdcard = cal;
	return 1;
}

__setup("board_mecha.disable_sdcard=", mecha_disablesdcard_setup);

static struct vreg *vreg_sdslot;	/* SD slot power */

struct mmc_vdd_xlat {
	int mask;
	int level;
};

static struct mmc_vdd_xlat mmc_vdd_table[] = {
	{ MMC_VDD_28_29,	2850 },
	{ MMC_VDD_29_30,	2900 },
};

static unsigned int sdslot_vdd = 0xffffffff;
static unsigned int sdslot_vreg_enabled;

static uint32_t mecha_sdslot_switchvdd(struct device *dev, unsigned int vdd)
{
	int i;

	BUG_ON(!vreg_sdslot);

	if (vdd == sdslot_vdd)
		return 0;

	sdslot_vdd = vdd;

	if (vdd == 0) {
		printk(KERN_INFO "%s: Disabling SD slot power\n", __func__);
		config_gpio_table(sdcard_off_gpio_table,
				  ARRAY_SIZE(sdcard_off_gpio_table));
		vreg_disable(vreg_sdslot);
		sdslot_vreg_enabled = 0;
		return 0;
	}

	if (!sdslot_vreg_enabled) {
		mdelay(5);
		vreg_enable(vreg_sdslot);
		udelay(500);
		config_gpio_table(sdcard_on_gpio_table,
				  ARRAY_SIZE(sdcard_on_gpio_table));
		sdslot_vreg_enabled = 1;
	}

	for (i = 0; i < ARRAY_SIZE(mmc_vdd_table); i++) {
		if (mmc_vdd_table[i].mask == (1 << vdd)) {
			printk(KERN_INFO "%s: Setting level to %u\n",
				__func__, mmc_vdd_table[i].level);
			vreg_set_level(vreg_sdslot, mmc_vdd_table[i].level);
			return 0;
		}
	}

	printk(KERN_ERR "%s: Invalid VDD %d specified\n", __func__, vdd);
	return 0;
}

static unsigned int mecha_sdslot_status(struct device *dev)
{
	unsigned int status;

	status = (unsigned int) gpio_get_value(MECHA_SDMC_CD_N_TO_SYS);

	return (!status);
}

#define MECHA_MMC_VDD		(MMC_VDD_28_29 | MMC_VDD_29_30)

static unsigned int mecha_sdslot_type = MMC_TYPE_SD;

static struct mmc_platform_data mecha_sdslot_data = {
	.ocr_mask	= MECHA_MMC_VDD,
	.status_irq	= MSM_GPIO_TO_INT(MECHA_SDMC_CD_N_TO_SYS),
	.status		= mecha_sdslot_status,
	.translate_vdd	= mecha_sdslot_switchvdd,
	.slot_type	= &mecha_sdslot_type,
	.dat0_gpio	= 63,
};

static unsigned int mecha_emmcslot_type = MMC_TYPE_MMC;
static struct mmc_platform_data mecha_movinand_data = {
	.ocr_mask	=  MECHA_MMC_VDD,
	.slot_type	= &mecha_emmcslot_type,
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
};
#endif

/* ---- WIFI ---- */

static uint32_t wifi_on_gpio_table[] = {
	PCOM_GPIO_CFG(116, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA), /* DAT3 */
	PCOM_GPIO_CFG(117, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA), /* DAT2 */
	PCOM_GPIO_CFG(118, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA), /* DAT1 */
	PCOM_GPIO_CFG(119, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_4MA), /* DAT0 */
	PCOM_GPIO_CFG(111, 1, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_8MA), /* CMD */
	PCOM_GPIO_CFG(110, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_8MA), /* CLK */
	PCOM_GPIO_CFG(180, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_4MA), /* WLAN IRQ */
};

static uint32_t wifi_off_gpio_table[] = {
	PCOM_GPIO_CFG(116, 0, GPIO_INPUT, GPIO_PULL_UP, GPIO_4MA), /* DAT3 */
	PCOM_GPIO_CFG(117, 0, GPIO_INPUT, GPIO_PULL_UP, GPIO_4MA), /* DAT2 */
	PCOM_GPIO_CFG(118, 0, GPIO_INPUT, GPIO_PULL_UP, GPIO_4MA), /* DAT1 */
	PCOM_GPIO_CFG(119, 0, GPIO_INPUT, GPIO_PULL_UP, GPIO_4MA), /* DAT0 */
	PCOM_GPIO_CFG(111, 0, GPIO_INPUT, GPIO_PULL_UP, GPIO_4MA), /* CMD */
	PCOM_GPIO_CFG(110, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_4MA), /* CLK */
	PCOM_GPIO_CFG(180, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_4MA), /* WLAN IRQ */
};

/* BCM4329 returns wrong sdio_vsn(1) when we read cccr,
 * we use predefined value (sdio_vsn=2) here to initial sdio driver well
 */
static struct embedded_sdio_data mecha_wifi_emb_data = {
	.cccr	= {
		.sdio_vsn	= 2,
		.multi_block	= 1,
		.low_speed	= 0,
		.wide_bus	= 0,
		.high_power	= 1,
		.high_speed	= 1,
	}
};

static void (*wifi_status_cb)(int card_present, void *dev_id);
static void *wifi_status_cb_devid;

static int
mecha_wifi_status_register(void (*callback)(int card_present, void *dev_id),
				void *dev_id)
{
	if (wifi_status_cb)
		return -EAGAIN;
	wifi_status_cb = callback;
	wifi_status_cb_devid = dev_id;
	return 0;
}

static int mecha_wifi_cd;	/* WiFi virtual 'card detect' status */

static unsigned int mecha_wifi_status(struct device *dev)
{
	return mecha_wifi_cd;
}

static unsigned int mecha_wifislot_type = MMC_TYPE_SDIO_WIFI;
static struct mmc_platform_data mecha_wifi_data = {
		.ocr_mask				= MMC_VDD_28_29,
		.status					= mecha_wifi_status,
		.register_status_notify	= mecha_wifi_status_register,
		.embedded_sdio			= &mecha_wifi_emb_data,
		.slot_type				= &mecha_wifislot_type,
		.mmc_bus_width			= MMC_CAP_4_BIT_DATA,
		.msmsdcc_fmin  			= 144000,
		.msmsdcc_fmid   		= 24576000,
//		.msmsdcc_fmax			= 49152000,
        .msmsdcc_fmax			= 50000000,
//		.msmsdcc_fmax   		= 49152000,
		.nonremovable   		= 0,
		/* HTC_WIFI_MOD, temp remove dummy52
		.dummy52_required 		= 1, */
};

int mecha_wifi_set_carddetect(int val)
{
	printk(KERN_INFO "%s: %d\n", __func__, val);
	mecha_wifi_cd = val;
	if (wifi_status_cb)
		wifi_status_cb(val, wifi_status_cb_devid);
	else
		printk(KERN_WARNING "%s: Nobody to notify\n", __func__);
	return 0;
}
EXPORT_SYMBOL(mecha_wifi_set_carddetect);

int mecha_wifi_power(int on)
{
	printk(KERN_INFO "%s: %d\n", __func__, on);

	if (on) {
	   config_gpio_table(wifi_on_gpio_table,
				ARRAY_SIZE(wifi_on_gpio_table));
	} else {
      config_gpio_table(wifi_off_gpio_table,
				ARRAY_SIZE(wifi_off_gpio_table));
	}

	/* spade_wifi_bt_sleep_clk_ctl(on, ID_WIFI); */
	gpio_set_value(MECHA_GPIO_WIFI_SHUTDOWN_N, on); /* WIFI_SHUTDOWN */
	mdelay(120);
	return 0;
}
EXPORT_SYMBOL(mecha_wifi_power);

int mecha_wifi_reset(int on)
{
	printk(KERN_INFO "%s: do nothing\n", __func__);
	return 0;
}

int __init mecha_init_mmc(unsigned int sys_rev)
{
	uint32_t id;
	wifi_status_cb = NULL;
	/*sdslot_vreg_enabled = 0;*/

	printk(KERN_INFO "mecha: %s\n", __func__);
	/* SDC2: MoviNAND */
	/*
	register_msm_irq_mask(INT_SDC2_0);
	register_msm_irq_mask(INT_SDC2_1);
	*/
	config_gpio_table(movinand_on_gpio_table,
			  ARRAY_SIZE(movinand_on_gpio_table));
#if 0
	msm_add_sdcc(1, &mecha_movinand_data);
#endif

	/* initial WIFI_SHUTDOWN# */
	id = PCOM_GPIO_CFG(MECHA_GPIO_WIFI_SHUTDOWN_N, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),
	msm_proc_comm(PCOM_RPC_GPIO_TLMM_CONFIG_EX, &id, 0);
	gpio_set_value(MECHA_GPIO_WIFI_SHUTDOWN_N, 0);

	msm_add_sdcc(3, &mecha_wifi_data);

#if 0
	register_msm_irq_mask(INT_SDC4_0);
	register_msm_irq_mask(INT_SDC4_1);
#endif

#if 0
	if (opt_disable_sdcard) {
		printk(KERN_INFO "mecha: SD-Card interface disabled\n");
		goto done;
	}

	vreg_sdslot = vreg_get(0, "gp10");
	if (IS_ERR(vreg_sdslot))
		return PTR_ERR(vreg_sdslot);

	set_irq_wake(MSM_GPIO_TO_INT(MECHA_SDMC_CD_N_TO_SYS), 1);

	msm_add_sdcc(4, &mecha_sdslot_data,
			MSM_GPIO_TO_INT(MECHA_SDMC_CD_N_TO_SYS),
			IORESOURCE_IRQ_LOWEDGE | IORESOURCE_IRQ_HIGHEDGE);
done:
#endif

	/* reset eMMC for write protection test */
	gpio_set_value(MECHA_GPIO_EMMC_RST, 0);	/* this should not work!!! */
	udelay(100);
	gpio_set_value(MECHA_GPIO_EMMC_RST, 1);

	return 0;
}
