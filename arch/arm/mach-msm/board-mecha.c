/* Copyright (c) 2009-2011, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/kernel.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/mfd/pmic8058.h>
#include <linux/delay.h>
#include <linux/bootmem.h>
#include <linux/io.h>
#ifdef CONFIG_SPI_QSD
#include <linux/spi/spi.h>
#endif
#include <linux/mfd/pmic8058.h>
#include <linux/leds.h>
#include <linux/mfd/marimba.h>
#include <linux/i2c.h>
#include <linux/akm8975.h>
#include <linux/bma150.h>
#include <linux/capella_cm3602.h>
#include <linux/cm3628.h>
#include <linux/lightsensor.h>
#include <linux/input.h>
#include <linux/atmel_qt602240.h>
#include <linux/smsc911x.h>
#include <linux/ofn_atlab.h>
#include <mach/atmega_microp.h>
#include <linux/power_supply.h>
#include <linux/i2c/isa1200.h>
#include <linux/input/kp_flip_switch.h>
#include <linux/leds-pm8058.h>
#include <linux/msm_adc.h>
#include <linux/dma-mapping.h>
#include <linux/proc_fs.h>

#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/setup.h>
#include <mach/panel_id.h>

#include <mach/system.h>
#include <mach/mpp.h>
#include <mach/board.h>
#include <mach/camera-7x30.h>
#include <mach/memory.h>
#include <mach/msm_iomap.h>
#ifdef CONFIG_USB_MSM_OTG_72K
#include <mach/msm_hsusb.h>
#else
#include <linux/usb/msm_hsusb.h>
#endif
#include <mach/msm_spi.h>
#include <mach/dma.h>
#include <linux/android_pmem.h>
#include <linux/input/msm_ts.h>
#include <mach/rpc_pmapp.h>
#include <mach/qdsp5v2_2x/msm_lpa.h>
#include <mach/qdsp5v2_2x/aux_pcm.h>
#include <mach/qdsp5v2_2x/mi2s.h>
#include <mach/qdsp5v2_2x/audio_dev_ctl.h>
#include <mach/htc_battery.h>
#include <linux/tps65200.h>
#include <mach/tpa2051d3.h>
#include <mach/rpc_server_handset.h>
#include <mach/msm_tsif.h>
#include <mach/socinfo.h>
#include <mach/msm_memtypes.h>
#include <asm/mach/mmc.h>
#include <asm/mach/flash.h>
#include <linux/htc_flashlight.h>
#include <mach/vreg.h>
#include <linux/platform_data/qcom_crypto_device.h>
#include <mach/htc_headset_mgr.h>
#include <mach/htc_headset_gpio.h>
#include <mach/htc_headset_pmic.h>
#include <mach/htc_headset_microp.h>

#include "pmic.h"
#include "proc_comm.h"
#include "devices.h"
#include "timer.h"
#ifdef CONFIG_USB_G_ANDROID
#include <mach/htc_usb.h>
#include <linux/usb/android_composite.h>
#include <linux/usb/android.h>
#include <mach/usbdiag.h>
#endif
#include "pm.h"
#include "pm-boot.h"
#include "spm.h"
#include "acpuclock.h"
#include <mach/dal_axi.h>
#include <mach/msm_serial_hs.h>
#include <mach/smsc251x.h>
#include <mach/qdsp5v2_2x/mi2s.h>
#include <mach/qdsp5v2_2x/audio_dev_ctl.h>
#include <mach/sdio_al.h>
#include "smd_private.h"
#include "board-mecha.h"
#include <mach/tpa2051d3.h>
#include "board-msm7x30-regulator.h"
#include <mach/board_htc.h>
#include <mach/cable_detect.h>
#ifdef CONFIG_PERFLOCK
#include <mach/perflock.h>
#endif
#ifdef CONFIG_BT
int htc_get_usb_accessory_adc_level(uint32_t *buffer);
#include <mach/htc_bdaddress.h>
#endif

#define GPIO_2MA	0
#define GPIO_4MA	1
#define GPIO_6MA	2
#define GPIO_8MA	3
#define GPIO_10MA	4
#define GPIO_12MA	5
#define GPIO_14MA	6
#define GPIO_16MA	7

#define PMIC_GPIO_INT		27

#define ADV7520_I2C_ADDR	0x39

#define FPGA_SDCC_STATUS       0x8E0001A8

#define FPGA_OPTNAV_GPIO_ADDR	0x8E000026
#define OPTNAV_I2C_SLAVE_ADDR	(0xB0 >> 1)
#define OPTNAV_IRQ		20
#define OPTNAV_CHIP_SELECT	19

/* Macros assume PMIC GPIOs start at 0 */
#define PM8058_GPIO_PM_TO_SYS(pm_gpio)     (pm_gpio + NR_GPIO_IRQS)
#define PM8058_GPIO_SYS_TO_PM(sys_gpio)    (sys_gpio - NR_GPIO_IRQS)
#define PM8058_MPP_BASE			   PM8058_GPIO_PM_TO_SYS(PM8058_GPIOS)
#define PM8058_MPP_PM_TO_SYS(pm_gpio)	   (pm_gpio + PM8058_MPP_BASE)
#define PM8058_IRQ_BASE				(NR_MSM_IRQS + NR_GPIO_IRQS)

#define MECHA_GPIO_PS_HOLD	(29) /*ACPU GPIO Number 29*/
#define PMIC_GPIO_HAP_ENABLE   16  /* PMIC GPIO Number 17 */
#define BMA150_GPIO_INT 1
#define HAP_LVL_SHFT_MSM_GPIO 24
#define PMIC_GPIO_QUICKVX_CLK 37 /* PMIC GPIO 38 */
#define	PM_FLIP_MPP 5 /* PMIC MPP 06 */
#define PMIC_GPIO_SD_DET 34

static void mecha_usb_hub_enable(bool enable);
static void mecha_usb_hub_gpio_config(bool);
static int (*mecha_smsc251x_switch_cb)(uint8_t);

struct pm8xxx_gpio_init_info {
	unsigned			gpio;
	struct pm_gpio			config;
};

int __init mecha_init_panel(void);

static unsigned int skuid;
unsigned int mecha_get_skuid(void)
{
	return skuid;
}

static unsigned int smi_sz;
unsigned int mecha_get_smi_sz(void)
{
	return smi_sz;
}

static unsigned int hwid;
unsigned int mecha_get_hwid(void)
{
	return hwid;
}

static unsigned int engineerid;
unsigned int mecha_get_engineerid(void)
{
	return engineerid;
}

#define GPIO_INPUT      0
#define GPIO_OUTPUT     1

#define GPIO_NO_PULL    0
#define GPIO_PULL_DOWN  1
#define GPIO_PULL_UP    3

#define PCOM_GPIO_CFG(gpio, func, dir, pull, drvstr) \
		((((gpio) & 0x3FF) << 4)        | \
		((func) & 0xf)                  | \
		(((dir) & 0x1) << 14)           | \
		(((pull) & 0x3) << 15)          | \
		(((drvstr) & 0xF) << 17))

static void config_gpio_table(uint32_t *table, int len)
{
	int n, rc;
	for (n = 0; n < len; n++) {
		rc = gpio_tlmm_config(table[n], GPIO_CFG_ENABLE);
		if (rc) {
			pr_err("[CAM] %s: gpio_tlmm_config(%#x)=%d\n",
				__func__, table[n], rc);
			break;
		}
	}
}

#if defined(CONFIG_MSM7KV2_AUDIO)
static struct resource msm_aictl_resources[] = {
	{
		.name = "aictl",
		.start = 0xa5000100,
		.end = 0xa5000100,
		.flags = IORESOURCE_MEM,
	}
};

static struct resource msm_mi2s_resources[] = {
	{
		.name = "hdmi",
		.start = 0xac900000,
		.end = 0xac900038,
		.flags = IORESOURCE_MEM,
	},
	{
		.name = "codec_rx",
		.start = 0xac940040,
		.end = 0xac940078,
		.flags = IORESOURCE_MEM,
	},
	{
		.name = "codec_tx",
		.start = 0xac980080,
		.end = 0xac9800B8,
		.flags = IORESOURCE_MEM,
	}

};

static struct msm_lpa_platform_data lpa_pdata = {
	.obuf_hlb_size = 0x2BFF8,
	.dsp_proc_id = 0,
	.app_proc_id = 2,
	.nosb_config = {
		.llb_min_addr = 0,
		.llb_max_addr = 0x3ff8,
		.sb_min_addr = 0,
		.sb_max_addr = 0,
	},
	.sb_config = {
		.llb_min_addr = 0,
		.llb_max_addr = 0x37f8,
		.sb_min_addr = 0x3800,
		.sb_max_addr = 0x3ff8,
	}
};

static struct resource msm_lpa_resources[] = {
	{
		.name = "lpa",
		.start = 0xa5000000,
		.end = 0xa50000a0,
		.flags = IORESOURCE_MEM,
	}
};

static struct resource msm_aux_pcm_resources[] = {

	{
		.name = "aux_codec_reg_addr",
		.start = 0xac9c00c0,
		.end = 0xac9c00c8,
		.flags = IORESOURCE_MEM,
	},
	{
		.name   = "aux_pcm_dout",
		.start  = 138,
		.end    = 138,
		.flags  = IORESOURCE_IO,
	},
	{
		.name   = "aux_pcm_din",
		.start  = 139,
		.end    = 139,
		.flags  = IORESOURCE_IO,
	},
	{
		.name   = "aux_pcm_syncout",
		.start  = 140,
		.end    = 140,
		.flags  = IORESOURCE_IO,
	},
	{
		.name   = "aux_pcm_clkin_a",
		.start  = 141,
		.end    = 141,
		.flags  = IORESOURCE_IO,
	},
};

static struct platform_device msm_aux_pcm_device = {
	.name   = "msm_aux_pcm",
	.id     = 0,
	.num_resources  = ARRAY_SIZE(msm_aux_pcm_resources),
	.resource       = msm_aux_pcm_resources,
};

static struct platform_device msm_aictl_device = {
	.name = "audio_interct",
	.id   = 0,
	.num_resources = ARRAY_SIZE(msm_aictl_resources),
	.resource = msm_aictl_resources,
};

static struct platform_device msm_mi2s_device = {
	.name = "mi2s",
	.id   = 0,
	.num_resources = ARRAY_SIZE(msm_mi2s_resources),
	.resource = msm_mi2s_resources,
};

static struct platform_device msm_lpa_device = {
	.name = "lpa",
	.id   = 0,
	.num_resources = ARRAY_SIZE(msm_lpa_resources),
	.resource = msm_lpa_resources,
	.dev		= {
		.platform_data = &lpa_pdata,
	},
};
#endif

static int mecha_ts_power(int on)
{
	pr_info("%s: power %d\n", __func__, on);

	if (on == 1) {
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(MECHA_TP_RSTz), 1);
	} else if (on == 2) {
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(MECHA_TP_RSTz), 0);
		msleep(5);
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(MECHA_TP_RSTz), 1);
		msleep(40);
	}

	return 0;
}

struct atmel_i2c_platform_data mecha_ts_atmel_data[] = {
	{
		.version = 0x0020,
		.abs_x_min = 0,
		.abs_x_max = 1023,
		.abs_y_min = 0,
		.abs_y_max = 959,
		.abs_pressure_min = 0,
		.abs_pressure_max = 255,
		.abs_width_min = 0,
		.abs_width_max = 20,
		.gpio_irq = PM8058_GPIO_PM_TO_SYS(MECHA_GPIO_TP_INT_N),
		.power = mecha_ts_power,
		.config_T6 = {0, 0, 0, 0, 0, 0},
		.config_T7 = {16, 15, 25},
		.config_T8 = {10, 0, 5, 2, 0, 0, 5, 25, 4, 170},
		.config_T9 = {139, 0, 0, 19, 11, 0, 16, 30, 3, 1, 0, 5, 2, 0, 4, 20, 10, 10, 0, 0, 0, 0, 250, 3, 42, 50, 143, 50, 135, 65, 40, 10},
		.config_T15 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T18 = {0, 0},
		.config_T19 = {3, 0, 0, 52, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T20 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T22 = {15, 0, 0, 0, 0, 0, 0, 0, 25, 0, 0, 0, 7, 18, 255, 255, 0},
		.config_T23 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T24 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T25 = {3, 0, 40, 35, 136, 19, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T28 = {0, 0, 3, 4, 8, 60},
		.object_crc = {0xEA, 0x0B, 0x22},
		.cable_config = {35, 25, 8, 16},
		.wlc_config = {20, 20, 50, 60, 50, 22, 22},
		.wlc_freq = {38, 48, 58, 255, 255},
		.GCAF_level = {20, 24, 28, 40, 63},
	},
	{
		.version = 0x0015,
		.abs_x_min = 0,
		.abs_x_max = 1023,
		.abs_y_min = 0,
		.abs_y_max = 959,
		.abs_pressure_min = 0,
		.abs_pressure_max = 255,
		.abs_width_min = 0,
		.abs_width_max = 20,
		.gpio_irq = PM8058_GPIO_PM_TO_SYS(MECHA_GPIO_TP_INT_N),
		.power = mecha_ts_power,
		.config_T6 = {0, 0, 0, 0, 0, 0},
		.config_T7 = {16, 15, 25},
		.config_T8 = {10, 0, 5, 2, 0, 0, 5, 25},
		.config_T9 = {139, 0, 0, 19, 11, 0, 16, 30, 3, 1, 0, 5, 2, 0, 4, 20, 10, 10, 0, 0, 0, 0, 250, 3, 42, 50, 143, 50, 135, 65, 40},
		.config_T15 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T18 = {0, 0},
		.config_T19 = {3, 0, 0, 52, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T20 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T22 = {15, 0, 0, 0, 0, 0, 0, 0, 25, 0, 0, 0, 7, 18, 255, 255, 0},
		.config_T23 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T24 = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T25 = {3, 0, 40, 35, 136, 19, 0, 0, 0, 0, 0, 0, 0, 0},
		.config_T27 = {0, 0, 0, 0, 0, 0, 0},
		.config_T28 = {0, 0, 3, 4, 8, 60},
		.object_crc = {0x11, 0x20, 0xDD},
		.cable_config = {35, 25, 8, 16},
		.wlc_config = {30, 30, 35, 45, 40, 8, 16},
		.GCAF_level = {20, 24, 28, 40, 63},
	},
};

static int mecha_smsc251x_switch_register(int (*callback)(uint8_t hub_enable))
{
	if (mecha_smsc251x_switch_cb)
		return -EAGAIN;

	mecha_smsc251x_switch_cb = callback;
	return 0;
}

static void mecha_usb_hub_enable(bool enable)
{
	mecha_smsc251x_switch_cb(enable);
}

struct smsc251x_platform_data mecha_smsc251x_data = {
	.usb_hub_gpio_config	= mecha_usb_hub_gpio_config,
	.register_switch_func	= mecha_smsc251x_switch_register,
};

static uint32_t proximity_on_gpio_table[] = {
	PCOM_GPIO_CFG(MECHA_GPIO_PS_INT_N,
		0, GPIO_INPUT, GPIO_NO_PULL, 0), /* PS_VOUT */
};

static uint32_t proximity_off_gpio_table[] = {
	PCOM_GPIO_CFG(MECHA_GPIO_PS_INT_N,
		0, GPIO_INPUT, GPIO_PULL_DOWN, 0) /* PS_VOUT */
};
void config_mecha_proximity_gpios(int on)
{
	if (on)
		config_gpio_table(proximity_on_gpio_table,
			ARRAY_SIZE(proximity_on_gpio_table));
	else
		config_gpio_table(proximity_off_gpio_table,
			ARRAY_SIZE(proximity_off_gpio_table));
}

static int capella_cm3602_power(int pwr_device, uint8_t enable);
static int capella_cm3628_power(int pwr_device, uint8_t enable);

static int __capella_cm3602_power(int on)
{
	int rc;
	struct vreg *vreg = vreg_get(0, "gp7");

	if (!vreg) {
		printk(KERN_ERR "%s: vreg error\n", __func__);
		return -EIO;
	}
	rc = vreg_set_level(vreg, 2850);

	printk(KERN_DEBUG "%s: Turn the capella_cm3602 power %s\n",
		__func__, (on) ? "on" : "off");

	if (on) {
		config_mecha_proximity_gpios(1);
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(MECHA_GPIO_PS_EN), 1);
		rc = vreg_enable(vreg);
		if (rc < 0)
			printk(KERN_ERR "%s: vreg enable failed\n", __func__);
	} else {
		rc = vreg_disable(vreg);
		if (rc < 0)
			printk(KERN_ERR "%s: vreg disable failed\n", __func__);
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(MECHA_GPIO_PS_EN), 0);
		config_mecha_proximity_gpios(0);
	}

	return rc;
}

static int __capella_cm3628_power(int on)
{
	int rc;
	struct vreg *vreg = vreg_get(0, "gp7");

	if (!vreg) {
		printk(KERN_ERR "%s: vreg error\n", __func__);
		return -EIO;
	}
	rc = vreg_set_level(vreg, 2850);

	printk(KERN_DEBUG "%s: Turn the capella_cm3628 power %s\n",
		__func__, (on) ? "on" : "off");

	if (on) {
		rc = vreg_enable(vreg);
		if (rc < 0)
			printk(KERN_ERR "%s: vreg enable failed\n", __func__);
		msleep(5);
		pm8xxx_gpio_cfg(MECHA_ALS_SHUTDOWN, PM_GPIO_DIR_IN, 0, 0, PM_GPIO_PULL_UP_31P5, PM8058_GPIO_VIN_L5, 0, PM_GPIO_FUNC_NORMAL, 0);
	} else {
		pm8xxx_gpio_cfg(MECHA_ALS_SHUTDOWN, PM_GPIO_DIR_IN, 0, 0, PM_GPIO_PULL_DN, PM8058_GPIO_VIN_L5, 0, PM_GPIO_FUNC_NORMAL, 0);
		rc = vreg_disable(vreg);
		if (rc < 0)
			printk(KERN_ERR "%s: vreg disable failed\n", __func__);
	}

	return rc;
}
static DEFINE_MUTEX(capella_cm3602_lock);
static int als_power_control;

static int capella_cm3602_power(int pwr_device, uint8_t enable)
{
	unsigned int old_status = 0;
	int ret = 0, on = 0;
	mutex_lock(&capella_cm3602_lock);

	old_status = als_power_control;
	if (enable)
		als_power_control |= pwr_device;
	else
		als_power_control &= ~pwr_device;

	on = als_power_control ? 1 : 0;
	if (old_status == 0 && on)
		ret = __capella_cm3602_power(1);
	else if (!on)
		ret = __capella_cm3602_power(0);

	mutex_unlock(&capella_cm3602_lock);
	return ret;
}

static int capella_cm3628_power(int pwr_device, uint8_t enable)
{
	unsigned int old_status = 0;
	int ret = 0, on = 0;
	mutex_lock(&capella_cm3602_lock);

	old_status = als_power_control;
	if (enable)
		als_power_control |= pwr_device;
	else
		als_power_control &= ~pwr_device;

	on = als_power_control ? 1 : 0;
	if (old_status == 0 && on)
		ret = __capella_cm3628_power(1);
	else if (!on)
		ret = __capella_cm3628_power(0);

	mutex_unlock(&capella_cm3602_lock);
	return ret;
}
#ifdef CONFIG_INPUT_CAPELLA_CM3602
static struct capella_cm3602_platform_data capella_cm3602_pdata = {
	.power = capella_cm3602_power,
	.p_en = PM8058_GPIO_PM_TO_SYS(MECHA_GPIO_PS_EN),
	.p_out = PM8058_GPIO_PM_TO_SYS(MECHA_GPIO_PS_INT_N),
	.irq = MSM_GPIO_TO_INT(PM8058_GPIO_PM_TO_SYS(MECHA_GPIO_PS_INT_N)),
};

static struct platform_device capella_cm3602 = {
	.name = CAPELLA_CM3602,
	.id = -1,
	.dev = {
		.platform_data = &capella_cm3602_pdata
	}
};
#endif

static uint8_t cm3628_mapping_table[] = {0x0, 0x3, 0x6, 0x9, 0xC, 0xF,
					0x12, 0x15, 0x18, 0x1B, 0x1E, 0x21,
					0x24, 0x27, 0x2A, 0x2D, 0x30, 0x33,
					0x36, 0x39, 0x3C, 0x3F, 0x42, 0x45,
					0x48, 0x4B, 0x4E, 0x51, 0x54, 0x57,
					0x5A, 0x5D, 0x60, 0x63, 0x66, 0x69,
					0x6C, 0x6F, 0x72, 0x75, 0x78, 0x7B,
					0x7E, 0x82, 0x86, 0x8A, 0x8E, 0x92,
					0x96, 0x9A, 0x9E, 0xA2, 0xA6, 0xAA,
					0xAE, 0xB2, 0xB6, 0xBA, 0xBE, 0xC3,
					0xC8, 0xCD, 0xD2, 0xD7, 0xDC, 0xE1,
					0xE6, 0xEB, 0xF0, 0xF5, 0xFA, 0xFF};

static struct cm3628_platform_data cm3628_pdata = {
	.intr = PM8058_GPIO_PM_TO_SYS(MECHA_ALS_SHUTDOWN),
	.levels = { 0x1, 0xb, 0x11, 0x57, 0x83, 0x406,
			0x6EF, 0x77B, 0x806, 0xFFFF},
	.golden_adc = 0x410,
	.power = capella_cm3628_power,
	.ALS_slave_address = 0x30>>1,
	.PS_slave_address = 0x32>>1,
	.check_interrupt_add = 0x18>>1	,
	.ps_thd_set = 0x06,
	.ps_conf2_val = CM3628_IT_4X_MP,
	.mapping_table = cm3628_mapping_table,
	.mapping_size = ARRAY_SIZE(cm3628_mapping_table),
};

static struct akm8975_platform_data compass_platform_data = {
	.layouts = MECHA_LAYOUTS,
};

static struct bma150_platform_data gsensor_platform_data = {
	.intr = MSM_GPIO_TO_INT(PM8058_GPIO_PM_TO_SYS(
			MECHA_GPIO_GSENSOR_INT_N)),
	.chip_layout = 1,
};

static struct i2c_board_info i2c_Sensors_devices[] = {
	{
		I2C_BOARD_INFO(AKM8975_I2C_NAME, 0x1A >> 1),
		.platform_data = &compass_platform_data,
		.irq = MSM_GPIO_TO_INT(PM8058_GPIO_PM_TO_SYS(MECHA_GPIO_COMPASS_INT_N)),
	},
	{
		I2C_BOARD_INFO(BMA150_I2C_NAME, 0x70 >> 1),
		.platform_data = &gsensor_platform_data,
		.irq = MSM_GPIO_TO_INT(PM8058_GPIO_PM_TO_SYS(MECHA_GPIO_GSENSOR_INT_N)),
	},
};

static struct i2c_board_info i2c_CM3628_devices[] = {
	{
		I2C_BOARD_INFO(CM3628_I2C_NAME, 0x30 >> 1),
		.platform_data = &cm3628_pdata,
		.irq = MSM_GPIO_TO_INT(PM8058_GPIO_PM_TO_SYS(MECHA_ALS_SHUTDOWN)),
	},
};


static int pm8058_gpios_init(void)
{
	int rc;

	/* touch panel */
	struct pm8xxx_gpio_init_info tp_rstz = {
		PM8058_GPIO_PM_TO_SYS(MECHA_TP_RSTz),
		{
			.direction      = PM_GPIO_DIR_OUT,
			.output_buffer  = PM_GPIO_OUT_BUF_CMOS,
			.output_value   = 1,
			.pull           = PM_GPIO_PULL_NO,
			.vin_sel        = PM8058_GPIO_VIN_L5,
			.out_strength   = PM_GPIO_STRENGTH_HIGH,
			.function       = PM_GPIO_FUNC_NORMAL,
			.inv_int_pol    = 0,
		},
	};

	/* led */
	struct pm8xxx_gpio_init_info green_led = {
		PM8058_GPIO_PM_TO_SYS(MECHA_GREEN_LED),
		{
			.direction      = PM_GPIO_DIR_OUT,
			.output_buffer  = PM_GPIO_OUT_BUF_CMOS,
			.output_value   = 1,
			.pull           = PM_GPIO_PULL_NO,
			.vin_sel        = PM8058_GPIO_VIN_L5,
			.out_strength   = PM_GPIO_STRENGTH_HIGH,
			.function       = PM_GPIO_FUNC_2,
			.inv_int_pol    = 0,
		},
	};

	struct pm8xxx_gpio_init_info amber_led = {
		PM8058_GPIO_PM_TO_SYS(MECHA_AMBER_LED),
		{
			.direction      = PM_GPIO_DIR_OUT,
			.output_buffer  = PM_GPIO_OUT_BUF_CMOS,
			.output_value   = 1,
			.pull           = PM_GPIO_PULL_NO,
			.vin_sel        = PM8058_GPIO_VIN_L5,
			.out_strength   = PM_GPIO_STRENGTH_HIGH,
			.function       = PM_GPIO_FUNC_2,
			.inv_int_pol    = 0,
		},
	};

	/* Headset */
	struct pm8xxx_gpio_init_info aud_hp_detz = {
		PM8058_GPIO_PM_TO_SYS(MECHA_AUD_HP_DETz),
		{
			.direction      = PM_GPIO_DIR_IN,
			.output_buffer  = 0,
			.output_value   = 0,
			.pull           = PM_GPIO_PULL_UP_31P5,
			.vin_sel        = PM_GPIO_VIN_S3,
			.out_strength   = 0,
			.function       = PM_GPIO_FUNC_NORMAL,
			.inv_int_pol    = 0,
		},
	};

	/* direct key */
	struct pm8xxx_gpio_init_info vol_up = {
		PM8058_GPIO_PM_TO_SYS(MECHA_VOL_UP),
		{
			.direction      = PM_GPIO_DIR_IN,
			.output_buffer  = PM_GPIO_OUT_BUF_CMOS,
			.output_value   = 0,
			.pull           = PM_GPIO_PULL_UP_31P5,
			.vin_sel        = PM8058_GPIO_VIN_S3,
			.out_strength   = PM_GPIO_STRENGTH_NO,
			.function       = PM_GPIO_FUNC_NORMAL,
		}
	};

	struct pm8xxx_gpio_init_info vol_dn = {
		PM8058_GPIO_PM_TO_SYS(MECHA_VOL_DN),
		{
			.direction      = PM_GPIO_DIR_IN,
			.output_buffer  = PM_GPIO_OUT_BUF_CMOS,
			.output_value   = 0,
			.pull           = PM_GPIO_PULL_UP_31P5,
			.vin_sel        = PM8058_GPIO_VIN_S3,
			.out_strength   = PM_GPIO_STRENGTH_NO,
			.function       = PM_GPIO_FUNC_NORMAL,
		}
	};

	/* sd detect */
	struct pm8xxx_gpio_init_info gpio_sdmc_cd_n = {
		PM8058_GPIO_PM_TO_SYS(MECHA_GPIO_SDMC_CD_N),
		{
			.direction      = PM_GPIO_DIR_IN,
			.output_buffer  = 0,
			.output_value   = 0,
			.pull           = PM_GPIO_PULL_UP_31P5,
			.vin_sel        = PM8058_GPIO_VIN_L5,
			.out_strength   = 0,
			.function       = PM_GPIO_FUNC_NORMAL,
			.inv_int_pol    = 0,
		}
	};

//	if ((engineerid & 0x3) == 0x0){
		struct pm8xxx_gpio_init_info als_shutdown = {
			PM8058_GPIO_PM_TO_SYS(MECHA_ALS_SHUTDOWN),
			{
				.direction      = PM_GPIO_DIR_OUT,
				.output_buffer  = PM_GPIO_OUT_BUF_CMOS,
				.output_value   = 0,
				.pull           = PM_GPIO_PULL_NO,
				.vin_sel        = PM8058_GPIO_VIN_L5,
				.out_strength   = PM_GPIO_STRENGTH_HIGH,
				.function       = PM_GPIO_FUNC_NORMAL,
				.inv_int_pol    = 0,
			},
//		},
	};

	/* P-sensor and light sensor */
	struct pm8xxx_gpio_init_info gpio_ps_en = {
		PM8058_GPIO_PM_TO_SYS(MECHA_GPIO_PS_EN),
		{
			.direction      = PM_GPIO_DIR_OUT,
			.output_buffer  = PM_GPIO_OUT_BUF_CMOS,
			.output_value   = 0,
			.pull           = PM_GPIO_PULL_NO,
			.vin_sel        = PM8058_GPIO_VIN_L5,
			.out_strength   = PM_GPIO_STRENGTH_HIGH,
			.function       = PM_GPIO_FUNC_NORMAL,
			.inv_int_pol    = 0,
		}
	};

	struct pm8xxx_gpio_init_info gpio_ps_int_n = {
		PM8058_GPIO_PM_TO_SYS(MECHA_GPIO_PS_INT_N),
		{
			.direction      = PM_GPIO_DIR_IN,
			.output_buffer  = 0,
			.output_value   = 0,
			.pull           = PM_GPIO_PULL_DN,
			.vin_sel        = PM8058_GPIO_VIN_L5,
			.out_strength   = 0,
			.function       = PM_GPIO_FUNC_NORMAL,
			.inv_int_pol    = 0,
		}
	};

	/* MicroP INT*/
	struct pm8xxx_gpio_init_info gpio_up_int_n = {
		PM8058_GPIO_PM_TO_SYS(MECHA_GPIO_UP_INT_N),
		{
			.direction      = PM_GPIO_DIR_IN,
			.output_buffer  = 0,
			.output_value   = 0,
			.pull           = PM_GPIO_PULL_NO,
			.vin_sel        = PM8058_GPIO_VIN_L5,
			.out_strength   = 0,
			.function       = PM_GPIO_FUNC_NORMAL,
			.inv_int_pol    = 0,
		}
	};

	/* G Sensor INT*/
	/* struct pm8xxx_gpio_init_info gpio_gsensor_int_n = {
		PM8058_GPIO_PM_TO_SYS(MECHA_GPIO_GSENSOR_INT_N),
		{
			.direction      = PM_GPIO_DIR_IN,
			.output_buffer  = 0,
			.output_value   = 0,
			.pull           = PM_GPIO_PULL_NO,
			.vin_sel        = PM8058_GPIO_VIN_L5,
			.out_strength   = 0,
			.function       = PM_GPIO_FUNC_NORMAL,
			.inv_int_pol    = 0,
		},
	}; */

	/* E-Compass INT */
	struct pm8xxx_gpio_init_info gpio_compass_int_n = {
		PM8058_GPIO_PM_TO_SYS(MECHA_GPIO_COMPASS_INT_N),
		{
			.direction      = PM_GPIO_DIR_IN,
			.output_buffer  = 0,
			.output_value   = 0,
			.pull           = PM_GPIO_PULL_NO,
			.vin_sel        = PM8058_GPIO_VIN_L5,
			.out_strength   = 0,
			.function       = PM_GPIO_FUNC_NORMAL,
			.inv_int_pol    = 0,
		},
	};

	/* uP Reset */
	struct pm8xxx_gpio_init_info gpio_up_reset_n = {
		PM8058_GPIO_PM_TO_SYS(MECHA_GPIO_UP_RESET_N),
		{
			.direction      = PM_GPIO_DIR_OUT,
			.output_buffer  = PM_GPIO_OUT_BUF_CMOS,
			.output_value   = 1,
			.pull           = PM_GPIO_PULL_NO,
			.vin_sel        = PM8058_GPIO_VIN_L5,
			.out_strength   = PM_GPIO_STRENGTH_HIGH,
			.function       = PM_GPIO_FUNC_NORMAL,
			.inv_int_pol    = 0,
		}
	};

	/* usb hub switch */
//	if (system_rev >= 2)
//	{
		struct pm8xxx_gpio_init_info gpio_usb_hub_switch = {
			PM8058_GPIO_PM_TO_SYS(MECHA_GPIO_USB_HUB_SWITCH),
			{
				.direction      = PM_GPIO_DIR_OUT,
				.output_buffer  = PM_GPIO_OUT_BUF_CMOS,
				.output_value   = 0,
				.pull           = PM_GPIO_PULL_NO,
				.vin_sel        = PM8058_GPIO_VIN_L5,
				.out_strength   = PM_GPIO_STRENGTH_HIGH,
				.function       = PM_GPIO_FUNC_NORMAL,
				.inv_int_pol    = 0,
			},
//		},
	};

	/* usb hub reset*/
	struct pm8xxx_gpio_init_info usb_hub_reset = {
		PM8058_GPIO_PM_TO_SYS(MECHA_USB_HUB_RESET),
		{
			.direction      = PM_GPIO_DIR_OUT,
			.output_buffer  = PM_GPIO_OUT_BUF_CMOS,
			.output_value   = 0,
			.pull           = PM_GPIO_PULL_NO,
			.vin_sel        = PM8058_GPIO_VIN_L5,
			.out_strength   = PM_GPIO_STRENGTH_HIGH,
			.function       = PM_GPIO_FUNC_NORMAL,
			.inv_int_pol    = 0,
		},
	};

	/* usb hub power*/
	struct pm8xxx_gpio_init_info usb_hub_pwr = {
		PM8058_GPIO_PM_TO_SYS(MECHA_USB_HUB_PWR),
		{
			.direction      = PM_GPIO_DIR_OUT,
			.output_buffer  = PM_GPIO_OUT_BUF_CMOS,
			.output_value   = 0,
			.pull           = PM_GPIO_PULL_NO,
			.vin_sel        = PM8058_GPIO_VIN_L5,
			.out_strength   = PM_GPIO_STRENGTH_HIGH,
			.function       = PM_GPIO_FUNC_NORMAL,
			.inv_int_pol    = 0,
		},
	};

	rc = pm8xxx_gpio_config(tp_rstz.gpio, &tp_rstz.config);
	if (rc) {
		pr_err("%s MECHA_TP_RSTz config failed\n", __func__);
		return rc;
	}

	rc = pm8xxx_gpio_config(green_led.gpio, &green_led.config);
	if (rc) {
		pr_err("%s MECHA_GREEN_LED config failed\n", __func__);
		return rc;
	}

	rc = pm8xxx_gpio_config(amber_led.gpio, &amber_led.config);
	if (rc) {
		pr_err("%s MECHA_AMBER_LED config failed\n", __func__);
		return rc;
	}

	rc = pm8xxx_gpio_config(aud_hp_detz.gpio, &aud_hp_detz.config);
	if (rc) {
		pr_err("%s MECHA_AUD_HP_DETz config failed\n", __func__);
		return rc;
	}

	rc = pm8xxx_gpio_config(gpio_sdmc_cd_n.gpio, &gpio_sdmc_cd_n.config);
	if (rc) {
		pr_err("%s MECHA_GPIO_SDMC_CD_N config failed\n", __func__);
		return rc;
	}

	rc = pm8xxx_gpio_config(als_shutdown.gpio, &als_shutdown.config);
	if (rc) {
		pr_err("%s MECHA_ALS_SHUTDOWN config failed\n", __func__);
		return rc;
	} else
		pr_info("%s [cm3628][PS]MECHA_ALS_SHUTDOWN config ok\n", __func__);

	rc = pm8xxx_gpio_config(gpio_ps_en.gpio, &gpio_ps_en.config);
	if (rc) {
		pr_err("%s MECHA_GPIO_PS_EN config failed\n", __func__);
		return rc;
	} else
		pr_info("%s [cm3628][PS]MECHA_GPIO_PS_EN config ok\n", __func__);

	rc = pm8xxx_gpio_config(gpio_ps_int_n.gpio, &gpio_ps_int_n.config);
	if (rc) {
		pr_err("%s MECHA_GPIO_PS_INT_N config failed\n", __func__);
		return rc;
	} else
		pr_info("%s [cm3628][PS]MECHA_GPIO_PS_INT_N config ok\n", __func__);

	rc = pm8xxx_gpio_config(gpio_up_int_n.gpio, &gpio_up_int_n.config);
	if (rc) {
		pr_err("%s MECHA_GPIO_UP_INT_N config failed\n", __func__);
		return rc;
	}

	/*rc = pm8xxx_gpio_config(gpio_gsensor_int_n.gpio, &gpio_gsensor_int_n.config);
	if (rc) {
		pr_err("%s MECHA_GPIO_GSENSOR_INT_N config failed\n", __func__);
		return rc;
	}*/

	rc = pm8xxx_gpio_config(gpio_compass_int_n.gpio, &gpio_compass_int_n.config);
	if (rc) {
		pr_err("%s MECHA_GPIO_COMPASS_INT_N config failed\n", __func__);
		return rc;
	} else
		pr_info("%s [AKM8975] MECHA_GPIO_COMPASS_INT_N config ok\n",
				__func__);

	rc = pm8xxx_gpio_config(gpio_up_reset_n.gpio, &gpio_up_reset_n.config);
	if (rc) {
		pr_err("%s MECHA_GPIO_UP_RESET_N config failed\n", __func__);
		return rc;
	}

	rc = pm8xxx_gpio_config(gpio_usb_hub_switch.gpio, &gpio_usb_hub_switch.config);
	if (rc) {
		pr_err("%s MECHA_GPIO_USB_HUB_SWITCH config failed\n", __func__);
		return rc;
	}

	rc = pm8xxx_gpio_config(usb_hub_reset.gpio, &usb_hub_reset.config);
	if (rc) {
		pr_err("%s MECHA_USB_HUB_RESET config failed\n", __func__);
		return rc;
	}

	rc = pm8xxx_gpio_config(usb_hub_pwr.gpio, &usb_hub_pwr.config);
	if (rc) {
		pr_err("%s MECHA_USB_HUB_PWR config failed\n", __func__);
		return rc;
	}


	vol_up.gpio = MECHA_VOL_UP;
	pm8xxx_gpio_config(vol_up.gpio, &vol_up.config);
	vol_dn.gpio = MECHA_VOL_DN;
	pm8xxx_gpio_config(vol_dn.gpio, &vol_dn.config);

	return 0;
}

static struct htc_headset_gpio_platform_data htc_headset_gpio_data = {
	.hpin_gpio			= PM8058_GPIO_PM_TO_SYS(MECHA_AUD_HP_DETz),
	.key_enable_gpio	= 0,
	.mic_select_gpio	= MECHA_AUD_MICPATH_SEL,
};

static struct platform_device htc_headset_gpio = {
	.name	= "HTC_HEADSET_GPIO",
	.id	= -1,
	.dev	= {
		.platform_data	= &htc_headset_gpio_data,
	},
};

/* HTC_HEADSET_PMIC Driver */
static struct htc_headset_pmic_platform_data htc_headset_pmic_data = {
	.driver_flag	= DRIVER_HS_PMIC_RPC_KEY |
			  DRIVER_HS_PMIC_DYNAMIC_THRESHOLD,
	.hpin_gpio	= PM8058_GPIO_PM_TO_SYS(MECHA_AUD_HP_DETz),
	.hpin_irq	= MSM_GPIO_TO_INT(
			  PM8058_GPIO_PM_TO_SYS(MECHA_AUD_HP_DETz)),
	.adc_mic	= 15672,
	.adc_remote	= {0, 2130, 3153, 7050, 8567, 15672},
	.hs_controller	= HS_PMIC_CONTROLLER_2,
	.hs_switch	= HS_PMIC_SC_SWITCH_TYPE,
};

static struct platform_device htc_headset_pmic = {
	.name	= "HTC_HEADSET_PMIC",
	.id	= -1,
	.dev	= {
		.platform_data	= &htc_headset_pmic_data,
	},
};

static struct platform_device *headset_devices[] = {
	&htc_headset_gpio,
	&htc_headset_pmic,
	/* Please put the headset detection driver on the last */
};

static struct htc_headset_mgr_platform_data htc_headset_mgr_data = {
	.driver_flag		= DRIVER_HS_MGR_RPC_SERVER,
	.headset_devices_num	= ARRAY_SIZE(headset_devices),
	.headset_devices	= headset_devices,
};

static struct lightsensor_smd_platform_data lightsensor_data = {
	.levels = { 0x1, 0x3, 0x5, 0x3E7, 0x4B0, 0x1E4C, 0x2453, 0x2AF7,
			0x3140, 0xFFFF },
	.golden_adc = 0x432A,
	.ls_power = capella_cm3602_power,
};

static struct platform_device lightsensor_pdev = {
	.name = "lightsensor_smd",
	.id = -1,
	.dev = {
		.platform_data = &lightsensor_data
	}
};

static struct platform_device microp_devices[] = {
	{
		.name	= "HTC_HEADSET_MGR",
		.id	= -1,
		.dev	= {
			.platform_data	= &htc_headset_mgr_data,
		},
	},
};

static struct microp_i2c_platform_data microp_data = {
	.num_devices = ARRAY_SIZE(microp_devices),
	.microp_devices = microp_devices,
	.gpio_reset = MECHA_GPIO_UP_RESET_N,
};
static struct pm8058_led_config pm_led_config[] = {
	{
		.name = "green",
		.type = PM8058_LED_RGB,
		.bank = 0,
		.pwm_size = 9,
		.clk = PM_PWM_CLK_32KHZ,
		.pre_div = PM_PWM_PREDIVIDE_2,
		.pre_div_exp = 1,
		.pwm_value = 511,
	},
	{
		.name = "amber",
		.type = PM8058_LED_RGB,
		.bank = 1,
		.pwm_size = 9,
		.clk = PM_PWM_CLK_32KHZ,
		.pre_div = PM_PWM_PREDIVIDE_2,
		.pre_div_exp = 1,
		.pwm_value = 511,
	},
	{
		.name = "button-backlight",
		.type = PM8058_LED_CURRENT,
		.bank = 3,
		.flags = PM8058_LED_LTU_EN | PM8058_LED_FADE_EN,
		.period_us = USEC_PER_SEC / 1000,
		.start_index = 0,
		.duites_size = 8,
		.duty_time_ms = 32,
		.lut_flag = PM_PWM_LUT_RAMP_UP | PM_PWM_LUT_PAUSE_HI_EN,
		.out_current = 200,
	},

};

static struct pm8058_led_platform_data pm8058_leds_data = {
	.led_config = pm_led_config,
	.num_leds = ARRAY_SIZE(pm_led_config),
	.duties = {0, 15, 30, 45, 60, 75, 90, 100,
		   100, 90, 75, 60, 45, 30, 15, 0,
		   0, 0, 0, 0, 0, 0, 0, 0,
		   0, 0, 0, 0, 0, 0, 0, 0,
		   0, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0},
};

static struct platform_device pm8058_leds = {
	.name	= "leds-pm8058",
	.id	= -1,
	.dev	= {
		.platform_data	= &pm8058_leds_data,
	},
};

static struct pm8058_led_config pm_led_config_XC[] = {
	{
		.name = "green",
		.type = PM8058_LED_RGB,
		.bank = 0,
		.pwm_size = 9,
		.clk = PM_PWM_CLK_32KHZ,
		.pre_div = PM_PWM_PREDIVIDE_2,
		.pre_div_exp = 1,
		.pwm_value = 511,
	},
	{
		.name = "amber",
		.type = PM8058_LED_RGB,
		.bank = 1,
		.pwm_size = 9,
		.clk = PM_PWM_CLK_32KHZ,
		.pre_div = PM_PWM_PREDIVIDE_2,
		.pre_div_exp = 1,
		.pwm_value = 511,
	},
	{
		.name = "button-backlight",
		.type = PM8058_LED_DRVX,
		.bank = 6,
		.flags = PM8058_LED_LTU_EN | PM8058_LED_FADE_EN,
		.period_us = USEC_PER_SEC / 1000,
		.start_index = 0,
		.duites_size = 8,
		.duty_time_ms = 32,
		.lut_flag = PM_PWM_LUT_RAMP_UP | PM_PWM_LUT_PAUSE_HI_EN,
		.out_current = 6,
	},
};

static struct pm8058_led_platform_data pm8058_leds_data_XC = {
	.led_config = pm_led_config_XC,
	.num_leds = ARRAY_SIZE(pm_led_config),
	.duties = {0, 15, 30, 45, 60, 75, 90, 100,
		   100, 90, 75, 60, 45, 30, 15, 0,
		   0, 0, 0, 0, 0, 0, 0, 0,
		   0, 0, 0, 0, 0, 0, 0, 0,
		   0, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0, 0, 0},
};

static struct platform_device pm8058_leds_XC = {
	.name	= "leds-pm8058-xc",
	.id	= -1,
	.dev	= {
		.platform_data	= &pm8058_leds_data_XC,
	},
};

static struct i2c_board_info i2c_devices[] = {
	{
		I2C_BOARD_INFO(ATMEL_QT602240_NAME, 0x94 >> 1),
		.platform_data = &mecha_ts_atmel_data,
		.irq = MSM_GPIO_TO_INT(MECHA_GPIO_TP_INT_N)
	},
	{
		I2C_BOARD_INFO(SMSC251X_NAME, 0x58 >> 1),
		.platform_data = &mecha_smsc251x_data
	},
};

static struct i2c_board_info i2c_microp_devices[] = {
	{
		I2C_BOARD_INFO(MICROP_I2C_NAME, 0xCC >> 1),
		.platform_data = &microp_data,
		.irq = MSM_GPIO_TO_INT(PM8058_GPIO_PM_TO_SYS(MECHA_GPIO_UP_INT_N)),
	},
};

/* Regulator API support */

#ifdef CONFIG_MSM_PROC_COMM_REGULATOR
static struct platform_device msm_proccomm_regulator_dev = {
	.name = PROCCOMM_REGULATOR_DEV_NAME,
	.id   = -1,
	.dev  = {
		.platform_data = &msm7x30_proccomm_regulator_data
	}
};
#endif

static int pm8058_pwm_config(struct pwm_device *pwm, int ch, int on)
{
	struct pm_gpio pwm_gpio_config = {
		.direction      = PM_GPIO_DIR_OUT,
		.output_buffer  = PM_GPIO_OUT_BUF_CMOS,
		.output_value   = 0,
		.pull           = PM_GPIO_PULL_NO,
		.vin_sel        = PM8058_GPIO_VIN_S3,
		.out_strength   = PM_GPIO_STRENGTH_HIGH,
		.function       = PM_GPIO_FUNC_2,
	};
	int	rc = -EINVAL;
	int	id, mode, max_mA;

	id = mode = max_mA = 0;
	switch (ch) {
	case 0:
	case 1:
	case 2:
		if (on) {
			id = 24 + ch;
			rc = pm8xxx_gpio_config(PM8058_GPIO_PM_TO_SYS(id - 1),
							&pwm_gpio_config);
			if (rc)
				pr_err("%s: pm8xxx_gpio_config(%d): rc=%d\n",
				       __func__, id, rc);
		}
		break;

	case 3:
		id = PM_PWM_LED_KPD;
		mode = PM_PWM_CONF_DTEST3;
		max_mA = 200;
		break;

	case 4:
		id = PM_PWM_LED_0;
		mode = PM_PWM_CONF_PWM1;
		max_mA = 40;
		break;

	case 5:
		id = PM_PWM_LED_2;
		mode = PM_PWM_CONF_PWM2;
		max_mA = 40;
		break;

	case 6:
		id = PM_PWM_LED_FLASH;
		mode = PM_PWM_CONF_DTEST3;
		max_mA = 200;
		break;

	default:
		break;
	}

	if (ch >= 3 && ch <= 6) {
		if (!on) {
			mode = PM_PWM_CONF_NONE;
			max_mA = 0;
		}
		rc = pm8058_pwm_config_led(pwm, id, mode, max_mA);
		if (rc)
			pr_err("%s: pm8058_pwm_config_led(ch=%d): rc=%d\n",
			       __func__, ch, rc);
	}

	return rc;
}

static int pm8058_pwm_enable(struct pwm_device *pwm, int ch, int on)
{
	int	rc;

	switch (ch) {
	case 7:
		rc = pm8058_pwm_set_dtest(pwm, on);
		if (rc)
			pr_err("%s: pwm_set_dtest(%d): rc=%d\n",
			       __func__, on, rc);
		break;
	default:
		rc = -EINVAL;
		break;
	}
	return rc;
}


static struct pm8058_pwm_pdata pm8058_pwm_data = {
	.config         = pm8058_pwm_config,
	.enable         = pm8058_pwm_enable,
};

static struct pm8xxx_irq_platform_data pm8xxx_irq_pdata = {
	.irq_base		= PMIC8058_IRQ_BASE,
	.devirq			= MSM_GPIO_TO_INT(PMIC_GPIO_INT),
	.irq_trigger_flag       = IRQF_TRIGGER_LOW,
};

static struct pm8xxx_gpio_platform_data pm8xxx_gpio_pdata = {
	.gpio_base		= PM8058_GPIO_PM_TO_SYS(0),
};

static struct pm8xxx_mpp_platform_data pm8xxx_mpp_pdata = {
	.mpp_base	= PM8058_MPP_PM_TO_SYS(0),
};

static struct pm8058_platform_data pm8058_7x30_data = {
	.irq_pdata		= &pm8xxx_irq_pdata,
	.gpio_pdata		= &pm8xxx_gpio_pdata,
	.mpp_pdata		= &pm8xxx_mpp_pdata,
	.pwm_pdata		= &pm8058_pwm_data,
};

#ifdef CONFIG_MSM_SSBI
static struct msm_ssbi_platform_data msm7x30_ssbi_pm8058_pdata __devinitdata = {
	.controller_type = MSM_SBI_CTRL_PMIC_ARBITER,
	.slave	= {
		.name			= "pm8058-core",
		.irq = MSM_GPIO_TO_INT(PMIC_GPIO_INT),
		.platform_data		= &pm8058_7x30_data,
	},
	.rspinlock_name	= "D:PMIC_SSBI",
};
#endif

static struct tps65200_platform_data tps65200_data = {
	.gpio_chg_int = PM8058_GPIO_IRQ(PM8058_IRQ_BASE, MECHA_GPIO_CHG_INT),
};

static struct i2c_board_info i2c_tps_devices[] = {
	{
		I2C_BOARD_INFO("tps65200", 0xD4 >> 1),
		.platform_data = &tps65200_data,
	},
};

static struct i2c_board_info msm_camera_boardinfo[] __initdata = {
#ifdef CONFIG_S5K3H1GX
	{
		I2C_BOARD_INFO("s5k3h1gx", 0x20 >> 1),
	},
#endif
#ifdef CONFIG_S5K6AAFX
	{
		I2C_BOARD_INFO("s5k6aafx", 0x78 >> 1),
	},
#endif
};

static uint32_t camera_off_gpio_table[] = {
  GPIO_CFG(CAM2_RST, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
  GPIO_CFG(CAM1_PWD, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
  GPIO_CFG(CAM2_PWD, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
  GPIO_CFG(4, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_8MA), /* DAT2 */
  GPIO_CFG(5, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_8MA), /* DAT3 */
  GPIO_CFG(6, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_8MA), /* DAT4 */
  GPIO_CFG(7, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_8MA), /* DAT5 */
  GPIO_CFG(8, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_8MA), /* DAT6 */
  GPIO_CFG(9, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_8MA), /* DAT7 */
  GPIO_CFG(10, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_8MA), /* DAT8 */
  GPIO_CFG(11, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_8MA), /* DAT9 */
  GPIO_CFG(12, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_8MA), /* PCLK */
  GPIO_CFG(13, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_8MA), /* HSYNC_IN */
  GPIO_CFG(14, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_8MA), /* VSYNC_IN */
  GPIO_CFG(15, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_14MA), /* MCLK */
};

static uint32_t camera_on_gpio_table[] = {
#if 0
  GPIO_CFG(CAM2_RST, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
  GPIO_CFG(CAM1_PWD, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
  GPIO_CFG(CAM2_PWD, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
#endif
  GPIO_CFG(4, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_6MA), /* DAT2 */
  GPIO_CFG(5, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_6MA), /* DAT3 */
  GPIO_CFG(6, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_6MA), /* DAT4 */
  GPIO_CFG(7, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_6MA), /* DAT5 */
  GPIO_CFG(8, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_6MA), /* DAT6 */
  GPIO_CFG(9, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_6MA), /* DAT7 */
  GPIO_CFG(10, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_6MA), /* DAT8 */
  GPIO_CFG(11, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_6MA), /* DAT9 */
  GPIO_CFG(12, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_6MA), /* PCLK */
  GPIO_CFG(13, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_6MA), /* HSYNC_IN */
  GPIO_CFG(14, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_6MA), /* VSYNC_IN */
  GPIO_CFG(15, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA), /* MCLK */
};

static int sensor_power_enable(char *power, unsigned volt)
{
	struct vreg *vreg_gp;
	int rc;

	if (power == NULL)
		return EIO;

	vreg_gp = vreg_get(NULL, power);
	if (IS_ERR(vreg_gp)) {
		pr_err("[CAM] %s: vreg_get(%s) failed (%ld)\n",
			__func__, power, PTR_ERR(vreg_gp));
		return EIO;
	}

	rc = vreg_set_level(vreg_gp, volt);
	if (rc) {
		pr_err("[CAM] %s: vreg set %s level failed (%d)\n",
			__func__, power, rc);
		return EIO;
	}

	rc = vreg_enable(vreg_gp);
	if (rc) {
		pr_err("[CAM] %s: vreg enable %s failed (%d)\n",
			__func__, power, rc);
		return EIO;
	}
	return rc;
}

static int sensor_power_disable(char *power)
{
	struct vreg *vreg_gp;
	int rc;
	vreg_gp = vreg_get(NULL, power);
	if (IS_ERR(vreg_gp)) {
		pr_err("[CAM] %s: vreg_get(%s) failed (%ld)\n",
			__func__, power, PTR_ERR(vreg_gp));
		return EIO;
	}

	rc = vreg_disable(vreg_gp);
	if (rc) {
		pr_err("[CAM] %s: vreg disable %s failed (%d)\n",
			__func__, power, rc);
		return EIO;
	}
	return rc;
}

static int sensor_vreg_on(void)
{
  int rc;
  pr_info("[CAM] %s camera vreg on\n", __func__);

  /* main camera VCM power */
  rc = sensor_power_enable("gp4", 2850);
  pr_info("[CAM] sensor_power_enable(\"gp4\", 2850) == %d\n", rc);

  /* main / 2nd camera analog power */
  rc = sensor_power_enable("gp6", 2850);
  pr_info("[CAM] sensor_power_enable(\"gp6\", 2850) == %d\n", rc);
  msleep(5);

  /* main / 2nd camera digital power */
  rc = sensor_power_enable("gp2", 1800);
  pr_info("[CAM] sensor_power_enable(\"gp2\", 1800) == %d\n", rc);
  msleep(5);

  /* 2nd camera IO power */
  rc = gpio_request(36, "s5k6aafx");
  if (rc < 0)
    pr_err("GPIO (%d) request fail\n", 36);
  else
    gpio_direction_output(36, 1);
  gpio_free(36);

  return rc;
}



static int sensor_vreg_off(void)
{
  int rc;

  /* 2nd camera IO power */
  rc = gpio_request(36, "s5k6aafx");
  if (rc < 0)
    pr_err("GPIO (%d) request fail\n", 36);
  else
    gpio_direction_output(36, 0);
  gpio_free(36);

  /* main / 2nd camera digital power */
  rc = sensor_power_disable("gp2");
  pr_info("[CAM] sensor_power_disable(\"gp2\") == %d\n", rc);

  /* main / 2nd camera analog power */
  rc = sensor_power_disable("gp6");
  pr_info("[CAM] sensor_power_disable(\"gp6\") == %d\n", rc);

  /* main camera VCM power */
  rc = sensor_power_disable("gp4");
  pr_info("[CAM] sensor_power_disable(\"gp4\") == %d\n", rc);

  return rc;
}


static int config_camera_on_gpios(void)
{
	pr_info("[CAM] config_camera_on_gpios\n");
	config_gpio_table(camera_on_gpio_table,
		ARRAY_SIZE(camera_on_gpio_table));
	return 0;
}

static void config_camera_off_gpios(void)
{
	pr_info("[CAM] config_camera_off_gpios\n");
	config_gpio_table(camera_off_gpio_table,
		ARRAY_SIZE(camera_off_gpio_table));
}

static struct resource msm_camera_resources[] = {
	{
		.start	= 0xA6000000,
		.end	= 0xA6000000 + SZ_1M - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_VFE,
		.end	= INT_VFE,
		.flags	= IORESOURCE_IRQ,
	},
};

struct msm_camera_device_platform_data camera_device_data = {
  .camera_gpio_on  = config_camera_on_gpios,
  .camera_gpio_off = config_camera_off_gpios,
  .ioext.mdcphy = MSM_MDC_PHYS,
  .ioext.mdcsz  = MSM_MDC_SIZE,
  .ioext.appphy = MSM_CLK_CTL_PHYS,
  .ioext.appsz  = MSM_CLK_CTL_SIZE,
  .ioext.camifpadphy = 0xAB000000,
  .ioext.camifpadsz  = 0x00000400,
#ifdef CONFIG_S5K3H1GX
  .ioext.csiphy = 0xA6100000,
  .ioext.csisz  = 0x00000400,
  .ioext.csiirq = INT_CSI,
#endif
};

static void s5k3h1gx_clk_switch(void){
	int rc = 0;
	pr_info("[CAM] Doing clk switch (s5k3h1gx)\n");
	rc = gpio_request(CLK_SWITCH, "s5k3h1gx");

	if (rc < 0)
		pr_err("[CAM] GPIO (%d) request fail\n", CLK_SWITCH);
	else
		gpio_direction_output(CLK_SWITCH, 0);

	gpio_free(CLK_SWITCH);
	return;
}

#ifdef CONFIG_S5K6AAFX
static void s5k6aafx_clk_switch(void){
	int rc = 0;
	pr_info("[CAM] Doing clk switch (s5k6aafx)\n");
	rc = gpio_request(CLK_SWITCH, "s5k6aafx");

	if (rc < 0)
		pr_err("[CAM] GPIO (%d) request fail\n", CLK_SWITCH);
	else
		gpio_direction_output(CLK_SWITCH, 1);

	gpio_free(CLK_SWITCH);
	return;
}
#endif
//#if 1 // PG
static int flashlight_control(int mode)
{
#ifdef CONFIG_FLASHLIGHT_AAT1271
	return aat1271_flashlight_control(mode);
//#else
//	return 0;
//#endif
}

static struct camera_flash_cfg msm_camera_sensor_flash_cfg = {
	.camera_flash = flashlight_control,
	.num_flash_levels = FLASHLIGHT_NUM,
	.low_temp_limit = 5,
	.low_cap_limit = 15,
};
#endif
#ifdef CONFIG_S5K3H1GX
static struct msm_camera_sensor_info msm_camera_sensor_s5k3h1gx_data = {
  .sensor_name = "s5k3h1gx",
  .sensor_pwd = CAM1_PWD,
  .vcm_pwd = CAM1_VCM_PWD,
  .camera_power_on = sensor_vreg_on,
  .camera_power_off = sensor_vreg_off,
  .camera_clk_switch = s5k3h1gx_clk_switch,
  .pdata = &camera_device_data,
  .flash_type = MSM_CAMERA_FLASH_LED,
  .resource = msm_camera_resources,
  .num_resources = ARRAY_SIZE(msm_camera_resources),
#ifdef CONFIG_ARCH_MSM_FLASHLIGHT
  .flash_cfg = &msm_camera_sensor_flash_cfg,
#endif
  .csi_if = 1,
};

static struct platform_device msm_camera_sensor_s5k3h1gx = {
  .name = "msm_camera_s5k3h1gx",
  .dev = {
    .platform_data = &msm_camera_sensor_s5k3h1gx_data,
  },
};
#endif

#ifdef CONFIG_S5K6AAFX
static struct msm_camera_sensor_info msm_camera_sensor_s5k6aafx_data = {
	.sensor_name = "s5k6aafx",
	.sensor_reset = CAM2_RST,
	.sensor_pwd = CAM2_PWD,
	.camera_power_on = sensor_vreg_on,
	.camera_power_off = sensor_vreg_off,
	.camera_clk_switch = s5k6aafx_clk_switch,
	.pdata = &camera_device_data,
	.flash_type = MSM_CAMERA_FLASH_LED,
	.resource = msm_camera_resources,
	.num_resources = ARRAY_SIZE(msm_camera_resources),
};

static struct platform_device msm_camera_sensor_s5k6aafx = {
	.name	   = "msm_camera_s5k6aafx",
	.dev	    = {
		.platform_data = &msm_camera_sensor_s5k6aafx_data,
	},
};
#endif

#ifdef CONFIG_MSM_GEMINI
static struct resource msm_gemini_resources[] = {
	{
		.start  = 0xA3A00000,
		.end    = 0xA3A00000 + 0x0150 - 1,
		.flags  = IORESOURCE_MEM,
	},
	{
		.start  = INT_JPEG,
		.end    = INT_JPEG,
		.flags  = IORESOURCE_IRQ,
	},
};

static struct platform_device msm_gemini_device = {
	.name           = "msm_gemini",
	.resource       = msm_gemini_resources,
	.num_resources  = ARRAY_SIZE(msm_gemini_resources),
};
#endif


#ifdef CONFIG_MSM7KV2_AUDIO

//static unsigned aux_pcm_gpio_off[] = {
//	GPIO_CFG(138, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),   /* PCM_DOUT */
//	GPIO_CFG(139, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),   /* PCM_DIN  */
//	GPIO_CFG(140, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),   /* PCM_SYNC */
//	GPIO_CFG(141, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),   /* PCM_CLK  */
//};

static struct tpa2051d3_platform_data tpa2051d3_platform_data = {
};
//static void __init aux_pcm_gpio_init(void)
//{
//	config_gpio_table(aux_pcm_gpio_off,
//		ARRAY_SIZE(aux_pcm_gpio_off));
//}

#endif /* CONFIG_MSM7KV2_AUDIO */
static struct vreg *vreg_marimba_1;
static struct vreg *vreg_marimba_2;

static unsigned int msm_marimba_setup_power(void)
{
	int rc;

	rc = vreg_enable(vreg_marimba_1);
	if (rc) {
		printk(KERN_ERR "%s: vreg_enable() = %d \n",
					__func__, rc);
		goto out;
	}
	rc = vreg_enable(vreg_marimba_2);
	if (rc) {
		printk(KERN_ERR "%s: vreg_enable() = %d \n",
					__func__, rc);
		goto out;
	}

out:
	return rc;
};

static void msm_marimba_shutdown_power(void)
{
	int rc;

	rc = vreg_disable(vreg_marimba_1);
	if (rc) {
		printk(KERN_ERR "%s: return val: %d \n",
					__func__, rc);
	}
	rc = vreg_disable(vreg_marimba_2);
	if (rc) {
		printk(KERN_ERR "%s: return val: %d \n",
					__func__, rc);
	}
};

/* Slave id address for FM/CDC/QMEMBIST
 * Values can be programmed using Marimba slave id 0
 * should there be a conflict with other I2C devices
 * */
/*#define MARIMBA_SLAVE_ID_FM_ADDR	0x2A*/
#define MARIMBA_SLAVE_ID_CDC_ADDR	0x77
#define MARIMBA_SLAVE_ID_QMEMBIST_ADDR	0X66

static const char *tsadc_id = "MADC";
static const char *vregs_tsadc_name[] = {
	"gp12",
	"s2",
};
static struct vreg *vregs_tsadc[ARRAY_SIZE(vregs_tsadc_name)];

static int marimba_tsadc_power(int vreg_on)
{
	int i, rc = 0;

	for (i = 0; i < ARRAY_SIZE(vregs_tsadc_name); i++) {
		if (!vregs_tsadc[i]) {
			printk(KERN_ERR "%s: vreg_get %s failed (%d)\n",
				__func__, vregs_tsadc_name[i], rc);
			goto vreg_fail;
		}

		rc = vreg_on ? vreg_enable(vregs_tsadc[i]) :
			  vreg_disable(vregs_tsadc[i]);
		if (rc < 0) {
			printk(KERN_ERR "%s: vreg %s %s failed (%d)\n",
				__func__, vregs_tsadc_name[i],
			       vreg_on ? "enable" : "disable", rc);
			goto vreg_fail;
		}
	}
	/* vote for D0 buffer */
	rc = pmapp_clock_vote(tsadc_id, PMAPP_CLOCK_ID_DO,
		vreg_on ? PMAPP_CLOCK_VOTE_ON : PMAPP_CLOCK_VOTE_OFF);
	if (rc)	{
		printk(KERN_ERR "%s: unable to %svote for d0 clk\n",
			__func__, vreg_on ? "" : "de-");
		goto do_vote_fail;
	}

	mdelay(5); /* ensure power is stable */

	return 0;

do_vote_fail:
vreg_fail:
	while (i)
		vreg_disable(vregs_tsadc[--i]);
	return rc;
}

static int marimba_tsadc_vote(int vote_on)
{
	int rc, level;

	level = vote_on ? 1300 : 0;

	rc = pmapp_vreg_level_vote(tsadc_id, PMAPP_VREG_S2, level);
	if (rc < 0)
		printk(KERN_ERR "%s: vreg level %s failed (%d)\n",
			__func__, vote_on ? "on" : "off", rc);

	return rc;
}

static int marimba_tsadc_init(void)
{
	int i, rc = 0;

	for (i = 0; i < ARRAY_SIZE(vregs_tsadc_name); i++) {
		vregs_tsadc[i] = vreg_get(NULL, vregs_tsadc_name[i]);
		if (IS_ERR(vregs_tsadc[i])) {
			printk(KERN_ERR "%s: vreg get %s failed (%ld)\n",
			       __func__, vregs_tsadc_name[i],
			       PTR_ERR(vregs_tsadc[i]));
			rc = PTR_ERR(vregs_tsadc[i]);
			goto vreg_get_fail;
		}
	}

	return rc;

vreg_get_fail:
	while (i)
		vreg_put(vregs_tsadc[--i]);
	return rc;
}

static int marimba_tsadc_exit(void)
{
	int i, rc;

	for (i = 0; i < ARRAY_SIZE(vregs_tsadc_name); i++) {
		if (vregs_tsadc[i])
			vreg_put(vregs_tsadc[i]);
	}

	rc = pmapp_vreg_level_vote(tsadc_id, PMAPP_VREG_S2, 0);
	if (rc < 0)
		printk(KERN_ERR "%s: vreg level off failed (%d)\n",
			__func__, rc);

	return rc;
}

static struct marimba_tsadc_platform_data marimba_tsadc_pdata = {
	.marimba_tsadc_power = marimba_tsadc_power,
	.init		     =  marimba_tsadc_init,
	.exit		     =  marimba_tsadc_exit,
	.level_vote	     =  marimba_tsadc_vote,
	.tsadc_prechg_en = true,
	.setup = {
		.pen_irq_en	=	true,
		.tsadc_en	=	true,
	},
	.params2 = {
		.input_clk_khz		=	2400,
		.sample_prd		=	TSADC_CLK_3,
	},
	.params3 = {
		.prechg_time_nsecs	=	6400,
		.stable_time_nsecs	=	6400,
		.tsadc_test_mode	=	0,
	},
};

static struct vreg *vreg_codec_s4;
static int msm_marimba_codec_power(int vreg_on)
{
	int rc = 0;

	if (!vreg_codec_s4) {

		vreg_codec_s4 = vreg_get(NULL, "s4");

		if (IS_ERR(vreg_codec_s4)) {
			printk(KERN_ERR "%s: vreg_get() failed (%ld)\n",
				__func__, PTR_ERR(vreg_codec_s4));
			rc = PTR_ERR(vreg_codec_s4);
			goto  vreg_codec_s4_fail;
		}
	}

	if (vreg_on) {
		rc = vreg_enable(vreg_codec_s4);
		if (rc)
			printk(KERN_ERR "%s: vreg_enable() = %d \n",
					__func__, rc);
		goto vreg_codec_s4_fail;
	} else {
		rc = vreg_disable(vreg_codec_s4);
		if (rc)
			printk(KERN_ERR "%s: vreg_disable() = %d \n",
					__func__, rc);
		goto vreg_codec_s4_fail;
	}

vreg_codec_s4_fail:
	return rc;
}

static struct marimba_codec_platform_data mariba_codec_pdata = {
	.marimba_codec_power =  msm_marimba_codec_power,
};

static struct marimba_platform_data marimba_pdata = {
	/*.slave_id[MARIMBA_SLAVE_ID_FM]       = MARIMBA_SLAVE_ID_FM_ADDR,*/
	.slave_id[MARIMBA_SLAVE_ID_CDC]	     = MARIMBA_SLAVE_ID_CDC_ADDR,
	.slave_id[MARIMBA_SLAVE_ID_QMEMBIST] = MARIMBA_SLAVE_ID_QMEMBIST_ADDR,
	.marimba_setup = msm_marimba_setup_power,
	.marimba_shutdown = msm_marimba_shutdown_power,
	/*.fm = &marimba_fm_pdata,*/
	.tsadc = &marimba_tsadc_pdata,
	.codec = &mariba_codec_pdata,
};

static void __init mecha_init_marimba(void)
{
	vreg_marimba_1 = vreg_get(NULL, "s2");
	if (IS_ERR(vreg_marimba_1)) {
		printk(KERN_ERR "%s: vreg get failed (%ld)\n",
			__func__, PTR_ERR(vreg_marimba_1));
		return;
	}
	vreg_marimba_2 = vreg_get(NULL, "gp16");
	if (IS_ERR(vreg_marimba_2)) {
		printk(KERN_ERR "%s: vreg get failed (%ld)\n",
			__func__, PTR_ERR(vreg_marimba_2));
		return;
	}
}

static struct i2c_board_info tpa2051_devices[] = {
	{
		I2C_BOARD_INFO(TPA2051D3_I2C_NAME, 0xE0 >> 1),
		.platform_data = &tpa2051d3_platform_data,
	},
};

#define DEC0_FORMAT ((1<<MSM_ADSP_CODEC_MP3)| \
	(1<<MSM_ADSP_CODEC_AAC)|(1<<MSM_ADSP_CODEC_WMA)| \
	(1<<MSM_ADSP_CODEC_WMAPRO)|(1<<MSM_ADSP_CODEC_AMRWB)| \
	(1<<MSM_ADSP_CODEC_AMRNB)|(1<<MSM_ADSP_CODEC_WAV)| \
	(1<<MSM_ADSP_CODEC_ADPCM)|(1<<MSM_ADSP_CODEC_YADPCM)| \
	(1<<MSM_ADSP_CODEC_EVRC)|(1<<MSM_ADSP_CODEC_QCELP))
#define DEC1_FORMAT ((1<<MSM_ADSP_CODEC_MP3)| \
	(1<<MSM_ADSP_CODEC_AAC)|(1<<MSM_ADSP_CODEC_WMA)| \
	(1<<MSM_ADSP_CODEC_WMAPRO)|(1<<MSM_ADSP_CODEC_AMRWB)| \
	(1<<MSM_ADSP_CODEC_AMRNB)|(1<<MSM_ADSP_CODEC_WAV)| \
	(1<<MSM_ADSP_CODEC_ADPCM)|(1<<MSM_ADSP_CODEC_YADPCM)| \
	(1<<MSM_ADSP_CODEC_EVRC)|(1<<MSM_ADSP_CODEC_QCELP))
 #define DEC2_FORMAT ((1<<MSM_ADSP_CODEC_MP3)| \
	(1<<MSM_ADSP_CODEC_AAC)|(1<<MSM_ADSP_CODEC_WMA)| \
	(1<<MSM_ADSP_CODEC_WMAPRO)|(1<<MSM_ADSP_CODEC_AMRWB)| \
	(1<<MSM_ADSP_CODEC_AMRNB)|(1<<MSM_ADSP_CODEC_WAV)| \
	(1<<MSM_ADSP_CODEC_ADPCM)|(1<<MSM_ADSP_CODEC_YADPCM)| \
	(1<<MSM_ADSP_CODEC_EVRC)|(1<<MSM_ADSP_CODEC_QCELP))
 #define DEC3_FORMAT ((1<<MSM_ADSP_CODEC_MP3)| \
	(1<<MSM_ADSP_CODEC_AAC)|(1<<MSM_ADSP_CODEC_WMA)| \
	(1<<MSM_ADSP_CODEC_WMAPRO)|(1<<MSM_ADSP_CODEC_AMRWB)| \
	(1<<MSM_ADSP_CODEC_AMRNB)|(1<<MSM_ADSP_CODEC_WAV)| \
	(1<<MSM_ADSP_CODEC_ADPCM)|(1<<MSM_ADSP_CODEC_YADPCM)| \
	(1<<MSM_ADSP_CODEC_EVRC)|(1<<MSM_ADSP_CODEC_QCELP))
#define DEC4_FORMAT (1<<MSM_ADSP_CODEC_MIDI)

static unsigned int dec_concurrency_table[] = {
	/* Audio LP */
	0,
	0,
	0,
	0,
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_MODE_LP)|
	(1<<MSM_ADSP_OP_DM)),

	/* Concurrency 1 */
	(DEC4_FORMAT),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),

	 /* Concurrency 2 */
	(DEC4_FORMAT),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),

	/* Concurrency 3 */
	(DEC4_FORMAT),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),

	/* Concurrency 4 */
	(DEC4_FORMAT),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),

	/* Concurrency 5 */
	(DEC4_FORMAT),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_TUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),

	/* Concurrency 6 */
	(DEC4_FORMAT),
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC0_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
};

#define DEC_INFO(name, queueid, decid, nr_codec) { .module_name = name, \
	.module_queueid = queueid, .module_decid = decid, \
	.nr_codec_support = nr_codec}

#define DEC_INSTANCE(max_instance_same, max_instance_diff) { \
	.max_instances_same_dec = max_instance_same, \
	.max_instances_diff_dec = max_instance_diff}

static struct msm_adspdec_info dec_info_list[] = {
	DEC_INFO("AUDPLAY4TASK", 17, 4, 1),  /* AudPlay4BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY3TASK", 16, 3, 11),  /* AudPlay3BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY2TASK", 15, 2, 11),  /* AudPlay2BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY1TASK", 14, 1, 11),  /* AudPlay1BitStreamCtrlQueue */
	DEC_INFO("AUDPLAY0TASK", 13, 0, 11), /* AudPlay0BitStreamCtrlQueue */
};

static struct dec_instance_table dec_instance_list[][MSM_MAX_DEC_CNT] = {
	/* Non Turbo Mode */
	{
		DEC_INSTANCE(4, 3), /* WAV */
		DEC_INSTANCE(4, 3), /* ADPCM */
		DEC_INSTANCE(4, 2), /* MP3 */
		DEC_INSTANCE(0, 0), /* Real Audio */
		DEC_INSTANCE(4, 2), /* WMA */
		DEC_INSTANCE(3, 2), /* AAC */
		DEC_INSTANCE(0, 0), /* Reserved */
		DEC_INSTANCE(0, 0), /* MIDI */
		DEC_INSTANCE(4, 3), /* YADPCM */
		DEC_INSTANCE(4, 3), /* QCELP */
		DEC_INSTANCE(4, 3), /* AMRNB */
		DEC_INSTANCE(1, 1), /* AMRWB/WB+ */
		DEC_INSTANCE(4, 3), /* EVRC */
		DEC_INSTANCE(1, 1), /* WMAPRO */
	},
	/* Turbo Mode */
	{
		DEC_INSTANCE(4, 3), /* WAV */
		DEC_INSTANCE(4, 3), /* ADPCM */
		DEC_INSTANCE(4, 3), /* MP3 */
		DEC_INSTANCE(0, 0), /* Real Audio */
		DEC_INSTANCE(4, 3), /* WMA */
		DEC_INSTANCE(4, 3), /* AAC */
		DEC_INSTANCE(0, 0), /* Reserved */
		DEC_INSTANCE(0, 0), /* MIDI */
		DEC_INSTANCE(4, 3), /* YADPCM */
		DEC_INSTANCE(4, 3), /* QCELP */
		DEC_INSTANCE(4, 3), /* AMRNB */
		DEC_INSTANCE(2, 3), /* AMRWB/WB+ */
		DEC_INSTANCE(4, 3), /* EVRC */
		DEC_INSTANCE(1, 2), /* WMAPRO */
	},
};

static struct msm_adspdec_database msm_device_adspdec_database = {
	.num_dec = ARRAY_SIZE(dec_info_list),
	.num_concurrency_support = (ARRAY_SIZE(dec_concurrency_table) / \
					ARRAY_SIZE(dec_info_list)),
	.dec_concurrency_table = dec_concurrency_table,
	.dec_info_list = dec_info_list,
	.dec_instance_list = &dec_instance_list[0][0],
};

static struct platform_device msm_device_adspdec = {
	.name = "msm_adspdec",
	.id = -1,
	.dev    = {
		.platform_data = &msm_device_adspdec_database
	},
};

static unsigned aux_pcm_gpio_off[] = {
	PCOM_GPIO_CFG(138, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),   /* PCM_DOUT */
	PCOM_GPIO_CFG(139, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),   /* PCM_DIN  */
	PCOM_GPIO_CFG(140, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),   /* PCM_SYNC */
	PCOM_GPIO_CFG(141, 0, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),   /* PCM_CLK  */
};

static void __init aux_pcm_gpio_init(void)
{
	config_gpio_table(aux_pcm_gpio_off,
		ARRAY_SIZE(aux_pcm_gpio_off));
}

#ifdef CONFIG_USB_GADGET_VERIZON_PRODUCT_ID
static int mecha_usb_product_id_match(int product_id, int intrsharing)
{
	int *pid_array = mecha_usb_product_id_match_array;
	int *rndis_array = mecha_usb_product_id_rndis;

	if (!pid_array)
		return product_id;

	/* VZW pid re-match will not apply on MFG mode */
	if (board_mfg_mode() == 1)
		return product_id;

	while (pid_array[0] >= 0) {
		if (product_id == pid_array[0])
			return pid_array[1];
		pid_array += 2;
	}

	if (product_id == 0x0ffe) {
		/* rndis */
		if (intrsharing)
			return rndis_array[0];
		else
			return rndis_array[1];
	}

	return product_id;
}
#endif

#ifdef CONFIG_USB_G_ANDROID
static struct android_usb_platform_data android_usb_pdata = {
	.vendor_id		= 0x0bb4,
	.product_id		= 0x0ca4,
	.version		= 0x0100,
	.product_name		= "Android Phone",
	.manufacturer_name	= "HTC",
	.num_products		= ARRAY_SIZE(usb_products),
	.products		= usb_products,
	.num_functions		= ARRAY_SIZE(usb_functions_all),
	.functions		= usb_functions_all,
#ifdef CONFIG_USB_GADGET_VERIZON_PRODUCT_ID
	.match = mecha_usb_product_id_match,
#endif
};

static struct platform_device android_usb_device = {
	.name	= "android_usb",
	.id		= -1,
	.dev		= {
		.platform_data = &android_usb_pdata,
	},
};
#endif

static struct msm_gpio optnav_config_data[] = {
	{ GPIO_CFG(OPTNAV_CHIP_SELECT, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA),
	"optnav_chip_select" },
};


static void __iomem *virtual_optnav;

static int optnav_gpio_setup(void)
{
	int rc = -ENODEV;
	rc = msm_gpios_request_enable(optnav_config_data,
			ARRAY_SIZE(optnav_config_data));

	/* Configure the FPGA for GPIOs */
	virtual_optnav = ioremap(FPGA_OPTNAV_GPIO_ADDR, 0x4);
	if (!virtual_optnav) {
		pr_err("%s:Could not ioremap region\n", __func__);
		return -ENOMEM;
	}
	/*
	 * Configure the FPGA to set GPIO 19 as
	 * normal, active(enabled), output(MSM to SURF)
	 */
	writew(0x311E, virtual_optnav);
	return rc;
}

static void optnav_gpio_release(void)
{
	msm_gpios_disable_free(optnav_config_data,
		ARRAY_SIZE(optnav_config_data));
	iounmap(virtual_optnav);
}

static struct vreg *vreg_gp7;
static struct vreg *vreg_gp4;
static struct vreg *vreg_gp9;
static struct vreg *vreg_usb3_3;

static int optnav_enable(void)
{
	int rc;
	/*
	 * Enable the VREGs L8(gp7), L10(gp4), L12(gp9), L6(usb)
	 * for I2C communication with keyboard.
	 */
	vreg_gp7 = vreg_get(NULL, "gp7");
	rc = vreg_set_level(vreg_gp7, 1800);
	if (rc) {
		pr_err("%s: vreg_set_level failed \n", __func__);
		goto fail_vreg_gp7;
	}

	rc = vreg_enable(vreg_gp7);
	if (rc) {
		pr_err("%s: vreg_enable failed \n", __func__);
		goto fail_vreg_gp7;
	}

	vreg_gp4 = vreg_get(NULL, "gp4");
	rc = vreg_set_level(vreg_gp4, 2600);
	if (rc) {
		pr_err("%s: vreg_set_level failed \n", __func__);
		goto fail_vreg_gp4;
	}

	rc = vreg_enable(vreg_gp4);
	if (rc) {
		pr_err("%s: vreg_enable failed \n", __func__);
		goto fail_vreg_gp4;
	}

	vreg_gp9 = vreg_get(NULL, "gp9");
	rc = vreg_set_level(vreg_gp9, 1800);
	if (rc) {
		pr_err("%s: vreg_set_level failed \n", __func__);
		goto fail_vreg_gp9;
	}

	rc = vreg_enable(vreg_gp9);
	if (rc) {
		pr_err("%s: vreg_enable failed \n", __func__);
		goto fail_vreg_gp9;
	}

	vreg_usb3_3 = vreg_get(NULL, "usb");
	rc = vreg_set_level(vreg_usb3_3, 3300);
	if (rc) {
		pr_err("%s: vreg_set_level failed \n", __func__);
		goto fail_vreg_3_3;
	}

	rc = vreg_enable(vreg_usb3_3);
	if (rc) {
		pr_err("%s: vreg_enable failed \n", __func__);
		goto fail_vreg_3_3;
	}

	/* Enable the chip select GPIO */
	gpio_set_value(OPTNAV_CHIP_SELECT, 1);
	gpio_set_value(OPTNAV_CHIP_SELECT, 0);

	return 0;

fail_vreg_3_3:
	vreg_disable(vreg_gp9);
fail_vreg_gp9:
	vreg_disable(vreg_gp4);
fail_vreg_gp4:
	vreg_disable(vreg_gp7);
fail_vreg_gp7:
	return rc;
}

static void optnav_disable(void)
{
	vreg_disable(vreg_usb3_3);
	vreg_disable(vreg_gp9);
	vreg_disable(vreg_gp4);
	vreg_disable(vreg_gp7);

	gpio_set_value(OPTNAV_CHIP_SELECT, 1);
}

static struct ofn_atlab_platform_data optnav_data = {
	.gpio_setup    = optnav_gpio_setup,
	.gpio_release  = optnav_gpio_release,
	.optnav_on     = optnav_enable,
	.optnav_off    = optnav_disable,
	.rotate_xy     = 0,
	.function1 = {
		.no_motion1_en		= true,
		.touch_sensor_en	= true,
		.ofn_en			= true,
		.clock_select_khz	= 1500,
		.cpi_selection		= 1200,
	},
	.function2 =  {
		.invert_y		= false,
		.invert_x		= true,
		.swap_x_y		= false,
		.hold_a_b_en		= true,
		.motion_filter_en       = true,
	},
};

#ifdef CONFIG_BOSCH_BMA150
static struct vreg *vreg_gp6;
static int sensors_ldo_enable(void)
{
	int rc;

	/*
	 * Enable the VREGs L8(gp7), L15(gp6)
	 * for I2C communication with sensors.
	 */
	pr_info("sensors_ldo_enable called!!\n");
	vreg_gp7 = vreg_get(NULL, "gp7");
	if (IS_ERR(vreg_gp7)) {
		pr_err("%s: vreg_get gp7 failed\n", __func__);
		rc = PTR_ERR(vreg_gp7);
		goto fail_gp7_get;
	}

	rc = vreg_set_level(vreg_gp7, 1800);
	if (rc) {
		pr_err("%s: vreg_set_level gp7 failed\n", __func__);
		goto fail_gp7_level;
	}

	rc = vreg_enable(vreg_gp7);
	if (rc) {
		pr_err("%s: vreg_enable gp7 failed\n", __func__);
		goto fail_gp7_level;
	}

	vreg_gp6 = vreg_get(NULL, "gp6");
	if (IS_ERR(vreg_gp6)) {
		pr_err("%s: vreg_get gp6 failed\n", __func__);
		rc = PTR_ERR(vreg_gp6);
		goto fail_gp6_get;
	}

	rc = vreg_set_level(vreg_gp6, 3050);
	if (rc) {
		pr_err("%s: vreg_set_level gp6 failed\n", __func__);
		goto fail_gp6_level;
	}

	rc = vreg_enable(vreg_gp6);
	if (rc) {
		pr_err("%s: vreg_enable gp6 failed\n", __func__);
		goto fail_gp6_level;
	}

	return 0;

fail_gp6_level:
	vreg_put(vreg_gp6);
fail_gp6_get:
	vreg_disable(vreg_gp7);
fail_gp7_level:
	vreg_put(vreg_gp7);
fail_gp7_get:
	return rc;
}

static void sensors_ldo_disable(void)
{
	pr_info("sensors_ldo_disable called!!\n");
	vreg_disable(vreg_gp6);
	vreg_put(vreg_gp6);
	vreg_disable(vreg_gp7);
	vreg_put(vreg_gp7);
}
static struct bma150_platform_data bma150_data = {
	.power_on = sensors_ldo_enable,
	.power_off = sensors_ldo_disable,
};

static struct i2c_board_info bma150_board_info[] __initdata = {
	{
		I2C_BOARD_INFO("bma150", 0x38),
		.flags = I2C_CLIENT_WAKE,
		.irq = MSM_GPIO_TO_INT(BMA150_GPIO_INT),
		.platform_data = &bma150_data,
	},
};
#endif

static struct i2c_board_info msm_i2c_board_info[] = {
	{
		I2C_BOARD_INFO("m33c01", OPTNAV_I2C_SLAVE_ADDR),
		.irq		= MSM_GPIO_TO_INT(OPTNAV_IRQ),
		.platform_data = &optnav_data,
	},
};

static struct i2c_board_info msm_marimba_board_info[] = {
	{
		I2C_BOARD_INFO("marimba", 0xc),
		.platform_data = &marimba_pdata,
	}
};

static struct msm_handset_platform_data hs_platform_data = {
	.hs_name = "7k_handset",
	.pwr_key_delay_ms = 500, /* 0 will disable end key */
};

static struct platform_device hs_device = {
	.name   = "msm-handset",
	.id     = -1,
	.dev    = {
		.platform_data = &hs_platform_data,
	},
};

static struct msm_pm_platform_data msm_pm_data[MSM_PM_SLEEP_MODE_NR] = {
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE] = {
		.idle_supported = 1,
		.suspend_supported = 1,
		.idle_enabled = 1,
		.suspend_enabled = 1,
		.latency = 8594,
		.residency = 23740,
	},
	[MSM_PM_SLEEP_MODE_APPS_SLEEP] = {
		.idle_supported = 1,
		.suspend_supported = 1,
		.idle_enabled = 1,
		.suspend_enabled = 1,
		.latency = 8594,
		.residency = 23740,
	},
/*
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN] = {
		.idle_supported = 1,
		.suspend_supported = 1,
		.idle_enabled = 1,
		.suspend_enabled = 1,
		.latency = 4594,
		.residency = 23740,
	},
*/
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_STANDALONE] = {
#ifdef CONFIG_MSM_STANDALONE_POWER_COLLAPSE
		.idle_supported = 1,
		.suspend_supported = 1,
		.idle_enabled = 1,
		.suspend_enabled = 0,
#else /*CONFIG_MSM_STANDALONE_POWER_COLLAPSE*/
		.idle_supported = 0,
		.suspend_supported = 0,
		.idle_enabled = 0,
		.suspend_enabled = 0,
#endif /*CONFIG_MSM_STANDALONE_POWER_COLLAPSE*/
		.latency = 500,
		.residency = 6000,
	},
	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT] = {
		.idle_supported = 1,
		.suspend_supported = 1,
		.idle_enabled = 0,
		.suspend_enabled = 1,
		.latency = 443,
		.residency = 1098,
	},
	[MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT] = {
		.idle_supported = 1,
		.suspend_supported = 1,
		.idle_enabled = 1,
		.suspend_enabled = 1,
		.latency = 2,
		.residency = 0,
	},
};

#ifdef CONFIG_SPI_QSD
static struct spi_board_info msm_spi_board_info[] __initdata = {
#ifdef CONFIG_SPI_DISPLAY
	{
		.modalias	= "spi_display",
		.mode		= SPI_MODE_3,
		.bus_num	= 0,
		.chip_select	= 2,
		.max_speed_hz	= 10000000,
	},
#else
	{
		.modalias	= "spi_qsd",
		.mode		= SPI_MODE_3,
		.bus_num	= 0,
		.chip_select	= 2,
		.max_speed_hz	= 10000000,
	},
#endif
	{
		.modalias	= "spi_aic3254",
		.mode           = SPI_MODE_1,
		.bus_num        = 0,
		.chip_select    = 3,
		.max_speed_hz   = 9963243,
	}
};
#endif


static uint32_t qsd_spi_gpio_on_table[] = {
	PCOM_GPIO_CFG(45, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_6MA),
	PCOM_GPIO_CFG(47, 1, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_6MA),
	PCOM_GPIO_CFG(48, 1, GPIO_INPUT, GPIO_NO_PULL, GPIO_6MA),
	PCOM_GPIO_CFG(87, 2, GPIO_INPUT, GPIO_NO_PULL, GPIO_16MA),
	PCOM_GPIO_CFG(89, 2, GPIO_OUTPUT, GPIO_PULL_UP, GPIO_6MA)
};

static uint32_t qsd_spi_gpio_off_table[] = {
	PCOM_GPIO_CFG(45, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_16MA),
	PCOM_GPIO_CFG(47, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_16MA),
	PCOM_GPIO_CFG(48, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_16MA),
	PCOM_GPIO_CFG(87, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_16MA),
	PCOM_GPIO_CFG(89, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_16MA)
};

static int msm_qsd_spi_gpio_config(void)
{
	config_gpio_table(qsd_spi_gpio_on_table,
		ARRAY_SIZE(qsd_spi_gpio_on_table));
	return 0;
}

static void msm_qsd_spi_gpio_release(void)
{
	config_gpio_table(qsd_spi_gpio_off_table,
		ARRAY_SIZE(qsd_spi_gpio_off_table));
}

static struct msm_spi_platform_data qsd_spi_pdata = {
	.max_clock_speed = 26000000,
	.gpio_config  = msm_qsd_spi_gpio_config,
	.gpio_release = msm_qsd_spi_gpio_release,
};

static void __init msm_qsd_spi_init(void)
{
	qsd_device_spi.dev.platform_data = &qsd_spi_pdata;
}

#ifdef CONFIG_USB_EHCI_MSM_72K
static void msm_hsusb_vbus_power(unsigned phy_info, int on)
{
	int rc;
	static int vbus_is_on;
	struct pm8xxx_gpio_init_info usb_vbus = {
		PM8058_GPIO_PM_TO_SYS(36),
		{
			.direction      = PM_GPIO_DIR_OUT,
			.pull           = PM_GPIO_PULL_NO,
			.output_buffer  = PM_GPIO_OUT_BUF_CMOS,
			.output_value   = 1,
			.vin_sel        = 2,
			.out_strength   = PM_GPIO_STRENGTH_MED,
			.function       = PM_GPIO_FUNC_NORMAL,
			.inv_int_pol    = 0,
		},
	};

	/* If VBUS is already on (or off), do nothing. */
	if (unlikely(on == vbus_is_on))
		return;

	if (on) {
		rc = pm8xxx_gpio_config(usb_vbus.gpio, &usb_vbus.config);
		if (rc) {
			pr_err("%s PMIC GPIO 36 write failed\n", __func__);
			return;
		}
	} else {
		gpio_set_value_cansleep(PM8058_GPIO_PM_TO_SYS(36), 0);
	}

	vbus_is_on = on;
}

static struct msm_usb_host_platform_data msm_usb_host_pdata = {
		.phy_info   = (USB_PHY_INTEGRATED | USB_PHY_MODEL_45NM),
		.vbus_power = msm_hsusb_vbus_power,
		.power_budget   = 180,
};
#endif

#ifdef CONFIG_USB_MSM_OTG_72K
static int hsusb_rpc_connect(int connect)
{
	if (connect)
		return msm_hsusb_rpc_connect();
	else
		return msm_hsusb_rpc_close();
}
#endif

#ifdef CONFIG_USB_MSM_OTG_72K
static struct vreg *vreg_3p3;
static int msm_hsusb_ldo_init(int init)
{
	uint32_t version = 0;
	int def_vol = 3400;

	version = socinfo_get_version();

	if (SOCINFO_VERSION_MAJOR(version) >= 2 &&
			SOCINFO_VERSION_MINOR(version) >= 1) {
		def_vol = 3075;
		pr_debug("%s: default voltage:%d\n", __func__, def_vol);
	}

	if (init) {
		vreg_3p3 = vreg_get(NULL, "usb");
		if (IS_ERR(vreg_3p3))
			return PTR_ERR(vreg_3p3);
		vreg_set_level(vreg_3p3, def_vol);
	} else
		vreg_put(vreg_3p3);

	return 0;
}

static int msm_hsusb_ldo_enable(int enable)
{
	static int ldo_status;

	if (!vreg_3p3 || IS_ERR(vreg_3p3))
		return -ENODEV;

	if (ldo_status == enable)
		return 0;

	ldo_status = enable;

	if (enable)
		return vreg_enable(vreg_3p3);

	return vreg_disable(vreg_3p3);
}

static int msm_hsusb_ldo_set_voltage(int mV)
{
	static int cur_voltage = 3400;

	if (!vreg_3p3 || IS_ERR(vreg_3p3))
		return -ENODEV;

	if (cur_voltage == mV)
		return 0;

	cur_voltage = mV;

	pr_debug("%s: (%d)\n", __func__, mV);

	return vreg_set_level(vreg_3p3, mV);
}
#endif

static int mecha_phy_init_seq[] = { 0x06, 0x36, 0x0C, 0x31, 0x31, 0x32, 0x1, 0x0D, 0x1, 0x10, -1 };
static struct msm_otg_platform_data msm_otg_pdata = {
	.phy_init_seq		= mecha_phy_init_seq,
	.mode			= USB_PERIPHERAL,
	.otg_control		= OTG_PMIC_CONTROL,
	.power_budget		= 750,
	.phy_type		= CI_45NM_INTEGRATED_PHY,
};

static struct android_pmem_platform_data android_pmem_pdata = {
	.name = "pmem",
	.allocator_type = PMEM_ALLOCATORTYPE_ALLORNOTHING,
	.cached = 1,
	.memory_type = MEMTYPE_EBI1,
};

static struct platform_device android_pmem_device = {
	.name = "android_pmem",
	.id = 0,
	.dev = { .platform_data = &android_pmem_pdata },
};

#if defined(CONFIG_FB_MSM_HDMI_ADV7520_PANEL) || defined(CONFIG_BOSCH_BMA150)
/* there is an i2c address conflict between adv7520 and bma150 sensor after
 * power up on fluid. As a solution, the default address of adv7520's packet
 * memory is changed as soon as possible
 */
static int __init fluid_i2c_address_fixup(void)
{
	unsigned char wBuff[16];
	unsigned char rBuff[16];
	struct i2c_msg msgs[3];
	int res;
	int rc = -EINVAL;
	struct vreg *vreg_ldo8;
	struct i2c_adapter *adapter;

	if (machine_is_msm7x30_fluid()) {
		adapter = i2c_get_adapter(0);
		if (!adapter) {
			pr_err("%s: invalid i2c adapter\n", __func__);
			return PTR_ERR(adapter);
		}

		/* turn on LDO8 */
		vreg_ldo8 = vreg_get(NULL, "gp7");
		if (!vreg_ldo8) {
			pr_err("%s: VREG L8 get failed\n", __func__);
			goto adapter_put;
		}

		rc = vreg_set_level(vreg_ldo8, 1800);
		if (rc) {
			pr_err("%s: VREG L8 set failed\n", __func__);
			goto ldo8_put;
		}

		rc = vreg_enable(vreg_ldo8);
		if (rc) {
			pr_err("%s: VREG L8 enable failed\n", __func__);
			goto ldo8_put;
		}

		/* change packet memory address to 0x74 */
		wBuff[0] = 0x45;
		wBuff[1] = 0x74;

		msgs[0].addr = ADV7520_I2C_ADDR;
		msgs[0].flags = 0;
		msgs[0].buf = (unsigned char *) wBuff;
		msgs[0].len = 2;

		res = i2c_transfer(adapter, msgs, 1);
		if (res != 1) {
			pr_err("%s: error writing adv7520\n", __func__);
			goto ldo8_disable;
		}

		/* powerdown adv7520 using bit 6 */
		/* i2c read first */
		wBuff[0] = 0x41;

		msgs[0].addr = ADV7520_I2C_ADDR;
		msgs[0].flags = 0;
		msgs[0].buf = (unsigned char *) wBuff;
		msgs[0].len = 1;

		msgs[1].addr = ADV7520_I2C_ADDR;
		msgs[1].flags = I2C_M_RD;
		msgs[1].buf = rBuff;
		msgs[1].len = 1;
		res = i2c_transfer(adapter, msgs, 2);
		if (res != 2) {
			pr_err("%s: error reading adv7520\n", __func__);
			goto ldo8_disable;
		}

		/* i2c write back */
		wBuff[0] = 0x41;
		wBuff[1] = rBuff[0] | 0x40;

		msgs[0].addr = ADV7520_I2C_ADDR;
		msgs[0].flags = 0;
		msgs[0].buf = (unsigned char *) wBuff;
		msgs[0].len = 2;

		res = i2c_transfer(adapter, msgs, 1);
		if (res != 1) {
			pr_err("%s: error writing adv7520\n", __func__);
			goto ldo8_disable;
		}

		/* for successful fixup, we release the i2c adapter */
		/* but leave ldo8 on so that the adv7520 is not repowered */
		i2c_put_adapter(adapter);
		pr_info("%s: fluid i2c address conflict resolved\n", __func__);
	}
	return 0;

ldo8_disable:
	vreg_disable(vreg_ldo8);
ldo8_put:
	vreg_put(vreg_ldo8);
adapter_put:
	i2c_put_adapter(adapter);
	return rc;
}
fs_initcall_sync(fluid_i2c_address_fixup);
#endif

static struct resource msm_fb_resources[] = {
	{
		.flags  = IORESOURCE_DMA,
	}
};

static struct platform_device msm_migrate_pages_device = {
	.name   = "msm_migrate_pages",
	.id     = -1,
};

static struct android_pmem_platform_data android_pmem_adsp_pdata = {
       .name = "pmem_adsp",
       .allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
       .cached = 1,
	.memory_type = MEMTYPE_EBI1,
};

static struct android_pmem_platform_data android_pmem_audio_pdata = {
       .name = "pmem_audio",
       .allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
       .cached = 0,
	.memory_type = MEMTYPE_EBI1,
};

static struct platform_device android_pmem_adsp_device = {
       .name = "android_pmem",
       .id = 2,
       .dev = { .platform_data = &android_pmem_adsp_pdata },
};

static struct platform_device android_pmem_audio_device = {
       .name = "android_pmem",
       .id = 4,
       .dev = { .platform_data = &android_pmem_audio_pdata },
};
#if defined(CONFIG_CRYPTO_DEV_QCRYPTO) || \
		defined(CONFIG_CRYPTO_DEV_QCRYPTO_MODULE) || \
		defined(CONFIG_CRYPTO_DEV_QCEDEV) || \
		defined(CONFIG_CRYPTO_DEV_QCEDEV_MODULE)

#define QCE_SIZE		0x10000
#define QCE_0_BASE		0xA8400000

#define QCE_HW_KEY_SUPPORT	1
#define QCE_SHA_HMAC_SUPPORT	0
#define QCE_SHARE_CE_RESOURCE	0
#define QCE_CE_SHARED		0

static struct resource qcrypto_resources[] = {
	[0] = {
		.start = QCE_0_BASE,
		.end = QCE_0_BASE + QCE_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.name = "crypto_channels",
		.start = DMOV_CE_IN_CHAN,
		.end = DMOV_CE_OUT_CHAN,
		.flags = IORESOURCE_DMA,
	},
	[2] = {
		.name = "crypto_crci_in",
		.start = DMOV_CE_IN_CRCI,
		.end = DMOV_CE_IN_CRCI,
		.flags = IORESOURCE_DMA,
	},
	[3] = {
		.name = "crypto_crci_out",
		.start = DMOV_CE_OUT_CRCI,
		.end = DMOV_CE_OUT_CRCI,
		.flags = IORESOURCE_DMA,
	},
	[4] = {
		.name = "crypto_crci_hash",
		.start = DMOV_CE_HASH_CRCI,
		.end = DMOV_CE_HASH_CRCI,
		.flags = IORESOURCE_DMA,
	},
};

static struct resource qcedev_resources[] = {
	[0] = {
		.start = QCE_0_BASE,
		.end = QCE_0_BASE + QCE_SIZE - 1,
		.flags = IORESOURCE_MEM,
	},
	[1] = {
		.name = "crypto_channels",
		.start = DMOV_CE_IN_CHAN,
		.end = DMOV_CE_OUT_CHAN,
		.flags = IORESOURCE_DMA,
	},
	[2] = {
		.name = "crypto_crci_in",
		.start = DMOV_CE_IN_CRCI,
		.end = DMOV_CE_IN_CRCI,
		.flags = IORESOURCE_DMA,
	},
	[3] = {
		.name = "crypto_crci_out",
		.start = DMOV_CE_OUT_CRCI,
		.end = DMOV_CE_OUT_CRCI,
		.flags = IORESOURCE_DMA,
	},
	[4] = {
		.name = "crypto_crci_hash",
		.start = DMOV_CE_HASH_CRCI,
		.end = DMOV_CE_HASH_CRCI,
		.flags = IORESOURCE_DMA,
	},
};

#endif

#if defined(CONFIG_CRYPTO_DEV_QCRYPTO) || \
		defined(CONFIG_CRYPTO_DEV_QCRYPTO_MODULE)

static struct msm_ce_hw_support qcrypto_ce_hw_suppport = {
	.ce_shared = QCE_CE_SHARED,
	.shared_ce_resource = QCE_SHARE_CE_RESOURCE,
	.hw_key_support = QCE_HW_KEY_SUPPORT,
	.sha_hmac = QCE_SHA_HMAC_SUPPORT,
};

static struct platform_device qcrypto_device = {
	.name		= "qcrypto",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(qcrypto_resources),
	.resource	= qcrypto_resources,
	.dev		= {
		.coherent_dma_mask = DMA_BIT_MASK(32),
		.platform_data = &qcrypto_ce_hw_suppport,
	},
};
#endif

#if defined(CONFIG_CRYPTO_DEV_QCEDEV) || \
		defined(CONFIG_CRYPTO_DEV_QCEDEV_MODULE)

static struct msm_ce_hw_support qcedev_ce_hw_suppport = {
	.ce_shared = QCE_CE_SHARED,
	.shared_ce_resource = QCE_SHARE_CE_RESOURCE,
	.hw_key_support = QCE_HW_KEY_SUPPORT,
	.sha_hmac = QCE_SHA_HMAC_SUPPORT,
};
static struct platform_device qcedev_device = {
	.name		= "qce",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(qcedev_resources),
	.resource	= qcedev_resources,
	.dev		= {
		.coherent_dma_mask = DMA_BIT_MASK(32),
		.platform_data = &qcedev_ce_hw_suppport,
	},
};

#endif
static struct htc_battery_platform_data htc_battery_pdev_data = {
	.guage_driver = GUAGE_MODEM,
	.charger = SWITCH_CHARGER_TPS65200,
	.m2a_cable_detect = 1,
	.gpio_mchg_en_n = MSM_GPIO_TO_INT(PM8058_GPIO_PM_TO_SYS(MECHA_GPIO_CHG_INT)),
};

static struct platform_device htc_battery_pdev = {
	.name = "htc_battery",
	.id   = -1,
	.dev  = {
		.platform_data = &htc_battery_pdev_data,
	},
};

static struct msm_adc_platform_data msm_adc_pdata;

static struct platform_device msm_adc_device = {
	.name   = "msm_adc",
	.id = -1,
	.dev = {
		.platform_data = &msm_adc_pdata,
	},
};

#ifdef CONFIG_SERIAL_MSM_HS
static struct msm_serial_hs_platform_data msm_uart_dm1_pdata = {
	.inject_rx_on_wakeup = 0,
	.cpu_lock_supported = 1,

	/* for bcm BT */
	.bt_wakeup_pin_supported = 1,
	.bt_wakeup_pin = MECHA_GPIO_BT_CHIP_WAKE,
	.host_wakeup_pin = MECHA_GPIO_BT_HOST_WAKE,
};
#endif

#ifdef CONFIG_MSM_VPE
static struct resource msm_vpe_resources[] = {
	{
		.start	= 0xAD200000,
		.end	= 0xAD200000 + SZ_1M - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.start	= INT_VPE,
		.end	= INT_VPE,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device msm_vpe_device = {
       .name = "msm_vpe",
       .id   = 0,
       .num_resources = ARRAY_SIZE(msm_vpe_resources),
       .resource = msm_vpe_resources,
};
#endif
#ifdef CONFIG_BT
static struct platform_device mecha_rfkill = {
	.name = "mecha_rfkill",
	.id = -1,
};
#endif

#ifdef CONFIG_MSM_SDIO_AL
static struct msm_gpio mdm2ap_status = {
	GPIO_CFG(77, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	"mdm2ap_status"
};


static int configure_mdm2ap_status(int on)
{
	if (on)
		return msm_gpios_request_enable(&mdm2ap_status, 1);
	else {
		msm_gpios_disable_free(&mdm2ap_status, 1);
		return 0;
	}
}

static int get_mdm2ap_status(void)
{
	return gpio_get_value(GPIO_PIN(mdm2ap_status.gpio_cfg));
}

static struct sdio_al_platform_data sdio_al_pdata = {
	.config_mdm2ap_status = configure_mdm2ap_status,
	.get_mdm2ap_status = get_mdm2ap_status,
	.allow_sdioc_version_major_2 = 1,
	.peer_sdioc_version_minor = 0x0001,
	.peer_sdioc_version_major = 0x0003,
	.peer_sdioc_boot_version_minor = 0x0001,
	.peer_sdioc_boot_version_major = 0x0003,
};

struct platform_device msm_device_sdio_al = {
	.name = "msm_sdio_al",
	.id = -1,
	.dev		= {
		.platform_data	= &sdio_al_pdata,
	},
};
#endif /* CONFIG_MSM_SDIO_AL */

static struct resource ram_console_resources[] = {
	{
		.start  = MSM_RAM_CONSOLE_BASE,
		.end    = MSM_RAM_CONSOLE_BASE + MSM_RAM_CONSOLE_SIZE - 1,
		.flags  = IORESOURCE_MEM,
	},
};

static struct platform_device ram_console_device = {
	.name           = "ram_console",
	.id             = -1,
	.num_resources  = ARRAY_SIZE(ram_console_resources),
	.resource       = ram_console_resources,
};

#ifdef CONFIG_FLASHLIGHT_AAT1271
static void config_mecha_flashlight_gpios(void)
{
	uint32_t flashlight_gpio_table[] = {
		PCOM_GPIO_CFG(MECHA_GPIO_TORCH_EN, 0, GPIO_OUTPUT,
						GPIO_NO_PULL, GPIO_2MA),
		PCOM_GPIO_CFG(MECHA_GPIO_FLASH_EN, 0, GPIO_OUTPUT,
						GPIO_NO_PULL, GPIO_2MA),
	};
	config_gpio_table(flashlight_gpio_table,
		ARRAY_SIZE(flashlight_gpio_table));
}

static struct flashlight_platform_data mecha_flashlight_data = {
	.gpio_init = config_mecha_flashlight_gpios,
	.torch = MECHA_GPIO_TORCH_EN,
	.flash = MECHA_GPIO_FLASH_EN,
	.flash_duration_ms = 600,
	.led_count = 1,
};

static struct platform_device mecha_flashlight_device = {
	.name = "FLASHLIGHT_AAT1271",
	.dev = {
		.platform_data	= &mecha_flashlight_data,
	},
};
#endif

#define PM8058ADC_16BIT(adc) ((adc * 2200) / 65535) /* vref=2.2v, 16-bits resolution */

int64_t mecha_get_usbid_adc(void)
{
	uint32_t adc_value = 0xffffffff;
	htc_get_usb_accessory_adc_level(&adc_value);
	adc_value = PM8058ADC_16BIT(adc_value);
	return adc_value;
}

static uint32_t usb_ID_PIN_input_table[] = {
	PCOM_GPIO_CFG(MECHA_GPIO_USB_ID_PIN, 0, GPIO_INPUT, GPIO_NO_PULL, GPIO_2MA),
};

static uint32_t usb_ID_PIN_ouput_table[] = {
	PCOM_GPIO_CFG(MECHA_GPIO_USB_ID_PIN, 0, GPIO_OUTPUT, GPIO_NO_PULL, GPIO_2MA),
};

void config_mecha_usb_id_gpios(bool output)
{
	if (output) {
		config_gpio_table(usb_ID_PIN_ouput_table, ARRAY_SIZE(usb_ID_PIN_ouput_table));
		gpio_set_value(MECHA_GPIO_USB_ID_PIN, 1);
		printk(KERN_INFO "%s %d output high\n",  __func__, MECHA_GPIO_USB_ID_PIN);
	} else {
		config_gpio_table(usb_ID_PIN_input_table, ARRAY_SIZE(usb_ID_PIN_input_table));
		printk(KERN_INFO "%s %d input none pull\n",  __func__, MECHA_GPIO_USB_ID_PIN);
	}
}

static struct cable_detect_platform_data cable_detect_pdata = {
	.detect_type 		= CABLE_TYPE_PMIC_ADC,
	.usb_id_pin_gpio 		= MECHA_GPIO_USB_ID_PIN,
	.config_usb_id_gpios 	= config_mecha_usb_id_gpios,
	.usb_hub_enable			= mecha_usb_hub_enable,
	.get_adc_cb				= mecha_get_usbid_adc,
};

static struct platform_device cable_detect_device = {
	.name	= "cable_detect",
	.id	= -1,
	.dev	= {
		.platform_data = &cable_detect_pdata,
	},
};

struct platform_device htc_drm = {
	.name = "htcdrm",
	.id = 0,
};

static struct platform_device *device_pm8058_led[] __initdata = {
	&pm8058_leds,
};

static struct platform_device *device_pm8058_led_XC[] __initdata = {
	&pm8058_leds_XC,
};

static struct platform_device *devices_CM3602_Proximity[] __initdata = {
#ifdef CONFIG_INPUT_CAPELLA_CM3602
	&capella_cm3602,
#endif
};


static struct platform_device *devices_Lightsensor[] __initdata = {
#ifdef CONFIG_INPUT_CAPELLA_CM3602
	&lightsensor_pdev,
#endif
};

static struct msm_gpio msm_i2c_gpios_hw[] = {
	{ GPIO_CFG(70, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "i2c_scl" },
	{ GPIO_CFG(71, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "i2c_sda" },
};

static struct msm_gpio msm_i2c_gpios_io[] = {
	{ GPIO_CFG(70, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "i2c_scl" },
	{ GPIO_CFG(71, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "i2c_sda" },
};

static struct msm_gpio qup_i2c_gpios_io[] = {
	{ GPIO_CFG(16, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "qup_scl" },
	{ GPIO_CFG(17, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "qup_sda" },
};
static struct msm_gpio qup_i2c_gpios_hw[] = {
	{ GPIO_CFG(16, 2, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "qup_scl" },
	{ GPIO_CFG(17, 2, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "qup_sda" },
};

static void
msm_i2c_gpio_config(int adap_id, int config_type)
{
	struct msm_gpio *msm_i2c_table;

	/* Each adapter gets 2 lines from the table */
	if (adap_id > 0)
		return;
	if (config_type)
		msm_i2c_table = &msm_i2c_gpios_hw[adap_id*2];
	else
		msm_i2c_table = &msm_i2c_gpios_io[adap_id*2];
	msm_gpios_enable(msm_i2c_table, 2);
}
/*This needs to be enabled only for OEMS*/
#ifndef CONFIG_QUP_EXCLUSIVE_TO_CAMERA
static struct vreg *qup_vreg;
#endif
static void
qup_i2c_gpio_config(int adap_id, int config_type)
{
	int rc = 0;
	struct msm_gpio *qup_i2c_table;
	/* Each adapter gets 2 lines from the table */
	if (adap_id != 4)
		return;
	if (config_type)
		qup_i2c_table = qup_i2c_gpios_hw;
	else
		qup_i2c_table = qup_i2c_gpios_io;
	rc = msm_gpios_enable(qup_i2c_table, 2);
	if (rc < 0)
		printk(KERN_ERR "QUP GPIO enable failed: %d\n", rc);
	/*This needs to be enabled only for OEMS*/
#ifndef CONFIG_QUP_EXCLUSIVE_TO_CAMERA
	if (qup_vreg) {
		int rc = vreg_set_level(qup_vreg, 1800);
		if (rc) {
			pr_err("%s: vreg LVS1 set level failed (%d)\n",
			__func__, rc);
		}
		rc = vreg_enable(qup_vreg);
		if (rc) {
			pr_err("%s: vreg_enable() = %d \n",
			__func__, rc);
		}
	}
#endif
}

static struct msm_i2c_platform_data msm_i2c_pdata = {
	.clk_freq = 400000,
	.pri_clk = 70,
	.pri_dat = 71,
	.rmutex  = 1,
	.rsl_id = "D:I2C02000021",
	.msm_i2c_config_gpio = msm_i2c_gpio_config,
};

static void __init msm_device_i2c_init(void)
{
	if (msm_gpios_request(msm_i2c_gpios_hw, ARRAY_SIZE(msm_i2c_gpios_hw)))
		pr_err("failed to request I2C gpios\n");

	msm_device_i2c.dev.platform_data = &msm_i2c_pdata;
}

static struct msm_i2c_platform_data msm_i2c_2_pdata = {
	.clk_freq = 100000,
	.rmutex  = 1,
	.rsl_id = "D:I2C02000022",
	.msm_i2c_config_gpio = msm_i2c_gpio_config,
};

static void __init msm_device_i2c_2_init(void)
{
	msm_device_i2c_2.dev.platform_data = &msm_i2c_2_pdata;
}

static struct msm_i2c_platform_data qup_i2c_pdata = {
	.clk_freq = 384000,
	.msm_i2c_config_gpio = qup_i2c_gpio_config,
};

static void __init qup_device_i2c_init(void)
{
	/* Remove the gpio_request due to i2c-qup.c is done so. */
	/*if (msm_gpios_request(qup_i2c_gpios_hw, ARRAY_SIZE(qup_i2c_gpios_hw)))
		pr_err("failed to request I2C gpios\n");*/

	qup_device_i2c.dev.platform_data = &qup_i2c_pdata;
	/*This needs to be enabled only for OEMS*/
#ifndef CONFIG_QUP_EXCLUSIVE_TO_CAMERA
	qup_vreg = vreg_get(NULL, "lvsw1");
	if (IS_ERR(qup_vreg)) {
		printk(KERN_ERR "%s: vreg get failed (%ld)\n",
			__func__, PTR_ERR(qup_vreg));
	}
#endif
}

#ifdef CONFIG_I2C_SSBI
/*
static struct msm_i2c_ssbi_platform_data msm_i2c_ssbi6_pdata = {
	.rsl_id = "D:PMIC_SSBI",
	.controller_type = MSM_SBI_CTRL_SSBI2,
};*/

static struct msm_i2c_ssbi_platform_data msm_i2c_ssbi7_pdata = {
	.rsl_id = "D:CODEC_SSBI",
	.controller_type = MSM_SBI_CTRL_SSBI,
};
#endif

static void __init mecha_init_irq(void)
{
	msm_init_irq();
}
#if 0
static struct msm_gpio msm_nand_ebi2_cfg_data[] = {
	{GPIO_CFG(86, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "ebi2_cs1"},
	{GPIO_CFG(115, 2, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "ebi2_busy1"},
};
#endif
struct vreg *vreg_s3;
struct vreg *vreg_mmc;
struct vreg *vreg_sd;

#if (defined(CONFIG_MMC_MSM_SDC1_SUPPORT)\
	|| defined(CONFIG_MMC_MSM_SDC2_SUPPORT)\
	|| defined(CONFIG_MMC_MSM_SDC3_SUPPORT)\
	|| defined(CONFIG_MMC_MSM_SDC4_SUPPORT))

struct sdcc_gpio {
	struct msm_gpio *cfg_data;
	uint32_t size;
	struct msm_gpio *sleep_cfg_data;
};
#if defined(CONFIG_MMC_MSM_SDC1_SUPPORT)
static struct msm_gpio sdc1_lvlshft_cfg_data[] = {
	{GPIO_CFG(35, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_16MA), "sdc1_lvlshft"},
};
#endif

static struct msm_gpio sdc1_cfg_data[] = {
	{GPIO_CFG(38, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "sdc1_clk"},
	{GPIO_CFG(39, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc1_cmd"},
	{GPIO_CFG(40, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc1_dat_3"},
	{GPIO_CFG(41, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc1_dat_2"},
	{GPIO_CFG(42, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc1_dat_1"},
	{GPIO_CFG(43, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc1_dat_0"},
};

static struct msm_gpio sdc2_cfg_data[] = {
	{GPIO_CFG(64, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "sdc2_clk"},
	{GPIO_CFG(65, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc2_cmd"},
	{GPIO_CFG(66, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc2_dat_3"},
	{GPIO_CFG(67, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc2_dat_2"},
	{GPIO_CFG(68, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc2_dat_1"},
	{GPIO_CFG(69, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc2_dat_0"},

#ifdef CONFIG_MMC_MSM_SDC2_8_BIT_SUPPORT
	{GPIO_CFG(115, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc2_dat_4"},
	{GPIO_CFG(114, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc2_dat_5"},
	{GPIO_CFG(113, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc2_dat_6"},
	{GPIO_CFG(112, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc2_dat_7"},
#endif
};

static struct msm_gpio sdc3_cfg_data[] = {
	{GPIO_CFG(110, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "sdc3_clk"},
	{GPIO_CFG(111, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc3_cmd"},
	{GPIO_CFG(116, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc3_dat_3"},
	{GPIO_CFG(117, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc3_dat_2"},
	{GPIO_CFG(118, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc3_dat_1"},
	{GPIO_CFG(119, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc3_dat_0"},
};

static struct msm_gpio sdc3_sleep_cfg_data[] = {
	{GPIO_CFG(110, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
			"sdc3_clk"},
	{GPIO_CFG(111, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
			"sdc3_cmd"},
	{GPIO_CFG(116, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
			"sdc3_dat_3"},
	{GPIO_CFG(117, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
			"sdc3_dat_2"},
	{GPIO_CFG(118, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
			"sdc3_dat_1"},
	{GPIO_CFG(119, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
			"sdc3_dat_0"},
};

static struct msm_gpio sdc4_cfg_data[] = {
	{GPIO_CFG(58, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA) ,  "sdc4_clk"},
	{GPIO_CFG(59, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA) , "sdc4_cmd"},
	{GPIO_CFG(60, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA) , "sdc4_dat_3"},
	{GPIO_CFG(61, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA) , "sdc4_dat_2"},
	{GPIO_CFG(62, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA) , "sdc4_dat_1"},
	{GPIO_CFG(63, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA) , "sdc4_dat_0"},
};

static struct msm_gpio sdc4_sleep_cfg_data[] = {
	{GPIO_CFG(58, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA) , "sdc4_clk"},
	{GPIO_CFG(59, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_4MA) , "sdc4_cmd"},
	{GPIO_CFG(60, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_4MA) , "sdc4_dat_3"},
	{GPIO_CFG(61, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_4MA) , "sdc4_dat_2"},
	{GPIO_CFG(62, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_4MA) , "sdc4_dat_1"},
	{GPIO_CFG(63, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_4MA) , "sdc4_dat_0"},
};

static struct sdcc_gpio sdcc_cfg_data[] = {
	{
		.cfg_data = sdc1_cfg_data,
		.size = ARRAY_SIZE(sdc1_cfg_data),
		.sleep_cfg_data = NULL,
	},
	{
		.cfg_data = sdc2_cfg_data,
		.size = ARRAY_SIZE(sdc2_cfg_data),
		.sleep_cfg_data = NULL,
	},
	{
		.cfg_data = sdc3_cfg_data,
		.size = ARRAY_SIZE(sdc3_cfg_data),
		.sleep_cfg_data = sdc3_sleep_cfg_data,
	},
	{
		.cfg_data = sdc4_cfg_data,
		.size = ARRAY_SIZE(sdc4_cfg_data),
		.sleep_cfg_data = sdc4_sleep_cfg_data,
	},
};


struct sdcc_vreg {
	struct vreg *vreg_data;
	unsigned level;
};

static struct sdcc_vreg sdcc_vreg_data[4];


static unsigned long vreg_sts, gpio_sts;

static uint32_t msm_sdcc_setup_gpio(int dev_id, unsigned int enable)
{
	int rc = 0;
	struct sdcc_gpio *curr;

	curr = &sdcc_cfg_data[dev_id - 1];

	if (!(test_bit(dev_id, &gpio_sts)^enable))
		return rc;

	if (enable) {
		set_bit(dev_id, &gpio_sts);
		rc = msm_gpios_request_enable(curr->cfg_data, curr->size);
		if (rc)
			printk(KERN_ERR "%s: Failed to turn on GPIOs for slot %d\n",
				__func__,  dev_id);
	} else {
		clear_bit(dev_id, &gpio_sts);
		if (curr->sleep_cfg_data) {
			msm_gpios_enable(curr->sleep_cfg_data, curr->size);
			msm_gpios_free(curr->sleep_cfg_data, curr->size);
		} else {
			msm_gpios_disable_free(curr->cfg_data, curr->size);
		}
	}

	return rc;
}

static uint32_t msm_sdcc_setup_vreg(int dev_id, unsigned int enable)
{
	int rc = 0;
	struct sdcc_vreg *curr;
	static int enabled_once[] = {0, 0, 0, 0};

	curr = &sdcc_vreg_data[dev_id - 1];

	if (!(test_bit(dev_id, &vreg_sts)^enable))
		return rc;

	if (dev_id != 4) {
		if (!enable || enabled_once[dev_id - 1])
			return 0;
	}

	if (enable) {
		set_bit(dev_id, &vreg_sts);
		rc = vreg_set_level(curr->vreg_data, curr->level);
		if (rc) {
			printk(KERN_ERR "%s: vreg_set_level() = %d \n",
					__func__, rc);
		}
		if (dev_id == 4){
				printk(KERN_INFO "%s: Enabling SD slot power\n", __func__);
				mdelay(5);
		}
		rc = vreg_enable(curr->vreg_data);
		if (rc) {
			printk(KERN_ERR "%s: vreg_enable() = %d \n",
					__func__, rc);
		}
		enabled_once[dev_id - 1] = 1;
	} else {
		clear_bit(dev_id, &vreg_sts);
		if (dev_id == 4){
				printk(KERN_INFO "%s: Disabling SD slot power\n", __func__);
				mdelay(5);
		}
		rc = vreg_disable(curr->vreg_data);
		if (rc) {
			printk(KERN_ERR "%s: vreg_disable() = %d \n",
					__func__, rc);
		}
	}
	return rc;
}

static uint32_t msm_sdcc_setup_power(struct device *dv, unsigned int vdd)
{
	int rc = 0;
	struct platform_device *pdev;

	pdev = container_of(dv, struct platform_device, dev);
	rc = msm_sdcc_setup_gpio(pdev->id, (vdd ? 1 : 0));
	if (rc)
		goto out;

	if (pdev->id == 4) /* S3 is always ON and cannot be disabled */
		rc = msm_sdcc_setup_vreg(pdev->id, (vdd ? 1 : 0));
out:
	return rc;
}


#if defined(CONFIG_MMC_MSM_SDC1_SUPPORT) && \
	defined(CONFIG_CSDIO_VENDOR_ID) && \
	defined(CONFIG_CSDIO_DEVICE_ID) && \
	(CONFIG_CSDIO_VENDOR_ID == 0x70 && CONFIG_CSDIO_DEVICE_ID == 0x1117)

#define MBP_ON  1
#define MBP_OFF 0

#define MBP_RESET_N \
	GPIO_CFG(44, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA)
#if 0 /* 46 is power key GPIO */
#define MBP_INT0 \
	GPIO_CFG(46, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA)
#endif

#define MBP_MODE_CTRL_0 \
	GPIO_CFG(35, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA)
#define MBP_MODE_CTRL_1 \
	GPIO_CFG(36, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA)
#define MBP_MODE_CTRL_2 \
	GPIO_CFG(34, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA)
#define TSIF_EN \
	GPIO_CFG(35, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN,	GPIO_CFG_2MA)
#define TSIF_DATA \
	GPIO_CFG(36, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN,	GPIO_CFG_2MA)
#define TSIF_CLK \
	GPIO_CFG(34, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA)

static struct msm_gpio mbp_cfg_data[] = {
	{GPIO_CFG(44, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA),
		"mbp_reset"},
	{GPIO_CFG(85, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA),
		"mbp_io_voltage"},
};

static int mbp_config_gpios_pre_init(int enable)
{
	int rc = 0;

	if (enable) {
		rc = msm_gpios_request_enable(mbp_cfg_data,
			ARRAY_SIZE(mbp_cfg_data));
		if (rc) {
			printk(KERN_ERR
				"%s: Failed to turnon GPIOs for mbp chip(%d)\n",
				__func__, rc);
		}
	} else
		msm_gpios_disable_free(mbp_cfg_data, ARRAY_SIZE(mbp_cfg_data));
	return rc;
}

static int mbp_setup_rf_vregs(int state)
{
	struct vreg *vreg_rf = NULL;
	struct vreg *vreg_rf_switch	= NULL;
	int rc;

	vreg_rf = vreg_get(NULL, "s2");
	if (IS_ERR(vreg_rf)) {
		pr_err("%s: s2 vreg get failed (%ld)",
				__func__, PTR_ERR(vreg_rf));
		return -EFAULT;
	}
	vreg_rf_switch = vreg_get(NULL, "rf");
	if (IS_ERR(vreg_rf_switch)) {
		pr_err("%s: rf vreg get failed (%ld)",
				__func__, PTR_ERR(vreg_rf_switch));
		return -EFAULT;
	}

	if (state) {
		rc = vreg_set_level(vreg_rf, 1300);
		if (rc) {
			pr_err("%s: vreg s2 set level failed (%d)\n",
					__func__, rc);
			return rc;
		}

		rc = vreg_enable(vreg_rf);
		if (rc) {
			printk(KERN_ERR "%s: vreg_enable(s2) = %d\n",
					__func__, rc);
		}

		rc = vreg_set_level(vreg_rf_switch, 2600);
		if (rc) {
			pr_err("%s: vreg rf switch set level failed (%d)\n",
					__func__, rc);
			return rc;
		}
		rc = vreg_enable(vreg_rf_switch);
		if (rc) {
			printk(KERN_ERR "%s: vreg_enable(rf) = %d\n",
					__func__, rc);
		}
	} else {
		(void) vreg_disable(vreg_rf);
		(void) vreg_disable(vreg_rf_switch);
	}
	return 0;
}

static int mbp_setup_vregs(int state)
{
	struct vreg *vreg_analog = NULL;
	struct vreg *vreg_io = NULL;
	int rc;

	vreg_analog = vreg_get(NULL, "gp4");
	if (IS_ERR(vreg_analog)) {
		pr_err("%s: gp4 vreg get failed (%ld)",
				__func__, PTR_ERR(vreg_analog));
		return -EFAULT;
	}
	vreg_io = vreg_get(NULL, "s3");
	if (IS_ERR(vreg_io)) {
		pr_err("%s: s3 vreg get failed (%ld)",
				__func__, PTR_ERR(vreg_io));
		return -EFAULT;
	}
	if (state) {
		rc = vreg_set_level(vreg_analog, 2600);
		if (rc) {
			pr_err("%s: vreg_set_level failed (%d)",
					__func__, rc);
		}
		rc = vreg_enable(vreg_analog);
		if (rc) {
			pr_err("%s: analog vreg enable failed (%d)",
					__func__, rc);
		}
		rc = vreg_set_level(vreg_io, 1800);
		if (rc) {
			pr_err("%s: vreg_set_level failed (%d)",
					__func__, rc);
		}
		rc = vreg_enable(vreg_io);
		if (rc) {
			pr_err("%s: io vreg enable failed (%d)",
					__func__, rc);
		}
	} else {
		rc = vreg_disable(vreg_analog);
		if (rc) {
			pr_err("%s: analog vreg disable failed (%d)",
					__func__, rc);
		}
		rc = vreg_disable(vreg_io);
		if (rc) {
			pr_err("%s: io vreg disable failed (%d)",
					__func__, rc);
		}
	}
	return rc;
}

static int mbp_set_tcxo_en(int enable)
{
	int rc;
	const char *id = "UBMC";
	struct vreg *vreg_analog = NULL;

	rc = pmapp_clock_vote(id, PMAPP_CLOCK_ID_A1,
		enable ? PMAPP_CLOCK_VOTE_ON : PMAPP_CLOCK_VOTE_OFF);
	if (rc < 0) {
		printk(KERN_ERR "%s: unable to %svote for a1 clk\n",
			__func__, enable ? "" : "de-");
		return -EIO;
	}
	if (!enable) {
		vreg_analog = vreg_get(NULL, "gp4");
		if (IS_ERR(vreg_analog)) {
			pr_err("%s: gp4 vreg get failed (%ld)",
					__func__, PTR_ERR(vreg_analog));
			return -EFAULT;
		}

		(void) vreg_disable(vreg_analog);
	}
	return rc;
}

static void mbp_set_freeze_io(int state)
{
	if (state)
		gpio_set_value(85, 0);
	else
		gpio_set_value(85, 1);
}

static int mbp_set_core_voltage_en(int enable)
{
	int rc;
	struct vreg *vreg_core1p2 = NULL;

	vreg_core1p2 = vreg_get(NULL, "gp16");
	if (IS_ERR(vreg_core1p2)) {
		pr_err("%s: gp16 vreg get failed (%ld)",
				__func__, PTR_ERR(vreg_core1p2));
		return -EFAULT;
	}
	if (enable) {
		rc = vreg_set_level(vreg_core1p2, 1200);
		if (rc) {
			pr_err("%s: vreg_set_level failed (%d)",
					__func__, rc);
		}
		(void) vreg_enable(vreg_core1p2);

		return 80;
	} else {
		gpio_set_value(85, 1);
		return 0;
	}
	return rc;
}

static void mbp_set_reset(int state)
{
	if (state)
		gpio_set_value(GPIO_PIN(MBP_RESET_N), 0);
	else
		gpio_set_value(GPIO_PIN(MBP_RESET_N), 1);
}

static int mbp_config_interface_mode(int state)
{
	if (state) {
		gpio_tlmm_config(MBP_MODE_CTRL_0, GPIO_CFG_ENABLE);
		gpio_tlmm_config(MBP_MODE_CTRL_1, GPIO_CFG_ENABLE);
		gpio_tlmm_config(MBP_MODE_CTRL_2, GPIO_CFG_ENABLE);
		gpio_set_value(GPIO_PIN(MBP_MODE_CTRL_0), 0);
		gpio_set_value(GPIO_PIN(MBP_MODE_CTRL_1), 1);
		gpio_set_value(GPIO_PIN(MBP_MODE_CTRL_2), 0);
	} else {
		gpio_tlmm_config(MBP_MODE_CTRL_0, GPIO_CFG_DISABLE);
		gpio_tlmm_config(MBP_MODE_CTRL_1, GPIO_CFG_DISABLE);
		gpio_tlmm_config(MBP_MODE_CTRL_2, GPIO_CFG_DISABLE);
	}
	return 0;
}

static int mbp_setup_adc_vregs(int state)
{
	struct vreg *vreg_adc = NULL;
	int rc;

	vreg_adc = vreg_get(NULL, "s4");
	if (IS_ERR(vreg_adc)) {
		pr_err("%s: s4 vreg get failed (%ld)",
				__func__, PTR_ERR(vreg_adc));
		return -EFAULT;
	}
	if (state) {
		rc = vreg_set_level(vreg_adc, 2200);
		if (rc) {
			pr_err("%s: vreg_set_level failed (%d)",
					__func__, rc);
		}
		rc = vreg_enable(vreg_adc);
		if (rc) {
			pr_err("%s: enable vreg adc failed (%d)",
					__func__, rc);
		}
	} else {
		rc = vreg_disable(vreg_adc);
		if (rc) {
			pr_err("%s: disable vreg adc failed (%d)",
					__func__, rc);
		}
	}
	return rc;
}

static int mbp_power_up(void)
{
	int rc;

	rc = mbp_config_gpios_pre_init(MBP_ON);
	if (rc)
		goto exit;
	pr_debug("%s: mbp_config_gpios_pre_init() done\n", __func__);

	rc = mbp_setup_vregs(MBP_ON);
	if (rc)
		goto exit;
	pr_debug("%s: gp4 (2.6) and s3 (1.8) done\n", __func__);

	rc = mbp_set_tcxo_en(MBP_ON);
	if (rc)
		goto exit;
	pr_debug("%s: tcxo clock done\n", __func__);

	mbp_set_freeze_io(MBP_OFF);
	pr_debug("%s: set gpio 85 to 1 done\n", __func__);

	udelay(100);
	mbp_set_reset(MBP_ON);

	udelay(300);
	rc = mbp_config_interface_mode(MBP_ON);
	if (rc)
		goto exit;
	pr_debug("%s: mbp_config_interface_mode() done\n", __func__);

	udelay(100 + mbp_set_core_voltage_en(MBP_ON));
	pr_debug("%s: power gp16 1.2V done\n", __func__);

	mbp_set_freeze_io(MBP_ON);
	pr_debug("%s: set gpio 85 to 0 done\n", __func__);

	udelay(100);

	rc = mbp_setup_rf_vregs(MBP_ON);
	if (rc)
		goto exit;
	pr_debug("%s: s2 1.3V and rf 2.6V done\n", __func__);

	rc = mbp_setup_adc_vregs(MBP_ON);
	if (rc)
		goto exit;
	pr_debug("%s: s4 2.2V  done\n", __func__);

	udelay(200);

	mbp_set_reset(MBP_OFF);
	pr_debug("%s: close gpio 44 done\n", __func__);

	msleep(20);
exit:
	return rc;
}

static int mbp_power_down(void)
{
	int rc;
	struct vreg *vreg_adc = NULL;

	vreg_adc = vreg_get(NULL, "s4");
	if (IS_ERR(vreg_adc)) {
		pr_err("%s: s4 vreg get failed (%ld)",
				__func__, PTR_ERR(vreg_adc));
		return -EFAULT;
	}

	mbp_set_reset(MBP_ON);
	pr_debug("%s: mbp_set_reset(MBP_ON) done\n", __func__);

	udelay(100);

	rc = mbp_setup_adc_vregs(MBP_OFF);
	if (rc)
		goto exit;
	pr_debug("%s: vreg_disable(vreg_adc) done\n", __func__);

	udelay(5);

	rc = mbp_setup_rf_vregs(MBP_OFF);
	if (rc)
		goto exit;
	pr_debug("%s: mbp_setup_rf_vregs(MBP_OFF) done\n", __func__);

	udelay(5);

	mbp_set_freeze_io(MBP_OFF);
	pr_debug("%s: mbp_set_freeze_io(MBP_OFF) done\n", __func__);

	udelay(100);
	rc = mbp_set_core_voltage_en(MBP_OFF);
	if (rc)
		goto exit;
	pr_debug("%s: mbp_set_core_voltage_en(MBP_OFF) done\n", __func__);

	gpio_set_value(85, 1);

	rc = mbp_set_tcxo_en(MBP_OFF);
	if (rc)
		goto exit;
	pr_debug("%s: mbp_set_tcxo_en(MBP_OFF) done\n", __func__);

	rc = mbp_config_gpios_pre_init(MBP_OFF);
	if (rc)
		goto exit;
exit:
	return rc;
}

static void (*mbp_status_notify_cb)(int card_present, void *dev_id);
static void *mbp_status_notify_cb_devid;
static int mbp_power_status;
static int mbp_power_init_done;

static uint32_t mbp_setup_power(struct device *dv,
	unsigned int power_status)
{
	int rc = 0;
	struct platform_device *pdev;

	pdev = container_of(dv, struct platform_device, dev);

	if (power_status == mbp_power_status)
		goto exit;
	if (power_status) {
		pr_debug("turn on power of mbp slot");
		rc = mbp_power_up();
		mbp_power_status = 1;
	} else {
		pr_debug("turn off power of mbp slot");
		rc = mbp_power_down();
		mbp_power_status = 0;
	}
exit:
	return rc;
};

int mbp_register_status_notify(void (*callback)(int, void *),
	void *dev_id)
{
	mbp_status_notify_cb = callback;
	mbp_status_notify_cb_devid = dev_id;
	return 0;
}

static unsigned int mbp_status(struct device *dev)
{
	return mbp_power_status;
}

static uint32_t msm_sdcc_setup_power_mbp(struct device *dv, unsigned int vdd)
{
	struct platform_device *pdev;
	uint32_t rc = 0;

	pdev = container_of(dv, struct platform_device, dev);
	rc = msm_sdcc_setup_power(dv, vdd);
	if (rc) {
		pr_err("%s: Failed to setup power (%d)\n",
			__func__, rc);
		goto out;
	}
	if (!mbp_power_init_done) {
		mbp_setup_power(dv, 1);
		mbp_setup_power(dv, 0);
		mbp_power_init_done = 1;
	}
	if (vdd >= 0x8000) {
		rc = mbp_setup_power(dv, (0x8000 == vdd) ? 0 : 1);
		if (rc) {
			pr_err("%s: Failed to config mbp chip power (%d)\n",
				__func__, rc);
			goto out;
		}
		if (mbp_status_notify_cb) {
			mbp_status_notify_cb(mbp_power_status,
				mbp_status_notify_cb_devid);
		}
	}
out:
	/* should return 0 only */
	return 0;
}

#endif

#endif

#ifdef CONFIG_MMC_MSM_SDC4_SUPPORT
#ifdef CONFIG_MMC_MSM_CARD_HW_DETECTION
static unsigned int msm7x30_sdcc_slot_status(struct device *dev)
{
	return (unsigned int)
		!gpio_get_value_cansleep(
			PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_SD_DET - 1));
}
#endif
#ifdef CONFIG_MMC_MSM_SDC4_WP_SUPPORT
static int msm_sdcc_get_wpswitch(struct device *dv)
{
	void __iomem *wp_addr = 0;
	uint32_t ret = 0;
	struct platform_device *pdev;

	if (!(machine_is_msm7x30_surf()))
		return -1;
	pdev = container_of(dv, struct platform_device, dev);

	wp_addr = ioremap(FPGA_SDCC_STATUS, 4);
	if (!wp_addr) {
		pr_err("%s: Could not remap %x\n", __func__, FPGA_SDCC_STATUS);
		return -ENOMEM;
	}

	ret = (((readl(wp_addr) >> 4) >> (pdev->id-1)) & 0x01);
	pr_info("%s: WP Status for Slot %d = 0x%x \n", __func__,
							pdev->id, ret);
	iounmap(wp_addr);

	return ret;
}
#endif
#endif

#if defined(CONFIG_MMC_MSM_SDC1_SUPPORT)
#if defined(CONFIG_CSDIO_VENDOR_ID) && \
	defined(CONFIG_CSDIO_DEVICE_ID) && \
	(CONFIG_CSDIO_VENDOR_ID == 0x70 && CONFIG_CSDIO_DEVICE_ID == 0x1117)
static struct mmc_platform_data msm7x30_sdc1_data = {
	.ocr_mask	= MMC_VDD_165_195 | MMC_VDD_27_28 | MMC_VDD_28_29,
	.translate_vdd	= msm_sdcc_setup_power_mbp,
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
	.status	        = mbp_status,
	.register_status_notify = mbp_register_status_notify,
	.msmsdcc_fmin	= 144000,
	.msmsdcc_fmid	= 24576000,
	.msmsdcc_fmax	= 24576000,
	.nonremovable	= 0,
};
#else
static struct mmc_platform_data msm7x30_sdc1_data = {
	.ocr_mask	= MMC_VDD_165_195,
	.translate_vdd	= msm_sdcc_setup_power,
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
	.msmsdcc_fmin	= 144000,
	.msmsdcc_fmid	= 24576000,
	.msmsdcc_fmax	= 49152000,
	.nonremovable	= 0,
};
#endif
#endif

#ifdef CONFIG_MMC_MSM_SDC2_SUPPORT
static unsigned int mecha_sdc2_slot_type = MMC_TYPE_MMC;
static struct mmc_platform_data msm7x30_sdc2_data = {
	.ocr_mask       = MMC_VDD_165_195 | MMC_VDD_27_28,
#ifdef CONFIG_MMC_MSM_SDC2_8_BIT_SUPPORT
	.mmc_bus_width  = MMC_CAP_8_BIT_DATA,
#else
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
#endif
	.msmsdcc_fmin	= 144000,
	.msmsdcc_fmid	= 24576000,
	.msmsdcc_fmax	= 49152000,
	.slot_type		= &mecha_sdc2_slot_type,
	.nonremovable	= 1,
	.emmc_dma_ch	= 7,
};
#endif

#ifdef CONFIG_MMC_MSM_SDC3_SUPPORT
#if 0
static unsigned int mecha_sdc3_slot_type = MMC_TYPE_SDIO_WIFI;
static struct mmc_platform_data msm7x30_sdc3_data = {
	.ocr_mask	= MMC_VDD_27_28 | MMC_VDD_28_29,
	.translate_vdd	= msm_sdcc_setup_power,
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
#ifdef CONFIG_MMC_MSM_SDIO_SUPPORT
	.sdiowakeup_irq = MSM_GPIO_TO_INT(118),
#endif
	.msmsdcc_fmin	= 144000,
	.msmsdcc_fmid	= 24576000,
	.msmsdcc_fmax	= 49152000,
	.slot_type		= &mecha_sdc3_slot_type,
	.nonremovable	= 0,
};
#endif
#endif

#ifdef CONFIG_MMC_MSM_SDC4_SUPPORT
static unsigned int mecha_sdc4_slot_type = MMC_TYPE_SD;
static struct mmc_platform_data msm7x30_sdc4_data = {
	.ocr_mask	= MMC_VDD_27_28 | MMC_VDD_28_29,
	.translate_vdd	= msm_sdcc_setup_power,
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
#ifdef CONFIG_MMC_MSM_CARD_HW_DETECTION
	.status      = msm7x30_sdcc_slot_status,
	.status_irq  = PM8058_GPIO_IRQ(PMIC8058_IRQ_BASE, PMIC_GPIO_SD_DET - 1),
	.irq_flags   = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
#endif
	/*.wpswitch    = msm_sdcc_get_wpswitch,*/
	.msmsdcc_fmin	= 144000,
	.msmsdcc_fmid	= 24576000,
	.msmsdcc_fmax	= 49152000,
	.slot_type		= &mecha_sdc4_slot_type,
	.nonremovable	= 0,
};
#endif

#ifdef CONFIG_MMC_MSM_SDC1_SUPPORT
static void msm_sdc1_lvlshft_enable(void)
{
	int rc;

	/* Enable LDO5, an input to the FET that powers slot 1 */
	rc = vreg_set_level(vreg_mmc, 2850);
	if (rc)
		printk(KERN_ERR "%s: vreg_set_level() = %d \n",	__func__, rc);

	rc = vreg_enable(vreg_mmc);
	if (rc)
		printk(KERN_ERR "%s: vreg_enable() = %d \n", __func__, rc);

	/* Enable GPIO 35, to turn on the FET that powers slot 1 */
	rc = msm_gpios_request_enable(sdc1_lvlshft_cfg_data,
				ARRAY_SIZE(sdc1_lvlshft_cfg_data));
	if (rc)
		printk(KERN_ERR "%s: Failed to enable GPIO 35\n", __func__);

	rc = gpio_direction_output(GPIO_PIN(sdc1_lvlshft_cfg_data[0].gpio_cfg),
				1);
	if (rc)
		printk(KERN_ERR "%s: Failed to turn on GPIO 35\n", __func__);
}
#endif

static void __init msm7x30_init_mmc(void)
{
	vreg_s3 = vreg_get(NULL, "s3");
	if (IS_ERR(vreg_s3)) {
		printk(KERN_ERR "%s: vreg get failed (%ld)\n",
		       __func__, PTR_ERR(vreg_s3));
		return;
	}

	vreg_mmc = vreg_get(NULL, "gp10");
	if (IS_ERR(vreg_mmc)) {
		printk(KERN_ERR "%s: vreg get failed (%ld)\n",
		       __func__, PTR_ERR(vreg_mmc));
		return;
	}

#ifdef CONFIG_MMC_MSM_SDC1_SUPPORT
	if (machine_is_msm7x30_fluid()) {
		msm7x30_sdc1_data.ocr_mask =  MMC_VDD_27_28 | MMC_VDD_28_29;
		msm_sdc1_lvlshft_enable();
	}
	sdcc_vreg_data[0].vreg_data = vreg_s3;
	sdcc_vreg_data[0].level = 1800;
	msm_add_sdcc(1, &msm7x30_sdc1_data);
#endif
#ifdef CONFIG_MMC_MSM_SDC2_SUPPORT
	if (machine_is_msm8x55_svlte_surf())
		msm7x30_sdc2_data.msmsdcc_fmax =  24576000;
	sdcc_vreg_data[1].vreg_data = vreg_s3;
	sdcc_vreg_data[1].level = 1800;
	msm7x30_sdc2_data.swfi_latency =
		msm_pm_data[MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT].latency;
	msm_add_sdcc(2, &msm7x30_sdc2_data);
#endif
#ifdef CONFIG_MMC_MSM_SDC3_SUPPORT
	sdcc_vreg_data[2].vreg_data = vreg_s3;
	sdcc_vreg_data[2].level = 1800;
/* HTC_WIFI_START */
	/*
	msm_sdcc_setup_gpio(3, 1);
	msm_add_sdcc(3, &msm7x30_sdc3_data);
	*/
	mecha_init_mmc(system_rev);
/* HTC_WIFI_END*/
#endif
#ifdef CONFIG_MMC_MSM_SDC4_SUPPORT
	sdcc_vreg_data[3].vreg_data = vreg_mmc;
	sdcc_vreg_data[3].level = 2850;
	msm_add_sdcc(4, &msm7x30_sdc4_data);
#endif

}

static struct msm_spm_platform_data msm_spm_data __initdata = {
	.reg_base_addr = MSM_SAW_BASE,

	.reg_init_values[MSM_SPM_REG_SAW_CFG] = 0x05,
	.reg_init_values[MSM_SPM_REG_SAW_SPM_CTL] = 0x18,
	.reg_init_values[MSM_SPM_REG_SAW_SPM_SLP_TMR_DLY] = 0x00006666,
	.reg_init_values[MSM_SPM_REG_SAW_SPM_WAKE_TMR_DLY] = 0xFF000666,

	.reg_init_values[MSM_SPM_REG_SAW_SLP_CLK_EN] = 0x01,
	.reg_init_values[MSM_SPM_REG_SAW_SLP_HSFS_PRECLMP_EN] = 0x03,
	.reg_init_values[MSM_SPM_REG_SAW_SLP_HSFS_POSTCLMP_EN] = 0x00,

	.reg_init_values[MSM_SPM_REG_SAW_SLP_CLMP_EN] = 0x01,
	.reg_init_values[MSM_SPM_REG_SAW_SLP_RST_EN] = 0x00,
	.reg_init_values[MSM_SPM_REG_SAW_SPM_MPM_CFG] = 0x00,

	.awake_vlevel = 0xF2,
	.retention_vlevel = 0xE0,
	.collapse_vlevel = 0x72,
	.retention_mid_vlevel = 0xE0,
	.collapse_mid_vlevel = 0xE0,

	.vctl_timeout_us = 50,
};

static ssize_t mecha_virtual_keys_show(struct kobject *kobj,
			struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf,
		__stringify(EV_KEY) ":" __stringify(KEY_HOME)		":55:830:98:50"
		":" __stringify(EV_KEY) ":" __stringify(KEY_MENU)	":184:830:120:50"
		":" __stringify(EV_KEY) ":" __stringify(KEY_BACK)	":300:830:100:50"
		":" __stringify(EV_KEY) ":" __stringify(KEY_SEARCH) ":428:830:88:50"
		"\n");
}

static struct kobj_attribute mecha_virtual_keys_attr = {
	.attr = {
		.name = "virtualkeys.atmel-touchscreen",
		.mode = S_IRUGO,
	},
	.show = &mecha_virtual_keys_show,
};

static struct attribute *mecha_properties_attrs[] = {
	&mecha_virtual_keys_attr.attr,
	NULL
};

static struct attribute_group mecha_properties_attr_group = {
	.attrs = mecha_properties_attrs,
};

#ifdef CONFIG_PERFLOCK
static unsigned mecha_perf_acpu_table[] = {
	245000000,
	768000000,
	1024000000,
};

static struct perflock_platform_data mecha_perflock_data = {
	.perf_acpu_table = mecha_perf_acpu_table,
	.table_size = ARRAY_SIZE(mecha_perf_acpu_table),
};
#endif

static void mecha_reset(void)
{
	gpio_set_value(MECHA_GPIO_PS_HOLD, 0);
}

static void mecha_usb_hub_gpio_config(bool enable)
{
	if (enable) {
		if (system_rev >= 2) {
			/*XC board: hub switch off*/
			gpio_set_value(PM8058_GPIO_PM_TO_SYS(MECHA_GPIO_USB_HUB_SWITCH), 0);
		}
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(MECHA_USB_HUB_PWR), 0);
		mdelay(5);
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(MECHA_USB_HUB_PWR), 1);

		if (system_rev >= 2) {
			/*XC board: hub switch on*/
			mdelay(1);
			gpio_set_value(PM8058_GPIO_PM_TO_SYS(MECHA_GPIO_USB_HUB_SWITCH), 1);
		}
		mdelay(5);
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(MECHA_USB_HUB_RESET), 1);
		mdelay(5);
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(MECHA_USB_HUB_RESET), 0);
		mdelay(5);
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(MECHA_USB_HUB_RESET), 1);
	} else {
		if (system_rev >= 2) {
			/*XC board: hub switch off*/
			gpio_set_value(PM8058_GPIO_PM_TO_SYS(MECHA_GPIO_USB_HUB_SWITCH), 0);
		}
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(MECHA_USB_HUB_RESET), 0);
		gpio_set_value(PM8058_GPIO_PM_TO_SYS(MECHA_USB_HUB_PWR), 0);
		udelay(50);
	}
}

void mecha_add_usb_devices(void)
{
	printk(KERN_INFO "%s rev: %d\n", __func__, system_rev);
	android_usb_pdata.products[0].product_id =
			android_usb_pdata.product_id;


	/* diag bit set */
	if (get_radio_flag() & 0x20000)
		android_usb_pdata.diag_init = 1;

	msm_device_gadget_peripheral.dev.parent = &msm_device_otg.dev;
	platform_device_register(&msm_device_gadget_peripheral);
	platform_device_register(&android_usb_device);
}

static int __init board_serialno_setup(char *serialno)
{
	android_usb_pdata.serial_number = serialno;
	return 1;
}

__setup("androidboot.serialno=", board_serialno_setup);

#ifdef CONFIG_MDP4_HW_VSYNC
static void mecha_te_gpio_config(void)
{
	uint32_t te_gpio_table[] = {
		PCOM_GPIO_CFG(30, 1, GPIO_INPUT, GPIO_PULL_DOWN, GPIO_2MA),
	};
	config_gpio_table(te_gpio_table, ARRAY_SIZE(te_gpio_table));
}
#endif

static struct platform_device *devices[] __initdata = {
	&ram_console_device,
#if defined(CONFIG_SERIAL_MSM) || defined(CONFIG_MSM_SERIAL_DEBUGGER)
	&msm_device_uart2,
#endif
#ifdef CONFIG_MSM_PROC_COMM_REGULATOR
	&msm_proccomm_regulator_dev,
#endif
	&asoc_msm_pcm,
	&asoc_msm_dai0,
	&asoc_msm_dai1,
#if defined(CONFIG_SND_MSM_MVS_DAI_SOC)
	&asoc_msm_mvs,
	&asoc_mvs_dai0,
	&asoc_mvs_dai1,
#endif
	&msm_device_smd,
	&msm_device_dmov,
#if 0
	&smc91x_device,
	&smsc911x_device,
	&msm_device_nand,
#endif
	&msm_device_otg,
	&qsd_device_spi,
#ifdef CONFIG_MSM_SSBI
	&msm_device_ssbi_pmic1,
#endif
#ifdef CONFIG_I2C_SSBI
	/*&msm_device_ssbi6,*/
	&msm_device_ssbi7,
#endif
	&android_pmem_device,
	&msm_migrate_pages_device,
#ifdef CONFIG_MSM_ROTATOR
	&msm_rotator_device,
#endif
	&android_pmem_adsp_device,
	&android_pmem_audio_device,
	&msm_device_i2c,
	&msm_device_i2c_2,
	&hs_device,
#ifdef CONFIG_MSM7KV2_AUDIO
	&msm_aictl_device,
	&msm_mi2s_device,
	&msm_lpa_device,
	&msm_aux_pcm_device,
#endif

#ifdef CONFIG_S5K3H1GX
	&msm_camera_sensor_s5k3h1gx,
#endif

#ifdef CONFIG_S5K6AAFX
	&msm_camera_sensor_s5k6aafx,
#endif

	&msm_device_adspdec,
	&qup_device_i2c,
#if defined(CONFIG_MARIMBA_CORE) && \
   (defined(CONFIG_MSM_BT_POWER) || defined(CONFIG_MSM_BT_POWER_MODULE))
	/*&msm_bt_power_device,*/
#endif
	&msm_kgsl_3d0,
	&msm_kgsl_2d0,
	&msm_device_vidc_720p,
#ifdef CONFIG_MSM_GEMINI
	&msm_gemini_device,
#endif
#ifdef CONFIG_MSM_VPE
	&msm_vpe_device,
#endif
#if defined(CONFIG_TSIF) || defined(CONFIG_TSIF_MODULE)
	&msm_device_tsif,
#endif
#ifdef CONFIG_MSM_SDIO_AL
	/* &msm_device_sdio_al, */
#endif

#if defined(CONFIG_CRYPTO_DEV_QCRYPTO) || \
		defined(CONFIG_CRYPTO_DEV_QCRYPTO_MODULE)
	&qcrypto_device,
#endif

#if defined(CONFIG_CRYPTO_DEV_QCEDEV) || \
		defined(CONFIG_CRYPTO_DEV_QCEDEV_MODULE)
	&qcedev_device,
#endif

	&htc_battery_pdev,
	&msm_adc_device,
	&msm_ebi0_thermal,
	&msm_ebi1_thermal,
#ifdef CONFIG_SERIAL_MSM_HS
	&msm_device_uart_dm1,
#endif
#ifdef CONFIG_BT
	&mecha_rfkill,
#endif
#ifdef CONFIG_INPUT_CAPELLA_CM3602
	&capella_cm3602,
#endif
#ifdef CONFIG_FLASHLIGHT_AAT1271
	&mecha_flashlight_device,
#endif
	&pm8058_leds,
	&cable_detect_device,
	&htc_drm,
};

static void __init mecha_init(void)
{
	int rc = 0;
	struct kobject *properties_kobj;
	unsigned smem_size;
	uint32_t soc_version = 0;
	struct proc_dir_entry *entry = NULL;

	soc_version = socinfo_get_version();

	/* Must set msm_hw_reset_hook before first proc comm */
	msm_hw_reset_hook = mecha_reset;

	msm_clock_init(&msm7x30_clock_init_data);
	msm_spm_init(&msm_spm_data, 1);
	acpuclk_init(&acpuclk_7x30_soc_data);

#ifdef CONFIG_PERFLOCK
	perflock_init(&mecha_perflock_data);
#endif

#ifdef CONFIG_BT
	bt_export_bd_address();
#endif

#ifdef CONFIG_SERIAL_MSM_HS
	msm_uart_dm1_pdata.rx_wakeup_irq = gpio_to_irq(MECHA_GPIO_BT_HOST_WAKE);
	msm_device_uart_dm1.name = "msm_serial_hs_brcm";
	msm_device_uart_dm1.dev.platform_data = &msm_uart_dm1_pdata;
#endif

#ifdef CONFIG_USB_MSM_OTG_72K
	if (SOCINFO_VERSION_MAJOR(soc_version) >= 2 &&
			SOCINFO_VERSION_MINOR(soc_version) >= 1) {
		pr_debug("%s: SOC Version:2.(1 or more)\n", __func__);
		msm_otg_pdata.ldo_set_voltage = 0;
	}

#ifdef CONFIG_USB_GADGET
	msm_otg_pdata.swfi_latency =
	msm_pm_data
	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].latency;
	msm_device_gadget_peripheral.dev.platform_data = &msm_gadget_pdata;
#endif
#endif
	msm_device_otg.dev.platform_data = &msm_otg_pdata;

#ifdef CONFIG_MSM_SSBI
	msm_device_ssbi_pmic1.dev.platform_data =
				&msm7x30_ssbi_pm8058_pdata;
#endif
//	buses_init();
	platform_add_devices(devices, ARRAY_SIZE(devices));

	mecha_add_usb_devices();

#ifdef CONFIG_USB_EHCI_MSM_72K
	msm_add_host(0, &msm_usb_host_pdata);
#endif
	msm7x30_init_mmc();
	msm_qsd_spi_init();

	spi_register_board_info(msm_spi_board_info, ARRAY_SIZE(msm_spi_board_info));

	msm_pm_set_platform_data(msm_pm_data, ARRAY_SIZE(msm_pm_data));
	BUG_ON(msm_pm_boot_init(MSM_PM_BOOT_CONFIG_RESET_VECTOR, ioremap(0x0, PAGE_SIZE)));

	msm_device_i2c_init();
	msm_device_i2c_2_init();
#ifdef CONFIG_MICROP_COMMON
	mecha_microp_init();
#endif
	qup_device_i2c_init();
	mecha_init_marimba();
#ifdef CONFIG_MSM7KV2_AUDIO
	aux_pcm_gpio_init();
	spi_register_board_info(msm_spi_board_info,
		ARRAY_SIZE(msm_spi_board_info));
	msm_snddev_init();
	mecha_audio_init();
#endif

	i2c_register_board_info(0, msm_i2c_board_info,
			ARRAY_SIZE(msm_i2c_board_info));

	i2c_register_board_info(0, i2c_tps_devices,
			ARRAY_SIZE(i2c_tps_devices));

#ifdef CONFIG_BOSCH_BMA150
	if (machine_is_msm7x30_fluid())
		i2c_register_board_info(0, bma150_board_info,
					ARRAY_SIZE(bma150_board_info));
#endif

	if ((engineerid & 0x3) == 0x0) {
		printk("[%s]\n, engineerid = 0x%x, register devices_CM3602_Proximity", __func__, engineerid);
		platform_add_devices(devices_CM3602_Proximity, ARRAY_SIZE(devices_CM3602_Proximity));
		platform_add_devices(devices_Lightsensor, ARRAY_SIZE(devices_Lightsensor));
		i2c_register_board_info(0, i2c_microp_devices,
			ARRAY_SIZE(i2c_microp_devices));
	}

	if ((engineerid & 0x3) == 0x1 || (engineerid & 0x3) == 0x2) {
		printk("[%s]\n, engineerid = 0x%x, register i2c_CM3628_devices", __func__, engineerid);
		if ((engineerid & 0x3) == 0x1)
			cm3628_pdata.ps_conf2_val = CM3628_IT_4X_ES; /*ES smp*/
		i2c_register_board_info(0, i2c_CM3628_devices,
				ARRAY_SIZE(i2c_CM3628_devices));
	}

	i2c_register_board_info(2, msm_marimba_board_info,
			ARRAY_SIZE(msm_marimba_board_info));
	i2c_register_board_info(0, tpa2051_devices,
			ARRAY_SIZE(tpa2051_devices));
//	i2c_register_board_info(2, msm_i2c_gsbi7_timpani_info,
//			ARRAY_SIZE(msm_i2c_gsbi7_timpani_info));
#if 0
	if (gpio_get_value(MECHA_CAM_ID) == 0) {
		i2c_register_board_info(4 /* QUP ID */, msm_camera_boardinfo,
					ARRAY_SIZE(msm_camera_boardinfo));
	}	else {
		i2c_register_board_info(4 /* QUP ID */, msm_camera_boardinfo_OV8830,
					ARRAY_SIZE(msm_camera_boardinfo_OV8830));
	}
#else

	i2c_register_board_info(4 /* QUP ID */, msm_camera_boardinfo,
			ARRAY_SIZE(msm_camera_boardinfo));


#endif

	/*bt_power_init();*/
#ifdef CONFIG_I2C_SSBI
	/*msm_device_ssbi6.dev.platform_data = &msm_i2c_ssbi6_pdata;*/
	msm_device_ssbi7.dev.platform_data = &msm_i2c_ssbi7_pdata;
#endif
#if 0
	if (machine_is_msm7x30_fluid())
		i2c_register_board_info(0, msm_isa1200_board_info,
			ARRAY_SIZE(msm_isa1200_board_info));
#endif

	pm8058_gpios_init();

	entry = create_proc_read_entry("emmc", 0, NULL, emmc_partition_read_proc, NULL);
	if (!entry)
		printk(KERN_ERR "Create /proc/emmc FAILED!\n");

	entry = create_proc_read_entry("dying_processes", 0, NULL, dying_processors_read_proc, NULL);
	if (!entry)
		printk(KERN_ERR"Create /proc/dying_processes FAILED!\n");

	boot_reason = *(unsigned int *)
		(smem_get_entry(SMEM_POWER_ON_STATUS_INFO, &smem_size));
	printk(KERN_NOTICE "Boot Reason = 0x%02x\n", boot_reason);

	i2c_register_board_info(0, i2c_devices,
				ARRAY_SIZE(i2c_devices));

	i2c_register_board_info(0, i2c_Sensors_devices,
				ARRAY_SIZE(i2c_Sensors_devices));

	if (system_rev <= 1) {
		printk(KERN_INFO"add device_pm8058_led\n");
		platform_add_devices(device_pm8058_led,
			ARRAY_SIZE(device_pm8058_led));
	} else
		platform_add_devices(device_pm8058_led_XC,
			ARRAY_SIZE(device_pm8058_led_XC));


	/*Virtual_key*/
	properties_kobj = kobject_create_and_add("board_properties", NULL);
	if (properties_kobj)
		rc = sysfs_create_group(properties_kobj,
				&mecha_properties_attr_group);
	if (!properties_kobj || rc)
		pr_err("failed to create board_properties\n");

	mecha_init_keypad();
#ifdef CONFIG_MDP4_HW_VSYNC
	mecha_te_gpio_config();
#endif
	mecha_init_panel();
	mecha_wifi_init();
	msm_init_pmic_vibrator(3000);
}

static unsigned pmem_sf_size = MSM_PMEM_SF_SIZE;
static int __init pmem_sf_size_setup(char *p)
{
	pmem_sf_size = memparse(p, NULL);
	return 0;
}
early_param("pmem_sf_size", pmem_sf_size_setup);

static unsigned fb_size = MSM_FB_SIZE;
static int __init fb_size_setup(char *p)
{
	fb_size = memparse(p, NULL);
	return 0;
}
early_param("fb_size", fb_size_setup);

static unsigned pmem_adsp_size = MSM_PMEM_ADSP_SIZE;
static int __init pmem_adsp_size_setup(char *p)
{
	pmem_adsp_size = memparse(p, NULL);
	return 0;
}
early_param("pmem_adsp_size", pmem_adsp_size_setup);

static unsigned pmem_audio_size = MSM_PMEM_AUDIO_SIZE;
static int __init pmem_audio_size_setup(char *p)
{
	pmem_audio_size = memparse(p, NULL);
	return 0;
}
early_param("pmem_audio_size", pmem_audio_size_setup);

static struct memtype_reserve msm7x30_reserve_table[] __initdata = {
	[MEMTYPE_SMI] = {
	},
	[MEMTYPE_EBI0] = {
		.flags	=	MEMTYPE_FLAGS_1M_ALIGN,
	},
	[MEMTYPE_EBI1] = {
		.flags	=	MEMTYPE_FLAGS_1M_ALIGN,
	},
};

static void __init size_pmem_device(struct android_pmem_platform_data *pdata, unsigned long start, unsigned long size)
{
	pdata->start = start;
	pdata->size = size;
	if (pdata->start)
		pr_info("%s: pmem %s requests %lu bytes at 0x%p (0x%lx physical).\r\n",
			__func__, pdata->name, size, __va(start), start);
	else
		pr_info("%s: pmem %s requests %lu bytes dynamically.\r\n",
			__func__, pdata->name, size);
}
static void __init size_pmem_devices(void)
{
#ifdef CONFIG_ANDROID_PMEM
	size_pmem_device(&android_pmem_adsp_pdata, 0, pmem_adsp_size);
	size_pmem_device(&android_pmem_audio_pdata, 0, pmem_audio_size);
	size_pmem_device(&android_pmem_pdata, 0, pmem_sf_size);
	msm7x30_reserve_table[MEMTYPE_EBI1].size += PMEM_KERNEL_EBI1_SIZE;
#endif
}

static void __init reserve_memory_for(struct android_pmem_platform_data *p)
{
	if (p->start == 0) {
		pr_info("%s: reserve %lu bytes from memory %d for %s.\r\n", __func__, p->size, p->memory_type, p->name);
		msm7x30_reserve_table[p->memory_type].size += p->size;
	}
}

static void __init reserve_pmem_memory(void)
{
#ifdef CONFIG_ANDROID_PMEM
	reserve_memory_for(&android_pmem_adsp_pdata);
	reserve_memory_for(&android_pmem_audio_pdata);
	reserve_memory_for(&android_pmem_pdata);
#endif
}

static void __init msm7x30_calculate_reserve_sizes(void)
{
	size_pmem_devices();
	reserve_pmem_memory();
}

static int msm7x30_paddr_to_memtype(unsigned int paddr)
{
	if (paddr < 0x40000000)
		return MEMTYPE_EBI1;
	if (paddr >= 0x40000000 && paddr < 0x80000000)
		return MEMTYPE_EBI1;
	return MEMTYPE_NONE;
}

static struct reserve_info msm7x30_reserve_info __initdata = {
	.memtype_reserve_table = msm7x30_reserve_table,
	.calculate_reserve_sizes = msm7x30_calculate_reserve_sizes,
	.paddr_to_memtype = msm7x30_paddr_to_memtype,
};

static void __init mecha_reserve(void)
{
	reserve_info = &msm7x30_reserve_info;
	msm_reserve();
}

static void __init mecha_allocate_memory_regions(void)
{
	unsigned long size;

	size = MSM_FB_SIZE;
	msm_fb_resources[0].start = MSM_FB_BASE;
	msm_fb_resources[0].end = msm_fb_resources[0].start + size - 1;
	pr_info("allocating %lu bytes at 0x%p (0x%lx physical) for fb\n",
		size, __va(MSM_FB_BASE), (unsigned long) MSM_FB_BASE);
}

static void __init mecha_map_io(void)
{
	msm_shared_ram_phys = 0x00400000;
	msm_map_msm7x30_io();
	if (socinfo_init() < 0)
		printk(KERN_ERR "%s: socinfo_init() failed!\n",
		       __func__);
}

static void __init mecha_init_early(void)
{
	mecha_allocate_memory_regions();
}

static void __init mecha_fixup(struct machine_desc *desc, struct tag *tags,
								char **cmdline, struct meminfo *mi)
{
	engineerid = parse_tag_engineerid((const struct tag *)tags);
	printk(KERN_INFO "[%s] engineerid=0x%x\n", __func__, engineerid);

	smi_sz = parse_tag_smi((const struct tag *)tags);
	printk(KERN_INFO "[%s] smi_sz=%d\n", __func__, smi_sz);

	hwid = parse_tag_hwid((const struct tag *)tags);
	printk(KERN_INFO "[%s] hwid=0x%x\n", __func__, hwid);

	skuid = parse_tag_skuid((const struct tag *)tags);
	printk(KERN_INFO "[%s] skuid=0x%x\n", __func__, skuid);

//	engineerid = parse_tag_engineerid(tags);

	mi->nr_banks = 2;
	mi->bank[0].start = MSM_LINUX_BASE1;
	mi->bank[0].size = MSM_LINUX_SIZE1 + MSM_MEM_256MB_OFFSET;
	mi->bank[1].start = MSM_LINUX_BASE2;
	mi->bank[1].size = MSM_LINUX_SIZE2;
}

MACHINE_START(MECHA, "mecha")
	.fixup = mecha_fixup,
	.map_io = mecha_map_io,
	.reserve = mecha_reserve,
	.init_irq = mecha_init_irq,
	.init_machine = mecha_init,
	.timer = &msm_timer,
	.init_early = mecha_init_early,
MACHINE_END
