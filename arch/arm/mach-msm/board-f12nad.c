/* Copyright (c) 2009-2012, Code Aurora Forum. All rights reserved.
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
/*----------------------------------------------------------------------------*/
// COPYRIGHT(C) FUJITSU LIMITED 2011-2012
/*----------------------------------------------------------------------------*/

#include <linux/kernel.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
// Fujitsu 2012.01.06 WLAN/BT add start
#include <linux/wifi_tiwlan.h>
// Fujitsu 2012.01.06 WLAN/BT add end
#include <linux/delay.h>
#include <linux/bootmem.h>
#include <linux/io.h>
#ifdef CONFIG_SPI_QSD
#include <linux/spi/spi.h>
#endif
#include <linux/msm_ssbi.h>
#include <linux/mfd/pmic8058.h>
// Fujitsu 2012.01.06 WLAN/BT change start
#include <linux/mfd/pm8xxx/core.h>
#include <linux/mfd/pm8xxx/misc.h>
// Fujitsu 2012.01.06 WLAN/BT change end
#include <linux/leds.h>
#include <linux/mfd/marimba.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/smsc911x.h>
#include <linux/ofn_atlab.h>
#include <linux/power_supply.h>
#include <linux/i2c/isa1200.h>
#include <linux/i2c/tsc2007.h>
#include <linux/input/kp_flip_switch.h>
#include <linux/leds-pmic8058.h>
#include <linux/input/cy8c_ts.h>
#include <linux/msm_adc.h>
#include <linux/dma-mapping.h>
#include <linux/regulator/consumer.h>
/* FUJITSU:2011-12-01 add bsp-sensor start */
#include <linux/walkmotion.h>
/* FUJITSU:2012-03-28 NAD change start */
///* FUJITSU:2011-12-01 add bsp-sensor end */
///* FUJITSU:2012-01-16 bsp-sensor add start */
//#include <linux/compassdriver.h>
///* FUJITSU:2012-01-16 bsp-sensor add end */
///* FUJITSU 2011-12-22 add start */
#include <linux/hscdtd002a.h>
/* FUJITSU:2012-03-28 NAD change end */
#include <linux/input/touch_platform.h>
/* FUJITSU 2011-12-22 add end */


#include <asm/mach-types.h>
#include <asm/mach/arch.h>
#include <asm/setup.h>

#include <mach/mpp.h>
#include <mach/board.h>
#include <mach/camera.h>
#include <mach/memory.h>
#include <mach/msm_iomap.h>
#include <mach/msm_hsusb.h>
#include <mach/rpc_hsusb.h>
#include <mach/msm_spi.h>
#include <mach/qdsp5v2/msm_lpa.h>
#include <mach/dma.h>
// Fujitsu 2012.01.06 WLAN/BT add start
#include <mach/gpio.h>
// Fujitsu 2012.01.06 WLAN/BT add end
#include <linux/android_pmem.h>
#include <linux/input/msm_ts.h>
#include <mach/pmic.h>
#include <mach/rpc_pmapp.h>
#include <mach/qdsp5v2/aux_pcm.h>
#include <mach/qdsp5v2/mi2s.h>
#include <mach/qdsp5v2/audio_dev_ctl.h>
#include <mach/msm_battery.h>
#include <mach/rpc_server_handset.h>
#include <mach/msm_tsif.h>
#include <mach/socinfo.h>
#include <mach/msm_memtypes.h>
/* FUJITSU:2011-12-22 del cyttsp start */
#if 0
#include <linux/cyttsp.h>
#endif
/* FUJITSU:2011-12-22 del cyttsp end */

// Fujitsu 2012.01.06 WLAN/BT add start
#include <linux/mfd/pmic8058.h>
// Fujitsu 2012.01.06 WLAN/BT add end
#include <asm/mach/mmc.h>
#include <asm/mach/flash.h>
#include <mach/vreg.h>
#include <linux/platform_data/qcom_crypto_device.h>

#include "devices.h"
#include "timer.h"
/* FUJITSU:2011-12-01 add keypad start */
#include "msm-keypad-devices.h"
/* FUJITSU:2011-12-01 add keypad end */
#ifdef CONFIG_USB_G_ANDROID
#include <linux/usb/android.h>
#include <mach/usbdiag.h>
/* FUJITSU:2011-12-01 add smem start */
#include "proc_comm.h"
/* FUJITSU:2011-12-01 add smem end */
#endif
#include "pm.h"
#include "pm-boot.h"
#include "spm.h"
#include "acpuclock.h"
#include <mach/dal_axi.h>
#include <mach/msm_serial_hs.h>
#include <mach/qdsp5v2/mi2s.h>
#include <mach/qdsp5v2/audio_dev_ctl.h>
#include <mach/sdio_al.h>
#include "smd_private.h"
/* FUJITSU:2011-12-01 del BMA150 start */
#ifdef CONFIG_BOSCH_BMA150
#include <linux/bma150.h>
#endif
/* FUJITSU:2011-12-01 del BMA150 end */

#include "board-msm7x30-regulator.h"
#include "pm.h"

// Fujitsu 2012.01.06 WLAN/BT add start
/* FUJITSU:2011-05-06 add BT HS start */
#define MSM_UARTDM2_USE
#define MSM_SPI_CRCI_DISABLE
/* FUJITSU:2011-05-06 add BT HS end */
// Fujitsu 2012.01.06 WLAN/BT add end

/* FUJITSU:2012-02-28 DISP change pmem size start */
//#define MSM_PMEM_SF_SIZE	0x1700000
#define MSM_PMEM_SF_SIZE	0x2000000
/* FUJITSU:2012-02-28 DISP change pmem size end */
#ifdef CONFIG_FB_MSM_TRIPLE_BUFFER
#define MSM_FB_SIZE            0x780000
#else
#define MSM_FB_SIZE            0x500000
#endif
/* FUJITSU:2012-01-31 modify pmem adsp size change start */
//#define MSM_PMEM_ADSP_SIZE    0x1E00000
#define MSM_PMEM_ADSP_SIZE      0x2300000
/* FUJITSU:2012-01-31 modify pmem adsp size change end */
#define MSM_FLUID_PMEM_ADSP_SIZE	0x2800000
#define PMEM_KERNEL_EBI0_SIZE   0x600000
#define MSM_PMEM_AUDIO_SIZE     0x200000

#define PMIC_GPIO_INT		27
#define PMIC_VREG_WLAN_LEVEL	2900

/* FUJITSU:2012-03-15 touchscreen start */
#include "board-touch-f12nad.c"
/* FUJITSU:2012-03-15 touchscreen end */

/* FUJITSU:2011-12-01 modify SD_DET GPIO port start */
#if 0
#define PMIC_GPIO_SD_DET	36
#else
#define PMIC_GPIO_SD_DET	31
#endif
/* FUJITSU:2011-12-01 modify SD_DET GPIO port end */
#define PMIC_GPIO_SDC4_EN_N	17  /* PMIC GPIO Number 18 */
/* FUJITSU:2011-12-01 del DTV start */

/* FUJITSU:2011-12-22 add LED start */
#if defined(CONFIG_MACH_F11SKY) || defined(CONFIG_MACH_F12ACE) || defined(CONFIG_MACH_F12NAD)
#define PMIC_GPIO_KEYLIGHT		11  /* PMIC GPIO Number 12 */
#define PMIC_GPIO_MOBILELIGHT	24  /* PMIC GPIO Number 25 */
#endif
/* FUJITSU:2011-12-22 add LED end */


#if 0
#define PMIC_GPIO_HDMI_5V_EN_V3 32  /* PMIC GPIO for V3 H/W */
#define PMIC_GPIO_HDMI_5V_EN_V2 39 /* PMIC GPIO for V2 H/W */
#endif
/* FUJITSU:2011-12-01 del DTV end */

/* FUJITSU:2011-12-01 del BMA150 start */
#if 0
#define ADV7520_I2C_ADDR	0x39
#endif
/* FUJITSU:2011-12-01 del BMA150 end */

#define FPGA_SDCC_STATUS       0x8E0001A8

/* FUJITSU:2011-12-01 del OPTNAV start */
#if 0
#define FPGA_OPTNAV_GPIO_ADDR	0x8E000026
#define OPTNAV_I2C_SLAVE_ADDR	(0xB0 >> 1)
#define OPTNAV_IRQ		20
#define OPTNAV_CHIP_SELECT	19
#endif
/* FUJITSU:2011-12-01 del OPTNAV end */
#define PMIC_GPIO_SDC4_PWR_EN_N 24  /* PMIC GPIO Number 25 */

/* FUJITSU:2011-12-22 add BL SLAVE ADDR start */
#define LED_I2C_SLAVE_ADDR	0x76
/* FUJITSU:2011-12-22 add BL SLAVE ADDR end */

/* Macros assume PMIC GPIOs start at 0 */
#define PM8058_GPIO_PM_TO_SYS(pm_gpio)     (pm_gpio + NR_GPIO_IRQS)
#define PM8058_GPIO_SYS_TO_PM(sys_gpio)    (sys_gpio - NR_GPIO_IRQS)
#define PM8058_MPP_BASE			   PM8058_GPIO_PM_TO_SYS(PM8058_GPIOS)
#define PM8058_MPP_PM_TO_SYS(pm_gpio)	   (pm_gpio + PM8058_MPP_BASE)

#define PMIC_GPIO_FLASH_BOOST_ENABLE	15	/* PMIC GPIO Number 16 */
/* FUJITSU:2011-12-22 mod GPIO start */
#define PMIC_GPIO_HAP_ENABLE   14  /* PMIC GPIO Number 15 */
/* FUJITSU:2011-12-22 mod GPIO end */

/* FUJITSU:2011-12-01 del WLAN_EXT_POR start */
#if 0
#define PMIC_GPIO_WLAN_EXT_POR  22 /* PMIC GPIO NUMBER 23 */
#endif
/* FUJITSU:2011-12-01 del WLAN_EXT_POR end */

/* FUJITSU:2011-12-01 del BMA150 start */
#if 0
#define BMA150_GPIO_INT 1
#endif
/* FUJITSU:2011-12-01 del BMA150 end */

#define HAP_LVL_SHFT_MSM_GPIO 24

#define PMIC_GPIO_QUICKVX_CLK 37 /* PMIC GPIO 38 */

/* FUJITSU:2012-01-13 factory start */
#define FACTORY_DEFAULT_VALUE 0
static int factory_value = FACTORY_DEFAULT_VALUE;
void factory_read_nvitem_init(void);
#define NV_BOOT_TESTMODE_STATUS_I   10145
#define NV_BOOT_NONE                            0 /* normal boot */
#define NV_BOOT_TESTMODE                        1 /* testmode */
#define NV_BOOT_KERNEL_TEST                     2 /* kernel-testmode */
#define NV_BOOT_ANDROID_TEST            3 /* android-testmode */
/* FUJITSU:2012-01-13 factory end */

/* FUJITSU:2011-12-01 add VIB start */
void msm_init_pmic_vibrator(void);
/* FUJITSU:2011-12-01 add VIB end */

/* FUJITSU:2011-12-01 spi gpio setting start */
int set_spi_gpio_use_state = 0;
EXPORT_SYMBOL(set_spi_gpio_use_state);
static DEFINE_SPINLOCK(set_spi_gpio_use_state_lock);
/* FUJITSU:2011-12-01 spi gpio setting end */

#define	PM_FLIP_MPP 5 /* PMIC MPP 06 */

struct pm8xxx_gpio_init_info {
	unsigned			gpio;
	struct pm_gpio			config;
};

static int pm8058_gpios_init(void)
{
	int rc;

	struct pm8xxx_gpio_init_info sdc4_en = {
		PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_SDC4_EN_N),
		{
			.direction      = PM_GPIO_DIR_OUT,
			.pull           = PM_GPIO_PULL_NO,
			.vin_sel        = PM8058_GPIO_VIN_L5,
			.function       = PM_GPIO_FUNC_NORMAL,
			.inv_int_pol    = 0,
			.out_strength   = PM_GPIO_STRENGTH_LOW,
			.output_value   = 0,
		},
	};

	struct pm8xxx_gpio_init_info sdc4_pwr_en = {
		PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_SDC4_PWR_EN_N),
		{
			.direction      = PM_GPIO_DIR_OUT,
			.pull           = PM_GPIO_PULL_NO,
			.vin_sel        = PM8058_GPIO_VIN_L5,
			.function       = PM_GPIO_FUNC_NORMAL,
			.inv_int_pol    = 0,
			.out_strength   = PM_GPIO_STRENGTH_LOW,
			.output_value   = 0,
		},
	};

	struct pm8xxx_gpio_init_info haptics_enable = {
		PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_HAP_ENABLE),
		{
			.direction      = PM_GPIO_DIR_OUT,
			.pull           = PM_GPIO_PULL_NO,
			.out_strength   = PM_GPIO_STRENGTH_HIGH,
			.function       = PM_GPIO_FUNC_NORMAL,
			.inv_int_pol    = 0,
			.vin_sel        = 2,
			.output_buffer  = PM_GPIO_OUT_BUF_CMOS,
			.output_value   = 0,
		},
	};

/* FUJITSU:2011-12-01 del DTV start */
#if 0
	struct pm8xxx_gpio_init_info hdmi_5V_en = {
		PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_HDMI_5V_EN_V3),
		{
			.direction      = PM_GPIO_DIR_OUT,
			.pull           = PM_GPIO_PULL_NO,
			.vin_sel        = PM8058_GPIO_VIN_VPH,
			.function       = PM_GPIO_FUNC_NORMAL,
			.out_strength   = PM_GPIO_STRENGTH_LOW,
			.output_value   = 0,
		},
	};
#endif
/* FUJITSU:2011-12-01 del DTV end */

	struct pm8xxx_gpio_init_info flash_boost_enable = {
		PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_FLASH_BOOST_ENABLE),
		{
			.direction      = PM_GPIO_DIR_OUT,
			.output_buffer  = PM_GPIO_OUT_BUF_CMOS,
			.output_value   = 0,
			.pull           = PM_GPIO_PULL_NO,
			.vin_sel        = PM8058_GPIO_VIN_S3,
			.out_strength   = PM_GPIO_STRENGTH_HIGH,
			.function        = PM_GPIO_FUNC_2,
		},
	};
/* FUJITSU:2011-12-01 del WLAN_EXT_POR start */
#if 0
	struct pm8xxx_gpio_init_info gpio23 = {
		PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_WLAN_EXT_POR),
		{
			.direction      = PM_GPIO_DIR_OUT,
			.output_buffer  = PM_GPIO_OUT_BUF_CMOS,
			.output_value   = 0,
			.pull           = PM_GPIO_PULL_NO,
			.vin_sel        = 2,
			.out_strength   = PM_GPIO_STRENGTH_LOW,
			.function       = PM_GPIO_FUNC_NORMAL,
		}
	};
#endif
/* FUJITSU:2011-12-01 del WLAN_EXT_POR end */

/* FUJITSU:2011-12-22 add lights start */
	struct pm8xxx_gpio_init_info keybacklight = {
		PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_KEYLIGHT),
		{
			.direction      = PM_GPIO_DIR_OUT,
			.pull           = PM_GPIO_PULL_NO,
			.vin_sel        = PM8058_GPIO_VIN_S3,
			.function       = PM_GPIO_FUNC_NORMAL,
			.inv_int_pol    = 0,
			.output_value   = 0,
			.out_strength   = PM_GPIO_STRENGTH_HIGH,
		},
	};

	struct pm8xxx_gpio_init_info mobilelight = {
		PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_MOBILELIGHT),
		{
			.direction      = PM_GPIO_DIR_OUT,
			.pull           = PM_GPIO_PULL_NO,
			.vin_sel        = PM8058_GPIO_VIN_S3,
			.function       = PM_GPIO_FUNC_NORMAL,
			.inv_int_pol    = 0,
			.output_value   = 0,
			.out_strength   = PM_GPIO_STRENGTH_HIGH,
		},
	};
/* FUJITSU:2011-12-22 add lights end */

#ifdef CONFIG_MMC_MSM_CARD_HW_DETECTION
	struct pm8xxx_gpio_init_info sdcc_det = {
		PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_SD_DET - 1),
		{
			.direction      = PM_GPIO_DIR_IN,
			.pull           = PM_GPIO_PULL_UP_1P5,
			.vin_sel        = 2,
			.function       = PM_GPIO_FUNC_NORMAL,
			.inv_int_pol    = 0,
		},
	};

	if (machine_is_msm7x30_fluid())
		sdcc_det.config.inv_int_pol = 1;

	rc = pm8xxx_gpio_config(sdcc_det.gpio, &sdcc_det.config);
	if (rc) {
		pr_err("%s PMIC_GPIO_SD_DET config failed\n", __func__);
		return rc;
	}
#endif

/* FUJITSU:2011-12-01 del DTV start */
#if 0
	if (machine_is_msm8x55_svlte_surf() || machine_is_msm8x55_svlte_ffa() ||
						machine_is_msm7x30_fluid())
		hdmi_5V_en.gpio = PMIC_GPIO_HDMI_5V_EN_V2;
	else
		hdmi_5V_en.gpio = PMIC_GPIO_HDMI_5V_EN_V3;

	hdmi_5V_en.gpio = PM8058_GPIO_PM_TO_SYS(hdmi_5V_en.gpio);

	rc = pm8xxx_gpio_config(hdmi_5V_en.gpio, &hdmi_5V_en.config);
	if (rc) {
		pr_err("%s PMIC_GPIO_HDMI_5V_EN config failed\n", __func__);
		return rc;
	}
#endif
/* FUJITSU:2011-12-01 del DTV end */
/* FUJITSU:2011-12-01 del WLAN_EXT_POR start */
#if 0
	/* Deassert GPIO#23 (source for Ext_POR on WLAN-Volans) */
	rc = pm8xxx_gpio_config(gpio23.gpio, &gpio23.config);
	if (rc) {
		pr_err("%s PMIC_GPIO_WLAN_EXT_POR config failed\n", __func__);
		return rc;
	}
#endif
/* FUJITSU:2011-12-01 del WLAN_EXT_POR end */

	if (machine_is_msm7x30_fluid()) {
		/* Haptics gpio */
		rc = pm8xxx_gpio_config(haptics_enable.gpio,
						&haptics_enable.config);
		if (rc) {
			pr_err("%s: PMIC GPIO %d write failed\n", __func__,
							haptics_enable.gpio);
			return rc;
		}
		/* Flash boost gpio */
		rc = pm8xxx_gpio_config(flash_boost_enable.gpio,
						&flash_boost_enable.config);
		if (rc) {
			pr_err("%s: PMIC GPIO %d write failed\n", __func__,
						flash_boost_enable.gpio);
			return rc;
		}
		/* SCD4 gpio */
		rc = pm8xxx_gpio_config(sdc4_en.gpio, &sdc4_en.config);
		if (rc) {
			pr_err("%s PMIC_GPIO_SDC4_EN_N config failed\n",
								 __func__);
			return rc;
		}
		rc = gpio_request(sdc4_en.gpio, "sdc4_en");
		if (rc) {
			pr_err("%s PMIC_GPIO_SDC4_EN_N gpio_request failed\n",
				__func__);
			return rc;
		}
		gpio_set_value_cansleep(sdc4_en.gpio, 0);
	}
	/* FFA -> gpio_25 controls vdd of sdcc4 */
	else {
		/* SCD4 gpio_25 */
		rc = pm8xxx_gpio_config(sdc4_pwr_en.gpio, &sdc4_pwr_en.config);
		if (rc) {
			pr_err("%s PMIC_GPIO_SDC4_PWR_EN_N config failed: %d\n",
			       __func__, rc);
			return rc;
		}

		rc = gpio_request(sdc4_pwr_en.gpio, "sdc4_pwr_en");
		if (rc) {
			pr_err("PMIC_GPIO_SDC4_PWR_EN_N gpio_req failed: %d\n",
			       rc);
			return rc;
		}
	}

/* FUJITSU:2011-12-22 add lights start */
	rc = pm8xxx_gpio_config(keybacklight.gpio, &keybacklight.config);
	if (rc) {
		pr_err("%s PMIC_GPIO_KEYLIGHT config failed\n", __func__);
		return rc;
	}
	rc = pm8xxx_gpio_config(mobilelight.gpio, &mobilelight.config);
	if (rc) {
		pr_err("%s PMIC_GPIO_MOBILELIGHT config failed\n", __func__);
		return rc;
	}
/* FUJITSU:2011-12-22 add lights end */

	return 0;
}

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

/* FUJITSU:2011-12-01  del tma300 start */
#if 0
/*virtual key support */
static ssize_t tma300_vkeys_show(struct kobject *kobj,
			struct kobj_attribute *attr, char *buf)
{
	return sprintf(buf,
	__stringify(EV_KEY) ":" __stringify(KEY_BACK) ":50:842:80:100"
	":" __stringify(EV_KEY) ":" __stringify(KEY_MENU) ":170:842:80:100"
	":" __stringify(EV_KEY) ":" __stringify(KEY_HOME) ":290:842:80:100"
	":" __stringify(EV_KEY) ":" __stringify(KEY_SEARCH) ":410:842:80:100"
	"\n");
}

static struct kobj_attribute tma300_vkeys_attr = {
	.attr = {
		.mode = S_IRUGO,
	},
	.show = &tma300_vkeys_show,
};

static struct attribute *tma300_properties_attrs[] = {
	&tma300_vkeys_attr.attr,
	NULL
};

static struct attribute_group tma300_properties_attr_group = {
	.attrs = tma300_properties_attrs,
};

static struct kobject *properties_kobj;
#endif
/* FUJITSU:2011-12-01  del tma300 end */

/* FUJITSU:2011-12-01 del cyttsp start */
#if 0
static struct regulator_bulk_data cyttsp_regs[] = {
	{ .supply = "ldo8",  .min_uV = 1800000, .max_uV = 1800000 },
	{ .supply = "ldo15", .min_uV = 3050000, .max_uV = 3100000 },
};

#define CYTTSP_TS_GPIO_IRQ	150
static int cyttsp_platform_init(struct i2c_client *client)
{
	int rc = -EINVAL;

	rc = regulator_bulk_get(NULL, ARRAY_SIZE(cyttsp_regs), cyttsp_regs);

	if (rc) {
		pr_err("%s: could not get regulators: %d\n", __func__, rc);
		goto out;
	}

	rc = regulator_bulk_set_voltage(ARRAY_SIZE(cyttsp_regs), cyttsp_regs);

	if (rc) {
		pr_err("%s: could not set regulator voltages: %d\n", __func__,
				rc);
		goto regs_free;
	}

	rc = regulator_bulk_enable(ARRAY_SIZE(cyttsp_regs), cyttsp_regs);

	if (rc) {
		pr_err("%s: could not enable regulators: %d\n", __func__, rc);
		goto regs_free;
	}

	/* check this device active by reading first byte/register */
	rc = i2c_smbus_read_byte_data(client, 0x01);
	if (rc < 0) {
		pr_err("%s: i2c sanity check failed\n", __func__);
		goto regs_disable;
	}

	rc = gpio_tlmm_config(GPIO_CFG(CYTTSP_TS_GPIO_IRQ, 0, GPIO_CFG_INPUT,
					GPIO_CFG_PULL_UP, GPIO_CFG_6MA), GPIO_CFG_ENABLE);
	if (rc) {
		pr_err("%s: Could not configure gpio %d\n",
					 __func__, CYTTSP_TS_GPIO_IRQ);
		goto regs_disable;
	}

	/* virtual keys */
	tma300_vkeys_attr.attr.name = "virtualkeys.cyttsp-i2c";
	properties_kobj = kobject_create_and_add("board_properties",
				NULL);
	if (properties_kobj)
		rc = sysfs_create_group(properties_kobj,
			&tma300_properties_attr_group);
	if (!properties_kobj || rc)
		pr_err("%s: failed to create board_properties\n",
				__func__);

	return CY_OK;

regs_disable:
	regulator_bulk_disable(ARRAY_SIZE(cyttsp_regs), cyttsp_regs);
regs_free:
	regulator_bulk_free(ARRAY_SIZE(cyttsp_regs), cyttsp_regs);
out:
	return rc;
}

/* TODO: Put the regulator to LPM / HPM in suspend/resume*/
static int cyttsp_platform_suspend(struct i2c_client *client)
{
	msleep(20);

	return CY_OK;
}

static int cyttsp_platform_resume(struct i2c_client *client)
{
	/* add any special code to strobe a wakeup pin or chip reset */
	mdelay(10);

	return CY_OK;
}

static struct cyttsp_platform_data cyttsp_data = {
	.fw_fname = "cyttsp_7630_fluid.hex",
	.panel_maxx = 479,
	.panel_maxy = 799,
	.disp_maxx = 469,
	.disp_maxy = 799,
	.disp_minx = 10,
	.disp_miny = 0,
	.flags = 0,
	.gen = CY_GEN3,	/* or */
	.use_st = CY_USE_ST,
	.use_mt = CY_USE_MT,
	.use_hndshk = CY_SEND_HNDSHK,
	.use_trk_id = CY_USE_TRACKING_ID,
	.use_sleep = CY_USE_DEEP_SLEEP_SEL | CY_USE_LOW_POWER_SEL,
	.use_gestures = CY_USE_GESTURES,
	/* activate up to 4 groups
	 * and set active distance
	 */
	.gest_set = CY_GEST_GRP1 | CY_GEST_GRP2 |
				CY_GEST_GRP3 | CY_GEST_GRP4 |
				CY_ACT_DIST,
	/* change act_intrvl to customize the Active power state
	 * scanning/processing refresh interval for Operating mode
	 */
	.act_intrvl = CY_ACT_INTRVL_DFLT,
	/* change tch_tmout to customize the touch timeout for the
	 * Active power state for Operating mode
	 */
	.tch_tmout = CY_TCH_TMOUT_DFLT,
	/* change lp_intrvl to customize the Low Power power state
	 * scanning/processing refresh interval for Operating mode
	 */
	.lp_intrvl = CY_LP_INTRVL_DFLT,
	.resume = cyttsp_platform_resume,
	.suspend = cyttsp_platform_suspend,
	.init = cyttsp_platform_init,
	.sleep_gpio = -1,
	.resout_gpio = -1,
	.irq_gpio = CYTTSP_TS_GPIO_IRQ,
	.correct_fw_ver = 2,
};
#endif
/* FUJITSU:2011-12-01 del cyttsp end */

/* FUJITSU:2012-03-15 CYPRESS TOUCH PRESS add start */
static struct regulator_bulk_data cyttsp_press_regs[] = {
    { .supply = "ldo12", .min_uV = 1800000, .max_uV = 1800000},
};

static uint32_t cyttsp_press_i2c_gpio_table[] = {
    GPIO_CFG(CY_PR_I2C_XRST_GPIO, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),  /* RESET */
    GPIO_CFG(CY_PR_I2C_IRQ_GPIO,  0, GPIO_CFG_INPUT,  GPIO_CFG_PULL_UP, GPIO_CFG_2MA),  /* INTERRUPT */
};

static struct touch_settings cyttsp_press_sett_op_regs = {
    .data = NULL,
    .size = 0,
    .tag = 0,
};

static struct touch_settings cyttsp_press_sett_si_regs = {
    .data = NULL,
    .size = 0,
    .tag = 0,
};

static struct touch_settings cyttsp_press_sett_bl_keys = {
    .data = NULL,
    .size = 0,
    .tag = 0,
};

static struct touch_firmware cyttsp_press_firmware = {
    .img = NULL,
    .size = 0,
    .ver = NULL,
    .vsize = 0,
};

static const uint16_t cyttsp_press_abs[] = {
    ABS_MT_PRESSURE, CY_ABS_MIN_P, CY_ABS_MAX_P, 0, 0,
    ABS_MT_ORIENTATION, CY_ABS_MIN_P, CY_ABS_MAX_P, 0, 0,
};

struct touch_framework cyttsp_press_framework = {
    .abs = (uint16_t *)&cyttsp_press_abs[0],
    .size = ARRAY_SIZE(cyttsp_press_abs),
    .enable_vkeys = 0,
};

static int cyttsp_press_hw_reset(void)
{
    /* Reset Lo */
    gpio_set_value(CY_PR_I2C_XRST_GPIO, 0);

    /* 1ms wait reset Lo->Hi */
    mdelay(1);

    /* Reset Hi */
    gpio_set_value(CY_PR_I2C_XRST_GPIO, 1);

    /* 1ms wait reset Lo->Hi */
    mdelay(1);

    /* Reset Lo */
    gpio_set_value(CY_PR_I2C_XRST_GPIO, 0);
    mdelay(40);

    return 0;
}

static int cyttsp_press_hw_recov(int on)
{
    return 0;
}

static int cyttsp_press_hw_power(int on)
{
    int rc = 0;

    rc = on ?
        regulator_bulk_enable(ARRAY_SIZE(cyttsp_press_regs), cyttsp_press_regs) :
        regulator_bulk_disable(ARRAY_SIZE(cyttsp_press_regs), cyttsp_press_regs);

    if (rc) {
        pr_err("[PPD-board]%s:%d could not %sable regulators: %d\n",
                __func__, __LINE__, on ? "en" : "dis", rc);
    } else {
        pr_info("[PPD-board]%s:%d regulators %sable : %d\n",
                __func__, __LINE__, on ? "en" : "dis", rc);
    }
    mdelay(5);
    return rc;
}

static int cyttsp_press_hw_irq_stat(void)
{
    return gpio_get_value(CY_PR_I2C_IRQ_GPIO);
}

static int cyttsp_press_hw_i2c_init(int on)
{
    int     ret = 0;
    int     cnt = 0;

    if (on) {
        ret = regulator_bulk_get(NULL, ARRAY_SIZE(cyttsp_press_regs), cyttsp_press_regs);
        if (ret) {
            pr_err("[PPD-board]%s:%d could not get regulators: %d\n", __func__, __LINE__, ret);
            goto out;
        }

        ret = regulator_bulk_set_voltage(ARRAY_SIZE(cyttsp_press_regs), cyttsp_press_regs);
        if (ret) {
            pr_err("[PPD-board]%s:%d could not set regulator voltages: %d\n", __func__, __LINE__, ret);
            goto regs_free;
        }

        /* request */
        ret = gpio_request(CY_PR_I2C_IRQ_GPIO, "cyttsp_press_i2c_irq");
        if (ret < 0) {
            printk(KERN_ERR "[PPD-board]%s: Failed to request GPIO %d\n", __func__, CY_PR_I2C_IRQ_GPIO);
            goto regs_free;
        }

        ret = gpio_request(CY_PR_I2C_XRST_GPIO, "cyttsp_press_i2c_rst");
        if (ret < 0) {
            printk(KERN_ERR "[PPD-board]%s: Failed to request GPIO %d\n", __func__, CY_PR_I2C_XRST_GPIO);
            goto regs_free;
        }

        /* gpio config */
        for (cnt = 0; cnt < ARRAY_SIZE(cyttsp_press_i2c_gpio_table); cnt++) {
            ret = gpio_tlmm_config(cyttsp_press_i2c_gpio_table[cnt], GPIO_CFG_ENABLE);
            if (ret < 0) {
                printk(KERN_ERR "[PPD-board]%s: gpio_tlmm_config(%#x)=%d\n", __func__, cyttsp_press_i2c_gpio_table[cnt], ret);
            }
        }
    } else {
        gpio_free(CY_PR_I2C_IRQ_GPIO);
        gpio_free(CY_PR_I2C_XRST_GPIO);
        goto regs_disable;
    }
    goto out;

regs_disable:
    regulator_bulk_disable(ARRAY_SIZE(cyttsp_press_regs), cyttsp_press_regs);
regs_free:
    regulator_bulk_free(ARRAY_SIZE(cyttsp_press_regs), cyttsp_press_regs);
out:
    return ret;
}

struct touch_platform_data cyttsp_press_platform_data = {
    .sett = {
        NULL,
        &cyttsp_press_sett_op_regs,
        &cyttsp_press_sett_si_regs,
        &cyttsp_press_sett_bl_keys,
    },
    .fw = &cyttsp_press_firmware,
    .frmwrk = &cyttsp_press_framework,
    .addr = {CY_PR_I2C_ADR, 0},
    .flags = 0x01 | 0x02,
    .hw_reset = cyttsp_press_hw_reset,
    .hw_recov = cyttsp_press_hw_recov,
    .hw_power = cyttsp_press_hw_power,
    .irq_stat = cyttsp_press_hw_irq_stat,
    .init     = cyttsp_press_hw_i2c_init
};
/* FUJITSU:2012-03-15 CYPRESS TOUCH PRESS add end */

/* FUJITSU:2011-12-22 add cyttsp start */
static uint32_t cyttsp_i2c_gpio_table[] = {
    GPIO_CFG( CY_I2C_XRST_GPIO, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /* RESET */
    GPIO_CFG( CY_I2C_IRQ_GPIO,  0, GPIO_CFG_INPUT,  GPIO_CFG_PULL_UP, GPIO_CFG_2MA)  /* INTERRUPT */
};

static struct touch_settings cyttsp_sett_op_regs = {
	.data = NULL,
	.size = 0,
	.tag = 0,
};

static struct touch_settings cyttsp_sett_si_regs = {
	.data = NULL,
	.size = 0,
	.tag = 0,
};

static struct touch_settings cyttsp_sett_bl_keys = {
	.data = NULL,
	.size = 0,
	.tag = 0,
};

static struct touch_firmware cyttsp_firmware = {
	.img = NULL,
	.size = 0,
	.ver = NULL,
	.vsize = 0,
};

static const uint16_t cyttsp_abs[] = {
	ABS_MT_POSITION_X, CY_ABS_MIN_X, CY_ABS_MAX_X, 0, 0,
	ABS_MT_POSITION_Y, CY_ABS_MIN_Y, CY_ABS_MAX_Y, 0, 0,
	ABS_MT_PRESSURE,   CY_ABS_MIN_P, CY_ABS_MAX_P, 0, 0,
	ABS_MT_TOUCH_MAJOR,CY_ABS_MIN_W, CY_ABS_MAX_W, 0, 0,
	/* ABS_MT_ORIENTATION */0, CY_ABS_MIN_T, CY_ABS_MAX_T, 0, 0,
};

struct touch_framework cyttsp_framework = {
	.abs = (uint16_t *)&cyttsp_abs[0],
	.size = sizeof(cyttsp_abs),
	.enable_vkeys = 0,
};

static int cyttsp_hw_reset(void)
{
//	printk("%s: IN\n", __func__);
	/* Reset Hi */
	gpio_set_value(CY_I2C_XRST_GPIO, 1);

	/* 1ms wait reset Lo->Hi */
	mdelay(1);

	/* Reset Lo */
	gpio_set_value(CY_I2C_XRST_GPIO, 0);

	/* 1ms wait reset Lo->Hi */
	mdelay(1);

	/* Reset Hi */
	gpio_set_value(CY_I2C_XRST_GPIO, 1);
	mdelay(40);

//	printk("%s: OUT\n", __func__);

	return 0;
}

static int cyttsp_hw_recov(int on)
{
//	printk("%s: IN\n", __func__);

	gpio_direction_output(CY_I2C_IRQ_GPIO, 1);

	/* Reset Lo change Reset->Int */
	gpio_set_value(CY_I2C_IRQ_GPIO, 0);

	mdelay(1);

	/* Reset Hi change Reset->Int */
	gpio_set_value(CY_I2C_IRQ_GPIO, 1);

	mdelay(1);

	gpio_set_value(CY_I2C_IRQ_GPIO, 0);

	mdelay(1);

	gpio_set_value(CY_I2C_IRQ_GPIO, 1);

	gpio_direction_input(CY_I2C_IRQ_GPIO);

//	printk("%s: OUT\n", __func__);

	return 0;
}

static int cyttsp_hw_power(int on)
{
	int ret;
	struct  i2c_msg msg;
	struct  i2c_adapter *i2c_bkl;
	u_int8_t buf[8];

	/* Get adpter */
	i2c_bkl = i2c_get_adapter(0);

	/* Set Param    */
	msg.addr    = LED_I2C_SLAVE_ADDR;
	msg.buf     = buf;
	msg.len     = 2;
	msg.flags   = 0;

	if (on) {
		printk("%s: Power ON\n", __func__);
		/* Set Voltage 14H(0xA4:2.8V+1.8V),15H(0xA4:2.8V+1.8V) */
		buf[0] = 0x15;  /* client addr     */
		buf[1] = 0xA4;  /* 15H: LDO4 LDO3  */

		ret = i2c_transfer(i2c_bkl, &msg, 1);
		if (ret < 0) {
		    printk("%s: I2C ERROR ret = %d\n", __func__, ret);
		    return ret;
		}

		mdelay(5);
		   
		/* Set Param    */
		/* Voltage ON  13H(0x0F)    */
		buf[0] = 0x13;  /* client addr     */
		buf[1] = 0x0F;  /* 13H: LDO1-4 ON  */  

		ret = i2c_transfer(i2c_bkl, &msg, 1);
		if (ret < 0) {
		    printk("%s: I2C ERROR ret = %d\n", __func__, ret);
		    return ret;
		}

		mdelay(10);
	} else {
		printk("%s: Power OFF\n", __func__);
		/* Set Param    */
		/* Voltage OFF  13H(0x03) */
		buf[0] = 0x13;  /* client addr */
		buf[1] = 0x03;  /* 13H: LDO3,4 OFF */ 

		ret = i2c_transfer(i2c_bkl, &msg, 1);
		if (ret < 0) {
		    printk("%s: I2C ERROR ret = %d\n", __func__, ret);
		    return ret;
		}

		mdelay(10);
	}

	return 0;
}

static int cyttsp_irq_stat(void)
{
	return gpio_get_value(CY_I2C_IRQ_GPIO);
}

static int cyttsp_i2c_init(int on)
{
    int     ret;
    int     cnt;
    struct  i2c_msg msg;
    struct  i2c_adapter *i2c_bkl;
    u_int8_t buf[8];

    if (on) {
        /* config */
        for (cnt = 0; cnt < ARRAY_SIZE(cyttsp_i2c_gpio_table); cnt++) {
            ret = gpio_tlmm_config(cyttsp_i2c_gpio_table[cnt], GPIO_CFG_ENABLE);
            if (ret < 0) {
                printk(KERN_ERR "%s: gpio_tlmm_config(%#x)=%d\n", __func__, cyttsp_i2c_gpio_table[cnt], ret );
            }
        }

        /* request */
   		ret = gpio_request(CY_I2C_IRQ_GPIO, "cyttsp_i2c_irq");
		if (ret < 0) {
			printk(KERN_ERR "%s: Failed to request GPIO %d\n", __func__, CY_I2C_IRQ_GPIO);
			return ret;
		}
   		ret = gpio_request(CY_I2C_XRST_GPIO, "cyttsp_i2c_rst");
		if (ret < 0) {
			printk(KERN_ERR "%s: Failed to request GPIO %d\n", __func__, CY_I2C_XRST_GPIO);
			return ret;
		}

        /* Get adpter */
		i2c_bkl = i2c_get_adapter(0);

        /* Set Param    */
        msg.addr    = LED_I2C_SLAVE_ADDR;
        msg.buf     = buf;
        msg.len     = 2;
        msg.flags   = 0;
    
        /* Set Voltage 14H(0xA4:2.8V+1.8V),15H(0xA4:2.8V+1.8V) */
        buf[0] = 0x15;  /* client addr     */
        buf[1] = 0xA4;  /* 15H: LDO4 LDO3  */

        ret = i2c_transfer(i2c_bkl, &msg, 1);
        if (ret < 0) {
            printk("%s: I2C ERROR ret = %d\n", __func__, ret);
            return ret;
        }

        mdelay(5);
           
        /* Set Param    */
        /* Voltage ON  13H(0x0C)    */
        buf[0] = 0x13;  /* client addr     */
        buf[1] = 0x0F;  /* 13H: LDO1-4 ON  */
 
        ret = i2c_transfer(i2c_bkl, &msg, 1);
        if (ret < 0) {
            printk("%s: I2C ERROR ret = %d\n", __func__, ret);
            return ret;
        }

		mdelay(20);

    } else {
		gpio_free(CY_I2C_IRQ_GPIO);
		gpio_free(CY_I2C_XRST_GPIO);
	}
	return 0;
}

struct touch_platform_data cypress_i2c_ttsp_platform_data = {
	.sett = {
		NULL,
		&cyttsp_sett_op_regs,
		&cyttsp_sett_si_regs,
		&cyttsp_sett_bl_keys,
	},
	.fw = &cyttsp_firmware,
	.frmwrk = &cyttsp_framework,
	.addr = {CY_I2C_ADR, 0},
	.flags = 0x01 | 0x02,
	.hw_reset = cyttsp_hw_reset,
	.hw_recov = cyttsp_hw_recov,
	.hw_power = cyttsp_hw_power,
	.irq_stat = cyttsp_irq_stat,
	.init     = cyttsp_i2c_init
};
/* FUJITSU:2011-12-22 add cyttsp end */

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

static const unsigned int fluid_keymap[] = {
	KEY(0, 0, KEY_7),
	KEY(0, 1, KEY_ENTER),
	KEY(0, 2, KEY_UP),
	/* drop (0,3) as it always shows up in pair with(0,2) */
	KEY(0, 4, KEY_DOWN),

	KEY(1, 0, KEY_CAMERA_SNAPSHOT),
	KEY(1, 1, KEY_SELECT),
	KEY(1, 2, KEY_1),
	KEY(1, 3, KEY_VOLUMEUP),
	KEY(1, 4, KEY_VOLUMEDOWN),
};

static const unsigned int surf_keymap[] = {
	KEY(0, 0, KEY_7),
	KEY(0, 1, KEY_DOWN),
	KEY(0, 2, KEY_UP),
	KEY(0, 3, KEY_RIGHT),
	KEY(0, 4, KEY_ENTER),
	KEY(0, 5, KEY_L),
	KEY(0, 6, KEY_BACK),
	KEY(0, 7, KEY_M),

	KEY(1, 0, KEY_LEFT),
	KEY(1, 1, KEY_SEND),
	KEY(1, 2, KEY_1),
	KEY(1, 3, KEY_4),
	KEY(1, 4, KEY_CLEAR),
	KEY(1, 5, KEY_MSDOS),
	KEY(1, 6, KEY_SPACE),
	KEY(1, 7, KEY_COMMA),

	KEY(2, 0, KEY_6),
	KEY(2, 1, KEY_5),
	KEY(2, 2, KEY_8),
	KEY(2, 3, KEY_3),
	KEY(2, 4, KEY_NUMERIC_STAR),
	KEY(2, 5, KEY_UP),
	KEY(2, 6, KEY_DOWN), /* SYN */
	KEY(2, 7, KEY_LEFTSHIFT),

	KEY(3, 0, KEY_9),
	KEY(3, 1, KEY_NUMERIC_POUND),
	KEY(3, 2, KEY_0),
	KEY(3, 3, KEY_2),
	KEY(3, 4, KEY_SLEEP),
	KEY(3, 5, KEY_F1),
	KEY(3, 6, KEY_F2),
	KEY(3, 7, KEY_F3),

	KEY(4, 0, KEY_BACK),
	KEY(4, 1, KEY_HOME),
	KEY(4, 2, KEY_MENU),
	KEY(4, 3, KEY_VOLUMEUP),
	KEY(4, 4, KEY_VOLUMEDOWN),
	KEY(4, 5, KEY_F4),
	KEY(4, 6, KEY_F5),
	KEY(4, 7, KEY_F6),

	KEY(5, 0, KEY_R),
	KEY(5, 1, KEY_T),
	KEY(5, 2, KEY_Y),
	KEY(5, 3, KEY_LEFTALT),
	KEY(5, 4, KEY_KPENTER),
	KEY(5, 5, KEY_Q),
	KEY(5, 6, KEY_W),
	KEY(5, 7, KEY_E),

	KEY(6, 0, KEY_F),
	KEY(6, 1, KEY_G),
	KEY(6, 2, KEY_H),
	KEY(6, 3, KEY_CAPSLOCK),
	KEY(6, 4, KEY_PAGEUP),
	KEY(6, 5, KEY_A),
	KEY(6, 6, KEY_S),
	KEY(6, 7, KEY_D),

	KEY(7, 0, KEY_V),
	KEY(7, 1, KEY_B),
	KEY(7, 2, KEY_N),
	KEY(7, 3, KEY_MENU), /* REVISIT - SYM */
	KEY(7, 4, KEY_PAGEDOWN),
	KEY(7, 5, KEY_Z),
	KEY(7, 6, KEY_X),
	KEY(7, 7, KEY_C),

	KEY(8, 0, KEY_P),
	KEY(8, 1, KEY_J),
	KEY(8, 2, KEY_K),
	KEY(8, 3, KEY_INSERT),
	KEY(8, 4, KEY_LINEFEED),
	KEY(8, 5, KEY_U),
	KEY(8, 6, KEY_I),
	KEY(8, 7, KEY_O),

	KEY(9, 0, KEY_4),
	KEY(9, 1, KEY_5),
	KEY(9, 2, KEY_6),
	KEY(9, 3, KEY_7),
	KEY(9, 4, KEY_8),
	KEY(9, 5, KEY_1),
	KEY(9, 6, KEY_2),
	KEY(9, 7, KEY_3),

	KEY(10, 0, KEY_F7),
	KEY(10, 1, KEY_F8),
	KEY(10, 2, KEY_F9),
	KEY(10, 3, KEY_F10),
	KEY(10, 4, KEY_FN),
	KEY(10, 5, KEY_9),
	KEY(10, 6, KEY_0),
	KEY(10, 7, KEY_DOT),

	KEY(11, 0, KEY_LEFTCTRL),
	KEY(11, 1, KEY_F11),  /* START */
	KEY(11, 2, KEY_ENTER),
	KEY(11, 3, KEY_SEARCH),
	KEY(11, 4, KEY_DELETE),
	KEY(11, 5, KEY_RIGHT),
	KEY(11, 6, KEY_LEFT),
	KEY(11, 7, KEY_RIGHTSHIFT),
};

static struct matrix_keymap_data surf_keymap_data = {
	.keymap_size    = ARRAY_SIZE(surf_keymap),
	.keymap		= surf_keymap,
};

static struct pm8xxx_keypad_platform_data surf_keypad_data = {
	.input_name		= "surf_keypad",
	.input_phys_device	= "surf_keypad/input0",
	.num_rows		= 12,
	.num_cols		= 8,
	.rows_gpio_start	= PM8058_GPIO_PM_TO_SYS(8),
	.cols_gpio_start	= PM8058_GPIO_PM_TO_SYS(0),
	.debounce_ms		= 15,
	.scan_delay_ms		= 32,
	.row_hold_ns		= 91500,
	.wakeup			= 1,
	.keymap_data		= &surf_keymap_data,
};

static struct matrix_keymap_data fluid_keymap_data = {
	.keymap_size	= ARRAY_SIZE(fluid_keymap),
	.keymap		= fluid_keymap,
};

static struct pm8xxx_keypad_platform_data fluid_keypad_data = {
	.input_name		= "fluid-keypad",
	.input_phys_device	= "fluid-keypad/input0",
	.num_rows		= 5,
	.num_cols		= 5,
	.rows_gpio_start	= PM8058_GPIO_PM_TO_SYS(8),
	.cols_gpio_start	= PM8058_GPIO_PM_TO_SYS(0),
	.debounce_ms		= 15,
	.scan_delay_ms		= 32,
	.row_hold_ns		= 91500,
	.wakeup			= 1,
	.keymap_data		= &fluid_keymap_data,
};

static struct pm8058_pwm_pdata pm8058_pwm_data = {
	.config         = pm8058_pwm_config,
	.enable         = pm8058_pwm_enable,
};

static struct pmic8058_led pmic8058_ffa_leds[] = {
	[0] = {
		.name		= "keyboard-backlight",
		.max_brightness = 15,
		.id		= PMIC8058_ID_LED_KB_LIGHT,
	},
};

static struct pmic8058_leds_platform_data pm8058_ffa_leds_data = {
	.num_leds = ARRAY_SIZE(pmic8058_ffa_leds),
	.leds	= pmic8058_ffa_leds,
};

static struct pmic8058_led pmic8058_surf_leds[] = {
	[0] = {
		.name		= "keyboard-backlight",
		.max_brightness = 15,
		.id		= PMIC8058_ID_LED_KB_LIGHT,
	},
	[1] = {
		.name		= "voice:red",
		.max_brightness = 20,
		.id		= PMIC8058_ID_LED_0,
	},
	[2] = {
		.name		= "wlan:green",
		.max_brightness = 20,
		.id		= PMIC8058_ID_LED_2,
	},
};

static struct pmic8058_leds_platform_data pm8058_surf_leds_data = {
	.num_leds = ARRAY_SIZE(pmic8058_surf_leds),
	.leds	= pmic8058_surf_leds,
};

static struct pmic8058_led pmic8058_fluid_leds[] = {
	[0] = {
		.name		= "keyboard-backlight",
		.max_brightness = 15,
		.id		= PMIC8058_ID_LED_KB_LIGHT,
	},
	[1] = {
		.name		= "flash:led_0",
		.max_brightness = 15,
		.id		= PMIC8058_ID_FLASH_LED_0,
	},
	[2] = {
		.name		= "flash:led_1",
		.max_brightness = 15,
		.id		= PMIC8058_ID_FLASH_LED_1,
	},
};

static struct pmic8058_leds_platform_data pm8058_fluid_leds_data = {
	.num_leds = ARRAY_SIZE(pmic8058_fluid_leds),
	.leds	= pmic8058_fluid_leds,
};

/* FUJITSU:2012-04-09 add LED start */
static struct pmic8058_led pmic8058_leds[] = {
        [0] = {
                .name           = "red",
                .max_brightness = 16777215,
                .id             = PMIC8058_ID_LED_0,
        },
        [1] = {
                .name           = "green",
                .max_brightness = 16777215,
                .id             = PMIC8058_ID_LED_1,
        },
        [2] = {
                .name           = "blue",
                .max_brightness = 16777215,
                .id             = PMIC8058_ID_LED_2,
        },
        [3] = {
                .name           = "yellow",
                .max_brightness = 16777215,
                .id             = PMIC8058_ID_LED_01,
        },
        [4] = {
                .name           = "magenta",
                .max_brightness = 16777215,
                .id             = PMIC8058_ID_LED_02,
        },
        [5] = {
                .name           = "cyan",
                .max_brightness = 16777215,
                .id             = PMIC8058_ID_LED_12,
        },
        [6] = {
                .name           = "white",
                .max_brightness = 16777215,
                .id             = PMIC8058_ID_LED_012,
        },
};

static struct pmic8058_leds_platform_data pm8058_leds_data = {
        .num_leds = ARRAY_SIZE(pmic8058_leds),
        .leds   = pmic8058_leds,
};
/* FUJITSU:2012-04-09 add LED end */

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
/* FUJITSU:2011-12-01 add IrDA start */
static struct pm8xxx_misc_platform_data pm8xxx_misc_pdata = {
	.priority		= 0,
};
/* FUJITSU:2011-12-01 add IrDA end */

static struct pm8058_platform_data pm8058_7x30_data = {
	.irq_pdata		= &pm8xxx_irq_pdata,
	.gpio_pdata		= &pm8xxx_gpio_pdata,
/* FUJITSU:2011-12-01 add IrDA start */
	.misc_pdata		= &pm8xxx_misc_pdata,
/* FUJITSU:2011-12-01 add IrDA end */
	.mpp_pdata		= &pm8xxx_mpp_pdata,
	.pwm_pdata		= &pm8058_pwm_data,
};

#ifdef CONFIG_MSM_SSBI
static struct msm_ssbi_platform_data msm7x30_ssbi_pm8058_pdata = {
	.rsl_id = "D:PMIC_SSBI",
	.controller_type = MSM_SBI_CTRL_SSBI2,
	.slave	= {
		.name			= "pm8058-core",
		.platform_data		= &pm8058_7x30_data,
	},
};
#endif

/* FUJITSU:2012-03-15 CYPRESS TOUCH PRESS add start */
static struct i2c_board_info cyttsp_press_info[] __initdata = {
    {
        I2C_BOARD_INFO(CY_PR_I2C_NAME, CY_PR_I2C_ADR),
        .platform_data = &cyttsp_press_platform_data,
        .irq = MSM_GPIO_TO_INT(CY_PR_I2C_IRQ_GPIO),
    },
};
/* FUJITSU:2012-03-15 CYPRESS TOUCH PRESS add end */

/* FUJITSU:2012-03-15 del cyttps start */
#if 0
static struct i2c_board_info cy8info[] __initdata = {
	{
		I2C_BOARD_INFO(CY_I2C_NAME, CY_I2C_ADR),
		.platform_data = &cypress_i2c_ttsp_platform_data,
		.irq = MSM_GPIO_TO_INT(CY_I2C_IRQ_GPIO),
	},
};
#endif
/* FUJITSU:2012-03-15 del cyttps start */

/* FUJITSU:2011-12-01 del cyttps start */
#if 0
static struct i2c_board_info cy8info[] __initdata = {
	{
		I2C_BOARD_INFO(CY_I2C_NAME, 0x24),
		.platform_data = &cyttsp_data,
#ifndef CY_USE_TIMER
		.irq = MSM_GPIO_TO_INT(CYTTSP_TS_GPIO_IRQ),
#endif /* CY_USE_TIMER */
	},
};
#endif
/* FUJITSU:2011-12-01 del cyttps start */

static struct i2c_board_info msm_camera_boardinfo[] __initdata = {
#ifdef CONFIG_MT9D112
	{
		I2C_BOARD_INFO("mt9d112", 0x78 >> 1),
	},
#endif
#ifdef CONFIG_WEBCAM_OV9726
	{
		I2C_BOARD_INFO("ov9726", 0x10),
	},
#endif
#ifdef CONFIG_S5K3E2FX
	{
		I2C_BOARD_INFO("s5k3e2fx", 0x20 >> 1),
	},
#endif
#ifdef CONFIG_MT9P012
	{
		I2C_BOARD_INFO("mt9p012", 0x6C >> 1),
	},
#endif
#ifdef CONFIG_VX6953
	{
		I2C_BOARD_INFO("vx6953", 0x20),
	},
#endif
#ifdef CONFIG_MT9E013
	{
		I2C_BOARD_INFO("mt9e013", 0x6C >> 2),
	},
#endif
#ifdef CONFIG_SN12M0PZ
	{
		I2C_BOARD_INFO("sn12m0pz", 0x34 >> 1),
	},
#endif
#if defined(CONFIG_MT9T013) || defined(CONFIG_SENSORS_MT9T013)
	{
		I2C_BOARD_INFO("mt9t013", 0x6C),
	},
#endif
/* FUJITSU:2012_01_19 add camera start */
#if defined(CONFIG_M6MORL_IMX046ES)
	{
		I2C_BOARD_INFO("m6morl_imx046es", 0x1F),
	},
#endif

#if defined(CONFIG_S5K6AAFX13)
	{
		I2C_BOARD_INFO("s5k6aafx13", 0x3C),
	},
#endif
/* FUJITSU:2012_01_19 add camera end */
	
/* FUJITSU:2012_03_09 add out_camera start */
#if defined(CONFIG_M6MORL_IMX091)
	{
		I2C_BOARD_INFO("m6morl_imx091", 0x1F),
	},
#endif
/* FUJITSU:2012_03_09 add out_camera end */
/* FUJITSU:2012_03_12 add in_camera start */
#if defined(CONFIG_RJ6ABA)
	{
		I2C_BOARD_INFO("rj6aba", 0x3C),
	},
#endif
/* FUJITSU:2012_03_12 add in_camera end */

};

#ifdef CONFIG_MSM_CAMERA
/* FUJITSU:2011-12-01 del camera GPIO start */
#if 0
#define	CAM_STNDBY	143
static uint32_t camera_off_vcm_gpio_table[] = {
GPIO_CFG(1, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* VCM */
};

static uint32_t camera_off_gpio_table[] = {
	/* parallel CAMERA interfaces */
	GPIO_CFG(0,  0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* RST */
	GPIO_CFG(2,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT2 */
	GPIO_CFG(3,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT3 */
	GPIO_CFG(4,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT4 */
	GPIO_CFG(5,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT5 */
	GPIO_CFG(6,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT6 */
	GPIO_CFG(7,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT7 */
	GPIO_CFG(8,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT8 */
	GPIO_CFG(9,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT9 */
	GPIO_CFG(10, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT10 */
	GPIO_CFG(11, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT11 */
	GPIO_CFG(12, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* PCLK */
	GPIO_CFG(13, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* HSYNC_IN */
	GPIO_CFG(14, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* VSYNC_IN */
	GPIO_CFG(15, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /* MCLK */
};

static uint32_t camera_on_vcm_gpio_table[] = {
GPIO_CFG(1, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), /* VCM */
};

static uint32_t camera_on_gpio_table[] = {
	/* parallel CAMERA interfaces */
	GPIO_CFG(0,  0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* RST */
	GPIO_CFG(2,  1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT2 */
	GPIO_CFG(3,  1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT3 */
	GPIO_CFG(4,  1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT4 */
	GPIO_CFG(5,  1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT5 */
	GPIO_CFG(6,  1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT6 */
	GPIO_CFG(7,  1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT7 */
	GPIO_CFG(8,  1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT8 */
	GPIO_CFG(9,  1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT9 */
	GPIO_CFG(10, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT10 */
	GPIO_CFG(11, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT11 */
	GPIO_CFG(12, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* PCLK */
	GPIO_CFG(13, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* HSYNC_IN */
	GPIO_CFG(14, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* VSYNC_IN */
	GPIO_CFG(15, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), /* MCLK */
};

static uint32_t camera_off_gpio_fluid_table[] = {
	/* FLUID: CAM_VGA_RST_N */
	GPIO_CFG(31, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	/* FLUID: CAMIF_STANDBY */
	GPIO_CFG(CAM_STNDBY, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA)
};

static uint32_t camera_on_gpio_fluid_table[] = {
	/* FLUID: CAM_VGA_RST_N */
	GPIO_CFG(31, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),
	/* FLUID: CAMIF_STANDBY */
	GPIO_CFG(CAM_STNDBY, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA)
};

#endif
/* FUJITSU:2011_07_27 mod camera end */

/* FUJITSU:2012_01_19 add camera start */
#ifdef CONFIG_S5K6AAFX13
static uint32_t front_camera_off_gpio_table[] = {
	/* parallel CAMERA interfaces */
	GPIO_CFG(4,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT4 */
	GPIO_CFG(5,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT5 */
	GPIO_CFG(6,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT6 */
	GPIO_CFG(7,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT7 */
	GPIO_CFG(8,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT8 */
	GPIO_CFG(9,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT9 */
	GPIO_CFG(10, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT10 */
	GPIO_CFG(11, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT11 */
	GPIO_CFG(12, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* PCLK */
	GPIO_CFG(13, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* HSYNC_IN */
	GPIO_CFG(14, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* VSYNC_IN */
};

static uint32_t front_camera_on_gpio_table[] = {
	/* parallel CAMERA interfaces */
	GPIO_CFG(4,  1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT4 */
	GPIO_CFG(5,  1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT5 */
	GPIO_CFG(6,  1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT6 */
	GPIO_CFG(7,  1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT7 */
	GPIO_CFG(8,  1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT8 */
	GPIO_CFG(9,  1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT9 */
	GPIO_CFG(10, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT10 */
	GPIO_CFG(11, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT11 */
	GPIO_CFG(12, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* PCLK */
	GPIO_CFG(13, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* HSYNC_IN */
	GPIO_CFG(14, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* VSYNC_IN */
};

static void config_gpio_table(uint32_t *table, int len)
{
	int n, rc;
	for (n = 0; n < len; n++) {
		rc = gpio_tlmm_config(table[n], GPIO_CFG_ENABLE);
		if (rc) {
			pr_err("%s: gpio_tlmm_config(%#x)=%d\n",
				__func__, table[n], rc);
			break;
		}
	}
}
#endif // CONFIG_S5K6AAFX13
/* FUJITSU:2012_01_19 add camera end */

/* FUJITSU:2012_03_12 add in_camera start */
#ifdef CONFIG_RJ6ABA
static uint32_t front_camera_off_gpio_table[] = {
	/* parallel CAMERA interfaces */
	GPIO_CFG(4,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT4 */
	GPIO_CFG(5,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT5 */
	GPIO_CFG(6,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT6 */
	GPIO_CFG(7,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT7 */
	GPIO_CFG(8,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT8 */
	GPIO_CFG(9,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT9 */
	GPIO_CFG(10, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT10 */
	GPIO_CFG(11, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT11 */
	GPIO_CFG(12, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* PCLK */
	GPIO_CFG(13, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* HSYNC_IN */
	GPIO_CFG(14, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* VSYNC_IN */
};

static uint32_t front_camera_on_gpio_table[] = {
	/* parallel CAMERA interfaces */
	GPIO_CFG(4,  1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT4 */
	GPIO_CFG(5,  1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT5 */
	GPIO_CFG(6,  1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT6 */
	GPIO_CFG(7,  1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT7 */
	GPIO_CFG(8,  1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT8 */
	GPIO_CFG(9,  1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT9 */
	GPIO_CFG(10, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT10 */
	GPIO_CFG(11, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* DAT11 */
	GPIO_CFG(12, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* PCLK */
	GPIO_CFG(13, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* HSYNC_IN */
	GPIO_CFG(14, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), /* VSYNC_IN */
};

static void config_gpio_table(uint32_t *table, int len)
{
	int n, rc;
	for (n = 0; n < len; n++) {
		rc = gpio_tlmm_config(table[n], GPIO_CFG_ENABLE);
		if (rc) {
			pr_err("%s: gpio_tlmm_config(%#x)=%d\n",
				__func__, table[n], rc);
			break;
		}
	}
}
#endif // CONFIG_RJ6ABA
/* FUJITSU:2012_03_12 add in_camera end */

/* FUJITSU:2011-12-01 del camera GPIO end */
static int config_camera_on_gpios(void)
{
/* FUJITSU:2011-12-01 del camera GPIO start */
#if 0
	config_gpio_table(camera_on_gpio_table,
		ARRAY_SIZE(camera_on_gpio_table));

	if (adie_get_detected_codec_type() != TIMPANI_ID)
		/* GPIO1 is shared also used in Timpani RF card so
		only configure it for non-Timpani RF card */
		config_gpio_table(camera_on_vcm_gpio_table,
			ARRAY_SIZE(camera_on_vcm_gpio_table));

	if (machine_is_msm7x30_fluid()) {
		config_gpio_table(camera_on_gpio_fluid_table,
			ARRAY_SIZE(camera_on_gpio_fluid_table));
		/* FLUID: turn on 5V booster */
		gpio_set_value(
			PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_FLASH_BOOST_ENABLE), 1);
		/* FLUID: drive high to put secondary sensor to STANDBY */
		gpio_set_value(CAM_STNDBY, 1);
	}
#endif
/* FUJITSU:2011-12-01 del camera GPIO end */
	return 0;
}

static void config_camera_off_gpios(void)
{
/* FUJITSU:2011-12-01 del camera GPIO start */
#if 0
	config_gpio_table(camera_off_gpio_table,
		ARRAY_SIZE(camera_off_gpio_table));

	if (adie_get_detected_codec_type() != TIMPANI_ID)
		/* GPIO1 is shared also used in Timpani RF card so
		only configure it for non-Timpani RF card */
		config_gpio_table(camera_off_vcm_gpio_table,
			ARRAY_SIZE(camera_off_vcm_gpio_table));

	if (machine_is_msm7x30_fluid()) {
		config_gpio_table(camera_off_gpio_fluid_table,
			ARRAY_SIZE(camera_off_gpio_fluid_table));
		/* FLUID: turn off 5V booster */
		gpio_set_value(
			PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_FLASH_BOOST_ENABLE), 0);
	}
#endif
/* FUJITSU:2011-12-01 del camera GPIO end */
}

/* FUJITSU:2012_01_19 add camera start */
#ifdef CONFIG_S5K6AAFX13
static int config_front_camera_on_gpios(void)
{
    config_gpio_table(front_camera_on_gpio_table,
        ARRAY_SIZE(front_camera_on_gpio_table));
    return 0;
}

static void config_front_camera_off_gpios(void)
{
    config_gpio_table(front_camera_off_gpio_table,
        ARRAY_SIZE(front_camera_off_gpio_table));
}
#endif // CONFIG_S5K6AAFX13
/* FUJITSU:2012_01_19 add camera end */

/* FUJITSU:2012_03_12 add in_camera start */
#ifdef CONFIG_RJ6ABA
static int config_front_camera_on_gpios(void)
{
    config_gpio_table(front_camera_on_gpio_table,
        ARRAY_SIZE(front_camera_on_gpio_table));
    return 0;
}

static void config_front_camera_off_gpios(void)
{
    config_gpio_table(front_camera_off_gpio_table,
        ARRAY_SIZE(front_camera_off_gpio_table));
}
#endif // CONFIG_RJ6ABA
/* FUJITSU:2012_03_12 add in_camera end */

struct resource msm_camera_resources[] = {
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
	{
		.flags  = IORESOURCE_DMA,
	}
};

struct msm_camera_device_platform_data msm_camera_device_data = {
	.camera_gpio_on  = config_camera_on_gpios,
	.camera_gpio_off = config_camera_off_gpios,
	.ioext.camifpadphy = 0xAB000000,
	.ioext.camifpadsz  = 0x00000400,
	.ioext.csiphy = 0xA6100000,
	.ioext.csisz  = 0x00000400,
	.ioext.csiirq = INT_CSI,
	.ioclk.mclk_clk_rate = 24000000,
	.ioclk.vfe_clk_rate  = 147456000,
};

/* FUJITSU:2012_01_19 add camera start */
#ifdef CONFIG_M6MORL_IMX046ES
struct msm_camera_device_platform_data msm_camera_device_m6morl_imx046es_data = {
	.camera_gpio_on  = config_camera_on_gpios,
	.camera_gpio_off = config_camera_off_gpios,
	.ioext.camifpadphy = 0xAB000000,
	.ioext.camifpadsz  = 0x00000400,
	.ioext.csiphy = 0xA6100000,
	.ioext.csisz  = 0x00000400,
	.ioext.csiirq = INT_CSI,
	.ioclk.mclk_clk_rate = 24576000,
	.ioclk.vfe_clk_rate  = 122880000,
};
#endif
/* FUJITSU:2012_01_19 add camera end */

/* FUJITSU:2012_03_09 add out_camera start */
#ifdef CONFIG_M6MORL_IMX091
struct msm_camera_device_platform_data msm_camera_device_m6morl_imx091_data = {
	.camera_gpio_on  = config_camera_on_gpios,
	.camera_gpio_off = config_camera_off_gpios,
	.ioext.camifpadphy = 0xAB000000,
	.ioext.camifpadsz  = 0x00000400,
	.ioext.csiphy = 0xA6100000,
	.ioext.csisz  = 0x00000400,
	.ioext.csiirq = INT_CSI,
	.ioclk.mclk_clk_rate = 24576000,
	.ioclk.vfe_clk_rate  = 122880000,
};
#endif
/* FUJITSU:2012_03_09 add out_camera end */

/* FUJITSU:2012_01_19 add camera start */
#ifdef CONFIG_S5K6AAFX13
struct msm_camera_device_platform_data msm_camera_device_s5k6aafx13_data = {
	.camera_gpio_on  = config_front_camera_on_gpios,
	.camera_gpio_off = config_front_camera_off_gpios,
	.ioext.camifpadphy = 0xAB000000,
	.ioext.camifpadsz  = 0x00000400,
	.ioext.csiphy = 0xA6100000,
	.ioext.csisz  = 0x00000400,
	.ioext.csiirq = INT_CSI,
//	.ioclk.mclk_clk_rate = 23840000,
	.ioclk.mclk_clk_rate = 21333333,
	.ioclk.vfe_clk_rate  = 122880000,
};
#endif
/* FUJITSU:2012_01_19 mod camera end */

/* FUJITSU:2012_03_12 add in_camera start */
#ifdef CONFIG_RJ6ABA
struct msm_camera_device_platform_data msm_camera_device_rj6aba_data = {
	.camera_gpio_on  = config_front_camera_on_gpios,
	.camera_gpio_off = config_front_camera_off_gpios,
	.ioext.camifpadphy = 0xAB000000,
	.ioext.camifpadsz  = 0x00000400,
	.ioext.csiphy = 0xA6100000,
	.ioext.csisz  = 0x00000400,
	.ioext.csiirq = INT_CSI,
//	.ioclk.mclk_clk_rate = 23840000,
//	.ioclk.mclk_clk_rate = 24576000,
	.ioclk.mclk_clk_rate = 25600000,
	.ioclk.vfe_clk_rate  = 122880000,
};
#endif
/* FUJITSU:2012_03_12 add in_camera end */

#if 1 /* FUJITSU:2012_01_19 mod camera start */
//#ifdef CONFIG_MACH_F11EIF
/* FUJITSU:2012_03_09 add CONFIG_M6MORL_IMX091 out_camera */
#if defined(CONFIG_M6MORL_IMX046ES) || defined(CONFIG_S5K6AAFX13) || defined(CONFIG_M6MORL_IMX091) || defined(CONFIG_RJ6ABA)
static struct msm_camera_sensor_flash_src msm_flash_src_pwm = {
	.flash_sr_type = MSM_CAMERA_FLASH_SRC_PWM,
	._fsrc.pwm_src.freq  = 1000,
	._fsrc.pwm_src.max_load = 300,
	._fsrc.pwm_src.low_load = 30,
	._fsrc.pwm_src.high_load = 100,
	._fsrc.pwm_src.channel = 7,
};
#endif
#else
/* FUJITSU:2011-12-01 del camera start */
//static struct msm_camera_sensor_flash_src msm_flash_src_pwm = {
//	.flash_sr_type = MSM_CAMERA_FLASH_SRC_PWM,
//	._fsrc.pwm_src.freq  = 1000,
//	._fsrc.pwm_src.max_load = 300,
//	._fsrc.pwm_src.low_load = 30,
//	._fsrc.pwm_src.high_load = 100,
//	._fsrc.pwm_src.channel = 7,
//};
/* FUJITSU:2011-12-01 del camera end */
#endif /* FUJITSU:2012_01_19 mod camera end */

#ifdef CONFIG_MT9D112
static struct msm_camera_sensor_flash_data flash_mt9d112 = {
	.flash_type = MSM_CAMERA_FLASH_LED,
	.flash_src  = &msm_flash_src_pwm
};

static struct msm_camera_sensor_info msm_camera_sensor_mt9d112_data = {
	.sensor_name    = "mt9d112",
	.sensor_reset   = 0,
	.sensor_pwd     = 85,
	.vcm_pwd        = 1,
	.vcm_enable     = 0,
	.pdata          = &msm_camera_device_data,
	.resource       = msm_camera_resources,
	.num_resources  = ARRAY_SIZE(msm_camera_resources),
	.flash_data     = &flash_mt9d112,
	.csi_if         = 0
};

static struct platform_device msm_camera_sensor_mt9d112 = {
	.name      = "msm_camera_mt9d112",
	.dev       = {
		.platform_data = &msm_camera_sensor_mt9d112_data,
	},
};
#endif

#ifdef CONFIG_WEBCAM_OV9726

static struct msm_camera_sensor_platform_info ov9726_sensor_7630_info = {
	.mount_angle = 90
};

static struct msm_camera_sensor_flash_data flash_ov9726 = {
	.flash_type	= MSM_CAMERA_FLASH_LED,
	.flash_src	= &msm_flash_src_pwm
};
static struct msm_camera_sensor_info msm_camera_sensor_ov9726_data = {
	.sensor_name	= "ov9726",
	.sensor_reset	= 0,
	.sensor_pwd	= 85,
	.vcm_pwd	= 1,
	.vcm_enable	= 0,
	.pdata		= &msm_camera_device_data,
	.resource	= msm_camera_resources,
	.num_resources	= ARRAY_SIZE(msm_camera_resources),
	.flash_data	= &flash_ov9726,
	.sensor_platform_info = &ov9726_sensor_7630_info,
	.csi_if		= 1
};
struct platform_device msm_camera_sensor_ov9726 = {
	.name	= "msm_camera_ov9726",
	.dev	= {
		.platform_data = &msm_camera_sensor_ov9726_data,
	},
};
#endif

#ifdef CONFIG_S5K3E2FX
static struct msm_camera_sensor_flash_data flash_s5k3e2fx = {
	.flash_type = MSM_CAMERA_FLASH_LED,
	.flash_src  = &msm_flash_src_pwm,
};

static struct msm_camera_sensor_info msm_camera_sensor_s5k3e2fx_data = {
	.sensor_name    = "s5k3e2fx",
	.sensor_reset   = 0,
	.sensor_pwd     = 85,
	.vcm_pwd        = 1,
	.vcm_enable     = 0,
	.pdata          = &msm_camera_device_data,
	.resource       = msm_camera_resources,
	.num_resources  = ARRAY_SIZE(msm_camera_resources),
	.flash_data     = &flash_s5k3e2fx,
	.csi_if         = 0
};

static struct platform_device msm_camera_sensor_s5k3e2fx = {
	.name      = "msm_camera_s5k3e2fx",
	.dev       = {
		.platform_data = &msm_camera_sensor_s5k3e2fx_data,
	},
};
#endif

#ifdef CONFIG_MT9P012
static struct msm_camera_sensor_flash_data flash_mt9p012 = {
	.flash_type = MSM_CAMERA_FLASH_LED,
	.flash_src  = &msm_flash_src_pwm
};

static struct msm_camera_sensor_info msm_camera_sensor_mt9p012_data = {
	.sensor_name    = "mt9p012",
	.sensor_reset   = 0,
	.sensor_pwd     = 85,
	.vcm_pwd        = 1,
	.vcm_enable     = 1,
	.pdata          = &msm_camera_device_data,
	.resource       = msm_camera_resources,
	.num_resources  = ARRAY_SIZE(msm_camera_resources),
	.flash_data     = &flash_mt9p012,
	.csi_if         = 0
};

static struct platform_device msm_camera_sensor_mt9p012 = {
	.name      = "msm_camera_mt9p012",
	.dev       = {
		.platform_data = &msm_camera_sensor_mt9p012_data,
	},
};
#endif

#ifdef CONFIG_MT9E013
static struct msm_camera_sensor_platform_info mt9e013_sensor_7630_info = {
	.mount_angle = 0
};

static struct msm_camera_sensor_flash_data flash_mt9e013 = {
	.flash_type = MSM_CAMERA_FLASH_LED,
	.flash_src  = &msm_flash_src_pwm
};

static struct msm_camera_sensor_info msm_camera_sensor_mt9e013_data = {
	.sensor_name    = "mt9e013",
	.sensor_reset   = 0,
	.sensor_pwd     = 85,
	.vcm_pwd        = 1,
	.vcm_enable     = 1,
	.pdata          = &msm_camera_device_data,
	.resource       = msm_camera_resources,
	.num_resources  = ARRAY_SIZE(msm_camera_resources),
	.flash_data     = &flash_mt9e013,
	.sensor_platform_info = &mt9e013_sensor_7630_info,
	.csi_if         = 1
};

static struct platform_device msm_camera_sensor_mt9e013 = {
	.name      = "msm_camera_mt9e013",
	.dev       = {
		.platform_data = &msm_camera_sensor_mt9e013_data,
	},
};
#endif

#ifdef CONFIG_VX6953
static struct msm_camera_sensor_platform_info vx6953_sensor_7630_info = {
	.mount_angle = 0
};

static struct msm_camera_sensor_flash_data flash_vx6953 = {
	.flash_type = MSM_CAMERA_FLASH_LED,
	.flash_src  = &msm_flash_src_pwm
};
static struct msm_camera_sensor_info msm_camera_sensor_vx6953_data = {
	.sensor_name    = "vx6953",
	.sensor_reset   = 0,
	.sensor_pwd     = 85,
	.vcm_pwd        = 1,
	.vcm_enable		= 0,
	.pdata          = &msm_camera_device_data,
	.resource       = msm_camera_resources,
	.num_resources  = ARRAY_SIZE(msm_camera_resources),
	.sensor_platform_info = &vx6953_sensor_7630_info,
	.flash_data     = &flash_vx6953,
	.csi_if         = 1
};
static struct platform_device msm_camera_sensor_vx6953 = {
	.name  	= "msm_camera_vx6953",
	.dev   	= {
		.platform_data = &msm_camera_sensor_vx6953_data,
	},
};
#endif

#ifdef CONFIG_SN12M0PZ
static struct msm_camera_sensor_flash_src msm_flash_src_current_driver = {
	.flash_sr_type = MSM_CAMERA_FLASH_SRC_CURRENT_DRIVER,
	._fsrc.current_driver_src.low_current = 210,
	._fsrc.current_driver_src.high_current = 700,
	._fsrc.current_driver_src.driver_channel = &pm8058_fluid_leds_data,
};

static struct msm_camera_sensor_flash_data flash_sn12m0pz = {
	.flash_type = MSM_CAMERA_FLASH_LED,
	.flash_src  = &msm_flash_src_current_driver
};
static struct msm_camera_sensor_info msm_camera_sensor_sn12m0pz_data = {
	.sensor_name    = "sn12m0pz",
	.sensor_reset   = 0,
	.sensor_pwd     = 85,
	.vcm_pwd        = 1,
	.vcm_enable     = 1,
	.pdata          = &msm_camera_device_data,
	.flash_data     = &flash_sn12m0pz,
	.resource       = msm_camera_resources,
	.num_resources  = ARRAY_SIZE(msm_camera_resources),
	.csi_if         = 0
};

static struct platform_device msm_camera_sensor_sn12m0pz = {
	.name      = "msm_camera_sn12m0pz",
	.dev       = {
		.platform_data = &msm_camera_sensor_sn12m0pz_data,
	},
};
#endif

#ifdef CONFIG_MT9T013
static struct msm_camera_sensor_flash_data flash_mt9t013 = {
	.flash_type = MSM_CAMERA_FLASH_LED,
	.flash_src  = &msm_flash_src_pwm
};

static struct msm_camera_sensor_info msm_camera_sensor_mt9t013_data = {
	.sensor_name    = "mt9t013",
	.sensor_reset   = 0,
	.sensor_pwd     = 85,
	.vcm_pwd        = 1,
	.vcm_enable     = 0,
	.pdata          = &msm_camera_device_data,
	.resource       = msm_camera_resources,
	.num_resources  = ARRAY_SIZE(msm_camera_resources),
	.flash_data     = &flash_mt9t013,
	.csi_if         = 1
};

static struct platform_device msm_camera_sensor_mt9t013 = {
	.name      = "msm_camera_mt9t013",
	.dev       = {
		.platform_data = &msm_camera_sensor_mt9t013_data,
	},
};
#endif

/* FUJITSU:2012_01_19 add camera start */
#ifdef CONFIG_M6MORL_IMX046ES
static struct msm_camera_sensor_flash_data flash_m6morl_imx046es = {
//	.flash_type = MSM_CAMERA_FLASH_LED,
	.flash_type = MSM_CAMERA_FLASH_NONE,
	.flash_src  = &msm_flash_src_pwm
};

static struct msm_camera_sensor_info msm_camera_sensor_m6morl_imx046es_data = {
	.sensor_name    = "m6morl_imx046es",
	.sensor_reset   = 0,
	.sensor_pwd     = 85,
	.vcm_pwd        = 1,
	.vcm_enable     = 1,
	.pdata          = &msm_camera_device_m6morl_imx046es_data,
	.resource       = msm_camera_resources,
	.num_resources  = ARRAY_SIZE(msm_camera_resources),
	.flash_data     = &flash_m6morl_imx046es,
	.csi_if         = 1
};

static struct platform_device msm_camera_sensor_m6morl_imx046es = {
	.name      = "msm_camera_m6morl_imx046es",
	.dev       = {
		.platform_data = &msm_camera_sensor_m6morl_imx046es_data,
	},
};
#endif
/* FUJITSU:2012_01_19 add camera end */

/* FUJITSU:2012_03_09 add out_camera start */
#ifdef CONFIG_M6MORL_IMX091
static struct msm_camera_sensor_flash_data flash_m6morl_imx091 = {
//	.flash_type = MSM_CAMERA_FLASH_LED,
	.flash_type = MSM_CAMERA_FLASH_NONE,
	.flash_src  = &msm_flash_src_pwm
};

static struct msm_camera_sensor_info msm_camera_sensor_m6morl_imx091_data = {
	.sensor_name    = "m6morl_imx091",
	.sensor_reset   = 0,
	.sensor_pwd     = 85,
	.vcm_pwd        = 1,
	.vcm_enable     = 1,
	.pdata          = &msm_camera_device_m6morl_imx091_data,
	.resource       = msm_camera_resources,
	.num_resources  = ARRAY_SIZE(msm_camera_resources),
	.flash_data     = &flash_m6morl_imx091,
	.csi_if         = 1
};

static struct platform_device msm_camera_sensor_m6morl_imx091 = {
	.name      = "msm_camera_m6morl_imx091",
	.dev       = {
		.platform_data = &msm_camera_sensor_m6morl_imx091_data,
	},
};
#endif //CONFIG_M6MORL_IMX091
/* FUJITSU:2012_03_09 add out_camera end */

/* FUJITSU:2012_01_19 add camera start */
#ifdef CONFIG_S5K6AAFX13
static struct msm_camera_sensor_flash_data flash_s5k6aafx13 = {
//	.flash_type = MSM_CAMERA_FLASH_LED,
	.flash_type = MSM_CAMERA_FLASH_NONE,
	.flash_src  = &msm_flash_src_pwm
};

static struct msm_camera_sensor_info msm_camera_sensor_s5k6aafx13_data = {
	.sensor_name    = "s5k6aafx13",
	.sensor_reset   = 0,
	.sensor_pwd     = 85,
	.vcm_pwd        = 1,
	.vcm_enable     = 1,
	.pdata          = &msm_camera_device_s5k6aafx13_data,
	.resource       = msm_camera_resources,
	.num_resources  = ARRAY_SIZE(msm_camera_resources),
	.flash_data     = &flash_s5k6aafx13,
	.csi_if         = 0
};

static struct platform_device msm_camera_sensor_s5k6aafx13 = {
	.name      = "msm_camera_s5k6aafx13",
	.dev       = {
		.platform_data = &msm_camera_sensor_s5k6aafx13_data,
	},
};
#endif
/* FUJITSU:2012_01_19 add camera end */

/* FUJITSU:2012_03_12 add in_camera start */
#ifdef CONFIG_RJ6ABA
static struct msm_camera_sensor_flash_data flash_rj6aba = {
//	.flash_type = MSM_CAMERA_FLASH_LED,
	.flash_type = MSM_CAMERA_FLASH_NONE,
	.flash_src  = &msm_flash_src_pwm
};

static struct msm_camera_sensor_info msm_camera_sensor_rj6aba_data = {
	.sensor_name    = "rj6aba",
	.sensor_reset   = 0,
	.sensor_pwd     = 85,
	.vcm_pwd        = 1,
	.vcm_enable     = 1,
	.pdata          = &msm_camera_device_rj6aba_data,
	.resource       = msm_camera_resources,
	.num_resources  = ARRAY_SIZE(msm_camera_resources),
	.flash_data     = &flash_rj6aba,
	.csi_if         = 0
};

static struct platform_device msm_camera_sensor_rj6aba = {
	.name      = "msm_camera_rj6aba",
	.dev       = {
		.platform_data = &msm_camera_sensor_rj6aba_data,
	},
};
#endif
/* FUJITSU:2012_03_12 add in_camera end */

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

#endif /*CONFIG_MSM_CAMERA*/

/* FUJITSU:2011-12-01 add bsp-sensor start */
static struct i2c_board_info compassinfo[] __initdata = {
	{
		I2C_BOARD_INFO("compass", 0x0c),
	},
};
/* FUJITSU:2011-12-01 add bsp-sensor end */

/* FUJITSU:2012.03.27  HapticFeedback [start] */
static struct i2c_board_info ts5k_info[] __initdata = {
     {
          I2C_BOARD_INFO("ts5k", 0x59),
     },
};
/* FUJITSU:2012.03.27  HapticFeedback [end]   */

#ifdef CONFIG_MSM7KV2_AUDIO
static uint32_t audio_pamp_gpio_config =
   GPIO_CFG(82, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA);

static uint32_t audio_fluid_icodec_tx_config =
  GPIO_CFG(85, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA);

static int __init snddev_poweramp_gpio_init(void)
{
	int rc;

	pr_info("snddev_poweramp_gpio_init \n");
	rc = gpio_tlmm_config(audio_pamp_gpio_config, GPIO_CFG_ENABLE);
	if (rc) {
		printk(KERN_ERR
			"%s: gpio_tlmm_config(%#x)=%d\n",
			__func__, audio_pamp_gpio_config, rc);
	}
	return rc;
}

void msm_snddev_tx_route_config(void)
{
	int rc;

	pr_debug("%s()\n", __func__);

	if (machine_is_msm7x30_fluid()) {
		rc = gpio_tlmm_config(audio_fluid_icodec_tx_config,
		GPIO_CFG_ENABLE);
		if (rc) {
			printk(KERN_ERR
				"%s: gpio_tlmm_config(%#x)=%d\n",
				__func__, audio_fluid_icodec_tx_config, rc);
		} else
			gpio_set_value(85, 0);
	}
}

void msm_snddev_tx_route_deconfig(void)
{
	int rc;

	pr_debug("%s()\n", __func__);

	if (machine_is_msm7x30_fluid()) {
		rc = gpio_tlmm_config(audio_fluid_icodec_tx_config,
		GPIO_CFG_DISABLE);
		if (rc) {
			printk(KERN_ERR
				"%s: gpio_tlmm_config(%#x)=%d\n",
				__func__, audio_fluid_icodec_tx_config, rc);
		}
	}
}

void msm_snddev_poweramp_on(void)
{
/* FUJITSU:2011-12-22 speaker amp start */
#if 0
	gpio_set_value(82, 1);	/* enable spkr poweramp */
#else
	gpio_tlmm_config( GPIO_CFG(19, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE );
	gpio_set_value(19,1);
#endif
/* FUJITSU:2011-12-22 speaker amp end */
	pr_info("%s: power on amplifier\n", __func__);
}

void msm_snddev_poweramp_off(void)
{
/* FUJITSU:2011-12-22 speaker amp start */
#if 0
	gpio_set_value(82, 0);	/* disable spkr poweramp */
#else
	gpio_set_value(19,0);
	gpio_tlmm_config( GPIO_CFG(19, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE );
#endif
/* FUJITSU:2011-12-22 speaker amp end */
	pr_info("%s: power off amplifier\n", __func__);
}

static struct regulator_bulk_data snddev_regs[] = {
	{ .supply = "gp4", .min_uV = 2600000, .max_uV = 2600000 },
	{ .supply = "ncp", .min_uV = 1800000, .max_uV = 1800000 },
};

static int __init snddev_hsed_voltage_init(void)
{
	int rc;

	rc = regulator_bulk_get(NULL, ARRAY_SIZE(snddev_regs), snddev_regs);

	if (rc) {
		pr_err("%s: could not get regulators: %d\n", __func__, rc);
		goto out;
	}

	rc = regulator_bulk_set_voltage(ARRAY_SIZE(snddev_regs), snddev_regs);

	if (rc) {
		pr_err("%s: could not set regulator voltages: %d\n",
				__func__, rc);
		goto regs_free;
	}

	return 0;

regs_free:
	regulator_bulk_free(ARRAY_SIZE(snddev_regs), snddev_regs);
out:
	return rc;
}


void msm_snddev_hsed_voltage_on(void)
{
	int rc = regulator_bulk_enable(ARRAY_SIZE(snddev_regs), snddev_regs);

	if (rc)
		pr_err("%s: could not enable regulators: %d\n", __func__, rc);
}

void msm_snddev_hsed_voltage_off(void)
{
	int rc = regulator_bulk_disable(ARRAY_SIZE(snddev_regs), snddev_regs);

	if (rc) {
		pr_err("%s: could not disable regulators: %d\n", __func__, rc);
	}
}

static unsigned aux_pcm_gpio_on[] = {
	GPIO_CFG(138, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),   /* PCM_DOUT */
	GPIO_CFG(139, 1, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA),   /* PCM_DIN  */
	GPIO_CFG(140, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),   /* PCM_SYNC */
	GPIO_CFG(141, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),   /* PCM_CLK  */
};

static int __init aux_pcm_gpio_init(void)
{
	int pin, rc;

	pr_info("aux_pcm_gpio_init \n");
	for (pin = 0; pin < ARRAY_SIZE(aux_pcm_gpio_on); pin++) {
		rc = gpio_tlmm_config(aux_pcm_gpio_on[pin],
					GPIO_CFG_ENABLE);
		if (rc) {
			printk(KERN_ERR
				"%s: gpio_tlmm_config(%#x)=%d\n",
				__func__, aux_pcm_gpio_on[pin], rc);
		}
	}
	return rc;
}

static struct msm_gpio mi2s_clk_gpios[] = {
/* FUJITSU:2011-12-01 del mi2s GPIO start */
#if 0
	{ GPIO_CFG(145, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	    "MI2S_SCLK"},
	{ GPIO_CFG(144, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	    "MI2S_WS"},
	{ GPIO_CFG(120, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	    "MI2S_MCLK_A"},
#endif
/* FUJITSU:2011-12-01 del mi2s GPIO end */
};

static struct msm_gpio mi2s_rx_data_lines_gpios[] = {
/* FUJITSU:2012-05-08 Del GPIO start */
#if 0
	{ GPIO_CFG(121, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	    "MI2S_DATA_SD0_A"},
	{ GPIO_CFG(122, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	    "MI2S_DATA_SD1_A"},
	{ GPIO_CFG(123, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	    "MI2S_DATA_SD2_A"},
	{ GPIO_CFG(146, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	    "MI2S_DATA_SD3"},
#endif
/* FUJITSU:2012-05-08 Del GPIO end */
};

static struct msm_gpio mi2s_tx_data_lines_gpios[] = {
/* FUJITSU:2012-05-08 Del GPIO start */
#if 0
	{ GPIO_CFG(146, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	    "MI2S_DATA_SD3"},
#endif
/* FUJITSU:2012-05-08 Del GPIO end */
};

int mi2s_config_clk_gpio(void)
{
	int rc = 0;

	rc = msm_gpios_request_enable(mi2s_clk_gpios,
			ARRAY_SIZE(mi2s_clk_gpios));
	if (rc) {
		pr_err("%s: enable mi2s clk gpios  failed\n",
					__func__);
		return rc;
	}
	return 0;
}

int  mi2s_unconfig_data_gpio(u32 direction, u8 sd_line_mask)
{
	int i, rc = 0;
	sd_line_mask &= MI2S_SD_LINE_MASK;

	switch (direction) {
	case DIR_TX:
		msm_gpios_disable_free(mi2s_tx_data_lines_gpios, 1);
		break;
	case DIR_RX:
		i = 0;
		while (sd_line_mask) {
			if (sd_line_mask & 0x1)
				msm_gpios_disable_free(
					mi2s_rx_data_lines_gpios + i , 1);
			sd_line_mask = sd_line_mask >> 1;
			i++;
		}
		break;
	default:
		pr_err("%s: Invaild direction  direction = %u\n",
						__func__, direction);
		rc = -EINVAL;
		break;
	}
	return rc;
}

int mi2s_config_data_gpio(u32 direction, u8 sd_line_mask)
{
	int i , rc = 0;
	u8 sd_config_done_mask = 0;

	sd_line_mask &= MI2S_SD_LINE_MASK;

	switch (direction) {
	case DIR_TX:
		if ((sd_line_mask & MI2S_SD_0) || (sd_line_mask & MI2S_SD_1) ||
		   (sd_line_mask & MI2S_SD_2) || !(sd_line_mask & MI2S_SD_3)) {
			pr_err("%s: can not use SD0 or SD1 or SD2 for TX"
				".only can use SD3. sd_line_mask = 0x%x\n",
				__func__ , sd_line_mask);
			rc = -EINVAL;
		} else {
			rc = msm_gpios_request_enable(mi2s_tx_data_lines_gpios,
							 1);
			if (rc)
				pr_err("%s: enable mi2s gpios for TX failed\n",
					   __func__);
		}
		break;
	case DIR_RX:
		i = 0;
		while (sd_line_mask && (rc == 0)) {
			if (sd_line_mask & 0x1) {
				rc = msm_gpios_request_enable(
					mi2s_rx_data_lines_gpios + i , 1);
				if (rc) {
					pr_err("%s: enable mi2s gpios for"
					 "RX failed.  SD line = %s\n",
					 __func__,
					 (mi2s_rx_data_lines_gpios + i)->label);
					mi2s_unconfig_data_gpio(DIR_RX,
						sd_config_done_mask);
				} else
					sd_config_done_mask |= (1 << i);
			}
			sd_line_mask = sd_line_mask >> 1;
			i++;
		}
		break;
	default:
		pr_err("%s: Invaild direction  direction = %u\n",
			__func__, direction);
		rc = -EINVAL;
		break;
	}
	return rc;
}

int mi2s_unconfig_clk_gpio(void)
{
	msm_gpios_disable_free(mi2s_clk_gpios, ARRAY_SIZE(mi2s_clk_gpios));
	return 0;
}

#endif /* CONFIG_MSM7KV2_AUDIO */

static int __init buses_init(void)
{
	if (gpio_tlmm_config(GPIO_CFG(PMIC_GPIO_INT, 1, GPIO_CFG_INPUT,
				  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE))
		pr_err("%s: gpio_tlmm_config (gpio=%d) failed\n",
		       __func__, PMIC_GPIO_INT);

	if (machine_is_msm8x60_fluid())
		pm8058_7x30_data.keypad_pdata = &fluid_keypad_data;
	else
		pm8058_7x30_data.keypad_pdata = &surf_keypad_data;

	return 0;
}

/* FUJITSU:2011-12-01 del TIMPANI RESET start */
#if 0
#define TIMPANI_RESET_GPIO	1
#endif
/* FUJITSU:2011-12-01 del TIMPANI RESET end */

struct bahama_config_register{
	u8 reg;
	u8 value;
	u8 mask;
};

enum version{
	VER_1_0,
	VER_2_0,
	VER_UNSUPPORTED = 0xFF
};

static struct regulator *vreg_marimba_1;
static struct regulator *vreg_marimba_2;
static struct regulator *vreg_bahama;

/* FUJITSU:2011-12-01 del TIMPANI RESET start */
#if 0
static struct msm_gpio timpani_reset_gpio_cfg[] = {
{ GPIO_CFG(TIMPANI_RESET_GPIO, 0, GPIO_CFG_OUTPUT,
	GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "timpani_reset"} };
#endif
/* FUJITSU:2011-12-01 del TIMPANI RESET end */

static u8 read_bahama_ver(void)
{
	int rc;
	struct marimba config = { .mod_id = SLAVE_ID_BAHAMA };
	u8 bahama_version;

	rc = marimba_read_bit_mask(&config, 0x00,  &bahama_version, 1, 0x1F);
	if (rc < 0) {
		printk(KERN_ERR
			 "%s: version read failed: %d\n",
			__func__, rc);
			return rc;
	} else {
		printk(KERN_INFO
		"%s: version read got: 0x%x\n",
		__func__, bahama_version);
	}

	switch (bahama_version) {
	case 0x08: /* varient of bahama v1 */
	case 0x10:
	case 0x00:
		return VER_1_0;
	case 0x09: /* variant of bahama v2 */
		return VER_2_0;
	default:
		return VER_UNSUPPORTED;
	}
}
/* FUJITSU:2011-12-01 del TIMPANI RESET start */
#if 0
static int config_timpani_reset(void)
{
	int rc;

	rc = msm_gpios_request_enable(timpani_reset_gpio_cfg,
				ARRAY_SIZE(timpani_reset_gpio_cfg));
	if (rc < 0) {
		printk(KERN_ERR
			"%s: msm_gpios_request_enable failed (%d)\n",
				__func__, rc);
	}
	return rc;
}
#endif
/* FUJITSU:2011-12-01 del TIMPANI RESET end */
static unsigned int msm_timpani_setup_power(void)
{
	int rc;
/* FUJITSU:2011-12-01 del TIMPANI RESET start */
#if 0
	rc = config_timpani_reset();
	if (rc < 0)
		goto out;
#endif
/* FUJITSU:2011-12-01 del TIMPANI RESET end */

	rc = regulator_enable(vreg_marimba_1);
	if (rc) {
		pr_err("%s: regulator_enable failed (%d)\n", __func__, rc);
		goto out;
	}

	rc = regulator_enable(vreg_marimba_2);
	if (rc) {
		pr_err("%s: regulator_enable failed (%d)\n", __func__, rc);
		goto disable_marimba_1;
	}
/* FUJITSU:2011-12-01 del TIMPANI RESET start */
#if 0
	rc = gpio_direction_output(TIMPANI_RESET_GPIO, 1);
	if (rc < 0) {
		pr_err("%s: gpio_direction_output failed (%d)\n",
				__func__, rc);
		msm_gpios_free(timpani_reset_gpio_cfg,
				ARRAY_SIZE(timpani_reset_gpio_cfg));
		goto disable_marimba_2;
	}
		goto out;
#else
	if(1) goto out;
#endif
/* FUJITSU:2011-12-01 del TIMPANI RESET end */
	return 0;

/* FUJITSU:2011-12-01 del TIMPANI RESET start */
#if 0
disable_marimba_2:
	regulator_disable(vreg_marimba_2);
#endif
/* FUJITSU:2011-12-01 del TIMPANI RESET end */
disable_marimba_1:
	regulator_disable(vreg_marimba_1);
out:
	return rc;
};

static void msm_timpani_shutdown_power(void)
{
	int rc;

	rc = regulator_disable(vreg_marimba_2);
	if (rc)
		pr_err("%s: regulator_disable failed (%d)\n", __func__, rc);

	rc = regulator_disable(vreg_marimba_1);
	if (rc)
		pr_err("%s: regulator_disable failed (%d)\n", __func__, rc);

/* FUJITSU:2011-12-01 del TIMPANI RESET start */
#if 0
	rc = gpio_direction_output(TIMPANI_RESET_GPIO, 0);
	if (rc < 0)
		pr_err("%s: gpio_direction_output failed (%d)\n",
				__func__, rc);

	msm_gpios_free(timpani_reset_gpio_cfg,
				   ARRAY_SIZE(timpani_reset_gpio_cfg));
#endif
/* FUJITSU:2011-12-01 del TIMPANI RESET end */
};

static unsigned int msm_bahama_core_config(int type)
{
	int rc = 0;

	if (type == BAHAMA_ID) {

		int i;
		struct marimba config = { .mod_id = SLAVE_ID_BAHAMA };

		const struct bahama_config_register v20_init[] = {
			/* reg, value, mask */
			{ 0xF4, 0x84, 0xFF }, /* AREG */
			{ 0xF0, 0x04, 0xFF } /* DREG */
		};

		if (read_bahama_ver() == VER_2_0) {
			for (i = 0; i < ARRAY_SIZE(v20_init); i++) {
				u8 value = v20_init[i].value;
				rc = marimba_write_bit_mask(&config,
					v20_init[i].reg,
					&value,
					sizeof(v20_init[i].value),
					v20_init[i].mask);
				if (rc < 0) {
					printk(KERN_ERR
						"%s: reg %d write failed: %d\n",
						__func__, v20_init[i].reg, rc);
					return rc;
				}
				printk(KERN_INFO "%s: reg 0x%02x value 0x%02x"
					" mask 0x%02x\n",
					__func__, v20_init[i].reg,
					v20_init[i].value, v20_init[i].mask);
			}
		}
	}
	printk(KERN_INFO "core type: %d\n", type);

	return rc;
}

static unsigned int msm_bahama_setup_power(void)
{
	int rc = regulator_enable(vreg_bahama);

	if (rc)
		pr_err("%s: regulator_enable failed (%d)\n", __func__, rc);

	return rc;
};

static unsigned int msm_bahama_shutdown_power(int value)
{
	int rc = 0;

	if (value != BAHAMA_ID) {
		rc = regulator_disable(vreg_bahama);

		if (rc)
			pr_err("%s: regulator_disable failed (%d)\n",
					__func__, rc);
	}

	return rc;
};

static struct msm_gpio marimba_svlte_config_clock[] = {
	{ GPIO_CFG(34, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
		"MARIMBA_SVLTE_CLOCK_ENABLE" },
};

static unsigned int msm_marimba_gpio_config_svlte(int gpio_cfg_marimba)
{
	if (machine_is_msm8x55_svlte_surf() ||
		machine_is_msm8x55_svlte_ffa()) {
		if (gpio_cfg_marimba)
			gpio_set_value(GPIO_PIN
				(marimba_svlte_config_clock->gpio_cfg), 1);
		else
			gpio_set_value(GPIO_PIN
				(marimba_svlte_config_clock->gpio_cfg), 0);
	}

	return 0;
};

static unsigned int msm_marimba_setup_power(void)
{
	int rc;

	rc = regulator_enable(vreg_marimba_1);
	if (rc) {
		pr_err("%s: regulator_enable failed (%d)\n", __func__, rc);
		goto out;
	}

	rc = regulator_enable(vreg_marimba_2);
	if (rc) {
		pr_err("%s: regulator_enable failed (%d)\n", __func__, rc);
		goto disable_marimba_1;
	}

	if (machine_is_msm8x55_svlte_surf() || machine_is_msm8x55_svlte_ffa()) {
		rc = msm_gpios_request_enable(marimba_svlte_config_clock,
				ARRAY_SIZE(marimba_svlte_config_clock));
		if (rc < 0) {
			pr_err("%s: msm_gpios_request_enable failed (%d)\n",
					__func__, rc);
			goto disable_marimba_2;
		}

		rc = gpio_direction_output(GPIO_PIN
			(marimba_svlte_config_clock->gpio_cfg), 0);
		if (rc < 0) {
			pr_err("%s: gpio_direction_output failed (%d)\n",
					__func__, rc);
			goto disable_marimba_2;
		}
	}

	return 0;

disable_marimba_2:
	regulator_disable(vreg_marimba_2);
disable_marimba_1:
	regulator_disable(vreg_marimba_1);
out:
	return rc;
};

static void msm_marimba_shutdown_power(void)
{
	int rc;

	rc = regulator_disable(vreg_marimba_2);
	if (rc)
		pr_err("%s: regulator_disable failed (%d)\n", __func__, rc);

	rc = regulator_disable(vreg_marimba_1);
	if (rc)
		pr_err("%s: regulator_disable failed (%d)\n", __func__, rc);
};

/* FUJITSU:2011-12-01 del bahama function start */
#if 0
static int bahama_present(void)
{
	int id;
	switch (id = adie_get_detected_connectivity_type()) {
	case BAHAMA_ID:
		return 1;

	case MARIMBA_ID:
		return 0;

	case TIMPANI_ID:
	default:
	printk(KERN_ERR "%s: unexpected adie connectivity type: %d\n",
			__func__, id);
	return -ENODEV;
	}
}
#endif
/* FUJITSU:2011-12-01 del bahama function start */
/* FUJITSU:2011-12-01 del FM start */
#if 0
struct regulator *fm_regulator;
static int fm_radio_setup(struct marimba_fm_platform_data *pdata)
{
	int rc, voltage;
	uint32_t irqcfg;
	const char *id = "FMPW";

	int bahama_not_marimba = bahama_present();

	if (bahama_not_marimba < 0) {
		pr_warn("%s: bahama_present: %d\n",
				__func__, bahama_not_marimba);
		rc = -ENODEV;
		goto out;
	}
	if (bahama_not_marimba) {
		fm_regulator = regulator_get(NULL, "s3");
		voltage = 1800000;
	} else {
		fm_regulator = regulator_get(NULL, "s2");
		voltage = 1300000;
	}

	if (IS_ERR(fm_regulator)) {
		rc = PTR_ERR(fm_regulator);
		pr_err("%s: regulator_get failed (%d)\n", __func__, rc);
		goto out;
	}

	rc = regulator_set_voltage(fm_regulator, voltage, voltage);

	if (rc) {
		pr_err("%s: regulator_set_voltage failed (%d)\n", __func__, rc);
		goto regulator_free;
	}

	rc = regulator_enable(fm_regulator);

	if (rc) {
		pr_err("%s: regulator_enable failed (%d)\n", __func__, rc);
		goto regulator_free;
	}

	rc = pmapp_clock_vote(id, PMAPP_CLOCK_ID_DO, PMAPP_CLOCK_VOTE_ON);

	if (rc < 0) {
		pr_err("%s: clock vote failed (%d)\n", __func__, rc);
		goto regulator_disable;
	}

	/*Request the Clock Using GPIO34/AP2MDM_MRMBCK_EN in case
	of svlte*/
	if (machine_is_msm8x55_svlte_surf() || machine_is_msm8x55_svlte_ffa()) {
		rc = marimba_gpio_config(1);
		if (rc < 0) {
			pr_err("%s: clock enable for svlte : %d\n",
					__func__, rc);
			goto clock_devote;
		}
	}
	irqcfg = GPIO_CFG(147, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL,
					GPIO_CFG_2MA);
	rc = gpio_tlmm_config(irqcfg, GPIO_CFG_ENABLE);
	if (rc) {
		pr_err("%s: gpio_tlmm_config(%#x)=%d\n", __func__, irqcfg, rc);
		rc = -EIO;
		goto gpio_deconfig;

	}
	return 0;

gpio_deconfig:
	if (machine_is_msm8x55_svlte_surf() || machine_is_msm8x55_svlte_ffa())
		marimba_gpio_config(0);
clock_devote:
	pmapp_clock_vote(id, PMAPP_CLOCK_ID_DO, PMAPP_CLOCK_VOTE_OFF);
regulator_disable:
	regulator_disable(fm_regulator);
regulator_free:
	regulator_put(fm_regulator);
	fm_regulator = NULL;
out:
	return rc;
};

static void fm_radio_shutdown(struct marimba_fm_platform_data *pdata)
{
	int rc;
	const char *id = "FMPW";
	uint32_t irqcfg = GPIO_CFG(147, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP,
					GPIO_CFG_2MA);

	int bahama_not_marimba = bahama_present();
	if (bahama_not_marimba == -1) {
		pr_warn("%s: bahama_present: %d\n",
				__func__, bahama_not_marimba);
		return;
	}

	rc = gpio_tlmm_config(irqcfg, GPIO_CFG_ENABLE);
	if (rc) {
		pr_err("%s: gpio_tlmm_config(%#x)=%d\n", __func__, irqcfg, rc);
	}
	if (!IS_ERR_OR_NULL(fm_regulator)) {
		rc = regulator_disable(fm_regulator);

		if (rc)
			pr_err("%s: return val: %d\n", __func__, rc);

		regulator_put(fm_regulator);
		fm_regulator = NULL;
	}
	rc = pmapp_clock_vote(id, PMAPP_CLOCK_ID_DO,
					  PMAPP_CLOCK_VOTE_OFF);
	if (rc < 0)
		pr_err("%s: clock_vote return val: %d\n", __func__, rc);

	/*Disable the Clock Using GPIO34/AP2MDM_MRMBCK_EN in case
	of svlte*/
	if (machine_is_msm8x55_svlte_surf() || machine_is_msm8x55_svlte_ffa()) {
		rc = marimba_gpio_config(0);
		if (rc < 0)
			pr_err("%s: clock disable for svlte : %d\n",
					__func__, rc);
	}
}

static struct marimba_fm_platform_data marimba_fm_pdata = {
	.fm_setup =  fm_radio_setup,
	.fm_shutdown = fm_radio_shutdown,
	.irq = MSM_GPIO_TO_INT(147),
	.vreg_s2 = NULL,
	.vreg_xo_out = NULL,
	.is_fm_soc_i2s_master = false,
	.config_i2s_gpio = NULL,
};
#endif
/* FUJITSU:2011-12-01 del FM end */

/* Slave id address for FM/CDC/QMEMBIST
 * Values can be programmed using Marimba slave id 0
 * should there be a conflict with other I2C devices
 * */
/* FUJITSU:2011-12-01 del FM start */
#if 0
#define MARIMBA_SLAVE_ID_FM_ADDR	0x2A
#endif
/* FUJITSU:2011-12-01 del FM end */
#define MARIMBA_SLAVE_ID_CDC_ADDR	0x77
#define MARIMBA_SLAVE_ID_QMEMBIST_ADDR	0X66

/* FUJITSU:2011-12-01 del FM start */
#if 0
#define BAHAMA_SLAVE_ID_FM_ADDR         0x2A
#endif
/* FUJITSU:2011-12-01 del FM end */
#define BAHAMA_SLAVE_ID_QMEMBIST_ADDR   0x7B

/* FUJITSU:2011-12-01 del TSADC start */
#if 0
static const char *tsadc_id = "MADC";

static struct regulator_bulk_data regs_tsadc_marimba[] = {
	{ .supply = "gp12", .min_uV = 2200000, .max_uV = 2200000 },
	{ .supply = "s2",   .min_uV = 1300000, .max_uV = 1300000 },
};

static struct regulator_bulk_data regs_tsadc_timpani[] = {
	{ .supply = "s3",   .min_uV = 1800000, .max_uV = 1800000 },
	{ .supply = "gp12", .min_uV = 2200000, .max_uV = 2200000 },
	{ .supply = "gp16", .min_uV = 1200000, .max_uV = 1200000 },
};

static struct regulator_bulk_data *regs_tsadc;
static int regs_tsadc_count;

static int marimba_tsadc_power(int vreg_on)
{
	int rc = 0;
	int tsadc_adie_type = adie_get_detected_codec_type();

	switch (tsadc_adie_type) {
	case TIMPANI_ID:
		rc = pmapp_clock_vote(tsadc_id, PMAPP_CLOCK_ID_D1,
			vreg_on ? PMAPP_CLOCK_VOTE_ON : PMAPP_CLOCK_VOTE_OFF);
		if (rc)	{
			pr_err("%s: unable to %svote for d1 clk\n",
				__func__, vreg_on ? "" : "de-");
			goto D1_vote_fail;
		}

		/* fall through */
	case MARIMBA_ID:
		rc = pmapp_clock_vote(tsadc_id, PMAPP_CLOCK_ID_DO,
			vreg_on ? PMAPP_CLOCK_VOTE_ON : PMAPP_CLOCK_VOTE_OFF);
		if (rc)	{
			pr_err("%s: unable to %svote for d1 clk\n",
				__func__, vreg_on ? "" : "de-");
			goto D0_vote_fail;
		}

		WARN_ON(regs_tsadc_count == 0);

		rc = vreg_on ?
			regulator_bulk_enable(regs_tsadc_count, regs_tsadc) :
			regulator_bulk_disable(regs_tsadc_count, regs_tsadc);

		if (rc) {
			pr_err("%s: regulator %sable failed: %d\n",
					__func__, vreg_on ? "en" : "dis", rc);
			goto regulator_switch_fail;
		}

		break;
	default:
		pr_err("%s:Adie %d not supported\n",
				__func__, tsadc_adie_type);
		return -ENODEV;
	}

	msleep(5); /* ensure power is stable */

	return 0;

regulator_switch_fail:
	pmapp_clock_vote(tsadc_id, PMAPP_CLOCK_ID_DO,
		vreg_on ? PMAPP_CLOCK_VOTE_OFF : PMAPP_CLOCK_VOTE_ON);
D0_vote_fail:
	if (tsadc_adie_type == TIMPANI_ID)
		pmapp_clock_vote(tsadc_id, PMAPP_CLOCK_ID_D1,
			vreg_on ? PMAPP_CLOCK_VOTE_OFF : PMAPP_CLOCK_VOTE_ON);
D1_vote_fail:
	return rc;
}

static int marimba_tsadc_init(void)
{
	int rc = 0;
	int tsadc_adie_type = adie_get_detected_codec_type();

	switch (tsadc_adie_type) {
	case MARIMBA_ID:
		regs_tsadc = regs_tsadc_marimba;
		regs_tsadc_count = ARRAY_SIZE(regs_tsadc_marimba);
		break;
	case TIMPANI_ID:
		regs_tsadc = regs_tsadc_timpani;
		regs_tsadc_count = ARRAY_SIZE(regs_tsadc_timpani);
		break;
	default:
		pr_err("%s:Adie %d not supported\n",
				__func__, tsadc_adie_type);
		rc = -ENODEV;
		goto out;
	}

	rc = regulator_bulk_get(NULL, regs_tsadc_count, regs_tsadc);
	if (rc) {
		pr_err("%s: could not get regulators: %d\n",
				__func__, rc);
		goto out;
	}

	rc = regulator_bulk_set_voltage(regs_tsadc_count, regs_tsadc);
	if (rc) {
		pr_err("%s: could not set regulator voltages: %d\n",
				__func__, rc);
		goto vreg_free;
	}

	return 0;

vreg_free:
	regulator_bulk_free(regs_tsadc_count, regs_tsadc);
out:
	regs_tsadc = NULL;
	regs_tsadc_count = 0;
	return rc;
}

static int marimba_tsadc_exit(void)
{
	regulator_bulk_free(regs_tsadc_count, regs_tsadc);
	regs_tsadc_count = 0;
	regs_tsadc = NULL;

	return 0;
}


static struct msm_ts_platform_data msm_ts_data = {
	.min_x          = 284,
	.max_x          = 3801,
	.min_y          = 155,
	.max_y          = 3929,
	.min_press      = 0,
	.max_press      = 255,
	.inv_x          = 4096,
	.inv_y          = 4096,
	.can_wakeup	= false,
};

static struct marimba_tsadc_platform_data marimba_tsadc_pdata = {
	.marimba_tsadc_power =  marimba_tsadc_power,
	.init		     =  marimba_tsadc_init,
	.exit		     =  marimba_tsadc_exit,
	.tsadc_prechg_en = true,
	.can_wakeup	= false,
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
	.tssc_data = &msm_ts_data,
};
#endif
/* FUJITSU:2011-12-01 del TSADC end */

static struct regulator_bulk_data codec_regs[] = {
	{ .supply = "s4", .min_uV = 2200000, .max_uV = 2200000 },
};

static int __init msm_marimba_codec_init(void)
{
	int rc = regulator_bulk_get(NULL, ARRAY_SIZE(codec_regs), codec_regs);

	if (rc) {
		pr_err("%s: could not get regulators: %d\n", __func__, rc);
		goto out;
	}

	rc = regulator_bulk_set_voltage(ARRAY_SIZE(codec_regs), codec_regs);
	if (rc) {
		pr_err("%s: could not set regulator voltages: %d\n",
				__func__, rc);
		goto reg_free;
	}

	return rc;

reg_free:
	regulator_bulk_free(ARRAY_SIZE(codec_regs), codec_regs);
out:
	return rc;
}

static int msm_marimba_codec_power(int vreg_on)
{
	int rc = vreg_on ?
		regulator_bulk_enable(ARRAY_SIZE(codec_regs), codec_regs) :
		regulator_bulk_disable(ARRAY_SIZE(codec_regs), codec_regs);

	if (rc) {
		pr_err("%s: could not %sable regulators: %d",
				__func__, vreg_on ? "en" : "dis", rc);
		return rc;
	}

	return 0;
}

static struct marimba_codec_platform_data mariba_codec_pdata = {
	.marimba_codec_power =  msm_marimba_codec_power,
#ifdef CONFIG_MARIMBA_CODEC
	.snddev_profile_init = msm_snddev_init,
#endif
};

static struct marimba_platform_data marimba_pdata = {
/* FUJITSU:2011-12-01 del FM start */
#if 0
	.slave_id[MARIMBA_SLAVE_ID_FM]       = MARIMBA_SLAVE_ID_FM_ADDR,
#endif
/* FUJITSU:2011-12-01 del FM end */
	.slave_id[MARIMBA_SLAVE_ID_CDC]	     = MARIMBA_SLAVE_ID_CDC_ADDR,
	.slave_id[MARIMBA_SLAVE_ID_QMEMBIST] = MARIMBA_SLAVE_ID_QMEMBIST_ADDR,
/* FUJITSU:2011-12-01 del FM start */
#if 0
	.slave_id[SLAVE_ID_BAHAMA_FM]        = BAHAMA_SLAVE_ID_FM_ADDR,
#endif
/* FUJITSU:2011-12-01 del FM end */
	.slave_id[SLAVE_ID_BAHAMA_QMEMBIST]  = BAHAMA_SLAVE_ID_QMEMBIST_ADDR,
	.marimba_setup = msm_marimba_setup_power,
	.marimba_shutdown = msm_marimba_shutdown_power,
	.bahama_setup = msm_bahama_setup_power,
	.bahama_shutdown = msm_bahama_shutdown_power,
	.marimba_gpio_config = msm_marimba_gpio_config_svlte,
	.bahama_core_config = msm_bahama_core_config,
/* FUJITSU:2011-12-01 del FM start */
#if 0
	.fm = &marimba_fm_pdata,
#endif
/* FUJITSU:2011-12-01 del FM end */
	.codec = &mariba_codec_pdata,
	.tsadc_ssbi_adap = MARIMBA_SSBI_ADAP,
};

static void __init msm7x30_init_marimba(void)
{
	int rc;

	struct regulator_bulk_data regs[] = {
		{ .supply = "s3",   .min_uV = 1800000, .max_uV = 1800000 },
		{ .supply = "gp16", .min_uV = 1200000, .max_uV = 1200000 },
		{ .supply = "usb2", .min_uV = 1800000, .max_uV = 1800000 },
	};

	rc = msm_marimba_codec_init();

	if (rc) {
		pr_err("%s: msm_marimba_codec_init failed (%d)\n",
				__func__, rc);
		return;
	}

	rc = regulator_bulk_get(NULL, ARRAY_SIZE(regs), regs);

	if (rc) {
		pr_err("%s: could not get regulators: %d\n", __func__, rc);
		return;
	}

	rc = regulator_bulk_set_voltage(ARRAY_SIZE(regs), regs);

	if (rc) {
		pr_err("%s: could not set voltages: %d\n", __func__, rc);
		regulator_bulk_free(ARRAY_SIZE(regs), regs);
		return;
	}

	vreg_marimba_1 = regs[0].consumer;
	vreg_marimba_2 = regs[1].consumer;
	vreg_bahama    = regs[2].consumer;
}

static struct marimba_codec_platform_data timpani_codec_pdata = {
	.marimba_codec_power =  msm_marimba_codec_power,
#ifdef CONFIG_TIMPANI_CODEC
	.snddev_profile_init = msm_snddev_init_timpani,
#endif
};

static struct marimba_platform_data timpani_pdata = {
	.slave_id[MARIMBA_SLAVE_ID_CDC]	= MARIMBA_SLAVE_ID_CDC_ADDR,
	.slave_id[MARIMBA_SLAVE_ID_QMEMBIST] = MARIMBA_SLAVE_ID_QMEMBIST_ADDR,
	.marimba_setup = msm_timpani_setup_power,
	.marimba_shutdown = msm_timpani_shutdown_power,
	.codec = &timpani_codec_pdata,
/* FUJITSU:2011-12-01 del TSADC start */
#if 0
	.tsadc = &marimba_tsadc_pdata,
#endif
/* FUJITSU:2011-12-01 del TSADC end */
	.tsadc_ssbi_adap = MARIMBA_SSBI_ADAP,
};

#define TIMPANI_I2C_SLAVE_ADDR	0xD

static struct i2c_board_info msm_i2c_gsbi7_timpani_info[] = {
	{
		I2C_BOARD_INFO("timpani", TIMPANI_I2C_SLAVE_ADDR),
		.platform_data = &timpani_pdata,
	},
};

#ifdef CONFIG_MSM7KV2_AUDIO
static struct resource msm_aictl_resources[] = {
	{
		.name = "aictl",
		.start = 0xa5000100,
		.end = 0xa5000100,
		.flags = IORESOURCE_MEM,
	}
};

static struct resource msm_mi2s_resources[] = {
/* FUJITSU:2011-12-01 del DTV start */
#if 0
	{
		.name = "hdmi",
		.start = 0xac900000,
		.end = 0xac900038,
		.flags = IORESOURCE_MEM,
	},
#endif
/* FUJITSU:2011-12-01 del DTV end */
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

struct platform_device msm_aictl_device = {
	.name = "audio_interct",
	.id   = 0,
	.num_resources = ARRAY_SIZE(msm_aictl_resources),
	.resource = msm_aictl_resources,
};

struct platform_device msm_mi2s_device = {
	.name = "mi2s",
	.id   = 0,
	.num_resources = ARRAY_SIZE(msm_mi2s_resources),
	.resource = msm_mi2s_resources,
};

struct platform_device msm_lpa_device = {
	.name = "lpa",
	.id   = 0,
	.num_resources = ARRAY_SIZE(msm_lpa_resources),
	.resource = msm_lpa_resources,
	.dev		= {
		.platform_data = &lpa_pdata,
	},
};
#endif /* CONFIG_MSM7KV2_AUDIO */

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
	(DEC3_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC2_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
	(DEC1_FORMAT|(1<<MSM_ADSP_MODE_NONTUNNEL)|(1<<MSM_ADSP_OP_DM)),
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
/* FUJUTSU:2011-12-01 add bsp-sensor start */
static struct fj_wm_platform_data fj_wm_device_platform_data = {
/* FUJITSU:2012-01-16 bsp-sensor mod start */
	.motion_irq = MSM_GPIO_TO_INT( FJ_WM_GPIO_HOSU_IRQ ),
/* FUJITSU:2012-01-16 bsp-sensor mod end */
	.mc_init_delay = 20,
};

static struct platform_device fj_walkmotion_device = {
	.name           = "fj-walkmotion",
	.id             = 0,
	.dev = {
		.platform_data = &fj_wm_device_platform_data
	},
};
/* FUJUTSU:2011-12-01 add bsp-sensor end */

static struct resource smc91x_resources[] = {
	[0] = {
		.start = 0x8A000300,
		.end = 0x8A0003ff,
		.flags  = IORESOURCE_MEM,
	},
	[1] = {
		.start = MSM_GPIO_TO_INT(156),
		.end = MSM_GPIO_TO_INT(156),
		.flags  = IORESOURCE_IRQ,
	},
};

static struct platform_device smc91x_device = {
	.name           = "smc91x",
	.id             = 0,
	.num_resources  = ARRAY_SIZE(smc91x_resources),
	.resource       = smc91x_resources,
};

static struct smsc911x_platform_config smsc911x_config = {
	.phy_interface	= PHY_INTERFACE_MODE_MII,
	.irq_polarity	= SMSC911X_IRQ_POLARITY_ACTIVE_LOW,
	.irq_type	= SMSC911X_IRQ_TYPE_PUSH_PULL,
	.flags		= SMSC911X_USE_32BIT,
};

static struct resource smsc911x_resources[] = {
	[0] = {
		.start		= 0x8D000000,
		.end		= 0x8D000100,
		.flags		= IORESOURCE_MEM,
	},
	[1] = {
		.start		= MSM_GPIO_TO_INT(88),
		.end		= MSM_GPIO_TO_INT(88),
		.flags		= IORESOURCE_IRQ,
	},
};

static struct platform_device smsc911x_device = {
	.name		= "smsc911x",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(smsc911x_resources),
	.resource	= smsc911x_resources,
	.dev		= {
		.platform_data = &smsc911x_config,
	},
};

static struct msm_gpio smsc911x_gpios[] = {
/* FUJITSU:2012-05-08 Del GPIO start */
#if 0
    { GPIO_CFG(172, 2, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), "ebi2_addr6" },
    { GPIO_CFG(173, 2, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), "ebi2_addr5" },
    { GPIO_CFG(174, 2, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), "ebi2_addr4" },
    { GPIO_CFG(175, 2, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), "ebi2_addr3" },
    { GPIO_CFG(176, 2, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), "ebi2_addr2" },
    { GPIO_CFG(177, 2, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), "ebi2_addr1" },
    { GPIO_CFG(178, 2, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), "ebi2_addr0" },
    { GPIO_CFG(88, 2, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), "smsc911x_irq"  },
#endif
/* FUJITSU:2012-05-08 Del GPIO end */
};

static void msm7x30_cfg_smsc911x(void)
{
	int rc;

	rc = msm_gpios_request_enable(smsc911x_gpios,
			ARRAY_SIZE(smsc911x_gpios));
	if (rc)
		pr_err("%s: unable to enable gpios\n", __func__);
}

#ifdef CONFIG_USB_G_ANDROID
static struct android_usb_platform_data android_usb_pdata = {
	.update_pid_and_serial_num = usb_diag_update_pid_and_serial_num,
};

static struct platform_device android_usb_device = {
	.name	= "android_usb",
	.id		= -1,
	.dev		= {
		.platform_data = &android_usb_pdata,
	},
};
#endif

/* FUJITSU:2011-12-01 del OPTNAV start */
#if 0
static struct msm_gpio optnav_config_data[] = {
	{ GPIO_CFG(OPTNAV_CHIP_SELECT, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA),
	"optnav_chip_select" },
};

static struct regulator_bulk_data optnav_regulators[] = {
	{ .supply = "gp7", .min_uV = 1800000, .max_uV = 1800000 },
	{ .supply = "gp4", .min_uV = 2600000, .max_uV = 2600000 },
	{ .supply = "gp9", .min_uV = 1800000, .max_uV = 1800000 },
	{ .supply = "usb", .min_uV = 3300000, .max_uV = 3300000 },
};

static void __iomem *virtual_optnav;

static int optnav_gpio_setup(void)
{
	int rc = -ENODEV;
	rc = msm_gpios_request_enable(optnav_config_data,
			ARRAY_SIZE(optnav_config_data));

	if (rc)
		return rc;

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

	rc = regulator_bulk_get(NULL, ARRAY_SIZE(optnav_regulators),
			optnav_regulators);
	if (rc)
		return rc;

	rc = regulator_bulk_set_voltage(ARRAY_SIZE(optnav_regulators),
			optnav_regulators);

	if (rc)
		goto regulator_put;

	return rc;

regulator_put:
	regulator_bulk_free(ARRAY_SIZE(optnav_regulators), optnav_regulators);
	return rc;
}

static void optnav_gpio_release(void)
{
	msm_gpios_disable_free(optnav_config_data,
		ARRAY_SIZE(optnav_config_data));
	iounmap(virtual_optnav);
	regulator_bulk_free(ARRAY_SIZE(optnav_regulators), optnav_regulators);
}

static int optnav_enable(void)
{
	int rc;
	/*
	 * Enable the VREGs L8(gp7), L10(gp4), L12(gp9), L6(usb)
	 * for I2C communication with keyboard.
	 */

	rc = regulator_bulk_enable(ARRAY_SIZE(optnav_regulators),
			optnav_regulators);

	if (rc)
		return rc;

	/* Enable the chip select GPIO */
	gpio_set_value(OPTNAV_CHIP_SELECT, 1);
	gpio_set_value(OPTNAV_CHIP_SELECT, 0);

	return 0;
}

static void optnav_disable(void)
{
	regulator_bulk_disable(ARRAY_SIZE(optnav_regulators),
			optnav_regulators);

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
#endif
/* FUJITSU:2011-12-01 del OPTNAV end */

/* FUJITSU:2011-12-01 del DTV start */
#if 0
static int hdmi_comm_power(int on, int show);
static int hdmi_init_irq(void);
static int hdmi_enable_5v(int on);
static int hdmi_core_power(int on, int show);
static int hdmi_cec_power(int on);
static bool hdmi_check_hdcp_hw_support(void);

static struct msm_hdmi_platform_data adv7520_hdmi_data = {
	.irq = MSM_GPIO_TO_INT(18),
	.comm_power = hdmi_comm_power,
	.init_irq = hdmi_init_irq,
	.enable_5v = hdmi_enable_5v,
	.core_power = hdmi_core_power,
	.cec_power = hdmi_cec_power,
	.check_hdcp_hw_support = hdmi_check_hdcp_hw_support,
};
#endif
/* FUJITSU:2011-12-01 del DTV end */

#ifdef CONFIG_BOSCH_BMA150

static struct regulator_bulk_data sensors_ldo[] = {
	{ .supply = "gp7", .min_uV = 1800000, .max_uV = 1800000 },
	{ .supply = "gp6", .min_uV = 3050000, .max_uV = 3100000 },
};

static int __init sensors_ldo_init(void)
{
	int rc;

	rc = regulator_bulk_get(NULL, ARRAY_SIZE(sensors_ldo), sensors_ldo);

	if (rc) {
		pr_err("%s: could not get regulators: %d\n", __func__, rc);
		goto out;
	}

	rc = regulator_bulk_set_voltage(ARRAY_SIZE(sensors_ldo), sensors_ldo);

	if (rc) {
		pr_err("%s: could not set voltages: %d\n", __func__, rc);
		goto reg_free;
	}

	return 0;

reg_free:
	regulator_bulk_free(ARRAY_SIZE(sensors_ldo), sensors_ldo);
out:
	return rc;
}

static int sensors_ldo_set(int on)
{
	int rc = on ?
		regulator_bulk_enable(ARRAY_SIZE(sensors_ldo), sensors_ldo) :
		regulator_bulk_disable(ARRAY_SIZE(sensors_ldo), sensors_ldo);

	if (rc)
		pr_err("%s: could not %sable regulators: %d\n",
				__func__, on ? "en" : "dis", rc);

	return rc;
}

static int sensors_ldo_enable(void)
{
	return sensors_ldo_set(1);
}

static void sensors_ldo_disable(void)
{
	sensors_ldo_set(0);
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
/* FUJITSU:2011-12-01 del OPTNAV start */
#if 0
	{
		I2C_BOARD_INFO("m33c01", OPTNAV_I2C_SLAVE_ADDR),
		.irq		= MSM_GPIO_TO_INT(OPTNAV_IRQ),
		.platform_data = &optnav_data,
	},
#endif
/* FUJITSU:2011-12-01 del OPTNAV end */
/* FUJITSU:2011-12-01 del DTV start */
#if 0
	{
		I2C_BOARD_INFO("adv7520", ADV7520_I2C_ADDR),
		.platform_data = &adv7520_hdmi_data,
	},
#endif
/* FUJITSU:2011-12-01 del DTV end */
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
/* FUJITSU:2012-05-03 modify disable idle power collapse start */
//		.idle_enabled = 1,
		.idle_enabled = 0,
/* FUJITSU:2012-05-03 modify disable idle power collapse end */
		.suspend_enabled = 1,
		.latency = 8594,
		.residency = 23740,
	},
	[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN] = {
		.idle_supported = 1,
		.suspend_supported = 1,
/* FUJITSU:2012-05-03 modify disable idle power collapse start */
//	    .idle_enabled = 1,
	    .idle_enabled = 0,
/* FUJITSU:2012-05-03 modify disable idle power collapse end */
		.suspend_enabled = 1,
		.latency = 4594,
		.residency = 23740,
	},
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

u32 msm7x30_power_collapse_latency(enum msm_pm_sleep_mode mode)
{
	switch (mode) {
	case MSM_PM_SLEEP_MODE_POWER_COLLAPSE:
		return msm_pm_data
		[MSM_PM_SLEEP_MODE_POWER_COLLAPSE].latency;
	case MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN:
		return msm_pm_data
		[MSM_PM_SLEEP_MODE_POWER_COLLAPSE_NO_XO_SHUTDOWN].latency;
	case MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT:
		return msm_pm_data
		[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].latency;
	case MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT:
		return msm_pm_data
		[MSM_PM_SLEEP_MODE_WAIT_FOR_INTERRUPT].latency;
	default:
	return 0;
	}
}

static struct msm_pm_boot_platform_data msm_pm_boot_pdata __initdata = {
	.mode = MSM_PM_BOOT_CONFIG_RESET_VECTOR_VIRT,
	.v_addr = (uint32_t *)PAGE_OFFSET,
};

static struct resource qsd_spi_resources[] = {
	{
		.name   = "spi_irq_in",
		.start	= INT_SPI_INPUT,
		.end	= INT_SPI_INPUT,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name   = "spi_irq_out",
		.start	= INT_SPI_OUTPUT,
		.end	= INT_SPI_OUTPUT,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name   = "spi_irq_err",
		.start	= INT_SPI_ERROR,
		.end	= INT_SPI_ERROR,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.name   = "spi_base",
		.start	= 0xA8000000,
		.end	= 0xA8000000 + SZ_4K - 1,
		.flags	= IORESOURCE_MEM,
	},
	{
		.name   = "spidm_channels",
		.flags  = IORESOURCE_DMA,
	},
	{
		.name   = "spidm_crci",
		.flags  = IORESOURCE_DMA,
	},
};

#define AMDH0_BASE_PHYS		0xAC200000
#define ADMH0_GP_CTL		(ct_adm_base + 0x3D8)
static int msm_qsd_spi_dma_config(void)
{
	void __iomem *ct_adm_base = 0;
	u32 spi_mux = 0;
	int ret = 0;

// Fujitsu 2012.01.06 WLAN/BT add start
/* FUJITSU:2011-05-06 add BT HS start */
#ifdef MSM_SPI_CRCI_DISABLE
	u32 gp_ctl = 0;
#endif
/* FUJITSU:2011-05-06 add BT HS end */
// Fujitsu 2012.01.06 WLAN/BT add end

	ct_adm_base = ioremap(AMDH0_BASE_PHYS, PAGE_SIZE);
	if (!ct_adm_base) {
		pr_err("%s: Could not remap %x\n", __func__, AMDH0_BASE_PHYS);
		return -ENOMEM;
	}

// Fujitsu 2012.01.06 WLAN/BT add start
/* FUJITSU:2011-05-06 add BT HS start */
#ifdef MSM_SPI_CRCI_DISABLE
	gp_ctl = ioread32(ADMH0_GP_CTL);

	gp_ctl = gp_ctl & 0xFFFFCFFF;
	iowrite32(gp_ctl,ADMH0_GP_CTL);

	gp_ctl = ioread32(ADMH0_GP_CTL);
#endif
/* FUJITSU:2011-05-06 add BT HS end */
// Fujitsu 2012.01.06 WLAN/BT add end

	spi_mux = (ioread32(ADMH0_GP_CTL) & (0x3 << 12)) >> 12;

	qsd_spi_resources[4].start  = DMOV_USB_CHAN;
	qsd_spi_resources[4].end    = DMOV_TSIF_CHAN;

	switch (spi_mux) {
	case (1):
		qsd_spi_resources[5].start  = DMOV_HSUART1_RX_CRCI;
		qsd_spi_resources[5].end    = DMOV_HSUART1_TX_CRCI;
		break;
	case (2):
		qsd_spi_resources[5].start  = DMOV_HSUART2_RX_CRCI;
		qsd_spi_resources[5].end    = DMOV_HSUART2_TX_CRCI;
		break;
	case (3):
		qsd_spi_resources[5].start  = DMOV_CE_OUT_CRCI;
		qsd_spi_resources[5].end    = DMOV_CE_IN_CRCI;
		break;
	default:
		ret = -ENOENT;
	}

	iounmap(ct_adm_base);

	return ret;
}

static struct platform_device qsd_device_spi = {
	.name		= "spi_qsd",
	.id		= 0,
	.num_resources	= ARRAY_SIZE(qsd_spi_resources),
	.resource	= qsd_spi_resources,
};

#ifdef CONFIG_SPI_QSD
/* FUJITSU: 2011_07_13 mod KOUTEI start */
#if	0
static struct spi_board_info lcdc_sharp_spi_board_info[] __initdata = {
	{
		.modalias	= "lcdc_sharp_ls038y7dx01",
		.mode		= SPI_MODE_1,
		.bus_num	= 0,
		.chip_select	= 0,
		.max_speed_hz	= 26331429,
	}
};
static struct spi_board_info lcdc_toshiba_spi_board_info[] __initdata = {
	{
		.modalias       = "lcdc_toshiba_ltm030dd40",
		.mode           = SPI_MODE_3|SPI_CS_HIGH,
		.bus_num        = 0,
		.chip_select    = 0,
		.max_speed_hz   = 9963243,
	}
};
#else
static struct spi_board_info lcdc_sharp_spi_board_info[] __initdata = {
	{
		.modalias	= "wm0010_wolfson",
		.mode		= SPI_MODE_1,
		.bus_num	= 0,
		.chip_select	= 0,
/*		.max_speed_hz	= 26331429, FUJITSU:2012-2-6 wolfson clk change */
		.max_speed_hz	= 25896198, /* FUJITSU:2012-2-6 wolfson clk change */
	},
    {
        .modalias    = "tmc112_firm_transfer",
        .mode        = SPI_MODE_1,
        .bus_num    = 0,
        .chip_select    = 1,
        .max_speed_hz    = 26331429,
    }
};
static struct spi_board_info lcdc_toshiba_spi_board_info[] __initdata = {
	{
		.modalias	= "wm0010_wolfson",
		.mode		= SPI_MODE_1,
		.bus_num	= 0,
		.chip_select	= 0,
/*		.max_speed_hz	= 26331429, FUJITSU:2012-2-6 wolfson clk change */
		.max_speed_hz	= 25896198, /* FUJITSU:2012-2-6 wolfson clk change */
	},
    {
        .modalias    = "tmc112_firm_transfer",
        .mode        = SPI_MODE_3|SPI_CS_HIGH,
        .bus_num    = 0,
        .chip_select    = 1,
        .max_speed_hz    = 26331429,
    }
};
#endif
/* FUJITSU: 2011_07_13 mod KOUTEI end */
#endif

static struct msm_gpio qsd_spi_gpio_config_data[] = {
/* FUJITSU:2011-12-01 mod SPI GPIO start */
#if 0
	{ GPIO_CFG(45, 1, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "spi_clk" },
	{ GPIO_CFG(46, 1, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "spi_cs0" },
	{ GPIO_CFG(47, 1, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_8MA), "spi_mosi" },
	{ GPIO_CFG(48, 1, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "spi_miso" },
#else
	{ GPIO_CFG(44, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "spi_cs1" },
	{ GPIO_CFG(45, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "spi_clk" },
	{ GPIO_CFG(46, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "spi_cs0" },
	{ GPIO_CFG(47, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "spi_mosi" },
	{ GPIO_CFG(48, 1, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "spi_miso" },
#endif
/* FUJITSU:2011-12-01 mod SPI GPIO end */
};

static int msm_qsd_spi_gpio_config(void)
{
	return msm_gpios_request_enable(qsd_spi_gpio_config_data,
		ARRAY_SIZE(qsd_spi_gpio_config_data));
}

static void msm_qsd_spi_gpio_release(void)
{
	msm_gpios_disable_free(qsd_spi_gpio_config_data,
		ARRAY_SIZE(qsd_spi_gpio_config_data));
}

static struct msm_spi_platform_data qsd_spi_pdata = {
	.max_clock_speed = 26331429,
	.gpio_config  = msm_qsd_spi_gpio_config,
	.gpio_release = msm_qsd_spi_gpio_release,
	.dma_config = msm_qsd_spi_dma_config,
};

static void __init msm_qsd_spi_init(void)
{
	qsd_device_spi.dev.platform_data = &qsd_spi_pdata;
}

#ifdef CONFIG_USB_EHCI_MSM_72K
/* FUJITSU:2012-04-03 USB start */
static int msm_hsusb_vbus_init(int init)
{
    int ret = 0;
    unsigned int is_enable = 
        (init == 1) ? GPIO_CFG_ENABLE : GPIO_CFG_DISABLE;

    printk("%s: enable = %s \n", __func__,
      (is_enable == GPIO_CFG_ENABLE) ? "GPIO_CFG_ENABLE" : "GPIO_CFG_DISABLE");

    if (gpio_tlmm_config( GPIO_CFG(165, 0, GPIO_CFG_OUTPUT,
              GPIO_CFG_NO_PULL, GPIO_CFG_2MA), is_enable)){
        ret = 1;
        pr_err("%s: gpio_tlmm_config (gpio=%d) failed\n",
               __func__, 165);
    }
    return ret;
}
/* FUJITSU:2012-04-03 USB end */

static void msm_hsusb_vbus_power(unsigned phy_info, int on)
{
/* FUJITSU:2012-04-03 USB start */
    static int vbus_is_on;
    /* If VBUS is already on (or off), do nothing. */
    if (unlikely(on == vbus_is_on)) {
        pr_info("%s: vbus is already %s\n", __func__, (vbus_is_on ? "on" : "off"));
        return;
    }
    gpio_set_value(165, on);
    vbus_is_on = on;
    pr_info("%s: set vbus %s\n", __func__, (vbus_is_on ? "on" : "off"));
/* FUJITSU:2012-04-03 USB end */
}

static struct msm_usb_host_platform_data msm_usb_host_pdata = {
        .phy_info   = (USB_PHY_INTEGRATED | USB_PHY_MODEL_45NM),
        .vbus_power = msm_hsusb_vbus_power,
        .power_budget   = 180,
/* FUJITSU:2012-04-03 USB start */
        .vbus_init = msm_hsusb_vbus_init,
/* FUJITSU:2012-04-03 USB end */
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
static struct regulator *vreg_3p3;
static int msm_hsusb_ldo_init(int init)
{
	uint32_t version = 0;
	int def_vol = 3400000;

	version = socinfo_get_version();

	if (SOCINFO_VERSION_MAJOR(version) >= 2 &&
			SOCINFO_VERSION_MINOR(version) >= 1) {
		def_vol = 3075000;
		pr_debug("%s: default voltage:%d\n", __func__, def_vol);
	}

	if (init) {
		vreg_3p3 = regulator_get(NULL, "usb");
		if (IS_ERR(vreg_3p3))
			return PTR_ERR(vreg_3p3);
		regulator_set_voltage(vreg_3p3, def_vol, def_vol);
	} else
		regulator_put(vreg_3p3);

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
		return regulator_enable(vreg_3p3);
	else
		return regulator_disable(vreg_3p3);
}

static int msm_hsusb_ldo_set_voltage(int mV)
{
	static int cur_voltage;

	if (!vreg_3p3 || IS_ERR(vreg_3p3))
		return -ENODEV;

	if (cur_voltage == mV)
		return 0;

	cur_voltage = mV;

	pr_debug("%s: (%d)\n", __func__, mV);

	return regulator_set_voltage(vreg_3p3, mV*1000, mV*1000);
}
#endif

#ifndef CONFIG_USB_EHCI_MSM_72K
static int msm_hsusb_pmic_notif_init(void (*callback)(int online), int init);
#endif
static struct msm_otg_platform_data msm_otg_pdata = {
	.rpc_connect	= hsusb_rpc_connect,

#ifndef CONFIG_USB_EHCI_MSM_72K
	.pmic_vbus_notif_init         = msm_hsusb_pmic_notif_init,
#else
	.vbus_power = msm_hsusb_vbus_power,
/* FUJITSU:2012-02-23 USB start */
	.phy_power = hsusb_notifi_usb_ctrl_power,
	.atom_notif_init = hsusb_cable_det_notifi_initialize_complete,
	.atom_notif_cable_disconnect = hsusb_cable_det_notifi_disconnect,
	.atom_notif_usb_vdd_off = hsusb_cable_usb_vdd_off,
/* FUJITSU:2012-02-23 USB end */
#endif
	.pemp_level		 = PRE_EMPHASIS_WITH_20_PERCENT,
	.cdr_autoreset		 = CDR_AUTO_RESET_DISABLE,
	.drv_ampl		 = HS_DRV_AMPLITUDE_DEFAULT,
	.se1_gating		 = SE1_GATING_DISABLE,
	.chg_vbus_draw		 = hsusb_chg_vbus_draw,
	.chg_connected		 = hsusb_chg_connected,
	.chg_init		 = hsusb_chg_init,
	.ldo_enable		 = msm_hsusb_ldo_enable,
	.ldo_init		 = msm_hsusb_ldo_init,
	.ldo_set_voltage	 = msm_hsusb_ldo_set_voltage,
};
static struct platform_device memread_dev = {
	.name = "mread",
};
#ifdef CONFIG_USB_GADGET
static struct msm_hsusb_gadget_platform_data msm_gadget_pdata = {
	.is_phy_status_timer_on = 1,
};
#endif
#ifndef CONFIG_USB_EHCI_MSM_72K
typedef void (*notify_vbus_state) (int);
notify_vbus_state notify_vbus_state_func_ptr;
int vbus_on_irq;
static irqreturn_t pmic_vbus_on_irq(int irq, void *data)
{
	pr_info("%s: vbus notification from pmic\n", __func__);

	(*notify_vbus_state_func_ptr) (1);

	return IRQ_HANDLED;
}
static int msm_hsusb_pmic_notif_init(void (*callback)(int online), int init)
{
	int ret;

	if (init) {
		if (!callback)
			return -ENODEV;

		notify_vbus_state_func_ptr = callback;
		vbus_on_irq = platform_get_irq_byname(&msm_device_otg,
			"vbus_on");
		if (vbus_on_irq <= 0) {
			pr_err("%s: unable to get vbus on irq\n", __func__);
			return -ENODEV;
		}

		ret = request_any_context_irq(vbus_on_irq, pmic_vbus_on_irq,
			IRQF_TRIGGER_RISING, "msm_otg_vbus_on", NULL);
		if (ret < 0) {
			pr_info("%s: request_irq for vbus_on"
				"interrupt failed\n", __func__);
			return ret;
		}
		msm_otg_pdata.pmic_vbus_irq = vbus_on_irq;
		return 0;
	} else {
		free_irq(vbus_on_irq, 0);
		notify_vbus_state_func_ptr = NULL;
		return 0;
	}
}
#endif

static struct android_pmem_platform_data android_pmem_pdata = {
	.name = "pmem",
	.allocator_type = PMEM_ALLOCATORTYPE_ALLORNOTHING,
	.cached = 1,
	.memory_type = MEMTYPE_EBI0,
};

static struct platform_device android_pmem_device = {
	.name = "android_pmem",
	.id = 0,
	.dev = { .platform_data = &android_pmem_pdata },
};

/* FUJITSU:2011-12-22 del LCDC start */
#if 0
#ifndef CONFIG_SPI_QSD
static int lcdc_gpio_array_num[] = {
				45, /* spi_clk */
				46, /* spi_cs  */
				47, /* spi_mosi */
				48, /* spi_miso */
				};

static struct msm_gpio lcdc_gpio_config_data[] = {
	{ GPIO_CFG(45, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "spi_clk" },
	{ GPIO_CFG(46, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "spi_cs0" },
	{ GPIO_CFG(47, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "spi_mosi" },
	{ GPIO_CFG(48, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "spi_miso" },
};

static void lcdc_config_gpios(int enable)
{
	if (enable) {
		msm_gpios_request_enable(lcdc_gpio_config_data,
					      ARRAY_SIZE(
						      lcdc_gpio_config_data));
	} else
		msm_gpios_disable_free(lcdc_gpio_config_data,
					    ARRAY_SIZE(
						    lcdc_gpio_config_data));
}
#endif
#endif
/* FUJITSU:2011-12-22 del LCDC end */

/* FUJITSU:2011-12-01 del spi devices start */
#if 0
static struct msm_panel_common_pdata lcdc_sharp_panel_data = {
#ifndef CONFIG_SPI_QSD
	.panel_config_gpio = lcdc_config_gpios,
	.gpio_num          = lcdc_gpio_array_num,
#endif
	.gpio = 2, 	/* LPG PMIC_GPIO26 channel number */
};

static struct platform_device lcdc_sharp_panel_device = {
	.name   = "lcdc_sharp_wvga",
	.id     = 0,
	.dev    = {
		.platform_data = &lcdc_sharp_panel_data,
	}
};
#endif
/* FUJITSU:2011-12-01 del spi devices end */

/* FUJITSU:2011-12-01 del DTV start */
#if 0
static struct msm_gpio dtv_panel_irq_gpios[] = {
	{ GPIO_CFG(18, 0, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_4MA),
		"hdmi_int" },
};

static struct msm_gpio dtv_panel_gpios[] = {
	{ GPIO_CFG(120, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_4MA), "wca_mclk" },
	{ GPIO_CFG(121, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_4MA), "wca_sd0" },
	{ GPIO_CFG(122, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_4MA), "wca_sd1" },
	{ GPIO_CFG(123, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_4MA), "wca_sd2" },
	{ GPIO_CFG(124, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_8MA), "dtv_pclk" },
	{ GPIO_CFG(125, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_4MA), "dtv_en" },
	{ GPIO_CFG(126, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_4MA), "dtv_vsync" },
	{ GPIO_CFG(127, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_4MA), "dtv_hsync" },
	{ GPIO_CFG(128, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_4MA), "dtv_data0" },
	{ GPIO_CFG(129, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_4MA), "dtv_data1" },
	{ GPIO_CFG(130, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_4MA), "dtv_data2" },
	{ GPIO_CFG(131, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_4MA), "dtv_data3" },
	{ GPIO_CFG(132, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_4MA), "dtv_data4" },
	{ GPIO_CFG(160, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_4MA), "dtv_data5" },
	{ GPIO_CFG(161, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_4MA), "dtv_data6" },
	{ GPIO_CFG(162, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_4MA), "dtv_data7" },
	{ GPIO_CFG(163, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_4MA), "dtv_data8" },
	{ GPIO_CFG(164, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_4MA), "dtv_data9" },
	{ GPIO_CFG(165, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_4MA), "dtv_dat10" },
	{ GPIO_CFG(166, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_4MA), "dtv_dat11" },
	{ GPIO_CFG(167, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_4MA), "dtv_dat12" },
	{ GPIO_CFG(168, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_4MA), "dtv_dat13" },
	{ GPIO_CFG(169, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_4MA), "dtv_dat14" },
	{ GPIO_CFG(170, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_4MA), "dtv_dat15" },
	{ GPIO_CFG(171, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_4MA), "dtv_dat16" },
	{ GPIO_CFG(172, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_4MA), "dtv_dat17" },
	{ GPIO_CFG(173, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_4MA), "dtv_dat18" },
	{ GPIO_CFG(174, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_4MA), "dtv_dat19" },
	{ GPIO_CFG(175, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_4MA), "dtv_dat20" },
	{ GPIO_CFG(176, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_4MA), "dtv_dat21" },
	{ GPIO_CFG(177, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_4MA), "dtv_dat22" },
	{ GPIO_CFG(178, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_4MA), "dtv_dat23" },
};


#ifdef HDMI_RESET
static unsigned dtv_reset_gpio =
	GPIO_CFG(37, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA);
#endif

static struct regulator_bulk_data hdmi_core_regs[] = {
	{ .supply = "ldo8",  .min_uV = 1800000, .max_uV = 1800000 },
};

static struct regulator_bulk_data hdmi_comm_regs[] = {
	{ .supply = "ldo8",  .min_uV = 1800000, .max_uV = 1800000 },
	{ .supply = "ldo10", .min_uV = 2600000, .max_uV = 2600000 },
};

static struct regulator_bulk_data hdmi_cec_regs[] = {
	{ .supply = "ldo17", .min_uV = 2600000, .max_uV = 2600000 },
};

static int __init hdmi_init_regs(void)
{
	int rc;

	rc = regulator_bulk_get(NULL, ARRAY_SIZE(hdmi_core_regs),
			hdmi_core_regs);

	if (rc) {
		pr_err("%s: could not get %s regulators: %d\n",
				__func__, "core", rc);
		goto out;
	}

	rc = regulator_bulk_set_voltage(ARRAY_SIZE(hdmi_core_regs),
			hdmi_core_regs);

	if (rc) {
		pr_err("%s: could not set %s voltages: %d\n",
				__func__, "core", rc);
		goto free_core;
	}

	rc = regulator_bulk_get(NULL, ARRAY_SIZE(hdmi_comm_regs),
			hdmi_comm_regs);

	if (rc) {
		pr_err("%s: could not get %s regulators: %d\n",
				__func__, "comm", rc);
		goto free_core;
	}

	rc = regulator_bulk_set_voltage(ARRAY_SIZE(hdmi_comm_regs),
			hdmi_comm_regs);

	if (rc) {
		pr_err("%s: could not set %s voltages: %d\n",
				__func__, "comm", rc);
		goto free_comm;
	}

	rc = regulator_bulk_get(NULL, ARRAY_SIZE(hdmi_cec_regs),
			hdmi_cec_regs);

	if (rc) {
		pr_err("%s: could not get %s regulators: %d\n",
				__func__, "cec", rc);
		goto free_comm;
	}

	rc = regulator_bulk_set_voltage(ARRAY_SIZE(hdmi_cec_regs),
			hdmi_cec_regs);

	if (rc) {
		pr_err("%s: could not set %s voltages: %d\n",
				__func__, "cec", rc);
		goto free_cec;
	}

	return 0;

free_cec:
	regulator_bulk_free(ARRAY_SIZE(hdmi_cec_regs), hdmi_cec_regs);
free_comm:
	regulator_bulk_free(ARRAY_SIZE(hdmi_comm_regs), hdmi_comm_regs);
free_core:
	regulator_bulk_free(ARRAY_SIZE(hdmi_core_regs), hdmi_core_regs);
out:
	return rc;
}

static int hdmi_init_irq(void)
{
	int rc = msm_gpios_enable(dtv_panel_irq_gpios,
			ARRAY_SIZE(dtv_panel_irq_gpios));
	if (rc < 0) {
		pr_err("%s: gpio enable failed: %d\n", __func__, rc);
		return rc;
	}
	pr_info("%s\n", __func__);

	return 0;
}

static int hdmi_enable_5v(int on)
{
	int pmic_gpio_hdmi_5v_en ;

	if (machine_is_msm8x55_svlte_surf() || machine_is_msm8x55_svlte_ffa() ||
						machine_is_msm7x30_fluid())
		pmic_gpio_hdmi_5v_en = PMIC_GPIO_HDMI_5V_EN_V2 ;
	else
		pmic_gpio_hdmi_5v_en = PMIC_GPIO_HDMI_5V_EN_V3 ;

	pr_info("%s: %d\n", __func__, on);
	if (on) {
		int rc;
		rc = gpio_request(PM8058_GPIO_PM_TO_SYS(pmic_gpio_hdmi_5v_en),
			"hdmi_5V_en");
		if (rc) {
			pr_err("%s PMIC_GPIO_HDMI_5V_EN gpio_request failed\n",
				__func__);
			return rc;
		}
		gpio_set_value_cansleep(
			PM8058_GPIO_PM_TO_SYS(pmic_gpio_hdmi_5v_en), 1);
	} else {
		gpio_set_value_cansleep(
			PM8058_GPIO_PM_TO_SYS(pmic_gpio_hdmi_5v_en), 0);
		gpio_free(PM8058_GPIO_PM_TO_SYS(pmic_gpio_hdmi_5v_en));
	}
	return 0;
}

static int hdmi_comm_power(int on, int show)
{
	if (show)
		pr_info("%s: i2c comm: %d <LDO8+LDO10>\n", __func__, on);
	return on ?
		regulator_bulk_enable(ARRAY_SIZE(hdmi_comm_regs),
				hdmi_comm_regs) :
		regulator_bulk_disable(ARRAY_SIZE(hdmi_comm_regs),
				hdmi_comm_regs);
}

static int hdmi_core_power(int on, int show)
{
	if (show)
		pr_info("%s: %d <LDO8>\n", __func__, on);
	return on ?
		regulator_bulk_enable(ARRAY_SIZE(hdmi_core_regs),
				hdmi_core_regs) :
		regulator_bulk_disable(ARRAY_SIZE(hdmi_core_regs),
				hdmi_core_regs);
}

static int hdmi_cec_power(int on)
{
	pr_info("%s: %d <LDO17>\n", __func__, on);
	return on ? regulator_bulk_enable(ARRAY_SIZE(hdmi_cec_regs),
				hdmi_cec_regs) :
		regulator_bulk_disable(ARRAY_SIZE(hdmi_cec_regs),
				hdmi_cec_regs);
}

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
	struct i2c_adapter *adapter;

	if (machine_is_msm7x30_fluid()) {
		adapter = i2c_get_adapter(0);
		if (!adapter) {
			pr_err("%s: invalid i2c adapter\n", __func__);
			return PTR_ERR(adapter);
		}

		/* turn on LDO8 */
		rc = hdmi_core_power(1, 0);
		if (rc) {
			pr_err("%s: could not enable hdmi core regs: %d",
					__func__, rc);
			goto adapter_put;
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
	hdmi_core_power(0, 0);
adapter_put:
	i2c_put_adapter(adapter);
	return rc;
}
fs_initcall_sync(fluid_i2c_address_fixup);
#endif

static bool hdmi_check_hdcp_hw_support(void)
{
	if (machine_is_msm7x30_fluid())
		return false;
	else
		return true;
}

static int dtv_panel_power(int on)
{
	int flag_on = !!on;
	static int dtv_power_save_on;
	int rc;

	if (dtv_power_save_on == flag_on)
		return 0;

	dtv_power_save_on = flag_on;
	pr_info("%s: %d\n", __func__, on);

#ifdef HDMI_RESET
	if (on) {
		/* reset Toshiba WeGA chip -- toggle reset pin -- gpio_180 */
		rc = gpio_tlmm_config(dtv_reset_gpio, GPIO_CFG_ENABLE);
		if (rc) {
			pr_err("%s: gpio_tlmm_config(%#x)=%d\n",
				       __func__, dtv_reset_gpio, rc);
			return rc;
		}

		/* bring reset line low to hold reset*/
		gpio_set_value(37, 0);
	}
#endif

	if (on) {
		rc = msm_gpios_enable(dtv_panel_gpios,
				ARRAY_SIZE(dtv_panel_gpios));
		if (rc < 0) {
			printk(KERN_ERR "%s: gpio enable failed: %d\n",
				__func__, rc);
			return rc;
		}
	} else {
		rc = msm_gpios_disable(dtv_panel_gpios,
				ARRAY_SIZE(dtv_panel_gpios));
		if (rc < 0) {
			printk(KERN_ERR "%s: gpio disable failed: %d\n",
				__func__, rc);
			return rc;
		}
	}

	mdelay(5);		/* ensure power is stable */

#ifdef HDMI_RESET
	if (on) {
		gpio_set_value(37, 1);	/* bring reset line high */
		mdelay(10);		/* 10 msec before IO can be accessed */
	}
#endif

	return rc;
}

static struct lcdc_platform_data dtv_pdata = {
	.lcdc_power_save   = dtv_panel_power,
};
#endif
/* FUJITSU:2011-12-01 del DTV end */

/* FUJITSU:2011-12-01 del uart_dm1 start */
#if 0
static struct msm_serial_hs_platform_data msm_uart_dm1_pdata = {
       .inject_rx_on_wakeup = 1,
       .rx_to_inject = 0xFD,
};
#endif
/* FUJITSU:2011-12-01 del uart_dm1 end */

// Fujitsu 2012.01.06 WLAN/BT add start
/* FUJITSU:2011-05-06 add BT HS start */
#ifdef MSM_UARTDM2_USE
static struct msm_serial_hs_platform_data msm_uart_dm2_pdata = {
       .inject_rx_on_wakeup = 1,
       .rx_to_inject = 0xFD,
};
#endif
/* FUJITSU:2011-05-06 add BT HS end */
// Fujitsu 2012.01.06 WLAN/BT add end


static struct resource msm_fb_resources[] = {
	{
		.flags  = IORESOURCE_DMA,
	}
};

static int msm_fb_detect_panel(const char *name)
{
	if (machine_is_msm7x30_fluid()) {
		if (!strcmp(name, "lcdc_sharp_wvga_pt"))
			return 0;
	} else {
		if (!strncmp(name, "mddi_toshiba_wvga_pt", 20))
			return -EPERM;
		else if (!strncmp(name, "lcdc_toshiba_wvga_pt", 20))
			return 0;
		else if (!strcmp(name, "mddi_orise"))
			return -EPERM;
		else if (!strcmp(name, "mddi_quickvx"))
			return -EPERM;
	}
	return -ENODEV;
}

static struct msm_fb_platform_data msm_fb_pdata = {
	.detect_client = msm_fb_detect_panel,
	.mddi_prescan = 1,
};

static struct platform_device msm_fb_device = {
	.name   = "msm_fb",
	.id     = 0,
	.num_resources  = ARRAY_SIZE(msm_fb_resources),
	.resource       = msm_fb_resources,
	.dev    = {
		.platform_data = &msm_fb_pdata,
	}
};

static struct platform_device msm_migrate_pages_device = {
	.name   = "msm_migrate_pages",
	.id     = -1,
};

/* FUJITSU:2012-04-20 LED start */
static struct platform_device mobile_led_device = {
        .name   = "mobile-led",
        .id     = -1,
};
/* FUJITSU:2012-04-20 LED end */

static struct android_pmem_platform_data android_pmem_adsp_pdata = {
       .name = "pmem_adsp",
       .allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
       .cached = 0,
	.memory_type = MEMTYPE_EBI0,
};

static struct android_pmem_platform_data android_pmem_audio_pdata = {
       .name = "pmem_audio",
       .allocator_type = PMEM_ALLOCATORTYPE_BITMAP,
       .cached = 0,
	.memory_type = MEMTYPE_EBI0,
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
	/* Bus Scaling declaration*/
	.bus_scale_table = NULL,
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
	/* Bus Scaling declaration*/
	.bus_scale_table = NULL,
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

static int mddi_toshiba_pmic_bl(int level)
{
	int ret = -EPERM;

	ret = pmic_set_led_intensity(LED_LCD, level);

	if (ret)
		printk(KERN_WARNING "%s: can't set lcd backlight!\n",
					__func__);
	return ret;
}

static struct msm_panel_common_pdata mddi_toshiba_pdata = {
	.pmic_backlight = mddi_toshiba_pmic_bl,
};

static struct platform_device mddi_toshiba_device = {
	.name   = "mddi_toshiba",
	.id     = 0,
	.dev    = {
		.platform_data = &mddi_toshiba_pdata,
	}
};

/* FUJITSU:2011-12-22 add mddi start */
#define MLCD_KEY   (105)
#define MLCD_IM    (107)

typedef enum {
    LCD_IC_UNKNOWN,
    LCD_IC_OTM8009A,
    LCD_IC_R61408
} lcd_ic_type_e;

static bool display_on_status = true;	//	paneloff
static struct i2c_adapter *i2c_lcd = NULL;
static lcd_ic_type_e lcd_ic_type = LCD_IC_UNKNOWN;

static int bkl_i2c_write(unsigned char addr,unsigned char data)
{
    struct i2c_msg msg;
    u_int8_t buf[8];
    int ret = 0;
    int try = 0;
    
    msg.addr  = LED_I2C_SLAVE_ADDR;
    msg.buf   = buf;
    msg.len   = 2;
    msg.flags = 0;
    
    buf[0] = addr;
    buf[1] = data;
    
    if (i2c_lcd == NULL) {
        i2c_lcd = i2c_get_adapter(0);
    }
    
    printk("adapt(%p) addr[0x%x] data[0x%x]\n",i2c_lcd,addr,data);
    
    ret = i2c_transfer(i2c_lcd, &msg, 1);
    if (ret < 0) {
        printk(KERN_ERR "[LCD_atBRD]%s I2C(addr:%x,data:%x) ERROR ret = %d\n",__func__,addr,data,ret);
        for (try = 0; try < 50; try++) {
            msleep(5);
            ret = i2c_transfer(i2c_lcd, &msg, 1);
            if (ret >= 0 ) {
                printk(KERN_ERR "[LCD_atBRD]%s I2C(addr:%x,data:%x) ret = %d retry(%d):OK\n",__func__,addr,data,ret,try);
                break;
            }
        }
        if(ret < 0){
            printk(KERN_ERR "[LCD_atBRD]%s I2C(addr:%x,data:%x) ERROR ret = %d retry(%d):NG\n",__func__,addr,data,ret,try);
        }
    }
    else {
        /* I2C transfer successful. return success(0) */
        ret = 0;
    }
    return ret;
}

static bool mddi_panel_is_OTM8009A(void)
{
    int gpio_val = 0;
    bool ret = false;
    
    if (lcd_ic_type == LCD_IC_UNKNOWN){
        //pull up
        gpio_tlmm_config(GPIO_CFG(MLCD_KEY, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
        
        //read
        gpio_val = gpio_get_value(MLCD_KEY);
        
        if (gpio_val == 1){
            //High:OTM8009A
            printk("[LCD]%s OTM8009A connected\n",__func__);
            
            lcd_ic_type = LCD_IC_OTM8009A;
            gpio_tlmm_config(GPIO_CFG(MLCD_KEY, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
            ret = true;
        }
        else {
            //Low:R61408
            printk("[LCD]%s R61408 connected\n",__func__);
            
            lcd_ic_type = LCD_IC_R61408;
            gpio_tlmm_config(GPIO_CFG(MLCD_KEY, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
            ret = false;
        }
    }
    else if (lcd_ic_type == LCD_IC_OTM8009A){
        ret = true;
    }
    else if (lcd_ic_type == LCD_IC_R61408){
        ret = false;
    }
    else {
        printk("[LCD]%s IC_TYPE(%d) return false\n",__func__,lcd_ic_type);
        ret = false;
    }
    
    return ret;
}

static int display_common_power(int on)
{
    int ret = 0;
    
    pr_err("[LCD_atBRD]%s: Enter(%d)\n", __func__,on);
    
    gpio_tlmm_config(GPIO_CFG(MLCD_IM, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
    
    if (on) {
        //lcd driver
        //LDO1 and LDO2 Vout control
        ret = bkl_i2c_write(0x14, 0xA4);

        if(display_on_status != true)
        {
            //initialize to Low
            gpio_set_value(MLCD_IM, 0);
            
            //LDO1 Power control
            ret = bkl_i2c_write(0x13, 0x0D);

            mdelay(2);	/* FUJITSU:2012-04-23 DISP changed 2ms from 10ms */
            
            if (mddi_panel_is_OTM8009A()) {
                gpio_set_value(MLCD_IM, 1);
            } else {
                gpio_set_value(MLCD_IM, 0);
            }
            
            mdelay(1);	/* FUJITSU:2012-04-23 DISP add wait 1ms */
            
            //LDO1 and LDO2 Power control
            ret = bkl_i2c_write(0x13, 0x0F);
            
        }
        display_on_status = true;
        
    }
    else
    {
        if(display_on_status == true)
        {
            //LDO2 PowerOff
            ret = bkl_i2c_write(0x13, 0x0D);
            
            mdelay(40);	/* FUJITSU:2012-05-23 DISP changed 40ms from 10ms */
            gpio_set_value(MLCD_IM, 0);
            
            //LDO1 LDO2 PowerOff
            ret = bkl_i2c_write(0x13, 0x0C);
        }
        display_on_status = false;
    }

    pr_err("[LCD_atBRD]%s: Leave(%d)\n", __func__,on);

    return ret;
}
/* FUJITSU:2011-12-22 add mddi end */

/* FUJITSU:2011-12-01 del mddi start */
#if 0
static unsigned wega_reset_gpio =
	GPIO_CFG(180, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA);

static struct msm_gpio fluid_vee_reset_gpio[] = {
	{ GPIO_CFG(20, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "vee_reset" },
};

static unsigned char quickvx_mddi_client = 1, other_mddi_client = 1;
static unsigned char quickvx_ldo_enabled;

static unsigned quickvx_vlp_gpio =
	GPIO_CFG(97, 0, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL,	GPIO_CFG_2MA);

static struct pm8xxx_gpio_init_info pmic_quickvx_clk_gpio = {
	PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_QUICKVX_CLK),
	{
		.direction      = PM_GPIO_DIR_OUT,
		.output_buffer  = PM_GPIO_OUT_BUF_CMOS,
		.output_value   = 1,
		.pull           = PM_GPIO_PULL_NO,
		.vin_sel        = PM8058_GPIO_VIN_S3,
		.out_strength   = PM_GPIO_STRENGTH_HIGH,
		.function       = PM_GPIO_FUNC_2,
	},
};

static struct regulator *mddi_ldo20;
static struct regulator *mddi_ldo12;
static struct regulator *mddi_ldo16;
static struct regulator *mddi_ldo6;
static struct regulator *mddi_lcd;

static int display_common_init(void)
{
	struct regulator_bulk_data regs[5] = {
		{ .supply = "ldo20", /* voltage set in display_common_power */},
		{ .supply = "ldo12", .min_uV = 1800000, .max_uV = 1800000 },
		{ .supply = "ldo6",  .min_uV = 3075000, .max_uV = 3400000 },
		{ .supply = "ldo16", .min_uV = 2600000, .max_uV = 2600000 },
		{ .supply = NULL,    /* mddi_lcd, initialized below */ },
	};

	int rc = 0;

	if (machine_is_msm7x30_fluid()) {
		/* lcd: LDO8 @1.8V */
		regs[4].supply = "ldo8";
		regs[4].min_uV = 1800000;
		regs[4].max_uV = 1800000;
	} else {
		/* lcd: LDO15 @3.1V */
		regs[4].supply = "ldo15";
		regs[4].min_uV = 3100000;
		regs[4].max_uV = 3100000;
	}

	rc = regulator_bulk_get(NULL, ARRAY_SIZE(regs), regs);
	if (rc) {
		pr_err("%s: regulator_bulk_get failed: %d\n",
				__func__, rc);
		goto bail;
	}

	rc = regulator_bulk_set_voltage(ARRAY_SIZE(regs), regs);
	if (rc) {
		pr_err("%s: regulator_bulk_set_voltage failed: %d\n",
				__func__, rc);
		goto put_regs;
	}

	mddi_ldo20 = regs[0].consumer;
	mddi_ldo12 = regs[1].consumer;
	mddi_ldo6  = regs[2].consumer;
	mddi_ldo16 = regs[3].consumer;
	mddi_lcd   = regs[4].consumer;

	return rc;

put_regs:
	regulator_bulk_free(ARRAY_SIZE(regs), regs);
bail:
	return rc;
}

static int display_common_power(int on)
{
	int rc = 0, flag_on = !!on;
	static int display_common_power_save_on;
	static bool display_regs_initialized;

	if (display_common_power_save_on == flag_on)
		return 0;

	display_common_power_save_on = flag_on;

	if (unlikely(!display_regs_initialized)) {
		rc = display_common_init();
		if (rc) {
			pr_err("%s: regulator init failed: %d\n",
					__func__, rc);
			return rc;
		}
		display_regs_initialized = true;
	}


	if (on) {
		/* reset Toshiba WeGA chip -- toggle reset pin -- gpio_180 */
		rc = gpio_tlmm_config(wega_reset_gpio, GPIO_CFG_ENABLE);
		if (rc) {
			pr_err("%s: gpio_tlmm_config(%#x)=%d\n",
				       __func__, wega_reset_gpio, rc);
			return rc;
		}

		/* bring reset line low to hold reset*/
		gpio_set_value(180, 0);

		if (quickvx_mddi_client) {
			/* QuickVX chip -- VLP pin -- gpio 97 */
			rc = gpio_tlmm_config(quickvx_vlp_gpio,
				GPIO_CFG_ENABLE);
			if (rc) {
				pr_err("%s: gpio_tlmm_config(%#x)=%d\n",
					__func__, quickvx_vlp_gpio, rc);
				return rc;
			}

			/* bring QuickVX VLP line low */
			gpio_set_value(97, 0);

			rc = pm8xxx_gpio_config(pmic_quickvx_clk_gpio.gpio,
						&pmic_quickvx_clk_gpio.config);
			if (rc) {
				pr_err("%s: pm8xxx_gpio_config(%#x)=%d\n",
					__func__, pmic_quickvx_clk_gpio.gpio,
					rc);
				return rc;
			}

			gpio_set_value_cansleep(PM8058_GPIO_PM_TO_SYS(
				PMIC_GPIO_QUICKVX_CLK), 0);
		}
	}

	if (quickvx_mddi_client)
		rc = regulator_set_voltage(mddi_ldo20, 1800000, 1800000);
	else
		rc = regulator_set_voltage(mddi_ldo20, 1500000, 1500000);

	if (rc) {
		pr_err("%s: could not set voltage for ldo20: %d\n",
				__func__, rc);
		return rc;
	}

	if (on) {
		rc = regulator_enable(mddi_ldo20);
		if (rc) {
			pr_err("%s: LDO20 regulator enable failed (%d)\n",
			       __func__, rc);
			return rc;
		}

		rc = regulator_enable(mddi_ldo12);
		if (rc) {
			pr_err("%s: LDO12 regulator enable failed (%d)\n",
			       __func__, rc);
			return rc;
		}

		if (other_mddi_client) {
			rc = regulator_enable(mddi_ldo16);
			if (rc) {
				pr_err("%s: LDO16 regulator enable failed (%d)\n",
					   __func__, rc);
				return rc;
			}
		}

		if (quickvx_ldo_enabled) {
			/* Disable LDO6 during display ON */
			rc = regulator_disable(mddi_ldo6);
			if (rc) {
				pr_err("%s: LDO6 regulator disable failed (%d)\n",
					   __func__, rc);
				return rc;
			}
			quickvx_ldo_enabled = 0;
		}

		rc = regulator_enable(mddi_lcd);
		if (rc) {
			pr_err("%s: LCD regulator enable failed (%d)\n",
				__func__, rc);
			return rc;
		}

		mdelay(5);		/* ensure power is stable */

		if (machine_is_msm7x30_fluid()) {
			rc = msm_gpios_request_enable(fluid_vee_reset_gpio,
					ARRAY_SIZE(fluid_vee_reset_gpio));
			if (rc)
				pr_err("%s gpio_request_enable failed rc=%d\n",
							__func__, rc);
			else {
				/* assert vee reset_n */
				gpio_set_value(20, 1);
				gpio_set_value(20, 0);
				mdelay(1);
				gpio_set_value(20, 1);
			}
		}

		gpio_set_value(180, 1); /* bring reset line high */
		mdelay(10);	/* 10 msec before IO can be accessed */

		if (quickvx_mddi_client) {
			gpio_set_value(97, 1);
			msleep(2);
			gpio_set_value_cansleep(PM8058_GPIO_PM_TO_SYS(
				PMIC_GPIO_QUICKVX_CLK), 1);
			msleep(2);
		}

		rc = pmapp_display_clock_config(1);
		if (rc) {
			pr_err("%s pmapp_display_clock_config rc=%d\n",
					__func__, rc);
			return rc;
		}

	} else {
		rc = regulator_disable(mddi_ldo20);
		if (rc) {
			pr_err("%s: LDO20 regulator disable failed (%d)\n",
			       __func__, rc);
			return rc;
		}


		if (other_mddi_client) {
			rc = regulator_disable(mddi_ldo16);
			if (rc) {
				pr_err("%s: LDO16 regulator disable failed (%d)\n",
					   __func__, rc);
				return rc;
			}
		}

		if (quickvx_mddi_client && !quickvx_ldo_enabled) {
			/* Enable LDO6 during display OFF for
			   Quicklogic chip to sleep with data retention */
			rc = regulator_enable(mddi_ldo6);
			if (rc) {
				pr_err("%s: LDO6 regulator enable failed (%d)\n",
					   __func__, rc);
				return rc;
			}
			quickvx_ldo_enabled = 1;
		}

		gpio_set_value(180, 0); /* bring reset line low */

		if (quickvx_mddi_client) {
			gpio_set_value(97, 0);
			gpio_set_value_cansleep(PM8058_GPIO_PM_TO_SYS(
				PMIC_GPIO_QUICKVX_CLK), 0);
		}

		rc = regulator_disable(mddi_lcd);
		if (rc) {
			pr_err("%s: LCD regulator disable failed (%d)\n",
				__func__, rc);
			return rc;
		}

		mdelay(5);	/* ensure power is stable */

		rc = regulator_disable(mddi_ldo12);
		if (rc) {
			pr_err("%s: LDO12 regulator disable failed (%d)\n",
			       __func__, rc);
			return rc;
		}

		if (machine_is_msm7x30_fluid()) {
			msm_gpios_disable_free(fluid_vee_reset_gpio,
					ARRAY_SIZE(fluid_vee_reset_gpio));
		}

		rc = pmapp_display_clock_config(0);
		if (rc) {
			pr_err("%s pmapp_display_clock_config rc=%d\n",
					__func__, rc);
			return rc;
		}
	}

	return rc;
}
#endif
/* FUJITSU:2011-12-01 del mddi end */

static int msm_fb_mddi_sel_clk(u32 *clk_rate)
{
	*clk_rate *= 2;
	return 0;
}

/* FUJITSU:2011-12-22 del display power start */
#if 0
static int msm_fb_mddi_client_power(u32 client_id)
{
	int rc;
	printk(KERN_NOTICE "\n client_id = 0x%x", client_id);
	/* Check if it is Quicklogic client */
	if (client_id == 0xc5835800) {
		printk(KERN_NOTICE "\n Quicklogic MDDI client");
		other_mddi_client = 0;
		if (IS_ERR(mddi_ldo16)) {
			rc = PTR_ERR(mddi_ldo16);
			pr_err("%s: gp10 vreg get failed (%d)\n", __func__, rc);
			return rc;
		}
		rc = regulator_disable(mddi_ldo16);
		if (rc) {
			pr_err("%s: LDO16 vreg enable failed (%d)\n",
							__func__, rc);
			return rc;
		}

	} else {
		printk(KERN_NOTICE "\n Non-Quicklogic MDDI client");
		quickvx_mddi_client = 0;
		gpio_set_value(97, 0);
		gpio_set_value_cansleep(PM8058_GPIO_PM_TO_SYS(
			PMIC_GPIO_QUICKVX_CLK), 0);
	}

	return 0;
}
#endif
/* FUJITSU:2011-12-22 del display power end */

static struct mddi_platform_data mddi_pdata = {
	.mddi_power_save = display_common_power,
	.mddi_sel_clk = msm_fb_mddi_sel_clk,
/* FUJITSU:2011-12-22 del display power start */
#if 0
	.mddi_client_power = msm_fb_mddi_client_power,
#endif
/* FUJITSU:2011-12-22 del display power end */
};


int mdp_core_clk_rate_table[] = {
/* FUJITSU:2011-12-22 change mdp_core_clk_rate start */
#if 0
	122880000,
	122880000,
#else
	192000000,
	192000000,
#endif
/* FUJITSU:2011-12-22 change mdp_core_clk_rate end */
	192000000,
	192000000,
};

static struct msm_panel_common_pdata mdp_pdata = {
	.hw_revision_addr = 0xac001270,
	.gpio = 30,
/* FUJITSU:2011-12-22 change mdp_core_clk_rate start */
#if 0
	.mdp_core_clk_rate = 122880000,
#else
	.mdp_core_clk_rate = 192000000,
#endif
/* FUJITSU:2011-12-22 change mdp_core_clk_rate end */
	.mdp_core_clk_table = mdp_core_clk_rate_table,
	.num_mdp_clk = ARRAY_SIZE(mdp_core_clk_rate_table),
	.mdp_rev = MDP_REV_40,
};

/* FUJITSU:2011-12-22 del LCDC start */
#if 0
static int lcd_panel_spi_gpio_num[] = {
			45, /* spi_clk */
			46, /* spi_cs  */
			47, /* spi_mosi */
			48, /* spi_miso */
		};

static struct msm_gpio lcd_panel_gpios[] = {
/* Workaround, since HDMI_INT is using the same GPIO line (18), and is used as
 * input.  if there is a hardware revision; we should reassign this GPIO to a
 * new open line; and removing it will just ensure that this will be missed in
 * the future.
	{ GPIO_CFG(18, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_grn0" },
 */
	{ GPIO_CFG(19, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_grn1" },
	{ GPIO_CFG(20, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_blu0" },
	{ GPIO_CFG(21, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_blu1" },
	{ GPIO_CFG(22, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_blu2" },
	{ GPIO_CFG(23, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_red0" },
	{ GPIO_CFG(24, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_red1" },
	{ GPIO_CFG(25, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_red2" },
#ifndef CONFIG_SPI_QSD
	{ GPIO_CFG(45, 0, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "spi_clk" },
	{ GPIO_CFG(46, 0, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "spi_cs0" },
	{ GPIO_CFG(47, 0, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "spi_mosi" },
	{ GPIO_CFG(48, 0, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "spi_miso" },
#endif
	{ GPIO_CFG(90, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_pclk" },
	{ GPIO_CFG(91, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_en" },
	{ GPIO_CFG(92, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_vsync" },
	{ GPIO_CFG(93, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_hsync" },
	{ GPIO_CFG(94, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_grn2" },
	{ GPIO_CFG(95, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_grn3" },
	{ GPIO_CFG(96, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_grn4" },
	{ GPIO_CFG(97, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_grn5" },
	{ GPIO_CFG(98, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_grn6" },
	{ GPIO_CFG(99, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_grn7" },
	{ GPIO_CFG(100, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_blu3" },
	{ GPIO_CFG(101, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_blu4" },
	{ GPIO_CFG(102, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_blu5" },
	{ GPIO_CFG(103, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_blu6" },
	{ GPIO_CFG(104, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_blu7" },
	{ GPIO_CFG(105, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_red3" },
	{ GPIO_CFG(106, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_red4" },
	{ GPIO_CFG(107, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_red5" },
	{ GPIO_CFG(108, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_red6" },
	{ GPIO_CFG(109, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_red7" },
};

static struct msm_gpio lcd_sharp_panel_gpios[] = {
	{ GPIO_CFG(22, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_blu2" },
	{ GPIO_CFG(25, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_red2" },
	{ GPIO_CFG(90, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_pclk" },
	{ GPIO_CFG(91, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_en" },
	{ GPIO_CFG(92, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_vsync" },
	{ GPIO_CFG(93, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_hsync" },
	{ GPIO_CFG(94, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_grn2" },
	{ GPIO_CFG(95, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_grn3" },
	{ GPIO_CFG(96, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_grn4" },
	{ GPIO_CFG(97, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_grn5" },
	{ GPIO_CFG(98, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_grn6" },
	{ GPIO_CFG(99, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_grn7" },
	{ GPIO_CFG(100, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_blu3" },
	{ GPIO_CFG(101, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_blu4" },
	{ GPIO_CFG(102, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_blu5" },
	{ GPIO_CFG(103, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_blu6" },
	{ GPIO_CFG(104, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_blu7" },
	{ GPIO_CFG(105, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_red3" },
	{ GPIO_CFG(106, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_red4" },
	{ GPIO_CFG(107, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_red5" },
	{ GPIO_CFG(108, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_red6" },
	{ GPIO_CFG(109, 1, GPIO_CFG_OUTPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "lcdc_red7" },
};

static int lcdc_toshiba_panel_power(int on)
{
	int rc, i;
	struct msm_gpio *gp;

	rc = display_common_power(on);
	if (rc < 0) {
		printk(KERN_ERR "%s display_common_power failed: %d\n",
				__func__, rc);
		return rc;
	}

	if (on) {
		rc = msm_gpios_enable(lcd_panel_gpios,
				ARRAY_SIZE(lcd_panel_gpios));
		if (rc < 0) {
			printk(KERN_ERR "%s: gpio enable failed: %d\n",
					__func__, rc);
		}
	} else {	/* off */
		gp = lcd_panel_gpios;
		for (i = 0; i < ARRAY_SIZE(lcd_panel_gpios); i++) {
			/* ouput low */
			gpio_set_value(GPIO_PIN(gp->gpio_cfg), 0);
			gp++;
		}
	}

	return rc;
}

static int lcdc_sharp_panel_power(int on)
{
	int rc, i;
	struct msm_gpio *gp;

	rc = display_common_power(on);
	if (rc < 0) {
		printk(KERN_ERR "%s display_common_power failed: %d\n",
				__func__, rc);
		return rc;
	}

	if (on) {
		rc = msm_gpios_enable(lcd_sharp_panel_gpios,
				ARRAY_SIZE(lcd_sharp_panel_gpios));
		if (rc < 0) {
			printk(KERN_ERR "%s: gpio enable failed: %d\n",
				__func__, rc);
		}
	} else {	/* off */
		gp = lcd_sharp_panel_gpios;
		for (i = 0; i < ARRAY_SIZE(lcd_sharp_panel_gpios); i++) {
			/* ouput low */
			gpio_set_value(GPIO_PIN(gp->gpio_cfg), 0);
			gp++;
		}
	}

	return rc;
}

static int lcdc_panel_power(int on)
{
	int flag_on = !!on;
	static int lcdc_power_save_on, lcdc_power_initialized;

	if (lcdc_power_save_on == flag_on)
		return 0;

	lcdc_power_save_on = flag_on;

	if (unlikely(!lcdc_power_initialized)) {
		quickvx_mddi_client = 0;
		display_common_init();
		lcdc_power_initialized = 1;
	}

	if (machine_is_msm7x30_fluid())
		return lcdc_sharp_panel_power(on);
	else
		return lcdc_toshiba_panel_power(on);
}

static struct lcdc_platform_data lcdc_pdata = {
	.lcdc_power_save   = lcdc_panel_power,
};
#endif
/* FUJITSU:2011-12-22 del LCDC end */

/* FUJITSU:2011-12-01 del DTV start */
#if 0
static struct regulator *atv_s4, *atv_ldo9;

static int __init atv_dac_power_init(void)
{
	int rc;
	struct regulator_bulk_data regs[] = {
		{ .supply = "smps4", .min_uV = 2200000, .max_uV = 2200000 },
		{ .supply = "ldo9",  .min_uV = 2050000, .max_uV = 2050000 },
	};

	rc = regulator_bulk_get(NULL, ARRAY_SIZE(regs), regs);

	if (rc) {
		pr_err("%s: could not get regulators: %d\n", __func__, rc);
		goto bail;
	}

	rc = regulator_bulk_set_voltage(ARRAY_SIZE(regs), regs);

	if (rc) {
		pr_err("%s: could not set voltages: %d\n", __func__, rc);
		goto reg_free;
	}

	atv_s4   = regs[0].consumer;
	atv_ldo9 = regs[1].consumer;

reg_free:
	regulator_bulk_free(ARRAY_SIZE(regs), regs);
bail:
	return rc;
}

static int atv_dac_power(int on)
{
	int rc = 0;

	if (on) {
		rc = regulator_enable(atv_s4);
		if (rc) {
			pr_err("%s: s4 vreg enable failed (%d)\n",
				__func__, rc);
			return rc;
		}
		rc = regulator_enable(atv_ldo9);
		if (rc) {
			pr_err("%s: ldo9 vreg enable failed (%d)\n",
				__func__, rc);
			return rc;
		}
	} else {
		rc = regulator_disable(atv_ldo9);
		if (rc) {
			pr_err("%s: ldo9 vreg disable failed (%d)\n",
				   __func__, rc);
			return rc;
		}
		rc = regulator_disable(atv_s4);
		if (rc) {
			pr_err("%s: s4 vreg disable failed (%d)\n",
				   __func__, rc);
			return rc;
		}
	}
	return rc;
}

static struct tvenc_platform_data atv_pdata = {
	.poll		 = 1,
	.pm_vid_en	 = atv_dac_power,
};
#endif
/* FUJITSU:2011-12-01 del DTV end */

static void __init msm_fb_add_devices(void)
{
	msm_fb_register_device("mdp", &mdp_pdata);
	msm_fb_register_device("pmdh", &mddi_pdata);
/* FUJITSU:2011-12-22 del lcdc start */
#if 0
	msm_fb_register_device("lcdc", &lcdc_pdata);
#endif
/* FUJITSU:2011-12-22 del lcdc end */
/* FUJITSU:2011-12-01 del DTV start */
#if 0
	msm_fb_register_device("dtv", &dtv_pdata);
	msm_fb_register_device("tvenc", &atv_pdata);
#endif
/* FUJITSU:2011-12-01 del DTV end */
#ifdef CONFIG_FB_MSM_TVOUT
	msm_fb_register_device("tvout_device", NULL);
#endif
}

/* FUJITSU:2011-12-01 del spi devices start */
#if 0
static struct msm_panel_common_pdata lcdc_toshiba_panel_data = {
	.gpio_num          = lcd_panel_spi_gpio_num,
};

static struct platform_device lcdc_toshiba_panel_device = {
	.name   = "lcdc_toshiba_wvga",
	.id     = 0,
	.dev    = {
		.platform_data = &lcdc_toshiba_panel_data,
	}
};
#endif
/* FUJITSU:2011-12-01 del spi devices end */

#if defined(CONFIG_MARIMBA_CORE) && \
   (defined(CONFIG_MSM_BT_POWER) || defined(CONFIG_MSM_BT_POWER_MODULE))
static struct platform_device msm_bt_power_device = {
	.name = "bt_power",
	.id     = -1
};

enum {
	BT_RFR,
	BT_CTS,
	BT_RX,
	BT_TX,
};
// Fujitsu 2012.01.06 WLAN/BT add start
// PMIC GPIO PORT
#define KO_PMICUART1_BT_TXD_21   (21-1)
#define KO_PMICUART1_BT_RXD_33   (33-1)
#define KO_PMICUART1_BT_PMIC_TXD_36 (36-1)
#define KO_PMICUART1_BT_PMIC_RXD_37 (37-1)

static unsigned bt_config_power_on[] = {
	GPIO_CFG(138, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,   GPIO_CFG_2MA),/* KO_BT_PCMOUT         */
	GPIO_CFG(139, 1, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),/* KO_BT_PCMIN          */
	GPIO_CFG(140, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,   GPIO_CFG_2MA),/* KO_BT_PCM_FSYNC      */
	GPIO_CFG(141, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,   GPIO_CFG_2MA),/* KO_BT_PCM_DCLK       */
	GPIO_CFG(147, 0, GPIO_CFG_INPUT,  GPIO_CFG_PULL_UP,   GPIO_CFG_2MA),/* KO_BT_HOSTWAKEUP     */
#if 0  // Fujitsu 2011.07.29 WLAN/BT change start
	GPIO_CFG(150,  0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,   GPIO_CFG_2MA),/* KO_BT_WAKEUP         */
#else  // Fujitsu 2011.07.29 WLAN/BT change
	GPIO_CFG(21,  0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,   GPIO_CFG_2MA),/* KO_BT_WAKEUP         */
#endif // Fujitsu 2011.07.29 WLAN/BT change end
	GPIO_CFG(41,  0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,   GPIO_CFG_2MA),/* KO_BT_RESET          */
	GPIO_CFG(43,  0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,   GPIO_CFG_2MA),/* KO_BT_PWON           */
#if 0  // Fujitsu 2011.07.29 WLAN/BT change start
	GPIO_CFG(149,  0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,   GPIO_CFG_2MA),/* KO_WLAN_BT_PWR_ONOFF */
#else  // Fujitsu 2011.07.29 WLAN/BT change
	GPIO_CFG(96,  0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,   GPIO_CFG_2MA),/* KO_WLAN_BT_PWR_ONOFF */
#endif // Fujitsu 2011.07.29 WLAN/BT change end
#if 0  // Fujitsu 2011.08.04 WLAN/BT del start
	GPIO_CFG(38,  0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,   GPIO_CFG_2MA),/* KO_WLAN_RESET        */
#endif // Fujitsu 2011.08.04 WLAN/BT del end
	GPIO_CFG(39,  0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,   GPIO_CFG_2MA),/* KO_WLAN_PWON         */
/* FUJITSU:2011-07-08 change WLAN start */
#if 0  // Fujitsu 2011.07.29 WLAN/BT change start
	GPIO_CFG(145,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),	/* KO_WLAN_HOSTWAKEUP   */
#else  // Fujitsu 2011.07.29 WLAN/BT change
	GPIO_CFG(20,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),	/* KO_WLAN_HOSTWAKEUP   */
#endif // Fujitsu 2011.07.29 WLAN/BT change end
/* FUJITSU:2011-07-08 change WLAN end */
/* FUJITSU:2011-05-06 add BT HS start */
#ifdef MSM_UARTDM2_USE
	GPIO_CFG(49, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,   GPIO_CFG_2MA),
#if 0  // Fujitsu 2011.11.09 WLAN/BT change start
	GPIO_CFG(50, 1, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL,   GPIO_CFG_2MA),
#else  // Fujitsu 2011.11.09 WLAN/BT change
	GPIO_CFG(50, 1, GPIO_CFG_INPUT,  GPIO_CFG_PULL_UP,   GPIO_CFG_2MA),
#endif // Fujitsu 2011.11.09 WLAN/BT change end
	GPIO_CFG(51, 1, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL,   GPIO_CFG_2MA),
	GPIO_CFG(52, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,   GPIO_CFG_2MA),
#endif
/* FUJITSU:2011-05-06 add BT HS end */
	};
// Fujitsu 2012.01.06 WLAN/BT add end
// Fujitsu 2012.01.06 WLAN/BT del start
#if 0
static struct msm_gpio bt_config_power_on[] = {
	{ GPIO_CFG(134, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,   GPIO_CFG_2MA),
		"UART1DM_RFR" },
	{ GPIO_CFG(135, 1, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL,   GPIO_CFG_2MA),
		"UART1DM_CTS" },
	{ GPIO_CFG(136, 1, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL,   GPIO_CFG_2MA),
		"UART1DM_Rx" },
	{ GPIO_CFG(137, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,   GPIO_CFG_2MA),
		"UART1DM_Tx" }
};

static struct msm_gpio bt_config_power_off[] = {
	{ GPIO_CFG(134, 0, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN,   GPIO_CFG_2MA),
		"UART1DM_RFR" },
	{ GPIO_CFG(135, 0, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN,   GPIO_CFG_2MA),
		"UART1DM_CTS" },
	{ GPIO_CFG(136, 0, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN,   GPIO_CFG_2MA),
		"UART1DM_Rx" },
	{ GPIO_CFG(137, 0, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN,   GPIO_CFG_2MA),
		"UART1DM_Tx" }
};
#endif
// Fujitsu 2012.01.06 WLAN/BT del end


// Fujitsu 2012.01.06 WLAN/BT add start
static unsigned bt_config_power_off[] = {
	GPIO_CFG(138, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),/* KO_BT_PCMOUT         */
	GPIO_CFG(139, 1, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),/* KO_BT_PCMIN          */
	GPIO_CFG(140, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),/* KO_BT_PCM_FSYNC      */
	GPIO_CFG(141, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),/* KO_BT_PCM_DCLK       */
	GPIO_CFG(147, 0, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),/* KO_BT_HOSTWAKEUP     */
#if 0  // Fujitsu 2011.07.29 WLAN/BT change start
	GPIO_CFG(150,  0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),/* KO_BT_WAKEUP         */
#else  // Fujitsu 2011.07.29 WLAN/BT change
	GPIO_CFG(21,  0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),/* KO_BT_WAKEUP         */
#endif // Fujitsu 2011.07.29 WLAN/BT change end
	GPIO_CFG(41,  0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),/* KO_BT_RESET          */
	GPIO_CFG(43,  0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),/* KO_BT_PWON           */
#if 0  // Fujitsu 2011.07.29 WLAN/BT change start
	GPIO_CFG(149,  0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),/* KO_WLAN_BT_PWR_ONOFF */
#else  // Fujitsu 2011.07.29 WLAN/BT change
	GPIO_CFG(96,  0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),/* KO_WLAN_BT_PWR_ONOFF */
#endif // Fujitsu 2011.07.29 WLAN/BT change end
#if 0  // Fujitsu 2011.08.04 del start
	GPIO_CFG(38,  0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),/* KO_WLAN_RESET        */
#endif // Fujitsu 2011.08.04 del end
	GPIO_CFG(39,  0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),/* KO_WLAN_PWON         */
/* FUJITSU:2011-07-08 change WLAN start */
#if 0  // Fujitsu 2011.07.29 WLAN/BT change start
	GPIO_CFG(145,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),/* KO_WLAN_HOSTWAKEUP   */
#else  // Fujitsu 2011.07.29 WLAN/BT change
	GPIO_CFG(20,  0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA),/* KO_WLAN_HOSTWAKEUP   */
#endif // Fujitsu 2011.07.29 WLAN/BT change end
/* FUJITSU:2011-07-08 change WLAN end */
	};
// Fujitsu 2012.01.06 WLAN/BT add end

#if 0
static u8 bahama_version;

static struct regulator_bulk_data regs_bt_marimba[] = {
	{ .supply = "smps3", .min_uV = 1800000, .max_uV = 1800000 },
	{ .supply = "smps2", .min_uV = 1300000, .max_uV = 1300000 },
	{ .supply = "ldo24", .min_uV = 1200000, .max_uV = 1200000 },
	{ .supply = "ldo13", .min_uV = 2900000, .max_uV = 3050000 },
};

static struct regulator_bulk_data regs_bt_bahama_v1[] = {
	{ .supply = "smps3", .min_uV = 1800000, .max_uV = 1800000 },
	{ .supply = "ldo7",  .min_uV = 1800000, .max_uV = 1800000 },
	{ .supply = "smps2", .min_uV = 1300000, .max_uV = 1300000 },
	{ .supply = "ldo13", .min_uV = 2900000, .max_uV = 3050000 },
};

static struct regulator_bulk_data regs_bt_bahama_v2[] = {
	{ .supply = "smps3", .min_uV = 1800000, .max_uV = 1800000 },
	{ .supply = "ldo7",  .min_uV = 1800000, .max_uV = 1800000 },
	{ .supply = "ldo13", .min_uV = 2900000, .max_uV = 3050000 },
};

static struct regulator_bulk_data *regs_bt;
static int regs_bt_count;

static int marimba_bt(int on)
{
	int rc;
	int i;
	struct marimba config = { .mod_id = MARIMBA_SLAVE_ID_MARIMBA };

	struct marimba_config_register {
		u8 reg;
		u8 value;
		u8 mask;
	};

	struct marimba_variant_register {
		const size_t size;
		const struct marimba_config_register *set;
	};

	const struct marimba_config_register *p;

	u8 version;

	const struct marimba_config_register v10_bt_on[] = {
		{ 0xE5, 0x0B, 0x0F },
		{ 0x05, 0x02, 0x07 },
		{ 0x06, 0x88, 0xFF },
		{ 0xE7, 0x21, 0x21 },
		{ 0xE3, 0x38, 0xFF },
		{ 0xE4, 0x06, 0xFF },
	};

	const struct marimba_config_register v10_bt_off[] = {
		{ 0xE5, 0x0B, 0x0F },
		{ 0x05, 0x08, 0x0F },
		{ 0x06, 0x88, 0xFF },
		{ 0xE7, 0x00, 0x21 },
		{ 0xE3, 0x00, 0xFF },
		{ 0xE4, 0x00, 0xFF },
	};

	const struct marimba_config_register v201_bt_on[] = {
		{ 0x05, 0x08, 0x07 },
		{ 0x06, 0x88, 0xFF },
		{ 0xE7, 0x21, 0x21 },
		{ 0xE3, 0x38, 0xFF },
		{ 0xE4, 0x06, 0xFF },
	};

	const struct marimba_config_register v201_bt_off[] = {
		{ 0x05, 0x08, 0x07 },
		{ 0x06, 0x88, 0xFF },
		{ 0xE7, 0x00, 0x21 },
		{ 0xE3, 0x00, 0xFF },
		{ 0xE4, 0x00, 0xFF },
	};

	const struct marimba_config_register v210_bt_on[] = {
		{ 0xE9, 0x01, 0x01 },
		{ 0x06, 0x88, 0xFF },
		{ 0xE7, 0x21, 0x21 },
		{ 0xE3, 0x38, 0xFF },
		{ 0xE4, 0x06, 0xFF },
	};

	const struct marimba_config_register v210_bt_off[] = {
		{ 0x06, 0x88, 0xFF },
		{ 0xE7, 0x00, 0x21 },
		{ 0xE9, 0x00, 0x01 },
		{ 0xE3, 0x00, 0xFF },
		{ 0xE4, 0x00, 0xFF },
	};

	const struct marimba_variant_register bt_marimba[2][4] = {
		{
			{ ARRAY_SIZE(v10_bt_off), v10_bt_off },
			{ 0, NULL },
			{ ARRAY_SIZE(v201_bt_off), v201_bt_off },
			{ ARRAY_SIZE(v210_bt_off), v210_bt_off }
		},
		{
			{ ARRAY_SIZE(v10_bt_on), v10_bt_on },
			{ 0, NULL },
			{ ARRAY_SIZE(v201_bt_on), v201_bt_on },
			{ ARRAY_SIZE(v210_bt_on), v210_bt_on }
		}
	};

	on = on ? 1 : 0;

	rc = marimba_read_bit_mask(&config, 0x11,  &version, 1, 0x1F);
	if (rc < 0) {
		printk(KERN_ERR
			"%s: version read failed: %d\n",
			__func__, rc);
		return rc;
	}

	if ((version >= ARRAY_SIZE(bt_marimba[on])) ||
	    (bt_marimba[on][version].size == 0)) {
		printk(KERN_ERR
			"%s: unsupported version\n",
			__func__);
		return -EIO;
	}

	p = bt_marimba[on][version].set;

	printk(KERN_INFO "%s: found version %d\n", __func__, version);

	for (i = 0; i < bt_marimba[on][version].size; i++) {
		u8 value = (p+i)->value;
		rc = marimba_write_bit_mask(&config,
			(p+i)->reg,
			&value,
			sizeof((p+i)->value),
			(p+i)->mask);
		if (rc < 0) {
			printk(KERN_ERR
				"%s: reg %d write failed: %d\n",
				__func__, (p+i)->reg, rc);
			return rc;
		}
		printk(KERN_INFO "%s: reg 0x%02x value 0x%02x mask 0x%02x\n",
				__func__, (p+i)->reg,
				value, (p+i)->mask);
	}
	return 0;
}

static int bahama_bt(int on)
{
	int rc;
	int i;
	struct marimba config = { .mod_id = SLAVE_ID_BAHAMA };

	struct bahama_variant_register {
		const size_t size;
		const struct bahama_config_register *set;
	};

	const struct bahama_config_register *p;


	const struct bahama_config_register v10_bt_on[] = {
		{ 0xE9, 0x00, 0xFF },
		{ 0xF4, 0x80, 0xFF },
		{ 0xF0, 0x06, 0xFF },
		{ 0xE4, 0x00, 0xFF },
		{ 0xE5, 0x00, 0x0F },
#ifdef CONFIG_WLAN
		{ 0xE6, 0x38, 0x7F },
		{ 0xE7, 0x06, 0xFF },
#endif
		{ 0x11, 0x13, 0xFF },
		{ 0xE9, 0x21, 0xFF },
		{ 0x01, 0x0C, 0x1F },
		{ 0x01, 0x08, 0x1F },
	};

	const struct bahama_config_register v20_bt_on_fm_off[] = {
		{ 0x11, 0x0C, 0xFF },
		{ 0x13, 0x01, 0xFF },
		{ 0xF4, 0x80, 0xFF },
		{ 0xF0, 0x00, 0xFF },
		{ 0xE9, 0x00, 0xFF },
#ifdef CONFIG_WLAN
		{ 0x81, 0x00, 0xFF },
		{ 0x82, 0x00, 0xFF },
		{ 0xE6, 0x38, 0x7F },
		{ 0xE7, 0x06, 0xFF },
#endif
		{ 0xE9, 0x21, 0xFF }
	};

	const struct bahama_config_register v20_bt_on_fm_on[] = {
		{ 0x11, 0x0C, 0xFF },
		{ 0x13, 0x01, 0xFF },
		{ 0xF4, 0x86, 0xFF },
		{ 0xF0, 0x06, 0xFF },
		{ 0xE9, 0x00, 0xFF },
#ifdef CONFIG_WLAN
		{ 0x81, 0x00, 0xFF },
		{ 0x82, 0x00, 0xFF },
		{ 0xE6, 0x38, 0x7F },
		{ 0xE7, 0x06, 0xFF },
#endif
		{ 0xE9, 0x21, 0xFF }
	};

	const struct bahama_config_register v10_bt_off[] = {
		{ 0xE9, 0x00, 0xFF },
	};

	const struct bahama_config_register v20_bt_off_fm_off[] = {
		{ 0xF4, 0x84, 0xFF },
		{ 0xF0, 0x04, 0xFF },
		{ 0xE9, 0x00, 0xFF }
	};

	const struct bahama_config_register v20_bt_off_fm_on[] = {
		{ 0xF4, 0x86, 0xFF },
		{ 0xF0, 0x06, 0xFF },
		{ 0xE9, 0x00, 0xFF }
	};

	const struct bahama_variant_register bt_bahama[2][3] = {
		{
			{ ARRAY_SIZE(v10_bt_off), v10_bt_off },
			{ ARRAY_SIZE(v20_bt_off_fm_off), v20_bt_off_fm_off },
			{ ARRAY_SIZE(v20_bt_off_fm_on), v20_bt_off_fm_on }
		},
		{
			{ ARRAY_SIZE(v10_bt_on), v10_bt_on },
			{ ARRAY_SIZE(v20_bt_on_fm_off), v20_bt_on_fm_off },
			{ ARRAY_SIZE(v20_bt_on_fm_on), v20_bt_on_fm_on }
		}
	};

	u8 offset = 0; /* index into bahama configs */

	on = on ? 1 : 0;


	if (bahama_version == VER_2_0) {
		if (marimba_get_fm_status(&config))
			offset = 0x01;
	}

	p = bt_bahama[on][bahama_version + offset].set;

	dev_info(&msm_bt_power_device.dev,
		"%s: found version %d\n", __func__, bahama_version);

	for (i = 0; i < bt_bahama[on][bahama_version + offset].size; i++) {
		u8 value = (p+i)->value;
		rc = marimba_write_bit_mask(&config,
			(p+i)->reg,
			&value,
			sizeof((p+i)->value),
			(p+i)->mask);
		if (rc < 0) {
			dev_err(&msm_bt_power_device.dev,
				"%s: reg %d write failed: %d\n",
				__func__, (p+i)->reg, rc);
			return rc;
		}
		dev_info(&msm_bt_power_device.dev,
			"%s: reg 0x%02x write value 0x%02x mask 0x%02x\n",
				__func__, (p+i)->reg,
				value, (p+i)->mask);
	}
	/* Update BT status */
	if (on)
		marimba_set_bt_status(&config, true);
	else
		marimba_set_bt_status(&config, false);

	return 0;
}

static int bluetooth_regs_init(int bahama_not_marimba)
{
	int rc = 0;
	struct device *const dev = &msm_bt_power_device.dev;

	if (bahama_not_marimba) {
		bahama_version = read_bahama_ver();

		switch (bahama_version) {
		case VER_1_0:
			regs_bt = regs_bt_bahama_v1;
			regs_bt_count = ARRAY_SIZE(regs_bt_bahama_v1);
			break;
		case VER_2_0:
			regs_bt = regs_bt_bahama_v2;
			regs_bt_count = ARRAY_SIZE(regs_bt_bahama_v2);
			break;
		case VER_UNSUPPORTED:
		default:
			dev_err(dev,
				"%s: i2c failure or unsupported version: %d\n",
				__func__, bahama_version);
			rc = -EIO;
			goto out;
		}
	} else {
		regs_bt = regs_bt_marimba;
		regs_bt_count = ARRAY_SIZE(regs_bt_marimba);
	}

	rc = regulator_bulk_get(&msm_bt_power_device.dev,
			regs_bt_count, regs_bt);
	if (rc) {
		dev_err(dev, "%s: could not get regulators: %d\n",
				__func__, rc);
		goto out;
	}

	rc = regulator_bulk_set_voltage(regs_bt_count, regs_bt);
	if (rc) {
		dev_err(dev, "%s: could not set voltages: %d\n",
				__func__, rc);
		goto reg_free;
	}

	return 0;

reg_free:
	regulator_bulk_free(regs_bt_count, regs_bt);
out:
	regs_bt_count = 0;
	regs_bt = NULL;
	return rc;
}
#endif
static void set_power_wlan_bt(int, int);
// Fujitsu 2012.01.06 WLAN/BT add end

static int bluetooth_power(int on)
{
	int rc;
// Fujitsu 2012.01.06 WLAN/BT add start
	int pin;
#if 0
	const char *id = "BTPW";
#endif

	// PMIC GPIO 21
	struct pm8xxx_gpio_init_info bt_txd_en_21 = {
		PM8058_GPIO_PM_TO_SYS(KO_PMICUART1_BT_TXD_21),
		{
			.direction      = PM_GPIO_DIR_OUT,
			.pull           = PM_GPIO_PULL_NO,
			.vin_sel        = PM8058_GPIO_VIN_S3,
			.function       = PM_GPIO_FUNC_2,
			.inv_int_pol    = 0,
			.out_strength   = PM_GPIO_STRENGTH_MED,
			.output_value   = 0,
		},
	};

	// PMIC GPIO 33
	struct pm8xxx_gpio_init_info bt_rxd_en_33 = {
		PM8058_GPIO_PM_TO_SYS(KO_PMICUART1_BT_RXD_33),
		{
			.direction      = PM_GPIO_DIR_IN,
			.pull           = PM_GPIO_PULL_NO,
			.vin_sel        = PM8058_GPIO_VIN_S3,
			.function       = PM_GPIO_FUNC_1,
			.inv_int_pol    = 0,
			.out_strength   = PM_GPIO_STRENGTH_MED,
			.output_value   = 0,
		},
	};

	// PMIC GPIO 36
	struct pm8xxx_gpio_init_info bt_pmic_txd_en_36 = {
		PM8058_GPIO_PM_TO_SYS(KO_PMICUART1_BT_PMIC_TXD_36),
		{
			.direction      = PM_GPIO_DIR_IN,
			.pull           = PM_GPIO_PULL_NO,
			.vin_sel        = PM8058_GPIO_VIN_S3,
			.function       = PM_GPIO_FUNC_1,
			.inv_int_pol    = 0,
			.out_strength   = PM_GPIO_STRENGTH_MED,
			.output_value   = 0,
		},
	};

	// PMIC GPIO 37
	struct pm8xxx_gpio_init_info bt_pmic_rxd_en_37 = {
		PM8058_GPIO_PM_TO_SYS(KO_PMICUART1_BT_PMIC_RXD_37),
		{
			.direction      = PM_GPIO_DIR_OUT,
			.pull           = PM_GPIO_PULL_NO,
			.vin_sel        = PM8058_GPIO_VIN_S3,
			.function       = PM_GPIO_FUNC_2,
			.inv_int_pol    = 0,
			.out_strength   = PM_GPIO_STRENGTH_MED,
			.output_value   = 0,
		},
	};


#if 0
	int bahama_not_marimba = bahama_present();

	if (bahama_not_marimba == -1) {
		printk(KERN_WARNING "%s: bahama_present: %d\n",
				__func__, bahama_not_marimba);
		return -ENODEV;
	}

	if (unlikely(regs_bt_count == 0)) {
		rc = bluetooth_regs_init(bahama_not_marimba);
		if (rc)
			return rc;
	}
#endif
// Fujitsu 2012.01.06 WLAN/BT add end

	if (on) {
// Fujitsu 2012.01.06 WLAN/BT add start
#if 0
		rc = regulator_bulk_enable(regs_bt_count, regs_bt);
		if (rc)
			return rc;

		rc = pmapp_clock_vote(id, PMAPP_CLOCK_ID_DO,
					  PMAPP_CLOCK_VOTE_ON);
		if (rc < 0)
			return -EIO;

		if (machine_is_msm8x55_svlte_surf() ||
				machine_is_msm8x55_svlte_ffa()) {
					rc = marimba_gpio_config(1);
					if (rc < 0)
						return -EIO;
		}

		rc = (bahama_not_marimba ? bahama_bt(on) : marimba_bt(on));
		if (rc < 0)
			return -EIO;

		msleep(10);

		rc = pmapp_clock_vote(id, PMAPP_CLOCK_ID_DO,
					  PMAPP_CLOCK_VOTE_PIN_CTRL);
		if (rc < 0)
			return -EIO;

		if (machine_is_msm8x55_svlte_surf() ||
				machine_is_msm8x55_svlte_ffa()) {
					rc = marimba_gpio_config(0);
					if (rc < 0)
						return -EIO;
		}

		rc = msm_gpios_enable(bt_config_power_on,
			ARRAY_SIZE(bt_config_power_on));

		if (rc < 0)
			return rc;
#endif

		// TX
		rc = pm8xxx_gpio_config(bt_txd_en_21.gpio, &bt_txd_en_21.config);
		if (rc) {
			pr_err("%s: KO_PMICUART1_BT_TXD config failed\n", __func__);
		return -EIO;
		}

		// RX
		rc = pm8xxx_gpio_config(bt_rxd_en_33.gpio, &bt_rxd_en_33.config);
		if (rc) {
			pr_err("%s: KO_PMICUART1_BT_RXD config failed\n", __func__);
			return -EIO;
		}

// Fujitsu 2012.01.06 WLAN/BT change start

//		rc = pm8058_masked_write_non_static(0x1CC, PM8058_UART_MUX_1, PM8058_UART_MUX_MASK);
		rc =  pm8xxx_uart_gpio_mux_ctrl(UART_TX1_RX1);
		if (rc) {
			printk(KERN_ERR "%s: pm8058_masked_write failed=%d\n",__func__,rc);
		}
		
// Fujitsu 2012.01.06 WLAN/BT change end
		rc = pm8xxx_gpio_config(bt_pmic_txd_en_36.gpio, &bt_pmic_txd_en_36.config);
		if (rc) {
			pr_err("%s: KO_PMICUART1_BT_TXD config failed\n", __func__);
			return -EIO;
		}
		rc = pm8xxx_gpio_config(bt_pmic_rxd_en_37.gpio, &bt_pmic_rxd_en_37.config);
		if (rc) {
			pr_err("%s: KO_PMICUART1_BT_RXD config failed\n", __func__);
			return -EIO;
		}

		// GPIO TLMM CONFIG
		for (pin = 0; pin < ARRAY_SIZE(bt_config_power_on); pin++) {
			rc = gpio_tlmm_config(bt_config_power_on[pin],GPIO_CFG_ENABLE);
			if (rc) {
					printk(KERN_ERR
				       "%s: gpio_tlmm_config(%#x)=%d\n",
				       __func__, bt_config_power_on[pin], rc);
					return -EIO;
				}
			}

		// GPIO SET VALUE
		set_power_wlan_bt(1,0);
		gpio_set_value(41, on); /* KO_BT_RESET   		*/
		msleep(110);

// Fujitsu 2012.01.06 WLAN/BT add end

	} else {
// Fujitsu 2012.01.06 WLAN/BT add start
#if 0
		rc = msm_gpios_enable(bt_config_power_off,
					ARRAY_SIZE(bt_config_power_off));
		if (rc < 0)
			return rc;

		/* check for initial RFKILL block (power off) */
		if (platform_get_drvdata(&msm_bt_power_device) == NULL)
			goto out;

		rc = (bahama_not_marimba ? bahama_bt(on) : marimba_bt(on));
		if (rc < 0)
			return -EIO;

		rc = pmapp_clock_vote(id, PMAPP_CLOCK_ID_DO,
					  PMAPP_CLOCK_VOTE_OFF);
		if (rc < 0)
			return -EIO;

		rc = regulator_bulk_disable(regs_bt_count, regs_bt);
		if (rc)
			return rc;
#endif
		// GPIO SET VALUE
		gpio_set_value(41, on); /* KO_BT_RESET 			*/
		msleep(110);
		set_power_wlan_bt(0,0); 

		// PMIC GPIO CONFIG
		rc = pm8xxx_gpio_config(bt_txd_en_21.gpio, &bt_txd_en_21.config);
		if (rc) {
			pr_err("%s: KO_PMICUART1_BT_TXD_21 config failed\n", __func__);
			return -EIO;
		}
		rc = pm8xxx_gpio_config(bt_rxd_en_33.gpio, &bt_rxd_en_33.config);
		if (rc) {
			pr_err("%s: KO_PMICUART1_BT_RXD config failed\n", __func__);
			return -EIO;
		}

		rc = pm8xxx_gpio_config(bt_pmic_txd_en_36.gpio, &bt_pmic_txd_en_36.config);
		if (rc) {
			pr_err("%s: KO_PMICUART1_BT_PMIC_TXD config failed\n", __func__);
			return -EIO;
		}
		rc = pm8xxx_gpio_config(bt_pmic_rxd_en_37.gpio, &bt_pmic_rxd_en_37.config);
		if (rc) {
			pr_err("%s: KO_PMICUART1_BT_PMIC_RXD config failed\n", __func__);
			return -EIO;
		}

		// GPIO TLMM CONFIG
		for (pin = 0; pin < ARRAY_SIZE(bt_config_power_off); pin++) {
			rc = gpio_tlmm_config(bt_config_power_off[pin],GPIO_CFG_ENABLE);
			if (rc) {
				printk(KERN_ERR
					"%s: gpio_tlmm_config(%#x)=%d\n",
					__func__, bt_config_power_off[pin], rc);
				return -EIO;
				}
		}

// Fujitsu 2012.01.06 WLAN/BT add end
	}

// Fujitsu 2012.01.06 WLAN/BT add end
//out:
// Fujitsu 2012.01.06 WLAN/BT add end
	printk(KERN_DEBUG "Bluetooth power switch: %d\n", on);

	return 0;
}

static void __init bt_power_init(void)
{
	msm_bt_power_device.dev.platform_data = &bluetooth_power;
}
#else
#define bt_power_init(x) do {} while (0)
#endif


// Fujitsu 2012.01.06 WLAN/BT add start
/* Status (Power) */
static int wlan_power_flg = 0;
static int bt_power_flg = 0;

// Fujitsu 2011.9.15 Del // Fujitsu 2011.08.22 add start
// Fujitsu 2011.9.15 Del struct wake_lock wakelock;
// Fujitsu 2011.9.15 Del // Fujitsu 2011.08.22 add end


// see also: mach/gpio.h
static int bcm43xx_gpio_tlmm_config(unsigned int config, unsigned int disable)
{
	int retval;
	printk(KERN_INFO "%s: calling gpio_tlmm_config(): gpio=%u, func=0x%02X, dir=0x%02X, pull=0x%02X, drvstr=0x%02X => %d\n",
			__func__, GPIO_PIN(config), GPIO_FUNC(config), GPIO_DIR(config),
			GPIO_PULL(config), GPIO_DRVSTR(config), (retval = gpio_tlmm_config(config, disable)));
	return retval;
}

static int bcm43xx_gpio_request(int gpio, const char* label)
{
	int retval;
	printk(KERN_INFO "%s(): calling gpio_request(%d, %s) => %d\n", __func__, gpio, label,(retval = gpio_request(gpio, label)));
	return retval;
}

static void bcm43xx_gpio_set_value(int gpio, int value)
{
	int oldvalue = 0;
	int newvalue = 0;
//	if (NULL != entry) {
		oldvalue = gpio_get_value(gpio);
		gpio_set_value(gpio, value);
		newvalue = gpio_get_value(gpio);
		printk(KERN_INFO "%s(): gpio_set_value(%d) : %d -> %d\n", __func__, gpio, oldvalue, newvalue);
//	}
}

static void bcm43xx_gpio_direction_output(int gpio, int value)
{
	printk(KERN_INFO "%s(): calling gpio_direction_output(%d) => %d\n", __func__, gpio, value);
	gpio_direction_output(gpio, value);
}

static void bcm43xx_gpio_direction_input(int gpio)
{
	printk(KERN_INFO "%s(): calling gpio_direction_input(%d)\n", __func__, gpio);
	gpio_direction_input(gpio);
}

/* Function (set power) */
void set_power_wlan_bt(int on, int is_wlan)
{
//	printk(KERN_DEBUG "%s S\n", __func__);
printk(KERN_ERR "%s S\n", __func__);

// Fujitsu 2011.08.02 WLAN/BT add
//	unsigned val;

	if ( on )
	{
		printk(KERN_ERR "BT/WLAN-ON \n");
		if ( ( ! wlan_power_flg ) && ( ! bt_power_flg ) )
		{
// Fujitsu 2011.9.15 Del 			// Fujitsu 2011.08.22 add start
// Fujitsu 2011.9.15 Del 			wake_lock_init(&wakelock, WAKE_LOCK_SUSPEND, "wlan_bt_wakelock");
// Fujitsu 2011.9.15 Del 			wake_lock(&wakelock);
// Fujitsu 2011.9.15 Del 			// Fujitsu 2011.08.22 add end

#if 0  // Fujitsu 2011.07.29 WLAN/BT change start
			gpio_set_value(96, on); /* KO_WLAN_BT_PWR_ONOFF */
#else  // Fujitsu 2011.07.29 WLAN/BT change
			bcm43xx_gpio_set_value(96, on); /* KO_WLAN_BT_PWR_ONOFF */
#endif // Fujitsu 2011.07.29 WLAN/BT change end
			msleep(1);
#if 0  // Fujitsu 2011.07.29 WLAN/BT change start
			gpio_set_value(150, on); /* KO_BT_WAKEUP 		*/
#else  // Fujitsu 2011.07.29 WLAN/BT change
			bcm43xx_gpio_set_value(21, on); /* KO_BT_WAKEUP 		*/
#endif // Fujitsu 2011.07.29 WLAN/BT change end

			bcm43xx_gpio_set_value(43, on); /* KO_BT_PWON    		*/
// Fujitsu 2011.09.10 del start debug
//			bcm43xx_gpio_set_value(39, on); /* KO_WLAN_PWON  */
// Fujitsu 2011.09.10 del end debug

		}

// Fujitsu 2011.08.02 WLAN/BT add start
//	val = readl(0xAB00027C);
//	printk(KERN_ERR "%s  0xAB00027C = %d \n", __func__, val);
// Fujitsu 2011.08.02 WLAN/BT add end

		
		if ( is_wlan )
		{
			wlan_power_flg = 1;
		}
		else
		{
			bt_power_flg = 1;
		}
	}
	else
	{
		printk(KERN_ERR "BT/WLAN-OFF \n");
		if ( ( wlan_power_flg ) || ( bt_power_flg ) )
		{
			if ( is_wlan )
			{
				wlan_power_flg = 0;
			}
			else
			{
				bt_power_flg = 0;
			}
			if ( ( ! wlan_power_flg ) && ( ! bt_power_flg ) )
			{
// Fujitsu 2011.09.10 del start debug
//				bcm43xx_gpio_set_value(39, on); /* KO_WLAN_PWON  */
// Fujitsu 2011.09.10 del end debug
				bcm43xx_gpio_set_value(43, on); /* KO_BT_PWON    		*/
#if 0  // Fujitsu 2011.07.29 WLAN/BT change start
				gpio_set_value(150, on); /* KO_BT_WAKEUP 		*/
#else  // Fujitsu 2011.07.29 WLAN/BT change
				bcm43xx_gpio_set_value(21, on); /* KO_BT_WAKEUP 		*/
#endif // Fujitsu 2011.07.29 WLAN/BT change end
#if 0  // Fujitsu 2011.07.29 WLAN/BT change start
				gpio_set_value(96, on); /* KO_WLAN_BT_PWR_ONOFF */
#else  // Fujitsu 2011.07.29 WLAN/BT change
				bcm43xx_gpio_set_value(96, on); /* KO_WLAN_BT_PWR_ONOFF */
#endif // Fujitsu 2011.07.29 WLAN/BT change end
				msleep(1);

// Fujitsu 2011.9.15 Del 				// Fujitsu 2011.08.22 add start
// Fujitsu 2011.9.15 Del 				wake_unlock(&wakelock);
// Fujitsu 2011.9.15 Del 				wake_lock_destroy(&wakelock);
// Fujitsu 2011.9.15 Del 				// Fujitsu 2011.08.22 add end
			}
		}
	}
	
//	printk(KERN_DEBUG "%s E\n", __func__);
printk(KERN_ERR "%s S\n", __func__);
	
	return;
}


/* Function List */
int wlan_power(int on);
int wlan_set_carddetect(void);

/* Platform Data (WLAN) */
static struct wifi_platform_data wlan_wifi_control =
{
	.set_power			=	wlan_power,
	.set_reset			=	NULL,
	.set_carddetect			=	NULL,
	.mem_prealloc			=	NULL,
};

/* Resource (WLAN) */
static struct resource wlan_resources[] =
{
	[0] =
	{
		.name			=	"bcm4329_wlan_irq",
#if 0  // Fujitsu 2011.07.29 WLAN/BT change start
		.start			=	MSM_GPIO_TO_INT(145),
		.end			=	MSM_GPIO_TO_INT(145),
#else  // Fujitsu 2011.07.29 WLAN/BT change
		.start			=	MSM_GPIO_TO_INT(20),
		.end			=	MSM_GPIO_TO_INT(20),
#endif // Fujitsu 2011.07.29 WLAN/BT change end
		.flags			=	IORESOURCE_IRQ | IORESOURCE_IRQ_HIGHLEVEL | IORESOURCE_IRQ_SHAREABLE,
	},
};

/* Platform Device (WLAN) */
static struct platform_device wlan_device =
{
	.name				=	"bcm4329_wlan",
	.num_resources			=	ARRAY_SIZE(wlan_resources),
	.resource			=	wlan_resources,
	.dev				=
	{
		.platform_data		=	&wlan_wifi_control,
	},
};

int wlan_power(int on)
{
	int rc,pin;

//	printk(KERN_DEBUG "%s S\n", __func__);
printk(KERN_ERR "%s S\n", __func__);

	if (on) {
		// GPIO TLMM CONFIG

		for (pin = 0; pin < ARRAY_SIZE(bt_config_power_on); pin++) {
			rc = bcm43xx_gpio_tlmm_config(bt_config_power_on[pin],GPIO_CFG_ENABLE);
			if (rc) {
				printk(KERN_ERR
					"%s: Err gpio_tlmm_config(%#x)=%d\n",
					__func__, bt_config_power_on[pin], rc);
				return -EIO;
			}
		}

printk(KERN_ERR "%s set_power_wlan_bt(1,1) CALL\n", __func__);
		set_power_wlan_bt(1,1);
printk(KERN_ERR "%s set_power_wlan_bt(1,1) CALL E\n", __func__);
		
// Fujitsu 2011.09.10 add start debug
		bcm43xx_gpio_set_value(39, on); /* KO_WLAN_PWON  */
// Fujitsu 2011.09.10 add end debug

// Fujitsu 2011.07.07 del start
//		gpio_set_value(38, on); /* KO_WLAN_RESET */
// Fujitsu 2011.07.07 del end
//		msleep(110);
//		msleep(120);  // Fujitsu 2011.07.13 over 118ms wait WLAN
		msleep(160);  // Fujitsu 2011.08.31 over 150ms wait WLAN
	} else {
		// GPIO TLMM CONFIG

		for (pin = 0; pin < ARRAY_SIZE(bt_config_power_off); pin++) {
			rc = bcm43xx_gpio_tlmm_config(bt_config_power_off[pin],GPIO_CFG_ENABLE);
			if (rc) {
				printk(KERN_ERR
					"%s: gpio_tlmm_config(%#x)=%d\n",
					__func__, bt_config_power_off[pin], rc);
				return -EIO;
			}
		}

		// GPIO SET VALUE
printk(KERN_ERR "%s wlan_set_carddetect CALL\n", __func__);
		wlan_set_carddetect();
printk(KERN_ERR "%s wlan_set_carddetect CALL E\n", __func__);
		msleep(130);
// Fujitsu 2011.07.07 del start
//		gpio_set_value(38, on); /* KO_WLAN_RESET */
// Fujitsu 2011.07.07 del end
#if 0
		// BCM4329 (KO_WLAN_RESET)WLAN = ON
		gpio_set_value(38, on); /* KO_WLAN_RESET */
#else
// Fujitsu 2011.09.13 BCM4330 (KO_WLAN_PWON)WLAN = OFF Flag start
		// BCM4329 WLAN = OFF()
		bcm43xx_gpio_set_value(39, on); /* KO_WLAN_PWON  */
// Fujitsu 2011.09.13 BCM4330 WLAN = OFF Flag start
#endif
		msleep(110);
		
printk(KERN_ERR "%s set_power_wlan_bt(0,1) CALL\n", __func__);
		set_power_wlan_bt(0,1);
printk(KERN_ERR "%s set_power_wlan_bt(0,1) CALL\n", __func__);
	}
//	printk(KERN_DEBUG "%s E\n", __func__);
printk(KERN_ERR "%s S\n", __func__);

	return 0;
}
EXPORT_SYMBOL_GPL(wlan_power);

//void mmc_detect_change_for_bcm4329(unsigned long);

/* Function (set carddetect) */
int wlan_set_carddetect(void)
{
//	mmc_detect_change_for_bcm4329(msecs_to_jiffies(2000));

	return 0;
}

static struct platform_device msm_wlan_power_device = {
	.name = "wlan_power",
};

static void __init wlan_power_init(void)
{
int pin,rc;
		// GPIO TLMM CONFIG

		for (pin = 0; pin < ARRAY_SIZE(bt_config_power_on); pin++) {
			rc = bcm43xx_gpio_tlmm_config(bt_config_power_on[pin],GPIO_CFG_ENABLE);
			if (rc) {
				printk(KERN_ERR
				       "%s: Err gpio_tlmm_config(%#x)=%d\n",
				       __func__, bt_config_power_on[pin], rc);
				return ;
			}
		}

		// GPIO REQUEST
		rc = bcm43xx_gpio_request(96, "KO_WLAN_BT_PWR_ONOFF");// Fujitsu 2011.07.29 WLAN/BT change
		if (rc) {
			printk(KERN_ERR "Err gpio_request KO_WLAN_BT_PWR_ONOFF\n");
		}
		rc = bcm43xx_gpio_request(39, "KO_WLAN_PWON");
		if (rc) {
			printk(KERN_ERR "Err gpio_request KO_WLAN_PWON\n");
		}

// FUJITSU 2012.06.21 WLAN/BT add start
		rc = bcm43xx_gpio_request(20, "KO_WLAN_HOSTWAKEUP");// Fujitsu 2011.07.29 WLAN/BT change
		if (rc) {
			printk(KERN_ERR "Err gpio_request KO_WLAN_HOSTWAKEUP\n");
		}
// FUJITSU 2012.06.21 WLAN/BT add end

// Fujitsu 2011.08.04 WLAN/BT add start
// 2011.08.15 WLAN/BT del start
//	bcm43xx_gpio_direction_input(147);			// BT_HOSTWAKEUP(IN)
//	bcm43xx_gpio_direction_output(141, 0);		// BT_PCM_DCLK(OUT,Low)
//	bcm43xx_gpio_direction_output(140, 0);		// BT_PCM_FSYNC(OUT,Low)
//	bcm43xx_gpio_direction_input(139);			// BT_PCMIN(IN)
//	bcm43xx_gpio_direction_output(138, 0);		// BT_PCMOUT(OUT,Low)

//	bcm43xx_gpio_direction_output(119, 1);		// WLAN_SD_DAT0(OUT,High)
//	bcm43xx_gpio_direction_output(118, 1);		// WLAN_SD_DAT1(OUT,High)
//	bcm43xx_gpio_direction_output(117, 1);		// WLAN_SD_DAT2(OUT,High)
//	bcm43xx_gpio_direction_output(116, 1);		// WLAN_SD_DAT3(OUT,High)
//	bcm43xx_gpio_direction_output(111, 1);		// WLAN_SD_CMD(OUT,High)
//	bcm43xx_gpio_direction_output(110, 0);		// WLAN_SD_CLK(OUT,Low)
// 2011.08.15 WLAN/BT del end

	bcm43xx_gpio_direction_output(96, 0);		// WLAN_BT_PWR_ONOFF(OUT,Low)
// 2011.08.15 WLAN/BT del start
//	bcm43xx_gpio_direction_output(50, 0);		// UART2_CTS(OUT,Low)
//	bcm43xx_gpio_direction_output(49, 0);		// UART2_RTC(OUT,Low)
//	bcm43xx_gpio_direction_output(43, 0);		// BT_PWON(OUT,Low)
//	bcm43xx_gpio_direction_output(41, 0);		// BT_RESET(OUT,Low)
// 2011.08.15 WLAN/BT del end
	bcm43xx_gpio_direction_output(39, 0);		// WLAN_PWON(OUT,Low)

// 2011.08.15 WLAN/BT del start
//	bcm43xx_gpio_direction_output(150, 1);		// BT_WAKEUP(OUT,High)
// 2011.08.15 WLAN/BT del end
	bcm43xx_gpio_direction_input(20);			// WLAN_HOSTWAKEUP(IN)
// Fujitsu 2011.08.04 WLAN/BT add end

	msm_wlan_power_device.dev.platform_data = &wlan_power;
}
// Fujitsu 2012.01.06 WLAN/BT add end

// Fujitsu 2012.01.17 WLAN/BT add start
static struct resource bluesleep_resources[] = {
	{
		.name	= "gpio_host_wake",
		.start	= 147,
		.end	= 147,
		.flags	= IORESOURCE_IO,
	},
	{
		.name	= "gpio_ext_wake",
		.start	= 21,
		.end	= 21,
		.flags	= IORESOURCE_IO,
	},
	{
		.name	= "host_wake",
		.start	= MSM_GPIO_TO_INT(147),
		.end	= MSM_GPIO_TO_INT(147),
		.flags	= IORESOURCE_IRQ,
	},
};

static struct platform_device msm_bluesleep_device = {
	.name = "bluesleep",
	.id		= -1,
	.num_resources	= ARRAY_SIZE(bluesleep_resources),
	.resource	= bluesleep_resources,
};
// Fujitsu 2011.01.17 WLAN/BT add end

static struct msm_psy_batt_pdata msm_psy_batt_data = {
/* FUJITSU:2011-12-01 mod Battery start */
#if 0
	.voltage_min_design 	= 2800,
	.voltage_max_design	= 4300,
#else
	.voltage_min_design 	= 3200,
	.voltage_max_design	= 4200,
#endif
	.avail_chg_sources   	= AC_CHG | USB_CHG | WIRELESS_CHG | MHL_CHG | HOLDER_CHG,
	.batt_technology        = POWER_SUPPLY_TECHNOLOGY_LION,
/* FUJITSU:2011-12-01 mod Battery end */
};

static struct platform_device msm_batt_device = {
	.name 		    = "msm-battery",
	.id		    = -1,
	.dev.platform_data  = &msm_psy_batt_data,
};

static char *msm_adc_fluid_device_names[] = {
	"LTC_ADC1",
	"LTC_ADC2",
	"LTC_ADC3",
};

static char *msm_adc_surf_device_names[] = {
	"XO_ADC",
};

static struct msm_adc_platform_data msm_adc_pdata;

static struct platform_device msm_adc_device = {
	.name   = "msm_adc",
	.id = -1,
	.dev = {
		.platform_data = &msm_adc_pdata,
	},
};

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

static struct platform_device *devices[] __initdata = {
#if defined(CONFIG_SERIAL_MSM) || defined(CONFIG_MSM_SERIAL_DEBUGGER)
	&msm_device_uart2,
#endif
#ifdef CONFIG_MSM_PROC_COMM_REGULATOR
	&msm_proccomm_regulator_dev,
#endif
	&asoc_msm_pcm,
	&asoc_msm_dai0,
	&asoc_msm_dai1,
#if defined (CONFIG_SND_MSM_MVS_DAI_SOC)
	&asoc_msm_mvs,
	&asoc_mvs_dai0,
	&asoc_mvs_dai1,
#endif
	&msm_device_smd,
	&msm_device_dmov,
	&smc91x_device,
	&smsc911x_device,
	&msm_device_nand,
#ifdef CONFIG_USB_MSM_OTG_72K
	&msm_device_otg,
#ifdef CONFIG_USB_GADGET
	&msm_device_gadget_peripheral,
#endif
#endif
#ifdef CONFIG_USB_G_ANDROID
	&android_usb_device,
#endif
	&qsd_device_spi,

#ifdef CONFIG_MSM_SSBI
	&msm_device_ssbi_pmic1,
#endif
#ifdef CONFIG_I2C_SSBI
	&msm_device_ssbi7,
#endif
	&android_pmem_device,
	&msm_fb_device,
	&msm_migrate_pages_device,
	&mddi_toshiba_device,
/* FUJITSU:2011-12-01 del spi devices start */
#if 0
	&lcdc_toshiba_panel_device,
#endif
/* FUJITSU:2011-12-01 del spi devices end */
#ifdef CONFIG_MSM_ROTATOR
	&msm_rotator_device,
#endif
/* FUJITSU:2011-12-01 del spi devices start */
#if 0
	&lcdc_sharp_panel_device,
#endif
/* FUJITSU:2011-12-01 del spi devices end */
	&android_pmem_adsp_device,
	&android_pmem_audio_device,
	&msm_device_i2c,
	&msm_device_i2c_2,
/* FUJITSU:2011-12-01 del uart_dm1 start */
#if 0
	&msm_device_uart_dm1,
#endif
/* FUJITSU:2011-12-01 del uart_dm1 end */
// Fujitsu 2012.01.06 WLAN/BT add start
#ifdef MSM_UARTDM2_USE
	&msm_device_uart_dm2,
#endif
// Fujitsu 2012.01.06 WLAN/BT add end
	
	&hs_device,
#ifdef CONFIG_MSM7KV2_AUDIO
	&msm_aictl_device,
	&msm_mi2s_device,
	&msm_lpa_device,
	&msm_aux_pcm_device,
#endif
	&msm_device_adspdec,
	&qup_device_i2c,
#if defined(CONFIG_MARIMBA_CORE) && \
   (defined(CONFIG_MSM_BT_POWER) || defined(CONFIG_MSM_BT_POWER_MODULE))
	&msm_bt_power_device,
#endif
	&msm_kgsl_3d0,
	&msm_kgsl_2d0,
// Fujitsu 2012.01.17 WLAN/BT add start
	&msm_bluesleep_device,
// Fujitsu 2012.01.17 WLAN/BT add end
#ifdef CONFIG_MT9T013
	&msm_camera_sensor_mt9t013,
#endif
#ifdef CONFIG_MT9D112
	&msm_camera_sensor_mt9d112,
#endif
#ifdef CONFIG_WEBCAM_OV9726
	&msm_camera_sensor_ov9726,
#endif
#ifdef CONFIG_S5K3E2FX
	&msm_camera_sensor_s5k3e2fx,
#endif
#ifdef CONFIG_MT9P012
	&msm_camera_sensor_mt9p012,
#endif
#ifdef CONFIG_MT9E013
	&msm_camera_sensor_mt9e013,
#endif
#ifdef CONFIG_VX6953
	&msm_camera_sensor_vx6953,
#endif
#ifdef CONFIG_SN12M0PZ
	&msm_camera_sensor_sn12m0pz,
#endif
/* FUJITSU:2012_01_19 add camera start */
#ifdef CONFIG_M6MORL_IMX046ES
	&msm_camera_sensor_m6morl_imx046es,
#endif
/* FUJITSU:2012_01_19 add camera end */

/* FUJITSU:2012_03_09 add out_camera start */
#ifdef CONFIG_M6MORL_IMX091
	&msm_camera_sensor_m6morl_imx091,
#endif
/* FUJITSU:2012_03_09 add out_camera end */

/* FUJITSU:2012_01_19 add camera start */
#ifdef CONFIG_S5K6AAFX13
	&msm_camera_sensor_s5k6aafx13,
#endif
/* FUJITSU:2012_01_19 add camera end */
/* FUJITSU:2012_03_12 add in_camera start */
#ifdef CONFIG_RJ6ABA
	&msm_camera_sensor_rj6aba,
#endif
/* FUJITSU:2012_03_12 add in_camera end */

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
	&msm_device_sdio_al,
#endif

#if defined(CONFIG_CRYPTO_DEV_QCRYPTO) || \
		defined(CONFIG_CRYPTO_DEV_QCRYPTO_MODULE)
	&qcrypto_device,
#endif

#if defined(CONFIG_CRYPTO_DEV_QCEDEV) || \
		defined(CONFIG_CRYPTO_DEV_QCEDEV_MODULE)
	&qcedev_device,
#endif

	&msm_batt_device,
	&msm_adc_device,
	&msm_ebi0_thermal,
	&msm_ebi1_thermal,
    &memread_dev,
/* FUJITSU:2011-12-01 add bsp-sensor start */
	&fj_walkmotion_device,
/* FUJITSU:2011-12-01 add bsp-sensor end */
// Fujitsu 2012.01.06 WLAN/BT add start
	&wlan_device,
// Fujitsu 2012.01.06 WLAN/BT add end
/* FUJITSU:2012-04-20 LED start */
	&mobile_led_device
/* FUJITSU:2012-04-20 LED end */
};
/* FUJITSU:2011-12-01 mod i2c gpios 16MA -> 2MA start */
static struct msm_gpio msm_i2c_gpios_hw[] = {
	{ GPIO_CFG(70, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "i2c_scl" },
	{ GPIO_CFG(71, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "i2c_sda" },
};

static struct msm_gpio msm_i2c_gpios_io[] = {
	{ GPIO_CFG(70, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "i2c_scl" },
	{ GPIO_CFG(71, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "i2c_sda" },
};

/* FUJITSU:2012-03-17 mod i2cstart */
static struct msm_gpio qup_i2c_gpios_io[] = {
#if 0
	{ GPIO_CFG(0, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "qup_scl" },
	{ GPIO_CFG(1, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "qup_sda" },
	{ GPIO_CFG(16, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "qup_scl" },
	{ GPIO_CFG(17, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "qup_sda" },
#endif
	{ GPIO_CFG(16, 2, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "qup_scl" },
	{ GPIO_CFG(17, 2, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "qup_sda" },
};

static struct msm_gpio qup_i2c_gpios_hw[] = {
#if 0
	{ GPIO_CFG(0, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "qup_scl" },
	{ GPIO_CFG(1, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "qup_sda" },
	{ GPIO_CFG(16, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "qup_scl" },
	{ GPIO_CFG(17, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "qup_sda" },
#endif
	{ GPIO_CFG(16, 2, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "qup_scl" },
	{ GPIO_CFG(17, 2, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "qup_sda" },
};

/* FUJITSU:2012-03-17 mod i2c end */
/* FUJITSU:2011-12-01 mod i2c gpios 16MA -> 2MA end */

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
static struct regulator *qup_vreg;
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
	if (!IS_ERR_OR_NULL(qup_vreg)) {
		rc = regulator_enable(qup_vreg);
		if (rc) {
			pr_err("%s: regulator_enable failed: %d\n",
			__func__, rc);
		}
	}
#endif
}

static struct msm_i2c_platform_data msm_i2c_pdata = {
/* FUJITSU:2011-12-01 I2C Clock 400Kbps start */
#if 0
	.clk_freq = 100000,
#else
	.clk_freq = 400000,
#endif
/* FUJITSU:2011-12-01 I2C Clock 400Kbps end */
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
	if (msm_gpios_request(qup_i2c_gpios_hw, ARRAY_SIZE(qup_i2c_gpios_hw)))
		pr_err("failed to request I2C gpios\n");

	qup_device_i2c.dev.platform_data = &qup_i2c_pdata;
	/*This needs to be enabled only for OEMS*/
#ifndef CONFIG_QUP_EXCLUSIVE_TO_CAMERA
	qup_vreg = regulator_get(&qup_device_i2c.dev, "lvsw1");
	if (IS_ERR(qup_vreg)) {
		dev_err(&qup_device_i2c.dev,
			"%s: regulator_get failed: %ld\n",
			__func__, PTR_ERR(qup_vreg));
	}
#endif
}

/* FUJITSU:2012-03-27 NAD change start */
#if 1
/* FUJITSU:2011-12-22 add bsp-sensor start */
static struct msm_gpio pedo_gpios[] = {
/* FUJITSU:2012-01-16 bsp-sensor mod start */
//	{ GPIO_CFG(FJ_WM_GPIO_HOSU_I2C2, 1, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "ped_i2c_irq" },
	{ GPIO_CFG(FJ_WM_GPIO_RESET,     0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "ped_rst" },
//	{ GPIO_CFG(FJ_WM_GPIO_HOSU_REL,  0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "ped_release" },
/* FUJITSU:2012-03-27 NAD change end */
	{ GPIO_CFG(FJ_WM_GPIO_HOSU_WUP,  0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "ped_wake" },
	{ GPIO_CFG(FJ_WM_GPIO_HOSU_IRQ,  0, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "ped_irq" },
/* FUJITSU:2012-01-16 bsp-sensor mod end */
};

static void __init pedo_init(void)
{
	int ret = 0;

	ret = msm_gpios_request( pedo_gpios, ARRAY_SIZE(pedo_gpios) );
	ret = msm_gpios_enable( pedo_gpios, ARRAY_SIZE(pedo_gpios) );
}
#endif

static struct msm_gpio compass_gpios[] = {
/* FUJITSU:2012-01-16 bsp-sensor mod start */
/* FUJITSU:2012-03-27 NAD change start */
//	{ GPIO_CFG(JIKI_DRDY_GPIO, 0, GPIO_CFG_INPUT,  GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), "compass" },
	{ GPIO_CFG(JIKI_DRDY_GPIO, 0, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "compass" },
/* FUJITSU:2012-03-27 NAD change end */
/* FUJITSU:2012-01-16 bsp-sensor mod end */
};

static void __init compass_init(void)
{
	int ret = 0;

	ret = msm_gpios_request( compass_gpios, ARRAY_SIZE(compass_gpios) );
	if (ret) {
		printk( KERN_INFO "%s: msm_gpios_request() error[%d]\n", __func__, ret );
	}

	ret = msm_gpios_enable( compass_gpios, ARRAY_SIZE(compass_gpios) );
	if (ret) {
		printk( KERN_INFO "%s: msm_gpios_enable() error[%d]\n", __func__, ret );
	}
}
/* FUJITSU:FUJITSU:2011-12-01 add bsp-sensor end */

#ifdef CONFIG_I2C_SSBI
static struct msm_i2c_ssbi_platform_data msm_i2c_ssbi7_pdata = {
	.rsl_id = "D:CODEC_SSBI",
	.controller_type = MSM_SBI_CTRL_SSBI,
};
#endif

static void __init msm7x30_init_irq(void)
{
	msm_init_irq();
}

static struct msm_gpio msm_nand_ebi2_cfg_data[] = {
	{GPIO_CFG(86, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "ebi2_cs1"},
/* FUJITSU:2011-12-01 del for keypad start */
#if 0
	{GPIO_CFG(115, 2, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "ebi2_busy1"},
#endif
/* FUJITSU:2011-12-01 del for keypad end */
};

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
// Fujitsu 2012.01.07 WLAN/BT add start
#if 0
	{GPIO_CFG(38, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "sdc1_clk"},
	{GPIO_CFG(39, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc1_cmd"},
	{GPIO_CFG(40, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc1_dat_3"},
	{GPIO_CFG(41, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc1_dat_2"},
	{GPIO_CFG(42, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc1_dat_1"},
	{GPIO_CFG(43, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc1_dat_0"},
#endif
// Fujitsu 2012.01.07 WLAN/BT add end
};

static struct msm_gpio sdc2_cfg_data[] = {
/* FUJITSU:2012-05-14 mod for eMMC start */
#if 0
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
#else
	{GPIO_CFG(64, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "sdc2_clk"},
	{GPIO_CFG(65, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), "sdc2_cmd"},
	{GPIO_CFG(66, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), "sdc2_dat_3"},
	{GPIO_CFG(67, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), "sdc2_dat_2"},
	{GPIO_CFG(68, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), "sdc2_dat_1"},
	{GPIO_CFG(69, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), "sdc2_dat_0"},

#ifdef CONFIG_MMC_MSM_SDC2_8_BIT_SUPPORT
	{GPIO_CFG(115, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), "sdc2_dat_4"},
	{GPIO_CFG(114, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), "sdc2_dat_5"},
	{GPIO_CFG(113, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), "sdc2_dat_6"},
	{GPIO_CFG(112, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), "sdc2_dat_7"},
#endif
#endif
/* FUJITSU:2012-05-14 mod for eMMC end */
};

/* FUJITSU:2012-05-14 add for eMMC start */
static struct msm_gpio sdc2_sleep_cfg_data[] = {
	{GPIO_CFG(64, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "sdc2_clk"},
	{GPIO_CFG(65, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), "sdc2_cmd"},
	{GPIO_CFG(66, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), "sdc2_dat_3"},
	{GPIO_CFG(67, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), "sdc2_dat_2"},
	{GPIO_CFG(68, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), "sdc2_dat_1"},
	{GPIO_CFG(69, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), "sdc2_dat_0"},

#ifdef CONFIG_MMC_MSM_SDC2_8_BIT_SUPPORT
	{GPIO_CFG(115, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), "sdc2_dat_4"},
	{GPIO_CFG(114, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), "sdc2_dat_5"},
	{GPIO_CFG(113, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), "sdc2_dat_6"},
	{GPIO_CFG(112, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), "sdc2_dat_7"},
#endif
};
/* FUJITSU:2012-05-14 add for eMMC end */

static struct msm_gpio sdc3_cfg_data[] = {
// Fujitsu 2012.01.06 WLAN/BT add start
#if 0
	{GPIO_CFG(110, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "sdc3_clk"},
	{GPIO_CFG(111, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc3_cmd"},
	{GPIO_CFG(116, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc3_dat_3"},
	{GPIO_CFG(117, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc3_dat_2"},
	{GPIO_CFG(118, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc3_dat_1"},
	{GPIO_CFG(119, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc3_dat_0"},
#endif
	{GPIO_CFG(110, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA), "sdc3_clk"},
	{GPIO_CFG(111, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc3_cmd"},
	{GPIO_CFG(116, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_6MA), "sdc3_dat_3"},
	{GPIO_CFG(117, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_6MA), "sdc3_dat_2"},
	{GPIO_CFG(118, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_6MA), "sdc3_dat_1"},
	{GPIO_CFG(119, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_6MA), "sdc3_dat_0"},
// Fujitsu 2012.01.06 WLAN/BT add end
};

static struct msm_gpio sdc3_sleep_cfg_data[] = {
// Fujitsu 2012.01.06 WLAN/BT add start
#if 0
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
#endif
	{GPIO_CFG(110, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA),
			"sdc3_clk"},
	{GPIO_CFG(111, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA),
			"sdc3_cmd"},
	{GPIO_CFG(116, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_6MA),
			"sdc3_dat_3"},
	{GPIO_CFG(117, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_6MA),
			"sdc3_dat_2"},
	{GPIO_CFG(118, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_6MA),
			"sdc3_dat_1"},
	{GPIO_CFG(119, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_6MA),
			"sdc3_dat_0"},
// Fujitsu 2012.01.06 WLAN/BT add end
};

static struct msm_gpio sdc4_cfg_data[] = {
/* FUJITSU:2012-05-14 mod SDCard start */
#if 0
	{GPIO_CFG(58, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "sdc4_clk"},
	{GPIO_CFG(59, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc4_cmd"},
	{GPIO_CFG(60, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc4_dat_3"},
	{GPIO_CFG(61, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc4_dat_2"},
	{GPIO_CFG(62, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc4_dat_1"},
	{GPIO_CFG(63, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA), "sdc4_dat_0"},
#else
    {GPIO_CFG(58, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA), "sdc4_clk"},
	{GPIO_CFG(59, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA), "sdc4_cmd"},
	{GPIO_CFG(60, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA), "sdc4_dat_3"},
	{GPIO_CFG(61, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA), "sdc4_dat_2"},
	{GPIO_CFG(62, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA), "sdc4_dat_1"},
	{GPIO_CFG(63, 1, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_8MA), "sdc4_dat_0"},
#endif
/* FUJITSU:2012-05-14 mod SDCard end */
};

/* FUJITSU:2012-05-14 add SDCard start */
static struct msm_gpio sdc4_sleep_cfg_data[] = {
    {GPIO_CFG(58, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_8MA), "sdc4_clk"},
	{GPIO_CFG(59, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_8MA), "sdc4_cmd"},
	{GPIO_CFG(60, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_8MA), "sdc4_dat_3"},
	{GPIO_CFG(61, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_8MA), "sdc4_dat_2"},
	{GPIO_CFG(62, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_8MA), "sdc4_dat_1"},
	{GPIO_CFG(63, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_8MA), "sdc4_dat_0"},
};
/* FUJITSU:2012-05-14 add SDCard end */

static struct sdcc_gpio sdcc_cfg_data[] = {
	{
		.cfg_data = sdc1_cfg_data,
		.size = ARRAY_SIZE(sdc1_cfg_data),
		.sleep_cfg_data = NULL,
	},
	{
		.cfg_data = sdc2_cfg_data,
		.size = ARRAY_SIZE(sdc2_cfg_data),
/* FUJITSU:2012-05-14 mod for eMMC start */
#if 0
		.sleep_cfg_data = NULL,
#else
		.sleep_cfg_data = sdc2_sleep_cfg_data,
#endif
/* FUJITSU:2012-05-14 mod for eMMC end */
	},
	{
		.cfg_data = sdc3_cfg_data,
		.size = ARRAY_SIZE(sdc3_cfg_data),
		.sleep_cfg_data = sdc3_sleep_cfg_data,
	},
	{
		.cfg_data = sdc4_cfg_data,
		.size = ARRAY_SIZE(sdc4_cfg_data),
/* FUJITSU:2012-05-14 mod SDCard start */
#if 0
		.sleep_cfg_data = NULL,
#else
		.sleep_cfg_data = sdc4_sleep_cfg_data,
#endif
/* FUJITSU:2012-05-14 mod SDCard end */
	},
};

static struct regulator *sdcc_vreg_data[ARRAY_SIZE(sdcc_cfg_data)];

/* FUJITSU:2011-12-28 del sdcc vreg start */
#if 1
static unsigned long vreg_sts, gpio_sts;
#else
static unsigned long gpio_sts;
#endif
/* FUJITSU:2011-12-28 del sdcc vreg end */

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
/* FUJITSU:2011-12-28 del sdcc vreg start */
#if 1
static uint32_t msm_sdcc_setup_vreg(int dev_id, unsigned int enable)
{
	int rc = 0;
	struct regulator *curr = sdcc_vreg_data[dev_id - 1];
/* FUJITSU:2012-06-11 msmsdcc del start */
//	static int enabled_once[] = {0, 0, 0, 0};
/* FUJITSU:2012-06-11 msmsdcc del end */

	if (test_bit(dev_id, &vreg_sts) == enable)
		return rc;

/* FUJITSU:2012-06-08 del sdcc vreg start */
#if 0
	if (dev_id == 4) {
		if (enable) {
			pr_debug("Enable Vdd dev_%d\n", dev_id);
			gpio_set_value_cansleep(
				PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_SDC4_PWR_EN_N),
						0);
			set_bit(dev_id, &vreg_sts);
		} else {
			pr_debug("Disable Vdd dev_%d\n", dev_id);
			gpio_set_value_cansleep(
				PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_SDC4_PWR_EN_N),
				1);
			clear_bit(dev_id, &vreg_sts);
		}
	}
#endif
/* FUJITSU:2012-06-08 del sdcc vreg end */

/* FUJITSU:2012-06-11 msmsdcc del start */
//	if (!enable || enabled_once[dev_id - 1])
//			return 0;
/* FUJITSU:2012-06-11 msmsdcc del end */
	if (!curr)
		return -ENODEV;

	if (IS_ERR(curr))
		return PTR_ERR(curr);

	if (enable) {
		set_bit(dev_id, &vreg_sts);

/* FUJITSU:2012-06-11 msmsdcc add start */
		pr_info("msmsdcc%d regulator_enable\n", dev_id);
/* FUJITSU:2012-06-11 msmsdcc add end */
		rc = regulator_enable(curr);
		if (rc)
			pr_err("%s: could not enable regulator: %d\n",
					__func__, rc);

/* FUJITSU:2012-06-11 msmsdcc del start */
//		enabled_once[dev_id - 1] = 1;
/* FUJITSU:2012-06-11 msmsdcc del end */
	} else {
		clear_bit(dev_id, &vreg_sts);

/* FUJITSU:2012-06-11 msmsdcc add start */
		pr_info("msmsdcc%d regulator_disable\n", dev_id);
/* FUJITSU:2012-06-11 msmsdcc add end */
		rc = regulator_disable(curr);
		if (rc)
			pr_err("%s: could not disable regulator: %d\n",
					__func__, rc);
	}
	return rc;
}
#endif
/* FUJITSU:2011-12-28 del sdcc vreg end */
static uint32_t msm_sdcc_setup_power(struct device *dv, unsigned int vdd)
{
	int rc = 0;
	struct platform_device *pdev;

	pdev = container_of(dv, struct platform_device, dev);
	rc = msm_sdcc_setup_gpio(pdev->id, (vdd ? 1 : 0));
	if (rc)
		goto out;
/* FUJITSU:2011-12-28 del sdcc vreg start */
#if 1
	if (pdev->id == 4) /* S3 is always ON and cannot be disabled */
		rc = msm_sdcc_setup_vreg(pdev->id, (vdd ? 1 : 0));
#endif
/* FUJITSU:2011-12-28 del sdcc vreg end */
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
#define MBP_INT0 \
	GPIO_CFG(46, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_8MA)

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

static struct regulator_bulk_data mbp_regs_io[2];
static struct regulator_bulk_data mbp_regs_rf[2];
static struct regulator_bulk_data mbp_regs_adc[1];
static struct regulator_bulk_data mbp_regs_core[1];

static int mbp_init_regs(struct device *dev)
{
	struct regulator_bulk_data regs[] = {
		/* Analog and I/O regs */
		{ .supply = "gp4",  .min_uV = 2600000, .max_uV = 2600000 },
		{ .supply = "s3",   .min_uV = 1800000, .max_uV = 1800000 },
		/* RF regs */
		{ .supply = "s2",   .min_uV = 1300000, .max_uV = 1300000 },
		{ .supply = "rf",   .min_uV = 2600000, .max_uV = 2600000 },
		/* ADC regs */
		{ .supply = "s4",   .min_uV = 2200000, .max_uV = 2200000 },
		/* Core regs */
		{ .supply = "gp16", .min_uV = 1200000, .max_uV = 1200000 },
	};

	struct regulator_bulk_data *regptr = regs;
	int rc;

	rc = regulator_bulk_get(dev, ARRAY_SIZE(regs), regs);

	if (rc) {
		dev_err(dev, "%s: could not get regulators: %d\n",
				__func__, rc);
		goto out;
	}

	rc = regulator_bulk_set_voltage(ARRAY_SIZE(regs), regs);

	if (rc) {
		dev_err(dev, "%s: could not set voltages: %d\n",
				__func__, rc);
		goto reg_free;
	}

	memcpy(mbp_regs_io, regptr, sizeof(mbp_regs_io));
	regptr += ARRAY_SIZE(mbp_regs_io);

	memcpy(mbp_regs_rf, regptr, sizeof(mbp_regs_rf));
	regptr += ARRAY_SIZE(mbp_regs_rf);

	memcpy(mbp_regs_adc, regptr, sizeof(mbp_regs_adc));
	regptr += ARRAY_SIZE(mbp_regs_adc);

	memcpy(mbp_regs_core, regptr, sizeof(mbp_regs_core));

	return 0;

reg_free:
	regulator_bulk_free(ARRAY_SIZE(regs), regs);
out:
	return rc;
}

static int mbp_setup_rf_vregs(int state)
{
	return state ?
		regulator_bulk_enable(ARRAY_SIZE(mbp_regs_rf), mbp_regs_rf) :
		regulator_bulk_disable(ARRAY_SIZE(mbp_regs_rf), mbp_regs_rf);
}

static int mbp_setup_vregs(int state)
{
	return state ?
		regulator_bulk_enable(ARRAY_SIZE(mbp_regs_io), mbp_regs_io) :
		regulator_bulk_disable(ARRAY_SIZE(mbp_regs_io), mbp_regs_io);
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
	static bool is_enabled;
	int rc = 0;

	if (enable && !is_enabled) {
		rc = regulator_bulk_enable(ARRAY_SIZE(mbp_regs_core),
				mbp_regs_core);
		if (rc) {
			pr_err("%s: could not enable regulators: %d\n",
					__func__, rc);
		} else {
			is_enabled = true;
		}
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
	return state ?
		regulator_bulk_enable(ARRAY_SIZE(mbp_regs_adc), mbp_regs_adc) :
		regulator_bulk_disable(ARRAY_SIZE(mbp_regs_adc), mbp_regs_adc);
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

	rc = mbp_set_tcxo_en(MBP_OFF);
	if (rc)
		goto exit;
	pr_debug("%s: mbp_set_tcxo_en(MBP_OFF) done\n", __func__);

	rc = mbp_setup_vregs(MBP_OFF);
	if (rc)
		goto exit;
	pr_debug("%s: mbp_setup_vregs(MBP_OFF) done\n", __func__);

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
		rc = mbp_init_regs(dv);
		if (rc) {
			dev_err(dv, "%s: regulator init failed: %d\n",
					__func__, rc);
			goto out;
		}
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
/* FUJITSU:2011-12-01 mod SD_DET start */
		!gpio_get_value_cansleep(
/* FUJITSU:2011-12-01 mod SD_DET end */
			PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_SD_DET - 1));
}
#endif

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
static struct mmc_platform_data msm7x30_sdc2_data = {
	.ocr_mask	= MMC_VDD_165_195 | MMC_VDD_27_28,
	.translate_vdd	= msm_sdcc_setup_power,
#ifdef CONFIG_MMC_MSM_SDC2_8_BIT_SUPPORT
	.mmc_bus_width  = MMC_CAP_8_BIT_DATA,
#else
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
#endif
	.msmsdcc_fmin	= 144000,
	.msmsdcc_fmid	= 24576000,
	.msmsdcc_fmax	= 49152000,
	.nonremovable	= 1,
};
#endif

#ifdef CONFIG_MMC_MSM_SDC3_SUPPORT
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
	.nonremovable	= 0,
};
#endif

#ifdef CONFIG_MMC_MSM_SDC4_SUPPORT
static struct mmc_platform_data msm7x30_sdc4_data = {
	.ocr_mask	= MMC_VDD_27_28 | MMC_VDD_28_29,
	.translate_vdd	= msm_sdcc_setup_power,
	.mmc_bus_width  = MMC_CAP_4_BIT_DATA,
#ifdef CONFIG_MMC_MSM_CARD_HW_DETECTION
	.status      = msm7x30_sdcc_slot_status,
	.status_irq  = PM8058_GPIO_IRQ(PMIC8058_IRQ_BASE, PMIC_GPIO_SD_DET - 1),
	.irq_flags   = IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING,
#endif
	.wpswitch    = msm_sdcc_get_wpswitch,
	.msmsdcc_fmin	= 144000,
	.msmsdcc_fmid	= 24576000,
	.msmsdcc_fmax	= 49152000,
/* FUJITSU: 2012-03-09 Modify Extsd-driver Start*/
	.nonremovable	= 1,
//	.nonremovable	= 0,
/* FUJITSU: 2012-03-09 Modify Extsd-driver End  */
};
#endif

#ifdef CONFIG_MMC_MSM_SDC1_SUPPORT
static int msm_sdc1_lvlshft_enable(void)
{
	static struct regulator *ldo5;
	int rc;

	/* Enable LDO5, an input to the FET that powers slot 1 */

	ldo5 = regulator_get(NULL, "ldo5");

	if (IS_ERR(ldo5)) {
		rc = PTR_ERR(ldo5);
		pr_err("%s: could not get ldo5: %d\n", __func__, rc);
		goto out;
	}

	rc = regulator_set_voltage(ldo5, 2850000, 2850000);
	if (rc) {
		pr_err("%s: could not set ldo5 voltage: %d\n", __func__, rc);
		goto ldo5_free;
	}

	rc = regulator_enable(ldo5);
	if (rc) {
		pr_err("%s: could not enable ldo5: %d\n", __func__, rc);
		goto ldo5_free;
	}

	/* Enable GPIO 35, to turn on the FET that powers slot 1 */
	rc = msm_gpios_request_enable(sdc1_lvlshft_cfg_data,
				ARRAY_SIZE(sdc1_lvlshft_cfg_data));
	if (rc)
		printk(KERN_ERR "%s: Failed to enable GPIO 35\n", __func__);

	rc = gpio_direction_output(GPIO_PIN(sdc1_lvlshft_cfg_data[0].gpio_cfg),
				1);
	if (rc)
		printk(KERN_ERR "%s: Failed to turn on GPIO 35\n", __func__);

	return 0;

ldo5_free:
	regulator_put(ldo5);
out:
	ldo5 = NULL;
	return rc;
}
#endif

static int mmc_regulator_init(int sdcc_no, const char *supply, int uV)
{
	int rc;

	BUG_ON(sdcc_no < 1 || sdcc_no > 4);

	sdcc_no--;

	sdcc_vreg_data[sdcc_no] = regulator_get(NULL, supply);

	if (IS_ERR(sdcc_vreg_data[sdcc_no])) {
		rc = PTR_ERR(sdcc_vreg_data[sdcc_no]);
		pr_err("%s: could not get regulator \"%s\": %d\n",
				__func__, supply, rc);
		goto out;
	}

	rc = regulator_set_voltage(sdcc_vreg_data[sdcc_no], uV, uV);

	if (rc) {
		pr_err("%s: could not set voltage for \"%s\" to %d uV: %d\n",
				__func__, supply, uV, rc);
		goto reg_free;
	}

	return rc;

reg_free:
	regulator_put(sdcc_vreg_data[sdcc_no]);
out:
	sdcc_vreg_data[sdcc_no] = NULL;
	return rc;
}

static void __init msm7x30_init_mmc(void)
{
#ifdef CONFIG_MMC_MSM_SDC1_SUPPORT
	if (mmc_regulator_init(1, "s3", 1800000))
		goto out1;
	msm7x30_sdc1_data.swfi_latency = msm7x30_power_collapse_latency(
		MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT);

	if (machine_is_msm7x30_fluid()) {
		msm7x30_sdc1_data.ocr_mask =  MMC_VDD_27_28 | MMC_VDD_28_29;
		if (msm_sdc1_lvlshft_enable()) {
			pr_err("%s: could not enable level shift\n");
			goto out1;
		}
	}

	msm_add_sdcc(1, &msm7x30_sdc1_data);
out1:
#endif
#ifdef CONFIG_MMC_MSM_SDC2_SUPPORT
	if (mmc_regulator_init(2, "s3", 1800000))
		goto out2;
	msm7x30_sdc2_data.swfi_latency = msm7x30_power_collapse_latency(
		MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT);

	if (machine_is_msm8x55_svlte_surf())
		msm7x30_sdc2_data.msmsdcc_fmax =  24576000;
	if (machine_is_msm8x55_svlte_surf() ||
			machine_is_msm8x55_svlte_ffa()) {
		msm7x30_sdc2_data.sdiowakeup_irq = MSM_GPIO_TO_INT(68);
		msm7x30_sdc2_data.is_sdio_al_client = 1;
	}

	msm_add_sdcc(2, &msm7x30_sdc2_data);
out2:
#endif
#ifdef CONFIG_MMC_MSM_SDC3_SUPPORT
	if (mmc_regulator_init(3, "s3", 1800000))
		goto out3;
	msm7x30_sdc3_data.swfi_latency = msm7x30_power_collapse_latency(
		MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT);

	msm_sdcc_setup_gpio(3, 1);
	msm_add_sdcc(3, &msm7x30_sdc3_data);
out3:
#endif
#ifdef CONFIG_MMC_MSM_SDC4_SUPPORT
#if defined(CONFIG_MACH_F12NAD)
/* FUJITSU:2012-06-07 mod sdcc start */
//	if (mmc_regulator_init(4, "gp6", 2850000))
	if (mmc_regulator_init(4, "gp6", 2900000))
/* FUJITSU:2012-06-07 mod sdcc start */
#else
	if (mmc_regulator_init(4, "mmc", 2850000))
#endif
		return;
	msm7x30_sdc4_data.swfi_latency = msm7x30_power_collapse_latency(
		MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT);

	msm_add_sdcc(4, &msm7x30_sdc4_data);
#endif

}

static void __init msm7x30_init_nand(void)
{
	char *build_id;
	struct flash_platform_data *plat_data;

	build_id = socinfo_get_build_id();
	if (build_id == NULL) {
		pr_err("%s: Build ID not available from socinfo\n", __func__);
		return;
	}

	if (build_id[8] == 'C' &&
			!msm_gpios_request_enable(msm_nand_ebi2_cfg_data,
			ARRAY_SIZE(msm_nand_ebi2_cfg_data))) {
		plat_data = msm_device_nand.dev.platform_data;
		plat_data->interleave = 1;
		printk(KERN_INFO "%s: Interleave mode Build ID found\n",
			__func__);
	}
}

#ifdef CONFIG_SERIAL_MSM_CONSOLE
static struct msm_gpio uart2_config_data[] = {
	{ GPIO_CFG(49, 2, GPIO_CFG_OUTPUT,  GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), "UART2_RFR"},
	{ GPIO_CFG(50, 2, GPIO_CFG_INPUT,   GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), "UART2_CTS"},
	{ GPIO_CFG(51, 2, GPIO_CFG_INPUT,   GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), "UART2_Rx"},
	{ GPIO_CFG(52, 2, GPIO_CFG_OUTPUT,  GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), "UART2_Tx"},
};

static void msm7x30_init_uart2(void)
{
	msm_gpios_request_enable(uart2_config_data,
			ARRAY_SIZE(uart2_config_data));

}
#endif

/* TSIF begin */
#if defined(CONFIG_TSIF) || defined(CONFIG_TSIF_MODULE)

#define TSIF_B_SYNC      GPIO_CFG(37, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA)
#define TSIF_B_DATA      GPIO_CFG(36, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA)
#define TSIF_B_EN        GPIO_CFG(35, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA)
#define TSIF_B_CLK       GPIO_CFG(34, 1, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA)

static const struct msm_gpio tsif_gpios[] = {
	{ .gpio_cfg = TSIF_B_CLK,  .label =  "tsif_clk", },
	{ .gpio_cfg = TSIF_B_EN,   .label =  "tsif_en", },
	{ .gpio_cfg = TSIF_B_DATA, .label =  "tsif_data", },
};

static struct msm_tsif_platform_data tsif_platform_data = {
	.num_gpios = ARRAY_SIZE(tsif_gpios),
	.gpios = tsif_gpios,
	.tsif_pclk = "iface_clk",
	.tsif_ref_clk = "ref_clk",
};
#endif /* defined(CONFIG_TSIF) || defined(CONFIG_TSIF_MODULE) */
/* TSIF end   */

static void __init pmic8058_leds_init(void)
{
/* FUJITSU:2012-04-09 mod LED start */
#if 0
	if (machine_is_msm7x30_surf())
#else
    if (machine_is_f12nad()) {
        pm8058_7x30_data.leds_pdata = &pm8058_leds_data;
	} else if (machine_is_msm7x30_surf())
#endif
/* FUJITSU:2012-04-09 mod LED end */
		pm8058_7x30_data.leds_pdata = &pm8058_surf_leds_data;
	else if (!machine_is_msm7x30_fluid())
		pm8058_7x30_data.leds_pdata = &pm8058_ffa_leds_data;
	else if (machine_is_msm7x30_fluid())
		pm8058_7x30_data.leds_pdata = &pm8058_fluid_leds_data;
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

#if defined(CONFIG_TOUCHSCREEN_TSC2007) || \
	defined(CONFIG_TOUCHSCREEN_TSC2007_MODULE)

#define TSC2007_TS_PEN_INT	20

static struct msm_gpio tsc2007_config_data[] = {
	{ GPIO_CFG(TSC2007_TS_PEN_INT, 0, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA),
	"tsc2007_irq" },
};

static struct regulator_bulk_data tsc2007_regs[] = {
	{ .supply = "s3", .min_uV = 1800000, .max_uV = 1800000 },
	{ .supply = "s2", .min_uV = 1300000, .max_uV = 1300000 },
};

static int tsc2007_init(void)
{
	int rc;

	rc = regulator_bulk_get(NULL, ARRAY_SIZE(tsc2007_regs), tsc2007_regs);

	if (rc) {
		pr_err("%s: could not get regulators: %d\n", __func__, rc);
		goto out;
	}

	rc = regulator_bulk_set_voltage(ARRAY_SIZE(tsc2007_regs), tsc2007_regs);

	if (rc) {
		pr_err("%s: could not set voltages: %d\n", __func__, rc);
		goto reg_free;
	}

	rc = regulator_bulk_enable(ARRAY_SIZE(tsc2007_regs), tsc2007_regs);

	if (rc) {
		pr_err("%s: could not enable regulators: %d\n", __func__, rc);
		goto reg_free;
	}

	rc = msm_gpios_request_enable(tsc2007_config_data,
			ARRAY_SIZE(tsc2007_config_data));
	if (rc) {
		pr_err("%s: Unable to request gpios\n", __func__);
		goto reg_disable;
	}

	return 0;

reg_disable:
	regulator_bulk_disable(ARRAY_SIZE(tsc2007_regs), tsc2007_regs);
reg_free:
	regulator_bulk_free(ARRAY_SIZE(tsc2007_regs), tsc2007_regs);
out:
	return rc;
}

static int tsc2007_get_pendown_state(void)
{
	int rc;

	rc = gpio_get_value(TSC2007_TS_PEN_INT);
	if (rc < 0) {
		pr_err("%s: MSM GPIO %d read failed\n", __func__,
						TSC2007_TS_PEN_INT);
		return rc;
	}

	return (rc == 0 ? 1 : 0);
}

static void tsc2007_exit(void)
{

	regulator_bulk_disable(ARRAY_SIZE(tsc2007_regs), tsc2007_regs);
	regulator_bulk_free(ARRAY_SIZE(tsc2007_regs), tsc2007_regs);

	msm_gpios_disable_free(tsc2007_config_data,
		ARRAY_SIZE(tsc2007_config_data));
}

static int tsc2007_power_shutdown(bool enable)
{
	int rc;

	rc = (enable == false) ?
		regulator_bulk_enable(ARRAY_SIZE(tsc2007_regs), tsc2007_regs) :
		regulator_bulk_disable(ARRAY_SIZE(tsc2007_regs), tsc2007_regs);

	if (rc) {
		pr_err("%s: could not %sable regulators: %d\n",
				__func__, enable ? "dis" : "en", rc);
		return rc;
	}

	if (enable == false)
		msleep(20);

	return 0;
}

static struct tsc2007_platform_data tsc2007_ts_data = {
	.model = 2007,
	.x_plate_ohms = 300,
	.min_x		= 210,
	.max_x		= 3832,
	.min_y		= 150,
	.max_y		= 3936,
	.irq_flags    = IRQF_TRIGGER_LOW,
	.init_platform_hw = tsc2007_init,
	.exit_platform_hw = tsc2007_exit,
	.power_shutdown	  = tsc2007_power_shutdown,
	.invert_x	  = true,
	.invert_y	  = true,
	/* REVISIT: Temporary fix for reversed pressure */
	.invert_z1	  = true,
	.invert_z2	  = true,
	.get_pendown_state = tsc2007_get_pendown_state,
};

static struct i2c_board_info tsc_i2c_board_info[] = {
	{
		I2C_BOARD_INFO("tsc2007", 0x48),
		.irq		= MSM_GPIO_TO_INT(TSC2007_TS_PEN_INT),
		.platform_data = &tsc2007_ts_data,
	},
};
#endif

static struct regulator_bulk_data regs_isa1200[] = {
	{ .supply = "gp7",  .min_uV = 1800000, .max_uV = 1800000 },
	{ .supply = "gp10", .min_uV = 2600000, .max_uV = 2600000 },
};

static int isa1200_power(int vreg_on)
{
	int rc = 0;

	rc = vreg_on ?
		regulator_bulk_enable(ARRAY_SIZE(regs_isa1200), regs_isa1200) :
		regulator_bulk_disable(ARRAY_SIZE(regs_isa1200), regs_isa1200);

	if (rc) {
		pr_err("%s: could not %sable regulators: %d\n",
				__func__, vreg_on ? "en" : "dis", rc);
		goto out;
	}

	/* vote for DO buffer */
	rc = pmapp_clock_vote("VIBR", PMAPP_CLOCK_ID_DO,
		vreg_on ? PMAPP_CLOCK_VOTE_ON : PMAPP_CLOCK_VOTE_OFF);
	if (rc)	{
		pr_err("%s: unable to %svote for d0 clk\n",
			__func__, vreg_on ? "" : "de-");
		goto vreg_fail;
	}

	return 0;

vreg_fail:
	if (vreg_on)
		regulator_bulk_disable(ARRAY_SIZE(regs_isa1200), regs_isa1200);
	else
		regulator_bulk_enable(ARRAY_SIZE(regs_isa1200), regs_isa1200);
out:
	return rc;
}

static int isa1200_dev_setup(bool enable)
{
	int rc;

	if (enable == true) {
		rc = regulator_bulk_get(NULL, ARRAY_SIZE(regs_isa1200),
				regs_isa1200);

		if (rc) {
			pr_err("%s: could not get regulators: %d\n",
					__func__, rc);
			goto out;
		}

		rc = regulator_bulk_set_voltage(ARRAY_SIZE(regs_isa1200),
				regs_isa1200);
		if (rc) {
			pr_err("%s: could not set voltages: %d\n",
					__func__, rc);
			goto reg_free;
		}

		rc = gpio_tlmm_config(GPIO_CFG(HAP_LVL_SHFT_MSM_GPIO, 0,
				GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,
				GPIO_CFG_2MA), GPIO_CFG_ENABLE);
		if (rc) {
			pr_err("%s: Could not configure gpio %d\n",
					__func__, HAP_LVL_SHFT_MSM_GPIO);
			goto reg_free;
		}

		rc = gpio_request(HAP_LVL_SHFT_MSM_GPIO, "haptics_shft_lvl_oe");
		if (rc) {
			pr_err("%s: unable to request gpio %d (%d)\n",
					__func__, HAP_LVL_SHFT_MSM_GPIO, rc);
			goto reg_free;
		}

		gpio_set_value(HAP_LVL_SHFT_MSM_GPIO, 1);
	} else {
		regulator_bulk_free(ARRAY_SIZE(regs_isa1200), regs_isa1200);
		gpio_free(HAP_LVL_SHFT_MSM_GPIO);
	}

	return 0;

reg_free:
	regulator_bulk_free(ARRAY_SIZE(regs_isa1200), regs_isa1200);
out:
	return rc;
}
static struct isa1200_platform_data isa1200_1_pdata = {
	.name = "vibrator",
	.power_on = isa1200_power,
	.dev_setup = isa1200_dev_setup,
	.pwm_ch_id = 1, /*channel id*/
	/*gpio to enable haptic*/
	.hap_en_gpio = PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_HAP_ENABLE),
	.hap_len_gpio = -1,
	.max_timeout = 15000,
	.mode_ctrl = PWM_GEN_MODE,
	.pwm_fd = {
		.pwm_div = 256,
	},
	.is_erm = false,
	.smart_en = true,
	.ext_clk_en = true,
	.chip_en = 1,
};

static struct i2c_board_info msm_isa1200_board_info[] = {
	{
		I2C_BOARD_INFO("isa1200_1", 0x90>>1),
		.platform_data = &isa1200_1_pdata,
	},
};


static int kp_flip_mpp_config(void)
{
	struct pm8xxx_mpp_config_data kp_flip_mpp = {
		.type = PM8XXX_MPP_TYPE_D_INPUT,
		.level = PM8018_MPP_DIG_LEVEL_S3,
		.control = PM8XXX_MPP_DIN_TO_INT,
	};

	return pm8xxx_mpp_config(PM8058_MPP_PM_TO_SYS(PM_FLIP_MPP),
						&kp_flip_mpp);
}

static struct flip_switch_pdata flip_switch_data = {
	.name = "kp_flip_switch",
	.flip_gpio = PM8058_GPIO_PM_TO_SYS(PM8058_GPIOS) + PM_FLIP_MPP,
	.left_key = KEY_OPEN,
	.right_key = KEY_CLOSE,
	.active_low = 0,
	.wakeup = 1,
	.flip_mpp_config = kp_flip_mpp_config,
};

static struct platform_device flip_switch_device = {
	.name   = "kp_flip_switch",
	.id	= -1,
	.dev    = {
		.platform_data = &flip_switch_data,
	}
};

/* FUJITSU:2011-12-01 del tma300 start */
#if 0
static struct regulator_bulk_data regs_tma300[] = {
	{ .supply = "gp6", .min_uV = 3050000, .max_uV = 3100000 },
	{ .supply = "gp7", .min_uV = 1800000, .max_uV = 1800000 },
};

static int tma300_power(int vreg_on)
{
	int rc;

	rc = vreg_on ?
		regulator_bulk_enable(ARRAY_SIZE(regs_tma300), regs_tma300) :
		regulator_bulk_disable(ARRAY_SIZE(regs_tma300), regs_tma300);

	if (rc)
		pr_err("%s: could not %sable regulators: %d\n",
				__func__, vreg_on ? "en" : "dis", rc);
	return rc;
}

#define TS_GPIO_IRQ 150

static int tma300_dev_setup(bool enable)
{
	int rc;

	if (enable) {
		rc = regulator_bulk_get(NULL, ARRAY_SIZE(regs_tma300),
				regs_tma300);

		if (rc) {
			pr_err("%s: could not get regulators: %d\n",
					__func__, rc);
			goto out;
		}

		rc = regulator_bulk_set_voltage(ARRAY_SIZE(regs_tma300),
				regs_tma300);

		if (rc) {
			pr_err("%s: could not set voltages: %d\n",
					__func__, rc);
			goto reg_free;
		}

		/* enable interrupt gpio */
		rc = gpio_tlmm_config(GPIO_CFG(TS_GPIO_IRQ, 0, GPIO_CFG_INPUT,
				GPIO_CFG_PULL_UP, GPIO_CFG_6MA), GPIO_CFG_ENABLE);
		if (rc) {
			pr_err("%s: Could not configure gpio %d\n",
					__func__, TS_GPIO_IRQ);
			goto reg_free;
		}

		/* virtual keys */
		tma300_vkeys_attr.attr.name = "virtualkeys.msm_tma300_ts";
		properties_kobj = kobject_create_and_add("board_properties",
					NULL);
		if (!properties_kobj) {
			pr_err("%s: failed to create a kobject "
					"for board_properties\n", __func__);
			rc = -ENOMEM;
			goto reg_free;
		}
		rc = sysfs_create_group(properties_kobj,
				&tma300_properties_attr_group);
		if (rc) {
			pr_err("%s: failed to create a sysfs entry %s\n",
					__func__, tma300_vkeys_attr.attr.name);
			goto kobj_free;
		}
	} else {
		regulator_bulk_free(ARRAY_SIZE(regs_tma300), regs_tma300);
		/* destroy virtual keys */
		if (properties_kobj) {
			sysfs_remove_group(properties_kobj,
				&tma300_properties_attr_group);
			kobject_put(properties_kobj);
		}
	}
	return 0;

kobj_free:
	kobject_put(properties_kobj);
	properties_kobj = NULL;
reg_free:
	regulator_bulk_free(ARRAY_SIZE(regs_tma300), regs_tma300);
out:
	return rc;
}

static struct cy8c_ts_platform_data cy8ctma300_pdata = {
	.power_on = tma300_power,
	.dev_setup = tma300_dev_setup,
	.ts_name = "msm_tma300_ts",
	.dis_min_x = 0,
	.dis_max_x = 479,
	.dis_min_y = 0,
	.dis_max_y = 799,
	.res_x	 = 479,
	.res_y	 = 1009,
	.min_tid = 1,
	.max_tid = 255,
	.min_touch = 0,
	.max_touch = 255,
	.min_width = 0,
	.max_width = 255,
	.invert_y = 1,
	.nfingers = 4,
	.irq_gpio = TS_GPIO_IRQ,
	.resout_gpio = -1,
};

static struct i2c_board_info cy8ctma300_board_info[] = {
	{
		I2C_BOARD_INFO("cy8ctma300", 0x2),
		.platform_data = &cy8ctma300_pdata,
	}
};
#endif
/* FUJITSU:2011-12-01 del TMA300 end */

/* FUJITSU:2012-04-05 mod gpio function start */
int factory_mode(void);
static int f12nad_dumy_key_mode(void)
{
#ifdef F12NAD_DEBUG
//#warning HW Key F12NAD_DEBUG in
    if (factory_mode()){
        return 0;
    }else{
        return 1;
    }
#else
    return 0;
#endif
}
/* FUJITSU:2012-04-05 mod gpio function end */


static void __init msm7x30_init(void)
{
	int rc;
	unsigned smem_size;
	uint32_t usb_hub_gpio_cfg_value = GPIO_CFG(56,
						0,
						GPIO_CFG_OUTPUT,
						GPIO_CFG_NO_PULL,
						GPIO_CFG_2MA);
	uint32_t soc_version = 0;

	soc_version = socinfo_get_version();

	msm_clock_init(&msm7x30_clock_init_data);
#ifdef CONFIG_SERIAL_MSM_CONSOLE
	msm7x30_init_uart2();
#endif
	msm_spm_init(&msm_spm_data, 1);
	acpuclk_init(&acpuclk_7x30_soc_data);
	if (machine_is_msm7x30_surf() || machine_is_msm7x30_fluid())
		msm7x30_cfg_smsc911x();

#ifdef CONFIG_USB_MSM_OTG_72K
	if (SOCINFO_VERSION_MAJOR(soc_version) >= 2 &&
			SOCINFO_VERSION_MINOR(soc_version) >= 1) {
		pr_debug("%s: SOC Version:2.(1 or more)\n", __func__);
		msm_otg_pdata.ldo_set_voltage = 0;
	}

	msm_device_otg.dev.platform_data = &msm_otg_pdata;
#ifdef CONFIG_USB_GADGET
	msm_otg_pdata.swfi_latency =
 	msm_pm_data
 	[MSM_PM_SLEEP_MODE_RAMP_DOWN_AND_WAIT_FOR_INTERRUPT].latency;
	msm_device_gadget_peripheral.dev.platform_data = &msm_gadget_pdata;
#endif
#endif
/* FUJITSU:2011-12-01 del uart_dm1 start */
#if 0
	msm_uart_dm1_pdata.wakeup_irq = gpio_to_irq(136);
	msm_device_uart_dm1.dev.platform_data = &msm_uart_dm1_pdata;
#endif
/* FUJITSU:2011-12-01 del uart_dm1 end */

// Fujitsu 2012.01.06 WLAN/BT add start
#ifdef MSM_UARTDM2_USE
	msm_uart_dm2_pdata.wakeup_irq = gpio_to_irq(51);
	msm_device_uart_dm2.dev.platform_data = &msm_uart_dm2_pdata;
#endif
// Fujitsu 2012.01.06 WLAN/BT add end

#if defined(CONFIG_TSIF) || defined(CONFIG_TSIF_MODULE)
	msm_device_tsif.dev.platform_data = &tsif_platform_data;
#endif
	if (machine_is_msm7x30_fluid()) {
		msm_adc_pdata.dev_names = msm_adc_fluid_device_names;
		msm_adc_pdata.num_adc = ARRAY_SIZE(msm_adc_fluid_device_names);
	} else {
		msm_adc_pdata.dev_names = msm_adc_surf_device_names;
		msm_adc_pdata.num_adc = ARRAY_SIZE(msm_adc_surf_device_names);
	}

	pmic8058_leds_init();

	buses_init();

#ifdef CONFIG_MSM_SSBI
	msm_device_ssbi_pmic1.dev.platform_data =
				&msm7x30_ssbi_pm8058_pdata;
#endif

	platform_add_devices(msm_footswitch_devices,
			     msm_num_footswitch_devices);
	platform_add_devices(devices, ARRAY_SIZE(devices));
#ifdef CONFIG_USB_EHCI_MSM_72K
	msm_add_host(0, &msm_usb_host_pdata);
#endif
	msm7x30_init_mmc();
	msm7x30_init_nand();
	msm_qsd_spi_init();

#ifdef CONFIG_SPI_QSD
	if (machine_is_msm7x30_fluid())
		spi_register_board_info(lcdc_sharp_spi_board_info,
			ARRAY_SIZE(lcdc_sharp_spi_board_info));
	else
		spi_register_board_info(lcdc_toshiba_spi_board_info,
			ARRAY_SIZE(lcdc_toshiba_spi_board_info));
#endif

/* FUJITSU:2011-12-01 del DTV start */
#if 0
	atv_dac_power_init();
#endif
/* FUJITSU:2011-12-01 del DTV end */
/* FUJITSU:2011-12-01 del BMA150 start */
#ifdef CONFIG_BOSCH_BMA150
	sensors_ldo_init();
#endif
/* FUJITSU:2011-12-01 del BMA150 end */
/* FUJITSU:2011-12-01 del DTV start */
#if 0
	hdmi_init_regs();
#endif
/* FUJITSU:2011-12-01 del DTV end */
	msm_fb_add_devices();
	msm_pm_set_platform_data(msm_pm_data, ARRAY_SIZE(msm_pm_data));
	BUG_ON(msm_pm_boot_init(&msm_pm_boot_pdata));
	msm_device_i2c_init();
	msm_device_i2c_2_init();
	qup_device_i2c_init();
/* FUJITSU:2011-12-01 add bsp-sensor start */
/* FUJITSU:2012-03-27 NAD change start */
	pedo_init();
/* FUJITSU:2012-03-27 NAD change end */
/* FUJITSU:2011-12-01 add bsp-sensor end */
	msm7x30_init_marimba();
#ifdef CONFIG_MSM7KV2_AUDIO
	snddev_poweramp_gpio_init();
	snddev_hsed_voltage_init();
	aux_pcm_gpio_init();
#endif
/* FUJITSU:2011-12-1 add VIB start */
	msm_init_pmic_vibrator();
/* FUJITSU:2011-12-1 add VIB end */

/* FUJITSU:2011-12-01 add bsp-sensor start */
	compass_init();
/* FUJITSU:2011-12-01 add bsp-sensor end */

/* FUJITSU:2012-01-13 factory start */
	factory_read_nvitem_init();
/* FUJITSU:2012-01-13 factory end */

/* FUJITSU:2011-12-22 add audio gpio start */
	gpio_tlmm_config( GPIO_CFG(19, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE );
	gpio_tlmm_config( GPIO_CFG(127, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE );
	gpio_set_value(127,0);
/* FUJITSU:2011-12-22 add audio gpio end */

/* FUJITSU:2012-02-29 add keypad gpio start */
    gpio_tlmm_config( GPIO_CFG(180, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE );
	gpio_tlmm_config( GPIO_CFG(174, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE );
	gpio_tlmm_config( GPIO_CFG(173, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE );
	gpio_tlmm_config( GPIO_CFG(172, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE );
	gpio_tlmm_config( GPIO_CFG(145, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE );
/* FUJITSU:2012-02-29 add keypad gpio end */

	i2c_register_board_info(0, msm_i2c_board_info,
			ARRAY_SIZE(msm_i2c_board_info));

/* FUJITSU:2011-12-01 del TSADC start */
#if 0
	if (!machine_is_msm8x55_svlte_ffa() && !machine_is_msm7x30_fluid())
		marimba_pdata.tsadc = &marimba_tsadc_pdata;
#endif
/* FUJITSU:2011-12-01 del TSADC end */

/* FUJITSU:2011-12-01 del cyttsp start */
#if 0
	if (machine_is_msm7x30_fluid())
		i2c_register_board_info(0, cy8info,
					ARRAY_SIZE(cy8info));
#endif
/* FUJITSU:2011-12-01 del cyttsp end */

/* FUJITSU:2012-03-15 del cyttsp start */
#if 0
	i2c_register_board_info(4, cy8info,ARRAY_SIZE(cy8info));
#endif
/* FUJITSU:2012-03-15 del cyttsp end */

/* FUJITSU:2012-03-15 del touchscreen start */
	i2c_register_board_info( 4, mxt_info, ARRAY_SIZE(mxt_info) );
/* FUJITSU:2012-03-15 del touchscreen end */

/* FUJITSU:2012-03-15 CYPRESS TOUCH PRESS add start */
    if(system_rev >= 0x0c) {
        i2c_register_board_info(4, cyttsp_press_info, ARRAY_SIZE(cyttsp_press_info));
    } else {
        i2c_register_board_info(0, cyttsp_press_info, ARRAY_SIZE(cyttsp_press_info));
    }
/* FUJITSU:2012-03-15 CYPRESS TOUCH PRESS add end */

#ifdef CONFIG_BOSCH_BMA150
	if (machine_is_msm7x30_fluid())
		i2c_register_board_info(0, bma150_board_info,
					ARRAY_SIZE(bma150_board_info));
#endif

	i2c_register_board_info(2, msm_marimba_board_info,
			ARRAY_SIZE(msm_marimba_board_info));

	i2c_register_board_info(2, msm_i2c_gsbi7_timpani_info,
			ARRAY_SIZE(msm_i2c_gsbi7_timpani_info));

	i2c_register_board_info(4 /* QUP ID */, msm_camera_boardinfo,
				ARRAY_SIZE(msm_camera_boardinfo));

	bt_power_init();
// Fujitsu 2012.01.06 WLAN/BT add start
	wlan_power_init();  
// Fujitsu 2012.01.06 WLAN/BT add end
#ifdef CONFIG_I2C_SSBI
	msm_device_ssbi7.dev.platform_data = &msm_i2c_ssbi7_pdata;
#endif
	if (machine_is_msm7x30_fluid())
		i2c_register_board_info(0, msm_isa1200_board_info,
			ARRAY_SIZE(msm_isa1200_board_info));

#if defined(CONFIG_TOUCHSCREEN_TSC2007) || \
	defined(CONFIG_TOUCHSCREEN_TSC2007_MODULE)
	if (machine_is_msm8x55_svlte_ffa())
		i2c_register_board_info(2, tsc_i2c_board_info,
				ARRAY_SIZE(tsc_i2c_board_info));
#endif

	if (machine_is_msm7x30_surf())
		platform_device_register(&flip_switch_device);

	pm8058_gpios_init();

/* FUJITSU:2012-04-10 add keypad start */
	if (f12nad_dumy_key_mode()){
		platform_device_register(&keypad_device_fj_d_debug); 
	}else{
	platform_device_register(&keypad_device_fj_d); 
	}
/* FUJITSU:2012-04-10 add keypad end */

/* FUJITSU:2011-12-01 del TMA300 start */
#if 0
	if (machine_is_msm7x30_fluid()) {
		/* Initialize platform data for fluid v2 hardware */
		if (SOCINFO_VERSION_MAJOR(
				socinfo_get_platform_version()) == 2) {
			cy8ctma300_pdata.res_y = 920;
			cy8ctma300_pdata.invert_y = 0;
		}
		i2c_register_board_info(0, cy8ctma300_board_info,
			ARRAY_SIZE(cy8ctma300_board_info));
	}
#endif
/* FUJITSU:2011-12-01 del TMA300 end */
	if (machine_is_msm8x55_svlte_surf() || machine_is_msm8x55_svlte_ffa()) {
		rc = gpio_tlmm_config(usb_hub_gpio_cfg_value, GPIO_CFG_ENABLE);
		if (rc)
			pr_err("%s: gpio_tlmm_config(%#x)=%d\n",
				__func__, usb_hub_gpio_cfg_value, rc);
	}

/* FUJITSU:2011-12-01 add bsp-sensor start */
	i2c_register_board_info(0, compassinfo,ARRAY_SIZE(compassinfo));
/* FUJITSU:2011-12-01 add bsp-sensor end */

/* FUJITSU:2012.03.27  HapticFeedback [start] */
        i2c_register_board_info(0, ts5k_info,ARRAY_SIZE(ts5k_info));
/* FUJITSU:2012.03.27  HapticFeedback [end]   */

/* FUJITSU:2012-03-17 mod i2c start */
#if 1
    gpio_request( 16, "gpio_ts_scl" );
    gpio_request( 17, "gpio_ts_sda" );
    gpio_tlmm_config( GPIO_CFG(16, 2, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE );
    gpio_tlmm_config( GPIO_CFG(17, 2, GPIO_CFG_INPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE );
#endif
/* FUJITSU:2012-03-17 mod i2c end */
    boot_reason = *(unsigned int *)
		(smem_get_entry(SMEM_POWER_ON_STATUS_INFO, &smem_size));
	printk(KERN_NOTICE "Boot Reason = 0x%02x\n", boot_reason);
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

static unsigned fluid_pmem_adsp_size = MSM_FLUID_PMEM_ADSP_SIZE;
static int __init fluid_pmem_adsp_size_setup(char *p)
{
	fluid_pmem_adsp_size = memparse(p, NULL);
	return 0;
}
early_param("fluid_pmem_adsp_size", fluid_pmem_adsp_size_setup);

static unsigned pmem_audio_size = MSM_PMEM_AUDIO_SIZE;
static int __init pmem_audio_size_setup(char *p)
{
	pmem_audio_size = memparse(p, NULL);
	return 0;
}
early_param("pmem_audio_size", pmem_audio_size_setup);

static unsigned pmem_kernel_ebi0_size = PMEM_KERNEL_EBI0_SIZE;
static int __init pmem_kernel_ebi0_size_setup(char *p)
{
	pmem_kernel_ebi0_size = memparse(p, NULL);
	return 0;
}
early_param("pmem_kernel_ebi0_size", pmem_kernel_ebi0_size_setup);

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

static void __init size_pmem_devices(void)
{
#ifdef CONFIG_ANDROID_PMEM
	unsigned long size;

	if machine_is_msm7x30_fluid()
		size = fluid_pmem_adsp_size;
	else
		size = pmem_adsp_size;
	android_pmem_adsp_pdata.size = size;
	android_pmem_audio_pdata.size = pmem_audio_size;
	android_pmem_pdata.size = pmem_sf_size;
#endif
}

static void __init reserve_memory_for(struct android_pmem_platform_data *p)
{
	msm7x30_reserve_table[p->memory_type].size += p->size;
}

static void __init reserve_pmem_memory(void)
{
#ifdef CONFIG_ANDROID_PMEM
	reserve_memory_for(&android_pmem_adsp_pdata);
	reserve_memory_for(&android_pmem_audio_pdata);
	reserve_memory_for(&android_pmem_pdata);
	msm7x30_reserve_table[MEMTYPE_EBI0].size += pmem_kernel_ebi0_size;
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
		return MEMTYPE_EBI0;
	if (paddr >= 0x40000000 && paddr < 0x80000000)
		return MEMTYPE_EBI1;
	return MEMTYPE_NONE;
}

static struct reserve_info msm7x30_reserve_info __initdata = {
	.memtype_reserve_table = msm7x30_reserve_table,
	.calculate_reserve_sizes = msm7x30_calculate_reserve_sizes,
	.paddr_to_memtype = msm7x30_paddr_to_memtype,
};

static void __init msm7x30_reserve(void)
{
	reserve_info = &msm7x30_reserve_info;
	msm_reserve();
}

static void __init msm7x30_allocate_memory_regions(void)
{
	void *addr;
	unsigned long size;

	size = fb_size ? : MSM_FB_SIZE;
	addr = alloc_bootmem_align(size, 0x1000);
	msm_fb_resources[0].start = __pa(addr);
	msm_fb_resources[0].end = msm_fb_resources[0].start + size - 1;
	pr_info("allocating %lu bytes at %p (%lx physical) for fb\n",
		size, addr, __pa(addr));
}

static void __init msm7x30_map_io(void)
{
	msm_shared_ram_phys = 0x00100000;
	msm_map_msm7x30_io();
	if (socinfo_init() < 0)
		printk(KERN_ERR "%s: socinfo_init() failed!\n",
		       __func__);
}

static void __init msm7x30_init_early(void)
{
	msm7x30_allocate_memory_regions();
}
/* FUJITSU:2011-12-01 add PROXIMITY setting start */
/* FUJITSU:2012-01-16 bsp-sensor del start */
//int msm_prodrv_read_nvitem(unsigned *id, unsigned *data)
//{
//    int rc;
//
//    if(id == NULL || data == NULL)
//        return -1;
//
//    /* read NV  */
//    rc = msm_proc_comm(PCOM_OEM_011, id, data);
//    if (rc != 0)
//    {
//        /* unsuccess */
//        printk("proximity: nv read error %d\n",rc);
//    }
//
//    return rc;
//}
//EXPORT_SYMBOL(msm_prodrv_read_nvitem);
/* FUJITSU:2012-01-16 bsp-sensor del end */
/* FUJITSU:2011-12-01 add PROXIMITY setting end */

/* FUJITSU:2011-12-01 spi spio setting start */
int set_spi_gpio_exclusive_ctrl(int runtype, int csno)
{
	unsigned long spinlock_flags;
	int set_spi_gpio_use_state_old;
	int ret = 0;
	/*******************/
	/* parameter check */
	/*******************/
	if (((csno < 0) || (csno > 3)) || ((runtype < 0) || (runtype > 1))) {
		pr_err("%s: csno input data failed csno = %d runtype = %d\n", __func__, csno, runtype);
		return -2;
	}

	spin_lock_irqsave(&set_spi_gpio_use_state_lock, spinlock_flags);

	/********************/
	/* update use_state */
	/********************/
	set_spi_gpio_use_state_old = set_spi_gpio_use_state;
	if (runtype == 0) {
		set_spi_gpio_use_state &= ~(1<<csno);
	}
	else {
    	set_spi_gpio_use_state |= (1<<csno);
	}

	/***************************/
	/* use_state changing 0->1 */
	/***************************/
	if ((set_spi_gpio_use_state_old == 0) && (set_spi_gpio_use_state != 0)) {
		/* -- start gpio setting -- */
	    /* spi_clk */
	    ret = gpio_tlmm_config(GPIO_CFG(45, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), GPIO_CFG_ENABLE);
	    if (ret) {
			pr_err("%s: Could not configure gpio errno = %x portno = %d runtype = %d csno = %d\n", __func__, ret, 45, runtype, csno);
			spin_unlock_irqrestore(&set_spi_gpio_use_state_lock, spinlock_flags);
			return -1;
	    }
	    /* spi_mosi */
	    ret = gpio_tlmm_config(GPIO_CFG(47, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), GPIO_CFG_ENABLE);
	    if (ret) {
			pr_err("%s: Could not configure gpio errno = %x portno = %d runtype = %d csno = %d\n", __func__, ret, 47, runtype, csno);
			spin_unlock_irqrestore(&set_spi_gpio_use_state_lock, spinlock_flags);
			return -1;
	    }
	    /* spi_miso */
	    ret = gpio_tlmm_config(GPIO_CFG(48, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	    if (ret) {
			pr_err("%s: Could not configure gpio errno = %x portno = %d runtype = %d csno = %d\n", __func__, ret, 48, runtype, csno);
			spin_unlock_irqrestore(&set_spi_gpio_use_state_lock, spinlock_flags);
			return -1;
	    }
		printk("%s: success runtype = %d csno = %d state = %d\n", __func__, runtype, csno, set_spi_gpio_use_state);
	}

	/***************************/
	/* use_state changing 1->0 */
	/***************************/
	else if ((set_spi_gpio_use_state_old != 0) && (set_spi_gpio_use_state == 0)) {
		/* -- start spi setting -- */
	    /* spi_clk */
	    ret = gpio_tlmm_config(GPIO_CFG(45, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	    if (ret) {
			pr_err("%s: Could not configure gpio errno = %x portno = %d runtype = %d csno = %d\n", __func__, ret, 45, runtype, csno);
			spin_unlock_irqrestore(&set_spi_gpio_use_state_lock, spinlock_flags);
			return -1;
	    }
	    /* spi_mosi */
	    ret = gpio_tlmm_config(GPIO_CFG(47, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	    if (ret) {
			pr_err("%s: Could not configure gpio errno = %x portno = %d runtype = %d csno = %d\n", __func__, ret, 47, runtype, csno);
			spin_unlock_irqrestore(&set_spi_gpio_use_state_lock, spinlock_flags);
			return -1;
	    }
	    /* spi_miso */
	    ret = gpio_tlmm_config(GPIO_CFG(48, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	    if (ret) {
			pr_err("%s: Could not configure gpio errno = %x portno = %d runtype = %d csno = %d\n", __func__, ret, 48, runtype, csno);
			spin_unlock_irqrestore(&set_spi_gpio_use_state_lock, spinlock_flags);
			return -1;
	    }
		printk("%s: success runtype = %d csno = %d state = %d\n", __func__, runtype, csno, set_spi_gpio_use_state);
	}

	/***********************************/
	/* use_state changing 0->0 or 1->1 */
	/***********************************/
	else {
		printk("%s: non setting runtype = %d csno = %d state = %d\n", __func__, runtype, csno, set_spi_gpio_use_state);
	}

	spin_unlock_irqrestore(&set_spi_gpio_use_state_lock, spinlock_flags);
	return 0;

}
EXPORT_SYMBOL(set_spi_gpio_exclusive_ctrl);
/* FUJITSU:2011-12-01 spi spio setting end */

/* FUJITSU:2011-12-1 add factory start */
int factory_mode(void)
{
/* FUJITSU:2012-01-13 factory start */
	return factory_value;
#if 0
	return 0;
#endif
/* FUJITSU:2012-01-13 factory end */
}

/* FUJITSU:2012-01-13 factory start */
void factory_read_nvitem_init(void){
	static unsigned char *dataread = NULL;

	unsigned int nv_data = 0;
	int ret = 0;
	unsigned int tmp_id = NV_BOOT_TESTMODE_STATUS_I;

	dataread = (unsigned char *)smem_alloc_vendor1(SMEM_OEM_013);
	if(dataread == NULL){
		printk( "fct flg null.\n");
		factory_value = FACTORY_DEFAULT_VALUE;
	}
	else if(*dataread != 0){
		printk( "nm boot.\n");
		factory_value = FACTORY_DEFAULT_VALUE;
	}
	else if(*dataread == 0){
		printk( "fct boot.\n");

		ret = msm_proc_comm(PCOM_OEM_011, &tmp_id, &nv_data);

		if (ret == -1) {
			printk("NV read error %d\n", ret);
			factory_value = FACTORY_DEFAULT_VALUE;
		}

		//printk("NV_BOOT_TESTMODE_STATUS_I=%d\n", nv_data);

		switch(nv_data){
		case NV_BOOT_NONE:
			factory_value = NV_BOOT_NONE;
			break;
		case NV_BOOT_TESTMODE:
			factory_value = NV_BOOT_TESTMODE;
			break;
		case NV_BOOT_KERNEL_TEST:
			factory_value = NV_BOOT_KERNEL_TEST;
			break;
		case NV_BOOT_ANDROID_TEST:
			factory_value = NV_BOOT_ANDROID_TEST;
			break;
		default:
			factory_value = FACTORY_DEFAULT_VALUE;
		}
	}
}
/* FUJITSU:2012-01-13 factory end */

EXPORT_SYMBOL(factory_mode);
/* FUJITSU:2011-12-1 add factory end */

/* FUJITSU:2011-12-22 add hardware start */
MACHINE_START(F12NAD, "F12NAD")
	.boot_params = PLAT_PHYS_OFFSET + 0x100,
	.map_io = msm7x30_map_io,
	.reserve = msm7x30_reserve,
	.init_irq = msm7x30_init_irq,
	.init_machine = msm7x30_init,
	.timer = &msm_timer,
	.init_early = msm7x30_init_early,
	.handle_irq = vic_handle_irq,
MACHINE_END
/* FUJITSU:2011-12-22 add hardware end */

/* FUJITSU:2011-12-1 del hardware start */
#if 0
MACHINE_START(MSM7X30_SURF, "QCT MSM7X30 SURF")
	.boot_params = PLAT_PHYS_OFFSET + 0x100,
	.map_io = msm7x30_map_io,
	.reserve = msm7x30_reserve,
	.init_irq = msm7x30_init_irq,
	.init_machine = msm7x30_init,
	.timer = &msm_timer,
	.init_early = msm7x30_init_early,
	.handle_irq = vic_handle_irq,
MACHINE_END

MACHINE_START(MSM7X30_FFA, "QCT MSM7X30 FFA")
	.boot_params = PLAT_PHYS_OFFSET + 0x100,
	.map_io = msm7x30_map_io,
	.reserve = msm7x30_reserve,
	.init_irq = msm7x30_init_irq,
	.init_machine = msm7x30_init,
	.timer = &msm_timer,
	.init_early = msm7x30_init_early,
	.handle_irq = vic_handle_irq,
MACHINE_END

MACHINE_START(MSM7X30_FLUID, "QCT MSM7X30 FLUID")
	.boot_params = PLAT_PHYS_OFFSET + 0x100,
	.map_io = msm7x30_map_io,
	.reserve = msm7x30_reserve,
	.init_irq = msm7x30_init_irq,
	.init_machine = msm7x30_init,
	.timer = &msm_timer,
	.init_early = msm7x30_init_early,
	.handle_irq = vic_handle_irq,
MACHINE_END

MACHINE_START(MSM8X55_SURF, "QCT MSM8X55 SURF")
	.boot_params = PHYS_OFFSET + 0x100,
	.map_io = msm7x30_map_io,
	.reserve = msm7x30_reserve,
	.init_irq = msm7x30_init_irq,
	.init_machine = msm7x30_init,
	.timer = &msm_timer,
	.init_early = msm7x30_init_early,
	.handle_irq = vic_handle_irq,
MACHINE_END

MACHINE_START(MSM8X55_FFA, "QCT MSM8X55 FFA")
	.boot_params = PHYS_OFFSET + 0x100,
	.map_io = msm7x30_map_io,
	.reserve = msm7x30_reserve,
	.init_irq = msm7x30_init_irq,
	.init_machine = msm7x30_init,
	.timer = &msm_timer,
	.init_early = msm7x30_init_early,
	.handle_irq = vic_handle_irq,
MACHINE_END
MACHINE_START(MSM8X55_SVLTE_SURF, "QCT MSM8X55 SVLTE SURF")
	.boot_params = PHYS_OFFSET + 0x100,
	.map_io = msm7x30_map_io,
	.reserve = msm7x30_reserve,
	.init_irq = msm7x30_init_irq,
	.init_machine = msm7x30_init,
	.timer = &msm_timer,
	.init_early = msm7x30_init_early,
	.handle_irq = vic_handle_irq,
MACHINE_END
MACHINE_START(MSM8X55_SVLTE_FFA, "QCT MSM8X55 SVLTE FFA")
	.boot_params = PHYS_OFFSET + 0x100,
	.map_io = msm7x30_map_io,
	.reserve = msm7x30_reserve,
	.init_irq = msm7x30_init_irq,
	.init_machine = msm7x30_init,
	.timer = &msm_timer,
	.init_early = msm7x30_init_early,
	.handle_irq = vic_handle_irq,
MACHINE_END
#endif
/* FUJITSU:2011-12-1 del hardware end */
