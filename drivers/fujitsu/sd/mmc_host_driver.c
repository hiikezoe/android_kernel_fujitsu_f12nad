/*
  mmc host Driver

  Copyright (C) 2010 TOSHIBA CORPORATION Mobile Communication Company.

  This program is free software; you can redistribute it and/or
  modify it under the terms of the GNU General Public License
  as published by the Free Software Foundation; either version 2
  of the License, or (at your option) any later version.

  This program is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA
*/
/*----------------------------------------------------------------------------*/
// COPYRIGHT(C) FUJITSU LIMITED 2011-2012
/*----------------------------------------------------------------------------*/
#define THIS_FILE   "mmc_host_driver.c"

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/mfd/pmic8058.h>
/* FUJITSU: 2012-03-09 Extsd-driver add Strat */
#include <linux/pm_runtime.h>
/* FUJITSU: 2012-03-09 Extsd-driver add End   */

#include <mach/gpio.h>
#include <mach/vreg.h>
/* debug */
#include <linux/delay.h>
//#include <linux/io.h>
/* debug */

#define XYZ(fmt, args...)  printk(KERN_INFO THIS_FILE "@%d " fmt "\n", __LINE__, args)

#define PM8058_GPIO_PM_TO_SYS(pm_gpio)     (pm_gpio + NR_GPIO_IRQS)
#define PM8058_GPIO_SYS_TO_PM(sys_gpio)    (sys_gpio - NR_GPIO_IRQS)

#define SD_CFG_POWER_ON   1
#define SD_CFG_POWER_OFF  2
#define SD_RST_ON         3
#define SD_RST_OFF        4
#define SD_VDD_ON         5
#define SD_VDD_OFF        6
#define SDC_CLK_ON        7
#define SDC_CLK_OFF       8

/* FUJITSU:2011-05-24 mmc_host start  */
#define SD_GPIONO_SDC4_CLK   58
#define SD_GPIONO_SDC4_CMD   59
#define SD_GPIONO_SDC4_D0    60
#define SD_GPIONO_SDC4_D1    61
#define SD_GPIONO_SDC4_D2    62
#define SD_GPIONO_SDC4_D3    63
#define SD_GPIONO_SD_RST    168
#if 0 /* FUJITSU:2011/08/04 del for Sky-SDCC start */
#define SD_GPIONO_SD_CLK     98
#endif/* FUJITSU:2011/08/04 del for Sky-SDCC end */
#define SD_GPIONO_SD_INT    169
#define SD_GPIONO_SD_DET     (19 - 1)
#if 1 /* FUJITSU:2011/07/19 mod for Sky-SDCC start */
#define SD_GPIONO_SD_CLK_EN  171
#else
#define SD_GPIONO_SD_CLK_EN  (24 - 1)
#endif/* FUJITSU:2011/07/19 mod for Sky-SDCC end */

/* FUJITSU:2011-06-08 mmc_host start  */
#define PRODUCT_F11SKY_EVM             0x05
#define PRODUCT_F11SKY_EVM_FSL         0x15
#define PRODUCT_F11SKY_1_0             0x00
#define PRODUCT_F11SKY_1_2             0x01
#define PRODUCT_F11SKY_2_0             0x02
#define PRODUCT_F11SKY_2_0_FSL         0x12
#define PRODUCT_F11SKY_2_2             0x03
#define PRODUCT_F11SKY_2_2_FSL         0x13
#define PRODUCT_F11SKY_PR_1            0x04
#define PRODUCT_F11SKY_PR_1_FSL        0x14
#define PRODUCT_F11SKY_CM_1            0x0F
#define PRODUCT_F11SKY_CM_1_FSL        0x1F
#define PRODUCT_F11SKY_CM_2            0x0E
#define PRODUCT_F11SKY_CM_2_FSL        0x1E
#define PRODUCT_F11SKY_CM_3            0x0D
#define PRODUCT_F11SKY_CM_3_FSL        0x1D
/* FUJITSU:2011-06-08 mmc_host end  */

#if 1 /* FUJITSU:2011/07/19 mod for Sky-SDCC start */
static const struct msm_gpio gpio_sdcard_poweron[] = {
#else
static const struct msm_gpio gpio_sdcard_poweron[9] = {
#endif/* FUJITSU:2011/07/19 mod for Sky-SDCC end */
#if 0 /* FUJITSU:2011/08/12 del for Sky-SDCC start */
    { GPIO_CFG( SD_GPIONO_SDC4_CLK , 0, GPIO_CFG_INPUT , GPIO_CFG_NO_PULL, GPIO_CFG_16MA), "sdc4_clk"},
    { GPIO_CFG( SD_GPIONO_SDC4_CMD , 0, GPIO_CFG_INPUT , GPIO_CFG_NO_PULL, GPIO_CFG_6MA) , "sdc4_cmd" },
    { GPIO_CFG( SD_GPIONO_SDC4_D0  , 0, GPIO_CFG_INPUT , GPIO_CFG_NO_PULL, GPIO_CFG_6MA) , "sdc4_dat_0" },
    { GPIO_CFG( SD_GPIONO_SDC4_D1  , 0, GPIO_CFG_INPUT , GPIO_CFG_NO_PULL, GPIO_CFG_6MA) , "sdc4_dat_1" },
    { GPIO_CFG( SD_GPIONO_SDC4_D2  , 0, GPIO_CFG_INPUT , GPIO_CFG_NO_PULL, GPIO_CFG_6MA) , "sdc4_dat_2" },
    { GPIO_CFG( SD_GPIONO_SDC4_D3  , 0, GPIO_CFG_INPUT , GPIO_CFG_NO_PULL, GPIO_CFG_6MA) , "sdc4_dat_3" },
#endif/* FUJITSU:2011/08/12 del for Sky-SDCC end */
    { GPIO_CFG( SD_GPIONO_SD_RST   , 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA) , "SD_RST" },          /* 1=NG */
#if 0 /* FUJITSU:2011/08/04 del for Sky-SDCC start */
    { GPIO_CFG( SD_GPIONO_SD_CLK   , 2, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA) , "gp_clk" },
#endif/* FUJITSU:2011/08/04 del for Sky-SDCC end */
    { GPIO_CFG( SD_GPIONO_SD_INT   , 0, GPIO_CFG_INPUT , GPIO_CFG_NO_PULL, GPIO_CFG_2MA) , "SD_INT" },
/* FUJITSU:2011/07/19 add for Sky-SDCC start */
    { GPIO_CFG( SD_GPIONO_SD_CLK_EN, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA) , "gp_clk_en" },
/* FUJITSU:2011/07/19 add for Sky-SDCC start */
/*    { GPIO_CFG( 88, 1, GPIO_CFG_OUTPUT , GPIO_CFG_NO_PULL, GPIO_CFG_4MA), "SD_CS" }, */
/*    { GPIO_CFG(172, 2, GPIO_CFG_OUTPUT , GPIO_CFG_KEEPER, GPIO_CFG_4MA), "EBI2_ADR_OUT6" }, */
/*    { GPIO_CFG(173, 2, GPIO_CFG_OUTPUT , GPIO_CFG_KEEPER, GPIO_CFG_4MA), "EBI2_ADR_OUT5" }, */
/*    { GPIO_CFG(174, 2, GPIO_CFG_OUTPUT , GPIO_CFG_KEEPER, GPIO_CFG_4MA), "EBI2_ADR_OUT4" }, */
/*    { GPIO_CFG(175, 2, GPIO_CFG_OUTPUT , GPIO_CFG_KEEPER, GPIO_CFG_4MA), "EBI2_ADR_OUT3" }, */
/*    { GPIO_CFG(176, 2, GPIO_CFG_OUTPUT , GPIO_CFG_KEEPER, GPIO_CFG_4MA), "EBI2_ADR_OUT2" }, */
/*    { GPIO_CFG(177, 2, GPIO_CFG_OUTPUT , GPIO_CFG_KEEPER, GPIO_CFG_4MA), "EBI2_ADR_OUT1" }, */
/*    { GPIO_CFG(178, 2, GPIO_CFG_OUTPUT , GPIO_CFG_KEEPER, GPIO_CFG_4MA), "EBI2_ADR_OUT0" }, */
};

/* SD card power off */
#if 1 /* FUJITSU:2011/07/19 mod for Sky-SDCC start */
static const struct msm_gpio gpio_sdcard_poweroff[] = {
#else
static const struct msm_gpio gpio_sdcard_poweroff[9] = {
#endif/* FUJITSU:2011/07/19 mod for Sky-SDCC end */
#if 0 /* FUJITSU:2011/08/12 del for Sky-SDCC start */
    { GPIO_CFG( SD_GPIONO_SDC4_CLK , 0, GPIO_CFG_INPUT , GPIO_CFG_PULL_DOWN, GPIO_CFG_6MA), "SDC4_CLK" },
    { GPIO_CFG( SD_GPIONO_SDC4_CMD , 0, GPIO_CFG_INPUT , GPIO_CFG_PULL_DOWN, GPIO_CFG_6MA), "SDC4_CMD" },
    { GPIO_CFG( SD_GPIONO_SDC4_D0  , 0, GPIO_CFG_INPUT , GPIO_CFG_PULL_DOWN, GPIO_CFG_6MA), "SDC4_D0" },
    { GPIO_CFG( SD_GPIONO_SDC4_D1  , 0, GPIO_CFG_INPUT , GPIO_CFG_PULL_DOWN, GPIO_CFG_6MA), "SDC4_D1" },
    { GPIO_CFG( SD_GPIONO_SDC4_D2  , 0, GPIO_CFG_INPUT , GPIO_CFG_PULL_DOWN, GPIO_CFG_6MA), "SDC4_D2" },
    { GPIO_CFG( SD_GPIONO_SDC4_D3  , 0, GPIO_CFG_INPUT , GPIO_CFG_PULL_DOWN, GPIO_CFG_6MA), "SDC4_D3" },
#endif/* FUJITSU:2011/08/12 del for Sky-SDCC end */
    { GPIO_CFG( SD_GPIONO_SD_RST   , 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "SD_RST" },
#if 0 /* FUJITSU:2011/08/04 del for Sky-SDCC start */
    { GPIO_CFG( SD_GPIONO_SD_CLK   , 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA), "gp_clk" },
#endif/* FUJITSU:2011/08/04 del for Sky-SDCC end */
    { GPIO_CFG( SD_GPIONO_SD_INT   , 0, GPIO_CFG_INPUT , GPIO_CFG_NO_PULL, GPIO_CFG_2MA), "SD_INT" },
/* FUJITSU:2011/07/19 add for Sky-SDCC start */
    { GPIO_CFG( SD_GPIONO_SD_CLK_EN, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA) , "gp_clk_en" },
/* FUJITSU:2011/07/19 add for Sky-SDCC end */
};
/* FUJITSU:2011-05-24 mmc_host end  */

static int mmc_host_sysfs_create_group(struct kobject *kobj, const struct attribute_group *grp)
{
    return sysfs_create_group(kobj, grp);
}

static void mmc_host_sysfs_remove_group(struct kobject *kobj, const struct attribute_group *grp)
{
    sysfs_remove_group(kobj, grp);
}

static int __init mmc_host_init(void)
{
    return 0;
}

static void __exit mmc_host_exit(void)
{
    ;
}

static int mmc_host_driver_register(struct platform_driver *skel_driver)
{
    return platform_driver_register(skel_driver);
}

static void mmc_host_driver_unregister(struct platform_driver *skel_driver)
{
    platform_driver_unregister(skel_driver);
}
/* FUJITSU: 2012-03-09 Extsd-driver add Strat */
static void mmc_host_runtime_put_noidle(struct device *dev)
{
	pm_runtime_put_noidle(dev);
}
static void mmc_host_runtime_get_noresume(struct device *dev)
{
	pm_runtime_get_noresume(dev);
}
static int mmc_host_schedule_suspend(struct device *dev, unsigned int delay)
{
	return pm_schedule_suspend(dev, delay);
}
static bool mmc_host_runtime_suspended(struct device *dev)
{
	return pm_runtime_suspended(dev);
}
static int mmc_host_runtime_get_sync(struct device *dev)
{
	return pm_runtime_get_sync(dev);
}
static int mmc_host_runtime_put_sync(struct device *dev)
{
	return pm_runtime_put_sync(dev);
}
static int mmc_host_runtime_set_active(struct device *dev)
{
	return pm_runtime_set_active(dev);
}
static void mmc_host_suspend_ignore_children(struct device *dev, bool enable)
{
	pm_suspend_ignore_children(dev, enable);
}
static void mmc_host_runtime_enable(struct device *dev)
{
	pm_runtime_enable(dev);
}


static int mmc_host_runtime_suspend(struct device *dev)
{
	return pm_runtime_suspend(dev);
}
	
static int mmc_host_request_resume(struct device *dev)
{
	return pm_request_resume(dev);
}
	
	
static unsigned int mmc_host_work_busy(struct work_struct *work)
{
	return work_busy(work);
}
/* FUJITSU: 2012-03-09 Extsd-driver add End */

static int mmc_host_dev_model_get_model_sd(void)
{
    return 1/*dev_model_get_model_sd()*/;
}

struct vreg *vreg_sdc;
//iwa
#if 0
struct pm8058_gpio sdc4_en = {
    .direction      = PM_GPIO_DIR_OUT,
    .pull           = PM_GPIO_PULL_NO,
    .vin_sel        = PM_GPIO_VIN_L5,
    .function       = PM_GPIO_FUNC_NORMAL,
    .inv_int_pol    = 0,
    .out_strength   = PM_GPIO_STRENGTH_LOW,
    .output_value   = 0,
};
#endif
//iwa
/* FUJITSU:2011-09-28 SD I/O Unique Que START */
static int mmc_host_queue_work(struct workqueue_struct * wq,struct work_struct *w)
{
	return queue_work(wq, w);
}
static struct workqueue_struct* mmc_host_create_singlethread_workqueue(const char* name)
{
	return create_singlethread_workqueue(name);
}
void mmc_host_destroy_workqueue(struct workqueue_struct *wq)
{
	destroy_workqueue(wq);
}
/* FUJITSU:2011-09-28 SD I/O Unique Que END */

static int mmc_host_io_ctrl(int cmd)
{
	int rc;

    switch (cmd){
    case SD_CFG_POWER_ON:
/* FUJITSU:2011-05-24 mmc_host start  */
/* FUJITSU:2011-06-08 mmc_host start  */
        rc = msm_gpios_enable(gpio_sdcard_poweron, ARRAY_SIZE(gpio_sdcard_poweron));
        if (rc >= 0){
/* FUJITSU:2011-06-08 mmc_host end  */

            gpio_set_value(SD_GPIONO_SD_RST, 0);    /* SD_RST(GPIO[168])   -> 'L' */

#if 0 /* FUJITSU:2011/07/19 del for Sky-SDCC start */
	        rc = pm8058_gpio_config(SD_GPIONO_SD_CLK_EN, &sdc4_en);
	        if (rc) {
	            pr_err("%s SD_GPIONO_SD_CLK_EN config failed\n",
	                                 __func__);
	        }
	        gpio_set_value_cansleep(
	            PM8058_GPIO_PM_TO_SYS(SD_GPIONO_SD_CLK_EN), 0);
#endif/* FUJITSU:2011/07/19 mod for Sky-SDCC end */

        }else{
            pr_err("%s: GPIO_POWER_ON failed\n",
                                 __func__);
            return -1;
        }
/* FUJITSU:2011-05-24 mmc_host end  */
        break;

    case SD_CFG_POWER_OFF:
/* FUJITSU:2011-05-24 mmc_host start  */
/* FUJITSU:2011-06-08 mmc_host start  */
        rc = msm_gpios_enable(gpio_sdcard_poweroff, ARRAY_SIZE(gpio_sdcard_poweroff));
        if (rc >= 0){
/* FUJITSU:2011-06-08 mmc_host end  */
            gpio_set_value(SD_GPIONO_SD_RST, 0);    /* SD_RST(GPIO[168])   -> 'L' */
#if 0 /* FUJITSU:2011/08/04 del for Sky-SDCC start */
            gpio_set_value(SD_GPIONO_SD_CLK, 0);    /* SD_CLK(GPIO[98])    -> 'L' */
#endif/* FUJITSU:2011/08/04 del for Sky-SDCC end */

#if 0 /* FUJITSU:2011/07/19 del for Sky-SDCC start */
	        rc = pm8058_gpio_config(SD_GPIONO_SD_CLK_EN, &sdc4_en);
	        if (rc) {
	            pr_err("%s SD_GPIONO_SD_CLK_EN config failed\n",
	                                 __func__);
	        }
	        gpio_set_value_cansleep(
	            PM8058_GPIO_PM_TO_SYS(SD_GPIONO_SD_CLK_EN), 0);
#endif/* FUJITSU:2011/07/19 del for Sky-SDCC end */
        
        }else{
            printk(KERN_ERR "%s: GPIO_POWER_OFF\n",
                   __func__);
            return -1;
        }
/* FUJITSU:2011-05-24 mmc_host end  */

        break;

    case SD_RST_ON:
        XYZ("SD_RST is '%c' +", gpio_get_value(SD_GPIONO_SD_RST) ? 'H' : 'L');
/* FUJITSU:2011-05-24 mmc_host start */
        gpio_set_value(SD_GPIONO_SD_RST, 1);
/* FUJITSU:2011-05-24 mmc_host end  */
        XYZ("SD_RST is '%c' +", gpio_get_value(SD_GPIONO_SD_RST) ? 'H' : 'L');
        break;
#if 0
    case SD_RST_OFF:
        XYZ("SD_RST is '%c' +", gpio_get_value(SD_GPIONO_SD_RST) ? 'H' : 'L');
        gpio_set_value(SD_GPIONO_SD_RST, 0);
        XYZ("SD_RST is '%c' +", gpio_get_value(SD_GPIONO_SD_RST) ? 'H' : 'L');
        break;
#endif

    case SD_VDD_ON:
/* FUJITSU:2011-05-24 mmc_host start  */
        vreg_sdc = vreg_get(NULL, "gp6"/*"mmc"*/);
        if (IS_ERR(vreg_sdc)) {
            printk(KERN_ERR "%s: vreg get failed (%ld)\n",
                   __func__, PTR_ERR(vreg_sdc));
            break;
        }
        rc = vreg_set_level(vreg_sdc, 2900);
        if (rc)
            printk(KERN_ERR "%s: vreg_set_level() = %d \n",	__func__, rc);
        rc = vreg_enable(vreg_sdc);
        if (rc)
            printk(KERN_ERR "%s: vreg_enable() = %d \n",	__func__, rc);
/* FUJITSU:2011-05-24 mmc_host end  */
        break;

    case SD_VDD_OFF:
/* FUJITSU:2011-05-24 mmc_host start  */
        vreg_sdc = vreg_get(NULL, "gp6"/*"mmc"*/);
        if (IS_ERR(vreg_sdc)) {
            printk(KERN_ERR "%s: vreg get failed (%ld)\n",
                   __func__, PTR_ERR(vreg_sdc));
            break;
        }
        vreg_disable(vreg_sdc);
/* FUJITSU:2011-05-24 mmc_host end  */
        break;

    case SDC_CLK_ON:
#if 1 /* FUJITSU:2011/07/19 mod for Sky-SDCC start */
        XYZ("SD_CLK_EN is '%c' +", gpio_get_value(SD_GPIONO_SD_CLK_EN) ? 'H' : 'L');
        gpio_set_value(SD_GPIONO_SD_CLK_EN, 1);
        XYZ("SD_CLK_EN is '%c' +", gpio_get_value(SD_GPIONO_SD_CLK_EN) ? 'H' : 'L');
#else
        XYZ("SD_CLK_EN is '%c' +", gpio_get_value_cansleep(PM8058_GPIO_PM_TO_SYS(SD_GPIONO_SD_CLK_EN)) ? 'H' : 'L');
/* FUJITSU:2011-05-24 mmc_host start */
//        gpio_set_value(SD_GPIONO_SD_CLK_EN, 1);
//        gpio_set_value_cansleep(PM8058_GPIO_PM_TO_SYS(SD_GPIONO_SD_CLK_EN), 1);
        if( ((system_rev & 0x0f) >= PRODUCT_F11SKY_2_0) && ((system_rev & 0x0f) != PRODUCT_F11SKY_EVM) ) {
        	XYZ("SD_CLK_ON is set L MODEL = %Xh ",system_rev);
            gpio_set_value_cansleep(PM8058_GPIO_PM_TO_SYS(SD_GPIONO_SD_CLK_EN), 0);
        }else{
        	XYZ("SD_CLK_ON is set H MODEL = %Xh ",system_rev);
            gpio_set_value_cansleep(PM8058_GPIO_PM_TO_SYS(SD_GPIONO_SD_CLK_EN), 1);
        }
/* FUJITSU:2011-05-24 mmc_host end  */
        XYZ("SD_CLK_EN is '%c' +", gpio_get_value_cansleep(PM8058_GPIO_PM_TO_SYS(SD_GPIONO_SD_CLK_EN)) ? 'H' : 'L');
#endif/* FUJITSU:2011/07/19 mod for Sky-SDCC end */
        break;

    case SDC_CLK_OFF:
#if 1 /* FUJITSU:2011/07/19 mod for Sky-SDCC start */
        XYZ("SD_CLK_EN is '%c' +", gpio_get_value(SD_GPIONO_SD_CLK_EN) ? 'H' : 'L');
        gpio_set_value(SD_GPIONO_SD_CLK_EN, 0);
        XYZ("SD_CLK_EN is '%c' +", gpio_get_value(SD_GPIONO_SD_CLK_EN) ? 'H' : 'L');
#else
        XYZ("SD_CLK_EN is '%c' +", gpio_get_value_cansleep(PM8058_GPIO_PM_TO_SYS(SD_GPIONO_SD_CLK_EN)) ? 'H' : 'L');
/* FUJITSU:2011-05-24 mmc_host start  */
//        gpio_set_value(SD_GPIONO_SD_CLK_EN, 0);
        gpio_set_value_cansleep(PM8058_GPIO_PM_TO_SYS(SD_GPIONO_SD_CLK_EN), 0);
/* FUJITSU:2011-05-24 mmc_host end  */
        XYZ("SD_CLK_EN is '%c' +", gpio_get_value_cansleep(PM8058_GPIO_PM_TO_SYS(SD_GPIONO_SD_CLK_EN)) ? 'H' : 'L');
#endif/* FUJITSU:2011/07/19 mod for Sky-SDCC end */
        break;

    default:
        return -1;
    }
    return 0;
}

EXPORT_SYMBOL(mmc_host_sysfs_create_group);
EXPORT_SYMBOL(mmc_host_sysfs_remove_group);
EXPORT_SYMBOL(mmc_host_driver_register);
EXPORT_SYMBOL(mmc_host_driver_unregister);
/* FUJITSU: 2012-03-09 Extsd-driver add Strat */
EXPORT_SYMBOL(mmc_host_runtime_put_noidle);
EXPORT_SYMBOL(mmc_host_runtime_get_noresume);
EXPORT_SYMBOL(mmc_host_schedule_suspend);
EXPORT_SYMBOL(mmc_host_runtime_suspended);
EXPORT_SYMBOL(mmc_host_runtime_get_sync);
EXPORT_SYMBOL(mmc_host_runtime_put_sync);
EXPORT_SYMBOL(mmc_host_runtime_set_active);
EXPORT_SYMBOL(mmc_host_suspend_ignore_children);
EXPORT_SYMBOL(mmc_host_runtime_enable);

EXPORT_SYMBOL(mmc_host_request_resume);
EXPORT_SYMBOL(mmc_host_runtime_suspend);

EXPORT_SYMBOL(mmc_host_work_busy);
/* FUJITSU: 2012-03-09 Extsd-driver add End */
EXPORT_SYMBOL(mmc_host_dev_model_get_model_sd);

//EXPORT_SYMBOL(mmc_host_dev_set_value);
EXPORT_SYMBOL(mmc_host_io_ctrl);
/* FUJITSU:2011-09-28 SD I/O Unique Que START */
EXPORT_SYMBOL(mmc_host_queue_work);
EXPORT_SYMBOL(mmc_host_create_singlethread_workqueue);
EXPORT_SYMBOL(mmc_host_destroy_workqueue);
/* FUJITSU:2011-09-28 SD I/O Unique Que END */

module_init(mmc_host_init);
module_exit(mmc_host_exit);

MODULE_DESCRIPTION("mmc host driver interface");
MODULE_LICENSE("GPL");

