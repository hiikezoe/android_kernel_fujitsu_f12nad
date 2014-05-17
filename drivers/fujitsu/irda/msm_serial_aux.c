/*
 * Copyright(C) 2011-2012 FUJITSU LIMITED
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */


#include <linux/slab.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/ioport.h>
#include <asm/io.h>
#include <linux/platform_device.h>
#include <linux/mfd/pmic8058.h>
#include <linux/gpio.h>
#include <mach/gpio.h>
#include "msm_serial_aux.h"
/* FUJITSU:2011-12-01 add Irda start */
#include <linux/mfd/pm8xxx/gpio.h>
#include <linux/mfd/pm8xxx/misc.h>
/* FUJITSU:2011-12-01 add Irda end */
#define DEV_NAME "MSM_UART_AUX"

// PMIC GPIO PORT
/* FUJITSU:2011-12-01 del Irda start */
#if 0
#define PM8058_UART_TX2_22			(22-1)
#define PM8058_IRDA_PWD_26			(26-1)
#define PM8058_UART_RX2_34			(34-1)
#define PM8058_UART_PMIC_TXD_36		(36-1)
#define PM8058_UART_PMIC_RXD_37		(37-1)
#endif
/* FUJITSU:2011-12-01 del Irda end */
#define PM8058_GPIO_PM_TO_SYS(pm_gpio)	(pm_gpio + NR_GPIO_IRQS)
/* FUJITSU:2011-12-01 add Irda start */
#define PM8058_UART_TX2_22			PM8058_GPIO_PM_TO_SYS(22-1)
#define PM8058_IRDA_PWD_26			PM8058_GPIO_PM_TO_SYS(26-1)
#define PM8058_UART_RX2_34			PM8058_GPIO_PM_TO_SYS(34-1)
#define PM8058_UART_PMIC_TXD_36		PM8058_GPIO_PM_TO_SYS(36-1)
#define PM8058_UART_PMIC_RXD_37		PM8058_GPIO_PM_TO_SYS(37-1)
/* FUJITSU:2011-12-01 add Irda end */

// IrDA POWER On/Off ------------------------------------------
// PMIC GPIO 26
/* FUJITSU:2011-12-01 mod Irda start */
#if 0
struct pm8058_gpio irda_pwdown_26 = {
#else
struct pm_gpio irda_pwdown_26 = {
#endif
/* FUJITSU:2011-12-01 mod Irda end */
 .direction      = PM_GPIO_DIR_OUT,
 .output_buffer  = PM_GPIO_OUT_BUF_CMOS,
 .pull           = PM_GPIO_PULL_NO,
/* FUJITSU:2011-12-01 mod Irda start */
#if 0
 .vin_sel        = PM_GPIO_VIN_L5,
#else
 .vin_sel        = PM8058_GPIO_VIN_L5,
#endif
/* FUJITSU:2011-12-01 mod Irda end */
 .function       = PM_GPIO_FUNC_NORMAL,
 .inv_int_pol    = 0,
 .out_strength   = PM_GPIO_STRENGTH_HIGH,
 .output_value   = 0,
};

// UART Switch ------------------------------------------------
// PMIC GPIO 22
/* FUJITSU:2011-12-01 mod Irda start */
#if 0
struct pm8058_gpio uart_tx2_en_22 = {
#else
struct pm_gpio uart_tx2_en_22 = {
#endif
/* FUJITSU:2011-12-01 mod Irda end */
 .direction      = PM_GPIO_DIR_OUT,
 .pull           = PM_GPIO_PULL_NO,
/* FUJITSU:2011-12-01 mod Irda start */
#if 0
 .vin_sel        = PM_GPIO_VIN_L5,
#else
 .vin_sel        = PM8058_GPIO_VIN_L5,
#endif
/* FUJITSU:2011-12-01 mod Irda end */
 .function       = PM_GPIO_FUNC_2,
 .inv_int_pol    = 0,
 .out_strength   = PM_GPIO_STRENGTH_LOW,
 .output_value   = 0,
};

// PMIC GPIO 34
/* FUJITSU:2011-12-01 mod Irda start */
#if 0
struct pm8058_gpio uart_rx2_en_34 = {
#else
struct pm_gpio uart_rx2_en_34 = {
#endif
/* FUJITSU:2011-12-01 mod Irda end */
 .direction      = PM_GPIO_DIR_IN,
 .pull           = PM_GPIO_PULL_NO,
/* FUJITSU:2011-12-01 mod Irda start */
#if 0
 .vin_sel        = PM_GPIO_VIN_L5,
#else
 .vin_sel        = PM8058_GPIO_VIN_L5,
#endif
/* FUJITSU:2011-12-01 mod Irda end */
 .function       = PM_GPIO_FUNC_1,
 .inv_int_pol    = 0,
 .out_strength   = PM_GPIO_STRENGTH_LOW,
 .output_value   = 0,
};

// MSM ---------------------------------------------------------
// PMIC GPIO 36
/* FUJITSU:2011-12-01 mod Irda start */
#if 0
struct pm8058_gpio uart_pmic_txd_en_36 = {
#else
struct pm_gpio uart_pmic_txd_en_36 = {
#endif
/* FUJITSU:2011-12-01 mod Irda end */
 .direction      = PM_GPIO_DIR_IN,
 .pull           = PM_GPIO_PULL_NO,
/* FUJITSU:2011-12-01 mod Irda start */
#if 0
 .vin_sel        = PM_GPIO_VIN_S3,
#else
 .vin_sel        = PM8058_GPIO_VIN_S3,
#endif
/* FUJITSU:2011-12-01 mod Irda end */
 .function       = PM_GPIO_FUNC_1,
 .inv_int_pol    = 0,
 .out_strength   = PM_GPIO_STRENGTH_MED,
 .output_value   = 0,
};

// PMIC GPIO 37
/* FUJITSU:2011-12-01 mod Irda start */
#if 0
struct pm8058_gpio uart_pmic_rxd_en_37 = {
#else
struct pm_gpio uart_pmic_rxd_en_37 = {
#endif
/* FUJITSU:2011-12-01 mod Irda end */
 .direction      = PM_GPIO_DIR_OUT,
 .pull           = PM_GPIO_PULL_NO,
/* FUJITSU:2011-12-01 mod Irda start */
#if 0
 .vin_sel        = PM_GPIO_VIN_S3,
#else
 .vin_sel        = PM8058_GPIO_VIN_S3,
#endif
/* FUJITSU:2011-12-01 mod Irda end */
 .function       = PM_GPIO_FUNC_2,
 .inv_int_pol    = 0,
 .out_strength   = PM_GPIO_STRENGTH_HIGH,
 .output_value   = 0,
};

static int driver_major_no = 0;
module_param( driver_major_no, int, 0 );

static struct cdev char_dev;
static unsigned long uart_membase;
static struct class* irda_class;
static struct device *irda_device;

static int msm_uart_aux_chg_irda( void )
{
	int rc = 0;

	// TX
	uart_tx2_en_22.function = PM_GPIO_FUNC_2;
/* FUJITSU:2011-10-21 gpio chg start  */
#if defined(CONFIG_MACH_F11APO) || defined(CONFIG_MACH_F12APON)
	if((system_rev != 0x08) || (system_rev != 0x0F))
	{
		uart_tx2_en_22.out_strength = PM_GPIO_STRENGTH_MED;
	}
#elif defined(CONFIG_MACH_FJI12)
	if((system_rev != 0x00) || (system_rev != 0x07))
	{
		uart_tx2_en_22.out_strength = PM_GPIO_STRENGTH_MED;
	}
#endif
/* FUJITSU:2011-10-21 gpio chg end  */
/* FUJITSU:2011-12-01 mod irda start  */
#if 0
	rc = pm8058_gpio_config( PM8058_UART_TX2_22, &uart_tx2_en_22 );
#else
	rc = pm8xxx_gpio_config( PM8058_UART_TX2_22, &uart_tx2_en_22 );
#endif
/* FUJITSU:2011-12-01 mod irda end  */
	if ( rc )
	{
		printk( KERN_ERR "%s: PM8058_UART_TX config failed\n", __func__ );
		return -EIO;
	}
	// RX
/* FUJITSU:2011-09-28 gpio chg start  */
#if defined(CONFIG_MACH_F11APO) || defined(CONFIG_MACH_F12APON)
	if((system_rev != 0x08) || (system_rev != 0x0F))
	{
		uart_rx2_en_34.pull = PM_GPIO_PULL_UP_1P5;
	}
#elif defined(CONFIG_MACH_FJI12)
	if((system_rev != 0x00) || (system_rev != 0x07))
	{
		uart_rx2_en_34.pull = PM_GPIO_PULL_UP_1P5;
	}
#endif
/* FUJITSU:2011-09-28 gpio chg end  */
/* FUJITSU:2011-12-01 mod irda start  */
#if 0
	rc = pm8058_gpio_config( PM8058_UART_RX2_34, &uart_rx2_en_34 );
#else
	rc = pm8xxx_gpio_config( PM8058_UART_RX2_34, &uart_rx2_en_34 );
#endif
/* FUJITSU:2011-12-01 mod irda end  */
	if ( rc )
	{
		printk( KERN_ERR "%s: PM8058_UART_RX config failed\n", __func__ );
		return -EIO;
	}
	// PMIC
/* FUJITSU:2011-12-01 mod irda start */
#if 0
	rc = pm8058_misc_control( NULL, PM8058_UART_MUX_MASK, PM8058_UART_MUX_2 );
#else
	rc = pm8xxx_uart_gpio_mux_ctrl(UART_TX2_RX2);
#endif
/* FUJITSU:2011-12-01 mod irda end  */
	if ( rc )
	{
/* FUJITSU:2011-12-01 mod irda start */
#if 0
		printk( KERN_ERR "%s:pmM8058_misc_control failed=%d\n", __func__, rc );
#else
		printk( KERN_ERR "%s:pm8xxx_uart_gpio_mux_ctrl failed=%d\n", __func__, rc );
#endif
/* FUJITSU:2011-12-01 mod irda end  */
	}
	printk( KERN_INFO DEV_NAME " %s: UART Change -> IRDA\n", __func__ );
	// TXD
/* FUJITSU:2011-09-28 gpio chg start  */
#if defined(CONFIG_MACH_F11APO) || defined(CONFIG_MACH_F12APON)
	if((system_rev != 0x08) || (system_rev != 0x0F))
	{
		uart_pmic_txd_en_36.pull = PM_GPIO_PULL_UP_1P5;
	}
#elif(defined CONFIG_MACH_FJI12)
	if((system_rev != 0x00) || (system_rev != 0x07))
	{
		uart_pmic_txd_en_36.pull = PM_GPIO_PULL_UP_1P5;
	}
#endif
/* FUJITSU:2011-09-28 gpio chg end  */
/* FUJITSU:2011-12-01 mod irda start  */
#if 0
	rc = pm8058_gpio_config( PM8058_UART_PMIC_TXD_36, &uart_pmic_txd_en_36 );
#else
	rc = pm8xxx_gpio_config( PM8058_UART_PMIC_TXD_36, &uart_pmic_txd_en_36 );
#endif
/* FUJITSU:2011-12-01 mod irda end  */
	if ( rc )
	{
		printk( KERN_ERR "%s: PM8058_UART_PMIC_TXD config failed\n", __func__ );
		return -EIO;
	}
	// RXD
/* FUJITSU:2011-09-28 gpio chg start  */
#if defined(CONFIG_MACH_F11APO) || defined(CONFIG_MACH_F12APON)
	if((system_rev != 0x08) || (system_rev != 0x0F))
	{
		uart_pmic_rxd_en_37.out_strength = PM_GPIO_STRENGTH_MED;
		uart_pmic_rxd_en_37.pull = PM_GPIO_PULL_UP_1P5;
	}
#elif defined(CONFIG_MACH_FJI12)
	if((system_rev != 0x00) || (system_rev != 0x07))
	{
		uart_pmic_rxd_en_37.out_strength = PM_GPIO_STRENGTH_MED;
		uart_pmic_rxd_en_37.pull = PM_GPIO_PULL_UP_1P5;
	}
#endif
/* FUJITSU:2011-09-28 gpio chg end  */
/* FUJITSU:2011-12-01 mod irda start  */
#if 0
	rc = pm8058_gpio_config( PM8058_UART_PMIC_RXD_37, &uart_pmic_rxd_en_37 );
#else
	rc = pm8xxx_gpio_config( PM8058_UART_PMIC_RXD_37, &uart_pmic_rxd_en_37 );
#endif
/* FUJITSU:2011-12-01 mod irda end  */
	if ( rc )
	{
		printk( KERN_ERR "%s: PM8058_UART_PMIC_RXD config failed\n", __func__ );
		return -EIO;
	}

	// MSM
	rc = gpio_tlmm_config( GPIO_CFG( 51, 1, GPIO_CFG_INPUT,  GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE );
	if ( rc )
	{
		printk( KERN_ERR "%s: MSM8655_UART_RXD_A config failed\n", __func__ );
		return -EIO;
	}
/* FUJITSU:2011-09-28 gpio chg start  */
#if defined(CONFIG_MACH_F11APO) || defined(CONFIG_MACH_F12APON)
	if((system_rev != 0x08) || (system_rev != 0x0F))
	{
		rc = gpio_tlmm_config( GPIO_CFG( 52, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE );
	}else{
		rc = gpio_tlmm_config( GPIO_CFG( 52, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE );
	}
#elif defined(CONFIG_MACH_FJI12)
	if((system_rev != 0x00) || (system_rev != 0x07))
	{
		rc = gpio_tlmm_config( GPIO_CFG( 52, 1, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE );
	}else{
		rc = gpio_tlmm_config( GPIO_CFG( 52, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE );
	}
#else
	rc = gpio_tlmm_config( GPIO_CFG( 52, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE );
#endif
/* FUJITSU:2011-09-28 gpio chg end  */
	if ( rc )
	{
		printk( KERN_ERR "%s: MSM8655_UART_TXD_A config failed\n", __func__ );
		return -EIO;
	}

	// SD
/* FUJITSU:2011-09-28 gpio chg start  */
#if defined(CONFIG_MACH_F11APO) || defined(CONFIG_MACH_F12APON)
	if((system_rev != 0x08) || (system_rev != 0x0F))
	{
		irda_pwdown_26.output_buffer = PM_GPIO_OUT_BUF_OPEN_DRAIN;
		irda_pwdown_26.out_strength = PM_GPIO_STRENGTH_NO;
	}
#elif defined(CONFIG_MACH_FJI12)
	if((system_rev != 0x00) || (system_rev != 0x07))
	{
		irda_pwdown_26.output_buffer = PM_GPIO_OUT_BUF_OPEN_DRAIN;
		irda_pwdown_26.out_strength = PM_GPIO_STRENGTH_NO;
	}
#endif
/* FUJITSU:2011-09-28 gpio chg end  */
/* FUJITSU:2011-12-01 mod irda start  */
#if 0
	rc = pm8058_gpio_config( PM8058_IRDA_PWD_26, &irda_pwdown_26 );
#else
	rc = pm8xxx_gpio_config( PM8058_IRDA_PWD_26, &irda_pwdown_26 );
#endif
/* FUJITSU:2011-12-01 mod irda end  */
	if( rc < 0 )
	{
		printk( KERN_ERR "%s: PM8058_IRDA_PWD_26 config failed[%d]\n", __func__, rc );
		return -EIO;
	}
	printk( KERN_INFO DEV_NAME " %s: IRDA GPIO Config Complete\n", __func__ );

	return 0;
}

static int msm_uart_aux_chg_uart( void )
{
	int rc = 0;

	// TX -> Low
	uart_tx2_en_22.function = PM_GPIO_FUNC_NORMAL;
/* FUJITSU:2011-10-21 gpio chg start  */
#if defined(CONFIG_MACH_F11APO) || defined(CONFIG_MACH_F12APON)
	if((system_rev != 0x08) || (system_rev != 0x0F))
	{
		uart_tx2_en_22.out_strength = PM_GPIO_STRENGTH_MED;
	}
#elif defined(CONFIG_MACH_FJI12)
	if((system_rev != 0x00) || (system_rev != 0x07))
	{
		uart_tx2_en_22.out_strength = PM_GPIO_STRENGTH_MED;
	}
#endif
/* FUJITSU:2011-10-21 gpio chg end  */
/* FUJITSU:2011-12-01 mod irda start  */
#if 0
	rc = pm8058_gpio_config( PM8058_UART_TX2_22, &uart_tx2_en_22 );
#else
	rc = pm8xxx_gpio_config( PM8058_UART_TX2_22, &uart_tx2_en_22 );
#endif
/* FUJITSU:2011-12-01 mod irda end  */
	if ( rc )
	{
		printk( KERN_ERR "%s: PM8058_UART_TX config failed\n", __func__ );
		return -EIO;
	}
/* FUJITSU:2011-12-01 mod irda start  */
#if 0
	gpio_set_value_cansleep( PM8058_GPIO_PM_TO_SYS( PM8058_UART_TX2_22 ), 0 );
#else
	gpio_set_value_cansleep( PM8058_UART_TX2_22, 0 );
#endif
/* FUJITSU:2011-12-01 mod irda end  */
	printk( KERN_INFO DEV_NAME " %s: IRDA GPIO TX -> Low\n", __func__ );
	return 0;
}

static int msm_uart_aux_irda_start( void )
{
	//SD(GPIO_26): Low
/* FUJITSU:2011-12-01 mod irda start  */
#if 0
	gpio_set_value_cansleep( PM8058_GPIO_PM_TO_SYS( PM8058_IRDA_PWD_26 ), 0 );
#else
	gpio_set_value_cansleep( PM8058_IRDA_PWD_26, 0 );
#endif
/* FUJITSU:2011-12-01 mod irda end  */
	printk( KERN_INFO DEV_NAME " %s: Start IRDA Mode SD -> Low\n", __func__ );

	return 0;
}

static int msm_uart_aux_irda_stop( void )
{
	//SD(GPIO_26): High
/* FUJITSU:2011-12-01 mod irda start  */
#if 0
	gpio_set_value_cansleep( PM8058_GPIO_PM_TO_SYS( PM8058_IRDA_PWD_26 ), 1 );
#else
	gpio_set_value_cansleep( PM8058_IRDA_PWD_26, 1 );
#endif
/* FUJITSU:2011-12-01 mod irda end  */
	printk( KERN_INFO DEV_NAME " %s: IRDA GPIO SD -> High\n", __func__ );

	return 0;
}

/* 
 * open() method:
 */
static int msm_uart_aux_open(struct inode *inode, struct file *filp)
{
	pr_debug( DEV_NAME ":open() called but NOP\n");

	return 0;
}

/* 
 * ioctl() method:
 */
/* FUJITSU:2011-12-01 mod irda start */
long  msm_uart_aux_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
/* FUJITSU:2011-12-01 mod irda start */
{
/* FUJITSU:2011-12-01 mod irda start */
	long ret = 0;
/* FUJITSU:2011-12-01 mod irda start */

	pr_debug(DEV_NAME":ioctl() cmd=%x\n", cmd );

 	if ( _IOC_TYPE(cmd) != MSM_AUX_IOC_TYPE )
	{
  		printk( KERN_ERR DEV_NAME ":ioctl() TYPE error" );
  		printk( KERN_ERR DEV_NAME ":        IOCTL_CHG_IRDA=0x%x", MSM_AUX_IOCTL_CHG_IRDA );
  		printk( KERN_ERR DEV_NAME ":        IOCTL_CHG_UART=0x%x\n", MSM_AUX_IOCTL_CHG_UART );
  		return -ENOTTY;
 	}

 	switch ( cmd ) 
 	{
   		case MSM_AUX_IOCTL_CHG_IRDA:
		  	printk( KERN_INFO DEV_NAME ":ioctl(MSM_AUX_IOCTL_CHG_IRDA) set IRDA mode\n");
		  	/* UART_IRDA is write only */
 			iowrite32( 0x3, (void *)(uart_membase  +  UART_IRDA ) ); /* set IRDA_EN=1 INVERT_IRDA_RX = 1 */
	 		ret = msm_uart_aux_chg_irda();
 			ret = msm_uart_aux_irda_start();
  			break;

   		case MSM_AUX_IOCTL_CHG_UART:
 			ret = msm_uart_aux_irda_stop();
		  	printk( KERN_INFO DEV_NAME ":ioctl(MSM_AUX_IOCTL_CHG_UART) set UART mode\n");
  			iowrite32( 0x0, (void *)(uart_membase  +  UART_IRDA ) ); /* set IRDA_EN=0 INVERT_IRDA_RX = 0 */
	 		ret = msm_uart_aux_chg_uart();
  			break;

	   default:  /* invalid cmd */
	   		printk( KERN_ERR DEV_NAME":ioctl() cmd No. error %d\n",  cmd );
	  		ret = -EINVAL;
	  		break;
	 } /* end of switch */

	return ret;
}

#ifdef SUPPORT_MSM_UART_AUX_MMAP
/* 
 * mmap() method: debug
 */
int msm_uart_aux_mmap(struct file *filp, struct vm_area_struct *vma )
{
  pr_debug(DEV_NAME":mmap() called but NOP\n");

  return 0;
}
#endif /* SUPPORT_MSM_UART_AUX_MMAP */


/* 
 * release method:
 */
static int msm_uart_aux_release(struct inode *inode, struct file *filp)
{
	pr_debug(DEV_NAME":close() called but NOP\n");

	return 0;
}

static int msm_uart_aux_probe(struct platform_device *pdev)
{
	return 0;
}

static int msm_uart_aux_remove(struct platform_device *pdev)
{
	return 0;
}

static struct file_operations driver_fops = {
	.open 		= msm_uart_aux_open,
/* FUJITSU:2011-12-01 mod irda start */
	.unlocked_ioctl		= msm_uart_aux_ioctl,
/* FUJITSU:2011-12-01 mod irda end */
#ifdef SUPPORT_MSM_UART_AUX_MMAP
	.mmap		= msm_uart_aux_mmap,
#endif /* SUPPORT_MSM_UART_AUX_MMAP */
	.release 	= msm_uart_aux_release,
};

static struct platform_driver irda_driver = {
	.probe      = msm_uart_aux_probe,
	.remove     = msm_uart_aux_remove,
	.driver = {
		.name   = DEV_NAME,
		.owner  = THIS_MODULE,
	},
};

/* 
 * init method:
 */
static int msm_uart_aux_init(void)
{
	int ret = 0;
	void *res;
	dev_t dev = MKDEV( driver_major_no, 0 );

	// register class "MSM_UART_AUX"
	irda_class = class_create( THIS_MODULE, DEV_NAME );
	if( IS_ERR( irda_class ) )
	{
  		printk(KERN_ERR  DEV_NAME ":class create failed\n" );
		ret = PTR_ERR( irda_class );
		goto err_class;
	}
	pr_debug(DEV_NAME ": init_module: %s start\n", DEV_NAME );

	// Register char_dev "MSM_UART_AUX"
	if ( driver_major_no )
	{
		ret = register_chrdev_region( dev, 2, DEV_NAME );
	}
	else
	{
		ret = alloc_chrdev_region( &dev, 0, 2, DEV_NAME );
		driver_major_no = MAJOR(dev);
	}
	if ( ret < 0 )
	{
  		printk(KERN_ERR  DEV_NAME ":can't set major %d\n", driver_major_no );
		goto err_alloc;
	}
	if ( driver_major_no == 0 )
	{ 
		driver_major_no = ret;
		pr_debug(DEV_NAME ": Major no. is assigned to %d.\n", ret);
	}
	dev = MKDEV( driver_major_no, 0 );
  	printk(KERN_INFO  DEV_NAME ": UART Major No = %d\n", driver_major_no );
	cdev_init( &char_dev, &driver_fops );
	char_dev.owner = THIS_MODULE;
	ret = cdev_add( &char_dev, dev, 1 );
	if ( ret < 0 )
	{
  		printk(KERN_ERR  DEV_NAME ": cdev_add failed\n" );
		goto err_cdev;
	}
	irda_device = device_create( irda_class, NULL, dev, NULL, DEV_NAME );
	if( IS_ERR( irda_device ) )
	{
  		printk(KERN_ERR  DEV_NAME ": device create failed\n" );
		ret = PTR_ERR( irda_device );
		goto err_device;
	}

	// register driver "MSM_UART_AUX"
	ret = platform_driver_register( &irda_driver );
	if ( ret < 0 )
	{
  		printk(KERN_ERR  DEV_NAME ": platform driver register failed\n" );
		goto err_platform;
	}

	// Set UART MemBase
	res = request_mem_region(THIS_UART_MEMBASE, THIS_UART_MEMSIZE, "sir_iomem" );
	if (res < 0)
	{
		printk(KERN_ERR  DEV_NAME ":request mem region fail.\n" );
		ret = -EBUSY;
		goto err_mem;
	}
	uart_membase = (unsigned long)ioremap_nocache(THIS_UART_MEMBASE,THIS_UART_MEMSIZE);
	if ( uart_membase == 0 )
	{
		printk(KERN_ERR  DEV_NAME ":ioremap() fail.\n" );
		ret = -EBUSY;
		goto err_iomap;
	}
	pr_debug(DEV_NAME ": ioremap() retuns %x\n", (int)uart_membase);
	printk(KERN_INFO DEV_NAME ": ioremap() retuns %x\n", (int)uart_membase );

	// RX Config
/* FUJITSU:2011-09-28 gpio chg start  */
#if defined(CONFIG_MACH_F11APO) || defined(CONFIG_MACH_F12APON)
	if((system_rev != 0x08) || (system_rev != 0x0F))
	{
		uart_rx2_en_34.pull = PM_GPIO_PULL_UP_1P5;
	}
#elif defined(CONFIG_MACH_FJI12)
	if((system_rev != 0x00) || (system_rev != 0x07))
	{
		uart_rx2_en_34.pull = PM_GPIO_PULL_UP_1P5;
	}
#endif
/* FUJITSU:2011-09-28 gpio chg end  */
/* FUJITSU:2011-12-01 mod irda start  */
#if 0
	ret = pm8058_gpio_config( PM8058_UART_RX2_34, &uart_rx2_en_34 );
#else
	ret = pm8xxx_gpio_config( PM8058_UART_RX2_34, &uart_rx2_en_34 );
#endif
/* FUJITSU:2011-12-01 mod irda end  */
	
	if ( ret )
	{
		printk( KERN_ERR "%s: PM8058_UART_RX config failed[%d]\n", __func__, ret );
	}
	// SD -> High
/* FUJITSU:2011-09-28 gpio chg start  */
#if defined(CONFIG_MACH_F11APO) || defined(CONFIG_MACH_F12APON)
	if((system_rev != 0x08) || (system_rev != 0x0F))
	{
		irda_pwdown_26.output_buffer = PM_GPIO_OUT_BUF_OPEN_DRAIN;
		irda_pwdown_26.out_strength = PM_GPIO_STRENGTH_NO;
	}
#elif defined(CONFIG_MACH_FJI12)
	if((system_rev != 0x00) || (system_rev != 0x07))
	{
		irda_pwdown_26.output_buffer = PM_GPIO_OUT_BUF_OPEN_DRAIN;
		irda_pwdown_26.out_strength = PM_GPIO_STRENGTH_NO;
	}
#endif
/* FUJITSU:2011-09-28 gpio chg end  */
/* FUJITSU:2011-12-01 mod irda start  */
#if 0
	ret = pm8058_gpio_config( PM8058_IRDA_PWD_26, &irda_pwdown_26 );
#else
	ret = pm8xxx_gpio_config( PM8058_IRDA_PWD_26, &irda_pwdown_26 );
#endif
/* FUJITSU:2011-12-01 mod irda end  */
	if( ret < 0 )
	{
		printk( KERN_ERR "%s: PM8058_IRDA_PWD_26 config failed[%d]\n", __func__, ret );
	}
/* FUJITSU:2011-12-01 mod irda start  */
#if 0
	gpio_set_value_cansleep( PM8058_GPIO_PM_TO_SYS( PM8058_IRDA_PWD_26 ), 1 );
#else
	gpio_set_value_cansleep( PM8058_IRDA_PWD_26, 1 );
#endif
/* FUJITSU:2011-12-01 mod irda end  */

	// Init Normally
  	printk(KERN_INFO DEV_NAME ":Load normally\n");

  	return 0;

err_iomap:
	release_mem_region( THIS_UART_MEMBASE, THIS_UART_MEMSIZE );
err_mem:
	platform_driver_unregister( &irda_driver );
err_platform:
	device_destroy( irda_class, MKDEV( driver_major_no, 0 ) );
err_device:
	cdev_del( &char_dev );
err_cdev:
	unregister_chrdev_region( dev, 2 );
err_alloc:
	class_destroy( irda_class );
err_class:
	return ret;
}

/* 
 * exit method:
 */
static void msm_uart_aux_exit(void)
{
	dev_t dev = MKDEV( driver_major_no, 0 );

	iounmap( (void *)uart_membase );
	release_mem_region( THIS_UART_MEMBASE, THIS_UART_MEMSIZE );
	device_destroy( irda_class, MKDEV( driver_major_no, 0 ) );
	platform_driver_unregister( &irda_driver );

	cdev_del( &char_dev );
	unregister_chrdev_region( dev, 1 );

	class_destroy( irda_class );

  	printk(KERN_INFO DEV_NAME ":Unload normally\n");
}

module_init(msm_uart_aux_init);
module_exit(msm_uart_aux_exit);


MODULE_AUTHOR("FUJITSU");
MODULE_LICENSE( "GPL" );
MODULE_VERSION("1.0.0");