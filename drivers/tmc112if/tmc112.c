/*----------------------------------------------------------------------------*/
// COPYRIGHT(C) FUJITSU LIMITED 2011
/*----------------------------------------------------------------------------*/

#define DEBUG

#ifdef	DEBUG
#define	DBG_PRINTK(x, ...)	printk("tmc112(%d):" x, __LINE__, ##__VA_ARGS__)
#endif

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/irqreturn.h>
#include <linux/init.h>
#include <linux/spi/spi.h>
#include <linux/firmware.h>
#include <linux/ihex.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>

#include <linux/gpio.h>
#include <linux/types.h>
#include <linux/gfp.h>
#include <linux/errno.h>
#include <linux/sched.h>
#include <linux/poll.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/spinlock_types.h>

#include <linux/clk.h>
#include <linux/ioctl.h>

#include <mach/msm_iomap.h>

#ifndef _TMC112_H_
#include "tmc112.h"
#endif


/**********************************************************************/
/* macros															  */
/**********************************************************************/
#define	DEBUG_SLOW_WRITE				1		//

/*
	IFFEL GPIO
*/
#define	MBP_MODE_CLK_B		GPIO_CFG(98, 2, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_4MA)	///< CLK_B
#define	MBP_MODE_RESET		GPIO_CFG(100, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA)	///< RESET
#define	MBP_MODE_REQUEST	GPIO_CFG(162, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA)	///< REQU
#define	MBP_MODE_READY		GPIO_CFG(99, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA)		///< READY
#define	MBP_MODE_XIRQ		GPIO_CFG(92, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA)		///< XIRQ


#define	SPI_CLK_MAX_2ND	(26331429)			///26MHz

#ifdef	DEBUG_SLOW_WRITE
/* 2011-10-04 SPI-CLK 1.6MHz [mod] START */
//#define	SPI_CLK_MAX_RW	400000				///< Camera Firm
//#define	SPI_CLK_MAX_RW	1638400				///< Camera Firm
#define	SPI_CLK_MAX_RW	9963243				///< Camera Firm
/* 2011-10-04 SPI-CLK 1.6MHz [mod] END */
#else
#define	SPI_CLK_MAX_RW	SPI_CLK_MAX_2ND		///< Cmd/Status26MHz
#endif


/**********************************************************************/
/* globals															  */
/**********************************************************************/
static struct spi_device *tmc112 = NULL;	/* spi_device* created in probe */

/**
 *	tmc112_read
 *
 *	read the status from ISP
 *
 *	@param	*f		[in]	file descriptor
 *	@param	*data	[in]	buffer
 *	@param	len		[in]	max size
 *	@param	*off	[in]	read offset in buffer
 *	@return	>0	bytes read<br>
 *			==0	canceled by Close() is invoked<br>
 *			<0	error
 */
static ssize_t tmc112_read(struct file *f, char __user *data, size_t len, loff_t *off)
{
	return len;
}


/**
 *	tmc112_write
 *
 *	write() sends command/parameter to ISP.
 *
 *	@param	*f		[in]	file descriptor
 *	@param	*data	[in]	buffer
 *	@param	len		[in]	data size
 *	@param	*off	[in]	offset in buffer
 *	@return	>0	written size<br>
 *			<=0	error
 */
static ssize_t tmc112_write(struct file *f, const char __user *data, size_t len, loff_t *off)
{
	u8 *buf = kzalloc(len, GFP_KERNEL);
	struct spi_message m;
	struct spi_transfer t;
	int ret;

	DBG_PRINTK("%s\n", __func__);
	DBG_PRINTK("*data=%p, len=%x, off=%p\n", data, len, off);


	if (!buf) {
		return -ENOMEM;
	}

	if (copy_from_user(buf, data, len)) {
		ret = -EFAULT;
		goto err;
	}

	spi_message_init(&m);
	memset(&t, 0, sizeof(t));
	t.tx_buf = buf;
	t.len = len;
	t.bits_per_word = 8;
#ifdef	DEBUG_SLOW_WRITE
	t.speed_hz = SPI_CLK_MAX_RW;
#endif
	spi_message_add_tail(&t, &m);

	ret = spi_sync(tmc112, &m);
	if (ret != 0)
		goto err;

	kfree(buf);
	DBG_PRINTK("%s OK\n", __func__);
	return len;

err:
	DBG_PRINTK("%s ERR=%d\n", __func__, ret);
	kfree(buf);
	return ret;
}


/**
 *	tmc112_open
 *
 *	tmc112 initialize
 *
 *	@param	inode	[in]	inode
 *	@param	filp	[in]	filp
 *	@return	status<br>
 *			==0	OK<br>
 *			<0	error
 */
static int tmc112_open(struct inode *inode, struct file *filp)
{
	int rc = 0;

	DBG_PRINTK("%s\n", __func__);

	if (filp == NULL) {
		return -ENOMEM;
	}
	if (!try_module_get(THIS_MODULE)) {
		return -ENODEV;
	}


	/* GPIO value dump start */
#if 0
	DBG_PRINTK("%s ***********************************\n", __func__);
	rc = gpio_get_value(44);
	DBG_PRINTK("%s GPIO44 value = %d\n", __func__,rc);
	rc = gpio_get_value(45);
	DBG_PRINTK("%s GPIO45 value = %d\n", __func__,rc);
	rc = gpio_get_value(46);
	DBG_PRINTK("%s GPIO46 value = %d\n", __func__,rc);
	rc = gpio_get_value(47);
	DBG_PRINTK("%s GPIO47 value = %d\n", __func__,rc);
	rc = gpio_get_value(48);
	DBG_PRINTK("%s GPIO48 value = %d\n", __func__,rc);
	DBG_PRINTK("%s ***********************************\n", __func__);
#endif
	/* GPIO value dump end */

	/* GPIO44(CS1) Low Setting */
	gpio_tlmm_config(GPIO_CFG(44, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	rc = gpio_get_value(44);
	DBG_PRINTK("%s GPIO44 [before] gpio_get_value = %d\n", __func__,rc);
	gpio_set_value(44, 0);
	DBG_PRINTK("%s GPIO44 Set Low\n", __func__);
	rc = gpio_get_value(44);
	DBG_PRINTK("%s GPIO44 [after] gpio_get_value = %d\n", __func__,rc);
	
	/* wait 1ms */
	msleep(1);
	DBG_PRINTK("%s 1ms wait\n", __func__);

	/* GPIO45(SPI_CLK) High Setting */
	gpio_tlmm_config(GPIO_CFG(45, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), GPIO_CFG_ENABLE);
	rc = gpio_get_value(45);
	DBG_PRINTK("%s GPIO45 [before] gpio_get_value = %d\n", __func__,rc);
	gpio_set_value(45, 1);
	DBG_PRINTK("%s GPIO45 Set High\n", __func__);
	rc = gpio_get_value(45);
	DBG_PRINTK("%s GPIO45 [after] gpio_get_value = %d\n", __func__,rc);
	gpio_tlmm_config(GPIO_CFG(45, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), GPIO_CFG_ENABLE);

	/* wait 1ms */
	msleep(1);
	DBG_PRINTK("%s 1ms wait\n", __func__);

#if 0	/* GPIO47 CTRL F11SKY only */
	/* GPIO47(SPI_MOSI_DATA) Setting */
	gpio_tlmm_config(GPIO_CFG(47, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL,	GPIO_CFG_16MA), GPIO_CFG_ENABLE);
#endif	/* GPIO47 CTRL F11SKY only */

	/* GPIO44(CS1) High Setting */
	gpio_tlmm_config(GPIO_CFG(44, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	rc = gpio_get_value(44);
	DBG_PRINTK("%s GPIO44 [before] gpio_get_value = %d\n", __func__,rc);
	gpio_set_value(44, 1);
	DBG_PRINTK("%s GPIO44 Set High\n", __func__);
	rc = gpio_get_value(44);
	DBG_PRINTK("%s GPIO44 [after] gpio_get_value = %d\n", __func__,rc);

	msleep(5);
	DBG_PRINTK("%s 5ms wait\n", __func__);
	


DBG_PRINTK("%s() SUCCESS\n", __func__);
	return 0;

}

/**
 *	tmc112_release
 *
 *	disable ISP<br>
 *
 *	@param	inode	[in]	inode
 *	@param	filp	[in]	filp
 *	@return	error code<br>
 *			0	success<br>
 *			<0	fail
 */
static int tmc112_release(struct inode *inode, struct file *filp)
{
	int rc = 0;

	DBG_PRINTK("%s\n", __func__);

	/* parameter check */
	if (filp == NULL) {
		return -EBADF;
	}

	/* wait 5ms */
	msleep(5);
	DBG_PRINTK("%s 5ms wait\n", __func__);

	/* GPIO44(CS1) Low Setting */
	gpio_tlmm_config(GPIO_CFG(44, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	rc = gpio_get_value(44);
	DBG_PRINTK("%s GPIO44 [before] gpio_get_value = %d\n", __func__,rc);
	gpio_set_value(44, 0);
	DBG_PRINTK("%s GPIO44 Set Low\n", __func__);
	rc = gpio_get_value(44);
	DBG_PRINTK("%s GPIO44 [after] gpio_get_value = %d\n", __func__,rc);
	
	/* GPIO45(SPI_CLK) Low Setting */
	gpio_tlmm_config(GPIO_CFG(45, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	rc = gpio_get_value(45);
	DBG_PRINTK("%s GPIO45 [before] gpio_get_value = %d\n", __func__,rc);
	gpio_set_value(45, 0);
	DBG_PRINTK("%s GPIO45 Set Low\n", __func__);
	rc = gpio_get_value(45);
	DBG_PRINTK("%s GPIO45 [after] gpio_get_value = %d\n", __func__,rc);
#if 1	/* GPIO45 CTRL F11SKY only */
	gpio_tlmm_config(GPIO_CFG(45, 1, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_16MA), GPIO_CFG_ENABLE);
#endif	/* GPIO45 CTRL F11SKY only */

#if 0	/* GPIO47 CTRL F11SKY only */
	/* GPIO47(SPI_MOSI_DATA) Setting */
	gpio_tlmm_config(GPIO_CFG(47, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
	gpio_set_value(47, 0);
	DBG_PRINTK("%s GPIO47 Set Low\n", __func__);
	rc = gpio_get_value(47);
	DBG_PRINTK("%s GPIO47 Restore default value = %d\n", __func__,rc);
#endif	/* GPIO47 CTRL F11SKY only */



	module_put(THIS_MODULE);

	return 0;
}

/* FUJITSU 2012-03-27:KOUTEI mod start */
//static int tmc112_ioctl(
static long tmc112_ioctl(
/* FUJITSU 2012-03-27:KOUTEI mod end */
//	struct inode *inode,
	struct file *filp,
	unsigned int cmd,
	unsigned long arg)
{
	int ret = 0;

	DBG_PRINTK("%s\n", __func__);

	return  ret;
}

/* tmc112 driver FOPS */
static struct file_operations tmc112_fops = {
	.read			= tmc112_read,				/* read Entry    */
	.write			= tmc112_write,				/* write Entry   */
	.unlocked_ioctl	= tmc112_ioctl,				/* ioctl Entry   */
	.open			= tmc112_open,				/* open Entry    */
	.release		= tmc112_release,			/* release Entry */
};

/* driver definition */
static struct miscdevice tmc112_miscdev = {
	.minor = MISC_DYNAMIC_MINOR,			/* auto        */
	.name = "tmc112",						/* Driver name */
	.fops = &tmc112_fops,					/* FOPS        */
};

/**
 *	tmc112_halt
 *
 *	halt the driver
 *
 *	@return	0	success
 */
int tmc112_halt(void)
{

	DBG_PRINTK("%s\n", __func__);

	return 0;
}


/*
 *	tmc112_resume
 *
 *	resume from suspend<br>
 *
 *	@param	spi	[in]	spi_device*
 *	@return	0	success
 */
static int tmc112_resume(struct spi_device *spi)
{
	DBG_PRINTK("%s\n", __func__);

	dev_dbg(&spi->dev, "%s()\n", __func__);
	return 0;
}

/*
 *	tmc112_suspend
 *
 *	goto suspend<br>
 *
 *	@param	spi	[in]		spi_device*
 *	@param	mesg	[in]	pm_message_t
 *	@return	0	success
 */
static int tmc112_suspend(struct spi_device *spi, pm_message_t mesg)
{
	int ret = 0;
	DBG_PRINTK("%s\n", __func__);

	dev_dbg(&spi->dev, "%s(), msg=%d\n", __func__, mesg.event);
	return ret;
}


/**
 *	tmc112_spi_probe
 *
 *	Probe<br>
 *
 *	@param	spi	[in]	spi_device
 *	@return	0	success<br>
 *			<0	fail
 */
static int __devinit tmc112_spi_probe(struct spi_device *spi)
{
	int ret;
	struct tmc112_pdata *pdata = NULL;

	DBG_PRINTK("tmc112_spi_probe\n");

	pdata = (struct tmc112_pdata *)kzalloc(sizeof(struct tmc112_pdata), GFP_KERNEL);
	if (!pdata) {
		dev_err(&spi->dev, "Please provide valid platform data\n");
		return -EINVAL;
	}

	pdata->pin_reset = 0;
	pdata->pin_request = 0;
	pdata->pin_ready = 0;
	pdata->pin_xirq = 0;
	pdata->gpio_request_irq = 0;
	pdata->vreg_core = NULL;
	pdata->vreg_io = NULL;
	pdata->clk_b = NULL;

	pdata->request_raised = 0;
	pdata->ready_raised = 0;
	
	pdata->suspend = 0;
	pdata->suspend_withrun = 0;
	pdata->resume = 0;

	pdata->running = 0;
	pdata->cancel_read = 0;


	tmc112_miscdev.parent = &spi->dev;
	ret = misc_register(&tmc112_miscdev);
	if (ret < 0) {
		dev_err(&spi->dev, "Failed to register miscdev: %d\n", ret);
		kfree(pdata);
		return ret;
	}


	tmc112 = spi;
	spi->dev.platform_data = pdata;
	spi->chip_select = 1;

	return 0;

}

/**
 *	tmc112_spi_remove
 *
 *	remove<br>
 *	deregister driver it was registered in Probe
 *
 *	@param	spi	[in]	spi_device*
 *	@return	0	success<br>
 */
static int __devexit tmc112_spi_remove(struct spi_device *spi)
{
	struct tmc112_pdata *pdata = dev_get_platdata(&spi->dev);

	dev_err(&spi->dev, "%s\n", __func__);

	misc_deregister(&tmc112_miscdev);
	tmc112 = NULL;
	if (pdata) {
		kfree(pdata);
	}

	return 0;
}


/* tmc112 driver as spi child driver */
static struct spi_driver tmc112_spi_driver = {
	.driver = {
		.name	= "tmc112_firm_transfer",				///< 
		.bus 	= &spi_bus_type,						///< 
		.owner	= THIS_MODULE,							///< owner
	},
	.probe		= tmc112_spi_probe,						///< Probe Entry
	.remove		= __devexit_p(tmc112_spi_remove),		///< Remove Entry
	.suspend	= tmc112_suspend,						///< Suspend Entry
	.resume		= tmc112_resume,						///< Resume Entry
};


/**
 *	tmc112_init
 *
 *	Initialize driver<br>
 *	register with spi_register_driver
 *
 *	@return	0	success<br>
 *			<0	fail
 */
static int __init tmc112_init(void)
{
	int ret;
	DBG_PRINTK("tmc112_init\n");

	ret = spi_register_driver(&tmc112_spi_driver);
	if (ret) {
		DBG_PRINTK("tmc112_init ret=%d\n", ret);
	}
	return ret;
}
module_init(tmc112_init);

/**
 *	tmc112_exit
 *
 *	finalize driver<br>
 *	deregister spi_register_driver
 */
static void __exit tmc112_exit(void)
{
	DBG_PRINTK("tmc112_exit\n");
	spi_unregister_driver(&tmc112_spi_driver);
}
module_exit(tmc112_exit);

MODULE_DESCRIPTION("ASoC tmc112 driver");
MODULE_AUTHOR("Fujitsu");
MODULE_LICENSE("GPL");
