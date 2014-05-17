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

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/leds.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/workqueue.h>
#include <linux/err.h>
#include <linux/ctype.h>
#include <linux/gpio.h>
#include "leds.h"
/* FUJITSU:2011-08-18 AN32155AB start */
#include <linux/ctype.h>
#include "../arch/arm/mach-msm/smd_private.h"
/* FUJITSU:2011-08-18 AN32155AB end */

#define PMIC_GPIO_LIGHT	25  /* PMIC GPIO Number 25 */
#define PM8058_GPIO_PM_TO_SYS(pm_gpio)     (pm_gpio + NR_GPIO_IRQS)

static struct workqueue_struct *mobile_led_workqueue;

/*===========================================================================
  GLOBAL VARIABLES
  ===========================================================================*/

extern void timer_trig_probe(struct led_classdev *led_cdev);

static void
mobile_led_queue_work(struct led_classdev * led_cdev, enum led_brightness value)
{
	led_cdev->w_brightness = value;
	queue_work(mobile_led_workqueue, &led_cdev->work);
}

static void
mobile_led_set (struct led_classdev *led_cdev, enum led_brightness value)
{
	int i;

	if (value){
		gpio_set_value_cansleep(PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_LIGHT -1), 1);
		mdelay(2);
		for (i = 0; i < value; i++) {
			gpio_set_value_cansleep(PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_LIGHT -1), 0);
			gpio_set_value_cansleep(PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_LIGHT -1), 1);
		}
	}
	else {
		gpio_set_value_cansleep(PM8058_GPIO_PM_TO_SYS(PMIC_GPIO_LIGHT -1), 0);
	}
}

static struct led_classdev mobile_led_data = {
   .name = "mobile-light",
   .brightness_set = mobile_led_queue_work,
   .brightness = LED_OFF,
   .max_brightness = 16,
};

static void mobile_led_work(struct work_struct * w)
{
	struct led_classdev * led = container_of(w, struct led_classdev, work);
	if (!strcmp(led->name, "mobile-light"))
		mobile_led_set(led, led->w_brightness);
	else
		pr_err("unknown LED device %s\n", led->name);
}

static int
mobile_led_probe (struct platform_device *pdev)
{
	int rc;

	INIT_WORK(&mobile_led_data.work, mobile_led_work);
	rc = led_classdev_register (&pdev->dev, &mobile_led_data);
	if (rc){
		printk (KERN_ERR "unable to register led class driver :%s\n",
		mobile_led_data.name);
		return rc;
	}
	timer_trig_probe(&mobile_led_data);

	mobile_led_workqueue = create_singlethread_workqueue("mobile_led");
	return rc;
}

static int __devexit
mobile_led_remove (struct platform_device *pdev)
{
	led_classdev_unregister (&mobile_led_data);
	destroy_workqueue(mobile_led_workqueue);
	return 0;
}

static struct platform_driver mobile_led_driver = {
	.probe = mobile_led_probe,
	.remove = mobile_led_remove,
	.driver = {
	.name = "mobile-led",
	.owner = THIS_MODULE,
	},
};

static int __init
i2c_led_init (void)
{
	return platform_driver_register (&mobile_led_driver);
}
module_init (i2c_led_init);

static void __exit
i2c_led_exit (void)
{
	platform_driver_unregister (&mobile_led_driver);
}
module_exit (i2c_led_exit);

MODULE_AUTHOR("FUJITSU");
MODULE_DESCRIPTION ("MOBILE LED driver");
MODULE_LICENSE ("GPL v2");
MODULE_VERSION("1.0");
MODULE_ALIAS ("platform:mobile-leds");
