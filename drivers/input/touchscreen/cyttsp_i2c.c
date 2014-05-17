/*
 * Source for:
 * Cypress TrueTouch(TM) Standard Product (TTSP) I2C touchscreen driver.
 * For use with Cypress Txx3xx parts.
 * Supported parts include:
 * CY8CTST341
 * CY8CTMA340
 *
 * Copyright (C) 2009-2011 Cypress Semiconductor, Inc.
 * Copyright (C) 2010-2011 Motorola Mobility, Inc.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2, and only version 2, as published by the
 * Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301, USA.
 *
 * Contact Cypress Semiconductor at www.cypress.com <kev@cypress.com>
 *
 */
/*----------------------------------------------------------------------------*/
// COPYRIGHT(C) FUJITSU LIMITED 2011
/*----------------------------------------------------------------------------*/

/* FUJITSU:2011-08-11 add HeaderFile start */
#include "cyttsp_core.h"
#include <linux/input/touch_platform.h>
/* FUJITSU:2011-08-11 add HeaderFile end */

#include <linux/i2c.h>
#include <linux/slab.h>

#define CY_I2C_DATA_SIZE  128

struct cyttsp_i2c {
	struct cyttsp_bus_ops ops;
	struct i2c_client *client;
	void *ttsp_client;
	u8 wr_buf[CY_I2C_DATA_SIZE];
};

static s32 ttsp_i2c_read_block_data(void *handle, u8 addr,
	u8 length, void *values)
{
	struct cyttsp_i2c *ts = container_of(handle, struct cyttsp_i2c, ops);
	int retval = 0;

	retval = i2c_master_send(ts->client, &addr, 1);
	if (retval < 0)
		return retval;
	else if (retval != 1)
		return ~EIO;

	retval = i2c_master_recv(ts->client, values, length);

	return (retval < 0) ? retval : retval != length ? ~EIO : 0;
}

static s32 ttsp_i2c_write_block_data(void *handle, u8 addr,
	u8 length, const void *values)
{
	struct cyttsp_i2c *ts = container_of(handle, struct cyttsp_i2c, ops);
	int retval;

	ts->wr_buf[0] = addr;
	memcpy(&ts->wr_buf[1], values, length);

	retval = i2c_master_send(ts->client, ts->wr_buf, length+1);

	return (retval < 0) ? retval : retval != length+1 ? ~EIO : 0;
}

static int __devinit cyttsp_i2c_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	struct cyttsp_i2c *ts;
	int retval;

	pr_info("%s: Starting %s probe...\n", __func__, CY_I2C_NAME);

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		pr_err("%s: fail check I2C functionality\n", __func__);
		return -EIO;
	}

	/* allocate and clear memory */
	ts = kzalloc(sizeof(*ts), GFP_KERNEL);
	if (!ts) {
		pr_err("%s: Error, kzalloc.\n", __func__);
		return -ENOMEM;
	}

	/* register driver_data */
	ts->client = client;
	i2c_set_clientdata(client, ts);
	ts->ops.write = ttsp_i2c_write_block_data;
	ts->ops.read = ttsp_i2c_read_block_data;
	ts->ops.dev = &client->dev;
	ts->ops.dev->bus = &i2c_bus_type;

	ts->ttsp_client = cyttsp_core_init(&ts->ops, &client->dev,
			client->irq, client->name);

	if (IS_ERR(ts->ttsp_client)) {
		retval = PTR_ERR(ts->ttsp_client);
		kfree(ts);
		pr_err("%s: Registration fail ret=%d\n", __func__, retval);
		return retval;
	}

	/* FUJITSU:2011-08-31 mod start */
	pr_info("%s: Registration for %s complete (Built %s @ %s)\n",
		__func__, CY_I2C_NAME, __DATE__, __TIME__);
	/* FUJITSU:2011-08-31 mod end */

	return 0;
}

/* registered in driver struct */
static int __devexit cyttsp_i2c_remove(struct i2c_client *client)
{
	struct cyttsp_i2c *ts;

	pr_info("%s: exit\n", __func__);
	ts = i2c_get_clientdata(client);
	cyttsp_core_release(ts->ttsp_client);
	kfree(ts);
	return 0;
}

#if defined(CONFIG_PM) && !defined(CONFIG_HAS_EARLYSUSPEND)
static int cyttsp_i2c_suspend(struct i2c_client *client, pm_message_t message)
{
	struct cyttsp_i2c *ts = i2c_get_clientdata(client);

	return cyttsp_suspend(ts);
}

static int cyttsp_i2c_resume(struct i2c_client *client)
{
	struct cyttsp_i2c *ts = i2c_get_clientdata(client);

	return cyttsp_resume(ts);
}
#endif

static const struct i2c_device_id cyttsp_i2c_id[] = {
	{ CY_I2C_NAME, 0 },  { }
};

static struct i2c_driver cyttsp_i2c_driver = {
	.driver = {
		.name = CY_I2C_NAME,
		.owner = THIS_MODULE,
	},
	.probe = cyttsp_i2c_probe,
	.remove = __devexit_p(cyttsp_i2c_remove),
	.id_table = cyttsp_i2c_id,
#if defined(CONFIG_PM) && !defined(CONFIG_HAS_EARLYSUSPEND)
	.suspend = cyttsp_i2c_suspend,
	.resume = cyttsp_i2c_resume,
#endif
};

static int __init cyttsp_i2c_init(void)
{
	return i2c_add_driver(&cyttsp_i2c_driver);
}

static void __exit cyttsp_i2c_exit(void)
{
	return i2c_del_driver(&cyttsp_i2c_driver);
}

module_init(cyttsp_i2c_init);
module_exit(cyttsp_i2c_exit);

MODULE_ALIAS("i2c:cyttsp");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Cypress TrueTouch(R) Standard Product (TTSP) I2C driver");
MODULE_AUTHOR("Cypress");
MODULE_DEVICE_TABLE(i2c, cyttsp_i2c_id);
