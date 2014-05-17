/*
 * Source for:
 * Cypress TrueTouch(TM) Standard Product (TTSP) I2C TouchPress driver.
 * For use with Cypress Txx3xx parts.
 * Supported parts include:
 * CY8CTST341
 * CY8CTMA340
 *
 * Copyright (C) 2009-2012 Cypress Semiconductor, Inc.
 * Copyright (C) 2010-2012 Motorola Mobility, Inc.
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
// COPYRIGHT(C) FUJITSU LIMITED 2012
/*----------------------------------------------------------------------------*/

/*============================================================================
                        INCLUDE FILES
============================================================================*/
#include "cyttsp_press_core.h"
#include <linux/input/touch_platform.h>
#include <linux/i2c.h>
#include <linux/slab.h>

#ifndef _CYTTSP_PRESS_GPIO_H_
#include "cyttsp_press_gpio.h"
#endif

/*============================================================================
                        INTERNAL FEATURES
============================================================================*/

/*============================================================================
                        CONSTANTS
============================================================================*/
#define CY_PR_I2C_DATA_SIZE     128

struct cyttsp_i2c {
    struct cyttsp_bus_ops ops;
    struct i2c_client *client;
    void *ttsp_client;
    u8 wr_buf[CY_PR_I2C_DATA_SIZE];
};

bool is_use_i2c = false;

/*===========================================================================
    FUNCTION  CYTTSP_PRESS_I2C_READ
===========================================================================*/
static s32 cyttsp_press_i2c_read(void *handle, u8 addr, u8 length, void *values)
{
    int retval = 0;
    struct cyttsp_i2c *ts = container_of(handle, struct cyttsp_i2c, ops);

    retval = i2c_master_send(ts->client, &addr, 1);
    if (retval < 0) {
        return retval;
    } else if (retval != 1) {
        return ~EIO;
    }

    retval = i2c_master_recv(ts->client, values, length);

    return (retval < 0) ? retval : retval != length ? ~EIO : 0;
}

/*===========================================================================
    FUNCTION  CYTTSP_PRESS_I2C_WRITE
===========================================================================*/
static s32 cyttsp_press_i2c_write(void *handle, u8 addr, u8 length, const void *values)
{
    int retval = 0;
    struct cyttsp_i2c *ts = container_of(handle, struct cyttsp_i2c, ops);

    ts->wr_buf[0] = addr;
    memcpy(&ts->wr_buf[1], values, length);

    retval = i2c_master_send(ts->client, ts->wr_buf, length + 1);

    return (retval < 0) ? retval : retval != length + 1 ? ~EIO : 0;
}

/*===========================================================================
    FUNCTION  CYTTSP_PRESS_GPIOI2C_READ
===========================================================================*/
static s32 cyttsp_press_gpioi2c_read(void *handle, u8 addr, u8 length, void *values)
{
    int retval = 0;
    struct PressI2CCmdType recvI2CCmd;
    struct cyttsp_i2c *ts = container_of(handle, struct cyttsp_i2c, ops);

    recvI2CCmd.slave_addr = CY_PR_I2C_ADR;
    recvI2CCmd.pwdata     = (char *)&addr;
    recvI2CCmd.wlen       = 1;
    recvI2CCmd.prdata     = values;
    recvI2CCmd.rlen       = length;
    recvI2CCmd.tsdebug    = ts->ops.tsdebug;

    retval = press_gpioi2c_read(&recvI2CCmd);
    return (retval < 0) ? retval : 0;
}

/*===========================================================================
    FUNCTION  CYTTSP_PRESS_GPIOI2C_WRITE
===========================================================================*/
static s32 cyttsp_press_gpioi2c_write(void *handle, u8 addr, u8 length, const void *values)
{
    int retval = 0;
    struct PressI2CCmdType sendI2CCmd;
    u8 wr_buf[CY_PR_I2C_DATA_SIZE] = "";
    struct cyttsp_i2c *ts = container_of(handle, struct cyttsp_i2c, ops);

    wr_buf[0] = addr;
    memcpy(&wr_buf[1], values, length);

    sendI2CCmd.slave_addr = CY_PR_I2C_ADR;
    sendI2CCmd.pwdata     = (char *)wr_buf;
    sendI2CCmd.wlen       = length + 1;
    sendI2CCmd.tsdebug    = ts->ops.tsdebug;

    retval = press_gpioi2c_write(&sendI2CCmd);
    return (retval < 0) ? retval : 0;
}

/*===========================================================================
    FUNCTION  WRAPPER_I2C_WRITE
===========================================================================*/
static int wrapper_i2c_write(void *handle, u8 addr, u8 length, const void *values)
{
    int rc = 0;
    if (is_use_i2c) {
        rc = cyttsp_press_i2c_write(handle, addr, length, values);
    } else {
        rc = cyttsp_press_gpioi2c_write(handle, addr, length, values);
    }
    return rc;
}

/*===========================================================================
    FUNCTION  WRAPPER_I2C_READ
===========================================================================*/
static int wrapper_i2c_read(void *handle, u8 addr, u8 length, void *values)
{
    int rc = 0;

    if (is_use_i2c) {
        rc = cyttsp_press_i2c_read(handle, addr, length, values);
    } else {
        rc = cyttsp_press_gpioi2c_read(handle, addr, length, values);
    }
    return rc;
}

/*===========================================================================
    FUNCTION  CYTTSP_PRESS_I2C_PROBE
===========================================================================*/
static int __devinit cyttsp_press_i2c_probe(struct i2c_client *client,
    const struct i2c_device_id *id)
{
    struct cyttsp_i2c *ts;

    pr_info("[PPD-i2c]%s: Starting %s probe...\n", __func__, CY_PR_I2C_NAME);

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
        pr_err("[PPD-i2c]%s: fail check I2C functionality\n", __func__);
        return -EIO;
    }

    // allocate and clear memory
    ts = kzalloc(sizeof(*ts), GFP_KERNEL);
    if (!ts) {
        pr_err("[PPD-i2c]%s: Error, kzalloc.\n", __func__);
        return -ENOMEM;
    }

    // register driver_data
    ts->client = client;
    i2c_set_clientdata(client, ts);
    ts->ops.write = wrapper_i2c_write;
    ts->ops.read = wrapper_i2c_read;
    ts->ops.dev = &client->dev;
    ts->ops.dev->bus = &i2c_bus_type;

    ts->ttsp_client = cyttsp_press_core_init(&ts->ops, &client->dev,
            client->irq, client->name);

    if (ts->ttsp_client == NULL) {
        kfree(ts);
        pr_err("[PPD-i2c]%s: Registration for %s failed\n", __func__, CY_PR_I2C_NAME);
        return -EFAULT;
    }

    pr_info("[PPD-i2c]%s: Registration for %s complete (Built %s @ %s)\n",
        __func__, CY_PR_I2C_NAME, __DATE__, __TIME__);
    return 0;
}

/*===========================================================================
    FUNCTION  CYTTSP_PRESS_I2C_REMOVE
===========================================================================*/
static int __devexit cyttsp_press_i2c_remove(struct i2c_client *client)
{
    struct cyttsp_i2c *ts = i2c_get_clientdata(client);
    cyttsp_press_core_release(ts->ttsp_client);
    kfree(ts);
    return 0;
}

#if defined(CONFIG_PM) && !defined(CONFIG_HAS_EARLYSUSPEND)
/*===========================================================================
    FUNCTION  CYTTSP_PRESS_I2C_SUSPEND
===========================================================================*/
static int cyttsp_press_i2c_suspend(struct i2c_client *client, pm_message_t message)
{
    struct cyttsp_i2c *ts = i2c_get_clientdata(client);
    return cyttsp_press_suspend(ts);
}

/*===========================================================================
    FUNCTION  CYTTSP_PRESS_I2C_RESUME
===========================================================================*/
static int cyttsp_press_i2c_resume(struct i2c_client *client)
{
    struct cyttsp_i2c *ts = i2c_get_clientdata(client);
    return cyttsp_press_resume(ts);
}
#endif

static const struct i2c_device_id cyttsp_press_i2c_id[] = {
    { CY_PR_I2C_NAME, 0 },  { }
};

static struct i2c_driver cyttsp_i2c_driver = {
    .driver = {
        .name = CY_PR_I2C_NAME,
        .owner = THIS_MODULE,
    },
    .probe = cyttsp_press_i2c_probe,
    .remove = __devexit_p(cyttsp_press_i2c_remove),
    .id_table = cyttsp_press_i2c_id,
#if defined(CONFIG_PM) && !defined(CONFIG_HAS_EARLYSUSPEND)
    .suspend = cyttsp_press_i2c_suspend,
    .resume = cyttsp_press_i2c_resume,
#endif
};

/*===========================================================================
    FUNCTION  CYTTSP_PRESS_I2C_INIT
===========================================================================*/
static int __init cyttsp_press_i2c_init(void)
{
    return i2c_add_driver(&cyttsp_i2c_driver);
}

/*===========================================================================
    FUNCTION  CYTTSP_PRESS_I2C_EXIT
===========================================================================*/
static void __exit cyttsp_press_i2c_exit(void)
{
    return i2c_del_driver(&cyttsp_i2c_driver);
}

module_init(cyttsp_press_i2c_init);
module_exit(cyttsp_press_i2c_exit);

MODULE_ALIAS("i2c:cyttsp_press");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Cypress TrueTouch(R) Standard Product (TTSP) I2C Touch Press driver");
MODULE_AUTHOR("Cypress");
MODULE_DEVICE_TABLE(i2c, cyttsp_press_i2c_id);
