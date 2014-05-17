/*
 * Copyright(C) 2012 FUJITSU LIMITED
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

/*============================================================================

                Touch Press Sensor GPIO Driver Source File

============================================================================*/

/*============================================================================
                        INCLUDE FILES
============================================================================*/
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <mach/gpio.h>
#include <asm/mach-types.h>
#include "cyttsp_press_gpio.h"

/*============================================================================
                        INTERNAL FEATURES
============================================================================*/

/*============================================================================
                        CONSTANTS
============================================================================*/
#define TRUE                    1
#define FALSE                   0

#define SCL_Low()               gpio_direction_output(SCL, 0)
#define SCL_High()              gpio_direction_input(SCL)
#define SDA_Low()               gpio_direction_output(SDA, 0)
#define SDA_High()              gpio_direction_input(SDA)
#define SDA_Read()              gpio_get_value(SDA)
#define SCL_Read()              gpio_get_value(SCL)

/*============================================================================
                        EXTERNAL VARIABLES DEFINITIONS
============================================================================*/
int _PRESS_I2C_WAIT_ = 12;
u8  _DEBUG_LEVEL_ = 0;

#define LOGE(fmt, args...) { \
    if (_DEBUG_LEVEL_ > 0) \
        pr_err("[PPD-gpio]" fmt, ##args); \
}

#define LOGI(fmt, args...) { \
    if (_DEBUG_LEVEL_ > 0) \
        pr_info("[PPD-gpio]" fmt, ##args); \
}


/*============================================================================
                        INTERNAL API DECLARATIONS
============================================================================*/
static void WaitHighSpeed(int us) {}
static void (*Wait)(int us) = WaitHighSpeed;

/*===========================================================================
    FUNCTION  SCL_ISHIGH
===========================================================================*/
static boolean SCL_IsHigh(void)
{
    int32_t cnt = 0;

    // Check SCL High
    // for (cnt = 1000 * 2; cnt; cnt--)
    for (cnt = 1000 * _PRESS_I2C_WAIT_; cnt; cnt--) {
        udelay(1);
        if (SCL_Read()) {
            return TRUE;    // SCL High
        }
    }

    return FALSE;   // SCL Low
}

/*===========================================================================
    FUNCTION  PRESS_GPIO_START_CONDITION
===========================================================================*/
static boolean press_gpio_start_condition(void)
{
    // In : SCL In (Pull-UP), SDA In (Pull-UP)
    // Out: SCL Out (Drive-Low), SDA Out (Drive-Low)

    // Bus free Check
    if (!SCL_IsHigh()) {    // SCL Low
        LOGE("%s(%d) fault (SCL Low)\n", __func__, __LINE__);
        return FALSE;
    }

    // SDA Low (Drive-Low)
    SDA_Low();
    Wait(5);

    // SCL Low (Drive-Low)
    SCL_Low();
    Wait(2);
    return TRUE;
}

/*===========================================================================
    FUNCTION  PRESS_GPIO_STOP_CONDITION
===========================================================================*/
static boolean press_gpio_stop_condition(void)
{
    // In : SCL Out (Drive-Low), SDA In/Out
    // Out: SCL In (Pull-UP), SDA In (Pull-UP)

    // SCL Low (Drive-Low)
    SCL_Low();
    Wait(2);

    // SDA Low (Drive-Low)
    SDA_Low();
    Wait(2);

    // SCL High (Pull-UP)
    SCL_High();

    // SCL High wait
    if (!SCL_IsHigh()) {    // SCL Low
        LOGE("%s(%d) fault (SCL Low)\n", __func__, __LINE__);
    }
    Wait(3);

    // SDA HIGH (Pull-UP)
    SDA_High();
    Wait(2);
    return TRUE;
}

/*===========================================================================
    FUNCTION  PRESS_GPIO_RESTART_CONDITION
===========================================================================*/
static boolean press_gpio_restart_condition(void)
{
    // In : SCL Out (Drive-Low), SDA Out (Drive-Low)
    // Out: SCL Out (Drive-Low), SDA Out (Drive-Low)

    // SDA High (Pull-UP)
    SDA_High();
    Wait(2);

    // SCL High (Pull-UP)
    SCL_High();

    // SCL High Check
    if (!SCL_IsHigh()) {    // SCL Low
        LOGE("%s(%d) fault (SCL Low)\n", __func__, __LINE__);
        return FALSE;
    }
    Wait(3);

    // SDA Low (Drive-Low)
    SDA_Low();
    Wait(3);

    // SCL Low (Drive Low)
    SCL_Low();
    Wait(3);

    return TRUE;
}

/*===========================================================================
    FUNCTION  PRESS_GPIO_CHECK_ACK
===========================================================================*/
static boolean press_gpio_check_ack(void)
{
    int sda = 0;

    // In : SCL Out (Drive-Low), SDA In (Pull-UP)
    // Out: SCL Out (Drive-Low), SDA In (Pull-UP)

    // SCL High (Pull-UP)
    SCL_High();

    // SCL High Check
    if (!SCL_IsHigh()) {    // SCL Low
        LOGE("%s(%d) fault (SCL Low)\n", __func__, __LINE__);
        return FALSE;
    }

    // SDA Signal Read
    sda = SDA_Read();
    Wait(3);

    // SCL Low (Drive-Low)
    SCL_Low();
    Wait(3);
 
    // response check
    if (sda) {
        LOGE("%s(%d) fault (NACK Receive) sda=%d\n", __func__, __LINE__, sda);
        return FALSE;       // Nack recv
    }

    return TRUE;        // Ack Recv
}

/*===========================================================================
    FUNCTION  PRESS_GPIO_SEND_NACK
===========================================================================*/
static boolean press_gpio_send_ack(uint16_t len)
{
    // In : SCL Out (Drive-Low), SDA In/Out
    // Out: SCL Out (Drive-Low), SDA Out (Drive-Low)

    if (!len) {     // Nack
        // SDA High (Pull-UP)
        SDA_High();
        Wait(2);
    } else {        // Ack
        // SDA Low (Drive-Low)
        SDA_Low();
        Wait(2);
    }

    // SCL High (Pull-UP)
    SCL_High();
    Wait(5);

    // SCL Low (Drive-Low)
    SCL_Low();
    Wait(2);

    // SDA Low (Drive-Low)
    SDA_Low();
    Wait(2);

    return TRUE;
}

/*===========================================================================
    FUNCTION  PRESS_GPIO_SEND_BYTE
===========================================================================*/
static boolean press_gpio_send_byte(uint8_t val)
{
    int     dir = 0;
    uint8_t mask = 0x80;

    // In : SCL Out (Drive-Low), SDA In/Out
    // Out: SCL Out (Drive-Low), SDA In (Pull-UP)

    // MSB Bit Output
    dir = val & mask;
    if (dir) {
        // SDA High (Pull-UP)
        SDA_High();
    } else {
        // SDA Low (Drive-Low)
        SDA_Low();
    }
    Wait(2);

    // SCL High (Pull-UP)
    SCL_High();

    // SCL High Check
    if (!SCL_IsHigh()) {    // SCL Low
        LOGE("%s(%d) fault (SCL Low)\n", __func__, __LINE__);
        return FALSE;
    }
    Wait(3);

    // SCL Low (Drive-Low)
    SCL_Low();
    Wait(3);

    // 7bit output
    mask >>= 1;
    while (mask) {
        // SDA 1Bit out
        dir = val & mask;
        if (dir) {
            // SDA High (Pull-UP)
            SDA_High();
        } else {
            // SDA Low (Drive-Low)
            SDA_Low();
        }
        Wait(2);

        // SCL High (Pull-UP) 
        SCL_High();
        Wait(4);

        // SCL Low (Drive-Low)
        SCL_Low();
        Wait(2);

        mask >>= 1;
    }

    // SDA High (Pull-UP)
    SDA_High();
    Wait(4);

    return TRUE;
}

/*===========================================================================
    FUNCTION  PRESS_GPIO_RECV_BYTE
===========================================================================*/
static boolean press_gpio_recv_byte(uint8_t *val)
{
    int     sda = 0;
    uint8_t mask = 0x80;
    uint8_t data = 0x00;

    // In : SCL Out (Drive-Low), SDA Out (Drive-Low)
    // Out: SCL Out (Drive-Low), SDA In (Pull-UP)

    // SDA Hige (Pull-UP)
    SDA_High();
    Wait(2);

    // SCL High (Pull-UP)
    SCL_High();

    // SCL High Check
    if (!SCL_IsHigh()) {    // SCL Low
        LOGE("%s(%d) fault (SCL Low)\n", __func__, __LINE__);
        return FALSE;
    }
    Wait(3);

    // SDA in
    sda = SDA_Read();
    if (sda) {
        data |= mask;
    }

    // SCL Low (Drive-Low)
    SCL_Low();
    Wait(3);

    // 7bit output
    mask >>= 1;
    while (mask) {
        // SCL High (Pull-UP)
        SCL_High();
        Wait(2);

        // SDA in
        sda = SDA_Read();
        if (sda) {
          data |= mask;
        }
        Wait(2);

        // SCL Low (Drive-Low)
        SCL_Low();
        Wait(4);

        mask >>= 1;
    }
    // Done, SCL is in LOW.
    *val = data;

    Wait(2);

    return TRUE;
}

#ifdef CY_USE_TOUCH_PRESS_IC_DUMMY_RECV
/*===========================================================================
    FUNCTION  PRESS_GPIO_DUMMY_RECV
===========================================================================*/
static void press_gpio_dummy_recv(uint8_t slave)
{
    int      rc = TRUE;
    uint8_t  readBuf = 0x00;

    // 1.SEND - Slave Address again(Touch Press Device)
    if (!(rc = press_gpio_send_byte(slave | 0x01))) {
        LOGE("%s(%d) fault (send_byte Error)\n", __func__, __LINE__);
    }

    // Chk Acknowledge
    if (!(rc = press_gpio_check_ack())) {
        LOGE("%s(%d) fault (check_ack Error)\n", __func__, __LINE__);
    }

    // 2.RCV - Data
    if (!(rc = press_gpio_recv_byte(&readBuf))) {
        LOGE("%s(%d) fault (recv_byte Error)\n", __func__, __LINE__);
    }

    // 3.Send Ack
    if (!(rc = press_gpio_send_ack(0))) {
        LOGE("%s(%d) fault (send_ack Error)\n", __func__, __LINE__);
    }

    // 4.Restart Condition
    if (!(rc = press_gpio_restart_condition())) {
        LOGE("%s(%d) fault (restart_condition Error)\n", __func__, __LINE__);
    }

    return;
}
#endif

/*============================================================================
                        EXTERNAL API DEFINITIONS
============================================================================*/

/*===========================================================================
    FUNCTION      PRESS_GPIOI2C_WRITE
===========================================================================*/
int press_gpioi2c_write(struct PressI2CCmdType *pI2CCmd)
{
    int      rc = TRUE;
    int      idx = 0;
    char     tszDump[512] = "";
    uint8_t  *pd = NULL;
    uint8_t  slave = 0;
    uint16_t len = 0;

    _DEBUG_LEVEL_ = pI2CCmd->tsdebug;
    // 1.Start Condition
    if (!(rc = press_gpio_start_condition())) {
        LOGE("%s(%d) fault (start_condition Error)\n", __func__, __LINE__);
        goto fault;
    }

    slave = pI2CCmd->slave_addr << 1;
#ifdef CY_USE_TOUCH_PRESS_IC_DUMMY_RECV
    // Dummy Recv
    press_gpio_dummy_recv(slave);
#endif

    // 2.SEND - Slave Address(Touch Press Device)
    if (!(rc = press_gpio_send_byte(slave))) {
        LOGE("%s(%d) fault (send_byte Error)\n", __func__, __LINE__);
        goto fault;
    }

    // Chk Acknowledge
    if (!(rc = press_gpio_check_ack())) {
        LOGE("%s(%d) fault (check_ack Error)\n", __func__, __LINE__);
        goto fault;
    }

    // 3.SEND - Data
    for (len = pI2CCmd->wlen, pd = pI2CCmd->pwdata; len > 0; --len, ++pd) {
        if (!(rc = press_gpio_send_byte(*pd))) {
            LOGE("%s(%d) fault (send_byte Error)\n", __func__, __LINE__);
            goto fault;
        }

        // Chk Acknowledge
        if (!(rc = press_gpio_check_ack())) {
            LOGE("%s(%d) fault (check_ack Error)\n", __func__, __LINE__);
            goto fault;
        }

        idx += sprintf(&tszDump[idx], "%02X ", *pd);
    }

    // 4.Stop Condition
    press_gpio_stop_condition();
    goto exit;

fault :
    press_gpio_stop_condition();

exit :
    if (_DEBUG_LEVEL_ >= 3) {
        pr_info("%s(%d) send data (%s) dump [%s]\n", __func__, __LINE__, rc ? "Ok" : "Error", tszDump);
    }

    return rc ? RET_OK : RET_ERR;
}

/*===========================================================================
    FUNCTION      PRESS_GPIOI2C_READ
===========================================================================*/
int press_gpioi2c_read(struct PressI2CCmdType *pI2CCmd)
{
    int      rc = TRUE;
    int      idx = 0;
    char     tszDump[512] = "";
    uint8_t  *pd = NULL;
    uint8_t  slave = 0;
    uint16_t len = 0;

    _DEBUG_LEVEL_ = pI2CCmd->tsdebug;
    // 1.Start Condition
    if (!(rc = press_gpio_start_condition())) {
        LOGE("%s(%d) fault (start_condition Error)\n", __func__, __LINE__);
        goto fault;
    }

    slave = pI2CCmd->slave_addr << 1;
#ifdef CY_USE_TOUCH_PRESS_IC_DUMMY_RECV
    // Dummy Recv
    press_gpio_dummy_recv(slave);
#endif

    // 2. SEND - Slave Address(Touch Press Device)
    if (!(rc = press_gpio_send_byte(slave))) {
        LOGE("%s(%d) fault (send_byte Error)\n", __func__, __LINE__);
        goto fault;
    }

    // Chk Acknowledge
    if (!(rc = press_gpio_check_ack())) {
        LOGE("%s(%d) fault (check_ack Error)\n", __func__, __LINE__);
        goto fault;
    } 

    // 3.SEND - Data
    for (len = pI2CCmd->wlen, pd = pI2CCmd->pwdata; len > 0; --len, ++pd) {
        if (!(rc = press_gpio_send_byte(*pd))) {
            LOGE("%s(%d) fault (send_byte Error)\n", __func__, __LINE__);
            goto fault;
        }

        // Chk Acknowledge
        if (!(rc = press_gpio_check_ack())) {
            LOGE("%s(%d) fault (check_ack Error)\n", __func__, __LINE__);
            goto fault;
        }
    }

    // 4.Restart Condition
    if (!(rc = press_gpio_restart_condition())) {
        LOGE("%s(%d) fault (restart_condition Error)\n", __func__, __LINE__);
        goto fault;
    }

    // 5.SEND - Slave Address again(Touch Press Device)
    if (!(rc = press_gpio_send_byte(slave | 0x01))) {
        LOGE("%s(%d) fault (send_byte Error)\n", __func__, __LINE__);
        goto fault;
    }

    // Chk Acknowledge
    if (!(rc = press_gpio_check_ack())) {
        LOGE("%s(%d) fault (check_ack Error)\n", __func__, __LINE__);
        goto fault;
    }

    // 6.RCV - Data
    for (len = pI2CCmd->rlen, pd = pI2CCmd->prdata; len > 0; --len, ++pd) {
        if (!(rc = press_gpio_recv_byte(pd))) {
            LOGE("%s(%d) fault (recv_byte Error)\n", __func__, __LINE__);
            goto fault;
        }

        // Send Ack
        if (!(rc = press_gpio_send_ack(len - 1))) {
            LOGE("%s(%d) fault (send_ack Error)\n", __func__, __LINE__);
            goto fault;
        }

        idx += sprintf(&tszDump[idx], "%02X ", *pd);
    }

    // 7.Stop Condition
    press_gpio_stop_condition();

    goto exit;

fault :
    press_gpio_stop_condition();

exit :
    if (_DEBUG_LEVEL_ >= 3) {
        pr_info("%s(%d) receive data (%s) dump [%s]\n", __func__, __LINE__, rc ? "Ok" : "Error", tszDump);
    }

    return rc ? RET_OK : RET_ERR;
}

/*===========================================================================
    FUNCTION      PRESS_I2C_READ
===========================================================================*/
int press_i2c_read(struct i2c_client* client, struct PressI2CCmdType* pI2CCmd)
{
    int            rc = RET_OK;
    int            i = 0;
    struct i2c_msg msg[2];

    unsigned char  read_data[256] = "";
    unsigned char  req_data[256] = "";

    _DEBUG_LEVEL_ = pI2CCmd->tsdebug;
    if (!client->adapter) {
        LOGE("%s(%d) fault (Adapter Error)\n", __func__, __LINE__);
        return -ENODEV;
    }

    msg[0].addr = client->addr;
    msg[0].flags = 0;
    msg[0].len = pI2CCmd->wlen;
    for (i = 0; i < pI2CCmd->wlen; i++) {
        LOGI("%s(%d) I2C Transfer_normal 0x%02x \n", __func__, __LINE__, pI2CCmd->pwdata[i]);
        req_data[i] = pI2CCmd->pwdata[i];
    }
    msg[0].buf = req_data;

    msg[1].addr = client->addr;
    msg[1].flags = I2C_M_RD;
    msg[1].len = pI2CCmd->rlen;
    msg[1].buf = read_data;

    rc = i2c_transfer(client->adapter, msg, 2);
    if (rc < 0) {
        LOGE("%s(%d) I2C Transfer Error %d Read8:[%02x-%02x]\n",
            __func__, __LINE__, rc, msg[1].addr, msg[1].len);
        return -EIO;
    }

    for (i = 0; i < pI2CCmd->rlen; i++) {
        pI2CCmd->prdata[i] = read_data[i];
    }

    LOGI("%s(%d) I2C Transfer OK %d Read: 0x%02x\n",
        __func__, __LINE__, rc, pI2CCmd->prdata[0]);

    return RET_OK;
}

/*===========================================================================
    FUNCTION      PRESS_I2C_WRITE
===========================================================================*/
int press_i2c_write(struct i2c_client* client, struct PressI2CCmdType* pI2CCmd)
{
    int            rc = RET_OK;
    int            i = 0;
    struct i2c_msg msg;
    unsigned char  req_data[256] = "";

    _DEBUG_LEVEL_ = pI2CCmd->tsdebug;
    if (!client->adapter) {
        LOGE("%s(%d) fault (Adapter Error)\n", __func__, __LINE__);
        return -ENODEV;
    }

    msg.addr = client->addr;
    msg.flags = 0;
    msg.len = pI2CCmd->wlen;  // add address bytes
    for (i = 0; i < pI2CCmd->wlen; i++) {
        LOGI("%s(%d) I2C Transfer_normal 0x%02x \n", __func__, __LINE__, pI2CCmd->pwdata[i]);
        req_data[i] = pI2CCmd->pwdata[i];
    }
    msg.buf = req_data;

    rc = i2c_transfer(client->adapter, &msg, 1);
    if (rc < 0) {
        LOGE("%s(%d) I2C Transfer Error %d write:[%02x-%02x]\n",
            __func__, __LINE__, rc, msg.addr, msg.len);
        return -EIO;
    }

    LOGI("%s(%d) I2C Transfer OK %d write \n", __func__, __LINE__, rc);

    return RET_OK;
}
