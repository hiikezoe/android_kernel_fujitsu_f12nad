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

                ImmVibe GPIO Driver Source File

============================================================================*/

/*============================================================================
                        INCLUDE FILES
============================================================================*/
#include <linux/kernel.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <mach/gpio.h>
#include <asm/mach-types.h>
#include "ImmVibe_gpio.h"

/*============================================================================
                        INTERNAL FEATURES
============================================================================*/

/*============================================================================
                        CONSTANTS
============================================================================*/
#define TRUE                    1
#define FALSE                   0

#define _DUMP                   1

#if defined(CONFIG_MACH_F12ACE)
#define SCL                     2          // I2C SCL GPIO No
#define SDA                     3          // I2C SDA GPIO No
#else
#define SCL                     176        // I2C SCL GPIO No
#define SDA                     175        // I2C SDA GPIO No
#endif
#if 0
#define SCL_Low()               gpio_configure(SCL, GPIOF_DRIVE_OUTPUT)
#define SCL_High()              gpio_configure(SCL, GPIOF_INPUT)
#define SDA_Low()               gpio_configure(SDA, GPIOF_DRIVE_OUTPUT)
#define SDA_High()              gpio_configure(SDA, GPIOF_INPUT)
#else
#define SCL_Low()               gpio_direction_output(SCL, 0)
#define SCL_High()              gpio_direction_input(SCL)
#define SDA_Low()               gpio_direction_output(SDA, 0)
#define SDA_High()              gpio_direction_input(SDA)
#endif

#define SDA_Read()              gpio_get_value(SDA)
#define SCL_Read()              gpio_get_value(SCL)

#define LOGE(fmt, args...)      printk(KERN_ERR "###ImmVibe I2C: " fmt, ##args)
#if DEBUG
#define LOGD(fmt, args...)      printk(KERN_DEBUG "###ImmVibe I2C: " fmt, ##args)
#define LOGI(fmt, args...)      printk(KERN_INFO "###ImmVibe I2C: " fmt, ##args)
#else
#define LOGD(fmt, args...)      do {} while (0)
#define LOGI(fmt, args...)      do {} while (0)
#endif
/*============================================================================
                        EXTERNAL VARIABLES DEFINITIONS
============================================================================*/
int _IMMVIBE_I2C_LOG_  = 0;
int _IMMVIBE_I2C_WAIT_ = 12;

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
    for (cnt = 1000 * _IMMVIBE_I2C_WAIT_; cnt; cnt--) {
// 2012.04.11 del.    udelay(3);
        if (SCL_Read()) {
            return TRUE;    // SCL High
        }
    }

    return FALSE;   // SCL Low
}

/*===========================================================================
    FUNCTION  IMMVIBE_GPIO_START_CONDITION
===========================================================================*/
static boolean immvibe_gpio_start_condition(void)
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
    FUNCTION  IMMVIBE_GPIO_STOP_CONDITION
===========================================================================*/
static boolean immvibe_gpio_stop_condition(void)
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
    FUNCTION  IMMVIBE_GPIO_RESTART_CONDITION
===========================================================================*/
static boolean immvibe_gpio_restart_condition(void)
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
    FUNCTION  IMMVIBE_GPIO_CHECK_ACK
===========================================================================*/
static boolean immvibe_gpio_check_ack(void)
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
    FUNCTION  IMMVIBE_GPIO_SEND_NACK
===========================================================================*/
static boolean immvibe_gpio_send_ack(uint16_t len)
{
    // In : SCL Out (Drive-Low), SDA In/Out
    // Out: SCL Out (Drive-Low), SDA Out (Drive-Low)

//    printk("$$$(%d):%s $$$\n", __LINE__, __func__);
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
    FUNCTION  IMMVIBE_GPIO_SEND_BYTE
===========================================================================*/
static boolean immvibe_gpio_send_byte(uint8_t val)
{
    int     dir = 0;
    uint8_t mask = 0x80;

    // In : SCL Out (Drive-Low), SDA In/Out
    // Out: SCL Out (Drive-Low), SDA In (Pull-UP)

//    printk("$$$(%d):%s val=%02X$$$\n", __LINE__, __func__, val);
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
    FUNCTION  IMMVIBE_GPIO_RECV_BYTE
===========================================================================*/
static boolean immvibe_gpio_recv_byte(uint8_t *val)
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
//    printk("$$$(%d):%s data=%02X(%d)$$$\n", __LINE__, __func__, data, data);

    Wait(2);

    return TRUE;
}

/*============================================================================
                        EXTERNAL API DEFINITIONS
============================================================================*/

/*===========================================================================
    FUNCTION      IMMVIBE_GPIOI2C_WRITE
===========================================================================*/
int immvibe_gpioi2c_write(struct ImmVibeI2CCmdType *pI2CCmd)
{
    int      rc = TRUE;
    int      idx = 0;
    char     tszDump[256] = "";
    uint8_t  *pd = NULL;
    uint8_t  slave = 0;
    uint16_t len = 0;
    uint16_t outbytes = 0;

//    printk("$$$(%d):%s $$$\n", __LINE__, __func__);
    // 1.Start Condition
    if (!(rc = immvibe_gpio_start_condition())) {
        LOGE("%s(%d) fault (start_condition Error)\n", __func__, __LINE__);
        goto fault;
    }

    slave = pI2CCmd->slave_addr << 1;

    // 2.SEND - Slave Address(Touch ImmVibe Device)
    if (!(rc = immvibe_gpio_send_byte(slave))) {
        LOGE("%s(%d) fault (send_byte Error)\n", __func__, __LINE__);
        goto fault;
    }

    // Chk Acknowledge
    if (!(rc = immvibe_gpio_check_ack())) {
        LOGE("%s(%d) fault (check_ack Error)\n", __func__, __LINE__);
        goto fault;
    }

//    LOGE("%s(%d) 3.SEND - Data loop[%d]\n", __func__, __LINE__, pI2CCmd->wlen);
    // 3.SEND - Data
    for (len = pI2CCmd->wlen, pd = pI2CCmd->pwdata; len > 0; --len, ++pd) {
        if (!(rc = immvibe_gpio_send_byte(*pd))) {
            LOGE("%s(%d) fault (send_byte Error)\n", __func__, __LINE__);
            goto fault;
        }

        // Chk Acknowledge
        if (!(rc = immvibe_gpio_check_ack())) {
            LOGE("%s(%d) fault (check_ack Error)\n", __func__, __LINE__);
            goto fault;
        }
        outbytes++;
    }

    // 4.Stop Condition
    immvibe_gpio_stop_condition();
    goto exit;

fault : ;
    immvibe_gpio_stop_condition();

exit : ;
#if _DUMP
    //for (len = 1, pd = pI2CCmd->pwdata + 1; len < pI2CCmd->wlen && len < 12; ++len, ++pd) {
    for (len = pI2CCmd->wlen, pd = pI2CCmd->pwdata; len > 0; --len, ++pd) {
        idx += sprintf(&tszDump[idx], "%02X ", *pd);
    }
#endif
    if (_IMMVIBE_I2C_LOG_ != 0 || !rc) {
        LOGI("%s(%d) C:%02X T:%s(len=%d) (%s)\n",
            __func__, __LINE__, *pI2CCmd->pwdata, tszDump, pI2CCmd->wlen, rc ? "Ok" : "Error");
    }
    pI2CCmd->outbytes = outbytes;
    return rc ? RET_OK : RET_ERR;
}

/*===========================================================================
    FUNCTION      IMMVIBE_GPIOI2C_READ
===========================================================================*/
int immvibe_gpioi2c_read(struct ImmVibeI2CCmdType *pI2CCmd)
{
    int      rc = TRUE;
    char     tszDump[256] = "";
    uint8_t  *pd = NULL;
    uint8_t  slave = 0;
    uint16_t len = 0;

//    printk("$$(%d):%s start.$$$\n", __LINE__, __func__);

    // 1.Start Condition
    if (!(rc = immvibe_gpio_start_condition())) {
        LOGE("%s(%d) fault (start_condition Error)\n", __func__, __LINE__);
        goto fault;
    }

//    printk("$$$%s(%d) slave_addr=0x%02X(%d)\n", __func__, __LINE__, pI2CCmd->slave_addr, pI2CCmd->slave_addr);
    slave = pI2CCmd->slave_addr << 1;

    // 2. SEND - Slave Address(Touch ImmVibe Device)
    if (!(rc = immvibe_gpio_send_byte(slave))) {
        LOGE("%s(%d) fault (send_byte Error)\n", __func__, __LINE__);
        goto fault;
    }

    // Chk Acknowledge
    if (!(rc = immvibe_gpio_check_ack())) {
        LOGE("%s(%d) fault (check_ack Error)\n", __func__, __LINE__);
        goto fault;
    } 

    // 3.SEND - Data
    for (len = pI2CCmd->wlen, pd = pI2CCmd->pwdata; len > 0; --len, ++pd) {
        if (!(rc = immvibe_gpio_send_byte(*pd))) {
            LOGE("%s(%d) fault (send_byte Error)\n", __func__, __LINE__);
            goto fault;
        }

        // Chk Acknowledge
        if (!(rc = immvibe_gpio_check_ack())) {
            LOGE("%s(%d) fault (check_ack Error)\n", __func__, __LINE__);
            goto fault;
        }
    }

    // 4.Restart Condition
    if (!(rc = immvibe_gpio_restart_condition())) {
        LOGE("%s(%d) fault (restart_condition Error)\n", __func__, __LINE__);
        goto fault;
    }

    // 5.SEND - Slave Address again(Touch ImmVibe Device)
    if (!(rc = immvibe_gpio_send_byte(slave | 0x01))) {
        LOGE("%s(%d) fault (send_byte Error)\n", __func__, __LINE__);
        goto fault;
    }

    // Chk Acknowledge
    if (!(rc = immvibe_gpio_check_ack())) {
        LOGE("%s(%d) fault (check_ack Error)\n", __func__, __LINE__);
        goto fault;
    }

    // 6.RCV - Data
    for (len = pI2CCmd->rlen, pd = pI2CCmd->prdata; len > 0; --len, ++pd) {
        if (!(rc = immvibe_gpio_recv_byte(pd))) {
            LOGE("%s(%d) fault (recv_byte Error)\n", __func__, __LINE__);
            goto fault;
        }

        // Send Ack
        if (!(rc = immvibe_gpio_send_ack(len - 1))) {
            LOGE("%s(%d) fault (send_ack Error)\n", __func__, __LINE__);
            goto fault;
        }
    }

    // 7.Stop Condition
    immvibe_gpio_stop_condition();

    goto exit;

fault : ;
    immvibe_gpio_stop_condition();
    sprintf(tszDump, "%02X ",*pI2CCmd->prdata);

exit : ;
    if (_IMMVIBE_I2C_LOG_ != 0 || !rc) {
        LOGI("%s(%d)  C:%02X %s (%s)\n", 
             __func__, __LINE__, *pI2CCmd->prdata, tszDump, rc ? "Ok" : "Error");
    }

//    printk("$$$(%d):%s end.$$$\n", __LINE__, __func__);
    return rc ? RET_OK : RET_ERR;
}

/*===========================================================================
    FUNCTION      IMMVIBE_GPIOI2C_WRITE_ISP
===========================================================================*/
int immvibe_gpioi2c_write_isp(struct ImmVibeI2CCmdTypeIsp *pI2CCmd)
{
    int      rc = TRUE;
    int      i = 0;
    char     req_data[12] = "";     // Max send buffer
    char     *req_buf = NULL;
    uint8_t  *pd = NULL;
    uint8_t  slave = 0;
    uint16_t len = 0;

    // 1.Start Condition
    if (!(rc = immvibe_gpio_start_condition())) {
        LOGE("%s(%d) fault (start_condition Error)\n", __func__, __LINE__);
        goto fault;
    }

    slave = pI2CCmd->slave_addr << 1;

    // 2.SEND - Slave Address(Touch ImmVibe Device)
    if (!(rc = immvibe_gpio_send_byte(slave))) {
        LOGE("%s(%d) fault (send_byte Error)\n", __func__, __LINE__);
        goto fault;
    }

    // Chk Acknowledge
    if (!(rc = immvibe_gpio_check_ack())) {
        LOGE("%s(%d) fault (check_ack Error)\n", __func__, __LINE__);
        goto fault;
    }

    // for M6Mo Parameter
    req_data[0] = 4 + pI2CCmd->wlen;
    req_data[1] = 0x02;     // 0x02 is Write cmd.
    req_data[2] = pI2CCmd->category;
    req_data[3] = pI2CCmd->byte;
    req_buf = &req_data[4];
    LOGI("%s(%d) s:%02x [c:%02x b:%02x]\n",
        __func__, __LINE__, slave, pI2CCmd->category, pI2CCmd->byte);

    for (i = 0; i < pI2CCmd->wlen; i++) {
        req_buf[i] = pI2CCmd->pwdata[i];
        LOGI("%s(%d) - data:%02x\n", __func__, __LINE__, req_buf[i]);
    }
    pI2CCmd->wlen += 4;     // Length is no send. because +3byte.

    // 3.SEND - Data
    for (len = pI2CCmd->wlen, pd = req_data; len > 0; --len, ++pd) {
        if (!(rc = immvibe_gpio_send_byte(*pd))) {
            LOGE("%s(%d) fault (send_byte Error)\n", __func__, __LINE__);
            goto fault;
        }

        // Chk Acknowledge
        if (!(rc = immvibe_gpio_check_ack())) {
            LOGE("%s(%d) fault (check_ack Error)\n", __func__, __LINE__);
            goto fault;
        }
    }

    // 4.Stop Condition
    immvibe_gpio_stop_condition();
    goto exit;

fault : ;
    immvibe_gpio_stop_condition();

exit : ;
    if (_IMMVIBE_I2C_LOG_ != 0 || !rc) {
        LOGI("%s(%d) C:%02X (%s)\n",
               __func__, __LINE__, *pI2CCmd->pwdata, rc ? "Ok" : "Error");
    }

    return rc ? RET_OK : RET_ERR;
}

/*===========================================================================
    FUNCTION      IMMVIBE_GPIOI2C_READ_ISP
===========================================================================*/
int immvibe_gpioi2c_read_isp(struct ImmVibeI2CCmdTypeIsp *pI2CCmd)
{
    int      rc = TRUE;
    int      i = 0;
    char     req_data[12] = "";
    char     read_data[256] = "";
    uint8_t  *pd = NULL;
    uint8_t  slave = 0;
    uint16_t len = 0;

    // 1.Start Condition
    if (!(rc = immvibe_gpio_start_condition())) {
        LOGE("%s(%d) fault (start_condition Error)\n", __func__, __LINE__);
        goto fault;
    }

    slave = pI2CCmd->slave_addr << 1;
    
    // 2. SEND - Slave Address(Touch ImmVibe Device)
    if (!(rc = immvibe_gpio_send_byte(slave))) {
        LOGE("%s(%d) fault (send_byte Error)\n", __func__, __LINE__);
        goto fault;
    }

    // Chk Acknowledge
    if (!(rc = immvibe_gpio_check_ack())) {
        LOGE("%s(%d) fault (check_ack Error)\n", __func__, __LINE__);
        goto fault;
    }

    // for M6Mo Parameter
    req_data[0] = 5;
    req_data[1] = 0x01;         // 0x01 is Read cmd.
    req_data[2] = pI2CCmd->category;
    req_data[3] = pI2CCmd->byte;
    req_data[4] = pI2CCmd->rlen;
    LOGI("%s(%d) s:%02x [c:%02x b:%02x] l:%d\n",
        __func__, __LINE__, slave, pI2CCmd->category, pI2CCmd->byte, pI2CCmd->rlen);
    pI2CCmd->wlen = 5;          // Length is no send. because +4byte.

    // 3.SEND - Data
    for (len = pI2CCmd->wlen, pd = req_data; len > 0; --len, ++pd) {
        if (!(rc = immvibe_gpio_send_byte(*pd))) {
            LOGE("%s(%d) fault (send_byte Error)\n", __func__, __LINE__);
            goto fault;
        }

        // Chk Acknowledge
        if (!(rc = immvibe_gpio_check_ack())) {
            LOGE("%s(%d) fault (check_ack Error)\n", __func__, __LINE__);
            goto fault;
        }
    }

    // 4-1.Stop Condition
    if (!(rc = immvibe_gpio_stop_condition())) {
        LOGE("%s(%d) fault (stop_condition Error)\n", __func__, __LINE__);
        goto fault;
    }
    Wait(10);
    // 4-2.Start Condition
    if (!(rc = immvibe_gpio_start_condition())) {
        LOGE("%s(%d) fault (start_condition Error)\n", __func__, __LINE__);
        goto fault;
    }

    // 5.SEND - Slave Address again(Touch ImmVibe Device)
    if (!(rc = immvibe_gpio_send_byte(slave | 0x01))) {
        LOGE("%s(%d) fault (send_byte Error)\n", __func__, __LINE__);
        goto fault;
    }

    // Chk Acknowledge
    if (!(rc = immvibe_gpio_check_ack())) {
        LOGE("%s(%d) fault (check_ack Error)\n", __func__, __LINE__);
        goto fault;
    }

    // 6.RCV - Data
    for (len = pI2CCmd->rlen + 1, pd = read_data; len > 0; --len, ++pd) {
        if (!(rc = immvibe_gpio_recv_byte(pd))) {
            LOGE("%s(%d) fault (recv_byte Error)\n", __func__, __LINE__);
            goto fault;
        }

        // Send Ack
        if (!(rc = immvibe_gpio_send_ack(len - 1))) {
            LOGE("%s(%d) fault (send_ack Error)\n", __func__, __LINE__);
            goto fault;
        }
    }

    // 7.Stop Condition
    immvibe_gpio_stop_condition();

    for (i = 0; i < pI2CCmd->rlen; i++) {
        pI2CCmd->prdata[i] = read_data[i + 1];
        LOGI("%s(%d) r:%d [c:%02x b:%02x]0x%02x\n",
            __func__, __LINE__, rc, pI2CCmd->category, pI2CCmd->byte, pI2CCmd->prdata[i]);
    }
    goto exit;

fault : ;
    immvibe_gpio_stop_condition();

exit : ;
    if (_IMMVIBE_I2C_LOG_ != 0 || !rc) {
        LOGI("%s(%d)  C:%02X (%s)\n", 
             __func__, __LINE__, *req_data, rc ? "Ok" : "Error");
    }

    return rc ? RET_OK : RET_ERR;
}

/*===========================================================================
    FUNCTION      IMMVIBE_I2C_READ
===========================================================================*/
int immvibe_i2c_read(struct i2c_client* client, struct ImmVibeI2CCmdType* pI2CCmd)
{
    int            rc = RET_OK;
    int            i = 0;
    struct i2c_msg msg[2];

    unsigned char  read_data[256] = "";
    unsigned char  req_data[256] = "";

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
    FUNCTION      IMMVIBE_I2C_WRITE
===========================================================================*/
int immvibe_i2c_write(struct i2c_client* client, struct ImmVibeI2CCmdType* pI2CCmd)
{
    int            rc = RET_OK;
    int            i = 0;
    struct i2c_msg msg;
    unsigned char  req_data[256] = "";

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
        LOGE("%s(%d) I2C Transfer Error %d write8:[%02x-%02x]\n",
            __func__, __LINE__, rc, msg.addr, msg.len);
        return -EIO;
    }

    LOGI("%s(%d) I2C Transfer OK %d write \n", __func__, __LINE__, rc);

    return RET_OK;
}

/*===========================================================================
    FUNCTION      IMMVIBE_I2C_READ_ISP
===========================================================================*/
int immvibe_i2c_read_isp(struct i2c_client* client, struct ImmVibeI2CCmdTypeIsp* pI2CCmd)
{
    int            rc = RET_OK;
    int            i = 0;
    struct i2c_msg msg[2];
    unsigned char  read_data[256] = "";
    unsigned char  req_data[5] = "";

    if (!client->adapter) {
        LOGE("%s(%d) fault (Adapter Error)\n", __func__, __LINE__);
        return -ENODEV;
    }

    req_data[0] = 5;
    req_data[1] = 0x01;
    req_data[2] = pI2CCmd->category;
    req_data[3] = pI2CCmd->byte;
    req_data[4] = pI2CCmd->rlen;

    msg[0].addr = client->addr;
    msg[0].flags = 0;
    msg[0].len = 5;
    msg[0].buf = req_data;
    
    msg[1].addr = client->addr;
    msg[1].flags = I2C_M_RD;
    msg[1].len = pI2CCmd->rlen + 1;
    msg[1].buf = read_data;

    rc = i2c_transfer(client->adapter, msg, 2);
    if (rc < 0) {
        LOGE("%s(%d) I2C Transfer Error %d Read8:[%02x:0x%x-0x%x]\n",
            __func__, __LINE__, rc, client->addr, pI2CCmd->category, pI2CCmd->byte);
        return -EIO;
    }

    for (i = 0; i < pI2CCmd->rlen; i++) {
        pI2CCmd->prdata[i] = read_data[i + 1];
        LOGI("%s(%d) r:%d [c:%02x b:%02x]0x%02x\n",
            __func__, __LINE__, rc, pI2CCmd->category, pI2CCmd->byte, pI2CCmd->prdata[i]);
    }

    LOGI("%s(%d) I2C Transfer OK %d Read:[0x%x-0x%x] %02x\n",
        __func__, __LINE__, rc, pI2CCmd->category, pI2CCmd->byte, pI2CCmd->prdata[0]);
    return RET_OK;
}

/*===========================================================================
    FUNCTION      IMMVIBE_I2C_WRITE_ISP
===========================================================================*/
int immvibe_i2c_write_isp(struct i2c_client* client, struct ImmVibeI2CCmdTypeIsp* pI2CCmd)
{
    int            rc = RET_OK;
    int            i = 0;
    struct i2c_msg msg;
    unsigned char  req_data[256] = "";

    if (!client->adapter) {
        LOGE("%s(%d) fault (Adapter Error)\n", __func__, __LINE__);
        return -ENODEV;
    }

    req_data[0] = 4 + pI2CCmd->wlen;
    req_data[1] = 0x02;
    req_data[2] = pI2CCmd->category;
    req_data[3] = pI2CCmd->byte;
    LOGI("%s(%d) s:%02x [c:%02x b:%02x]\n",
        __func__, __LINE__, client->addr, pI2CCmd->category, pI2CCmd->byte);

    for (i = 0; i < pI2CCmd->wlen; i++) {
        req_data[i + 4] = pI2CCmd->pwdata[i];
        LOGI("%s(%d) - data:%02x\n", __func__, __LINE__, req_data[i + 4]);
    }

    msg.addr = client->addr;
    msg.flags = 0;
    msg.len = 4 + pI2CCmd->wlen;        // add address bytes
    msg.buf = req_data;

    rc = i2c_transfer(client->adapter, &msg, 1);
    if (rc < 0) {
        LOGE("%s(%d) I2C Transfer Error %d write8:[%02x:0x%x-0x%x]\n",
            __func__, __LINE__, rc, client->addr, pI2CCmd->category, pI2CCmd->byte);
        return -EIO;
    }

    LOGI("%s(%d) I2C Transfer OK %d write:[0x%x-0x%x] %02x\n",
        __func__, __LINE__, rc, pI2CCmd->category, pI2CCmd->byte, req_data[4]);

    return RET_OK;
}
