/*
** =========================================================================
** File:
**     ImmVibeSPI.c
**
** Description: 
**     Device-dependent functions called by Immersion TSP API
**     to control PWM duty cycle, amp enable/disable, save IVT file, etc...
**
** Portions Copyright (c) 2008-2012 Immersion Corporation. All Rights Reserved. 
**
** This file contains Original Code and/or Modifications of Original Code 
** as defined in and that are subject to the GNU Public License v2 - 
** (the 'License'). You may not use this file except in compliance with the 
** License. You should have received a copy of the GNU General Public License 
** along with this program; if not, write to the Free Software Foundation, Inc.,
** 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA or contact 
** TouchSenseSales@immersion.com.
**
** The Original Code and all software distributed under the License are 
** distributed on an 'AS IS' basis, WITHOUT WARRANTY OF ANY KIND, EITHER 
** EXPRESS OR IMPLIED, AND IMMERSION HEREBY DISCLAIMS ALL SUCH WARRANTIES, 
** INCLUDING WITHOUT LIMITATION, ANY WARRANTIES OF MERCHANTABILITY, FITNESS 
** FOR A PARTICULAR PURPOSE, QUIET ENJOYMENT OR NON-INFRINGEMENT. Please see 
** the License for the specific language governing rights and limitations 
** under the License.
** =========================================================================
*/
/*----------------------------------------------------------------------------*/
// COPYRIGHT(C) FUJITSU LIMITED 2012
/*----------------------------------------------------------------------------*/
#include <linux/i2c.h>
#include <linux/wakelock.h>
#include <linux/io.h>
#include <linux/pwm.h>
#include <linux/gpio.h>

/* FUJITSU:2012-02-14  add start */
#include "tspdrv.h"
#include <mach/msm_iomap.h>
#include <linux/clk.h>
#include <linux/delay.h>
/* FUJITSU:2012-02-14  add end */

/* Add by ISB-GPIO-I2C on 2012.04.06 -< */
#include "ImmVibe_gpio.h"
/* Add by ISB-GPIO-I2C on 2012.04.06 -< */

#ifdef IMMVIBESPIAPI
#undef IMMVIBESPIAPI
#endif
#define IMMVIBESPIAPI static

/*
** Debug-Mode [0=OFF,1=ON]
*/
#define DEBUG 0 
/* #define DEBUG 1 */

/*
** Delay (in ms) before resending data after NAK
** (8 ms should let TS5000 consume 64 bytes before resending)
*/
#define NAK_RESEND_DELAY_MS 8


/* Add by ISB-GPIO-I2C on 2012.04.06 -< */
#define IMMVIBE_I2C_DATA_SIZE 128
/* Add by ISB-GPIO-I2C on 2012.04.06 -< */


/*
** Number of NAK resend attempts before aborting
*/
#define NAK_RESEND_ATTEMPT  3

/*
** I2C FW info
*/
typedef struct _FWINFO
{
    char cMajorVersion;
    char cMinorVersion;
    char cInterfaceType;    /* 3 = I2C */
    char cAmpType[9];       /* 1 = differential amp */
    char cAmpUsage;         /* 1 = single boost, 2 = dual boost */
    char cClockSource;      /* 2 = external 18MHz clock */
} FWINFO;
FWINFO g_FWInfo;

/*
** I2C clock value corresponding to 400kHz.
** Notes: 
**  (see drivers/i2c/busses/i2c-msm.c for original implementation)
**  This value is obtained by the following maths:
**      i2c_clk = 19200000;     (input clock)
**      target_clk = 400000;    (400kHz)
**      fs_div = ((i2c_clk / target_clk) / 2) - 3;
**      hs_div = 3;
**      I2C_CLK_400KHZ = ((hs_div & 0x7) << 8) | (fs_div & 0xff);
*/
#define I2C_CLK_400KHZ  0x315

/*
** I2C bus dev struct (copied from drivers/i2c/busses/i2c-omap.c).
*/

struct msm_i2c_dev {
    struct device      *dev;
    void __iomem       *base;       /* virtual */
    int                 irq;
    struct clk         *clk;
    struct i2c_adapter  adapter;

    spinlock_t          lock;

    struct i2c_msg      *msg;
    int                 rem;
    int                 pos;
    int                 cnt;
    int                 ret;
    bool                need_flush;
    int                 flush_cnt;
    void                *complete;
    struct wake_lock    wakelock;
};

/*
** This SPI supports only one actuator.
*/
#define NUM_ACTUATORS       1

/* FUJITSU:2012-03-23  TS5K [start] */
#define TS5K_BOARD_NAME   "ts5k"
/* FUJITSU:2012-03-23  TS5K [start] */

/*
** TS5000 Board-Type
*/
#define TI_DRV2665      1
#define BOARD_TYPE      TI_DRV2665

/*
** TS5000 Play-Command
*/
#define PLAY_CMD_2665       0x0B
#define PLAY_CMD_PIEZO_4KHZ 0x27
#define PLAY_CMD_ERM        0x29
#define PLAY_CMD_PIEZO      0x28

/*
** Number of data bytes when processing a 4kHz Piezo signal
*/
#define NUM_BYTES_4KHZ      20

/*
** Number of NAK resend attempts before aborting
*/
#define NAK_RESEND_ATTEMPT  3

/*
** Global variables
*/
static bool      g_bAmpEnabled[NUM_ACTUATORS] = { false };
static struct    i2c_client* g_pTheClient = NULL;
static struct    msm_i2c_dev* g_pI2CDev = NULL;
static VibeUInt8 g_i2cBuf[VIBE_OUTPUT_SAMPLE_SIZE+1];
static char      g_szFWVersion[VIBE_MAX_DEVICE_NAME_LENGTH];

/*
** TS5000/I2C structs and forward declarations
*/
static int   ts5k_probe(struct i2c_client* client, const struct i2c_device_id* id);
static int   ts5k_remove(struct i2c_client* client);
static void  setStandby(bool standbyState);

/* 2012.05.15 tspdrv func-chg [start]
static void  fifo_check(void);
** 2012.05.15 tspdrv func-chg [end] */
static void  reg1_init(void);	/* 2012.05.15 func-chg */

static const struct i2c_device_id ts5k_id[] =
{
    {TS5K_BOARD_NAME, 0},
    {}
};
static const unsigned short normal_i2c[] = {0x59,I2C_CLIENT_END};
static struct i2c_driver ts5k_driver =
{
   .probe        = ts5k_probe,
   .remove       = ts5k_remove,
   .id_table     = ts5k_id,
   .address_list = normal_i2c, 
   .driver =
   {
       .name = TS5K_BOARD_NAME,
   },
};

/*
** Register adress
*/
#define TS5K_0x00_ADDR 0x00
#define TS5K_0x01_ADDR 0x01
#define TS5K_0x02_ADDR 0x02
#define TS5K_0x0B_ADDR 0x0B

/*
** external val 
*/
int mc_count=0;

/* Add by ISB-GPIO-I2C on 2012.04.06 -> */
/*===========================================================================
    FUNCTION  IMMVIBE_GPIOI2C_READ
===========================================================================*/
static s32 ImmVibe_gpioi2c_read(u8 addr, u8 length, void *values)
{
    int retval = 0;
    struct ImmVibeI2CCmdType recvI2CCmd;

    recvI2CCmd.slave_addr = IMMVIBE_I2C_ADR;
    recvI2CCmd.pwdata     = (char *)&addr;
    recvI2CCmd.wlen       = 1;
    recvI2CCmd.prdata     = values;
    recvI2CCmd.rlen       = length;

    retval = immvibe_gpioi2c_read(&recvI2CCmd);
    return (retval < 0) ? retval : 0;
}

/*===========================================================================
    FUNCTION  IMMVIBE_GPIOI2C_WRITE
===========================================================================*/
static s32 ImmVibe_gpioi2c_write(u8 addr, u8 length, const void *values)
{
    int retval = 0;
    struct ImmVibeI2CCmdType sendI2CCmd;
    u8 wr_buf[IMMVIBE_I2C_DATA_SIZE] = "";
    struct msm_i2c_dev *dev = i2c_get_adapdata(g_pTheClient->adapter);

    wr_buf[0] = addr;
    memcpy(&wr_buf[1], values, length);

    sendI2CCmd.slave_addr = IMMVIBE_I2C_ADR;
    sendI2CCmd.pwdata     = (char *)wr_buf;
    sendI2CCmd.wlen       = length + 1;
    sendI2CCmd.outbytes   = 0; // initialize the Count of bytes to transfered

    retval = immvibe_gpioi2c_write(&sendI2CCmd);
    if (sendI2CCmd.outbytes > 1) {
        dev->cnt = length - sendI2CCmd.outbytes + 1; /* Count of bytes to untransfered */
    }
    else{
        dev->cnt = length;
    }

#if DEBUG == 1
    DbgOut((KERN_DEBUG "## ImmVibeSPI: ImmVibe_gpioi2c_write: remainder bytes = %d\n",
                     dev->cnt));
#endif

    return (retval < 0) ? retval : 0;
}

/*===========================================================================
    FUNCTION  WRAPER_TRANSFER
===========================================================================*/
int wraper_transfer(struct i2c_client *client, struct i2c_msg *msgs, int num)
{
    int ret = 0;
    
/* 2012.06.05 chg[start]
    if (is_use_i2c) {
** 2012.06.05 chg[end]   */
    if (is_use_i2c_tspdrv) {	/* 2012.06.05 chg */
        ret = i2c_transfer(client->adapter, msgs, num);
    } else {
        if (1 == num) {

#if DEBUG == 1
DbgOut((KERN_DEBUG "wraper_transfer: ImmVibe_gpioi2c_write() {msgs[0].buf[0]=%d, msgs[0].len=%d, num=%d}\n"
                , msgs[0].buf[0], msgs[0].len, num));
#endif

            ret = ImmVibe_gpioi2c_write((u8)(msgs[0].buf[0]), msgs[0].len - 1, msgs[0].buf + 1);
        } else {

#if DEBUG == 1
DbgOut((KERN_DEBUG "wraper_transfer: ImmVibe_gpioi2c_read() {msgs[0].buf[0]=%d, msgs[1].len=%d, num=%d}\n"
                , msgs[0].buf[0], msgs[1].len, num));
#endif

            ret = ImmVibe_gpioi2c_read((u8)(msgs[0].buf[0]), msgs[1].len, msgs[1].buf);
        }
    }
    
    return ret;
}

/* Add by ISB-GPIO-I2C on 2012.04.06 -< */

/****** ****** ****** ****** ****** ****** ****** ****** ****** ****** ******/
/*
** Called to disable amp
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_AmpDisable(VibeUInt8 nActuatorIndex)
{

#if DEBUG == 1
    DbgOut((KERN_DEBUG "ImmVibeSPI_ForceOut_AmpDisable  nActuatorIndex=%d [start]\n", nActuatorIndex));
#endif

    if (g_bAmpEnabled[nActuatorIndex]){
        g_bAmpEnabled[nActuatorIndex] = false;

#if BOARD_TYPE == TI_DRV2665
        /* DRV2665 has real amp to disable */
        schedule_timeout_interruptible(msecs_to_jiffies(13));
        setStandby(true);
//      DbgOut((KERN_DEBUG "ImmVibeSPI_ForceOut_AmpDisable [setStandby=True]\n"));
#endif

    }

#if DEBUG == 1
    DbgOut((KERN_DEBUG "ImmVibeSPI_ForceOut_AmpDisable [end]\n"));
#endif

    return VIBE_S_SUCCESS;
}

/****** ****** ****** ****** ****** ****** ****** ****** ****** ****** ******/
/*
** Called to enable amp (enable output force)
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_AmpEnable(VibeUInt8 nActuatorIndex)
{

#if DEBUG == 1
    DbgOut((KERN_DEBUG "ImmVibeSPI_ForceOut_Ampable  nActuatorIndex=%d [strat]\n", nActuatorIndex));
#endif
    
    if((!g_pI2CDev) || (!g_pI2CDev->base)) {
       return VIBE_E_FAIL;
    }

    if (!g_bAmpEnabled[nActuatorIndex]){

#if DEBUG == 1
        DbgOut((KERN_DEBUG "ImmVibeSPI_ForceOut_AmpEnable [nActuatorIndex=%d]\n", nActuatorIndex));
#endif

        g_bAmpEnabled[nActuatorIndex] = true;

#if BOARD_TYPE == TI_DRV2665
        /* No real amp to enable on TS5000 board */
        /* DRV2665 has real amp to disable */
        setStandby(false);
//      DbgOut((KERN_DEBUG "ImmVibeSPI_ForceOut_AmpEnable [setStandby=false]\n"));
#endif
    }
/** del.2012.05.11 [start] 
    fifo_check();
 ** del.2012.05.11 [end]   */
    reg1_init();	/* 2012.05.15 func-chg */

#if DEBUG == 1
    DbgOut((KERN_DEBUG "ImmVibeSPI_ForceOut_Ampable [end]\n"));
#endif

    return VIBE_S_SUCCESS;
}

/****** ****** ****** ****** ****** ****** ****** ****** ****** ****** ******/
/*
** Called to init
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_Initialize(void)
{
   struct i2c_msg msg[2];	/* initialized below */
   int nActuatorIndex;		/* Initialized below. */
   int rtn=0;

#if DEBUG == 1
   DbgOut((KERN_DEBUG "ImmVibeSPI_ForceOut_Initialize. [start]\n"));
#endif

   /* Add TS5000 driver */
   rtn=i2c_add_driver(&ts5k_driver);

#if DEBUG == 1
   DbgOut((KERN_DEBUG "ImmVibeSPI_ForceOut_Initialize. [i2c_add_driver()]\n"));
#endif

/* Board-type check[Reg=0x01 ID[3:0]] */
#if BOARD_TYPE == TI_DRV2665
    g_i2cBuf[0] = TS5K_0x01_ADDR ;
    msg[0].addr = g_pTheClient->addr;
    msg[0].flags = 0;
    msg[0].len = 1;
    msg[0].buf = g_i2cBuf;

    msg[1].addr = g_pTheClient->addr;
    msg[1].flags = I2C_M_RD;
    msg[1].len = 1;
    msg[1].buf = g_i2cBuf+1;
/* Mod by ISB-GPIO-I2C on 2012.04.06 -> */
    //i2c_transfer(g_pTheClient->adapter, msg, 2);
    wraper_transfer(g_pTheClient, msg, 2);
/* Mod by ISB-GPIO-I2C on 2012.04.06 -< */

    if (((g_i2cBuf[1] & 0x78) >> 3) == 5) { // check for DRV2665 die ID
        g_i2cBuf[0] = 0x15;
        msg[0].addr = g_pTheClient->addr;
        msg[0].flags = 0;
        msg[0].len = 1;
        msg[0].buf = g_i2cBuf;

        msg[1].addr = g_pTheClient->addr;
        msg[1].flags = I2C_M_RD;
        msg[1].len = 1;
        msg[1].buf = g_i2cBuf+1;
/* Mod by ISB-GPIO-I2C on 2012.04.06 -> */
        //i2c_transfer(g_pTheClient->adapter, msg, 2);
        wraper_transfer(g_pTheClient, msg, 2);
/* Mod by ISB-GPIO-I2C on 2012.04.06 -< */

        if ((g_i2cBuf[1] & 0xF0) == 0x00) { // check only the upper nibble
            g_i2cBuf[0] = 0x0C;
            g_i2cBuf[1] = 0x02;
            msg[0].len = 2;
/* Mod by ISB-GPIO-I2C on 2012.04.06 -> */
            //i2c_transfer(g_pTheClient->adapter, msg, 1);
            wraper_transfer(g_pTheClient, msg, 1);
/* Mod by ISB-GPIO-I2C on 2012.04.06 -< */

            g_i2cBuf[0] = 0x0C;
            g_i2cBuf[1] = 0x01;
/* Mod by ISB-GPIO-I2C on 2012.04.06 -> */
            //i2c_transfer(g_pTheClient->adapter, msg, 1);
            wraper_transfer(g_pTheClient, msg, 1);
/* Mod by ISB-GPIO-I2C on 2012.04.06 -< */

            g_i2cBuf[0] = 0x0C;
            g_i2cBuf[1] = 0x04;
/* Mod by ISB-GPIO-I2C on 2012.04.06 -> */
            //i2c_transfer(g_pTheClient->adapter, msg, 1);
            wraper_transfer(g_pTheClient, msg, 1);
/* Mod by ISB-GPIO-I2C on 2012.04.06 -< */

            g_i2cBuf[0] = 0x0C;
            g_i2cBuf[1] = 0x0F;
/* Mod by ISB-GPIO-I2C on 2012.04.06 -> */
            //i2c_transfer(g_pTheClient->adapter, msg, 1);
            wraper_transfer(g_pTheClient, msg, 1);
/* Mod by ISB-GPIO-I2C on 2012.04.06 -< */

            g_i2cBuf[0] = 0x16;
            g_i2cBuf[1] = 0xFF;
/* Mod by ISB-GPIO-I2C on 2012.04.06 -> */
            //i2c_transfer(g_pTheClient->adapter, msg, 1);
            wraper_transfer(g_pTheClient, msg, 1);
/* Mod by ISB-GPIO-I2C on 2012.04.06 -< */

            g_i2cBuf[0] = 0x16;
            g_i2cBuf[1] = 0x00;
/* Mod by ISB-GPIO-I2C on 2012.04.06 -> */
            //i2c_transfer(g_pTheClient->adapter, msg, 1);
            wraper_transfer(g_pTheClient, msg, 1);
/* Mod by ISB-GPIO-I2C on 2012.04.06 -< */

            g_i2cBuf[0] = 0x0C;
            g_i2cBuf[1] = 0x00;
/* Mod by ISB-GPIO-I2C on 2012.04.06 -> */
            //i2c_transfer(g_pTheClient->adapter, msg, 1);
            wraper_transfer(g_pTheClient, msg, 1);
/* Mod by ISB-GPIO-I2C on 2012.04.06 -< */
        }
    }

/*-<< timeout[1][0] check >>-*/
    g_i2cBuf[0] = TS5K_0x02_ADDR ;
    g_i2cBuf[1] = 0x0C; // make sure chip is out of standby and set timeout to 20ms
    msg[0].len = 2;
/* Mod by ISB-GPIO-I2C on 2012.04.06 -> */
    //i2c_transfer(g_pTheClient->adapter, msg, 1);
    wraper_transfer(g_pTheClient, msg, 1);
/* Mod by ISB-GPIO-I2C on 2012.04.06 -< */

    g_i2cBuf[0] = TS5K_0x01_ADDR ;
    msg[0].len = 1;
/* Mod by ISB-GPIO-I2C on 2012.04.06 -> */
    //i2c_transfer(g_pTheClient->adapter, msg, 2);
    wraper_transfer(g_pTheClient, msg, 2);
/* Mod by ISB-GPIO-I2C on 2012.04.06 -< */

    g_i2cBuf[1] &= 0xF8; // leave the die ID alone, set chip to FIFO mode and gain to 28dB
    g_i2cBuf[1] |= 0x02; // leave the die ID alone, set chip to FIFO mode and gain to 28dB
    msg[0].len = 2;
/* Mod by ISB-GPIO-I2C on 2012.04.06 -> */
    //i2c_transfer(g_pTheClient->adapter, msg, 1);
    wraper_transfer(g_pTheClient, msg, 1);
/* Mod by ISB-GPIO-I2C on 2012.04.06 -< */
#endif

   /* For each actuator... */
   for (nActuatorIndex = 0; NUM_ACTUATORS > nActuatorIndex; ++nActuatorIndex)
   {
      /* Disable amp */
      g_bAmpEnabled[nActuatorIndex] = true;   /* to force ImmVibeSPI_ForceOut_AmpDisable disabling the amp */
      ImmVibeSPI_ForceOut_AmpDisable(nActuatorIndex);

#if DEBUG == 1
      DbgOut((KERN_DEBUG "ImmVibeSPI_ForceOut_Initialize. [ImmVibeSPI_ForceOut_AmpDisable]\n"));
#endif

   }

#if DEBUG == 1
   DbgOut((KERN_DEBUG "ImmVibeSPI_ForceOut_Initialize. [end]\n"));
#endif

   return VIBE_S_SUCCESS;
}

/****** ****** ****** ****** ****** ****** ****** ****** ****** ****** ******/
/*
** Called at termination time to set PWM freq, disable amp, etc...
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_Terminate(void)
{
   int nActuatorIndex;  /* Initialized below. */

#if DEBUG == 1
   DbgOut((KERN_DEBUG "ImmVibeSPI_ForceOut_Terminate. [strat]\n"));
#endif

   /* For each actuator... */
   for (nActuatorIndex = 0; NUM_ACTUATORS > nActuatorIndex; ++nActuatorIndex)
   {
      /* Disable amp */
      ImmVibeSPI_ForceOut_AmpDisable(nActuatorIndex);

#if DEBUG == 1
      DbgOut((KERN_DEBUG "ImmVibeSPI_ForceOut_Terminate.[%d] [ImmVibeSPI_ForceOut_AmpDisable]\n",nActuatorIndex));
#endif

   }
   i2c_del_driver(&ts5k_driver);

#if DEBUG == 1
   DbgOut((KERN_DEBUG "ImmVibeSPI_ForceOut_Terminate. [end]\n"));
#endif

   return VIBE_S_SUCCESS;
}

/****** ****** ****** ****** ****** ****** ****** ****** ****** ****** ******/
/*
** Called by the real-time loop to set force output, and enable amp if required
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_ForceOut_SetSamples(VibeUInt8 nActuatorIndex, VibeUInt16 nOutputSignalBitDepth, VibeUInt16 nBufferSizeInBytes, VibeInt8* pForceOutputBuffer)
{
    int txRes=0;		/* initialized below */
    struct i2c_msg msg[1];	/* initialized below */
    int i;			/* initialized below */
    int nUnWritten = 0;
    int nResendAttempt = NAK_RESEND_ATTEMPT;

    struct msm_i2c_dev *dev = i2c_get_adapdata(g_pTheClient->adapter);

#if DEBUG == 1
    DbgOut((KERN_DEBUG "ImmVibeSPI_ForceOut_SetSamples [start]\n"));
#endif

    if (!g_pTheClient){
       return VIBE_E_FAIL;
    }

#if DEBUG == 1
    DbgOut((KERN_DEBUG "ImmVibeSPI_ForceOut_SetSamples[1]: nBufferSizeInBytes=%d  VIBE_OUTPUT_SAMPLE_SIZE=%d\n", nBufferSizeInBytes,VIBE_OUTPUT_SAMPLE_SIZE));
#endif

    if (nBufferSizeInBytes > VIBE_OUTPUT_SAMPLE_SIZE){
       return VIBE_E_FAIL;
    }

    /* Prepare TS5000 play effect command */
#if BOARD_TYPE == TI_DRV2665
    g_i2cBuf[0] = PLAY_CMD_2665;
#else
    g_i2cBuf[0] = (0 == nActuatorIndex) ? ((NUM_BYTES_4KHZ == nBufferSizeInBytes)? PLAY_CMD_PIEZO_4KHZ : PLAY_CMD_PIEZO) : PLAY_CMD_ERM;
#endif
    /* Copy the remaining data */
    for (i=0; i<nBufferSizeInBytes; ++i){
        g_i2cBuf[i+1] = pForceOutputBuffer[i];
    }

    msg[0].addr = g_pTheClient->addr;
    msg[0].flags = 0;
    msg[0].len = nBufferSizeInBytes + 1 /* room for PLAY_CMD */;
    msg[0].buf = g_i2cBuf;
/* Mod by ISB-GPIO-I2C on 2012.04.06 -> */
    //txRes = i2c_transfer(g_pTheClient->adapter, msg, 1);
    txRes = wraper_transfer(g_pTheClient, msg, 1);
/* Mod by ISB-GPIO-I2C on 2012.04.06 -< */
    if (txRes < 0){

        nUnWritten = dev->cnt;
/* Mod by ISB-GPIO-I2C on 2012.04.06 -> */
        // DbgOut((KERN_ERR "ImmVibeSPI_ForceOut_SetSamples: i2c_transfer error (%d), %d bytes unwritten.\n", txRes, nUnWritten)); /* 2012.05.15 tspdrv chg*/
        DbgOut((KERN_DEBUG "ImmVibeSPI_ForceOut_SetSamples: wraper_transfer warning (%d), %d bytes unwritten.\n", txRes, nUnWritten)); /* 2012.05.15 tspdrv chg*/
/* Mod by ISB-GPIO-I2C on 2012.04.06 -< */

        while ((txRes < 0) && (nResendAttempt--) && (msg[0].len > 0) && (nUnWritten > 0)){
            /* Wait NAK_RESEND_DELAY_MS ms before resending the unwritten bytes */

            /* FUJITSU:2012.04.10  start */
            if(mc_count == 0){	/* Normal mode */
               schedule_timeout_interruptible(msecs_to_jiffies(NAK_RESEND_DELAY_MS));
            }
            else{		/* MC mode */
               mdelay(5);
            }
            /* FUJITSU:2012.04.10 end   */

            msg[0].buf += (msg[0].len - nUnWritten - 1 /* room for PLAY_CMD */);
            msg[0].len = (nUnWritten + 1 /* room for PLAY_CMD */);

            /* Prepare TS5000 play effect command */
#if BOARD_TYPE == TI_DRV2665
            msg[0].buf[0] = PLAY_CMD_2665;
#else
            msg[0].buf[0] = (0 == nActuatorIndex) ? ((NUM_BYTES_4KHZ == nBufferSizeInBytes)? PLAY_CMD_PIEZO_4KHZ : PLAY_CMD_PIEZO) : PLAY_CMD_ERM;
#endif
/* Mod by ISB-GPIO-I2C on 2012.04.06 -> */
            //txRes = i2c_transfer(g_pTheClient->adapter, msg, 1);
            txRes = wraper_transfer(g_pTheClient, msg, 1);
/* Mod by ISB-GPIO-I2C on 2012.04.06 -< */
            if (txRes < 0){
/* Mod by ISB-GPIO-I2C on 2012.04.06 -> */
                //DbgOut((KERN_ERR "ImmVibeSPI_ForceOut_SetSamples: i2c_transfer error (%d) bytes unwritten.\n", txRes));
                //DbgOut((KERN_ERR "ImmVibeSPI_ForceOut_SetSamples: wraper_transfer error (%d) bytes unwritten.\n", txRes));	/* 2012.05.15 tspdrv chg*/
                DbgOut((KERN_DEBUG "ImmVibeSPI_ForceOut_SetSamples: wraper_transfer warning (%d) bytes unwritten.\n", txRes));	/* 2012.05.15 tspdrv chg*/
/* Mod by ISB-GPIO-I2C on 2012.04.06 -< */
            }
        }
    }

#if DEBUG == 1
    DbgOut((KERN_DEBUG "ImmVibeSPI_ForceOut_SetSamples [end]\n"));
#endif

    return (nResendAttempt == NAK_RESEND_ATTEMPT) ? VIBE_S_SUCCESS : VIBE_E_FAIL;
}

/****** ****** ****** ****** ****** ****** ****** ****** ****** ****** ******/
/*
** Called to get the device name (device name must be returned as ANSI char)
*/
IMMVIBESPIAPI VibeStatus ImmVibeSPI_Device_GetName(VibeUInt8 nActuatorIndex, char *szDevName, int nSize)
{
    char szDeviceName[VIBE_MAX_DEVICE_NAME_LENGTH] = "TS5K-2665";

#if DEBUG == 1
    DbgOut((KERN_DEBUG "ImmVibeSPI_Device_GetName [start]\n"));
#endif

    if ((strlen(szDeviceName) + 1 + strlen(g_szFWVersion)) >= nSize){
        return VIBE_E_FAIL;
    }
    sprintf(szDevName, "%s %s", szDeviceName, g_szFWVersion);

#if DEBUG == 1
    DbgOut((KERN_DEBUG "ImmVibeSPI_Device_GetName  szDevName=%s  nSize=%d [end]\n",szDevName,nSize));
#endif

    return VIBE_S_SUCCESS;
}

/****** ****** ****** ****** ****** ****** ****** ****** ****** ****** ******/
/*
** TS5000 callback functions
*/
static int ts5k_probe(struct i2c_client* client, const struct i2c_device_id* id)
{
    int nRet = 0;

#if DEBUG == 1
    DbgOut((KERN_DEBUG "ImmVibeSPI: ts5k_probe [start]\n"));	/* debug-out */
#endif

    if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)){
        nRet = -ENODEV;
        DbgOut((KERN_ERR "ImmVibeSPI: ts5k_probe: i2c_check_functionality failed.\n"));
    }
    else{

#if DEBUG == 1
        DbgOut((KERN_DEBUG "ImmVibeSPI: ts5k_probe [i2c_check_functionality=True]\n"));
#endif

        g_pTheClient = client;
        g_pI2CDev = (struct msm_i2c_dev*)i2c_get_adapdata(client->adapter);

        /* 1ms delay */
        mdelay(1);
      
        /* Standby-Mode setting */
        setStandby(true);
    }

#if DEBUG == 1
    DbgOut((KERN_DEBUG "ImmVibeSPI: ts5k_probe [end]\n"));	/* debug-out */
#endif

    return nRet;
}

/****** ****** ****** ****** ****** ****** ****** ****** ****** ****** ******/
/*
** TS5000 callback functions
*/
static int ts5k_remove(struct i2c_client *client)
{

#if DEBUG == 1
    DbgOut((KERN_DEBUG "ts5k_remove. [start]\n"));
#endif

    g_pTheClient = NULL;

#if DEBUG == 1
    DbgOut((KERN_DEBUG "ts5k_remove. [end]\n"));
#endif

    return 0;
}

/****** ****** ****** ****** ****** ****** ****** ****** ****** ****** ******/
/*
** Manufacturer Command
*/

static int mc_mode_haptic(VibeUInt8 nCycle,VibeUInt8 nCount)
{
    int i=0;
    VibeUInt8 mc_databuf[40];

    mc_count=0;
    /* mc_mode_haptic start */
//  printk("## mc_mode_haptic [nCycle=%d nCount=%d] [start]\n",nCycle,nCount);

    mc_count = nCycle * nCount;
//  printk("## mc_mode_haptic mc_count=%d\n",mc_count);

  /*-<< Wave-data create >>-*/
    mc_databuf[0] = 0 ;
    mc_databuf[1] = 20 ;
    mc_databuf[2] = 39 ;
    mc_databuf[3] = 58 ;
    mc_databuf[4] = 75 ;
    mc_databuf[5] = 90 ;
    mc_databuf[6] = 103 ;
    mc_databuf[7] = 113 ;
    mc_databuf[8] = 121 ;
    mc_databuf[9] = 125 ;
    mc_databuf[10] = 127 ;
    mc_databuf[11] = 125 ;
    mc_databuf[12] = 121 ;
    mc_databuf[13] = 113 ;
    mc_databuf[14] = 103 ;
    mc_databuf[15] = 90 ;
    mc_databuf[16] = 75 ;
    mc_databuf[17] = 58 ;
    mc_databuf[18] = 39 ;
    mc_databuf[19] = 20 ;
    mc_databuf[20] = 0 ;
    mc_databuf[21] = -20 ;
    mc_databuf[22] = -39 ;
    mc_databuf[23] = -58 ;
    mc_databuf[24] = -75 ;
    mc_databuf[25] = -90 ;
    mc_databuf[26] = -103 ;
    mc_databuf[27] = -113 ;
    mc_databuf[28] = -121 ;
    mc_databuf[29] = -125 ;
    mc_databuf[30] = -127 ;
    mc_databuf[31] = -125 ;
    mc_databuf[32] = -121 ;
    mc_databuf[33] = -113 ;
    mc_databuf[34] = -103 ;
    mc_databuf[35] = -90 ;
    mc_databuf[36] = -75 ;
    mc_databuf[37] = -58 ;
    mc_databuf[38] = -39 ;
    mc_databuf[39] = -20 ;

  /*-<< AmpEnable >>-*/
//  printk("## mc_mode_haptic - AmpEnable\n");
    if (!g_bAmpEnabled[0]){		/* actuator-index[Piezo] */
       g_bAmpEnabled[0] = true;
       setStandby(false);
       printk("## mc_mode_haptic - ImmVibeSPI_ForceOut_AmpEnable [setStandby=false]\n");
    }
/* 2012.05.15 tspdrv func-chg [start]
    fifo_check();
** 2012.05.15 tspdrv func-chg [end] */
    reg1_init();	/* fifo_check()-> reg1_init()  2012.05.15 */

  /*-<< SetSample >>-*/
//  printk("## mc_mode_haptic - SetSample\n");
//  ImmVibeSPI_ForceOut_SetSamples(0, 8, 40, (VibeInt8 *)mc_databuf);

   /*-<< Loop-Counter setting & SetSample call >>-*/
    for(i=0;i<mc_count;i++){
//     printk("## mc_mode_haptic count =%d [start]\n",i);
       ImmVibeSPI_ForceOut_SetSamples(0, 8, 40, (VibeInt8 *)mc_databuf);
    }
    mc_count=0;
//  printk("## mc_mode_haptic [end]\n");
    return 0;
}

/****** ****** ****** ****** ****** ****** ****** ****** ****** ****** ******/
/*
** Stanby-mode setting
** standbyState=true[standby->H] false[standby->saspend]
*/
static void setStandby(bool standbyState)
{
   struct i2c_msg msg[2];

// printk("setStandby[start]\n");
   g_i2cBuf[0] = TS5K_0x02_ADDR ;
   msg[0].addr = g_pTheClient->addr;
   msg[0].flags = 0;
   msg[0].len = 1;
   msg[0].buf = g_i2cBuf;
/* Mod by ISB-GPIO-I2C on 2012.04.06 -> */
   //i2c_transfer(g_pTheClient->adapter, msg, 1);
   wraper_transfer(g_pTheClient, msg, 1);
/* Mod by ISB-GPIO-I2C on 2012.04.06 -< */

   msg[0].flags = I2C_M_RD;
   msg[0].len = 1;
   msg[0].buf = g_i2cBuf+1;
/* Mod by ISB-GPIO-I2C on 2012.04.06 -> */
   //i2c_transfer(g_pTheClient->adapter, msg, 1);
   wraper_transfer(g_pTheClient, msg, 1);
/* Mod by ISB-GPIO-I2C on 2012.04.06 -< */

   g_i2cBuf[1] &= 0xFD; /* 2012.05.11 add[0x02:EN_Override -> 1'b0 set] */

   msg[0].flags = 0;
   msg[0].len = 2;
   msg[0].buf = g_i2cBuf;

/* STANDBY monitor */
   if (standbyState && ((g_i2cBuf[1] & 0x40) == 0)){
      g_i2cBuf[1] = g_i2cBuf[1] | 0x40;
/* Mod by ISB-GPIO-I2C on 2012.04.06 -> */
      //i2c_transfer(g_pTheClient->adapter, msg, 1);
      wraper_transfer(g_pTheClient, msg, 1);
/* Mod by ISB-GPIO-I2C on 2012.04.06 -< */
   }
   else{
      g_i2cBuf[1] &= 0xBF;
/* Mod by ISB-GPIO-I2C on 2012.04.06 -> */
      //i2c_transfer(g_pTheClient->adapter, msg, 1);
      wraper_transfer(g_pTheClient, msg, 1);
/* Mod by ISB-GPIO-I2C on 2012.04.06 -< */
   }

   mdelay(2);

// printk(KERN_DEBUG "setStandby[end]\n");
}

/****** ****** ****** ****** ****** ****** ****** ****** ****** ****** ******/
/*
** Idol-Mode setting
*/
/*  2012.05.15 tspdrv func-chg [start]
static void fifo_check(void)
**  2012.05.15 tspdrv func-chg [end] */
static void reg1_init(void)	/* 2012.05.15 tspdrv func-chg */
{
   struct i2c_msg msg[2];

#if DEBUG == 1
   printk(KERN_DEBUG "reg1_init[start]\n");
#endif

/* reg[0x01] FIFOEmpty */
   g_i2cBuf[0] = TS5K_0x01_ADDR ; 
   msg[0].addr = g_pTheClient->addr;
   msg[0].flags = 0;
   msg[0].len = 1;
   msg[0].buf = g_i2cBuf ;

   msg[1].addr = g_pTheClient->addr;
   msg[1].flags = I2C_M_RD;
   msg[1].len = 1;
   msg[1].buf = g_i2cBuf+1 ;
/* Mod by ISB-GPIO-I2C on 2012.04.06 -> */
//   //i2c_transfer(g_pTheClient->adapter, msg, 2);
   wraper_transfer(g_pTheClient, msg, 2);	/* reg[0x01] value Read */
/* Mod by ISB-GPIO-I2C on 2012.04.06 -< */

/* reg[0x01] write */
/* add 2012.05.15[start] */
   g_i2cBuf[0] = TS5K_0x01_ADDR ;
   msg[0].addr = g_pTheClient->addr;
   msg[0].flags = 0;
/* add 2012.05.15[end]   */

   g_i2cBuf[1] &= 0xF8; // leave the die ID alone, set chip to FIFO mode and gain to 28dB
   g_i2cBuf[1] |= 0x02; // leave the die ID alone, set chip to FIFO mode and gain to 28dB
   msg[0].len = 2;
/* Mod by ISB-GPIO-I2C on 2012.04.06 -> */
   //i2c_transfer(g_pTheClient->adapter, msg, 1);
   wraper_transfer(g_pTheClient, msg, 1);
/* Mod by ISB-GPIO-I2C on 2012.04.06 -< */

#if DEBUG == 1
   printk(KERN_DEBUG "reg1_init[end]\n");
#endif

}
