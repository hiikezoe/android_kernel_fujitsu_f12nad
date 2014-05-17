/*
** =========================================================================
** File:
**     tspdrv.c
**
** Description: 
**     TouchSense Kernel Module main entry-point.
**
** Portions Copyright (c) 2008-2011 Immersion Corporation. All Rights Reserved. 
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

#ifndef __KERNEL__
#define __KERNEL__
#endif
/* del.2012.06.04 [start]
#ifndef MODULE
#define MODULE
#endif
** del.2012.06.04 [end] */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/timer.h>
#include <linux/fs.h>
#include <linux/version.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <asm/uaccess.h>
#include "tspdrv.h"
#include "ImmVibeSPI.c"

/* Add by ISB-GPIO-I2C on 2012.04.xx -< */
#include "ImmVibe_gpio.c"
/* Add by ISB-GPIO-I2C on 2012.04.xx -< */

/* FUJITSU:2012.03.23  TS5K [start] */
#if 0
#include "tspdrv_gpio.h"
#include "tspdrv_gpio.c"
#endif
/* FUJITSU:2012.03.23  TS5K [end]   */

#if defined(VIBE_DEBUG) && defined(VIBE_RECORD)
#include <tspdrvRecorder.c>
#endif

/* Device name and version information */
#define VERSION_STR " v3.4.55.8\n"                  /* DO NOT CHANGE - this is auto-generated */
#define VERSION_STR_LEN 16                          /* account extra space for future extra digits in version number */
static char g_szDeviceName[  (VIBE_MAX_DEVICE_NAME_LENGTH 
                            + VERSION_STR_LEN)
                            * NUM_ACTUATORS];       /* initialized in init_module */
static size_t g_cchDeviceName;                      /* initialized in init_module */

/* Flag indicating whether the driver is in use */
static char g_bIsPlaying = false;

/* Buffer to store data sent to SPI */
#define SPI_BUFFER_SIZE (NUM_ACTUATORS * (VIBE_OUTPUT_SAMPLE_SIZE + SPI_HEADER_SIZE))
static int g_bStopRequested = false;
static actuator_samples_buffer g_SamplesBuffer[NUM_ACTUATORS] = {{0}}; 
static char g_cWriteBuffer[SPI_BUFFER_SIZE];

/* For QA purposes */
#ifdef QA_TEST
#define FORCE_LOG_BUFFER_SIZE   128
#define TIME_INCREMENT          5
static int g_nTime = 0;
static int g_nForceLogIndex = 0;
static VibeInt8 g_nForceLog[FORCE_LOG_BUFFER_SIZE];
#endif

#if ((LINUX_VERSION_CODE & 0xFFFF00) < KERNEL_VERSION(2,6,0))
#error Unsupported Kernel version
#endif

#ifndef HAVE_UNLOCKED_IOCTL
#define HAVE_UNLOCKED_IOCTL 1 
#endif

#ifdef IMPLEMENT_AS_CHAR_DRIVER
static int g_nMajor = 0;
#endif

/* Needs to be included after the global variables because it uses them */
#ifdef CONFIG_HIGH_RES_TIMERS
    #include "VibeOSKernelLinuxHRTime.c"
#else
    #include <VibeOSKernelLinuxTime.c>
#endif

/* FUJITSU:2011-12-06 VIB add start */
#define WKUP_VIB_MODE_MC 1
#if 0
extern int makercmd_mode;
#endif
int makercmd_mode=0;
/* FUJITSU:2011-12-06 VIB add end */

/* File IO */
static int     open(struct inode *inode, struct file *file);
static int     release(struct inode *inode, struct file *file);
static ssize_t read(struct file *file, char *buf, size_t count, loff_t *ppos);
static ssize_t write(struct file *file, const char *buf, size_t count, loff_t *ppos);
#if HAVE_UNLOCKED_IOCTL
static long    unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg);
#else
static int     ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg);
#endif
static struct file_operations fops = 
{
    .owner =            THIS_MODULE,
    .read =             read,
    .write =            write,
#if HAVE_UNLOCKED_IOCTL
    .unlocked_ioctl =   unlocked_ioctl,
#else
    .ioctl =            ioctl,
#endif
    .open =             open,
    .release =          release,
    .llseek =           default_llseek    /* using default implementation as declared in linux/fs.h */
};

#ifndef IMPLEMENT_AS_CHAR_DRIVER
static struct miscdevice miscdev = 
{
	.minor =    MISC_DYNAMIC_MINOR,
	.name =     MODULE_NAME,
	.fops =     &fops
};
#endif

static int    suspend(struct platform_device *pdev, pm_message_t state);
static int    resume(struct platform_device *pdev);
static struct platform_driver platdrv = 
{
    .suspend =  suspend,	
    .resume =   resume,	
    .driver = 
    {		
        .name = MODULE_NAME,	
    },	
};

static void   platform_release(struct device *dev);
static struct platform_device platdev = 
{	
    .name =     MODULE_NAME,
    .id =       -1,                     /* means that there is only one device */
    .dev = 
    {
        .platform_data = NULL, 		
        .release = platform_release,    /* a warning is thrown during rmmod if this is absent */
    },
};

/* Module info */
MODULE_AUTHOR("Immersion Corporation");
MODULE_DESCRIPTION("TouchSense Kernel Module");
MODULE_LICENSE("GPL v2");

/* FUJITSU:2012.03.23  TS5K [start] */
MODULE_DEVICE_TABLE(i2c, ts5k_id);
/* FUJITSU:2012.03.23  TS5K [end]   */

static int __init tspdrv_init(void)
{
    int nRet, i;   /* initialized below */

#if DEBUG == 1
    DbgOut((KERN_INFO "tspdrv: init_module [start]\n"));
#endif

/* Add by ISB-GPIO-I2C on 2012.04.xx -> */
/* 2012.06.06 chg [start]
    if (is_use_i2c) {
** 2012.06.06 chg [end]   */
    if (is_use_i2c_tspdrv) {	/* 2012.06.06 chg */
        DbgOut((KERN_INFO "tspdrv: REAL-I2C [enabled]\n"));
    } else {
        // GPIO CFG
        gpio_tlmm_config(GPIO_CFG(SCL, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
        gpio_tlmm_config(GPIO_CFG(SDA, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
        DbgOut((KERN_INFO "tspdrv: GPIO-I2C SCL=%d,SDA=%d [enabled]\n", SCL, SDA));
    }
/* Add by ISB-GPIO-I2C on 2012.04.xx -< */

#ifdef IMPLEMENT_AS_CHAR_DRIVER
    g_nMajor = register_chrdev(0, MODULE_NAME, &fops);
    if (g_nMajor < 0) 
    {
        DbgOut((KERN_ERR "tspdrv: can't get major number.\n"));
        return g_nMajor;
    }
#else
    nRet = misc_register(&miscdev);
    if (nRet) 
    {
        DbgOut((KERN_ERR "tspdrv: misc_register failed.\n"));
		return nRet;
    }
#endif

    nRet = platform_device_register(&platdev);
    if (nRet) 
    {
        DbgOut((KERN_ERR "tspdrv: platform_device_register failed.\n"));
    }

    nRet = platform_driver_register(&platdrv);
    if (nRet) 
    {
        DbgOut((KERN_ERR "tspdrv: platform_driver_register failed.\n"));
    }

    DbgRecorderInit(());

    ImmVibeSPI_ForceOut_Initialize();
    VibeOSKernelLinuxInitTimer();

    /* Get and concatenate device name and initialize data buffer */
    g_cchDeviceName = 0;
    for (i=0; i<NUM_ACTUATORS; i++)
    {
        char *szName = g_szDeviceName + g_cchDeviceName;

        ImmVibeSPI_Device_GetName(i, szName, VIBE_MAX_DEVICE_NAME_LENGTH);

        /* Append version information and get buffer length */
        strcat(szName, VERSION_STR);
        g_cchDeviceName += strlen(szName);

        g_SamplesBuffer[i].nIndexPlayingBuffer = -1; /* Not playing */
        g_SamplesBuffer[i].actuatorSamples[0].nBufferSize = 0;
        g_SamplesBuffer[i].actuatorSamples[1].nBufferSize = 0;
    }

#if DEBUG == 1
    DbgOut((KERN_INFO "tspdrv: init_module [end]\n"));
#endif

    return 0;
}

static void __exit tspdrv_exit(void)
{

#if DEBUG == 1
    DbgOut((KERN_INFO "tspdrv: tspdrv_exit [start]\n"));
#endif

    DbgRecorderTerminate(());

    VibeOSKernelLinuxTerminateTimer();
    ImmVibeSPI_ForceOut_Terminate();

    platform_driver_unregister(&platdrv);
    platform_device_unregister(&platdev);

#ifdef IMPLEMENT_AS_CHAR_DRIVER
    unregister_chrdev(g_nMajor, MODULE_NAME);
#else
    misc_deregister(&miscdev);
#endif
}

static int open(struct inode *inode, struct file *file) 
{
    int rtn=0;

#if DEBUG == 1
    DbgOut((KERN_INFO "tspdrv: open [start]\n"));
#endif

    rtn=try_module_get(THIS_MODULE);
    if(rtn==0){
       return -ENODEV;
    }

#if DEBUG == 1
    DbgOut((KERN_INFO "tspdrv: open [end]\n"));
#endif

    return 0; 
}

static int release(struct inode *inode, struct file *file) 
{

#if DEBUG == 1
    DbgOut((KERN_INFO "tspdrv: release[start]\n"));
#endif

    /* 
    ** Reset force and stop timer when the driver is closed, to make sure
    ** no dangling semaphore remains in the system, especially when the
    ** driver is run outside of immvibed for testing purposes.
    */
    VibeOSKernelLinuxStopTimer();

    /* 
    ** Clear the variable used to store the magic number to prevent 
    ** unauthorized caller to write data. TouchSense service is the only 
    ** valid caller.
    */
    file->private_data = (void*)NULL;

    module_put(THIS_MODULE);

#if DEBUG == 1
    DbgOut((KERN_INFO "tspdrv: release[end]\n"));
#endif

    return 0; 
}

static ssize_t read(struct file *file, char *buf, size_t count, loff_t *ppos)
{
    const size_t nBufSize = (g_cchDeviceName > (size_t)(*ppos)) ? min(count, g_cchDeviceName - (size_t)(*ppos)) : 0;

#if DEBUG == 1
    DbgOut((KERN_INFO "tspdrv: read[start]\n"));
#endif

    /* End of buffer, exit */
    if (0 == nBufSize) return 0;

    if (0 != copy_to_user(buf, g_szDeviceName + (*ppos), nBufSize)) 
    {
        /* Failed to copy all the data, exit */
        DbgOut((KERN_ERR "tspdrv: copy_to_user failed.\n"));
        return 0;
    }

    /* Update file position and return copied buffer size */
    *ppos += nBufSize;

#if DEBUG == 1
    DbgOut((KERN_INFO "tspdrv: read[end]\n"));
#endif

    return nBufSize;
}

static ssize_t write(struct file *file, const char *buf, size_t count, loff_t *ppos)
{
    int i = 0;
    *ppos = 0;  /* file position not used, always set to 0 */

#if DEBUG ==1
    DbgOut((KERN_INFO "tspdrv: write[start]\n"));
#endif

    /* 
    ** Prevent unauthorized caller to write data. 
    ** TouchSense service is the only valid caller.
    */

/* FUJITSU:2012.03.29  TS5K[start] */
#if 0
    if (file->private_data != (void*)TSPDRV_MAGIC_NUMBER) 
    {
        DbgOut((KERN_ERR "tspdrv: unauthorized write.\n"));
        return 0;
    }
#endif
/* FUJITSU:2012.03.29  TS5K[end] */

    /* Copy immediately the input buffer */
    if (0 != copy_from_user(g_cWriteBuffer, buf, count))
    {
        /* Failed to copy all the data, exit */
        DbgOut((KERN_ERR "tspdrv: copy_from_user failed.\n"));
        return 0;
    }

    /* Check buffer size */
    if ((count <= SPI_HEADER_SIZE) || (count > SPI_BUFFER_SIZE))
    {
        DbgOut((KERN_ERR "tspdrv: invalid write buffer size.\n"));
        return 0;
    }

    while (i < count)
    {
        int nIndexFreeBuffer;   /* initialized below */

        samples_buffer* pInputBuffer = (samples_buffer*)(&g_cWriteBuffer[i]);

        if ((i + SPI_HEADER_SIZE) >= count)
        {
            /*
            ** Index is about to go beyond the buffer size.
            ** (Should never happen).
            */
            DbgOut((KERN_EMERG "tspdrv: invalid buffer index.\n"));
        }

        /* Check bit depth */
        if (8 != pInputBuffer->nBitDepth)
        {
            DbgOut((KERN_WARNING "tspdrv: invalid bit depth. Use default value (8).\n"));
        }

        /* The above code not valid if SPI header size is not 3 */
#if (SPI_HEADER_SIZE != 3)
#error "SPI_HEADER_SIZE expected to be 3"
#endif

        /* Check buffer size */
        if ((i + SPI_HEADER_SIZE + pInputBuffer->nBufferSize) > count)
        {
            /*
            ** Index is about to go beyond the buffer size.
            ** (Should never happen).
            */
            DbgOut((KERN_EMERG "tspdrv: invalid data size.\n"));
        }
        
        /* Check actuator index */
        if (NUM_ACTUATORS <= pInputBuffer->nActuatorIndex)
        {
            DbgOut((KERN_ERR "tspdrv: invalid actuator index.\n"));
            i += (SPI_HEADER_SIZE + pInputBuffer->nBufferSize);
            continue;
        }

        if (0 == g_SamplesBuffer[pInputBuffer->nActuatorIndex].actuatorSamples[0].nBufferSize)
        {
            nIndexFreeBuffer = 0;
        }
        else if (0 == g_SamplesBuffer[pInputBuffer->nActuatorIndex].actuatorSamples[1].nBufferSize)
        {
             nIndexFreeBuffer = 1;
        }
        else
        {
            /* No room to store new samples  */
            DbgOut((KERN_ERR "tspdrv: no room to store new samples.\n"));
            return 0;
        }

        /* Store the data in the free buffer of the given actuator */
        memcpy(&(g_SamplesBuffer[pInputBuffer->nActuatorIndex].actuatorSamples[nIndexFreeBuffer]), &g_cWriteBuffer[i], (SPI_HEADER_SIZE + pInputBuffer->nBufferSize));

        /* If the no buffer is playing, prepare to play g_SamplesBuffer[pInputBuffer->nActuatorIndex].actuatorSamples[nIndexFreeBuffer] */
        if ( -1 == g_SamplesBuffer[pInputBuffer->nActuatorIndex].nIndexPlayingBuffer)
        {
           g_SamplesBuffer[pInputBuffer->nActuatorIndex].nIndexPlayingBuffer = nIndexFreeBuffer;
           g_SamplesBuffer[pInputBuffer->nActuatorIndex].nIndexOutputValue = 0;
        }

        /* Increment buffer index */
        i += (SPI_HEADER_SIZE + pInputBuffer->nBufferSize);
    }

#ifdef QA_TEST
    g_nForceLog[g_nForceLogIndex++] = g_cSPIBuffer[0];
    if (g_nForceLogIndex >= FORCE_LOG_BUFFER_SIZE)
    {
        for (i=0; i<FORCE_LOG_BUFFER_SIZE; i++)
        {
            printk("<6>%d\t%d\n", g_nTime, g_nForceLog[i]);
            g_nTime += TIME_INCREMENT;
        }
        g_nForceLogIndex = 0;
    }
#endif

    /* Start the timer after receiving new output force */
    g_bIsPlaying = true;
    VibeOSKernelLinuxStartTimer();

#if DEBUG == 1
    DbgOut((KERN_INFO "tspdrv: write[end]\n"));
#endif

    return count;
}

#if HAVE_UNLOCKED_IOCTL
static long unlocked_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
#else
static int ioctl(struct inode *inode, struct file *file, unsigned int cmd, unsigned long arg)
#endif
{
#ifdef QA_TEST
    int i;
#endif

/* FUJITSU:2012-01-19 VIB add start */
#if HAVE_UNLOCKED_IOCTL
	long	ret = 0;
#else
	int	ret = 0;
#endif
   haptic_buffer buf;
/* FUJITSU:2012-01-19 VIB add end */

#if DEBUG == 1
   DbgOut((KERN_INFO "tspdrv: ioctl [start]\n"));
#endif

    switch (cmd)
    {
        case TSPDRV_STOP_KERNEL_TIMER:
            /* 
            ** As we send one sample ahead of time, we need to finish playing the last sample
            ** before stopping the timer. So we just set a flag here.
            */
            if (true == g_bIsPlaying) g_bStopRequested = true;

#ifdef VIBEOSKERNELPROCESSDATA
            /* Last data processing to disable amp and stop timer */
            VibeOSKernelProcessData(NULL);
#endif

#ifdef QA_TEST
            if (g_nForceLogIndex)
            {
                for (i=0; i<g_nForceLogIndex; i++)
                {
                    printk("<6>%d\t%d\n", g_nTime, g_nForceLog[i]);
                    g_nTime += TIME_INCREMENT;
                }
            }
            g_nTime = 0;
            g_nForceLogIndex = 0;
#endif
            break;

        case TSPDRV_MAGIC_NUMBER:
            file->private_data = (void*)TSPDRV_MAGIC_NUMBER;
            break;

        case TSPDRV_ENABLE_AMP:

            ImmVibeSPI_ForceOut_AmpEnable(arg);
/* FUJITSU:2012.03.23  TS5K [end] */

            DbgRecorderReset((arg));
            DbgRecord((arg,";------- TSPDRV_ENABLE_AMP ---------\n"));
            break;

        case TSPDRV_DISABLE_AMP:

            /* Small fix for now to handle proper combination of TSPDRV_STOP_KERNEL_TIMER and TSPDRV_DISABLE_AMP together */
            /* If a stop was requested, ignore the request as the amp will be disabled by the timer proc when it's ready */
            if(!g_bStopRequested)
            {
                ImmVibeSPI_ForceOut_AmpDisable(arg);
            }
            break;

        case TSPDRV_GET_NUM_ACTUATORS:

/* 2012.06.11 [start]
            ImmVibeSPI_ForceOut_AmpEnable(arg);
** 2012.06.11 [end] */

            return NUM_ACTUATORS;

/* FUJITSU:2012.03.30  MC-Mode[start] */
        case TSPDRV_MC_HAPTIC:

            ret = copy_from_user(&buf, (int __user *) arg, sizeof(haptic_buffer));
            if (ret != 0) {
                 printk(KERN_ERR "tspdrv: copy_from_user err %ld \n",ret);
            }

            ret = mc_mode_haptic(buf.nCycle, buf.nCount);

            break;

/* FUJITSU:2012.03.30  MC-Mode[end]   */

#if 0
        case TSPDRV_CALIBRATE:

		/* Copy the input buffer */
		if (0 != copy_from_user(g_CalibrateBuffer, (void __user *)arg, CAB_BUFFER_SIZE))
		{
			/* Failed to copy all the data, exit */
			DbgOut((KERN_ERR "tspdrv: copy_from_user failed.\n"));
			ret = 1;
		}

		ret = calibrate_adux(g_CalibrateBuffer);


		/* Copy the output buffer */
		if (0 != copy_to_user((void __user *)arg, g_CalibrateBuffer, CAB_BUFFER_SIZE))
		{
			/* Failed to copy all the data, exit */
			DbgOut((KERN_ERR "tspdrv: copy_to_user failed.\n"));
			ret = 1;
		}
		break;
#endif

	default:
		printk("tspdrv: Command Error cmd=%d] \n",cmd);   /* debug */
		return 1;
/* FUJITSU:2011-12-06 add end*/
    }

#if DEBUG == 1
    DbgOut((KERN_INFO "tspdrv: ioctl [end]\n"));
#endif

    return ret;
}

static int suspend(struct platform_device *pdev, pm_message_t state) 
{
    if (g_bIsPlaying)
    {
        DbgOut((KERN_INFO "tspdrv: can't suspend, still playing effects.\n"));
        return -EBUSY;
    }
    else
    {
        DbgOut((KERN_INFO "tspdrv: suspend.\n"));

/* FUJITSU:2012.03.23  TS5K [start] */
#if 0
/* FUJITSU:2012-02-22  add start */
		//haptics pwoer OFF
		haptics_pon_off();
/* FUJITSU:2012-02-22  add end */
#endif
/* FUJITSU:2012.03.23  TS5K [end] */

        return 0;
    }
}

static int resume(struct platform_device *pdev) 
{	

#if DEBUG == 1
  DbgOut((KERN_INFO "tspdrv: resume.\n"));
#endif

/* FUJITSU:2012.03.23  TS5K [start] */
#if 0
/* FUJITSU:2012-02-22  add start */
	//haptics pwoer ON
	haptics_pon_on();
/* FUJITSU:2012-02-22  add end */
#endif
/* FUJITSU:2012.03.23  TS5K [end]  */

	return 0;   /* can resume */
}

static void platform_release(struct device *dev) 
{	
//  DbgOut((KERN_INFO "tspdrv: platform_release.\n"));
}

module_init(tspdrv_init);
module_exit(tspdrv_exit);
