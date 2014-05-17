/*
 * Copyright(C) 2011 FUJITSU LIMITED
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
/*----------------------------------------------------------------------------*/
// COPYRIGHT(C) FUJITSU LIMITED 2011-2012
/*----------------------------------------------------------------------------*/
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/i2c.h>
#include <linux/irq.h>

#include <asm/uaccess.h>
#include <asm/io.h>
#include <asm/gpio.h>

#include <linux/ctype.h>
#include <linux/kobject.h>
#include <linux/sysfs.h>
/* FUJITSU:2012-01-16 bsp-sensor mod start */
#include <linux/compassdriver.h>
/* FUJITSU:2012-01-16 bsp-sensor mod end */

#define debug(format, arg...)	\
	if (0)	compass_printk(KERN_DEBUG, format, ##arg)


const char DEVNAME[] = "AKM";

// akm Peculiar instance data
akm_t akm[1];

#if 1   /* FUJITSU:2011-12-01 ICS mod start */
DEFINE_SEMAPHORE(akm_sem);		// For exclusive control
#else
DECLARE_MUTEX(akm_sem);		// For exclusive control
#endif  /* FUJITSU:2011-12-01 ICS mod end */
/*--------------------------------------------------------------------
akm Interruption terminal (AKM.DRDY -> JIKI_DRDY)
--------------------------------------------------------------------*/
#define JIKI_DRDY_OFF  0x00

// When the interruption demand is generated, the flag becomes one.
// It is flag automatically clearness output data is read.
// GPIO True where interruption has been generated.
#define GPIO_ISACTIVE()   (gpio_get_value(JIKI_DRDY_GPIO))

//----------------------------------------------------------------


///*--------------------------------------------------------------------
//Prototype of foreign function not declared in header file
//--------------------------------------------------------------------*/
//#ifdef	CONFIG_BRUTUS
//extern int case_getstatus(uint8_t * isOpen);
//#endif	// CONFIG_BRUTUS


/*--------------------------------------------------------------------
Function  :The frequency is totaled the present error/retrying.
Attention :Execute it while excluded.
--------------------------------------------------------------------*/
#define akm_update_error_count()						\
	do {												\
		akm->total_error_count += akm->error_count;	\
		akm->total_retry_count += akm->retry_count;	\
		akm->error_count = akm->retry_count = 0;		\
	} while(false)


/*== AKM Basic control function (Reading and writing of register)==*/

/*--------------------------------------------------------------------
akm_read_(verify_)regs() : reg_data[] Number of arguments of minimum elements.
--------------------------------------------------------------------*/
#define READREGS_MINBUFSIZE     3

#define H_ADJ(h, asa)    (h * (asa + 128))

#define SELFTEST_JUDGE_HX_MIN   (-50 * 256)
#define SELFTEST_JUDGE_HX_MAX   ( 50 * 256)
#define SELFTEST_JUDGE_HY_MIN   (-50 * 256)
#define SELFTEST_JUDGE_HY_MAX   ( 50 * 256)
#define SELFTEST_JUDGE_HZ_MIN   (-500 * 256)
#define SELFTEST_JUDGE_HZ_MAX   (-100 * 256)

static struct mutex xfer_lock;
static bool fuse_read_success;
/*--------------------------------------------------------------------
Function:AKM of RANDOM READ  register is read.
@param	: (1)start_reg:The first register number.
          (2)n_regs:Number of read registers.
          (3)akm->max_retry_count:Maximum retrying frequency.
output  :reg_data[0 - n_regs-1]:Read data.
I/O	    :When the reading error occurs, the following variable is updated.
          (1)akm->error_count:+1 
          (2)akm->retry_count:The generation frequency is multiplied retrying.
@retval :(1)0:Normal termination.
       	  (2)-EBUSY:I2C Communication fault (time out).
--------------------------------------------------------------------*/
static int akm_read_regs(uint8_t start_reg, uint8_t reg_data[], uint16_t n_regs)
{
	unsigned retry_count = 0;
	int ret;
	struct i2c_msg msg[2];
	uint8_t buf0[2];

	debug("%s:start(read)regs:0x%02X bytesize:%u)\n", __func__, start_reg, n_regs);

	//i2c_transfer Parameter setting
	msg[0].addr = akm->reg.st_client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = buf0;
	buf0[0] = start_reg;		// Reading beginning position setting

	msg[1].addr = akm->reg.st_client->addr;
	msg[1].flags = I2C_M_RD;	// Read the register value
	msg[1].len = n_regs;		// Reading number
	msg[1].buf = reg_data;		// Area where reading result is put

	for (;;) {
		debug("%s:i2c_transfer start \n", __func__);

		mutex_lock(&xfer_lock);
		if ((ret = i2c_transfer(akm->reg.st_client->adapter, msg, 2)) >= 0) {
			debug("%s:i2c_transfer normal end \n", __func__);
			mutex_unlock(&xfer_lock);
			break;				// Normal termination
		}
		mutex_unlock(&xfer_lock);

		debug("%s:i2c_transfer error end \n", __func__);
		
		if (ret == -ETIMEDOUT) {
			compass_printk(KERN_ERR, "%s: %s I2C TimeOut error 0x%02X ret(%d),retry_count %d\n",
										DEVNAME, __func__, start_reg, ret,retry_count);
			akm->error_count++;
			akm->retry_count += retry_count;
			return ret;
		}

		// When the error occurs

		if (retry_count >= MAX_RETRY_COUNT) {
			// When ..retrying.. frequency reaches the upper bound:error
			compass_printk(KERN_ERR, "%s: I2C receive error retry over (error code:%d)\n",
														DEVNAME, ret);
			akm->error_count++;
			akm->retry_count += retry_count;
			return ret;
		}
		// It retries. 
		retry_count++;
		
	}


	return ret;
}
/*--------------------------------------------------------------------
Function  :AKM of WRITE It writes it in the register.
@param  :(1) start_reg:The first register number.
          (2) reg_data[0 - n_regs-1]:Data written in register.
          (3) n_regs (<=AKM_MAX_CONTIGUOUS_REGS):Number of registers.
          (4) akm->max_retry_count:Maximum retrying frequency.
I/O:When the I/O error occurs, the following variable is updated. .
        (1) akm->error_count
        (2) akm->retry_count
@retval:(1) 0:Normal termination.
         (2) -EINVAL:n_regs is too large.
         (3) -EBUSY:I2C Communication fault (time out).
--------------------------------------------------------------------*/
static int akm_write_regs(uint8_t start_reg,
						   const uint8_t reg_data[], uint16_t n_regs)
{
	// buffer[]:Array that adds slave addr(W) 
	//			 and start_reg in front of reg_data[]. 
	uint8_t buffer[MAX_CONTIGUOUS_REGS + 2];
	uint8_t *dest;
	unsigned i;
	int ret;
	unsigned retry_count = 0;


	debug("%s(start(write)regs:0x%02X writecontent:%d bytesize:%u)\n", __func__, start_reg, reg_data[0], n_regs);

	if (n_regs > MAX_CONTIGUOUS_REGS) {
		ret = -EINVAL;
		debug("Too many registers (> %u).\n", MAX_CONTIGUOUS_REGS);
		return ret;
	}

	dest = buffer;
	*dest++ = start_reg;
	for (i = 0; i < n_regs; i++)
		*dest++ = reg_data[i];

	for (;;) {
		mutex_lock(&xfer_lock);
		ret = i2c_master_send(akm->reg.st_client, buffer, n_regs + 1);
		if (ret >= 0) {
			mutex_unlock(&xfer_lock);
			break;				// Normal termination
		}
		mutex_unlock(&xfer_lock);
		
		if (ret == -ETIMEDOUT) {
			compass_printk(KERN_ERR, "%s: %s I2C TimeOut error 0x%02X ret(%d),retry_count %d\n",
										DEVNAME, __func__, start_reg, ret,retry_count);
			akm->error_count++;
			akm->retry_count += retry_count;
			return ret;
		}
		// When the error occurs
		if (retry_count >= MAX_RETRY_COUNT) {
			// When ..retrying.. frequency reaches the upper bound:error
			compass_printk(KERN_ERR, "%s: I2C retry over 0x%02X ret(%d)\n", 
												DEVNAME, start_reg, ret);
			//ret = -EIO;
			akm->error_count++;
			akm->retry_count += retry_count;
			return ret;
		}
		// It retries. 
		retry_count++;
	}


	debug("%s -> return code:%d\n", __func__, ret);
	return ret;
}
/*--------------------------------------------------------------------
Function  :The operational mode of AKM is set.
@param    :mode:AKM_MODE_{STANBY,FORCESTATE}.
Attention :When it is active.Begin the temperature survey at the same time. 
@retval   :Success:0,Error: -errno.
error     :(1) EINVAL:mode is illegal.
            (2) other :akm_write_verify_regs() 
--------------------------------------------------------------------*/
static int akm_setmode(uint8_t mode)
{
	int ret;
	debug("%s\n", __func__);
	
	udelay(MODE_Wait);
	if (mode == MODE_SELF_TEST){
		mode = MODE_POWER_DOWN;
		ret = akm_write_regs(REG_CNTL ,&mode, 1);
		if(ret < 0){
	    	compass_printk(KERN_ERR, 
	    		"%s: %s mode_stanby Can't Write REG_CNTL.\n",
											DEVNAME, __func__);
	    	return ret;
	    }
		mode = SELFTEST_SET;
		ret = akm_write_regs(REG_ASTC ,&mode, 1);
		if(ret < 0){
	    	compass_printk(KERN_ERR, 
	    		"%s: %s selftest Can't Write REG_ASTC.\n",
											DEVNAME, __func__);
	    	return ret;
	    }
		udelay(MODE_Wait);
		mode = MODE_SELF_TEST;
	}

	if (mode == MODE_POWER_DOWN || mode == MODE_SINGLE_MEASUREMENT ||
				mode == MODE_SELF_TEST || mode == MODE_FUSE_ROM_ACCESS){
					
		ret = akm_write_regs(REG_CNTL ,&mode, 1);
	    if(ret < 0){
	    	compass_printk(KERN_ERR, 
	    		"%s: %s mode_stanby Can't Write REG_CNTL.\n",
											DEVNAME, __func__);
	    	return ret;
	                                  		
	    }
		ret = OK;
	}else{
		compass_printk(KERN_ERR,"%s:Don't use mode %d", __func__, mode);
		ret = -EINVAL;
	}
	
	debug("%s(mode select:0x%02X) -> return code:%d\n", __func__, mode, ret);
		
	return ret;
	
}

/*--------------------------------------------------------------------
Function  :Port that AKM uses is initialized. 
--------------------------------------------------------------------*/
static void akm_port_init(void)
{
	debug("%s\n", __func__);

	gpio_request(JIKI_DRDY_GPIO, "compass");
	gpio_direction_input(JIKI_DRDY_GPIO);
}

/*--------------------------------------------------------------------
Function  :register of akm is read. 
output    :akm->reg:content of shcd
@retval   :Success:0,Error: -errno.
error     :shcd_{read,write}_verify_regs() 
--------------------------------------------------------------------*/
static int akm_read_reg_all(void)
{
	int ret;
	uint8_t buffer[OUTPUTDATA_AND_CHECKCODE_REGS];

	debug("%s start\n", __func__);
	
	ret = akm_read_regs(REG_ST1 ,buffer, OUTPUTDATA_AND_CHECKCODE_REGS);
	if(ret <0){
		compass_printk(KERN_ERR, "%s: %s Can't read REGdata(REG_ST1~REG_ST2). ret=%d\n",
												DEVNAME, __func__, ret);
		return ret;
	}
	akm->reg.data.x = buffer[1] + (buffer[2] << 8);
	akm->reg.data.y = buffer[3] + (buffer[4] << 8);
	akm->reg.data.z = buffer[5] + (buffer[6] << 8);
	
	
	ret = akm_read_regs(REG_ASTC ,buffer ,1);
	if(ret <0){
		compass_printk(KERN_ERR, "%s: %s Can't read REG_ASTC. ret=%d\n",
												DEVNAME, __func__, ret);
		return ret;
	}
	
	ret = akm_read_regs(REG_I2CDIST ,buffer ,1);
	if(ret <0){
		compass_printk(KERN_ERR, "%s: %s Can't read REG_I2CDIST. ret=%d\n",
												DEVNAME, __func__, ret);
		return ret;
	}
	
	ret = akm_setmode(MODE_FUSE_ROM_ACCESS);
	if(ret <0){
			compass_printk(KERN_ERR, "%s: %s Can't modeset MODE_FUSE_ROM_ACCESS. ret=%d\n",
												DEVNAME, __func__, ret);
			return ret;
	}
	
	ret = akm_read_regs(REG_ASAX, buffer, 3);
	
	akm_setmode(MODE_POWER_DOWN);
	
	if (ret < 0){
		compass_printk(KERN_ERR, "%s: %s Can't read REG_ASAX. ret=%d\n",
												DEVNAME, __func__, ret);
		return ret;
	}
	
	fuse_read_success = true;
	
	akm->reg.fuse.x = buffer[0];
	akm->reg.fuse.y = buffer[1];
	akm->reg.fuse.z = buffer[2];

	debug("%s:FuseRomDATA:(X:%d Y:%d Z:%d)\n",
					__func__, akm->reg.fuse.x, akm->reg.fuse.y, akm->reg.fuse.z);
	
	return ret;
}

/*--------------------------------------------------------------------
Function  :AKM reset and initialize the driver.
@param    :first:Initialization : true
                   Reinitialize   : false
--------------------------------------------------------------------*/
static void akm_driver_init(void)
{
	debug("%s\n", __func__);

	// Set the default collation and retry
//	akm->max_retry_count = 3;

	// *akm Reset.
	SETSTATE(STAT_IDLE);	// Idle state

	// past measuring data is invalidated. 
	akm->reg.data.x = 0;
	akm->reg.data.y = 0;
	akm->reg.data.z = 0;
	
	fuse_read_success = false;
	
	// read register
	akm_read_reg_all();

}

/*--------------------------------------------------------------------
Function  :Driver's termination. 
--------------------------------------------------------------------*/
static void akm_driver_term(void)
{
	debug("%s\n", __func__);

	// *akm Reset.
	SETSTATE(STAT_IDLE);	// Idle state

	// past measuring data is invalidated. 
	akm->reg.data.x = 0;
	akm->reg.data.y = 0;
	akm->reg.data.z = 0;
	
	fuse_read_success = false;
}

/*--------------------------------------------------------------------
Function  :Module initialization.
--------------------------------------------------------------------*/
void akm_module_init(void)
{
	mutex_init(&xfer_lock);
	init_waitqueue_head(&akm->measure_end_queue);
	akm_port_init();	// GPIO Initialization
	akm_driver_init();
}

/*--------------------------------------------------------------------
Function  :Module termination.
--------------------------------------------------------------------*/
void akm_module_term(void)
{
	gpio_free(JIKI_DRDY_GPIO);
}

/*--------------------------------------------------------------------
Function  :interrupt handler of AKM.
@param    :(1) irq:IRQ Number.
            (2) data:request_irq() ,Device-specific data (not used).
output    :akm->measurement
            (1) time:End time measurement
@retval   :AKM Interrupt: IRQ_HANDLED,other: IRQ_NONE.
--------------------------------------------------------------------*/
static irqreturn_t akm_isr(int irq, void *data)
{
	compass_data_t *const m = &akm->measurement;

	debug("%s(IRQ No:%d, input data:%p)\n", __func__, irq, data);

	// If GPIO has not been generated, disregards. 
//	if (unlikely(!GPIO_ISACTIVE()))
//		return IRQ_NONE;

	if ( STATE() == STAT_MEASURING ){
		SETSTATE(STAT_MEASURED);	// to measurement completion state.

		// Recording Time.
		do_gettimeofday(&m->time);

		// The task is restarted. 
		wake_up_interruptible(&akm->measure_end_queue);
	}

	debug("%s end\n", __func__);

	return IRQ_HANDLED;
}

/*--------------------------------------------------------------------
Function  :Clear interrupt flag AKM.
output    :akm->reg.data
@retval   :Success:OK,Error: -errno.
error     :(1) error number:I/O error.
            (2) other:akm_read_regs()
--------------------------------------------------------------------*/
static int akm_clear_interrupt(void)
{
	bool error = false;
	unsigned retry_count = 0;
	int ret = OK;
	unsigned nRegs;
	uint8_t buffer[MAX_CONTIGUOUS_REGS];

	debug("%s start\n", __func__);

	if (likely(GPIO_ISACTIVE())) {

		debug("%s gpio active\n", __func__);

		for (;;) {
			// When the GPIO signal is High:
			// Clear interrupt.
			nRegs = sizeof(akm->reg.data);
			if ((ret = akm_read_regs(REG_XLSB, buffer, nRegs)) >= 0) {

				// check GPIO is Low .
				if (likely(!GPIO_ISACTIVE()))
					break;		// Success
			}
			// At the error
			error = true;
			if (retry_count >= MAX_RETRY_COUNT) {
				// when retries reach upper limit:error
				compass_printk(KERN_ERR, "Error: Unable to clear GPIO%d.\n", JIKI_DRDY_GPIO);
				break;
			}
			// retries
			retry_count++;
		}
	}

	if (error) {
		// error/retries total.
		akm->error_count++;
		akm->retry_count += retry_count;
		compass_printk(KERN_ERR, "%s: %s ret(%d) retry_count %d\n",
					DEVNAME, __func__, ret, retry_count);
	}

	debug("%s -> return code:%d\n", __func__, ret);

	return ret;
}

/*--------------------------------------------------------------------
Function  :start the measurement of magnetic.
@retval   :(1) 0:Success.
            (2) -EBUSY:I2C Communication error (time out).
            (3) -EIO  :I/O error.
--------------------------------------------------------------------*/
static int akm_measure_start(void)
{
	int ret;

	debug("%s\n", __func__);

	// Clear interrupt AKM.
	if ((ret = akm_clear_interrupt()) >= 0) {
		// Measurement start.
		SETSTATE(STAT_MEASURING);

		ret = akm_setmode(MODE_SINGLE_MEASUREMENT);
		if(ret <0){
			compass_printk(KERN_ERR, "%s: Error akm_setmode() ret=%d\n", __func__, ret);
		}
	}
	return ret;
}

/*--------------------------------------------------------------------
Function  :measuring data of terrestrial magnetism read.
output    :akm->measurement:Result of a measurement.
@retval   :akm_read_verify_regs() 
--------------------------------------------------------------------*/
static int akm_measure_end(void)
{
	int ret;
//	unsigned nRegs;
	uint8_t buffer[OUTPUTDATA_AND_CHECKCODE_REGS];
	compass_data_t *const m = &akm->measurement;

	debug("%s\n", __func__);
	
	ret = akm_read_regs(REG_ST1, buffer, OUTPUTDATA_AND_CHECKCODE_REGS);
	if(ret <0){
		compass_printk(KERN_ERR, "%s: %s Can't read REG_ST1. ret=%d\n",
												DEVNAME, __func__, ret);
		return ret;
	}
	
	SETSTATE(STAT_AVAILABLE);
	
	if(buffer[0] & 0x01){
		if(!buffer[7]) {
			// MSB/LSB of buffer is replaced
			akm->reg.data.x = buffer[1] + (buffer[2] << 8);
			akm->reg.data.y = buffer[3] + (buffer[4] << 8);
			akm->reg.data.z = buffer[5] + (buffer[6] << 8);

			debug("%s:MeasurementDATA:(X:%d Y:%d Z:%d)\n",
							__func__, akm->reg.data.x,
							akm->reg.data.y, akm->reg.data.z);

			// The measuring data is copied onto akm->measurement. 
			m->magnetism[0] = akm->reg.data.x;	// DATAX
			m->magnetism[1] = akm->reg.data.y;	// DATAY
			m->magnetism[2] = akm->reg.data.z;	// DATAZ
			
			ret = OK;

			// Measurement data acquisition completion.
			
			
		}else if(buffer[7] & 0x04){
			debug("%s:Measurement_Data_err",__func__);
			ret = -DERR;
		}else if(buffer[7] & 0x08){
			debug("%s:Measurement_Data_Over_Frow",__func__);
			ret = -HOFL;
		}else{
			debug("REG_ST2:Error outside assumption. bitcode:0x%04X",buffer[7]);
			ret = -EIO;
		}
		
		
			
	}else{
		debug("%s:Output_Cannot_Be_Prepared",__func__);
		ret = -EFAULT;
	}
		

	return ret;
}

/*--------------------------------------------------------------------
Function  :start the measurement of magnetic,and Getting Results.
output    :result of a measurement, 
			and opening and shutting and state of the turn of the case.
	        *measurement
	        akm->measurement
	        akm->reg.temp
	        akm->reg.data
@retval   :(1) 0:success
            (2) -EBUSY:I2C Communication error (time out).
            (3) -EIO:I/O error retries reach upper limit.
            (4) EINTR:Received a signal while waiting for measurement.
            (5) EFAULT:measurement adress Injustice.
--------------------------------------------------------------------*/
static int akm_ioctl_measure(compass_data_t __user * measurement)
{
	int ret;

	debug("%s(%p) start\n", __func__, measurement);

	if ((ret = down_interruptible(&akm_sem)) >= 0) {
		// Exclusive control start
		
		ret = copy_from_user(&akm->measurement.measurement_range,
						&measurement->measurement_range,
						sizeof(measurement->measurement_range));
		if (ret != 0) {
			debug("%s:Can't copy to user area. return code:%u\n", __func__, ret);
			ret = -EFAULT;
			goto Exit;
		}
		
		debug("STATE:%d (0:STAT_IDLE 1:STAT_MEASURING 2:STAT_MEASURED 3:STAT_AVAILABLE)\n", STATE());
		if ((ret = akm_measure_start()) < 0)
			goto Exit;

		// Measurement start success
  		enable_irq(AKM_IRQ());		// GPIO Permission
		
		// Waiting for  measurement complete
		debug("%s:loop in\n", __func__);

		{
			const long timeout = msecs_to_jiffies(MAX_MEASUREMENT_TIME);

			//STAT Reading confirmation
			debug("STATE:%d (0:STAT_IDLE 1:STAT_MEASURING 2:STAT_MEASURED 3:STAT_AVAILABLE)\n", STATE());

			//During the measurement:wait
			ret = wait_event_interruptible_timeout(akm->measure_end_queue,
				(STATE() == STAT_MEASURED), timeout);
			debug("STATE:%d (0:STAT_IDLE 1:STAT_MEASURING 2:STAT_MEASURED 3:STAT_AVAILABLE)\n", STATE());
			debug("%s: wait_event_interruptible_timeout ret=%d\n",__func__, ret);

			disable_irq(AKM_IRQ());

			if (ret < 0) {
				goto Exit;
			}
			else if (ret == 0) {
				// time out:condition is revalued. 
				if( STATE() != STAT_MEASURED ) {
					compass_printk(KERN_ERR, 
						"%s: measurement wait event error. timeout(%ld)\n", DEVNAME, timeout);
					ret = -EBUSY;
					goto Exit;
				}
				debug("%s: measurement wait event timeout(%ld). but measured.\n",__func__, timeout);
			}
		}

		debug("%s:loop out\n", __func__);

		if ((ret = akm_measure_end()) < 0)
			goto Exit;

		SETSTATE(STAT_IDLE);

		// Return the number of retries and error
		akm_update_error_count();
		akm->measurement.total_error_count = akm->total_error_count;
		akm->measurement.total_retry_count = akm->total_retry_count;
		if (copy_to_user(measurement, &akm->measurement, 
			sizeof(*measurement)) != 0) {
			debug("%s:Can't copy to user area.return code:%u\n", __func__, ret);
			ret = -EFAULT;
		}

Exit:
		akm_update_error_count();
		up(&akm_sem);			// Exclusive control end
	}

	debug("%s -> return code:%d\n", __func__, ret);

	return ret;
}

/*--------------------------------------------------------------------
Function  :AKM device is opened.  
@param    :(1) inode:inode Information of Device file (Unused).
            (2) file:open file Structure.
@retval   :(1) 0:success.
            (2) -EACCES:mode is (O_RDONLY) except.
            (3) -EINTR:Interrupt occurs during execution.
            (4) other : request_irq() 
--------------------------------------------------------------------*/
int akm_open(struct inode *inode, struct file *file)
{
	int ret = OK;
	uint8_t mode;

	debug("%s:start\n", __func__);
	debug("IRQ%u\n", AKM_IRQ());
	
	mode = MODE_POWER_DOWN;
	
	// mode is (O_RDONLY) excep EACCES.
	
	if (file->f_mode & FMODE_WRITE) {
		ret = -EACCES;
		compass_printk(KERN_ERR, 
			"%s: Write operation not allowed mode=0x%02X.\n",
												DEVNAME, file->f_mode);
		goto Exit;
	}

	if ((ret = down_interruptible(&akm_sem)) >= 0) {
		// Exclusive control start
		if (akm->use_count == 0) {
			// When not in use:Initialization
			akm_driver_init();

			// Handler registration
			ret = request_irq(AKM_IRQ(), akm_isr, IRQ_TYPE_EDGE_RISING,
							DEVNAME, &akm);
			if (unlikely(ret < 0)) {
				debug("%s: Can't use IRQ%u.   ret=%d\\n", DEVNAME,
													AKM_IRQ(),ret);
				up(&akm_sem);		// Exclusive control end
				goto Exit;
			}
			
  			disable_irq(AKM_IRQ());
			
			
		}
		akm->use_count++;

		// changes to an active mode
		ret = akm_write_regs(REG_CNTL, &mode, 1);
		if (ret < 0) {
			compass_printk(KERN_ERR, "%s: %s  Can't Write AKM_CNTL1.\n",
												DEVNAME, __func__);
		} else {
			ret = OK;
		}

		up(&akm_sem);		// Exclusive control end
	}

Exit:
	debug("%s() -> return code:%d\n", __func__, ret);
	return ret;
}

/*--------------------------------------------------------------------
Function  :AKM device is close
@param    :(1) inode:inode Information of Device file (Unused).
            (2) file:close file Structure.
@retval   :(1) 0:success.
            (2) -EINTR:Interrupt occurs during execution.
--------------------------------------------------------------------*/
int akm_release(struct inode *inode, struct file *file)
{
	int ret;

	debug("%s\n", __func__);

	if ((ret = down_interruptible(&akm_sem)) >= 0) {
		if (akm->use_count > 0) {
			akm->use_count--;
		
			//  Exclusive control start
			if (akm->use_count == 0) {
				// When becoming unused
				// GPIO Interruption prohibition,IRQ Liberating.
	  			disable_irq(AKM_IRQ());
				free_irq(AKM_IRQ(), &akm);

				// AKM Reset
				akm_driver_term();
			}
		}
		up(&akm_sem);			//  Exclusive control end
	}
	debug("akm_release end ret=%d\n", ret);
	return ret;
}

/*--------------------------------------------------------------------
Function  :Get AKM register contents and status of the case.
@param    :regs->mask:Select a group mask register to read.
	        The combination of the following flags.
	        CMPS_REGMASK_DATA:Previous measurements (TEMP,DATA[XYZ])
	        CMPS_REGMASK_OFFSET:Sensor offset (OFF[XYZ])
	        CMPS_REGMASK_AMPGAIN:Sensor amplifier gain (AMP)
output    :member of *regs.
	        temp,data:TEMP,The contents of the register of DATA[XYZ].
	          (mask.CMPS_REGMASK_DATA = ON)
	        off:The contents of the register of OFF[XYZ].
	          (mask.CMPS_REGMASK_OFFSET = ON)
	        amp:The contents of the register of AMP.
	          (mask.CMPS_REGMASK_AMPGAIN = ON)
@retval   :(1) 0:success.
	        (2) -EBUSY:I2C Communication fault (time out).
	        (3) -EIO:I/O error retries reach upper limit.
	        (4) EINTR:Received a signal while waiting for Exclusive control.
	        (5) EFAULT:regs adress Injustice.
--------------------------------------------------------------------*/
static int akm_ioctl_getregs(cmps_registers_t __user * regs)
{
	uint8_t mask, buffer[MAX_CONTIGUOUS_REGS];
	unsigned nRegs;
	int ret;
	debug("%s\n", __func__);
	
	if ((ret = down_interruptible(&akm_sem)) >= 0) {
		// Exclusive control start
		debug("%s:Exclusive control start", __func__);
		if (copy_from_user(&mask, &regs->mask, sizeof(mask)) != 0) {
			debug("%s:Can't copy from user area. ret=%d\n", __func__,ret);
			goto CopyFailed;
		}
		if (mask & CMPS_REGMASK_DATA) {
			nRegs = sizeof(regs->data);
			ret = akm_read_regs(REG_XLSB, buffer, nRegs);
			if (ret < 0)
				goto Exit;
			// buffer in MSB / LSB swap the set xyz
			akm->reg.data.x = buffer[0] + (buffer[1] << 8);
			akm->reg.data.y = buffer[2] + (buffer[3] << 8);
			akm->reg.data.z = buffer[4] + (buffer[5] << 8);
			if (copy_to_user(&regs->data, &akm->reg.data, nRegs) != 0)
				goto CopyFailed;
		}
		
		if( fuse_read_success == false ) {
			ret = akm_setmode(MODE_FUSE_ROM_ACCESS);
			if(ret <0){
				compass_printk(KERN_ERR, "%s: %s Can't modeset MODE_FUSE_ROM_ACCESS. ret=%d\n",
													DEVNAME, __func__, ret);
				goto Exit;
			}
			
			ret = akm_read_regs(REG_ASAX, buffer, 3);
			
			akm_setmode(MODE_POWER_DOWN);
			
			if (ret < 0)
				goto Exit;
			
			akm->reg.fuse.x = buffer[0];
			akm->reg.fuse.y = buffer[1];
			akm->reg.fuse.z = buffer[2];
			
			fuse_read_success = true;
		}
		
		if (copy_to_user(&regs->fuse, &akm->reg.fuse, sizeof(regs->fuse)) != 0){
			goto CopyFailed;
		}
		
		ret = OK;
		goto Exit;

CopyFailed:
		debug("%s:Can't copy user area.\n", __func__);
		ret = -EFAULT;

Exit:
		akm_update_error_count();
		up(&akm_sem);			// Exclusive control end
	}
	return ret;
}

/*--------------------------------------------------------------------
Function  :selftest of AKM is executed. 
@param    :selfdata->testmode:select test mode.
output    :member of *selfdata.
	        test_ret:result value of self test
	        err_code:STBB value of self test
	          (test_mode = modeB)
@retval   :(1) 0:success.
	        (2) -EBUSY:I2C Communication fault (time out).
	        (3) -EIO:I/O error retries reach upper limit.
	        (4) -EINTR:Received a signal while waiting for Exclusive control.
	        (5) -EFAULT:regs adress Injustice.
--------------------------------------------------------------------*/
static int akm_ioctl_selftest(selftest_data_t __user * selfdata)
{
	int ret;
	uint8_t buffer[OUTPUTDATA_AND_CHECKCODE_REGS];
	
	if ((ret = down_interruptible(&akm_sem)) >= 0) {
		
		// Clear interrupt AKM.
		ret = akm_clear_interrupt();
		if (ret < 0) {
			up(&akm_sem);
			return ret;
		}

		SETSTATE( STAT_MEASURING );

		ret = akm_setmode(MODE_SELF_TEST);
		if(ret <0){
			compass_printk(KERN_ERR, "%s: %s Can't modeset MODE_SELF_TEST. ret=%d\n",
												DEVNAME, __func__, ret);
			goto Exit;
		}
		enable_irq(AKM_IRQ());
		
		debug("%s start\n",__func__);
		
		///////////////////////////////////////////////////////////////////////////
		{
			const long timeout = msecs_to_jiffies(MAX_MEASUREMENT_TIME);

			//STAT Reading confirmation
			debug("STATE:%d (0:STAT_IDLE 1:STAT_MEASURING 2:STAT_MEASURED 3:STAT_AVAILABLE)\n", STATE());

			//During the measurement:wait
			ret = wait_event_interruptible_timeout(akm->measure_end_queue,
				(STATE() == STAT_MEASURED), timeout);
			debug("STATE:%d (0:STAT_IDLE 1:STAT_MEASURING 2:STAT_MEASURED 3:STAT_AVAILABLE)\n", STATE());
			debug("%s: wait_event_interruptible_timeout ret=%d\n",__func__, ret);

			disable_irq(AKM_IRQ());

			if (ret < 0) {
				goto Exit;
			}
			else if (ret == 0) {
				// time out:condition is revalued. 
				compass_printk(KERN_ERR, 
					"%s: measurement wait event error. timeout(%ld)\n", DEVNAME, timeout);
				ret = -EBUSY;
				goto Exit;
			}
		}
		//////////////////////////////////////////////////////////////////////////////

		
		SETSTATE(STAT_IDLE);

		ret = akm_read_regs(REG_ST1 ,buffer, OUTPUTDATA_AND_CHECKCODE_REGS);
		if(ret <0){
			compass_printk(KERN_ERR, "%s: %s Can't read REG_ST1. ret=%d\n",
												DEVNAME, __func__, ret);
			goto Exit;
		}
		
		debug("0x%02X:REG_ST1 ",buffer[0]);
		debug("0x%02X:REG_ST2 ",buffer[7]);
		
		if(buffer[0] & 0x01){
			if(!buffer[7]) {
				cmps_xyz_t data;
				long adj_x, adj_y, adj_z;

				// MSB/LSB of buffer is replaced
				data.x = buffer[1] + (buffer[2] << 8);
				data.y = buffer[3] + (buffer[4] << 8);
				data.z = buffer[5] + (buffer[6] << 8);

				debug("%s:MeasurementDATA:(X:%d Y:%d Z:%d)\n",
								__func__, data.x, data.y, data.z);

				debug("%s:FuseRomDATA:(X:%d Y:%d Z:%d)\n",
								__func__, akm->reg.fuse.x, akm->reg.fuse.y, akm->reg.fuse.z);

				adj_x = H_ADJ(data.x, akm->reg.fuse.x);
				adj_y = H_ADJ(data.y, akm->reg.fuse.y);
				adj_z = H_ADJ(data.z, akm->reg.fuse.z);
				
				debug("%s:AdjustmentDATA:(X:%ld Y:%ld Z:%ld)\n", __func__, adj_x, adj_y, adj_z);

				if( (adj_x < SELFTEST_JUDGE_HX_MIN) || (adj_x > SELFTEST_JUDGE_HX_MAX) ) {
					debug("%s:Error judge HX OutOfRange(min:%d max:%d)\n", 
								__func__, SELFTEST_JUDGE_HX_MIN, SELFTEST_JUDGE_HX_MAX);
					ret = -DERR;
				} else if( (adj_y < SELFTEST_JUDGE_HY_MIN) || (adj_y > SELFTEST_JUDGE_HY_MAX) ) {
					debug("%s:Error judge HY OutOfRange(min:%d max:%d)\n", 
								__func__, SELFTEST_JUDGE_HY_MIN, SELFTEST_JUDGE_HY_MAX);
					ret = -DERR;
				} else if( (adj_z < SELFTEST_JUDGE_HZ_MIN) || (adj_z > SELFTEST_JUDGE_HZ_MAX) ) {
					debug("%s:Error judge HZ OutOfRange(min:%d max:%d)\n", 
								__func__, SELFTEST_JUDGE_HZ_MIN, SELFTEST_JUDGE_HZ_MAX);
					ret = -DERR;
				} else {
					ret = OK;
				}
			}else if(buffer[7] & 0x04){
				debug("%s:Selftest_Data_err",__func__);
				ret = -DERR;
			}else if(buffer[7] & 0x08){
				debug("%s:Selftest_Data_Over_Frow",__func__);
				ret = -HOFL;
			}else{
				debug("REG_ST2:Error outside assumption. bitcode:0x%04X",buffer[7]);
				ret = -EIO;
			}
		}else{
			ret = -EFAULT;
			compass_printk(KERN_ERR, "%s: %s Can't modeset MODE_SELF_TEST. ret=%d\n",
												DEVNAME, __func__, ret);
		}
	
Exit:
		buffer[0] = RESET_CODE;
		akm_write_regs(REG_ASTC ,&buffer[0] ,1);
		akm_setmode(MODE_POWER_DOWN);
		up(&akm_sem);			// Exclusive control end
	}
	
	return ret;
}

/*--------------------------------------------------------------------
Function  :ioctl() System call processing
@param    :(1) inode :inode Information of Device file (Unused).
	        (2) file :file descriptor of ioctl() file Structure (Unused).
	        (3) command:command number.
	            COMPASS_IOC_GETDATA:Magnetic measurements
	            COMPASS_IOC_GETREGS:Register of AKM to read the content.
	            COMPASS_IOC_SETREGS:Write data register of AKM.
	            COMPASS_IOC_GETDATASTOP:exit the magnetic measurements
	            COMPASS_IOC_SELFTEST:Self Test
	        (4) arg:Command Arguments.
	            COMPASS_IOC_GETDATA (Type:compass_data_t*):
	            COMPASS_IOC_GETREGS 
	            COMPASS_IOC_SETREGS (Type:const cmps_registers_t*):
	            COMPASS_IOC_GETDATASTOP  
	            COMPASS_IOC_SELFTEST (Type:selftest_data_t*)
@retval   :(1) 0:success.
	        (2) -EINVAL:Undefined command number.
	        (3) -EBUSY:I2C Communication fault (time out).
	        (4) -EIO:I/O error retries reach upper limit.
	        (5) -EFAULT:arg adress Injustice.
--------------------------------------------------------------------*/
#if 1   /* FUJITSU:2011-12-01 ICS mod start */
long akm_ioctl( struct file *file, unsigned command, u_long arg )
#else
int akm_ioctl(struct inode *inode, struct file *file,
			   unsigned command, u_long arg)
#endif  /* FUJITSU:2011-12-01 ICS mod end */
{
	int ret;

	debug("%s(command:0x%02X, arg:0x%08lX)\n", __func__, command, arg);


	switch (command) {
	case COMPASS_IOC_GETDATA:
		ret = akm_ioctl_measure((compass_data_t __user *)arg);
		break;
	case COMPASS_IOC_TEMP_COR:
		ret = -ENOTSUPP;
		break;
	case COMPASS_IOC_GETREGS:
		ret = akm_ioctl_getregs((cmps_registers_t __user *)arg);
		break;
	case COMPASS_IOC_SETREGS:
		ret = -ENOTSUPP;
		break;
	case COMPASS_IOC_OFFCAL:
		ret = -ENOTSUPP;
		break;
	case COMPASS_IOC_GETDATASTOP:
		ret = -ENOTSUPP;
		break;
	case COMPASS_IOC_SELFTEST:
		ret = akm_ioctl_selftest((selftest_data_t __user *)arg);
		break;
	default:
		ret = -EINVAL;		// Undefined command number
		compass_printk(KERN_ERR, "%s: %s command error.(0x%08X)\n",
									DEVNAME, __func__, command);
		break;
	}

	debug("%s() -> return code:%d\n", __func__, ret);

	return ret;
}

MODULE_AUTHOR ("FUJITSU,2011");
MODULE_DESCRIPTION ("compass driver");
MODULE_LICENSE ("GPL");
