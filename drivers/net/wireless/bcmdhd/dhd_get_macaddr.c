#include <../../../../arch/arm/mach-msm/proc_comm.h>
#include <linux/kernel.h>

#include <linux/delay.h> /* for msleep() */

extern int read_macaddr(unsigned char *addr_buf)
{
	unsigned nvid = 4678;	/* NV_WLAN_MAC_ADDRESS_I */
	unsigned data;

// FUJITSU 2012.06.20 WLAN/BT NV Access Change start
	int      ret = 0;
	int      cnt = 0;
	
#define RTC_NV_DURING_EFS_SYNC_S   (-16)  /* NV NOT ACTIVE STATUS */
#define RTC_NV_RW_WAIT_TIME        500    /* NV Read/Write Retry wait time */
    for(;;) {
        ret = msm_proc_comm(PCOM_OEM_011, &nvid, &data); 
        if(ret != RTC_NV_DURING_EFS_SYNC_S) break; // NV_NOTACTIVE_S
        if(cnt == 20) break;
        msleep(RTC_NV_RW_WAIT_TIME); //100 msec
        printk("%s: msm_proc_comm warning NV ACCESS count=%d\n", __FUNCTION__, cnt);
        cnt++;
    }

//	ret = msm_proc_comm(PCOM_OEM_011, &nvid, &data); 
// FUJITSU 2012.06.20 WLAN/BT NV Access Change end 

	if (ret == 0)
	{
		/* Success */
		addr_buf[0] = data		& 0xFF;
		addr_buf[1] = (data>>8) & 0xFF;
		addr_buf[2] = (data>>16)& 0xFF;
		addr_buf[3] = (data>>24)& 0xFF;
		addr_buf[4] = nvid		& 0xFF;
		addr_buf[5] = (nvid>>8) & 0xFF;
	}
	else
	{
		/* Fail */
		printk("%s: msm_proc_comm err ret=%d\n", __FUNCTION__, ret);
	}

	return ret;
}

