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

/*============================================================================
    INCLUDE FILES FOR MODULE
============================================================================*/
#include <linux/clk.h>
#include <mach/clk.h>
#include <mach/gpio.h>
#include "msm_fb.h"
#include "mddihost.h"
#include "mddihosti.h"

/*===========================================================================
    LOCAL FUNCTIONS PROTOTYPES
============================================================================*/
#define BD6184_I2C_SLAVE_ADDR 0x76
static struct i2c_adapter *i2c_bkl;

typedef enum {
    BKL_STATE_OFF,
    BKL_STATE_1ST_ON,
    BKL_STATE_ON
} mddi_R61408_bkl_state_e;

mddi_R61408_bkl_state_e mddi_R61408_bkl_state;

#define MDDI_COMMAND_MAX_NUM    20

typedef struct {
    uint32 cmd;
    uint32 value;
} drv_mddi_cmd_table;

//Display Driver State
typedef enum {
    LCD_STATE_INIT,
    LCD_STATE_OFF,
    LCD_STATE_WAIT_UPDATE,
    LCD_STATE_WAIT_DISPLAY_ON,
    LCD_STATE_ON
} lcd_state_e;

//Dipslay Driver
typedef enum {
    LCD_OFF,
    LCD_ON
} lcd_info_e;

static void drv_display_control(lcd_info_e lcd_info);
//static void drv_display_reset(void);/* FUJITSU:2011-09-14 delete  system_rev check */
static void drv_display_reset_low(void);
static void drv_display_reset_high(void);
static void drv_display_poweron(void);
static void drv_display_poweroff(void);
static void drv_display_mddi_cmd(drv_mddi_cmd_table *cmd_table);
static void drv_display_enter_sleep_mode(void);
static void drv_display_exit_sleep_mode(void);
static void drv_display_set_display_off(void);
static void drv_display_set_display_on(void);

static void drv_display_delay_on(struct work_struct *ignored);
static DECLARE_WORK(display_on_wq, drv_display_delay_on);
void mddi_panel_fullscrn_update_notify(void);
void mddi_panel_updatedone_notify(void);
extern int factory_mode(void);

static lcd_state_e lcd_state = LCD_STATE_INIT;

//command

static drv_mddi_cmd_table cmd_2A[] =
{
{0x2A, 0xDF010000},
{0xFFFFFFFF, 0xFFFFFFFF}
};

static drv_mddi_cmd_table cmd_2B[] =
{
{0x2B, 0x1F030000},
{0xFFFFFFFF, 0xFFFFFFFF}
};

//Tearing effect line on
static drv_mddi_cmd_table cmd_35[] =
{
{0x35,  0x00000000},
{0xFFFFFFFF, 0xFFFFFFFF}
};

static drv_mddi_cmd_table cmd_36[] =
{
{0x36, 0x00000000},
{0xFFFFFFFF, 0xFFFFFFFF}
};

/* FUJITSU:2011-08-25 change panel settings start */
static drv_mddi_cmd_table cmd_3A[] =
{
//{0x3A, 0x00000066},
{0x3A, 0x00000077},
{0xFFFFFFFF, 0xFFFFFFFF}
};
/* FUJITSU:2011-08-25 change panel settings end */

//sleep out command and wait
static drv_mddi_cmd_table cmd_11[] =
{
{0x11,  0x00000000},
{0xFFFFFFFF, 0xFFFFFFFF}
};

//sleep in command and wait
static drv_mddi_cmd_table cmd_10[] =
{
{0x10,  0x00000000},
{0xFFFFFFFF, 0xFFFFFFFF}
};

//display on command
static drv_mddi_cmd_table cmd_29[] =
{
{0x29,  0x00000000},
{0xFFFFFFFF, 0xFFFFFFFF}
};

//display off command
static drv_mddi_cmd_table cmd_28[] =
{
{0x28,  0x00000000},
{0xFFFFFFFF, 0xFFFFFFFF}
};

/* FUJITSU:2011-09-21 change panel settings start */
//	Manufacturer Command Access Protect
static drv_mddi_cmd_table cmd_B0_protect[] =
{
{0xB0,  0x00000004},
{0xFFFFFFFF, 0xFFFFFFFF}
};

//	Manufacturer Command Access Protect
static drv_mddi_cmd_table cmd_B0_unprotect[] =
{
{0xB0,  0x00000003},
{0xFFFFFFFF, 0xFFFFFFFF}
};

//	MDDI Control
static drv_mddi_cmd_table cmd_B7[] =
{
{0xB7,  0x25117280},
{0xFFFFFFFF, 0xFFFFFFFF}
};
/* FUJITSU:2011-09-21 change panel settings end */

static void drv_display_mddi_cmd(drv_mddi_cmd_table *cmd_table)
{
    uint32 i = 0;
    uint32 value[MDDI_COMMAND_MAX_NUM] = {0};
    uint32 cmd;
    
    int ret = 0;
    
    cmd = cmd_table[i].cmd;
    
    while (cmd_table[i].cmd != 0xFFFFFFFF || cmd_table[i].value != 0xFFFFFFFF ) {
        if(i >= MDDI_COMMAND_MAX_NUM){
            printk(KERN_ERR "[DD]%s: command buffer overflow cmd=(0x%08X)\n", __func__, cmd_table[i].cmd);
            break;
        }
        if (cmd_table[i].cmd == 0 && cmd_table[i].value) {
            mddi_wait(cmd_table[i].value);
        }else if(cmd == cmd_table[i].cmd && cmd == cmd_table[i+1].cmd){
            value[i] = cmd_table[i].value;
        }else{
            value[i] = cmd_table[i].value;
            ret = mddi_host_register_multiwrite(cmd_table[i].cmd, value, i+1, TRUE, NULL, MDDI_HOST_PRIM);
            if (ret != 0) {
                printk(KERN_ERR "[LCD]%s: mddi_host_register_multiwrite() cmd=0x%08X, value[%d]=0x%08X, count=%d  ret=%d\n", __func__, cmd_table[i].cmd, i, value[i], i+1, ret);
            }
        }
        cmd = cmd_table[i].cmd;
        i++;
    }
    return;
}

void mddi_panel_fullscrn_update_notify(void)
{
    if (lcd_state == LCD_STATE_WAIT_UPDATE) {
        printk(KERN_INFO "[LCD]%s FullScreen Update..\n",__func__);
        lcd_state = LCD_STATE_WAIT_DISPLAY_ON;
    }
}

void mddi_panel_updatedone_notify(void)
{
    if (lcd_state == LCD_STATE_WAIT_DISPLAY_ON) {
        printk(KERN_INFO "[LCD]%s FullScreen Updated! set work queue.\n",__func__);
        schedule_work(&display_on_wq);
    }
}


static void drv_display_delay_on(struct work_struct *ignored)
{
    if (lcd_state == LCD_STATE_WAIT_DISPLAY_ON) {
        drv_display_set_display_on();
        printk(KERN_INFO "[LCD]%s Display ON\n",__func__);
        lcd_state = LCD_STATE_ON;
        //wait 20ms
        mddi_wait(20);
    }
    else {
        printk(KERN_ERR "%s: lcd_state (%d).\n", __func__, lcd_state);
    }
}

static void drv_display_set_display_on(void)
{
    drv_display_mddi_cmd(cmd_29);
/* FUJITSU:2011-09-21 change panel settings start */
    drv_display_mddi_cmd(cmd_B0_protect);
    drv_display_mddi_cmd(cmd_B7);
    drv_display_mddi_cmd(cmd_B0_unprotect);
/* FUJITSU:2011-09-21 change panel settings end */
}

static void drv_display_set_display_off(void)
{
    drv_display_mddi_cmd(cmd_28);
}

#if 0 /* FUJITSU:2011-09-14 delete  system_rev check --> */
static void drv_display_reset(void)
{
    
    //gpio init
    gpio_tlmm_config(GPIO_CFG(33, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
    
    //Reset
    gpio_set_value(33, 0);
    mddi_wait(15);  //min 10ms  //CHECK:adjust wait time
    gpio_set_value(33, 1);
    mddi_wait(15);  //min 10ms  //CHECK:adjust wait time
    
}
#endif /* FUJITSU:2011-09-14 delete  system_rev check <-- */

static void drv_display_reset_low(void)
{
    //gpio init
    gpio_tlmm_config(GPIO_CFG(33, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
    
    gpio_set_value(33, 0);
    
}

static void drv_display_reset_high(void)
{
    //gpio init
    gpio_tlmm_config(GPIO_CFG(33, 0, GPIO_CFG_OUTPUT, GPIO_CFG_NO_PULL, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
    
    mddi_wait(1);  //min 10us
    gpio_set_value(33, 1);
    mddi_wait(15);
}

static void drv_display_poweron(void)
{
/* FUJITSU:2011-09-21 change panel settings start */
    drv_display_mddi_cmd(cmd_B0_protect);
    drv_display_mddi_cmd(cmd_B7);
    drv_display_mddi_cmd(cmd_B0_unprotect);
/* FUJITSU:2011-09-21 change panel settings end */
    drv_display_exit_sleep_mode();
/* FUJITSU:2011-09-21 change panel settings start */
    drv_display_mddi_cmd(cmd_B0_protect);
    drv_display_mddi_cmd(cmd_B7);
    drv_display_mddi_cmd(cmd_B0_unprotect);
/* FUJITSU:2011-09-21 change panel settings end */
    drv_display_mddi_cmd(cmd_35);		//	set_tear_on
}

static void drv_display_poweroff(void)
{
	drv_display_set_display_off();
	drv_display_enter_sleep_mode();
}

static void drv_display_enter_sleep_mode(void)
{
    drv_display_mddi_cmd(cmd_10);		//	enter_sleep_mode
    msleep(100);							//	wait 72ms(min)
}

static void drv_display_exit_sleep_mode(void)
{
    drv_display_mddi_cmd(cmd_2A);		//	set_column_address
    drv_display_mddi_cmd(cmd_2B);		//	set_page_address
    drv_display_mddi_cmd(cmd_36);		//	set_address_mode
/* FUJITSU:2011-08-25 change panel settings start */
    drv_display_mddi_cmd(cmd_3A);		//	set_pixel_format
/* FUJITSU:2011-08-25 change panel settings end */
    drv_display_mddi_cmd(cmd_11);		//	exit_sleep_mode
    msleep(150);						//	wait 126ms(min)
}


static void drv_display_control(lcd_info_e lcd_info)
{
    printk(KERN_DEBUG "[LCD]PANEL = R61408\n");
    printk(KERN_DEBUG "[LCD]%s(%d): enter state=%d\n",__func__,lcd_info,lcd_state);
    
    switch (lcd_state) {
        case LCD_STATE_INIT:
#if 0 /* FUJITSU:2011-09-14 delete  system_rev check --> */
            if (system_rev >= 0x0c) {
                printk(KERN_DEBUG "[LCD]%s: power_on\n",__func__);
				drv_display_reset();
                drv_display_poweron();
	            lcd_state = LCD_STATE_WAIT_UPDATE;
            }
			else
			{
/* FUJITSU:2011-08-25 change panel settings start */
			    drv_display_mddi_cmd(cmd_3A);		//	set_pixel_format
/* FUJITSU:2011-08-25 change panel settings end */
    	        lcd_state = LCD_STATE_ON;
			}
#else
/* FUJITSU:2011-08-25 change panel settings start */
            drv_display_mddi_cmd(cmd_3A);		//	set_pixel_format
/* FUJITSU:2011-08-25 change panel settings end */
/* FUJITSU:2012-03-13 DISP Issue of a displayed color start */
            msleep(20);							//	wait 20ms(min)
            drv_display_mddi_cmd(cmd_3A);		//	set_pixel_format
            msleep(20);							//	wait 20ms(min)
            drv_display_mddi_cmd(cmd_3A);		//	set_pixel_format
/* FUJITSU:2012-03-13 DISP Issue of a displayed color end */
            lcd_state = LCD_STATE_ON;
#endif /* FUJITSU:2011-09-14 delete  system_rev check <-- */
            break;
        case LCD_STATE_OFF:
            if (lcd_info == LCD_ON) {
				mddi_host_client_cnt_reset();
				drv_display_reset_high();
	            drv_display_poweron();
                lcd_state = LCD_STATE_WAIT_UPDATE;
            }else{
                //NOP
            }
            break;
        case LCD_STATE_ON:
            if (lcd_info == LCD_OFF) {
				mddi_host_client_cnt_reset();
				drv_display_poweroff();
				drv_display_reset_low();
                lcd_state = LCD_STATE_OFF;
            }else{
                //NOP
            }
            break;
        case LCD_STATE_WAIT_DISPLAY_ON:
        case LCD_STATE_WAIT_UPDATE:
/* FUJITSU:2012-02-17 DISP change STM start */
            if (lcd_info == LCD_OFF) {
                printk(KERN_DEBUG "[LCD]%s: state=%d to OFF\n",__func__, lcd_state);
                mddi_host_client_cnt_reset();
                drv_display_enter_sleep_mode();
                drv_display_reset_low();
                lcd_state = LCD_STATE_OFF;
            }else{
                //NOP
            }
/* FUJITSU:2012-02-17 DISP change STM end */
        default:
            break;
    }
    printk(KERN_DEBUG "[LCD]%s: leave state=%d\n",__func__, lcd_state);
}

static int mddi_lcd_on(struct platform_device *pdev)
{
    drv_display_control(LCD_ON);
    return 0;
}

static int mddi_lcd_off(struct platform_device *pdev)
{
    drv_display_control(LCD_OFF);
    return 0;
}

static int __devinit mddi_lcd_probe(struct platform_device *pdev)
{
    printk(KERN_DEBUG "[LCD]%s: enter\n", __func__);
    msm_fb_add_device(pdev);
    printk(KERN_DEBUG "[LCD]%s: leave\n", __func__);
    return 0;
}

static int _mddi_R61408_BD6184_i2c_write(unsigned char addr,unsigned char data)
{
    struct i2c_msg msg;
    u_int8_t buf[8];
    int ret = 0;
    int try = 0;/* FUJITSU:2011-08-25 i2c-retry */

    msg.addr  = BD6184_I2C_SLAVE_ADDR;
    msg.buf   = buf;
    msg.len   = 2;
    msg.flags = 0;
    
    buf[0] = addr;
    buf[1] = data;
    
    ret = i2c_transfer(i2c_bkl, &msg, 1);
    if (ret < 0) {
        printk(KERN_ERR "[BKL]%s I2C(addr:%x,data:%x) ERROR ret = %d\n",__func__,addr,data,ret);
/* FUJITSU:2011-08-25 i2c-retry start */
        for (try = 0; try < 50; try++) {
            msleep(5);
            ret = i2c_transfer(i2c_bkl, &msg, 1);
            if (ret >= 0 ) {
                printk(KERN_ERR "[BKL]%s I2C(addr:%x,data:%x) ERROR ret = %d retry(%d):OK\n",__func__,addr,data,ret,try);
                break;
            }
        }
        if(ret < 0){
            printk(KERN_ERR "[BKL]%s I2C(addr:%x,data:%x) ERROR ret = %d retry(%d):NG\n",__func__,addr,data,ret,try);
        }
/* FUJITSU:2011-08-25 i2c-retry end */
    }
    else {
        /* I2C transfer successful. return success(0) */
        ret = 0;
    }
    
    return ret;
}

static void set_backlight(struct msm_fb_data_type *mfd)
{
    int ret = 0;
    int32 level = 0;
    
    level = mfd->bl_level;
    
    if(level) {
        /* BKL ON */
        if(mddi_R61408_bkl_state == BKL_STATE_OFF) {
            //initial configure
            ret |= _mddi_R61408_BD6184_i2c_write(0x01,0x1E);
            
            /* NOTICE : light turn on sharply at 1st time */
            ret |= _mddi_R61408_BD6184_i2c_write(0x09,0x00); //TLH set sharply
            
            /* Intensity */
            ret |= _mddi_R61408_BD6184_i2c_write(0x03,level);
            
            /* MLED Power On */
            ret |= _mddi_R61408_BD6184_i2c_write(0x02,0x01);
            
            if(ret != 0) {
                printk(KERN_ERR "[BKL]:I2C set failed. state change skipped.\n");
            }
            else {
                mddi_R61408_bkl_state = BKL_STATE_1ST_ON;
                printk(KERN_INFO "[BKL]%s(%d) Backlight turn ON.\n",__func__,level);
            }
        }
        else {
            if(mddi_R61408_bkl_state == BKL_STATE_1ST_ON){
                /* 2nd time. slope reset */
/* FUJITSU:2011-08-22 change slope time settings start */
                _mddi_R61408_BD6184_i2c_write(0x09,0x66);
//                _mddi_R61408_BD6184_i2c_write(0x09,0x00);
/* FUJITSU:2011-08-22 change slope time settings end */
                mddi_R61408_bkl_state = BKL_STATE_ON;
            }

            if(!factory_mode()){
                /* Intensity */
                _mddi_R61408_BD6184_i2c_write(0x03,level);
            }
        }
    }
    else {
        /* BKL OFF */
        
        /* MLED PowerOff */
        _mddi_R61408_BD6184_i2c_write(0x02,0x00);
        
        mddi_R61408_bkl_state = BKL_STATE_OFF;
        printk(KERN_INFO "[BKL]%s(%d) Backlight turn OFF.\n",__func__,level);
    }
    
    return;
}

static struct platform_driver this_driver = {
    .probe  = mddi_lcd_probe,
    .shutdown = NULL,
    .driver = {
        .name   = "mddi_R61408",
    },
};

static struct msm_fb_panel_data mddi_R61408_panel_data = {
    .on   = mddi_lcd_on,
    .off  = mddi_lcd_off,
    .set_backlight  = set_backlight
};

static struct platform_device this_device = {
    .name   = "mddi_R61408",
    .id  = 0,
    .dev  = {
        .platform_data = &mddi_R61408_panel_data,
    }
};

/* FUJITSU:2012-02-27 DISP MDPreg customize start */
int mddi_panel_get_padcal(boolean is_hivernating)
{
    int ret = 0x10220020;
    
    if (is_hivernating) {
        ret = 0xf1fe0020;
    }
    else {
        ret = 0x01fe0020;
    }
    
    return ret;
}

int mddi_panel_get_drvcnt(boolean is_hivernating)
{
    int ret = 0x60006;
    
    if (is_hivernating) {
        ret = 0x00800000;
    }
    else {
        ret = 0x00380000;
    }
    
    return ret;
}
/* FUJITSU:2012-02-27 DISP MDPreg customize end */

static int __init mddi_R61408_init(void)
{
    int ret;
    struct msm_panel_info *pinfo;
    
    printk(KERN_DEBUG "[LCD]%s: enter\n", __func__);
    
    lcd_state = LCD_STATE_INIT;
    
    i2c_bkl = i2c_get_adapter(0);
    printk(KERN_DEBUG "[BKL]%s: get i2c_adapter\n",__func__);
    if (!i2c_bkl) {
        printk(KERN_ERR "[BKL]%s i2c_get_adapter() failure.\n",__func__);
    }
    mddi_R61408_bkl_state = BKL_STATE_OFF;
    
    ret = platform_driver_register(&this_driver);
    
    if (!ret) {
        pinfo = &mddi_R61408_panel_data.panel_info;
        pinfo->xres                       = 480;
        pinfo->yres                       = 800;
        pinfo->type                       = MDDI_PANEL;
        pinfo->pdest                      = DISPLAY_1;
        pinfo->mddi.vdopkt                = MDDI_DEFAULT_PRIM_PIX_ATTR;
        pinfo->wait_cycle                 = 0;
        pinfo->bpp                        = 24;
//        pinfo->bpp                        = 18;
        pinfo->fb_num                     = 2;
        pinfo->clk_rate                   = 192000000;
        pinfo->clk_min                    = 192000000;
        pinfo->clk_max                    = 192000000;
        pinfo->lcd.vsync_enable           = TRUE;
        pinfo->lcd.refx100                = 5995;
        pinfo->lcd.v_back_porch           = 12;
        pinfo->lcd.v_front_porch          = 20;
        pinfo->lcd.v_pulse_width          = 2;
        pinfo->lcd.hw_vsync_mode          = TRUE;
        pinfo->lcd.vsync_notifier_period  = 0;
        pinfo->lcd.rev                    = 2;
        pinfo->bl_min                     = 1;
/* FUJITSU:2011-09-30 change backlight MAX start */
//        pinfo->bl_max                     = 127;
        pinfo->bl_max                     = 83;
/* FUJITSU:2011-09-30 change backlight MAX  end */
        
/* FUJITSU:2012-02-27 DISP add LCD actual size start */
        pinfo->actual_height          = 80;
        pinfo->actual_width           = 48;
/* FUJITSU:2012-02-27 DISP add LCD actual size end */
        
        
        ret = platform_device_register(&this_device);
        if (ret) {
            platform_driver_unregister(&this_driver);
        }
    }
    printk(KERN_INFO "[LCD]%s: leave(%d)\n", __func__,ret);
    return ret;
}

module_init(mddi_R61408_init);
