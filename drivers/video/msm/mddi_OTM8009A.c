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
    INTERNAL FEATURES
============================================================================*/
#define FEATURE_LCD_WITH_R61408
#undef FEATURE_LCD_WITHOUT_BOOTLOADER
#undef FEATURE_LOCAL_DEBUG

/*===========================================================================
    EXTERNAL FUNCTIONS PROTOTYPES
============================================================================*/
static int mddi_OTM8009A_lcd_on(struct platform_device *pdev);
static int mddi_OTM8009A_lcd_off(struct platform_device *pdev);
#ifdef    FEATURE_LCD_WITH_R61408
static int mddi_R61408_lcd_on(struct platform_device *pdev);
static int mddi_R61408_lcd_off(struct platform_device *pdev);
static void mddi_R61408_lcd_shutdown(struct platform_device *pdev);	/* FUJITSU:2012-06-12 DISP add shutdown */
#endif /* FEATURE_LCD_WITH_R61408 */
static void mddi_panel_common_power_off(void);	/* FUJITSU:2012-06-12 DISP add shutdown */
void mddi_panel_fullscrn_update_notify(void);
void mddi_panel_updatedone_notify(void);
int mddi_panel_get_padcal(boolean is_hivernating);
int mddi_panel_get_drvcnt(boolean is_hivernating);
static int __devinit mddi_OTM8009A_lcd_probe(struct platform_device *pdev);
static void mddi_OTM8009A_lcd_shutdown(struct platform_device *pdev);	/* FUJITSU:2012-06-08 DISP add shutdown */
static int __init mddi_OTM8009A_init(void);

static void set_backlight(struct msm_fb_data_type *mfd);

/*============================================================================
    INTERNAL TYPE DEFINITIONS
============================================================================*/
#ifdef FEATURE_LOCAL_DEBUG
#define DBG_PRINTF(fmt, args...) printk(KERN_DEBUG "[LCD_DBG]%s: " fmt, __func__ , ## args)
#else
#define DBG_PRINTF(fmt, args...)
#endif

#define BKL_I2C_SLAVE_ADDR 0x76

typedef enum {
    BKL_STATE_OFF,
    BKL_STATE_1ST_ON,
    BKL_STATE_ON
} bkl_state_e;

typedef enum {
    LCD_STATE_INIT,
    LCD_STATE_OFF,
    LCD_STATE_WAIT_UPDATE,
    LCD_STATE_WAIT_DISPLAY_ON,
    LCD_STATE_ON
} lcd_state_e;


typedef enum {
    SEND_CMD,   //send mddi register command
    WAIT,       //wait 
    RESX_CTL,   //lcd-hw reset control
    SEQ_END,
} lcd_seq_type_e;

#define MLCD_RESX  (33)
#define RESET_LOW  (0)
#define RESET_HIGH (1)

typedef struct _lcd_seq_table{
    lcd_seq_type_e type;
    uint32 data1;
    uint32 data2;
} lcd_seq_table;

#ifdef    FEATURE_LCD_WITH_R61408
typedef enum {
    LCD_IC_OTM8009A,
    LCD_IC_R61408
} lcd_ic_type_e;
#define MLCD_KEY (105)
#endif /* FEATURE_LCD_WITH_R61408 */

#define MLCD_IM  (107)	/* FUJITSU:2012-06-08 DISP add shutdown */

/*===========================================================================
    LOCAL FUNCTIONS PROTOTYPES
============================================================================*/
static void mddi_OTM8009A_delay_on(struct work_struct *ignored);
static DECLARE_WORK(display_on_wq, mddi_OTM8009A_delay_on);

#ifdef    FEATURE_LCD_WITH_R61408
static boolean mddi_panel_is_OTM8009A(void);
#endif /* FEATURE_LCD_WITH_R61408 */
static int mddi_OTM8009A_exec_sequence(const lcd_seq_table *seq_table);
static int mddi_OTM8009A_send_cmd(uint32 addr,uint32 data);
static void mddi_OTM8009A_reset(uint32 reset_type);

static int mddi_OTM8009A_driver_register(void);
#ifdef    FEATURE_LCD_WITH_R61408
static int mddi_R61408_driver_register(void);
#endif /* FEATURE_LCD_WITH_R61408 */

static int bkl_i2c_write(unsigned char addr,unsigned char data);

/*============================================================================
    INTERNAL VARIABLES DEFINITIONS
============================================================================*/
static struct i2c_adapter *i2c_bkl;

static bkl_state_e bkl_state = BKL_STATE_OFF;
static lcd_state_e lcd_state = LCD_STATE_INIT;

#ifdef    FEATURE_LCD_WITH_R61408
static lcd_ic_type_e lcd_ic_type = LCD_IC_OTM8009A;
#endif /* FEATURE_LCD_WITH_R61408 */


static struct platform_driver mddi_OTM8009A_driver = {
    .probe  = mddi_OTM8009A_lcd_probe,
    .shutdown = mddi_OTM8009A_lcd_shutdown,	/* FUJITSU:2012-06-08 DISP add shutdown */
    .driver = {
        .name   = "mddi_OTM8009A",
    },
};

static struct msm_fb_panel_data mddi_OTM8009A_panel_data = {
    .on   = mddi_OTM8009A_lcd_on,
    .off  = mddi_OTM8009A_lcd_off,
    .set_backlight  = set_backlight
};

static struct platform_device mddi_OTM8009A_device = {
    .name   = "mddi_OTM8009A",
    .id  = 0,
    .dev  = {
        .platform_data = &mddi_OTM8009A_panel_data,
    }
};

#ifdef    FEATURE_LCD_WITH_R61408
static struct platform_driver mddi_R61408_driver = {
    .probe  = mddi_OTM8009A_lcd_probe,
    .shutdown = mddi_R61408_lcd_shutdown,		/* FUJITSU:2012-06-12 DISP add shutdown */
    .driver = {
        .name   = "mddi_R61408",
    },
};

static struct msm_fb_panel_data mddi_R61408_panel_data = {
    .on   = mddi_R61408_lcd_on,
    .off  = mddi_R61408_lcd_off,
    .set_backlight  = set_backlight
};

static struct platform_device mddi_R61408_device = {
    .name   = "mddi_R61408",
    .id  = 0,
    .dev  = {
        .platform_data = &mddi_R61408_panel_data,
    }
};
#endif /* FEATURE_LCD_WITH_R61408 */

/************************************************************/
/*   LCD CONTROL TABLES                                     */
/************************************************************/
static const lcd_seq_table mddi_OTM8009A_resume_seq[] = {
    {RESX_CTL   ,   RESET_LOW   ,0},
    {WAIT       ,   10          ,0},
    {RESX_CTL   ,   RESET_HIGH  ,0},
    {WAIT       ,   10          ,0},
    {RESX_CTL   ,   RESET_LOW   ,0},
    {WAIT       ,   1           ,0},
    {RESX_CTL   ,   RESET_HIGH  ,0},
    {WAIT       ,   20          ,0},

    {SEND_CMD   ,   0x1100,0x0000 },
    
    {WAIT       ,   150         ,0},
    
    {SEND_CMD   ,   0x3600,0x0000 },
    {SEND_CMD   ,   0x3A00,0x0077 },
    {SEND_CMD   ,   0xFF00,0x0080 },
    {SEND_CMD   ,   0xFF01,0x0009 },
    {SEND_CMD   ,   0xFF02,0x0000 },
    {SEND_CMD   ,   0xFF80,0x0080 },
    {SEND_CMD   ,   0xFF81,0x0009 },
    {SEND_CMD   ,   0xC580,0x00D8 },
    {SEND_CMD   ,   0xC581,0x0000 },
    {SEND_CMD   ,   0xC582,0x0083 },
    {SEND_CMD   ,   0xC583,0x0000 },
    {SEND_CMD   ,   0xC590,0x0096 },
    {SEND_CMD   ,   0xC591,0x005C },
    {SEND_CMD   ,   0xC592,0x0005 },
    {SEND_CMD   ,   0xC593,0x00BC },
    {SEND_CMD   ,   0xC594,0x0077 },
    {SEND_CMD   ,   0xC595,0x0037 },
    {SEND_CMD   ,   0xC596,0x0034 },
    {SEND_CMD   ,   0xC5A0,0x0096 },
    {SEND_CMD   ,   0xC5A1,0x005C },
    {SEND_CMD   ,   0xC5A2,0x0005 },
    {SEND_CMD   ,   0xC5A3,0x00BC },
    {SEND_CMD   ,   0xC5A4,0x0033 },
    {SEND_CMD   ,   0xC5A5,0x0033 },
    {SEND_CMD   ,   0xC5A6,0x0034 },
    {SEND_CMD   ,   0xD800,0x007F },
    {SEND_CMD   ,   0xD801,0x007F },

    {WAIT       ,   20          ,0},
    
    {SEQ_END    ,   0           ,0},
};

static const lcd_seq_table mddi_OTM8009A_suspend_seq[] = {
    {SEND_CMD   ,   0x2800,0x0000 },
    
    {SEND_CMD   ,   0x1000,0x0000 },
    
    {WAIT       ,   150         ,0},
    
    {RESX_CTL   ,   RESET_LOW   ,0},
    {WAIT       ,   10          ,0},
    
    {SEQ_END    ,   0           ,0}
};

static const lcd_seq_table mddi_OTM8009A_interrupted_suspend_seq[] = {
    {SEND_CMD   ,   0x1000,0x0000 },
    
    {WAIT       ,   150         ,0},
    
    {RESX_CTL   ,   RESET_LOW   ,0},
    {WAIT       ,   10          ,0},
    
    {SEQ_END    ,   0           ,0},
};

static const lcd_seq_table mddi_OTM8009A_displayon_seq[] = {
    {SEND_CMD   ,   0x2900,0x0000 },
    
    {SEND_CMD   ,   0x3500,0x0000 },
    {SEND_CMD   ,   0x4400,0x0000 },
    {SEND_CMD   ,   0x4401,0x0000 },
    
    {SEQ_END    ,   0           ,0},
};

#ifdef    FEATURE_LCD_WITH_R61408
static const lcd_seq_table mddi_R61408_resume_seq[] = {
    {RESX_CTL   ,   RESET_LOW    ,0},
    {WAIT       ,   1            ,0},
    {RESX_CTL   ,   RESET_HIGH   ,0},
    {WAIT       ,   15           ,0},

    {SEND_CMD   ,  0xB0,0x00000003 },
    {SEND_CMD   ,  0xB7,0x25117280 },
    {SEND_CMD   ,  0xB0,0x00000004 },

    {SEND_CMD   ,  0x2A,0xDF010000 },
    {SEND_CMD   ,  0x2B,0x1F030000 },
    {SEND_CMD   ,  0x36,0x00000000 },
    {SEND_CMD   ,  0x3A,0x00000077 },

    {SEND_CMD   ,  0x11,0x00000000 },

    {WAIT       ,  126           ,0},

    {SEND_CMD   ,  0xB0,0x00000003 },
    {SEND_CMD   ,  0xB7,0x25117280 },
    {SEND_CMD   ,  0xB0,0x00000004 },

    {SEND_CMD   ,  0x35,0x00000000 },

    {SEQ_END    ,   0            ,0}

};

static const lcd_seq_table mddi_R61408_boot_seq[] = {
    {SEND_CMD   ,  0x3A,0x00000077 },
/* FUJITSU:2012-05-21 DISP Issue of a displayed color start */
    {WAIT       ,  20            ,0},
    {SEND_CMD   ,  0x3A,0x00000077 },
    {WAIT       ,  20            ,0},
    {SEND_CMD   ,  0x3A,0x00000077 },
/* FUJITSU:2012-05-21 DISP Issue of a displayed color end */

    {SEQ_END    ,   0            ,0}
};

static const lcd_seq_table mddi_R61408_suspend_seq[] = {
    {SEND_CMD   ,  0x28,0x00000000 },
    
    {SEND_CMD   ,  0x10,0x00000000 },
    
    {WAIT       ,   72           ,0},
    {RESX_CTL   ,   RESET_LOW    ,0},
    {SEQ_END    ,   0            ,0}

};


static const lcd_seq_table mddi_R61408_interrupted_suspend_seq[] = {
    {SEND_CMD   ,  0x10,0x00000000 },
    
    {WAIT       ,   72           ,0},
    {RESX_CTL   ,   RESET_LOW    ,0},
    {SEQ_END    ,   0            ,0},
};

static const lcd_seq_table mddi_R61408_displayon_seq[] = {
    {SEND_CMD   , 0x29,0x00000000  },
    {SEQ_END    ,   0            ,0}
};

#endif /* FEATURE_LCD_WITH_R61408 */


/*===========================================================================
    EXTERNAL FUNCTIONS
============================================================================*/
static int mddi_OTM8009A_lcd_on(struct platform_device *pdev)
{
    int ret = 0;

    printk(KERN_DEBUG "[LCD]%s: enter state=%d\n",__func__,lcd_state);
    
    switch(lcd_state)
    {
        case LCD_STATE_INIT:
#ifdef    FEATURE_LCD_WITHOUT_BOOTLOADER
            mddi_host_client_cnt_reset();
            ret = mddi_OTM8009A_exec_sequence(mddi_OTM8009A_resume_seq);
            if(ret==0) {
                lcd_state = LCD_STATE_WAIT_UPDATE;
            }
#else
            lcd_state = LCD_STATE_ON; //already on
#endif /* FEATURE_LCD_WITHOUT_BOOTLOADER */
            break;
        case LCD_STATE_OFF:
            mddi_host_client_cnt_reset();
            ret = mddi_OTM8009A_exec_sequence(mddi_OTM8009A_resume_seq);
            if(ret==0) {
                lcd_state = LCD_STATE_WAIT_UPDATE;
            }
            break;
        case LCD_STATE_WAIT_UPDATE:
        case LCD_STATE_WAIT_DISPLAY_ON:
        case LCD_STATE_ON:
            printk(KERN_DEBUG "[LCD]%s: keep current state(%d)\n",__func__,lcd_state);
            break;
        default:
            break;
    }
    
    printk(KERN_DEBUG "[LCD]%s: leave state=%d ret=%d\n",__func__,lcd_state,ret);
    
    return ret;
}

static int mddi_OTM8009A_lcd_off(struct platform_device *pdev)
{
    int ret = 0;

    printk(KERN_DEBUG "[LCD]%s: enter state=%d\n",__func__,lcd_state);
    
    switch(lcd_state)
    {
        case LCD_STATE_INIT:
        case LCD_STATE_OFF:
            printk(KERN_DEBUG "[LCD]%s: keep current state(%d)\n",__func__,lcd_state);
            break;
        case LCD_STATE_WAIT_UPDATE:
        case LCD_STATE_WAIT_DISPLAY_ON:
            ret = mddi_OTM8009A_exec_sequence(mddi_OTM8009A_interrupted_suspend_seq);
            if(ret==0) {
                lcd_state = LCD_STATE_OFF;
            }
            break;
        case LCD_STATE_ON:
            ret = mddi_OTM8009A_exec_sequence(mddi_OTM8009A_suspend_seq);
            if(ret==0) {
                lcd_state = LCD_STATE_OFF;
            }
            break;
        default:
            break;
    }
    
    printk(KERN_DEBUG "[LCD]%s: leave state=%d ret=%d\n",__func__,lcd_state,ret);
    
    return ret;
}

#ifdef    FEATURE_LCD_WITH_R61408
static int mddi_R61408_lcd_on(struct platform_device *pdev)
{
    int ret = 0;

    printk(KERN_DEBUG "[LCD]%s: enter state=%d\n",__func__,lcd_state);
    
    switch(lcd_state)
    {
        case LCD_STATE_INIT:
#ifdef    FEATURE_LCD_WITHOUT_BOOTLOADER
            mddi_host_client_cnt_reset();
            ret = mddi_OTM8009A_exec_sequence(mddi_R61408_resume_seq);
            if(ret==0) {
                lcd_state = LCD_STATE_WAIT_UPDATE;
            }
#else
            ret = mddi_OTM8009A_exec_sequence(mddi_R61408_boot_seq);
            if(ret==0) {
                lcd_state = LCD_STATE_ON; //already on
            }
#endif /* FEATURE_LCD_WITHOUT_BOOTLOADER */
            break;
        case LCD_STATE_OFF:
            mddi_host_client_cnt_reset();
            ret = mddi_OTM8009A_exec_sequence(mddi_R61408_resume_seq);
            if(ret==0) {
                lcd_state = LCD_STATE_WAIT_UPDATE;
            }
            break;
        case LCD_STATE_WAIT_UPDATE:
        case LCD_STATE_WAIT_DISPLAY_ON:
        case LCD_STATE_ON:
            printk(KERN_DEBUG "[LCD]%s: keep current state(%d)\n",__func__,lcd_state);
            break;
        default:
            break;
    }
    
    printk(KERN_DEBUG "[LCD]%s: leave state=%d ret=%d\n",__func__,lcd_state,ret);
    
    return ret;
}

static int mddi_R61408_lcd_off(struct platform_device *pdev)
{
    int ret = 0;

    printk(KERN_DEBUG "[LCD]%s: enter state=%d\n",__func__,lcd_state);
    
    switch(lcd_state)
    {
        case LCD_STATE_INIT:
        case LCD_STATE_OFF:
            printk(KERN_DEBUG "[LCD]%s: keep current state(%d)\n",__func__,lcd_state);
            break;
        case LCD_STATE_WAIT_UPDATE:
        case LCD_STATE_WAIT_DISPLAY_ON:
            ret = mddi_OTM8009A_exec_sequence(mddi_R61408_interrupted_suspend_seq);
            if(ret==0) {
                lcd_state = LCD_STATE_OFF;
            }
            break;
        case LCD_STATE_ON:
            ret = mddi_OTM8009A_exec_sequence(mddi_R61408_suspend_seq);
            if(ret==0) {
                lcd_state = LCD_STATE_OFF;
            }
            break;
        default:
            break;
    }
    
    printk(KERN_DEBUG "[LCD]%s: leave state=%d ret=%d\n",__func__,lcd_state,ret);
    
    return ret;
}
#endif /* FEATURE_LCD_WITH_R61408 */

/* FUJITSU:2012-06-12 DISP add shutdown start */
static void mddi_panel_common_power_off(void)
{
    printk(KERN_DEBUG "[LCD]%s: enter\n", __func__);
    
    //LDO2 PowerOff
    bkl_i2c_write(0x13, 0x0D);
    
    mdelay(40);
    gpio_set_value(MLCD_IM, 0);
    
    //LDO1 LDO2 PowerOff
    bkl_i2c_write(0x13, 0x0C);
    
    printk(KERN_DEBUG "[LCD]%s: leave\n", __func__);
    return;
}
/* FUJITSU:2012-06-12 DISP add shutdown end*/

void mddi_panel_fullscrn_update_notify(void)
{
    DBG_PRINTF("enter curr_state(%d)\n",lcd_state);
    
    if (lcd_state == LCD_STATE_WAIT_UPDATE) {
        printk(KERN_INFO "[LCD]%s FullScreen Update..\n",__func__);
        lcd_state = LCD_STATE_WAIT_DISPLAY_ON;
    }
}

void mddi_panel_updatedone_notify(void)
{
    DBG_PRINTF("enter curr_state(%d)\n",lcd_state);
    
    if (lcd_state == LCD_STATE_WAIT_DISPLAY_ON) {
        printk(KERN_INFO "[LCD]%s FullScreen Updated! set work queue.\n",__func__);
        schedule_work(&display_on_wq);
    }
}

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

/*===========================================================================
    LOCAL FUNCTIONS
============================================================================*/
static void mddi_OTM8009A_delay_on(struct work_struct *ignored)
{
    int ret = 0;
    
    DBG_PRINTF("enter curr_state(%d)\n",lcd_state);
    
    if (lcd_state == LCD_STATE_WAIT_DISPLAY_ON)
    {
#ifdef    FEATURE_LCD_WITH_R61408
        if (lcd_ic_type == LCD_IC_OTM8009A ){
            ret = mddi_OTM8009A_exec_sequence(mddi_OTM8009A_displayon_seq);
        }
        else {
            ret = mddi_OTM8009A_exec_sequence(mddi_R61408_displayon_seq);
        }
#else
        ret = mddi_OTM8009A_exec_sequence(mddi_OTM8009A_displayon_seq);
#endif /* FEATURE_LCD_WITH_R61408 */
        
        if (ret != 0) {
            printk("[LCD]%s Display ON Fail(%d)\n",__func__,ret);
        }
        else {
            printk("[LCD]%s Display ON\n",__func__);
            lcd_state = LCD_STATE_ON;
        }
    }
    else
    {
        printk("[LCD]%s State(%d).Skip Request\n",__func__,lcd_state);
    }
    
    return;
}

#ifdef    FEATURE_LCD_WITH_R61408
static boolean mddi_panel_is_OTM8009A(void)
{
    int gpio_val = 0;
    boolean ret = FALSE;
    
    //pull up
    gpio_tlmm_config(GPIO_CFG(MLCD_KEY, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
    
    //read
    gpio_val = gpio_get_value(MLCD_KEY);
    
    if (gpio_val == 1){
        //High:OTM8009A
        printk("[LCD]%s OTM8009A connected\n",__func__);
        
        lcd_ic_type = LCD_IC_OTM8009A;
        gpio_tlmm_config(GPIO_CFG(MLCD_KEY, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_UP, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
        ret = TRUE;
    }
    else {
        //Low:R61408
        printk("[LCD]%s R61408 connected\n",__func__);
        
        lcd_ic_type = LCD_IC_R61408;
        gpio_tlmm_config(GPIO_CFG(MLCD_KEY, 0, GPIO_CFG_INPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
        ret = FALSE;
    }
    return ret;
}
#endif /* FEATURE_LCD_WITH_R61408 */

static int mddi_OTM8009A_exec_sequence(const lcd_seq_table *seq_table)
{
    int ret = 0;
    int i = 0;
    
    while ( seq_table[i].type != SEQ_END ) {
        if ( seq_table[i].type == SEND_CMD ) { 
            DBG_PRINTF("index[%d] send mddi_command(0x%x,0x%x)\n",i,seq_table[i].data1, seq_table[i].data2);
            ret = mddi_OTM8009A_send_cmd( seq_table[i].data1, seq_table[i].data2 );
        }
        else if (seq_table[i].type == WAIT ) { 
            DBG_PRINTF("index[%d] msleep(%d) start\n",i,seq_table[i].data1);
            msleep(seq_table[i].data1);
            DBG_PRINTF("index[%d] msleep(%d) done\n",i,seq_table[i].data1);
            
        }
        else if (seq_table[i].type == RESX_CTL ) { 
            DBG_PRINTF("index[%d] reset(%d)\n",i,seq_table[i].data1);
            mddi_OTM8009A_reset( seq_table[i].data1 );
        }
        
        if (ret!= 0){
            printk("[LCD]%s table index[%d] failed(%d)\n",__func__,i,ret);
            break;
        }
        i++;
    }
    
    DBG_PRINTF(" done ret(%d)\n",ret);
    
    return ret;
}

static int mddi_OTM8009A_send_cmd(uint32 addr,uint32 data)
{
    int ret = 0;
    
    DBG_PRINTF(" send(0x%x,0x%x) \n",addr,data);
    
    ret = mddi_queue_register_write(addr, data, TRUE, 0);
    
    if (ret!= 0){
        printk("[LCD] mddi command set fail (addr:0x%8x data:0x%8x) ret:%d \n",addr,data,ret);
    }
    DBG_PRINTF(" send(0x%x,0x%x) ret(%d)\n",addr,data,ret);
    return ret;
}

static void mddi_OTM8009A_reset(uint32 reset_type)
{
    //gpio init
    gpio_tlmm_config(GPIO_CFG(MLCD_RESX, 0, GPIO_CFG_OUTPUT, GPIO_CFG_PULL_DOWN, GPIO_CFG_2MA), GPIO_CFG_ENABLE);
    
    if (reset_type == RESET_LOW) {
        gpio_set_value(MLCD_RESX, 0);
    }
    else {
        gpio_set_value(MLCD_RESX, 1);
    }
    
    return;
}

static int bkl_i2c_write(unsigned char addr,unsigned char data)
{
    struct i2c_msg msg;
    u_int8_t buf[8];
    int ret = 0;
    int try = 0;
    
    msg.addr  = BKL_I2C_SLAVE_ADDR;
    msg.buf   = buf;
    msg.len   = 2;
    msg.flags = 0;
    
    buf[0] = addr;
    buf[1] = data;
    
    DBG_PRINTF("addr[0x%x] data[0x%x]\n",addr,data);
    
    ret = i2c_transfer(i2c_bkl, &msg, 1);
    if (ret < 0) {
        printk(KERN_ERR "[BKL]%s I2C(addr:%x,data:%x) ERROR ret = %d\n",__func__,addr,data,ret);
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
    }
    else {
        /* I2C transfer successful. return success(0) */
        ret = 0;
    }
    DBG_PRINTF(" send(0x%x,0x%x) ret(%d)\n",addr,data,ret);
    return ret;
}

static void set_backlight(struct msm_fb_data_type *mfd)
{
    int ret = 0;
    int32 level = 0;
    
    level = mfd->bl_level;
    
    if(level) {
        /* BKL ON */
        if(bkl_state == BKL_STATE_OFF) {
            //initial configure
            ret |= bkl_i2c_write(0x01,0x3E);
            
            /* NOTICE : light turn on sharply at 1st time */
            ret |= bkl_i2c_write(0x09,0x00); //TLH set sharply
            
            /* Intensity */
            ret |= bkl_i2c_write(0x03,level);
            
            /* MLED Power On */
            ret |= bkl_i2c_write(0x02,0x01);
            
            if(ret != 0) {
                printk(KERN_ERR "[BKL]:I2C set failed. state change skipped.\n");
            }
            else {
                bkl_state = BKL_STATE_1ST_ON;
                printk(KERN_INFO "[BKL]%s(%d) Backlight turn ON.\n",__func__,level);
            }
        }
        else {
            if(bkl_state == BKL_STATE_1ST_ON){
                /* 2nd time. slope reset */
                bkl_i2c_write(0x09,0x66);
                bkl_state = BKL_STATE_ON;
            }

            /* Intensity */
            bkl_i2c_write(0x03,level);
        }
    }
    else {
        /* BKL OFF */
        
        /* MLED PowerOff */
        bkl_i2c_write(0x02,0x00);
        
        bkl_state = BKL_STATE_OFF;
        printk(KERN_INFO "[BKL]%s(%d) Backlight turn OFF.\n",__func__,level);
    }
    
    return;
}

static int __devinit mddi_OTM8009A_lcd_probe(struct platform_device *pdev)
{
    printk(KERN_DEBUG "[LCD]%s: enter\n", __func__);
    msm_fb_add_device(pdev);
    printk(KERN_DEBUG "[LCD]%s: leave\n", __func__);
    return 0;
}

/* FUJITSU:2012-06-08 DISP add shutdown start*/
static void mddi_OTM8009A_lcd_shutdown(struct platform_device *pdev)
{
    struct msm_fb_data_type mfd;
    
    printk(KERN_DEBUG "[LCD]%s: enter\n", __func__);
    
    mfd.bl_level = 0;
    set_backlight(&mfd);

/* FUJITSU:2012-06-26 DISP del lcd off sequence on shutdown start*/
//    mddi_OTM8009A_lcd_off(pdev);
/* FUJITSU:2012-06-26 DISP del lcd off sequence on shutdown end*/

    mddi_panel_common_power_off();
    
    printk(KERN_DEBUG "[LCD]%s: leave\n", __func__);
    return;
}
/* FUJITSU:2012-06-08 DISP add shutdown end*/

/* FUJITSU:2012-06-12 DISP add shutdown start*/
#ifdef    FEATURE_LCD_WITH_R61408
static void mddi_R61408_lcd_shutdown(struct platform_device *pdev)
{
    struct msm_fb_data_type mfd;
    
    printk(KERN_DEBUG "[LCD]%s: enter\n", __func__);
    
    mfd.bl_level = 0;
    set_backlight(&mfd);
    
/* FUJITSU:2012-06-26 DISP del lcd off sequence on shutdown start*/
//   mddi_R61408_lcd_off(pdev);
/* FUJITSU:2012-06-26 DISP del lcd off sequence on shutdown end*/

    mddi_panel_common_power_off();
    
    printk(KERN_DEBUG "[LCD]%s: leave\n", __func__);
    return;
}
#endif /* FEATURE_LCD_WITH_R61408 */
/* FUJITSU:2012-06-12 DISP add shutdown end*/

static int mddi_OTM8009A_driver_register(void)
{
    int ret = 0;
    struct msm_panel_info *pinfo;
    
    DBG_PRINTF("enter\n");
    
    ret = platform_driver_register(&mddi_OTM8009A_driver);
    
    if (!ret) {
        pinfo = &mddi_OTM8009A_panel_data.panel_info;
        pinfo->xres                       = 480;
        pinfo->yres                       = 800;
        pinfo->type                       = MDDI_PANEL;
        pinfo->pdest                      = DISPLAY_1;
        pinfo->mddi.vdopkt                = MDDI_DEFAULT_PRIM_PIX_ATTR;
        pinfo->wait_cycle                 = 0;
        pinfo->bpp                        = 24;
        pinfo->fb_num                     = 2;
        pinfo->clk_rate                   = 192000000;
        pinfo->clk_min                    = 192000000;
        pinfo->clk_max                    = 192000000;
        pinfo->lcd.vsync_enable           = TRUE;
        pinfo->lcd.refx100                = 5937;
        pinfo->lcd.v_back_porch           = 16;
        pinfo->lcd.v_front_porch          = 16;
        pinfo->lcd.v_pulse_width          = 0;
        pinfo->lcd.hw_vsync_mode          = TRUE;
        pinfo->lcd.vsync_notifier_period  = 0;
        pinfo->lcd.rev                    = 2;
        pinfo->bl_min                     = 1;
        pinfo->bl_max                     = 127;
        pinfo->actual_height              = 86;
        pinfo->actual_width               = 51;
        ret = platform_device_register(&mddi_OTM8009A_device);
        if (ret) {
            platform_driver_unregister(&mddi_OTM8009A_driver);
        }
    }
    
    return ret;
}

#ifdef    FEATURE_LCD_WITH_R61408
static int mddi_R61408_driver_register(void)
{
    int ret = 0;
    struct msm_panel_info *pinfo;
    
    DBG_PRINTF("enter\n");
    
    ret = platform_driver_register(&mddi_R61408_driver);

    if (!ret) {
        pinfo = &mddi_R61408_panel_data.panel_info;
        pinfo->xres                       = 480;
        pinfo->yres                       = 800;
        pinfo->type                       = MDDI_PANEL;
        pinfo->pdest                      = DISPLAY_1;
        pinfo->mddi.vdopkt                = MDDI_DEFAULT_PRIM_PIX_ATTR;
        pinfo->wait_cycle                 = 0;
        pinfo->bpp                        = 24;
        pinfo->fb_num                     = 2;
        pinfo->clk_rate                   = 192000000;
        pinfo->clk_min                    = 192000000;
        pinfo->clk_max                    = 192000000;
        pinfo->lcd.vsync_enable           = TRUE;
        pinfo->lcd.refx100                = 5996;
        pinfo->lcd.v_back_porch           = 12;
        pinfo->lcd.v_front_porch          = 20;
        pinfo->lcd.v_pulse_width          = 2;
        pinfo->lcd.hw_vsync_mode          = TRUE;
        pinfo->lcd.vsync_notifier_period  = 0;
        pinfo->lcd.rev                    = 2;
        pinfo->bl_min                     = 1;
        pinfo->bl_max                     = 127;
        pinfo->actual_height              = 86;
        pinfo->actual_width               = 51;
        ret = platform_device_register(&mddi_R61408_device);
        if (ret) {
            platform_driver_unregister(&mddi_R61408_driver);
        }
    }
    
    return ret;
}
#endif /* FEATURE_LCD_WITH_R61408 */

static int __init mddi_OTM8009A_init(void)
{
    int ret;
    
    printk(KERN_DEBUG "[LCD]%s: enter\n", __func__);
    
    lcd_state = LCD_STATE_INIT;
    
    i2c_bkl = i2c_get_adapter(0);
    printk(KERN_DEBUG "[BKL]%s: get i2c_adapter\n",__func__);
    if (!i2c_bkl) {
        printk(KERN_ERR "[BKL]%s i2c_get_adapter() failure.\n",__func__);
    }
    bkl_state = BKL_STATE_OFF;
    
#ifdef    FEATURE_LCD_WITH_R61408
    //check lcd type
    if ( !mddi_panel_is_OTM8009A() ) {
        ret = mddi_R61408_driver_register();
        printk(KERN_INFO "[LCD]%s: leave(%d) WITH_R61408\n", __func__,ret);
        return ret;
    }
#endif /* FEATURE_LCD_WITH_R61408 */
    
    ret = mddi_OTM8009A_driver_register();
    printk(KERN_INFO "[LCD]%s: leave(%d)\n", __func__,ret);
    return ret;
}

module_init(mddi_OTM8009A_init);
