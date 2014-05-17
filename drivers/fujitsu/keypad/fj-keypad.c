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
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/gpio_event.h>
#include <linux/interrupt.h>
#include <asm/mach-types.h>
#include <linux/proc_fs.h>

#if (defined(CONFIG_MACH_F12APON) || defined(CONFIG_MACH_F12ACE))
static struct gpio_event_direct_entry keypad_nav_map[] = {
    {115,       KEY_HOME},
    {114,       KEY_MENU},
    {113,       KEY_BACK},
    { 69,       KEY_VOLUMEUP},
    { 67,       KEY_VOLUMEDOWN},
};

static struct gpio_event_direct_entry keypad_nav_map_debug[] = {
    {115,       KEY_HOME},
    {114,       KEY_MENU},
    {113,       KEY_BACK},
    { 69,       KEY_VOLUMEUP},
    { 67,       KEY_VOLUMEDOWN},
};

#elif (defined(CONFIG_MACH_F12NAD))
static struct gpio_event_direct_entry keypad_nav_map[] = {
    {180,       KEY_HOME},
    {174,       KEY_VOLUMEUP},
    {173,       KEY_VOLUMEDOWN},
    {172,       KEY_CAMERA},
    {145,       KEY_XEMG_ALM},
};

static struct gpio_event_direct_entry keypad_nav_map_debug[] = {
    {180,       KEY_HOME},
    {174,       KEY_VOLUMEUP},
    {173,       KEY_VOLUMEDOWN},
//    {172,       KEY_MENU},
    {172,       KEY_CAMERA},
//    {145,       KEY_BACK},
    {145,       KEY_XEMG_ALM},

};

#endif



static int keypad_gpio_event_nav_func(
    struct gpio_event_input_devs  *input_dev,
    struct gpio_event_info *info,
    void **data, int func);

static struct gpio_event_input_info keypad_nav_info = {
        .info.func = keypad_gpio_event_nav_func,
        .info.no_suspend = true,
        .flags = GPIOKPF_LEVEL_TRIGGERED_IRQ | GPIOKPF_DRIVE_INACTIVE | GPIOEDF_PRINT_KEYS | GPIOEDF_PRINT_KEY_DEBOUNCE,
        .type = EV_KEY,
        .keymap = keypad_nav_map,
        .debounce_time.tv64 = 20 * NSEC_PER_MSEC,
        .keymap_size = ARRAY_SIZE(keypad_nav_map)
};

static struct gpio_event_input_info keypad_nav_info_debug = {
        .info.func = keypad_gpio_event_nav_func,
        .info.no_suspend = true,
        .flags = GPIOKPF_LEVEL_TRIGGERED_IRQ | GPIOKPF_DRIVE_INACTIVE | GPIOEDF_PRINT_KEYS | GPIOEDF_PRINT_KEY_DEBOUNCE,
        .type = EV_KEY,
        .keymap = keypad_nav_map_debug,
        .debounce_time.tv64 = 20 * NSEC_PER_MSEC,
        .keymap_size = ARRAY_SIZE(keypad_nav_map_debug)
};


static struct gpio_event_info *keypad_info[] = {
    &keypad_nav_info.info
};


static struct gpio_event_info *keypad_info_debug[] = {
    &keypad_nav_info_debug.info
};

static struct gpio_event_platform_data keypad_data = {
    .name       = "keypad",
    .info       = keypad_info,
    .info_count = ARRAY_SIZE(keypad_info)
};


static struct gpio_event_platform_data keypad_data_debug = {
    .name       = "keypad",
    .info       = keypad_info_debug,
    .info_count = ARRAY_SIZE(keypad_info_debug)
};


struct platform_device keypad_device_fj_d = {
    .name   = GPIO_EVENT_DEV_NAME,
    .id = -1,
    .dev    = {
        .platform_data  = &keypad_data,
    },
};


struct platform_device keypad_device_fj_d_debug = {
    .name   = GPIO_EVENT_DEV_NAME,
    .id = -1,
    .dev    = {
        .platform_data  = &keypad_data_debug,
    },
};

#ifdef CONFIG_MACH_F12NAD
static int proc_xemg_read(char *page, char **start, off_t off, int count, int *eof, void *data)
{
    int xemg_value;

    xemg_value = gpio_get_value(145);

    if (xemg_value == 1){
        return sprintf(page, "0\n");
    }else{
        return sprintf(page, "1\n");
    }
}
#endif

struct gpio_event_input_devs *keypad_dev;

static int keypad_gpio_event_nav_func(
    struct gpio_event_input_devs  *input_dev,
    struct gpio_event_info *info,
    void **data, int func)
{
    int err;

    err = gpio_event_input_func(input_dev, info, data, func);

    if (func == GPIO_EVENT_FUNC_INIT && !err) {
        keypad_dev = input_dev;
#ifdef CONFIG_MACH_F12NAD
        create_proc_read_entry("xemg_key", 0444, NULL, proc_xemg_read, NULL);
#endif
    } else if (func == GPIO_EVENT_FUNC_UNINIT) {
        keypad_dev = NULL;
    }

    return err;
}

struct gpio_event_input_devs *msm_keypad_get_input_dev(void)
{
    return keypad_dev;
}
MODULE_LICENSE("GPL");
