/**
 * TSIF driver client
 *
 * Character device that, being read
 * returns stream of TSIF packets.
 *
 * Copyright (c) 2009-2010, Code Aurora Forum. All rights
 * reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 */

#include <linux/module.h>       /* Needed by all modules */
#include <linux/kernel.h>       /* Needed for KERN_INFO */
#include <linux/cdev.h>
#include <linux/err.h>          /* IS_ERR etc. */
#include <linux/fs.h>
#include <linux/device.h>
#include <linux/sched.h>        /* TASK_INTERRUPTIBLE */

#include <linux/uaccess.h>        /* copy_to_user */

#include <linux/tsif_api.h>

#include <linux/proc_fs.h>

/* TS TBL */
typedef struct{
	unsigned char  S;
	unsigned char  W0;
	unsigned char  W1;
	unsigned char  H;
	unsigned char  tbl[256];
}S_TsConvTbl;

struct tsif_chrdev {
	struct cdev cdev;
	struct device *dev;
	wait_queue_head_t wq_read;
	void *cookie;
	/* mirror for tsif data */
	void *data_buffer;
	unsigned buf_size_packets; /**< buffer size in packets */
	unsigned ri, wi;
	enum tsif_state state;
	unsigned rptr;
	/* TS TBL */
	unsigned int ts_dat;
	S_TsConvTbl ts_tbl;
};

static ssize_t tsif_open(struct inode *inode, struct file *file)
{
	int rc;
	struct tsif_chrdev *the_dev =
	       container_of(inode->i_cdev, struct tsif_chrdev, cdev);
	if (!the_dev->cookie)  /* not bound yet */
		return -ENODEV;
	file->private_data = the_dev;
	rc = tsif_start(the_dev->cookie);
	if (rc)
		return rc;
	tsif_get_info(the_dev->cookie, &the_dev->data_buffer,
		      &the_dev->buf_size_packets);
	the_dev->rptr = 0;
	return nonseekable_open(inode, file);
}

static ssize_t tsif_release(struct inode *inode, struct file *filp)
{
	struct tsif_chrdev *the_dev = filp->private_data;
	tsif_stop(the_dev->cookie);
	return 0;
}

static ssize_t tsif_read(struct file *filp, char __user *buf, size_t count,
			 loff_t *f_pos)
{
	int avail = 0;
	int wi;
	int i,j;
	int off,index;
	unsigned char *user_buf;
	struct tsif_chrdev *the_dev = filp->private_data;
	tsif_get_state(the_dev->cookie, &the_dev->ri, &the_dev->wi,
		       &the_dev->state);

	/* consistency check */
	if (the_dev->ri != (the_dev->rptr / TSIF_PKT_SIZE)) {
		dev_err(the_dev->dev,
			"%s: inconsistent read pointers: ri %d rptr %d\n",
			__func__, the_dev->ri, the_dev->rptr);
		the_dev->rptr = the_dev->ri * TSIF_PKT_SIZE;
	}
	/* ri == wi if no data */
	if (the_dev->ri == the_dev->wi) {
		/* shall I block waiting for data? */
		if (filp->f_flags & O_NONBLOCK) {
			if (the_dev->state == tsif_state_running) {
				return -EAGAIN;
			} else {
				/* not running -> EOF */
				return 0;
			}
		}
		if (wait_event_interruptible(the_dev->wq_read,
		      (the_dev->ri != the_dev->wi) ||
		      (the_dev->state != tsif_state_running))) {
			/* got signal -> tell FS to handle it */
			return -ERESTARTSYS;
		}
		if (the_dev->ri == the_dev->wi) {
			/* still no data -> EOF */
			return 0;
		}
	}
	/* contiguous chunk last up to wi or end of buffer */
	wi = (the_dev->wi > the_dev->ri) ?
		the_dev->wi : the_dev->buf_size_packets;
	avail = min(wi * TSIF_PKT_SIZE - the_dev->rptr, count);

	user_buf = the_dev->data_buffer + the_dev->rptr;
	for (i = 0; i < the_dev->ts_tbl.H; i++){
		off = the_dev->ts_tbl.S + i * the_dev->ts_tbl.W0;
	    for (j = 0; j < the_dev->ts_tbl.W1; j = the_dev->ts_dat + j){
			index =  user_buf[ off + j];
			user_buf[off + j] = the_dev->ts_tbl.tbl[ index ];
	    }
	}
	if (copy_to_user(buf, the_dev->data_buffer + the_dev->rptr, avail))
		return -EFAULT;
	the_dev->rptr = (the_dev->rptr + avail) %
		(TSIF_PKT_SIZE * the_dev->buf_size_packets);
	the_dev->ri = the_dev->rptr / TSIF_PKT_SIZE;
	*f_pos += avail;
	tsif_reclaim_packets(the_dev->cookie, the_dev->ri);
	return avail;
}

static void tsif_notify(void *data)
{
	struct tsif_chrdev *the_dev = data;
	tsif_get_state(the_dev->cookie, &the_dev->ri, &the_dev->wi,
		       &the_dev->state);
	wake_up_interruptible(&the_dev->wq_read);
}
/*---------------------------------------------------------------------------
    tsif_ioctl
---------------------------------------------------------------------------*/
/* IOCTL CMD */
#define TSIF_IOCTL_MAGIC 't'
#define TSIF_IOCTL_CMD_01 _IOC( _IOC_NONE,TSIF_IOCTL_MAGIC, 1, 0 )
/* struct CMD01 */
typedef struct {
	unsigned int ts_dat;
	S_TsConvTbl  ts_tbl;
} tsif_ioctl_cmd_01;
static long tsif_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	tsif_ioctl_cmd_01 param1;
	struct tsif_chrdev *the_dev = filp->private_data;

	if(cmd == TSIF_IOCTL_CMD_01) {
		if (copy_from_user(&param1,(tsif_ioctl_cmd_01 __user *)arg,sizeof(tsif_ioctl_cmd_01))) {
			printk("tsif_ioctl:%s: copy_from_user failed\n", __func__ );
			return (-EFAULT);
		}
		the_dev->ts_dat = param1.ts_dat;
		memcpy(&the_dev->ts_tbl ,&param1.ts_tbl ,sizeof(S_TsConvTbl));
	}
	return 0;
}

static const struct file_operations tsif_fops = {
	.owner   = THIS_MODULE,
	.read    = tsif_read,
	.open    = tsif_open,
	.unlocked_ioctl = tsif_ioctl,
	.release = tsif_release,
};

static struct class *tsif_class;
static dev_t tsif_dev;  /**< 1-st dev_t from allocated range */
static dev_t tsif_dev0; /**< next not yet assigned dev_t */

static int tsif_init_one(struct tsif_chrdev *the_dev, int index)
{
	int rc;
	pr_info("%s[%d]\n", __func__, index);
	cdev_init(&the_dev->cdev, &tsif_fops);
	the_dev->cdev.owner = THIS_MODULE;
	init_waitqueue_head(&the_dev->wq_read);
	rc = cdev_add(&the_dev->cdev, tsif_dev0++, 1);
	the_dev->dev = device_create(tsif_class, NULL, the_dev->cdev.dev,
				     the_dev, "tsif%d", index);
	if (IS_ERR(the_dev->dev)) {
		rc = PTR_ERR(the_dev->dev);
		pr_err("device_create failed: %d\n", rc);
		goto err_create;
	}
	the_dev->cookie = tsif_attach(index, tsif_notify, the_dev);
	if (IS_ERR(the_dev->cookie)) {
		rc = PTR_ERR(the_dev->cookie);
		pr_err("tsif_attach failed: %d\n", rc);
		goto err_attach;
	}
	/* now data buffer is not allocated yet */
	tsif_get_info(the_dev->cookie, &the_dev->data_buffer, NULL);
	dev_info(the_dev->dev,
		 "Device %d.%d attached to TSIF, buffer size %d\n",
		 MAJOR(the_dev->cdev.dev), MINOR(the_dev->cdev.dev),
		 the_dev->buf_size_packets);
	return 0;
err_attach:
	device_destroy(tsif_class, the_dev->cdev.dev);
err_create:
	cdev_del(&the_dev->cdev);
	return rc;
}

static void tsif_exit_one(struct tsif_chrdev *the_dev)
{
	dev_info(the_dev->dev, "%s\n", __func__);
	tsif_detach(the_dev->cookie);
	device_destroy(tsif_class, the_dev->cdev.dev);
	cdev_del(&the_dev->cdev);
}

#define TSIF_NUM_DEVS 1 /**< support this many devices */

struct tsif_chrdev the_devices[TSIF_NUM_DEVS];

static int __init mod_init(void)
{
	int rc;
	rc = alloc_chrdev_region(&tsif_dev, 0, TSIF_NUM_DEVS, "tsif");
	if (rc) {
		pr_err("alloc_chrdev_region failed: %d\n", rc);
		goto err_devrgn;
	}
	tsif_dev0 = tsif_dev;
	tsif_class = class_create(THIS_MODULE, "tsif");
	if (IS_ERR(tsif_class)) {
		rc = PTR_ERR(tsif_class);
		pr_err("Error creating tsif class: %d\n", rc);
		goto err_class;
	}
	rc = tsif_init_one(&the_devices[0], 0);
	if (rc)
		goto err_init1;
	return 0;
err_init1:
	class_destroy(tsif_class);
err_class:
	unregister_chrdev_region(tsif_dev, TSIF_NUM_DEVS);
err_devrgn:
	return rc;
}

static void __exit mod_exit(void)
{
	tsif_exit_one(&the_devices[0]);
	class_destroy(tsif_class);
	unregister_chrdev_region(tsif_dev, TSIF_NUM_DEVS);
}

module_init(mod_init);
module_exit(mod_exit);

MODULE_DESCRIPTION("TSIF character device interface");
MODULE_LICENSE("GPL v2");

