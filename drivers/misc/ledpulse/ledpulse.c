/*
  * COPYRIGHT(C) FUJITSU CONNECTED TECHNOLOGIES LIMITED 2018
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
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/miscdevice.h>
#include <linux/poll.h>
#include <linux/compat.h>
#include <linux/sched.h>
#include <linux/ledpulse.h>

#include "ledpulse_local.h"

/******************************************************************************/
/* Structure                                                                  */
/******************************************************************************/
struct ledpulse_data {
	struct ledpulse_dev_operations *dev_ops;
	struct miscdevice ledpulse_device;
	wait_queue_head_t waitq;
};

/******************************************************************************/
/* file_operations                                                            */
/******************************************************************************/
static int ledpulse_open(struct inode *inode, struct file *filp)
{
	struct ledpulse_data *ledpulse = container_of(filp->private_data,
					struct ledpulse_data, ledpulse_device);

	LEDPULSE_LOG_D("%s\n", __func__);
	filp->private_data = ledpulse;
	return 0;
}

static int ledpulse_release(struct inode *inode, struct file *filp)
{
	LEDPULSE_LOG_D("%s\n", __func__);
	return 0;
}

static long ledpulse_ioctl(
	struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct ledpulse_data *ledpulse = filp->private_data;
	struct ledpulse_dev_operations *dev_ops = ledpulse->dev_ops;
	int ret = 0;

	LEDPULSE_LOG_D("%s cmd:0x%x\n", __func__, cmd);
	switch (cmd) {
	case LEDPULSE_POWER_CTL:
	case LEDPULSE_POWER_CTL_MC:
		if (arg == 1) {
			ret = dev_ops->power_ctl(dev_ops->device, true);
		} else if (arg == 0) {
			ret = dev_ops->power_ctl(dev_ops->device, false);
		} else {
			LEDPULSE_LOG_E("%s:0x%x %ld\n", __func__, cmd, arg);
			ret = -EINVAL;
		}
		break;
	case LEDPULSE_MEASUREMENT_CTL:
	case LEDPULSE_MEASUREMENT_CTL_MC:
		if (arg == 1) {
			ret = dev_ops->measurement_ctl(dev_ops->device, true);
		} else if (arg == 0) {
			ret = dev_ops->measurement_ctl(dev_ops->device, false);
		} else {
			LEDPULSE_LOG_E("%s:0x%x %ld\n", __func__, cmd, arg);
			ret = -EINVAL;
		}
		break;
	case LEDPULSE_GET_DEVICEINFO:
		if ((void __user *)arg == NULL)
			ret = -EINVAL;
		else
			ret = dev_ops->get_device_info(
				dev_ops->device, (void __user *)arg);
		break;
	case LEDPULSE_SET_PA_MC:
		if ((void __user *)arg == NULL)
			ret = -EINVAL;
		else
			ret = dev_ops->set_pa_info(
				dev_ops->device, (void __user *)arg);
		break;
	case LEDPULSE_GET_GRAY_IR_DATA:
		if ((void __user *)arg == NULL)
			ret = -EINVAL;
		else
			ret = dev_ops->get_gray_ir_data(
				dev_ops->device, (void __user *)arg);
		break;
	default:
		LEDPULSE_LOG_E("%s cmd:0x%x\n", __func__, cmd);
		ret = -EINVAL;
		break;
	}
	return ret;
}

#ifdef CONFIG_COMPAT
static long ledpulse_compat_ioctl(
	struct file *filep, unsigned int cmd, unsigned long arg)
{
	arg = (unsigned long)compat_ptr(arg);

	LEDPULSE_LOG_D("%s cmd:0x%x\n", __func__, cmd);
	return ledpulse_ioctl(filep, cmd, arg);
}
#endif /* CONFIG_COMPAT */

static unsigned int ledpulse_poll(struct file *filp, poll_table *wait)
{
	struct ledpulse_data *ledpulse = filp->private_data;
	struct ledpulse_dev_operations *dev_ops = ledpulse->dev_ops;

	LEDPULSE_LOG_D("%s\n", __func__);
	poll_wait(filp, &ledpulse->waitq, wait);

	if (dev_ops->is_data_exist(dev_ops->device)) {
		LEDPULSE_LOG_D("poll data exist\n");
		return POLLIN | POLLRDNORM;
	}

	LEDPULSE_LOG_D("poll no data\n");
	return 0;
}

static ssize_t ledpulse_read(struct file *filp, char __user *buf,
	size_t count, loff_t *offset)
{
	struct ledpulse_data *ledpulse = filp->private_data;
	struct ledpulse_dev_operations *dev_ops = ledpulse->dev_ops;
	int ret;

	LEDPULSE_LOG_D("%s\n", __func__);
	ret = dev_ops->read_data(
		dev_ops->device, (void __user *)buf, count, offset);
	if (!ret)
		ret = count;

	return ret;
}

static const struct file_operations ledpulse_fops = {
	.owner			= THIS_MODULE,
	.open			= ledpulse_open,
	.release		= ledpulse_release,
	.read			= ledpulse_read,
	.poll			= ledpulse_poll,
	.unlocked_ioctl = ledpulse_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl	= ledpulse_compat_ioctl,
#endif /* CONFIG_COMPAT */
};

/******************************************************************************/
/* ledpulse functions                                                         */
/******************************************************************************/
ledpulse_h ledpulse_register(struct ledpulse_dev_operations *dev_ops)
{
	struct ledpulse_data *ledpulse;
	int ret = 0;

	LEDPULSE_LOG_D("%s\n", __func__);

	ledpulse = kzalloc(sizeof(*ledpulse), GFP_KERNEL);
	if (!ledpulse) {
		LEDPULSE_LOG_E("failed to allocate memory\n");
		return NULL;
	}

	ledpulse->ledpulse_device.fops = &ledpulse_fops;
	ledpulse->ledpulse_device.name = "ledpulse";
	ledpulse->ledpulse_device.minor = MISC_DYNAMIC_MINOR;

	ret = misc_register(&ledpulse->ledpulse_device);
	if (ret) {
		LEDPULSE_LOG_E("failed to register device:%d\n", ret);
		kfree(ledpulse);
		return NULL;
	}

	init_waitqueue_head(&ledpulse->waitq);

	ledpulse->dev_ops = dev_ops;
	return ledpulse;
}

void ledpulse_notify(ledpulse_h handle)
{
	struct ledpulse_data *ledpulse = handle;

	if (ledpulse) {
		LEDPULSE_LOG_D("wakeup\n");
		wake_up(&ledpulse->waitq);
	}
}

int ledpulse_deregister(ledpulse_h handle)
{
	struct ledpulse_data *ledpulse = handle;

	LEDPULSE_LOG_D("%s\n", __func__);

	if (ledpulse != NULL) {
		misc_deregister(&ledpulse->ledpulse_device);

		kfree(ledpulse);
	}
	return 0;
}

MODULE_AUTHOR("FUJITSU CONNECTED TECHNOLOGIES LIMITED");
MODULE_DESCRIPTION("ledpulse driver");
MODULE_LICENSE("GPL v2");
