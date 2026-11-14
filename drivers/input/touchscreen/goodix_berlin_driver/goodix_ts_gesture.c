/*
 * Goodix Gesture Module
 *
 * Copyright (C) 2019 - 2020 Goodix, Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be a reference
 * to you, when you are integrating the GOODiX's CTP IC into your system,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 */
#include <linux/spinlock.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/input.h>
#include <linux/platform_device.h>
#include <linux/version.h>
#include <linux/delay.h>
#include <linux/atomic.h>
#include <linux/input/mt.h>
#include <linux/proc_fs.h>
#include "goodix_ts_core.h"


#define GOODIX_GESTURE_DOUBLE_TAP		0xCC
#define GOODIX_GESTURE_GLOVE_TAP		0x32
#define GOODIX_GESTURE_EDGE_TAP		0x17
#define GOODIX_GESTURE_CHARGER_TAP		0xAF
#define GOODIX_GESTURE_SINGLE_TAP		0x4C
#define GOODIX_GESTURE_FOD_DOWN			0x46
#define GOODIX_GESTURE_FOD_UP			0x55
#define GOODIX_GLOVE_CMD		0x32
#define GOODIX_EDGE_CMD		0x17

bool __attribute__((weak)) is_tp_on(void);

static ssize_t gsx_double_type_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct device *device =
		container_of(((struct kobject *)dev)->parent, struct device, kobj);
	struct goodix_ts_core *cd = dev_get_drvdata(device);
	uint32_t type = cd->gesture_type;

	return sprintf(buf, "%s\n",
			(type & GESTURE_DOUBLE_TAP) ? "enable" : "disable");
}

static ssize_t gsx_double_type_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct device *device =
		container_of(((struct kobject *)dev)->parent, struct device, kobj);	
	struct goodix_ts_core *cd = dev_get_drvdata(device);

	if (buf[0] == '1' || buf[0] == 1) {
		ts_info("enable double tap");
		cd->gesture_type |= GESTURE_DOUBLE_TAP;
	} else if (buf[0] == '0' || buf[0] == 0) {
		ts_info("disable double tap");
		cd->gesture_type &= ~GESTURE_DOUBLE_TAP;
	} else
		ts_err("invalid cmd[%d]", buf[0]);

	return count;
}

static ssize_t gsx_single_type_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct device *device =
		container_of(((struct kobject *)dev)->parent, struct device, kobj);	
	struct goodix_ts_core *cd = dev_get_drvdata(device);
	uint32_t type = cd->gesture_type;

	return sprintf(buf, "%s\n",
			(type & GESTURE_SINGLE_TAP) ? "enable" : "disable");
}

static ssize_t gsx_single_type_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct device *device =
		container_of(((struct kobject *)dev)->parent, struct device, kobj);	
	struct goodix_ts_core *cd = dev_get_drvdata(device);

	if (buf[0] == '1' || buf[0] == 1) {
		ts_info("enable single tap");
		cd->gesture_type |= GESTURE_SINGLE_TAP;
	} else if (buf[0] == '0' || buf[0] == 0) {
		ts_info("disable single tap");
		cd->gesture_type &= ~GESTURE_SINGLE_TAP;
	} else
		ts_err("invalid cmd[%d]", buf[0]);

	return count;
}

static ssize_t gsx_fod_type_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct device *device =
		container_of(((struct kobject *)dev)->parent, struct device, kobj);	
	struct goodix_ts_core *cd = dev_get_drvdata(device);
	uint32_t type = cd->gesture_type;

	return sprintf(buf, "%s\n",
			(type & GESTURE_FOD_PRESS) ? "enable" : "disable");
}

static ssize_t gsx_fod_type_store(struct device *dev,
				      struct device_attribute *attr,
				      const char *buf, size_t count)
{
	struct device *device =
		container_of(((struct kobject *)dev)->parent, struct device, kobj);
	struct goodix_ts_core *cd = dev_get_drvdata(device);

	if (buf[0] == '1' || buf[0] == 1) {
		ts_info("enable fod");
		cd->gesture_type |= GESTURE_FOD_PRESS;
	} else if (buf[0] == '0' || buf[0] == 0) {
		ts_info("disable fod");
		cd->gesture_type &= ~GESTURE_FOD_PRESS;
	} else
		ts_err("invalid cmd[%d]", buf[0]);

	return count;
}


int goodix_ts_report_gesture(struct goodix_ts_core *cd, struct goodix_ts_event *event)
{
	int fodx, fody, overlay_area;

	switch (event->gesture_type) {
	case GOODIX_GESTURE_SINGLE_TAP:
		if (cd->gesture_type & GESTURE_SINGLE_TAP) {
			ts_info("get SINGLE-TAP gesture");
			input_report_key(cd->input_dev, KEY_WAKEUP, 1);
			// input_report_key(cd->input_dev, KEY_GOTO, 1);
			input_sync(cd->input_dev);
			input_report_key(cd->input_dev, KEY_WAKEUP, 0);
			// input_report_key(cd->input_dev, KEY_GOTO, 0);
			input_sync(cd->input_dev);
		} else {
			ts_debug("not enable SINGLE-TAP");
		}
		break;
	case GOODIX_GESTURE_DOUBLE_TAP:
		if (cd->gesture_type & GESTURE_DOUBLE_TAP) {
			ts_info("get DOUBLE-TAP gesture");
			input_report_key(cd->input_dev, KEY_WAKEUP, 1);
			input_sync(cd->input_dev);
			input_report_key(cd->input_dev, KEY_WAKEUP, 0);
			input_sync(cd->input_dev);
			cd->double_tap_mode = true;
		} else {
			ts_info("not enable DOUBLE-TAP");
			cd->double_tap_mode = false;
		}
		break;

	case GOODIX_GESTURE_GLOVE_TAP:
		if (cd->glove_type & GESTURE_GLOVE_TAP) {
			ts_info("get GLOVE-TAP gesture");
			input_report_key(cd->input_dev, KEY_WAKEUP, 1);
			input_sync(cd->input_dev);
			input_report_key(cd->input_dev, KEY_WAKEUP, 0);
			input_sync(cd->input_dev);
		} else {
			ts_debug("not enable GLOVE-TAP");
		}
		break;

	case GOODIX_GESTURE_CHARGER_TAP:
		if (cd->charger_type & GESTURE_CHARGER_TAP) {
			ts_info("get CHARGER-TAP gesture");
			input_report_key(cd->input_dev, KEY_WAKEUP, 1);
			input_sync(cd->input_dev);
			input_report_key(cd->input_dev, KEY_WAKEUP, 0);
			input_sync(cd->input_dev);
		} else {
			ts_debug("not enable CHARGER-TAP");
		}
		break;

	case GOODIX_GESTURE_EDGE_TAP:
		if (cd->edge_type & GESTURE_EDGE_TAP) {
			ts_info("get CHARGER-TAP edge");
			input_report_key(cd->input_dev, KEY_WAKEUP, 1);
			input_sync(cd->input_dev);
			input_report_key(cd->input_dev, KEY_WAKEUP, 0);
			input_sync(cd->input_dev);
		} else {
			ts_debug("not enable EDGE-TAP");
		}
		break;

	case GOODIX_GESTURE_FOD_DOWN:
		if (cd->gesture_type & GESTURE_FOD_PRESS) {
			ts_info("get FOD-DOWN gesture");
			fodx = le16_to_cpup((__le16 *)event->gesture_data);
			fody = le16_to_cpup((__le16 *)(event->gesture_data + 2));
			overlay_area = event->gesture_data[4];
			ts_debug("fodx:%d fody:%d overlay_area:%d", fodx, fody, overlay_area);
			input_report_key(cd->input_dev, BTN_TOUCH, 1);
			input_mt_slot(cd->input_dev, 0);
			input_mt_report_slot_state(cd->input_dev, MT_TOOL_FINGER, 1);
			input_report_abs(cd->input_dev, ABS_MT_POSITION_X, fodx);
			input_report_abs(cd->input_dev, ABS_MT_POSITION_Y, fody);
			input_report_abs(cd->input_dev, ABS_MT_WIDTH_MAJOR, overlay_area);
			input_sync(cd->input_dev);
		} else {
			ts_debug("not enable FOD-DOWN");
		}
		break;
	case GOODIX_GESTURE_FOD_UP:
		if (cd->gesture_type & GESTURE_FOD_PRESS) {
			ts_info("get FOD-UP gesture");
			// fodx = le16_to_cpup((__le16 *)gs_event.gesture_data);
			// fody = le16_to_cpup((__le16 *)(gs_event.gesture_data + 2));
			// overlay_area = gs_event.gesture_data[4];
			input_report_key(cd->input_dev, BTN_TOUCH, 0);
			input_mt_slot(cd->input_dev, 0);
			input_mt_report_slot_state(cd->input_dev,
					MT_TOOL_FINGER, 0);
			input_sync(cd->input_dev);
		} else {
			ts_debug("not enable FOD-UP");
		}
		break;

	default:
		ts_err("not support gesture type[%02X]", event->gesture_type);
		break;
	}

	return 0;
}

static DEVICE_ATTR(double_type, 0664, gsx_double_type_show, gsx_double_type_store);
static DEVICE_ATTR(single_type, 0664, gsx_single_type_show, gsx_single_type_store);
static DEVICE_ATTR(fod_type, 0664, gsx_fod_type_show, gsx_fod_type_store);

static struct attribute *gesture_attrs[] = {
	&dev_attr_double_type.attr,
	&dev_attr_single_type.attr,
	&dev_attr_fod_type.attr,
	NULL,
};

const static struct attribute_group gesture_sysfs_group = {
	.attrs = gesture_attrs,
};

static ssize_t ges_double_read(struct file *file, char __user *buf,
				     size_t count, loff_t *pos)
{
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 17, 0))
		struct goodix_ts_core *cd = pde_data(file_inode(file));
#else
		struct goodix_ts_core *cd = PDE_DATA(file_inode(file));
#endif
	uint32_t type = cd->gesture_type;
	char tmp_buf[10] = {0};
	int cnt;

	if (*pos > 0)
		return 0;

	cnt = sprintf(tmp_buf, "%s\n",
			(type & GESTURE_DOUBLE_TAP) ? "enable" : "disable");
	if (copy_to_user(buf, tmp_buf, cnt)) {
		return -EFAULT;
	}
	*pos += cnt;
	return cnt;
}

static ssize_t ges_double_write(struct file *file, const char __user *buf,
				     size_t count, loff_t *pos)
{
	char tmp_buf[10] = {0};
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 17, 0))
		struct goodix_ts_core *cd = pde_data(file_inode(file));
#else
		struct goodix_ts_core *cd = PDE_DATA(file_inode(file));
#endif

	printk("enable double tap 3");

	if (copy_from_user(tmp_buf, buf, count)) {
		return -EFAULT;
	}

	if (tmp_buf[0] == '1' || tmp_buf[0] == 1) {
		ts_info("enable double tap");
		cd->gesture_type |= GESTURE_DOUBLE_TAP;
	} else if (tmp_buf[0] == '0' || tmp_buf[0] == 0) {
		ts_info("disable double tap");
		cd->gesture_type &= ~GESTURE_DOUBLE_TAP;
	} else
		ts_err("invalid cmd[%d]", tmp_buf[0]);

	return count;
}

static ssize_t ges_glove_read(struct file *file, char __user *buf,
				     size_t count, loff_t *pos)
{
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 17, 0))
		struct goodix_ts_core *cd = pde_data(file_inode(file));
#else
		struct goodix_ts_core *cd = PDE_DATA(file_inode(file));
#endif
	uint32_t type = cd->glove_type;
	char tmp_buf[10] = {0};
	int cnt;
	if (*pos > 0)
		return 0;
	cnt = sprintf(tmp_buf, "%s\n",
			(type & GESTURE_GLOVE_TAP) ? "1" : "0");
	if (copy_to_user(buf, tmp_buf, cnt)) {
		return -EFAULT;
	}
	*pos += cnt;
	return cnt;
}

extern bool glove_switch;
ssize_t ges_glove_write(struct file *file, const char __user *buf,
				     size_t count, loff_t *pos)
{
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 17, 0))
		struct goodix_ts_core *cd = pde_data(file_inode(file));
#else
		struct goodix_ts_core *cd = PDE_DATA(file_inode(file));
		struct goodix_ts_cmd cmd;
#endif
	char tmp_buf[10] = {0};
	if (copy_from_user(tmp_buf, buf, count)) {
		return -EFAULT;
	}
	if (tmp_buf[0] == '1' || tmp_buf[0] == 1) {
		ts_info("enable glove tap");
		cmd.cmd = GOODIX_GLOVE_CMD;
		cmd.len = 5;

		cmd.data[0] = 0x01;
		cmd.data[1] = 0xFF;
		ts_info("glove enabled1");
		cd->glove_type |= GESTURE_GLOVE_TAP;

		glove_switch = true;
		ts_info("glove_switch:%d", glove_switch);

		if (cd->hw_ops->send_cmd(cd, &cmd)) 
			ts_err("failed send glove cmd");
		
	} else if (tmp_buf[0] == '0' || tmp_buf[0] == 0) {
		ts_info("disable glove tap");
		cmd.cmd = GOODIX_GLOVE_CMD;
		cmd.len = 5;

		cmd.data[0] = 0x00;
		cmd.data[1] = 0xFF;
		ts_info("glove enabled2");
		cd->glove_type &= ~GESTURE_GLOVE_TAP;

		glove_switch = false;
		ts_info("glove_switch:%d", glove_switch);

		if (cd->hw_ops->send_cmd(cd, &cmd)) 
			ts_err("failed send glove cmd");
	} else
		ts_err("invalid cmd[%d]", tmp_buf[0]);
	return count;
}

static ssize_t ges_edge_read(struct file *file, char __user *buf,
				     size_t count, loff_t *pos)
{
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 17, 0))
		struct goodix_ts_core *cd = pde_data(file_inode(file));
#else
		struct goodix_ts_core *cd = PDE_DATA(file_inode(file));
#endif
	uint32_t type = cd->edge_type;
	char tmp_buf[10] = {0};
	int cnt;
	if (*pos > 0)
		return 0;
	cnt = sprintf(tmp_buf, "%s\n",
			(type & GESTURE_EDGE_TAP) ? "1" : "0");
	if (copy_to_user(buf, tmp_buf, cnt)) {
		return -EFAULT;
	}
	*pos += cnt;
	return cnt;
}
static ssize_t ges_edge_write(struct file *file, const char __user *buf,
				     size_t count, loff_t *pos)
{
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 17, 0))
		struct goodix_ts_core *cd = pde_data(file_inode(file));
#else
		struct goodix_ts_core *cd = PDE_DATA(file_inode(file));
		struct goodix_ts_cmd cmd;
#endif
	char tmp_buf[10] = {0};
	if (copy_from_user(tmp_buf, buf, count)) {
		return -EFAULT;
	}
	if (tmp_buf[0] == '1' || tmp_buf[0] == 1) {
		ts_info("enable edge tap right");
		cmd.cmd = GOODIX_EDGE_CMD;
		cmd.len = 6;

		cmd.data[0] = 0x40;
		cmd.data[1] = 0x48;
		ts_info("edge enabled1 right");
		cd->edge_type |= GESTURE_EDGE_TAP;
		if (cd->hw_ops->send_cmd(cd, &cmd)) 
			ts_err("failed send edge cmd");
	
	} else if (tmp_buf[0] == '2' || tmp_buf[0] == 2) {
		ts_info("enable edge tap left");
		cmd.cmd = GOODIX_EDGE_CMD;
		cmd.len = 6;

		cmd.data[0] = 0x80;
		cmd.data[1] = 0x48;
		ts_info("edge enabled1 left");
		cd->edge_type |= GESTURE_EDGE_TAP;
		if (cd->hw_ops->send_cmd(cd, &cmd)) 
			ts_err("failed send edge cmd");		
		
	} else if (tmp_buf[0] == '0' || tmp_buf[0] == 0) {
		ts_info("disable edge tap");
		cmd.cmd = GOODIX_EDGE_CMD;
		cmd.len = 6;

		cmd.data[0] = 0x00;
		cmd.data[1] = 0x00;
		ts_info("edge enabled2");
		cd->edge_type &= ~GESTURE_EDGE_TAP;
		if (cd->hw_ops->send_cmd(cd, &cmd)) 
			ts_err("failed send edge cmd");
	} else
		ts_err("invalid cmd[%d]", tmp_buf[0]);
	return count;
}

static ssize_t ges_charger_read(struct file *file, char __user *buf,
				     size_t count, loff_t *pos)
{
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 17, 0))
		struct goodix_ts_core *cd = pde_data(file_inode(file));
#else
		struct goodix_ts_core *cd = PDE_DATA(file_inode(file));
#endif
	uint32_t type = cd->charger_type;
	char tmp_buf[10] = {0};
	int cnt;
	if (*pos > 0)
		return 0;
	cnt = sprintf(tmp_buf, "%s\n",
			(type & GESTURE_CHARGER_TAP) ? "enable" : "disable");
	if (copy_to_user(buf, tmp_buf, cnt)) {
		return -EFAULT;
	}
	*pos += cnt;
	return cnt;
}
static ssize_t ges_charger_write(struct file *file, const char __user *buf,
				     size_t count, loff_t *pos)
{
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 17, 0))
		struct goodix_ts_core *cd = pde_data(file_inode(file));
#else
		struct goodix_ts_core *cd = PDE_DATA(file_inode(file));
#endif
	char tmp_buf[10] = {0};
	if (copy_from_user(tmp_buf, buf, count)) {
		return -EFAULT;
	}
	if (is_tp_on()) {
		ts_info("enable glove tap");
		cd->charger_type |= GESTURE_CHARGER_TAP;
	} 
	if (!is_tp_on()) {
		ts_info("disable glove tap");
		cd->charger_type &= ~GESTURE_CHARGER_TAP;
	} 
	return count;
}

static ssize_t ges_single_read(struct file *file, char __user *buf,
				     size_t count, loff_t *pos)
{
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 17, 0))
		struct goodix_ts_core *cd = pde_data(file_inode(file));
#else
		struct goodix_ts_core *cd = PDE_DATA(file_inode(file));
#endif
	uint32_t type = cd->gesture_type;
	char tmp_buf[10] = {0};
	int cnt;

	if (*pos > 0)
		return 0;

	cnt = sprintf(tmp_buf, "%s\n",
			(type & GESTURE_SINGLE_TAP) ? "enable" : "disable");
	if (copy_to_user(buf, tmp_buf, cnt)) {
		return -EFAULT;
	}
	*pos += cnt;
	return cnt;
}

static ssize_t ges_single_write(struct file *file, const char __user *buf,
				     size_t count, loff_t *pos)
{
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 17, 0))
		struct goodix_ts_core *cd = pde_data(file_inode(file));
#else
		struct goodix_ts_core *cd = PDE_DATA(file_inode(file));
#endif
	char tmp_buf[10] = {0};

	if (copy_from_user(tmp_buf, buf, count)) {
		return -EFAULT;
	}

	if (tmp_buf[0] == '1' || tmp_buf[0] == 1) {
		ts_info("enable double tap");
		cd->gesture_type |= GESTURE_SINGLE_TAP;
	} else if (tmp_buf[0] == '0' || tmp_buf[0] == 0) {
		ts_info("disable double tap");
		cd->gesture_type &= ~GESTURE_SINGLE_TAP;
	} else
		ts_err("invalid cmd[%d]", tmp_buf[0]);

	return count;
}

static ssize_t ges_fod_read(struct file *file, char __user *buf,
				     size_t count, loff_t *pos)
{
	#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 17, 0))
		struct goodix_ts_core *cd = pde_data(file_inode(file));
	#else
		struct goodix_ts_core *cd = PDE_DATA(file_inode(file));
	#endif
	uint32_t type = cd->gesture_type;
	char tmp_buf[10] = {0};
	int cnt;

	if (*pos > 0)
		return 0;

	cnt = sprintf(tmp_buf, "%s\n",
			(type & GESTURE_FOD_PRESS) ? "enable" : "disable");
	if (copy_to_user(buf, tmp_buf, cnt)) {
		return -EFAULT;
 	}
	*pos += cnt;
	return cnt;
	}

static ssize_t ges_fod_write(struct file *file, const char __user *buf,
				     size_t count, loff_t *pos)
{
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 17, 0))
		struct goodix_ts_core *cd = pde_data(file_inode(file));
#else
		struct goodix_ts_core *cd = PDE_DATA(file_inode(file));
#endif
	char tmp_buf[10] = {0};

	if (copy_from_user(tmp_buf, buf, count)) {
		return -EFAULT;
	}

	if (tmp_buf[0] == '1' || tmp_buf[0] == 1) {
		ts_info("enable double tap");
		cd->gesture_type |= GESTURE_FOD_PRESS;
	} else if (tmp_buf[0] == '0' || tmp_buf[0] == 0) {
		ts_info("disable double tap");
		cd->gesture_type &= ~GESTURE_FOD_PRESS;
	} else
		ts_err("invalid cmd[%d]", tmp_buf[0]);

	return count;
}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(5, 6, 0))
static const struct proc_ops ges_double_ops = {
	.proc_read = ges_double_read,
	.proc_write = ges_double_write,
};
static const struct proc_ops ges_single_ops = {
	.proc_read = ges_single_read,
	.proc_write = ges_single_write,
};
static const struct proc_ops ges_glove_ops = {
	.proc_read = ges_glove_read,
	.proc_write = ges_glove_write,
};
static const struct proc_ops ges_charger_ops = {
	.proc_read = ges_charger_read,
	.proc_write = ges_charger_write,
};
static const struct proc_ops ges_edge_ops = {
	.proc_read = ges_edge_read,
	.proc_write = ges_edge_write,
};
static const struct proc_ops ges_fod_ops = {
	.proc_read = ges_fod_read,
	.proc_write = ges_fod_write,
};

#else
static const struct file_operations ges_double_ops = {
	.read = ges_double_read,
	.write = ges_double_write,
};
static const struct file_operations ges_single_ops = {
	.read = ges_single_read,
	.write = ges_single_write,
};
static const struct proc_ops ges_edge_ops = {
	.proc_read = ges_edge_read,
	.proc_write = ges_edge_write,
};
static const struct proc_ops ges_glove_ops = {
	.proc_read = ges_glove_read,
	.proc_write = ges_glove_write,
};
static const struct proc_ops ges_charger_ops = {
	.proc_read = ges_charger_read,
	.proc_write = ges_charger_write,
};
static const struct file_operations ges_fod_ops = {
	.read = ges_fod_read,
	.write = ges_fod_write,
};

#endif

int gesture_module_init(struct goodix_ts_core *core_data)
{
	proc_create_data("double_type",
			0660, core_data->proc_dir_entry, &ges_double_ops, core_data);
	proc_create_data("glove_type",
			0660, core_data->proc_dir_entry, &ges_glove_ops, core_data);
	proc_create_data("charger_type",
			0660, core_data->proc_dir_entry, &ges_charger_ops, core_data);
	proc_create_data("edge_type",
			0660, core_data->proc_dir_entry, &ges_edge_ops, core_data);
	proc_create_data("single_type",
			0660, core_data->proc_dir_entry, &ges_single_ops, core_data);
	proc_create_data("fod_type",
			0660, core_data->proc_dir_entry, &ges_fod_ops, core_data);

	ts_info("gesture module init success");
	return 0;

}

void gesture_module_exit(struct goodix_ts_core *core_data)
{
	ts_info("gesture module exit");

	remove_proc_entry("double_type", core_data->proc_dir_entry);
	remove_proc_entry("glove_type", core_data->proc_dir_entry);
	remove_proc_entry("charger_type", core_data->proc_dir_entry);
	remove_proc_entry("edge_type", core_data->proc_dir_entry);
	remove_proc_entry("single_type", core_data->proc_dir_entry);
	remove_proc_entry("fod_type", core_data->proc_dir_entry);
}
