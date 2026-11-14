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
#ifndef __LEDPULSE_LOCAL_H__
#define __LEDPULSE_LOCAL_H__

#define ledpulse_h void *

struct ledpulse_dev_operations {
	void *device;
	int (*power_ctl)(void *, bool);
	int (*measurement_ctl)(void *, bool);
	int (*get_device_info)(void *, void __user *);
	int (*read_data)(void *, void __user *, size_t, loff_t *);
	bool (*is_data_exist)(void *);
	int (*set_pa_info)(void *, void __user *);
	int (*get_gray_ir_data)(void *, void __user *);
};

#define LEDPULSE_LOG_D(str, ...) \
	pr_debug("[LEDPULSE][DEBUG]"str, ## __VA_ARGS__)
#define LEDPULSE_LOG_W(str, ...) \
	pr_warn("[LEDPULSE][WARN]"str, ## __VA_ARGS__)
#define LEDPULSE_LOG_I(str, ...) \
	pr_info("[LEDPULSE][INFO]"str, ## __VA_ARGS__)
#define LEDPULSE_LOG_E(str, ...) \
	pr_err("[LEDPULSE][ERROR]"str, ## __VA_ARGS__)

ledpulse_h ledpulse_register(struct ledpulse_dev_operations *dev_ops);
int ledpulse_deregister(ledpulse_h handle);
void ledpulse_notify(ledpulse_h handle);
#endif /* __LEDPULSE_LOCAL_H__ */
