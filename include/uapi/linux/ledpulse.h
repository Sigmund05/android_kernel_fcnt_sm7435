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
#ifndef __LEDPULSE_H__
#define __LEDPULSE_H__

#define LEDPULSE_DEVICE_NAME (256)

struct ledpulse_dev_info {
	char name[LEDPULSE_DEVICE_NAME];
	int data_size;
	int interval_ms;
};

struct ledpulse_pa_info {
	char red;
	char green;
	char ir;
};

#define LEDPULSE_STATUS_SUCCESS		(0x0)
#define LEDPULSE_STATUS_OVERFLOW	(0x1)
#define LEDPULSE_STATUS_OTHERERROR	(0x2)

#define LEDPULSE_MAGIC 'L'

#define LEDPULSE_POWER_CTL			_IO(LEDPULSE_MAGIC, 0x01)
#define LEDPULSE_MEASUREMENT_CTL	_IO(LEDPULSE_MAGIC, 0x02)
#define LEDPULSE_GET_DEVICEINFO		_IOR(LEDPULSE_MAGIC, 0x03, struct ledpulse_dev_info)

#define LEDPULSE_SET_PA_MC			_IOW(LEDPULSE_MAGIC, 0x04, struct ledpulse_pa_info)
#define LEDPULSE_POWER_CTL_MC		_IO(LEDPULSE_MAGIC, 0x05)
#define LEDPULSE_MEASUREMENT_CTL_MC	_IO(LEDPULSE_MAGIC, 0x06)
#define LEDPULSE_GET_GRAY_IR_DATA	_IOW(LEDPULSE_MAGIC, 0x07, int)
#endif /* __LEDPULSE_H__ */
