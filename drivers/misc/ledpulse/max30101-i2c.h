/*
  * COPYRIGHT(C) FUJITSU CONNECTED TECHNOLOGIES LIMITED 2018
  * COPYRIGHT(C) FCNT 2021
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
#ifndef __MAX30101_I2C_H__
#define __MAX30101_I2C_H__

/******************************************************************************/
/* define/macro/enum                                                          */
/******************************************************************************/
/*
 * Debugging
 */
/* Dump FIFO data */
/* #define MAX30101_DUMP_FIFO */

/* Log output macro */
#define MAX30101_LOG_D					LEDPULSE_LOG_D
#define MAX30101_LOG_W					LEDPULSE_LOG_W
#define MAX30101_LOG_I					LEDPULSE_LOG_I
#define MAX30101_LOG_E					LEDPULSE_LOG_E
#define MAX30101_LOG_S					LEDPULSE_LOG_E

#define GRAY_IR_FILE "Gray_Ir.txt"
/*
 * General purpose
 */
#define MAX30101_BIT_MASK(a, mask)		((a) & (mask))
#define MAX30101_BIT_CHK(a, mask)		(MAX30101_BIT_MASK(a, mask) == (mask))
#define MAX30101_RED_1(data)			(*((data) + 0))
#define MAX30101_RED_2(data)			(*((data) + 1))
#define MAX30101_RED_3(data)			(*((data) + 2))

#define MAX30101_IR_1(data)				(*((data) + 3))
#define MAX30101_IR_2(data)				(*((data) + 4))
#define MAX30101_IR_3(data)				(*((data) + 5))

#define MAX30101_GREEN_1(data)			(*((data) + 6))
#define MAX30101_GREEN_2(data)			(*((data) + 7))
#define MAX30101_GREEN_3(data)			(*((data) + 8))

#define MAX30101_DATA_CONSTRUCTION(a, b, c) \
	( \
		(((int)(a) & 0xFF) << 16) | \
		(((int)(b) & 0xFF) << 8) | \
		((int)(c) & 0xFF) \
	)

/*
 * Hardware specification
 */
/* Register address */
#define MAX30101_REG_INT_STATUS_1		(0x00)
	#define MAX30101_PWR_RDY_MASK			(0x1 << 0)
#define MAX30101_REG_INT_STATUS_2		(0x01)
#define MAX30101_REG_INT_ENABLE_1		(0x02)
#define MAX30101_REG_INT_ENABLE_2		(0x03)
#define MAX30101_REG_FIFO_WR_PTR		(0x04)
#define MAX30101_REG_OVF_CNT			(0x05)
#define MAX30101_OVF_CNT_MASK			(0x1F << 0)
#define MAX30101_REG_FIFO_RD_PTR		(0x06)
#define MAX30101_REG_FIFO_DATA_REG		(0x07)
#define MAX30101_REG_FIFO_CFG			(0x08)
#define MAX30101_REG_MODE_CONFIG		(0x09)
#define MAX30101_REG_SPO2_CONFIG		(0x0A)
#define MAX30101_REG_LED1_PA			(0x0C)
#define MAX30101_REG_LED2_PA			(0x0D)
#define MAX30101_REG_LED3_PA			(0x0E)
#define MAX30101_REG_PILOT_PA			(0x10)
#define MAX30101_REG_M_LED_CTRL_1		(0x11)
#define MAX30101_REG_M_LED_CTRL_2		(0x12)
#define MAX30101_REG_TEMP_INTEGER		(0x1F)
#define MAX30101_REG_TEMP_FRACTION		(0x20)
#define MAX30101_REG_DIE_TEMP_CFG		(0x21)
#define MAX30101_REG_PROX_INT_T			(0x30)

/* FIFO data related */
#define MAX30101_FIFO_MAX_NUM			(32)
#define MAX30101_1SAMPLE_SIZE			(3)
#define MAX30101_LED_NUM				(3)

/*
 * Hardware control specification
 */
#define MAX30101_POWER_INT_WAIT_MS		(1)
#define MAX30101_POWER_REG_READ_WAIT_MS	(1)
#define MAX30101_POWER_RST_INTERVAL_MS	(5)
#define MAX30101_POWER_RST_INTERVAL_NS	\
	(MAX30101_POWER_RST_INTERVAL_MS * NSEC_PER_MSEC)
#define MAX30101_POWER_RECOVERY_CNT		(1)
#define MAX30101_TEMP_WAIT_MS			(29)
#define MAX30101_TEMP_THRESHOLD			(1)
#define MAX30101_READ_SAMPLE_NUM		(10)
#define MAX30101_SOFT_RESET_WAIT_MS		(1)
#define MAX30101_LOAD_UA				(100000)

/* Register setting value */
#define MAX30101_MODE_RESET_VALUE		(0x40)
#define MAX30101_MODE_MEASURE_VALUE		(0x07)
#define MAX30101_PA_RED_VALUE			(0x00)
#define MAX30101_PA_IR_VALUE			(0x80)
#define MAX30101_PA_GREEN_INIT_VALUE	(0x00)
#define MAX30101_PA_GREEN_VALUE			(0x80)
#define MAX30101_SPO2_CONF_VALUE		(0x51)
#define MAX30101_FIFO_CONF_VALUE		(0x2C)
#define MAX30101_M_LED_CTRL1_VALUE		(0x21)
#define MAX30101_M_LED_CTRL2_VALUE		(0x03)
#define MAX30101_INT_ENABLE_1_VALUE		(0x80)
#define MAX30101_PILOT_PA_VALUE			(0x00)
#define MAX30101_PROX_INT_T_VALUE		(0x00)
#define MAX30101_UNUSED					(0)

/*
 * Driver
 */
#define MAX30101_FIFO_NUM				(50)
#define MAX30101_FIFO_DATA_SIZE			\
	(MAX30101_1SAMPLE_SIZE * MAX30101_LED_NUM * MAX30101_READ_SAMPLE_NUM)
#define MAX30101_FIFO_INTERVAL_MS		(50)
#define MAX30101_DEVICE_NAME			"MAX30101r"

/* state */
enum MAX30101_STATE {
	MAX30101_STATE_OFF = 0,
	MAX30101_STATE_ON,
	MAX30101_STATE_MEASURING,
};

/* sequence type */
enum MAX30101_SEQUENCE {
	MAX30101_SEQUENCE_I2C = 0,
	MAX30101_SEQUENCE_DELAY_MS,
};

/*
 * Green LED control
 */
/* APNV */
#define MAX30101_APNV_FINGER_FITTING	(49441)
#define MAX30101_FINGER_FITTING_SIZE	(6)
#define MAX30101_FINGER_DETECT_DEFAULT	200000

/* Sample number to take the average to judge whether it is touched */
#define MAX30101_TOUCH_AVERAGE_COUNT	(20)

/*
 * Overflow control
 */
/* Interval for accumulating the number of overflows */
#define MAX30101_OVF_ACCUM_INTERVAL		(200)	/* 200sample = 1s */
/* Number of times to accumulate overflow count */
#define MAX30101_OVF_ACCUM_NUM			(6)
/* Number of samples accumulating the number of overflows */
#define MAX30101_OVF_ACCUM_TOTAL		\
	(MAX30101_OVF_ACCUM_INTERVAL * MAX30101_OVF_ACCUM_NUM)

/* Number of overflows allowed in one interval */
#define MAX30101_OVF_NOTIFY_THRESHOLD1	(3)
/* Number of intervals allowed in the accumulation period */
#define MAX30101_OVF_NOTIFY_THRESHOLD2	(1)

#endif /* __MAX30101_I2C_H__ */
