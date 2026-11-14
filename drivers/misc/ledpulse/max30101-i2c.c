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
#include <linux/string.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/list.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/jiffies.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/of_gpio.h>
#include <linux/compat.h>
#include <linux/sched.h>
#include <linux/regulator/consumer.h>
#include <linux/ledpulse.h>
//#include <linux/nonvolatile_common.h>
#include <linux/dma-mapping.h>

#include "max30101-i2c.h"
#include "ledpulse_local.h"
#include <linux/hardware_info.h>

#define CONFIG_OF 1
#define MAX30101_DUMP_FIFO 1
/******************************************************************************/
/* Structure                                                                  */
/******************************************************************************/
/* Green LED control */
struct max30101_g_ctrl {
	bool g_lighting;
	bool force_pa;
	int touch_threshold;
	int touch_count;
	int ir_average;
};

/* Overflow control */
struct max30101_ovf_ctrl {
	unsigned int ovf_cnt[MAX30101_OVF_ACCUM_NUM];
	unsigned int read_cnt;
};

struct max30101_data {
	struct device *dev;
	// struct regulator *vled_reg;
	// struct regulator *vdd_reg;
	struct pinctrl *pinctrl;
	struct pinctrl_state *active_state;
	struct mutex state_mutex;
	int irq_gpio;
	int vdd_en;
    int led_boost_en;
    int vdd_gpio;
	int irq_no;
	enum MAX30101_STATE state;
	ledpulse_h ledpulse;
	spinlock_t q_lock;
	bool first_data;
	struct timespec64 lastpoff_ts;
	/* Green LED control */
	struct max30101_g_ctrl g_ctrl;
	/* Overflow control */
	struct max30101_ovf_ctrl ovf_ctrl;
	uint8_t *i2c_buff_p;
	// dma_addr_t paddr;
};

struct max30101_i2c_data {
	unsigned char addr;
	uint8_t *data_p;
	unsigned int length;
	bool write;
};

struct max30101_i2c_sequence {
	enum MAX30101_SEQUENCE type;
	struct max30101_i2c_data i2c;
	unsigned long delay_ms;
};

struct max30101_fifo_data {
	struct list_head list;
	uint8_t status;
	uint8_t data[MAX30101_FIFO_DATA_SIZE];
	bool is_malloc;
};

/******************************************************************************/
/* define/macro/enum                                                          */
/******************************************************************************/
#define max30101_mdelay(ms)		udelay((ms) * USEC_PER_MSEC)
#define max30101_mdelay_l(ms)	mdelay((ms))
#define max30101_ndelay(ns)		udelay((ns) / NSEC_PER_USEC)

#define MAX30101_I2C_W(reg, value_p)	\
	{	MAX30101_SEQUENCE_I2C, \
		{ reg, value_p, 1, true, }, \
		MAX30101_UNUSED, \
	}

#define MAX30101_DELAY_MS(value)	\
	{	MAX30101_SEQUENCE_DELAY_MS, \
		{ MAX30101_UNUSED, MAX30101_UNUSED, MAX30101_UNUSED, false, }, \
		value, \
	}

#define MAX30101_CALC_SAMPLE_NUM(wr, rd, ov, result)	\
	do { \
		if ((wr) > (rd)) { \
			(result) = (wr) - (rd); \
		} else if ((wr) < (rd)) { \
			(result) = MAX30101_FIFO_MAX_NUM - (rd) + (wr); \
		} else if ((wr) == (rd)) { \
			if (MAX30101_BIT_MASK((ov), MAX30101_OVF_CNT_MASK) != 0) \
				(result) = MAX30101_FIFO_MAX_NUM; \
			else \
				(result) = 0; \
		} \
	} while (0)
/******************************************************************************/
/* prototype                                                                  */
/******************************************************************************/
/* Queue control functions                                                    */
static struct max30101_fifo_data *_max30101_get_free_q(
	struct max30101_data *info);

static void _max30101_put_free_q(
	struct max30101_data *info, struct max30101_fifo_data *fifo);

static struct max30101_fifo_data *_max30101_get_use_q(
	struct max30101_data *info);

static void _max30101_put_use_q(
	struct max30101_data *info, struct max30101_fifo_data *fifo);

static void _max30101_release_malloc_q(struct max30101_data *info);
static void _max30101_init_q(struct max30101_data *info);

/* Measurement control functions                                              */
static bool _max30101_measurement_recovery(
	struct max30101_data *info, int *retry);

static int _max30101_measurement_start(struct max30101_data *info);
static int _max30101_measurement_end(struct max30101_data *info);
static int _max30101_measurement_ctl(void *dev_data, bool enable);

/* Power control functions                                                    */
static bool _max30101_power_recovery(struct max30101_data *info, int *retry);
static int _max30101_power_on(struct max30101_data *info);
static int _max30101_power_off(struct max30101_data *info);
static int _max30101_power_ctl(void *dev_data, bool enable);

/* Data reading functions                                                     */
static int _max30101_read_data(void *dev_data, void __user *out,
	size_t count, loff_t *offset);

static bool _max30101_is_data_exist(void *dev_data);

/* Device information acquisition functions                                   */
static int _max30101_get_device_info(void *dev_data, void __user *out);
/* Pulse amplitude setting functions                                          */
static int _max30101_set_pa_info(void *dev_data, void __user *in);

static int _max30101_get_gray_ir_data(void *dev_data, void __user *in);

/* I2C control functions                                                      */
static int _max30101_reg_read(struct max30101_data *info,
	unsigned char addr, unsigned char *data, unsigned int length);

static int _max30101_reg_write(struct max30101_data *info,
	unsigned char addr, unsigned char *data, unsigned int length);

static int _max30101_reg_sequence(struct max30101_data *info,
	struct max30101_i2c_sequence *sequence, unsigned int num);

/* Green LED control */
static void _max30101_init_touch_count(struct max30101_g_ctrl *g_ctrl);
static void _max30101_calc_ir_average(
	struct max30101_g_ctrl *g_ctrl, int value);

static void _max30101_set_force_green(
	struct max30101_g_ctrl *g_ctrl, bool enable, uint8_t pa_value);

static void _max30101_init_g_ctrl(struct max30101_g_ctrl *g_ctrl);
static void _max30101_ctrl_green_led(
	struct max30101_data *info, struct max30101_g_ctrl *g_ctrl);

/* Overflow control */
static void _max30101_init_ovf_ctrl(struct max30101_ovf_ctrl *ovf_ctrl);
static bool _max30101_ovf_ctrl(
	struct max30101_ovf_ctrl *ovf_ctrl, uint8_t ovf_cnt);

/* FIFO data reading functions                                                */
static void _max30101_processing_fifo_data(
	struct max30101_data *info,
	uint8_t *data_i,
	uint8_t *data_o,
	unsigned int size);

static int _max30101_read_fifo_data(struct max30101_data *info);

/* Driver local functions                                                     */
static int _max30101_parse_dt(struct max30101_data *info);
// static int _max30101_init_regulator(struct max30101_data *info);
static int _max30101_ctrl_regulator(struct max30101_data *info, bool enable);
static int _max30101_init_pinctl(struct max30101_data *info);
static int _max30101_init_irq(struct max30101_data *info);
static void _max30101_release(struct max30101_data *info);

/* Driver functions                                                           */
static irqreturn_t max30101_irq_thread(int irq, void *data);

extern void get_hardware_info_data(enum hardware_id id, const void *data);
/******************************************************************************/
/* LOCAL DATA                                                                 */
/******************************************************************************/
static LIST_HEAD(q_use);
static LIST_HEAD(q_free);
static struct max30101_fifo_data fifo_data[MAX30101_FIFO_NUM];

static uint8_t pa_red = MAX30101_PA_RED_VALUE;
static uint8_t pa_ir = MAX30101_PA_IR_VALUE;
static uint8_t pa_green = MAX30101_PA_GREEN_VALUE;
static uint8_t pa_init_green = MAX30101_PA_GREEN_INIT_VALUE;
static uint8_t spo2_conf = MAX30101_SPO2_CONF_VALUE;
static uint8_t fifo_conf = MAX30101_FIFO_CONF_VALUE;
static uint8_t flex_ctrl1 = MAX30101_M_LED_CTRL1_VALUE;
static uint8_t flex_ctrl2 = MAX30101_M_LED_CTRL2_VALUE;
static bool always_on;	/* = false */
static bool ovf_test;	/* = false */
static unsigned int ovf_threshold1 = MAX30101_OVF_NOTIFY_THRESHOLD1;
static unsigned int ovf_threshold2 = MAX30101_OVF_NOTIFY_THRESHOLD2;

static uint8_t mode_reset = MAX30101_MODE_RESET_VALUE;
static uint8_t pilot_pa = MAX30101_PILOT_PA_VALUE;
static uint8_t prox_int_t = MAX30101_PROX_INT_T_VALUE;
static uint8_t int_enable1 = MAX30101_INT_ENABLE_1_VALUE;
static uint8_t mode_measure = MAX30101_MODE_MEASURE_VALUE;
static unsigned int finger_detect_ir_data = MAX30101_FINGER_DETECT_DEFAULT;


module_param(pa_red, byte, 0644);
module_param(pa_green, byte, 0644);
module_param(pa_ir, byte, 0644);
module_param(spo2_conf, byte, 0644);
module_param(fifo_conf, byte, 0644);
module_param(flex_ctrl1, byte, 0644);
module_param(flex_ctrl2, byte, 0644);
module_param(always_on, bool, 0644);
module_param(ovf_test, bool, 0644);
module_param(ovf_threshold1, uint, 0644);
module_param(ovf_threshold2, uint, 0644);
module_param(finger_detect_ir_data, uint, 0644);

static struct max30101_i2c_sequence init_sequence[] = {
	MAX30101_I2C_W(MAX30101_REG_MODE_CONFIG,	&mode_reset),

	MAX30101_DELAY_MS(MAX30101_SOFT_RESET_WAIT_MS),

	MAX30101_I2C_W(MAX30101_REG_FIFO_CFG,		&fifo_conf),
	MAX30101_I2C_W(MAX30101_REG_SPO2_CONFIG,	&spo2_conf),
	MAX30101_I2C_W(MAX30101_REG_LED1_PA,		&pa_red),
	MAX30101_I2C_W(MAX30101_REG_LED2_PA,		&pa_ir),
	MAX30101_I2C_W(MAX30101_REG_LED3_PA,		&pa_init_green),
	MAX30101_I2C_W(MAX30101_REG_PILOT_PA,		&pilot_pa),
	MAX30101_I2C_W(MAX30101_REG_PROX_INT_T,		&prox_int_t),
	MAX30101_I2C_W(MAX30101_REG_M_LED_CTRL_1,	&flex_ctrl1),
	MAX30101_I2C_W(MAX30101_REG_M_LED_CTRL_2,	&flex_ctrl2),
	MAX30101_I2C_W(MAX30101_REG_INT_ENABLE_1,	&int_enable1),
	MAX30101_I2C_W(MAX30101_REG_MODE_CONFIG,	&mode_measure),
};

static struct max30101_i2c_sequence term_sequence[] = {
	MAX30101_I2C_W(MAX30101_REG_MODE_CONFIG,	&mode_reset),
};

static struct ledpulse_dev_operations max30101_ops = {
	.device = NULL,
	.power_ctl = _max30101_power_ctl,
	.measurement_ctl = _max30101_measurement_ctl,
	.get_device_info = _max30101_get_device_info,
	.read_data = _max30101_read_data,
	.is_data_exist = _max30101_is_data_exist,
	.set_pa_info = _max30101_set_pa_info,
	.get_gray_ir_data = _max30101_get_gray_ir_data,
};

/******************************************************************************/
/* Queue control functions                                                    */
/******************************************************************************/
static struct max30101_fifo_data *_max30101_get_free_q(
	struct max30101_data *info)
{
	struct max30101_fifo_data *new_q = NULL;
	unsigned long flags;

	spin_lock_irqsave(&info->q_lock, flags);

	new_q = list_first_entry_or_null(&q_free, struct max30101_fifo_data, list);
	if (new_q != NULL)
		list_del_init(&new_q->list);

	spin_unlock_irqrestore(&info->q_lock, flags);

	if (new_q == NULL) {
		/* it is fail-safe */
		new_q = kmalloc(sizeof(struct max30101_fifo_data), GFP_KERNEL);
		MAX30101_LOG_S("queue overflow and new memory alloc:%p\n", new_q);
		if (new_q != NULL) {
			INIT_LIST_HEAD(&new_q->list);
			new_q->status = LEDPULSE_STATUS_SUCCESS;
			new_q->is_malloc = true;
			memset(&new_q->data[0], 0, sizeof(new_q->data));
		}
	}
	return new_q;
}

static void _max30101_put_free_q(
	struct max30101_data *info, struct max30101_fifo_data *fifo)
{
	unsigned long flags;

	spin_lock_irqsave(&info->q_lock, flags);

	fifo->status = LEDPULSE_STATUS_SUCCESS;
	memset(&fifo->data[0], 0, sizeof(fifo->data));
	list_add_tail(&fifo->list, &q_free);

	spin_unlock_irqrestore(&info->q_lock, flags);
}

static struct max30101_fifo_data *_max30101_get_use_q(
	struct max30101_data *info)
{
	struct max30101_fifo_data *use_q = NULL;
	unsigned long flags;

	spin_lock_irqsave(&info->q_lock, flags);

	use_q = list_first_entry_or_null(&q_use, struct max30101_fifo_data, list);
	if (use_q != NULL)
		list_del_init(&use_q->list);

	spin_unlock_irqrestore(&info->q_lock, flags);

	return use_q;
}

static void _max30101_put_use_q(
	struct max30101_data *info, struct max30101_fifo_data *fifo)
{
	unsigned long flags;

	spin_lock_irqsave(&info->q_lock, flags);

	list_add_tail(&fifo->list, &q_use);

	spin_unlock_irqrestore(&info->q_lock, flags);
}

static void _max30101_release_malloc_q(struct max30101_data *info)
{
	unsigned long flags;
	struct list_head *item = NULL, *tmp = NULL;
	struct max30101_fifo_data *q = NULL;

	spin_lock_irqsave(&info->q_lock, flags);

	list_for_each_safe(item, tmp, &q_free) {
		q = list_entry(item, struct max30101_fifo_data, list);
		if (q && q->is_malloc) {
			MAX30101_LOG_I("Release the queue acquired by malloc:%p\n", q);
			list_del(&q->list);
			kfree(q);
		}
	}
	list_for_each_safe(item, tmp, &q_use) {
		q = list_entry(item, struct max30101_fifo_data, list);
		if (q && q->is_malloc) {
			MAX30101_LOG_I("Release the queue acquired by malloc:%p\n", q);
			list_del(&q->list);
			kfree(q);
		}
	}
	spin_unlock_irqrestore(&info->q_lock, flags);
}

static void _max30101_init_q(struct max30101_data *info)
{
	int i;
	unsigned long flags;

	spin_lock_irqsave(&info->q_lock, flags);

	INIT_LIST_HEAD(&q_use);
	INIT_LIST_HEAD(&q_free);

	for (i = 0; i < MAX30101_FIFO_NUM; i++) {
		fifo_data[i].is_malloc = false;
		fifo_data[i].status = LEDPULSE_STATUS_SUCCESS;
		memset(&fifo_data[i].data[0], 0, sizeof(fifo_data[i].data));
		INIT_LIST_HEAD(&fifo_data[i].list);
		list_add_tail(&fifo_data[i].list, &q_free);
	}
	info->first_data = true;

	spin_unlock_irqrestore(&info->q_lock, flags);
}

/******************************************************************************/
/* Measurement control functions                                              */
/******************************************************************************/
static bool _max30101_measurement_recovery(
	struct max30101_data *info, int *retry)
{
	bool ret = false;

	if (*retry >= MAX30101_POWER_RECOVERY_CNT) {
		MAX30101_LOG_S("Hardware has failed\n");
		ret = false;
	} else {
		MAX30101_LOG_E("%s recovery start cnt:%d\n", __func__, *retry);

		if (_max30101_power_off(info)) {
			MAX30101_LOG_E("failed to power off\n");
			/* fall through */
		}

		if (_max30101_power_on(info)) {
			MAX30101_LOG_E("failed to power on\n");
			/* fall through */
		}
		(*retry)++;
		ret = true;
	}
	return ret;
}

static int _max30101_measurement_start(struct max30101_data *info)
{
	int rc = 0;
	int8_t temp = 0;
	uint8_t temp_cfg = 0x01;
	int retry = 0;

	MAX30101_LOG_D("'%s' enter.", __func__);

	do {
		if (info->state == MAX30101_STATE_MEASURING) {
			MAX30101_LOG_W("%s already start\n", __func__);
			break;
		}

		if (info->state == MAX30101_STATE_OFF) {
			MAX30101_LOG_W("%s bad request\n", __func__);
			rc = -EBADRQC;
			break;
		}
retry:
		do {
			rc = _max30101_reg_write(info, MAX30101_REG_DIE_TEMP_CFG, &temp_cfg, 1);
			if (rc) {
				MAX30101_LOG_E("DIE_TEMP_CFG write error\n");
				rc = -EIO;
				break;
			}

			max30101_mdelay_l(MAX30101_TEMP_WAIT_MS);

			rc = _max30101_reg_read(info, MAX30101_REG_TEMP_INTEGER, (unsigned char *)&temp, 1);
			if (rc || !(temp >= MAX30101_TEMP_THRESHOLD)) {
				MAX30101_LOG_E("Temperature error ret:%d temp:%d\n", rc, temp);
				rc = -EIO;
				break;
			}
			MAX30101_LOG_D("Temperature Integer:%d(0x%x)\n", temp, temp);

		} while (0);

		if (rc) {
			if (_max30101_measurement_recovery(info, &retry)) {
				rc = 0;
				temp = 0;
				goto retry;
			}
			break;
		}

		_max30101_init_q(info);
		/* Green LED control */
		_max30101_init_touch_count(&info->g_ctrl);
		/* Overflow control */
		_max30101_init_ovf_ctrl(&info->ovf_ctrl);

		MAX30101_LOG_D("%s enable_irq\n", __func__);
		enable_irq(info->irq_no);
		enable_irq_wake(info->irq_no);

		rc = _max30101_reg_sequence(info, init_sequence, ARRAY_SIZE(init_sequence));
		if (rc) {
			MAX30101_LOG_E("init_sequence error\n");
			disable_irq_wake(info->irq_no);
			disable_irq_nosync(info->irq_no);
			rc = -EIO;
			break;
		}

		MAX30101_LOG_D("state measuring\n");
		info->state = MAX30101_STATE_MEASURING;

	} while (0);

	return rc;
}

static int _max30101_measurement_end(struct max30101_data *info)
{
	int rc = 0;

	MAX30101_LOG_D("'%s' enter.", __func__);

	do {
		if (info->state == MAX30101_STATE_ON) {
			MAX30101_LOG_W("%s already end\n", __func__);
			break;
		}

		if (info->state == MAX30101_STATE_OFF) {
			MAX30101_LOG_W("%s bad request\n", __func__);
			rc = -EBADRQC;
			break;
		}

		rc = _max30101_reg_sequence(info, term_sequence, ARRAY_SIZE(term_sequence));
		if (rc) {
			MAX30101_LOG_E("term_sequence error\n");
			rc = -EIO;
			break;
		}

		MAX30101_LOG_D("%s disable_irq\n", __func__);
		disable_irq_wake(info->irq_no);
		disable_irq_nosync(info->irq_no);

		MAX30101_LOG_D("state on\n");
		info->state = MAX30101_STATE_ON;

		/* Green LED control */
		_max30101_set_force_green(&info->g_ctrl, false, MAX30101_PA_GREEN_INIT_VALUE);
		_max30101_release_malloc_q(info);

	} while (0);

	return rc;
}

static int _max30101_measurement_ctl(void *dev_data, bool enable)
{
	struct max30101_data *info = (struct max30101_data *)dev_data;
	int rc = 0;

	MAX30101_LOG_D("'%s' enter.", __func__);
	mutex_lock(&info->state_mutex);

	if (enable)
		rc = _max30101_measurement_start(info);
	else
		rc = _max30101_measurement_end(info);

	mutex_unlock(&info->state_mutex);
	MAX30101_LOG_D("%s e\n", __func__);
	return rc;
}

/******************************************************************************/
/* Power control functions                                                    */
/******************************************************************************/
static bool _max30101_power_recovery(struct max30101_data *info, int *retry)
{
	bool ret = false;
	MAX30101_LOG_D("'%s' enter.", __func__);
	if (*retry >= MAX30101_POWER_RECOVERY_CNT) {
		MAX30101_LOG_S("Hardware has failed\n");
		ret = false;
	} else {
		MAX30101_LOG_E("%s recovery start cnt:%d\n", __func__, *retry);

		if (_max30101_ctrl_regulator(info, false)) {
			MAX30101_LOG_E("failed to regulator disable\n");
			/* fall through */
		}

		if (_max30101_ctrl_regulator(info, true)) {
			MAX30101_LOG_E("failed to regulator enable\n");
			/* fall through */
		}
		(*retry)++;
		ret = true;
	}
	return ret;
}

static int _max30101_power_on(struct max30101_data *info)
{
	int rc = 0;
	int power_retry = 0;
	uint8_t int_status1 = 0;

	MAX30101_LOG_D("'%s' enter.", __func__);

	do {
		if (info->state != MAX30101_STATE_OFF) {
			MAX30101_LOG_W("%s already enable\n", __func__);
			break;
		}

		rc = _max30101_ctrl_regulator(info, true);
		if (rc)
			break;

retry:
		do {
			max30101_mdelay(MAX30101_POWER_INT_WAIT_MS);

			if (gpio_get_value(info->irq_gpio) != 0 && !always_on) {
				MAX30101_LOG_E("int GPIO not LOW\n");
				rc = -EIO;
				break;
			}
			MAX30101_LOG_E("1->irq_no=%d\n", gpio_get_value(info->irq_gpio));
			max30101_mdelay(MAX30101_POWER_REG_READ_WAIT_MS);

			rc = _max30101_reg_read(info, MAX30101_REG_INT_STATUS_1,
									&int_status1, 1);
			MAX30101_LOG_E("2->irq_no=%d\n", gpio_get_value(info->irq_gpio));
			if (rc ||
				(!MAX30101_BIT_CHK(int_status1, MAX30101_PWR_RDY_MASK) &&
				!always_on)) {
				MAX30101_LOG_E("power ready error ret:%d status:0x%x\n", rc, int_status1);
				rc = -EIO;
				break;
			}
			MAX30101_LOG_E("3->irq_no=%d\n", gpio_get_value(info->irq_gpio));
			max30101_mdelay(MAX30101_POWER_INT_WAIT_MS);
			
			if (gpio_get_value(info->irq_gpio) != 1) {
				MAX30101_LOG_E("int GPIO not HIGH\n");
				rc = -EIO;
				break;
			}

		} while (0);

		if (rc) {
			if (_max30101_power_recovery(info, &power_retry)) {
				rc = 0;
				int_status1 = 0;
				goto retry;
			}
			break;
		}
		MAX30101_LOG_D("state on\n");
		info->state = MAX30101_STATE_ON;

	} while (0);

	if (rc)
		_max30101_ctrl_regulator(info, false);

	return rc;
};

static int _max30101_power_off(struct max30101_data *info)
{
	int rc = 0;

	MAX30101_LOG_D("'%s' enter.", __func__);

	do {
		if (info->state == MAX30101_STATE_OFF) {
			MAX30101_LOG_W("%s already disable\n", __func__);
			break;
		}

		if (info->state == MAX30101_STATE_MEASURING) {
			MAX30101_LOG_W("%s stop the measurement\n", __func__);
			rc = _max30101_measurement_end(info);
			if (rc) {
				MAX30101_LOG_E("error measurement_end %d\n", rc);
				/* fall through */
			}
		}

		rc = _max30101_ctrl_regulator(info, false);
		if (rc) {
			MAX30101_LOG_E("error regulator disable %d\n", rc);
			/* fall through */
		}

		MAX30101_LOG_D("state off\n");
		info->state = MAX30101_STATE_OFF;
		/* We will not return an error to prevent higher confusion. */
		rc = 0;

	} while (0);

	return rc;
};

static int _max30101_power_ctl(void *dev_data, bool enable)
{
	struct max30101_data *info = (struct max30101_data *)dev_data;
	int rc = 0;

	MAX30101_LOG_D("'%s' enter.", __func__);
	mutex_lock(&info->state_mutex);

	if (enable)
		rc = _max30101_power_on(info);
	else
		rc = _max30101_power_off(info);

	mutex_unlock(&info->state_mutex);
	MAX30101_LOG_D("%s e\n", __func__);

	return rc;
};

/******************************************************************************/
/* Data reading functions                                                     */
/******************************************************************************/
static int _max30101_read_data(void *dev_data, void __user *out,
	size_t count, loff_t *offset)
{
	struct max30101_data *info = (struct max30101_data *)dev_data;
	struct max30101_fifo_data *fifo = NULL;
	int ret = 0;

	MAX30101_LOG_D("'%s' enter.", __func__);
	do {
		if (count < MAX30101_FIFO_DATA_SIZE + 1) {
			ret = -EINVAL;
			MAX30101_LOG_E("%s:count is small:%zu\n", __func__, count);
			break;
		}

		fifo = _max30101_get_use_q(info);
		if (!fifo) {
			ret = -EAGAIN;
			MAX30101_LOG_E("%s:fifo null\n", __func__);
			break;
		}

		if (copy_to_user(out, &fifo->status, sizeof(fifo->status))) {
			ret = -EFAULT;
			MAX30101_LOG_E("%s:failed to copy_to_user status\n", __func__);
			break;
		}

		if (copy_to_user((uint8_t __user *)out + sizeof(fifo->status),
					&fifo->data[0], sizeof(fifo->data))) {
			ret = -EFAULT;
			MAX30101_LOG_E("%s:failed to copy_to_user data\n", __func__);
			break;
		}

	} while (0);

	if (fifo)
		_max30101_put_free_q(info, fifo);

	return ret;
}

static bool _max30101_is_data_exist(void *dev_data)
{
	return !list_empty(&q_use);
}

/******************************************************************************/
/* Device information acquisition functions                                   */
/******************************************************************************/
static int _max30101_get_device_info(void *dev_data, void __user *out)
{
	struct ledpulse_dev_info dev_info;
	int ret = 0;

	MAX30101_LOG_D("'%s' enter.", __func__);

	strlcpy(&dev_info.name[0], MAX30101_DEVICE_NAME, sizeof(dev_info.name));
	dev_info.data_size = MAX30101_FIFO_DATA_SIZE + 1;
	dev_info.interval_ms = MAX30101_FIFO_INTERVAL_MS;

	if (copy_to_user(out, &dev_info, sizeof(dev_info))) {
		ret = -EFAULT;
		MAX30101_LOG_E("%s:failed to copy_to_user\n", __func__);
	}
	return ret;
}

/******************************************************************************/
/* Pulse amplitude setting functions                                          */
/******************************************************************************/
static int _max30101_set_pa_info(void *dev_data, void __user *in)
{
	struct max30101_data *info = (struct max30101_data *)dev_data;
	struct ledpulse_pa_info	pa;
	int ret = 0;

	MAX30101_LOG_D("'%s' enter.", __func__);

	do {
		if (copy_from_user(&pa, in, sizeof(pa))) {
			ret = -EFAULT;
			MAX30101_LOG_E("%s:failed to copy_from_user\n", __func__);
			break;
		}

		MAX30101_LOG_I("%s:R:0x%x I:0x%x G:0x%x\n",
					__func__, pa.red, pa.ir, pa.green);
		pa_red = pa.red;
		pa_ir = pa.ir;
		/* Green LED control */
		_max30101_set_force_green(&info->g_ctrl, true, pa.green);

	} while (0);

	return ret;
}

static int _max30101_get_gray_ir_data(void *dev_data, void __user *in)
{
	int gray_ir_data;
	int ret = 0;

	MAX30101_LOG_D("'%s' enter.", __func__);

	do {
		if (copy_from_user(&gray_ir_data, in, sizeof(gray_ir_data))) {
			ret = -EFAULT;
			MAX30101_LOG_E("%s:failed to copy_from_user\n", __func__);
 			finger_detect_ir_data = MAX30101_FINGER_DETECT_DEFAULT;
			break;
		}

		MAX30101_LOG_I("%s:gray_ir_data %d\n", __func__, gray_ir_data);
		finger_detect_ir_data = gray_ir_data;

	} while (0);

	return ret;
}


/******************************************************************************/
/* I2C control functions                                                      */
/******************************************************************************/
static int _max30101_reg_read(struct max30101_data *info,
	unsigned char addr, unsigned char *data, unsigned int length)
{
	#define MAX30101_I2C_READ_MSG_NUM	(2)
	#define MAX30101_I2C_READ_MAX_LEN	(256)

	struct i2c_client *i2c = to_i2c_client(info->dev);
	unsigned char buf = addr;
	struct i2c_msg msg[MAX30101_I2C_READ_MSG_NUM];
	int retval;

	if (length > MAX30101_I2C_READ_MAX_LEN) {
		MAX30101_LOG_E("I2c read fail ADR:0x%02x len:%d\n", addr, length);
		return -EINVAL;
	}

	msg[0].addr = i2c->addr;
	msg[0].flags = 0;
	msg[0].len = sizeof(buf);
	msg[0].buf = &buf;

	msg[1].addr = i2c->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = (unsigned short)length;
	msg[1].buf = data;

	MAX30101_LOG_D("I2c read ADR:0x%02x len:%d\n", addr, length);

	retval = i2c_transfer(i2c->adapter, msg, MAX30101_I2C_READ_MSG_NUM);
	if (retval == MAX30101_I2C_READ_MSG_NUM)
		retval = 0;
	else
		MAX30101_LOG_E("I2c read fail ADR:0x%02x RET:%d\n", addr, retval);

	return retval;
}

static int _max30101_reg_write(struct max30101_data *info,
		unsigned char addr, unsigned char *data, unsigned int length)
{
	#define MAX30101_I2C_WRITE_MSG_NUM	(1)
	#define MAX30101_I2C_WRITE_MAX_LEN	(1)

	struct i2c_client *i2c = to_i2c_client(info->dev);
	unsigned char buf[MAX30101_I2C_WRITE_MAX_LEN + 1];
	struct i2c_msg msg[MAX30101_I2C_WRITE_MSG_NUM];
	int retval;

	if (length > MAX30101_I2C_WRITE_MAX_LEN) {
		MAX30101_LOG_E("I2c write fail ADR:0x%02x len:%d\n", addr, length);
		return -EINVAL;
	}

	buf[0] = addr;
	memcpy(&buf[1], data, length);

	msg[0].addr = i2c->addr;
	msg[0].flags = 0;
	msg[0].len = (unsigned short)length + 1;
	msg[0].buf = &buf[0];

	MAX30101_LOG_D("I2c write ADR:0x%02x len:%d\n", addr, length);

	retval = i2c_transfer(i2c->adapter, msg, MAX30101_I2C_WRITE_MSG_NUM);
	if (retval == MAX30101_I2C_WRITE_MSG_NUM)
		retval = 0;
	else
		MAX30101_LOG_E("I2c write fail ADR:0x%02x RET:%d\n", addr, retval);

	return retval;
}

static int _max30101_reg_sequence(struct max30101_data *info,
		struct max30101_i2c_sequence *sequence, unsigned int num)
{
	int rc = 0;
	unsigned int i;

	MAX30101_LOG_D("'%s' enter.", __func__);

	for (i = 0; i < num; i++) {
		switch (sequence[i].type) {
		case MAX30101_SEQUENCE_I2C:
			if (sequence[i].i2c.write) {
				rc = _max30101_reg_write(info, sequence[i].i2c.addr,
										sequence[i].i2c.data_p,
										sequence[i].i2c.length);
			} else {
				rc = _max30101_reg_read(info, sequence[i].i2c.addr,
										sequence[i].i2c.data_p,
										sequence[i].i2c.length);
			}
			break;
		case MAX30101_SEQUENCE_DELAY_MS:
			max30101_mdelay(sequence[i].delay_ms);
			break;
		default:
			break;
		}
		if (rc)
			break;
	}

	return rc;
}

/******************************************************************************/
/* Green LED control functions                                                */
/******************************************************************************/
static void _max30101_init_touch_count(struct max30101_g_ctrl *g_ctrl)
{
	g_ctrl->g_lighting = false;
	g_ctrl->touch_count = 0;
	g_ctrl->ir_average = 0;
}

static void _max30101_calc_ir_average(struct max30101_g_ctrl *g_ctrl, int value)
{
	g_ctrl->touch_count++;
	if (g_ctrl->touch_count == 0)
		return;

	g_ctrl->ir_average += (value - g_ctrl->ir_average) / g_ctrl->touch_count;
}

static void _max30101_set_force_green(
	struct max30101_g_ctrl *g_ctrl, bool enable, uint8_t pa_value)
{
	g_ctrl->force_pa = enable;
	pa_init_green = pa_value;
}

static void _max30101_init_g_ctrl(struct max30101_g_ctrl *g_ctrl)
{
	//uint8_t nvdata[MAX30101_FINGER_FITTING_SIZE];
	//ret = get_nonvolatile(nvdata, MAX30101_APNV_FINGER_FITTING, MAX30101_FINGER_FITTING_SIZE);
	g_ctrl->touch_threshold = MAX30101_FINGER_DETECT_DEFAULT;
	// if (ret == MAX30101_FINGER_FITTING_SIZE) {
	// 	g_ctrl->touch_threshold = MAX30101_DATA_CONSTRUCTION(nvdata[0], nvdata[1], nvdata[2]);
	// } else {
	// 	g_ctrl->touch_threshold = MAX30101_FINGER_DETECT_DEFAULT;
	// 	MAX30101_LOG_E("%s: nonvolatile get error %d\n", __func__, ret);
	// }
	MAX30101_LOG_D("%s: value %d\n", __func__, g_ctrl->touch_threshold);

	_max30101_init_touch_count(g_ctrl);
	_max30101_set_force_green(g_ctrl, false, MAX30101_PA_GREEN_INIT_VALUE);
}

static void _max30101_ctrl_green_led(
	struct max30101_data *info, struct max30101_g_ctrl *g_ctrl)
{
	uint8_t green;
	bool lighting = g_ctrl->g_lighting;
	int rc = 0;
	int ir_average = g_ctrl->ir_average;

	do {
		if (g_ctrl->touch_count < MAX30101_TOUCH_AVERAGE_COUNT)
			break;

		g_ctrl->ir_average = 0;
		g_ctrl->touch_count = 0;

		if (g_ctrl->force_pa)
			break;

		g_ctrl->touch_threshold = finger_detect_ir_data;
		MAX30101_LOG_D("g_ctrl->touch_threshold = %d\n", g_ctrl->touch_threshold);
		if (!lighting && g_ctrl->touch_threshold >= ir_average) {
			/* OFF -> ON */
			green = pa_green;
			lighting = true;
		} else if (lighting && g_ctrl->touch_threshold < ir_average) {
			/* ON -> OFF */
			green = pa_init_green;
			lighting = false;
		} else {
			MAX30101_LOG_D("green LED not change %d %d 0x%x\n",
					lighting, g_ctrl->touch_threshold, ir_average);
			break;
		}
		MAX30101_LOG_D("green LED %d 0x%x %d 0x%x\n",
				lighting, green, g_ctrl->touch_threshold, ir_average);

		rc = _max30101_reg_write(info, MAX30101_REG_LED3_PA, &green, 1);
		if (rc) {
			MAX30101_LOG_E("LED3_PA write error %d\n", rc);
			break;
		}
		g_ctrl->g_lighting = lighting;

	} while (0);
}

/******************************************************************************/
/* Overflow control functions                                                 */
/******************************************************************************/
static void _max30101_init_ovf_ctrl(struct max30101_ovf_ctrl *ovf_ctrl)
{
	ovf_ctrl->read_cnt = 0;
	memset(ovf_ctrl->ovf_cnt, 0, sizeof(ovf_ctrl->ovf_cnt));
}

static bool _max30101_ovf_ctrl(
	struct max30101_ovf_ctrl *ovf_ctrl, uint8_t ovf_value)
{
	bool notify = false;
	unsigned int index = ovf_ctrl->read_cnt / MAX30101_OVF_ACCUM_INTERVAL;
	unsigned int rem = ovf_ctrl->read_cnt % MAX30101_OVF_ACCUM_INTERVAL;
	unsigned int i;
	uint8_t ovf_num = 0;
	uint8_t ovf = MAX30101_BIT_MASK(ovf_value, MAX30101_OVF_CNT_MASK);

	if (unlikely(index >= MAX30101_OVF_ACCUM_NUM))
		index = MAX30101_OVF_ACCUM_NUM - 1;

	if (rem == 0)
		ovf_ctrl->ovf_cnt[index] = 0;

	if (ovf != 0) {
		MAX30101_LOG_S("FIFO overflow! ovf_cnt:0x%x\n", ovf_value);
		ovf_ctrl->ovf_cnt[index] += ovf;

		for (i = 0; i < MAX30101_OVF_ACCUM_NUM; i++) {
			if (ovf_ctrl->ovf_cnt[i] != 0)
				ovf_num++;
		}

		MAX30101_LOG_E("FIFO overflow info:%d(%d)[%d][%d][%d][%d][%d][%d]\n",
						ovf_ctrl->read_cnt, index,
						ovf_ctrl->ovf_cnt[0], ovf_ctrl->ovf_cnt[1],
						ovf_ctrl->ovf_cnt[2], ovf_ctrl->ovf_cnt[3],
						ovf_ctrl->ovf_cnt[4], ovf_ctrl->ovf_cnt[5]);

		if ((ovf_ctrl->ovf_cnt[index] > ovf_threshold1) ||
			(ovf_num > ovf_threshold2)) {
			MAX30101_LOG_E("Notify FIFO overflow\n");
			notify = true;
		}
	}

	ovf_ctrl->read_cnt += MAX30101_READ_SAMPLE_NUM;
	if (ovf_ctrl->read_cnt >= MAX30101_OVF_ACCUM_TOTAL)
		ovf_ctrl->read_cnt = 0;

	if (notify)
		_max30101_init_ovf_ctrl(ovf_ctrl);

	return notify;
}

/******************************************************************************/
/* FIFO data reading functions                                                */
/******************************************************************************/
static void _max30101_processing_fifo_data(
	struct max30101_data *info,
	uint8_t *data_i,
	uint8_t *data_o,
	unsigned int size)
{
	unsigned int i = 0;

	for (i = 0; i < size; i += (MAX30101_1SAMPLE_SIZE * MAX30101_LED_NUM)) {
		/* bit flip & mask only valid data */
		MAX30101_RED_1(data_o + i) = (~MAX30101_RED_1(data_i + i) & 0x03);
		MAX30101_RED_2(data_o + i) = ~MAX30101_RED_2(data_i + i);
		MAX30101_RED_3(data_o + i) = ~MAX30101_RED_3(data_i + i);

		MAX30101_IR_1(data_o + i) = (~MAX30101_IR_1(data_i + i) & 0x03);
		MAX30101_IR_2(data_o + i) = ~MAX30101_IR_2(data_i + i);
		MAX30101_IR_3(data_o + i) = ~MAX30101_IR_3(data_i + i);

		MAX30101_GREEN_1(data_o + i) = (~MAX30101_GREEN_1(data_i + i) & 0x03);
		MAX30101_GREEN_2(data_o + i) = ~MAX30101_GREEN_2(data_i + i);
		MAX30101_GREEN_3(data_o + i) = ~MAX30101_GREEN_3(data_i + i);

		/* Green LED control */
		_max30101_calc_ir_average(&info->g_ctrl,
								MAX30101_DATA_CONSTRUCTION(
									MAX30101_IR_1(data_o + i),
									MAX30101_IR_2(data_o + i),
									MAX30101_IR_3(data_o + i)));
#ifdef MAX30101_DUMP_FIFO
		MAX30101_LOG_D("0x%02x%02x%02x 0x%02x%02x%02x 0x%02x%02x%02x\n",
			MAX30101_RED_1(data_o + i), MAX30101_RED_2(data_o + i), MAX30101_RED_3(data_o + i),
			MAX30101_IR_1(data_o + i), MAX30101_IR_2(data_o + i), MAX30101_IR_3(data_o + i),
			MAX30101_GREEN_1(data_o + i), MAX30101_GREEN_2(data_o + i), MAX30101_GREEN_3(data_o + i));
#endif /* MAX30101_DUMP_FIFO */
	}
}

static int _max30101_read_fifo_data(struct max30101_data *info)
{
	#define MAX30101_READ_FIFO_LOOP_MAX	(10)

	#define MAX30101_BURST_READ1_ADDR	MAX30101_REG_INT_STATUS_1
	enum {
		MAX30101_INT_STATUS_1 = 0,
		MAX30101_INT_STATUS_2,
		MAX30101_BURST_READ1_SIZE
	};

	#define MAX30101_BURST_READ2_ADDR	MAX30101_REG_FIFO_WR_PTR
	enum {
		MAX30101_FIFO_WR_PTR = 0,
		MAX30101_OVF_CNT,
		MAX30101_FIFO_RD_PTR,
		MAX30101_BURST_READ2_SIZE
	};

	uint8_t reg1[MAX30101_BURST_READ1_SIZE] = {0};
	uint8_t reg2[MAX30101_BURST_READ2_SIZE] = {0};
	uint8_t available = 0, i;
	struct max30101_fifo_data *free_fifo = NULL;
	int rc = 0, rc2 = 0;

	/* Prevention of endless loop */
	for (i = 0; i < MAX30101_READ_FIFO_LOOP_MAX; i++) {

		rc = _max30101_reg_read(info, MAX30101_BURST_READ2_ADDR, reg2, MAX30101_BURST_READ2_SIZE);
		if (rc) {
			rc = -EIO;
			break;
		}

		/* Calculating the number of samples in FIFO */
		MAX30101_CALC_SAMPLE_NUM(
						reg2[MAX30101_FIFO_WR_PTR],
						reg2[MAX30101_FIFO_RD_PTR],
						reg2[MAX30101_OVF_CNT],
						available);

		MAX30101_LOG_D("wr_ptr:0x%x ovf_cnt:0x%x rd_ptr:0x%x available:%d\n",
						reg2[MAX30101_FIFO_WR_PTR],
						reg2[MAX30101_OVF_CNT],
						reg2[MAX30101_FIFO_RD_PTR],
						available);

		if (available < MAX30101_READ_SAMPLE_NUM) {
			MAX30101_LOG_D("Data is not stored in the FIFO.\n");
			break;
		}

		free_fifo = _max30101_get_free_q(info);
		if (!free_fifo) {
			rc = -ENOMEM;
			MAX30101_LOG_E("%s:no fifo data\n", __func__);
			break;
		}

		/* Overflow control */
		if (_max30101_ovf_ctrl(&info->ovf_ctrl, reg2[MAX30101_OVF_CNT])) {
			free_fifo->status = LEDPULSE_STATUS_OVERFLOW;
			MAX30101_LOG_E("%s:Overflow control\n", __func__);
			_max30101_put_use_q(info, free_fifo);
			ledpulse_notify(info->ledpulse);
			break;
		}

		memset(info->i2c_buff_p, 0, MAX30101_FIFO_DATA_SIZE);

		/* Read FIFO DATA */
		rc = _max30101_reg_read(info, MAX30101_REG_FIFO_DATA_REG,
								info->i2c_buff_p, MAX30101_FIFO_DATA_SIZE);
		if (rc) {
			_max30101_put_free_q(info, free_fifo);
			rc = -EIO;
			break;
		}

		if (info->first_data) {
			_max30101_put_free_q(info, free_fifo);
			/* Since the first data can not have a stable value, */
			/* it is discarded. */
			MAX30101_LOG_D("discard the first data\n");
			info->first_data = false;
		} else {
			_max30101_processing_fifo_data(info, info->i2c_buff_p, free_fifo->data, sizeof(free_fifo->data));
			/* Green LED control */
			_max30101_ctrl_green_led(info, &info->g_ctrl);
			_max30101_put_use_q(info, free_fifo);
			ledpulse_notify(info->ledpulse);
		}
		if (ovf_test) {
			MAX30101_LOG_I("FIFO overflow test\n");
			msleep((MAX30101_FIFO_MAX_NUM - available + MAX30101_READ_SAMPLE_NUM) * 5);
			ovf_test = false;
		}
	}

	/* Interrupt clear */
	rc2 = _max30101_reg_read(info, MAX30101_BURST_READ1_ADDR, reg1, MAX30101_BURST_READ1_SIZE);
	if (rc2)
		rc = -EIO;
	else
		MAX30101_LOG_D("status1:0x%x status2:0x%x\n", reg1[MAX30101_INT_STATUS_1], reg1[MAX30101_INT_STATUS_2]);

	return rc;
}

/******************************************************************************/
/* Driver local functions                                                     */
/******************************************************************************/
#ifdef CONFIG_OF
static int _max30101_parse_dt(struct max30101_data *info)
{
	int ret = 0;
	struct device_node *np = info->dev->of_node;

	do {
		info->vdd_en = of_get_named_gpio(np, "max30101,vdd_en", 0);
		if (info->vdd_en < 0) {
			MAX30101_LOG_E("failed to get max30101_vdd_en\n");
			ret = -ENODEV;
			break;
		}
		info->led_boost_en = of_get_named_gpio(np, "max30101,led_boost_en", 0);
		if (info->led_boost_en < 0) {
			MAX30101_LOG_E("failed to get max30101,led_boost_en\n");
			ret = -ENODEV;
			break;
		}

		MAX30101_LOG_D("vdd_en:%d\n", info->vdd_en);
		MAX30101_LOG_D("led_boost_en:%d\n", info->led_boost_en);
	} while (0);

	return ret;
}
#endif /* CONFIG_OF */

static int max30101_init_power(struct max30101_data *info)
{
	int ret = 0;
	if (gpio_is_valid(info->vdd_en)) {
		ret = gpio_request(info->vdd_en, "max_vdd_gpio");
		if (ret < 0) {
			MAX30101_LOG_D("vdd_en request failed\n");
			return ret;
		}

		ret = gpio_direction_output(info->vdd_en, 0);
		if (ret < 0) {
			MAX30101_LOG_D("set_direction for vdd gpio failed\n");
        }
	}

	if (gpio_is_valid(info->led_boost_en)) {
		ret = gpio_request(info->led_boost_en, "max_led_boost_vdd");
		if (ret < 0) {
			MAX30101_LOG_D("vdd_en request failed\n");
			return ret;
		}

		ret = gpio_direction_output(info->led_boost_en, 0);
		if (ret < 0) {
			MAX30101_LOG_D("set_direction for vdd gpio failed\n");
        }
	}
	return ret;
}

static int max30101_vdd_enable(struct max30101_data *info, int flag)
{
	int ret = 0;
	ret = gpio_direction_output(info->vdd_en, flag);
	if (ret < 0) 
		MAX30101_LOG_D("set_direction for vdd gpio failed\n");
	return ret;
}

static int max30101_led_vdd_enable(struct max30101_data *info, int flag)
{
	int ret = 0;
	ret = gpio_direction_output(info->led_boost_en, flag);
	if (ret < 0) 
		MAX30101_LOG_D("set_direction for vdd gpio failed\n");
	return ret;
}


static int _max30101_ctrl_regulator(struct max30101_data *info, bool enable)
{
	int rc = 0;
	struct timespec64 now_ts = {0};
	struct timespec64 end_ts = {0};
	struct timespec64 wait_ts = {0};
	s64 wait_ns = 0;

	if (always_on)
		return rc;

	if (enable) {
		if (timespec64_valid(&info->lastpoff_ts)) {
			end_ts = info->lastpoff_ts;
			timespec64_add_ns(&end_ts, MAX30101_POWER_RST_INTERVAL_NS);

			ktime_get_real_ts64(&now_ts);

			if (timespec64_compare(&end_ts, &now_ts) > 0) {
				wait_ts = timespec64_sub(end_ts, now_ts);
				wait_ns = timespec64_to_ns(&wait_ts);

				if (wait_ns > MAX30101_POWER_RST_INTERVAL_NS)
					wait_ns = MAX30101_POWER_RST_INTERVAL_NS;

				MAX30101_LOG_I("power off wait:%lld(ns)\n", wait_ns);
				max30101_ndelay((unsigned long)wait_ns);
				MAX30101_LOG_D("power off wait end\n");
			} else {
				MAX30101_LOG_D("power off no wait\n");
			}
		} else {
			MAX30101_LOG_D("first power on\n");
		}

		do {
			max30101_vdd_enable(info, enable);

			max30101_mdelay(MAX30101_POWER_INT_WAIT_MS);

			max30101_led_vdd_enable(info, enable);

		} while (0);
	} else {

		max30101_led_vdd_enable(info,enable);
		max30101_vdd_enable(info, enable);
		ktime_get_real_ts64(&info->lastpoff_ts);
	}

	return rc;
}

static int _max30101_init_pinctl(struct max30101_data *info)
{
	int ret = 0;
	MAX30101_LOG_D("'%s' enter.", __func__);

	do {
		info->pinctrl = devm_pinctrl_get(info->dev);
		if (PTR_ERR(info->pinctrl) == -EPROBE_DEFER) {
			MAX30101_LOG_W("pinctrl not ready\n");
			ret = -EPROBE_DEFER;
			break;
		}
		if (IS_ERR(info->pinctrl)) {
			MAX30101_LOG_E("Target does not use pinctrl\n");
			ret = -ENODEV;
			break;
		}

		info->active_state = pinctrl_lookup_state(info->pinctrl, "ledpulse_active");
		if (IS_ERR(info->active_state)) {
			MAX30101_LOG_E("no ledpulse_active pinctrl state\n");
			ret = -ENODEV;
			break;
		}

		ret = pinctrl_select_state(info->pinctrl, info->active_state);
		if (ret) {
			MAX30101_LOG_E("pinctrl_select_state error\n");
			ret = -ENODEV;
			break;
		}
	} while (0);

	return ret;
}

static int _max30101_init_irq(struct max30101_data *info)
{
	int ret = 0;
	struct device_node *np = info->dev->of_node;

	do {
		MAX30101_LOG_D("'%s' enter.", __func__);
		if (!np) {
			MAX30101_LOG_E("device node is null");
			ret = -ENODEV;
			break;
		}

		info->irq_gpio = of_get_named_gpio(np, "max30101,irq_gpio", 0);
		if (!gpio_is_valid(info->irq_gpio)) {
			MAX30101_LOG_E("failed to get irq_gpio\n");
			gpio_free(info->irq_gpio);
			ret = -ENODEV;
			break;
		}
		ret = gpio_request(info->irq_gpio, "max30101,irq_gpio");
		if (ret < 0) {
			MAX30101_LOG_E("irq gpio request failed");
			gpio_free(info->irq_gpio);
			ret = -ENODEV;
			break;
		}

		ret = gpio_direction_input(info->irq_gpio);
		if (ret < 0) {
			MAX30101_LOG_E("set_direction for irq gpio failed");
			gpio_free(info->irq_gpio);
			ret = -ENODEV;
			break;
		}

		info->irq_no = gpio_to_irq(info->irq_gpio);
		MAX30101_LOG_E("set_direction for irq gpio no = %d\n",info->irq_no);

		/* Interrupts are not enabled here. */
		/* Enable interrupt at measurement start. */
		irq_set_status_flags(info->irq_no, IRQ_NOAUTOEN);

		ret = request_threaded_irq(info->irq_no,
									NULL, max30101_irq_thread,
									(IRQF_TRIGGER_FALLING | IRQF_ONESHOT),
									"max30101_int",
									info);
		if (ret) {
			MAX30101_LOG_E("could not request irq ret:%d irq no:%d\n",
							ret, info->irq_no);
			info->irq_no = -1;
			ret = -ENODEV;
			break;
		}
		MAX30101_LOG_D("irq_no:%d\n", info->irq_no);

	} while (0);

	return ret;
}

static void _max30101_release(struct max30101_data *info)
{
	MAX30101_LOG_D("%s\n", __func__);

	if (info != NULL) {
		if (info->irq_no != -1) {
			free_irq(info->irq_no, info);
			info->irq_no = -1;
		}

		if (info->vdd_en != -1) {
			gpio_free(info->vdd_en);
			info->vdd_en = -1;
		}

		if (info->led_boost_en != -1) {
			gpio_free(info->led_boost_en);
			info->led_boost_en = -1;
		}

		if (!IS_ERR_OR_NULL(info->ledpulse)) {
			ledpulse_deregister(info->ledpulse);
			info->ledpulse = NULL;
		}

		if (info->i2c_buff_p) {
			// dma_free_coherent(info->dev, MAX30101_FIFO_DATA_SIZE, info->i2c_buff_p, info->paddr);
			kfree(info->i2c_buff_p);
			info->i2c_buff_p = NULL;
		}
		/* max30101_data will be freed automatically */
	}
}

/******************************************************************************/
/* Driver functions                                                           */
/******************************************************************************/
static irqreturn_t max30101_irq_thread(int irq, void *data)
{
	struct max30101_data *info = (struct max30101_data *)data;
	int rc = 0;

	if (info == NULL) {
		MAX30101_LOG_E("%s max30101_data NULL\n", __func__);
		return IRQ_HANDLED;
	}

	MAX30101_LOG_D("%s s\n", __func__);
	mutex_lock(&info->state_mutex);

	if (info->state == MAX30101_STATE_MEASURING) {
		rc = _max30101_read_fifo_data(info);
		if (rc)
			MAX30101_LOG_E("faild to read fifo\n");
	} else {
		/* Even if an interrupt occurs in a */
		/* state other than during measurement, it does nothing. */
		MAX30101_LOG_W("%s STATE:%d\n", __func__, info->state);
	}

	mutex_unlock(&info->state_mutex);
	MAX30101_LOG_D("%s e\n", __func__);

	return IRQ_HANDLED;
}

static int max30101_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int ret = 0;
	struct device *dev = &client->dev;
	struct max30101_data *info = NULL;

	MAX30101_LOG_D("%s\n", __func__);
	do {
		if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
			MAX30101_LOG_E("need I2C_FUNC_I2C\n");
			ret = -ENODEV;
			break;
		}

		info = devm_kzalloc(dev, sizeof(*info), GFP_KERNEL);
		if (!info) {
			MAX30101_LOG_E("failed to allocate memory\n");
			ret = -ENOMEM;
			break;
		}

		info->dev = dev;
		info->irq_no = -1;
		// info->vdd_reg = ERR_PTR(-1);
		// info->vled_reg = ERR_PTR(-1);

		// arch_setup_dma_ops(dev, 0, 0, NULL, true);
		// info->i2c_buff_p = dma_alloc_coherent(dev, MAX30101_FIFO_DATA_SIZE, &info->paddr, GFP_KERNEL);
		info->i2c_buff_p = kmalloc(MAX30101_FIFO_DATA_SIZE, GFP_KERNEL);
		if (!info->i2c_buff_p) {
			MAX30101_LOG_E("failed to allocate buff memory\n");
			ret = -ENOMEM;
			break;
		}
		LEDPULSE_LOG_I("buff %p\n", info->i2c_buff_p);

		mutex_init(&info->state_mutex);
		spin_lock_init(&info->q_lock);

		_max30101_init_q(info);

#if 1
#ifdef CONFIG_OF
		ret = _max30101_parse_dt(info);
		if (ret)
			break;
#endif /* CONFIG_OF */
#endif

		 ret = max30101_init_power(info);
		 if (ret)
		 	break;

		ret = _max30101_init_pinctl(info);
		if (ret)
			break;

		ret = _max30101_init_irq(info);
		if (ret)
			break;

		max30101_ops.device = info;
		info->ledpulse = ledpulse_register(&max30101_ops);
		if (IS_ERR_OR_NULL(info->ledpulse)) {
			MAX30101_LOG_E("faild to ledpulse_register\n");
			ret = -ENODEV;
			break;
		}

		get_hardware_info_data(HWID_PPG_SENSOR, "MAX30101r");
		MAX30101_LOG_E("HWID_PPG_SENSOR = %d \n", HWID_PPG_SENSOR);

		/* Green LED control */
		_max30101_init_g_ctrl(&info->g_ctrl);

		i2c_set_clientdata(client, info);


	} while (0);

	if (ret)
		_max30101_release(info);

	return ret;
}

static void max30101_shutdown(struct i2c_client *client)
{
	struct max30101_data *info = i2c_get_clientdata(client);

	LEDPULSE_LOG_I("%s\n", __func__);
	_max30101_release(info);
}

static int max30101_remove(struct i2c_client *client)
{
	LEDPULSE_LOG_I("%s\n", __func__);
	max30101_shutdown(client);
	return 0;
}

static const struct i2c_device_id max30101_id[] = {
	{"max30101-i2c", 0},
	{{0}, 0}
};
MODULE_DEVICE_TABLE(i2c, max30101_id);

static struct i2c_driver max30101_driver = {
	.id_table = max30101_id,
	.probe = max30101_probe,
	.remove = max30101_remove,
	.shutdown = max30101_shutdown,
	.driver = {
		.owner = THIS_MODULE,
		.name = "max30101-i2c",
	},
};

static int __init max30101_init(void)
{
	LEDPULSE_LOG_I("%s\n", __func__);
	return i2c_add_driver(&max30101_driver);
}
module_init(max30101_init);

static void __exit max30101_exit(void)
{
	i2c_del_driver(&max30101_driver);
}
module_exit(max30101_exit);

MODULE_AUTHOR("FUJITSU CONNECTED TECHNOLOGIES LIMITED");
MODULE_DESCRIPTION("max30101 driver");
MODULE_LICENSE("GPL v2");
