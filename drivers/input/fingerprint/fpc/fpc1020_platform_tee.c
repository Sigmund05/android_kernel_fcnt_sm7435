/*
 * FPC1020 Fingerprint sensor device driver
 *
 * This driver will control the platform resources that the FPC fingerprint
 * sensor needs to operate. The major things are probing the sensor to check
 * that it is actually connected and let the Kernel know this and with that also
 * enabling and disabling of regulators, controlling GPIOs such as sensor reset
 * line, sensor IRQ line.
 *
 * The driver will expose most of its available functionality in sysfs which
 * enables dynamic control of these features from eg. a user space process.
 *
 * The sensor's IRQ events will be pushed to Kernel's event handling system and
 * are exposed in the drivers event node.
 *
 * This driver will NOT send any commands to the sensor it only controls the
 * electrical parts.
 *
 *
 * Copyright (c) 2015 Fingerprint Cards AB <tech@fingerprints.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License Version 2
 * as published by the Free Software Foundation.
 */

#include <linux/atomic.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>
#include <linux/pinctrl/consumer.h>
#include <linux/hardware_info.h>

#define FPC_TTW_HOLD_TIME 2000
#define RESET_LOW_SLEEP_MIN_US 5000
#define RESET_LOW_SLEEP_MAX_US (RESET_LOW_SLEEP_MIN_US + 100)
#define RESET_HIGH_SLEEP1_MIN_US 100
#define RESET_HIGH_SLEEP1_MAX_US (RESET_HIGH_SLEEP1_MIN_US + 100)
#define RESET_HIGH_SLEEP2_MIN_US 5000
#define RESET_HIGH_SLEEP2_MAX_US (RESET_HIGH_SLEEP2_MIN_US + 100)
#define PWR_ON_SLEEP_MIN_US 100
#define PWR_ON_SLEEP_MAX_US (PWR_ON_SLEEP_MIN_US + 900)
#define PWR_OFF_SLEEP_MIN_US 1000
#define PWR_OFF_SLEEP_MAX_US (PWR_OFF_SLEEP_MIN_US + 100)
#define NUM_PARAMS_REG_ENABLE_SET 2
#define MAX_RESET_NUM 3
#define RELEASE_WAKELOCK_W_V "release_wakelock_with_verification"
#define RELEASE_WAKELOCK "release_wakelock"
#define START_IRQS_RECEIVED_CNT "start_irqs_received_counter"

#define CONFIG_FPC_COMPAT 1
extern void get_hardware_info_data(enum hardware_id id, const void *data);

static const char * const pctl_names[] = {
	"fpc1020_reset_reset",
	"fpc1020_reset_active",
	"fpc1020_irq_active",
	"fpc1020_irq_suspend",
	"fpc1020_cs_low",
};

struct fpc1020_data {
	struct device *dev;

	struct pinctrl *fingerprint_pinctrl;
	struct pinctrl_state *pinctrl_state[ARRAY_SIZE(pctl_names)];

	struct wakeup_source *ttw_wl;
	int irq_gpio;
	int rst_gpio;
	int pwr_gpio;

	int nbr_irqs_received;
	int nbr_irqs_received_counter_start;

	struct mutex lock; /* To set/get exported values in sysfs */
	bool prepared;
#ifdef CONFIG_FPC_COMPAT
	bool compatible_enabled;
#endif
	atomic_t wakeup_enabled; /* Used both in ISR and non-ISR */
};
static int reset_num = 0;
static irqreturn_t fpc1020_irq_handler(int irq, void *handle);
static int fpc1020_request_named_gpio(struct fpc1020_data *fpc1020,
		const char *label, int *gpio);
static int hw_reset(struct  fpc1020_data *fpc1020);

static int fpc_power_setup(struct fpc1020_data *fpc1020, bool enable)
{
	int rc = 0;
    struct device *dev = fpc1020->dev;
	dev_info(dev, "enter%d----\n", rc);
	if (gpio_is_valid(fpc1020->pwr_gpio)) {
		if(enable)
		{
			rc = gpio_direction_output(fpc1020->pwr_gpio, 1);
			dev_info(dev, "----fpc pwr on result: %d----\n", rc);
			msleep(10);

		} else
		{
			rc = gpio_direction_output(fpc1020->pwr_gpio, 0);
			dev_info(dev, "---- power off result: %d----\n", rc);
		}

	} else {
		dev_info(dev, "%s: fpc pwr gpio_is_invalid\n", __func__);
		return -EINVAL;
	}

	return rc;
}


/**
 * sysfs node for controlling clocks.
 *
 * This is disabled in platform variant of this driver but kept for
 * backwards compatibility. Only prints a debug print that it is
 * disabled.
 */
static ssize_t clk_enable_set(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	dev_dbg(dev,
		"clk_enable sysfs node not enabled in platform driver\n");

	return count;
}
static DEVICE_ATTR(clk_enable, S_IWUSR, NULL, clk_enable_set);

/**
 * Will try to select the set of pins (GPIOS) defined in a pin control node of
 * the device tree named @p name.
 *
 * The node can contain several eg. GPIOs that is controlled when selecting it.
 * The node may activate or deactivate the pins it contains, the action is
 * defined in the device tree node itself and not here. The states used
 * internally is fetched at probe time.
 *
 * @see pctl_names
 * @see fpc1020_probe
 */
static int select_pin_ctl(struct fpc1020_data *fpc1020, const char *name)
{
	size_t i;
	int rc;
	struct device *dev = fpc1020->dev;

	for (i = 0; i < ARRAY_SIZE(fpc1020->pinctrl_state); i++) {
		const char *n = pctl_names[i];

		if (!strncmp(n, name, strlen(n))) {
			rc = pinctrl_select_state(fpc1020->fingerprint_pinctrl,
					fpc1020->pinctrl_state[i]);
			if (rc)
				dev_err(dev, "cannot select '%s'\n", name);
			else
				dev_dbg(dev, "Selected '%s'\n", name);
			goto exit;
		}
	}

	rc = -EINVAL;
	dev_err(dev, "%s:'%s' not found\n", __func__, name);

exit:
	return rc;
}

static ssize_t pinctl_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct fpc1020_data *fpc1020 = dev_get_drvdata(dev);
	int rc;

	mutex_lock(&fpc1020->lock);
	rc = select_pin_ctl(fpc1020, buf);
	mutex_unlock(&fpc1020->lock);

	return rc ? rc : count;
}
static DEVICE_ATTR(pinctl_set, S_IWUSR, NULL, pinctl_set);

static ssize_t regulator_enable_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct fpc1020_data *fpc1020 = dev_get_drvdata(dev);
	char op;
	char name[16];
	int rc;
	bool enable;

	if (NUM_PARAMS_REG_ENABLE_SET != sscanf(buf, "%15[^,],%c", name, &op))
		return -EINVAL;
	if (op == 'e')
		enable = true;
	else if (op == 'd')
		enable = false;
	else
		return -EINVAL;

	mutex_lock(&fpc1020->lock);
	rc = fpc_power_setup(fpc1020, enable);
	mutex_unlock(&fpc1020->lock);

	return rc ? rc : count;
}
static DEVICE_ATTR(regulator_enable, S_IWUSR, NULL, regulator_enable_set);

static int hw_reset(struct fpc1020_data *fpc1020)
{
	int irq_gpio;
	int rc;
	struct device *dev = fpc1020->dev;

	dev_info(dev, "%s --->\n", __func__);
	irq_gpio = gpio_get_value(fpc1020->irq_gpio);
	dev_info(dev, "IRQ before reset %d\n", irq_gpio);
	rc = select_pin_ctl(fpc1020, "fpc1020_reset_active");

	if (rc)
		goto exit;
	usleep_range(RESET_HIGH_SLEEP1_MIN_US, RESET_HIGH_SLEEP1_MAX_US);

	rc = select_pin_ctl(fpc1020, "fpc1020_reset_reset");
	if (rc)
		goto exit;
	usleep_range(RESET_LOW_SLEEP_MIN_US, RESET_LOW_SLEEP_MAX_US);

	rc = select_pin_ctl(fpc1020, "fpc1020_reset_active");
	if (rc)
		goto exit;
	usleep_range(RESET_HIGH_SLEEP2_MIN_US, RESET_HIGH_SLEEP2_MAX_US);

	irq_gpio = gpio_get_value(fpc1020->irq_gpio);
	dev_info(dev, "IRQ after reset %d, reset_num:%d\n", irq_gpio, reset_num);

exit:
	dev_info(dev, "%s <---\n", __func__);
	return rc;
}

static int hw_power_reset(struct fpc1020_data *fpc1020, bool enable)
{
	int rc = 0;
	struct device *dev = fpc1020->dev;
	dev_info(dev, "%s --->\n", __func__);
	rc = fpc_power_setup(fpc1020, false);
	if(rc){
		dev_info(dev, "%s fpc_power_setup false failure.\n", __func__);
		goto exit;
	}
	usleep_range(PWR_OFF_SLEEP_MIN_US, PWR_OFF_SLEEP_MAX_US);
	if(enable){
		rc = fpc_power_setup(fpc1020, true);
		if(rc){
			dev_info(dev, "%s fpc_power_setup true failure.\n", __func__);
			goto exit;
		}
		usleep_range(PWR_OFF_SLEEP_MIN_US, PWR_OFF_SLEEP_MAX_US);
		rc = hw_reset(fpc1020);
		if(rc){
			dev_info(dev, "%s hw_reset failure.\n", __func__);
		}
	}
	else {
		gpio_direction_output(fpc1020->rst_gpio,0);
		rc = select_pin_ctl(fpc1020, "fpc1020_cs_low");
		if (rc){
			dev_info(dev, "%s fpc_power_setup down failure.\n", __func__);
		}
	}
exit:
	dev_info(dev, "%s <---\n", __func__);
	return rc;
}

static ssize_t hw_reset_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int rc;
	struct fpc1020_data *fpc1020 = dev_get_drvdata(dev);

	if (!strncmp(buf, "reset", strlen("reset"))) {
		mutex_lock(&fpc1020->lock);
		if(reset_num > MAX_RESET_NUM){
			dev_info(dev, "%s hardware abnormal, shut down.\n", __func__);
			rc = hw_power_reset(fpc1020, false);
			reset_num = 0;
			goto exit;
		}
		reset_num++;
		rc = hw_power_reset(fpc1020, true);
		mutex_unlock(&fpc1020->lock);
	}  else if (!strncmp(buf, "clear", strlen("clear"))) {
        reset_num = 0;
        return count;
    }
    else {
		return -EINVAL;
	}

exit:
	return rc ? rc : count;
}
static DEVICE_ATTR(hw_reset, S_IWUSR, NULL, hw_reset_set);

/**
 * Will setup GPIOs, and regulators to correctly initialize the touch sensor to
 * be ready for work.
 *
 * In the correct order according to the sensor spec this function will
 * enable/disable regulators, and reset line, all to set the sensor in a
 * correct power on or off state "electrical" wise.
 *
 * @see  device_prepare_set
 * @note This function will not send any commands to the sensor it will only
 *       control it "electrically".
 */
static int device_prepare(struct fpc1020_data *fpc1020, bool enable)
{
	int rc;
	struct device *dev = fpc1020->dev;
	dev_info(dev, "-->%s, enable=%d, fpc1020->prepare=%d\n", __func__, enable, fpc1020->prepared);
	mutex_lock(&fpc1020->lock);
	if (enable && !fpc1020->prepared) {
		fpc1020->prepared = true;
		select_pin_ctl(fpc1020, "fpc1020_reset_reset");
	    rc = fpc_power_setup(fpc1020, true);

		if (rc){
            dev_info(dev, "%s, fpc_power_setup failed!\n", __func__);
            goto exit;
        }

		usleep_range(PWR_ON_SLEEP_MIN_US, PWR_ON_SLEEP_MAX_US);

		/* As we can't control chip select here the other part of the
		 * sensor driver eg. the TEE driver needs to do a _SOFT_ reset
		 * on the sensor after power up to be sure that the sensor is
		 * in a good state after power up. Okeyed by ASIC. */

		(void)select_pin_ctl(fpc1020, "fpc1020_reset_active");
	} else if (!enable && fpc1020->prepared) {
		rc = 0;
		(void)select_pin_ctl(fpc1020, "fpc1020_reset_reset");

		usleep_range(PWR_ON_SLEEP_MIN_US, PWR_ON_SLEEP_MAX_US);

	    (void)fpc_power_setup(fpc1020, false);
exit:
		fpc1020->prepared = false;
	} else {
		rc = 0;
	}
	mutex_unlock(&fpc1020->lock);
	dev_info(dev, "<--%s", __func__);
	return rc;
}

/**
 * sysfs node to enable/disable (power up/power down) the touch sensor
 *
 * @see device_prepare
 */
static ssize_t device_prepare_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int rc;
	struct fpc1020_data *fpc1020 = dev_get_drvdata(dev);

	if (!strncmp(buf, "enable", strlen("enable")))
		rc = device_prepare(fpc1020, true);
	else if (!strncmp(buf, "disable", strlen("disable")))
		rc = device_prepare(fpc1020, false);
	else
		return -EINVAL;

	return rc ? rc : count;
}
static DEVICE_ATTR(device_prepare, S_IWUSR, NULL, device_prepare_set);

/**
 * sysfs node for controlling whether the driver is allowed
 * to wake up the platform on interrupt.
 */
static ssize_t wakeup_enable_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct fpc1020_data *fpc1020 = dev_get_drvdata(dev);
	ssize_t ret = count;

	mutex_lock(&fpc1020->lock);
	if (!strncmp(buf, "enable", strlen("enable")))
		atomic_set(&fpc1020->wakeup_enabled, 1);
	else if (!strncmp(buf, "disable", strlen("disable")))
		atomic_set(&fpc1020->wakeup_enabled, 0);
	else
		ret = -EINVAL;
	mutex_unlock(&fpc1020->lock);

	return ret;
}
static ssize_t wakeup_enable_get(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	struct fpc1020_data *fpc1020 = dev_get_drvdata(dev);
	
	if (atomic_read(&fpc1020->wakeup_enabled))
		return snprintf(buf, PAGE_SIZE, "%s", "true");
	else 
		return snprintf(buf, PAGE_SIZE, "%s", "false");

}

static DEVICE_ATTR(wakeup_enable, S_IRUSR | S_IWUSR, wakeup_enable_get, wakeup_enable_set);

/**
 * sysfs node for controlling the wakelock.
 */
static ssize_t handle_wakelock_cmd(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct  fpc1020_data *fpc1020 = dev_get_drvdata(dev);
	ssize_t ret = count;

	mutex_lock(&fpc1020->lock);
	if (!strncmp(buf, RELEASE_WAKELOCK_W_V,
		min(count, strlen(RELEASE_WAKELOCK_W_V)))) {
		if (fpc1020->nbr_irqs_received_counter_start ==
				fpc1020->nbr_irqs_received) {
			__pm_relax(fpc1020->ttw_wl);
		} else {
			dev_dbg(dev, "Ignore releasing of wakelock %d != %d",
				fpc1020->nbr_irqs_received_counter_start,
				fpc1020->nbr_irqs_received);
		}
	} else if (!strncmp(buf, RELEASE_WAKELOCK, min(count,
				strlen(RELEASE_WAKELOCK)))) {
		__pm_relax(fpc1020->ttw_wl);
	} else if (!strncmp(buf, START_IRQS_RECEIVED_CNT,
			min(count, strlen(START_IRQS_RECEIVED_CNT)))) {
		fpc1020->nbr_irqs_received_counter_start =
		fpc1020->nbr_irqs_received;
	} else
		ret = -EINVAL;
	mutex_unlock(&fpc1020->lock);

	return ret;
}
static DEVICE_ATTR(handle_wakelock, S_IWUSR, NULL, handle_wakelock_cmd);

/**
 * sysf node to check the interrupt status of the sensor, the interrupt
 * handler should perform sysf_notify to allow userland to poll the node.
 */
static ssize_t irq_get(struct device *dev,
	struct device_attribute *attr,
	char *buf)
{
	struct fpc1020_data *fpc1020 = dev_get_drvdata(dev);
	int irq = gpio_get_value(fpc1020->irq_gpio);

	return scnprintf(buf, PAGE_SIZE, "%i\n", irq);
}

/**
 * writing to the irq node will just drop a printk message
 * and return success, used for latency measurement.
 */
static ssize_t irq_ack(struct device *dev,
	struct device_attribute *attr,
	const char *buf, size_t count)
{
	struct fpc1020_data *fpc1020 = dev_get_drvdata(dev);

	dev_dbg(fpc1020->dev, "%s\n", __func__);

	return count;
}
static DEVICE_ATTR(irq, S_IRUSR | S_IWUSR, irq_get, irq_ack);

#ifdef CONFIG_FPC_COMPAT
static ssize_t compatible_all_set(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int rc;
	int i;
	int irqf;
	struct  fpc1020_data *fpc1020 = dev_get_drvdata(dev);

	dev_err(dev, "compatible all enter %d\n", fpc1020->compatible_enabled);
	if(!strncmp(buf, "enable", strlen("enable")) && fpc1020->compatible_enabled != 1){
		get_hardware_info_data(HWID_FINGERPRINT, "fpc1020");
		rc = fpc1020_request_named_gpio(fpc1020, "fpc,gpio_irq",
			&fpc1020->irq_gpio);
		if (rc)
			goto exit;

		rc = fpc1020_request_named_gpio(fpc1020, "fpc,gpio_rst",
			&fpc1020->rst_gpio);
		dev_err(dev, "fpc request reset result = %d\n",rc);
		if (rc)
			goto exit;

		rc = fpc1020_request_named_gpio(fpc1020, "fpc,gpio_pwr",
			&fpc1020->pwr_gpio);
		dev_err(dev, "fpc request pwr_gpio result = %d\n",rc);
		if (rc)
			goto exit;

		fpc1020->fingerprint_pinctrl = devm_pinctrl_get(dev);
		if (IS_ERR(fpc1020->fingerprint_pinctrl)) {
			if (PTR_ERR(fpc1020->fingerprint_pinctrl) == -EPROBE_DEFER) {
				dev_info(dev, "pinctrl not ready\n");
				rc = -EPROBE_DEFER;
				goto exit;
			}
			dev_err(dev, "Target does not use pinctrl\n");
			fpc1020->fingerprint_pinctrl = NULL;
			rc = -EINVAL;
			goto exit;
		}

		for (i = 0; i < ARRAY_SIZE(fpc1020->pinctrl_state); i++) {
			const char *n = pctl_names[i];
			struct pinctrl_state *state =
				pinctrl_lookup_state(fpc1020->fingerprint_pinctrl, n);
			if (IS_ERR(state)) {
				dev_err(dev, "cannot find '%s'\n", n);
				rc = -EINVAL;
				goto exit;
			}
			dev_info(dev, "found pin control %s\n", n);
			fpc1020->pinctrl_state[i] = state;
		}
		rc = select_pin_ctl(fpc1020, "fpc1020_reset_reset");
		if (rc)
			goto exit;
		rc = select_pin_ctl(fpc1020, "fpc1020_irq_active");
		if (rc)
			goto exit;

		atomic_set(&fpc1020->wakeup_enabled, 1);

		irqf = IRQF_TRIGGER_RISING | IRQF_ONESHOT;
		if (of_property_read_bool(dev->of_node, "fpc,enable-wakeup")) {
			device_init_wakeup(dev, 1);
			dev_err(dev, "fpc enable-wakeup done!\n");
		}

		rc = devm_request_threaded_irq(dev, gpio_to_irq(fpc1020->irq_gpio),
			NULL, fpc1020_irq_handler, irqf,
			dev_name(dev), fpc1020);
		if (rc) {
			dev_err(dev, "could not request irq %d\n",
				gpio_to_irq(fpc1020->irq_gpio));
		goto exit;
		}
		dev_err(dev, "requested irq %d\n", gpio_to_irq(fpc1020->irq_gpio));

		/* Request that the interrupt should be wakeable */
		enable_irq_wake(gpio_to_irq(fpc1020->irq_gpio));
		fpc1020->compatible_enabled = 1;
		if (of_property_read_bool(dev->of_node, "fpc,enable-on-boot")) {
			dev_err(dev, "Enabling hardware\n");
			(void)device_prepare(fpc1020, true);
			dev_err(dev, "Enabling hardware done.\n");
		}
		
		hw_reset(fpc1020);
	}else if(!strncmp(buf, "disable", strlen("disable")) && fpc1020->compatible_enabled != 0){
		if (gpio_is_valid(fpc1020->irq_gpio))
		{
			devm_gpio_free(dev, fpc1020->irq_gpio);
			dev_err(dev, "remove irq_gpio success\n");
		}
		if (gpio_is_valid(fpc1020->rst_gpio))
		{
			devm_gpio_free(dev, fpc1020->rst_gpio);
			dev_err(dev, "remove rst_gpio success\n");
		}
		if(gpio_is_valid(fpc1020->pwr_gpio))
		{
			devm_gpio_free(dev, fpc1020->pwr_gpio);
			dev_err(dev, "remove pwr_gpio success\n");
		}
		devm_free_irq(dev, gpio_to_irq(fpc1020->irq_gpio), fpc1020);
		fpc1020->compatible_enabled = 0;
		get_hardware_info_data(HWID_FINGERPRINT, "Not Found");
	}
	return count;
exit:
	return -EINVAL;
}
static DEVICE_ATTR(compatible_all, S_IWUSR, NULL, compatible_all_set);
#endif

static struct attribute *attributes[] = {
	&dev_attr_pinctl_set.attr,
	&dev_attr_device_prepare.attr,
	&dev_attr_regulator_enable.attr,
	&dev_attr_hw_reset.attr,
	&dev_attr_wakeup_enable.attr,
	&dev_attr_handle_wakelock.attr,
	&dev_attr_clk_enable.attr,
	&dev_attr_irq.attr,
#ifdef CONFIG_FPC_COMPAT
	&dev_attr_compatible_all.attr,
#endif
	NULL
};

static const struct attribute_group attribute_group = {
	.attrs = attributes,
};

static irqreturn_t fpc1020_irq_handler(int irq, void *handle)
{
	struct fpc1020_data *fpc1020 = handle;

	dev_dbg(fpc1020->dev, "%s\n", __func__);

	mutex_lock(&fpc1020->lock);
	if (atomic_read(&fpc1020->wakeup_enabled)) {
		fpc1020->nbr_irqs_received++;
		__pm_wakeup_event(fpc1020->ttw_wl,
					FPC_TTW_HOLD_TIME);
		dev_dbg(fpc1020->dev, "fpc wakeup.");
	}
	mutex_unlock(&fpc1020->lock);

	sysfs_notify(&fpc1020->dev->kobj, NULL, dev_attr_irq.attr.name);

	return IRQ_HANDLED;
}

static int fpc1020_request_named_gpio(struct fpc1020_data *fpc1020,
	const char *label, int *gpio)
{
	struct device *dev = fpc1020->dev;
	struct device_node *np = dev->of_node;
	int rc = of_get_named_gpio(np, label, 0);

	if (rc < 0) {
		dev_err(dev, "failed to get '%s'\n", label);
		return rc;
	}
	*gpio = rc;

	rc = devm_gpio_request(dev, *gpio, label);
	if (rc) {
		dev_err(dev, "failed to request %s - gpio %d\n", label, *gpio);
		return rc;
	}
	dev_dbg(dev, "%s %d\n", label, *gpio);

	return 0;
}

static int fpc1020_probe(struct platform_device *pdev)
{
	struct device *dev;
	struct device_node *np;
	struct fpc1020_data *fpc1020;
	int rc = 0;
	reset_num = 0;
#ifndef CONFIG_FPC_COMPAT
	size_t i;
	int irqf;
#endif
	dev = &pdev->dev;
	np = dev->of_node;
	fpc1020 = devm_kzalloc(dev, sizeof(*fpc1020), GFP_KERNEL);

	if (!fpc1020) {
		dev_err(dev,
			"failed to allocate memory for struct fpc1020_data\n");
		rc = -ENOMEM;
		goto exit;
	}

	fpc1020->dev = dev;
	platform_set_drvdata(pdev, fpc1020);

	if (!np) {
		dev_err(dev, "no of node found\n");
		rc = -EINVAL;
		goto exit;
	}
#ifndef CONFIG_FPC_COMPAT
	rc = fpc1020_request_named_gpio(fpc1020, "fpc,gpio_irq",
			&fpc1020->irq_gpio);
	if (rc)
		goto exit;
	rc = fpc1020_request_named_gpio(fpc1020, "fpc,gpio_rst",
			&fpc1020->rst_gpio);
	if (rc)
		goto exit;

	fpc1020->fingerprint_pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(fpc1020->fingerprint_pinctrl)) {
		if (PTR_ERR(fpc1020->fingerprint_pinctrl) == -EPROBE_DEFER) {
			dev_info(dev, "pinctrl not ready\n");
			rc = -EPROBE_DEFER;
			goto exit;
		}
		dev_err(dev, "Target does not use pinctrl\n");
		fpc1020->fingerprint_pinctrl = NULL;
		rc = -EINVAL;
		goto exit;
	}

	for (i = 0; i < ARRAY_SIZE(fpc1020->pinctrl_state); i++) {
		const char *n = pctl_names[i];
		struct pinctrl_state *state =
			pinctrl_lookup_state(fpc1020->fingerprint_pinctrl, n);
		if (IS_ERR(state)) {
			dev_err(dev, "cannot find '%s'\n", n);
			rc = -EINVAL;
			goto exit;
		}
		dev_info(dev, "found pin control %s\n", n);
		fpc1020->pinctrl_state[i] = state;
	}

	rc = select_pin_ctl(fpc1020, "fpc1020_reset_reset");
	if (rc)
		goto exit;
	rc = select_pin_ctl(fpc1020, "fpc1020_irq_active");
	if (rc)
		goto exit;

	atomic_set(&fpc1020->wakeup_enabled, 1);

	irqf = IRQF_TRIGGER_RISING | IRQF_ONESHOT;
	if (of_property_read_bool(dev->of_node, "fpc,enable-wakeup")) {
		device_init_wakeup(dev, 1);
	}

	mutex_init(&fpc1020->lock);
	rc = devm_request_threaded_irq(dev, gpio_to_irq(fpc1020->irq_gpio),
			NULL, fpc1020_irq_handler, irqf,
			dev_name(dev), fpc1020);
	if (rc) {
		dev_err(dev, "could not request irq %d\n",
				gpio_to_irq(fpc1020->irq_gpio));
		goto exit;
	}

	dev_dbg(dev, "requested irq %d\n", gpio_to_irq(fpc1020->irq_gpio));

	/* Request that the interrupt should be wakeable */
	enable_irq_wake(gpio_to_irq(fpc1020->irq_gpio));

	fpc1020->ttw_wl = wakeup_source_register("fpc_ttw_wl");
	if (!fpc1020->ttw_wl)
		return -ENOMEM;

	rc = sysfs_create_group(&dev->kobj, &attribute_group);
	if (rc) {
		dev_err(dev, "could not create sysfs\n");
		goto exit;
	}

	if (of_property_read_bool(dev->of_node, "fpc,enable-on-boot")) {
		dev_info(dev, "Enabling hardware\n");
		(void)device_prepare(fpc1020, true);
	}

	rc = hw_reset(fpc1020);
	if (rc) {
		dev_err(dev, "hardware reset failed\n");
		goto exit;
	}
#else
	mutex_init(&fpc1020->lock);

	fpc1020->ttw_wl = wakeup_source_register(dev,"fpc_ttw_wl");
	if (!fpc1020->ttw_wl)
		return -ENOMEM;

	rc = sysfs_create_group(&dev->kobj, &attribute_group);
	if (rc) {
		dev_err(dev, "could not create sysfs\n");
		goto exit;
	}
#endif
	dev_info(dev, "%s: ok\n", __func__);

exit:
	return rc;
}

static int fpc1020_remove(struct platform_device *pdev)
{
	struct fpc1020_data *fpc1020 = platform_get_drvdata(pdev);

	sysfs_remove_group(&pdev->dev.kobj, &attribute_group);
	mutex_destroy(&fpc1020->lock);
	wakeup_source_unregister(fpc1020->ttw_wl);
	fpc_power_setup(fpc1020, false);
	dev_info(&pdev->dev, "%s\n", __func__);

	return 0;
}
#ifdef USE_PM
static int fpc1020_suspend(struct device *dev)
{
	struct fpc1020_data *fpc1020 = dev_get_drvdata(dev);
	int ret = 0;
	dev_err(dev, "fpc suspend.\n");
	ret = select_pin_ctl(fpc1020, "fpc1020_irq_suspend");
	if(ret)
		dev_err(dev, "could not set irq suspend.\n");
	return ret;
}

static int fpc1020_resume(struct device *dev)
{
	struct fpc1020_data *fpc1020 = dev_get_drvdata(dev);
	int ret = 0;
	dev_err(dev, "fpc resume.\n");
	ret = select_pin_ctl(fpc1020, "fpc1020_irq_active");
	if(ret)
		dev_err(dev, "could not set irq resume.\n");

	return ret;
}

static const struct dev_pm_ops fpc1020_pm_ops = {
	.suspend = fpc1020_suspend,
	.resume = fpc1020_resume,
};
#endif
static struct of_device_id fpc1020_of_match[] = {
	{ .compatible = "fpc,fpc1020", },
	{}
};
MODULE_DEVICE_TABLE(of, fpc1020_of_match);

static struct platform_driver fpc1020_driver = {
	.driver = {
		.name	= "fpc1020",
		.owner	= THIS_MODULE,
		.of_match_table = fpc1020_of_match,
#ifdef USE_PM
		.pm = &fpc1020_pm_ops,
#endif
	},
	.probe	= fpc1020_probe,
	.remove	= fpc1020_remove,
};

static int __init fpc1020_init(void)
{
	int rc = platform_driver_register(&fpc1020_driver);

	if (!rc)
		pr_info("%s OK\n", __func__);
	else
		pr_err("%s %d\n", __func__, rc);

	return rc;
}

static void __exit fpc1020_exit(void)
{
	pr_info("%s\n", __func__);
	platform_driver_unregister(&fpc1020_driver);
}

module_init(fpc1020_init);
module_exit(fpc1020_exit);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Aleksej Makarov");
MODULE_AUTHOR("Henrik Tillman <henrik.tillman@fingerprints.com>");
MODULE_DESCRIPTION("FPC1020 Fingerprint sensor device driver.");
