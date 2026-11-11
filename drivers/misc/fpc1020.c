// SPDX-License-Identifier: GPL-2.0-only
// SPDX-FileCopyrightText: The LineageOS Project
/*
 * Fingerprint Cards FPC1020 fingerprint sensor driver
 *
 * The sensor is driven over SPI by the TEE, so this driver only handles its
 * power, reset and interrupt lines on behalf of the fingerprint HAL.
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/pm_wakeup.h>
#include <linux/property.h>
#include <linux/string.h>
#include <linux/sysfs.h>

#define FPC1020_POWER_ON_US		10000
#define FPC1020_POWER_SETTLE_US		100
#define FPC1020_POWER_CYCLE_US		1000
#define FPC1020_RESET_HIGH_US		100
#define FPC1020_RESET_LOW_US		5000
#define FPC1020_RESET_SETTLE_US		5000
#define FPC1020_DELAY_SLACK_US		100

#define FPC1020_WAKEUP_TIMEOUT_MS	2000
#define FPC1020_MAX_RESETS		4

struct fpc1020 {
	struct device *dev;
	struct gpio_desc *irq_gpio;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *power_gpio;
	int irq;
	bool wakeup_capable;
	bool wakeup_enabled;

	struct mutex lock; /* protects the fields below */
	bool enabled;
	bool powered;
	unsigned int reset_count;
};

/* Indexed by the requested state */
static const char * const fpc1020_states[] = { "disable", "enable" };

static void fpc1020_delay_us(unsigned int us)
{
	usleep_range(us, us + FPC1020_DELAY_SLACK_US);
}

static int fpc1020_set_reset(struct fpc1020 *fpc, bool asserted)
{
	return gpiod_direction_output(fpc->reset_gpio, asserted);
}

static int fpc1020_power_on(struct fpc1020 *fpc)
{
	int ret;

	ret = gpiod_direction_output(fpc->power_gpio, 1);
	if (ret)
		return ret;

	fpc->powered = true;
	fpc1020_delay_us(FPC1020_POWER_ON_US);

	return 0;
}

static int fpc1020_power_off(struct fpc1020 *fpc)
{
	int ret;

	ret = gpiod_direction_output(fpc->power_gpio, 0);
	if (ret)
		return ret;

	fpc->powered = false;

	return 0;
}

static int fpc1020_reset(struct fpc1020 *fpc)
{
	int ret;

	ret = fpc1020_set_reset(fpc, false);
	if (ret)
		return ret;

	fpc1020_delay_us(FPC1020_RESET_HIGH_US);

	ret = fpc1020_set_reset(fpc, true);
	if (ret)
		return ret;

	fpc1020_delay_us(FPC1020_RESET_LOW_US);

	ret = fpc1020_set_reset(fpc, false);
	if (ret)
		return ret;

	fpc1020_delay_us(FPC1020_RESET_SETTLE_US);

	return 0;
}

static int fpc1020_set_power(struct fpc1020 *fpc, bool on)
{
	int ret;

	if (fpc->powered == on)
		return 0;

	ret = fpc1020_set_reset(fpc, true);
	if (ret)
		return ret;

	if (!on) {
		fpc1020_delay_us(FPC1020_POWER_SETTLE_US);
		return fpc1020_power_off(fpc);
	}

	ret = fpc1020_power_on(fpc);
	if (ret)
		return ret;

	fpc1020_delay_us(FPC1020_POWER_SETTLE_US);

	/* The TEE soft resets the sensor once it is out of reset */
	return fpc1020_set_reset(fpc, false);
}

static int fpc1020_power_cycle(struct fpc1020 *fpc)
{
	int ret;

	ret = fpc1020_power_off(fpc);
	if (ret)
		return ret;

	fpc1020_delay_us(FPC1020_POWER_CYCLE_US);

	ret = fpc1020_power_on(fpc);
	if (ret)
		return ret;

	fpc1020_delay_us(FPC1020_POWER_CYCLE_US);

	return fpc1020_reset(fpc);
}

static int fpc1020_power_down(struct fpc1020 *fpc)
{
	int ret;

	ret = fpc1020_power_off(fpc);
	if (ret)
		return ret;

	fpc1020_delay_us(FPC1020_POWER_CYCLE_US);

	return fpc1020_set_reset(fpc, true);
}

static int fpc1020_enable(struct fpc1020 *fpc)
{
	int ret;

	/* Start from reset, even if the sensor is still powered */
	ret = fpc1020_set_reset(fpc, true);
	if (ret)
		return ret;

	WRITE_ONCE(fpc->wakeup_enabled, true);
	enable_irq(fpc->irq);
	if (fpc->wakeup_capable)
		enable_irq_wake(fpc->irq);

	fpc->enabled = true;

	ret = fpc1020_set_power(fpc, true);
	if (ret)
		return ret;

	return fpc1020_reset(fpc);
}

static void fpc1020_disable(struct fpc1020 *fpc)
{
	if (fpc->wakeup_capable)
		disable_irq_wake(fpc->irq);
	disable_irq(fpc->irq);

	fpc->enabled = false;
}

static ssize_t compatible_all_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct fpc1020 *fpc = dev_get_drvdata(dev);
	int enable, ret = 0;

	enable = sysfs_match_string(fpc1020_states, buf);
	if (enable < 0)
		return enable;

	mutex_lock(&fpc->lock);

	if (enable && !fpc->enabled)
		ret = fpc1020_enable(fpc);
	else if (!enable && fpc->enabled)
		fpc1020_disable(fpc);

	mutex_unlock(&fpc->lock);

	return ret ?: count;
}
static DEVICE_ATTR_WO(compatible_all);

static ssize_t device_prepare_store(struct device *dev,
				    struct device_attribute *attr,
				    const char *buf, size_t count)
{
	struct fpc1020 *fpc = dev_get_drvdata(dev);
	int enable, ret;

	enable = sysfs_match_string(fpc1020_states, buf);
	if (enable < 0)
		return enable;

	mutex_lock(&fpc->lock);
	ret = fpc1020_set_power(fpc, enable);
	mutex_unlock(&fpc->lock);

	return ret ?: count;
}
static DEVICE_ATTR_WO(device_prepare);

static ssize_t hw_reset_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct fpc1020 *fpc = dev_get_drvdata(dev);
	int ret = 0;

	mutex_lock(&fpc->lock);

	if (sysfs_streq(buf, "reset")) {
		/* Give up on a sensor that keeps failing until it is cleared */
		if (fpc->reset_count < FPC1020_MAX_RESETS) {
			fpc->reset_count++;
			ret = fpc1020_power_cycle(fpc);
		} else {
			dev_warn(dev, "too many resets, powering off\n");
			fpc->reset_count = 0;
			ret = fpc1020_power_down(fpc);
		}
	} else if (sysfs_streq(buf, "clear")) {
		fpc->reset_count = 0;
	} else {
		ret = -EINVAL;
	}

	mutex_unlock(&fpc->lock);

	return ret ?: count;
}
static DEVICE_ATTR_WO(hw_reset);

static ssize_t wakeup_enable_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct fpc1020 *fpc = dev_get_drvdata(dev);
	int enable;

	enable = sysfs_match_string(fpc1020_states, buf);
	if (enable < 0)
		return enable;

	WRITE_ONCE(fpc->wakeup_enabled, enable);

	return count;
}
static DEVICE_ATTR_WO(wakeup_enable);

static ssize_t irq_show(struct device *dev, struct device_attribute *attr,
			char *buf)
{
	struct fpc1020 *fpc = dev_get_drvdata(dev);
	int val;

	val = gpiod_get_value_cansleep(fpc->irq_gpio);
	if (val < 0)
		return val;

	return sysfs_emit(buf, "%d\n", val);
}

/* The HAL writes here after each interrupt and expects it to succeed */
static ssize_t irq_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	return count;
}
static DEVICE_ATTR_ADMIN_RW(irq);

static struct attribute *fpc1020_attrs[] = {
	&dev_attr_compatible_all.attr,
	&dev_attr_device_prepare.attr,
	&dev_attr_hw_reset.attr,
	&dev_attr_wakeup_enable.attr,
	&dev_attr_irq.attr,
	NULL
};
ATTRIBUTE_GROUPS(fpc1020);

/* Threaded, as sysfs_notify() may sleep */
static irqreturn_t fpc1020_irq_thread(int irq, void *data)
{
	struct fpc1020 *fpc = data;

	if (READ_ONCE(fpc->wakeup_enabled))
		pm_wakeup_event(fpc->dev, FPC1020_WAKEUP_TIMEOUT_MS);

	sysfs_notify(&fpc->dev->kobj, NULL, dev_attr_irq.attr.name);

	return IRQ_HANDLED;
}

static int fpc1020_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct fpc1020 *fpc;
	int ret;

	fpc = devm_kzalloc(dev, sizeof(*fpc), GFP_KERNEL);
	if (!fpc)
		return -ENOMEM;

	fpc->dev = dev;
	mutex_init(&fpc->lock);

	/* Leave the sensor alone until the HAL takes it over */
	fpc->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_ASIS);
	if (IS_ERR(fpc->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(fpc->reset_gpio),
				     "failed to get reset GPIO\n");

	fpc->power_gpio = devm_gpiod_get(dev, "power", GPIOD_ASIS);
	if (IS_ERR(fpc->power_gpio))
		return dev_err_probe(dev, PTR_ERR(fpc->power_gpio),
				     "failed to get power GPIO\n");

	fpc->irq_gpio = devm_gpiod_get(dev, "irq", GPIOD_IN);
	if (IS_ERR(fpc->irq_gpio))
		return dev_err_probe(dev, PTR_ERR(fpc->irq_gpio),
				     "failed to get irq GPIO\n");

	fpc->irq = gpiod_to_irq(fpc->irq_gpio);
	if (fpc->irq < 0)
		return dev_err_probe(dev, fpc->irq, "failed to get IRQ\n");

	ret = devm_request_threaded_irq(dev, fpc->irq, NULL, fpc1020_irq_thread,
					IRQF_TRIGGER_RISING | IRQF_ONESHOT |
					IRQF_NO_AUTOEN, dev_name(dev), fpc);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request IRQ\n");

	fpc->wakeup_capable = device_property_read_bool(dev, "wakeup-source");
	device_init_wakeup(dev, fpc->wakeup_capable);

	platform_set_drvdata(pdev, fpc);

	return 0;
}

static int fpc1020_remove(struct platform_device *pdev)
{
	struct fpc1020 *fpc = platform_get_drvdata(pdev);

	mutex_lock(&fpc->lock);

	if (fpc->enabled)
		fpc1020_disable(fpc);
	fpc1020_set_power(fpc, false);

	mutex_unlock(&fpc->lock);

	device_init_wakeup(&pdev->dev, false);

	return 0;
}

static const struct of_device_id fpc1020_of_match[] = {
	{ .compatible = "fpc,fpc1020" },
	{ }
};
MODULE_DEVICE_TABLE(of, fpc1020_of_match);

static struct platform_driver fpc1020_driver = {
	.driver = {
		.name = "fpc1020",
		.of_match_table = fpc1020_of_match,
		.dev_groups = fpc1020_groups,
	},
	.probe = fpc1020_probe,
	.remove = fpc1020_remove,
};
module_platform_driver(fpc1020_driver);

MODULE_DESCRIPTION("Fingerprint Cards FPC1020 fingerprint sensor driver");
MODULE_LICENSE("GPL");
