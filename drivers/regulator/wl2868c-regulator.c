// SPDX-License-Identifier: GPL-2.0-only
// SPDX-FileCopyrightText: The LineageOS Project
/*
 * Willsemi WL2868C LDO regulator driver
 */

#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/regulator/driver.h>

#define WL2868C_DEVICE_ID		0x00
#define WL2868C_DEVICE_ID_VAL		0x82

#define WL2868C_DISCHARGE		0x02
#define WL2868C_LDO_VOUT(id)		(0x03 + (id))

#define WL2868C_LDO_SEQ(n)		(0x0a + (n))
#define WL2868C_NUM_LDO_SEQ		4

#define WL2868C_LDO_EN			0x0e
#define WL2868C_LDO_EN_MASK		GENMASK(6, 0)
#define WL2868C_SYS_EN			BIT(7)

#define WL2868C_LDO_STATUS(id)		(0x10 + (id) * 2)
#define WL2868C_STATUS_VIN_OK		BIT(0)
#define WL2868C_STATUS_OCP		BIT(5)
#define WL2868C_STATUS_ERROR		BIT(6)
#define WL2868C_STATUS_READY		BIT(7)

#define WL2868C_LDO_OCP_CTL(id)		(0x11 + (id) * 2)
#define WL2868C_OCP_STATUS_CLR		BIT(1)

#define WL2868C_MAX_REG			0x25

/* LDO1-2 cover 0.496-1.512 V and LDO3-7 cover 1.504-3.544 V in 8 mV steps */
#define WL2868C_LDO12_MIN_UV		496000
#define WL2868C_LDO12_VSEL_MASK		0x7f
#define WL2868C_LDO37_MIN_UV		1504000
#define WL2868C_LDO37_VSEL_MASK		0xff
#define WL2868C_VOLTAGE_STEP_UV		8000

/* The output voltage range is specified for inputs 200 mV above the output */
#define WL2868C_MIN_DROPOUT_UV		200000

/* Turn-on delay and soft-start take about 100 us, so leave some margin */
#define WL2868C_ENABLE_TIME_US		200

/* No start-up time is specified, so allow plenty after reset and sleep */
#define WL2868C_STARTUP_DELAY_US	5000

enum wl2868c_regulator_ids {
	WL2868C_LDO1,
	WL2868C_LDO2,
	WL2868C_LDO3,
	WL2868C_LDO4,
	WL2868C_LDO5,
	WL2868C_LDO6,
	WL2868C_LDO7,
};

struct wl2868c {
	struct gpio_desc *reset_gpio;
	struct mutex lock; /* serializes the updates of the enable register */
};

static void wl2868c_wait_startup(void)
{
	usleep_range(WL2868C_STARTUP_DELAY_US, WL2868C_STARTUP_DELAY_US + 1000);
}

/* A latched over-current event has to be cleared and re-armed */
static int wl2868c_clear_ocp(struct regulator_dev *rdev)
{
	unsigned int id = rdev_get_id(rdev);
	unsigned int status;
	int ret;

	ret = regmap_read(rdev->regmap, WL2868C_LDO_STATUS(id), &status);
	if (ret)
		return ret;

	if (!(status & WL2868C_STATUS_OCP))
		return 0;

	ret = regmap_update_bits(rdev->regmap, WL2868C_LDO_OCP_CTL(id),
				 WL2868C_OCP_STATUS_CLR,
				 WL2868C_OCP_STATUS_CLR);
	if (ret)
		return ret;

	return regmap_update_bits(rdev->regmap, WL2868C_LDO_OCP_CTL(id),
				  WL2868C_OCP_STATUS_CLR, 0);
}

static int wl2868c_enable(struct regulator_dev *rdev)
{
	struct wl2868c *chip = rdev_get_drvdata(rdev);
	unsigned int val;
	int ret;

	mutex_lock(&chip->lock);

	ret = regmap_read(rdev->regmap, WL2868C_LDO_EN, &val);
	if (ret)
		goto unlock;

	/* The chip sleeps while SYS_EN is clear, so wake it up first */
	if (!(val & WL2868C_SYS_EN)) {
		val = WL2868C_SYS_EN;

		ret = regmap_write(rdev->regmap, WL2868C_LDO_EN, val);
		if (ret)
			goto unlock;

		wl2868c_wait_startup();
	}

	ret = wl2868c_clear_ocp(rdev);
	if (ret)
		goto unlock;

	ret = regmap_write(rdev->regmap, WL2868C_LDO_EN,
			   val | rdev->desc->enable_mask);

unlock:
	mutex_unlock(&chip->lock);

	return ret;
}

static int wl2868c_disable(struct regulator_dev *rdev)
{
	struct wl2868c *chip = rdev_get_drvdata(rdev);
	unsigned int val;
	int ret;

	mutex_lock(&chip->lock);

	ret = regmap_read(rdev->regmap, WL2868C_LDO_EN, &val);
	if (ret)
		goto unlock;

	val &= ~rdev->desc->enable_mask;

	/* Let the chip sleep once the last LDO is disabled */
	if (!(val & WL2868C_LDO_EN_MASK))
		val = 0;

	ret = regmap_write(rdev->regmap, WL2868C_LDO_EN, val);

unlock:
	mutex_unlock(&chip->lock);

	return ret;
}

static int wl2868c_is_enabled(struct regulator_dev *rdev)
{
	unsigned int mask = WL2868C_SYS_EN | rdev->desc->enable_mask;
	unsigned int val;
	int ret;

	ret = regmap_read(rdev->regmap, WL2868C_LDO_EN, &val);
	if (ret)
		return ret;

	return (val & mask) == mask;
}

static int wl2868c_get_status(struct regulator_dev *rdev)
{
	unsigned int id = rdev_get_id(rdev);
	unsigned int status;
	int ret;

	ret = wl2868c_is_enabled(rdev);
	if (ret < 0)
		return ret;

	if (!ret)
		return REGULATOR_STATUS_OFF;

	ret = regmap_read(rdev->regmap, WL2868C_LDO_STATUS(id), &status);
	if (ret)
		return ret;

	if (status & WL2868C_STATUS_READY)
		return REGULATOR_STATUS_ON;

	if (status & (WL2868C_STATUS_OCP | WL2868C_STATUS_ERROR))
		return REGULATOR_STATUS_ERROR;

	/* The output is still ramping */
	return REGULATOR_STATUS_UNDEFINED;
}

static int wl2868c_get_error_flags(struct regulator_dev *rdev,
				   unsigned int *flags)
{
	unsigned int id = rdev_get_id(rdev);
	unsigned int status;
	int ret;

	ret = regmap_read(rdev->regmap, WL2868C_LDO_STATUS(id), &status);
	if (ret)
		return ret;

	*flags = 0;

	if (status & WL2868C_STATUS_OCP)
		*flags |= REGULATOR_ERROR_OVER_CURRENT;

	/* The other status bits are only valid while the LDO is enabled */
	ret = wl2868c_is_enabled(rdev);
	if (ret <= 0)
		return ret;

	if (!(status & WL2868C_STATUS_VIN_OK))
		*flags |= REGULATOR_ERROR_UNDER_VOLTAGE;

	if (status & WL2868C_STATUS_ERROR)
		*flags |= REGULATOR_ERROR_REGULATION_OUT;

	return 0;
}

static const struct regulator_ops wl2868c_ldo_ops = {
	.list_voltage = regulator_list_voltage_linear,
	.map_voltage = regulator_map_voltage_linear,
	.set_voltage_sel = regulator_set_voltage_sel_regmap,
	.get_voltage_sel = regulator_get_voltage_sel_regmap,
	.enable = wl2868c_enable,
	.disable = wl2868c_disable,
	.is_enabled = wl2868c_is_enabled,
	.set_active_discharge = regulator_set_active_discharge_regmap,
	.get_status = wl2868c_get_status,
	.get_error_flags = wl2868c_get_error_flags,
};

#define WL2868C_LDO(_num, _supply, _range)				\
	[WL2868C_LDO##_num] = {						\
		.name = "ldo" #_num,					\
		.of_match = "ldo" #_num,				\
		.regulators_node = "regulators",			\
		.supply_name = _supply,					\
		.id = WL2868C_LDO##_num,				\
		.ops = &wl2868c_ldo_ops,				\
		.type = REGULATOR_VOLTAGE,				\
		.owner = THIS_MODULE,					\
		.min_uV = WL2868C_##_range##_MIN_UV,			\
		.uV_step = WL2868C_VOLTAGE_STEP_UV,			\
		.n_voltages = WL2868C_##_range##_VSEL_MASK + 1,		\
		.min_dropout_uV = WL2868C_MIN_DROPOUT_UV,		\
		.vsel_reg = WL2868C_LDO_VOUT(WL2868C_LDO##_num),	\
		.vsel_mask = WL2868C_##_range##_VSEL_MASK,		\
		.enable_mask = BIT(WL2868C_LDO##_num),			\
		.enable_time = WL2868C_ENABLE_TIME_US,			\
		.active_discharge_reg = WL2868C_DISCHARGE,		\
		.active_discharge_mask = BIT(WL2868C_LDO##_num),	\
		.active_discharge_on = 0,				\
		.active_discharge_off = BIT(WL2868C_LDO##_num),		\
	}

static const struct regulator_desc wl2868c_regulators[] = {
	WL2868C_LDO(1, "vin12", LDO12),
	WL2868C_LDO(2, "vin12", LDO12),
	WL2868C_LDO(3, "vin34", LDO37),
	WL2868C_LDO(4, "vin34", LDO37),
	WL2868C_LDO(5, "vin5", LDO37),
	WL2868C_LDO(6, "vin6", LDO37),
	WL2868C_LDO(7, "vin7", LDO37),
};

static const struct regmap_config wl2868c_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = WL2868C_MAX_REG,
};

static int wl2868c_init(struct regmap *regmap)
{
	unsigned int val;
	unsigned int i;
	int ret;

	/* Control the LDOs with their enable bits instead of the sequencer */
	for (i = 0; i < WL2868C_NUM_LDO_SEQ; i++) {
		ret = regmap_write(regmap, WL2868C_LDO_SEQ(i), 0);
		if (ret)
			return ret;
	}

	ret = regmap_read(regmap, WL2868C_LDO_EN, &val);
	if (ret)
		return ret;

	/* Keep the chip awake only if an LDO has been left enabled */
	if ((val & WL2868C_SYS_EN) && (val & WL2868C_LDO_EN_MASK))
		return 0;

	return regmap_write(regmap, WL2868C_LDO_EN, 0);
}

static void wl2868c_disable_vsys(void *vsys)
{
	regulator_disable(vsys);
}

static int wl2868c_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct regulator_config config = { };
	struct regulator_dev *rdev;
	struct regulator *vsys;
	struct regmap *regmap;
	struct wl2868c *chip;
	unsigned int id;
	unsigned int i;
	int ret;

	chip = devm_kzalloc(dev, sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;

	mutex_init(&chip->lock);

	/* VSYS biases the LDOs and, like VIN7, powers the control logic */
	vsys = devm_regulator_get(dev, "vsys");
	if (IS_ERR(vsys))
		return dev_err_probe(dev, PTR_ERR(vsys),
				     "failed to get vsys supply\n");

	ret = regulator_enable(vsys);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to enable vsys supply\n");

	ret = devm_add_action_or_reset(dev, wl2868c_disable_vsys, vsys);
	if (ret)
		return ret;

	/* A chip that is already out of reset keeps its registers */
	chip->reset_gpio = devm_gpiod_get_optional(dev, "reset",
						   GPIOD_OUT_LOW);
	if (IS_ERR(chip->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(chip->reset_gpio),
				     "failed to get reset GPIO\n");

	wl2868c_wait_startup();

	regmap = devm_regmap_init_i2c(client, &wl2868c_regmap_config);
	if (IS_ERR(regmap))
		return dev_err_probe(dev, PTR_ERR(regmap),
				     "failed to init regmap\n");

	ret = regmap_read(regmap, WL2868C_DEVICE_ID, &id);
	if (ret)
		return dev_err_probe(dev, ret, "failed to read device ID\n");

	if (id != WL2868C_DEVICE_ID_VAL)
		return dev_err_probe(dev, -ENODEV,
				     "unknown device ID 0x%02x\n", id);

	ret = wl2868c_init(regmap);
	if (ret)
		return dev_err_probe(dev, ret, "failed to init chip\n");

	config.dev = dev;
	config.regmap = regmap;
	config.driver_data = chip;

	for (i = 0; i < ARRAY_SIZE(wl2868c_regulators); i++) {
		rdev = devm_regulator_register(dev, &wl2868c_regulators[i],
					       &config);
		if (IS_ERR(rdev))
			return dev_err_probe(dev, PTR_ERR(rdev),
					     "failed to register %s\n",
					     wl2868c_regulators[i].name);
	}

	i2c_set_clientdata(client, chip);

	return 0;
}

static int wl2868c_remove(struct i2c_client *client)
{
	struct wl2868c *chip = i2c_get_clientdata(client);

	/* Turn the LDOs off before VSYS is disabled */
	gpiod_set_value_cansleep(chip->reset_gpio, 1);

	return 0;
}

static const struct of_device_id wl2868c_of_match[] = {
	{ .compatible = "willsemi,wl2868c" },
	{ }
};
MODULE_DEVICE_TABLE(of, wl2868c_of_match);

static const struct i2c_device_id wl2868c_id[] = {
	{ "wl2868c" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, wl2868c_id);

static struct i2c_driver wl2868c_driver = {
	.driver = {
		.name = "wl2868c",
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
		.of_match_table = wl2868c_of_match,
	},
	.probe_new = wl2868c_probe,
	.remove = wl2868c_remove,
	.id_table = wl2868c_id,
};
module_i2c_driver(wl2868c_driver);

MODULE_DESCRIPTION("Willsemi WL2868C LDO regulator driver");
MODULE_LICENSE("GPL");
