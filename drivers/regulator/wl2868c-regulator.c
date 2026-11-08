// SPDX-License-Identifier: GPL-2.0-only
// SPDX-FileCopyrightText: The LineageOS Project
/*
 * Willsemi WL2868C seven channel LDO regulator driver
 */

#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>

#define WL2868C_CHIP_ID		0x00
#define WL2868C_LDO1_VOUT	0x03
#define WL2868C_LDO2_VOUT	0x04
#define WL2868C_LDO3_VOUT	0x05
#define WL2868C_LDO4_VOUT	0x06
#define WL2868C_LDO5_VOUT	0x07
#define WL2868C_LDO6_VOUT	0x08
#define WL2868C_LDO7_VOUT	0x09
#define WL2868C_LDO_EN		0x0e

#define WL2868C_CHIP_ID_VAL	0x82

enum wl2868c_regulator_ids {
	WL2868C_LDO1,
	WL2868C_LDO2,
	WL2868C_LDO3,
	WL2868C_LDO4,
	WL2868C_LDO5,
	WL2868C_LDO6,
	WL2868C_LDO7,
};

static const struct regulator_ops wl2868c_ldo_ops = {
	.list_voltage		= regulator_list_voltage_linear,
	.map_voltage		= regulator_map_voltage_linear,
	.set_voltage_sel	= regulator_set_voltage_sel_regmap,
	.get_voltage_sel	= regulator_get_voltage_sel_regmap,
	.enable			= regulator_enable_regmap,
	.disable		= regulator_disable_regmap,
	.is_enabled		= regulator_is_enabled_regmap,
};

#define WL2868C_LDO(_num, _supply, _min_uV, _vsel_mask, _enable_time)	\
	[WL2868C_LDO##_num] = {						\
		.name		= "ldo" #_num,				\
		.of_match	= "ldo" #_num,				\
		.regulators_node = "regulators",			\
		.supply_name	= _supply,				\
		.id		= WL2868C_LDO##_num,			\
		.ops		= &wl2868c_ldo_ops,			\
		.type		= REGULATOR_VOLTAGE,			\
		.owner		= THIS_MODULE,				\
		.min_uV		= _min_uV,				\
		.uV_step	= 8000,					\
		.n_voltages	= (_vsel_mask) + 1,			\
		.vsel_reg	= WL2868C_LDO##_num##_VOUT,		\
		.vsel_mask	= _vsel_mask,				\
		.enable_reg	= WL2868C_LDO_EN,			\
		.enable_mask	= BIT(WL2868C_LDO##_num),		\
		.enable_time	= _enable_time,				\
	}

/*
 * LDO1-2 cover 0.496-1.512 V and LDO3-7 cover 1.504-3.544 V in 8 mV steps.
 * The enable time is the turn-on delay plus the soft-start time.
 */
static const struct regulator_desc wl2868c_regulators[] = {
	WL2868C_LDO(1, "vin12", 496000, 0x7f, 85),
	WL2868C_LDO(2, "vin12", 496000, 0x7f, 85),
	WL2868C_LDO(3, "vin34", 1504000, 0xff, 105),
	WL2868C_LDO(4, "vin34", 1504000, 0xff, 105),
	WL2868C_LDO(5, "vin5", 1504000, 0xff, 105),
	WL2868C_LDO(6, "vin6", 1504000, 0xff, 105),
	WL2868C_LDO(7, "vin7", 1504000, 0xff, 105),
};

static const struct regmap_config wl2868c_regmap_config = {
	.reg_bits	= 8,
	.val_bits	= 8,
	.max_register	= WL2868C_LDO_EN,
};

static int wl2868c_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct regulator_config config = { };
	struct regulator_dev *rdev;
	struct gpio_desc *reset_gpio;
	struct regmap *regmap;
	unsigned int id;
	int i, ret;

	reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(reset_gpio))
		return dev_err_probe(dev, PTR_ERR(reset_gpio),
				     "failed to get reset GPIO\n");

	/* Wait for the chip to come out of reset */
	if (reset_gpio)
		usleep_range(10000, 11000);

	regmap = devm_regmap_init_i2c(client, &wl2868c_regmap_config);
	if (IS_ERR(regmap))
		return dev_err_probe(dev, PTR_ERR(regmap),
				     "failed to init regmap\n");

	ret = regmap_read(regmap, WL2868C_CHIP_ID, &id);
	if (ret)
		return dev_err_probe(dev, ret, "failed to read chip ID\n");

	if (id != WL2868C_CHIP_ID_VAL)
		return dev_err_probe(dev, -ENODEV,
				     "unknown chip ID 0x%02x\n", id);

	config.dev = dev;
	config.regmap = regmap;

	for (i = 0; i < ARRAY_SIZE(wl2868c_regulators); i++) {
		rdev = devm_regulator_register(dev, &wl2868c_regulators[i],
					       &config);
		if (IS_ERR(rdev))
			return dev_err_probe(dev, PTR_ERR(rdev),
					     "failed to register %s\n",
					     wl2868c_regulators[i].name);
	}

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
	.id_table = wl2868c_id,
};
module_i2c_driver(wl2868c_driver);

MODULE_DESCRIPTION("Willsemi WL2868C regulator driver");
MODULE_LICENSE("GPL");
