// SPDX-License-Identifier: GPL-2.0-only
// SPDX-FileCopyrightText: The LineageOS Project
/*
 * Parade PS5169 Type-C linear redriver driver
 */

#include <linux/bits.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/lockdep.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/pm.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/usb/redriver.h>

#include <asm/byteorder.h>

#define PS5169_CONFIG_REG		0x40
#define PS5169_CONFIG_REVERSE		BIT(4)
#define PS5169_CONFIG_DP_EN		BIT(5)
#define PS5169_CONFIG_USB3_EN		BIT(6)
#define PS5169_CONFIG_BASE		BIT(7)

#define PS5169_GAIN_REG			0x5c
#define PS5169_GAIN_USB_TX		BIT(0)
#define PS5169_GAIN_USB_RX		BIT(2)
#define PS5169_GAIN_DP			BIT(4)
#define PS5169_GAIN_MASK		(PS5169_GAIN_USB_TX | \
					 PS5169_GAIN_USB_RX | \
					 PS5169_GAIN_DP)

#define PS5169_USB_RX1_LFPS_REG		0x8d
#define PS5169_USB_RX2_LFPS_REG		0x90
#define PS5169_USB_RX_LFPS_50_OHM	BIT(0)

#define PS5169_RESET_REG		0x9d
#define PS5169_RESET			BIT(7)

#define PS5169_AUX_MONITORING_REG	0xa0
#define PS5169_AUX_MONITORING_DIS	BIT(1)

#define PS5169_HPD_LEVEL_REG		0xa1
#define PS5169_HPD_LEVEL_HIGH		BIT(2)

#define PS5169_CHIP_ID_REG		0xac
#define PS5169_CHIP_ID			0x6987

#define PS5169_MAX_REG			0xaf

#define PS5169_RESET_DELAY_US		10000
#define PS5169_EQ_LEVELS		8
#define PS5169_LANES_DP			4

enum ps5169_mode {
	PS5169_MODE_NONE = 0,
	PS5169_MODE_USB = BIT(0),
	PS5169_MODE_DP = BIT(1),
	PS5169_MODE_USB_DP = PS5169_MODE_USB | PS5169_MODE_DP,
};

struct ps5169_eq {
	u8 reg;
	u8 mask;
	u8 val[PS5169_EQ_LEVELS];
};

struct ps5169 {
	struct usb_redriver r;
	struct device *dev;
	struct regmap *regmap;
	struct gpio_desc *reset_gpio;
	struct gpio_desc *orientation_gpio;
	struct mutex lock; /* protects mode and orientation */
	enum ps5169_mode mode;
	int orientation;
};

/* Default settings, the equalizer levels of the board are applied on top */
static const struct reg_sequence ps5169_init_seq[] = {
	{ 0x04, 0x44 },
	{ 0x50, 0x20 },
	{ 0x51, 0x87 },
	{ 0x52, 0x50 },
	{ 0x54, 0x11 },
	{ 0x55, 0x00 },
	{ 0x56, 0x00 },
	{ 0x57, 0x00 },
	{ 0x58, 0x00 },
	{ 0x59, 0x00 },
	{ 0x5a, 0x00 },
	{ 0x5b, 0x00 },
	{ 0x5d, 0x66 },
	{ 0x5e, 0x06 },
	{ 0x5f, 0x00 },
	{ 0x60, 0x00 },
	{ 0x61, 0x03 },
	{ 0x65, 0x40 },
	{ 0x66, 0x00 },
	{ 0x67, 0x03 },
	{ 0x75, 0x0c },
	{ 0x77, 0x00 },
	{ 0x78, 0x7c },
};

/* 2, 5.5, 6.5, 7.5, 8, 8.5, 9.5 and 10 dB */
static const struct ps5169_eq ps5169_dp_eq[] = {
	{ 0x52, 0x70, { 0x00, 0x10, 0x20, 0x30, 0x40, 0x50, 0x60, 0x70 } },
	{ 0x5e, 0x07, { 0x04, 0x05, 0x06, 0x06, 0x06, 0x07, 0x07, 0x07 } },
};

/* 5.2, 6, 7, 8, 8.8, 9.6, 10.4 and 11.2 dB */
static const struct ps5169_eq ps5169_usb_rx_eq[] = {
	{ 0x51, 0xf6, { 0x86, 0x96, 0xa6, 0xb6, 0xc6, 0xd6, 0xe6, 0xf6 } },
	{ 0x54, 0x0f, { 0x00, 0x01, 0x01, 0x05, 0x0c, 0x05, 0x0f, 0x0f } },
	{ 0x77, 0xf0, { 0x00, 0x00, 0x50, 0x50, 0xb0, 0xf0, 0xf0, 0x30 } },
	{ 0x78, 0xe1, { 0x20, 0x20, 0x40, 0x40, 0x80, 0x80, 0x80, 0xa1 } },
};

/* 2, 5.5, 6.5, 7.5, 8, 8.5, 9.5 and 10 dB */
static const struct ps5169_eq ps5169_usb_tx_eq[] = {
	{ 0x50, 0x70, { 0x00, 0x10, 0x20, 0x30, 0x40, 0x50, 0x60, 0x70 } },
	{ 0x54, 0xf0, { 0x00, 0x10, 0x10, 0x50, 0xc0, 0x50, 0xf0, 0xf0 } },
	{ 0x5d, 0x70, { 0x40, 0x50, 0x60, 0x60, 0x60, 0x70, 0x70, 0x70 } },
};

static int ps5169_write_mode(struct ps5169 *ps, enum ps5169_mode mode)
{
	bool usb = mode & PS5169_MODE_USB;
	bool dp = mode & PS5169_MODE_DP;
	bool reverse = mode && ps->orientation == ORIENTATION_CC2;
	unsigned int config = PS5169_CONFIG_BASE |
			      (usb ? PS5169_CONFIG_USB3_EN : 0) |
			      (dp ? PS5169_CONFIG_DP_EN : 0) |
			      (reverse ? PS5169_CONFIG_REVERSE : 0);
	unsigned int lfps = usb ? PS5169_USB_RX_LFPS_50_OHM : 0;
	unsigned int aux = dp ? 0 : PS5169_AUX_MONITORING_DIS;
	unsigned int hpd = dp ? PS5169_HPD_LEVEL_HIGH : 0;
	const struct reg_sequence seq[] = {
		{ PS5169_CONFIG_REG, config },
		{ PS5169_USB_RX1_LFPS_REG, lfps },
		{ PS5169_USB_RX2_LFPS_REG, lfps },
		{ PS5169_AUX_MONITORING_REG, aux },
		{ PS5169_HPD_LEVEL_REG, hpd },
	};

	return regmap_multi_reg_write(ps->regmap, seq, ARRAY_SIZE(seq));
}

static int ps5169_set_mode(struct ps5169 *ps, enum ps5169_mode mode, int ort)
{
	int ret;

	lockdep_assert_held(&ps->lock);

	ps->mode = mode;
	ps->orientation = ort;

	ret = ps5169_write_mode(ps, mode);
	if (ret)
		dev_err(ps->dev, "failed to set mode %d: %d\n", mode, ret);

	return ret;
}

static int ps5169_release_usb_lanes(struct usb_redriver *r, int ort, int num)
{
	struct ps5169 *ps = container_of(r, struct ps5169, r);
	enum ps5169_mode mode = num == PS5169_LANES_DP ? PS5169_MODE_DP :
							 PS5169_MODE_USB_DP;
	int ret;

	mutex_lock(&ps->lock);
	ret = ps5169_set_mode(ps, mode, ort);
	mutex_unlock(&ps->lock);

	return ret;
}

static int ps5169_notify_connect(struct usb_redriver *r, int ort)
{
	struct ps5169 *ps = container_of(r, struct ps5169, r);
	int ret;

	mutex_lock(&ps->lock);
	/* Keep the lanes the display driver may have already claimed */
	ret = ps5169_set_mode(ps, ps->mode ?: PS5169_MODE_USB, ort);
	mutex_unlock(&ps->lock);

	return ret;
}

static int ps5169_notify_disconnect(struct usb_redriver *r)
{
	struct ps5169 *ps = container_of(r, struct ps5169, r);
	int ret;

	mutex_lock(&ps->lock);
	ret = ps5169_set_mode(ps, PS5169_MODE_NONE, ps->orientation);
	mutex_unlock(&ps->lock);

	return ret;
}

static int ps5169_get_orientation(struct usb_redriver *r)
{
	struct ps5169 *ps = container_of(r, struct ps5169, r);
	int ret;

	ret = gpiod_get_value_cansleep(ps->orientation_gpio);
	if (ret < 0)
		return ret;

	return ret ? ORIENTATION_CC2 : ORIENTATION_CC1;
}

static int ps5169_set_eq(struct ps5169 *ps, const char *prop,
			 const struct ps5169_eq *eq, unsigned int num)
{
	unsigned int i;
	u32 level;
	int ret;

	/* Keep the default setting if the board does not provide a level */
	if (!device_property_present(ps->dev, prop))
		return 0;

	ret = device_property_read_u32(ps->dev, prop, &level);
	if (ret)
		return ret;

	if (level >= PS5169_EQ_LEVELS) {
		dev_err(ps->dev, "invalid %s: %u\n", prop, level);
		return -EINVAL;
	}

	for (i = 0; i < num; i++) {
		ret = regmap_update_bits(ps->regmap, eq[i].reg, eq[i].mask,
					 eq[i].val[level]);
		if (ret)
			return ret;
	}

	return 0;
}

static int ps5169_set_gain(struct ps5169 *ps)
{
	unsigned int gain = 0;

	/* Each of these lowers the gain of its channels by 0.9 dB */
	if (device_property_read_bool(ps->dev, "parade,dp-set-gain"))
		gain |= PS5169_GAIN_DP;
	if (device_property_read_bool(ps->dev, "parade,usb-rx-set-gain"))
		gain |= PS5169_GAIN_USB_RX;
	if (device_property_read_bool(ps->dev, "parade,usb-tx-set-gain"))
		gain |= PS5169_GAIN_USB_TX;

	return regmap_update_bits(ps->regmap, PS5169_GAIN_REG,
				  PS5169_GAIN_MASK, gain);
}

static int ps5169_init(struct ps5169 *ps)
{
	int ret;

	ret = regmap_write(ps->regmap, PS5169_RESET_REG, PS5169_RESET);
	if (ret)
		return ret;

	usleep_range(PS5169_RESET_DELAY_US, PS5169_RESET_DELAY_US + 1000);

	ret = regmap_write(ps->regmap, PS5169_RESET_REG, 0);
	if (ret)
		return ret;

	ret = regmap_multi_reg_write(ps->regmap, ps5169_init_seq,
				     ARRAY_SIZE(ps5169_init_seq));
	if (ret)
		return ret;

	ret = ps5169_set_eq(ps, "parade,dp-eq-level", ps5169_dp_eq,
			    ARRAY_SIZE(ps5169_dp_eq));
	if (ret)
		return ret;

	ret = ps5169_set_eq(ps, "parade,usb-rx-eq-level", ps5169_usb_rx_eq,
			    ARRAY_SIZE(ps5169_usb_rx_eq));
	if (ret)
		return ret;

	ret = ps5169_set_eq(ps, "parade,usb-tx-eq-level", ps5169_usb_tx_eq,
			    ARRAY_SIZE(ps5169_usb_tx_eq));
	if (ret)
		return ret;

	ret = ps5169_set_gain(ps);
	if (ret)
		return ret;

	return ps5169_write_mode(ps, PS5169_MODE_NONE);
}

static const struct regmap_config ps5169_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = PS5169_MAX_REG,
};

static int ps5169_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct ps5169 *ps;
	__le16 id;
	int ret;

	ps = devm_kzalloc(dev, sizeof(*ps), GFP_KERNEL);
	if (!ps)
		return -ENOMEM;

	ps->dev = dev;
	ps->mode = PS5169_MODE_NONE;
	ps->orientation = ORIENTATION_CC1;
	mutex_init(&ps->lock);

	ps->reset_gpio = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(ps->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ps->reset_gpio),
				     "failed to get reset GPIO\n");

	ps->orientation_gpio = devm_gpiod_get_optional(dev, "orientation",
						       GPIOD_IN);
	if (IS_ERR(ps->orientation_gpio))
		return dev_err_probe(dev, PTR_ERR(ps->orientation_gpio),
				     "failed to get orientation GPIO\n");

	/* Wait for the chip to come out of reset */
	if (ps->reset_gpio)
		usleep_range(PS5169_RESET_DELAY_US,
			     PS5169_RESET_DELAY_US + 1000);

	ps->regmap = devm_regmap_init_i2c(client, &ps5169_regmap_config);
	if (IS_ERR(ps->regmap))
		return dev_err_probe(dev, PTR_ERR(ps->regmap),
				     "failed to init regmap\n");

	ret = regmap_bulk_read(ps->regmap, PS5169_CHIP_ID_REG, &id, sizeof(id));
	if (ret)
		return dev_err_probe(dev, ret, "failed to read chip ID\n");

	if (le16_to_cpu(id) != PS5169_CHIP_ID)
		return dev_err_probe(dev, -ENODEV, "unknown chip ID 0x%04x\n",
				     le16_to_cpu(id));

	ret = ps5169_init(ps);
	if (ret)
		return dev_err_probe(dev, ret, "failed to init chip\n");

	i2c_set_clientdata(client, ps);

	ps->r.of_node = dev->of_node;
	ps->r.release_usb_lanes = ps5169_release_usb_lanes;
	ps->r.notify_connect = ps5169_notify_connect;
	ps->r.notify_disconnect = ps5169_notify_disconnect;
	if (ps->orientation_gpio) {
		ps->r.get_orientation = ps5169_get_orientation;
		ps->r.has_orientation = true;
	}

	return usb_add_redriver(&ps->r);
}

static int ps5169_remove(struct i2c_client *client)
{
	struct ps5169 *ps = i2c_get_clientdata(client);

	return usb_remove_redriver(&ps->r);
}

static void ps5169_shutdown(struct i2c_client *client)
{
	struct ps5169 *ps = i2c_get_clientdata(client);

	gpiod_set_value_cansleep(ps->reset_gpio, 1);
}

static int ps5169_suspend(struct device *dev)
{
	struct ps5169 *ps = dev_get_drvdata(dev);
	int ret = 0;

	mutex_lock(&ps->lock);

	/* DP only links have to keep running across suspend */
	if (ps->mode & PS5169_MODE_USB)
		ret = ps5169_write_mode(ps, PS5169_MODE_NONE);

	mutex_unlock(&ps->lock);

	return ret;
}

static int ps5169_resume(struct device *dev)
{
	struct ps5169 *ps = dev_get_drvdata(dev);
	int ret = 0;

	mutex_lock(&ps->lock);

	if (ps->mode & PS5169_MODE_USB)
		ret = ps5169_write_mode(ps, ps->mode);

	mutex_unlock(&ps->lock);

	return ret;
}

DEFINE_SIMPLE_DEV_PM_OPS(ps5169_pm_ops, ps5169_suspend, ps5169_resume);

static const struct of_device_id ps5169_of_match[] = {
	{ .compatible = "parade,ps5169" },
	{ }
};
MODULE_DEVICE_TABLE(of, ps5169_of_match);

static struct i2c_driver ps5169_driver = {
	.driver = {
		.name = "ps5169",
		.of_match_table = ps5169_of_match,
		.pm = pm_sleep_ptr(&ps5169_pm_ops),
		.suppress_bind_attrs = true,
	},
	.probe_new = ps5169_probe,
	.remove = ps5169_remove,
	.shutdown = ps5169_shutdown,
};
module_i2c_driver(ps5169_driver);

MODULE_DESCRIPTION("Parade PS5169 Type-C linear redriver driver");
MODULE_LICENSE("GPL");
