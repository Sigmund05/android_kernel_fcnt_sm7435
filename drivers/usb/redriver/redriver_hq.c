#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/regmap.h>
#include <linux/of_gpio.h>
#include <linux/gpio.h>
#include <linux/unistd.h>
#include <linux/of.h>
#include <linux/usb/redriver.h>
#include <linux/usb/ch9.h>

#define GEN_MODE_CFG_REG            0x40
#define GEN_MODE_DP_EQ0             0x52
#define GEN_MODE_DP_EQ1             0x5E
#define GEN_MODE_DP_GAIN_REG        0x5C
#define GEN_MODE_USB_TX_EQ0_REG     0x50
#define GEN_MODE_USB_TX_EQ1_REG     0x5D
#define GEN_MODE_USB_TX_EQ2_REG     0x54

#define GEN_MODE_USB_RX_RS_REG      0x51
#define GEN_MODE_USB_RX_DCCM_REG    0x77
#define GEN_MODE_USB_RX_ITAIL_REG   0x78

//DP EQ Level            DP EQ      DP gain     USB TX1/2 EQ    USB TX1/2 gain      USB RX1/2 EQ      USB RX1/2 gain
#define LEVEL_1      1   //2          0             2                0                   5.2                 0
#define LEVEL_2      2   //5.5       -0.9           5.5             -0.9                 6                  -0.9
#define LEVEL_3      3   //6.5                      6.5                                  7
#define LEVEL_4      4   //7.5                      7.5                                  8
#define LEVEL_5      5   //8                        8                                    8.8
#define LEVEL_6      6   //8.5                      8.5                                  9.6
#define LEVEL_7      7   //9.5                      9.5                                  10.4
#define LEVEL_8      8   //                                                              11.2

#define BIT_0   BIT(0)
#define BIT_1   BIT(1)
#define BIT_2   BIT(2)
#define BIT_3   BIT(3)
#define BIT_4   BIT(4)
#define BIT_5   BIT(5)
#define BIT_6   BIT(6)
#define BIT_7   BIT(7)

enum openration_mode {
    OP_MODE_NONE,
    OP_MODE_USB,
    OP_MODE_DP,
    OP_MODE_DP_AND_USB,
    OP_MODE_DEFAULT,
};

#define LANES_DP		4
#define LANES_DP_AND_USB	2

struct ps5169_redriver {
    struct usb_redriver r;
    struct device       *dev;
    struct regmap       *regmap;
    struct i2c_client   *client;

    u8                  dp_eq;
    u8                  dp_gain;
    u8                  usb_tx_eq;
    u8                  usb_tx_gain;
    u8                  usb_rx_eq;
    u8                  usb_rx_gain;

    enum openration_mode    op_mode;

    int                 en_gpio;
    int                 orientation_gpio;
    int                 typec_orientation;

    struct class        debug_class;
};

static const char * const opmode_string[] = {
    [OP_MODE_NONE] = "NONE",
    [OP_MODE_USB] = "USB",
    [OP_MODE_DP] = "DP",
    [OP_MODE_DP_AND_USB] = "USB and DP",
    [OP_MODE_DEFAULT] = "DEFAULT",
};
#define OPMODESTR(x) opmode_string[x]

static void setting_dp_eq(struct ps5169_redriver *redriver);
static void setting_dp_gain(struct ps5169_redriver *redriver);
static void setting_usb_tx_eq(struct ps5169_redriver *redriver);
static void setting_usb_tx_gain(struct ps5169_redriver *redriver);
static void setting_usb_rx_eq(struct ps5169_redriver *redriver);
static void setting_usb_rx_gain(struct ps5169_redriver *redriver);

static int ps5169_reg_set(struct ps5169_redriver *redriver, u8 reg, u8 val)
{
    int ret;

    ret = regmap_write(redriver->regmap, (unsigned int)reg,
                    (unsigned int)val);
    if (ret < 0) {
        dev_err(redriver->dev, "writing reg 0x%02x failure\n", reg);
        return ret;
    }

    dev_dbg(redriver->dev, "writing reg 0x%02x=0x%02x\n", reg, val);

    return 0;
}

static int ps5169_reg_get(struct ps5169_redriver *redriver, u8 reg, u8 *val)
{
    unsigned int value;
    int ret;

    ret = regmap_read(redriver->regmap, (unsigned int)reg,
                    &value);
    if (ret < 0) {
        dev_err(redriver->dev, "reading reg 0x%02x failure\n", reg);
        return ret;
    }

    *val = (u8)value;

    dev_dbg(redriver->dev, "reading reg 0x%02x=0x%02x\n", reg, *val);

    return 0;
}

static int ps5169_masked_write(struct ps5169_redriver *redriver, u8 reg, u8 mask, u8 val)
{
    int rc;

    dev_dbg(redriver->dev, "%s\n",__func__);

    rc = regmap_update_bits(redriver->regmap, reg, mask, val);
    if (rc) {
        dev_err(redriver->dev, "%s: mask write error:%d\n",__func__, rc);
        return rc;
    } 

    return 0;
}

static ssize_t debug_set_dp_eq_show(struct class *c, struct class_attribute *attr, char *buf)
{
    struct ps5169_redriver *redriver = container_of(c, struct ps5169_redriver, debug_class);

    return scnprintf(buf, PAGE_SIZE, "%d\n", redriver->dp_eq);
}

static ssize_t debug_set_dp_eq_store(struct class *c, struct class_attribute *attr, const char *buf, size_t count)
{
    struct ps5169_redriver *redriver = container_of(c, struct ps5169_redriver, debug_class);

    int val;

    if (kstrtoint(buf, 0, &val))
            return -EINVAL;

    redriver->dp_eq = val;

    setting_dp_eq(redriver);

    return count;
}
static CLASS_ATTR_RW(debug_set_dp_eq);

static ssize_t debug_set_dp_gain_show(struct class *c, struct class_attribute *attr, char *buf)
{
    struct ps5169_redriver *redriver = container_of(c, struct ps5169_redriver, debug_class);

    return scnprintf(buf, PAGE_SIZE, "%d\n", redriver->dp_gain);
}

static ssize_t debug_set_dp_gain_store(struct class *c, struct class_attribute *attr, const char *buf, size_t count)
{
    struct ps5169_redriver *redriver = container_of(c, struct ps5169_redriver, debug_class);

    int val;

    if (kstrtoint(buf, 0, &val))
            return -EINVAL;

    redriver->dp_gain = val;

    setting_dp_gain(redriver);

    return count;
}
static CLASS_ATTR_RW(debug_set_dp_gain);

static ssize_t debug_set_usb_tx_eq_show(struct class *c, struct class_attribute *attr, char *buf)
{
    struct ps5169_redriver *redriver = container_of(c, struct ps5169_redriver, debug_class);

    return scnprintf(buf, PAGE_SIZE, "%d\n", redriver->usb_tx_eq);
}

static ssize_t debug_set_usb_tx_eq_store(struct class *c, struct class_attribute *attr, const char *buf, size_t count)
{
    struct ps5169_redriver *redriver = container_of(c, struct ps5169_redriver, debug_class);

    int val;

    if (kstrtoint(buf, 0, &val))
            return -EINVAL;

    redriver->usb_tx_eq = val;

    setting_usb_tx_eq(redriver);

    return count;
}
static CLASS_ATTR_RW(debug_set_usb_tx_eq);

static ssize_t debug_set_usb_tx_gain_show(struct class *c, struct class_attribute *attr, char *buf)
{
    struct ps5169_redriver *redriver = container_of(c, struct ps5169_redriver, debug_class);

    return scnprintf(buf, PAGE_SIZE, "%d\n", redriver->usb_tx_eq);
}

static ssize_t debug_set_usb_tx_gain_store(struct class *c, struct class_attribute *attr, const char *buf, size_t count)
{
    struct ps5169_redriver *redriver = container_of(c, struct ps5169_redriver, debug_class);

    int val;

    if (kstrtoint(buf, 0, &val))
            return -EINVAL;

    redriver->usb_tx_gain = val;

    setting_usb_tx_gain(redriver);

    return count;
}
static CLASS_ATTR_RW(debug_set_usb_tx_gain);

static ssize_t debug_set_usb_rx_eq_show(struct class *c, struct class_attribute *attr, char *buf)
{
    struct ps5169_redriver *redriver = container_of(c, struct ps5169_redriver, debug_class);

    return scnprintf(buf, PAGE_SIZE, "%d\n", redriver->usb_rx_eq);
}

static ssize_t debug_set_usb_rx_eq_store(struct class *c, struct class_attribute *attr, const char *buf, size_t count)
{
    struct ps5169_redriver *redriver = container_of(c, struct ps5169_redriver, debug_class);

    int val;

    if (kstrtoint(buf, 0, &val))
            return -EINVAL;

    redriver->usb_rx_eq = val;

    setting_usb_rx_eq(redriver);

    return count;
}
static CLASS_ATTR_RW(debug_set_usb_rx_eq);

static ssize_t debug_set_usb_rx_gain_show(struct class *c, struct class_attribute *attr, char *buf)
{
    struct ps5169_redriver *redriver = container_of(c, struct ps5169_redriver, debug_class);

    return scnprintf(buf, PAGE_SIZE, "%d\n", redriver->usb_rx_eq);
}

static ssize_t debug_set_usb_rx_gain_store(struct class *c, struct class_attribute *attr, const char *buf, size_t count)
{
    struct ps5169_redriver *redriver = container_of(c, struct ps5169_redriver, debug_class);

    int val;

    if (kstrtoint(buf, 0, &val))
            return -EINVAL;

    redriver->usb_rx_gain = val;

    setting_usb_rx_gain(redriver);

    return count;
}
static CLASS_ATTR_RW(debug_set_usb_rx_gain);

static struct attribute *debug_node_class_attrs[] = {
    &class_attr_debug_set_dp_eq.attr,
    &class_attr_debug_set_dp_gain.attr,
    &class_attr_debug_set_usb_tx_eq.attr,
    &class_attr_debug_set_usb_tx_gain.attr,
    &class_attr_debug_set_usb_rx_eq.attr,
    &class_attr_debug_set_usb_rx_gain.attr,
    NULL,
};
ATTRIBUTE_GROUPS(debug_node_class);

static void ps5169_gpio_init(struct ps5169_redriver *redriver)
{
    struct device *dev = redriver->dev;
    int rc;

    redriver->en_gpio = of_get_gpio(dev->of_node, 0);
    if (!gpio_is_valid(redriver->en_gpio)) {
        dev_err(dev, "failed to get gpio\n");
        return;
    }

    rc = devm_gpio_request(dev, redriver->en_gpio, "redriver_en");
    if (rc < 0) {
        dev_err(dev, "failed to request gpio\n");
        redriver->en_gpio = -EINVAL;
        return;
    }

    redriver->orientation_gpio = of_get_named_gpio(dev->of_node, "usb_cc_dir_gpio", 0);
    if (!gpio_is_valid(redriver->orientation_gpio)) {
        dev_err(dev, "failed to get gpio\n");
        return;
    }

    rc = devm_gpio_request(dev, redriver->orientation_gpio, "usb_cc_dir");
    if (rc < 0) {
        dev_err(dev, "failed to request gpio\n");
        redriver->orientation_gpio = -EINVAL;
        return;
    }

    redriver->r.has_orientation = true;

    dev_err(dev, "en_gpio = %d,usb_cc_dir = %d\n", redriver->en_gpio, redriver->orientation_gpio);
}

static int ps5169_read_configuration(struct ps5169_redriver *redriver)
{
    struct device_node *node = redriver->dev->of_node;
    int ret = 0;

    if (of_find_property(node, "dp_eq", NULL)) {
        ret = of_property_read_u8(node, "dp_eq",
                        &redriver->dp_eq);
        if (ret)
            goto err;
    }

    if (of_find_property(node, "dp_gain", NULL)) {
        ret = of_property_read_u8(node, "dp_gain",
                        &redriver->dp_gain);
        if (ret)
            goto err;
    }

    if (of_find_property(node, "usb_tx_eq", NULL)) {
        ret = of_property_read_u8(node, "usb_tx_eq",
                        &redriver->usb_tx_eq);
        if (ret)
            goto err;
    }

    if (of_find_property(node, "usb_tx_gain", NULL)) {
        ret = of_property_read_u8(node, "usb_tx_gain",
                        &redriver->usb_tx_gain);
        if (ret)
            goto err;
    }

    if (of_find_property(node, "usb_rx_eq", NULL)) {
        ret = of_property_read_u8(node, "usb_rx_eq",
                        &redriver->usb_rx_eq);
        if (ret)
            goto err;
    }

    if (of_find_property(node, "usb_rx_gain", NULL)) {
        ret = of_property_read_u8(node, "usb_rx_gain",
                        &redriver->usb_rx_gain);
        if (ret)
            goto err;
    }

    dev_err(redriver->dev,"dp_eq=%d,dp_gain=%d,usb_tx_eq=%d,usb_tx_gain=%d,usb_rx_eq=%d,usb_rx_gain=%d\n",
       redriver->dp_eq, redriver->dp_gain, redriver->usb_tx_eq, redriver->usb_tx_gain, redriver->usb_rx_eq, redriver->usb_rx_gain);

    return 0;

err:
    dev_err(redriver->dev,
                "%s: error read parameters.\n", __func__);
    return ret;
}

static void setting_dp_eq(struct ps5169_redriver *redriver)
{
    switch (redriver->dp_eq)
    {
        case LEVEL_1:
            ps5169_masked_write(redriver, GEN_MODE_DP_EQ0, (BIT_4|BIT_5|BIT_6), 0x00);
            ps5169_masked_write(redriver, GEN_MODE_DP_EQ1, (BIT_0|BIT_1|BIT_2), 0x04);
            break;
        case LEVEL_2:
            ps5169_masked_write(redriver, GEN_MODE_DP_EQ0, (BIT_4|BIT_5|BIT_6), 0x10);
            ps5169_masked_write(redriver, GEN_MODE_DP_EQ1, (BIT_0|BIT_1|BIT_2), 0x05);
            break;
        case LEVEL_3:
            ps5169_masked_write(redriver, GEN_MODE_DP_EQ0, (BIT_4|BIT_5|BIT_6), 0x20);
            ps5169_masked_write(redriver, GEN_MODE_DP_EQ1, (BIT_0|BIT_1|BIT_2), 0x06);
            break;
        case LEVEL_4:
            ps5169_masked_write(redriver, GEN_MODE_DP_EQ0, (BIT_4|BIT_5|BIT_6), 0x30);
            ps5169_masked_write(redriver, GEN_MODE_DP_EQ1, (BIT_0|BIT_1|BIT_2), 0x06);
            break;
        case LEVEL_5:
            ps5169_masked_write(redriver, GEN_MODE_DP_EQ0, (BIT_4|BIT_5|BIT_6), 0x40);
            ps5169_masked_write(redriver, GEN_MODE_DP_EQ1, (BIT_0|BIT_1|BIT_2), 0x06);
            break;
        case LEVEL_6:
            ps5169_masked_write(redriver, GEN_MODE_DP_EQ0, (BIT_4|BIT_5|BIT_6), 0x50);
            ps5169_masked_write(redriver, GEN_MODE_DP_EQ1, (BIT_0|BIT_1|BIT_2), 0x07);
            break;
        case LEVEL_7:
            ps5169_masked_write(redriver, GEN_MODE_DP_EQ0, (BIT_4|BIT_5|BIT_6), 0x60);
            ps5169_masked_write(redriver, GEN_MODE_DP_EQ1, (BIT_0|BIT_1|BIT_2), 0x07);
            break;
        default:
            dev_err(redriver->dev, "dp eq level error\n");
            break;
    }
    dev_err(redriver->dev, "dp eq level = %d\n", redriver->dp_eq);
}

static void setting_dp_gain(struct ps5169_redriver *redriver)
{
    switch (redriver->dp_gain)
    {
        case LEVEL_1:
            ps5169_masked_write(redriver, GEN_MODE_DP_GAIN_REG, BIT_4, 0x00);
            break;
        case LEVEL_2:
            ps5169_masked_write(redriver, GEN_MODE_DP_GAIN_REG, BIT_4, 0x10);
            break;
        default:
            dev_err(redriver->dev, "dp eq gain level error\n");
            break;
    }
    dev_err(redriver->dev, "dp eq gain level = %d\n", redriver->dp_gain);
}

static void setting_usb_tx_eq(struct ps5169_redriver *redriver)
{
    switch (redriver->usb_tx_eq)
    {
        case LEVEL_1:
            ps5169_masked_write(redriver, GEN_MODE_USB_TX_EQ0_REG, (BIT_4|BIT_5|BIT_6), 0x00);
            ps5169_masked_write(redriver, GEN_MODE_USB_TX_EQ1_REG, (BIT_4|BIT_5|BIT_6), 0x04);
            ps5169_masked_write(redriver, GEN_MODE_USB_TX_EQ2_REG, (BIT_4|BIT_5|BIT_6|BIT_7), 0x00);
            break;
        case LEVEL_2:
            ps5169_masked_write(redriver, GEN_MODE_USB_TX_EQ0_REG, (BIT_4|BIT_5|BIT_6), 0x01);
            ps5169_masked_write(redriver, GEN_MODE_USB_TX_EQ1_REG, (BIT_4|BIT_5|BIT_6), 0x05);
            ps5169_masked_write(redriver, GEN_MODE_USB_TX_EQ2_REG, (BIT_4|BIT_5|BIT_6|BIT_7), 0x10);
            break;
        case LEVEL_3:
            ps5169_masked_write(redriver, GEN_MODE_USB_TX_EQ0_REG, (BIT_4|BIT_5|BIT_6), 0x02);
            ps5169_masked_write(redriver, GEN_MODE_USB_TX_EQ1_REG, (BIT_4|BIT_5|BIT_6), 0x06);
            ps5169_masked_write(redriver, GEN_MODE_USB_TX_EQ2_REG, (BIT_4|BIT_5|BIT_6|BIT_7), 0x10);
            break;
        case LEVEL_4:
            ps5169_masked_write(redriver, GEN_MODE_USB_TX_EQ0_REG, (BIT_4|BIT_5|BIT_6), 0x03);
            ps5169_masked_write(redriver, GEN_MODE_USB_TX_EQ1_REG, (BIT_4|BIT_5|BIT_6), 0x06);
            ps5169_masked_write(redriver, GEN_MODE_USB_TX_EQ2_REG, (BIT_4|BIT_5|BIT_6|BIT_7), 0x50);
            break;
        case LEVEL_5:
            ps5169_masked_write(redriver, GEN_MODE_USB_TX_EQ0_REG, (BIT_4|BIT_5|BIT_6), 0x04);
            ps5169_masked_write(redriver, GEN_MODE_USB_TX_EQ1_REG, (BIT_4|BIT_5|BIT_6), 0x06);
            ps5169_masked_write(redriver, GEN_MODE_USB_TX_EQ2_REG, (BIT_4|BIT_5|BIT_6|BIT_7), 0xC0);
            break;
        case LEVEL_6:
            ps5169_masked_write(redriver, GEN_MODE_USB_TX_EQ0_REG, (BIT_4|BIT_5|BIT_6), 0x06);
            ps5169_masked_write(redriver, GEN_MODE_USB_TX_EQ1_REG, (BIT_4|BIT_5|BIT_6), 0x07);
            ps5169_masked_write(redriver, GEN_MODE_USB_TX_EQ2_REG, (BIT_4|BIT_5|BIT_6|BIT_7), 0x50);
            break;
        case LEVEL_7:
            ps5169_masked_write(redriver, GEN_MODE_USB_TX_EQ0_REG, (BIT_4|BIT_5|BIT_6), 0x06);
            ps5169_masked_write(redriver, GEN_MODE_USB_TX_EQ1_REG, (BIT_4|BIT_5|BIT_6), 0x07);
            ps5169_masked_write(redriver, GEN_MODE_USB_TX_EQ2_REG, (BIT_4|BIT_5|BIT_6|BIT_7), 0xF0);
            break;
        case LEVEL_8:
            ps5169_masked_write(redriver, GEN_MODE_USB_TX_EQ0_REG, (BIT_4|BIT_5|BIT_6), 0x07);
            ps5169_masked_write(redriver, GEN_MODE_USB_TX_EQ1_REG, (BIT_4|BIT_5|BIT_6), 0x07);
            ps5169_masked_write(redriver, GEN_MODE_USB_TX_EQ2_REG, (BIT_4|BIT_5|BIT_6|BIT_7), 0xF0);
            break;
        default:
            dev_err(redriver->dev, "usb tx eq level error\n");
            break;
    }
    dev_err(redriver->dev, "usb tx eq level = %d\n", redriver->usb_tx_eq);
}

static void setting_usb_tx_gain(struct ps5169_redriver *redriver)
{
    switch (redriver->usb_tx_gain)
    {
        case LEVEL_1:
            ps5169_masked_write(redriver, GEN_MODE_DP_GAIN_REG, BIT_0, 0x00);
            break;
        case LEVEL_2:
            ps5169_masked_write(redriver, GEN_MODE_DP_GAIN_REG, BIT_0, 0x01);
            break;
        default:
            dev_err(redriver->dev, "usb tx gain level error\n");
            break;
    }
    dev_err(redriver->dev, "usb tx gain level = %d\n", redriver->usb_tx_gain);
}

static void setting_usb_rx_eq(struct ps5169_redriver *redriver)
{
    switch (redriver->usb_rx_eq)
    {
        case LEVEL_1:
            ps5169_masked_write(redriver, GEN_MODE_USB_RX_RS_REG, (BIT_7|BIT_6|BIT_5|BIT_4|BIT_2|BIT_1), 0x86);
            ps5169_masked_write(redriver, GEN_MODE_USB_RX_DCCM_REG, (BIT_7|BIT_6|BIT_5|BIT_4), 0x00);
            ps5169_masked_write(redriver, GEN_MODE_USB_RX_ITAIL_REG, (BIT_7|BIT_6|BIT_5|BIT_0), 0x20);
            ps5169_masked_write(redriver, GEN_MODE_USB_TX_EQ2_REG, (BIT_3|BIT_2|BIT_1|BIT_0), 0x00);
            break;
        case LEVEL_2:
            ps5169_masked_write(redriver, GEN_MODE_USB_RX_RS_REG, (BIT_7|BIT_6|BIT_5|BIT_4|BIT_2|BIT_1), 0x96);
            ps5169_masked_write(redriver, GEN_MODE_USB_RX_DCCM_REG, (BIT_7|BIT_6|BIT_5|BIT_4), 0x00);
            ps5169_masked_write(redriver, GEN_MODE_USB_RX_ITAIL_REG, (BIT_7|BIT_6|BIT_5|BIT_0), 0x20);
            ps5169_masked_write(redriver, GEN_MODE_USB_TX_EQ2_REG, (BIT_3|BIT_2|BIT_1|BIT_0), 0x01);
            break;
        case LEVEL_3:
            ps5169_masked_write(redriver, GEN_MODE_USB_RX_RS_REG, (BIT_7|BIT_6|BIT_5|BIT_4|BIT_2|BIT_1), 0xA6);
            ps5169_masked_write(redriver, GEN_MODE_USB_RX_DCCM_REG, (BIT_7|BIT_6|BIT_5|BIT_4), 0x50);
            ps5169_masked_write(redriver, GEN_MODE_USB_RX_ITAIL_REG, (BIT_7|BIT_6|BIT_5|BIT_0), 0x40);
            ps5169_masked_write(redriver, GEN_MODE_USB_TX_EQ2_REG, (BIT_3|BIT_2|BIT_1|BIT_0), 0x01);
            break;
        case LEVEL_4:
            ps5169_masked_write(redriver, GEN_MODE_USB_RX_RS_REG, (BIT_7|BIT_6|BIT_5|BIT_4|BIT_2|BIT_1), 0xB6);
            ps5169_masked_write(redriver, GEN_MODE_USB_RX_DCCM_REG, (BIT_7|BIT_6|BIT_5|BIT_4), 0x50);
            ps5169_masked_write(redriver, GEN_MODE_USB_RX_ITAIL_REG, (BIT_7|BIT_6|BIT_5|BIT_0), 0x40);
            ps5169_masked_write(redriver, GEN_MODE_USB_TX_EQ2_REG, (BIT_3|BIT_2|BIT_1|BIT_0), 0x05);
            break;
        case LEVEL_5:
            ps5169_masked_write(redriver, GEN_MODE_USB_RX_RS_REG, (BIT_7|BIT_6|BIT_5|BIT_4|BIT_2|BIT_1), 0xC6);
            ps5169_masked_write(redriver, GEN_MODE_USB_RX_DCCM_REG, (BIT_7|BIT_6|BIT_5|BIT_4), 0xB0);
            ps5169_masked_write(redriver, GEN_MODE_USB_RX_ITAIL_REG, (BIT_7|BIT_6|BIT_5|BIT_0), 0x80);
            ps5169_masked_write(redriver, GEN_MODE_USB_TX_EQ2_REG, (BIT_3|BIT_2|BIT_1|BIT_0), 0x0C);
            break;
        case LEVEL_6:
            ps5169_masked_write(redriver, GEN_MODE_USB_RX_RS_REG, (BIT_7|BIT_6|BIT_5|BIT_4|BIT_2|BIT_1), 0xD6);
            ps5169_masked_write(redriver, GEN_MODE_USB_RX_DCCM_REG, (BIT_7|BIT_6|BIT_5|BIT_4), 0xF0);
            ps5169_masked_write(redriver, GEN_MODE_USB_RX_ITAIL_REG, (BIT_7|BIT_6|BIT_5|BIT_0), 0x80);
            ps5169_masked_write(redriver, GEN_MODE_USB_TX_EQ2_REG, (BIT_3|BIT_2|BIT_1|BIT_0), 0x05);
            break;
        case LEVEL_7:
            ps5169_masked_write(redriver, GEN_MODE_USB_RX_RS_REG, (BIT_7|BIT_6|BIT_5|BIT_4|BIT_2|BIT_1), 0xE6);
            ps5169_masked_write(redriver, GEN_MODE_USB_RX_DCCM_REG, (BIT_7|BIT_6|BIT_5|BIT_4), 0xF0);
            ps5169_masked_write(redriver, GEN_MODE_USB_RX_ITAIL_REG, (BIT_7|BIT_6|BIT_5|BIT_0), 0x80);
            ps5169_masked_write(redriver, GEN_MODE_USB_TX_EQ2_REG, (BIT_3|BIT_2|BIT_1|BIT_0), 0x0F);
            break;
        case LEVEL_8:
            ps5169_masked_write(redriver, GEN_MODE_USB_RX_RS_REG, (BIT_7|BIT_6|BIT_5|BIT_4|BIT_2|BIT_1), 0xF6);
            ps5169_masked_write(redriver, GEN_MODE_USB_RX_DCCM_REG, (BIT_7|BIT_6|BIT_5|BIT_4), 0x30);
            ps5169_masked_write(redriver, GEN_MODE_USB_RX_ITAIL_REG, (BIT_7|BIT_6|BIT_5|BIT_0), 0xA1);
            ps5169_masked_write(redriver, GEN_MODE_USB_TX_EQ2_REG, (BIT_3|BIT_2|BIT_1|BIT_0), 0x0F);
            break;
        default:
            dev_err(redriver->dev, "usb rx eq level error\n");
            break;
    }
    dev_err(redriver->dev, "usb rx eq level = %d\n", redriver->usb_rx_eq);
}

static void setting_usb_rx_gain(struct ps5169_redriver *redriver)
{
    switch (redriver->usb_rx_gain)
    {
        case LEVEL_1:
            ps5169_masked_write(redriver, GEN_MODE_DP_GAIN_REG, BIT_2, 0x00);
            break;
        case LEVEL_2:
            ps5169_masked_write(redriver, GEN_MODE_DP_GAIN_REG, BIT_2, 0x01);
            break;
        default:
            dev_err(redriver->dev, "usb rx gain level error\n");
            break;
    }
    dev_err(redriver->dev, "usb rx gain level = %d\n", redriver->usb_rx_gain);
}

static int ps5169_chip_init(struct ps5169_redriver *redriver)
{
    ps5169_reg_set(redriver, 0x9d, 0x80);
    msleep(10);
    ps5169_reg_set(redriver, 0x9D, 0x00);
    ps5169_reg_set(redriver, 0x40, 0x80);   //auto power down
    ps5169_reg_set(redriver, 0x04, 0x44);   //disable U1 status RX_Det
    ps5169_reg_set(redriver, 0xA0, 0x02);   //disable AUX channel
    ps5169_reg_set(redriver, 0x51, 0x87);
    ps5169_reg_set(redriver, 0x50, 0x20);
    ps5169_reg_set(redriver, 0x54, 0x11);
    ps5169_reg_set(redriver, 0x5D, 0x66);
    ps5169_reg_set(redriver, 0x52, 0x50);   //DP EQ 8.5dB
    ps5169_reg_set(redriver, 0x55, 0x00);
    ps5169_reg_set(redriver, 0x56, 0x00);
    ps5169_reg_set(redriver, 0x57, 0x00);
    ps5169_reg_set(redriver, 0x58, 0x00);
    ps5169_reg_set(redriver, 0x59, 0x00);
    ps5169_reg_set(redriver, 0x5A, 0x00);
    ps5169_reg_set(redriver, 0x5B, 0x00);
    ps5169_reg_set(redriver, 0x5E, 0x06);
    ps5169_reg_set(redriver, 0x5F, 0x00);
    ps5169_reg_set(redriver, 0x60, 0x00);
    ps5169_reg_set(redriver, 0x61, 0x03);
    ps5169_reg_set(redriver, 0x65, 0x40);
    ps5169_reg_set(redriver, 0x66, 0x00);
    ps5169_reg_set(redriver, 0x67, 0x03);
    ps5169_reg_set(redriver, 0x75, 0x0C);
    ps5169_reg_set(redriver, 0x77, 0x00);
    ps5169_reg_set(redriver, 0x78, 0x7C);

    setting_dp_eq(redriver);
    setting_dp_gain(redriver);
    setting_usb_tx_eq(redriver);
    setting_usb_tx_gain(redriver);
    setting_usb_rx_eq(redriver);
    setting_usb_rx_gain(redriver);

    return 0;
}

static void DP_MODE_FLIP(struct ps5169_redriver *redriver)
{
    if (redriver->typec_orientation) {
        ps5169_reg_set(redriver, 0x40, 0xB0);
    } else {
        ps5169_reg_set(redriver, 0x40, 0xA0);
    }
    ps5169_reg_set(redriver, 0xA0, 0x00);
    ps5169_reg_set(redriver, 0xA1, 0x04);
}

static void USB_MODE_FLIP(struct ps5169_redriver *redriver)
{
    if (redriver->typec_orientation) {
        ps5169_reg_set(redriver, 0x40, 0xD0);
    } else {
        ps5169_reg_set(redriver, 0x40, 0xC0);
    }
    ps5169_reg_set(redriver, 0x8D, 0x01);
    ps5169_reg_set(redriver, 0x90, 0x01);
}

static void DP_AND_USB_MODE_FLIP(struct ps5169_redriver *redriver)
{
    if (redriver->typec_orientation) {
        ps5169_reg_set(redriver, 0x40, 0xF0);
    } else {
        ps5169_reg_set(redriver, 0x40, 0xE0);
    }
    ps5169_reg_set(redriver, 0x8D, 0x01);
    ps5169_reg_set(redriver, 0x90, 0x01);
    ps5169_reg_set(redriver, 0xA0, 0x00);
    ps5169_reg_set(redriver, 0xA1, 0x04);
}

static void DP_MODE_CLEAN(struct ps5169_redriver *redriver)
{
    ps5169_reg_set(redriver, 0x40, 0x80);   //auto power down
    ps5169_reg_set(redriver, 0xA0, 0x02);   //Disable AUX channel
    ps5169_reg_set(redriver, 0xA1, 0x00);   //HPD low
}

static void USB_MODE_CLEAN(struct ps5169_redriver *redriver)
{
    ps5169_reg_set(redriver, 0x40, 0x80);   //auto power down
    ps5169_reg_set(redriver, 0x8D, 0x00);   //RX1 50ohm termination off
    ps5169_reg_set(redriver, 0x90, 0x00);   //RX2 50ohm termination off
}

static void DP_AND_USB_MODE_CLEAN(struct ps5169_redriver *redriver)
{
    ps5169_reg_set(redriver, 0x40, 0x80);   //auto power down
    ps5169_reg_set(redriver, 0xA0, 0x02);   //Disable AUX channel
    ps5169_reg_set(redriver, 0x8D, 0x00);   //RX1 50ohm termination off
    ps5169_reg_set(redriver, 0x90, 0x00);   //RX2 50ohm termination off
    ps5169_reg_set(redriver, 0xA1, 0x00);   //HPD low

}

static int ps5169_mode_dev_set(struct ps5169_redriver *redriver)
{
    u8 val;

    switch (redriver->op_mode)
    {
        case OP_MODE_DP:
            DP_MODE_FLIP(redriver);
            break;
        case OP_MODE_USB:
            USB_MODE_FLIP(redriver);
            break;
        case OP_MODE_DP_AND_USB:
            DP_AND_USB_MODE_FLIP(redriver);
            break;
        case OP_MODE_NONE:
            DP_AND_USB_MODE_CLEAN(redriver);
            break;
        default:
            dev_err(redriver->dev, "dev set op mode error\n");
            break;
    }

    ps5169_reg_get(redriver, 0x40, &val);

    dev_err(redriver->dev, "op mode = %s,typc_orientation = %d, 0x40:0x%02x\n", OPMODESTR(redriver->op_mode), redriver->typec_orientation, val);

    return 0;
}

static void ps5169_mode_dev_clean(struct ps5169_redriver *redriver)
{
    switch (redriver->op_mode)
    {
        case OP_MODE_DP:
            DP_MODE_CLEAN(redriver);
            break;
        case OP_MODE_USB:
            USB_MODE_CLEAN(redriver);
            break;
        case OP_MODE_DP_AND_USB:
            DP_AND_USB_MODE_CLEAN(redriver);
            break;
        case OP_MODE_NONE:
            DP_MODE_CLEAN(redriver);
            USB_MODE_CLEAN(redriver);
            DP_AND_USB_MODE_CLEAN(redriver);
            break;
        default:
            dev_err(redriver->dev, "op mode = %s\n", OPMODESTR(redriver->op_mode));
            break;
    }
}

static const struct regmap_config redriver_regmap = {
    .reg_bits = 8,
    .val_bits = 8,
};

static inline void orientation_set(struct ps5169_redriver *redriver, int ort)
{
    redriver->typec_orientation = ort;

    dev_dbg(redriver->dev, "%s: mode:%s, orientation:%s\n", __func__,
        OPMODESTR(redriver->op_mode), ort == ORIENTATION_CC1 ? "CC1" : "CC2");

    if (redriver->typec_orientation == ORIENTATION_CC2) {
        redriver->typec_orientation = ORIENTATION_CC2;
    } else {
        redriver->typec_orientation = ORIENTATION_CC1;
    }

    dev_err(redriver->dev, "%s:typec_orientation = %d\n", __func__, redriver->typec_orientation);
}

static int ps5169_release_usb_lanes(struct usb_redriver *r, int ort, int num)
{
    struct ps5169_redriver *redriver = container_of(r, struct ps5169_redriver, r);

    dev_dbg(redriver->dev, "%s: mode %s, orientation %s, lanes %d\n", __func__,
        OPMODESTR(redriver->op_mode), ort == ORIENTATION_CC1 ? "CC1" : "CC2", num);

    if (num == LANES_DP)
        redriver->op_mode = OP_MODE_DP;
    else if (num == LANES_DP_AND_USB)
        redriver->op_mode = OP_MODE_DP_AND_USB;

    orientation_set(redriver, ort);

    ps5169_mode_dev_set(redriver);

    return 0;
}

static int ps5169_notify_connect(struct usb_redriver *r, int ort)
{
    struct ps5169_redriver *redriver = container_of(r, struct ps5169_redriver, r);

    dev_dbg(redriver->dev, "%s: mode %s, orientation %s, %d\n", __func__,
            OPMODESTR(redriver->op_mode),
            ort == ORIENTATION_CC1 ? "CC1" : "CC2");

    if (redriver->op_mode == OP_MODE_NONE)
        redriver->op_mode = OP_MODE_USB;

    orientation_set(redriver, ort);

    ps5169_mode_dev_set(redriver);

    return 0;
}

static int ps5169_notify_disconnect(struct usb_redriver *r)
{
    struct ps5169_redriver *redriver = container_of(r, struct ps5169_redriver, r);

    dev_dbg(redriver->dev, "%s: mode %s\n", __func__,
        OPMODESTR(redriver->op_mode));

    if (redriver->op_mode == OP_MODE_NONE)
        return 0;

    redriver->op_mode = OP_MODE_NONE;

    ps5169_mode_dev_clean(redriver);

    return 0;
}

static int ps5169_get_orientation(struct usb_redriver *r)
{
    struct ps5169_redriver *redriver = container_of(r, struct ps5169_redriver, r);
    int orientation_gpio;

    orientation_gpio = gpio_get_value(redriver->orientation_gpio);

    dev_dbg(redriver->dev, "%s: mode %s, cc:%d\n", __func__,
        OPMODESTR(redriver->op_mode), orientation_gpio);

    return orientation_gpio;
}

static int ps5169_probe(struct i2c_client *client, const struct i2c_device_id *dev_id)
{
    struct ps5169_redriver *redriver;
    int ret;

    redriver = devm_kzalloc(&client->dev, sizeof(struct ps5169_redriver), GFP_KERNEL);
    if (!redriver)
        return -ENOMEM;

    redriver->regmap = devm_regmap_init_i2c(client, &redriver_regmap);
    if (IS_ERR(redriver->regmap)) {
        ret = PTR_ERR(redriver->regmap);
        dev_err(&client->dev,
            "Failed to allocate register map: %d\n", ret);
        goto error;
    }

    redriver->dev = &client->dev;
    i2c_set_clientdata(client, redriver);

    ps5169_gpio_init(redriver);

    if (gpio_is_valid(redriver->en_gpio)) {
        ret = gpio_direction_output(redriver->en_gpio, 1);
        if (ret < 0) {
            dev_err(&client->dev,
                "Failed to gpio output error: %d\n", ret);
            goto error;
        }
        gpio_set_value(redriver->en_gpio, 1);
    }

    ret = ps5169_read_configuration(redriver);
    if (ret < 0) {
        dev_err(&client->dev,
                "Failed to read default configuration: %d\n", ret);
        goto error;
    }

    ret = ps5169_chip_init(redriver);
    if (ret < 0) {
        dev_err(&client->dev,
                "Failed to int chip error: %d\n", ret);
        goto error;
    }

    redriver->op_mode = OP_MODE_NONE;
    ps5169_mode_dev_set(redriver);

    redriver->debug_class.name = "redriver_debug_node";
    redriver->debug_class.class_groups = debug_node_class_groups;
    ret = class_register(&redriver->debug_class);
    if (ret < 0) {
        dev_err(&client->dev,
                "debug create group failed: %d\n", ret);
        return ret;
    }

    redriver->r.of_node = redriver->dev->of_node;
    redriver->r.release_usb_lanes = ps5169_release_usb_lanes;
    redriver->r.notify_connect = ps5169_notify_connect;
    redriver->r.notify_disconnect = ps5169_notify_disconnect;
    redriver->r.get_orientation = ps5169_get_orientation;
    redriver->r.gadget_pullup_enter = NULL;
    redriver->r.gadget_pullup_exit = NULL;
    redriver->r.host_powercycle = NULL;
    usb_add_redriver(&redriver->r);

    pr_err("%s is success\n", __func__);

    return 0;

error:
    devm_kfree(&client->dev, redriver);
    redriver = NULL;
    return ret;
}

static int __maybe_unused ps5169_suspend(struct device *dev)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct ps5169_redriver *redriver = i2c_get_clientdata(client);

    dev_dbg(redriver->dev, "%s: SS USB redriver suspend.\n", __func__);

    if (redriver->op_mode == OP_MODE_DP ||
        redriver->op_mode == OP_MODE_NONE ||
        redriver->op_mode == OP_MODE_DEFAULT)
            return 0;

    ps5169_mode_dev_clean(redriver);

    return 0;
}

static int __maybe_unused ps5169_resume(struct device *dev)
{
    struct i2c_client *client = to_i2c_client(dev);
    struct ps5169_redriver *redriver = i2c_get_clientdata(client);

    dev_dbg(redriver->dev, "%s: SS USB redriver resume.\n", __func__);

    if (redriver->op_mode == OP_MODE_DP ||
        redriver->op_mode == OP_MODE_NONE ||
        redriver->op_mode == OP_MODE_DEFAULT)
            return 0;

    ps5169_mode_dev_set(redriver);

    return 0;
}

static SIMPLE_DEV_PM_OPS(ps5169_pm, ps5169_suspend,
                        ps5169_resume);

static int ps5169_remove(struct i2c_client *client)
{
    struct ps5169_redriver *redriver = i2c_get_clientdata(client);

    if (usb_remove_redriver(&redriver->r))
        return -EINVAL;

    class_unregister(&redriver->debug_class);

    return 0;
}

static void ps5169_shutdown(struct i2c_client *client)
{
    struct ps5169_redriver *redriver = i2c_get_clientdata(client);

    int ret;

    if (gpio_is_valid(redriver->en_gpio)) {
        ret = gpio_direction_output(redriver->en_gpio, 0);
        if (ret < 0) {
            dev_err(&client->dev,
                "Failed to gpio output error: %d\n", ret);
        }
        gpio_set_value(redriver->en_gpio, 0);
    }
}

static const struct i2c_device_id redriver_id_teble[] = {
    {"usb-redriver", 0},
    { /* sentinel */}
};
MODULE_DEVICE_TABLE(i2c, redriver_id_teble);

static const struct of_device_id ps5169_match_table[] = {
    {.compatible = "usb,redriver"},
    { /* sentinel */ }
};

static struct i2c_driver ps5169_driver = {
    .driver = {
        .name = "usb-redriver",
        .of_match_table = ps5169_match_table,
        .pm = &ps5169_pm
    },
    .probe = ps5169_probe,
    .remove = ps5169_remove,
    .shutdown = ps5169_shutdown,
    .id_table = redriver_id_teble,
};

module_i2c_driver(ps5169_driver);

MODULE_DESCRIPTION("USB Super Speed Linear Re-Driver");
MODULE_LICENSE("GPL v2");
