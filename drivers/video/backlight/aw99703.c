// SPDX-License-Identifier: GPL-2.0-only
/*
 * Awinic AW99703 3-channel WLED backlight driver.
 *
 * The bootloader configures the chip (boost, full-scale current, channels,
 * ramps) and switches the backlight on. This driver takes that state over
 * without writing anything at probe time. Only an explicit write to the
 * backlight sysfs attributes (brightness, bl_power) changes the brightness
 * registers or the work mode; framebuffer blank events are ignored.
 *
 * Register layout from the Awinic/Motorola vendor driver leds_aw99703 v1.0.3
 * (Motorola kernel modules, "Copyright (c) 2019
 * AWINIC Technology CO., LTD", GPL-2.0-or-later). The code is a new
 * implementation on top of regmap and the backlight class.
 */

#include <linux/backlight.h>
#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/device.h>
#include <linux/i2c.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/regmap.h>

#define AW99703_REG_ID			0x00
#define AW99703_ID			0x03
#define AW99703_REG_MODE		0x02
#define AW99703_MODE_PDIS		BIT(4)
#define AW99703_MODE_MAP_LINEAR		BIT(2)
#define AW99703_MODE_WORKMODE		GENMASK(1, 0)
#define AW99703_WORKMODE_STANDBY	0
#define AW99703_WORKMODE_BACKLIGHT	1
#define AW99703_REG_LEDCUR		0x03
#define AW99703_LEDCUR_BLFS		GENMASK(7, 3)
#define AW99703_LEDCUR_CH		GENMASK(2, 0)
#define AW99703_REG_BSTCTR1		0x04
#define AW99703_REG_BSTCTR2		0x05
#define AW99703_REG_LEDLSB		0x06
#define AW99703_LEDLSB_MASK		GENMASK(2, 0)
#define AW99703_REG_LEDMSB		0x07
#define AW99703_REG_PWM			0x08
#define AW99703_REG_TURNCFG		0x09
#define AW99703_REG_TRANCFG		0x0a
#define AW99703_REG_FLASH		0x0b
#define AW99703_REG_AFHIGH		0x0c
#define AW99703_REG_AFLOW		0x0d
#define AW99703_REG_FLAGS1		0x0e
#define AW99703_REG_FLAGS2		0x0f
#define AW99703_REG_FLAGS3		0x11
#define AW99703_REG_AUTOZERO		0x21
#define AW99703_REG_PWMLSB		0x29
#define AW99703_REG_TEST		0x31
#define AW99703_REG_FLTDIS		0x33
#define AW99703_REG_EFUSE1		0x40
#define AW99703_REG_EFMODE		0x49
#define AW99703_REG_WPRT1		0x67
#define AW99703_REG_SCANEN		0x69

#define AW99703_MAX_BRIGHTNESS		2047

struct aw99703 {
	struct device *dev;
	struct regmap *regmap;
};

/* readable registers of the vendor access table (aw99703_reg_access[]) */
static bool aw99703_readable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case AW99703_REG_ID:
	case AW99703_REG_MODE ... AW99703_REG_FLAGS2:
	case AW99703_REG_FLAGS3:
	case AW99703_REG_AUTOZERO ... AW99703_REG_PWMLSB:
	case AW99703_REG_TEST:
	case AW99703_REG_FLTDIS:
	case AW99703_REG_EFUSE1 ... AW99703_REG_EFMODE:
	case AW99703_REG_WPRT1 ... AW99703_REG_SCANEN:
		return true;
	default:
		return false;
	}
}

/* the driver only ever writes the brightness and the work mode */
static bool aw99703_writeable_reg(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case AW99703_REG_MODE:
	case AW99703_REG_LEDLSB:
	case AW99703_REG_LEDMSB:
		return true;
	default:
		return false;
	}
}

static bool aw99703_volatile_reg(struct device *dev, unsigned int reg)
{
	return true;
}

static const struct regmap_config aw99703_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = AW99703_REG_SCANEN,
	.readable_reg = aw99703_readable_reg,
	.writeable_reg = aw99703_writeable_reg,
	.volatile_reg = aw99703_volatile_reg,
	.cache_type = REGCACHE_NONE,
};

static int aw99703_read_brightness(struct aw99703 *aw, unsigned int *brt)
{
	unsigned int lsb, msb;
	int ret;

	ret = regmap_read(aw->regmap, AW99703_REG_LEDLSB, &lsb);
	if (ret)
		return ret;
	ret = regmap_read(aw->regmap, AW99703_REG_LEDMSB, &msb);
	if (ret)
		return ret;

	*brt = (msb << 3) | (lsb & AW99703_LEDLSB_MASK);
	return 0;
}

static int aw99703_update_status(struct backlight_device *bd)
{
	struct aw99703 *aw = bl_get_data(bd);
	unsigned int brt = backlight_get_brightness(bd);
	int ret;

	if (!brt)
		return regmap_update_bits(aw->regmap, AW99703_REG_MODE,
					  AW99703_MODE_WORKMODE,
					  AW99703_WORKMODE_STANDBY);

	/* same order as the vendor driver: LSB, MSB, then work mode */
	ret = regmap_write(aw->regmap, AW99703_REG_LEDLSB, brt & AW99703_LEDLSB_MASK);
	if (ret)
		return ret;
	ret = regmap_write(aw->regmap, AW99703_REG_LEDMSB, brt >> 3);
	if (ret)
		return ret;
	return regmap_update_bits(aw->regmap, AW99703_REG_MODE,
				  AW99703_MODE_WORKMODE,
				  AW99703_WORKMODE_BACKLIGHT);
}

static int aw99703_get_brightness(struct backlight_device *bd)
{
	struct aw99703 *aw = bl_get_data(bd);
	unsigned int brt, mode;
	int ret;

	ret = regmap_read(aw->regmap, AW99703_REG_MODE, &mode);
	if (ret)
		return ret;
	if (FIELD_GET(AW99703_MODE_WORKMODE, mode) != AW99703_WORKMODE_BACKLIGHT)
		return 0;

	ret = aw99703_read_brightness(aw, &brt);
	return ret ? ret : brt;
}

/*
 * Not tied to any display device: framebuffer blank events must not switch
 * the backlight; only sysfs writes do.
 */
static bool aw99703_controls_device(struct backlight_device *bd,
				    struct device *display_dev)
{
	return false;
}

static const struct backlight_ops aw99703_bl_ops = {
	.update_status = aw99703_update_status,
	.get_brightness = aw99703_get_brightness,
	.controls_device = aw99703_controls_device,
};

static int aw99703_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct backlight_properties props = { };
	struct backlight_device *bd;
	unsigned int id, mode, ledcur, brt;
	struct aw99703 *aw;
	int ret;

	aw = devm_kzalloc(dev, sizeof(*aw), GFP_KERNEL);
	if (!aw)
		return -ENOMEM;
	aw->dev = dev;

	aw->regmap = devm_regmap_init_i2c(client, &aw99703_regmap_config);
	if (IS_ERR(aw->regmap))
		return dev_err_probe(dev, PTR_ERR(aw->regmap), "regmap init failed\n");

	ret = regmap_read(aw->regmap, AW99703_REG_ID, &id);
	if (ret)
		return dev_err_probe(dev, ret, "cannot read chip id\n");
	if (id != AW99703_ID)
		return dev_err_probe(dev, -ENODEV, "unexpected chip id 0x%02x\n", id);

	ret = regmap_read(aw->regmap, AW99703_REG_MODE, &mode);
	if (!ret)
		ret = regmap_read(aw->regmap, AW99703_REG_LEDCUR, &ledcur);
	if (!ret)
		ret = aw99703_read_brightness(aw, &brt);
	if (ret)
		return dev_err_probe(dev, ret, "cannot read bootloader state\n");

	/* take over what the bootloader left, write nothing */
	props.type = BACKLIGHT_RAW;
	props.max_brightness = AW99703_MAX_BRIGHTNESS;
	props.scale = (mode & AW99703_MODE_MAP_LINEAR) ?
		      BACKLIGHT_SCALE_LINEAR : BACKLIGHT_SCALE_NON_LINEAR;
	if (FIELD_GET(AW99703_MODE_WORKMODE, mode) == AW99703_WORKMODE_BACKLIGHT) {
		props.brightness = brt;
		props.power = BACKLIGHT_POWER_ON;
	} else {
		props.brightness = 0;
		props.power = BACKLIGHT_POWER_OFF;
	}

	bd = devm_backlight_device_register(dev, dev_name(dev), dev, aw,
					    &aw99703_bl_ops, &props);
	if (IS_ERR(bd))
		return dev_err_probe(dev, PTR_ERR(bd), "backlight register failed\n");

	dev_info(dev,
		 "AW99703 taken over: mode 0x%02x, ledcur 0x%02x (fs %lu, ch 0x%lx), brightness %u/%u; no register written\n",
		 mode, ledcur, FIELD_GET(AW99703_LEDCUR_BLFS, ledcur),
		 FIELD_GET(AW99703_LEDCUR_CH, ledcur), brt, AW99703_MAX_BRIGHTNESS);

	return 0;
}

static const struct of_device_id aw99703_of_match[] = {
	{ .compatible = "awinic,aw99703" },
	{ }
};
MODULE_DEVICE_TABLE(of, aw99703_of_match);

static const struct i2c_device_id aw99703_ids[] = {
	{ "aw99703" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, aw99703_ids);

static struct i2c_driver aw99703_driver = {
	.driver = {
		.name = "aw99703",
		.of_match_table = aw99703_of_match,
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
	},
	.probe = aw99703_probe,
	.id_table = aw99703_ids,
};
module_i2c_driver(aw99703_driver);

MODULE_DESCRIPTION("Awinic AW99703 WLED backlight driver (bootloader state takeover)");
MODULE_LICENSE("GPL");
