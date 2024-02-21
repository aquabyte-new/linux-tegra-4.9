// SPDX-License-Identifier: GPL-2.0-only
/*
 * Aquablue I2C virtual multiplexer
 *
 * Copyright (C) 2024 Aquabyte Inc
 *
 * Based on: i2c-mux-aquablue_mux.c
 *
 * Protocol: https://docs.google.com/document/d/1Q2I2aDl68YrjKM8ehA6bF7av-oBXX_kBrosqSAmb3Qk/edit
 */

#include <linux/gpio/consumer.h>
#include <linux/gpio/driver.h>
#include <linux/i2c-mux.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/slab.h>

#define AQUABLUE_MUX_MAX_CHANNELS	16

#define AQUABLUE_MUX_REG_MAGIC		0x0
#define AQUABLUE_MUX_REG_CONFIG		0x1
#define AQUABLUE_MUX_REG_COUNT		0x2
#define AQUABLUE_MUX_REG_SWITCH		0x3

#define AQUABLUE_MUX_MAGIC_NUMBER	0xab

struct aquablue_mux {
	struct i2c_client *client;
	struct regmap *regmap;

	struct gpio_desc *reset_gpio;
};

// =====================================================================

static void aquablue_reset(struct aquablue_mux *mux)
{
	gpiod_set_value(mux->reset_gpio, 1);
	mdelay(3);
	gpiod_set_value(mux->reset_gpio, 0);
	mdelay(50);

	dev_info(&mux->client->dev, "reset\n");
}

// =====================================================================

static bool aquablue_mux_is_volatile_reg(struct device *dev, unsigned int reg)
{
	return reg != AQUABLUE_MUX_REG_SWITCH;
}

static const struct regmap_config aquablue_mux_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = AQUABLUE_MUX_REG_SWITCH,
	.volatile_reg = aquablue_mux_is_volatile_reg,
	.cache_type = REGCACHE_FLAT,
};

static int aquablue_mux_select_mux(struct i2c_mux_core *muxc, u32 chan)
{
	struct aquablue_mux *data = i2c_mux_priv(muxc);

	return regmap_write(data->regmap, AQUABLUE_MUX_REG_SWITCH, chan);
}

static const struct i2c_device_id aquablue_mux_id[] = {
	{ "i2c-mux-aquablue", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, aquablue_mux_id);

static const struct of_device_id aquablue_mux_of_match[] = {
	{ .compatible = "aquabyte,i2c-mux-aquablue" },
	{ }
};
MODULE_DEVICE_TABLE(of, aquablue_mux_of_match);

#if 0
static int aquablue_mux_probe(struct i2c_client *client)
#else
static int aquablue_mux_probe(struct i2c_client *client, const struct i2c_device_id *id)
#endif
{
	struct i2c_adapter *adap = client->adapter;
	struct i2c_mux_core *muxc;
	struct aquablue_mux *data;
	unsigned int magic, count;
	int num, ret;
/* 
	const struct chip_desc *chip;
	chip = of_device_get_match_data(&client->dev);
	if (!chip) {
//		chip = &chips[i2c_match_id(aquablue_mux_id, client)->driver_data];
		dev_warn(&client->dev, "chip not found\n");
//		return -ENODEV;
	}
*/

	muxc = i2c_mux_alloc(adap, &client->dev,
			     AQUABLUE_MUX_MAX_CHANNELS, sizeof(*data), I2C_MUX_LOCKED,
			     aquablue_mux_select_mux, NULL);
	if (!muxc)
		return -ENOMEM;
	data = i2c_mux_priv(muxc);
	//data->chip = chip;
	data->client = client;

	data->reset_gpio = devm_gpiod_get(&client->dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(data->reset_gpio)) {
		dev_err(&client->dev, "unable to find reset gpio");
		return PTR_ERR(data->reset_gpio);
	}
	gpiod_direction_output(data->reset_gpio, 0);

	aquablue_reset(data);

	i2c_set_clientdata(client, muxc);

	data->regmap = devm_regmap_init_i2c(client, &aquablue_mux_regmap_config);
	if (IS_ERR(data->regmap)) {
		ret = PTR_ERR(data->regmap);
		dev_err(&client->dev, "Failed to allocate register map: %d\n",
			ret);
		return ret;
	}

	if (regmap_read(data->regmap, AQUABLUE_MUX_REG_MAGIC, &magic) < 0) {
		dev_err(&client->dev, "probe failed, invalid magic\n");
		return -ENODEV;
	}

	if (magic != AQUABLUE_MUX_MAGIC_NUMBER) {
		dev_info(&client->dev, "magic=%x\n", magic);
		return -ENODEV;
	}

	/*
	 * Read the count register to determine number of mux adapters
	 * that are present.
	 */
	if (regmap_read(data->regmap, AQUABLUE_MUX_REG_COUNT, &count) < 0) {
		dev_err(&client->dev, "probe failed, unable to get adapter count\n");
		return -ENODEV;
	}

	/* Now create an adapter for each channel */
	for (num = 0; num < count; num++) {
		ret = i2c_mux_add_adapter(muxc, 0, num, 0);
		if (ret) {
			i2c_mux_del_adapters(muxc);
			return ret;
		}
	}

	/* Switch to first channel */
	if (regmap_write(data->regmap, AQUABLUE_MUX_REG_SWITCH, 0) < 0) {
		dev_err(&client->dev, "probe failed, unable to switch\n");
		return -ENODEV;
	}

	dev_info(&client->dev,
		 "registered %d multiplexed busses for I2C switch %s\n",
		 count, client->name);

	return 0;
}

#if 0
static void aquablue_mux_remove(struct i2c_client *client)
#else
static int aquablue_mux_remove(struct i2c_client *client)
#endif
{
	struct i2c_mux_core *muxc = i2c_get_clientdata(client);

	i2c_mux_del_adapters(muxc);

	return 0;
}

static struct i2c_driver aquablue_mux_driver = {
	.driver		= {
		.name	= "i2c-mux-aquablue",
		.of_match_table = of_match_ptr(aquablue_mux_of_match),
	},
	.probe		= aquablue_mux_probe,
	.remove		= aquablue_mux_remove,
	.id_table	= aquablue_mux_id,
};

module_i2c_driver(aquablue_mux_driver);

MODULE_AUTHOR("Zachary T Welch <zach@aquabyte.ai>");
MODULE_DESCRIPTION("Aquabyte Aquablue virtual I2C multiplexer driver");
MODULE_LICENSE("GPL v2");
