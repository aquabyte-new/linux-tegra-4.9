// gpio-aquablue.c: support for GPIO expansion via the Aquablue firmware
// Copyright 2023 Aquabyte, Inc.
// Derived from gpio-ab.c

#include <linux/delay.h>
//#include <linux/init.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/seq_file.h>
#include <linux/regmap.h>
#include <linux/slab.h>

#include <linux/gpio/consumer.h>
#include <linux/gpio/driver.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/of_irq.h>


#define AQUABLUE_GPIO_REG_DIR		0x00
#define AQUABLUE_GPIO_REG_READ		0x10
#define AQUABLUE_GPIO_REG_SET		0x20
#define AQUABLUE_GPIO_REG_CLR		0x30
#define AQUABLUE_GPIO_REG_IER		0x40
#define AQUABLUE_GPIO_REG_ISR		0x50

#define AQUABLUE_GPIO_NUM_REGISTERS	0x60

struct aquablue {
	// Control
	struct gpio_desc *irq_gpio;

	// I2C
	struct i2c_client *client;
	struct regmap *regmap;
	struct mutex i2c_lock;

	// GPIOs
	struct gpio_chip gpio;
	unsigned reg_shift;

	// GPIO interrupts
	struct mutex irq_lock;
	u8 *irq_enable;
	u8 *irq_level;
	u8 *irq_rise;
	u8 *irq_fall;
	u8 *irq_high;
	u8 *irq_low;
};

// =====================================================================

static bool aquablue_gpio_is_volatile_reg(struct device *dev, unsigned int reg)
{
	return reg != AQUABLUE_GPIO_REG_DIR;
}

static const struct regmap_config aquablue_gpio_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
	.max_register = AQUABLUE_GPIO_NUM_REGISTERS,
	.volatile_reg = aquablue_gpio_is_volatile_reg,
	.cache_type = REGCACHE_FLAT,
};

static int aquablue_read(struct aquablue *ab, unsigned offset, uint8_t *value)
{
	int err;
	unsigned int data;

	err = regmap_read(ab->regmap, offset, &data);
	if (err < 0) {
		dev_err(ab->gpio.parent, "%s failed: %d\n",
			"i2c_smbus_read_byte_data()", err);
		return err;
	}
	*value = data;

	return err;
}

static int aquablue_write(struct aquablue *ab, unsigned offset, uint8_t value)
{
	int err;

	err = regmap_write(ab->regmap, offset, value);
	if (err < 0) {
		dev_err(ab->gpio.parent, "%s failed: %d\n",
			"regmap_write()", err);
	}
	return err;
}

// =====================================================================

static int aquablue_gpio_get(struct gpio_chip *chip, unsigned offset)
{
	struct aquablue *ab = gpiochip_get_data(chip);
	unsigned int reg = offset >> ab->reg_shift;
	unsigned int pos = offset & 7;
	u8 value;
	int err;

	err = aquablue_read(ab, AQUABLUE_GPIO_REG_READ + reg, &value);
	if (err < 0)
		return err;

	return !!(value & BIT(pos));
}

static void aquablue_gpio_set(struct gpio_chip *chip, unsigned offset, int value)
{
	struct aquablue *ab = gpiochip_get_data(chip);
	unsigned int base = value ? AQUABLUE_GPIO_REG_SET : AQUABLUE_GPIO_REG_CLR;
	unsigned int reg = offset >> ab->reg_shift;
	unsigned int pos = offset & 7;

	aquablue_write(ab, base + reg, BIT(pos));
}

static int aquablue_gpio_direction_input(struct gpio_chip *chip, unsigned offset)
{
	struct aquablue *ab = gpiochip_get_data(chip);
	unsigned int reg = offset >> ab->reg_shift;
	unsigned int pos = offset & 7;
	u8 value;
	int err;

	mutex_lock(&ab->i2c_lock);

	err = aquablue_read(ab, AQUABLUE_GPIO_REG_DIR + reg, &value);
	if (err < 0)
		goto out;

	value |= BIT(pos);

	err = aquablue_write(ab, AQUABLUE_GPIO_REG_DIR + reg, value);
	if (err < 0)
		goto out;

	err = aquablue_read(ab, AQUABLUE_GPIO_REG_DIR + reg, &value);
	if (err < 0)
		goto out;

	if (!(value & BIT(pos))) {
		err = -EPERM;
		goto out;
	}

	err = 0;

out:
	mutex_unlock(&ab->i2c_lock);
	return err;
}

static int aquablue_gpio_direction_output(struct gpio_chip *chip, unsigned offset,
				      int value)
{
	struct aquablue *ab = gpiochip_get_data(chip);
	unsigned int reg = offset >> ab->reg_shift;
	unsigned int pos = offset & 7;
	int err;
	u8 val;

	mutex_lock(&ab->i2c_lock);

	err = aquablue_read(ab, AQUABLUE_GPIO_REG_DIR + reg, &val);
	if (err < 0)
		goto out;

	val &= ~BIT(pos);

	err = aquablue_write(ab, AQUABLUE_GPIO_REG_DIR + reg, val);
	if (err < 0)
		goto out;

	err = aquablue_read(ab, AQUABLUE_GPIO_REG_DIR + reg, &val);
	if (err < 0)
		goto out;

	if (val & BIT(pos)) {
		err = -EPERM;
		goto out;
	}

	aquablue_gpio_set(chip, offset, value);
	err = 0;

out:
	mutex_unlock(&ab->i2c_lock);
	return err;
}

static void aquablue_gpio_dbg_show(struct seq_file *s, struct gpio_chip *chip)
{
	struct aquablue *ab = gpiochip_get_data(chip);
	unsigned int num_regs = 1 << ab->reg_shift, i, j;
	int err;

	for (i = 0; i < num_regs; i++) {
		u8 dir, levels;
#if 0
		u8 ier, isr;
#endif

		mutex_lock(&ab->i2c_lock);

		err = aquablue_read(ab, AQUABLUE_GPIO_REG_DIR + i, &dir);
		if (err < 0) {
			mutex_unlock(&ab->i2c_lock);
			return;
		}

		err = aquablue_read(ab, AQUABLUE_GPIO_REG_READ + i, &levels);
		if (err < 0) {
			mutex_unlock(&ab->i2c_lock);
			return;
		}

#if 0
		err = aquablue_read(ab, AQUABLUE_GPIO_REG_IER + i, &ier);
		if (err < 0) {
			mutex_unlock(&ab->i2c_lock);
			return;
		}

		err = aquablue_read(ab, AQUABLUE_GPIO_REG_ISR + i, &isr);
		if (err < 0) {
			mutex_unlock(&ab->i2c_lock);
			return;
		}
#endif

		mutex_unlock(&ab->i2c_lock);

		for (j = 0; j < 8; j++) {
			unsigned int bit = (i << ab->reg_shift) + j;
			const char *direction = "out";
			const char *level = "lo";
#if 0
			const char *interrupt = "disabled";
			const char *pending = "";
#endif
			if (dir & BIT(j))
				direction = "in ";

			if (levels & BIT(j))
				level = "hi";
#if 0
			if (ier & BIT(j))
				interrupt = "enabled ";

			if (isr & BIT(j))
				pending = "pending";

			seq_printf(s, "%2u: %s %s IRQ %s %s\n", bit,
				   direction, level, interrupt, pending);
#else
			seq_printf(s, " gpio-%3u (%20s|%20s) %s %s\n",
					chip->base + bit, "", "", direction, level);
#endif
		}
	}
}


static int aquablue_gpio_setup(struct aquablue *ab, unsigned int num_gpios)
{
	struct gpio_chip *chip = &ab->gpio;
	int err;

	ab->reg_shift = get_count_order(num_gpios) - 3;

	chip->direction_input = aquablue_gpio_direction_input;
	chip->direction_output = aquablue_gpio_direction_output;
	chip->get = aquablue_gpio_get;
	chip->set = aquablue_gpio_set;
	chip->can_sleep = true;

	if (IS_ENABLED(CONFIG_DEBUG_FS))
		chip->dbg_show = aquablue_gpio_dbg_show;

	chip->base = -1;
	chip->ngpio = num_gpios;
	chip->label = ab->client->name;
	chip->parent = &ab->client->dev;
	chip->of_node = chip->parent->of_node;
	chip->owner = THIS_MODULE;

	err = devm_gpiochip_add_data(&ab->client->dev, chip, ab);
	if (err)
		return err;

	return 0;
}

// =====================================================================

static irqreturn_t aquablue_irq(int irq, void *data)
{
	struct aquablue *ab = data;
	unsigned int num_regs, i;

	num_regs = 1 << ab->reg_shift;

	for (i = 0; i < num_regs; i++) {
		unsigned int base = i << ab->reg_shift, bit;
		u8 changed, level, isr, ier;
		unsigned long pending;
		int err;

		mutex_lock(&ab->i2c_lock);

		err = aquablue_read(ab, AQUABLUE_GPIO_REG_READ + i, &level);
		if (err < 0) {
			mutex_unlock(&ab->i2c_lock);
			continue;
		}

		err = aquablue_read(ab, AQUABLUE_GPIO_REG_ISR + i, &isr);
		if (err < 0) {
			mutex_unlock(&ab->i2c_lock);
			continue;
		}

		err = aquablue_read(ab, AQUABLUE_GPIO_REG_IER + i, &ier);
		if (err < 0) {
			mutex_unlock(&ab->i2c_lock);
			continue;
		}

		mutex_unlock(&ab->i2c_lock);

		/* determine pins that changed levels */
		changed = level ^ ab->irq_level[i];

		/* compute edge-triggered interrupts */
		pending = changed & ((ab->irq_fall[i] & ~level) |
				     (ab->irq_rise[i] & level));

		/* add in level-triggered interrupts */
		pending |= (ab->irq_high[i] & level) |
			   (ab->irq_low[i] & ~level);

		/* mask out non-pending and disabled interrupts */
		pending &= isr & ier;

		for_each_set_bit(bit, &pending, 8) {
			unsigned int child_irq;
			child_irq = irq_find_mapping(ab->gpio.irqdomain,
						     base + bit);
			handle_nested_irq(child_irq);
		}
	}
	return IRQ_HANDLED;
}

static void aquablue_irq_mask(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct aquablue *ab = gpiochip_get_data(gc);
	unsigned int reg = d->hwirq >> ab->reg_shift;
	unsigned int pos = d->hwirq & 7;

	ab->irq_enable[reg] &= ~BIT(pos);
}

static void aquablue_irq_unmask(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct aquablue *ab = gpiochip_get_data(gc);
	unsigned int reg = d->hwirq >> ab->reg_shift;
	unsigned int pos = d->hwirq & 7;

	ab->irq_enable[reg] |= BIT(pos);
}

static int aquablue_irq_set_type(struct irq_data *d, unsigned int type)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct aquablue *ab = gpiochip_get_data(gc);
	unsigned int reg = d->hwirq >> ab->reg_shift;
	unsigned int pos = d->hwirq & 7;

	if (type & IRQ_TYPE_EDGE_RISING)
		ab->irq_rise[reg] |= BIT(pos);
	else
		ab->irq_rise[reg] &= ~BIT(pos);

	if (type & IRQ_TYPE_EDGE_FALLING)
		ab->irq_fall[reg] |= BIT(pos);
	else
		ab->irq_fall[reg] &= ~BIT(pos);

	if (type & IRQ_TYPE_LEVEL_HIGH)
		ab->irq_high[reg] |= BIT(pos);
	else
		ab->irq_high[reg] &= ~BIT(pos);

	if (type & IRQ_TYPE_LEVEL_LOW)
		ab->irq_low[reg] |= BIT(pos);
	else
		ab->irq_low[reg] &= ~BIT(pos);

	return 0;
}

static void aquablue_irq_bus_lock(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct aquablue *ab = gpiochip_get_data(gc);

	mutex_lock(&ab->irq_lock);
}

static void aquablue_irq_bus_unlock(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct aquablue *ab = gpiochip_get_data(gc);
	unsigned int num_regs = 1 << ab->reg_shift, i;

	mutex_lock(&ab->i2c_lock);

	for (i = 0; i < num_regs; i++)
		aquablue_write(ab, AQUABLUE_GPIO_REG_IER + i, ab->irq_enable[i]);

	mutex_unlock(&ab->i2c_lock);
	mutex_unlock(&ab->irq_lock);
}

static struct irq_chip aquablue_irq_chip = {
	.name = "gpio-aquablue",
	.irq_mask = aquablue_irq_mask,
	.irq_unmask = aquablue_irq_unmask,
	.irq_set_type = aquablue_irq_set_type,
	.irq_bus_lock = aquablue_irq_bus_lock,
	.irq_bus_sync_unlock = aquablue_irq_bus_unlock,
};

int aquablue_irq_setup(struct aquablue *ab)
{
	unsigned int num_regs = 1 << ab->reg_shift, i;
	struct gpio_chip *chip = &ab->gpio;
	int err;

	mutex_init(&ab->irq_lock);

	/*
	 * Allocate memory to keep track of the current level and trigger
	 * modes of the interrupts. To avoid multiple allocations, a single
	 * large buffer is allocated and pointers are setup to point at the
	 * corresponding offsets. For consistency, the layout of the buffer
	 * is chosen to match the register layout of the hardware in that
	 * each segment contains the corresponding bits for all interrupts.
	 */
	ab->irq_enable = devm_kzalloc(chip->parent, num_regs * 6,
					GFP_KERNEL);
	if (!ab->irq_enable)
		return -ENOMEM;

	ab->irq_level = ab->irq_enable + (num_regs * 1);
	ab->irq_rise = ab->irq_enable + (num_regs * 2);
	ab->irq_fall = ab->irq_enable + (num_regs * 3);
	ab->irq_high = ab->irq_enable + (num_regs * 4);
	ab->irq_low = ab->irq_enable + (num_regs * 5);

	for (i = 0; i < num_regs; i++) {
		/*
		 * Read the initial level of all pins to allow the emulation
		 * of edge triggered interrupts.
		 */
		err = aquablue_read(ab, AQUABLUE_GPIO_REG_READ + i, &ab->irq_level[i]);
		if (err < 0)
			return err;

		/* disable all interrupts */
		err = aquablue_write(ab, AQUABLUE_GPIO_REG_IER + i, 0);
		if (err < 0)
			return err;

		ab->irq_enable[i] = 0x00;
	}
	err = devm_request_threaded_irq(chip->parent, ab->client->irq,
					NULL, aquablue_irq,
					IRQF_TRIGGER_RISING | IRQF_ONESHOT,
					dev_name(chip->parent), ab);
	if (err != 0) {
		dev_err(chip->parent, "can't request IRQ#%d: %d\n",
			ab->client->irq, err);
		return err;
	}

	err = gpiochip_irqchip_add(chip,
				   &aquablue_irq_chip,
				   0,
				   handle_simple_irq,
				   IRQ_TYPE_NONE);
	if (err) {
		dev_err(chip->parent,
			"could not connect irqchip to gpiochip\n");
		return err;
	}

	return 0;
}

// =====================================================================

int aquablue_swd_setup(struct aquablue *ab)
{
	return 0;
}

// =====================================================================

static int aquablue_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct device_node *np = client->dev.of_node;
	struct aquablue *ab;
	u32 num_gpios;
	int err;


	if (!i2c_check_functionality(client->adapter,
			I2C_FUNC_SMBUS_BYTE_DATA))
		return -EIO;

	err = of_property_read_u32(np, "nr-gpios", &num_gpios);
	if (err < 0)
		return err;

#if 0
	client->irq = irq_of_parse_and_map(np, 0);
	if (!client->irq)
		return -EPROBE_DEFER;
#endif

	ab = devm_kzalloc(&client->dev, sizeof(struct aquablue), GFP_KERNEL);
	if (!ab)
		return -ENOMEM;

	mutex_init(&ab->i2c_lock);
	ab->client = client;

	ab->regmap = devm_regmap_init_i2c(client, &aquablue_gpio_regmap_config);
	if (IS_ERR(ab->regmap)) {
		err = PTR_ERR(ab->regmap);
		dev_err(&client->dev, "failed to allocate register map: %d\n", err);
		return err;
	}

#if 0
	dev_info(&client->dev, "probe6\n");
	ab->irq_gpio = devm_gpiod_get(&client->dev, "irq", GPIOD_OUT_LOW);
	if (IS_ERR(ab->irq_gpio)) {
		dev_err(&client->dev, "unable to find interrupt gpio");
		return PTR_ERR(ab->irq_gpio);
	}

	dev_info(&client->dev, "probe7\n");
	if (!of_find_property(np, "interrupt-controller", NULL)) {
		dev_err(&client->dev, "missing interrupt-controller property\n");
		return -EIO;
	}

	err = aquablue_irq_setup(ab);
	if (err)
		return err;
#endif

	err = aquablue_gpio_setup(ab, num_gpios);
	if (err)
		return err;

	i2c_set_clientdata(client, ab);

	return 0;
}

static int aquablue_remove(struct i2c_client *client)
{
	return 0;
}

static const struct i2c_device_id aquablue_id[] = {
	{ "gpio-aquablue", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, aquablue_id);

static const struct of_device_id aquablue_of_match[] = {
	{ .compatible = "aquabyte,gpio-aquablue", },
	{ },
};
MODULE_DEVICE_TABLE(of, aquablue_of_match);

static struct i2c_driver aquablue_driver = {
	.driver = {
		.name = "gpio-aquablue",
		.of_match_table = aquablue_of_match,
	},
	.probe = aquablue_probe,
	.remove = aquablue_remove,
	.id_table = aquablue_id,
};

static int __init aquablue_init(void)
{
	return i2c_add_driver(&aquablue_driver);
}
subsys_initcall(aquablue_init);

static void __exit aquablue_exit(void)
{
	i2c_del_driver(&aquablue_driver);
}
module_exit(aquablue_exit);

MODULE_DESCRIPTION("GPIO expansion via Aquablue firmware");
MODULE_AUTHOR("Zachary T Welch <zach@aquabyte.ai>");
MODULE_LICENSE("GPL");
