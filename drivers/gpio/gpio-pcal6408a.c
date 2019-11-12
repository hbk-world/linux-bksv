/*
 *  PCAL6408 8 bit I/O ports
 *
 *  Copyright (C) 2005 Ben Gardner <bgardner@wabtec.com>
 *  Copyright (C) 2007 Marvell International Ltd.
 *  Copyright (c) 2018 Lars Thestrup <LarsErling.Thestrup@bksv.com>
 *
 *  Derived from drivers/i2c/chips/pca953x.c
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/platform_data/pca953x.h>
#include <linux/reset.h>
#include <linux/slab.h>
#include <asm/unaligned.h>
#include <linux/of_platform.h>
#include <linux/acpi.h>
#include <linux/regulator/consumer.h>
#include "gpiolib.h"

#define REG_ADDR_AI		0x80

#define PCAL6408_IN		    0x00
#define PCAL6408_OUT		0x01
#define PCAL6408_INVRT		0x02
#define PCAL6408_CFG		0x03
#define PCAL6408_ODS0		0x40
#define PCAL6408_ODS1		0x41
#define PCAL6408_IN_LATCH	0x42
#define PCAL6408_PUPDEN		0x43
#define PCAL6408_PUPD		0x44
#define PCAL6408_INT_MASK	0x45
#define PCAL6408_INT_STAT	0x46
#define PCAL6408_OUTCFG		0x4F

#define PCA_GPIO_MASK		0x00FF
#define PCA_INT			    0x0100
#define PCA_PCAL		    0x0200
#define PCA953X_TYPE		0x1000
#define PCA957X_TYPE		0x2000
#define PCAL6408_TYPE		0x3000
#define PCA_TYPE_MASK		0xF000

#define PCA_CHIP_TYPE(x)	((x) & PCA_TYPE_MASK)

static const struct i2c_device_id pcal6408_id[] = {
	{ "pcal6408", 8 | PCAL6408_TYPE | PCA_INT, },
	{ }
};
MODULE_DEVICE_TABLE(i2c, pcal6408_id);

#define MAX_BANK 5
#define BANK_SZ 8

#define NBANK(chip) DIV_ROUND_UP(chip->gpio_chip.ngpio, BANK_SZ)
struct pcal6408_reg_config {                                    //power-up default
	u8 input;      // 00h Input port read byte                        xxxx xxxx
	u8 output;     // 01h Output port read/write byte                 1111 1111
	u8 inversion;  // 02h Polarity Inversion read/write byte          0000 0000
	u8 direction;  // 03h Configuration read/write byte               1111 1111
	u8 strength0;  // 40h Output drive strength 0 read/write byte     1111 1111
	u8 strength1;  // 41h Output drive strength 1 read/write byte     1111 1111
	u8 in_latch;   // 42h Input latch read/write byte                 0000 0000
	u8 pd_enable;  // 43h Pull-up/pull-down enable read/write byte    0000 0000
	u8 pd_select;  // 44h Pull-up/pull-down selection read/write byte 1111 1111
	u8 irq_mask;   // 45h Interrupt mask read/write byte              1111 1111
	u8 irq_status; // 46h Interrupt status read byte                  0000 0000
	u8 out_conf;   // 4Fh Output port configuration read/write byte   0000 0000
};

static const struct pcal6408_reg_config pcal6408_init_values = {
.input     = 0x00,
.output    = 0xFF,
.inversion = 0x00,
.direction = 0xFF,
.strength0 = 0xFF,
.strength1 = 0xFF,
.in_latch  = 0x00,
.pd_enable = 0x00,
.pd_select = 0xFF,
.irq_mask  = 0xFF,
.irq_status= 0x00,
.out_conf  = 0x00,
};

static const struct pcal6408_reg_config pcal6408_regs = {
.input     = PCAL6408_IN,
.output    = PCAL6408_OUT,
.inversion = PCAL6408_INVRT,
.direction = PCAL6408_CFG,
.strength0 = PCAL6408_ODS0,
.strength1 = PCAL6408_ODS1,
.in_latch  = PCAL6408_IN_LATCH,
.pd_enable = PCAL6408_PUPDEN,
.pd_select = PCAL6408_PUPD,
.irq_mask  = PCAL6408_INT_MASK,
.irq_status= PCAL6408_INT_STAT,
.out_conf  = PCAL6408_OUTCFG,
};

struct pcal6408_chip {
	unsigned gpio_start;
	u8 reg_output[MAX_BANK];
	u8 reg_direction[MAX_BANK];
	struct mutex i2c_lock;

#ifdef CONFIG_GPIO_PCA953X_IRQ
	struct mutex irq_lock;
	u8 irq_mask[MAX_BANK];
	u8 irq_stat[MAX_BANK];
	u8 irq_trig_raise[MAX_BANK];
	u8 irq_trig_fall[MAX_BANK];
#endif

	struct i2c_client *client;
	struct gpio_chip gpio_chip;
	const char *const *names;
	unsigned long driver_data;
	struct regulator *regulator;
    struct notifier_block nb;
    bool regulator_is_on;
    const struct pcal6408_reg_config *regs;
          struct pcal6408_reg_config init_values;

	int (*write_regs)(struct pcal6408_chip *, int, u8 *);
	int (*read_regs)(struct pcal6408_chip *, int, u8 *);
};

static int pca953x_read_single(struct pcal6408_chip *chip, int reg, u32 *val,
				int off)
{
	int ret;
	int bank_shift = fls((chip->gpio_chip.ngpio - 1) / BANK_SZ);
	int offset = off / BANK_SZ;

	ret = i2c_smbus_read_byte_data(chip->client,
				(reg << bank_shift) + offset);
	*val = ret;

	if (ret < 0) {
		dev_err(&chip->client->dev, "failed reading register 0x%x\n",(reg << bank_shift) + offset);
		return ret;
	}

	return 0;
}

static int pca953x_write_single(struct pcal6408_chip *chip, int reg, u32 val,
				int off)
{
    if (chip->regulator_is_on)
    {//if not on, registers is set by the init string in device tree
        int ret;
        int bank_shift = fls((chip->gpio_chip.ngpio - 1) / BANK_SZ);
        int offset = off / BANK_SZ;

        ret = i2c_smbus_write_byte_data(chip->client,
                        (reg << bank_shift) + offset, val);

        if (ret < 0) {
            dev_err(&chip->client->dev, "failed writing register %x val %x\n",reg,val);
            return ret;
        }
    }
	return 0;
}

static int pca953x_write_regs_8(struct pcal6408_chip *chip, int reg, u8 *val)
{
	return i2c_smbus_write_byte_data(chip->client, reg, *val);
}

static int pca953x_write_regs_16(struct pcal6408_chip *chip, int reg, u8 *val)
{
	__le16 word = cpu_to_le16(get_unaligned((u16 *)val));

	return i2c_smbus_write_word_data(chip->client,
					 reg << 1, (__force u16)word);
}

static int pca957x_write_regs_16(struct pcal6408_chip *chip, int reg, u8 *val)
{
	int ret;

	ret = i2c_smbus_write_byte_data(chip->client, reg << 1, val[0]);
	if (ret < 0)
		return ret;

	return i2c_smbus_write_byte_data(chip->client, (reg << 1) + 1, val[1]);
}

static int pca953x_write_regs_24(struct pcal6408_chip *chip, int reg, u8 *val)
{
	int bank_shift = fls((chip->gpio_chip.ngpio - 1) / BANK_SZ);

	return i2c_smbus_write_i2c_block_data(chip->client,
					      (reg << bank_shift) | REG_ADDR_AI,
					      NBANK(chip), val);
}

static int pca953x_write_regs(struct pcal6408_chip *chip, int reg, u8 *val)
{
	int ret = 0;

	ret = chip->write_regs(chip, reg, val);
	if (ret < 0) {
		dev_err(&chip->client->dev, "failed writing register\n");
		return ret;
	}

	return 0;
}

static int pca953x_read_regs_8(struct pcal6408_chip *chip, int reg, u8 *val)
{
	int ret;

	ret = i2c_smbus_read_byte_data(chip->client, reg);
	*val = ret;

	return ret;
}

static int pca953x_read_regs_16(struct pcal6408_chip *chip, int reg, u8 *val)
{
	int ret;

	ret = i2c_smbus_read_word_data(chip->client, reg << 1);
	val[0] = (u16)ret & 0xFF;
	val[1] = (u16)ret >> 8;

	return ret;
}

static int pca953x_read_regs_24(struct pcal6408_chip *chip, int reg, u8 *val)
{
	int bank_shift = fls((chip->gpio_chip.ngpio - 1) / BANK_SZ);

	return i2c_smbus_read_i2c_block_data(chip->client,
					     (reg << bank_shift) | REG_ADDR_AI,
					     NBANK(chip), val);
}

static int pca953x_read_regs(struct pcal6408_chip *chip, int reg, u8 *val)
{
	int ret;

	ret = chip->read_regs(chip, reg, val);
	if (ret < 0) {
		dev_err(&chip->client->dev, "failed reading register 0x%x\n",reg);
		return ret;
	}

	return 0;
}

static int pca953x_gpio_direction_input(struct gpio_chip *gc, unsigned off)
{
	struct pcal6408_chip *chip;
	u8 reg_val;
	int ret=0;
	chip = gpiochip_get_data(gc);

	mutex_lock(&chip->i2c_lock);
	reg_val = chip->reg_direction[off / BANK_SZ] | (1u << (off % BANK_SZ));

    if (chip->regulator_is_on)
    {
	    ret = pca953x_write_single(chip, chip->regs->direction, reg_val, off);
        if (ret)
            goto exit;
    }

	chip->reg_direction[off / BANK_SZ] = reg_val;
exit:
	mutex_unlock(&chip->i2c_lock);
	return ret;
}

static int pca953x_gpio_direction_output(struct gpio_chip *gc,
		unsigned off, int val)
{
	struct pcal6408_chip *chip = gpiochip_get_data(gc);
	u8 reg_val;
	int ret;

	mutex_lock(&chip->i2c_lock);
	/* set output level */
	if (val)
		reg_val = chip->reg_output[off / BANK_SZ]
			| (1u << (off % BANK_SZ));
	else
		reg_val = chip->reg_output[off / BANK_SZ]
			& ~(1u << (off % BANK_SZ));

    if (chip->regulator_is_on)
    {
        ret = pca953x_write_single(chip, chip->regs->output, reg_val, off);
        if (ret)
            goto exit;
    }

	chip->reg_output[off / BANK_SZ] = reg_val;

	/* then direction */
	reg_val = chip->reg_direction[off / BANK_SZ] & ~(1u << (off % BANK_SZ));
	ret = pca953x_write_single(chip, chip->regs->direction, reg_val, off);
	if (ret)
		goto exit;

	chip->reg_direction[off / BANK_SZ] = reg_val;
exit:
	mutex_unlock(&chip->i2c_lock);
	return ret;
}
static int pca953x_gpio_get_value(struct gpio_chip *gc, unsigned off)
{
	struct pcal6408_chip *chip = gpiochip_get_data(gc);
	u32 reg_val;
	int ret;

	mutex_lock(&chip->i2c_lock);

	ret = pca953x_read_single(chip, chip->regs->input, &reg_val, off);
	mutex_unlock(&chip->i2c_lock);
	if (ret < 0) {
		/* NOTE:  diagnostic already emitted; that's all we should
		 * do unless gpio_*_value_cansleep() calls become different
		 * from their nonsleeping siblings (and report faults).
		 */
		return 0;
	}

	return (reg_val & (1u << (off % BANK_SZ))) ? 1 : 0;
}

static void pca953x_gpio_set_value(struct gpio_chip *gc, unsigned off, int val)
{
	struct pcal6408_chip *chip = gpiochip_get_data(gc);
	u8 reg_val;
	int ret;

	mutex_lock(&chip->i2c_lock);
	if (val)
		reg_val = chip->reg_output[off / BANK_SZ]
			| (1u << (off % BANK_SZ));
	else
		reg_val = chip->reg_output[off / BANK_SZ]
			& ~(1u << (off % BANK_SZ));

	ret = pca953x_write_single(chip, chip->regs->output, reg_val, off);
	if (ret)
		goto exit;

	chip->reg_output[off / BANK_SZ] = reg_val;
exit:
	mutex_unlock(&chip->i2c_lock);
}

static void pca953x_gpio_set_multiple(struct gpio_chip *gc,
				      unsigned long *mask, unsigned long *bits)
{
	struct pcal6408_chip *chip = gpiochip_get_data(gc);
	unsigned int bank_mask, bank_val;
	int bank_shift, bank;
	u8 reg_val[MAX_BANK];
	int ret;

    printk("%s\n",__func__);
	bank_shift = fls((chip->gpio_chip.ngpio - 1) / BANK_SZ);

	mutex_lock(&chip->i2c_lock);
	memcpy(reg_val, chip->reg_output, NBANK(chip));
	for (bank = 0; bank < NBANK(chip); bank++) {
		bank_mask = mask[bank / sizeof(*mask)] >>
			   ((bank % sizeof(*mask)) * 8);
		if (bank_mask) {
			bank_val = bits[bank / sizeof(*bits)] >>
				  ((bank % sizeof(*bits)) * 8);
			bank_val &= bank_mask;
			reg_val[bank] = (reg_val[bank] & ~bank_mask) | bank_val;
		}
	}

	ret = i2c_smbus_write_i2c_block_data(chip->client,
					     chip->regs->output << bank_shift,
					     NBANK(chip), reg_val);
	if (ret)
		goto exit;

	memcpy(chip->reg_output, reg_val, NBANK(chip));
exit:
	mutex_unlock(&chip->i2c_lock);
}

static void pca953x_setup_gpio(struct pcal6408_chip *chip, int gpios)
{
	struct gpio_chip *gc;

	gc = &chip->gpio_chip;

	gc->direction_input  = pca953x_gpio_direction_input;
	gc->direction_output = pca953x_gpio_direction_output;
	gc->get = pca953x_gpio_get_value;
	gc->set = pca953x_gpio_set_value;
	gc->set_multiple = pca953x_gpio_set_multiple;
	gc->can_sleep = true;

	gc->base = chip->gpio_start;
	gc->ngpio = gpios;
	gc->label = chip->client->name;
	gc->parent = &chip->client->dev;
	gc->owner = THIS_MODULE;
	gc->names = chip->names;
}

#ifdef CONFIG_GPIO_PCA953X_IRQ
static void pca953x_irq_mask(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct pcal6408_chip *chip = gpiochip_get_data(gc);

	chip->irq_mask[d->hwirq / BANK_SZ] &= ~(1 << (d->hwirq % BANK_SZ));
}

static void pca953x_irq_unmask(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct pcal6408_chip *chip = gpiochip_get_data(gc);

	chip->irq_mask[d->hwirq / BANK_SZ] |= 1 << (d->hwirq % BANK_SZ);
}

static void pca953x_irq_bus_lock(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct pcal6408_chip *chip = gpiochip_get_data(gc);

	mutex_lock(&chip->irq_lock);
}

static void pca953x_irq_bus_sync_unlock(struct irq_data *d)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct pcal6408_chip *chip = gpiochip_get_data(gc);
	u8 new_irqs;
	int level, i;
	u8 invert_irq_mask[MAX_BANK];

	if (chip->driver_data & PCA_PCAL) {
		/* Enable latch on interrupt-enabled inputs */
		pca953x_write_regs(chip, PCAL953X_IN_LATCH, chip->irq_mask);

		for (i = 0; i < NBANK(chip); i++)
			invert_irq_mask[i] = ~chip->irq_mask[i];

		/* Unmask enabled interrupts */
		pca953x_write_regs(chip, PCAL953X_INT_MASK, invert_irq_mask);
	}

	/* Look for any newly setup interrupt */
	for (i = 0; i < NBANK(chip); i++) {
		new_irqs = chip->irq_trig_fall[i] | chip->irq_trig_raise[i];
		new_irqs &= ~chip->reg_direction[i];

		while (new_irqs) {
			level = __ffs(new_irqs);
			pca953x_gpio_direction_input(&chip->gpio_chip,
							level + (BANK_SZ * i));
			new_irqs &= ~(1 << level);
		}
	}

	mutex_unlock(&chip->irq_lock);
}

static int pca953x_irq_set_type(struct irq_data *d, unsigned int type)
{
	struct gpio_chip *gc = irq_data_get_irq_chip_data(d);
	struct pcal6408_chip *chip = gpiochip_get_data(gc);
	int bank_nb = d->hwirq / BANK_SZ;
	u8 mask = 1 << (d->hwirq % BANK_SZ);

	if (!(type & IRQ_TYPE_EDGE_BOTH)) {
		dev_err(&chip->client->dev, "irq %d: unsupported type %d\n",
			d->irq, type);
		return -EINVAL;
	}

	if (type & IRQ_TYPE_EDGE_FALLING)
		chip->irq_trig_fall[bank_nb] |= mask;
	else
		chip->irq_trig_fall[bank_nb] &= ~mask;

	if (type & IRQ_TYPE_EDGE_RISING)
		chip->irq_trig_raise[bank_nb] |= mask;
	else
		chip->irq_trig_raise[bank_nb] &= ~mask;

	return 0;
}

static struct irq_chip pca953x_irq_chip = {
	.name			= "pca953x",
	.irq_mask		= pca953x_irq_mask,
	.irq_unmask		= pca953x_irq_unmask,
	.irq_bus_lock		= pca953x_irq_bus_lock,
	.irq_bus_sync_unlock	= pca953x_irq_bus_sync_unlock,
	.irq_set_type		= pca953x_irq_set_type,
};

static bool pca953x_irq_pending(struct pcal6408_chip *chip, u8 *pending)
{
	u8 cur_stat[MAX_BANK];
	u8 old_stat[MAX_BANK];
	bool pending_seen = false;
	bool trigger_seen = false;
	u8 trigger[MAX_BANK];
	int ret, i;

	if (chip->driver_data & PCA_PCAL) {
		/* Read the current interrupt status from the device */
		ret = pca953x_read_regs(chip, PCAL953X_INT_STAT, trigger);
		if (ret)
			return false;

		/* Check latched inputs and clear interrupt status */
		ret = pca953x_read_regs(chip, PCA953X_INPUT, cur_stat);
		if (ret)
			return false;

		for (i = 0; i < NBANK(chip); i++) {
			/* Apply filter for rising/falling edge selection */
			pending[i] = (~cur_stat[i] & chip->irq_trig_fall[i]) |
				(cur_stat[i] & chip->irq_trig_raise[i]);
			pending[i] &= trigger[i];
			if (pending[i])
				pending_seen = true;
		}

		return pending_seen;
	}

	ret = pca953x_read_regs(chip, chip->regs->input, cur_stat);
	if (ret)
		return false;

	/* Remove output pins from the equation */
	for (i = 0; i < NBANK(chip); i++)
		cur_stat[i] &= chip->reg_direction[i];

	memcpy(old_stat, chip->irq_stat, NBANK(chip));

	for (i = 0; i < NBANK(chip); i++) {
		trigger[i] = (cur_stat[i] ^ old_stat[i]) & chip->irq_mask[i];
		if (trigger[i])
			trigger_seen = true;
	}

	if (!trigger_seen)
		return false;

	memcpy(chip->irq_stat, cur_stat, NBANK(chip));

	for (i = 0; i < NBANK(chip); i++) {
		pending[i] = (old_stat[i] & chip->irq_trig_fall[i]) |
			(cur_stat[i] & chip->irq_trig_raise[i]);
		pending[i] &= trigger[i];
		if (pending[i])
			pending_seen = true;
	}

	return pending_seen;
}

static irqreturn_t pca953x_irq_handler(int irq, void *devid)
{
	struct pcal6408_chip *chip = devid;
	u8 pending[MAX_BANK];
	u8 level;
	unsigned nhandled = 0;
	int i;

	if (!pca953x_irq_pending(chip, pending))
		return IRQ_NONE;

	for (i = 0; i < NBANK(chip); i++) {
		while (pending[i]) {
			level = __ffs(pending[i]);
			handle_nested_irq(irq_find_mapping(chip->gpio_chip.irqdomain,
							level + (BANK_SZ * i)));
			pending[i] &= ~(1 << level);
			nhandled++;
		}
	}

	return (nhandled > 0) ? IRQ_HANDLED : IRQ_NONE;
}

static int pca953x_irq_setup(struct pcal6408_chip *chip,
			     int irq_base)
{
	struct i2c_client *client = chip->client;
	int ret, i;

	if (client->irq && irq_base != -1
			&& (chip->driver_data & PCA_INT)) {
		ret = pca953x_read_regs(chip,
					chip->regs->input, chip->irq_stat);
		if (ret)
			return ret;

		/*
		 * There is no way to know which GPIO line generated the
		 * interrupt.  We have to rely on the previous read for
		 * this purpose.
		 */
		for (i = 0; i < NBANK(chip); i++)
			chip->irq_stat[i] &= chip->reg_direction[i];
		mutex_init(&chip->irq_lock);

		ret = devm_request_threaded_irq(&client->dev,
					client->irq,
					   NULL,
					   pca953x_irq_handler,
					   IRQF_TRIGGER_LOW | IRQF_ONESHOT |
						   IRQF_SHARED,
					   dev_name(&client->dev), chip);
		if (ret) {
			dev_err(&client->dev, "failed to request irq %d\n",
				client->irq);
			return ret;
		}

		ret =  gpiochip_irqchip_add(&chip->gpio_chip,
					    &pca953x_irq_chip,
					    irq_base,
					    handle_simple_irq,
					    IRQ_TYPE_NONE);
		if (ret) {
			dev_err(&client->dev,
				"could not connect irqchip to gpiochip\n");
			return ret;
		}

		gpiochip_set_chained_irqchip(&chip->gpio_chip,
					     &pca953x_irq_chip,
					     client->irq, NULL);
	}

	return 0;
}

#else /* CONFIG_GPIO_PCA953X_IRQ */
static int pca953x_irq_setup(struct pcal6408_chip *chip,
			     int irq_base)
{
	struct i2c_client *client = chip->client;

	if (irq_base != -1 && (chip->driver_data & PCA_INT))
		dev_warn(&client->dev, "interrupt support not compiled in\n");

	return 0;
}
#endif

static int device_pcal6408_init(struct pcal6408_chip *chip, u32 invert)
{
	int ret;

    ret = pca953x_write_regs(chip, chip->regs->out_conf,  &chip->init_values.out_conf);
	ret = pca953x_write_regs(chip, chip->regs->direction, &chip->init_values.direction);
	ret = pca953x_write_regs(chip, chip->regs->inversion, &chip->init_values.inversion);
	ret = pca953x_write_regs(chip, chip->regs->strength0, &chip->init_values.strength0);
	ret = pca953x_write_regs(chip, chip->regs->strength1, &chip->init_values.strength1);
	ret = pca953x_write_regs(chip, chip->regs->in_latch,  &chip->init_values.in_latch);
	ret = pca953x_write_regs(chip, chip->regs->pd_enable, &chip->init_values.pd_enable);
	ret = pca953x_write_regs(chip, chip->regs->pd_select, &chip->init_values.pd_select);
	ret = pca953x_write_regs(chip, chip->regs->irq_mask,  &chip->init_values.irq_mask);
	ret = pca953x_write_regs(chip, chip->regs->output,    &chip->reg_output[0]);


	ret = pca953x_read_regs(chip, chip->regs->direction, chip->reg_direction);

	return ret;
}
/*
disable
ana_notify action=1024    REGULATOR_EVENT_PRE_DISABLE
ana_notify action=8192    REGULATOR_EVENT_PRE_DO_DISABLE
ana_notify action=128     REGULATOR_EVENT_DISABLE

enable
ana_notify action=4096    REGULATOR_EVENT_PRE_DO_ENABLE
ana_notify action=16384   REGULATOR_EVENT_AFT_DO_ENABLE
*/
static int ana_notify(struct notifier_block *self, unsigned long action, void *data)
{
	struct pcal6408_chip *chip;
//    printk("%s_i2c action=%lu\n",__func__,action);
	chip = container_of(self, struct pcal6408_chip, nb); 

    switch(action)
    {
        case REGULATOR_EVENT_AFT_DO_ENABLE:
        if (PCA_CHIP_TYPE(chip->driver_data) == PCAL6408_TYPE)
        {
            chip->regulator_is_on=true;
		    device_pcal6408_init(chip, 0);
        }
        break;
        case REGULATOR_EVENT_PRE_DISABLE:
            chip->regulator_is_on=false;
        break;
    }
	return NOTIFY_OK;
}
static struct gpio ana_gpios[] = {
     //windscreen
    { 0, GPIOF_DIR_IN, "WSX" },
    { 0, GPIOF_DIR_IN, "WSY" },
    { 0, GPIOF_DIR_IN, "WSZ" },
     //ADC OSR
    { 0, GPIOF_DIR_OUT, "OSR0"},
    { 0, GPIOF_DIR_OUT, "OSR1"}
};

static const struct of_device_id pca953x_dt_ids[];

static int pcal6408_probe(struct i2c_client *client,
				        const struct i2c_device_id *i2c_id)
{
	struct pca953x_platform_data *pdata;
	struct pcal6408_chip *chip;
	int irq_base = 0;
	int ret = -1,reg_ret;
	u32 invert = 0;
	struct regulator *reg;
	struct device_node *np;
   	struct property *prop;
    u32 regs_init[11][2];
    int err;

    printk("%s driver probing\n",__func__);

	chip = devm_kzalloc(&client->dev,
			sizeof(struct pcal6408_chip), GFP_KERNEL);
	if (chip == NULL)
		return -ENOMEM;

	pdata = dev_get_platdata(&client->dev);
	if (pdata) {
		irq_base = pdata->irq_base;
		chip->gpio_start = pdata->gpio_base;
		invert = pdata->invert;
		chip->names = pdata->names;
	} else {
//    printk("%s pdata not valid\n",__func__);
		chip->gpio_start = -1;
		irq_base = 0;
	}

	chip->client = client;

	reg = devm_regulator_get(&client->dev, "ana_on");
	if (IS_ERR(reg)) {
		ret = PTR_ERR(reg);
		if (ret != -EPROBE_DEFER)
			dev_err(&client->dev, "reg get err: %d\n", ret);
		return ret;
	}
	chip->regulator = reg;

    if (regulator_is_enabled(chip->regulator) > 0)
        chip->regulator_is_on=true;
    else
        chip->regulator_is_on=false;

    memset(&chip->nb,0,sizeof(struct notifier_block));
    chip->nb.notifier_call=ana_notify;
    reg_ret = devm_regulator_register_notifier(reg,&chip->nb);
    
    memcpy(&chip->init_values,&pcal6408_init_values,sizeof(struct pcal6408_reg_config));
	memset(regs_init,0xFF,sizeof(u32)*11*2);
	np = client->dev.of_node;
   	prop = of_find_property(np, "regs_init", NULL);
    if (prop && prop->value)
    {
        size_t sz;
        u32 i;
        const __be32 *val;
        val = prop->value;
        sz = prop->length >> 3; //sizeof(u32)*2
        i=0;
        while (sz--)
        {
            regs_init[i][0]  =be32_to_cpup(val++);
            regs_init[i++][1]=be32_to_cpup(val++);
        }
        for (i=0;i<11;i++)
        {
            if (regs_init[i][0] ==0xFFFFFFFF)
                break;
            switch (regs_init[i][0])
            {
                case PCAL6408_IN:       chip->init_values.input     =(u8)regs_init[i][1]; break;
                case PCAL6408_OUT:      chip->init_values.output    =(u8)regs_init[i][1]; break;
                case PCAL6408_INVRT:    chip->init_values.inversion =(u8)regs_init[i][1]; break;
                case PCAL6408_CFG:      chip->init_values.direction =(u8)regs_init[i][1]; break;
                case PCAL6408_ODS0:     chip->init_values.strength0 =(u8)regs_init[i][1]; break;
                case PCAL6408_ODS1:     chip->init_values.strength1 =(u8)regs_init[i][1]; break;
                case PCAL6408_IN_LATCH: chip->init_values.in_latch  =(u8)regs_init[i][1]; break;
                case PCAL6408_PUPDEN:   chip->init_values.pd_enable =(u8)regs_init[i][1]; break;
                case PCAL6408_PUPD:     chip->init_values.pd_select =(u8)regs_init[i][1]; break;
                case PCAL6408_INT_MASK: chip->init_values.irq_mask  =(u8)regs_init[i][1]; break;
                case PCAL6408_INT_STAT: chip->init_values.irq_status=(u8)regs_init[i][1]; break;
                case PCAL6408_OUTCFG:   chip->init_values.out_conf  =(u8)regs_init[i][1]; break;
            }
        }

    }

//		led.name =
//			of_get_property(child, "label", NULL) ? : child->name;

	if (i2c_id) {
		chip->driver_data = i2c_id->driver_data;
	}

	mutex_init(&chip->i2c_lock);
	/*
	 * In case we have an i2c-mux controlled by a GPIO provided by an
	 * expander using the same driver higher on the device tree, read the
	 * i2c adapter nesting depth and use the retrieved value as lockdep
	 * subclass for chip->i2c_lock.
	 *
	 * REVISIT: This solution is not complete. It protects us from lockdep
	 * false positives when the expander controlling the i2c-mux is on
	 * a different level on the device tree, but not when it's on the same
	 * level on a different branch (in which case the subclass number
	 * would be the same).
	 *
	 * TODO: Once a correct solution is developed, a similar fix should be
	 * applied to all other i2c-controlled GPIO expanders (and potentially
	 * regmap-i2c).
	 */
	lockdep_set_subclass(&chip->i2c_lock,
			     i2c_adapter_depth(client->adapter));

	ret = device_reset(&client->dev);
	if (ret == -EPROBE_DEFER)
		return -EPROBE_DEFER;

	/* initialize cached registers from their original values.
	 * we can't share this chip with another i2c master.
	 */
	pca953x_setup_gpio(chip, chip->driver_data & PCA_GPIO_MASK);

	chip->regs = &pcal6408_regs;

	if (chip->gpio_chip.ngpio <= 8) {
		chip->write_regs = pca953x_write_regs_8;
		chip->read_regs = pca953x_read_regs_8;
	} else if (chip->gpio_chip.ngpio >= 24) {
		chip->write_regs = pca953x_write_regs_24;
		chip->read_regs = pca953x_read_regs_24;
	} else {
		if (PCA_CHIP_TYPE(chip->driver_data) == PCA953X_TYPE)
			chip->write_regs = pca953x_write_regs_16;
		else
			chip->write_regs = pca957x_write_regs_16;
		chip->read_regs = pca953x_read_regs_16;
	}

    if (regulator_is_enabled(reg) > 0)
	{
        if (PCA_CHIP_TYPE(chip->driver_data) == PCAL6408_TYPE)
		    ret = device_pcal6408_init(chip, invert);
	    if (ret)
		    goto err_exit;
    }

	ret = devm_gpiochip_add_data(&client->dev, &chip->gpio_chip, chip);
	if (ret)
		goto err_exit;

	ret = pca953x_irq_setup(chip, irq_base);
	if (ret)
		goto err_exit;

	if (pdata && pdata->setup) {
		ret = pdata->setup(client, chip->gpio_chip.base,
				chip->gpio_chip.ngpio, pdata->context);
		if (ret < 0)
			dev_warn(&client->dev, "setup failed, %d\n", ret);
	}

	i2c_set_clientdata(client, chip);

    ana_gpios[0].gpio=chip->gpio_chip.base+0;
    ana_gpios[1].gpio=chip->gpio_chip.base+1;
    ana_gpios[2].gpio=chip->gpio_chip.base+2;
    ana_gpios[3].gpio=chip->gpio_chip.base+4;
    ana_gpios[4].gpio=chip->gpio_chip.base+5;
   	err  = gpio_request_array(ana_gpios, 5);
	if (err)
    {
        printk("%s gpio_request failed, err=%d\n",__func__,err);
    }
    else
    {
        gpio_export(ana_gpios[0].gpio,false);
        gpio_export(ana_gpios[1].gpio,false);
        gpio_export(ana_gpios[2].gpio,false);
        gpio_export(ana_gpios[3].gpio,false);
        gpio_export(ana_gpios[4].gpio,false);
    }
    printk("%s driver probed\n",__func__);
	return 0;

err_exit:
	regulator_disable(chip->regulator);
	return ret;
}

static int pcal6408_remove(struct i2c_client *client)
{
	struct pca953x_platform_data *pdata = dev_get_platdata(&client->dev);
	struct pcal6408_chip *chip = i2c_get_clientdata(client);
	int ret;

    gpio_free(ana_gpios[0].gpio);
    gpio_free(ana_gpios[1].gpio);
    gpio_free(ana_gpios[2].gpio);
    gpio_free(ana_gpios[3].gpio);
    gpio_free(ana_gpios[4].gpio);

	if (pdata && pdata->teardown) {
		ret = pdata->teardown(client, chip->gpio_chip.base,
				chip->gpio_chip.ngpio, pdata->context);
		if (ret < 0)
			dev_err(&client->dev, "%s failed, %d\n",
					"teardown", ret);
	} else {
		ret = 0;
	}

	regulator_disable(chip->regulator);

	return ret;
}

/* convenience to stop overlong match-table lines */
#define OF_953X(__nrgpio, __int) (void *)(__nrgpio | PCA953X_TYPE | __int)
#define OF_957X(__nrgpio, __int) (void *)(__nrgpio | PCA957X_TYPE | __int)
#define OF_6408(__nrgpio, __int) (void *)(__nrgpio | PCAL6408_TYPE | __int)

static const struct of_device_id pcaL6408_dt_ids[] = {
	{ .compatible = "nxp,pcal6408", .data = OF_6408(8, PCA_INT), },
	{ }
};

MODULE_DEVICE_TABLE(of, pca953x_dt_ids);

static struct i2c_driver pca953x_driver = {
	.driver = {
		.name	= "pcal6408",
		.of_match_table = pcaL6408_dt_ids,
	},
	.probe		= pcal6408_probe,
	.remove		= pcal6408_remove,
	.id_table	= pcal6408_id,
};

static int __init pca953x_init(void)
{
	return i2c_add_driver(&pca953x_driver);
}
/* register after i2c postcore initcall and before
 * subsys initcalls that may rely on these GPIOs
 */
late_initcall(pca953x_init);

static void __exit pca953x_exit(void)
{
	i2c_del_driver(&pca953x_driver);
}
module_exit(pca953x_exit);

MODULE_AUTHOR("Lars Thestrup <LarsErling.Thestrup@bksv.com>");
MODULE_DESCRIPTION("GPIO expander driver for PCAL6408");
MODULE_LICENSE("GPL");
