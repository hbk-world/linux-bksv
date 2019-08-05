/*
 *  dac8560.c - Linux kernel module for
 * 	Texas Instruments DAC8560
 *
 *  Copyright (c) 2017 Lars Thestrup <LarsErling.Thestrup@bksv.com>
 *  Based on dac7512 by Daniel Mack <daniel@caiaq.de>
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 675 Mass Ave, Cambridge, MA 02139, USA.
 */

#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/of.h>
#include <linux/of_device.h>

struct dacdata
{
	unsigned long pwr;
	unsigned long val;  
};

static void dac_write(struct spi_device *spi)
{
 	struct dacdata *dac=spi->dev.driver_data;
    unsigned char tmp[3];
 	tmp[0] = dac->pwr & 0x3;        // 000000pp where pp bits is powerdown mode, 00==normal operation
	tmp[1] = dac->val >> 8;
	tmp[2] = dac->val & 0xff;
	spi_write(spi, tmp, sizeof(tmp));
}

static ssize_t dac8560_store_pwr(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct spi_device *spi = to_spi_device(dev);
    struct dacdata *dac=dev->driver_data;
	int ret;

	ret = kstrtoul(buf, 10, &dac->pwr);
	if (ret)
		return ret;
    dac_write(spi);
	return count;
}

static ssize_t dac8560_store_val(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct spi_device *spi = to_spi_device(dev);
    struct dacdata *dac=dev->driver_data;
	int ret;

	ret = kstrtoul(buf, 10, &dac->val);
	if (ret)
		return ret;
    dac_write(spi);
	return count;
}

static ssize_t dac8560_show_val(struct device *dev,
    struct device_attribute *attr,
    char *buf)
{
    struct spi_device *spi = to_spi_device(dev);
    struct dacdata *dac=dev->driver_data;
    return sprintf(buf, "%d\n", dac->val);
}

static DEVICE_ATTR(powerdown, S_IWUSR, NULL, dac8560_store_pwr);
static DEVICE_ATTR(value,     S_IWUSR|S_IRUSR, dac8560_show_val, dac8560_store_val);


static struct attribute *dac8560_attributes[] = {
	&dev_attr_value.attr,
	&dev_attr_powerdown.attr,
	NULL
};

static const struct attribute_group dac8560_attr_group = {
	.attrs = dac8560_attributes,
};


static int dac8560_probe(struct spi_device *spi)
{
	int ret;
	struct device_node *np = spi->dev.of_node;
    struct dacdata *dac;
	dac = devm_kzalloc(&spi->dev, sizeof(*dac), GFP_KERNEL);
	if (!dac)
		return -ENOMEM;
    spi->dev.driver_data=dac;
   
	spi->bits_per_word = 8;
	spi->mode = SPI_MODE_1;
	ret = spi_setup(spi);
	if (ret < 0)
		return ret;
    ret= sysfs_create_group(&spi->dev.kobj, &dac8560_attr_group);
    if (np)
    {
        u32 val;
		if (device_property_read_u32(&spi->dev,"init-val", &val) >=0)
        {
            dac->val=(u16)val;
            dac_write(spi);
        }
    }
	return ret;
}

static int dac8560_remove(struct spi_device *spi)
{
	sysfs_remove_group(&spi->dev.kobj, &dac8560_attr_group);
	return 0;
}


static const struct spi_device_id dac8560_id_table[] = {
	{ "dac8560", 0 },
	{ }
};
MODULE_DEVICE_TABLE(spi, dac8560_id_table);

#ifdef CONFIG_OF
static const struct of_device_id dac8560_of_match[] = {
	{ .compatible = "ti,dac8560", .data = &dac8560_attr_group,},
	{ }
};
MODULE_DEVICE_TABLE(of, dac8560_of_match);
#endif

static struct spi_driver dac8560_driver = {
	.driver = {
		.name	= "dac8560",
		.owner	= THIS_MODULE,
		.of_match_table = of_match_ptr(dac8560_of_match),
	},
	.probe	  = dac8560_probe,
	.remove	  = dac8560_remove,
	.id_table = dac8560_id_table,
};

module_spi_driver(dac8560_driver);

MODULE_AUTHOR("Lars Thestrup <LarsErling.Thestrup@bksv.com>");
MODULE_DESCRIPTION("DAC8560 16-bit DAC");
MODULE_LICENSE("GPL v2");
