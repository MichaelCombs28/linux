/* Silicon Laboratories Si7005 humidity and temperature sensor driver
 *
 * Copyright (C) 2012 Silicon Laboratories
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA

 * This device driver supports the Si7005 and Si7015 sensor ICs.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/err.h>

/* Module documentation */
MODULE_DESCRIPTION("Silicon Laboratories Si7005 humidity and temperature sensor");
MODULE_AUTHOR("Quentin Stephenson <Quentin.Stephenson@silabs.com>");
MODULE_LICENSE("GPL");

/* Si7005 Registers */
#define REG_STATUS		0x00
#define REG_DATA		0x01
#define REG_CONFIG		0x03
#define REG_ID			0x11

/* Status Register */
#define STATUS_NOT_READY	0x01

/* Config Register */
#define CONFIG_START		0x01
#define CONFIG_HEAT		0x02
#define CONFIG_HUMIDITY		0x00
#define CONFIG_TEMPERATURE	0x10
#define CONFIG_FAST		0x20

/* ID Register */
#define ID_SI7005		0x50
#define ID_SI7015		0xF0

/* Coefficients */
#define TEMPERATURE_OFFSET	50
#define TEMPERATURE_SLOPE	32
#define HUMIDITY_OFFSET		24
#define HUMIDITY_SLOPE		16
#define SCALAR			16384
#define A0			(-78388)	/* -4.7844   * SCALAR */
#define A1			6567		/*  0.4008   * SCALAR */
#define A2			(-64)		/* -0.00393  * SCALAR */
#define Q0_7005			3233		/*  0.1973   * SCALAR */
#define Q1_7005			39		/*  0.00237  * SCALAR */
#define Q0_7015			986		/*  0.060162 * SCALAR */
#define Q1_7015			8		/*  0.000508 * SCALAR */

/* Global variables */
static struct mutex si7005_lock;
static struct device *si7005_dev;
static int si7005_device_id;
static int si7005_temperature = 25000;

/*
 * si7005_show_device_id() - show device ID in sysfs
 * @dev:  device
 * @attr: device attribute
 * @buf:  sysfs buffer (PAGE_SIZE) where measurement values are written to
 *
 * Will be called on read access to device_id sysfs attribute.
 * Returns number of bytes written into buffer, negative errno on error.
 */
static ssize_t si7005_show_device_id(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", si7005_device_id);
}

/*
 * si7005_measure() - Take a measurement
 * @client: I2C client device
 * @config: Configuration for measuring temperature or humidity
 *
 * Returns the 16-bit value from the Si7005's data register or errno on error.
 */
static int si7005_measure(struct i2c_client *client, u8 config)
{
	unsigned long timeout;
	int status;
	int error;
	union {
		u8  byte[4];
		s32 value;
	} data;

	/* Start the conversion */
	error = i2c_smbus_write_byte_data(client, REG_CONFIG,
		CONFIG_START | config);
	if (error)
		return error;

	/* Initialize working variables */
	status	= STATUS_NOT_READY;
	timeout = jiffies + msecs_to_jiffies(500);

	/* Poll until the conversion is ready */
	while (status & STATUS_NOT_READY) {
		status = i2c_smbus_read_byte_data(client, REG_STATUS);
		if (status < 0) 
			return status;
		if (time_after(jiffies,timeout)) 
			return -ETIMEDOUT;
	}

	/* Stop the conversion */
	error = i2c_smbus_write_byte_data(client, REG_CONFIG, config);
	if (error)
		return error;

	/* Read the data */
	data.value = i2c_smbus_read_word_data(client, REG_DATA);
	if (data.value < 0)
		return data.value;

	return (data.byte[0] * 256) + data.byte[1];
}

/*
 * si7005_show_temperature() - show temperature measurement value in sysfs
 * @dev:  device
 * @attr: device attribute
 * @buf:  sysfs buffer (PAGE_SIZE) where measurement values are written to
 *
 * Will be called on read access to temp1_input sysfs attribute.
 * Returns number of bytes written into buffer, negative errno on error.
 */
static ssize_t si7005_show_temperature(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int value;

	/* Measure the temperature */
	mutex_lock(&si7005_lock);
	value = si7005_measure(to_i2c_client(dev), CONFIG_TEMPERATURE);
	mutex_unlock(&si7005_lock);
	if (value < 0) 
		return value;
		
	/* Convert the temperature to millidegree Celsius */
	si7005_temperature = (((value >> 2) * 1000) / TEMPERATURE_SLOPE) -
		(TEMPERATURE_OFFSET * 1000);

	return sprintf(buf, "%d\n", si7005_temperature);
}

/*
 * si7005_show_humidity() - show humidity measurement value in sysfs
 * @dev:  device
 * @attr: device attribute
 * @buf:  sysfs buffer (PAGE_SIZE) where measurement values are written to
 *
 * Will be called on read access to humidity1_input sysfs attribute.
 * Returns number of bytes written into buffer, negative errno on error.
 */
static ssize_t si7005_show_humidity(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int value;
	int curve;
	int linear;
	int humidity;

	/* Measure the humidity */
	mutex_lock(&si7005_lock);
	value = si7005_measure(to_i2c_client(dev), CONFIG_HUMIDITY);
	mutex_unlock(&si7005_lock);
	if (value < 0)
		return value;

	/* Convert the humidity to milli-percent (pcm) */
	curve = ((value >> 4) * 1000 )/ HUMIDITY_SLOPE - HUMIDITY_OFFSET * 1000;

	if (si7005_device_id == ID_SI7005) {
		/* Linearization is needed for Si7005 */
		linear = (curve * SCALAR - (curve * curve * A2) / 1000 - 
			curve * A1 - A0 * 1000) / SCALAR;

		/* Temperature Compensation */
		linear = (linear * SCALAR + (si7005_temperature - 30000) *
			((linear * Q1_7005) / 1000 + Q0_7005)) / SCALAR;
	} else {
		/* Linearization is done within the Si7015 */
		linear = curve;

		/* Temperature Compensation */
		linear = (linear * SCALAR + (si7005_temperature - 30000) *
			((linear * Q1_7015) / 1000 + Q0_7015)) / SCALAR;
	}	 

	/* Limit the humidity to valid values */
	if (linear < 0)
		humidity = 0;
	else if (linear > 100000)
		humidity = 100000;
	else
		humidity = linear;		

	return sprintf(buf, "%d\n", humidity);
}

/* Device attributes for sysfs */
static SENSOR_DEVICE_ATTR(device_id, S_IRUGO, si7005_show_device_id,
	NULL, 0);
static SENSOR_DEVICE_ATTR(temp1_input, S_IRUGO, si7005_show_temperature,
	NULL, 0);
static SENSOR_DEVICE_ATTR(humidity1_input, S_IRUGO, si7005_show_humidity,
	NULL, 0);

static struct attribute *si7005_attributes[] = {
	&sensor_dev_attr_device_id.dev_attr.attr,
	&sensor_dev_attr_temp1_input.dev_attr.attr,
	&sensor_dev_attr_humidity1_input.dev_attr.attr,
	NULL
};

/* Attribute group */
static const struct attribute_group si7005_attr_group = {
	.attrs = si7005_attributes,
};

/*
 * si7005_probe() - Verify that this is the correct driver for the device
 * @client: I2C client device
 * @dev_id: device ID
 *
 * Called by the I2C core when an entry in the ID table matches a
 * device's name.
 * Returns 0 on success.
 */
static int __devinit si7005_probe(struct i2c_client *client, 
	const struct i2c_device_id *dev_id)
{
	int error;

	/* Read the ID register */
	si7005_device_id = i2c_smbus_read_byte_data(client, REG_ID);
	if (si7005_device_id < 0)
		return si7005_device_id;

	/* Verify that the ID is correct */
	if ((si7005_device_id != ID_SI7005) && (si7005_device_id != ID_SI7015))
		return -ENODEV;

	mutex_init(&si7005_lock);

	/* Create the sysfs group */
	error = sysfs_create_group(&client->dev.kobj, &si7005_attr_group);
	if (error)
		return error;

	/* Register with hwmon */
	si7005_dev = hwmon_device_register(&client->dev);
	if (IS_ERR(si7005_dev)) {
		error = PTR_ERR(si7005_dev);
		sysfs_remove_group(&client->dev.kobj, &si7005_attr_group);
		return error;
	}

	return 0;  /* Success */
}

/*
 * si7005_remove() - Remove this driver from the device
 * @client: I2C client device
 */
static int __devexit si7005_remove(struct i2c_client *client)
{
	hwmon_device_unregister(si7005_dev);
	sysfs_remove_group(&client->dev.kobj, &si7005_attr_group);

	return 0;  /* Success */
}

/* Device ID table */
static const struct i2c_device_id si7005_id[] = {
	{"si7005", 0},
	{}
};

/* Add the device ID to the module device table */
MODULE_DEVICE_TABLE(i2c, si7005_id);

/* Driver structure */
static struct i2c_driver si7005_driver = {
	.driver.name	= "si7005",
	.probe		= si7005_probe,
	.remove		= __devexit_p(si7005_remove),
	.id_table	= si7005_id,
};

/*
 * si7005_init() - Initialize the device driver
 */
static int __init si7005_init(void)
{
	/* Register this I2C chip driver */
	return i2c_add_driver(&si7005_driver);
}
module_init(si7005_init);

/*
 * si7005_exit() - Exit the device driver
 */
static void __exit si7005_exit(void)
{
	/* Unregister this I2C chip driver */
	i2c_del_driver(&si7005_driver);
}
module_exit(si7005_exit);

