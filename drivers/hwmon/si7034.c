
/* Silicon Laboratories Si7034 humidity and temperature sensor driver
 *
 * Copyright (C) 2014 Silicon Laboratories
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
 *
 * This device driver supports the Si7034 sensor IC.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/err.h>

/* Module documentation */
MODULE_DESCRIPTION("Silicon Laboratories Si7034 humidity and temperature sensor");
MODULE_AUTHOR("Quentin Stephenson <Quentin.Stephenson@silabs.com>");
MODULE_LICENSE("GPL");

/* Device Identification */
#define ID_SAMPLE		0xFF
#define ID_SI7034		0x22

/* Commands */
#define MEASURE_TEMPERATURE	0
#define MEASURE_HUMIDITY	1

#define BUF_SIZE		6


/* Global variables */
struct si7034 {
	struct mutex lock;
	struct device *dev;
	int device_id;
};	


/*
 * si7034_show_device_id() - show device ID in sysfs
 * @dev:  device
 * @attr: device attribute
 * @buf:  sysfs buffer (PAGE_SIZE) where measurement values are written to
 *
 * Will be called on read access to device_id sysfs attribute.
 * Returns number of bytes written into buffer, negative errno on error.
 */
static ssize_t si7034_show_device_id(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct si7034 *si7034 = i2c_get_clientdata(client);

	return sprintf(buf, "%d\n", si7034->device_id);
}

/*
 * si7034_command() - Send a command to the device
 * @client: I2C client device
 * @buf: pointer to command/result buffer
 *
 * Returns 0 on success.
 */
static int si7034_command(struct i2c_client *client, char *buf)
{
	int error;

	/* Send the 2-byte command */
	error = i2c_master_send(client, buf, 2);
	if (error < 0)
		return error;

	/* Receive the 6-byte result */
	error = i2c_master_recv(client, buf, BUF_SIZE);
	if (error < 0)
		return error;

	return 0;  /* Success */
}

/*
 * si7034_measure() - Take a measurement
 * @client:  I2C client device
 * @command: Command for measuring temperature or humidity
 *
 * Returns the 16-bit value from the Si7034 or errno on error.
 */
static int si7034_measure(struct i2c_client *client, int command)
{
	char buf[BUF_SIZE];
	int  error;

	/* Put the 2-byte command into the buffer */
	buf[0] = 0x7C;
	buf[1] = 0xA2;

	/* Send the command */
	error = si7034_command(client, buf);
	if (error < 0)
		return error;

	if ( command == MEASURE_TEMPERATURE )
		return (((int)buf[0]) << 8) + buf[1];
	else
		return (((int)buf[3]) << 8) + buf[4];
}

/*
 * Si7034_show_temperature() - show temperature measurement value in sysfs
 * @dev:  device
 * @attr: device attribute
 * @buf:  sysfs buffer (PAGE_SIZE) where measurement values are written to
 *
 * Will be called on read access to temp1_input sysfs attribute.
 * Returns number of bytes written into buffer, negative errno on error.
 */
static ssize_t Si7034_show_temperature(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct si7034 *si7034 = i2c_get_clientdata(client);
	int value;
	int temperature;

	/* Measure the temperature value */
	mutex_lock(&si7034->lock);
	value = si7034_measure(client, MEASURE_TEMPERATURE);
	mutex_unlock(&si7034->lock);
	if (value < 0) 
		return value;

	/* Convert the value to millidegrees Celsius */
	temperature = ((value*21875)>>13)-45000;

	return sprintf(buf, "%d\n", temperature);
}

/*
 * si7034_show_humidity() - show humidity measurement value in sysfs
 * @dev:  device
 * @attr: device attribute
 * @buf:  sysfs buffer (PAGE_SIZE) where measurement values are written to
 *
 * Will be called on read access to humidity1_input sysfs attribute.
 * Returns number of bytes written into buffer, negative errno on error.
 */
static ssize_t si7034_show_humidity(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct si7034 *si7034 = i2c_get_clientdata(client);
	int value;
	int humidity;

	/* Measure the humidity value */
	mutex_lock(&si7034->lock);
	value = si7034_measure(client, MEASURE_HUMIDITY);
	mutex_unlock(&si7034->lock);
	if (value < 0)
		return value;

	/* Convert the value to milli-percent (pcm) relative humidity */
	value = (value*12500)>>13;

	/* Limit the humidity to valid values */
	if (value < 0)
		humidity = 0;
	else if (value > 100000)
		humidity = 100000;
	else
		humidity = value;		

	return sprintf(buf, "%d\n", humidity);
}

/* Device attributes for sysfs */
static SENSOR_DEVICE_ATTR(device_id, S_IRUGO, si7034_show_device_id,
	NULL, 0);
static SENSOR_DEVICE_ATTR(temp1_input, S_IRUGO, Si7034_show_temperature,
	NULL, 0);
static SENSOR_DEVICE_ATTR(humidity1_input, S_IRUGO, si7034_show_humidity,
	NULL, 0);

/* Attribute array */
static struct attribute *si7034_attributes[] = {
	&sensor_dev_attr_device_id.dev_attr.attr,
	&sensor_dev_attr_temp1_input.dev_attr.attr,
	&sensor_dev_attr_humidity1_input.dev_attr.attr,
	NULL
};

/* Attribute group */
static const struct attribute_group si7034_attr_group = {
	.attrs = si7034_attributes,
};


/*
 * si7034_get_device_id() - Get the device ID from the device
 * @client: I2C client device
 * @id: pointer to device ID
 *
 * Returns 0 on success.
 */
static int si7034_get_device_id(struct i2c_client *client, int *id)
{
	char buf[BUF_SIZE];
	int  error;

	/* Put the 2-byte command into the buffer */
	buf[0] = 0xFC;
	buf[1] = 0xC9;

	/* Send the command */
	error = si7034_command(client, buf);
	if (error < 0)
		return error;

	/* Return the device ID */
	*id = buf[0];
	
	return 0;  /* Success */
}

/*
 * si7034_probe() - Verify that this is the correct driver for the device
 * @client: I2C client device
 * @dev_id: device ID
 *
 * Called by the I2C core when an entry in the ID table matches a
 * device's name.
 * Returns 0 on success.
 */
static int __devinit si7034_probe(struct i2c_client *client, 
	const struct i2c_device_id *dev_id)
{
	struct si7034 *si7034;
	int error;

	/* Allocate memory for global variables */
	si7034 = kzalloc(sizeof(*si7034), GFP_KERNEL);
	if (si7034 == NULL)
		return -ENOMEM;
	i2c_set_clientdata(client, si7034);

	/* Initialize the mutex */
	mutex_init(&si7034->lock);

	/* Get the device ID from the device */
	mutex_lock(&si7034->lock);
	error = si7034_get_device_id(client, &si7034->device_id);
	mutex_unlock(&si7034->lock);
	if (error < 0)
	{
		kfree(si7034);
		return error;
	}	 

	/* Validate the device ID */
	if ((si7034->device_id != ID_SAMPLE) && 
	    (si7034->device_id != ID_SI7034)) {
		kfree(si7034);
		return -ENODEV;
	}

	/* Create the sysfs group */
	error = sysfs_create_group(&client->dev.kobj, &si7034_attr_group);
	if (error)
	{
		kfree(si7034);
		return error;
	}	 

	/* Register with hwmon */
	si7034->dev = hwmon_device_register(&client->dev);
	if (IS_ERR(si7034->dev)) {
		error = PTR_ERR(si7034->dev);
		sysfs_remove_group(&client->dev.kobj, &si7034_attr_group);
		kfree(si7034);
		return error;
	}

	return 0;  /* Success */
}

/*
 * si7034_remove() - Remove this driver from the device
 * @client: I2C client device
 */
static int __devexit si7034_remove(struct i2c_client *client)
{
	struct si7034 *si7034 = i2c_get_clientdata(client);

	hwmon_device_unregister(si7034->dev);
	sysfs_remove_group(&client->dev.kobj, &si7034_attr_group);
	kfree(si7034);

	return 0;  /* Success */
}

/* Device ID table */
static const struct i2c_device_id si7034_id[] = {
	{"si7034", 0},
	{}
};

/* Add the device ID to the module device table */
MODULE_DEVICE_TABLE(i2c, si7034_id);

/* Driver structure */
static struct i2c_driver si7034_driver = {
	.driver.name	= "si7034",
	.probe		= si7034_probe,
	.remove		= __devexit_p(si7034_remove),
	.id_table	= si7034_id,
};

/*
 * si7034_init() - Initialize the device driver
 */
static int __init si7034_init(void)
{
	/* Register this I2C chip driver */
	return i2c_add_driver(&si7034_driver);
}
module_init(si7034_init);

/*
 * si7034_exit() - Exit the device driver
 */
static void __exit si7034_exit(void)
{
	/* Unregister this I2C chip driver */
	i2c_del_driver(&si7034_driver);
}
module_exit(si7034_exit);

