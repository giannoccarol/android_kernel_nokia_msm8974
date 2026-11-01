/*
 * ST LIS2HH12 3-axis accelerometer, polled input driver for the
 * Nokia Lumia 1520 ("bandit").
 *
 * Copyright (c) 2026, Lorenzo Giannoccaro.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Written from the ST LIS2HH12 datasheet (DocID025821); the overall
 * structure (regulators, polled delayed_work, enable/poll_delay sysfs)
 * follows the usual ST input drivers such as drivers/input/misc/lis3dh_acc.c
 * (STMicroelectronics, Matteo Dameno, Carmine Iascone, Samuel Huo), but no
 * code is copied from it.
 *
 * Hardware: i2c-5 (BLSP2 QUP6), address 0x1e, WHO_AM_I = 0x41, no IRQ line
 * known. Supplies: PM8941 L18 ("vdd", 2.85 V) and LVS3 ("vio", 1.8 V).
 *
 * Interface (contract with the sensors HAL):
 *   input device name   "bandit-accel"
 *   events              EV_ABS ABS_X/ABS_Y/ABS_Z = raw signed 16-bit counts
 *                       in chip axes (no rotation here), then EV_SYN
 *   sysfs (input dev)   enable      0/1, rw
 *                       poll_delay  ms, rw, clamped to [3, 1000]
 *
 * Scale: full scale +/-2 g (CTRL4 FS = 00), sensitivity 0.061 mg/LSB:
 *   1 LSB = 0.061e-3 * 9.80665 m/s^2 = 5.98206e-4 m/s^2
 * (equivalently m/s^2 = counts * 9.80665 / 16393.4).
 *
 * The output data rate is the slowest of 10/50/100/200/400 Hz that gives at
 * least one new sample per poll. Polling is jiffy based (HZ = 300 on this
 * kernel), so the effective period is poll_delay rounded up to 3.33 ms.
 */

#include <linux/delay.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/pm.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/sysfs.h>
#include <linux/workqueue.h>

#define LIS2HH12_DEV_NAME	"bandit-accel"

#define LIS2HH12_WHO_AM_I	0x0f
#define LIS2HH12_WHO_AM_I_VAL	0x41
#define LIS2HH12_CTRL1		0x20
#define LIS2HH12_CTRL2		0x21
#define LIS2HH12_CTRL3		0x22
#define LIS2HH12_CTRL4		0x23
#define LIS2HH12_CTRL5		0x24
#define LIS2HH12_OUT_X_L	0x28	/* X_L, X_H, Y_L, Y_H, Z_L, Z_H */

/* CTRL1: HR | ODR[2:0] | BDU | ZEN | YEN | XEN */
#define CTRL1_HR		0x80
#define CTRL1_ODR_SHIFT		4
#define CTRL1_BDU		0x08
#define CTRL1_XYZ_EN		0x07

/* CTRL4: BW[1:0] | FS[1:0] | BW_SCALE_ODR | IF_ADD_INC | I2C_DISABLE | SIM */
#define CTRL4_FS_2G		0x00
#define CTRL4_IF_ADD_INC	0x04

#define LIS2HH12_MIN_POLL_MS	3	/* 400 Hz is the fastest ODR used */
#define LIS2HH12_MAX_POLL_MS	1000
#define LIS2HH12_DEF_POLL_MS	200
#define LIS2HH12_SETTLE_ODRS	2	/* samples dropped after power-up */

static const struct {
	unsigned int hz;
	u8 bits;
} lis2hh12_odr_table[] = {
	{  10, 1 },
	{  50, 2 },
	{ 100, 3 },
	{ 200, 4 },
	{ 400, 5 },
};

struct lis2hh12_data {
	struct i2c_client *client;
	struct input_dev *input;
	struct regulator *vdd;
	struct regulator *vio;
	struct delayed_work work;
	struct mutex lock;		/* protects enabled, poll_ms, chip config */
	bool enabled;
	unsigned int poll_ms;
	unsigned int odr_hz;
	unsigned long valid_after;	/* jiffies: first sample to report */
};

static int lis2hh12_write(struct lis2hh12_data *d, u8 reg, u8 val)
{
	int ret = i2c_smbus_write_byte_data(d->client, reg, val);

	if (ret < 0)
		dev_err(&d->client->dev, "write 0x%02x=0x%02x failed: %d\n",
			reg, val, ret);
	return ret;
}

static unsigned int lis2hh12_pick_odr(unsigned int poll_ms, u8 *bits)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(lis2hh12_odr_table) - 1; i++)
		if (lis2hh12_odr_table[i].hz * poll_ms >= 1000)
			break;
	*bits = lis2hh12_odr_table[i].bits;
	return lis2hh12_odr_table[i].hz;
}

/* Program CTRL1 for the current poll rate, or ODR = 0 (power-down). */
static int lis2hh12_set_power(struct lis2hh12_data *d, bool on)
{
	u8 bits = 0;
	int ret;

	if (!on)
		return lis2hh12_write(d, LIS2HH12_CTRL1,
				      CTRL1_HR | CTRL1_BDU | CTRL1_XYZ_EN);

	d->odr_hz = lis2hh12_pick_odr(d->poll_ms, &bits);
	ret = lis2hh12_write(d, LIS2HH12_CTRL1, CTRL1_HR |
			     (bits << CTRL1_ODR_SHIFT) | CTRL1_BDU |
			     CTRL1_XYZ_EN);
	if (ret < 0)
		return ret;

	d->valid_after = jiffies +
		msecs_to_jiffies(DIV_ROUND_UP(LIS2HH12_SETTLE_ODRS * 1000,
					      d->odr_hz));
	return 0;
}

static int lis2hh12_chip_init(struct lis2hh12_data *d)
{
	int ret;

	/* Power-down first, then filters/interrupts off, 2 g, auto-increment */
	ret = lis2hh12_set_power(d, false);
	if (!ret)
		ret = lis2hh12_write(d, LIS2HH12_CTRL2, 0x00);
	if (!ret)
		ret = lis2hh12_write(d, LIS2HH12_CTRL3, 0x00);
	if (!ret)
		ret = lis2hh12_write(d, LIS2HH12_CTRL4,
				     CTRL4_FS_2G | CTRL4_IF_ADD_INC);
	if (!ret)
		ret = lis2hh12_write(d, LIS2HH12_CTRL5, 0x00);
	return ret;
}

static void lis2hh12_work(struct work_struct *work)
{
	struct lis2hh12_data *d = container_of(to_delayed_work(work),
					       struct lis2hh12_data, work);
	u8 buf[6];
	int ret;

	ret = i2c_smbus_read_i2c_block_data(d->client, LIS2HH12_OUT_X_L,
					    sizeof(buf), buf);
	if (ret != sizeof(buf)) {
		dev_err_ratelimited(&d->client->dev, "data read failed: %d\n",
				    ret);
	} else if (!time_before(jiffies, d->valid_after)) {
		input_report_abs(d->input, ABS_X, (s16)(buf[1] << 8 | buf[0]));
		input_report_abs(d->input, ABS_Y, (s16)(buf[3] << 8 | buf[2]));
		input_report_abs(d->input, ABS_Z, (s16)(buf[5] << 8 | buf[4]));
		input_sync(d->input);
	}

	if (d->enabled)
		queue_delayed_work(system_freezable_wq, &d->work,
				   msecs_to_jiffies(d->poll_ms));
}

/* Caller holds d->lock. */
static int lis2hh12_enable(struct lis2hh12_data *d, bool on)
{
	int ret;

	if (on == d->enabled)
		return 0;

	if (on) {
		ret = lis2hh12_set_power(d, true);
		if (ret < 0)
			return ret;
		d->enabled = true;
		queue_delayed_work(system_freezable_wq, &d->work,
				   msecs_to_jiffies(d->poll_ms));
	} else {
		d->enabled = false;
		cancel_delayed_work_sync(&d->work);
		lis2hh12_set_power(d, false);
	}
	return 0;
}

static ssize_t lis2hh12_enable_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct lis2hh12_data *d = input_get_drvdata(to_input_dev(dev));

	return sprintf(buf, "%d\n", d->enabled ? 1 : 0);
}

static ssize_t lis2hh12_enable_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct lis2hh12_data *d = input_get_drvdata(to_input_dev(dev));
	unsigned long val;
	int ret;

	if (kstrtoul(buf, 0, &val) || val > 1)
		return -EINVAL;

	mutex_lock(&d->lock);
	ret = lis2hh12_enable(d, val);
	mutex_unlock(&d->lock);

	return ret < 0 ? ret : count;
}

static ssize_t lis2hh12_poll_delay_show(struct device *dev,
					struct device_attribute *attr,
					char *buf)
{
	struct lis2hh12_data *d = input_get_drvdata(to_input_dev(dev));

	return sprintf(buf, "%u\n", d->poll_ms);
}

static ssize_t lis2hh12_poll_delay_store(struct device *dev,
					 struct device_attribute *attr,
					 const char *buf, size_t count)
{
	struct lis2hh12_data *d = input_get_drvdata(to_input_dev(dev));
	unsigned long val;
	int ret = 0;

	if (kstrtoul(buf, 0, &val))
		return -EINVAL;
	val = clamp_t(unsigned long, val, LIS2HH12_MIN_POLL_MS,
		      LIS2HH12_MAX_POLL_MS);

	mutex_lock(&d->lock);
	d->poll_ms = val;
	/* Reprogram the ODR; the running work picks up the new period. */
	if (d->enabled)
		ret = lis2hh12_set_power(d, true);
	mutex_unlock(&d->lock);

	return ret < 0 ? ret : count;
}

static DEVICE_ATTR(enable, S_IRUGO | S_IWUSR | S_IWGRP,
		   lis2hh12_enable_show, lis2hh12_enable_store);
static DEVICE_ATTR(poll_delay, S_IRUGO | S_IWUSR | S_IWGRP,
		   lis2hh12_poll_delay_show, lis2hh12_poll_delay_store);

static struct attribute *lis2hh12_attrs[] = {
	&dev_attr_enable.attr,
	&dev_attr_poll_delay.attr,
	NULL
};

static const struct attribute_group lis2hh12_attr_group = {
	.attrs = lis2hh12_attrs,
};

/* Optional supply: a missing one is reported and ignored. */
static struct regulator *lis2hh12_supply_on(struct device *dev,
					    const char *id)
{
	struct regulator *reg = regulator_get(dev, id);
	int ret;

	if (IS_ERR(reg)) {
		if (PTR_ERR(reg) == -EPROBE_DEFER)
			return reg;
		dev_warn(dev, "supply %s not found (%ld), continuing\n",
			 id, PTR_ERR(reg));
		return NULL;
	}
	ret = regulator_enable(reg);
	if (ret) {
		dev_err(dev, "cannot enable supply %s: %d\n", id, ret);
		regulator_put(reg);
		return ERR_PTR(ret);
	}
	return reg;
}

static void lis2hh12_supply_off(struct regulator *reg)
{
	if (reg) {
		regulator_disable(reg);
		regulator_put(reg);
	}
}

static int __devinit lis2hh12_probe(struct i2c_client *client,
				    const struct i2c_device_id *id)
{
	struct lis2hh12_data *d;
	struct input_dev *input;
	int ret;

	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_SMBUS_BYTE_DATA |
				     I2C_FUNC_SMBUS_I2C_BLOCK))
		return -ENODEV;

	d = devm_kzalloc(&client->dev, sizeof(*d), GFP_KERNEL);
	if (!d)
		return -ENOMEM;
	d->client = client;
	d->poll_ms = LIS2HH12_DEF_POLL_MS;
	mutex_init(&d->lock);
	INIT_DELAYED_WORK(&d->work, lis2hh12_work);

	d->vdd = lis2hh12_supply_on(&client->dev, "vdd");
	if (IS_ERR(d->vdd))
		return PTR_ERR(d->vdd);
	d->vio = lis2hh12_supply_on(&client->dev, "vio");
	if (IS_ERR(d->vio)) {
		ret = PTR_ERR(d->vio);
		d->vio = NULL;
		goto err_supply;
	}
	msleep(20);	/* let the boot procedure (trimming reload) finish */

	ret = i2c_smbus_read_byte_data(client, LIS2HH12_WHO_AM_I);
	if (ret < 0) {
		dev_err(&client->dev, "no response at 0x%02x: %d\n",
			client->addr, ret);
		goto err_supply;
	}
	if (ret != LIS2HH12_WHO_AM_I_VAL) {
		dev_err(&client->dev, "WHO_AM_I 0x%02x, expected 0x%02x\n",
			ret, LIS2HH12_WHO_AM_I_VAL);
		ret = -ENODEV;
		goto err_supply;
	}

	ret = lis2hh12_chip_init(d);
	if (ret < 0)
		goto err_supply;

	input = input_allocate_device();
	if (!input) {
		ret = -ENOMEM;
		goto err_supply;
	}
	input->name = LIS2HH12_DEV_NAME;
	input->id.bustype = BUS_I2C;
	input->dev.parent = &client->dev;
	__set_bit(EV_ABS, input->evbit);
	input_set_abs_params(input, ABS_X, -32768, 32767, 0, 0);
	input_set_abs_params(input, ABS_Y, -32768, 32767, 0, 0);
	input_set_abs_params(input, ABS_Z, -32768, 32767, 0, 0);
	input_set_drvdata(input, d);
	d->input = input;

	ret = input_register_device(input);
	if (ret) {
		input_free_device(input);
		goto err_supply;
	}

	ret = sysfs_create_group(&input->dev.kobj, &lis2hh12_attr_group);
	if (ret)
		goto err_input;

	i2c_set_clientdata(client, d);
	dev_info(&client->dev, "LIS2HH12 at 0x%02x, +/-2 g\n", client->addr);
	return 0;

err_input:
	input_unregister_device(input);
err_supply:
	lis2hh12_supply_off(d->vio);
	lis2hh12_supply_off(d->vdd);
	return ret;
}

static int __devexit lis2hh12_remove(struct i2c_client *client)
{
	struct lis2hh12_data *d = i2c_get_clientdata(client);

	sysfs_remove_group(&d->input->dev.kobj, &lis2hh12_attr_group);
	mutex_lock(&d->lock);
	lis2hh12_enable(d, false);
	mutex_unlock(&d->lock);
	input_unregister_device(d->input);
	lis2hh12_supply_off(d->vio);
	lis2hh12_supply_off(d->vdd);
	return 0;
}

#ifdef CONFIG_PM_SLEEP
/* Supplies stay on; only the chip goes to power-down while suspended. */
static int lis2hh12_suspend(struct device *dev)
{
	struct lis2hh12_data *d = i2c_get_clientdata(to_i2c_client(dev));

	mutex_lock(&d->lock);
	if (d->enabled) {
		cancel_delayed_work_sync(&d->work);
		lis2hh12_set_power(d, false);
	}
	mutex_unlock(&d->lock);
	return 0;
}

static int lis2hh12_resume(struct device *dev)
{
	struct lis2hh12_data *d = i2c_get_clientdata(to_i2c_client(dev));

	mutex_lock(&d->lock);
	if (d->enabled && !lis2hh12_set_power(d, true))
		queue_delayed_work(system_freezable_wq, &d->work,
				   msecs_to_jiffies(d->poll_ms));
	mutex_unlock(&d->lock);
	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(lis2hh12_pm_ops, lis2hh12_suspend, lis2hh12_resume);

static const struct i2c_device_id lis2hh12_id[] = {
	{ "lis2hh12_bandit", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, lis2hh12_id);

static const struct of_device_id lis2hh12_of_match[] = {
	{ .compatible = "st,lis2hh12" },
	{ }
};
MODULE_DEVICE_TABLE(of, lis2hh12_of_match);

static struct i2c_driver lis2hh12_driver = {
	.driver = {
		.name		= "lis2hh12_bandit",
		.owner		= THIS_MODULE,
		.of_match_table	= lis2hh12_of_match,
		.pm		= &lis2hh12_pm_ops,
	},
	.probe		= lis2hh12_probe,
	.remove		= __devexit_p(lis2hh12_remove),
	.id_table	= lis2hh12_id,
};

module_i2c_driver(lis2hh12_driver);

MODULE_AUTHOR("Lorenzo Giannoccaro");
MODULE_DESCRIPTION("ST LIS2HH12 accelerometer driver for Lumia 1520 (bandit)");
MODULE_LICENSE("GPL v2");
