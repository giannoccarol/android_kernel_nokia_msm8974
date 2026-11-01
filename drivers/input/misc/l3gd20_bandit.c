/*
 * ST L3GD20-compatible 3-axis gyroscope, polled input driver for the
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
 * Written from the ST L3GD20 datasheet (DocID022116) register map; the
 * driver structure follows the usual ST polled input drivers (for example
 * drivers/input/misc/lis3dh_acc.c, STMicroelectronics: Matteo Dameno,
 * Carmine Iascone, Samuel Huo), but no code is copied from them.
 *
 * Hardware: i2c-4 (BLSP1 QUP3), address 0x68. On this phone the part does
 * NOT identify as an L3GD20: WHO_AM_I (0x0f) reads 0x00 instead of 0xd4 and
 * registers 0x00-0x0e hold random values. It does behave like one from 0x20
 * upwards (CTRL_REG1 = 0x0f powers it on, OUT_X_L..OUT_Z_H at 0x28-0x2d
 * change, OUT_TEMP 0x26, FIFO_SRC 0x2f = 0x20), so presence is checked with
 * a CTRL_REG1/CTRL_REG4 write and read-back instead of WHO_AM_I.
 * Supplies: PM8941 L18 ("vdd", 2.85 V) and LVS3 ("vio", 1.8 V).
 *
 * Interface (contract with the sensors HAL):
 *   input device name   "bandit-gyro"
 *   events              EV_ABS ABS_X/ABS_Y/ABS_Z = raw signed 16-bit counts
 *                       in chip axes (no rotation here), then EV_SYN
 *   sysfs (input dev)   enable      0/1, rw
 *                       poll_delay  ms, rw, clamped to [2, 1000]
 *
 * Scale: full scale +/-2000 dps (CTRL_REG4 FS = 10), sensitivity
 * 70 mdps/LSB:
 *   1 LSB = 0.070 dps = 0.070 * pi / 180 rad/s = 1.221730e-3 rad/s
 *
 * The output data rate is the slowest of 95/190/380/760 Hz that gives at
 * least one new sample per poll. After power-up the samples of the first
 * 250 ms (turn-on time) are discarded. Polling is jiffy based (HZ = 300 on
 * this kernel), so the effective period is poll_delay rounded up to 3.33 ms.
 *
 * TODO: the interrupt lines are TLMM GPIO 66 and 68 (INT1/DRDY, mapping not
 * verified); data-ready interrupts could replace polling.
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

#define L3GD20_DEV_NAME		"bandit-gyro"

#define L3GD20_CTRL_REG1	0x20
#define L3GD20_CTRL_REG2	0x21
#define L3GD20_CTRL_REG3	0x22
#define L3GD20_CTRL_REG4	0x23
#define L3GD20_CTRL_REG5	0x24
#define L3GD20_OUT_X_L		0x28	/* X_L, X_H, Y_L, Y_H, Z_L, Z_H */
#define L3GD20_FIFO_CTRL	0x2e
#define L3GD20_AUTO_INC		0x80	/* sub-address MSB: auto-increment */

/* CTRL_REG1: DR[1:0] | BW[1:0] | PD | ZEN | YEN | XEN */
#define CTRL1_PD		0x08
#define CTRL1_XYZ_EN		0x07

/* CTRL_REG4: BDU | BLE | FS[1:0] | - | - | - | SIM */
#define CTRL4_BDU		0x80
#define CTRL4_FS_2000DPS	0x20
#define L3GD20_CTRL4_VAL	(CTRL4_BDU | CTRL4_FS_2000DPS)

#define L3GD20_MIN_POLL_MS	2	/* 760 Hz is the fastest ODR */
#define L3GD20_MAX_POLL_MS	1000
#define L3GD20_DEF_POLL_MS	200
#define L3GD20_TURN_ON_MS	250	/* discard window after power-up */
#define L3GD20_SETTLE_ODRS	2	/* discard window after an ODR change */

/* DR/BW pairs: 95 Hz/25 Hz, 190 Hz/70 Hz, 380 Hz/100 Hz, 760 Hz/100 Hz */
static const struct {
	unsigned int hz;
	u8 drbw;
} l3gd20_odr_table[] = {
	{  95, 0x10 },
	{ 190, 0x70 },
	{ 380, 0xb0 },
	{ 760, 0xf0 },
};

struct l3gd20_data {
	struct i2c_client *client;
	struct input_dev *input;
	struct regulator *vdd;
	struct regulator *vio;
	struct delayed_work work;
	struct mutex lock;		/* protects enabled, poll_ms, chip config */
	bool enabled;
	bool powered;
	unsigned int poll_ms;
	unsigned int odr_hz;
	unsigned long valid_after;	/* jiffies: first sample to report */
};

static int l3gd20_write(struct l3gd20_data *d, u8 reg, u8 val)
{
	int ret = i2c_smbus_write_byte_data(d->client, reg, val);

	if (ret < 0)
		dev_err(&d->client->dev, "write 0x%02x=0x%02x failed: %d\n",
			reg, val, ret);
	return ret;
}

/* Write a register and check that it reads back unchanged. */
static int l3gd20_write_verify(struct l3gd20_data *d, u8 reg, u8 val)
{
	int ret = l3gd20_write(d, reg, val);

	if (ret < 0)
		return ret;
	ret = i2c_smbus_read_byte_data(d->client, reg);
	if (ret < 0)
		return ret;
	if (ret != val) {
		dev_err(&d->client->dev, "reg 0x%02x reads 0x%02x, wrote 0x%02x\n",
			reg, ret, val);
		return -ENODEV;
	}
	return 0;
}

static unsigned int l3gd20_pick_odr(unsigned int poll_ms, u8 *drbw)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(l3gd20_odr_table) - 1; i++)
		if (l3gd20_odr_table[i].hz * poll_ms >= 1000)
			break;
	*drbw = l3gd20_odr_table[i].drbw;
	return l3gd20_odr_table[i].hz;
}

/* Program CTRL_REG1 for the current poll rate, or power-down (PD = 0). */
static int l3gd20_set_power(struct l3gd20_data *d, bool on)
{
	unsigned int settle_ms;
	u8 drbw = 0;
	int ret;

	if (!on) {
		ret = l3gd20_write(d, L3GD20_CTRL_REG1, CTRL1_XYZ_EN);
		if (!ret)
			d->powered = false;
		return ret;
	}

	d->odr_hz = l3gd20_pick_odr(d->poll_ms, &drbw);
	ret = l3gd20_write(d, L3GD20_CTRL_REG1, drbw | CTRL1_PD | CTRL1_XYZ_EN);
	if (ret < 0)
		return ret;

	/* Full turn-on time from power-down, filter settling otherwise. */
	if (d->powered)
		settle_ms = DIV_ROUND_UP(L3GD20_SETTLE_ODRS * 1000, d->odr_hz);
	else
		settle_ms = L3GD20_TURN_ON_MS;
	d->valid_after = jiffies + msecs_to_jiffies(settle_ms);
	d->powered = true;
	return 0;
}

static int l3gd20_chip_init(struct l3gd20_data *d)
{
	int ret;

	/* Power-down with axes enabled (reset value) proves the part is there */
	ret = l3gd20_write_verify(d, L3GD20_CTRL_REG1, CTRL1_XYZ_EN);
	if (ret < 0)
		return ret;
	/* No high-pass filter, no interrupts, no FIFO (bypass mode) */
	ret = l3gd20_write(d, L3GD20_CTRL_REG2, 0x00);
	if (!ret)
		ret = l3gd20_write(d, L3GD20_CTRL_REG3, 0x00);
	if (!ret)
		ret = l3gd20_write(d, L3GD20_CTRL_REG5, 0x00);
	if (!ret)
		ret = l3gd20_write(d, L3GD20_FIFO_CTRL, 0x00);
	if (ret < 0)
		return ret;
	/* BDU, 2000 dps, little endian */
	return l3gd20_write_verify(d, L3GD20_CTRL_REG4, L3GD20_CTRL4_VAL);
}

static void l3gd20_work(struct work_struct *work)
{
	struct l3gd20_data *d = container_of(to_delayed_work(work),
					     struct l3gd20_data, work);
	u8 buf[6];
	int ret;

	if (time_before(jiffies, d->valid_after))
		goto resched;

	ret = i2c_smbus_read_i2c_block_data(d->client,
					    L3GD20_OUT_X_L | L3GD20_AUTO_INC,
					    sizeof(buf), buf);
	if (ret != sizeof(buf)) {
		dev_err_ratelimited(&d->client->dev, "data read failed: %d\n",
				    ret);
		goto resched;
	}
	input_report_abs(d->input, ABS_X, (s16)(buf[1] << 8 | buf[0]));
	input_report_abs(d->input, ABS_Y, (s16)(buf[3] << 8 | buf[2]));
	input_report_abs(d->input, ABS_Z, (s16)(buf[5] << 8 | buf[4]));
	input_sync(d->input);

resched:
	if (d->enabled)
		queue_delayed_work(system_freezable_wq, &d->work,
				   msecs_to_jiffies(d->poll_ms));
}

/* Caller holds d->lock. */
static int l3gd20_enable(struct l3gd20_data *d, bool on)
{
	int ret;

	if (on == d->enabled)
		return 0;

	if (on) {
		ret = l3gd20_set_power(d, true);
		if (ret < 0)
			return ret;
		d->enabled = true;
		queue_delayed_work(system_freezable_wq, &d->work,
				   msecs_to_jiffies(d->poll_ms));
	} else {
		d->enabled = false;
		cancel_delayed_work_sync(&d->work);
		l3gd20_set_power(d, false);
	}
	return 0;
}

static ssize_t l3gd20_enable_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct l3gd20_data *d = input_get_drvdata(to_input_dev(dev));

	return sprintf(buf, "%d\n", d->enabled ? 1 : 0);
}

static ssize_t l3gd20_enable_store(struct device *dev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	struct l3gd20_data *d = input_get_drvdata(to_input_dev(dev));
	unsigned long val;
	int ret;

	if (kstrtoul(buf, 0, &val) || val > 1)
		return -EINVAL;

	mutex_lock(&d->lock);
	ret = l3gd20_enable(d, val);
	mutex_unlock(&d->lock);

	return ret < 0 ? ret : count;
}

static ssize_t l3gd20_poll_delay_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct l3gd20_data *d = input_get_drvdata(to_input_dev(dev));

	return sprintf(buf, "%u\n", d->poll_ms);
}

static ssize_t l3gd20_poll_delay_store(struct device *dev,
				       struct device_attribute *attr,
				       const char *buf, size_t count)
{
	struct l3gd20_data *d = input_get_drvdata(to_input_dev(dev));
	unsigned long val;
	int ret = 0;

	if (kstrtoul(buf, 0, &val))
		return -EINVAL;
	val = clamp_t(unsigned long, val, L3GD20_MIN_POLL_MS,
		      L3GD20_MAX_POLL_MS);

	mutex_lock(&d->lock);
	d->poll_ms = val;
	/* Reprogram the ODR; the running work picks up the new period. */
	if (d->enabled)
		ret = l3gd20_set_power(d, true);
	mutex_unlock(&d->lock);

	return ret < 0 ? ret : count;
}

static DEVICE_ATTR(enable, S_IRUGO | S_IWUSR | S_IWGRP,
		   l3gd20_enable_show, l3gd20_enable_store);
static DEVICE_ATTR(poll_delay, S_IRUGO | S_IWUSR | S_IWGRP,
		   l3gd20_poll_delay_show, l3gd20_poll_delay_store);

static struct attribute *l3gd20_attrs[] = {
	&dev_attr_enable.attr,
	&dev_attr_poll_delay.attr,
	NULL
};

static const struct attribute_group l3gd20_attr_group = {
	.attrs = l3gd20_attrs,
};

/* Optional supply: a missing one is reported and ignored. */
static struct regulator *l3gd20_supply_on(struct device *dev, const char *id)
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

static void l3gd20_supply_off(struct regulator *reg)
{
	if (reg) {
		regulator_disable(reg);
		regulator_put(reg);
	}
}

static int __devinit l3gd20_probe(struct i2c_client *client,
				  const struct i2c_device_id *id)
{
	struct l3gd20_data *d;
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
	d->poll_ms = L3GD20_DEF_POLL_MS;
	mutex_init(&d->lock);
	INIT_DELAYED_WORK(&d->work, l3gd20_work);

	d->vdd = l3gd20_supply_on(&client->dev, "vdd");
	if (IS_ERR(d->vdd))
		return PTR_ERR(d->vdd);
	d->vio = l3gd20_supply_on(&client->dev, "vio");
	if (IS_ERR(d->vio)) {
		ret = PTR_ERR(d->vio);
		d->vio = NULL;
		goto err_supply;
	}
	msleep(10);	/* let the boot procedure (trimming reload) finish */

	/* WHO_AM_I is not usable here (reads 0x00): only require an ACK. */
	ret = i2c_smbus_read_byte_data(client, L3GD20_CTRL_REG1);
	if (ret < 0) {
		dev_err(&client->dev, "no response at 0x%02x: %d\n",
			client->addr, ret);
		goto err_supply;
	}

	ret = l3gd20_chip_init(d);
	if (ret < 0)
		goto err_supply;

	input = input_allocate_device();
	if (!input) {
		ret = -ENOMEM;
		goto err_supply;
	}
	input->name = L3GD20_DEV_NAME;
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

	ret = sysfs_create_group(&input->dev.kobj, &l3gd20_attr_group);
	if (ret)
		goto err_input;

	i2c_set_clientdata(client, d);
	dev_info(&client->dev, "L3GD20-compatible gyro at 0x%02x, 2000 dps\n",
		 client->addr);
	return 0;

err_input:
	input_unregister_device(input);
err_supply:
	l3gd20_supply_off(d->vio);
	l3gd20_supply_off(d->vdd);
	return ret;
}

static int __devexit l3gd20_remove(struct i2c_client *client)
{
	struct l3gd20_data *d = i2c_get_clientdata(client);

	sysfs_remove_group(&d->input->dev.kobj, &l3gd20_attr_group);
	mutex_lock(&d->lock);
	l3gd20_enable(d, false);
	mutex_unlock(&d->lock);
	input_unregister_device(d->input);
	l3gd20_supply_off(d->vio);
	l3gd20_supply_off(d->vdd);
	return 0;
}

#ifdef CONFIG_PM_SLEEP
/* Supplies stay on; only the chip goes to power-down while suspended. */
static int l3gd20_suspend(struct device *dev)
{
	struct l3gd20_data *d = i2c_get_clientdata(to_i2c_client(dev));

	mutex_lock(&d->lock);
	if (d->enabled) {
		cancel_delayed_work_sync(&d->work);
		l3gd20_set_power(d, false);
	}
	mutex_unlock(&d->lock);
	return 0;
}

static int l3gd20_resume(struct device *dev)
{
	struct l3gd20_data *d = i2c_get_clientdata(to_i2c_client(dev));

	mutex_lock(&d->lock);
	if (d->enabled && !l3gd20_set_power(d, true))
		queue_delayed_work(system_freezable_wq, &d->work,
				   msecs_to_jiffies(d->poll_ms));
	mutex_unlock(&d->lock);
	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(l3gd20_pm_ops, l3gd20_suspend, l3gd20_resume);

static const struct i2c_device_id l3gd20_id[] = {
	{ "l3gd20_bandit", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, l3gd20_id);

static const struct of_device_id l3gd20_of_match[] = {
	{ .compatible = "st,l3gd20-bandit" },
	{ }
};
MODULE_DEVICE_TABLE(of, l3gd20_of_match);

static struct i2c_driver l3gd20_driver = {
	.driver = {
		.name		= "l3gd20_bandit",
		.owner		= THIS_MODULE,
		.of_match_table	= l3gd20_of_match,
		.pm		= &l3gd20_pm_ops,
	},
	.probe		= l3gd20_probe,
	.remove		= __devexit_p(l3gd20_remove),
	.id_table	= l3gd20_id,
};

module_i2c_driver(l3gd20_driver);

MODULE_AUTHOR("Lorenzo Giannoccaro");
MODULE_DESCRIPTION("L3GD20-compatible gyroscope driver for Lumia 1520 (bandit)");
MODULE_LICENSE("GPL v2");
