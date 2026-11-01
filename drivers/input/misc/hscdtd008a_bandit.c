/*
 * Alps HSCDTD008A 3-axis geomagnetic sensor, polled input driver for the
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
 * Written from the Alps HSCDTD008A data sheet (Rev. 2.9) register map.
 * The chip programming (CTRL4 = MMD | RS, forced measurement through
 * CTRL3 FRC and STAT DRDY, the STB 0x55 -> 0xAA -> 0x55 self-test, the
 * 3 ms measurement and 50 ms power-on delays) follows the HSCDTD008A
 * support in mainline drivers/iio/magnetometer/ak8974.c (Samu Onkalo,
 * Linus Walleij; HSCDTD008A variant by Nick Reitemeyer, 2020). The
 * register bit layout was cross-checked with the MikroElektronika
 * "Compass 6 click" library (mikrosdk_click_v2) and Zoltan Hudak's mbed
 * HSCDTD008A library; no code is copied from those.
 *
 * Hardware: i2c-5 (BLSP2 QUP6), address 0x0f, WIA = 0x49, no IRQ line
 * known. Supplies: PM8941 L18 ("vdd", 2.85 V) and LVS3 ("vio", 1.8 V).
 *
 * Interface (contract with the sensors HAL):
 *   input device name   "bandit-mag"
 *   events              EV_ABS ABS_X/ABS_Y/ABS_Z = raw signed counts in
 *                       chip axes (no rotation here), then EV_SYN
 *   sysfs (input dev)   enable      0/1, rw
 *                       poll_delay  ms, rw, clamped to [10, 1000]
 *
 * Scale: 15-bit output (CTRL4 RS = 1), counts in [-16384, 16383],
 * sensitivity 0.150 uT/LSB (data sheet typical value, same LSB weight in
 * 14- and 15-bit mode; 15-bit only extends the range to +/-2.4 mT):
 *   uT = counts * 0.15
 * (mainline ak8974 reports 2400 uT / 16384 = 0.1465 uT/LSB, derived from
 * the nominal range rather than from the sensitivity figure.)
 *
 * Mode: force state. Each poll starts one measurement (CTRL3 FRC), waits
 * for STAT DRDY and reads X/Y/Z. Compared with normal (continuous) state
 * this gives a fresh sample for any poll_delay instead of only for the
 * four fixed ODRs (0.5/10/20/100 Hz), never leaves a stale or overrun
 * sample in the output registers, keeps the chip idle between polls and
 * is the sequence used by the tested mainline driver. A temperature
 * measurement (CTRL3 TCS, wait STAT TRDY, read TEMP) is run on enable and
 * then every 10 s: the data sheet ties it to the chip's temperature
 * compensation; the 10 s interval is a choice of this driver, in line with
 * Alps' Android sample drivers, not a data sheet requirement. When
 * disabled the chip is in stand-by (CTRL1 PC = 0), the supplies stay on.
 */

#include <linux/bitops.h>
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

#define HSCD_DEV_NAME		"bandit-mag"

#define HSCD_STB		0x0c	/* self-test response */
#define HSCD_STB_IDLE		0x55
#define HSCD_STB_OK		0xaa
#define HSCD_INFO1		0x0d
#define HSCD_INFO2		0x0e
#define HSCD_WIA		0x0f
#define HSCD_WIA_VAL		0x49
#define HSCD_DATA_X		0x10	/* X_L, X_H, Y_L, Y_H, Z_L, Z_H */
#define HSCD_STAT		0x18
#define HSCD_CTRL1		0x1b
#define HSCD_CTRL2		0x1c
#define HSCD_CTRL3		0x1d
#define HSCD_CTRL4		0x1e
#define HSCD_TEMP		0x31	/* signed, 1 degC/LSB */

/* STAT */
#define STAT_DRDY		BIT(6)
#define STAT_TRDY		BIT(1)

/* CTRL1: PC | - | - | ODR[1:0] | - | FS | - */
#define CTRL1_PC		BIT(7)	/* 0 = stand-by, 1 = active */
#define CTRL1_FS		BIT(1)	/* 0 = normal state, 1 = force state */

/* CTRL2: AVG FCO AOR FF DEN DRP DTS DOS; DRDY pin unused, keep default */
#define CTRL2_DEFAULT		BIT(2)	/* DRP: DRDY active high */

/* CTRL3: one bit at a time; each self-clears when done */
#define CTRL3_SRST		BIT(7)
#define CTRL3_FRC		BIT(6)
#define CTRL3_STC		BIT(4)
#define CTRL3_TCS		BIT(1)

/* CTRL4: MMD[1:0] must be 10b, RS = 15-bit output, AS must be 0 */
#define CTRL4_MMD		BIT(7)
#define CTRL4_RS		BIT(4)
#define CTRL4_VALUE		(CTRL4_MMD | CTRL4_RS)

#define HSCD_RANGE		16384	/* 15-bit signed output */

#define HSCD_MIN_POLL_MS	10	/* 100 Hz, the fastest chip ODR */
#define HSCD_MAX_POLL_MS	1000
#define HSCD_DEF_POLL_MS	200
#define HSCD_TCS_INTERVAL_MS	10000
#define HSCD_POWERON_MS		50
#define HSCD_MEAS_US		3000	/* force-state measurement time */
#define HSCD_READY_TRIES	5	/* extra 1 ms checks after that */

struct hscd_data {
	struct i2c_client *client;
	struct input_dev *input;
	struct regulator *vdd;
	struct regulator *vio;
	struct delayed_work work;
	struct mutex lock;		/* protects enabled, poll_ms, chip mode */
	bool enabled;
	unsigned int poll_ms;
	unsigned long next_tcs;		/* jiffies: next temperature run */
	int temp_c;
};

static int hscd_write(struct hscd_data *d, u8 reg, u8 val)
{
	int ret = i2c_smbus_write_byte_data(d->client, reg, val);

	if (ret < 0)
		dev_err_ratelimited(&d->client->dev,
				    "write 0x%02x=0x%02x failed: %d\n",
				    reg, val, ret);
	return ret;
}

static int hscd_read(struct hscd_data *d, u8 reg)
{
	return i2c_smbus_read_byte_data(d->client, reg);
}

/* Wait until (reg & mask) == want; returns 0, -ETIMEDOUT or an I2C error. */
static int hscd_wait(struct hscd_data *d, u8 reg, u8 mask, u8 want,
		     unsigned int first_us)
{
	int i, ret;

	usleep_range(first_us, first_us + 1000);
	for (i = 0; i <= HSCD_READY_TRIES; i++) {
		ret = hscd_read(d, reg);
		if (ret < 0)
			return ret;
		if ((ret & mask) == want)
			return 0;
		usleep_range(1000, 1500);
	}
	return -ETIMEDOUT;
}

/* Stand-by or active force state. */
static int hscd_set_active(struct hscd_data *d, bool on)
{
	int ret = hscd_write(d, HSCD_CTRL1, (on ? CTRL1_PC : 0) | CTRL1_FS);

	if (!ret && on)
		usleep_range(1000, 2000);
	return ret;
}

/* Temperature measurement for the internal compensation (active mode). */
static void hscd_temp_comp(struct hscd_data *d)
{
	int ret;

	d->next_tcs = jiffies + msecs_to_jiffies(HSCD_TCS_INTERVAL_MS);
	if (hscd_write(d, HSCD_CTRL3, CTRL3_TCS) < 0)
		return;
	ret = hscd_wait(d, HSCD_STAT, STAT_TRDY, STAT_TRDY, 1000);
	if (ret) {
		dev_warn_ratelimited(&d->client->dev,
				     "temperature measurement: %d\n", ret);
		return;
	}
	ret = hscd_read(d, HSCD_TEMP);	/* reading clears TRDY */
	if (ret >= 0) {
		d->temp_c = (s8)ret;
		dev_dbg(&d->client->dev, "temperature %d C\n", d->temp_c);
	}
}

static int hscd_selftest(struct hscd_data *d)
{
	int before, during, after;

	before = hscd_read(d, HSCD_STB);
	if (before < 0 || hscd_write(d, HSCD_CTRL3, CTRL3_STC) < 0)
		return -EIO;
	usleep_range(1000, 2000);
	during = hscd_read(d, HSCD_STB);
	after = hscd_read(d, HSCD_STB);	/* reading STB restores 0x55 */

	if (before != HSCD_STB_IDLE || during != HSCD_STB_OK ||
	    after != HSCD_STB_IDLE) {
		dev_warn(&d->client->dev,
			 "self-test failed: STB 0x%02x -> 0x%02x -> 0x%02x\n",
			 before & 0xff, during & 0xff, after & 0xff);
		return -EIO;
	}
	return 0;
}

static int hscd_chip_init(struct hscd_data *d)
{
	struct device *dev = &d->client->dev;
	int ret;

	ret = hscd_write(d, HSCD_CTRL3, CTRL3_SRST);
	if (ret < 0)
		return ret;
	ret = hscd_wait(d, HSCD_CTRL3, CTRL3_SRST, 0, 1000);
	if (ret)
		dev_warn(dev, "soft reset did not complete: %d\n", ret);
	dev_info(dev, "after reset CTRL1 0x%02x CTRL4 0x%02x\n",
		 hscd_read(d, HSCD_CTRL1) & 0xff,
		 hscd_read(d, HSCD_CTRL4) & 0xff);

	/* Self-test runs in active mode; the result is only logged. */
	ret = hscd_set_active(d, true);
	if (ret < 0)
		return ret;
	if (!hscd_selftest(d))
		dev_info(dev, "self-test passed\n");

	ret = hscd_write(d, HSCD_CTRL2, CTRL2_DEFAULT);
	if (!ret)
		ret = hscd_write(d, HSCD_CTRL4, CTRL4_VALUE);
	if (!ret)
		ret = hscd_set_active(d, false);
	return ret;
}

/* One forced measurement; fills xyz with sign-extended 15-bit counts. */
static int hscd_measure(struct hscd_data *d, int *xyz)
{
	u8 buf[6];
	int ret, i;

	/* Discard a sample left over from a failed poll so DRDY is clear. */
	ret = hscd_read(d, HSCD_STAT);
	if (ret < 0)
		return ret;
	if (ret & STAT_DRDY)
		i2c_smbus_read_i2c_block_data(d->client, HSCD_DATA_X,
					      sizeof(buf), buf);

	ret = hscd_write(d, HSCD_CTRL3, CTRL3_FRC);
	if (ret < 0)
		return ret;
	ret = hscd_wait(d, HSCD_STAT, STAT_DRDY, STAT_DRDY, HSCD_MEAS_US);
	if (ret)
		return ret;

	ret = i2c_smbus_read_i2c_block_data(d->client, HSCD_DATA_X,
					    sizeof(buf), buf);
	if (ret != sizeof(buf))
		return ret < 0 ? ret : -EIO;

	for (i = 0; i < 3; i++)
		xyz[i] = sign_extend32(buf[2 * i + 1] << 8 | buf[2 * i], 14);
	return 0;
}

static void hscd_work(struct work_struct *work)
{
	struct hscd_data *d = container_of(to_delayed_work(work),
					   struct hscd_data, work);
	unsigned long start = jiffies, period, spent;
	int xyz[3], ret;

	if (time_after_eq(jiffies, d->next_tcs))
		hscd_temp_comp(d);

	ret = hscd_measure(d, xyz);
	if (ret) {
		dev_err_ratelimited(&d->client->dev,
				    "measurement failed: %d\n", ret);
	} else {
		input_report_abs(d->input, ABS_X, xyz[0]);
		input_report_abs(d->input, ABS_Y, xyz[1]);
		input_report_abs(d->input, ABS_Z, xyz[2]);
		input_sync(d->input);
	}

	if (!d->enabled)
		return;
	/* Keep the period close to poll_delay despite the measurement wait. */
	period = msecs_to_jiffies(d->poll_ms);
	spent = jiffies - start;
	queue_delayed_work(system_freezable_wq, &d->work,
			   period > spent ? period - spent : 1);
}

/* Caller holds d->lock. */
static int hscd_enable(struct hscd_data *d, bool on)
{
	int ret;

	if (on == d->enabled)
		return 0;

	if (on) {
		ret = hscd_set_active(d, true);
		if (ret < 0)
			return ret;
		d->next_tcs = jiffies;	/* compensate before the first sample */
		d->enabled = true;
		queue_delayed_work(system_freezable_wq, &d->work, 0);
	} else {
		d->enabled = false;
		cancel_delayed_work_sync(&d->work);
		hscd_set_active(d, false);
	}
	return 0;
}

static ssize_t hscd_enable_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct hscd_data *d = input_get_drvdata(to_input_dev(dev));

	return sprintf(buf, "%d\n", d->enabled ? 1 : 0);
}

static ssize_t hscd_enable_store(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	struct hscd_data *d = input_get_drvdata(to_input_dev(dev));
	unsigned long val;
	int ret;

	if (kstrtoul(buf, 0, &val) || val > 1)
		return -EINVAL;

	mutex_lock(&d->lock);
	ret = hscd_enable(d, val);
	mutex_unlock(&d->lock);

	return ret < 0 ? ret : count;
}

static ssize_t hscd_poll_delay_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct hscd_data *d = input_get_drvdata(to_input_dev(dev));

	return sprintf(buf, "%u\n", d->poll_ms);
}

static ssize_t hscd_poll_delay_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t count)
{
	struct hscd_data *d = input_get_drvdata(to_input_dev(dev));
	unsigned long val;

	if (kstrtoul(buf, 0, &val))
		return -EINVAL;
	val = clamp_t(unsigned long, val, HSCD_MIN_POLL_MS, HSCD_MAX_POLL_MS);

	/* Force state: nothing to reprogram, the next poll uses the value. */
	mutex_lock(&d->lock);
	d->poll_ms = val;
	mutex_unlock(&d->lock);

	return count;
}

static DEVICE_ATTR(enable, S_IRUGO | S_IWUSR | S_IWGRP,
		   hscd_enable_show, hscd_enable_store);
static DEVICE_ATTR(poll_delay, S_IRUGO | S_IWUSR | S_IWGRP,
		   hscd_poll_delay_show, hscd_poll_delay_store);

static struct attribute *hscd_attrs[] = {
	&dev_attr_enable.attr,
	&dev_attr_poll_delay.attr,
	NULL
};

static const struct attribute_group hscd_attr_group = {
	.attrs = hscd_attrs,
};

/* Optional supply: a missing one is reported and ignored. */
static struct regulator *hscd_supply_on(struct device *dev, const char *id)
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

static void hscd_supply_off(struct regulator *reg)
{
	if (reg) {
		regulator_disable(reg);
		regulator_put(reg);
	}
}

static int __devinit hscd_probe(struct i2c_client *client,
				const struct i2c_device_id *id)
{
	struct hscd_data *d;
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
	d->poll_ms = HSCD_DEF_POLL_MS;
	mutex_init(&d->lock);
	INIT_DELAYED_WORK(&d->work, hscd_work);

	d->vdd = hscd_supply_on(&client->dev, "vdd");
	if (IS_ERR(d->vdd))
		return PTR_ERR(d->vdd);
	d->vio = hscd_supply_on(&client->dev, "vio");
	if (IS_ERR(d->vio)) {
		ret = PTR_ERR(d->vio);
		d->vio = NULL;
		goto err_supply;
	}
	msleep(HSCD_POWERON_MS);

	ret = hscd_read(d, HSCD_WIA);
	if (ret < 0) {
		dev_err(&client->dev, "no response at 0x%02x: %d\n",
			client->addr, ret);
		goto err_supply;
	}
	if (ret != HSCD_WIA_VAL) {
		dev_err(&client->dev, "WIA 0x%02x, expected 0x%02x\n",
			ret, HSCD_WIA_VAL);
		ret = -ENODEV;
		goto err_supply;
	}
	dev_info(&client->dev, "HSCDTD008A INFO 0x%02x 0x%02x\n",
		 hscd_read(d, HSCD_INFO1) & 0xff,
		 hscd_read(d, HSCD_INFO2) & 0xff);

	ret = hscd_chip_init(d);
	if (ret < 0)
		goto err_supply;

	input = input_allocate_device();
	if (!input) {
		ret = -ENOMEM;
		goto err_supply;
	}
	input->name = HSCD_DEV_NAME;
	input->id.bustype = BUS_I2C;
	input->dev.parent = &client->dev;
	__set_bit(EV_ABS, input->evbit);
	input_set_abs_params(input, ABS_X, -HSCD_RANGE, HSCD_RANGE - 1, 0, 0);
	input_set_abs_params(input, ABS_Y, -HSCD_RANGE, HSCD_RANGE - 1, 0, 0);
	input_set_abs_params(input, ABS_Z, -HSCD_RANGE, HSCD_RANGE - 1, 0, 0);
	input_set_drvdata(input, d);
	d->input = input;

	ret = input_register_device(input);
	if (ret) {
		input_free_device(input);
		goto err_supply;
	}

	ret = sysfs_create_group(&input->dev.kobj, &hscd_attr_group);
	if (ret)
		goto err_input;

	i2c_set_clientdata(client, d);
	dev_info(&client->dev, "HSCDTD008A at 0x%02x, 15-bit, 0.15 uT/LSB\n",
		 client->addr);
	return 0;

err_input:
	input_unregister_device(input);
err_supply:
	hscd_supply_off(d->vio);
	hscd_supply_off(d->vdd);
	return ret;
}

static int __devexit hscd_remove(struct i2c_client *client)
{
	struct hscd_data *d = i2c_get_clientdata(client);

	sysfs_remove_group(&d->input->dev.kobj, &hscd_attr_group);
	mutex_lock(&d->lock);
	hscd_enable(d, false);
	mutex_unlock(&d->lock);
	input_unregister_device(d->input);
	hscd_supply_off(d->vio);
	hscd_supply_off(d->vdd);
	return 0;
}

#ifdef CONFIG_PM_SLEEP
/* Supplies stay on; only the chip goes to stand-by while suspended. */
static int hscd_suspend(struct device *dev)
{
	struct hscd_data *d = i2c_get_clientdata(to_i2c_client(dev));

	mutex_lock(&d->lock);
	if (d->enabled) {
		cancel_delayed_work_sync(&d->work);
		hscd_set_active(d, false);
	}
	mutex_unlock(&d->lock);
	return 0;
}

static int hscd_resume(struct device *dev)
{
	struct hscd_data *d = i2c_get_clientdata(to_i2c_client(dev));

	mutex_lock(&d->lock);
	if (d->enabled && !hscd_set_active(d, true)) {
		d->next_tcs = jiffies;
		queue_delayed_work(system_freezable_wq, &d->work, 0);
	}
	mutex_unlock(&d->lock);
	return 0;
}
#endif

static SIMPLE_DEV_PM_OPS(hscd_pm_ops, hscd_suspend, hscd_resume);

static const struct i2c_device_id hscd_id[] = {
	{ "hscdtd008a_bandit", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, hscd_id);

static const struct of_device_id hscd_of_match[] = {
	{ .compatible = "alps,hscdtd008a" },
	{ }
};
MODULE_DEVICE_TABLE(of, hscd_of_match);

static struct i2c_driver hscd_driver = {
	.driver = {
		.name		= "hscdtd008a_bandit",
		.owner		= THIS_MODULE,
		.of_match_table	= hscd_of_match,
		.pm		= &hscd_pm_ops,
	},
	.probe		= hscd_probe,
	.remove		= __devexit_p(hscd_remove),
	.id_table	= hscd_id,
};

module_i2c_driver(hscd_driver);

MODULE_AUTHOR("Lorenzo Giannoccaro");
MODULE_DESCRIPTION("Alps HSCDTD008A magnetometer driver for Lumia 1520 (bandit)");
MODULE_LICENSE("GPL v2");
