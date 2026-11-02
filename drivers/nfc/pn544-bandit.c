/*
 * Character device driver for the NXP PN544 on the Nokia Lumia 1520.
 *
 * Exposes /dev/pn544 with the interface of the Android PN544 kernel driver
 * by Sylvain Fonteneau (NXP), which the legacy userspace stack
 * (external/libnfc-nxp) expects: raw HCI/LLC frames through read()/write()
 * and PN544_SET_PWR to drive VEN and the firmware download line.
 *
 * Differences from that driver: GPIOs come from the device tree, the
 * reference clock buffer (BBCLK2, pin controlled by the chip CLK_REQ) is
 * voted while the chip is powered, and the chip starts powered off.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>
#include <linux/wait.h>

#define PN544_MAGIC		0xE9
#define PN544_SET_PWR		_IOW(PN544_MAGIC, 0x01, unsigned int)

#define MAX_BUFFER_SIZE		512

struct pn544_bandit {
	struct i2c_client *client;
	struct miscdevice misc;
	wait_queue_head_t read_wq;
	struct mutex read_mutex;
	spinlock_t irq_lock;
	bool irq_enabled;
	int irq;
	int ven_gpio;
	int firm_gpio;
	int irq_gpio;
	struct clk *ref_clk;
	bool clk_on;
	bool powered;
};

static void pn544_disable_irq(struct pn544_bandit *pn)
{
	unsigned long flags;

	spin_lock_irqsave(&pn->irq_lock, flags);
	if (pn->irq_enabled) {
		disable_irq_nosync(pn->irq);
		pn->irq_enabled = false;
	}
	spin_unlock_irqrestore(&pn->irq_lock, flags);
}

static void pn544_enable_irq(struct pn544_bandit *pn)
{
	unsigned long flags;

	spin_lock_irqsave(&pn->irq_lock, flags);
	if (!pn->irq_enabled) {
		pn->irq_enabled = true;
		enable_irq(pn->irq);
	}
	spin_unlock_irqrestore(&pn->irq_lock, flags);
}

static irqreturn_t pn544_irq_handler(int irq, void *dev_id)
{
	struct pn544_bandit *pn = dev_id;

	if (!gpio_get_value(pn->irq_gpio))
		return IRQ_HANDLED;

	pn544_disable_irq(pn);
	wake_up(&pn->read_wq);

	return IRQ_HANDLED;
}

static void pn544_clock(struct pn544_bandit *pn, bool on)
{
	if (!pn->ref_clk || pn->clk_on == on)
		return;

	if (on) {
		if (clk_prepare_enable(pn->ref_clk)) {
			dev_err(&pn->client->dev, "cannot enable ref_clk\n");
			return;
		}
	} else {
		clk_disable_unprepare(pn->ref_clk);
	}
	pn->clk_on = on;
}

static void pn544_set_firm(struct pn544_bandit *pn, int value)
{
	if (gpio_is_valid(pn->firm_gpio))
		gpio_set_value(pn->firm_gpio, value);
}

static int pn544_set_power(struct pn544_bandit *pn, unsigned long mode)
{
	switch (mode) {
	case 2:
		/* Power on in firmware download mode (needs a VEN reset). */
		dev_info(&pn->client->dev, "power on, download mode\n");
		pn544_clock(pn, true);
		gpio_set_value(pn->ven_gpio, 1);
		pn544_set_firm(pn, 1);
		msleep(10);
		gpio_set_value(pn->ven_gpio, 0);
		msleep(50);
		gpio_set_value(pn->ven_gpio, 1);
		msleep(10);
		pn->powered = true;
		break;
	case 1:
		dev_info(&pn->client->dev, "power on\n");
		pn544_clock(pn, true);
		pn544_set_firm(pn, 0);
		gpio_set_value(pn->ven_gpio, 1);
		msleep(10);
		pn->powered = true;
		break;
	case 0:
		dev_info(&pn->client->dev, "power off\n");
		pn544_set_firm(pn, 0);
		gpio_set_value(pn->ven_gpio, 0);
		msleep(50);
		pn544_clock(pn, false);
		pn->powered = false;
		break;
	default:
		dev_err(&pn->client->dev, "bad power mode %lu\n", mode);
		return -EINVAL;
	}

	return 0;
}

static ssize_t pn544_read(struct file *filp, char __user *buf,
			  size_t count, loff_t *offset)
{
	struct pn544_bandit *pn = filp->private_data;
	char tmp[MAX_BUFFER_SIZE];
	int ret;

	if (count > MAX_BUFFER_SIZE)
		count = MAX_BUFFER_SIZE;

	mutex_lock(&pn->read_mutex);

	if (!gpio_get_value(pn->irq_gpio)) {
		if (filp->f_flags & O_NONBLOCK) {
			ret = -EAGAIN;
			goto out_unlock;
		}

		pn544_enable_irq(pn);
		ret = wait_event_interruptible(pn->read_wq,
				gpio_get_value(pn->irq_gpio));
		pn544_disable_irq(pn);
		if (ret)
			goto out_unlock;
	}

	ret = i2c_master_recv(pn->client, tmp, count);
	mutex_unlock(&pn->read_mutex);

	if (ret < 0) {
		dev_err(&pn->client->dev, "i2c read failed: %d\n", ret);
		return ret;
	}
	if (ret > count)
		return -EIO;
	if (copy_to_user(buf, tmp, ret))
		return -EFAULT;

	return ret;

out_unlock:
	mutex_unlock(&pn->read_mutex);
	return ret;
}

static ssize_t pn544_write(struct file *filp, const char __user *buf,
			   size_t count, loff_t *offset)
{
	struct pn544_bandit *pn = filp->private_data;
	char tmp[MAX_BUFFER_SIZE];
	int ret;

	if (count > MAX_BUFFER_SIZE)
		count = MAX_BUFFER_SIZE;
	if (copy_from_user(tmp, buf, count))
		return -EFAULT;

	ret = i2c_master_send(pn->client, tmp, count);
	if (ret != count) {
		dev_err(&pn->client->dev, "i2c write failed: %d\n", ret);
		return -EIO;
	}

	return ret;
}

static int pn544_open(struct inode *inode, struct file *filp)
{
	filp->private_data = container_of(filp->private_data,
					  struct pn544_bandit, misc);
	return 0;
}

static long pn544_ioctl(struct file *filp, unsigned int cmd,
			unsigned long arg)
{
	struct pn544_bandit *pn = filp->private_data;

	if (cmd != PN544_SET_PWR) {
		dev_err(&pn->client->dev, "bad ioctl 0x%x\n", cmd);
		return -EINVAL;
	}

	return pn544_set_power(pn, arg);
}

static const struct file_operations pn544_fops = {
	.owner		= THIS_MODULE,
	.llseek		= no_llseek,
	.read		= pn544_read,
	.write		= pn544_write,
	.open		= pn544_open,
	.unlocked_ioctl	= pn544_ioctl,
};

static int pn544_request_gpio(struct device *dev, const char *prop,
			      const char *label, bool required)
{
	int gpio = of_get_named_gpio(dev->of_node, prop, 0);
	int ret;

	if (!gpio_is_valid(gpio)) {
		if (required)
			dev_err(dev, "missing %s\n", prop);
		return required ? -EINVAL : -ENOENT;
	}

	ret = gpio_request(gpio, label);
	if (ret) {
		dev_err(dev, "cannot request %s (%d): %d\n", label, gpio, ret);
		return ret;
	}

	return gpio;
}

static int pn544_probe(struct i2c_client *client,
		       const struct i2c_device_id *id)
{
	struct device *dev = &client->dev;
	struct pn544_bandit *pn;
	int ret;

	if (!dev->of_node)
		return -ENODEV;
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -ENODEV;

	pn = kzalloc(sizeof(*pn), GFP_KERNEL);
	if (!pn)
		return -ENOMEM;

	pn->client = client;
	pn->ven_gpio = -EINVAL;
	pn->firm_gpio = -EINVAL;

	pn->irq_gpio = pn544_request_gpio(dev, "nxp,irq-gpio", "nfc_irq", true);
	if (pn->irq_gpio < 0) {
		ret = pn->irq_gpio;
		goto err_free;
	}
	pn->ven_gpio = pn544_request_gpio(dev, "nxp,ven-gpio", "nfc_ven", true);
	if (pn->ven_gpio < 0) {
		ret = pn->ven_gpio;
		goto err_irq_gpio;
	}
	pn->firm_gpio = pn544_request_gpio(dev, "nxp,firm-gpio", "nfc_firm",
					   false);
	if (pn->firm_gpio < 0 && pn->firm_gpio != -ENOENT) {
		ret = pn->firm_gpio;
		goto err_ven_gpio;
	}

	gpio_direction_input(pn->irq_gpio);
	gpio_direction_output(pn->ven_gpio, 0);
	if (gpio_is_valid(pn->firm_gpio))
		gpio_direction_output(pn->firm_gpio, 0);

	pn->ref_clk = clk_get(dev, "ref_clk");
	if (IS_ERR(pn->ref_clk)) {
		dev_warn(dev, "no ref_clk: %ld\n", PTR_ERR(pn->ref_clk));
		pn->ref_clk = NULL;
	}

	init_waitqueue_head(&pn->read_wq);
	mutex_init(&pn->read_mutex);
	spin_lock_init(&pn->irq_lock);

	pn->irq = gpio_to_irq(pn->irq_gpio);
	pn->irq_enabled = true;
	ret = request_irq(pn->irq, pn544_irq_handler, IRQF_TRIGGER_HIGH,
			  "pn544", pn);
	if (ret) {
		dev_err(dev, "request_irq %d failed: %d\n", pn->irq, ret);
		goto err_clk;
	}
	pn544_disable_irq(pn);
	enable_irq_wake(pn->irq);

	pn->misc.minor = MISC_DYNAMIC_MINOR;
	pn->misc.name = "pn544";
	pn->misc.fops = &pn544_fops;
	ret = misc_register(&pn->misc);
	if (ret) {
		dev_err(dev, "misc_register failed: %d\n", ret);
		goto err_irq;
	}

	i2c_set_clientdata(client, pn);
	dev_info(dev, "PN544 at 0x%02x, irq gpio %d, ven %d, firm %d%s\n",
		 client->addr, pn->irq_gpio, pn->ven_gpio, pn->firm_gpio,
		 pn->ref_clk ? ", ref_clk" : "");

	return 0;

err_irq:
	disable_irq_wake(pn->irq);
	free_irq(pn->irq, pn);
err_clk:
	mutex_destroy(&pn->read_mutex);
	if (pn->ref_clk)
		clk_put(pn->ref_clk);
	if (gpio_is_valid(pn->firm_gpio))
		gpio_free(pn->firm_gpio);
err_ven_gpio:
	gpio_free(pn->ven_gpio);
err_irq_gpio:
	gpio_free(pn->irq_gpio);
err_free:
	kfree(pn);
	return ret;
}

static int pn544_remove(struct i2c_client *client)
{
	struct pn544_bandit *pn = i2c_get_clientdata(client);

	misc_deregister(&pn->misc);
	pn544_set_power(pn, 0);
	disable_irq_wake(pn->irq);
	free_irq(pn->irq, pn);
	mutex_destroy(&pn->read_mutex);
	if (pn->ref_clk)
		clk_put(pn->ref_clk);
	if (gpio_is_valid(pn->firm_gpio))
		gpio_free(pn->firm_gpio);
	gpio_free(pn->ven_gpio);
	gpio_free(pn->irq_gpio);
	kfree(pn);

	return 0;
}

static void pn544_shutdown(struct i2c_client *client)
{
	struct pn544_bandit *pn = i2c_get_clientdata(client);

	if (pn && pn->powered)
		pn544_set_power(pn, 0);
}

static const struct of_device_id pn544_bandit_of_match[] = {
	{ .compatible = "nokia,bandit-pn544", },
	{ },
};
MODULE_DEVICE_TABLE(of, pn544_bandit_of_match);

static const struct i2c_device_id pn544_bandit_id[] = {
	{ "pn544-bandit", 0 },
	{ },
};
MODULE_DEVICE_TABLE(i2c, pn544_bandit_id);

static struct i2c_driver pn544_bandit_driver = {
	.id_table	= pn544_bandit_id,
	.probe		= pn544_probe,
	.remove		= pn544_remove,
	.shutdown	= pn544_shutdown,
	.driver		= {
		.owner		= THIS_MODULE,
		.name		= "pn544-bandit",
		.of_match_table	= pn544_bandit_of_match,
	},
};

module_i2c_driver(pn544_bandit_driver);

MODULE_DESCRIPTION("NXP PN544 character device for the Nokia Lumia 1520");
MODULE_LICENSE("GPL");
