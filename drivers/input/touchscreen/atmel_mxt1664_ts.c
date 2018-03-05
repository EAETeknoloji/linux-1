/*
 * Driver for Atmel MXT1664 Multiple Touch Controller
 *
 * Copyright (C) 2017 EAE Technology
 * Author: Recep Birol Gul
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */


#include <linux/kernel.h>
#include <linux/dmi.h>
#include <linux/firmware.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/of.h>
#include <asm/unaligned.h>
#include <linux/gpio.h>
#include <linux/bitops.h>
#include <linux/of_gpio.h>

#define MAX_SUPPORT_POINTS		5

#define EVENT_VALID_OFFSET	7
#define EVENT_VALID_MASK	(0x1 << EVENT_VALID_OFFSET)
#define EVENT_ID_OFFSET		2
#define EVENT_ID_MASK		(0xf << EVENT_ID_OFFSET)
#define EVENT_IN_RANGE		(0x1 << 1)
#define EVENT_DOWN_UP		(0X1 << 0)

#define MAX_I2C_DATA_LEN	20

#define MXT1664_MAX_X	 	4095
#define MXT1664_MAX_Y	 	4095
#define MXT1664_MAX_TRIES 	 100

struct mxt1664_ts {
	struct i2c_client	*client;
	struct input_dev	*input_dev;
};

static void mxt1664_ts_read_one_set(struct input_dev *input_dev,
	struct i2c_client *client)
{
	int id, ret, x, y, z;
	int tries = 0;
	bool down;
	u8 state;
	u8 buf[MAX_I2C_DATA_LEN];

	do {
		ret = i2c_master_recv(client, buf, MAX_I2C_DATA_LEN);
	} while (ret == -EAGAIN && tries++ < MXT1664_MAX_TRIES);

	if (ret < 0) {
		dev_err(&client->dev, "I2C transfer error: %d\n", ret);
		return;
	}
		

	state = buf[1];
	x = (buf[6] << 8) | buf[5];
	y = (buf[10] << 8) | buf[9];
	z = (buf[14] << 8) | buf[13];
	id =buf[4];//(state & EVENT_ID_MASK) >> EVENT_ID_OFFSET;
	down = buf[3]==1;//state & EVENT_DOWN_UP;


	dev_info(&client->dev, "down: %d\n", down);
	//dev_info(&client->dev, "state: %d\n", state);
	//dev_info(&client->dev, "state & EVENT_DOWN_UP: %d\n", state&EVENT_DOWN_UP);
	dev_info(&client->dev, "id ->%d\n",id);
	//dev_info(&client->dev, "mode ->%d\n", buf[0]);
	dev_info(&client->dev, "x ->%d\n", x);
	dev_info(&client->dev, "y ->%d\n", y);
	dev_info(&client->dev, "z ->%d\n", z);

	/*
	int i;
	for( i = 0; i < MAX_I2C_DATA_LEN; i++)
		dev_info(&client->dev, "buf[%d] = %d\n",i, buf[i]);
	*/

	if (id > MAX_SUPPORT_POINTS) {
		dev_err(&client->dev, "point invalid id ->%d\n",id);
		return;
	}


	input_mt_slot(input_dev, id);
	input_mt_report_slot_state(input_dev, MT_TOOL_FINGER, down);

	if (down) {
		input_report_abs(input_dev, ABS_MT_POSITION_X, x);
		input_report_abs(input_dev, ABS_MT_POSITION_Y, y);
		input_report_abs(input_dev, ABS_MT_PRESSURE, z);
	}

	input_mt_report_pointer_emulation(input_dev, true);
	input_sync(input_dev);
}

static irqreturn_t mxt1664_ts_interrupt(int irq, void *dev_id)
{
	struct mxt1664_ts *ts = dev_id;
	struct input_dev *input_dev = ts->input_dev;
	struct i2c_client *client = ts->client;

	mxt1664_ts_read_one_set(input_dev, client);
	
	return IRQ_HANDLED;
}

/* wake up controller with a pulse on the reset gpio.  */
static int init_reset_pin(struct i2c_client *client)
{
	struct device_node *np = client->dev.of_node;
	int ret;
	int reset_pin;

	if (!np)
		return -ENODEV;
		

	reset_pin = of_get_named_gpio(np, "reset-gpios", 0);
	
	if (!gpio_is_valid(reset_pin))
		return -ENODEV;

	gpio_free(reset_pin);

	ret = gpio_request(reset_pin, "mxt1664_reset");
	if (ret < 0) {
		dev_err(&client->dev,
			"request gpio failed, cannot wake up controller: %d\n",
			ret);
		return ret;
	}	

	gpio_direction_output(reset_pin, 1);

	dev_info(&client->dev, "init_reset_pin() ok! reset gpio %d\n", reset_pin);

	return 0;
}

static int mxt1664_ts_enter_app_mode(struct i2c_client *client)
{
	char info[7];
	int loop;
	int ret;
	int original_addr = client->addr;

	for (loop = 1; loop <= 3; loop++) {
		/* read the 7 byte information header from reg 0 */
		info[0] = info[1] = 0x00;
		ret = i2c_master_send(client, info, 2);
		if (ret < 0) {
			dev_err(&client->dev, "Failed to set read address, ret=%d\n", ret);
		} else {
			ret = i2c_master_recv(client, info, 7);
			if (ret < 0) {
				dev_err(&client->dev, "Failed to read information, ret=%d\n", ret);
			} else {
				/* could read the information */
				break;
			}
		}

		/* Check CRC_FAIL bit from bootloader addr 1*/
		client->addr = 0x27;
		ret = i2c_master_recv(client, info, 1);
		if (ret < 0) {
			/* no response from primary address, test secondary */
			dev_err(&client->dev, "No response from bootloader addr 0x%02x (%d), ret=%d\n", client->addr, client->addr, ret);
			client->addr = 0x28;
			ret = i2c_master_recv(client, info, 1);
			if (ret < 0) {
				/* Neither primary nor secondary answers */
				dev_err(&client->dev, "No response from bootloader addr 0x%02x (%d), ret=%d\n", client->addr, client->addr, ret);
				dev_err(&client->dev, "No response from bootloaders, aborting\n");
				client->addr = original_addr;
				return ret;
			} else {
				dev_err(&client->dev, "Found bootloader on 0x%02x (%d), CRC=%s\n", client->addr, client->addr, ((info[0]&0xc0)==0x40)?"APP_CRC_FAIL":"");
			}
		} else {
			dev_err(&client->dev, "Found bootloader on 0x%02x (%d), CRC=%s\n", client->addr, client->addr, ((info[0]&0xc0)==0x40)?"APP_CRC_FAIL":"");
		}

		dev_err(&client->dev, "Going into APP mode..\n");
		info[0] = info[1] = 0x01;
		ret = i2c_master_send(client, info, 2);
		if (ret < 0) {
			dev_err(&client->dev, "Failed to send 0x0101 to 0x%02x (%d), ret=%d\n", client->addr, client->addr, ret);
		}
		msleep(3000);
	}

	if (loop == 4) {
		dev_err(&client->dev, "Failed 3 attempts, aborting\n");
		return -ENODEV;
	}

	dev_info(&client->dev, "Info: 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x, 0x%02x\n",
		info[0], info[1], info[2], info[3], info[4], info[5], info[6]);

	return 0;
}

static int mxt1664_ts_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct mxt1664_ts *ts;
	struct input_dev *input_dev;
	int error;

	dev_info(&client->dev, "mxt1664_ts_probe() I2C Address: 0x%02x\n", client->addr);
	dev_info(&client->dev, "client->irq %d\n", client->irq);

	// reset pinini ayarlar
	init_reset_pin(client);

	ts = devm_kzalloc(&client->dev, sizeof(struct mxt1664_ts), GFP_KERNEL);
	input_dev = devm_input_allocate_device(&client->dev);

	if (!ts || !input_dev) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		return -ENOMEM;
	}

	ts->client = client;
	ts->input_dev = input_dev;

	i2c_set_clientdata(client, ts);

	input_dev->name = "Atmel MXT1664 Touch Screen";
	input_dev->id.bustype = BUS_I2C;

	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(BTN_TOUCH, input_dev->keybit);

	input_set_abs_params(input_dev, ABS_X, 0, MXT1664_MAX_X, 0, 0);
	input_set_abs_params(input_dev, ABS_Y, 0, MXT1664_MAX_Y, 0, 0);
	input_set_abs_params(input_dev,
			     ABS_MT_POSITION_X, 0, MXT1664_MAX_X, 0, 0);
	input_set_abs_params(input_dev,
			     ABS_MT_POSITION_Y, 0, MXT1664_MAX_Y, 0, 0);
	input_mt_init_slots(input_dev, MAX_SUPPORT_POINTS, 0);

	input_set_drvdata(input_dev, ts);

	error = input_register_device(ts->input_dev);
	if (error)
		return error;

	error = mxt1664_ts_enter_app_mode(client);
	if (error) {
		dev_err(&client->dev, "Failed to put controller in app mode\n");
		return error;
	}

	error = devm_request_threaded_irq(&client->dev, client->irq, NULL,
					  mxt1664_ts_interrupt,
					  IRQF_TRIGGER_LOW | IRQF_ONESHOT,
					  "mxt1664_ts", ts);

	if (error < 0) {
		dev_err(&client->dev, "Failed to register interrupt\n");
		return error;
	}

	dev_info(&client->dev, "mxt1664_ts_probe() ok!");

	return 0;
}


static int __maybe_unused mxt1664_ts_suspend(struct device *dev)
{
	/*static const u8 suspend_cmd[MAX_I2C_DATA_LEN] = {
		0x3, 0x6, 0xa, 0x3, 0x36, 0x3f, 0x2, 0, 0, 0
	};
	struct i2c_client *client = to_i2c_client(dev);
	int ret;

	ret = i2c_master_send(client, suspend_cmd, MAX_I2C_DATA_LEN);
	return ret > 0 ? 0 : ret;
	*/
	return 0;
}

static int __maybe_unused mxt1664_ts_resume(struct device *dev)
{
	/*struct i2c_client *client = to_i2c_client(dev);

	return mxt1664_wake_up_device(client);*/
	return 0;
}

static SIMPLE_DEV_PM_OPS(mxt1664_ts_pm_ops, mxt1664_ts_suspend, mxt1664_ts_resume);

static const struct i2c_device_id mxt1664_ts_id[] = {
	{ "mxt1664_ts", 0 },
	{ }
};

MODULE_DEVICE_TABLE(i2c, mxt1664_ts_id);

static const struct of_device_id mxt1664_ts_of_match[] = {
	{ .compatible = "atmel,mxt1664_ts" },
	{ }
};

MODULE_DEVICE_TABLE(of, mxt1664_ts_of_match);

static struct i2c_driver mxt1664_ts_driver = {
	.driver = {
		.name	= "mxt1664_ts",
		.owner	= THIS_MODULE,
		.pm	= &mxt1664_ts_pm_ops,
		.of_match_table	= of_match_ptr(mxt1664_ts_of_match),
	},
	.id_table	= mxt1664_ts_id,
	.probe		= mxt1664_ts_probe,
};

module_i2c_driver(mxt1664_ts_driver);

MODULE_AUTHOR("EAE Technology");
MODULE_DESCRIPTION("Touchscreen driver for Atmel MXT1664 touch controller");
MODULE_LICENSE("GPL");
