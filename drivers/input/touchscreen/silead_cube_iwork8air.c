// SPDX-License-Identifier: GPL-2.0-or-later
/* -------------------------------------------------------------------------
 * Copyright (C) 2014-2015, Intel Corporation
 *
 * Derived from:
 *  gslX68X.c
 *  Copyright (C) 2010-2015, Shanghai Sileadinc Co.Ltd
 *
 * -------------------------------------------------------------------------
 */

#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/acpi.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/pm.h>
#include <linux/irq.h>
#include <linux/gpio.h>

#include <asm/unaligned.h>

#define SILEAD_TS_NAME		"silead_ts"

#define SILEAD_REG_RESET	0xE0
#define SILEAD_REG_DATA		0x80
#define SILEAD_REG_TOUCH_NR	0x80
#define SILEAD_REG_POWER	0xBC
#define SILEAD_REG_CLOCK	0xE4
#define SILEAD_REG_STATUS	0xB0
#define SILEAD_REG_ID		0xFC
#define SILEAD_REG_MEM_CHECK	0xB0

#define SILEAD_STATUS_OK	0x5A5A5A5A
#define SILEAD_TS_DATA_LEN	44
#define SILEAD_CLOCK		0x04

#define SILEAD_CMD_RESET	0x88
#define SILEAD_CMD_START	0x00

#define SILEAD_POINT_DATA_LEN	0x04
#define SILEAD_POINT_Y_OFF      0x00
#define SILEAD_POINT_Y_MSB_OFF	0x01
#define SILEAD_POINT_X_OFF	0x02
#define SILEAD_POINT_X_MSB_OFF	0x03
#define SILEAD_EXTRA_DATA_MASK	0xF0

#define SILEAD_CMD_SLEEP_MIN	10000
#define SILEAD_CMD_SLEEP_MAX	20000
#define SILEAD_POWER_SLEEP	20
#define SILEAD_STARTUP_SLEEP	30

#define SILEAD_MAX_FINGERS	10

/*
 * Another HACK, this makes us generate events in the same way as the silead
 * driver as shipped on X86 devs with Android 5.1, dunno why this is necessary.
 */
#define ANDROID5_STYLE_TOUCH_EVENTS 1

/* HACK copied over from mainline to make mainline driver work */
struct touchscreen_properties {
	unsigned int min_x;
	unsigned int max_x;
	unsigned int min_y;
	unsigned int max_y;
	bool invert_x;
	bool invert_y;
	bool swap_x_y;
};

static void
touchscreen_apply_prop_to_x_y(const struct touchscreen_properties *prop,
			      unsigned int *x, unsigned int *y)
{
	if (prop->invert_x)
		*x = prop->max_x - *x;

	if (prop->invert_y)
		*y = prop->max_y - *y;

	if (prop->swap_x_y)
		swap(*x, *y);
}

static void touchscreen_set_mt_pos(struct input_mt_pos *pos,
			    const struct touchscreen_properties *prop,
			    unsigned int x, unsigned int y)
{
	touchscreen_apply_prop_to_x_y(prop, &x, &y);
	pos->x = x;
	pos->y = y;
}
/* End HACK copied over from mainline to make mainline driver work */

struct silead_ts_data {
	struct i2c_client *client;
	struct input_dev *input;
	char fw_name[64];
	struct touchscreen_properties prop;
	u32 max_fingers;
	u32 chip_id;
	struct input_mt_pos pos[SILEAD_MAX_FINGERS];
	int slots[SILEAD_MAX_FINGERS];
	int id[SILEAD_MAX_FINGERS];
};

struct silead_fw_data {
	u32 offset;
	u32 val;
};

static int silead_ts_request_input_dev(struct silead_ts_data *data)
{
	struct device *dev = &data->client->dev;
	int error;

	data->input = devm_input_allocate_device(dev);
	if (!data->input) {
		dev_err(dev,
			"Failed to allocate input device\n");
		return -ENOMEM;
	}

	if (data->prop.invert_x) {
		data->prop.max_x -= data->prop.min_x;
		data->prop.min_x = 0;
	}

	if (data->prop.invert_y) {
		data->prop.max_y -= data->prop.min_y;
		data->prop.min_y = 0;
	}

	input_set_abs_params(data->input, ABS_MT_POSITION_X,
			     data->prop.min_x, data->prop.max_x, 0, 0);
	input_set_abs_params(data->input, ABS_MT_POSITION_Y,
			     data->prop.min_y, data->prop.max_y, 0, 0);

	if (data->prop.swap_x_y)
		swap(data->input->absinfo[ABS_MT_POSITION_X],
		     data->input->absinfo[ABS_MT_POSITION_Y]);

#if ANDROID5_STYLE_TOUCH_EVENTS
	__set_bit(EV_ABS, data->input->evbit);
	__set_bit(EV_KEY, data->input->evbit);
	__set_bit(EV_REP, data->input->evbit);
	__set_bit(INPUT_PROP_DIRECT, data->input->propbit);
	input_mt_init_slots(data->input, data->max_fingers, 0);
	input_set_abs_params(data->input, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(data->input, ABS_MT_WIDTH_MAJOR, 0, 200, 0, 0);
#else
	input_mt_init_slots(data->input, data->max_fingers,
			    INPUT_MT_DIRECT | INPUT_MT_DROP_UNUSED |
			    INPUT_MT_TRACK);
#endif

	input_set_capability(data->input, EV_KEY, KEY_LEFTMETA);

	data->input->name = SILEAD_TS_NAME;
	data->input->phys = "input/ts";
	data->input->id.bustype = BUS_I2C;

	error = input_register_device(data->input);
	if (error) {
		dev_err(dev, "Failed to register input device: %d\n", error);
		return error;
	}

	return 0;
}

static void silead_ts_read_data(struct i2c_client *client)
{
	struct silead_ts_data *data = i2c_get_clientdata(client);
	struct input_dev *input = data->input;
	struct device *dev = &client->dev;
	u8 *bufp, buf[SILEAD_TS_DATA_LEN];
	int touch_nr, softbutton, error, i;
	bool softbutton_pressed = false;

	error = i2c_smbus_read_i2c_block_data(client, SILEAD_REG_DATA,
					      SILEAD_TS_DATA_LEN, buf);
	if (error < 0) {
		dev_err(dev, "Data read error %d\n", error);
		return;
	}

	if (buf[0] > data->max_fingers) {
		dev_warn(dev, "More touches reported then supported %d > %d\n",
			 buf[0], data->max_fingers);
		buf[0] = data->max_fingers;
	}

	touch_nr = 0;
	bufp = buf + SILEAD_POINT_DATA_LEN;
	for (i = 0; i < buf[0]; i++, bufp += SILEAD_POINT_DATA_LEN) {
		softbutton = (bufp[SILEAD_POINT_Y_MSB_OFF] &
			      SILEAD_EXTRA_DATA_MASK) >> 4;

		if (softbutton) {
			/*
			 * For now only respond to softbutton == 0x01, some
			 * tablets *without* a capacative button send 0x04
			 * when crossing the edges of the screen.
			 */
			if (softbutton == 0x01)
				softbutton_pressed = true;

			continue;
		}

		/*
		 * Bits 4-7 are the touch id, note not all models have
		 * hardware touch ids so atm we don't use these.
		 */
		data->id[touch_nr] = (bufp[SILEAD_POINT_X_MSB_OFF] &
				      SILEAD_EXTRA_DATA_MASK) >> 4;
		touchscreen_set_mt_pos(&data->pos[touch_nr], &data->prop,
			get_unaligned_le16(&bufp[SILEAD_POINT_X_OFF]) & 0xfff,
			get_unaligned_le16(&bufp[SILEAD_POINT_Y_OFF]) & 0xfff);
		touch_nr++;
	}

#if !ANDROID5_STYLE_TOUCH_EVENTS
	input_mt_assign_slots(input, data->slots, data->pos, touch_nr);
#endif

	for (i = 0; i < touch_nr; i++) {
#if ANDROID5_STYLE_TOUCH_EVENTS
		input_mt_slot(input, i);
		input_report_abs(input, ABS_MT_TRACKING_ID, i);
		input_report_abs(input, ABS_MT_TOUCH_MAJOR, 10);
		input_report_abs(input, ABS_MT_POSITION_X, data->pos[i].x);
		input_report_abs(input, ABS_MT_POSITION_Y, data->pos[i].y);
		input_report_abs(input, ABS_MT_WIDTH_MAJOR, 1);
#else
		input_mt_slot(input, data->slots[i]);
		input_mt_report_slot_state(input, MT_TOOL_FINGER, true);
		input_report_abs(input, ABS_MT_POSITION_X, data->pos[i].x);
		input_report_abs(input, ABS_MT_POSITION_Y, data->pos[i].y);
#endif

		dev_dbg(dev, "x=%d y=%d hw_id=%d sw_id=%d\n", data->pos[i].x,
			data->pos[i].y, data->id[i], data->slots[i]);
	}

#if ANDROID5_STYLE_TOUCH_EVENTS
	for (i = touch_nr; i < data->max_fingers; i++) {
		input_mt_slot(input, i);
		input_report_abs(input, ABS_MT_TRACKING_ID, -1);
	}
#else
	input_mt_sync_frame(input);
#endif
	input_report_key(input, KEY_LEFTMETA, softbutton_pressed);
	input_sync(input);
}

static int silead_ts_init(struct i2c_client *client)
{
	struct silead_ts_data *data = i2c_get_clientdata(client);
	int error;

	error = i2c_smbus_write_byte_data(client, SILEAD_REG_RESET,
					  SILEAD_CMD_RESET);
	if (error)
		goto i2c_write_err;
	usleep_range(SILEAD_CMD_SLEEP_MIN, SILEAD_CMD_SLEEP_MAX);

	error = i2c_smbus_write_byte_data(client, SILEAD_REG_TOUCH_NR,
					data->max_fingers);
	if (error)
		goto i2c_write_err;
	usleep_range(SILEAD_CMD_SLEEP_MIN, SILEAD_CMD_SLEEP_MAX);

	error = i2c_smbus_write_byte_data(client, SILEAD_REG_CLOCK,
					  SILEAD_CLOCK);
	if (error)
		goto i2c_write_err;
	usleep_range(SILEAD_CMD_SLEEP_MIN, SILEAD_CMD_SLEEP_MAX);

	error = i2c_smbus_write_byte_data(client, SILEAD_REG_RESET,
					  SILEAD_CMD_START);
	if (error)
		goto i2c_write_err;
	usleep_range(SILEAD_CMD_SLEEP_MIN, SILEAD_CMD_SLEEP_MAX);

	return 0;

i2c_write_err:
	dev_err(&client->dev, "Registers clear error %d\n", error);
	return error;
}

static int silead_ts_reset(struct i2c_client *client)
{
	int error;

	error = i2c_smbus_write_byte_data(client, SILEAD_REG_RESET,
					  SILEAD_CMD_RESET);
	if (error)
		goto i2c_write_err;
	usleep_range(SILEAD_CMD_SLEEP_MIN, SILEAD_CMD_SLEEP_MAX);

	error = i2c_smbus_write_byte_data(client, SILEAD_REG_CLOCK,
					  SILEAD_CLOCK);
	if (error)
		goto i2c_write_err;
	usleep_range(SILEAD_CMD_SLEEP_MIN, SILEAD_CMD_SLEEP_MAX);

	error = i2c_smbus_write_byte_data(client, SILEAD_REG_POWER,
					  SILEAD_CMD_START);
	if (error)
		goto i2c_write_err;
	usleep_range(SILEAD_CMD_SLEEP_MIN, SILEAD_CMD_SLEEP_MAX);

	return 0;

i2c_write_err:
	dev_err(&client->dev, "Chip reset error %d\n", error);
	return error;
}

static int silead_ts_startup(struct i2c_client *client)
{
	int error;

	error = i2c_smbus_write_byte_data(client, SILEAD_REG_RESET, 0x00);
	if (error) {
		dev_err(&client->dev, "Startup error %d\n", error);
		return error;
	}

	msleep(SILEAD_STARTUP_SLEEP);

	return 0;
}

static int silead_ts_load_fw(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct silead_ts_data *data = i2c_get_clientdata(client);
	unsigned int fw_size, i;
	const struct firmware *fw;
	struct silead_fw_data *fw_data;
	int error;

	dev_dbg(dev, "Firmware file name: %s", data->fw_name);

	error = request_firmware(&fw, data->fw_name, dev);
	if (error) {
		dev_err(dev, "Firmware request error %d\n", error);
		return error;
	}

	fw_size = fw->size / sizeof(*fw_data);
	fw_data = (struct silead_fw_data *)fw->data;

	for (i = 0; i < fw_size; i++) {
		error = i2c_smbus_write_i2c_block_data(client,
						       fw_data[i].offset,
						       4,
						       (u8 *)&fw_data[i].val);
		if (error) {
			dev_err(dev, "Firmware load error %d\n", error);
			break;
		}
	}

	release_firmware(fw);
	return error ?: 0;
}

static u32 silead_ts_get_status(struct i2c_client *client)
{
	int error;
	__le32 status;

	error = i2c_smbus_read_i2c_block_data(client, SILEAD_REG_STATUS,
					      sizeof(status), (u8 *)&status);
	if (error < 0) {
		dev_err(&client->dev, "Status read error %d\n", error);
		return error;
	}

	return le32_to_cpu(status);
}

static int silead_ts_get_id(struct i2c_client *client)
{
	struct silead_ts_data *data = i2c_get_clientdata(client);
	__le32 chip_id;
	int error;

	error = i2c_smbus_read_i2c_block_data(client, SILEAD_REG_ID,
					      sizeof(chip_id), (u8 *)&chip_id);
	if (error < 0) {
		dev_err(&client->dev, "Chip ID read error %d\n", error);
		return error;
	}

	data->chip_id = le32_to_cpu(chip_id);
	dev_info(&client->dev, "Silead chip ID: 0x%8X", data->chip_id);

	return 0;
}

static int silead_ts_setup(struct i2c_client *client)
{
	int error;
	u32 status;

	error = silead_ts_get_id(client);
	if (error)
		return error;

	error = silead_ts_init(client);
	if (error)
		return error;

	error = silead_ts_reset(client);
	if (error)
		return error;

	error = silead_ts_load_fw(client);
	if (error)
		return error;

	error = silead_ts_startup(client);
	if (error)
		return error;

	status = silead_ts_get_status(client);
	if (status != SILEAD_STATUS_OK) {
		dev_err(&client->dev,
			"Initialization error, status: 0x%X\n", status);
		return -ENODEV;
	}

	return 0;
}

static irqreturn_t silead_ts_threaded_irq_handler(int irq, void *id)
{
	struct silead_ts_data *data = id;
	struct i2c_client *client = data->client;

	silead_ts_read_data(client);

	return IRQ_HANDLED;
}

static int silead_ts_probe(struct i2c_client *client,
			   const struct i2c_device_id *id)
{
	struct silead_ts_data *data;
	struct device *dev = &client->dev;
	int error;

	if (!i2c_check_functionality(client->adapter,
				     I2C_FUNC_I2C |
				     I2C_FUNC_SMBUS_READ_I2C_BLOCK |
				     I2C_FUNC_SMBUS_WRITE_I2C_BLOCK)) {
		dev_err(dev, "I2C functionality check failed\n");
		return -ENXIO;
	}

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	i2c_set_clientdata(client, data);
	data->client = client;

	/* HACK hardcoded values for Cube iWork8 Air i1TF32GB1629xxxx */
#define IRQ_PORT			360  
#define RST_PORT			366

	data->prop.min_x = 1;
	data->prop.max_x = 1663;
	data->prop.min_y = 3;
	data->prop.max_y = 895;
	data->prop.invert_x = false;
	data->prop.invert_y = false;
	data->prop.swap_x_y = true;
	data->max_fingers = 10;

	snprintf(data->fw_name, sizeof(data->fw_name),
		 "silead/gsl3670-cube-iwork8-air.fw");

	/* End HACK */

	/* HACK because android boot does not enumerate the client + irq through ACPI */
	error = gpio_request(IRQ_PORT, "GSL_IRQ");
	if (error) {
		dev_err(dev, "gpio_request(IRQ_PORT) failed, error: %d\n", error);
		return error;
	}
        gpio_direction_input(IRQ_PORT); 
	client->irq = gpio_to_irq(IRQ_PORT);
	/* End HACK because android boot does not enumerate the client + irq through ACPI */

	error = silead_ts_setup(client);
	if (error)
		return error;

	error = silead_ts_request_input_dev(data);
	if (error)
		return error;

	error = devm_request_threaded_irq(dev, client->irq,
					  NULL, silead_ts_threaded_irq_handler,
					  IRQF_ONESHOT|IRQF_TRIGGER_RISING, client->name, data);
	if (error) {
		if (error != -EPROBE_DEFER)
			dev_err(dev, "IRQ request failed %d\n", error);
		return error;
	}

	return 0;
}

static int __maybe_unused silead_ts_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);

	disable_irq(client->irq);
	return 0;
}

static int __maybe_unused silead_ts_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	bool second_try = false;
	int error, status;

 retry:
	error = silead_ts_reset(client);
	if (error)
		return error;

	if (second_try) {
		error = silead_ts_load_fw(client);
		if (error)
			return error;
	}

	error = silead_ts_startup(client);
	if (error)
		return error;

	status = silead_ts_get_status(client);
	if (status != SILEAD_STATUS_OK) {
		if (!second_try) {
			second_try = true;
			dev_dbg(dev, "Reloading firmware after unsuccessful resume\n");
			goto retry;
		}
		dev_err(dev, "Resume error, status: 0x%02x\n", status);
		return -ENODEV;
	}

	enable_irq(client->irq);

	return 0;
}

static SIMPLE_DEV_PM_OPS(silead_ts_pm, silead_ts_suspend, silead_ts_resume);

static const struct i2c_device_id silead_ts_id[] = {
	{ "gsl1680", 0 },
	{ "gsl1688", 0 },
	{ "gsl3670", 0 },
	{ "gsl3675", 0 },
	{ "gsl3692", 0 },
	{ "mssl1680", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, silead_ts_id);

#ifdef CONFIG_ACPI
static const struct acpi_device_id silead_ts_acpi_match[] = {
	{ "GSL1680", 0 },
	{ "GSL1688", 0 },
	{ "GSL3670", 0 },
	{ "GSL3675", 0 },
	{ "GSL3692", 0 },
	{ "MSSL1680", 0 },
	{ "MSSL0001", 0 },
	{ "MSSL0002", 0 },
	{ "MSSL0017", 0 },
	{ }
};
MODULE_DEVICE_TABLE(acpi, silead_ts_acpi_match);
#endif

#ifdef CONFIG_OF
static const struct of_device_id silead_ts_of_match[] = {
	{ .compatible = "silead,gsl1680" },
	{ .compatible = "silead,gsl1688" },
	{ .compatible = "silead,gsl3670" },
	{ .compatible = "silead,gsl3675" },
	{ .compatible = "silead,gsl3692" },
	{ },
};
MODULE_DEVICE_TABLE(of, silead_ts_of_match);
#endif

static struct i2c_driver silead_ts_driver = {
	.probe = silead_ts_probe,
	.id_table = silead_ts_id,
	.driver = {
		.name = SILEAD_TS_NAME,
		.acpi_match_table = ACPI_PTR(silead_ts_acpi_match),
		.of_match_table = of_match_ptr(silead_ts_of_match),
		.pm = &silead_ts_pm,
	},
};

/* HACK hardcoded adapter idx + client addr for Chuwi Hi8 Pro pq32g221709xxxx */
static int silead_adapter_index = 5;
static struct i2c_board_info silead_i2c_info = {
	.addr = 0x40,
	.type = "gsl1680",
};

int silead_init(void)
{
	struct i2c_client *client;
	struct i2c_adapter *adapter;

	/* HACK because android boot does not enumerate the client + irq through ACPI */
	adapter = i2c_get_adapter(silead_adapter_index);
	if (!adapter) {
		pr_err("Could not find adapter for silead touchscreen\n");
		return -ENODEV;
	}

	client = i2c_new_device(adapter, &silead_i2c_info);
	if (!adapter) {
		pr_err("Could not create i2c-client for silead touchscreen\n");
		return -ENODEV;
	}
	/* End HACK because android boot does not enumerate the client + irq through ACPI */

	return i2c_add_driver(&silead_ts_driver);
}

module_init(silead_init);

MODULE_AUTHOR("Robert Dolca <robert.dolca@intel.com>");
MODULE_DESCRIPTION("Silead I2C touchscreen driver");
MODULE_LICENSE("GPL");
