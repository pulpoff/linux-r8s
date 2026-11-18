// SPDX-License-Identifier: GPL-2.0-only
/*
 * Zinitix ZT7650 touchscreen controller, as on the Galaxy S20 FE (r8s)
 *
 * The ZT7650 speaks another dialect than the BT4xx/BT5xx parts zinitix.c
 * drives: other vendor commands start its firmware, and it reports every
 * contact as a 16-byte event packet instead of a status word and a point
 * table. Its firmware lives in the chip and comes up with the panel's
 * resolution set, so nothing but the mode needs to be written.
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/input/touchscreen.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/regulator/consumer.h>

#define ZT_SWRESET_CMD			0x0000
#define ZT_CLEAR_INT_STATUS_CMD		0x0003
#define ZT_TOUCH_MODE			0x0010
#define ZT_CHIP_REVISION		0x0011
#define ZT_FIRMWARE_VERSION		0x0012
#define ZT_DATA_VERSION			0x0013
#define ZT_SUPPORTED_FINGER_NUM		0x0015
#define ZT_X_RESOLUTION			0x00c0
#define ZT_Y_RESOLUTION			0x00c1
#define ZT_OPTIONAL_SETTING		0x0116
#define ZT_MINOR_FW_VERSION		0x0121
#define ZT_CHECKSUM_RESULT		0x012c
#define ZT_POINT_STATUS_REG		0x0200	/* the first event packet */
#define ZT_POINT_STATUS_REG1		0x0201	/* the packets after it */
#define ZT_COVER_CONTROL		0x023e

/* vendor commands: they start the firmware from the chip's flash */
#define ZT_VCMD_ENABLE			0x10f0
#define ZT_VCMD_NVM_PROG_START		0x11f0
#define ZT_VCMD_NVM_INIT		0x12f0
#define ZT_VCMD_INTN_CLR		0x14f0
#define ZT_VCMD_ID			0x17f0

#define ZT_CHECKSUM_OK			0x55aa
#define ZT_COVER_OPEN			0x0200
#define ZT_TOUCH_POINT_MODE		0

#define ZT_MAX_FINGERS			10

#define ZT_CHIP_ON_DELAY_MS		100
#define ZT_FIRMWARE_ON_DELAY_MS		150

enum zt_event_id {
	ZT_EVENT_COORDINATE = 0,
	ZT_EVENT_STATUS,
	ZT_EVENT_GESTURE,
	ZT_EVENT_CUSTOM,
};

enum zt_touch_status {
	ZT_FINGER_NONE = 0,
	ZT_FINGER_PRESS,
	ZT_FINGER_MOVE,
	ZT_FINGER_RELEASE,
};

#define ZT_TYPE_PALM			5

struct zt_event {
	u8 id;		/* event id (1:0), touch id (5:2), touch status (7:6) */
	u8 x_hi;
	u8 y_hi;
	u8 xy_lo;	/* y bits 3:0 (3:0), x bits 3:0 (7:4) */
	u8 major;
	u8 minor;
	u8 z_type;	/* z (5:0), touch type bits 3:2 (7:6) */
	u8 left_type;	/* packets left after this one (3:0), touch type bits 1:0 (7:6) */
	u8 rest[8];
} __packed;
static_assert(sizeof(struct zt_event) == 16);

#define ZT_EID(ev)		((ev)->id & 0x3)
#define ZT_TID(ev)		(((ev)->id >> 2) & 0xf)
#define ZT_STATUS(ev)		((ev)->id >> 6)
#define ZT_LEFT(ev)		((ev)->left_type & 0xf)
#define ZT_TYPE(ev)		((((ev)->z_type >> 6) << 2) | ((ev)->left_type >> 6))

struct zt7650 {
	struct i2c_client *client;
	struct input_dev *input;
	struct touchscreen_properties prop;
	struct regulator *vdd;
	struct zt_event ev[ZT_MAX_FINGERS];
};

/* The address and the data must be separate transfers, with a gap between them. */
static int zt_read(struct zt7650 *zt, u16 reg, void *buf, size_t len)
{
	__le16 reg_le = cpu_to_le16(reg);
	int ret;

	ret = i2c_master_send(zt->client, (u8 *)&reg_le, sizeof(reg_le));
	if (ret != sizeof(reg_le))
		return ret < 0 ? ret : -EIO;

	usleep_range(50, 60);

	ret = i2c_master_recv(zt->client, buf, len);
	if (ret != len)
		return ret < 0 ? ret : -EIO;

	return 0;
}

static int zt_read_u16(struct zt7650 *zt, u16 reg, u16 *val)
{
	__le16 v;
	int error;

	error = zt_read(zt, reg, &v, sizeof(v));
	if (!error)
		*val = le16_to_cpu(v);

	return error;
}

static int zt_write_u16(struct zt7650 *zt, u16 reg, u16 val)
{
	__le16 packet[2] = { cpu_to_le16(reg), cpu_to_le16(val) };
	int ret;

	ret = i2c_master_send(zt->client, (u8 *)packet, sizeof(packet));
	if (ret != sizeof(packet))
		return ret < 0 ? ret : -EIO;

	return 0;
}

static int zt_write_cmd(struct zt7650 *zt, u16 reg)
{
	__le16 reg_le = cpu_to_le16(reg);
	int ret;

	ret = i2c_master_send(zt->client, (u8 *)&reg_le, sizeof(reg_le));
	if (ret != sizeof(reg_le))
		return ret < 0 ? ret : -EIO;

	return 0;
}

/* Power the chip and start its firmware; the checksum reads 0x55aa once it runs. */
static int zt7650_power_on_once(struct zt7650 *zt)
{
	struct device *dev = &zt->client->dev;
	u16 val;
	int error;

	error = regulator_enable(zt->vdd);
	if (error) {
		dev_err(dev, "failed to enable the supply: %d\n", error);
		return error;
	}

	msleep(ZT_CHIP_ON_DELAY_MS);

	error = zt_write_u16(zt, ZT_VCMD_ENABLE, 1);
	if (error)
		goto fail;
	usleep_range(10, 20);

	error = zt_read_u16(zt, ZT_VCMD_ID, &val);
	if (error)
		goto fail;
	dev_dbg(dev, "chip code %#06x\n", val);
	usleep_range(10, 20);

	error = zt_write_cmd(zt, ZT_VCMD_INTN_CLR);
	if (error)
		goto fail;
	usleep_range(10, 20);

	error = zt_write_u16(zt, ZT_VCMD_NVM_INIT, 1);
	if (error)
		goto fail;
	usleep_range(2000, 3000);

	error = zt_write_u16(zt, ZT_VCMD_NVM_PROG_START, 1);
	if (error)
		goto fail;
	msleep(ZT_FIRMWARE_ON_DELAY_MS);

	error = zt_read_u16(zt, ZT_CHECKSUM_RESULT, &val);
	if (error)
		goto fail;
	if (val != ZT_CHECKSUM_OK) {
		dev_err(dev, "firmware did not start (checksum %#06x)\n", val);
		error = -EIO;
		goto fail;
	}

	return 0;

fail:
	regulator_disable(zt->vdd);
	return error;
}

static int zt7650_power_on(struct zt7650 *zt)
{
	struct device *dev = &zt->client->dev;
	int error;

	error = zt7650_power_on_once(zt);
	if (!error)
		return 0;

	/*
	 * The first transfer after a cold power-up can time out (after a kexec from a kernel that had powered the
	 * chip down); the vendor driver then powers it off and on again. The supply's off-on delay keeps it off
	 * for long enough.
	 */
	dev_warn(dev, "power-on sequence failed (%d), once more\n", error);
	error = zt7650_power_on_once(zt);
	if (error)
		dev_err(dev, "power-on sequence failed: %d\n", error);
	return error;
}

static int zt7650_init(struct zt7650 *zt)
{
	int error;
	int i;

	error = zt_write_cmd(zt, ZT_SWRESET_CMD);
	if (error)
		return error;

	error = zt_write_u16(zt, ZT_TOUCH_MODE, ZT_TOUCH_POINT_MODE);
	if (error)
		return error;

	error = zt_write_u16(zt, ZT_COVER_CONTROL, ZT_COVER_OPEN);
	if (error)
		return error;

	error = zt_write_u16(zt, ZT_OPTIONAL_SETTING, 0);
	if (error)
		return error;

	/* drop what queued up before */
	for (i = 0; i < 10; i++) {
		zt_write_cmd(zt, ZT_CLEAR_INT_STATUS_CMD);
		udelay(10);
	}

	return 0;
}

static void zt7650_report(struct zt7650 *zt, const struct zt_event *ev)
{
	unsigned int status = ZT_STATUS(ev);
	unsigned int slot = ZT_TID(ev);
	unsigned int x, y;
	int tool;

	if (status == ZT_FINGER_NONE || slot >= ZT_MAX_FINGERS)
		return;

	x = (ev->x_hi << 4) | (ev->xy_lo >> 4);
	y = (ev->y_hi << 4) | (ev->xy_lo & 0xf);
	tool = ZT_TYPE(ev) == ZT_TYPE_PALM ? MT_TOOL_PALM : MT_TOOL_FINGER;

	input_mt_slot(zt->input, slot);
	if (input_mt_report_slot_state(zt->input, tool, status != ZT_FINGER_RELEASE)) {
		touchscreen_report_pos(zt->input, &zt->prop, x, y, true);
		input_report_abs(zt->input, ABS_MT_TOUCH_MAJOR, ev->major);
		input_report_abs(zt->input, ABS_MT_TOUCH_MINOR, ev->minor);
	}
}

static irqreturn_t zt7650_irq(int irq, void *data)
{
	struct zt7650 *zt = data;
	struct device *dev = &zt->client->dev;
	unsigned int n, i;
	int error;

	error = zt_read(zt, ZT_POINT_STATUS_REG, &zt->ev[0], sizeof(zt->ev[0]));
	if (error) {
		dev_err_ratelimited(dev, "failed to read an event: %d\n", error);
		goto out;
	}

	/* status, gesture and custom events are about modes this driver leaves off */
	if (ZT_EID(&zt->ev[0]) != ZT_EVENT_COORDINATE)
		goto out;

	n = 1 + ZT_LEFT(&zt->ev[0]);
	if (n > ZT_MAX_FINGERS) {
		dev_warn_ratelimited(dev, "bad event count %u\n", n);
		goto out;
	}

	if (n > 1) {
		error = zt_read(zt, ZT_POINT_STATUS_REG1, &zt->ev[1], (n - 1) * sizeof(zt->ev[0]));
		if (error) {
			dev_err_ratelimited(dev, "failed to read %u events: %d\n", n - 1, error);
			goto out;
		}
	}

	for (i = 0; i < n; i++)
		zt7650_report(zt, &zt->ev[i]);

	input_mt_sync_frame(zt->input);
	input_sync(zt->input);

out:
	zt_write_cmd(zt, ZT_CLEAR_INT_STATUS_CMD);
	return IRQ_HANDLED;
}

static int zt7650_start(struct zt7650 *zt)
{
	int error;

	error = zt7650_power_on(zt);
	if (error)
		return error;

	error = zt7650_init(zt);
	if (error) {
		dev_err(&zt->client->dev, "failed to set the chip up: %d\n", error);
		regulator_disable(zt->vdd);
		return error;
	}

	enable_irq(zt->client->irq);
	return 0;
}

static void zt7650_stop(struct zt7650 *zt)
{
	disable_irq(zt->client->irq);
	regulator_disable(zt->vdd);
}

static int zt7650_open(struct input_dev *input)
{
	return zt7650_start(input_get_drvdata(input));
}

static void zt7650_close(struct input_dev *input)
{
	zt7650_stop(input_get_drvdata(input));
}

static int zt7650_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct input_dev *input;
	struct zt7650 *zt;
	u16 rev, fw, minor, data, xres, yres, fingers;
	int error;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C))
		return -ENXIO;

	zt = devm_kzalloc(dev, sizeof(*zt), GFP_KERNEL);
	if (!zt)
		return -ENOMEM;

	zt->client = client;
	i2c_set_clientdata(client, zt);

	zt->vdd = devm_regulator_get(dev, "vdd");
	if (IS_ERR(zt->vdd))
		return dev_err_probe(dev, PTR_ERR(zt->vdd), "failed to get the supply\n");

	input = devm_input_allocate_device(dev);
	if (!input)
		return -ENOMEM;

	zt->input = input;
	input->name = "Zinitix ZT7650 Touchscreen";
	input->phys = "i2c/zt7650";
	input->id.bustype = BUS_I2C;
	input->open = zt7650_open;
	input->close = zt7650_close;

	input_set_capability(input, EV_ABS, ABS_MT_POSITION_X);
	input_set_capability(input, EV_ABS, ABS_MT_POSITION_Y);
	input_set_abs_params(input, ABS_MT_TOUCH_MAJOR, 0, 255, 0, 0);
	input_set_abs_params(input, ABS_MT_TOUCH_MINOR, 0, 255, 0, 0);
	input_set_abs_params(input, ABS_MT_TOOL_TYPE, 0, MT_TOOL_MAX, 0, 0);

	touchscreen_parse_properties(input, true, &zt->prop);
	if (!zt->prop.max_x || !zt->prop.max_y)
		return dev_err_probe(dev, -EINVAL, "touchscreen-size-x and -y are needed\n");

	error = input_mt_init_slots(input, ZT_MAX_FINGERS, INPUT_MT_DIRECT);
	if (error)
		return error;

	input_set_drvdata(input, zt);

	error = devm_request_threaded_irq(dev, client->irq, NULL, zt7650_irq,
					  IRQF_ONESHOT | IRQF_NO_AUTOEN, client->name, zt);
	if (error)
		return dev_err_probe(dev, error, "failed to request the interrupt\n");

	/* is the chip there, and does its firmware run? */
	error = zt7650_power_on(zt);
	if (error)
		return dev_err_probe(dev, error, "no working ZT7650\n");

	if (zt_read_u16(zt, ZT_CHIP_REVISION, &rev) ||
	    zt_read_u16(zt, ZT_FIRMWARE_VERSION, &fw) ||
	    zt_read_u16(zt, ZT_MINOR_FW_VERSION, &minor) ||
	    zt_read_u16(zt, ZT_DATA_VERSION, &data) ||
	    zt_read_u16(zt, ZT_X_RESOLUTION, &xres) ||
	    zt_read_u16(zt, ZT_Y_RESOLUTION, &yres) ||
	    zt_read_u16(zt, ZT_SUPPORTED_FINGER_NUM, &fingers))
		dev_warn(dev, "failed to read the chip's versions\n");
	else
		dev_info(dev, "ZT7650 revision %#x, firmware %#x.%#x, data %#x, %ux%u, %u fingers\n",
			 rev, fw, minor, data, xres, yres, fingers);

	regulator_disable(zt->vdd);

	error = input_register_device(input);
	if (error)
		return dev_err_probe(dev, error, "failed to register the input device\n");

	return 0;
}

static int zt7650_suspend(struct device *dev)
{
	struct zt7650 *zt = i2c_get_clientdata(to_i2c_client(dev));

	guard(mutex)(&zt->input->mutex);
	if (input_device_enabled(zt->input))
		zt7650_stop(zt);

	return 0;
}

static int zt7650_resume(struct device *dev)
{
	struct zt7650 *zt = i2c_get_clientdata(to_i2c_client(dev));

	guard(mutex)(&zt->input->mutex);
	if (input_device_enabled(zt->input))
		return zt7650_start(zt);

	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(zt7650_pm_ops, zt7650_suspend, zt7650_resume);

static const struct of_device_id zt7650_of_match[] = {
	{ .compatible = "zinitix,zt7650" },
	{ }
};
MODULE_DEVICE_TABLE(of, zt7650_of_match);

static struct i2c_driver zt7650_driver = {
	.probe = zt7650_probe,
	.driver = {
		.name = "zinitix-zt7650",
		.pm = pm_sleep_ptr(&zt7650_pm_ops),
		.of_match_table = zt7650_of_match,
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
	},
};
module_i2c_driver(zt7650_driver);

MODULE_DESCRIPTION("Zinitix ZT7650 touchscreen driver");
MODULE_LICENSE("GPL");
