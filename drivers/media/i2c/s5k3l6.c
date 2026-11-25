// SPDX-License-Identifier: GPL-2.0
/*
 * Samsung S5K3L6 (12 MP, fixed focus; the rear ultrawide of the Galaxy S20
 * FE).
 *
 * A V4L2 sub-device with one source pad, RAW10 on 4 lanes at 1196 Mbps. The
 * sensor is powered only while it streams: stream on powers it, loads the
 * stock global and mode tables, applies the controls and starts it; stream
 * off stops it and powers it off. One mode for now, 2000x1500 binned from
 * the whole array at 30 fps.
 *
 * The same structure as the S10e's s5k3p9 driver; the 3L6 needs no
 * firmware patch and no paged (0x6f12) writes.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>
#include <linux/unaligned.h>

#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-subdev.h>

struct s5k3l6_reg {
	u16 addr, val;
};

#include "s5k3l6-tables.h"

#define S5K3L6_REG_MODEL_ID	0x0000
#define S5K3L6_REG_REVISION	0x0002
#define S5K3L6_REG_STREAM	0x0100
#define S5K3L6_REG_CIT		0x0202
#define S5K3L6_REG_AGAIN	0x0204
#define S5K3L6_REG_FLL		0x0340
#define S5K3L6_REG_TEST_PATTERN	0x0600
#define S5K3L6_REG_PLL_POWER	0x3c1e
#define S5K3L6_MODEL_ID		0x30c6
#define S5K3L6_MCLK_HZ		26000000

#define S5K3L6_AGAIN_MIN	0x20	/* code = gain x 32: x1 to x16 */
#define S5K3L6_AGAIN_MAX	0x200
#define S5K3L6_FLL_MAX		0xffff
#define S5K3L6_CIT_MIN		2	/* coarse integration time, lines */
#define S5K3L6_CIT_MARGIN	4

#define S5K3L6_ARRAY_WIDTH	4000
#define S5K3L6_ARRAY_HEIGHT	3000

/* 1196 Mbps a lane: half of that is the link frequency */
static const s64 s5k3l6_link_freqs[] = { 598000000 };

/* stock's video timing pixel clock, in which the line length counts */
#define S5K3L6_PIXEL_RATE	480000000ULL

struct s5k3l6_mode {
	u32 width, height;
	u32 llp, fll;			/* line length (pixel clocks), frame length (lines) */
	const struct s5k3l6_reg *regs;
	unsigned int n;
};

static const struct s5k3l6_mode s5k3l6_modes[] = {
	{ 2000, 1500, 0x1320, 0x0cc1, s5k3l6_mode_2000x1500, ARRAY_SIZE(s5k3l6_mode_2000x1500) },
};

/* stock power-on order: analog (an LDO switched by a GPIO), IO, then digital */
static const char * const s5k3l6_supplies[] = { "avdd", "dovdd", "dvdd" };

static const char * const s5k3l6_test_patterns[] = {
	"Disabled",
	"Solid Colour",
	"Colour Bars",
	"Fade To Grey Colour Bars",
	"PN9",
};

struct s5k3l6 {
	struct i2c_client *client;
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct clk *mclk;
	struct gpio_desc *reset;
	struct regulator_bulk_data supplies[ARRAY_SIZE(s5k3l6_supplies)];

	struct v4l2_ctrl_handler ctrls;
	struct v4l2_ctrl *pixel_rate, *link_freq, *hblank, *vblank, *exposure;

	const struct s5k3l6_mode *mode;
	bool streaming;
};

static inline struct s5k3l6 *to_s5k3l6(struct v4l2_subdev *sd)
{
	return container_of(sd, struct s5k3l6, sd);
}

static int s5k3l6_read16(struct s5k3l6 *s, u16 reg, u16 *val)
{
	u8 addr[2], buf[2];
	struct i2c_msg msgs[] = {
		{ .addr = s->client->addr, .len = 2, .buf = addr },
		{ .addr = s->client->addr, .flags = I2C_M_RD, .len = 2, .buf = buf },
	};
	int ret;

	put_unaligned_be16(reg, addr);
	ret = i2c_transfer(s->client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret != ARRAY_SIZE(msgs))
		return ret < 0 ? ret : -EIO;
	*val = get_unaligned_be16(buf);
	return 0;
}

static int s5k3l6_write16(struct s5k3l6 *s, u16 reg, u16 val)
{
	u8 buf[4];
	int ret;

	put_unaligned_be16(reg, buf);
	put_unaligned_be16(val, buf + 2);
	ret = i2c_master_send(s->client, buf, sizeof(buf));
	if (ret < 0)
		return ret;
	return ret == sizeof(buf) ? 0 : -EIO;
}

static int s5k3l6_write_table(struct s5k3l6 *s, const struct s5k3l6_reg *t, unsigned int n)
{
	unsigned int i;
	int ret;

	for (i = 0; i < n; i++) {
		ret = s5k3l6_write16(s, t[i].addr, t[i].val);
		if (ret) {
			dev_err(&s->client->dev, "write %#06x: %d\n", t[i].addr, ret);
			return ret;
		}
	}
	return 0;
}

/*
 * Stock: reset low; avdd, dovdd, dvdd and 500 us; reset high; the MCLK pin
 * function and MCLK, then 3 ms. The MCLK pin keeps its function (pinctrl
 * default state) while the clock is off.
 */
static int s5k3l6_power_on(struct s5k3l6 *s)
{
	int i, ret;

	gpiod_set_value_cansleep(s->reset, 1);
	for (i = 0; i < ARRAY_SIZE(s->supplies); i++) {
		ret = regulator_enable(s->supplies[i].consumer);
		if (ret)
			goto err_reg;
	}
	usleep_range(1000, 1100);
	gpiod_set_value_cansleep(s->reset, 0);
	ret = clk_prepare_enable(s->mclk);
	if (ret)
		goto err_reset;
	usleep_range(3000, 3400);
	return 0;

err_reset:
	gpiod_set_value_cansleep(s->reset, 1);
err_reg:
	while (--i >= 0)
		regulator_disable(s->supplies[i].consumer);
	return ret;
}

/* stock: 1 ms, MCLK off, reset low, then the supplies */
static void s5k3l6_power_off(struct s5k3l6 *s)
{
	int i;

	usleep_range(1000, 1100);
	clk_disable_unprepare(s->mclk);
	gpiod_set_value_cansleep(s->reset, 1);
	for (i = ARRAY_SIZE(s->supplies) - 1; i >= 0; i--)
		regulator_disable(s->supplies[i].consumer);
}

/* the exposure range follows the frame length */
static void s5k3l6_update_exposure_range(struct s5k3l6 *s)
{
	u32 max = s->mode->height + s->vblank->val - S5K3L6_CIT_MARGIN;

	__v4l2_ctrl_modify_range(s->exposure, S5K3L6_CIT_MIN, max, 1,
				 min_t(u32, s->exposure->val, max));
}

static int s5k3l6_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct s5k3l6 *s = container_of(ctrl->handler, struct s5k3l6, ctrls);
	int ret = 0;

	if (ctrl->id == V4L2_CID_VBLANK)
		s5k3l6_update_exposure_range(s);

	if (!s->streaming)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		ret = s5k3l6_write16(s, S5K3L6_REG_CIT, ctrl->val);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		ret = s5k3l6_write16(s, S5K3L6_REG_AGAIN, ctrl->val);
		break;
	case V4L2_CID_VBLANK:
		ret = s5k3l6_write16(s, S5K3L6_REG_FLL, s->mode->height + ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = s5k3l6_write16(s, S5K3L6_REG_TEST_PATTERN, ctrl->val);
		break;
	default:
		break;
	}
	return ret;
}

static const struct v4l2_ctrl_ops s5k3l6_ctrl_ops = {
	.s_ctrl = s5k3l6_set_ctrl,
};

/* blanking for a new mode */
static void s5k3l6_update_mode_ctrls(struct s5k3l6 *s)
{
	const struct s5k3l6_mode *m = s->mode;
	u32 vblank = m->fll - m->height;

	__v4l2_ctrl_modify_range(s->hblank, m->llp - m->width, m->llp - m->width, 1,
				 m->llp - m->width);
	__v4l2_ctrl_modify_range(s->vblank, vblank, S5K3L6_FLL_MAX - m->height, 1, vblank);
	__v4l2_ctrl_s_ctrl(s->vblank, vblank);
	s5k3l6_update_exposure_range(s);
}

static int s5k3l6_init_ctrls(struct s5k3l6 *s)
{
	struct v4l2_ctrl_handler *h = &s->ctrls;
	const struct s5k3l6_mode *m = s->mode;
	struct v4l2_fwnode_device_properties props;
	u32 cit_max = m->fll - S5K3L6_CIT_MARGIN;
	int ret;

	v4l2_ctrl_handler_init(h, 10);

	s->pixel_rate = v4l2_ctrl_new_std(h, &s5k3l6_ctrl_ops, V4L2_CID_PIXEL_RATE,
					  S5K3L6_PIXEL_RATE, S5K3L6_PIXEL_RATE, 1,
					  S5K3L6_PIXEL_RATE);
	s->link_freq = v4l2_ctrl_new_int_menu(h, &s5k3l6_ctrl_ops, V4L2_CID_LINK_FREQ,
					      ARRAY_SIZE(s5k3l6_link_freqs) - 1, 0,
					      s5k3l6_link_freqs);
	if (s->link_freq)
		s->link_freq->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	s->hblank = v4l2_ctrl_new_std(h, &s5k3l6_ctrl_ops, V4L2_CID_HBLANK,
				      m->llp - m->width, m->llp - m->width, 1, m->llp - m->width);
	if (s->hblank)
		s->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;
	s->vblank = v4l2_ctrl_new_std(h, &s5k3l6_ctrl_ops, V4L2_CID_VBLANK,
				      m->fll - m->height, S5K3L6_FLL_MAX - m->height, 1,
				      m->fll - m->height);
	s->exposure = v4l2_ctrl_new_std(h, &s5k3l6_ctrl_ops, V4L2_CID_EXPOSURE,
					S5K3L6_CIT_MIN, cit_max, 1, cit_max);
	v4l2_ctrl_new_std(h, &s5k3l6_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			  S5K3L6_AGAIN_MIN, S5K3L6_AGAIN_MAX, 1, S5K3L6_AGAIN_MIN);
	v4l2_ctrl_new_std_menu_items(h, &s5k3l6_ctrl_ops, V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(s5k3l6_test_patterns) - 1, 0, 0,
				     s5k3l6_test_patterns);

	ret = v4l2_fwnode_device_parse(&s->client->dev, &props);
	if (!ret)
		v4l2_ctrl_new_fwnode_properties(h, &s5k3l6_ctrl_ops, &props);

	if (h->error) {
		ret = h->error;
		v4l2_ctrl_handler_free(h);
		return ret;
	}
	s->sd.ctrl_handler = h;
	return 0;
}

static void s5k3l6_fill_fmt(const struct s5k3l6_mode *m, struct v4l2_mbus_framefmt *fmt)
{
	fmt->width = m->width;
	fmt->height = m->height;
	fmt->code = MEDIA_BUS_FMT_SGRBG10_1X10;
	fmt->field = V4L2_FIELD_NONE;
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->ycbcr_enc = V4L2_YCBCR_ENC_601;
	fmt->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->xfer_func = V4L2_XFER_FUNC_NONE;
}

static int s5k3l6_enum_mbus_code(struct v4l2_subdev *sd, struct v4l2_subdev_state *state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index)
		return -EINVAL;
	code->code = MEDIA_BUS_FMT_SGRBG10_1X10;
	return 0;
}

static int s5k3l6_enum_frame_size(struct v4l2_subdev *sd, struct v4l2_subdev_state *state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->code != MEDIA_BUS_FMT_SGRBG10_1X10 || fse->index >= ARRAY_SIZE(s5k3l6_modes))
		return -EINVAL;
	fse->min_width = fse->max_width = s5k3l6_modes[fse->index].width;
	fse->min_height = fse->max_height = s5k3l6_modes[fse->index].height;
	return 0;
}

static int s5k3l6_set_fmt(struct v4l2_subdev *sd, struct v4l2_subdev_state *state,
			  struct v4l2_subdev_format *fmt)
{
	struct s5k3l6 *s = to_s5k3l6(sd);
	const struct s5k3l6_mode *m;

	m = v4l2_find_nearest_size(s5k3l6_modes, ARRAY_SIZE(s5k3l6_modes), width, height,
				   fmt->format.width, fmt->format.height);
	s5k3l6_fill_fmt(m, &fmt->format);
	*v4l2_subdev_state_get_format(state, 0) = fmt->format;

	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE) {
		if (s->streaming)
			return -EBUSY;
		s->mode = m;
		s5k3l6_update_mode_ctrls(s);
	}
	return 0;
}

static int s5k3l6_get_selection(struct v4l2_subdev *sd, struct v4l2_subdev_state *state,
				struct v4l2_subdev_selection *sel)
{
	switch (sel->target) {
	case V4L2_SEL_TGT_CROP:		/* every mode reads the whole array */
	case V4L2_SEL_TGT_NATIVE_SIZE:
	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = S5K3L6_ARRAY_WIDTH;
		sel->r.height = S5K3L6_ARRAY_HEIGHT;
		return 0;
	}
	return -EINVAL;
}

static int s5k3l6_enable_streams(struct v4l2_subdev *sd, struct v4l2_subdev_state *state,
				 u32 pad, u64 streams_mask)
{
	struct s5k3l6 *s = to_s5k3l6(sd);
	const struct s5k3l6_mode *m = s->mode;
	int ret;

	ret = s5k3l6_power_on(s);
	if (ret)
		return ret;
	ret = s5k3l6_write_table(s, s5k3l6_global, ARRAY_SIZE(s5k3l6_global));
	usleep_range(3000, 3100);
	if (!ret)
		ret = s5k3l6_write_table(s, m->regs, m->n);
	if (ret)
		goto err;

	s->streaming = true;
	ret = __v4l2_ctrl_handler_setup(&s->ctrls);
	if (!ret)
		ret = s5k3l6_write_table(s, s5k3l6_dualsync_slave_off,
					 ARRAY_SIZE(s5k3l6_dualsync_slave_off));
	/* stock: the stream on goes between a PLL power control on and off */
	if (!ret)
		ret = s5k3l6_write16(s, S5K3L6_REG_PLL_POWER, 0x0100);
	if (!ret)
		ret = s5k3l6_write16(s, S5K3L6_REG_STREAM, 0x0100);
	if (!ret)
		ret = s5k3l6_write16(s, S5K3L6_REG_PLL_POWER, 0x0000);
	if (ret)
		goto err;
	return 0;

err:
	s->streaming = false;
	s5k3l6_power_off(s);
	return ret;
}

static int s5k3l6_disable_streams(struct v4l2_subdev *sd, struct v4l2_subdev_state *state,
				  u32 pad, u64 streams_mask)
{
	struct s5k3l6 *s = to_s5k3l6(sd);

	s5k3l6_write16(s, S5K3L6_REG_STREAM, 0x0000);
	msleep(40);	/* a frame at 30 fps */
	s->streaming = false;
	s5k3l6_power_off(s);
	return 0;
}

static int s5k3l6_init_state(struct v4l2_subdev *sd, struct v4l2_subdev_state *state)
{
	s5k3l6_fill_fmt(&s5k3l6_modes[0], v4l2_subdev_state_get_format(state, 0));
	return 0;
}

static const struct v4l2_subdev_video_ops s5k3l6_video_ops = {
	.s_stream = v4l2_subdev_s_stream_helper,
};

static const struct v4l2_subdev_pad_ops s5k3l6_pad_ops = {
	.enum_mbus_code = s5k3l6_enum_mbus_code,
	.enum_frame_size = s5k3l6_enum_frame_size,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = s5k3l6_set_fmt,
	.get_selection = s5k3l6_get_selection,
	.enable_streams = s5k3l6_enable_streams,
	.disable_streams = s5k3l6_disable_streams,
};

static const struct v4l2_subdev_ops s5k3l6_subdev_ops = {
	.video = &s5k3l6_video_ops,
	.pad = &s5k3l6_pad_ops,
};

static const struct v4l2_subdev_internal_ops s5k3l6_internal_ops = {
	.init_state = s5k3l6_init_state,
};

/* bring-up: a sensor that does not answer keeps its power this long, to measure the rails */
static uint probe_hold_ms;
module_param(probe_hold_ms, uint, 0644);

static int s5k3l6_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct s5k3l6 *s;
	u16 model = 0, rev = 0;
	int i, ret;

	s = devm_kzalloc(dev, sizeof(*s), GFP_KERNEL);
	if (!s)
		return -ENOMEM;
	s->client = client;
	s->mode = &s5k3l6_modes[0];

	s->mclk = devm_clk_get(dev, NULL);
	if (IS_ERR(s->mclk))
		return dev_err_probe(dev, PTR_ERR(s->mclk), "no mclk\n");
	ret = clk_set_rate(s->mclk, S5K3L6_MCLK_HZ);
	if (ret)
		return dev_err_probe(dev, ret, "mclk rate\n");

	/* asserted = held in reset */
	s->reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(s->reset))
		return dev_err_probe(dev, PTR_ERR(s->reset), "no reset gpio\n");

	for (i = 0; i < ARRAY_SIZE(s->supplies); i++)
		s->supplies[i].supply = s5k3l6_supplies[i];
	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(s->supplies), s->supplies);
	if (ret)
		return dev_err_probe(dev, ret, "supplies\n");

	ret = s5k3l6_power_on(s);
	if (ret)
		return dev_err_probe(dev, ret, "power on\n");
	ret = s5k3l6_read16(s, S5K3L6_REG_MODEL_ID, &model);
	if (!ret)
		ret = s5k3l6_read16(s, S5K3L6_REG_REVISION, &rev);
	if (ret && probe_hold_ms) {
		dev_info(dev, "no answer: holding power %u ms\n", probe_hold_ms);
		msleep(probe_hold_ms);
	}
	s5k3l6_power_off(s);
	if (ret)
		return dev_err_probe(dev, ret, "no answer at %#x\n", client->addr);
	dev_info(dev, "model %#06x revision %#06x, mclk %lu Hz\n", model, rev,
		 clk_get_rate(s->mclk));
	if (model != S5K3L6_MODEL_ID)
		return dev_err_probe(dev, -ENODEV, "not an S5K3L6\n");

	v4l2_i2c_subdev_init(&s->sd, client, &s5k3l6_subdev_ops);
	s->sd.internal_ops = &s5k3l6_internal_ops;
	s->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	s->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;

	ret = s5k3l6_init_ctrls(s);
	if (ret)
		return dev_err_probe(dev, ret, "controls\n");

	s->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&s->sd.entity, 1, &s->pad);
	if (ret)
		goto err_ctrls;

	s->sd.state_lock = s->ctrls.lock;
	ret = v4l2_subdev_init_finalize(&s->sd);
	if (ret)
		goto err_entity;

	ret = v4l2_async_register_subdev_sensor(&s->sd);
	if (ret)
		goto err_state;
	return 0;

err_state:
	v4l2_subdev_cleanup(&s->sd);
err_entity:
	media_entity_cleanup(&s->sd.entity);
err_ctrls:
	v4l2_ctrl_handler_free(&s->ctrls);
	return ret;
}

static void s5k3l6_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct s5k3l6 *s = to_s5k3l6(sd);

	v4l2_async_unregister_subdev(sd);
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(&s->ctrls);
	if (s->streaming)
		s5k3l6_power_off(s);
}

static const struct of_device_id s5k3l6_of_match[] = {
	{ .compatible = "samsung,s5k3l6" },
	{ }
};
MODULE_DEVICE_TABLE(of, s5k3l6_of_match);

static struct i2c_driver s5k3l6_driver = {
	.driver = {
		.name = "s5k3l6",
		.of_match_table = s5k3l6_of_match,
	},
	.probe = s5k3l6_probe,
	.remove = s5k3l6_remove,
};
module_i2c_driver(s5k3l6_driver);

MODULE_DESCRIPTION("Samsung S5K3L6 image sensor");
MODULE_LICENSE("GPL");
