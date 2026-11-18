// SPDX-License-Identifier: GPL-2.0
/*
 * Samsung EA8079 panel of the Galaxy S20 FE (r8s): 1080x2400 AMOLED, MIPI DSI command mode, 4 lanes, DSC with two
 * 540x40 slices, 120 Hz.
 *
 * The bootloader lights the panel and the DSI link, and nothing on mainline can power-cycle it yet (no DSIM/D-PHY
 * init, no display PMIC sequencing), so this driver takes it over lit: off is DISPLAY_OFF + SLEEP_IN, on is
 * SLEEP_OUT, the init registers again, then DISPLAY_ON. Reset and supplies are never touched.
 *
 * Sequences and bytes are from the Samsung vendor kernel, panel/ea8079/ea8079_r8s_panel.h (init, display-on,
 * display-off, exit and set-brightness command tables, the 120 Hz rows of its maps). A non-zero offset there is a
 * B0h write before the register (this DDI: one offset byte), and a frame between the two would take the offset, so
 * sequences with offsets keep frames off the link (exynos990_decon_hold_frames()). Brightness is the DBV of the
 * vendor's 256 normal steps; the HBM steps are left out.
 */

#include <linux/backlight.h>
#include <linux/cleanup.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>

#include <video/mipi_display.h>

#include <drm/display/drm_dsc.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>
#include <drm/drm_probe_helper.h>

#include "../tiny/exynos990-decon.h"

#define MAX_BRIGHTNESS	255

struct r8s_panel {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
	struct backlight_device *bl;
	struct mutex lock;	/* the command sequences against the backlight */
	bool asleep;		/* SLEEP_IN sent; the bootloader leaves it awake */
	bool display_on;
};

static inline struct r8s_panel *to_r8s(struct drm_panel *panel)
{
	return container_of(panel, struct r8s_panel, panel);
}

#define key1_on(ctx)	mipi_dsi_dcs_write_seq_multi(ctx, 0xf0, 0x5a, 0x5a)
#define key1_off(ctx)	mipi_dsi_dcs_write_seq_multi(ctx, 0xf0, 0xa5, 0xa5)
#define key2_on(ctx)	mipi_dsi_dcs_write_seq_multi(ctx, 0xfc, 0x5a, 0x5a)
#define key2_off(ctx)	mipi_dsi_dcs_write_seq_multi(ctx, 0xfc, 0xa5, 0xa5)
/* the parameter the next write starts at */
#define gpara(ctx, ofs)	mipi_dsi_dcs_write_seq_multi(ctx, 0xb0, ofs)

/* WRDISBV of the vendor's normal steps 0-255 (r8s_brt_table) */
static const u16 r8s_dbv[MAX_BRIGHTNESS + 1] = {
	0x003, 0x003, 0x006, 0x006, 0x009, 0x009, 0x00c, 0x00c, 0x00f, 0x00f,
	0x012, 0x015, 0x018, 0x018, 0x01b, 0x01d, 0x01f, 0x022, 0x024, 0x027,
	0x029, 0x02c, 0x02e, 0x031, 0x033, 0x036, 0x039, 0x03c, 0x03e, 0x041,
	0x043, 0x046, 0x049, 0x04c, 0x04f, 0x052, 0x055, 0x058, 0x05b, 0x05e,
	0x061, 0x064, 0x067, 0x06a, 0x06d, 0x070, 0x073, 0x077, 0x07a, 0x07d,
	0x080, 0x084, 0x087, 0x08a, 0x08d, 0x091, 0x094, 0x098, 0x09b, 0x09e,
	0x0a2, 0x0a5, 0x0a9, 0x0ac, 0x0b0, 0x0b3, 0x0b7, 0x0ba, 0x0be, 0x0c1,
	0x0c5, 0x0c9, 0x0cc, 0x0d0, 0x0d4, 0x0d7, 0x0db, 0x0df, 0x0e2, 0x0e6,
	0x0ea, 0x0ee, 0x0f1, 0x0f5, 0x0f9, 0x0fd, 0x100, 0x104, 0x108, 0x10c,
	0x110, 0x114, 0x118, 0x11c, 0x120, 0x124, 0x127, 0x12b, 0x12f, 0x133,
	0x137, 0x13b, 0x13f, 0x143, 0x148, 0x14c, 0x150, 0x154, 0x158, 0x15c,
	0x160, 0x164, 0x168, 0x16d, 0x171, 0x175, 0x179, 0x17d, 0x181, 0x186,
	0x18a, 0x18e, 0x192, 0x197, 0x19b, 0x19f, 0x1a3, 0x1a8, 0x1ac, 0x1b0,
	0x1b5, 0x1b9, 0x1bd, 0x1c4, 0x1c8, 0x1cc, 0x1d1, 0x1d5, 0x1da, 0x1de,
	0x1e2, 0x1e7, 0x1eb, 0x1f0, 0x1f4, 0x1f8, 0x1fd, 0x201, 0x206, 0x20a,
	0x20f, 0x213, 0x218, 0x21c, 0x221, 0x225, 0x22a, 0x22e, 0x233, 0x238,
	0x23c, 0x241, 0x245, 0x24a, 0x24e, 0x253, 0x258, 0x25c, 0x261, 0x266,
	0x26a, 0x26f, 0x274, 0x278, 0x27d, 0x282, 0x286, 0x28b, 0x290, 0x295,
	0x299, 0x29e, 0x2a3, 0x2a8, 0x2ac, 0x2b1, 0x2b6, 0x2bb, 0x2bf, 0x2c4,
	0x2c9, 0x2ce, 0x2d3, 0x2d8, 0x2dc, 0x2e1, 0x2e6, 0x2eb, 0x2f0, 0x2f5,
	0x2fa, 0x2ff, 0x303, 0x308, 0x30d, 0x312, 0x317, 0x31c, 0x321, 0x326,
	0x32b, 0x330, 0x335, 0x33a, 0x33f, 0x344, 0x349, 0x34b, 0x351, 0x355,
	0x359, 0x35e, 0x362, 0x366, 0x36b, 0x370, 0x375, 0x37a, 0x37f, 0x385,
	0x38b, 0x38f, 0x394, 0x399, 0x39e, 0x3a2, 0x3a6, 0x3ab, 0x3af, 0x3b4,
	0x3b9, 0x3bd, 0x3c2, 0x3c7, 0x3cc, 0x3d1, 0x3d6, 0x3db, 0x3e1, 0x3e6,
	0x3ec, 0x3ef, 0x3f3, 0x3f7, 0x3fb, 0x3ff,
};

/* the PPS of r8s_pps: 1080x2400, 540x40 slices, 8 bpc in, 8 bpp out */
static const u8 r8s_pps[] = {
	0x11, 0x00, 0x00, 0x89, 0x30, 0x80, 0x09, 0x60, 0x04, 0x38, 0x00, 0x28, 0x02, 0x1c, 0x02, 0x1c,
	0x02, 0x00, 0x02, 0x0e, 0x00, 0x20, 0x03, 0xdd, 0x00, 0x07, 0x00, 0x0c, 0x02, 0x77, 0x02, 0x8b,
	0x18, 0x00, 0x10, 0xf0, 0x03, 0x0c, 0x20, 0x00, 0x06, 0x0b, 0x0b, 0x33, 0x0e, 0x1c, 0x2a, 0x38,
	0x46, 0x54, 0x62, 0x69, 0x70, 0x77, 0x79, 0x7b, 0x7d, 0x7e, 0x01, 0x02, 0x01, 0x00, 0x09, 0x40,
	0x09, 0xbe, 0x19, 0xfc, 0x19, 0xfa, 0x19, 0xf8, 0x1a, 0x38, 0x1a, 0x78, 0x1a, 0xb6, 0x2a, 0xf6,
	0x2b, 0x34, 0x2b, 0x74, 0x3b, 0x74, 0x6b, 0xf4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
};
static_assert(sizeof(r8s_pps) == sizeof(struct drm_dsc_picture_parameter_set));

static void r8s_write_dbv(struct mipi_dsi_multi_context *ctx, int level)
{
	u16 dbv = r8s_dbv[clamp(level, 0, MAX_BRIGHTNESS)];
	u8 cmd[] = { MIPI_DCS_SET_DISPLAY_BRIGHTNESS, dbv >> 8, dbv & 0xff };

	mipi_dsi_dcs_write_buffer_multi(ctx, cmd, sizeof(cmd));
}

/* set-bl's DBV, behind the keys; the ELVSS, temperature and ACL registers keep what init or the bootloader wrote */
static void r8s_set_brightness(struct mipi_dsi_multi_context *ctx, int level)
{
	key1_on(ctx);
	key2_on(ctx);
	mipi_dsi_dcs_write_seq_multi(ctx, 0x53, 0x20);		/* HBM off, no smooth transition */
	r8s_write_dbv(ctx, level);
	key2_off(ctx);
	key1_off(ctx);
}

static int r8s_prepare(struct drm_panel *panel)
{
	struct r8s_panel *rp = to_r8s(panel);
	struct mipi_dsi_multi_context ctx = { .dsi = rp->dsi };
	struct drm_dsc_picture_parameter_set pps;
	unsigned int got_in;

	guard(mutex)(&rp->lock);
	if (!rp->asleep)
		return 0;

	memcpy(&pps, r8s_pps, sizeof(pps));
	exynos990_decon_hold_frames(rp->dsi->host, true);

	/* init-seq without the reset and supply steps */
	mipi_dsi_dcs_exit_sleep_mode_multi(&ctx);
	key1_on(&ctx);
	key2_on(&ctx);
	mipi_dsi_msleep(&ctx, 10);
	mipi_dsi_compression_mode_multi(&ctx, true);
	mipi_dsi_picture_parameter_set_multi(&ctx, &pps);
	mipi_dsi_dcs_write_seq_multi(&ctx, MIPI_DCS_SET_COLUMN_ADDRESS, 0x00, 0x00, 0x04, 0x37);
	mipi_dsi_dcs_write_seq_multi(&ctx, MIPI_DCS_SET_PAGE_ADDRESS, 0x00, 0x00, 0x09, 0x5f);
	/* FFC for the 1110 Mbit/s lane rate the bootloader programs */
	gpara(&ctx, 0x01);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xe4, 0xa4, 0xf5, 0xbb);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xe9, 0x11, 0x75, 0xa4, 0xf5, 0xbb, 0x9e, 0x1f, 0x46,
				     0x9e, 0x1f, 0x46, 0x00, 0x32, 0x32);
	gpara(&ctx, 0x01);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xe6, 0x01);			/* VSYNC */
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xe8, 0x57, 0x1a);		/* PCD: self reset and sleep */
	gpara(&ctx, 0x06);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xb7, 0x08);			/* smooth dimming */
	gpara(&ctx, 0x03);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xd4, 0x10, 0x10);		/* SAP */
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xf7, 0x03);			/* panel update */
	gpara(&ctx, 0x09);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xee, 0x00);			/* AOD off */
	/* set-fps at 120 Hz */
	mipi_dsi_dcs_write_seq_multi(&ctx, 0x60, 0x10);
	gpara(&ctx, 0x0b);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xd8, 0xa9, 0x70);
	gpara(&ctx, 0xd4);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xba, 0x12, 0xe0, 0x09, 0x2e, 0x09, 0x03, 0x08, 0x96,
				     0x08, 0x2c, 0x07, 0x52, 0x06, 0x42, 0x05, 0x2a,
				     0x03, 0xda, 0x02, 0x82, 0x01, 0x2c, 0x00, 0x14);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xbb, 0x7a, 0x01, 0x25, 0x00, 0x14, 0x20, 0x30);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xf7, 0x03);
	/* set-bl: normal ELVSS, ACL at 16 frames from 50 %, dimming over 32 frames, on (OPR 3-8 %), the DBV */
	mipi_dsi_dcs_write_seq_multi(&ctx, 0x53, 0x20);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xb7, 0x01, 0x5c, 0xcb, 0x0d, 0xc0, 0x16);
	gpara(&ctx, 0xca);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xbb, 0x09);
	gpara(&ctx, 0xcf);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xbb, 0x41, 0xff);
	gpara(&ctx, 0xd9);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xbb, 0x20);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0x55, 0x01);
	r8s_write_dbv(&ctx, backlight_get_brightness(rp->bl));
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xdf, 0x00, 0x00, 0x00, 0x01, 0x95, 0x80, 0x0c);	/* TE offset */
	mipi_dsi_dcs_set_tear_on_multi(&ctx, MIPI_DSI_DCS_TEAR_MODE_VBLANK);
	key2_off(&ctx);
	key1_off(&ctx);

	got_in = exynos990_decon_hold_frames(rp->dsi->host, false);
	if (got_in)
		dev_warn(&rp->dsi->dev, "%u frames reached the link during init\n", got_in);
	mipi_dsi_msleep(&ctx, 110);

	if (!ctx.accum_err)
		rp->asleep = false;
	return ctx.accum_err;
}

static int r8s_enable(struct drm_panel *panel)
{
	struct r8s_panel *rp = to_r8s(panel);
	struct mipi_dsi_multi_context ctx = { .dsi = rp->dsi };

	if (rp->display_on)
		return 0;

	/* the vendor driver turns the display on after the first frame */
	mipi_dsi_msleep(&ctx, 17);
	key1_on(&ctx);
	mipi_dsi_dcs_set_display_on_multi(&ctx);
	key1_off(&ctx);

	if (!ctx.accum_err)
		rp->display_on = true;
	return ctx.accum_err;
}

static int r8s_disable(struct drm_panel *panel)
{
	struct r8s_panel *rp = to_r8s(panel);
	struct mipi_dsi_multi_context ctx = { .dsi = rp->dsi };

	key1_on(&ctx);
	mipi_dsi_dcs_set_display_off_multi(&ctx);
	key1_off(&ctx);
	rp->display_on = false;
	return ctx.accum_err;
}

static int r8s_unprepare(struct drm_panel *panel)
{
	struct r8s_panel *rp = to_r8s(panel);
	struct mipi_dsi_multi_context ctx = { .dsi = rp->dsi };

	guard(mutex)(&rp->lock);
	/* exit-seq */
	exynos990_decon_hold_frames(rp->dsi->host, true);
	key1_on(&ctx);
	key2_on(&ctx);
	gpara(&ctx, 0x0d);
	mipi_dsi_dcs_write_seq_multi(&ctx, 0xec, 0x11);			/* ERR_FG off */
	mipi_dsi_dcs_enter_sleep_mode_multi(&ctx);
	key2_off(&ctx);
	key1_off(&ctx);
	exynos990_decon_hold_frames(rp->dsi->host, false);
	mipi_dsi_msleep(&ctx, 120);
	rp->asleep = true;
	return ctx.accum_err;
}

static const struct drm_display_mode r8s_mode = {
	/* command mode: these only pace vblank; porches from the vendor DT */
	.clock = 1086 * 2419 * 120 / 1000,
	.hdisplay = 1080,
	.hsync_start = 1080 + 2,
	.hsync_end = 1080 + 2 + 2,
	.htotal = 1080 + 2 + 2 + 2,
	.vdisplay = 2400,
	.vsync_start = 2400 + 3,
	.vsync_end = 2400 + 3 + 1,
	.vtotal = 2400 + 3 + 1 + 15,
	.width_mm = 67,
	.height_mm = 150,
	.type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED,
};

static int r8s_get_modes(struct drm_panel *panel, struct drm_connector *connector)
{
	return drm_connector_helper_get_modes_fixed(connector, &r8s_mode);
}

static const struct drm_panel_funcs r8s_panel_funcs = {
	.prepare = r8s_prepare,
	.enable = r8s_enable,
	.disable = r8s_disable,
	.unprepare = r8s_unprepare,
	.get_modes = r8s_get_modes,
};

static int r8s_bl_update_status(struct backlight_device *bl)
{
	struct r8s_panel *rp = bl_get_data(bl);
	struct mipi_dsi_multi_context ctx = { .dsi = rp->dsi };

	guard(mutex)(&rp->lock);
	/* applied by prepare when the panel wakes up */
	if (rp->asleep)
		return 0;

	r8s_set_brightness(&ctx, backlight_get_brightness(bl));
	return ctx.accum_err;
}

static const struct backlight_ops r8s_bl_ops = {
	.update_status = r8s_bl_update_status,
};

static int r8s_probe(struct mipi_dsi_device *dsi)
{
	struct backlight_properties props = {
		.type = BACKLIGHT_RAW,
		.brightness = MAX_BRIGHTNESS / 2,
		.max_brightness = MAX_BRIGHTNESS,
		.scale = BACKLIGHT_SCALE_NON_LINEAR,
	};
	struct device *dev = &dsi->dev;
	struct r8s_panel *rp;
	int ret;

	rp = devm_drm_panel_alloc(dev, struct r8s_panel, panel, &r8s_panel_funcs, DRM_MODE_CONNECTOR_DSI);
	if (IS_ERR(rp))
		return PTR_ERR(rp);

	rp->dsi = dsi;
	rp->display_on = true;
	ret = devm_mutex_init(dev, &rp->lock);
	if (ret)
		return ret;
	mipi_dsi_set_drvdata(dsi, rp);

	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_CLOCK_NON_CONTINUOUS;

	rp->bl = devm_backlight_device_register(dev, dev_name(dev), dev, rp, &r8s_bl_ops, &props);
	if (IS_ERR(rp->bl))
		return dev_err_probe(dev, PTR_ERR(rp->bl), "failed to register backlight\n");
	rp->panel.backlight = rp->bl;

	drm_panel_add(&rp->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret) {
		drm_panel_remove(&rp->panel);
		return dev_err_probe(dev, ret, "failed to attach to DSI host\n");
	}

	/* the lit panel shows the bootloader's brightness until told otherwise */
	backlight_update_status(rp->bl);
	return 0;
}

static void r8s_remove(struct mipi_dsi_device *dsi)
{
	struct r8s_panel *rp = mipi_dsi_get_drvdata(dsi);

	mipi_dsi_detach(dsi);
	drm_panel_remove(&rp->panel);
}

static const struct of_device_id r8s_of_match[] = {
	{ .compatible = "samsung,ea8079-r8s" },
	{ }
};
MODULE_DEVICE_TABLE(of, r8s_of_match);

static struct mipi_dsi_driver r8s_driver = {
	.probe = r8s_probe,
	.remove = r8s_remove,
	.driver = {
		.name = "panel-samsung-ea8079-r8s",
		.of_match_table = r8s_of_match,
	},
};
module_mipi_dsi_driver(r8s_driver);

MODULE_DESCRIPTION("Samsung EA8079 panel of the Galaxy S20 FE (bootloader handover)");
MODULE_LICENSE("GPL");
