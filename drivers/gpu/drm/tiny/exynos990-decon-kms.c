// SPDX-License-Identifier: GPL-2.0
//
// Exynos 990 DECON0 "handover" KMS: page flips on the pipeline S-Boot left
// running (Galaxy S20 FE: EA8079 at 1080x2400, 120 Hz, MIPI command mode with
// DSC, the hardware TE trigger left unmasked, so DECON fetches a frame on
// every TE).
//
// Nothing here initialises DECON, DSIM, the D-PHY, DSC, clocks or the power
// domain. A flip writes the IDMA channel 0 base and DECON's shadow-update
// request; FRAME_START is vblank. The DPU SysMMUs are left in bypass, so the
// base is a physical address and every buffer has to sit below 4 GiB.
// Disabling the CRTC hands the scanout back to S-Boot's framebuffer.
//
// With psr=1 (the default) the trigger is masked again once nothing wants a
// frame: the panel then refreshes itself from its own RAM, and DECON, the
// IDMA and the DSI link stop moving 10 MB a frame at 120 Hz. Every update,
// and every vblank reference, unmasks it.
//
// DSIM0 is offered as a MIPI DSI host for panel commands, through its command
// FIFO on the link S-Boot brought up. With a panel node under ours the DRM
// device registers once the panel attaches; without one it registers at once
// with a fixed-mode connector, and the panel stays as S-Boot left it.

#include <linux/aperture.h>
#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/seq_file.h>

#include <drm/clients/drm_client_setup.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_bridge.h>
#include <drm/drm_connector.h>
#include <drm/drm_debugfs.h>
#include <drm/drm_drv.h>
#include <drm/drm_fb_dma_helper.h>
#include <drm/drm_fbdev_dma.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_gem_atomic_helper.h>
#include <drm/drm_gem_dma_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_modeset_helper_vtables.h>
#include <drm/drm_of.h>
#include <drm/drm_panel.h>
#include <drm/drm_print.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_simple_kms_helper.h>
#include <drm/drm_vblank.h>

#include "exynos990-decon.h"

/* DECON0 (dpu30 cal_9830 regs-decon.h) */
#define DECON_GLOBAL_CONTROL		0x0000
#define DECON_GC_CMD_MODE		BIT(8)
#define DECON_GC_IDLE_STATUS		BIT(5)
#define DECON_GC_RUN_STATUS		BIT(4)
#define DECON_GC_EN			BIT(1)
#define DECON_GC_EN_F			BIT(0)
#define DECON_GLOBAL_RUN		(DECON_GC_CMD_MODE | DECON_GC_RUN_STATUS | DECON_GC_EN | DECON_GC_EN_F)
#define DECON_INTERRUPT_PENDING		0x004c
#define DECON_INT_FRAME_DONE		BIT(13)
#define DECON_INT_FRAME_START		BIT(12)
#define DECON_INT_EXTRA			BIT(4)
#define DECON_EXTRA_INT_PENDING		0x0050
#define DECON_EXTRA_ERRORS		(BIT(8) | BIT(4))	/* resource conflict, timeout */
#define DECON_SHADOW_REG_UPDATE_REQ	0x0060
#define DECON_SHADOW_GLOBAL		BIT(31)
#define DECON_HW_SW_TRIG_CONTROL	0x0070
#define DECON_HW_TRIG_EN		BIT(0)
#define DECON_HW_TRIG_MASK_DECON	BIT(4)
#define DECON_HW_TE_CNT			0x0078	/* TE pulses from the panel, both halves */
#define DECON_BLENDER_BG_IMAGE_SIZE_0	0x0200
#define DECON_DATA_PATH_CONTROL_0	0x0214
#define DECON_DATA_PATH_CONTROL_1	0x0218
#define DECON_FRAME_COUNT		0x02a0
#define DECON_NR_WINDOWS		6

/* IDMA channel 0 (dpu30 cal_9830 regs-dpp.h; the DPP node's first reg range) */
#define IDMA_IN_CON			0x0008
#define IDMA_IMG_FORMAT			GENMASK(13, 8)
#define IDMA_FMT_ARGB8888		0
#define IDMA_FMT_XRGB8888		4
#define IDMA_SRC_SIZE			0x0010
#define IDMA_IMG_SIZE			0x0018
#define IDMA_IN_BASE_ADDR_Y8		0x0040

/* DSIM0 (dpu30 cal_9830 regs-dsim.h) */
#define DSIM_INTSRC			0x0050
/*
 * PH overflow, invalid SFR, LPRX/BTA timeouts, underrun. Not ABNRMAL_CMD_ST (bit 22): this link raises it on every
 * frame, from before the handover on, and the vendor driver keeps it masked and never looks at it.
 */
#define DSIM_INTSRC_ERRORS		(BIT(27) | BIT(23) | GENMASK(21, 19))
#define DSIM_PKTHDR			0x0058
#define DSIM_PAYLOAD			0x005c
#define DSIM_RXFIFO			0x0060
#define DSIM_FIFOCTRL			0x0068
#define DSIM_FULL_PH_SFR		BIT(11)
#define DSIM_EMPTY_PH_SFR		BIT(10)
#define DSIM_FULL_PL_SFR		BIT(9)

#define R8S_WIDTH			1080
#define R8S_HEIGHT			2400
#define R8S_SIZE_REG			((R8S_HEIGHT << 16) | R8S_WIDTH)
#define R8S_FB_SIZE			(R8S_WIDTH * R8S_HEIGHT * 4)

struct decon_kms {
	struct drm_device drm;
	struct drm_simple_display_pipe pipe;
	struct drm_connector connector;	/* without a panel driver */

	void __iomem *decon;
	void __iomem *idma;
	void __iomem *dsim;
	unsigned int win;
	int irq;

	/* PSR: which frames the TE trigger still has to start (trig_lock) */
	spinlock_t trig_lock;
	unsigned int trig_frames;
	bool trig_vblank;
	bool trig_masked;
	/* the panel's command sequence under way: no frame starts (trig_lock) */
	bool trig_held;
	/* frame interrupts that came anyway, since the hold took effect (trig_lock) */
	unsigned int held_frames;

	struct mipi_dsi_host dsi_host;
	struct mutex dsi_lock;		/* one packet in the FIFO at a time */
	bool stuck_dumped;		/* the registers of the first stuck FIFO are in the log (dsi_lock) */
	bool registered;

	/* what S-Boot was scanning out, restored (blacked) on disable */
	void *boot_fb;
	u32 boot_base;
	u32 boot_in_con;
	u32 boot_src_size;
};

static bool psr = true;
module_param(psr, bool, 0644);
MODULE_PARM_DESC(psr, "send frames only when the picture changes (default: 1)");

static inline struct decon_kms *to_decon(struct drm_device *drm)
{
	return container_of(drm, struct decon_kms, drm);
}

static void rmw(void __iomem *base, u32 reg, u32 mask, u32 val)
{
	writel((readl(base + reg) & ~mask) | (val & mask), base + reg);
}

/*
 * Let the next TE start a frame, and keep going for @frames of them. Two
 * after an update: the first latches it, and the second is the vblank that
 * drm_atomic_helper_wait_for_vblanks() may only start counting after the
 * first one has already gone by.
 */
static void decon_trig_kick(struct decon_kms *dk, unsigned int frames)
{
	unsigned long flags;

	spin_lock_irqsave(&dk->trig_lock, flags);
	dk->trig_frames = max(dk->trig_frames, frames);
	if (dk->trig_masked && !dk->trig_held) {
		rmw(dk->decon, DECON_HW_SW_TRIG_CONTROL, DECON_HW_TRIG_MASK_DECON, 0);
		dk->trig_masked = false;
	}
	spin_unlock_irqrestore(&dk->trig_lock, flags);
}

/* At FRAME_START: mask the trigger once no one wants the next frame. */
static void decon_trig_idle(struct decon_kms *dk)
{
	spin_lock(&dk->trig_lock);
	if (dk->trig_frames)
		dk->trig_frames--;
	if (READ_ONCE(psr) && !dk->trig_frames && !dk->trig_vblank && !dk->trig_masked &&
	    !readl(dk->decon + DECON_SHADOW_REG_UPDATE_REQ)) {
		rmw(dk->decon, DECON_HW_SW_TRIG_CONTROL, DECON_HW_TRIG_MASK_DECON,
		    DECON_HW_TRIG_MASK_DECON);
		dk->trig_masked = true;
	}
	spin_unlock(&dk->trig_lock);
}

/* Point IDMA channel 0 at a buffer and have DECON latch it at the next frame start. */
static void decon_scanout(struct decon_kms *dk, u32 base, u32 in_con, u32 src_size)
{
	u32 val;

	/* the previous request must have been latched first (<= 1 frame) */
	if (readl_poll_timeout(dk->decon + DECON_SHADOW_REG_UPDATE_REQ, val, !val, 100, 50000))
		dev_warn_ratelimited(dk->drm.dev, "shadow update stuck: 0x%08x (trigger 0x%08x, masked %d, held %d)\n",
				     val, readl(dk->decon + DECON_HW_SW_TRIG_CONTROL), dk->trig_masked, dk->trig_held);

	writel(in_con, dk->idma + IDMA_IN_CON);
	writel(src_size, dk->idma + IDMA_SRC_SIZE);
	writel(base, dk->idma + IDMA_IN_BASE_ADDR_Y8);
	writel(DECON_SHADOW_GLOBAL | BIT(dk->win), dk->decon + DECON_SHADOW_REG_UPDATE_REQ);
}

static int decon_pipe_check(struct drm_simple_display_pipe *pipe,
			    struct drm_plane_state *plane_state,
			    struct drm_crtc_state *crtc_state)
{
	struct drm_framebuffer *fb = plane_state->fb;
	dma_addr_t addr;
	int ret;

	ret = drm_atomic_helper_check_plane_state(plane_state, crtc_state,
						  DRM_PLANE_NO_SCALING, DRM_PLANE_NO_SCALING,
						  false, false);
	if (ret || !fb || !plane_state->visible)
		return ret;

	/* full-screen, unpanned, stride a whole number of pixels */
	if (plane_state->src.x1 || plane_state->src.y1 ||
	    fb->width < R8S_WIDTH || fb->height < R8S_HEIGHT ||
	    fb->pitches[0] % 4 || fb->pitches[0] / 4 > 0xffff)
		return -EINVAL;

	/* the SysMMU is in bypass and the base register is 32 bits wide */
	addr = drm_fb_dma_get_gem_addr(fb, plane_state, 0);
	if (upper_32_bits(addr + (u64)fb->pitches[0] * R8S_HEIGHT - 1))
		return -EINVAL;

	return 0;
}

static void decon_pipe_update(struct drm_simple_display_pipe *pipe,
			      struct drm_plane_state *old_state)
{
	struct decon_kms *dk = to_decon(pipe->crtc.dev);
	struct drm_plane_state *state = pipe->plane.state;
	struct drm_crtc *crtc = &pipe->crtc;
	struct drm_pending_vblank_event *event;
	struct drm_framebuffer *fb = state->fb;
	unsigned long flags;

	if (fb && state->visible) {
		dma_addr_t addr = drm_fb_dma_get_gem_addr(fb, state, 0);
		u32 fmt = fb->format->format == DRM_FORMAT_ARGB8888 ?
			  IDMA_FMT_ARGB8888 : IDMA_FMT_XRGB8888;

		decon_scanout(dk, lower_32_bits(addr),
			      (dk->boot_in_con & ~IDMA_IMG_FORMAT) | FIELD_PREP(IDMA_IMG_FORMAT, fmt),
			      (R8S_HEIGHT << 16) | (fb->pitches[0] / 4));
	}
	/* a damage-only commit (DirtyFB, fbdev) changes pixels in place */
	decon_trig_kick(dk, 2);

	event = crtc->state->event;
	if (!event)
		return;
	crtc->state->event = NULL;

	spin_lock_irqsave(&crtc->dev->event_lock, flags);
	if (drm_crtc_vblank_get(crtc) == 0)
		drm_crtc_arm_vblank_event(crtc, event);
	else
		drm_crtc_send_vblank_event(crtc, event);
	spin_unlock_irqrestore(&crtc->dev->event_lock, flags);
}

static void decon_pipe_enable(struct drm_simple_display_pipe *pipe,
			      struct drm_crtc_state *crtc_state,
			      struct drm_plane_state *plane_state)
{
	/*
	 * The pipeline is already running; only the buffer changes. Vblank
	 * goes off as soon as no one holds it, so an idle screen sends nothing.
	 * Without a hardware counter the count only moves on real frames,
	 * which with PSR is what a vblank is.
	 */
	const struct drm_vblank_crtc_config config = {
		.offdelay_ms = 5000,
		.disable_immediate = true,
	};

	drm_crtc_vblank_on_config(&pipe->crtc, &config);
}

static void decon_pipe_disable(struct drm_simple_display_pipe *pipe)
{
	struct decon_kms *dk = to_decon(pipe->crtc.dev);

	drm_crtc_vblank_off(&pipe->crtc);
	/*
	 * Our buffers may be freed now: scan out S-Boot's reserved one again.
	 * Without a panel driver the panel cannot be switched off, so "off" is
	 * black, which on OLED is pixels off.
	 */
	memset(dk->boot_fb, 0, R8S_FB_SIZE);
	wmb();
	decon_scanout(dk, dk->boot_base, dk->boot_in_con, dk->boot_src_size);
	decon_trig_kick(dk, 2);
}

/* FRAME_START comes only with a frame, so wanting vblanks means frames */
static int decon_pipe_enable_vblank(struct drm_simple_display_pipe *pipe)
{
	struct decon_kms *dk = to_decon(pipe->crtc.dev);
	unsigned long flags;

	spin_lock_irqsave(&dk->trig_lock, flags);
	dk->trig_vblank = true;
	spin_unlock_irqrestore(&dk->trig_lock, flags);
	decon_trig_kick(dk, 1);
	return 0;
}

static void decon_pipe_disable_vblank(struct drm_simple_display_pipe *pipe)
{
	struct decon_kms *dk = to_decon(pipe->crtc.dev);
	unsigned long flags;

	spin_lock_irqsave(&dk->trig_lock, flags);
	dk->trig_vblank = false;
	spin_unlock_irqrestore(&dk->trig_lock, flags);
}

static const struct drm_simple_display_pipe_funcs decon_pipe_funcs = {
	.check = decon_pipe_check,
	.update = decon_pipe_update,
	.enable = decon_pipe_enable,
	.disable = decon_pipe_disable,
	.enable_vblank = decon_pipe_enable_vblank,
	.disable_vblank = decon_pipe_disable_vblank,
};

static const u32 decon_formats[] = {
	DRM_FORMAT_XRGB8888,
	DRM_FORMAT_ARGB8888,
};

/* The panel as S-Boot runs it (EA8079: 67 x 150 mm, 120 Hz), for when no panel driver is bound */
static const struct drm_display_mode r8s_mode = {
	DRM_MODE_INIT(120, R8S_WIDTH, R8S_HEIGHT, 67, 150),
};

static int decon_connector_get_modes(struct drm_connector *connector)
{
	return drm_connector_helper_get_modes_fixed(connector, &r8s_mode);
}

static const struct drm_connector_helper_funcs decon_connector_helper_funcs = {
	.get_modes = decon_connector_get_modes,
};

static const struct drm_connector_funcs decon_connector_funcs = {
	.reset = drm_atomic_helper_connector_reset,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = drm_connector_cleanup,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static void decon_check_errors(struct decon_kms *dk, const char *when)
{
	u32 dsim = readl(dk->dsim + DSIM_INTSRC) & DSIM_INTSRC_ERRORS;
	u32 extra = readl(dk->decon + DECON_EXTRA_INT_PENDING) & DECON_EXTRA_ERRORS;

	if (!dsim && !extra)
		return;
	writel(dsim, dk->dsim + DSIM_INTSRC);
	writel(extra, dk->decon + DECON_EXTRA_INT_PENDING);
	dev_warn_ratelimited(dk->drm.dev, "link errors %s: DSIM 0x%08x, DECON 0x%03x\n", when, dsim, extra);
}

static irqreturn_t decon_irq(int irq, void *data)
{
	struct decon_kms *dk = data;
	u32 pend = readl(dk->decon + DECON_INTERRUPT_PENDING) &
		   (DECON_INT_FRAME_START | DECON_INT_FRAME_DONE | DECON_INT_EXTRA);

	/*
	 * The line is FRAME_START's alone (SPI 254; FRAME_DONE and EXTRA have 253 and 252, not wired here), so the
	 * other two bits are only cleared along with it, and whether a frame is running is DECON's own idle status
	 * (decon_frame_running()).
	 */
	if (!(pend & DECON_INT_FRAME_START))
		return IRQ_NONE;
	writel(pend, dk->decon + DECON_INTERRUPT_PENDING);

	drm_crtc_handle_vblank(&dk->pipe.crtc);
	decon_trig_idle(dk);
	decon_check_errors(dk, "at a frame start");

	spin_lock(&dk->trig_lock);
	if (dk->trig_held)
		dk->held_frames++;
	spin_unlock(&dk->trig_lock);
	return IRQ_HANDLED;
}

static const struct drm_mode_config_funcs decon_mode_config_funcs = {
	/* front-buffer writers (fbdev, DirtyFB) must commit to get a frame */
	.fb_create = drm_gem_fb_create_with_dirty,
	.atomic_check = drm_atomic_helper_check,
	.atomic_commit = drm_atomic_helper_commit,
};

DEFINE_DRM_GEM_DMA_FOPS(decon_fops);

static const struct drm_driver decon_driver = {
	.driver_features = DRIVER_GEM | DRIVER_MODESET | DRIVER_ATOMIC,
	/* "exynos": Mesa's kmsro then renders with the GPU driver into our buffers */
	.name = "exynos",
	.desc = "Exynos 990 DECON handover",
	.major = 1,
	.minor = 0,
	.fops = &decon_fops,
	DRM_GEM_DMA_DRIVER_OPS,
	DRM_FBDEV_DMA_DRIVER_OPS,
};

/*
 * DECON0 and DSIM0 as they are, for comparing one boot with another. Eight words to a line, lines of zeros left
 * out; the counters are read again 50 ms on, to show whether the panel still sends TE and DECON still starts
 * frames. DSIM's FIFO ports are skipped: reading RXFIFO pops it.
 */
static void decon_print_regs(struct decon_kms *dk, struct drm_printer *p, const char *why)
{
	static const struct {
		const char *name;
		bool dsim;
		u32 start, end;
	} blocks[] = {
		{ "decon", false, 0x0000, 0x02a8 },
		{ "decon", false, 0x1000, 0x1120 },
		{ "dsim", true, 0x0000, 0x0100 },
	};
	unsigned int b, i;
	u32 te, frames, off, v[8];

	for (b = 0; b < ARRAY_SIZE(blocks); b++) {
		void __iomem *base = blocks[b].dsim ? dk->dsim : dk->decon;

		for (off = blocks[b].start; off < blocks[b].end; off += sizeof(v)) {
			bool any = false;

			for (i = 0; i < ARRAY_SIZE(v); i++) {
				u32 reg = off + 4 * i;

				v[i] = reg >= blocks[b].end ||
				       (blocks[b].dsim && reg >= DSIM_PKTHDR && reg <= DSIM_RXFIFO) ?
				       0 : readl(base + reg);
				any |= v[i];
			}
			if (any)
				drm_printf(p, "%s%s+%04x %08x %08x %08x %08x %08x %08x %08x %08x\n", why,
					   blocks[b].name, off, v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7]);
		}
	}
	drm_printf(p, "%sidma0 in_con %08x src %08x img %08x base %08x\n", why,
		   readl(dk->idma + IDMA_IN_CON), readl(dk->idma + IDMA_SRC_SIZE),
		   readl(dk->idma + IDMA_IMG_SIZE), readl(dk->idma + IDMA_IN_BASE_ADDR_Y8));

	te = readl(dk->decon + DECON_HW_TE_CNT);
	frames = readl(dk->decon + DECON_FRAME_COUNT);
	msleep(50);
	drm_printf(p, "%sTE %08x -> %08x, frames %08x -> %08x in 50 ms\n", why, te,
		   readl(dk->decon + DECON_HW_TE_CNT), frames, readl(dk->decon + DECON_FRAME_COUNT));
}

static int decon_regs_show(struct seq_file *m, void *data)
{
	struct drm_debugfs_entry *entry = m->private;
	struct drm_printer p = drm_seq_file_printer(m);

	decon_print_regs(to_decon(entry->dev), &p, "");
	return 0;
}

/* Refuse anything but the state S-Boot is known to leave. */
static int decon_check_handover(struct decon_kms *dk, struct device *dev)
{
	struct drm_printer p = drm_info_printer(dev);
	u32 ctrl, trig, path0, path1;
	unsigned int w, nwin = 0;

	decon_print_regs(dk, &p, "handover: ");
	decon_check_errors(dk, "at the handover");
	ctrl = readl(dk->decon + DECON_GLOBAL_CONTROL);
	trig = readl(dk->decon + DECON_HW_SW_TRIG_CONTROL);
	path0 = readl(dk->decon + DECON_DATA_PATH_CONTROL_0);
	path1 = readl(dk->decon + DECON_DATA_PATH_CONTROL_1);

	if ((ctrl & DECON_GLOBAL_RUN) != DECON_GLOBAL_RUN ||
	    (trig & (DECON_HW_TRIG_EN | DECON_HW_TRIG_MASK_DECON)) != DECON_HW_TRIG_EN) {
		dev_err(dev, "DECON not running continuously (0x%08x 0x%08x)\n", ctrl, trig);
		return -ENODEV;
	}

	for (w = 0; w < DECON_NR_WINDOWS; w++) {
		if (path0 & BIT(4 * w)) {
			dk->win = w;
			nwin++;
		}
	}
	if (nwin != 1 || ((path1 >> (4 * dk->win)) & 7) != 0) {
		dev_err(dev, "expected one window on IDMA channel 0 (0x%08x 0x%08x)\n", path0, path1);
		return -ENODEV;
	}

	dk->boot_in_con = readl(dk->idma + IDMA_IN_CON);
	dk->boot_src_size = readl(dk->idma + IDMA_SRC_SIZE);
	dk->boot_base = readl(dk->idma + IDMA_IN_BASE_ADDR_Y8);
	if (readl(dk->decon + DECON_BLENDER_BG_IMAGE_SIZE_0) != R8S_SIZE_REG ||
	    readl(dk->idma + IDMA_IMG_SIZE) != R8S_SIZE_REG || !dk->boot_base ||
	    FIELD_GET(IDMA_IMG_FORMAT, dk->boot_in_con) != IDMA_FMT_ARGB8888) {
		dev_err(dev, "unexpected boot scanout (IN_CON 0x%08x, base 0x%08x)\n",
			dk->boot_in_con, dk->boot_base);
		return -ENODEV;
	}

	dev_info(dev, "taking over window %u, IDMA0 at 0x%08x\n", dk->win, dk->boot_base);
	return 0;
}

/* DSI host on S-Boot's link: short and long writes through the command FIFO */

/* IDLE_STATUS: no frame on the link. A FRAME_START not handled yet counts as one. */
static bool decon_frame_running(struct decon_kms *dk)
{
	return !(readl(dk->decon + DECON_GLOBAL_CONTROL) & DECON_GC_IDLE_STATUS) ||
	       (readl(dk->decon + DECON_INTERRUPT_PENDING) & DECON_INT_FRAME_START);
}

/**
 * exynos990_decon_hold_frames - keep frames off the link around a command sequence
 * @host: the panel's DSI host
 * @hold: true before the sequence, false after it
 *
 * Commands go out through the command FIFO as they come, and a frame (2Ch/3Ch) can reach the panel between two of
 * them. While held, the TE trigger stays masked, and holding waits for a frame already under way to end; letting go
 * starts the frames that were wanted meanwhile.
 *
 * Return: when letting go, how many frame interrupts came while held, which should be none: any is a frame that
 * reached the link inside the sequence, and the caller may send it again. 0 when holding.
 */
unsigned int exynos990_decon_hold_frames(struct mipi_dsi_host *host, bool hold)
{
	struct decon_kms *dk = container_of(host, struct decon_kms, dsi_host);
	unsigned long flags;
	unsigned int frames;
	bool running;

	spin_lock_irqsave(&dk->trig_lock, flags);
	dk->trig_held = hold;
	frames = dk->held_frames;
	dk->held_frames = 0;
	if (hold && !dk->trig_masked) {
		rmw(dk->decon, DECON_HW_SW_TRIG_CONTROL, DECON_HW_TRIG_MASK_DECON, DECON_HW_TRIG_MASK_DECON);
		dk->trig_masked = true;
	} else if (!hold && dk->trig_masked && (dk->trig_frames || dk->trig_vblank || !READ_ONCE(psr))) {
		rmw(dk->decon, DECON_HW_SW_TRIG_CONTROL, DECON_HW_TRIG_MASK_DECON, 0);
		dk->trig_masked = false;
	}
	spin_unlock_irqrestore(&dk->trig_lock, flags);

	if (!hold) {
		decon_check_errors(dk, "in a held sequence");
		return frames;
	}

	decon_check_errors(dk, "before a held sequence");
	if (read_poll_timeout(decon_frame_running, running, !running, 100, 50000, false, dk))
		dev_warn_ratelimited(dk->drm.dev, "a frame did not end in 50 ms\n");
	/* from here on: the frame waited for is not one that got in, a frame still running is */
	spin_lock_irqsave(&dk->trig_lock, flags);
	dk->held_frames = running;
	spin_unlock_irqrestore(&dk->trig_lock, flags);
	return 0;
}
EXPORT_SYMBOL_GPL(exynos990_decon_hold_frames);

static int dsim_wait_clear(struct decon_kms *dk, u32 bits)
{
	u32 val;

	return readl_poll_timeout(dk->dsim + DSIM_FIFOCTRL, val, !(val & bits), 10, 100000);
}

static ssize_t decon_dsi_transfer(struct mipi_dsi_host *host, const struct mipi_dsi_msg *msg)
{
	struct decon_kms *dk = container_of(host, struct decon_kms, dsi_host);
	struct mipi_dsi_packet pkt;
	u32 val;
	size_t i;
	int ret;

	if (msg->rx_len)
		return -EOPNOTSUPP;	/* no BTA/RX path yet */

	ret = mipi_dsi_create_packet(&pkt, msg);
	if (ret)
		return ret;

	guard(mutex)(&dk->dsi_lock);

	ret = dsim_wait_clear(dk, DSIM_FULL_PH_SFR);
	for (i = 0; !ret && i < pkt.payload_length; i += 4) {
		size_t n = min_t(size_t, 4, pkt.payload_length - i);

		val = 0;
		memcpy(&val, pkt.payload + i, n);
		ret = dsim_wait_clear(dk, DSIM_FULL_PL_SFR);
		if (!ret)
			writel(le32_to_cpu((__force __le32)val), dk->dsim + DSIM_PAYLOAD);
	}
	if (!ret) {
		writel(pkt.header[0] | pkt.header[1] << 8 | pkt.header[2] << 16, dk->dsim + DSIM_PKTHDR);
		ret = readl_poll_timeout(dk->dsim + DSIM_FIFOCTRL, val, val & DSIM_EMPTY_PH_SFR, 10, 100000);
	}
	if (ret) {
		dev_warn_ratelimited(dk->drm.dev, "DSIM FIFO stuck (FIFOCTRL 0x%08x)\n",
				     readl(dk->dsim + DSIM_FIFOCTRL));
		if (!dk->stuck_dumped) {
			struct drm_printer p = drm_info_printer(dk->drm.dev);

			dk->stuck_dumped = true;
			decon_print_regs(dk, &p, "stuck: ");
		}
		return ret;
	}

	return msg->tx_len;
}

static int decon_drm_init(struct decon_kms *dk, struct drm_bridge *bridge)
{
	struct drm_device *drm = &dk->drm;
	struct device *dev = drm->dev;
	struct drm_connector *connector = NULL;
	int ret;

	ret = drmm_mode_config_init(drm);
	if (ret)
		return ret;
	drm->mode_config.min_width = R8S_WIDTH;
	drm->mode_config.max_width = 4096;
	drm->mode_config.min_height = R8S_HEIGHT;
	drm->mode_config.max_height = 4096;
	drm->mode_config.preferred_depth = 24;
	drm->mode_config.funcs = &decon_mode_config_funcs;

	if (!bridge) {
		connector = &dk->connector;
		ret = drm_connector_init(drm, connector, &decon_connector_funcs, DRM_MODE_CONNECTOR_DSI);
		if (ret)
			return ret;
		drm_connector_helper_add(connector, &decon_connector_helper_funcs);
		connector->display_info.width_mm = r8s_mode.width_mm;
		connector->display_info.height_mm = r8s_mode.height_mm;
	}

	ret = drm_simple_display_pipe_init(drm, &dk->pipe, &decon_pipe_funcs, decon_formats,
					   ARRAY_SIZE(decon_formats), NULL, connector);
	if (ret)
		return ret;
	drm_plane_enable_fb_damage_clips(&dk->pipe.plane);

	/* the panel bridge creates the DSI connector */
	if (bridge) {
		ret = drm_simple_display_pipe_attach_bridge(&dk->pipe, bridge);
		if (ret)
			return ret;
	}

	ret = drm_vblank_init(drm, drm->mode_config.num_crtc);
	if (ret)
		return ret;

	ret = devm_request_irq(dev, dk->irq, decon_irq, 0, dev_name(dev), dk);
	if (ret)
		return ret;

	drm_mode_config_reset(drm);
	drm_debugfs_add_file(drm, "decon_regs", decon_regs_show, NULL);

	/* evict simpledrm from S-Boot's framebuffer */
	ret = aperture_remove_conflicting_devices(dk->boot_base, R8S_FB_SIZE, decon_driver.name);
	if (ret)
		return ret;

	ret = drm_dev_register(drm, 0);
	if (ret)
		return ret;
	dk->registered = true;

	drm_client_setup(drm, NULL);
	return 0;
}

static int decon_dsi_attach(struct mipi_dsi_host *host, struct mipi_dsi_device *dsi)
{
	struct decon_kms *dk = container_of(host, struct decon_kms, dsi_host);
	struct device *dev = dk->drm.dev;
	struct drm_bridge *bridge;
	struct drm_panel *panel;
	int ret;

	if (dk->registered)
		return -EBUSY;	/* one panel, and no re-init after a detach */

	panel = of_drm_find_panel(dsi->dev.of_node);
	if (IS_ERR(panel))
		return PTR_ERR(panel);

	bridge = devm_drm_panel_bridge_add_typed(dev, panel, DRM_MODE_CONNECTOR_DSI);
	if (IS_ERR(bridge))
		return PTR_ERR(bridge);

	ret = decon_drm_init(dk, bridge);
	if (ret)
		dev_err(dev, "DRM init failed: %d\n", ret);
	return ret;
}

static int decon_dsi_detach(struct mipi_dsi_host *host, struct mipi_dsi_device *dsi)
{
	struct decon_kms *dk = container_of(host, struct decon_kms, dsi_host);

	if (dk->registered) {
		drm_dev_unplug(&dk->drm);
		drm_atomic_helper_shutdown(&dk->drm);
	}
	return 0;
}

static const struct mipi_dsi_host_ops decon_dsi_host_ops = {
	.attach = decon_dsi_attach,
	.detach = decon_dsi_detach,
	.transfer = decon_dsi_transfer,
};

static int decon_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct decon_kms *dk;
	int ret;

	dk = devm_drm_dev_alloc(dev, &decon_driver, struct decon_kms, drm);
	if (IS_ERR(dk))
		return PTR_ERR(dk);
	platform_set_drvdata(pdev, dk);
	spin_lock_init(&dk->trig_lock);

	dk->decon = devm_platform_ioremap_resource_byname(pdev, "decon");
	if (IS_ERR(dk->decon))
		return PTR_ERR(dk->decon);
	dk->idma = devm_platform_ioremap_resource_byname(pdev, "idma0");
	if (IS_ERR(dk->idma))
		return PTR_ERR(dk->idma);
	dk->dsim = devm_platform_ioremap_resource_byname(pdev, "dsim");
	if (IS_ERR(dk->dsim))
		return PTR_ERR(dk->dsim);

	ret = decon_check_handover(dk, dev);
	if (ret)
		return ret;

	dk->boot_fb = devm_memremap(dev, dk->boot_base, R8S_FB_SIZE, MEMREMAP_WC);
	if (IS_ERR(dk->boot_fb))
		return PTR_ERR(dk->boot_fb);

	/* no IOMMU in front of the IDMA, and its base register is 32 bits wide */
	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;

	dk->irq = platform_get_irq(pdev, 0);
	if (dk->irq < 0)
		return dk->irq;

	ret = devm_mutex_init(dev, &dk->dsi_lock);
	if (ret)
		return ret;

	dk->dsi_host.dev = dev;
	dk->dsi_host.ops = &decon_dsi_host_ops;
	ret = mipi_dsi_host_register(&dk->dsi_host);
	if (ret)
		return ret;

	/* a panel's probe attaches to this host, which registers the DRM device; without one, register now */
	if (!of_get_available_child_count(dev->of_node)) {
		ret = decon_drm_init(dk, NULL);
		if (ret) {
			mipi_dsi_host_unregister(&dk->dsi_host);
			return dev_err_probe(dev, ret, "DRM init failed\n");
		}
	}
	return 0;
}

static void decon_remove(struct platform_device *pdev)
{
	struct decon_kms *dk = platform_get_drvdata(pdev);

	if (dk->registered && !of_get_available_child_count(pdev->dev.of_node)) {
		drm_dev_unplug(&dk->drm);
		drm_atomic_helper_shutdown(&dk->drm);
	}
	mipi_dsi_host_unregister(&dk->dsi_host);
}

static void decon_shutdown(struct platform_device *pdev)
{
	struct decon_kms *dk = platform_get_drvdata(pdev);

	if (dk->registered)
		drm_atomic_helper_shutdown(&dk->drm);
}

static const struct of_device_id decon_of_match[] = {
	{ .compatible = "samsung,exynos990-decon-handover" },
	{ }
};
MODULE_DEVICE_TABLE(of, decon_of_match);

static struct platform_driver decon_platform_driver = {
	.driver = {
		.name = "exynos990-decon-kms",
		.of_match_table = decon_of_match,
	},
	.probe = decon_probe,
	.remove = decon_remove,
	.shutdown = decon_shutdown,
};
module_platform_driver(decon_platform_driver);

MODULE_DESCRIPTION("Exynos 990 DECON0 handover KMS and DSI command host");
MODULE_LICENSE("GPL");
