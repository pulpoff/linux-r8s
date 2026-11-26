// SPDX-License-Identifier: GPL-2.0
/*
 * Exynos 990 DisplayPort transmitter (Galaxy S20 FE, USB-C DP alt mode),
 * following the stock kernel's cal_9830/displayport_reg.c, decon_reg.c,
 * dpp_reg.c and phy-exynos-usbdp-gen2-v2.c.
 *
 * A DRM device of its own ("exynos", like the panel's, so Mesa's kmsro renders
 * for it): DECON1 reads the framebuffer through DMA channel 1 and DPP1 into
 * window 3 and sends it to DP0, whose own timing generator paces the stream.
 * The connector is polled: a sink that answers on AUX is connected. The
 * debugfs commands below stay for bring-up and are refused while the DRM pipe
 * runs.
 *
 * The cable has to be in DisplayPort alt mode: the MAX77705 USB-C driver
 * (max77705_usbc) enters it when a DP dock or hub attaches.
 *
 *   echo phy  > /sys/kernel/debug/r8s-dp/ctl   PMU isolation off, combo PHY PMA init
 *   echo link > ctl                            DP link reset, DP PHY setup, function enables
 *   echo aux on|off > ctl                      the board's SBU/AUX analogue switch
 *   echo dpcd ADDR LEN > ctl                   native AUX read (LEN <= 16)
 *   echo edid > ctl                            EDID block 0 over I2C-over-AUX
 *   echo train RATE LANES > ctl                link training (RATE 0x06/0x0a/0x14)
 *   echo bist on|off > ctl                     1080p60 colour bars from the link itself
 *   echo linkst > ctl                          the sink's DPCD link status
 *   echo decon on [RRGGBB] | off | st > ctl    DECON1 -> DP0: a solid-colour window (3) at 1080p60
 *   echo decon img > ctl                       window 3 from memory: DMA channel 1 + DPP1, a test pattern
 *   echo replug > ctl                          the connector goes disconnected and back, with a hotplug
 *                                              event each way (mutter only looks at a changed state)
 *   echo dump > ctl                            link and PHY registers
 *   cat ctl                                    what the last command found
 *
 * Every register it touches is clocked by the clocks of its DT node, which it
 * holds enabled while bound.
 */
#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/gpio/consumer.h>
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/interrupt.h>

#include <drm/drm_atomic_helper.h>
#include <drm/drm_connector.h>
#include <drm/drm_drv.h>
#include <drm/drm_edid.h>
#include <drm/drm_fb_dma_helper.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_framebuffer.h>
#include <drm/drm_gem_atomic_helper.h>
#include <drm/drm_gem_dma_helper.h>
#include <drm/drm_gem_framebuffer_helper.h>
#include <drm/drm_modes.h>
#include <drm/drm_modeset_helper_vtables.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_simple_kms_helper.h>
#include <drm/drm_vblank.h>

enum { CTRL, PMA, PCS, DELAY = 0xff };
struct pma_op { u8 base; u16 off; u32 mask; u32 val; };
#define PMA_W(b, o, m, v)	{ b, o, m, v }
#define PMA_DELAY(us)		{ DELAY, 0, 0, us }
#include "exynos990-dp-pma.h"

/* DP link, stock cal_9830/regs-displayport.h */
#define SYSTEM_SW_RESET_CONTROL		0x0004
#define SYSTEM_SW_FUNCTION_ENABLE	0x0014
#define SYSTEM_COMMON_FUNCTION_ENABLE	0x0018
#define  PCS_FUNC_EN			BIT(1)
#define  AUX_FUNC_EN			BIT(0)
#define SYSTEM_SST1_FUNCTION_ENABLE	0x001c
#define SYSTEM_SST2_FUNCTION_ENABLE	0x0020
#define SYSTEM_PLL_LOCK_CONTROL		0x002c
#define  PLL_LOCK_STATUS		BIT(4)
#define AUX_CONTROL			0x1000
#define  AUX_REPLY_TIMER_MODE		GENMASK(13, 12)
#define AUX_TRANSACTION_START		0x1004
#define AUX_BUFFER_CLEAR		0x1008
#define AUX_ADDR_ONLY_COMMAND		0x100c
#define AUX_REQUEST_CONTROL		0x1010
#define  REQ_COMM			GENMASK(31, 28)
#define  REQ_ADDR			GENMASK(27, 8)
#define  REQ_LENGTH			GENMASK(5, 0)
#define AUX_COMMAND_CONTROL		0x1014
#define  DEFER_CTRL_EN			BIT(8)
#define AUX_MONITOR_1			0x1018
#define  AUX_CMD_STATUS			GENMASK(11, 8)
#define AUX_MONITOR_2			0x101c
#define AUX_TX_DATA_SET0		0x1030
#define AUX_RX_DATA_SET0		0x1040
#define SYSTEM_MAIN_LINK_LANE_COUNT	0x0010
#define PCS_CONTROL			0x3000
#define  LINK_TRAINING_PATTERN_SET	GENMASK(5, 4)
#define  SCRAMBLE_BYPASS		BIT(0)
#define PCS_LANE_CONTROL		0x3004
#define PCS_TEST_PATTERN_CONTROL	0x3008
#define MST_STREAM_1_ENABLE		0x2010
#define SST1_MAIN_CONTROL		0x5000
#define  MVID_MODE			BIT(11)
#define  VIDEO_MODE			BIT(6)
#define  ENHANCED_MODE			BIT(5)
#define SST1_MVID_MASTER_MODE		0x5044
#define SST1_NVID_MASTER_MODE		0x5048
#define SST1_MVID_SFR_CONFIGURE		0x504c
#define SST1_NVID_SFR_CONFIGURE		0x5050
#define SST1_AS_INTEGER_FEC_OFF		0x5080
#define SST1_AS_FRACTION_FEC_OFF	0x5084
#define SST1_AS_THRESHOLD_FEC_OFF	0x5088
#define SST1_AS_THRESHOLD_SEL_FEC_OFF	0x508c
#define SST1_AS_INTEGER_FEC_ON		0x5090
#define SST1_AS_FRACTION_FEC_ON		0x5094
#define SST1_AS_THRESHOLD_FEC_ON	0x5098
#define SST1_AS_THRESHOLD_SEL_FEC_ON	0x509c
#define SST1_AS_MODE_CONTROL		0x5104
#define SST1_VIDEO_CONTROL		0x5400
#define  STRM_VALID_FORCE		BIT(9)
#define  STRM_VALID_CTRL		BIT(8)
#define  DYNAMIC_RANGE_MODE		BIT(7)
#define  VIDEO_BPC			GENMASK(6, 4)
#define  COLOR_FORMAT			GENMASK(3, 2)
#define  SYNC_POLARITY			GENMASK(1, 0)
#define SST1_VIDEO_ENABLE		0x5404
#define SST1_VIDEO_MASTER_TIMING_GEN	0x5408
#define SST1_VIDEO_HTOTAL		0x5414
#define SST1_VIDEO_VTOTAL		0x5418
#define SST1_VIDEO_HFP			0x541c
#define SST1_VIDEO_HBP			0x5420
#define SST1_VIDEO_HACTIVE		0x5424
#define SST1_VIDEO_VFP			0x5428
#define SST1_VIDEO_VBP			0x542c
#define SST1_VIDEO_VACTIVE		0x5430
#define SST1_VIDEO_BIST_CONTROL		0x5450
#define  CTS_BIST_EN			BIT(17)
#define  BIST_EN			BIT(3)
#define  BIST_TYPE			GENMASK(1, 0)
#define SST1_VIDEO_DEBUG_FSM_STATE	0x5460
#define SYSTEM_SST1_FUNCTION_ENABLE	0x001c
#define  SST1_LH_PWR_ON_STATUS		BIT(5)
#define  SST1_LH_PWR_ON			BIT(4)

/*
 * DECON0/1/2 (dpu30 cal_9830 regs-decon.h). The three are one IP behind one clock gate and one APB port, so
 * DECON1 is as reachable as DECON0, which the panel driver runs. The window blenders are one pool in DECON0's
 * space; a DECON enables the windows it uses in its own DATA_PATH_CONTROL_0. The panel has window 5 on DMA
 * channel 0, so DP gets window 3 (stock's default for DECON1) and channel 1.
 */
#define DPU_DECON0			0x19050000
#define DPU_DECON1			0x19060000
#define D_GLOBAL_CONTROL		0x0000
#define  D_GC_OPERATION_CMD		BIT(8)
#define  D_GC_IDLE			BIT(5)
#define  D_GC_RUN			BIT(4)
#define  D_GC_EN			BIT(1)
#define  D_GC_EN_F			BIT(0)
#define D_SRAM_SHARE_MAIN		0x0030
#define  D_SRAM3_4			(BIT(12) | BIT(16))	/* stock: 8 KiB for DECON1 */
#define D_INTERRUPT_ENABLE		0x0040
#define  D_INT_FRAME_START_EN		(BIT(12) | BIT(0))
#define D_INTERRUPT_PENDING		0x004c
#define  D_INT_FRAME_START		BIT(12)
#define D_EXTRA_INTERRUPT_PENDING	0x0050
#define D_SHADOW_REG_UPDATE_REQ		0x0060
#define  D_SHADOW_GLOBAL		BIT(31)
#define D_SCALED_SIZE			0x00a0
#define D_CLOCK_CONTROL_0		0x00f0
#define  D_CG_QACTIVE			(0x11111 | BIT(24) | BIT(28))
#define  D_TE_QACTIVE_PLL_ON		BIT(28)
#define D_OUTFIFO_SIZE_0		0x0120
#define D_OUTFIFO_TH_0			0x012c
#define D_OUTFIFO_ORDER			0x0130
#define D_BLENDER_BG_SIZE		0x0200
#define D_DATA_PATH_0			0x0214
#define D_DATA_PATH_1			0x0218
#define D_DATA_PATH_2			0x0230
#define  D_DPATH_NOCOMP_DPIF		0x008
#define D_DP_CONNECTION			0x0270	/* in DECON0: SEL_DP0 bits 2:0, 3 = none */
#define D_FRAME_COUNT			0x02a0
#define D_WIN_CONTROL_0(w)		(0x1000 + (w) * 0x30)
#define D_WIN_CONTROL_1(w)		(0x1004 + (w) * 0x30)
#define D_WIN_START(w)			(0x1008 + (w) * 0x30)
#define D_WIN_END(w)			(0x100c + (w) * 0x30)
#define D_WIN_COLORMAP_0(w)		(0x1010 + (w) * 0x30)
#define D_WIN_COLORMAP_1(w)		(0x1014 + (w) * 0x30)
#define D_WIN_START_TIME(w)		(0x1018 + (w) * 0x30)
#define DP_WIN				3
#define DP_CH				1

/* DPU_DMA channel 1 and DPP1 (dpu30 cal_9830 regs-dpp.h), set up the way S-Boot set channel 0 for the panel */
#define DPU_DMA_CH1			0x19092000
#define DPU_DPP1			0x19042000
#define IDMA_IN_CON			0x0008
#define  IDMA_IN_CON_XRGB8888		0xff400400	/* plane alpha 0xff, as S-Boot's, format 4 */
#define IDMA_SRC_SIZE			0x0010
#define IDMA_SRC_OFFSET			0x0014
#define IDMA_IMG_SIZE			0x0018
#define IDMA_IN_BASE_ADDR_Y8		0x0040
#define IDMA_RECOVERY_CTRL		0x0070
#define IDMA_DEADLOCK_EN		0x0100
#define DPP_IMG_SIZE			0x0018
#define DPP_LINECNT_CON			0x0d00

/* the DP side of the combo PHY's PMA */
#define CMN_REG0008			0x0020
#define  OVRD_AUX_EN			BIT(3)
#define CMN_REG000A			0x0028
#define  ANA_AUX_TX_LVL_CTRL		GENMASK(6, 3)
#define CMN_REG00B8			0x02e0
#define  SSC_EN				BIT(5)
#define  LANE_MUX_SEL_DP		GENMASK(3, 0)
#define CMN_REG00B9			0x02e4
#define  DP_TX_LINK_BW			GENMASK(5, 4)
#define  DP_LANE_EN			GENMASK(3, 0)
#define CMN_REG00BD			0x02f4
#define  DP_RSTN			GENMASK(1, 0)	/* DP_INIT_RSTN, DP_CMN_RSTN */
#define CMN_REG01C0			0x0700		/* LCPLL lock 7, AFC done 6 */

#define PMU_USBDP_PHY_CONTROL		0x0704		/* bit 0: 1 = isolation off */

enum { I2C_WRITE = 4, I2C_READ = 5, DPCD_WRITE = 8, DPCD_READ = 9 };
#define EDID_ADDRESS			0x50

/* stock's phy_default_value for EVT1 silicon */
static const u16 dp_phy_defaults[][2] = {
	{0x0bbc, 0x77}, {0x10b4, 0x77}, {0x1bbc, 0x77}, {0x20b4, 0x77}, {0x10b0, 0x05},
	{0x20b0, 0x05}, {0x0298, 0x18}, {0x0594, 0x08}, {0x0420, 0x77}, {0x0858, 0x63},
	{0x1058, 0x63}, {0x1858, 0x63}, {0x2058, 0x63}, {0x0894, 0x00}, {0x0a1c, 0x00},
	{0x1094, 0x00}, {0x1894, 0x00}, {0x1a1c, 0x00}, {0x2094, 0x00},
};

/* stock's phy_tune_parameters: [swing][pre-emphasis] = {amp, post, pre, idrv, accdrv} */
static const u8 phy_tune[4][4][5] = {
	{ {0x23, 0x10, 0x42, 0x82, 0x00}, {0x27, 0x14, 0x42, 0x82, 0x00},
	  {0x28, 0x17, 0x43, 0x82, 0x00}, {0x2b, 0x1b, 0x43, 0x83, 0x30} },
	{ {0x27, 0x10, 0x42, 0x82, 0x00}, {0x2b, 0x15, 0x42, 0x83, 0x30},
	  {0x2b, 0x19, 0x43, 0x83, 0x30}, {0x2b, 0x19, 0x43, 0x83, 0x30} },
	{ {0x28, 0x10, 0x43, 0x83, 0x38}, {0x2b, 0x16, 0x43, 0x83, 0x38},
	  {0x2b, 0x16, 0x43, 0x83, 0x38}, {0x2b, 0x16, 0x43, 0x83, 0x38} },
	{ {0x2b, 0x10, 0x43, 0x83, 0x30}, {0x2b, 0x10, 0x43, 0x83, 0x30},
	  {0x2b, 0x10, 0x43, 0x83, 0x30}, {0x2b, 0x10, 0x43, 0x83, 0x30} },
};
static const u16 tune_base[4] = { 0x0810, 0x1010, 0x1810, 0x2010 };	/* TRSV_REG0204.. */

#define OUTSZ	8192

struct r8s_dp {
	struct drm_device drm;
	struct drm_simple_display_pipe pipe;
	struct drm_connector conn;
	int irq;
	bool connected;			/* the sink answered on AUX at the last detect (lock) */
	bool streaming;			/* DECON1 runs for the DRM pipe (lock) */
	bool unplugged;			/* replug: the connector reads disconnected for a moment */
	bool hw_up;			/* dp_hw_up() ran, for the plug orientation in hw_flip (lock) */
	int hw_flip;
	u8 caps[16];			/* the sink's DPCD receiver capabilities */
	const struct drm_edid *edid;
	struct device *dev;
	void __iomem *link, *base[3];		/* CTRL, PMA, PCS */
	struct clk_bulk_data *clks;
	int nclks;
	struct regmap *pmu;
	void __iomem *d0, *d1;			/* DECON0, DECON1 */
	void __iomem *dma1, *dpp1;
	u32 *img;
	dma_addr_t img_dma;
	struct gpio_desc *aux_oe, *sbu_sel, *orient;
	int flip;				/* gpm26-0: the MAX77705's orientation */
	int lanes;				/* 2: pin D, 4: pin C */
	struct mutex lock;
	struct dentry *dbg;
	char *out;
	size_t len;
};

static __printf(2, 3) void outf(struct r8s_dp *dp, const char *fmt, ...)
{
	va_list ap;

	va_start(ap, fmt);
	dp->len += vscnprintf(dp->out + dp->len, OUTSZ - dp->len, fmt, ap);
	va_end(ap);
}

static void rmw(void __iomem *r, u32 mask, u32 val)
{
	writel((readl(r) & ~mask) | (val & mask), r);
}

/*
 * The PMA and PCS answer only once USBCON 0x48 bit 0 (PMA_APB_SW_RST) is clear,
 * which it is not after boot: a read before that hangs the bus (it did, 11-18).
 */
static bool pma_ok(struct r8s_dp *dp)
{
	unsigned int pmu = 0;

	regmap_read(dp->pmu, PMU_USBDP_PHY_CONTROL, &pmu);
	return (pmu & BIT(0)) && !(readl(dp->base[CTRL] + 0x48) & BIT(0));
}

/* stock exynos_usbdrd_pipe3_init: PMU isolation off, then the combo PHY's PMA */
static void dp_phy_on(struct r8s_dp *dp)
{
	const struct pma_op *seq = dp->flip ? pma_seq_port1 : pma_seq_port0;
	size_t i, n = dp->flip ? ARRAY_SIZE(pma_seq_port1) : ARRAY_SIZE(pma_seq_port0);
	u32 v = 0;
	int cnt;

	dev_info(dp->dev, "phy: PMU isolation off, PMA init (port %d)\n", dp->flip);
	regmap_update_bits(dp->pmu, PMU_USBDP_PHY_CONTROL, BIT(0), BIT(0));
	udelay(10);
	for (i = 0; i < n; i++) {
		if (seq[i].base == DELAY)
			udelay(seq[i].val);
		else
			rmw(dp->base[seq[i].base] + seq[i].off, seq[i].mask, seq[i].val);
	}
	for (cnt = 1000; cnt; cnt--) {
		v = readl(dp->base[PMA] + CMN_REG01C0);
		if (v & (BIT(7) | BIT(6)))
			break;
		udelay(1);
	}
	outf(dp, "phy: port %d, %zu PMA writes, USBCON 0x48 %#x, LCPLL %#x%s\n", dp->flip, n,
	     readl(dp->base[CTRL] + 0x48), v, cnt ? "" : " (no lock)");
}

/* stock displayport_reg_sw_reset + displayport_reg_init */
static void dp_link_on(struct r8s_dp *dp)
{
	void __iomem *pma = dp->base[PMA], *link = dp->link;
	/* stock dp_sw_sel = !usb_dir: DP on lanes 3/2 when it is set */
	int sw_sel = !dp->flip;
	u32 lanes, state;
	int cnt, i;

	if (!pma_ok(dp)) {
		outf(dp, "link: run phy first (the PMA is in reset or isolated)\n");
		return;
	}
	dev_info(dp->dev, "link: sw reset\n");
	writel(readl(link + SYSTEM_SW_RESET_CONTROL) | BIT(0), link + SYSTEM_SW_RESET_CONTROL);
	for (cnt = 10; cnt && (readl(link + SYSTEM_SW_RESET_CONTROL) & BIT(0)); cnt--)
		udelay(1);
	outf(dp, "link: sw reset %s\n", cnt ? "done" : "TIMEOUT");

	rmw(pma + CMN_REG00BD, DP_RSTN, 0);
	for (i = 0; i < ARRAY_SIZE(dp_phy_defaults); i++)
		writel(dp_phy_defaults[i][1], pma + dp_phy_defaults[i][0]);
	rmw(pma + CMN_REG0008, OVRD_AUX_EN, 0);
	rmw(pma + CMN_REG000A, ANA_AUX_TX_LVL_CTRL, FIELD_PREP(ANA_AUX_TX_LVL_CTRL, 0xc));
	if (dp->lanes == 4)
		lanes = 0xf;
	else
		lanes = sw_sel ? 0xc : 0x3;
	rmw(pma + CMN_REG00B8, LANE_MUX_SEL_DP, lanes);
	rmw(pma + CMN_REG00B9, DP_LANE_EN, lanes);
	rmw(pma + CMN_REG00B8, SSC_EN, 0);
	rmw(pma + CMN_REG00BD, DP_RSTN, DP_RSTN);

	for (cnt = 165; cnt; cnt--) {
		state = readl(link + SYSTEM_PLL_LOCK_CONTROL);
		if (state & PLL_LOCK_STATUS)
			break;
		udelay(1);
	}
	outf(dp, "link: DP lanes %#x, PLL_LOCK_CONTROL %#x%s\n", lanes, state,
	     cnt ? "" : " (no lock)");

	rmw(link + SYSTEM_COMMON_FUNCTION_ENABLE, PCS_FUNC_EN | AUX_FUNC_EN,
	    PCS_FUNC_EN | AUX_FUNC_EN);
	rmw(link + SYSTEM_SST1_FUNCTION_ENABLE, BIT(0), BIT(0));
	rmw(link + SYSTEM_SST2_FUNCTION_ENABLE, BIT(0), BIT(0));
	rmw(link + SYSTEM_SW_FUNCTION_ENABLE, BIT(0), BIT(0));
	/* stock displayport_reg_set_lane_map_config, pins C/D/E/F */
	rmw(link + PCS_LANE_CONTROL, 0x3333, sw_sel ? 0x1023 : 0x2310);
	outf(dp, "link: functions on, AUX_CONTROL %#x, PCS_LANE_CONTROL %#x\n",
	     readl(link + AUX_CONTROL), readl(link + PCS_LANE_CONTROL));
}

/* stock displayport_reg_set_aux_ch_operation_enable */
static int aux_go(struct r8s_dp *dp)
{
	void __iomem *link = dp->link;
	u32 m1, m2;
	int cnt;

	writel(1, link + AUX_TRANSACTION_START);
	for (cnt = 5000; cnt && (readl(link + AUX_TRANSACTION_START) & 1); cnt--)
		udelay(10);
	if (!cnt)
		return -ETIME;
	m1 = readl(link + AUX_MONITOR_1);
	m2 = readl(link + AUX_MONITOR_2);
	if ((m1 & AUX_CMD_STATUS) || m2) {
		outf(dp, "  aux: MONITOR_1 %#x MONITOR_2 %#x\n", m1, m2);
		usleep_range(400, 500);
		return -EIO;
	}
	return 0;
}

static void aux_setup(struct r8s_dp *dp, int cmd, u32 addr, u32 len)
{
	void __iomem *link = dp->link;

	rmw(link + AUX_REQUEST_CONTROL, REQ_COMM | REQ_ADDR | REQ_LENGTH,
	    FIELD_PREP(REQ_COMM, cmd) | FIELD_PREP(REQ_ADDR, addr) |
	    FIELD_PREP(REQ_LENGTH, len ? len - 1 : 0));
	writel(1, link + AUX_BUFFER_CLEAR);
	rmw(link + AUX_COMMAND_CONTROL, DEFER_CTRL_EN, DEFER_CTRL_EN);
	rmw(link + AUX_CONTROL, AUX_REPLY_TIMER_MODE, FIELD_PREP(AUX_REPLY_TIMER_MODE, 3));
}

static void aux_rx(struct r8s_dp *dp, u8 *buf, u32 len)
{
	u32 i;

	for (i = 0; i < len; i++)
		buf[i] = readl(dp->link + AUX_RX_DATA_SET0 + (i / 4) * 4) >> ((i % 4) * 8);
}

static int dpcd_read(struct r8s_dp *dp, u32 addr, u32 len, u8 *buf)
{
	int ret = -EIO, retry;

	for (retry = 3; retry && ret; retry--) {
		aux_setup(dp, DPCD_READ, addr, len);
		ret = aux_go(dp);
	}
	if (!ret)
		aux_rx(dp, buf, len);
	return ret;
}

/* an address-only I2C transaction: the MOT start / the stop */
static int i2c_addr_only(struct r8s_dp *dp)
{
	int ret;

	rmw(dp->link + AUX_REQUEST_CONTROL, REQ_COMM | REQ_ADDR,
	    FIELD_PREP(REQ_COMM, I2C_WRITE) | FIELD_PREP(REQ_ADDR, EDID_ADDRESS));
	writel(1, dp->link + AUX_ADDR_ONLY_COMMAND);
	ret = aux_go(dp);
	writel(0, dp->link + AUX_ADDR_ONLY_COMMAND);
	return ret;
}

/* stock displayport_reg_edid_read, block 0 or 1 */
static int edid_read(struct r8s_dp *dp, int block, u8 *buf)
{
	u8 offset = (block & 1) * 128;
	int ret = -EIO, retry, i;

	for (retry = 3; retry && ret; retry--) {
		i2c_addr_only(dp);
		aux_setup(dp, I2C_WRITE, EDID_ADDRESS, 1);
		writel(offset, dp->link + AUX_TX_DATA_SET0);
		ret = aux_go(dp);
		for (i = 0; !ret && i < 128; i += 16) {
			aux_setup(dp, I2C_READ, EDID_ADDRESS, 16);
			ret = aux_go(dp);
			if (!ret)
				aux_rx(dp, buf + i, 16);
		}
		if (!ret)
			ret = i2c_addr_only(dp);
	}
	return ret;
}

static int dpcd_write(struct r8s_dp *dp, u32 addr, u32 len, const u8 *buf)
{
	int ret = -EIO, retry;
	u32 i;

	for (retry = 3; retry && ret; retry--) {
		aux_setup(dp, DPCD_WRITE, addr, len);
		for (i = 0; i < len; i++)
			rmw(dp->link + AUX_TX_DATA_SET0 + (i / 4) * 4, 0xffu << ((i % 4) * 8),
			    (u32)buf[i] << ((i % 4) * 8));
		ret = aux_go(dp);
	}
	return ret;
}

static void set_pattern(struct r8s_dp *dp, int tps)
{
	rmw(dp->link + PCS_TEST_PATTERN_CONTROL, 7, 0);
	rmw(dp->link + PCS_CONTROL, LINK_TRAINING_PATTERN_SET,
	    FIELD_PREP(LINK_TRAINING_PATTERN_SET, tps));
	rmw(dp->link + PCS_CONTROL, SCRAMBLE_BYPASS, tps ? SCRAMBLE_BYPASS : 0);
}

/* stock displayport_reg_set_phy_voltage_and_pre_emphasis, pins C/D */
static void set_drive(struct r8s_dp *dp, const u8 *sw, const u8 *pe)
{
	static const u8 map4[2][4] = { { 0, 1, 3, 2 }, { 2, 3, 1, 0 } };	/* phy lane -> DP lane */
	int sw_sel = !dp->flip, lane, i;

	for (lane = 0; lane < 4; lane++) {
		int l;

		if (dp->lanes == 4)
			l = map4[sw_sel][lane];
		else if (sw_sel)
			l = lane == 2 ? 1 : lane == 3 ? 0 : -1;
		else
			l = lane < 2 ? lane : -1;
		if (l < 0)
			continue;
		for (i = 0; i < 5; i++)
			writel(phy_tune[sw[l]][pe[l]][i], dp->base[PMA] + tune_base[lane] + i * 4);
	}
}

static void train_wait(u8 interval)
{
	if (interval)
		msleep(interval * 4);
	else
		usleep_range(100, 200);
}

/* stock displayport_full_link_training, without the rate fallback */
static int dp_train(struct r8s_dp *dp, u8 rate, u8 lanes)
{
	void __iomem *pma = dp->base[PMA], *link = dp->link;
	u8 caps[16], v[4], st[6], sw[4] = {}, pe[4] = {}, want = lanes == 4 ? 0xf : lanes == 2 ? 3 : 1;
	u8 cr = 0, eq = 0, lock = 0, align = 0;
	u32 bw = rate == 0x1e ? 3 : rate == 0x14 ? 2 : rate == 0x0a ? 1 : 0, state = 0;
	bool tps3, enh;
	int i, tries, cnt;

	if (dpcd_read(dp, 0, 16, caps)) {
		outf(dp, "train: no DPCD\n");
		return -EIO;
	}
	tps3 = caps[2] & 0x40;
	enh = caps[2] & 0x80;
	dp->lanes = lanes;
	v[0] = 1;			/* SET_POWER D0 */
	dpcd_write(dp, 0x600, 1, v);
	usleep_range(1000, 2000);

	rmw(pma + CMN_REG00BD, DP_RSTN, 0);
	for (i = 0; i < ARRAY_SIZE(dp_phy_defaults); i++)
		writel(dp_phy_defaults[i][1], pma + dp_phy_defaults[i][0]);
	rmw(pma + CMN_REG0008, OVRD_AUX_EN, 0);
	rmw(pma + CMN_REG000A, ANA_AUX_TX_LVL_CTRL, FIELD_PREP(ANA_AUX_TX_LVL_CTRL, 0xc));
	rmw(pma + CMN_REG00B9, DP_TX_LINK_BW, FIELD_PREP(DP_TX_LINK_BW, bw));
	v[0] = lanes == 4 ? 0xf : !dp->flip ? 0xc : 0x3;
	rmw(pma + CMN_REG00B8, LANE_MUX_SEL_DP, v[0]);
	rmw(pma + CMN_REG00B9, DP_LANE_EN, v[0]);
	writel(lanes, link + SYSTEM_MAIN_LINK_LANE_COUNT);
	rmw(link + SST1_MAIN_CONTROL, ENHANCED_MODE, enh ? ENHANCED_MODE : 0);
	usleep_range(60, 100);
	rmw(pma + CMN_REG00BD, DP_RSTN, DP_RSTN);

	v[0] = rate;
	v[1] = lanes | (enh ? 0x80 : 0);
	dpcd_write(dp, 0x100, 2, v);
	for (cnt = 165; cnt; cnt--) {
		state = readl(link + SYSTEM_PLL_LOCK_CONTROL);
		if (state & PLL_LOCK_STATUS)
			break;
		udelay(1);
	}
	outf(dp, "train: rate %#x lanes %d enh %d tps3 %d, PLL %#x, interval %d\n",
	     rate, lanes, enh, tps3, state, caps[14] & 0x7f);

	set_pattern(dp, 1);
	v[0] = 0x21;
	dpcd_write(dp, 0x102, 1, v);
	for (tries = 0; tries < 10; tries++) {
		set_drive(dp, sw, pe);
		for (i = 0; i < 4; i++)
			v[i] = pe[i] << 3 | sw[i] | (sw[i] >= 3 ? 4 : 0) | (pe[i] >= 3 ? 0x20 : 0);
		dpcd_write(dp, 0x103, 4, v);
		train_wait(caps[14] & 0x7f);
		dpcd_read(dp, 0x202, 6, st);
		cr = (st[0] & 1) | (st[0] >> 3 & 2) | (st[1] << 2 & 4) | (st[1] >> 1 & 8);
		outf(dp, "  CR %d: drive %d/%d %d/%d -> status %*ph\n", tries, sw[0], pe[0],
		     sw[1], pe[1], 6, st);
		if ((cr & want) == want)
			break;
		for (i = 0; i < 4; i++) {
			u8 a = st[4 + i / 2] >> ((i & 1) * 4);

			sw[i] = a & 3;
			pe[i] = (a >> 2) & 3;
		}
	}
	if ((cr & want) != want) {
		outf(dp, "train: clock recovery FAILED\n");
		return -EIO;
	}

	set_pattern(dp, tps3 ? 3 : 2);
	v[0] = tps3 ? 0x23 : 0x22;
	dpcd_write(dp, 0x102, 1, v);
	for (tries = 0; tries < 6; tries++) {
		set_drive(dp, sw, pe);
		for (i = 0; i < 4; i++)
			v[i] = pe[i] << 3 | sw[i] | (sw[i] >= 3 ? 4 : 0) | (pe[i] >= 3 ? 0x20 : 0);
		dpcd_write(dp, 0x103, 4, v);
		train_wait(caps[14] & 0x7f);
		dpcd_read(dp, 0x202, 6, st);
		cr = (st[0] & 1) | (st[0] >> 3 & 2) | (st[1] << 2 & 4) | (st[1] >> 1 & 8);
		eq = (st[0] >> 1 & 1) | (st[0] >> 4 & 2) | (st[1] << 1 & 4) | (st[1] >> 2 & 8);
		lock = (st[0] >> 2 & 1) | (st[0] >> 5 & 2) | (st[1] & 4) | (st[1] >> 3 & 8);
		align = st[2] & 1;
		outf(dp, "  EQ %d: drive %d/%d %d/%d -> status %*ph\n", tries, sw[0], pe[0],
		     sw[1], pe[1], 6, st);
		if ((cr & want) != want)
			break;
		if ((eq & want) == want && (lock & want) == want && align) {
			set_pattern(dp, 0);
			v[0] = 0;
			dpcd_write(dp, 0x102, 1, v);
			outf(dp, "train: DONE at %#x x%d, lane set %*ph\n", rate, lanes, 4, v);
			return 0;
		}
		for (i = 0; i < 4; i++) {
			u8 a = st[4 + i / 2] >> ((i & 1) * 4);

			sw[i] = a & 3;
			pe[i] = (a >> 2) & 3;
		}
	}
	set_pattern(dp, 0);
	v[0] = 0;
	dpcd_write(dp, 0x102, 1, v);
	outf(dp, "train: equalization FAILED (cr %x eq %x lock %x align %d)\n", cr, eq, lock, align);
	return -EIO;
}

/* CEA-861 1080p60 (VIC 16), for the debugfs commands */
static const struct drm_display_mode cea_1080p60 = {
	DRM_MODE("1920x1080", DRM_MODE_TYPE_DRIVER, 148500, 1920, 2008, 2052, 2200, 0,
		 1080, 1084, 1089, 1125, 0, DRM_MODE_FLAG_PHSYNC | DRM_MODE_FLAG_PVSYNC),
};

/* the trained link's symbol clock */
static u64 link_khz(struct r8s_dp *dp)
{
	u32 bw = FIELD_GET(DP_TX_LINK_BW, readl(dp->base[PMA] + CMN_REG00B9));

	return bw == 3 ? 810000 : bw == 2 ? 540000 : bw == 1 ? 270000 : 162000;
}

/*
 * stock displayport_reg_set_video_configuration: the link's own timing generator paces the stream, RGB, 8 bpc,
 * full range (what mutter draws)
 */
static int dp_video_config(struct r8s_dp *dp, const struct drm_display_mode *m)
{
	void __iomem *link = dp->link;
	const u64 clk = m->clock, bpp = 24, e10 = 10000000000ULL;	/* kHz */
	u64 ls = link_khz(dp);
	u64 lanes = readl(link + SYSTEM_MAIN_LINK_LANE_COUNT), tu_off, tu_on;
	u32 i_off, f_off, i_on, f_on, pol = 0;

	if (!lanes) {
		outf(dp, "train first\n");
		return -EINVAL;
	}
	if (m->flags & DRM_MODE_FLAG_NVSYNC)
		pol |= BIT(1);
	if (m->flags & DRM_MODE_FLAG_NHSYNC)
		pol |= BIT(0);
	rmw(link + SST1_VIDEO_CONTROL, DYNAMIC_RANGE_MODE | VIDEO_BPC | COLOR_FORMAT | SYNC_POLARITY,
	    FIELD_PREP(VIDEO_BPC, 1) | pol);
	writel(m->vtotal, link + SST1_VIDEO_VTOTAL);
	writel(m->htotal, link + SST1_VIDEO_HTOTAL);
	writel(m->vdisplay, link + SST1_VIDEO_VACTIVE);
	writel(m->vsync_start - m->vdisplay, link + SST1_VIDEO_VFP);
	writel(m->vtotal - m->vsync_end, link + SST1_VIDEO_VBP);
	writel(m->hdisplay, link + SST1_VIDEO_HACTIVE);
	writel(m->hsync_start - m->hdisplay, link + SST1_VIDEO_HFP);
	writel(m->htotal - m->hsync_end, link + SST1_VIDEO_HBP);

	writel(clk >> 1, link + SST1_MVID_MASTER_MODE);
	writel(ls, link + SST1_NVID_MASTER_MODE);
	rmw(link + SST1_MAIN_CONTROL, MVID_MODE, MVID_MODE);
	writel(clk, link + SST1_MVID_SFR_CONFIGURE);
	writel(ls, link + SST1_NVID_SFR_CONFIGURE);

	tu_off = div64_u64(clk * bpp * 32 * e10, lanes * ls * 8);
	tu_on = div64_u64(tu_off * 1000, 976);
	i_off = div64_u64(tu_off, e10);
	f_off = div64_u64(tu_off - i_off * e10, 10);
	i_on = div64_u64(tu_on, e10);
	f_on = div64_u64(tu_on - i_on * e10, 10);
	writel(1, link + SST1_AS_MODE_CONTROL);
	writel(1, link + SST1_AS_THRESHOLD_SEL_FEC_OFF);
	writel(1, link + SST1_AS_THRESHOLD_SEL_FEC_ON);
	writel(i_off, link + SST1_AS_INTEGER_FEC_OFF);
	writel(f_off, link + SST1_AS_FRACTION_FEC_OFF);
	writel(i_off <= 2 ? 7 : i_off <= 5 ? 8 : 9, link + SST1_AS_THRESHOLD_FEC_OFF);
	writel(i_on, link + SST1_AS_INTEGER_FEC_ON);
	writel(f_on, link + SST1_AS_FRACTION_FEC_ON);
	writel(i_on <= 2 ? 7 : i_on <= 5 ? 8 : 9, link + SST1_AS_THRESHOLD_FEC_ON);

	rmw(link + SST1_VIDEO_MASTER_TIMING_GEN, 1, 1);
	rmw(link + SST1_MAIN_CONTROL, VIDEO_MODE, 0);
	outf(dp, "video: %dx%d@%d (%d kHz) at ls %llu kHz x%llu, TU %u.%010u\n", m->hdisplay, m->vdisplay,
	     drm_mode_vrefresh(m), m->clock, ls, lanes, i_off, f_off);
	return 0;
}

/* stock displayport_reg_start */
static void dp_video_start(struct r8s_dp *dp)
{
	rmw(dp->link + MST_STREAM_1_ENABLE, 1, 1);
	rmw(dp->link + SST1_VIDEO_ENABLE, 1, 1);
}

static void dp_video_off(struct r8s_dp *dp)
{
	void __iomem *link = dp->link;

	rmw(link + SST1_VIDEO_ENABLE, 1, 0);
	rmw(link + SST1_VIDEO_BIST_CONTROL, BIST_EN, 0);
	rmw(link + SST1_VIDEO_CONTROL, STRM_VALID_FORCE | STRM_VALID_CTRL, 0);
	rmw(link + MST_STREAM_1_ENABLE, 1, 0);
}

/* stock displayport_reg_set_bist_video_configuration + displayport_reg_start */
static void dp_bist(struct r8s_dp *dp, bool on)
{
	void __iomem *link = dp->link;

	dp_video_off(dp);
	if (!on) {
		outf(dp, "bist off\n");
		return;
	}
	if (dp_video_config(dp, &cea_1080p60))
		return;
	rmw(link + SST1_VIDEO_BIST_CONTROL, BIST_TYPE | CTS_BIST_EN, 0);	/* colour bars */
	rmw(link + SST1_VIDEO_CONTROL, STRM_VALID_FORCE | STRM_VALID_CTRL,
	    STRM_VALID_FORCE | STRM_VALID_CTRL);
	rmw(link + SST1_VIDEO_BIST_CONTROL, BIST_EN, BIST_EN);
	dp_video_start(dp);
	msleep(50);
	outf(dp, "bist on: FSM %#x, VIDEO_CONTROL %#x\n", readl(link + SST1_VIDEO_DEBUG_FSM_STATE),
	     readl(link + SST1_VIDEO_CONTROL));
}

static void decon_status(struct r8s_dp *dp)
{
	void __iomem *d0 = dp->d0, *d1 = dp->d1;
	u32 fc = readl(d1 + D_FRAME_COUNT);

	msleep(100);
	outf(dp, "DECON1 GC %#x DP0 %#x INT %#x/%#x frames +%u in 100 ms, DP FSM %#x LH %#x\n",
	     readl(d1 + D_GLOBAL_CONTROL), readl(d0 + D_DP_CONNECTION),
	     readl(d1 + D_INTERRUPT_PENDING), readl(d1 + D_EXTRA_INTERRUPT_PENDING),
	     readl(d1 + D_FRAME_COUNT) - fc, readl(dp->link + SST1_VIDEO_DEBUG_FSM_STATE),
	     readl(dp->link + SYSTEM_SST1_FUNCTION_ENABLE));
	outf(dp, "DECON1 DP0..2 %#x %#x %#x OUTFIFO %#x win3 %#x %#x %#x %#x %#x\n",
	     readl(d1 + D_DATA_PATH_0), readl(d1 + D_DATA_PATH_1), readl(d1 + D_DATA_PATH_2),
	     readl(d1 + D_OUTFIFO_SIZE_0), readl(d0 + D_WIN_CONTROL_0(DP_WIN)),
	     readl(d0 + D_WIN_START(DP_WIN)), readl(d0 + D_WIN_END(DP_WIN)),
	     readl(d0 + D_WIN_COLORMAP_0(DP_WIN)), readl(d0 + D_WIN_COLORMAP_1(DP_WIN)));
}

/* DECON0 runs the panel: while it is enabled the DPU's power domain and clocks are on */
static bool dpu_on(struct r8s_dp *dp)
{
	return readl(dp->d0 + D_GLOBAL_CONTROL) & D_GC_EN;
}

/* stock decon_reg_init for DECON1 in video mode on DP0 */
static void decon1_init(struct r8s_dp *dp, u32 w, u32 h)
{
	void __iomem *d0 = dp->d0, *d1 = dp->d1;

	rmw(d1 + D_CLOCK_CONTROL_0, D_CG_QACTIVE, 0);
	rmw(d0 + D_CLOCK_CONTROL_0, D_TE_QACTIVE_PLL_ON, D_TE_QACTIVE_PLL_ON);
	writel(D_SRAM3_4, d1 + D_SRAM_SHARE_MAIN);
	rmw(d1 + D_GLOBAL_CONTROL, D_GC_OPERATION_CMD, 0);		/* video mode */
	writel(h << 16 | w, d1 + D_BLENDER_BG_SIZE);
	writel(h << 16 | w, d1 + D_SCALED_SIZE);
	rmw(d1 + D_OUTFIFO_ORDER, GENMASK(6, 4), 0);			/* RGB */
	rmw(d1 + D_DATA_PATH_2, GENMASK(25, 24) | GENMASK(14, 12) | GENMASK(7, 0), D_DPATH_NOCOMP_DPIF);
	writel(h << 16 | w, d1 + D_OUTFIFO_SIZE_0);
	rmw(d1 + D_OUTFIFO_TH_0, 7, 5);					/* 1H */
	rmw(d1 + D_GLOBAL_CONTROL, D_GC_EN_F, 0);
	writel(readl(d1 + D_INTERRUPT_PENDING), d1 + D_INTERRUPT_PENDING);
	writel(readl(d1 + D_EXTRA_INTERRUPT_PENDING), d1 + D_EXTRA_INTERRUPT_PENDING);
	rmw(d0 + D_DP_CONNECTION, 7, 1);				/* DP0 <- DECON1 */
}

/* window 3, full screen: a colour map (8-bit fields), or DMA channel 1 */
static void decon1_window(struct r8s_dp *dp, u32 w, u32 h, bool cmap, u32 rgb)
{
	void __iomem *d0 = dp->d0, *d1 = dp->d1;

	/* the blend function S-Boot gave the panel's window */
	writel(0x102, d0 + D_WIN_CONTROL_0(DP_WIN));
	writel(0, d0 + D_WIN_CONTROL_1(DP_WIN));
	writel(0, d0 + D_WIN_START(DP_WIN));
	writel((h - 1) << 16 | (w - 1), d0 + D_WIN_END(DP_WIN));
	writel(0, d0 + D_WIN_START_TIME(DP_WIN));
	if (cmap) {
		writel(0xff << 16 | (rgb >> 16 & 0xff), d0 + D_WIN_COLORMAP_0(DP_WIN));
		writel((rgb >> 8 & 0xff) << 16 | (rgb & 0xff), d0 + D_WIN_COLORMAP_1(DP_WIN));
	}
	rmw(d1 + D_DATA_PATH_1, 7 << (4 * DP_WIN), DP_CH << (4 * DP_WIN));
	rmw(d1 + D_DATA_PATH_0, 3 << (4 * DP_WIN), (cmap ? 3 : 1) << (4 * DP_WIN));
	rmw(d1 + D_SHADOW_REG_UPDATE_REQ, BIT(DP_WIN), BIT(DP_WIN));
}

/* DMA channel 1 and DPP1 read @w x @h pixels of a buffer @stride pixels wide */
static void dma1_set(struct r8s_dp *dp, u32 base, u32 fmt, u32 stride, u32 w, u32 h)
{
	writel((IDMA_IN_CON_XRGB8888 & ~GENMASK(13, 8)) | FIELD_PREP(GENMASK(13, 8), fmt),
	       dp->dma1 + IDMA_IN_CON);
	writel(h << 16 | stride, dp->dma1 + IDMA_SRC_SIZE);
	writel(0, dp->dma1 + IDMA_SRC_OFFSET);
	writel(h << 16 | w, dp->dma1 + IDMA_IMG_SIZE);
	writel(base, dp->dma1 + IDMA_IN_BASE_ADDR_Y8);
	writel(0xd0840100, dp->dma1 + IDMA_RECOVERY_CTRL);	/* channel 0's values */
	writel(0x8a4c3301, dp->dma1 + IDMA_DEADLOCK_EN);
	writel(h << 16 | w, dp->dpp1 + DPP_IMG_SIZE);
	writel(1, dp->dpp1 + DPP_LINECNT_CON);
}

/* stock decon_reg_start: the DPU-to-DP channel, then DECON1 */
static int decon1_start(struct r8s_dp *dp)
{
	void __iomem *d1 = dp->d1, *link = dp->link;
	u32 v;
	int i;

	rmw(link + SYSTEM_SST1_FUNCTION_ENABLE, SST1_LH_PWR_ON, SST1_LH_PWR_ON);
	for (i = 0; i < 20000 && !(readl(link + SYSTEM_SST1_FUNCTION_ENABLE) & SST1_LH_PWR_ON_STATUS); i++)
		udelay(1);
	v = readl(link + SYSTEM_SST1_FUNCTION_ENABLE);
	rmw(d1 + D_GLOBAL_CONTROL, D_GC_EN | D_GC_EN_F, D_GC_EN | D_GC_EN_F);
	rmw(d1 + D_SHADOW_REG_UPDATE_REQ, D_SHADOW_GLOBAL, D_SHADOW_GLOBAL);
	for (i = 0; i < 2000 && !(readl(d1 + D_GLOBAL_CONTROL) & D_GC_RUN); i++)
		udelay(10);
	outf(dp, "decon1: LH %s (%#x), %s\n", v & SST1_LH_PWR_ON_STATUS ? "on" : "TIMEOUT", v,
	     readl(d1 + D_GLOBAL_CONTROL) & D_GC_RUN ? "runs" : "NOT running");
	return readl(d1 + D_GLOBAL_CONTROL) & D_GC_RUN ? 0 : -ETIMEDOUT;
}

/* stock decon_reg_stop: at the end of a frame if it stops in time, at once if not */
static void decon1_stop(struct r8s_dp *dp)
{
	void __iomem *d0 = dp->d0, *d1 = dp->d1;
	int i;

	writel(0, d1 + D_INTERRUPT_ENABLE);
	rmw(d1 + D_GLOBAL_CONTROL, D_GC_EN_F, 0);
	rmw(d1 + D_SHADOW_REG_UPDATE_REQ, D_SHADOW_GLOBAL, D_SHADOW_GLOBAL);
	for (i = 0; i < 50 && (readl(d1 + D_GLOBAL_CONTROL) & D_GC_RUN); i++)
		msleep(1);
	if (readl(d1 + D_GLOBAL_CONTROL) & D_GC_RUN) {
		rmw(d1 + D_GLOBAL_CONTROL, D_GC_EN | D_GC_EN_F, 0);
		rmw(d1 + D_SHADOW_REG_UPDATE_REQ, D_SHADOW_GLOBAL, D_SHADOW_GLOBAL);
		msleep(20);
	}
	rmw(dp->link + SYSTEM_SST1_FUNCTION_ENABLE, SST1_LH_PWR_ON, 0);
	rmw(d1 + D_DATA_PATH_0, 3 << (4 * DP_WIN), 0);
	rmw(d0 + D_DP_CONNECTION, 7, 3);
	writel(readl(d1 + D_INTERRUPT_PENDING), d1 + D_INTERRUPT_PENDING);
}

/* 1920x1080 XRGB: eight bars above, a grey ramp below, a white frame and a red square in the top left corner */
static void fill_pattern(u32 *px, u32 w, u32 h)
{
	static const u32 bars[8] = { 0xffffff, 0xffff00, 0x00ffff, 0x00ff00, 0xff00ff, 0xff0000, 0x0000ff, 0x000000 };
	u32 x, y, v;

	for (y = 0; y < h; y++)
		for (x = 0; x < w; x++) {
			if (x < 8 || y < 8 || x >= w - 8 || y >= h - 8)
				v = 0xffffff;
			else if (x >= 40 && x < 240 && y >= 40 && y < 240)
				v = 0xff0000;
			else if (y < h / 2)
				v = bars[x * 8 / w];
			else
				v = (x * 255 / (w - 1)) * 0x010101;
			px[y * w + x] = v;
		}
}

/* window 3 reads a test pattern through DMA channel 1 and DPP1 instead of its colour map */
static void dp_decon_img(struct r8s_dp *dp)
{
	const u32 w = 1920, h = 1080;

	if (!(readl(dp->d1 + D_GLOBAL_CONTROL) & D_GC_RUN)) {
		outf(dp, "decon on first\n");
		return;
	}
	if (!dp->img) {
		dp->img = dmam_alloc_coherent(dp->dev, w * h * 4, &dp->img_dma, GFP_KERNEL);
		if (!dp->img) {
			outf(dp, "no memory for the picture\n");
			return;
		}
		fill_pattern(dp->img, w, h);
	}
	if (upper_32_bits(dp->img_dma)) {
		outf(dp, "picture at %pad, above 4 GiB\n", &dp->img_dma);
		return;
	}
	dma1_set(dp, lower_32_bits(dp->img_dma), 4, w, w, h);
	decon1_window(dp, w, h, false, 0);
	rmw(dp->d1 + D_SHADOW_REG_UPDATE_REQ, D_SHADOW_GLOBAL, D_SHADOW_GLOBAL);
	msleep(50);
	outf(dp, "decon img: picture at %pad, DMA1 shadow base %#x size %#x, DPP1 %#x\n", &dp->img_dma,
	     readl(dp->dma1 + 0x800 + IDMA_IN_BASE_ADDR_Y8), readl(dp->dma1 + 0x800 + IDMA_IMG_SIZE),
	     readl(dp->dpp1 + 0xb18));
	decon_status(dp);
}

/* DECON1 on DP0 at 1080p60 with a colour-map window: no DMA, no DPP, nothing in memory */
static void dp_decon(struct r8s_dp *dp, bool on, u32 rgb)
{
	const struct drm_display_mode *m = &cea_1080p60;

	if (!dpu_on(dp)) {
		outf(dp, "DECON0 is not running: leaving the DPU alone\n");
		return;
	}
	if (!on) {
		decon1_stop(dp);
		dp_video_off(dp);
		outf(dp, "decon off: DECON1 GC %#x\n", readl(dp->d1 + D_GLOBAL_CONTROL));
		return;
	}
	if (readl(dp->d1 + D_GLOBAL_CONTROL) & D_GC_RUN) {
		outf(dp, "DECON1 already runs: decon off first\n");
		return;
	}
	dp_video_off(dp);
	if (dp_video_config(dp, m))
		return;
	dp_video_start(dp);
	decon1_init(dp, m->hdisplay, m->vdisplay);
	decon1_window(dp, m->hdisplay, m->vdisplay, true, rgb);
	decon1_start(dp);
	decon_status(dp);
}

static void hexdump(struct r8s_dp *dp, u32 addr, const u8 *buf, u32 len)
{
	u32 i;

	for (i = 0; i < len; i += 16)
		outf(dp, "%05x: %*ph\n", addr + i, min(16u, len - i), buf + i);
}

static void dump(struct r8s_dp *dp)
{
	static const u16 link_regs[] = { 0x0004, 0x0008, 0x000c, 0x0010, 0x0014, 0x0018,
		0x001c, 0x0020, 0x0028, 0x002c, 0x0108, 0x0200, 0x1000, 0x1010, 0x1014,
		0x1018, 0x101c, 0x3004 };
	static const u16 pma_regs[] = { CMN_REG0008, CMN_REG000A, CMN_REG00B8, CMN_REG00B9,
		CMN_REG00BD, CMN_REG01C0 };
	unsigned int pmu = 0;
	int i;

	regmap_read(dp->pmu, PMU_USBDP_PHY_CONTROL, &pmu);
	dev_info(dp->dev, "dump: usbcon\n");
	outf(dp, "PMU 0x704 %#x, USBCON 0x48 %#x, orientation gpio %d, aux_oe %d, sbu_sel %d\n",
	     pmu, readl(dp->base[CTRL] + 0x48), dp->flip,
	     gpiod_get_value(dp->aux_oe), gpiod_get_value(dp->sbu_sel));
	dev_info(dp->dev, "dump: link\n");
	for (i = 0; i < ARRAY_SIZE(link_regs); i++)
		outf(dp, "link %04x: %08x\n", link_regs[i], readl(dp->link + link_regs[i]));
	if (!pma_ok(dp)) {
		outf(dp, "pma: in reset or isolated, not read\n");
		return;
	}
	dev_info(dp->dev, "dump: pma\n");
	for (i = 0; i < ARRAY_SIZE(pma_regs); i++)
		outf(dp, "pma  %04x: %08x\n", pma_regs[i], readl(dp->base[PMA] + pma_regs[i]));
}

static ssize_t ctl_write(struct file *f, const char __user *ubuf, size_t n, loff_t *pos)
{
	struct r8s_dp *dp = f->private_data;
	char cmd[64], arg[16] = "";
	u32 a = 0, b = 0;
	u8 buf[128];
	int ret;

	if (n >= sizeof(cmd))
		return -EINVAL;
	if (copy_from_user(cmd, ubuf, n))
		return -EFAULT;
	cmd[n] = 0;

	/* detect takes the lock itself */
	if (sysfs_streq(cmd, "replug")) {
		/* drm_helper_hpd_irq_event() skips polled connectors: this one checks ours */
		WRITE_ONCE(dp->unplugged, true);
		drm_connector_helper_hpd_irq_event(&dp->conn);
		msleep(2000);
		WRITE_ONCE(dp->unplugged, false);
		drm_connector_helper_hpd_irq_event(&dp->conn);
		mutex_lock(&dp->lock);
		dp->len = 0;
		outf(dp, "replugged: %s\n", dp->connected ? "connected" : "disconnected");
		mutex_unlock(&dp->lock);
		return n;
	}

	mutex_lock(&dp->lock);
	dp->len = 0;
	dp->flip = gpiod_get_value(dp->orient);
	if (dp->streaming && (sysfs_streq(cmd, "phy") || sysfs_streq(cmd, "link") || !strncmp(cmd, "aux", 3) ||
			      !strncmp(cmd, "train", 5) || !strncmp(cmd, "bist", 4) ||
			      (!strncmp(cmd, "decon", 5) && strcmp(cmd, "decon st\n") && strcmp(cmd, "decon st")))) {
		outf(dp, "the DRM pipe has the link and DECON1\n");
	} else if (sysfs_streq(cmd, "phy")) {
		dp_phy_on(dp);
	} else if (sysfs_streq(cmd, "link")) {
		dp_link_on(dp);
	} else if (sscanf(cmd, "aux %15s", arg) == 1) {
		if (!strcmp(arg, "on")) {
			/* stock displayport_aux_sel + displayport_aux_onoff */
			gpiod_set_value(dp->sbu_sel, dp->flip);
			gpiod_set_value(dp->aux_oe, 0);
			msleep(100);
		} else {
			gpiod_set_value(dp->aux_oe, 1);
		}
		outf(dp, "aux switch %s, sbu_sel %d\n", gpiod_get_value(dp->aux_oe) ? "off" : "on",
		     gpiod_get_value(dp->sbu_sel));
	} else if ((!strncmp(cmd, "dpcd", 4) || !strncmp(cmd, "edid", 4) || !strncmp(cmd, "train", 5) ||
		    !strncmp(cmd, "bist", 4) || !strncmp(cmd, "linkst", 6) || !strncmp(cmd, "decon", 5)) &&
		   !pma_ok(dp)) {
		outf(dp, "run phy and link first\n");
	} else if (sscanf(cmd, "dpcd %x %u", &a, &b) == 2 && b && b <= 16) {
		ret = dpcd_read(dp, a, b, buf);
		if (ret)
			outf(dp, "dpcd %#x: %d\n", a, ret);
		else
			hexdump(dp, a, buf, b);
	} else if (sysfs_streq(cmd, "edid")) {
		ret = edid_read(dp, 0, buf);
		if (ret)
			outf(dp, "edid: %d\n", ret);
		else
			hexdump(dp, 0, buf, 128);
	} else if (sscanf(cmd, "train %hhx %hhu", (u8 *)&arg[0], (u8 *)&arg[1]) == 2) {
		if (arg[1] == 1 || arg[1] == 2 || arg[1] == 4)
			dp_train(dp, arg[0], arg[1]);
		else
			outf(dp, "lanes: 1, 2 or 4\n");
	} else if (sscanf(cmd, "bist %15s", arg) == 1) {
		dp_bist(dp, !strcmp(arg, "on"));
	} else if (sscanf(cmd, "decon %15s", arg) == 1) {
		u32 rgb = 0xff8000;

		sscanf(cmd, "decon %*s %x", &rgb);
		if (!strcmp(arg, "st"))
			decon_status(dp);
		else if (!strcmp(arg, "img"))
			dp_decon_img(dp);
		else
			dp_decon(dp, !strcmp(arg, "on"), rgb);
	} else if (sysfs_streq(cmd, "linkst")) {
		if (!dpcd_read(dp, 0x200, 8, buf))
			hexdump(dp, 0x200, buf, 8);
		if (!dpcd_read(dp, 0x100, 8, buf))
			hexdump(dp, 0x100, buf, 8);
	} else if (sysfs_streq(cmd, "dump")) {
		dump(dp);
	} else {
		mutex_unlock(&dp->lock);
		return -EINVAL;
	}
	mutex_unlock(&dp->lock);
	return n;
}

static ssize_t ctl_read(struct file *f, char __user *ubuf, size_t n, loff_t *pos)
{
	struct r8s_dp *dp = f->private_data;
	ssize_t ret;

	mutex_lock(&dp->lock);
	ret = simple_read_from_buffer(ubuf, n, pos, dp->out, dp->len);
	mutex_unlock(&dp->lock);
	return ret;
}

static const struct file_operations ctl_fops = {
	.owner	= THIS_MODULE,
	.open	= simple_open,
	.read	= ctl_read,
	.write	= ctl_write,
};

/* the SoC's GPIO banks have no phandles in this DT: take the lines by bank */
/* ---- DRM: DECON1 + DMA channel 1 on DP0, a display device of its own for mutter ---- */

#define R8S_MAX_W	1920
#define R8S_MAX_H	1080

static inline struct r8s_dp *to_dp(struct drm_device *drm)
{
	return container_of(drm, struct r8s_dp, drm);
}

/* the PHY up, the link's functions on and the AUX switch closed: what phy, link and aux on do (lock) */
static bool dp_hw_ready(struct r8s_dp *dp)
{
	return dp->hw_up && dp->hw_flip == gpiod_get_value(dp->orient) &&
	       pma_ok(dp) && !gpiod_get_value(dp->aux_oe) &&
	       (readl(dp->link + SYSTEM_COMMON_FUNCTION_ENABLE) & AUX_FUNC_EN);
}

static void dp_hw_up(struct r8s_dp *dp)
{
	if (dp_hw_ready(dp))
		return;
	dp->flip = gpiod_get_value(dp->orient);
	dp_phy_on(dp);
	dp_link_on(dp);
	gpiod_set_value(dp->sbu_sel, dp->flip);
	gpiod_set_value(dp->aux_oe, 0);
	msleep(100);
	dp->hw_up = true;
	dp->hw_flip = dp->flip;
}

/* EDID blocks 0 and 1 (the TV's CTA extension), over I2C-over-AUX */
static const struct drm_edid *dp_read_edid(struct r8s_dp *dp)
{
	const struct drm_edid *edid = NULL;
	u8 *buf = kzalloc(256, GFP_KERNEL);
	int blocks = 1;

	if (!buf)
		return NULL;
	if (!edid_read(dp, 0, buf)) {
		if (buf[126] && !edid_read(dp, 1, buf + 128))
			blocks = 2;
		edid = drm_edid_alloc(buf, blocks * 128);
		if (edid && !drm_edid_valid(edid)) {
			drm_edid_free(edid);
			edid = NULL;
		}
	}
	kfree(buf);
	return edid;
}

/*
 * Two lanes (pin D) at HBR. The hub trains at HBR2 too, but 1080p120 over it
 * left the TV on "no signal" (11-19) while 1080p60 at HBR shows: HBR only until
 * HBR2 is understood.
 */
#define R8S_LINK_RATE	0x0a

static enum drm_mode_status dp_mode_valid(struct r8s_dp *dp, const struct drm_display_mode *m)
{
	u64 rate = R8S_LINK_RATE;

	if (m->hdisplay > R8S_MAX_W || m->vdisplay > R8S_MAX_H)
		return MODE_BAD;
	if (m->flags & DRM_MODE_FLAG_INTERLACE)
		return MODE_NO_INTERLACE;
	if (m->hdisplay & 1)
		return MODE_BAD_WIDTH;
	if ((u64)m->clock * 24 > rate * 27000 * 2 * 8)
		return MODE_CLOCK_HIGH;
	return MODE_OK;
}

static int dp_conn_detect(struct drm_connector *conn, struct drm_modeset_acquire_ctx *ctx, bool force)
{
	struct r8s_dp *dp = to_dp(conn->dev);
	bool on;
	u8 caps[16];

	mutex_lock(&dp->lock);
	dp->len = 0;
	if (!dp->streaming)
		dp_hw_up(dp);
	on = !READ_ONCE(dp->unplugged) && dp_hw_ready(dp) && !dpcd_read(dp, 0, 16, caps) && caps[0];
	if (on && (!dp->connected || !dp->edid)) {
		memcpy(dp->caps, caps, sizeof(caps));
		drm_edid_free(dp->edid);
		dp->edid = dp_read_edid(dp);
		dev_info(dp->dev, "sink: DPCD %*ph, EDID %s\n", 16, caps, dp->edid ? "read" : "unreadable");
	} else if (!on && dp->connected) {
		drm_edid_free(dp->edid);
		dp->edid = NULL;
		dev_info(dp->dev, "sink gone\n");
	}
	dp->connected = on;
	drm_edid_connector_update(conn, dp->edid);
	mutex_unlock(&dp->lock);
	return on ? connector_status_connected : connector_status_disconnected;
}

static int dp_conn_get_modes(struct drm_connector *conn)
{
	int n = drm_edid_connector_add_modes(conn);

	if (!n)
		n = drm_add_modes_noedid(conn, R8S_MAX_W, R8S_MAX_H);
	/* the TV prefers 4K, which two lanes cannot carry */
	drm_set_preferred_mode(conn, R8S_MAX_W, R8S_MAX_H);
	return n;
}

static enum drm_mode_status dp_conn_mode_valid(struct drm_connector *conn, const struct drm_display_mode *m)
{
	return dp_mode_valid(to_dp(conn->dev), m);
}

static const struct drm_connector_helper_funcs dp_conn_helper_funcs = {
	.get_modes = dp_conn_get_modes,
	.mode_valid = dp_conn_mode_valid,
	.detect_ctx = dp_conn_detect,
};

static const struct drm_connector_funcs dp_conn_funcs = {
	.reset = drm_atomic_helper_connector_reset,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.destroy = drm_connector_cleanup,
	.atomic_duplicate_state = drm_atomic_helper_connector_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_connector_destroy_state,
};

static enum drm_mode_status dp_pipe_mode_valid(struct drm_simple_display_pipe *pipe,
					       const struct drm_display_mode *m)
{
	return dp_mode_valid(to_dp(pipe->crtc.dev), m);
}

static int dp_pipe_check(struct drm_simple_display_pipe *pipe, struct drm_plane_state *plane_state,
			 struct drm_crtc_state *crtc_state)
{
	const struct drm_display_mode *m = &crtc_state->mode;
	struct drm_framebuffer *fb = plane_state->fb;
	dma_addr_t addr;
	int ret;

	ret = drm_atomic_helper_check_plane_state(plane_state, crtc_state, DRM_PLANE_NO_SCALING,
						  DRM_PLANE_NO_SCALING, false, false);
	if (ret || !fb || !plane_state->visible)
		return ret;
	/* full-screen, unpanned, stride a whole number of pixels */
	if (plane_state->src.x1 || plane_state->src.y1 || fb->width < m->hdisplay || fb->height < m->vdisplay ||
	    fb->pitches[0] % 4 || fb->pitches[0] / 4 > 0xffff)
		return -EINVAL;
	/* the SysMMU is in bypass and the base register is 32 bits wide */
	addr = drm_fb_dma_get_gem_addr(fb, plane_state, 0);
	if (upper_32_bits(addr + (u64)fb->pitches[0] * m->vdisplay - 1))
		return -EINVAL;
	return 0;
}

static void dp_scanout(struct r8s_dp *dp, struct drm_plane_state *state, const struct drm_display_mode *m)
{
	struct drm_framebuffer *fb = state->fb;
	dma_addr_t addr = drm_fb_dma_get_gem_addr(fb, state, 0);

	dma1_set(dp, lower_32_bits(addr), fb->format->format == DRM_FORMAT_ARGB8888 ? 0 : 4,
		 fb->pitches[0] / 4, m->hdisplay, m->vdisplay);
}

static void dp_pipe_enable(struct drm_simple_display_pipe *pipe, struct drm_crtc_state *crtc_state,
			   struct drm_plane_state *plane_state)
{
	struct r8s_dp *dp = to_dp(pipe->crtc.dev);
	const struct drm_display_mode *m = &crtc_state->adjusted_mode;
	bool with_fb = plane_state->fb && plane_state->visible;
	u8 rate = R8S_LINK_RATE;
	int ret = -ENODEV;

	mutex_lock(&dp->lock);
	dp->len = 0;
	if (!dpu_on(dp)) {
		dev_err(dp->dev, "DECON0 is off: the DPU may be powered down\n");
		goto out;
	}
	if (readl(dp->d1 + D_GLOBAL_CONTROL) & D_GC_RUN)	/* left running from debugfs */
		decon1_stop(dp);
	dp_video_off(dp);
	dp_hw_up(dp);
	ret = dp_train(dp, rate, 2);
	if (!ret)
		ret = dp_video_config(dp, m);
	if (!ret) {
		dp_video_start(dp);
		decon1_init(dp, m->hdisplay, m->vdisplay);
		if (with_fb)
			dp_scanout(dp, plane_state, m);
		decon1_window(dp, m->hdisplay, m->vdisplay, !with_fb, 0);
		ret = decon1_start(dp);
		if (!ret) {
			writel(readl(dp->d1 + D_INTERRUPT_PENDING), dp->d1 + D_INTERRUPT_PENDING);
			writel(D_INT_FRAME_START_EN, dp->d1 + D_INTERRUPT_ENABLE);
		}
	}
	dp->streaming = !ret;
out:
	dev_info(dp->dev, "enable %dx%d@%d: %d\n%s", m->hdisplay, m->vdisplay, drm_mode_vrefresh(m), ret,
		 dp->out);
	mutex_unlock(&dp->lock);
	drm_crtc_vblank_on(&pipe->crtc);
}

static void dp_pipe_disable(struct drm_simple_display_pipe *pipe)
{
	struct r8s_dp *dp = to_dp(pipe->crtc.dev);

	drm_crtc_vblank_off(&pipe->crtc);
	mutex_lock(&dp->lock);
	if (dp->streaming) {
		decon1_stop(dp);
		dp_video_off(dp);
		dp->streaming = false;
	}
	mutex_unlock(&dp->lock);
}

static void dp_pipe_update(struct drm_simple_display_pipe *pipe, struct drm_plane_state *old_state)
{
	struct r8s_dp *dp = to_dp(pipe->crtc.dev);
	struct drm_plane_state *state = pipe->plane.state;
	struct drm_crtc *crtc = &pipe->crtc;
	struct drm_pending_vblank_event *event;
	unsigned long flags;

	if (READ_ONCE(dp->streaming) && state->fb && state->visible) {
		dp_scanout(dp, state, &crtc->state->mode);
		writel(D_SHADOW_GLOBAL | BIT(DP_WIN), dp->d1 + D_SHADOW_REG_UPDATE_REQ);
	}

	event = crtc->state->event;
	if (!event)
		return;
	crtc->state->event = NULL;
	spin_lock_irqsave(&crtc->dev->event_lock, flags);
	if (READ_ONCE(dp->streaming) && drm_crtc_vblank_get(crtc) == 0)
		drm_crtc_arm_vblank_event(crtc, event);
	else
		drm_crtc_send_vblank_event(crtc, event);
	spin_unlock_irqrestore(&crtc->dev->event_lock, flags);
}

static int dp_pipe_enable_vblank(struct drm_simple_display_pipe *pipe)
{
	struct r8s_dp *dp = to_dp(pipe->crtc.dev);

	if (!READ_ONCE(dp->streaming))
		return -EINVAL;
	writel(D_INT_FRAME_START_EN, dp->d1 + D_INTERRUPT_ENABLE);
	return 0;
}

static void dp_pipe_disable_vblank(struct drm_simple_display_pipe *pipe)
{
	writel(0, to_dp(pipe->crtc.dev)->d1 + D_INTERRUPT_ENABLE);
}

static const struct drm_simple_display_pipe_funcs dp_pipe_funcs = {
	.mode_valid = dp_pipe_mode_valid,
	.check = dp_pipe_check,
	.enable = dp_pipe_enable,
	.disable = dp_pipe_disable,
	.update = dp_pipe_update,
	.enable_vblank = dp_pipe_enable_vblank,
	.disable_vblank = dp_pipe_disable_vblank,
};

/* DECON1's FRAME_START line (SPI 257) is vblank */
static irqreturn_t dp_decon1_irq(int irq, void *data)
{
	struct r8s_dp *dp = data;
	u32 pend = readl(dp->d1 + D_INTERRUPT_PENDING);

	if (!(pend & D_INT_FRAME_START))
		return IRQ_NONE;
	writel(pend, dp->d1 + D_INTERRUPT_PENDING);
	drm_crtc_handle_vblank(&dp->pipe.crtc);
	return IRQ_HANDLED;
}

/* the overlay node has no interrupts property: map SPI 257 on its interrupt parent, the GIC */
static const u32 dp_formats[] = {
	DRM_FORMAT_XRGB8888,
	DRM_FORMAT_ARGB8888,
};

static const struct drm_mode_config_funcs dp_mode_config_funcs = {
	.fb_create = drm_gem_fb_create,
	.atomic_check = drm_atomic_helper_check,
	.atomic_commit = drm_atomic_helper_commit,
};

DEFINE_DRM_GEM_DMA_FOPS(dp_fops);

static const struct drm_driver dp_drm_driver = {
	.driver_features = DRIVER_GEM | DRIVER_MODESET | DRIVER_ATOMIC,
	/* "exynos", as the panel's device: Mesa's kmsro then renders into our buffers with the GPU */
	.name = "exynos",
	.desc = "Exynos 990 DisplayPort (DECON1)",
	.major = 1,
	.minor = 0,
	.fops = &dp_fops,
	DRM_GEM_DMA_DRIVER_OPS,
};

static int dp_drm_init(struct r8s_dp *dp)
{
	struct drm_device *drm = &dp->drm;
	int ret;

	ret = drmm_mode_config_init(drm);
	if (ret)
		return ret;
	drm->mode_config.min_width = 640;
	drm->mode_config.max_width = 4096;
	drm->mode_config.min_height = 480;
	drm->mode_config.max_height = 4096;
	drm->mode_config.preferred_depth = 24;
	drm->mode_config.funcs = &dp_mode_config_funcs;

	ret = drm_connector_init(drm, &dp->conn, &dp_conn_funcs, DRM_MODE_CONNECTOR_DisplayPort);
	if (ret)
		return ret;
	drm_connector_helper_add(&dp->conn, &dp_conn_helper_funcs);
	dp->conn.polled = DRM_CONNECTOR_POLL_CONNECT | DRM_CONNECTOR_POLL_DISCONNECT;

	ret = drm_simple_display_pipe_init(drm, &dp->pipe, &dp_pipe_funcs, dp_formats, ARRAY_SIZE(dp_formats),
					   NULL, &dp->conn);
	if (ret)
		return ret;
	ret = drm_vblank_init(drm, 1);
	if (ret)
		return ret;
	ret = devm_request_irq(dp->dev, dp->irq, dp_decon1_irq, 0, "r8s-dp-decon1", dp);
	if (ret)
		return ret;
	drm_mode_config_reset(drm);
	drm_kms_helper_poll_init(drm);
	ret = drm_dev_register(drm, 0);
	if (ret)
		drm_kms_helper_poll_fini(drm);
	return ret;
}

static void free_gpios(struct r8s_dp *dp)
{
	if (!IS_ERR_OR_NULL(dp->aux_oe))
		gpiod_set_value(dp->aux_oe, 1);		/* AUX switch off; devm releases the lines */
}

static int r8s_dp_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct r8s_dp *dp;
	struct resource *res;
	void __iomem **map[4];
	int i, ret;

	dp = devm_drm_dev_alloc(dev, &dp_drm_driver, struct r8s_dp, drm);
	if (IS_ERR(dp))
		return PTR_ERR(dp);
	dp->out = devm_kzalloc(dev, OUTSZ, GFP_KERNEL);
	if (!dp->out)
		return -ENOMEM;
	dp->dev = dev;
	dp->lanes = 2;
	mutex_init(&dp->lock);

	/* not requested: USBCON belongs to the USB PHY driver */
	map[0] = &dp->link;
	map[1] = &dp->base[PMA];
	map[2] = &dp->base[PCS];
	map[3] = &dp->base[CTRL];
	for (i = 0; i < 4; i++) {
		res = platform_get_resource(pdev, IORESOURCE_MEM, i);
		if (!res)
			return -EINVAL;
		*map[i] = devm_ioremap(dev, res->start, resource_size(res));
		if (!*map[i])
			return -ENOMEM;
	}

	/* not requested either: DECON0 belongs to the panel driver */
	dp->d0 = devm_ioremap(dev, DPU_DECON0, 0x2000);
	dp->d1 = devm_ioremap(dev, DPU_DECON1, 0x1000);
	dp->dma1 = devm_ioremap(dev, DPU_DMA_CH1, 0x1000);
	dp->dpp1 = devm_ioremap(dev, DPU_DPP1, 0x1000);
	if (!dp->d0 || !dp->d1 || !dp->dma1 || !dp->dpp1)
		return -ENOMEM;
	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;

	dp->pmu = syscon_regmap_lookup_by_phandle(dev->of_node, "samsung,pmu-syscon");
	if (IS_ERR(dp->pmu))
		return dev_err_probe(dev, PTR_ERR(dp->pmu), "pmu\n");

	dp->nclks = devm_clk_bulk_get_all(dev, &dp->clks);
	if (dp->nclks < 0)
		return dev_err_probe(dev, dp->nclks, "clocks\n");
	ret = clk_bulk_prepare_enable(dp->nclks, dp->clks);
	if (ret)
		return ret;

	/* the plug's orientation, the SBU-to-AUX switch's select and its enable (high = off) */
	dp->orient = devm_gpiod_get(dev, "orientation", GPIOD_IN);
	dp->sbu_sel = devm_gpiod_get(dev, "sbu-sel", GPIOD_ASIS);
	dp->aux_oe = devm_gpiod_get(dev, "aux-oe", GPIOD_OUT_HIGH);
	if (IS_ERR(dp->orient) || IS_ERR(dp->sbu_sel) || IS_ERR(dp->aux_oe)) {
		ret = IS_ERR(dp->orient) ? PTR_ERR(dp->orient) :
		      IS_ERR(dp->sbu_sel) ? PTR_ERR(dp->sbu_sel) : PTR_ERR(dp->aux_oe);
		free_gpios(dp);
		clk_bulk_disable_unprepare(dp->nclks, dp->clks);
		return dev_err_probe(dev, ret, "gpios\n");
	}
	gpiod_direction_output(dp->sbu_sel, gpiod_get_value(dp->orient));

	dp->dbg = debugfs_create_dir("r8s-dp", NULL);
	debugfs_create_file("ctl", 0600, dp->dbg, dp, &ctl_fops);
	platform_set_drvdata(pdev, dp);
	dev_info(dev, "%d clocks on, orientation %d\n", dp->nclks, gpiod_get_value(dp->orient));

	dp->irq = platform_get_irq(pdev, 0);	/* DECON1's */
	ret = dp->irq < 0 ? dp->irq : dp_drm_init(dp);
	if (ret) {
		debugfs_remove_recursive(dp->dbg);
		free_gpios(dp);
		clk_bulk_disable_unprepare(dp->nclks, dp->clks);
		return dev_err_probe(dev, ret, "DECON1 interrupt or DRM device\n");
	}
	return 0;
}

static void r8s_dp_remove(struct platform_device *pdev)
{
	struct r8s_dp *dp = platform_get_drvdata(pdev);

	drm_kms_helper_poll_fini(&dp->drm);
	drm_dev_unplug(&dp->drm);
	drm_atomic_helper_shutdown(&dp->drm);
	drm_edid_free(dp->edid);
	debugfs_remove_recursive(dp->dbg);
	free_gpios(dp);
	clk_bulk_disable_unprepare(dp->nclks, dp->clks);
}

static const struct of_device_id r8s_dp_of_match[] = {
	{ .compatible = "samsung,exynos990-dp" },
	{ }
};
MODULE_DEVICE_TABLE(of, r8s_dp_of_match);

static struct platform_driver r8s_dp_driver = {
	.probe	= r8s_dp_probe,
	.remove	= r8s_dp_remove,
	.driver	= {
		.name		= "exynos990-dp",
		.of_match_table	= r8s_dp_of_match,
	},
};
module_platform_driver(r8s_dp_driver);

MODULE_DESCRIPTION("Exynos 990 DisplayPort (USB-C DP alt mode) DRM driver");
MODULE_LICENSE("GPL");
