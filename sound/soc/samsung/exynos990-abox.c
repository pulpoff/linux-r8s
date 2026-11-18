// SPDX-License-Identifier: GPL-2.0
//
// Exynos 990 ABOX audio block (v3.0.1), driven without its CA32 firmware.
//
// ABOX normally runs Samsung's "calliope" firmware on two internal CA32 cores
// and the kernel only talks to it over IPC. The data path itself is plain
// memory-mapped hardware, so this driver leaves the cores off and programs
// it from the AP: RDMA8 reads the PCM buffer, SPUS function chain 8 hands it
// to SIFS1 and SIFS1 feeds UAIF2, the DSP_A TDM bus to the two CS35L40 amps
// (earpiece on slot 0, loudspeaker on slot 1). Capture comes the other way:
// the VTS serial LIF clocks the digital microphones into UAIF6, NSRC4 takes
// UAIF6 and WDMA4 writes it to the PCM buffer, as stock routes the main mic.
//
// S-Boot leaves the AUD power domain on with PLL_AUD0 locked at 1179.648 MHz
// and every CMU_AUD gate open. The DMAs go through SysMMU_AUD ("iommus") or,
// without it, use physical addresses; Samsung's EL2 S2MPU lets them through
// either way (its fault detector stays quiet).
//
// Calls need the firmware: the modem's voice DSP (VSS) exchanges the call's
// audio with it over PCIe, through a shared area at 0xF4200000, and nothing
// else on the AP can take its place. With "memory-region" in the DT the driver
// owns SysMMU_AUD through its own domain, maps everything at the addresses the
// firmware has built in, and can boot the cores on request (debugfs
// "firmware"). While the firmware runs, it drives RDMA8 and WDMA4 itself and
// our two PCMs only keep UAIF2, the amps, UAIF6 and the mics powered.

#include <linux/arm-smccc.h>
#include <linux/bitfield.h>
#include <linux/completion.h>
#include <linux/crc32.h>
#include <linux/ctype.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/firmware.h>
#include <linux/hrtimer.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/io-64-nonatomic-lo-hi.h>
#include <linux/iommu.h>
#include <linux/iopoll.h>
#include <linux/mfd/syscon.h>
#include <linux/mutex.h>
#include <linux/sizes.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_reserved_mem.h>
#include <linux/pci.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/sched/clock.h>
#include <linux/workqueue.h>
#include <sound/dmaengine_pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

/* CMU_AUD */
#define CMU_PLL_CON3_AUD0		0x010c
#define CMU_PLL_ENABLE			BIT(31)
#define CMU_PLL_STABLE			BIT(29)
#define CMU_MUX_UAIF(x)			(0x100c + 4 * (x))
#define CMU_MUX_UAIF_AUDIF		0
#define CMU_MUX_UAIF6_LIF_BCLK		1	/* UAIF6 only: the serial LIF's BCLK */
#define CMU_DIV_AUDIF			0x1800
#define CMU_DIV_UAIF(x)			(0x182c + 4 * (x))
#define CMU_DIV_BUSY			BIT(16)
#define CMU_DIV_RATIO			GENMASK(8, 0)

/*
 * PLL_AUD0 runs at 1179.648 MHz (the 48 kHz family). The stock driver sets
 * AUDIF to 384 kHz x 96 x 2 whenever UAIF dividers go up to 512, so every
 * BCLK of a 16/24/32-bit stream divides it evenly.
 */
#define AUD_PLL_RATE			1179648000UL
#define AUD_AUDIF_RATE			73728000UL

/* ABOX SFR */
#define ABOX_IP_INDEX			0x0000
#define ABOX_IP_INDEX_VAL		0x41424f58	/* "ABOX" */
#define ABOX_VERSION			0x0004
#define ABOX_SYSPOWER_CTRL		0x0010
#define ABOX_SYSPOWER_STATUS		0x0014
#define ABOX_QCHANNEL_DISABLE		0x0038
#define ABOX_QCH_BCLK_UAIF(x)		BIT(2 + (x))
#define ABOX_ROUTE_CTRL0		0x0040
#define ABOX_ROUTE_UAIF_SPK(x)		(0xf << (4 * (x)))
#define ABOX_ROUTE_SIFS1		2
#define ABOX_ROUTE_CTRL1		0x0044
#define ABOX_ROUTE_NSRC(x)		(0x1f << (5 * (x)))
#define ABOX_ROUTE_NSRC_UAIF(x)		(8 + (x))
/* SPUM function chains, 4 bits each from bit 4: OUT [3] (0: WDMA), ASRC [0] */
#define ABOX_SPUM_CTRL0			0x0300
#define ABOX_SPUM_CHAIN(x)		(0xf << (4 + 4 * (x)))
#define ABOX_SPUM_SBANK_NSRC(x)		(0x0328 + 4 * (x))
/* SPUS function chains, 8 bits each: IN [6:5], OUT [4:1], ASRC [0] */
#define ABOX_SPUS_CTRL_FC0		0x0200
#define ABOX_FC_CHAIN(x)		(0x7f << (8 * (x)))
#define ABOX_FC_IN_RDMA			0
#define ABOX_FC_OUT_SIFS1		2
#define ABOX_SPUS_CTRL1			0x0204
#define ABOX_SIFS_OUT_SEL(x)		(0xf << (8 + 4 * (x)))	/* SIFS1..5 */
#define ABOX_SBANK_RDMA(x)		(0x0220 + 4 * (x))
#define ABOX_SBANK_MIXP			0x0280
#define ABOX_SIFS_CNT(x)		((x) > 4 ? 0x0288 : (x) > 3 ? 0x028c : \
					 0x0290 + 4 * (x))

#define ABOX_UAIF_CTRL0(x)		(0x0500 + 0x10 * (x))
#define ABOX_UAIF_SPK_ENABLE		BIT(0)
#define ABOX_UAIF_MIC_ENABLE		BIT(1)
#define ABOX_UAIF_MODE_PROVIDER		BIT(2)
#define ABOX_UAIF_CTRL0_RESET		0x01880010
#define ABOX_UAIF_CTRL1(x)		(0x0504 + 0x10 * (x))
#define ABOX_UAIF_FORMAT		GENMASK(28, 24)
#define ABOX_UAIF_BCLK_POLARITY		BIT(23)
#define ABOX_UAIF_WS_MODE		BIT(22)
#define ABOX_UAIF_WS_POLAR		BIT(21)
#define ABOX_UAIF_SLOT_MAX		GENMASK(20, 18)
#define ABOX_UAIF_SBIT_MAX		GENMASK(17, 12)
#define ABOX_UAIF_VALID_STR		GENMASK(11, 6)
#define ABOX_UAIF_VALID_END		GENMASK(5, 0)
#define ABOX_UAIF_STATUS(x)		(0x050c + 0x10 * (x))

#define ABOX_RDMA(x)			(0x1000 + 0x100 * (x))
#define ABOX_WDMA(x)			(0x3000 + 0x100 * (x))
#define ABOX_DMA_CTRL0			0x00
#define ABOX_DMA_ENABLE			BIT(0)
#define ABOX_DMA_BUF_UPDATE		BIT(5)
#define ABOX_DMA_FORMAT			GENMASK(12, 8)	/* (ch - 1) | (bytes - 1) << 3 */
#define ABOX_DMA_BURST_LEN		GENMASK(22, 19)
#define ABOX_DMA_CTRL1			0x04
#define ABOX_DMA_BUF_STR		0x08
#define ABOX_DMA_BUF_END		0x0c
#define ABOX_DMA_BUF_OFFSET		0x10
#define ABOX_DMA_STR_POINT		0x14
#define ABOX_DMA_VOL_FACTOR		0x18
#define ABOX_DMA_VOL_CHANGE		0x1c
#define ABOX_DMA_STATUS			0x30
#define ABOX_DMA_PROGRESS		BIT(31)
/* v3.0.1: the period being read, and the bytes read of it so far */
#define ABOX_DMA_RBUF_OFFSET		GENMASK(27, 20)
#define ABOX_DMA_RBUF_CNT		GENMASK(19, 0)

/* every DMA: RDMA0-11, WDMA0-4, their dual halves, WDMA_DEBUG0-5; ENABLE is bit 0 */
#define ABOX_NR_DMA			(12 + 5 + 5 + 6)
#define ABOX_DMA_STATUS_OFF		0x30

#define ABOX_UAIF_SPK			2	/* UAIF2: the amp TDM bus */
#define ABOX_UAIF_MIC			6	/* UAIF6: the VTS serial LIF */
/*
 * Capture: NSRC4 -> WDMA4, as stock. SPUM_CTRL4 ties NSRC0-3 to the SPUM
 * ASRCs and NSRC4 to none (ID 7), so NSRC4 cannot stall like SPUS chains 0-7.
 */
#define ABOX_NSRC_MIC			4
#define ABOX_WDMA_MIC			4

#define ABOX_BUFFER_BYTES		(256 * 1024)
/* BUF_OFFSET (the period size) is bits [15:4]: a 64 KiB period reads back 0 */
#define ABOX_PERIOD_BYTES_MAX		(32 * 1024)

/*
 * The firmware's address space ("IOVA" = what the cores and DMAs see):
 * below 0x80000000 is SRAM; the rest goes through SysMMU_AUD. The windows
 * are compiled into the firmware (stock abox.h).
 */
#define IOVA_PCM_BUF(stream)		(0x9f000000 + SZ_1M * (stream))	/* ours */
#define IOVA_FW_DRAM			0x80000000
#define IOVA_CHIPID			0x10000000
#define IOVA_RDMA_BUF(x)		(0x91000000 + SZ_1M * (x))
#define IOVA_WDMA_BUF(x)		(0x92000000 + SZ_1M * (x))
#define IOVA_VSS_PCI			0xa2000000	/* the modem's voice area */
#define IOVA_VSS_PCI_DB			0xa3000000	/* the modem's doorbell */
#define IOVA_SLOG			0xe0000000	/* the firmware's "silent log" */
#define SLOG_SIZE			0x2800000	/* written from the first boot on */
#define SLOG_CHUNK			SZ_1M
#define FW_DRAM_SIZE			0x2a00000	/* at PA 0xF8000000 */
#define FW_DRAM_END			0xfaa00000
#define FW_BUF_BYTES			SZ_64K

/* IPC: two queues in the firmware DRAM, AP -> firmware (TX) and back */
#define FW_IPC_TX			0x850000
#define FW_IPC_RX			0x868000
#define FW_IPC_SIZE			0x18000
#define FW_LOG_OFFSET			0xb00000
#define Q_CMD_IDX_S			0x0000
#define Q_CMD_IDX_E			0x0004
#define Q_CMD_ELEM(i)			(0x0008 + 56 * (i))
#define Q_CMD_LEN			128
#define Q_DATA_IDX_S			0x1c08
#define Q_DATA_IDX_E			0x1c0c
#define Q_DATA_LEN			0x1c10
#define Q_DATA_ELEM			0x1c14
#define Q_DATA_SIZE			(FW_IPC_SIZE - Q_DATA_ELEM)	/* 0x163ec */

/* PMU: the two CA32 cores (plain MMIO here; 0x1920 is read only) */
#define PMU_AUD_CPU0_POWER		0x3180
#define PMU_AUD_CPU0_ENABLE		0x31a0
#define PMU_AUD_CPU0_STANDBY		0x31a4
#define PMU_AUD_CPU1_POWER		0x3200
#define PMU_AUD_CPU1_ENABLE		0x3220
#define PMU_AUD_CPU1_STANDBY		0x3224
#define PMU_AUD_PAD			0x1920

#define ABOX_CORE0_PC			0x507c
#define ABOX_CORE1_PC			0x50fc
#define TIMER_PRESET_LSB1		0x628
#define TIMER_CURVALUD_LSB1		0x630

/* ABOX's own GIC-400: core 0 is CPU interface 0, the AP interface 2 */
#define GICD_CTLR			0x000
#define GICD_ISENABLER(n)		(0x100 + 4 * ((n) / 32))
#define GICD_ITARGETSR(n)		(0x800 + ((n) & ~3))
#define GICD_SGIR			0xf00
#define GICC_CTLR			0x00
#define GICC_PMR			0x04
#define GICC_IAR			0x0c
#define GICC_EOIR			0x10
#define GICC_DIR			0x1000
#define ABOX_GIC_AP			2
#define ABOX_SGI_MSG			15
#define ABOX_IRQ_WDT			106

/* CMU_AUD, read only: the cores' clock (+0x1000 is the timer's, not theirs) */
#define CMU_AUD_CPU_MUX			0x1004
#define CMU_AUD_CPU_USER		0x0600
#define CMU_AUD_PLL_DIV			0x1824
#define CMU_AUD_CPU_HCHGEN		0x1028

/* EL3 register access for the secure ABOX GIC (stock exynos_smc SMC_CMD_REG) */
#define SMC_CMD_REG			((unsigned long)(long)-101)
#define SMC_CMD_REG_READ		0xffffff9bUL

enum { IPC_SYSTEM = 1, IPC_PCMPLAYBACK, IPC_PCMCAPTURE, IPC_ABOX_CONFIG = 0xa };
enum {
	ABOX_SUSPEND = 1, ABOX_RESUME, ABOX_BOOT_DONE, ABOX_CHANGE_GEAR,
	ABOX_REQUEST_SYSCLK = 7, ABOX_CHANGED_GEAR = 9, ABOX_REPORT_LOG = 0x10,
	ABOX_SET_MODE = 0x50, ABOX_SET_TYPE = 0x60, ABOX_START_VSS = 0xa0,
	ABOX_STOP_VSS, ABOX_REPORT_COMPONENT = 0xc0, ABOX_UPDATE_COMPONENT_CONTROL,
	ABOX_REQUEST_COMPONENT_CONTROL, ABOX_REPORT_COMPONENT_CONTROL,
	ABOX_REPORT_FAULT = 0xfa,
};
enum {
	PCM_OPEN = 11, PCM_CLOSE = 12, PCM_HW_PARAMS = 14, PCM_HW_FREE = 15,
	PCM_PREPARE = 16, PCM_TRIGGER = 17, PCM_SET_BUFFER = 20,
};
enum { CFG_SIFS1_RATE = 2, CFG_SIFS1_FORMAT = 0x11, CFG_SIFM4_FORMAT = 0x18 };

struct abox_msg_cmd {
	s32 id, cmd, arg[3];
	u32 data_idx;
	u64 send, recv, time_put, time_get;
} __packed;
static_assert(sizeof(struct abox_msg_cmd) == 56);

/* stock ABOX_IPC_MSG: the AArch32 firmware lays it out the same way */
struct abox_ipc_msg {
	s32 ipcid, task_id;
	union {
		struct {
			s32 msgtype, param1, param2, param3;
			union { s32 s32[185]; u64 u64[92]; u8 raw[740]; };
		} sys;
		struct { s32 msgtype, channel_id, param[4]; } pcm;
		struct { s32 msgtype, param1, param2; } cfg;
	};
};
static_assert(offsetof(struct abox_ipc_msg, sys.raw) == 24);
static_assert(sizeof(struct abox_ipc_msg) == 768);

/*
 * The topology's component controls (stock abox_tplg.bin): widgets and
 * muxes under GID_W, the named controls under GID_C. All muxes share one
 * item list: RDMAn n+1, WDMAn 13+n, BD_MIXER 42, TONEGEN_1KHZ 43,
 * VSS_RXBRIDGE 46, VSS_TXADAPTER 47, AUDIO_DAMPER0 50.
 */
#define GID_W				0xab0c
#define GID_C				0xc291
#define CTL_PATH_CHANGE_ID		1792
#define W_RDMA8				124
#define W_RDMA8_A			125
#define W_WDMA4				148
#define W_TONEGEN			224

/*
 * The call, built only from routes stock uses: downlink VSS_RXBRIDGE ->
 * BD_SRCCOM4 -> BD_MIXER -> RDMA8 (our SPUS chain 8 -> SIFS1 -> UAIF2 ->
 * amps); uplink WDMA4 (UAIF6, the mics) -> VSS_TXADAPTER -> VSSIF_TX, with
 * BD_MIXER -> AUDIO_DAMPER0 as the echo reference.
 */
static const u32 abox_call_mux[][2] = {
	{ 215, 46 },	/* BD_SRCCOM4_A = VSS_RXBRIDGE */
	{ 125, 42 },	/* RDMA8_A = BD_MIXER */
	{ 235, 17 },	/* VSS_TXADAPTER_A = WDMA4 */
	{ 244, 42 },	/* AUDIO_DAMPER0_A = BD_MIXER */
	{ 236, 50 },	/* VSS_TXADAPTER_B = AUDIO_DAMPER0 */
	{ 280, 47 },	/* VSSIF_TX_A = VSS_TXADAPTER */
};
/* VSS_RXBRIDGE, BD_SRCCOM4, BD_MIXER, RDMA8, WDMA4, AUDIO_DAMPER0, VSS_TXADAPTER, VSSIF_TX */
static const u32 abox_call_en[] = { 231, 214, 124, 148, 243, 234, 279 };	/* 223: "invalid ctrl" */

/* fallback: report the period timer's position instead of the DMA's */
static bool swpos;
module_param(swpos, bool, 0644);
MODULE_PARM_DESC(swpos, "report the period timer's position, not the DMA's");
/*
 * SPUS chains 0-7 are each tied to ASRC 0-7 (SPUS_CTRL4/5 reset values) and
 * stall, ASRC off or not, while that ASRC is not set up: the RDMA fetches
 * one burst and never moves again. Chains 8-11 have no ASRC and run.
 */
static int rdma_idx = 8;
module_param(rdma_idx, int, 0444);

/*
 * The call's RDMA8 format. The firmware's BD mixer writes S24 in 32-bit words
 * (8 bytes a stereo frame, the voice in both channels): as 32-bit samples
 * that plays 48 dB down, as 16-bit it is noise that follows the speech.
 * Verified by ear on a call 11-18 22:30: 24-bit memory format (CTRL0 format
 * 0x11) and a 32-bit DMA output (BIT_CTRL0 DST_BIT_WIDTH 3, SIFS1 is 32).
 */
static int fw_rdma_bits = 24;
module_param(fw_rdma_bits, int, 0644);
MODULE_PARM_DESC(fw_rdma_bits, "RDMA8 sample width under the firmware: 24 (the mixer's), 16 or 32");

/* RDMA8's CTRL0 format for that width: (ch - 1) | width code << 3 */
static u32 abox_fw_rdma_format(void)
{
	switch (fw_rdma_bits) {
	case 16:
		return 1 | 1 << 3;
	case 32:
		return 1 | 3 << 3;
	default:
		return 1 | 2 << 3;
	}
}
MODULE_PARM_DESC(rdma_idx, "RDMA used for playback (8-11 have no ASRC), at probe");

#define ABOX_DMA_SBANK_LIMIT		0x20
/* BIT_CTRL0 [1:0]: DST_BIT_WIDTH, the sample width handed to SPUS (bytes - 1) */
#define ABOX_DMA_BIT_CTRL0		0x24

struct abox {
	struct device *dev;
	void __iomem *sfr;
	void __iomem *cmu;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_active;
	struct pinctrl_state *pins_idle;

	/* indexed by stream: RDMA for playback, WDMA for capture */
	struct abox_dma {
		struct abox *abox;
		u32 base;
		struct snd_pcm_substream *substream;
		struct hrtimer timer;
		ktime_t period;
		unsigned int sw_pos;	/* bytes, when the DMA reports nothing */
		u32 format;
		/* the firmware owns the DMA: only keep the path powered */
		bool keepalive;
		u32 saved_ctrl[3];	/* CTRL0, CTRL1, BIT_CTRL0 */	/* CTRL0 (no ENABLE), CTRL1 before a boot */
	} dma[2];

	unsigned int fmt[7];		/* per UAIF */
	spinlock_t dma_lock;		/* DMA enable against a firmware takeover */

	struct device *card_dev;	/* allocates buffers: it has no IOMMU */
	struct pinctrl_state *pins_pci;	/* optional: stock's aud-i2s4-pci */
	struct mutex fw_lock;		/* boot, stop and the debugfs commands */

	struct abox_fw {
		struct iommu_domain *domain;	/* ours: fixed IOVAs */
		bool pcm_mapped[2];
		void *mem;			/* firmware DRAM, write-combined */
		phys_addr_t dram_pa;
		void *pci;			/* the modem's voice area */
		phys_addr_t pci_pa;
		size_t pci_size;
		u32 *pci_crc;			/* per 4 KiB page, for debugfs "pci" */
		phys_addr_t db_pa;		/* the modem's doorbell, when mapped */
		void __iomem *sram, *timer, *gicd, *gicc;
		phys_addr_t gicd_pa, gicc_pa;
		size_t sram_size;
		struct regmap *pmu;
		int irq;
		bool secure;			/* GIC through EL3 (chip rev > 0) */
		bool running, vss;
		bool stuck;			/* a core never halted: keep everything */
		enum { ROUTE_NONE, ROUTE_TONE, ROUTE_LOOP, ROUTE_CALL } route;
		spinlock_t tx_lock;
		struct mutex rx_lock;
		struct completion boot_done, ctl_done;
		u32 version, rx_count, tx_count, faults, log_addr, peek, irqs;
		s32 fault[3], ctl_reply[8];
		struct work_struct wdt_work;
		struct delayed_work rdma_work;	/* keeps the call's RDMA8 running */
		bool rdma_watch;
		unsigned int rdma_kicks;
		struct abox_fw_buf {		/* firmware-owned DMA buffers */
			void *cpu;
			dma_addr_t pa;
			u32 iova;
		} buf[2];			/* RDMA8, WDMA4 */
		void *slog[SLOG_SIZE / SLOG_CHUNK];
		dma_addr_t slog_pa[SLOG_SIZE / SLOG_CHUNK];
	} fw;
};

static inline u32 abox_readl(struct abox *abox, u32 reg)
{
	return readl(abox->sfr + reg);
}

static inline void abox_writel(struct abox *abox, u32 reg, u32 val)
{
	writel(val, abox->sfr + reg);
}

static void abox_update(struct abox *abox, u32 reg, u32 mask, u32 val)
{
	abox_writel(abox, reg, (abox_readl(abox, reg) & ~mask) | (val & mask));
}

static u32 abox_dma_ctrl_reg(int i)
{
	if (i < 12)
		return ABOX_RDMA(i);
	if (i < 17)
		return ABOX_WDMA(i - 12);
	if (i < 22)
		return ABOX_WDMA(i - 17) + 0x80;	/* the dual half */
	return 0x3800 + 0x100 * (i - 22);		/* WDMA_DEBUG */
}

/* which DMAs run (a bit per abox_dma_ctrl_reg() index) */
static u32 abox_dmas_running(struct abox *abox)
{
	u32 on = 0;
	int i;

	for (i = 0; i < ABOX_NR_DMA; i++)
		if (abox_readl(abox, abox_dma_ctrl_reg(i)) & ABOX_DMA_ENABLE)
			on |= BIT(i);
	return on;
}

/*
 * Stop every running DMA but those in keep. Once ABOX suspends, SysMMU_AUD
 * stops translating: a DMA left running then writes physical memory.
 */
static u32 abox_dmas_stop(struct abox *abox, u32 keep)
{
	u32 on = abox_dmas_running(abox) & ~keep, val;
	int i;

	for (i = 0; i < ABOX_NR_DMA; i++) {
		u32 reg = abox_dma_ctrl_reg(i);

		if (!(on & BIT(i)))
			continue;
		abox_update(abox, reg, ABOX_DMA_ENABLE, 0);
		readl_poll_timeout_atomic(abox->sfr + reg + ABOX_DMA_STATUS_OFF, val,
					  !(val & ABOX_DMA_PROGRESS), 10, 20000);
	}
	return on;
}

static int abox_cmu_div(struct abox *abox, u32 reg, u32 ratio)
{
	u32 val;

	writel((readl(abox->cmu + reg) & ~CMU_DIV_RATIO) | ratio,
	       abox->cmu + reg);
	return readl_poll_timeout_atomic(abox->cmu + reg, val,
					 !(val & CMU_DIV_BUSY), 1, 1000);
}

static int abox_cmu_init(struct abox *abox)
{
	u32 pll = readl(abox->cmu + CMU_PLL_CON3_AUD0);

	if ((pll & (CMU_PLL_ENABLE | CMU_PLL_STABLE)) !=
	    (CMU_PLL_ENABLE | CMU_PLL_STABLE)) {
		dev_err(abox->dev, "PLL_AUD0 is not running: 0x%08x\n", pll);
		return -EIO;
	}

	/* UAIF2 BCLK from DIV_CLK_AUD_UAIF2 (AUDIF), not from the pad */
	writel(0, abox->cmu + CMU_MUX_UAIF(ABOX_UAIF_SPK));
	return abox_cmu_div(abox, CMU_DIV_AUDIF,
			    AUD_PLL_RATE / AUD_AUDIF_RATE - 1);
}

static int abox_hw_init(struct abox *abox)
{
	u32 val;
	int i, ret;

	ret = abox_cmu_init(abox);
	if (ret)
		return ret;

	val = abox_readl(abox, ABOX_IP_INDEX);
	if (val != ABOX_IP_INDEX_VAL) {
		dev_err(abox->dev, "unexpected IP index 0x%08x\n", val);
		return -ENODEV;
	}

	/* the firmware's modem lines (gpb2 function 3) stay up while it runs */
	ret = pinctrl_select_state(abox->pinctrl,
				   abox->fw.running && abox->pins_pci ?
				   abox->pins_pci : abox->pins_active);
	if (ret)
		return ret;

	abox_writel(abox, ABOX_SYSPOWER_CTRL, 1);
	ret = readl_poll_timeout(abox->sfr + ABOX_SYSPOWER_STATUS, val,
				 val & 1, 10, 10000);
	if (ret) {
		dev_err(abox->dev, "SYSPOWER did not come up\n");
		return ret;
	}

	/* SRAM banks: 512 bytes per RDMA and the mixer, as the S10e uses */
	for (i = 0; i < 12; i++)
		abox_writel(abox, ABOX_SBANK_RDMA(i), (512 << 16) | (i * 512));
	abox_writel(abox, ABOX_SBANK_MIXP, (512 << 16) | 0x3000);

	/*
	 * RDMAn -> SPUS chain n (no ASRC) -> SIFS1 (SPUS OUTn) -> UAIF2; any
	 * other chain that points at SIFS1 is cut. Chains 0-3 sit in FC0,
	 * 4-7 in FC1, 8-11 in FC2, 8 bits each.
	 */
	for (i = 0; i < 12; i++) {
		u32 reg = i < 4 ? ABOX_SPUS_CTRL_FC0 : i < 8 ? 0x0218 : 0x021c;
		u32 chain = (ABOX_FC_IN_RDMA << 5) | (ABOX_FC_OUT_SIFS1 << 1);

		if (i == rdma_idx)
			abox_update(abox, reg, ABOX_FC_CHAIN(i % 4),
				    chain << (8 * (i % 4)));
		else if (((abox_readl(abox, reg) >> (8 * (i % 4))) & 0x1e) ==
			 (ABOX_FC_OUT_SIFS1 << 1))
			abox_update(abox, reg, 0x1e << (8 * (i % 4)), 0);
	}
	/* SIFS1 takes SPUS OUTn, n = the chain */
	abox_update(abox, ABOX_SPUS_CTRL1, ABOX_SIFS_OUT_SEL(1),
		    rdma_idx << __ffs(ABOX_SIFS_OUT_SEL(1)));
	abox_update(abox, ABOX_ROUTE_CTRL0, ABOX_ROUTE_UAIF_SPK(ABOX_UAIF_SPK),
		    ABOX_ROUTE_SIFS1 << (4 * ABOX_UAIF_SPK));

	/* UAIF6 -> NSRC4 -> SIFM4 -> WDMA4, no ASRC; stock's SRAM bank */
	abox_writel(abox, ABOX_SPUM_SBANK_NSRC(ABOX_NSRC_MIC),
		    (512 << 16) | (ABOX_NSRC_MIC * 512));
	abox_update(abox, ABOX_SPUM_CTRL0, ABOX_SPUM_CHAIN(ABOX_NSRC_MIC), 0);
	abox_update(abox, ABOX_ROUTE_CTRL1, ABOX_ROUTE_NSRC(ABOX_NSRC_MIC),
		    ABOX_ROUTE_NSRC_UAIF(ABOX_UAIF_MIC) << (5 * ABOX_NSRC_MIC));
	return 0;
}

static int abox_runtime_resume(struct device *dev)
{
	return abox_hw_init(dev_get_drvdata(dev));
}

static int abox_runtime_suspend(struct device *dev)
{
	struct abox *abox = dev_get_drvdata(dev);
	u32 on = abox_dmas_stop(abox, 0);

	if (on)
		dev_warn(dev, "DMAs %#x still ran at suspend: stopped\n", on);

	pinctrl_select_state(abox->pinctrl, abox->pins_idle);
	abox_writel(abox, ABOX_SYSPOWER_CTRL, 0);
	return 0;
}

/* UAIF2: the amps, playback (ABOX clocks). UAIF6: the mics (VTS clocks) */

static int abox_uaif_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	struct abox *abox = snd_soc_dai_get_drvdata(dai);
	unsigned int clk = fmt & SND_SOC_DAIFMT_CLOCK_PROVIDER_MASK;

	switch (fmt & SND_SOC_DAIFMT_FORMAT_MASK) {
	case SND_SOC_DAIFMT_DSP_A:
	case SND_SOC_DAIFMT_I2S:
		break;
	default:
		return -EINVAL;
	}

	/* UAIF2 drives the amps' clocks; UAIF6 takes the serial LIF's */
	if (dai->id == ABOX_UAIF_MIC ? clk != SND_SOC_DAIFMT_BC_FC :
				       clk != SND_SOC_DAIFMT_BP_FP)
		return -EINVAL;

	abox->fmt[dai->id] = fmt;
	return 0;
}

static int abox_uaif_hw_params(struct snd_pcm_substream *substream,
			       struct snd_pcm_hw_params *params,
			       struct snd_soc_dai *dai)
{
	struct abox *abox = snd_soc_dai_get_drvdata(dai);
	unsigned int id = dai->id;
	unsigned int fmt = abox->fmt[id];
	bool provider = (fmt & SND_SOC_DAIFMT_CLOCK_PROVIDER_MASK) ==
			SND_SOC_DAIFMT_BP_FP;
	unsigned int ch = params_channels(params);
	unsigned int width = params_physical_width(params);
	unsigned long bclk = params_rate(params) * ch * width;
	unsigned int code;
	u32 ctrl1;
	int ret;

	switch (width) {
	case 16:
		code = 1;
		break;
	case 24:
		code = 2;
		break;
	case 32:
		code = 3;
		break;
	default:
		return -EINVAL;
	}

	if (provider) {
		if (AUD_AUDIF_RATE % bclk || AUD_AUDIF_RATE / bclk > 512) {
			dev_err(abox->dev, "BCLK %lu Hz not reachable\n", bclk);
			return -EINVAL;
		}
		writel(CMU_MUX_UAIF_AUDIF, abox->cmu + CMU_MUX_UAIF(id));
		ret = abox_cmu_div(abox, CMU_DIV_UAIF(id),
				   AUD_AUDIF_RATE / bclk - 1);
	} else {
		/* clocked by the serial LIF's BCLK, undivided */
		writel(CMU_MUX_UAIF6_LIF_BCLK, abox->cmu + CMU_MUX_UAIF(id));
		ret = abox_cmu_div(abox, CMU_DIV_UAIF(id), 0);
	}
	if (ret)
		return ret;

	ctrl1 = FIELD_PREP(ABOX_UAIF_FORMAT, (ch - 1) | (code << 3)) |
		FIELD_PREP(ABOX_UAIF_SLOT_MAX, ch - 1) |
		FIELD_PREP(ABOX_UAIF_SBIT_MAX, width - 1);
	if ((fmt & SND_SOC_DAIFMT_FORMAT_MASK) == SND_SOC_DAIFMT_DSP_A)
		ctrl1 |= ABOX_UAIF_WS_MODE |
			 FIELD_PREP(ABOX_UAIF_VALID_STR, width - 1) |
			 FIELD_PREP(ABOX_UAIF_VALID_END, width - 1);
	/* as the stock driver maps it: normal BCLK sets the polarity bit */
	switch (fmt & SND_SOC_DAIFMT_INV_MASK) {
	case SND_SOC_DAIFMT_NB_NF:
		ctrl1 |= ABOX_UAIF_BCLK_POLARITY;
		break;
	case SND_SOC_DAIFMT_IB_NF:
		break;
	default:
		return -EINVAL;
	}

	abox_writel(abox, ABOX_UAIF_CTRL1(id), ctrl1);
	abox_writel(abox, ABOX_UAIF_CTRL0(id), ABOX_UAIF_CTRL0_RESET |
		    (provider ? ABOX_UAIF_MODE_PROVIDER : 0));
	if (substream->stream == SNDRV_PCM_STREAM_PLAYBACK)
		abox_writel(abox, ABOX_SIFS_CNT(1), 0);
	abox_update(abox, ABOX_QCHANNEL_DISABLE, ABOX_QCH_BCLK_UAIF(id),
		    ABOX_QCH_BCLK_UAIF(id));

	dev_dbg(abox->dev, "UAIF%u %u ch x %u bit, BCLK %lu Hz%s, CTRL1 0x%08x\n",
		id, ch, width, bclk, provider ? "" : " (in)", ctrl1);
	return 0;
}

static int abox_uaif_trigger(struct snd_pcm_substream *substream, int cmd,
			     struct snd_soc_dai *dai)
{
	struct abox *abox = snd_soc_dai_get_drvdata(dai);
	u32 en = substream->stream == SNDRV_PCM_STREAM_PLAYBACK ?
		 ABOX_UAIF_SPK_ENABLE : ABOX_UAIF_MIC_ENABLE;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		abox_update(abox, ABOX_UAIF_CTRL0(dai->id), en, en);
		return 0;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		abox_update(abox, ABOX_UAIF_CTRL0(dai->id), en, 0);
		return 0;
	}
	return -EINVAL;
}

static int abox_uaif_hw_free(struct snd_pcm_substream *substream,
			     struct snd_soc_dai *dai)
{
	struct abox *abox = snd_soc_dai_get_drvdata(dai);

	abox_update(abox, ABOX_QCHANNEL_DISABLE, ABOX_QCH_BCLK_UAIF(dai->id), 0);
	return 0;
}

static const struct snd_soc_dai_ops abox_uaif_ops = {
	.set_fmt	= abox_uaif_set_fmt,
	.hw_params	= abox_uaif_hw_params,
	.trigger	= abox_uaif_trigger,
	.hw_free	= abox_uaif_hw_free,
};

static struct snd_soc_dai_driver abox_dais[] = {
	{
		.name = "abox-uaif2",
		.id = ABOX_UAIF_SPK,
		.playback = {
			.stream_name = "UAIF2 Playback",
			.channels_min = 2,
			.channels_max = 4,
			.rates = SNDRV_PCM_RATE_48000,
			.formats = SNDRV_PCM_FMTBIT_S32_LE,
		},
		.ops = &abox_uaif_ops,
	},
	{
		.name = "abox-uaif6",
		.id = ABOX_UAIF_MIC,
		.capture = {
			.stream_name = "UAIF6 Capture",
			.channels_min = 2,
			.channels_max = 2,	/* see exynos990-vts-slif.c */
			.rates = SNDRV_PCM_RATE_48000,
			.formats = SNDRV_PCM_FMTBIT_S16_LE,
		},
		.ops = &abox_uaif_ops,
	},
};

/* PCM: RDMA8 playback, WDMA4 capture */

static const struct snd_pcm_hardware abox_pcm_hw = {
	.info = SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_MMAP_VALID |
		SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_BLOCK_TRANSFER,
	.formats = SNDRV_PCM_FMTBIT_S32_LE,
	.rates = SNDRV_PCM_RATE_48000,
	.rate_min = 48000,
	.rate_max = 48000,
	.channels_min = 2,
	.channels_max = 4,
	.buffer_bytes_max = ABOX_BUFFER_BYTES,
	.period_bytes_min = 1024,
	.period_bytes_max = ABOX_PERIOD_BYTES_MAX,
	.periods_min = 2,
	.periods_max = 32,
};

static const struct snd_pcm_hardware abox_pcm_hw_capture = {
	.info = SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_MMAP_VALID |
		SNDRV_PCM_INFO_INTERLEAVED | SNDRV_PCM_INFO_BLOCK_TRANSFER,
	.formats = SNDRV_PCM_FMTBIT_S16_LE,
	.rates = SNDRV_PCM_RATE_48000,
	.rate_min = 48000,
	.rate_max = 48000,
	.channels_min = 2,
	.channels_max = 2,
	.buffer_bytes_max = ABOX_BUFFER_BYTES,
	.period_bytes_min = 1024,
	.period_bytes_max = ABOX_PERIOD_BYTES_MAX,
	.periods_min = 2,
	.periods_max = 32,
};

static u32 abox_dma_read(struct abox_dma *dma, u32 reg)
{
	return abox_readl(dma->abox, dma->base + reg);
}

static void abox_dma_write(struct abox_dma *dma, u32 reg, u32 val)
{
	abox_writel(dma->abox, dma->base + reg, val);
}

static enum hrtimer_restart abox_period_timer(struct hrtimer *timer)
{
	struct abox_dma *dma = container_of(timer, struct abox_dma, timer);
	struct snd_pcm_substream *substream = READ_ONCE(dma->substream);

	if (!substream)
		return HRTIMER_NORESTART;

	dma->sw_pos += snd_pcm_lib_period_bytes(substream);
	dma->sw_pos %= snd_pcm_lib_buffer_bytes(substream);
	snd_pcm_period_elapsed(substream);
	hrtimer_forward_now(timer, dma->period);
	return HRTIMER_RESTART;
}

static int abox_pcm_open(struct snd_soc_component *component,
			 struct snd_pcm_substream *substream)
{
	struct abox *abox = snd_soc_component_get_drvdata(component);
	int ret;

	ret = pm_runtime_resume_and_get(abox->dev);
	if (ret)
		return ret;

	snd_soc_set_runtime_hwparams(substream,
				     substream->stream == SNDRV_PCM_STREAM_PLAYBACK ?
				     &abox_pcm_hw : &abox_pcm_hw_capture);
	/* DMA addresses are 16-byte units; keep periods aligned */
	snd_pcm_hw_constraint_step(substream->runtime, 0,
				   SNDRV_PCM_HW_PARAM_PERIOD_BYTES, 64);
	snd_pcm_hw_constraint_step(substream->runtime, 0,
				   SNDRV_PCM_HW_PARAM_BUFFER_BYTES, 64);
	/* RBUF_OFFSET counts periods in 8 bits */
	snd_pcm_hw_constraint_minmax(substream->runtime,
				     SNDRV_PCM_HW_PARAM_PERIODS, 2, 32);
	return 0;
}

static int abox_pcm_close(struct snd_soc_component *component,
			  struct snd_pcm_substream *substream)
{
	struct abox *abox = snd_soc_component_get_drvdata(component);

	pm_runtime_mark_last_busy(abox->dev);
	pm_runtime_put_autosuspend(abox->dev);
	return 0;
}

static int abox_pcm_hw_params(struct snd_soc_component *component,
			      struct snd_pcm_substream *substream,
			      struct snd_pcm_hw_params *params)
{
	struct abox *abox = snd_soc_component_get_drvdata(component);
	struct abox_dma *dma = &abox->dma[substream->stream];

	dma->format = (params_channels(params) - 1) |
		      ((params_physical_width(params) / 8 - 1) << 3);
	dma->period = ns_to_ktime(div_u64((u64)NSEC_PER_SEC *
					  params_period_size(params),
					  params_rate(params)));
	return 0;
}

static int abox_pcm_prepare(struct snd_soc_component *component,
			    struct snd_pcm_substream *substream)
{
	struct abox *abox = snd_soc_component_get_drvdata(component);
	struct abox_dma *dma = &abox->dma[substream->stream];
	/* with our own SysMMU domain the buffer sits at a fixed IOVA */
	dma_addr_t addr = abox->fw.domain ? IOVA_PCM_BUF(substream->stream) :
			  substream->runtime->dma_addr;
	size_t bytes = snd_pcm_lib_buffer_bytes(substream);
	u32 ctrl0;

	dma->sw_pos = 0;
	/* the firmware owns the DMA: this stream only keeps the path powered */
	WRITE_ONCE(dma->keepalive, READ_ONCE(abox->fw.running));
	if (dma->keepalive)
		return 0;
	if (abox->fw.domain && !abox->fw.pcm_mapped[substream->stream])
		return -EIO;

	/* END is exclusive, OFFSET is the period size, STR_POINT the start */
	ctrl0 = abox_dma_read(dma, ABOX_DMA_CTRL0) & ~ABOX_DMA_ENABLE;
	ctrl0 = (ctrl0 & ~ABOX_DMA_FORMAT) |
		FIELD_PREP(ABOX_DMA_FORMAT, dma->format) | ABOX_DMA_BUF_UPDATE;
	abox_dma_write(dma, ABOX_DMA_CTRL0, ctrl0);
	abox_dma_write(dma, ABOX_DMA_BUF_STR, lower_32_bits(addr));
	abox_dma_write(dma, ABOX_DMA_BUF_END, lower_32_bits(addr + bytes));
	abox_dma_write(dma, ABOX_DMA_BUF_OFFSET,
		       snd_pcm_lib_period_bytes(substream));
	abox_dma_write(dma, ABOX_DMA_STR_POINT, lower_32_bits(addr));
	abox_dma_write(dma, ABOX_DMA_VOL_FACTOR, 0x800000);
	if (substream->stream == SNDRV_PCM_STREAM_CAPTURE)
		abox_dma_write(dma, ABOX_DMA_VOL_CHANGE, 0x10000);
	/* v3: the sample width between the DMA and SPUS/SPUM; reset 0 (8 bit) */
	abox_dma_write(dma, ABOX_DMA_BIT_CTRL0,
		       FIELD_GET(GENMASK(4, 3), dma->format));
	return 0;
}

static int abox_pcm_trigger(struct snd_soc_component *component,
			    struct snd_pcm_substream *substream, int cmd)
{
	struct abox *abox = snd_soc_component_get_drvdata(component);
	struct abox_dma *dma = &abox->dma[substream->stream];
	u32 val;

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
	case SNDRV_PCM_TRIGGER_RESUME:
	case SNDRV_PCM_TRIGGER_PAUSE_RELEASE:
		WRITE_ONCE(dma->substream, substream);
		spin_lock(&abox->dma_lock);
		if (!dma->keepalive)
			abox_dma_write(dma, ABOX_DMA_CTRL0,
				       abox_dma_read(dma, ABOX_DMA_CTRL0) |
				       ABOX_DMA_ENABLE);
		spin_unlock(&abox->dma_lock);
		hrtimer_start(&dma->timer, dma->period, HRTIMER_MODE_REL);
		return 0;
	case SNDRV_PCM_TRIGGER_STOP:
	case SNDRV_PCM_TRIGGER_SUSPEND:
	case SNDRV_PCM_TRIGGER_PAUSE_PUSH:
		hrtimer_try_to_cancel(&dma->timer);
		WRITE_ONCE(dma->substream, NULL);
		if (READ_ONCE(dma->keepalive))
			return 0;
		spin_lock(&abox->dma_lock);
		abox_dma_write(dma, ABOX_DMA_CTRL0,
			       abox_dma_read(dma, ABOX_DMA_CTRL0) &
			       ~ABOX_DMA_ENABLE);
		spin_unlock(&abox->dma_lock);
		/* the DMA drains its burst before PROGRESS drops */
		readl_poll_timeout_atomic(abox->sfr + dma->base +
					  ABOX_DMA_STATUS, val,
					  !(val & ABOX_DMA_PROGRESS), 10, 40000);
		return 0;
	}
	return -EINVAL;
}

static snd_pcm_uframes_t abox_pcm_pointer(struct snd_soc_component *component,
					  struct snd_pcm_substream *substream)
{
	struct abox *abox = snd_soc_component_get_drvdata(component);
	struct abox_dma *dma = &abox->dma[substream->stream];
	size_t period = snd_pcm_lib_period_bytes(substream);
	u32 status = abox_dma_read(dma, ABOX_DMA_STATUS);
	unsigned int pos;

	/*
	 * v3.0.1: RBUF_OFFSET is the index of the period being played and
	 * RBUF_CNT the 16-byte units moved within it (measured: 20 ms periods
	 * of 7680 bytes count 0..0x1df, the index 0..3 for four periods).
	 */
	if (!swpos && !READ_ONCE(dma->keepalive) && (status & ABOX_DMA_PROGRESS))
		pos = FIELD_GET(ABOX_DMA_RBUF_OFFSET, status) * period +
		      (FIELD_GET(ABOX_DMA_RBUF_CNT, status) << 4);
	else
		pos = dma->sw_pos;

	return bytes_to_frames(substream->runtime,
			       pos % snd_pcm_lib_buffer_bytes(substream));
}

static int abox_pcm_new(struct snd_soc_component *component,
			struct snd_soc_pcm_runtime *rtd)
{
	struct abox *abox = snd_soc_component_get_drvdata(component);
	/*
	 * With our SysMMU domain the DMA API is gone from abox->dev: allocate
	 * from the card's device (no IOMMU, so the address is physical) and
	 * map each buffer once at its fixed IOVA.
	 */
	struct device *bdev = abox->fw.domain ? rtd->card->dev : abox->dev;
	int dir, ret;

	abox->card_dev = rtd->card->dev;
	snd_pcm_set_managed_buffer_all(rtd->pcm, SNDRV_DMA_TYPE_DEV, bdev,
				       ABOX_BUFFER_BYTES, ABOX_BUFFER_BYTES);
	if (!abox->fw.domain)
		return 0;

	for_each_pcm_streams(dir) {
		struct snd_pcm_substream *ss = rtd->pcm->streams[dir].substream;

		if (!ss)
			continue;
		if (!ss->dma_buffer.area || ss->dma_buffer.bytes < ABOX_BUFFER_BYTES ||
		    abox->fw.pcm_mapped[dir])
			return -ENOMEM;
		ret = iommu_map(abox->fw.domain, IOVA_PCM_BUF(dir), ss->dma_buffer.addr,
				ABOX_BUFFER_BYTES, IOMMU_READ | IOMMU_WRITE, GFP_KERNEL);
		if (ret)
			return ret;
		abox->fw.pcm_mapped[dir] = true;
	}
	return 0;
}

static void abox_pcm_free(struct snd_soc_component *component,
			  struct snd_pcm *pcm)
{
	struct abox *abox = snd_soc_component_get_drvdata(component);
	int dir;

	if (!abox->fw.domain)
		return;
	for_each_pcm_streams(dir) {
		if (!pcm->streams[dir].substream || !abox->fw.pcm_mapped[dir])
			continue;
		iommu_unmap(abox->fw.domain, IOVA_PCM_BUF(dir), ABOX_BUFFER_BYTES);
		abox->fw.pcm_mapped[dir] = false;
	}
}

/* Firmware: our SysMMU domain */

static int abox_iommu_fault(struct iommu_domain *d, struct device *dev,
			    unsigned long iova, int flags, void *token)
{
	struct abox *abox = token;

	abox->fw.faults++;
	dev_err_ratelimited(dev, "SysMMU_AUD fault @0x%08lx %s (PC %08x %08x)\n",
			    iova, flags & IOMMU_FAULT_WRITE ? "write" : "read",
			    abox_readl(abox, ABOX_CORE0_PC),
			    abox_readl(abox, ABOX_CORE1_PC));
	return 0;	/* anything else panics (exynos-iommu) */
}

static void abox_fw_domain_free(void *data)
{
	struct abox *abox = data;

	if (abox->fw.stuck) {
		/* detaching puts SysMMU_AUD in bypass under a running core */
		dev_crit(abox->dev, "the cores never halted: the SysMMU domain is leaked\n");
		return;
	}
	iommu_detach_device(abox->fw.domain, abox->dev);
	iommu_domain_free(abox->fw.domain);
}

static int abox_fw_attach(struct abox *abox)
{
	struct device *dev = abox->dev;
	struct iommu_domain *d;
	int ret;

	if (!device_iommu_mapped(dev) ||
	    !of_property_present(dev->of_node, "memory-region"))
		return 0;

	d = iommu_paging_domain_alloc(dev);
	if (IS_ERR(d))
		return dev_err_probe(dev, PTR_ERR(d), "no SysMMU domain\n");
	ret = iommu_attach_device(d, dev);
	if (ret) {
		iommu_domain_free(d);
		return dev_err_probe(dev, ret, "SysMMU attach\n");
	}
	abox->fw.domain = d;
	iommu_set_fault_handler(d, abox_iommu_fault, abox);
	return devm_add_action_or_reset(dev, abox_fw_domain_free, abox);
}

/* Firmware: the ABOX GIC, through EL3 on a secure part */

static bool abox_smc_ok(struct abox *abox, phys_addr_t pa)
{
	return (pa >= abox->fw.gicd_pa && pa < abox->fw.gicd_pa + SZ_4K) ||
	       (pa >= abox->fw.gicc_pa && pa < abox->fw.gicc_pa + SZ_8K);
}

static void abox_smc_wr(struct abox *abox, phys_addr_t pa, u32 val)
{
	struct arm_smccc_res r;

	if (WARN_ON_ONCE(!abox_smc_ok(abox, pa)))
		return;
	arm_smccc_smc(SMC_CMD_REG, 0x40000000 | (u32)(pa >> 2), val,
		      0, 0, 0, 0, 0, &r);
	if ((long)r.a0 < 0)
		dev_err_ratelimited(abox->dev, "SMC write %pa: %ld\n", &pa, (long)r.a0);
}

static u32 abox_smc_rd(struct abox *abox, phys_addr_t pa)
{
	struct arm_smccc_res r;

	if (WARN_ON_ONCE(!abox_smc_ok(abox, pa)))
		return 0;
	arm_smccc_smc(SMC_CMD_REG_READ, 0xc0000000 | (u32)(pa >> 2), 0,
		      0, 0, 0, 0, 0, &r);
	if (r.a0) {
		dev_err_ratelimited(abox->dev, "SMC read %pa: %ld\n", &pa, (long)r.a0);
		return 0;
	}
	return r.a2;
}

static u32 abox_gicd_rd(struct abox *abox, u32 off)
{
	return abox->fw.secure ? abox_smc_rd(abox, abox->fw.gicd_pa + off) :
				 readl(abox->fw.gicd + off);
}

static void abox_gicd_wr(struct abox *abox, u32 off, u32 val)
{
	if (abox->fw.secure)
		abox_smc_wr(abox, abox->fw.gicd_pa + off, val);
	else
		writel(val, abox->fw.gicd + off);
}

static u32 abox_gicc_rd(struct abox *abox, u32 off)
{
	return abox->fw.secure ? abox_smc_rd(abox, abox->fw.gicc_pa + off) :
				 readl(abox->fw.gicc + off);
}

static void abox_gicc_wr(struct abox *abox, u32 off, u32 val)
{
	if (abox->fw.secure)
		abox_smc_wr(abox, abox->fw.gicc_pa + off, val);
	else
		writel(val, abox->fw.gicc + off);
}

/* SGI 15 to core 0: always a plain write, as stock */
static void abox_gic_sgi(struct abox *abox)
{
	writel(BIT(16) | ABOX_SGI_MSG, abox->fw.gicd + GICD_SGIR);
}

static void abox_gic_route_ap(struct abox *abox, u32 irq)
{
	u32 sh = (irq & 3) * 8, v = abox_gicd_rd(abox, GICD_ITARGETSR(irq));

	v = (v & ~(0xff << sh)) | (BIT(ABOX_GIC_AP) << sh);
	abox_gicd_wr(abox, GICD_ITARGETSR(irq), v);
	abox_gicd_wr(abox, GICD_ISENABLER(irq), BIT(irq % 32));
}

/* Firmware: IPC */

static int abox_fw_send(struct abox *abox, const struct abox_ipc_msg *m,
			s32 cmd, s32 a0, s32 a1, s32 a2, bool sync)
{
	void *q = abox->fw.mem + FW_IPC_TX;
	s32 *ds = q + Q_DATA_IDX_S, *de = q + Q_DATA_IDX_E;
	s32 *cs = q + Q_CMD_IDX_S, *ce = q + Q_CMD_IDX_E;
	struct abox_msg_cmd c = { .id = m->ipcid, .cmd = cmd, .arg = { a0, a1, a2 } };
	u32 len, size = 4 + sizeof(*m);
	unsigned long flags;
	s32 s, e, blank;
	int ret = 0;
	u64 end;

	if (!READ_ONCE(abox->fw.running))
		return -EIO;
	len = READ_ONCE(*(u32 *)(q + Q_DATA_LEN));
	if (len != Q_DATA_SIZE)
		return -EIO;			/* never "% 0" */

	spin_lock_irqsave(&abox->fw.tx_lock, flags);
	end = local_clock() + NSEC_PER_MSEC;
	for (;;) {				/* stock abox_msg.c is_avail() */
		s = READ_ONCE(*ds);
		e = READ_ONCE(*de);
		if ((u32)s >= len || (u32)e >= len) {
			ret = -EIO;
			goto out;
		}
		if (s > e) {
			blank = s - e;
		} else {
			blank = len - e;
			if (blank < size) {
				WRITE_ONCE(*de, 0);
				e = 0;
				blank = s;
			}
		}
		if (blank >= size)
			break;
		if (local_clock() > end) {
			ret = -EBUSY;
			goto out;
		}
		cpu_relax();
	}
	*(s32 *)(q + Q_DATA_ELEM + e) = sizeof(*m);
	memcpy(q + Q_DATA_ELEM + e + 4, m, sizeof(*m));
	WRITE_ONCE(*de, (e + size) % len);
	c.data_idx = e;
	c.time_put = local_clock();
	for (;;) {
		s = READ_ONCE(*ce);
		if ((u32)s >= Q_CMD_LEN) {
			ret = -EIO;
			goto out;
		}
		if (READ_ONCE(*cs) != (s + 1) % Q_CMD_LEN)
			break;
		if (local_clock() > end) {
			ret = -EBUSY;
			goto out;
		}
		cpu_relax();
	}
	memcpy(q + Q_CMD_ELEM(s), &c, sizeof(c));
	mb();
	WRITE_ONCE(*ce, (s + 1) % Q_CMD_LEN);
	abox->fw.tx_count++;
out:
	spin_unlock_irqrestore(&abox->fw.tx_lock, flags);
	if (ret) {
		dev_err_ratelimited(abox->dev, "IPC %d/%#x not sent: %d\n", m->ipcid, cmd, ret);
		return ret;
	}
	abox_gic_sgi(abox);
	if (!sync)
		return 0;
	end = local_clock() + 20 * NSEC_PER_MSEC;	/* stock flush: 20 ms */
	while (READ_ONCE(*cs) != READ_ONCE(*ce)) {
		if (local_clock() > end) {
			dev_err_ratelimited(abox->dev, "IPC %d/%#x not taken\n", m->ipcid, cmd);
			return -ETIMEDOUT;
		}
		udelay(100);
		abox_gic_sgi(abox);
	}
	return 0;
}

static int abox_fw_sys(struct abox *abox, s32 type, s32 p1, s32 p2, s32 p3,
		       const void *bundle, size_t blen, bool sync)
{
	struct abox_ipc_msg m = { .ipcid = IPC_SYSTEM };

	m.sys.msgtype = type;
	m.sys.param1 = p1;
	m.sys.param2 = p2;
	m.sys.param3 = p3;
	if (bundle)
		memcpy(m.sys.raw, bundle, min(blen, sizeof(m.sys.raw)));
	return abox_fw_send(abox, &m, type, p1, p2, p3, sync);
}

static int abox_fw_cfg(struct abox *abox, s32 type, s32 p1, s32 p2)
{
	struct abox_ipc_msg m = { .ipcid = IPC_ABOX_CONFIG };

	m.cfg.msgtype = type;
	m.cfg.param1 = p1;
	m.cfg.param2 = p2;
	return abox_fw_send(abox, &m, type, p1, p2, 0, false);
}

static int abox_fw_ctl(struct abox *abox, u32 gid, u32 id, const s32 *v, int n)
{
	return abox_fw_sys(abox, ABOX_UPDATE_COMPONENT_CONTROL, gid, id, 0,
			   v, n * sizeof(*v), false);
}

static int abox_fw_ctl1(struct abox *abox, u32 gid, u32 id, s32 v)
{
	return abox_fw_ctl(abox, gid, id, &v, 1);
}

static int abox_fw_pcm(struct abox *abox, bool cap, u32 ch, s32 type,
		       s32 p0, s32 p1, s32 p2, s32 p3, bool sync)
{
	struct abox_ipc_msg m = {
		.ipcid = cap ? IPC_PCMCAPTURE : IPC_PCMPLAYBACK,
		.task_id = ch,
	};

	m.pcm.msgtype = type;
	m.pcm.channel_id = ch;
	m.pcm.param[0] = p0;
	m.pcm.param[1] = p1;
	m.pcm.param[2] = p2;
	m.pcm.param[3] = p3;
	return abox_fw_send(abox, &m, type, ch, 0, 0, sync);
}

/* the cores' clock as CMU_AUD has it (0: not from the PLL or the oscillator) */
static u32 abox_cpu_hz(struct abox *abox)
{
	if (!(readl(abox->cmu + CMU_AUD_CPU_MUX) & 1))
		return AUD_PLL_RATE / ((readl(abox->cmu + CMU_AUD_PLL_DIV) & 0xf) + 1);
	if (!(readl(abox->cmu + CMU_AUD_CPU_USER) & BIT(4)))
		return 26000000;
	return 0;
}

/* the ABOX timer in ns (26 MHz) */
static u64 abox_atime(struct abox *abox)
{
	return div_u64(lo_hi_readq(abox->fw.timer + TIMER_CURVALUD_LSB1) * 500, 13);
}

static void abox_fw_handle(struct abox *abox, const struct abox_ipc_msg *m)
{
	static const u32 gear_khz[] = { 1180000, 800000, 590000, 394000, 0 };
	s32 p1 = m->sys.param1;
	u32 hz, ns;
	u64 t;

	if (m->ipcid != IPC_SYSTEM) {
		dev_dbg(abox->dev, "PCM msg %d/%d ch %d\n", m->ipcid, m->pcm.msgtype,
			m->pcm.channel_id);
		return;
	}
	switch (m->sys.msgtype) {
	case ABOX_BOOT_DONE:
		abox->fw.version = m->sys.param3;
		complete(&abox->fw.boot_done);
		break;
	case ABOX_CHANGE_GEAR:
		/* no DVFS here: tell it the clock it actually runs at */
		hz = abox_cpu_hz(abox);
		if (!hz)
			hz = p1 > 12 ? p1 * 1000 :
			     p1 >= 1 && p1 <= 5 ? gear_khz[p1 - 1] * 1000 : 0;
		t = sched_clock();
		ns = do_div(t, NSEC_PER_SEC);
		abox_fw_sys(abox, ABOX_CHANGED_GEAR, hz, t, ns, NULL, 0, false);
		dev_dbg(abox->dev, "gear %d (id %#x): %u Hz\n", p1, m->sys.param2, hz);
		break;
	case ABOX_REQUEST_SYSCLK:
		break;
	case ABOX_REPORT_LOG:
		abox->fw.log_addr = m->sys.param2;
		dev_info(abox->dev, "firmware log %d at %#x\n", p1, m->sys.param2);
		break;
	case ABOX_REPORT_COMPONENT_CONTROL:
		memcpy(abox->fw.ctl_reply, m->sys.s32, sizeof(abox->fw.ctl_reply));
		complete(&abox->fw.ctl_done);
		break;
	case ABOX_REPORT_FAULT:
		abox->fw.fault[0] = p1;
		abox->fw.fault[1] = m->sys.param2;
		abox->fw.fault[2] = m->sys.param3;
		dev_err(abox->dev, "calliope fault %#x %#x %#x (PC %08x %08x)\n",
			p1, m->sys.param2, m->sys.param3,
			abox_readl(abox, ABOX_CORE0_PC), abox_readl(abox, ABOX_CORE1_PC));
		break;
	default:
		dev_info_ratelimited(abox->dev, "calliope sys msg %#x %#x %#x %#x\n",
				     m->sys.msgtype, p1, m->sys.param2, m->sys.param3);
	}
}

static void abox_fw_rx(struct abox *abox)
{
	void *q = abox->fw.mem + FW_IPC_RX;
	struct abox_msg_cmd c;
	struct abox_ipc_msg m;
	s32 s, e, n;
	u32 len;

	mutex_lock(&abox->fw.rx_lock);
	len = READ_ONCE(*(u32 *)(q + Q_DATA_LEN));
	if (!len || len > Q_DATA_SIZE) {
		dev_err_ratelimited(abox->dev, "RX queue length %u\n", len);
		goto out;
	}
	for (;;) {
		s = READ_ONCE(*(s32 *)(q + Q_CMD_IDX_S));
		e = READ_ONCE(*(s32 *)(q + Q_CMD_IDX_E));
		if (s == e || (u32)s >= Q_CMD_LEN || (u32)e >= Q_CMD_LEN)
			break;
		rmb();
		memcpy(&c, q + Q_CMD_ELEM(s), sizeof(c));
		n = c.data_idx + 4 <= len ?
		    READ_ONCE(*(s32 *)(q + Q_DATA_ELEM + c.data_idx)) : -1;
		if (n >= 0 && c.data_idx + 4 + n <= len) {
			memset(&m, 0, sizeof(m));
			memcpy(&m, q + Q_DATA_ELEM + c.data_idx + 4,
			       min_t(u32, n, sizeof(m)));
			WRITE_ONCE(*(s32 *)(q + Q_DATA_IDX_S),
				   (c.data_idx + 4 + n) % len);
		} else {
			dev_err_ratelimited(abox->dev, "bad RX record %u/%d (cmd %#x)\n",
					    c.data_idx, n, c.cmd);
			n = -1;
		}
		mb();
		WRITE_ONCE(*(s32 *)(q + Q_CMD_IDX_S), (s + 1) % Q_CMD_LEN);
		abox->fw.rx_count++;
		if (n >= 0)
			abox_fw_handle(abox, &m);
	}
out:
	mutex_unlock(&abox->fw.rx_lock);
}

static irqreturn_t abox_gic_thread(int irq, void *data)
{
	struct abox *abox = data;
	bool msg = false;
	int n;

	abox->fw.irqs++;
	for (n = 0; n < 64; n++) {
		u32 iar = abox_gicc_rd(abox, GICC_IAR), id = iar & 0x3ff;

		if (id >= 1021)
			break;
		abox_gicc_wr(abox, GICC_EOIR, iar);
		if (id < 16)
			abox_gicc_wr(abox, GICC_DIR, iar);
		if (id == ABOX_SGI_MSG)
			msg = true;
		else if (id == ABOX_IRQ_WDT)
			schedule_work(&abox->fw.wdt_work);
		else
			dev_dbg(abox->dev, "ABOX GIC irq %u\n", id);
	}
	/* the queue is the truth: drain it even if the GIC said nothing */
	if (msg || !n)
		abox_fw_rx(abox);
	return IRQ_HANDLED;
}

/* Firmware: load, boot, stop */

static const struct abox_fw_file {
	const char *name;
	u32 off;
	bool sram, required;
} abox_fw_files[] = {
	{ "calliope_sram.bin",   0x000000, true,  true },
	{ "calliope_dram.bin",   0x000000, false, true },
	{ "calliope_sram_2.bin", 0x032000, true,  true },	/* core 1 */
	{ "calliope_dram_2.bin", 0x280000, false, true },
	/* speech enhancement, loaded when present (stock DTBO offsets) */
	{ "rxse.bin",            0x1e4f000, false, false },
	{ "txse1.bin",           0x1800000, false, false },
	{ "txse2.bin",           0x19e4000, false, false },
};

static int abox_fw_download(struct abox *abox)
{
	int i, ret;

	memset(abox->fw.mem, 0, FW_DRAM_SIZE);
	memset_io(abox->fw.sram, 0, abox->fw.sram_size);
	for (i = 0; i < ARRAY_SIZE(abox_fw_files); i++) {
		const struct abox_fw_file *f = &abox_fw_files[i];
		size_t max = f->sram ? abox->fw.sram_size : FW_DRAM_SIZE;
		const struct firmware *fw;

		ret = request_firmware_direct(&fw, f->name, abox->dev);
		if (ret) {
			if (f->required) {
				dev_err(abox->dev, "no %s: %d\n", f->name, ret);
				return ret;
			}
			continue;
		}
		if (f->off + fw->size > max) {
			release_firmware(fw);
			return -EFBIG;
		}
		if (f->sram) {
			size_t n = fw->size & ~3;
			u32 tail = 0;

			__iowrite32_copy(abox->fw.sram + f->off, fw->data, n / 4);
			if (fw->size & 3) {
				memcpy(&tail, fw->data + n, fw->size & 3);
				writel(tail, abox->fw.sram + f->off + n);
			}
		} else {
			memcpy(abox->fw.mem + f->off, fw->data, fw->size);
		}
		dev_dbg(abox->dev, "%s: %zu bytes at %s+%#x\n", f->name, fw->size,
			f->sram ? "SRAM" : "DRAM", f->off);
		release_firmware(fw);
	}
	/* stock sets q_data.len once at probe; the wipe above cleared it */
	*(u32 *)(abox->fw.mem + FW_IPC_TX + Q_DATA_LEN) = Q_DATA_SIZE;
	*(u32 *)(abox->fw.mem + FW_IPC_RX + Q_DATA_LEN) = Q_DATA_SIZE;
	wmb();
	return 0;
}

/*
 * Stock's watchdog order: power, then enable on the way up; enable (the
 * core reset) first on the way down. Clearing only the power bit waits for
 * the cores to reach WFI: a core that never got SUSPEND keeps running, and
 * once SysMMU_AUD suspends it writes physical memory.
 */
static void abox_fw_cores(struct abox *abox, bool on)
{
	if (on) {
		regmap_update_bits(abox->fw.pmu, PMU_AUD_CPU0_POWER, 1, 1);
		regmap_update_bits(abox->fw.pmu, PMU_AUD_CPU1_POWER, 1, 1);
		regmap_update_bits(abox->fw.pmu, PMU_AUD_CPU0_ENABLE, 1, 1);
		regmap_update_bits(abox->fw.pmu, PMU_AUD_CPU1_ENABLE, 1, 1);
		return;
	}
	regmap_update_bits(abox->fw.pmu, PMU_AUD_CPU0_ENABLE, 1, 0);
	regmap_update_bits(abox->fw.pmu, PMU_AUD_CPU1_ENABLE, 1, 0);
	udelay(10);
	regmap_update_bits(abox->fw.pmu, PMU_AUD_CPU0_POWER, 1, 0);
	regmap_update_bits(abox->fw.pmu, PMU_AUD_CPU1_POWER, 1, 0);
}

/* both PCs unchanged over 1 ms: the cores are in reset (or at least WFI) */
static bool abox_fw_halted(struct abox *abox)
{
	u32 pc0 = abox_readl(abox, ABOX_CORE0_PC);
	u32 pc1 = abox_readl(abox, ABOX_CORE1_PC);
	int i;

	for (i = 0; i < 10; i++) {
		udelay(100);
		if (abox_readl(abox, ABOX_CORE0_PC) != pc0 ||
		    abox_readl(abox, ABOX_CORE1_PC) != pc1)
			return false;
	}
	return true;
}

static void abox_fw_state(struct abox *abox, const char *when);
static void abox_fw_restore(struct abox *abox);

/*
 * The cores are off: release what boot took. If they still run, keep ABOX
 * (and so SysMMU_AUD) resumed and every mapping in place: a leak beats a
 * DMA master loose on physical memory.
 */
static void abox_fw_release(struct abox *abox, const char *when)
{
	if (!abox_fw_halted(abox)) {
		abox->fw.stuck = true;
		abox_fw_state(abox, when);
		dev_err(abox->dev, "%s: the cores still run; ABOX stays powered, mappings stay\n",
			when);
		return;
	}
	abox_fw_restore(abox);
	pinctrl_select_state(abox->pinctrl, abox->pins_active);
	pm_runtime_put(abox->dev);
}

static void abox_fw_state(struct abox *abox, const char *when)
{
	u32 p[7] = {};

	regmap_read(abox->fw.pmu, PMU_AUD_CPU0_POWER, &p[0]);
	regmap_read(abox->fw.pmu, PMU_AUD_CPU0_ENABLE, &p[1]);
	regmap_read(abox->fw.pmu, PMU_AUD_CPU0_STANDBY, &p[2]);
	regmap_read(abox->fw.pmu, PMU_AUD_CPU1_POWER, &p[3]);
	regmap_read(abox->fw.pmu, PMU_AUD_CPU1_ENABLE, &p[4]);
	regmap_read(abox->fw.pmu, PMU_AUD_CPU1_STANDBY, &p[5]);
	regmap_read(abox->fw.pmu, PMU_AUD_PAD, &p[6]);
	dev_info(abox->dev, "%s: PMU cpu0 %x/%x/%x cpu1 %x/%x/%x pad %x; CMU cpu mux %08x user %08x plldiv %08x hchgen %08x (%u Hz); timer preset1 %08x; PC %08x %08x\n",
		 when, p[0], p[1], p[2], p[3], p[4], p[5], p[6],
		 readl(abox->cmu + CMU_AUD_CPU_MUX), readl(abox->cmu + CMU_AUD_CPU_USER),
		 readl(abox->cmu + CMU_AUD_PLL_DIV), readl(abox->cmu + CMU_AUD_CPU_HCHGEN),
		 abox_cpu_hz(abox), readl(abox->fw.timer + TIMER_PRESET_LSB1),
		 abox_readl(abox, ABOX_CORE0_PC), abox_readl(abox, ABOX_CORE1_PC));
}

/* stop our own use of a DMA: the firmware programs it from now on */
static void abox_dma_takeover(struct abox *abox, int dir)
{
	struct abox_dma *dma = &abox->dma[dir];
	unsigned long flags;
	u32 val;

	spin_lock_irqsave(&abox->dma_lock, flags);
	dma->saved_ctrl[0] = abox_dma_read(dma, ABOX_DMA_CTRL0) & ~ABOX_DMA_ENABLE;
	dma->saved_ctrl[1] = abox_dma_read(dma, ABOX_DMA_CTRL1);
	dma->saved_ctrl[2] = abox_dma_read(dma, ABOX_DMA_BIT_CTRL0);
	WRITE_ONCE(dma->keepalive, true);
	abox_dma_write(dma, ABOX_DMA_CTRL0, dma->saved_ctrl[0]);
	spin_unlock_irqrestore(&abox->dma_lock, flags);
	readl_poll_timeout(abox->sfr + dma->base + ABOX_DMA_STATUS, val,
			   !(val & ABOX_DMA_PROGRESS), 10, 40000);
}

/* pd_aud never goes down here: undo what the firmware owned, then ours */
static void abox_fw_restore(struct abox *abox)
{
	u32 on = abox_dmas_stop(abox, 0);
	int i;

	if (on)
		dev_info(abox->dev, "stopped the firmware's DMAs %#x\n", on);

	for (i = 0; i < 2; i++) {
		struct abox_dma *dma = &abox->dma[i];
		u32 val;

		abox_dma_write(dma, ABOX_DMA_CTRL0, dma->saved_ctrl[0]);
		readl_poll_timeout(abox->sfr + dma->base + ABOX_DMA_STATUS, val,
				   !(val & ABOX_DMA_PROGRESS), 10, 40000);
		abox_dma_write(dma, ABOX_DMA_CTRL1, dma->saved_ctrl[1]);
		abox_dma_write(dma, ABOX_DMA_BIT_CTRL0, dma->saved_ctrl[2]);
	}
	abox_hw_init(abox);
}

/*
 * The silent log: stock carves 40 MiB out at boot (abox-slog) and maps it
 * at 0xE0000000; the firmware writes there before it reports BOOT_DONE,
 * and every write is a SysMMU fault without it. 1 MiB coherent chunks from
 * the card's (IOMMU-less, 32-bit) device.
 */
static void abox_fw_slog_free(struct abox *abox)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(abox->fw.slog); i++) {
		if (!abox->fw.slog[i])
			continue;
		iommu_unmap(abox->fw.domain, IOVA_SLOG + i * SLOG_CHUNK, SLOG_CHUNK);
		dma_free_coherent(abox->card_dev, SLOG_CHUNK, abox->fw.slog[i],
				  abox->fw.slog_pa[i]);
		abox->fw.slog[i] = NULL;
	}
}

static int abox_fw_slog_alloc(struct abox *abox)
{
	int i, ret;

	if (!abox->card_dev)
		return -ENODEV;
	for (i = 0; i < ARRAY_SIZE(abox->fw.slog); i++) {
		dma_addr_t pa;
		void *cpu;

		if (abox->fw.slog[i])
			continue;
		cpu = dma_alloc_coherent(abox->card_dev, SLOG_CHUNK, &pa,
					 GFP_KERNEL | __GFP_NOWARN);
		if (!cpu) {
			dev_err(abox->dev, "silent log: no memory for chunk %d\n", i);
			abox_fw_slog_free(abox);
			return -ENOMEM;
		}
		ret = iommu_map(abox->fw.domain, IOVA_SLOG + i * SLOG_CHUNK, pa,
				SLOG_CHUNK, IOMMU_READ | IOMMU_WRITE, GFP_KERNEL);
		if (ret) {
			dma_free_coherent(abox->card_dev, SLOG_CHUNK, cpu, pa);
			abox_fw_slog_free(abox);
			return ret;
		}
		abox->fw.slog[i] = cpu;
		abox->fw.slog_pa[i] = pa;
	}
	return 0;
}

static int abox_fw_boot(struct abox *abox)
{
	u32 pc[4][2], p0 = 0, p1 = 0;
	bool booted = false;
	u64 b[2];
	int i, ret;

	if (!abox->fw.sram)
		return -ENODEV;
	if (abox->fw.running)
		return -EALREADY;
	if (abox->fw.stuck)
		return -EBUSY;		/* a core never halted: reboot first */
	ret = abox_fw_slog_alloc(abox);
	if (ret)
		return ret;
	ret = pm_runtime_resume_and_get(abox->dev);	/* SYSPOWER (DRAM on) */
	if (ret)
		return ret;
	/*
	 * Stock's abox-pci puts gpb2-0..3 in function 3 ("pci_on") at probe and
	 * never back: the modem's lines into the firmware. Ours, while it runs.
	 */
	if (abox->pins_pci && pinctrl_select_state(abox->pinctrl, abox->pins_pci))
		dev_warn(abox->dev, "no pci pins: the modem cannot signal the firmware\n");
	abox_fw_state(abox, "before boot");
	regmap_read(abox->fw.pmu, PMU_AUD_CPU0_POWER, &p0);
	regmap_read(abox->fw.pmu, PMU_AUD_CPU1_POWER, &p1);
	if ((p0 | p1) & 1) {
		dev_warn(abox->dev, "the cores were on: off first\n");
		abox_fw_cores(abox, false);
		usleep_range(1000, 2000);
	}
	for (i = 0; i < 2; i++)
		abox_dma_takeover(abox, i);

	/* stock abox_enable() order */
	abox->fw.irqs = 0;
	enable_irq(abox->fw.irq);
	writel(1, abox->fw.gicd + GICD_CTLR);		/* plain, as stock */
	abox_gic_route_ap(abox, ABOX_IRQ_WDT);
	abox_gicc_wr(abox, GICC_PMR, 0xff);
	abox_gicd_wr(abox, GICD_CTLR, 3);
	abox_gicc_wr(abox, GICC_CTLR, 3);
	ret = abox_fw_download(abox);
	if (ret)
		goto err;

	reinit_completion(&abox->fw.boot_done);
	abox->fw.rx_count = abox->fw.tx_count = 0;
	abox->fw.version = 0;
	abox->fw.log_addr = 0;
	abox->fw.vss = false;
	abox->fw.route = ROUTE_NONE;
	memset(abox->fw.fault, 0, sizeof(abox->fw.fault));
	WRITE_ONCE(abox->fw.running, true);
	abox_fw_cores(abox, true);
	for (i = 0; i < 4; i++) {
		udelay(20);
		pc[i][0] = abox_readl(abox, ABOX_CORE0_PC);
		pc[i][1] = abox_readl(abox, ABOX_CORE1_PC);
	}
	dev_info(abox->dev, "PC0 %08x %08x %08x %08x PC1 %08x %08x %08x %08x\n",
		 pc[0][0], pc[1][0], pc[2][0], pc[3][0],
		 pc[0][1], pc[1][1], pc[2][1], pc[3][1]);

	/* 500 ms as stock; poll the queue too, in case SGI 15 never reaches us */
	for (i = 0; i < 50 && !booted; i++) {
		/* the wait consumes the completion: keep its answer */
		booted = wait_for_completion_timeout(&abox->fw.boot_done,
						     msecs_to_jiffies(10));
		if (!booted)
			abox_fw_rx(abox);
	}
	if (!booted) {
		dev_err(abox->dev, "no BOOT_DONE: rx q %d/%d tx q %d/%d, %u irqs, last SRAM word %08x\n",
			*(s32 *)(abox->fw.mem + FW_IPC_RX + Q_CMD_IDX_S),
			*(s32 *)(abox->fw.mem + FW_IPC_RX + Q_CMD_IDX_E),
			*(s32 *)(abox->fw.mem + FW_IPC_TX + Q_CMD_IDX_S),
			*(s32 *)(abox->fw.mem + FW_IPC_TX + Q_CMD_IDX_E),
			abox->fw.irqs, readl(abox->fw.sram + abox->fw.sram_size - 4));
		abox_fw_state(abox, "boot failed");
		ret = -ETIMEDOUT;
		goto err_cores;
	}
	dev_info(abox->dev, "Calliope is ready to sing (version %08x \"%c%c%c%c\", %u irqs)\n",
		 abox->fw.version,
		 isprint(abox->fw.version >> 24) ? abox->fw.version >> 24 : '.',
		 isprint((abox->fw.version >> 16) & 0xff) ? (abox->fw.version >> 16) & 0xff : '.',
		 isprint((abox->fw.version >> 8) & 0xff) ? (abox->fw.version >> 8) & 0xff : '.',
		 isprint(abox->fw.version & 0xff) ? abox->fw.version & 0xff : '.',
		 abox->fw.irqs);

	b[0] = sched_clock();
	b[1] = abox_atime(abox);
	ret = abox_fw_sys(abox, ABOX_RESUME, 0, 0, 0, b, sizeof(b), true);
	if (ret)
		dev_err(abox->dev, "RESUME: %d\n", ret);
	return 0;

err_cores:
	abox_fw_cores(abox, false);
	WRITE_ONCE(abox->fw.running, false);
	disable_irq(abox->fw.irq);
	abox_fw_release(abox, "boot failed");
	return ret;
err:
	WRITE_ONCE(abox->fw.running, false);
	disable_irq(abox->fw.irq);
	abox_fw_restore(abox);
	pinctrl_select_state(abox->pinctrl, abox->pins_active);
	pm_runtime_put(abox->dev);
	return ret;
}

static void abox_fw_hangup(struct abox *abox);

static void abox_fw_stop(struct abox *abox)
{
	int r1, r2, r3;
	u64 b[2];
	u32 v;

	if (!abox->fw.running)
		return;
	abox_fw_hangup(abox);
	b[0] = sched_clock();
	b[1] = abox_atime(abox);
	abox_fw_sys(abox, ABOX_SUSPEND, 0, 0, 0, b, sizeof(b), true);
	/* "ABOX enters WFI", then both cores in standby */
	r1 = readl_poll_timeout(abox->fw.timer + TIMER_PRESET_LSB1, v,
				v == 0xfffffffe, 100, 100000);
	r2 = regmap_read_poll_timeout(abox->fw.pmu, PMU_AUD_CPU0_STANDBY, v,
				      (v & 0x11) == 0x11, 100, 1000000);
	r3 = regmap_read_poll_timeout(abox->fw.pmu, PMU_AUD_CPU1_STANDBY, v,
				      v & 1, 100, 1000000);
	if (r1 || r2 || r3)
		abox_fw_state(abox, "stop: no WFI/standby");
	abox_fw_cores(abox, false);
	WRITE_ONCE(abox->fw.running, false);
	disable_irq(abox->fw.irq);
	abox_fw_state(abox, "stopped");
	abox_fw_release(abox, "stop");
}

static void abox_fw_wdt_work(struct work_struct *work)
{
	struct abox *abox = container_of(work, struct abox, fw.wdt_work);

	dev_err(abox->dev, "calliope watchdog: PC %08x %08x, last fault %#x %#x %#x, last SRAM word %08x\n",
		abox_readl(abox, ABOX_CORE0_PC), abox_readl(abox, ABOX_CORE1_PC),
		abox->fw.fault[0], abox->fw.fault[1], abox->fw.fault[2],
		readl(abox->fw.sram + abox->fw.sram_size - 4));
	mutex_lock(&abox->fw_lock);
	abox_fw_stop(abox);
	mutex_unlock(&abox->fw_lock);
}

/* Firmware: the routes (call, test tone, mic loop) */

static int abox_fw_bufs(struct abox *abox)
{
	static const u32 iova[2] = { IOVA_RDMA_BUF(8), IOVA_WDMA_BUF(4) };
	int i, ret;

	if (!abox->card_dev)
		return -ENODEV;
	for (i = 0; i < 2; i++) {
		struct abox_fw_buf *b = &abox->fw.buf[i];

		if (b->cpu)
			continue;
		b->cpu = dma_alloc_coherent(abox->card_dev, FW_BUF_BYTES, &b->pa,
					    GFP_KERNEL);
		if (!b->cpu)
			return -ENOMEM;
		ret = iommu_map(abox->fw.domain, iova[i], b->pa, FW_BUF_BYTES,
				IOMMU_READ | IOMMU_WRITE, GFP_KERNEL);
		if (ret) {
			dma_free_coherent(abox->card_dev, FW_BUF_BYTES, b->cpu, b->pa);
			b->cpu = NULL;
			return ret;
		}
		b->iova = iova[i];
	}
	return 0;
}

static void abox_fw_bufs_free(struct abox *abox)
{
	int i;

	for (i = 0; i < 2; i++) {
		struct abox_fw_buf *b = &abox->fw.buf[i];

		if (!b->cpu)
			continue;
		iommu_unmap(abox->fw.domain, b->iova, FW_BUF_BYTES);
		dma_free_coherent(abox->card_dev, FW_BUF_BYTES, b->cpu, b->pa);
		b->cpu = NULL;
	}
}

/* stock abox_rdma/abox_wdma order: OPEN, SET_BUFFER, HW_PARAMS */
static void abox_fw_dma_open(struct abox *abox, bool cap)
{
	u32 ch = cap ? 4 : 8;

	u32 bits = cap ? 16 : fw_rdma_bits;
	u32 frame = 2 * (bits == 16 ? 2 : 4);	/* 2 ch, S24 sits in 32 bits */

	abox_fw_pcm(abox, cap, ch, PCM_OPEN, 0, 0, 0, 0, false);
	/* RDMA8 480 frames (10 ms) a period; WDMA4 S16 2 ch 240 frames */
	abox_fw_pcm(abox, cap, ch, PCM_SET_BUFFER, abox->fw.buf[cap].iova,
		    cap ? 960 : 480 * frame, 4, 0, false);
	abox_fw_pcm(abox, cap, ch, PCM_HW_PARAMS, 48000, bits, 2, 0, false);
	/*
	 * BIT_CTRL0 DST_BIT_WIDTH, the width the DMA hands to SPUS/takes from
	 * SPUM, is the AP's (the firmware sets only the CTRL0 format; takeover
	 * saved ours): 32 into SIFS1, which the call route sets to 32; 16 for
	 * WDMA4, the stock default.
	 */
	abox_dma_write(&abox->dma[cap], ABOX_DMA_BIT_CTRL0, cap ? 1 : 3);
}

static int abox_fw_dma_go(struct abox *abox, bool cap)
{
	u32 ch = cap ? 4 : 8;

	abox_fw_pcm(abox, cap, ch, PCM_PREPARE, 0, 0, 0, 0, false);
	return abox_fw_pcm(abox, cap, ch, PCM_TRIGGER, 1, 0, 0, 0, true);
}

/*
 * The firmware's BD mixer restarts its output when the modem's voice comes up
 * ("bd_mixer_start_device"), after our PCM trigger, and leaves RDMA8 stopped
 * ("rdma-8 update with stopped dma"). While a call is routed: re-send the
 * trigger, and if that does not take, enable the DMA (its buffer is the
 * firmware's, behind SysMMU).
 */
static void abox_fw_rdma_work(struct work_struct *work)
{
	struct abox *abox = container_of(work, struct abox, fw.rdma_work.work);
	struct abox_dma *dma = &abox->dma[SNDRV_PCM_STREAM_PLAYBACK];
	unsigned long flags;
	u32 ctrl0;

	if (!READ_ONCE(abox->fw.rdma_watch) || !READ_ONCE(abox->fw.running))
		return;
	ctrl0 = abox_dma_read(dma, ABOX_DMA_CTRL0);
	if (FIELD_GET(ABOX_DMA_FORMAT, ctrl0) != abox_fw_rdma_format() &&
	    abox_dma_read(dma, ABOX_DMA_BUF_STR) == IOVA_RDMA_BUF(8)) {
		spin_lock_irqsave(&abox->dma_lock, flags);
		ctrl0 = abox_dma_read(dma, ABOX_DMA_CTRL0);
		abox_dma_write(dma, ABOX_DMA_CTRL0, (ctrl0 & ~ABOX_DMA_FORMAT) |
			       FIELD_PREP(ABOX_DMA_FORMAT, abox_fw_rdma_format()));
		spin_unlock_irqrestore(&abox->dma_lock, flags);
		dev_info_ratelimited(abox->dev, "RDMA8 format %#lx under the call: set %#x\n",
				     FIELD_GET(ABOX_DMA_FORMAT, ctrl0), abox_fw_rdma_format());
	}
	if (!(abox_dma_read(dma, ABOX_DMA_CTRL0) & ABOX_DMA_ENABLE)) {
		abox->fw.rdma_kicks++;
		abox_fw_pcm(abox, false, 8, PCM_TRIGGER, 1, 0, 0, 0, true);
		usleep_range(5000, 6000);
		if (!(abox_dma_read(dma, ABOX_DMA_CTRL0) & ABOX_DMA_ENABLE) &&
		    abox_dma_read(dma, ABOX_DMA_BUF_STR) == IOVA_RDMA_BUF(8)) {
			spin_lock_irqsave(&abox->dma_lock, flags);
			abox_dma_write(dma, ABOX_DMA_CTRL0,
				       abox_dma_read(dma, ABOX_DMA_CTRL0) | ABOX_DMA_ENABLE);
			spin_unlock_irqrestore(&abox->dma_lock, flags);
			dev_info_ratelimited(abox->dev, "RDMA8 stopped under the call: enabled it (kick %u)\n",
					     abox->fw.rdma_kicks);
		} else {
			dev_info_ratelimited(abox->dev, "RDMA8 stopped under the call: re-triggered (kick %u)\n",
					     abox->fw.rdma_kicks);
		}
	}
	if (READ_ONCE(abox->fw.rdma_watch))
		schedule_delayed_work(&abox->fw.rdma_work, msecs_to_jiffies(50));
}

static int abox_fw_route(struct abox *abox, int route, bool speaker)
{
	int i, ret;

	if (!abox->fw.running)
		return -ENODEV;
	if (abox->fw.route)
		return -EBUSY;
	if (rdma_idx != 8)
		return -EINVAL;		/* the firmware's downlink is RDMA8 */
	ret = abox_fw_bufs(abox);
	if (ret)
		return ret;
	memset(abox->fw.buf[0].cpu, 0, FW_BUF_BYTES);
	memset(abox->fw.buf[1].cpu, 0, FW_BUF_BYTES);
	abox->fw.route = route;

	switch (route) {
	case ROUTE_TONE:	/* a 1 kHz tone from the firmware to RDMA8 */
		abox_fw_ctl1(abox, GID_W, W_RDMA8_A, 43);
		abox_fw_dma_open(abox, false);
		abox_fw_ctl1(abox, GID_W, W_TONEGEN, 1);
		abox_fw_ctl1(abox, GID_W, W_RDMA8, 1);
		return abox_fw_dma_go(abox, false);
	case ROUTE_LOOP:	/* the mics (WDMA4) straight back to RDMA8 */
		abox_fw_ctl1(abox, GID_W, W_RDMA8_A, 17);
		abox_fw_dma_open(abox, false);
		abox_fw_dma_open(abox, true);
		abox_fw_ctl1(abox, GID_W, W_WDMA4, 1);
		abox_fw_ctl1(abox, GID_W, W_RDMA8, 1);
		abox_fw_dma_go(abox, true);
		return abox_fw_dma_go(abox, false);
	}

	/* the call: the stock abox_cmpnt_restore subset, then the graph */
	abox_fw_cfg(abox, CFG_SIFS1_RATE, 48000, 0);
	abox_fw_cfg(abox, CFG_SIFS1_FORMAT, 32, 2);
	/* this firmware rejects SIFM4_RATE (9) and ASRC_FACTOR_CP (0x30) */
	abox_fw_cfg(abox, CFG_SIFM4_FORMAT, 16, 2);
	if (!abox->fw.vss) {
		abox_fw_sys(abox, ABOX_START_VSS, 0, 0, 0, NULL, 0, false);
		abox->fw.vss = true;
	}
	abox_fw_sys(abox, ABOX_SET_MODE, 2 /* IN_CALL */, 0, 0, NULL, 0, false);
	abox_fw_ctl1(abox, GID_C, CTL_PATH_CHANGE_ID, 12);
	for (i = 0; i < ARRAY_SIZE(abox_call_mux); i++)
		abox_fw_ctl1(abox, GID_W, abox_call_mux[i][0], abox_call_mux[i][1]);
	abox_fw_sys(abox, ABOX_SET_TYPE, speaker, 0, 0, NULL, 0, false);
	abox_fw_dma_open(abox, false);
	abox_fw_dma_open(abox, true);
	for (i = 0; i < ARRAY_SIZE(abox_call_en); i++)	/* DAPM PRE_PMU */
		abox_fw_ctl1(abox, GID_W, abox_call_en[i], 1);
	abox_fw_dma_go(abox, false);
	ret = abox_fw_dma_go(abox, true);
	abox->fw.rdma_kicks = 0;
	WRITE_ONCE(abox->fw.rdma_watch, true);
	schedule_delayed_work(&abox->fw.rdma_work, msecs_to_jiffies(50));
	return ret;
}

/* the reverse: TRIGGER 0, EN 0, CLOSE, muxes 0, then SET_MODE NORMAL */
static void abox_fw_hangup(struct abox *abox)
{
	int i, route = abox->fw.route;
	bool cap = route != ROUTE_TONE;

	if (!route || !abox->fw.running)
		return;
	/* the work never takes fw_lock: safe to wait for it here */
	WRITE_ONCE(abox->fw.rdma_watch, false);
	cancel_delayed_work_sync(&abox->fw.rdma_work);
	abox_fw_pcm(abox, false, 8, PCM_TRIGGER, 0, 0, 0, 0, true);
	if (cap)
		abox_fw_pcm(abox, true, 4, PCM_TRIGGER, 0, 0, 0, 0, true);
	if (route == ROUTE_CALL) {
		for (i = 0; i < ARRAY_SIZE(abox_call_en); i++)
			abox_fw_ctl1(abox, GID_W, abox_call_en[i], 0);
	} else {
		abox_fw_ctl1(abox, GID_W, W_RDMA8, 0);
		abox_fw_ctl1(abox, GID_W, route == ROUTE_TONE ? W_TONEGEN : W_WDMA4, 0);
	}
	abox_fw_pcm(abox, false, 8, PCM_CLOSE, 0, 0, 0, 0, false);
	if (cap)
		abox_fw_pcm(abox, true, 4, PCM_CLOSE, 0, 0, 0, 0, false);
	if (route == ROUTE_CALL) {
		for (i = 0; i < ARRAY_SIZE(abox_call_mux); i++)
			abox_fw_ctl1(abox, GID_W, abox_call_mux[i][0], 0);
		abox_fw_sys(abox, ABOX_SET_MODE, 0, 0, 0, NULL, 0, false);
	} else {
		abox_fw_ctl1(abox, GID_W, W_RDMA8_A, 0);
	}
	abox->fw.route = ROUTE_NONE;
}

/* the modem rings itself 0x10000 above the AP's doorbell: BAR0 + 0x70000 */
static int abox_fw_doorbell(struct abox *abox, phys_addr_t pa)
{
	if (!pa || !IS_ALIGNED(pa, SZ_4K))
		return -EINVAL;
	if (abox->fw.db_pa)
		iommu_unmap(abox->fw.domain, IOVA_VSS_PCI_DB, SZ_16K);
	abox->fw.db_pa = 0;
	if (iommu_map(abox->fw.domain, IOVA_VSS_PCI_DB, pa, SZ_16K,
		      IOMMU_READ | IOMMU_WRITE | IOMMU_MMIO, GFP_KERNEL))
		return -EIO;
	abox->fw.db_pa = pa;
	return 0;
}

static s32 abox_peak(const void *buf, size_t bytes, bool wide)
{
	s32 peak = 0, v;
	size_t i;

	for (i = 0; i < bytes; i += wide ? 4 : 2) {
		v = wide ? *(const s32 *)(buf + i) >> 16 : *(const s16 *)(buf + i);
		peak = max(peak, abs(v));
	}
	return peak;
}

static int abox_fw_show(struct seq_file *s, void *unused)
{
	struct abox *abox = s->private;
	void *tx = abox->fw.mem + FW_IPC_TX, *rx = abox->fw.mem + FW_IPC_RX;
	u32 p[4] = {};
	int i;

	mutex_lock(&abox->fw_lock);
	regmap_read(abox->fw.pmu, PMU_AUD_CPU0_POWER, &p[0]);
	regmap_read(abox->fw.pmu, PMU_AUD_CPU0_STANDBY, &p[1]);
	regmap_read(abox->fw.pmu, PMU_AUD_CPU1_POWER, &p[2]);
	regmap_read(abox->fw.pmu, PMU_AUD_CPU1_STANDBY, &p[3]);
	seq_printf(s, "%s, version %08x, GIC %s, route %d, vss %d\n",
		   abox->fw.running ? "running" : "stopped", abox->fw.version,
		   abox->fw.secure ? "secure (SMC)" : "plain", abox->fw.route,
		   abox->fw.vss);
	seq_printf(s, "PC %08x %08x; PMU cpu0 %x/%x cpu1 %x/%x; cpu %u Hz\n",
		   abox_readl(abox, ABOX_CORE0_PC), abox_readl(abox, ABOX_CORE1_PC),
		   p[0], p[1], p[2], p[3], abox_cpu_hz(abox));
	seq_printf(s, "irqs %u rx %u tx %u SysMMU faults %u; fault %#x %#x %#x; last SRAM word %08x\n",
		   abox->fw.irqs, abox->fw.rx_count, abox->fw.tx_count, abox->fw.faults,
		   abox->fw.fault[0], abox->fw.fault[1], abox->fw.fault[2],
		   readl(abox->fw.sram + abox->fw.sram_size - 4));
	seq_printf(s, "TX cmd %d/%d data %d/%d len %#x; RX cmd %d/%d data %d/%d len %#x\n",
		   *(s32 *)(tx + Q_CMD_IDX_S), *(s32 *)(tx + Q_CMD_IDX_E),
		   *(s32 *)(tx + Q_DATA_IDX_S), *(s32 *)(tx + Q_DATA_IDX_E),
		   *(u32 *)(tx + Q_DATA_LEN),
		   *(s32 *)(rx + Q_CMD_IDX_S), *(s32 *)(rx + Q_CMD_IDX_E),
		   *(s32 *)(rx + Q_DATA_IDX_S), *(s32 *)(rx + Q_DATA_IDX_E),
		   *(u32 *)(rx + Q_DATA_LEN));
	seq_printf(s, "DMAs running %#x (RDMA0-11, WDMA0-4, dual 0-4, debug 0-5); RDMA8 kicks %u\n",
		   abox_dmas_running(abox), abox->fw.rdma_kicks);
	seq_printf(s, "log %#x; doorbell %pa; ctl reply %d %d %d %d\n",
		   abox->fw.log_addr, &abox->fw.db_pa, abox->fw.ctl_reply[0],
		   abox->fw.ctl_reply[1], abox->fw.ctl_reply[2], abox->fw.ctl_reply[3]);
	if (abox->fw.buf[0].cpu)
		seq_printf(s, "peaks: RDMA8 %d WDMA4 %d\n",
			   abox_peak(abox->fw.buf[0].cpu, 3840 * 4, true),
			   abox_peak(abox->fw.buf[1].cpu, 960 * 4, false));
	seq_printf(s, "DRAM+%#x:", abox->fw.peek);
	for (i = 0; i < 64; i += 4)
		seq_printf(s, " %08x", *(u32 *)(abox->fw.mem + abox->fw.peek + i));
	seq_putc(s, '\n');
	mutex_unlock(&abox->fw_lock);
	return 0;
}

static int abox_fw_open(struct inode *inode, struct file *file)
{
	return single_open(file, abox_fw_show, inode->i_private);
}

static ssize_t abox_fw_write(struct file *file, const char __user *ubuf,
			     size_t count, loff_t *ppos)
{
	struct abox *abox = ((struct seq_file *)file->private_data)->private;
	char buf[128], cmd[16], arg[16];
	s32 v[10] = {};
	int n, ret = 0;

	if (count >= sizeof(buf))
		return -EINVAL;
	if (copy_from_user(buf, ubuf, count))
		return -EFAULT;
	buf[count] = 0;
	arg[0] = 0;
	n = sscanf(buf, "%15s %i %i %i %i %i %i %i %i %i %i", cmd, &v[0], &v[1],
		   &v[2], &v[3], &v[4], &v[5], &v[6], &v[7], &v[8], &v[9]);
	if (n < 1)
		return -EINVAL;
	n--;
	sscanf(buf, "%*s %15s", arg);

	mutex_lock(&abox->fw_lock);
	if (!strcmp(cmd, "boot")) {
		ret = abox_fw_boot(abox);
	} else if (!strcmp(cmd, "stop")) {
		abox_fw_stop(abox);
	} else if (!strcmp(cmd, "pins")) {
		ret = abox->pins_pci ? pinctrl_select_state(abox->pinctrl, abox->pins_pci) :
				       -ENODEV;
	} else if (!strcmp(cmd, "peek")) {
		ret = n == 1 && v[0] >= 0 && v[0] <= FW_DRAM_SIZE - 64 ? 0 : -EINVAL;
		if (!ret)
			abox->fw.peek = v[0] & ~3;
	} else if (!strcmp(cmd, "doorbell")) {
		u64 pa;

		ret = sscanf(buf, "%*s %lli", &pa) == 1 ? abox_fw_doorbell(abox, pa) : -EINVAL;
	} else if (!abox->fw.running) {
		ret = -ENODEV;			/* the rest needs the firmware */
	} else if (!strcmp(cmd, "rx")) {
		abox_fw_rx(abox);
	} else if (!strcmp(cmd, "sys") && n >= 1) {
		ret = abox_fw_sys(abox, v[0], v[1], v[2], v[3], NULL, 0, false);
	} else if (!strcmp(cmd, "cfg") && n >= 1) {
		ret = abox_fw_cfg(abox, v[0], v[1], v[2]);
	} else if (!strcmp(cmd, "ctl") && n >= 3) {
		ret = abox_fw_ctl(abox, v[0], v[1], &v[2], n - 2);
	} else if (!strcmp(cmd, "get") && n == 2) {
		reinit_completion(&abox->fw.ctl_done);
		memset(abox->fw.ctl_reply, 0, sizeof(abox->fw.ctl_reply));
		ret = abox_fw_sys(abox, ABOX_REQUEST_COMPONENT_CONTROL, v[0], v[1], 0,
				  NULL, 0, false);
		if (!ret && !wait_for_completion_timeout(&abox->fw.ctl_done, HZ))
			ret = -ETIMEDOUT;
	} else if (!strcmp(cmd, "mode") && n == 1) {
		ret = abox_fw_sys(abox, ABOX_SET_MODE, v[0], 0, 0, NULL, 0, false);
	} else if (!strcmp(cmd, "type") && n == 1) {
		ret = abox_fw_sys(abox, ABOX_SET_TYPE, v[0], 0, 0, NULL, 0, false);
	} else if (!strcmp(cmd, "vss")) {
		bool on = strcmp(arg, "stop");

		ret = abox_fw_sys(abox, on ? ABOX_START_VSS : ABOX_STOP_VSS, 0, 0, 0,
				  NULL, 0, false);
		if (!ret)
			abox->fw.vss = on;
	} else if (!strcmp(cmd, "tone")) {
		ret = abox_fw_route(abox, ROUTE_TONE, false);
	} else if (!strcmp(cmd, "loop")) {
		ret = abox_fw_route(abox, ROUTE_LOOP, false);
	} else if (!strcmp(cmd, "call")) {
		ret = abox_fw_route(abox, ROUTE_CALL, !strcmp(arg, "speaker"));
	} else if (!strcmp(cmd, "hangup")) {
		abox_fw_hangup(abox);
	} else {
		ret = -EINVAL;
	}
	mutex_unlock(&abox->fw_lock);
	return ret ?: count;
}

static const struct file_operations abox_fw_fops = {
	.owner		= THIS_MODULE,
	.open		= abox_fw_open,
	.read		= seq_read,
	.write		= abox_fw_write,
	.llseek		= seq_lseek,
	.release	= single_release,
};

/* which 4 KiB pages of the modem's voice area changed since the last read */
static int abox_pci_show(struct seq_file *s, void *unused)
{
	struct abox *abox = s->private;
	size_t i, pages = abox->fw.pci_size / SZ_4K;
	int changed = 0;

	for (i = 0; i < pages; i++) {
		u32 crc = crc32_le(0, abox->fw.pci + i * SZ_4K, SZ_4K);

		if (crc == abox->fw.pci_crc[i])
			continue;
		abox->fw.pci_crc[i] = crc;
		if (changed++ < 64)
			seq_printf(s, "%zu ", i);
	}
	seq_printf(s, "\n%d of %zu pages changed; head %*ph\n", changed, pages,
		   16, abox->fw.pci);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(abox_pci);

/* the firmware's log ring: REPORT_LOG's buffer, else the stock log area */
static int abox_fwlog_show(struct seq_file *s, void *unused)
{
	struct abox *abox = s->private;
	u32 a = abox->fw.log_addr, off, w, size;
	const char *p;

	if (a >= IOVA_FW_DRAM && a - IOVA_FW_DRAM < FW_DRAM_SIZE - 12) {
		off = a - IOVA_FW_DRAM;
		w = *(u32 *)(abox->fw.mem + off);
		size = *(u32 *)(abox->fw.mem + off + 8);
		seq_printf(s, "ring at %#x: writer %u reader %u size %u\n", a, w,
			   *(u32 *)(abox->fw.mem + off + 4), size);
		off += 12;
		if (size > FW_DRAM_SIZE - off || w > size)
			return 0;
	} else {
		seq_printf(s, "no ring reported (%#x): DRAM+%#x\n", a, FW_LOG_OFFSET);
		off = FW_LOG_OFFSET;
		size = SZ_1M;
		w = 0;
	}
	/* oldest first; NULs are gaps */
	for (p = abox->fw.mem + off + w; p < (char *)abox->fw.mem + off + size; p++)
		if (*p)
			seq_putc(s, *p);
	for (p = abox->fw.mem + off; p < (char *)abox->fw.mem + off + w; p++)
		if (*p)
			seq_putc(s, *p);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(abox_fwlog);

static void abox_fw_remove(void *data)
{
	struct abox *abox = data;

	cancel_work_sync(&abox->fw.wdt_work);
	WRITE_ONCE(abox->fw.rdma_watch, false);
	cancel_delayed_work_sync(&abox->fw.rdma_work);
	mutex_lock(&abox->fw_lock);
	abox_fw_stop(abox);
	if (abox->fw.stuck) {
		/* the cores may still write all of it: leak it */
		mutex_unlock(&abox->fw_lock);
		return;
	}
	abox_fw_bufs_free(abox);
	abox_fw_slog_free(abox);
	if (abox->fw.db_pa)
		iommu_unmap(abox->fw.domain, IOVA_VSS_PCI_DB, SZ_16K);
	mutex_unlock(&abox->fw_lock);
}

static int abox_fw_probe(struct abox *abox, struct platform_device *pdev)
{
	struct device *dev = abox->dev;
	struct resource r, *res;
	void __iomem *chipid;
	struct pci_dev *cp;
	u32 pid, rev;
	int ret;

	if (!abox->fw.domain)
		return 0;		/* firmware-less DT */

	ret = of_reserved_mem_region_to_resource_byname(dev->of_node, "fw", &r);
	if (ret || r.end + 1 != FW_DRAM_END || resource_size(&r) < FW_DRAM_SIZE)
		return dev_err_probe(dev, -EINVAL, "fw memory-region %pR\n", &r);
	abox->fw.dram_pa = r.end + 1 - FW_DRAM_SIZE;	/* 0xf8000000 */
	abox->fw.mem = devm_memremap(dev, abox->fw.dram_pa, FW_DRAM_SIZE, MEMREMAP_WC);
	if (IS_ERR(abox->fw.mem))
		return PTR_ERR(abox->fw.mem);

	abox->fw.sram = devm_platform_get_and_ioremap_resource(pdev, 2, &res);
	if (IS_ERR(abox->fw.sram))
		return PTR_ERR(abox->fw.sram);
	abox->fw.sram_size = resource_size(res);	/* 0x78c00 */
	abox->fw.timer = devm_platform_ioremap_resource_byname(pdev, "timer");
	if (IS_ERR(abox->fw.timer))
		return PTR_ERR(abox->fw.timer);
	abox->fw.gicd = devm_platform_get_and_ioremap_resource(pdev, 4, &res);
	if (IS_ERR(abox->fw.gicd))
		return PTR_ERR(abox->fw.gicd);
	abox->fw.gicd_pa = res->start;
	abox->fw.gicc = devm_platform_get_and_ioremap_resource(pdev, 5, &res);
	if (IS_ERR(abox->fw.gicc))
		return PTR_ERR(abox->fw.gicc);
	abox->fw.gicc_pa = res->start;
	abox->fw.pmu = syscon_regmap_lookup_by_phandle(dev->of_node, "samsung,pmu-syscon");
	if (IS_ERR(abox->fw.pmu))
		return dev_err_probe(dev, PTR_ERR(abox->fw.pmu), "no PMU\n");
	abox->fw.irq = platform_get_irq_byname(pdev, "gic");
	if (abox->fw.irq < 0)
		return abox->fw.irq;

	/* stock is_secure_gic(): the 990 is secure from chip revision 1 */
	chipid = ioremap(IOVA_CHIPID, 0x20);
	if (!chipid)
		return -ENOMEM;
	pid = readl(chipid);
	rev = readl(chipid + 0x10) >> 16;
	iounmap(chipid);
	abox->fw.secure = pid == 0xe9830000 ? rev > 0 : true;
	dev_info(dev, "ABOX GIC %s (PRO_ID %08x rev %x)\n",
		 abox->fw.secure ? "secure: SMC" : "plain MMIO", pid, rev);

	ret = iommu_map(abox->fw.domain, IOVA_FW_DRAM, abox->fw.dram_pa, FW_DRAM_SIZE,
			IOMMU_READ | IOMMU_WRITE, GFP_KERNEL);
	ret = ret ?: iommu_map(abox->fw.domain, IOVA_CHIPID, IOVA_CHIPID, PAGE_SIZE,
			       IOMMU_READ | IOMMU_WRITE | IOMMU_MMIO, GFP_KERNEL);
	if (ret)
		return ret;

	/* the modem's voice area: zeroed while the modem is down, as stock at boot */
	if (!of_reserved_mem_region_to_resource_byname(dev->of_node, "pci", &r)) {
		abox->fw.pci_pa = r.start;
		abox->fw.pci_size = resource_size(&r);
		abox->fw.pci = devm_memremap(dev, r.start, abox->fw.pci_size, MEMREMAP_WC);
		if (IS_ERR(abox->fw.pci))
			return PTR_ERR(abox->fw.pci);
		abox->fw.pci_crc = devm_kcalloc(dev, abox->fw.pci_size / SZ_4K,
						sizeof(u32), GFP_KERNEL);
		if (!abox->fw.pci_crc)
			return -ENOMEM;
		cp = pci_get_domain_bus_and_slot(1, 1, PCI_DEVFN(0, 0));
		if (cp)
			dev_info(dev, "the modem is up: its voice area stays as it is\n");
		else
			memset(abox->fw.pci, 0, abox->fw.pci_size);
		pci_dev_put(cp);
		ret = iommu_map(abox->fw.domain, IOVA_VSS_PCI, r.start, abox->fw.pci_size,
				IOMMU_READ | IOMMU_WRITE, GFP_KERNEL);
		if (ret)
			return ret;
	}

	spin_lock_init(&abox->fw.tx_lock);
	mutex_init(&abox->fw.rx_lock);
	init_completion(&abox->fw.boot_done);
	init_completion(&abox->fw.ctl_done);
	INIT_WORK(&abox->fw.wdt_work, abox_fw_wdt_work);
	INIT_DELAYED_WORK(&abox->fw.rdma_work, abox_fw_rdma_work);
	ret = devm_request_threaded_irq(dev, abox->fw.irq, NULL, abox_gic_thread,
					IRQF_ONESHOT | IRQF_NO_AUTOEN, "abox-gic", abox);
	if (ret)
		return ret;
	return devm_add_action_or_reset(dev, abox_fw_remove, abox);
}

static int abox_regs_show(struct seq_file *s, void *unused)
{
	struct abox *abox = s->private;
	static const u32 regs[] = {
		ABOX_VERSION, ABOX_SYSPOWER_STATUS, ABOX_QCHANNEL_DISABLE,
		ABOX_ROUTE_CTRL0, ABOX_SPUS_CTRL_FC0, ABOX_SPUS_CTRL1,
		ABOX_SIFS_CNT(1), 0x0030, 0x0050, 0x0210, 0x0214, 0x0218, 0x021c,
		ABOX_UAIF_CTRL0(ABOX_UAIF_SPK), ABOX_UAIF_CTRL1(ABOX_UAIF_SPK),
		ABOX_UAIF_STATUS(ABOX_UAIF_SPK), ABOX_ROUTE_CTRL1, ABOX_SPUM_CTRL0,
		ABOX_SPUM_SBANK_NSRC(ABOX_NSRC_MIC), 0x0310,
		ABOX_UAIF_CTRL0(ABOX_UAIF_MIC), ABOX_UAIF_CTRL1(ABOX_UAIF_MIC),
		ABOX_UAIF_STATUS(ABOX_UAIF_MIC),
	};
	int i, d;

	if (pm_runtime_get_if_active(abox->dev) <= 0) {
		seq_puts(s, "suspended\n");
		return 0;
	}

	for (i = 0; i < ARRAY_SIZE(regs); i++)
		seq_printf(s, "%04x: %08x\n", regs[i], abox_readl(abox, regs[i]));
	for (d = 0; d < 2; d++)
		for (i = 0; i <= ABOX_DMA_STATUS; i += 4)
			seq_printf(s, "%04x: %08x\n", abox->dma[d].base + i,
				   abox_dma_read(&abox->dma[d], i));
	seq_printf(s, "cmu pll %08x audif %08x uaif2 mux %08x div %08x uaif6 mux %08x div %08x\n",
		   readl(abox->cmu + CMU_PLL_CON3_AUD0),
		   readl(abox->cmu + CMU_DIV_AUDIF),
		   readl(abox->cmu + CMU_MUX_UAIF(ABOX_UAIF_SPK)),
		   readl(abox->cmu + CMU_DIV_UAIF(ABOX_UAIF_SPK)),
		   readl(abox->cmu + CMU_MUX_UAIF(ABOX_UAIF_MIC)),
		   readl(abox->cmu + CMU_DIV_UAIF(ABOX_UAIF_MIC)));

	pm_runtime_put(abox->dev);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(abox_regs);

static const struct snd_soc_component_driver abox_component = {
	.name		= "exynos990-abox",
	.open		= abox_pcm_open,
	.close		= abox_pcm_close,
	.hw_params	= abox_pcm_hw_params,
	.prepare	= abox_pcm_prepare,
	.trigger	= abox_pcm_trigger,
	.pointer	= abox_pcm_pointer,
	.pcm_new	= abox_pcm_new,
	.pcm_free	= abox_pcm_free,
};

static void abox_debugfs_remove(void *dir)
{
	debugfs_remove_recursive(dir);
}

static int abox_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct dentry *dir;
	struct abox *abox;
	int i, ret;

	abox = devm_kzalloc(dev, sizeof(*abox), GFP_KERNEL);
	if (!abox)
		return -ENOMEM;
	abox->dev = dev;
	platform_set_drvdata(pdev, abox);
	spin_lock_init(&abox->dma_lock);
	mutex_init(&abox->fw_lock);

	abox->sfr = devm_platform_ioremap_resource_byname(pdev, "sfr");
	if (IS_ERR(abox->sfr))
		return PTR_ERR(abox->sfr);
	abox->cmu = devm_platform_ioremap_resource_byname(pdev, "cmu");
	if (IS_ERR(abox->cmu))
		return PTR_ERR(abox->cmu);

	abox->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(abox->pinctrl))
		return dev_err_probe(dev, PTR_ERR(abox->pinctrl), "no pinctrl\n");
	abox->pins_active = pinctrl_lookup_state(abox->pinctrl, "active");
	if (IS_ERR(abox->pins_active))
		return dev_err_probe(dev, PTR_ERR(abox->pins_active),
				     "no active pin state\n");
	abox->pins_idle = pinctrl_lookup_state(abox->pinctrl, "idle");
	if (IS_ERR(abox->pins_idle))
		return dev_err_probe(dev, PTR_ERR(abox->pins_idle),
				     "no idle pin state\n");
	abox->pins_pci = pinctrl_lookup_state(abox->pinctrl, "pci");
	if (IS_ERR(abox->pins_pci))
		abox->pins_pci = NULL;

	/* the DMA address registers are 32 bit */
	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;

	if (rdma_idx < 0 || rdma_idx > 11)
		return -EINVAL;
	abox->dma[SNDRV_PCM_STREAM_PLAYBACK].base = ABOX_RDMA(rdma_idx);
	abox->dma[SNDRV_PCM_STREAM_CAPTURE].base = ABOX_WDMA(ABOX_WDMA_MIC);
	for (i = 0; i < 2; i++) {
		abox->dma[i].abox = abox;
		hrtimer_setup(&abox->dma[i].timer, abox_period_timer,
			      CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	}
	dev_info(dev, "playback on RDMA%d, capture on WDMA%d\n", rdma_idx,
		 ABOX_WDMA_MIC);

	pm_runtime_set_autosuspend_delay(dev, 3000);
	pm_runtime_use_autosuspend(dev);
	ret = devm_pm_runtime_enable(dev);
	if (ret)
		return ret;

	dir = debugfs_create_dir("exynos990-abox", NULL);
	debugfs_create_file("regs", 0400, dir, abox, &abox_regs_fops);
	ret = devm_add_action_or_reset(dev, abox_debugfs_remove, dir);
	if (ret)
		return ret;

	/* before anything uses the DMA API on dev: it is gone once attached */
	ret = abox_fw_attach(abox);
	if (ret)
		return ret;
	ret = abox_fw_probe(abox, pdev);
	if (ret)
		return ret;
	if (abox->fw.domain) {
		debugfs_create_file("firmware", 0600, dir, abox, &abox_fw_fops);
		if (abox->fw.pci)
			debugfs_create_file("pci", 0400, dir, abox, &abox_pci_fops);
		debugfs_create_file("fwlog", 0400, dir, abox, &abox_fwlog_fops);
		dev_info(dev, "own SysMMU domain; firmware %pa+%#x, voice area %pa\n",
			 &abox->fw.dram_pa, FW_DRAM_SIZE, &abox->fw.pci_pa);
	}

	return devm_snd_soc_register_component(dev, &abox_component, abox_dais,
					       ARRAY_SIZE(abox_dais));
}

static const struct dev_pm_ops abox_pm_ops = {
	RUNTIME_PM_OPS(abox_runtime_suspend, abox_runtime_resume, NULL)
};

static const struct of_device_id abox_of_match[] = {
	{ .compatible = "samsung,exynos990-abox" },
	{ }
};
MODULE_DEVICE_TABLE(of, abox_of_match);

static struct platform_driver abox_driver = {
	.driver = {
		.name = "exynos990-abox",
		.of_match_table = abox_of_match,
		.pm = pm_ptr(&abox_pm_ops),
	},
	.probe = abox_probe,
};
module_platform_driver(abox_driver);

MODULE_DESCRIPTION("Exynos 990 ABOX audio (RDMA/WDMA/UAIF; calliope firmware for calls)");
MODULE_LICENSE("GPL");
