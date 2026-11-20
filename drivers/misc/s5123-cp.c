// SPDX-License-Identifier: GPL-2.0
//
// Samsung S5123 (Exynos Modem 5123) link over PCIe, for the Exynos 990
// Galaxy S20 FE (SM-G780F).
//
// The CP is a separate chip on the GEN4 PCIe port (HSI2, channel 1).
// Booting it stays in userspace for now (VibeOS's cp-boot.sh: the CP
// power GPIOs, its bootloader over SPI16, the link, then cbd's stage
// protocol on the legacy raw rings). This driver takes over when the CP
// posts INIT_START: it lays the SBD rings and the pktproc v2 queue out the
// way stock cpif does, answers with PIF_INIT_DONE, waits for PHONE_START,
// sends INIT_END once the IPC and RFS channels are open, and then carries
// the channels. Userspace sees the S10e driver's contract
// (drivers/misc/exynos9820-cp.c in the S10e tree), so cpd runs on both:
//   /dev/umts_ipc0, /dev/umts_ipc1  Samsung IPC (SIPC channels 235, 236)
//   /dev/umts_rfs0                  the CP's remote file system (245)
//   /dev/umts_atbridge              far side of a WWAN AT port (/dev/wwan0at0)
//   rmnet0                          raw-IP PS data (channels 10..17), while
//                                   the AT bridge is open
// One read() returns one frame without its SIPC5 link header; one write()
// sends one frame.
//
// Transport (from the stock cpif sources):
//   - the IPC region is AP 0xF0000000, 8 MiB (the RC's "ipc" memory-region);
//   - AP->CP: a word at IPC+0x400, then 0x10000 written to EP BAR0 + 0x60000;
//   - CP->AP: a word at IPC+0x404 with MSI vector 0 (vector 1: the status
//     word at IPC+0x40C changed). The MSIs go to 0xF6200000, which the RC
//     glue makes its iMSI-RX target, so they never reach memory;
//   - the PS downlink is pktproc v2 (SktBuf descriptors) in the RC's
//     "pktproc" memory-region, AP 0xD0000000 = CP 0x28000000.
// Without IOCC the CP's writes do not snoop the AP caches, so both regions
// are mapped write-combined (Normal non-cacheable) and never cached.
//
// The character and network devices hang off a platform device "s5123-cp"
// that lives as long as the module (ModemManager needs a platform, PCI or
// USB parent for the AT port to count as a modem); the PCI endpoint comes
// and goes with every CP power cycle (the RC is unbound
// before the CP is powered off), and the link attaches to them while it is
// there. Losing the endpoint ends every reader with -EIO, as a CP crash does.
//
// Link power follows stock's CP2AP_WAKE protocol (modem_ctrl_s5100.c), which
// the CP firmware relies on: once INIT_END is out, CP2AP_WAKE low for 30 ms
// takes the link to L2 (AP2CP_WAKE low, PME_Turn_Off, PERST#), and
// CP2AP_WAKE high brings it back (AP2CP_WAKE high, link-up, config restore,
// the link_ack doorbell, then any doorbell that waited). A doorbell while the
// link is down waits, and raising AP2CP_WAKE asks the CP for CP2AP_WAKE.
// Without this the CP keeps everything it produces after dropping
// CP2AP_WAKE until the AP happens to send again.

#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/gpio/consumer.h>
#include <linux/gpio/machine.h>
#include <linux/if_arp.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/of.h>
#include <linux/of_reserved_mem.h>
#include <linux/pci.h>
#include <linux/pcie-exynos990-gen4.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/skbuff.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/unaligned.h>
#include <linux/wait.h>
#include <linux/workqueue.h>
#include <linux/wwan.h>

#define PCI_DEVICE_ID_S5123		0xa5a5

/* EP BAR0: the doorbell (s51xx_pcie.c; DT pci_db_addr = BAR0 + 0x60000) */
#define DB_OFFSET			0x60000
#define DB_INT_MSG			0x10000		/* int 0: ap2cp_msg */
#define DB_INT_LINK_ACK			0x1000e		/* int 14: mif,int_ap2cp_pcie_link_ack */
#define CP_LINK_OFF_DELAY_MS		30		/* s5100_poweroff_pcie */

/* legacy map at the start of the IPC region (link_device_memory.h) */
#define IPC_SIZE			0x800000
#define IPC_MAGIC			0x000
#define IPC_ACCESS			0x004
#define IPC_HEADS_FIRST			0x008		/* fmt tx/rx, raw tx/rx head + tail */
#define IPC_HEADS_LAST			0x024
#define IPC_AP2CP_MSG			0x400
#define IPC_CP2AP_MSG			0x404
#define IPC_AP2CP_STATUS		0x408
#define IPC_CP2AP_STATUS		0x40c
#define IPC_BOOTLOG			0xc00		/* u32 length, text; gone at SBD init */
#define IPC_BOOTLOG_MAX			0x1fb
#define MEM_IPC_MAGIC			0xaa
#define CP2AP_FLOWCTL			BIT(2)

/* SIPC5 link header */
#define SIPC5_START			0xf8
#define SIPC5_EXT_LEN			BIT(1)

/* ap2cp_msg / cp2ap_msg words */
#define MSG_INT_VALID			0x80
#define MSG_CMD_VALID			0x40
#define MSG_CMD_FIELD			0x3f
#define MSG_SEND_DATA			0x01
#define CMD_INIT_START			0x01
#define CMD_INIT_END			0x02
#define CMD_CRASH_RESET			0x07
#define CMD_PHONE_START			0x08
#define CMD_CRASH_EXIT			0x09
#define CMD_PIF_INIT_DONE		0x0d

/*
 * SBD rings over the IPC region (include/sbd.h, link_device_memory_sbd.c):
 * a global descriptor at 0x800, the rp/wp arrays, per-ring buffer position
 * and length arrays, and the buffers from 0x10000 in link order.
 */
#define SBD_DESC_RGN			0x800
#define SBD_DESC_END			0x10000
#define SBD_BUFF_RGN			0x10000
#define SBD_MAX_LINKS			32
#define SBD_RB_CH			(SBD_DESC_RGN + 12)
#define SBD_RB_DESC			(SBD_RB_CH + SBD_MAX_LINKS * 16)
#define SBD_RBPS			(SBD_RB_DESC + SBD_MAX_LINKS * 2 * 20)
#define CP_NUM_IODS			3

/*
 * PS data: rmnet0..7 are SIPC channels 10..17. UL goes through ring 9
 * (multipdp) with the channel in buff_len bits 23:16; DL comes through
 * pktproc, and through rings 8/9 if the CP ever uses them.
 */
#define PS_CH_FIRST			10
#define PS_CH_LAST			17
#define PS_UL_LINK			9
#define PS_MTU				1500

/*
 * pktproc v2 (link_rx_pktproc.[ch], stock DT): one queue, SktBuf mode.
 * Info at +0 (u32 {num_queues:4, mode:4, max_packet_size:16}, then
 * q_info[] {desc_base, num_desc, data_base, fore_ptr, rear_ptr}), 16-byte
 * descriptors at +0x100, data at +0x100000. The AP hands the CP buffers
 * (data_addr, advancing fore_ptr); the CP fills them and advances rear_ptr.
 * Stock uses the first 32 MiB of the region.
 */
#define PP_SIZE				0x2000000
#define PP_CP_BASE			0x28000000
#define PP_INFO_MODE			(1 | 1 << 4 | 2048 << 8)
#define PP_Q0				0x04
#define PP_FORE				(PP_Q0 + 12)
#define PP_REAR				(PP_Q0 + 16)
#define PP_DESC				0x100
#define PP_DESC_SIZE			16
#define PP_DATA				0x100000
#define PP_DATA_SIZE			(PP_SIZE - PP_DATA)
#define PP_CELL				2560		/* MIF_BUFF_DEFAULT_CELL_SIZE */
#define PP_HEADROOM			66		/* stock NET_SKB_PAD + NET_IP_ALIGN */
#define PP_NUM				((PP_DATA_SIZE / PP_CELL) * 50 / 100)
#define PP_MAX_PKT			2048
#define PP_CTRL_HEAD			BIT(7)
#define PP_CTRL_RINGEND			BIT(3)

static unsigned int ps_ul_ch = PS_CH_FIRST;
module_param(ps_ul_ch, uint, 0644);
MODULE_PARM_DESC(ps_ul_ch, "SIPC channel rmnet0 sends on (10 = PDP cid 1 [guess])");

static unsigned int poll_ms;
module_param(poll_ms, uint, 0644);
MODULE_PARM_DESC(poll_ms, "also poll cp2ap_msg every N ms once up (0 = MSI only)");

static bool link_pm = true;
module_param(link_pm, bool, 0644);
MODULE_PARM_DESC(link_pm, "take the link down and up with CP2AP_WAKE after INIT_END, as stock does");

static bool pm_log;
module_param(pm_log, bool, 0644);
MODULE_PARM_DESC(pm_log, "log every link down and up");

static unsigned int wake_ms = 1000;
module_param(wake_ms, uint, 0644);
MODULE_PARM_DESC(wake_ms, "keep the system awake this long after the CP raises CP2AP_WAKE");

/* the channels userspace gets, with their ring (link id) */
static const struct {
	const char *name;
	u8 ch, link;
} cp_iod_cfg[CP_NUM_IODS] = {
	{ "umts_ipc0", 235, 1 },
	{ "umts_ipc1", 236, 2 },
	{ "umts_rfs0", 245, 3 },
};

enum { UL, DL };

struct cp_link_cfg {
	u8 ch;
	bool ps;		/* raw IP slots, not SIPC5 frames */
	u16 len[2];
	u16 size[2];
};

/* the stock DT's iodevs that get a ring, in order (ids 0..9) */
static const struct cp_link_cfg cp_links[] = {
	{  35, false, {   16,   16 }, { 2048, 2048 } },	/* umts_cass */
	{ 235, false, {   16,   32 }, { 4096, 4096 } },	/* umts_ipc0 */
	{ 236, false, {   16,   32 }, { 4096, 4096 } },	/* umts_ipc1 */
	{ 245, false, {   16,  512 }, { 2048, 2048 } },	/* umts_rfs0 */
	{   1, false, {   32,   64 }, { 2048, 2048 } },	/* umts_csd */
	{  25, false, {   16,   16 }, { 2048, 2048 } },	/* umts_router */
	{  28, false, {   16,  128 }, { 2048, 2048 } },	/* umts_dm0 */
	{ 244, false, {   16,   32 }, { 2048, 2048 } },	/* ipc_loopback0 */
	{  10, true,  { 1024,  256 }, { 2048, 2048 } },	/* multipdp_hiprio */
	{  11, true,  {  512, 1024 }, { 2048, 2048 } },	/* multipdp */
};
#define CP_NUM_LINKS			ARRAY_SIZE(cp_links)

struct cp_ring {
	u16 len, size;
	u32 buff;		/* offsets into the IPC region */
	u32 addr_v, size_v;
	u32 rp, wp;		/* offsets of the u16 pointers */
	/* RX reassembly of a frame that spans slots */
	u8 *frm;
	unsigned int total, rcvd;
};

enum cp_state {
	CP_OFFLINE,		/* no endpoint */
	CP_BOOTING,		/* endpoint there, before PHONE_START */
	CP_ONLINE,
	CP_CRASHED,		/* crashed, or the endpoint went away */
};

static const char * const cp_state_name[] = {
	"offline", "booting", "online", "crashed",
};

struct s5123_cp {
	struct platform_device *plat;
	struct device *dev;
	struct mutex lock;	/* attach/detach, commands, INIT_END */
	struct mutex svc_lock;	/* one cp2ap_msg service at a time */
	struct mutex tx_lock;	/* SBD UL rings of the char devices */
	spinlock_t ps_tx_lock;	/* ring 9 UL, from ndo_start_xmit */
	spinlock_t msg_lock;	/* ap2cp_msg + doorbell */
	wait_queue_head_t wq;

	/* the link, while the endpoint is there */
	struct pci_dev *pdev;
	void __iomem *db;
	void __iomem *ipc;
	void __iomem *pp;
	int irq_msg, irq_status;
	struct delayed_work poll;
	u32 poll_last;

	/* link power: stock's CP2AP_WAKE protocol */
	struct gpio_desc *ap2cp_wake, *cp2ap_wake;
	int irq_wake;
	bool wake_irq_on;
	struct mutex pm_lock;	/* link up/down, AP2CP_WAKE */
	struct work_struct pm_work;
	bool pm_enabled;
	bool link_on;		/* the EP can take a doorbell (msg_lock) */
	bool db_reserved;	/* a doorbell waits for the link (msg_lock) */
	unsigned long link_downs, link_ups, link_fails;
	struct pci_saved_state *ep_state;	/* the EP's config space at the last link down */

	/* system sleep: stock's PDA_ACTIVE, and CP2AP_WAKE edges held for resume */
	struct gpio_desc *pda_active;
	bool sleeping;		/* from our suspend to our resume (pm_lock) */
	bool wake_deferred;	/* CP2AP_WAKE moved while sleeping (pm_lock) */
	bool wake_armed;	/* CP2AP_WAKE is a wakeup interrupt now */

	enum cp_state state;
	u32 cp2ap_msg;
	unsigned long msg_count, status_count;
	bool sbd_up;		/* rings laid out, PIF_INIT_DONE sent */
	bool ipc_up;		/* PHONE_START seen, IPC magic set */
	bool init_end;		/* INIT_END sent */
	struct cp_ring ring[CP_NUM_LINKS][2];
	u32 pp_fore, pp_done;
	unsigned long rx_frames, rx_dropped, pp_rx;

	/* PS data */
	struct net_device *ps_dev;
	bool ps_ch_seen[PS_CH_LAST - PS_CH_FIRST + 1];

	/* WWAN AT port <-> /dev/umts_atbridge */
	struct wwan_port *at_port;
	struct miscdevice at_bridge;
	struct sk_buff_head at_txq;
	wait_queue_head_t at_wq;
	atomic_t at_bridge_open;

	struct cp_iod {
		struct s5123_cp *cp;
		struct miscdevice misc;
		u8 ch, link;
		atomic_t opened;
		struct sk_buff_head rxq;
		wait_queue_head_t wq;
	} iod[CP_NUM_IODS];
};

static struct s5123_cp *the_cp;

/* AP -> CP */

/* pcie_send_ap2cp_irq: the word always, the doorbell when the link is there */
static void cp_send_msg(struct s5123_cp *cp, u32 val)
{
	unsigned long flags;
	bool wait = false;
	u32 rb = 0;

	spin_lock_irqsave(&cp->msg_lock, flags);
	writel(val, cp->ipc + IPC_AP2CP_MSG);
	if (cp->link_on) {
		writel(DB_INT_MSG, cp->db);	/* writel orders the word before it */
		rb = readl(cp->db);
	} else {
		cp->db_reserved = true;
		wait = true;
	}
	spin_unlock_irqrestore(&cp->msg_lock, flags);
	if (wait)
		queue_work(system_highpri_wq, &cp->pm_work);
	else if (rb == 0xffffffff)
		dev_err_ratelimited(cp->dev, "doorbell reads all ones: link down?\n");
}

static void cp_send_cmd(struct s5123_cp *cp, u32 cmd)
{
	dev_info(cp->dev, "cmd 0x%02x -> CP\n", MSG_INT_VALID | MSG_CMD_VALID | cmd);
	cp_send_msg(cp, MSG_INT_VALID | MSG_CMD_VALID | cmd);
}

static u16 ring_get(struct s5123_cp *cp, u32 off)
{
	return readw(cp->ipc + off);
}

static void ring_set(struct s5123_cp *cp, u32 off, u16 val)
{
	writew(val, cp->ipc + off);
}

/* link power */

static void cp_service(struct s5123_cp *cp);

static bool cp_ul_pending(struct s5123_cp *cp)
{
	unsigned int i;

	for (i = 0; i < CP_NUM_LINKS; i++) {
		struct cp_ring *rb = &cp->ring[i][UL];

		if (rb->len && ring_get(cp, rb->rp) != ring_get(cp, rb->wp))
			return true;
	}
	return false;
}

/*
 * s5100_poweroff_pcie with s51xx_pcie_save_state: the endpoint goes to D3hot
 * with bus mastering and MSI off before the AP drops AP2CP_WAKE and sends
 * PME_Turn_Off. With pm_lock.
 */
static void cp_link_down(struct s5123_cp *cp)
{
	struct pci_dev *pdev = cp->pdev, *rp = pcie_find_root_port(pdev);
	unsigned long flags;
	int ret;

	spin_lock_irqsave(&cp->msg_lock, flags);
	cp->link_on = false;
	spin_unlock_irqrestore(&cp->msg_lock, flags);

	if (rp)
		pci_save_state(rp);
	pci_clear_master(pdev);
	pci_save_state(pdev);
	/*
	 * Keep a copy of our own: across a system sleep the PCI core saves and
	 * restores the endpoint as well, through a link that is down, and that
	 * leaves all ones (or nothing) for the restore at the next link up.
	 */
	kfree(cp->ep_state);
	cp->ep_state = pci_store_saved_state(pdev);
	disable_irq(cp->irq_msg);
	if (cp->irq_status >= 0)
		disable_irq(cp->irq_status);
	pci_disable_device(pdev);
	pci_wake_from_d3(pdev, false);
	if (pci_set_power_state(pdev, PCI_D3hot))
		dev_warn_ratelimited(cp->dev, "no D3hot\n");

	gpiod_set_value(cp->ap2cp_wake, 0);
	ret = exynos990_gen4_link_off(pdev);
	if (ret)
		dev_err(cp->dev, "link off: %d\n", ret);
	cp->link_downs++;
	if (pm_log)
		dev_info(cp->dev, "link down (%lu)\n", cp->link_downs);
}

/* s5100_poweron_pcie; with pm_lock */
static int cp_link_up(struct s5123_cp *cp)
{
	struct pci_dev *rp = pcie_find_root_port(cp->pdev);
	unsigned long flags;
	int ret;

	gpiod_set_value(cp->ap2cp_wake, 1);
	usleep_range(5000, 6000);
	ret = exynos990_gen4_link_on(cp->pdev);
	if (ret) {
		cp->link_fails++;
		dev_err_ratelimited(cp->dev, "link on: %d\n", ret);
		return ret;
	}
	/* s51xx_pcie_restore_state */
	if (rp)
		pci_restore_state(rp);
	if (pci_set_power_state(cp->pdev, PCI_D0))
		dev_warn_ratelimited(cp->dev, "no D0\n");
	if (cp->ep_state)
		pci_load_saved_state(cp->pdev, cp->ep_state);
	pci_restore_state(cp->pdev);
	pci_enable_wake(cp->pdev, PCI_D0, false);
	if (pci_enable_device(cp->pdev))
		dev_err_ratelimited(cp->dev, "cannot re-enable the endpoint\n");
	pci_set_master(cp->pdev);
	enable_irq(cp->irq_msg);
	if (cp->irq_status >= 0)
		enable_irq(cp->irq_status);

	spin_lock_irqsave(&cp->msg_lock, flags);
	cp->link_on = true;
	writel(DB_INT_LINK_ACK, cp->db);
	if (cp->db_reserved) {
		cp->db_reserved = false;
		writel(DB_INT_MSG, cp->db);
	}
	spin_unlock_irqrestore(&cp->msg_lock, flags);
	cp->link_ups++;
	if (pm_log)
		dev_info(cp->dev, "link up (%lu)\n", cp->link_ups);
	return 0;
}

/* cp2ap_wakeup_work / cp2ap_suspend_work, and s5100_try_gpio_cp_wakeup */
static void cp_pm_work(struct work_struct *work)
{
	struct s5123_cp *cp = container_of(work, struct s5123_cp, pm_work);
	bool up = false;

	mutex_lock(&cp->pm_lock);
	if (!READ_ONCE(cp->pm_enabled) || !cp->pdev)
		goto out;
	if (cp->sleeping) {		/* the PCI side is suspended: after resume */
		cp->wake_deferred = true;
		goto out;
	}
	if (gpiod_get_value(cp->cp2ap_wake)) {
		if (!cp->link_on)
			up = !cp_link_up(cp);
	} else if (cp->link_on) {
		/* "CP reads Tx RP (or tail) after CP2AP_WAKEUP = 1" */
		msleep(CP_LINK_OFF_DELAY_MS);
		if (!gpiod_get_value(cp->cp2ap_wake) && !cp_ul_pending(cp) &&
		    !READ_ONCE(cp->db_reserved))
			cp_link_down(cp);
	}
	/* a doorbell waits on a link that is down: ask the CP for CP2AP_WAKE */
	if (!cp->link_on && READ_ONCE(cp->db_reserved) && !gpiod_get_value(cp->cp2ap_wake))
		gpiod_set_value(cp->ap2cp_wake, 1);
out:
	mutex_unlock(&cp->pm_lock);
	if (up)
		cp_service(cp);		/* MSIs sent while the link was down are lost */
}

static irqreturn_t cp_wake_irq(int irq, void *data)
{
	struct s5123_cp *cp = data;

	/*
	 * The CP wants the link: a call, an SMS, data. Keep the system up long
	 * enough to answer it, and abort a suspend that is on its way.
	 */
	if (READ_ONCE(cp->pm_enabled) && gpiod_get_value(cp->cp2ap_wake))
		pm_wakeup_dev_event(cp->dev, wake_ms, true);
	queue_work(system_highpri_wq, &cp->pm_work);
	return IRQ_HANDLED;
}

/* complete_normal_boot: CP2AP_WAKE is only acted on once the CP is up */
static void cp_pm_enable(struct s5123_cp *cp)
{
	if (!link_pm || cp->irq_wake <= 0 || READ_ONCE(cp->pm_enabled))
		return;
	WRITE_ONCE(cp->pm_enabled, true);
	/* cp-boot.sh raised PDA_ACTIVE with the CP; from here a system sleep lowers it */
	if (cp->pda_active && gpiod_direction_output(cp->pda_active, 1))
		cp->pda_active = NULL;
	if (!cp->wake_irq_on) {
		enable_irq(cp->irq_wake);
		cp->wake_irq_on = true;
	}
	queue_work(system_highpri_wq, &cp->pm_work);
	dev_info(cp->dev, "link power management on\n");
}

/* INIT_START */

/* what the CP bootloader left at IPC+0xC00, before SBD init overwrites it */
static void cp_print_bootlog(struct s5123_cp *cp)
{
	char *buf;
	u32 len, i;

	len = readl(cp->ipc + IPC_BOOTLOG);
	if (!len || len > IPC_BOOTLOG_MAX)
		return;
	buf = kzalloc(len + 1, GFP_KERNEL);
	if (!buf)
		return;
	memcpy_fromio(buf, cp->ipc + IPC_BOOTLOG + 4, len);
	for (i = 0; i < len; i++)
		if (buf[i] != '\n' && (buf[i] < 0x20 || buf[i] > 0x7e))
			buf[i] = '.';
	dev_info(cp->dev, "CP boot log: %s\n", buf);
	kfree(buf);
}

/* pktproc_fill_data_addr: hand the CP a buffer in every free descriptor */
static void cp_pp_refill(struct s5123_cp *cp)
{
	u32 fore = cp->pp_fore;
	u32 space = (cp->pp_done + PP_NUM - fore - 1) % PP_NUM;

	while (space--) {
		void __iomem *d = cp->pp + PP_DESC + fore * PP_DESC_SIZE;
		u32 ctl = (fore == 0 ? PP_CTRL_HEAD : 0) |
			  (fore == PP_NUM - 1 ? PP_CTRL_RINGEND : 0);

		/* descriptor n always gets cell n: there are more cells than descriptors */
		writel(PP_CP_BASE + PP_DATA + fore * PP_CELL + PP_HEADROOM, d);
		writel(ctl << 8, d + 4);	/* data_addr_msb 0, control, status 0 */
		writel(0, d + 8);
		writel(0, d + 12);
		fore = (fore + 1) % PP_NUM;
	}
	wmb();
	cp->pp_fore = fore;
	writel(fore, cp->pp + PP_FORE);
}

/* pktproc_create + pktproc_init */
static void cp_pp_init(struct s5123_cp *cp)
{
	memset_io(cp->pp, 0, PP_DATA);
	writel(PP_INFO_MODE, cp->pp);
	writel(PP_CP_BASE + PP_DESC, cp->pp + PP_Q0);
	writel(PP_NUM, cp->pp + PP_Q0 + 4);
	writel(PP_CP_BASE + PP_DATA, cp->pp + PP_Q0 + 8);
	writel(0, cp->pp + PP_FORE);
	writel(0, cp->pp + PP_REAR);
	cp->pp_fore = cp->pp_done = 0;
	cp_pp_refill(cp);
	dev_info(cp->dev, "pktproc: %u descriptors, fore %u\n", (unsigned int)PP_NUM, cp->pp_fore);
}

/* init_sbd_link: lay the rings out the way stock does */
static void cp_sbd_init(struct s5123_cp *cp)
{
	void __iomem *ipc = cp->ipc;
	u32 desc = SBD_RBPS + CP_NUM_LINKS * 2 * 2 * 2;
	u32 buff = SBD_BUFF_RGN;
	unsigned int i, d, j;

	memset_io(ipc + SBD_DESC_RGN, 0, SBD_DESC_END - SBD_DESC_RGN);
	writel(1, ipc + SBD_DESC_RGN);			/* version */
	writel(CP_NUM_LINKS, ipc + SBD_DESC_RGN + 4);
	writel(SBD_RBPS, ipc + SBD_DESC_RGN + 8);

	for (i = 0; i < CP_NUM_LINKS; i++) {
		for (d = UL; d <= DL; d++) {
			struct cp_ring *rb = &cp->ring[i][d];
			u32 rbd = SBD_RB_DESC + (i * 2 + d) * 20;

			rb->len = cp_links[i].len[d];
			rb->size = cp_links[i].size[d];
			rb->buff = buff;
			buff += rb->len * rb->size;
			rb->addr_v = desc;
			rb->size_v = desc + rb->len * 4;
			desc += rb->len * 8;
			/* rbps = {ul_rp[], ul_wp[], dl_rp[], dl_wp[]} */
			rb->rp = SBD_RBPS + ((d * 2) * CP_NUM_LINKS + i) * 2;
			rb->wp = SBD_RBPS + ((d * 2 + 1) * CP_NUM_LINKS + i) * 2;
			kfree(rb->frm);
			rb->frm = NULL;
			rb->total = rb->rcvd = 0;

			for (j = 0; j < rb->len; j++) {
				writel(rb->buff + j * rb->size, ipc + rb->addr_v + j * 4);
				writel(0, ipc + rb->size_v + j * 4);
			}

			writew(cp_links[i].ch, ipc + rbd);
			writew(0, ipc + rbd + 2);
			writew(d, ipc + rbd + 4);		/* direction */
			writew(1, ipc + rbd + 6);		/* signaling: irq */
			writel(MSG_INT_VALID | MSG_SEND_DATA, ipc + rbd + 8);
			writew(rb->len, ipc + rbd + 12);
			writew(i, ipc + rbd + 14);		/* id */
			writew(rb->size, ipc + rbd + 16);
			writew(0, ipc + rbd + 18);		/* payload_offset */

			writel(rbd, ipc + SBD_RB_CH + i * 16 + d * 8);
			writel(rb->addr_v, ipc + SBD_RB_CH + i * 16 + d * 8 + 4);
		}
	}
	wmb();
	dev_info(cp->dev, "SBD: %zu links, desc end %#x, buffers end %#x\n",
		 CP_NUM_LINKS, desc, buff);
}

/* cmd_init_start_handler: pktproc, SBD, then PIF_INIT_DONE */
static void cp_init_start(struct s5123_cp *cp)
{
	cp_print_bootlog(cp);
	cp_pp_init(cp);
	cp_sbd_init(cp);
	WRITE_ONCE(cp->sbd_up, true);
	cp_send_cmd(cp, CMD_PIF_INIT_DONE);
}

/* rild_ready: the FMT channel and the RFS channel are both open */
static bool cp_iods_open(struct s5123_cp *cp)
{
	return atomic_read(&cp->iod[0].opened) &&
	       atomic_read(&cp->iod[2].opened);
}

/* called with cp->lock */
static void cp_try_init_end(struct s5123_cp *cp)
{
	if (!READ_ONCE(cp->ipc_up) || !cp_iods_open(cp))
		return;
	cp->init_end = true;
	cp_send_cmd(cp, CMD_INIT_END);
	cp_pm_enable(cp);
}

/* cmd_phone_start_handler: init_legacy_link, then online; with cp->lock */
static void cp_phone_start(struct s5123_cp *cp)
{
	void __iomem *ipc = cp->ipc;
	unsigned int off;

	if (READ_ONCE(cp->ipc_up)) {
		/* the CP repeats CP_START every 100 ms until INIT_END */
		if (!cp->init_end)
			cp_try_init_end(cp);
		return;
	}
	writel(0, ipc + IPC_MAGIC);
	writel(0, ipc + IPC_ACCESS);
	for (off = IPC_HEADS_FIRST; off <= IPC_HEADS_LAST; off += 4)
		writel(0, ipc + off);
	writel(MEM_IPC_MAGIC, ipc + IPC_MAGIC);
	writel(1, ipc + IPC_ACCESS);
	wmb();
	if (readl(ipc + IPC_MAGIC) != MEM_IPC_MAGIC || readl(ipc + IPC_ACCESS) != 1) {
		dev_err(cp->dev, "IPC magic did not stick\n");
		return;
	}
	WRITE_ONCE(cp->ipc_up, true);
	cp->state = CP_ONLINE;
	dev_info(cp->dev, "CP_START: online\n");
	cp_try_init_end(cp);
}

/* RX */

static struct cp_iod *cp_ch2iod(struct s5123_cp *cp, u8 ch)
{
	unsigned int i;

	for (i = 0; i < CP_NUM_IODS; i++)
		if (cp->iod[i].ch == ch)
			return &cp->iod[i];
	return NULL;
}

static unsigned int sipc5_hdr_len(u8 cfg)
{
	if ((cfg & 3) == 3)
		return 5;		/* multi-frame control byte */
	if (cfg & SIPC5_EXT_LEN)
		return 6;		/* 32-bit length */
	return 4;
}

static unsigned int sipc5_frame_len(const u8 *h)
{
	if ((h[0] & 3) == SIPC5_EXT_LEN)
		return get_unaligned_le32(h + 2);
	return get_unaligned_le16(h + 2);
}

static void cp_deliver(struct s5123_cp *cp, u8 *frm, unsigned int len)
{
	unsigned int hdr = sipc5_hdr_len(frm[0]);
	struct cp_iod *iod = cp_ch2iod(cp, frm[1]);
	struct sk_buff *skb;

	cp->rx_frames++;
	if (!iod || !atomic_read(&iod->opened) || len < hdr ||
	    skb_queue_len(&iod->rxq) > 1024) {
		if (!iod)
			dev_dbg(cp->dev, "ch %u frame of %u bytes, no reader\n", frm[1], len);
		cp->rx_dropped++;
		return;
	}
	if ((frm[0] & 3) == 3)
		dev_warn_ratelimited(cp->dev, "ch %u multi-frame ctl 0x%02x\n",
				     frm[1], frm[4]);
	skb = alloc_skb(len - hdr, GFP_KERNEL);
	if (!skb) {
		cp->rx_dropped++;
		return;
	}
	skb_put_data(skb, frm + hdr, len - hdr);
	skb_queue_tail(&iod->rxq, skb);
	wake_up_interruptible(&iod->wq);
}

/* one raw IP packet of PS channel ch, from the CP's memory */
static void cp_ps_deliver(struct s5123_cp *cp, u8 ch, const void __iomem *src,
			  unsigned int len)
{
	struct net_device *ndev = READ_ONCE(cp->ps_dev);	/* with svc_lock */
	struct sk_buff *skb;
	u8 v;

	if (ch < PS_CH_FIRST || ch > PS_CH_LAST || !len || len > PP_MAX_PKT) {
		dev_err_ratelimited(cp->dev, "PS DL ch %u len %u dropped\n", ch, len);
		return;
	}
	if (!cp->ps_ch_seen[ch - PS_CH_FIRST]) {
		cp->ps_ch_seen[ch - PS_CH_FIRST] = true;
		dev_info(cp->dev, "PS DL on ch %u\n", ch);
	}
	if (!ndev || !netif_running(ndev)) {
		cp->rx_dropped++;
		return;
	}
	skb = netdev_alloc_skb(ndev, len);
	if (!skb) {
		ndev->stats.rx_dropped++;
		return;
	}
	memcpy_fromio(skb_put(skb, len), src, len);
	v = skb->data[0] >> 4;
	if (v != 4 && v != 6) {
		ndev->stats.rx_errors++;
		dev_kfree_skb_any(skb);
		return;
	}
	skb->protocol = v == 6 ? htons(ETH_P_IPV6) : htons(ETH_P_IP);
	skb_reset_mac_header(skb);
	skb_reset_network_header(skb);
	ndev->stats.rx_packets++;
	ndev->stats.rx_bytes += len;
	netif_rx(skb);
}

/* rings 8 and 9 DL: raw IP per slot, channel in buff_len[23:16] */
static void cp_rx_ps_ring(struct s5123_cp *cp, unsigned int id)
{
	struct cp_ring *rb = &cp->ring[id][DL];
	void __iomem *ipc = cp->ipc;
	u16 rp = ring_get(cp, rb->rp), wp = ring_get(cp, rb->wp);
	u32 sz, off;

	if (rp >= rb->len || wp >= rb->len) {
		dev_err_ratelimited(cp->dev, "ring %u DL dirty rp %u wp %u\n", id, rp, wp);
		return;
	}
	while (rp != wp) {
		rmb();
		sz = readl(ipc + rb->size_v + rp * 4);
		off = readl(ipc + rb->addr_v + rp * 4);
		if ((sz & 0xffff) <= rb->size && off < IPC_SIZE - (sz & 0xffff))
			cp_ps_deliver(cp, (sz >> 16) & 0xff, ipc + off, sz & 0xffff);
		else
			dev_err_ratelimited(cp->dev, "ring %u slot %u bad %#x @%#x\n",
					    id, rp, sz, off);
		mb();
		rp = (rp + 1) % rb->len;
		ring_set(cp, rb->rp, rp);
	}
}

/* pktproc: the CP advances rear_ptr; read up to it, then hand the buffers back */
static void cp_rx_pktproc(struct s5123_cp *cp)
{
	u32 rear = readl(cp->pp + PP_REAR), done = cp->pp_done, n = 0;

	if (rear >= PP_NUM) {
		dev_err_ratelimited(cp->dev, "pktproc rear %u out of range\n", rear);
		return;
	}
	while (done != rear) {
		void __iomem *d = cp->pp + PP_DESC + done * PP_DESC_SIZE;
		u32 addr, len, ch, off;

		rmb();
		addr = readl(d);
		len = readl(d + 8) & 0xffff;
		ch = (readl(d + 12) >> 16) & 0xff;
		off = addr - (PP_CP_BASE + PP_DATA);
		if (ch != 0xff && off < PP_DATA_SIZE && len <= PP_DATA_SIZE - off)
			cp_ps_deliver(cp, ch, cp->pp + PP_DATA + off, len);
		else
			dev_err_ratelimited(cp->dev, "pktproc %u: addr %#x len %u ch %u\n",
					    done, addr, len, ch);
		done = (done + 1) % PP_NUM;
		n++;
	}
	if (n) {
		cp->pp_done = done;
		cp->pp_rx += n;
		cp_pp_refill(cp);
	}
}

/* sbd_pio_rx: one SIPC5 frame per slot, or one spread over several */
static void cp_rx_ring(struct s5123_cp *cp, unsigned int id)
{
	struct cp_ring *rb = &cp->ring[id][DL];
	void __iomem *ipc = cp->ipc;
	u16 rp = ring_get(cp, rb->rp), wp = ring_get(cp, rb->wp);
	unsigned int len;
	u32 off;

	if (rp >= rb->len || wp >= rb->len) {
		dev_err_ratelimited(cp->dev, "ring %u DL dirty rp %u wp %u\n", id, rp, wp);
		return;
	}
	while (rp != wp) {
		rmb();
		len = readl(ipc + rb->size_v + rp * 4) & 0xffff;
		off = readl(ipc + rb->addr_v + rp * 4);
		if (len > rb->size || off < SBD_BUFF_RGN || off > IPC_SIZE - len) {
			dev_err_ratelimited(cp->dev, "ring %u slot %u bad len %u @%#x\n",
					    id, rp, len, off);
			goto next;
		}
		if (!rb->total) {
			u8 h[6];

			if (len < 4)
				goto next;
			memcpy_fromio(h, ipc + off, min(len, 6u));
			rb->total = sipc5_frame_len(h);
			if ((h[0] & SIPC5_START) != SIPC5_START ||
			    rb->total < 4 || rb->total > 0x40000) {
				dev_err_ratelimited(cp->dev, "ring %u bad frame %6ph\n", id, h);
				rb->total = 0;
				goto next;
			}
			rb->rcvd = 0;
			kfree(rb->frm);
			rb->frm = kmalloc(rb->total, GFP_KERNEL);
		}
		if (rb->frm)
			memcpy_fromio(rb->frm + rb->rcvd, ipc + off,
				      min(len, rb->total - rb->rcvd));
		rb->rcvd += len;
		if (rb->rcvd >= rb->total) {
			if (rb->frm)
				cp_deliver(cp, rb->frm, rb->total);
			else
				cp->rx_dropped++;
			kfree(rb->frm);
			rb->frm = NULL;
			rb->total = rb->rcvd = 0;
		}
next:
		rp = (rp + 1) % rb->len;
		ring_set(cp, rb->rp, rp);
	}
}

static void cp_rx(struct s5123_cp *cp)
{
	struct net_device *ndev;
	unsigned int i;

	if (!READ_ONCE(cp->sbd_up))
		return;
	cp_rx_pktproc(cp);
	for (i = 0; i < CP_NUM_LINKS; i++) {
		if (cp_links[i].ps)
			cp_rx_ps_ring(cp, i);
		else
			cp_rx_ring(cp, i);
	}
	ndev = READ_ONCE(cp->ps_dev);
	if (ndev && netif_queue_stopped(ndev))
		netif_wake_queue(ndev);
}

/* the CP posted a word at cp2ap_msg (MSI vector 0, or the poll) */
static void cp_service(struct s5123_cp *cp)
{
	u32 msg;

	mutex_lock(&cp->svc_lock);
	msg = readl(cp->ipc + IPC_CP2AP_MSG);
	if (msg != READ_ONCE(cp->cp2ap_msg) && (msg & MSG_CMD_VALID))
		dev_info(cp->dev, "cp2ap_msg 0x%02x\n", msg);
	WRITE_ONCE(cp->cp2ap_msg, msg);
	cp->msg_count++;

	if (!(msg & MSG_INT_VALID))
		goto out;
	if (!(msg & MSG_CMD_VALID)) {
		cp_rx(cp);
		goto out;
	}
	mutex_lock(&cp->lock);
	switch (msg & MSG_CMD_FIELD) {
	case CMD_INIT_START:
		if (!cp->sbd_up && cp->state == CP_BOOTING)
			cp_init_start(cp);
		break;
	case CMD_PHONE_START:
		if (cp->sbd_up)
			cp_phone_start(cp);
		break;
	case CMD_CRASH_RESET:
	case CMD_CRASH_EXIT:
		if (cp->state != CP_CRASHED)
			dev_err(cp->dev, "CP crashed (cmd 0x%02x), cp2ap_status %#x\n",
				msg & MSG_CMD_FIELD, readl(cp->ipc + IPC_CP2AP_STATUS));
		cp->state = CP_CRASHED;
		WRITE_ONCE(cp->ipc_up, false);
		WRITE_ONCE(cp->sbd_up, false);
		break;
	default:
		dev_info_ratelimited(cp->dev, "unhandled CP cmd 0x%02x\n",
				     msg & MSG_CMD_FIELD);
		break;
	}
	mutex_unlock(&cp->lock);
out:
	mutex_unlock(&cp->svc_lock);
	if (cp->state == CP_CRASHED) {
		unsigned int i;

		for (i = 0; i < CP_NUM_IODS; i++)
			wake_up_interruptible(&cp->iod[i].wq);
	}
	wake_up(&cp->wq);
}

static irqreturn_t cp_msg_irq(int irq, void *data)
{
	cp_service(data);
	return IRQ_HANDLED;
}

/* MSI vector 1: cp2ap_status changed (bit 2: TX flow control) */
static irqreturn_t cp_status_irq(int irq, void *data)
{
	struct s5123_cp *cp = data;
	u32 st = readl(cp->ipc + IPC_CP2AP_STATUS);

	cp->status_count++;
	dev_dbg(cp->dev, "cp2ap_status %#x\n", st);
	return IRQ_HANDLED;
}

/*
 * The handshake also polls cp2ap_msg, so a lost MSI cannot stall it; once
 * INIT_END is out only poll_ms keeps polling.
 */
static void cp_poll_work(struct work_struct *work)
{
	struct s5123_cp *cp = container_of(to_delayed_work(work), struct s5123_cp, poll);
	unsigned int ms = READ_ONCE(poll_ms);
	u32 msg = readl(cp->ipc + IPC_CP2AP_MSG);

	if (msg != cp->poll_last || ms) {
		cp->poll_last = msg;
		cp_service(cp);
	}
	if (cp->state == CP_CRASHED)
		return;
	if (!READ_ONCE(cp->init_end) || ms)
		schedule_delayed_work(&cp->poll, msecs_to_jiffies(ms ?: 10));
}

/* TX */

/* sbd_pio_tx */
static int cp_tx(struct s5123_cp *cp, struct cp_iod *iod,
		 const char __user *data, size_t count)
{
	struct cp_ring *rb;
	unsigned int len = 4 + count;
	u16 rp, wp;
	u8 *frm;
	int ret = count;

	frm = kmalloc(len, GFP_KERNEL);
	if (!frm)
		return -ENOMEM;
	frm[0] = SIPC5_START;			/* IPC_ALIGNED is off: no padding */
	frm[1] = iod->ch;
	put_unaligned_le16(len, frm + 2);
	if (copy_from_user(frm + 4, data, count)) {
		kfree(frm);
		return -EFAULT;
	}

	mutex_lock(&cp->tx_lock);
	rb = &cp->ring[iod->link][UL];
	if (cp->state != CP_ONLINE) {
		ret = -EPERM;
	} else if (len > rb->size) {
		ret = -EMSGSIZE;
	} else {
		rp = ring_get(cp, rb->rp);
		wp = ring_get(cp, rb->wp);
		if (rp >= rb->len || wp >= rb->len) {
			ret = -EIO;
		} else if ((wp + 1) % rb->len == rp) {
			ret = -EAGAIN;
		} else {
			memcpy_toio(cp->ipc + readl(cp->ipc + rb->addr_v + wp * 4), frm, len);
			writel(len, cp->ipc + rb->size_v + wp * 4);
			wmb();
			ring_set(cp, rb->wp, (wp + 1) % rb->len);
			cp_send_msg(cp, MSG_INT_VALID | MSG_SEND_DATA);
		}
	}
	mutex_unlock(&cp->tx_lock);
	kfree(frm);
	return ret;
}

/* rmnet0 */

struct cp_ps_priv {
	struct s5123_cp *cp;
};

static netdev_tx_t cp_ps_xmit(struct sk_buff *skb, struct net_device *ndev)
{
	struct s5123_cp *cp = ((struct cp_ps_priv *)netdev_priv(ndev))->cp;
	struct cp_ring *rb = &cp->ring[PS_UL_LINK][UL];
	unsigned long flags;
	u16 rp, wp;

	if (skb_linearize(skb))
		goto drop;
	spin_lock_irqsave(&cp->ps_tx_lock, flags);
	if (!READ_ONCE(cp->ipc_up) || cp->state != CP_ONLINE || skb->len > rb->size) {
		spin_unlock_irqrestore(&cp->ps_tx_lock, flags);
		goto drop;
	}
	rp = ring_get(cp, rb->rp);
	wp = ring_get(cp, rb->wp);
	if (rp >= rb->len || wp >= rb->len) {
		spin_unlock_irqrestore(&cp->ps_tx_lock, flags);
		ndev->stats.tx_errors++;
		dev_kfree_skb_any(skb);
		return NETDEV_TX_OK;
	}
	if ((wp + 1) % rb->len == rp) {
		netif_stop_queue(ndev);
		spin_unlock_irqrestore(&cp->ps_tx_lock, flags);
		return NETDEV_TX_BUSY;
	}
	memcpy_toio(cp->ipc + readl(cp->ipc + rb->addr_v + wp * 4), skb->data, skb->len);
	writel(skb->len | (READ_ONCE(ps_ul_ch) & 0xff) << 16, cp->ipc + rb->size_v + wp * 4);
	wmb();
	ring_set(cp, rb->wp, (wp + 1) % rb->len);
	cp_send_msg(cp, MSG_INT_VALID | MSG_SEND_DATA);
	spin_unlock_irqrestore(&cp->ps_tx_lock, flags);

	ndev->stats.tx_packets++;
	ndev->stats.tx_bytes += skb->len;
	dev_consume_skb_any(skb);
	return NETDEV_TX_OK;
drop:
	ndev->stats.tx_dropped++;
	dev_kfree_skb_any(skb);
	return NETDEV_TX_OK;
}

static const struct net_device_ops cp_ps_ops = {
	.ndo_start_xmit	= cp_ps_xmit,
};

static void cp_ps_setup(struct net_device *ndev)
{
	ndev->netdev_ops = &cp_ps_ops;
	ndev->type = ARPHRD_RAWIP;
	ndev->flags = IFF_POINTOPOINT | IFF_NOARP;
	ndev->hard_header_len = 0;
	ndev->addr_len = 0;
	ndev->mtu = PS_MTU;
	ndev->min_mtu = 68;
	ndev->max_mtu = 2048;
	ndev->tx_queue_len = 1000;
	ndev->needs_free_netdev = true;
}

/* umts_ipc0 / umts_ipc1 / umts_rfs0 */

static int cp_iod_open(struct inode *inode, struct file *file)
{
	struct cp_iod *iod = container_of(file->private_data, struct cp_iod, misc);
	struct s5123_cp *cp = iod->cp;

	if (atomic_inc_return(&iod->opened) == 1)
		skb_queue_purge(&iod->rxq);
	file->private_data = iod;
	mutex_lock(&cp->lock);
	if (!cp->init_end)
		cp_try_init_end(cp);
	mutex_unlock(&cp->lock);
	return 0;
}

static int cp_iod_release(struct inode *inode, struct file *file)
{
	struct cp_iod *iod = file->private_data;

	if (atomic_dec_and_test(&iod->opened))
		skb_queue_purge(&iod->rxq);
	return 0;
}

static ssize_t cp_iod_read(struct file *file, char __user *data, size_t count,
			   loff_t *ppos)
{
	struct cp_iod *iod = file->private_data;
	struct s5123_cp *cp = iod->cp;
	struct sk_buff *skb;
	size_t len;
	int ret;

	for (;;) {
		skb = skb_dequeue(&iod->rxq);
		if (skb)
			break;
		if (READ_ONCE(cp->state) == CP_CRASHED)
			return -EIO;
		if (file->f_flags & O_NONBLOCK)
			return -EAGAIN;
		ret = wait_event_interruptible(iod->wq,
					       !skb_queue_empty(&iod->rxq) ||
					       READ_ONCE(cp->state) == CP_CRASHED);
		if (ret)
			return ret;
	}
	len = min_t(size_t, skb->len, count);
	if (len < skb->len)
		dev_warn_ratelimited(cp->dev, "ch %u: read %zu of %u bytes\n",
				     iod->ch, count, skb->len);
	ret = copy_to_user(data, skb->data, len) ? -EFAULT : len;
	kfree_skb(skb);
	return ret;
}

static ssize_t cp_iod_write(struct file *file, const char __user *data,
			    size_t count, loff_t *ppos)
{
	struct cp_iod *iod = file->private_data;

	if (!count)
		return 0;
	if (count > 0x10000)
		return -EMSGSIZE;
	return cp_tx(iod->cp, iod, data, count);
}

static __poll_t cp_iod_poll(struct file *file, poll_table *wait)
{
	struct cp_iod *iod = file->private_data;
	struct s5123_cp *cp = iod->cp;
	enum cp_state st = READ_ONCE(cp->state);
	__poll_t mask = 0;

	poll_wait(file, &iod->wq, wait);
	if (!skb_queue_empty(&iod->rxq))
		mask |= EPOLLIN | EPOLLRDNORM;
	if (st == CP_ONLINE)
		mask |= EPOLLOUT | EPOLLWRNORM;
	if (st == CP_CRASHED)
		mask |= EPOLLHUP | EPOLLERR;
	return mask;
}

static const struct file_operations cp_iod_fops = {
	.owner		= THIS_MODULE,
	.open		= cp_iod_open,
	.release	= cp_iod_release,
	.read		= cp_iod_read,
	.write		= cp_iod_write,
	.poll		= cp_iod_poll,
	.llseek		= noop_llseek,
};

/* WWAN AT port, bridged to userspace */

static int cp_at_start(struct wwan_port *port)
{
	return 0;
}

static void cp_at_stop(struct wwan_port *port)
{
}

static int cp_at_tx(struct wwan_port *port, struct sk_buff *skb)
{
	struct s5123_cp *cp = wwan_port_get_drvdata(port);

	if (!atomic_read(&cp->at_bridge_open) || skb_queue_len(&cp->at_txq) > 256) {
		/* nobody translates: answer like a dead modem would, not at all */
		consume_skb(skb);
		return 0;
	}
	skb_queue_tail(&cp->at_txq, skb);
	wake_up_interruptible(&cp->at_wq);
	return 0;
}

static const struct wwan_port_ops cp_at_ops = {
	.start	= cp_at_start,
	.stop	= cp_at_stop,
	.tx	= cp_at_tx,
};

/*
 * rmnet0 lives as long as the AT port: ModemManager builds a modem from the
 * ports that show up together, so a net device registered long before the AT
 * port would be left out of it (no data port). Registered from the AT
 * bridge's open, before the port, and gone again at its release.
 */
static void cp_ps_add(struct s5123_cp *cp)
{
	struct net_device *ndev;

	ndev = alloc_netdev(sizeof(struct cp_ps_priv), "rmnet%d", NET_NAME_ENUM, cp_ps_setup);
	if (!ndev)
		return;
	((struct cp_ps_priv *)netdev_priv(ndev))->cp = cp;
	SET_NETDEV_DEV(ndev, cp->dev);
	if (register_netdev(ndev)) {
		free_netdev(ndev);
		dev_warn(cp->dev, "no PS netdev\n");
		return;
	}
	mutex_lock(&cp->svc_lock);
	WRITE_ONCE(cp->ps_dev, ndev);
	mutex_unlock(&cp->svc_lock);
}

static void cp_ps_del(struct s5123_cp *cp)
{
	struct net_device *ndev;

	mutex_lock(&cp->svc_lock);		/* the RX side reads it with svc_lock */
	ndev = cp->ps_dev;
	WRITE_ONCE(cp->ps_dev, NULL);
	mutex_unlock(&cp->svc_lock);
	if (ndev)
		unregister_netdev(ndev);	/* needs_free_netdev: freed there */
}

static int cp_atb_open(struct inode *inode, struct file *file)
{
	struct s5123_cp *cp = container_of(file->private_data, struct s5123_cp, at_bridge);
	struct wwan_port *port;

	if (atomic_cmpxchg(&cp->at_bridge_open, 0, 1))
		return -EBUSY;
	skb_queue_purge(&cp->at_txq);
	cp_ps_add(cp);
	/*
	 * The port only exists while a translator listens: ModemManager
	 * probes a new port at once and gives up on one that stays silent.
	 */
	port = wwan_create_port(cp->dev, WWAN_PORT_AT, &cp_at_ops, NULL, cp);
	if (IS_ERR(port)) {
		cp_ps_del(cp);
		atomic_set(&cp->at_bridge_open, 0);
		return PTR_ERR(port);
	}
	WRITE_ONCE(cp->at_port, port);
	file->private_data = cp;
	return 0;
}

static int cp_atb_release(struct inode *inode, struct file *file)
{
	struct s5123_cp *cp = file->private_data;

	wwan_remove_port(cp->at_port);
	WRITE_ONCE(cp->at_port, NULL);
	cp_ps_del(cp);
	atomic_set(&cp->at_bridge_open, 0);
	skb_queue_purge(&cp->at_txq);
	return 0;
}

static ssize_t cp_atb_read(struct file *file, char __user *data, size_t count,
			   loff_t *ppos)
{
	struct s5123_cp *cp = file->private_data;
	struct sk_buff *skb;
	size_t len;
	int ret;

	for (;;) {
		skb = skb_dequeue(&cp->at_txq);
		if (skb)
			break;
		if (file->f_flags & O_NONBLOCK)
			return -EAGAIN;
		ret = wait_event_interruptible(cp->at_wq, !skb_queue_empty(&cp->at_txq));
		if (ret)
			return ret;
	}
	len = min_t(size_t, skb->len, count);
	if (copy_to_user(data, skb->data, len)) {
		skb_queue_head(&cp->at_txq, skb);
		return -EFAULT;
	}
	if (len < skb->len) {
		skb_pull(skb, len);
		skb_queue_head(&cp->at_txq, skb);
	} else {
		consume_skb(skb);
	}
	return len;
}

static ssize_t cp_atb_write(struct file *file, const char __user *data,
			    size_t count, loff_t *ppos)
{
	struct s5123_cp *cp = file->private_data;
	struct sk_buff *skb;

	if (!count)
		return 0;
	count = min_t(size_t, count, 4096);
	skb = alloc_skb(count, GFP_KERNEL);
	if (!skb)
		return -ENOMEM;
	if (copy_from_user(skb_put(skb, count), data, count)) {
		kfree_skb(skb);
		return -EFAULT;
	}
	wwan_port_rx(cp->at_port, skb);
	return count;
}

static __poll_t cp_atb_poll(struct file *file, poll_table *wait)
{
	struct s5123_cp *cp = file->private_data;
	__poll_t mask = EPOLLOUT | EPOLLWRNORM;

	poll_wait(file, &cp->at_wq, wait);
	if (!skb_queue_empty(&cp->at_txq))
		mask |= EPOLLIN | EPOLLRDNORM;
	return mask;
}

static const struct file_operations cp_atb_fops = {
	.owner		= THIS_MODULE,
	.open		= cp_atb_open,
	.release	= cp_atb_release,
	.read		= cp_atb_read,
	.write		= cp_atb_write,
	.poll		= cp_atb_poll,
	.llseek		= noop_llseek,
};

/* /sys/devices/platform/s5123-cp/status */
static ssize_t status_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct s5123_cp *cp = dev_get_drvdata(dev);
	ssize_t n;

	mutex_lock(&cp->lock);
	n = sysfs_emit(buf, "state %s link %s sbd %d ipc %d init_end %d msg 0x%02x count %lu status_irqs %lu rx %lu dropped %lu pktproc %lu",
		       cp_state_name[cp->state], cp->pdev ? pci_name(cp->pdev) : "-",
		       cp->sbd_up, cp->ipc_up, cp->init_end, cp->cp2ap_msg,
		       cp->msg_count, cp->status_count, cp->rx_frames, cp->rx_dropped,
		       cp->pp_rx);
	if (cp->ipc)
		n += sysfs_emit_at(buf, n, " | magic %#x access %u ap2cp 0x%02x ap2cp_stat %#x cp2ap_stat %#x pp fore %u rear %u",
				   readl(cp->ipc + IPC_MAGIC), readl(cp->ipc + IPC_ACCESS),
				   readl(cp->ipc + IPC_AP2CP_MSG),
				   readl(cp->ipc + IPC_AP2CP_STATUS),
				   readl(cp->ipc + IPC_CP2AP_STATUS),
				   readl(cp->pp + PP_FORE), readl(cp->pp + PP_REAR));
	n += sysfs_emit_at(buf, n, " | link_pm %d link %s ups %lu downs %lu fails %lu cp2ap_wake %d ap2cp_wake %d\n",
			   READ_ONCE(cp->pm_enabled), cp->link_on ? "on" : "off",
			   cp->link_ups, cp->link_downs, cp->link_fails,
			   cp->cp2ap_wake ? gpiod_get_value(cp->cp2ap_wake) : -1,
			   cp->ap2cp_wake ? gpiod_get_value(cp->ap2cp_wake) : -1);
	mutex_unlock(&cp->lock);
	return n;
}
static DEVICE_ATTR_RO(status);

/* SBD ring pointers, one line per link: id ch UL rp/wp DL rp/wp */
static ssize_t rings_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct s5123_cp *cp = dev_get_drvdata(dev);
	ssize_t n = 0;
	unsigned int i;

	mutex_lock(&cp->lock);
	if (!cp->ipc || !READ_ONCE(cp->sbd_up)) {
		mutex_unlock(&cp->lock);
		return sysfs_emit(buf, "no rings\n");
	}
	for (i = 0; i < CP_NUM_LINKS; i++) {
		struct cp_ring *ul = &cp->ring[i][UL], *dl = &cp->ring[i][DL];

		n += sysfs_emit_at(buf, n, "%u ch %3u UL %4u/%4u of %4u DL %4u/%4u of %4u\n",
				   i, cp_links[i].ch, ring_get(cp, ul->rp), ring_get(cp, ul->wp),
				   ul->len, ring_get(cp, dl->rp), ring_get(cp, dl->wp), dl->len);
	}
	mutex_unlock(&cp->lock);
	return n;
}
static DEVICE_ATTR_RO(rings);

/* cp-boot.sh drives AP2CP_WAKE through this while it boots the CP */
static ssize_t ap2cp_wake_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct s5123_cp *cp = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", cp->ap2cp_wake ? gpiod_get_value(cp->ap2cp_wake) : -1);
}

static ssize_t ap2cp_wake_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct s5123_cp *cp = dev_get_drvdata(dev);
	bool v;
	int ret = kstrtobool(buf, &v);

	if (ret)
		return ret;
	if (!cp->ap2cp_wake)
		return -ENODEV;
	mutex_lock(&cp->pm_lock);
	gpiod_set_value(cp->ap2cp_wake, v);
	mutex_unlock(&cp->pm_lock);
	return count;
}
static DEVICE_ATTR_RW(ap2cp_wake);

static struct attribute *cp_attrs[] = {
	&dev_attr_status.attr,
	&dev_attr_ap2cp_wake.attr,
	&dev_attr_rings.attr,
	NULL
};
ATTRIBUTE_GROUPS(cp);

/* the endpoint */

static int s5123_cp_probe(struct pci_dev *pdev, const struct pci_device_id *id)
{
	struct s5123_cp *cp = the_cp;
	struct pci_host_bridge *hb = pci_find_host_bridge(pdev->bus);
	struct device_node *np = hb && hb->dev.parent ? hb->dev.parent->of_node : NULL;
	struct device *dev = &pdev->dev;
	struct resource ipc_res, pp_res;
	void __iomem *db, *ipc, *pp;
	u32 msi_addr = 0;
	int ret, nvec, i;

	if (!cp)
		return -ENODEV;
	if (!np || of_reserved_mem_region_to_resource_byname(np, "ipc", &ipc_res) ||
	    of_reserved_mem_region_to_resource_byname(np, "pktproc", &pp_res))
		return dev_err_probe(dev, -ENODEV, "no ipc/pktproc memory-region on the RC\n");
	if (resource_size(&ipc_res) < IPC_SIZE || resource_size(&pp_res) < PP_SIZE)
		return dev_err_probe(dev, -EINVAL, "ipc %pR / pktproc %pR too small\n",
				     &ipc_res, &pp_res);
	if (pci_resource_len(pdev, 0) < DB_OFFSET + 4)
		return dev_err_probe(dev, -EINVAL, "BAR0 %pR has no doorbell\n",
				     &pdev->resource[0]);

	ret = pci_enable_device(pdev);
	if (ret)
		return ret;
	pci_set_master(pdev);

	ret = -ENOMEM;
	db = pci_iomap_range(pdev, 0, DB_OFFSET, 4);
	ipc = ioremap_wc(ipc_res.start, IPC_SIZE);
	pp = ioremap_wc(pp_res.start, PP_SIZE);
	if (!db || !ipc || !pp)
		goto err_unmap;

	nvec = pci_alloc_irq_vectors(pdev, 1, 4, PCI_IRQ_MSI);
	if (nvec < 0) {
		ret = nvec;
		dev_err(dev, "no MSI: %d\n", ret);
		goto err_unmap;
	}
	if (pdev->msi_cap)
		pci_read_config_dword(pdev, pdev->msi_cap + PCI_MSI_ADDRESS_LO, &msi_addr);

	mutex_lock(&cp->lock);
	cp->pdev = pdev;
	cp->db = db;
	cp->ipc = ipc;
	cp->pp = pp;
	cp->state = CP_BOOTING;
	cp->sbd_up = cp->ipc_up = cp->init_end = false;
	cp->link_on = true;
	cp->db_reserved = false;
	cp->link_ups = cp->link_downs = cp->link_fails = 0;
	cp->cp2ap_msg = cp->poll_last = readl(ipc + IPC_CP2AP_MSG);
	cp->msg_count = cp->status_count = cp->rx_frames = cp->rx_dropped = cp->pp_rx = 0;
	memset(cp->ps_ch_seen, 0, sizeof(cp->ps_ch_seen));
	for (i = 0; i < CP_NUM_IODS; i++)
		skb_queue_purge(&cp->iod[i].rxq);
	mutex_unlock(&cp->lock);

	cp->irq_msg = pci_irq_vector(pdev, 0);
	ret = request_threaded_irq(cp->irq_msg, NULL, cp_msg_irq, IRQF_ONESHOT,
				   "s5123-cp msg", cp);
	if (ret)
		goto err_detach;
	cp->irq_status = nvec > 1 ? pci_irq_vector(pdev, 1) : -1;
	if (cp->irq_status >= 0 &&
	    request_threaded_irq(cp->irq_status, NULL, cp_status_irq, IRQF_ONESHOT,
				 "s5123-cp status", cp))
		cp->irq_status = -1;

	pci_set_drvdata(pdev, cp);
	schedule_delayed_work(&cp->poll, 0);
	dev_info(dev, "link attached: doorbell %pa, %d MSI vectors to %#x, ipc %pR, pktproc %pR, cp2ap_msg 0x%02x\n",
		 &pdev->resource[0].start, nvec, msi_addr, &ipc_res, &pp_res, cp->cp2ap_msg);
	return 0;

err_detach:
	mutex_lock(&cp->lock);
	cp->pdev = NULL;
	cp->db = cp->ipc = cp->pp = NULL;
	cp->state = CP_OFFLINE;
	mutex_unlock(&cp->lock);
	pci_free_irq_vectors(pdev);
err_unmap:
	if (pp)
		iounmap(pp);
	if (ipc)
		iounmap(ipc);
	if (db)
		pci_iounmap(pdev, db);
	pci_clear_master(pdev);
	pci_disable_device(pdev);
	return ret;
}

static void s5123_cp_remove(struct pci_dev *pdev)
{
	struct s5123_cp *cp = pci_get_drvdata(pdev);
	void __iomem *db = cp->db, *ipc = cp->ipc, *pp = cp->pp;
	unsigned long flags;
	unsigned int i;
	bool link_on;

	WRITE_ONCE(cp->pm_enabled, false);
	if (cp->wake_irq_on) {
		disable_irq(cp->irq_wake);
		cp->wake_irq_on = false;
	}
	cancel_work_sync(&cp->pm_work);
	link_on = cp->link_on;		/* down: already disabled, in D3hot */
	cancel_delayed_work_sync(&cp->poll);
	if (cp->irq_status >= 0)
		free_irq(cp->irq_status, cp);
	free_irq(cp->irq_msg, cp);
	pci_free_irq_vectors(pdev);

	/* every writer checks the state under one of these locks */
	mutex_lock(&cp->lock);
	mutex_lock(&cp->tx_lock);
	spin_lock_irqsave(&cp->ps_tx_lock, flags);
	cp->state = CP_CRASHED;		/* readers end with -EIO */
	cp->sbd_up = cp->ipc_up = cp->init_end = false;
	cp->pdev = NULL;
	cp->db = cp->ipc = cp->pp = NULL;
	spin_unlock_irqrestore(&cp->ps_tx_lock, flags);
	mutex_unlock(&cp->tx_lock);
	mutex_unlock(&cp->lock);
	for (i = 0; i < CP_NUM_IODS; i++)
		wake_up_interruptible(&cp->iod[i].wq);
	wake_up(&cp->wq);

	iounmap(pp);
	iounmap(ipc);
	pci_iounmap(pdev, db);
	if (link_on) {
		pci_clear_master(pdev);
		pci_disable_device(pdev);
	}
	dev_info(&pdev->dev, "link detached\n");
}

static const struct pci_device_id s5123_cp_ids[] = {
	{ PCI_DEVICE(PCI_VENDOR_ID_SAMSUNG, PCI_DEVICE_ID_S5123) },
	{ }
};
MODULE_DEVICE_TABLE(pci, s5123_cp_ids);

static struct pci_driver s5123_cp_pci_driver = {
	.name		= "s5123-cp",
	.id_table	= s5123_cp_ids,
	.probe		= s5123_cp_probe,
	.remove		= s5123_cp_remove,
};

/* module: the device nodes */

static void s5123_cp_unregister(struct s5123_cp *cp)
{
	unsigned int i, d;

	cp_ps_del(cp);
	for (i = 0; i < CP_NUM_IODS; i++) {
		if (cp->iod[i].misc.this_device)
			misc_deregister(&cp->iod[i].misc);
		skb_queue_purge(&cp->iod[i].rxq);
	}
	if (cp->at_bridge.this_device)
		misc_deregister(&cp->at_bridge);	/* no opener left: port is gone */
	skb_queue_purge(&cp->at_txq);
	for (i = 0; i < CP_NUM_LINKS; i++)
		for (d = UL; d <= DL; d++)
			kfree(cp->ring[i][d].frm);
}

static int s5123_cp_plat_probe(struct platform_device *plat)
{
	struct s5123_cp *cp;
	int ret, i;

	cp = devm_kzalloc(&plat->dev, sizeof(*cp), GFP_KERNEL);
	if (!cp)
		return -ENOMEM;
	mutex_init(&cp->lock);
	mutex_init(&cp->svc_lock);
	mutex_init(&cp->tx_lock);
	spin_lock_init(&cp->ps_tx_lock);
	spin_lock_init(&cp->msg_lock);
	init_waitqueue_head(&cp->wq);
	INIT_DELAYED_WORK(&cp->poll, cp_poll_work);
	mutex_init(&cp->pm_lock);
	INIT_WORK(&cp->pm_work, cp_pm_work);
	cp->state = CP_OFFLINE;
	cp->irq_status = -1;
	cp->irq_wake = -1;
	cp->link_on = true;
	cp->plat = plat;
	cp->dev = &plat->dev;
	platform_set_drvdata(plat, cp);

	/* AP2CP_WAKE stays low until cp-boot.sh raises it (ap2cp_wake) */
	cp->ap2cp_wake = devm_gpiod_get_optional(cp->dev, "ap2cp-wake", GPIOD_OUT_LOW);
	cp->cp2ap_wake = devm_gpiod_get_optional(cp->dev, "cp2ap-wake", GPIOD_IN);
	if (IS_ERR(cp->ap2cp_wake) || IS_ERR(cp->cp2ap_wake)) {
		dev_warn(cp->dev, "wake GPIOs: %d / %d\n", PTR_ERR_OR_ZERO(cp->ap2cp_wake),
			 PTR_ERR_OR_ZERO(cp->cp2ap_wake));
		cp->ap2cp_wake = IS_ERR(cp->ap2cp_wake) ? NULL : cp->ap2cp_wake;
		cp->cp2ap_wake = NULL;
	}
	if (cp->ap2cp_wake && cp->cp2ap_wake) {
		cp->irq_wake = gpiod_to_irq(cp->cp2ap_wake);
		if (cp->irq_wake <= 0 ||
		    devm_request_irq(cp->dev, cp->irq_wake, cp_wake_irq,
				     IRQF_TRIGGER_RISING | IRQF_TRIGGER_FALLING | IRQF_NO_AUTOEN,
				     "s5123 cp2ap_wake", cp)) {
			dev_warn(cp->dev, "no CP2AP_WAKE interrupt (%d): no link power management\n",
				 cp->irq_wake);
			cp->irq_wake = -1;
		}
	}
	/* left as cp-boot.sh drives it until the CP is up (cp_pm_enable) */
	cp->pda_active = devm_gpiod_get_optional(cp->dev, "pda-active", GPIOD_ASIS);
	if (IS_ERR(cp->pda_active)) {
		dev_warn(cp->dev, "no PDA_ACTIVE: %ld\n", PTR_ERR(cp->pda_active));
		cp->pda_active = NULL;
	}
	if (cp->irq_wake > 0)
		device_init_wakeup(cp->dev, true);

	skb_queue_head_init(&cp->at_txq);
	init_waitqueue_head(&cp->at_wq);
	cp->at_bridge.minor = MISC_DYNAMIC_MINOR;
	cp->at_bridge.name = "umts_atbridge";
	cp->at_bridge.fops = &cp_atb_fops;
	cp->at_bridge.parent = cp->dev;
	ret = misc_register(&cp->at_bridge);
	if (ret)
		goto err;

	for (i = 0; i < CP_NUM_IODS; i++) {
		struct cp_iod *iod = &cp->iod[i];

		iod->cp = cp;
		iod->ch = cp_iod_cfg[i].ch;
		iod->link = cp_iod_cfg[i].link;
		skb_queue_head_init(&iod->rxq);
		init_waitqueue_head(&iod->wq);
		iod->misc.minor = MISC_DYNAMIC_MINOR;
		iod->misc.name = cp_iod_cfg[i].name;
		iod->misc.fops = &cp_iod_fops;
		iod->misc.parent = cp->dev;
		ret = misc_register(&iod->misc);
		if (ret)
			goto err;
	}

	the_cp = cp;
	return 0;

err:
	s5123_cp_unregister(cp);
	return ret;
}

static void s5123_cp_plat_remove(struct platform_device *plat)
{
	struct s5123_cp *cp = platform_get_drvdata(plat);

	the_cp = NULL;
	device_init_wakeup(cp->dev, false);
	s5123_cp_unregister(cp);
	kfree(cp->ep_state);
}

/*
 * System sleep as stock's s5100 control does it (suspend/resume in its
 * modem_ctrl): no sleep while the CP holds CP2AP_WAKE or the link is up, as
 * it is using the link; PDA_ACTIVE low while the AP sleeps, so the CP knows
 * to wake it only for what needs it; CP2AP_WAKE as a wakeup interrupt.
 * Edges that come while the PCI side is suspended are acted on at resume.
 */
static int s5123_cp_suspend(struct device *dev)
{
	struct s5123_cp *cp = dev_get_drvdata(dev);

	if (!READ_ONCE(cp->pm_enabled))
		return 0;
	flush_work(&cp->pm_work);
	mutex_lock(&cp->pm_lock);
	if (gpiod_get_value(cp->cp2ap_wake) || cp->link_on) {
		mutex_unlock(&cp->pm_lock);
		dev_info(dev, "the CP is using the link (cp2ap_wake %d, link %s): not now\n",
			 gpiod_get_value(cp->cp2ap_wake), cp->link_on ? "up" : "down");
		return -EBUSY;
	}
	cp->sleeping = true;
	cp->wake_deferred = false;
	if (cp->pda_active)
		gpiod_set_value(cp->pda_active, 0);
	mutex_unlock(&cp->pm_lock);
	cp->wake_armed = device_may_wakeup(dev) && !enable_irq_wake(cp->irq_wake);
	return 0;
}

static int s5123_cp_resume(struct device *dev)
{
	struct s5123_cp *cp = dev_get_drvdata(dev);
	bool deferred;

	if (cp->wake_armed) {
		disable_irq_wake(cp->irq_wake);
		cp->wake_armed = false;
	}
	mutex_lock(&cp->pm_lock);
	if (!cp->sleeping) {
		mutex_unlock(&cp->pm_lock);
		return 0;
	}
	cp->sleeping = false;
	deferred = cp->wake_deferred;
	if (cp->pda_active)
		gpiod_set_value(cp->pda_active, 1);
	mutex_unlock(&cp->pm_lock);
	if (pm_log || deferred)
		dev_info(dev, "resume: cp2ap_wake %d%s\n", gpiod_get_value(cp->cp2ap_wake),
			 deferred ? ", it moved while asleep" : "");
	queue_work(system_highpri_wq, &cp->pm_work);
	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(s5123_cp_pm_ops, s5123_cp_suspend, s5123_cp_resume);

static struct platform_driver s5123_cp_plat_driver = {
	.driver = {
		.name = "s5123-cp",
		.dev_groups = cp_groups,
		.pm = pm_sleep_ptr(&s5123_cp_pm_ops),
	},
	.probe = s5123_cp_plat_probe,
	.remove = s5123_cp_plat_remove,
};

static struct platform_device *s5123_cp_plat;

/* stock dtbo cpif: gpio_ap2cp_wake_up gpg1-3, gpio_cp2ap_wake_up gpm30-0, gpio_ap2cp_pda_active gpm25-0 */
static struct gpiod_lookup_table s5123_cp_gpios = {
	.dev_id = "s5123-cp",
	.table = {
		GPIO_LOOKUP("gpg1", 3, "ap2cp-wake", GPIO_ACTIVE_HIGH),
		GPIO_LOOKUP("gpm30", 0, "cp2ap-wake", GPIO_ACTIVE_HIGH),
		GPIO_LOOKUP("gpm25", 0, "pda-active", GPIO_ACTIVE_HIGH),
		{ }
	},
};

static int __init s5123_cp_init(void)
{
	int ret;

	gpiod_add_lookup_table(&s5123_cp_gpios);
	ret = platform_driver_register(&s5123_cp_plat_driver);
	if (ret)
		goto err_gpios;
	s5123_cp_plat = platform_device_register_simple("s5123-cp", PLATFORM_DEVID_NONE, NULL, 0);
	if (IS_ERR(s5123_cp_plat)) {
		ret = PTR_ERR(s5123_cp_plat);
		goto err_drv;
	}
	if (!the_cp) {
		ret = -ENODEV;		/* the platform probe failed */
		goto err_dev;
	}
	ret = pci_register_driver(&s5123_cp_pci_driver);
	if (ret)
		goto err_dev;
	return 0;

err_dev:
	platform_device_unregister(s5123_cp_plat);
err_drv:
	platform_driver_unregister(&s5123_cp_plat_driver);
err_gpios:
	gpiod_remove_lookup_table(&s5123_cp_gpios);
	return ret;
}

static void __exit s5123_cp_exit(void)
{
	pci_unregister_driver(&s5123_cp_pci_driver);
	platform_device_unregister(s5123_cp_plat);
	platform_driver_unregister(&s5123_cp_plat_driver);
	gpiod_remove_lookup_table(&s5123_cp_gpios);
}

module_init(s5123_cp_init);
module_exit(s5123_cp_exit);

MODULE_DESCRIPTION("Samsung S5123 modem link over PCIe (Exynos 990)");
MODULE_LICENSE("GPL");
