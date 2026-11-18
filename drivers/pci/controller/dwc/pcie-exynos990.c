// SPDX-License-Identifier: GPL-2.0
/*
 * PCIe host controller glue for the Exynos 990 GEN2 channel (pcie@133b0000),
 * which the Galaxy S20 FE uses for its Wi-Fi: a QCA6390 behind one lane.
 *
 * The sequence is the downstream Samsung one (pcie-exynos-host-v0.c and
 * pcie-exynos9830-v0-cal.c in the 4.19 vendor kernel, EP_QC_WIFI paths),
 * reduced to a cold link-up: no L1SS, no runtime PM, no I/A sequencer (it only
 * reruns the CDR reset on an L1.2 exit), SysMMU in bypass. The PHY setup is
 * interleaved with resets in the controller's ELBI block, so it lives here
 * rather than in a separate PHY driver. It is the Exynos 9820 driver's layout
 * (same controller generation), with the 9830 PHY tables and the QC resets.
 *
 * MSI: the DesignWare iMSI-RX takes the endpoint's MSI writes, and its output
 * reaches the GIC only as a bit in the ELBI pulse interrupt (the controller's
 * one SPI). So the DW core's MSI domain is used without its chained handler,
 * and this driver's interrupt handler runs the MSI demux.
 *
 * Bring-up aid: the "stage" parameter stops early. 0 = only log the state
 * left by the bootloader (ELBI and syscons), 1 = also power and configure the
 * PHY and log the PLL state, 2 (default) = full link-up and bus scan.
 */
#include <linux/arm-smccc.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/iopoll.h>
#include <linux/ioport.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/sizes.h>

#include "pcie-designware.h"

/* ELBI (application) registers */
#define ELBI_IRQ_PULSE		0x000
#define ELBI_IRQ_LEVEL		0x004
#define ELBI_IRQ_SPECIAL	0x008
#define ELBI_IRQ_EN_PULSE	0x00c
#define ELBI_IRQ_EN_LEVEL	0x010
#define ELBI_IRQ_EN_SPECIAL	0x014
#define ELBI_LTSSM_ENABLE	0x02c
#define ELBI_IRQ_EN_PULSE2	0x034
#define ELBI_IRQ_EN_PULSE3	0x038
#define ELBI_RDLH_LINKUP	0x074
#define ELBI_PM_DSTATE		0x088
#define ELBI_HISTORY(i)		(0x138 + 4 * (i))
#define ELBI_APP_REQ_EXIT_L1_MODE 0x0f4
#define ELBI_LINKDOWN_RST_SEL	0x1b8
#define ELBI_SOFT_CORE_RESET	0x1d0
#define ELBI_HISTORY_CTRL	0x274
#define ELBI_INIT_RSTN		0x288
#define ELBI_CMN_RSTN		0x28c
#define ELBI_LANE_RSTN		0x290
#define ELBI_TRSV_RSTN		0x294
#define ELBI_STATE_POWER_S	0x2bc
#define ELBI_STATE_POWER_M	0x2c0
#define ELBI_QCH_SEL		0x2c8
#define ELBI_L12ERR_CTRL	0x2f0

#define IRQ_INTX_MASK		0x55	/* INTA..INTD */
#define IRQ_MSI			BIT(29)
#define IRQ_LINK_DOWN		BIT(30)
#define LTSSM_MASK		0x1f
#define L1_EXIT_HW_MODE		BIT(0)	/* in APP_REQ_EXIT_L1_MODE */
#define L1_REQ_NAK_MASTER	BIT(4)
#define LINKDOWN_RST_MANUAL	BIT(1)
#define QCH_CLOCK_GATING_MASK	0xfff	/* PMU, APB and AXI fields */

/* PCS registers */
#define PCS_L1SS_TIMEOUT	0x00c
#define PCS_ELECIDLE		0x0ec
#define PCS_ELECIDLE_IGNORE	BIT(3)
#define PCS_TX_LATENCY		0x0f8
#define PCS_REFCLK_CTRL		0x100
#define PCS_REFCLK_CTRL2	0x104

#define PHY_PLL_STATE		0x0bc

/* syscons */
#define PMU_PCIE_PHY_CTRL	0x718
#define SYSREG_PCIE_PHY_CTRL	0x1044
#define SYSREG_PCIE_PHY_CTRL2	0x1050

/* DBI extras (downstream exynos_pcie_setup_rc) */
#define DBI_LINK_CTRL_STAT	0x080	/* the root port's PCI_EXP_LNKCTL */
#define DBI_LNKCTL_EXT_SYNCH	BIT(7)
#define DBI_AUX_CLK_FREQ	0xb40
#define DBI_L1_SUBSTATES	0xb44

/* Samsung EL2 hypercall: open a range in one S2MPU for one VID */
#define HVC_FID_SET_S2MPU	0xc6000100
#define S2MPU_AP_RW		3

static int stage = 2;
module_param(stage, int, 0444);
MODULE_PARM_DESC(stage, "0 = log only, 1 = + PHY config, 2 = full link-up");

struct exynos990_pcie {
	struct dw_pcie pci;
	void __iomem *elbi;
	void __iomem *phy;
	void __iomem *pcs;
	struct regmap *pmu;
	struct regmap *sysreg;
	struct gpio_desc *perst;
	struct regulator *vpcie;
};

#define to_exynos990(x)	dev_get_drvdata((x)->dev)

/* "Ch0 HS1 Gen2" PHY tables, 26 MHz reference (pcie-exynos9830-v0-cal.c) */
static const u8 phy_cmn[48] = {
	0x01, 0xe1, 0x05, 0x00, 0x88, 0x88, 0x88, 0x0c,
	0x71, 0x45, 0x65, 0x24, 0x33, 0x18, 0xe3, 0xfc,
	0xd8, 0x05, 0xe6, 0x80, 0x00, 0x00, 0x00, 0x00,
	0x60, 0x11, 0x00, 0xa0, 0x05, 0x04, 0x18, 0x88,
	0xc0, 0xff, 0x9b, 0x52, 0x22, 0x30, 0x4f, 0xdc,
	0x40, 0x00, 0x00, 0x00, 0x00, 0x0b, 0x00, 0x80,
};

static const u8 phy_trsv[80] = {
	0x31, 0x40, 0x37, 0x99, 0x85, 0x00, 0xc0, 0xff,
	0xff, 0x3f, 0x8c, 0xc8, 0x02, 0x01, 0x88, 0x80,
	0x06, 0x90, 0x6c, 0x66, 0x09, 0x61, 0x42, 0x44,
	0xc6, 0x50, 0x0a, 0x33, 0x58, 0xe7, 0x20, 0x22,
	0x80, 0x38, 0x05, 0x85, 0x00, 0x00, 0x00, 0x7e,
	0x00, 0x00, 0x55, 0x15, 0xac, 0xaa, 0x3e, 0x00,
	0x00, 0x00, 0x20, 0x3f, 0x00, 0x03, 0x40, 0x00,
	0x10, 0x07, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40,
	0x05, 0x85, 0xff, 0x00, 0x00, 0x00, 0x00, 0x00,
};

static void rmw(void __iomem *reg, u32 clear, u32 set)
{
	writel((readl(reg) & ~clear) | set, reg);
}

/*
 * Only the ELBI and the syscons are safe to read before the PHY is
 * un-isolated and configured: the PHY and PCS abort while the PMU isolates
 * them, and a DBI access before the PHY runs hangs the SoC (Exynos 9820).
 */
static void exynos990_pcie_log_state(struct exynos990_pcie *ep, const char *when,
				     bool phy_up)
{
	struct device *dev = ep->pci.dev;
	u32 pmu = 0, s0 = 0, s1 = 0;

	regmap_read(ep->pmu, PMU_PCIE_PHY_CTRL, &pmu);
	regmap_read(ep->sysreg, SYSREG_PCIE_PHY_CTRL, &s0);
	regmap_read(ep->sysreg, SYSREG_PCIE_PHY_CTRL2, &s1);
	dev_info(dev, "%s: pmu %08x sysreg %08x/%08x ltssm %02x l12err %08x qch %08x\n",
		 when, pmu, s0, s1, readl(ep->elbi + ELBI_RDLH_LINKUP) & LTSSM_MASK,
		 readl(ep->elbi + ELBI_L12ERR_CTRL), readl(ep->elbi + ELBI_QCH_SEL));
	if (phy_up)
		dev_info(dev, "%s: pcs elecidle %08x refclk %08x/%08x phy pll %02x\n", when,
			 readl(ep->pcs + PCS_ELECIDLE), readl(ep->pcs + PCS_REFCLK_CTRL),
			 readl(ep->pcs + PCS_REFCLK_CTRL2), readl(ep->phy + PHY_PLL_STATE));
}

static void exynos990_pcie_dump_history(struct exynos990_pcie *ep)
{
	int i;

	for (i = 0; i < 32; i++) {
		u32 v = readl(ep->elbi + ELBI_HISTORY(i));

		if (v)
			dev_info(ep->pci.dev, "history %2d: ltssm %02x dstate %u l1sub %u\n",
				 i, (v >> 16) & 0x3f, (v >> 8) & 7, v & 7);
	}
}

/* exynos_phy_all_pwrdn_clear(), ch_num 0 */
static void exynos990_pcie_phy_pwrdn_clear(struct exynos990_pcie *ep)
{
	regmap_update_bits(ep->sysreg, SYSREG_PCIE_PHY_CTRL2, BIT(12), BIT(12));
	rmw(ep->pcs + PCS_REFCLK_CTRL, BIT(6), 0);
	rmw(ep->pcs + PCS_REFCLK_CTRL2, BIT(7), 0);
	writel(0xc0, ep->phy + 0x20 * 4);	/* common */
	writel(0x7e, ep->phy + 0x57 * 4);	/* transceiver */
}

/* exynos_phy_all_pwrdn(), ch_num 0 */
static void exynos990_pcie_phy_pwrdn(struct exynos990_pcie *ep)
{
	writel(0xfe, ep->phy + 0x57 * 4);
	writel(0xc1, ep->phy + 0x20 * 4);
	rmw(ep->pcs + PCS_REFCLK_CTRL, 0, BIT(6));
	rmw(ep->pcs + PCS_REFCLK_CTRL2, 0, BIT(7));
}

/* exynos_pcie_phy_config(), ch_num 0, EP_QC_WIFI */
static void exynos990_pcie_phy_config(struct exynos990_pcie *ep)
{
	int i;

	/* PCS_HIGH_SPEED, then the PHY mode fields and Lane 0 */
	regmap_update_bits(ep->sysreg, SYSREG_PCIE_PHY_CTRL, BIT(1), BIT(1));
	regmap_update_bits(ep->sysreg, SYSREG_PCIE_PHY_CTRL2,
			   GENMASK(7, 1), 0x3 << 2);
	regmap_update_bits(ep->sysreg, SYSREG_PCIE_PHY_CTRL2, BIT(12), BIT(12));

	/* All four PHY resets pulsed; INIT comes out of it first */
	writel(1, ep->elbi + ELBI_INIT_RSTN);
	writel(1, ep->elbi + ELBI_LANE_RSTN);
	writel(1, ep->elbi + ELBI_TRSV_RSTN);
	writel(1, ep->elbi + ELBI_CMN_RSTN);
	udelay(10);
	writel(0, ep->elbi + ELBI_INIT_RSTN);
	writel(0, ep->elbi + ELBI_LANE_RSTN);
	writel(0, ep->elbi + ELBI_TRSV_RSTN);
	writel(0, ep->elbi + ELBI_CMN_RSTN);
	udelay(10);
	writel(1, ep->elbi + ELBI_INIT_RSTN);

	for (i = 0; i < ARRAY_SIZE(phy_cmn); i++)
		writel(phy_cmn[i], ep->phy + i * 4);
	for (i = 0; i < ARRAY_SIZE(phy_trsv); i++)
		writel(phy_trsv[i], ep->phy + (0x30 + i) * 4);

	writel(0x70, ep->pcs + PCS_TX_LATENCY);
	writel(0x89, ep->pcs + PCS_REFCLK_CTRL);	/* REFCLK out to the EP (QC value) */
	writel(0x50, ep->pcs + PCS_REFCLK_CTRL2);
	rmw(ep->pcs + PCS_L1SS_TIMEOUT, BIT(1) | BIT(4), 0);

	rmw(ep->phy + 0x11c, 0xf << 1, 0x4 << 1);
	rmw(ep->phy + 0x120, 0x1f << 1, 0x6 << 1);

	mdelay(1);
	writel(1, ep->elbi + ELBI_LANE_RSTN);
	writel(1, ep->elbi + ELBI_TRSV_RSTN);
	writel(1, ep->elbi + ELBI_CMN_RSTN);
	mdelay(5);
}

/* The part of exynos_pcie_establish_link() before the LTSSM is enabled (EP_QC_WIFI order) */
static void exynos990_pcie_reset_and_config(struct exynos990_pcie *ep)
{
	rmw(ep->pcs + PCS_ELECIDLE, 0, PCS_ELECIDLE_IGNORE);

	exynos990_pcie_phy_config(ep);

	writel(0, ep->elbi + ELBI_SOFT_CORE_RESET);
	udelay(20);
	writel(1, ep->elbi + ELBI_SOFT_CORE_RESET);
	usleep_range(10000, 12000);

	gpiod_set_value_cansleep(ep->perst, 0);		/* PERST# released */
	usleep_range(18000, 20000);

	rmw(ep->elbi + ELBI_APP_REQ_EXIT_L1_MODE, 0, L1_EXIT_HW_MODE | L1_REQ_NAK_MASTER);
	writel(LINKDOWN_RST_MANUAL, ep->elbi + ELBI_LINKDOWN_RST_SEL);
	rmw(ep->elbi + ELBI_QCH_SEL, QCH_CLOCK_GATING_MASK, 0);
}

static int exynos990_pcie_host_init(struct dw_pcie_rp *pp)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct exynos990_pcie *ep = to_exynos990(pci);
	int ret;

	exynos990_pcie_log_state(ep, "bootloader", false);
	if (stage < 1)
		return -ENODEV;

	if (ep->vpcie) {
		ret = regulator_enable(ep->vpcie);	/* VDD_WIFI_0P95 + WLAN_EN */
		if (ret)
			return ret;
	}

	regmap_update_bits(ep->pmu, PMU_PCIE_PHY_CTRL, BIT(0), BIT(0));
	exynos990_pcie_phy_pwrdn_clear(ep);
	writel(0, ep->elbi + ELBI_L12ERR_CTRL);		/* NCLK_OFF: hang, not error */

	writel(IRQ_INTX_MASK | (pci_msi_enabled() ? IRQ_MSI : 0),
	       ep->elbi + ELBI_IRQ_EN_PULSE);
	writel(0, ep->elbi + ELBI_IRQ_EN_PULSE2);
	writel(0, ep->elbi + ELBI_IRQ_EN_PULSE3);
	writel(0, ep->elbi + ELBI_IRQ_EN_LEVEL);
	writel(0, ep->elbi + ELBI_IRQ_EN_SPECIAL);

	exynos990_pcie_reset_and_config(ep);
	exynos990_pcie_log_state(ep, "phy configured", true);
	if (stage < 2) {
		gpiod_set_value_cansleep(ep->perst, 1);
		if (ep->vpcie)
			regulator_disable(ep->vpcie);
		return -ENODEV;
	}

	/* PCS keeps ignoring RX electrical idle until start_link */
	dw_pcie_dbi_ro_wr_en(pci);
	dw_pcie_writel_dbi(pci, DBI_AUX_CLK_FREQ, 0x1a);	/* 26 MHz */
	dw_pcie_writel_dbi(pci, DBI_L1_SUBSTATES, 0xea);
	dw_pcie_writel_dbi(pci, DBI_LINK_CTRL_STAT,
			   dw_pcie_readl_dbi(pci, DBI_LINK_CTRL_STAT) | DBI_LNKCTL_EXT_SYNCH);
	dw_pcie_dbi_ro_wr_dis(pci);

	return 0;
}

static void exynos990_pcie_host_deinit(struct dw_pcie_rp *pp)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct exynos990_pcie *ep = to_exynos990(pci);

	gpiod_set_value_cansleep(ep->perst, 1);
	exynos990_pcie_phy_pwrdn(ep);
	regmap_update_bits(ep->pmu, PMU_PCIE_PHY_CTRL, BIT(0), 0);	/* isolated again */
	if (ep->vpcie)
		regulator_disable(ep->vpcie);
}

static const struct dw_pcie_host_ops exynos990_pcie_host_ops = {
	.init = exynos990_pcie_host_init,
	.deinit = exynos990_pcie_host_deinit,
};

static bool exynos990_pcie_link_up(struct dw_pcie *pci)
{
	struct exynos990_pcie *ep = to_exynos990(pci);
	u32 v = readl(ep->elbi + ELBI_RDLH_LINKUP) & LTSSM_MASK;

	return v >= 0x0d && v <= 0x15;
}

static int exynos990_pcie_start_link(struct dw_pcie *pci)
{
	struct exynos990_pcie *ep = to_exynos990(pci);
	u32 v;
	int try, ret;

	for (try = 0; try < 10; try++) {
		if (try)
			exynos990_pcie_reset_and_config(ep);

		rmw(ep->pcs + PCS_ELECIDLE, PCS_ELECIDLE_IGNORE, 0);
		writel(0, ep->elbi + ELBI_HISTORY_CTRL);
		writel(3, ep->elbi + ELBI_HISTORY_CTRL);
		writel(0x200000, ep->elbi + ELBI_STATE_POWER_S);
		writel(0xffffffff, ep->elbi + ELBI_STATE_POWER_M);
		writel(1, ep->elbi + ELBI_LTSSM_ENABLE);

		/* downstream: 12000 x 10 us */
		ret = readl_poll_timeout(ep->elbi + ELBI_RDLH_LINKUP, v,
					 (v & LTSSM_MASK) >= 0x0d &&
					 (v & LTSSM_MASK) <= 0x14, 10, 120000);
		if (!ret) {
			writel(readl(ep->elbi + ELBI_IRQ_PULSE), ep->elbi + ELBI_IRQ_PULSE);
			rmw(ep->elbi + ELBI_IRQ_EN_PULSE, 0, IRQ_LINK_DOWN);
			dev_info(pci->dev, "link up (ltssm %02x, try %d, dstate %u)\n",
				 v & LTSSM_MASK, try, readl(ep->elbi + ELBI_PM_DSTATE) & 7);
			return 0;
		}

		dev_info(pci->dev, "no link, ltssm %02x (try %d)\n", v & LTSSM_MASK, try);
		exynos990_pcie_dump_history(ep);
		gpiod_set_value_cansleep(ep->perst, 1);
		writel(0, ep->elbi + ELBI_LTSSM_ENABLE);
	}

	return -ETIMEDOUT;
}

static void exynos990_pcie_stop_link(struct dw_pcie *pci)
{
	struct exynos990_pcie *ep = to_exynos990(pci);

	writel(0, ep->elbi + ELBI_LTSSM_ENABLE);
	gpiod_set_value_cansleep(ep->perst, 1);
}

static const struct dw_pcie_ops exynos990_dw_pcie_ops = {
	.link_up = exynos990_pcie_link_up,
	.start_link = exynos990_pcie_start_link,
	.stop_link = exynos990_pcie_stop_link,
};

static irqreturn_t exynos990_pcie_irq(int irq, void *arg)
{
	struct exynos990_pcie *ep = arg;
	struct dw_pcie_rp *pp = &ep->pci.pp;
	u32 pulse = readl(ep->elbi + ELBI_IRQ_PULSE);
	u32 level = readl(ep->elbi + ELBI_IRQ_LEVEL);
	u32 special = readl(ep->elbi + ELBI_IRQ_SPECIAL);

	/* W1C; the INTx consumer shares the line and runs either way */
	writel(pulse, ep->elbi + ELBI_IRQ_PULSE);
	writel(level, ep->elbi + ELBI_IRQ_LEVEL);
	writel(special, ep->elbi + ELBI_IRQ_SPECIAL);

	if (pulse & IRQ_LINK_DOWN)
		dev_warn(ep->pci.dev, "link down\n");

	if (IS_ENABLED(CONFIG_PCI_MSI) && (pulse & IRQ_MSI) && pp->use_imsi_rx) {
		unsigned long flags;
		u32 mask;

		dw_handle_msi_irq(pp);
		/*
		 * The pulse bit is the iMSI-RX output's rising edge. A vector
		 * raised while the others were being handled keeps that output
		 * high without a new edge: masking every vector and restoring
		 * the mask drops and raises it again (downstream does the same).
		 */
		raw_spin_lock_irqsave(&pp->lock, flags);
		mask = dw_pcie_readl_dbi(&ep->pci, PCIE_MSI_INTR0_MASK);
		dw_pcie_writel_dbi(&ep->pci, PCIE_MSI_INTR0_MASK, ~0);
		dw_pcie_writel_dbi(&ep->pci, PCIE_MSI_INTR0_MASK, mask);
		raw_spin_unlock_irqrestore(&pp->lock, flags);
	}

	return (pulse | level | special) ? IRQ_HANDLED : IRQ_NONE;
}

/*
 * Samsung's EL2 puts a Stage-2 Protection Unit (S2MPU) in front of HSI1, and
 * it blocks the PCIe master until told otherwise: the QCA6390's first DMA
 * read (its SBL image over BHI) fails with MHI ERROR_CODE 0xef128100, and
 * EL2's fault detector logs a blocked READ from VID 1.  The vendor kernel
 * opens ranges with the SET_S2MPU hypercall; open every System RAM range
 * for the instance and VID named in "samsung,s2mpu" before anything DMAs.
 */
struct exynos990_s2mpu_grant {
	struct device *dev;
	u32 instance;
	u32 vid;
	unsigned int failed;
};

static int exynos990_pcie_s2mpu_grant_one(struct resource *res, void *arg)
{
	struct exynos990_s2mpu_grant *g = arg;
	u64 start = round_down(res->start, SZ_4K);
	u64 size = round_up(res->end + 1, SZ_4K) - start;
	struct arm_smccc_res r;

	arm_smccc_hvc(HVC_FID_SET_S2MPU, (g->vid << 16) | g->instance,
		      start, size, S2MPU_AP_RW, 0, 0, 0, &r);
	if (r.a0) {
		dev_warn(g->dev, "S2MPU grant %#llx+%#llx refused: %#lx\n",
			 start, size, r.a0);
		g->failed++;
	}
	return 0;
}

static void exynos990_pcie_s2mpu_grant(struct device *dev)
{
	struct exynos990_s2mpu_grant g = { .dev = dev };

	if (of_property_read_u32_index(dev->of_node, "samsung,s2mpu", 0, &g.instance) ||
	    of_property_read_u32_index(dev->of_node, "samsung,s2mpu", 1, &g.vid))
		return;

	walk_system_ram_res(0, U64_MAX, &g, exynos990_pcie_s2mpu_grant_one);
	if (!g.failed)
		dev_info(dev, "S2MPU %u: VID %u may DMA to System RAM\n",
			 g.instance, g.vid);
}

static int exynos990_pcie_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct exynos990_pcie *ep;
	int irq, ret;

	ep = devm_kzalloc(dev, sizeof(*ep), GFP_KERNEL);
	if (!ep)
		return -ENOMEM;

	ep->pci.dev = dev;
	ep->pci.ops = &exynos990_dw_pcie_ops;
	ep->pci.pp.ops = &exynos990_pcie_host_ops;
	/* MSI through the ELBI interrupt, not a chained one (see the top) */
	ep->pci.pp.msi_irq[0] = -ENODEV;
	platform_set_drvdata(pdev, ep);

	ep->elbi = devm_platform_ioremap_resource_byname(pdev, "elbi");
	if (IS_ERR(ep->elbi))
		return PTR_ERR(ep->elbi);
	ep->pci.elbi_base = ep->elbi;	/* or the DW core maps it a second time */
	ep->phy = devm_platform_ioremap_resource_byname(pdev, "phy");
	if (IS_ERR(ep->phy))
		return PTR_ERR(ep->phy);
	ep->pcs = devm_platform_ioremap_resource_byname(pdev, "pcs");
	if (IS_ERR(ep->pcs))
		return PTR_ERR(ep->pcs);

	ep->pmu = syscon_regmap_lookup_by_phandle(dev->of_node, "samsung,pmu-syscon");
	if (IS_ERR(ep->pmu))
		return dev_err_probe(dev, PTR_ERR(ep->pmu), "no PMU syscon\n");
	ep->sysreg = syscon_regmap_lookup_by_phandle(dev->of_node, "samsung,sysreg");
	if (IS_ERR(ep->sysreg))
		return dev_err_probe(dev, PTR_ERR(ep->sysreg), "no sysreg\n");

	/* Asserted (line low) until the link is brought up */
	ep->perst = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ep->perst))
		return dev_err_probe(dev, PTR_ERR(ep->perst), "no PERST# gpio\n");

	ep->vpcie = devm_regulator_get_optional(dev, "vpcie");
	if (IS_ERR(ep->vpcie)) {
		if (PTR_ERR(ep->vpcie) != -ENODEV)
			return PTR_ERR(ep->vpcie);
		ep->vpcie = NULL;
	}

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;
	ret = devm_request_irq(dev, irq, exynos990_pcie_irq, IRQF_SHARED,
			       dev_name(dev), ep);
	if (ret)
		return ret;

	exynos990_pcie_s2mpu_grant(dev);

	return dw_pcie_host_init(&ep->pci.pp);
}

static void exynos990_pcie_remove(struct platform_device *pdev)
{
	struct exynos990_pcie *ep = platform_get_drvdata(pdev);

	dw_pcie_host_deinit(&ep->pci.pp);
}

static const struct of_device_id exynos990_pcie_of_match[] = {
	{ .compatible = "samsung,exynos990-pcie" },
	{ }
};
MODULE_DEVICE_TABLE(of, exynos990_pcie_of_match);

static struct platform_driver exynos990_pcie_driver = {
	.probe = exynos990_pcie_probe,
	.remove = exynos990_pcie_remove,
	.driver = {
		.name = "exynos990-pcie",
		.of_match_table = exynos990_pcie_of_match,
		.suppress_bind_attrs = true,
	},
};
module_platform_driver(exynos990_pcie_driver);

MODULE_DESCRIPTION("Exynos 990 GEN2 PCIe host controller (Wi-Fi channel)");
MODULE_LICENSE("GPL");
