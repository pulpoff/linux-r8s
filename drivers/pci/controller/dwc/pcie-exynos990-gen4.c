// SPDX-License-Identifier: GPL-2.0
/*
 * PCIe host controller glue for the Exynos 990 GEN4 channel (pcie@13d20000,
 * HSI2), which the Galaxy S20 FE uses for its S5123 modem.
 *
 * The sequence is the downstream Samsung one (pcie-exynos-rc.c and
 * pcie-exynos9830-rc-cal.c in the 4.19 vendor kernel, chip_ver 1 PHY table,
 * EP_SAMSUNG_MODEM paths), reduced to a cold link-up: no IOCC (the shared
 * memory is mapped uncached by its user), no L1SS, no I/A sequencer (it only
 * reruns the CDR reset on an L1.2 exit), no lane change.
 *
 * The modem's link only trains while its bootloader runs: the CP raises
 * CP2AP_WAKE_UP and expects the host to bring the link up then. So the link
 * is not started at boot. The "stage" parameter gates the probe: 0 (default)
 * = touch nothing and fail the probe, 1 = power and configure the PHY, log
 * its state and stop, 2 = full link-up and bus scan. Set it, then bind the
 * device through sysfs when the CP wants the link.
 *
 * MSI: the DesignWare iMSI-RX takes the endpoint's MSI writes, and its output
 * reaches the GIC only as a bit in the ELBI IRQ2 register. The MSI target is
 * a fixed address the CP firmware can reach (stock cp_msi_rmem), so it comes
 * from the "msi" memory region instead of the DW core's own choice.
 */
#include <linux/arm-smccc.h>
#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/iopoll.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_reserved_mem.h>
#include <linux/pci.h>
#include <linux/pcie-exynos990-gen4.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/sizes.h>

#include "pcie-designware.h"

/* ELBI (application) registers */
#define ELBI_IRQ0		0x000
#define ELBI_IRQ1		0x004
#define ELBI_IRQ2		0x008
#define ELBI_IRQ0_EN		0x010
#define ELBI_IRQ1_EN		0x014
#define ELBI_IRQ2_EN		0x018
#define ELBI_LTSSM_ENABLE	0x054
#define ELBI_APP_REQ_EXIT_L1	0x06c
#define ELBI_XMIT_PME_TURNOFF	0x118
#define ELBI_RDLH_LINKUP	0x2c8
#define ELBI_PM_DSTATE		0x2e8
#define ELBI_LINKDOWN_RST_SEL	0x3a0
#define ELBI_SOFT_RESET		0x3a4
#define ELBI_QCH_SEL		0x3a8
#define ELBI_APP_REQ_EXIT_L1_MODE 0x3bc
#define ELBI_MSTR_PEND_SEL_NAK	0x474
#define ELBI_HISTORY_CTRL	0xc00
#define ELBI_STATE_POWER_S	0xc04
#define ELBI_STATE_POWER_M	0xc08
#define ELBI_HISTORY(i)		(0xc0c + 4 * (i))
#define ELBI_PCS_PORT_RESET	0x1400
#define ELBI_PCS_PHY_INIT_RESET	0x1404
#define ELBI_PHY_CMN_RESET	0x1408

#define IRQ0_RADM_PM_TO_ACK	BIT(29)
#define IRQ1_LINK_DOWN		BIT(10)
#define IRQ2_MSI_RISING		BIT(17)
#define IRQ2_CPL_TIMEOUT	BIT(24)
#define SOFT_PWR_RESET		BIT(1)
#define L1_EXIT_HW_MODE		BIT(0)
#define L1_REQ_NAK_MASTER	BIT(4)
#define LINKDOWN_RST_MANUAL	BIT(1)
#define QCH_CLOCK_GATING_MASK	0xfff	/* PMU, APB and AXI fields */
#define HISTORY_ENABLE		0x3
#define HISTORY_COND_SEL	BIT(2)
#define LTSSM_MASK		0x3f
#define LTSSM_L0		0x11
#define LTSSM_L23_READY		0x15

/* PHY and PCS */
#define PHY_INPUT_CLK_PATH	0x0d8
#define PHY_INPUT_CLK_ON	0x28
#define PHY_INPUT_CLK_OFF	0x6d
#define PHY_ALL_PWRDN		0x400
#define PHY_AES			0x550
#define PCS_FORCE_PCLK		0x180

/* DBI */
#define DBI_PCIE_CAP		0x70
#define DBI_GEN3_RELATED	0x890
#define DBI_GEN3_EQ_OFF		0x12000
#define DBI_AUX_CLK_FREQ	0xb40
#define DBI_PM_CAP		0x40

/* Samsung EL2 hypercall: open a range in one S2MPU for one VID */
#define HVC_FID_SET_S2MPU	0xc6000100
#define S2MPU_AP_RW		3

static int stage;
module_param(stage, int, 0644);
MODULE_PARM_DESC(stage, "0 = do nothing (default), 1 = PHY config only, 2 = full link-up, "
		 "3 = one link try, dump and leave the controller powered on failure");

struct exynos990_gen4 {
	struct dw_pcie pci;
	void __iomem *elbi;
	void __iomem *phy;
	void __iomem *pcs;
	struct regmap *pmu;
	u32 pmu_offset;
	struct gpio_desc *perst;
	struct clk_bulk_data *clks;
	int num_clks;
	phys_addr_t msi_target;
	bool link_off;		/* taken to L2 by exynos990_gen4_link_off() */
};

#define to_gen4(x)	dev_get_drvdata((x)->dev)

struct phy_reg {
	u16 off;
	u8 val;
};

/* exynos_pcie_rc_pcie_phy_config(), chip_ver 1 ("9830 EVT1 GEN4 PHY"), RC */
static const struct phy_reg phy_evt1[] = {
	{ 0x018, 0x50 }, { 0x048, 0x33 }, { 0x068, 0x01 }, { 0x070, 0x12 },
	{ 0x078, 0x7f }, { 0x084, 0x0a }, { 0x088, 0x58 }, { 0x08c, 0x00 },
	{ 0x090, 0x21 }, { 0x104, 0xb6 }, { 0x458, 0x06 }, { 0x550, 0x38 },
	{ 0x5b0, 0x34 },
	/* lane 0 */
	{ 0x8d0, 0x01 }, { 0x8f4, 0x30 }, { 0x90c, 0x60 }, { 0x9b4, 0x06 },
	{ 0x9c0, 0x3f }, { 0x9c4, 0xfb }, { 0xa6c, 0x03 }, { 0xb40, 0x06 },
	{ 0xb44, 0x06 }, { 0xb48, 0x06 }, { 0xb4c, 0x06 }, { 0xb50, 0x00 },
	{ 0xb54, 0x00 }, { 0xb58, 0x00 }, { 0xb5c, 0x03 }, { 0xc44, 0x16 },
	{ 0xc48, 0x16 }, { 0xc4c, 0x16 }, { 0xc50, 0x16 }, { 0xc54, 0x0a },
	{ 0xc58, 0x0a }, { 0xc5c, 0x0a }, { 0xc60, 0x0a }, { 0xc64, 0x08 },
	{ 0xc68, 0x08 }, { 0xc6c, 0x02 }, { 0xc70, 0x0a }, { 0xca8, 0xe7 },
	{ 0xcac, 0x00 }, { 0xcb0, 0x0e }, { 0xccc, 0x1c }, { 0xcd4, 0x05 },
	{ 0xcd8, 0x77 }, { 0xcdc, 0x7a }, { 0xdb4, 0x2f }, { 0x15b4, 0x2f },
	/* lane 1 */
	{ 0x10d0, 0x01 }, { 0x10f4, 0x30 }, { 0x110c, 0x60 }, { 0x11b4, 0x06 },
	{ 0x11c0, 0x3f }, { 0x11c4, 0xfb }, { 0x126c, 0x03 }, { 0x1340, 0x06 },
	{ 0x1344, 0x06 }, { 0x1348, 0x06 }, { 0x134c, 0x06 }, { 0x1350, 0x00 },
	{ 0x1354, 0x00 }, { 0x1358, 0x00 }, { 0x135c, 0x03 }, { 0x1444, 0x16 },
	{ 0x1448, 0x16 }, { 0x144c, 0x16 }, { 0x1450, 0x16 }, { 0x1454, 0x0a },
	{ 0x1458, 0x0a }, { 0x145c, 0x0a }, { 0x1460, 0x0a }, { 0x1464, 0x08 },
	{ 0x1468, 0x08 }, { 0x146c, 0x02 }, { 0x1470, 0x0a }, { 0x14a8, 0xe7 },
	{ 0x14ac, 0x00 }, { 0x14b0, 0x0e }, { 0x14cc, 0x1c }, { 0x14d4, 0x05 },
	{ 0x14d8, 0x77 }, { 0x14dc, 0x7a },
	/* "for Gen4 0709" */
	{ 0xc70, 0x00 }, { 0xca8, 0xe7 }, { 0xcac, 0x00 }, { 0xcb0, 0x0a },
	{ 0x9b4, 0xc0 }, { 0xa6c, 0x01 }, { 0xba8, 0x01 }, { 0xc08, 0x04 },
	{ 0xc40, 0x06 }, { 0xb4c, 0x04 },
	{ 0x1470, 0x00 }, { 0x14a8, 0xe7 }, { 0x14ac, 0x00 }, { 0x14b0, 0x0a },
	{ 0x11b4, 0xc0 }, { 0x126c, 0x01 }, { 0x13a8, 0x01 }, { 0x1408, 0x04 },
	{ 0x1440, 0x06 }, { 0x134c, 0x04 },
	{ 0x450, 0x20 },
	{ 0x908, 0x05 }, { 0x1108, 0x05 },
	{ 0x840, 0x10 }, { 0x1040, 0x10 },	/* TX slew rate */
	{ 0x82c, 0x08 }, { 0x102c, 0x08 },
	/* AES off, DFE off */
	{ PHY_AES, 0x00 }, { 0xb9c, 0x01 }, { 0x139c, 0x01 },
};

static void rmw(void __iomem *reg, u32 clear, u32 set)
{
	writel((readl(reg) & ~clear) | set, reg);
}

static void gen4_pulse(struct exynos990_gen4 *g, u32 reg)
{
	writel(1, g->elbi + reg);
	udelay(10);
	writel(0, g->elbi + reg);
	mdelay(1);
}

static void gen4_dump_history(struct exynos990_gen4 *g)
{
	u32 prev = ~0, v;
	int i;

	for (i = 0; i < 32; i++) {
		v = readl(g->elbi + ELBI_HISTORY(i));
		if (v && v != prev)
			dev_info(g->pci.dev, "history %2d: ltssm %02x dstate %u l1sub %u\n",
				 i, (v >> 16) & 0x3f, (v >> 8) & 7, v & 7);
		prev = v;
	}
}

/* exynos_pcie_rc_register_dump(), non-zero rows only */
static void gen4_dump_range(struct exynos990_gen4 *g, const char *name,
			    void __iomem *base, u32 start, u32 end)
{
	u32 i, v[4];

	for (i = start; i < end; i += 16) {
		v[0] = readl(base + i);
		v[1] = readl(base + i + 4);
		v[2] = readl(base + i + 8);
		v[3] = readl(base + i + 12);
		if (v[0] | v[1] | v[2] | v[3])
			dev_info(g->pci.dev, "%s %04x: %08x %08x %08x %08x\n",
				 name, i, v[0], v[1], v[2], v[3]);
	}
}

static void gen4_dump(struct exynos990_gen4 *g)
{
	u32 v;

	gen4_dump_range(g, "elbi", g->elbi, 0x000, 0x480);
	gen4_dump_range(g, "elbi", g->elbi, 0xc00, 0xc90);
	gen4_dump_range(g, "elbi", g->elbi, 0x1400, 0x1410);
	gen4_dump_range(g, "phy", g->phy, 0x000, 0x200);
	gen4_dump_range(g, "phy", g->phy, 0x3f0, 0x400);
	gen4_dump_range(g, "phy", g->phy, 0xe00, 0xed0);
	gen4_dump_range(g, "phy", g->phy, 0xfc0, 0xfd0);
	gen4_dump_range(g, "phy", g->phy, 0x1600, 0x16d0);
	gen4_dump_range(g, "phy", g->phy, 0x17c0, 0x17d0);
	gen4_dump_range(g, "pcs", g->pcs, 0x000, 0x200);
	regmap_read(g->pmu, g->pmu_offset, &v);
	dev_info(g->pci.dev, "pmu %#x = %08x\n", g->pmu_offset, v);
}

/* exynos_pcie_rc_pcie_phy_config() */
static void gen4_phy_config(struct exynos990_gen4 *g)
{
	int i;

	gen4_pulse(g, ELBI_PCS_PHY_INIT_RESET);
	gen4_pulse(g, ELBI_PCS_PORT_RESET);
	gen4_pulse(g, ELBI_PHY_CMN_RESET);

	writel(PHY_INPUT_CLK_ON, g->phy + PHY_INPUT_CLK_PATH);
	for (i = 0; i < ARRAY_SIZE(phy_evt1); i++)
		writel(phy_evt1[i].val, g->phy + phy_evt1[i].off);

	writel(1, g->elbi + ELBI_PCS_PORT_RESET);
	mdelay(1);

	writel(0x700d5, g->pcs + 0x154);
	writel(0x700d5, g->pcs + 0x954);
	writel(0x300ff, g->pcs + 0x150);	/* GEN4 PHY: for L2 power */
}

/* The part of exynos_pcie_rc_establish_link() before dw_pcie_setup_rc() */
static void gen4_reset_and_config(struct exynos990_gen4 *g)
{
	gen4_phy_config(g);

	rmw(g->elbi + ELBI_SOFT_RESET, SOFT_PWR_RESET, 0);
	udelay(20);
	rmw(g->elbi + ELBI_SOFT_RESET, 0, SOFT_PWR_RESET);

	mdelay(1);
	writel(1, g->elbi + ELBI_PCS_PHY_INIT_RESET);
	mdelay(2);
	writel(1, g->elbi + ELBI_PHY_CMN_RESET);
	mdelay(1);

	dw_pcie_writel_dbi(&g->pci, DBI_GEN3_RELATED, DBI_GEN3_EQ_OFF);

	gpiod_set_value_cansleep(g->perst, 0);		/* PERST# released */
	usleep_range(18000, 20000);

	rmw(g->elbi + ELBI_APP_REQ_EXIT_L1_MODE, 0, L1_EXIT_HW_MODE | L1_REQ_NAK_MASTER);
	writel(LINKDOWN_RST_MANUAL, g->elbi + ELBI_LINKDOWN_RST_SEL);
	rmw(g->elbi + ELBI_QCH_SEL, QCH_CLOCK_GATING_MASK, 0);
	writel(1, g->elbi + ELBI_MSTR_PEND_SEL_NAK);	/* NAK while AXI is pending */
}

/* exynos_pcie_setup_rc(): what dw_pcie_setup_rc() doesn't do */
static void gen4_setup_rc_extras(struct exynos990_gen4 *g)
{
	struct dw_pcie *pci = &g->pci;
	u32 v;

	dw_pcie_dbi_ro_wr_en(pci);
	/* no BAR and no expansion ROM on the root port */
	dw_pcie_writel_dbi2(pci, PCI_BASE_ADDRESS_0, 0);
	dw_pcie_writel_dbi2(pci, PCI_ROM_ADDRESS1, 0);
	dw_pcie_writew_dbi(pci, PCI_VENDOR_ID, PCI_VENDOR_ID_SAMSUNG);
	dw_pcie_writew_dbi(pci, PCI_DEVICE_ID, 0xecec + 1);

	v = dw_pcie_readl_dbi(pci, DBI_PCIE_CAP + PCI_EXP_LNKCAP);
	v &= ~PCI_EXP_LNKCAP_L1EL;
	v |= FIELD_PREP(PCI_EXP_LNKCAP_L1EL, 7);	/* 64 us */
	dw_pcie_writel_dbi(pci, DBI_PCIE_CAP + PCI_EXP_LNKCAP, v);

	dw_pcie_writel_dbi(pci, DBI_AUX_CLK_FREQ, 0x1a);	/* 26 MHz */
	dw_pcie_writel_dbi(pci, DBI_PM_CAP + PCI_PM_CTRL, 0);

	v = dw_pcie_readl_dbi(pci, DBI_PCIE_CAP + PCI_EXP_DEVCTL2);
	v &= ~PCI_EXP_DEVCTL2_COMP_TIMEOUT;
	v |= 0x2;					/* 4.1 .. 6.2 ms */
	dw_pcie_writel_dbi(pci, DBI_PCIE_CAP + PCI_EXP_DEVCTL2, v);
	dw_pcie_dbi_ro_wr_dis(pci);
}

static void gen4_power_down(struct exynos990_gen4 *g)
{
	if (stage >= 3) {
		dev_info(g->pci.dev, "stage 3: left powered for inspection\n");
		return;
	}
	gpiod_set_value_cansleep(g->perst, 1);
	writel(0, g->elbi + ELBI_LTSSM_ENABLE);
	writel(0x23, g->phy + PHY_ALL_PWRDN);
	writel(PHY_INPUT_CLK_OFF, g->phy + PHY_INPUT_CLK_PATH);
	regmap_update_bits(g->pmu, g->pmu_offset, BIT(0), 0);	/* isolated again */
	clk_bulk_disable_unprepare(g->num_clks, g->clks);
}

static int gen4_host_init(struct dw_pcie_rp *pp)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct exynos990_gen4 *g = to_gen4(pci);
	int ret;

	if (stage < 1)
		return -ENODEV;

	ret = clk_bulk_prepare_enable(g->num_clks, g->clks);
	if (ret)
		return ret;

	/* exynos_pcie_rc_poweron() */
	regmap_update_bits(g->pmu, g->pmu_offset, BIT(0), BIT(0));	/* PHY isolation off */
	writel(PHY_INPUT_CLK_ON, g->phy + PHY_INPUT_CLK_PATH);
	writel(0, g->phy + PHY_ALL_PWRDN);
	writel(0x0d, g->pcs + PCS_FORCE_PCLK);

	rmw(g->elbi + ELBI_HISTORY_CTRL, HISTORY_COND_SEL, 0);
	writel(~0, g->elbi + ELBI_STATE_POWER_S);
	writel(~0, g->elbi + ELBI_STATE_POWER_M);
	rmw(g->elbi + ELBI_HISTORY_CTRL, 0, HISTORY_ENABLE);

	writel(0, g->elbi + ELBI_IRQ0_EN);
	writel(0, g->elbi + ELBI_IRQ1_EN);
	writel(0, g->elbi + ELBI_IRQ2_EN);

	gen4_reset_and_config(g);
	dev_info(pci->dev, "phy configured: ltssm %02x pcs 0x180 %08x phy 0xd8 %02x\n",
		 readl(g->elbi + ELBI_RDLH_LINKUP) & LTSSM_MASK,
		 readl(g->pcs + PCS_FORCE_PCLK), readl(g->phy + PHY_INPUT_CLK_PATH));
	if (stage < 2) {
		gen4_power_down(g);
		return -ENODEV;
	}

	gen4_setup_rc_extras(g);
	return 0;
}

static void gen4_host_deinit(struct dw_pcie_rp *pp)
{
	gen4_power_down(to_gen4(to_dw_pcie_from_pp(pp)));
}

static const struct dw_pcie_host_ops gen4_host_ops = {
	.init = gen4_host_init,
	.deinit = gen4_host_deinit,
};

static bool gen4_link_up(struct dw_pcie *pci)
{
	struct exynos990_gen4 *g = to_gen4(pci);
	u32 v = readl(g->elbi + ELBI_RDLH_LINKUP) & LTSSM_MASK;

	return v >= LTSSM_L0 && v <= 0x14;
}

static int gen4_start_link(struct dw_pcie *pci)
{
	struct exynos990_gen4 *g = to_gen4(pci);
	u16 lnksta;
	u32 v;
	int try, ret;

	/*
	 * The DW core points its iMSI-RX at the config window; the CP writes
	 * its MSIs to a fixed address. Move the target there before any
	 * endpoint composes a message (dw_pcie_setup_rc() reuses msi_data on
	 * the retries below).
	 */
	if (g->msi_target && pci->pp.use_imsi_rx) {
		pci->pp.msi_data = g->msi_target;
		dw_pcie_writel_dbi(pci, PCIE_MSI_ADDR_LO, lower_32_bits(g->msi_target));
		dw_pcie_writel_dbi(pci, PCIE_MSI_ADDR_HI, upper_32_bits(g->msi_target));
	}

	for (try = 0; try < (stage >= 3 ? 1 : 10); try++) {
		if (try) {
			gen4_reset_and_config(g);
			dw_pcie_setup_rc(&pci->pp);
			gen4_setup_rc_extras(g);
		}

		usleep_range(48000, 50000);
		writel(1, g->elbi + ELBI_LTSSM_ENABLE);

		/* downstream: 12000 x 10 us */
		ret = readl_poll_timeout(g->elbi + ELBI_RDLH_LINKUP, v,
					 (v & LTSSM_MASK) == LTSSM_L0, 10, 120000);
		if (!ret) {
			bool quiet = g->link_off;	/* a link_on(), not the first */

			writel(readl(g->elbi + ELBI_IRQ0), g->elbi + ELBI_IRQ0);
			writel(readl(g->elbi + ELBI_IRQ1), g->elbi + ELBI_IRQ1);
			writel(readl(g->elbi + ELBI_IRQ2), g->elbi + ELBI_IRQ2);
			writel(IRQ1_LINK_DOWN, g->elbi + ELBI_IRQ1_EN);
			writel(IRQ2_CPL_TIMEOUT |
			       (pci_msi_enabled() ? IRQ2_MSI_RISING : 0),
			       g->elbi + ELBI_IRQ2_EN);
			lnksta = dw_pcie_readw_dbi(pci, DBI_PCIE_CAP + PCI_EXP_LNKSTA);
			if (!quiet || try)
				dev_info(pci->dev, "link up: Gen%u x%u (try %d, dstate %u)\n",
					 lnksta & PCI_EXP_LNKSTA_CLS,
					 FIELD_GET(PCI_EXP_LNKSTA_NLW, lnksta), try,
					 readl(g->elbi + ELBI_PM_DSTATE) & 7);
			return 0;
		}

		dev_info(pci->dev, "no link, ltssm %02x (try %d)\n", v & LTSSM_MASK, try);
		gen4_dump_history(g);
		if (stage >= 3) {
			gen4_dump(g);
			break;
		}
		gpiod_set_value_cansleep(g->perst, 1);
		writel(0, g->elbi + ELBI_LTSSM_ENABLE);
	}

	return -ETIMEDOUT;
}

static void gen4_stop_link(struct dw_pcie *pci)
{
	struct exynos990_gen4 *g = to_gen4(pci);

	writel(0, g->elbi + ELBI_LTSSM_ENABLE);
	gpiod_set_value_cansleep(g->perst, 1);
}

static const struct dw_pcie_ops gen4_dw_ops = {
	.link_up = gen4_link_up,
	.start_link = gen4_start_link,
	.stop_link = gen4_stop_link,
};

static struct platform_driver gen4_driver;

static struct exynos990_gen4 *gen4_from_ep(struct pci_dev *ep)
{
	struct pci_host_bridge *bridge = pci_find_host_bridge(ep->bus);
	struct device *dev = bridge ? bridge->dev.parent : NULL;

	if (!dev || dev->driver != &gen4_driver.driver)
		return NULL;
	return dev_get_drvdata(dev);
}

/*
 * exynos_pcie_rc_poweroff() up to PERST#: PME_Turn_Off, wait for the
 * endpoint's PME_TO_Ack and L2/L3 Ready, then PERST# and LTSSM off. The
 * controller itself stays powered and clocked, so a config access while the
 * link is down fails in dw_pcie_other_conf_map_bus() (no link) instead of
 * reaching an unclocked DBI. The caller saves the endpoint's config space
 * before and restores it after exynos990_gen4_link_on(), as stock's modem
 * driver does (s51xx_pcie_save_state/restore_state).
 */
int exynos990_gen4_link_off(struct pci_dev *ep)
{
	struct exynos990_gen4 *g = gen4_from_ep(ep);
	struct device *dev;
	u32 v;
	int ret;

	if (!g)
		return -ENODEV;
	if (g->link_off)
		return 0;
	dev = g->pci.dev;

	/* nothing to report while it is down; link_on() enables them again */
	writel(0, g->elbi + ELBI_IRQ1_EN);
	writel(0, g->elbi + ELBI_IRQ2_EN);

	v = readl(g->elbi + ELBI_RDLH_LINKUP) & 0x1f;
	if (v >= 0x0d && v <= 0x14) {
		writel(IRQ0_RADM_PM_TO_ACK, g->elbi + ELBI_IRQ0);
		writel(1, g->elbi + ELBI_APP_REQ_EXIT_L1);
		rmw(g->elbi + ELBI_APP_REQ_EXIT_L1_MODE, L1_EXIT_HW_MODE, L1_REQ_NAK_MASTER);
		writel(1, g->elbi + ELBI_XMIT_PME_TURNOFF);
		ret = readl_poll_timeout(g->elbi + ELBI_IRQ0, v, v & IRQ0_RADM_PM_TO_ACK,
					 10, 20000);
		writel(0, g->elbi + ELBI_XMIT_PME_TURNOFF);
		if (ret)
			dev_warn(dev, "no PME_TO_Ack from the endpoint\n");
		ret = readl_poll_timeout(g->elbi + ELBI_RDLH_LINKUP, v,
					 (v & 0x1f) == LTSSM_L23_READY, 10, 20000);
		if (ret)
			dev_warn(dev, "no L2/L3 Ready (ltssm %02x)\n", v & LTSSM_MASK);
		writel(IRQ0_RADM_PM_TO_ACK, g->elbi + ELBI_IRQ0);
	} else {
		dev_warn(dev, "link_off: link not up (ltssm %02x)\n", v);
	}

	writel(0x0d, g->pcs + PCS_FORCE_PCLK);
	gpiod_set_value_cansleep(g->perst, 1);
	writel(0, g->elbi + ELBI_LTSSM_ENABLE);
	g->link_off = true;
	return 0;
}
EXPORT_SYMBOL_GPL(exynos990_gen4_link_off);

/* exynos_pcie_rc_establish_link() again, for a link exynos990_gen4_link_off() took down */
int exynos990_gen4_link_on(struct pci_dev *ep)
{
	struct exynos990_gen4 *g = gen4_from_ep(ep);
	int ret;

	if (!g)
		return -ENODEV;
	if (!g->link_off)
		return 0;

	gen4_reset_and_config(g);
	dw_pcie_setup_rc(&g->pci.pp);
	gen4_setup_rc_extras(g);
	ret = gen4_start_link(&g->pci);
	if (ret)
		return ret;
	g->link_off = false;
	return 0;
}
EXPORT_SYMBOL_GPL(exynos990_gen4_link_on);

static irqreturn_t gen4_irq(int irq, void *arg)
{
	struct exynos990_gen4 *g = arg;
	struct dw_pcie_rp *pp = &g->pci.pp;
	u32 irq0 = readl(g->elbi + ELBI_IRQ0);
	u32 irq1 = readl(g->elbi + ELBI_IRQ1);
	u32 irq2 = readl(g->elbi + ELBI_IRQ2);

	writel(irq0, g->elbi + ELBI_IRQ0);
	writel(irq1, g->elbi + ELBI_IRQ1);
	writel(irq2, g->elbi + ELBI_IRQ2);

	if (irq1 & IRQ1_LINK_DOWN)
		dev_warn(g->pci.dev, "link down (ltssm %02x)\n",
			 readl(g->elbi + ELBI_RDLH_LINKUP) & LTSSM_MASK);
	if (irq2 & IRQ2_CPL_TIMEOUT)
		dev_warn(g->pci.dev, "completion timeout\n");

	if (IS_ENABLED(CONFIG_PCI_MSI) && (irq2 & IRQ2_MSI_RISING) && pp->use_imsi_rx) {
		unsigned long flags;
		u32 mask;

		dw_handle_msi_irq(pp);
		/*
		 * The ELBI bit is the iMSI-RX output's rising edge: a vector
		 * raised while the others were handled keeps the output high
		 * without a new edge. Masking every vector and restoring the
		 * mask drops and raises it again (downstream does the same).
		 */
		raw_spin_lock_irqsave(&pp->lock, flags);
		mask = dw_pcie_readl_dbi(&g->pci, PCIE_MSI_INTR0_MASK);
		dw_pcie_writel_dbi(&g->pci, PCIE_MSI_INTR0_MASK, ~0);
		dw_pcie_writel_dbi(&g->pci, PCIE_MSI_INTR0_MASK, mask);
		raw_spin_unlock_irqrestore(&pp->lock, flags);
	}

	return (irq0 | irq1 | irq2) ? IRQ_HANDLED : IRQ_NONE;
}

/*
 * Samsung's EL2 puts a Stage-2 Protection Unit (S2MPU) in front of HSI2 and
 * blocks the PCIe master until told otherwise. Open exactly the memory
 * regions the modem shares with the AP (never all of RAM: that would hand
 * the CP firmware the whole AP memory), for the instance and VIDs in
 * "samsung,s2mpu" = <instance vid...>.
 */
static void gen4_s2mpu_grant(struct device *dev)
{
	struct device_node *np = dev->of_node;
	int i, n, nvid = of_property_count_u32_elems(np, "samsung,s2mpu");
	u32 instance, vid;

	if (nvid < 2 || of_property_read_u32_index(np, "samsung,s2mpu", 0, &instance))
		return;
	if (instance == 0)	/* "aud": a SET there hangs EL2 */
		return;

	n = of_count_phandle_with_args(np, "memory-region", NULL);
	for (i = 0; i < n; i++) {
		const char *name = NULL;
		struct resource res;
		int v;

		of_property_read_string_index(np, "memory-region-names", i, &name);
		if (name && !strcmp(name, "msi"))
			continue;	/* the iMSI-RX catches those writes */
		if (of_reserved_mem_region_to_resource(np, i, &res))
			continue;
		for (v = 1; v < nvid; v++) {
			struct arm_smccc_res r;

			of_property_read_u32_index(np, "samsung,s2mpu", v, &vid);
			arm_smccc_hvc(HVC_FID_SET_S2MPU, (vid << 16) | instance,
				      res.start, resource_size(&res), S2MPU_AP_RW,
				      0, 0, 0, &r);
			dev_info(dev, "S2MPU %u VID %u: %pR %s\n", instance, vid, &res,
				 r.a0 ? "refused" : "open");
		}
	}

	/* register pages the master must reach: "samsung,s2mpu-mmio" = <base size>... */
	n = of_property_count_u32_elems(np, "samsung,s2mpu-mmio");
	for (i = 0; i + 1 < n; i += 2) {
		u32 base, size;
		int v;

		of_property_read_u32_index(np, "samsung,s2mpu-mmio", i, &base);
		of_property_read_u32_index(np, "samsung,s2mpu-mmio", i + 1, &size);
		for (v = 1; v < nvid; v++) {
			struct arm_smccc_res r;

			of_property_read_u32_index(np, "samsung,s2mpu", v, &vid);
			arm_smccc_hvc(HVC_FID_SET_S2MPU, (vid << 16) | instance, base, size,
				      S2MPU_AP_RW, 0, 0, 0, &r);
			dev_info(dev, "S2MPU %u VID %u: MMIO %#x+%#x %s\n", instance, vid,
				 base, size, r.a0 ? "refused" : "open");
		}
	}
}

static int gen4_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct exynos990_gen4 *g;
	struct resource res;
	int i, irq, ret;

	if (stage < 1)
		return -ENODEV;	/* see the top: bound later, when the CP wants the link */

	g = devm_kzalloc(dev, sizeof(*g), GFP_KERNEL);
	if (!g)
		return -ENOMEM;

	g->pci.dev = dev;
	g->pci.ops = &gen4_dw_ops;
	g->pci.pp.ops = &gen4_host_ops;
	/* MSI through the ELBI interrupt, not a chained one (see the top) */
	g->pci.pp.msi_irq[0] = -ENODEV;
	platform_set_drvdata(pdev, g);

	g->elbi = devm_platform_ioremap_resource_byname(pdev, "elbi");
	if (IS_ERR(g->elbi))
		return PTR_ERR(g->elbi);
	g->pci.elbi_base = g->elbi;
	g->phy = devm_platform_ioremap_resource_byname(pdev, "phy");
	if (IS_ERR(g->phy))
		return PTR_ERR(g->phy);
	g->pcs = devm_platform_ioremap_resource_byname(pdev, "pcs");
	if (IS_ERR(g->pcs))
		return PTR_ERR(g->pcs);

	g->pmu = syscon_regmap_lookup_by_phandle_args(dev->of_node, "samsung,pmu-syscon",
						      1, &g->pmu_offset);
	if (IS_ERR(g->pmu))
		return dev_err_probe(dev, PTR_ERR(g->pmu), "no PMU syscon\n");

	ret = devm_clk_bulk_get_all(dev, &g->clks);
	if (ret < 0)
		return dev_err_probe(dev, ret, "no clocks\n");
	g->num_clks = ret;

	/* Asserted (line low) until the link is brought up */
	g->perst = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(g->perst))
		return dev_err_probe(dev, PTR_ERR(g->perst), "no PERST# gpio\n");

	i = of_property_match_string(dev->of_node, "memory-region-names", "msi");
	if (i >= 0 && !of_reserved_mem_region_to_resource(dev->of_node, i, &res))
		g->msi_target = res.start;

	irq = platform_get_irq(pdev, 0);
	if (irq < 0)
		return irq;
	ret = devm_request_irq(dev, irq, gen4_irq, IRQF_SHARED, dev_name(dev), g);
	if (ret)
		return ret;

	gen4_s2mpu_grant(dev);

	return dw_pcie_host_init(&g->pci.pp);
}

static void gen4_remove(struct platform_device *pdev)
{
	struct exynos990_gen4 *g = platform_get_drvdata(pdev);

	dw_pcie_host_deinit(&g->pci.pp);
}

/*
 * The S5123 reports class 0 ("not defined"), and the PCI core never assigns
 * the BARs of a classless device. Call it what it is so they get addresses.
 */
static void gen4_fixup_s5123_class(struct pci_dev *dev)
{
	if ((dev->class >> 8) == PCI_CLASS_NOT_DEFINED)
		dev->class = PCI_CLASS_COMMUNICATION_OTHER << 8;
}
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_SAMSUNG, 0xa5a5, gen4_fixup_s5123_class);

static const struct of_device_id gen4_of_match[] = {
	{ .compatible = "samsung,exynos990-pcie-gen4" },
	{ }
};
MODULE_DEVICE_TABLE(of, gen4_of_match);

static struct platform_driver gen4_driver = {
	.probe = gen4_probe,
	.remove = gen4_remove,
	.driver = {
		.name = "exynos990-pcie-gen4",
		.of_match_table = gen4_of_match,
	},
};
module_platform_driver(gen4_driver);

MODULE_DESCRIPTION("Exynos 990 GEN4 PCIe host controller (modem channel)");
MODULE_LICENSE("GPL");
