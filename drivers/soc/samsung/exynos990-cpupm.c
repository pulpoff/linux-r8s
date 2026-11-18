// SPDX-License-Identifier: GPL-2.0-only
/*
 * Exynos 990 CPU power-down handshake for CPU hotplug
 *
 * As on gs101, a core's power-down has to be set up in the PMU before PSCI
 * CPU_OFF: the power-mode hint in its CPU_INFORM register, its bit in the PMU
 * interrupt generator's group 2 (the wake-up that powers it on again) and its
 * own INT_EN bit 3 (power it down at its next WFI). Without it, CPU_OFF leaves
 * the core powered in the firmware's last WFI and CPU_ON never revives it; a
 * kexec leaves every secondary core that way.
 *
 * The vendor kernel does it on the core itself, as late on the way down and as
 * early on the way up as it can: in a hotplug step that runs with IRQs off just
 * before the core dies, and in one that runs before it first idles. The steps
 * are set up before the secondary cores start: after a kexec they come up still
 * armed to lose power at their first WFI, and must be disarmed before that.
 */

#include <linux/bits.h>
#include <linux/cpuhotplug.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/printk.h>
#include <linux/soc/samsung/exynos-regs-pmu.h>
#include <linux/spinlock.h>
#include <linux/topology.h>

#define CPU_INFORM_CLEAR	0
#define CPU_INFORM_C2		1
#define CPU_INT_EN_POWER_DOWN	BIT(3)

static const unsigned int exynos990_clusters[] = {
	GS101_CLUSTER0_OFFSET,
	GS101_CLUSTER1_OFFSET,
	GS101_CLUSTER2_OFFSET,
};

static void __iomem *pmu;
static void __iomem *intr_gen;
static DEFINE_RAW_SPINLOCK(lock);

static void exynos990_cpu_pmu(unsigned int cpu, bool down)
{
	int cluster = topology_cluster_id(cpu);
	int core = topology_core_id(cpu);
	void __iomem *int_en;
	u32 bit = BIT(cpu);
	u32 val;

	if (WARN_ON_ONCE(cluster < 0 || cluster >= ARRAY_SIZE(exynos990_clusters) || core < 0))
		return;

	int_en = pmu + GS101_CLUSTER_CPU_INT_EN(exynos990_clusters[cluster], core);

	raw_spin_lock(&lock);

	writel(down ? CPU_INFORM_C2 : CPU_INFORM_CLEAR, pmu + GS101_CPU_INFORM(cpu));

	val = readl(intr_gen + GS101_GRP2_INTR_BID_ENABLE);
	writel(down ? val | bit : val & ~bit, intr_gen + GS101_GRP2_INTR_BID_ENABLE);

	if (down) {
		/* stale power-down events of this core */
		val = readl(intr_gen + GS101_GRP1_INTR_BID_UPEND);
		writel(val & (bit | bit << 8), intr_gen + GS101_GRP1_INTR_BID_CLEAR);
	} else {
		/* the wake-up that brought it here */
		val = readl(intr_gen + GS101_GRP2_INTR_BID_UPEND);
		writel(val & bit, intr_gen + GS101_GRP2_INTR_BID_CLEAR);
	}

	val = readl(int_en);
	writel(down ? val | CPU_INT_EN_POWER_DOWN : val & ~CPU_INT_EN_POWER_DOWN, int_en);

	raw_spin_unlock(&lock);
}

static int exynos990_cpu_power_up(unsigned int cpu)
{
	exynos990_cpu_pmu(cpu, false);
	return 0;
}

static int exynos990_cpu_power_down(unsigned int cpu)
{
	exynos990_cpu_pmu(cpu, true);
	return 0;
}

static int __init exynos990_cpupm_init(void)
{
	struct device_node *np __free(device_node) =
		of_find_compatible_node(NULL, NULL, "samsung,exynos990-pmu");
	struct device_node *intr_gen_np __free(device_node) = NULL;
	int ret;

	if (!np)
		return 0;

	intr_gen_np = of_parse_phandle(np, "google,pmu-intr-gen-syscon", 0);
	if (!intr_gen_np) {
		pr_warn("exynos990-cpupm: no pmu-intr-gen, CPUs can not come back once off\n");
		return 0;
	}

	pmu = of_iomap(np, 0);
	intr_gen = of_iomap(intr_gen_np, 0);
	if (!pmu || !intr_gen) {
		ret = -ENOMEM;
		goto unmap;
	}

	ret = cpuhp_setup_state_nocalls(CPUHP_AP_EXYNOS_CPU_POWER_UP, "soc/exynos990:power-up",
					exynos990_cpu_power_up, NULL);
	if (ret)
		goto unmap;

	ret = cpuhp_setup_state_nocalls(CPUHP_AP_EXYNOS_CPU_POWER_DOWN, "soc/exynos990:power-down",
					NULL, exynos990_cpu_power_down);
	if (ret) {
		cpuhp_remove_state_nocalls(CPUHP_AP_EXYNOS_CPU_POWER_UP);
		goto unmap;
	}

	return 0;

unmap:
	if (intr_gen)
		iounmap(intr_gen);
	if (pmu)
		iounmap(pmu);
	pr_err("exynos990-cpupm: %d, CPUs can not come back once off\n", ret);
	return ret;
}
/* before the secondary cores start (smp_init runs after the early initcalls) */
early_initcall(exynos990_cpupm_init);
