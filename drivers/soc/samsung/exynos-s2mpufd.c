// SPDX-License-Identifier: GPL-2.0-only
/*
 * Exynos S2MPU fault detector
 *
 * Samsung's EL2 owns the Stage-2 Protection Units (S2MPUs) in front of the
 * SoC's DMA masters and raises these interrupts when one blocks an access.
 * The fault record stays in EL2: register a buffer for it once
 * (INIT_S2MPUFD); each GET_S2MPUFD_FAIL_INFO then fills that channel's entry
 * and clears the interrupt.  Without this driver a blocked DMA is silent:
 * the device only sees its read fail, and the interrupt stays pending.
 */
#include <linux/arm-smccc.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/mod_devicetable.h>
#include <linux/platform_device.h>

#define HVC_INIT_S2MPUFD		0xc6000601
#define HVC_GET_S2MPUFD_FAIL_INFO	0xc6000602
#define S2MPUFD_MAX_CH			2

/* Layout shared with EL2 */
struct s2mpufd_fail_info {
	u32 intr_stat;
	u32 addr_low;
	u32 addr_high;
	u32 vid;
	u32 rw;
	u32 len;
	u32 axid;
};

struct s2mpufd {
	struct device *dev;
	struct s2mpufd_fail_info *info;
	dma_addr_t info_pa;
	unsigned int nch;
	int irq[S2MPUFD_MAX_CH];
};

static unsigned long s2mpufd_hvc(unsigned long fid, unsigned long a1,
				 unsigned long a2, unsigned long a3)
{
	struct arm_smccc_res res;

	arm_smccc_hvc(fid, a1, a2, a3, 0, 0, 0, 0, &res);
	return res.a0;
}

static irqreturn_t s2mpufd_irq(int irq, void *data)
{
	struct s2mpufd *fd = data;
	struct s2mpufd_fail_info *f;
	unsigned int ch;

	for (ch = 0; ch < fd->nch; ch++)
		if (fd->irq[ch] == irq)
			break;
	if (ch == fd->nch)
		return IRQ_NONE;

	s2mpufd_hvc(HVC_GET_S2MPUFD_FAIL_INFO, fd->info_pa, ch, 0);
	f = &fd->info[ch];
	dev_err_ratelimited(fd->dev,
			    "blocked %s of %#x bytes at %#llx by VID %u, AxID %#x (channel %u)\n",
			    f->rw ? "write" : "read", f->len,
			    ((u64)f->addr_high << 32) | f->addr_low,
			    f->vid, f->axid, ch);
	return IRQ_HANDLED;
}

static int s2mpufd_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct s2mpufd *fd;
	unsigned long r;
	int i, n, ret;

	fd = devm_kzalloc(dev, sizeof(*fd), GFP_KERNEL);
	if (!fd)
		return -ENOMEM;
	fd->dev = dev;

	n = platform_irq_count(pdev);
	if (n <= 0)
		return n ? n : -EINVAL;
	fd->nch = min(n, S2MPUFD_MAX_CH);

	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;
	/* EL2 keeps this address: the device can never unbind */
	fd->info = dmam_alloc_coherent(dev, sizeof(*fd->info) * fd->nch,
				       &fd->info_pa, GFP_KERNEL);
	if (!fd->info)
		return -ENOMEM;

	r = s2mpufd_hvc(HVC_INIT_S2MPUFD, fd->info_pa, fd->nch,
			sizeof(struct s2mpufd_fail_info));
	if (r)
		return dev_err_probe(dev, -ENODEV, "EL2 refused INIT_S2MPUFD: %#lx\n", r);

	for (i = 0; i < fd->nch; i++) {
		fd->irq[i] = platform_get_irq(pdev, i);
		if (fd->irq[i] < 0)
			return fd->irq[i];
		ret = devm_request_irq(dev, fd->irq[i], s2mpufd_irq, 0,
				       dev_name(dev), fd);
		if (ret)
			return ret;
	}
	return 0;
}

static const struct of_device_id s2mpufd_of_match[] = {
	{ .compatible = "samsung,exynos990-s2mpufd" },
	{ }
};

static struct platform_driver s2mpufd_driver = {
	.probe = s2mpufd_probe,
	.driver = {
		.name = "exynos-s2mpufd",
		.of_match_table = s2mpufd_of_match,
		.suppress_bind_attrs = true,
	},
};
builtin_platform_driver(s2mpufd_driver);
