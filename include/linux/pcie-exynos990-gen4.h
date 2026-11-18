/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Link power control of the Exynos 990 GEN4 PCIe channel, for the driver of
 * the device behind it (the S5123 modem takes its link to L2 whenever its
 * CP2AP_WAKE line drops, and back up when it rises).
 */
#ifndef _LINUX_PCIE_EXYNOS990_GEN4_H
#define _LINUX_PCIE_EXYNOS990_GEN4_H

struct pci_dev;

int exynos990_gen4_link_off(struct pci_dev *ep);
int exynos990_gen4_link_on(struct pci_dev *ep);

#endif
