/* SPDX-License-Identifier: GPL-2.0-only */
/* What the Galaxy S20 FE's panel driver needs from the DECON handover driver, its DSI host */
#ifndef __EXYNOS990_DECON_H
#define __EXYNOS990_DECON_H

#include <linux/types.h>

struct mipi_dsi_host;

unsigned int exynos990_decon_hold_frames(struct mipi_dsi_host *host, bool hold);

#endif
