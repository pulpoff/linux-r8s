// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Link Layer for Samsung S3FWRN5 NCI based Driver
 *
 * Copyright (C) 2015 Samsung Electronics
 * Robert Baldyga <r.baldyga@samsung.com>
 * Copyright (C) 2020 Samsung Electronics
 * Bongsu Jeon <bongsu.jeon@samsung.com>
 */

#include <linux/delay.h>
#include <linux/module.h>

#include "phy_common.h"

void s3fwrn5_phy_set_wake(void *phy_id, bool wake)
{
	struct phy_common *phy = phy_id;

	mutex_lock(&phy->mutex);
	gpiod_set_value(phy->gpio_fw_wake, wake);
	if (wake)
		msleep(S3FWRN5_EN_WAIT_TIME);
	mutex_unlock(&phy->mutex);
}
EXPORT_SYMBOL(s3fwrn5_phy_set_wake);

static void s3fwrn5_phy_pvdd(struct phy_common *phy, bool on)
{
	int ret;

	if (!phy->pvdd || phy->pvdd_on == on)
		return;

	ret = on ? regulator_enable(phy->pvdd) : regulator_disable(phy->pvdd);
	if (ret) {
		pr_err("s3fwrn5: PVDD %s failed: %d\n", on ? "on" : "off", ret);
		return;
	}
	phy->pvdd_on = on;
	if (on)
		msleep(S3FWRN5_PVDD_WAIT_TIME);
}

bool s3fwrn5_phy_power_ctrl(struct phy_common *phy, enum s3fwrn5_mode mode)
{
	unsigned int wait = phy->en_wait_ms ?: S3FWRN5_EN_WAIT_TIME;

	if (phy->mode == mode)
		return false;

	phy->mode = mode;

	gpiod_set_value(phy->gpio_en, 1);
	gpiod_set_value(phy->gpio_fw_wake, 0);

	if (mode == S3FWRN5_MODE_COLD) {
		/* held in power-down: the supply can go */
		s3fwrn5_phy_pvdd(phy, false);
		return true;
	}

	/* the supply first, the chip still held in power-down */
	s3fwrn5_phy_pvdd(phy, true);

	if (mode == S3FWRN5_MODE_FW)
		gpiod_set_value(phy->gpio_fw_wake, 1);

	msleep(wait);
	gpiod_set_value(phy->gpio_en, 0);
	msleep(wait);

	return true;
}
EXPORT_SYMBOL(s3fwrn5_phy_power_ctrl);

void s3fwrn5_phy_set_mode(void *phy_id, enum s3fwrn5_mode mode)
{
	struct phy_common *phy = phy_id;

	mutex_lock(&phy->mutex);

	s3fwrn5_phy_power_ctrl(phy, mode);

	mutex_unlock(&phy->mutex);
}
EXPORT_SYMBOL(s3fwrn5_phy_set_mode);

enum s3fwrn5_mode s3fwrn5_phy_get_mode(void *phy_id)
{
	struct phy_common *phy = phy_id;
	enum s3fwrn5_mode mode;

	mutex_lock(&phy->mutex);

	mode = phy->mode;

	mutex_unlock(&phy->mutex);

	return mode;
}
EXPORT_SYMBOL(s3fwrn5_phy_get_mode);
