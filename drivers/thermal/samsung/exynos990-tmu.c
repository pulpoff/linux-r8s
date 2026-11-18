// SPDX-License-Identifier: GPL-2.0-only
/*
 * Exynos 990 temperature sensors, read through the APM firmware
 *
 * The SoC's two TMU blocks (the CPU clusters' at 0x10090000; the GPU's, ISP's
 * and NPU's at 0x100a0000) belong to the APM firmware: the kernel asks it for
 * a sensor's temperature over the ACPM TMU channel, in whole degrees. Sensor
 * numbers are the firmware's: 0 big cores, 1 middle cores, 2 little cores,
 * 3 GPU, 4 ISP, 5 NPU.
 */

#include <linux/err.h>
#include <linux/firmware/samsung/exynos-acpm-protocol.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/thermal.h>
#include <linux/units.h>

#include "../thermal_hwmon.h"

#define EXYNOS990_TMU_CHAN	9
#define EXYNOS990_TMU_SENSORS	6

struct exynos990_tmu;

struct exynos990_tmu_sensor {
	struct exynos990_tmu *tmu;
	u8 id;
};

struct exynos990_tmu {
	struct acpm_handle *acpm;
	struct exynos990_tmu_sensor sensors[EXYNOS990_TMU_SENSORS];
};

static int exynos990_tmu_read(struct exynos990_tmu *tmu, u8 id, int *celsius)
{
	return tmu->acpm->ops->tmu.read_temp(tmu->acpm, EXYNOS990_TMU_CHAN, id,
					     celsius);
}

static int exynos990_tmu_get_temp(struct thermal_zone_device *tz, int *temp)
{
	struct exynos990_tmu_sensor *s = thermal_zone_device_priv(tz);
	int celsius, ret;

	ret = exynos990_tmu_read(s->tmu, s->id, &celsius);
	if (ret)
		return ret;

	*temp = celsius * MILLIDEGREE_PER_DEGREE;
	return 0;
}

static const struct thermal_zone_device_ops exynos990_tmu_ops = {
	.get_temp = exynos990_tmu_get_temp,
};

static int exynos990_tmu_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct exynos990_tmu *tmu;
	int i, ret, n = 0;

	tmu = devm_kzalloc(dev, sizeof(*tmu), GFP_KERNEL);
	if (!tmu)
		return -ENOMEM;

	tmu->acpm = devm_acpm_get_by_phandle(dev);
	if (IS_ERR(tmu->acpm))
		return dev_err_probe(dev, PTR_ERR(tmu->acpm), "no ACPM handle\n");

	/*
	 * The sensors already run when the kernel starts and read the same
	 * either side of INIT; the stock kernel sends it, so this one does too.
	 */
	ret = tmu->acpm->ops->tmu.init(tmu->acpm, EXYNOS990_TMU_CHAN);
	if (ret)
		dev_warn(dev, "firmware INIT failed: %d\n", ret);

	for (i = 0; i < EXYNOS990_TMU_SENSORS; i++) {
		struct exynos990_tmu_sensor *s = &tmu->sensors[i];
		struct thermal_zone_device *tzd;

		s->tmu = tmu;
		s->id = i;

		tzd = devm_thermal_of_zone_register(dev, i, s, &exynos990_tmu_ops);
		if (IS_ERR(tzd)) {
			if (PTR_ERR(tzd) == -ENODEV)
				continue;
			return dev_err_probe(dev, PTR_ERR(tzd),
					     "sensor %d: no thermal zone\n", i);
		}

		devm_thermal_add_hwmon_sysfs(dev, tzd);
		n++;
	}

	return n ? 0 : -ENODEV;
}

static const struct of_device_id exynos990_tmu_of_match[] = {
	{ .compatible = "samsung,exynos990-tmu" },
	{ }
};
MODULE_DEVICE_TABLE(of, exynos990_tmu_of_match);

static struct platform_driver exynos990_tmu_driver = {
	.probe = exynos990_tmu_probe,
	.driver = {
		.name = "exynos990-tmu",
		.of_match_table = exynos990_tmu_of_match,
	},
};
module_platform_driver(exynos990_tmu_driver);

MODULE_DESCRIPTION("Exynos 990 temperature sensors through the APM firmware");
MODULE_LICENSE("GPL");
