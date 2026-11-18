Linux 7.2.15 for the Samsung Galaxy S20 FE (r8s, SM-G780F)
==========================================================

Mainline Linux for the Samsung Galaxy S20 FE 4G (SM-G780F, codename r8s, Exynos 990), the kernel
of the phone's [VibeOS](https://vibeos.top) port. Bring-up has just started: the phone boots
Debian from its own storage and charges, but there is no display driver, touch or radio yet.

The tree is stable **v7.2.15** plus the S20 FE changes below, each its own commit on `main`; new
work is pushed as it lands. 7.2 is end of life on kernel.org and this tree stays on 7.2.15.

What works
==========

| Part | State |
| --- | --- |
| Boot | from the BOOT partition through a small stub and U-Boot; test kernels are started from there with kexec, under the SoC watchdog |
| Console | the bootloader's framebuffer (1080x2400) with fbcon, through simpledrm |
| USB | device mode (dwc3): USB networking, ssh |
| Storage | UFS 3.1: new Exynos 990 host variant and UFS PHY, HS-G4 x2 (about 1.1 GB/s reads); Debian boots from its own partition |
| RAM | all 6 GB the bootloader hands over (about 5.3 GB usable after the firmware's carve-outs); the stock carve-outs are reserved |
| Charging | MAX77705C charger and fuel gauge on a dedicated USI I2C channel: charges from USB, battery voltage, current, charge and cycle count. The float voltage is 4.2 V on purpose (stock: 4.38 V), as a development phone stays plugged in |
| Watchdog | the SoC watchdog keeps running across kexec, so a hung test kernel resets the phone; the kernel log of a panic survives in pstore (ramoops at the stock address) |
| Reboot | into recovery through the PMU's reboot-reason register (download and bootloader mode are in the DT, untested) |
| Keys | the power key (the volume keys are in the DT, untested) |
| Scheduling | the scheduler knows the three core types (capacities from the stock firmware); HZ=1000, full preemption, schedutil, BFQ, MGLRU |

Not yet
=======

Display (DECON/DSIM and the panel), touchscreen (its I2C bus is up, no driver yet), Wi-Fi
(QCA6390 on PCIe), Bluetooth, sound, modem, cameras, GPU (Mali-G77), sensors, cpufreq and
suspend. A kernel started with kexec runs on one core: the firmware's PSCI CPU_ON does not bring
back cores that CPU_OFF stopped.

Changes on top of 7.2.15
========================

* `soc: samsung: exynos-usi`: `samsung,sysreg` is optional, for the Exynos 990's dedicated I2C
  channels (USIxx_I2C), which have no protocol selector.
* `mfd: max77705`: accepts the MAX77705C (PMIC ID 0x15, pass 2) the S20 FE carries.
* `phy: samsung-ufs` and `scsi: ufs: exynos`: Exynos 990 UFS PHY and host variant.
* `watchdog: s3c2410_wdt`: a running watchdog stays running across kexec.
* `arm64: dts: exynos990`: UFS, the USI03/USI07 I2C channels, CPU capacities and power
  coefficients.
* `arm64: dts: exynos990-r8s`: carve-outs, ramoops, reboot modes, USB device mode, UFS supply,
  all of the RAM, the MAX77705 charger and fuel gauge, the touchscreen bus.
* `arm64: configs: r8s_defconfig`.

Building
========

    make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- r8s_defconfig
    make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- -j$(nproc) Image exynos/exynos990-r8s.dtb

UFS is `status = "disabled"` in `exynos990-r8s.dts` for now, so a kernel that is only being
tried cannot touch the phone's storage; enable `ufs@13100000` and `phy@13104000` (for example
with `fdtput -t s exynos990-r8s.dtb /soc@0/ufs@13100000 status okay`, and the same for the PHY)
to mount it.

The boot chain (stub, U-Boot, the boot image tools and the initramfs) is not in this repository yet.
