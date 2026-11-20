Linux 7.2.15 for the Samsung Galaxy S20 FE (r8s, SM-G780F)
==========================================================

Mainline Linux for the Samsung Galaxy S20 FE 4G (SM-G780F, codename r8s, Exynos 990), the kernel
of the phone's [VibeOS](https://vibeos.top) port. The phone boots Debian from its own storage
on all 8 cores, charges, and drives its panel, touchscreen and GPU: GNOME runs on it. It is on
Wi-Fi and LTE, answers calls with sound both ways, plays through its speakers and records
through both microphones. It reads its accelerometer, light sensor and compass, so the screen turns
with the phone. Bluetooth and the cameras are not done yet.

The tree is stable **v7.2.15** plus the S20 FE changes below, each its own commit on `main`; new
work is pushed as it lands. 7.2 is end of life on kernel.org and this tree stays on 7.2.15.

What works
==========

| Part | State |
| --- | --- |
| Boot | from the BOOT partition through a small stub and U-Boot; test kernels are started from there with kexec, under the SoC watchdog |
| Display | DRM/KMS on the pipeline the bootloader leaves running (DECON0, IDMA channel 0, DSIM0, EA8079 at 1080x2400 and 120 Hz in command mode with DSC): page flips at frame start, vblank events, fbcon. Frames go to the panel only when the picture changes; the panel refreshes itself in between |
| Panel | the EA8079 as a DSI device of DECON0: brightness (256 steps, a backlight device), and screen off and on through sleep mode with the vendor init sequence on wake (off 235 ms, on 210 ms) |
| Touchscreen | Zinitix ZT7650 on I2C, 1080x2400, through a new driver: taps and swipes drive GNOME (the quick settings and their brightness slider, the edge swipes). Several fingers at once and palm rejection are not checked yet |
| GPU | Mali-G77 MP11 (GPU_ID 0x90800011) through Panfrost, with its power domain (the secure monitor restores its TZPC after each power-up) and its clock from the APM firmware (the stock kernel's 156 to 800 MHz, devfreq). Jobs run: an offscreen GLES2 test renders the same pixels as llvmpipe. Mesa 26.2 does not know the G77, so userspace is given it as the G57, the Valhall with the same architecture: GNOME Shell draws on it (OpenGL ES; Mesa's Vulkan driver does not cover this Valhall generation) |
| Temperatures | the six SoC sensors (big, middle and little cores, GPU, ISP, NPU) through the APM firmware, as thermal zones and hwmon devices. Each zone shuts the phone down at 110 C; the CPU clusters and the GPU slow down from 75 C (the GPU from 80 C) in rungs: full load on both M5s holds at 80 to 84 C |
| Power firmware | ACPM, the APM firmware's IPC (mailbox and SRAM channels): DVFS rates for the GPU; it sets the voltage with each rate |
| PMIC | the S2MPS19 through the APM firmware: its 12 bucks and 27 LDOs as regulators (read and named; the kernel switches none of them yet), its interrupt, and its RTC, which sets the clock at boot and raises alarms |
| USB | device mode (dwc3): USB networking, ssh |
| Storage | UFS 3.1: new Exynos 990 host variant and UFS PHY, HS-G4 x2 (about 1.1 GB/s reads), IO-coherent as on stock (no more stale-descriptor errors); Debian boots from its own partition |
| RAM | all 6 GB the bootloader hands over (about 5.3 GB usable after the firmware's carve-outs); the stock carve-outs are reserved |
| Charging | MAX77705C charger and fuel gauge on a dedicated USI I2C channel: charges from USB, battery voltage, current, charge and cycle count. The float voltage is 4.2 V on purpose (stock: 4.38 V), as a development phone stays plugged in |
| Watchdog | the SoC watchdog keeps running across kexec, so a hung test kernel resets the phone; the kernel log of a panic survives in pstore (ramoops at the stock address) |
| Reboot | into recovery through the PMU's reboot-reason register (download and bootloader mode are in the DT, untested) |
| Keys | the power key (the volume keys are in the DT, untested) |
| CPUs | all 8 cores (4 Cortex-A55, 2 Cortex-A76, 2 Mongoose M5) through PSCI, with the PMU power-down handshake of the stock kernel: CPU hotplug works, and a kernel started with kexec gets every core. Frequency scaling (schedutil) through the APM firmware, on the ECT's levels: A55s 442 to 2002 MHz, A76s 507 to 2504 MHz, M5s 546 to 2730 MHz. No idle states yet |
| Scheduling | the scheduler knows the three core types (capacities from the stock firmware); HZ=1000, full preemption, schedutil, BFQ, MGLRU |
| Wi-Fi | QCA6390 on the GEN2 PCIe channel through ath11k, with linux-firmware's QCA6390 hw2.0 files: associates at boot, about 105 Mbit/s down and 86 up on 2.4 GHz. Samsung's EL2 blocks the Wi-Fi's DMA in an S2MPU until the kernel opens a window for it (`samsung,s2mpu` in the DT); a new fault detector logs any DMA it still blocks |
| Modem | S5123 on the GEN4 PCIe channel. Userspace boots it (power, its bootloader over SPI, the link, then the modem image from the phone's own partitions; the tools are VibeOS's, not in this repository); `s5123-cp` then carries Samsung IPC, the modem's remote file system, an AT port and packet data: ModemManager registers on LTE and brings `rmnet0` up |
| Calls | incoming calls ring, are answered and hung up through ModemManager (placing a call is untested: the test SIM has no credit). The call's sound runs in ABOX's own firmware, which this kernel boots for each call: the caller is heard in the earpiece or the loudspeaker and the microphones are heard by the caller. ALSA switches for the call-audio service move the call to the loudspeaker and mute the microphones |
| Sound | ABOX without its firmware: PCM to the earpiece and loudspeaker amps (two CS35L40 on one TDM bus), as PipeWire's speakers |
| Microphones | both PDM microphones through the VTS serial LIF into ABOX, as PipeWire's internal microphone ("Mic Select": main, top or both) |
| Vibrator | the MAX77705's haptic motor on a PWM timer (force feedback, feedbackd), with a minimum strength and pulse length from the DT so short, light feedback is still felt |
| Sensors | the BCM4775 B1 sensor hub on USI14's SPI, through the same driver as the Galaxy S10e's B0: its RAM patch (Samsung's, not in this repository) loads at probe, then the accelerometer, light sensor and compass are IIO devices (iio-sensor-proxy turns the screen). The S20 FE's hub has no proximity sensor |
| Torch | the RT8547 camera flash in torch mode, up to 225 mA |

Not yet
=======

Bluetooth (the QCA6390's ROM ignores the baud-rate change and its patch never starts), echo
cancellation in calls (a module option, not verified), cameras, CPU idle states and suspend.

Changes on top of 7.2.15
========================

* `soc: samsung: exynos-usi`: `samsung,sysreg` is optional, for the Exynos 990's dedicated I2C
  channels (USIxx_I2C), which have no protocol selector.
* `mfd: max77705`: accepts the MAX77705C (PMIC ID 0x15, pass 2) the S20 FE carries.
* `phy: samsung-ufs` and `scsi: ufs: exynos`: Exynos 990 UFS PHY and host variant, with the
  stock IO-coherency bits.
* `watchdog: s3c2410_wdt`: a running watchdog stays running across kexec.
* `clk: samsung: exynos990`: HSI2's bus clock stays on; USI14 and USI16 set their rates.
* `regulator: s2mps11`: the S2MPS19's LDO13 and LDO14.
* `iommu/exynos`: fault reports are rate-limited.
* `soc: samsung: exynos-s2mpufd`: reports DMA that the EL2 S2MPU blocked.
* `PCI: dwc`: the Exynos 990 GEN2 (Wi-Fi) and GEN4 (modem) root complexes.
* `misc: s5123-cp`: the S5123 modem's IPC and data over PCIe.
* `leds: rt8547`: the torch.
* `Input: max77693-haptic`: a minimum magnitude and pulse length.
* `iio: common: bcm4775_shub`: the BCM4775 sensor hub (Galaxy S10e B0, S20 FE B1).
* `ASoC: cs35l41`: 32-bit slots.
* `ASoC: samsung`: Exynos 990 ABOX (and its firmware for calls, with the call's loudspeaker and
  microphone switches), the VTS serial LIF and the S20 FE card.
* `arm64: dts: exynos990`: UFS (IO-coherent), the USI03/USI07 I2C channels, USI14's SPI, CPU
  capacities and power coefficients, the PCIe channels, ABOX, the VTS serial LIF, the PWM timers.
* `arm64: dts: exynos990-r8s`: carve-outs, ramoops, reboot modes, USB device mode, UFS supply,
  all of the RAM, the MAX77705 charger and fuel gauge, the touchscreen bus, Wi-Fi, the modem,
  the amps, the microphones, the vibrator and the torch, the sensor hub, fixed SPI bus numbers.
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
