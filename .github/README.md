# Linux for MediaTek MT6785 phones

This branch is a Linux stable release with downstream support for the MediaTek
MT6785 (Helio G90/G95) and the Motorola Moto G60s (`lisbon`). It runs Debian
(Mobian) with Plasma Mobile or Phosh on the phone.

This is a downstream tree. Please do not report problems with it to the Linux
kernel maintainers.

## Branches

| Branch | Base |
|---|---|
| `7.3` (default) | `v7.3.6`, rebased onto each new 7.3 stable release; tags `v7.3.<n>-<m>` |
| `7.1` | [mt6785-mainline](https://gitlab.nura.eco/mt6785-mainline/linux) `7.1`, frozen |

Each branch has three layers:

1. the stable release,
2. the commits of [mt6785-mainline](https://gitlab.nura.eco/mt6785-mainline/linux)
   (MT6785 basics and the Xiaomi Redmi Note 8 Pro), kept as individual commits
   with their original authors,
3. the series described below.

## Changes

| Area | Change |
|---|---|
| Camera | Sensor drivers OV64B40, OV16A1Q, S5K4H7, S5K5E9 and the DW9800V lens driver. MT6785 support for the ISP3.0 sensor interface (seninf) and raw capture (camsv), based on the MT8365/MT8183 work of Louis Kuo, Phi-bang Nguyen and Václav Pavlín. New drivers for the image processor DIP (ISP pass 2) and the multi-frame blending unit MFB. The DIP driver has neutral defaults; image tuning is passed in by userspace (libcamera). |
| Video | MT6785 decoders H.264, HEVC, MPEG-2, MPEG-4, VP8 and VP9 and the H.264 encoder, all without the video coprocessor firmware. JPEG encoder. A memory-to-memory converter for the decoder output formats. |
| Display | MT6785 DSI, OVL and MIPI-DSI PHY; the display path set up by the bootloader is taken over without a blank screen. Drivers for the MT6382 DSC bridge, the Tianma ILI7807S panel and the Awinic AW99703 backlight. |
| GPU | panfrost fixes for shared buffers and address space reference counting on the Mali-G76. |
| Touch | ILITEK TDDI SPI touchscreen driver that follows the panel. |
| Audio | MT6785 AFE platform and machine driver, Awinic AW88258 amplifier, MT6359 codec and headset detection fixes. |
| Power | MT6360 charger with charge zones and input source selection, MT6359 fuel gauge driver, OCV-based capacity in generic-adc-battery. Power domains keep the state the bootloader left. |
| CPU and thermal | MT6785 cpufreq and CCI devfreq, PSCI idle states, LVTS thermal sensors, flash LEDs as cooling devices. |
| Connectivity | Driver for the CONNAC1 connectivity subsystem (power, firmware download, WLAN/BT/GNSS functions) and a Bluetooth HCI driver on top of it. |
| Modem | Device tree and bindings for the CCCI modem interface; n_gsm fix for the AT multiplexer. |
| Sensors | MT6785 SCP remote processor and a sensor hub driver for the accelerometer, magnetometer and light/proximity sensor. |
| Other | Fingerprint sensor lines for the Trustonic TEE, keys and vibration motor, watchdog with a sleep timer, RTC reboot modes, microSD slot, UFS lane count, `CMDLINE_EXTEND` on arm64, optional 1-of-N SPI routing in GIC-v3. |

## Not in this tree

Some functions need parts that are not published here:

- WLAN and the modem use vendor drivers (gen4m, ECCCI) built as external modules.
- Firmware for WLAN/BT, the modem, the sensor hub and the camera tuning come from
  the vendor partitions of the phone.
- The fingerprint sensor and GNSS need proprietary userspace components.

## Build

The configuration for the Moto G60s is `arch/arm64/configs/lisbon_defconfig`.
The kernel is built with clang:

    make ARCH=arm64 LLVM=1 lisbon_defconfig
    make ARCH=arm64 LLVM=1 Image.gz dtbs modules

The configuration builds the wireless regulatory database into the kernel. It
expects the upstream-signed `regulatory.db` and `regulatory.db.p7s` of
[wireless-regdb](https://git.kernel.org/pub/scm/linux/kernel/git/wens/wireless-regdb.git)
in `/opt/regdb` (Debian ships them as `regulatory.db-upstream` and
`regulatory.db.p7s-upstream` in `/usr/lib/firmware`); otherwise change
`CONFIG_EXTRA_FIRMWARE_DIR`.

The device tree is `arch/arm64/boot/dts/mediatek/mt6785-motorola-lisbon-full.dtb`.
The bootloader expects an Android boot image (header version 2) with the
kernel, the DTB and an initramfs.

Note: created with AI assistance.
