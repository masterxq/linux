// SPDX-License-Identifier: GPL-2.0
/*
 * Samsung S5K5E9 image sensor driver (5 MP Bayer, 2-lane MIPI CSI-2)
 *
 * One mode: 2592x1940 at 30 fps, SGRBG10, 2 lanes at 876 Mbit/s. The register
 * sequences, the power sequence, the exposure, gain, frame length and test
 * pattern registers and the stream-off handshake come from the vendor kernel
 * of the Motorola Moto G60s (GPL-2.0). The OTP access, its layout and the
 * CRC come from the same driver.
 *
 * The OTP is read once while the sensor is detected, like in the vendor
 * driver, and offered as a read-only nvmem device "s5k5e9-otpN" (root only,
 * it holds the module serial number) with one cell per block: "lsc" (the
 * on-sensor shading table), "module-info", "awb" and "calibration-info". If
 * that read fails, the first nvmem read retries it unless the sensor
 * streams.
 *
 * With a valid shading table in the OTP the sensor corrects the lens shading
 * itself (vendor s5k5e9_otp_lsc_update), the output is already corrected;
 * without one the correction is off. There is no control for it.
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/crc16.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/mux/consumer.h>
#include <linux/nvmem-provider.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/unaligned.h>
#include <media/v4l2-cci.h>
#include <media/v4l2-common.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-mediabus.h>

#define S5K5E9_LANES			2
/* 876 Mbit/s per lane: output PLL 24 MHz / 4 * 146 (0x030d..0x030f, 0x0820) */
#define S5K5E9_LINK_FREQ		438000000ULL
/* timing clock of the line length and frame length registers */
#define S5K5E9_PIXEL_RATE		190000000ULL
#define S5K5E9_MCLK_FREQ		24000000

#define S5K5E9_REG_CHIP_ID		CCI_REG16(0x0000)
#define S5K5E9_CHIP_ID			0x559b
#define S5K5E9_REG_FRAME_COUNT		CCI_REG8(0x0005)
#define S5K5E9_FRAME_COUNT_STOPPED	0xff

#define S5K5E9_REG_MODE_SELECT		CCI_REG8(0x0100)
#define S5K5E9_MODE_STANDBY		0x00
#define S5K5E9_MODE_STREAMING		0x01

#define S5K5E9_REG_EXPOSURE		CCI_REG16(0x0202)
#define S5K5E9_EXPOSURE_MIN		2
#define S5K5E9_EXPOSURE_MARGIN		10
/* analogue gain code: 32 = 1x up to 512 = 16x */
#define S5K5E9_REG_ANALOGUE_GAIN	CCI_REG16(0x0204)
#define S5K5E9_GAIN_MIN			0x20
#define S5K5E9_GAIN_MAX			0x200
#define S5K5E9_REG_FRAME_LENGTH		CCI_REG16(0x0340)
#define S5K5E9_FRAME_LENGTH_MAX		0xffff
#define S5K5E9_REG_TEST_PATTERN		CCI_REG8(0x0601)
#define S5K5E9_TEST_PATTERN_BARS	0x02
/*
 * Written to 0 together with the test pattern by the vendor driver; the init
 * sequence sets it to 1, which is restored when the pattern is switched off.
 */
#define S5K5E9_REG_TEST_PATTERN_AUX	CCI_REG8(0x3200)
#define S5K5E9_TEST_PATTERN_AUX_ON	0x00
#define S5K5E9_TEST_PATTERN_AUX_OFF	0x01

/*
 * On-sensor lens shading correction: 0x3400 = 0 lets the sensor load its
 * table from the OTP, 0x0b00 switches the correction on.
 */
#define S5K5E9_REG_LSC_SOURCE		CCI_REG8(0x3400)
#define S5K5E9_LSC_SOURCE_OTP		0x00
#define S5K5E9_REG_SHADING_CORRECTION	CCI_REG8(0x0b00)
#define S5K5E9_SHADING_CORRECTION_ON	0x01
#define S5K5E9_SHADING_CORRECTION_OFF	0x00

#define S5K5E9_WIDTH			2592
#define S5K5E9_HEIGHT			1940
#define S5K5E9_LINE_LENGTH		3112
#define S5K5E9_FRAME_LENGTH_DEF		2030

/*
 * Extent of the pixel array the vendor readout window addresses (x 0..2599,
 * y 0..1951); the physical array may be larger. The window starts at (8, 8),
 * the sensor sends its first 1940 lines.
 */
#define S5K5E9_NATIVE_WIDTH		2600
#define S5K5E9_NATIVE_HEIGHT		1952
#define S5K5E9_CROP_LEFT		8
#define S5K5E9_CROP_TOP			8

/*
 * Stream off: the frame counter reads 0xff once the current frame ended. The
 * longest frame (65535 lines of 16.4 us) takes 1.07 s.
 */
#define S5K5E9_STOP_POLL_US		1000
#define S5K5E9_STOP_TIMEOUT_US		1200000

/*
 * One-time programmable memory, read a 64-byte page at a time: page number to
 * 0x0a02, 1 to 0x0a00, wait for 0x0a01 to read 1, read the page from
 * 0x0a04..0x0a43, then 4 (clear the error bits) and 0 to 0x0a00. The read
 * only works while the sensor streams; the vendor driver streams for 50 ms
 * before the first page.
 */
#define S5K5E9_REG_OTP_CTRL		CCI_REG8(0x0a00)
#define S5K5E9_OTP_CTRL_IDLE		0x00
#define S5K5E9_OTP_CTRL_READ		0x01
#define S5K5E9_OTP_CTRL_CLEAR		0x04
#define S5K5E9_REG_OTP_STATUS		CCI_REG8(0x0a01)
#define S5K5E9_OTP_STATUS_READY		0x01
#define S5K5E9_REG_OTP_PAGE		CCI_REG8(0x0a02)
#define S5K5E9_OTP_BUF			0x0a04
#define S5K5E9_OTP_PAGE_SIZE		64
#define S5K5E9_OTP_PAGES		21
/* the pages the module maker programmed: 0-12 shading, 17-20 module data */
#define S5K5E9_OTP_PAGES_USED		(GENMASK(12, 0) | GENMASK(20, 17))
#define S5K5E9_OTP_START_US		50000
#define S5K5E9_OTP_POLL_US		1000
#define S5K5E9_OTP_TIMEOUT_US		100000

/*
 * The nvmem image: four blocks in the order of the vendor driver's OTP dump,
 * each the flag byte of the valid group, its data and a big-endian CRC-16.
 * A block without a valid group reads as zeros.
 */
#define S5K5E9_OTP_LSC_TABLE_SIZE	360
#define S5K5E9_OTP_LSC_OFFSET		0
#define S5K5E9_OTP_LSC_SIZE		(1 + S5K5E9_OTP_LSC_TABLE_SIZE + 2)
#define S5K5E9_OTP_MODULE_OFFSET	363
#define S5K5E9_OTP_MODULE_SIZE		(1 + 37 + 2)
#define S5K5E9_OTP_AWB_OFFSET		403
#define S5K5E9_OTP_AWB_SIZE		(1 + 43 + 2)
#define S5K5E9_OTP_INFO_OFFSET		449
#define S5K5E9_OTP_INFO_SIZE		(1 + 11 + 2)
#define S5K5E9_OTP_SIZE			463
#define S5K5E9_OTP_CRC_SIZE		2
#define S5K5E9_OTP_CRC_POLY		0x8005
static_assert(S5K5E9_OTP_MODULE_OFFSET ==
	      S5K5E9_OTP_LSC_OFFSET + S5K5E9_OTP_LSC_SIZE);
static_assert(S5K5E9_OTP_AWB_OFFSET ==
	      S5K5E9_OTP_MODULE_OFFSET + S5K5E9_OTP_MODULE_SIZE);
static_assert(S5K5E9_OTP_INFO_OFFSET ==
	      S5K5E9_OTP_AWB_OFFSET + S5K5E9_OTP_AWB_SIZE);
static_assert(S5K5E9_OTP_SIZE ==
	      S5K5E9_OTP_INFO_OFFSET + S5K5E9_OTP_INFO_SIZE);
#define S5K5E9_OTP_GROUP_SPANS		8
/* flag of the information blocks: bits 7:6 = 01 for the valid group */
#define S5K5E9_OTP_FLAG_STATE		GENMASK(7, 6)
#define S5K5E9_OTP_FLAG_VALID		FIELD_PREP_CONST(S5K5E9_OTP_FLAG_STATE, 1)

enum s5k5e9_otp_block_id {
	S5K5E9_OTP_LSC,
	S5K5E9_OTP_MODULE,
	S5K5E9_OTP_AWB,
	S5K5E9_OTP_INFO,
	S5K5E9_OTP_BLOCKS,
};

struct s5k5e9_otp_span {
	u8 page;
	u16 addr;
	u8 len;
};

struct s5k5e9_otp_group {
	struct s5k5e9_otp_span flag;
	u8 flag_mask;
	u8 flag_value;
	/* data after the flag byte up to and including the CRC */
	struct s5k5e9_otp_span data[S5K5E9_OTP_GROUP_SPANS];
};

struct s5k5e9_otp_block {
	unsigned int offset;
	unsigned int size;
	/* the CRC covers the block from crc_from up to the CRC itself */
	unsigned int crc_from;
	struct s5k5e9_otp_group group[2];
};

#define OTP_SPAN(p, a, l)	{ .page = (p), .addr = (a), .len = (l) }
#define OTP_INFO_FLAG(p, a)	.flag = OTP_SPAN(p, a, 1), \
				.flag_mask = S5K5E9_OTP_FLAG_STATE, \
				.flag_value = S5K5E9_OTP_FLAG_VALID

static const struct s5k5e9_otp_block s5k5e9_otp_blocks[S5K5E9_OTP_BLOCKS] = {
	/*
	 * On-sensor lens shading table. The flag on page 0 selects the group:
	 * 1 = pages 1-6, 3 = pages 6-12. The CRC on page 17 covers the table.
	 */
	[S5K5E9_OTP_LSC] = {
		.offset = S5K5E9_OTP_LSC_OFFSET,
		.size = S5K5E9_OTP_LSC_SIZE,
		.crc_from = 1,
		.group = {
			{
				.flag = OTP_SPAN(0, 0x0a3d, 1),
				.flag_mask = 0xff,
				.flag_value = 0x01,
				.data = {
					OTP_SPAN(1, 0x0a04, 64),
					OTP_SPAN(2, 0x0a04, 64),
					OTP_SPAN(3, 0x0a04, 64),
					OTP_SPAN(4, 0x0a04, 64),
					OTP_SPAN(5, 0x0a04, 64),
					OTP_SPAN(6, 0x0a04, 40),
					OTP_SPAN(17, 0x0a04, 2),
				},
			}, {
				.flag = OTP_SPAN(0, 0x0a3d, 1),
				.flag_mask = 0xff,
				.flag_value = 0x03,
				.data = {
					OTP_SPAN(6, 0x0a2c, 24),
					OTP_SPAN(7, 0x0a04, 64),
					OTP_SPAN(8, 0x0a04, 64),
					OTP_SPAN(9, 0x0a04, 64),
					OTP_SPAN(10, 0x0a04, 64),
					OTP_SPAN(11, 0x0a04, 64),
					OTP_SPAN(12, 0x0a04, 16),
					OTP_SPAN(17, 0x0a06, 2),
				},
			},
		},
	},
	/*
	 * Module information: table version, part number, lens and maker
	 * codes, production date, module serial number.
	 */
	[S5K5E9_OTP_MODULE] = {
		.offset = S5K5E9_OTP_MODULE_OFFSET,
		.size = S5K5E9_OTP_MODULE_SIZE,
		.group = {
			{
				OTP_INFO_FLAG(17, 0x0a08),
				.data = { OTP_SPAN(17, 0x0a09, 39) },
			}, {
				OTP_INFO_FLAG(18, 0x0a2c),
				.data = {
					OTP_SPAN(18, 0x0a2d, 23),
					OTP_SPAN(19, 0x0a04, 16),
				},
			},
		},
	},
	/* white balance of the module and of the golden module, limits */
	[S5K5E9_OTP_AWB] = {
		.offset = S5K5E9_OTP_AWB_OFFSET,
		.size = S5K5E9_OTP_AWB_SIZE,
		.group = {
			{
				OTP_INFO_FLAG(17, 0x0a30),
				.data = {
					OTP_SPAN(17, 0x0a31, 19),
					OTP_SPAN(18, 0x0a04, 26),
				},
			}, {
				OTP_INFO_FLAG(19, 0x0a14),
				.data = { OTP_SPAN(19, 0x0a15, 45) },
			},
		},
	},
	/* calibration type, sensor type, format version */
	[S5K5E9_OTP_INFO] = {
		.offset = S5K5E9_OTP_INFO_OFFSET,
		.size = S5K5E9_OTP_INFO_SIZE,
		.group = {
			{
				OTP_INFO_FLAG(18, 0x0a1e),
				.data = { OTP_SPAN(18, 0x0a1f, 13) },
			}, {
				OTP_INFO_FLAG(19, 0x0a42),
				.data = {
					OTP_SPAN(19, 0x0a43, 1),
					OTP_SPAN(20, 0x0a04, 12),
				},
			},
		},
	},
};

/* nvmem cells, one per block; the names also appear in the probe message */
static const struct nvmem_cell_info s5k5e9_otp_cells[S5K5E9_OTP_BLOCKS] = {
	[S5K5E9_OTP_LSC] = {
		.name = "lsc",
		.offset = S5K5E9_OTP_LSC_OFFSET,
		.bytes = S5K5E9_OTP_LSC_SIZE,
	},
	[S5K5E9_OTP_MODULE] = {
		.name = "module-info",
		.offset = S5K5E9_OTP_MODULE_OFFSET,
		.bytes = S5K5E9_OTP_MODULE_SIZE,
	},
	[S5K5E9_OTP_AWB] = {
		.name = "awb",
		.offset = S5K5E9_OTP_AWB_OFFSET,
		.bytes = S5K5E9_OTP_AWB_SIZE,
	},
	[S5K5E9_OTP_INFO] = {
		.name = "calibration-info",
		.offset = S5K5E9_OTP_INFO_OFFSET,
		.bytes = S5K5E9_OTP_INFO_SIZE,
	},
};

static const char * const s5k5e9_supply_names[] = {
	"avdd",		/* order of the vendor power sequence */
	"dvdd",
	"dovdd",
};

static const struct cci_reg_sequence s5k5e9_init_regs[] = {
	{ CCI_REG8(0x0100), 0x00 },
	{ CCI_REG8(0x0000), 0x04 },
	{ CCI_REG8(0x0000), 0x55 },
	{ CCI_REG8(0x0a02), 0x3f },
	{ CCI_REG8(0x0100), 0x00 },
	{ CCI_REG8(0x3b45), 0x01 },
	{ CCI_REG8(0x0b05), 0x01 },
	{ CCI_REG8(0x392f), 0x01 },
	{ CCI_REG8(0x3930), 0x00 },
	{ CCI_REG8(0x3924), 0x7f },
	{ CCI_REG8(0x3925), 0xfd },
	{ CCI_REG8(0x3c08), 0xff },
	{ CCI_REG8(0x3c09), 0xff },
	{ CCI_REG8(0x3c31), 0xff },
	{ CCI_REG8(0x3c32), 0xff },
	{ CCI_REG8(0x3290), 0x10 },
	{ CCI_REG8(0x3200), 0x01 },
	{ CCI_REG8(0x3074), 0x06 },
	{ CCI_REG8(0x3075), 0x2f },
	{ CCI_REG8(0x308a), 0x20 },
	{ CCI_REG8(0x308b), 0x08 },
	{ CCI_REG8(0x308c), 0x0b },
	{ CCI_REG8(0x3081), 0x07 },
	{ CCI_REG8(0x307b), 0x85 },
	{ CCI_REG8(0x307a), 0x0a },
	{ CCI_REG8(0x3079), 0x0a },
	{ CCI_REG8(0x306e), 0x71 },
	{ CCI_REG8(0x306f), 0x28 },
	{ CCI_REG8(0x301f), 0x20 },
	{ CCI_REG8(0x3012), 0x4e },
	{ CCI_REG8(0x306b), 0x9a },
	{ CCI_REG8(0x3091), 0x16 },
	{ CCI_REG8(0x30c4), 0x06 },
	{ CCI_REG8(0x306a), 0x79 },
	{ CCI_REG8(0x30b0), 0xff },
	{ CCI_REG8(0x306d), 0x08 },
	{ CCI_REG8(0x3084), 0x16 },
	{ CCI_REG8(0x3070), 0x0f },
	{ CCI_REG8(0x30c2), 0x05 },
	{ CCI_REG8(0x3069), 0x87 },
	{ CCI_REG8(0x3c0f), 0x00 },
	{ CCI_REG8(0x3083), 0x14 },
	{ CCI_REG8(0x3080), 0x08 },
	{ CCI_REG8(0x3c34), 0xea },
	{ CCI_REG8(0x3c35), 0x5c },
};

/* vendor s5k5e9_preview_setting (the same for all vendor scenarios) */
static const struct cci_reg_sequence s5k5e9_2592x1940_regs[] = {
	{ CCI_REG8(0x0100), 0x00 },
	{ CCI_REG8(0x0136), 0x18 },
	{ CCI_REG8(0x0137), 0x00 },
	{ CCI_REG8(0x0305), 0x04 },
	{ CCI_REG8(0x0306), 0x00 },
	{ CCI_REG8(0x0307), 0x5f },
	{ CCI_REG8(0x030d), 0x04 },
	{ CCI_REG8(0x030e), 0x00 },
	{ CCI_REG8(0x030f), 0x92 },
	{ CCI_REG8(0x3c1f), 0x00 },
	{ CCI_REG8(0x3c17), 0x00 },
	{ CCI_REG8(0x0112), 0x0a },
	{ CCI_REG8(0x0113), 0x0a },
	{ CCI_REG8(0x0114), 0x01 },
	{ CCI_REG8(0x0820), 0x03 },
	{ CCI_REG8(0x0821), 0x6c },
	{ CCI_REG8(0x0822), 0x00 },
	{ CCI_REG8(0x0823), 0x00 },
	{ CCI_REG8(0x3929), 0x0f },
	{ CCI_REG8(0x0344), 0x00 },
	{ CCI_REG8(0x0345), 0x08 },
	{ CCI_REG8(0x0346), 0x00 },
	{ CCI_REG8(0x0347), 0x08 },
	{ CCI_REG8(0x0348), 0x0a },
	{ CCI_REG8(0x0349), 0x27 },
	{ CCI_REG8(0x034a), 0x07 },
	{ CCI_REG8(0x034b), 0x9f },
	{ CCI_REG8(0x034c), 0x0a },
	{ CCI_REG8(0x034d), 0x20 },
	{ CCI_REG8(0x034e), 0x07 },
	{ CCI_REG8(0x034f), 0x98 },
	{ CCI_REG8(0x0900), 0x00 },
	{ CCI_REG8(0x0901), 0x00 },
	{ CCI_REG8(0x0381), 0x01 },
	{ CCI_REG8(0x0383), 0x01 },
	{ CCI_REG8(0x0385), 0x01 },
	{ CCI_REG8(0x0387), 0x01 },
	{ CCI_REG8(0x0101), 0x00 },
	{ CCI_REG8(0x0340), 0x07 },
	{ CCI_REG8(0x0341), 0xee },
	{ CCI_REG8(0x0342), 0x0c },
	{ CCI_REG8(0x0343), 0x28 },
	{ CCI_REG8(0x0200), 0x0b },
	{ CCI_REG8(0x0201), 0x9c },
	{ CCI_REG8(0x0202), 0x00 },
	{ CCI_REG8(0x0203), 0x02 },
	{ CCI_REG8(0x30b8), 0x2e },
	{ CCI_REG8(0x30ba), 0x36 },
};

static const s64 s5k5e9_link_freqs[] = {
	S5K5E9_LINK_FREQ,
};

static const char * const s5k5e9_test_pattern_menu[] = {
	"Disabled",
	"Color Bars",
};

static const struct v4l2_rect s5k5e9_crop = {
	.left = S5K5E9_CROP_LEFT,
	.top = S5K5E9_CROP_TOP,
	.width = S5K5E9_WIDTH,
	.height = S5K5E9_HEIGHT,
};

struct s5k5e9 {
	struct device *dev;
	struct regmap *regmap;
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *exposure;

	struct clk *mclk;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_default;
	struct pinctrl_state *pins_mclk;
	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[ARRAY_SIZE(s5k5e9_supply_names)];
	struct mux_state *mipi_switch;

	/* CSI-2 lane layout from the endpoint, reported by get_mbus_config */
	struct v4l2_mbus_config_mipi_csi2 csi2;

	/*
	 * OTP image, read once (at probe, or on the first nvmem read if that
	 * failed) under the active state lock, and the blocks with a valid
	 * group and CRC (bit per enum s5k5e9_otp_block_id).
	 */
	struct nvmem_device *nvmem;
	bool otp_read;
	unsigned long otp_valid;
	u8 otp[S5K5E9_OTP_SIZE];
};

static inline struct s5k5e9 *to_s5k5e9(struct v4l2_subdev *sd)
{
	return container_of(sd, struct s5k5e9, sd);
}

/*
 * Vendor power-up: XSHUTDOWN low, AVDD, DVDD, DOVDD,
 * MCLK with 1 ms between the steps, XSHUTDOWN high, then 18 ms plus the 5 ms
 * the vendor power code adds before the first access. Power-down in reverse.
 */
static int s5k5e9_power_on(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct s5k5e9 *s5k = to_s5k5e9(sd);
	unsigned int i;
	int ret;

	gpiod_set_value_cansleep(s5k->reset_gpio, 1);
	fsleep(1000);

	for (i = 0; i < ARRAY_SIZE(s5k5e9_supply_names); i++) {
		ret = regulator_enable(s5k->supplies[i].consumer);
		if (ret)
			goto err_supplies;
		fsleep(1000);
	}

	if (s5k->pins_mclk) {
		ret = pinctrl_select_state(s5k->pinctrl, s5k->pins_mclk);
		if (ret)
			goto err_supplies;
	}
	ret = clk_prepare_enable(s5k->mclk);
	if (ret)
		goto err_pins;
	fsleep(1000);

	gpiod_set_value_cansleep(s5k->reset_gpio, 0);
	fsleep(23000);

	return 0;

err_pins:
	if (s5k->pins_default)
		pinctrl_select_state(s5k->pinctrl, s5k->pins_default);
err_supplies:
	while (i--)
		regulator_disable(s5k->supplies[i].consumer);
	return ret;
}

static int s5k5e9_power_off(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct s5k5e9 *s5k = to_s5k5e9(sd);
	unsigned int i = ARRAY_SIZE(s5k5e9_supply_names);

	gpiod_set_value_cansleep(s5k->reset_gpio, 1);
	fsleep(1000);
	clk_disable_unprepare(s5k->mclk);
	if (s5k->pins_default)
		pinctrl_select_state(s5k->pinctrl, s5k->pins_default);
	while (i--) {
		fsleep(1000);
		regulator_disable(s5k->supplies[i].consumer);
	}

	return 0;
}

static int s5k5e9_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct s5k5e9 *s5k = container_of(ctrl->handler, struct s5k5e9,
					  ctrl_handler);
	int ret = 0;

	if (ctrl->id == V4L2_CID_VBLANK) {
		s64 max = S5K5E9_HEIGHT + ctrl->val - S5K5E9_EXPOSURE_MARGIN;

		ret = __v4l2_ctrl_modify_range(s5k->exposure,
					       S5K5E9_EXPOSURE_MIN, max, 1,
					       min(s5k->exposure->default_value,
						   max));
		if (ret)
			return ret;
	}

	if (!pm_runtime_get_if_in_use(s5k->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		cci_write(s5k->regmap, S5K5E9_REG_EXPOSURE, ctrl->val, &ret);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		cci_write(s5k->regmap, S5K5E9_REG_ANALOGUE_GAIN, ctrl->val,
			  &ret);
		break;
	case V4L2_CID_VBLANK:
		cci_write(s5k->regmap, S5K5E9_REG_FRAME_LENGTH,
			  S5K5E9_HEIGHT + ctrl->val, &ret);
		break;
	case V4L2_CID_TEST_PATTERN:
		cci_write(s5k->regmap, S5K5E9_REG_TEST_PATTERN,
			  ctrl->val ? S5K5E9_TEST_PATTERN_BARS : 0, &ret);
		cci_write(s5k->regmap, S5K5E9_REG_TEST_PATTERN_AUX,
			  ctrl->val ? S5K5E9_TEST_PATTERN_AUX_ON :
				      S5K5E9_TEST_PATTERN_AUX_OFF, &ret);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	pm_runtime_put_autosuspend(s5k->dev);

	return ret;
}

static const struct v4l2_ctrl_ops s5k5e9_ctrl_ops = {
	.s_ctrl = s5k5e9_set_ctrl,
};

static int s5k5e9_init_controls(struct s5k5e9 *s5k)
{
	struct v4l2_ctrl_handler *hdl = &s5k->ctrl_handler;
	struct v4l2_fwnode_device_properties props;
	struct v4l2_ctrl *ctrl;
	s64 vblank_def, hblank;
	int ret;

	v4l2_ctrl_handler_init(hdl, 9);

	v4l2_ctrl_new_std(hdl, NULL, V4L2_CID_PIXEL_RATE, S5K5E9_PIXEL_RATE,
			  S5K5E9_PIXEL_RATE, 1, S5K5E9_PIXEL_RATE);
	ctrl = v4l2_ctrl_new_int_menu(hdl, NULL, V4L2_CID_LINK_FREQ, 0, 0,
				      s5k5e9_link_freqs);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	hblank = S5K5E9_LINE_LENGTH - S5K5E9_WIDTH;
	ctrl = v4l2_ctrl_new_std(hdl, NULL, V4L2_CID_HBLANK, hblank, hblank, 1,
				 hblank);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	vblank_def = S5K5E9_FRAME_LENGTH_DEF - S5K5E9_HEIGHT;
	v4l2_ctrl_new_std(hdl, &s5k5e9_ctrl_ops, V4L2_CID_VBLANK, vblank_def,
			  S5K5E9_FRAME_LENGTH_MAX - S5K5E9_HEIGHT, 1,
			  vblank_def);
	s5k->exposure = v4l2_ctrl_new_std(hdl, &s5k5e9_ctrl_ops,
					  V4L2_CID_EXPOSURE, S5K5E9_EXPOSURE_MIN,
					  S5K5E9_FRAME_LENGTH_DEF -
					  S5K5E9_EXPOSURE_MARGIN, 1,
					  S5K5E9_FRAME_LENGTH_DEF -
					  S5K5E9_EXPOSURE_MARGIN);
	v4l2_ctrl_new_std(hdl, &s5k5e9_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			  S5K5E9_GAIN_MIN, S5K5E9_GAIN_MAX, 1, S5K5E9_GAIN_MIN);
	v4l2_ctrl_new_std_menu_items(hdl, &s5k5e9_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(s5k5e9_test_pattern_menu) - 1,
				     0, 0, s5k5e9_test_pattern_menu);

	if (hdl->error) {
		ret = hdl->error;
		goto err_free;
	}

	ret = v4l2_fwnode_device_parse(s5k->dev, &props);
	if (ret)
		goto err_free;
	ret = v4l2_ctrl_new_fwnode_properties(hdl, &s5k5e9_ctrl_ops, &props);
	if (ret)
		goto err_free;

	s5k->sd.ctrl_handler = hdl;

	return 0;

err_free:
	v4l2_ctrl_handler_free(hdl);
	return dev_err_probe(s5k->dev, ret, "control init failed\n");
}

static void s5k5e9_fill_fmt(struct v4l2_mbus_framefmt *fmt)
{
	fmt->code = MEDIA_BUS_FMT_SGRBG10_1X10;
	fmt->width = S5K5E9_WIDTH;
	fmt->height = S5K5E9_HEIGHT;
	fmt->field = V4L2_FIELD_NONE;
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->ycbcr_enc = V4L2_YCBCR_ENC_601;
	fmt->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->xfer_func = V4L2_XFER_FUNC_NONE;
}

static int s5k5e9_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index)
		return -EINVAL;

	code->code = MEDIA_BUS_FMT_SGRBG10_1X10;

	return 0;
}

static int s5k5e9_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index || fse->code != MEDIA_BUS_FMT_SGRBG10_1X10)
		return -EINVAL;

	fse->min_width = S5K5E9_WIDTH;
	fse->max_width = S5K5E9_WIDTH;
	fse->min_height = S5K5E9_HEIGHT;
	fse->max_height = S5K5E9_HEIGHT;

	return 0;
}

/* a single mode: every request gets 2592x1940 SGRBG10 */
static int s5k5e9_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *state,
			  struct v4l2_subdev_format *fmt)
{
	s5k5e9_fill_fmt(&fmt->format);
	*v4l2_subdev_state_get_format(state, 0) = fmt->format;

	return 0;
}

static int s5k5e9_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *state,
				struct v4l2_subdev_selection *sel)
{
	switch (sel->target) {
	case V4L2_SEL_TGT_NATIVE_SIZE:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = S5K5E9_NATIVE_WIDTH;
		sel->r.height = S5K5E9_NATIVE_HEIGHT;
		return 0;
	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP:
		sel->r = s5k5e9_crop;
		return 0;
	}

	return -EINVAL;
}

static int s5k5e9_get_mbus_config(struct v4l2_subdev *sd, unsigned int pad,
				  struct v4l2_mbus_config *cfg)
{
	struct s5k5e9 *s5k = to_s5k5e9(sd);

	cfg->type = V4L2_MBUS_CSI2_DPHY;
	cfg->link_freq = S5K5E9_LINK_FREQ;
	cfg->bus.mipi_csi2 = s5k->csi2;

	return 0;
}

static int s5k5e9_enable_streams(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state, u32 pad,
				 u64 streams_mask)
{
	struct s5k5e9 *s5k = to_s5k5e9(sd);
	int ret;

	ret = pm_runtime_resume_and_get(s5k->dev);
	if (ret < 0)
		return ret;

	/*
	 * The board MIPI switch in front of the CSI-2 receiver is powered from
	 * the I/O supply: select it while the sensor is on. Another sensor on
	 * the switch holding the other position makes the start fail.
	 */
	if (s5k->mipi_switch) {
		ret = mux_state_try_select(s5k->mipi_switch);
		if (ret) {
			dev_err(s5k->dev, "MIPI switch busy: %d\n", ret);
			goto err_put;
		}
	}

	ret = cci_multi_reg_write(s5k->regmap, s5k5e9_init_regs,
				  ARRAY_SIZE(s5k5e9_init_regs), NULL);
	if (ret)
		goto err_switch;

	/*
	 * Like the vendor driver after its init sequence: the on-sensor
	 * shading correction with the OTP table is on whenever the OTP holds
	 * a table with a valid CRC, otherwise off.
	 */
	if (test_bit(S5K5E9_OTP_LSC, &s5k->otp_valid)) {
		cci_write(s5k->regmap, S5K5E9_REG_LSC_SOURCE,
			  S5K5E9_LSC_SOURCE_OTP, &ret);
		cci_write(s5k->regmap, S5K5E9_REG_SHADING_CORRECTION,
			  S5K5E9_SHADING_CORRECTION_ON, &ret);
	} else {
		cci_write(s5k->regmap, S5K5E9_REG_SHADING_CORRECTION,
			  S5K5E9_SHADING_CORRECTION_OFF, &ret);
	}
	if (ret)
		goto err_switch;
	ret = cci_multi_reg_write(s5k->regmap, s5k5e9_2592x1940_regs,
				  ARRAY_SIZE(s5k5e9_2592x1940_regs), NULL);
	if (ret)
		goto err_switch;

	ret = __v4l2_ctrl_handler_setup(s5k->sd.ctrl_handler);
	if (ret)
		goto err_switch;

	/* vendor streaming_control(): 10 ms after stream on */
	ret = cci_write(s5k->regmap, S5K5E9_REG_MODE_SELECT,
			S5K5E9_MODE_STREAMING, NULL);
	if (ret)
		goto err_switch;
	fsleep(10000);

	return 0;

err_switch:
	if (s5k->mipi_switch)
		mux_state_deselect(s5k->mipi_switch);
err_put:
	dev_err(s5k->dev, "stream on failed: %d\n", ret);
	pm_runtime_put_autosuspend(s5k->dev);
	return ret;
}

static int s5k5e9_wait_stopped(struct s5k5e9 *s5k)
{
	u64 count = 0;
	int ret;

	return read_poll_timeout(cci_read, ret,
				 ret || count == S5K5E9_FRAME_COUNT_STOPPED,
				 S5K5E9_STOP_POLL_US, S5K5E9_STOP_TIMEOUT_US,
				 false, s5k->regmap, S5K5E9_REG_FRAME_COUNT,
				 &count, NULL) ?: ret;
}

/*
 * CRC-16 with the polynomial 0x8005 and the initial value 0. Module makers
 * store it bit-reflected (CRC-16/ARC) or not (CRC-16/UMTS); the vendor
 * driver accepts both.
 */
static bool s5k5e9_otp_crc_ok(const u8 *data, size_t len, u16 ref)
{
	unsigned int bit;
	u16 crc = 0;
	size_t i;

	if (crc16(0, data, len) == ref)
		return true;

	for (i = 0; i < len; i++) {
		crc ^= data[i] << 8;
		for (bit = 0; bit < 8; bit++)
			crc = crc & BIT(15) ? (crc << 1) ^ S5K5E9_OTP_CRC_POLY :
					      crc << 1;
	}

	return crc == ref;
}

static int s5k5e9_otp_read_page(struct s5k5e9 *s5k, unsigned int page, u8 *buf)
{
	u64 status = 0;
	int ret = 0;
	int err = 0;

	cci_write(s5k->regmap, S5K5E9_REG_OTP_PAGE, page, &ret);
	cci_write(s5k->regmap, S5K5E9_REG_OTP_CTRL, S5K5E9_OTP_CTRL_READ, &ret);
	if (!ret)
		ret = read_poll_timeout(cci_read, err,
					err || status == S5K5E9_OTP_STATUS_READY,
					S5K5E9_OTP_POLL_US,
					S5K5E9_OTP_TIMEOUT_US, false,
					s5k->regmap, S5K5E9_REG_OTP_STATUS,
					&status, NULL) ?: err;
	if (!ret)
		ret = regmap_bulk_read(s5k->regmap, S5K5E9_OTP_BUF, buf,
				       S5K5E9_OTP_PAGE_SIZE);

	/* end the access in any case */
	err = 0;
	cci_write(s5k->regmap, S5K5E9_REG_OTP_CTRL, S5K5E9_OTP_CTRL_CLEAR, &err);
	cci_write(s5k->regmap, S5K5E9_REG_OTP_CTRL, S5K5E9_OTP_CTRL_IDLE, &err);

	return ret ?: err;
}

static const u8 *s5k5e9_otp_at(const u8 *pages,
			       const struct s5k5e9_otp_span *span)
{
	return pages + span->page * S5K5E9_OTP_PAGE_SIZE + span->addr -
	       S5K5E9_OTP_BUF;
}

/* build the nvmem image from the raw pages and check the CRCs */
static void s5k5e9_otp_assemble(struct s5k5e9 *s5k, const u8 *pages)
{
	unsigned int b, g, i, pos;

	memset(s5k->otp, 0, sizeof(s5k->otp));
	s5k->otp_valid = 0;

	for (b = 0; b < S5K5E9_OTP_BLOCKS; b++) {
		const struct s5k5e9_otp_block *blk = &s5k5e9_otp_blocks[b];
		u8 *dst = s5k->otp + blk->offset;

		for (g = 0; g < ARRAY_SIZE(blk->group); g++) {
			const struct s5k5e9_otp_group *grp = &blk->group[g];
			const struct s5k5e9_otp_span *span = grp->data;
			u8 flag = *s5k5e9_otp_at(pages, &grp->flag);

			if ((flag & grp->flag_mask) != grp->flag_value)
				continue;

			dst[0] = flag;
			pos = 1;
			for (i = 0; i < S5K5E9_OTP_GROUP_SPANS && span[i].len;
			     i++) {
				memcpy(dst + pos, s5k5e9_otp_at(pages, &span[i]),
				       span[i].len);
				pos += span[i].len;
			}
			if (WARN_ON_ONCE(pos != blk->size))
				break;

			if (s5k5e9_otp_crc_ok(dst + blk->crc_from,
					      blk->size - blk->crc_from -
					      S5K5E9_OTP_CRC_SIZE,
					      get_unaligned_be16(dst + blk->size -
						      S5K5E9_OTP_CRC_SIZE)))
				__set_bit(b, &s5k->otp_valid);
			break;
		}
	}
}

/*
 * Read the programmed OTP pages. The sensor is powered and does not stream;
 * the caller serialises against the stream (probe, or the active state lock).
 */
static int s5k5e9_otp_load(struct s5k5e9 *s5k)
{
	unsigned long used = S5K5E9_OTP_PAGES_USED;
	unsigned int page;
	int ret, err = 0;
	u8 *pages;

	pages = kcalloc(S5K5E9_OTP_PAGES, S5K5E9_OTP_PAGE_SIZE, GFP_KERNEL);
	if (!pages)
		return -ENOMEM;

	ret = cci_write(s5k->regmap, S5K5E9_REG_MODE_SELECT,
			S5K5E9_MODE_STREAMING, NULL);
	if (ret)
		goto out;
	fsleep(S5K5E9_OTP_START_US);

	for_each_set_bit(page, &used, S5K5E9_OTP_PAGES) {
		ret = s5k5e9_otp_read_page(s5k, page,
					   pages + page * S5K5E9_OTP_PAGE_SIZE);
		if (ret)
			break;
	}

	cci_write(s5k->regmap, S5K5E9_REG_MODE_SELECT, S5K5E9_MODE_STANDBY,
		  &err);
	if (!err)
		err = s5k5e9_wait_stopped(s5k);
	ret = ret ?: err;
	if (ret)
		goto out;

	s5k5e9_otp_assemble(s5k, pages);
	s5k->otp_read = true;
out:
	kfree(pages);
	return ret;
}

/* which blocks are valid; the content (serial number) is not logged */
static void s5k5e9_otp_report(struct s5k5e9 *s5k)
{
	char valid[64] = "";
	size_t len = 0;
	unsigned int b;

	for_each_set_bit(b, &s5k->otp_valid, S5K5E9_OTP_BLOCKS)
		len += scnprintf(valid + len, sizeof(valid) - len, " %s",
				 s5k5e9_otp_cells[b].name);

	dev_info(s5k->dev, "OTP valid blocks:%s\n", len ? valid : " none");
}

static int s5k5e9_nvmem_read(void *priv, unsigned int off, void *val,
			     size_t bytes)
{
	struct s5k5e9 *s5k = priv;
	struct v4l2_subdev_state *state;
	int ret = 0;

	state = v4l2_subdev_lock_and_get_active_state(&s5k->sd);

	if (!s5k->otp_read) {
		/* the OTP read streams for a moment: not during a capture */
		if (v4l2_subdev_is_streaming(&s5k->sd)) {
			ret = -EBUSY;
			goto out;
		}
		ret = pm_runtime_resume_and_get(s5k->dev);
		if (ret < 0)
			goto out;
		ret = s5k5e9_otp_load(s5k);
		pm_runtime_put_autosuspend(s5k->dev);
		if (ret)
			goto out;
		s5k5e9_otp_report(s5k);
	}

	memcpy(val, s5k->otp + off, bytes);
out:
	v4l2_subdev_unlock_state(state);
	return ret;
}

static int s5k5e9_register_nvmem(struct s5k5e9 *s5k)
{
	struct nvmem_config config = {
		.dev = s5k->dev,
		.name = "s5k5e9-otp",
		.id = NVMEM_DEVID_AUTO,
		.owner = THIS_MODULE,
		.type = NVMEM_TYPE_OTP,
		.read_only = true,
		/* the module information holds the serial number */
		.root_only = true,
		.cells = s5k5e9_otp_cells,
		.ncells = ARRAY_SIZE(s5k5e9_otp_cells),
		.reg_read = s5k5e9_nvmem_read,
		.priv = s5k,
		.size = S5K5E9_OTP_SIZE,
		.word_size = 1,
		.stride = 1,
	};
	struct nvmem_device *nvmem;

	nvmem = nvmem_register(&config);
	if (IS_ERR(nvmem)) {
		/* without NVMEM support the OTP only serves the shading */
		if (PTR_ERR(nvmem) == -EOPNOTSUPP)
			return 0;
		return PTR_ERR(nvmem);
	}
	s5k->nvmem = nvmem;

	return 0;
}

static int s5k5e9_disable_streams(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state, u32 pad,
				  u64 streams_mask)
{
	struct s5k5e9 *s5k = to_s5k5e9(sd);
	int ret;

	/* vendor streaming_control(): wait for the end of the current frame */
	ret = cci_write(s5k->regmap, S5K5E9_REG_MODE_SELECT,
			S5K5E9_MODE_STANDBY, NULL);
	if (!ret)
		ret = s5k5e9_wait_stopped(s5k);
	if (ret)
		dev_warn(s5k->dev, "stream off: %d\n", ret);

	if (s5k->mipi_switch)
		mux_state_deselect(s5k->mipi_switch);
	pm_runtime_put_autosuspend(s5k->dev);

	return 0;
}

static int s5k5e9_init_state(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state)
{
	s5k5e9_fill_fmt(v4l2_subdev_state_get_format(state, 0));

	return 0;
}

static const struct v4l2_subdev_video_ops s5k5e9_video_ops = {
	.s_stream = v4l2_subdev_s_stream_helper,
};

static const struct v4l2_subdev_pad_ops s5k5e9_pad_ops = {
	.enum_mbus_code = s5k5e9_enum_mbus_code,
	.enum_frame_size = s5k5e9_enum_frame_size,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = s5k5e9_set_fmt,
	.get_selection = s5k5e9_get_selection,
	.get_mbus_config = s5k5e9_get_mbus_config,
	.enable_streams = s5k5e9_enable_streams,
	.disable_streams = s5k5e9_disable_streams,
};

static const struct v4l2_subdev_ops s5k5e9_subdev_ops = {
	.video = &s5k5e9_video_ops,
	.pad = &s5k5e9_pad_ops,
};

static const struct v4l2_subdev_internal_ops s5k5e9_internal_ops = {
	.init_state = s5k5e9_init_state,
};

static int s5k5e9_parse_endpoint(struct s5k5e9 *s5k)
{
	struct v4l2_fwnode_endpoint bus_cfg = {
		.bus_type = V4L2_MBUS_CSI2_DPHY,
	};
	struct fwnode_handle *ep;
	unsigned int i;
	int ret;

	ep = fwnode_graph_get_next_endpoint(dev_fwnode(s5k->dev), NULL);
	if (!ep)
		return dev_err_probe(s5k->dev, -ENXIO, "no endpoint\n");

	ret = v4l2_fwnode_endpoint_alloc_parse(ep, &bus_cfg);
	fwnode_handle_put(ep);
	if (ret)
		return dev_err_probe(s5k->dev, ret, "endpoint parse failed\n");

	if (bus_cfg.bus.mipi_csi2.num_data_lanes != S5K5E9_LANES) {
		ret = dev_err_probe(s5k->dev, -EINVAL, "only 2 data lanes\n");
		goto out;
	}
	for (i = 0; i < bus_cfg.nr_of_link_frequencies; i++)
		if (bus_cfg.link_frequencies[i] == S5K5E9_LINK_FREQ)
			break;
	if (i == bus_cfg.nr_of_link_frequencies) {
		ret = dev_err_probe(s5k->dev, -EINVAL,
				    "link frequency %llu not in DT\n",
				    S5K5E9_LINK_FREQ);
		goto out;
	}

	s5k->csi2 = bus_cfg.bus.mipi_csi2;
out:
	v4l2_fwnode_endpoint_free(&bus_cfg);
	return ret;
}

static int s5k5e9_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct s5k5e9 *s5k;
	unsigned int i;
	u64 id;
	int ret;

	s5k = devm_kzalloc(dev, sizeof(*s5k), GFP_KERNEL);
	if (!s5k)
		return -ENOMEM;
	s5k->dev = dev;

	ret = s5k5e9_parse_endpoint(s5k);
	if (ret)
		return ret;

	s5k->regmap = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(s5k->regmap))
		return dev_err_probe(dev, PTR_ERR(s5k->regmap),
				     "CCI init failed\n");

	s5k->mclk = devm_v4l2_sensor_clk_get(dev, NULL);
	if (IS_ERR(s5k->mclk))
		return dev_err_probe(dev, PTR_ERR(s5k->mclk), "no mclk\n");
	if (clk_get_rate(s5k->mclk) != S5K5E9_MCLK_FREQ)
		return dev_err_probe(dev, -EINVAL, "mclk %lu Hz, need 24 MHz\n",
				     clk_get_rate(s5k->mclk));

	/* optional: MCLK pin in GPIO mode while off ("default"), clock while on */
	s5k->pinctrl = devm_pinctrl_get(dev);
	if (!IS_ERR(s5k->pinctrl)) {
		s5k->pins_mclk = pinctrl_lookup_state(s5k->pinctrl, "mclk");
		s5k->pins_default = pinctrl_lookup_state(s5k->pinctrl,
							 PINCTRL_STATE_DEFAULT);
		if (IS_ERR(s5k->pins_mclk) || IS_ERR(s5k->pins_default)) {
			s5k->pins_mclk = NULL;
			s5k->pins_default = NULL;
		}
	}

	s5k->mipi_switch = devm_mux_state_get_optional(dev, NULL);
	if (IS_ERR(s5k->mipi_switch))
		return dev_err_probe(dev, PTR_ERR(s5k->mipi_switch),
				     "MIPI switch\n");

	s5k->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(s5k->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(s5k->reset_gpio),
				     "no reset gpio\n");

	for (i = 0; i < ARRAY_SIZE(s5k5e9_supply_names); i++)
		s5k->supplies[i].supply = s5k5e9_supply_names[i];
	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(s5k5e9_supply_names),
				      s5k->supplies);
	if (ret)
		return dev_err_probe(dev, ret, "supplies\n");

	v4l2_i2c_subdev_init(&s5k->sd, client, &s5k5e9_subdev_ops);

	ret = s5k5e9_power_on(dev);
	if (ret)
		return dev_err_probe(dev, ret, "power on failed\n");

	ret = cci_read(s5k->regmap, S5K5E9_REG_CHIP_ID, &id, NULL);
	if (ret || id != S5K5E9_CHIP_ID) {
		ret = dev_err_probe(dev, ret ?: -ENODEV, "chip id 0x%04llx\n",
				    id);
		goto err_power_off;
	}

	/* like the vendor driver, read the OTP once while detecting the sensor */
	ret = s5k5e9_otp_load(s5k);
	if (ret)
		dev_warn(dev, "OTP read failed: %d\n", ret);
	else
		s5k5e9_otp_report(s5k);

	ret = s5k5e9_init_controls(s5k);
	if (ret)
		goto err_power_off;

	s5k->sd.internal_ops = &s5k5e9_internal_ops;
	s5k->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	s5k->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	s5k->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&s5k->sd.entity, 1, &s5k->pad);
	if (ret)
		goto err_ctrls;

	s5k->sd.state_lock = s5k->ctrl_handler.lock;
	ret = v4l2_subdev_init_finalize(&s5k->sd);
	if (ret)
		goto err_entity;

	pm_runtime_set_active(dev);
	pm_runtime_get_noresume(dev);
	pm_runtime_enable(dev);
	pm_runtime_set_autosuspend_delay(dev, 1000);
	pm_runtime_use_autosuspend(dev);

	ret = s5k5e9_register_nvmem(s5k);
	if (ret) {
		dev_err_probe(dev, ret, "nvmem registration failed\n");
		goto err_pm;
	}

	ret = v4l2_async_register_subdev_sensor(&s5k->sd);
	if (ret)
		goto err_nvmem;

	pm_runtime_put_autosuspend(dev);

	dev_info(dev, "S5K5E9 0x%04llx\n", id);

	return 0;

err_nvmem:
	nvmem_unregister(s5k->nvmem);
err_pm:
	pm_runtime_dont_use_autosuspend(dev);
	pm_runtime_disable(dev);
	pm_runtime_put_noidle(dev);
	pm_runtime_set_suspended(dev);
	v4l2_subdev_cleanup(&s5k->sd);
err_entity:
	media_entity_cleanup(&s5k->sd.entity);
err_ctrls:
	v4l2_ctrl_handler_free(&s5k->ctrl_handler);
err_power_off:
	s5k5e9_power_off(dev);
	return ret;
}

static void s5k5e9_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct s5k5e9 *s5k = to_s5k5e9(sd);

	nvmem_unregister(s5k->nvmem);
	v4l2_async_unregister_subdev(sd);
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(&s5k->ctrl_handler);

	pm_runtime_dont_use_autosuspend(s5k->dev);
	pm_runtime_disable(s5k->dev);
	if (!pm_runtime_status_suspended(s5k->dev))
		s5k5e9_power_off(s5k->dev);
	pm_runtime_set_suspended(s5k->dev);
}

static DEFINE_RUNTIME_DEV_PM_OPS(s5k5e9_pm_ops, s5k5e9_power_off,
				 s5k5e9_power_on, NULL);

static const struct of_device_id s5k5e9_of_match[] = {
	{ .compatible = "samsung,s5k5e9" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, s5k5e9_of_match);

static struct i2c_driver s5k5e9_driver = {
	.driver = {
		.name = "s5k5e9",
		.of_match_table = s5k5e9_of_match,
		.pm = pm_ptr(&s5k5e9_pm_ops),
	},
	.probe = s5k5e9_probe,
	.remove = s5k5e9_remove,
};
module_i2c_driver(s5k5e9_driver);

MODULE_DESCRIPTION("Samsung S5K5E9 image sensor driver");
MODULE_LICENSE("GPL");
