// SPDX-License-Identifier: GPL-2.0
/*
 * Samsung S5K4H7 image sensor driver (8 MP Bayer, 4-lane MIPI CSI-2)
 *
 * Two modes at 30 fps: 3264x2448 (full field) and 1280x720 (2x2 binned centre
 * 2560x1440), SGRBG10, 4 lanes at 700 Mbit/s. The register sequences, the
 * power sequence, the exposure, gain, frame length and test pattern registers
 * and the stream-off handshake come from the vendor kernel of the Motorola
 * Moto G60s (GPL-2.0). The OTP access, its layout and the CRC come from the
 * same driver.
 *
 * The sensor mirrors and flips its readout (HFLIP, VFLIP) without moving the
 * window, which changes the Bayer order of the output.
 *
 * The OTP is read once while the sensor is detected, like in the vendor
 * driver, and offered as a read-only nvmem device "s5k4h7-otpN" (root only,
 * it holds the module serial number) with one cell per block: "module-info",
 * "light-source", "awb", "optical-center" and "lsc" (the lens shading table
 * for the ISP; the sensor does not apply it itself). If that read fails, the
 * first nvmem read retries it unless the sensor streams.
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

#define S5K4H7_LANES			4
/* 700 Mbit/s per lane: output PLL 24 MHz / 6 * 175 (0x030d..0x030f, 0x0820) */
#define S5K4H7_LINK_FREQ		350000000ULL
/* timing clock of the line length and frame length registers */
#define S5K4H7_PIXEL_RATE		280000000ULL
#define S5K4H7_MCLK_FREQ		24000000

#define S5K4H7_REG_CHIP_ID		CCI_REG16(0x0000)
#define S5K4H7_CHIP_ID			0x487b
#define S5K4H7_REG_FRAME_COUNT		CCI_REG8(0x0005)
#define S5K4H7_FRAME_COUNT_STOPPED	0xff

#define S5K4H7_REG_MODE_SELECT		CCI_REG8(0x0100)
#define S5K4H7_MODE_STANDBY		0x00
#define S5K4H7_MODE_STREAMING		0x01
#define S5K4H7_REG_ORIENTATION		CCI_REG8(0x0101)
#define S5K4H7_HFLIP			BIT(0)
#define S5K4H7_VFLIP			BIT(1)

#define S5K4H7_REG_EXPOSURE		CCI_REG16(0x0202)
#define S5K4H7_EXPOSURE_MIN		4
#define S5K4H7_EXPOSURE_MARGIN		4
/* analogue gain code: 32 = 1x up to 512 = 16x */
#define S5K4H7_REG_ANALOGUE_GAIN	CCI_REG16(0x0204)
#define S5K4H7_GAIN_MIN			0x20
#define S5K4H7_GAIN_MAX			0x200
#define S5K4H7_REG_FRAME_LENGTH		CCI_REG16(0x0340)
#define S5K4H7_FRAME_LENGTH_MAX		0xfffa
#define S5K4H7_REG_TEST_PATTERN		CCI_REG8(0x0601)
#define S5K4H7_TEST_PATTERN_BARS	0x02
/*
 * Written to 0 together with the test pattern by the vendor driver; the init
 * sequence sets it to 0x09, which is restored when the pattern is switched
 * off.
 */
#define S5K4H7_REG_TEST_PATTERN_AUX	CCI_REG8(0x3200)
#define S5K4H7_TEST_PATTERN_AUX_ON	0x00
#define S5K4H7_TEST_PATTERN_AUX_OFF	0x09

/* line length 3688 (0x0342) at the 280 MHz timing clock in both modes */
#define S5K4H7_LINE_LENGTH		3688
#define S5K4H7_FRAME_LENGTH_DEF		2530

/*
 * Extent of the pixel array the vendor readout windows address (x 0..3271,
 * y 0..2455); the physical array may be larger.
 */
#define S5K4H7_NATIVE_WIDTH		3272
#define S5K4H7_NATIVE_HEIGHT		2456

/* stream off: the frame counter reads 0xff once the current frame ended */
#define S5K4H7_STOP_POLL_US		2000
#define S5K4H7_STOP_TIMEOUT_US		1000000

/*
 * One-time programmable memory, read a 64-byte page at a time: page number to
 * 0x0a02, the read setup of the vendor driver, 0x0100 to the 16-bit control
 * register 0x0a00 (read start, clears the status), wait for 0x0a01 to read 1,
 * read the page from 0x0a04..0x0a43, then 0x0400 (clear the error bits) and 0
 * to 0x0a00. The read only works while the sensor streams; the vendor driver
 * streams for 50 ms before the first page.
 */
#define S5K4H7_REG_OTP_CTRL		CCI_REG16(0x0a00)
#define S5K4H7_OTP_CTRL_IDLE		0x0000
#define S5K4H7_OTP_CTRL_READ		0x0100
#define S5K4H7_OTP_CTRL_CLEAR		0x0400
#define S5K4H7_REG_OTP_STATUS		CCI_REG8(0x0a01)
#define S5K4H7_OTP_STATUS_READY		0x01
#define S5K4H7_REG_OTP_PAGE		CCI_REG8(0x0a02)
#define S5K4H7_OTP_BUF			0x0a04
#define S5K4H7_OTP_PAGE_SIZE		64
#define S5K4H7_OTP_START_US		50000
#define S5K4H7_OTP_POLL_US		1000
#define S5K4H7_OTP_TIMEOUT_US		100000

/*
 * The module data comes in three groups of 238 bytes (three pages and 46
 * bytes of a fourth) from pages 21, 25 and 29; each block in a group starts
 * with a flag, bits 7:6 = 01 marks it valid, and ends with a big-endian
 * CRC-16. The lens shading table comes in three groups of 30 pages from
 * pages 33, 63 and 93: flag, 1868 bytes of table, CRC.
 */
#define S5K4H7_OTP_INFO_PAGE		21
#define S5K4H7_OTP_INFO_GROUP_PAGES	4
#define S5K4H7_OTP_INFO_GROUP_SIZE	238
#define S5K4H7_OTP_LSC_PAGE		33
#define S5K4H7_OTP_LSC_GROUP_PAGES	30
#define S5K4H7_OTP_GROUPS		3
#define S5K4H7_OTP_FLAG_STATE		GENMASK(7, 6)
#define S5K4H7_OTP_FLAG_VALID		FIELD_PREP_CONST(S5K4H7_OTP_FLAG_STATE, 1)
#define S5K4H7_OTP_CRC_SIZE		2
#define S5K4H7_OTP_CRC_POLY		0x8005

/*
 * The nvmem image: the blocks of the valid groups one after the other, each
 * the flag byte, the data and the CRC; a block without a valid group reads
 * as zeros. The module data blocks lie in a group at the same offsets as in
 * the image (the group continues with three sharpness test blocks), the lens
 * shading table follows them.
 */
#define S5K4H7_OTP_MODULE_OFFSET	0
#define S5K4H7_OTP_MODULE_SIZE		(1 + 30 + 2)
#define S5K4H7_OTP_LIGHT_OFFSET		33
#define S5K4H7_OTP_LIGHT_SIZE		(1 + 20 + 2)
#define S5K4H7_OTP_AWB_OFFSET		56
#define S5K4H7_OTP_AWB_SIZE		(1 + 56 + 2)
#define S5K4H7_OTP_OC_OFFSET		115
#define S5K4H7_OTP_OC_SIZE		(1 + 18 + 2)
#define S5K4H7_OTP_LSC_OFFSET		136
#define S5K4H7_OTP_LSC_SIZE		(1 + 1868 + 2)
#define S5K4H7_OTP_SIZE			2007
static_assert(S5K4H7_OTP_LIGHT_OFFSET ==
	      S5K4H7_OTP_MODULE_OFFSET + S5K4H7_OTP_MODULE_SIZE);
static_assert(S5K4H7_OTP_AWB_OFFSET ==
	      S5K4H7_OTP_LIGHT_OFFSET + S5K4H7_OTP_LIGHT_SIZE);
static_assert(S5K4H7_OTP_OC_OFFSET ==
	      S5K4H7_OTP_AWB_OFFSET + S5K4H7_OTP_AWB_SIZE);
static_assert(S5K4H7_OTP_LSC_OFFSET ==
	      S5K4H7_OTP_OC_OFFSET + S5K4H7_OTP_OC_SIZE);
static_assert(S5K4H7_OTP_SIZE ==
	      S5K4H7_OTP_LSC_OFFSET + S5K4H7_OTP_LSC_SIZE);
static_assert(S5K4H7_OTP_OC_OFFSET + S5K4H7_OTP_OC_SIZE <=
	      S5K4H7_OTP_INFO_GROUP_SIZE);
static_assert(S5K4H7_OTP_INFO_GROUP_SIZE <=
	      S5K4H7_OTP_INFO_GROUP_PAGES * S5K4H7_OTP_PAGE_SIZE);
static_assert(S5K4H7_OTP_LSC_SIZE <=
	      S5K4H7_OTP_LSC_GROUP_PAGES * S5K4H7_OTP_PAGE_SIZE);

enum s5k4h7_otp_block_id {
	S5K4H7_OTP_MODULE,
	S5K4H7_OTP_LIGHT,
	S5K4H7_OTP_AWB,
	S5K4H7_OTP_OC,
	S5K4H7_OTP_LSC,
	S5K4H7_OTP_BLOCKS,
};

/* nvmem cells, one per block; the names also appear in the probe message */
static const struct nvmem_cell_info s5k4h7_otp_cells[S5K4H7_OTP_BLOCKS] = {
	[S5K4H7_OTP_MODULE] = {
		.name = "module-info",
		.offset = S5K4H7_OTP_MODULE_OFFSET,
		.bytes = S5K4H7_OTP_MODULE_SIZE,
	},
	[S5K4H7_OTP_LIGHT] = {
		.name = "light-source",
		.offset = S5K4H7_OTP_LIGHT_OFFSET,
		.bytes = S5K4H7_OTP_LIGHT_SIZE,
	},
	[S5K4H7_OTP_AWB] = {
		.name = "awb",
		.offset = S5K4H7_OTP_AWB_OFFSET,
		.bytes = S5K4H7_OTP_AWB_SIZE,
	},
	[S5K4H7_OTP_OC] = {
		.name = "optical-center",
		.offset = S5K4H7_OTP_OC_OFFSET,
		.bytes = S5K4H7_OTP_OC_SIZE,
	},
	[S5K4H7_OTP_LSC] = {
		.name = "lsc",
		.offset = S5K4H7_OTP_LSC_OFFSET,
		.bytes = S5K4H7_OTP_LSC_SIZE,
	},
};

/* written before each page read by the vendor driver, meaning not known */
static const struct cci_reg_sequence s5k4h7_otp_setup_regs[] = {
	{ CCI_REG8(0x3b41), 0x01 },
	{ CCI_REG8(0x3b42), 0x03 },
	{ CCI_REG8(0x3b40), 0x01 },
};

struct s5k4h7_mode {
	u32 width;
	u32 height;
	struct v4l2_rect crop;
	const struct cci_reg_sequence *regs;
	unsigned int num_regs;
};

static const char * const s5k4h7_supply_names[] = {
	"avdd",		/* order of the vendor power sequence */
	"dvdd",
	"dovdd",
};

static const struct cci_reg_sequence s5k4h7_init_regs[] = {
	{ CCI_REG8(0x0100), 0x00 },
	{ CCI_REG8(0x0000), 0x12 },
	{ CCI_REG8(0x0000), 0x48 },
	{ CCI_REG8(0x0a02), 0x15 },
	{ CCI_REG8(0x0100), 0x00 },
	{ CCI_REG8(0x0b05), 0x01 },
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
	{ CCI_REG8(0x306b), 0x9a },
	{ CCI_REG8(0x3091), 0x1f },
	{ CCI_REG8(0x30c4), 0x06 },
	{ CCI_REG8(0x3200), 0x09 },
	{ CCI_REG8(0x306a), 0x79 },
	{ CCI_REG8(0x30b0), 0xff },
	{ CCI_REG8(0x306d), 0x08 },
	{ CCI_REG8(0x3080), 0x00 },
	{ CCI_REG8(0x3929), 0x3f },
	{ CCI_REG8(0x3084), 0x16 },
	{ CCI_REG8(0x3070), 0x0f },
	{ CCI_REG8(0x3b45), 0x01 },
	{ CCI_REG8(0x30c2), 0x05 },
	{ CCI_REG8(0x3069), 0x87 },
	{ CCI_REG8(0x3924), 0x7f },
	{ CCI_REG8(0x3925), 0xfd },
	{ CCI_REG8(0x3c08), 0xff },
	{ CCI_REG8(0x3c09), 0xff },
	{ CCI_REG8(0x3c31), 0xff },
	{ CCI_REG8(0x3c32), 0xff },
	{ CCI_REG8(0x0100), 0x00 },
};

/* vendor preview_setting() without the final stream on */
static const struct cci_reg_sequence s5k4h7_3264x2448_regs[] = {
	{ CCI_REG8(0x0136), 0x18 },
	{ CCI_REG8(0x0137), 0x00 },
	{ CCI_REG8(0x0305), 0x06 },
	{ CCI_REG8(0x0306), 0x00 },
	{ CCI_REG8(0x0307), 0x8c },
	{ CCI_REG8(0x030d), 0x06 },
	{ CCI_REG8(0x030e), 0x00 },
	{ CCI_REG8(0x030f), 0xaf },
	{ CCI_REG8(0x3c1f), 0x00 },
	{ CCI_REG8(0x3c17), 0x00 },
	{ CCI_REG8(0x3c1c), 0x05 },
	{ CCI_REG8(0x3c1d), 0x15 },
	{ CCI_REG8(0x0301), 0x04 },
	{ CCI_REG8(0x0820), 0x02 },
	{ CCI_REG8(0x0821), 0xbc },
	{ CCI_REG8(0x0822), 0x00 },
	{ CCI_REG8(0x0823), 0x00 },
	{ CCI_REG8(0x0112), 0x0a },
	{ CCI_REG8(0x0113), 0x0a },
	{ CCI_REG8(0x0114), 0x03 },
	{ CCI_REG8(0x3906), 0x04 },
	{ CCI_REG8(0x0344), 0x00 },
	{ CCI_REG8(0x0345), 0x08 },
	{ CCI_REG8(0x0346), 0x00 },
	{ CCI_REG8(0x0347), 0x08 },
	{ CCI_REG8(0x0348), 0x0c },
	{ CCI_REG8(0x0349), 0xc7 },
	{ CCI_REG8(0x034a), 0x09 },
	{ CCI_REG8(0x034b), 0x97 },
	{ CCI_REG8(0x034c), 0x0c },
	{ CCI_REG8(0x034d), 0xc0 },
	{ CCI_REG8(0x034e), 0x09 },
	{ CCI_REG8(0x034f), 0x90 },
	{ CCI_REG8(0x0900), 0x00 },
	{ CCI_REG8(0x0901), 0x00 },
	{ CCI_REG8(0x0381), 0x01 },
	{ CCI_REG8(0x0383), 0x01 },
	{ CCI_REG8(0x0385), 0x01 },
	{ CCI_REG8(0x0387), 0x01 },
	{ CCI_REG8(0x0101), 0x00 },
	{ CCI_REG8(0x0340), 0x09 },
	{ CCI_REG8(0x0341), 0xe2 },
	{ CCI_REG8(0x0342), 0x0e },
	{ CCI_REG8(0x0343), 0x68 },
	{ CCI_REG8(0x0200), 0x0d },
	{ CCI_REG8(0x0201), 0xd8 },
	{ CCI_REG8(0x0202), 0x02 },
	{ CCI_REG8(0x0203), 0x08 },
	{ CCI_REG8(0x3400), 0x01 },
};

/* vendor slim_video_setting() without the final stream on */
static const struct cci_reg_sequence s5k4h7_1280x720_regs[] = {
	{ CCI_REG8(0x0136), 0x18 },
	{ CCI_REG8(0x0137), 0x00 },
	{ CCI_REG8(0x0305), 0x06 },
	{ CCI_REG8(0x0306), 0x00 },
	{ CCI_REG8(0x0307), 0x8c },
	{ CCI_REG8(0x030d), 0x06 },
	{ CCI_REG8(0x030e), 0x00 },
	{ CCI_REG8(0x030f), 0xaf },
	{ CCI_REG8(0x3c1f), 0x00 },
	{ CCI_REG8(0x3c17), 0x00 },
	{ CCI_REG8(0x3c1c), 0x05 },
	{ CCI_REG8(0x3c1d), 0x15 },
	{ CCI_REG8(0x0301), 0x04 },
	{ CCI_REG8(0x0820), 0x02 },
	{ CCI_REG8(0x0821), 0xbc },
	{ CCI_REG8(0x0822), 0x00 },
	{ CCI_REG8(0x0823), 0x00 },
	{ CCI_REG8(0x0112), 0x0a },
	{ CCI_REG8(0x0113), 0x0a },
	{ CCI_REG8(0x0114), 0x03 },
	{ CCI_REG8(0x3906), 0x00 },
	{ CCI_REG8(0x0344), 0x01 },
	{ CCI_REG8(0x0345), 0x68 },
	{ CCI_REG8(0x0346), 0x02 },
	{ CCI_REG8(0x0347), 0x00 },
	{ CCI_REG8(0x0348), 0x0b },
	{ CCI_REG8(0x0349), 0x67 },
	{ CCI_REG8(0x034a), 0x07 },
	{ CCI_REG8(0x034b), 0x9f },
	{ CCI_REG8(0x034c), 0x05 },
	{ CCI_REG8(0x034d), 0x00 },
	{ CCI_REG8(0x034e), 0x02 },
	{ CCI_REG8(0x034f), 0xd0 },
	{ CCI_REG8(0x0900), 0x01 },
	{ CCI_REG8(0x0901), 0x22 },
	{ CCI_REG8(0x0381), 0x01 },
	{ CCI_REG8(0x0383), 0x01 },
	{ CCI_REG8(0x0385), 0x01 },
	{ CCI_REG8(0x0387), 0x03 },
	{ CCI_REG8(0x0101), 0x00 },
	{ CCI_REG8(0x0340), 0x09 },
	{ CCI_REG8(0x0341), 0xe2 },
	{ CCI_REG8(0x0342), 0x0e },
	{ CCI_REG8(0x0343), 0x68 },
	{ CCI_REG8(0x0200), 0x0d },
	{ CCI_REG8(0x0201), 0xd8 },
	{ CCI_REG8(0x0202), 0x02 },
	{ CCI_REG8(0x0203), 0x08 },
	{ CCI_REG8(0x3400), 0x01 },
};

static const struct s5k4h7_mode s5k4h7_modes[] = {
	{
		.width = 3264,
		.height = 2448,
		.crop = { 8, 8, 3264, 2448 },
		.regs = s5k4h7_3264x2448_regs,
		.num_regs = ARRAY_SIZE(s5k4h7_3264x2448_regs),
	},
	{
		.width = 1280,
		.height = 720,
		.crop = { 360, 512, 2560, 1440 },
		.regs = s5k4h7_1280x720_regs,
		.num_regs = ARRAY_SIZE(s5k4h7_1280x720_regs),
	},
};

static const s64 s5k4h7_link_freqs[] = {
	S5K4H7_LINK_FREQ,
};

static const char * const s5k4h7_test_pattern_menu[] = {
	"Disabled",
	"Color Bars",
};

/*
 * Bayer order by flip (bit 0 horizontal, bit 1 vertical): the readout window
 * stays, so mirroring swaps the colours of a line and flipping the lines.
 */
static const u32 s5k4h7_codes[] = {
	MEDIA_BUS_FMT_SGRBG10_1X10,
	MEDIA_BUS_FMT_SRGGB10_1X10,
	MEDIA_BUS_FMT_SBGGR10_1X10,
	MEDIA_BUS_FMT_SGBRG10_1X10,
};

struct s5k4h7 {
	struct device *dev;
	struct regmap *regmap;
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *hblank;
	struct v4l2_ctrl *exposure;
	struct v4l2_ctrl *hflip;
	struct v4l2_ctrl *vflip;

	struct clk *mclk;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_default;
	struct pinctrl_state *pins_mclk;
	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[ARRAY_SIZE(s5k4h7_supply_names)];
	struct mux_state *mipi_switch;

	/* CSI-2 lane layout from the endpoint, reported by get_mbus_config */
	struct v4l2_mbus_config_mipi_csi2 csi2;

	const struct s5k4h7_mode *mode;

	/*
	 * OTP image, read once (at probe, or on the first nvmem read if that
	 * failed) under the active state lock, and the blocks with a valid
	 * group and CRC (bit per enum s5k4h7_otp_block_id).
	 */
	struct nvmem_device *nvmem;
	bool otp_read;
	unsigned long otp_valid;
	u8 otp[S5K4H7_OTP_SIZE];
};

static inline struct s5k4h7 *to_s5k4h7(struct v4l2_subdev *sd)
{
	return container_of(sd, struct s5k4h7, sd);
}

/* the bus code of the current flips */
static u32 s5k4h7_code(struct s5k4h7 *s5k)
{
	return s5k4h7_codes[(s5k->vflip->val ? 2 : 0) |
			    (s5k->hflip->val ? 1 : 0)];
}

/*
 * Vendor power-up: XSHUTDOWN low, MCLK, AVDD, DVDD,
 * DOVDD with 1 ms between the steps, XSHUTDOWN high, 5 ms. Power-down in
 * reverse with the same waits.
 */
static int s5k4h7_power_on(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct s5k4h7 *s5k = to_s5k4h7(sd);
	unsigned int i;
	int ret;

	gpiod_set_value_cansleep(s5k->reset_gpio, 1);
	fsleep(1000);

	if (s5k->pins_mclk) {
		ret = pinctrl_select_state(s5k->pinctrl, s5k->pins_mclk);
		if (ret)
			return ret;
	}
	ret = clk_prepare_enable(s5k->mclk);
	if (ret)
		goto err_pins;
	fsleep(1000);

	for (i = 0; i < ARRAY_SIZE(s5k4h7_supply_names); i++) {
		ret = regulator_enable(s5k->supplies[i].consumer);
		if (ret)
			goto err_supplies;
		fsleep(1000);
	}

	gpiod_set_value_cansleep(s5k->reset_gpio, 0);
	fsleep(5000);

	return 0;

err_supplies:
	while (i--)
		regulator_disable(s5k->supplies[i].consumer);
	clk_disable_unprepare(s5k->mclk);
err_pins:
	if (s5k->pins_default)
		pinctrl_select_state(s5k->pinctrl, s5k->pins_default);
	return ret;
}

static int s5k4h7_power_off(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct s5k4h7 *s5k = to_s5k4h7(sd);
	unsigned int i = ARRAY_SIZE(s5k4h7_supply_names);

	gpiod_set_value_cansleep(s5k->reset_gpio, 1);
	fsleep(5000);
	while (i--) {
		regulator_disable(s5k->supplies[i].consumer);
		fsleep(1000);
	}
	clk_disable_unprepare(s5k->mclk);
	if (s5k->pins_default)
		pinctrl_select_state(s5k->pinctrl, s5k->pins_default);

	return 0;
}

static int s5k4h7_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct s5k4h7 *s5k = container_of(ctrl->handler, struct s5k4h7,
					  ctrl_handler);
	struct v4l2_subdev_state *state;
	struct v4l2_mbus_framefmt *fmt;
	int ret = 0;

	state = v4l2_subdev_get_locked_active_state(&s5k->sd);
	fmt = v4l2_subdev_state_get_format(state, 0);

	/* the flips change the Bayer order of the format */
	if (ctrl->id == V4L2_CID_HFLIP || ctrl->id == V4L2_CID_VFLIP)
		fmt->code = s5k4h7_code(s5k);

	if (ctrl->id == V4L2_CID_VBLANK) {
		s64 max = fmt->height + ctrl->val - S5K4H7_EXPOSURE_MARGIN;

		ret = __v4l2_ctrl_modify_range(s5k->exposure,
					       S5K4H7_EXPOSURE_MIN, max, 1,
					       min(s5k->exposure->default_value,
						   max));
		if (ret)
			return ret;
	}

	if (!pm_runtime_get_if_in_use(s5k->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		cci_write(s5k->regmap, S5K4H7_REG_EXPOSURE, ctrl->val, &ret);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		cci_write(s5k->regmap, S5K4H7_REG_ANALOGUE_GAIN, ctrl->val,
			  &ret);
		break;
	case V4L2_CID_VBLANK:
		cci_write(s5k->regmap, S5K4H7_REG_FRAME_LENGTH,
			  fmt->height + ctrl->val, &ret);
		break;
	case V4L2_CID_TEST_PATTERN:
		cci_write(s5k->regmap, S5K4H7_REG_TEST_PATTERN,
			  ctrl->val ? S5K4H7_TEST_PATTERN_BARS : 0, &ret);
		cci_write(s5k->regmap, S5K4H7_REG_TEST_PATTERN_AUX,
			  ctrl->val ? S5K4H7_TEST_PATTERN_AUX_ON :
				      S5K4H7_TEST_PATTERN_AUX_OFF, &ret);
		break;
	case V4L2_CID_HFLIP:
	case V4L2_CID_VFLIP:
		cci_write(s5k->regmap, S5K4H7_REG_ORIENTATION,
			  (s5k->hflip->val ? S5K4H7_HFLIP : 0) |
			  (s5k->vflip->val ? S5K4H7_VFLIP : 0), &ret);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	pm_runtime_put_autosuspend(s5k->dev);

	return ret;
}

static const struct v4l2_ctrl_ops s5k4h7_ctrl_ops = {
	.s_ctrl = s5k4h7_set_ctrl,
};

static int s5k4h7_init_controls(struct s5k4h7 *s5k)
{
	const struct s5k4h7_mode *mode = &s5k4h7_modes[0];
	struct v4l2_ctrl_handler *hdl = &s5k->ctrl_handler;
	struct v4l2_fwnode_device_properties props;
	struct v4l2_ctrl *ctrl;
	s64 vblank_def, hblank;
	int ret;

	v4l2_ctrl_handler_init(hdl, 12);

	v4l2_ctrl_new_std(hdl, NULL, V4L2_CID_PIXEL_RATE, S5K4H7_PIXEL_RATE,
			  S5K4H7_PIXEL_RATE, 1, S5K4H7_PIXEL_RATE);
	ctrl = v4l2_ctrl_new_int_menu(hdl, NULL, V4L2_CID_LINK_FREQ, 0, 0,
				      s5k4h7_link_freqs);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	hblank = S5K4H7_LINE_LENGTH - mode->width;
	s5k->hblank = v4l2_ctrl_new_std(hdl, NULL, V4L2_CID_HBLANK, hblank,
					hblank, 1, hblank);
	if (s5k->hblank)
		s5k->hblank->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	vblank_def = S5K4H7_FRAME_LENGTH_DEF - mode->height;
	s5k->vblank = v4l2_ctrl_new_std(hdl, &s5k4h7_ctrl_ops, V4L2_CID_VBLANK,
					vblank_def,
					S5K4H7_FRAME_LENGTH_MAX - mode->height,
					1, vblank_def);
	s5k->exposure = v4l2_ctrl_new_std(hdl, &s5k4h7_ctrl_ops,
					  V4L2_CID_EXPOSURE, S5K4H7_EXPOSURE_MIN,
					  S5K4H7_FRAME_LENGTH_DEF -
					  S5K4H7_EXPOSURE_MARGIN, 1,
					  S5K4H7_FRAME_LENGTH_DEF -
					  S5K4H7_EXPOSURE_MARGIN);
	v4l2_ctrl_new_std(hdl, &s5k4h7_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			  S5K4H7_GAIN_MIN, S5K4H7_GAIN_MAX, 1, S5K4H7_GAIN_MIN);
	v4l2_ctrl_new_std_menu_items(hdl, &s5k4h7_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(s5k4h7_test_pattern_menu) - 1,
				     0, 0, s5k4h7_test_pattern_menu);
	s5k->hflip = v4l2_ctrl_new_std(hdl, &s5k4h7_ctrl_ops, V4L2_CID_HFLIP,
				       0, 1, 1, 0);
	if (s5k->hflip)
		s5k->hflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;
	s5k->vflip = v4l2_ctrl_new_std(hdl, &s5k4h7_ctrl_ops, V4L2_CID_VFLIP,
				       0, 1, 1, 0);
	if (s5k->vflip)
		s5k->vflip->flags |= V4L2_CTRL_FLAG_MODIFY_LAYOUT;

	if (hdl->error) {
		ret = hdl->error;
		goto err_free;
	}

	ret = v4l2_fwnode_device_parse(s5k->dev, &props);
	if (ret)
		goto err_free;
	ret = v4l2_ctrl_new_fwnode_properties(hdl, &s5k4h7_ctrl_ops, &props);
	if (ret)
		goto err_free;

	s5k->sd.ctrl_handler = hdl;

	return 0;

err_free:
	v4l2_ctrl_handler_free(hdl);
	return dev_err_probe(s5k->dev, ret, "control init failed\n");
}

static void s5k4h7_fill_fmt(struct s5k4h7 *s5k,
			    const struct s5k4h7_mode *mode,
			    struct v4l2_mbus_framefmt *fmt)
{
	fmt->code = s5k4h7_code(s5k);
	fmt->width = mode->width;
	fmt->height = mode->height;
	fmt->field = V4L2_FIELD_NONE;
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->ycbcr_enc = V4L2_YCBCR_ENC_601;
	fmt->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->xfer_func = V4L2_XFER_FUNC_NONE;
}

static int s5k4h7_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index)
		return -EINVAL;

	code->code = s5k4h7_code(to_s5k4h7(sd));

	return 0;
}

static int s5k4h7_enum_frame_size(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index >= ARRAY_SIZE(s5k4h7_modes) ||
	    fse->code != s5k4h7_code(to_s5k4h7(sd)))
		return -EINVAL;

	fse->min_width = s5k4h7_modes[fse->index].width;
	fse->max_width = fse->min_width;
	fse->min_height = s5k4h7_modes[fse->index].height;
	fse->max_height = fse->min_height;

	return 0;
}

static const struct s5k4h7_mode *
s5k4h7_mode_of(const struct v4l2_mbus_framefmt *fmt)
{
	return v4l2_find_nearest_size(s5k4h7_modes, ARRAY_SIZE(s5k4h7_modes),
				      width, height, fmt->width, fmt->height);
}

static int s5k4h7_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *state,
			  struct v4l2_subdev_format *fmt)
{
	struct s5k4h7 *s5k = to_s5k4h7(sd);
	const struct s5k4h7_mode *mode;
	s64 vblank_def, hblank;
	int ret;

	mode = s5k4h7_mode_of(&fmt->format);
	s5k4h7_fill_fmt(s5k, mode, &fmt->format);
	*v4l2_subdev_state_get_format(state, 0) = fmt->format;

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY)
		return 0;

	s5k->mode = mode;

	hblank = S5K4H7_LINE_LENGTH - mode->width;
	ret = __v4l2_ctrl_modify_range(s5k->hblank, hblank, hblank, 1, hblank);
	if (ret)
		return ret;

	/* start a new mode at its own frame length (the exposure limit follows) */
	vblank_def = S5K4H7_FRAME_LENGTH_DEF - mode->height;
	ret = __v4l2_ctrl_modify_range(s5k->vblank, vblank_def,
				       S5K4H7_FRAME_LENGTH_MAX - mode->height,
				       1, vblank_def);
	if (ret)
		return ret;

	return __v4l2_ctrl_s_ctrl(s5k->vblank, vblank_def);
}

static int s5k4h7_get_selection(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *state,
				struct v4l2_subdev_selection *sel)
{
	switch (sel->target) {
	case V4L2_SEL_TGT_NATIVE_SIZE:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = S5K4H7_NATIVE_WIDTH;
		sel->r.height = S5K4H7_NATIVE_HEIGHT;
		return 0;
	case V4L2_SEL_TGT_CROP_DEFAULT:
		sel->r = s5k4h7_modes[0].crop;
		return 0;
	case V4L2_SEL_TGT_CROP:
		sel->r = s5k4h7_mode_of(v4l2_subdev_state_get_format(state, 0))->crop;
		return 0;
	}

	return -EINVAL;
}

static int s5k4h7_get_mbus_config(struct v4l2_subdev *sd, unsigned int pad,
				  struct v4l2_mbus_config *cfg)
{
	struct s5k4h7 *s5k = to_s5k4h7(sd);

	cfg->type = V4L2_MBUS_CSI2_DPHY;
	cfg->link_freq = S5K4H7_LINK_FREQ;
	cfg->bus.mipi_csi2 = s5k->csi2;

	return 0;
}

static int s5k4h7_enable_streams(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state, u32 pad,
				 u64 streams_mask)
{
	struct s5k4h7 *s5k = to_s5k4h7(sd);
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

	ret = cci_multi_reg_write(s5k->regmap, s5k4h7_init_regs,
				  ARRAY_SIZE(s5k4h7_init_regs), NULL);
	if (ret)
		goto err_switch;
	ret = cci_multi_reg_write(s5k->regmap, s5k->mode->regs,
				  s5k->mode->num_regs, NULL);
	if (ret)
		goto err_switch;

	ret = __v4l2_ctrl_handler_setup(s5k->sd.ctrl_handler);
	if (ret)
		goto err_switch;

	/* vendor streaming_control(): 10 ms after stream on */
	cci_write(s5k->regmap, S5K4H7_REG_MODE_SELECT, S5K4H7_MODE_STREAMING,
		  &ret);
	if (ret)
		goto err_switch;
	fsleep(10000);

	/* the Bayer order of the running stream stays */
	__v4l2_ctrl_grab(s5k->hflip, true);
	__v4l2_ctrl_grab(s5k->vflip, true);

	return 0;

err_switch:
	if (s5k->mipi_switch)
		mux_state_deselect(s5k->mipi_switch);
err_put:
	dev_err(s5k->dev, "stream on failed: %d\n", ret);
	pm_runtime_put_autosuspend(s5k->dev);
	return ret;
}

static int s5k4h7_wait_stopped(struct s5k4h7 *s5k)
{
	u64 count = 0;
	int ret;

	return read_poll_timeout(cci_read, ret,
				 ret || count == S5K4H7_FRAME_COUNT_STOPPED,
				 S5K4H7_STOP_POLL_US, S5K4H7_STOP_TIMEOUT_US,
				 false, s5k->regmap, S5K4H7_REG_FRAME_COUNT,
				 &count, NULL) ?: ret;
}

/*
 * CRC-16 with the polynomial 0x8005 and the initial value 0. Module makers
 * store it bit-reflected (CRC-16/ARC) or not (CRC-16/UMTS); the vendor
 * driver accepts both.
 */
static bool s5k4h7_otp_crc_ok(const u8 *data, size_t len, u16 ref)
{
	unsigned int bit;
	u16 crc = 0;
	size_t i;

	if (crc16(0, data, len) == ref)
		return true;

	for (i = 0; i < len; i++) {
		crc ^= data[i] << 8;
		for (bit = 0; bit < 8; bit++)
			crc = crc & BIT(15) ? (crc << 1) ^ S5K4H7_OTP_CRC_POLY :
					      crc << 1;
	}

	return crc == ref;
}

static int s5k4h7_otp_read_page(struct s5k4h7 *s5k, unsigned int page, u8 *buf)
{
	u64 status = 0;
	int ret = 0;
	int err = 0;

	cci_write(s5k->regmap, S5K4H7_REG_OTP_PAGE, page, &ret);
	cci_multi_reg_write(s5k->regmap, s5k4h7_otp_setup_regs,
			    ARRAY_SIZE(s5k4h7_otp_setup_regs), &ret);
	cci_write(s5k->regmap, S5K4H7_REG_OTP_CTRL, S5K4H7_OTP_CTRL_READ, &ret);
	if (!ret)
		ret = read_poll_timeout(cci_read, err,
					err || status == S5K4H7_OTP_STATUS_READY,
					S5K4H7_OTP_POLL_US,
					S5K4H7_OTP_TIMEOUT_US, false,
					s5k->regmap, S5K4H7_REG_OTP_STATUS,
					&status, NULL) ?: err;
	if (!ret)
		ret = regmap_bulk_read(s5k->regmap, S5K4H7_OTP_BUF, buf,
				       S5K4H7_OTP_PAGE_SIZE);

	/* end the access in any case */
	err = 0;
	cci_write(s5k->regmap, S5K4H7_REG_OTP_CTRL, S5K4H7_OTP_CTRL_CLEAR, &err);
	cci_write(s5k->regmap, S5K4H7_REG_OTP_CTRL, S5K4H7_OTP_CTRL_IDLE, &err);

	return ret ?: err;
}

static int s5k4h7_otp_read_pages(struct s5k4h7 *s5k, unsigned int first,
				 unsigned int count, u8 *buf)
{
	unsigned int i;
	int ret;

	for (i = 0; i < count; i++) {
		ret = s5k4h7_otp_read_page(s5k, first + i,
					   buf + i * S5K4H7_OTP_PAGE_SIZE);
		if (ret)
			return ret;
	}

	return 0;
}

static bool s5k4h7_otp_flag_valid(u8 flag)
{
	return (flag & S5K4H7_OTP_FLAG_STATE) == S5K4H7_OTP_FLAG_VALID;
}

/*
 * Copy a block (flag, data, CRC) to the image and check its CRC. The vendor
 * check covers the flag and the data for the module information, light
 * source and white balance blocks; for the other two the coverage is not
 * known, so the CRC is accepted over the data with or without the flag.
 */
static void s5k4h7_otp_put_block(struct s5k4h7 *s5k, unsigned int b,
				 const u8 *src)
{
	const struct nvmem_cell_info *cell = &s5k4h7_otp_cells[b];
	u8 *dst = s5k->otp + cell->offset;
	size_t len = cell->bytes - S5K4H7_OTP_CRC_SIZE;
	u16 crc;

	memcpy(dst, src, cell->bytes);
	crc = get_unaligned_be16(dst + len);
	if (s5k4h7_otp_crc_ok(dst, len, crc) ||
	    s5k4h7_otp_crc_ok(dst + 1, len - 1, crc))
		__set_bit(b, &s5k->otp_valid);
}

/*
 * Read the programmed OTP pages and build the image. The sensor is powered
 * and does not stream; the caller serialises against the stream (probe, or
 * the active state lock).
 */
static int s5k4h7_otp_load(struct s5k4h7 *s5k)
{
	const size_t info_size = S5K4H7_OTP_GROUPS *
				 S5K4H7_OTP_INFO_GROUP_PAGES *
				 S5K4H7_OTP_PAGE_SIZE;
	unsigned int b, g, page;
	u8 *info, *lsc;
	int ret, err = 0;

	info = kzalloc(info_size + S5K4H7_OTP_LSC_GROUP_PAGES *
		       S5K4H7_OTP_PAGE_SIZE, GFP_KERNEL);
	if (!info)
		return -ENOMEM;
	lsc = info + info_size;

	ret = cci_write(s5k->regmap, S5K4H7_REG_MODE_SELECT,
			S5K4H7_MODE_STREAMING, NULL);
	if (ret)
		goto out;
	fsleep(S5K4H7_OTP_START_US);

	memset(s5k->otp, 0, sizeof(s5k->otp));
	s5k->otp_valid = 0;

	ret = s5k4h7_otp_read_pages(s5k, S5K4H7_OTP_INFO_PAGE,
				    S5K4H7_OTP_GROUPS *
				    S5K4H7_OTP_INFO_GROUP_PAGES, info);

	/* the first group with a valid flag of each module data block */
	for (b = 0; !ret && b < S5K4H7_OTP_LSC; b++) {
		for (g = 0; g < S5K4H7_OTP_GROUPS; g++) {
			const u8 *src = info + g * S5K4H7_OTP_INFO_GROUP_PAGES *
					S5K4H7_OTP_PAGE_SIZE +
					s5k4h7_otp_cells[b].offset;

			if (s5k4h7_otp_flag_valid(*src)) {
				s5k4h7_otp_put_block(s5k, b, src);
				break;
			}
		}
	}

	/* the lens shading table: first page of each group for the flag */
	for (g = 0; !ret && g < S5K4H7_OTP_GROUPS; g++) {
		page = S5K4H7_OTP_LSC_PAGE + g * S5K4H7_OTP_LSC_GROUP_PAGES;
		ret = s5k4h7_otp_read_page(s5k, page, lsc);
		if (ret || !s5k4h7_otp_flag_valid(lsc[0]))
			continue;

		ret = s5k4h7_otp_read_pages(s5k, page + 1,
					    S5K4H7_OTP_LSC_GROUP_PAGES - 1,
					    lsc + S5K4H7_OTP_PAGE_SIZE);
		if (!ret)
			s5k4h7_otp_put_block(s5k, S5K4H7_OTP_LSC, lsc);
		break;
	}

	cci_write(s5k->regmap, S5K4H7_REG_MODE_SELECT, S5K4H7_MODE_STANDBY,
		  &err);
	if (!err)
		err = s5k4h7_wait_stopped(s5k);
	ret = ret ?: err;
	if (ret) {
		memset(s5k->otp, 0, sizeof(s5k->otp));
		s5k->otp_valid = 0;
		goto out;
	}

	s5k->otp_read = true;
out:
	kfree(info);
	return ret;
}

/* which blocks are valid; the content (serial number) is not logged */
static void s5k4h7_otp_report(struct s5k4h7 *s5k)
{
	char valid[80] = "";
	size_t len = 0;
	unsigned int b;

	for_each_set_bit(b, &s5k->otp_valid, S5K4H7_OTP_BLOCKS)
		len += scnprintf(valid + len, sizeof(valid) - len, " %s",
				 s5k4h7_otp_cells[b].name);

	dev_info(s5k->dev, "OTP valid blocks:%s\n", len ? valid : " none");
}

static int s5k4h7_nvmem_read(void *priv, unsigned int off, void *val,
			     size_t bytes)
{
	struct s5k4h7 *s5k = priv;
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
		ret = s5k4h7_otp_load(s5k);
		pm_runtime_put_autosuspend(s5k->dev);
		if (ret)
			goto out;
		s5k4h7_otp_report(s5k);
	}

	memcpy(val, s5k->otp + off, bytes);
out:
	v4l2_subdev_unlock_state(state);
	return ret;
}

static int s5k4h7_register_nvmem(struct s5k4h7 *s5k)
{
	struct nvmem_config config = {
		.dev = s5k->dev,
		.name = "s5k4h7-otp",
		.id = NVMEM_DEVID_AUTO,
		.owner = THIS_MODULE,
		.type = NVMEM_TYPE_OTP,
		.read_only = true,
		/* the module information holds the serial number */
		.root_only = true,
		.cells = s5k4h7_otp_cells,
		.ncells = ARRAY_SIZE(s5k4h7_otp_cells),
		.reg_read = s5k4h7_nvmem_read,
		.priv = s5k,
		.size = S5K4H7_OTP_SIZE,
		.word_size = 1,
		.stride = 1,
	};
	struct nvmem_device *nvmem;

	nvmem = nvmem_register(&config);
	if (IS_ERR(nvmem)) {
		/* without NVMEM support the OTP is simply not offered */
		if (PTR_ERR(nvmem) == -EOPNOTSUPP)
			return 0;
		return PTR_ERR(nvmem);
	}
	s5k->nvmem = nvmem;

	return 0;
}

static int s5k4h7_disable_streams(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state, u32 pad,
				  u64 streams_mask)
{
	struct s5k4h7 *s5k = to_s5k4h7(sd);
	int ret;

	/* vendor streaming_control(): wait for the end of the current frame */
	ret = cci_write(s5k->regmap, S5K4H7_REG_MODE_SELECT,
			S5K4H7_MODE_STANDBY, NULL);
	if (!ret)
		ret = s5k4h7_wait_stopped(s5k);
	if (ret)
		dev_warn(s5k->dev, "stream off: %d\n", ret);

	__v4l2_ctrl_grab(s5k->hflip, false);
	__v4l2_ctrl_grab(s5k->vflip, false);

	if (s5k->mipi_switch)
		mux_state_deselect(s5k->mipi_switch);
	pm_runtime_put_autosuspend(s5k->dev);

	return 0;
}

static int s5k4h7_init_state(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state)
{
	struct v4l2_mbus_framefmt *fmt = v4l2_subdev_state_get_format(state, 0);

	s5k4h7_fill_fmt(to_s5k4h7(sd), &s5k4h7_modes[0], fmt);

	return 0;
}

static const struct v4l2_subdev_video_ops s5k4h7_video_ops = {
	.s_stream = v4l2_subdev_s_stream_helper,
};

static const struct v4l2_subdev_pad_ops s5k4h7_pad_ops = {
	.enum_mbus_code = s5k4h7_enum_mbus_code,
	.enum_frame_size = s5k4h7_enum_frame_size,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = s5k4h7_set_fmt,
	.get_selection = s5k4h7_get_selection,
	.get_mbus_config = s5k4h7_get_mbus_config,
	.enable_streams = s5k4h7_enable_streams,
	.disable_streams = s5k4h7_disable_streams,
};

static const struct v4l2_subdev_ops s5k4h7_subdev_ops = {
	.video = &s5k4h7_video_ops,
	.pad = &s5k4h7_pad_ops,
};

static const struct v4l2_subdev_internal_ops s5k4h7_internal_ops = {
	.init_state = s5k4h7_init_state,
};

static int s5k4h7_check_endpoint(struct s5k4h7 *s5k)
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

	if (bus_cfg.bus.mipi_csi2.num_data_lanes != S5K4H7_LANES) {
		ret = dev_err_probe(s5k->dev, -EINVAL, "only 4 data lanes\n");
		goto out;
	}
	for (i = 0; i < bus_cfg.nr_of_link_frequencies; i++)
		if (bus_cfg.link_frequencies[i] == S5K4H7_LINK_FREQ)
			break;
	if (i == bus_cfg.nr_of_link_frequencies) {
		ret = dev_err_probe(s5k->dev, -EINVAL,
				    "link frequency %llu not in DT\n",
				    S5K4H7_LINK_FREQ);
		goto out;
	}

	s5k->csi2 = bus_cfg.bus.mipi_csi2;
out:
	v4l2_fwnode_endpoint_free(&bus_cfg);
	return ret;
}

static int s5k4h7_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct s5k4h7 *s5k;
	unsigned int i;
	u64 id;
	int ret;

	s5k = devm_kzalloc(dev, sizeof(*s5k), GFP_KERNEL);
	if (!s5k)
		return -ENOMEM;
	s5k->dev = dev;
	s5k->mode = &s5k4h7_modes[0];

	ret = s5k4h7_check_endpoint(s5k);
	if (ret)
		return ret;

	s5k->regmap = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(s5k->regmap))
		return dev_err_probe(dev, PTR_ERR(s5k->regmap),
				     "CCI init failed\n");

	s5k->mclk = devm_v4l2_sensor_clk_get(dev, NULL);
	if (IS_ERR(s5k->mclk))
		return dev_err_probe(dev, PTR_ERR(s5k->mclk), "no mclk\n");
	if (clk_get_rate(s5k->mclk) != S5K4H7_MCLK_FREQ)
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

	for (i = 0; i < ARRAY_SIZE(s5k4h7_supply_names); i++)
		s5k->supplies[i].supply = s5k4h7_supply_names[i];
	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(s5k4h7_supply_names),
				      s5k->supplies);
	if (ret)
		return dev_err_probe(dev, ret, "supplies\n");

	v4l2_i2c_subdev_init(&s5k->sd, client, &s5k4h7_subdev_ops);

	ret = s5k4h7_power_on(dev);
	if (ret)
		return dev_err_probe(dev, ret, "power on failed\n");

	ret = cci_read(s5k->regmap, S5K4H7_REG_CHIP_ID, &id, NULL);
	if (ret || id != S5K4H7_CHIP_ID) {
		ret = dev_err_probe(dev, ret ?: -ENODEV, "chip id 0x%04llx\n",
				    id);
		goto err_power_off;
	}

	/* like the vendor driver, read the OTP once while detecting the sensor */
	ret = s5k4h7_otp_load(s5k);
	if (ret)
		dev_warn(dev, "OTP read failed: %d\n", ret);
	else
		s5k4h7_otp_report(s5k);

	ret = s5k4h7_init_controls(s5k);
	if (ret)
		goto err_power_off;

	s5k->sd.internal_ops = &s5k4h7_internal_ops;
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

	ret = s5k4h7_register_nvmem(s5k);
	if (ret) {
		dev_err_probe(dev, ret, "nvmem registration failed\n");
		goto err_pm;
	}

	ret = v4l2_async_register_subdev_sensor(&s5k->sd);
	if (ret)
		goto err_nvmem;

	pm_runtime_put_autosuspend(dev);

	dev_info(dev, "S5K4H7 0x%04llx\n", id);

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
	s5k4h7_power_off(dev);
	return ret;
}

static void s5k4h7_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct s5k4h7 *s5k = to_s5k4h7(sd);

	nvmem_unregister(s5k->nvmem);
	v4l2_async_unregister_subdev(sd);
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(&s5k->ctrl_handler);

	pm_runtime_dont_use_autosuspend(s5k->dev);
	pm_runtime_disable(s5k->dev);
	if (!pm_runtime_status_suspended(s5k->dev))
		s5k4h7_power_off(s5k->dev);
	pm_runtime_set_suspended(s5k->dev);
}

static DEFINE_RUNTIME_DEV_PM_OPS(s5k4h7_pm_ops, s5k4h7_power_off,
				 s5k4h7_power_on, NULL);

static const struct of_device_id s5k4h7_of_match[] = {
	{ .compatible = "samsung,s5k4h7" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, s5k4h7_of_match);

static struct i2c_driver s5k4h7_driver = {
	.driver = {
		.name = "s5k4h7",
		.of_match_table = s5k4h7_of_match,
		.pm = pm_ptr(&s5k4h7_pm_ops),
	},
	.probe = s5k4h7_probe,
	.remove = s5k4h7_remove,
};
module_i2c_driver(s5k4h7_driver);

MODULE_DESCRIPTION("Samsung S5K4H7 image sensor driver");
MODULE_LICENSE("GPL");
