// SPDX-License-Identifier: GPL-2.0
/*
 * OmniVision OV16A1Q image sensor driver (16 MP quad Bayer, 4-lane MIPI CSI-2)
 *
 * One mode: 2304x1728 (2x2 binned, cropped), SBGGR10, 4 lanes at 745 Mbit/s,
 * 30 fps. The register sequences (sensor_init, preview_setting), the power
 * sequence, the exposure, gain and test pattern registers come from the
 * vendor kernel of the Motorola Moto G60s (GPL-2.0).
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/pinctrl/consumer.h>
#include <linux/pm_runtime.h>
#include <linux/regulator/consumer.h>
#include <media/v4l2-cci.h>
#include <media/v4l2-common.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-fwnode.h>

#define OV16A1Q_LANES			4
#define OV16A1Q_BITS_PER_SAMPLE		10
#define OV16A1Q_LINK_FREQ		372500000ULL	/* 745 Mbit/s per lane */
#define OV16A1Q_PIXEL_RATE		298000000ULL
#define OV16A1Q_MCLK_FREQ		24000000

#define OV16A1Q_REG_CHIP_ID		CCI_REG16(0x300b)
#define OV16A1Q_CHIP_ID			0x1641

#define OV16A1Q_REG_MODE_SELECT		CCI_REG8(0x0100)
#define OV16A1Q_MODE_STANDBY		0x00
#define OV16A1Q_MODE_STREAMING		0x01
#define OV16A1Q_REG_SOFTWARE_RESET	CCI_REG8(0x0103)

#define OV16A1Q_REG_EXPOSURE		CCI_REG24(0x3500)
#define OV16A1Q_EXPOSURE_MIN		8
#define OV16A1Q_EXPOSURE_MARGIN		8
#define OV16A1Q_REG_ANALOGUE_GAIN	CCI_REG16(0x3508)
#define OV16A1Q_GAIN_MIN		0x100	/* 1x */
#define OV16A1Q_GAIN_MAX		0xf80	/* 15.5x */
#define OV16A1Q_REG_VTS			CCI_REG16(0x380e)
#define OV16A1Q_VTS_MAX			0x7ffe
#define OV16A1Q_REG_TEST_PATTERN	CCI_REG8(0x5081)

#define OV16A1Q_NATIVE_WIDTH		4656
#define OV16A1Q_NATIVE_HEIGHT		3496

/*
 * The vendor timing is HTS 850 x VTS 3920 at a 100 MHz timing clock (30 fps).
 * Expressed in pixel clock periods of the 298 MHz pixel rate one line is
 * 850 * 298 / 100 = 2533 pixels.
 */
#define OV16A1Q_LINE_LENGTH		2533

struct ov16a1q_mode {
	u32 width;
	u32 height;
	u32 vts;
	const struct cci_reg_sequence *regs;
	unsigned int num_regs;
};

static const char * const ov16a1q_supply_names[] = {
	"dovdd",	/* digital I/O, first in the vendor sequence */
	"avdd",
	"dvdd",
};

static const struct cci_reg_sequence ov16a1q_init_regs[] = {
	{ CCI_REG8(0x0102), 0x01 },
	{ CCI_REG8(0x0301), 0x48 },
	{ CCI_REG8(0x0302), 0x31 },
	{ CCI_REG8(0x0303), 0x04 },
	{ CCI_REG8(0x0305), 0xc2 },
	{ CCI_REG8(0x0306), 0x00 },
	{ CCI_REG8(0x0320), 0x02 },
	{ CCI_REG8(0x0323), 0x04 },
	{ CCI_REG8(0x0326), 0xd8 },
	{ CCI_REG8(0x0327), 0x0b },
	{ CCI_REG8(0x0329), 0x01 },
	{ CCI_REG8(0x0343), 0x04 },
	{ CCI_REG8(0x0344), 0x01 },
	{ CCI_REG8(0x0345), 0x2c },
	{ CCI_REG8(0x0346), 0xc0 },
	{ CCI_REG8(0x034a), 0x07 },
	{ CCI_REG8(0x300e), 0x22 },
	{ CCI_REG8(0x3012), 0x41 },
	{ CCI_REG8(0x3016), 0xd2 },
	{ CCI_REG8(0x3018), 0x70 },
	{ CCI_REG8(0x301e), 0x98 },
	{ CCI_REG8(0x3025), 0x03 },
	{ CCI_REG8(0x3026), 0x10 },
	{ CCI_REG8(0x3027), 0x08 },
	{ CCI_REG8(0x3102), 0x00 },
	{ CCI_REG8(0x3400), 0x04 },
	{ CCI_REG8(0x3406), 0x04 },
	{ CCI_REG8(0x3408), 0x04 },
	{ CCI_REG8(0x3421), 0x09 },
	{ CCI_REG8(0x3422), 0x20 },
	{ CCI_REG8(0x3423), 0x15 },
	{ CCI_REG8(0x3424), 0x40 },
	{ CCI_REG8(0x3425), 0x14 },
	{ CCI_REG8(0x3426), 0x04 },
	{ CCI_REG8(0x3504), 0x08 },
	{ CCI_REG8(0x3508), 0x01 },
	{ CCI_REG8(0x3509), 0x00 },
	{ CCI_REG8(0x350a), 0x01 },
	{ CCI_REG8(0x350b), 0x00 },
	{ CCI_REG8(0x350c), 0x00 },
	{ CCI_REG8(0x3548), 0x01 },
	{ CCI_REG8(0x3549), 0x00 },
	{ CCI_REG8(0x354a), 0x01 },
	{ CCI_REG8(0x354b), 0x00 },
	{ CCI_REG8(0x354c), 0x00 },
	{ CCI_REG8(0x3600), 0xff },
	{ CCI_REG8(0x3602), 0x42 },
	{ CCI_REG8(0x3603), 0x7b },
	{ CCI_REG8(0x3608), 0x9b },
	{ CCI_REG8(0x360a), 0x69 },
	{ CCI_REG8(0x360b), 0x53 },
	{ CCI_REG8(0x3618), 0xc0 },
	{ CCI_REG8(0x361a), 0x8b },
	{ CCI_REG8(0x361d), 0x20 },
	{ CCI_REG8(0x361e), 0x30 },
	{ CCI_REG8(0x361f), 0x01 },
	{ CCI_REG8(0x3620), 0x89 },
	{ CCI_REG8(0x3624), 0x8f },
	{ CCI_REG8(0x3629), 0x09 },
	{ CCI_REG8(0x362e), 0x50 },
	{ CCI_REG8(0x3631), 0xe2 },
	{ CCI_REG8(0x3632), 0xe2 },
	{ CCI_REG8(0x3634), 0x10 },
	{ CCI_REG8(0x3635), 0x10 },
	{ CCI_REG8(0x3636), 0x10 },
	{ CCI_REG8(0x3639), 0xa6 },
	{ CCI_REG8(0x363a), 0xaa },
	{ CCI_REG8(0x363b), 0x0c },
	{ CCI_REG8(0x363c), 0x16 },
	{ CCI_REG8(0x363d), 0x29 },
	{ CCI_REG8(0x363e), 0x4f },
	{ CCI_REG8(0x3642), 0xa8 },
	{ CCI_REG8(0x3652), 0x00 },
	{ CCI_REG8(0x3653), 0x00 },
	{ CCI_REG8(0x3654), 0x8a },
	{ CCI_REG8(0x3656), 0x0c },
	{ CCI_REG8(0x3657), 0x8e },
	{ CCI_REG8(0x3660), 0x80 },
	{ CCI_REG8(0x3663), 0x00 },
	{ CCI_REG8(0x3664), 0x00 },
	{ CCI_REG8(0x3668), 0x05 },
	{ CCI_REG8(0x3669), 0x05 },
	{ CCI_REG8(0x370d), 0x10 },
	{ CCI_REG8(0x370e), 0x05 },
	{ CCI_REG8(0x370f), 0x10 },
	{ CCI_REG8(0x3711), 0x01 },
	{ CCI_REG8(0x3712), 0x09 },
	{ CCI_REG8(0x3713), 0x40 },
	{ CCI_REG8(0x3714), 0xe4 },
	{ CCI_REG8(0x3716), 0x04 },
	{ CCI_REG8(0x3717), 0x01 },
	{ CCI_REG8(0x3718), 0x02 },
	{ CCI_REG8(0x3719), 0x01 },
	{ CCI_REG8(0x371a), 0x02 },
	{ CCI_REG8(0x371b), 0x02 },
	{ CCI_REG8(0x371c), 0x01 },
	{ CCI_REG8(0x371d), 0x02 },
	{ CCI_REG8(0x371e), 0x12 },
	{ CCI_REG8(0x371f), 0x02 },
	{ CCI_REG8(0x3720), 0x14 },
	{ CCI_REG8(0x3721), 0x12 },
	{ CCI_REG8(0x3722), 0x44 },
	{ CCI_REG8(0x3723), 0x60 },
	{ CCI_REG8(0x372f), 0x34 },
	{ CCI_REG8(0x3726), 0x21 },
	{ CCI_REG8(0x37d0), 0x02 },
	{ CCI_REG8(0x37d1), 0x10 },
	{ CCI_REG8(0x37db), 0x08 },
	{ CCI_REG8(0x3808), 0x12 },
	{ CCI_REG8(0x3809), 0x30 },
	{ CCI_REG8(0x380a), 0x0d },
	{ CCI_REG8(0x380b), 0xa8 },
	{ CCI_REG8(0x380c), 0x03 },
	{ CCI_REG8(0x380d), 0x52 },
	{ CCI_REG8(0x380e), 0x0f },
	{ CCI_REG8(0x380f), 0x50 },
	{ CCI_REG8(0x3814), 0x11 },
	{ CCI_REG8(0x3815), 0x11 },
	{ CCI_REG8(0x3820), 0x00 },
	{ CCI_REG8(0x3821), 0x06 },
	{ CCI_REG8(0x3822), 0x00 },
	{ CCI_REG8(0x3823), 0x04 },
	{ CCI_REG8(0x3837), 0x10 },
	{ CCI_REG8(0x383c), 0x34 },
	{ CCI_REG8(0x383d), 0xff },
	{ CCI_REG8(0x383e), 0x0d },
	{ CCI_REG8(0x383f), 0x22 },
	{ CCI_REG8(0x3857), 0x00 },
	{ CCI_REG8(0x388f), 0x00 },
	{ CCI_REG8(0x3890), 0x00 },
	{ CCI_REG8(0x3891), 0x00 },
	{ CCI_REG8(0x3d81), 0x10 },
	{ CCI_REG8(0x3d83), 0x0c },
	{ CCI_REG8(0x3d84), 0x00 },
	{ CCI_REG8(0x3d85), 0x1b },
	{ CCI_REG8(0x3d88), 0x00 },
	{ CCI_REG8(0x3d89), 0x00 },
	{ CCI_REG8(0x3d8a), 0x00 },
	{ CCI_REG8(0x3d8b), 0x01 },
	{ CCI_REG8(0x3d8c), 0x77 },
	{ CCI_REG8(0x3d8d), 0xa0 },
	{ CCI_REG8(0x3f00), 0x02 },
	{ CCI_REG8(0x3f0c), 0x07 },
	{ CCI_REG8(0x3f0d), 0x2f },
	{ CCI_REG8(0x4012), 0x0d },
	{ CCI_REG8(0x4015), 0x04 },
	{ CCI_REG8(0x4016), 0x1b },
	{ CCI_REG8(0x4017), 0x04 },
	{ CCI_REG8(0x4018), 0x0b },
	{ CCI_REG8(0x401b), 0x1f },
	{ CCI_REG8(0x401e), 0x01 },
	{ CCI_REG8(0x401f), 0x38 },
	{ CCI_REG8(0x4500), 0x20 },
	{ CCI_REG8(0x4501), 0x6a },
	{ CCI_REG8(0x4502), 0xb4 },
	{ CCI_REG8(0x4586), 0x00 },
	{ CCI_REG8(0x4588), 0x02 },
	{ CCI_REG8(0x4640), 0x01 },
	{ CCI_REG8(0x4641), 0x04 },
	{ CCI_REG8(0x4643), 0x00 },
	{ CCI_REG8(0x4645), 0x03 },
	{ CCI_REG8(0x4806), 0x40 },
	{ CCI_REG8(0x480e), 0x00 },
	{ CCI_REG8(0x4815), 0x2b },
	{ CCI_REG8(0x481b), 0x3c },
	{ CCI_REG8(0x4833), 0x18 },
	{ CCI_REG8(0x4837), 0x08 },
	{ CCI_REG8(0x484b), 0x07 },
	{ CCI_REG8(0x4850), 0x41 },
	{ CCI_REG8(0x4860), 0x00 },
	{ CCI_REG8(0x4861), 0xec },
	{ CCI_REG8(0x4864), 0x00 },
	{ CCI_REG8(0x4883), 0x00 },
	{ CCI_REG8(0x4888), 0x10 },
	{ CCI_REG8(0x4a00), 0x10 },
	{ CCI_REG8(0x4e00), 0x00 },
	{ CCI_REG8(0x4e01), 0x04 },
	{ CCI_REG8(0x4e02), 0x01 },
	{ CCI_REG8(0x4e03), 0x00 },
	{ CCI_REG8(0x4e04), 0x08 },
	{ CCI_REG8(0x4e05), 0x04 },
	{ CCI_REG8(0x4e06), 0x00 },
	{ CCI_REG8(0x4e07), 0x13 },
	{ CCI_REG8(0x4e08), 0x01 },
	{ CCI_REG8(0x4e09), 0x00 },
	{ CCI_REG8(0x4e0a), 0x15 },
	{ CCI_REG8(0x4e0b), 0x0e },
	{ CCI_REG8(0x4e0c), 0x00 },
	{ CCI_REG8(0x4e0d), 0x17 },
	{ CCI_REG8(0x4e0e), 0x07 },
	{ CCI_REG8(0x4e0f), 0x00 },
	{ CCI_REG8(0x4e10), 0x19 },
	{ CCI_REG8(0x4e11), 0x06 },
	{ CCI_REG8(0x4e12), 0x00 },
	{ CCI_REG8(0x4e13), 0x1b },
	{ CCI_REG8(0x4e14), 0x08 },
	{ CCI_REG8(0x4e15), 0x00 },
	{ CCI_REG8(0x4e16), 0x1f },
	{ CCI_REG8(0x4e17), 0x08 },
	{ CCI_REG8(0x4e18), 0x00 },
	{ CCI_REG8(0x4e19), 0x21 },
	{ CCI_REG8(0x4e1a), 0x0e },
	{ CCI_REG8(0x4e1b), 0x00 },
	{ CCI_REG8(0x4e1c), 0x2d },
	{ CCI_REG8(0x4e1d), 0x30 },
	{ CCI_REG8(0x4e1e), 0x00 },
	{ CCI_REG8(0x4e1f), 0x6a },
	{ CCI_REG8(0x4e20), 0x05 },
	{ CCI_REG8(0x4e21), 0x00 },
	{ CCI_REG8(0x4e22), 0x6c },
	{ CCI_REG8(0x4e23), 0x05 },
	{ CCI_REG8(0x4e24), 0x00 },
	{ CCI_REG8(0x4e25), 0x6e },
	{ CCI_REG8(0x4e26), 0x39 },
	{ CCI_REG8(0x4e27), 0x00 },
	{ CCI_REG8(0x4e28), 0x7a },
	{ CCI_REG8(0x4e29), 0x6d },
	{ CCI_REG8(0x4e2a), 0x00 },
	{ CCI_REG8(0x4e2b), 0x00 },
	{ CCI_REG8(0x4e2c), 0x00 },
	{ CCI_REG8(0x4e2d), 0x00 },
	{ CCI_REG8(0x4e2e), 0x00 },
	{ CCI_REG8(0x4e2f), 0x00 },
	{ CCI_REG8(0x4e30), 0x00 },
	{ CCI_REG8(0x4e31), 0x00 },
	{ CCI_REG8(0x4e32), 0x00 },
	{ CCI_REG8(0x4e33), 0x00 },
	{ CCI_REG8(0x4e34), 0x00 },
	{ CCI_REG8(0x4e35), 0x00 },
	{ CCI_REG8(0x4e36), 0x00 },
	{ CCI_REG8(0x4e37), 0x00 },
	{ CCI_REG8(0x4e38), 0x00 },
	{ CCI_REG8(0x4e39), 0x00 },
	{ CCI_REG8(0x4e3a), 0x00 },
	{ CCI_REG8(0x4e3b), 0x00 },
	{ CCI_REG8(0x4e3c), 0x00 },
	{ CCI_REG8(0x4e3d), 0x00 },
	{ CCI_REG8(0x4e3e), 0x00 },
	{ CCI_REG8(0x4e3f), 0x00 },
	{ CCI_REG8(0x4e40), 0x00 },
	{ CCI_REG8(0x4e41), 0x00 },
	{ CCI_REG8(0x4e42), 0x00 },
	{ CCI_REG8(0x4e43), 0x00 },
	{ CCI_REG8(0x4e44), 0x00 },
	{ CCI_REG8(0x4e45), 0x00 },
	{ CCI_REG8(0x4e46), 0x00 },
	{ CCI_REG8(0x4e47), 0x00 },
	{ CCI_REG8(0x4e48), 0x00 },
	{ CCI_REG8(0x4e49), 0x00 },
	{ CCI_REG8(0x4e4a), 0x00 },
	{ CCI_REG8(0x4e4b), 0x00 },
	{ CCI_REG8(0x4e4c), 0x00 },
	{ CCI_REG8(0x4e4d), 0x00 },
	{ CCI_REG8(0x4e4e), 0x00 },
	{ CCI_REG8(0x4e4f), 0x00 },
	{ CCI_REG8(0x4e50), 0x00 },
	{ CCI_REG8(0x4e51), 0x00 },
	{ CCI_REG8(0x4e52), 0x00 },
	{ CCI_REG8(0x4e53), 0x00 },
	{ CCI_REG8(0x4e54), 0x00 },
	{ CCI_REG8(0x4e55), 0x00 },
	{ CCI_REG8(0x4e56), 0x00 },
	{ CCI_REG8(0x4e57), 0x00 },
	{ CCI_REG8(0x4e58), 0x00 },
	{ CCI_REG8(0x4e59), 0x00 },
	{ CCI_REG8(0x4e5a), 0x00 },
	{ CCI_REG8(0x4e5b), 0x00 },
	{ CCI_REG8(0x4e5c), 0x00 },
	{ CCI_REG8(0x4e5d), 0x00 },
	{ CCI_REG8(0x4e5e), 0x00 },
	{ CCI_REG8(0x4e5f), 0x00 },
	{ CCI_REG8(0x4e60), 0x00 },
	{ CCI_REG8(0x4e61), 0x00 },
	{ CCI_REG8(0x4e62), 0x00 },
	{ CCI_REG8(0x4e63), 0x00 },
	{ CCI_REG8(0x4e64), 0x00 },
	{ CCI_REG8(0x4e65), 0x00 },
	{ CCI_REG8(0x4e66), 0x00 },
	{ CCI_REG8(0x4e67), 0x00 },
	{ CCI_REG8(0x4e68), 0x00 },
	{ CCI_REG8(0x4e69), 0x00 },
	{ CCI_REG8(0x4e6a), 0x00 },
	{ CCI_REG8(0x4e6b), 0x00 },
	{ CCI_REG8(0x4e6c), 0x00 },
	{ CCI_REG8(0x4e6d), 0x00 },
	{ CCI_REG8(0x4e6e), 0x00 },
	{ CCI_REG8(0x4e6f), 0x00 },
	{ CCI_REG8(0x4e70), 0x00 },
	{ CCI_REG8(0x4e71), 0x00 },
	{ CCI_REG8(0x4e72), 0x00 },
	{ CCI_REG8(0x4e73), 0x00 },
	{ CCI_REG8(0x4e74), 0x00 },
	{ CCI_REG8(0x4e75), 0x00 },
	{ CCI_REG8(0x4e76), 0x00 },
	{ CCI_REG8(0x4e77), 0x00 },
	{ CCI_REG8(0x4e78), 0x1c },
	{ CCI_REG8(0x4e79), 0x1e },
	{ CCI_REG8(0x4e7a), 0x00 },
	{ CCI_REG8(0x4e7b), 0x00 },
	{ CCI_REG8(0x4e7c), 0x2c },
	{ CCI_REG8(0x4e7d), 0x2f },
	{ CCI_REG8(0x4e7e), 0x79 },
	{ CCI_REG8(0x4e7f), 0x7b },
	{ CCI_REG8(0x4e80), 0x0a },
	{ CCI_REG8(0x4e81), 0x31 },
	{ CCI_REG8(0x4e82), 0x66 },
	{ CCI_REG8(0x4e83), 0x81 },
	{ CCI_REG8(0x4e84), 0x03 },
	{ CCI_REG8(0x4e85), 0x40 },
	{ CCI_REG8(0x4e86), 0x02 },
	{ CCI_REG8(0x4e87), 0x09 },
	{ CCI_REG8(0x4e88), 0x43 },
	{ CCI_REG8(0x4e89), 0x53 },
	{ CCI_REG8(0x4e8a), 0x32 },
	{ CCI_REG8(0x4e8b), 0x67 },
	{ CCI_REG8(0x4e8c), 0x05 },
	{ CCI_REG8(0x4e8d), 0x83 },
	{ CCI_REG8(0x4e8e), 0x00 },
	{ CCI_REG8(0x4e8f), 0x00 },
	{ CCI_REG8(0x4e90), 0x00 },
	{ CCI_REG8(0x4e91), 0x00 },
	{ CCI_REG8(0x4e92), 0x00 },
	{ CCI_REG8(0x4e93), 0x00 },
	{ CCI_REG8(0x4e94), 0x00 },
	{ CCI_REG8(0x4e95), 0x00 },
	{ CCI_REG8(0x4e96), 0x00 },
	{ CCI_REG8(0x4e97), 0x00 },
	{ CCI_REG8(0x4e98), 0x00 },
	{ CCI_REG8(0x4e99), 0x00 },
	{ CCI_REG8(0x4e9a), 0x00 },
	{ CCI_REG8(0x4e9b), 0x00 },
	{ CCI_REG8(0x4e9c), 0x00 },
	{ CCI_REG8(0x4e9d), 0x00 },
	{ CCI_REG8(0x4e9e), 0x00 },
	{ CCI_REG8(0x4e9f), 0x00 },
	{ CCI_REG8(0x4ea0), 0x00 },
	{ CCI_REG8(0x4ea1), 0x00 },
	{ CCI_REG8(0x4ea2), 0x00 },
	{ CCI_REG8(0x4ea3), 0x00 },
	{ CCI_REG8(0x4ea4), 0x00 },
	{ CCI_REG8(0x4ea5), 0x00 },
	{ CCI_REG8(0x4ea6), 0x1e },
	{ CCI_REG8(0x4ea7), 0x20 },
	{ CCI_REG8(0x4ea8), 0x32 },
	{ CCI_REG8(0x4ea9), 0x6d },
	{ CCI_REG8(0x4eaa), 0x18 },
	{ CCI_REG8(0x4eab), 0x7f },
	{ CCI_REG8(0x4eac), 0x00 },
	{ CCI_REG8(0x4ead), 0x00 },
	{ CCI_REG8(0x4eae), 0x7c },
	{ CCI_REG8(0x4eaf), 0x07 },
	{ CCI_REG8(0x4eb0), 0x7c },
	{ CCI_REG8(0x4eb1), 0x07 },
	{ CCI_REG8(0x4eb2), 0x07 },
	{ CCI_REG8(0x4eb3), 0x1c },
	{ CCI_REG8(0x4eb4), 0x07 },
	{ CCI_REG8(0x4eb5), 0x1c },
	{ CCI_REG8(0x4eb6), 0x07 },
	{ CCI_REG8(0x4eb7), 0x1c },
	{ CCI_REG8(0x4eb8), 0x07 },
	{ CCI_REG8(0x4eb9), 0x1c },
	{ CCI_REG8(0x4eba), 0x07 },
	{ CCI_REG8(0x4ebb), 0x14 },
	{ CCI_REG8(0x4ebc), 0x07 },
	{ CCI_REG8(0x4ebd), 0x1c },
	{ CCI_REG8(0x4ebe), 0x07 },
	{ CCI_REG8(0x4ebf), 0x1c },
	{ CCI_REG8(0x4ec0), 0x07 },
	{ CCI_REG8(0x4ec1), 0x1c },
	{ CCI_REG8(0x4ec2), 0x07 },
	{ CCI_REG8(0x4ec3), 0x1c },
	{ CCI_REG8(0x4ec4), 0x2c },
	{ CCI_REG8(0x4ec5), 0x2f },
	{ CCI_REG8(0x4ec6), 0x79 },
	{ CCI_REG8(0x4ec7), 0x7b },
	{ CCI_REG8(0x4ec8), 0x7c },
	{ CCI_REG8(0x4ec9), 0x07 },
	{ CCI_REG8(0x4eca), 0x7c },
	{ CCI_REG8(0x4ecb), 0x07 },
	{ CCI_REG8(0x4ecc), 0x00 },
	{ CCI_REG8(0x4ecd), 0x00 },
	{ CCI_REG8(0x4ece), 0x07 },
	{ CCI_REG8(0x4ecf), 0x31 },
	{ CCI_REG8(0x4ed0), 0x69 },
	{ CCI_REG8(0x4ed1), 0x7f },
	{ CCI_REG8(0x4ed2), 0x67 },
	{ CCI_REG8(0x4ed3), 0x00 },
	{ CCI_REG8(0x4ed4), 0x00 },
	{ CCI_REG8(0x4ed5), 0x00 },
	{ CCI_REG8(0x4ed6), 0x7c },
	{ CCI_REG8(0x4ed7), 0x07 },
	{ CCI_REG8(0x4ed8), 0x7c },
	{ CCI_REG8(0x4ed9), 0x07 },
	{ CCI_REG8(0x4eda), 0x33 },
	{ CCI_REG8(0x4edb), 0x7f },
	{ CCI_REG8(0x4edc), 0x00 },
	{ CCI_REG8(0x4edd), 0x16 },
	{ CCI_REG8(0x4ede), 0x00 },
	{ CCI_REG8(0x4edf), 0x00 },
	{ CCI_REG8(0x4ee0), 0x32 },
	{ CCI_REG8(0x4ee1), 0x70 },
	{ CCI_REG8(0x4ee2), 0x01 },
	{ CCI_REG8(0x4ee3), 0x30 },
	{ CCI_REG8(0x4ee4), 0x22 },
	{ CCI_REG8(0x4ee5), 0x28 },
	{ CCI_REG8(0x4ee6), 0x6f },
	{ CCI_REG8(0x4ee7), 0x75 },
	{ CCI_REG8(0x4ee8), 0x00 },
	{ CCI_REG8(0x4ee9), 0x00 },
	{ CCI_REG8(0x4eea), 0x30 },
	{ CCI_REG8(0x4eeb), 0x7f },
	{ CCI_REG8(0x4eec), 0x00 },
	{ CCI_REG8(0x4eed), 0x00 },
	{ CCI_REG8(0x4eee), 0x00 },
	{ CCI_REG8(0x4eef), 0x00 },
	{ CCI_REG8(0x4ef0), 0x69 },
	{ CCI_REG8(0x4ef1), 0x7f },
	{ CCI_REG8(0x4ef2), 0x07 },
	{ CCI_REG8(0x4ef3), 0x30 },
	{ CCI_REG8(0x4ef4), 0x32 },
	{ CCI_REG8(0x4ef5), 0x09 },
	{ CCI_REG8(0x4ef6), 0x7d },
	{ CCI_REG8(0x4ef7), 0x65 },
	{ CCI_REG8(0x4ef8), 0x00 },
	{ CCI_REG8(0x4ef9), 0x00 },
	{ CCI_REG8(0x4efa), 0x00 },
	{ CCI_REG8(0x4efb), 0x00 },
	{ CCI_REG8(0x4efc), 0x7f },
	{ CCI_REG8(0x4efd), 0x09 },
	{ CCI_REG8(0x4efe), 0x7f },
	{ CCI_REG8(0x4eff), 0x09 },
	{ CCI_REG8(0x4f00), 0x1e },
	{ CCI_REG8(0x4f01), 0x7c },
	{ CCI_REG8(0x4f02), 0x7f },
	{ CCI_REG8(0x4f03), 0x09 },
	{ CCI_REG8(0x4f04), 0x7f },
	{ CCI_REG8(0x4f05), 0x0b },
	{ CCI_REG8(0x4f06), 0x7c },
	{ CCI_REG8(0x4f07), 0x02 },
	{ CCI_REG8(0x4f08), 0x7c },
	{ CCI_REG8(0x4f09), 0x02 },
	{ CCI_REG8(0x4f0a), 0x32 },
	{ CCI_REG8(0x4f0b), 0x64 },
	{ CCI_REG8(0x4f0c), 0x32 },
	{ CCI_REG8(0x4f0d), 0x64 },
	{ CCI_REG8(0x4f0e), 0x32 },
	{ CCI_REG8(0x4f0f), 0x64 },
	{ CCI_REG8(0x4f10), 0x32 },
	{ CCI_REG8(0x4f11), 0x64 },
	{ CCI_REG8(0x4f12), 0x31 },
	{ CCI_REG8(0x4f13), 0x4f },
	{ CCI_REG8(0x4f14), 0x83 },
	{ CCI_REG8(0x4f15), 0x84 },
	{ CCI_REG8(0x4f16), 0x63 },
	{ CCI_REG8(0x4f17), 0x64 },
	{ CCI_REG8(0x4f18), 0x83 },
	{ CCI_REG8(0x4f19), 0x84 },
	{ CCI_REG8(0x4f1a), 0x31 },
	{ CCI_REG8(0x4f1b), 0x32 },
	{ CCI_REG8(0x4f1c), 0x7b },
	{ CCI_REG8(0x4f1d), 0x7c },
	{ CCI_REG8(0x4f1e), 0x2f },
	{ CCI_REG8(0x4f1f), 0x30 },
	{ CCI_REG8(0x4f20), 0x30 },
	{ CCI_REG8(0x4f21), 0x69 },
	{ CCI_REG8(0x4d06), 0x08 },
	{ CCI_REG8(0x5000), 0x01 },
	{ CCI_REG8(0x5001), 0x40 },
	{ CCI_REG8(0x5002), 0x53 },
	{ CCI_REG8(0x5003), 0x42 },
	{ CCI_REG8(0x5004), 0x08 },
	{ CCI_REG8(0x5005), 0x00 },
	{ CCI_REG8(0x5012), 0x60 },
	{ CCI_REG8(0x5038), 0x00 },
	{ CCI_REG8(0x5081), 0x00 },
	{ CCI_REG8(0x5180), 0x00 },
	{ CCI_REG8(0x5181), 0x10 },
	{ CCI_REG8(0x5182), 0x07 },
	{ CCI_REG8(0x5183), 0x8f },
	{ CCI_REG8(0x5184), 0x03 },
	{ CCI_REG8(0x5208), 0xc2 },
	{ CCI_REG8(0x5820), 0xc5 },
	{ CCI_REG8(0x5854), 0x00 },
	{ CCI_REG8(0x58cb), 0x03 },
	{ CCI_REG8(0x5bd0), 0x15 },
	{ CCI_REG8(0x5bd1), 0x02 },
	{ CCI_REG8(0x5c0e), 0x11 },
	{ CCI_REG8(0x5c11), 0x00 },
	{ CCI_REG8(0x5c16), 0x02 },
	{ CCI_REG8(0x5c17), 0x01 },
	{ CCI_REG8(0x5c1a), 0x04 },
	{ CCI_REG8(0x5c1b), 0x03 },
	{ CCI_REG8(0x5c21), 0x10 },
	{ CCI_REG8(0x5c22), 0x10 },
	{ CCI_REG8(0x5c23), 0x04 },
	{ CCI_REG8(0x5c24), 0x0c },
	{ CCI_REG8(0x5c25), 0x04 },
	{ CCI_REG8(0x5c26), 0x0c },
	{ CCI_REG8(0x5c27), 0x04 },
	{ CCI_REG8(0x5c28), 0x0c },
	{ CCI_REG8(0x5c29), 0x04 },
	{ CCI_REG8(0x5c2a), 0x0c },
	{ CCI_REG8(0x5c2b), 0x01 },
	{ CCI_REG8(0x5c2c), 0x01 },
	{ CCI_REG8(0x5c2e), 0x08 },
	{ CCI_REG8(0x5c30), 0x04 },
	{ CCI_REG8(0x5c35), 0x03 },
	{ CCI_REG8(0x5c36), 0x03 },
	{ CCI_REG8(0x5c37), 0x03 },
	{ CCI_REG8(0x5c38), 0x03 },
	{ CCI_REG8(0x5d00), 0xff },
	{ CCI_REG8(0x5d01), 0x0f },
	{ CCI_REG8(0x5d02), 0x80 },
	{ CCI_REG8(0x5d03), 0x44 },
	{ CCI_REG8(0x5d05), 0xfc },
	{ CCI_REG8(0x5d06), 0x0b },
	{ CCI_REG8(0x5d08), 0x10 },
	{ CCI_REG8(0x5d09), 0x10 },
	{ CCI_REG8(0x5d0a), 0x04 },
	{ CCI_REG8(0x5d0b), 0x0c },
	{ CCI_REG8(0x5d0c), 0x04 },
	{ CCI_REG8(0x5d0d), 0x0c },
	{ CCI_REG8(0x5d0e), 0x04 },
	{ CCI_REG8(0x5d0f), 0x0c },
	{ CCI_REG8(0x5d10), 0x04 },
	{ CCI_REG8(0x5d11), 0x0c },
	{ CCI_REG8(0x5d12), 0x01 },
	{ CCI_REG8(0x5d13), 0x01 },
	{ CCI_REG8(0x5d15), 0x10 },
	{ CCI_REG8(0x5d16), 0x10 },
	{ CCI_REG8(0x5d17), 0x10 },
	{ CCI_REG8(0x5d18), 0x10 },
	{ CCI_REG8(0x5d1a), 0x10 },
	{ CCI_REG8(0x5d1b), 0x10 },
	{ CCI_REG8(0x5d1c), 0x10 },
	{ CCI_REG8(0x5d1d), 0x10 },
	{ CCI_REG8(0x5d1e), 0x04 },
	{ CCI_REG8(0x5d1f), 0x04 },
	{ CCI_REG8(0x5d20), 0x04 },
	{ CCI_REG8(0x5d27), 0x64 },
	{ CCI_REG8(0x5d28), 0xc8 },
	{ CCI_REG8(0x5d29), 0x96 },
	{ CCI_REG8(0x5d2a), 0xff },
	{ CCI_REG8(0x5d2b), 0xc8 },
	{ CCI_REG8(0x5d2c), 0xff },
	{ CCI_REG8(0x5d2d), 0x04 },
	{ CCI_REG8(0x5d34), 0x00 },
	{ CCI_REG8(0x5d35), 0x08 },
	{ CCI_REG8(0x5d36), 0x00 },
	{ CCI_REG8(0x5d37), 0x04 },
	{ CCI_REG8(0x5d4a), 0x00 },
	{ CCI_REG8(0x5d4c), 0x00 },
	{ CCI_REG8(0x0305), 0x73 },
	{ CCI_REG8(0x0307), 0x00 },
	{ CCI_REG8(0x4837), 0x0a },
	{ CCI_REG8(0x4202), 0x01 },
};

static const struct cci_reg_sequence ov16a1q_2304x1728_regs[] = {
	{ CCI_REG8(0x0307), 0x01 },
	{ CCI_REG8(0x4837), 0x15 },
	{ CCI_REG8(0x0329), 0x01 },
	{ CCI_REG8(0x0344), 0x01 },
	{ CCI_REG8(0x0345), 0x2c },
	{ CCI_REG8(0x034a), 0x07 },
	{ CCI_REG8(0x3608), 0x75 },
	{ CCI_REG8(0x360a), 0x69 },
	{ CCI_REG8(0x361a), 0x8b },
	{ CCI_REG8(0x361e), 0x30 },
	{ CCI_REG8(0x3639), 0xa6 },
	{ CCI_REG8(0x363a), 0xaa },
	{ CCI_REG8(0x3642), 0xa8 },
	{ CCI_REG8(0x3654), 0x8a },
	{ CCI_REG8(0x3656), 0x0c },
	{ CCI_REG8(0x3663), 0x00 },
	{ CCI_REG8(0x370e), 0xa5 },
	{ CCI_REG8(0x3712), 0x59 },
	{ CCI_REG8(0x3713), 0x40 },
	{ CCI_REG8(0x3714), 0xe4 },
	{ CCI_REG8(0x37d0), 0x02 },
	{ CCI_REG8(0x37d1), 0x10 },
	{ CCI_REG8(0x37db), 0x04 },
	{ CCI_REG8(0x3808), 0x09 },
	{ CCI_REG8(0x3809), 0x00 },
	{ CCI_REG8(0x380a), 0x06 },
	{ CCI_REG8(0x380b), 0xc0 },
	{ CCI_REG8(0x380c), 0x03 },
	{ CCI_REG8(0x380d), 0x52 },
	{ CCI_REG8(0x380e), 0x0f },
	{ CCI_REG8(0x380f), 0x50 },
	{ CCI_REG8(0x3814), 0x11 },
	{ CCI_REG8(0x3815), 0x22 },
	{ CCI_REG8(0x3820), 0x01 },
	{ CCI_REG8(0x3821), 0x06 },
	{ CCI_REG8(0x3822), 0x01 },
	{ CCI_REG8(0x383c), 0x22 },
	{ CCI_REG8(0x383f), 0x33 },
	{ CCI_REG8(0x4015), 0x02 },
	{ CCI_REG8(0x4016), 0x0d },
	{ CCI_REG8(0x4017), 0x00 },
	{ CCI_REG8(0x4018), 0x07 },
	{ CCI_REG8(0x401b), 0x1f },
	{ CCI_REG8(0x401f), 0x38 },
	{ CCI_REG8(0x4500), 0x20 },
	{ CCI_REG8(0x4501), 0x6f },
	{ CCI_REG8(0x4502), 0xe4 },
	{ CCI_REG8(0x4e05), 0x04 },
	{ CCI_REG8(0x4e11), 0x06 },
	{ CCI_REG8(0x4e1d), 0x30 },
	{ CCI_REG8(0x4e26), 0x39 },
	{ CCI_REG8(0x4e29), 0x6d },
	{ CCI_REG8(0x5000), 0x29 },
	{ CCI_REG8(0x5001), 0xc2 },
	{ CCI_REG8(0x5003), 0xc2 },
	{ CCI_REG8(0x5820), 0xc0 },
	{ CCI_REG8(0x5854), 0x01 },
	{ CCI_REG8(0x5bd0), 0x19 },
	{ CCI_REG8(0x5c0e), 0x11 },
	{ CCI_REG8(0x5c11), 0x01 },
	{ CCI_REG8(0x5c16), 0x02 },
	{ CCI_REG8(0x5c17), 0x00 },
	{ CCI_REG8(0x5c1a), 0x00 },
	{ CCI_REG8(0x5c1b), 0x00 },
	{ CCI_REG8(0x5c21), 0x10 },
	{ CCI_REG8(0x5c22), 0x08 },
	{ CCI_REG8(0x5c23), 0x04 },
	{ CCI_REG8(0x5c24), 0x0c },
	{ CCI_REG8(0x5c25), 0x04 },
	{ CCI_REG8(0x5c26), 0x0c },
	{ CCI_REG8(0x5c27), 0x02 },
	{ CCI_REG8(0x5c28), 0x06 },
	{ CCI_REG8(0x5c29), 0x02 },
	{ CCI_REG8(0x5c2a), 0x06 },
	{ CCI_REG8(0x5c2b), 0x01 },
	{ CCI_REG8(0x5c2c), 0x00 },
	{ CCI_REG8(0x5d01), 0x07 },
	{ CCI_REG8(0x5d08), 0x08 },
	{ CCI_REG8(0x5d09), 0x08 },
	{ CCI_REG8(0x5d0a), 0x02 },
	{ CCI_REG8(0x5d0b), 0x06 },
	{ CCI_REG8(0x5d0c), 0x02 },
	{ CCI_REG8(0x5d0d), 0x06 },
	{ CCI_REG8(0x5d0e), 0x02 },
	{ CCI_REG8(0x5d0f), 0x06 },
	{ CCI_REG8(0x5d10), 0x02 },
	{ CCI_REG8(0x5d11), 0x06 },
	{ CCI_REG8(0x5d12), 0x00 },
	{ CCI_REG8(0x5d13), 0x00 },
	{ CCI_REG8(0x3500), 0x00 },
	{ CCI_REG8(0x3501), 0x0f },
	{ CCI_REG8(0x3502), 0x48 },
	{ CCI_REG8(0x3508), 0x01 },
	{ CCI_REG8(0x3509), 0x00 },
};

static const struct ov16a1q_mode ov16a1q_modes[] = {
	{
		.width = 2304,
		.height = 1728,
		.vts = 3920,
		.regs = ov16a1q_2304x1728_regs,
		.num_regs = ARRAY_SIZE(ov16a1q_2304x1728_regs),
	},
};

static const s64 ov16a1q_link_freqs[] = {
	OV16A1Q_LINK_FREQ,
};

static const char * const ov16a1q_test_pattern_menu[] = {
	"Disabled",
	"Color Bars",
	"Color Bars (vendor)",
	"Random",
	"Squares",
	"Black",
};

/* 0x5081: bit 0 enable, bits 5:4 pattern; the vendor driver uses 0x09 */
static const u8 ov16a1q_test_pattern_val[] = {
	0x00, 0x01, 0x09, 0x11, 0x21, 0x31,
};

struct ov16a1q {
	struct device *dev;
	struct regmap *regmap;
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *vblank;
	struct v4l2_ctrl *exposure;

	struct clk *mclk;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_default;
	struct pinctrl_state *pins_mclk;
	struct gpio_desc *reset_gpio;
	struct regulator_bulk_data supplies[ARRAY_SIZE(ov16a1q_supply_names)];

	const struct ov16a1q_mode *mode;
};

static inline struct ov16a1q *to_ov16a1q(struct v4l2_subdev *sd)
{
	return container_of(sd, struct ov16a1q, sd);
}

/*
 * Vendor power-up: reset low, MCLK on, 2 ms, DOVDD,
 * AVDD, DVDD, reset high, 5 ms. Power-down in reverse with the same waits.
 */
static int ov16a1q_power_on(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct ov16a1q *ov = to_ov16a1q(sd);
	unsigned int i;
	int ret;

	gpiod_set_value_cansleep(ov->reset_gpio, 1);

	if (ov->pins_mclk) {
		ret = pinctrl_select_state(ov->pinctrl, ov->pins_mclk);
		if (ret)
			return ret;
	}
	ret = clk_prepare_enable(ov->mclk);
	if (ret)
		goto err_pins;
	fsleep(2000);

	for (i = 0; i < ARRAY_SIZE(ov16a1q_supply_names); i++) {
		ret = regulator_enable(ov->supplies[i].consumer);
		if (ret)
			goto err_supplies;
	}

	gpiod_set_value_cansleep(ov->reset_gpio, 0);
	fsleep(5000);

	return 0;

err_supplies:
	while (i--)
		regulator_disable(ov->supplies[i].consumer);
	clk_disable_unprepare(ov->mclk);
err_pins:
	if (ov->pins_default)
		pinctrl_select_state(ov->pinctrl, ov->pins_default);
	return ret;
}

static int ov16a1q_power_off(struct device *dev)
{
	struct v4l2_subdev *sd = dev_get_drvdata(dev);
	struct ov16a1q *ov = to_ov16a1q(sd);
	unsigned int i = ARRAY_SIZE(ov16a1q_supply_names);

	gpiod_set_value_cansleep(ov->reset_gpio, 1);
	fsleep(5000);
	while (i--)
		regulator_disable(ov->supplies[i].consumer);
	fsleep(2000);
	clk_disable_unprepare(ov->mclk);
	if (ov->pins_default)
		pinctrl_select_state(ov->pinctrl, ov->pins_default);

	return 0;
}

static int ov16a1q_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct ov16a1q *ov = container_of(ctrl->handler, struct ov16a1q,
					  ctrl_handler);
	struct v4l2_subdev_state *state;
	const struct v4l2_mbus_framefmt *fmt;
	int ret = 0;

	state = v4l2_subdev_get_locked_active_state(&ov->sd);
	fmt = v4l2_subdev_state_get_format(state, 0);

	if (ctrl->id == V4L2_CID_VBLANK) {
		s64 max = fmt->height + ctrl->val - OV16A1Q_EXPOSURE_MARGIN;

		ret = __v4l2_ctrl_modify_range(ov->exposure,
					       OV16A1Q_EXPOSURE_MIN, max, 1,
					       min(ov->exposure->default_value,
						   max));
		if (ret)
			return ret;
	}

	if (!pm_runtime_get_if_in_use(ov->dev))
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_EXPOSURE:
		/* whole lines; the vendor driver keeps the LSB clear */
		cci_write(ov->regmap, OV16A1Q_REG_EXPOSURE, ctrl->val & ~1, &ret);
		break;
	case V4L2_CID_ANALOGUE_GAIN:
		cci_write(ov->regmap, OV16A1Q_REG_ANALOGUE_GAIN, ctrl->val, &ret);
		break;
	case V4L2_CID_VBLANK:
		cci_write(ov->regmap, OV16A1Q_REG_VTS,
			  (fmt->height + ctrl->val) & OV16A1Q_VTS_MAX, &ret);
		break;
	case V4L2_CID_TEST_PATTERN:
		cci_write(ov->regmap, OV16A1Q_REG_TEST_PATTERN,
			  ov16a1q_test_pattern_val[ctrl->val], &ret);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	pm_runtime_put(ov->dev);

	return ret;
}

static const struct v4l2_ctrl_ops ov16a1q_ctrl_ops = {
	.s_ctrl = ov16a1q_set_ctrl,
};

static int ov16a1q_init_controls(struct ov16a1q *ov)
{
	const struct ov16a1q_mode *mode = &ov16a1q_modes[0];
	struct v4l2_ctrl_handler *hdl = &ov->ctrl_handler;
	struct v4l2_fwnode_device_properties props;
	struct v4l2_ctrl *ctrl;
	s64 vblank_def, hblank;
	int ret;

	v4l2_ctrl_handler_init(hdl, 10);

	v4l2_ctrl_new_std(hdl, NULL, V4L2_CID_PIXEL_RATE, OV16A1Q_PIXEL_RATE,
			  OV16A1Q_PIXEL_RATE, 1, OV16A1Q_PIXEL_RATE);
	ctrl = v4l2_ctrl_new_int_menu(hdl, NULL, V4L2_CID_LINK_FREQ, 0, 0,
				      ov16a1q_link_freqs);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	hblank = OV16A1Q_LINE_LENGTH - mode->width;
	ctrl = v4l2_ctrl_new_std(hdl, NULL, V4L2_CID_HBLANK, hblank, hblank, 1,
				 hblank);
	if (ctrl)
		ctrl->flags |= V4L2_CTRL_FLAG_READ_ONLY;

	vblank_def = mode->vts - mode->height;
	ov->vblank = v4l2_ctrl_new_std(hdl, &ov16a1q_ctrl_ops, V4L2_CID_VBLANK,
				       vblank_def, OV16A1Q_VTS_MAX - mode->height,
				       2, vblank_def);
	ov->exposure = v4l2_ctrl_new_std(hdl, &ov16a1q_ctrl_ops,
					 V4L2_CID_EXPOSURE, OV16A1Q_EXPOSURE_MIN,
					 mode->vts - OV16A1Q_EXPOSURE_MARGIN, 1,
					 mode->vts - OV16A1Q_EXPOSURE_MARGIN);
	v4l2_ctrl_new_std(hdl, &ov16a1q_ctrl_ops, V4L2_CID_ANALOGUE_GAIN,
			  OV16A1Q_GAIN_MIN, OV16A1Q_GAIN_MAX, 1, OV16A1Q_GAIN_MIN);
	v4l2_ctrl_new_std_menu_items(hdl, &ov16a1q_ctrl_ops,
				     V4L2_CID_TEST_PATTERN,
				     ARRAY_SIZE(ov16a1q_test_pattern_menu) - 1,
				     0, 0, ov16a1q_test_pattern_menu);

	if (hdl->error) {
		ret = hdl->error;
		goto err_free;
	}

	ret = v4l2_fwnode_device_parse(ov->dev, &props);
	if (ret)
		goto err_free;
	ret = v4l2_ctrl_new_fwnode_properties(hdl, &ov16a1q_ctrl_ops, &props);
	if (ret)
		goto err_free;

	ov->sd.ctrl_handler = hdl;

	return 0;

err_free:
	v4l2_ctrl_handler_free(hdl);
	return dev_err_probe(ov->dev, ret, "control init failed\n");
}

static void ov16a1q_fill_fmt(const struct ov16a1q_mode *mode,
			     struct v4l2_mbus_framefmt *fmt)
{
	fmt->code = MEDIA_BUS_FMT_SBGGR10_1X10;
	fmt->width = mode->width;
	fmt->height = mode->height;
	fmt->field = V4L2_FIELD_NONE;
	fmt->colorspace = V4L2_COLORSPACE_RAW;
	fmt->ycbcr_enc = V4L2_YCBCR_ENC_601;
	fmt->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	fmt->xfer_func = V4L2_XFER_FUNC_NONE;
}

static int ov16a1q_enum_mbus_code(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index)
		return -EINVAL;

	code->code = MEDIA_BUS_FMT_SBGGR10_1X10;

	return 0;
}

static int ov16a1q_enum_frame_size(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *state,
				   struct v4l2_subdev_frame_size_enum *fse)
{
	if (fse->index >= ARRAY_SIZE(ov16a1q_modes) ||
	    fse->code != MEDIA_BUS_FMT_SBGGR10_1X10)
		return -EINVAL;

	fse->min_width = ov16a1q_modes[fse->index].width;
	fse->max_width = fse->min_width;
	fse->min_height = ov16a1q_modes[fse->index].height;
	fse->max_height = fse->min_height;

	return 0;
}

static int ov16a1q_set_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_state *state,
			   struct v4l2_subdev_format *fmt)
{
	struct ov16a1q *ov = to_ov16a1q(sd);
	const struct ov16a1q_mode *mode;
	s64 vblank_def;
	int ret;

	mode = v4l2_find_nearest_size(ov16a1q_modes, ARRAY_SIZE(ov16a1q_modes),
				      width, height, fmt->format.width,
				      fmt->format.height);
	ov16a1q_fill_fmt(mode, &fmt->format);
	*v4l2_subdev_state_get_format(state, 0) = fmt->format;

	if (fmt->which == V4L2_SUBDEV_FORMAT_TRY)
		return 0;

	ov->mode = mode;

	/*
	 * Start a new mode at its own frame length: the VBLANK handler then
	 * also moves the exposure limit. A frame length left over from the
	 * previous user would otherwise stay in effect.
	 */
	vblank_def = mode->vts - mode->height;
	ret = __v4l2_ctrl_modify_range(ov->vblank, vblank_def,
				       OV16A1Q_VTS_MAX - mode->height, 2,
				       vblank_def);
	if (ret)
		return ret;

	return __v4l2_ctrl_s_ctrl(ov->vblank, vblank_def);
}

static int ov16a1q_get_selection(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_selection *sel)
{
	switch (sel->target) {
	case V4L2_SEL_TGT_NATIVE_SIZE:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		sel->r.left = 0;
		sel->r.top = 0;
		sel->r.width = OV16A1Q_NATIVE_WIDTH;
		sel->r.height = OV16A1Q_NATIVE_HEIGHT;
		return 0;
	case V4L2_SEL_TGT_CROP:
	case V4L2_SEL_TGT_CROP_DEFAULT:
		/* 2328x1748 binned output, 12/10 pixels cropped on each side */
		sel->r.left = 24;
		sel->r.top = 20;
		sel->r.width = OV16A1Q_NATIVE_WIDTH - 48;
		sel->r.height = OV16A1Q_NATIVE_HEIGHT - 40;
		return 0;
	}

	return -EINVAL;
}

static int ov16a1q_enable_streams(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state, u32 pad,
				  u64 streams_mask)
{
	struct ov16a1q *ov = to_ov16a1q(sd);
	int ret;

	ret = pm_runtime_resume_and_get(ov->dev);
	if (ret < 0)
		return ret;

	/* vendor open(): software reset, 5 ms, init sequence, 10 ms, mode */
	cci_write(ov->regmap, OV16A1Q_REG_SOFTWARE_RESET, 0x01, &ret);
	if (ret)
		goto err_put;
	fsleep(5000);
	ret = cci_multi_reg_write(ov->regmap, ov16a1q_init_regs,
				  ARRAY_SIZE(ov16a1q_init_regs), NULL);
	if (ret)
		goto err_put;
	fsleep(10000);
	ret = cci_multi_reg_write(ov->regmap, ov->mode->regs,
				  ov->mode->num_regs, NULL);
	if (ret)
		goto err_put;

	ret = __v4l2_ctrl_handler_setup(ov->sd.ctrl_handler);
	if (ret)
		goto err_put;

	/* vendor streaming_control(): 5 ms before and after */
	fsleep(5000);
	cci_write(ov->regmap, OV16A1Q_REG_MODE_SELECT, OV16A1Q_MODE_STREAMING,
		  &ret);
	if (ret)
		goto err_put;
	fsleep(5000);

	return 0;

err_put:
	dev_err(ov->dev, "stream on failed: %d\n", ret);
	pm_runtime_put(ov->dev);
	return ret;
}

static int ov16a1q_disable_streams(struct v4l2_subdev *sd,
				   struct v4l2_subdev_state *state, u32 pad,
				   u64 streams_mask)
{
	struct ov16a1q *ov = to_ov16a1q(sd);
	int ret;

	ret = cci_write(ov->regmap, OV16A1Q_REG_MODE_SELECT,
			OV16A1Q_MODE_STANDBY, NULL);
	pm_runtime_put(ov->dev);

	return ret;
}

static int ov16a1q_init_state(struct v4l2_subdev *sd,
			      struct v4l2_subdev_state *state)
{
	struct v4l2_subdev_format fmt = {
		.which = V4L2_SUBDEV_FORMAT_TRY,
		.pad = 0,
		.format = {
			.width = ov16a1q_modes[0].width,
			.height = ov16a1q_modes[0].height,
		},
	};

	ov16a1q_fill_fmt(&ov16a1q_modes[0], &fmt.format);
	*v4l2_subdev_state_get_format(state, 0) = fmt.format;

	return 0;
}

static const struct v4l2_subdev_video_ops ov16a1q_video_ops = {
	.s_stream = v4l2_subdev_s_stream_helper,
};

static const struct v4l2_subdev_pad_ops ov16a1q_pad_ops = {
	.enum_mbus_code = ov16a1q_enum_mbus_code,
	.enum_frame_size = ov16a1q_enum_frame_size,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = ov16a1q_set_fmt,
	.get_selection = ov16a1q_get_selection,
	.enable_streams = ov16a1q_enable_streams,
	.disable_streams = ov16a1q_disable_streams,
};

static const struct v4l2_subdev_ops ov16a1q_subdev_ops = {
	.video = &ov16a1q_video_ops,
	.pad = &ov16a1q_pad_ops,
};

static const struct v4l2_subdev_internal_ops ov16a1q_internal_ops = {
	.init_state = ov16a1q_init_state,
};

static int ov16a1q_check_endpoint(struct ov16a1q *ov)
{
	struct v4l2_fwnode_endpoint bus_cfg = {
		.bus_type = V4L2_MBUS_CSI2_DPHY,
	};
	struct fwnode_handle *ep;
	unsigned int i;
	int ret;

	ep = fwnode_graph_get_next_endpoint(dev_fwnode(ov->dev), NULL);
	if (!ep)
		return dev_err_probe(ov->dev, -ENXIO, "no endpoint\n");

	ret = v4l2_fwnode_endpoint_alloc_parse(ep, &bus_cfg);
	fwnode_handle_put(ep);
	if (ret)
		return dev_err_probe(ov->dev, ret, "endpoint parse failed\n");

	if (bus_cfg.bus.mipi_csi2.num_data_lanes != OV16A1Q_LANES) {
		ret = dev_err_probe(ov->dev, -EINVAL, "only 4 data lanes\n");
		goto out;
	}
	for (i = 0; i < bus_cfg.nr_of_link_frequencies; i++)
		if (bus_cfg.link_frequencies[i] == OV16A1Q_LINK_FREQ)
			break;
	if (i == bus_cfg.nr_of_link_frequencies)
		ret = dev_err_probe(ov->dev, -EINVAL,
				    "link frequency %llu not in DT\n",
				    OV16A1Q_LINK_FREQ);
out:
	v4l2_fwnode_endpoint_free(&bus_cfg);
	return ret;
}

static int ov16a1q_probe(struct i2c_client *client)
{
	struct device *dev = &client->dev;
	struct ov16a1q *ov;
	unsigned int i;
	u64 id;
	int ret;

	ov = devm_kzalloc(dev, sizeof(*ov), GFP_KERNEL);
	if (!ov)
		return -ENOMEM;
	ov->dev = dev;
	ov->mode = &ov16a1q_modes[0];

	ret = ov16a1q_check_endpoint(ov);
	if (ret)
		return ret;

	ov->regmap = devm_cci_regmap_init_i2c(client, 16);
	if (IS_ERR(ov->regmap))
		return dev_err_probe(dev, PTR_ERR(ov->regmap), "CCI init failed\n");

	ov->mclk = devm_v4l2_sensor_clk_get(dev, NULL);
	if (IS_ERR(ov->mclk))
		return dev_err_probe(dev, PTR_ERR(ov->mclk), "no mclk\n");
	if (clk_get_rate(ov->mclk) != OV16A1Q_MCLK_FREQ)
		return dev_err_probe(dev, -EINVAL, "mclk %lu Hz, need 24 MHz\n",
				     clk_get_rate(ov->mclk));

	/* optional: MCLK pin in GPIO mode while off ("default"), CMMCLK while on */
	ov->pinctrl = devm_pinctrl_get(dev);
	if (!IS_ERR(ov->pinctrl)) {
		ov->pins_mclk = pinctrl_lookup_state(ov->pinctrl, "mclk");
		ov->pins_default = pinctrl_lookup_state(ov->pinctrl,
							PINCTRL_STATE_DEFAULT);
		if (IS_ERR(ov->pins_mclk) || IS_ERR(ov->pins_default)) {
			ov->pins_mclk = NULL;
			ov->pins_default = NULL;
		}
	}

	ov->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(ov->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ov->reset_gpio), "no reset gpio\n");

	for (i = 0; i < ARRAY_SIZE(ov16a1q_supply_names); i++)
		ov->supplies[i].supply = ov16a1q_supply_names[i];
	ret = devm_regulator_bulk_get(dev, ARRAY_SIZE(ov16a1q_supply_names),
				      ov->supplies);
	if (ret)
		return dev_err_probe(dev, ret, "supplies\n");
	/* DOVDD is MT6359 VCAMIO, settable 1.7..1.9 V: the vendor sets 1.8 V */
	ret = regulator_set_voltage(ov->supplies[0].consumer, 1800000, 1800000);
	if (ret)
		return dev_err_probe(dev, ret, "DOVDD not settable to 1.8 V\n");

	v4l2_i2c_subdev_init(&ov->sd, client, &ov16a1q_subdev_ops);

	ret = ov16a1q_power_on(dev);
	if (ret)
		return dev_err_probe(dev, ret, "power on failed\n");

	ret = cci_read(ov->regmap, OV16A1Q_REG_CHIP_ID, &id, NULL);
	if (ret || id != OV16A1Q_CHIP_ID) {
		ret = dev_err_probe(dev, ret ?: -ENODEV, "chip id 0x%04llx\n", id);
		goto err_power_off;
	}

	ret = ov16a1q_init_controls(ov);
	if (ret)
		goto err_power_off;

	ov->sd.internal_ops = &ov16a1q_internal_ops;
	ov->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	ov->sd.entity.function = MEDIA_ENT_F_CAM_SENSOR;
	ov->pad.flags = MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&ov->sd.entity, 1, &ov->pad);
	if (ret)
		goto err_ctrls;

	ov->sd.state_lock = ov->ctrl_handler.lock;
	ret = v4l2_subdev_init_finalize(&ov->sd);
	if (ret)
		goto err_entity;

	pm_runtime_set_active(dev);
	pm_runtime_enable(dev);

	ret = v4l2_async_register_subdev_sensor(&ov->sd);
	if (ret)
		goto err_pm;

	pm_runtime_set_autosuspend_delay(dev, 1000);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_idle(dev);

	dev_info(dev, "OV16A1Q 0x%04llx\n", id);

	return 0;

err_pm:
	pm_runtime_disable(dev);
	pm_runtime_set_suspended(dev);
	v4l2_subdev_cleanup(&ov->sd);
err_entity:
	media_entity_cleanup(&ov->sd.entity);
err_ctrls:
	v4l2_ctrl_handler_free(&ov->ctrl_handler);
err_power_off:
	ov16a1q_power_off(dev);
	return ret;
}

static void ov16a1q_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ov16a1q *ov = to_ov16a1q(sd);

	v4l2_async_unregister_subdev(sd);
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&sd->entity);
	v4l2_ctrl_handler_free(&ov->ctrl_handler);

	pm_runtime_disable(ov->dev);
	if (!pm_runtime_status_suspended(ov->dev))
		ov16a1q_power_off(ov->dev);
	pm_runtime_set_suspended(ov->dev);
}

static DEFINE_RUNTIME_DEV_PM_OPS(ov16a1q_pm_ops, ov16a1q_power_off,
				 ov16a1q_power_on, NULL);

static const struct of_device_id ov16a1q_of_match[] = {
	{ .compatible = "ovti,ov16a1q" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, ov16a1q_of_match);

static struct i2c_driver ov16a1q_driver = {
	.driver = {
		.name = "ov16a1q",
		.of_match_table = ov16a1q_of_match,
		.pm = pm_ptr(&ov16a1q_pm_ops),
	},
	.probe = ov16a1q_probe,
	.remove = ov16a1q_remove,
};
module_i2c_driver(ov16a1q_driver);

MODULE_DESCRIPTION("OmniVision OV16A1Q image sensor driver");
MODULE_LICENSE("GPL");
