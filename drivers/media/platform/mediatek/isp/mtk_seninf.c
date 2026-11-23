// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2022 MediaTek Inc.
 */
#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/debugfs.h>
#include <linux/seq_file.h>
#include <linux/property.h>
#include <linux/videodev2.h>
#include <media/media-device.h>
#include <media/media-entity.h>
#include <media/mipi-csi2.h>
#include <media/v4l2-async.h>
#include <media/v4l2-common.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-event.h>
#include <media/v4l2-fwnode.h>
#include <media/v4l2-mc.h>
#include <media/v4l2-subdev.h>

#include "mtk_seninf.h"
#include "mtk_seninf_reg.h"
#include "mtk_seninf_mt6785_reg.h"

#define SENINF_TIMESTAMP_STEP		0x67
#define SENINF_SETTLE_DELAY		0x15
#define SENINF_HS_TRAIL_PARAMETER	0x8

#define SENINF_MAX_NUM_INPUTS		4
#define SENINF_MAX_NUM_OUTPUTS		6
#define SENINF_MAX_NUM_MUXES		8
#define SENINF_MAX_NUM_PADS		(SENINF_MAX_NUM_INPUTS + \
					 SENINF_MAX_NUM_OUTPUTS)

#define SENINF_DEFAULT_BUS_FMT		MEDIA_BUS_FMT_SGRBG10_1X10
#define SENINF_DEFAULT_WIDTH		1920
#define SENINF_DEFAULT_HEIGHT		1080

#define SENINF_PAD_10BIT		0

#define SENINF_TEST_MODEL		0
#define SENINF_NORMAL_MODEL		1
#define SENINF_ALL_ERR_IRQ_EN		0x7f
#define SENINF_IRQ_CLR_SEL		0x80000000

#define SENINF_MIPI_SENSOR		0x8

#define MTK_CSI_MAX_LANES 4

/* Port number in the device tree. */
enum mtk_seninf_port {
	CSI_PORT_0 = 0,	/* 4D1C or 2D1C */
	CSI_PORT_1,	/* 4D1C */
	CSI_PORT_2,	/* 4D1C */
	CSI_PORT_0B,	/* 2D1C */
};

enum mtk_seninf_id {
	SENINF_1 = 0,
	SENINF_2 = 1,
	SENINF_3 = 2,
	SENINF_5 = 4,
};

static const u32 port_to_seninf_id[] = {
	[CSI_PORT_0] = SENINF_1,
	[CSI_PORT_1] = SENINF_3,
	[CSI_PORT_2] = SENINF_5,
	[CSI_PORT_0B] = SENINF_2,
};

enum mtk_seninf_phy_mode {
	SENINF_PHY_MODE_NONE,
	SENINF_PHY_MODE_4D1C,
	SENINF_PHY_MODE_2D1C,
};

enum mtk_seninf_format_flag {
	MTK_SENINF_FORMAT_BAYER = BIT(0),
	MTK_SENINF_FORMAT_DPCM = BIT(1),
	MTK_SENINF_FORMAT_JPEG = BIT(2),
	MTK_SENINF_FORMAT_INPUT_ONLY = BIT(3),
};

/**
 * struct mtk_seninf_conf - Model-specific SENINF parameters
 * @model: Model description
 * @nb_inputs: Number of SENINF inputs
 * @nb_muxes: Number of SENINF MUX (FIFO) instances
 * @nb_outputs: Number of outputs (to CAM and CAMSV instances)
 * @default_routes: Source pad of each sink pad in the default routing, 0
 *	for no route, the same whatever is connected; without the table the
 *	sink pads are routed 1:1 in order, the connected ones first
 */
struct mtk_seninf_conf {
	const char *model;
	u8 nb_inputs;
	u8 nb_muxes;
	u8 nb_outputs;
	const u8 *default_routes;
	bool two_pixel_mode;	/* MUX PIX_SEL 2-pixel (MT8365) vs 1-pixel */
	bool set_ck_sel_2;	/* force PHY_SENINF_CTL_CSI0.CK_SEL_2 = 2 */
	bool cam_mux_8365;	/* CAM2 <- mux0, CAM3 <- mux1 hard-wired */
	bool init_top_ctrl;	/* TOP_CTRL resets to MUX_LP_MODE, pixel clocks off */
	bool csi_phy_2_0;	/* MT6785: newer register generation, see mtk_seninf_mt6785_reg.h */
};

/* MT8183 bring-up knobs */
static int lane_mux_override = -1;
module_param(lane_mux_override, int, 0644);
MODULE_PARM_DESC(lane_mux_override, "raw MIPI_RX_CON24 LN0..3 mux byte (bits 31:24), e.g. 0xc9 (-1 = from DT)");
static bool test_pattern;
module_param(test_pattern, bool, 0644);
MODULE_PARM_DESC(test_pattern, "feed the seninf test-pattern generator instead of the sensor (sensor still streams)");
static int lens_focus = 480;
module_param(lens_focus, int, 0644);
MODULE_PARM_DESC(lens_focus, "lens (VCM) position applied while a sensor with a lens streams, returned to 0 afterwards (-1 = leave the lens alone; lisbon ov64b40+dw9800v: 480 ~ 2 m..far; krane ov8856+dw9768: 330 ~ 0.4 m..far)");
/* MT8183: CAM2 feeds CAMSV1 and CAM3 feeds CAMSV2 (same as MT8365, found on HW). */
static int cam_mux_out = 2;
module_param(cam_mux_out, int, 0644);
MODULE_PARM_DESC(cam_mux_out, "MT8183: CAMx output fed by output pad i is CAM(cam_mux_out + i) (default 2: CAM2->CAMSV1, CAM3->CAMSV2); -1 = route the mux to ALL 8 CAMx outputs (single camera only)");
/* MT6785 debug parameters */
static int mt6785_pix_mode = -1;
module_param(mt6785_pix_mode, int, 0644);
MODULE_PARM_DESC(mt6785_pix_mode, "MT6785: MUX and CAM mux pixel mode, -1 = from the sensor pixel rate (default), 0 = 1 pixel, 1 = 2 pixels, 2 = 4 pixels per clock (the CAMSV follows)");

/**
 * struct mtk_seninf_format_info - Information about media bus formats
 * @code: V4L2 media bus code
 * @flags: Flags describing the format, as a combination of MTK_SENINF_FORMAT_*
 * @bpp: Bits per pixel
 */
struct mtk_seninf_format_info {
	u32 code;
	u32 uncompressed;
	u32 flags;
	u8 bpp;
};

/*
 * Sources (sensors) of one input, one endpoint each: a board switch in front
 * of the CSI-2 port connects one of them at a time.
 */
#define SENINF_MAX_NUM_SOURCES		2

/**
 * struct mtk_seninf_source - A source endpoint of a SENINF input
 * @bus: CSI-2 bus configuration from DT
 * @sd: Source subdev, NULL while it is not bound
 * @pad: Source pad of @sd linked to the input
 */
struct mtk_seninf_source {
	struct v4l2_mbus_config_mipi_csi2 bus;
	struct v4l2_subdev *sd;
	struct media_pad *pad;
};

/**
 * struct mtk_seninf_input - SENINF input block
 * @pad: DT port and media entity pad number
 * @seninf_id: SENINF hardware instance ID
 * @base: Memory mapped I/O based address
 * @seninf: Back pointer to the mtk_seninf
 * @phy: PHY connected to the input
 * @phy_mode: PHY operation mode (NONE when the input is not connected)
 * @sources: Source endpoints of the input, in the order of their endpoint IDs
 * @num_sources: Number of entries in @sources
 * @active: Source with the enabled media link, NULL for none. With one source
 *	its link is immutable; with several the first one is linked by default
 *	and user space moves the link (disable, then enable another).
 * @bus: CSI-2 bus configuration of the active source
 * @source_sd: Subdev of the active source, NULL while not bound
 * @source_pad: Source pad of the active source
 * @lens: Lens of the source, if any
 * @lens_active: The lens is held at its streaming position
 * @phy_on: MT6785: phy_power_on() succeeded for the running streams
 * @users: Streams of the input running, protected by the lock of the active
 *	state; the input and its source run while there is one
 *
 * @active, @bus, @source_sd and @source_pad change under the lock of the
 * active state.
 */
struct mtk_seninf_input {
	enum mtk_seninf_port pad;
	enum mtk_seninf_id seninf_id;
	void __iomem *base;
	struct mtk_seninf *seninf;

	struct phy *phy;
	enum mtk_seninf_phy_mode phy_mode;

	struct mtk_seninf_source sources[SENINF_MAX_NUM_SOURCES];
	unsigned int num_sources;
	struct mtk_seninf_source *active;

	struct v4l2_mbus_config_mipi_csi2 bus;

	struct v4l2_subdev *source_sd;
	struct media_pad *source_pad;

	struct v4l2_subdev *lens;
	bool lens_active;
	bool phy_on;
	unsigned int users;
};

/**
 * struct mtk_seninf_mux - SENINF MUX channel
 * @pad: DT port and media entity pad number
 * @mux_id: MUX hardware instance ID
 * @base: Memory mapped I/O based address
 * @seninf: Back pointer to the mtk_seninf
 * @cam_mux_en: MT6785: CAM mux outputs enabled for this MUX
 * @stream: MT6785: stream of the input this MUX carries
 */
struct mtk_seninf_mux {
	unsigned int pad;
	unsigned int mux_id;
	void __iomem *base;
	struct mtk_seninf *seninf;
	u32 cam_mux_en;
	u32 stream;
};

/**
 * struct mtk_seninf - Top-level SENINF device
 * @dev: The (platform) device
 * @phy: PHYs at the SENINF inputs
 * @num_clks: Number of clocks in the clks array
 * @clks: Clocks
 * @base: Memory mapped I/O base address
 * @media_dev: Media controller device
 * @v4l2_dev: V4L2 device
 * @subdev: V4L2 subdevice
 * @pads: Media entity pads
 * @notifier: V4L2 async notifier for source subdevs
 * @ctrl_handler: V4L2 controls handler
 * @inputs: Array of SENINF inputs
 * @muxes: Array of MUXes
 * @conf: Model-specific SENINF parameters
 * @streaming: Source pads with a running stream (BIT(pad)), protected by
 *	the lock of the active state
 * @sleeping: A system sleep is in progress (from .prepare to .complete).
 *	User space is frozen then: streams stop and start only from the
 *	system sleep callbacks of the CAMSVs.
 */
struct mtk_seninf {
	struct device *dev;
	struct phy *phy[5];
	unsigned int num_clks;
	struct clk_bulk_data *clks;
	void __iomem *base;

	struct media_device media_dev;
	struct v4l2_device v4l2_dev;
	struct v4l2_subdev subdev;
	struct media_pad pads[SENINF_MAX_NUM_PADS];
	struct v4l2_async_notifier notifier;
	struct v4l2_ctrl_handler ctrl_handler;

	struct mtk_seninf_input inputs[SENINF_MAX_NUM_INPUTS];
	struct mtk_seninf_mux muxes[SENINF_MAX_NUM_MUXES];

	const struct mtk_seninf_conf *conf;
	struct dentry *dbg;
	u32 streaming;
	bool sleeping;
};

inline struct mtk_seninf *sd_to_mtk_seninf(struct v4l2_subdev *sd)
{
	return container_of(sd, struct mtk_seninf, subdev);
}

static inline bool mtk_seninf_pad_is_sink(struct mtk_seninf *priv,
					  unsigned int pad)
{
	return pad < priv->conf->nb_inputs;
}

static inline bool mtk_seninf_pad_is_source(struct mtk_seninf *priv,
					    unsigned int pad)
{
	return !mtk_seninf_pad_is_sink(priv, pad);
}

/* -----------------------------------------------------------------------------
 * Formats
 */

static const struct mtk_seninf_format_info mtk_seninf_formats[] = {
	{
		.code = MEDIA_BUS_FMT_SBGGR8_1X8,
		.flags = MTK_SENINF_FORMAT_BAYER,
		.bpp = 8,
	}, {
		.code = MEDIA_BUS_FMT_SGBRG8_1X8,
		.flags = MTK_SENINF_FORMAT_BAYER,
		.bpp = 8,
	}, {
		.code = MEDIA_BUS_FMT_SGRBG8_1X8,
		.flags = MTK_SENINF_FORMAT_BAYER,
		.bpp = 8,
	}, {
		.code = MEDIA_BUS_FMT_SRGGB8_1X8,
		.flags = MTK_SENINF_FORMAT_BAYER,
		.bpp = 8,
	}, {
		.code = MEDIA_BUS_FMT_SGRBG10_1X10,
		.flags = MTK_SENINF_FORMAT_BAYER,
		.bpp = 10,
	}, {
		.code = MEDIA_BUS_FMT_SRGGB10_1X10,
		.flags = MTK_SENINF_FORMAT_BAYER,
		.bpp = 10,
	}, {
		.code = MEDIA_BUS_FMT_SBGGR10_1X10,
		.flags = MTK_SENINF_FORMAT_BAYER,
		.bpp = 10,
	}, {
		.code = MEDIA_BUS_FMT_SGBRG10_1X10,
		.flags = MTK_SENINF_FORMAT_BAYER,
		.bpp = 10,
	}, {
		.code = MEDIA_BUS_FMT_SBGGR12_1X12,
		.flags = MTK_SENINF_FORMAT_BAYER,
		.bpp = 12,
	}, {
		.code = MEDIA_BUS_FMT_SGBRG12_1X12,
		.flags = MTK_SENINF_FORMAT_BAYER,
		.bpp = 12,
	}, {
		.code = MEDIA_BUS_FMT_SGRBG12_1X12,
		.flags = MTK_SENINF_FORMAT_BAYER,
		.bpp = 12,
	}, {
		.code = MEDIA_BUS_FMT_SRGGB12_1X12,
		.flags = MTK_SENINF_FORMAT_BAYER,
		.bpp = 12,
	}, {
		.code = MEDIA_BUS_FMT_SBGGR14_1X14,
		.flags = MTK_SENINF_FORMAT_BAYER,
		.bpp = 14,
	}, {
		.code = MEDIA_BUS_FMT_SGBRG14_1X14,
		.flags = MTK_SENINF_FORMAT_BAYER,
		.bpp = 14,
	}, {
		.code = MEDIA_BUS_FMT_SGRBG14_1X14,
		.flags = MTK_SENINF_FORMAT_BAYER,
		.bpp = 14,
	}, {
		.code = MEDIA_BUS_FMT_SRGGB14_1X14,
		.flags = MTK_SENINF_FORMAT_BAYER,
		.bpp = 14,
	}, {
		.code = MEDIA_BUS_FMT_SBGGR16_1X16,
		.flags = MTK_SENINF_FORMAT_BAYER,
		.bpp = 16,
	}, {
		.code = MEDIA_BUS_FMT_SGBRG16_1X16,
		.flags = MTK_SENINF_FORMAT_BAYER,
		.bpp = 16,
	}, {
		.code = MEDIA_BUS_FMT_SGRBG16_1X16,
		.flags = MTK_SENINF_FORMAT_BAYER,
		.bpp = 16,
	}, {
		.code = MEDIA_BUS_FMT_SRGGB16_1X16,
		.flags = MTK_SENINF_FORMAT_BAYER,
		.bpp = 16,
	}, {
		/* phase detection pixels on a virtual channel of their own */
		.code = MEDIA_BUS_FMT_Y10_1X10,
		.bpp = 10,
	}, {
		.code = MEDIA_BUS_FMT_UYVY8_1X16,
		.bpp = 16,
	}, {
		.code = MEDIA_BUS_FMT_VYUY8_1X16,
		.bpp = 16,
	}, {
		.code = MEDIA_BUS_FMT_YUYV8_1X16,
		.bpp = 16,
	}, {
		.code = MEDIA_BUS_FMT_YVYU8_1X16,
		.bpp = 16,
	}, {
		.code = MEDIA_BUS_FMT_JPEG_1X8,
		.flags = MTK_SENINF_FORMAT_JPEG,
		.bpp = 8,
	}, {
		.code = MEDIA_BUS_FMT_S5C_UYVY_JPEG_1X8,
		.flags = MTK_SENINF_FORMAT_JPEG,
		.bpp = 8,
	},
	/* Keep the input-only formats last. */
	{
		.code = MEDIA_BUS_FMT_SGRBG10_DPCM8_1X8,
		.uncompressed = MEDIA_BUS_FMT_SGRBG10_1X10,
		.flags = MTK_SENINF_FORMAT_DPCM | MTK_SENINF_FORMAT_INPUT_ONLY,
		.bpp = 8,
	}, {
		.code = MEDIA_BUS_FMT_SRGGB10_DPCM8_1X8,
		.uncompressed = MEDIA_BUS_FMT_SRGGB10_1X10,
		.flags = MTK_SENINF_FORMAT_DPCM | MTK_SENINF_FORMAT_INPUT_ONLY,
		.bpp = 8,
	}, {
		.code = MEDIA_BUS_FMT_SBGGR10_DPCM8_1X8,
		.uncompressed = MEDIA_BUS_FMT_SBGGR10_1X10,
		.flags = MTK_SENINF_FORMAT_DPCM | MTK_SENINF_FORMAT_INPUT_ONLY,
		.bpp = 8,
	}, {
		.code = MEDIA_BUS_FMT_SGBRG10_DPCM8_1X8,
		.uncompressed = MEDIA_BUS_FMT_SGBRG10_1X10,
		.flags = MTK_SENINF_FORMAT_DPCM | MTK_SENINF_FORMAT_INPUT_ONLY,
		.bpp = 8,
	}
};

static const struct mtk_seninf_format_info *mtk_seninf_format_info(u32 code)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(mtk_seninf_formats); ++i) {
		if (mtk_seninf_formats[i].code == code)
			return &mtk_seninf_formats[i];
	}

	return NULL;
}

static u32 mtk_seninf_read(struct mtk_seninf *priv, u32 reg)
{
	return readl(priv->base + reg);
}

static void mtk_seninf_write(struct mtk_seninf *priv, u32 reg, u32 value)
{
	writel(value, priv->base + reg);
}

static void __mtk_seninf_update(struct mtk_seninf *priv, u32 reg,
				u32 mask, u32 value)
{
	u32 val = mtk_seninf_read(priv, reg);

	writel((val & ~mask) | (value & mask), priv->base + reg);
}

#define mtk_seninf_update(priv, reg, field, val)		\
	__mtk_seninf_update(priv, reg, reg##_##field,		\
			    FIELD_PREP(reg##_##field, val))

static u32 mtk_seninf_inuput_read(struct mtk_seninf_input *input, u32 reg)
{
	return readl(input->base + reg);
}

static void mtk_seninf_input_write(struct mtk_seninf_input *input, u32 reg,
				   u32 value)
{
	writel(value, input->base + reg);
}

static void __mtk_seninf_input_update(struct mtk_seninf_input *input, u32 reg,
				      u32 mask, u32 value)
{
	u32 val = mtk_seninf_inuput_read(input, reg);

	mtk_seninf_input_write(input, reg, (val & ~mask) | (value & mask));
}

#define mtk_seninf_input_update(input, reg, field, val)		\
	__mtk_seninf_input_update(input, reg, reg##_##field,	\
				  FIELD_PREP(reg##_##field, val))

static u32 mtk_seninf_mux_read(struct mtk_seninf_mux *mux, u32 reg)
{
	return readl(mux->base + reg);
}

static void mtk_seninf_mux_write(struct mtk_seninf_mux *mux, u32 reg,
				 u32 value)
{
	writel(value, mux->base + reg);
}

static void __mtk_seninf_mux_update(struct mtk_seninf_mux *mux, u32 reg,
				    u32 mask, u32 value)
{
	u32 val = mtk_seninf_mux_read(mux, reg);

	mtk_seninf_mux_write(mux, reg, (val & ~mask) | (value & mask));
}

#define mtk_seninf_mux_update(mux, reg, field, val)		\
	__mtk_seninf_mux_update(mux, reg, reg##_##field,	\
				FIELD_PREP(reg##_##field, val))

/* -----------------------------------------------------------------------------
 * Hardware Configuration
 *
 * The SENINF is the camera sensor interface. On the input side it contains
 * input channels (also named SENINF), each made of a CSI-2 receiver, an
 * interface for parallel sensors, and a test pattern generator. The inputs are
 * routed through a N:M crossbar switch (TOP MUX) to VC/DT filters with a FIFO
 * (MUX). The MUX are routed to another N:M crossbar switch (CAM MUX), whose
 * output is then connected to other IP cores.
 *
 *            +-------------------------------------------------------+
 *            | SENINF                                                |
 *            |                                                       |
 * +-------+  |   +----------+    TOP MUX                             |
 * |       |  |   |  SENINF  |      |\                        CAM MUX |
 * | D-PHY | ---> | CSI-2 RX | ---> | |      +------------+      |\   |
 * |       |  |   |   TPG    |   -> | | ---> | MUX (FIFO) | ---> | | ---> CAMSV
 * +-------+  |   +----------+   -> | |      +------------+   -> | |  |
 *            |                     |/                        -> | |  |
 *            |                                                  |/   |
 *            |                                                       |
 *    ...     |       ...                         ...                --->
 *            |                                                       |
 *            |                                                       |
 *            +-------------------------------------------------------+
 *
 * The number of PHYs, SENINF and MUX differ between SoCs. MT8167 has a single
 * MUX and thus no output CAM MUX crossbar switch.
 */

static void mtk_seninf_csi2_setup_phy(struct mtk_seninf *priv)
{
	/* CSI0 */
	if (priv->inputs[CSI_PORT_0].phy) {
		const struct mtk_seninf_input *input = &priv->inputs[CSI_PORT_0];

		mtk_seninf_update(priv, SENINF_TOP_PHY_SENINF_CTL_CSI0,
				  DPHY_MODE, 0 /* 4D1C */);
		mtk_seninf_update(priv, SENINF_TOP_PHY_SENINF_CTL_CSI0,
				  CK_SEL_1, input->bus.clock_lane);
		if (priv->conf->set_ck_sel_2)
			mtk_seninf_update(priv, SENINF_TOP_PHY_SENINF_CTL_CSI0,
					  CK_SEL_2, 2);
		mtk_seninf_update(priv, SENINF_TOP_PHY_SENINF_CTL_CSI0,
				  PHY_SENINF_LANE_MUX_CSI0_EN, 1);
	}

	/* CSI1 */
	if (priv->inputs[CSI_PORT_1].phy) {
		const struct mtk_seninf_input *input = &priv->inputs[CSI_PORT_1];

		mtk_seninf_update(priv, SENINF_TOP_PHY_SENINF_CTL_CSI1,
				  DPHY_MODE, 0 /* 4D1C */);
		mtk_seninf_update(priv, SENINF_TOP_PHY_SENINF_CTL_CSI1,
				  CK_SEL_1, input->bus.clock_lane);
		mtk_seninf_update(priv, SENINF_TOP_PHY_SENINF_CTL_CSI1,
				  PHY_SENINF_LANE_MUX_CSI1_EN, 1);
	}
}

static void mtk_seninf_input_setup_csi2_rx(struct mtk_seninf_input *input)
{
	unsigned int lanes[MTK_CSI_MAX_LANES] = { };
	unsigned int i;

	/*
	 * Configure data lane muxing. In 2D1C mode, lanes 0 to 2 correspond to
	 * CSIx[AB]_L{0,1,2}, and in 4D1C lanes 0 to 5 correspond to
	 * CSIxA_L{0,1,2}, CSIxB_L{0,1,2}.
	 *
	 * The clock lane must be skipped when calculating the index of the
	 * physical data lane. For instance, in 4D1C mode, the sensor clock
	 * lane is typically connected to lane 2 (CSIxA_L2), and the sensor
	 * data lanes 0-3 to lanes 1 (CSIxA_L1), 3 (CSIxB_L0), 0 (CSIxA_L0) and
	 * 4 (CSIxB_L1). The when skipping the clock lane, the data lane
	 * indices become 1, 2, 0 and 3.
	 */
	for (i = 0; i < input->bus.num_data_lanes; ++i) {
		lanes[i] = input->bus.data_lanes[i];
		if (lanes[i] > input->bus.clock_lane)
			lanes[i]--;
	}

	if (lane_mux_override >= 0)
		for (i = 0; i < MTK_CSI_MAX_LANES; ++i)
			lanes[i] = (lane_mux_override >> (2 * i)) & 0x3;

	dev_dbg(input->seninf->dev,
		 "input %u: %u data lanes, clock lane %u, lane mux %u %u %u %u\n",
		 input->pad, input->bus.num_data_lanes, input->bus.clock_lane,
		 lanes[0], lanes[1], lanes[2], lanes[3]);

	mtk_seninf_input_update(input, MIPI_RX_CON24_CSI0,
				CSI0_BIST_LN0_MUX, lanes[0]);
	mtk_seninf_input_update(input, MIPI_RX_CON24_CSI0,
				CSI0_BIST_LN1_MUX, lanes[1]);
	mtk_seninf_input_update(input, MIPI_RX_CON24_CSI0,
				CSI0_BIST_LN2_MUX, lanes[2]);
	mtk_seninf_input_update(input, MIPI_RX_CON24_CSI0,
				CSI0_BIST_LN3_MUX, lanes[3]);
}

static s64 mtk_seninf_get_clk_divider(struct mtk_seninf_input *input,
				      int pad_num, u8 bpp,
				      unsigned int num_data_lanes)
{
	struct media_entity *entity = &input->seninf->subdev.entity;
	struct media_pad *pad;
	s64 link_frequency, pixel_clock;

	pad = media_pad_remote_pad_first(&entity->pads[pad_num]);

	link_frequency = v4l2_get_link_freq(pad, bpp, num_data_lanes * 2);
	pixel_clock = div_u64(link_frequency * 2 * num_data_lanes, bpp);
	/*
	 * According to datasheet: Sensor master clock = ISP_clock/(CLKCNT +1)
	 * we also have the following constraint:
	 * pixel_clock >= Sensor master clock
	 */
	return div_u64(clk_get_rate(input->seninf->clks[0].clk), pixel_clock) - 1;
}

static void mtk_seninf_input_setup_csi2(struct mtk_seninf_input *input,
				       struct v4l2_subdev_state *state)
{
	const struct mtk_seninf_format_info *fmtinfo;
	const struct v4l2_mbus_framefmt *format;
	unsigned int num_data_lanes = input->bus.num_data_lanes;
	unsigned int val = 0;
	s64 clock_count;

	format = v4l2_subdev_state_get_format(state, input->pad, 0);
	fmtinfo = mtk_seninf_format_info(format->code);

	/* Configure timestamp */
	mtk_seninf_input_write(input, SENINF_TG1_TM_STP, SENINF_TIMESTAMP_STEP);

	/* HQ */
	/*
	 * Configure phase counter. Zero means:
	 * - Sensor master clock: ISP_CLK
	 * - Sensor clock polarity: Rising edge
	 * - Sensor reset deasserted
	 * - Sensor powered up
	 * - Pixel clock inversion disabled
	 * - Sensor master clock polarity disabled
	 * - Phase counter disabled
	 */
	mtk_seninf_input_write(input, SENINF_TG1_PH_CNT, 0x0);

	clock_count = mtk_seninf_get_clk_divider(input, input->pad,
						 fmtinfo->bpp,
						 num_data_lanes);

	clock_count = FIELD_PREP(SENINF_TG1_SEN_CK_CLKCNT, clock_count) | 0x1;
	mtk_seninf_input_write(input, SENINF_TG1_SEN_CK, clock_count);

	/* First Enable Sensor interface and select pad (0x1a04_0200) */
	mtk_seninf_input_update(input, SENINF_CTRL, SENINF_EN, 1);
	mtk_seninf_input_update(input, SENINF_CTRL, PAD2CAM_DATA_SEL,
				SENINF_PAD_10BIT);
	mtk_seninf_input_update(input, SENINF_CTRL, SENINF_SRC_SEL, 0);
	mtk_seninf_input_update(input, SENINF_CTRL_EXT, SENINF_CSI2_IP_EN, 1);
	mtk_seninf_input_update(input, SENINF_CTRL_EXT, SENINF_NCSI2_IP_EN, 0);

	/* DPCM Enable */
	if (fmtinfo->flags & MTK_SENINF_FORMAT_DPCM)
		val = SENINF_CSI2_DPCM_DI_2A_DPCM_EN;
	else
		val = SENINF_CSI2_DPCM_DI_30_DPCM_EN;
	mtk_seninf_input_write(input, SENINF_CSI2_DPCM, val);

	/* Settle delay */
	mtk_seninf_input_update(input, SENINF_CSI2_LNRD_TIMING,
				DATA_SETTLE_PARAMETER, SENINF_SETTLE_DELAY);

	/* CSI2 control */
	val = mtk_seninf_inuput_read(input, SENINF_CSI2_CTL)
	      | (FIELD_PREP(SENINF_CSI2_CTL_ED_SEL,
			    DATA_HEADER_ORDER_DI_WCL_WCH)
	      | SENINF_CSI2_CTL_CLOCK_LANE_EN | (BIT(num_data_lanes) - 1));
	mtk_seninf_input_write(input, SENINF_CSI2_CTL, val);

	mtk_seninf_input_update(input, SENINF_CSI2_RESYNC_MERGE_CTL,
				BYPASS_LANE_RESYNC, 0);
	mtk_seninf_input_update(input, SENINF_CSI2_RESYNC_MERGE_CTL,
				CDPHY_SEL, 0);
	mtk_seninf_input_update(input, SENINF_CSI2_RESYNC_MERGE_CTL,
				CPHY_LANE_RESYNC_CNT, 3);
	mtk_seninf_input_update(input, SENINF_CSI2_MODE, CSR_CSI2_MODE, 0);
	mtk_seninf_input_update(input, SENINF_CSI2_MODE, CSR_CSI2_HEADER_LEN,
				0);
	mtk_seninf_input_update(input, SENINF_CSI2_DPHY_SYNC, SYNC_SEQ_MASK_0,
				0xff00);
	mtk_seninf_input_update(input, SENINF_CSI2_DPHY_SYNC, SYNC_SEQ_PAT_0,
				0x001d);

	mtk_seninf_input_update(input, SENINF_CSI2_CTL, CLOCK_HS_OPTION, 0);
	mtk_seninf_input_update(input, SENINF_CSI2_CTL, HSRX_DET_EN, 0);
	mtk_seninf_input_update(input, SENINF_CSI2_CTL, HS_TRAIL_EN, 1);
	mtk_seninf_input_update(input, SENINF_CSI2_HS_TRAIL, HS_TRAIL_PARAMETER,
				SENINF_HS_TRAIL_PARAMETER);

	/* Set debug port to output packet number */
	mtk_seninf_input_update(input, SENINF_CSI2_DGB_SEL, DEBUG_EN, 1);
	mtk_seninf_input_update(input, SENINF_CSI2_DGB_SEL, DEBUG_SEL, 0x1a);

	/* HQ */
	mtk_seninf_input_write(input, SENINF_CSI2_SPARE0, 0xfffffffe);

	/* Reset the CSI2 to commit changes */
	mtk_seninf_input_update(input, SENINF_CTRL, CSI2_SW_RST, 1);
	udelay(1);
	mtk_seninf_input_update(input, SENINF_CTRL, CSI2_SW_RST, 0);
}

static void mtk_seninf_mux_setup(struct mtk_seninf_mux *mux,
				 struct mtk_seninf_input *input,
				 struct v4l2_subdev_state *state)
{
	const struct mtk_seninf_format_info *fmtinfo;
	const struct v4l2_mbus_framefmt *format;
	unsigned int pix_sel_ext;
	unsigned int pix_sel;
	unsigned int hs_pol = 0;
	unsigned int vs_pol = 0;
	unsigned int val;
	u32 rst_mask;

	format = v4l2_subdev_state_get_format(state, input->pad, 0);
	fmtinfo = mtk_seninf_format_info(format->code);

	/* Enable mux */
	mtk_seninf_mux_update(mux, SENINF_MUX_CTRL, SENINF_MUX_EN, 1);
	mtk_seninf_mux_update(mux, SENINF_MUX_CTRL, SENINF_SRC_SEL,
			      SENINF_MIPI_SENSOR);
	mtk_seninf_mux_update(mux, SENINF_MUX_CTRL_EXT, SENINF_SRC_SEL_EXT,
			      SENINF_NORMAL_MODEL);

	pix_sel_ext = 0;
	pix_sel = input->seninf->conf->two_pixel_mode ? 1 : 0;
	mtk_seninf_mux_update(mux, SENINF_MUX_CTRL_EXT, SENINF_PIX_SEL_EXT,
			      pix_sel_ext);
	mtk_seninf_mux_update(mux, SENINF_MUX_CTRL, SENINF_PIX_SEL, pix_sel);

	if (fmtinfo->flags & MTK_SENINF_FORMAT_JPEG) {
		mtk_seninf_mux_update(mux, SENINF_MUX_CTRL, FIFO_FULL_WR_EN, 0);
		mtk_seninf_mux_update(mux, SENINF_MUX_CTRL, FIFO_FLUSH_EN,
				      FIFO_FLUSH_EN_JPEG_2_PIXEL_MODE);
		mtk_seninf_mux_update(mux, SENINF_MUX_CTRL, FIFO_PUSH_EN,
				      FIFO_PUSH_EN_JPEG_2_PIXEL_MODE);
	} else {
		mtk_seninf_mux_update(mux, SENINF_MUX_CTRL, FIFO_FULL_WR_EN, 2);
		mtk_seninf_mux_update(mux, SENINF_MUX_CTRL, FIFO_FLUSH_EN,
				      FIFO_FLUSH_EN_NORMAL_MODE);
		mtk_seninf_mux_update(mux, SENINF_MUX_CTRL, FIFO_PUSH_EN,
				      FIFO_PUSH_EN_NORMAL_MODE);
	}

	mtk_seninf_mux_update(mux, SENINF_MUX_CTRL, SENINF_HSYNC_POL, hs_pol);
	mtk_seninf_mux_update(mux, SENINF_MUX_CTRL, SENINF_VSYNC_POL, vs_pol);

	val = mtk_seninf_mux_read(mux, SENINF_MUX_CTRL);
	rst_mask = SENINF_MUX_CTRL_SENINF_IRQ_SW_RST |
		   SENINF_MUX_CTRL_SENINF_MUX_SW_RST;

	mtk_seninf_mux_write(mux, SENINF_MUX_CTRL, val | rst_mask);
	mtk_seninf_mux_write(mux, SENINF_MUX_CTRL, val & ~rst_mask);

	/* HQ */
	val = SENINF_FIFO_FULL_SEL;

	/* SPARE field meaning is unknown */
	val |= 0xc0000;
	mtk_seninf_mux_write(mux, SENINF_MUX_SPARE, val);
}

static void mtk_seninf_top_mux_setup(struct mtk_seninf *priv,
				     enum mtk_seninf_id seninf_id,
				     const struct mtk_seninf_mux *mux)
{
	unsigned int val;

	/*
	 * Use the top mux (from SENINF input to MUX) to configure routing, and
	 * hardcode a 1:1 mapping from the MUX instances to the SENINF outputs.
	 */
	val = mtk_seninf_read(priv, SENINF_TOP_MUX_CTRL)
		    & ~(0xf << (mux->mux_id * 4));
	val |= (seninf_id & 0xf) << (mux->mux_id * 4);
	mtk_seninf_write(priv, SENINF_TOP_MUX_CTRL, val);

	if (priv->conf->cam_mux_8365) {
		/*
		 * We currently support only seninf version 3.0
		 * where camsv0 and camsv1 are hardwired respectively to
		 * SENINF_CAM2 and SENINF_CAM3 i.e :
		 *  - SENINF_TOP_CAM_MUX_CTRL[11:8] = 0
		 *  - SENINF_TOP_CAM_MUX_CTRL[15:12] = 1
		 * so we hardcode it here
		 */
		mtk_seninf_update(priv, SENINF_TOP_CAM_MUX_CTRL,
				  SENINF_CAM2_MUX_SRC_SEL, 0);
		mtk_seninf_update(priv, SENINF_TOP_CAM_MUX_CTRL,
				  SENINF_CAM3_MUX_SRC_SEL, 1);
		return;
	}

	/*
	 * MT8183: CAMx -> CAMSV wiring found on hardware: CAM2 -> CAMSV1,
	 * CAM3 -> CAMSV2. cam_mux_out = -1 routes the mux to every CAMx
	 * output instead (as the ChromeOS driver did by writing 0), which only
	 * works with one camera streaming at a time.
	 */
	if (cam_mux_out < 0) {
		val = 0;
		for (unsigned int i = 0; i < 8; i++)
			val |= (mux->mux_id & 0xf) << (i * 4);
	} else {
		unsigned int out = cam_mux_out + mux->pad - priv->conf->nb_inputs;

		val = mtk_seninf_read(priv, SENINF_TOP_CAM_MUX_CTRL);
		val &= ~(0xf << (out * 4));
		val |= (mux->mux_id & 0xf) << (out * 4);
	}
	mtk_seninf_write(priv, SENINF_TOP_CAM_MUX_CTRL, val);
	dev_dbg(priv->dev, "TOP_MUX_CTRL %08x CAM_MUX_CTRL %08x\n",
		 mtk_seninf_read(priv, SENINF_TOP_MUX_CTRL), val);
}

static void mtk_seninf_test_pattern(struct mtk_seninf_input *input,
				    struct mtk_seninf_mux *mux,
				    struct v4l2_subdev_state *state)
{
	const struct v4l2_mbus_framefmt *format;

	format = v4l2_subdev_state_get_format(state, input->pad, 0);

	/* Sequence from the ChromeOS MT8183 driver (TEST_GEN_PATTERN). */
	mtk_seninf_input_update(input, SENINF_CTRL, SENINF_SRC_SEL, 1);
	mtk_seninf_input_update(input, SENINF_CTRL_EXT, SENINF_TESTMDL_IP_EN, 1);
	mtk_seninf_input_write(input, SENINF_TG1_TM_SIZE,
			       ((format->height + 0x100) << 16) |
			       (format->width + 0x100));
	mtk_seninf_input_write(input, SENINF_TG1_TM_CLK, 0);
	mtk_seninf_input_write(input, SENINF_TG1_TM_STP, 1);
	mtk_seninf_input_write(input, SENINF_TG1_TM_CTL, 0x404c1);

	mtk_seninf_mux_update(mux, SENINF_MUX_CTRL, SENINF_SRC_SEL, 1);
	mtk_seninf_mux_update(mux, SENINF_MUX_CTRL_EXT, SENINF_SRC_SEL_EXT,
			      SENINF_TEST_MODEL);
	dev_dbg(input->seninf->dev, "test pattern %ux%u on input %u mux %u\n",
		 format->width, format->height, input->pad, mux->mux_id);
}

/*
 * The simple libcamera pipeline does not drive lenses, and a voice-coil lens
 * left at its rest position is focused beyond infinity. Move the lens linked
 * to the sensor (DT lens-focus -> ancillary link) to a fixed position while
 * streaming, and back to rest (no coil current) when the stream stops.
 */
static struct v4l2_subdev *mtk_seninf_find_lens(struct mtk_seninf_input *input)
{
	struct media_entity *sensor = &input->source_sd->entity;
	struct media_link *link;

	list_for_each_entry(link, &sensor->links, list) {
		struct media_entity *ent;

		if ((link->flags & MEDIA_LNK_FL_LINK_TYPE) !=
		    MEDIA_LNK_FL_ANCILLARY_LINK)
			continue;

		ent = gobj_to_entity(link->gobj1);
		if (ent == sensor)
			ent = gobj_to_entity(link->gobj0);
		if (ent->function == MEDIA_ENT_F_LENS &&
		    is_media_entity_v4l2_subdev(ent))
			return media_entity_to_v4l2_subdev(ent);
	}

	return NULL;
}

static void mtk_seninf_lens_set(struct mtk_seninf_input *input, bool on)
{
	struct v4l2_ctrl *ctrl;
	int ret;

	if (on) {
		if (lens_focus < 0 || input->lens_active)
			return;
		input->lens = mtk_seninf_find_lens(input);
		if (!input->lens)
			return;
		ret = pm_runtime_resume_and_get(input->lens->dev);
		if (ret < 0) {
			dev_warn(input->seninf->dev, "lens power-up failed: %d\n", ret);
			return;
		}
		input->lens_active = true;
	} else if (!input->lens_active || input->seninf->sleeping) {
		/*
		 * A stream stopped for a system sleep keeps the lens and its
		 * position: the lens parks and returns to that position in
		 * its own system sleep callbacks, and the restart after the
		 * sleep finds the lens active and leaves it alone.
		 */
		return;
	}

	ctrl = v4l2_ctrl_find(input->lens->ctrl_handler, V4L2_CID_FOCUS_ABSOLUTE);
	if (ctrl) {
		ret = v4l2_ctrl_s_ctrl(ctrl, on ? clamp_t(s32, lens_focus, ctrl->minimum,
							  ctrl->maximum) :
				       ctrl->minimum);
		if (ret)
			dev_warn(input->seninf->dev, "lens focus: %d\n", ret);
		else
			dev_dbg(input->seninf->dev, "lens %s at %d\n",
				input->lens->name, on ? lens_focus : 0);
	}

	if (!on) {
		pm_runtime_put(input->lens->dev);
		input->lens_active = false;
	}
}

/* -----------------------------------------------------------------------------
 * MT6785 ("csi_phy_2_0" generation)
 *
 * Same building blocks as above, but a different register map: the TOP MUX
 * selects the source SENINF of each MUX, the CAM MUX block at 0x400 selects
 * the source MUX of each of the 11 CAM mux outputs (CAM A, CAM B, CAMSV1..6,
 * ...), and the CSI-2 receiver has its own register block at 0xa00 in each
 * SENINF. The analog PHY and the D-PHY lane/timing registers belong to the
 * CSI-RX PHY driver.
 */

static u32 mtk_seninf_mt6785_dt(u32 code)
{
	const struct mtk_seninf_format_info *fmtinfo = mtk_seninf_format_info(code);

	switch (fmtinfo ? fmtinfo->bpp : 10) {
	case 8:
		return MIPI_CSI2_DT_RAW8;
	case 12:
		return MIPI_CSI2_DT_RAW12;
	case 14:
		return MIPI_CSI2_DT_RAW14;
	default:
		return MIPI_CSI2_DT_RAW10;
	}
}

static void mtk_seninf_mt6785_cam_mux(struct mtk_seninf *priv,
				      unsigned int k, unsigned int mux_id,
				      u32 width, u32 height,
				      unsigned int pix_mode)
{
	void __iomem *cm = priv->base + MT6785_SENINF_CAM_MUX_BASE;
	u32 val;

	val = readl(cm + MT6785_CAM_MUX_CTRL(k));
	val &= ~(0xf << MT6785_CAM_MUX_CTRL_SHIFT(k));
	val |= (mux_id & 0xf) << MT6785_CAM_MUX_CTRL_SHIFT(k);
	writel(val, cm + MT6785_CAM_MUX_CTRL(k));

	writel(width | (height << 16), cm + MT6785_CAM_MUX_CHK_CTL_1(k));
	val = readl(cm + MT6785_CAM_MUX_CHK_CTL_0(k));
	val &= ~MT6785_CAM_MUX_CHK_PIX_MODE;
	val |= FIELD_PREP(MT6785_CAM_MUX_CHK_PIX_MODE, pix_mode);
	writel(val, cm + MT6785_CAM_MUX_CHK_CTL_0(k));

	writel(readl(cm + MT6785_CAM_MUX_EN) | BIT(k), cm + MT6785_CAM_MUX_EN);
	writel(readl(cm + MT6785_CAM_MUX_IRQ_EN) | (3 << (2 * k)),
	       cm + MT6785_CAM_MUX_IRQ_EN);
	writel(3 << (2 * k), cm + MT6785_CAM_MUX_IRQ_STATUS);
}

/* Link frequency of the source connected to an input, in Hz. */
static s64 mtk_seninf_mt6785_link_freq(struct mtk_seninf_input *input,
				       const struct v4l2_mbus_framefmt *format)
{
	const struct mtk_seninf_format_info *fmtinfo;

	fmtinfo = mtk_seninf_format_info(format->code);
	if (!fmtinfo)
		return -EINVAL;

	return v4l2_get_link_freq(input->source_pad, fmtinfo->bpp,
				  2 * input->bus.num_data_lanes);
}

/*
 * Pixels per clock of the MUX and CAM mux path for a stream: the smallest
 * mode in which the CSI-2 pixel rate of the source fits into the CAM clock
 * (the camsys clock of the SENINF). At a 560 MHz CAM clock one pixel per
 * clock carries a pixel rate of 298 MHz, 680 MHz need two.
 */
static int mtk_seninf_mt6785_pix_mode(struct mtk_seninf *priv,
				      struct v4l2_subdev_state *state,
				      struct mtk_seninf_input *input)
{
	const struct mtk_seninf_format_info *fmtinfo;
	const struct v4l2_mbus_framefmt *format;
	unsigned long cam_rate;
	u64 pixel_rate;
	s64 link_freq;
	int mode;

	if (mt6785_pix_mode >= 0)
		return min(mt6785_pix_mode, 2);

	format = v4l2_subdev_state_get_format(state, input->pad, 0);
	fmtinfo = mtk_seninf_format_info(format->code);
	link_freq = mtk_seninf_mt6785_link_freq(input, format);
	cam_rate = clk_get_rate(priv->clks[0].clk);
	if (!fmtinfo || link_freq <= 0 || !cam_rate)
		return 0;

	pixel_rate = div_u64((u64)link_freq * 2 * input->bus.num_data_lanes,
			     fmtinfo->bpp);
	for (mode = 0; mode < 2; mode++)
		if (pixel_rate <= (u64)cam_rate << mode)
			break;

	return mode;
}

/* The SENINF input routed to a source pad, with a bound source. */
static struct mtk_seninf_input *
mtk_seninf_routed_input(struct mtk_seninf *priv,
			struct v4l2_subdev_state *state, unsigned int pad)
{
	struct mtk_seninf_input *input;
	u32 sink_pad;

	if (v4l2_subdev_routing_find_opposite_end(&state->routing, pad, 0,
						  &sink_pad, NULL))
		return NULL;

	input = &priv->inputs[sink_pad];
	if (!input->base || !input->source_sd)
		return NULL;

	return input;
}

static const struct v4l2_subdev_ops seninf_subdev_ops;

int mtk_seninf_pixel_mode(struct v4l2_subdev *sd, unsigned int pad)
{
	struct mtk_seninf *priv = sd_to_mtk_seninf(sd);
	struct mtk_seninf_input *input;
	struct v4l2_subdev_state *state;
	int mode;

	if (sd->ops != &seninf_subdev_ops)
		return -EINVAL;
	if (!priv->conf->csi_phy_2_0)
		return -EOPNOTSUPP;
	if (!mtk_seninf_pad_is_source(priv, pad))
		return -EINVAL;

	state = v4l2_subdev_lock_and_get_active_state(sd);
	input = mtk_seninf_routed_input(priv, state, pad);
	mode = input ? mtk_seninf_mt6785_pix_mode(priv, state, input) : -ENOLINK;
	v4l2_subdev_unlock_state(state);

	return mode;
}
EXPORT_SYMBOL_GPL(mtk_seninf_pixel_mode);

/*
 * Virtual channel and data type of a stream of the source connected to an
 * input. A sensor without the streams API sends its image as stream 0 and
 * describes further streams (other virtual channels) only in its frame
 * descriptor. Stream 0 keeps the data type of its bus code; a source without
 * a frame descriptor sends it on virtual channel 0.
 */
static int mtk_seninf_stream_vc_dt(struct mtk_seninf_input *input,
				   struct v4l2_subdev_state *state, u32 stream,
				   u8 *vc, u8 *dt)
{
	const struct v4l2_mbus_framefmt *format;
	struct v4l2_mbus_frame_desc fd = { };
	unsigned int i;
	int ret;

	format = v4l2_subdev_state_get_format(state, input->pad, stream);
	if (!format)
		return -EINVAL;

	ret = v4l2_subdev_call(input->source_sd, pad, get_frame_desc,
			       input->source_pad->index, &fd);
	if (ret == -ENOIOCTLCMD && stream == 0) {
		*vc = 0;
		*dt = mtk_seninf_mt6785_dt(format->code);
		return 0;
	}
	if (ret)
		return ret;
	if (fd.type != V4L2_MBUS_FRAME_DESC_TYPE_CSI2)
		return -EINVAL;

	for (i = 0; i < fd.num_entries; i++) {
		if (fd.entry[i].stream != stream)
			continue;

		*vc = fd.entry[i].bus.csi2.vc;
		*dt = stream ? fd.entry[i].bus.csi2.dt :
			       mtk_seninf_mt6785_dt(format->code);
		return 0;
	}

	return -EPIPE;
}

/*
 * The CSI-2 receiver of an input, set up for the first stream that starts:
 * one receiver stream per active route from the input, stream s on output
 * channel s. While the input runs it passes all of them; a MUX picks its
 * stream by the channel.
 */
static int mtk_seninf_mt6785_input_start(struct mtk_seninf *priv,
					 struct v4l2_subdev_state *state,
					 struct mtk_seninf_input *input)
{
	unsigned int csi = input->pad == CSI_PORT_0B ? CSI_PORT_0 : input->pad;
	u32 di[MT6785_SENINF_CSI2_NUM_STREAMS] = { };
	unsigned int lanes = input->bus.num_data_lanes;
	const struct v4l2_mbus_framefmt *format;
	union phy_configure_opts opts = { };
	struct v4l2_subdev_route *route;
	unsigned int s;
	s64 link_freq;
	u8 vc, dt;
	u32 val;
	int ret;

	for_each_active_route(&state->routing, route) {
		if (route->sink_pad != input->pad)
			continue;

		s = route->sink_stream;
		if (s >= MT6785_SENINF_CSI2_NUM_STREAMS)
			continue;
		ret = mtk_seninf_stream_vc_dt(input, state, s, &vc, &dt);
		if (ret) {
			dev_err(priv->dev, "no virtual channel for stream %u of port %u: %d\n",
				s, input->pad, ret);
			return ret;
		}
		di[s] = FIELD_PREP(MT6785_CSI2_DT_SEL, dt) |
			FIELD_PREP(MT6785_CSI2_VC_SEL, vc) |
			FIELD_PREP(MT6785_CSI2_DT_INTERLEAVE_MODE, 1) |
			MT6785_CSI2_VC_INTERLEAVE_EN;
	}

	/* The PHY needs the lane rate (deskew above 1.427 Gbit/s). */
	format = v4l2_subdev_state_get_format(state, input->pad, 0);
	link_freq = mtk_seninf_mt6785_link_freq(input, format);
	if (link_freq < 0) {
		dev_err(priv->dev, "no link frequency for port %u: %lld\n",
			input->pad, link_freq);
		return link_freq;
	}
	opts.mipi_dphy.hs_clk_rate = 2 * link_freq;
	opts.mipi_dphy.lanes = lanes;
	ret = phy_configure(input->phy, &opts);
	if (ret && ret != -EOPNOTSUPP) {
		dev_err(priv->dev, "PHY configuration failed: %d\n", ret);
		return ret;
	}

	/* Reset the CSI-2 receiver */
	mtk_seninf_input_write(input, MT6785_SENINF_CSI_CTRL,
			       mtk_seninf_inuput_read(input, MT6785_SENINF_CSI_CTRL) |
			       MT6785_SENINF_CSI2_SW_RST);
	udelay(1);
	mtk_seninf_input_write(input, MT6785_SENINF_CSI_CTRL,
			       mtk_seninf_inuput_read(input, MT6785_SENINF_CSI_CTRL) &
			       ~MT6785_SENINF_CSI2_SW_RST);

	/* CSI-2 receiver, D-PHY */
	mtk_seninf_input_write(input, MT6785_SENINF_CSI2_RESYNC_MERGE_CTRL,
			       mtk_seninf_inuput_read(input, MT6785_SENINF_CSI2_RESYNC_MERGE_CTRL) |
			       MT6785_CSI2_RESYNC_CYCLE_CNT_OPT);
	val = mtk_seninf_inuput_read(input, MT6785_SENINF_CSI2_HDR_MODE_0);
	val &= ~(MT6785_CSI2_HEADER_MODE | MT6785_CSI2_HEADER_LEN);
	mtk_seninf_input_write(input, MT6785_SENINF_CSI2_HDR_MODE_0, val);
	mtk_seninf_input_write(input, MT6785_SENINF_CSI2_OPT,
			       mtk_seninf_inuput_read(input, MT6785_SENINF_CSI2_OPT) &
			       ~MT6785_CSI2_CPHY_SEL);
	mtk_seninf_input_write(input, MT6785_SENINF_CSI2_IRQ_EN,
			       mtk_seninf_inuput_read(input, MT6785_SENINF_CSI2_IRQ_EN) |
			       MT6785_CSI2_IRQ_CLR_MODE);
	mtk_seninf_input_write(input, MT6785_SENINF_CSI2_EN, BIT(lanes) - 1);

	/*
	 * Receiver streams and output channels for a sensor with phase
	 * detection data: image on stream and channel 0, the second virtual
	 * channel on stream and channel 1.
	 */
	for (s = 0; s < MT6785_SENINF_CSI2_NUM_STREAMS; s++) {
		if (di[s]) {
			mtk_seninf_input_write(input,
					       MT6785_SENINF_CSI2_DI_CTRL(s), di[s]);
			mtk_seninf_input_write(input,
					       MT6785_SENINF_CSI2_CH_CTRL(s),
					       MT6785_CSI2_CH_STREAM(s));
		} else if (s) {
			mtk_seninf_input_write(input,
					       MT6785_SENINF_CSI2_CH_CTRL(s), 0);
		}
	}

	/* Enable the SENINF and its CSI-2 path */
	mtk_seninf_input_write(input, MT6785_SENINF_CSI_CTRL,
			       mtk_seninf_inuput_read(input, MT6785_SENINF_CSI_CTRL) |
			       MT6785_SENINF_CSI_CSI2_EN);
	mtk_seninf_input_write(input, MT6785_SENINF_CTRL,
			       mtk_seninf_inuput_read(input, MT6785_SENINF_CTRL) |
			       MT6785_SENINF_EN);

	/* Port operation mode: D-PHY */
	val = mtk_seninf_read(priv, MT6785_SENINF_TOP_PHY_CTRL_CSI(csi));
	val &= ~MT6785_PHY_CTRL_CPHY_EN;
	val |= MT6785_PHY_CTRL_DPHY_EN;
	mtk_seninf_write(priv, MT6785_SENINF_TOP_PHY_CTRL_CSI(csi), val);

	return 0;
}

/* The MUX and the CAM mux outputs of one stream of an input */
static void mtk_seninf_mt6785_mux_start(struct mtk_seninf *priv,
					struct v4l2_subdev_state *state,
					struct mtk_seninf_input *input,
					struct mtk_seninf_mux *mux, u32 stream,
					int pix_mode)
{
	void __iomem *cm = priv->base + MT6785_SENINF_CAM_MUX_BASE;
	unsigned int out = mux->pad - priv->conf->nb_inputs;
	const struct v4l2_mbus_framefmt *format;
	unsigned int shift, k;
	u32 val, opt = 0;
	u8 vc, dt;

	format = v4l2_subdev_state_get_format(state, input->pad, stream);
	mux->stream = stream;

	/*
	 * The CAM mux of a further virtual channel (phase detection data)
	 * passes only its channel and data type. Stream 0 keeps the CAM mux
	 * without a filter.
	 */
	if (stream && !mtk_seninf_stream_vc_dt(input, state, stream, &vc, &dt))
		opt = FIELD_PREP(MT6785_CAM_MUX_OPT_VC_SEL, vc) |
		      FIELD_PREP(MT6785_CAM_MUX_OPT_DT_SEL, dt) |
		      MT6785_CAM_MUX_OPT_VC_EN | MT6785_CAM_MUX_OPT_DT_EN;

	/* TOP MUX: MUX m <- SENINF n */
	if (mux->mux_id < 4) {
		shift = 8 * mux->mux_id;
		val = mtk_seninf_read(priv, MT6785_SENINF_TOP_MUX_CTRL_0);
		val &= ~(0xf << shift);
		val |= input->seninf_id << shift;
		mtk_seninf_write(priv, MT6785_SENINF_TOP_MUX_CTRL_0, val);
	} else {
		shift = 8 * (mux->mux_id - 4);
		val = mtk_seninf_read(priv, MT6785_SENINF_TOP_MUX_CTRL_1);
		val &= ~(0xf << shift);
		val |= input->seninf_id << shift;
		mtk_seninf_write(priv, MT6785_SENINF_TOP_MUX_CTRL_1, val);
	}

	/* MUX: CSI-2 channel of the stream, pixel mode, sync polarity, reset, enable */
	val = mtk_seninf_mux_read(mux, MT6785_SENINF_MUX_CTRL_1);
	val &= ~(MT6785_SENINF_MUX_SRC_SEL | MT6785_SENINF_MUX_PIX_MODE);
	val |= FIELD_PREP(MT6785_SENINF_MUX_SRC_SEL,
			  test_pattern ? MT6785_MUX_SRC_TEST_MODEL :
					 MT6785_MUX_SRC_MIPI + stream);
	val |= FIELD_PREP(MT6785_SENINF_MUX_PIX_MODE, pix_mode);
	mtk_seninf_mux_write(mux, MT6785_SENINF_MUX_CTRL_1, val);
	mtk_seninf_mux_write(mux, MT6785_SENINF_MUX_OPT,
			     mtk_seninf_mux_read(mux, MT6785_SENINF_MUX_OPT) &
			     ~(MT6785_SENINF_MUX_HSYNC_POL |
			       MT6785_SENINF_MUX_VSYNC_POL));
	val = mtk_seninf_mux_read(mux, MT6785_SENINF_MUX_CTRL_0);
	mtk_seninf_mux_write(mux, MT6785_SENINF_MUX_CTRL_0,
			     val | MT6785_SENINF_MUX_IRQ_SW_RST |
			     MT6785_SENINF_MUX_SW_RST);
	mtk_seninf_mux_write(mux, MT6785_SENINF_MUX_CTRL_0,
			     (val & ~(MT6785_SENINF_MUX_IRQ_SW_RST |
				      MT6785_SENINF_MUX_SW_RST)) |
			     MT6785_SENINF_MUX_EN);

	if (test_pattern && !stream) {
		/* Test model */
		mtk_seninf_input_write(input, MT6785_SENINF_TESTMDL_CTRL,
				       MT6785_SENINF_TESTMDL_EN);
		mtk_seninf_input_write(input, MT6785_SENINF_TM_SIZE, 0x10001f00);
		mtk_seninf_input_write(input, MT6785_SENINF_TM_CLK, 0x7);
		mtk_seninf_input_write(input, MT6785_SENINF_TM_CTL, 0x00300161);
	}

	/* CAM MUX: output pad i feeds CAM mux cam_mux_out + i, or all */
	mux->cam_mux_en = 0;
	if (cam_mux_out < 0) {
		for (k = 0; k < MT6785_NUM_CAM_MUXES; k++)
			mux->cam_mux_en |= BIT(k);
	} else {
		k = cam_mux_out + out;
		if (k < MT6785_NUM_CAM_MUXES)
			mux->cam_mux_en |= BIT(k);
	}
	for (k = 0; k < MT6785_NUM_CAM_MUXES; k++) {
		if (!(mux->cam_mux_en & BIT(k)))
			continue;
		if (opt)
			writel(opt, cm + MT6785_CAM_MUX_OPT(k));
		mtk_seninf_mt6785_cam_mux(priv, k, mux->mux_id, format->width,
					  format->height, pix_mode);
	}
}

static void mtk_seninf_mt6785_stop(struct mtk_seninf_input *input,
				   struct mtk_seninf_mux *mux);

/* Start a stream of an input, and the input itself for its first stream. */
static int mtk_seninf_mt6785_start(struct mtk_seninf *priv,
				   struct v4l2_subdev_state *state,
				   struct mtk_seninf_input *input,
				   struct mtk_seninf_mux *mux, u32 stream)
{
	const struct v4l2_mbus_framefmt *format;
	int pix_mode, ret;

	pix_mode = mtk_seninf_mt6785_pix_mode(priv, state, input);

	if (!input->users) {
		ret = mtk_seninf_mt6785_input_start(priv, state, input);
		if (ret)
			return ret;
	}

	mtk_seninf_mt6785_mux_start(priv, state, input, mux, stream, pix_mode);

	/* PHY last: analog part, then the D-PHY lane mapping (vendor order) */
	if (!input->users) {
		ret = phy_power_on(input->phy);
		if (ret) {
			dev_err(priv->dev, "PHY power on failed: %d\n", ret);
			mtk_seninf_mt6785_stop(input, mux);
			return ret;
		}
		input->phy_on = true;

		mtk_seninf_lens_set(input, true);
	}

	format = v4l2_subdev_state_get_format(state, input->pad, stream);
	dev_dbg(priv->dev,
		"start: seninf%u stream %u mux%u %ux%u code 0x%x lanes %u pix mode %d DI %08x/%08x CH %08x/%08x TOP_MUX %08x/%08x CAM_MUX %08x/%08x/%08x\n",
		input->seninf_id + 1, stream, mux->mux_id, format->width,
		format->height, format->code, input->bus.num_data_lanes,
		pix_mode,
		mtk_seninf_inuput_read(input, MT6785_SENINF_CSI2_DI_CTRL(0)),
		mtk_seninf_inuput_read(input, MT6785_SENINF_CSI2_DI_CTRL(1)),
		mtk_seninf_inuput_read(input, MT6785_SENINF_CSI2_CH_CTRL(0)),
		mtk_seninf_inuput_read(input, MT6785_SENINF_CSI2_CH_CTRL(1)),
		mtk_seninf_read(priv, MT6785_SENINF_TOP_MUX_CTRL_0),
		mtk_seninf_read(priv, MT6785_SENINF_TOP_MUX_CTRL_1),
		readl(priv->base + MT6785_SENINF_CAM_MUX_BASE + MT6785_CAM_MUX_CTRL(0)),
		readl(priv->base + MT6785_SENINF_CAM_MUX_BASE + MT6785_CAM_MUX_CTRL(4)),
		readl(priv->base + MT6785_SENINF_CAM_MUX_BASE + MT6785_CAM_MUX_CTRL(8)));

	return 0;
}

/*
 * Stop a stream of an input, and the input itself once no other stream runs
 * (input->users already counts this stream out).
 */
static void mtk_seninf_mt6785_stop(struct mtk_seninf_input *input,
				   struct mtk_seninf_mux *mux)
{
	struct mtk_seninf *priv = input->seninf;
	void __iomem *cm = priv->base + MT6785_SENINF_CAM_MUX_BASE;
	unsigned int k;
	u32 pkts, irq;

	pkts = mtk_seninf_inuput_read(input, MT6785_SENINF_CSI2_PACKET_CNT_STATUS);
	irq = mtk_seninf_inuput_read(input, MT6785_SENINF_CSI2_IRQ_STATUS);
	dev_dbg(priv->dev, "stop: seninf%u stream %u packets %08x csi2 irq %08x mux irq %08x cam mux irq %08x\n",
		input->seninf_id + 1, mux->stream, pkts, irq,
		mtk_seninf_mux_read(mux, MT6785_SENINF_MUX_IRQ_STATUS),
		readl(cm + MT6785_CAM_MUX_IRQ_STATUS));

	mtk_seninf_mux_write(mux, MT6785_SENINF_MUX_CTRL_0,
			     mtk_seninf_mux_read(mux, MT6785_SENINF_MUX_CTRL_0) &
			     ~MT6785_SENINF_MUX_EN);
	/* only the CAM mux outputs of this stream, the other may still run */
	writel(readl(cm + MT6785_CAM_MUX_EN) & ~mux->cam_mux_en,
	       cm + MT6785_CAM_MUX_EN);
	if (mux->stream)
		for (k = 0; k < MT6785_NUM_CAM_MUXES; k++)
			if (mux->cam_mux_en & BIT(k))
				writel(0, cm + MT6785_CAM_MUX_OPT(k));
	mux->cam_mux_en = 0;

	if (input->users)
		return;

	mtk_seninf_input_write(input, MT6785_SENINF_CSI2_EN, 0);
	mtk_seninf_input_write(input, MT6785_SENINF_TESTMDL_CTRL, 0);
	mtk_seninf_input_write(input, MT6785_SENINF_TM_CTL, 0);
	mtk_seninf_input_write(input, MT6785_SENINF_CSI_CTRL,
			       mtk_seninf_inuput_read(input, MT6785_SENINF_CSI_CTRL) &
			       ~MT6785_SENINF_CSI_CSI2_EN);
	mtk_seninf_input_write(input, MT6785_SENINF_CTRL,
			       mtk_seninf_inuput_read(input, MT6785_SENINF_CTRL) &
			       ~MT6785_SENINF_EN);

	mtk_seninf_lens_set(input, false);

	/* only if the start reached it (power_on and power_off pair up) */
	if (input->phy_on)
		phy_power_off(input->phy);
	input->phy_on = false;
}

static int mtk_seninf_start(struct mtk_seninf *priv,
			    struct v4l2_subdev_state *state,
			    struct mtk_seninf_input *input,
			    struct mtk_seninf_mux *mux, u32 stream)
{
	if (priv->conf->csi_phy_2_0)
		return mtk_seninf_mt6785_start(priv, state, input, mux, stream);

	if (priv->conf->init_top_ctrl) {
		/* As the ChromeOS MT8183 driver: SENINF_PCLK_EN | SENINF2_PCLK_EN. */
		mtk_seninf_write(priv, SENINF_TOP_CTRL,
				 SENINF_TOP_CTRL_SENINF_PCLK_EN |
				 SENINF_TOP_CTRL_SENINF2_PCLK_EN);
	}

	phy_power_on(input->phy);

	mtk_seninf_lens_set(input, true);

	mtk_seninf_input_setup_csi2_rx(input);
	mtk_seninf_input_setup_csi2(input, state);

	mtk_seninf_mux_setup(mux, input, state);
	mtk_seninf_top_mux_setup(priv, input->seninf_id, mux);

	if (test_pattern)
		mtk_seninf_test_pattern(input, mux, state);

	return 0;
}

static void mtk_seninf_stop(struct mtk_seninf_input *input,
			    struct mtk_seninf_mux *mux)
{
	unsigned int val;

	if (input->seninf->conf->csi_phy_2_0) {
		mtk_seninf_mt6785_stop(input, mux);
		return;
	}

	/* Disable CSI2(2.5G) first */
	val = mtk_seninf_inuput_read(input, SENINF_CSI2_CTL);
	val &= ~(SENINF_CSI2_CTL_CLOCK_LANE_EN |
		 SENINF_CSI2_CTL_DATA_LANE3_EN |
		 SENINF_CSI2_CTL_DATA_LANE2_EN |
		 SENINF_CSI2_CTL_DATA_LANE1_EN |
		 SENINF_CSI2_CTL_DATA_LANE0_EN);
	mtk_seninf_input_write(input, SENINF_CSI2_CTL, val);

	mtk_seninf_lens_set(input, false);

	phy_power_off(input->phy);
}

/* -----------------------------------------------------------------------------
 * V4L2 Subdev Operations
 */

static int seninf_enable_streams(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state, u32 pad,
				 u64 streams_mask)
{
	struct mtk_seninf *priv = sd_to_mtk_seninf(sd);
	struct mtk_seninf_input *input;
	struct mtk_seninf_mux *mux;
	struct v4l2_subdev *source;
	u32 sink_pad, sink_stream;
	int ret;

	/* Stream control can only operate on source pads. */
	if (pad < priv->conf->nb_inputs ||
	    pad >= priv->conf->nb_inputs + priv->conf->nb_outputs)
		return -EINVAL;

	/* Locate the SENINF input and MUX for the source pad. */
	ret = v4l2_subdev_routing_find_opposite_end(&state->routing, pad,
						    0, &sink_pad, &sink_stream);
	if (ret) {
		dev_dbg(priv->dev, "No sink pad routed to source pad %u\n",
			pad);
		return ret;
	}

	input = &priv->inputs[sink_pad];
	mux = &priv->muxes[pad - priv->conf->nb_inputs];

	/* The routed sink must have an endpoint and a bound source. */
	if (!input->base || !input->source_sd) {
		dev_err(priv->dev, "no source on sink pad %u (routed to pad %u)\n",
			sink_pad, pad);
		return -ENOLINK;
	}

	ret = pm_runtime_resume_and_get(priv->dev);
	if (ret < 0) {
		dev_err(priv->dev, "failed to get pm_runtime: %d\n", ret);
		return ret;
	}

	/*
	 * Start the SENINF first and then the source. The source sends all
	 * its streams (virtual channels) at once: it starts with the first
	 * stream of the input and stops with the last one.
	 */
	ret = mtk_seninf_start(priv, state, input, mux, sink_stream);
	if (ret)
		goto err_pm;

	if (!input->users) {
		source = input->source_sd;
		ret = v4l2_subdev_enable_streams(source,
						 input->source_pad->index,
						 BIT(0));
		if (ret) {
			dev_err(priv->dev, "failed to start source %s: %d\n",
				source->entity.name, ret);
			goto err_no_stream;
		}

		/* PHY checks that need the data lanes running (MT6785 deskew) */
		if (priv->conf->csi_phy_2_0) {
			ret = phy_calibrate(input->phy);
			if (ret && ret != -EOPNOTSUPP)
				dev_warn(priv->dev, "PHY calibration: %d\n", ret);
		}
	}

	input->users++;
	priv->streaming |= BIT(pad);

	return 0;

err_no_stream:
	mtk_seninf_stop(input, mux);
err_pm:
	pm_runtime_mark_last_busy(priv->dev);
	pm_runtime_put_autosuspend(priv->dev);

	return ret;
}

static int seninf_disable_streams(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *state, u32 pad,
				  u64 streams_mask)
{
	struct mtk_seninf *priv = sd_to_mtk_seninf(sd);
	struct mtk_seninf_input *input;
	struct mtk_seninf_mux *mux;
	struct v4l2_subdev *source;
	u32 sink_pad;
	int ret;

	/* Stream control can only operate on source pads. */
	if (pad < priv->conf->nb_inputs ||
	    pad >= priv->conf->nb_inputs + priv->conf->nb_outputs)
		return -EINVAL;

	/* Locate the SENINF input.*/
	ret = v4l2_subdev_routing_find_opposite_end(&state->routing, pad,
						    0, &sink_pad, NULL);
	if (ret) {
		dev_dbg(priv->dev, "No sink pad routed to source pad %u\n",
			pad);
		return ret;
	}

	input = &priv->inputs[sink_pad];
	mux = &priv->muxes[pad - priv->conf->nb_inputs];

	/* Only reachable after a successful enable, which checked the input. */
	if (WARN_ON(!input->base || !input->users))
		return -EINVAL;

	/* The source may already be unbound (notifier teardown). */
	source = input->source_sd;
	if (source && input->users == 1) {
		ret = v4l2_subdev_disable_streams(source,
						  input->source_pad->index,
						  BIT(0));
		if (ret)
			dev_err(priv->dev, "failed to stop source %s: %d\n",
				source->entity.name, ret);
	}

	input->users--;
	mtk_seninf_stop(input, mux);
	pm_runtime_mark_last_busy(priv->dev);
	pm_runtime_put_autosuspend(priv->dev);
	priv->streaming &= ~BIT(pad);

	/*
	 * The stream is stopped here even if the source failed to stop: an
	 * error would leave it enabled for the V4L2 core, its next start would
	 * fail with -EALREADY and its next stop drop the runtime PM reference
	 * a second time.
	 */
	return 0;
}

static const struct v4l2_mbus_framefmt mtk_seninf_default_fmt = {
	.code = SENINF_DEFAULT_BUS_FMT,
	.width = SENINF_DEFAULT_WIDTH,
	.height = SENINF_DEFAULT_HEIGHT,
	.field = V4L2_FIELD_NONE,
	.colorspace = V4L2_COLORSPACE_SRGB,
	.xfer_func = V4L2_XFER_FUNC_DEFAULT,
	.ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT,
	.quantization = V4L2_QUANTIZATION_DEFAULT,
};

/*
 * Whether the stream through @pad (sink or source end) runs. Called with the
 * lock of the active state held, which also protects priv->streaming.
 */
static bool mtk_seninf_pad_streaming(struct mtk_seninf *priv,
				     struct v4l2_subdev_state *state,
				     u32 pad, u32 stream)
{
	u32 source = pad;

	if (mtk_seninf_pad_is_sink(priv, pad) &&
	    v4l2_subdev_routing_find_opposite_end(&state->routing, pad, stream,
						  &source, NULL))
		return false;

	return priv->streaming & BIT(source);
}

/* The same routes, in any order */
static bool mtk_seninf_routing_equal(const struct v4l2_subdev_krouting *a,
				     const struct v4l2_subdev_krouting *b)
{
	unsigned int i, j;

	if (a->num_routes != b->num_routes)
		return false;

	for (i = 0; i < a->num_routes; i++) {
		const struct v4l2_subdev_route *ra = &a->routes[i];

		for (j = 0; j < b->num_routes; j++) {
			const struct v4l2_subdev_route *rb = &b->routes[j];

			if (ra->sink_pad == rb->sink_pad &&
			    ra->sink_stream == rb->sink_stream &&
			    ra->source_pad == rb->source_pad &&
			    ra->source_stream == rb->source_stream &&
			    ra->flags == rb->flags)
				break;
		}
		if (j == b->num_routes)
			return false;
	}

	return true;
}

/*
 * Each source pad carries one stream. A sink pad of the MT6785 SENINF takes
 * up to MT6785_SENINF_CSI2_NUM_STREAMS streams (virtual channels of the
 * sensor), the older SENINFs only stream 0.
 */
static int __seninf_set_routing(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *state,
				struct v4l2_subdev_krouting *routing)
{
	struct mtk_seninf *priv = sd_to_mtk_seninf(sd);
	u32 max_streams = priv->conf->csi_phy_2_0 ?
			  MT6785_SENINF_CSI2_NUM_STREAMS : 1;
	unsigned int i;
	int ret;

	ret = v4l2_subdev_routing_validate(sd, routing,
					   V4L2_SUBDEV_ROUTING_ONLY_1_TO_1);
	if (ret)
		return ret;

	for (i = 0; i < routing->num_routes; i++) {
		const struct v4l2_subdev_route *route = &routing->routes[i];

		if (route->source_stream || route->sink_stream >= max_streams) {
			dev_dbg(priv->dev, "route %u/%u -> %u/%u not supported\n",
				route->sink_pad, route->sink_stream,
				route->source_pad, route->source_stream);
			return -EINVAL;
		}
	}

	return v4l2_subdev_set_routing_with_fmt(sd, state, routing,
						&mtk_seninf_default_fmt);
}

/* A sink port is connected when the DT graph has an (available) endpoint. */
static bool mtk_seninf_port_connected(struct mtk_seninf *priv,
				      unsigned int port)
{
	struct fwnode_handle *ep;

	ep = fwnode_graph_get_endpoint_by_id(dev_fwnode(priv->dev), port, 0, 0);
	fwnode_handle_put(ep);

	return ep;
}

static int seninf_init_state(struct v4l2_subdev *sd,
			     struct v4l2_subdev_state *state)
{
	struct mtk_seninf *priv = sd_to_mtk_seninf(sd);
	struct v4l2_subdev_route routes[SENINF_MAX_NUM_OUTPUTS] = { };
	struct v4l2_subdev_krouting routing = {
		.routes = routes,
		.num_routes = min(priv->conf->nb_outputs, priv->conf->nb_inputs),
	};
	unsigned int sinks[SENINF_MAX_NUM_INPUTS];
	unsigned int i, n = 0;

	/*
	 * A fixed table keeps every input on its own output: a sensor added
	 * to one input does not move the streams of the others.
	 */
	if (priv->conf->default_routes) {
		routing.num_routes = 0;
		for (i = 0; i < priv->conf->nb_inputs; i++) {
			struct v4l2_subdev_route *route;

			if (!priv->conf->default_routes[i])
				continue;

			route = &routes[routing.num_routes++];
			route->sink_pad = i;
			route->source_pad = priv->conf->default_routes[i];
			route->flags = V4L2_SUBDEV_ROUTE_FL_ACTIVE;
		}

		return __seninf_set_routing(sd, state, &routing);
	}

	/*
	 * Route the sink pads 1:1 to the source pads in order, the sink pads
	 * with a connected sensor first: a board with a single sensor on CSI1
	 * then streams from the first source pad without a routing change.
	 */
	for (i = 0; i < priv->conf->nb_inputs; i++)
		if (mtk_seninf_port_connected(priv, i))
			sinks[n++] = i;
	for (i = 0; i < priv->conf->nb_inputs; i++)
		if (!mtk_seninf_port_connected(priv, i))
			sinks[n++] = i;

	for (i = 0; i < routing.num_routes; i++) {
		struct v4l2_subdev_route *route = &routes[i];

		route->sink_pad = sinks[i];
		route->sink_stream = 0;
		route->source_pad = priv->conf->nb_inputs + i;
		route->source_stream = 0;
		route->flags = V4L2_SUBDEV_ROUTE_FL_ACTIVE;
	}

	return __seninf_set_routing(sd, state, &routing);
}

static int seninf_enum_mbus_code(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_mbus_code_enum *code)
{
	const struct mtk_seninf_format_info *fmtinfo;
	struct mtk_seninf *priv = sd_to_mtk_seninf(sd);

	if (code->index >= ARRAY_SIZE(mtk_seninf_formats))
		return -EINVAL;

	fmtinfo = &mtk_seninf_formats[code->index];
	if (fmtinfo->flags & MTK_SENINF_FORMAT_INPUT_ONLY &&
	    mtk_seninf_pad_is_source(priv, code->pad))
		return -EINVAL;

	code->code = fmtinfo->code;

	return 0;
}

static int seninf_set_fmt(struct v4l2_subdev *sd,
			  struct v4l2_subdev_state *state,
			  struct v4l2_subdev_format *fmt)
{
	struct mtk_seninf *priv = sd_to_mtk_seninf(sd);
	const struct mtk_seninf_format_info *fmtinfo;
	struct v4l2_mbus_framefmt *format;

	/*
	 * TODO (?): We should disallow setting formats on the source pad
	 * completely, as the SENINF can't perform any processing. This would
	 * however break usage of the test pattern generator, as there would be
	 * no way to configure formats at all when no active input is selected.
	 */

	/* The hardware of a running stream keeps its format. */
	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE &&
	    mtk_seninf_pad_streaming(priv, state, fmt->pad, fmt->stream))
		return -EBUSY;

	/*
	 * Default to the first format if the requested media bus code isn't
	 * supported.
	 */
	fmtinfo = mtk_seninf_format_info(fmt->format.code);
	if (!fmtinfo) {
		fmtinfo = &mtk_seninf_formats[0];
		fmt->format.code = fmtinfo->code;
	}

	/* Interlaced formats are not supported yet. */
	fmt->format.field = V4L2_FIELD_NONE;

	/* Store the format. */
	format = v4l2_subdev_state_get_format(state, fmt->pad, fmt->stream);
	if (!format)
		return -EINVAL;

	*format = fmt->format;

	if (mtk_seninf_pad_is_source(priv, fmt->pad))
		return 0;

	/* Propagate the format to the corresponding source pad. */
	format = v4l2_subdev_state_get_opposite_stream_format(state, fmt->pad,
							      fmt->stream);
	if (!format)
		return -EINVAL;

	*format = fmt->format;

	if (fmtinfo->flags & MTK_SENINF_FORMAT_INPUT_ONLY)
		format->code = fmtinfo->uncompressed;

	return 0;
}

/*
 * A new routing table rebuilds the stream configurations of the state,
 * which also hold the enabled flags of the running streams. Replaced under
 * a running stream (libcamera resets the routing table of the seninf
 * whenever a process enumerates the cameras), the stream would no longer be
 * enabled for the V4L2 core: its stop would fail with -EALREADY before it
 * reaches the seninf, and the sensor would keep streaming. While a stream
 * runs, the same table is accepted and changes nothing, any other one is
 * refused.
 */
static int seninf_set_routing(struct v4l2_subdev *sd,
			      struct v4l2_subdev_state *state,
			      enum v4l2_subdev_format_whence which,
			      struct v4l2_subdev_krouting *routing)
{
	struct mtk_seninf *priv = sd_to_mtk_seninf(sd);

	if (which == V4L2_SUBDEV_FORMAT_ACTIVE && priv->streaming) {
		if (mtk_seninf_routing_equal(&state->routing, routing))
			return 0;
		return -EBUSY;
	}

	return __seninf_set_routing(sd, state, routing);
}

static const struct v4l2_subdev_core_ops seninf_subdev_core_ops = {
	.subscribe_event = v4l2_ctrl_subdev_subscribe_event,
	.unsubscribe_event = v4l2_event_subdev_unsubscribe,
};

static const struct v4l2_subdev_pad_ops seninf_subdev_pad_ops = {
	.enum_mbus_code = seninf_enum_mbus_code,
	.get_fmt = v4l2_subdev_get_fmt,
	.set_fmt = seninf_set_fmt,
	.link_validate = v4l2_subdev_link_validate_default,
	.set_routing = seninf_set_routing,
	.enable_streams = seninf_enable_streams,
	.disable_streams = seninf_disable_streams,
};

static const struct v4l2_subdev_ops seninf_subdev_ops = {
	.core = &seninf_subdev_core_ops,
	.pad = &seninf_subdev_pad_ops,
};

static const struct v4l2_subdev_internal_ops seninf_subdev_internal_ops = {
	.init_state = seninf_init_state,
};

/* -----------------------------------------------------------------------------
 * Media Entity Operations
 */

/*
 * A source without the streams API (a sensor) has the implicit stream 0 and
 * describes further streams on other virtual channels only in its frame
 * descriptor. Validate stream 0 of the link by its formats as for any link,
 * the other streams routed from the sink pad against the frame descriptor:
 * the stream has to be in it, with the bus code of the sink stream if the
 * entry names one.
 */
static int seninf_link_validate(struct media_link *link)
{
	struct v4l2_subdev *sd = media_entity_to_v4l2_subdev(link->sink->entity);
	struct v4l2_subdev_format source_fmt = {
		.which = V4L2_SUBDEV_FORMAT_ACTIVE,
		.pad = link->source->index,
	};
	struct v4l2_subdev_format sink_fmt = {
		.which = V4L2_SUBDEV_FORMAT_ACTIVE,
		.pad = link->sink->index,
	};
	struct mtk_seninf *priv = sd_to_mtk_seninf(sd);
	struct v4l2_mbus_frame_desc fd = { };
	const struct v4l2_mbus_framefmt *format;
	struct v4l2_subdev_state *state;
	struct v4l2_subdev_route *route;
	struct v4l2_subdev *source;
	bool has_fd;
	unsigned int i;
	int ret;

	if (!is_media_entity_v4l2_subdev(link->source->entity))
		return -EINVAL;

	source = media_entity_to_v4l2_subdev(link->source->entity);
	if (source->flags & V4L2_SUBDEV_FL_STREAMS)
		return v4l2_subdev_link_validate(link);

	/* The source first, without the lock of the SENINF state */
	ret = v4l2_subdev_call_state_active(source, pad, get_fmt, &source_fmt);
	if (ret)
		return ret;
	ret = v4l2_subdev_call(source, pad, get_frame_desc, link->source->index,
			       &fd);
	if (ret && ret != -ENOIOCTLCMD)
		return ret;
	has_fd = !ret && fd.type == V4L2_MBUS_FRAME_DESC_TYPE_CSI2;

	state = v4l2_subdev_lock_and_get_active_state(sd);

	for_each_active_route(&state->routing, route) {
		if (route->sink_pad != link->sink->index)
			continue;

		if (!route->sink_stream) {
			format = v4l2_subdev_state_get_format(state,
							      route->sink_pad, 0);
			sink_fmt.format = *format;
			ret = v4l2_subdev_link_validate_default(sd, link,
								&source_fmt,
								&sink_fmt);
			if (ret)
				goto out;
			continue;
		}

		ret = -EPIPE;
		for (i = 0; has_fd && i < fd.num_entries; i++) {
			if (fd.entry[i].stream != route->sink_stream)
				continue;

			format = v4l2_subdev_state_get_format(state,
							      route->sink_pad,
							      route->sink_stream);
			if (fd.entry[i].pixelcode &&
			    fd.entry[i].pixelcode != format->code) {
				dev_dbg(priv->dev,
					"stream %u: code 0x%04x, source sends 0x%04x\n",
					route->sink_stream, format->code,
					fd.entry[i].pixelcode);
				goto out;
			}
			ret = 0;
			break;
		}
		if (ret) {
			dev_dbg(priv->dev, "stream %u not sent by %s\n",
				route->sink_stream, source->entity.name);
			goto out;
		}
	}

	ret = 0;
out:
	v4l2_subdev_unlock_state(state);
	return ret;
}

/*
 * Make @src the source of @input (NULL: none). Called with the lock of the
 * active state held.
 */
static void mtk_seninf_input_activate(struct mtk_seninf_input *input,
				      struct mtk_seninf_source *src)
{
	input->active = src;
	if (src)
		input->bus = src->bus;
	input->source_sd = src ? src->sd : NULL;
	input->source_pad = src ? src->pad : NULL;
}

/*
 * The links of an input with several sources: one at a time, as with a
 * video mux. Enabling a link while another one of the input is enabled fails
 * with -EBUSY, user space disables the old link first. The media core refuses
 * any change while a stream runs through the sink pad; the stream count of
 * the input guards the start that has not reached the pipeline yet.
 */
static int seninf_link_setup(struct media_entity *entity,
			     const struct media_pad *local,
			     const struct media_pad *remote, u32 flags)
{
	struct v4l2_subdev *sd = media_entity_to_v4l2_subdev(entity);
	struct mtk_seninf *priv = sd_to_mtk_seninf(sd);
	struct mtk_seninf_source *src = NULL;
	struct v4l2_subdev_state *state;
	struct mtk_seninf_input *input;
	unsigned int i;
	int ret = 0;

	if (!mtk_seninf_pad_is_sink(priv, local->index))
		return 0;

	input = &priv->inputs[local->index];
	for (i = 0; i < input->num_sources; i++)
		if (input->sources[i].pad == remote)
			src = &input->sources[i];
	if (!src)
		return -EINVAL;

	state = v4l2_subdev_lock_and_get_active_state(sd);

	if (flags & MEDIA_LNK_FL_ENABLED) {
		if (input->active && input->active != src)
			ret = -EBUSY;
		else
			mtk_seninf_input_activate(input, src);
	} else if (input->active == src) {
		if (input->users)
			ret = -EBUSY;
		else
			mtk_seninf_input_activate(input, NULL);
	}

	v4l2_subdev_unlock_state(state);

	if (ret)
		dev_dbg(priv->dev, "link %s -> port %u: %d\n",
			remote->entity->name, local->index, ret);

	return ret;
}

static const struct media_entity_operations seninf_media_ops = {
	.get_fwnode_pad = v4l2_subdev_get_fwnode_pad_1_to_1,
	.link_setup = seninf_link_setup,
	.link_validate = seninf_link_validate,
	.has_pad_interdep = v4l2_subdev_has_pad_interdep,
};

static int seninf_subdev_init(struct mtk_seninf *priv)
{
	const unsigned int num_pads = priv->conf->nb_outputs
		+ priv->conf->nb_inputs;
	struct v4l2_subdev *sd = &priv->subdev;
	struct media_pad *pads = priv->pads;
	struct device *dev = priv->dev;
	unsigned int i;
	int ret;

	/* Initialize the entity. */
	for (i = 0; i < priv->conf->nb_inputs; i++)
		pads[i].flags = MEDIA_PAD_FL_SINK;
	for ( ; i < num_pads; i++)
		pads[i].flags = MEDIA_PAD_FL_SOURCE;

	ret = media_entity_pads_init(&priv->subdev.entity, num_pads, pads);
	if (ret)
		return ret;

	/* Initialize the subdev and its controls. */
	v4l2_subdev_init(sd, &seninf_subdev_ops);
	sd->internal_ops = &seninf_subdev_internal_ops;
	sd->dev = priv->dev;
	/* needed by v4l2_create_fwnode_links_to_pad() for the camsv links */
	sd->fwnode = dev_fwnode(priv->dev);
	sd->entity.function = MEDIA_ENT_F_VID_IF_BRIDGE;
	sd->entity.ops = &seninf_media_ops;
	sd->flags |= V4L2_SUBDEV_FL_HAS_DEVNODE | V4L2_SUBDEV_FL_HAS_EVENTS |
		V4L2_SUBDEV_FL_STREAMS;
	strscpy(sd->name, dev_name(priv->dev), sizeof(sd->name));
	v4l2_set_subdevdata(sd, priv);

	ret = v4l2_device_register_subdev(&priv->v4l2_dev, sd);
	if (ret) {
		dev_err_probe(dev, ret, "Failed to register subdev\n");
		goto err_subdev;
	}

	ret = v4l2_subdev_init_finalize(sd);
	if (ret)
		goto err_free_handler;

	return 0;

err_free_handler:
	v4l2_ctrl_handler_free(&priv->ctrl_handler);
err_subdev:
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&sd->entity);
	return ret;
}

static void seninf_subdev_cleanup(struct mtk_seninf *priv)
{
	struct v4l2_subdev *sd = &priv->subdev;

	v4l2_ctrl_handler_free(&priv->ctrl_handler);
	v4l2_subdev_cleanup(sd);
	media_entity_cleanup(&sd->entity);
}

/* -----------------------------------------------------------------------------
 * Async Subdev Notifier
 */

struct mtk_seninf_async_subdev {
	struct v4l2_async_connection asc;
	struct mtk_seninf_input *input;
	struct mtk_seninf_source *source;
	unsigned int port;
};

static int mtk_seninf_fwnode_parse(struct device *dev, unsigned int id,
				   unsigned int ep_id)
{
	static const char * const phy_names[] = {
		"csi0", "csi1", "csi2", "csi0b"
	};
	struct mtk_seninf *priv = dev_get_drvdata(dev);
	struct fwnode_handle *ep, *fwnode;
	struct mtk_seninf_input *input;
	struct mtk_seninf_source *src;
	struct mtk_seninf_async_subdev *asd;
	struct v4l2_fwnode_endpoint vep = { .bus_type = V4L2_MBUS_CSI2_DPHY };
	unsigned int port;
	int ret;

	ep = fwnode_graph_get_endpoint_by_id(dev_fwnode(dev), id, ep_id, 0);
	if (!ep)
		return 0;

	fwnode = fwnode_graph_get_remote_endpoint(ep);
	ret = v4l2_fwnode_endpoint_parse(ep, &vep);
	if (ret) {
		dev_err(dev, "Failed to parse %p fw\n", to_of_node(fwnode));
		ret = -EINVAL;
		goto out;
	}

	asd = v4l2_async_nf_add_fwnode(&priv->notifier, fwnode,
				       struct mtk_seninf_async_subdev);
	if (IS_ERR(asd)) {
		ret = PTR_ERR(asd);
		goto out;
	}

	port = vep.base.port;
	asd->port = port;

	if (mtk_seninf_pad_is_source(priv, port)) {
		ret = 0;
		goto out;
	}

	input = &priv->inputs[port];

	if (!input->num_sources) {
		input->pad = port;
		input->seninf_id = port_to_seninf_id[port];
		input->base = priv->base + 0x1000 * input->seninf_id;
		input->seninf = priv;

		/* until a source is active, the bus of the first one */
		input->bus = vep.bus.mipi_csi2;

		input->phy = devm_phy_get(dev, phy_names[port]);
		if (IS_ERR(input->phy)) {
			dev_err(dev, "failed to get phy: %ld\n", PTR_ERR(input->phy));
			ret = PTR_ERR(input->phy);
			goto out;
		}
		input->phy_mode = SENINF_PHY_MODE_4D1C;
	}

	src = &input->sources[input->num_sources++];
	src->bus = vep.bus.mipi_csi2;

	asd->input = input;
	asd->source = src;

	ret = 0;
out:
	fwnode_handle_put(ep);
	fwnode_handle_put(fwnode);
	return ret;
}


static int mtk_seninf_notifier_bound(struct v4l2_async_notifier *notifier,
				     struct v4l2_subdev *sd,
				     struct v4l2_async_connection *asc)
{
	struct mtk_seninf *priv = container_of(notifier, struct mtk_seninf,
					       notifier);
	struct mtk_seninf_async_subdev *asd =
		container_of(asc, struct mtk_seninf_async_subdev, asc);
	struct device_link *link;
	int ret;

	dev_dbg(priv->dev, "%s bound to SENINF port %u\n", sd->entity.name,
		asd->port);

	if (mtk_seninf_pad_is_sink(priv, asd->port)) {
		struct mtk_seninf_input *input = asd->input;
		struct mtk_seninf_source *src = asd->source;
		struct media_pad *sink = &priv->pads[input->pad];
		struct v4l2_subdev_state *state;
		struct media_link *mlink;
		bool enable;
		u32 flags;

		link = device_link_add(priv->dev, sd->dev,
				       DL_FLAG_PM_RUNTIME | DL_FLAG_STATELESS);
		if (!link) {
			dev_err(priv->dev,
				"Failed to create device link from source %s\n",
				sd->name);
			return -EINVAL;
		}

		/*
		 * A single source keeps its immutable link. Of several, the
		 * one on the first endpoint is linked when it binds and no
		 * other source is linked, whatever order they bind in.
		 */
		state = v4l2_subdev_lock_and_get_active_state(&priv->subdev);
		enable = !input->active && src == &input->sources[0];
		flags = enable ? MEDIA_LNK_FL_ENABLED : 0;
		if (input->num_sources == 1)
			flags |= MEDIA_LNK_FL_IMMUTABLE;

		ret = v4l2_create_fwnode_links_to_pad(sd, sink, flags);

		src->pad = NULL;
		for_each_media_entity_data_link(&sd->entity, mlink) {
			if (mlink->sink == sink) {
				src->pad = mlink->source;
				break;
			}
		}
		if (!ret && !src->pad)
			ret = -ENOLINK;
		if (!ret) {
			src->sd = sd;
			if (enable)
				mtk_seninf_input_activate(input, src);
		}
		v4l2_subdev_unlock_state(state);

		if (ret == -ENOLINK) {
			dev_err(priv->dev, "No connected sensor pad\n");
			return ret;
		}
	} else {
		link = device_link_add(sd->dev, priv->dev,
				       DL_FLAG_PM_RUNTIME | DL_FLAG_STATELESS);
		if (!link) {
			dev_err(priv->dev,
				"Failed to create device link to output %s\n",
				sd->name);
			return -EINVAL;
		}

		ret = v4l2_create_fwnode_links_to_pad(&priv->subdev,
						      &sd->entity.pads[0],
						      MEDIA_LNK_FL_IMMUTABLE |
						      MEDIA_LNK_FL_ENABLED);
	}

	if (ret) {
		dev_err(priv->dev, "Failed to create links between SENINF port %u and %s (%d)\n",
			asd->port, sd->entity.name, ret);
		return ret;
	}

	/*
	 * Register the device nodes of everything bound so far: a camera whose
	 * chain is complete works even if another sensor in the graph never
	 * binds (the notifier then never completes).
	 */
	ret = v4l2_device_register_subdev_nodes(&priv->v4l2_dev);
	if (ret)
		dev_err(priv->dev, "Failed to register subdev nodes: %d\n", ret);

	return ret;
}

static void mtk_seninf_notifier_unbind(struct v4l2_async_notifier *notifier,
				       struct v4l2_subdev *sd,
				       struct v4l2_async_connection *asc)
{
	struct mtk_seninf *priv = container_of(notifier, struct mtk_seninf,
					       notifier);
	struct mtk_seninf_async_subdev *asd =
		container_of(asc, struct mtk_seninf_async_subdev, asc);
	struct v4l2_subdev_state *state;

	if (!asd->input)
		return;

	/*
	 * Forget the source, its subdev is about to be unregistered and its
	 * link removed. An input left without a linked source links its first
	 * source when that binds again, or the one user space enables.
	 */
	state = v4l2_subdev_lock_and_get_active_state(&priv->subdev);
	asd->source->sd = NULL;
	asd->source->pad = NULL;
	if (asd->input->active == asd->source)
		mtk_seninf_input_activate(asd->input, NULL);
	v4l2_subdev_unlock_state(state);
}

static int mtk_seninf_notifier_complete(struct v4l2_async_notifier *notifier)
{
	struct mtk_seninf *priv = container_of(notifier, struct mtk_seninf,
					       notifier);
	int ret;

	ret = v4l2_device_register_subdev_nodes(&priv->v4l2_dev);
	if (ret) {
		dev_err(priv->dev, "Failed to register subdev nodes: %d\n",
			ret);
		return ret;
	}

	return 0;
}

static const struct v4l2_async_notifier_operations mtk_seninf_async_ops = {
	.bound = mtk_seninf_notifier_bound,
	.unbind = mtk_seninf_notifier_unbind,
	.complete = mtk_seninf_notifier_complete,
};

static int mtk_seninf_v4l2_async_register(struct mtk_seninf *priv)
{
	const struct mtk_seninf_conf *conf = priv->conf;
	struct device *dev = priv->dev;
	unsigned int i, ep;
	int ret;

	v4l2_async_nf_init(&priv->notifier, &priv->v4l2_dev);

	/* an input takes several sources (endpoints), an output one sink */
	for (i = 0; i < conf->nb_inputs + conf->nb_outputs; ++i) {
		for (ep = 0; ep < SENINF_MAX_NUM_SOURCES; ep++) {
			if (ep && mtk_seninf_pad_is_source(priv, i))
				break;

			ret = mtk_seninf_fwnode_parse(dev, i, ep);
			if (ret) {
				dev_err(dev,
					"Failed to parse endpoint %u at port %u: %d\n",
					ep, i, ret);
				goto err_clean_notififer;
			}
		}
	}

	priv->notifier.ops = &mtk_seninf_async_ops;
	ret = v4l2_async_nf_register(&priv->notifier);
	if (ret) {
		dev_err(dev, "Failed to register async notifier: %d\n", ret);
		goto err_clean_notififer;
	}
	return 0;

err_clean_notififer:
	v4l2_async_nf_cleanup(&priv->notifier);

	return ret;
}

/* -----------------------------------------------------------------------------
 * Probe & Remove
 */

static int mtk_seninf_v4l2_register(struct mtk_seninf *priv)
{
	struct device *dev = priv->dev;
	int ret;

	/* Initialize media device & v4l2_device. */
	struct media_device *media_dev = &priv->media_dev;
	const struct mtk_seninf_conf *conf = priv->conf;

	media_dev->dev = dev;
	strscpy(media_dev->model, conf->model, sizeof(media_dev->model));
	media_dev->hw_revision = 0;
	media_device_init(media_dev);

	priv->v4l2_dev.mdev = &priv->media_dev;

	ret = v4l2_device_register(dev, &priv->v4l2_dev);
	if (ret) {
		dev_err_probe(dev, ret, "Failed to register V4L2 device\n");
		goto err_clean_media;
	}

	 /* Initialize and register the SENINF subdev. */
	ret = seninf_subdev_init(priv);
	if (ret)
		goto err_unreg_v4l2;

	/* Set up async device */
	ret = mtk_seninf_v4l2_async_register(priv);
	if (ret) {
		dev_err_probe(dev, ret,
			      "Failed to register v4l2 async notifier\n");
		goto err_subdev_cleanup;
	}

	/* Register media device */
	ret = media_device_register(&priv->media_dev);
	if (ret) {
		dev_err_probe(dev, ret, "Failed to register media device\n");
		goto err_unreg_notifier;
	}

	return 0;

err_unreg_notifier:
	v4l2_async_nf_unregister(&priv->notifier);
err_subdev_cleanup:
	seninf_subdev_cleanup(priv);
err_unreg_v4l2:
	v4l2_device_unregister(&priv->v4l2_dev);
err_clean_media:
	media_device_cleanup(&priv->media_dev);
	return ret;
}

static void seninf_dump_range(struct seq_file *m, struct mtk_seninf *priv,
			      const char *name, u32 start, u32 end)
{
	u32 off;

	seq_printf(m, "%s:", name);
	for (off = start; off < end; off += 4)
		seq_printf(m, "%s%08x", (off - start) % 32 ? " " : "\n  ",
			   readl(priv->base + off));
	seq_puts(m, "\n");
}

/* Register snapshot of the inputs/muxes used on krane (only while powered). */
static int seninf_regs_show(struct seq_file *m, void *unused)
{
	struct mtk_seninf *priv = m->private;
	char name[48];
	unsigned int i;

	if (!pm_runtime_get_if_in_use(priv->dev)) {
		seq_puts(m, "not powered (only readable while streaming)\n");
		return 0;
	}
	if (priv->conf->csi_phy_2_0) {
		seninf_dump_range(m, priv, "top 0x000", 0x0, 0x80);
		seninf_dump_range(m, priv, "cam mux 0x400", 0x400, 0x4b0);
		seninf_dump_range(m, priv, "cam mux chk 0x500", 0x500, 0x5b0);
		for (i = 0; i < 5; i++) {
			u32 b = 0x1000 * i;

			snprintf(name, sizeof(name), "seninf%u ctrl 0x%x", i + 1, b + 0x200);
			seninf_dump_range(m, priv, name, b + 0x200, b + 0x230);
			snprintf(name, sizeof(name), "seninf%u tm 0x%x", i + 1, b + 0x600);
			seninf_dump_range(m, priv, name, b + 0x600, b + 0x624);
			snprintf(name, sizeof(name), "seninf%u csi2 0x%x", i + 1, b + 0xa00);
			seninf_dump_range(m, priv, name, b + 0xa00, b + 0xb00);
		}
		for (i = 0; i < 6; i++) {
			snprintf(name, sizeof(name), "mux%u 0x%x", i, 0x1000 * i + 0xd00);
			seninf_dump_range(m, priv, name, 0x1000 * i + 0xd00, 0x1000 * i + 0xd40);
		}
		pm_runtime_put(priv->dev);
		return 0;
	}
	seninf_dump_range(m, priv, "top 0x000", 0x0, 0x30);
	for (i = 0; i < 3; i++) {	/* SENINF1, SENINF2, SENINF3 blocks */
		snprintf(name, sizeof(name), "seninf%u ctrl 0x%x", i + 1, 0x1000 * i + 0x200);
		seninf_dump_range(m, priv, name, 0x1000 * i + 0x200, 0x1000 * i + 0x220);
		snprintf(name, sizeof(name), "seninf%u tg 0x%x", i + 1, 0x1000 * i + 0x600);
		seninf_dump_range(m, priv, name, 0x1000 * i + 0x600, 0x1000 * i + 0x620);
		snprintf(name, sizeof(name), "seninf%u rx 0x%x", i + 1, 0x1000 * i + 0x800);
		seninf_dump_range(m, priv, name, 0x1000 * i + 0x800, 0x1000 * i + 0x840);
		snprintf(name, sizeof(name), "seninf%u csi2 0x%x", i + 1, 0x1000 * i + 0xa00);
		seninf_dump_range(m, priv, name, 0x1000 * i + 0xa00, 0x1000 * i + 0xb40);
		snprintf(name, sizeof(name), "mux%u 0x%x", i, 0x1000 * i + 0xd00);
		seninf_dump_range(m, priv, name, 0x1000 * i + 0xd00, 0x1000 * i + 0xd40);
	}
	pm_runtime_put(priv->dev);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(seninf_regs);

static int seninf_pm_suspend(struct device *dev)
{
	struct mtk_seninf *priv = dev_get_drvdata(dev);

	clk_bulk_disable_unprepare(priv->num_clks, priv->clks);

	return 0;
}

static int seninf_pm_resume(struct device *dev)
{
	struct mtk_seninf *priv = dev_get_drvdata(dev);
	int ret;

	ret = clk_bulk_prepare_enable(priv->num_clks, priv->clks);
	if (ret) {
		dev_err(dev, "failed to enable clocks: %d\n", ret);
		return ret;
	}

	if (!priv->conf->csi_phy_2_0)
		mtk_seninf_csi2_setup_phy(priv);

	return 0;
}

static int seninf_prepare(struct device *dev)
{
	struct mtk_seninf *priv = dev_get_drvdata(dev);

	priv->sleeping = true;

	return 0;
}

/*
 * A stream that did not start again after the sleep left its input without
 * users and its lens held (see mtk_seninf_lens_set()): let the lens go.
 */
static void seninf_complete(struct device *dev)
{
	struct mtk_seninf *priv = dev_get_drvdata(dev);
	struct v4l2_subdev_state *state;
	unsigned int i;

	priv->sleeping = false;

	/* the subdev and its state exist from the probe on */
	state = v4l2_subdev_lock_and_get_active_state(&priv->subdev);
	for (i = 0; i < priv->conf->nb_inputs; i++) {
		struct mtk_seninf_input *input = &priv->inputs[i];

		if (!input->users && input->lens_active)
			mtk_seninf_lens_set(input, false);
	}
	v4l2_subdev_unlock_state(state);
}

static const struct dev_pm_ops runtime_pm_ops = {
	SET_RUNTIME_PM_OPS(seninf_pm_suspend, seninf_pm_resume, NULL)
	SET_SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend,
				pm_runtime_force_resume)
	.prepare = pm_sleep_ptr(seninf_prepare),
	.complete = pm_sleep_ptr(seninf_complete),
};

static int seninf_probe(struct platform_device *pdev)
{
	static const char * const clk_names[] = { "camsys", "top_mux" };
	struct device *dev = &pdev->dev;
	struct mtk_seninf *priv;
	unsigned int i;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->conf = device_get_match_data(dev);

	dev_set_drvdata(dev, priv);
	priv->dev = dev;

	priv->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(priv->base))
		return PTR_ERR(priv->base);

	if (priv->conf->csi_phy_2_0) {
		/* camsys gate and the three topckgen seninf muxes, as the vendor kernel */
		ret = devm_clk_bulk_get_all(dev, &priv->clks);
		if (ret < 0)
			return dev_err_probe(dev, ret, "Failed to get seninf clocks\n");
		priv->num_clks = ret;
		for (i = 0; i < priv->num_clks; ++i)
			dev_info(dev, "clock %s: %lu Hz\n", priv->clks[i].id,
				 clk_get_rate(priv->clks[i].clk));
	} else {
		priv->num_clks = ARRAY_SIZE(clk_names);
		priv->clks = devm_kcalloc(dev, priv->num_clks,
					  sizeof(*priv->clks), GFP_KERNEL);
		if (!priv->clks)
			return -ENOMEM;

		for (i = 0; i < priv->num_clks; ++i)
			priv->clks[i].id = clk_names[i];

		ret = devm_clk_bulk_get(dev, priv->num_clks, priv->clks);
		if (ret)
			return dev_err_probe(dev, ret, "Failed to get seninf clocks\n");
	}

	for (i = 0; i < priv->conf->nb_muxes; ++i) {
		struct mtk_seninf_mux *mux = &priv->muxes[i];

		mux->pad = priv->conf->nb_inputs + i;
		mux->mux_id = i;
		mux->base = priv->base + 0x1000 * i;
		mux->seninf = priv;
	}

	pm_runtime_enable(dev);

	priv->dbg = debugfs_create_dir(dev_name(dev), NULL);
	debugfs_create_file("regs", 0400, priv->dbg, priv, &seninf_regs_fops);

	ret = mtk_seninf_v4l2_register(priv);
	if (ret) {
		debugfs_remove_recursive(priv->dbg);
		pm_runtime_disable(dev);
	}
	return ret;
}

static void seninf_remove(struct platform_device *pdev)
{
	struct mtk_seninf *priv = dev_get_drvdata(&pdev->dev);

	debugfs_remove_recursive(priv->dbg);
	/*
	 * Unbind the sensors and CAMSVs while the media graph still exists:
	 * a CAMSV that is still streaming stops the pipeline on unbind.
	 */
	v4l2_async_nf_unregister(&priv->notifier);
	v4l2_async_nf_cleanup(&priv->notifier);
	media_device_unregister(&priv->media_dev);
	media_device_cleanup(&priv->media_dev);
	v4l2_device_unregister_subdev(&priv->subdev);
	seninf_subdev_cleanup(priv);
	v4l2_device_unregister(&priv->v4l2_dev);

	/*
	 * Do not leave the clocks running when the device is still active
	 * here: an enable count left behind keeps the camsys gate marked on
	 * while the CAM domain loses its state, and the next bind finds the
	 * SENINF unclocked.
	 */
	pm_runtime_disable(&pdev->dev);
	if (!pm_runtime_status_suspended(&pdev->dev))
		seninf_pm_suspend(&pdev->dev);
	pm_runtime_set_suspended(&pdev->dev);
}

static const struct mtk_seninf_conf seninf_8365_conf = {
	.model = "mtk-camsys-3.0",
	.nb_inputs = 4,
	.nb_muxes = 6,
	.nb_outputs = 4,
	.two_pixel_mode = true,
	.set_ck_sel_2 = true,
	.cam_mux_8365 = true,
};

/* MT8183: outputs are DT ports 4..7 -> CAMSV1..4, 1:1 with MUX0..3. */
static const struct mtk_seninf_conf seninf_8183_conf = {
	.model = "mtk-camsys-mt8183",
	.nb_inputs = 4,
	.nb_muxes = 8,
	.nb_outputs = 4,
	.init_top_ctrl = true,
};

/*
 * MT6785: default routes, the same with any set of connected sensors:
 * CSI1 -> port 4 (CAMSV1), CSI2 -> port 5 (CAMSV2), CSI0 -> port 7 (CAMSV4).
 * Port 6 (CAMSV3) has no default route: it takes a further stream (virtual
 * channel) of an input, such as the phase detection data of a sensor on
 * CSI2. CSI0B has no output left.
 */
static const u8 seninf_6785_default_routes[] = {
	[CSI_PORT_0] = 7,
	[CSI_PORT_1] = 4,
	[CSI_PORT_2] = 5,
	[CSI_PORT_0B] = 0,
};

/*
 * MT6785: inputs as on MT8183 (DT ports 0..3 = CSI0, CSI1, CSI2, CSI0B ->
 * SENINF1, 3, 5, 2), outputs DT ports 4..7 -> MUX0..3 -> CAM mux 2..5
 * (CAMSV1..4).
 */
static const struct mtk_seninf_conf seninf_6785_conf = {
	.model = "mtk-camsys-mt6785",
	.nb_inputs = 4,
	.nb_muxes = 6,
	.nb_outputs = 4,
	.default_routes = seninf_6785_default_routes,
	.csi_phy_2_0 = true,
};

static const struct of_device_id mtk_seninf_of_match[] = {
	{ .compatible = "mediatek,mt6785-seninf", .data = &seninf_6785_conf },
	{ .compatible = "mediatek,mt8365-seninf", .data = &seninf_8365_conf },
	{ .compatible = "mediatek,mt8183-seninf", .data = &seninf_8183_conf },
	{ /* sentinel */ },
};
MODULE_DEVICE_TABLE(of, mtk_seninf_of_match);

static struct platform_driver seninf_pdrv = {
	.driver	= {
		.name = "mtk-seninf",
		.pm = &runtime_pm_ops,
		.of_match_table = mtk_seninf_of_match,
	},
	.probe	= seninf_probe,
	.remove	= seninf_remove,
};

module_platform_driver(seninf_pdrv);

#if IS_ENABLED(CONFIG_MTK_SENINF30_KUNIT_TEST)
#include "mtk_seninf_test.c"
#endif

MODULE_DESCRIPTION("MTK sensor interface driver");
MODULE_AUTHOR("Louis Kuo <louis.kuo@mediatek.com>");
MODULE_LICENSE("GPL");
