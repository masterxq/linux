// SPDX-License-Identifier: GPL-2.0-only
/*
 * MediaTek MT6785 ISP pass 2 (DIP): the module profiles.
 *
 * The base profile: Bayer in, NV12 out through the Y and UV write ports, with
 * black level, white balance, raw noise reduction, demosaicing, colour matrix,
 * gamma, RGB to YUV and two luma/chroma noise reduction stages; no statistics
 * upscaler, no table input. The shading profile adds the lens shading module
 * and its table input port (MTK_DIP_PROFILE_LSC), the edge enhancement profile
 * the edge enhancement behind the first noise reduction stage with its radial
 * weights (MTK_DIP_PROFILE_EE); both combine. The scaled output adds the
 * scaler, its write port and the second stripe store (MTK_DIP_PROFILE_SCALE),
 * with or without the main output (MTK_DIP_PROFILE_NO_MAIN: 4:2:0 converter,
 * plane writer and the Y and UV ports off). The temporal noise reduction
 * profile (MTK_DIP_PROFILE_NR3D) adds the module behind the second stripe
 * store, with the two read ports of the reference, its 4:2:0 to 4:2:2 converter
 * and its radial weights.
 *
 * The YUV profile (MTK_DIP_PROFILE_YUV) reads an NV12 frame through the Y and
 * UV read ports and runs only the YUV part: the 4:2:0 to 4:2:2 and 4:2:2 to
 * 4:4:4 converters of the input, then the modules of the base profile from the
 * 4:4:4 to 4:2:2 converter on (both noise reduction stages, the edge
 * enhancement in its profile, the main output). The blend profile
 * (MTK_DIP_PROFILE_MIX) adds the blend module behind the 4:4:4 to 4:2:2
 * converter: it mixes the input with a base image (NV12, the two read ports and
 * the 4:2:0 to 4:2:2 converter the temporal noise reduction reads its reference
 * with) by a weight map (8 bits per pixel, its own read port).
 *
 * The command queue of the block loads the register ranges of these modules
 * from a register image in memory before each tile: each entry of the
 * descriptor list names a register range (address bits, word count) and the
 * bus address of its values in the image, which mirrors the register window.
 * Unused slots of the list hold a skip entry, a last entry ends it.
 *
 * Register values of the profile that the driver does not compute are
 * constants. The driver computes the geometry, the tile walker enables, the
 * frame grids and the parameter registers.
 */

#include <linux/bitfield.h>
#include <linux/io.h>
#include <linux/math.h>
#include <linux/minmax.h>
#include <linux/string.h>

#include "mtk_dip.h"
#include "mtk_dip_regs.h"

/* descriptor list: slots of 8 bytes, the size the thread reads */
#define DIP_CQ_SLOTS			(DIP_CQ_THR_DESC_SIZE_INIT / 8)
#define DIP_CQ_CMD_ADDR_LO		GENMASK(15, 0)
#define DIP_CQ_CMD_WORDS		GENMASK(25, 16)
#define DIP_CQ_CMD_OP			GENMASK(28, 26)
#define DIP_CQ_CMD_ADDR_HI		GENMASK(31, 29)
#define DIP_CQ_OP_LOAD			0
/* register address of the DIP window in the address space of the queue */
#define DIP_CQ_REG_BASE			0x22000
/* skip slot and end of list */
#define DIP_CQ_SKIP			0x04004184
#define DIP_CQ_END			0x1c000000

/*
 * Module enables of the profile. Raw input, Y and UV output, the stripe store
 * ports; black level, white balance, demosaicing, colour matrix, gamma, radial
 * weights, raw noise reduction, unpacker; RGB to YUV, chroma subsampling, both
 * noise reduction stages, plane writer; border pre-stage, decompanding,
 * companding.
 */
#define DIP_YUV_EN_PROFILE		0x10201878
/* edge enhancement profile: the module and its radial weights on top */
#define DIP_YUV_EN_EE			BIT(8)
#define DIP_YUV_EN_EE_RADIAL		BIT(13)
#define DIP_YUV2_EN_PROFILE		0x00000100
#define DIP_RGB_EN_PROFILE		0x101408dd
#define DIP_DMA_EN_PROFILE		0x00486001
#define DIP_PATH_SEL_PROFILE		0x00006040
#define DIP_RGB2_EN_PROFILE		0x0000000e
/* shading profile: the lens shading module and its table port on top */
#define DIP_RGB_EN_LSC			BIT(15)
#define DIP_DMA_EN_IMGCI		BIT(2)
/*
 * Scaled output: the scaler, the second stripe store with its two ports and
 * the scaled output port on top; without the main output the 4:2:0
 * converter, the plane writer and the Y and UV ports off
 */
#define DIP_YUV_EN_SCALER		BIT(17)
#define DIP_YUV2_EN_STRIPE2		BIT(9)
/*
 * The noise generators of the two noise reduction stages: on with the
 * scaler; without them the scaler does not end a tile. Their seed and size
 * registers keep their reset values.
 */
#define DIP_YUV2_EN_NOISE		(BIT(6) | BIT(7))
#define DIP_DMA_EN_SCALED		BIT(11)
#define DIP_DMA_EN_STRIPE2		(BIT(20) | BIT(23))
#define DIP_YUV_EN_MAIN			(BIT(21) | BIT(28))
/*
 * Temporal noise reduction: the module, its radial weights and the 4:2:0
 * to 4:2:2 converter of the reference; the Y and UV read ports of the
 * reference
 */
#define DIP_YUV_EN_NR3D			(BIT(10) | BIT(14) | BIT(24))
#define DIP_DMA_EN_REF			(BIT(4) | BIT(5))
#define DIP_DMA_EN_MAIN			(BIT(13) | BIT(14))
/*
 * YUV profile: the converters of the input, the second noise reduction
 * stage and the rest of the YUV part, the plane reader and writer; the Y
 * and UV input ports, the main output and the stripe store ports
 */
#define DIP_YUV_EN_YUVIN		0x90201876
#define DIP_RGB_EN_YUVIN		0x10000080
#define DIP_DMA_EN_YUVIN		0x00486003
#define DIP_PATH_SEL_YUVIN		0x00006050
#define DIP_FMT_IN_NV12			3
/* blend: the module and the 4:2:0 to 4:2:2 converter of the base image */
#define DIP_YUV_EN_MIX			(BIT(20) | BIT(24))
#define DIP_DMA_EN_WEIGHT		BIT(9)
/* the 4:2:0 to 4:2:2 and 4:2:2 to 4:4:4 converters of the input */
#define DIP_IN422_CTL			0x30c0
#define DIP_IN422_CTL_VAL		0x1f
#define DIP_IN422_X			0x30c4
#define DIP_IN422_Y			0x30c8
#define DIP_IN444_CTL			0x3000
#define DIP_IN444_CTL_VAL		0xf
/*
 * Shading strength per Bayer position, one byte each, in both strength
 * registers, in units of 1/32. It scales the brightness part of the table,
 * the colour part (the ratios between the Bayer positions) applies in full:
 * with 16, gains of the same size in all positions apply at half, a gain
 * on red and blue alone in full. 32 applies the table as it is.
 */
#define DIP_LSC_STRENGTH_TABLE		0x20202020
/*
 * Bit 24: the first noise reduction stage takes a gain map from the
 * statistics upscaler. Without the upscaler it waits for that map forever,
 * so the bit is clear in this profile, as is the link bit of the stage.
 */
#define DIP_MISC_SEL_PROFILE		0x00249010
#define DIP_NR1_CTL			0x3700
#define DIP_NR1_CTL_MAP_LINK		BIT(20)

/*
 * Edge enhancement: mode word (bits 1:0, 1 = edge enhancement; the six words
 * of the second mode of the module stay 0), then the words of the module.
 */
#define DIP_EE_BASE			0x46c0
#define DIP_EE_MODE			0x46c4
#define DIP_EE_MODE_EE			1
#define DIP_EE_WORD(i)			(0x46c8 + 4 * (i))
#define DIP_EE_END			0x4750
#define DIP_EE_RADIAL			0x4780

/*
 * Radial weight modules, all with the same weights: behind the demosaicing,
 * at the raw noise reduction, in front of both luma/chroma noise reduction
 * stages, of the edge enhancement and of the temporal noise reduction
 */
static const u16 mtk_dip_radial_base[] = {
	0x15c0, 0x1500, 0x3800, 0x4900, DIP_EE_RADIAL, DIP_NR3D_RADIAL,
};

/* raw noise reduction */
#define DIP_RNR_BASE			0x14c0

/*
 * First noise reduction stage: words 0-42 from 0x3700, 43-47 from 0x37d4;
 * between them ten words of the local tone map, behind its lookup table.
 * Without a tone map the table is 0 and the ten words have fixed values.
 * Bits 19:16 of word 0 are fixed at 0xf.
 */
#define DIP_YCNR_FIRST			0x3300
#define DIP_YCNR_LAST			0x37e4
#define DIP_YCNR_WORD_LO(i)		(0x3700 + 4 * (i))
#define DIP_YCNR_WORDS_LO		43
#define DIP_YCNR_WORD_HI(i)		(0x37d4 + 4 * ((i) - DIP_YCNR_WORDS_LO))
#define DIP_YCNR_TONE			0x37ac
#define DIP_YCNR_W0_FIXED		GENMASK(19, 16)

static const u32 mtk_dip_ycnr_tone[] = {
	0x00000000, 0x00000000, 0x9b7b5800, 0xe6d9d0c4, 0x9b7b5800,
	0xe6d9d0c4, 0x01000100, 0x01000100, 0x01000100, 0x01000000,
};

/* demosaicing: 14 words from 0x1540, then six single words */
#define DIP_DEMOSAIC_FIRST		0x1540
#define DIP_DEMOSAIC_LAST		0x1598

static const u16 mtk_dip_demosaic_regs[MTK_DIP_DEMOSAIC_WORDS] = {
	0x1540, 0x1544, 0x1548, 0x154c, 0x1550, 0x1554, 0x1558,
	0x155c, 0x1560, 0x1564, 0x1568, 0x156c, 0x1570, 0x1574,
	0x157c, 0x1580, 0x1584, 0x1588, 0x1594, 0x1598,
};

/* colour matrix control words */
#define DIP_CCM_CTL0_VAL		0x1fbe0419
#define DIP_CCM_CTL1_VAL		0x1ce729ce

/*
 * RGB to YUV: BT.601 full range, coefficients in units of 1/512 (JFIF:
 * Y = 0.299 R + 0.587 G + 0.114 B, U and V scaled to +-0.5), offsets 0; the
 * hardware adds 128 to U and V.
 */
static const s16 mtk_dip_bt601[3][3] = {
	{ 153, 301, 58 },
	{ -86, -170, 256 },
	{ 256, -214, -42 },
};

struct mtk_dip_cq_load {
	u8 slot;
	u16 reg;
	u16 words;
	u32 profile;
};

struct mtk_dip_reg_val {
	u16 reg;
	u32 val;
};

/*
 * Second noise reduction stage: its 18 words in the register window; the
 * words between them and behind them belong to parts of the stage that stay
 * off (0).
 */
#define DIP_NBC2_FIRST			0x4800
#define DIP_NBC2_LAST			0x4874

static const u16 mtk_dip_nbc2_regs[MTK_DIP_NBC2_WORDS] = {
	0x4800, 0x4804, 0x4808, 0x4810, 0x4814, 0x4818, 0x4830, 0x4834, 0x4838,
	0x4840, 0x4844, 0x4848, 0x484c, 0x4850, 0x4854, 0x4858, 0x485c, 0x4874,
};

/* register ranges the command queue loads before each tile: slot, first register, words */
static const struct mtk_dip_cq_load mtk_dip_cq_loads[] = {
	{ 2, 0x0050, 4 },
	{ 3, 0x00e0, 1 },
	{ 6, 0x0310, 1 },
	{ 7, 0x031c, 1 },
	{ 8, 0x0334, 6, MTK_DIP_PROFILE_SCALE },
	{ 9, 0x0358, 1, MTK_DIP_PROFILE_SCALE },
	{ 12, 0x0394, 6 },
	{ 13, 0x03b8, 1 },
	{ 14, 0x03c4, 6 },
	{ 15, 0x03e8, 1 },
	{ 21, 0x0504, 6 },
	{ 22, 0x0534, 6 },
	{ 23, 0x0564, 6 },
	{ 24, 0x0594, 6, MTK_DIP_PROFILE_NR3D | MTK_DIP_PROFILE_MIX },
	{ 25, 0x05c4, 6, MTK_DIP_PROFILE_NR3D | MTK_DIP_PROFILE_MIX },
	{ 27, 0x0624, 6, MTK_DIP_PROFILE_MIX },
	{ 28, 0x0654, 6 },
	{ 29, 0x0684, 6 },
	{ 32, 0x1440, 6 },
	{ 33, 0x15c0, 12 },
	{ 34, 0x1540, 23 },
	{ 35, 0x2080, 8 },
	{ 39, 0x3240, 13 },
	{ 40, 0x3040, 1 },
	{ 42, 0x3880, 9 },
	{ 44, DIP_EE_BASE, 37, MTK_DIP_PROFILE_EE },
	{ 46, 0x5380, 39 },
	{ 47, 0x3800, 12 },
	{ 48, 0x4900, 12 },
	{ 51, DIP_SCALER_CTL, 14, MTK_DIP_PROFILE_SCALE },
	{ 54, DIP_MIX_CTL0, 2, MTK_DIP_PROFILE_MIX },
	{ 56, DIP_EE_RADIAL, 12, MTK_DIP_PROFILE_EE },
	{ 59, 0x4800, 60 },
	{ 60, 0x1000, 1 },
	{ 61, DIP_REF422_EDGE, 3, MTK_DIP_PROFILE_NR3D | MTK_DIP_PROFILE_MIX },
	{ 63, 0x5480, 6 },
	{ 64, DIP_IN422_CTL, 3, MTK_DIP_PROFILE_YUV },
	{ 65, DIP_IN444_CTL, 1, MTK_DIP_PROFILE_YUV },
	{ 66, 0x54c0, 1 },
	{ 67, 0x2100, 29 },
	{ 69, 0x2180, 193 },
	{ 71, 0x11c0, 8 },
	{ 72, 0x1240, 32 },
	{ 76, 0x1600, 21 },
	{ 77, 0x1680, 8 },
	{ 78, 0x16c0, 16 },
	{ 79, 0x1340, 11 },
	{ 81, 0x3300, 314 },
	{ 83, 0x1500, 12 },
	{ 86, 0x14c0, 16 },
	{ 88, DIP_NR3D_RADIAL, 12, MTK_DIP_PROFILE_NR3D },
	{ 91, 0x2640, 193 },
	{ 94, 0x5000, 155 },
};

/*
 * Register values of the profile: sizes of the table ports the profile
 * leaves off, the controls of the 4:4:4 to 4:2:2 converter, the statistics
 * upscaler, the output chroma resampler and the shading module, and the
 * bypass of the second gamma stage. The filter modules take their words from
 * the parameters (mtk_dip_profile_params()).
 */
static const struct mtk_dip_reg_val mtk_dip_profile_regs[] = {
	{ 0x0540, 0x000035bf }, { 0x0548, 0x000035c0 }, { 0x0690, 0x000002bf },
	{ 0x0694, 0x00000107 }, { 0x0698, 0x000102c0 }, { 0x1340, 0x30400000 },
	{ 0x2940, 0x100307ff }, { 0x3040, 0x000000f0 }, { 0x3880, 0x00000003 },
	{ 0x5480, 0x80000002 }, { 0x548c, 0x01040004 }, { 0x5490, 0xffff0000 },
	{ 0x5494, 0xffff0000 }, { 0x54c0, 0x0000000f },
};

static void mtk_dip_img_wr(struct mtk_dip *dip, u32 reg, u32 val)
{
	u32 *img = dip->ctrl.cpu + MTK_DIP_CTRL_IMAGE;

	img[reg / 4] = val;
}

/*
 * Descriptor list: load entries in their slots, skip entries elsewhere; the
 * ranges of optional modules only in a profile with one of their flags
 */
static void mtk_dip_profile_desc(struct mtk_dip *dip)
{
	u32 *d = dip->ctrl.cpu + MTK_DIP_CTRL_DESC;
	u32 img = lower_32_bits(dip->ctrl.dma) + MTK_DIP_CTRL_IMAGE;
	const struct mtk_dip_cq_load *ld;
	unsigned int i;
	u32 addr;

	for (i = 0; i < DIP_CQ_SLOTS - 1; i++) {
		d[2 * i] = DIP_CQ_SKIP;
		d[2 * i + 1] = 0;
	}
	d[2 * i] = DIP_CQ_END;
	d[2 * i + 1] = 0;

	for (i = 0; i < ARRAY_SIZE(mtk_dip_cq_loads); i++) {
		ld = &mtk_dip_cq_loads[i];
		if (ld->profile && !(ld->profile & dip->geom.profile))
			continue;
		addr = DIP_CQ_REG_BASE + ld->reg;
		d[2 * ld->slot] = FIELD_PREP(DIP_CQ_CMD_ADDR_LO, lower_16_bits(addr)) |
				  FIELD_PREP(DIP_CQ_CMD_WORDS, ld->words - 1) |
				  FIELD_PREP(DIP_CQ_CMD_OP, DIP_CQ_OP_LOAD) |
				  FIELD_PREP(DIP_CQ_CMD_ADDR_HI, (addr >> 16) & 7);
		d[2 * ld->slot + 1] = img + ld->reg;
	}
}

/*
 * Values of the profile in [@first, @last]: 0 where the profile has none,
 * as in the image that mtk_dip_profile_build() writes
 */
static void mtk_dip_profile_defaults(struct mtk_dip *dip, u32 first, u32 last)
{
	u32 *img = dip->ctrl.cpu + MTK_DIP_CTRL_IMAGE;
	const struct mtk_dip_reg_val *rv;
	unsigned int i;

	memset(&img[first / 4], 0, last + 4 - first);
	for (i = 0; i < ARRAY_SIZE(mtk_dip_profile_regs); i++) {
		rv = &mtk_dip_profile_regs[i];
		if (rv->reg >= first && rv->reg <= last)
			img[rv->reg / 4] = rv->val;
	}
	/* no statistics upscaler: no gain map for the first noise stage */
	if (DIP_NR1_CTL >= first && DIP_NR1_CTL <= last)
		img[DIP_NR1_CTL / 4] &= ~DIP_NR1_CTL_MAP_LINK;
}

static u32 mtk_dip_s11(s16 v)
{
	return (u32)v & DIP_FIELD11_LO;
}

/**
 * mtk_dip_profile_build - register image and descriptors for a geometry
 * @dip: the device, @geom, @frame and @layout set
 *
 * Writes everything but the parameter registers; called with the job runner
 * holding the hardware idle.
 */
void mtk_dip_profile_build(struct mtk_dip *dip)
{
	const struct mtk_dip_tile_geom *g = &dip->geom;
	const struct mtk_dip_frame_cfg *f = &dip->frame;
	const struct mtk_dip_tile_layout *l = dip->layout;
	bool yuv = g->profile & MTK_DIP_PROFILE_YUV;
	unsigned int i;
	u32 lsc;

	memset(dip->ctrl.cpu, 0, MTK_DIP_CTRL_TDR);
	mtk_dip_profile_desc(dip);

	mtk_dip_profile_defaults(dip, 0, DIP_REG_SIZE - 4);

	/* tile walker */
	mtk_dip_img_wr(dip, DIP_TDR_CTL, DIP_TDR_CTL_VAL);
	mtk_dip_img_wr(dip, DIP_TDR_TILE, 0);
	mtk_dip_img_wr(dip, DIP_TDR_TCM0_EN, l->tcm[0]);
	mtk_dip_img_wr(dip, DIP_TDR_TCM1_EN, l->tcm[1]);
	mtk_dip_img_wr(dip, DIP_TDR_TCM2_EN, l->tcm[2]);

	/* ports: line size, lines, stride; the tiles set the windows */
	mtk_dip_img_wr(dip, DIP_IMGI_BASE + DIP_DMA_XSIZE,
		       DIV_ROUND_UP(g->frame_w * g->bpp, 8) - 1);
	mtk_dip_img_wr(dip, DIP_IMGI_BASE + DIP_DMA_YSIZE, g->h - 1);
	if (yuv) {
		/* NV12: Y and UV plane, the stride without the bus bit */
		mtk_dip_img_wr(dip, DIP_IMGI_BASE + DIP_DMA_STRIDE, g->in_stride);
		mtk_dip_img_wr(dip, DIP_IMGBI_BASE + DIP_DMA_XSIZE, g->w - 1);
		mtk_dip_img_wr(dip, DIP_IMGBI_BASE + DIP_DMA_YSIZE, g->h / 2 - 1);
		mtk_dip_img_wr(dip, DIP_IMGBI_BASE + DIP_DMA_STRIDE, g->in_stride);
		mtk_dip_img_wr(dip, DIP_IN422_CTL, DIP_IN422_CTL_VAL);
		mtk_dip_img_wr(dip, DIP_IN422_X, (g->w - 1) << 16);
		mtk_dip_img_wr(dip, DIP_IN422_Y, (g->h - 1) << 16);
		mtk_dip_img_wr(dip, DIP_IN444_CTL, DIP_IN444_CTL_VAL);
	} else {
		mtk_dip_img_wr(dip, DIP_IMGI_BASE + DIP_DMA_STRIDE,
			       DIP_DMA_STRIDE_BUS1 | g->in_stride);
	}
	mtk_dip_img_wr(dip, DIP_IMG3O_BASE + DIP_DMA_XSIZE, g->w - 1);
	mtk_dip_img_wr(dip, DIP_IMG3O_BASE + DIP_DMA_YSIZE, g->h - 1);
	mtk_dip_img_wr(dip, DIP_IMG3O_BASE + DIP_DMA_STRIDE, g->out_stride);
	mtk_dip_img_wr(dip, DIP_IMG3BO_BASE + DIP_DMA_XSIZE, g->w - 1);
	mtk_dip_img_wr(dip, DIP_IMG3BO_BASE + DIP_DMA_YSIZE, g->h / 2 - 1);
	mtk_dip_img_wr(dip, DIP_IMG3BO_BASE + DIP_DMA_STRIDE, g->out_stride);
	/* table ports: shading table in the shading profile, the other off */
	mtk_dip_img_wr(dip, DIP_IMGCI_BASE + DIP_DMA_XSIZE, f->lsc_stride - 1);
	mtk_dip_img_wr(dip, DIP_IMGCI_BASE + DIP_DMA_YSIZE, f->lsc_ny);
	mtk_dip_img_wr(dip, DIP_IMGCI_BASE + DIP_DMA_STRIDE, f->lsc_stride);
	mtk_dip_img_wr(dip, DIP_DEPI_BASE + DIP_DMA_XSIZE, f->stat_stride - 1);
	mtk_dip_img_wr(dip, DIP_DEPI_BASE + DIP_DMA_YSIZE, f->stat_ny - 1);
	mtk_dip_img_wr(dip, DIP_DEPI_BASE + DIP_DMA_STRIDE,
		       DIP_DMA_STRIDE_BUS1 | f->stat_stride);

	/*
	 * The statistics upscaler is off, the shading module off in the base
	 * profile; their registers get the grids of the input frame anyway.
	 * Shading strength 0 without the module.
	 */
	mtk_dip_img_wr(dip, DIP_LSC_GRID_X,
		       FIELD_PREP(DIP_LSC_GRID_SIZE, f->lsc_bw) |
		       FIELD_PREP(DIP_LSC_GRID_NUM, f->lsc_nx));
	mtk_dip_img_wr(dip, DIP_LSC_GRID_Y,
		       FIELD_PREP(DIP_LSC_GRID_SIZE, f->lsc_bh) |
		       FIELD_PREP(DIP_LSC_GRID_NUM, f->lsc_ny));
	mtk_dip_img_wr(dip, DIP_LSC_LAST,
		       FIELD_PREP(DIP_LSC_LAST_W, f->lsc_lw) |
		       FIELD_PREP(DIP_LSC_LAST_H, f->lsc_lh));
	lsc = g->profile & MTK_DIP_PROFILE_LSC ? DIP_LSC_STRENGTH_TABLE : 0;
	mtk_dip_img_wr(dip, DIP_LSC_STRENGTH0, lsc);
	mtk_dip_img_wr(dip, DIP_LSC_STRENGTH1, lsc);

	mtk_dip_img_wr(dip, DIP_STAT_SCALER_IN, f->stat_nx | f->stat_ny << 16);
	mtk_dip_img_wr(dip, DIP_STAT_SCALER_OUT, g->frame_w | g->frame_h << 16);
	mtk_dip_img_wr(dip, DIP_STAT_SCALER_STEP_X, f->stat_sx);
	mtk_dip_img_wr(dip, DIP_STAT_SCALER_STEP_Y, f->stat_sy);

	/* scaled output: YUYV port and scaler over the processed area */
	if (g->profile & MTK_DIP_PROFILE_SCALE) {
		mtk_dip_img_wr(dip, DIP_IMG2O_BASE + DIP_DMA_XSIZE, 2 * g->sw - 1);
		mtk_dip_img_wr(dip, DIP_IMG2O_BASE + DIP_DMA_YSIZE, g->sh - 1);
		mtk_dip_img_wr(dip, DIP_IMG2O_BASE + DIP_DMA_STRIDE,
			       DIP_DMA_STRIDE_OUT_S_YUYV | g->s_stride);
		mtk_dip_img_wr(dip, DIP_SCALER_CTL,
			       DIP_SCALER_CTL_H_EN | DIP_SCALER_CTL_V_EN |
			       FIELD_PREP(DIP_SCALER_CTL_H_MODE,
					  DIP_SCALER_MODE_ACC) |
			       FIELD_PREP(DIP_SCALER_CTL_V_MODE,
					  DIP_SCALER_MODE_ACC));
		mtk_dip_img_wr(dip, DIP_SCALER_IN, g->w | g->h << 16);
		mtk_dip_img_wr(dip, DIP_SCALER_OUT, g->sw | g->sh << 16);
		mtk_dip_img_wr(dip, DIP_SCALER_STEP_X, l->scale_x);
		mtk_dip_img_wr(dip, DIP_SCALER_STEP_Y, l->scale_y);
	}

	/*
	 * Temporal noise reduction: reference ports over the part the motion
	 * leaves (the base addresses start there), the blend window where the
	 * reference has pixels; the tiles set the windows of each tile
	 */
	if (g->profile & MTK_DIP_PROFILE_NR3D) {
		u32 vh = g->h - abs(g->ref_dy);

		mtk_dip_img_wr(dip, DIP_VIPI_BASE + DIP_DMA_XSIZE, g->w - 1);
		mtk_dip_img_wr(dip, DIP_VIPI_BASE + DIP_DMA_YSIZE, vh - 1);
		mtk_dip_img_wr(dip, DIP_VIPI_BASE + DIP_DMA_STRIDE,
			       DIP_DMA_STRIDE_REF_Y | g->ref_stride);
		mtk_dip_img_wr(dip, DIP_VIP2I_BASE + DIP_DMA_XSIZE, g->w - 1);
		mtk_dip_img_wr(dip, DIP_VIP2I_BASE + DIP_DMA_YSIZE, vh / 2 - 1);
		mtk_dip_img_wr(dip, DIP_VIP2I_BASE + DIP_DMA_STRIDE,
			       g->ref_stride);
		mtk_dip_img_wr(dip, DIP_REF422_EDGE, DIP_REF422_EDGE_ALL);
		mtk_dip_img_wr(dip, DIP_REF422_X, (g->w - 1) << 16);
		mtk_dip_img_wr(dip, DIP_REF422_Y, (vh - 1) << 16);
		mtk_dip_img_wr(dip, DIP_NR3D_ON_OFST,
			       max(-g->ref_dx, 0) | max(-g->ref_dy, 0) << 16);
		mtk_dip_img_wr(dip, DIP_NR3D_ON_SIZE,
			       (g->w - abs(g->ref_dx)) | vh << 16);
	}

	/*
	 * Blend: base image and weight map over the whole frame, its 4:2:0 to
	 * 4:2:2 converter like the one of the reference
	 */
	if (g->profile & MTK_DIP_PROFILE_MIX) {
		mtk_dip_img_wr(dip, DIP_VIPI_BASE + DIP_DMA_XSIZE, g->w - 1);
		mtk_dip_img_wr(dip, DIP_VIPI_BASE + DIP_DMA_YSIZE, g->h - 1);
		mtk_dip_img_wr(dip, DIP_VIPI_BASE + DIP_DMA_STRIDE,
			       DIP_DMA_STRIDE_REF_Y | g->ref_stride);
		mtk_dip_img_wr(dip, DIP_VIP2I_BASE + DIP_DMA_XSIZE, g->w - 1);
		mtk_dip_img_wr(dip, DIP_VIP2I_BASE + DIP_DMA_YSIZE, g->h / 2 - 1);
		mtk_dip_img_wr(dip, DIP_VIP2I_BASE + DIP_DMA_STRIDE,
			       g->ref_stride);
		mtk_dip_img_wr(dip, DIP_DMGI_BASE + DIP_DMA_XSIZE, g->w - 1);
		mtk_dip_img_wr(dip, DIP_DMGI_BASE + DIP_DMA_YSIZE, g->h - 1);
		mtk_dip_img_wr(dip, DIP_DMGI_BASE + DIP_DMA_STRIDE,
			       DIP_DMA_STRIDE_BUS1 | g->wt_stride);
		mtk_dip_img_wr(dip, DIP_REF422_EDGE, DIP_REF422_EDGE_ALL);
		mtk_dip_img_wr(dip, DIP_REF422_X, (g->w - 1) << 16);
		mtk_dip_img_wr(dip, DIP_REF422_Y, (g->h - 1) << 16);
	}

	/* colour matrix control, RGB to YUV */
	mtk_dip_img_wr(dip, DIP_CCM_CTL0, DIP_CCM_CTL0_VAL);
	mtk_dip_img_wr(dip, DIP_CCM_CTL1, DIP_CCM_CTL1_VAL);
	for (i = 0; i < 3; i++) {
		mtk_dip_img_wr(dip, DIP_RGB2YUV(2 * i),
			       mtk_dip_s11(mtk_dip_bt601[i][0]) |
			       mtk_dip_s11(mtk_dip_bt601[i][1]) << 16);
		mtk_dip_img_wr(dip, DIP_RGB2YUV(2 * i + 1),
			       mtk_dip_s11(mtk_dip_bt601[i][2]));
	}
}

/**
 * mtk_dip_profile_params - parameter registers into the register image
 * @dip: the device
 */
void mtk_dip_profile_params(struct mtk_dip *dip)
{
	const struct mtk_dip_param_state *p = &dip->par;
	u32 blc[ARRAY_SIZE(p->blc)], wb[ARRAY_SIZE(p->wb)];
	unsigned int i, k;

	/* the colour registers for the Bayer phase of the job */
	mtk_dip_params_bayer(p, dip->geom.pix_id, blc, wb);
	for (i = 0; i < 4; i++) {
		mtk_dip_img_wr(dip, DIP_BLC_OFFSET(i), blc[i]);
		mtk_dip_img_wr(dip, DIP_BLC_GAIN(i), blc[4 + i]);
	}
	mtk_dip_img_wr(dip, DIP_WB_SAT0, DIP_WB_SAT_FULL);
	mtk_dip_img_wr(dip, DIP_WB_SAT1, DIP_WB_SAT_FULL);
	mtk_dip_img_wr(dip, DIP_WB_GAIN_BG, wb[0]);
	mtk_dip_img_wr(dip, DIP_WB_GAIN_GR, wb[1]);
	mtk_dip_img_wr(dip, DIP_WB_FLARE0, 0);
	mtk_dip_img_wr(dip, DIP_WB_FLARE1, 0);
	for (i = 0; i < ARRAY_SIZE(p->ccm); i++)
		mtk_dip_img_wr(dip, DIP_CCM(i), p->ccm[i]);
	for (i = 0; i < MTK_DIP_GAMMA_POINTS; i++)
		mtk_dip_img_wr(dip, DIP_GAMMA_LUT(i), p->gamma[i]);
	mtk_dip_img_wr(dip, DIP_GAMMA_CTL, p->gamma_ctl);

	/* edge enhancement: loaded only in its profile */
	mtk_dip_profile_defaults(dip, DIP_EE_BASE, DIP_EE_END);
	if (p->ee) {
		mtk_dip_img_wr(dip, DIP_EE_MODE, DIP_EE_MODE_EE);
		for (i = 0; i < MTK_DIP_EE_WORDS; i++)
			mtk_dip_img_wr(dip, DIP_EE_WORD(i), p->ee_w[i]);
	}

	/* no tone map: lookup table 0 */
	mtk_dip_profile_defaults(dip, DIP_YCNR_FIRST, DIP_YCNR_LAST);
	for (i = 0; i < MTK_DIP_YCNR_WORDS; i++)
		mtk_dip_img_wr(dip, i < DIP_YCNR_WORDS_LO ?
			       DIP_YCNR_WORD_LO(i) : DIP_YCNR_WORD_HI(i),
			       p->ycnr_w[i] | (i ? 0 : DIP_YCNR_W0_FIXED));
	for (i = 0; i < ARRAY_SIZE(mtk_dip_ycnr_tone); i++)
		mtk_dip_img_wr(dip, DIP_YCNR_TONE + 4 * i, mtk_dip_ycnr_tone[i]);

	mtk_dip_profile_defaults(dip, DIP_DEMOSAIC_FIRST, DIP_DEMOSAIC_LAST);
	for (i = 0; i < MTK_DIP_DEMOSAIC_WORDS; i++)
		mtk_dip_img_wr(dip, mtk_dip_demosaic_regs[i], p->demosaic_w[i]);

	mtk_dip_profile_defaults(dip, DIP_NBC2_FIRST, DIP_NBC2_LAST);
	for (i = 0; i < MTK_DIP_NBC2_WORDS; i++)
		mtk_dip_img_wr(dip, mtk_dip_nbc2_regs[i], p->nbc2_w[i]);

	for (k = 0; k < ARRAY_SIZE(mtk_dip_radial_base); k++)
		for (i = 0; i < MTK_DIP_RADIAL_WORDS; i++)
			mtk_dip_img_wr(dip, mtk_dip_radial_base[k] + 4 * i,
				       p->radial_w[i]);

	for (i = 0; i < MTK_DIP_RNR_WORDS; i++)
		mtk_dip_img_wr(dip, DIP_RNR_BASE + 4 * i, p->rnr_w[i]);

	/* blend: in its profile only */
	mtk_dip_profile_defaults(dip, DIP_MIX_CTL0, DIP_MIX_CTL1);
	if (dip->geom.profile & MTK_DIP_PROFILE_MIX) {
		mtk_dip_img_wr(dip, DIP_MIX_CTL0, p->mix_w[0]);
		mtk_dip_img_wr(dip, DIP_MIX_CTL1, p->mix_w[1]);
	}

	/* temporal noise reduction: in its profile only */
	mtk_dip_profile_defaults(dip, DIP_NR3D_CTL, DIP_NR3D_CTL);
	mtk_dip_profile_defaults(dip, DIP_NR3D_MODE, DIP_NR3D_MODE);
	mtk_dip_profile_defaults(dip, DIP_NR3D_WORD(1), DIP_NR3D_COUNT);
	if (dip->geom.profile & MTK_DIP_PROFILE_NR3D) {
		u32 cnt = 0;

		mtk_dip_img_wr(dip, DIP_NR3D_CTL, DIP_NR3D_CTL_ON);
		mtk_dip_img_wr(dip, DIP_NR3D_MODE, p->nr3d_w[0]);
		for (i = 1; i < MTK_DIP_NR3D_WORDS; i++)
			mtk_dip_img_wr(dip, DIP_NR3D_WORD(i), p->nr3d_w[i]);
		for (i = 0; i < ARRAY_SIZE(dip->nr3d.count); i++)
			cnt |= (u32)dip->nr3d.count[i] << DIP_NR3D_COUNT_SHIFT(i);
		mtk_dip_img_wr(dip, DIP_NR3D_COUNT, cnt);
	}
}

/* DMA ports of the block that the profiles leave off, or set below */
static const u16 mtk_dip_ports_off[] = {
	DIP_IMG2O_BASE, DIP_IMG2BO_BASE, DIP_IMG3CO_BASE, DIP_FEO_BASE,
	DIP_PAK2O_BASE, DIP_IMGBI_BASE, DIP_IMGCI_BASE, DIP_VIPI_BASE,
	DIP_VIP2I_BASE, DIP_DEPI_BASE,
	DIP_VIP3I_BASE, DIP_DMGI_BASE, DIP_LCEI_BASE, DIP_UFDI_BASE,
	DIP_SMX1O_BASE, DIP_SMX3O_BASE, DIP_SMX4O_BASE, DIP_SMX1I_BASE,
	DIP_SMX3I_BASE, DIP_SMX4I_BASE,
};

/**
 * mtk_dip_profile_start - frame registers before the first tile
 * @dip: the device, powered
 * @a: bus addresses of the job
 *
 * Module enables of the profile, formats, port base addresses and the
 * command queue; the caller writes the tile description address and starts
 * each tile.
 */
void mtk_dip_profile_start(struct mtk_dip *dip, const struct mtk_dip_addrs *a)
{
	bool lsc = dip->geom.profile & MTK_DIP_PROFILE_LSC;
	bool ee = dip->geom.profile & MTK_DIP_PROFILE_EE;
	bool scale = dip->geom.profile & MTK_DIP_PROFILE_SCALE;
	bool main = !(dip->geom.profile & MTK_DIP_PROFILE_NO_MAIN);
	bool nr3d = dip->geom.profile & MTK_DIP_PROFILE_NR3D;
	bool yuv_in = dip->geom.profile & MTK_DIP_PROFILE_YUV;
	bool mix = dip->geom.profile & MTK_DIP_PROFILE_MIX;
	bool stripe2 = scale || nr3d;
	const struct mtk_dip_tile_geom *g = &dip->geom;
	void __iomem *r = dip->regs;
	u32 fmt, yuv, dma, ref;
	unsigned int i;

	fmt = FIELD_PREP(DIP_FMT_IN, dip->geom.bpp == 8 ? DIP_FMT_IN_RAW8 :
			 DIP_FMT_IN_RAW10) |
	      FIELD_PREP(DIP_FMT_OUT, DIP_FMT_OUT_NV12) |
	      FIELD_PREP(DIP_FMT_PIX_ID, dip->geom.pix_id) |
	      (nr3d ? FIELD_PREP(DIP_FMT_REF, DIP_FMT_REF_NV12) : 0);

	yuv = DIP_YUV_EN_PROFILE |
	      (ee ? DIP_YUV_EN_EE | DIP_YUV_EN_EE_RADIAL : 0) |
	      (scale ? DIP_YUV_EN_SCALER : 0) | (nr3d ? DIP_YUV_EN_NR3D : 0);
	dma = DIP_DMA_EN_PROFILE | (lsc ? DIP_DMA_EN_IMGCI : 0) |
	      (scale ? DIP_DMA_EN_SCALED : 0) |
	      (stripe2 ? DIP_DMA_EN_STRIPE2 : 0) | (nr3d ? DIP_DMA_EN_REF : 0);
	if (!main) {
		yuv &= ~DIP_YUV_EN_MAIN;
		dma &= ~DIP_DMA_EN_MAIN;
	}
	if (yuv_in) {
		/* the YUV profile has no scaler, no shading, no main-less job */
		fmt = FIELD_PREP(DIP_FMT_IN, DIP_FMT_IN_NV12) |
		      FIELD_PREP(DIP_FMT_OUT, DIP_FMT_OUT_NV12) |
		      (mix ? FIELD_PREP(DIP_FMT_REF, DIP_FMT_REF_NV12) : 0);
		yuv = DIP_YUV_EN_YUVIN |
		      (ee ? DIP_YUV_EN_EE | DIP_YUV_EN_EE_RADIAL : 0) |
		      (mix ? DIP_YUV_EN_MIX : 0);
		dma = DIP_DMA_EN_YUVIN |
		      (mix ? DIP_DMA_EN_REF | DIP_DMA_EN_WEIGHT : 0);
	}
	writel(yuv, r + DIP_CTL_YUV_EN);
	writel(DIP_YUV2_EN_PROFILE | (stripe2 ? DIP_YUV2_EN_STRIPE2 : 0) |
	       (scale ? DIP_YUV2_EN_NOISE : 0), r + DIP_CTL_YUV2_EN);
	writel(yuv_in ? DIP_RGB_EN_YUVIN :
	       DIP_RGB_EN_PROFILE | (lsc ? DIP_RGB_EN_LSC : 0),
	       r + DIP_CTL_RGB_EN);
	writel(dma, r + DIP_CTL_DMA_EN);
	writel(fmt, r + DIP_CTL_FMT_SEL);
	writel(yuv_in ? DIP_PATH_SEL_YUVIN : DIP_PATH_SEL_PROFILE,
	       r + DIP_CTL_PATH_SEL);
	writel(DIP_MISC_SEL_PROFILE, r + DIP_CTL_MISC_SEL);
	writel(DIP_RGB2_EN_PROFILE, r + DIP_CTL_RGB2_EN);

	for (i = 0; i < ARRAY_SIZE(mtk_dip_ports_off); i++)
		writel(0, r + mtk_dip_ports_off[i]);
	writel(main ? lower_32_bits(a->out) : 0, r + DIP_IMG3O_BASE);
	writel(main ? lower_32_bits(a->out) + a->out_uv : 0,
	       r + DIP_IMG3BO_BASE);
	writel(lower_32_bits(a->in), r + DIP_IMGI_BASE);
	if (yuv_in)
		writel(lower_32_bits(a->in) + a->in_uv, r + DIP_IMGBI_BASE);
	if (mix) {
		writel(lower_32_bits(a->ref), r + DIP_VIPI_BASE);
		writel(lower_32_bits(a->ref) + a->ref_uv, r + DIP_VIP2I_BASE);
		writel(lower_32_bits(a->wt), r + DIP_DMGI_BASE);
	}
	writel(lower_32_bits(dip->stripe.dma), r + DIP_SMX2O_BASE);
	writel(lower_32_bits(dip->stripe.dma), r + DIP_SMX2I_BASE);
	if (scale)
		writel(lower_32_bits(a->scaled), r + DIP_IMG2O_BASE);
	if (stripe2) {
		writel(lower_32_bits(dip->stripe.dma) + dip->stripe2_off,
		       r + DIP_SMX3O_BASE);
		writel(lower_32_bits(dip->stripe.dma) + dip->stripe2_off,
		       r + DIP_SMX3I_BASE);
	}
	if (nr3d) {
		/* the read ports start where the motion puts the frame */
		ref = max(g->ref_dy, 0) * g->ref_stride + max(g->ref_dx, 0);
		writel(lower_32_bits(a->ref) + ref, r + DIP_VIPI_BASE);
		ref = max(g->ref_dy, 0) / 2 * g->ref_stride + max(g->ref_dx, 0);
		writel(lower_32_bits(a->ref) + a->ref_uv + ref,
		       r + DIP_VIP2I_BASE);
	}
	if (lsc)
		writel(lower_32_bits(dip->lsc.dma), r + DIP_IMGCI_BASE);

	writel(lower_32_bits(dip->ctrl.dma) + MTK_DIP_CTRL_DESC,
	       r + DIP_CQ_THR_BASEADDR(MTK_DIP_CQ_THREAD));
}
