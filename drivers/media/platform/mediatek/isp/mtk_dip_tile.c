// SPDX-License-Identifier: GPL-2.0-only
/*
 * MediaTek MT6785 ISP pass 2 (DIP): tile computation for the fixed module
 * profile (Bayer to NV12).
 *
 * Behaviour of the hardware:
 *
 * The frame is cut into vertical tiles over the whole height, processed left
 * to right. Each module of the processing chain eats columns at a tile edge
 * that is not a frame edge (its loss left and right) and takes a limited
 * input width. The tiles are computed backwards from the output: the output
 * of tile k starts one column behind the output of tile k-1 and runs to the
 * frame end; walking the chain from the output to the input adds the losses
 * and cuts the input of a module to its largest width. Walking forwards
 * again from the input with the same losses gives the final ranges, and the
 * end of the output range. A stripe store between the two noise reduction
 * stages hands the right edge of a tile to the next one, so the modules in
 * front of it only process what the previous tile has not processed yet.
 *
 * All positions that index grids (shading table, statistics, radial weights)
 * are positions in the input frame, also for a cropped job.
 *
 * The scaled output branches off behind the second noise reduction stage,
 * next to the main output. The scaler needs input columns in front of the
 * first output pixel of a tile that the tile before has already processed;
 * a second stripe store behind the second noise reduction stage hands them
 * on, and the 4:2:0 converter of the main output cuts them away again. Each
 * tile writes the scaled pixels whose input lies in the tile completely.
 * The two noise generators of the noise reduction stages run with the
 * scaler; without them the scaler does not end a tile. They get a window
 * over the output of their stage.
 *
 * The temporal noise reduction sits behind the second stripe store, in
 * front of the main output and the scaler. It loses columns at inner tile
 * edges like a module of the chain (6 on the left, 8 on the right); the stripe store hands it the columns
 * in front of the tile that the tile before has processed, so the chain in
 * front of the stripe store computes every column once. Its reference (the
 * output of the job before) comes through two read ports, Y and UV of an
 * NV12 frame, and a 4:2:0 to 4:2:2 converter; each tile reads the window of
 * the reference that matches its input columns, moved by the global motion
 * of the frame, and the module blends only where that window has pixels.
 *
 * A YUV input (NV12) enters behind the raw stages: its Y and UV read ports
 * feed a 4:2:0 to 4:2:2 and a 4:2:2 to 4:4:4 converter, then the chain goes
 * on with the 4:4:4 to 4:2:2 converter of the raw path. The blend with a
 * base image by a weight map sits behind that converter: the read ports of
 * the base image (and its own 4:2:0 to 4:2:2 converter) and of the weight
 * map take exactly the input columns of the first noise reduction stage of
 * the tile (they start 2 columns behind the input port of the tile).
 *
 * Integer arithmetic only; all products stay below 2^32 for frames of up to
 * 8191 pixels in each direction.
 */

#include <linux/array_size.h>
#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/errno.h>
#include <linux/math.h>
#include <linux/math64.h>
#include <linux/minmax.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/types.h>

#include "mtk_dip_tile.h"

/*
 * Loss and largest input width of each chain module.
 * A module with a profile flag is in the chain only in profiles with it.
 */
struct mtk_dip_chain_step {
	u8 loss_l;
	u8 loss_r;
	u16 max_in;
	bool stripe;
	u32 profile;
	u32 off;
};

#define MTK_DIP_RAW_ONLY	MTK_DIP_PROFILE_YUV

static const struct mtk_dip_chain_step mtk_dip_chain[MTK_DIP_C_NUM] = {
	[MTK_DIP_C_IN]		= { 0, 0, 0 },
	[MTK_DIP_C_EDGE]	= { 6, 6, 754, false, 0, MTK_DIP_RAW_ONLY },
	[MTK_DIP_C_DECOMP]	= { 2, 2, 734, false, 0, MTK_DIP_RAW_ONLY },
	[MTK_DIP_C_COMP]	= { 2, 2, 730, false, 0, MTK_DIP_RAW_ONLY },
	[MTK_DIP_C_RAWNR]	= { 6, 6, 722, false, 0, MTK_DIP_RAW_ONLY },
	[MTK_DIP_C_DEMOSAIC]	= { 4, 4, 714, false, 0, MTK_DIP_RAW_ONLY },
	[MTK_DIP_C_420TO422]	= { 0, 0, 710, false, MTK_DIP_PROFILE_YUV },
	[MTK_DIP_C_422TO444]	= { 0, 2, 0, false, MTK_DIP_PROFILE_YUV },
	[MTK_DIP_C_444TO422]	= { 2, 0, 0 },
	[MTK_DIP_C_NR1]		= { 20, 20, 708 },
	[MTK_DIP_C_EE]		= { 6, 6, 688, false, MTK_DIP_PROFILE_EE },
	[MTK_DIP_C_STRIPE]	= { 0, 0, 0, true },
	[MTK_DIP_C_NR2]		= { 40, 40, 718 },
};

/* stripe handed from tile to tile: loss of the second stage left + right */
#define MTK_DIP_STRIPE_W	80

/*
 * Units of the tile description in their order in the block: number of
 * words, bit in the module words of the block header (word 1-4) and bit in
 * the three module enable registers of the tile walker. A unit is either
 * fully present (enable bit, words, header bit and register bit) or absent;
 * the hardware splits the block by the registers alone.
 */
enum mtk_dip_unit {
	MTK_DIP_U_IN_RAW,	/* raw input port, Y port of a YUV input */
	MTK_DIP_U_IN_UV,	/* UV port of a YUV input */
	MTK_DIP_U_IN_LSC,	/* shading table port */
	MTK_DIP_U_REF_Y,	/* reference read port, Y */
	MTK_DIP_U_REF_UV,	/* reference read port, UV */
	MTK_DIP_U_WEIGHT,	/* weight map read port of the blend */
	MTK_DIP_U_IN_STAT,	/* statistics port of the upscaler */
	MTK_DIP_U_OUT_S,	/* scaled output write port */
	MTK_DIP_U_OUT_Y,	/* Y write port */
	MTK_DIP_U_OUT_UV,	/* UV write port */
	MTK_DIP_U_UNPACK,
	MTK_DIP_U_DEMOSAIC,
	MTK_DIP_U_LSC,		/* shading */
	MTK_DIP_U_RADIAL_A,	/* radial weights, behind demosaicing */
	MTK_DIP_U_RADIAL_B,	/* radial weights, first noise stage */
	MTK_DIP_U_RADIAL_C,	/* radial weights, second noise stage */
	MTK_DIP_U_RADIAL_D,	/* radial weights, edge enhancement */
	MTK_DIP_U_RADIAL_E,	/* radial weights, temporal noise reduction */
	MTK_DIP_U_RGB2YUV,
	MTK_DIP_U_SCALER,	/* scaler of the scaled output */
	MTK_DIP_U_NR3D,		/* temporal noise reduction */
	MTK_DIP_U_420,		/* 4:2:2 to 4:2:0 with crop */
	MTK_DIP_U_REF_422,	/* reference 4:2:0 to 4:2:2 with crop */
	MTK_DIP_U_IN_422,	/* YUV input 4:2:0 to 4:2:2 with crop */
	MTK_DIP_U_IN_444,	/* YUV input 4:2:2 to 4:4:4 */
	MTK_DIP_U_422,		/* 4:4:4 to 4:2:2 */
	MTK_DIP_U_EE,		/* edge enhancement */
	MTK_DIP_U_NR1,
	MTK_DIP_U_NR2,
	MTK_DIP_U_STAT_SCALER,	/* statistics upscaler */
	MTK_DIP_U_RAWNR,
	MTK_DIP_U_RADIAL_RAW,	/* radial weights, raw noise reduction */
	MTK_DIP_U_NOISE1,	/* noise generator at the first noise stage */
	MTK_DIP_U_NOISE2,	/* noise generator at the second noise stage */
	MTK_DIP_U_STRIPE,	/* stripe store control */
	MTK_DIP_U_STRIPE_IN,	/* stripe store read port */
	MTK_DIP_U_STRIPE_OUT,	/* stripe store write port */
	MTK_DIP_U_STRIPE2,	/* second stripe store control */
	MTK_DIP_U_STRIPE2_IN,	/* second stripe store read port */
	MTK_DIP_U_STRIPE2_OUT,	/* second stripe store write port */
	MTK_DIP_U_NR3D_B,	/* temporal noise reduction, second part */
	MTK_DIP_U_EDGE,
	MTK_DIP_U_DECOMP,
	MTK_DIP_U_COMP,
	MTK_DIP_U_NUM,
};

struct mtk_dip_unit_desc {
	u8 words;
	u8 hdr_word;
	u8 hdr_bit;
	u8 reg;
	u8 reg_bit;
};

static const struct mtk_dip_unit_desc mtk_dip_units[MTK_DIP_U_NUM] = {
	[MTK_DIP_U_IN_RAW]	= { 2, 1, 0, 0, 1 },
	[MTK_DIP_U_IN_UV]	= { 3, 1, 1, 0, 2 },
	[MTK_DIP_U_WEIGHT]	= { 3, 1, 9, 0, 9 },
	[MTK_DIP_U_IN_422]	= { 3, 1, 28, 1, 6 },
	[MTK_DIP_U_IN_444]	= { 1, 1, 29, 1, 7 },
	[MTK_DIP_U_IN_LSC]	= { 2, 1, 2, 0, 3 },
	[MTK_DIP_U_REF_Y]	= { 2, 1, 4, 0, 6 },
	[MTK_DIP_U_REF_UV]	= { 2, 1, 5, 0, 7 },
	[MTK_DIP_U_IN_STAT]	= { 2, 1, 8, 0, 10 },
	[MTK_DIP_U_OUT_S]	= { 3, 1, 11, 0, 11 },
	[MTK_DIP_U_OUT_Y]	= { 3, 1, 13, 0, 13 },
	[MTK_DIP_U_OUT_UV]	= { 3, 1, 14, 0, 14 },
	[MTK_DIP_U_UNPACK]	= { 1, 1, 17, 0, 19 },
	[MTK_DIP_U_DEMOSAIC]	= { 1, 1, 21, 0, 20 },
	[MTK_DIP_U_LSC]		= { 4, 3, 5, 0, 21 },
	[MTK_DIP_U_RADIAL_A]	= { 3, 1, 20, 0, 22 },
	[MTK_DIP_U_RADIAL_B]	= { 3, 2, 7, 0, 23 },
	[MTK_DIP_U_RADIAL_C]	= { 3, 2, 8, 0, 24 },
	[MTK_DIP_U_RADIAL_D]	= { 3, 2, 9, 0, 25 },
	[MTK_DIP_U_RADIAL_E]	= { 3, 2, 10, 0, 26 },
	[MTK_DIP_U_RGB2YUV]	= { 3, 1, 30, 0, 27 },
	[MTK_DIP_U_SCALER]	= { 8, 2, 13, 0, 0 },
	[MTK_DIP_U_NR3D]	= { 4, 2, 6, 1, 2 },
	[MTK_DIP_U_420]		= { 4, 2, 17, 1, 4 },
	[MTK_DIP_U_REF_422]	= { 3, 2, 20, 1, 5 },
	[MTK_DIP_U_422]		= { 1, 1, 31, 1, 8 },
	[MTK_DIP_U_EE]		= { 1, 2, 4, 1, 12 },
	[MTK_DIP_U_NR1]		= { 1, 2, 0, 1, 13 },
	[MTK_DIP_U_NR2]		= { 1, 2, 1, 1, 14 },
	[MTK_DIP_U_STAT_SCALER]	= { 4, 3, 7, 1, 20 },
	[MTK_DIP_U_RAWNR]	= { 1, 3, 10, 1, 23 },
	[MTK_DIP_U_RADIAL_RAW]	= { 3, 3, 12, 1, 25 },
	[MTK_DIP_U_STRIPE]	= { 8, 4, 7, 2, 1 },
	[MTK_DIP_U_STRIPE_IN]	= { 3, 4, 2, 2, 6 },
	[MTK_DIP_U_STRIPE_OUT]	= { 4, 4, 3, 2, 9 },
	[MTK_DIP_U_STRIPE2]	= { 8, 4, 8, 2, 2 },
	[MTK_DIP_U_NOISE1]	= { 4, 3, 21, 0, 28 },
	[MTK_DIP_U_NOISE2]	= { 4, 3, 22, 1, 0 },
	[MTK_DIP_U_STRIPE2_IN]	= { 3, 4, 4, 2, 7 },
	[MTK_DIP_U_STRIPE2_OUT]	= { 4, 4, 5, 2, 10 },
	/* no bit in the block header */
	[MTK_DIP_U_NR3D_B]	= { 2, 0, 0, 2, 4 },
	[MTK_DIP_U_EDGE]	= { 1, 4, 20, 2, 14 },
	[MTK_DIP_U_DECOMP]	= { 2, 4, 21, 2, 16 },
	[MTK_DIP_U_COMP]	= { 2, 4, 22, 2, 17 },
};

/*
 * Modules without words of their own that still have a bit in the block
 * header: white balance, colour matrix, gamma, black level (raw input only),
 * the plane reader of a YUV input, the plane writer of the main output and
 * the blend.
 */
static const struct {
	u8 word;
	u8 bit;
	u32 profile;	/* all of these flags set */
	u32 off;	/* none of these flags set */
} mtk_dip_hdr_only[] = {
	{ 1, 19, 0, MTK_DIP_PROFILE_YUV },
	{ 1, 24, 0, MTK_DIP_PROFILE_YUV },
	{ 1, 25, 0, MTK_DIP_PROFILE_YUV },
	{ 3, 1, 0, MTK_DIP_PROFILE_YUV },
	{ 2, 24, MTK_DIP_PROFILE_YUV, 0 },
	{ 2, 25, 0, MTK_DIP_PROFILE_NO_MAIN },
	{ 2, 16, MTK_DIP_PROFILE_MIX, 0 },
};

/* tile description header */
#define MTK_DIP_HDR_RAW_EDGE		GENMASK(3, 0)
#define MTK_DIP_HDR_FIXED		BIT(5)
#define MTK_DIP_HDR_PIX_ID		GENMASK(7, 6)
#define MTK_DIP_HDR_OUT_EDGE		GENMASK(15, 12)
#define MTK_DIP_HDR_WORDS		GENMASK(23, 16)

/* tile edge flags: top and bottom always (one tile row), left, right */
#define MTK_DIP_EDGE_TOP_BOTTOM		0xc
#define MTK_DIP_EDGE_LEFT		BIT(0)
#define MTK_DIP_EDGE_RIGHT		BIT(1)

/* edge stage word: mirrored phase of the first column */
#define MTK_DIP_EDGE_PHASE_INV		0xa

/* grid of the windows of the noise generators */
#define MTK_DIP_NOISE_GRID		16

/* the 4:2:2 to 4:2:0 converter: constant offset word */
#define MTK_DIP_420_OFFSET		0x10

/* statistics grid of the upscaler */
#define MTK_DIP_STAT_NX			352
#define MTK_DIP_STAT_NY			264

#define MTK_DIP_RADIAL_FACTOR		2048

/* stripe store control word */
#define MTK_DIP_STRIPE_CTL_BASE		0x8
#define MTK_DIP_STRIPE_CTL_OUT		0x11
#define MTK_DIP_STRIPE_CTL_IN		0x6
#define MTK_DIP_STRIPE_CTL_FIRST	0x10000

/*
 * All units the profiles use, in block order; the optional ones need their
 * profile flag.
 */
static const u8 mtk_dip_profile[] = {
	MTK_DIP_U_IN_RAW, MTK_DIP_U_IN_UV, MTK_DIP_U_IN_LSC, MTK_DIP_U_REF_Y,
	MTK_DIP_U_REF_UV, MTK_DIP_U_WEIGHT, MTK_DIP_U_IN_STAT,
	MTK_DIP_U_OUT_S, MTK_DIP_U_OUT_Y, MTK_DIP_U_OUT_UV, MTK_DIP_U_UNPACK,
	MTK_DIP_U_DEMOSAIC, MTK_DIP_U_LSC, MTK_DIP_U_RADIAL_A,
	MTK_DIP_U_RADIAL_B, MTK_DIP_U_RADIAL_C, MTK_DIP_U_RADIAL_D,
	MTK_DIP_U_RADIAL_E, MTK_DIP_U_RGB2YUV, MTK_DIP_U_SCALER, MTK_DIP_U_NR3D,
	MTK_DIP_U_420, MTK_DIP_U_REF_422, MTK_DIP_U_IN_422, MTK_DIP_U_IN_444,
	MTK_DIP_U_422,
	MTK_DIP_U_EE, MTK_DIP_U_NR1, MTK_DIP_U_NR2,
	MTK_DIP_U_STAT_SCALER, MTK_DIP_U_RAWNR, MTK_DIP_U_RADIAL_RAW,
	MTK_DIP_U_NOISE1, MTK_DIP_U_NOISE2,
	MTK_DIP_U_STRIPE, MTK_DIP_U_STRIPE_IN, MTK_DIP_U_STRIPE_OUT,
	MTK_DIP_U_STRIPE2, MTK_DIP_U_STRIPE2_IN, MTK_DIP_U_STRIPE2_OUT,
	MTK_DIP_U_NR3D_B, MTK_DIP_U_EDGE, MTK_DIP_U_DECOMP, MTK_DIP_U_COMP,
};

#define MTK_DIP_HDR_WORDS_N		5

static bool mtk_dip_unit_on(const struct mtk_dip_tile_geom *g, unsigned int u)
{
	bool yuv = g->profile & MTK_DIP_PROFILE_YUV;

	switch (u) {
	case MTK_DIP_U_IN_UV:
	case MTK_DIP_U_IN_422:
	case MTK_DIP_U_IN_444:
		return yuv;
	case MTK_DIP_U_UNPACK:
	case MTK_DIP_U_DEMOSAIC:
	case MTK_DIP_U_RADIAL_A:
	case MTK_DIP_U_RGB2YUV:
	case MTK_DIP_U_RAWNR:
	case MTK_DIP_U_RADIAL_RAW:
	case MTK_DIP_U_EDGE:
	case MTK_DIP_U_DECOMP:
	case MTK_DIP_U_COMP:
		return !yuv;
	case MTK_DIP_U_WEIGHT:
		return g->profile & MTK_DIP_PROFILE_MIX;
	case MTK_DIP_U_IN_LSC:
	case MTK_DIP_U_LSC:
		return g->profile & MTK_DIP_PROFILE_LSC;
	case MTK_DIP_U_IN_STAT:
	case MTK_DIP_U_STAT_SCALER:
		return g->profile & MTK_DIP_PROFILE_STAT;
	case MTK_DIP_U_RADIAL_D:
	case MTK_DIP_U_EE:
		return g->profile & MTK_DIP_PROFILE_EE;
	case MTK_DIP_U_OUT_S:
	case MTK_DIP_U_SCALER:
	case MTK_DIP_U_NOISE1:
	case MTK_DIP_U_NOISE2:
		return g->profile & MTK_DIP_PROFILE_SCALE;
	case MTK_DIP_U_STRIPE2:
	case MTK_DIP_U_STRIPE2_IN:
	case MTK_DIP_U_STRIPE2_OUT:
		return g->profile & (MTK_DIP_PROFILE_SCALE | MTK_DIP_PROFILE_NR3D);
	case MTK_DIP_U_REF_Y:
	case MTK_DIP_U_REF_UV:
	case MTK_DIP_U_REF_422:
		return g->profile & (MTK_DIP_PROFILE_NR3D | MTK_DIP_PROFILE_MIX);
	case MTK_DIP_U_RADIAL_E:
	case MTK_DIP_U_NR3D:
	case MTK_DIP_U_NR3D_B:
		return g->profile & MTK_DIP_PROFILE_NR3D;
	case MTK_DIP_U_OUT_Y:
	case MTK_DIP_U_OUT_UV:
	case MTK_DIP_U_420:
		return !(g->profile & MTK_DIP_PROFILE_NO_MAIN);
	}

	return true;
}

static inline u32 lo_hi(s32 lo, s32 hi)
{
	return ((u32)lo & 0xffff) | (((u32)hi & 0xffff) << 16);
}

/**
 * mtk_dip_frame_cfg_init - frame grids for an input frame
 * @f: result
 * @frame_w: input frame width
 * @frame_h: input frame height
 *
 * Shading grid: 16 x 16 blocks of 2x2 cells, the last block takes the rest.
 * Statistics grid of the upscaler: fixed size, step (n - 1) / (size - 1) in
 * Q15, rounded up.
 */
void mtk_dip_frame_cfg_init(struct mtk_dip_frame_cfg *f, u32 frame_w,
			    u32 frame_h)
{
	u32 cw = frame_w / 2, ch = frame_h / 2;

	memset(f, 0, sizeof(*f));
	f->lsc_bw = cw / MTK_DIP_LSC_BLOCKS;
	f->lsc_bh = ch / MTK_DIP_LSC_BLOCKS;
	f->lsc_nx = MTK_DIP_LSC_BLOCKS - 1;
	f->lsc_ny = MTK_DIP_LSC_BLOCKS - 1;
	f->lsc_lw = cw - f->lsc_nx * f->lsc_bw;
	f->lsc_lh = ch - f->lsc_ny * f->lsc_bh;
	f->lsc_stride = MTK_DIP_LSC_BLOCKS * MTK_DIP_LSC_BLOCK_BYTES;
	f->stat_nx = MTK_DIP_STAT_NX;
	f->stat_ny = MTK_DIP_STAT_NY;
	f->stat_sx = ((MTK_DIP_STAT_NX - 1) << 15) / (frame_w - 1) +
		     !!(((MTK_DIP_STAT_NX - 1) << 15) % (frame_w - 1));
	f->stat_sy = ((MTK_DIP_STAT_NY - 1) << 15) / (frame_h - 1) +
		     !!(((MTK_DIP_STAT_NY - 1) << 15) % (frame_h - 1));
	f->stat_stride = 2 * MTK_DIP_STAT_NX;
	f->radial_cx = MTK_DIP_RADIAL_FACTOR;
	f->radial_cy = MTK_DIP_RADIAL_FACTOR;
}

static bool mtk_dip_chain_on(const struct mtk_dip_chain_step *m, u32 profile)
{
	return (m->profile & profile) == m->profile && !(m->off & profile);
}

/* Output to input through the chain, from the last module down to the first */
static void mtk_dip_tile_back(s32 w, u32 profile,
			      struct mtk_dip_tile_range *bw, s32 s, s32 e,
			      s32 prev_stripe_e)
{
	const struct mtk_dip_chain_step *m;
	s32 ins, ine;
	int i;

	for (i = MTK_DIP_C_NUM - 1; i >= 0; i--) {
		m = &mtk_dip_chain[i];
		if (!mtk_dip_chain_on(m, profile)) {
			/* not in the chain: passes the range through */
			bw[i].in_s = s;
			bw[i].in_e = e;
			continue;
		}
		if (m->stripe) {
			bw[i].in_s = s;
			bw[i].in_e = e;
			if (prev_stripe_e >= 0)
				s = max(s, prev_stripe_e + 1);
			continue;
		}
		ins = s > 0 ? s - m->loss_l : 0;
		ine = e < w - 1 ? e + m->loss_r : w - 1;
		ine = min(ine, w - 1);
		if (m->max_in && ine - ins + 1 > m->max_in)
			ine = ins + m->max_in - 1;
		bw[i].in_s = ins;
		bw[i].in_e = ine;
		s = ins;
		e = ine;
	}
}

/* Input to output: the final ranges; -ERANGE for a width the rule cannot serve */
static int mtk_dip_tile_fwd(s32 w, u32 profile,
			    const struct mtk_dip_tile_range *bw,
			    struct mtk_dip_tile *t, s32 prev_stripe_e)
{
	const struct mtk_dip_chain_step *m;
	s32 s = bw[MTK_DIP_C_IN].in_s, e = bw[MTK_DIP_C_IN].in_e;
	struct mtk_dip_tile_range *r;
	int i;

	for (i = 0; i < MTK_DIP_C_NUM; i++) {
		m = &mtk_dip_chain[i];
		r = &t->r[i];
		if (!mtk_dip_chain_on(m, profile)) {
			r->in_s = s;
			r->in_e = e;
			r->out_s = s;
			r->out_e = e;
			continue;
		}
		if (m->stripe) {
			r->in_s = bw[i].in_s;
			r->in_e = e;
			r->out_s = s;
			r->out_e = e;
			if (prev_stripe_e >= 0 && s != prev_stripe_e + 1)
				return -ERANGE;
			s = bw[i].in_s;
			continue;
		}
		/*
		 * A width over the limit here means the backwards limited tile
		 * reached the frame end upstream already and would leave a
		 * last tile of a few columns: not served.
		 */
		if (m->max_in && e - s + 1 > m->max_in)
			return -ERANGE;
		r->in_s = s;
		r->in_e = e;
		r->out_s = s > 0 ? s + m->loss_l : 0;
		r->out_e = e < w - 1 ? e - m->loss_r : w - 1;
		s = r->out_s;
		e = r->out_e;
	}
	t->out_s = s;
	t->out_e = e;

	return 0;
}

/*
 * Scaler (source accumulation): output pixel j covers the input around
 * j / step. These are the input columns an output range needs and the
 * output pixels an input range serves, as the hardware takes them.
 */
#define MTK_DIP_SCALE_ONE		BIT(MTK_DIP_SCALE_SHIFT)

static u32 mtk_dip_scale_step(u32 in, u32 out)
{
	return DIV_ROUND_UP_ULL((u64)(out - 1) << MTK_DIP_SCALE_SHIFT, in - 1);
}

/* First input column output pixel @a needs, even */
static s32 mtk_dip_scale_in_s(s32 a, u32 step)
{
	s64 x = 2 * ((s64)a << MTK_DIP_SCALE_SHIFT) + step - MTK_DIP_SCALE_ONE;

	if (x <= 0)
		return 0;

	return (s32)div64_u64(x, 2 * (u64)step) & ~1;
}

/* Last input column output pixel @e needs, odd unless the frame end */
static s32 mtk_dip_scale_in_e(s32 e, u32 step, s32 w)
{
	u64 x = MTK_DIP_SCALE_ONE + step + 2 * ((u64)e << MTK_DIP_SCALE_SHIFT);
	s32 v;

	if (x >= 2 * (u64)step * (w - 1))
		return w - 1;
	v = DIV_ROUND_UP_ULL(x, 2 * (u64)step) - 1;

	return v | 1;
}

/* Last output pixel the input columns up to @de serve */
static s32 mtk_dip_scale_last(s32 de, u32 step, s32 w, s32 sw)
{
	s32 e;

	if (de >= w - 1)
		return sw - 1;
	e = min_t(s64, sw - 1, ((u64)(de + 1) * step) >> MTK_DIP_SCALE_SHIFT);
	while (e >= 0 && mtk_dip_scale_in_e(e, step, w) > de)
		e--;
	while (e + 1 < sw && mtk_dip_scale_in_e(e + 1, step, w) <= de)
		e++;

	return e;
}

/*
 * Scaled output of tile @t: the pixels behind those of the tile before
 * (@prev_e) up to the last one its input serves; an even number, as the
 * write port takes YUYV pairs. @t->ctx and @t->out_e are set.
 */
static int mtk_dip_tile_scale(struct mtk_dip_tile *t, s32 w, s32 sw, u32 step,
			      s32 prev_e, bool last)
{
	s32 ds = t->out_s - t->ctx, e, first;
	u64 acc = (u64)ds * step;

	t->sc_s = prev_e + 1;
	if (last) {
		e = sw - 1;
	} else {
		e = mtk_dip_scale_last(t->out_e, step, w, sw);
		if ((e - t->sc_s + 1) & 1)
			e--;
	}
	/* at least one pair; the input of the first pixel is in the tile */
	if (e < t->sc_s + 1 || mtk_dip_scale_in_s(t->sc_s, step) < ds)
		return -ERANGE;
	t->sc_e = e;

	first = (acc + MTK_DIP_SCALE_ONE - 1) >> MTK_DIP_SCALE_SHIFT;
	if (first > t->sc_s)
		return -ERANGE;
	t->sc_skip = t->sc_s - first;
	t->sc_phase = (u32)-acc & (MTK_DIP_SCALE_ONE - 1);

	return 0;
}

/*
 * One tile with its output from @os, and from @os - @ctx behind the second
 * stripe store, to at most @end: the modules behind the chain (the scaler
 * limit and the temporal noise reduction) backwards, then the chain from the
 * own part of the second stripe store, forwards through all. -EAGAIN: a
 * module behind the chain would get more than it takes, a shorter tile may
 * work.
 */
static int mtk_dip_tile_try(s32 w, u32 profile, s32 os, s32 ctx, s32 end,
			    s32 prev_stripe_e, s32 prev_own_e,
			    struct mtk_dip_tile *t)
{
	bool nr3d = profile & MTK_DIP_PROFILE_NR3D;
	bool scale = profile & MTK_DIP_PROFILE_SCALE;
	struct mtk_dip_tile_range bw[MTK_DIP_C_NUM];
	s32 ds = os - ctx, de = end, ns, ne, s, e;

	if (scale && de - ds + 1 > MTK_DIP_SCALE_MAX_IN)
		de = ds + MTK_DIP_SCALE_MAX_IN - 1;
	ns = ds;
	ne = de;
	if (nr3d) {
		ns = ds > 0 ? ds - MTK_DIP_NR3D_LOSS_L : 0;
		ne = de < w - 1 ? min(de + MTK_DIP_NR3D_LOSS_R, w - 1) : w - 1;
		if (ne - ns + 1 > MTK_DIP_NR3D_MAX_IN)
			ne = ns + MTK_DIP_NR3D_MAX_IN - 1;
		if (ns < 0)
			return -ERANGE;
	}

	/* the chain computes the own part of the second stripe store */
	memset(bw, 0, sizeof(bw));
	mtk_dip_tile_back(w, profile, bw, prev_own_e >= 0 ? prev_own_e + 1 : ns,
			  ne, prev_stripe_e);
	if (mtk_dip_tile_fwd(w, profile, bw, t, prev_stripe_e))
		return -EAGAIN;
	s = t->r[MTK_DIP_C_NR2].out_s;
	e = t->r[MTK_DIP_C_NR2].out_e;
	if (prev_own_e >= 0 && s != prev_own_e + 1)
		return -ERANGE;
	t->own_s = s;
	t->own_e = e;
	t->s2_s = ns;

	if (nr3d) {
		if (e - ns + 1 > MTK_DIP_NR3D_MAX_IN)
			return -EAGAIN;
		s = ns > 0 ? ns + MTK_DIP_NR3D_LOSS_L : 0;
		e = e < w - 1 ? e - MTK_DIP_NR3D_LOSS_R : w - 1;
	} else {
		s = ns;
	}
	if (scale && e - s + 1 > MTK_DIP_SCALE_MAX_IN)
		return -EAGAIN;
	if (s != ds || e < os)
		return -ERANGE;
	t->out_s = os;
	t->out_e = e;
	t->ctx = ctx;

	return 0;
}

/*
 * Column ranges of all tiles for an output of @w columns, and with
 * MTK_DIP_PROFILE_SCALE of the scaled output of @sw columns
 */
static int mtk_dip_tile_columns(s32 w, u32 profile, s32 sw,
				struct mtk_dip_tile_layout *l)
{
	bool scale = profile & MTK_DIP_PROFILE_SCALE;
	bool stripe2 = profile & (MTK_DIP_PROFILE_SCALE | MTK_DIP_PROFILE_NR3D);
	s32 os = 0, prev_stripe_e = -1, prev_sc_e = -1, prev_own_e = -1;
	s32 ctx, end;
	struct mtk_dip_tile *t;
	unsigned int k = 0;
	u32 step = 0;
	int i, ret;

	if (w < 2)
		return -EINVAL;
	if (scale) {
		if (sw < 2 || sw & 1 || 2 * sw > w ||
		    sw * MTK_DIP_SCALE_MAX_DOWN < w)
			return -EINVAL;
		step = mtk_dip_scale_step(w, sw);
	}
	l->scale_x = step;

	while (os < w) {
		if (k == MTK_DIP_TILE_MAX)
			return -E2BIG;
		t = &l->tile[k];
		end = w - 1;
		ctx = 0;
		/*
		 * Columns in front of the output that the second stripe store
		 * hands on: those the next scaled pixel needs, at least the
		 * smallest stripe
		 */
		if (stripe2 && k) {
			ctx = MTK_DIP_SCALE_MIN_CTX;
			if (scale)
				ctx = max(ctx, os - mtk_dip_scale_in_s(prev_sc_e + 1,
								       step));
			if (os - ctx < 0)
				return -ERANGE;
		}
		if (scale)
			end = min(end, os - ctx + MTK_DIP_SCALE_MAX_IN - 1);
		/*
		 * With the second stripe store: a tile cut to a width limit
		 * that reached the frame end upstream lost no columns at its
		 * right edge, so a module gets more than it takes: end it
		 * earlier, the next tile takes the rest.
		 */
		for (;;) {
			ret = mtk_dip_tile_try(w, profile, os, ctx, end,
					       prev_stripe_e, prev_own_e, t);
			if (ret != -EAGAIN || !stripe2)
				break;
			end -= 2;
			if (end < os)
				return -ERANGE;
		}
		if (ret)
			return ret == -EAGAIN ? -ERANGE : ret;
		for (i = 0; i < MTK_DIP_C_NUM; i++)
			if (t->r[i].in_s < 0 || t->r[i].in_e >= w ||
			    t->r[i].in_s > t->r[i].in_e)
				return -ERANGE;
		if (scale) {
			ret = mtk_dip_tile_scale(t, w, sw, step, prev_sc_e,
						 t->out_e == w - 1);
			if (ret)
				return ret;
			prev_sc_e = t->sc_e;
		}
		prev_stripe_e = t->r[MTK_DIP_C_STRIPE].in_e;
		if (stripe2)
			prev_own_e = t->own_e;
		os = t->out_e + 1;
		k++;
	}
	l->n_tiles = k;

	return 0;
}

/**
 * mtk_dip_tile_width_ok - whether the tile rule serves an output width
 * @w: output width
 * @profile: module set, MTK_DIP_PROFILE_*, without the scaled output
 *
 * Only the edge enhancement changes the column ranges; the other optional
 * modules do not matter here.
 *
 * Return: 0 or a negative error code (-ERANGE: width in a blocked band).
 */
int mtk_dip_tile_width_ok(u32 w, u32 profile)
{
	struct mtk_dip_tile_layout *l;
	int ret;

	if (profile & MTK_DIP_PROFILE_SCALE)
		return -EINVAL;
	/* the layout is large, keep it off the stack */
	l = kzalloc_obj(*l, GFP_KERNEL);
	if (!l)
		return -ENOMEM;
	ret = mtk_dip_tile_columns(w, profile, 0, l);
	kfree(l);

	return ret;
}

/**
 * mtk_dip_tile_fit_width - largest even width up to @w the tile rule serves
 *	with and without edge enhancement
 * @w: wanted width
 * @min: smallest width to return
 *
 * The bands of the module set with edge enhancement contain those of the set
 * without it; a width outside them serves both, so the module set may change
 * from job to job.
 *
 * Return: the width, or 0 if none from @min to @w is served (or no memory).
 */
u32 mtk_dip_tile_fit_width(u32 w, u32 min)
{
	struct mtk_dip_tile_layout *l;
	u32 ret = 0;

	l = kzalloc_obj(*l, GFP_KERNEL);
	if (!l)
		return 0;
	for (w &= ~1U; w >= min && w >= 2; w -= 2) {
		if (!mtk_dip_tile_columns(w, 0, 0, l) &&
		    !mtk_dip_tile_columns(w, MTK_DIP_PROFILE_EE, 0, l)) {
			ret = w;
			break;
		}
	}
	kfree(l);

	return ret;
}

/**
 * mtk_dip_tile_fit_width_yuv - mtk_dip_tile_fit_width() for a YUV input
 * @w: wanted width
 * @min: smallest width to return
 *
 * Return: the width, or 0 if none from @min to @w is served (or no memory).
 */
u32 mtk_dip_tile_fit_width_yuv(u32 w, u32 min)
{
	struct mtk_dip_tile_layout *l;
	u32 ret = 0;

	l = kzalloc_obj(*l, GFP_KERNEL);
	if (!l)
		return 0;
	for (w &= ~1U; w >= min && w >= 2; w -= 2) {
		if (!mtk_dip_tile_columns(w, MTK_DIP_PROFILE_YUV, 0, l) &&
		    !mtk_dip_tile_columns(w, MTK_DIP_PROFILE_YUV |
					  MTK_DIP_PROFILE_EE, 0, l)) {
			ret = w;
			break;
		}
	}
	kfree(l);

	return ret;
}

/* Scaling the scaler serves on one axis: 1/2 down to 1/16, even sizes */
static bool mtk_dip_scale_axis_ok(u32 in, u32 out)
{
	return out >= 2 && !(out & 1) && 2 * out <= in &&
	       out * MTK_DIP_SCALE_MAX_DOWN >= in;
}

/**
 * mtk_dip_tile_scale_ok - whether the tile rule serves a scaled output
 * @w: width of the processed area
 * @h: height of the processed area
 * @sw: width of the scaled output
 * @sh: height of the scaled output
 * @profile: module set without MTK_DIP_PROFILE_SCALE and _NO_MAIN
 *
 * The main output may be on or off; the column ranges are the same.
 *
 * Return: 0, -EINVAL for a size the scaler does not serve, -ERANGE for a
 * combination of widths the tile rule cannot serve.
 */
int mtk_dip_tile_scale_ok(u32 w, u32 h, u32 sw, u32 sh, u32 profile)
{
	struct mtk_dip_tile_layout *l;
	int ret;

	if (!mtk_dip_scale_axis_ok(w, sw) || !mtk_dip_scale_axis_ok(h, sh))
		return -EINVAL;
	l = kzalloc_obj(*l, GFP_KERNEL);
	if (!l)
		return -ENOMEM;
	ret = mtk_dip_tile_columns(w, profile | MTK_DIP_PROFILE_SCALE, sw, l);
	kfree(l);

	return ret;
}

/**
 * mtk_dip_tile_fit_scale_width - largest even scaled width up to @sw the
 *	tile rule serves with and without edge enhancement
 * @w: width of the processed area
 * @h: height of the processed area
 * @sw: wanted scaled width
 * @sh: scaled height (served by the scaler)
 * @min: smallest width to return
 *
 * Return: the width, or 0 if none from @min to @sw is served (or no memory).
 */
u32 mtk_dip_tile_fit_scale_width(u32 w, u32 h, u32 sw, u32 sh, u32 min)
{
	for (sw &= ~1U; sw >= min && sw >= 2; sw -= 2)
		if (!mtk_dip_tile_scale_ok(w, h, sw, sh, 0) &&
		    !mtk_dip_tile_scale_ok(w, h, sw, sh, MTK_DIP_PROFILE_EE))
			return sw;

	return 0;
}

/* State of the word generation of one tile */
struct mtk_dip_tile_ctx {
	const struct mtk_dip_tile_geom *g;
	const struct mtk_dip_frame_cfg *f;
	const struct mtk_dip_tile *t;
	unsigned int k;
	unsigned int n;
	u32 edge;
	u32 *w;
	unsigned int len;
};

static void mtk_dip_put(struct mtk_dip_tile_ctx *c, u32 v)
{
	c->w[c->len++] = v;
}

/* Bit position of a pixel column in the packed input line */
static void mtk_dip_in_cols(const struct mtk_dip_tile_ctx *c, u32 *a, u32 *z)
{
	const struct mtk_dip_tile_range *r = &c->t->r[MTK_DIP_C_EDGE];

	*a = (c->g->x0 + r->in_s) * c->g->bpp;
	*z = (c->g->x0 + r->in_e + 1) * c->g->bpp;
}

/*
 * Raw input port: 4-byte aligned start, the rest of the start as a bit
 * offset in the unpacker; size up to the next 4-byte boundary.
 */
static void mtk_dip_w_in_raw(struct mtk_dip_tile_ctx *c)
{
	u32 a, z, off, end;

	mtk_dip_in_cols(c, &a, &z);
	off = (a >> 5) << 2;
	end = ((z + 31) >> 5) << 2;
	mtk_dip_put(c, off);
	mtk_dip_put(c, lo_hi(end - off - 1, c->g->h - 1));
}

static void mtk_dip_w_unpack(struct mtk_dip_tile_ctx *c)
{
	u32 a, z;

	mtk_dip_in_cols(c, &a, &z);
	mtk_dip_put(c, lo_hi(a & 31, z & 31));
}

/* Block of the shading grid for a frame position along one axis */
struct mtk_dip_lsc_win {
	u32 b0;
	u32 ofs;
	u32 n;
	u32 last;
};

static void mtk_dip_lsc_win(u32 a, u32 b, u32 bz, u32 nb, u32 lastsz,
			    struct mtk_dip_lsc_win *win)
{
	u32 b1;

	win->b0 = min(a / (2 * bz), nb);
	b1 = min(b / (2 * bz), nb);
	win->ofs = (a - win->b0 * 2 * bz) / 2;
	win->n = b1 - win->b0;
	win->last = b1 == nb ? lastsz : bz;
}

/* Shading: range = input of the companding stage */
static void mtk_dip_lsc(struct mtk_dip_tile_ctx *c, struct mtk_dip_lsc_win *x,
			struct mtk_dip_lsc_win *y, s32 *w)
{
	const struct mtk_dip_tile_range *r = &c->t->r[MTK_DIP_C_COMP];
	const struct mtk_dip_frame_cfg *f = c->f;

	mtk_dip_lsc_win(c->g->x0 + r->in_s, c->g->x0 + r->in_e, f->lsc_bw,
			f->lsc_nx, f->lsc_lw, x);
	mtk_dip_lsc_win(c->g->y0, c->g->y0 + c->g->h - 1, f->lsc_bh,
			f->lsc_ny, f->lsc_lh, y);
	*w = r->in_e - r->in_s + 1;
}

static void mtk_dip_w_in_lsc(struct mtk_dip_tile_ctx *c)
{
	struct mtk_dip_lsc_win x, y;
	s32 w;

	mtk_dip_lsc(c, &x, &y, &w);
	mtk_dip_put(c, x.b0 * MTK_DIP_LSC_BLOCK_BYTES + y.b0 * c->f->lsc_stride);
	mtk_dip_put(c, lo_hi((x.n + 1) * MTK_DIP_LSC_BLOCK_BYTES - 1, y.n));
}

static void mtk_dip_w_lsc(struct mtk_dip_tile_ctx *c)
{
	struct mtk_dip_lsc_win x, y;
	s32 w;

	mtk_dip_lsc(c, &x, &y, &w);
	mtk_dip_put(c, lo_hi(w, c->g->h));
	mtk_dip_put(c, lo_hi(x.ofs, y.ofs));
	mtk_dip_put(c, lo_hi(x.n, y.n));
	mtk_dip_put(c, lo_hi(x.last, y.last));
}

/* Statistics window of the upscaler for frame positions a..b along an axis */
struct mtk_dip_stat_win {
	u32 s;
	u32 n;
	u32 sub;
};

static void mtk_dip_stat_win(u32 a, u32 b, u32 step, u32 n,
			     struct mtk_dip_stat_win *win)
{
	u32 p = a * step, e;

	win->s = p >> 15;
	e = min(n - 1, ((b * step) >> 15) + 1);
	win->n = e - win->s + 1;
	win->sub = p & 0x7fff;
}

/* Upscaler: range = input of the first noise stage */
static void mtk_dip_stat(struct mtk_dip_tile_ctx *c, struct mtk_dip_stat_win *x,
			 struct mtk_dip_stat_win *y, s32 *w)
{
	const struct mtk_dip_tile_range *r = &c->t->r[MTK_DIP_C_NR1];
	const struct mtk_dip_tile_geom *g = c->g;
	const struct mtk_dip_frame_cfg *f = c->f;

	mtk_dip_stat_win(g->x0 + r->in_s, g->x0 + r->in_e, f->stat_sx,
			 f->stat_nx, x);
	if (!g->y0 && g->h == g->frame_h) {
		/* whole height: all rows, no offset */
		y->s = 0;
		y->n = f->stat_ny;
		y->sub = 0;
	} else {
		mtk_dip_stat_win(g->y0, g->y0 + g->h - 1, f->stat_sy,
				 f->stat_ny, y);
	}
	*w = r->in_e - r->in_s + 1;
}

static void mtk_dip_w_in_stat(struct mtk_dip_tile_ctx *c)
{
	struct mtk_dip_stat_win x, y;
	s32 w;

	mtk_dip_stat(c, &x, &y, &w);
	mtk_dip_put(c, 2 * x.s + y.s * c->f->stat_stride);
	mtk_dip_put(c, lo_hi(2 * x.n - 1, y.n - 1));
}

static void mtk_dip_w_stat_scaler(struct mtk_dip_tile_ctx *c)
{
	struct mtk_dip_stat_win x, y;
	s32 w;

	mtk_dip_stat(c, &x, &y, &w);
	mtk_dip_put(c, lo_hi(x.n, y.n));
	mtk_dip_put(c, lo_hi(w, c->g->h));
	mtk_dip_put(c, lo_hi(0, x.sub));
	mtk_dip_put(c, lo_hi(0, y.sub));
}

/* Radial weights: frame position times the module factor */
static void mtk_dip_w_radial(struct mtk_dip_tile_ctx *c, s32 s, s32 e)
{
	mtk_dip_put(c, (c->g->x0 + s) * c->f->radial_cx);
	mtk_dip_put(c, c->g->y0 * c->f->radial_cy);
	mtk_dip_put(c, lo_hi(e - s + 1, c->g->h));
}

/*
 * Stripe store: @comb columns, the first @left of them from the tile before,
 * the last @right of them for the next tile
 */
static void mtk_dip_w_stripe(struct mtk_dip_tile_ctx *c, s32 comb, s32 left,
			     s32 right)
{
	bool first = !c->k, last = c->k == c->n - 1;
	s32 own = comb - (first ? 0 : left);
	u32 h = c->g->h, ctl;

	/* left part from the previous tile */
	mtk_dip_put(c, first ? 0 : lo_hi(0, left - 1));
	mtk_dip_put(c, first ? 0 : lo_hi(0, h - 1));
	/* own part */
	mtk_dip_put(c, lo_hi(0, own - 1));
	mtk_dip_put(c, lo_hi(0, h - 1));
	/* right part for the next tile */
	mtk_dip_put(c, last ? 0 : lo_hi(comb - right, comb - 1));
	mtk_dip_put(c, last ? 0 : lo_hi(0, h - 1));
	ctl = MTK_DIP_STRIPE_CTL_BASE;
	if (!last)
		ctl |= MTK_DIP_STRIPE_CTL_OUT;
	if (!first)
		ctl |= MTK_DIP_STRIPE_CTL_IN;
	else
		ctl |= MTK_DIP_STRIPE_CTL_FIRST;
	mtk_dip_put(c, ctl);
	mtk_dip_put(c, first ? 0 : lo_hi(h, left));
	/*
	 * Tile k writes the half k mod 2 of the buffer, tile k + 1 reads it;
	 * the read address of the first tile (which reads nothing) is that of
	 * the odd half.
	 */
	mtk_dip_put(c, (c->k + 1) & 1 ? MTK_DIP_STRIPE_HALF : 0);
	mtk_dip_put(c, first ? 0 : left * 2 * h - 1);
	mtk_dip_put(c, 0);
	mtk_dip_put(c, last ? 0 : (c->k & 1 ? MTK_DIP_STRIPE_HALF : 0));
	mtk_dip_put(c, last ? 0 : right * 2 * h - 1);
	mtk_dip_put(c, 0);
	mtk_dip_put(c, 0);
}

/*
 * Window of the reference for the input columns of the temporal noise
 * reduction in a tile: the same columns and all lines, moved by the global
 * motion and cut to the reference. The read ports start at the moved
 * frame (their base addresses include the positive part of the motion);
 * @ox, @oy: where the window starts in the input of the module.
 */
struct mtk_dip_ref_win {
	s32 s;
	s32 e;
	s32 w;
	s32 h;
	s32 ox;
	s32 oy;
};

static void mtk_dip_ref_win(const struct mtk_dip_tile_ctx *c,
			    struct mtk_dip_ref_win *v)
{
	const struct mtk_dip_tile_geom *g = c->g;
	s32 dx = g->ref_dx, dy = g->ref_dy;
	s32 wv = g->w - max(dx, 0), hv = g->h - max(dy, 0);
	s32 s = c->t->s2_s + min(dx, 0), e = c->t->own_e + min(dx, 0);
	s32 ys = min(dy, 0), ye = g->h - 1 + min(dy, 0);

	v->ox = max(0, -s);
	v->s = max(0, s);
	v->e = min(wv - 1, e);
	v->w = v->e - v->s + 1;
	v->oy = max(0, -ys);
	ys = max(0, ys);
	ye = min(hv - 1, ye);
	v->h = ye - ys + 1;
}

/*
 * Window of the base image and of the weight map of the blend: the input
 * columns of the first noise reduction stage, all lines
 */
static void mtk_dip_mix_win(const struct mtk_dip_tile_ctx *c,
			    struct mtk_dip_ref_win *v)
{
	const struct mtk_dip_tile_range *r = &c->t->r[MTK_DIP_C_444TO422];

	v->s = r->out_s;
	v->e = r->out_e;
	v->w = v->e - v->s + 1;
	v->h = c->g->h;
	v->ox = 0;
	v->oy = 0;
}

static void mtk_dip_ref_or_mix_win(const struct mtk_dip_tile_ctx *c,
				   struct mtk_dip_ref_win *v)
{
	if (c->g->profile & MTK_DIP_PROFILE_MIX)
		mtk_dip_mix_win(c, v);
	else
		mtk_dip_ref_win(c, v);
}

/* YUV input: Y or UV port over the input columns of the tile */
static void mtk_dip_w_in_yuv(struct mtk_dip_tile_ctx *c, bool uv)
{
	const struct mtk_dip_tile_range *r = &c->t->r[MTK_DIP_C_IN];
	u32 h = uv ? c->g->h / 2 : c->g->h;

	mtk_dip_put(c, c->g->x0 + r->in_s);
	if (uv) {
		mtk_dip_put(c, r->in_e - r->in_s);
		mtk_dip_put(c, h - 1);
	} else {
		mtk_dip_put(c, lo_hi(r->in_e - r->in_s, h - 1));
	}
}

static void mtk_dip_w_unit(struct mtk_dip_tile_ctx *c, unsigned int u)
{
	struct mtk_dip_ref_win v;
	const struct mtk_dip_tile *t = c->t;
	const struct mtk_dip_tile_geom *g = c->g;
	const struct mtk_dip_tile_range *r;
	s32 ow = t->out_e - t->out_s + 1, s, e;

	switch (u) {
	case MTK_DIP_U_IN_RAW:
		if (g->profile & MTK_DIP_PROFILE_YUV)
			mtk_dip_w_in_yuv(c, false);
		else
			mtk_dip_w_in_raw(c);
		break;
	case MTK_DIP_U_IN_UV:
		mtk_dip_w_in_yuv(c, true);
		break;
	case MTK_DIP_U_WEIGHT:
		/* one byte per pixel */
		mtk_dip_mix_win(c, &v);
		mtk_dip_put(c, v.s);
		mtk_dip_put(c, v.w - 1);
		mtk_dip_put(c, v.h - 1);
		break;
	case MTK_DIP_U_IN_422:
		r = &t->r[MTK_DIP_C_420TO422];
		mtk_dip_put(c, c->edge);
		mtk_dip_put(c, lo_hi(0, r->in_e - r->in_s));
		mtk_dip_put(c, lo_hi(0, g->h - 1));
		break;
	case MTK_DIP_U_IN_444:
		mtk_dip_put(c, c->edge);
		break;
	case MTK_DIP_U_IN_LSC:
		mtk_dip_w_in_lsc(c);
		break;
	case MTK_DIP_U_IN_STAT:
		mtk_dip_w_in_stat(c);
		break;
	case MTK_DIP_U_REF_Y:
		mtk_dip_ref_or_mix_win(c, &v);
		mtk_dip_put(c, v.s);
		mtk_dip_put(c, lo_hi(v.w - 1, v.h - 1));
		break;
	case MTK_DIP_U_REF_UV:
		/* NV12: interleaved UV, the same bytes per line as Y */
		mtk_dip_ref_or_mix_win(c, &v);
		mtk_dip_put(c, v.s);
		mtk_dip_put(c, lo_hi(v.w - 1, v.h / 2 - 1));
		break;
	case MTK_DIP_U_NR3D:
		mtk_dip_ref_win(c, &v);
		mtk_dip_put(c, lo_hi(v.ox, v.oy));
		mtk_dip_put(c, lo_hi(v.w, v.h));
		mtk_dip_put(c, c->edge);
		mtk_dip_put(c, lo_hi(t->own_e - t->s2_s + 1, g->h));
		break;
	case MTK_DIP_U_NR3D_B:
		mtk_dip_put(c, c->edge);
		mtk_dip_put(c, lo_hi(t->own_e - t->s2_s + 1, g->h));
		break;
	case MTK_DIP_U_REF_422:
		/* edges of the reference window, not of the tile */
		mtk_dip_ref_or_mix_win(c, &v);
		mtk_dip_put(c, MTK_DIP_EDGE_TOP_BOTTOM |
			    (v.s ? 0 : MTK_DIP_EDGE_LEFT) |
			    (v.e == g->w - 1 ? MTK_DIP_EDGE_RIGHT : 0));
		mtk_dip_put(c, lo_hi(0, v.w - 1));
		mtk_dip_put(c, lo_hi(0, v.h - 1));
		break;
	case MTK_DIP_U_RADIAL_E:
		/* input of the temporal noise reduction */
		mtk_dip_w_radial(c, t->s2_s, t->own_e);
		break;
	case MTK_DIP_U_OUT_S:
		/* YUYV: two bytes per pixel */
		mtk_dip_put(c, 2 * t->sc_s);
		mtk_dip_put(c, lo_hi(2 * (t->sc_e - t->sc_s + 1) - 1, g->sh - 1));
		mtk_dip_put(c, 0);
		break;
	case MTK_DIP_U_OUT_Y:
		mtk_dip_put(c, t->out_s);
		mtk_dip_put(c, lo_hi(ow - 1, g->h - 1));
		mtk_dip_put(c, 0);
		break;
	case MTK_DIP_U_OUT_UV:
		/* NV12: interleaved UV, full width, half height */
		mtk_dip_put(c, t->out_s);
		mtk_dip_put(c, lo_hi(ow - 1, g->h / 2 - 1));
		mtk_dip_put(c, 0);
		break;
	case MTK_DIP_U_UNPACK:
		mtk_dip_w_unpack(c);
		break;
	case MTK_DIP_U_NOISE1:
	case MTK_DIP_U_NOISE2:
		/*
		 * window over the output of its noise stage, on a grid of 16
		 * columns, and the output inside it
		 */
		r = &t->r[u == MTK_DIP_U_NOISE1 ? MTK_DIP_C_NR1 : MTK_DIP_C_NR2];
		s = r->out_s / MTK_DIP_NOISE_GRID * MTK_DIP_NOISE_GRID;
		e = min_t(s32, g->w - 1, (r->out_e + MTK_DIP_NOISE_GRID) /
			  MTK_DIP_NOISE_GRID * MTK_DIP_NOISE_GRID - 1);
		mtk_dip_put(c, lo_hi(s, 0));
		mtk_dip_put(c, lo_hi(e - s + 1, g->h));
		mtk_dip_put(c, lo_hi(r->out_s - s, r->out_e - s));
		mtk_dip_put(c, lo_hi(0, g->h - 1));
		break;
	case MTK_DIP_U_DEMOSAIC:
	case MTK_DIP_U_422:
	case MTK_DIP_U_EE:
	case MTK_DIP_U_NR1:
	case MTK_DIP_U_RAWNR:
		mtk_dip_put(c, c->edge);
		break;
	case MTK_DIP_U_LSC:
		mtk_dip_w_lsc(c);
		break;
	case MTK_DIP_U_RADIAL_A:
		r = &t->r[MTK_DIP_C_DEMOSAIC];
		mtk_dip_w_radial(c, r->out_s, r->out_e);
		break;
	case MTK_DIP_U_RADIAL_B:
		r = &t->r[MTK_DIP_C_NR1];
		mtk_dip_w_radial(c, r->in_s, r->in_e);
		break;
	case MTK_DIP_U_RADIAL_C:
		r = &t->r[MTK_DIP_C_STRIPE];
		mtk_dip_w_radial(c, r->in_s, r->in_e);
		break;
	case MTK_DIP_U_RADIAL_D:
		/* edge enhancement input, 4 columns in at inner edges */
		r = &t->r[MTK_DIP_C_EE];
		s = r->in_s + (c->k ? 4 : 0);
		e = r->in_e - (c->k == c->n - 1 ? 0 : 4);
		mtk_dip_w_radial(c, s, e);
		break;
	case MTK_DIP_U_RADIAL_RAW:
		/* raw noise reduction input, 2 columns in at inner edges */
		r = &t->r[MTK_DIP_C_RAWNR];
		s = r->in_s + (c->k ? 2 : 0);
		e = r->in_e - (c->k == c->n - 1 ? 0 : 2);
		mtk_dip_w_radial(c, s, e);
		break;
	case MTK_DIP_U_RGB2YUV:
		/* its shading part is off */
		mtk_dip_put(c, 0);
		mtk_dip_put(c, 0);
		mtk_dip_put(c, 0);
		break;
	case MTK_DIP_U_SCALER:
		/* input: the tile output with the columns in front of it */
		mtk_dip_put(c, lo_hi(t->ctx + ow, g->h));
		mtk_dip_put(c, lo_hi(t->sc_e - t->sc_s + 1, g->sh));
		/* luma and chroma: pixels to skip, phase; no vertical offset */
		mtk_dip_put(c, t->sc_skip);
		mtk_dip_put(c, t->sc_skip);
		mtk_dip_put(c, t->sc_phase);
		mtk_dip_put(c, 0);
		mtk_dip_put(c, t->sc_phase);
		mtk_dip_put(c, 0);
		break;
	case MTK_DIP_U_420:
		/* cuts away the columns in front of the output (scaled output) */
		mtk_dip_put(c, lo_hi(MTK_DIP_420_OFFSET, ow));
		mtk_dip_put(c, lo_hi(g->h, t->ctx));
		mtk_dip_put(c, lo_hi(t->ctx + ow - 1, 0));
		mtk_dip_put(c, g->h - 1);
		break;
	case MTK_DIP_U_NR2:
		s = t->r[MTK_DIP_C_STRIPE].in_s;
		mtk_dip_put(c, ((g->x0 + s) & 3) | (c->edge << 28));
		break;
	case MTK_DIP_U_STAT_SCALER:
		mtk_dip_w_stat_scaler(c);
		break;
	case MTK_DIP_U_STRIPE:
		r = &t->r[MTK_DIP_C_STRIPE];
		mtk_dip_w_stripe(c, r->in_e - r->in_s + 1, MTK_DIP_STRIPE_W,
				 MTK_DIP_STRIPE_W);
		break;
	case MTK_DIP_U_STRIPE2:
		/* own part from the chain, the columns in front from the tile before */
		mtk_dip_w_stripe(c, t->own_e - t->s2_s + 1, t->own_s - t->s2_s,
				 c->k == c->n - 1 ? 0 : t[1].own_s - t[1].s2_s);
		break;
	case MTK_DIP_U_STRIPE_IN:
	case MTK_DIP_U_STRIPE_OUT:
	case MTK_DIP_U_STRIPE2_IN:
	case MTK_DIP_U_STRIPE2_OUT:
		/* in the 15 words of the stripe store */
		break;
	case MTK_DIP_U_EDGE:
		r = &t->r[MTK_DIP_C_EDGE];
		mtk_dip_put(c, lo_hi((g->x0 + r->in_s) & 2 ?
				     MTK_DIP_EDGE_PHASE_INV : 0, c->edge));
		break;
	case MTK_DIP_U_DECOMP:
	case MTK_DIP_U_COMP:
		r = &t->r[u == MTK_DIP_U_DECOMP ? MTK_DIP_C_DECOMP :
			  MTK_DIP_C_COMP];
		mtk_dip_put(c, lo_hi(g->h, r->in_e - r->in_s + 1));
		mtk_dip_put(c, c->edge);
		break;
	}
}

/* Header and module words 1-4 */
static void mtk_dip_w_header(struct mtk_dip_tile_ctx *c, u32 *hdr)
{
	const struct mtk_dip_unit_desc *d;
	u32 mod[MTK_DIP_HDR_WORDS_N] = {};
	unsigned int i, u;
	u32 edge;

	for (i = 0; i < ARRAY_SIZE(mtk_dip_profile); i++) {
		u = mtk_dip_profile[i];
		d = &mtk_dip_units[u];
		if (!mtk_dip_unit_on(c->g, u))
			continue;
		/* no stripe read in the first tile, no write in the last */
		if ((u == MTK_DIP_U_STRIPE_IN || u == MTK_DIP_U_STRIPE2_IN) &&
		    !c->k)
			continue;
		if ((u == MTK_DIP_U_STRIPE_OUT || u == MTK_DIP_U_STRIPE2_OUT) &&
		    c->k == c->n - 1)
			continue;
		mod[d->hdr_word] |= BIT(d->hdr_bit);
	}
	for (i = 0; i < ARRAY_SIZE(mtk_dip_hdr_only); i++) {
		u32 p = c->g->profile;

		if ((p & mtk_dip_hdr_only[i].profile) !=
		    mtk_dip_hdr_only[i].profile || p & mtk_dip_hdr_only[i].off)
			continue;
		mod[mtk_dip_hdr_only[i].word] |= BIT(mtk_dip_hdr_only[i].bit);
	}

	/* edge fields: none with a YUV input */
	edge = c->g->profile & MTK_DIP_PROFILE_YUV ? 0 : c->edge;
	hdr[0] = FIELD_PREP(MTK_DIP_HDR_RAW_EDGE, edge) | MTK_DIP_HDR_FIXED |
		 FIELD_PREP(MTK_DIP_HDR_PIX_ID, c->g->pix_id) |
		 FIELD_PREP(MTK_DIP_HDR_OUT_EDGE, edge) |
		 FIELD_PREP(MTK_DIP_HDR_WORDS, c->len - 1);
	for (i = 1; i < MTK_DIP_HDR_WORDS_N; i++)
		hdr[i] = mod[i];
}

static int mtk_dip_tile_geom_check(const struct mtk_dip_tile_geom *g)
{
	if ((g->bpp != 8 && g->bpp != 10) || g->pix_id > 3 ||
	    g->profile & ~MTK_DIP_PROFILE_ALL)
		return -EINVAL;
	if (g->profile & MTK_DIP_PROFILE_NO_MAIN &&
	    !(g->profile & MTK_DIP_PROFILE_SCALE))
		return -EINVAL;
	if (g->profile & MTK_DIP_PROFILE_NR3D &&
	    (g->profile & MTK_DIP_PROFILE_NO_MAIN || g->ref_stride < g->w ||
	     g->ref_dx & 1 || g->ref_dy & 1 ||
	     abs(g->ref_dx) > MTK_DIP_NR3D_DXY_MAX ||
	     abs(g->ref_dy) > MTK_DIP_NR3D_DXY_MAX))
		return -EINVAL;
	if (g->profile & MTK_DIP_PROFILE_SCALE &&
	    (!mtk_dip_scale_axis_ok(g->w, g->sw) ||
	     !mtk_dip_scale_axis_ok(g->h, g->sh) || g->s_stride < 2 * g->sw))
		return -EINVAL;
	if (g->profile & MTK_DIP_PROFILE_YUV &&
	    (g->bpp != 8 || g->pix_id || g->profile &
	     (MTK_DIP_PROFILE_LSC | MTK_DIP_PROFILE_STAT |
	      MTK_DIP_PROFILE_SCALE | MTK_DIP_PROFILE_NO_MAIN |
	      MTK_DIP_PROFILE_NR3D)))
		return -EINVAL;
	if (g->profile & MTK_DIP_PROFILE_MIX &&
	    (!(g->profile & MTK_DIP_PROFILE_YUV) || g->ref_stride < g->w ||
	     g->wt_stride < g->w))
		return -EINVAL;
	if (!g->w || !g->h || g->h & 1 || g->w & 1 || g->h > 8190)
		return -EINVAL;
	if (g->x0 + g->w > g->frame_w || g->y0 + g->h > g->frame_h ||
	    g->frame_w > 8191 || g->frame_h > 8191)
		return -EINVAL;
	if (g->in_stride < DIV_ROUND_UP(g->frame_w * g->bpp, 8))
		return -EINVAL;
	/* without the main output nothing reads its line size */
	if (!(g->profile & MTK_DIP_PROFILE_NO_MAIN) && g->out_stride < g->w)
		return -EINVAL;

	return 0;
}

/**
 * mtk_dip_tile_check - whether each job of a geometry gets a tile layout
 * @g: geometry; its MTK_DIP_PROFILE_EE flag does not matter
 *
 * The parameters may switch the edge enhancement on and off from job to job:
 * the geometry is checked with and without it. The other optional modules
 * change neither the column ranges nor the limits.
 *
 * Return: 0 or the error mtk_dip_tile_compute() returns for the geometry.
 */
int mtk_dip_tile_check(const struct mtk_dip_tile_geom *g)
{
	struct mtk_dip_tile_geom t = *g;
	struct mtk_dip_tile_layout *l;
	unsigned int i;
	int ret = 0;

	l = kzalloc_obj(*l, GFP_KERNEL);
	if (!l)
		return -ENOMEM;
	for (i = 0; i < 2 && !ret; i++) {
		if (i)
			t.profile |= MTK_DIP_PROFILE_EE;
		else
			t.profile &= ~MTK_DIP_PROFILE_EE;
		ret = mtk_dip_tile_geom_check(&t);
		if (!ret)
			ret = mtk_dip_tile_columns(t.w, t.profile, t.sw, l);
	}
	kfree(l);

	return ret;
}

/* Bytes behind the base address each port reads or writes */
static void mtk_dip_tile_needs(const struct mtk_dip_tile_geom *g,
			       const struct mtk_dip_frame_cfg *f,
			       struct mtk_dip_tile_layout *l, const u32 *b)
{
	unsigned int i, u, p = MTK_DIP_HDR_WORDS_N;
	const u32 *v;

	for (i = 0; i < ARRAY_SIZE(mtk_dip_profile); i++) {
		u = mtk_dip_profile[i];
		if (!mtk_dip_unit_on(g, u))
			continue;
		v = b + p;
		p += mtk_dip_units[u].words;
		switch (u) {
		case MTK_DIP_U_IN_RAW:
			l->need_in = max(l->need_in, v[0] + (v[1] & 0xffff) +
					 1 + (g->h - 1) * g->in_stride);
			break;
		case MTK_DIP_U_OUT_Y:
			l->need_y = max(l->need_y, v[0] + (v[1] & 0xffff) +
					1 + (g->h - 1) * g->out_stride);
			break;
		case MTK_DIP_U_OUT_UV:
			l->need_uv = max(l->need_uv, v[0] + (v[1] & 0xffff) +
					 1 + (g->h / 2 - 1) * g->out_stride);
			break;
		case MTK_DIP_U_IN_LSC:
			l->need_lsc = max(l->need_lsc, v[0] + (v[1] & 0xffff) +
					  1 + (v[1] >> 16) * f->lsc_stride);
			break;
		case MTK_DIP_U_IN_STAT:
			l->need_stat = max(l->need_stat, v[0] +
					   (v[1] & 0xffff) + 1 +
					   (v[1] >> 16) * f->stat_stride);
			break;
		case MTK_DIP_U_OUT_S:
			l->need_s = max(l->need_s, v[0] + (v[1] & 0xffff) +
					1 + (g->sh - 1) * g->s_stride);
			break;
		case MTK_DIP_U_REF_Y:
			l->need_ref = max(l->need_ref, v[0] + (v[1] & 0xffff) +
					  1 + (v[1] >> 16) * g->ref_stride);
			break;
		case MTK_DIP_U_REF_UV:
			l->need_ref_uv = max(l->need_ref_uv, v[0] +
					     (v[1] & 0xffff) + 1 +
					     (v[1] >> 16) * g->ref_stride);
			break;
		case MTK_DIP_U_IN_UV:
			l->need_in_uv = max(l->need_in_uv, v[0] + v[1] + 1 +
					    v[2] * g->in_stride);
			break;
		case MTK_DIP_U_WEIGHT:
			l->need_wt = max(l->need_wt, v[0] + v[1] + 1 +
					 v[2] * g->wt_stride);
			break;
		case MTK_DIP_U_STRIPE:
			/* read port at word 8, write port at word 11 */
			if (v[9])
				l->need_stripe = max(l->need_stripe,
						     v[8] + v[9] + 1);
			if (v[12])
				l->need_stripe = max(l->need_stripe,
						     v[11] + v[12] + 1);
			break;
		case MTK_DIP_U_STRIPE2:
			if (v[9])
				l->need_stripe2 = max(l->need_stripe2,
						      v[8] + v[9] + 1);
			if (v[12])
				l->need_stripe2 = max(l->need_stripe2,
						      v[11] + v[12] + 1);
			break;
		}
	}
}

/**
 * mtk_dip_tile_compute - tiles and tile descriptions of a job
 * @g: geometry
 * @f: frame grids, see mtk_dip_frame_cfg_init()
 * @l: result: tiles, module enable registers, buffer needs
 * @tdr: tile descriptions of all tiles, one block behind the other
 * @tdr_max_words: room in @tdr
 *
 * Return: 0, -EINVAL for a geometry outside the limits, -ERANGE for an output
 * width the tile rule cannot serve, -E2BIG for too many tiles or words.
 */
int mtk_dip_tile_compute(const struct mtk_dip_tile_geom *g,
			 const struct mtk_dip_frame_cfg *f,
			 struct mtk_dip_tile_layout *l, u32 *tdr,
			 unsigned int tdr_max_words)
{
	struct mtk_dip_tile_ctx c = { .g = g, .f = f };
	unsigned int k, i, u, words, pos = 0;
	int ret;

	memset(l, 0, sizeof(*l));
	ret = mtk_dip_tile_geom_check(g);
	if (ret)
		return ret;
	ret = mtk_dip_tile_columns(g->w, g->profile, g->sw, l);
	if (ret)
		return ret;
	if (g->profile & MTK_DIP_PROFILE_SCALE)
		l->scale_y = mtk_dip_scale_step(g->h, g->sh);
	/* the global motion leaves each tile a part of the reference */
	for (k = 0; k < l->n_tiles && g->profile & MTK_DIP_PROFILE_NR3D; k++) {
		struct mtk_dip_ref_win v;

		c.t = &l->tile[k];
		mtk_dip_ref_win(&c, &v);
		if (v.w < 2 || v.h < 2)
			return -ERANGE;
	}

	words = MTK_DIP_HDR_WORDS_N;
	for (i = 0; i < ARRAY_SIZE(mtk_dip_profile); i++) {
		const struct mtk_dip_unit_desc *d =
			&mtk_dip_units[mtk_dip_profile[i]];

		if (!mtk_dip_unit_on(g, mtk_dip_profile[i]))
			continue;
		l->tcm[d->reg] |= BIT(d->reg_bit);
		words += d->words;
	}

	c.n = l->n_tiles;
	for (k = 0; k < l->n_tiles; k++) {
		if (pos + MTK_DIP_TILE_WORDS_MAX > tdr_max_words)
			return -E2BIG;
		c.t = &l->tile[k];
		c.k = k;
		c.edge = MTK_DIP_EDGE_TOP_BOTTOM;
		if (!k)
			c.edge |= MTK_DIP_EDGE_LEFT;
		if (k == l->n_tiles - 1)
			c.edge |= MTK_DIP_EDGE_RIGHT;
		c.w = tdr + pos;
		c.len = MTK_DIP_HDR_WORDS_N;
		for (i = 0; i < ARRAY_SIZE(mtk_dip_profile); i++) {
			unsigned int start = c.len;

			u = mtk_dip_profile[i];
			if (!mtk_dip_unit_on(g, u))
				continue;
			mtk_dip_w_unit(&c, u);
			/* a stripe block holds the words of its two ports */
			if (u == MTK_DIP_U_STRIPE || u == MTK_DIP_U_STRIPE2)
				start += mtk_dip_units[u + 1].words +
					 mtk_dip_units[u + 2].words;
			else if (u == MTK_DIP_U_STRIPE_IN ||
				 u == MTK_DIP_U_STRIPE_OUT ||
				 u == MTK_DIP_U_STRIPE2_IN ||
				 u == MTK_DIP_U_STRIPE2_OUT)
				start -= mtk_dip_units[u].words;
			if (c.len - start != mtk_dip_units[u].words)
				return -EINVAL;
		}
		if (c.len != words)
			return -EINVAL;
		mtk_dip_w_header(&c, c.w);
		mtk_dip_tile_needs(g, f, l, c.w);
		pos += c.len;
	}
	l->words = words;
	l->tdr_bytes = pos * 4;

	return 0;
}
