/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * MediaTek MT6785 ISP pass 2 (DIP): tile computation.
 *
 * The DIP processes a frame as a row of vertical tiles, one after the other.
 * Each tile is described by a block of 32-bit words (the tile description
 * the TDRI port reads) that gives every enabled module the column range it
 * works on. The block layout and the rules below describe the behaviour of
 * the MT6785 hardware.
 */

#ifndef _MTK_DIP_TILE_H
#define _MTK_DIP_TILE_H

#include <linux/types.h>

#define MTK_DIP_TILE_MAX		32
/* the hardware reads at most 256 words per tile */
#define MTK_DIP_TILE_WORDS_MAX		256

/*
 * Modules of the processing chain that change the column range, input to
 * output. The edge enhancement is part of the chain only in profiles with
 * MTK_DIP_PROFILE_EE; with MTK_DIP_PROFILE_YUV the two chroma converters of
 * the YUV input take the place of the raw stages (border pre-stage to
 * demosaicing).
 */
enum mtk_dip_chain_mod {
	MTK_DIP_C_IN,		/* raw input port */
	MTK_DIP_C_EDGE,		/* border pre-stage */
	MTK_DIP_C_DECOMP,	/* decompanding */
	MTK_DIP_C_COMP,		/* companding */
	MTK_DIP_C_RAWNR,	/* raw noise reduction */
	MTK_DIP_C_DEMOSAIC,	/* demosaicing */
	MTK_DIP_C_420TO422,	/* YUV input: chroma 4:2:0 to 4:2:2 */
	MTK_DIP_C_422TO444,	/* YUV input: chroma 4:2:2 to 4:4:4 */
	MTK_DIP_C_444TO422,	/* chroma subsampling 4:4:4 to 4:2:2 */
	MTK_DIP_C_NR1,		/* luma/chroma noise reduction, first stage */
	MTK_DIP_C_EE,		/* edge enhancement */
	MTK_DIP_C_STRIPE,	/* stripe store: right edge to the next tile */
	MTK_DIP_C_NR2,		/* luma/chroma noise reduction, second stage */
	MTK_DIP_C_NUM,
};

/**
 * struct mtk_dip_frame_cfg - frame registers the tile words depend on
 * @lsc_bw: shading grid block width in 2x2 cells
 * @lsc_bh: shading grid block height in 2x2 cells
 * @lsc_nx: shading grid blocks per row minus one
 * @lsc_ny: shading grid blocks per column minus one
 * @lsc_lw: width of the last block of a row in 2x2 cells
 * @lsc_lh: height of the last block of a column in 2x2 cells
 * @lsc_stride: bytes per row of blocks in the shading table
 * @stat_nx: columns of the statistics grid of the upscaler
 * @stat_ny: rows of the statistics grid of the upscaler
 * @stat_sx: horizontal grid step per pixel, Q15
 * @stat_sy: vertical grid step per pixel, Q15
 * @stat_stride: bytes per row of the statistics grid
 * @radial_cx: horizontal factor of the radial weight modules
 * @radial_cy: vertical factor of the radial weight modules
 *
 * All positions in the tile words are positions in the input frame; these
 * values describe grids over the whole input frame.
 */
struct mtk_dip_frame_cfg {
	u32 lsc_bw;
	u32 lsc_bh;
	u32 lsc_nx;
	u32 lsc_ny;
	u32 lsc_lw;
	u32 lsc_lh;
	u32 lsc_stride;
	u32 stat_nx;
	u32 stat_ny;
	u32 stat_sx;
	u32 stat_sy;
	u32 stat_stride;
	u32 radial_cx;
	u32 radial_cy;
};

/*
 * Bayer phase of the DIP: colour of the top left pixel of the frame, 0 blue,
 * 1 green of a blue row, 2 green of a red row, 3 red, as the vendor sensor
 * drivers number the first pixel. Bit 0 shifts the phase by one column, bit
 * 1 by one row: the pixel at raster position p of a 2x2 cell (0 top left,
 * 1 top right, 2 bottom left, 3 bottom right) has colour p ^ pix_id.
 */
#define MTK_DIP_PIX_ID_BGGR		0
#define MTK_DIP_PIX_ID_GBRG		1
#define MTK_DIP_PIX_ID_GRBG		2
#define MTK_DIP_PIX_ID_RGGB		3

/**
 * struct mtk_dip_tile_geom - geometry of one job
 * @frame_w: input frame width in pixels
 * @frame_h: input frame height in pixels
 * @x0: left edge of the processed area in the input frame
 * @y0: top edge of the processed area in the input frame
 * @w: width of the processed area = output width
 * @h: height of the processed area = output height
 * @bpp: bits per pixel of the packed input (8 or 10)
 * @pix_id: Bayer phase of the top left pixel of the frame, MTK_DIP_PIX_ID_*;
 *	the crop keeps it (even left and top)
 * @in_stride: bytes per input line
 * @out_stride: bytes per line of the Y and of the UV plane; not used with
 *	MTK_DIP_PROFILE_NO_MAIN
 * @profile: optional modules, MTK_DIP_PROFILE_*
 * @sw: width of the scaled output (MTK_DIP_PROFILE_SCALE)
 * @sh: height of the scaled output (MTK_DIP_PROFILE_SCALE)
 * @s_stride: bytes per line of the scaled output (MTK_DIP_PROFILE_SCALE)
 * @ref_stride: bytes per line of the reference (MTK_DIP_PROFILE_NR3D), of
 *	the base image (MTK_DIP_PROFILE_MIX)
 * @ref_dx: global motion, horizontal: position of a pixel in the reference
 *	minus its position in this frame, even (MTK_DIP_PROFILE_NR3D)
 * @ref_dy: global motion, vertical, even (MTK_DIP_PROFILE_NR3D)
 * @wt_stride: bytes per line of the weight map (MTK_DIP_PROFILE_MIX)
 *
 * With MTK_DIP_PROFILE_YUV the input is an NV12 frame: @bpp is 8, @pix_id 0,
 * @in_stride the bytes per line of its Y and of its UV plane.
 */
struct mtk_dip_tile_geom {
	u32 frame_w;
	u32 frame_h;
	u32 x0;
	u32 y0;
	u32 w;
	u32 h;
	u32 bpp;
	u32 pix_id;
	u32 in_stride;
	u32 out_stride;
	u32 profile;
	u32 sw;
	u32 sh;
	u32 s_stride;
	u32 ref_stride;
	s32 ref_dx;
	s32 ref_dy;
	u32 wt_stride;
};

/*
 * Optional modules on top of the base profile (Bayer to NV12 with black
 * level, white balance, raw noise reduction, demosaicing, colour matrix,
 * gamma and two luma/chroma noise reduction stages):
 * MTK_DIP_PROFILE_LSC: lens shading with its table port
 * MTK_DIP_PROFILE_STAT: statistics upscaler with its port, feeding a gain map
 *	to the first noise reduction stage
 * MTK_DIP_PROFILE_EE: edge enhancement with its radial weights; it changes
 *	the column ranges of the tiles
 * MTK_DIP_PROFILE_SCALE: the scaled output: the scaler with its write port
 *	(YUYV) behind a second stripe store, which hands each tile the columns
 *	in front of its output that the scaler needs, and the noise generators
 *	of the two noise reduction stages; it changes the column ranges of the
 *	tiles
 * MTK_DIP_PROFILE_NO_MAIN: with MTK_DIP_PROFILE_SCALE only: the main output
 *	(4:2:0 converter, Y and UV write ports, plane writer) is off; it does
 *	not change the column ranges of the tiles
 * MTK_DIP_PROFILE_NR3D: the temporal noise reduction behind the second
 *	stripe store, with its two read ports for the reference (NV12), the
 *	4:2:0 to 4:2:2 converter of the reference and its radial weights; it
 *	changes the column ranges of the tiles. Not with
 *	MTK_DIP_PROFILE_NO_MAIN: its output is the reference of the next job.
 * MTK_DIP_PROFILE_YUV: the input is an NV12 frame (Y and UV read ports, the
 *	4:2:0 to 4:2:2 and 4:2:2 to 4:4:4 converters); the raw stages (black
 *	level to RGB to YUV) are off. With the main output, edge enhancement
 *	and MTK_DIP_PROFILE_MIX only.
 * MTK_DIP_PROFILE_MIX: with MTK_DIP_PROFILE_YUV only: the blend of the
 *	input with a base image (NV12, two read ports and a 4:2:0 to 4:2:2
 *	converter) by a weight map (8 bits per pixel, one read port) behind the
 *	4:4:4 to 4:2:2 converter; it does not change the column ranges.
 */
#define MTK_DIP_PROFILE_LSC		BIT(0)
#define MTK_DIP_PROFILE_STAT		BIT(1)
#define MTK_DIP_PROFILE_EE		BIT(2)
#define MTK_DIP_PROFILE_SCALE		BIT(3)
#define MTK_DIP_PROFILE_NO_MAIN		BIT(4)
#define MTK_DIP_PROFILE_NR3D		BIT(5)
#define MTK_DIP_PROFILE_YUV		BIT(6)
#define MTK_DIP_PROFILE_MIX		BIT(7)
#define MTK_DIP_PROFILE_ALL		(MTK_DIP_PROFILE_LSC | \
					 MTK_DIP_PROFILE_STAT | \
					 MTK_DIP_PROFILE_EE | \
					 MTK_DIP_PROFILE_SCALE | \
					 MTK_DIP_PROFILE_NO_MAIN | \
					 MTK_DIP_PROFILE_NR3D | \
					 MTK_DIP_PROFILE_YUV | \
					 MTK_DIP_PROFILE_MIX)

/*
 * Scaler: it takes at most MTK_DIP_SCALE_MAX_IN columns per tile and makes
 * each output pixel from the input pixels it covers (source accumulation),
 * which serves down-scaling by 1/2 down to 1/MTK_DIP_SCALE_MAX_DOWN on each
 * axis. Positions in Q20.
 */
#define MTK_DIP_SCALE_MAX_IN		656
#define MTK_DIP_SCALE_MAX_DOWN		16
#define MTK_DIP_SCALE_SHIFT		20
/* smallest stripe the second stripe store hands on */
#define MTK_DIP_SCALE_MIN_CTX		24

/*
 * Temporal noise reduction: columns it loses at an inner tile edge on the
 * left and on the right, largest input width; largest global motion in
 * pixels, either direction. With a further module in front of it the pair
 * loses 8 columns on each side; without that module the output of a tile
 * starts 6 columns behind its input (with 8, every tile but the first is
 * shifted 2 columns to the right). On the right, 8 keep the columns the next
 * modules count on.
 */
#define MTK_DIP_NR3D_LOSS_L		6
#define MTK_DIP_NR3D_LOSS_R		8
#define MTK_DIP_NR3D_MAX_IN		656
#define MTK_DIP_NR3D_DXY_MAX		64

/**
 * struct mtk_dip_tile_range - column range of a module in one tile
 * @in_s: first input column
 * @in_e: last input column
 * @out_s: first output column
 * @out_e: last output column
 *
 * For the stripe store, @in_s..@in_e is the combined range (stripe from the
 * previous tile plus own part) and @out_s the first column of the own part.
 */
struct mtk_dip_tile_range {
	s32 in_s;
	s32 in_e;
	s32 out_s;
	s32 out_e;
};

/**
 * struct mtk_dip_tile - one tile
 * @r: range of each chain module, indexed by enum mtk_dip_chain_mod
 * @out_s: first output column
 * @out_e: last output column
 * @ctx: columns in front of @out_s that the second stripe store hands on
 *	(scaled output only, 0 in the first tile)
 * @sc_s: first column of the scaled output
 * @sc_e: last column of the scaled output
 * @sc_skip: outputs the scaler makes from the columns @out_s - @ctx on
 *	before @sc_s, not written
 * @sc_phase: position of @out_s - @ctx in output pixels, fraction, Q20
 *	(distance to the next output pixel)
 * @own_s: first output column of the chain (r[MTK_DIP_C_NR2]); with the
 *	second stripe store the first column of its own part
 * @own_e: last output column of the chain
 * @s2_s: first column of the second stripe store (input of the temporal
 *	noise reduction); its columns up to @own_s come from the tile before
 */
struct mtk_dip_tile {
	struct mtk_dip_tile_range r[MTK_DIP_C_NUM];
	s32 out_s;
	s32 out_e;
	s32 ctx;
	s32 sc_s;
	s32 sc_e;
	u32 sc_skip;
	u32 sc_phase;
	s32 own_s;
	s32 own_e;
	s32 s2_s;
};

/**
 * struct mtk_dip_tile_layout - result of the tile computation
 * @n_tiles: number of tiles
 * @words: words per tile
 * @tcm: the three module enable registers of the tile walker (TCM)
 * @need_in: bytes the raw input port reads from its base address
 * @need_y: bytes the Y port writes from its base address
 * @need_uv: bytes the UV port writes from its base address
 * @need_lsc: bytes the shading table port reads
 * @need_stat: bytes the statistics port reads
 * @need_stripe: bytes the stripe store reads and writes from its base
 * @need_s: bytes the scaled output port writes from its base address
 * @need_stripe2: bytes the second stripe store reads and writes from its base
 * @need_ref: bytes the reference Y port reads from its base address
 * @need_ref_uv: bytes the reference UV port reads from its base address
 * @need_in_uv: bytes the UV input port reads (MTK_DIP_PROFILE_YUV)
 * @need_wt: bytes the weight map port reads (MTK_DIP_PROFILE_MIX)
 * @scale_x: horizontal step of the scaler, output pixels per input pixel, Q20
 * @scale_y: vertical step of the scaler, Q20
 * @tdr_bytes: size of the tile descriptions of all tiles
 * @tile: the tiles
 */
struct mtk_dip_tile_layout {
	u32 n_tiles;
	u32 words;
	u32 tcm[3];
	u32 need_in;
	u32 need_y;
	u32 need_uv;
	u32 need_lsc;
	u32 need_stat;
	u32 need_stripe;
	u32 need_s;
	u32 need_stripe2;
	u32 need_ref;
	u32 need_ref_uv;
	u32 need_in_uv;
	u32 need_wt;
	u32 scale_x;
	u32 scale_y;
	u32 tdr_bytes;
	struct mtk_dip_tile tile[MTK_DIP_TILE_MAX];
};

/* offset of the second half of the stripe store buffer */
#define MTK_DIP_STRIPE_HALF		0x1f4004

/*
 * Shading table: MTK_DIP_LSC_BLOCKS x MTK_DIP_LSC_BLOCKS blocks over the
 * input frame, one row of blocks per table line, MTK_DIP_LSC_BLOCK_BYTES
 * per block; gains at the MTK_DIP_LSC_CORNERS x MTK_DIP_LSC_CORNERS block
 * corners for each of the four Bayer positions.
 */
#define MTK_DIP_LSC_BLOCKS		16
#define MTK_DIP_LSC_CORNERS		(MTK_DIP_LSC_BLOCKS + 1)
#define MTK_DIP_LSC_BLOCK_BYTES		96
#define MTK_DIP_LSC_TABLE_SIZE		(MTK_DIP_LSC_BLOCKS * \
					 MTK_DIP_LSC_BLOCKS * \
					 MTK_DIP_LSC_BLOCK_BYTES)

void mtk_dip_frame_cfg_init(struct mtk_dip_frame_cfg *f, u32 frame_w,
			    u32 frame_h);
int mtk_dip_lsc_table(const u16 *gain, u32 pix_id,
		      const struct mtk_dip_frame_cfg *f, void *table);
int mtk_dip_tile_compute(const struct mtk_dip_tile_geom *g,
			 const struct mtk_dip_frame_cfg *f,
			 struct mtk_dip_tile_layout *l, u32 *tdr,
			 unsigned int tdr_max_words);
int mtk_dip_tile_check(const struct mtk_dip_tile_geom *g);
int mtk_dip_tile_width_ok(u32 w, u32 profile);
u32 mtk_dip_tile_fit_width(u32 w, u32 min);
u32 mtk_dip_tile_fit_width_yuv(u32 w, u32 min);
int mtk_dip_tile_scale_ok(u32 w, u32 h, u32 sw, u32 sh, u32 profile);
u32 mtk_dip_tile_fit_scale_width(u32 w, u32 h, u32 sw, u32 sh, u32 min);

#endif /* _MTK_DIP_TILE_H */
