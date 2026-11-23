/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * MediaTek MT6785 multi-frame blending block (MFB): register values and tile
 * descriptions of one blend, computed without the hardware.
 */

#ifndef _MTK_MFB_JOB_H
#define _MTK_MFB_JOB_H

#include <linux/types.h>

/* sizes the driver accepts */
#define MTK_MFB_MIN_W			64
#define MTK_MFB_MIN_H			32
#define MTK_MFB_MAX_W			4624
#define MTK_MFB_MAX_H			3472
/* the width is a multiple of the input alignment of the blending core */
#define MTK_MFB_W_STEP			4
#define MTK_MFB_H_STEP			2

/* words of the description of one tile, and the most tiles of a frame */
#define MTK_MFB_TDR_WORDS		40
#define MTK_MFB_TILE_MAX		24
#define MTK_MFB_TDR_BYTES		(4 * MTK_MFB_TDR_WORDS * MTK_MFB_TILE_MAX)

/* frame registers of one blend, written in this order before the start */
#define MTK_MFB_REG_MAX			96

/* tuning words of the vendor driver's register map */
#define MTK_MFB_LL_WORDS		10
#define MTK_MFB_MBD_WORDS		11

/**
 * struct mtk_mfb_geom - geometry of a blend
 * @w: image width, a multiple of %MTK_MFB_W_STEP
 * @h: image height, even
 * @base_bpl: line step of the base image (NV12, Y and UV plane)
 * @ref_bpl: line step of the reference image (NV12)
 * @out_bpl: line step of the blended image (NV12)
 * @wt_in_bpl: line step of the weight map input, 0 without it
 * @wt_out_bpl: line step of the weight map output
 *
 * A blend with a weight map input continues a series (second and later
 * blend); without it, the block starts a new weight map.
 */
struct mtk_mfb_geom {
	u32 w;
	u32 h;
	u32 base_bpl;
	u32 ref_bpl;
	u32 out_bpl;
	u32 wt_in_bpl;
	u32 wt_out_bpl;
};

/**
 * struct mtk_mfb_tile - columns of one tile
 * @in_xs: first column the blending core reads (all inputs)
 * @in_xe: last column the blending core reads
 * @out_xs: first column written (all outputs)
 * @out_xe: last column written
 */
struct mtk_mfb_tile {
	u16 in_xs;
	u16 in_xe;
	u16 out_xs;
	u16 out_xe;
};

/**
 * struct mtk_mfb_layout - tiles of a frame, left to right, over the whole height
 * @n_tiles: number of tiles
 * @tile: the tiles
 */
struct mtk_mfb_layout {
	unsigned int n_tiles;
	struct mtk_mfb_tile tile[MTK_MFB_TILE_MAX];
};

/**
 * struct mtk_mfb_addrs - bus addresses of a blend (32 bit)
 * @base: base image, Y plane; the UV plane follows at @base_uv
 * @base_uv: UV plane of the base image
 * @ref: reference image, Y plane
 * @ref_uv: UV plane of the reference image
 * @wt_in: weight map input, 0 without it
 * @out: blended image, Y plane
 * @out_uv: UV plane of the blended image
 * @wt_out: weight map output
 * @tdr: tile descriptions
 */
struct mtk_mfb_addrs {
	u32 base;
	u32 base_uv;
	u32 ref;
	u32 ref_uv;
	u32 wt_in;
	u32 out;
	u32 out_uv;
	u32 wt_out;
	u32 tdr;
};

/**
 * struct mtk_mfb_tuning - tuning words of the blending core
 * @con: bits 15:4 of the control register (MFB_CON); the driver sets the
 *	blend mode in bits 3:0
 * @ll: LL_CON1 to LL_CON10
 * @edge: EDGE
 * @mbd: MBD_CON0 to MBD_CON10
 */
struct mtk_mfb_tuning {
	u32 con;
	u32 ll[MTK_MFB_LL_WORDS];
	u32 edge;
	u32 mbd[MTK_MFB_MBD_WORDS];
};

/**
 * struct mtk_mfb_reg - one register write
 * @off: register offset
 * @val: value
 */
struct mtk_mfb_reg {
	u16 off;
	u32 val;
};

int mtk_mfb_geom_check(const struct mtk_mfb_geom *g);
int mtk_mfb_tiles(const struct mtk_mfb_geom *g, struct mtk_mfb_layout *l);
void mtk_mfb_tdr(const struct mtk_mfb_geom *g, const struct mtk_mfb_layout *l,
		 u32 *words);
unsigned int mtk_mfb_frame_regs(const struct mtk_mfb_geom *g,
				const struct mtk_mfb_addrs *a,
				const struct mtk_mfb_tuning *t,
				struct mtk_mfb_reg *regs);

#endif /* _MTK_MFB_JOB_H */
