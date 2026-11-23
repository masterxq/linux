// SPDX-License-Identifier: GPL-2.0-only
/*
 * MediaTek MT6785 multi-frame blending block (MFB): register values and tile
 * descriptions of one blend.
 *
 * The block blends a reference image into a base image (both NV12) under
 * control of a weight map of one byte per pixel, and writes the blended
 * image and a new weight map. The first blend of a series has no weight map
 * input; each further blend reads the map the blend before wrote.
 *
 * Frame registers: order and the fixed port settings (FIFO control words,
 * the x size taken from the line step, the interrupt sources, the start bit)
 * follow the MediaTek vendor driver of this block (camera_mfb.c, isp_50,
 * Copyright (C) 2015 MediaTek Inc., GPL-2.0), which writes every register
 * of a blend, used or not. For an NV12 blend the driver sets the format and
 * stride bits of the image inputs and the converter and resizer settings;
 * the tuning words besides the per-ISO ones are zero.
 *
 * Tiles: the blending core reads at most 256 columns per tile and loses 4
 * columns at each tile edge that is not a frame edge; its input starts and
 * ends on a 4-column grid. The block walks through the tiles of a frame on
 * its own: one start, one interrupt after the last tile. Each tile has a
 * description of %MTK_MFB_TDR_WORDS words with the edges, the window of
 * every port relative to its base address, the size of the core's input
 * and the crop windows of the chroma converters.
 *
 * Integer arithmetic only.
 */

#include <linux/bits.h>
#include <linux/errno.h>
#include <linux/minmax.h>
#include <linux/string.h>
#include <linux/types.h>

#include "mtk_mfb_job.h"
#include "mtk_mfb_regs.h"

/* tile limits of the blending core */
#define MFB_CORE_MAX_IN			256
#define MFB_CORE_LOSS			4
#define MFB_CORE_ALIGN			4
#define MFB_CORE_MAX_H			8190
/* the 4:2:0 converter in front of the output ports takes as much */
#define MFB_CRSP_MAX_IN			256

/* tile description: word 0 and the words of each part */
#define MFB_TDR_EDGE			GENMASK(3, 0)
#define MFB_TDR_EDGE_LEFT		BIT(0)
#define MFB_TDR_EDGE_RIGHT		BIT(1)
#define MFB_TDR_EDGE_TOP		BIT(2)
#define MFB_TDR_EDGE_BOTTOM		BIT(3)
#define MFB_TDR_LAST			BIT(5)
#define MFB_TDR_LOAD_SHIFT		16
#define MFB_TDR_BASE			1	/* base Y: offset, size */
#define MFB_TDR_REF			3	/* reference Y */
#define MFB_TDR_WT_IN			5	/* weight map input */
#define MFB_TDR_MFB4I			7	/* fourth input, unused */
#define MFB_TDR_OUT			9	/* blended Y: offset, size, offsets */
#define MFB_TDR_WT_OUT			12	/* weight map output */
#define MFB_TDR_CORE			15	/* grid phase, input width, height */
#define MFB_TDR_SRZ			18	/* resizer of the fourth input, unused */
#define MFB_TDR_BASE_UV			22	/* base UV */
#define MFB_TDR_REF_UV			24	/* reference UV */
#define MFB_TDR_OUT_UV			26	/* blended UV */
#define MFB_TDR_C02_BASE		29	/* 4:2:0 to 4:2:2, base: edge, crop */
#define MFB_TDR_C02_REF			32	/* the same for the reference */
#define MFB_TDR_CRSP			35	/* 4:2:2 to 4:2:0: size, phases, crop */
#define MFB_CORE_GRID			GENMASK(3, 0)

/* NV12 inputs: input format 1 of the block (two planes, 4:2:0) */
#define MFB_FMT_YUV420_2P		1
#define MFB_TOP_CFG0_ON			GENMASK(1, 0)
/*
 * Bit 2 belongs to the resizer of the fourth input, but has to be set in any
 * case: without it the block starts, its input DMAs fill and nothing ever
 * leaves the blending core (no output, no interrupt), with or without the
 * fourth input, with the resizer itself left as for no input.
 */
#define MFB_TOP_CFG0_SRZ		BIT(2)
#define MFB_TOP_CFG0_FMT_SHIFT		3
/* DMA enable bits in TOP_CFG2 */
#define MFB_EN_MFBI			BIT(0)
#define MFB_EN_MFB2I			BIT(1)
#define MFB_EN_MFB3I			BIT(2)
#define MFB_EN_MFB4I			BIT(3)
#define MFB_EN_MFBO			BIT(4)
#define MFB_EN_MFB2O			BIT(5)
#define MFB_EN_MFBI_B			BIT(6)
#define MFB_EN_MFB2I_B			BIT(7)
#define MFB_EN_MFBO_B			BIT(8)
/* format bits of the stride register of the image inputs, NV12 */
#define MFB_STRIDE_YUV420_2P		0x00300000
#define MFB_STRIDE_LINE			GENMASK(15, 0)
/* FIFO control words of the vendor driver */
#define MFB_PORT_CON_IN			0x80000040
#define MFB_PORT_CON_OUT_SMALL		0x80000020
/* done, chroma done and weight done interrupts of the vendor driver */
#define MFB_INT_CTL_INIT		0x00300001
#define MFB_TDRI_XSIZE_TILE		(4 * MTK_MFB_TDR_WORDS - 1)
/* converters and resizer for an NV12 blend without the fourth input */
#define MFB_SRZ_CONTROL_INIT		0x13
#define MFB_SRZ_OUT_IMG_NONE		0x00010001
#define MFB_C02_CON_INIT		0x1f
#define MFB_CRSP_CTRL_INIT		0x80005e12
#define MFB_CRSP_STEP_OFST_YUV420	0x01040002
/* blend mode in MFB_CON: 0 starts a weight map, 1 reads one */
#define MFB_MODE_FIRST			0
#define MFB_MODE_NEXT			1

static inline u32 mfb_pair(u32 lo, u32 hi)
{
	return (lo & 0xffff) | (hi << 16);
}

/**
 * mtk_mfb_geom_check - check the geometry of a blend
 * @g: the geometry
 *
 * Return: 0 or -EINVAL.
 */
int mtk_mfb_geom_check(const struct mtk_mfb_geom *g)
{
	u32 bpl_max = MFB_STRIDE_LINE;

	if (g->w < MTK_MFB_MIN_W || g->w > MTK_MFB_MAX_W ||
	    g->w % MTK_MFB_W_STEP || g->h < MTK_MFB_MIN_H ||
	    g->h > MTK_MFB_MAX_H || g->h % MTK_MFB_H_STEP)
		return -EINVAL;
	if (g->base_bpl < g->w || g->ref_bpl < g->w || g->out_bpl < g->w ||
	    g->wt_out_bpl < g->w || (g->wt_in_bpl && g->wt_in_bpl < g->w))
		return -EINVAL;
	if (g->base_bpl > bpl_max || g->ref_bpl > bpl_max ||
	    g->out_bpl > bpl_max || g->wt_in_bpl > bpl_max ||
	    g->wt_out_bpl > bpl_max)
		return -EINVAL;

	return 0;
}

/**
 * mtk_mfb_tiles - cut a frame into tiles
 * @g: the geometry, checked by mtk_mfb_geom_check()
 * @l: the tiles
 *
 * Backwards from the output: the output of tile k starts one column behind
 * the output of tile k-1 and runs as far as the 4:2:0 converter takes it;
 * the core input adds its loss at inner edges, is cut to its largest width
 * and kept on its grid. Forwards, the core output is its input without the
 * loss at inner edges.
 *
 * Return: 0, -EINVAL or -E2BIG.
 */
int mtk_mfb_tiles(const struct mtk_mfb_geom *g, struct mtk_mfb_layout *l)
{
	u32 w = g->w, os = 0, oe, ce, xs, xe;
	unsigned int n = 0;

	if (g->h > MFB_CORE_MAX_H || w % MFB_CORE_ALIGN)
		return -EINVAL;

	memset(l, 0, sizeof(*l));
	while (os < w) {
		if (n == MTK_MFB_TILE_MAX)
			return -E2BIG;

		/* backwards: converter input, then core input */
		ce = min(w - 1, os + MFB_CRSP_MAX_IN - 1);
		xs = os ? os - MFB_CORE_LOSS : 0;
		xe = ce < w - 1 ? ce + MFB_CORE_LOSS : w - 1;
		xe = min(xe, w - 1);
		if (xe - xs + 1 > MFB_CORE_MAX_IN)
			xe = xs + MFB_CORE_MAX_IN - 1;
		xs -= xs % MFB_CORE_ALIGN;
		xe += (MFB_CORE_ALIGN - (xe + 1) % MFB_CORE_ALIGN) %
		      MFB_CORE_ALIGN;
		if (xe > w - 1)
			return -EINVAL;

		/* forwards: core output */
		oe = xe < w - 1 ? xe - MFB_CORE_LOSS : w - 1;
		if (oe < os || (xs ? xs + MFB_CORE_LOSS : 0) != os)
			return -EINVAL;

		l->tile[n].in_xs = xs;
		l->tile[n].in_xe = xe;
		l->tile[n].out_xs = os;
		l->tile[n].out_xe = oe;
		n++;
		os = oe + 1;
	}
	l->n_tiles = n;

	return 0;
}

static void mfb_tdr_port(u32 *t, unsigned int idx, u32 xs, u32 cols, u32 rows)
{
	/* window relative to the base address: first line 0, column xs */
	t[idx] = xs;
	t[idx + 1] = mfb_pair(cols - 1, rows - 1);
}

/**
 * mtk_mfb_tdr - tile descriptions of a frame
 * @g: the geometry
 * @l: its tiles, from mtk_mfb_tiles()
 * @words: l->n_tiles * %MTK_MFB_TDR_WORDS words
 *
 * The weight map input part stays 0 without the input, as do the parts of
 * the fourth input and its resizer, which the driver does not use.
 */
void mtk_mfb_tdr(const struct mtk_mfb_geom *g, const struct mtk_mfb_layout *l,
		 u32 *words)
{
	u32 h = g->h, uv_h = g->h / 2;
	unsigned int k;

	for (k = 0; k < l->n_tiles; k++) {
		const struct mtk_mfb_tile *tl = &l->tile[k];
		u32 *t = words + k * MTK_MFB_TDR_WORDS;
		u32 iw = tl->in_xe - tl->in_xs + 1;
		u32 ow = tl->out_xe - tl->out_xs + 1;
		u32 edge = MFB_TDR_EDGE_TOP | MFB_TDR_EDGE_BOTTOM;
		bool last = k == l->n_tiles - 1;

		memset(t, 0, 4 * MTK_MFB_TDR_WORDS);
		if (!k)
			edge |= MFB_TDR_EDGE_LEFT;
		if (last)
			edge |= MFB_TDR_EDGE_RIGHT;
		t[0] = edge | (last ? MFB_TDR_LAST : 0) |
		       (MTK_MFB_TDR_WORDS - 1) << MFB_TDR_LOAD_SHIFT;

		/* inputs: the core input window, Y full and UV half height */
		mfb_tdr_port(t, MFB_TDR_BASE, tl->in_xs, iw, h);
		mfb_tdr_port(t, MFB_TDR_REF, tl->in_xs, iw, h);
		mfb_tdr_port(t, MFB_TDR_BASE_UV, tl->in_xs, iw, uv_h);
		mfb_tdr_port(t, MFB_TDR_REF_UV, tl->in_xs, iw, uv_h);
		/*
		 * The weight map goes past the core's filters: it is read for
		 * the output window of the tile (with the input window the
		 * block stalls in the second tile).
		 */
		if (g->wt_in_bpl)
			mfb_tdr_port(t, MFB_TDR_WT_IN, tl->out_xs, ow, h);

		/* outputs: the core output window, write offsets 0 */
		mfb_tdr_port(t, MFB_TDR_OUT, tl->out_xs, ow, h);
		mfb_tdr_port(t, MFB_TDR_WT_OUT, tl->out_xs, ow, h);
		mfb_tdr_port(t, MFB_TDR_OUT_UV, tl->out_xs, ow, uv_h);

		/* core: phase of its input on the 16-column grid, input size */
		t[MFB_TDR_CORE] = tl->in_xs & MFB_CORE_GRID;
		t[MFB_TDR_CORE + 1] = iw << 16;
		t[MFB_TDR_CORE + 2] = h;

		/* chroma upsamplers of base and reference: whole window */
		t[MFB_TDR_C02_BASE] = MFB_TDR_EDGE;
		t[MFB_TDR_C02_BASE + 1] = mfb_pair(0, iw - 1);
		t[MFB_TDR_C02_BASE + 2] = mfb_pair(0, h - 1);
		t[MFB_TDR_C02_REF] = MFB_TDR_EDGE;
		t[MFB_TDR_C02_REF + 1] = mfb_pair(0, iw - 1);
		t[MFB_TDR_C02_REF + 2] = mfb_pair(0, h - 1);

		/* 4:2:0 converter: output size, phases 0, nothing cropped */
		t[MFB_TDR_CRSP] = mfb_pair(ow, h);
		t[MFB_TDR_CRSP + 3] = mfb_pair(0, ow - 1);
		t[MFB_TDR_CRSP + 4] = mfb_pair(0, h - 1);
	}
}

static unsigned int mfb_port(struct mtk_mfb_reg *r, unsigned int n, u16 port,
			     u32 addr, u32 stride, u32 ysize, u32 con)
{
	/* x size from the line step, as the vendor driver; 0 - 1 if unused */
	r[n++] = (struct mtk_mfb_reg){ port + MFB_PORT_BASE_ADDR, addr };
	r[n++] = (struct mtk_mfb_reg){ port + MFB_PORT_STRIDE, stride };
	r[n++] = (struct mtk_mfb_reg){ port + MFB_PORT_XSIZE,
				       (stride & MFB_STRIDE_LINE) - 1 };
	r[n++] = (struct mtk_mfb_reg){ port + MFB_PORT_YSIZE, ysize };
	r[n++] = (struct mtk_mfb_reg){ port + MFB_PORT_CON, con };

	return n;
}

/**
 * mtk_mfb_frame_regs - frame registers of a blend
 * @g: the geometry
 * @a: the bus addresses
 * @t: the tuning words
 * @regs: at least %MTK_MFB_REG_MAX entries
 *
 * The start bit is not part of the list.
 *
 * Return: the number of entries.
 */
unsigned int mtk_mfb_frame_regs(const struct mtk_mfb_geom *g,
				const struct mtk_mfb_addrs *a,
				const struct mtk_mfb_tuning *t,
				struct mtk_mfb_reg *regs)
{
	bool wt_in = g->wt_in_bpl;
	u32 en, img = MFB_STRIDE_YUV420_2P, mode;
	unsigned int n = 0, i;

	en = MFB_EN_MFBI | MFB_EN_MFB2I | MFB_EN_MFBO | MFB_EN_MFB2O |
	     MFB_EN_MFBI_B | MFB_EN_MFB2I_B | MFB_EN_MFBO_B;
	if (wt_in)
		en |= MFB_EN_MFB3I;
	mode = wt_in ? MFB_MODE_NEXT : MFB_MODE_FIRST;

	regs[n++] = (struct mtk_mfb_reg){ MFB_INT_CTL, MFB_INT_CTL_INIT };
	regs[n++] = (struct mtk_mfb_reg){ MFB_TOP_CFG0, MFB_TOP_CFG0_ON |
		MFB_TOP_CFG0_SRZ | MFB_FMT_YUV420_2P << MFB_TOP_CFG0_FMT_SHIFT };
	regs[n++] = (struct mtk_mfb_reg){ MFB_TOP_CFG2, en };

	/* the UV planes of the inputs take the image height, as the vendor */
	n = mfb_port(regs, n, MFB_MFBI, a->base, g->base_bpl | img, g->h - 1,
		     MFB_PORT_CON_IN);
	n = mfb_port(regs, n, MFB_MFBI_B, a->base_uv, g->base_bpl | img,
		     g->h - 1, MFB_PORT_CON_IN);
	n = mfb_port(regs, n, MFB_MFB2I, a->ref, g->ref_bpl | img, g->h - 1,
		     MFB_PORT_CON_IN);
	n = mfb_port(regs, n, MFB_MFB2I_B, a->ref_uv, g->ref_bpl | img,
		     g->h - 1, MFB_PORT_CON_IN);
	n = mfb_port(regs, n, MFB_MFB3I, wt_in ? a->wt_in : 0,
		     wt_in ? g->wt_in_bpl : 0, wt_in ? g->h - 1 : 0,
		     MFB_PORT_CON_IN);
	n = mfb_port(regs, n, MFB_MFB4I, 0, 0, 0, MFB_PORT_CON_IN);
	n = mfb_port(regs, n, MFB_MFBO, a->out, g->out_bpl, g->h - 1,
		     MFB_PORT_CON_IN);
	n = mfb_port(regs, n, MFB_MFBO_B, a->out_uv, g->out_bpl, g->h / 2 - 1,
		     MFB_PORT_CON_OUT_SMALL);
	n = mfb_port(regs, n, MFB_MFB2O, a->wt_out, g->wt_out_bpl, g->h - 1,
		     MFB_PORT_CON_OUT_SMALL);

	regs[n++] = (struct mtk_mfb_reg){ MFB_TDRI_BASE_ADDR, a->tdr };
	regs[n++] = (struct mtk_mfb_reg){ MFB_TDRI_XSIZE, MFB_TDRI_XSIZE_TILE };

	/* resizer of the fourth input: off, as the vendor leaves it */
	regs[n++] = (struct mtk_mfb_reg){ MFB_SRZ_CONTROL, MFB_SRZ_CONTROL_INIT };
	regs[n++] = (struct mtk_mfb_reg){ MFB_SRZ_IN_IMG, 0 };
	regs[n++] = (struct mtk_mfb_reg){ MFB_SRZ_OUT_IMG, MFB_SRZ_OUT_IMG_NONE };
	regs[n++] = (struct mtk_mfb_reg){ MFB_SRZ_HORI_STEP, 0 };
	regs[n++] = (struct mtk_mfb_reg){ MFB_SRZ_VERT_STEP, 0 };
	regs[n++] = (struct mtk_mfb_reg){ MFB_SRZ_HORI_INT_OFST, 0 };
	regs[n++] = (struct mtk_mfb_reg){ MFB_SRZ_HORI_SUB_OFST, 0 };
	regs[n++] = (struct mtk_mfb_reg){ MFB_SRZ_VERT_INT_OFST, 0 };
	regs[n++] = (struct mtk_mfb_reg){ MFB_SRZ_VERT_SUB_OFST, 0 };

	regs[n++] = (struct mtk_mfb_reg){ MFB_C02A_CON, MFB_C02_CON_INIT };
	regs[n++] = (struct mtk_mfb_reg){ MFB_C02B_CON, MFB_C02_CON_INIT };

	regs[n++] = (struct mtk_mfb_reg){ MFB_CRSP_CTRL, MFB_CRSP_CTRL_INIT };
	regs[n++] = (struct mtk_mfb_reg){ MFB_CRSP_OUT_IMG, mfb_pair(g->w, g->h) };
	regs[n++] = (struct mtk_mfb_reg){ MFB_CRSP_STEP_OFST,
					  MFB_CRSP_STEP_OFST_YUV420 };

	regs[n++] = (struct mtk_mfb_reg){ MFB_CON,
		(t->con & MFB_CON_TUNING) | mode };
	regs[n++] = (struct mtk_mfb_reg){ MFB_LL_CON1, t->ll[0] };
	regs[n++] = (struct mtk_mfb_reg){ MFB_LL_CON2, t->ll[1] };
	regs[n++] = (struct mtk_mfb_reg){ MFB_LL_CON3, t->ll[2] };
	regs[n++] = (struct mtk_mfb_reg){ MFB_LL_CON4, t->ll[3] };
	regs[n++] = (struct mtk_mfb_reg){ MFB_EDGE, t->edge };
	for (i = 4; i < MTK_MFB_LL_WORDS; i++)
		regs[n++] = (struct mtk_mfb_reg){ MFB_LL_CON5 + 4 * (i - 4),
						  t->ll[i] };
	for (i = 0; i < MTK_MFB_MBD_WORDS; i++)
		regs[n++] = (struct mtk_mfb_reg){ MFB_MBD_CON0 + 4 * i,
						  t->mbd[i] };

	return n;
}
