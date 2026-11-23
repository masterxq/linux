// SPDX-License-Identifier: GPL-2.0-only
/*
 * MediaTek MT6785 ISP pass 2 (DIP): lens shading table.
 *
 * Table format: the shading module reads one line per row of the 16 x 16 blocks
 * of the frame grid. A block holds, for each colour of the Bayer cell in the
 * order of the Bayer phase (blue, green of a blue row, green of a red row, red:
 * the raster positions of a BGGR frame), the coefficients of a polynomial in
 * the position inside the block (u to the right, v down, both 0 at the top left
 * corner and 1 at the next corner):
 *
 *   G(u, v) = c00 + c10 u + c20 u^2 + c30 u^3 + c01 v + c02 v^2 + c03 v^3
 *           + c11 u v + c21 u^2 v + c31 u^3 v + c12 u v^2 + c13 u v^3
 *
 * as 12 half-words: c00 unsigned with 1.0 = 4096, the others as sign and
 * magnitude (bit 15 the sign) in units of 2/4096.
 *
 * These twelve terms are exactly those of a Coons patch: four cubic edge
 * curves of a block blended bilinearly. The table is built from the gains at
 * the block corners: a natural cubic spline along every row and every column
 * of corners gives the edges, and the patch of the four edges of a block its
 * coefficients. The values are continuous across the block edges, the
 * splines also in their first and second derivative.
 *
 * Integer arithmetic only. Corner gains are unsigned 16 bit, block sizes at
 * most 2^8 cells (frames up to 8191 pixels); the intermediate values of the
 * spline stay below 2^62.
 */

#include <linux/array_size.h>
#include <linux/bits.h>
#include <linux/errno.h>
#include <linux/math64.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/unaligned.h>

#include "mtk_dip_tile.h"

#define MTK_DIP_LSC_N		MTK_DIP_LSC_BLOCKS
#define MTK_DIP_LSC_POS		4
#define MTK_DIP_LSC_FIELDS	12
/* fraction bits of the second derivatives and of the elimination factors */
#define MTK_DIP_LSC_Q		16
/* the coefficients other than c00 are stored in units of 2 */
#define MTK_DIP_LSC_COEF_SCALE	2
#define MTK_DIP_LSC_COEF_MAX	0x7fff
#define MTK_DIP_LSC_COEF_NEG	BIT(15)

/* half-word of each coefficient in a block entry */
enum {
	MTK_DIP_LSC_C31 = 0,
	MTK_DIP_LSC_C13 = 1,
	MTK_DIP_LSC_C30 = 2,
	MTK_DIP_LSC_C21 = 3,
	MTK_DIP_LSC_C12 = 4,
	MTK_DIP_LSC_C03 = 5,
	MTK_DIP_LSC_C20 = 6,
	MTK_DIP_LSC_C11 = 7,
	MTK_DIP_LSC_C02 = 8,
	MTK_DIP_LSC_C10 = 9,
	MTK_DIP_LSC_C01 = 10,
	MTK_DIP_LSC_C00 = 11,
};

/* a + b t + c t^2 + d t^3 over one interval, t = 0..1; Q4.12 */
struct mtk_dip_cubic {
	s32 a;
	s32 b;
	s32 c;
	s32 d;
};

struct mtk_dip_lsc_work {
	struct mtk_dip_cubic row[MTK_DIP_LSC_CORNERS][MTK_DIP_LSC_N];
	struct mtk_dip_cubic col[MTK_DIP_LSC_CORNERS][MTK_DIP_LSC_N];
};

/* a / b rounded half away from zero, b > 0 */
static s64 mtk_dip_div_round(s64 a, s64 b)
{
	if (a >= 0)
		return div64_s64(a + b / 2, b);

	return -div64_s64(-a + b / 2, b);
}

/*
 * Natural cubic spline through the MTK_DIP_LSC_CORNERS values @y with the
 * interval lengths @h: second derivatives m (scaled by 2^Q, zero at both
 * ends) from the tridiagonal system, solved by elimination, then the cubic
 * of each interval.
 */
static void mtk_dip_lsc_spline(const s32 *y, const u32 *h,
			       struct mtk_dip_cubic *seg)
{
	s64 m[MTK_DIP_LSC_N + 1] = {}, cp[MTK_DIP_LSC_N] = {};
	s64 dp[MTK_DIP_LSC_N] = {};
	s64 r, den, hh;
	unsigned int i;

	for (i = 1; i < MTK_DIP_LSC_N; i++) {
		r = 6 * ((s64)(y[i + 1] - y[i]) * h[i - 1] -
			 (s64)(y[i] - y[i - 1]) * h[i]);
		r = mtk_dip_div_round(r * (1LL << MTK_DIP_LSC_Q),
				      (s64)h[i] * h[i - 1]);
		den = 2LL * (h[i - 1] + h[i]) * (1LL << 16) -
		      (s64)h[i - 1] * cp[i - 1];
		cp[i] = mtk_dip_div_round((s64)h[i] << 32, den);
		dp[i] = mtk_dip_div_round((r - (s64)h[i - 1] * dp[i - 1]) *
					  (1LL << 16), den);
	}
	for (i = MTK_DIP_LSC_N - 1; i > 0; i--)
		m[i] = dp[i] - mtk_dip_div_round(cp[i] * m[i + 1], 1LL << 16);

	for (i = 0; i < MTK_DIP_LSC_N; i++) {
		hh = (s64)h[i] * h[i];
		seg[i].a = y[i];
		seg[i].b = y[i + 1] - y[i] -
			   mtk_dip_div_round(hh * (2 * m[i] + m[i + 1]),
					     6LL << MTK_DIP_LSC_Q);
		seg[i].c = mtk_dip_div_round(hh * m[i], 2LL << MTK_DIP_LSC_Q);
		seg[i].d = mtk_dip_div_round(hh * (m[i + 1] - m[i]),
					     6LL << MTK_DIP_LSC_Q);
	}
}

/* Stored form of a coefficient; counts the ones out of range */
static u16 mtk_dip_lsc_coef(s64 v, unsigned int *clamped)
{
	v = mtk_dip_div_round(v, MTK_DIP_LSC_COEF_SCALE);
	if (v > MTK_DIP_LSC_COEF_MAX || v < -MTK_DIP_LSC_COEF_MAX) {
		v = v > 0 ? MTK_DIP_LSC_COEF_MAX : -MTK_DIP_LSC_COEF_MAX;
		(*clamped)++;
	}

	return v < 0 ? MTK_DIP_LSC_COEF_NEG | -v : v;
}

static void mtk_dip_lsc_block(const s32 *g, unsigned int bx, unsigned int by,
			      const struct mtk_dip_lsc_work *w, u8 *out,
			      unsigned int *clamped)
{
	const struct mtk_dip_cubic *t = &w->row[by][bx];
	const struct mtk_dip_cubic *b = &w->row[by + 1][bx];
	const struct mtk_dip_cubic *l = &w->col[bx][by];
	const struct mtk_dip_cubic *r = &w->col[bx + 1][by];
	s64 c[MTK_DIP_LSC_FIELDS];
	s64 g00 = g[by * MTK_DIP_LSC_CORNERS + bx];
	s64 g10 = g[by * MTK_DIP_LSC_CORNERS + bx + 1];
	s64 g01 = g[(by + 1) * MTK_DIP_LSC_CORNERS + bx];
	s64 g11 = g[(by + 1) * MTK_DIP_LSC_CORNERS + bx + 1];
	unsigned int f;

	/* top edge in u, left edge in v, the rest from the bilinear blend */
	c[MTK_DIP_LSC_C10] = t->b;
	c[MTK_DIP_LSC_C20] = t->c;
	c[MTK_DIP_LSC_C30] = t->d;
	c[MTK_DIP_LSC_C01] = l->b;
	c[MTK_DIP_LSC_C02] = l->c;
	c[MTK_DIP_LSC_C03] = l->d;
	c[MTK_DIP_LSC_C11] = (s64)b->b - t->b + r->b - l->b -
			     (g00 - g10 - g01 + g11);
	c[MTK_DIP_LSC_C21] = (s64)b->c - t->c;
	c[MTK_DIP_LSC_C31] = (s64)b->d - t->d;
	c[MTK_DIP_LSC_C12] = (s64)r->c - l->c;
	c[MTK_DIP_LSC_C13] = (s64)r->d - l->d;

	for (f = 0; f < MTK_DIP_LSC_FIELDS; f++)
		put_unaligned_le16(f == MTK_DIP_LSC_C00 ? g00 :
				   mtk_dip_lsc_coef(c[f], clamped),
				   out + 2 * f);
}

/**
 * mtk_dip_lsc_table - shading table from the gains at the block corners
 * @gain: gains, unsigned Q4.12, [Bayer position][row][column] of
 *	MTK_DIP_LSC_CORNERS x MTK_DIP_LSC_CORNERS corners each; the Bayer
 *	position in raster order of the frame
 * @pix_id: Bayer phase of the frame, MTK_DIP_PIX_ID_*: the colour k of the
 *	table takes the gains of raster position k ^ @pix_id
 * @f: frame grids, see mtk_dip_frame_cfg_init()
 * @table: result, MTK_DIP_LSC_TABLE_SIZE bytes with lines of @f->lsc_stride
 *
 * Return: the number of coefficients clamped to the stored range (steps
 * between corners too steep for the format), -EINVAL for a frame too small
 * for the grid, -ENOMEM.
 */
int mtk_dip_lsc_table(const u16 *gain, u32 pix_id,
		      const struct mtk_dip_frame_cfg *f, void *table)
{
	u32 hx[MTK_DIP_LSC_N], hy[MTK_DIP_LSC_N];
	s32 g[MTK_DIP_LSC_CORNERS * MTK_DIP_LSC_CORNERS];
	s32 line[MTK_DIP_LSC_CORNERS];
	struct mtk_dip_lsc_work *w;
	unsigned int clamped = 0;
	unsigned int k, i, j;
	u8 *out;

	if (pix_id > MTK_DIP_PIX_ID_RGGB ||
	    !f->lsc_bw || !f->lsc_bh || f->lsc_nx != MTK_DIP_LSC_N - 1 ||
	    f->lsc_ny != MTK_DIP_LSC_N - 1 ||
	    f->lsc_stride != MTK_DIP_LSC_N * MTK_DIP_LSC_BLOCK_BYTES)
		return -EINVAL;

	w = kmalloc_obj(*w, GFP_KERNEL);
	if (!w)
		return -ENOMEM;

	for (i = 0; i < MTK_DIP_LSC_N; i++) {
		hx[i] = i < MTK_DIP_LSC_N - 1 ? f->lsc_bw : f->lsc_lw;
		hy[i] = i < MTK_DIP_LSC_N - 1 ? f->lsc_bh : f->lsc_lh;
	}

	for (k = 0; k < MTK_DIP_LSC_POS; k++) {
		for (i = 0; i < ARRAY_SIZE(g); i++)
			g[i] = gain[(k ^ pix_id) * ARRAY_SIZE(g) + i];

		for (j = 0; j < MTK_DIP_LSC_CORNERS; j++)
			mtk_dip_lsc_spline(&g[j * MTK_DIP_LSC_CORNERS], hx,
					   w->row[j]);
		for (i = 0; i < MTK_DIP_LSC_CORNERS; i++) {
			for (j = 0; j < MTK_DIP_LSC_CORNERS; j++)
				line[j] = g[j * MTK_DIP_LSC_CORNERS + i];
			mtk_dip_lsc_spline(line, hy, w->col[i]);
		}

		for (j = 0; j < MTK_DIP_LSC_N; j++) {
			for (i = 0; i < MTK_DIP_LSC_N; i++) {
				out = (u8 *)table + j * f->lsc_stride +
				      i * MTK_DIP_LSC_BLOCK_BYTES +
				      k * MTK_DIP_LSC_FIELDS * 2;
				mtk_dip_lsc_block(g, i, j, w, out, &clamped);
			}
		}
	}

	kfree(w);

	return clamped;
}
