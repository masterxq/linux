// SPDX-License-Identifier: GPL-2.0-only
/*
 * MediaTek MT6785 ISP pass 2 (DIP): parameter buffers.
 *
 * Checks the params buffers of user space (struct v4l2_isp_params_buffer
 * with the blocks of <linux/media/mediatek/mtk-dip-mt6785.h>) and translates
 * them into register values. Encodings:
 *  - black level: offset added before the gain, signed 13 bit in units of
 *    the 12-bit internal data; gain unsigned 13 bit, 512 = 1.0.
 *  - white balance: unsigned 13 bit gains, 512 = 1.0.
 *  - colour matrix: signed 13 bit, 512 = 1.0, row = output R, G, B.
 *  - gamma: 192 points, three 10-bit outputs per point (R in the low bits,
 *    then G, then B); bit 0 of the control register bypasses the curve.
 *  - lens shading: the corner gains are kept as they are; the job runner
 *    computes the table for the frame grid (mtk_dip_lsc.c).
 *  - edge enhancement, both noise reduction stages, demosaicing, temporal
 *    noise reduction, radial weights, raw noise reduction: one field per
 *    parameter in the register words of the module (mtk_dip_bq.c).
 */

#include <linux/bitfield.h>
#include <linux/build_bug.h>
#include <linux/limits.h>
#include <linux/minmax.h>
#include <linux/string.h>
#include <media/v4l2-isp.h>

#include "mtk_dip.h"
#include "mtk_dip_regs.h"

#define DIP_BLC_GAIN_ONE		512
#define DIP_WB_GAIN_ONE			512
#define DIP_CCM_ONE			512
#define DIP_FIELD13_MAX			0x1fff

/* gamma control: curve on, and bypass */
#define DIP_GAMMA_CTL_CURVE		0x100307fe
#define DIP_GAMMA_CTL_BYPASS		0x100307ff

#define DIP_GAMMA_OUT_MAX		1023

/*
 * Colour (blue, green of a blue row, green of a red row, red: the Bayer
 * positions of a BGGR frame) to black level register (B, Gr, Gb, R)
 */
static const u8 mtk_dip_blc_reg[MTK_DIP_BAYER_CHANNELS] = { 0, 2, 1, 3 };

static const struct v4l2_isp_params_block_type_info
mtk_dip_block_info[] = {
	[MTK_DIP_PARAMS_BLOCK_BLC] = { sizeof(struct mtk_dip_params_blc) },
	[MTK_DIP_PARAMS_BLOCK_WB] = { sizeof(struct mtk_dip_params_wb) },
	[MTK_DIP_PARAMS_BLOCK_CCM] = { sizeof(struct mtk_dip_params_ccm) },
	[MTK_DIP_PARAMS_BLOCK_GAMMA] = { sizeof(struct mtk_dip_params_gamma) },
	[MTK_DIP_PARAMS_BLOCK_LSC] = { sizeof(struct mtk_dip_params_lsc) },
	[MTK_DIP_PARAMS_BLOCK_EE] = { sizeof(struct mtk_dip_params_ee) },
	[MTK_DIP_PARAMS_BLOCK_YCNR] = { sizeof(struct mtk_dip_params_ycnr) },
	[MTK_DIP_PARAMS_BLOCK_DEMOSAIC] = {
		sizeof(struct mtk_dip_params_demosaic) },
	[MTK_DIP_PARAMS_BLOCK_NBC2] = { sizeof(struct mtk_dip_params_nbc2) },
	/* not used: no block has this size, the check refuses a header alone */
	[MTK_DIP_PARAMS_BLOCK_NBC2 + 1] = { SIZE_MAX },
	[MTK_DIP_PARAMS_BLOCK_NR3D] = { sizeof(struct mtk_dip_params_nr3d) },
	[MTK_DIP_PARAMS_BLOCK_MIX] = { sizeof(struct mtk_dip_params_mix) },
	[MTK_DIP_PARAMS_BLOCK_RADIAL] = {
		sizeof(struct mtk_dip_params_radial) },
	[MTK_DIP_PARAMS_BLOCK_RNR] = { sizeof(struct mtk_dip_params_rnr) },
};

/* one block of each type fits a params buffer */
static_assert(sizeof(struct v4l2_isp_params_buffer) +
	      sizeof(struct mtk_dip_params_blc) +
	      sizeof(struct mtk_dip_params_wb) +
	      sizeof(struct mtk_dip_params_ccm) +
	      sizeof(struct mtk_dip_params_gamma) +
	      sizeof(struct mtk_dip_params_lsc) +
	      sizeof(struct mtk_dip_params_ee) +
	      sizeof(struct mtk_dip_params_ycnr) +
	      sizeof(struct mtk_dip_params_demosaic) +
	      sizeof(struct mtk_dip_params_nbc2) +
	      sizeof(struct mtk_dip_params_nr3d) +
	      sizeof(struct mtk_dip_params_mix) +
	      sizeof(struct mtk_dip_params_radial) +
	      sizeof(struct mtk_dip_params_rnr) <= MTK_DIP_PARAMS_MAX_SIZE);
static_assert(MTK_DIP_NR3D_DXY_MAX == MTK_DIP_NR3D_MOTION_MAX);

/* the grid of the interface is the corner grid of the shading table */
static_assert(MTK_DIP_LSC_POINTS == MTK_DIP_LSC_CORNERS);
static_assert(sizeof(((struct mtk_dip_param_state *)0)->lsc_gain) ==
	      sizeof(((struct mtk_dip_params_lsc *)0)->gain));

/* Q4.12 to the 512-based unsigned 13-bit field, rounded, clamped */
static u32 mtk_dip_u13(u16 q412)
{
	return min_t(u32, (q412 + 4) >> 3, DIP_FIELD13_MAX);
}

/* signed Q4.12 to the 512-based signed 13-bit field, rounded */
static u32 mtk_dip_s13(s16 q412)
{
	s32 v = q412 >= 0 ? (q412 + 4) >> 3 : -((-(s32)q412 + 4) >> 3);

	return (u32)clamp(v, -4096, 4095) & DIP_FIELD13_MAX;
}

/* Input position of gamma point i, in 1/MTK_DIP_GAMMA_INPUT_SCALE */
static u32 mtk_dip_gamma_pos(unsigned int i)
{
	u32 pos = 0;

	if (i < MTK_DIP_GAMMA_SEG0)
		return MTK_DIP_GAMMA_STEP0 * i;
	pos += MTK_DIP_GAMMA_STEP0 * MTK_DIP_GAMMA_SEG0;
	i -= MTK_DIP_GAMMA_SEG0;
	if (i < MTK_DIP_GAMMA_SEG1)
		return pos + MTK_DIP_GAMMA_STEP1 * i;
	pos += MTK_DIP_GAMMA_STEP1 * MTK_DIP_GAMMA_SEG1;
	i -= MTK_DIP_GAMMA_SEG1;

	return pos + MTK_DIP_GAMMA_STEP2 * i;
}

static u32 mtk_dip_gamma_word(u16 r, u16 g, u16 b)
{
	return FIELD_PREP(DIP_GAMMA_R, r) | FIELD_PREP(DIP_GAMMA_G, g) |
	       FIELD_PREP(DIP_GAMMA_B, b);
}

static void mtk_dip_blc_neutral(struct mtk_dip_param_state *par)
{
	unsigned int i;

	for (i = 0; i < 4; i++) {
		par->blc[i] = 0;
		par->blc[4 + i] = DIP_BLC_GAIN_ONE;
	}
}

static void mtk_dip_wb_neutral(struct mtk_dip_param_state *par)
{
	par->wb[0] = FIELD_PREP(DIP_FIELD13_LO, DIP_WB_GAIN_ONE) |
		     FIELD_PREP(DIP_FIELD13_HI, DIP_WB_GAIN_ONE);
	par->wb[1] = par->wb[0];
}

static void mtk_dip_ccm_neutral(struct mtk_dip_param_state *par)
{
	memset(par->ccm, 0, sizeof(par->ccm));
	par->ccm[0] = FIELD_PREP(DIP_FIELD13_LO, DIP_CCM_ONE);
	par->ccm[2] = FIELD_PREP(DIP_FIELD13_HI, DIP_CCM_ONE);
	par->ccm[5] = FIELD_PREP(DIP_FIELD13_LO, DIP_CCM_ONE);
}

/* Bypass, with an identity curve in the points */
static void mtk_dip_gamma_neutral(struct mtk_dip_param_state *par)
{
	unsigned int i;
	u16 v;

	/* output scale 1023 at input scale 1024: the position is the value */
	for (i = 0; i < MTK_DIP_GAMMA_POINTS; i++) {
		v = mtk_dip_gamma_pos(i);
		par->gamma[i] = mtk_dip_gamma_word(v, v, v);
	}
	par->gamma_ctl = DIP_GAMMA_CTL_BYPASS;
}

/* Neutral settings of a module that stays in the pipeline */
static void mtk_dip_quality_neutral(struct mtk_dip_param_state *par,
				    unsigned int type)
{
	switch (type) {
	case MTK_DIP_PARAMS_BLOCK_YCNR:
		par->ycnr = false;
		mtk_dip_bq_neutral_words(type, par->ycnr_w);
		break;
	case MTK_DIP_PARAMS_BLOCK_DEMOSAIC:
		par->demosaic = false;
		mtk_dip_bq_neutral_words(type, par->demosaic_w);
		break;
	case MTK_DIP_PARAMS_BLOCK_NBC2:
		par->nbc2 = false;
		mtk_dip_bq_neutral_words(type, par->nbc2_w);
		break;
	case MTK_DIP_PARAMS_BLOCK_RADIAL:
		par->radial = false;
		mtk_dip_bq_neutral_words(type, par->radial_w);
		break;
	case MTK_DIP_PARAMS_BLOCK_RNR:
		par->rnr = false;
		mtk_dip_bq_neutral_words(type, par->rnr_w);
		break;
	}
}

/**
 * mtk_dip_params_neutral - parameters before the first job
 * @par: result
 */
void mtk_dip_params_neutral(struct mtk_dip_param_state *par)
{
	mtk_dip_blc_neutral(par);
	mtk_dip_wb_neutral(par);
	mtk_dip_ccm_neutral(par);
	mtk_dip_gamma_neutral(par);
	par->lsc = false;
	par->lsc_new = false;
	par->ee = false;
	mtk_dip_quality_neutral(par, MTK_DIP_PARAMS_BLOCK_YCNR);
	mtk_dip_quality_neutral(par, MTK_DIP_PARAMS_BLOCK_DEMOSAIC);
	mtk_dip_quality_neutral(par, MTK_DIP_PARAMS_BLOCK_NBC2);
	mtk_dip_quality_neutral(par, MTK_DIP_PARAMS_BLOCK_RADIAL);
	mtk_dip_quality_neutral(par, MTK_DIP_PARAMS_BLOCK_RNR);
	par->nr3d = false;
	par->nr3d_restart = false;
	par->nr3d_dx = 0;
	par->nr3d_dy = 0;
	/* blend: the map value is the weight of the input */
	par->mix = true;
	par->mix_w[0] = DIP_MIX_WT_BASE | DIP_MIX_Y_EN | DIP_MIX_UV_EN |
			FIELD_PREP(DIP_MIX_WT_MIN, 0) |
			FIELD_PREP(DIP_MIX_WT_MAX, 255) |
			FIELD_PREP(DIP_MIX_SLOPE, 1);
	par->mix_w[1] = FIELD_PREP(DIP_MIX_MAP_START, 0) |
			FIELD_PREP(DIP_MIX_MAP_END, 255);
}

static int mtk_dip_mix_check(const struct mtk_dip_params_mix *mx)
{
	unsigned int i;

	if (mx->luma_enable > 1 || mx->chroma_enable > 1 ||
	    mx->luma_enable != mx->chroma_enable || mx->weight_to_base > 1 ||
	    mx->luma_pass_base > 1 || mx->chroma_pass_base > 1 ||
	    mx->weight_min > mx->weight_max)
		return -EINVAL;
	for (i = 0; i < ARRAY_SIZE(mx->reserved); i++)
		if (mx->reserved[i])
			return -EINVAL;

	return 0;
}

static void mtk_dip_apply_mix(struct mtk_dip_param_state *par,
			      const struct mtk_dip_params_mix *mx)
{
	par->mix_w[0] = (mx->weight_to_base ? DIP_MIX_WT_BASE : 0) |
			(mx->luma_enable ? DIP_MIX_Y_EN : 0) |
			(mx->luma_pass_base ? DIP_MIX_Y_PASS_BASE : 0) |
			(mx->chroma_enable ? DIP_MIX_UV_EN : 0) |
			(mx->chroma_pass_base ? DIP_MIX_UV_PASS_BASE : 0) |
			FIELD_PREP(DIP_MIX_WT_MIN, mx->weight_min) |
			FIELD_PREP(DIP_MIX_WT_MAX, mx->weight_max) |
			FIELD_PREP(DIP_MIX_SLOPE, mx->slope);
	par->mix_w[1] = FIELD_PREP(DIP_MIX_MAP_START, mx->map_start) |
			FIELD_PREP(DIP_MIX_MAP_END, mx->map_end);
}

static int mtk_dip_block_check(const struct v4l2_isp_params_block_header *h)
{
	const struct mtk_dip_params_gamma *gm;
	const struct mtk_dip_params_ccm *ccm;
	unsigned int c, i;

	if (h->flags & ~(MTK_DIP_PARAMS_BLOCK_FL_DISABLE |
			 MTK_DIP_PARAMS_BLOCK_FL_ENABLE) ||
	    h->type == MTK_DIP_PARAMS_BLOCK_NBC2 + 1)
		return -EINVAL;
	if (h->flags & MTK_DIP_PARAMS_BLOCK_FL_DISABLE)
		return 0;

	switch (h->type) {
	case MTK_DIP_PARAMS_BLOCK_CCM:
		ccm = container_of(h, struct mtk_dip_params_ccm, header);
		for (i = 0; i < ARRAY_SIZE(ccm->reserved); i++)
			if (ccm->reserved[i])
				return -EINVAL;
		break;
	case MTK_DIP_PARAMS_BLOCK_GAMMA:
		gm = container_of(h, struct mtk_dip_params_gamma, header);
		for (c = 0; c < 3; c++)
			for (i = 0; i < MTK_DIP_GAMMA_POINTS; i++)
				if (gm->lut[c][i] > DIP_GAMMA_OUT_MAX)
					return -EINVAL;
		break;
	case MTK_DIP_PARAMS_BLOCK_EE:
	case MTK_DIP_PARAMS_BLOCK_YCNR:
	case MTK_DIP_PARAMS_BLOCK_DEMOSAIC:
	case MTK_DIP_PARAMS_BLOCK_NBC2:
	case MTK_DIP_PARAMS_BLOCK_NR3D:
	case MTK_DIP_PARAMS_BLOCK_RADIAL:
	case MTK_DIP_PARAMS_BLOCK_RNR:
		return mtk_dip_bq_check(h);
	case MTK_DIP_PARAMS_BLOCK_MIX:
		return mtk_dip_mix_check(container_of(h,
						      struct mtk_dip_params_mix,
						      header));
	}

	return 0;
}

/**
 * mtk_dip_params_check - copy and check a queued params buffer
 * @dip: the device
 * @vb: the buffer
 * @buf: its driver part; gets the checked copy
 *
 * Return: 0 or -EINVAL.
 */
int mtk_dip_params_check(struct mtk_dip *dip, struct vb2_buffer *vb,
			 struct mtk_dip_buffer *buf)
{
	const struct v4l2_isp_params_buffer *p = (const void *)buf->params;
	const struct v4l2_isp_params_block_header *h;
	size_t len = vb2_get_plane_payload(vb, 0);
	size_t off;
	int ret;

	ret = v4l2_isp_params_validate_buffer_size(dip->dev, vb,
						   MTK_DIP_PARAMS_MAX_SIZE);
	if (ret)
		return ret;

	/* user space cannot change the copy behind the checks */
	memcpy(buf->params, vb2_plane_vaddr(vb, 0), len);

	ret = v4l2_isp_params_validate_buffer(dip->dev, vb, p,
					      mtk_dip_block_info,
					      ARRAY_SIZE(mtk_dip_block_info));
	if (ret)
		return ret;
	if (p->version != MTK_DIP_PARAMS_VERSION_V1)
		return -EINVAL;

	for (off = 0; off < p->data_size; off += h->size) {
		h = (const void *)(p->data + off);
		ret = mtk_dip_block_check(h);
		if (ret) {
			dev_dbg(dip->dev, "params: block %u at %zu refused\n",
				h->type, off);
			return ret;
		}
	}

	return 0;
}

static void mtk_dip_apply_blc(struct mtk_dip_param_state *par,
			      const struct mtk_dip_params_blc *b)
{
	unsigned int c, reg;
	s32 off;

	for (c = 0; c < MTK_DIP_BAYER_CHANNELS; c++) {
		reg = mtk_dip_blc_reg[c];
		/* Q0.16 of full scale to the 12-bit internal data, negated */
		off = -(s32)((b->level[c] + 8) >> 4);
		par->blc[reg] = (u32)off & DIP_FIELD13_MAX;
		par->blc[4 + reg] = mtk_dip_u13(b->gain[c]);
	}
}

static void mtk_dip_apply_wb(struct mtk_dip_param_state *par,
			     const struct mtk_dip_params_wb *wb)
{
	/* B, G of the blue row; G of the red row, R */
	par->wb[0] = FIELD_PREP(DIP_FIELD13_LO, mtk_dip_u13(wb->gain[0])) |
		     FIELD_PREP(DIP_FIELD13_HI, mtk_dip_u13(wb->gain[1]));
	par->wb[1] = FIELD_PREP(DIP_FIELD13_LO, mtk_dip_u13(wb->gain[2])) |
		     FIELD_PREP(DIP_FIELD13_HI, mtk_dip_u13(wb->gain[3]));
}

static void mtk_dip_apply_ccm(struct mtk_dip_param_state *par,
			      const struct mtk_dip_params_ccm *ccm)
{
	unsigned int r;

	for (r = 0; r < 3; r++) {
		par->ccm[2 * r] =
			FIELD_PREP(DIP_FIELD13_LO, mtk_dip_s13(ccm->coeff[r][0])) |
			FIELD_PREP(DIP_FIELD13_HI, mtk_dip_s13(ccm->coeff[r][1]));
		par->ccm[2 * r + 1] =
			FIELD_PREP(DIP_FIELD13_LO, mtk_dip_s13(ccm->coeff[r][2]));
	}
}

static void mtk_dip_apply_gamma(struct mtk_dip_param_state *par,
				const struct mtk_dip_params_gamma *gm)
{
	unsigned int i;

	for (i = 0; i < MTK_DIP_GAMMA_POINTS; i++)
		par->gamma[i] = mtk_dip_gamma_word(gm->lut[0][i], gm->lut[1][i],
						   gm->lut[2][i]);
	par->gamma_ctl = DIP_GAMMA_CTL_CURVE;
}

static void mtk_dip_apply_lsc(struct mtk_dip_param_state *par,
			      const struct mtk_dip_params_lsc *lsc)
{
	memcpy(par->lsc_gain, lsc->gain, sizeof(par->lsc_gain));
	par->lsc = true;
	par->lsc_new = true;
}

static void mtk_dip_apply_nr3d(struct mtk_dip_param_state *par,
			       const struct mtk_dip_params_nr3d *nr)
{
	mtk_dip_bq_nr3d_words(nr, par->nr3d_w);
	par->nr3d_restart = nr->flags & MTK_DIP_NR3D_FL_RESTART;
	par->nr3d_dx = nr->motion_x;
	par->nr3d_dy = nr->motion_y;
}

/* White balance gain of colour @c in the words of @wb (B, Gb; Gr, R) */
static u32 mtk_dip_wb_gain(const u32 *wb, unsigned int c)
{
	return c & 1 ? FIELD_GET(DIP_FIELD13_HI, wb[c / 2]) :
		       FIELD_GET(DIP_FIELD13_LO, wb[c / 2]);
}

/**
 * mtk_dip_params_bayer - black level and white balance words for a phase
 * @par: parameters, their words for BGGR (Bayer position k is colour k)
 * @pix_id: Bayer phase of the job, MTK_DIP_PIX_ID_*
 * @blc: black level offsets and gains, in the order of @par->blc
 * @wb: white balance gain words, in the order of @par->wb
 *
 * The registers hold one value per colour, and the hardware finds the
 * pixels of a colour from the Bayer phase. The values of the params buffer
 * are per Bayer position of the frame: colour c takes those of position
 * c ^ @pix_id.
 */
void mtk_dip_params_bayer(const struct mtk_dip_param_state *par, u32 pix_id,
			  u32 *blc, u32 *wb)
{
	unsigned int c, p;
	u32 g;

	wb[0] = 0;
	wb[1] = 0;
	for (c = 0; c < MTK_DIP_BAYER_CHANNELS; c++) {
		p = c ^ (pix_id % MTK_DIP_BAYER_CHANNELS);
		blc[mtk_dip_blc_reg[c]] = par->blc[mtk_dip_blc_reg[p]];
		blc[4 + mtk_dip_blc_reg[c]] = par->blc[4 + mtk_dip_blc_reg[p]];
		g = mtk_dip_wb_gain(par->wb, p);
		wb[c / 2] |= c & 1 ? FIELD_PREP(DIP_FIELD13_HI, g) :
				     FIELD_PREP(DIP_FIELD13_LO, g);
	}
}

/**
 * mtk_dip_params_apply - take over the blocks of a checked params buffer
 * @par: parameters of the last job, updated
 * @buf: buffer checked by mtk_dip_params_check()
 */
void mtk_dip_params_apply(struct mtk_dip_param_state *par,
			  const struct mtk_dip_buffer *buf)
{
	const struct v4l2_isp_params_buffer *p = (const void *)buf->params;
	const struct v4l2_isp_params_block_header *h;
	bool off;
	size_t o;

	for (o = 0; o < p->data_size; o += h->size) {
		h = (const void *)(p->data + o);
		off = h->flags & MTK_DIP_PARAMS_BLOCK_FL_DISABLE;
		switch (h->type) {
		case MTK_DIP_PARAMS_BLOCK_BLC:
			if (off)
				mtk_dip_blc_neutral(par);
			else
				mtk_dip_apply_blc(par, (const void *)h);
			break;
		case MTK_DIP_PARAMS_BLOCK_WB:
			if (off)
				mtk_dip_wb_neutral(par);
			else
				mtk_dip_apply_wb(par, (const void *)h);
			break;
		case MTK_DIP_PARAMS_BLOCK_CCM:
			if (off)
				mtk_dip_ccm_neutral(par);
			else
				mtk_dip_apply_ccm(par, (const void *)h);
			break;
		case MTK_DIP_PARAMS_BLOCK_GAMMA:
			if (off)
				par->gamma_ctl = DIP_GAMMA_CTL_BYPASS;
			else
				mtk_dip_apply_gamma(par, (const void *)h);
			break;
		case MTK_DIP_PARAMS_BLOCK_LSC:
			if (off)
				par->lsc = false;
			else
				mtk_dip_apply_lsc(par, (const void *)h);
			break;
		case MTK_DIP_PARAMS_BLOCK_EE:
			par->ee = !off;
			if (!off)
				mtk_dip_bq_ee_words((const void *)h, par->ee_w);
			break;
		case MTK_DIP_PARAMS_BLOCK_YCNR:
			par->ycnr = !off;
			if (off)
				mtk_dip_quality_neutral(par, h->type);
			else
				mtk_dip_bq_ycnr_words((const void *)h,
						      par->ycnr_w);
			break;
		case MTK_DIP_PARAMS_BLOCK_DEMOSAIC:
			par->demosaic = !off;
			if (off)
				mtk_dip_quality_neutral(par, h->type);
			else
				mtk_dip_bq_demosaic_words((const void *)h,
							  par->demosaic_w);
			break;
		case MTK_DIP_PARAMS_BLOCK_NBC2:
			par->nbc2 = !off;
			if (off)
				mtk_dip_quality_neutral(par, h->type);
			else
				mtk_dip_bq_nbc2_words((const void *)h,
						      par->nbc2_w);
			break;
		case MTK_DIP_PARAMS_BLOCK_RADIAL:
			par->radial = !off;
			if (off)
				mtk_dip_quality_neutral(par, h->type);
			else
				mtk_dip_bq_radial_words((const void *)h,
							par->radial_w);
			break;
		case MTK_DIP_PARAMS_BLOCK_RNR:
			par->rnr = !off;
			if (off)
				mtk_dip_quality_neutral(par, h->type);
			else
				mtk_dip_bq_rnr_words((const void *)h,
						     par->rnr_w);
			break;
		case MTK_DIP_PARAMS_BLOCK_NR3D:
			par->nr3d = !off;
			if (!off)
				mtk_dip_apply_nr3d(par, (const void *)h);
			break;
		case MTK_DIP_PARAMS_BLOCK_MIX:
			par->mix = !off;
			if (!off)
				mtk_dip_apply_mix(par, (const void *)h);
			break;
		}
	}
}
