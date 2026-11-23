// SPDX-License-Identifier: GPL-2.0
/*
 * Helpers of the encoders programmed through their registers by the CPU,
 * without firmware: the parameter set writer and the picture rate control
 * shared by the H.264 and HEVC encoders.
 */

#include <linux/bitops.h>
#include <linux/errno.h>
#include <linux/int_log.h>
#include <linux/kernel.h>
#include <linux/math64.h>
#include <linux/string.h>

#include "../mtk_vcodec_enc_drv.h"
#include "venc_reg_common.h"

/* rate control: QP in 1/256 units */
#define RC_QP_SHIFT			8
#define RC_QP_ONE			256	/* 1 << RC_QP_SHIFT, signed */
#define RC_MAX_STEP			3	/* QP change per picture */
#define RC_I_OFFSET			3	/* I pictures this much below P */

/* ---- parameter sets ---- */

void venc_reg_bits_put(struct venc_reg_bits *b, u32 val, unsigned int n)
{
	while (n--) {
		unsigned int byte = b->pos / 8;

		if (byte >= sizeof(b->buf)) {
			b->overflow = true;
			return;
		}
		if (val & BIT(n))
			b->buf[byte] |= BIT(7 - b->pos % 8);
		b->pos++;
	}
}

/* unsigned Exp-Golomb code ue(v) */
void venc_reg_bits_ue(struct venc_reg_bits *b, u32 val)
{
	unsigned int len = fls(val + 1);

	venc_reg_bits_put(b, 0, len - 1);
	venc_reg_bits_put(b, val + 1, len);
}

/* signed Exp-Golomb code se(v) */
void venc_reg_bits_se(struct venc_reg_bits *b, s32 val)
{
	venc_reg_bits_ue(b, val > 0 ? 2 * val - 1 : -2 * val);
}

void venc_reg_bits_trailing(struct venc_reg_bits *b)
{
	venc_reg_bits_put(b, 1, 1);
	while (b->pos % 8)
		venc_reg_bits_put(b, 0, 1);
}

/*
 * Append a NAL unit with a four-byte start code and the given NAL unit
 * header to @hdr, with emulation prevention bytes in the payload.
 */
int venc_reg_put_nal(u8 *hdr, size_t *hdr_len, const u8 *nal_hdr,
		     unsigned int nal_hdr_len, const struct venc_reg_bits *b)
{
	unsigned int len = b->pos / 8, zeros = 0, i;
	u8 *out = hdr + *hdr_len;
	u8 *end = hdr + VENC_REG_HDR_MAX;

	if (b->overflow || end - out < 4 + nal_hdr_len)
		return -ENOSPC;

	*out++ = 0;
	*out++ = 0;
	*out++ = 0;
	*out++ = 1;
	memcpy(out, nal_hdr, nal_hdr_len);
	out += nal_hdr_len;
	for (i = 0; i < len; i++) {
		if (end - out < 2)
			return -ENOSPC;
		if (zeros >= 2 && b->buf[i] <= 3) {
			*out++ = 3;
			zeros = 0;
		}
		*out++ = b->buf[i];
		zeros = b->buf[i] ? 0 : zeros + 1;
	}
	*hdr_len = out - hdr;

	return 0;
}

/* video signal type of the VUI from the colorimetry of the source */
void venc_reg_colour(struct mtk_vcodec_enc_ctx *ctx,
		     struct venc_reg_colour *colour)
{
	switch (ctx->colorspace) {
	case V4L2_COLORSPACE_REC709:
		colour->prim = 1;
		colour->xfer = 1;
		colour->matrix = 1;
		break;
	case V4L2_COLORSPACE_SMPTE170M:
		colour->prim = 6;
		colour->xfer = 6;
		colour->matrix = 6;
		break;
	case V4L2_COLORSPACE_BT2020:
		colour->prim = 9;
		colour->xfer = 14;
		colour->matrix = 9;
		break;
	default:
		/* unspecified */
		colour->prim = 2;
		colour->xfer = 2;
		colour->matrix = 2;
		break;
	}

	colour->full_range = V4L2_MAP_QUANTIZATION_DEFAULT(false,
							   ctx->colorspace,
							   ctx->ycbcr_enc) ==
			     V4L2_QUANTIZATION_FULL_RANGE;
	if (ctx->quantization != V4L2_QUANTIZATION_DEFAULT)
		colour->full_range =
			ctx->quantization == V4L2_QUANTIZATION_FULL_RANGE;
}

/* ---- rate control ---- */

/* 6 * log2(val) in 1/256 QP units: six QP steps double the bits */
static s32 rc_log(u32 val)
{
	return 6 * (s32)(intlog2(max(val, 1U)) >> (24 - RC_QP_SHIFT));
}

static u32 rc_frame_bits(struct venc_reg_rc *rc)
{
	return max_t(u64, div_u64((u64)rc->bitrate * rc->fps_den, rc->fps_num),
		     1);
}

void venc_reg_rc_init(struct venc_reg_rc *rc)
{
	rc->buffer = 0;
	memset(rc->cplx_valid, 0, sizeof(rc->cplx_valid));
	memset(rc->qp_valid, 0, sizeof(rc->qp_valid));
}

static int rc_clamp(struct venc_reg_rc *rc, int qp)
{
	return clamp_t(int, qp, rc->min_qp, rc->max_qp);
}

/*
 * QP for the next picture. The complexity of a picture kind, measured as
 * QP + 6 * log2(bits) of the last pictures, gives the QP that meets a bit
 * target. The target is the share of one picture of the bitrate, less the
 * surplus of the bits produced so far spread over one second. The change
 * from one picture to the next is limited. Without a measurement the first
 * QP comes from the bits per pixel, and I pictures in a stream with P
 * pictures take the QP of the P pictures less a fixed offset.
 */
int venc_reg_rc_qp(struct venc_reg_rc *rc, bool intra, bool all_intra)
{
	enum venc_reg_kind kind = intra ? VENC_REG_KIND_I : VENC_REG_KIND_P;
	s64 target, fps;
	int qp;

	if (!rc->enabled)
		return rc_clamp(rc, rc->fixed_qp[kind]);

	if (intra && !all_intra) {
		if (rc->qp_valid[VENC_REG_KIND_P])
			qp = rc->qp[VENC_REG_KIND_P] - RC_I_OFFSET;
		else if (rc->qp_valid[VENC_REG_KIND_I])
			qp = rc->qp[VENC_REG_KIND_I];
		else
			goto estimate;
		return rc_clamp(rc, qp);
	}

	if (!rc->cplx_valid[kind]) {
		/* first P picture after the first I picture */
		if (!intra && rc->qp_valid[VENC_REG_KIND_I])
			return rc_clamp(rc, rc->qp[VENC_REG_KIND_I] +
					RC_I_OFFSET);
		goto estimate;
	}

	fps = max_t(s64, div_u64(rc->fps_num, rc->fps_den), 1);
	target = (s64)rc_frame_bits(rc) - div64_s64(rc->buffer, fps);
	target = clamp_t(s64, target, rc_frame_bits(rc) / 4,
			 (s64)rc_frame_bits(rc) * 4);
	qp = DIV_ROUND_CLOSEST(rc->cplx[kind] - rc_log(target), RC_QP_ONE);
	if (rc->qp_valid[kind])
		qp = clamp(qp, rc->qp[kind] - RC_MAX_STEP,
			   rc->qp[kind] + RC_MAX_STEP);
	return rc_clamp(rc, qp);

estimate:
	/*
	 * About QP 30 at 1/8 bit per pixel, six steps per factor of two:
	 * QP = 12 - 6 * log2(bits per pixel).
	 */
	qp = DIV_ROUND_CLOSEST(12 * RC_QP_ONE - rc_log(rc_frame_bits(rc)) +
			       rc_log(rc->pixels), RC_QP_ONE);
	return rc_clamp(rc, qp);
}

void venc_reg_rc_update(struct venc_reg_rc *rc, bool intra, int qp, u32 bits)
{
	enum venc_reg_kind kind = intra ? VENC_REG_KIND_I : VENC_REG_KIND_P;
	s64 limit = 2 * (s64)rc->bitrate;
	s32 cplx = qp * RC_QP_ONE + rc_log(bits);

	rc->qp[kind] = qp;
	rc->qp_valid[kind] = true;
	rc->cplx[kind] = rc->cplx_valid[kind] ? (rc->cplx[kind] + cplx) / 2 :
			 cplx;
	rc->cplx_valid[kind] = true;
	rc->buffer = clamp(rc->buffer + bits - rc_frame_bits(rc), -limit,
			   limit);
}

/* the picture did not fit into the buffer: assume it was more complex */
void venc_reg_rc_overflow(struct venc_reg_rc *rc, bool intra)
{
	enum venc_reg_kind kind = intra ? VENC_REG_KIND_I : VENC_REG_KIND_P;

	if (rc->cplx_valid[kind])
		rc->cplx[kind] += 6 * RC_QP_ONE;
}
