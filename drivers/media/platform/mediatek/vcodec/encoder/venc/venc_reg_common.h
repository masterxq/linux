/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Helpers of the encoders programmed through their registers by the CPU,
 * without firmware: parameter set writer and picture rate control.
 */

#ifndef _VENC_REG_COMMON_H_
#define _VENC_REG_COMMON_H_

#include <linux/types.h>

struct mtk_vcodec_enc_ctx;

#define VENC_REG_HDR_MAX		128	/* parameter sets of a stream */

/**
 * struct venc_reg_bits - bit writer of a raw byte sequence payload
 * @buf: payload
 * @pos: bits written
 * @overflow: @buf too small
 */
struct venc_reg_bits {
	u8 buf[VENC_REG_HDR_MAX];
	unsigned int pos;
	bool overflow;
};

void venc_reg_bits_put(struct venc_reg_bits *b, u32 val, unsigned int n);
void venc_reg_bits_ue(struct venc_reg_bits *b, u32 val);
void venc_reg_bits_se(struct venc_reg_bits *b, s32 val);
void venc_reg_bits_trailing(struct venc_reg_bits *b);
int venc_reg_put_nal(u8 *hdr, size_t *hdr_len, const u8 *nal_hdr,
		     unsigned int nal_hdr_len, const struct venc_reg_bits *b);

/**
 * struct venc_reg_colour - video signal type of the stream
 * @prim: colour_primaries
 * @xfer: transfer_characteristics
 * @matrix: matrix_coefficients
 * @full_range: video_full_range_flag
 */
struct venc_reg_colour {
	u8 prim;
	u8 xfer;
	u8 matrix;
	bool full_range;
};

void venc_reg_colour(struct mtk_vcodec_enc_ctx *ctx,
		     struct venc_reg_colour *colour);

enum venc_reg_kind {
	VENC_REG_KIND_I,
	VENC_REG_KIND_P,
	VENC_REG_KINDS,
};

/**
 * struct venc_reg_rc - picture rate control
 * @enabled: QP chosen per picture for the bitrate, else fixed QP
 * @bitrate: target bitrate in bits per second
 * @fps_num: frame rate numerator
 * @fps_den: frame rate denominator
 * @min_qp: lowest QP
 * @max_qp: highest QP
 * @fixed_qp: QP of I and P pictures without rate control
 * @buffer: bits produced minus bits budgeted, spread over one second
 * @cplx: complexity per picture kind: QP + 6 * log2(bits), 1/256 units
 * @cplx_valid: @cplx holds a measurement
 * @qp: QP of the last picture per kind
 * @qp_valid: @qp holds a value
 * @pixels: luma samples per picture
 */
struct venc_reg_rc {
	bool enabled;
	u32 bitrate;
	u32 fps_num;
	u32 fps_den;
	u32 min_qp;
	u32 max_qp;
	u32 fixed_qp[VENC_REG_KINDS];
	s64 buffer;
	s32 cplx[VENC_REG_KINDS];
	bool cplx_valid[VENC_REG_KINDS];
	int qp[VENC_REG_KINDS];
	bool qp_valid[VENC_REG_KINDS];
	u32 pixels;
};

void venc_reg_rc_init(struct venc_reg_rc *rc);
int venc_reg_rc_qp(struct venc_reg_rc *rc, bool intra, bool all_intra);
void venc_reg_rc_update(struct venc_reg_rc *rc, bool intra, int qp, u32 bits);
void venc_reg_rc_overflow(struct venc_reg_rc *rc, bool intra);

#endif /* _VENC_REG_COMMON_H_ */
