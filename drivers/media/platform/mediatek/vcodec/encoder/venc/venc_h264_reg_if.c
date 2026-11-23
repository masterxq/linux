// SPDX-License-Identifier: GPL-2.0
/*
 * H.264 encoder programmed through its registers by the CPU, without
 * firmware: MT6785 and encoders of the same generation.
 *
 * The encoder core takes one picture per run. The driver writes the
 * sequence, picture and buffer registers, starts the core and waits for the
 * frame interrupt; the core writes start code, slice header and slice data
 * of one slice per picture and reports the length. Sequence and picture
 * parameter sets are written by the driver. The core has its own rate
 * control processor, which needs a program of the vendor; it is not used.
 * The core encodes with the quantization parameter (QP) given in its
 * registers, and a simple rate control in this file chooses that QP per
 * picture.
 *
 * Reference handling: one reference picture. The reconstructed picture of
 * a run is the reference of the next run; two picture buffers and two
 * motion data buffers swap with every run. The reconstructed pictures are
 * stored in the 16x32 block format of the decoder (MM21).
 */

#include <linux/bitfield.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/math64.h>
#include <linux/slab.h>

#include "../mtk_vcodec_enc_drv.h"
#include "../../common/mtk_vcodec_intr.h"
#include "../mtk_vcodec_enc.h"
#include "../venc_drv_base.h"
#include "venc_reg_common.h"

/* encoder core registers */
#define VENC_MODE			0x000
#define  VENC_MODE_FRAME		0x0200bf60	/* picture mode, core rate control off */
#define VENC_QP_GOP			0x004
#define  VENC_QP_GOP_QP_I		GENMASK(31, 26)
#define  VENC_QP_GOP_INTRA_PERIOD	GENMASK(18, 14)
#define  VENC_QP_GOP_IDR_PERIOD		GENMASK(9, 0)
#define VENC_QP_SEARCH			0x008
#define  VENC_QP_SEARCH_QP_P		GENMASK(15, 10)
#define  VENC_QP_SEARCH_DEFAULT		0x626e03ae	/* B QP 24, search range, modes */
#define VENC_TOOLS			0x00c
#define  VENC_TOOLS_DEFAULT		0x104f8000
#define  VENC_TOOLS_T8X8		0x82000000	/* 8x8 transform decisions */
#define VENC_FRAME_TYPE			0x010
#define  VENC_FRAME_TYPE_SLICE		GENMASK(1, 0)
#define  VENC_FRAME_TYPE_SLICE_P	0
#define  VENC_FRAME_TYPE_SLICE_I	2
#define  VENC_FRAME_TYPE_IDR		BIT(2)
#define  VENC_FRAME_TYPE_IDR_PIC_ID	BIT(3)
#define  VENC_FRAME_TYPE_PREV_P		BIT(8)	/* motion data of the previous run valid */
#define  VENC_FRAME_TYPE_ONE		BIT(10)
#define  VENC_FRAME_TYPE_INTRA		GENMASK(13, 12)
#define  VENC_FRAME_TYPE_REF		BIT(15)
#define  VENC_FRAME_TYPE_MODE		GENMASK(23, 20)
#define  VENC_FRAME_TYPE_HI		GENMASK(25, 24)
#define VENC_ENC_ORDER			0x014
#define VENC_FRAME_IDX			0x018	/* frame_num = FRAME_IDX - IDR_IDX */
#define VENC_IDR_IDX			0x01c
#define VENC_CFG_020			0x020
#define VENC_PIC_SIZE			0x024
#define  VENC_PIC_SIZE_H_MB		GENMASK(31, 16)
#define  VENC_PIC_SIZE_W_MB		GENMASK(15, 0)
#define VENC_SRC_FMT			0x028
#define  VENC_SRC_FMT_SCAN		GENMASK(31, 29)
#define  VENC_SRC_FMT_SCAN_NV12		0
#define  VENC_SRC_FMT_SCAN_NV21		4
#define  VENC_SRC_FMT_C_PITCH		GENMASK(28, 20)	/* 16-byte units */
#define  VENC_SRC_FMT_Y_PITCH		GENMASK(19, 11)	/* 16-byte units */
#define  VENC_SRC_FMT_PAD_R		GENMASK(10, 5)
#define  VENC_SRC_FMT_PAD_B		GENMASK(4, 0)
#define VENC_PIC_MBS			0x02c
#define VENC_SEQ			0x030
#define  VENC_SEQ_CABAC			BIT(31)
#define  VENC_SEQ_PROFILE		GENMASK(30, 29)
#define  VENC_SEQ_PROFILE_BASELINE	0
#define  VENC_SEQ_PROFILE_MAIN		1
#define  VENC_SEQ_PROFILE_HIGH		2
#define  VENC_SEQ_LEVEL			GENMASK(27, 22)
#define  VENC_SEQ_T8X8			BIT(21)
#define VENC_RC_BITRATE			0x048
#define VENC_RC_FPS			0x04c
#define VENC_QP_RANGE			0x050
#define  VENC_QP_RANGE_MODE		GENMASK(31, 16)
#define  VENC_QP_RANGE_MODE_DEFAULT	0x300
#define  VENC_QP_RANGE_MAX		GENMASK(15, 8)
#define  VENC_QP_RANGE_MIN		GENMASK(7, 0)
#define VENC_CFG_054			0x054
#define VENC_START			0x058
#define  VENC_START_FRAME		4
#define VENC_BS_ADDR			0x064	/* address >> 4 */
#define VENC_BS_SIZE			0x068	/* size >> 7 */
#define VENC_SRC_Y_ADDR			0x06c	/* all addresses >> 4 */
#define VENC_SRC_C_ADDR			0x070
#define VENC_REF_Y_ADDR			0x074
#define VENC_REF_C_ADDR			0x078
#define VENC_REC_Y_ADDR			0x07c
#define VENC_REC_C_ADDR			0x080
#define VENC_MV_RD_ADDR			0x084
#define VENC_MV_WR_ADDR			0x088
#define VENC_SRC_V_ADDR			0x094
#define VENC_PIC_BITSTREAM_BYTE_CNT	0x098
#define VENC_IRQ_EN			0x0a4
#define  VENC_IRQ_EN_ALL		GENMASK(5, 0)
#define VENC_SW_RST			0x0a8
#define VENC_CFG_0EC			0x0ec
#define VENC_HIER_IDX			0x124
#define VENC_HIER_CFG			0x128
#define VENC_CFG_12C			0x12c
#define VENC_HIER_FRAME			0x16c
#define VENC_CFG_1078			0x1078

#define VENC_H264_TIMEOUT_MS		500
#define VENC_H264_BS_ALIGN		128
#define VENC_H264_ADDR_ALIGN		16
#define VENC_H264_MAX_QP		51

/* H.264 limits used for the parameter sets */
#define H264_LOG2_MAX_FRAME_NUM		16	/* both fixed by the core */
#define H264_LOG2_MAX_POC_LSB		16
#define H264_NAL_SPS			7
#define H264_NAL_PPS			8

/**
 * struct venc_h264_reg_inst - encoder instance
 * @ctx: context of the instance
 * @base: encoder core registers
 * @work: reconstructed pictures and motion data of the instance
 * @rec_y: luma of the two reconstructed pictures
 * @rec_c: chroma of the two reconstructed pictures
 * @mv: the two motion data buffers
 * @width: visible width
 * @height: visible height
 * @w_mb: width in macroblocks
 * @h_mb: height in macroblocks
 * @y_pitch: luma line pitch of the source in bytes
 * @c_offset: offset of the chroma plane in a one-plane source
 * @scan: source scan type of the core
 * @two_planes: luma and chroma in separate buffers
 * @profile: profile of the core
 * @profile_idc: H.264 profile_idc
 * @level_idc: H.264 level_idc
 * @cabac: CABAC entropy coding
 * @t8x8: 8x8 transform
 * @idr_period: pictures from one IDR picture to the next, 0: only the first
 * @frame_idx: index of the next picture since the start
 * @idr_idx: index of the last IDR picture
 * @runs: pictures run through the core, select the buffers of a run
 * @idr_pic_id: idr_pic_id of the next IDR picture
 * @force_idr: next picture is an IDR picture
 * @prepend_once: put the parameter sets before the next IDR picture
 * @prepend_idr: put the parameter sets before every IDR picture
 * @hdr: sequence and picture parameter set NAL units
 * @hdr_len: length of @hdr
 * @rc: rate control
 */
struct venc_h264_reg_inst {
	struct mtk_vcodec_enc_ctx *ctx;
	void __iomem *base;
	struct mtk_vcodec_mem work;
	dma_addr_t rec_y[2];
	dma_addr_t rec_c[2];
	dma_addr_t mv[2];
	unsigned int width;
	unsigned int height;
	unsigned int w_mb;
	unsigned int h_mb;
	unsigned int y_pitch;
	unsigned int c_offset;
	unsigned int scan;
	bool two_planes;
	unsigned int profile;
	unsigned int profile_idc;
	unsigned int level_idc;
	bool cabac;
	bool t8x8;
	unsigned int idr_period;
	u32 frame_idx;
	u32 idr_idx;
	u32 runs;
	bool idr_pic_id;
	bool force_idr;
	bool prepend_once;
	bool prepend_idr;
	u8 hdr[VENC_REG_HDR_MAX];
	size_t hdr_len;
	struct venc_reg_rc rc;
};

static void venc_write(struct venc_h264_reg_inst *inst, u32 reg, u32 val)
{
	writel(val, inst->base + reg);
}

static u32 venc_read(struct venc_h264_reg_inst *inst, u32 reg)
{
	return readl(inst->base + reg);
}

/* ---- parameter sets ---- */

static int venc_h264_put_nal(struct venc_h264_reg_inst *inst,
			     const struct venc_reg_bits *b, u8 nal_type)
{
	u8 nal_hdr = (3 << 5) | nal_type;	/* nal_ref_idc 3 */

	return venc_reg_put_nal(inst->hdr, &inst->hdr_len, &nal_hdr, 1, b);
}

static int venc_h264_write_sps(struct venc_h264_reg_inst *inst)
{
	struct mtk_vcodec_enc_ctx *ctx = inst->ctx;
	unsigned int crop_r = (inst->w_mb * 16 - inst->width) / 2;
	unsigned int crop_b = (inst->h_mb * 16 - inst->height) / 2;
	struct venc_reg_bits b = { };
	struct venc_reg_colour colour;

	venc_reg_bits_put(&b, inst->profile_idc, 8);
	/* constraint_set1_flag: a Baseline stream of this core obeys Main */
	venc_reg_bits_put(&b, inst->profile_idc == 66 ? 0x40 : 0, 8);
	venc_reg_bits_put(&b, inst->level_idc, 8);
	venc_reg_bits_ue(&b, 0);		/* seq_parameter_set_id */
	if (inst->profile_idc == 100) {
		venc_reg_bits_ue(&b, 1);	/* chroma_format_idc 4:2:0 */
		venc_reg_bits_ue(&b, 0);	/* bit_depth_luma_minus8 */
		venc_reg_bits_ue(&b, 0);	/* bit_depth_chroma_minus8 */
		venc_reg_bits_put(&b, 0, 1);	/* qpprime_y_zero_transform_bypass */
		venc_reg_bits_put(&b, 0, 1);	/* seq_scaling_matrix_present */
	}
	venc_reg_bits_ue(&b, H264_LOG2_MAX_FRAME_NUM - 4);
	venc_reg_bits_ue(&b, 0);		/* pic_order_cnt_type */
	venc_reg_bits_ue(&b, H264_LOG2_MAX_POC_LSB - 4);
	venc_reg_bits_ue(&b, 1);		/* max_num_ref_frames */
	venc_reg_bits_put(&b, 0, 1);		/* gaps_in_frame_num_allowed */
	venc_reg_bits_ue(&b, inst->w_mb - 1);
	venc_reg_bits_ue(&b, inst->h_mb - 1);
	venc_reg_bits_put(&b, 1, 1);		/* frame_mbs_only_flag */
	venc_reg_bits_put(&b, 1, 1);		/* direct_8x8_inference_flag */
	venc_reg_bits_put(&b, crop_r || crop_b, 1);	/* frame_cropping_flag */
	if (crop_r || crop_b) {
		venc_reg_bits_ue(&b, 0);
		venc_reg_bits_ue(&b, crop_r);
		venc_reg_bits_ue(&b, 0);
		venc_reg_bits_ue(&b, crop_b);
	}

	venc_reg_bits_put(&b, 1, 1);		/* vui_parameters_present_flag */
	venc_reg_bits_put(&b, 0, 1);		/* aspect_ratio_info_present */
	venc_reg_bits_put(&b, 0, 1);		/* overscan_info_present */
	venc_reg_colour(ctx, &colour);
	venc_reg_bits_put(&b, 1, 1);		/* video_signal_type_present */
	venc_reg_bits_put(&b, 5, 3);		/* video_format: unspecified */
	venc_reg_bits_put(&b, colour.full_range, 1);
	venc_reg_bits_put(&b, 1, 1);		/* colour_description_present */
	venc_reg_bits_put(&b, colour.prim, 8);
	venc_reg_bits_put(&b, colour.xfer, 8);
	venc_reg_bits_put(&b, colour.matrix, 8);
	venc_reg_bits_put(&b, 0, 1);		/* chroma_loc_info_present */
	venc_reg_bits_put(&b, 0, 1);		/* timing_info_present */
	venc_reg_bits_put(&b, 0, 1);		/* nal_hrd_parameters_present */
	venc_reg_bits_put(&b, 0, 1);		/* vcl_hrd_parameters_present */
	venc_reg_bits_put(&b, 0, 1);		/* pic_struct_present */
	venc_reg_bits_put(&b, 1, 1);		/* bitstream_restriction_flag */
	venc_reg_bits_put(&b, 1, 1);		/* motion_vectors_over_pic_boundaries */
	venc_reg_bits_ue(&b, 0);		/* max_bytes_per_pic_denom */
	venc_reg_bits_ue(&b, 0);		/* max_bits_per_mb_denom */
	venc_reg_bits_ue(&b, 16);		/* log2_max_mv_length_horizontal */
	venc_reg_bits_ue(&b, 16);		/* log2_max_mv_length_vertical */
	venc_reg_bits_ue(&b, 0);		/* max_num_reorder_frames */
	venc_reg_bits_ue(&b, 1);		/* max_dec_frame_buffering */
	venc_reg_bits_trailing(&b);

	return venc_h264_put_nal(inst, &b, H264_NAL_SPS);
}

static int venc_h264_write_pps(struct venc_h264_reg_inst *inst)
{
	struct venc_reg_bits b = { };

	venc_reg_bits_ue(&b, 0);		/* pic_parameter_set_id */
	venc_reg_bits_ue(&b, 0);		/* seq_parameter_set_id */
	venc_reg_bits_put(&b, inst->cabac, 1);	/* entropy_coding_mode_flag */
	venc_reg_bits_put(&b, 0, 1);		/* bottom_field_pic_order_in_frame */
	venc_reg_bits_ue(&b, 0);		/* num_slice_groups_minus1 */
	/* the slice header of the core overrides it with one reference */
	venc_reg_bits_ue(&b, 1);		/* num_ref_idx_l0_default_minus1 */
	venc_reg_bits_ue(&b, 1);		/* num_ref_idx_l1_default_minus1 */
	venc_reg_bits_put(&b, 0, 1);		/* weighted_pred_flag */
	venc_reg_bits_put(&b, 0, 2);		/* weighted_bipred_idc */
	venc_reg_bits_ue(&b, 0);		/* pic_init_qp_minus26: se(0) */
	venc_reg_bits_ue(&b, 0);		/* pic_init_qs_minus26: se(0) */
	venc_reg_bits_ue(&b, 0);		/* chroma_qp_index_offset: se(0) */
	venc_reg_bits_put(&b, 1, 1);		/* deblocking_filter_control_present */
	venc_reg_bits_put(&b, 0, 1);		/* constrained_intra_pred_flag */
	venc_reg_bits_put(&b, 0, 1);		/* redundant_pic_cnt_present_flag */
	if (inst->t8x8) {
		venc_reg_bits_put(&b, 1, 1);	/* transform_8x8_mode_flag */
		venc_reg_bits_put(&b, 0, 1);	/* pic_scaling_matrix_present */
		venc_reg_bits_ue(&b, 0);	/* second_chroma_qp_index_offset */
	}
	venc_reg_bits_trailing(&b);

	return venc_h264_put_nal(inst, &b, H264_NAL_PPS);
}

/* H.264 Table A-1: level_idc, MaxMBPS, MaxFS, MaxBR (1000 or 1200 bit/s) */
static const struct {
	u8 level_idc;
	u32 max_mbps;
	u32 max_fs;
	u32 max_br;
} venc_h264_levels[] = {
	{ 10, 1485, 99, 64 },
	{ 11, 3000, 396, 192 },
	{ 12, 6000, 396, 384 },
	{ 13, 11880, 396, 768 },
	{ 20, 11880, 396, 2000 },
	{ 21, 19800, 792, 4000 },
	{ 22, 20250, 1620, 4000 },
	{ 30, 40500, 1620, 10000 },
	{ 31, 108000, 3600, 14000 },
	{ 32, 216000, 5120, 20000 },
	{ 40, 245760, 8192, 20000 },
	{ 41, 245760, 8192, 50000 },
	{ 42, 522240, 8704, 50000 },
	{ 50, 589824, 22080, 135000 },
	{ 51, 983040, 36864, 240000 },
	{ 52, 2073600, 36864, 240000 },
};

static unsigned int venc_h264_v4l2_level(unsigned int level)
{
	static const u8 idc[] = {
		[V4L2_MPEG_VIDEO_H264_LEVEL_1_0] = 10,
		[V4L2_MPEG_VIDEO_H264_LEVEL_1B] = 10,
		[V4L2_MPEG_VIDEO_H264_LEVEL_1_1] = 11,
		[V4L2_MPEG_VIDEO_H264_LEVEL_1_2] = 12,
		[V4L2_MPEG_VIDEO_H264_LEVEL_1_3] = 13,
		[V4L2_MPEG_VIDEO_H264_LEVEL_2_0] = 20,
		[V4L2_MPEG_VIDEO_H264_LEVEL_2_1] = 21,
		[V4L2_MPEG_VIDEO_H264_LEVEL_2_2] = 22,
		[V4L2_MPEG_VIDEO_H264_LEVEL_3_0] = 30,
		[V4L2_MPEG_VIDEO_H264_LEVEL_3_1] = 31,
		[V4L2_MPEG_VIDEO_H264_LEVEL_3_2] = 32,
		[V4L2_MPEG_VIDEO_H264_LEVEL_4_0] = 40,
		[V4L2_MPEG_VIDEO_H264_LEVEL_4_1] = 41,
		[V4L2_MPEG_VIDEO_H264_LEVEL_4_2] = 42,
		[V4L2_MPEG_VIDEO_H264_LEVEL_5_0] = 50,
		[V4L2_MPEG_VIDEO_H264_LEVEL_5_1] = 51,
	};

	return level < ARRAY_SIZE(idc) ? idc[level] : 10;
}

/*
 * Lowest level that allows picture size, macroblock rate and bitrate of
 * the stream, at least the level the application asked for.
 */
static unsigned int venc_h264_level(struct venc_h264_reg_inst *inst,
				    unsigned int v4l2_level)
{
	u32 fs = inst->w_mb * inst->h_mb;
	u64 mbps = DIV_ROUND_UP_ULL((u64)fs * inst->rc.fps_num,
				    inst->rc.fps_den);
	u32 br_unit = inst->profile_idc == 100 ? 1250 : 1000;
	u32 max_dim = max(inst->w_mb, inst->h_mb);
	unsigned int asked = venc_h264_v4l2_level(v4l2_level), i;

	for (i = 0; i < ARRAY_SIZE(venc_h264_levels); i++) {
		if (venc_h264_levels[i].level_idc < asked ||
		    fs > venc_h264_levels[i].max_fs ||
		    mbps > venc_h264_levels[i].max_mbps ||
		    (u64)inst->rc.bitrate >
		    (u64)venc_h264_levels[i].max_br * br_unit ||
		    max_dim * max_dim > 8 * venc_h264_levels[i].max_fs)
			continue;
		return venc_h264_levels[i].level_idc;
	}

	return venc_h264_levels[ARRAY_SIZE(venc_h264_levels) - 1].level_idc;
}

/* ---- encoder core ---- */

static void venc_h264_reset(struct venc_h264_reg_inst *inst)
{
	venc_write(inst, VENC_SW_RST, 0);
	venc_read(inst, VENC_SW_RST);
	venc_write(inst, VENC_SW_RST, 1);
	venc_read(inst, VENC_SW_RST);
}

static void venc_h264_set_regs(struct venc_h264_reg_inst *inst, bool idr,
			       int qp, dma_addr_t src_y, dma_addr_t src_c,
			       dma_addr_t bs, size_t bs_size)
{
	unsigned int idr_period = inst->idr_period < 1024 ? inst->idr_period : 0;
	unsigned int rec = inst->runs & 1, ref = rec ^ 1;
	u32 type;

	/* sequence */
	venc_write(inst, VENC_QP_GOP,
		   FIELD_PREP(VENC_QP_GOP_QP_I, qp) |
		   FIELD_PREP(VENC_QP_GOP_INTRA_PERIOD,
			      idr_period < 31 ? idr_period : 0) |
		   FIELD_PREP(VENC_QP_GOP_IDR_PERIOD, idr_period));
	venc_write(inst, VENC_QP_SEARCH, VENC_QP_SEARCH_DEFAULT |
		   FIELD_PREP(VENC_QP_SEARCH_QP_P, qp));
	venc_write(inst, VENC_TOOLS, VENC_TOOLS_DEFAULT |
		   (inst->t8x8 ? VENC_TOOLS_T8X8 : 0));
	venc_write(inst, VENC_PIC_SIZE,
		   FIELD_PREP(VENC_PIC_SIZE_H_MB, inst->h_mb) |
		   FIELD_PREP(VENC_PIC_SIZE_W_MB, inst->w_mb));
	venc_write(inst, VENC_SRC_FMT,
		   FIELD_PREP(VENC_SRC_FMT_SCAN, inst->scan) |
		   FIELD_PREP(VENC_SRC_FMT_C_PITCH, inst->y_pitch / 16) |
		   FIELD_PREP(VENC_SRC_FMT_Y_PITCH, inst->y_pitch / 16) |
		   FIELD_PREP(VENC_SRC_FMT_PAD_R,
			      (inst->y_pitch - inst->width) % 16) |
		   FIELD_PREP(VENC_SRC_FMT_PAD_B,
			      inst->h_mb * 16 - inst->height));
	venc_write(inst, VENC_PIC_MBS, inst->w_mb * inst->h_mb);
	venc_write(inst, VENC_SEQ,
		   (inst->cabac ? VENC_SEQ_CABAC : 0) |
		   FIELD_PREP(VENC_SEQ_PROFILE, inst->profile) |
		   FIELD_PREP(VENC_SEQ_LEVEL, inst->level_idc) |
		   (inst->t8x8 ? VENC_SEQ_T8X8 : 0));
	/* rate control of the core off */
	venc_write(inst, VENC_RC_BITRATE, 0);
	venc_write(inst, VENC_RC_FPS, 0);
	venc_write(inst, VENC_QP_RANGE,
		   FIELD_PREP(VENC_QP_RANGE_MODE, VENC_QP_RANGE_MODE_DEFAULT) |
		   FIELD_PREP(VENC_QP_RANGE_MAX, VENC_H264_MAX_QP) |
		   FIELD_PREP(VENC_QP_RANGE_MIN, 0));
	venc_write(inst, VENC_CFG_1078, 6);
	venc_write(inst, VENC_CFG_054, 0x808);
	venc_write(inst, VENC_CFG_12C, 0x200);

	/* picture */
	type = VENC_FRAME_TYPE_ONE | VENC_FRAME_TYPE_REF |
	       FIELD_PREP(VENC_FRAME_TYPE_MODE, 1) |
	       FIELD_PREP(VENC_FRAME_TYPE_HI, 3);
	if (idr)
		type |= FIELD_PREP(VENC_FRAME_TYPE_SLICE,
				   VENC_FRAME_TYPE_SLICE_I) |
			VENC_FRAME_TYPE_IDR |
			(inst->idr_pic_id ? VENC_FRAME_TYPE_IDR_PIC_ID : 0) |
			FIELD_PREP(VENC_FRAME_TYPE_INTRA, 3);
	else if (inst->frame_idx != inst->idr_idx + 1)
		type |= VENC_FRAME_TYPE_PREV_P;
	venc_write(inst, VENC_FRAME_TYPE, type);
	venc_write(inst, VENC_HIER_FRAME, 0);
	venc_write(inst, VENC_ENC_ORDER, inst->frame_idx);
	venc_write(inst, VENC_FRAME_IDX, inst->frame_idx);
	venc_write(inst, VENC_IDR_IDX, inst->idr_idx);
	venc_write(inst, VENC_CFG_020, 0xff);
	venc_write(inst, VENC_HIER_IDX, 0);
	venc_write(inst, VENC_HIER_CFG, 0);

	/* buffers */
	venc_write(inst, VENC_BS_SIZE, bs_size >> 7);
	venc_write(inst, VENC_BS_ADDR, bs >> 4);
	venc_write(inst, VENC_SRC_Y_ADDR, src_y >> 4);
	venc_write(inst, VENC_SRC_C_ADDR, src_c >> 4);
	venc_write(inst, VENC_SRC_V_ADDR, 0);
	venc_write(inst, VENC_REF_Y_ADDR, inst->rec_y[ref] >> 4);
	venc_write(inst, VENC_REF_C_ADDR, inst->rec_c[ref] >> 4);
	venc_write(inst, VENC_REC_Y_ADDR, inst->rec_y[rec] >> 4);
	venc_write(inst, VENC_REC_C_ADDR, inst->rec_c[rec] >> 4);
	venc_write(inst, VENC_MV_RD_ADDR, inst->mv[rec] >> 4);
	venc_write(inst, VENC_MV_WR_ADDR, inst->mv[ref] >> 4);
}

static int venc_h264_run(struct venc_h264_reg_inst *inst, u32 *bytes,
			 size_t bs_size)
{
	struct mtk_vcodec_enc_ctx *ctx = inst->ctx;
	unsigned int status = 0;

	ctx->irq_status = 0;
	venc_write(inst, VENC_IRQ_EN, VENC_IRQ_EN_ALL);
	venc_write(inst, VENC_MODE, VENC_MODE_FRAME);
	venc_write(inst, VENC_CFG_0EC, 1);
	venc_write(inst, VENC_START, VENC_START_FRAME);

	if (mtk_vcodec_wait_for_done_ctx(ctx, MTK_INST_IRQ_RECEIVED,
					 VENC_H264_TIMEOUT_MS, 0)) {
		mtk_venc_err(ctx, "timeout, status %#x, %u bytes",
			     venc_read(inst, MTK_VENC_IRQ_STATUS_OFFSET),
			     venc_read(inst, VENC_PIC_BITSTREAM_BYTE_CNT));
		goto err_stop;
	}

	status = ctx->irq_status;
	if (status != MTK_VENC_IRQ_STATUS_FRM) {
		mtk_venc_err(ctx, "irq status %#x", status);
		goto err_stop;
	}

	*bytes = venc_read(inst, VENC_PIC_BITSTREAM_BYTE_CNT);
	if (*bytes > bs_size) {
		mtk_venc_err(ctx, "%u bytes in a buffer of %zu", *bytes,
			     bs_size);
		return -ENOSPC;
	}

	return 0;

err_stop:
	/* stop the core; the next run starts with a reset as well */
	venc_write(inst, VENC_IRQ_EN, 0);
	venc_write(inst, VENC_SW_RST, 0);
	return status & MTK_VENC_IRQ_STATUS_DRAM ? -ENOSPC : -ETIMEDOUT;
}

static int venc_h264_encode_frame(struct venc_h264_reg_inst *inst,
				  struct venc_frm_buf *frm_buf,
				  struct mtk_vcodec_mem *bs_buf,
				  struct venc_done_result *result)
{
	struct mtk_vcodec_enc_ctx *ctx = inst->ctx;
	dma_addr_t src_y = frm_buf->fb_addr[0].dma_addr, src_c;
	size_t hdr = 0, bs_size;
	bool idr;
	u32 bytes = 0;
	int qp, ret;

	src_c = inst->two_planes ? frm_buf->fb_addr[1].dma_addr :
		src_y + inst->c_offset;
	if (!IS_ALIGNED(src_y | src_c, VENC_H264_ADDR_ALIGN) ||
	    !IS_ALIGNED(bs_buf->dma_addr, VENC_H264_BS_ALIGN)) {
		mtk_venc_err(ctx, "buffer address not aligned");
		return -EINVAL;
	}

	idr = inst->frame_idx == 0 || inst->force_idr ||
	      (inst->idr_period &&
	       inst->frame_idx - inst->idr_idx >= inst->idr_period);

	/*
	 * Parameter sets before the picture: the core writes from an aligned
	 * address, the gap after them is zero bytes (trailing_zero_8bits).
	 */
	if (idr && (inst->prepend_once || inst->prepend_idr)) {
		hdr = ALIGN(inst->hdr_len, VENC_H264_BS_ALIGN);
		if (hdr >= bs_buf->size)
			return -ENOSPC;
		memcpy(bs_buf->va, inst->hdr, inst->hdr_len);
		memset(bs_buf->va + inst->hdr_len, 0, hdr - inst->hdr_len);
	}
	bs_size = ALIGN_DOWN(bs_buf->size - hdr, VENC_H264_BS_ALIGN);
	if (!bs_size)
		return -ENOSPC;

	if (idr)
		inst->idr_idx = inst->frame_idx;
	qp = venc_reg_rc_qp(&inst->rc, idr, inst->idr_period == 1);

	venc_h264_reset(inst);
	venc_h264_set_regs(inst, idr, qp, src_y, src_c, bs_buf->dma_addr + hdr,
			   bs_size);

	enable_irq(ctx->dev->enc_irq);
	ret = venc_h264_run(inst, &bytes, bs_size);
	disable_irq(ctx->dev->enc_irq);
	inst->runs++;

	if (ret) {
		/* the reference is lost: start again with an IDR picture */
		inst->force_idr = true;
		if (ret == -ENOSPC)
			venc_reg_rc_overflow(&inst->rc, idr);
		return ret;
	}

	venc_reg_rc_update(&inst->rc, idr, qp, (hdr + bytes) * 8);
	if (idr) {
		inst->idr_pic_id = !inst->idr_pic_id;
		inst->prepend_once = false;
		inst->force_idr = false;
	}
	inst->frame_idx++;

	result->bs_size = hdr + bytes;
	result->is_key_frm = idr;
	mtk_venc_debug(ctx, "frame %u %s qp %d %u bytes", inst->frame_idx - 1,
		       idr ? "IDR" : "P", qp, bytes);

	return 0;
}

static int venc_h264_reg_encode(void *handle, enum venc_start_opt opt,
				struct venc_frm_buf *frm_buf,
				struct mtk_vcodec_mem *bs_buf,
				struct venc_done_result *result)
{
	struct venc_h264_reg_inst *inst = handle;

	switch (opt) {
	case VENC_START_OPT_ENCODE_SEQUENCE_HEADER:
		if (inst->hdr_len > bs_buf->size)
			return -ENOSPC;
		memcpy(bs_buf->va, inst->hdr, inst->hdr_len);
		result->bs_size = inst->hdr_len;
		result->is_key_frm = false;
		return 0;
	case VENC_START_OPT_ENCODE_FRAME:
		return venc_h264_encode_frame(inst, frm_buf, bs_buf, result);
	default:
		return -EINVAL;
	}
}

static void venc_h264_free_work(struct venc_h264_reg_inst *inst)
{
	if (inst->work.va)
		mtk_vcodec_mem_free(inst->ctx, &inst->work);
}

/*
 * Two reconstructed pictures in the block format, luma w16 x h32 and
 * chroma half of it, and two motion data buffers.
 */
static int venc_h264_alloc_work(struct venc_h264_reg_inst *inst)
{
	size_t luma = ALIGN(inst->width, 16) * ALIGN(inst->height, 32);
	size_t chroma = luma / 2;
	size_t mv = ALIGN(((((inst->width + 127) >> 2) & ~31) *
			   (ALIGN(inst->height, 32) / 16) * 2), 128);
	unsigned int i;
	int ret;

	venc_h264_free_work(inst);
	inst->work.size = 2 * (luma + chroma + mv);
	ret = mtk_vcodec_mem_alloc(inst->ctx, &inst->work);
	if (ret)
		return ret;

	for (i = 0; i < 2; i++) {
		inst->rec_y[i] = inst->work.dma_addr + i * (luma + chroma);
		inst->rec_c[i] = inst->rec_y[i] + luma;
		inst->mv[i] = inst->work.dma_addr + 2 * (luma + chroma) + i * mv;
	}

	return 0;
}

static int venc_h264_set_enc(struct venc_h264_reg_inst *inst,
			     struct venc_enc_param *param)
{
	struct mtk_vcodec_enc_ctx *ctx = inst->ctx;
	struct mtk_q_data *q = &ctx->q_data[MTK_Q_DATA_SRC];
	int ret;

	switch (param->input_yuv_fmt) {
	case VENC_YUV_FORMAT_NV12:
		inst->scan = VENC_SRC_FMT_SCAN_NV12;
		break;
	case VENC_YUV_FORMAT_NV21:
		inst->scan = VENC_SRC_FMT_SCAN_NV21;
		break;
	default:
		return -EINVAL;
	}

	inst->width = param->width;
	inst->height = param->height;
	inst->w_mb = DIV_ROUND_UP(inst->width, 16);
	inst->h_mb = DIV_ROUND_UP(inst->height, 16);
	inst->y_pitch = q->bytesperline[0];
	inst->two_planes = q->fmt->num_planes == 2;
	inst->c_offset = q->bytesperline[0] * q->coded_height;
	if (!IS_ALIGNED(inst->y_pitch, 16) ||
	    inst->y_pitch < ALIGN(inst->width, 16) ||
	    inst->y_pitch > ALIGN(inst->width, 16) + 16 ||
	    inst->height % 2 || inst->width % 2)
		return -EINVAL;

	switch (param->h264_profile) {
	case V4L2_MPEG_VIDEO_H264_PROFILE_BASELINE:
	case V4L2_MPEG_VIDEO_H264_PROFILE_CONSTRAINED_BASELINE:
		inst->profile = VENC_SEQ_PROFILE_BASELINE;
		inst->profile_idc = 66;
		inst->cabac = false;
		inst->t8x8 = false;
		break;
	case V4L2_MPEG_VIDEO_H264_PROFILE_MAIN:
		inst->profile = VENC_SEQ_PROFILE_MAIN;
		inst->profile_idc = 77;
		inst->cabac = true;
		inst->t8x8 = false;
		break;
	case V4L2_MPEG_VIDEO_H264_PROFILE_HIGH:
		inst->profile = VENC_SEQ_PROFILE_HIGH;
		inst->profile_idc = 100;
		inst->cabac = true;
		inst->t8x8 = true;
		break;
	default:
		return -EINVAL;
	}

	inst->idr_period = param->gop_size ?: param->intra_period;
	inst->prepend_idr = param->prepend_sps_pps_idr;
	inst->rc.enabled = param->rc_frame;
	inst->rc.bitrate = max(param->bitrate, 1U);
	inst->rc.fps_num = param->framerate_num ?: 30;
	inst->rc.fps_den = param->framerate_denom ?: 1;
	inst->rc.min_qp = min(param->min_qp, param->max_qp);
	inst->rc.max_qp = min(param->max_qp, VENC_H264_MAX_QP);
	inst->rc.fixed_qp[VENC_REG_KIND_I] = param->i_qp;
	inst->rc.fixed_qp[VENC_REG_KIND_P] = param->p_qp;
	inst->rc.pixels = inst->width * inst->height;
	venc_reg_rc_init(&inst->rc);

	inst->level_idc = venc_h264_level(inst, param->h264_level);
	inst->hdr_len = 0;
	ret = venc_h264_write_sps(inst);
	if (!ret)
		ret = venc_h264_write_pps(inst);
	if (ret)
		return ret;

	inst->frame_idx = 0;
	inst->idr_idx = 0;
	inst->runs = 0;
	inst->idr_pic_id = false;
	inst->force_idr = false;
	inst->prepend_once = false;

	mtk_venc_debug(ctx, "%ux%u pitch %u profile %u level %u gop %u rc %d",
		       inst->width, inst->height, inst->y_pitch,
		       inst->profile_idc, inst->level_idc, inst->idr_period,
		       inst->rc.enabled);

	return venc_h264_alloc_work(inst);
}

static int venc_h264_reg_set_param(void *handle, enum venc_set_param_type type,
				   struct venc_enc_param *param)
{
	struct venc_h264_reg_inst *inst = handle;

	switch (type) {
	case VENC_SET_PARAM_ENC:
		return venc_h264_set_enc(inst, param);
	case VENC_SET_PARAM_FORCE_INTRA:
		inst->force_idr = true;
		return 0;
	case VENC_SET_PARAM_ADJUST_BITRATE:
		inst->rc.bitrate = max(param->bitrate, 1U);
		return 0;
	case VENC_SET_PARAM_ADJUST_FRAMERATE:
		if (!param->framerate_num || !param->framerate_denom)
			return -EINVAL;
		inst->rc.fps_num = param->framerate_num;
		inst->rc.fps_den = param->framerate_denom;
		return 0;
	case VENC_SET_PARAM_GOP_SIZE:
	case VENC_SET_PARAM_INTRA_PERIOD:
		inst->idr_period = param->gop_size ?: param->intra_period;
		return 0;
	case VENC_SET_PARAM_PREPEND_HEADER:
		inst->prepend_once = true;
		return 0;
	default:
		return -EINVAL;
	}
}

static int venc_h264_reg_init(struct mtk_vcodec_enc_ctx *ctx)
{
	struct venc_h264_reg_inst *inst;

	inst = kzalloc_obj(*inst);
	if (!inst)
		return -ENOMEM;

	inst->ctx = ctx;
	inst->base = ctx->dev->reg_base[VENC_SYS];
	ctx->drv_handle = inst;

	return 0;
}

static int venc_h264_reg_deinit(void *handle)
{
	struct venc_h264_reg_inst *inst = handle;

	venc_h264_free_work(inst);
	kfree(inst);

	return 0;
}

const struct venc_common_if venc_h264_reg_if = {
	.init = venc_h264_reg_init,
	.encode = venc_h264_reg_encode,
	.set_param = venc_h264_reg_set_param,
	.deinit = venc_h264_reg_deinit,
};
