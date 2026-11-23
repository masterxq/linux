// SPDX-License-Identifier: GPL-2.0
/*
 * HEVC encoder programmed through its registers by the CPU, without
 * firmware: MT6785 and encoders of the same generation.
 *
 * The encoder core is the one of the H.264 encoder and takes one picture per
 * run. It codes Main profile with 32x32 coding tree blocks, coding blocks of
 * 16x16 and 32x32, transform blocks of 8x8 and 16x16, sample adaptive offset
 * and deblocking, one slice per picture and one reference picture. The
 * driver writes the sequence, picture and buffer registers, starts the core
 * and waits for the frame interrupt; the core writes start code, NAL unit
 * header, slice segment header and slice data and reports the length. The
 * video, sequence and picture parameter sets are written by the driver and
 * must match what the core assumes for its slice headers: one short-term
 * reference picture set {-1}, temporal motion vector prediction,
 * log2_max_pic_order_cnt_lsb 8, deblocking override enabled, three merge
 * candidates. The rate control processor of the core needs a program of the
 * vendor and is not used: the core codes every picture with the
 * quantization parameter (QP) of its registers, without QP changes inside
 * the picture, and the picture rate control shared with the H.264 encoder
 * chooses that QP.
 *
 * Reference handling as for H.264: the reconstructed picture of a run is the
 * reference of the next run; two picture buffers and two motion data buffers
 * swap with every run.
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
#define  VENC_MODE_HEVC_FRAME		0x0200ffe5	/* picture, core rate control off */
#define  VENC_MODE_FIRST		BIT(19)		/* first run of the stream */
#define VENC_QP_GOP			0x004
#define  VENC_QP_GOP_QP_I		GENMASK(31, 26)
#define  VENC_QP_GOP_INTRA_PERIOD	GENMASK(25, 14)
#define  VENC_QP_GOP_IDR_PERIOD		GENMASK(13, 0)
#define VENC_QP_SEARCH			0x008
#define  VENC_QP_SEARCH_QP_P		GENMASK(15, 10)
#define  VENC_QP_SEARCH_HEVC		0x3a0		/* search ranges */
#define VENC_TOOLS			0x00c
#define VENC_FRAME_TYPE			0x010
#define  VENC_FRAME_TYPE_SLICE		GENMASK(1, 0)
#define  VENC_FRAME_TYPE_SLICE_P	0
#define  VENC_FRAME_TYPE_SLICE_I	2
#define  VENC_FRAME_TYPE_IDR		BIT(2)
#define  VENC_FRAME_TYPE_IDR_ALT	BIT(3)		/* clear for IDR after IDR */
#define  VENC_FRAME_TYPE_PREV_P		BIT(8)		/* motion data of the previous run valid */
#define  VENC_FRAME_TYPE_REF		BIT(15)
#define VENC_ENC_ORDER			0x014
#define VENC_FRAME_IDX			0x018	/* POC = FRAME_IDX - IDR_IDX */
#define VENC_IDR_IDX			0x01c
#define VENC_REF_CFG			0x020
#define  VENC_REF_CFG_HEVC		0x200000ff
#define  VENC_REF_CFG_NUM_REF		GENMASK(19, 16)
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
#define VENC_PIC_CTU			0x02c
#define  VENC_PIC_CTU_CROP		BIT(31)
#define  VENC_PIC_CTU_PAD_R		GENMASK(30, 26)	/* luma columns */
#define  VENC_PIC_CTU_PAD_B_C		GENMASK(25, 22)	/* chroma rows */
#define  VENC_PIC_CTU_PAD_B		GENMASK(21, 17)	/* luma rows */
#define  VENC_PIC_CTU_NUM		GENMASK(16, 0)
#define VENC_SEQ			0x030
#define  VENC_SEQ_PROFILE		GENMASK(30, 29)
#define  VENC_SEQ_PROFILE_HEVC_MAIN	2
#define  VENC_SEQ_LEVEL			GENMASK(27, 22)	/* level * 10 */
#define VENC_HEVC_TOOLS			0x038
#define  VENC_HEVC_TOOLS_B13		BIT(13)
#define  VENC_HEVC_TOOLS_SPEED		GENMASK(12, 10)
#define  VENC_HEVC_TOOLS_DEBLOCK	BIT(8)
#define  VENC_HEVC_TOOLS_ONE_REF	BIT(7)
#define  VENC_HEVC_TOOLS_SAO		BIT(5)
#define  VENC_HEVC_TOOLS_SAO_LUMA	BIT(4)
#define  VENC_HEVC_TOOLS_SAO_CHROMA	BIT(3)
#define  VENC_HEVC_TOOLS_MERGE_CAND	GENMASK(2, 0)
#define VENC_RC_BITRATE			0x048
#define  VENC_RC_BITRATE_SEQ		0x01060000
#define  VENC_RC_BITRATE_PIC		0x01040000
#define  VENC_RC_BITRATE_KBPS		GENMASK(15, 0)
#define VENC_RC_FPS			0x04c
#define  VENC_RC_FPS_FPS		GENMASK(23, 16)
#define  VENC_RC_FPS_QLIM_B		GENMASK(15, 8)
#define  VENC_RC_FPS_QLIM_P		GENMASK(7, 0)
#define VENC_QP_RANGE			0x050
#define  VENC_QP_RANGE_MODE		GENMASK(31, 16)
#define  VENC_QP_RANGE_MODE_HEVC	0x200
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
#define  VENC_IRQ_EN_HEVC		0xbf
#define VENC_SW_RST			0x0a8
#define VENC_CFG_0E4			0x0e4
#define VENC_CFG_0EC			0x0ec
#define VENC_CFG_128			0x128
#define VENC_BUS_THRESHOLD		0x12c
#define  VENC_BUS_THRESHOLD_HEVC	0x600
#define VENC_EXEC_CNT			0x17c
#define VENC_CFG_180			0x180
#define VENC_CFG_184			0x184
#define VENC_HEVC_FPS			0x190
#define VENC_CFG_194			0x194
#define VENC_REF_COMPRESS		0x1c0
#define  VENC_REF_COMPRESS_OFF		0x10
#define VENC_PIC_SIZE_PX		0x100c
#define  VENC_PIC_SIZE_PX_H		GENMASK(31, 16)	/* rounded up to 16 */
#define  VENC_PIC_SIZE_PX_W		GENMASK(15, 0)
#define VENC_CFG_1010			0x1010
#define VENC_PIC_SIZE_MB		0x1024
#define  VENC_PIC_SIZE_MB_H		GENMASK(23, 12)
#define  VENC_PIC_SIZE_MB_W		GENMASK(11, 0)
#define VENC_FIXED_QP			0x10c0	/* for the rate control processor */

#define VENC_HEVC_TIMEOUT_MS		500
#define VENC_HEVC_BS_ALIGN		128
#define VENC_HEVC_ADDR_ALIGN		16
#define VENC_HEVC_MAX_QP		51
#define VENC_HEVC_CTB			32
#define VENC_HEVC_MERGE_CAND		3
#define VENC_HEVC_QLIM			6

/* HEVC syntax fixed by the core */
#define HEVC_LOG2_MAX_POC_LSB		8
#define HEVC_LOG2_MIN_CB		4
#define HEVC_LOG2_MIN_TB		3
#define HEVC_LOG2_PARALLEL_MERGE	5
#define HEVC_NAL_VPS			32
#define HEVC_NAL_SPS			33
#define HEVC_NAL_PPS			34
#define HEVC_PROFILE_MAIN		1

/**
 * struct venc_hevc_reg_inst - encoder instance
 * @ctx: context of the instance
 * @base: encoder core registers
 * @work: reconstructed pictures and motion data of the instance
 * @rec_y: luma of the two reconstructed pictures
 * @rec_c: chroma of the two reconstructed pictures
 * @mv: the two motion data buffers
 * @width: visible width
 * @height: visible height
 * @w_mb: width in units of 16 samples
 * @h_mb: height in units of 16 samples
 * @y_pitch: luma line pitch of the source in bytes
 * @c_offset: offset of the chroma plane in a one-plane source
 * @scan: source scan type of the core
 * @two_planes: luma and chroma in separate buffers
 * @level_idc: general_level_idc
 * @speed: speed setting of the core for the picture rate
 * @fps: frame rate in pictures per second, rounded
 * @idr_period: pictures from one IDR picture to the next, 0: only the first
 * @frame_idx: index of the next picture since the last start
 * @idr_idx: index of the last IDR picture
 * @runs: pictures run through the core, select the buffers of a run
 * @prev_idr: the last picture was an IDR picture
 * @idr_alt: value of the alternating IDR flag of the last IDR picture
 * @force_idr: next picture is an IDR picture
 * @prepend_once: put the parameter sets before the next IDR picture
 * @prepend_idr: put the parameter sets before every IDR picture
 * @hdr: video, sequence and picture parameter set NAL units
 * @hdr_len: length of @hdr
 * @rc: rate control
 */
struct venc_hevc_reg_inst {
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
	unsigned int level_idc;
	unsigned int speed;
	unsigned int fps;
	unsigned int idr_period;
	u32 frame_idx;
	u32 idr_idx;
	u32 runs;
	bool prev_idr;
	bool idr_alt;
	bool force_idr;
	bool prepend_once;
	bool prepend_idr;
	u8 hdr[VENC_REG_HDR_MAX];
	size_t hdr_len;
	struct venc_reg_rc rc;
};

static void venc_write(struct venc_hevc_reg_inst *inst, u32 reg, u32 val)
{
	writel(val, inst->base + reg);
}

static u32 venc_read(struct venc_hevc_reg_inst *inst, u32 reg)
{
	return readl(inst->base + reg);
}

/* ---- parameter sets ---- */

static int venc_hevc_put_nal(struct venc_hevc_reg_inst *inst,
			     const struct venc_reg_bits *b, u8 nal_type)
{
	/* nuh_layer_id 0, nuh_temporal_id_plus1 1 */
	u8 nal_hdr[2] = { nal_type << 1, 1 };

	return venc_reg_put_nal(inst->hdr, &inst->hdr_len, nal_hdr,
				sizeof(nal_hdr), b);
}

/* profile_tier_level(1, 0): Main profile, Main tier */
static void venc_hevc_write_ptl(struct venc_hevc_reg_inst *inst,
				struct venc_reg_bits *b)
{
	venc_reg_bits_put(b, 0, 2);		/* general_profile_space */
	venc_reg_bits_put(b, 0, 1);		/* general_tier_flag */
	venc_reg_bits_put(b, HEVC_PROFILE_MAIN, 5);
	/* general_profile_compatibility_flag: Main and Main 10 */
	venc_reg_bits_put(b, BIT(31 - 1) | BIT(31 - 2), 32);
	venc_reg_bits_put(b, 1, 1);		/* progressive_source */
	venc_reg_bits_put(b, 0, 1);		/* interlaced_source */
	venc_reg_bits_put(b, 1, 1);		/* non_packed_constraint */
	venc_reg_bits_put(b, 1, 1);		/* frame_only_constraint */
	venc_reg_bits_put(b, 0, 32);		/* 43 reserved bits */
	venc_reg_bits_put(b, 0, 11);
	venc_reg_bits_put(b, 0, 1);		/* general_inbld_flag */
	venc_reg_bits_put(b, inst->level_idc, 8);
}

static int venc_hevc_write_vps(struct venc_hevc_reg_inst *inst)
{
	struct venc_reg_bits b = { };

	venc_reg_bits_put(&b, 0, 4);		/* vps_video_parameter_set_id */
	venc_reg_bits_put(&b, 1, 1);		/* vps_base_layer_internal_flag */
	venc_reg_bits_put(&b, 1, 1);		/* vps_base_layer_available_flag */
	venc_reg_bits_put(&b, 0, 6);		/* vps_max_layers_minus1 */
	venc_reg_bits_put(&b, 0, 3);		/* vps_max_sub_layers_minus1 */
	venc_reg_bits_put(&b, 1, 1);		/* vps_temporal_id_nesting_flag */
	venc_reg_bits_put(&b, 0xffff, 16);	/* vps_reserved_0xffff_16bits */
	venc_hevc_write_ptl(inst, &b);
	venc_reg_bits_put(&b, 1, 1);		/* sub_layer_ordering_info */
	venc_reg_bits_ue(&b, 1);		/* vps_max_dec_pic_buffering_minus1 */
	venc_reg_bits_ue(&b, 0);		/* vps_max_num_reorder_pics */
	venc_reg_bits_ue(&b, 0);		/* vps_max_latency_increase_plus1 */
	venc_reg_bits_put(&b, 0, 6);		/* vps_max_layer_id */
	venc_reg_bits_ue(&b, 0);		/* vps_num_layer_sets_minus1 */
	venc_reg_bits_put(&b, 0, 1);		/* vps_timing_info_present_flag */
	venc_reg_bits_put(&b, 0, 1);		/* vps_extension_flag */
	venc_reg_bits_trailing(&b);

	return venc_hevc_put_nal(inst, &b, HEVC_NAL_VPS);
}

static int venc_hevc_write_sps(struct venc_hevc_reg_inst *inst)
{
	unsigned int crop_r = (inst->w_mb * 16 - inst->width) / 2;
	unsigned int crop_b = (inst->h_mb * 16 - inst->height) / 2;
	struct venc_reg_bits b = { };
	struct venc_reg_colour colour;

	venc_reg_bits_put(&b, 0, 4);		/* sps_video_parameter_set_id */
	venc_reg_bits_put(&b, 0, 3);		/* sps_max_sub_layers_minus1 */
	venc_reg_bits_put(&b, 1, 1);		/* sps_temporal_id_nesting_flag */
	venc_hevc_write_ptl(inst, &b);
	venc_reg_bits_ue(&b, 0);		/* sps_seq_parameter_set_id */
	venc_reg_bits_ue(&b, 1);		/* chroma_format_idc 4:2:0 */
	venc_reg_bits_ue(&b, inst->w_mb * 16);	/* pic_width_in_luma_samples */
	venc_reg_bits_ue(&b, inst->h_mb * 16);	/* pic_height_in_luma_samples */
	venc_reg_bits_put(&b, crop_r || crop_b, 1);	/* conformance_window */
	if (crop_r || crop_b) {
		venc_reg_bits_ue(&b, 0);
		venc_reg_bits_ue(&b, crop_r);
		venc_reg_bits_ue(&b, 0);
		venc_reg_bits_ue(&b, crop_b);
	}
	venc_reg_bits_ue(&b, 0);		/* bit_depth_luma_minus8 */
	venc_reg_bits_ue(&b, 0);		/* bit_depth_chroma_minus8 */
	venc_reg_bits_ue(&b, HEVC_LOG2_MAX_POC_LSB - 4);
	venc_reg_bits_put(&b, 1, 1);		/* sub_layer_ordering_info */
	venc_reg_bits_ue(&b, 1);		/* sps_max_dec_pic_buffering_minus1 */
	venc_reg_bits_ue(&b, 0);		/* sps_max_num_reorder_pics */
	venc_reg_bits_ue(&b, 0);		/* sps_max_latency_increase_plus1 */
	venc_reg_bits_ue(&b, HEVC_LOG2_MIN_CB - 3);
	venc_reg_bits_ue(&b, 1);		/* coding tree block 32x32 */
	venc_reg_bits_ue(&b, HEVC_LOG2_MIN_TB - 2);
	venc_reg_bits_ue(&b, 1);		/* transform blocks up to 16x16 */
	venc_reg_bits_ue(&b, 1);		/* max_transform_hierarchy_depth_inter */
	venc_reg_bits_ue(&b, 0);		/* max_transform_hierarchy_depth_intra */
	venc_reg_bits_put(&b, 0, 1);		/* scaling_list_enabled_flag */
	venc_reg_bits_put(&b, 0, 1);		/* amp_enabled_flag */
	venc_reg_bits_put(&b, 1, 1);		/* sample_adaptive_offset_enabled */
	venc_reg_bits_put(&b, 0, 1);		/* pcm_enabled_flag */
	venc_reg_bits_ue(&b, 1);		/* num_short_term_ref_pic_sets */
	venc_reg_bits_ue(&b, 1);		/* num_negative_pics */
	venc_reg_bits_ue(&b, 0);		/* num_positive_pics */
	venc_reg_bits_ue(&b, 0);		/* delta_poc_s0_minus1: -1 */
	venc_reg_bits_put(&b, 1, 1);		/* used_by_curr_pic_s0_flag */
	venc_reg_bits_put(&b, 0, 1);		/* long_term_ref_pics_present */
	venc_reg_bits_put(&b, 1, 1);		/* sps_temporal_mvp_enabled_flag */
	venc_reg_bits_put(&b, 1, 1);		/* strong_intra_smoothing */

	venc_reg_bits_put(&b, 1, 1);		/* vui_parameters_present_flag */
	venc_reg_bits_put(&b, 0, 1);		/* aspect_ratio_info_present */
	venc_reg_bits_put(&b, 0, 1);		/* overscan_info_present */
	venc_reg_colour(inst->ctx, &colour);
	venc_reg_bits_put(&b, 1, 1);		/* video_signal_type_present */
	venc_reg_bits_put(&b, 5, 3);		/* video_format: unspecified */
	venc_reg_bits_put(&b, colour.full_range, 1);
	venc_reg_bits_put(&b, 1, 1);		/* colour_description_present */
	venc_reg_bits_put(&b, colour.prim, 8);
	venc_reg_bits_put(&b, colour.xfer, 8);
	venc_reg_bits_put(&b, colour.matrix, 8);
	venc_reg_bits_put(&b, 0, 1);		/* chroma_loc_info_present */
	venc_reg_bits_put(&b, 0, 1);		/* neutral_chroma_indication */
	venc_reg_bits_put(&b, 0, 1);		/* field_seq_flag */
	venc_reg_bits_put(&b, 0, 1);		/* frame_field_info_present */
	venc_reg_bits_put(&b, 0, 1);		/* default_display_window */
	venc_reg_bits_put(&b, 0, 1);		/* vui_timing_info_present */
	venc_reg_bits_put(&b, 0, 1);		/* bitstream_restriction_flag */
	venc_reg_bits_put(&b, 0, 1);		/* sps_extension_present_flag */
	venc_reg_bits_trailing(&b);

	return venc_hevc_put_nal(inst, &b, HEVC_NAL_SPS);
}

static int venc_hevc_write_pps(struct venc_hevc_reg_inst *inst)
{
	struct venc_reg_bits b = { };

	venc_reg_bits_ue(&b, 0);		/* pps_pic_parameter_set_id */
	venc_reg_bits_ue(&b, 0);		/* pps_seq_parameter_set_id */
	venc_reg_bits_put(&b, 0, 1);		/* dependent_slice_segments */
	venc_reg_bits_put(&b, 0, 1);		/* output_flag_present_flag */
	venc_reg_bits_put(&b, 0, 3);		/* num_extra_slice_header_bits */
	venc_reg_bits_put(&b, 1, 1);		/* sign_data_hiding_enabled_flag */
	venc_reg_bits_put(&b, 0, 1);		/* cabac_init_present_flag */
	venc_reg_bits_ue(&b, 0);		/* num_ref_idx_l0_default_minus1 */
	venc_reg_bits_ue(&b, 0);		/* num_ref_idx_l1_default_minus1 */
	venc_reg_bits_se(&b, 0);		/* init_qp_minus26 */
	venc_reg_bits_put(&b, 0, 1);		/* constrained_intra_pred_flag */
	venc_reg_bits_put(&b, 0, 1);		/* transform_skip_enabled_flag */
	/* one QP per picture without the rate control of the core */
	venc_reg_bits_put(&b, 0, 1);		/* cu_qp_delta_enabled_flag */
	venc_reg_bits_se(&b, 0);		/* pps_cb_qp_offset */
	venc_reg_bits_se(&b, 0);		/* pps_cr_qp_offset */
	venc_reg_bits_put(&b, 0, 1);		/* slice_chroma_qp_offsets */
	venc_reg_bits_put(&b, 0, 1);		/* weighted_pred_flag */
	venc_reg_bits_put(&b, 0, 1);		/* weighted_bipred_flag */
	venc_reg_bits_put(&b, 0, 1);		/* transquant_bypass_enabled */
	venc_reg_bits_put(&b, 0, 1);		/* tiles_enabled_flag */
	venc_reg_bits_put(&b, 0, 1);		/* entropy_coding_sync_enabled */
	venc_reg_bits_put(&b, 1, 1);		/* loop_filter_across_slices */
	venc_reg_bits_put(&b, 1, 1);		/* deblocking_filter_control */
	venc_reg_bits_put(&b, 1, 1);		/* deblocking_filter_override */
	venc_reg_bits_put(&b, 0, 1);		/* pps_deblocking_filter_disabled */
	venc_reg_bits_se(&b, 0);		/* pps_beta_offset_div2 */
	venc_reg_bits_se(&b, 0);		/* pps_tc_offset_div2 */
	venc_reg_bits_put(&b, 0, 1);		/* pps_scaling_list_data_present */
	venc_reg_bits_put(&b, 0, 1);		/* lists_modification_present */
	venc_reg_bits_ue(&b, HEVC_LOG2_PARALLEL_MERGE - 2);
	venc_reg_bits_put(&b, 0, 1);		/* slice_segment_header_extension */
	venc_reg_bits_put(&b, 0, 1);		/* pps_extension_present_flag */
	venc_reg_bits_trailing(&b);

	return venc_hevc_put_nal(inst, &b, HEVC_NAL_PPS);
}

/*
 * HEVC Table A.8, Main tier: general_level_idc, MaxLumaPs, MaxLumaSr,
 * MaxBR (1000 bit/s for the coded picture buffer of the VCL, 1100 for NAL)
 */
static const struct {
	u8 level_idc;
	u32 max_luma_ps;
	u32 max_luma_sr;
	u32 max_br;
} venc_hevc_levels[] = {
	{ 30, 36864, 552960, 128 },
	{ 60, 122880, 3686400, 1500 },
	{ 63, 245760, 7372800, 3000 },
	{ 90, 552960, 16588800, 6000 },
	{ 93, 983040, 33177600, 10000 },
	{ 120, 2228224, 66846720, 12000 },
	{ 123, 2228224, 133693440, 20000 },
	{ 150, 8912896, 267386880, 25000 },
	{ 153, 8912896, 534773760, 40000 },
	{ 156, 8912896, 1069547520, 60000 },
	{ 180, 35651584, 1069547520, 60000 },
	{ 183, 35651584, 2139095040, 120000 },
	{ 186, 35651584, 4278190080, 240000 },
};

static unsigned int venc_hevc_v4l2_level(unsigned int level)
{
	static const u8 idc[] = {
		[V4L2_MPEG_VIDEO_HEVC_LEVEL_1] = 30,
		[V4L2_MPEG_VIDEO_HEVC_LEVEL_2] = 60,
		[V4L2_MPEG_VIDEO_HEVC_LEVEL_2_1] = 63,
		[V4L2_MPEG_VIDEO_HEVC_LEVEL_3] = 90,
		[V4L2_MPEG_VIDEO_HEVC_LEVEL_3_1] = 93,
		[V4L2_MPEG_VIDEO_HEVC_LEVEL_4] = 120,
		[V4L2_MPEG_VIDEO_HEVC_LEVEL_4_1] = 123,
		[V4L2_MPEG_VIDEO_HEVC_LEVEL_5] = 150,
		[V4L2_MPEG_VIDEO_HEVC_LEVEL_5_1] = 153,
		[V4L2_MPEG_VIDEO_HEVC_LEVEL_5_2] = 156,
		[V4L2_MPEG_VIDEO_HEVC_LEVEL_6] = 180,
		[V4L2_MPEG_VIDEO_HEVC_LEVEL_6_1] = 183,
		[V4L2_MPEG_VIDEO_HEVC_LEVEL_6_2] = 186,
	};

	return level < ARRAY_SIZE(idc) ? idc[level] : 30;
}

/*
 * Lowest level that allows picture size, sample rate and bitrate of the
 * stream, at least the level the application asked for.
 */
static unsigned int venc_hevc_level(struct venc_hevc_reg_inst *inst,
				    unsigned int v4l2_level)
{
	u32 ps = inst->w_mb * 16 * inst->h_mb * 16;
	u64 sr = DIV_ROUND_UP_ULL((u64)ps * inst->rc.fps_num,
				  inst->rc.fps_den);
	u32 max_dim = max(inst->w_mb, inst->h_mb) * 16;
	unsigned int asked = venc_hevc_v4l2_level(v4l2_level), i;

	for (i = 0; i < ARRAY_SIZE(venc_hevc_levels); i++) {
		if (venc_hevc_levels[i].level_idc < asked ||
		    ps > venc_hevc_levels[i].max_luma_ps ||
		    sr > venc_hevc_levels[i].max_luma_sr ||
		    (u64)inst->rc.bitrate >
		    (u64)venc_hevc_levels[i].max_br * 1100 ||
		    (u64)max_dim * max_dim >
		    8ULL * venc_hevc_levels[i].max_luma_ps)
			continue;
		return venc_hevc_levels[i].level_idc;
	}

	return venc_hevc_levels[ARRAY_SIZE(venc_hevc_levels) - 1].level_idc;
}

/*
 * Speed setting of the core: full search below 1920x1080, faster modes for
 * larger pictures so that 1080p and 2160p keep 30 pictures per second.
 */
static unsigned int venc_hevc_speed(struct venc_hevc_reg_inst *inst)
{
	u32 pixels = inst->width * inst->height;

	if (pixels < 1920 * 1080)
		return 0;
	if (pixels <= 1920 * 1088)
		return 2;
	return 7;
}

/* ---- encoder core ---- */

static void venc_hevc_reset(struct venc_hevc_reg_inst *inst)
{
	venc_write(inst, VENC_SW_RST, 0);
	venc_read(inst, VENC_SW_RST);
	venc_write(inst, VENC_SW_RST, 1);
	venc_read(inst, VENC_SW_RST);
}

static u32 venc_hevc_tools(struct venc_hevc_reg_inst *inst)
{
	return VENC_HEVC_TOOLS_B13 | VENC_HEVC_TOOLS_DEBLOCK |
	       VENC_HEVC_TOOLS_SAO |
	       FIELD_PREP(VENC_HEVC_TOOLS_SPEED, inst->speed) |
	       FIELD_PREP(VENC_HEVC_TOOLS_MERGE_CAND, VENC_HEVC_MERGE_CAND);
}

static void venc_hevc_set_seq(struct venc_hevc_reg_inst *inst, int qp)
{
	unsigned int idr_period = min_t(unsigned int, inst->idr_period,
					FIELD_MAX(VENC_QP_GOP_INTRA_PERIOD));
	unsigned int pad_r = inst->w_mb * 16 - inst->width;
	unsigned int pad_b = inst->h_mb * 16 - inst->height;
	u32 kbps = min_t(u32, inst->rc.bitrate / 1000,
			 FIELD_MAX(VENC_RC_BITRATE_KBPS));

	venc_write(inst, VENC_QP_GOP,
		   FIELD_PREP(VENC_QP_GOP_QP_I, qp) |
		   FIELD_PREP(VENC_QP_GOP_INTRA_PERIOD, idr_period) |
		   FIELD_PREP(VENC_QP_GOP_IDR_PERIOD, idr_period));
	venc_write(inst, VENC_QP_SEARCH, VENC_QP_SEARCH_HEVC |
		   FIELD_PREP(VENC_QP_SEARCH_QP_P, qp));
	venc_write(inst, VENC_TOOLS, 0);
	venc_write(inst, VENC_REF_CFG, VENC_REF_CFG_HEVC |
		   FIELD_PREP(VENC_REF_CFG_NUM_REF, 1));
	venc_write(inst, VENC_PIC_SIZE,
		   FIELD_PREP(VENC_PIC_SIZE_H_MB, inst->h_mb) |
		   FIELD_PREP(VENC_PIC_SIZE_W_MB, inst->w_mb));
	venc_write(inst, VENC_SRC_FMT,
		   FIELD_PREP(VENC_SRC_FMT_SCAN, inst->scan) |
		   FIELD_PREP(VENC_SRC_FMT_C_PITCH, inst->y_pitch / 16) |
		   FIELD_PREP(VENC_SRC_FMT_Y_PITCH, inst->y_pitch / 16) |
		   FIELD_PREP(VENC_SRC_FMT_PAD_R,
			      (inst->y_pitch - inst->width) % 16) |
		   FIELD_PREP(VENC_SRC_FMT_PAD_B, pad_b));
	venc_write(inst, VENC_PIC_CTU, VENC_PIC_CTU_CROP |
		   FIELD_PREP(VENC_PIC_CTU_PAD_R, pad_r) |
		   FIELD_PREP(VENC_PIC_CTU_PAD_B_C, pad_b / 2) |
		   FIELD_PREP(VENC_PIC_CTU_PAD_B, pad_b) |
		   FIELD_PREP(VENC_PIC_CTU_NUM,
			      DIV_ROUND_UP(inst->width, VENC_HEVC_CTB) *
			      DIV_ROUND_UP(inst->height, VENC_HEVC_CTB)));
	venc_write(inst, VENC_PIC_SIZE_PX,
		   FIELD_PREP(VENC_PIC_SIZE_PX_H, inst->h_mb * 16) |
		   FIELD_PREP(VENC_PIC_SIZE_PX_W, inst->width));
	venc_write(inst, VENC_CFG_1010, 0);
	venc_write(inst, VENC_PIC_SIZE_MB,
		   FIELD_PREP(VENC_PIC_SIZE_MB_H, inst->h_mb) |
		   FIELD_PREP(VENC_PIC_SIZE_MB_W, inst->w_mb));
	venc_write(inst, VENC_SEQ,
		   FIELD_PREP(VENC_SEQ_PROFILE, VENC_SEQ_PROFILE_HEVC_MAIN) |
		   FIELD_PREP(VENC_SEQ_LEVEL, inst->level_idc / 3));
	venc_write(inst, VENC_HEVC_TOOLS, venc_hevc_tools(inst));
	venc_write(inst, VENC_CFG_128, 0);
	venc_write(inst, VENC_RC_BITRATE, VENC_RC_BITRATE_SEQ |
		   FIELD_PREP(VENC_RC_BITRATE_KBPS, kbps));
	venc_write(inst, VENC_RC_FPS,
		   FIELD_PREP(VENC_RC_FPS_FPS, inst->fps) |
		   FIELD_PREP(VENC_RC_FPS_QLIM_B, 5) |
		   FIELD_PREP(VENC_RC_FPS_QLIM_P, VENC_HEVC_QLIM));
	venc_write(inst, VENC_QP_RANGE,
		   FIELD_PREP(VENC_QP_RANGE_MODE, VENC_QP_RANGE_MODE_HEVC) |
		   FIELD_PREP(VENC_QP_RANGE_MAX, inst->rc.max_qp) |
		   FIELD_PREP(VENC_QP_RANGE_MIN, inst->rc.min_qp));
	venc_write(inst, VENC_CFG_054, 0x808);
	venc_write(inst, VENC_REF_COMPRESS, VENC_REF_COMPRESS_OFF);
	venc_write(inst, VENC_BUS_THRESHOLD, VENC_BUS_THRESHOLD_HEVC);
	venc_write(inst, VENC_CFG_180, 0x80000040);
	venc_write(inst, VENC_CFG_184, 1);
	venc_write(inst, VENC_HEVC_FPS, inst->fps);
	venc_write(inst, VENC_CFG_194, 1);
	venc_write(inst, VENC_FIXED_QP, 0);
}

static void venc_hevc_set_pic(struct venc_hevc_reg_inst *inst, bool idr,
			      int qp)
{
	u32 kbps = min_t(u32, inst->rc.bitrate / 1000,
			 FIELD_MAX(VENC_RC_BITRATE_KBPS));
	u32 type = VENC_FRAME_TYPE_REF, tools;

	if (idr) {
		/* the flag alternates only between consecutive IDR pictures */
		inst->idr_alt = inst->prev_idr ? !inst->idr_alt : true;
		type |= FIELD_PREP(VENC_FRAME_TYPE_SLICE,
				   VENC_FRAME_TYPE_SLICE_I) |
			VENC_FRAME_TYPE_IDR |
			(inst->idr_alt ? VENC_FRAME_TYPE_IDR_ALT : 0);
	} else if (inst->frame_idx != inst->idr_idx + 1) {
		type |= VENC_FRAME_TYPE_PREV_P;
	}

	/* sample adaptive offset, for chroma not at the fastest speeds */
	tools = venc_hevc_tools(inst) | VENC_HEVC_TOOLS_ONE_REF |
		VENC_HEVC_TOOLS_SAO_LUMA |
		((inst->speed & ~1) != 6 ? VENC_HEVC_TOOLS_SAO_CHROMA : 0);

	venc_write(inst, VENC_QP_SEARCH, VENC_QP_SEARCH_HEVC |
		   FIELD_PREP(VENC_QP_SEARCH_QP_P, qp));
	venc_write(inst, VENC_TOOLS, 0);
	venc_write(inst, VENC_FRAME_TYPE, type);
	venc_write(inst, VENC_ENC_ORDER, inst->frame_idx);
	venc_write(inst, VENC_FRAME_IDX, inst->frame_idx);
	venc_write(inst, VENC_IDR_IDX, inst->idr_idx);
	venc_write(inst, VENC_REF_CFG, VENC_REF_CFG_HEVC |
		   FIELD_PREP(VENC_REF_CFG_NUM_REF, 1));
	venc_write(inst, VENC_HEVC_TOOLS, tools);
	venc_write(inst, VENC_RC_BITRATE, VENC_RC_BITRATE_PIC |
		   FIELD_PREP(VENC_RC_BITRATE_KBPS, kbps));
	venc_write(inst, VENC_RC_FPS,
		   FIELD_PREP(VENC_RC_FPS_FPS, inst->fps) |
		   FIELD_PREP(VENC_RC_FPS_QLIM_B, 3) |
		   FIELD_PREP(VENC_RC_FPS_QLIM_P, VENC_HEVC_QLIM));
	venc_write(inst, VENC_QP_RANGE,
		   FIELD_PREP(VENC_QP_RANGE_MODE, VENC_QP_RANGE_MODE_HEVC) |
		   FIELD_PREP(VENC_QP_RANGE_MAX, inst->rc.max_qp) |
		   FIELD_PREP(VENC_QP_RANGE_MIN, inst->rc.min_qp));
	venc_write(inst, VENC_CFG_054, 0x808);
	venc_write(inst, VENC_CFG_0E4, 0);
}

static void venc_hevc_set_bufs(struct venc_hevc_reg_inst *inst,
			       dma_addr_t src_y, dma_addr_t src_c,
			       dma_addr_t bs, size_t bs_size)
{
	unsigned int rec = inst->runs & 1, ref = rec ^ 1;

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

static int venc_hevc_run(struct venc_hevc_reg_inst *inst, u32 *bytes,
			 size_t bs_size)
{
	struct mtk_vcodec_enc_ctx *ctx = inst->ctx;
	unsigned int status = 0;
	int ret = 0;

	ctx->irq_status = 0;
	venc_write(inst, VENC_IRQ_EN, VENC_IRQ_EN_HEVC);
	venc_write(inst, VENC_MODE, VENC_MODE_HEVC_FRAME |
		   (inst->runs ? 0 : VENC_MODE_FIRST));
	venc_write(inst, VENC_EXEC_CNT, 0);
	venc_write(inst, VENC_CFG_0EC, 1);
	venc_write(inst, VENC_START, VENC_START_FRAME);

	if (mtk_vcodec_wait_for_done_ctx(ctx, MTK_INST_IRQ_RECEIVED,
					 VENC_HEVC_TIMEOUT_MS, 0)) {
		mtk_venc_err(ctx, "timeout, status %#x, %u bytes",
			     venc_read(inst, MTK_VENC_IRQ_STATUS_OFFSET),
			     venc_read(inst, VENC_PIC_BITSTREAM_BYTE_CNT));
		ret = -ETIMEDOUT;
		goto err_stop;
	}

	status = ctx->irq_status;
	if (status != MTK_VENC_IRQ_STATUS_FRM) {
		mtk_venc_err(ctx, "irq status %#x", status);
		ret = status & MTK_VENC_IRQ_STATUS_DRAM ? -ENOSPC : -EIO;
		goto err_stop;
	}

	*bytes = venc_read(inst, VENC_PIC_BITSTREAM_BYTE_CNT);
	venc_write(inst, VENC_HEVC_TOOLS, 0);
	venc_write(inst, VENC_CFG_0EC, 0);
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
	return ret;
}

static int venc_hevc_encode_frame(struct venc_hevc_reg_inst *inst,
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
	if (!IS_ALIGNED(src_y | src_c, VENC_HEVC_ADDR_ALIGN) ||
	    !IS_ALIGNED(bs_buf->dma_addr, VENC_HEVC_BS_ALIGN)) {
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
		hdr = ALIGN(inst->hdr_len, VENC_HEVC_BS_ALIGN);
		if (hdr >= bs_buf->size)
			return -ENOSPC;
		memcpy(bs_buf->va, inst->hdr, inst->hdr_len);
		memset(bs_buf->va + inst->hdr_len, 0, hdr - inst->hdr_len);
	}
	bs_size = ALIGN_DOWN(bs_buf->size - hdr, VENC_HEVC_BS_ALIGN);
	if (!bs_size)
		return -ENOSPC;

	/* a new IDR picture restarts the picture order count */
	if (idr)
		inst->idr_idx = inst->frame_idx;
	qp = venc_reg_rc_qp(&inst->rc, idr, inst->idr_period == 1);

	venc_hevc_reset(inst);
	venc_hevc_set_seq(inst, qp);
	venc_hevc_set_pic(inst, idr, qp);
	venc_hevc_set_bufs(inst, src_y, src_c, bs_buf->dma_addr + hdr,
			   bs_size);

	enable_irq(ctx->dev->enc_irq);
	ret = venc_hevc_run(inst, &bytes, bs_size);
	disable_irq(ctx->dev->enc_irq);
	inst->runs++;
	inst->prev_idr = idr;

	if (ret) {
		/* the reference is lost: start again with an IDR picture */
		inst->force_idr = true;
		inst->prev_idr = false;
		if (ret == -ENOSPC)
			venc_reg_rc_overflow(&inst->rc, idr);
		return ret;
	}

	venc_reg_rc_update(&inst->rc, idr, qp, (hdr + bytes) * 8);
	if (idr) {
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

static int venc_hevc_reg_encode(void *handle, enum venc_start_opt opt,
				struct venc_frm_buf *frm_buf,
				struct mtk_vcodec_mem *bs_buf,
				struct venc_done_result *result)
{
	struct venc_hevc_reg_inst *inst = handle;

	switch (opt) {
	case VENC_START_OPT_ENCODE_SEQUENCE_HEADER:
		if (inst->hdr_len > bs_buf->size)
			return -ENOSPC;
		memcpy(bs_buf->va, inst->hdr, inst->hdr_len);
		result->bs_size = inst->hdr_len;
		result->is_key_frm = false;
		return 0;
	case VENC_START_OPT_ENCODE_FRAME:
		return venc_hevc_encode_frame(inst, frm_buf, bs_buf, result);
	default:
		return -EINVAL;
	}
}

static void venc_hevc_free_work(struct venc_hevc_reg_inst *inst)
{
	if (inst->work.va)
		mtk_vcodec_mem_free(inst->ctx, &inst->work);
}

/*
 * Two reconstructed pictures, luma and half of it for chroma, and two
 * motion data buffers of 16 bytes per coding tree block. The core writes
 * whole coding tree blocks; the sizes are rounded up to 64 samples, as the
 * core rounds the width of the motion data to two blocks.
 */
static int venc_hevc_alloc_work(struct venc_hevc_reg_inst *inst)
{
	size_t luma = ALIGN(inst->width, 64) * ALIGN(inst->height, 64);
	size_t chroma = luma / 2;
	size_t mv = ALIGN(ALIGN(inst->width, 64) / VENC_HEVC_CTB *
			  ALIGN(inst->height, 64) / VENC_HEVC_CTB * 16, 128);
	unsigned int i;
	int ret;

	venc_hevc_free_work(inst);
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

static int venc_hevc_set_enc(struct venc_hevc_reg_inst *inst,
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

	if (param->hevc_profile != V4L2_MPEG_VIDEO_HEVC_PROFILE_MAIN ||
	    param->hevc_tier != V4L2_MPEG_VIDEO_HEVC_TIER_MAIN)
		return -EINVAL;

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

	inst->idr_period = param->gop_size ?: param->intra_period;
	inst->prepend_idr = param->prepend_sps_pps_idr;
	inst->rc.enabled = param->rc_frame;
	inst->rc.bitrate = max(param->bitrate, 1U);
	inst->rc.fps_num = param->framerate_num ?: 30;
	inst->rc.fps_den = param->framerate_denom ?: 1;
	inst->rc.min_qp = min(param->min_qp, param->max_qp);
	inst->rc.max_qp = min(param->max_qp, VENC_HEVC_MAX_QP);
	inst->rc.fixed_qp[VENC_REG_KIND_I] = param->i_qp;
	inst->rc.fixed_qp[VENC_REG_KIND_P] = param->p_qp;
	inst->rc.pixels = inst->width * inst->height;
	venc_reg_rc_init(&inst->rc);
	inst->fps = clamp_t(u32, DIV_ROUND_CLOSEST(inst->rc.fps_num,
						   inst->rc.fps_den),
			    1, FIELD_MAX(VENC_RC_FPS_FPS));
	inst->speed = venc_hevc_speed(inst);

	inst->level_idc = venc_hevc_level(inst, param->hevc_level);
	inst->hdr_len = 0;
	ret = venc_hevc_write_vps(inst);
	if (!ret)
		ret = venc_hevc_write_sps(inst);
	if (!ret)
		ret = venc_hevc_write_pps(inst);
	if (ret)
		return ret;

	inst->frame_idx = 0;
	inst->idr_idx = 0;
	inst->runs = 0;
	inst->prev_idr = false;
	inst->idr_alt = false;
	inst->force_idr = false;
	inst->prepend_once = false;

	mtk_venc_debug(ctx, "%ux%u pitch %u level %u speed %u gop %u rc %d",
		       inst->width, inst->height, inst->y_pitch,
		       inst->level_idc, inst->speed, inst->idr_period,
		       inst->rc.enabled);

	return venc_hevc_alloc_work(inst);
}

static int venc_hevc_reg_set_param(void *handle, enum venc_set_param_type type,
				   struct venc_enc_param *param)
{
	struct venc_hevc_reg_inst *inst = handle;

	switch (type) {
	case VENC_SET_PARAM_ENC:
		return venc_hevc_set_enc(inst, param);
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
		inst->fps = clamp_t(u32, DIV_ROUND_CLOSEST(inst->rc.fps_num,
							   inst->rc.fps_den),
				    1, FIELD_MAX(VENC_RC_FPS_FPS));
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

static int venc_hevc_reg_init(struct mtk_vcodec_enc_ctx *ctx)
{
	struct venc_hevc_reg_inst *inst;

	inst = kzalloc_obj(*inst);
	if (!inst)
		return -ENOMEM;

	inst->ctx = ctx;
	inst->base = ctx->dev->reg_base[VENC_SYS];
	ctx->drv_handle = inst;

	return 0;
}

static int venc_hevc_reg_deinit(void *handle)
{
	struct venc_hevc_reg_inst *inst = handle;

	venc_hevc_free_work(inst);
	kfree(inst);

	return 0;
}

const struct venc_common_if venc_hevc_reg_if = {
	.init = venc_hevc_reg_init,
	.encode = venc_hevc_reg_encode,
	.set_param = venc_hevc_reg_set_param,
	.deinit = venc_hevc_reg_deinit,
};
