// SPDX-License-Identifier: GPL-2.0
/*
 * HEVC decoding on MediaTek decoders that the host programs through their
 * registers, without firmware (MT6785).
 *
 * One hardware run decodes a whole picture with all of its slice segments,
 * tiles and wavefront rows (frame based, Annex B start codes). The host
 * reads the header of the first slice segment through the bitstream reader
 * of the HEVC syntax block up to the start of the slice data and programs
 * the picture from it and from the V4L2 controls. The decoder reads the
 * reference picture list modification and the prediction weight table of
 * that header itself, on request, and all headers of the following slice
 * segments.
 *
 * Reference lists are given to the decoder as the initial lists
 * (RefPicListTemp0/1, repeated to 16 entries); the decoder applies the
 * list modification and the active reference count of each slice. Each
 * reference carries the identifier of its capture buffer and its picture
 * order count distance for the temporal motion vector prediction.
 *
 * Output is MM21 for 8 bits and MT2110T for 10 bits (luma and chroma in
 * 16x32 and 16x16 tiles, the low two bits of each sample behind the tiles
 * of the high bits), one motion vector buffer per capture buffer.
 * Supported: Main, Main 10 and Main Still Picture, all coding tree block
 * sizes, tiles, wavefront parallel processing, dependent slice segments,
 * scaling lists, long-term references, weighted prediction, PCM and
 * lossless coding. Not supported: 4:0:0, 4:2:2, 4:4:4 and the range
 * extensions.
 */

#include <linux/bitfield.h>
#include <linux/bitmap.h>
#include <linux/iopoll.h>
#include <linux/log2.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-dma-contig.h>

#include "../mtk_vcodec_dec.h"
#include "../../common/mtk_vcodec_intr.h"
#include "../../common/mtk_vcodec_util.h"
#include "../vdec_drv_base.h"
#include "../vdec_drv_if.h"

/* bitstream reader (VDEC_LD) */
#define VLD_BS_CTRL			0x08c
#define VLD_BS_CTRL_FETCH		BIT(20)
#define VLD_BS_CTRL_RUN			BIT(23)
#define VLD_RPTR			0x0b0
#define VLD_VSTART			0x0b4
#define VLD_VEND			0x0b8
#define VLD_FETCH_STA			0x0e8
#define VLD_FETCH_STA_READY		BIT(0)
#define VLD_VBAR			0x0ec
#define VLD_VBAR_EN			BIT(28)
#define VLD_PROC_STA			0x0f4
#define VLD_SRST_READER			BIT(8)
#define VLD_WPTR			0x110
#define VLD_SCL_ADDR			0x260
#define VLD_SCL_DATA			0x264

/* picture level of the reader (VDEC_TOP) */
#define TOP_PIC_CTB_SIZE_M1		0x068
#define TOP_PIC_PIX_SIZE		0x070
#define TOP_PIC_CTB_SIZE		0x094
#define TOP_BIT_DEPTH			0x100

/* motion compensation (VDEC_CM) */
#define MC_PIC_W			0x208
#define MC_PIC_H			0x20c
#define MC_CFG_220			0x220
#define MC_Y_ADDR_512			0x224
#define MC_C_ADDR_256			0x228
#define MC_PIC_W_16			0x22c
#define MC_CFG_238			0x238
#define MC_CFG_250			0x250
#define MC_PIC_W_16_M1			0x260
#define MC_PIC_H_16_M1			0x264
#define MC_L0_Y(i)			(0x45c + 4 * (i))
#define MC_L1_Y(i)			(0x4dc + 4 * (i))
#define MC_PITCH_16			0x980
#define MC_CFG_A60			0xa60
#define MC_CFG_AF8			0xaf8
#define MC_CFG_AF8_VAL			0x100
#define MC_YC_SEP			0xb5c
#define MC_YC_SEP_EN			BIT(0)
#define MC_L0_C(i)			(0xbe0 + 4 * (i))
#define MC_L1_C(i)			(0xc60 + 4 * (i))
#define MC_10BIT			0xcec

/* motion vectors (VDEC_AV) */
#define MV_L0_ADDR(i)			(0x000 + 4 * (i))
#define MV_L1_ADDR(i)			(0x040 + 4 * (i))
#define MV_DST_ADDR			0x20c
#define MV_CFG_210			0x210
#define MV_NO_BACKWARD			0x320
#define MV_LONG_TERM			0x324
#define MV_LT_IDS			0x328
#define MV_L0_REF(i)			(0x338 + 4 * (i))
#define MV_L1_REF(i)			(0x378 + 4 * (i))
#define MV_REF_ID			GENMASK(12, 8)
#define MV_REF_POC_DIFF			GENMASK(7, 0)

/* post-processing (VDEC_PP) */
#define PP_CFG_040			0x040
#define PP_CFG_040_VAL			0xf011
#define PP_SCL_TABLE			0x504
#define PP_SCL_EN			0x630
#define PP_SCL_DC_0			0x634
#define PP_SCL_DC_1			0x638
#define PP_CFG_B80			0xb80
#define PP_CFG_B80_LARGE		BIT(12)
#define PP_10BIT_0			0xb8c
#define PP_10BIT_1			0xb90

/* control (VDEC_MISC) */
#define MISC_CFG_004			0x004
#define MISC_CFG_004_HEVC		0x100
#define MISC_SYS_SEL			0x084
#define MISC_SYS_SEL_VAL		2
#define MISC_SPEC_CON			0x0c8
#define MISC_SPEC_CON_HEVC		0xfeff
#define MISC_FUNC_CON			0x0cc
#define MISC_FUNC_CON_HEVC		0x77e30180
#define MISC_DCM_CON			0x0ec
#define MISC_SRST			0x0f4
#define MISC_SIZE_CFG			0x160
#define MISC_SIZE_CFG_RESET		0xf1000
#define MISC_SIZE_CFG_HEVC_LARGE	0x10
#define MISC_CFG_178			0x178
#define MISC_CFG_178_HEVC		0xfffdfa60
#define MISC_CFG_1F0			0x1f0
#define MISC_CFG_1F0_RESET		0
#define MISC_CFG_1F0_HEVC_LARGE		0x1e40fff

/* HEVC syntax block */
#define HEVC_GETBITS			0x000
#define HEVC_SKIP_BITS(n)		(HEVC_GETBITS + 4 * (n))
#define HEVC_READER			0x084
#define HEVC_READER_INIT		1
#define HEVC_READER_SEARCH		0x10
#define HEVC_READER_BIT_POS		GENMASK(10, 8)
#define HEVC_BITS_COUNT			0x088
#define HEVC_BITS_COUNT_RESET		BIT(8)
#define HEVC_DEC_START			0x090
#define HEVC_PARSE			0x094
#define HEVC_PARSE_WEIGHTS		BIT(0)
#define HEVC_PARSE_LIST_MOD		BIT(4)
#define HEVC_PARSE_START_CODE		BIT(8)
#define HEVC_UE				0x098
#define HEVC_SE				0x09c
#define HEVC_SPS0			0x0a0
#define HEVC_SPS1			0x0a4
#define HEVC_SPS2			0x0a8
#define HEVC_PPS0			0x0ac
#define HEVC_PPS1			0x0b0
#define HEVC_PPS2			0x0b4
#define HEVC_PPS3			0x0b8
#define HEVC_SHDR0			0x0bc
#define HEVC_SHDR1			0x0c0
#define HEVC_SHDR2			0x0c4
#define HEVC_SHDR3			0x0c8
#define HEVC_REF_CFG			0x0cc
#define HEVC_DATA_SIZE			0x0d0
#define HEVC_CFG_0D4			0x0d4
#define HEVC_CFG_0D4_VAL		0x4011d01
#define HEVC_ERR_MASK			0x0e0
/* error conditions reported in HEVC_ERR_STA0/1 */
#define HEVC_ERR_MASK_VAL		0xfffef7fb
#define HEVC_ERR_STA0			0x0e4
#define HEVC_ERR_STA1			0x0e8
#define HEVC_CTB_POS			0x12c
#define HEVC_CTB_POS_X			GENMASK(15, 0)
#define HEVC_CTB_POS_Y			GENMASK(31, 16)
#define HEVC_TILE_COLS(i)		(0x140 + 4 * (i))
#define HEVC_TILE_CFG_1C0		0x1c0
#define HEVC_TILE_ROWS(i)		(0x180 + 4 * (i))
#define HEVC_TILE_CFG_1D0		0x1d0
#define HEVC_TILE_WORDS			16
#define HEVC_CFG_2AC			0x2ac
#define HEVC_CFG_2AC_VAL		BIT(31)

#define HEVC_SPS0_CHROMA_FORMAT		GENMASK(1, 0)
#define HEVC_SPS0_PCM			BIT(2)
#define HEVC_SPS0_LISTS_MOD		BIT(3)
#define HEVC_SPS0_AMP			BIT(4)
#define HEVC_SPS0_SAO			BIT(5)
#define HEVC_SPS0_PCM_NO_FILTER		BIT(6)
#define HEVC_SPS0_TMVP			BIT(7)
#define HEVC_SPS0_PCM_DEPTH_Y		GENMASK(11, 8)
#define HEVC_SPS0_PCM_DEPTH_C		GENMASK(15, 12)
#define HEVC_SPS0_NUM_LT_SPS		GENMASK(21, 16)
#define HEVC_SPS0_LT_PRESENT		BIT(22)
#define HEVC_SPS0_NUM_ST_RPS		GENMASK(30, 24)

#define HEVC_SPS1_LOG2_MAX_POC_LSB	GENMASK(4, 0)
#define HEVC_SPS1_ST_RPS_IDX_BITS	GENMASK(11, 8)
#define HEVC_SPS1_LT_IDX_BITS		GENMASK(15, 12)
#define HEVC_SPS1_STRONG_INTRA		BIT(16)

#define HEVC_SPS2_LOG2_MIN_CB		GENMASK(3, 0)
#define HEVC_SPS2_LOG2_CTB		GENMASK(7, 4)
#define HEVC_SPS2_LOG2_MIN_TB		GENMASK(11, 8)
#define HEVC_SPS2_LOG2_MAX_TB		GENMASK(15, 12)
#define HEVC_SPS2_LOG2_MIN_PCM		GENMASK(19, 16)
#define HEVC_SPS2_LOG2_MAX_PCM		GENMASK(23, 20)
#define HEVC_SPS2_TH_DEPTH_INTER	GENMASK(27, 24)
#define HEVC_SPS2_TH_DEPTH_INTRA	GENMASK(31, 28)

#define HEVC_PPS0_SIGN_HIDING		BIT(0)
#define HEVC_PPS0_CABAC_INIT		BIT(1)
#define HEVC_PPS0_CONSTR_INTRA		BIT(2)
#define HEVC_PPS0_TRANSFORM_SKIP	BIT(3)
#define HEVC_PPS0_WEIGHTED_PRED		BIT(4)
#define HEVC_PPS0_WEIGHTED_BIPRED	BIT(5)
#define HEVC_PPS0_OUTPUT_FLAG		BIT(6)
#define HEVC_PPS0_TRANSQUANT_BYPASS	BIT(7)
#define HEVC_PPS0_DEPENDENT_SLICES	BIT(8)
#define HEVC_PPS0_ENTROPY_SYNC		BIT(9)
#define HEVC_PPS0_SHDR_EXTENSION	BIT(11)
#define HEVC_PPS0_NUM_REF_L0		GENMASK(19, 16)
#define HEVC_PPS0_NUM_REF_L1		GENMASK(23, 20)
#define HEVC_PPS0_LOG2_MERGE_LEVEL	GENMASK(26, 24)
#define HEVC_PPS0_EXTRA_SHDR_BITS	GENMASK(30, 28)

#define HEVC_PPS1_CU_QP_DELTA		BIT(0)
#define HEVC_PPS1_CHROMA_QP_OFFSETS	BIT(1)
#define HEVC_PPS1_LOG2_MIN_QP_CU	GENMASK(7, 4)
#define HEVC_PPS1_INIT_QP		GENMASK(15, 8)
#define HEVC_PPS1_CB_QP_OFFSET		GENMASK(20, 16)
#define HEVC_PPS1_CR_QP_OFFSET		GENMASK(28, 24)

#define HEVC_PPS2_TILES			BIT(0)
#define HEVC_PPS2_TILE_COLS_M1		GENMASK(15, 8)
#define HEVC_PPS2_TILE_ROWS_M1		GENMASK(23, 16)
#define HEVC_PPS2_LF_ACROSS_TILES	BIT(24)

#define HEVC_PPS3_LF_ACROSS_SLICES	BIT(0)
#define HEVC_PPS3_DEBLOCK_CTRL		BIT(1)
#define HEVC_PPS3_DEBLOCK_OVERRIDE	BIT(2)
#define HEVC_PPS3_DEBLOCK_DISABLED	BIT(3)
#define HEVC_PPS3_BETA_OFFSET		GENMASK(8, 4)
#define HEVC_PPS3_TC_OFFSET		GENMASK(13, 9)

#define HEVC_SHDR0_ADDR_BITS		GENMASK(27, 20)
#define HEVC_SHDR0_SLICE_TYPE		GENMASK(29, 28)
#define HEVC_SHDR0_IDR			BIT(31)

#define HEVC_SHDR1_SAO_LUMA		BIT(5)
#define HEVC_SHDR1_SAO_CHROMA		BIT(6)
#define HEVC_SHDR1_CABAC_INIT		BIT(7)
#define HEVC_SHDR1_QP			GENMASK(14, 8)
#define HEVC_SHDR1_CB_QP_OFFSET		GENMASK(20, 16)
#define HEVC_SHDR1_CR_QP_OFFSET		GENMASK(28, 24)

#define HEVC_SHDR2_NUM_REF_L0		GENMASK(3, 0)
#define HEVC_SHDR2_NUM_REF_L1		GENMASK(7, 4)
#define HEVC_SHDR2_NO_REF		0xff
#define HEVC_SHDR2_P_SLICE		0xf0
#define HEVC_SHDR2_COL_REF_IDX		GENMASK(11, 8)
#define HEVC_SHDR2_MAX_MERGE		GENMASK(15, 12)
#define HEVC_SHDR2_BETA_OFFSET		GENMASK(20, 16)
#define HEVC_SHDR2_TC_OFFSET		GENMASK(25, 21)
#define HEVC_SHDR2_TMVP			BIT(26)
#define HEVC_SHDR2_COL_FROM_L0		BIT(27)
#define HEVC_SHDR2_MVD_L1_ZERO		BIT(28)
#define HEVC_SHDR2_LF_ACROSS_SLICES	BIT(29)
#define HEVC_SHDR2_DEBLOCK_DISABLED	BIT(30)

#define HEVC_SHDR3_REF_RPS_DELTAS	GENMASK(7, 0)
#define HEVC_SHDR3_RPS_DELTAS		GENMASK(15, 8)
#define HEVC_SHDR3_NUM_LT		GENMASK(23, 16)
#define HEVC_SHDR3_NUM_LT_SPS		GENMASK(31, 24)

#define HEVC_REF_CFG_NUM_TOTAL		GENMASK(3, 0)
#define HEVC_REF_CFG_ENTRY_BITS		GENMASK(7, 4)

#define HEVC_PIC_CFG			0x0f4
#define HEVC_PIC_CFG_VAL		0xffff0015
#define HEVC_PIC_CFG_NAL_TYPE		GENMASK(13, 8)

/* larger pictures than this need the large picture settings */
#define VDEC_HEVC_LARGE_PIC		(2560 * 1600)

#define VDEC_HEVC_MV_EXTRA		512
#define VDEC_HEVC_ALIGN			64

/* bytes the decoder counts behind the end of the slice data */
#define VDEC_HEVC_DATA_EXTRA		9

/* zeros the reader needs behind the bitstream */
#define VDEC_HEVC_BS_PAD		384
#define VDEC_HEVC_BS_MIN_PAD		64

#define VDEC_HEVC_MAX_FB		MTK_VDEC_DIRECT_MAX_CAPTURE
/* five bits for the picture identifier, zero is unused */
#define VDEC_HEVC_IDS			32
#define VDEC_HEVC_LIST_LEN		16
#define VDEC_HEVC_MAX_NALS		16
#define VDEC_HEVC_SCL_WORDS		248

#define VDEC_HEVC_POLL_US		10000
#define VDEC_HEVC_TIMEOUT_MS		500

enum {
	NAL_TRAIL_N = 0,
	NAL_RSV_VCL_N14 = 14,
	NAL_BLA_W_LP = 16,
	NAL_IDR_W_RADL = 19,
	NAL_IDR_N_LP = 20,
	NAL_CRA_NUT = 21,
	NAL_RSV_IRAP_23 = 23,
};

/**
 * struct vdec_hevc_direct_hdr - fields of the first slice segment header
 * @nal_type:		nal_unit_type
 * @slice_type:		slice_type
 * @sao_luma:		slice_sao_luma_flag
 * @sao_chroma:		slice_sao_chroma_flag
 * @tmvp:		slice_temporal_mvp_enabled_flag
 * @num_ref_l0:		num_ref_idx_l0_active_minus1
 * @num_ref_l1:		num_ref_idx_l1_active_minus1
 * @mvd_l1_zero:	mvd_l1_zero_flag
 * @cabac_init:		cabac_init_flag
 * @col_from_l0:	collocated_from_l0_flag
 * @col_ref_idx:	collocated_ref_idx
 * @max_merge:		MaxNumMergeCand
 * @qp_delta:		slice_qp_delta
 * @cb_qp_offset:	slice_cb_qp_offset
 * @cr_qp_offset:	slice_cr_qp_offset
 * @deblock_disabled:	slice_deblocking_filter_disabled_flag
 * @beta_offset:	slice_beta_offset_div2
 * @tc_offset:		slice_tc_offset_div2
 * @lf_across_slices:	slice_loop_filter_across_slices_enabled_flag
 * @rps_deltas:		NumDeltaPocs of a short-term set coded in the header
 * @num_lt:		num_long_term_sps + num_long_term_pics
 * @num_lt_sps:		num_long_term_sps
 */
struct vdec_hevc_direct_hdr {
	u32 nal_type;
	u32 slice_type;
	u32 sao_luma;
	u32 sao_chroma;
	u32 tmvp;
	u32 num_ref_l0;
	u32 num_ref_l1;
	u32 mvd_l1_zero;
	u32 cabac_init;
	u32 col_from_l0;
	u32 col_ref_idx;
	u32 max_merge;
	s32 qp_delta;
	s32 cb_qp_offset;
	s32 cr_qp_offset;
	u32 deblock_disabled;
	s32 beta_offset;
	s32 tc_offset;
	u32 lf_across_slices;
	u32 rps_deltas;
	u32 num_lt;
	u32 num_lt_sps;
};

/**
 * struct vdec_hevc_direct_ref - picture as the decoder sees it
 * @y:		luma address
 * @c:		chroma address
 * @mv:		motion vector buffer
 * @id:		identifier of the picture (1 to 31), see vdec_hevc_pick_id()
 * @poc:	picture order count
 * @lt:		long-term reference
 */
struct vdec_hevc_direct_ref {
	dma_addr_t y;
	dma_addr_t c;
	dma_addr_t mv;
	u32 id;
	s32 poc;
	bool lt;
};

/**
 * struct vdec_hevc_direct_inst - HEVC decoder instance
 * @ctx:	decoder context
 * @dev:	decoder device
 * @hevc:	registers of the HEVC syntax block
 * @mv_buf:	motion vector buffer of each capture buffer
 * @id:		identifier of the picture in each capture buffer
 * @sps:	sequence parameter set of the current picture
 * @pps:	picture parameter set of the current picture
 * @scaling:	scaling matrix of the current picture
 * @dec:	decode parameters of the current picture
 * @hdr:	fields of the first slice segment header
 * @dpb:	pictures of the DPB of the decode parameters
 * @l0:		initial reference list 0 (RefPicListTemp0)
 * @l1:		initial reference list 1 (RefPicListTemp1)
 * @num_refs:	NumPicTotalCurr, period of the initial lists
 * @log2_ctb:	CtbLog2SizeY
 * @ctb_w:	width in coding tree blocks
 * @ctb_h:	height in coding tree blocks
 * @ten_bit:	10-bit samples
 */
struct vdec_hevc_direct_inst {
	struct mtk_vcodec_dec_ctx *ctx;
	struct mtk_vcodec_dec_dev *dev;
	void __iomem *hevc;
	struct mtk_vcodec_mem mv_buf[VDEC_HEVC_MAX_FB];
	u8 id[VDEC_HEVC_MAX_FB];

	const struct v4l2_ctrl_hevc_sps *sps;
	const struct v4l2_ctrl_hevc_pps *pps;
	const struct v4l2_ctrl_hevc_scaling_matrix *scaling;
	const struct v4l2_ctrl_hevc_decode_params *dec;
	struct vdec_hevc_direct_hdr hdr;

	struct vdec_hevc_direct_ref dpb[V4L2_HEVC_DPB_ENTRIES_NUM_MAX];
	const struct vdec_hevc_direct_ref *l0[V4L2_HEVC_DPB_ENTRIES_NUM_MAX];
	const struct vdec_hevc_direct_ref *l1[V4L2_HEVC_DPB_ENTRIES_NUM_MAX];
	unsigned int num_refs;

	unsigned int log2_ctb;
	unsigned int ctb_w;
	unsigned int ctb_h;
	bool ten_bit;
};

static inline void vdec_write(struct vdec_hevc_direct_inst *inst, int blk,
			      u32 off, u32 val)
{
	writel(val, inst->dev->reg_base[blk] + off);
}

static inline u32 vdec_read(struct vdec_hevc_direct_inst *inst, int blk, u32 off)
{
	return readl(inst->dev->reg_base[blk] + off);
}

static inline void hevc_write(struct vdec_hevc_direct_inst *inst, u32 off, u32 val)
{
	writel(val, inst->hevc + off);
}

static inline u32 hevc_read(struct vdec_hevc_direct_inst *inst, u32 off)
{
	return readl(inst->hevc + off);
}

static int vdec_hevc_poll(struct vdec_hevc_direct_inst *inst, void __iomem *reg,
			  u32 mask, bool set, const char *what)
{
	u32 val;
	int ret;

	ret = readl_poll_timeout(reg, val, !!(val & mask) == set, 1,
				 VDEC_HEVC_POLL_US);
	if (ret)
		mtk_vdec_err(inst->ctx, "%s timeout: %#x", what, val);

	return ret;
}

static inline u32 s5(s32 v)
{
	return (u32)v & GENMASK(4, 0);
}

static unsigned int ceil_log2(unsigned int n)
{
	return n > 1 ? order_base_2(n) : 0;
}

/* ---------------- slice header through the bitstream reader ---------------- */

/* read n (0 to 32) bits */
static u32 vdec_hevc_u(struct vdec_hevc_direct_inst *inst, unsigned int n)
{
	u32 win;

	if (!n)
		return 0;
	win = hevc_read(inst, HEVC_GETBITS);
	hevc_read(inst, HEVC_SKIP_BITS(n));
	return n == 32 ? win : win >> (32 - n);
}

static void vdec_hevc_skip(struct vdec_hevc_direct_inst *inst, unsigned int n)
{
	while (n) {
		unsigned int k = min(n, 32U);

		hevc_read(inst, HEVC_SKIP_BITS(k));
		n -= k;
	}
}

static u32 vdec_hevc_ue(struct vdec_hevc_direct_inst *inst)
{
	return hevc_read(inst, HEVC_UE);
}

static s32 vdec_hevc_se(struct vdec_hevc_direct_inst *inst)
{
	return (s32)hevc_read(inst, HEVC_SE);
}

/* length in bits of the Exp-Golomb code of @v */
static unsigned int vdec_hevc_ue_bits(u32 v)
{
	return 2 * fls(v + 1) - 1;
}

static int vdec_hevc_start_code_search(struct vdec_hevc_direct_inst *inst)
{
	hevc_write(inst, HEVC_READER, HEVC_READER_SEARCH);
	hevc_write(inst, HEVC_PARSE, HEVC_PARSE_START_CODE);
	return vdec_hevc_poll(inst, inst->hevc + HEVC_PARSE, HEVC_PARSE_START_CODE,
			      false, "start code search");
}

/*
 * Move the reader behind the start code and the NAL unit header of the
 * first slice segment in the buffer. The reader stands on a start code of
 * three or four bytes after the initialisation and after a search; other
 * NAL units in front of the slice are skipped.
 */
static int vdec_hevc_find_slice(struct vdec_hevc_direct_inst *inst)
{
	struct vdec_hevc_direct_hdr *hdr = &inst->hdr;
	u32 win, type;
	int i, ret;

	for (i = 0; i < VDEC_HEVC_MAX_NALS; i++) {
		win = hevc_read(inst, HEVC_GETBITS);
		if (win == 1) {
			hevc_read(inst, HEVC_SKIP_BITS(8));
			win = hevc_read(inst, HEVC_GETBITS);
		}
		if ((win >> 8) != 1) {
			ret = vdec_hevc_start_code_search(inst);
			if (ret)
				return ret;
			win = hevc_read(inst, HEVC_GETBITS);
			if ((win >> 8) != 1)
				break;
		}

		/* 00 00 01 and the first byte of the NAL unit header */
		type = (win >> 1) & 0x3f;
		if (type <= NAL_RSV_VCL_N14 ||
		    (type >= NAL_BLA_W_LP && type <= NAL_RSV_IRAP_23)) {
			hevc_read(inst, HEVC_SKIP_BITS(8));
			hevc_read(inst, HEVC_SKIP_BITS(32));
			hdr->nal_type = type;
			return 0;
		}
		/* behind this start code, so that the search finds the next one */
		hevc_read(inst, HEVC_SKIP_BITS(32));
	}

	mtk_vdec_err(inst->ctx, "no slice in the bitstream buffer");
	return -EINVAL;
}

/*
 * st_ref_pic_set() coded in the slice header: only the number of its
 * entries is needed, the rest is skipped by its size from the decode
 * parameters.
 */
static int vdec_hevc_skip_st_rps(struct vdec_hevc_direct_inst *inst)
{
	const struct v4l2_ctrl_hevc_sps *sps = inst->sps;
	unsigned int size = inst->dec->short_term_ref_pic_set_size;
	unsigned int used = 0;
	u32 neg, pos;

	inst->hdr.rps_deltas = 0;
	if (sps->num_short_term_ref_pic_sets) {
		used = 1;
		if (vdec_hevc_u(inst, 1)) {
			/* predicted from a set of the SPS */
			vdec_hevc_skip(inst, size - used);
			return 0;
		}
	}
	neg = vdec_hevc_ue(inst);
	pos = vdec_hevc_ue(inst);
	used += vdec_hevc_ue_bits(neg) + vdec_hevc_ue_bits(pos);
	if (used > size || neg + pos > V4L2_HEVC_DPB_ENTRIES_NUM_MAX)
		return -EINVAL;
	inst->hdr.rps_deltas = neg + pos;
	vdec_hevc_skip(inst, size - used);

	return 0;
}

/* long-term part of the slice header: only the counts are needed */
static int vdec_hevc_skip_lt(struct vdec_hevc_direct_inst *inst)
{
	struct vdec_hevc_direct_hdr *hdr = &inst->hdr;
	unsigned int size = inst->dec->long_term_ref_pic_set_size;
	unsigned int used = 0;
	u32 pics;

	hdr->num_lt_sps = 0;
	if (inst->sps->num_long_term_ref_pics_sps) {
		hdr->num_lt_sps = vdec_hevc_ue(inst);
		used += vdec_hevc_ue_bits(hdr->num_lt_sps);
	}
	pics = vdec_hevc_ue(inst);
	used += vdec_hevc_ue_bits(pics);
	if (used > size || hdr->num_lt_sps + pics > 32)
		return -EINVAL;
	hdr->num_lt = hdr->num_lt_sps + pics;
	vdec_hevc_skip(inst, size - used);

	return 0;
}

static void vdec_hevc_write_shdr0(struct vdec_hevc_direct_inst *inst)
{
	u32 val;

	val = FIELD_PREP(HEVC_SHDR0_ADDR_BITS, ceil_log2(inst->ctb_w * inst->ctb_h)) |
	      FIELD_PREP(HEVC_SHDR0_SLICE_TYPE, inst->hdr.slice_type);
	if (inst->hdr.nal_type == NAL_IDR_W_RADL || inst->hdr.nal_type == NAL_IDR_N_LP)
		val |= HEVC_SHDR0_IDR;
	hevc_write(inst, HEVC_SHDR0, val);
}

static u32 vdec_hevc_shdr2_refs(struct vdec_hevc_direct_inst *inst)
{
	struct vdec_hevc_direct_hdr *hdr = &inst->hdr;

	if (hdr->slice_type == V4L2_HEVC_SLICE_TYPE_I)
		return HEVC_SHDR2_NO_REF;
	if (hdr->slice_type == V4L2_HEVC_SLICE_TYPE_P)
		return HEVC_SHDR2_P_SLICE | hdr->num_ref_l0;

	return FIELD_PREP(HEVC_SHDR2_NUM_REF_L0, hdr->num_ref_l0) |
	       FIELD_PREP(HEVC_SHDR2_NUM_REF_L1, hdr->num_ref_l1);
}

static u32 vdec_hevc_ref_cfg(struct vdec_hevc_direct_inst *inst)
{
	return FIELD_PREP(HEVC_REF_CFG_NUM_TOTAL, inst->num_refs) |
	       FIELD_PREP(HEVC_REF_CFG_ENTRY_BITS, ceil_log2(inst->num_refs));
}

/* let the decoder parse a part of the slice header at the reader position */
static int vdec_hevc_wait_parse(struct vdec_hevc_direct_inst *inst, u32 what,
				const char *name)
{
	hevc_write(inst, HEVC_PARSE, what);
	return vdec_hevc_poll(inst, inst->hevc + HEVC_PARSE, ~0, false, name);
}

/* slice_segment_header() of the first slice segment up to the slice data */
static int vdec_hevc_read_hdr(struct vdec_hevc_direct_inst *inst)
{
	const struct v4l2_ctrl_hevc_sps *sps = inst->sps;
	const struct v4l2_ctrl_hevc_pps *pps = inst->pps;
	struct vdec_hevc_direct_hdr *hdr = &inst->hdr;
	bool irap = hdr->nal_type >= NAL_BLA_W_LP && hdr->nal_type <= NAL_RSV_IRAP_23;
	bool idr = hdr->nal_type == NAL_IDR_W_RADL || hdr->nal_type == NAL_IDR_N_LP;
	bool weighted;
	u32 val, n;
	int ret;

	hevc_write(inst, HEVC_BITS_COUNT, HEVC_BITS_COUNT_RESET);
	hevc_write(inst, HEVC_BITS_COUNT, 0);

	if (!vdec_hevc_u(inst, 1)) {
		mtk_vdec_err(inst->ctx, "buffer does not start with the first slice");
		return -EINVAL;
	}
	if (irap)
		vdec_hevc_u(inst, 1);			/* no_output_of_prior_pics_flag */
	val = vdec_hevc_ue(inst);
	if (val != pps->pic_parameter_set_id)
		mtk_vdec_debug(inst->ctx, "slice refers to PPS %u, control has %u",
			       val, pps->pic_parameter_set_id);
	vdec_hevc_skip(inst, pps->num_extra_slice_header_bits);
	hdr->slice_type = vdec_hevc_ue(inst);
	if (hdr->slice_type > V4L2_HEVC_SLICE_TYPE_I)
		return -EINVAL;
	vdec_hevc_write_shdr0(inst);
	if (pps->flags & V4L2_HEVC_PPS_FLAG_OUTPUT_FLAG_PRESENT)
		vdec_hevc_u(inst, 1);			/* pic_output_flag */

	hdr->rps_deltas = 0;
	hdr->num_lt = 0;
	hdr->num_lt_sps = 0;
	hdr->tmvp = 0;
	if (!idr) {
		vdec_hevc_skip(inst, sps->log2_max_pic_order_cnt_lsb_minus4 + 4);
		if (!vdec_hevc_u(inst, 1)) {
			ret = vdec_hevc_skip_st_rps(inst);
			if (ret)
				return ret;
		} else {
			vdec_hevc_skip(inst, ceil_log2(sps->num_short_term_ref_pic_sets));
		}
		if (sps->flags & V4L2_HEVC_SPS_FLAG_LONG_TERM_REF_PICS_PRESENT) {
			ret = vdec_hevc_skip_lt(inst);
			if (ret)
				return ret;
		}
		if (sps->flags & V4L2_HEVC_SPS_FLAG_SPS_TEMPORAL_MVP_ENABLED)
			hdr->tmvp = vdec_hevc_u(inst, 1);
	}

	hdr->sao_luma = 0;
	hdr->sao_chroma = 0;
	if (sps->flags & V4L2_HEVC_SPS_FLAG_SAMPLE_ADAPTIVE_OFFSET) {
		hdr->sao_luma = vdec_hevc_u(inst, 1);
		hdr->sao_chroma = vdec_hevc_u(inst, 1);
	}

	hdr->num_ref_l0 = 0;
	hdr->num_ref_l1 = 0;
	hdr->mvd_l1_zero = 0;
	hdr->cabac_init = 0;
	hdr->col_from_l0 = 1;
	hdr->col_ref_idx = 0;
	hdr->max_merge = 0;
	if (hdr->slice_type != V4L2_HEVC_SLICE_TYPE_I) {
		hdr->num_ref_l0 = pps->num_ref_idx_l0_default_active_minus1;
		if (hdr->slice_type == V4L2_HEVC_SLICE_TYPE_B)
			hdr->num_ref_l1 = pps->num_ref_idx_l1_default_active_minus1;
		if (vdec_hevc_u(inst, 1)) {
			hdr->num_ref_l0 = vdec_hevc_ue(inst);
			if (hdr->slice_type == V4L2_HEVC_SLICE_TYPE_B)
				hdr->num_ref_l1 = vdec_hevc_ue(inst);
		}
		if (hdr->num_ref_l0 >= VDEC_HEVC_LIST_LEN ||
		    hdr->num_ref_l1 >= VDEC_HEVC_LIST_LEN)
			return -EINVAL;
		if (!inst->num_refs) {
			mtk_vdec_err(inst->ctx, "inter slice without reference pictures");
			return -EINVAL;
		}
		hevc_write(inst, HEVC_SHDR2, vdec_hevc_shdr2_refs(inst));

		if ((pps->flags & V4L2_HEVC_PPS_FLAG_LISTS_MODIFICATION_PRESENT) &&
		    inst->num_refs > 1) {
			hevc_write(inst, HEVC_REF_CFG, vdec_hevc_ref_cfg(inst));
			ret = vdec_hevc_wait_parse(inst, HEVC_PARSE_LIST_MOD,
						   "list modification");
			if (ret)
				return ret;
		}
		if (hdr->slice_type == V4L2_HEVC_SLICE_TYPE_B)
			hdr->mvd_l1_zero = vdec_hevc_u(inst, 1);
		if (pps->flags & V4L2_HEVC_PPS_FLAG_CABAC_INIT_PRESENT)
			hdr->cabac_init = vdec_hevc_u(inst, 1);
		if (hdr->tmvp) {
			if (hdr->slice_type == V4L2_HEVC_SLICE_TYPE_B)
				hdr->col_from_l0 = vdec_hevc_u(inst, 1);
			if ((hdr->col_from_l0 && hdr->num_ref_l0) ||
			    (!hdr->col_from_l0 && hdr->num_ref_l1))
				hdr->col_ref_idx = vdec_hevc_ue(inst);
			if (hdr->col_ref_idx >= VDEC_HEVC_LIST_LEN)
				return -EINVAL;
		}
		weighted = hdr->slice_type == V4L2_HEVC_SLICE_TYPE_P ?
			   pps->flags & V4L2_HEVC_PPS_FLAG_WEIGHTED_PRED :
			   pps->flags & V4L2_HEVC_PPS_FLAG_WEIGHTED_BIPRED;
		if (weighted) {
			ret = vdec_hevc_wait_parse(inst, HEVC_PARSE_WEIGHTS,
						   "weight table");
			if (ret)
				return ret;
		}
		val = vdec_hevc_ue(inst);
		if (val > 4)
			return -EINVAL;
		hdr->max_merge = 5 - val;
	}

	hdr->qp_delta = vdec_hevc_se(inst);
	hdr->cb_qp_offset = 0;
	hdr->cr_qp_offset = 0;
	if (pps->flags & V4L2_HEVC_PPS_FLAG_PPS_SLICE_CHROMA_QP_OFFSETS_PRESENT) {
		hdr->cb_qp_offset = vdec_hevc_se(inst);
		hdr->cr_qp_offset = vdec_hevc_se(inst);
	}

	hdr->deblock_disabled = !!(pps->flags & V4L2_HEVC_PPS_FLAG_PPS_DISABLE_DEBLOCKING_FILTER);
	hdr->beta_offset = pps->pps_beta_offset_div2;
	hdr->tc_offset = pps->pps_tc_offset_div2;
	if ((pps->flags & V4L2_HEVC_PPS_FLAG_DEBLOCKING_FILTER_OVERRIDE_ENABLED) &&
	    vdec_hevc_u(inst, 1)) {
		hdr->deblock_disabled = vdec_hevc_u(inst, 1);
		if (!hdr->deblock_disabled) {
			hdr->beta_offset = vdec_hevc_se(inst);
			hdr->tc_offset = vdec_hevc_se(inst);
		}
	}
	hdr->lf_across_slices =
		!!(pps->flags & V4L2_HEVC_PPS_FLAG_PPS_LOOP_FILTER_ACROSS_SLICES_ENABLED);
	if (hdr->lf_across_slices &&
	    (hdr->sao_luma || hdr->sao_chroma || !hdr->deblock_disabled))
		hdr->lf_across_slices = vdec_hevc_u(inst, 1);

	if (pps->flags & (V4L2_HEVC_PPS_FLAG_TILES_ENABLED |
			  V4L2_HEVC_PPS_FLAG_ENTROPY_CODING_SYNC_ENABLED)) {
		n = vdec_hevc_ue(inst);			/* num_entry_point_offsets */
		if (n > inst->ctb_w * inst->ctb_h)
			return -EINVAL;
		if (n) {
			val = vdec_hevc_ue(inst) + 1;	/* offset_len_minus1 + 1 */
			if (val > 32)
				return -EINVAL;
			while (n--)
				vdec_hevc_skip(inst, val);
		}
	}
	if (pps->flags & V4L2_HEVC_PPS_FLAG_SLICE_SEGMENT_HEADER_EXTENSION_PRESENT)
		vdec_hevc_skip(inst, 8 * vdec_hevc_ue(inst));

	/* byte_alignment(): one bit 1, then zeros up to the byte boundary */
	val = FIELD_GET(HEVC_READER_BIT_POS, hevc_read(inst, HEVC_READER));
	vdec_hevc_skip(inst, 8 - val);

	return 0;
}

/* ---------------- parameters, buffers and reference lists ---------------- */

static const void *vdec_hevc_get_ctrl_ptr(struct mtk_vcodec_dec_ctx *ctx, u32 id)
{
	struct v4l2_ctrl *ctrl = v4l2_ctrl_find(&ctx->ctrl_hdl, id);

	if (!ctrl)
		return ERR_PTR(-EINVAL);

	return ctrl->p_cur.p;
}

static int vdec_hevc_get_params(struct vdec_hevc_direct_inst *inst)
{
	struct mtk_vcodec_dec_ctx *ctx = inst->ctx;
	const struct mtk_vdec_direct_data *data = inst->dev->vdec_pdata->direct;
	const struct v4l2_ctrl_hevc_sps *sps;
	const struct v4l2_ctrl_hevc_pps *pps;
	unsigned int ctb;

	inst->dec = vdec_hevc_get_ctrl_ptr(ctx, V4L2_CID_STATELESS_HEVC_DECODE_PARAMS);
	inst->sps = vdec_hevc_get_ctrl_ptr(ctx, V4L2_CID_STATELESS_HEVC_SPS);
	inst->pps = vdec_hevc_get_ctrl_ptr(ctx, V4L2_CID_STATELESS_HEVC_PPS);
	inst->scaling = vdec_hevc_get_ctrl_ptr(ctx, V4L2_CID_STATELESS_HEVC_SCALING_MATRIX);
	if (IS_ERR(inst->dec) || IS_ERR(inst->sps) || IS_ERR(inst->pps) ||
	    IS_ERR(inst->scaling))
		return -EINVAL;
	sps = inst->sps;
	pps = inst->pps;

	if (sps->chroma_format_idc != 1 ||
	    sps->bit_depth_luma_minus8 != sps->bit_depth_chroma_minus8 ||
	    (sps->bit_depth_luma_minus8 != 0 && sps->bit_depth_luma_minus8 != 2)) {
		mtk_vdec_err(ctx, "only 4:2:0 with 8 or 10 bits supported");
		return -EOPNOTSUPP;
	}
	inst->ten_bit = sps->bit_depth_luma_minus8 == 2;
	if (inst->ten_bit != ctx->is_10bit_bitstream) {
		mtk_vdec_err(ctx, "bit depth differs from the capture format");
		return -EINVAL;
	}

	inst->log2_ctb = sps->log2_min_luma_coding_block_size_minus3 + 3 +
			 sps->log2_diff_max_min_luma_coding_block_size;
	if (inst->log2_ctb < 4 || inst->log2_ctb > 6)
		return -EINVAL;
	ctb = 1 << inst->log2_ctb;
	inst->ctb_w = DIV_ROUND_UP(sps->pic_width_in_luma_samples, ctb);
	inst->ctb_h = DIV_ROUND_UP(sps->pic_height_in_luma_samples, ctb);
	if (!sps->pic_width_in_luma_samples || !sps->pic_height_in_luma_samples ||
	    sps->pic_width_in_luma_samples > data->max_width ||
	    sps->pic_height_in_luma_samples > data->max_height) {
		mtk_vdec_err(ctx, "picture %ux%u not supported",
			     sps->pic_width_in_luma_samples,
			     sps->pic_height_in_luma_samples);
		return -EINVAL;
	}
	if ((pps->flags & V4L2_HEVC_PPS_FLAG_TILES_ENABLED) &&
	    (pps->num_tile_columns_minus1 >= inst->ctb_w ||
	     pps->num_tile_rows_minus1 >= inst->ctb_h ||
	     pps->num_tile_columns_minus1 >= ARRAY_SIZE(pps->column_width_minus1) ||
	     pps->num_tile_rows_minus1 >= ARRAY_SIZE(pps->row_height_minus1)))
		return -EINVAL;

	return 0;
}

static unsigned int vdec_hevc_buf_w(struct vdec_hevc_direct_inst *inst)
{
	return ALIGN(inst->sps->pic_width_in_luma_samples, VDEC_HEVC_ALIGN);
}

static unsigned int vdec_hevc_buf_h(struct vdec_hevc_direct_inst *inst)
{
	return ALIGN(inst->sps->pic_height_in_luma_samples, VDEC_HEVC_ALIGN);
}

/* 64 bytes per 16x16 block */
static unsigned int vdec_hevc_mv_size(struct vdec_hevc_direct_inst *inst)
{
	return vdec_hevc_buf_w(inst) * vdec_hevc_buf_h(inst) / 4 + VDEC_HEVC_MV_EXTRA;
}

static int vdec_hevc_mv_buf(struct vdec_hevc_direct_inst *inst, unsigned int idx)
{
	struct mtk_vcodec_mem *mem = &inst->mv_buf[idx];
	unsigned int size = vdec_hevc_mv_size(inst);

	if (mem->va && mem->size == size)
		return 0;
	if (mem->va)
		mtk_vcodec_mem_free(inst->ctx, mem);
	mem->size = size;

	return mtk_vcodec_mem_alloc(inst->ctx, mem);
}

/*
 * The decoder tells the pictures apart by an identifier in the reference
 * words of the motion vector buffers, also in those of the co-located
 * picture, and finds the long-term ones by it. There are fewer identifiers
 * than capture buffers: a picture gets one that no picture of its DPB holds
 * and keeps it while it is referenced.
 */
static int vdec_hevc_pick_id(struct vdec_hevc_direct_inst *inst)
{
	struct vb2_queue *vq = v4l2_m2m_get_vq(inst->ctx->m2m_ctx,
					       V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);
	const struct v4l2_ctrl_hevc_decode_params *dec = inst->dec;
	DECLARE_BITMAP(used, VDEC_HEVC_IDS);
	unsigned int i, id;

	bitmap_zero(used, VDEC_HEVC_IDS);
	__set_bit(0, used);
	for (i = 0; i < dec->num_active_dpb_entries; i++) {
		struct vb2_buffer *vb = vb2_find_buffer(vq, dec->dpb[i].timestamp);

		if (vb && vb->index < VDEC_HEVC_MAX_FB && inst->mv_buf[vb->index].va)
			__set_bit(inst->id[vb->index], used);
	}

	id = find_first_zero_bit(used, VDEC_HEVC_IDS);
	if (id >= VDEC_HEVC_IDS)
		return -EINVAL;

	return id;
}

/*
 * Pictures of the DPB with their capture buffers. A missing reference is
 * replaced by the target itself.
 */
static void vdec_hevc_build_dpb(struct vdec_hevc_direct_inst *inst,
				const struct vdec_hevc_direct_ref *dst)
{
	struct vb2_queue *vq = v4l2_m2m_get_vq(inst->ctx->m2m_ctx,
					       V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);
	const struct v4l2_ctrl_hevc_decode_params *dec = inst->dec;
	unsigned int i;

	for (i = 0; i < V4L2_HEVC_DPB_ENTRIES_NUM_MAX; i++) {
		const struct v4l2_hevc_dpb_entry *e = &dec->dpb[i];
		struct vdec_hevc_direct_ref *ref = &inst->dpb[i];
		struct vb2_buffer *vb;

		*ref = *dst;
		ref->poc = e->pic_order_cnt_val;
		ref->lt = !!(e->flags & V4L2_HEVC_DPB_ENTRY_LONG_TERM_REFERENCE);
		if (i >= dec->num_active_dpb_entries)
			continue;
		vb = vb2_find_buffer(vq, e->timestamp);
		if (!vb || vb->index >= VDEC_HEVC_MAX_FB || !inst->mv_buf[vb->index].va) {
			mtk_vdec_debug(inst->ctx, "reference %llu not found", e->timestamp);
			continue;
		}
		ref->y = vb2_dma_contig_plane_dma_addr(vb, 0);
		ref->c = vb2_dma_contig_plane_dma_addr(vb, 1);
		ref->mv = inst->mv_buf[vb->index].dma_addr;
		ref->id = inst->id[vb->index];
	}
}

/*
 * Initial reference lists (8.3.4): RefPicListTemp0 is StCurrBefore,
 * StCurrAfter, LtCurr, RefPicListTemp1 is StCurrAfter, StCurrBefore,
 * LtCurr. The decoder repeats them with the period NumPicTotalCurr.
 */
static int vdec_hevc_build_lists(struct vdec_hevc_direct_inst *inst)
{
	const struct v4l2_ctrl_hevc_decode_params *dec = inst->dec;
	unsigned int nb = dec->num_poc_st_curr_before;
	unsigned int na = dec->num_poc_st_curr_after;
	unsigned int nl = dec->num_poc_lt_curr;
	unsigned int i, n = 0;
	u8 idx;

	inst->num_refs = nb + na + nl;
	if (inst->num_refs > V4L2_HEVC_DPB_ENTRIES_NUM_MAX)
		return -EINVAL;
	for (i = 0; i < nb; i++, n++) {
		idx = dec->poc_st_curr_before[i];
		if (idx >= V4L2_HEVC_DPB_ENTRIES_NUM_MAX)
			return -EINVAL;
		inst->l0[n] = &inst->dpb[idx];
		inst->l1[na + i] = &inst->dpb[idx];
	}
	for (i = 0; i < na; i++, n++) {
		idx = dec->poc_st_curr_after[i];
		if (idx >= V4L2_HEVC_DPB_ENTRIES_NUM_MAX)
			return -EINVAL;
		inst->l0[n] = &inst->dpb[idx];
		inst->l1[i] = &inst->dpb[idx];
	}
	for (i = 0; i < nl; i++, n++) {
		idx = dec->poc_lt_curr[i];
		if (idx >= V4L2_HEVC_DPB_ENTRIES_NUM_MAX)
			return -EINVAL;
		inst->l0[n] = &inst->dpb[idx];
		inst->l1[n] = &inst->dpb[idx];
	}

	return 0;
}

/*
 * Make room for the zeros the reader needs behind the bitstream and return
 * the end of the reader window, 64-byte aligned.
 */
static int vdec_hevc_bs_end(struct vdec_hevc_direct_inst *inst,
			    struct mtk_vcodec_mem *bs, dma_addr_t *end)
{
	struct mtk_video_dec_buf *src = container_of(bs, struct mtk_video_dec_buf,
						     bs_buffer);
	dma_addr_t limit = bs->dma_addr +
			   vb2_plane_size(&src->m2m_buf.vb.vb2_buf, 0);
	dma_addr_t data_end = bs->dma_addr + bs->size;

	*end = ALIGN(data_end + VDEC_HEVC_BS_PAD, 64);
	if (*end > limit)
		*end = ALIGN_DOWN(limit, 64);
	if (*end < data_end + VDEC_HEVC_BS_MIN_PAD) {
		mtk_vdec_err(inst->ctx, "no room behind the bitstream (%zu of %pad)",
			     bs->size, &limit);
		return -EINVAL;
	}
	memset(bs->va + bs->size, 0, *end - data_end);

	return 0;
}

/* ---------------- register sequence of one picture ---------------- */

static void vdec_hevc_reset(struct vdec_hevc_direct_inst *inst)
{
	vdec_read(inst, VDEC_LD, VLD_PROC_STA);
	vdec_write(inst, VDEC_LD, VDEC_VLD_SRST, VLD_SRST_READER | VDEC_VLD_SRST_RESET);
	vdec_write(inst, VDEC_MISC, MISC_SRST, 0);
	vdec_write(inst, VDEC_LD, VDEC_VLD_SRST, VLD_SRST_READER | VDEC_VLD_SRST_RESET);
	vdec_write(inst, VDEC_MISC, MISC_SPEC_CON, MISC_SPEC_CON_HEVC);
	vdec_write(inst, VDEC_MISC, MISC_FUNC_CON, MISC_FUNC_CON_HEVC);
	vdec_write(inst, VDEC_MISC, MISC_CFG_178, MISC_CFG_178_HEVC);
	vdec_write(inst, VDEC_MISC, MISC_SYS_SEL, MISC_SYS_SEL_VAL);
	vdec_write(inst, VDEC_LD, VDEC_VLD_SRST, 0);
	mtk_vcodec_write_vdecsys(inst->ctx, VDEC_SYS_DCM_CON, 1);
	vdec_write(inst, VDEC_MISC, MISC_DCM_CON, 1);
	vdec_write(inst, VDEC_MISC, MISC_CFG_004, MISC_CFG_004_HEVC);
	hevc_write(inst, HEVC_READER, HEVC_READER_INIT);
}

static int vdec_hevc_init_reader(struct vdec_hevc_direct_inst *inst,
				 dma_addr_t start, dma_addr_t end)
{
	int ret;

	writel(readl(inst->dev->reg_base[VDEC_LD] + VLD_VBAR) | VLD_VBAR_EN,
	       inst->dev->reg_base[VDEC_LD] + VLD_VBAR);
	vdec_write(inst, VDEC_LD, VLD_VSTART, start >> 6);
	vdec_write(inst, VDEC_LD, VLD_VEND, end >> 6);
	vdec_read(inst, VDEC_LD, VLD_PROC_STA);
	vdec_write(inst, VDEC_LD, VLD_RPTR, start);
	vdec_write(inst, VDEC_LD, VLD_WPTR, end);
	vdec_write(inst, VDEC_LD, VDEC_VLD_SRST, VLD_SRST_READER);
	vdec_write(inst, VDEC_LD, VDEC_VLD_SRST, 0);
	vdec_write(inst, VDEC_LD, VLD_BS_CTRL, VLD_BS_CTRL_FETCH);
	ret = vdec_hevc_poll(inst, inst->dev->reg_base[VDEC_LD] + VLD_FETCH_STA,
			     VLD_FETCH_STA_READY, true, "bitstream fetch");
	vdec_write(inst, VDEC_LD, VLD_BS_CTRL, VLD_BS_CTRL_RUN);

	return ret;
}

static u32 vdec_hevc_ref_word(const struct vdec_hevc_direct_ref *ref, s32 poc)
{
	return FIELD_PREP(MV_REF_ID, ref->id) |
	       FIELD_PREP(MV_REF_POC_DIFF, (u32)(poc - ref->poc) & MV_REF_POC_DIFF);
}

static void vdec_hevc_write_lists(struct vdec_hevc_direct_inst *inst,
				  const struct vdec_hevc_direct_ref *dst, u32 data_size)
{
	const struct v4l2_ctrl_hevc_decode_params *dec = inst->dec;
	s32 poc = dec->pic_order_cnt_val;
	const struct vdec_hevc_direct_ref *r;
	unsigned int i, n = inst->num_refs;
	u32 lt = 0, ids = 0;

	vdec_write(inst, VDEC_CM, MC_CFG_A60, 0);
	vdec_write(inst, VDEC_CM, MC_YC_SEP, 0);
	vdec_write(inst, VDEC_PP, PP_CFG_B80, 0);

	/* all entries point to a valid picture, unused ones to the target */
	for (i = 0; i < VDEC_HEVC_LIST_LEN; i++) {
		vdec_write(inst, VDEC_CM, MC_L1_Y(i), dst->y);
		vdec_write(inst, VDEC_CM, MC_L0_Y(i), dst->y);
		vdec_write(inst, VDEC_CM, MC_L0_C(i), dst->c);
		vdec_write(inst, VDEC_CM, MC_L1_C(i), dst->c);
		vdec_write(inst, VDEC_AV, MV_L0_ADDR(i), dst->mv >> 4);
		vdec_write(inst, VDEC_AV, MV_L1_ADDR(i), dst->mv >> 4);
	}

	hevc_write(inst, HEVC_ERR_MASK, HEVC_ERR_MASK_VAL);
	hevc_write(inst, HEVC_DATA_SIZE, data_size);
	hevc_write(inst, HEVC_CFG_2AC, HEVC_CFG_2AC_VAL);
	hevc_write(inst, HEVC_CFG_0D4, HEVC_CFG_0D4_VAL);
	vdec_write(inst, VDEC_PP, PP_CFG_040, PP_CFG_040_VAL);
	vdec_write(inst, VDEC_AV, MV_CFG_210, 0);
	vdec_write(inst, VDEC_AV, MV_NO_BACKWARD,
		   inst->hdr.slice_type != V4L2_HEVC_SLICE_TYPE_I &&
		   !dec->num_poc_st_curr_after);

	if (inst->hdr.slice_type != V4L2_HEVC_SLICE_TYPE_I) {
		for (i = 0; i < VDEC_HEVC_LIST_LEN; i++) {
			if (inst->l0[i % n]->lt)
				lt |= BIT(i);
			if (inst->l1[i % n]->lt)
				lt |= BIT(16 + i);
		}
		vdec_write(inst, VDEC_AV, MV_LONG_TERM, lt);

		/* identifiers of the long-term pictures, four per word */
		n = 0;
		for (i = 0; i < dec->num_active_dpb_entries; i++) {
			if (!inst->dpb[i].lt)
				continue;
			ids |= inst->dpb[i].id << (8 * (n % 4));
			if (++n % 4 == 0) {
				vdec_write(inst, VDEC_AV, MV_LT_IDS + n - 4, ids);
				ids = 0;
			}
		}
		if (n % 4)
			vdec_write(inst, VDEC_AV, MV_LT_IDS + (n & ~3), ids);

		n = inst->num_refs;
		for (i = 0; i < VDEC_HEVC_LIST_LEN; i++)
			vdec_write(inst, VDEC_AV, MV_L0_REF(i),
				   vdec_hevc_ref_word(inst->l0[i % n], poc));
		for (i = 0; i < VDEC_HEVC_LIST_LEN; i++)
			vdec_write(inst, VDEC_AV, MV_L1_REF(i),
				   vdec_hevc_ref_word(inst->l1[i % n], poc));
	}

	vdec_write(inst, VDEC_AV, MV_DST_ADDR, dst->mv >> 4);
	/* chroma of each reference from its own address in the lists */
	vdec_write(inst, VDEC_CM, MC_YC_SEP, MC_YC_SEP_EN);

	if (inst->hdr.slice_type == V4L2_HEVC_SLICE_TYPE_I)
		return;
	for (i = 0; i < VDEC_HEVC_LIST_LEN; i++) {
		r = inst->l0[i % n];
		vdec_write(inst, VDEC_AV, MV_L0_ADDR(i), r->mv >> 4);
		vdec_write(inst, VDEC_CM, MC_L0_Y(i), r->y);
		vdec_write(inst, VDEC_CM, MC_L0_C(i), r->c);
	}
	for (i = 0; i < VDEC_HEVC_LIST_LEN; i++) {
		r = inst->l1[i % n];
		vdec_write(inst, VDEC_AV, MV_L1_ADDR(i), r->mv >> 4);
		vdec_write(inst, VDEC_CM, MC_L1_Y(i), r->y);
		vdec_write(inst, VDEC_CM, MC_L1_C(i), r->c);
	}
}

/*
 * The decoder takes the lists column by column (index x * size + y) with
 * four coefficients per word, the first in the lowest byte: 4x4 (six
 * lists), 8x8 (six), 16x16 (six) and 32x32 (two). V4L2 gives them in
 * raster order (index y * size + x).
 */
static void vdec_hevc_write_scl_lists(struct vdec_hevc_direct_inst *inst,
				      const u8 *lists, unsigned int num,
				      unsigned int size, unsigned int *addr)
{
	unsigned int l, i, x, y;
	u32 word = 0;

	for (l = 0; l < num; l++) {
		const u8 *m = lists + l * size * size;

		for (i = 0; i < size * size; i++) {
			x = i / size;
			y = i % size;
			word |= m[y * size + x] << (8 * (i % 4));
			if (i % 4 == 3) {
				vdec_write(inst, VDEC_LD, VLD_SCL_ADDR, (*addr)++);
				vdec_write(inst, VDEC_LD, VLD_SCL_DATA, word);
				word = 0;
			}
		}
	}
}

static void vdec_hevc_write_scaling(struct vdec_hevc_direct_inst *inst)
{
	const struct v4l2_ctrl_hevc_scaling_matrix *m = inst->scaling;
	unsigned int addr = 0;

	if (!(inst->sps->flags & V4L2_HEVC_SPS_FLAG_SCALING_LIST_ENABLED)) {
		vdec_write(inst, VDEC_PP, PP_SCL_EN, 0);
		return;
	}

	vdec_write(inst, VDEC_PP, PP_SCL_EN, 1);
	vdec_write(inst, VDEC_PP, PP_SCL_DC_0,
		   m->scaling_list_dc_coef_16x16[0] | m->scaling_list_dc_coef_16x16[1] << 8 |
		   m->scaling_list_dc_coef_16x16[2] << 16 |
		   m->scaling_list_dc_coef_16x16[3] << 24);
	vdec_write(inst, VDEC_PP, PP_SCL_DC_1,
		   m->scaling_list_dc_coef_16x16[4] | m->scaling_list_dc_coef_16x16[5] << 8 |
		   m->scaling_list_dc_coef_32x32[0] << 16 |
		   m->scaling_list_dc_coef_32x32[1] << 24);
	vdec_write(inst, VDEC_PP, PP_SCL_TABLE, 1);
	vdec_hevc_write_scl_lists(inst, &m->scaling_list_4x4[0][0], 6, 4, &addr);
	vdec_hevc_write_scl_lists(inst, &m->scaling_list_8x8[0][0], 6, 8, &addr);
	vdec_hevc_write_scl_lists(inst, &m->scaling_list_16x16[0][0], 6, 8, &addr);
	vdec_hevc_write_scl_lists(inst, &m->scaling_list_32x32[0][0], 2, 8, &addr);
	vdec_write(inst, VDEC_PP, PP_SCL_TABLE, 0);
	WARN_ON_ONCE(addr != VDEC_HEVC_SCL_WORDS);
}

static void vdec_hevc_write_shdr(struct vdec_hevc_direct_inst *inst)
{
	const struct v4l2_ctrl_hevc_pps *pps = inst->pps;
	struct vdec_hevc_direct_hdr *hdr = &inst->hdr;
	s32 qp = 26 + pps->init_qp_minus26 + hdr->qp_delta;
	u32 val;

	vdec_hevc_write_shdr0(inst);

	val = FIELD_PREP(HEVC_SHDR1_QP, (u32)qp & 0x7f) |
	      FIELD_PREP(HEVC_SHDR1_CB_QP_OFFSET, s5(pps->pps_cb_qp_offset + hdr->cb_qp_offset)) |
	      FIELD_PREP(HEVC_SHDR1_CR_QP_OFFSET, s5(pps->pps_cr_qp_offset + hdr->cr_qp_offset));
	if (hdr->sao_luma)
		val |= HEVC_SHDR1_SAO_LUMA;
	if (hdr->sao_chroma)
		val |= HEVC_SHDR1_SAO_CHROMA;
	if (hdr->cabac_init)
		val |= HEVC_SHDR1_CABAC_INIT;
	hevc_write(inst, HEVC_SHDR1, val);

	val = vdec_hevc_shdr2_refs(inst) |
	      FIELD_PREP(HEVC_SHDR2_COL_REF_IDX, hdr->col_ref_idx) |
	      FIELD_PREP(HEVC_SHDR2_MAX_MERGE, hdr->max_merge) |
	      FIELD_PREP(HEVC_SHDR2_BETA_OFFSET, s5(hdr->beta_offset)) |
	      FIELD_PREP(HEVC_SHDR2_TC_OFFSET, s5(hdr->tc_offset));
	if (hdr->tmvp)
		val |= HEVC_SHDR2_TMVP;
	if (hdr->col_from_l0)
		val |= HEVC_SHDR2_COL_FROM_L0;
	if (hdr->mvd_l1_zero)
		val |= HEVC_SHDR2_MVD_L1_ZERO;
	if (hdr->lf_across_slices)
		val |= HEVC_SHDR2_LF_ACROSS_SLICES;
	if (hdr->deblock_disabled)
		val |= HEVC_SHDR2_DEBLOCK_DISABLED;
	hevc_write(inst, HEVC_SHDR2, val);

	hevc_write(inst, HEVC_SHDR3,
		   FIELD_PREP(HEVC_SHDR3_REF_RPS_DELTAS,
			      inst->dec->short_term_ref_pic_set_size ?
			      inst->dec->num_delta_pocs_of_ref_rps_idx : 0) |
		   FIELD_PREP(HEVC_SHDR3_RPS_DELTAS, hdr->rps_deltas) |
		   FIELD_PREP(HEVC_SHDR3_NUM_LT, hdr->num_lt) |
		   FIELD_PREP(HEVC_SHDR3_NUM_LT_SPS, hdr->num_lt_sps));
	hevc_write(inst, HEVC_REF_CFG, vdec_hevc_ref_cfg(inst));
	hevc_write(inst, HEVC_PIC_CFG,
		   HEVC_PIC_CFG_VAL | FIELD_PREP(HEVC_PIC_CFG_NAL_TYPE, hdr->nal_type));
}

static void vdec_hevc_write_sps(struct vdec_hevc_direct_inst *inst)
{
	const struct v4l2_ctrl_hevc_sps *sps = inst->sps;
	unsigned int log2_min_cb = sps->log2_min_luma_coding_block_size_minus3 + 3;
	unsigned int log2_min_tb = sps->log2_min_luma_transform_block_size_minus2 + 2;
	unsigned int log2_min_pcm = 3, log2_max_pcm = 5;
	u32 val;

	val = FIELD_PREP(HEVC_SPS0_CHROMA_FORMAT, sps->chroma_format_idc) |
	      FIELD_PREP(HEVC_SPS0_NUM_LT_SPS, sps->num_long_term_ref_pics_sps) |
	      FIELD_PREP(HEVC_SPS0_NUM_ST_RPS, sps->num_short_term_ref_pic_sets);
	if (sps->flags & V4L2_HEVC_SPS_FLAG_PCM_ENABLED) {
		val |= HEVC_SPS0_PCM |
		       FIELD_PREP(HEVC_SPS0_PCM_DEPTH_Y, sps->pcm_sample_bit_depth_luma_minus1) |
		       FIELD_PREP(HEVC_SPS0_PCM_DEPTH_C, sps->pcm_sample_bit_depth_chroma_minus1);
		log2_min_pcm = sps->log2_min_pcm_luma_coding_block_size_minus3 + 3;
		log2_max_pcm = log2_min_pcm + sps->log2_diff_max_min_pcm_luma_coding_block_size;
		if (sps->flags & V4L2_HEVC_SPS_FLAG_PCM_LOOP_FILTER_DISABLED)
			val |= HEVC_SPS0_PCM_NO_FILTER;
	} else {
		val |= FIELD_PREP(HEVC_SPS0_PCM_DEPTH_Y, 7) |
		       FIELD_PREP(HEVC_SPS0_PCM_DEPTH_C, 7);
		log2_min_pcm = 7;
	}
	/* the decoder needs it for the headers of the following slice segments */
	if (inst->pps->flags & V4L2_HEVC_PPS_FLAG_LISTS_MODIFICATION_PRESENT)
		val |= HEVC_SPS0_LISTS_MOD;
	if (sps->flags & V4L2_HEVC_SPS_FLAG_AMP_ENABLED)
		val |= HEVC_SPS0_AMP;
	if (sps->flags & V4L2_HEVC_SPS_FLAG_SAMPLE_ADAPTIVE_OFFSET)
		val |= HEVC_SPS0_SAO;
	if (sps->flags & V4L2_HEVC_SPS_FLAG_SPS_TEMPORAL_MVP_ENABLED)
		val |= HEVC_SPS0_TMVP;
	if (sps->flags & V4L2_HEVC_SPS_FLAG_LONG_TERM_REF_PICS_PRESENT)
		val |= HEVC_SPS0_LT_PRESENT;
	hevc_write(inst, HEVC_SPS0, val);

	val = FIELD_PREP(HEVC_SPS1_LOG2_MAX_POC_LSB, sps->log2_max_pic_order_cnt_lsb_minus4 + 4) |
	      FIELD_PREP(HEVC_SPS1_ST_RPS_IDX_BITS,
			 max(1U, ceil_log2(sps->num_short_term_ref_pic_sets))) |
	      FIELD_PREP(HEVC_SPS1_LT_IDX_BITS,
			 max(1U, ceil_log2(sps->num_long_term_ref_pics_sps)));
	if (sps->flags & V4L2_HEVC_SPS_FLAG_STRONG_INTRA_SMOOTHING_ENABLED)
		val |= HEVC_SPS1_STRONG_INTRA;
	hevc_write(inst, HEVC_SPS1, val);

	hevc_write(inst, HEVC_SPS2,
		   FIELD_PREP(HEVC_SPS2_LOG2_MIN_CB, log2_min_cb) |
		   FIELD_PREP(HEVC_SPS2_LOG2_CTB, inst->log2_ctb) |
		   FIELD_PREP(HEVC_SPS2_LOG2_MIN_TB, log2_min_tb) |
		   FIELD_PREP(HEVC_SPS2_LOG2_MAX_TB,
			      log2_min_tb + sps->log2_diff_max_min_luma_transform_block_size) |
		   FIELD_PREP(HEVC_SPS2_LOG2_MIN_PCM, log2_min_pcm) |
		   FIELD_PREP(HEVC_SPS2_LOG2_MAX_PCM, log2_max_pcm) |
		   FIELD_PREP(HEVC_SPS2_TH_DEPTH_INTER, sps->max_transform_hierarchy_depth_inter) |
		   FIELD_PREP(HEVC_SPS2_TH_DEPTH_INTRA, sps->max_transform_hierarchy_depth_intra));
}

static void vdec_hevc_write_pps(struct vdec_hevc_direct_inst *inst)
{
	const struct v4l2_ctrl_hevc_pps *pps = inst->pps;
	static const struct {
		u64 flag;
		u32 bit;
	} pps0[] = {
		{ V4L2_HEVC_PPS_FLAG_SIGN_DATA_HIDING_ENABLED, HEVC_PPS0_SIGN_HIDING },
		{ V4L2_HEVC_PPS_FLAG_CABAC_INIT_PRESENT, HEVC_PPS0_CABAC_INIT },
		{ V4L2_HEVC_PPS_FLAG_CONSTRAINED_INTRA_PRED, HEVC_PPS0_CONSTR_INTRA },
		{ V4L2_HEVC_PPS_FLAG_TRANSFORM_SKIP_ENABLED, HEVC_PPS0_TRANSFORM_SKIP },
		{ V4L2_HEVC_PPS_FLAG_WEIGHTED_PRED, HEVC_PPS0_WEIGHTED_PRED },
		{ V4L2_HEVC_PPS_FLAG_WEIGHTED_BIPRED, HEVC_PPS0_WEIGHTED_BIPRED },
		{ V4L2_HEVC_PPS_FLAG_OUTPUT_FLAG_PRESENT, HEVC_PPS0_OUTPUT_FLAG },
		{ V4L2_HEVC_PPS_FLAG_TRANSQUANT_BYPASS_ENABLED, HEVC_PPS0_TRANSQUANT_BYPASS },
		{ V4L2_HEVC_PPS_FLAG_DEPENDENT_SLICE_SEGMENT_ENABLED, HEVC_PPS0_DEPENDENT_SLICES },
		{ V4L2_HEVC_PPS_FLAG_ENTROPY_CODING_SYNC_ENABLED, HEVC_PPS0_ENTROPY_SYNC },
		{ V4L2_HEVC_PPS_FLAG_SLICE_SEGMENT_HEADER_EXTENSION_PRESENT,
		  HEVC_PPS0_SHDR_EXTENSION },
	};
	unsigned int i;
	u32 val;

	val = FIELD_PREP(HEVC_PPS0_NUM_REF_L0, pps->num_ref_idx_l0_default_active_minus1) |
	      FIELD_PREP(HEVC_PPS0_NUM_REF_L1, pps->num_ref_idx_l1_default_active_minus1) |
	      FIELD_PREP(HEVC_PPS0_LOG2_MERGE_LEVEL, pps->log2_parallel_merge_level_minus2 + 2) |
	      FIELD_PREP(HEVC_PPS0_EXTRA_SHDR_BITS, pps->num_extra_slice_header_bits);
	for (i = 0; i < ARRAY_SIZE(pps0); i++)
		if (pps->flags & pps0[i].flag)
			val |= pps0[i].bit;
	hevc_write(inst, HEVC_PPS0, val);

	val = FIELD_PREP(HEVC_PPS1_LOG2_MIN_QP_CU, inst->log2_ctb - pps->diff_cu_qp_delta_depth) |
	      FIELD_PREP(HEVC_PPS1_INIT_QP, (u32)(26 + pps->init_qp_minus26) & 0xff) |
	      FIELD_PREP(HEVC_PPS1_CB_QP_OFFSET, s5(pps->pps_cb_qp_offset)) |
	      FIELD_PREP(HEVC_PPS1_CR_QP_OFFSET, s5(pps->pps_cr_qp_offset));
	if (pps->flags & V4L2_HEVC_PPS_FLAG_CU_QP_DELTA_ENABLED)
		val |= HEVC_PPS1_CU_QP_DELTA;
	if (pps->flags & V4L2_HEVC_PPS_FLAG_PPS_SLICE_CHROMA_QP_OFFSETS_PRESENT)
		val |= HEVC_PPS1_CHROMA_QP_OFFSETS;
	hevc_write(inst, HEVC_PPS1, val);

	val = 0;
	if (pps->flags & V4L2_HEVC_PPS_FLAG_TILES_ENABLED)
		val |= HEVC_PPS2_TILES |
		       FIELD_PREP(HEVC_PPS2_TILE_COLS_M1, pps->num_tile_columns_minus1) |
		       FIELD_PREP(HEVC_PPS2_TILE_ROWS_M1, pps->num_tile_rows_minus1);
	if (!(pps->flags & V4L2_HEVC_PPS_FLAG_TILES_ENABLED) ||
	    (pps->flags & V4L2_HEVC_PPS_FLAG_LOOP_FILTER_ACROSS_TILES_ENABLED))
		val |= HEVC_PPS2_LF_ACROSS_TILES;
	hevc_write(inst, HEVC_PPS2, val);

	val = FIELD_PREP(HEVC_PPS3_BETA_OFFSET, s5(pps->pps_beta_offset_div2)) |
	      FIELD_PREP(HEVC_PPS3_TC_OFFSET, s5(pps->pps_tc_offset_div2));
	if (pps->flags & V4L2_HEVC_PPS_FLAG_PPS_LOOP_FILTER_ACROSS_SLICES_ENABLED)
		val |= HEVC_PPS3_LF_ACROSS_SLICES;
	if (pps->flags & V4L2_HEVC_PPS_FLAG_DEBLOCKING_FILTER_CONTROL_PRESENT)
		val |= HEVC_PPS3_DEBLOCK_CTRL;
	if (pps->flags & V4L2_HEVC_PPS_FLAG_DEBLOCKING_FILTER_OVERRIDE_ENABLED)
		val |= HEVC_PPS3_DEBLOCK_OVERRIDE;
	if (pps->flags & V4L2_HEVC_PPS_FLAG_PPS_DISABLE_DEBLOCKING_FILTER)
		val |= HEVC_PPS3_DEBLOCK_DISABLED;
	hevc_write(inst, HEVC_PPS3, val);
}

/* tile boundaries: one bit for the last column and row of each tile */
static void vdec_hevc_write_tiles(struct vdec_hevc_direct_inst *inst)
{
	const struct v4l2_ctrl_hevc_pps *pps = inst->pps;
	u32 cols[HEVC_TILE_WORDS] = {}, rows[HEVC_TILE_WORDS] = {};
	unsigned int nc = 1, nr = 1, i, pos;

	if (pps->flags & V4L2_HEVC_PPS_FLAG_TILES_ENABLED) {
		nc = pps->num_tile_columns_minus1 + 1;
		nr = pps->num_tile_rows_minus1 + 1;
	}

	pos = 0;
	for (i = 0; i < nc; i++) {
		if (i == nc - 1)
			pos = inst->ctb_w;
		else if (pps->flags & V4L2_HEVC_PPS_FLAG_UNIFORM_SPACING)
			pos = (i + 1) * inst->ctb_w / nc;
		else
			pos += pps->column_width_minus1[i] + 1;
		if (pos > inst->ctb_w || pos > HEVC_TILE_WORDS * 32)
			break;
		cols[(pos - 1) / 32] |= BIT((pos - 1) % 32);
	}
	pos = 0;
	for (i = 0; i < nr; i++) {
		if (i == nr - 1)
			pos = inst->ctb_h;
		else if (pps->flags & V4L2_HEVC_PPS_FLAG_UNIFORM_SPACING)
			pos = (i + 1) * inst->ctb_h / nr;
		else
			pos += pps->row_height_minus1[i] + 1;
		if (pos > inst->ctb_h || pos > HEVC_TILE_WORDS * 32)
			break;
		rows[(pos - 1) / 32] |= BIT((pos - 1) % 32);
	}

	for (i = 0; i < HEVC_TILE_WORDS; i++)
		hevc_write(inst, HEVC_TILE_COLS(i), cols[i]);
	hevc_write(inst, HEVC_TILE_CFG_1C0, 0);
	for (i = 0; i < HEVC_TILE_WORDS; i++)
		hevc_write(inst, HEVC_TILE_ROWS(i), rows[i]);
	hevc_write(inst, HEVC_TILE_CFG_1D0, 0);
}

static void vdec_hevc_start(struct vdec_hevc_direct_inst *inst,
			    const struct vdec_hevc_direct_ref *dst)
{
	const struct v4l2_ctrl_hevc_sps *sps = inst->sps;
	unsigned int width = sps->pic_width_in_luma_samples;
	unsigned int height = sps->pic_height_in_luma_samples;
	unsigned int ctb_w16 = (inst->ctb_w << inst->log2_ctb) / 16;
	unsigned int ctb_h16 = (inst->ctb_h << inst->log2_ctb) / 16;
	bool large = (inst->ctb_w << inst->log2_ctb) * (inst->ctb_h << inst->log2_ctb) >
		     VDEC_HEVC_LARGE_PIC;

	vdec_write(inst, VDEC_CM, MC_CFG_AF8, MC_CFG_AF8_VAL);
	vdec_write(inst, VDEC_CM, MC_PIC_W, width);
	vdec_write(inst, VDEC_CM, MC_PIC_H, height);
	vdec_write(inst, VDEC_CM, MC_PITCH_16, vdec_hevc_buf_w(inst) / 16);
	vdec_write(inst, VDEC_CM, MC_CFG_220, 1);
	vdec_write(inst, VDEC_CM, MC_Y_ADDR_512, dst->y >> 9);
	vdec_write(inst, VDEC_CM, MC_C_ADDR_256, dst->c >> 8);
	vdec_write(inst, VDEC_CM, MC_PIC_W_16, ctb_w16);
	vdec_write(inst, VDEC_CM, MC_CFG_238, 3);
	vdec_write(inst, VDEC_CM, MC_CFG_250, 1);
	vdec_write(inst, VDEC_CM, MC_PIC_W_16_M1, ctb_w16 - 1);
	vdec_write(inst, VDEC_CM, MC_PIC_H_16_M1, ctb_h16 - 1);
	vdec_write(inst, VDEC_CM, MC_10BIT, inst->ten_bit);
	vdec_write(inst, VDEC_PP, PP_10BIT_0, inst->ten_bit);
	vdec_write(inst, VDEC_PP, PP_10BIT_1, inst->ten_bit);
	vdec_hevc_write_scaling(inst);
	vdec_hevc_write_shdr(inst);
	vdec_write(inst, VDEC_TOP, TOP_PIC_CTB_SIZE_M1,
		   (inst->ctb_h - 1) << 16 | (inst->ctb_w - 1));
	vdec_write(inst, VDEC_TOP, TOP_PIC_PIX_SIZE, height << 16 | width);
	vdec_write(inst, VDEC_TOP, TOP_PIC_CTB_SIZE, inst->ctb_h << 16 | inst->ctb_w);
	vdec_write(inst, VDEC_TOP, TOP_BIT_DEPTH,
		   sps->bit_depth_luma_minus8 | sps->bit_depth_chroma_minus8 << 4);
	vdec_hevc_write_sps(inst);
	vdec_hevc_write_pps(inst);
	vdec_hevc_write_tiles(inst);
	/*
	 * The decoder keeps these settings while it stays powered, also for the
	 * next picture of any codec; small pictures use the reset values.
	 */
	if (large) {
		vdec_write(inst, VDEC_PP, PP_CFG_B80, PP_CFG_B80_LARGE);
		vdec_write(inst, VDEC_MISC, MISC_SIZE_CFG, MISC_SIZE_CFG_HEVC_LARGE);
		vdec_write(inst, VDEC_MISC, MISC_CFG_1F0, MISC_CFG_1F0_HEVC_LARGE);
	} else {
		vdec_write(inst, VDEC_MISC, MISC_SIZE_CFG, MISC_SIZE_CFG_RESET);
		vdec_write(inst, VDEC_MISC, MISC_CFG_1F0, MISC_CFG_1F0_RESET);
	}
	hevc_read(inst, HEVC_GETBITS);
	hevc_write(inst, HEVC_DEC_START, 1);
}

/*
 * Offset of the NAL unit header of the first slice segment in the buffer;
 * the decoder takes the length of the data from there.
 */
static int vdec_hevc_slice_offset(struct mtk_vcodec_mem *bs, size_t *off)
{
	const u8 *p = bs->va;
	size_t i;
	u8 type;

	for (i = 0; i + 3 < bs->size; i++) {
		if (p[i] || p[i + 1] || p[i + 2] != 1)
			continue;
		type = (p[i + 3] >> 1) & 0x3f;
		if (type <= NAL_RSV_VCL_N14 ||
		    (type >= NAL_BLA_W_LP && type <= NAL_RSV_IRAP_23)) {
			*off = i + 3;
			return 0;
		}
		i += 2;
	}

	return -EINVAL;
}

static int vdec_hevc_run(struct vdec_hevc_direct_inst *inst, struct mtk_vcodec_mem *bs,
			 const struct vdec_hevc_direct_ref *dst)
{
	dma_addr_t end;
	size_t off;
	int ret;

	ret = vdec_hevc_slice_offset(bs, &off);
	if (ret) {
		mtk_vdec_err(inst->ctx, "no slice in the bitstream buffer");
		return ret;
	}
	ret = vdec_hevc_bs_end(inst, bs, &end);
	if (ret)
		return ret;

	vdec_hevc_reset(inst);
	ret = vdec_hevc_init_reader(inst, bs->dma_addr, end);
	if (ret)
		return ret;
	ret = vdec_hevc_find_slice(inst);
	if (ret)
		return ret;

	ret = vdec_hevc_read_hdr(inst);
	if (ret)
		return ret;

	vdec_hevc_write_lists(inst, dst, bs->size - off + VDEC_HEVC_DATA_EXTRA);
	vdec_hevc_start(inst, dst);

	return 0;
}

/* ---------------- codec interface ---------------- */

static int vdec_hevc_direct_decode(void *h_vdec, struct mtk_vcodec_mem *bs,
				   struct vdec_fb *unused, bool *res_chg)
{
	struct vdec_hevc_direct_inst *inst = h_vdec;
	struct mtk_vcodec_dec_ctx *ctx = inst->ctx;
	struct mtk_video_dec_buf *src_buf, *dst_buf;
	struct vdec_hevc_direct_ref dst;
	struct vdec_fb *fb;
	unsigned int idx, luma;
	u32 pos, err0, err1;
	int ret;

	*res_chg = false;
	/* nothing is held in the hardware between pictures */
	if (!bs)
		return 0;

	fb = inst->dev->vdec_pdata->get_cap_buffer(ctx);
	if (!fb)
		return -ENOMEM;
	src_buf = container_of(bs, struct mtk_video_dec_buf, bs_buffer);
	dst_buf = container_of(fb, struct mtk_video_dec_buf, frame_buffer);
	v4l2_m2m_buf_copy_metadata(&src_buf->m2m_buf.vb, &dst_buf->m2m_buf.vb);

	ret = vdec_hevc_get_params(inst);
	if (ret)
		return ret;

	luma = vdec_hevc_buf_w(inst) * vdec_hevc_buf_h(inst);
	if (inst->ten_bit)
		luma = luma * 5 / 4;
	if (vdec_hevc_buf_w(inst) != ctx->picinfo.buf_w ||
	    fb->base_y.size < luma || fb->base_c.size < luma / 2) {
		mtk_vdec_err(ctx, "picture %ux%u does not fit the %ux%u buffers",
			     inst->sps->pic_width_in_luma_samples,
			     inst->sps->pic_height_in_luma_samples,
			     ctx->picinfo.buf_w, ctx->picinfo.buf_h);
		return -EINVAL;
	}

	idx = dst_buf->m2m_buf.vb.vb2_buf.index;
	if (idx >= VDEC_HEVC_MAX_FB)
		return -EINVAL;
	ret = vdec_hevc_mv_buf(inst, idx);
	if (ret)
		return ret;

	dst.y = fb->base_y.dma_addr;
	dst.c = fb->base_c.dma_addr;
	dst.mv = inst->mv_buf[idx].dma_addr;
	ret = vdec_hevc_pick_id(inst);
	if (ret < 0)
		return ret;
	dst.id = ret;
	inst->id[idx] = dst.id;
	dst.poc = inst->dec->pic_order_cnt_val;
	dst.lt = false;
	vdec_hevc_build_dpb(inst, &dst);
	ret = vdec_hevc_build_lists(inst);
	if (ret)
		return ret;

	ret = vdec_hevc_run(inst, bs, &dst);
	if (ret)
		goto err_break;

	ret = mtk_vcodec_wait_for_done_ctx(ctx, MTK_INST_IRQ_RECEIVED,
					   VDEC_HEVC_TIMEOUT_MS, MTK_VDEC_CORE);
	if (ret) {
		ret = -ETIMEDOUT;
		goto err_break;
	}

	pos = hevc_read(inst, HEVC_CTB_POS);
	err0 = hevc_read(inst, HEVC_ERR_STA0);
	err1 = hevc_read(inst, HEVC_ERR_STA1);
	ctx->decoded_frame_cnt++;
	if (FIELD_GET(HEVC_CTB_POS_X, pos) != inst->ctb_w - 1 ||
	    FIELD_GET(HEVC_CTB_POS_Y, pos) != inst->ctb_h - 1) {
		mtk_vdec_err(ctx, "picture incomplete: last CTB %lu,%lu, error %#x %#x",
			     FIELD_GET(HEVC_CTB_POS_X, pos), FIELD_GET(HEVC_CTB_POS_Y, pos),
			     err0, err1);
		return -EBADMSG;
	}
	if ((err0 | err1) & HEVC_ERR_MASK_VAL) {
		mtk_vdec_err(ctx, "bitstream error %#x %#x in picture %d", err0, err1,
			     ctx->decoded_frame_cnt - 1);
		return -EBADMSG;
	}
	mtk_vdec_debug(ctx, "picture %d done", ctx->decoded_frame_cnt - 1);

	return 0;

err_break:
	mtk_vdec_direct_hw_break(inst->dev);
	return ret;
}

static void vdec_hevc_free(struct vdec_hevc_direct_inst *inst)
{
	unsigned int i;

	for (i = 0; i < VDEC_HEVC_MAX_FB; i++)
		if (inst->mv_buf[i].va)
			mtk_vcodec_mem_free(inst->ctx, &inst->mv_buf[i]);
}

static int vdec_hevc_direct_init(struct mtk_vcodec_dec_ctx *ctx)
{
	const struct mtk_vdec_direct_data *data = ctx->dev->vdec_pdata->direct;
	struct vdec_hevc_direct_inst *inst;

	if (!data->hevc_vld)
		return -EOPNOTSUPP;

	inst = kzalloc_obj(*inst);
	if (!inst)
		return -ENOMEM;
	inst->ctx = ctx;
	inst->dev = ctx->dev;
	inst->hevc = mtk_vdec_direct_block(ctx->dev, data->hevc_vld);

	ctx->drv_handle = inst;
	return 0;
}

static void vdec_hevc_direct_deinit(void *h_vdec)
{
	struct vdec_hevc_direct_inst *inst = h_vdec;

	vdec_hevc_free(inst);
	kfree(inst);
}

static int vdec_hevc_direct_get_param(void *h_vdec, enum vdec_get_param_type type,
				      void *out)
{
	struct vdec_hevc_direct_inst *inst = h_vdec;
	struct mtk_vcodec_dec_ctx *ctx = inst->ctx;
	struct vdec_pic_info *pic = &ctx->picinfo;
	struct v4l2_rect *crop;

	switch (type) {
	case GET_PARAM_PIC_INFO:
		/* coding tree blocks of up to 64x64, MM21 tiles */
		pic->buf_w = ALIGN(pic->pic_w, VDEC_HEVC_ALIGN);
		pic->buf_h = ALIGN(pic->pic_h, VDEC_HEVC_ALIGN);
		pic->fb_sz[0] = pic->buf_w * pic->buf_h;
		if (ctx->is_10bit_bitstream)
			pic->fb_sz[0] = pic->fb_sz[0] * 5 / 4;
		pic->fb_sz[1] = pic->fb_sz[0] / 2;
		*(struct vdec_pic_info *)out = *pic;
		break;
	case GET_PARAM_DPB_SIZE:
		*(unsigned int *)out = V4L2_HEVC_DPB_ENTRIES_NUM_MAX + 1;
		break;
	case GET_PARAM_CROP_INFO:
		crop = out;
		crop->left = 0;
		crop->top = 0;
		crop->width = pic->pic_w;
		crop->height = pic->pic_h;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

const struct vdec_common_if vdec_hevc_slice_direct_if = {
	.init		= vdec_hevc_direct_init,
	.decode		= vdec_hevc_direct_decode,
	.get_param	= vdec_hevc_direct_get_param,
	.deinit		= vdec_hevc_direct_deinit,
};
