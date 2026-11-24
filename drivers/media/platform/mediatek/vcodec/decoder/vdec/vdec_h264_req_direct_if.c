// SPDX-License-Identifier: GPL-2.0
/*
 * H.264 decoding on MediaTek decoders that the host programs through their
 * registers, without firmware (MT6785).
 *
 * One hardware run decodes a whole frame with all of its slices (frame
 * based, Annex B start codes). The host reads the header of the first slice
 * through the bitstream reader of the decoder up to the start of the slice
 * data and programs the picture from it and from the V4L2 controls. The
 * decoder reads the reference list modification and the prediction weight
 * table of that header itself, on request, and all headers of the
 * following slices.
 *
 * Each field of a field pair is a picture of its own: the decoder writes
 * it into every other line of the capture buffer that holds the pair.
 *
 * Output is MM21 (luma in 16x32 tiles, chroma in 16x16 tiles) for 8 bits
 * and MT2110R for 10 bits, one motion vector buffer per capture buffer.
 * For 4:0:0 (8 bits) the decoder writes the luma plane only; the host fills
 * the chroma plane with the mid value after each run.
 * Not supported: 4:2:2 and 4:4:4, 4:0:0 with 10 bits, bit depths other than
 * 8 and 10, slice groups, SP and SI slices, data partitioning.
 */

#include <linux/bitfield.h>
#include <linux/bitmap.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <media/v4l2-h264.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-dma-contig.h>

#include "../mtk_vcodec_dec.h"
#include "../../common/mtk_vcodec_intr.h"
#include "../../common/mtk_vcodec_util.h"
#include "../vdec_drv_base.h"
#include "../vdec_drv_if.h"
#include "vdec_h264_req_common.h"

/* bitstream reader (VDEC_LD) */
#define VLD_BS_CTRL			0x08c
#define VLD_BS_CTRL_FETCH		BIT(20)
#define VLD_BS_CTRL_RUN			BIT(23)
#define VLD_RPTR			0x0b0
#define VLD_VSTART			0x0b4
#define VLD_VEND			0x0b8
#define VLD_CFG_0C0			0x0c0
#define VLD_CFG_0C0_INIT		0x40000
#define VLD_CFG_0CC			0x0cc
#define VLD_FETCH_STA			0x0e8
#define VLD_FETCH_STA_READY		BIT(0)
#define VLD_VBAR			0x0ec
#define VLD_VBAR_EN			BIT(28)
#define VLD_PROC_STA			0x0f4
#define VLD_SRST_READER			BIT(8)
#define VLD_WPTR			0x110
#define VLD_SCL_ADDR			0x260
#define VLD_SCL_ADDR_WRITE		BIT(9)
#define VLD_SCL_ADDR_DONE		BIT(8)
#define VLD_SCL_DATA			0x264

/* scaling list memory: word address of the lists */
#define SCL_4X4_ADDR(list)		(0x10 * (list))
#define SCL_8X8_INTRA_Y_ADDR		0x60
#define SCL_8X8_INTER_Y_ADDR		0xa0

/* picture level of the reader (VDEC_TOP) */
#define TOP_PRED_ADDR			0x028
#define TOP_CFG_050			0x050
#define TOP_CFG_054			0x054
#define TOP_CFG_058			0x058
#define TOP_PIC_MB_SIZE_M1		0x068
#define TOP_PIC_MBS			0x06c
#define TOP_PIC_PIX_SIZE		0x070
#define TOP_CFG_09C			0x09c
#define TOP_CFG_09C_MASK		GENMASK(12, 0)
#define TOP_CFG_09C_VAL			0x400
#define TOP_BIT_DEPTH			0x100

/* motion compensation (VDEC_CM) */
#define MC_CFG_024			0x024
#define MC_MB_X				0x028
#define MC_MB_Y				0x02c
#define MC_CFG_090			0x090
#define MC_PIC_W			0x208
#define MC_PIC_H			0x20c
#define MC_CFG_220			0x220
#define MC_Y_ADDR_512			0x224
#define MC_C_ADDR_256			0x228
#define MC_PIC_W_MBS			0x22c
#define MC_CFG_238			0x238
#define MC_CFG_250			0x250
#define MC_PIC_W_MBS_M1			0x260
#define MC_PIC_H_MBS_M1			0x264
#define MC_P_L0_Y(i)			(0x3dc + 4 * (i))
#define MC_B_L0_Y(i)			(0x45c + 4 * (i))
#define MC_B_L1_Y(i)			(0x4dc + 4 * (i))
#define MC_C_OFFSET			0x55c
#define MC_Y_ADDR			0x560
/* bit i: entry i of the list is a bottom field (field pictures) */
#define MC_P_L0_BOTTOM			0x564
#define MC_B_L0_BOTTOM			0x568
#define MC_B_L1_BOTTOM			0x56c
#define MC_CFG_5E4			0x5e4
#define MC_CFG_5E4_INIT			0x20000
#define MC_CFG_5E4_RUN			BIT(12)
#define MC_CFG_768			0x768
#define MC_CFG_768_MASK			GENMASK(2, 0)
#define MC_CFG_768_VAL			4
#define MC_PIC_END			0x850
#define MC_PIC_START			0x854
#define MC_CFG_858			0x858
#define MC_CFG_920			0x920
#define MC_CFG_920_RUN			BIT(24)
#define MC_PIC_W_MBS2			0x980
#define MC_VLD_WRAP_ADDR		0xa20
#define MC_PP_WRAP_ADDR			0xa24
#define MC_YC_SEP			0xb5c
#define MC_YC_SEP_EN			BIT(0)
#define MC_YC_SEP_UFO			BIT(4)
#define MC_P_L0_C(i)			(0xb60 + 4 * (i))
#define MC_B_L0_C(i)			(0xbe0 + 4 * (i))
#define MC_B_L1_C(i)			(0xc60 + 4 * (i))
#define MC_10BIT			0xcec
#define MC_10BIT_EN			BIT(0)

/* H.264 syntax (VDEC_AD) */
#define AVC_GETBITS			0x000
#define AVC_SKIP_BITS(n)		(AVC_GETBITS + 4 * (n))
#define AVC_READER_CFG			0x084
#define AVC_READER_CFG_INIT		0x6600e007
#define AVC_READER_CFG_RUN		0x6680e007
#define AVC_SPS				0x088
#define AVC_SHDR0			0x08c
#define AVC_PPS0			0x090
#define AVC_PPS1			0x094
#define AVC_SHDR1			0x098
#define AVC_SHDR2			0x09c
#define AVC_MAX_PIC_NUM			0x0a0
#define AVC_CURR_PIC_NUM		0x0a4
#define AVC_P_REF(i)			(0x0a8 + 4 * (i))
#define AVC_B0_REF(i)			(0x128 + 4 * (i))
#define AVC_B1_REF(i)			(0x1a8 + 4 * (i))
#define AVC_REF_PIC_NUM			GENMASK(18, 0)
#define AVC_REF_LONG_TERM		BIT(19)
#define AVC_PRED_WEIGHT_CFG		0x0f0
#define AVC_PRED_WEIGHT_CFG_EN		BIT(0)
#define AVC_P_LIST_INIT			0x228
#define AVC_REORDER			0x22c
#define AVC_PRED_WEIGHT			0x230
#define AVC_CABAC_INIT			0x234
#define AVC_DEC_START			0x238
#define AVC_UE				0x23c
#define AVC_SE				0x240
#define AVC_BUSY			0x25c
#define AVC_BUSY_CABAC_INIT		BIT(25)
#define AVC_CFG_26C			0x26c
#define AVC_ERR_STA			0x270
#define AVC_CFG_280			0x280
/*
 * Error bits of AVC_ERR_STA that mark a damaged picture, also written to
 * AVC_CFG_26C and AVC_CFG_280. Bit 14 is left out: it is set for correct
 * CABAC P and B pictures.
 */
#define AVC_ERR_MASK			0x7fffbcb7
#define AVC_BS_INIT			0x28c
#define AVC_CFG_2B4			0x2b4
#define AVC_CFG_2B8			0x2b8
#define AVC_SC_CFG			0x2c8
#define AVC_SC_CFG_VAL			0xf900a03b
#define AVC_CFG_2D0			0x2d0
#define AVC_CFG_2D0_INIT		0x8009101
#define AVC_CFG_2D0_RUN			0x8001201
#define AVC_CFG_2D4			0x2d4
#define AVC_CFG_2D4_VAL			0x204008
#define AVC_SC_SEARCH			0x2d8

#define AVC_SPS_CHROMA_FORMAT		GENMASK(1, 0)
#define AVC_SPS_LOG2_MAX_FRAME_NUM	GENMASK(5, 2)
#define AVC_SPS_POC_TYPE		GENMASK(7, 6)
#define AVC_SPS_LOG2_MAX_POC_LSB	GENMASK(11, 8)
#define AVC_SPS_DELTA_POC_ZERO		BIT(12)
#define AVC_SPS_MAX_NUM_REF_FRAMES	GENMASK(17, 13)
#define AVC_SPS_FRAME_MBS_ONLY		BIT(18)
#define AVC_SPS_MBAFF			BIT(19)
#define AVC_SPS_DIRECT_8X8		BIT(20)
#define AVC_SPS_ENABLE			BIT(21)

#define AVC_PPS0_CABAC			BIT(0)
#define AVC_PPS0_BOTTOM_POC		BIT(1)
#define AVC_PPS0_WEIGHTED_PRED		BIT(2)
#define AVC_PPS0_WEIGHTED_BIPRED	GENMASK(4, 3)
#define AVC_PPS0_PIC_INIT_QP		GENMASK(10, 5)
#define AVC_PPS0_CHROMA_QP_OFFSET	GENMASK(15, 11)
#define AVC_PPS0_DEBLOCKING_CTRL	BIT(16)
#define AVC_PPS0_CONSTR_INTRA		BIT(17)
#define AVC_PPS0_TRANSFORM_8X8		BIT(18)
#define AVC_PPS0_CHROMA_QP_OFFSET2	GENMASK(23, 19)

#define AVC_PPS1_NUM_REF_L0		GENMASK(4, 0)
#define AVC_PPS1_NUM_REF_L1		GENMASK(9, 5)
#define AVC_PPS1_SCL_4X4		GENMASK(15, 10)
#define AVC_PPS1_SCL_8X8		GENMASK(17, 16)
#define AVC_PPS1_SCL_MATRIX		BIT(18)

#define AVC_SHDR1_SLICE_TYPE		GENMASK(16, 13)
#define AVC_SHDR1_FIELD_PIC		BIT(17)
#define AVC_SHDR1_BOTTOM_FIELD		BIT(18)
#define AVC_SHDR1_DIRECT_SPATIAL	BIT(19)
#define AVC_SHDR1_NUM_REF_L0		GENMASK(24, 20)
#define AVC_SHDR1_NUM_REF_L1		GENMASK(29, 25)

#define AVC_SHDR2_QP_DELTA		GENMASK(6, 0)
#define AVC_SHDR2_DEBLOCK_IDC		GENMASK(8, 7)
#define AVC_SHDR2_ALPHA			GENMASK(12, 9)
#define AVC_SHDR2_BETA			GENMASK(16, 13)
#define AVC_SHDR2_NAL_REF_IDC		GENMASK(18, 17)
#define AVC_SHDR2_IDR			BIT(19)
#define AVC_SHDR2_CABAC_INIT_IDC	GENMASK(21, 20)

/* motion vectors (VDEC_AV) */
#define MV_P_REF(i)			(0x000 + 4 * (i))
#define MV_B0_REF(i)			(0x080 + 4 * (i))
#define MV_B1_REF(i)			(0x100 + 4 * (i))
#define MV_COL_ADDR(i)			(0x180 + 4 * (i))
/*
 * Motion vector buffers in 16-byte units; the vectors of the bottom field
 * or the lower macroblocks start 2 units further with the reduced layout
 * for direct_8x8_inference_flag and 4 units further with the full layout.
 */
#define MV_ADDR_BOTTOM_REDUCED		2
#define MV_ADDR_BOTTOM_FULL		4
#define MV_POC				0x200
#define MV_POC_TOP			0x204
#define MV_POC_BOTTOM			0x208
#define MV_DST_ADDR			0x20c
#define MV_CFG_220			0x220
/* reduced motion vector layout: only for direct_8x8_inference_flag */
#define MV_CFG_220_REDUCED		BIT(5)

#define MV_REF_SLOT			GENMASK(27, 23)
#define MV_REF_BOTTOM			BIT(22)
/* frame picture: the top field of the reference comes first in output order */
#define MV_REF_TOP_FIRST		BIT(21)
#define MV_REF_LONG_TERM		BIT(20)
/* how the reference was decoded: as fields, as MBAFF frame or (none) as frame */
#define MV_REF_CODED_FIELD		BIT(19)
#define MV_REF_CODED_MBAFF		BIT(18)
#define MV_REF_POC			GENMASK(17, 0)

/* post-processing (VDEC_PP) */
#define PP_CFG_B80			0xb80
#define PP_CFG_B80_JUMP			BIT(2)
#define PP_CFG_B80_LARGE		BIT(12)
#define PP_10BIT			0xb8c
#define PP_10BIT_EN			BIT(0)

/* control (VDEC_MISC) */
#define MISC_CFG_000			0x000
#define MISC_CFG_000_INIT		0x801ff0
#define MISC_CFG_004			0x004
#define MISC_CFG_004_IDLE		0x10
#define MISC_CFG_004_LOAD		3
#define MISC_CFG_028			0x028
#define MISC_CFG_028_EN			BIT(0)
#define MISC_PIC_RANGE_START		0x03c
#define MISC_PIC_RANGE_END		0x040
#define MISC_MV_RANGE_START		0x054
#define MISC_MV_RANGE_END		0x058
#define MISC_SYS_SEL			0x084
#define MISC_SYS_SEL_VAL		2
#define MISC_SPEC_CON			0x0c8
#define MISC_SPEC_CON_VAL		0xfff7
#define MISC_FUNC_CON			0x0cc
#define MISC_FUNC_CON_VAL		0x51e30182
#define MISC_DCM_CON			0x0ec
#define MISC_PRED_WEIGHT_CFG		0x0f0
#define MISC_SRST			0x0f4
#define MISC_SIZE_CFG			0x160
#define MISC_SIZE_CFG_NORMAL		0x65ff10
#define MISC_SIZE_CFG_LARGE		0x1eff10
#define MISC_CFG_178			0x178
#define MISC_CFG_178_VAL		0xfffffe60
#define MISC_CFG_1F0			0x1f0
#define MISC_CFG_1F0_RESET		0

/* pictures above this size in pixels need the large picture settings */
#define VDEC_H264_LARGE_PIC		(1920 * 1088)

#define VDEC_H264_PRED_SZ		0x8000
#define VDEC_H264_VLD_WRAP_SZ		0xf000
#define VDEC_H264_PP_WRAP_SZ		0x1e000
#define VDEC_H264_MV_EXTRA		512

/* chroma sample value of a 4:0:0 picture (8 bits) */
#define VDEC_H264_CHROMA_MID		0x80

/* zeros the reader needs behind the bitstream */
#define VDEC_H264_BS_PAD		384
#define VDEC_H264_BS_MIN_PAD		64

#define VDEC_H264_MAX_FB		MTK_VDEC_DIRECT_MAX_CAPTURE
/* slots of MV_REF_SLOT */
#define VDEC_H264_SLOTS			32
#define VDEC_H264_LIST_LEN		32
#define VDEC_H264_MAX_NALS		16
#define VDEC_H264_MAX_MMCO		66

#define VDEC_H264_POLL_US		10000
#define VDEC_H264_TIMEOUT_MS		500

enum {
	SLICE_P,
	SLICE_B,
	SLICE_I,
	SLICE_SP,
	SLICE_SI,
};

/**
 * struct vdec_h264_direct_hdr - fields of the first slice header
 * @nal_ref_idc:	nal_ref_idc of the slice NAL unit
 * @idr:		IDR slice
 * @slice_type:		slice_type as coded (0 to 9)
 * @type:		slice_type % 5
 * @first_mb:		first_mb_in_slice
 * @field_pic:		field_pic_flag
 * @bottom:		bottom_field_flag
 * @direct_spatial:	direct_spatial_mv_pred_flag
 * @num_ref_l0:		num_ref_idx_l0_active_minus1
 * @num_ref_l1:		num_ref_idx_l1_active_minus1
 * @cabac_init_idc:	cabac_init_idc
 * @qp_delta:		slice_qp_delta
 * @deblock_idc:	disable_deblocking_filter_idc
 * @alpha:		slice_alpha_c0_offset_div2
 * @beta:		slice_beta_offset_div2
 */
struct vdec_h264_direct_hdr {
	u32 nal_ref_idc;
	bool idr;
	u32 slice_type;
	u32 type;
	u32 first_mb;
	bool field_pic;
	bool bottom;
	u32 direct_spatial;
	u32 num_ref_l0;
	u32 num_ref_l1;
	u32 cabac_init_idc;
	s32 qp_delta;
	u32 deblock_idc;
	s32 alpha;
	s32 beta;
};

/**
 * struct vdec_h264_direct_ref - reference picture as the decoder sees it
 * @y:		luma address
 * @c:		chroma address
 * @mv:		motion vector buffer
 * @slot:	slot of the picture, see vdec_h264_pick_slot()
 * @pic_num:	PicNum or LongTermPicNum of the frame or field
 * @poc:	top and bottom field order count
 * @bottom:	the entry is the bottom field (field pictures)
 * @long_term:	long-term reference
 * @coding:	MV_REF_CODED_* bits: how the picture was decoded
 */
struct vdec_h264_direct_ref {
	dma_addr_t y;
	dma_addr_t c;
	dma_addr_t mv;
	u32 slot;
	s32 pic_num;
	s32 poc[2];
	bool bottom;
	bool long_term;
	u32 coding;
};

/**
 * struct vdec_h264_direct_inst - H.264 decoder instance
 * @ctx:	decoder context
 * @dev:	decoder device
 * @pred_buf:	prediction working buffer
 * @vld_wrap:	reader working buffer
 * @pp_wrap:	post-processing working buffer
 * @mv_buf:	motion vector buffer of each capture buffer
 * @coding:	MV_REF_CODED_* bits of the picture in each capture buffer
 * @slot:	slot of the picture in each capture buffer
 * @sps:	sequence parameter set of the current picture
 * @pps:	picture parameter set of the current picture
 * @scaling:	scaling matrix of the current picture
 * @dec:	decode parameters of the current picture
 * @hdr:	fields of the first slice header
 * @p:		initial P reference list
 * @b0:		initial B reference list 0
 * @b1:		initial B reference list 1
 * @num_refs:	entries in each reference list
 * @field:	the picture is a field
 * @bottom:	the picture is the bottom field
 * @ten_bit:	10-bit luma and chroma
 * @mono:	4:0:0, the host fills the chroma plane
 * @mbw:	width in macroblocks
 * @mbh:	height in macroblocks (frame)
 * @mbh_pic:	height of the picture in macroblocks (field or frame)
 */
struct vdec_h264_direct_inst {
	struct mtk_vcodec_dec_ctx *ctx;
	struct mtk_vcodec_dec_dev *dev;
	struct mtk_vcodec_mem pred_buf;
	struct mtk_vcodec_mem vld_wrap;
	struct mtk_vcodec_mem pp_wrap;
	struct mtk_vcodec_mem mv_buf[VDEC_H264_MAX_FB];
	u32 coding[VDEC_H264_MAX_FB];
	u8 slot[VDEC_H264_MAX_FB];

	const struct v4l2_ctrl_h264_sps *sps;
	const struct v4l2_ctrl_h264_pps *pps;
	const struct v4l2_ctrl_h264_scaling_matrix *scaling;
	const struct v4l2_ctrl_h264_decode_params *dec;
	struct vdec_h264_direct_hdr hdr;

	struct vdec_h264_direct_ref p[VDEC_H264_LIST_LEN];
	struct vdec_h264_direct_ref b0[VDEC_H264_LIST_LEN];
	struct vdec_h264_direct_ref b1[VDEC_H264_LIST_LEN];
	unsigned int num_refs;

	bool field;
	bool bottom;
	bool ten_bit;
	bool mono;
	unsigned int mbw;
	unsigned int mbh;
	unsigned int mbh_pic;
};

static inline void vdec_write(struct vdec_h264_direct_inst *inst, int blk,
			      u32 off, u32 val)
{
	writel(val, inst->dev->reg_base[blk] + off);
}

static inline u32 vdec_read(struct vdec_h264_direct_inst *inst, int blk, u32 off)
{
	return readl(inst->dev->reg_base[blk] + off);
}

static inline void vdec_mask(struct vdec_h264_direct_inst *inst, int blk,
			     u32 off, u32 clear, u32 set)
{
	vdec_write(inst, blk, off, (vdec_read(inst, blk, off) & ~clear) | set);
}

static int vdec_poll(struct vdec_h264_direct_inst *inst, int blk, u32 off,
		     u32 mask, bool set, const char *what)
{
	void __iomem *reg = inst->dev->reg_base[blk] + off;
	u32 val;
	int ret;

	ret = readl_poll_timeout(reg, val, !!(val & mask) == set, 1,
				 VDEC_H264_POLL_US);
	if (ret)
		mtk_vdec_err(inst->ctx, "%s timeout: %#x", what, val);

	return ret;
}

/* ---------------- slice header through the bitstream reader ---------------- */

/* read n (1 to 32) bits */
static u32 vdec_h264_u(struct vdec_h264_direct_inst *inst, unsigned int n)
{
	u32 win = vdec_read(inst, VDEC_AD, AVC_GETBITS);

	vdec_read(inst, VDEC_AD, AVC_SKIP_BITS(n));
	return n == 32 ? win : win >> (32 - n);
}

static u32 vdec_h264_ue(struct vdec_h264_direct_inst *inst)
{
	return vdec_read(inst, VDEC_AD, AVC_UE);
}

static s32 vdec_h264_se(struct vdec_h264_direct_inst *inst)
{
	return (s32)vdec_read(inst, VDEC_AD, AVC_SE);
}

/*
 * Move the reader to the first slice NAL unit of the buffer and behind its
 * start code and NAL header. The search for a start code skips the one the
 * reader stands on, so it runs only if the reader is not on one already.
 */
static int vdec_h264_find_slice(struct vdec_h264_direct_inst *inst)
{
	struct vdec_h264_direct_hdr *hdr = &inst->hdr;
	u32 win, type;
	int i, ret;

	win = vdec_read(inst, VDEC_AD, AVC_GETBITS);
	vdec_read(inst, VDEC_AD, AVC_READER_CFG);
	for (i = 0; i < VDEC_H264_MAX_NALS; i++) {
		if (i || (win >> 8) != 1) {
			vdec_write(inst, VDEC_AD, AVC_SC_SEARCH, 1);
			ret = vdec_poll(inst, VDEC_AD, AVC_SC_SEARCH, BIT(0), false,
					"start code search");
			if (ret)
				return ret;
			vdec_read(inst, VDEC_AD, AVC_SC_CFG);
			win = vdec_read(inst, VDEC_AD, AVC_GETBITS);
			if ((win >> 8) != 1)
				break;
		}

		type = win & 0x1f;
		if (type == NAL_NON_IDR_SLICE || type == NAL_IDR_SLICE) {
			vdec_read(inst, VDEC_AD, AVC_SKIP_BITS(32));
			hdr->nal_ref_idc = (win >> 5) & 0x3;
			hdr->idr = type == NAL_IDR_SLICE;
			return 0;
		}
	}

	mtk_vdec_err(inst->ctx, "no slice in the bitstream buffer");
	return -EINVAL;
}

/* slice header up to and including num_ref_idx_active_override_flag */
static int vdec_h264_read_hdr_start(struct vdec_h264_direct_inst *inst)
{
	const struct v4l2_ctrl_h264_sps *sps = inst->sps;
	const struct v4l2_ctrl_h264_pps *pps = inst->pps;
	struct vdec_h264_direct_hdr *hdr = &inst->hdr;
	bool bottom_poc;
	u32 val;

	hdr->first_mb = vdec_h264_ue(inst);
	hdr->slice_type = vdec_h264_ue(inst);
	hdr->type = hdr->slice_type % 5;
	if (hdr->slice_type > 9 || hdr->type == SLICE_SP || hdr->type == SLICE_SI) {
		mtk_vdec_err(inst->ctx, "slice type %u not supported", hdr->slice_type);
		return -EOPNOTSUPP;
	}
	val = vdec_h264_ue(inst);
	if (val != pps->pic_parameter_set_id)
		mtk_vdec_debug(inst->ctx, "slice refers to PPS %u, control has %u",
			       val, pps->pic_parameter_set_id);

	val = vdec_h264_u(inst, sps->log2_max_frame_num_minus4 + 4);
	if (val != inst->dec->frame_num)
		mtk_vdec_debug(inst->ctx, "frame_num %u, control has %u", val,
			       inst->dec->frame_num);
	hdr->field_pic = false;
	hdr->bottom = false;
	if (!(sps->flags & V4L2_H264_SPS_FLAG_FRAME_MBS_ONLY)) {
		hdr->field_pic = vdec_h264_u(inst, 1);
		if (hdr->field_pic)
			hdr->bottom = vdec_h264_u(inst, 1);
	}
	if (hdr->field_pic != inst->field || hdr->bottom != inst->bottom) {
		mtk_vdec_err(inst->ctx, "picture structure differs from the controls");
		return -EINVAL;
	}
	if (hdr->idr)
		vdec_h264_ue(inst);			/* idr_pic_id */

	/* the order count of the bottom field is coded in frames only */
	bottom_poc = !hdr->field_pic &&
		     (pps->flags & V4L2_H264_PPS_FLAG_BOTTOM_FIELD_PIC_ORDER_IN_FRAME_PRESENT);
	if (sps->pic_order_cnt_type == 0) {
		vdec_h264_u(inst, sps->log2_max_pic_order_cnt_lsb_minus4 + 4);
		if (bottom_poc)
			vdec_h264_se(inst);		/* delta_pic_order_cnt_bottom */
	} else if (sps->pic_order_cnt_type == 1 &&
		   !(sps->flags & V4L2_H264_SPS_FLAG_DELTA_PIC_ORDER_ALWAYS_ZERO)) {
		vdec_h264_se(inst);			/* delta_pic_order_cnt[0] */
		if (bottom_poc)
			vdec_h264_se(inst);		/* delta_pic_order_cnt[1] */
	}
	if (pps->flags & V4L2_H264_PPS_FLAG_REDUNDANT_PIC_CNT_PRESENT)
		vdec_h264_ue(inst);			/* redundant_pic_cnt */

	hdr->direct_spatial = 0;
	if (hdr->type == SLICE_B)
		hdr->direct_spatial = vdec_h264_u(inst, 1);

	hdr->num_ref_l0 = pps->num_ref_idx_l0_default_active_minus1;
	hdr->num_ref_l1 = pps->num_ref_idx_l1_default_active_minus1;
	if (hdr->type != SLICE_I && vdec_h264_u(inst, 1)) {
		hdr->num_ref_l0 = vdec_h264_ue(inst);
		if (hdr->type == SLICE_B)
			hdr->num_ref_l1 = vdec_h264_ue(inst);
	}
	if (hdr->num_ref_l0 > 31 || hdr->num_ref_l1 > 31)
		return -EINVAL;

	return 0;
}

/*
 * Slice header from dec_ref_pic_marking() to the slice data, after the
 * decoder has read ref_pic_list_modification() and pred_weight_table().
 */
static int vdec_h264_read_hdr_end(struct vdec_h264_direct_inst *inst)
{
	const struct v4l2_ctrl_h264_pps *pps = inst->pps;
	struct vdec_h264_direct_hdr *hdr = &inst->hdr;
	u32 op;
	int i;

	/*
	 * dec_ref_pic_marking(): the reference marking is done by user space,
	 * the decoder only needs the reader behind it.
	 */
	if (hdr->nal_ref_idc) {
		if (hdr->idr) {
			/* no_output_of_prior_pics_flag, long_term_reference_flag */
			vdec_h264_u(inst, 2);
		} else if (vdec_h264_u(inst, 1)) {
			/* adaptive_ref_pic_marking_mode_flag: skip the operations */
			for (i = 0; i < VDEC_H264_MAX_MMCO; i++) {
				op = vdec_h264_ue(inst);
				if (!op)
					break;
				if (op > 6)
					return -EINVAL;
				if (op != 5)
					vdec_h264_ue(inst);
				if (op == 3)
					vdec_h264_ue(inst);
			}
			if (i == VDEC_H264_MAX_MMCO)
				return -EINVAL;
		}
	}

	hdr->cabac_init_idc = 0;
	if ((pps->flags & V4L2_H264_PPS_FLAG_ENTROPY_CODING_MODE) &&
	    hdr->type != SLICE_I)
		hdr->cabac_init_idc = vdec_h264_ue(inst);
	hdr->qp_delta = vdec_h264_se(inst);

	hdr->deblock_idc = 0;
	hdr->alpha = 0;
	hdr->beta = 0;
	if (pps->flags & V4L2_H264_PPS_FLAG_DEBLOCKING_FILTER_CONTROL_PRESENT) {
		hdr->deblock_idc = vdec_h264_ue(inst);
		if (hdr->deblock_idc != 1) {
			hdr->alpha = vdec_h264_se(inst);
			hdr->beta = vdec_h264_se(inst);
		}
	}
	if (hdr->cabac_init_idc > 2 || hdr->deblock_idc > 2)
		return -EINVAL;

	return 0;
}

/* ---------------- parameters, buffers and reference lists ---------------- */

static int vdec_h264_get_params(struct vdec_h264_direct_inst *inst)
{
	struct mtk_vcodec_dec_ctx *ctx = inst->ctx;
	const struct mtk_vdec_direct_data *data = inst->dev->vdec_pdata->direct;
	const struct v4l2_ctrl_h264_sps *sps;
	const struct v4l2_ctrl_h264_pps *pps;
	bool mt2110;
	u32 fourcc;

	inst->dec = mtk_vdec_h264_get_ctrl_ptr(ctx, V4L2_CID_STATELESS_H264_DECODE_PARAMS);
	inst->sps = mtk_vdec_h264_get_ctrl_ptr(ctx, V4L2_CID_STATELESS_H264_SPS);
	inst->pps = mtk_vdec_h264_get_ctrl_ptr(ctx, V4L2_CID_STATELESS_H264_PPS);
	inst->scaling = mtk_vdec_h264_get_ctrl_ptr(ctx, V4L2_CID_STATELESS_H264_SCALING_MATRIX);
	if (IS_ERR(inst->dec) || IS_ERR(inst->sps) || IS_ERR(inst->pps) ||
	    IS_ERR(inst->scaling))
		return -EINVAL;
	sps = inst->sps;
	pps = inst->pps;

	if (sps->chroma_format_idc > 1) {
		mtk_vdec_err(ctx, "only 4:2:0 and 4:0:0 supported");
		return -EOPNOTSUPP;
	}
	inst->mono = !sps->chroma_format_idc;
	if (inst->mono && sps->bit_depth_luma_minus8) {
		mtk_vdec_err(ctx, "4:0:0 only with 8 bits");
		return -EOPNOTSUPP;
	}
	if (sps->bit_depth_luma_minus8 != sps->bit_depth_chroma_minus8 ||
	    (sps->bit_depth_luma_minus8 && sps->bit_depth_luma_minus8 != 2)) {
		mtk_vdec_err(ctx, "only 8 and 10 bits supported");
		return -EOPNOTSUPP;
	}
	if (pps->num_slice_groups_minus1) {
		mtk_vdec_err(ctx, "slice groups not supported");
		return -EOPNOTSUPP;
	}

	inst->ten_bit = sps->bit_depth_luma_minus8 == 2;
	fourcc = ctx->q_data[MTK_Q_DATA_DST].fmt->fourcc;
	mt2110 = fourcc == V4L2_PIX_FMT_MT2110R;
	if (inst->ten_bit != mt2110) {
		mtk_vdec_err(ctx, "capture format %p4cc does not fit %u bits", &fourcc,
			     sps->bit_depth_luma_minus8 + 8);
		return -EINVAL;
	}

	inst->field = inst->dec->flags & V4L2_H264_DECODE_PARAM_FLAG_FIELD_PIC;
	inst->bottom = inst->dec->flags & V4L2_H264_DECODE_PARAM_FLAG_BOTTOM_FIELD;
	if (inst->field && (sps->flags & V4L2_H264_SPS_FLAG_FRAME_MBS_ONLY))
		return -EINVAL;

	inst->mbw = sps->pic_width_in_mbs_minus1 + 1;
	inst->mbh = (sps->pic_height_in_map_units_minus1 + 1) *
		    (sps->flags & V4L2_H264_SPS_FLAG_FRAME_MBS_ONLY ? 1 : 2);
	inst->mbh_pic = inst->field ? inst->mbh / 2 : inst->mbh;
	if (inst->mbw * MB_UNIT_LEN > data->max_width ||
	    inst->mbh * MB_UNIT_LEN > data->max_height) {
		mtk_vdec_err(ctx, "picture %ux%u too large", inst->mbw * MB_UNIT_LEN,
			     inst->mbh * MB_UNIT_LEN);
		return -EINVAL;
	}

	return 0;
}

/* 10 bits: 640 instead of 512 bytes per 16x32 tile, planes 512-byte aligned */
static unsigned int vdec_h264_plane_size(unsigned int width, unsigned int height,
					 bool ten_bit)
{
	unsigned int size = width * ALIGN(height, 32);

	return ten_bit ? ALIGN(size * 5 / 4, 512) : size;
}

static unsigned int vdec_h264_luma_size(struct vdec_h264_direct_inst *inst)
{
	return vdec_h264_plane_size(inst->mbw * MB_UNIT_LEN, inst->mbh * MB_UNIT_LEN,
				    inst->ten_bit);
}

static bool vdec_h264_mv_reduced(struct vdec_h264_direct_inst *inst)
{
	return inst->sps->flags & V4L2_H264_SPS_FLAG_DIRECT_8X8_INFERENCE;
}

/*
 * 64 bytes per macroblock of the 32-line aligned picture for the reduced
 * layout, twice that for the full one.
 */
static unsigned int vdec_h264_mv_size(struct vdec_h264_direct_inst *inst)
{
	unsigned int size = (4 * inst->mbw * MB_UNIT_LEN) *
			    ALIGN(inst->mbh * MB_UNIT_LEN, 32) / MB_UNIT_LEN;

	if (!vdec_h264_mv_reduced(inst))
		size *= 2;

	return size + VDEC_H264_MV_EXTRA;
}

/* motion vector address of a picture, of its bottom part with @bottom */
static u32 vdec_h264_mv_addr(struct vdec_h264_direct_inst *inst, dma_addr_t mv,
			     bool bottom)
{
	u32 val = mv >> 4;

	if (bottom)
		val += vdec_h264_mv_reduced(inst) ? MV_ADDR_BOTTOM_REDUCED :
						    MV_ADDR_BOTTOM_FULL;

	return val;
}

static int vdec_h264_mv_buf(struct vdec_h264_direct_inst *inst, unsigned int idx)
{
	struct mtk_vcodec_mem *mem = &inst->mv_buf[idx];
	unsigned int size = vdec_h264_mv_size(inst);

	if (mem->va && mem->size == size)
		return 0;
	if (mem->va)
		mtk_vcodec_mem_free(inst->ctx, mem);
	mem->size = size;

	return mtk_vcodec_mem_alloc(inst->ctx, mem);
}

static void vdec_h264_ref(struct vdec_h264_direct_inst *inst,
			  struct vdec_h264_direct_ref *ref,
			  const struct vdec_h264_direct_ref *dst,
			  const struct v4l2_h264_reflist_builder *builder,
			  const struct v4l2_h264_reference *entry)
{
	const struct v4l2_h264_dpb_entry *e = &inst->dec->dpb[entry->index];
	struct vb2_queue *vq = v4l2_m2m_get_vq(inst->ctx->m2m_ctx,
					       V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);
	struct vb2_buffer *vb = vb2_find_buffer(vq, e->reference_ts);
	int num = builder->refs[entry->index].frame_num;

	/*
	 * PicNum and LongTermPicNum (8.2.4.1): of a frame FrameNumWrap or
	 * LongTermFrameIdx, of a field twice that, plus one for the parity of
	 * the current field.
	 */
	if (inst->field)
		num = 2 * num + (entry->fields == builder->cur_pic_fields);
	ref->pic_num = num;
	ref->long_term = builder->refs[entry->index].longterm;
	ref->bottom = entry->fields == V4L2_H264_BOTTOM_FIELD_REF;
	ref->poc[0] = e->top_field_order_cnt;
	ref->poc[1] = e->bottom_field_order_cnt;
	if (!vb || vb->index >= VDEC_H264_MAX_FB || !inst->mv_buf[vb->index].va) {
		/* missing reference: decode from the target itself */
		mtk_vdec_debug(inst->ctx, "reference %llu not found", e->reference_ts);
		ref->y = dst->y;
		ref->c = dst->c;
		ref->mv = dst->mv;
		ref->slot = dst->slot;
		ref->coding = dst->coding;
		return;
	}
	ref->y = vb2_dma_contig_plane_dma_addr(vb, 0);
	ref->c = vb2_dma_contig_plane_dma_addr(vb, 1);
	ref->mv = inst->mv_buf[vb->index].dma_addr;
	ref->slot = inst->slot[vb->index];
	ref->coding = inst->coding[vb->index];
}

/*
 * The decoder tells the pictures apart by the slot in the reference words
 * of the motion vector buffers, also in those of the co-located picture.
 * There are fewer slots than capture buffers: a picture gets a slot that no
 * picture of its DPB holds and keeps it while it is referenced. The second
 * field of a frame goes to the buffer of the first one and keeps its slot.
 */
static int vdec_h264_pick_slot(struct vdec_h264_direct_inst *inst,
			       unsigned int idx)
{
	struct vb2_queue *vq = v4l2_m2m_get_vq(inst->ctx->m2m_ctx,
					       V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);
	DECLARE_BITMAP(used, VDEC_H264_SLOTS);
	unsigned int i, slot;

	bitmap_zero(used, VDEC_H264_SLOTS);
	for (i = 0; i < ARRAY_SIZE(inst->dec->dpb); i++) {
		const struct v4l2_h264_dpb_entry *e = &inst->dec->dpb[i];
		struct vb2_buffer *vb;

		if (!(e->flags & V4L2_H264_DPB_ENTRY_FLAG_VALID))
			continue;
		vb = vb2_find_buffer(vq, e->reference_ts);
		if (!vb || vb->index >= VDEC_H264_MAX_FB ||
		    !inst->mv_buf[vb->index].va)
			continue;
		if (vb->index == idx)
			return inst->slot[idx];
		__set_bit(inst->slot[vb->index], used);
	}

	slot = find_first_zero_bit(used, VDEC_H264_SLOTS);
	if (slot >= VDEC_H264_SLOTS)
		return -EINVAL;

	return slot;
}

/*
 * Initial reference lists with all references of the DPB: frames for a
 * frame, fields of alternating parity for a field (8.2.4.2).
 */
static void vdec_h264_build_lists(struct vdec_h264_direct_inst *inst,
				  const struct vdec_h264_direct_ref *dst)
{
	struct v4l2_h264_reference p[V4L2_H264_REF_LIST_LEN];
	struct v4l2_h264_reference b0[V4L2_H264_REF_LIST_LEN];
	struct v4l2_h264_reference b1[V4L2_H264_REF_LIST_LEN];
	struct v4l2_h264_reflist_builder builder;
	unsigned int i;

	v4l2_h264_init_reflist_builder(&builder, inst->dec, inst->sps, inst->dec->dpb);
	v4l2_h264_build_p_ref_list(&builder, p);
	v4l2_h264_build_b_ref_lists(&builder, b0, b1);

	inst->num_refs = builder.num_valid;
	for (i = 0; i < inst->num_refs; i++) {
		vdec_h264_ref(inst, &inst->p[i], dst, &builder, &p[i]);
		vdec_h264_ref(inst, &inst->b0[i], dst, &builder, &b0[i]);
		vdec_h264_ref(inst, &inst->b1[i], dst, &builder, &b1[i]);
	}
}

/*
 * Reference word of the motion vector block for field @bottom of @ref: the
 * slot of the picture, its parity, how it was decoded and, in the B lists,
 * the order count of the field.
 */
static u32 vdec_h264_mv_ref(struct vdec_h264_direct_inst *inst,
			    const struct vdec_h264_direct_ref *ref, bool bottom,
			    bool with_poc)
{
	u32 val;

	val = FIELD_PREP(MV_REF_SLOT, ref->slot) | ref->coding;
	if (bottom)
		val |= MV_REF_BOTTOM;
	if (ref->long_term)
		val |= MV_REF_LONG_TERM;
	if (!inst->field && ref->poc[0] <= ref->poc[1])
		val |= MV_REF_TOP_FIRST;
	if (with_poc)
		val |= FIELD_PREP(MV_REF_POC, ref->poc[bottom] & MV_REF_POC);

	return val;
}

/*
 * PicNum or LongTermPicNum of a reference for the list modification of the
 * decoder: 19 bits, two's complement, and the long-term flag. After
 * frame_num wraps, FrameNumWrap and with it PicNum of the older references
 * is negative.
 */
static u32 vdec_h264_pic_num(const struct vdec_h264_direct_ref *ref)
{
	return ((u32)ref->pic_num & AVC_REF_PIC_NUM) |
	       (ref->long_term ? AVC_REF_LONG_TERM : 0);
}

/*
 * Make room for the zeros the reader needs behind the bitstream and return
 * the end of the reader window, 64-byte aligned.
 */
static int vdec_h264_bs_end(struct vdec_h264_direct_inst *inst,
			    struct mtk_vcodec_mem *bs, dma_addr_t *end)
{
	struct mtk_video_dec_buf *src = container_of(bs, struct mtk_video_dec_buf,
						     bs_buffer);
	dma_addr_t limit = bs->dma_addr +
			   vb2_plane_size(&src->m2m_buf.vb.vb2_buf, 0);
	dma_addr_t data_end = bs->dma_addr + bs->size;

	*end = ALIGN(data_end + VDEC_H264_BS_PAD, 64);
	if (*end > limit)
		*end = ALIGN_DOWN(limit, 64);
	if (*end < data_end + VDEC_H264_BS_MIN_PAD) {
		mtk_vdec_err(inst->ctx, "no room behind the bitstream (%zu of %pad)",
			     bs->size, &limit);
		return -EINVAL;
	}
	memset(bs->va + bs->size, 0, *end - data_end);

	return 0;
}

/* ---------------- register sequence of one picture ---------------- */

static void vdec_h264_reset(struct vdec_h264_direct_inst *inst)
{
	vdec_write(inst, VDEC_MISC, MISC_SRST, 0);
	vdec_read(inst, VDEC_LD, VLD_PROC_STA);
	vdec_write(inst, VDEC_LD, VDEC_VLD_SRST, VLD_SRST_READER | VDEC_VLD_SRST_RESET);
	vdec_write(inst, VDEC_MISC, MISC_SYS_SEL, MISC_SYS_SEL_VAL);
	vdec_read(inst, VDEC_MISC, MISC_CFG_000);
	vdec_write(inst, VDEC_MISC, MISC_CFG_000, MISC_CFG_000_INIT);
	vdec_read(inst, VDEC_MISC, MISC_CFG_000);
	vdec_write(inst, VDEC_MISC, MISC_SPEC_CON, MISC_SPEC_CON_VAL);
	vdec_write(inst, VDEC_MISC, MISC_FUNC_CON, MISC_FUNC_CON_VAL);
	vdec_write(inst, VDEC_MISC, MISC_CFG_178, MISC_CFG_178_VAL);
	vdec_write(inst, VDEC_MISC, MISC_SYS_SEL, MISC_SYS_SEL_VAL);
	vdec_write(inst, VDEC_LD, VDEC_VLD_SRST, 0);
	mtk_vcodec_write_vdecsys(inst->ctx, VDEC_SYS_DCM_CON, 1);
	vdec_write(inst, VDEC_MISC, MISC_DCM_CON, 1);
	vdec_write(inst, VDEC_MISC, MISC_CFG_004, MISC_CFG_004_IDLE);
	vdec_write(inst, VDEC_MISC, MISC_SIZE_CFG, MISC_SIZE_CFG_NORMAL);
	vdec_write(inst, VDEC_MISC, MISC_CFG_1F0, MISC_CFG_1F0_RESET);
}

static int vdec_h264_init_reader(struct vdec_h264_direct_inst *inst,
				 dma_addr_t start, dma_addr_t end)
{
	int ret;

	vdec_write(inst, VDEC_LD, VLD_CFG_0C0, VLD_CFG_0C0_INIT);
	vdec_write(inst, VDEC_AD, AVC_READER_CFG, AVC_READER_CFG_INIT);
	vdec_read(inst, VDEC_TOP, TOP_CFG_054);
	vdec_write(inst, VDEC_TOP, TOP_CFG_054, 1);
	vdec_write(inst, VDEC_CM, MC_CFG_024, 6);

	vdec_write(inst, VDEC_LD, VLD_VSTART, start >> 6);
	vdec_write(inst, VDEC_LD, VLD_VEND, end >> 6);
	vdec_mask(inst, VDEC_LD, VLD_VBAR, 0, VLD_VBAR_EN);
	vdec_write(inst, VDEC_TOP, TOP_PRED_ADDR, inst->pred_buf.dma_addr);
	vdec_write(inst, VDEC_AD, AVC_BS_INIT, 1);
	vdec_write(inst, VDEC_AD, AVC_BS_INIT, 0);

	vdec_read(inst, VDEC_LD, VLD_PROC_STA);
	vdec_mask(inst, VDEC_LD, VLD_WPTR, 0, 0);
	vdec_write(inst, VDEC_LD, VLD_RPTR, start);
	vdec_write(inst, VDEC_LD, VLD_RPTR, start);
	vdec_write(inst, VDEC_LD, VLD_WPTR, end);
	vdec_write(inst, VDEC_AD, AVC_BS_INIT, 1);
	vdec_write(inst, VDEC_LD, VDEC_VLD_SRST, VLD_SRST_READER);
	vdec_write(inst, VDEC_LD, VDEC_VLD_SRST, 0);
	vdec_write(inst, VDEC_LD, VLD_CFG_0CC, 0);
	vdec_write(inst, VDEC_LD, VLD_BS_CTRL, VLD_BS_CTRL_FETCH);
	ret = vdec_poll(inst, VDEC_LD, VLD_FETCH_STA, VLD_FETCH_STA_READY, true,
			"bitstream fetch");
	vdec_write(inst, VDEC_LD, VLD_BS_CTRL, VLD_BS_CTRL_RUN);
	vdec_write(inst, VDEC_AD, AVC_BS_INIT, 0);

	return ret;
}

/*
 * Reference lists: for a frame two motion vector words per entry, one per
 * field; for a field one word per entry and a mask of the bottom fields.
 */
static void vdec_h264_write_lists(struct vdec_h264_direct_inst *inst,
				  const struct vdec_h264_direct_ref *dst)
{
	const struct v4l2_ctrl_h264_sps *sps = inst->sps;
	const struct vdec_h264_direct_ref *r;
	u32 max_pic_num, cur_pic_num, mask_p = 0, mask_b0 = 0, mask_b1 = 0;
	s32 top = inst->dec->top_field_order_cnt;
	s32 bottom = inst->dec->bottom_field_order_cnt;
	unsigned int i;

	max_pic_num = 1 << (sps->log2_max_frame_num_minus4 + 4);
	cur_pic_num = inst->dec->frame_num;
	if (inst->field) {
		max_pic_num *= 2;
		cur_pic_num = 2 * cur_pic_num + 1;
	}

	vdec_write(inst, VDEC_CM, MC_P_L0_BOTTOM, 0);
	vdec_write(inst, VDEC_AD, AVC_P_LIST_INIT, 1);
	vdec_poll(inst, VDEC_AD, AVC_P_LIST_INIT, BIT(0), false, "list init");
	vdec_write(inst, VDEC_AD, AVC_MAX_PIC_NUM, max_pic_num);
	vdec_write(inst, VDEC_AD, AVC_CURR_PIC_NUM, cur_pic_num);

	/* all entries point to a valid picture, unused ones to the target */
	for (i = 0; i < VDEC_H264_LIST_LEN; i++) {
		vdec_write(inst, VDEC_AV, MV_P_REF(i), 0);
		vdec_write(inst, VDEC_CM, MC_P_L0_Y(i), dst->y);
		vdec_write(inst, VDEC_CM, MC_P_L0_C(i), dst->c);
	}
	for (i = 0; i < VDEC_H264_LIST_LEN; i++)
		vdec_write(inst, VDEC_AV, MV_COL_ADDR(i), vdec_h264_mv_addr(inst, dst->mv, false));

	for (i = 0; i < inst->num_refs; i++) {
		r = &inst->p[i];
		vdec_write(inst, VDEC_CM, MC_P_L0_Y(i), r->y);
		vdec_write(inst, VDEC_CM, MC_P_L0_C(i), r->c);
		vdec_write(inst, VDEC_AD, AVC_P_REF(i), vdec_h264_pic_num(r));
		if (inst->field) {
			vdec_write(inst, VDEC_AV, MV_P_REF(i),
				   vdec_h264_mv_ref(inst, r, r->bottom, false));
			mask_p |= r->bottom ? BIT(i) : 0;
			continue;
		}
		vdec_write(inst, VDEC_AV, MV_P_REF(2 * i),
			   vdec_h264_mv_ref(inst, r, false, false));
		vdec_write(inst, VDEC_AV, MV_P_REF(2 * i + 1),
			   vdec_h264_mv_ref(inst, r, true, false));
	}
	vdec_write(inst, VDEC_CM, MC_P_L0_BOTTOM, mask_p);

	if (inst->field) {
		vdec_write(inst, VDEC_AV, MV_POC,
			   (inst->bottom ? bottom : top) & MV_REF_POC);
	} else {
		vdec_write(inst, VDEC_AV, MV_POC_TOP, top & MV_REF_POC);
		vdec_write(inst, VDEC_AV, MV_POC_BOTTOM, bottom & MV_REF_POC);
		vdec_write(inst, VDEC_AV, MV_POC, min(top, bottom) & MV_REF_POC);
	}

	for (i = 0; i < VDEC_H264_LIST_LEN; i++) {
		vdec_write(inst, VDEC_AV, MV_B0_REF(i), 0);
		vdec_write(inst, VDEC_AV, MV_B1_REF(i), 0);
		vdec_write(inst, VDEC_CM, MC_B_L0_Y(i), dst->y);
		vdec_write(inst, VDEC_CM, MC_B_L0_C(i), dst->c);
		vdec_write(inst, VDEC_CM, MC_B_L1_Y(i), dst->y);
		vdec_write(inst, VDEC_CM, MC_B_L1_C(i), dst->c);
	}

	if (inst->num_refs && vdec_h264_mv_reduced(inst))
		vdec_mask(inst, VDEC_AV, MV_CFG_220, 0, MV_CFG_220_REDUCED);
	for (i = 0; i < inst->num_refs; i++) {
		r = &inst->b0[i];
		vdec_write(inst, VDEC_CM, MC_B_L0_Y(i), r->y);
		vdec_write(inst, VDEC_CM, MC_B_L0_C(i), r->c);
		vdec_write(inst, VDEC_AD, AVC_B0_REF(i), vdec_h264_pic_num(r));
		if (inst->field) {
			vdec_write(inst, VDEC_AV, MV_B0_REF(i),
				   vdec_h264_mv_ref(inst, r, r->bottom, true));
			mask_b0 |= r->bottom ? BIT(i) : 0;
		} else {
			vdec_write(inst, VDEC_AV, MV_B0_REF(2 * i),
				   vdec_h264_mv_ref(inst, r, false, true));
			vdec_write(inst, VDEC_AV, MV_B0_REF(2 * i + 1),
				   vdec_h264_mv_ref(inst, r, true, true));
		}

		r = &inst->b1[i];
		vdec_write(inst, VDEC_CM, MC_B_L1_Y(i), r->y);
		vdec_write(inst, VDEC_CM, MC_B_L1_C(i), r->c);
		vdec_write(inst, VDEC_AD, AVC_B1_REF(i), vdec_h264_pic_num(r));
		if (inst->field) {
			vdec_write(inst, VDEC_AV, MV_COL_ADDR(i),
				   vdec_h264_mv_addr(inst, r->mv, r->bottom));
			vdec_write(inst, VDEC_AV, MV_B1_REF(i),
				   vdec_h264_mv_ref(inst, r, r->bottom, true));
			mask_b1 |= r->bottom ? BIT(i) : 0;
			continue;
		}
		vdec_write(inst, VDEC_AV, MV_B1_REF(2 * i),
			   vdec_h264_mv_ref(inst, r, false, true));
		vdec_write(inst, VDEC_AV, MV_B1_REF(2 * i + 1),
			   vdec_h264_mv_ref(inst, r, true, true));
		vdec_write(inst, VDEC_AV, MV_COL_ADDR(2 * i),
			   vdec_h264_mv_addr(inst, r->mv, false));
		vdec_write(inst, VDEC_AV, MV_COL_ADDR(2 * i + 1),
			   vdec_h264_mv_addr(inst, r->mv, true));
	}
	vdec_write(inst, VDEC_CM, MC_B_L0_BOTTOM, mask_b0);
	vdec_write(inst, VDEC_CM, MC_B_L1_BOTTOM, mask_b1);
}

static void vdec_h264_write_scl_list(struct vdec_h264_direct_inst *inst,
				     u32 addr, const u8 *list, unsigned int len)
{
	unsigned int i;

	for (i = 0; i < len; i += 4, addr += 4) {
		vdec_write(inst, VDEC_LD, VLD_SCL_ADDR, VLD_SCL_ADDR_WRITE | addr);
		vdec_write(inst, VDEC_LD, VLD_SCL_DATA,
			   list[i] << 24 | list[i + 1] << 16 | list[i + 2] << 8 | list[i + 3]);
		vdec_write(inst, VDEC_LD, VLD_SCL_ADDR, VLD_SCL_ADDR_DONE | addr);
	}
}

/*
 * The decoder keeps the lists across pictures; it takes a list from this
 * memory only when its bit in AVC_PPS1 is set, otherwise the flat list.
 */
static u32 vdec_h264_write_scaling(struct vdec_h264_direct_inst *inst)
{
	const struct v4l2_ctrl_h264_scaling_matrix *m = inst->scaling;
	u32 bits;
	int i;

	if (!(inst->pps->flags & V4L2_H264_PPS_FLAG_SCALING_MATRIX_PRESENT))
		return 0;

	for (i = 0; i < 6; i++)
		vdec_h264_write_scl_list(inst, SCL_4X4_ADDR(i), m->scaling_list_4x4[i], 16);
	bits = FIELD_PREP(AVC_PPS1_SCL_4X4, 0x3f) | AVC_PPS1_SCL_MATRIX;

	if (inst->pps->flags & V4L2_H264_PPS_FLAG_TRANSFORM_8X8_MODE) {
		vdec_h264_write_scl_list(inst, SCL_8X8_INTRA_Y_ADDR, m->scaling_list_8x8[0], 64);
		vdec_h264_write_scl_list(inst, SCL_8X8_INTER_Y_ADDR, m->scaling_list_8x8[1], 64);
		bits |= FIELD_PREP(AVC_PPS1_SCL_8X8, 0x3);
	}

	return bits;
}

static void vdec_h264_write_pic_params(struct vdec_h264_direct_inst *inst, u32 scl_bits)
{
	const struct v4l2_ctrl_h264_sps *sps = inst->sps;
	const struct v4l2_ctrl_h264_pps *pps = inst->pps;
	struct vdec_h264_direct_hdr *hdr = &inst->hdr;
	u32 val;

	val = FIELD_PREP(AVC_SPS_CHROMA_FORMAT, sps->chroma_format_idc) |
	      FIELD_PREP(AVC_SPS_LOG2_MAX_FRAME_NUM, sps->log2_max_frame_num_minus4) |
	      FIELD_PREP(AVC_SPS_POC_TYPE, sps->pic_order_cnt_type) |
	      FIELD_PREP(AVC_SPS_LOG2_MAX_POC_LSB, sps->log2_max_pic_order_cnt_lsb_minus4) |
	      FIELD_PREP(AVC_SPS_MAX_NUM_REF_FRAMES, sps->max_num_ref_frames) |
	      AVC_SPS_ENABLE;
	if (sps->flags & V4L2_H264_SPS_FLAG_DELTA_PIC_ORDER_ALWAYS_ZERO)
		val |= AVC_SPS_DELTA_POC_ZERO;
	if (sps->flags & V4L2_H264_SPS_FLAG_FRAME_MBS_ONLY)
		val |= AVC_SPS_FRAME_MBS_ONLY;
	if (sps->flags & V4L2_H264_SPS_FLAG_MB_ADAPTIVE_FRAME_FIELD)
		val |= AVC_SPS_MBAFF;
	if (sps->flags & V4L2_H264_SPS_FLAG_DIRECT_8X8_INFERENCE)
		val |= AVC_SPS_DIRECT_8X8;
	vdec_write(inst, VDEC_AD, AVC_SPS, val);
	vdec_write(inst, VDEC_TOP, TOP_PIC_MB_SIZE_M1,
		   sps->pic_height_in_map_units_minus1 << 16 | sps->pic_width_in_mbs_minus1);
	vdec_write(inst, VDEC_TOP, TOP_BIT_DEPTH,
		   sps->bit_depth_luma_minus8 | sps->bit_depth_chroma_minus8 << 4);
	/* 10 bits: MT2110 output, which does not work in jump mode */
	if (inst->ten_bit) {
		vdec_mask(inst, VDEC_CM, MC_10BIT, 0, MC_10BIT_EN);
		vdec_mask(inst, VDEC_PP, PP_10BIT, 0, PP_10BIT_EN);
		vdec_mask(inst, VDEC_PP, PP_CFG_B80, PP_CFG_B80_JUMP, 0);
	} else {
		vdec_mask(inst, VDEC_CM, MC_10BIT, MC_10BIT_EN, 0);
		vdec_mask(inst, VDEC_PP, PP_10BIT, PP_10BIT_EN, 0);
		vdec_mask(inst, VDEC_PP, PP_CFG_B80, 0, PP_CFG_B80_JUMP);
	}

	val = FIELD_PREP(AVC_PPS0_WEIGHTED_BIPRED, pps->weighted_bipred_idc) |
	      FIELD_PREP(AVC_PPS0_PIC_INIT_QP, pps->pic_init_qp_minus26 & 0x3f) |
	      FIELD_PREP(AVC_PPS0_CHROMA_QP_OFFSET, pps->chroma_qp_index_offset & 0x1f) |
	      FIELD_PREP(AVC_PPS0_CHROMA_QP_OFFSET2,
			 pps->second_chroma_qp_index_offset & 0x1f);
	if (pps->flags & V4L2_H264_PPS_FLAG_ENTROPY_CODING_MODE)
		val |= AVC_PPS0_CABAC;
	if (pps->flags & V4L2_H264_PPS_FLAG_BOTTOM_FIELD_PIC_ORDER_IN_FRAME_PRESENT)
		val |= AVC_PPS0_BOTTOM_POC;
	if (pps->flags & V4L2_H264_PPS_FLAG_WEIGHTED_PRED)
		val |= AVC_PPS0_WEIGHTED_PRED;
	if (pps->flags & V4L2_H264_PPS_FLAG_DEBLOCKING_FILTER_CONTROL_PRESENT)
		val |= AVC_PPS0_DEBLOCKING_CTRL;
	if (pps->flags & V4L2_H264_PPS_FLAG_CONSTRAINED_INTRA_PRED)
		val |= AVC_PPS0_CONSTR_INTRA;
	if (pps->flags & V4L2_H264_PPS_FLAG_TRANSFORM_8X8_MODE)
		val |= AVC_PPS0_TRANSFORM_8X8;
	vdec_write(inst, VDEC_AD, AVC_PPS0, val);
	vdec_write(inst, VDEC_AD, AVC_PPS1,
		   FIELD_PREP(AVC_PPS1_NUM_REF_L0, pps->num_ref_idx_l0_default_active_minus1) |
		   FIELD_PREP(AVC_PPS1_NUM_REF_L1, pps->num_ref_idx_l1_default_active_minus1) |
		   scl_bits);

	vdec_read(inst, VDEC_AD, AVC_SHDR0);
	vdec_write(inst, VDEC_AD, AVC_SHDR0, hdr->first_mb);
	vdec_read(inst, VDEC_AD, AVC_SHDR1);
	val = FIELD_PREP(AVC_SHDR1_SLICE_TYPE, hdr->slice_type) |
	      FIELD_PREP(AVC_SHDR1_NUM_REF_L0, hdr->num_ref_l0);
	if (hdr->field_pic)
		val |= AVC_SHDR1_FIELD_PIC;
	if (hdr->bottom)
		val |= AVC_SHDR1_BOTTOM_FIELD;
	if (hdr->type == SLICE_B) {
		val |= FIELD_PREP(AVC_SHDR1_NUM_REF_L1, hdr->num_ref_l1);
		if (hdr->direct_spatial)
			val |= AVC_SHDR1_DIRECT_SPATIAL;
	}
	vdec_write(inst, VDEC_AD, AVC_SHDR1, val);
}

/* let the decoder read ref_pic_list_modification() and pred_weight_table() */
static int vdec_h264_hw_hdr(struct vdec_h264_direct_inst *inst)
{
	const struct v4l2_ctrl_h264_pps *pps = inst->pps;
	struct vdec_h264_direct_hdr *hdr = &inst->hdr;
	u32 val;
	int ret;

	if (hdr->type != SLICE_I) {
		vdec_write(inst, VDEC_AD, AVC_REORDER, 1);
		ret = vdec_poll(inst, VDEC_AD, AVC_REORDER, ~0, true, "list modification");
		if (ret)
			return ret;
	}

	if (((pps->flags & V4L2_H264_PPS_FLAG_WEIGHTED_PRED) && hdr->type == SLICE_P) ||
	    (pps->weighted_bipred_idc == 1 && hdr->type == SLICE_B)) {
		val = vdec_read(inst, VDEC_MISC, MISC_PRED_WEIGHT_CFG);
		vdec_write(inst, VDEC_AD, AVC_PRED_WEIGHT, 1);
		ret = vdec_poll(inst, VDEC_AD, AVC_PRED_WEIGHT, ~0, true, "weight table");
		if (ret)
			return ret;
		vdec_write(inst, VDEC_AD, AVC_PRED_WEIGHT_CFG, val & ~AVC_PRED_WEIGHT_CFG_EN);
	}

	return 0;
}

static int vdec_h264_start(struct vdec_h264_direct_inst *inst,
			   const struct vdec_h264_direct_ref *dst, dma_addr_t c_addr)
{
	struct vdec_h264_direct_hdr *hdr = &inst->hdr;
	unsigned int width = inst->mbw * MB_UNIT_LEN;
	unsigned int height = inst->mbh * MB_UNIT_LEN;
	unsigned int luma = vdec_h264_luma_size(inst);
	dma_addr_t start = min(dst->y, c_addr);
	dma_addr_t end = max(dst->y + luma, c_addr + luma / 2);
	dma_addr_t mv_lo = ~(dma_addr_t)0, mv_hi = 0;
	unsigned int i;
	int ret;

	for (i = 0; i < VDEC_H264_MAX_FB; i++) {
		if (!inst->mv_buf[i].va)
			continue;
		mv_lo = min(mv_lo, inst->mv_buf[i].dma_addr);
		mv_hi = max(mv_hi, inst->mv_buf[i].dma_addr + inst->mv_buf[i].size);
	}

	if (vdec_h264_mv_reduced(inst))
		vdec_mask(inst, VDEC_AV, MV_CFG_220, 0, MV_CFG_220_REDUCED);
	else
		vdec_mask(inst, VDEC_AV, MV_CFG_220, MV_CFG_220_REDUCED, 0);
	vdec_write(inst, VDEC_MISC, MISC_CFG_004, MISC_CFG_004_LOAD);
	vdec_write(inst, VDEC_CM, MC_CFG_090, 4);
	vdec_write(inst, VDEC_TOP, TOP_CFG_058, 0xffffffff);
	vdec_write(inst, VDEC_CM, MC_PIC_W, width);
	vdec_write(inst, VDEC_CM, MC_PIC_H, height);
	vdec_write(inst, VDEC_MISC, MISC_CFG_004, MISC_CFG_004_IDLE);
	vdec_write(inst, VDEC_TOP, TOP_PIC_PIX_SIZE, height << 16 | width);
	vdec_write(inst, VDEC_AD, AVC_CFG_280, AVC_ERR_MASK);
	vdec_write(inst, VDEC_AD, AVC_CFG_26C, AVC_ERR_MASK);
	vdec_write(inst, VDEC_CM, MC_CFG_220, 1);
	vdec_write(inst, VDEC_CM, MC_Y_ADDR_512, dst->y >> 9);
	vdec_write(inst, VDEC_CM, MC_C_ADDR_256, c_addr >> 8);
	vdec_write(inst, VDEC_CM, MC_CFG_250, 1);
	vdec_write(inst, VDEC_CM, MC_CFG_238, 3);
	vdec_write(inst, VDEC_CM, MC_PIC_W_MBS_M1, inst->mbw - 1);
	vdec_write(inst, VDEC_CM, MC_PIC_H_MBS_M1, inst->mbh_pic - 1);
	vdec_write(inst, VDEC_CM, MC_PIC_W_MBS, inst->mbw);
	vdec_write(inst, VDEC_CM, MC_PIC_END, end >> 4);
	vdec_write(inst, VDEC_CM, MC_PIC_START, start >> 4);
	vdec_write(inst, VDEC_CM, MC_CFG_858, 1);
	vdec_write(inst, VDEC_CM, MC_CFG_5E4, MC_CFG_5E4_INIT);
	vdec_write(inst, VDEC_CM, MC_Y_ADDR, dst->y);
	vdec_write(inst, VDEC_CM, MC_C_OFFSET, (u32)(c_addr - dst->y));
	vdec_write(inst, VDEC_CM, MC_PIC_W_MBS2, inst->mbw);
	/*
	 * Take the chroma of each reference from its own address in the
	 * lists. Otherwise the decoder finds it at the luma address of the
	 * reference plus MC_C_OFFSET of the target, which holds only when all
	 * capture buffers have the same distance between their planes.
	 */
	vdec_mask(inst, VDEC_CM, MC_YC_SEP, MC_YC_SEP_UFO, MC_YC_SEP_EN);
	vdec_write(inst, VDEC_AD, AVC_SC_CFG, AVC_SC_CFG_VAL);
	vdec_write(inst, VDEC_AD, AVC_CFG_2D0, AVC_CFG_2D0_INIT);
	vdec_write(inst, VDEC_AD, AVC_CFG_2B4, 0xffffffff);
	vdec_write(inst, VDEC_AD, AVC_CFG_2B8, 0xffffffff);
	vdec_write(inst, VDEC_AV, MV_DST_ADDR, vdec_h264_mv_addr(inst, dst->mv, inst->bottom));
	vdec_write(inst, VDEC_AD, AVC_SHDR2,
		   FIELD_PREP(AVC_SHDR2_QP_DELTA, hdr->qp_delta & 0x7f) |
		   FIELD_PREP(AVC_SHDR2_DEBLOCK_IDC, hdr->deblock_idc) |
		   FIELD_PREP(AVC_SHDR2_ALPHA, hdr->alpha & 0xf) |
		   FIELD_PREP(AVC_SHDR2_BETA, hdr->beta & 0xf) |
		   FIELD_PREP(AVC_SHDR2_NAL_REF_IDC, hdr->nal_ref_idc) |
		   (hdr->idr ? AVC_SHDR2_IDR : 0) |
		   FIELD_PREP(AVC_SHDR2_CABAC_INIT_IDC, hdr->cabac_init_idc));
	if (inst->pps->flags & V4L2_H264_PPS_FLAG_ENTROPY_CODING_MODE) {
		vdec_write(inst, VDEC_AD, AVC_CABAC_INIT, 1);
		ret = vdec_poll(inst, VDEC_AD, AVC_BUSY, AVC_BUSY_CABAC_INIT, false,
				"CABAC init");
		if (ret)
			return ret;
	}
	vdec_mask(inst, VDEC_CM, MC_CFG_5E4, 0, MC_CFG_5E4_RUN);
	vdec_mask(inst, VDEC_CM, MC_CFG_920, 0, MC_CFG_920_RUN);
	vdec_mask(inst, VDEC_CM, MC_CFG_768, MC_CFG_768_MASK, MC_CFG_768_VAL);
	vdec_write(inst, VDEC_TOP, TOP_PIC_MBS, inst->mbw * inst->mbh_pic);
	vdec_write(inst, VDEC_TOP, TOP_CFG_050, 0xffffffff);
	vdec_write(inst, VDEC_CM, MC_VLD_WRAP_ADDR, inst->vld_wrap.dma_addr);
	vdec_write(inst, VDEC_CM, MC_PP_WRAP_ADDR, inst->pp_wrap.dma_addr);
	vdec_write(inst, VDEC_AD, AVC_CFG_2D0, AVC_CFG_2D0_RUN);
	vdec_write(inst, VDEC_AD, AVC_CFG_2D4, AVC_CFG_2D4_VAL);
	vdec_read(inst, VDEC_AD, AVC_GETBITS);
	vdec_mask(inst, VDEC_TOP, TOP_CFG_09C, TOP_CFG_09C_MASK, TOP_CFG_09C_VAL);
	if (width * height > VDEC_H264_LARGE_PIC) {
		vdec_write(inst, VDEC_MISC, MISC_SIZE_CFG, MISC_SIZE_CFG_LARGE);
		vdec_mask(inst, VDEC_PP, PP_CFG_B80, 0, PP_CFG_B80_LARGE);
	} else {
		vdec_mask(inst, VDEC_PP, PP_CFG_B80, PP_CFG_B80_LARGE, 0);
	}
	vdec_mask(inst, VDEC_MISC, MISC_CFG_028, 0, MISC_CFG_028_EN);
	vdec_write(inst, VDEC_MISC, MISC_PIC_RANGE_START, dst->y);
	vdec_write(inst, VDEC_MISC, MISC_PIC_RANGE_END, dst->y + ALIGN(luma * 3 / 4, 512));
	vdec_write(inst, VDEC_MISC, MISC_MV_RANGE_START, mv_lo);
	vdec_write(inst, VDEC_MISC, MISC_MV_RANGE_END, mv_hi);
	vdec_write(inst, VDEC_AD, AVC_READER_CFG, AVC_READER_CFG_RUN);
	vdec_write(inst, VDEC_AD, AVC_DEC_START, 1);

	return 0;
}

static int vdec_h264_run(struct vdec_h264_direct_inst *inst, struct mtk_vcodec_mem *bs,
			 const struct vdec_h264_direct_ref *dst, dma_addr_t c_addr)
{
	dma_addr_t end;
	u32 scl_bits;
	int ret;

	ret = vdec_h264_bs_end(inst, bs, &end);
	if (ret)
		return ret;

	vdec_h264_reset(inst);
	ret = vdec_h264_init_reader(inst, bs->dma_addr, end);
	if (ret)
		return ret;
	ret = vdec_h264_find_slice(inst);
	if (ret)
		return ret;
	if (!!inst->hdr.idr != !!(inst->dec->flags & V4L2_H264_DECODE_PARAM_FLAG_IDR_PIC))
		mtk_vdec_debug(inst->ctx, "IDR flag differs from the bitstream");
	ret = vdec_h264_read_hdr_start(inst);
	if (ret)
		return ret;

	vdec_h264_write_lists(inst, dst);
	scl_bits = vdec_h264_write_scaling(inst);
	vdec_h264_write_pic_params(inst, scl_bits);

	ret = vdec_h264_hw_hdr(inst);
	if (ret)
		return ret;
	ret = vdec_h264_read_hdr_end(inst);
	if (ret)
		return ret;

	return vdec_h264_start(inst, dst, c_addr);
}

/* ---------------- codec interface ---------------- */

/*
 * 4:0:0 needs the chroma plane filled by the host. Write it through the
 * kernel mapping of a capture buffer that the driver allocated coherent, so
 * that no cache maintenance is needed.
 */
static int vdec_h264_check_mono(struct vdec_h264_direct_inst *inst,
				struct mtk_video_dec_buf *dst_buf)
{
	struct vb2_queue *q = dst_buf->m2m_buf.vb.vb2_buf.vb2_queue;

	if (!inst->mono)
		return 0;
	if (q->memory != VB2_MEMORY_MMAP || q->non_coherent_mem ||
	    !dst_buf->frame_buffer.base_c.va) {
		mtk_vdec_err(inst->ctx, "4:0:0 needs coherent MMAP capture buffers");
		return -EOPNOTSUPP;
	}

	return 0;
}

static int vdec_h264_direct_decode(void *h_vdec, struct mtk_vcodec_mem *bs,
				   struct vdec_fb *unused, bool *res_chg)
{
	struct vdec_h264_direct_inst *inst = h_vdec;
	struct mtk_vcodec_dec_ctx *ctx = inst->ctx;
	struct mtk_video_dec_buf *src_buf, *dst_buf;
	struct vdec_h264_direct_ref dst = {};
	struct vdec_fb *fb;
	unsigned int idx, luma;
	u32 mb_x, mb_y, err;
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

	ret = vdec_h264_get_params(inst);
	if (ret)
		return ret;

	luma = vdec_h264_luma_size(inst);
	if (inst->mbw * MB_UNIT_LEN != ctx->picinfo.buf_w ||
	    fb->base_y.size < luma || fb->base_c.size < luma / 2) {
		mtk_vdec_err(ctx, "picture %ux%u does not fit the %ux%u buffers",
			     inst->mbw * MB_UNIT_LEN, inst->mbh * MB_UNIT_LEN,
			     ctx->picinfo.buf_w, ctx->picinfo.buf_h);
		return -EINVAL;
	}

	ret = vdec_h264_check_mono(inst, dst_buf);
	if (ret)
		return ret;

	idx = dst_buf->m2m_buf.vb.vb2_buf.index;
	if (idx >= VDEC_H264_MAX_FB)
		return -EINVAL;
	ret = vdec_h264_mv_buf(inst, idx);
	if (ret)
		return ret;

	dst.y = fb->base_y.dma_addr;
	dst.c = fb->base_c.dma_addr;
	dst.mv = inst->mv_buf[idx].dma_addr;
	ret = vdec_h264_pick_slot(inst, idx);
	if (ret < 0)
		return ret;
	dst.slot = ret;
	inst->slot[idx] = dst.slot;
	if (inst->field)
		dst.coding = MV_REF_CODED_FIELD;
	else if (inst->sps->flags & V4L2_H264_SPS_FLAG_MB_ADAPTIVE_FRAME_FIELD)
		dst.coding = MV_REF_CODED_MBAFF;
	else
		dst.coding = 0;
	inst->coding[idx] = dst.coding;
	vdec_h264_build_lists(inst, &dst);

	ret = vdec_h264_run(inst, bs, &dst, fb->base_c.dma_addr);
	if (ret)
		goto err_break;

	ret = mtk_vcodec_wait_for_done_ctx(ctx, MTK_INST_IRQ_RECEIVED,
					   VDEC_H264_TIMEOUT_MS, MTK_VDEC_CORE);
	if (ret) {
		ret = -ETIMEDOUT;
		goto err_break;
	}

	mb_x = vdec_read(inst, VDEC_CM, MC_MB_X);
	mb_y = vdec_read(inst, VDEC_CM, MC_MB_Y);
	err = vdec_read(inst, VDEC_AD, AVC_ERR_STA);
	ctx->decoded_frame_cnt++;
	if (inst->mono)
		memset(fb->base_c.va, VDEC_H264_CHROMA_MID, fb->base_c.size);
	if (mb_x != inst->mbw - 1 || mb_y != inst->mbh_pic - 1) {
		mtk_vdec_err(ctx, "picture incomplete: last MB %u,%u, error %#x",
			     mb_x, mb_y, err);
		return -EBADMSG;
	}
	/* the decoder conceals a damaged slice and runs to the last MB */
	if (err & AVC_ERR_MASK) {
		mtk_vdec_err(ctx, "bitstream error %#x in picture %d", err,
			     ctx->decoded_frame_cnt - 1);
		return -EBADMSG;
	}
	mtk_vdec_debug(ctx, "picture %d done, error status %#x",
		       ctx->decoded_frame_cnt - 1, err);

	return 0;

err_break:
	mtk_vdec_direct_hw_break(inst->dev);
	return ret;
}

static void vdec_h264_free(struct vdec_h264_direct_inst *inst)
{
	unsigned int i;

	for (i = 0; i < VDEC_H264_MAX_FB; i++)
		if (inst->mv_buf[i].va)
			mtk_vcodec_mem_free(inst->ctx, &inst->mv_buf[i]);
	if (inst->pp_wrap.va)
		mtk_vcodec_mem_free(inst->ctx, &inst->pp_wrap);
	if (inst->vld_wrap.va)
		mtk_vcodec_mem_free(inst->ctx, &inst->vld_wrap);
	if (inst->pred_buf.va)
		mtk_vcodec_mem_free(inst->ctx, &inst->pred_buf);
}

static int vdec_h264_direct_init(struct mtk_vcodec_dec_ctx *ctx)
{
	struct vdec_h264_direct_inst *inst;
	int ret;

	inst = kzalloc_obj(*inst);
	if (!inst)
		return -ENOMEM;
	inst->ctx = ctx;
	inst->dev = ctx->dev;

	inst->pred_buf.size = VDEC_H264_PRED_SZ;
	inst->vld_wrap.size = VDEC_H264_VLD_WRAP_SZ;
	inst->pp_wrap.size = VDEC_H264_PP_WRAP_SZ;
	ret = mtk_vcodec_mem_alloc(ctx, &inst->pred_buf);
	if (!ret)
		ret = mtk_vcodec_mem_alloc(ctx, &inst->vld_wrap);
	if (!ret)
		ret = mtk_vcodec_mem_alloc(ctx, &inst->pp_wrap);
	if (ret) {
		vdec_h264_free(inst);
		kfree(inst);
		return ret;
	}

	ctx->drv_handle = inst;
	return 0;
}

static void vdec_h264_direct_deinit(void *h_vdec)
{
	struct vdec_h264_direct_inst *inst = h_vdec;

	vdec_h264_free(inst);
	kfree(inst);
}

static int vdec_h264_direct_get_param(void *h_vdec, enum vdec_get_param_type type,
				      void *out)
{
	struct vdec_h264_direct_inst *inst = h_vdec;
	struct vdec_pic_info *pic = &inst->ctx->picinfo;
	struct v4l2_rect *crop;

	switch (type) {
	case GET_PARAM_PIC_INFO:
		/* MM21, MT2110: the tile rows of the decoder follow the width */
		pic->buf_w = ALIGN(pic->pic_w, MB_UNIT_LEN);
		pic->buf_h = ALIGN(pic->pic_h, 32);
		pic->fb_sz[0] = vdec_h264_plane_size(pic->buf_w, pic->buf_h,
						     inst->ctx->is_10bit_bitstream);
		pic->fb_sz[1] = pic->fb_sz[0] / 2;
		*(struct vdec_pic_info *)out = *pic;
		break;
	case GET_PARAM_DPB_SIZE:
		*(unsigned int *)out = V4L2_H264_NUM_DPB_ENTRIES + 1;
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

const struct vdec_common_if vdec_h264_slice_direct_if = {
	.init		= vdec_h264_direct_init,
	.decode		= vdec_h264_direct_decode,
	.get_param	= vdec_h264_direct_get_param,
	.deinit		= vdec_h264_direct_deinit,
};
