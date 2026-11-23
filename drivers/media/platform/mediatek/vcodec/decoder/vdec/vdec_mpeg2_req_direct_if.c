// SPDX-License-Identifier: GPL-2.0
/*
 * MPEG-2 decoding on MediaTek decoders that the host programs through their
 * registers, without firmware (MT6785).
 *
 * One hardware run decodes one picture (a frame or a field) with all of its
 * slices: the host points the bitstream reader at the first slice start
 * code, programs the picture coding parameters and quantiser matrices from
 * the V4L2 controls and the addresses of the target and reference pictures,
 * and the decoder parses the slices itself.
 *
 * The decoder has two anchor picture slots and a separate target for B
 * pictures. The forward reference always goes to slot 0; a P picture and
 * the second field of a frame are decoded into slot 1, which the decoder
 * then also uses for the first field of the same frame. Both fields of a
 * frame go to the same capture buffer, which user space keeps with
 * V4L2_BUF_FLAG_M2M_HOLD_CAPTURE_BUF between the two requests.
 *
 * The inverse DCT follows IEEE 1180 like the standard requires, but is not
 * bit-exact to other decoders; the output matches the vendor driver on the
 * same hardware.
 *
 * Output is MM21 (luma in 16x32 tiles, chroma in 16x16 tiles).
 */

#include <linux/bitfield.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-dma-contig.h>

#include "../mtk_vcodec_dec.h"
#include "../../common/mtk_vcodec_intr.h"
#include "../../common/mtk_vcodec_util.h"
#include "../vdec_drv_base.h"
#include "../vdec_drv_if.h"

/* bitstream reader and MPEG syntax decoder (VDEC_LD) */
#define VLD_PIC_CFG			0x088
#define VLD_PIC_STRUCT			GENMASK(31, 30)
#define VLD_PIC_FRAME_PRED_DCT		BIT(29)
#define VLD_PIC_CONCEALMENT_MV		BIT(28)
#define VLD_PIC_Q_SCALE_TYPE		BIT(27)
#define VLD_PIC_TOP_FIELD_FIRST		BIT(26)
#define VLD_PIC_CODING_TYPE		GENMASK(23, 21)
#define VLD_PIC_INTRA_VLC		BIT(20)
#define VLD_PIC_DC_PRECISION		GENMASK(19, 18)
#define VLD_PIC_ALT_SCAN		BIT(17)
#define VLD_PIC_F_CODE_FWD_H		GENMASK(15, 12)
#define VLD_PIC_F_CODE_FWD_V		GENMASK(11, 8)
#define VLD_PIC_F_CODE_BWD_H		GENMASK(7, 4)
#define VLD_PIC_F_CODE_BWD_V		GENMASK(3, 0)
#define VLD_BS_CTRL			0x08c
#define VLD_BS_CTRL_FETCH		BIT(20)
#define VLD_BS_CTRL_RUN			BIT(23)
#define VLD_BS_CTRL_MPEG_START		0x4000ff
#define VLD_SEQ_CFG			0x090
#define VLD_SEQ_MPEG2			BIT(30)
#define VLD_SEQ_HEIGHT			GENMASK(27, 16)
#define VLD_SEQ_CFG_LOW			0x1ff
#define VLD_CFG_094			0x094
#define VLD_CFG_094_VAL			0x1ff
#define VLD_CFG_098			0x098
#define VLD_CFG_098_VAL			0x3ff
#define VLD_CFG_09C			0x09c
#define VLD_CFG_09C_VAL			0x30fff
#define VLD_QMAT_CTRL			0x104
#define VLD_QMAT_INTRA			BIT(8)
#define VLD_QMAT_NON_INTRA		BIT(9)
#define VLD_QMAT_DEFAULT		BIT(10)
#define VLD_RPTR			0x0b0
#define VLD_VSTART			0x0b4
#define VLD_VEND			0x0b8
#define VLD_FETCH_STA			0x0e8
#define VLD_FETCH_STA_READY		BIT(0)
#define VLD_VBAR			0x0ec
#define VLD_VBAR_EN			BIT(28)
#define VLD_START_CODE			0x0f0
#define VLD_PROC_STA			0x0f4
#define VLD_SRST_READER			BIT(8)
#define VLD_RD_ADDR			0x0fc
#define VLD_WPTR			0x110
#define VLD_ERR_STA			0x114
#define VLD_TAB_ADDR			0x260
#define VLD_TAB_ADDR_WRITE		BIT(9)
#define VLD_TAB_ADDR_DONE		BIT(8)
#define VLD_TAB_DATA			0x264

#define QMAT_INTRA_ADDR			0x00
#define QMAT_NON_INTRA_ADDR		0x40

/* VDEC_TOP */
#define TOP_PIC_PIX_SIZE		0x070

/* motion compensation (VDEC_CM) */
#define MC_FWD_Y			0x000
#define MC_FWD_C			0x004
#define MC_BWD_Y			0x008
#define MC_BWD_C			0x00c
#define MC_FWD_SLOT			0x018
#define MC_SECOND_FIELD			0x020
#define MC_DST_SLOT			0x024
#define MC_DST_SLOT_BWD			1
#define MC_DST_SLOT_B			4
#define MC_MB_X				0x028
#define MC_MB_Y				0x02c
#define MC_B_Y				0x09c
#define MC_B_C				0x0a0
#define MC_PIC_W			0x208
#define MC_PIC_H			0x20c
#define MC_CFG_220			0x220
#define MC_OUT_C			0x850
#define MC_OUT_Y			0x854
#define MC_CFG_858			0x858
#define MC_CFG_920			0x920
#define MC_CFG_920_RUN			BIT(24)
#define MC_PIC_W_MBS			0x980
#define MC_CFG_A60			0xa60

/* post-processing (VDEC_PP) */
#define PP_CFG_03C			0x03c
#define PP_CFG_03C_VAL			BIT(28)

/* control (VDEC_MISC) */
#define MISC_CRC_EN			0x004
#define MISC_CRC(n)			(0x008 + 4 * (n))
#define MISC_SYS_SEL			0x084
#define MISC_SPEC_CON			0x0c8
#define MISC_SPEC_CON_VAL		0xfffe
#define MISC_FUNC_CON			0x0cc
#define MISC_FUNC_CON_VAL		0x7f6a11df
#define MISC_IRQ_STA			0x0a4
#define MISC_DCM_CON			0x0ec
#define MISC_SRST			0x0f4
#define MISC_SIZE_CFG			0x160
#define MISC_SIZE_CFG_RESET		0xf1000
#define MISC_CFG_178			0x178
#define MISC_CFG_178_VAL		0xfffffffb
#define MISC_CFG_1F0			0x1f0
#define MISC_CFG_1F0_RESET		0

/* zeros the reader needs behind the bitstream */
#define MPEG2_MB_SIZE			16

#define VDEC_MPEG2_BS_PAD		384
#define VDEC_MPEG2_BS_MIN_PAD		64

#define VDEC_MPEG2_POLL_US		10000
#define VDEC_MPEG2_TIMEOUT_MS		500

#define MPEG2_SLICE_START_MIN		0x101
#define MPEG2_SLICE_START_MAX		0x1af

struct vdec_mpeg2_direct_pic {
	dma_addr_t y;
	dma_addr_t c;
};

/**
 * struct vdec_mpeg2_direct_inst - MPEG-2 decoder instance
 * @ctx:	decoder context
 * @dev:	decoder device
 * @seq:	sequence control of the current request
 * @pic:	picture control of the current request
 * @quant:	quantisation control of the current request
 * @mbw:	width in macroblocks
 * @mbh:	frame height in macroblocks
 * @last_field_idx: capture buffer of the last decoded first field, or -1
 * @last_field_struct: picture structure of that field
 */
struct vdec_mpeg2_direct_inst {
	struct mtk_vcodec_dec_ctx *ctx;
	struct mtk_vcodec_dec_dev *dev;

	const struct v4l2_ctrl_mpeg2_sequence *seq;
	const struct v4l2_ctrl_mpeg2_picture *pic;
	const struct v4l2_ctrl_mpeg2_quantisation *quant;

	unsigned int mbw;
	unsigned int mbh;
	int last_field_idx;
	unsigned int last_field_struct;
};

/* default intra matrix of ISO/IEC 13818-2 in zigzag scan order */
static const u8 mpeg2_default_intra[64] = {
	8, 16, 16, 19, 16, 19, 22, 22, 22, 22, 22, 22, 26, 24, 26, 27,
	27, 27, 26, 26, 26, 26, 27, 27, 27, 29, 29, 29, 34, 34, 34, 29,
	29, 29, 27, 27, 29, 29, 32, 32, 34, 34, 37, 38, 37, 35, 35, 34,
	35, 38, 38, 40, 40, 40, 48, 48, 46, 46, 56, 56, 58, 69, 69, 83,
};

static const u8 mpeg2_seq_end_code[] = { 0x00, 0x00, 0x01, 0xb7 };

static inline void vdec_write(struct vdec_mpeg2_direct_inst *inst, int blk,
			      u32 off, u32 val)
{
	writel(val, inst->dev->reg_base[blk] + off);
}

static inline u32 vdec_read(struct vdec_mpeg2_direct_inst *inst, int blk, u32 off)
{
	return readl(inst->dev->reg_base[blk] + off);
}

static void *vdec_mpeg2_ctrl(struct mtk_vcodec_dec_ctx *ctx, u32 id)
{
	struct v4l2_ctrl *ctrl = v4l2_ctrl_find(&ctx->ctrl_hdl, id);

	if (!ctrl)
		return ERR_PTR(-EINVAL);

	return ctrl->p_cur.p;
}

static int vdec_mpeg2_get_params(struct vdec_mpeg2_direct_inst *inst)
{
	const struct mtk_vdec_direct_data *data = inst->dev->vdec_pdata->direct;
	struct mtk_vcodec_dec_ctx *ctx = inst->ctx;
	const struct v4l2_ctrl_mpeg2_sequence *seq;
	const struct v4l2_ctrl_mpeg2_picture *pic;

	inst->seq = vdec_mpeg2_ctrl(ctx, V4L2_CID_STATELESS_MPEG2_SEQUENCE);
	inst->pic = vdec_mpeg2_ctrl(ctx, V4L2_CID_STATELESS_MPEG2_PICTURE);
	inst->quant = vdec_mpeg2_ctrl(ctx, V4L2_CID_STATELESS_MPEG2_QUANTISATION);
	if (IS_ERR(inst->seq) || IS_ERR(inst->pic) || IS_ERR(inst->quant))
		return -EINVAL;
	seq = inst->seq;
	pic = inst->pic;

	if (seq->chroma_format != 1) {
		mtk_vdec_err(ctx, "only 4:2:0 supported");
		return -EOPNOTSUPP;
	}
	if (pic->picture_coding_type < V4L2_MPEG2_PIC_CODING_TYPE_I ||
	    pic->picture_coding_type > V4L2_MPEG2_PIC_CODING_TYPE_B ||
	    !pic->picture_structure) {
		mtk_vdec_err(ctx, "picture type %u structure %u not supported",
			     pic->picture_coding_type, pic->picture_structure);
		return -EOPNOTSUPP;
	}

	inst->mbw = DIV_ROUND_UP(seq->horizontal_size, MPEG2_MB_SIZE);
	/* an interlaced frame has an even number of macroblock rows */
	if (seq->flags & V4L2_MPEG2_SEQ_FLAG_PROGRESSIVE)
		inst->mbh = DIV_ROUND_UP(seq->vertical_size, MPEG2_MB_SIZE);
	else
		inst->mbh = 2 * DIV_ROUND_UP(seq->vertical_size, 2 * MPEG2_MB_SIZE);
	if (!inst->mbw || !inst->mbh ||
	    inst->mbw * MPEG2_MB_SIZE > data->max_width ||
	    inst->mbh * MPEG2_MB_SIZE > data->max_height) {
		mtk_vdec_err(ctx, "picture %ux%u not supported",
			     seq->horizontal_size, seq->vertical_size);
		return -EINVAL;
	}

	return 0;
}

static unsigned int vdec_mpeg2_luma_size(struct vdec_mpeg2_direct_inst *inst)
{
	return inst->mbw * MPEG2_MB_SIZE * ALIGN(inst->mbh * MPEG2_MB_SIZE, 32);
}

static void vdec_mpeg2_ref(struct vdec_mpeg2_direct_inst *inst, u64 ts,
			   const struct vdec_mpeg2_direct_pic *dst,
			   struct vdec_mpeg2_direct_pic *ref)
{
	struct vb2_queue *vq = v4l2_m2m_get_vq(inst->ctx->m2m_ctx,
					       V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);
	struct vb2_buffer *vb = vb2_find_buffer(vq, ts);

	if (!vb) {
		/* missing reference: predict from the target itself */
		mtk_vdec_debug(inst->ctx, "reference %llu not found", ts);
		*ref = *dst;
		return;
	}
	ref->y = vb2_dma_contig_plane_dma_addr(vb, 0);
	ref->c = vb2_dma_contig_plane_dma_addr(vb, 1);
}

static int vdec_mpeg2_bs_end(struct vdec_mpeg2_direct_inst *inst,
			     struct mtk_vcodec_mem *bs, dma_addr_t *end)
{
	struct mtk_video_dec_buf *src = container_of(bs, struct mtk_video_dec_buf,
						     bs_buffer);
	dma_addr_t limit = bs->dma_addr +
			   vb2_plane_size(&src->m2m_buf.vb.vb2_buf, 0);
	dma_addr_t data_end = bs->dma_addr + bs->size;

	*end = ALIGN(data_end + VDEC_MPEG2_BS_PAD, 64);
	if (*end > limit)
		*end = ALIGN_DOWN(limit, 64);
	if (*end < data_end + VDEC_MPEG2_BS_MIN_PAD) {
		mtk_vdec_err(inst->ctx, "no room behind the bitstream (%zu of %pad)",
			     bs->size, &limit);
		return -EINVAL;
	}
	memset(bs->va + bs->size, 0, *end - data_end);
	/*
	 * The decoder ends the last slice of the picture on the next start
	 * code; place a sequence end code behind the data.
	 */
	memcpy(bs->va + bs->size, mpeg2_seq_end_code, sizeof(mpeg2_seq_end_code));

	return 0;
}

static void vdec_mpeg2_reset(struct vdec_mpeg2_direct_inst *inst)
{
	vdec_write(inst, VDEC_LD, VDEC_VLD_SRST, VLD_SRST_READER | VDEC_VLD_SRST_RESET);
	vdec_write(inst, VDEC_MISC, MISC_SPEC_CON, MISC_SPEC_CON_VAL);
	vdec_write(inst, VDEC_MISC, MISC_FUNC_CON, MISC_FUNC_CON_VAL);
	vdec_write(inst, VDEC_MISC, MISC_CFG_178, MISC_CFG_178_VAL);
	vdec_write(inst, VDEC_MISC, MISC_SRST, 0);
	vdec_write(inst, VDEC_MISC, MISC_SYS_SEL, 1);
	vdec_write(inst, VDEC_LD, VDEC_VLD_SRST, 0);
	mtk_vcodec_write_vdecsys(inst->ctx, VDEC_SYS_DCM_CON, 1);
	vdec_write(inst, VDEC_MISC, MISC_DCM_CON, 1);
	vdec_write(inst, VDEC_MISC, MISC_CRC_EN, 1);
	/* settings that a large picture of another codec may have left */
	vdec_write(inst, VDEC_MISC, MISC_SIZE_CFG, MISC_SIZE_CFG_RESET);
	vdec_write(inst, VDEC_MISC, MISC_CFG_1F0, MISC_CFG_1F0_RESET);
}

static int vdec_mpeg2_init_reader(struct vdec_mpeg2_direct_inst *inst,
				  struct mtk_vcodec_mem *bs, dma_addr_t end)
{
	void __iomem *sta = inst->dev->reg_base[VDEC_LD] + VLD_FETCH_STA;
	u32 val;
	int ret;

	vdec_read(inst, VDEC_LD, VLD_PROC_STA);
	vdec_write(inst, VDEC_LD, VLD_VSTART, bs->dma_addr >> 6);
	vdec_write(inst, VDEC_LD, VLD_VEND, end >> 6);
	vdec_write(inst, VDEC_LD, VLD_RPTR, bs->dma_addr);
	vdec_write(inst, VDEC_LD, VLD_WPTR, end);
	vdec_write(inst, VDEC_LD, VDEC_VLD_SRST, VLD_SRST_READER);
	vdec_write(inst, VDEC_LD, VDEC_VLD_SRST, 0);
	vdec_write(inst, VDEC_LD, VLD_CFG_09C, VLD_CFG_09C_VAL);
	vdec_write(inst, VDEC_LD, VLD_VBAR,
		   vdec_read(inst, VDEC_LD, VLD_VBAR) | VLD_VBAR_EN);
	vdec_write(inst, VDEC_LD, VLD_BS_CTRL, VLD_BS_CTRL_FETCH);
	ret = readl_poll_timeout(sta, val, val & VLD_FETCH_STA_READY, 1,
				 VDEC_MPEG2_POLL_US);
	if (ret) {
		mtk_vdec_err(inst->ctx, "bitstream fetch timeout: %#x", val);
		return ret;
	}
	vdec_write(inst, VDEC_LD, VLD_BS_CTRL, VLD_BS_CTRL_RUN);

	/* the reader stops at the start code in front of it */
	val = vdec_read(inst, VDEC_LD, VLD_START_CODE);
	if (val < MPEG2_SLICE_START_MIN || val > MPEG2_SLICE_START_MAX) {
		mtk_vdec_err(inst->ctx, "no slice start code: %#x", val);
		return -EINVAL;
	}

	return 0;
}

static void vdec_mpeg2_write_qmat(struct vdec_mpeg2_direct_inst *inst,
				  const u8 *mat, u32 base)
{
	unsigned int i;

	/* four coefficients per word in zigzag order, the first one on top */
	for (i = 0; i < 64; i += 4) {
		vdec_write(inst, VDEC_LD, VLD_TAB_ADDR, VLD_TAB_ADDR_WRITE | (base + i));
		vdec_write(inst, VDEC_LD, VLD_TAB_DATA,
			   mat[i] << 24 | mat[i + 1] << 16 | mat[i + 2] << 8 | mat[i + 3]);
		vdec_write(inst, VDEC_LD, VLD_TAB_ADDR, VLD_TAB_ADDR_DONE | (base + i));
	}
}

static bool vdec_mpeg2_flat16(const u8 *mat)
{
	unsigned int i;

	for (i = 0; i < 64; i++)
		if (mat[i] != 16)
			return false;

	return true;
}

/*
 * Matrices that differ from the defaults are loaded into the table memory of
 * the decoder; without any, the decoder uses its built-in defaults.
 */
static void vdec_mpeg2_write_quant(struct vdec_mpeg2_direct_inst *inst)
{
	const struct v4l2_ctrl_mpeg2_quantisation *q = inst->quant;
	bool intra = memcmp(q->intra_quantiser_matrix, mpeg2_default_intra, 64);
	bool non_intra = !vdec_mpeg2_flat16(q->non_intra_quantiser_matrix);

	vdec_write(inst, VDEC_LD, VLD_QMAT_CTRL, 0);
	if (!intra && !non_intra) {
		vdec_write(inst, VDEC_LD, VLD_QMAT_CTRL, VLD_QMAT_DEFAULT);
		return;
	}
	vdec_write(inst, VDEC_LD, VLD_QMAT_CTRL,
		   (intra ? VLD_QMAT_INTRA : 0) | (non_intra ? VLD_QMAT_NON_INTRA : 0));
	if (intra) {
		vdec_mpeg2_write_qmat(inst, q->intra_quantiser_matrix, QMAT_INTRA_ADDR);
		vdec_write(inst, VDEC_LD, VLD_QMAT_CTRL, VLD_QMAT_INTRA);
	}
	if (non_intra) {
		vdec_mpeg2_write_qmat(inst, q->non_intra_quantiser_matrix,
				      QMAT_NON_INTRA_ADDR);
		vdec_write(inst, VDEC_LD, VLD_QMAT_CTRL, VLD_QMAT_NON_INTRA);
	}
}

static void vdec_mpeg2_start(struct vdec_mpeg2_direct_inst *inst,
			     const struct vdec_mpeg2_direct_pic *dst,
			     const struct vdec_mpeg2_direct_pic *fwd,
			     const struct vdec_mpeg2_direct_pic *bwd,
			     bool second_field)
{
	const struct v4l2_ctrl_mpeg2_picture *pic = inst->pic;
	unsigned int width = inst->mbw * MPEG2_MB_SIZE;
	unsigned int height = inst->mbh * MPEG2_MB_SIZE;
	unsigned int pic_height = height;
	const struct vdec_mpeg2_direct_pic *slot0, *slot1;
	u32 cfg, dst_slot;

	if (pic->picture_structure != V4L2_MPEG2_PIC_FRAME)
		pic_height /= 2;

	cfg = FIELD_PREP(VLD_PIC_STRUCT, pic->picture_structure) |
	      FIELD_PREP(VLD_PIC_CODING_TYPE, pic->picture_coding_type) |
	      FIELD_PREP(VLD_PIC_DC_PRECISION, pic->intra_dc_precision) |
	      FIELD_PREP(VLD_PIC_F_CODE_FWD_H, pic->f_code[0][0]) |
	      FIELD_PREP(VLD_PIC_F_CODE_FWD_V, pic->f_code[0][1]) |
	      FIELD_PREP(VLD_PIC_F_CODE_BWD_H, pic->f_code[1][0]) |
	      FIELD_PREP(VLD_PIC_F_CODE_BWD_V, pic->f_code[1][1]);
	if (pic->flags & V4L2_MPEG2_PIC_FLAG_FRAME_PRED_DCT)
		cfg |= VLD_PIC_FRAME_PRED_DCT;
	if (pic->flags & V4L2_MPEG2_PIC_FLAG_CONCEALMENT_MV)
		cfg |= VLD_PIC_CONCEALMENT_MV;
	if (pic->flags & V4L2_MPEG2_PIC_FLAG_Q_SCALE_TYPE)
		cfg |= VLD_PIC_Q_SCALE_TYPE;
	if (pic->flags & V4L2_MPEG2_PIC_FLAG_TOP_FIELD_FIRST)
		cfg |= VLD_PIC_TOP_FIELD_FIRST;
	if (pic->flags & V4L2_MPEG2_PIC_FLAG_INTRA_VLC)
		cfg |= VLD_PIC_INTRA_VLC;
	if (pic->flags & V4L2_MPEG2_PIC_FLAG_ALT_SCAN)
		cfg |= VLD_PIC_ALT_SCAN;

	vdec_write(inst, VDEC_LD, VLD_PIC_CFG, cfg);
	vdec_write(inst, VDEC_LD, VLD_SEQ_CFG, VLD_SEQ_MPEG2 |
		   FIELD_PREP(VLD_SEQ_HEIGHT, height) | VLD_SEQ_CFG_LOW);
	vdec_write(inst, VDEC_TOP, TOP_PIC_PIX_SIZE, pic_height << 16 | width);
	vdec_write(inst, VDEC_CM, MC_PIC_W_MBS, inst->mbw);
	vdec_mpeg2_write_quant(inst);
	vdec_write(inst, VDEC_LD, VLD_CFG_094, VLD_CFG_094_VAL);
	vdec_write(inst, VDEC_LD, VLD_CFG_098, VLD_CFG_098_VAL);
	vdec_write(inst, VDEC_CM, MC_CFG_220, 0);
	vdec_write(inst, VDEC_CM, MC_CFG_A60, 0);

	if (pic->picture_coding_type == V4L2_MPEG2_PIC_CODING_TYPE_B) {
		slot0 = fwd;
		slot1 = bwd;
		dst_slot = MC_DST_SLOT_B;
	} else if (pic->picture_coding_type == V4L2_MPEG2_PIC_CODING_TYPE_P ||
		   second_field) {
		slot0 = pic->picture_coding_type == V4L2_MPEG2_PIC_CODING_TYPE_P ?
			fwd : dst;
		slot1 = dst;
		dst_slot = MC_DST_SLOT_BWD;
	} else {
		slot0 = dst;
		slot1 = dst;
		dst_slot = 0;
	}
	vdec_write(inst, VDEC_CM, MC_FWD_Y, slot0->y >> 9);
	vdec_write(inst, VDEC_CM, MC_FWD_C, slot0->c >> 8);
	vdec_write(inst, VDEC_CM, MC_BWD_Y, slot1->y >> 9);
	vdec_write(inst, VDEC_CM, MC_BWD_C, slot1->c >> 8);
	vdec_write(inst, VDEC_CM, MC_FWD_SLOT, 0);
	if (pic->picture_coding_type == V4L2_MPEG2_PIC_CODING_TYPE_B) {
		vdec_write(inst, VDEC_CM, MC_B_Y, dst->y >> 9);
		vdec_write(inst, VDEC_CM, MC_B_C, dst->c >> 8);
	}
	vdec_write(inst, VDEC_CM, MC_CFG_920,
		   vdec_read(inst, VDEC_CM, MC_CFG_920) | MC_CFG_920_RUN);
	vdec_write(inst, VDEC_PP, PP_CFG_03C, PP_CFG_03C_VAL);
	vdec_write(inst, VDEC_CM, MC_OUT_C, dst->c >> 4);
	vdec_write(inst, VDEC_CM, MC_OUT_Y, dst->y >> 4);
	vdec_write(inst, VDEC_CM, MC_CFG_858, 1);
	vdec_write(inst, VDEC_CM, MC_SECOND_FIELD,
		   second_field && pic->picture_coding_type != V4L2_MPEG2_PIC_CODING_TYPE_B);
	vdec_write(inst, VDEC_CM, MC_DST_SLOT, dst_slot);
	vdec_write(inst, VDEC_CM, MC_PIC_W, width);
	vdec_write(inst, VDEC_CM, MC_PIC_H, height);
	vdec_write(inst, VDEC_LD, VLD_BS_CTRL, VLD_BS_CTRL_MPEG_START);
}

static int vdec_mpeg2_direct_decode(void *h_vdec, struct mtk_vcodec_mem *bs,
				    struct vdec_fb *unused, bool *res_chg)
{
	struct vdec_mpeg2_direct_inst *inst = h_vdec;
	struct mtk_vcodec_dec_ctx *ctx = inst->ctx;
	struct mtk_video_dec_buf *src_buf, *dst_buf;
	struct vdec_mpeg2_direct_pic dst, fwd, bwd;
	const struct v4l2_ctrl_mpeg2_picture *pic;
	unsigned int idx, luma, mbh;
	bool second_field = false;
	struct vdec_fb *fb;
	dma_addr_t end;
	u32 mb_x, mb_y, crc[8];
	unsigned int i;
	int ret;

	*res_chg = false;
	if (!bs)
		return 0;

	fb = inst->dev->vdec_pdata->get_cap_buffer(ctx);
	if (!fb)
		return -ENOMEM;
	src_buf = container_of(bs, struct mtk_video_dec_buf, bs_buffer);
	dst_buf = container_of(fb, struct mtk_video_dec_buf, frame_buffer);
	v4l2_m2m_buf_copy_metadata(&src_buf->m2m_buf.vb, &dst_buf->m2m_buf.vb);

	ret = vdec_mpeg2_get_params(inst);
	if (ret)
		return ret;
	pic = inst->pic;

	luma = vdec_mpeg2_luma_size(inst);
	if (inst->mbw * MPEG2_MB_SIZE != ctx->picinfo.buf_w ||
	    fb->base_y.size < luma || fb->base_c.size < luma / 2) {
		mtk_vdec_err(ctx, "picture %ux%u does not fit the %ux%u buffers",
			     inst->mbw * MPEG2_MB_SIZE, inst->mbh * MPEG2_MB_SIZE,
			     ctx->picinfo.buf_w, ctx->picinfo.buf_h);
		return -EINVAL;
	}

	/*
	 * The second field of a frame goes to the capture buffer of the first
	 * one; top_field_first carries no order for field pictures.
	 */
	idx = dst_buf->m2m_buf.vb.vb2_buf.index;
	if (pic->picture_structure != V4L2_MPEG2_PIC_FRAME) {
		second_field = inst->last_field_idx == idx &&
			       inst->last_field_struct != pic->picture_structure;
		inst->last_field_idx = second_field ? -1 : idx;
		inst->last_field_struct = pic->picture_structure;
	} else {
		inst->last_field_idx = -1;
	}

	dst.y = fb->base_y.dma_addr;
	dst.c = fb->base_c.dma_addr;
	vdec_mpeg2_ref(inst, pic->forward_ref_ts, &dst, &fwd);
	vdec_mpeg2_ref(inst, pic->backward_ref_ts, &dst, &bwd);

	ret = vdec_mpeg2_bs_end(inst, bs, &end);
	if (ret)
		return ret;
	vdec_mpeg2_reset(inst);
	ret = vdec_mpeg2_init_reader(inst, bs, end);
	if (ret)
		goto err_break;
	vdec_mpeg2_start(inst, &dst, &fwd, &bwd, second_field);

	ret = mtk_vcodec_wait_for_done_ctx(ctx, MTK_INST_IRQ_RECEIVED,
					   VDEC_MPEG2_TIMEOUT_MS, MTK_VDEC_CORE);
	if (ret) {
		mtk_vdec_err(ctx, "timeout at MB %u,%u: read %#x proc %#x err %#x irq %#x",
			     vdec_read(inst, VDEC_CM, MC_MB_X),
			     vdec_read(inst, VDEC_CM, MC_MB_Y),
			     vdec_read(inst, VDEC_LD, VLD_RD_ADDR),
			     vdec_read(inst, VDEC_LD, VLD_PROC_STA),
			     vdec_read(inst, VDEC_LD, VLD_ERR_STA),
			     vdec_read(inst, VDEC_MISC, MISC_IRQ_STA));
		ret = -ETIMEDOUT;
		goto err_break;
	}

	mb_x = vdec_read(inst, VDEC_CM, MC_MB_X);
	mb_y = vdec_read(inst, VDEC_CM, MC_MB_Y);
	mbh = pic->picture_structure == V4L2_MPEG2_PIC_FRAME ? inst->mbh : inst->mbh / 2;
	ctx->decoded_frame_cnt++;
	if (mb_x != inst->mbw - 1 || mb_y != mbh - 1) {
		mtk_vdec_err(ctx, "picture incomplete: last MB %u,%u", mb_x, mb_y);
		return -EBADMSG;
	}
	for (i = 0; i < ARRAY_SIZE(crc); i++)
		crc[i] = vdec_read(inst, VDEC_MISC, MISC_CRC(i));
	mtk_vdec_debug(ctx, "picture %d done, CRC %08x %08x %08x %08x %08x %08x %08x %08x",
		       ctx->decoded_frame_cnt - 1, crc[0], crc[1], crc[2], crc[3],
		       crc[4], crc[5], crc[6], crc[7]);

	return 0;

err_break:
	mtk_vdec_direct_hw_break(inst->dev);
	return ret;
}

static int vdec_mpeg2_direct_init(struct mtk_vcodec_dec_ctx *ctx)
{
	struct vdec_mpeg2_direct_inst *inst;

	inst = kzalloc_obj(*inst);
	if (!inst)
		return -ENOMEM;
	inst->ctx = ctx;
	inst->dev = ctx->dev;
	inst->last_field_idx = -1;
	ctx->drv_handle = inst;

	return 0;
}

static void vdec_mpeg2_direct_deinit(void *h_vdec)
{
	kfree(h_vdec);
}

static int vdec_mpeg2_direct_get_param(void *h_vdec, enum vdec_get_param_type type,
				       void *out)
{
	struct vdec_mpeg2_direct_inst *inst = h_vdec;
	struct vdec_pic_info *pic = &inst->ctx->picinfo;
	struct v4l2_rect *crop;

	switch (type) {
	case GET_PARAM_PIC_INFO:
		/* MM21: the tile rows of the decoder follow the picture width */
		pic->buf_w = ALIGN(pic->pic_w, MPEG2_MB_SIZE);
		pic->buf_h = ALIGN(pic->pic_h, 32);
		pic->fb_sz[0] = pic->buf_w * pic->buf_h;
		pic->fb_sz[1] = pic->fb_sz[0] / 2;
		*(struct vdec_pic_info *)out = *pic;
		break;
	case GET_PARAM_DPB_SIZE:
		/* two anchor pictures and the target */
		*(unsigned int *)out = 3;
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

const struct vdec_common_if vdec_mpeg2_slice_direct_if = {
	.init		= vdec_mpeg2_direct_init,
	.decode		= vdec_mpeg2_direct_decode,
	.get_param	= vdec_mpeg2_direct_get_param,
	.deinit		= vdec_mpeg2_direct_deinit,
};
