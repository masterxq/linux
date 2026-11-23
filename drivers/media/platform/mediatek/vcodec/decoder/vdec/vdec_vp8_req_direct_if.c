// SPDX-License-Identifier: GPL-2.0
/*
 * VP8 decoding on MediaTek decoders that the host programs through their
 * registers, without firmware (MT6785).
 *
 * One hardware run decodes one frame. The VP8 syntax block contains the
 * boolean decoder of the first partition (bitstream reader VDEC_LD) and the
 * one of the DCT partitions (second bitstream reader); it decodes the modes
 * and tokens of all macroblocks itself.
 *
 * The decoder has to be positioned behind the frame header, so the host
 * walks the header with the boolean decoder of the syntax block: it skips
 * the uncompressed part with raw reads, reads all flags and literals of
 * the header through the literal port and lets the decoder parse the token
 * and motion vector probability updates itself. Before that the host loads
 * the probability tables with the values of the frame control, which
 * already contain the updates of this frame; the decoder replaces each
 * updated probability with the same value again. The values that the host
 * reads are checked against the frame control.
 *
 * The clamping type is only coded in key frames and is not part of the
 * frame control; it is kept from the last key frame. The segment map of a
 * frame that does not update it comes from the macroblock data the decoder
 * wrote to the buffers of the instance in the previous frame.
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

/* bitstream readers (VDEC_LD for the first partition, the second reader for the DCT partitions) */
#define VLD_BS_CTRL			0x08c
#define VLD_BS_CTRL_FETCH		BIT(20)
#define VLD_BS_CTRL_RUN			BIT(23)
#define VLD_RPTR			0x0b0
#define VLD_VSTART			0x0b4
#define VLD_VEND			0x0b8
#define VLD_FETCH_STA			0x0e8
#define VLD_FETCH_STA_READY		BIT(0)
#define VLD_PROC_STA			0x0f4
#define VLD_PROC_STA_BUSY		BIT(15)
#define VLD_PROC_STA_IDLE		BIT(0)
#define VLD_SRST_READER			BIT(8)
#define VLD_WPTR			0x110
#define VLD_TAB_ADDR			0x260
#define VLD_TAB_DATA			0x264

/* VP8 syntax block */
#define VP8_BITS(n)			(0x000 + 4 * (n))	/* first reader: read 0 peeks, n skips n bits */
#define VP8_LITERAL(n)			(0x080 + 4 * (n))	/* n-bit literal of the boolean decoder */
#define VP8_CTRL			0x0a4
#define VP8_CTRL_EN			BIT(0)
#define VP8_PIC				0x0a8
#define VP8_PIC_PART1_END		GENMASK(31, 8)
#define VP8_PIC_VERSION			GENMASK(3, 1)
#define VP8_PIC_INTER			BIT(0)
#define VP8_HDR				0x0ac
#define VP8_HDR_CLAMP_TYPE		BIT(0)
#define VP8_HDR_SEG_EN			BIT(4)
#define VP8_HDR_SEG_UPDATE_MAP		BIT(8)
#define VP8_HDR_LOG2_PARTS		GENMASK(13, 12)
#define VP8_HDR_NO_SKIP_COEFF		BIT(16)
#define VP8_PROBS			0x0b0
#define VP8_PROBS_SKIP_FALSE		GENMASK(7, 0)
#define VP8_PROBS_INTRA			GENMASK(15, 8)
#define VP8_PROBS_LAST			GENMASK(23, 16)
#define VP8_PROBS_GF			GENMASK(31, 24)
#define VP8_START			0x0b8
#define VP8_PART_ADDR(i)		(0x0c0 + 4 * (i))
#define VP8_PART_SIZE(i)		(0x0e0 + 4 * (i))
#define VP8_COEF_ADDR			0x100
#define VP8_COEF_DATA			0x104
#define VP8_CTX_ADDR			0x108
#define VP8_CTX_DATA			0x10c
#define VP8_CTX_SEGMENT			0x00
#define VP8_CTX_YMODE			0x04
#define VP8_CTX_UVMODE			0x08
#define VP8_CTX_MV_SHORT(i)		(0x0c + 8 * (i))
#define VP8_CTX_MV_LONG(i)		(0x10 + 8 * (i))
#define VP8_BOOL_INIT1			0x110
#define VP8_BOOL_INIT2			0x114
#define VP8_MV_PROBS			0x120
#define VP8_MV_PROBS_SHORT_COL		GENMASK(7, 0)
#define VP8_MV_PROBS_SHORT_ROW		GENMASK(15, 8)
#define VP8_MV_PROBS_SIGN_COL		GENMASK(23, 16)
#define VP8_MV_PROBS_SIGN_ROW		GENMASK(31, 24)
#define VP8_READER_INIT			0x144
#define VP8_READER_INIT_FIRST		BIT(0)
#define VP8_READER_INIT_DCT		BIT(8)
#define VP8_COEF_UPDATE			0x148
#define VP8_MV_UPDATE			0x14c
#define VP8_ERR_STA			0x15c
#define VP8_DONE			0x174
#define VP8_BITS2(n)			(0x200 + 4 * (n))	/* the same for the second reader */

/* VDEC_TOP */
#define TOP_PIC_MBS			0x068
#define TOP_PIC_MBS_H_M1		GENMASK(31, 16)
#define TOP_PIC_MBS_W_M1		GENMASK(15, 0)
#define TOP_MB_BUF			0x028	/* macroblock data for the next frame */
#define TOP_CFG_02C			0x02c
#define TOP_CFG_02C_SEG_BUF		BIT(30)	/* segment ids and macroblock data in memory */
#define TOP_SEG_ID_BUF			0x03c	/* with the bit: segment ids, else internal memory */
#define TOP_CFG_05C			0x05c
#define TOP_CFG_05C_MASK		GENMASK(30, 29)
#define TOP_CFG_05C_VP8			BIT(29)

/* motion vectors (VDEC_AV) */
#define MV_SIGN_BIAS			0x2d4
#define MV_SIGN_BIAS_GOLDEN		BIT(0)
#define MV_SIGN_BIAS_ALT		BIT(1)

/* motion compensation (VDEC_CM) */
#define MC_PIC_W			0x208
#define MC_PIC_H			0x20c
#define MC_CFG_220			0x220
#define MC_Y_ADDR_512			0x224
#define MC_C_ADDR_256			0x228
#define MC_PIC_W_MBS			0x22c
#define MC_LF				0x238
#define MC_LF_NORMAL			3
#define MC_LF_SIMPLE			2
#define MC_CFG_250			0x250
#define MC_PIC_W_MBS_M1			0x260
#define MC_PIC_H_MBS_M1			0x264
#define MC_REF_Y(i)			(0x3e0 + 4 * (i))	/* last, golden, altref */
#define MC_CFG_57C			0x57c
#define MC_PIC_END			0x850
#define MC_PIC_START			0x854
#define MC_CFG_858			0x858
#define MC_CFG_920			0x920
#define MC_CFG_920_RUN			BIT(24)
#define MC_C_OFFSET			0x934
#define MC_FILTER			0x97c
#define MC_FILTER_KEEP			BIT(0)
#define MC_FILTER_SIXTAP		BIT(4)
#define MC_FILTER_FULL_PIXEL		BIT(8)
#define MC_PIC_W_MBS2			0x980
#define MC_VLD_WRAP_ADDR		0xa20
#define MC_PP_WRAP_ADDR			0xa24
#define MC_YC_SEP			0xb5c
#define MC_YC_SEP_EN			BIT(0)
#define MC_YC_SEP_UFO			BIT(4)
#define MC_REF_C(i)			(0xb64 + 4 * (i))

/* post-processing (VDEC_PP): loop filter */
#define PP_LF_CFG			0x020
#define PP_LF_CFG_EN			BIT(0)
#define PP_LF_CFG_SIMPLE		BIT(4)
#define PP_LF_CFG_DELTA			BIT(8)
#define PP_LF_CFG_INTER			BIT(12)
#define PP_LF_CFG_SHARPNESS		GENMASK(18, 16)
#define PP_LF_LEVEL			0x024	/* one byte per segment */
#define PP_LF_REF_DELTA			0x028	/* one signed byte per reference frame */
#define PP_LF_MODE_DELTA		0x02c	/* one signed byte per mode class */

/* control (VDEC_MISC) */
#define MISC_CRC_EN			0x004
#define MISC_CRC_EN_ON			BIT(0)
#define MISC_CRC_EN_RUN			BIT(1)
#define MISC_CRC(n)			(0x008 + 4 * (n))
#define MISC_SYS_SEL			0x084
#define MISC_IRQ_STA			0x0a4
#define MISC_SPEC_CON			0x0c8
#define MISC_SPEC_CON_VAL		0xff7f
#define MISC_FUNC_CON			0x0cc
#define MISC_FUNC_CON_VAL		0x71201102
#define MISC_DCM_CON			0x0ec
#define MISC_CFG_0F0			0x0f0
#define MISC_CFG_0F0_EN			BIT(0)
#define MISC_SRST			0x0f4
#define MISC_SIZE_CFG			0x160
#define MISC_SIZE_CFG_NORMAL		0x65ff10
#define MISC_SIZE_CFG_LARGE		0x1eff10
#define MISC_CFG_178			0x178
#define MISC_CFG_178_VAL		0xfffffffb
#define MISC_CFG_1F0			0x1f0
#define MISC_CFG_1F0_RESET		0

#define VP8_MB_SIZE			16
#define VP8_KEY_HDR_SIZE		10
#define VP8_INTER_HDR_SIZE		3
#define VP8_MAX_PARTS			8
#define VP8_NUM_SEGMENTS		4
#define VP8_COEF_ENTRIES		(4 * 8 * 3)
/* bytes per macroblock in the buffers of the segment map; rows counted in pairs */
#define VP8_SEG_MB_BYTES		4

/* zeros the readers may fetch behind the bitstream */
#define VDEC_VP8_BS_PAD			1024
#define VDEC_VP8_BS_MIN_PAD		64

#define VDEC_VP8_VLD_WRAP_SZ		0xf000
#define VDEC_VP8_PP_WRAP_SZ		0x1e000
#define VDEC_VP8_LARGE_PIC		(1920 * 1088)

#define VDEC_VP8_POLL_US		10000
#define VDEC_VP8_TIMEOUT_MS		500

/* dequantisation factors of RFC 6386, section 14.1 */
static const u8 vp8_dc_qlookup[128] = {
	4, 5, 6, 7, 8, 9, 10, 10, 11, 12, 13, 14, 15, 16, 17, 17,
	18, 19, 20, 20, 21, 21, 22, 22, 23, 23, 24, 25, 25, 26, 27, 28,
	29, 30, 31, 32, 33, 34, 35, 36, 37, 37, 38, 39, 40, 41, 42, 43,
	44, 45, 46, 46, 47, 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58,
	59, 60, 61, 62, 63, 64, 65, 66, 67, 68, 69, 70, 71, 72, 73, 74,
	75, 76, 76, 77, 78, 79, 80, 81, 82, 83, 84, 85, 86, 87, 88, 89,
	91, 93, 95, 96, 98, 100, 101, 102, 104, 106, 108, 110, 112, 114, 116, 118,
	122, 124, 126, 128, 130, 132, 134, 136, 138, 140, 143, 145, 148, 151, 154, 157,
};

static const u16 vp8_ac_qlookup[128] = {
	4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16, 17, 18, 19,
	20, 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32, 33, 34, 35,
	36, 37, 38, 39, 40, 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51,
	52, 53, 54, 55, 56, 57, 58, 60, 62, 64, 66, 68, 70, 72, 74, 76,
	78, 80, 82, 84, 86, 88, 90, 92, 94, 96, 98, 100, 102, 104, 106, 108,
	110, 112, 114, 116, 119, 122, 125, 128, 131, 134, 137, 140, 143, 146, 149, 152,
	155, 158, 161, 164, 167, 170, 173, 177, 181, 185, 189, 193, 197, 201, 205, 209,
	213, 217, 221, 225, 229, 234, 239, 245, 249, 254, 259, 264, 269, 274, 279, 284,
};

struct vdec_vp8_direct_pic {
	dma_addr_t y;
	dma_addr_t c;
};

/* header fields that the host reads from the bitstream */
struct vdec_vp8_direct_hdr {
	bool seg_en;
	bool seg_update_map;
	bool filter_simple;
	u8 level;
	u8 sharpness;
	bool lf_adj;
	u8 log2_parts;
	u8 y_ac_qi;
	bool no_skip_coeff;
	u8 prob_skip_false;
	u8 prob_intra;
	u8 prob_last;
	u8 prob_gf;
};

/**
 * struct vdec_vp8_direct_inst - VP8 decoder instance
 * @ctx:	decoder context
 * @dev:	decoder device
 * @vp8:	registers of the VP8 syntax block
 * @vld2:	registers of the second bitstream reader
 * @frame:	frame control of the current request
 * @hdr:	header fields read from the bitstream
 * @mbw:	width in macroblocks
 * @mbh:	height in macroblocks
 * @clamp_type:	clamping type of the last key frame
 * @seg_buf:	segment ids, kept from frame to frame
 * @mb_buf:	macroblock data that goes with them
 * @top_seg:	previous value of VDEC_TOP 0x03c, restored after the frame
 * @vld_wrap:	reader working buffer
 * @pp_wrap:	post-processing working buffer
 */
struct vdec_vp8_direct_inst {
	struct mtk_vcodec_dec_ctx *ctx;
	struct mtk_vcodec_dec_dev *dev;
	void __iomem *vp8;
	void __iomem *vld2;

	const struct v4l2_ctrl_vp8_frame *frame;
	struct vdec_vp8_direct_hdr hdr;

	unsigned int mbw;
	unsigned int mbh;
	bool clamp_type;

	struct mtk_vcodec_mem vld_wrap;
	struct mtk_vcodec_mem pp_wrap;
	struct mtk_vcodec_mem seg_buf;
	struct mtk_vcodec_mem mb_buf;
	u32 top_seg;
};

static inline void vdec_write(struct vdec_vp8_direct_inst *inst, int blk,
			      u32 off, u32 val)
{
	writel(val, inst->dev->reg_base[blk] + off);
}

static inline u32 vdec_read(struct vdec_vp8_direct_inst *inst, int blk, u32 off)
{
	return readl(inst->dev->reg_base[blk] + off);
}

static inline void vdec_mask(struct vdec_vp8_direct_inst *inst, int blk,
			     u32 off, u32 clear, u32 set)
{
	vdec_write(inst, blk, off, (vdec_read(inst, blk, off) & ~clear) | set);
}

static inline void vp8_write(struct vdec_vp8_direct_inst *inst, u32 off, u32 val)
{
	writel(val, inst->vp8 + off);
}

static inline u32 vp8_read(struct vdec_vp8_direct_inst *inst, u32 off)
{
	return readl(inst->vp8 + off);
}

static int vdec_vp8_poll(struct vdec_vp8_direct_inst *inst, void __iomem *reg,
			 u32 mask, bool set, const char *what)
{
	u32 val;
	int ret;

	ret = readl_poll_timeout(reg, val, !!(val & mask) == set, 1, VDEC_VP8_POLL_US);
	if (ret)
		mtk_vdec_err(inst->ctx, "%s timeout: %#x", what, val);

	return ret;
}

/* ---------------- bitstream access through the syntax block ---------------- */

/* n (1 to 24) raw bits of the first or the second reader, first bit on top */
static u32 vdec_vp8_raw(struct vdec_vp8_direct_inst *inst, bool second, unsigned int n)
{
	u32 base = second ? VP8_BITS2(0) : VP8_BITS(0);
	u32 val = vp8_read(inst, base) >> (32 - n);

	vp8_read(inst, base + 4 * n);

	return val;
}

/* little-endian value of n bytes from the raw bits */
static u32 vdec_vp8_raw_le(struct vdec_vp8_direct_inst *inst, bool second, unsigned int n)
{
	u32 val = 0;
	unsigned int i;

	for (i = 0; i < n; i++)
		val |= vdec_vp8_raw(inst, second, 8) << (8 * i);

	return val;
}

static inline u32 vdec_vp8_lit(struct vdec_vp8_direct_inst *inst, unsigned int n)
{
	return vp8_read(inst, VP8_LITERAL(n));
}

/* optional signed value: flag, magnitude, sign */
static void vdec_vp8_skip_signed(struct vdec_vp8_direct_inst *inst, unsigned int n)
{
	if (vdec_vp8_lit(inst, 1)) {
		vdec_vp8_lit(inst, n);
		vdec_vp8_lit(inst, 1);
	}
}

/* ---------------- parameters ---------------- */

static int vdec_vp8_get_params(struct vdec_vp8_direct_inst *inst, struct mtk_vcodec_mem *bs)
{
	const struct mtk_vdec_direct_data *data = inst->dev->vdec_pdata->direct;
	struct mtk_vcodec_dec_ctx *ctx = inst->ctx;
	const struct v4l2_ctrl_vp8_frame *f;
	struct v4l2_ctrl *ctrl;
	size_t hdr_len, end;
	unsigned int i;

	ctrl = v4l2_ctrl_find(&ctx->ctrl_hdl, V4L2_CID_STATELESS_VP8_FRAME);
	if (!ctrl)
		return -EINVAL;
	f = ctrl->p_cur.p;
	inst->frame = f;

	inst->mbw = DIV_ROUND_UP(f->width, VP8_MB_SIZE);
	inst->mbh = DIV_ROUND_UP(f->height, VP8_MB_SIZE);
	if (!inst->mbw || !inst->mbh ||
	    inst->mbw * VP8_MB_SIZE > data->max_width ||
	    inst->mbh * VP8_MB_SIZE > data->max_height) {
		mtk_vdec_err(ctx, "frame %ux%u not supported", f->width, f->height);
		return -EINVAL;
	}
	if (f->num_dct_parts != 1 && f->num_dct_parts != 2 &&
	    f->num_dct_parts != 4 && f->num_dct_parts != VP8_MAX_PARTS) {
		mtk_vdec_err(ctx, "%u DCT partitions", f->num_dct_parts);
		return -EINVAL;
	}

	hdr_len = V4L2_VP8_FRAME_IS_KEY_FRAME(f) ? VP8_KEY_HDR_SIZE : VP8_INTER_HDR_SIZE;
	end = hdr_len + f->first_part_size + 3 * (f->num_dct_parts - 1);
	for (i = 0; i < f->num_dct_parts; i++)
		end += f->dct_part_sizes[i];
	if (end > bs->size) {
		mtk_vdec_err(ctx, "partitions end at %zu behind the frame of %zu bytes",
			     end, bs->size);
		return -EINVAL;
	}

	return 0;
}

static unsigned int vdec_vp8_luma_size(struct vdec_vp8_direct_inst *inst)
{
	return inst->mbw * VP8_MB_SIZE * ALIGN(inst->mbh * VP8_MB_SIZE, 32);
}

static void vdec_vp8_ref(struct vdec_vp8_direct_inst *inst, u64 ts,
			 const struct vdec_vp8_direct_pic *dst,
			 struct vdec_vp8_direct_pic *ref)
{
	struct vb2_queue *vq = v4l2_m2m_get_vq(inst->ctx->m2m_ctx,
					       V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);
	struct vb2_buffer *vb;

	if (V4L2_VP8_FRAME_IS_KEY_FRAME(inst->frame)) {
		*ref = *dst;
		return;
	}
	vb = vb2_find_buffer(vq, ts);
	if (!vb) {
		/* missing reference: predict from the target itself */
		mtk_vdec_debug(inst->ctx, "reference %llu not found", ts);
		*ref = *dst;
		return;
	}
	ref->y = vb2_dma_contig_plane_dma_addr(vb, 0);
	ref->c = vb2_dma_contig_plane_dma_addr(vb, 1);
}

static int vdec_vp8_bs_end(struct vdec_vp8_direct_inst *inst,
			   struct mtk_vcodec_mem *bs, dma_addr_t *end)
{
	struct mtk_video_dec_buf *src = container_of(bs, struct mtk_video_dec_buf,
						     bs_buffer);
	dma_addr_t limit = bs->dma_addr +
			   vb2_plane_size(&src->m2m_buf.vb.vb2_buf, 0);
	dma_addr_t data_end = bs->dma_addr + bs->size;

	*end = ALIGN(data_end + VDEC_VP8_BS_PAD, 64);
	if (*end > limit)
		*end = ALIGN_DOWN(limit, 64);
	if (*end < data_end + VDEC_VP8_BS_MIN_PAD) {
		mtk_vdec_err(inst->ctx, "no room behind the bitstream (%zu of %pad)",
			     bs->size, &limit);
		return -EINVAL;
	}
	memset(bs->va + bs->size, 0, *end - data_end);

	return 0;
}

/* ---------------- register sequence of one frame ---------------- */

static void vdec_vp8_reset(struct vdec_vp8_direct_inst *inst)
{
	vdec_write(inst, VDEC_MISC, MISC_SRST, 0);
	vdec_write(inst, VDEC_LD, VDEC_VLD_SRST, VLD_SRST_READER | VDEC_VLD_SRST_RESET);
	writel(VLD_SRST_READER | VDEC_VLD_SRST_RESET, inst->vld2 + VDEC_VLD_SRST);
	vdec_write(inst, VDEC_MISC, MISC_SYS_SEL, 1);
	vdec_write(inst, VDEC_MISC, MISC_SPEC_CON, MISC_SPEC_CON_VAL);
	vdec_write(inst, VDEC_MISC, MISC_FUNC_CON, MISC_FUNC_CON_VAL);
	vdec_write(inst, VDEC_MISC, MISC_CFG_178, MISC_CFG_178_VAL);
	vdec_mask(inst, VDEC_CM, MC_CFG_920, 0, MC_CFG_920_RUN);
	vdec_write(inst, VDEC_LD, VDEC_VLD_SRST, 0);
	writel(0, inst->vld2 + VDEC_VLD_SRST);
	mtk_vcodec_write_vdecsys(inst->ctx, VDEC_SYS_DCM_CON, 1);
	vdec_write(inst, VDEC_MISC, MISC_DCM_CON, 1);
	vp8_write(inst, VP8_CTRL, VP8_CTRL_EN);
	vdec_write(inst, VDEC_MISC, MISC_SIZE_CFG,
		   inst->mbw * inst->mbh * VP8_MB_SIZE * VP8_MB_SIZE > VDEC_VP8_LARGE_PIC ?
		   MISC_SIZE_CFG_LARGE : MISC_SIZE_CFG_NORMAL);
	vdec_write(inst, VDEC_MISC, MISC_CFG_1F0, MISC_CFG_1F0_RESET);
}

/*
 * Point a reader at a byte of the bitstream. The reader starts at the 16
 * byte unit of the address; the bytes before it are skipped with raw reads.
 */
static int vdec_vp8_init_reader(struct vdec_vp8_direct_inst *inst, bool second,
				dma_addr_t base, dma_addr_t start, dma_addr_t end)
{
	void __iomem *vld = second ? inst->vld2 : inst->dev->reg_base[VDEC_LD];
	void __iomem *sta = inst->dev->reg_base[VDEC_LD] + VLD_PROC_STA;
	unsigned int i;
	int ret;

	if (readl(sta) & VLD_PROC_STA_BUSY) {
		ret = vdec_vp8_poll(inst, sta, VLD_PROC_STA_IDLE, true, "reader idle");
		if (ret)
			return ret;
	}
	writel(base >> 6, vld + VLD_VSTART);
	writel(end >> 6, vld + VLD_VEND);
	writel(start, vld + VLD_RPTR);
	writel(end, vld + VLD_WPTR);
	writel(VLD_SRST_READER, vld + VDEC_VLD_SRST);
	writel(0, vld + VDEC_VLD_SRST);
	writel(VLD_BS_CTRL_FETCH, vld + VLD_BS_CTRL);
	ret = vdec_vp8_poll(inst, vld + VLD_FETCH_STA, VLD_FETCH_STA_READY, true,
			    "bitstream fetch");
	if (ret)
		return ret;
	writel(VLD_BS_CTRL_RUN, vld + VLD_BS_CTRL);

	for (i = 0; i < (start & 0xf); i++)
		vdec_vp8_raw(inst, second, 8);

	return 0;
}

static void vdec_vp8_write_ctx(struct vdec_vp8_direct_inst *inst, u32 addr,
			       const u8 *p, unsigned int n)
{
	unsigned int i;
	u32 val = 0;

	/* four probabilities per word, the first one in the low byte */
	vp8_write(inst, VP8_CTX_ADDR, addr);
	for (i = 0; i < n; i++) {
		val |= p[i] << (8 * (i % 4));
		if (i % 4 == 3 || i == n - 1) {
			vp8_write(inst, VP8_CTX_DATA, val);
			val = 0;
		}
	}
}

/* mode and motion vector probabilities of the frame control */
static void vdec_vp8_write_mode_probs(struct vdec_vp8_direct_inst *inst)
{
	const struct v4l2_vp8_entropy *e = &inst->frame->entropy;
	unsigned int i;

	vdec_vp8_write_ctx(inst, VP8_CTX_YMODE, e->y_mode_probs, ARRAY_SIZE(e->y_mode_probs));
	vdec_vp8_write_ctx(inst, VP8_CTX_UVMODE, e->uv_mode_probs,
			   ARRAY_SIZE(e->uv_mode_probs));
	/* row first; is_short, sign, 7 short tree, 10 long bits */
	vp8_write(inst, VP8_MV_PROBS,
		  FIELD_PREP(VP8_MV_PROBS_SHORT_COL, e->mv_probs[1][0]) |
		  FIELD_PREP(VP8_MV_PROBS_SHORT_ROW, e->mv_probs[0][0]) |
		  FIELD_PREP(VP8_MV_PROBS_SIGN_COL, e->mv_probs[1][1]) |
		  FIELD_PREP(VP8_MV_PROBS_SIGN_ROW, e->mv_probs[0][1]));
	for (i = 0; i < 2; i++) {
		vdec_vp8_write_ctx(inst, VP8_CTX_MV_SHORT(i), &e->mv_probs[i][2], 7);
		vdec_vp8_write_ctx(inst, VP8_CTX_MV_LONG(i), &e->mv_probs[i][9], 10);
	}
}

static void vdec_vp8_write_coef_probs(struct vdec_vp8_direct_inst *inst)
{
	const u8 *p = &inst->frame->entropy.coeff_probs[0][0][0][0];
	unsigned int i, j;

	/* eleven probabilities of a context in three words, low byte first */
	for (i = 0; i < VP8_COEF_ENTRIES; i++, p += V4L2_VP8_COEFF_PROB_CNT) {
		vp8_write(inst, VP8_COEF_ADDR, 4 * i);
		for (j = 0; j < V4L2_VP8_COEFF_PROB_CNT; j += 4)
			vp8_write(inst, VP8_COEF_DATA,
				  p[j] | p[j + 1] << 8 | p[j + 2] << 16 |
				  (j + 3 < V4L2_VP8_COEFF_PROB_CNT ? p[j + 3] << 24 : 0));
	}
}

/*
 * Dequantisation factors per segment: Y2, Y1 and chroma, each DC << 16 | AC,
 * at 16 * segment + 0, 4 and 8 of the table port of VDEC_LD.
 */
static void vdec_vp8_write_dequant(struct vdec_vp8_direct_inst *inst)
{
	const struct v4l2_ctrl_vp8_frame *f = inst->frame;
	const struct v4l2_vp8_quantization *qt = &f->quant;
	u32 dc, ac, word[3];
	unsigned int s, k;
	int q;

	for (s = 0; s < VP8_NUM_SEGMENTS; s++) {
		q = qt->y_ac_qi;
		if (f->segment.flags & V4L2_VP8_SEGMENT_FLAG_ENABLED) {
			if (f->segment.flags & V4L2_VP8_SEGMENT_FLAG_DELTA_VALUE_MODE)
				q += f->segment.quant_update[s];
			else
				q = f->segment.quant_update[s];
		}
		q = clamp(q, 0, 127);

		dc = vp8_dc_qlookup[clamp(q + qt->y2_dc_delta, 0, 127)] * 2;
		ac = max(vp8_ac_qlookup[clamp(q + qt->y2_ac_delta, 0, 127)] * 155 / 100, 8);
		word[0] = dc << 16 | ac;
		dc = vp8_dc_qlookup[clamp(q + qt->y_dc_delta, 0, 127)];
		word[1] = dc << 16 | vp8_ac_qlookup[q];
		dc = min(vp8_dc_qlookup[clamp(q + qt->uv_dc_delta, 0, 127)], 132);
		word[2] = dc << 16 | vp8_ac_qlookup[clamp(q + qt->uv_ac_delta, 0, 127)];

		for (k = 0; k < ARRAY_SIZE(word); k++) {
			vdec_write(inst, VDEC_LD, VLD_TAB_ADDR, 16 * s + 4 * k);
			vdec_write(inst, VDEC_LD, VLD_TAB_DATA, word[k]);
		}
	}
}

/* DCT partitions: sizes from the table in front of them, read by the second reader */
static int vdec_vp8_init_parts(struct vdec_vp8_direct_inst *inst,
			       struct mtk_vcodec_mem *bs, dma_addr_t end)
{
	const struct v4l2_ctrl_vp8_frame *f = inst->frame;
	size_t off = (V4L2_VP8_FRAME_IS_KEY_FRAME(f) ? VP8_KEY_HDR_SIZE : VP8_INTER_HDR_SIZE) +
		     f->first_part_size;
	unsigned int i, n = f->num_dct_parts;
	u32 size;
	int ret;

	ret = vdec_vp8_init_reader(inst, true, bs->dma_addr, bs->dma_addr + off, end);
	if (ret)
		return ret;
	off += 3 * (n - 1);
	for (i = 0; i < n; i++) {
		size = i < n - 1 ? vdec_vp8_raw_le(inst, true, 3) : f->dct_part_sizes[i];
		if (size != f->dct_part_sizes[i]) {
			mtk_vdec_err(inst->ctx, "partition %u: %u bytes, control %u", i, size,
				     f->dct_part_sizes[i]);
			return -EINVAL;
		}
		vp8_write(inst, VP8_PART_ADDR(i), bs->dma_addr + off);
		vp8_write(inst, VP8_PART_SIZE(i), size);
		off += size;
	}
	vp8_write(inst, VP8_READER_INIT, VP8_READER_INIT_DCT);
	vp8_write(inst, VP8_READER_INIT, 0);
	vp8_read(inst, VP8_BOOL_INIT2);

	return 0;
}

/*
 * Walk the frame header with the boolean decoder of the first partition up
 * to the first macroblock (RFC 6386, section 19.2). Values the decoder
 * needs are taken from the frame control, the ones read here are compared.
 */
static int vdec_vp8_read_hdr(struct vdec_vp8_direct_inst *inst,
			     struct mtk_vcodec_mem *bs, dma_addr_t end)
{
	const struct v4l2_ctrl_vp8_frame *f = inst->frame;
	struct vdec_vp8_direct_hdr *h = &inst->hdr;
	bool key = V4L2_VP8_FRAME_IS_KEY_FRAME(f);
	unsigned int i;
	int ret;

	memset(h, 0, sizeof(*h));
	if (key) {
		vdec_vp8_lit(inst, 1);			/* color_space */
		inst->clamp_type = vdec_vp8_lit(inst, 1);
	}
	h->seg_en = vdec_vp8_lit(inst, 1);
	if (h->seg_en) {
		bool update_data;

		h->seg_update_map = vdec_vp8_lit(inst, 1);
		update_data = vdec_vp8_lit(inst, 1);
		if (update_data) {
			vdec_vp8_lit(inst, 1);		/* segment_feature_mode */
			for (i = 0; i < VP8_NUM_SEGMENTS; i++)
				vdec_vp8_skip_signed(inst, 7);
			for (i = 0; i < VP8_NUM_SEGMENTS; i++)
				vdec_vp8_skip_signed(inst, 6);
		}
		if (h->seg_update_map) {
			for (i = 0; i < 3; i++)
				if (vdec_vp8_lit(inst, 1))
					vdec_vp8_lit(inst, 8);
			vdec_vp8_write_ctx(inst, VP8_CTX_SEGMENT, f->segment.segment_probs,
					   ARRAY_SIZE(f->segment.segment_probs));
		}
	}
	h->filter_simple = vdec_vp8_lit(inst, 1);
	h->level = vdec_vp8_lit(inst, 6);
	h->sharpness = vdec_vp8_lit(inst, 3);
	h->lf_adj = vdec_vp8_lit(inst, 1);
	if (h->lf_adj && vdec_vp8_lit(inst, 1))
		for (i = 0; i < 8; i++)
			vdec_vp8_skip_signed(inst, 6);
	h->log2_parts = vdec_vp8_lit(inst, 2);
	if ((1 << h->log2_parts) != f->num_dct_parts) {
		mtk_vdec_err(inst->ctx, "%u DCT partitions, control %u",
			     1 << h->log2_parts, f->num_dct_parts);
		return -EINVAL;
	}
	ret = vdec_vp8_init_parts(inst, bs, end);
	if (ret)
		return ret;

	h->y_ac_qi = vdec_vp8_lit(inst, 7);
	for (i = 0; i < 5; i++)
		vdec_vp8_skip_signed(inst, 4);
	vdec_vp8_write_dequant(inst);

	if (key) {
		vdec_vp8_lit(inst, 1);			/* refresh_entropy_probs */
	} else {
		bool golden = vdec_vp8_lit(inst, 1);
		bool alt = vdec_vp8_lit(inst, 1);

		if (!golden)
			vdec_vp8_lit(inst, 2);		/* copy_buffer_to_golden */
		if (!alt)
			vdec_vp8_lit(inst, 2);		/* copy_buffer_to_alternate */
		vdec_vp8_lit(inst, 1);			/* sign_bias_golden */
		vdec_vp8_lit(inst, 1);			/* sign_bias_alternate */
		vdec_vp8_lit(inst, 1);			/* refresh_entropy_probs */
		vdec_vp8_lit(inst, 1);			/* refresh_last */
		vdec_write(inst, VDEC_AV, MV_SIGN_BIAS,
			   (f->flags & V4L2_VP8_FRAME_FLAG_SIGN_BIAS_GOLDEN ?
			    MV_SIGN_BIAS_GOLDEN : 0) |
			   (f->flags & V4L2_VP8_FRAME_FLAG_SIGN_BIAS_ALT ? MV_SIGN_BIAS_ALT : 0));
	}

	/* token probability updates, parsed by the decoder */
	vdec_vp8_write_coef_probs(inst);
	vp8_write(inst, VP8_COEF_UPDATE, 1);
	ret = vdec_vp8_poll(inst, inst->vp8 + VP8_COEF_UPDATE, 1, false, "token probabilities");
	if (ret)
		return ret;

	h->no_skip_coeff = vdec_vp8_lit(inst, 1);
	if (h->no_skip_coeff)
		h->prob_skip_false = vdec_vp8_lit(inst, 8);
	if (!key) {
		h->prob_intra = vdec_vp8_lit(inst, 8);
		h->prob_last = vdec_vp8_lit(inst, 8);
		h->prob_gf = vdec_vp8_lit(inst, 8);
		if (vdec_vp8_lit(inst, 1))
			for (i = 0; i < 4; i++)
				vdec_vp8_lit(inst, 8);
		if (vdec_vp8_lit(inst, 1))
			for (i = 0; i < 3; i++)
				vdec_vp8_lit(inst, 8);
		/* motion vector probability updates, parsed by the decoder */
		vp8_write(inst, VP8_MV_UPDATE, 1);
		ret = vdec_vp8_poll(inst, inst->vp8 + VP8_MV_UPDATE, 1, false,
				    "motion vector probabilities");
		if (ret)
			return ret;
	}

	if (h->seg_en != !!(f->segment.flags & V4L2_VP8_SEGMENT_FLAG_ENABLED) ||
	    h->seg_update_map != !!(f->segment.flags & V4L2_VP8_SEGMENT_FLAG_UPDATE_MAP) ||
	    h->filter_simple != !!(f->lf.flags & V4L2_VP8_LF_FILTER_TYPE_SIMPLE) ||
	    h->level != f->lf.level || h->sharpness != f->lf.sharpness_level ||
	    h->lf_adj != !!(f->lf.flags & V4L2_VP8_LF_ADJ_ENABLE) ||
	    h->y_ac_qi != f->quant.y_ac_qi ||
	    h->no_skip_coeff != !!(f->flags & V4L2_VP8_FRAME_FLAG_MB_NO_SKIP_COEFF) ||
	    h->prob_skip_false != (h->no_skip_coeff ? f->prob_skip_false : 0) ||
	    (!key && (h->prob_intra != f->prob_intra || h->prob_last != f->prob_last ||
		      h->prob_gf != f->prob_gf))) {
		mtk_vdec_err(inst->ctx, "frame header differs from the frame control");
		return -EINVAL;
	}

	return 0;
}

static int vdec_vp8_hdr(struct vdec_vp8_direct_inst *inst, struct mtk_vcodec_mem *bs,
			dma_addr_t end)
{
	const struct v4l2_ctrl_vp8_frame *f = inst->frame;
	bool key = V4L2_VP8_FRAME_IS_KEY_FRAME(f);
	const u8 *d = bs->va;
	u32 tag;
	int ret;

	ret = vdec_vp8_init_reader(inst, false, bs->dma_addr, bs->dma_addr, end);
	if (ret)
		return ret;
	vp8_write(inst, VP8_READER_INIT, VP8_READER_INIT_FIRST);
	vp8_write(inst, VP8_READER_INIT, 0);
	writel(bs->dma_addr >> 6, inst->vld2 + VLD_VSTART);
	writel(end >> 6, inst->vld2 + VLD_VEND);
	vdec_write(inst, VDEC_MISC, MISC_CRC_EN, MISC_CRC_EN_ON);

	/* frame tag and, in key frames, start code and size: raw bytes */
	tag = vdec_vp8_raw_le(inst, false, VP8_INTER_HDR_SIZE);
	if (!(tag & 1) != key || (tag >> 5) != f->first_part_size ||
	    tag != (d[0] | d[1] << 8 | d[2] << 16)) {
		mtk_vdec_err(inst->ctx, "frame tag %#x does not match the frame control", tag);
		return -EINVAL;
	}
	if (key) {
		vdec_vp8_raw_le(inst, false, 3);
		vdec_vp8_raw_le(inst, false, 2);
		vdec_vp8_raw_le(inst, false, 2);
	}

	vdec_write(inst, VDEC_TOP, TOP_PIC_MBS,
		   FIELD_PREP(TOP_PIC_MBS_H_M1, inst->mbh - 1) |
		   FIELD_PREP(TOP_PIC_MBS_W_M1, inst->mbw - 1));
	vp8_write(inst, VP8_PIC,
		  FIELD_PREP(VP8_PIC_PART1_END, (key ? VP8_KEY_HDR_SIZE : VP8_INTER_HDR_SIZE) +
			     f->first_part_size) |
		  FIELD_PREP(VP8_PIC_VERSION, f->version) | (key ? 0 : VP8_PIC_INTER));
	vdec_vp8_write_mode_probs(inst);
	vp8_read(inst, VP8_BOOL_INIT1);

	return vdec_vp8_read_hdr(inst, bs, end);
}

static void vdec_vp8_write_lf(struct vdec_vp8_direct_inst *inst)
{
	const struct v4l2_ctrl_vp8_frame *f = inst->frame;
	const struct v4l2_vp8_loop_filter *lf = &f->lf;
	bool simple = lf->flags & V4L2_VP8_LF_FILTER_TYPE_SIMPLE;
	u32 level = 0, ref = 0, mode = 0;
	unsigned int s;
	int l;

	/* a frame level of 0 turns the filter off, whatever the segments say */
	if (!lf->level)
		return;

	for (s = 0; s < VP8_NUM_SEGMENTS; s++) {
		l = lf->level;
		if (f->segment.flags & V4L2_VP8_SEGMENT_FLAG_ENABLED) {
			if (f->segment.flags & V4L2_VP8_SEGMENT_FLAG_DELTA_VALUE_MODE)
				l += f->segment.lf_update[s];
			else
				l = f->segment.lf_update[s];
		}
		level |= clamp(l, 0, 63) << (8 * s);
		ref |= (u8)lf->ref_frm_delta[s] << (8 * s);
		mode |= (u8)lf->mb_mode_delta[s] << (8 * s);
	}

	vdec_write(inst, VDEC_CM, MC_LF, simple ? MC_LF_SIMPLE : MC_LF_NORMAL);
	vdec_write(inst, VDEC_PP, PP_LF_CFG, PP_LF_CFG_EN |
		   (simple ? PP_LF_CFG_SIMPLE : 0) |
		   (lf->flags & V4L2_VP8_LF_ADJ_ENABLE ? PP_LF_CFG_DELTA : 0) |
		   (V4L2_VP8_FRAME_IS_KEY_FRAME(f) ? 0 : PP_LF_CFG_INTER) |
		   FIELD_PREP(PP_LF_CFG_SHARPNESS, lf->sharpness_level));
	vdec_write(inst, VDEC_PP, PP_LF_LEVEL, level);
	vdec_write(inst, VDEC_PP, PP_LF_REF_DELTA, ref);
	vdec_write(inst, VDEC_PP, PP_LF_MODE_DELTA, mode);
}

static void vdec_vp8_start(struct vdec_vp8_direct_inst *inst,
			   const struct vdec_vp8_direct_pic *dst,
			   const struct vdec_vp8_direct_pic *refs)
{
	const struct v4l2_ctrl_vp8_frame *f = inst->frame;
	struct vdec_vp8_direct_hdr *h = &inst->hdr;
	unsigned int luma = vdec_vp8_luma_size(inst);
	dma_addr_t start = min(dst->y, dst->c);
	dma_addr_t end = max(dst->y + luma, dst->c + luma / 2);
	u32 filter;
	unsigned int i;

	vp8_write(inst, VP8_HDR,
		  (inst->clamp_type ? VP8_HDR_CLAMP_TYPE : 0) |
		  (h->seg_en ? VP8_HDR_SEG_EN : 0) |
		  (h->seg_en && h->seg_update_map ? VP8_HDR_SEG_UPDATE_MAP : 0) |
		  FIELD_PREP(VP8_HDR_LOG2_PARTS, h->log2_parts) |
		  (h->no_skip_coeff ? VP8_HDR_NO_SKIP_COEFF : 0));
	vp8_write(inst, VP8_PROBS,
		  FIELD_PREP(VP8_PROBS_SKIP_FALSE, h->prob_skip_false) |
		  FIELD_PREP(VP8_PROBS_INTRA, h->prob_intra) |
		  FIELD_PREP(VP8_PROBS_LAST, h->prob_last) |
		  FIELD_PREP(VP8_PROBS_GF, h->prob_gf));

	vdec_write(inst, VDEC_CM, MC_PIC_W, inst->mbw * VP8_MB_SIZE);
	vdec_write(inst, VDEC_CM, MC_PIC_H, inst->mbh * VP8_MB_SIZE);
	vdec_write(inst, VDEC_CM, MC_PIC_W_MBS2, inst->mbw);
	/* version 0: six-tap filter; 1 and 2: bilinear; 3: full pixel */
	filter = vdec_read(inst, VDEC_CM, MC_FILTER) & MC_FILTER_KEEP;
	if (!f->version)
		filter |= MC_FILTER_SIXTAP;
	if (f->version == 3)
		filter |= MC_FILTER_FULL_PIXEL;
	vdec_write(inst, VDEC_CM, MC_FILTER, filter);
	vdec_write(inst, VDEC_CM, MC_C_OFFSET, (u32)(dst->c - dst->y));
	for (i = 0; i < 3; i++) {
		vdec_write(inst, VDEC_CM, MC_REF_Y(i), refs[i].y);
		vdec_write(inst, VDEC_CM, MC_REF_C(i), refs[i].c);
	}
	/* chroma of the references from their own addresses */
	vdec_mask(inst, VDEC_CM, MC_YC_SEP, MC_YC_SEP_UFO, MC_YC_SEP_EN);
	vdec_write(inst, VDEC_CM, MC_CFG_220, 1);
	vdec_write(inst, VDEC_CM, MC_CFG_250, 1);
	vdec_write(inst, VDEC_CM, MC_Y_ADDR_512, dst->y >> 9);
	vdec_write(inst, VDEC_CM, MC_C_ADDR_256, dst->c >> 8);
	vdec_write(inst, VDEC_CM, MC_PIC_W_MBS, inst->mbw);
	vdec_write(inst, VDEC_CM, MC_PIC_W_MBS_M1, inst->mbw - 1);
	vdec_write(inst, VDEC_CM, MC_PIC_H_MBS_M1, inst->mbh - 1);
	vdec_write(inst, VDEC_CM, MC_CFG_57C, 0);
	/*
	 * Segment ids go to memory, together with other macroblock data; a
	 * frame that keeps the segment map reads them from there. Otherwise
	 * the map lives in a decoder memory that loses it whenever the power
	 * domain is off between two frames. The memory path slows the decoder
	 * down by a third, so frames without segmentation, which leave the map
	 * alone, run without it.
	 */
	inst->top_seg = vdec_read(inst, VDEC_TOP, TOP_SEG_ID_BUF);
	if (h->seg_en) {
		vdec_write(inst, VDEC_TOP, TOP_SEG_ID_BUF, inst->seg_buf.dma_addr);
		vdec_write(inst, VDEC_TOP, TOP_MB_BUF, inst->mb_buf.dma_addr);
		vdec_mask(inst, VDEC_TOP, TOP_CFG_02C, 0, TOP_CFG_02C_SEG_BUF);
	}
	vdec_write(inst, VDEC_CM, MC_VLD_WRAP_ADDR, inst->vld_wrap.dma_addr);
	vdec_write(inst, VDEC_CM, MC_PP_WRAP_ADDR, inst->pp_wrap.dma_addr);
	vdec_mask(inst, VDEC_MISC, MISC_CFG_0F0, 0, MISC_CFG_0F0_EN);
	vdec_vp8_write_lf(inst);
	vdec_write(inst, VDEC_CM, MC_PIC_END, end >> 4);
	vdec_write(inst, VDEC_CM, MC_PIC_START, start >> 4);
	vdec_write(inst, VDEC_CM, MC_CFG_858, 1);
	vdec_mask(inst, VDEC_TOP, TOP_CFG_05C, TOP_CFG_05C_MASK, TOP_CFG_05C_VP8);
	vdec_write(inst, VDEC_MISC, MISC_CRC_EN, MISC_CRC_EN_ON | MISC_CRC_EN_RUN);
	vp8_write(inst, VP8_START, 1);
	vp8_write(inst, VP8_START, 0);
}

static int vdec_vp8_direct_decode(void *h_vdec, struct mtk_vcodec_mem *bs,
				  struct vdec_fb *unused, bool *res_chg)
{
	struct vdec_vp8_direct_inst *inst = h_vdec;
	struct mtk_vcodec_dec_ctx *ctx = inst->ctx;
	struct mtk_video_dec_buf *src_buf, *dst_buf;
	struct vdec_vp8_direct_pic dst, refs[3];
	const struct v4l2_ctrl_vp8_frame *f;
	unsigned int luma, i;
	struct vdec_fb *fb;
	dma_addr_t end;
	u32 done, err, crc[8];
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

	ret = vdec_vp8_get_params(inst, bs);
	if (ret)
		return ret;
	f = inst->frame;

	luma = vdec_vp8_luma_size(inst);
	if (inst->mbw * VP8_MB_SIZE != ctx->picinfo.buf_w ||
	    fb->base_y.size < luma || fb->base_c.size < luma / 2) {
		mtk_vdec_err(ctx, "frame %ux%u does not fit the %ux%u buffers",
			     f->width, f->height, ctx->picinfo.buf_w, ctx->picinfo.buf_h);
		return -EINVAL;
	}

	dst.y = fb->base_y.dma_addr;
	dst.c = fb->base_c.dma_addr;
	vdec_vp8_ref(inst, f->last_frame_ts, &dst, &refs[0]);
	vdec_vp8_ref(inst, f->golden_frame_ts, &dst, &refs[1]);
	vdec_vp8_ref(inst, f->alt_frame_ts, &dst, &refs[2]);

	ret = vdec_vp8_bs_end(inst, bs, &end);
	if (ret)
		return ret;
	vdec_vp8_reset(inst);
	ret = vdec_vp8_hdr(inst, bs, end);
	if (ret)
		goto err_break;
	vdec_vp8_start(inst, &dst, refs);

	ret = mtk_vcodec_wait_for_done_ctx(ctx, MTK_INST_IRQ_RECEIVED,
					   VDEC_VP8_TIMEOUT_MS, MTK_VDEC_CORE);
	/* the other codecs run with the internal memory */
	vdec_mask(inst, VDEC_TOP, TOP_CFG_02C, TOP_CFG_02C_SEG_BUF, 0);
	vdec_write(inst, VDEC_TOP, TOP_SEG_ID_BUF, inst->top_seg);
	done = vp8_read(inst, VP8_DONE);
	err = vp8_read(inst, VP8_ERR_STA);
	if (ret) {
		mtk_vdec_err(ctx, "timeout: done %#x err %#x read %#x/%#x irq %#x",
			     done, err, vdec_read(inst, VDEC_LD, VLD_RPTR),
			     readl(inst->vld2 + VLD_RPTR),
			     vdec_read(inst, VDEC_MISC, MISC_IRQ_STA));
		ret = -ETIMEDOUT;
		goto err_break;
	}
	ctx->decoded_frame_cnt++;
	if (!(done & 1)) {
		mtk_vdec_err(ctx, "frame incomplete: done %#x err %#x", done, err);
		return -EBADMSG;
	}
	for (i = 0; i < ARRAY_SIZE(crc); i++)
		crc[i] = vdec_read(inst, VDEC_MISC, MISC_CRC(i));
	mtk_vdec_debug(ctx, "frame %d done (err %#x), CRC %08x %08x %08x %08x %08x %08x %08x %08x",
		       ctx->decoded_frame_cnt - 1, err, crc[0], crc[1], crc[2], crc[3],
		       crc[4], crc[5], crc[6], crc[7]);

	return 0;

err_break:
	mtk_vdec_direct_hw_break(inst->dev);
	return ret;
}

static void vdec_vp8_free(struct vdec_vp8_direct_inst *inst)
{
	if (inst->mb_buf.va)
		mtk_vcodec_mem_free(inst->ctx, &inst->mb_buf);
	if (inst->seg_buf.va)
		mtk_vcodec_mem_free(inst->ctx, &inst->seg_buf);
	if (inst->pp_wrap.va)
		mtk_vcodec_mem_free(inst->ctx, &inst->pp_wrap);
	if (inst->vld_wrap.va)
		mtk_vcodec_mem_free(inst->ctx, &inst->vld_wrap);
}

static int vdec_vp8_direct_init(struct mtk_vcodec_dec_ctx *ctx)
{
	const struct mtk_vdec_direct_data *data = ctx->dev->vdec_pdata->direct;
	struct vdec_vp8_direct_inst *inst;
	int ret;

	if (!data->vp8_vld || !data->vld2)
		return -EINVAL;

	inst = kzalloc_obj(*inst);
	if (!inst)
		return -ENOMEM;
	inst->ctx = ctx;
	inst->dev = ctx->dev;
	inst->vp8 = mtk_vdec_direct_block(ctx->dev, data->vp8_vld);
	inst->vld2 = mtk_vdec_direct_block(ctx->dev, data->vld2);

	inst->vld_wrap.size = VDEC_VP8_VLD_WRAP_SZ;
	inst->pp_wrap.size = VDEC_VP8_PP_WRAP_SZ;
	inst->seg_buf.size = DIV_ROUND_UP(data->max_width, VP8_MB_SIZE) *
			     ALIGN(DIV_ROUND_UP(data->max_height, VP8_MB_SIZE), 2) * VP8_SEG_MB_BYTES;
	inst->mb_buf.size = inst->seg_buf.size;
	ret = mtk_vcodec_mem_alloc(ctx, &inst->vld_wrap);
	if (!ret)
		ret = mtk_vcodec_mem_alloc(ctx, &inst->pp_wrap);
	if (!ret)
		ret = mtk_vcodec_mem_alloc(ctx, &inst->seg_buf);
	if (!ret)
		ret = mtk_vcodec_mem_alloc(ctx, &inst->mb_buf);
	if (ret) {
		vdec_vp8_free(inst);
		kfree(inst);
		return ret;
	}

	ctx->drv_handle = inst;
	return 0;
}

static void vdec_vp8_direct_deinit(void *h_vdec)
{
	struct vdec_vp8_direct_inst *inst = h_vdec;

	vdec_vp8_free(inst);
	kfree(inst);
}

static int vdec_vp8_direct_get_param(void *h_vdec, enum vdec_get_param_type type,
				     void *out)
{
	struct vdec_vp8_direct_inst *inst = h_vdec;
	struct vdec_pic_info *pic = &inst->ctx->picinfo;
	struct v4l2_rect *crop;

	switch (type) {
	case GET_PARAM_PIC_INFO:
		/* MM21: the tile rows of the decoder follow the picture width */
		pic->buf_w = ALIGN(pic->pic_w, VP8_MB_SIZE);
		pic->buf_h = ALIGN(pic->pic_h, 32);
		pic->fb_sz[0] = pic->buf_w * pic->buf_h;
		pic->fb_sz[1] = pic->fb_sz[0] / 2;
		*(struct vdec_pic_info *)out = *pic;
		break;
	case GET_PARAM_DPB_SIZE:
		/* last, golden and altref frame and the target */
		*(unsigned int *)out = 4;
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

const struct vdec_common_if vdec_vp8_slice_direct_if = {
	.init		= vdec_vp8_direct_init,
	.decode		= vdec_vp8_direct_decode,
	.get_param	= vdec_vp8_direct_get_param,
	.deinit		= vdec_vp8_direct_deinit,
};
