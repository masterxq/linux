// SPDX-License-Identifier: GPL-2.0
/*
 * MPEG-4 Part 2 (Simple and Advanced Simple Profile) and H.263 baseline
 * decoding on MediaTek decoders that the host programs through their
 * registers, without firmware (MT6785).
 *
 * There are no stateless controls for these formats, so this codec runs
 * behind the stateful decoder interface: the host parses the VOL and VOP
 * headers (or the H.263 picture header) in the bitstream buffer, keeps the
 * two anchor pictures and the time stamps for B-VOP direct mode, and hands
 * the pictures out in display order. One hardware run decodes one VOP: the
 * host moves the bitstream reader behind the VOP header and the decoder
 * parses the macroblock data, including video packets, data partitioning
 * and reversible VLC.
 *
 * Global motion compensation is translational in the hardware: S-VOPs with
 * more than one warping point are decoded with the first point only, like
 * the vendor driver does. Static sprites, scalability, shapes other than
 * rectangular, complexity estimation, NEWPRED, reduced resolution VOPs,
 * more than 8 bits per sample and the optional H.263 modes (annexes, PLUSPTYPE)
 * are rejected. Buffers with more than one coded VOP (packed bitstreams)
 * are decoded VOP by VOP, each into its own capture buffer.
 *
 * The decoder predicts from its own reference pictures and writes each
 * picture to the capture buffer (MM21) as a second output, so capture
 * buffers are not held as references. That output path has a deblocking
 * post-filter, which stays off: the pictures are the decoded ones, equal to
 * other decoders within the IDCT accuracy.
 *
 * The decoder loses the macroblock data behind a resync marker in B-VOPs
 * (video packets in I-, P- and S-VOPs work), so B-VOPs with video packets
 * are copied without the packet headers and decoded with resync markers
 * off. See mp4_strip_packets().
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
#define VLD_BARL(n)			(4 * (n))	/* window after skipping n bits */
#define VLD_SHIFT_MAX			24
#define VLD_MP4_ERR_MASK		0x084
#define VLD_MP4_ERR_MASK_VAL		0xfffff600
#define VLD_PIC_CFG			0x088
#define VLD_PIC_TFF			BIT(26)
#define VLD_PIC_CODING_TYPE		GENMASK(22, 21)
#define VLD_PIC_ALT_SCAN		BIT(17)
#define VLD_PIC_FCODE_FWD		GENMASK(15, 12)
#define VLD_PIC_SHORT_HEADER		BIT(12)
#define VLD_PIC_FCODE_BWD		GENMASK(7, 4)
#define VLD_BS_CTRL			0x08c
#define VLD_BS_CTRL_FETCH		BIT(20)
#define VLD_BS_CTRL_RUN			BIT(23)
#define VLD_BS_CTRL_MP4			BIT(27)
#define VLD_SEQ_CFG			0x090
#define VLD_CFG_094			0x094
#define VLD_CFG_LOW			0x1ff
#define VLD_QMAT_CTRL			0x104
#define VLD_QMAT_INTRA			BIT(8)
#define VLD_QMAT_NON_INTRA		BIT(9)
#define VLD_RPTR			0x0b0
#define VLD_VSTART			0x0b4
#define VLD_VEND			0x0b8
#define VLD_MP4_DC_CFG			0x0dc
#define VLD_MP4_DC_CFG_VAL		0xb00000
#define VLD_FETCH_STA			0x0e8
#define VLD_FETCH_STA_READY		BIT(0)
#define VLD_PEEK			0x0f0
#define VLD_PROC_STA			0x0f4
#define VLD_SRST_READER			BIT(8)
#define VLD_RD_ADDR			0x0fc
#define VLD_WPTR			0x110
#define VLD_ERR_STA			0x114
#define VLD_TAB_ADDR			0x260
#define VLD_TAB_DATA			0x264
#define VLD_MP4_VOL			0x1c4
#define VLD_MP4_VOL_RES			GENMASK(15, 0)
#define VLD_MP4_VOL_SHORT_HEADER	BIT(19)
#define VLD_MP4_VOL_RVLC		BIT(20)
#define VLD_MP4_VOL_DATA_PART		BIT(21)
#define VLD_MP4_VOL_INTERLACED		BIT(22)
#define VLD_MP4_VOL_QUANT_TYPE		BIT(24)
#define VLD_MP4_VOL_QPEL		BIT(25)
#define VLD_MP4_VOL_NO_RESYNC		BIT(26)
#define VLD_MP4_CFG_1C8			0x1c8
#define VLD_MP4_CFG_1C8_VAL		0x10001
#define VLD_MP4_INIT			0x1cc
#define VLD_MP4_RUN			0x1d0
#define VLD_MP4_RUN_FETCH		BIT(8)
#define VLD_MP4_RUN_INIT		BIT(0)
#define VLD_MP4_VOP			0x1d4
#define VLD_MP4_VOP_DC_THR		GENMASK(10, 8)
#define VLD_MP4_VOP_QUANT		GENMASK(4, 0)
#define VLD_MP4_H263_FMT		0x1d8
#define VLD_MP4_START			0x1e4
#define VLD_MP4_BWD			0x1e8
#define VLD_MP4_BWD_BASE		0x100
#define VLD_MP4_BWD_NO_MV		BIT(0)
#define VLD_MP4_MODE			0x1fc
#define VLD_MP4_MODE_MPEG4		0x31
#define VLD_MP4_MODE_H263		0x21
#define VLD_MP4_DCAC_ADDR		0x2d4
#define VLD_MP4_DP_ADDR			0x2d8
#define VLD_MP4_DCAC_SIZE		0x2dc
#define VLD_MP4_DCAC_SIZE_BASE		0x6000000
#define VLD_MP4_CFG_2E0			0x2e0
#define VLD_MP4_CFG_2E0_VAL		0x2004310
#define VLD_MP4_CFG_2E8			0x2e8
#define VLD_MP4_CFG_2E8_MPEG4		0x20000
#define VLD_MP4_CFG_2E8_H263		0x80000
#define VLD_MP4_CONCEAL			0x2ec
#define VLD_MP4_CONCEAL_VAL		0xc001080
#define VLD_MP4_CONCEAL_INTER		0x45
#define VLD_MP4_CONCEAL_DATA_PART	BIT(17)
#define VLD_MP4_CONCEAL_INTRA		BIT(16)
#define VLD_MP4_CFG_2F0			0x2f0
#define VLD_MP4_CFG_2F0_VAL		0x80007400
#define VLD_MP4_CFG_2F0_INTRA		0x177ff
#define VLD_MP4_CFG_2F0_MPEG4		BIT(27)
#define VLD_MP4_CFG_2F0_H263		BIT(28)
#define VLD_MP4_CFG_2F4			0x2f4
#define VLD_MP4_CFG_2F4_VAL		0x2800040
#define VLD_MP4_CFG_2F8			0x2f8
#define VLD_MP4_CFG_2F8_VAL		0x8003
#define VLD_MP4_CFG_2F8_H263		0x840
#define VLD_MP4_CFG_304			0x304
#define VLD_MP4_CFG_304_VAL		0x3a93fff
#define VLD_MP4_CFG_304_RUN		BIT(15)
#define VLD_MP4_BITS			0x314	/* bits of macroblock data read */
#define VLD_MP4_STATUS			0x3d0
#define VLD_MP4_STATUS_ERR		BIT(31)
#define VLD_MP4_MBS			0x308
#define VLD_MP4_MBS2			0x30c

/* VDEC_TOP */
#define TOP_PRED_ADDR			0x028
#define TOP_ERR_MASK			0x044
#define TOP_PIC_MB_SIZE_M1		0x068
#define TOP_PIC_PIX_SIZE		0x070
#define TOP_SRAM_ADDR			0x2fc
#define TOP_SRAM_ADDR_NBR		0x1800
#define TOP_SRAM_ADDR_WRITE		BIT(16)
#define TOP_SRAM_DATA			0x300

#define MP4_PRED_SIZE			0x20000

/* motion compensation (VDEC_CM) */
#define MC_FWD_Y			0x000
#define MC_FWD_C			0x004
#define MC_BWD_Y			0x008
#define MC_BWD_C			0x00c
#define MC_FWD_SLOT			0x018
#define MC_DST_SLOT			0x024
#define MC_DST_SLOT_BWD			1
#define MC_DST_SLOT_B			4
#define MC_MB_X				0x028
#define MC_MB_Y				0x02c
#define MC_B_Y				0x09c
#define MC_B_C				0x0a0
#define MC_MP4_CFG_1FC			0x1fc
#define MC_QPEL				0x200
#define MC_ROUNDING			0x204
#define MC_PIC_W			0x208
#define MC_PIC_H			0x20c
#define MC_GMC_POINTS			0x210
#define MC_GMC_LUMA			0x214
#define MC_GMC_CHROMA			0x218
#define MC_MP4_CFG_21C			0x21c
#define MC_MP4_CFG_21C_INIT		0x30000
#define MC_OUT_EN			0x220
#define MC_OUT_Y			0x224
#define MC_OUT_C			0x228
#define MC_OUT_W_MBS			0x22c
#define MC_PP_DBK			0x238	/* post-filter: bit 0 luma, bit 1 chroma */
#define MC_CFG_250			0x250
#define MC_PIC_W_MBS_M1			0x260
#define MC_PIC_H_MBS_M1			0x264
#define MC_CFG_658			0x658
#define MC_CFG_658_MP4			0x10000a
#define MC_CFG_700			0x700
#define MC_CFG_920			0x920
#define MC_CFG_920_MP4			0x1000111
#define MC_PIC_W_MBS			0x980

/* motion vector block (VDEC_AV) */
#define MV_MP4_MV_ADDR			0x20c
#define MV_MP4_CFG_218			0x218
#define MV_MP4_CFG_220			0x220
#define MV_MP4_CFG_220_BASE		0xc00
#define MV_MP4_CFG_220_MPEG4		BIT(13)
#define MV_MP4_CFG_220_RUN		BIT(1)
#define MV_MP4_TR			0x2e0	/* TRB << 16 | TRD */
#define MV_MP4_TR_FIELD			0x2e4	/* the same for fields, +0 */
#define MV_MP4_TR_FIELD_P1		0x2e8	/* fields, +1 */
#define MV_MP4_TR_FIELD_M1		0x2ec	/* fields, -1 */
#define MV_MP4_SCALE			0x2f0	/* TRB / TRD << 16 | (TRD - TRB) / TRD */
#define MV_MP4_SCALE_FIELD		0x2f4
#define MV_MP4_SCALE_FIELD_P1		0x2f8
#define MV_MP4_SCALE_FIELD_M1		0x2fc
#define MV_MP4_SCALE_ONE		BIT(14)
#define MV_MP4_VOL			0x2d8
#define MV_MP4_VOL_QPEL			BIT(0)
#define MV_MP4_VOL_INTERLACED		BIT(1)
#define MV_MP4_GMC_MV			0x2dc
#define MV_MP4_MV_START			0x30c
#define MV_MP4_MV_LAST			0x310
#define MV_MP4_MV_LAST2			0x314

/* post-processing (VDEC_PP) */
#define PP_CFG_03C			0x03c
#define PP_CFG_03C_VAL			BIT(28)

/* control (VDEC_MISC) */
#define MISC_CRC_EN			0x004
#define MISC_CRC(n)			(0x008 + 4 * (n))
#define MISC_SYS_SEL			0x084
#define MISC_SPEC_CON			0x0c8
#define MISC_FUNC_CON			0x0cc
#define MISC_IRQ_STA			0x0a4
#define MISC_DCM_CON			0x0ec
#define MISC_SRST			0x0f4
#define MISC_MP4_CFG_160		0x160
#define MISC_MP4_CFG_160_VAL		0x65ff10
#define MISC_CFG_178			0x178
#define MISC_MP4_CFG_204		0x204
#define MISC_MP4_CFG_204_VAL		0x8

#define MP4_MB_SIZE			16
#define MP4_MAX_MBS			8160
#define MP4_MAX_W			1920
#define MP4_MAX_H			1152

#define VDEC_MPEG4_BS_PAD		576
#define VDEC_MPEG4_POLL_US		10000
#define VDEC_MPEG4_TIMEOUT_MS		500
#define VDEC_MPEG4_FB_QUEUE		8

#define MP4_SC_VOL_MIN			0x20
#define MP4_SC_VOL_MAX			0x2f
#define MP4_SC_VOP			0xb6
#define MP4_SC_VOS			0xb0
#define H263_PSC			0x20	/* first 22 bits of a picture header */

enum { VOP_I, VOP_P, VOP_B, VOP_S };

enum { SPRITE_NONE, SPRITE_STATIC, SPRITE_GMC };

struct vdec_mpeg4_vol {
	bool valid;
	bool short_header;
	unsigned int w, h;
	unsigned int res;
	unsigned int tib;
	u8 verid;
	u8 low_delay;
	u8 interlaced;
	u8 sprite;
	u8 warp_pts;
	u8 warp_acc;
	u8 quant_type;
	u8 load_intra;
	u8 load_inter;
	u8 qpel;
	u8 resync_disable;
	u8 dp;
	u8 rvlc;
	u8 rrv;
	u8 intra[64];
	u8 inter[64];
};

struct vdec_mpeg4_vop {
	u8 type;
	u8 coded;
	u8 rounding;
	u8 dc_thr;
	u8 tff;
	u8 alt;
	u8 quant;
	u8 fcode_f;
	u8 fcode_b;
	u8 h263_fmt;
	unsigned int mtb;
	unsigned int tinc;
	int du0, dv0;
	unsigned int warp_used;
	size_t start;
	unsigned int hdr_bits;
};

struct vdec_mpeg4_br {
	const u8 *d;
	size_t size;
	size_t pos;
	bool over;
};

/**
 * struct vdec_mpeg4_direct_inst - MPEG-4 / H.263 decoder instance
 * @ctx:	decoder context
 * @dev:	decoder device
 * @vol:	current video object layer (or H.263 picture format)
 * @pending_vol: layer of a size change not yet taken over
 * @reported:	the size change of @pending_vol has been reported
 * @mbw:	width in macroblocks
 * @mbh:	height in macroblocks
 * @work:	prediction, DC/AC and data partitioning buffers
 * @mv:		motion vector buffer of the last anchor
 * @nbr:	neighbour information of the last P-VOP for B-VOPs
 * @nbr_words:	size of @nbr in words
 * @pic:	reference pictures of the decoder: anchor slots 0 and 1, B-VOP slot
 * @slot:	anchor slot of the newest anchor
 * @ref_type:	type of the newest anchor
 * @undisplayed: newest anchor not handed out yet
 * @disp:	pictures to hand out, in display order
 * @free:	pictures the decoder does not use any more
 * @sec:	modulo time base of the newest anchor
 * @sec_prev:	modulo time base of the anchor before
 * @t_ref:	time of the newest anchor
 * @t_ref_prev:	time of the anchor before
 * @trd:	distance of the newest two anchors
 * @trb:	distance of the last B-VOP from the older anchor
 * @pp_f:	@trd in fields
 * @pb_f:	@trb in fields
 * @t_frame:	frame period for the field distances, 0 if unknown
 * @anchors:	number of decoded anchors
 * @bs_fix:	copy of a B-VOP without video packet headers
 * @packed_dma:	bitstream buffer with more VOPs to decode, 0 if none
 * @packed_size: size of that buffer
 * @packed_off:	where to look for the next VOP in it
 * @warned:	unsupported feature already reported
 * @err_reported: decoding error already reported
 * @pkt_warned: inexact removal of B-VOP video packets already reported
 */
struct vdec_mpeg4_direct_inst {
	struct mtk_vcodec_dec_ctx *ctx;
	struct mtk_vcodec_dec_dev *dev;

	struct vdec_mpeg4_vol vol;
	struct vdec_mpeg4_vol pending_vol;
	bool reported;
	unsigned int mbw;
	unsigned int mbh;

	struct mtk_vcodec_mem work;
	struct mtk_vcodec_mem mv;
	u32 *nbr;
	unsigned int nbr_words;

	struct mtk_vcodec_mem pic[3];
	unsigned int slot;
	u8 ref_type;
	struct vdec_fb *undisplayed;
	struct vdec_fb *disp[VDEC_MPEG4_FB_QUEUE];
	unsigned int disp_n;
	struct vdec_fb *free[VDEC_MPEG4_FB_QUEUE];
	unsigned int free_n;

	unsigned int sec, sec_prev;
	s64 t_ref, t_ref_prev;
	s32 trd, trb, pp_f, pb_f;
	s32 t_frame;
	unsigned int anchors;
	struct mtk_vcodec_mem bs_fix;
	dma_addr_t packed_dma;
	size_t packed_size;
	size_t packed_off;
	bool warned;
	bool err_reported;
	bool pkt_warned;
};

static inline void vdec_write(struct vdec_mpeg4_direct_inst *inst, int blk, u32 off, u32 val)
{
	writel(val, inst->dev->reg_base[blk] + off);
}

static inline u32 vdec_read(struct vdec_mpeg4_direct_inst *inst, int blk, u32 off)
{
	return readl(inst->dev->reg_base[blk] + off);
}

/* ---------------- headers ---------------- */

static u32 br_u(struct vdec_mpeg4_br *br, unsigned int n)
{
	u32 v = 0;

	while (n--) {
		size_t byte = br->pos >> 3;
		u32 bit = 0;

		if (byte < br->size)
			bit = (br->d[byte] >> (7 - (br->pos & 7))) & 1;
		else
			br->over = true;
		v = v << 1 | bit;
		br->pos++;
	}

	return v;
}

static bool br_marker(struct vdec_mpeg4_br *br)
{
	return br_u(br, 1) == 1;
}

static unsigned int mp4_log2ceil(unsigned int n)
{
	unsigned int b = 1;

	while ((1U << b) < n)
		b++;

	return b;
}

/* quantiser matrix in zigzag order as coded; a zero repeats the last value */
static void mp4_read_matrix(struct vdec_mpeg4_br *br, u8 *m)
{
	unsigned int k;
	u8 last = 0;

	for (k = 0; k < 64; k++) {
		u8 v = br_u(br, 8);

		if (!v)
			break;
		m[k] = v;
		last = v;
	}
	for (; k < 64; k++)
		m[k] = last;
}

static int mp4_parse_vol(struct vdec_mpeg4_direct_inst *inst, struct vdec_mpeg4_br *br,
			 struct vdec_mpeg4_vol *v)
{
	struct mtk_vcodec_dec_ctx *ctx = inst->ctx;
	u32 shape, chroma = 1;

	memset(v, 0, sizeof(*v));
	br_u(br, 1);				/* random_accessible_vol */
	br_u(br, 8);				/* video_object_type_indication */
	v->verid = 1;
	if (br_u(br, 1)) {
		v->verid = br_u(br, 4);
		br_u(br, 3);
	}
	if (br_u(br, 4) == 15)			/* extended PAR */
		br_u(br, 16);
	if (br_u(br, 1)) {			/* vol_control_parameters */
		chroma = br_u(br, 2);
		v->low_delay = br_u(br, 1);
		if (br_u(br, 1)) {		/* vbv_parameters */
			/* bit rate, buffer size, occupancy with markers */
			br_u(br, 16);
			br_u(br, 16);
			br_u(br, 16);
			br_u(br, 15);
			br_u(br, 16);
		}
	}
	shape = br_u(br, 2);
	if (shape) {
		mtk_vdec_err(ctx, "VOL shape %u not supported", shape);
		return -EOPNOTSUPP;
	}
	if (chroma != 1) {
		mtk_vdec_err(ctx, "chroma format %u not supported", chroma);
		return -EOPNOTSUPP;
	}
	br_marker(br);
	v->res = br_u(br, 16);
	if (!br_marker(br) || !v->res) {
		mtk_vdec_err(ctx, "bad VOL time base");
		return -EINVAL;
	}
	v->tib = mp4_log2ceil(v->res);
	if (br_u(br, 1))			/* fixed_vop_rate */
		br_u(br, v->tib);
	br_marker(br);
	v->w = br_u(br, 13);
	br_marker(br);
	v->h = br_u(br, 13);
	br_marker(br);
	v->interlaced = br_u(br, 1);
	br_u(br, 1);				/* obmc_disable */
	v->sprite = br_u(br, v->verid == 1 ? 1 : 2);
	if (v->sprite == SPRITE_STATIC || v->sprite > SPRITE_GMC) {
		mtk_vdec_err(ctx, "static sprites not supported");
		return -EOPNOTSUPP;
	}
	if (v->sprite == SPRITE_GMC) {
		v->warp_pts = br_u(br, 6);
		v->warp_acc = br_u(br, 2);
		if (br_u(br, 1) || v->warp_pts > 3) {
			mtk_vdec_err(ctx, "GMC with %u points or brightness change not supported",
				     v->warp_pts);
			return -EOPNOTSUPP;
		}
	}
	if (br_u(br, 1)) {			/* not_8_bit */
		mtk_vdec_err(ctx, "only 8 bits per sample supported");
		return -EOPNOTSUPP;
	}
	v->quant_type = br_u(br, 1);
	if (v->quant_type) {
		v->load_intra = br_u(br, 1);
		if (v->load_intra)
			mp4_read_matrix(br, v->intra);
		v->load_inter = br_u(br, 1);
		if (v->load_inter)
			mp4_read_matrix(br, v->inter);
	}
	if (v->verid != 1)
		v->qpel = br_u(br, 1);
	if (!br_u(br, 1)) {			/* complexity_estimation_disable */
		mtk_vdec_err(ctx, "complexity estimation not supported");
		return -EOPNOTSUPP;
	}
	v->resync_disable = br_u(br, 1);
	v->dp = br_u(br, 1);
	if (v->dp)
		v->rvlc = br_u(br, 1);
	if (v->verid != 1) {
		if (br_u(br, 1)) {
			mtk_vdec_err(ctx, "NEWPRED not supported");
			return -EOPNOTSUPP;
		}
		v->rrv = br_u(br, 1);
	}
	if (br_u(br, 1)) {
		mtk_vdec_err(ctx, "scalability not supported");
		return -EOPNOTSUPP;
	}
	if (br->over)
		return -EINVAL;
	v->valid = true;

	return 0;
}

/* sprite trajectory: length code of ISO/IEC 14496-2 table B-33, then the value */
static int mp4_warping_mv(struct vdec_mpeg4_br *br)
{
	unsigned int len, c;

	if (!br_u(br, 2)) {
		len = 0;
	} else {
		br->pos -= 2;
		c = br_u(br, 3);
		if (c < 7) {
			len = c - 1;
		} else {
			len = 6;
			while (br_u(br, 1) && len < 14)
				len++;
		}
	}
	if (!len) {
		br_marker(br);
		return 0;
	}
	c = br_u(br, len);
	br_marker(br);

	return c >> (len - 1) ? (int)c : (int)c - (int)(1U << len) + 1;
}

static int mp4_parse_vop(struct vdec_mpeg4_direct_inst *inst, struct vdec_mpeg4_br *br,
			 struct vdec_mpeg4_vop *p)
{
	const struct vdec_mpeg4_vol *v = &inst->vol;
	unsigned int i;

	memset(p, 0, sizeof(*p));
	p->start = (br->pos >> 3) - 4;
	p->type = br_u(br, 2);
	while (br_u(br, 1) && !br->over)
		p->mtb++;
	br_marker(br);
	p->tinc = br_u(br, v->tib);
	br_marker(br);
	p->coded = br_u(br, 1);
	if (!p->coded)
		return br->over ? -EINVAL : 0;
	if (p->type == VOP_P || (p->type == VOP_S && v->sprite == SPRITE_GMC))
		p->rounding = br_u(br, 1);
	if (v->rrv && (p->type == VOP_I || p->type == VOP_P) && br_u(br, 1)) {
		mtk_vdec_err(inst->ctx, "reduced resolution VOP not supported");
		return -EOPNOTSUPP;
	}
	p->dc_thr = br_u(br, 3);
	if (v->interlaced) {
		p->tff = br_u(br, 1);
		p->alt = br_u(br, 1);
	}
	if (p->type == VOP_S) {
		if (v->sprite != SPRITE_GMC) {
			mtk_vdec_err(inst->ctx, "S-VOP without GMC");
			return -EINVAL;
		}
		for (i = 0; i < v->warp_pts; i++) {
			int du = mp4_warping_mv(br);
			int dv = mp4_warping_mv(br);

			if (!i) {
				p->du0 = du;
				p->dv0 = dv;
			}
			if (du || dv)
				p->warp_used++;
		}
	}
	p->quant = br_u(br, 5);
	if (p->type != VOP_I)
		p->fcode_f = br_u(br, 3);
	if (p->type == VOP_B)
		p->fcode_b = br_u(br, 3);
	if (br->over || !p->quant || (p->type != VOP_I && !p->fcode_f) ||
	    (p->type == VOP_B && !p->fcode_b))
		return -EINVAL;
	p->hdr_bits = br->pos - p->start * 8;

	return 0;
}

static const u16 h263_size[][2] = {
	[1] = { 128, 96 }, [2] = { 176, 144 }, [3] = { 352, 288 },
	[4] = { 704, 576 }, [5] = { 1408, 1152 },
};

/* H.263 picture header (short video header) behind the 22 start bits */
static int mp4_parse_h263(struct vdec_mpeg4_direct_inst *inst, struct vdec_mpeg4_br *br,
			  struct vdec_mpeg4_vol *v, struct vdec_mpeg4_vop *p)
{
	struct mtk_vcodec_dec_ctx *ctx = inst->ctx;
	unsigned int fmt;

	memset(p, 0, sizeof(*p));
	p->start = br->pos >> 3;
	br->pos += 22;
	p->tinc = br_u(br, 8);
	br_marker(br);
	br_u(br, 4);		/* zero bit, split screen, document camera, freeze */
	fmt = br_u(br, 3);
	if (fmt < 1 || fmt > 5) {
		mtk_vdec_err(ctx, "H.263 source format %u not supported", fmt);
		return -EOPNOTSUPP;
	}
	p->type = br_u(br, 1) ? VOP_P : VOP_I;
	if (br_u(br, 4)) {
		mtk_vdec_err(ctx, "optional H.263 modes not supported");
		return -EOPNOTSUPP;
	}
	p->quant = br_u(br, 5);
	br_u(br, 1);		/* CPM */
	while (br_u(br, 1) && !br->over)
		br_u(br, 8);	/* PEI / PSUPP */
	if (br->over || !p->quant)
		return -EINVAL;
	p->coded = 1;
	p->h263_fmt = fmt;
	p->fcode_f = p->type == VOP_P;
	p->hdr_bits = br->pos - p->start * 8;

	memset(v, 0, sizeof(*v));
	v->valid = true;
	v->short_header = true;
	v->w = h263_size[fmt][0];
	v->h = h263_size[fmt][1];
	v->low_delay = 1;
	v->verid = 1;

	return 0;
}

static bool mp4_is_h263_psc(const u8 *d, size_t left)
{
	return left >= 4 && !d[0] && !d[1] && (d[2] >> 2) == H263_PSC;
}

/*
 * Walk the start codes of the buffer: take over a VOL into @vol and stop at
 * the first coded VOP. Returns 1 with @vop filled, 0 without a coded VOP.
 */
static int mp4_parse_buffer(struct vdec_mpeg4_direct_inst *inst, const u8 *d, size_t size,
			    size_t from, struct vdec_mpeg4_vol *vol, struct vdec_mpeg4_vop *vop)
{
	struct vdec_mpeg4_br br = { .d = d, .size = size };
	struct vdec_mpeg4_vol saved = inst->vol;
	size_t i;
	int ret;

	for (i = from; i + 4 <= size; i++) {
		if (d[i] || d[i + 1])
			continue;
		if (mp4_is_h263_psc(d + i, size - i) && !inst->vol.valid) {
			br.pos = i * 8;
			return mp4_parse_h263(inst, &br, vol, vop) ?: 1;
		}
		if (mp4_is_h263_psc(d + i, size - i) && inst->vol.short_header) {
			br.pos = i * 8;
			return mp4_parse_h263(inst, &br, vol, vop) ?: 1;
		}
		if (d[i + 2] != 1)
			continue;
		br.pos = (i + 4) * 8;
		if (d[i + 3] >= MP4_SC_VOL_MIN && d[i + 3] <= MP4_SC_VOL_MAX) {
			ret = mp4_parse_vol(inst, &br, vol);
			if (ret)
				return ret;
			/* the VOP syntax depends on the VOL */
			inst->vol = *vol;
		} else if (d[i + 3] == MP4_SC_VOP && inst->vol.valid && !inst->vol.short_header) {
			ret = mp4_parse_vop(inst, &br, vop);
			if (ret)
				return ret;
			if (vop->coded) {
				*vol = inst->vol;
				inst->vol = saved;
				return 1;
			}
		}
		i += 3;
	}
	if (vol->valid)
		inst->vol = saved;

	return 0;
}

/* ---------------- picture queues ---------------- */

static void mp4_push(struct vdec_fb **q, unsigned int *n, struct vdec_fb *fb)
{
	if (fb && *n < VDEC_MPEG4_FB_QUEUE)
		q[(*n)++] = fb;
}

static struct vdec_fb *mp4_pop(struct vdec_fb **q, unsigned int *n)
{
	struct vdec_fb *fb;

	if (!*n)
		return NULL;
	fb = q[0];
	memmove(q, q + 1, --(*n) * sizeof(*q));

	return fb;
}

static void mp4_flush_refs(struct vdec_mpeg4_direct_inst *inst)
{
	if (inst->undisplayed)
		mp4_push(inst->disp, &inst->disp_n, inst->undisplayed);
	mp4_push(inst->free, &inst->free_n, inst->undisplayed);
	inst->undisplayed = NULL;
	inst->anchors = 0;
	inst->slot = 0;
}

/* ---------------- buffers ---------------- */

static unsigned int mp4_dcac_size(unsigned int mbs)
{
	return ALIGN(DIV_ROUND_UP(mbs, 2), 64);
}

static unsigned int mp4_luma_size(struct vdec_mpeg4_direct_inst *inst)
{
	return inst->mbw * MP4_MB_SIZE * ALIGN(inst->mbh * MP4_MB_SIZE, 32);
}

static int mp4_alloc(struct vdec_mpeg4_direct_inst *inst)
{
	unsigned int mbs = inst->mbw * inst->mbh, i;
	int ret;

	if (inst->work.va)
		mtk_vcodec_mem_free(inst->ctx, &inst->work);
	if (inst->mv.va)
		mtk_vcodec_mem_free(inst->ctx, &inst->mv);
	for (i = 0; i < ARRAY_SIZE(inst->pic); i++)
		if (inst->pic[i].va)
			mtk_vcodec_mem_free(inst->ctx, &inst->pic[i]);
	kfree(inst->nbr);
	inst->nbr = NULL;

	/* reference pictures in MM21, chroma behind the luma */
	for (i = 0; i < ARRAY_SIZE(inst->pic); i++) {
		inst->pic[i].size = mp4_luma_size(inst) * 3 / 2;
		ret = mtk_vcodec_mem_alloc(inst->ctx, &inst->pic[i]);
		if (ret)
			return ret;
	}

	/* prediction, DC/AC prediction and data partitioning areas */
	inst->work.size = MP4_PRED_SIZE + mp4_dcac_size(mbs) + SZ_256K + mbs * 64;
	ret = mtk_vcodec_mem_alloc(inst->ctx, &inst->work);
	if (ret)
		return ret;
	/* 16 bytes per macroblock */
	inst->mv.size = ALIGN(mbs * 16, SZ_4K);
	ret = mtk_vcodec_mem_alloc(inst->ctx, &inst->mv);
	if (ret)
		return ret;
	/* 4 bits per macroblock, read out in 16-word bursts */
	inst->nbr_words = ALIGN(DIV_ROUND_UP(mbs, 8), 16);
	inst->nbr = kcalloc(inst->nbr_words, sizeof(u32), GFP_KERNEL);
	if (!inst->nbr)
		return -ENOMEM;

	return 0;
}

/* ---------------- hardware ---------------- */

/* weights of direct mode: b / d and (d - b) / d in units of 1/16384 */
static u32 mp4_scale(s32 b, s32 d)
{
	if (!d)
		return 0;

	return (u32)(div_s64((s64)b * 16384, d) & 0xffff) << 16 |
	       (div_s64((s64)(d - b) * 16384, d) & 0xffff);
}

static u32 mp4_pair(s32 hi, s32 lo)
{
	return (u32)(hi & 0xffff) << 16 | (lo & 0xffff);
}

static void mp4_times(struct vdec_mpeg4_direct_inst *inst, const struct vdec_mpeg4_vop *p)
{
	const struct vdec_mpeg4_vol *v = &inst->vol;
	s64 t;

	if (v->short_header)
		return;
	if (p->type != VOP_B) {
		inst->sec_prev = inst->sec;
		inst->sec += p->mtb;
		t = (s64)inst->sec * v->res + p->tinc;
		inst->t_ref_prev = inst->t_ref;
		inst->t_ref = t;
		inst->trd = inst->anchors ? t - inst->t_ref_prev : 0;
		return;
	}
	t = (s64)(inst->sec_prev + p->mtb) * v->res + p->tinc;
	inst->trb = t - inst->t_ref_prev;
	if (v->interlaced) {
		s32 tf, h;
		s64 base;

		if (!inst->t_frame)
			inst->t_frame = inst->trb ?: 1;
		tf = inst->t_frame;
		h = tf / 2;
		base = div_s64(inst->t_ref_prev + h, tf);
		inst->pb_f = 2 * (div_s64(t + h, tf) - base);
		inst->pp_f = 2 * (div_s64(inst->t_ref + h, tf) - base);
	}
}

static void mp4_reset(struct vdec_mpeg4_direct_inst *inst)
{
	vdec_write(inst, VDEC_LD, VDEC_VLD_SRST, VLD_SRST_READER | VDEC_VLD_SRST_RESET);
	vdec_write(inst, VDEC_MISC, MISC_SPEC_CON, 0);
	vdec_write(inst, VDEC_MISC, MISC_FUNC_CON, 0);
	vdec_write(inst, VDEC_MISC, MISC_CFG_178, 0);
	vdec_write(inst, VDEC_MISC, MISC_SRST, 0);
	vdec_write(inst, VDEC_MISC, MISC_SYS_SEL, 1);
	vdec_write(inst, VDEC_LD, VDEC_VLD_SRST, 0);
	mtk_vcodec_write_vdecsys(inst->ctx, VDEC_SYS_DCM_CON, 1);
	vdec_write(inst, VDEC_MISC, MISC_DCM_CON, 1);
	vdec_write(inst, VDEC_MISC, MISC_MP4_CFG_160, MISC_MP4_CFG_160_VAL);
	vdec_write(inst, VDEC_CM, MC_MP4_CFG_21C, MC_MP4_CFG_21C_INIT);
	vdec_write(inst, VDEC_MISC, MISC_CRC_EN, 1);
}

static void mp4_write_times(struct vdec_mpeg4_direct_inst *inst)
{
	s32 trb = inst->trb, trd = inst->trd, pb = inst->pb_f, pp = inst->pp_f;

	if (inst->vol.short_header) {
		trb = 0;
		trd = 0;
		pb = 0;
		pp = 0;
	}
	vdec_write(inst, VDEC_AV, MV_MP4_TR, mp4_pair(trb, trd));
	vdec_write(inst, VDEC_AV, MV_MP4_TR_FIELD, mp4_pair(pb, pp));
	vdec_write(inst, VDEC_AV, MV_MP4_TR_FIELD_P1, mp4_pair(pb + 1, pp + 1));
	vdec_write(inst, VDEC_AV, MV_MP4_TR_FIELD_M1, mp4_pair(pb - 1, pp - 1));
	vdec_write(inst, VDEC_AV, MV_MP4_SCALE, mp4_scale(trb, trd));
	vdec_write(inst, VDEC_AV, MV_MP4_SCALE_FIELD, mp4_scale(pb, pp));
	vdec_write(inst, VDEC_AV, MV_MP4_SCALE_FIELD_P1, mp4_scale(pb + 1, pp + 1));
	vdec_write(inst, VDEC_AV, MV_MP4_SCALE_FIELD_M1, mp4_scale(pb - 1, pp - 1));
}

/* global motion: the hardware takes the first warping point */
static void mp4_gmc(const struct vdec_mpeg4_vol *v, const struct vdec_mpeg4_vop *p,
		    u32 *luma, u32 *chroma, u32 *mv)
{
	unsigned int sh = v->warp_acc + 1;
	s32 xu = p->du0 * (2 << v->warp_acc), xv = p->dv0 * (2 << v->warp_acc);

	*luma = mp4_pair(2 * p->du0, 2 * p->dv0);
	*chroma = mp4_pair(((xu & ~3) | ((xu & 2) << 1)) >> sh,
			   ((xv & ~3) | ((xv & 2) << 1)) >> sh);
	*mv = mp4_pair(p->du0, p->dv0);
}

static void mp4_write_qmat(struct vdec_mpeg4_direct_inst *inst, const u8 *m, u32 base, u32 sel)
{
	unsigned int i;

	for (i = 0; i < 64; i += 4) {
		vdec_write(inst, VDEC_LD, VLD_TAB_ADDR, base + i);
		vdec_write(inst, VDEC_LD, VLD_TAB_DATA,
			   m[i] << 24 | m[i + 1] << 16 | m[i + 2] << 8 | m[i + 3]);
	}
	vdec_write(inst, VDEC_LD, VLD_QMAT_CTRL, sel);
}

static void mp4_program(struct vdec_mpeg4_direct_inst *inst, const struct vdec_mpeg4_vop *p,
			struct vdec_fb *dst)
{
	const struct vdec_mpeg4_vol *v = &inst->vol;
	unsigned int w = inst->mbw * MP4_MB_SIZE, h = inst->mbh * MP4_MB_SIZE;
	unsigned int mbs = inst->mbw * inst->mbh;
	u32 mv = inst->mv.dma_addr >> 4, cfg, dcac = mp4_dcac_size(mbs);
	u32 g_luma = 0, g_chroma = 0, g_mv = 0;
	u32 luma = mp4_luma_size(inst);
	unsigned int i;
	dma_addr_t pred = inst->work.dma_addr;

	if (p->type == VOP_S)
		mp4_gmc(v, p, &g_luma, &g_chroma, &g_mv);
	mp4_write_times(inst);
	vdec_write(inst, VDEC_AV, MV_MP4_VOL,
		   (v->qpel ? MV_MP4_VOL_QPEL : 0) | (v->interlaced ? MV_MP4_VOL_INTERLACED : 0));
	vdec_write(inst, VDEC_AV, MV_MP4_GMC_MV, g_mv);
	vdec_write(inst, VDEC_AV, MV_MP4_MV_ADDR, mv);
	vdec_write(inst, VDEC_AV, MV_MP4_CFG_218, 0);
	vdec_write(inst, VDEC_AV, MV_MP4_CFG_220, MV_MP4_CFG_220_BASE | MV_MP4_CFG_220_MPEG4);
	vdec_write(inst, VDEC_AV, MV_MP4_MV_START, mv);
	vdec_write(inst, VDEC_AV, MV_MP4_MV_LAST, mv + mbs - 1);
	vdec_write(inst, VDEC_AV, MV_MP4_MV_LAST2, mv + mbs - 1);
	vdec_write(inst, VDEC_AV, MV_MP4_CFG_220,
		   MV_MP4_CFG_220_BASE | MV_MP4_CFG_220_MPEG4 | MV_MP4_CFG_220_RUN);

	vdec_write(inst, VDEC_CM, MC_MP4_CFG_1FC, 1);
	vdec_write(inst, VDEC_CM, MC_QPEL, v->qpel);
	vdec_write(inst, VDEC_CM, MC_ROUNDING, p->rounding);
	vdec_write(inst, VDEC_CM, MC_PIC_W, w);
	vdec_write(inst, VDEC_CM, MC_PIC_H, h);
	vdec_write(inst, VDEC_CM, MC_MP4_CFG_21C, 0);
	if (p->type == VOP_S) {
		vdec_write(inst, VDEC_CM, MC_GMC_POINTS, p->warp_used);
		vdec_write(inst, VDEC_CM, MC_GMC_LUMA, g_luma);
		vdec_write(inst, VDEC_CM, MC_GMC_CHROMA, g_chroma);
	}

	/*
	 * The decoder keeps its references in its own pictures: the anchors
	 * alternate between slots 0 and 1, a B-VOP goes to the B slot. The
	 * picture is written to the capture buffer as a second output.
	 */
	for (i = 0; i < ARRAY_SIZE(inst->pic); i++) {
		static const u32 reg[3][2] = {
			{ MC_FWD_Y, MC_FWD_C }, { MC_BWD_Y, MC_BWD_C }, { MC_B_Y, MC_B_C },
		};

		vdec_write(inst, VDEC_CM, reg[i][0], inst->pic[i].dma_addr >> 9);
		vdec_write(inst, VDEC_CM, reg[i][1], (inst->pic[i].dma_addr + luma) >> 8);
	}
	if (p->type == VOP_B) {
		/* the older anchor is the forward reference */
		vdec_write(inst, VDEC_CM, MC_DST_SLOT, MC_DST_SLOT_B);
		vdec_write(inst, VDEC_CM, MC_FWD_SLOT, !inst->slot);
	} else {
		vdec_write(inst, VDEC_CM, MC_DST_SLOT, !inst->slot);
		vdec_write(inst, VDEC_CM, MC_FWD_SLOT, inst->slot);
	}
	vdec_write(inst, VDEC_CM, MC_CFG_920, MC_CFG_920_MP4);
	vdec_write(inst, VDEC_CM, MC_PIC_W_MBS, inst->mbw);
	vdec_write(inst, VDEC_PP, PP_CFG_03C, PP_CFG_03C_VAL);
	vdec_write(inst, VDEC_CM, MC_OUT_EN, 1);
	vdec_write(inst, VDEC_CM, MC_OUT_Y, dst->base_y.dma_addr >> 9);
	vdec_write(inst, VDEC_CM, MC_OUT_C, dst->base_c.dma_addr >> 8);
	vdec_write(inst, VDEC_CM, MC_OUT_W_MBS, inst->mbw);
	vdec_write(inst, VDEC_CM, MC_CFG_700, 0);
	vdec_write(inst, VDEC_CM, MC_PP_DBK, 0);
	vdec_write(inst, VDEC_CM, MC_CFG_250, 0);
	vdec_write(inst, VDEC_CM, MC_CFG_658, MC_CFG_658_MP4);
	vdec_write(inst, VDEC_CM, MC_PIC_W_MBS_M1, inst->mbw - 1);
	vdec_write(inst, VDEC_CM, MC_PIC_H_MBS_M1, inst->mbh - 1);

	if (v->load_intra)
		mp4_write_qmat(inst, v->intra, 0x00, VLD_QMAT_INTRA);
	if (v->load_inter)
		mp4_write_qmat(inst, v->inter, 0x40, VLD_QMAT_NON_INTRA);

	/* B-VOP whose backward anchor has no motion vectors (I- or S-VOP) */
	cfg = VLD_MP4_BWD_BASE;
	if (p->type == VOP_B && (inst->ref_type == VOP_I || inst->ref_type == VOP_S))
		cfg |= VLD_MP4_BWD_NO_MV;
	vdec_write(inst, VDEC_LD, VLD_MP4_BWD, cfg);
	vdec_write(inst, VDEC_LD, VLD_MP4_CFG_1C8, VLD_MP4_CFG_1C8_VAL);

	cfg = FIELD_PREP(VLD_PIC_CODING_TYPE, p->type) |
	      FIELD_PREP(VLD_PIC_FCODE_FWD, p->fcode_f) |
	      FIELD_PREP(VLD_PIC_FCODE_BWD, p->fcode_b);
	if (p->tff)
		cfg |= VLD_PIC_TFF;
	if (p->alt)
		cfg |= VLD_PIC_ALT_SCAN;
	if (v->short_header)
		cfg |= VLD_PIC_SHORT_HEADER;
	vdec_write(inst, VDEC_LD, VLD_PIC_CFG, cfg);

	cfg = FIELD_PREP(VLD_MP4_VOL_RES, v->res);
	if (v->short_header)
		cfg |= VLD_MP4_VOL_SHORT_HEADER | VLD_MP4_VOL_NO_RESYNC;
	if (v->rvlc && p->type != VOP_B)
		cfg |= VLD_MP4_VOL_RVLC;
	if (v->dp)
		cfg |= VLD_MP4_VOL_DATA_PART;
	if (v->interlaced)
		cfg |= VLD_MP4_VOL_INTERLACED;
	if (v->quant_type)
		cfg |= VLD_MP4_VOL_QUANT_TYPE;
	if (v->qpel)
		cfg |= VLD_MP4_VOL_QPEL;
	if (v->resync_disable || p->type == VOP_B)
		cfg |= VLD_MP4_VOL_NO_RESYNC;
	vdec_write(inst, VDEC_LD, VLD_MP4_VOL, cfg);
	vdec_write(inst, VDEC_LD, VLD_MP4_H263_FMT, p->h263_fmt);
	vdec_write(inst, VDEC_LD, VLD_MP4_VOP, FIELD_PREP(VLD_MP4_VOP_DC_THR, p->dc_thr) |
		   FIELD_PREP(VLD_MP4_VOP_QUANT, p->quant));
	vdec_write(inst, VDEC_LD, VLD_SEQ_CFG, VLD_CFG_LOW);
	vdec_write(inst, VDEC_LD, VLD_CFG_094, VLD_CFG_LOW);
	vdec_write(inst, VDEC_TOP, TOP_PIC_MB_SIZE_M1, (inst->mbw - 1) | (inst->mbh - 1) << 16);
	vdec_write(inst, VDEC_TOP, TOP_PIC_PIX_SIZE, (w + 15) | (h + 15) << 16);
	vdec_write(inst, VDEC_TOP, TOP_PRED_ADDR, pred);
	vdec_write(inst, VDEC_LD, VLD_MP4_DCAC_ADDR, pred + MP4_PRED_SIZE);
	vdec_write(inst, VDEC_LD, VLD_MP4_DP_ADDR, pred + MP4_PRED_SIZE + dcac);
	vdec_write(inst, VDEC_LD, VLD_MP4_DCAC_SIZE, VLD_MP4_DCAC_SIZE_BASE | dcac >> 4);
	vdec_write(inst, VDEC_LD, VLD_MP4_MODE,
		   v->short_header ? VLD_MP4_MODE_H263 : VLD_MP4_MODE_MPEG4);
	vdec_write(inst, VDEC_LD, VLD_MP4_CFG_2E0, VLD_MP4_CFG_2E0_VAL);
	vdec_write(inst, VDEC_LD, VLD_MP4_CFG_2F4, VLD_MP4_CFG_2F4_VAL);
	vdec_write(inst, VDEC_LD, VLD_MP4_CFG_2F8, VLD_MP4_CFG_2F8_VAL);
	vdec_write(inst, VDEC_LD, VLD_MP4_CFG_2F8, VLD_MP4_CFG_2F8_VAL |
		   (v->short_header ? VLD_MP4_CFG_2F8_H263 : 0));
	if (v->short_header)
		vdec_write(inst, VDEC_LD, VLD_MP4_CFG_2F4, VLD_MP4_CFG_2F4_VAL);
	vdec_write(inst, VDEC_LD, VLD_MP4_CFG_304, VLD_MP4_CFG_304_VAL);
	vdec_write(inst, VDEC_LD, VLD_MP4_MBS, mbs << 16);
	vdec_write(inst, VDEC_LD, VLD_MP4_MBS2, mbs | mbs << 16);
	vdec_write(inst, VDEC_LD, VLD_MP4_CFG_2E8, 0);
	vdec_write(inst, VDEC_LD, VLD_MP4_RUN, VLD_MP4_RUN_FETCH);
}

/* ---------------- video packets of B-VOPs ---------------- */

struct mp4_bw {
	u8 *d;
	size_t n;
	u32 acc;
	unsigned int bits;
};

static void bw_put(struct mp4_bw *w, u32 v, unsigned int n)
{
	w->acc = w->acc << n | (v & GENMASK(n - 1, 0));
	w->bits += n;
	while (w->bits >= 8) {
		w->bits -= 8;
		w->d[w->n++] = w->acc >> w->bits;
	}
}

static unsigned int mp4_bit(const u8 *d, size_t pos)
{
	return d[pos >> 3] >> (7 - (pos & 7)) & 1;
}

/* copy the bits [from, to) of @d */
static void bw_copy(struct mp4_bw *w, const u8 *d, size_t from, size_t to)
{
	for (; from < to && (from & 7); from++)
		bw_put(w, mp4_bit(d, from), 1);
	for (; from + 8 <= to; from += 8)
		bw_put(w, d[from >> 3], 8);
	for (; from < to; from++)
		bw_put(w, mp4_bit(d, from), 1);
}

/*
 * Length of a resync marker at byte @i (zeros and the final one) or 0. A
 * B-VOP marker has 15 + max(f_code) zeros, at least 16 (first edition of
 * ISO/IEC 14496-2) or at least 17 (later editions and the encoders); in
 * front of it the stuffing of the previous packet, a zero and up to seven
 * ones, ends the byte.
 */
static unsigned int mp4_marker(const u8 *d, size_t i, size_t end, size_t from,
			       unsigned int zmin)
{
	size_t pos = i * 8, st;
	unsigned int z = 0;

	if (i + 3 > end || d[i] || d[i + 1])
		return 0;
	while (z < 23 && pos + z < end * 8 && !mp4_bit(d, pos + z))
		z++;
	if (z < zmin || z > 22 || pos + z >= end * 8)
		return 0;
	for (st = pos - 1; st > from && pos - st <= 8 && mp4_bit(d, st); st--)
		;
	if (mp4_bit(d, st) || pos - st > 8)
		return 0;

	return z + 1;
}

/*
 * Copy the bitstream from @off with the video packet headers of the B-VOP
 * at @p removed: stuffing, resync marker, macroblock number, quantiser and
 * the optional header extension. Returns the size of the copy, 0 if the
 * VOP has no video packets. This is exact when the packets start at
 * macroblock rows (the motion vector predictors of B-VOPs restart there
 * anyway) and carry the quantiser in effect, as the encoders write them;
 * otherwise the VOP is still decoded and the deviation reported once.
 */
static size_t mp4_strip_packets(struct vdec_mpeg4_direct_inst *inst, struct mtk_vcodec_mem *bs,
				const struct vdec_mpeg4_vop *p, size_t off)
{
	const struct vdec_mpeg4_vol *v = &inst->vol;
	const u8 *d = bs->va;
	size_t i, mb = DIV_ROUND_UP(p->start * 8 + p->hdr_bits, 8), end = bs->size;
	size_t rpos = off * 8, need;
	unsigned int zmin = 15 + max_t(unsigned int, p->fcode_f, p->fcode_b);
	unsigned int len, mbn_bits = mp4_log2ceil(inst->mbw * inst->mbh);
	struct vdec_mpeg4_br br = { .d = d };
	struct mp4_bw w = {};
	bool found = false, exact = true;

	if (p->type != VOP_B || v->resync_disable)
		return 0;
	/* the VOP ends at the next start code */
	for (i = mb; i + 3 <= bs->size; i++) {
		if (!d[i] && !d[i + 1] && d[i + 2] == 1) {
			end = i;
			break;
		}
	}
	br.size = end;
	for (i = mb + 1; i < end && !found; i++)
		found = mp4_marker(d, i, end, mb * 8, zmin);
	if (!found)
		return 0;

	need = ALIGN(bs->size - off + VDEC_MPEG4_BS_PAD + 64, SZ_4K);
	if (inst->bs_fix.va && inst->bs_fix.size < need)
		mtk_vcodec_mem_free(inst->ctx, &inst->bs_fix);
	if (!inst->bs_fix.va) {
		inst->bs_fix.size = need;
		if (mtk_vcodec_mem_alloc(inst->ctx, &inst->bs_fix))
			return 0;
	}

	w.d = inst->bs_fix.va;
	for (i = mb + 1; i < end; i++) {
		size_t st;

		len = mp4_marker(d, i, end, rpos, zmin);
		if (!len)
			continue;
		for (st = i * 8 - 1; mp4_bit(d, st); st--)
			;
		bw_copy(&w, d, rpos, st);
		br.pos = i * 8 + len;
		if (br_u(&br, mbn_bits) % inst->mbw || br_u(&br, 5) != p->quant)
			exact = false;
		if (br_u(&br, 1)) {
			/* header extension: time, type, intra_dc_vlc_thr, f_codes */
			while (br_u(&br, 1) && !br.over)
				;
			br_marker(&br);
			br_u(&br, v->tib);
			br_marker(&br);
			br_u(&br, 2 + 3 + 3 + 3);
		}
		rpos = br.pos;
		i = rpos / 8;
	}
	bw_copy(&w, d, rpos, bs->size * 8);
	if (w.bits)
		bw_put(&w, 0, 8 - w.bits);

	if (!exact && !inst->pkt_warned) {
		mtk_vdec_err(inst->ctx, "B-VOP video packets inside a row or with a new quantiser");
		inst->pkt_warned = true;
	}

	return w.n;
}

static int mp4_decode_vop(struct vdec_mpeg4_direct_inst *inst, struct mtk_vcodec_mem *bs,
			  const struct vdec_mpeg4_vop *p, struct vdec_fb *dst)
{
	struct mtk_vcodec_dec_ctx *ctx = inst->ctx;
	struct vb2_v4l2_buffer *src = v4l2_m2m_next_src_buf(ctx->m2m_ctx);
	size_t plane = src ? vb2_plane_size(&src->vb2_buf, 0) : 0;
	size_t off = ALIGN_DOWN(p->start, 64), pad, fixed;
	dma_addr_t start = bs->dma_addr + off, end;
	unsigned int skip, n, i;
	u32 val, mb_x, mb_y, crc[8];
	int ret;

	/* the reader runs ahead: zeros behind the data, the write pointer behind them */
	pad = ALIGN(bs->size + VDEC_MPEG4_BS_PAD, 64) - bs->size;
	if (bs->size + pad > plane) {
		mtk_vdec_err(ctx, "no room behind the bitstream (%zu of %zu)", bs->size, plane);
		return -EINVAL;
	}
	memset(bs->va + bs->size, 0, pad);
	end = bs->dma_addr + bs->size + VDEC_MPEG4_BS_PAD;

	fixed = mp4_strip_packets(inst, bs, p, off);
	if (fixed) {
		mtk_vdec_debug(ctx, "B-VOP video packets removed: %zu -> %zu bytes",
			       bs->size - off, fixed);
		memset(inst->bs_fix.va + fixed, 0, inst->bs_fix.size - fixed);
		start = inst->bs_fix.dma_addr;
		end = start + fixed + VDEC_MPEG4_BS_PAD;
	}

	mp4_reset(inst);
	mp4_program(inst, p, dst);

	vdec_write(inst, VDEC_LD, VLD_VSTART, start >> 6);
	vdec_write(inst, VDEC_LD, VLD_VEND, ALIGN(end, 64) >> 6);
	vdec_write(inst, VDEC_LD, VLD_RPTR, start);
	vdec_write(inst, VDEC_LD, VLD_WPTR, end);
	vdec_write(inst, VDEC_LD, VDEC_VLD_SRST, VLD_SRST_READER);
	vdec_write(inst, VDEC_LD, VDEC_VLD_SRST, 0);
	vdec_write(inst, VDEC_LD, VLD_MP4_RUN, VLD_MP4_RUN_FETCH);
	vdec_write(inst, VDEC_LD, VLD_BS_CTRL, VLD_BS_CTRL_FETCH);
	ret = readl_poll_timeout(inst->dev->reg_base[VDEC_LD] + VLD_FETCH_STA, val,
				 val & VLD_FETCH_STA_READY, 1, VDEC_MPEG4_POLL_US);
	if (ret) {
		mtk_vdec_err(ctx, "bitstream fetch timeout: %#x", val);
		goto err_break;
	}
	vdec_write(inst, VDEC_LD, VLD_BS_CTRL, VLD_BS_CTRL_RUN);

	/* up to the start code of the VOP, check it, then behind the header */
	for (skip = (p->start - off) * 8; skip; skip -= n) {
		n = min(skip, VLD_SHIFT_MAX);
		vdec_read(inst, VDEC_LD, VLD_BARL(n));
	}
	val = vdec_read(inst, VDEC_LD, VLD_PEEK);
	if (inst->vol.short_header ? (val >> 10) != H263_PSC : val != (0x100 | MP4_SC_VOP)) {
		mtk_vdec_err(ctx, "no picture start code at %zu: %#x", p->start, val);
		ret = -EINVAL;
		goto err_break;
	}
	for (skip = p->hdr_bits; skip; skip -= n) {
		n = min(skip, VLD_SHIFT_MAX);
		vdec_read(inst, VDEC_LD, VLD_BARL(n));
	}

	vdec_write(inst, VDEC_LD, VLD_MP4_INIT, 1);
	vdec_write(inst, VDEC_LD, VLD_MP4_INIT, 0);
	vdec_write(inst, VDEC_LD, VLD_MP4_RUN, VLD_MP4_RUN_INIT);
	vdec_write(inst, VDEC_LD, VLD_MP4_RUN, 0);
	vdec_write(inst, VDEC_LD, VLD_BS_CTRL, VLD_BS_CTRL_MP4);
	vdec_write(inst, VDEC_LD, VLD_MP4_RUN, 0);
	vdec_write(inst, VDEC_TOP, TOP_ERR_MASK, 0xffffffff);
	vdec_write(inst, VDEC_LD, VLD_MP4_ERR_MASK, VLD_MP4_ERR_MASK_VAL);
	vdec_write(inst, VDEC_LD, VLD_MP4_DC_CFG, VLD_MP4_DC_CFG_VAL);

	if (p->type == VOP_B) {
		for (i = 0; i < inst->nbr_words; i++) {
			vdec_write(inst, VDEC_TOP, TOP_SRAM_ADDR,
				   TOP_SRAM_ADDR_WRITE | (TOP_SRAM_ADDR_NBR + i));
			vdec_write(inst, VDEC_TOP, TOP_SRAM_DATA, inst->nbr[i]);
		}
	}

	val = VLD_MP4_CONCEAL_VAL;
	if (p->type != VOP_I)
		val |= VLD_MP4_CONCEAL_INTER;
	vdec_write(inst, VDEC_LD, VLD_MP4_CONCEAL, val);
	if (p->type == VOP_I)
		vdec_write(inst, VDEC_LD, VLD_MP4_CFG_2F0,
			   VLD_MP4_CFG_2F0_VAL | VLD_MP4_CFG_2F0_INTRA);
	vdec_write(inst, VDEC_LD, VLD_MP4_CFG_2E8, inst->vol.short_header ?
		   VLD_MP4_CFG_2E8_H263 : VLD_MP4_CFG_2E8_MPEG4);
	vdec_write(inst, VDEC_LD, VLD_MP4_CFG_2F0, VLD_MP4_CFG_2F0_VAL |
		   (p->type == VOP_I ? VLD_MP4_CFG_2F0_INTRA : 0) |
		   (inst->vol.short_header ? VLD_MP4_CFG_2F0_H263 : VLD_MP4_CFG_2F0_MPEG4));
	vdec_write(inst, VDEC_LD, VLD_MP4_CFG_304, VLD_MP4_CFG_304_VAL | VLD_MP4_CFG_304_RUN);
	if (inst->vol.dp) {
		val |= VLD_MP4_CONCEAL_DATA_PART;
		vdec_write(inst, VDEC_LD, VLD_MP4_CONCEAL, val);
	}
	if (p->type == VOP_I)
		vdec_write(inst, VDEC_LD, VLD_MP4_CONCEAL, val | VLD_MP4_CONCEAL_INTRA);

	vdec_write(inst, VDEC_MISC, MISC_MP4_CFG_204, MISC_MP4_CFG_204_VAL);
	vdec_write(inst, VDEC_LD, VLD_MP4_START, 1);
	vdec_write(inst, VDEC_LD, VLD_MP4_START, 0);

	ret = mtk_vcodec_wait_for_done_ctx(ctx, MTK_INST_IRQ_RECEIVED,
					   VDEC_MPEG4_TIMEOUT_MS, MTK_VDEC_CORE);
	if (ret) {
		mtk_vdec_err(ctx, "timeout at MB %u,%u: read %#x proc %#x err %#x irq %#x",
			     vdec_read(inst, VDEC_CM, MC_MB_X), vdec_read(inst, VDEC_CM, MC_MB_Y),
			     vdec_read(inst, VDEC_LD, VLD_RD_ADDR),
			     vdec_read(inst, VDEC_LD, VLD_PROC_STA),
			     vdec_read(inst, VDEC_LD, VLD_ERR_STA),
			     vdec_read(inst, VDEC_MISC, MISC_IRQ_STA));
		ret = -ETIMEDOUT;
		goto err_break;
	}

	mb_x = vdec_read(inst, VDEC_CM, MC_MB_X);
	mb_y = vdec_read(inst, VDEC_CM, MC_MB_Y);
	if (mb_x != inst->mbw - 1 || mb_y != inst->mbh - 1)
		mtk_vdec_err(ctx, "VOP incomplete: last MB %u,%u", mb_x, mb_y);
	/* on errors the decoder conceals the rest of the VOP */
	val = vdec_read(inst, VDEC_LD, VLD_MP4_STATUS);
	if (val & VLD_MP4_STATUS_ERR) {
		u32 bits = vdec_read(inst, VDEC_LD, VLD_MP4_BITS);

		if (!inst->err_reported)
			mtk_vdec_err(ctx, "VOP %d: bitstream error after %u bits, concealed (%#x)",
				     ctx->decoded_frame_cnt, bits, val);
		else
			mtk_vdec_debug(ctx, "VOP %d: bitstream error after %u bits, concealed (%#x)",
				       ctx->decoded_frame_cnt, bits, val);
		inst->err_reported = true;
	}

	/* neighbour information of a P-VOP for the B-VOPs that follow */
	if (p->type == VOP_P) {
		for (i = 0; i < inst->nbr_words; i++) {
			vdec_write(inst, VDEC_TOP, TOP_SRAM_ADDR, TOP_SRAM_ADDR_NBR + i);
			inst->nbr[i] = vdec_read(inst, VDEC_TOP, TOP_SRAM_DATA);
		}
	}

	for (i = 0; i < ARRAY_SIZE(crc); i++)
		crc[i] = vdec_read(inst, VDEC_MISC, MISC_CRC(i));
	mtk_vdec_debug(ctx, "VOP %d type %u done, CRC %08x %08x %08x %08x %08x %08x %08x %08x, %u bits, status %#x",
		       ctx->decoded_frame_cnt, p->type, crc[0], crc[1], crc[2], crc[3],
		       crc[4], crc[5], crc[6], crc[7], vdec_read(inst, VDEC_LD, VLD_MP4_BITS),
		       vdec_read(inst, VDEC_LD, VLD_MP4_STATUS));

	return 0;

err_break:
	mtk_vdec_direct_hw_break(inst->dev);
	return ret;
}

/*
 * Packed bitstreams carry a P- and a B-VOP in one buffer. After decoding
 * the VOP at @p, look for another coded VOP in the buffer; if there is one,
 * the decoder returns -EAGAIN and is called again with the same buffer.
 */
static bool mp4_more_vops(struct vdec_mpeg4_direct_inst *inst, struct mtk_vcodec_mem *bs,
			  const struct vdec_mpeg4_vop *p)
{
	struct vdec_mpeg4_vol vol = {};
	struct vdec_mpeg4_vop vop;

	if (inst->vol.short_header ||
	    mp4_parse_buffer(inst, bs->va, bs->size, p->start + 4, &vol, &vop) != 1 || vol.valid)
		return false;
	inst->packed_dma = bs->dma_addr;
	inst->packed_size = bs->size;
	inst->packed_off = p->start + 4;

	return true;
}

/*
 * Take over a VOL (or H.263 picture format). A new size waits in @pending_vol
 * until the size change has been reported and the capture queue is set up.
 */
static int mp4_take_vol(struct vdec_mpeg4_direct_inst *inst, const struct vdec_mpeg4_vol *v)
{
	unsigned int mbw = DIV_ROUND_UP(v->w, MP4_MB_SIZE);
	unsigned int mbh = DIV_ROUND_UP(v->h, MP4_MB_SIZE);

	if (!mbw || !mbh || v->w > MP4_MAX_W || v->h > MP4_MAX_H || mbw * mbh > MP4_MAX_MBS ||
	    v->w < MTK_VDEC_MIN_W || v->h < MTK_VDEC_MIN_H) {
		mtk_vdec_err(inst->ctx, "picture %ux%u not supported", v->w, v->h);
		return -EOPNOTSUPP;
	}
	if (!inst->nbr || v->w != inst->vol.w || v->h != inst->vol.h)
		inst->pending_vol = *v;
	inst->vol = *v;

	return 0;
}

static int vdec_mpeg4_direct_decode(void *h_vdec, struct mtk_vcodec_mem *bs,
				    struct vdec_fb *fb, bool *res_chg)
{
	struct vdec_mpeg4_direct_inst *inst = h_vdec;
	struct mtk_vcodec_dec_ctx *ctx = inst->ctx;
	struct vdec_mpeg4_vol vol = {};
	struct vdec_mpeg4_vop vop;
	unsigned int luma;
	size_t from = 0;
	int ret;

	*res_chg = false;
	if (!bs) {
		/* drain: hand out the last anchor */
		mp4_flush_refs(inst);
		inst->packed_dma = 0;
		return 0;
	}

	/* the rest of a buffer with several VOPs (packed bitstream) */
	if (inst->packed_dma == bs->dma_addr && inst->packed_size == bs->size)
		from = inst->packed_off;
	inst->packed_dma = 0;
	ret = mp4_parse_buffer(inst, bs->va, bs->size, from, &vol, &vop);
	if (ret < 0)
		goto err_free;
	if (vol.valid) {
		int err = mp4_take_vol(inst, &vol);

		if (err) {
			ret = err;
			goto err_free;
		}
	}
	/*
	 * Report a new size with the buffer that holds the first VOP (decoded
	 * again after the change); a buffer with headers only is consumed.
	 */
	if (inst->pending_vol.valid && (ret == 1 || fb)) {
		*res_chg = true;
		inst->reported = true;
		ret = 0;
		goto out_free;
	}
	if (!fb || !ret)
		goto out_free;
	if (!inst->nbr) {
		ret = -EINVAL;
		goto err_free;
	}

	luma = mp4_luma_size(inst);
	if (fb->base_y.size < luma || fb->base_c.size < luma / 2) {
		mtk_vdec_err(ctx, "capture buffer too small for %ux%u", inst->vol.w, inst->vol.h);
		ret = -EINVAL;
		goto err_free;
	}
	if (vop.type == VOP_B && inst->anchors < 2) {
		mtk_vdec_debug(ctx, "B-VOP without two anchors skipped");
		goto out_free;
	}
	if (vop.type == VOP_S && vop.warp_used > 1 && !inst->warned) {
		mtk_vdec_err(ctx, "GMC with %u warping points decoded as translation",
			     vop.warp_used);
		inst->warned = true;
	}

	mp4_times(inst, &vop);
	ret = mp4_decode_vop(inst, bs, &vop, fb);
	if (ret)
		goto err_free;

	ctx->decoded_frame_cnt++;
	ret = mp4_more_vops(inst, bs, &vop) ? -EAGAIN : 0;
	if (vop.type == VOP_B) {
		mp4_push(inst->disp, &inst->disp_n, fb);
		mp4_push(inst->free, &inst->free_n, fb);
		return ret;
	}
	/* anchors wait for the next anchor, unless the stream has no B-VOPs */
	if (inst->undisplayed) {
		mp4_push(inst->disp, &inst->disp_n, inst->undisplayed);
		mp4_push(inst->free, &inst->free_n, inst->undisplayed);
	}
	inst->slot = !inst->slot;
	inst->ref_type = vop.type;
	inst->anchors++;
	inst->undisplayed = fb;
	if (inst->vol.low_delay) {
		mp4_push(inst->disp, &inst->disp_n, fb);
		mp4_push(inst->free, &inst->free_n, fb);
		inst->undisplayed = NULL;
	}

	return ret;

err_free:
	if (ret == -EINVAL)
		ret = -EBADMSG;
out_free:
	/* the capture buffer was not used */
	if (fb)
		mp4_push(inst->free, &inst->free_n, fb);
	return ret;
}

static int vdec_mpeg4_direct_init(struct mtk_vcodec_dec_ctx *ctx)
{
	struct vdec_mpeg4_direct_inst *inst;

	inst = kzalloc_obj(*inst);
	if (!inst)
		return -ENOMEM;
	inst->ctx = ctx;
	inst->dev = ctx->dev;
	ctx->drv_handle = inst;

	return 0;
}

static void vdec_mpeg4_direct_deinit(void *h_vdec)
{
	struct vdec_mpeg4_direct_inst *inst = h_vdec;
	unsigned int i;

	if (inst->work.va)
		mtk_vcodec_mem_free(inst->ctx, &inst->work);
	if (inst->mv.va)
		mtk_vcodec_mem_free(inst->ctx, &inst->mv);
	for (i = 0; i < ARRAY_SIZE(inst->pic); i++)
		if (inst->pic[i].va)
			mtk_vcodec_mem_free(inst->ctx, &inst->pic[i]);
	kfree(inst->nbr);
	if (inst->bs_fix.va)
		mtk_vcodec_mem_free(inst->ctx, &inst->bs_fix);
	kfree(inst);
}

static int vdec_mpeg4_direct_get_param(void *h_vdec, enum vdec_get_param_type type, void *out)
{
	struct vdec_mpeg4_direct_inst *inst = h_vdec;
	struct vdec_pic_info *pic = out;
	struct v4l2_rect *crop;
	struct vdec_fb *fb;
	int ret;

	switch (type) {
	case GET_PARAM_PIC_INFO:
		/* a size change reported by decode() takes effect here */
		if (inst->pending_vol.valid && inst->reported) {
			inst->pending_vol.valid = false;
			inst->reported = false;
			inst->mbw = DIV_ROUND_UP(inst->vol.w, MP4_MB_SIZE);
			inst->mbh = DIV_ROUND_UP(inst->vol.h, MP4_MB_SIZE);
			mp4_flush_refs(inst);
			ret = mp4_alloc(inst);
			if (ret)
				return ret;
		}
		*pic = inst->ctx->picinfo;
		pic->pic_w = inst->vol.w;
		pic->pic_h = inst->vol.h;
		pic->buf_w = inst->mbw * MP4_MB_SIZE;
		pic->buf_h = ALIGN(inst->mbh * MP4_MB_SIZE, 32);
		pic->fb_sz[0] = pic->buf_w * pic->buf_h;
		pic->fb_sz[1] = pic->fb_sz[0] / 2;
		break;
	case GET_PARAM_DPB_SIZE:
		/* two anchors, the target and one picture in display */
		*(unsigned int *)out = 4;
		break;
	case GET_PARAM_CROP_INFO:
		crop = out;
		crop->left = 0;
		crop->top = 0;
		crop->width = inst->vol.w;
		crop->height = inst->vol.h;
		break;
	case GET_PARAM_DISP_FRAME_BUFFER:
		fb = mp4_pop(inst->disp, &inst->disp_n);
		if (fb)
			fb->status |= FB_ST_DISPLAY;
		*(struct vdec_fb **)out = fb;
		break;
	case GET_PARAM_FREE_FRAME_BUFFER:
		fb = mp4_pop(inst->free, &inst->free_n);
		if (fb)
			fb->status |= FB_ST_FREE;
		*(struct vdec_fb **)out = fb;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

const struct vdec_common_if vdec_mpeg4_direct_if = {
	.init		= vdec_mpeg4_direct_init,
	.decode		= vdec_mpeg4_direct_decode,
	.get_param	= vdec_mpeg4_direct_get_param,
	.deinit		= vdec_mpeg4_direct_deinit,
};
