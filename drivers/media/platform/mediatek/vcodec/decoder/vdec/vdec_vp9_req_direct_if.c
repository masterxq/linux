// SPDX-License-Identifier: GPL-2.0
/*
 * VP9 decoding on MediaTek decoders that the host programs through their
 * registers, without firmware (MT6785).
 *
 * One hardware run decodes one frame with all of its tiles. The host takes
 * the uncompressed header from the frame control and programs the frame
 * from it. The decoder reads the compressed header itself, step by step on
 * request of the host, and applies its probability updates to the
 * probability memory that the host loaded with the frame context; the host
 * reads only tx_mode and reference_mode through the boolean decoder of the
 * syntax block, in the order of the header.
 *
 * After the frame the decoder writes the updated probabilities and the
 * symbol counts to memory; the backward adaptation and the four saved
 * frame contexts are kept by the host with the v4l2-vp9 helpers. The
 * probability and count tables have the layout of the MediaTek LAT
 * decoders (vdec_vp9_req_lat_if.c).
 *
 * The motion vectors of the previous frame and the segmentation map are
 * kept in one buffer each, which the decoder reads and overwrites in
 * place. References of another size are scaled by the decoder.
 *
 * Output is MM21 for 8 bits and MT2110T for 10 bits, both aligned to 64x64.
 * Supported: profiles 0 and 2 (4:2:0, 8 and 10 bits), tiles, intra-only
 * frames, reference scaling, segmentation, lossless coding and loop filter
 * deltas, up to 3840x2176.
 */

#include <linux/bitfield.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/unaligned.h>
#include <media/v4l2-mem2mem.h>
#include <media/v4l2-vp9.h>
#include <media/videobuf2-dma-contig.h>

#include "../mtk_vcodec_dec.h"
#include "../../common/mtk_vcodec_intr.h"
#include "../../common/mtk_vcodec_util.h"
#include "../vdec_drv_base.h"
#include "../vdec_drv_if.h"

/* bitstream readers (VDEC_LD and the second reader) */
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
#define VLD_PROC_STA_BUSY		BIT(15)
#define VLD_PROC_STA_IDLE		BIT(0)
#define VLD_SRST_READER			BIT(8)
#define VLD_WPTR			0x110
#define VLD_TILE_CFG			0x140
#define VLD_TILE_CFG_VAL		0x100014
#define VLD_SQT_ADDR			0x260
#define VLD_SQT_DATA			0x264

/* picture level of the reader (VDEC_TOP) */
#define TOP_PIC_SB_SIZE_M1		0x068
#define TOP_PIC_PIX_SIZE		0x070
#define TOP_TILE_BUF			0x0b8
#define TOP_BIT_DEPTH			0x100
#define TOP_BIT_DEPTH_10		0x22
#define TOP_SEG_RD_ADDR			0x118
#define TOP_SEG_WR_ADDR			0x11c
#define TOP_SEG_PREFETCH		0x124
#define TOP_SEG_PREFETCH_SIZE		GENMASK(15, 0)
#define TOP_SEG_PREFETCH_VAL		0x10

/* motion compensation (VDEC_CM) */
#define MC_PIC_W			0x208
#define MC_PIC_H			0x20c
#define MC_CFG_220			0x220
#define MC_Y_ADDR_512			0x224
#define MC_C_ADDR_256			0x228
#define MC_PITCH_16			0x22c
#define MC_CFG_238			0x238
#define MC_CFG_250			0x250
#define MC_PIC_W_16_M1			0x260
#define MC_PIC_H_16_M1			0x264
#define MC_CFG_A60			0xa60
#define MC_10BIT			0xcec
#define MC_REF_Y(i)			(0xd28 + 4 * (i))
#define MC_REF_C(i)			(0xd34 + 4 * (i))
#define MC_REF_STEP(i)			(0xd40 + 4 * (i))
#define MC_REF_STEP_X			GENMASK(31, 16)
#define MC_REF_STEP_Y			GENMASK(15, 0)
#define MC_REF_SIZE(i)			(0xd4c + 4 * (i))
#define MC_REF_SCALE_X(i)		(0xd58 + 8 * (i))
#define MC_REF_SCALE_Y(i)		(0xd5c + 8 * (i))
#define MC_REF_PITCH_16(i)		(0xd70 + 4 * (i))
#define MC_REF_RRF			0xd7c
#define MC_REF_SCALE_EN			0xd80
#define MC_MI_SIZE			0xd88

/* motion vectors (VDEC_AV) */
#define MV_WR_ADDR_16			0x20c
#define MV_RD_ADDR_16			0x3c0
#define MV_USE_PREV			0x3c4
#define MV_USE_PREV_EN			BIT(0)

/* post-processing (VDEC_PP) */
#define PP_LF_CTRL			0x0d8
#define PP_LF_CTRL_LEVEL		GENMASK(21, 16)
#define PP_LF_CTRL_SHARPNESS		GENMASK(10, 8)
#define PP_LF_CTRL_DELTA_EN		BIT(0)
#define PP_LF_REF_DELTAS		0x0dc
#define PP_LF_MODE_DELTAS		0x0e0
#define PP_LF_SEG_CTRL			0x0e4
#define PP_LF_SEG_ABS			BIT(8)
#define PP_LF_SEG_LO			0x0e8
#define PP_LF_SEG_HI			0x0ec
#define PP_PIC_SIZE			0x0f0
#define PP_SQT_WRITE			0x504
#define PP_LOSSLESS			0x9c0
#define PP_LOSSLESS_VAL			0xe00
#define PP_CFG_B80			0xb80
#define PP_CFG_B80_JUMP			BIT(2)
#define PP_10BIT_0			0xb8c
#define PP_10BIT_1			0xb90

/* control (VDEC_MISC) */
#define MISC_CFG_004			0x004
#define MISC_CFG_004_VP9		3
#define MISC_SYS_SEL			0x084
#define MISC_SYS_SEL_VAL		2
#define MISC_SPEC_CON			0x0c8
#define MISC_SPEC_CON_VP9		0xfdff
#define MISC_FUNC_CON			0x0cc
#define MISC_FUNC_CON_VP9		0x71e31182
#define MISC_DCM_CON			0x0ec
#define MISC_SRST			0x0f4
#define MISC_SIZE_CFG			0x160
#define MISC_SIZE_CFG_RESET		0xf1000
#define MISC_SIZE_CFG_VP9_LARGE		0x30ff10
#define MISC_CFG_178			0x178
#define MISC_CFG_178_VP9		0xfffff041
#define MISC_CFG_1F0			0x1f0
#define MISC_CFG_1F0_RESET		0
#define MISC_CFG_1F0_VP9_LARGE		0x1d00fff

/* clock enable of the decoder (VDEC_SYS) */
#define VDEC_SYS_CKEN			0x000

/* VP9 syntax block */
#define VP9_GETBITS			0x000
#define VP9_SKIP_BITS(n)		(VP9_GETBITS + 4 * (n))
#define VP9_BOOL_LITERAL(n)		(0x080 + 4 * (n))
#define VP9_BOOL_BIT			0x084
#define VP9_HDR_START			0x0a4
#define VP9_FRAME_CFG			0x0a8
#define VP9_FRAME_CFG_INTER		BIT(0)
#define VP9_FRAME_CFG_INTRA_ONLY	BIT(1)
#define VP9_FRAME_CFG_LOSSLESS		BIT(2)
#define VP9_FRAME_CFG_PARALLEL		BIT(3)
#define VP9_FRAME_CFG_ERROR_RES		BIT(4)
#define VP9_FRAME_CFG_SEG_EN		BIT(8)
#define VP9_FRAME_CFG_SEG_UPDATE_MAP	BIT(9)
#define VP9_FRAME_CFG_SEG_TEMPORAL	BIT(10)
#define VP9_FRAME_CFG_INTERP		GENMASK(26, 24)
#define VP9_FRAME_CFG_HIGH_PREC_MV	BIT(31)
#define VP9_SEG_FEATURES		0x0ac
#define VP9_SEG_SKIP(i)			BIT(i)
#define VP9_SEG_REF_SHIFT(i)		(8 + 2 * (i))
#define VP9_SEG_REF_EN(i)		BIT(24 + (i))
#define VP9_REF_CFG			0x0b0
#define VP9_REF_CFG_VAR			GENMASK(31, 24)
#define VP9_REF_CFG_SIGN_BIAS		GENMASK(19, 16)
#define VP9_REF_CFG_FIXED		GENMASK(15, 8)
#define VP9_REF_CFG_MODE		GENMASK(7, 0)
#define VP9_TX_MODE			0x0b4
#define VP9_DEC_START			0x0b8
#define VP9_TILE_SRC			0x0bc
#define VP9_TILE_SRC_FIFO		0x10111
#define VP9_TILE_SRC_MEM		0x111
#define VP9_TILE_NUM			0x0c4
#define VP9_TILE_NUM_ROWS_M1		GENMASK(31, 16)
#define VP9_TILE_NUM_COLS_M1		GENMASK(15, 0)
#define VP9_PROB_UPDATE			0x0c8
#define VP9_PROB_UPDATE_DONE		0x0cc
#define VP9_TILE_FIFO_RESET		0x100
#define VP9_TILE_FIFO			0x104
#define VP9_BOOL_INIT			0x110
#define VP9_BOOL_INIT_DONE		BIT(16)
#define VP9_BITS_COUNT_RESET		0x144
#define VP9_ERR_STA			0x15c
#define VP9_PROB_A_ADDR			0x198
#define VP9_PROB_A_DATA			0x19c
#define VP9_PROB_B_ADDR			0x1a0
#define VP9_PROB_B_DATA			0x1a4
#define VP9_PROB_ADDR_WRITE		BIT(16)
#define VP9_COUNTS_CLEAR		0x1a8
#define VP9_COUNTS_CLEAR_BUSY		BIT(0)
#define VP9_WDMA_SEL			0x1dc
#define VP9_WDMA_SEL_COUNTS		1
#define VP9_WDMA_SEL_PROBS		2
#define VP9_WDMA_ADDR			0x1e0
#define VP9_WDMA_START			0x1e8
#define VP9_WDMA_DONE			0x1ec
#define VP9_WDMA_DONE_BIT		BIT(0)
#define VP9_SEG_TREE_0			0x1f0
#define VP9_SEG_TREE_1			0x1f4
#define VP9_SEG_PRED			0x1f8

/* compressed header parts that the decoder reads on request (VP9_PROB_UPDATE) */
#define VP9_UPD_TX			BIT(0)
#define VP9_UPD_COEF			BIT(1)
#define VP9_UPD_SKIP			BIT(2)
#define VP9_UPD_INTER_MODE		BIT(3)
#define VP9_UPD_INTERP			BIT(4)
#define VP9_UPD_INTRA_INTER		BIT(5)
#define VP9_UPD_COMP_INTER		BIT(6)
#define VP9_UPD_SINGLE_REF		BIT(7)
#define VP9_UPD_COMP_REF		BIT(8)
#define VP9_UPD_Y_MODE			BIT(9)
#define VP9_UPD_PARTITION		BIT(10)
#define VP9_UPD_MV_INT			BIT(11)
#define VP9_UPD_MV_FP			BIT(12)
#define VP9_UPD_MV_HP			BIT(13)

/* words of the probability memory: port A 512, port B 128 */
#define VP9_PROB_COEF_ENTRIES		96
#define VP9_PROB_A_MODE			0x180
#define VP9_PROB_A_SEG			0x1a0
#define VP9_PROB_A_REST			0x1a8
#define VP9_PROB_A_WORDS		512
#define VP9_PROB_B_MODE			0x60
#define VP9_PROB_B_WORDS		128

/* tile descriptors: four words each, eight fit into the FIFO */
#define VP9_TILE_WORDS			4
#define VP9_TILE_FIFO_MAX		8
#define VP9_TILE_BUF_SIZE		4096
#define VP9_TILE_W0_BITS		GENMASK(6, 3)
#define VP9_TILE_W3_SB_ROWS_M1		GENMASK(14, 6)
#define VP9_TILE_W3_SB_COLS_M1		GENMASK(5, 0)

/* one reference without scaling */
#define VP9_RRF_UNSCALED		7
#define VP9_RRF_SMALL			4
#define VP9_RRF_LARGE			3
#define VP9_SCALE_SHIFT			14

#define VP9_PROB_BUF_SIZE		2560
#define VP9_COUNTS_BUF_SIZE		16384
/* motion vectors: 1152 bytes per superblock */
#define VP9_MV_SB_SIZE			1152
#define VP9_MV_EXTRA			0x900
/* segmentation map: 32 bytes per superblock */
#define VP9_SEG_SB_SIZE			32
#define VP9_SEG_EXTRA			0x100

/* larger pictures than this need the large picture settings */
#define VDEC_VP9_LARGE_PIC		0x1fe000
#define VDEC_VP9_ALIGN			64
/* bytes the reader may fetch behind the end of the frame */
#define VDEC_VP9_BS_PAD			0x200
#define VDEC_VP9_BS_MIN_PAD		64
#define VDEC_VP9_POLL_US		10000
#define VDEC_VP9_TIMEOUT_MS		500

#define VP9_NUM_REFS			3
#define VP9_NUM_REF_SLOTS		8
#define VP9_REF_LAST			1
#define VP9_REF_GOLDEN			2
#define VP9_REF_ALT			3

#define VP9_SEG_ENABLED(seg, i, f) \
	(!!((seg)->feature_enabled[i] & V4L2_VP9_SEGMENT_FEATURE_ENABLED(f)))

/*
 * struct vdec_vp9_direct_probs - probability memory of the decoder, 2560
 * bytes: entry e of the coefficient probabilities is at A[4e..4e+3] and
 * B[e], bytes 1920 to 2431 at A[384..511], the rest at B[96..127]
 */
struct vdec_vp9_direct_probs {
	struct {
		u8 probs[6][3];
		u8 padding[2];
	} coef[4][2][2][6];

	u8 y_mode[4][16];
	u8 interp_filter[4][16];
	u8 seg[32];
	u8 comp_mode[16];
	u8 comp_ref[16];
	u8 single_ref[5][2];
	u8 single_ref_padding[6];

	u8 joint[3];
	u8 joint_padding[13];
	struct {
		u8 sign;
		u8 classes[10];
		u8 padding[5];
	} sign_classes[2];
	struct {
		u8 class0[1];
		u8 bits[10];
		u8 padding[5];
	} class0_bits[2];
	struct {
		u8 class0_fp[2][3];
		u8 fp[3];
		u8 class0_hp;
		u8 hp;
		u8 padding[5];
	} class0_fp_hp[2];

	u8 uv_mode[10][16];
	u8 uv_mode_padding[2][16];

	u8 partition[16][4];

	u8 inter_mode[7][4];
	u8 skip[4];

	u8 tx8[2][4];
	u8 tx16[2][4];
	u8 tx32[2][4];
	u8 is_inter[8];
};

/* struct vdec_vp9_direct_counts - symbol counts as the decoder writes them */
struct vdec_vp9_direct_counts {
	union {
		struct {
			u32 band_0[3];
			u32 padding0[1];
			u32 band_1_5[5][6];
			u32 padding1[2];
		} eob_branch[4][2][2];
		u32 eob_branch_space[256 * 4];
	};

	struct {
		u32 band_0[3][4];
		u32 band_1_5[5][6][4];
	} coef[4][2][2];

	u32 intra_inter[4][2];
	u32 comp_inter[5][2];
	u32 comp_inter_padding[2];
	u32 comp_ref[5][2];
	u32 comp_ref_padding[2];
	u32 single_ref[5][2][2];
	u32 inter_mode[7][4];
	u32 y_mode[4][12];
	u32 uv_mode[10][10];
	u32 partition[16][4];
	u32 switchable_interp[4][4];

	u32 tx_p8x8[2][2];
	u32 tx_p16x16[2][4];
	u32 tx_p32x32[2][4];

	u32 skip[3][4];

	u32 joint[4];

	struct {
		u32 sign[2];
		u32 class0[2];
		u32 classes[12];
		u32 bits[10][2];
		u32 padding[4];
		u32 class0_fp[2][4];
		u32 fp[4];
		u32 class0_hp[2];
		u32 hp[2];
	} mvcomp[2];

	u32 reserved[126][4];
};

/* counts whose shape differs from struct v4l2_vp9_frame_symbol_counts */
struct vdec_vp9_direct_counts_map {
	u32 skip[3][2];
	u32 y_mode[4][10];
	u32 filter[4][3];
	u32 sign[2][2];
	u32 classes[2][11];
	u32 class0[2][2];
	u32 bits[2][10][2];
	u32 class0_fp[2][2][4];
	u32 fp[2][4];
	u32 class0_hp[2][2];
	u32 hp[2][2];
};

/**
 * struct vdec_vp9_direct_ref - picture as the decoder sees it
 * @y:		luma address
 * @c:		chroma address
 * @width:	frame width
 * @height:	frame height
 * @pitch:	width of the buffer layout, the pitch registers take it in 16s
 */
struct vdec_vp9_direct_ref {
	dma_addr_t y;
	dma_addr_t c;
	unsigned int width;
	unsigned int height;
	unsigned int pitch;
};

/**
 * struct vdec_vp9_direct_tile - one tile with rows of superblocks
 * @addr:	bus address of the tile data
 * @size:	size of the tile data in bytes
 * @sb_rows:	superblock rows of the tile
 * @sb_cols:	superblock columns of the tile
 */
struct vdec_vp9_direct_tile {
	dma_addr_t addr;
	u32 size;
	unsigned int sb_rows;
	unsigned int sb_cols;
};

/**
 * struct vdec_vp9_direct_inst - VP9 decoder instance
 * @ctx:		decoder context
 * @dev:		decoder device
 * @vp9:		registers of the VP9 syntax block
 * @vld2:		registers of the second bitstream reader
 * @frame:		frame control of the current frame
 * @refs:		LAST, GOLDEN and ALTREF of the current frame
 * @tiles:		tiles of the current frame in bitstream order
 * @num_tiles:		tiles with superblocks
 * @tile_rows:		tile rows with superblocks
 * @mv_buf:		motion vectors of the previous frame, overwritten
 * @seg_buf:		segmentation map, overwritten
 * @tile_buf:		tile descriptors of frames with more than eight tiles
 * @prob_buf:		probabilities after the compressed header
 * @counts_buf:		symbol counts of the frame
 * @fctx:		the four saved frame contexts
 * @adapt:		frame context under adaptation
 * @counts:		symbol counts for the v4l2-vp9 helpers
 * @counts_map:		counts in the shape of the helpers
 * @fctx_idx:		frame context of the current frame
 * @tx_mode:		tx_mode of the current frame
 * @ref_mode:		reference_mode of the current frame
 * @lossless:		the current frame is lossless
 * @intra:		the current frame is a key frame or intra-only
 * @comp_fixed:		compound fixed reference, kept between frames
 * @comp_var:		compound variable references, kept between frames
 * @seg_temporal:	segmentation temporal update, kept between frames
 * @last_w:		width of the previous frame
 * @last_h:		height of the previous frame
 * @last_show:		the previous frame was shown
 * @last_intra_only:	the previous frame was intra-only
 * @last_key:		the previous frame was a key frame
 * @have_last:		a frame was decoded before
 */
struct vdec_vp9_direct_inst {
	struct mtk_vcodec_dec_ctx *ctx;
	struct mtk_vcodec_dec_dev *dev;
	void __iomem *vp9;
	void __iomem *vld2;

	const struct v4l2_ctrl_vp9_frame *frame;
	struct vdec_vp9_direct_ref refs[VP9_NUM_REFS];
	struct vdec_vp9_direct_tile tiles[64 * 4];
	unsigned int num_tiles;
	unsigned int tile_rows;

	struct mtk_vcodec_mem mv_buf;
	struct mtk_vcodec_mem seg_buf;
	struct mtk_vcodec_mem tile_buf;
	struct mtk_vcodec_mem prob_buf;
	struct mtk_vcodec_mem counts_buf;

	struct v4l2_vp9_frame_context fctx[V4L2_VP9_NUM_FRAME_CTX];
	struct v4l2_vp9_frame_context adapt;
	struct v4l2_vp9_frame_symbol_counts counts;
	struct vdec_vp9_direct_counts_map counts_map;
	u8 fctx_idx;
	u8 tx_mode;
	u8 ref_mode;
	bool lossless;
	bool intra;

	u8 comp_fixed;
	u8 comp_var[2];
	bool seg_temporal;

	unsigned int last_w;
	unsigned int last_h;
	bool last_show;
	bool last_intra_only;
	bool last_key;
	bool have_last;
};

static_assert(sizeof(struct vdec_vp9_direct_probs) == VP9_PROB_BUF_SIZE);
static_assert(sizeof(struct vdec_vp9_direct_counts) == VP9_COUNTS_BUF_SIZE);

static inline void vdec_write(struct vdec_vp9_direct_inst *inst, int blk,
			      u32 off, u32 val)
{
	writel(val, inst->dev->reg_base[blk] + off);
}

static inline u32 vdec_read(struct vdec_vp9_direct_inst *inst, int blk, u32 off)
{
	return readl(inst->dev->reg_base[blk] + off);
}

static inline void vdec_mask(struct vdec_vp9_direct_inst *inst, int blk,
			     u32 off, u32 clear, u32 set)
{
	vdec_write(inst, blk, off, (vdec_read(inst, blk, off) & ~clear) | set);
}

static inline void vp9_write(struct vdec_vp9_direct_inst *inst, u32 off, u32 val)
{
	writel(val, inst->vp9 + off);
}

static inline u32 vp9_read(struct vdec_vp9_direct_inst *inst, u32 off)
{
	return readl(inst->vp9 + off);
}

static int vdec_vp9_poll(struct vdec_vp9_direct_inst *inst, void __iomem *reg,
			 u32 mask, bool set, const char *what)
{
	u32 val;
	int ret;

	ret = readl_poll_timeout(reg, val, !!(val & mask) == set, 1,
				 VDEC_VP9_POLL_US);
	if (ret)
		mtk_vdec_err(inst->ctx, "%s timeout: %#x", what, val);

	return ret;
}

static inline bool vdec_vp9_flag(const struct v4l2_ctrl_vp9_frame *frame, u32 flag)
{
	return !!(frame->flags & flag);
}

static unsigned int vdec_vp9_width(const struct v4l2_ctrl_vp9_frame *frame)
{
	return frame->frame_width_minus_1 + 1;
}

static unsigned int vdec_vp9_height(const struct v4l2_ctrl_vp9_frame *frame)
{
	return frame->frame_height_minus_1 + 1;
}

/* ---------------- frame contexts ---------------- */

static void vdec_vp9_probs_to_hw(const struct v4l2_vp9_frame_context *p,
				 struct vdec_vp9_direct_probs *hw)
{
	const struct v4l2_vp9_frame_mv_context *mv = &p->mv;
	unsigned int i, j, k, l;

	memset(hw, 0, sizeof(*hw));
	for (i = 0; i < 4; i++)
		for (j = 0; j < 2; j++)
			for (k = 0; k < 2; k++)
				for (l = 0; l < 6; l++)
					memcpy(hw->coef[i][j][k][l].probs, p->coef[i][j][k][l],
					       sizeof(p->coef[i][j][k][l]));

	for (i = 0; i < ARRAY_SIZE(p->y_mode); i++)
		memcpy(hw->y_mode[i], p->y_mode[i], sizeof(p->y_mode[i]));
	for (i = 0; i < ARRAY_SIZE(p->interp_filter); i++)
		memcpy(hw->interp_filter[i], p->interp_filter[i],
		       sizeof(p->interp_filter[i]));
	memcpy(hw->comp_mode, p->comp_mode, sizeof(p->comp_mode));
	memcpy(hw->comp_ref, p->comp_ref, sizeof(p->comp_ref));
	memcpy(hw->single_ref, p->single_ref, sizeof(p->single_ref));

	memcpy(hw->joint, mv->joint, sizeof(mv->joint));
	for (i = 0; i < 2; i++) {
		hw->sign_classes[i].sign = mv->sign[i];
		memcpy(hw->sign_classes[i].classes, mv->classes[i],
		       sizeof(mv->classes[i]));
		hw->class0_bits[i].class0[0] = mv->class0_bit[i];
		memcpy(hw->class0_bits[i].bits, mv->bits[i], sizeof(mv->bits[i]));
		memcpy(hw->class0_fp_hp[i].class0_fp, mv->class0_fr[i],
		       sizeof(mv->class0_fr[i]));
		memcpy(hw->class0_fp_hp[i].fp, mv->fr[i], sizeof(mv->fr[i]));
		hw->class0_fp_hp[i].class0_hp = mv->class0_hp[i];
		hw->class0_fp_hp[i].hp = mv->hp[i];
	}

	for (i = 0; i < ARRAY_SIZE(p->uv_mode); i++)
		memcpy(hw->uv_mode[i], p->uv_mode[i], sizeof(p->uv_mode[i]));
	for (i = 0; i < ARRAY_SIZE(p->partition); i++)
		memcpy(hw->partition[i], p->partition[i], sizeof(p->partition[i]));
	for (i = 0; i < ARRAY_SIZE(p->inter_mode); i++)
		memcpy(hw->inter_mode[i], p->inter_mode[i], sizeof(p->inter_mode[i]));
	memcpy(hw->skip, p->skip, sizeof(p->skip));
	for (i = 0; i < 2; i++) {
		memcpy(hw->tx8[i], p->tx8[i], sizeof(p->tx8[i]));
		memcpy(hw->tx16[i], p->tx16[i], sizeof(p->tx16[i]));
		memcpy(hw->tx32[i], p->tx32[i], sizeof(p->tx32[i]));
	}
	memcpy(hw->is_inter, p->is_inter, sizeof(p->is_inter));
}

/*
 * Probabilities of the decoder after the compressed header. With @coef
 * false the coefficient probabilities are left alone, with @noncoef false
 * the others.
 */
static void vdec_vp9_probs_from_hw(const struct vdec_vp9_direct_probs *hw,
				   struct v4l2_vp9_frame_context *p,
				   bool coef, bool noncoef)
{
	struct v4l2_vp9_frame_mv_context *mv = &p->mv;
	unsigned int i, j, k, l;

	if (coef)
		for (i = 0; i < 4; i++)
			for (j = 0; j < 2; j++)
				for (k = 0; k < 2; k++)
					for (l = 0; l < 6; l++)
						memcpy(p->coef[i][j][k][l],
						       hw->coef[i][j][k][l].probs,
						       sizeof(p->coef[i][j][k][l]));
	if (!noncoef)
		return;

	for (i = 0; i < ARRAY_SIZE(p->y_mode); i++)
		memcpy(p->y_mode[i], hw->y_mode[i], sizeof(p->y_mode[i]));
	for (i = 0; i < ARRAY_SIZE(p->interp_filter); i++)
		memcpy(p->interp_filter[i], hw->interp_filter[i],
		       sizeof(p->interp_filter[i]));
	memcpy(p->comp_mode, hw->comp_mode, sizeof(p->comp_mode));
	memcpy(p->comp_ref, hw->comp_ref, sizeof(p->comp_ref));
	memcpy(p->single_ref, hw->single_ref, sizeof(p->single_ref));

	memcpy(mv->joint, hw->joint, sizeof(mv->joint));
	for (i = 0; i < 2; i++) {
		mv->sign[i] = hw->sign_classes[i].sign;
		memcpy(mv->classes[i], hw->sign_classes[i].classes,
		       sizeof(mv->classes[i]));
		mv->class0_bit[i] = hw->class0_bits[i].class0[0];
		memcpy(mv->bits[i], hw->class0_bits[i].bits, sizeof(mv->bits[i]));
		memcpy(mv->class0_fr[i], hw->class0_fp_hp[i].class0_fp,
		       sizeof(mv->class0_fr[i]));
		memcpy(mv->fr[i], hw->class0_fp_hp[i].fp, sizeof(mv->fr[i]));
		mv->class0_hp[i] = hw->class0_fp_hp[i].class0_hp;
		mv->hp[i] = hw->class0_fp_hp[i].hp;
	}

	for (i = 0; i < ARRAY_SIZE(p->uv_mode); i++)
		memcpy(p->uv_mode[i], hw->uv_mode[i], sizeof(p->uv_mode[i]));
	for (i = 0; i < ARRAY_SIZE(p->partition); i++)
		memcpy(p->partition[i], hw->partition[i], sizeof(p->partition[i]));
	for (i = 0; i < ARRAY_SIZE(p->inter_mode); i++)
		memcpy(p->inter_mode[i], hw->inter_mode[i], sizeof(p->inter_mode[i]));
	memcpy(p->skip, hw->skip, sizeof(p->skip));
	for (i = 0; i < 2; i++) {
		memcpy(p->tx8[i], hw->tx8[i], sizeof(p->tx8[i]));
		memcpy(p->tx16[i], hw->tx16[i], sizeof(p->tx16[i]));
		memcpy(p->tx32[i], hw->tx32[i], sizeof(p->tx32[i]));
	}
	memcpy(p->is_inter, hw->is_inter, sizeof(p->is_inter));
}

/* point the counts of the helpers into the table the decoder wrote */
static void vdec_vp9_map_counts(struct vdec_vp9_direct_inst *inst)
{
	struct vdec_vp9_direct_counts *c = inst->counts_buf.va;
	struct vdec_vp9_direct_counts_map *m = &inst->counts_map;
	struct v4l2_vp9_frame_symbol_counts *h = &inst->counts;
	unsigned int i, j, k, l, n;

	h->partition = &c->partition;
	h->intra_inter = &c->intra_inter;
	h->tx32p = &c->tx_p32x32;
	h->tx16p = &c->tx_p16x16;
	h->tx8p = &c->tx_p8x8;
	h->uv_mode = &c->uv_mode;
	h->comp = &c->comp_inter;
	h->comp_ref = &c->comp_ref;
	h->single_ref = &c->single_ref;
	h->mv_mode = &c->inter_mode;
	h->mv_joint = &c->joint;

	for (i = 0; i < 3; i++)
		memcpy(m->skip[i], c->skip[i], sizeof(m->skip[i]));
	h->skip = &m->skip;
	for (i = 0; i < 4; i++)
		memcpy(m->y_mode[i], c->y_mode[i], sizeof(m->y_mode[i]));
	h->y_mode = &m->y_mode;
	for (i = 0; i < 4; i++)
		memcpy(m->filter[i], c->switchable_interp[i], sizeof(m->filter[i]));
	h->filter = &m->filter;

	for (i = 0; i < 2; i++) {
		memcpy(m->sign[i], c->mvcomp[i].sign, sizeof(m->sign[i]));
		memcpy(m->classes[i], c->mvcomp[i].classes, sizeof(m->classes[i]));
		memcpy(m->class0[i], c->mvcomp[i].class0, sizeof(m->class0[i]));
		memcpy(m->bits[i], c->mvcomp[i].bits, sizeof(m->bits[i]));
		memcpy(m->class0_fp[i], c->mvcomp[i].class0_fp, sizeof(m->class0_fp[i]));
		memcpy(m->fp[i], c->mvcomp[i].fp, sizeof(m->fp[i]));
		memcpy(m->class0_hp[i], c->mvcomp[i].class0_hp, sizeof(m->class0_hp[i]));
		memcpy(m->hp[i], c->mvcomp[i].hp, sizeof(m->hp[i]));
	}
	h->sign = &m->sign;
	h->classes = &m->classes;
	h->class0 = &m->class0;
	h->bits = &m->bits;
	h->class0_fp = &m->class0_fp;
	h->fp = &m->fp;
	h->class0_hp = &m->class0_hp;
	h->hp = &m->hp;

	/*
	 * Coefficients: the counts of the more_coefs token (eob) follow the
	 * three token counts of each context.
	 */
	for (i = 0; i < 4; i++)
		for (j = 0; j < 2; j++)
			for (k = 0; k < 2; k++) {
				for (n = 0; n < 3; n++) {
					h->coeff[i][j][k][0][n] =
						(u32 (*)[3])c->coef[i][j][k].band_0[n];
					h->eob[i][j][k][0][n][0] =
						&c->eob_branch[i][j][k].band_0[n];
					h->eob[i][j][k][0][n][1] =
						&c->coef[i][j][k].band_0[n][3];
				}
				for (l = 1; l < 6; l++)
					for (n = 0; n < 6; n++) {
						h->coeff[i][j][k][l][n] = (u32 (*)[3])
							c->coef[i][j][k].band_1_5[l - 1][n];
						h->eob[i][j][k][l][n][0] =
							&c->eob_branch[i][j][k].band_1_5[l - 1][n];
						h->eob[i][j][k][l][n][1] =
							&c->coef[i][j][k].band_1_5[l - 1][n][3];
					}
			}
}

/*
 * Backward adaptation (8.4.2) and saving of the frame context: the
 * adaptation starts from the probabilities loaded for the frame; the
 * probabilities that are not adapted keep the values after the compressed
 * header.
 */
static void vdec_vp9_update_fctx(struct vdec_vp9_direct_inst *inst)
{
	const struct v4l2_ctrl_vp9_frame *frame = inst->frame;
	const struct vdec_vp9_direct_probs *hw = inst->prob_buf.va;
	struct v4l2_vp9_frame_context *fc = &inst->fctx[inst->fctx_idx];
	struct v4l2_vp9_frame_context *a = &inst->adapt;
	bool frame_is_intra = inst->intra;

	if (!vdec_vp9_flag(frame, V4L2_VP9_FRAME_FLAG_REFRESH_FRAME_CTX))
		return;

	if (vdec_vp9_flag(frame, V4L2_VP9_FRAME_FLAG_ERROR_RESILIENT) ||
	    vdec_vp9_flag(frame, V4L2_VP9_FRAME_FLAG_PARALLEL_DEC_MODE)) {
		vdec_vp9_probs_from_hw(hw, fc, true, true);
		return;
	}

	*a = *fc;
	if (frame_is_intra)
		vdec_vp9_probs_from_hw(hw, a, false, true);
	vdec_vp9_map_counts(inst);
	v4l2_vp9_adapt_coef_probs(a, &inst->counts,
				  !frame_is_intra && inst->last_key, frame_is_intra);
	if (!frame_is_intra)
		v4l2_vp9_adapt_noncoef_probs(a, &inst->counts, inst->ref_mode,
					     frame->interpolation_filter, inst->tx_mode,
					     frame->flags);
	*fc = *a;
}

/* ---------------- buffers ---------------- */

static int vdec_vp9_mem(struct vdec_vp9_direct_inst *inst,
			struct mtk_vcodec_mem *mem, size_t size)
{
	if (mem->va && mem->size >= size)
		return 0;
	if (mem->va)
		mtk_vcodec_mem_free(inst->ctx, mem);
	mem->size = size;

	return mtk_vcodec_mem_alloc(inst->ctx, mem);
}

static int vdec_vp9_alloc(struct vdec_vp9_direct_inst *inst, unsigned int sb)
{
	int ret;

	ret = vdec_vp9_mem(inst, &inst->mv_buf, VP9_MV_EXTRA + sb * VP9_MV_SB_SIZE);
	if (!ret)
		ret = vdec_vp9_mem(inst, &inst->seg_buf,
				   VP9_SEG_EXTRA + sb * VP9_SEG_SB_SIZE);
	if (!ret)
		ret = vdec_vp9_mem(inst, &inst->tile_buf, VP9_TILE_BUF_SIZE);
	if (!ret)
		ret = vdec_vp9_mem(inst, &inst->prob_buf, VP9_PROB_BUF_SIZE);
	if (!ret)
		ret = vdec_vp9_mem(inst, &inst->counts_buf, VP9_COUNTS_BUF_SIZE);

	return ret;
}

static void vdec_vp9_free(struct vdec_vp9_direct_inst *inst)
{
	struct mtk_vcodec_mem *mems[] = {
		&inst->mv_buf, &inst->seg_buf, &inst->tile_buf, &inst->prob_buf,
		&inst->counts_buf,
	};
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(mems); i++)
		if (mems[i]->va)
			mtk_vcodec_mem_free(inst->ctx, mems[i]);
}

/*
 * Make room for the zeros the reader may fetch behind the frame and return
 * the end of the reader window, 64-byte aligned.
 */
static int vdec_vp9_bs_end(struct vdec_vp9_direct_inst *inst,
			   struct mtk_vcodec_mem *bs, dma_addr_t *end)
{
	struct mtk_video_dec_buf *src = container_of(bs, struct mtk_video_dec_buf,
						     bs_buffer);
	dma_addr_t limit = bs->dma_addr +
			   vb2_plane_size(&src->m2m_buf.vb.vb2_buf, 0);
	dma_addr_t data_end = bs->dma_addr + bs->size;

	*end = ALIGN(data_end + VDEC_VP9_BS_PAD, 64);
	if (*end > limit)
		*end = ALIGN_DOWN(limit, 64);
	if (*end < data_end + VDEC_VP9_BS_MIN_PAD) {
		mtk_vdec_err(inst->ctx, "no room behind the bitstream (%zu of %pad)",
			     bs->size, &limit);
		return -EINVAL;
	}
	memset(bs->va + bs->size, 0, *end - data_end);

	return 0;
}

/* LAST, GOLDEN and ALTREF with their capture buffers and frame sizes */
static void vdec_vp9_build_refs(struct vdec_vp9_direct_inst *inst,
				const struct vdec_vp9_direct_ref *dst)
{
	struct vb2_queue *vq = v4l2_m2m_get_vq(inst->ctx->m2m_ctx,
					       V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE);
	const struct v4l2_ctrl_vp9_frame *frame = inst->frame;
	const u64 ts[VP9_NUM_REFS] = {
		frame->last_frame_ts, frame->golden_frame_ts, frame->alt_frame_ts,
	};
	unsigned int i;

	for (i = 0; i < VP9_NUM_REFS; i++) {
		struct vdec_vp9_direct_ref *ref = &inst->refs[i];
		struct mtk_video_dec_buf *buf;
		struct vb2_buffer *vb;

		*ref = *dst;
		if (inst->intra)
			continue;
		vb = vb2_find_buffer(vq, ts[i]);
		buf = vb ? container_of(to_vb2_v4l2_buffer(vb), struct mtk_video_dec_buf,
					m2m_buf.vb) : NULL;
		if (!buf || !buf->frame_buffer.pic_w) {
			mtk_vdec_debug(inst->ctx, "reference %u (%llu) not found", i, ts[i]);
			continue;
		}
		/* a reference may have another size and layout than the frame */
		ref->y = vb2_dma_contig_plane_dma_addr(vb, 0);
		ref->c = vb2_dma_contig_plane_dma_addr(vb, 1);
		ref->width = buf->frame_buffer.pic_w;
		ref->height = buf->frame_buffer.pic_h;
		ref->pitch = buf->frame_buffer.buf_w;
	}
}

/*
 * Tiles (6.4) from the sizes in front of each tile but the last. Tiles of
 * rows without superblocks are skipped, the decoder only gets the others.
 */
static int vdec_vp9_build_tiles(struct vdec_vp9_direct_inst *inst,
				struct mtk_vcodec_mem *bs)
{
	const struct v4l2_ctrl_vp9_frame *frame = inst->frame;
	unsigned int mi_cols = DIV_ROUND_UP(vdec_vp9_width(frame), 8);
	unsigned int mi_rows = DIV_ROUND_UP(vdec_vp9_height(frame), 8);
	unsigned int sb_cols = DIV_ROUND_UP(mi_cols, 8);
	unsigned int sb_rows = DIV_ROUND_UP(mi_rows, 8);
	unsigned int rows = 1 << frame->tile_rows_log2;
	unsigned int cols = 1 << frame->tile_cols_log2;
	size_t pos = frame->uncompressed_header_size + frame->compressed_header_size;
	const u8 *data = bs->va;
	unsigned int r, c;

	inst->num_tiles = 0;
	inst->tile_rows = 0;
	for (r = 0; r < rows; r++) {
		unsigned int r0 = min(((r * sb_rows) >> frame->tile_rows_log2) * 8, mi_rows);
		unsigned int r1 = min((((r + 1) * sb_rows) >> frame->tile_rows_log2) * 8,
				      mi_rows);

		if (r1 > r0)
			inst->tile_rows++;
		for (c = 0; c < cols; c++) {
			unsigned int c0 = min(((c * sb_cols) >> frame->tile_cols_log2) * 8,
					      mi_cols);
			unsigned int c1 = min((((c + 1) * sb_cols) >> frame->tile_cols_log2) * 8,
					      mi_cols);
			bool last = r == rows - 1 && c == cols - 1;
			struct vdec_vp9_direct_tile *t;
			size_t size;

			if (last) {
				if (pos > bs->size)
					return -EINVAL;
				size = bs->size - pos;
			} else {
				if (pos + 4 > bs->size)
					return -EINVAL;
				size = get_unaligned_be32(data + pos);
				pos += 4;
				if (size > bs->size - pos)
					return -EINVAL;
			}
			if (r1 > r0) {
				t = &inst->tiles[inst->num_tiles++];
				t->addr = bs->dma_addr + pos;
				t->size = size;
				t->sb_rows = DIV_ROUND_UP(r1 - r0, 8);
				t->sb_cols = DIV_ROUND_UP(c1 - c0, 8);
			}
			pos += size;
		}
	}

	return inst->tile_rows ? 0 : -EINVAL;
}

/* ---------------- register sequence of one frame ---------------- */

static void vdec_vp9_reset(struct vdec_vp9_direct_inst *inst)
{
	mtk_vcodec_write_vdecsys(inst->ctx, VDEC_SYS_CKEN, 1);
	vdec_write(inst, VDEC_MISC, MISC_SRST, 0);
	vdec_write(inst, VDEC_MISC, MISC_DCM_CON, 1);
	mtk_vcodec_write_vdecsys(inst->ctx, VDEC_SYS_DCM_CON, 1);
	vdec_write(inst, VDEC_LD, VDEC_VLD_SRST, VLD_SRST_READER | VDEC_VLD_SRST_RESET);
	vdec_write(inst, VDEC_MISC, MISC_SPEC_CON, MISC_SPEC_CON_VP9);
	vdec_write(inst, VDEC_MISC, MISC_FUNC_CON, MISC_FUNC_CON_VP9);
	vdec_write(inst, VDEC_MISC, MISC_CFG_178, MISC_CFG_178_VP9);
	vdec_write(inst, VDEC_MISC, MISC_SYS_SEL, MISC_SYS_SEL_VAL);
	vdec_write(inst, VDEC_LD, VDEC_VLD_SRST, 0);
	vdec_write(inst, VDEC_MISC, MISC_CFG_004, MISC_CFG_004_VP9);
	vp9_write(inst, VP9_HDR_START, 1);
}

static int vdec_vp9_clear_counts(struct vdec_vp9_direct_inst *inst)
{
	vp9_write(inst, VP9_COUNTS_CLEAR, 1);
	return vdec_vp9_poll(inst, inst->vp9 + VP9_COUNTS_CLEAR,
			     VP9_COUNTS_CLEAR_BUSY, false, "count clear");
}

/*
 * Both readers cover the bitstream buffer; the first one stands on the
 * compressed header at @start.
 */
static int vdec_vp9_init_reader(struct vdec_vp9_direct_inst *inst,
				dma_addr_t base, dma_addr_t start, dma_addr_t end)
{
	void __iomem *vld = inst->dev->reg_base[VDEC_LD];
	unsigned int i;
	int ret;

	writel(base >> 6, vld + VLD_VSTART);
	writel(end >> 6, vld + VLD_VEND);
	writel(base >> 6, inst->vld2 + VLD_VSTART);
	writel(end >> 6, inst->vld2 + VLD_VEND);
	writel(readl(vld + VLD_VBAR) | VLD_VBAR_EN, vld + VLD_VBAR);
	writel(readl(inst->vld2 + VLD_VBAR) | VLD_VBAR_EN, inst->vld2 + VLD_VBAR);
	if (readl(vld + VLD_PROC_STA) & VLD_PROC_STA_BUSY) {
		ret = vdec_vp9_poll(inst, vld + VLD_PROC_STA, VLD_PROC_STA_IDLE, true,
				    "reader idle");
		if (ret)
			return ret;
	}
	writel(start, vld + VLD_RPTR);
	writel(end, vld + VLD_WPTR);
	for (i = 0; i < 5; i++)
		writel(VLD_SRST_READER, vld + VDEC_VLD_SRST);
	writel(0, vld + VDEC_VLD_SRST);
	writel(VLD_BS_CTRL_FETCH, vld + VLD_BS_CTRL);
	ret = vdec_vp9_poll(inst, vld + VLD_FETCH_STA, VLD_FETCH_STA_READY, true,
			    "bitstream fetch");
	writel(VLD_BS_CTRL_RUN, vld + VLD_BS_CTRL);
	if (ret)
		return ret;

	/* the reader starts at the 16-byte line of the read pointer */
	for (i = 0; i < (start & 0xf); i++) {
		vp9_read(inst, VP9_GETBITS);
		vp9_read(inst, VP9_SKIP_BITS(8));
	}
	vp9_write(inst, VP9_BITS_COUNT_RESET, 1);
	vp9_write(inst, VP9_BITS_COUNT_RESET, 0);

	return 0;
}

/* segment features, loop filter and the frame flags (uncompressed header) */
static void vdec_vp9_write_frame_cfg(struct vdec_vp9_direct_inst *inst)
{
	const struct v4l2_ctrl_vp9_frame *frame = inst->frame;
	const struct v4l2_vp9_segmentation *seg = &frame->seg;
	const struct v4l2_vp9_loop_filter *lf = &frame->lf;
	bool seg_en = seg->flags & V4L2_VP9_SEGMENTATION_FLAG_ENABLED;
	bool key = vdec_vp9_flag(frame, V4L2_VP9_FRAME_FLAG_KEY_FRAME);
	bool intra_only = vdec_vp9_flag(frame, V4L2_VP9_FRAME_FLAG_INTRA_ONLY);
	unsigned int width = vdec_vp9_width(frame);
	unsigned int height = vdec_vp9_height(frame);
	u32 cfg = 0, feat = 0, lf_en = 0, lf_lo = 0, lf_hi = 0;
	unsigned int i;

	if (seg->flags & V4L2_VP9_SEGMENTATION_FLAG_UPDATE_MAP)
		inst->seg_temporal = seg->flags & V4L2_VP9_SEGMENTATION_FLAG_TEMPORAL_UPDATE;

	vdec_write(inst, VDEC_TOP, TOP_BIT_DEPTH,
		   frame->bit_depth == 10 ? TOP_BIT_DEPTH_10 : 0);
	vdec_write(inst, VDEC_TOP, TOP_PIC_PIX_SIZE, height << 16 | width);
	vdec_write(inst, VDEC_TOP, TOP_PIC_SB_SIZE_M1,
		   (DIV_ROUND_UP(height, 64) - 1) << 16 | (DIV_ROUND_UP(width, 64) - 1));
	vp9_write(inst, VP9_HDR_START, 1);

	/* an intra-only frame of profile 0 is decoded like a key frame */
	if (!key && !(intra_only && frame->bit_depth == 8))
		cfg |= VP9_FRAME_CFG_INTER;
	if (intra_only)
		cfg |= VP9_FRAME_CFG_INTRA_ONLY;
	if (inst->lossless)
		cfg |= VP9_FRAME_CFG_LOSSLESS;
	if (vdec_vp9_flag(frame, V4L2_VP9_FRAME_FLAG_PARALLEL_DEC_MODE) ||
	    vdec_vp9_flag(frame, V4L2_VP9_FRAME_FLAG_ERROR_RESILIENT))
		cfg |= VP9_FRAME_CFG_PARALLEL;
	if (vdec_vp9_flag(frame, V4L2_VP9_FRAME_FLAG_ERROR_RESILIENT))
		cfg |= VP9_FRAME_CFG_ERROR_RES;
	if (seg_en) {
		cfg |= VP9_FRAME_CFG_SEG_EN;
		if (seg->flags & V4L2_VP9_SEGMENTATION_FLAG_UPDATE_MAP)
			cfg |= VP9_FRAME_CFG_SEG_UPDATE_MAP;
	}
	if (inst->seg_temporal)
		cfg |= VP9_FRAME_CFG_SEG_TEMPORAL;
	if (!inst->intra) {
		cfg |= FIELD_PREP(VP9_FRAME_CFG_INTERP, frame->interpolation_filter);
		if (vdec_vp9_flag(frame, V4L2_VP9_FRAME_FLAG_ALLOW_HIGH_PREC_MV))
			cfg |= VP9_FRAME_CFG_HIGH_PREC_MV;
	}
	vp9_write(inst, VP9_FRAME_CFG, cfg);

	for (i = 0; seg_en && i < 8; i++) {
		if (VP9_SEG_ENABLED(seg, i, V4L2_VP9_SEG_LVL_SKIP))
			feat |= VP9_SEG_SKIP(i);
		if (VP9_SEG_ENABLED(seg, i, V4L2_VP9_SEG_LVL_REF_FRAME))
			feat |= VP9_SEG_REF_EN(i);
		feat |= (seg->feature_data[i][V4L2_VP9_SEG_LVL_REF_FRAME] & 3) <<
			VP9_SEG_REF_SHIFT(i);
		if (VP9_SEG_ENABLED(seg, i, V4L2_VP9_SEG_LVL_ALT_L)) {
			u8 v = seg->feature_data[i][V4L2_VP9_SEG_LVL_ALT_L];

			lf_en |= BIT(i);
			if (i < 4)
				lf_lo |= v << (8 * i);
			else
				lf_hi |= v << (8 * (i - 4));
		}
	}
	vp9_write(inst, VP9_SEG_FEATURES, feat);

	vdec_write(inst, VDEC_PP, PP_LF_CTRL,
		   FIELD_PREP(PP_LF_CTRL_LEVEL, lf->level) |
		   FIELD_PREP(PP_LF_CTRL_SHARPNESS, lf->sharpness) |
		   (lf->flags & V4L2_VP9_LOOP_FILTER_FLAG_DELTA_ENABLED ?
		    PP_LF_CTRL_DELTA_EN : 0));
	if (lf->flags & V4L2_VP9_LOOP_FILTER_FLAG_DELTA_ENABLED) {
		vdec_write(inst, VDEC_PP, PP_LF_REF_DELTAS,
			   (u8)lf->ref_deltas[0] | (u8)lf->ref_deltas[1] << 8 |
			   (u8)lf->ref_deltas[2] << 16 | (u32)(u8)lf->ref_deltas[3] << 24);
		vdec_write(inst, VDEC_PP, PP_LF_MODE_DELTAS,
			   (u8)lf->mode_deltas[0] | (u8)lf->mode_deltas[1] << 8);
	}
	if (seg_en) {
		if (lf_en && (seg->flags & V4L2_VP9_SEGMENTATION_FLAG_ABS_OR_DELTA_UPDATE))
			lf_en |= PP_LF_SEG_ABS;
		vdec_write(inst, VDEC_PP, PP_LF_SEG_CTRL, lf_en);
		vdec_write(inst, VDEC_PP, PP_LF_SEG_LO, lf_lo);
		vdec_write(inst, VDEC_PP, PP_LF_SEG_HI, lf_hi);
	}
	if (inst->lossless)
		vdec_write(inst, VDEC_PP, PP_LOSSLESS, PP_LOSSLESS_VAL);

	if (seg_en) {
		const u8 *t = seg->tree_probs, *p = seg->pred_probs;

		vp9_write(inst, VP9_SEG_TREE_0, t[0] | t[1] << 8 | t[2] << 16 | t[3] << 24);
		vp9_write(inst, VP9_SEG_TREE_1, t[4] | t[5] << 8 | t[6] << 16);
		vp9_write(inst, VP9_SEG_PRED, p[0] | p[1] << 8 | p[2] << 16);
		vdec_write(inst, VDEC_TOP, TOP_SEG_RD_ADDR, inst->seg_buf.dma_addr);
		vdec_write(inst, VDEC_TOP, TOP_SEG_WR_ADDR, inst->seg_buf.dma_addr);
	}
}

/* load the frame context into the probability memory */
static void vdec_vp9_write_probs(struct vdec_vp9_direct_inst *inst)
{
	struct vdec_vp9_direct_probs *hw = inst->prob_buf.va;
	const u32 *w = (const u32 *)hw;
	unsigned int i, e;

	vdec_vp9_probs_to_hw(&inst->fctx[inst->fctx_idx], hw);

	vp9_write(inst, VP9_PROB_A_ADDR, VP9_PROB_ADDR_WRITE);
	vp9_write(inst, VP9_PROB_B_ADDR, VP9_PROB_ADDR_WRITE);
	for (e = 0; e < VP9_PROB_COEF_ENTRIES; e++, w += 5) {
		for (i = 0; i < 4; i++)
			vp9_write(inst, VP9_PROB_A_DATA, w[i]);
		vp9_write(inst, VP9_PROB_B_DATA, w[4]);
	}

	/* mode probabilities, without the unused segmentation part */
	vp9_write(inst, VP9_PROB_A_ADDR, VP9_PROB_ADDR_WRITE | VP9_PROB_A_MODE);
	for (i = VP9_PROB_A_MODE; i < VP9_PROB_A_SEG; i++)
		vp9_write(inst, VP9_PROB_A_DATA, *w++);
	w += VP9_PROB_A_REST - VP9_PROB_A_SEG;
	vp9_write(inst, VP9_PROB_A_ADDR, VP9_PROB_ADDR_WRITE | VP9_PROB_A_REST);
	for (i = VP9_PROB_A_REST; i < VP9_PROB_A_WORDS; i++)
		vp9_write(inst, VP9_PROB_A_DATA, *w++);
	vp9_write(inst, VP9_PROB_B_ADDR, VP9_PROB_ADDR_WRITE | VP9_PROB_B_MODE);
	for (i = VP9_PROB_B_MODE; i < VP9_PROB_B_WORDS; i++)
		vp9_write(inst, VP9_PROB_B_DATA, *w++);
}

static int vdec_vp9_prob_update(struct vdec_vp9_direct_inst *inst, u32 what)
{
	vp9_write(inst, VP9_PROB_UPDATE, what);
	vp9_write(inst, VP9_PROB_UPDATE, 0);
	return vdec_vp9_poll(inst, inst->vp9 + VP9_PROB_UPDATE_DONE, what, true,
			     "compressed header");
}

/*
 * compressed_header() (6.3): the host reads tx_mode and reference_mode,
 * the decoder each group of probability updates.
 */
static int vdec_vp9_compressed_hdr(struct vdec_vp9_direct_inst *inst)
{
	const struct v4l2_ctrl_vp9_frame *frame = inst->frame;
	u8 sb = frame->ref_frame_sign_bias;
	bool compound_allowed;
	u8 mode = V4L2_VP9_REFERENCE_MODE_SINGLE_REFERENCE;
	u32 upd[16];

	inst->ref_mode = mode;
	unsigned int n = 0, i;
	int ret;

	vp9_write(inst, VP9_BITS_COUNT_RESET, 1);
	vp9_write(inst, VP9_BITS_COUNT_RESET, 0);
	vp9_write(inst, VP9_BOOL_INIT, 1);
	ret = vdec_vp9_poll(inst, inst->vp9 + VP9_BOOL_INIT, VP9_BOOL_INIT_DONE, true,
			    "boolean decoder");
	if (ret)
		return ret;

	inst->tx_mode = V4L2_VP9_TX_MODE_ONLY_4X4;
	if (!inst->lossless) {
		inst->tx_mode = vp9_read(inst, VP9_BOOL_LITERAL(2));
		if (inst->tx_mode == V4L2_VP9_TX_MODE_ALLOW_32X32)
			inst->tx_mode += vp9_read(inst, VP9_BOOL_BIT);
	}
	vp9_write(inst, VP9_TX_MODE, inst->tx_mode);

	if (inst->tx_mode == V4L2_VP9_TX_MODE_SELECT)
		upd[n++] = VP9_UPD_TX;
	upd[n++] = VP9_UPD_COEF;
	upd[n++] = VP9_UPD_SKIP;
	if (!inst->intra) {
		upd[n++] = VP9_UPD_INTER_MODE;
		if (frame->interpolation_filter == V4L2_VP9_INTERP_FILTER_SWITCHABLE)
			upd[n++] = VP9_UPD_INTERP;
		upd[n++] = VP9_UPD_INTRA_INTER;
	}
	for (i = 0; i < n; i++) {
		ret = vdec_vp9_prob_update(inst, upd[i]);
		if (ret)
			return ret;
	}
	if (inst->intra)
		return 0;

	/* frame_reference_mode() */
	compound_allowed = !!(sb & V4L2_VP9_SIGN_BIAS_GOLDEN) !=
			   !!(sb & V4L2_VP9_SIGN_BIAS_LAST) ||
			   !!(sb & V4L2_VP9_SIGN_BIAS_ALT) !=
			   !!(sb & V4L2_VP9_SIGN_BIAS_LAST);
	if (compound_allowed && vp9_read(inst, VP9_BOOL_BIT))
		mode = vp9_read(inst, VP9_BOOL_BIT) ?
		       V4L2_VP9_REFERENCE_MODE_SELECT :
		       V4L2_VP9_REFERENCE_MODE_COMPOUND_REFERENCE;
	if (mode != frame->reference_mode)
		mtk_vdec_debug(inst->ctx, "reference_mode %u, control has %u", mode,
			       frame->reference_mode);
	if (mode != V4L2_VP9_REFERENCE_MODE_SINGLE_REFERENCE) {
		/* setup_compound_reference_mode() */
		bool last = sb & V4L2_VP9_SIGN_BIAS_LAST;

		if (last == !!(sb & V4L2_VP9_SIGN_BIAS_GOLDEN)) {
			inst->comp_fixed = VP9_REF_ALT;
			inst->comp_var[0] = VP9_REF_LAST;
			inst->comp_var[1] = VP9_REF_GOLDEN;
		} else if (last == !!(sb & V4L2_VP9_SIGN_BIAS_ALT)) {
			inst->comp_fixed = VP9_REF_GOLDEN;
			inst->comp_var[0] = VP9_REF_LAST;
			inst->comp_var[1] = VP9_REF_ALT;
		} else {
			inst->comp_fixed = VP9_REF_LAST;
			inst->comp_var[0] = VP9_REF_GOLDEN;
			inst->comp_var[1] = VP9_REF_ALT;
		}
	}

	n = 0;
	if (mode == V4L2_VP9_REFERENCE_MODE_SELECT)
		upd[n++] = VP9_UPD_COMP_INTER;
	if (mode != V4L2_VP9_REFERENCE_MODE_COMPOUND_REFERENCE)
		upd[n++] = VP9_UPD_SINGLE_REF;
	if (mode != V4L2_VP9_REFERENCE_MODE_SINGLE_REFERENCE)
		upd[n++] = VP9_UPD_COMP_REF;
	upd[n++] = VP9_UPD_Y_MODE;
	upd[n++] = VP9_UPD_PARTITION;
	upd[n++] = VP9_UPD_MV_INT;
	upd[n++] = VP9_UPD_MV_FP;
	if (vdec_vp9_flag(frame, V4L2_VP9_FRAME_FLAG_ALLOW_HIGH_PREC_MV))
		upd[n++] = VP9_UPD_MV_HP;
	for (i = 0; i < n; i++) {
		ret = vdec_vp9_prob_update(inst, upd[i]);
		if (ret)
			return ret;
	}

	inst->ref_mode = mode;

	return 0;
}

/* scaling of one reference (8.5.2.3), in 1/16384 */
static u32 vdec_vp9_scale(unsigned int ref, unsigned int cur)
{
	return (ref << VP9_SCALE_SHIFT) / cur;
}

static u32 vdec_vp9_rrf(u32 scale)
{
	return (u64)scale * 1000 < 0xfa4000 ? VP9_RRF_SMALL : VP9_RRF_LARGE;
}

static void vdec_vp9_write_refs(struct vdec_vp9_direct_inst *inst)
{
	const struct v4l2_ctrl_vp9_frame *frame = inst->frame;
	unsigned int width = vdec_vp9_width(frame);
	unsigned int height = vdec_vp9_height(frame);
	bool key = vdec_vp9_flag(frame, V4L2_VP9_FRAME_FLAG_KEY_FRAME);
	u32 scale_en = 0, rrf = 0;
	unsigned int i;

	for (i = 0; i < VP9_NUM_REFS; i++) {
		vdec_write(inst, VDEC_CM, MC_REF_Y(i), inst->refs[i].y);
		vdec_write(inst, VDEC_CM, MC_REF_C(i), inst->refs[i].c);
	}
	if (key)
		return;

	for (i = 0; i < VP9_NUM_REFS; i++) {
		const struct vdec_vp9_direct_ref *ref = &inst->refs[i];
		u32 xs = vdec_vp9_scale(ref->width, width);
		u32 ys = vdec_vp9_scale(ref->height, height);
		unsigned int shift = 4 * (2 - i);

		if (xs == BIT(VP9_SCALE_SHIFT) && ys == BIT(VP9_SCALE_SHIFT)) {
			rrf |= (VP9_RRF_UNSCALED << 16 | VP9_RRF_UNSCALED) << shift;
		} else {
			scale_en |= BIT(shift);
			rrf |= (vdec_vp9_rrf(xs) << 16 | vdec_vp9_rrf(ys)) << shift;
		}
	}
	vdec_write(inst, VDEC_CM, MC_REF_SCALE_EN, scale_en);
	for (i = 0; i < VP9_NUM_REFS; i++) {
		const struct vdec_vp9_direct_ref *ref = &inst->refs[i];
		u32 xs = vdec_vp9_scale(ref->width, width);
		u32 ys = vdec_vp9_scale(ref->height, height);

		vdec_write(inst, VDEC_CM, MC_REF_SIZE(i), ref->width << 16 | ref->height);
		if (scale_en & BIT(4 * (2 - i))) {
			vdec_write(inst, VDEC_CM, MC_REF_STEP(i),
				   FIELD_PREP(MC_REF_STEP_X, (16 * xs) >> VP9_SCALE_SHIFT) |
				   FIELD_PREP(MC_REF_STEP_Y, (16 * ys) >> VP9_SCALE_SHIFT));
			vdec_write(inst, VDEC_CM, MC_REF_SCALE_X(i), xs);
			vdec_write(inst, VDEC_CM, MC_REF_SCALE_Y(i), ys);
		}
		vdec_write(inst, VDEC_CM, MC_REF_PITCH_16(i), ref->pitch / 16);
	}
	vdec_write(inst, VDEC_CM, MC_REF_RRF, rrf);
}

/* quantizer step sizes of each segment (8.6.1) */
static void vdec_vp9_write_sqt(struct vdec_vp9_direct_inst *inst)
{
	const struct v4l2_ctrl_vp9_frame *frame = inst->frame;
	const struct v4l2_vp9_quantization *q = &frame->quant;
	const struct v4l2_vp9_segmentation *seg = &frame->seg;
	bool seg_en = seg->flags & V4L2_VP9_SEGMENTATION_FLAG_ENABLED;
	const u16 *dc = v4l2_vp9_dc_qlookup[(frame->bit_depth - 8) / 2];
	const u16 *ac = v4l2_vp9_ac_qlookup[(frame->bit_depth - 8) / 2];
	unsigned int i;

	vdec_write(inst, VDEC_PP, PP_SQT_WRITE, 1);
	for (i = 0; i < 8; i++) {
		int qi = q->base_q_idx;
		u32 y = 0, uv = 0;

		if (!i || seg_en) {
			if (seg_en && VP9_SEG_ENABLED(seg, i, V4L2_VP9_SEG_LVL_ALT_Q)) {
				int d = seg->feature_data[i][V4L2_VP9_SEG_LVL_ALT_Q];

				if (seg->flags & V4L2_VP9_SEGMENTATION_FLAG_ABS_OR_DELTA_UPDATE)
					qi = d;
				else
					qi += d;
				qi = clamp(qi, 0, 255);
			}
			y = dc[clamp(qi + q->delta_q_y_dc, 0, 255)] | (u32)ac[qi] << 16;
			uv = dc[clamp(qi + q->delta_q_uv_dc, 0, 255)] |
			     (u32)ac[clamp(qi + q->delta_q_uv_ac, 0, 255)] << 16;
		}
		vdec_write(inst, VDEC_LD, VLD_SQT_ADDR, 2 * i);
		vdec_write(inst, VDEC_LD, VLD_SQT_DATA, y);
		vdec_write(inst, VDEC_LD, VLD_SQT_ADDR, 2 * i + 1);
		vdec_write(inst, VDEC_LD, VLD_SQT_DATA, uv);
	}
	vdec_write(inst, VDEC_PP, PP_SQT_WRITE, 0);
}

static void vdec_vp9_write_pic(struct vdec_vp9_direct_inst *inst,
			       const struct vdec_vp9_direct_ref *dst)
{
	const struct v4l2_ctrl_vp9_frame *frame = inst->frame;
	unsigned int width = vdec_vp9_width(frame);
	unsigned int height = vdec_vp9_height(frame);
	bool ten_bit = frame->bit_depth == 10;
	u8 sb = inst->intra ? 0 : frame->ref_frame_sign_bias;
	bool use_prev;

	/* the compound references stay from the last frame that set them */
	vp9_write(inst, VP9_REF_CFG,
		  FIELD_PREP(VP9_REF_CFG_VAR, inst->comp_var[0] + 4 * inst->comp_var[1]) |
		  FIELD_PREP(VP9_REF_CFG_SIGN_BIAS, sb << 1) |
		  FIELD_PREP(VP9_REF_CFG_FIXED, inst->comp_fixed) |
		  FIELD_PREP(VP9_REF_CFG_MODE, inst->ref_mode));

	/* use_prev_frame_mvs (7.2) */
	use_prev = !vdec_vp9_flag(frame, V4L2_VP9_FRAME_FLAG_ERROR_RESILIENT) &&
		   inst->have_last && width == inst->last_w && height == inst->last_h &&
		   !inst->last_intra_only && inst->last_show;
	vdec_mask(inst, VDEC_AV, MV_USE_PREV, MV_USE_PREV_EN,
		  use_prev ? MV_USE_PREV_EN : 0);
	vdec_write(inst, VDEC_AV, MV_RD_ADDR_16, inst->mv_buf.dma_addr >> 4);
	vdec_write(inst, VDEC_AV, MV_WR_ADDR_16, inst->mv_buf.dma_addr >> 4);

	vdec_write(inst, VDEC_CM, MC_10BIT, ten_bit);
	vdec_vp9_write_refs(inst);
	vdec_write(inst, VDEC_CM, MC_MI_SIZE,
		   DIV_ROUND_UP(width, 8) << 16 | DIV_ROUND_UP(height, 8));
	vdec_write(inst, VDEC_CM, MC_CFG_220, 1);
	vdec_write(inst, VDEC_CM, MC_Y_ADDR_512, dst->y >> 9);
	vdec_write(inst, VDEC_CM, MC_C_ADDR_256, dst->c >> 8);
	vdec_write(inst, VDEC_CM, MC_PITCH_16, dst->pitch / 16);
	vdec_write(inst, VDEC_CM, MC_CFG_238, 3);
	vdec_write(inst, VDEC_CM, MC_CFG_250, 1);
	vdec_write(inst, VDEC_CM, MC_PIC_W_16_M1, ALIGN(width, 64) / 16 - 1);
	vdec_write(inst, VDEC_CM, MC_PIC_H_16_M1, ALIGN(height, 64) / 16 - 1);
	vdec_write(inst, VDEC_PP, PP_PIC_SIZE, height << 16 | width);
	/* 10 bits: MT2110 output, which does not work in jump mode */
	vdec_write(inst, VDEC_PP, PP_10BIT_0, ten_bit);
	vdec_write(inst, VDEC_PP, PP_10BIT_1, ten_bit);
	vdec_mask(inst, VDEC_PP, PP_CFG_B80, ten_bit ? PP_CFG_B80_JUMP : 0,
		  ten_bit ? 0 : PP_CFG_B80_JUMP);
	vdec_vp9_write_sqt(inst);
	vdec_write(inst, VDEC_CM, MC_CFG_A60, 0);
	vdec_write(inst, VDEC_CM, MC_PIC_W, width);
	vdec_write(inst, VDEC_CM, MC_PIC_H, height);
}

static void vdec_vp9_write_tiles(struct vdec_vp9_direct_inst *inst)
{
	const struct v4l2_ctrl_vp9_frame *frame = inst->frame;
	bool fifo = (inst->tile_rows << frame->tile_cols_log2) <= VP9_TILE_FIFO_MAX;
	u32 *mem = inst->tile_buf.va;
	unsigned int i, j;

	vp9_write(inst, VP9_TILE_NUM,
		  FIELD_PREP(VP9_TILE_NUM_ROWS_M1, inst->tile_rows - 1) |
		  FIELD_PREP(VP9_TILE_NUM_COLS_M1, (1 << frame->tile_cols_log2) - 1));
	vp9_write(inst, VP9_TILE_FIFO_RESET, 0);
	for (i = 0; i < inst->num_tiles; i++) {
		const struct vdec_vp9_direct_tile *t = &inst->tiles[i];
		u32 skip = (t->addr & 0xf) * 8;
		u32 w[VP9_TILE_WORDS] = {
			skip + t->size * 8,
			t->addr & ~0xf,
			skip,
			FIELD_PREP(VP9_TILE_W3_SB_ROWS_M1, t->sb_rows - 1) |
			FIELD_PREP(VP9_TILE_W3_SB_COLS_M1, t->sb_cols - 1),
		};

		for (j = 0; j < VP9_TILE_WORDS; j++) {
			if (fifo)
				vp9_write(inst, VP9_TILE_FIFO, w[j]);
			else
				mem[VP9_TILE_WORDS * i + j] = w[j];
		}
	}
	vdec_write(inst, VDEC_LD, VLD_TILE_CFG, VLD_TILE_CFG_VAL);
	writel(VLD_TILE_CFG_VAL, inst->vld2 + VLD_TILE_CFG);
	if (!fifo) {
		/* the descriptors must be in memory before the decoder reads them */
		wmb();
		vdec_write(inst, VDEC_TOP, TOP_TILE_BUF, inst->tile_buf.dma_addr);
	}
	vp9_write(inst, VP9_TILE_SRC, fifo ? VP9_TILE_SRC_FIFO : VP9_TILE_SRC_MEM);
}

static int vdec_vp9_run(struct vdec_vp9_direct_inst *inst, struct mtk_vcodec_mem *bs,
			const struct vdec_vp9_direct_ref *dst)
{
	const struct v4l2_ctrl_vp9_frame *frame = inst->frame;
	dma_addr_t end;
	int ret;

	ret = vdec_vp9_build_tiles(inst, bs);
	if (ret) {
		mtk_vdec_err(inst->ctx, "tile sizes do not fit the frame of %zu bytes",
			     bs->size);
		return ret;
	}
	ret = vdec_vp9_bs_end(inst, bs, &end);
	if (ret)
		return ret;

	vdec_vp9_reset(inst);
	ret = vdec_vp9_clear_counts(inst);
	if (ret)
		return ret;
	ret = vdec_vp9_init_reader(inst, bs->dma_addr,
				   bs->dma_addr + frame->uncompressed_header_size, end);
	if (ret)
		return ret;

	vdec_vp9_write_frame_cfg(inst);
	vdec_vp9_write_probs(inst);
	ret = vdec_vp9_compressed_hdr(inst);
	if (ret)
		return ret;

	vdec_vp9_write_pic(inst, dst);
	vdec_vp9_write_tiles(inst);
	ret = vdec_vp9_clear_counts(inst);
	if (ret)
		return ret;
	vdec_mask(inst, VDEC_TOP, TOP_SEG_PREFETCH, TOP_SEG_PREFETCH_SIZE,
		  TOP_SEG_PREFETCH_VAL);
	/*
	 * The decoder keeps these settings while it stays powered, also for the
	 * next picture of any codec; small pictures use the reset values.
	 */
	if (vdec_vp9_width(frame) * vdec_vp9_height(frame) > VDEC_VP9_LARGE_PIC) {
		vdec_write(inst, VDEC_MISC, MISC_SIZE_CFG, MISC_SIZE_CFG_VP9_LARGE);
		vdec_write(inst, VDEC_MISC, MISC_CFG_1F0, MISC_CFG_1F0_VP9_LARGE);
	} else {
		vdec_write(inst, VDEC_MISC, MISC_SIZE_CFG, MISC_SIZE_CFG_RESET);
		vdec_write(inst, VDEC_MISC, MISC_CFG_1F0, MISC_CFG_1F0_RESET);
	}
	vp9_write(inst, VP9_DEC_START, 1);

	return 0;
}

/* write the probabilities or the counts of the frame to memory */
static int vdec_vp9_wdma(struct vdec_vp9_direct_inst *inst, u32 sel, dma_addr_t addr)
{
	vp9_write(inst, VP9_WDMA_SEL, sel);
	vp9_write(inst, VP9_WDMA_ADDR, addr);
	vp9_write(inst, VP9_WDMA_START, sel);
	return vdec_vp9_poll(inst, inst->vp9 + VP9_WDMA_DONE, VP9_WDMA_DONE_BIT, true,
			     "table write");
}

/* ---------------- codec interface ---------------- */

static int vdec_vp9_get_params(struct vdec_vp9_direct_inst *inst)
{
	struct mtk_vcodec_dec_ctx *ctx = inst->ctx;
	const struct mtk_vdec_direct_data *data = inst->dev->vdec_pdata->direct;
	const struct v4l2_ctrl_vp9_frame *frame;
	struct v4l2_ctrl *ctrl;
	unsigned int width, height;
	bool ten_bits;

	ctrl = v4l2_ctrl_find(&ctx->ctrl_hdl, V4L2_CID_STATELESS_VP9_FRAME);
	if (!ctrl)
		return -EINVAL;
	frame = ctrl->p_cur.p;
	inst->frame = frame;

	width = vdec_vp9_width(frame);
	height = vdec_vp9_height(frame);
	if (width > data->max_width || height > data->max_height) {
		mtk_vdec_err(ctx, "frame %ux%u not supported", width, height);
		return -EINVAL;
	}
	if ((frame->bit_depth != 8 && frame->bit_depth != 10) ||
	    !vdec_vp9_flag(frame, V4L2_VP9_FRAME_FLAG_X_SUBSAMPLING) ||
	    !vdec_vp9_flag(frame, V4L2_VP9_FRAME_FLAG_Y_SUBSAMPLING)) {
		mtk_vdec_err(ctx, "only 4:2:0 with 8 or 10 bits supported");
		return -EOPNOTSUPP;
	}
	ten_bits = frame->bit_depth == 10;
	if (ctx->is_10bit_bitstream != ten_bits) {
		mtk_vdec_err(ctx, "bit depth differs from the capture format");
		return -EINVAL;
	}
	if (frame->tile_rows_log2 > 2 || frame->tile_cols_log2 > 6 ||
	    !frame->uncompressed_header_size || !frame->compressed_header_size)
		return -EINVAL;

	inst->intra = vdec_vp9_flag(frame, V4L2_VP9_FRAME_FLAG_KEY_FRAME) ||
		      vdec_vp9_flag(frame, V4L2_VP9_FRAME_FLAG_INTRA_ONLY);
	inst->lossless = !frame->quant.base_q_idx && !frame->quant.delta_q_y_dc &&
			 !frame->quant.delta_q_uv_dc && !frame->quant.delta_q_uv_ac;

	return 0;
}

/*
 * Frame contexts and buffers that the uncompressed header resets
 * (setup_past_independence, 7.2), and the segmentation map after a size
 * change.
 */
static void vdec_vp9_reset_state(struct vdec_vp9_direct_inst *inst)
{
	const struct v4l2_ctrl_vp9_frame *frame = inst->frame;
	bool reset = inst->intra ||
		     vdec_vp9_flag(frame, V4L2_VP9_FRAME_FLAG_ERROR_RESILIENT);

	inst->fctx_idx = v4l2_vp9_reset_frame_ctx(frame, inst->fctx);
	if (reset || !inst->have_last || vdec_vp9_width(frame) != inst->last_w ||
	    vdec_vp9_height(frame) != inst->last_h)
		memset(inst->seg_buf.va, 0, inst->seg_buf.size);
}

static void vdec_vp9_end_frame(struct vdec_vp9_direct_inst *inst, struct vdec_fb *fb)
{
	const struct v4l2_ctrl_vp9_frame *frame = inst->frame;

	fb->pic_w = vdec_vp9_width(frame);
	fb->pic_h = vdec_vp9_height(frame);
	inst->last_w = vdec_vp9_width(frame);
	inst->last_h = vdec_vp9_height(frame);
	inst->last_show = vdec_vp9_flag(frame, V4L2_VP9_FRAME_FLAG_SHOW_FRAME);
	inst->last_intra_only = vdec_vp9_flag(frame, V4L2_VP9_FRAME_FLAG_INTRA_ONLY);
	inst->last_key = vdec_vp9_flag(frame, V4L2_VP9_FRAME_FLAG_KEY_FRAME);
	inst->have_last = true;
}

static int vdec_vp9_direct_decode(void *h_vdec, struct mtk_vcodec_mem *bs,
				  struct vdec_fb *unused, bool *res_chg)
{
	struct vdec_vp9_direct_inst *inst = h_vdec;
	struct mtk_vcodec_dec_ctx *ctx = inst->ctx;
	struct mtk_video_dec_buf *src_buf, *dst_buf;
	struct vdec_vp9_direct_ref dst;
	struct vb2_buffer *dst_vb;
	struct vdec_fb *fb;
	unsigned int width, height, luma, sb;
	u32 err;
	int ret;

	*res_chg = false;
	/* nothing is held in the hardware between frames */
	if (!bs)
		return 0;

	fb = inst->dev->vdec_pdata->get_cap_buffer(ctx);
	if (!fb)
		return -ENOMEM;
	src_buf = container_of(bs, struct mtk_video_dec_buf, bs_buffer);
	dst_buf = container_of(fb, struct mtk_video_dec_buf, frame_buffer);
	v4l2_m2m_buf_copy_metadata(&src_buf->m2m_buf.vb, &dst_buf->m2m_buf.vb);

	ret = vdec_vp9_get_params(inst);
	if (ret)
		return ret;

	/*
	 * The capture buffer keeps the layout of the format it was queued
	 * with; the format may already be the one of a later frame size.
	 */
	width = vdec_vp9_width(inst->frame);
	height = vdec_vp9_height(inst->frame);
	if (ALIGN(width, VDEC_VP9_ALIGN) > fb->buf_w ||
	    ALIGN(height, VDEC_VP9_ALIGN) > fb->buf_h || fb->buf_w % 16) {
		mtk_vdec_err(ctx, "frame %ux%u does not fit the %ux%u buffer",
			     width, height, fb->buf_w, fb->buf_h);
		return -EINVAL;
	}
	dst_vb = &dst_buf->m2m_buf.vb.vb2_buf;
	luma = fb->buf_w * fb->buf_h;
	if (inst->frame->bit_depth == 10)
		luma = luma * 5 / 4;
	if (vb2_plane_size(dst_vb, 0) < luma || vb2_plane_size(dst_vb, 1) < luma / 2)
		return -EINVAL;

	/* motion vectors and segmentation map follow the frame size */
	sb = DIV_ROUND_UP(width, 64) * DIV_ROUND_UP(height, 64);
	ret = vdec_vp9_alloc(inst, sb);
	if (ret)
		return ret;

	vdec_vp9_reset_state(inst);
	dst.y = fb->base_y.dma_addr;
	dst.c = fb->base_c.dma_addr;
	dst.width = width;
	dst.height = height;
	dst.pitch = fb->buf_w;
	vdec_vp9_build_refs(inst, &dst);
	/* the buffer no longer holds the frame it had */
	fb->pic_w = 0;

	ret = vdec_vp9_run(inst, bs, &dst);
	if (ret)
		goto err_break;

	ret = mtk_vcodec_wait_for_done_ctx(ctx, MTK_INST_IRQ_RECEIVED,
					   VDEC_VP9_TIMEOUT_MS, MTK_VDEC_CORE);
	if (ret) {
		ret = -ETIMEDOUT;
		goto err_break;
	}

	err = vp9_read(inst, VP9_ERR_STA);
	ret = vdec_vp9_wdma(inst, VP9_WDMA_SEL_PROBS, inst->prob_buf.dma_addr);
	if (!ret)
		ret = vdec_vp9_wdma(inst, VP9_WDMA_SEL_COUNTS, inst->counts_buf.dma_addr);
	if (ret)
		goto err_break;

	vdec_vp9_update_fctx(inst);
	vdec_vp9_end_frame(inst, fb);
	ctx->decoded_frame_cnt++;
	if (err) {
		mtk_vdec_err(ctx, "bitstream error %#x in frame %d", err,
			     ctx->decoded_frame_cnt - 1);
		return -EBADMSG;
	}
	mtk_vdec_debug(ctx, "frame %d done", ctx->decoded_frame_cnt - 1);

	return 0;

err_break:
	mtk_vdec_direct_hw_break(inst->dev);
	inst->have_last = false;
	return ret;
}

static int vdec_vp9_direct_init(struct mtk_vcodec_dec_ctx *ctx)
{
	const struct mtk_vdec_direct_data *data = ctx->dev->vdec_pdata->direct;
	struct vdec_vp9_direct_inst *inst;
	unsigned int i;

	if (!data->vp9_vld || !data->vld2)
		return -EOPNOTSUPP;

	inst = kzalloc_obj(*inst);
	if (!inst)
		return -ENOMEM;
	inst->ctx = ctx;
	inst->dev = ctx->dev;
	inst->vp9 = mtk_vdec_direct_block(ctx->dev, data->vp9_vld);
	inst->vld2 = mtk_vdec_direct_block(ctx->dev, data->vld2);
	for (i = 0; i < V4L2_VP9_NUM_FRAME_CTX; i++)
		inst->fctx[i] = v4l2_vp9_default_probs;

	ctx->drv_handle = inst;
	return 0;
}

static void vdec_vp9_direct_deinit(void *h_vdec)
{
	struct vdec_vp9_direct_inst *inst = h_vdec;

	vdec_vp9_free(inst);
	kfree(inst);
}

static int vdec_vp9_direct_get_param(void *h_vdec, enum vdec_get_param_type type,
				     void *out)
{
	struct vdec_vp9_direct_inst *inst = h_vdec;
	struct mtk_vcodec_dec_ctx *ctx = inst->ctx;
	struct vdec_pic_info *pic = &ctx->picinfo;
	struct v4l2_rect *crop;

	switch (type) {
	case GET_PARAM_PIC_INFO:
		/* superblocks of 64x64, MM21 tiles */
		pic->buf_w = ALIGN(pic->pic_w, VDEC_VP9_ALIGN);
		pic->buf_h = ALIGN(pic->pic_h, VDEC_VP9_ALIGN);
		pic->fb_sz[0] = pic->buf_w * pic->buf_h;
		if (ctx->is_10bit_bitstream)
			pic->fb_sz[0] = pic->fb_sz[0] * 5 / 4;
		pic->fb_sz[1] = pic->fb_sz[0] / 2;
		*(struct vdec_pic_info *)out = *pic;
		break;
	case GET_PARAM_DPB_SIZE:
		*(unsigned int *)out = VP9_NUM_REF_SLOTS + 1;
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

const struct vdec_common_if vdec_vp9_slice_direct_if = {
	.init		= vdec_vp9_direct_init,
	.decode		= vdec_vp9_direct_decode,
	.get_param	= vdec_vp9_direct_get_param,
	.deinit		= vdec_vp9_direct_deinit,
};
