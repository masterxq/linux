// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2020 BayLibre
 * Copyright (c) 2022 MediaTek Inc.
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/iommu.h>
#include <linux/iopoll.h>
#include <linux/ktime.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/property.h>
#include <linux/debugfs.h>
#include <linux/dma-mapping.h>
#include <linux/seq_file.h>
#include <linux/slab.h>

#include "mtk_camsv.h"
#include "mtk_camsv_hw.h"
#include "mtk_camsv_reg.h"
#include "mtk_seninf.h"


#define INT_ST_MASK_CAMSV (CAMSV_IRQ_PASS1_DON)
#define CAMSV_IRQ_SOF	BIT(7)	/* MT6771/MT8183 SV_SOF_INT_ST */

/*
 * Only the pixel format the video node starts with: the format set on the
 * node selects the output (VIDIOC_S_FMT).
 */
static unsigned int out_mode;
module_param(out_mode, uint, 0444);
MODULE_PARM_DESC(out_mode, "initial pixel format of the video node: 0=8-bit Bayer (default), 1=MediaTek packed 10-bit Bayer");

#define INT_ST_MASK_CAMSV_ERR \
	(CAMSV_IRQ_TG_ERR | CAMSV_IRQ_TG_GBERR | CAMSV_IRQ_IMGO_ERR | \
	 CAMSV_IRQ_IMGO_OVERR)

#define MTK_CAMSV_AUTOSUSPEND_DELAY_MS 100

/*
 * Error interrupts within one frame after which they are masked until the
 * frame ends: an error bit can re-assert continuously.
 */
#define MTK_CAMSV_FRAME_ERR_MAX	16

/*
 * Stop: the TG finishes the frame in progress after VF off. Wait up to a
 * few frames of a slow mode (vendor: three VSYNCs) for it to go idle.
 */
#define MTK_CAMSV_TG_IDLE_TIMEOUT_US	500000
#define MTK_CAMSV_IMGO_RST_TIMEOUT_US	10000

static const struct mtk_cam_conf camsv_conf = {
	.imgo_xsize = CAMSV_IMGO_SV_XSIZE,
	.imgo_ysize = CAMSV_IMGO_SV_YSIZE,
	.imgo_stride = CAMSV_IMGO_SV_STRIDE,
	.imgo_con_reg = CAMSV_IMGO_SV_CON,
	.imgo_con2_reg = CAMSV_IMGO_SV_CON2,
	.has_fbc = true,
	.tg_sen_mode = CAMSV_TG_SEN_MODE_DBL_DATA_BUS_EN,
	.module_en = CAMSV_MODULE_EN_DB_EN | CAMSV_MODULE_EN_TG_EN,
	.imgo_con = ((0x8  << CAMSV_IMGO_SV_CON_MAX_BURST_LEN_SHIFT) &
		CAMSV_IMGO_SV_CON_MAX_BURST_LEN_MASK) |
		((0x80 << CAMSV_IMGO_SV_CON_FIFO_SIZE_SHIFT) &
		CAMSV_IMGO_SV_CON_FIFO_SIZE_MASK),
	.imgo_con2 = ((0x2 << CAMSV_IMGO_SV_CON2_FIFO_PREP_PRI_THRL_SHIFT) &
		CAMSV_IMGO_SV_CON2_FIFO_PREP_PRI_THRL_MASK) |
		((0x2 << CAMSV_IMGO_SV_CON2_FIFO_PREP_PRI_THRH_SHIFT) |
		CAMSV_IMGO_SV_CON2_FIFO_PREP_PRI_THRH_MASK),
};

/*
 * MT8183: same camsv family as MT8365 (top/TG blocks are relocated through the
 * DT reg windows: top = base + 0x510, TG = base + 0x220, IMGO = base + 0x20),
 * but the IMGO DMA uses the ISP5 sub-layout and a different FBC (left off).
 * 1-pixel mode (no DBL_DATA_BUS), matching seninf.
 */
static const struct mtk_cam_conf camsv_8183_conf = {
	.imgo_xsize = CAMSV_8183_IMGO_XSIZE,
	.imgo_ysize = CAMSV_8183_IMGO_YSIZE,
	.imgo_stride = CAMSV_8183_IMGO_STRIDE,
	.imgo_con_reg = CAMSV_8183_IMGO_CON,
	.imgo_con2_reg = CAMSV_8183_IMGO_CON2,
	.has_fbc = false,
	.pak_reversed = true,
	.tg_sen_mode = 0,
	.module_en = CAMSV_MODULE_EN_DB_EN | CAMSV_MODULE_EN_TG_EN,
	.imgo_con = ((0x8  << CAMSV_IMGO_SV_CON_MAX_BURST_LEN_SHIFT) &
		CAMSV_IMGO_SV_CON_MAX_BURST_LEN_MASK) |
		((0x80 << CAMSV_IMGO_SV_CON_FIFO_SIZE_SHIFT) &
		CAMSV_IMGO_SV_CON_FIFO_SIZE_MASK),
	.imgo_con2 = 0x00800080,	/* keep the reset value */
};

/*
 * MT6785: the CAMSV blocks are the MT8183 (ISP 5.0) design at the same offsets
 * (MODULE_EN +0x510, IMGO_BASE_ADDR +0x20, TG_SEN_MODE +0x230, FBC_IMGO_CTL1/2
 * +0x110/+0x114), so the MT8183 setup applies. The vendor kernel enables the
 * CAMSV0 and CAMSV1 gates of camsys for every CAMSV.
 */
static const struct mtk_cam_conf camsv_6785_conf = {
	.imgo_xsize = CAMSV_8183_IMGO_XSIZE,
	.imgo_ysize = CAMSV_8183_IMGO_YSIZE,
	.imgo_stride = CAMSV_8183_IMGO_STRIDE,
	.imgo_con_reg = CAMSV_8183_IMGO_CON,
	.imgo_con2_reg = CAMSV_8183_IMGO_CON2,
	.has_fbc = false,
	.pak_reversed = true,
	.pak_pix_mode = true,
	.all_clks = true,
	.tg_idle_stop = true,
	.tg_sen_mode = 0,
	.module_en = CAMSV_MODULE_EN_DB_EN | CAMSV_MODULE_EN_TG_EN,
	.imgo_con = ((0x8  << CAMSV_IMGO_SV_CON_MAX_BURST_LEN_SHIFT) &
		CAMSV_IMGO_SV_CON_MAX_BURST_LEN_MASK) |
		((0x80 << CAMSV_IMGO_SV_CON_FIFO_SIZE_SHIFT) &
		CAMSV_IMGO_SV_CON_FIFO_SIZE_MASK),
	/*
	 * DMA FIFO priority thresholds (vendor values):
	 * the reset value (0x80 of a 0x80 FIFO) raises the priority only
	 * when the FIFO is already full, too late for the 2-pixel stream.
	 */
	.imgo_con2 = 0x00020002,
	.imgo_con3 = 0x00020002,
	.imgo_con3_reg = CAMSV_8183_IMGO_CON3,
};

static u32 mtk_camsv_read(struct mtk_cam_dev *priv, u32 reg)
{
	return readl(priv->regs + reg);
}

static void mtk_camsv_write(struct mtk_cam_dev *priv, u32 reg, u32 value)
{
	writel(value, priv->regs + reg);
}

static void mtk_camsv_img0_write(struct mtk_cam_dev *priv, u32 reg, u32 value)
{
	writel(value, priv->regs_img0 + reg);
}

static u32 mtk_camsv_tg_read(struct mtk_cam_dev *priv, u32 reg)
{
	return readl(priv->regs_tg + reg);
}

static void mtk_camsv_tg_write(struct mtk_cam_dev *priv, u32 reg, u32 value)
{
	writel(value, priv->regs_tg + reg);
}

void mtk_camsv_update_buffers_add(struct mtk_cam_dev *cam_dev,
				  struct mtk_cam_dev_buffer *buf)
{
	mtk_camsv_img0_write(cam_dev, CAMSV_IMGO_SV_BASE_ADDR, buf->daddr);
}

static void mtk_camsv_set_addr(struct mtk_cam_dev *cam_dev, dma_addr_t addr)
{
	mtk_camsv_img0_write(cam_dev, CAMSV_IMGO_SV_BASE_ADDR, (u32)addr);
}

/*
 * No-FBC buffer handling (MT8183), called with buf_list_lock held:
 * take the next queued buffer as the one the DMA writes the next frame into.
 * Without one the DMA is parked on the scratch buffer, or, when there is no
 * scratch buffer (no CMA left for a second copy of a large frame), keeps
 * writing into the buffer it has and that frame is dropped. With
 * MODULE_EN.DB_EN the address is latched at the next frame start.
 * Returns false when the DMA stays on the current buffer.
 */
static bool mtk_camsv_arm_next(struct mtk_cam_dev *cam_dev)
{
	struct mtk_cam_dev_buffer *buf;

	buf = list_first_entry_or_null(&cam_dev->buf_list,
				       struct mtk_cam_dev_buffer, list);
	if (buf) {
		list_del(&buf->list);
		cam_dev->cur = buf;
		mtk_camsv_set_addr(cam_dev, buf->daddr);
	} else if (cam_dev->scratch_va) {
		cam_dev->cur = NULL;
		mtk_camsv_set_addr(cam_dev, cam_dev->scratch_dma);
	} else {
		return false;
	}

	return true;
}

/*
 * Reset the CAMSV and let the IMGO DMA finish its outstanding writes
 * (IMGO_RST_ST answers the trigger once the DMA is idle).
 */
static int mtk_camsv_reset(struct mtk_cam_dev *cam_dev)
{
	u32 tmp;
	int ret;

	mtk_camsv_write(cam_dev, CAMSV_SW_CTL, CAMSV_SW_RST);
	mtk_camsv_write(cam_dev, CAMSV_SW_CTL, 0x0U);
	mtk_camsv_write(cam_dev, CAMSV_SW_CTL, CAMSV_IMGO_RST_TRIG);
	ret = readl_poll_timeout(cam_dev->regs + CAMSV_SW_CTL, tmp,
				 tmp == (CAMSV_IMGO_RST_TRIG | CAMSV_IMGO_RST_ST),
				 10, MTK_CAMSV_IMGO_RST_TIMEOUT_US);
	mtk_camsv_write(cam_dev, CAMSV_SW_CTL, 0x0U);
	if (ret)
		dev_err(cam_dev->dev,
			"IMGO DMA does not go idle: SW_CTL %08x, MODULE_EN %08x INT_EN %08x CLK_EN %08x TG_SEN_MODE %08x TG_INTER_ST %08x\n",
			tmp,
			mtk_camsv_read(cam_dev, CAMSV_MODULE_EN),
			mtk_camsv_read(cam_dev, CAMSV_INT_EN),
			mtk_camsv_read(cam_dev, CAMSV_CLK_EN),
			mtk_camsv_tg_read(cam_dev, CAMSV_TG_SEN_MODE),
			mtk_camsv_tg_read(cam_dev, CAMSV_5_TG_INTER_ST));

	return ret;
}

void mtk_camsv_cmos_vf_hw_enable(struct mtk_cam_dev *cam_dev)
{
	unsigned int fbc_val;
	u32 clk_en = CAMSV_TG_DP_CLK_EN | CAMSV_DMA_DP_CLK_EN |
		     CAMSV_PAK_DP_CLK_EN;

	if (cam_dev->conf->has_fbc) {
		fbc_val = mtk_camsv_read(cam_dev, CAMSV_IMGO_FBC);
		fbc_val |= CAMSV_IMGO_FBC_EN;
		mtk_camsv_write(cam_dev, CAMSV_IMGO_FBC, fbc_val);
	}

	mtk_camsv_write(cam_dev,  CAMSV_CLK_EN, clk_en);
	mtk_camsv_tg_write(cam_dev, CAMSV_TG_SEN_MODE,
			     mtk_camsv_tg_read(cam_dev, CAMSV_TG_SEN_MODE) |
			     CAMSV_TG_SEN_MODE_CMOS_EN);
	mtk_camsv_tg_write(cam_dev, CAMSV_TG_VF_CON,
			     mtk_camsv_tg_read(cam_dev, CAMSV_TG_VF_CON) |
			     CAMSV_TG_VF_CON_VFDATA_EN);
	dev_dbg(cam_dev->dev,
		 "stream on: MODULE_EN %08x FMT_SEL %08x PAK %08x CLK_EN %08x TG_SEN_MODE %08x VF_CON %08x GRAB %08x/%08x\n",
		 mtk_camsv_read(cam_dev, CAMSV_MODULE_EN),
		 mtk_camsv_read(cam_dev, CAMSV_FMT_SEL),
		 mtk_camsv_read(cam_dev, CAMSV_PAK),
		 mtk_camsv_read(cam_dev, CAMSV_CLK_EN),
		 mtk_camsv_tg_read(cam_dev, CAMSV_TG_SEN_MODE),
		 mtk_camsv_tg_read(cam_dev, CAMSV_TG_VF_CON),
		 mtk_camsv_tg_read(cam_dev, CAMSV_TG_SEN_GRAB_PXL),
		 mtk_camsv_tg_read(cam_dev, CAMSV_TG_SEN_GRAB_LIN));
	dev_dbg(cam_dev->dev, "stream on: IMGO base %08x xsize %08x ysize %08x stride %08x con %08x con2 %08x\n",
		 readl(cam_dev->regs_img0 + CAMSV_IMGO_SV_BASE_ADDR),
		 readl(cam_dev->regs_img0 + cam_dev->conf->imgo_xsize),
		 readl(cam_dev->regs_img0 + cam_dev->conf->imgo_ysize),
		 readl(cam_dev->regs_img0 + cam_dev->conf->imgo_stride),
		 readl(cam_dev->regs_img0 + cam_dev->conf->imgo_con_reg),
		 readl(cam_dev->regs_img0 + cam_dev->conf->imgo_con2_reg));
}

/*
 * Stop the CAMSV while the source still sends, as the vendor driver does:
 * with VF off the TG ends the frame in progress and goes idle, then the
 * reset waits for the IMGO DMA to finish its writes. Only after that may the
 * source stop and the clocks go off; cutting the DMA off in the middle of a
 * frame leaves the larb with a write that never completes, and every later
 * stream of either CAMSV on that larb fails with IMGO overruns.
 */
void mtk_camsv_cmos_vf_hw_disable(struct mtk_cam_dev *cam_dev)
{
	unsigned int fbc_val;
	ktime_t start = ktime_get();
	u32 st = 0;
	int ret = 0;

	mtk_camsv_tg_write(cam_dev, CAMSV_TG_VF_CON,
			   mtk_camsv_tg_read(cam_dev, CAMSV_TG_VF_CON) &
			   ~CAMSV_TG_VF_CON_VFDATA_EN);

	if (cam_dev->conf->tg_idle_stop) {
		ret = readl_poll_timeout(cam_dev->regs_tg + CAMSV_5_TG_INTER_ST,
					 st,
					 FIELD_GET(CAMSV_5_TG_INTER_ST_STATE, st) ==
					 CAMSV_5_TG_INTER_ST_IDLE,
					 500, MTK_CAMSV_TG_IDLE_TIMEOUT_US);
		if (ret)
			dev_warn(cam_dev->dev,
				 "TG not idle after VF off: TG_INTER_ST %08x\n",
				 st);
		mtk_camsv_write(cam_dev, CAMSV_INT_EN, 0);
		mtk_camsv_reset(cam_dev);
		mtk_camsv_write(cam_dev, CAMSV_MODULE_EN,
				mtk_camsv_read(cam_dev, CAMSV_MODULE_EN) &
				~CAMSV_MODULE_EN_IMGO_EN);
	}

	mtk_camsv_tg_write(cam_dev, CAMSV_TG_SEN_MODE,
			   mtk_camsv_tg_read(cam_dev, CAMSV_TG_SEN_MODE) &
			   ~CAMSV_TG_SEN_MODE_CMOS_EN);
	if (cam_dev->conf->has_fbc) {
		fbc_val = mtk_camsv_read(cam_dev, CAMSV_IMGO_FBC);
		fbc_val &= ~CAMSV_IMGO_FBC_EN;
		mtk_camsv_write(cam_dev, CAMSV_IMGO_FBC, fbc_val);
	}
	dev_dbg(cam_dev->dev, "stream off: irqs %u frames %u scratch %u errors %u in %u frames, last status %08x, TG idle after %lld us (%08x)\n",
		cam_dev->irq_cnt, cam_dev->done_cnt, cam_dev->scratch_cnt,
		cam_dev->err_cnt, cam_dev->err_frames, cam_dev->last_status,
		ktime_us_delta(ktime_get(), start), st);
}

/*
 * Camsv module is able to handle underrruns using the FBC
 *
 *     - FBC_FB_NUM: the number of userspace buffers available (fifo depth)
 *     - RCNT_INC: user writes 1 when buffer is queued/unqueued
 *     - FBC_CNT: decreased at SOF indicating that the number of buffers filled
 *       by write-DMA decreases. Increased when write-dma done indicating that
 *       the number of buffers filled by write-DMA increases
 *     - RCNT: increased at the end of enqueue indicating that the software
 *       moves to the next buffer for reading
 *     - WCNT: increased to indicate that write-dma writes to the next buffer
 *     Drop condition: FBC_CNT == FBC_NUM
 */

void mtk_camsv_fbc_init(struct mtk_cam_dev *cam_dev,
				 unsigned int num_buffers)
{
	unsigned int fbc_val;

	if (!cam_dev->conf->has_fbc)
		return;

	if (pm_runtime_resume_and_get(cam_dev->dev) < 0) {
		dev_err(cam_dev->dev, "failed to get pm_runtime\n");
		return;
	}

	fbc_val = mtk_camsv_read(cam_dev, CAMSV_IMGO_FBC);
	fbc_val &= ~CAMSV_IMGO_FBC_FB_NUM;
	fbc_val |= CAMSV_IMGO_FBC_EN;
	fbc_val |= FIELD_PREP(CAMSV_IMGO_FBC_FB_NUM, num_buffers);
	mtk_camsv_write(cam_dev, CAMSV_IMGO_FBC, fbc_val);

	pm_runtime_mark_last_busy(cam_dev->dev);
	pm_runtime_put_autosuspend(cam_dev->dev);
}

void mtk_camsv_fbc_inc(struct mtk_cam_dev *cam_dev)
{
	unsigned int fbc_val;

	if (!cam_dev->conf->has_fbc)
		return;

	fbc_val = mtk_camsv_read(cam_dev, CAMSV_IMGO_FBC);
	fbc_val |= CAMSV_IMGO_FBC_RCNT_INC;
	mtk_camsv_write(cam_dev, CAMSV_IMGO_FBC, fbc_val);
	fbc_val &= ~CAMSV_IMGO_FBC_RCNT_INC;
	mtk_camsv_write(cam_dev, CAMSV_IMGO_FBC, fbc_val);
}

/*
 * The scratch buffer is optional: without it the DMA holds on to the last
 * user buffer while no other one is queued (the queue starts with one).
 */
void mtk_camsv_scratch_alloc(struct mtk_cam_dev *cam_dev, size_t size)
{
	if (cam_dev->conf->has_fbc)
		return;

	cam_dev->scratch_size = PAGE_ALIGN(size);
	cam_dev->scratch_va = dma_alloc_coherent(cam_dev->dev,
						 cam_dev->scratch_size,
						 &cam_dev->scratch_dma,
						 GFP_KERNEL | __GFP_NOWARN);
	if (!cam_dev->scratch_va)
		dev_dbg(cam_dev->dev, "no scratch buffer of %zu bytes\n",
			cam_dev->scratch_size);
}

void mtk_camsv_scratch_free(struct mtk_cam_dev *cam_dev)
{
	if (!cam_dev->scratch_va)
		return;
	dma_free_coherent(cam_dev->dev, cam_dev->scratch_size,
			  cam_dev->scratch_va, cam_dev->scratch_dma);
	cam_dev->scratch_va = NULL;
}

/*
 * The TG takes as many pixels per clock as the SENINF delivers: MT6785 picks
 * the SENINF pixel mode per stream from the sensor pixel rate (two pixels for
 * the OV64B40 at 680 MHz). Older SoCs keep the fixed mode of the SoC data.
 */
int mtk_camsv_pixel_mode(struct mtk_cam_dev *cam_dev)
{
	struct media_pad *remote;
	int mode;

	cam_dev->tg_sen_mode = cam_dev->conf->tg_sen_mode;

	remote = media_pad_remote_pad_first(&cam_dev->subdev_pads[MTK_CAM_CIO_PAD_SENINF]);
	if (!remote || !is_media_entity_v4l2_subdev(remote->entity))
		return -ENOLINK;

	mode = mtk_seninf_pixel_mode(media_entity_to_v4l2_subdev(remote->entity),
				     remote->index);
	switch (mode) {
	case -EOPNOTSUPP:
		return 0;
	case 0:
		cam_dev->tg_sen_mode &= ~CAMSV_TG_SEN_MODE_DBL_DATA_BUS_EN;
		return 0;
	case 1:
		cam_dev->tg_sen_mode |= CAMSV_TG_SEN_MODE_DBL_DATA_BUS_EN;
		return 0;
	default:
		dev_err(cam_dev->dev, "SENINF pixel mode %d not supported\n", mode);
		return mode < 0 ? mode : -EINVAL;
	}
}

void mtk_camsv_setup(struct mtk_cam_dev *cam_dev, unsigned int count,
		     u32 w, u32 h, u32 bpl,
		     const struct mtk_cam_format_info *fmtinfo)
{
	const struct mtk_cam_conf *conf = cam_dev->conf;
	struct mtk_cam_dev_buffer *buf;
	unsigned long flags;
	unsigned int i;
	u32 stride_bits, fmt_sel, tg_sen_mode, pak, bus;

	for (i = 0; i < count; i++)
		mtk_camsv_fbc_inc(cam_dev);

	tg_sen_mode = cam_dev->tg_sen_mode;
	mtk_camsv_tg_write(cam_dev, CAMSV_TG_SEN_MODE, tg_sen_mode);

	mtk_camsv_tg_write(cam_dev, CAMSV_TG_SEN_GRAB_PXL,
			     (w * fmtinfo->w_factor) << 16U);

	mtk_camsv_tg_write(cam_dev, CAMSV_TG_SEN_GRAB_LIN, h << 16U);

	/* YUV_U2S_DIS: disable YUV sensor unsigned to signed */
	mtk_camsv_tg_write(cam_dev, CAMSV_TG_PATH_CFG, 0x1000U);

	/*
	 * Reset cam, with the TG, packer and DMA clocks of the block on: after
	 * the CAM domain was off they are off, and the IMGO handshake then
	 * does not answer. A missing answer is logged, the stream still
	 * starts.
	 */
	mtk_camsv_write(cam_dev, CAMSV_CLK_EN, CAMSV_TG_DP_CLK_EN |
			CAMSV_DMA_DP_CLK_EN | CAMSV_PAK_DP_CLK_EN);
	mtk_camsv_reset(cam_dev);

	cam_dev->irq_cnt = 0;
	cam_dev->done_cnt = 0;
	cam_dev->err_cnt = 0;
	cam_dev->err_frames = 0;
	cam_dev->frame_errs = 0;
	cam_dev->frame_status = 0;
	cam_dev->scratch_cnt = 0;
	cam_dev->last_status = 0;

	mtk_camsv_write(cam_dev, CAMSV_INT_EN, INT_ST_MASK_CAMSV |
			(conf->has_fbc ? 0 : INT_ST_MASK_CAMSV_ERR));

	mtk_camsv_write(cam_dev, CAMSV_MODULE_EN,
			conf->module_en | fmtinfo->module_en_pak);
	fmt_sel = fmtinfo->fmt_sel;
	mtk_camsv_write(cam_dev, CAMSV_FMT_SEL, fmt_sel);
	pak = fmtinfo->pak;
	/* reversed mode encoding, and no double mode bit in this layout */
	if (conf->pak_reversed && (fmtinfo->module_en_pak & CAMSV_MODULE_EN_PAK_EN))
		pak = CAMSV_PAK_PAK_MODE_RAW_8 -
		      FIELD_GET(CAMSV_PAK_PAK_MODE, pak);
	/*
	 * The packer takes as many pixels per clock as the TG passes on;
	 * with a 1-pixel packer behind a 2-pixel TG the IMGO DMA reports
	 * errors and writes nothing.
	 */
	if (conf->pak_pix_mode && (tg_sen_mode & CAMSV_TG_SEN_MODE_DBL_DATA_BUS_EN))
		pak |= FIELD_PREP(CAMSV_6785_PAK_PIX_MODE, 1);
	mtk_camsv_write(cam_dev, CAMSV_PAK, pak);

	mtk_camsv_img0_write(cam_dev, conf->imgo_xsize, bpl - 1U);
	mtk_camsv_img0_write(cam_dev, conf->imgo_ysize, h - 1U);

	stride_bits = fmtinfo->imgo_stride;
	/*
	 * MT6785: the DMA bus is as wide as the pixels the TG passes on per
	 * clock (2 or 4 bytes for RAW), with its enable in bit 24. With the
	 * MT8365 layout (enable in bit 19) a 2-pixel stream on the 4-byte bus
	 * gives IMGO errors.
	 */
	if (conf->pak_pix_mode) {
		bus = tg_sen_mode & CAMSV_TG_SEN_MODE_DBL_DATA_BUS_EN ? 3 : 1;
		stride_bits = CAMSV_6785_IMGO_STRIDE_BUS_SIZE_EN |
			      FIELD_PREP(CAMSV_6785_IMGO_STRIDE_BUS_SIZE, bus);
	}
	mtk_camsv_img0_write(cam_dev, conf->imgo_stride, stride_bits | bpl);

	mtk_camsv_img0_write(cam_dev, conf->imgo_con_reg, conf->imgo_con);
	mtk_camsv_img0_write(cam_dev, conf->imgo_con2_reg, conf->imgo_con2);
	if (conf->imgo_con3_reg)
		mtk_camsv_img0_write(cam_dev, conf->imgo_con3_reg,
				     conf->imgo_con3);

	/* Set buf addr */
	spin_lock_irqsave(&cam_dev->buf_list_lock, flags);
	if (conf->has_fbc) {
		buf = list_first_entry_or_null(&cam_dev->buf_list,
					       struct mtk_cam_dev_buffer,
					       list);
		if (buf)
			mtk_camsv_update_buffers_add(cam_dev, buf);
	} else {
		mtk_camsv_arm_next(cam_dev);
	}
	spin_unlock_irqrestore(&cam_dev->buf_list_lock, flags);

	dev_dbg(cam_dev->dev, "setup %ux%u bpl %u mbus 0x%x %p4cc fmt_sel %08x pak %08x stride %08x tg_sen_mode %08x\n",
		w, h, bpl, fmtinfo->code, &fmtinfo->fourcc, fmt_sel, pak,
		stride_bits | bpl, tg_sen_mode);

	/* CMOS_EN first */
	mtk_camsv_tg_write(cam_dev, CAMSV_TG_SEN_MODE,
			     mtk_camsv_tg_read(cam_dev, CAMSV_TG_SEN_MODE) |
			     CAMSV_TG_SEN_MODE_CMOS_EN);

	/* finally, CAMSV_MODULE_EN : IMGO_EN */
	mtk_camsv_write(cam_dev, CAMSV_MODULE_EN,
			  mtk_camsv_read(cam_dev, CAMSV_MODULE_EN) |
			  CAMSV_MODULE_EN_IMGO_EN);
}

static irqreturn_t isp_irq_camsv(int irq, void *data)
{
	struct mtk_cam_dev *cam_dev = (struct mtk_cam_dev *)data;
	struct mtk_cam_dev_buffer *processed_buf, *buf;
	unsigned int irq_status;

	irq_status = mtk_camsv_read(cam_dev, CAMSV_INT_STATUS);
	cam_dev->irq_cnt++;
	cam_dev->last_status = irq_status;

	/*
	 * Errors are counted per frame and reported when it ends. An IMGO
	 * overrun (the DMA FIFO overflowed: the memory did not take the
	 * line data in time) comes in the middle of a frame, mostly while
	 * other engines load the memory; the frame is delivered.
	 */
	if (irq_status & INT_ST_MASK_CAMSV_ERR) {
		cam_dev->err_cnt++;
		cam_dev->frame_status |= irq_status & INT_ST_MASK_CAMSV_ERR;
		/* error bits can re-assert continuously: quiet until the frame ends */
		if (++cam_dev->frame_errs == MTK_CAMSV_FRAME_ERR_MAX && !cam_dev->conf->has_fbc)
			mtk_camsv_write(cam_dev, CAMSV_INT_EN, INT_ST_MASK_CAMSV);
	}

	if (cam_dev->irq_cnt <= 5)
		dev_dbg(cam_dev->dev, "irq #%u status %08x TG_INTER_ST %08x\n",
			 cam_dev->irq_cnt, irq_status,
			 mtk_camsv_tg_read(cam_dev, 0x4c));

	if (!(irq_status & CAMSV_IRQ_PASS1_DON))
		return IRQ_HANDLED;

	if (cam_dev->frame_errs) {
		cam_dev->err_frames++;
		dev_warn_ratelimited(cam_dev->dev,
				     "frame %u: %u error interrupts (status %08x)\n",
				     cam_dev->sequence + 1, cam_dev->frame_errs,
				     cam_dev->frame_status);
		if (cam_dev->frame_errs >= MTK_CAMSV_FRAME_ERR_MAX &&
		    !cam_dev->conf->has_fbc)
			mtk_camsv_write(cam_dev, CAMSV_INT_EN,
					INT_ST_MASK_CAMSV | INT_ST_MASK_CAMSV_ERR);
		cam_dev->frame_errs = 0;
		cam_dev->frame_status = 0;
	}

	if (!cam_dev->conf->has_fbc) {
		spin_lock(&cam_dev->buf_list_lock);
		cam_dev->sequence++;
		processed_buf = cam_dev->cur;
		if (!mtk_camsv_arm_next(cam_dev))
			processed_buf = NULL;	/* held, the frame is dropped */
		if (!processed_buf)
			cam_dev->scratch_cnt++;
		spin_unlock(&cam_dev->buf_list_lock);

		if (processed_buf) {
			cam_dev->done_cnt++;
			processed_buf->v4l2_buf.sequence = cam_dev->sequence;
			processed_buf->v4l2_buf.vb2_buf.timestamp = ktime_get_ns();
			vb2_buffer_done(&processed_buf->v4l2_buf.vb2_buf,
					VB2_BUF_STATE_DONE);
		}
		return IRQ_HANDLED;
	}

	/* De-queue frame */
	spin_lock(&cam_dev->buf_list_lock);
	cam_dev->sequence++;

	processed_buf = list_first_entry_or_null(&cam_dev->buf_list,
						 struct mtk_cam_dev_buffer,
						 list);
	if (processed_buf)
		list_del(&processed_buf->list);

	buf = list_first_entry_or_null(&cam_dev->buf_list,
				       struct mtk_cam_dev_buffer,
				       list);

	spin_unlock(&cam_dev->buf_list_lock);

	if (processed_buf) {
		cam_dev->done_cnt++;
		processed_buf->v4l2_buf.sequence = cam_dev->sequence;
		processed_buf->v4l2_buf.vb2_buf.timestamp =
		ktime_get_ns();
		vb2_buffer_done(&processed_buf->v4l2_buf.vb2_buf,
				VB2_BUF_STATE_DONE);
		mtk_camsv_fbc_inc(cam_dev);
	}
	/*
	 * If there is no user buffer available, hardware will drop
	 * automatically frames until buf_queue is called.
	 */
	if (buf)
		mtk_camsv_update_buffers_add(cam_dev, buf);

	return IRQ_HANDLED;
}

static int mtk_camsv_stats_show(struct seq_file *m, void *unused)
{
	struct mtk_cam_dev *cam_dev = m->private;

	seq_printf(m, "irqs %u frames %u scratch %u errors %u error frames %u last_status %08x sequence %u\n",
		   cam_dev->irq_cnt, cam_dev->done_cnt, cam_dev->scratch_cnt,
		   cam_dev->err_cnt, cam_dev->err_frames, cam_dev->last_status,
		   cam_dev->sequence);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(mtk_camsv_stats);

static int mtk_camsv_regs_show(struct seq_file *m, void *unused)
{
	struct mtk_cam_dev *cam_dev = m->private;
	unsigned int i;

	if (!pm_runtime_get_if_in_use(cam_dev->dev)) {
		seq_puts(m, "not powered (only readable while streaming)\n");
		return 0;
	}
	seq_puts(m, "top (base+0x510):");
	for (i = 0; i < 0x40; i += 4)
		seq_printf(m, "%s%08x", i % 16 ? " " : "\n  ", readl(cam_dev->regs + i));
	seq_puts(m, "\nimgo (base+0x20):");
	for (i = 0; i < 0x28; i += 4)
		seq_printf(m, "%s%08x", i % 16 ? " " : "\n  ", readl(cam_dev->regs_img0 + i));
	seq_puts(m, "\ntg (base+0x220):");
	for (i = 0; i < 0x90; i += 4)
		seq_printf(m, "%s%08x", i % 16 ? " " : "\n  ", readl(cam_dev->regs_tg + i));
	seq_puts(m, "\n");
	pm_runtime_put(cam_dev->dev);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(mtk_camsv_regs);

/* The reference of the driver goes with the unbind or a failed probe. */
static void mtk_camsv_put(void *data)
{
	mtk_cam_dev_put(data);
}

/*
 * Drains the deferred puts before the module goes (the last one frees the
 * structure with code of the module).
 */
static struct workqueue_struct *mtk_camsv_wq;

/*
 * Each pending put stands for a reference still held, so only the last put
 * of the loop can free the structure (and the work item in it).
 */
static void mtk_camsv_put_work(struct work_struct *work)
{
	struct mtk_cam_dev *cam_dev =
		container_of(work, struct mtk_cam_dev, put_work);
	int n = atomic_xchg(&cam_dev->deferred_puts, 0);

	while (n-- > 0)
		mtk_cam_dev_put(cam_dev);
}

void mtk_cam_dev_put_deferred(struct mtk_cam_dev *cam_dev)
{
	atomic_inc(&cam_dev->deferred_puts);
	queue_work(mtk_camsv_wq, &cam_dev->put_work);
}

static int mtk_camsv_probe(struct platform_device *pdev)
{
	static const char * const clk_names[] = { "cam", "camtg", "camsv"};

	struct mtk_cam_dev *cam_dev;
	struct device *dev = &pdev->dev;
	unsigned int i;
	int ret;

	if (!device_iommu_mapped(dev))
		return -EPROBE_DEFER;

	/*
	 * Open files of the video and the sub-device node keep the structure
	 * after the unbind, see struct mtk_cam_dev.
	 */
	cam_dev = kzalloc_obj(*cam_dev);
	if (!cam_dev)
		return -ENOMEM;
	kref_init(&cam_dev->kref);
	INIT_WORK(&cam_dev->put_work, mtk_camsv_put_work);
	ret = devm_add_action_or_reset(dev, mtk_camsv_put, cam_dev);
	if (ret)
		return ret;

	cam_dev->conf = device_get_match_data(dev);
	if (!cam_dev->conf)
		return -ENODEV;

	cam_dev->dev = dev;
	dev_set_drvdata(dev, cam_dev);

	/* IMGO_BASE_ADDR is a 32-bit IOVA */
	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return dev_err_probe(dev, ret, "failed to set DMA mask\n");

	if (out_mode > 1)
		dev_warn(dev, "out_mode %u not supported, 8-bit output\n",
			 out_mode);
	cam_dev->default_fmt = out_mode == 1 ? V4L2_PIX_FMT_MTISP_SBGGR10 :
			       V4L2_PIX_FMT_SBGGR8;

	cam_dev->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(cam_dev->regs))
		return dev_err_probe(dev, PTR_ERR(cam_dev->regs),
				     "failed to map register base\n");

	cam_dev->regs_img0 = devm_platform_ioremap_resource(pdev, 1);
	if (IS_ERR(cam_dev->regs_img0))
		return dev_err_probe(dev, PTR_ERR(cam_dev->regs_img0),
				     "failed to map img0 register base\n");

	cam_dev->regs_tg = devm_platform_ioremap_resource(pdev, 2);
	if (IS_ERR(cam_dev->regs_tg))
		return dev_err_probe(dev, PTR_ERR(cam_dev->regs_tg),
				     "failed to map TG register base\n");

	if (cam_dev->conf->all_clks) {
		ret = devm_clk_bulk_get_all(dev, &cam_dev->clks);
		if (ret < 0)
			return dev_err_probe(dev, ret, "failed to get clocks\n");
		cam_dev->num_clks = ret;
	} else {
		cam_dev->num_clks = ARRAY_SIZE(clk_names);
		cam_dev->clks = devm_kcalloc(dev, cam_dev->num_clks,
					     sizeof(*cam_dev->clks), GFP_KERNEL);
		if (!cam_dev->clks)
			return -ENOMEM;

		for (i = 0; i < cam_dev->num_clks; ++i)
			cam_dev->clks[i].id = clk_names[i];

		ret = devm_clk_bulk_get(dev, cam_dev->num_clks, cam_dev->clks);
		if (ret)
			return dev_err_probe(dev, ret, "failed to get clocks: %i\n",
					     ret);
	}

	cam_dev->irq = platform_get_irq(pdev, 0);
	ret = devm_request_irq(dev, cam_dev->irq, isp_irq_camsv, 0,
			       dev_name(dev), cam_dev);
	if (ret)
		return dev_err_probe(dev, ret, "failed to request irq=%d\n",
				     cam_dev->irq);

	spin_lock_init(&cam_dev->buf_list_lock);

	/* initialise runtime power management */
	pm_runtime_set_autosuspend_delay(dev, MTK_CAMSV_AUTOSUSPEND_DELAY_MS);
	pm_runtime_use_autosuspend(dev);
	pm_runtime_set_suspended(dev);
	devm_pm_runtime_enable(dev);

	cam_dev->dbg = debugfs_create_dir(dev_name(dev), NULL);
	debugfs_create_file("stats", 0444, cam_dev->dbg, cam_dev,
			    &mtk_camsv_stats_fops);
	debugfs_create_file("regs", 0400, cam_dev->dbg, cam_dev,
			    &mtk_camsv_regs_fops);

	dev_info(dev, "camsv irq %u fbc %s initial format %p4cc\n",
		 cam_dev->irq,
		 cam_dev->conf->has_fbc ? "on" : "off (scratch buffer)",
		 &cam_dev->default_fmt);

	/* Initialize the v4l2 common part */
	ret = mtk_cam_dev_init(cam_dev);
	if (ret)
		debugfs_remove_recursive(cam_dev->dbg);
	return ret;
}

static void mtk_camsv_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mtk_cam_dev *cam_dev = dev_get_drvdata(dev);
	int ret;

	debugfs_remove_recursive(cam_dev->dbg);
	/* open files keep the nodes and the structure, not the hardware */
	mtk_cam_dev_cleanup(cam_dev);

	/*
	 * Off now: after a stream the block stays on for the autosuspend
	 * delay, and runtime PM is disabled by the devm action of
	 * devm_pm_runtime_enable(), which would leave it on with the clocks
	 * of the device prepared. A stream that still holds a reference is a
	 * bug of the stop path.
	 */
	pm_runtime_dont_use_autosuspend(dev);
	ret = pm_runtime_suspend(dev);
	if (ret < 0)
		dev_warn(dev, "block not off at the unbind: %d\n", ret);
}

static int mtk_camsv_runtime_suspend(struct device *dev)
{
	struct mtk_cam_dev *cam_dev = dev_get_drvdata(dev);

	clk_bulk_disable_unprepare(cam_dev->num_clks, cam_dev->clks);

	return 0;
}

static int mtk_camsv_runtime_resume(struct device *dev)
{
	struct mtk_cam_dev *cam_dev = dev_get_drvdata(dev);
	int ret;

	ret = clk_bulk_prepare_enable(cam_dev->num_clks, cam_dev->clks);
	if (ret) {
		dev_err(dev, "failed to enable clock:%d\n", ret);
		return ret;
	}
	/* registers kept from the last stream if the CAM domain stayed on */
	dev_dbg(dev, "runtime resume: MODULE_EN %08x SW_CTL %08x CLK_EN %08x\n",
		mtk_camsv_read(cam_dev, CAMSV_MODULE_EN),
		mtk_camsv_read(cam_dev, CAMSV_SW_CTL),
		mtk_camsv_read(cam_dev, CAMSV_CLK_EN));

	return 0;
}

/*
 * System sleep with a running stream. The CAM power domain goes off with the
 * system and takes every register of the CAMSV, the seninf and the sensor
 * with it, while runtime PM still counts the stream. As the bridge of the
 * pipeline the CAMSV stops the stream from the CAMSV down to the sensor
 * before the sleep and starts it again after it, with the configuration and
 * the buffers of the stream. The device links of the seninf order the
 * callbacks: the CAMSV suspends before the seninf and the seninf before the
 * sensor, and they resume the other way round.
 *
 * The buffer the DMA was writing into when the stream stopped holds an
 * incomplete frame: it goes back to the head of the queue and is written
 * again. User space is frozen in between, so no ioctl changes the stream;
 * the lock of the video node is taken anyway, in the order of an ioctl.
 */
static void mtk_camsv_sleep_requeue(struct mtk_cam_dev *cam_dev)
{
	unsigned long flags;

	spin_lock_irqsave(&cam_dev->buf_list_lock, flags);
	if (cam_dev->cur) {
		list_add(&cam_dev->cur->list, &cam_dev->buf_list);
		cam_dev->cur = NULL;
	}
	spin_unlock_irqrestore(&cam_dev->buf_list_lock, flags);
}

static void mtk_camsv_sleep_stop(struct mtk_cam_dev *cam_dev)
{
	mtk_camsv_cmos_vf_hw_disable(cam_dev);
	/* no handler may touch the block or the buffers after this */
	synchronize_irq(cam_dev->irq);
	v4l2_subdev_disable_streams(&cam_dev->subdev,
				    cam_dev->subdev_pads[MTK_CAM_CIO_PAD_VIDEO].index,
				    BIT(0));
	mtk_camsv_sleep_requeue(cam_dev);
}

static int mtk_camsv_sleep_restart(struct mtk_cam_dev *cam_dev)
{
	struct mtk_cam_video_device *vdev = cam_dev->vdev;
	unsigned int count = 0;
	unsigned long flags;
	int ret;

	if (cam_dev->conf->has_fbc) {
		mtk_camsv_fbc_init(cam_dev, vb2_get_num_buffers(&vdev->vbq));
		spin_lock_irqsave(&cam_dev->buf_list_lock, flags);
		count = list_count_nodes(&cam_dev->buf_list);
		spin_unlock_irqrestore(&cam_dev->buf_list_lock, flags);
	}

	mtk_camsv_setup(cam_dev, count, cam_dev->stream_width,
			cam_dev->stream_height, cam_dev->stream_bpl,
			cam_dev->stream_fmtinfo);
	mtk_camsv_cmos_vf_hw_enable(cam_dev);

	ret = v4l2_subdev_enable_streams(&cam_dev->subdev,
					 cam_dev->subdev_pads[MTK_CAM_CIO_PAD_VIDEO].index,
					 BIT(0));
	if (!ret)
		return 0;

	/*
	 * The queue still streams for user space: give the buffers back
	 * with an error and fail the queue, STREAMOFF cleans up.
	 */
	dev_err(cam_dev->dev, "stream not restarted after the system sleep: %d\n",
		ret);
	mtk_camsv_cmos_vf_hw_disable(cam_dev);
	cam_dev->streaming = false;
	mtk_cam_vb2_return_all_buffers(cam_dev, VB2_BUF_STATE_ERROR);
	vb2_queue_error(&vdev->vbq);

	return ret;
}

static int mtk_camsv_suspend(struct device *dev)
{
	struct mtk_cam_dev *cam_dev = dev_get_drvdata(dev);
	struct mtk_cam_video_device *vdev = cam_dev->vdev;
	bool stopped = false;
	int ret;

	if (vdev) {
		mutex_lock(&vdev->vdev_lock);
		if (cam_dev->streaming) {
			mtk_camsv_sleep_stop(cam_dev);
			stopped = true;
		}
		mutex_unlock(&vdev->vdev_lock);
	}

	ret = pm_runtime_force_suspend(dev);
	if (ret && stopped) {
		/* the sleep is aborted without a resume of this device */
		mutex_lock(&vdev->vdev_lock);
		mtk_camsv_sleep_restart(cam_dev);
		mutex_unlock(&vdev->vdev_lock);
	}

	return ret;
}

static int mtk_camsv_resume(struct device *dev)
{
	struct mtk_cam_dev *cam_dev = dev_get_drvdata(dev);
	struct mtk_cam_video_device *vdev = cam_dev->vdev;
	int ret;

	ret = pm_runtime_force_resume(dev);
	if (ret)
		return ret;

	if (vdev) {
		mutex_lock(&vdev->vdev_lock);
		if (cam_dev->streaming)
			mtk_camsv_sleep_restart(cam_dev);
		mutex_unlock(&vdev->vdev_lock);
	}

	/* a failed restart is reported to user space through the queue */
	return 0;
}

static const struct dev_pm_ops mtk_camsv_pm_ops = {
	SYSTEM_SLEEP_PM_OPS(mtk_camsv_suspend, mtk_camsv_resume)
	RUNTIME_PM_OPS(mtk_camsv_runtime_suspend, mtk_camsv_runtime_resume,
		       NULL)
};

static const struct of_device_id mtk_camsv_of_ids[] = {
	{ .compatible = "mediatek,mt8365-camsv", .data = &camsv_conf },
	{ .compatible = "mediatek,mt8183-camsv", .data = &camsv_8183_conf },
	{ .compatible = "mediatek,mt6785-camsv", .data = &camsv_6785_conf },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mtk_camsv_of_ids);

static struct platform_driver mtk_camsv_driver = {
	.probe = mtk_camsv_probe,
	.remove = mtk_camsv_remove,
	.driver = {
		.name = "mtk-camsv-isp",
		.of_match_table = mtk_camsv_of_ids,
		.pm = pm_ptr(&mtk_camsv_pm_ops),
	}
};

static int __init mtk_camsv_init(void)
{
	int ret;

	mtk_camsv_wq = alloc_workqueue("mtk-camsv", WQ_UNBOUND, 0);
	if (!mtk_camsv_wq)
		return -ENOMEM;

	ret = platform_driver_register(&mtk_camsv_driver);
	if (ret)
		destroy_workqueue(mtk_camsv_wq);

	return ret;
}
module_init(mtk_camsv_init);

static void __exit mtk_camsv_exit(void)
{
	platform_driver_unregister(&mtk_camsv_driver);
	destroy_workqueue(mtk_camsv_wq);
}
module_exit(mtk_camsv_exit);

MODULE_DESCRIPTION("MediaTek CAMSV ISP3.0 driver");
MODULE_AUTHOR("Florian Sylvestre <fsylvestre@baylibre.com>");
MODULE_LICENSE("GPL");
