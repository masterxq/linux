// SPDX-License-Identifier: GPL-2.0-only
/*
 * MediaTek MT6785 ISP pass 2 (DIP): media device and video nodes.
 *
 * One processing entity with a video node per stream, as described in
 * <linux/media/mediatek/mtk-dip-mt6785.h>: raw input (Bayer or NV12),
 * parameters, main output, scaled output, and the base image and weight map
 * of the blend. A job takes one buffer from every streaming node, in queue
 * order.
 */

#include <linux/align.h>
#include <linux/minmax.h>
#include <linux/pm_runtime.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-dma-contig.h>
#include <media/videobuf2-vmalloc.h>

#include "mtk_dip.h"

#define MTK_DIP_NAME			"mtk-dip"
/* raw line step, as the CAMSV writes it */
#define MTK_DIP_RAW_BPL_ALIGN		2
/* output stride step */
#define MTK_DIP_OUT_BPL_ALIGN		16
/*
 * Longest line of the caller: a line of 16-bit pixels of the widest frame,
 * which also keeps the stride registers (16 bits) and the buffer sizes in
 * range. Longer lines only waste memory.
 */
#define MTK_DIP_RAW_BPL_MAX		ALIGN(2 * MTK_DIP_MAX_W, \
					      MTK_DIP_RAW_BPL_ALIGN)
#define MTK_DIP_OUT_BPL_MAX		ALIGN(2 * MTK_DIP_MAX_W, \
					      MTK_DIP_OUT_BPL_ALIGN)

struct mtk_dip_raw_fmt {
	u32 fourcc;
	u8 bpp;
	u8 pix_id;
};

/* NV12 last: the YUV input of the blend jobs */
static const struct mtk_dip_raw_fmt mtk_dip_raw_fmts[] = {
	{ V4L2_PIX_FMT_SBGGR8, 8, MTK_DIP_PIX_ID_BGGR },
	{ V4L2_PIX_FMT_MTISP_SBGGR10, 10, MTK_DIP_PIX_ID_BGGR },
	{ V4L2_PIX_FMT_SGBRG8, 8, MTK_DIP_PIX_ID_GBRG },
	{ V4L2_PIX_FMT_SGRBG8, 8, MTK_DIP_PIX_ID_GRBG },
	{ V4L2_PIX_FMT_SRGGB8, 8, MTK_DIP_PIX_ID_RGGB },
	{ V4L2_PIX_FMT_MTISP_SGBRG10, 10, MTK_DIP_PIX_ID_GBRG },
	{ V4L2_PIX_FMT_MTISP_SGRBG10, 10, MTK_DIP_PIX_ID_GRBG },
	{ V4L2_PIX_FMT_MTISP_SRGGB10, 10, MTK_DIP_PIX_ID_RGGB },
	{ V4L2_PIX_FMT_NV12, 8, 0 },
};

struct mtk_dip_node_desc {
	const char *name;
	u32 buf_type;
	u32 caps;
	bool output;
};

static const struct mtk_dip_node_desc mtk_dip_node_desc[MTK_DIP_NODE_NUM] = {
	[MTK_DIP_NODE_RAW] = {
		"mtk-dip-raw", V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
		V4L2_CAP_VIDEO_OUTPUT_MPLANE, true,
	},
	[MTK_DIP_NODE_PARAMS] = {
		"mtk-dip-params", V4L2_BUF_TYPE_META_OUTPUT,
		V4L2_CAP_META_OUTPUT, true,
	},
	[MTK_DIP_NODE_MAIN] = {
		"mtk-dip-main", V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
		V4L2_CAP_VIDEO_CAPTURE_MPLANE, false,
	},
	[MTK_DIP_NODE_SCALED] = {
		"mtk-dip-scaled", V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
		V4L2_CAP_VIDEO_CAPTURE_MPLANE, false,
	},
	[MTK_DIP_NODE_BASE] = {
		"mtk-dip-base", V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
		V4L2_CAP_VIDEO_OUTPUT_MPLANE, true,
	},
	[MTK_DIP_NODE_WEIGHT] = {
		"mtk-dip-weight", V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
		V4L2_CAP_VIDEO_OUTPUT_MPLANE, true,
	},
};

static const struct mtk_dip_raw_fmt *mtk_dip_raw_fmt(u32 fourcc)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(mtk_dip_raw_fmts); i++)
		if (mtk_dip_raw_fmts[i].fourcc == fourcc)
			return &mtk_dip_raw_fmts[i];

	return NULL;
}

bool mtk_dip_raw_is_10bit(const struct v4l2_pix_format_mplane *pix)
{
	const struct mtk_dip_raw_fmt *rf = mtk_dip_raw_fmt(pix->pixelformat);

	return rf && rf->bpp == 10;
}

/* Bayer phase of a raw format, 0 for the YUV input */
u32 mtk_dip_raw_pix_id(const struct v4l2_pix_format_mplane *pix)
{
	const struct mtk_dip_raw_fmt *rf = mtk_dip_raw_fmt(pix->pixelformat);

	return rf ? rf->pix_id : 0;
}

bool mtk_dip_raw_is_yuv(const struct v4l2_pix_format_mplane *pix)
{
	return pix->pixelformat == V4L2_PIX_FMT_NV12;
}

/**
 * mtk_dip_fit_width - largest output width up to @w the tiles serve
 * @w: wanted width
 * @max: largest width
 *
 * Even widths outside the bands of widths the tile computation cannot serve.
 */
u32 mtk_dip_fit_width(u32 w, u32 max)
{
	w = clamp(ALIGN_DOWN(w, 2), MTK_DIP_MIN_W, ALIGN_DOWN(max, 2));

	return mtk_dip_tile_fit_width(w, MTK_DIP_MIN_W) ?: MTK_DIP_MIN_W;
}

/*
 * Whole height, widest width the tiles serve, centred; the whole frame of a
 * YUV input (its width is one the tiles serve)
 */
static void mtk_dip_default_crop(const struct v4l2_pix_format_mplane *pix,
				 struct v4l2_rect *r)
{
	if (mtk_dip_raw_is_yuv(pix)) {
		r->left = 0;
		r->top = 0;
		r->width = pix->width;
		r->height = pix->height;
		return;
	}
	r->width = mtk_dip_fit_width(pix->width, pix->width);
	r->height = pix->height;
	r->left = ALIGN_DOWN((pix->width - r->width) / 2, 2);
	r->top = 0;
}

/*
 * NV12 in one plane (Y, then interleaved UV): a width the tiles serve with a
 * YUV input, even height, the line a multiple of 16
 */
static void mtk_dip_try_nv12(struct v4l2_pix_format_mplane *pix, u32 w_max,
			     bool fit)
{
	u32 bpl, w;

	pix->pixelformat = V4L2_PIX_FMT_NV12;
	w = clamp(ALIGN_DOWN(pix->width, 2), MTK_DIP_MIN_W, w_max);
	if (fit)
		w = mtk_dip_tile_fit_width_yuv(w, MTK_DIP_MIN_W) ?: MTK_DIP_MIN_W;
	pix->width = w;
	pix->height = clamp(ALIGN(pix->height, 2), MTK_DIP_MIN_H,
			    MTK_DIP_MAX_H);
	pix->field = V4L2_FIELD_NONE;
	pix->num_planes = 1;
	pix->colorspace = V4L2_COLORSPACE_SRGB;
	pix->xfer_func = V4L2_XFER_FUNC_DEFAULT;
	pix->ycbcr_enc = V4L2_YCBCR_ENC_601;
	pix->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	pix->flags = 0;

	bpl = ALIGN(pix->plane_fmt[0].bytesperline, MTK_DIP_OUT_BPL_ALIGN);
	pix->plane_fmt[0].bytesperline =
		clamp(bpl, ALIGN(pix->width, MTK_DIP_OUT_BPL_ALIGN),
		      MTK_DIP_OUT_BPL_MAX);
	pix->plane_fmt[0].sizeimage = pix->plane_fmt[0].bytesperline *
				      pix->height * 3 / 2;
	memset(pix->plane_fmt[0].reserved, 0,
	       sizeof(pix->plane_fmt[0].reserved));
	memset(pix->reserved, 0, sizeof(pix->reserved));
}

/* Weight map: one byte per pixel, the line a multiple of 16 */
static void mtk_dip_try_weight(struct v4l2_pix_format_mplane *pix)
{
	u32 bpl;

	pix->pixelformat = V4L2_PIX_FMT_GREY;
	pix->width = clamp(ALIGN_DOWN(pix->width, 2), MTK_DIP_MIN_W,
			   MTK_DIP_MAX_W);
	pix->height = clamp(ALIGN(pix->height, 2), MTK_DIP_MIN_H,
			    MTK_DIP_MAX_H);
	pix->field = V4L2_FIELD_NONE;
	pix->num_planes = 1;
	pix->colorspace = V4L2_COLORSPACE_RAW;
	pix->xfer_func = V4L2_XFER_FUNC_NONE;
	pix->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	pix->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	pix->flags = 0;

	bpl = ALIGN(pix->plane_fmt[0].bytesperline, MTK_DIP_OUT_BPL_ALIGN);
	pix->plane_fmt[0].bytesperline =
		clamp(bpl, ALIGN(pix->width, MTK_DIP_OUT_BPL_ALIGN),
		      MTK_DIP_OUT_BPL_MAX);
	pix->plane_fmt[0].sizeimage = pix->plane_fmt[0].bytesperline *
				      pix->height;
	memset(pix->plane_fmt[0].reserved, 0,
	       sizeof(pix->plane_fmt[0].reserved));
	memset(pix->reserved, 0, sizeof(pix->reserved));
}

static void mtk_dip_try_raw(struct v4l2_pix_format_mplane *pix)
{
	const struct mtk_dip_raw_fmt *rf = mtk_dip_raw_fmt(pix->pixelformat);
	u32 min_bpl, bpl;

	if (rf && rf->fourcc == V4L2_PIX_FMT_NV12) {
		mtk_dip_try_nv12(pix, MTK_DIP_MAX_W, true);
		return;
	}
	if (!rf)
		rf = &mtk_dip_raw_fmts[0];
	pix->pixelformat = rf->fourcc;
	pix->width = clamp(ALIGN(pix->width, 2), MTK_DIP_MIN_W, MTK_DIP_MAX_W);
	pix->height = clamp(ALIGN(pix->height, 2), MTK_DIP_MIN_H,
			    MTK_DIP_MAX_H);
	pix->field = V4L2_FIELD_NONE;
	pix->num_planes = 1;
	pix->colorspace = V4L2_COLORSPACE_RAW;
	pix->xfer_func = V4L2_XFER_FUNC_NONE;
	pix->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	pix->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	pix->flags = 0;

	/* a longer line of the caller is kept if it has the alignment */
	min_bpl = ALIGN(DIV_ROUND_UP(pix->width * rf->bpp, 8),
			MTK_DIP_RAW_BPL_ALIGN);
	bpl = ALIGN(pix->plane_fmt[0].bytesperline, MTK_DIP_RAW_BPL_ALIGN);
	pix->plane_fmt[0].bytesperline = clamp(bpl, min_bpl,
					       MTK_DIP_RAW_BPL_MAX);
	pix->plane_fmt[0].sizeimage = pix->plane_fmt[0].bytesperline *
				      pix->height;
	memset(pix->plane_fmt[0].reserved, 0,
	       sizeof(pix->plane_fmt[0].reserved));
	memset(pix->reserved, 0, sizeof(pix->reserved));
}

static void mtk_dip_try_main(struct v4l2_pix_format_mplane *pix)
{
	u32 bpl;

	pix->pixelformat = V4L2_PIX_FMT_NV12;
	pix->width = mtk_dip_fit_width(pix->width, MTK_DIP_MAX_W);
	pix->height = clamp(ALIGN(pix->height, 2), MTK_DIP_MIN_H,
			    MTK_DIP_MAX_H);
	pix->field = V4L2_FIELD_NONE;
	pix->num_planes = 1;
	pix->colorspace = V4L2_COLORSPACE_SRGB;
	pix->xfer_func = V4L2_XFER_FUNC_DEFAULT;
	pix->ycbcr_enc = V4L2_YCBCR_ENC_601;
	pix->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	pix->flags = 0;

	bpl = ALIGN(pix->plane_fmt[0].bytesperline, MTK_DIP_OUT_BPL_ALIGN);
	pix->plane_fmt[0].bytesperline =
		clamp(bpl, ALIGN(pix->width, MTK_DIP_OUT_BPL_ALIGN),
		      MTK_DIP_OUT_BPL_MAX);
	pix->plane_fmt[0].sizeimage = pix->plane_fmt[0].bytesperline *
				      pix->height * 3 / 2;
	memset(pix->plane_fmt[0].reserved, 0,
	       sizeof(pix->plane_fmt[0].reserved));
	memset(pix->reserved, 0, sizeof(pix->reserved));
}

/* The processed area of the raw node, a copy taken under the device lock */
static void mtk_dip_get_crop(struct mtk_dip *dip, struct v4l2_rect *r)
{
	unsigned long flags;

	spin_lock_irqsave(&dip->lock, flags);
	*r = dip->node[MTK_DIP_NODE_RAW].crop;
	spin_unlock_irqrestore(&dip->lock, flags);
}

static void mtk_dip_set_crop(struct mtk_dip_node *raw, const struct v4l2_rect *r)
{
	unsigned long flags;

	spin_lock_irqsave(&raw->dip->lock, flags);
	raw->crop = *r;
	spin_unlock_irqrestore(&raw->dip->lock, flags);
}

/*
 * Scaled output: YUYV, 1/2 down to 1/16 of the processed area @crop on each
 * axis, even sizes; widths the tile rule cannot serve with this area round
 * down
 */
static void mtk_dip_try_scaled(const struct v4l2_rect *crop,
			       struct v4l2_pix_format_mplane *pix)
{
	u32 min_w, max_w, min_h, max_h, w, bpl;

	min_w = max_t(u32, MTK_DIP_SCALED_MIN_W,
		      ALIGN(DIV_ROUND_UP(crop->width, MTK_DIP_SCALE_MAX_DOWN), 2));
	max_w = max(ALIGN_DOWN(crop->width / 2, 2), min_w);
	min_h = max_t(u32, MTK_DIP_SCALED_MIN_H,
		      ALIGN(DIV_ROUND_UP(crop->height, MTK_DIP_SCALE_MAX_DOWN), 2));
	max_h = max(ALIGN_DOWN(crop->height / 2, 2), min_h);

	pix->pixelformat = V4L2_PIX_FMT_YUYV;
	pix->width = clamp(ALIGN_DOWN(pix->width, 2), min_w, max_w);
	pix->height = clamp(ALIGN_DOWN(pix->height, 2), min_h, max_h);
	w = mtk_dip_tile_fit_scale_width(crop->width, crop->height, pix->width,
					 pix->height, min_w);
	if (w)
		pix->width = w;
	pix->field = V4L2_FIELD_NONE;
	pix->num_planes = 1;
	pix->colorspace = V4L2_COLORSPACE_SRGB;
	pix->xfer_func = V4L2_XFER_FUNC_DEFAULT;
	pix->ycbcr_enc = V4L2_YCBCR_ENC_601;
	pix->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	pix->flags = 0;

	bpl = ALIGN(pix->plane_fmt[0].bytesperline, MTK_DIP_OUT_BPL_ALIGN);
	pix->plane_fmt[0].bytesperline =
		clamp(bpl, ALIGN(2 * pix->width, MTK_DIP_OUT_BPL_ALIGN),
		      MTK_DIP_OUT_BPL_MAX);
	pix->plane_fmt[0].sizeimage = pix->plane_fmt[0].bytesperline *
				      pix->height;
	memset(pix->plane_fmt[0].reserved, 0,
	       sizeof(pix->plane_fmt[0].reserved));
	memset(pix->reserved, 0, sizeof(pix->reserved));
}

/* ioctls */

static int mtk_dip_querycap(struct file *file, void *fh,
			    struct v4l2_capability *cap)
{
	struct mtk_dip_node *node = video_drvdata(file);

	strscpy(cap->driver, MTK_DIP_NAME, sizeof(cap->driver));
	strscpy(cap->card, "mtk-dip", sizeof(cap->card));
	snprintf(cap->bus_info, sizeof(cap->bus_info), "platform:%s",
		 dev_name(node->dip->dev));

	return 0;
}

static int mtk_dip_enum_fmt_raw(struct file *file, void *fh,
				struct v4l2_fmtdesc *f)
{
	const struct mtk_dip_raw_fmt *rf;

	if (f->mbus_code || f->index >= ARRAY_SIZE(mtk_dip_raw_fmts))
		return -EINVAL;
	rf = &mtk_dip_raw_fmts[f->index];
	f->pixelformat = rf->fourcc;

	return 0;
}

static int mtk_dip_enum_fmt_main(struct file *file, void *fh,
				 struct v4l2_fmtdesc *f)
{
	if (f->mbus_code || f->index)
		return -EINVAL;
	f->pixelformat = V4L2_PIX_FMT_NV12;

	return 0;
}

static int mtk_dip_enum_fmt_scaled(struct file *file, void *fh,
				   struct v4l2_fmtdesc *f)
{
	if (f->mbus_code || f->index)
		return -EINVAL;
	f->pixelformat = V4L2_PIX_FMT_YUYV;

	return 0;
}

static int mtk_dip_enum_fmt_meta(struct file *file, void *fh,
				 struct v4l2_fmtdesc *f)
{
	if (f->mbus_code || f->index)
		return -EINVAL;
	f->pixelformat = V4L2_META_FMT_MTK_DIP_PARAMS;
	strscpy(f->description, "MediaTek DIP parameters",
		sizeof(f->description));

	return 0;
}

/*
 * Even sizes; the tile computation leaves out a few bands of widths, which
 * TRY_FMT and S_SELECTION round down.
 */
static int mtk_dip_enum_framesizes(struct file *file, void *fh,
				   struct v4l2_frmsizeenum *fs)
{
	struct mtk_dip_node *node = video_drvdata(file);

	if (fs->index)
		return -EINVAL;
	if (node->id == MTK_DIP_NODE_SCALED) {
		/* within these, 1/2 to 1/16 of the processed area */
		if (fs->pixel_format != V4L2_PIX_FMT_YUYV)
			return -EINVAL;
		fs->type = V4L2_FRMSIZE_TYPE_STEPWISE;
		fs->stepwise.min_width = MTK_DIP_SCALED_MIN_W;
		fs->stepwise.max_width = MTK_DIP_SCALED_MAX_W;
		fs->stepwise.step_width = 2;
		fs->stepwise.min_height = MTK_DIP_SCALED_MIN_H;
		fs->stepwise.max_height = MTK_DIP_SCALED_MAX_H;
		fs->stepwise.step_height = 2;
		return 0;
	}
	switch (node->id) {
	case MTK_DIP_NODE_MAIN:
	case MTK_DIP_NODE_BASE:
		if (fs->pixel_format != V4L2_PIX_FMT_NV12)
			return -EINVAL;
		break;
	case MTK_DIP_NODE_WEIGHT:
		if (fs->pixel_format != V4L2_PIX_FMT_GREY)
			return -EINVAL;
		break;
	default:
		if (!mtk_dip_raw_fmt(fs->pixel_format))
			return -EINVAL;
		break;
	}

	fs->type = V4L2_FRMSIZE_TYPE_STEPWISE;
	fs->stepwise.min_width = MTK_DIP_MIN_W;
	fs->stepwise.max_width = MTK_DIP_MAX_W;
	fs->stepwise.step_width = 2;
	fs->stepwise.min_height = MTK_DIP_MIN_H;
	fs->stepwise.max_height = MTK_DIP_MAX_H;
	fs->stepwise.step_height = 2;

	return 0;
}

static int mtk_dip_g_fmt(struct file *file, void *fh, struct v4l2_format *f)
{
	struct mtk_dip_node *node = video_drvdata(file);

	*f = node->fmt;

	return 0;
}

static int mtk_dip_try_fmt_raw(struct file *file, void *fh,
			       struct v4l2_format *f)
{
	mtk_dip_try_raw(&f->fmt.pix_mp);

	return 0;
}

static int mtk_dip_s_fmt_raw(struct file *file, void *fh,
			     struct v4l2_format *f)
{
	struct mtk_dip_node *node = video_drvdata(file);
	struct v4l2_rect crop;

	if (vb2_is_busy(&node->queue))
		return -EBUSY;
	mtk_dip_try_raw(&f->fmt.pix_mp);
	node->fmt = *f;
	mtk_dip_default_crop(&node->fmt.fmt.pix_mp, &crop);
	mtk_dip_set_crop(node, &crop);

	return 0;
}

static int mtk_dip_try_fmt_main(struct file *file, void *fh,
				struct v4l2_format *f)
{
	mtk_dip_try_main(&f->fmt.pix_mp);

	return 0;
}

static int mtk_dip_s_fmt_main(struct file *file, void *fh,
			      struct v4l2_format *f)
{
	struct mtk_dip_node *node = video_drvdata(file);

	if (vb2_is_busy(&node->queue))
		return -EBUSY;
	mtk_dip_try_main(&f->fmt.pix_mp);
	node->fmt = *f;

	return 0;
}

static int mtk_dip_try_fmt_scaled(struct file *file, void *fh,
				  struct v4l2_format *f)
{
	struct mtk_dip_node *node = video_drvdata(file);
	struct v4l2_rect crop;

	mtk_dip_get_crop(node->dip, &crop);
	mtk_dip_try_scaled(&crop, &f->fmt.pix_mp);

	return 0;
}

static int mtk_dip_s_fmt_scaled(struct file *file, void *fh,
				struct v4l2_format *f)
{
	struct mtk_dip_node *node = video_drvdata(file);

	if (vb2_is_busy(&node->queue))
		return -EBUSY;
	mtk_dip_try_fmt_scaled(file, fh, f);
	node->fmt = *f;

	return 0;
}

static int mtk_dip_try_fmt_meta(struct file *file, void *fh,
				struct v4l2_format *f)
{
	f->fmt.meta.dataformat = V4L2_META_FMT_MTK_DIP_PARAMS;
	f->fmt.meta.buffersize = MTK_DIP_PARAMS_MAX_SIZE;

	return 0;
}

static int mtk_dip_s_fmt_meta(struct file *file, void *fh,
			      struct v4l2_format *f)
{
	struct mtk_dip_node *node = video_drvdata(file);

	if (vb2_is_busy(&node->queue))
		return -EBUSY;
	mtk_dip_try_fmt_meta(file, fh, f);
	node->fmt = *f;

	return 0;
}

static int mtk_dip_enum_fmt_in(struct file *file, void *fh,
			       struct v4l2_fmtdesc *f)
{
	struct mtk_dip_node *node = video_drvdata(file);

	if (f->mbus_code || f->index)
		return -EINVAL;
	f->pixelformat = node->id == MTK_DIP_NODE_BASE ? V4L2_PIX_FMT_NV12 :
			 V4L2_PIX_FMT_GREY;

	return 0;
}

static void mtk_dip_try_in(struct mtk_dip_node *node,
			   struct v4l2_pix_format_mplane *pix)
{
	if (node->id == MTK_DIP_NODE_BASE)
		mtk_dip_try_nv12(pix, MTK_DIP_MAX_W, false);
	else
		mtk_dip_try_weight(pix);
}

static int mtk_dip_try_fmt_in(struct file *file, void *fh,
			      struct v4l2_format *f)
{
	mtk_dip_try_in(video_drvdata(file), &f->fmt.pix_mp);

	return 0;
}

static int mtk_dip_s_fmt_in(struct file *file, void *fh,
			    struct v4l2_format *f)
{
	struct mtk_dip_node *node = video_drvdata(file);

	if (vb2_is_busy(&node->queue))
		return -EBUSY;
	mtk_dip_try_in(node, &f->fmt.pix_mp);
	node->fmt = *f;

	return 0;
}

static int mtk_dip_g_selection(struct file *file, void *fh,
			       struct v4l2_selection *s)
{
	struct mtk_dip_node *node = video_drvdata(file);
	const struct v4l2_pix_format_mplane *pix = &node->fmt.fmt.pix_mp;

	if (s->type != V4L2_BUF_TYPE_VIDEO_OUTPUT)
		return -EINVAL;

	switch (s->target) {
	case V4L2_SEL_TGT_CROP:
		s->r = node->crop;
		return 0;
	case V4L2_SEL_TGT_CROP_DEFAULT:
		mtk_dip_default_crop(pix, &s->r);
		return 0;
	case V4L2_SEL_TGT_CROP_BOUNDS:
		s->r.left = 0;
		s->r.top = 0;
		s->r.width = pix->width;
		s->r.height = pix->height;
		return 0;
	}

	return -EINVAL;
}

static int mtk_dip_s_selection(struct file *file, void *fh,
			       struct v4l2_selection *s)
{
	struct mtk_dip_node *node = video_drvdata(file);
	const struct v4l2_pix_format_mplane *pix = &node->fmt.fmt.pix_mp;
	struct v4l2_rect r = s->r;

	if (s->type != V4L2_BUF_TYPE_VIDEO_OUTPUT ||
	    s->target != V4L2_SEL_TGT_CROP)
		return -EINVAL;
	if (vb2_is_busy(&node->queue))
		return -EBUSY;

	/* a YUV input is processed as a whole */
	if (mtk_dip_raw_is_yuv(pix)) {
		mtk_dip_default_crop(pix, &r);
		mtk_dip_set_crop(node, &r);
		s->r = r;
		return 0;
	}

	/* even position keeps the Bayer phase */
	r.left = ALIGN_DOWN(clamp_t(s32, r.left, 0,
				    pix->width - MTK_DIP_MIN_W), 2);
	r.top = ALIGN_DOWN(clamp_t(s32, r.top, 0,
				   pix->height - MTK_DIP_MIN_H), 2);
	r.width = mtk_dip_fit_width(min(r.width, pix->width - r.left),
				    pix->width - r.left);
	r.height = clamp(ALIGN_DOWN(r.height, 2), MTK_DIP_MIN_H,
			 pix->height - r.top);

	mtk_dip_set_crop(node, &r);
	s->r = r;

	return 0;
}

static const struct v4l2_ioctl_ops mtk_dip_raw_ioctl_ops = {
	.vidioc_querycap = mtk_dip_querycap,
	.vidioc_enum_fmt_vid_out = mtk_dip_enum_fmt_raw,
	.vidioc_g_fmt_vid_out_mplane = mtk_dip_g_fmt,
	.vidioc_try_fmt_vid_out_mplane = mtk_dip_try_fmt_raw,
	.vidioc_s_fmt_vid_out_mplane = mtk_dip_s_fmt_raw,
	.vidioc_enum_framesizes = mtk_dip_enum_framesizes,
	.vidioc_g_selection = mtk_dip_g_selection,
	.vidioc_s_selection = mtk_dip_s_selection,
	.vidioc_reqbufs = vb2_ioctl_reqbufs,
	.vidioc_create_bufs = vb2_ioctl_create_bufs,
	.vidioc_prepare_buf = vb2_ioctl_prepare_buf,
	.vidioc_querybuf = vb2_ioctl_querybuf,
	.vidioc_qbuf = vb2_ioctl_qbuf,
	.vidioc_dqbuf = vb2_ioctl_dqbuf,
	.vidioc_expbuf = vb2_ioctl_expbuf,
	.vidioc_streamon = vb2_ioctl_streamon,
	.vidioc_streamoff = vb2_ioctl_streamoff,
};

static const struct v4l2_ioctl_ops mtk_dip_params_ioctl_ops = {
	.vidioc_querycap = mtk_dip_querycap,
	.vidioc_enum_fmt_meta_out = mtk_dip_enum_fmt_meta,
	.vidioc_g_fmt_meta_out = mtk_dip_g_fmt,
	.vidioc_try_fmt_meta_out = mtk_dip_try_fmt_meta,
	.vidioc_s_fmt_meta_out = mtk_dip_s_fmt_meta,
	.vidioc_reqbufs = vb2_ioctl_reqbufs,
	.vidioc_create_bufs = vb2_ioctl_create_bufs,
	.vidioc_prepare_buf = vb2_ioctl_prepare_buf,
	.vidioc_querybuf = vb2_ioctl_querybuf,
	.vidioc_qbuf = vb2_ioctl_qbuf,
	.vidioc_dqbuf = vb2_ioctl_dqbuf,
	.vidioc_expbuf = vb2_ioctl_expbuf,
	.vidioc_streamon = vb2_ioctl_streamon,
	.vidioc_streamoff = vb2_ioctl_streamoff,
};

static const struct v4l2_ioctl_ops mtk_dip_main_ioctl_ops = {
	.vidioc_querycap = mtk_dip_querycap,
	.vidioc_enum_fmt_vid_cap = mtk_dip_enum_fmt_main,
	.vidioc_enum_framesizes = mtk_dip_enum_framesizes,
	.vidioc_g_fmt_vid_cap_mplane = mtk_dip_g_fmt,
	.vidioc_try_fmt_vid_cap_mplane = mtk_dip_try_fmt_main,
	.vidioc_s_fmt_vid_cap_mplane = mtk_dip_s_fmt_main,
	.vidioc_reqbufs = vb2_ioctl_reqbufs,
	.vidioc_create_bufs = vb2_ioctl_create_bufs,
	.vidioc_prepare_buf = vb2_ioctl_prepare_buf,
	.vidioc_querybuf = vb2_ioctl_querybuf,
	.vidioc_qbuf = vb2_ioctl_qbuf,
	.vidioc_dqbuf = vb2_ioctl_dqbuf,
	.vidioc_expbuf = vb2_ioctl_expbuf,
	.vidioc_streamon = vb2_ioctl_streamon,
	.vidioc_streamoff = vb2_ioctl_streamoff,
};

static const struct v4l2_ioctl_ops mtk_dip_scaled_ioctl_ops = {
	.vidioc_querycap = mtk_dip_querycap,
	.vidioc_enum_fmt_vid_cap = mtk_dip_enum_fmt_scaled,
	.vidioc_enum_framesizes = mtk_dip_enum_framesizes,
	.vidioc_g_fmt_vid_cap_mplane = mtk_dip_g_fmt,
	.vidioc_try_fmt_vid_cap_mplane = mtk_dip_try_fmt_scaled,
	.vidioc_s_fmt_vid_cap_mplane = mtk_dip_s_fmt_scaled,
	.vidioc_reqbufs = vb2_ioctl_reqbufs,
	.vidioc_create_bufs = vb2_ioctl_create_bufs,
	.vidioc_prepare_buf = vb2_ioctl_prepare_buf,
	.vidioc_querybuf = vb2_ioctl_querybuf,
	.vidioc_qbuf = vb2_ioctl_qbuf,
	.vidioc_dqbuf = vb2_ioctl_dqbuf,
	.vidioc_expbuf = vb2_ioctl_expbuf,
	.vidioc_streamon = vb2_ioctl_streamon,
	.vidioc_streamoff = vb2_ioctl_streamoff,
};

static const struct v4l2_ioctl_ops mtk_dip_in_ioctl_ops = {
	.vidioc_querycap = mtk_dip_querycap,
	.vidioc_enum_fmt_vid_out = mtk_dip_enum_fmt_in,
	.vidioc_enum_framesizes = mtk_dip_enum_framesizes,
	.vidioc_g_fmt_vid_out_mplane = mtk_dip_g_fmt,
	.vidioc_try_fmt_vid_out_mplane = mtk_dip_try_fmt_in,
	.vidioc_s_fmt_vid_out_mplane = mtk_dip_s_fmt_in,
	.vidioc_reqbufs = vb2_ioctl_reqbufs,
	.vidioc_create_bufs = vb2_ioctl_create_bufs,
	.vidioc_prepare_buf = vb2_ioctl_prepare_buf,
	.vidioc_querybuf = vb2_ioctl_querybuf,
	.vidioc_qbuf = vb2_ioctl_qbuf,
	.vidioc_dqbuf = vb2_ioctl_dqbuf,
	.vidioc_expbuf = vb2_ioctl_expbuf,
	.vidioc_streamon = vb2_ioctl_streamon,
	.vidioc_streamoff = vb2_ioctl_streamoff,
};

static const struct v4l2_ioctl_ops *mtk_dip_ioctl_ops[MTK_DIP_NODE_NUM] = {
	[MTK_DIP_NODE_RAW] = &mtk_dip_raw_ioctl_ops,
	[MTK_DIP_NODE_PARAMS] = &mtk_dip_params_ioctl_ops,
	[MTK_DIP_NODE_MAIN] = &mtk_dip_main_ioctl_ops,
	[MTK_DIP_NODE_SCALED] = &mtk_dip_scaled_ioctl_ops,
	[MTK_DIP_NODE_BASE] = &mtk_dip_in_ioctl_ops,
	[MTK_DIP_NODE_WEIGHT] = &mtk_dip_in_ioctl_ops,
};

static const struct v4l2_file_operations mtk_dip_fops = {
	.owner = THIS_MODULE,
	.open = v4l2_fh_open,
	.release = vb2_fop_release,
	.poll = vb2_fop_poll,
	.unlocked_ioctl = video_ioctl2,
	.mmap = vb2_fop_mmap,
};

/* vb2 */

static u32 mtk_dip_node_size(const struct mtk_dip_node *node)
{
	if (node->id == MTK_DIP_NODE_PARAMS)
		return node->fmt.fmt.meta.buffersize;

	return node->fmt.fmt.pix_mp.plane_fmt[0].sizeimage;
}

/*
 * Largest buffer VIDIOC_CREATE_BUFS may ask for. Bigger buffers are of no use
 * to the block, but a buffer larger than the format is valid, and
 * v4l2-compliance creates one 1 MiB above the format: twice the largest
 * image format, and for the small params buffers their size plus 1 MiB.
 */
static u32 mtk_dip_node_size_max(const struct mtk_dip_node *node)
{
	switch (node->id) {
	case MTK_DIP_NODE_RAW:
		return 2 * MTK_DIP_RAW_BPL_MAX * MTK_DIP_MAX_H;
	case MTK_DIP_NODE_MAIN:
	case MTK_DIP_NODE_BASE:
		return 3 * MTK_DIP_OUT_BPL_MAX * MTK_DIP_MAX_H;
	case MTK_DIP_NODE_WEIGHT:
		return 2 * MTK_DIP_OUT_BPL_MAX * MTK_DIP_MAX_H;
	case MTK_DIP_NODE_SCALED:
		return 2 * MTK_DIP_OUT_BPL_MAX * MTK_DIP_SCALED_MAX_H;
	default:
		return MTK_DIP_PARAMS_MAX_SIZE + SZ_1M;
	}
}

static int mtk_dip_queue_setup(struct vb2_queue *q, unsigned int *nbuffers,
			       unsigned int *nplanes, unsigned int sizes[],
			       struct device *alloc_devs[])
{
	struct mtk_dip_node *node = vb2_get_drv_priv(q);
	u32 size = mtk_dip_node_size(node);

	if (*nplanes)
		return *nplanes != 1 || sizes[0] < size ||
		       sizes[0] > mtk_dip_node_size_max(node) ? -EINVAL : 0;

	*nplanes = 1;
	sizes[0] = size;

	return 0;
}

static struct mtk_dip_buffer *to_mtk_dip_buffer(struct vb2_buffer *vb)
{
	return container_of(to_vb2_v4l2_buffer(vb), struct mtk_dip_buffer, vb);
}

static int mtk_dip_buf_prepare(struct vb2_buffer *vb)
{
	struct mtk_dip_node *node = vb2_get_drv_priv(vb->vb2_queue);
	struct mtk_dip *dip = node->dip;
	u32 size = mtk_dip_node_size(node);
	dma_addr_t dma;

	if (node->id == MTK_DIP_NODE_PARAMS)
		return mtk_dip_params_check(dip, vb, to_mtk_dip_buffer(vb));

	if (vb2_plane_size(vb, 0) < size) {
		dev_dbg(dip->dev, "%s: buffer of %lu bytes, %u needed\n",
			node->vdev.name, vb2_plane_size(vb, 0), size);
		return -EINVAL;
	}
	/* the frame starts at the start of the buffer */
	if (vb->planes[0].data_offset) {
		dev_dbg(dip->dev, "%s: data offset %u not supported\n",
			node->vdev.name, vb->planes[0].data_offset);
		return -EINVAL;
	}

	/* the block reaches 32-bit physical addresses only */
	dma = vb2_dma_contig_plane_dma_addr(vb, 0);
	if (upper_32_bits(dma + vb2_plane_size(vb, 0) - 1)) {
		dev_dbg(dip->dev, "%s: buffer above 4 GiB\n", node->vdev.name);
		return -EINVAL;
	}

	if (node->id == MTK_DIP_NODE_MAIN || node->id == MTK_DIP_NODE_SCALED)
		vb2_set_plane_payload(vb, 0, size);
	else if (vb2_get_plane_payload(vb, 0) < size)
		return -EINVAL;

	return 0;
}

/* frames to the block are progressive; a frame queued with any field gets none */
static int mtk_dip_buf_out_validate(struct vb2_buffer *vb)
{
	struct mtk_dip_node *node = vb2_get_drv_priv(vb->vb2_queue);
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);

	if (vbuf->field == V4L2_FIELD_ANY)
		vbuf->field = V4L2_FIELD_NONE;
	if (node->id == MTK_DIP_NODE_PARAMS)
		return 0;

	return vbuf->field == V4L2_FIELD_NONE ? 0 : -EINVAL;
}

static void mtk_dip_buf_queue(struct vb2_buffer *vb)
{
	struct mtk_dip_node *node = vb2_get_drv_priv(vb->vb2_queue);
	struct mtk_dip_buffer *buf = to_mtk_dip_buffer(vb);
	struct mtk_dip *dip = node->dip;
	unsigned long flags;

	spin_lock_irqsave(&dip->lock, flags);
	list_add_tail(&buf->list, &node->ready);
	spin_unlock_irqrestore(&dip->lock, flags);

	mtk_dip_kick(dip);
}

static void mtk_dip_return_ready(struct mtk_dip_node *node,
				 enum vb2_buffer_state state)
{
	struct mtk_dip *dip = node->dip;
	struct mtk_dip_buffer *buf, *tmp;
	unsigned long flags;
	LIST_HEAD(list);

	spin_lock_irqsave(&dip->lock, flags);
	list_splice_init(&node->ready, &list);
	spin_unlock_irqrestore(&dip->lock, flags);

	list_for_each_entry_safe(buf, tmp, &list, list) {
		list_del(&buf->list);
		vb2_buffer_done(&buf->vb.vb2_buf, state);
	}
}

/*
 * Checked when a node starts; no format and no crop changes while its queue
 * is busy. Every job the capture nodes that stream can make from now on gets
 * a tile layout: the job of each set of them with the starting node in it
 * (each set when the raw node starts, as a capture node that stops leaves a
 * smaller one), with and without edge enhancement, which the parameters
 * switch from job to job. The check takes the geometry the job takes: a job
 * does not fail in the tile computation then. The base image and the weight
 * map need a YUV input of their size; a YUV input has no scaled output.
 */
static int mtk_dip_check_link(struct mtk_dip_node *node)
{
	const unsigned int cap = BIT(MTK_DIP_NODE_MAIN) |
				 BIT(MTK_DIP_NODE_SCALED);
	const unsigned int ins = BIT(MTK_DIP_NODE_BASE) |
				 BIT(MTK_DIP_NODE_WEIGHT);
	const struct v4l2_pix_format_mplane *in =
		&node->dip->node[MTK_DIP_NODE_RAW].fmt.fmt.pix_mp;
	const struct v4l2_pix_format_mplane *out =
		&node->dip->node[MTK_DIP_NODE_MAIN].fmt.fmt.pix_mp;
	const struct v4l2_pix_format_mplane *sc =
		&node->dip->node[MTK_DIP_NODE_SCALED].fmt.fmt.pix_mp;
	const struct v4l2_pix_format_mplane *base =
		&node->dip->node[MTK_DIP_NODE_BASE].fmt.fmt.pix_mp;
	const struct v4l2_pix_format_mplane *wt =
		&node->dip->node[MTK_DIP_NODE_WEIGHT].fmt.fmt.pix_mp;
	struct mtk_dip *dip = node->dip;
	struct mtk_dip_tile_geom g;
	unsigned int outs, s, on;
	struct v4l2_rect crop;
	unsigned long flags;
	bool yuv, mix;
	int ret;

	if (node->id == MTK_DIP_NODE_PARAMS)
		return 0;

	spin_lock_irqsave(&dip->lock, flags);
	crop = dip->node[MTK_DIP_NODE_RAW].crop;
	on = dip->streaming | BIT(node->id);
	outs = on & cap;
	spin_unlock_irqrestore(&dip->lock, flags);

	yuv = mtk_dip_raw_is_yuv(in);
	if (!yuv && on & ins)
		return -EPIPE;
	if (yuv && outs & BIT(MTK_DIP_NODE_SCALED))
		return -EPIPE;
	if (on & BIT(MTK_DIP_NODE_BASE) &&
	    (base->width != in->width || base->height != in->height))
		return -EPIPE;
	if (on & BIT(MTK_DIP_NODE_WEIGHT) &&
	    (wt->width != in->width || wt->height != in->height))
		return -EPIPE;
	mix = (on & ins) == ins;

	/* each non-empty subset of the outputs */
	for (s = outs; s; s = (s - 1) & outs) {
		if (node->id != MTK_DIP_NODE_RAW &&
		    node->id != MTK_DIP_NODE_BASE &&
		    node->id != MTK_DIP_NODE_WEIGHT && !(s & BIT(node->id)))
			continue;
		ret = mtk_dip_job_geometry(in, &crop,
					   s & BIT(MTK_DIP_NODE_MAIN) ? out : NULL,
					   s & BIT(MTK_DIP_NODE_SCALED) ? sc : NULL,
					   mix ? base : NULL, mix ? wt : NULL,
					   &g);
		if (!ret)
			ret = mtk_dip_tile_check(&g);
		if (ret == -ENOMEM)
			return ret;
		if (ret) {
			dev_dbg(dip->dev,
				"no job for raw crop %ux%u with%s main %ux%u, with%s scaled %ux%u: %d\n",
				crop.width, crop.height,
				s & BIT(MTK_DIP_NODE_MAIN) ? "" : "out",
				out->width, out->height,
				s & BIT(MTK_DIP_NODE_SCALED) ? "" : "out",
				sc->width, sc->height, ret);
			return -EPIPE;
		}
	}

	return 0;
}

static int mtk_dip_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct mtk_dip_node *node = vb2_get_drv_priv(q);
	struct mtk_dip *dip = node->dip;
	unsigned long flags;
	int ret;

	ret = mtk_dip_check_link(node);
	if (ret) {
		mtk_dip_return_ready(node, VB2_BUF_STATE_QUEUED);
		return ret;
	}

	/* a new raw stream starts a new sequence of the temporal noise reduction */
	if (node->id == MTK_DIP_NODE_RAW) {
		mutex_lock(&dip->run_lock);
		mtk_dip_nr3d_restart(dip);
		mutex_unlock(&dip->run_lock);
	}

	spin_lock_irqsave(&dip->lock, flags);
	if (!dip->streaming)
		dip->sequence = 0;
	dip->streaming |= BIT(node->id);
	spin_unlock_irqrestore(&dip->lock, flags);

	mtk_dip_kick(dip);

	return 0;
}

static void mtk_dip_stop_streaming(struct vb2_queue *q)
{
	struct mtk_dip_node *node = vb2_get_drv_priv(q);
	struct mtk_dip *dip = node->dip;
	unsigned long flags;
	bool idle;

	spin_lock_irqsave(&dip->lock, flags);
	dip->streaming &= ~BIT(node->id);
	spin_unlock_irqrestore(&dip->lock, flags);

	/*
	 * Buffers not in a job yet; after the running job the reference of
	 * the temporal noise reduction (a main buffer the driver holds); then
	 * wait for the running job
	 */
	mtk_dip_return_ready(node, VB2_BUF_STATE_ERROR);
	mutex_lock(&dip->run_lock);
	mtk_dip_nr3d_drop(dip);
	mutex_unlock(&dip->run_lock);
	vb2_wait_for_all_buffers(q);

	mutex_lock(&dip->run_lock);
	spin_lock_irqsave(&dip->lock, flags);
	idle = !dip->streaming;
	spin_unlock_irqrestore(&dip->lock, flags);
	if (idle)
		mtk_dip_aux_free_all(dip);
	mutex_unlock(&dip->run_lock);
}

static const struct vb2_ops mtk_dip_vb2_ops = {
	.queue_setup = mtk_dip_queue_setup,
	.buf_out_validate = mtk_dip_buf_out_validate,
	.buf_prepare = mtk_dip_buf_prepare,
	.buf_queue = mtk_dip_buf_queue,
	.start_streaming = mtk_dip_start_streaming,
	.stop_streaming = mtk_dip_stop_streaming,
};

static void mtk_dip_node_defaults(struct mtk_dip_node *node)
{
	struct v4l2_format *f = &node->fmt;

	memset(f, 0, sizeof(*f));
	f->type = mtk_dip_node_desc[node->id].buf_type;
	switch (node->id) {
	case MTK_DIP_NODE_RAW:
		f->fmt.pix_mp.pixelformat = V4L2_PIX_FMT_SBGGR8;
		f->fmt.pix_mp.width = MTK_DIP_MAX_W;
		f->fmt.pix_mp.height = MTK_DIP_MAX_H;
		mtk_dip_try_raw(&f->fmt.pix_mp);
		mtk_dip_default_crop(&f->fmt.pix_mp, &node->crop);
		break;
	case MTK_DIP_NODE_PARAMS:
		f->fmt.meta.dataformat = V4L2_META_FMT_MTK_DIP_PARAMS;
		f->fmt.meta.buffersize = MTK_DIP_PARAMS_MAX_SIZE;
		break;
	case MTK_DIP_NODE_MAIN:
		f->fmt.pix_mp.width = MTK_DIP_MAX_W;
		f->fmt.pix_mp.height = MTK_DIP_MAX_H;
		mtk_dip_try_main(&f->fmt.pix_mp);
		break;
	case MTK_DIP_NODE_BASE:
	case MTK_DIP_NODE_WEIGHT:
		f->fmt.pix_mp.width = MTK_DIP_MAX_W;
		f->fmt.pix_mp.height = MTK_DIP_MAX_H;
		mtk_dip_try_in(node, &f->fmt.pix_mp);
		break;
	case MTK_DIP_NODE_SCALED:
		/* half of the default processed area, the full raw frame */
		f->fmt.pix_mp.width = MTK_DIP_SCALED_MAX_W;
		f->fmt.pix_mp.height = MTK_DIP_SCALED_MAX_H;
		mtk_dip_try_scaled(&node->dip->node[MTK_DIP_NODE_RAW].crop,
				   &f->fmt.pix_mp);
		break;
	default:
		break;
	}
}

static int mtk_dip_node_init(struct mtk_dip *dip, enum mtk_dip_node_id id)
{
	const struct mtk_dip_node_desc *desc = &mtk_dip_node_desc[id];
	struct mtk_dip_node *node = &dip->node[id];
	struct video_device *vdev = &node->vdev;
	struct vb2_queue *q = &node->queue;
	u32 flags;
	int ret;

	node->dip = dip;
	node->id = id;
	INIT_LIST_HEAD(&node->ready);
	mtk_dip_node_defaults(node);

	q->type = desc->buf_type;
	q->io_modes = VB2_MMAP | VB2_DMABUF;
	q->drv_priv = node;
	q->ops = &mtk_dip_vb2_ops;
	if (id == MTK_DIP_NODE_PARAMS) {
		q->mem_ops = &vb2_vmalloc_memops;
		q->buf_struct_size = sizeof(struct mtk_dip_buffer);
	} else {
		q->mem_ops = &vb2_dma_contig_memops;
		q->buf_struct_size = offsetof(struct mtk_dip_buffer, params);
	}
	/*
	 * A buffer that does not fit into the memory region fails with
	 * -ENOMEM; the page allocator need not warn about the order.
	 */
	q->gfp_flags = __GFP_NOWARN;
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	q->dev = dip->dev;
	q->lock = &node->lock;
	ret = vb2_queue_init(q);
	if (ret)
		return ret;

	strscpy(vdev->name, desc->name, sizeof(vdev->name));
	vdev->v4l2_dev = &dip->v4l2_dev;
	vdev->fops = &mtk_dip_fops;
	vdev->ioctl_ops = mtk_dip_ioctl_ops[id];
	vdev->release = video_device_release_empty;
	vdev->vfl_dir = desc->output ? VFL_DIR_TX : VFL_DIR_RX;
	vdev->device_caps = desc->caps | V4L2_CAP_STREAMING;
	vdev->lock = &node->lock;
	vdev->queue = q;
	video_set_drvdata(vdev, node);

	node->pad.flags = desc->output ? MEDIA_PAD_FL_SOURCE :
			  MEDIA_PAD_FL_SINK;
	ret = media_entity_pads_init(&vdev->entity, 1, &node->pad);
	if (ret)
		return ret;

	ret = video_register_device(vdev, VFL_TYPE_VIDEO, -1);
	if (ret)
		goto err_entity;

	/*
	 * raw and the outputs take part when they stream, params can be
	 * switched off
	 */
	flags = MEDIA_LNK_FL_ENABLED;
	if (id != MTK_DIP_NODE_PARAMS)
		flags |= MEDIA_LNK_FL_IMMUTABLE;
	if (desc->output)
		ret = media_create_pad_link(&vdev->entity, 0, &dip->sd.entity,
					    id, flags);
	else
		ret = media_create_pad_link(&dip->sd.entity, id, &vdev->entity,
					    0, flags);
	if (ret)
		goto err_video;

	return 0;

err_video:
	vb2_video_unregister_device(vdev);
err_entity:
	media_entity_cleanup(&vdev->entity);
	return ret;
}

/*
 * The node answers -ENODEV from now on, also to files that stay open. A
 * stream stops here: the buffers of the driver go back with an error, and
 * the queue lets its buffers go.
 */
static void mtk_dip_node_cleanup(struct mtk_dip_node *node)
{
	vb2_video_unregister_device(&node->vdev);
	media_entity_cleanup(&node->vdev.entity);
}

static const struct v4l2_subdev_ops mtk_dip_sd_ops = {};

/* Only the link of the params node can change */
static int mtk_dip_link_setup(struct media_entity *entity,
			      const struct media_pad *local,
			      const struct media_pad *remote, u32 flags)
{
	struct mtk_dip *dip = container_of(entity, struct mtk_dip, sd.entity);
	unsigned long irqflags;

	if (local->index != MTK_DIP_NODE_PARAMS)
		return -EINVAL;

	spin_lock_irqsave(&dip->lock, irqflags);
	dip->params_link = flags & MEDIA_LNK_FL_ENABLED;
	spin_unlock_irqrestore(&dip->lock, irqflags);

	return 0;
}

static const struct media_entity_operations mtk_dip_entity_ops = {
	.link_setup = mtk_dip_link_setup,
};

/* The last open file of a node is gone, after the unbind */
static void mtk_dip_release(struct v4l2_device *v4l2_dev)
{
	struct mtk_dip *dip = container_of(v4l2_dev, struct mtk_dip, v4l2_dev);
	unsigned int i;

	for (i = 0; i < MTK_DIP_NODE_NUM; i++)
		mutex_destroy(&dip->node[i].lock);
	media_device_cleanup(&dip->mdev);
	kfree(dip);
}

/* The unbind drops the reference of the driver */
static void mtk_dip_put(void *data)
{
	struct mtk_dip *dip = data;

	v4l2_device_unregister(&dip->v4l2_dev);
	v4l2_device_put(&dip->v4l2_dev);
}

/**
 * mtk_dip_alloc - device state, with the V4L2 and the media device
 * @dev: the platform device
 *
 * Open files of the video nodes keep the nodes, their queues and locks, and
 * the media device after the driver is unbound. The state goes with the
 * last of them, in the release of the V4L2 device; the reference of the
 * driver goes with the unbind or a failed probe.
 *
 * Return: the state or an error pointer.
 */
struct mtk_dip *mtk_dip_alloc(struct device *dev)
{
	struct media_device *mdev;
	struct mtk_dip *dip;
	unsigned int i;
	int ret;

	dip = kzalloc_obj(*dip);
	if (!dip)
		return ERR_PTR(-ENOMEM);
	dip->dev = dev;
	for (i = 0; i < MTK_DIP_NODE_NUM; i++)
		mutex_init(&dip->node[i].lock);

	mdev = &dip->mdev;
	mdev->dev = dev;
	strscpy(mdev->model, MTK_DIP_NAME, sizeof(mdev->model));
	snprintf(mdev->bus_info, sizeof(mdev->bus_info), "platform:%s",
		 dev_name(dev));
	media_device_init(mdev);

	dip->v4l2_dev.mdev = mdev;
	dip->v4l2_dev.release = mtk_dip_release;
	ret = v4l2_device_register(dev, &dip->v4l2_dev);
	if (ret) {
		mtk_dip_release(&dip->v4l2_dev);
		return ERR_PTR(ret);
	}

	ret = devm_add_action_or_reset(dev, mtk_dip_put, dip);
	if (ret)
		return ERR_PTR(ret);

	return dip;
}

/**
 * mtk_dip_v4l2_register - media device, processing entity and video nodes
 * @dip: the device
 *
 * Return: 0 or a negative error code.
 */
int mtk_dip_v4l2_register(struct mtk_dip *dip)
{
	unsigned int i;
	int ret;

	v4l2_subdev_init(&dip->sd, &mtk_dip_sd_ops);
	dip->sd.owner = THIS_MODULE;
	dip->sd.entity.function = MEDIA_ENT_F_PROC_VIDEO_ISP;
	dip->sd.entity.ops = &mtk_dip_entity_ops;
	dip->params_link = true;
	strscpy(dip->sd.name, "mtk-dip", sizeof(dip->sd.name));
	for (i = 0; i < MTK_DIP_NODE_NUM; i++)
		dip->sd_pads[i].flags = mtk_dip_node_desc[i].output ?
					MEDIA_PAD_FL_SINK : MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&dip->sd.entity, MTK_DIP_NODE_NUM,
				     dip->sd_pads);
	if (ret)
		return ret;
	ret = v4l2_device_register_subdev(&dip->v4l2_dev, &dip->sd);
	if (ret)
		goto err_sd_entity;

	for (i = 0; i < MTK_DIP_NODE_NUM; i++) {
		ret = mtk_dip_node_init(dip, i);
		if (ret)
			goto err_nodes;
	}

	ret = media_device_register(&dip->mdev);
	if (ret)
		goto err_nodes;

	return 0;

err_nodes:
	while (i--)
		mtk_dip_node_cleanup(&dip->node[i]);
	v4l2_device_unregister_subdev(&dip->sd);
err_sd_entity:
	media_entity_cleanup(&dip->sd.entity);
	return ret;
}

/**
 * mtk_dip_v4l2_unregister - undo mtk_dip_v4l2_register()
 * @dip: the device
 *
 * Stops the streams; no job runs or starts after this.
 */
void mtk_dip_v4l2_unregister(struct mtk_dip *dip)
{
	unsigned int i;

	media_device_unregister(&dip->mdev);
	for (i = 0; i < MTK_DIP_NODE_NUM; i++)
		mtk_dip_node_cleanup(&dip->node[i]);
	v4l2_device_unregister_subdev(&dip->sd);
	media_entity_cleanup(&dip->sd.entity);
}
