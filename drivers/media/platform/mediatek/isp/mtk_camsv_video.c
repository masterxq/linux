// SPDX-License-Identifier: GPL-2.0
/*
 * mtk_camsv_video.c - V4L2 video node support
 *
 * Copyright (c) 2020 BayLibre
 * Copyright (c) 2022 MediaTek Inc.
 */

#include <linux/slab.h>
#include <linux/version.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ioctl.h>
#include <media/v4l2-mediabus.h>

#include "mtk_camsv.h"
#include "mtk_camsv_hw.h"
#include "mtk_camsv_reg.h"

static inline struct mtk_cam_video_device *
file_to_mtk_cam_video_device(struct file *__file)
{
	return container_of(video_devdata(__file),
			    struct mtk_cam_video_device, vdev);
}

static inline struct mtk_cam_video_device *
vb2_queue_to_mtk_cam_video_device(struct vb2_queue *vq)
{
	return container_of(vq, struct mtk_cam_video_device, vbq);
}

static inline struct mtk_cam_dev_buffer *
to_mtk_cam_dev_buffer(struct vb2_buffer *buf)
{
	return container_of(buf, struct mtk_cam_dev_buffer, v4l2_buf.vb2_buf);
}

/* -----------------------------------------------------------------------------
 * Format Information
 */

/*
 * Bus code from the source and pixel format of the video node. A 10-bit
 * Bayer source can be written as 8 bits (the packer keeps the 8 MSBs) or
 * as MediaTek packed 10 bits; the pixel format of the video node selects
 * the packer mode, the bus code the format of the TG. The first entry of a
 * pixel format gives its bits per pixel.
 */
#define MTK_CAM_FMT_BAYER(_code, _fourcc, _bpp, _pak, _tg)		\
	{								\
		.code = _code,						\
		.fourcc = _fourcc,					\
		.bpp = _bpp,						\
		.w_factor = 1,						\
		.module_en_pak = CAMSV_MODULE_EN_PAK_EN,		\
		.pak = _pak,						\
		.fmt_sel = _tg,						\
		.imgo_stride = CAMSV_IMGO_SV_STRIDE_BUS_SIZE_EN		\
			| CAMSV_IMGO_SV_STRIDE_BUS_SIZE_32,		\
	}

#define MTK_CAM_FMT_YUV(_code, _fourcc)					\
	{								\
		.code = _code,						\
		.fourcc = _fourcc,					\
		.bpp = 16,						\
		.w_factor = 2,						\
		.module_en_pak = CAMSV_MODULE_EN_PAK_SEL,		\
		.pak = 0, /* ignored */					\
		.fmt_sel = CAMSV_FMT_SEL_IMGO_BUS_SIZE_16		\
			| CAMSV_FMT_SEL_TG1_FMT_YUV422,			\
		.imgo_stride = CAMSV_IMGO_SV_STRIDE_BUS_SIZE_EN		\
			| CAMSV_IMGO_SV_STRIDE_BUS_SIZE_16,		\
	}

#define MTK_CAM_PAK_8	(CAMSV_PAK_PAK_MODE_RAW_8 | CAMSV_PAK_PAK_DBL_MODE_2_PIXELS)
#define MTK_CAM_PAK_12	(CAMSV_PAK_PAK_MODE_RAW_12 | CAMSV_PAK_PAK_DBL_MODE_2_PIXELS)

static const struct mtk_cam_format_info mtk_cam_format_info[] = {
	/* 8-bit sources */
	MTK_CAM_FMT_BAYER(MEDIA_BUS_FMT_SBGGR8_1X8, V4L2_PIX_FMT_SBGGR8, 8,
			  MTK_CAM_PAK_8, CAMSV_FMT_SEL_TG1_FMT_RAW_8),
	MTK_CAM_FMT_BAYER(MEDIA_BUS_FMT_SGBRG8_1X8, V4L2_PIX_FMT_SGBRG8, 8,
			  MTK_CAM_PAK_8, CAMSV_FMT_SEL_TG1_FMT_RAW_8),
	MTK_CAM_FMT_BAYER(MEDIA_BUS_FMT_SGRBG8_1X8, V4L2_PIX_FMT_SGRBG8, 8,
			  MTK_CAM_PAK_8, CAMSV_FMT_SEL_TG1_FMT_RAW_8),
	MTK_CAM_FMT_BAYER(MEDIA_BUS_FMT_SRGGB8_1X8, V4L2_PIX_FMT_SRGGB8, 8,
			  MTK_CAM_PAK_8, CAMSV_FMT_SEL_TG1_FMT_RAW_8),
	/* 10-bit sources, the 8 MSBs */
	MTK_CAM_FMT_BAYER(MEDIA_BUS_FMT_SBGGR10_1X10, V4L2_PIX_FMT_SBGGR8, 8,
			  CAMSV_PAK_PAK_MODE_RAW_8, CAMSV_FMT_SEL_TG1_FMT_RAW_10),
	MTK_CAM_FMT_BAYER(MEDIA_BUS_FMT_SGBRG10_1X10, V4L2_PIX_FMT_SGBRG8, 8,
			  CAMSV_PAK_PAK_MODE_RAW_8, CAMSV_FMT_SEL_TG1_FMT_RAW_10),
	MTK_CAM_FMT_BAYER(MEDIA_BUS_FMT_SGRBG10_1X10, V4L2_PIX_FMT_SGRBG8, 8,
			  CAMSV_PAK_PAK_MODE_RAW_8, CAMSV_FMT_SEL_TG1_FMT_RAW_10),
	MTK_CAM_FMT_BAYER(MEDIA_BUS_FMT_SRGGB10_1X10, V4L2_PIX_FMT_SRGGB8, 8,
			  CAMSV_PAK_PAK_MODE_RAW_8, CAMSV_FMT_SEL_TG1_FMT_RAW_10),
	/* 10-bit sources, MediaTek packed */
	MTK_CAM_FMT_BAYER(MEDIA_BUS_FMT_SBGGR10_1X10, V4L2_PIX_FMT_MTISP_SBGGR10,
			  10, CAMSV_PAK_PAK_MODE_RAW_10,
			  CAMSV_FMT_SEL_TG1_FMT_RAW_10),
	MTK_CAM_FMT_BAYER(MEDIA_BUS_FMT_SGBRG10_1X10, V4L2_PIX_FMT_MTISP_SGBRG10,
			  10, CAMSV_PAK_PAK_MODE_RAW_10,
			  CAMSV_FMT_SEL_TG1_FMT_RAW_10),
	MTK_CAM_FMT_BAYER(MEDIA_BUS_FMT_SGRBG10_1X10, V4L2_PIX_FMT_MTISP_SGRBG10,
			  10, CAMSV_PAK_PAK_MODE_RAW_10,
			  CAMSV_FMT_SEL_TG1_FMT_RAW_10),
	MTK_CAM_FMT_BAYER(MEDIA_BUS_FMT_SRGGB10_1X10, V4L2_PIX_FMT_MTISP_SRGGB10,
			  10, CAMSV_PAK_PAK_MODE_RAW_10,
			  CAMSV_FMT_SEL_TG1_FMT_RAW_10),
	/* 10-bit greyscale (phase detection pixels), MediaTek packed */
	MTK_CAM_FMT_BAYER(MEDIA_BUS_FMT_Y10_1X10, V4L2_PIX_FMT_MTISP_Y10,
			  10, CAMSV_PAK_PAK_MODE_RAW_10,
			  CAMSV_FMT_SEL_TG1_FMT_RAW_10),
	/* 12-bit sources */
	MTK_CAM_FMT_BAYER(MEDIA_BUS_FMT_SBGGR12_1X12, V4L2_PIX_FMT_SBGGR12, 12,
			  MTK_CAM_PAK_12, CAMSV_FMT_SEL_TG1_FMT_RAW_12),
	MTK_CAM_FMT_BAYER(MEDIA_BUS_FMT_SGBRG12_1X12, V4L2_PIX_FMT_SGBRG12, 12,
			  MTK_CAM_PAK_12, CAMSV_FMT_SEL_TG1_FMT_RAW_12),
	MTK_CAM_FMT_BAYER(MEDIA_BUS_FMT_SGRBG12_1X12, V4L2_PIX_FMT_SGRBG12, 12,
			  MTK_CAM_PAK_12, CAMSV_FMT_SEL_TG1_FMT_RAW_12),
	MTK_CAM_FMT_BAYER(MEDIA_BUS_FMT_SRGGB12_1X12, V4L2_PIX_FMT_SRGGB12, 12,
			  MTK_CAM_PAK_12, CAMSV_FMT_SEL_TG1_FMT_RAW_12),
	/* YUV sources */
	MTK_CAM_FMT_YUV(MEDIA_BUS_FMT_UYVY8_1X16, V4L2_PIX_FMT_UYVY),
	MTK_CAM_FMT_YUV(MEDIA_BUS_FMT_VYUY8_1X16, V4L2_PIX_FMT_VYUY),
	MTK_CAM_FMT_YUV(MEDIA_BUS_FMT_YUYV8_1X16, V4L2_PIX_FMT_YUYV),
	MTK_CAM_FMT_YUV(MEDIA_BUS_FMT_YVYU8_1X16, V4L2_PIX_FMT_YVYU),
};

/* Pixel formats of the video node, each once, 8-bit Bayer first */
static const u32 mtk_cam_stream_fmts[] = {
	V4L2_PIX_FMT_SBGGR8, V4L2_PIX_FMT_SGBRG8,
	V4L2_PIX_FMT_SGRBG8, V4L2_PIX_FMT_SRGGB8,
	V4L2_PIX_FMT_MTISP_SBGGR10, V4L2_PIX_FMT_MTISP_SGBRG10,
	V4L2_PIX_FMT_MTISP_SGRBG10, V4L2_PIX_FMT_MTISP_SRGGB10,
	V4L2_PIX_FMT_MTISP_Y10,
	V4L2_PIX_FMT_SBGGR12, V4L2_PIX_FMT_SGBRG12,
	V4L2_PIX_FMT_SGRBG12, V4L2_PIX_FMT_SRGGB12,
	V4L2_PIX_FMT_UYVY, V4L2_PIX_FMT_VYUY,
	V4L2_PIX_FMT_YUYV, V4L2_PIX_FMT_YVYU,
};

static const struct mtk_cam_format_info *
mtk_cam_format_info_by_fourcc(u32 fourcc)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(mtk_cam_format_info); ++i) {
		const struct mtk_cam_format_info *info =
			&mtk_cam_format_info[i];

		if (info->fourcc == fourcc)
			return info;
	}

	return NULL;
}

/**
 * mtk_cam_format_info_find - the entry of a bus code and a pixel format
 * @code: media bus code of the source
 * @fourcc: pixel format of the video node
 *
 * Return: the entry, or NULL if the CAMSV cannot write @code as @fourcc.
 */
const struct mtk_cam_format_info *
mtk_cam_format_info_find(u32 code, u32 fourcc)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(mtk_cam_format_info); ++i)
		if (mtk_cam_format_info[i].code == code &&
		    mtk_cam_format_info[i].fourcc == fourcc)
			return &mtk_cam_format_info[i];

	return NULL;
}

/* the first entry of each bus code counts for the enumeration */
static bool mtk_cam_format_info_first_of_code(unsigned int idx)
{
	unsigned int i;

	for (i = 0; i < idx; ++i)
		if (mtk_cam_format_info[i].code == mtk_cam_format_info[idx].code)
			return false;

	return true;
}

static bool mtk_cam_dev_find_fmt(const struct mtk_cam_vdev_desc *desc,
				 u32 format)
{
	unsigned int i;

	for (i = 0; i < desc->num_fmts; i++) {
		if (desc->fmts[i] == format)
			return true;
	}

	return false;
}

static void calc_bpl_size_pix_mp(const struct mtk_cam_format_info *fmtinfo,
				 struct v4l2_pix_format_mplane *pix_mp)
{
	unsigned int bpl;
	unsigned int i;

	bpl = ALIGN(DIV_ROUND_UP(pix_mp->width * fmtinfo->bpp, 8), 2);

	for (i = 0; i < pix_mp->num_planes; ++i) {
		pix_mp->plane_fmt[i].bytesperline = bpl;
		pix_mp->plane_fmt[i].sizeimage = bpl * pix_mp->height;
	}
}

static void mtk_cam_dev_load_default_fmt(struct mtk_cam_dev *cam,
					 struct mtk_cam_video_device *vdev)
{
	struct v4l2_pix_format_mplane *fmt = &vdev->format;

	fmt->num_planes = MAX_SUPPORTED_NUM_PLANES;
	fmt->pixelformat = mtk_cam_dev_find_fmt(vdev->desc, cam->default_fmt) ?
			   cam->default_fmt : vdev->desc->fmts[0];
	fmt->width = IMG_DEF_WIDTH;
	fmt->height = IMG_DEF_HEIGHT;

	fmt->colorspace = V4L2_COLORSPACE_SRGB;
	fmt->field = V4L2_FIELD_NONE;
	fmt->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	fmt->quantization = V4L2_QUANTIZATION_DEFAULT;
	fmt->xfer_func = V4L2_XFER_FUNC_DEFAULT;

	vdev->fmtinfo = mtk_cam_format_info_by_fourcc(fmt->pixelformat);

	calc_bpl_size_pix_mp(vdev->fmtinfo, fmt);
}

/* -----------------------------------------------------------------------------
 * VB2 Queue Operations
 */

static int mtk_cam_vb2_queue_setup(struct vb2_queue *vq,
				   unsigned int *num_buffers,
				   unsigned int *num_planes,
				   unsigned int sizes[],
				   struct device *alloc_devs[])
{
	struct mtk_cam_video_device *vdev =
		vb2_queue_to_mtk_cam_video_device(vq);
	struct mtk_cam_dev *cam = vb2_get_drv_priv(vq);
	const struct v4l2_pix_format_mplane *fmt = &vdev->format;
	unsigned int size;

	size = fmt->plane_fmt[0].sizeimage;

	/*
	 * VIDIOC_CREATE_BUFS: at least the current format, at most twice the
	 * largest frame the node can capture.
	 */
	if (*num_planes == 0) {
		*num_planes = MAX_SUPPORTED_NUM_PLANES;
		sizes[0] = size;
	} else if (*num_planes != MAX_SUPPORTED_NUM_PLANES || sizes[0] < size ||
		   sizes[0] > 2 * IMG_MAX_SIZEIMAGE) {
		return -EINVAL;
	}

	mtk_camsv_fbc_init(cam, *num_buffers);

	return 0;
}

static int mtk_cam_vb2_buf_prepare(struct vb2_buffer *vb)
{
	struct mtk_cam_video_device *vdev =
		vb2_queue_to_mtk_cam_video_device(vb->vb2_queue);
	struct mtk_cam_dev *cam = vb2_get_drv_priv(vb->vb2_queue);
	struct mtk_cam_dev_buffer *buf = to_mtk_cam_dev_buffer(vb);
	const struct v4l2_pix_format_mplane *fmt = &vdev->format;
	dma_addr_t daddr;
	u32 size;
	unsigned int i;

	for (i = 0; i < vb->num_planes; i++) {
		size = fmt->plane_fmt[i].sizeimage;
		if (vb2_plane_size(vb, i) < size) {
			dev_dbg(cam->dev, "plane size is too small:%lu<%u\n",
				vb2_plane_size(vb, i), size);
			return -EINVAL;
		}
	}

	buf->v4l2_buf.field = V4L2_FIELD_NONE;

	for (i = 0; i < vb->num_planes; i++) {
		size = fmt->plane_fmt[i].sizeimage;
		vb2_set_plane_payload(vb, i, size);
	}

	/*
	 * Every time: an imported DMABUF can be another one at each QBUF.
	 * The address register of the DMA has 32 bits.
	 */
	daddr = vb2_dma_contig_plane_dma_addr(vb, 0);
	if (upper_32_bits(daddr + vb2_plane_size(vb, 0) - 1)) {
		dev_dbg(cam->dev, "buffer above 4 GiB\n");
		return -EINVAL;
	}
	buf->daddr = daddr;

	return 0;
}

static void mtk_cam_vb2_buf_queue(struct vb2_buffer *vb)
{
	struct mtk_cam_dev *cam = vb2_get_drv_priv(vb->vb2_queue);
	struct mtk_cam_dev_buffer *buf = to_mtk_cam_dev_buffer(vb);
	unsigned long flags;

	/* Add the buffer into the tracking list */
	spin_lock_irqsave(&cam->buf_list_lock, flags);
	if (!cam->conf->has_fbc) {
		/* picked up by the ISR at the next frame end */
		list_add_tail(&buf->list, &cam->buf_list);
		spin_unlock_irqrestore(&cam->buf_list_lock, flags);
		return;
	}
	if (vb2_start_streaming_called(vb->vb2_queue) && list_empty(&cam->buf_list))
		mtk_camsv_update_buffers_add(cam, buf);

	list_add_tail(&buf->list, &cam->buf_list);
	spin_unlock_irqrestore(&cam->buf_list_lock, flags);
	if (vb2_start_streaming_called(vb->vb2_queue))
		mtk_camsv_fbc_inc(cam);
}

void mtk_cam_vb2_return_all_buffers(struct mtk_cam_dev *cam,
				    enum vb2_buffer_state state)
{
	struct mtk_cam_dev_buffer *buf, *buf_prev;
	unsigned long flags;

	spin_lock_irqsave(&cam->buf_list_lock, flags);
	if (cam->cur) {
		cam->cur->daddr = 0ULL;
		vb2_buffer_done(&cam->cur->v4l2_buf.vb2_buf, state);
		cam->cur = NULL;
	}
	list_for_each_entry_safe(buf, buf_prev, &cam->buf_list, list) {
		buf->daddr = 0ULL;
		list_del(&buf->list);
		vb2_buffer_done(&buf->v4l2_buf.vb2_buf, state);
	}
	spin_unlock_irqrestore(&cam->buf_list_lock, flags);
}

static int mtk_cam_vb2_start_streaming(struct vb2_queue *vq,
				       unsigned int count)
{
	struct mtk_cam_dev *cam = vb2_get_drv_priv(vq);
	struct mtk_cam_video_device *vdev =
		vb2_queue_to_mtk_cam_video_device(vq);
	struct device *dev = cam->dev;
	const struct v4l2_pix_format_mplane *fmt = &vdev->format;
	const struct mtk_cam_format_info *fmtinfo;
	const struct v4l2_mbus_framefmt *src;
	struct v4l2_subdev_state *state;
	u32 code, width, height;
	int ret;

	/* Configure the hardware from the bus code actually received. */
	state = v4l2_subdev_lock_and_get_active_state(&cam->subdev);
	code = v4l2_subdev_state_get_format(state, MTK_CAM_CIO_PAD_SENINF)->code;
	src = v4l2_subdev_state_get_format(state, MTK_CAM_CIO_PAD_VIDEO);
	width = src->width;
	height = src->height;
	v4l2_subdev_unlock_state(state);

	/*
	 * Validate the pipeline before the hardware is touched: a grab window
	 * that does not match the frames coming in only produces TG and IMGO
	 * error interrupts. The pixel format has to be one the CAMSV can
	 * write the bus code as; it selects the packer mode.
	 */
	if (fmt->width != width || fmt->height != height) {
		dev_dbg(dev, "video format %ux%u does not match the source pad %ux%u\n",
			fmt->width, fmt->height, width, height);
		ret = -EPIPE;
		goto fail_return;
	}
	fmtinfo = mtk_cam_format_info_find(code, fmt->pixelformat);
	if (!fmtinfo) {
		dev_dbg(dev, "bus code 0x%04x cannot be written as %p4cc\n",
			code, &fmt->pixelformat);
		ret = -EPIPE;
		goto fail_return;
	}

	/*
	 * CAMSVs fed by streams of the same seninf input (image and phase
	 * detection data of one sensor) share the sensor and the seninf sink,
	 * so they have to share the pipeline: the first start populates it
	 * with every pad reached through active routes, which includes the
	 * video pads of the other CAMSVs, and a later start joins it.
	 */
	ret = video_device_pipeline_alloc_start(&vdev->vdev);
	if (ret)
		goto fail_return;

	ret = mtk_camsv_pixel_mode(cam);
	if (ret)
		goto fail_pipeline;

	mtk_camsv_scratch_alloc(cam, fmt->plane_fmt[0].sizeimage);

	ret = pm_runtime_resume_and_get(dev);
	if (ret) {
		dev_err(dev, "failed to get pm_runtime\n");
		goto fail_scratch;
	}

	/* kept for the restart after a system sleep */
	cam->stream_fmtinfo = fmtinfo;
	cam->stream_width = fmt->width;
	cam->stream_height = fmt->height;
	cam->stream_bpl = fmt->plane_fmt[0].bytesperline;

	mtk_camsv_setup(cam, count, cam->stream_width, cam->stream_height,
			cam->stream_bpl, fmtinfo);

	/* Enable CMOS and VF */
	mtk_camsv_cmos_vf_hw_enable(cam);

	cam->sequence = (unsigned int)-1;

	/* Stream on the sub-device */
	ret = v4l2_subdev_enable_streams(&cam->subdev,
					 cam->subdev_pads[MTK_CAM_CIO_PAD_VIDEO].index,
					 BIT(0));
	if (ret)
		goto fail_hw;

	cam->streaming = true;

	return 0;

fail_hw:
	mtk_camsv_cmos_vf_hw_disable(cam);
	pm_runtime_mark_last_busy(cam->dev);
	pm_runtime_put_autosuspend(cam->dev);
fail_scratch:
	mtk_camsv_scratch_free(cam);
fail_pipeline:
	video_device_pipeline_stop(&vdev->vdev);
fail_return:
	mtk_cam_vb2_return_all_buffers(cam, VB2_BUF_STATE_QUEUED);

	return ret;
}

static void mtk_cam_vb2_stop_streaming(struct vb2_queue *vq)
{
	struct mtk_cam_dev *cam = vb2_get_drv_priv(vq);
	struct mtk_cam_video_device *vdev =
		vb2_queue_to_mtk_cam_video_device(vq);

	cam->streaming = false;

	/* Disable CMOS and VF */
	mtk_camsv_cmos_vf_hw_disable(cam);

	/*
	 * The queue can be released from the unbind path; once the subdev
	 * has left the media graph there is no pipeline left to stop.
	 */
	if (cam->subdev.entity.graph_obj.mdev)
		v4l2_subdev_disable_streams(&cam->subdev,
					    cam->subdev_pads[MTK_CAM_CIO_PAD_VIDEO].index,
					    BIT(0));

	mtk_cam_vb2_return_all_buffers(cam, VB2_BUF_STATE_ERROR);

	video_device_pipeline_stop(&vdev->vdev);

	pm_runtime_mark_last_busy(cam->dev);
	pm_runtime_put_autosuspend(cam->dev);
	mtk_camsv_scratch_free(cam);
}

static const struct vb2_ops mtk_cam_vb2_ops = {
	.queue_setup = mtk_cam_vb2_queue_setup,
	.buf_prepare = mtk_cam_vb2_buf_prepare,
	.buf_queue = mtk_cam_vb2_buf_queue,
	.start_streaming = mtk_cam_vb2_start_streaming,
	.stop_streaming = mtk_cam_vb2_stop_streaming,
};

/* -----------------------------------------------------------------------------
 * V4L2 Subdev Operations
 */

static inline struct mtk_cam_dev *to_mtk_cam_dev(struct v4l2_subdev *sd)
{
	return container_of(sd, struct mtk_cam_dev, subdev);
}


static int mtk_cam_init_state(struct v4l2_subdev *sd,
			      struct v4l2_subdev_state *sd_state)
{
	static const struct v4l2_mbus_framefmt def_format = {
		.code = MEDIA_BUS_FMT_SGRBG10_1X10,
		.width = IMG_DEF_WIDTH,
		.height = IMG_DEF_HEIGHT,
		.field = V4L2_FIELD_NONE,
		.colorspace = V4L2_COLORSPACE_SRGB,
		.xfer_func = V4L2_XFER_FUNC_DEFAULT,
		.ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT,
		.quantization = V4L2_QUANTIZATION_DEFAULT,
	};
	struct v4l2_mbus_framefmt *format;
	unsigned int i;

	for (i = 0; i < sd->entity.num_pads; i++) {
		format = v4l2_subdev_state_get_format(sd_state, i);
		*format = def_format;
	}

	return 0;
}

static int mtk_cam_enum_mbus_code(struct v4l2_subdev *sd,
				  struct v4l2_subdev_state *sd_state,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	unsigned int i, n = 0;

	for (i = 0; i < ARRAY_SIZE(mtk_cam_format_info); ++i) {
		if (!mtk_cam_format_info_first_of_code(i))
			continue;
		if (n++ == code->index) {
			code->code = mtk_cam_format_info[i].code;
			return 0;
		}
	}

	return -EINVAL;
}

static int mtk_cam_set_fmt(struct v4l2_subdev *sd,
			   struct v4l2_subdev_state *sd_state,
			   struct v4l2_subdev_format *fmt)
{
	struct v4l2_mbus_framefmt *format;
	unsigned int i;

	/*
	 * We only support pass-through mode, the format on source pads can't
	 * be modified: report the propagated one.
	 */
	if (fmt->pad != MTK_CAM_CIO_PAD_SENINF)
		return v4l2_subdev_get_fmt(sd, sd_state, fmt);

	/* start_streaming() configured the hardware from the active format */
	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE && v4l2_subdev_is_streaming(sd))
		return -EBUSY;

	for (i = 0; i < ARRAY_SIZE(mtk_cam_format_info); ++i) {
		if (mtk_cam_format_info[i].code == fmt->format.code)
			break;
	}

	if (i == ARRAY_SIZE(mtk_cam_format_info))
		fmt->format.code = mtk_cam_format_info[0].code;

	format = v4l2_subdev_state_get_format(sd_state, fmt->pad);
	format->width = clamp_t(u32, fmt->format.width,
				IMG_MIN_WIDTH, IMG_MAX_WIDTH);
	format->height = clamp_t(u32, fmt->format.height,
				 IMG_MIN_HEIGHT, IMG_MAX_HEIGHT);
	format->code = fmt->format.code;

	fmt->format = *format;

	/* Propagate the format to the source pad. */
	format = v4l2_subdev_state_get_format(sd_state, MTK_CAM_CIO_PAD_VIDEO);
	format->width = fmt->format.width;
	format->height = fmt->format.height;
	format->code = fmt->format.code;

	return 0;
}

/*
 * The seninf (which owns the v4l2_device) has bound us. The sub-device and
 * its device node hold the CAMSV until mtk_cam_subdev_release().
 */
static int mtk_cam_subdev_registered(struct v4l2_subdev *sd)
{
	struct mtk_cam_dev *cam = to_mtk_cam_dev(sd);
	int ret;

	/* Create the video device and link. */
	ret = mtk_cam_video_register(cam);
	if (ret)
		return ret;

	mtk_cam_dev_get(cam);
	return 0;
}

/*
 * Called when the seninf unbinds us or the CAMSV goes (always under the
 * list lock of v4l2-async): drop the video node too, a later bind
 * registers a new one.
 */
static void mtk_cam_subdev_unregistered(struct v4l2_subdev *sd)
{
	struct mtk_cam_dev *cam = to_mtk_cam_dev(sd);

	mtk_cam_video_unregister(cam);
	cam->seninf = NULL;
}

/*
 * The device node of the sub-device is gone, after its last open file. The
 * V4L2 core still writes to the sub-device after this callback, so the
 * reference may not go here: it goes from the workqueue of the driver.
 */
static void mtk_cam_subdev_release(struct v4l2_subdev *sd)
{
	mtk_cam_dev_put_deferred(to_mtk_cam_dev(sd));
}

static int camsv_enable_streams(struct v4l2_subdev *sd,
				struct v4l2_subdev_state *state, u32 pad,
				u64 streams_mask)
{
	struct mtk_cam_dev *cam = to_mtk_cam_dev(sd);
	struct v4l2_subdev *seninf;
	int ret;

	/*
	 * Look the seninf up on every start: it can be unbound and bound
	 * again (module reload), and a cached pad would then be stale.
	 */
	cam->seninf = media_pad_remote_pad_first(&cam->subdev_pads[MTK_CAM_CIO_PAD_SENINF]);
	if (!cam->seninf) {
		dev_err(cam->dev, "No SENINF connected\n");
		return -ENOLINK;
	}

	seninf = media_entity_to_v4l2_subdev(cam->seninf->entity);

	/* Seninf must stream on first */
	ret = v4l2_subdev_enable_streams(seninf, cam->seninf->index, BIT(0));
	if (ret) {
		dev_err(cam->dev, "failed to stream on %s:%d\n",
			seninf->entity.name, ret);
		return ret;
	}

	return 0;
}

static int camsv_disable_streams(struct v4l2_subdev *sd,
				 struct v4l2_subdev_state *state, u32 pad,
				 u64 streams_mask)
{
	struct mtk_cam_dev *cam = to_mtk_cam_dev(sd);
	struct v4l2_subdev *seninf;
	int ret;

	if (cam->seninf && cam->seninf->entity->graph_obj.mdev) {
		seninf = media_entity_to_v4l2_subdev(cam->seninf->entity);
		ret = v4l2_subdev_disable_streams(seninf, cam->seninf->index,
						  BIT(0));
		if (ret)
			dev_err(cam->dev, "failed to stream off %s:%u: %d\n",
				seninf->entity.name, cam->seninf->index, ret);
	}

	/*
	 * The CAMSV is stopped (stop_streaming() switched it off before): an
	 * error here would leave it enabled for the V4L2 core, and its next
	 * start would fail with -EALREADY.
	 */
	return 0;
}

static const struct v4l2_subdev_pad_ops mtk_cam_subdev_pad_ops = {
	.enum_mbus_code = mtk_cam_enum_mbus_code,
	.set_fmt = mtk_cam_set_fmt,
	.get_fmt = v4l2_subdev_get_fmt,
	.link_validate = v4l2_subdev_link_validate_default,
	.enable_streams = camsv_enable_streams,
	.disable_streams = camsv_disable_streams,
};

static const struct v4l2_subdev_ops mtk_cam_subdev_ops = {
	.pad = &mtk_cam_subdev_pad_ops,
};

static const struct v4l2_subdev_internal_ops mtk_cam_internal_ops = {
	.init_state = mtk_cam_init_state,
	.registered = mtk_cam_subdev_registered,
	.unregistered = mtk_cam_subdev_unregistered,
	.release = mtk_cam_subdev_release,
};

/* -----------------------------------------------------------------------------
 * Media Entity Operations
 */

static const struct media_entity_operations mtk_cam_media_entity_ops = {
	.link_validate = v4l2_subdev_link_validate,
	.get_fwnode_pad = v4l2_subdev_get_fwnode_pad_1_to_1,
};

/* -----------------------------------------------------------------------------
 * Init & Cleanup
 */

static int mtk_cam_v4l2_register(struct mtk_cam_dev *cam)
{
	struct device *dev = cam->dev;
	int ret;

	cam->subdev_pads[MTK_CAM_CIO_PAD_SENINF].flags = MEDIA_PAD_FL_SINK;
	cam->subdev_pads[MTK_CAM_CIO_PAD_VIDEO].flags = MEDIA_PAD_FL_SOURCE;

	/* Initialize subdev pads */
	ret = media_entity_pads_init(&cam->subdev.entity,
				     ARRAY_SIZE(cam->subdev_pads),
				     cam->subdev_pads);
	if (ret) {
		dev_err(dev, "failed to initialize media pads:%d\n", ret);
		return ret;
	}

	/* Initialize subdev */
	v4l2_subdev_init(&cam->subdev, &mtk_cam_subdev_ops);

	cam->subdev.dev = dev;
	cam->subdev.entity.function = MEDIA_ENT_F_PROC_VIDEO_PIXEL_FORMATTER;
	cam->subdev.entity.ops = &mtk_cam_media_entity_ops;
	cam->subdev.internal_ops = &mtk_cam_internal_ops;
	cam->subdev.flags = V4L2_SUBDEV_FL_HAS_DEVNODE;
	strscpy(cam->subdev.name, dev_name(dev), sizeof(cam->subdev.name));
	v4l2_set_subdevdata(&cam->subdev, cam);

	v4l2_subdev_init_finalize(&cam->subdev);

	ret = v4l2_async_register_subdev(&cam->subdev);
	if (ret) {
		dev_err(dev, "failed to initialize subdev:%d\n", ret);
		media_entity_cleanup(&cam->subdev.entity);
		return ret;
	}

	return 0;
}

/*
 * Unregistering the sub-device from the seninf unregisters the video node
 * (mtk_cam_subdev_unregistered()); a stream stops there. Open files keep
 * the nodes, which answer -ENODEV from now on, and the structure.
 */
static void mtk_cam_v4l2_unregister(struct mtk_cam_dev *cam)
{
	v4l2_async_unregister_subdev(&cam->subdev);
	v4l2_subdev_cleanup(&cam->subdev);
	media_entity_cleanup(&cam->subdev.entity);
}

static void mtk_cam_dev_release(struct kref *kref)
{
	kfree(container_of(kref, struct mtk_cam_dev, kref));
}

void mtk_cam_dev_get(struct mtk_cam_dev *cam)
{
	kref_get(&cam->kref);
}

void mtk_cam_dev_put(struct mtk_cam_dev *cam)
{
	kref_put(&cam->kref, mtk_cam_dev_release);
}

int mtk_cam_dev_init(struct mtk_cam_dev *cam_dev)
{
	/* v4l2 sub-device registration */
	return mtk_cam_v4l2_register(cam_dev);
}

void mtk_cam_dev_cleanup(struct mtk_cam_dev *cam)
{
	mtk_cam_v4l2_unregister(cam);
}

/* -----------------------------------------------------------------------------
 * V4L2 Video IOCTLs
 */

static int mtk_cam_vidioc_querycap(struct file *file, void *fh,
				   struct v4l2_capability *cap)
{
	struct mtk_cam_dev *cam = video_drvdata(file);

	strscpy(cap->driver, dev_driver_string(cam->dev), sizeof(cap->driver));
	strscpy(cap->card, dev_driver_string(cam->dev), sizeof(cap->card));

	return 0;
}

static int mtk_cam_vidioc_enum_fmt(struct file *file, void *fh,
				   struct v4l2_fmtdesc *f)
{
	struct mtk_cam_video_device *vdev = file_to_mtk_cam_video_device(file);
	const struct mtk_cam_format_info *fmtinfo;
	unsigned int i, n = 0;

	/* If mbus_code is not set enumerate all supported formats. */
	if (!f->mbus_code) {
		if (f->index >= vdev->desc->num_fmts)
			return -EINVAL;

		/* f->description is filled in v4l_fill_fmtdesc function */
		f->pixelformat = vdev->desc->fmts[f->index];
		f->flags = 0;

		return 0;
	}

	/*
	 * Otherwise only enumerate the pixel formats the bus code can be
	 * written as: 8 bits and MediaTek packed for 10-bit Bayer.
	 */
	for (i = 0; i < ARRAY_SIZE(mtk_cam_format_info); ++i) {
		fmtinfo = &mtk_cam_format_info[i];
		if (fmtinfo->code != f->mbus_code ||
		    !mtk_cam_dev_find_fmt(vdev->desc, fmtinfo->fourcc))
			continue;
		if (n++ == f->index) {
			f->pixelformat = fmtinfo->fourcc;
			f->flags = 0;
			return 0;
		}
	}

	return -EINVAL;
}

static int mtk_cam_vidioc_g_fmt(struct file *file, void *fh,
				struct v4l2_format *f)
{
	struct mtk_cam_video_device *vdev = file_to_mtk_cam_video_device(file);

	f->fmt.pix_mp = vdev->format;

	return 0;
}

static int mtk_cam_vidioc_try_fmt(struct file *file, void *fh,
				  struct v4l2_format *f)
{
	struct mtk_cam_video_device *vdev = file_to_mtk_cam_video_device(file);
	struct v4l2_pix_format_mplane *pix_mp = &f->fmt.pix_mp;
	const struct mtk_cam_format_info *fmtinfo;

	/* Validate pixelformat */
	if (!mtk_cam_dev_find_fmt(vdev->desc, pix_mp->pixelformat))
		pix_mp->pixelformat = vdev->desc->fmts[0];

	pix_mp->width = clamp_val(pix_mp->width, IMG_MIN_WIDTH, IMG_MAX_WIDTH);
	pix_mp->height = clamp_val(pix_mp->height, IMG_MIN_HEIGHT,
				   IMG_MAX_HEIGHT);

	pix_mp->num_planes = 1;

	fmtinfo = mtk_cam_format_info_by_fourcc(pix_mp->pixelformat);
	calc_bpl_size_pix_mp(fmtinfo, pix_mp);

	/* Constant format fields */
	pix_mp->colorspace = V4L2_COLORSPACE_SRGB;
	pix_mp->field = V4L2_FIELD_NONE;
	pix_mp->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	pix_mp->quantization = V4L2_QUANTIZATION_DEFAULT;
	pix_mp->xfer_func = V4L2_XFER_FUNC_DEFAULT;

	return 0;
}

static int mtk_cam_vidioc_s_fmt(struct file *file, void *fh,
				struct v4l2_format *f)
{
	struct mtk_cam_video_device *vdev = file_to_mtk_cam_video_device(file);
	int ret;

	if (vb2_is_busy(vdev->vdev.queue))
		return -EBUSY;

	ret = mtk_cam_vidioc_try_fmt(file, fh, f);
	if (ret)
		return ret;

	/* Configure to video device */
	vdev->format = f->fmt.pix_mp;
	vdev->fmtinfo =
		mtk_cam_format_info_by_fourcc(f->fmt.pix_mp.pixelformat);

	return 0;
}

static int mtk_cam_vidioc_enum_framesizes(struct file *file, void *priv,
					  struct v4l2_frmsizeenum *sizes)
{
	struct mtk_cam_video_device *vdev = file_to_mtk_cam_video_device(file);

	if (sizes->index)
		return -EINVAL;

	if (!mtk_cam_dev_find_fmt(vdev->desc, sizes->pixel_format))
		return -EINVAL;

	sizes->type = vdev->desc->frmsizes->type;
	memcpy(&sizes->stepwise, &vdev->desc->frmsizes->stepwise,
	       sizeof(sizes->stepwise));

	return 0;
}

static const struct v4l2_ioctl_ops mtk_cam_v4l2_vcap_ioctl_ops = {
	.vidioc_querycap = mtk_cam_vidioc_querycap,
	.vidioc_enum_framesizes = mtk_cam_vidioc_enum_framesizes,
	.vidioc_enum_fmt_vid_cap = mtk_cam_vidioc_enum_fmt,
	.vidioc_g_fmt_vid_cap_mplane = mtk_cam_vidioc_g_fmt,
	.vidioc_s_fmt_vid_cap_mplane = mtk_cam_vidioc_s_fmt,
	.vidioc_try_fmt_vid_cap_mplane = mtk_cam_vidioc_try_fmt,
	.vidioc_reqbufs = vb2_ioctl_reqbufs,
	.vidioc_create_bufs = vb2_ioctl_create_bufs,
	.vidioc_prepare_buf = vb2_ioctl_prepare_buf,
	.vidioc_querybuf = vb2_ioctl_querybuf,
	.vidioc_qbuf = vb2_ioctl_qbuf,
	.vidioc_dqbuf = vb2_ioctl_dqbuf,
	.vidioc_streamon = vb2_ioctl_streamon,
	.vidioc_streamoff = vb2_ioctl_streamoff,
	.vidioc_expbuf = vb2_ioctl_expbuf,
	.vidioc_subscribe_event = v4l2_ctrl_subscribe_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,
};

static const struct v4l2_file_operations mtk_cam_v4l2_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = video_ioctl2,
	.open = v4l2_fh_open,
	.release = vb2_fop_release,
	.poll = vb2_fop_poll,
	.mmap = vb2_fop_mmap,
#ifdef CONFIG_COMPAT
	.compat_ioctl32 = v4l2_compat_ioctl32,
#endif
};

/* -----------------------------------------------------------------------------
 * Init & Cleanup
 */


static const struct mtk_cam_vdev_desc video_stream = {
	.fmts = mtk_cam_stream_fmts,
	.num_fmts = ARRAY_SIZE(mtk_cam_stream_fmts),
	.frmsizes =
		&(struct v4l2_frmsizeenum){
			.index = 0,
			.type = V4L2_FRMSIZE_TYPE_CONTINUOUS,
			.stepwise = {
				.max_width = IMG_MAX_WIDTH,
				.min_width = IMG_MIN_WIDTH,
				.max_height = IMG_MAX_HEIGHT,
				.min_height = IMG_MIN_HEIGHT,
				.step_height = 1,
				.step_width = 1,
			},
		},
};

/* The last open file of the node is gone, after its unregistration */
static void mtk_cam_video_release(struct video_device *vdev)
{
	struct mtk_cam_video_device *node =
		container_of(vdev, struct mtk_cam_video_device, vdev);
	struct mtk_cam_dev *cam = node->cam;

	media_entity_cleanup(&vdev->entity);
	mutex_destroy(&node->vdev_lock);
	kfree(node);
	mtk_cam_dev_put(cam);
}

int mtk_cam_video_register(struct mtk_cam_dev *cam)
{
	struct device *dev = cam->dev;
	struct mtk_cam_video_device *cam_vdev;
	struct video_device *vdev;
	struct vb2_queue *vbq;
	int ret;

	vb2_dma_contig_set_max_seg_size(cam->dev, DMA_BIT_MASK(32));

	/*
	 * A new node each time the seninf binds us: the node of an earlier
	 * bind can still be open, and stays until its release.
	 */
	cam_vdev = kzalloc_obj(*cam_vdev);
	if (!cam_vdev)
		return -ENOMEM;
	cam_vdev->cam = cam;
	mutex_init(&cam_vdev->vdev_lock);
	vdev = &cam_vdev->vdev;
	vbq = &cam_vdev->vbq;

	cam_vdev->desc = &video_stream;

	/* Initialize mtk_cam_video_device */
	mtk_cam_dev_load_default_fmt(cam, cam_vdev);

	cam_vdev->vdev_pad.flags = MEDIA_PAD_FL_SOURCE;

	/* Initialize media entities */
	ret = media_entity_pads_init(&vdev->entity, 1, &cam_vdev->vdev_pad);
	if (ret) {
		dev_err(dev, "failed to initialize media pad:%d\n", ret);
		goto fail_free;
	}
	cam_vdev->vdev_pad.flags = MEDIA_PAD_FL_SINK;

	vbq->type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	vbq->io_modes = VB2_MMAP | VB2_DMABUF;
	vbq->dev = dev;
	vbq->ops = &mtk_cam_vb2_ops;
	vbq->mem_ops = &vb2_dma_contig_memops;
	vbq->buf_struct_size = sizeof(struct mtk_cam_dev_buffer);
	/*
	 * Large frames need large physically contiguous buffers: when CMA
	 * cannot provide one the allocation fails with -ENOMEM, without the
	 * page allocator warning about the order.
	 */
	vbq->gfp_flags = __GFP_NOWARN;
	/*
	 * TODO: The hardware supports SOF interrupts, switch to a SOF
	 * timestamp source would give better accuracy, but first requires
	 * extending the V4L2 API to support it.
	 */
	vbq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC
			     | V4L2_BUF_FLAG_TSTAMP_SRC_EOF;

	/*
	 * Without FBC the DMA needs a buffer from the start: the scratch
	 * buffer is optional (CMA), so start with one queued buffer.
	 */
	vbq->min_queued_buffers = cam->conf->has_fbc ? 0 : 1;
	vbq->drv_priv = cam;

	vbq->lock = &cam_vdev->vdev_lock;
	ret = vb2_queue_init(vbq);
	if (ret) {
		dev_err(dev, "failed to init. vb2 queue:%d\n", ret);
		goto fail_media_clean;
	}

	/* Initialize vdev */
	snprintf(vdev->name, sizeof(vdev->name), "%s video stream",
		 dev_name(dev));

	/* Set cap/type/ioctl_ops of the video device */
	vdev->device_caps = V4L2_CAP_VIDEO_CAPTURE_MPLANE | V4L2_CAP_STREAMING
			  | V4L2_CAP_IO_MC;
	vdev->ioctl_ops = &mtk_cam_v4l2_vcap_ioctl_ops;
	vdev->fops = &mtk_cam_v4l2_fops;
	/*
	 * The release of the node frees it, but only once it is registered:
	 * a failed registration can end in the release or not, depending on
	 * the step that failed.
	 */
	vdev->release = video_device_release_empty;
	vdev->lock = &cam_vdev->vdev_lock;
	vdev->v4l2_dev = cam->subdev.v4l2_dev;
	vdev->queue = &cam_vdev->vbq;
	vdev->vfl_dir = VFL_DIR_RX;
	vdev->entity.function = MEDIA_ENT_F_IO_V4L;
	video_set_drvdata(vdev, cam);

	/* Initialize miscellaneous variables */
	INIT_LIST_HEAD(&cam->buf_list);

	ret = video_register_device(vdev, VFL_TYPE_VIDEO, -1);
	if (ret) {
		dev_err(dev, "failed to register vde:%d\n", ret);
		goto fail_vb2_rel;
	}
	mtk_cam_dev_get(cam);
	vdev->release = mtk_cam_video_release;

	/* Create link between the video pad and the subdev pad. */
	ret = media_create_pad_link(&cam->subdev.entity,
				    MTK_CAM_CIO_PAD_VIDEO,
				    &vdev->entity, 0,
				    MEDIA_LNK_FL_IMMUTABLE
				    | MEDIA_LNK_FL_ENABLED);
	if (ret) {
		/* the release frees the node */
		vb2_video_unregister_device(vdev);
		return ret;
	}

	cam->vdev = cam_vdev;

	return 0;

fail_vb2_rel:
	vb2_queue_release(vbq);
fail_media_clean:
	media_entity_cleanup(&vdev->entity);
fail_free:
	mutex_destroy(&cam_vdev->vdev_lock);
	kfree(cam_vdev);

	return ret;
}

/*
 * The node answers -ENODEV from now on, also to files that stay open. A
 * stream stops here: the buffers of the driver go back with an error, and
 * the queue lets its buffers go.
 *
 * The V4L2 core keeps the entity of the node, with the link to its device
 * node interface, in the media graph until the last open file is released
 * (v4l2_device_release()). Until then the graph would show an entity whose
 * device node no longer exists, next to the one of a later bind, and
 * libcamera waits for that device node and drops the whole media device
 * (no camera at all). A process that keeps the node open (a camera manager
 * in the session) would make this last until it ends. Take the entity out
 * of the graph here instead; the stream has stopped above, so no pipeline
 * uses it any more. Only the device node interface without links stays
 * until the release, and the core skips the entity there (it is no longer
 * part of a media device). Without open files the release comes with the
 * unregistration: hold the node until the entity is out.
 */
void mtk_cam_video_unregister(struct mtk_cam_dev *cam)
{
	struct mtk_cam_video_device *cam_vdev = cam->vdev;
	struct video_device *vdev;

	if (!cam_vdev)
		return;

	cam->vdev = NULL;
	vdev = &cam_vdev->vdev;
	get_device(&vdev->dev);
	vb2_video_unregister_device(vdev);
	media_device_unregister_entity(&vdev->entity);
	put_device(&vdev->dev);
}
