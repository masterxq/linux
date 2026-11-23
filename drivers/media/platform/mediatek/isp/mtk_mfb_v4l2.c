// SPDX-License-Identifier: GPL-2.0-only
/*
 * MediaTek MT6785 multi-frame blending block (MFB): media device and video
 * nodes.
 *
 * One processing entity with a video node per stream, as described in
 * <linux/media/mediatek/mtk-mfb-mt6785.h>: base, reference and weight map
 * input, parameters, blended image and weight map output.
 */

#include <linux/align.h>
#include <linux/minmax.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-dma-contig.h>
#include <media/videobuf2-vmalloc.h>

#include "mtk_mfb.h"

#define MTK_MFB_NAME			"mtk-mfb"
#define MTK_MFB_BPL_ALIGN		16
#define MTK_MFB_BPL_MAX			(2 * MTK_MFB_MAX_W)

struct mtk_mfb_node_desc {
	const char *name;
	u32 buf_type;
	u32 caps;
	u32 fourcc;
	bool output;
};

static const struct mtk_mfb_node_desc mtk_mfb_node_desc[MTK_MFB_NODE_NUM] = {
	[MTK_MFB_NODE_BASE] = {
		"mtk-mfb-base", V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
		V4L2_CAP_VIDEO_OUTPUT_MPLANE, V4L2_PIX_FMT_NV12, true,
	},
	[MTK_MFB_NODE_REF] = {
		"mtk-mfb-ref", V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
		V4L2_CAP_VIDEO_OUTPUT_MPLANE, V4L2_PIX_FMT_NV12, true,
	},
	[MTK_MFB_NODE_WT_IN] = {
		"mtk-mfb-weight-in", V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE,
		V4L2_CAP_VIDEO_OUTPUT_MPLANE, V4L2_PIX_FMT_GREY, true,
	},
	[MTK_MFB_NODE_PARAMS] = {
		"mtk-mfb-params", V4L2_BUF_TYPE_META_OUTPUT,
		V4L2_CAP_META_OUTPUT, V4L2_META_FMT_MTK_MFB_PARAMS, true,
	},
	[MTK_MFB_NODE_OUT] = {
		"mtk-mfb-out", V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
		V4L2_CAP_VIDEO_CAPTURE_MPLANE, V4L2_PIX_FMT_NV12, false,
	},
	[MTK_MFB_NODE_WT_OUT] = {
		"mtk-mfb-weight-out", V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE,
		V4L2_CAP_VIDEO_CAPTURE_MPLANE, V4L2_PIX_FMT_GREY, false,
	},
};

static bool mtk_mfb_is_nv12(const struct mtk_mfb_node *node)
{
	return mtk_mfb_node_desc[node->id].fourcc == V4L2_PIX_FMT_NV12;
}

/* NV12 (Y, then UV behind it) or GREY, sizes on the steps of the block */
static void mtk_mfb_try_pix(const struct mtk_mfb_node *node,
			    struct v4l2_pix_format_mplane *pix)
{
	bool nv12 = mtk_mfb_is_nv12(node);
	u32 bpl;

	pix->pixelformat = mtk_mfb_node_desc[node->id].fourcc;
	pix->width = clamp(ALIGN(pix->width, MTK_MFB_W_STEP), MTK_MFB_MIN_W,
			   MTK_MFB_MAX_W);
	pix->height = clamp(ALIGN(pix->height, MTK_MFB_H_STEP), MTK_MFB_MIN_H,
			    MTK_MFB_MAX_H);
	pix->field = V4L2_FIELD_NONE;
	pix->num_planes = 1;
	if (nv12) {
		pix->colorspace = V4L2_COLORSPACE_SRGB;
		pix->xfer_func = V4L2_XFER_FUNC_DEFAULT;
		pix->ycbcr_enc = V4L2_YCBCR_ENC_601;
	} else {
		pix->colorspace = V4L2_COLORSPACE_RAW;
		pix->xfer_func = V4L2_XFER_FUNC_NONE;
		pix->ycbcr_enc = V4L2_YCBCR_ENC_DEFAULT;
	}
	pix->quantization = V4L2_QUANTIZATION_FULL_RANGE;
	pix->flags = 0;

	bpl = ALIGN(pix->plane_fmt[0].bytesperline, MTK_MFB_BPL_ALIGN);
	pix->plane_fmt[0].bytesperline =
		clamp(bpl, ALIGN(pix->width, MTK_MFB_BPL_ALIGN), MTK_MFB_BPL_MAX);
	pix->plane_fmt[0].sizeimage = pix->plane_fmt[0].bytesperline *
				      pix->height;
	if (nv12)
		pix->plane_fmt[0].sizeimage += pix->plane_fmt[0].sizeimage / 2;
	memset(pix->plane_fmt[0].reserved, 0,
	       sizeof(pix->plane_fmt[0].reserved));
	memset(pix->reserved, 0, sizeof(pix->reserved));
}

static u32 mtk_mfb_bpl(const struct mtk_mfb *mfb, enum mtk_mfb_node_id id)
{
	return mfb->node[id].fmt.fmt.pix_mp.plane_fmt[0].bytesperline;
}

/**
 * mtk_mfb_job_geom - geometry of a job from the formats of its nodes
 * @mfb: the device
 * @wt_in: the job reads a weight map
 * @g: result
 *
 * Return: 0, -EPIPE if the sizes differ, -EINVAL if the block cannot take
 * the geometry.
 */
int mtk_mfb_job_geom(struct mtk_mfb *mfb, bool wt_in, struct mtk_mfb_geom *g)
{
	static const enum mtk_mfb_node_id ids[] = {
		MTK_MFB_NODE_REF, MTK_MFB_NODE_WT_IN, MTK_MFB_NODE_OUT,
		MTK_MFB_NODE_WT_OUT,
	};
	const struct v4l2_pix_format_mplane *base =
		&mfb->node[MTK_MFB_NODE_BASE].fmt.fmt.pix_mp;
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(ids); i++) {
		const struct v4l2_pix_format_mplane *p =
			&mfb->node[ids[i]].fmt.fmt.pix_mp;

		if (ids[i] == MTK_MFB_NODE_WT_IN && !wt_in)
			continue;
		if (p->width != base->width || p->height != base->height)
			return -EPIPE;
	}

	memset(g, 0, sizeof(*g));
	g->w = base->width;
	g->h = base->height;
	g->base_bpl = mtk_mfb_bpl(mfb, MTK_MFB_NODE_BASE);
	g->ref_bpl = mtk_mfb_bpl(mfb, MTK_MFB_NODE_REF);
	g->out_bpl = mtk_mfb_bpl(mfb, MTK_MFB_NODE_OUT);
	g->wt_out_bpl = mtk_mfb_bpl(mfb, MTK_MFB_NODE_WT_OUT);
	if (wt_in)
		g->wt_in_bpl = mtk_mfb_bpl(mfb, MTK_MFB_NODE_WT_IN);

	return mtk_mfb_geom_check(g);
}

/* ioctls */

static int mtk_mfb_querycap(struct file *file, void *fh,
			    struct v4l2_capability *cap)
{
	struct mtk_mfb_node *node = video_drvdata(file);

	strscpy(cap->driver, MTK_MFB_NAME, sizeof(cap->driver));
	strscpy(cap->card, MTK_MFB_NAME, sizeof(cap->card));
	snprintf(cap->bus_info, sizeof(cap->bus_info), "platform:%s",
		 dev_name(node->mfb->dev));

	return 0;
}

static int mtk_mfb_enum_fmt(struct file *file, void *fh, struct v4l2_fmtdesc *f)
{
	struct mtk_mfb_node *node = video_drvdata(file);

	if (f->mbus_code || f->index)
		return -EINVAL;
	f->pixelformat = mtk_mfb_node_desc[node->id].fourcc;
	if (node->id == MTK_MFB_NODE_PARAMS)
		strscpy(f->description, "MediaTek MFB parameters",
			sizeof(f->description));

	return 0;
}

static int mtk_mfb_enum_framesizes(struct file *file, void *fh,
				   struct v4l2_frmsizeenum *fs)
{
	struct mtk_mfb_node *node = video_drvdata(file);

	if (fs->index || fs->pixel_format != mtk_mfb_node_desc[node->id].fourcc)
		return -EINVAL;

	fs->type = V4L2_FRMSIZE_TYPE_STEPWISE;
	fs->stepwise.min_width = MTK_MFB_MIN_W;
	fs->stepwise.max_width = MTK_MFB_MAX_W;
	fs->stepwise.step_width = MTK_MFB_W_STEP;
	fs->stepwise.min_height = MTK_MFB_MIN_H;
	fs->stepwise.max_height = MTK_MFB_MAX_H;
	fs->stepwise.step_height = MTK_MFB_H_STEP;

	return 0;
}

static int mtk_mfb_g_fmt(struct file *file, void *fh, struct v4l2_format *f)
{
	struct mtk_mfb_node *node = video_drvdata(file);

	*f = node->fmt;

	return 0;
}

static int mtk_mfb_try_fmt_pix(struct file *file, void *fh,
			       struct v4l2_format *f)
{
	struct mtk_mfb_node *node = video_drvdata(file);

	mtk_mfb_try_pix(node, &f->fmt.pix_mp);

	return 0;
}

static int mtk_mfb_s_fmt_pix(struct file *file, void *fh, struct v4l2_format *f)
{
	struct mtk_mfb_node *node = video_drvdata(file);

	if (vb2_is_busy(&node->queue))
		return -EBUSY;
	mtk_mfb_try_pix(node, &f->fmt.pix_mp);
	node->fmt = *f;

	return 0;
}

static int mtk_mfb_try_fmt_meta(struct file *file, void *fh,
				struct v4l2_format *f)
{
	f->fmt.meta.dataformat = V4L2_META_FMT_MTK_MFB_PARAMS;
	f->fmt.meta.buffersize = MTK_MFB_PARAMS_MAX_SIZE;

	return 0;
}

static int mtk_mfb_s_fmt_meta(struct file *file, void *fh,
			      struct v4l2_format *f)
{
	struct mtk_mfb_node *node = video_drvdata(file);

	if (vb2_is_busy(&node->queue))
		return -EBUSY;
	mtk_mfb_try_fmt_meta(file, fh, f);
	node->fmt = *f;

	return 0;
}

#define MTK_MFB_VB2_IOCTLS					\
	.vidioc_reqbufs = vb2_ioctl_reqbufs,			\
	.vidioc_create_bufs = vb2_ioctl_create_bufs,		\
	.vidioc_prepare_buf = vb2_ioctl_prepare_buf,		\
	.vidioc_querybuf = vb2_ioctl_querybuf,			\
	.vidioc_qbuf = vb2_ioctl_qbuf,				\
	.vidioc_dqbuf = vb2_ioctl_dqbuf,			\
	.vidioc_expbuf = vb2_ioctl_expbuf,			\
	.vidioc_streamon = vb2_ioctl_streamon,			\
	.vidioc_streamoff = vb2_ioctl_streamoff

static const struct v4l2_ioctl_ops mtk_mfb_out_ioctl_ops = {
	.vidioc_querycap = mtk_mfb_querycap,
	.vidioc_enum_fmt_vid_out = mtk_mfb_enum_fmt,
	.vidioc_enum_framesizes = mtk_mfb_enum_framesizes,
	.vidioc_g_fmt_vid_out_mplane = mtk_mfb_g_fmt,
	.vidioc_try_fmt_vid_out_mplane = mtk_mfb_try_fmt_pix,
	.vidioc_s_fmt_vid_out_mplane = mtk_mfb_s_fmt_pix,
	MTK_MFB_VB2_IOCTLS,
};

static const struct v4l2_ioctl_ops mtk_mfb_cap_ioctl_ops = {
	.vidioc_querycap = mtk_mfb_querycap,
	.vidioc_enum_fmt_vid_cap = mtk_mfb_enum_fmt,
	.vidioc_enum_framesizes = mtk_mfb_enum_framesizes,
	.vidioc_g_fmt_vid_cap_mplane = mtk_mfb_g_fmt,
	.vidioc_try_fmt_vid_cap_mplane = mtk_mfb_try_fmt_pix,
	.vidioc_s_fmt_vid_cap_mplane = mtk_mfb_s_fmt_pix,
	MTK_MFB_VB2_IOCTLS,
};

static const struct v4l2_ioctl_ops mtk_mfb_params_ioctl_ops = {
	.vidioc_querycap = mtk_mfb_querycap,
	.vidioc_enum_fmt_meta_out = mtk_mfb_enum_fmt,
	.vidioc_g_fmt_meta_out = mtk_mfb_g_fmt,
	.vidioc_try_fmt_meta_out = mtk_mfb_try_fmt_meta,
	.vidioc_s_fmt_meta_out = mtk_mfb_s_fmt_meta,
	MTK_MFB_VB2_IOCTLS,
};

static const struct v4l2_file_operations mtk_mfb_fops = {
	.owner = THIS_MODULE,
	.open = v4l2_fh_open,
	.release = vb2_fop_release,
	.poll = vb2_fop_poll,
	.unlocked_ioctl = video_ioctl2,
	.mmap = vb2_fop_mmap,
};

/* vb2 */

static u32 mtk_mfb_node_size(const struct mtk_mfb_node *node)
{
	if (node->id == MTK_MFB_NODE_PARAMS)
		return node->fmt.fmt.meta.buffersize;

	return node->fmt.fmt.pix_mp.plane_fmt[0].sizeimage;
}

/*
 * Largest buffer VIDIOC_CREATE_BUFS may ask for: twice the largest image
 * format, and for the small params buffers their size plus 1 MiB (what
 * v4l2-compliance creates).
 */
static u32 mtk_mfb_node_size_max(const struct mtk_mfb_node *node)
{
	if (node->id == MTK_MFB_NODE_PARAMS)
		return MTK_MFB_PARAMS_MAX_SIZE + SZ_1M;

	return 3 * MTK_MFB_BPL_MAX * MTK_MFB_MAX_H;
}

static int mtk_mfb_queue_setup(struct vb2_queue *q, unsigned int *nbuffers,
			       unsigned int *nplanes, unsigned int sizes[],
			       struct device *alloc_devs[])
{
	struct mtk_mfb_node *node = vb2_get_drv_priv(q);
	u32 size = mtk_mfb_node_size(node);

	if (*nplanes)
		return *nplanes != 1 || sizes[0] < size ||
		       sizes[0] > mtk_mfb_node_size_max(node) ? -EINVAL : 0;

	*nplanes = 1;
	sizes[0] = size;

	return 0;
}

static struct mtk_mfb_buffer *to_mtk_mfb_buffer(struct vb2_buffer *vb)
{
	return container_of(to_vb2_v4l2_buffer(vb), struct mtk_mfb_buffer, vb);
}

static int mtk_mfb_buf_prepare(struct vb2_buffer *vb)
{
	struct mtk_mfb_node *node = vb2_get_drv_priv(vb->vb2_queue);
	struct mtk_mfb *mfb = node->mfb;
	u32 size = mtk_mfb_node_size(node);
	dma_addr_t dma;

	if (node->id == MTK_MFB_NODE_PARAMS)
		return mtk_mfb_params_check(mfb, vb, to_mtk_mfb_buffer(vb));

	if (vb2_plane_size(vb, 0) < size) {
		dev_dbg(mfb->dev, "%s: buffer of %lu bytes, %u needed\n",
			node->vdev.name, vb2_plane_size(vb, 0), size);
		return -EINVAL;
	}
	if (vb->planes[0].data_offset) {
		dev_dbg(mfb->dev, "%s: data offset %u not supported\n",
			node->vdev.name, vb->planes[0].data_offset);
		return -EINVAL;
	}

	/* the block reaches 32-bit physical addresses only */
	dma = vb2_dma_contig_plane_dma_addr(vb, 0);
	if (upper_32_bits(dma + vb2_plane_size(vb, 0) - 1)) {
		dev_dbg(mfb->dev, "%s: buffer above 4 GiB\n", node->vdev.name);
		return -EINVAL;
	}

	if (!mtk_mfb_node_desc[node->id].output)
		vb2_set_plane_payload(vb, 0, size);
	else if (vb2_get_plane_payload(vb, 0) < size)
		return -EINVAL;

	return 0;
}

/* frames to the block are progressive; a frame queued with any field gets none */
static int mtk_mfb_buf_out_validate(struct vb2_buffer *vb)
{
	struct mtk_mfb_node *node = vb2_get_drv_priv(vb->vb2_queue);
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);

	if (vbuf->field == V4L2_FIELD_ANY)
		vbuf->field = V4L2_FIELD_NONE;
	if (node->id == MTK_MFB_NODE_PARAMS)
		return 0;

	return vbuf->field == V4L2_FIELD_NONE ? 0 : -EINVAL;
}

static void mtk_mfb_buf_queue(struct vb2_buffer *vb)
{
	struct mtk_mfb_node *node = vb2_get_drv_priv(vb->vb2_queue);
	struct mtk_mfb_buffer *buf = to_mtk_mfb_buffer(vb);
	struct mtk_mfb *mfb = node->mfb;
	unsigned long flags;

	spin_lock_irqsave(&mfb->lock, flags);
	list_add_tail(&buf->list, &node->ready);
	spin_unlock_irqrestore(&mfb->lock, flags);

	mtk_mfb_kick(mfb);
}

static void mtk_mfb_return_ready(struct mtk_mfb_node *node,
				 enum vb2_buffer_state state)
{
	struct mtk_mfb *mfb = node->mfb;
	struct mtk_mfb_buffer *buf, *tmp;
	unsigned long flags;
	LIST_HEAD(list);

	spin_lock_irqsave(&mfb->lock, flags);
	list_splice_init(&node->ready, &list);
	spin_unlock_irqrestore(&mfb->lock, flags);

	list_for_each_entry_safe(buf, tmp, &list, list) {
		list_del(&buf->list);
		vb2_buffer_done(&buf->vb.vb2_buf, state);
	}
}

/*
 * Checked when a node starts: the geometry of the jobs it can take part in,
 * with the formats the nodes hold now (no format changes while a queue is
 * busy): without the weight map input unless the starting node is that
 * input, and with it while it streams or starts.
 */
static int mtk_mfb_check_link(struct mtk_mfb_node *node)
{
	struct mtk_mfb *mfb = node->mfb;
	struct mtk_mfb_layout *l;
	struct mtk_mfb_geom g;
	unsigned long flags;
	unsigned int wt, wts;
	int ret = 0;

	if (node->id == MTK_MFB_NODE_PARAMS)
		return 0;

	spin_lock_irqsave(&mfb->lock, flags);
	wts = mfb->streaming & BIT(MTK_MFB_NODE_WT_IN) ? BIT(0) | BIT(1) : BIT(0);
	spin_unlock_irqrestore(&mfb->lock, flags);
	if (node->id == MTK_MFB_NODE_WT_IN)
		wts = BIT(1);

	l = kzalloc_obj(*l);
	if (!l)
		return -ENOMEM;
	for (wt = 0; wt < 2 && !ret; wt++) {
		if (!(wts & BIT(wt)))
			continue;
		ret = mtk_mfb_job_geom(mfb, wt, &g);
		if (!ret)
			ret = mtk_mfb_tiles(&g, l);
		if (ret)
			dev_dbg(mfb->dev, "%s: no job with%s weight map input: %d\n",
				node->vdev.name, wt ? "" : "out", ret);
	}
	kfree(l);

	return ret ? -EPIPE : 0;
}

static int mtk_mfb_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct mtk_mfb_node *node = vb2_get_drv_priv(q);
	struct mtk_mfb *mfb = node->mfb;
	unsigned long flags;
	int ret;

	ret = mtk_mfb_check_link(node);
	if (ret) {
		mtk_mfb_return_ready(node, VB2_BUF_STATE_QUEUED);
		return ret;
	}

	spin_lock_irqsave(&mfb->lock, flags);
	if (!mfb->streaming)
		mfb->sequence = 0;
	mfb->streaming |= BIT(node->id);
	spin_unlock_irqrestore(&mfb->lock, flags);

	mtk_mfb_kick(mfb);

	return 0;
}

static void mtk_mfb_stop_streaming(struct vb2_queue *q)
{
	struct mtk_mfb_node *node = vb2_get_drv_priv(q);
	struct mtk_mfb *mfb = node->mfb;
	unsigned long flags;
	bool idle;

	spin_lock_irqsave(&mfb->lock, flags);
	mfb->streaming &= ~BIT(node->id);
	spin_unlock_irqrestore(&mfb->lock, flags);

	/* buffers not in a job yet, then wait for the running job */
	mtk_mfb_return_ready(node, VB2_BUF_STATE_ERROR);
	vb2_wait_for_all_buffers(q);

	mutex_lock(&mfb->run_lock);
	spin_lock_irqsave(&mfb->lock, flags);
	idle = !mfb->streaming;
	spin_unlock_irqrestore(&mfb->lock, flags);
	if (idle)
		mtk_mfb_aux_free_all(mfb);
	mutex_unlock(&mfb->run_lock);
}

static const struct vb2_ops mtk_mfb_vb2_ops = {
	.queue_setup = mtk_mfb_queue_setup,
	.buf_out_validate = mtk_mfb_buf_out_validate,
	.buf_prepare = mtk_mfb_buf_prepare,
	.buf_queue = mtk_mfb_buf_queue,
	.start_streaming = mtk_mfb_start_streaming,
	.stop_streaming = mtk_mfb_stop_streaming,
};

static void mtk_mfb_node_defaults(struct mtk_mfb_node *node)
{
	struct v4l2_format *f = &node->fmt;

	memset(f, 0, sizeof(*f));
	f->type = mtk_mfb_node_desc[node->id].buf_type;
	if (node->id == MTK_MFB_NODE_PARAMS) {
		f->fmt.meta.dataformat = V4L2_META_FMT_MTK_MFB_PARAMS;
		f->fmt.meta.buffersize = MTK_MFB_PARAMS_MAX_SIZE;
		return;
	}
	f->fmt.pix_mp.width = MTK_MFB_MAX_W;
	f->fmt.pix_mp.height = MTK_MFB_MAX_H;
	mtk_mfb_try_pix(node, &f->fmt.pix_mp);
}

static int mtk_mfb_node_init(struct mtk_mfb *mfb, enum mtk_mfb_node_id id)
{
	const struct mtk_mfb_node_desc *desc = &mtk_mfb_node_desc[id];
	struct mtk_mfb_node *node = &mfb->node[id];
	struct video_device *vdev = &node->vdev;
	struct vb2_queue *q = &node->queue;
	u32 flags;
	int ret;

	node->mfb = mfb;
	node->id = id;
	INIT_LIST_HEAD(&node->ready);
	mtk_mfb_node_defaults(node);

	q->type = desc->buf_type;
	q->io_modes = VB2_MMAP | VB2_DMABUF;
	q->drv_priv = node;
	q->ops = &mtk_mfb_vb2_ops;
	if (id == MTK_MFB_NODE_PARAMS) {
		q->mem_ops = &vb2_vmalloc_memops;
		q->buf_struct_size = sizeof(struct mtk_mfb_buffer);
	} else {
		q->mem_ops = &vb2_dma_contig_memops;
		q->buf_struct_size = offsetof(struct mtk_mfb_buffer, params);
	}
	/* a buffer that does not fit into the memory region fails quietly */
	q->gfp_flags = __GFP_NOWARN;
	q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
	q->dev = mfb->dev;
	q->lock = &node->lock;
	ret = vb2_queue_init(q);
	if (ret)
		return ret;

	strscpy(vdev->name, desc->name, sizeof(vdev->name));
	vdev->v4l2_dev = &mfb->v4l2_dev;
	vdev->fops = &mtk_mfb_fops;
	if (id == MTK_MFB_NODE_PARAMS)
		vdev->ioctl_ops = &mtk_mfb_params_ioctl_ops;
	else if (desc->output)
		vdev->ioctl_ops = &mtk_mfb_out_ioctl_ops;
	else
		vdev->ioctl_ops = &mtk_mfb_cap_ioctl_ops;
	vdev->release = video_device_release_empty;
	vdev->vfl_dir = desc->output ? VFL_DIR_TX : VFL_DIR_RX;
	vdev->device_caps = desc->caps | V4L2_CAP_STREAMING;
	vdev->lock = &node->lock;
	vdev->queue = q;
	video_set_drvdata(vdev, node);

	node->pad.flags = desc->output ? MEDIA_PAD_FL_SOURCE : MEDIA_PAD_FL_SINK;
	ret = media_entity_pads_init(&vdev->entity, 1, &node->pad);
	if (ret)
		return ret;

	ret = video_register_device(vdev, VFL_TYPE_VIDEO, -1);
	if (ret)
		goto err_entity;

	/* the image nodes take part when they stream, params can be switched off */
	flags = MEDIA_LNK_FL_ENABLED;
	if (id != MTK_MFB_NODE_PARAMS)
		flags |= MEDIA_LNK_FL_IMMUTABLE;
	if (desc->output)
		ret = media_create_pad_link(&vdev->entity, 0, &mfb->sd.entity,
					    id, flags);
	else
		ret = media_create_pad_link(&mfb->sd.entity, id, &vdev->entity,
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

static void mtk_mfb_node_cleanup(struct mtk_mfb_node *node)
{
	vb2_video_unregister_device(&node->vdev);
	media_entity_cleanup(&node->vdev.entity);
}

static const struct v4l2_subdev_ops mtk_mfb_sd_ops = {};

/* Only the link of the params node can change */
static int mtk_mfb_link_setup(struct media_entity *entity,
			      const struct media_pad *local,
			      const struct media_pad *remote, u32 flags)
{
	struct mtk_mfb *mfb = container_of(entity, struct mtk_mfb, sd.entity);
	unsigned long irqflags;

	if (local->index != MTK_MFB_NODE_PARAMS)
		return -EINVAL;

	spin_lock_irqsave(&mfb->lock, irqflags);
	mfb->params_link = flags & MEDIA_LNK_FL_ENABLED;
	spin_unlock_irqrestore(&mfb->lock, irqflags);

	return 0;
}

static const struct media_entity_operations mtk_mfb_entity_ops = {
	.link_setup = mtk_mfb_link_setup,
};

/* The last open file of a node is gone, after the unbind */
static void mtk_mfb_release(struct v4l2_device *v4l2_dev)
{
	struct mtk_mfb *mfb = container_of(v4l2_dev, struct mtk_mfb, v4l2_dev);
	unsigned int i;

	for (i = 0; i < MTK_MFB_NODE_NUM; i++)
		mutex_destroy(&mfb->node[i].lock);
	media_device_cleanup(&mfb->mdev);
	kfree(mfb);
}

/* The unbind drops the reference of the driver */
static void mtk_mfb_put(void *data)
{
	struct mtk_mfb *mfb = data;

	v4l2_device_unregister(&mfb->v4l2_dev);
	v4l2_device_put(&mfb->v4l2_dev);
}

/**
 * mtk_mfb_alloc - device state, with the V4L2 and the media device
 * @dev: the platform device
 *
 * Open files of the video nodes keep the nodes, their queues and locks, and
 * the media device after the driver is unbound. The state goes with the
 * last of them, in the release of the V4L2 device; the reference of the
 * driver goes with the unbind or a failed probe.
 *
 * Return: the state or an error pointer.
 */
struct mtk_mfb *mtk_mfb_alloc(struct device *dev)
{
	struct media_device *mdev;
	struct mtk_mfb *mfb;
	unsigned int i;
	int ret;

	mfb = kzalloc_obj(*mfb);
	if (!mfb)
		return ERR_PTR(-ENOMEM);
	mfb->dev = dev;
	for (i = 0; i < MTK_MFB_NODE_NUM; i++)
		mutex_init(&mfb->node[i].lock);

	mdev = &mfb->mdev;
	mdev->dev = dev;
	strscpy(mdev->model, MTK_MFB_NAME, sizeof(mdev->model));
	snprintf(mdev->bus_info, sizeof(mdev->bus_info), "platform:%s",
		 dev_name(dev));
	media_device_init(mdev);

	mfb->v4l2_dev.mdev = mdev;
	mfb->v4l2_dev.release = mtk_mfb_release;
	ret = v4l2_device_register(dev, &mfb->v4l2_dev);
	if (ret) {
		mtk_mfb_release(&mfb->v4l2_dev);
		return ERR_PTR(ret);
	}

	ret = devm_add_action_or_reset(dev, mtk_mfb_put, mfb);
	if (ret)
		return ERR_PTR(ret);

	return mfb;
}

/**
 * mtk_mfb_v4l2_register - media device, processing entity and video nodes
 * @mfb: the device
 *
 * Return: 0 or a negative error code.
 */
int mtk_mfb_v4l2_register(struct mtk_mfb *mfb)
{
	unsigned int i;
	int ret;

	v4l2_subdev_init(&mfb->sd, &mtk_mfb_sd_ops);
	mfb->sd.owner = THIS_MODULE;
	mfb->sd.entity.function = MEDIA_ENT_F_PROC_VIDEO_ISP;
	mfb->sd.entity.ops = &mtk_mfb_entity_ops;
	mfb->params_link = true;
	strscpy(mfb->sd.name, MTK_MFB_NAME, sizeof(mfb->sd.name));
	for (i = 0; i < MTK_MFB_NODE_NUM; i++)
		mfb->sd_pads[i].flags = mtk_mfb_node_desc[i].output ?
					MEDIA_PAD_FL_SINK : MEDIA_PAD_FL_SOURCE;
	ret = media_entity_pads_init(&mfb->sd.entity, MTK_MFB_NODE_NUM,
				     mfb->sd_pads);
	if (ret)
		return ret;
	ret = v4l2_device_register_subdev(&mfb->v4l2_dev, &mfb->sd);
	if (ret)
		goto err_sd_entity;

	for (i = 0; i < MTK_MFB_NODE_NUM; i++) {
		ret = mtk_mfb_node_init(mfb, i);
		if (ret)
			goto err_nodes;
	}

	ret = media_device_register(&mfb->mdev);
	if (ret)
		goto err_nodes;

	return 0;

err_nodes:
	while (i--)
		mtk_mfb_node_cleanup(&mfb->node[i]);
	v4l2_device_unregister_subdev(&mfb->sd);
err_sd_entity:
	media_entity_cleanup(&mfb->sd.entity);
	return ret;
}

/**
 * mtk_mfb_v4l2_unregister - undo mtk_mfb_v4l2_register()
 * @mfb: the device
 *
 * Stops the streams; no job runs or starts after this.
 */
void mtk_mfb_v4l2_unregister(struct mtk_mfb *mfb)
{
	unsigned int i;

	media_device_unregister(&mfb->mdev);
	for (i = 0; i < MTK_MFB_NODE_NUM; i++)
		mtk_mfb_node_cleanup(&mfb->node[i]);
	v4l2_device_unregister_subdev(&mfb->sd);
	media_entity_cleanup(&mfb->sd.entity);
}
