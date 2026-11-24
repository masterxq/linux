// SPDX-License-Identifier: GPL-2.0-only
/* V4L2 memory-to-memory detiler for MT6785 decoder output. */
#include <linux/dma-mapping.h>
#include <linux/module.h>
#include <linux/of_platform.h>
#include <linux/pm_runtime.h>
#include <linux/sizes.h>
#include <media/v4l2-ioctl.h>
#include "mdp-linear.h"

struct mdp_linear_buffer {
	struct v4l2_m2m_buffer buffer;
	unsigned long accounted;
};

static struct mdp_linear_ctx *mdp_linear_context(void *priv)
{
	return container_of(priv, struct mdp_linear_ctx, fh);
}

static bool mdp_linear_source_format(u32 format)
{
	return format == V4L2_PIX_FMT_MM21 || mdp_linear_tenbit(format);
}

static struct v4l2_pix_format_mplane *mdp_linear_format(struct mdp_linear_ctx *ctx,
							enum v4l2_buf_type type)
{
	return V4L2_TYPE_IS_OUTPUT(type) ? &ctx->source : &ctx->target;
}

static int mdp_linear_try_format(struct file *file, void *priv, struct v4l2_format *f)
{
	struct v4l2_pix_format_mplane *pix = &f->fmt.pix_mp;
	bool source = V4L2_TYPE_IS_OUTPUT(f->type);
	u32 stride, height;

	if (f->type != V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE &&
	    f->type != V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE)
		return -EINVAL;
	if (source && !mdp_linear_source_format(pix->pixelformat))
		pix->pixelformat = V4L2_PIX_FMT_MM21;
	if (!source && pix->pixelformat != V4L2_PIX_FMT_NV12 &&
	    pix->pixelformat != V4L2_PIX_FMT_P010)
		pix->pixelformat = V4L2_PIX_FMT_NV12;
	pix->width = ALIGN(clamp(pix->width, 64U, MDP_LINEAR_MAX_WIDTH), 16);
	pix->height = ALIGN(clamp(pix->height, 64U, ALIGN(MDP_LINEAR_MAX_HEIGHT, 32)),
			    source ? 32 : 2);
	pix->field = V4L2_FIELD_NONE;
	pix->num_planes = source ? 2 : 1;
	stride = pix->width;
	if (source && mdp_linear_tenbit(pix->pixelformat))
		stride = stride * 5 / 4;
	else if (pix->pixelformat == V4L2_PIX_FMT_P010)
		stride *= 2;
	stride = max(stride, min(pix->plane_fmt[0].bytesperline,
				 MDP_LINEAR_MAX_WIDTH * 2));
	stride = roundup(stride, source && mdp_linear_tenbit(pix->pixelformat) ? 20 : 16);
	if (stride > MDP_LINEAR_MAX_WIDTH * 2)
		return -EINVAL;
	height = pix->height;
	memset(pix->plane_fmt, 0, sizeof(pix->plane_fmt));
	pix->plane_fmt[0].bytesperline = stride;
	pix->plane_fmt[0].sizeimage = source ? stride * height : stride * height * 3 / 2;
	if (source) {
		pix->plane_fmt[1].bytesperline = stride;
		pix->plane_fmt[1].sizeimage = stride * height / 2;
	}
	if (source) {
		if (!pix->colorspace)
			pix->colorspace = V4L2_COLORSPACE_REC709;
	} else {
		struct mdp_linear_ctx *ctx = mdp_linear_context(file_to_v4l2_fh(file));

		pix->colorspace = ctx->source.colorspace;
		pix->ycbcr_enc = ctx->source.ycbcr_enc;
		pix->quantization = ctx->source.quantization;
		pix->xfer_func = ctx->source.xfer_func;
	}
	memset(pix->reserved, 0, sizeof(pix->reserved));
	return 0;
}

static int mdp_linear_get_format(struct file *file, void *priv, struct v4l2_format *f)
{
	struct mdp_linear_ctx *ctx = mdp_linear_context(file_to_v4l2_fh(file));

	f->fmt.pix_mp = *mdp_linear_format(ctx, f->type);
	return 0;
}

static int mdp_linear_set_format(struct file *file, void *priv, struct v4l2_format *f)
{
	struct mdp_linear_ctx *ctx = mdp_linear_context(file_to_v4l2_fh(file));
	struct vb2_queue *q = v4l2_m2m_get_vq(ctx->fh.m2m_ctx, f->type);
	int ret;

	if (!q)
		return -EINVAL;
	if (vb2_is_busy(q))
		return -EBUSY;
	ret = mdp_linear_try_format(file, priv, f);
	if (ret)
		return ret;
	*mdp_linear_format(ctx, f->type) = f->fmt.pix_mp;
	if (V4L2_TYPE_IS_OUTPUT(f->type)) {
		ctx->target.colorspace = ctx->source.colorspace;
		ctx->target.ycbcr_enc = ctx->source.ycbcr_enc;
		ctx->target.quantization = ctx->source.quantization;
		ctx->target.xfer_func = ctx->source.xfer_func;
		ctx->crop = (struct v4l2_rect) {
			.width = f->fmt.pix_mp.width,
			.height = min(f->fmt.pix_mp.height, MDP_LINEAR_MAX_HEIGHT),
		};
	}
	return 0;
}

static int mdp_linear_enum_format(struct file *file, void *priv, struct v4l2_fmtdesc *f)
{
	static const u32 source[] = { V4L2_PIX_FMT_MM21, V4L2_PIX_FMT_MT2110R,
				     V4L2_PIX_FMT_MT2110T };
	static const u32 target[] = { V4L2_PIX_FMT_NV12, V4L2_PIX_FMT_P010 };
	bool output = V4L2_TYPE_IS_OUTPUT(f->type);

	if (f->index >= (output ? ARRAY_SIZE(source) : ARRAY_SIZE(target)))
		return -EINVAL;
	f->pixelformat = output ? source[f->index] : target[f->index];
	return 0;
}

static int mdp_linear_querycap(struct file *file, void *priv, struct v4l2_capability *cap)
{
	strscpy(cap->driver, MDP_LINEAR_NAME, sizeof(cap->driver));
	strscpy(cap->card, "MediaTek linear video", sizeof(cap->card));
	return 0;
}

static int mdp_linear_get_selection(struct file *file, void *priv, struct v4l2_selection *s)
{
	struct mdp_linear_ctx *ctx = mdp_linear_context(file_to_v4l2_fh(file));

	if (s->type != V4L2_BUF_TYPE_VIDEO_OUTPUT)
		return -EINVAL;
	switch (s->target) {
	case V4L2_SEL_TGT_CROP:
		s->r = ctx->crop;
		break;
	case V4L2_SEL_TGT_CROP_DEFAULT:
	case V4L2_SEL_TGT_CROP_BOUNDS:
		s->r = (struct v4l2_rect) { .width = ctx->source.width,
			.height = min(ctx->source.height, MDP_LINEAR_MAX_HEIGHT) };
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static int mdp_linear_set_selection(struct file *file, void *priv, struct v4l2_selection *s)
{
	struct mdp_linear_ctx *ctx = mdp_linear_context(file_to_v4l2_fh(file));

	if (s->type != V4L2_BUF_TYPE_VIDEO_OUTPUT || s->target != V4L2_SEL_TGT_CROP ||
	    s->r.left || s->r.top || s->r.width < 64 || s->r.height < 64 ||
	    s->r.width > ctx->source.width || s->r.height > ctx->source.height ||
	    s->r.width > MDP_LINEAR_MAX_WIDTH || s->r.height > MDP_LINEAR_MAX_HEIGHT ||
	    (s->r.width | s->r.height) & 1)
		return -EINVAL;
	if (vb2_is_streaming(v4l2_m2m_get_src_vq(ctx->fh.m2m_ctx)) ||
	    vb2_is_streaming(v4l2_m2m_get_dst_vq(ctx->fh.m2m_ctx)))
		return -EBUSY;
	ctx->crop = s->r;
	return 0;
}

static int mdp_linear_queue_setup(struct vb2_queue *q, unsigned int *buffers,
				  unsigned int *planes, unsigned int sizes[],
				  struct device *alloc_devs[])
{
	struct mdp_linear_ctx *ctx = vb2_get_drv_priv(q);
	struct v4l2_pix_format_mplane *pix = mdp_linear_format(ctx, q->type);
	/* Only allocated buffers cost memory; imported ones belong to their exporter. */
	unsigned int limit = q->memory == VB2_MEMORY_MMAP ? MDP_LINEAR_MAX_BUFFERS :
			     MDP_LINEAR_MAX_IMPORTED;
	unsigned int i;

	if (vb2_get_num_buffers(q) >= limit)
		return -ENOSPC;
	*buffers = min(*buffers, limit - vb2_get_num_buffers(q));
	if (*planes) {
		if (*planes != pix->num_planes)
			return -EINVAL;
		for (i = 0; i < *planes; i++)
			if (sizes[i] < pix->plane_fmt[i].sizeimage)
				return -EINVAL;
	} else {
		*planes = pix->num_planes;
		for (i = 0; i < *planes; i++)
			sizes[i] = pix->plane_fmt[i].sizeimage;
	}
	return 0;
}

static int mdp_linear_buffer_init(struct vb2_buffer *vb)
{
	struct mdp_linear_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct mdp_linear_buffer *buf = container_of(to_vb2_v4l2_buffer(vb),
						   struct mdp_linear_buffer, buffer.vb);
	unsigned long bytes = 0;
	unsigned int i;

	if (vb->memory != VB2_MEMORY_MMAP)
		return 0;
	for (i = 0; i < vb->num_planes; i++)
		bytes += vb2_plane_size(vb, i);
	if (atomic_long_add_return(bytes, &ctx->mdp->queued_bytes) > MDP_LINEAR_MEMORY_LIMIT) {
		atomic_long_sub(bytes, &ctx->mdp->queued_bytes);
		return -ENOMEM;
	}
	buf->accounted = bytes;
	return 0;
}

static void mdp_linear_buffer_cleanup(struct vb2_buffer *vb)
{
	struct mdp_linear_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct mdp_linear_buffer *buf = container_of(to_vb2_v4l2_buffer(vb),
						   struct mdp_linear_buffer, buffer.vb);

	atomic_long_sub(buf->accounted, &ctx->mdp->queued_bytes);
	buf->accounted = 0;
}

static int mdp_linear_buffer_prepare(struct vb2_buffer *vb)
{
	struct mdp_linear_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);
	struct v4l2_pix_format_mplane *pix = mdp_linear_format(ctx, vb->type);
	unsigned int i;

	for (i = 0; i < pix->num_planes; i++) {
		u32 offset = V4L2_TYPE_IS_OUTPUT(vb->type) ? vb->planes[i].data_offset : 0;
		dma_addr_t address = vb2_dma_contig_plane_dma_addr(vb, i);

		if (offset > vb2_plane_size(vb, i) ||
		    pix->plane_fmt[i].sizeimage > vb2_plane_size(vb, i) - offset ||
		    address + offset + pix->plane_fmt[i].sizeimage > U32_MAX ||
		    (address + offset) & 15)
			return -EINVAL;
		if (V4L2_TYPE_IS_OUTPUT(vb->type) &&
		    vb2_get_plane_payload(vb, i) < offset + pix->plane_fmt[i].sizeimage)
			return -EINVAL;
		vb2_set_plane_payload(vb, i, offset + pix->plane_fmt[i].sizeimage);
	}
	return 0;
}

static void mdp_linear_buffer_queue(struct vb2_buffer *vb)
{
	struct mdp_linear_ctx *ctx = vb2_get_drv_priv(vb->vb2_queue);

	v4l2_m2m_buf_queue(ctx->fh.m2m_ctx, to_vb2_v4l2_buffer(vb));
}

static int mdp_linear_start_streaming(struct vb2_queue *q, unsigned int count)
{
	struct mdp_linear_ctx *ctx = vb2_get_drv_priv(q);
	bool ten = mdp_linear_tenbit(ctx->source.pixelformat);
	struct vb2_v4l2_buffer *buffer;

	if ((ten ? V4L2_PIX_FMT_P010 : V4L2_PIX_FMT_NV12) == ctx->target.pixelformat &&
	    ctx->target.width >= ctx->crop.width && ctx->target.height >= ctx->crop.height) {
		WRITE_ONCE(ctx->aborting, false);
		ctx->sequence = 0;
		return 0;
	}
	while ((buffer = V4L2_TYPE_IS_OUTPUT(q->type) ?
		v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx) :
		v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx)))
		v4l2_m2m_buf_done(buffer, VB2_BUF_STATE_QUEUED);
	return -EINVAL;
}

static void mdp_linear_stop_streaming(struct vb2_queue *q)
{
	struct mdp_linear_ctx *ctx = vb2_get_drv_priv(q);
	struct vb2_v4l2_buffer *buffer;

	/* The worker never takes q->lock; STREAMOFF waits before releasing DMA-BUFs. */
	WRITE_ONCE(ctx->aborting, true);
	flush_workqueue(ctx->mdp->workqueue);
	while ((buffer = V4L2_TYPE_IS_OUTPUT(q->type) ?
		v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx) :
		v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx)))
		v4l2_m2m_buf_done(buffer, VB2_BUF_STATE_ERROR);
}

static const struct vb2_ops mdp_linear_queue_ops = {
	.queue_setup = mdp_linear_queue_setup,
	.buf_init = mdp_linear_buffer_init,
	.buf_cleanup = mdp_linear_buffer_cleanup,
	.buf_prepare = mdp_linear_buffer_prepare,
	.buf_queue = mdp_linear_buffer_queue,
	.start_streaming = mdp_linear_start_streaming,
	.stop_streaming = mdp_linear_stop_streaming,
};

static int mdp_linear_queue_init(void *priv, struct vb2_queue *source, struct vb2_queue *target)
{
	struct mdp_linear_ctx *ctx = priv;
	struct vb2_queue *q;
	unsigned int i;
	int ret;

	for (i = 0; i < 2; i++) {
		q = i ? target : source;
		q->type = i ? V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE :
			     V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE;
		q->io_modes = VB2_MMAP | VB2_DMABUF;
		q->drv_priv = ctx;
		q->buf_struct_size = sizeof(struct mdp_linear_buffer);
		q->ops = &mdp_linear_queue_ops;
		q->mem_ops = &vb2_dma_contig_memops;
		q->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_COPY;
		q->lock = &ctx->mdp->lock;
		q->dev = ctx->mdp->dev;
		q->max_num_buffers = MDP_LINEAR_MAX_IMPORTED;
		ret = vb2_queue_init(q);
		if (ret)
			return ret;
	}
	return 0;
}

static void mdp_linear_worker(struct work_struct *work)
{
	struct mdp_linear_dev *mdp = container_of(work, struct mdp_linear_dev, work);
	struct mdp_linear_ctx *ctx = v4l2_m2m_get_curr_priv(mdp->m2m);
	struct vb2_v4l2_buffer *source, *target;
	int ret = -ECANCELED;

	if (!ctx)
		return;
	source = v4l2_m2m_next_src_buf(ctx->fh.m2m_ctx);
	target = v4l2_m2m_next_dst_buf(ctx->fh.m2m_ctx);
	if (!READ_ONCE(ctx->aborting))
		ret = mdp_linear_process(ctx, &source->vb2_buf, &target->vb2_buf);
	if (ret)
		dev_err_ratelimited(mdp->dev, "conversion failed: %d\n", ret);
	v4l2_m2m_buf_copy_metadata(source, target);
	source->sequence = ctx->sequence;
	target->sequence = ctx->sequence++;
	v4l2_m2m_src_buf_remove(ctx->fh.m2m_ctx);
	v4l2_m2m_dst_buf_remove(ctx->fh.m2m_ctx);
	v4l2_m2m_buf_done(source, ret ? VB2_BUF_STATE_ERROR : VB2_BUF_STATE_DONE);
	v4l2_m2m_buf_done(target, ret ? VB2_BUF_STATE_ERROR : VB2_BUF_STATE_DONE);
	v4l2_m2m_job_finish(mdp->m2m, ctx->fh.m2m_ctx);
}

static void mdp_linear_device_run(void *priv)
{
	struct mdp_linear_ctx *ctx = priv;

	queue_work(ctx->mdp->workqueue, &ctx->mdp->work);
}

static void mdp_linear_job_abort(void *priv)
{
	struct mdp_linear_ctx *ctx = priv;

	WRITE_ONCE(ctx->aborting, true);
}

static const struct v4l2_m2m_ops mdp_linear_m2m_ops = {
	.device_run = mdp_linear_device_run,
	.job_abort = mdp_linear_job_abort,
};

static int mdp_linear_open(struct file *file)
{
	struct mdp_linear_dev *mdp = video_drvdata(file);
	struct mdp_linear_ctx *ctx;
	struct v4l2_format f = { .type = V4L2_BUF_TYPE_VIDEO_OUTPUT_MPLANE };
	int ret;

	ctx = kzalloc_obj(*ctx);
	if (!ctx)
		return -ENOMEM;
	ctx->mdp = mdp;
	v4l2_fh_init(&ctx->fh, &mdp->video);
	file->private_data = &ctx->fh;
	f.fmt.pix_mp.width = 640;
	f.fmt.pix_mp.height = 480;
	f.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_MM21;
	mdp_linear_try_format(file, &ctx->fh, &f);
	ctx->source = f.fmt.pix_mp;
	f.type = V4L2_BUF_TYPE_VIDEO_CAPTURE_MPLANE;
	f.fmt.pix_mp.pixelformat = V4L2_PIX_FMT_NV12;
	memset(f.fmt.pix_mp.plane_fmt, 0, sizeof(f.fmt.pix_mp.plane_fmt));
	mdp_linear_try_format(file, &ctx->fh, &f);
	ctx->target = f.fmt.pix_mp;
	ctx->crop = (struct v4l2_rect) { .width = 640, .height = 480 };
	mutex_lock(&mdp->lock);
	ctx->fh.m2m_ctx = v4l2_m2m_ctx_init(mdp->m2m, ctx, mdp_linear_queue_init);
	mutex_unlock(&mdp->lock);
	if (IS_ERR(ctx->fh.m2m_ctx)) {
		ret = PTR_ERR(ctx->fh.m2m_ctx);
		v4l2_fh_exit(&ctx->fh);
		kfree(ctx);
		return ret;
	}
	v4l2_fh_add(&ctx->fh, file);
	return 0;
}

static int mdp_linear_release(struct file *file)
{
	struct mdp_linear_ctx *ctx = mdp_linear_context(file->private_data);

	mutex_lock(&ctx->mdp->lock);
	v4l2_m2m_ctx_release(ctx->fh.m2m_ctx);
	mutex_unlock(&ctx->mdp->lock);
	v4l2_fh_del(&ctx->fh, file);
	v4l2_fh_exit(&ctx->fh);
	kfree(ctx);
	return 0;
}

static const struct v4l2_file_operations mdp_linear_fops = {
	.owner = THIS_MODULE,
	.open = mdp_linear_open,
	.release = mdp_linear_release,
	.poll = v4l2_m2m_fop_poll,
	.unlocked_ioctl = video_ioctl2,
	.mmap = v4l2_m2m_fop_mmap,
};

static const struct v4l2_ioctl_ops mdp_linear_ioctl_ops = {
	.vidioc_querycap = mdp_linear_querycap,
	.vidioc_enum_fmt_vid_out = mdp_linear_enum_format,
	.vidioc_enum_fmt_vid_cap = mdp_linear_enum_format,
	.vidioc_g_fmt_vid_out_mplane = mdp_linear_get_format,
	.vidioc_g_fmt_vid_cap_mplane = mdp_linear_get_format,
	.vidioc_try_fmt_vid_out_mplane = mdp_linear_try_format,
	.vidioc_try_fmt_vid_cap_mplane = mdp_linear_try_format,
	.vidioc_s_fmt_vid_out_mplane = mdp_linear_set_format,
	.vidioc_s_fmt_vid_cap_mplane = mdp_linear_set_format,
	.vidioc_g_selection = mdp_linear_get_selection,
	.vidioc_s_selection = mdp_linear_set_selection,
	.vidioc_reqbufs = v4l2_m2m_ioctl_reqbufs,
	.vidioc_create_bufs = v4l2_m2m_ioctl_create_bufs,
	.vidioc_prepare_buf = v4l2_m2m_ioctl_prepare_buf,
	.vidioc_querybuf = v4l2_m2m_ioctl_querybuf,
	.vidioc_qbuf = v4l2_m2m_ioctl_qbuf,
	.vidioc_dqbuf = v4l2_m2m_ioctl_dqbuf,
	.vidioc_expbuf = v4l2_m2m_ioctl_expbuf,
	.vidioc_streamon = v4l2_m2m_ioctl_streamon,
	.vidioc_streamoff = v4l2_m2m_ioctl_streamoff,
};

static struct device *mdp_linear_supplier(struct device *dev, const char *property)
{
	struct device_node *node = of_parse_phandle(dev->of_node, property, 0);
	struct platform_device *pdev;

	if (!node)
		return ERR_PTR(-EINVAL);
	pdev = of_find_device_by_node(node);
	of_node_put(node);
	if (!pdev)
		return ERR_PTR(-EPROBE_DEFER);
	if (!platform_get_drvdata(pdev) ||
	    !device_link_add(dev, &pdev->dev, DL_FLAG_AUTOREMOVE_CONSUMER)) {
		put_device(&pdev->dev);
		return ERR_PTR(-EPROBE_DEFER);
	}
	put_device(&pdev->dev);
	return &pdev->dev;
}

static int mdp_linear_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mdp_linear_dev *mdp;
	struct device *mutex_dev;
	struct cmdq_client_reg reg;
	u32 events[4];
	unsigned int i;
	int ret;

	mdp = devm_kzalloc(dev, sizeof(*mdp), GFP_KERNEL);
	if (!mdp)
		return -ENOMEM;
	mdp->dev = dev;
	mdp->reader = devm_platform_ioremap_resource_byname(pdev, "reader");
	if (IS_ERR(mdp->reader))
		return PTR_ERR(mdp->reader);
	mdp->writer = devm_platform_ioremap_resource_byname(pdev, "writer");
	if (IS_ERR(mdp->writer))
		return PTR_ERR(mdp->writer);
	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return ret;
	mdp->clocks[0].id = "reader";
	mdp->clocks[1].id = "writer";
	ret = devm_clk_bulk_get(dev, ARRAY_SIZE(mdp->clocks), mdp->clocks);
	if (ret)
		return dev_err_probe(dev, ret, "cannot get clocks\n");
	mdp->mmsys = mdp_linear_supplier(dev, "mediatek,mmsys");
	if (IS_ERR(mdp->mmsys))
		return dev_err_probe(dev, PTR_ERR(mdp->mmsys), "MMSYS unavailable\n");
	mutex_dev = mdp_linear_supplier(dev, "mediatek,mutex");
	if (IS_ERR(mutex_dev))
		return dev_err_probe(dev, PTR_ERR(mutex_dev), "mutex unavailable\n");
	ret = of_property_read_u32_array(dev->of_node, "mediatek,gce-events", events, 4);
	if (ret)
		return dev_err_probe(dev, ret, "missing GCE events\n");
	for (i = 0; i < ARRAY_SIZE(events); i++) {
		if (events[i] > CMDQ_MAX_EVENT)
			return -EINVAL;
		mdp->events[i] = events[i];
	}
	ret = cmdq_dev_get_client_reg(dev, &reg, 0);
	if (ret)
		return dev_err_probe(dev, ret, "missing GCE register window\n");
	mdp->subsys = reg.subsys;
	mdp->hw_mutex = mtk_mutex_mdp_get(mutex_dev);
	if (IS_ERR(mdp->hw_mutex))
		return dev_err_probe(dev, PTR_ERR(mdp->hw_mutex),
				     "memory-path mutex unavailable\n");
	mdp->cmdq = cmdq_mbox_create(dev, 0);
	if (IS_ERR(mdp->cmdq)) {
		ret = PTR_ERR(mdp->cmdq);
		goto mutex_put;
	}
	platform_set_drvdata(pdev, mdp);
	init_completion(&mdp->done);
	mutex_init(&mdp->lock);
	mdp->cmdq->client.rx_callback = mdp_linear_cmdq_done;
	mdp->workqueue = alloc_ordered_workqueue(MDP_LINEAR_NAME, WQ_MEM_RECLAIM);
	if (!mdp->workqueue) {
		ret = -ENOMEM;
		goto cmdq_destroy;
	}
	INIT_WORK(&mdp->work, mdp_linear_worker);
	ret = v4l2_device_register(dev, &mdp->v4l2);
	if (ret)
		goto workqueue_destroy;
	mdp->m2m = v4l2_m2m_init(&mdp_linear_m2m_ops);
	if (IS_ERR(mdp->m2m)) {
		ret = PTR_ERR(mdp->m2m);
		goto v4l2_unregister;
	}
	mdp->video = (struct video_device) {
		.fops = &mdp_linear_fops,
		.ioctl_ops = &mdp_linear_ioctl_ops,
		.release = video_device_release_empty,
		.lock = &mdp->lock,
		.v4l2_dev = &mdp->v4l2,
		.vfl_dir = VFL_DIR_M2M,
		.device_caps = V4L2_CAP_VIDEO_M2M_MPLANE | V4L2_CAP_STREAMING,
	};
	strscpy(mdp->video.name, MDP_LINEAR_NAME, sizeof(mdp->video.name));
	video_set_drvdata(&mdp->video, mdp);
	pm_runtime_enable(dev);
	ret = video_register_device(&mdp->video, VFL_TYPE_VIDEO, -1);
	if (ret)
		goto m2m_release;
	return 0;

m2m_release:
	pm_runtime_disable(dev);
	v4l2_m2m_release(mdp->m2m);
v4l2_unregister:
	v4l2_device_unregister(&mdp->v4l2);
workqueue_destroy:
	destroy_workqueue(mdp->workqueue);
cmdq_destroy:
	if (mdp->cmdq)
		cmdq_mbox_destroy(mdp->cmdq);
mutex_put:
	mtk_mutex_put(mdp->hw_mutex);
	return ret;
}

static void mdp_linear_remove(struct platform_device *pdev)
{
	struct mdp_linear_dev *mdp = platform_get_drvdata(pdev);

	video_unregister_device(&mdp->video);
	destroy_workqueue(mdp->workqueue);
	v4l2_m2m_release(mdp->m2m);
	v4l2_device_unregister(&mdp->v4l2);
	if (mdp->cmdq)
		cmdq_mbox_destroy(mdp->cmdq);
	mtk_mutex_put(mdp->hw_mutex);
	pm_runtime_disable(&pdev->dev);
}

static int mdp_linear_suspend(struct device *dev)
{
	struct mdp_linear_dev *mdp = dev_get_drvdata(dev);

	v4l2_m2m_suspend(mdp->m2m);
	return 0;
}

static int mdp_linear_resume(struct device *dev)
{
	struct mdp_linear_dev *mdp = dev_get_drvdata(dev);

	v4l2_m2m_resume(mdp->m2m);
	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(mdp_linear_pm, mdp_linear_suspend, mdp_linear_resume);

static const struct of_device_id mdp_linear_of_match[] = {
	{ .compatible = "mediatek,mt6785-mdp-linear" },
	{ }
};
MODULE_DEVICE_TABLE(of, mdp_linear_of_match);

static struct platform_driver mdp_linear_driver = {
	.probe = mdp_linear_probe,
	.remove = mdp_linear_remove,
	.driver = {
		.name = MDP_LINEAR_NAME,
		.of_match_table = mdp_linear_of_match,
		.pm = pm_sleep_ptr(&mdp_linear_pm),
		.suppress_bind_attrs = true,
	},
};
module_platform_driver(mdp_linear_driver);
MODULE_DESCRIPTION("MediaTek memory-to-memory video detiler");
MODULE_LICENSE("GPL");
