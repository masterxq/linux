// SPDX-License-Identifier: GPL-2.0-only
/* MT6785 memory reader -> stream switch -> linear writer, without firmware. */
#include <linux/dma-mapping.h>
#include <linux/dma-buf.h>
#include <linux/dma-resv.h>
#include <linux/iopoll.h>
#include <linux/mailbox_client.h>
#include <linux/mailbox_controller.h>
#include <linux/pm_runtime.h>
#include "mdp-linear.h"

#define READER_RUN		0x000
#define READER_RESET		0x008
#define READER_LAYOUT		0x020
#define READER_BUS_CONTROL	0x028
#define READER_PIXEL_CONTROL	0x030
#define READER_FORMAT_EXTENSION	0x038
#define READER_LUMA_BLOCK_PITCH	0x060
#define READER_READ_SIZE		0x070
#define READER_VISIBLE_SIZE	0x078
#define READER_CLIP_ORIGIN	0x080
#define READER_CHROMA_BLOCK_PITCH 0x090
#define READER_PLANE_END(n)	(0x100 + (n) * 8)
#define READER_PLANE_OFFSET(n)	(0x118 + (n) * 8)
#define READER_COLOR_CONTROL	0x200
#define READER_BUFFER_CONTROL0	0x240
#define READER_BUFFER_HIGH0	0x248
#define READER_BUFFER_LOW0	0x250
#define READER_BUFFER_CONTROL1	0x258
#define READER_BUFFER_HIGH1	0x260
#define READER_BUFFER_LOW1	0x268
#define READER_BUFFER_HIGH3 0x290
#define READER_PRECISION	0x2a0
#define READER_RESET_STATE	0x408
#define READER_PLANE_BASE(n)	(0xf00 + (n) * 8)

#define WRITER_PIXEL_CONTROL	0x000
#define WRITER_LINE_BUFFER	0x008
#define WRITER_RESET		0x010
#define WRITER_RESET_STATE	0x014
#define WRITER_INTERRUPT	0x01c
#define WRITER_CLIP_ORIGIN	0x020
#define WRITER_VISIBLE_SIZE	0x024
#define WRITER_LUMA_OFFSET	0x02c
#define WRITER_LUMA_PITCH	0x030
#define WRITER_CHROMA_OFFSET	0x038
#define WRITER_CHROMA_PITCH	0x03c
#define WRITER_DITHER		0x054
#define WRITER_AUX_OFFSET	0x068
#define WRITER_AUX_PITCH		0x06c
#define WRITER_BUS_CONTROL	0x070
#define WRITER_WATERMARK	0x074
#define WRITER_INPUT_SIZE	0x078
#define WRITER_RUN		0x07c
#define WRITER_FIFO		0x080
#define WRITER_COLOR_CONTROL	0x084
#define WRITER_SAMPLE_STORAGE	0x0dc
#define WRITER_COMPRESSION	0x0e0
#define WRITER_PLANE_BASE(n)	(0xf00 + (n) * 4)

/* Register windows belong to subsystem 0x1400; no user-supplied addresses. */
#define READER_WINDOW 0x1000
#define WRITER_WINDOW 0x5000
#define MDP_LINEAR_STRIPE_WIDTH 640U
#define MDP_LINEAR_TIMEOUT_MS 500

struct mdp_linear_commands {
	struct mdp_linear_dev *mdp;
	struct cmdq_pkt packet;
	int error;
};

static void mdp_linear_write(struct mdp_linear_commands *cmd, u16 window,
			     u16 reg, u32 value, u32 mask)
{
	if (!cmd->error)
		cmd->error = cmdq_pkt_write_mask(&cmd->packet, cmd->mdp->subsys,
						 window + reg, value, mask);
}

static void mdp_linear_event(struct mdp_linear_commands *cmd, unsigned int index,
			     bool wait)
{
	if (cmd->error)
		return;
	if (wait)
		cmd->error = cmdq_pkt_wfe(&cmd->packet, cmd->mdp->events[index], true);
	else
		cmd->error = cmdq_pkt_clear_event(&cmd->packet, cmd->mdp->events[index]);
}

static int mdp_linear_reset(struct mdp_linear_dev *mdp)
{
	u32 value;
	int ret;

	writel(0, mdp->reader + READER_RUN);
	writel(0, mdp->writer + WRITER_RUN);
	writel(1, mdp->reader + READER_RESET);
	ret = readl_poll_timeout(mdp->reader + READER_RESET_STATE, value,
				 (value & GENMASK(18, 8)) == BIT(8), 10, 10000);
	writel(0, mdp->reader + READER_RESET);
	if (ret) {
		dev_err_ratelimited(mdp->dev, "reader reset timeout: %#x\n", value);
		return ret;
	}
	writel(1, mdp->writer + WRITER_RESET);
	ret = readl_poll_timeout(mdp->writer + WRITER_RESET_STATE, value,
				 value & BIT(0), 10, 10000);
	writel(0, mdp->writer + WRITER_RESET);
	if (ret) {
		dev_err_ratelimited(mdp->dev, "writer reset timeout: %#x\n", value);
		return ret;
	}
	return readl_poll_timeout(mdp->writer + WRITER_RESET_STATE, value,
				 !(value & BIT(0)), 10, 10000);
}

void mdp_linear_cmdq_done(struct mbox_client *client, void *message)
{
	struct cmdq_cb_data *result = message;
	struct cmdq_client *cmdq = container_of(client, struct cmdq_client, client);
	struct mdp_linear_dev *mdp = dev_get_drvdata(client->dev);

	if (cmdq != mdp->cmdq)
		return;
	WRITE_ONCE(mdp->job_error, result ? result->sta : -EIO);
	complete(&mdp->done);
}

static void mdp_linear_frame(struct mdp_linear_commands *cmd,
			     struct mdp_linear_ctx *ctx,
			     struct vb2_buffer *source, struct vb2_buffer *target)
{
	struct v4l2_pix_format_mplane *src = &ctx->source;
	struct v4l2_pix_format_mplane *dst = &ctx->target;
	bool ten = mdp_linear_tenbit(src->pixelformat);
	u32 pitch = src->plane_fmt[0].bytesperline;
	u32 target_pitch = dst->plane_fmt[0].bytesperline;
	dma_addr_t input[2], output;
	u32 pixel = 0xa860c;
	unsigned int i;

	for (i = 0; i < 2; i++)
		input[i] = vb2_dma_contig_plane_dma_addr(source, i) +
			   source->planes[i].data_offset;
	output = vb2_dma_contig_plane_dma_addr(target, 0);
	if (ten)
		pixel |= BIT(22);
	if (src->pixelformat == V4L2_PIX_FMT_MT2110T)
		pixel |= BIT(23);

#define R(reg, value, mask) mdp_linear_write(cmd, READER_WINDOW, reg, value, mask)
#define W(reg, value, mask) mdp_linear_write(cmd, WRITER_WINDOW, reg, value, mask)
	R(READER_PRECISION, ten ? 7 : 0, 7);
	R(READER_BUS_CONTROL, 0x10371, 0x30071);
	R(READER_PIXEL_CONTROL, pixel, 0x03c8fe0f);
	R(READER_FORMAT_EXTENSION, 0, GENMASK(31, 28));
	R(READER_LAYOUT, ten ? 0x30 : 0x10, U32_MAX);
	for (i = 0; i < 3; i++) {
		R(READER_PLANE_BASE(i), i < 2 ? input[i] : 0, U32_MAX);
		R(READER_PLANE_END(i), i < 2 ?
		  input[i] + src->plane_fmt[i].sizeimage : 0, U32_MAX);
	}
	R(READER_LUMA_BLOCK_PITCH, pitch * 32, U32_MAX);
	R(READER_CHROMA_BLOCK_PITCH, pitch * 16, U32_MAX);
	R(READER_COLOR_CONTROL, ten ? 0x0f010000 : 0, 0x0f110000);
	/* FIFO capacity and urgency limits, independent of image content. */
	R(READER_BUFFER_CONTROL0, 0x03000020, U32_MAX);
	R(READER_BUFFER_HIGH0, 0x3a002700, U32_MAX);
	R(READER_BUFFER_LOW0, 0x01000010, U32_MAX);
	R(READER_BUFFER_CONTROL1, 0x1d001300, U32_MAX);
	R(READER_BUFFER_HIGH1, 0, U32_MAX);
	R(READER_BUFFER_LOW1, 0, U32_MAX);
	R(READER_BUFFER_HIGH3, 0, U32_MAX);
	W(WRITER_PLANE_BASE(0), output, U32_MAX);
	W(WRITER_PLANE_BASE(1), output + target_pitch * dst->height, U32_MAX);
	W(WRITER_PLANE_BASE(2), 0, U32_MAX);
	W(WRITER_PIXEL_CONTROL, 0x500c, 0xf131510f);
	W(WRITER_SAMPLE_STORAGE, ten ? 1 : 0, U32_MAX);
	W(WRITER_COMPRESSION, 0, BIT(26));
	W(WRITER_WATERMARK, 0x000d80c4, U32_MAX);
	W(WRITER_LUMA_PITCH, target_pitch, U32_MAX);
	W(WRITER_CHROMA_PITCH, target_pitch, U32_MAX);
	W(WRITER_AUX_PITCH, target_pitch, U32_MAX);
	W(WRITER_COLOR_CONTROL, 0, U32_MAX);
	W(WRITER_DITHER, 0xff000000, 0xff000000);
	W(WRITER_BUS_CONTROL, BIT(31), BIT(31));
	W(WRITER_FIFO, 0x100, U32_MAX);
	W(WRITER_LINE_BUFFER, 0, 0x77);
	W(WRITER_RUN, 0x900000, 0x900000);
}

static void mdp_linear_stripes(struct mdp_linear_commands *cmd,
			       struct mdp_linear_ctx *ctx)
{
	bool ten = mdp_linear_tenbit(ctx->source.pixelformat);
	u32 x, width = ctx->crop.width, height = ctx->crop.height;
	u32 bytes = ten ? 2 : 1;

	for (x = 0; x < width; x += MDP_LINEAR_STRIPE_WIDTH) {
		u32 stripe = min(width - x, MDP_LINEAR_STRIPE_WIDTH);
		u32 aligned = ALIGN(stripe, 16);
		u32 read_size = ALIGN(height, 32) << 16 | aligned;
		u32 visible = height << 16 | stripe;
		/* Keep at least three writer lines, bounded by its FIFO capacity. */
		u32 lines = min(4U, 2048U / aligned);
		u32 luma_offset = x * 32 * (ten ? 5 : 4) / 4;

		W(WRITER_LUMA_OFFSET, x * bytes, U32_MAX);
		W(WRITER_CHROMA_OFFSET, x * bytes, U32_MAX);
		W(WRITER_AUX_OFFSET, x * bytes, U32_MAX);
		W(WRITER_INPUT_SIZE, visible, U32_MAX);
		W(WRITER_VISIBLE_SIZE, visible, U32_MAX);
		W(WRITER_CLIP_ORIGIN, 0, U32_MAX);
		W(WRITER_LINE_BUFFER, aligned << 16 | lines << 12, 0xffff7f00);
		W(WRITER_INTERRUPT, 1, U32_MAX);
		W(WRITER_RUN, 1, U32_MAX);
		R(READER_RUN, 1, BIT(0));
		R(READER_PLANE_OFFSET(0), luma_offset, U32_MAX);
		R(READER_PLANE_OFFSET(1), luma_offset / 2, U32_MAX);
		R(READER_PLANE_OFFSET(2), luma_offset / 2, U32_MAX);
		R(READER_READ_SIZE, read_size, U32_MAX);
		R(READER_VISIBLE_SIZE, visible, U32_MAX);
		R(READER_CLIP_ORIGIN, 0, U32_MAX);
		mdp_linear_event(cmd, 0, false);
		mdp_linear_event(cmd, 1, false);
		mdp_linear_event(cmd, 2, false);
		mdp_linear_event(cmd, 3, false);
		if (!cmd->error)
			cmd->error = mtk_mutex_enable_by_cmdq(cmd->mdp->hw_mutex, &cmd->packet);
		mdp_linear_event(cmd, 0, true);
		mdp_linear_event(cmd, 1, true);
		mdp_linear_event(cmd, 2, true);
		R(READER_RUN, 0, BIT(0));
		mdp_linear_event(cmd, 3, true);
		W(WRITER_RUN, 0, U32_MAX);
	}
#undef R
#undef W
}

static int mdp_linear_wait_buffers(struct vb2_buffer *source, struct vb2_buffer *target)
{
	unsigned long deadline = jiffies + msecs_to_jiffies(MDP_LINEAR_TIMEOUT_MS);
	unsigned int i, p;

	for (i = 0; i < source->num_planes; i++)
		if (source->planes[i].dbuf && source->planes[i].dbuf == target->planes[0].dbuf)
			return -EINVAL;
	for (i = 0; i < 2; i++) {
		struct vb2_buffer *buffer = i ? target : source;

		for (p = 0; p < buffer->num_planes; p++) {
			struct dma_buf *dmabuf = buffer->planes[p].dbuf;
			long ret;

			if (!dmabuf)
				continue;
			if (time_after_eq(jiffies, deadline))
				return -ETIMEDOUT;
			ret = dma_resv_wait_timeout(dmabuf->resv,
						    i ? DMA_RESV_USAGE_READ : DMA_RESV_USAGE_WRITE,
						   true, deadline - jiffies);
			if (ret <= 0)
				return ret ? ret : -ETIMEDOUT;
		}
	}
	return 0;
}

int mdp_linear_process(struct mdp_linear_ctx *ctx, struct vb2_buffer *source,
		       struct vb2_buffer *target)
{
	struct mdp_linear_dev *mdp = ctx->mdp;
	struct mdp_linear_commands cmd = { .mdp = mdp };
	struct device *gce;
	int ret;

	if (!mdp->cmdq)
		return -ENODEV;
	ret = mdp_linear_wait_buffers(source, target);
	if (ret)
		return ret;
	ret = pm_runtime_resume_and_get(mdp->dev);
	if (ret < 0)
		return ret;
	gce = mdp->cmdq->chan->mbox->dev;
	ret = pm_runtime_resume_and_get(gce);
	if (ret < 0)
		goto power_off;
	ret = clk_bulk_prepare_enable(ARRAY_SIZE(mdp->clocks), mdp->clocks);
	if (ret)
		goto gce_off;
	ret = mtk_mutex_prepare(mdp->hw_mutex);
	if (ret)
		goto clocks_off;
	ret = mdp_linear_reset(mdp);
	if (ret)
		goto mutex_off;
	ret = mtk_mmsys_mdp_linear_route(mdp->mmsys, true);
	if (ret)
		goto mutex_off;
	ret = mtk_mutex_mdp_config(mdp->hw_mutex, true);
	if (ret)
		goto route_off;
	ret = cmdq_pkt_create(mdp->cmdq, &cmd.packet, SZ_16K);
	if (ret)
		goto route_off;
	mdp_linear_frame(&cmd, ctx, source, target);
	mdp_linear_stripes(&cmd, ctx);
	if (!cmd.error)
		cmd.error = cmdq_pkt_eoc(&cmd.packet);
	if (!cmd.error)
		cmd.error = cmdq_pkt_jump_rel(&cmd.packet, CMDQ_INST_SIZE,
					      cmd.packet.priv.shift_pa);
	ret = cmd.error;
	if (ret)
		goto packet_free;
	reinit_completion(&mdp->done);
	WRITE_ONCE(mdp->job_error, 0);
	/* The packet allocation is DMA streaming memory owned by the GCE device. */
	dma_sync_single_for_device(mdp->cmdq->chan->mbox->dev, cmd.packet.pa_base,
				   cmd.packet.cmd_buf_size, DMA_TO_DEVICE);
	ret = mbox_send_message(mdp->cmdq->chan, &cmd.packet);
	if (ret < 0)
		goto packet_free;
	mbox_client_txdone(mdp->cmdq->chan, 0);
	if (!wait_for_completion_timeout(&mdp->done, msecs_to_jiffies(MDP_LINEAR_TIMEOUT_MS))) {
		/* Stop GCE before any packet or imported image can be released. */
		if (mbox_flush(mdp->cmdq->chan, 1000)) {
			cmdq_mbox_destroy(mdp->cmdq);
			mdp->cmdq = NULL;
			dma_unmap_single(gce, cmd.packet.pa_base, cmd.packet.buf_size,
					 DMA_TO_DEVICE);
			kfree(cmd.packet.va_base);
			mdp_linear_reset(mdp);
			ret = -EIO;
			goto route_off;
		}
		ret = -ETIMEDOUT;
	} else {
		ret = READ_ONCE(mdp->job_error);
	}
	if (ret)
		mdp_linear_reset(mdp);
packet_free:
	cmdq_pkt_destroy(mdp->cmdq, &cmd.packet);
route_off:
	mtk_mutex_disable(mdp->hw_mutex);
	mtk_mutex_mdp_config(mdp->hw_mutex, false);
	mtk_mmsys_mdp_linear_route(mdp->mmsys, false);
mutex_off:
	mtk_mutex_unprepare(mdp->hw_mutex);
clocks_off:
	clk_bulk_disable_unprepare(ARRAY_SIZE(mdp->clocks), mdp->clocks);
gce_off:
	pm_runtime_mark_last_busy(gce);
	pm_runtime_put_autosuspend(gce);
power_off:
	pm_runtime_put(mdp->dev);
	return ret;
}
