/* SPDX-License-Identifier: GPL-2.0-only */
#ifndef __MTK_MDP_LINEAR_H
#define __MTK_MDP_LINEAR_H

#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/platform_device.h>
#include <linux/sizes.h>
#include <linux/soc/mediatek/mtk-cmdq.h>
#include <linux/soc/mediatek/mtk-mmsys.h>
#include <linux/soc/mediatek/mtk-mutex.h>
#include <media/v4l2-device.h>
#include <media/v4l2-mem2mem.h>
#include <media/videobuf2-dma-contig.h>

#define MDP_LINEAR_NAME "mtk-mdp-linear"
#define MDP_LINEAR_MAX_WIDTH 3840U
#define MDP_LINEAR_MAX_HEIGHT 2160U
#define MDP_LINEAR_MAX_BUFFERS 8U
/* Imported buffers keep their attachment per index; allow one per decoder buffer. */
#define MDP_LINEAR_MAX_IMPORTED 64U
#define MDP_LINEAR_MEMORY_LIMIT SZ_256M

struct mdp_linear_dev {
	struct device *dev;
	struct device *mmsys;
	struct mtk_mutex *hw_mutex;
	struct clk_bulk_data clocks[2];
	void __iomem *reader;
	void __iomem *writer;
	struct cmdq_client *cmdq;
	struct completion done;
	int job_error;
	u16 events[4];
	u8 subsys;
	struct workqueue_struct *workqueue;
	struct work_struct work;
	struct mutex lock; /* Serializes file operations and queue configuration. */
	struct v4l2_device v4l2;
	struct video_device video;
	struct v4l2_m2m_dev *m2m;
	atomic_long_t queued_bytes;
};

struct mdp_linear_ctx {
	struct v4l2_fh fh;
	struct mdp_linear_dev *mdp;
	struct v4l2_pix_format_mplane source;
	struct v4l2_pix_format_mplane target;
	struct v4l2_rect crop;
	bool aborting;
	u32 sequence;
};

static inline bool mdp_linear_tenbit(u32 format)
{
	return format == V4L2_PIX_FMT_MT2110T || format == V4L2_PIX_FMT_MT2110R;
}

int mdp_linear_process(struct mdp_linear_ctx *ctx, struct vb2_buffer *source,
		       struct vb2_buffer *target);
void mdp_linear_cmdq_done(struct mbox_client *client, void *message);

#endif
