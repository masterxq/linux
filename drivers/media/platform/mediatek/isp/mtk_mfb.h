/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * MediaTek MT6785 multi-frame blending block (MFB): driver state.
 */

#ifndef _MTK_MFB_H
#define _MTK_MFB_H

#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/workqueue.h>
#include <media/media-device.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>
#include <media/videobuf2-v4l2.h>

#include <linux/media/mediatek/mtk-mfb-mt6785.h>

#include "mtk_mfb_job.h"

/* Video nodes */
enum mtk_mfb_node_id {
	MTK_MFB_NODE_BASE,
	MTK_MFB_NODE_REF,
	MTK_MFB_NODE_WT_IN,
	MTK_MFB_NODE_PARAMS,
	MTK_MFB_NODE_OUT,
	MTK_MFB_NODE_WT_OUT,
	MTK_MFB_NODE_NUM,
};

/**
 * struct mtk_mfb_buffer - buffer of a video node
 * @vb: vb2 buffer
 * @list: entry in the ready list of its node
 * @params: kernel copy of a params buffer (params node only), checked
 */
struct mtk_mfb_buffer {
	struct vb2_v4l2_buffer vb;
	struct list_head list;
	u8 params[MTK_MFB_PARAMS_MAX_SIZE] __aligned(8);
};

/**
 * struct mtk_mfb_node - video node
 * @mfb: the device
 * @id: enum mtk_mfb_node_id
 * @vdev: video device
 * @pad: pad of the video device entity
 * @queue: vb2 queue
 * @lock: serializes the ioctls of the node, vb2 queue lock
 * @ready: queued buffers not yet in a job, protected by mtk_mfb.lock
 * @fmt: format
 */
struct mtk_mfb_node {
	struct mtk_mfb *mfb;
	enum mtk_mfb_node_id id;
	struct video_device vdev;
	struct media_pad pad;
	struct vb2_queue queue;
	struct mutex lock; /* ioctls of the node */
	struct list_head ready;
	struct v4l2_format fmt;
};

/**
 * struct mtk_mfb_aux - coherent buffer of the driver with a guard behind it
 * @cpu: CPU address
 * @dma: bus address
 * @size: usable size, the guard follows
 */
struct mtk_mfb_aux {
	void *cpu;
	dma_addr_t dma;
	size_t size;
};

/**
 * struct mtk_mfb - MFB device
 * @dev: the platform device
 * @regs: MFB register window
 * @clk: clock of the block
 * @irq: interrupt of the block, enabled only while the block is powered
 * @v4l2_dev: V4L2 device
 * @mdev: media device
 * @sd: the processing entity
 * @sd_pads: pads of @sd, one per node
 * @node: the video nodes
 * @lock: streaming map, ready lists, interrupt status
 * @streaming: BIT(node id) of the nodes that stream
 * @params_link: the link of the params node is enabled
 * @sequence: sequence number of the next job
 * @wq: runs the jobs one after the other
 * @work: the job runner
 * @done: the block finished the frame
 * @irq_status: INT_STATUS bits since the start of the job
 * @run_lock: held by the job runner for a job, and for freeing its buffers
 * @tdr: tile descriptions of the current geometry
 * @layout: tiles of the current geometry
 * @geom: geometry of @layout and @tdr
 * @layout_ok: @layout and @tdr match @geom
 * @tuning: tuning words of the last job, zero after the probe
 *
 * Freed with the last open file of its video nodes, see mtk_mfb_alloc().
 */
struct mtk_mfb {
	struct device *dev;
	void __iomem *regs;
	struct clk *clk;
	int irq;

	struct v4l2_device v4l2_dev;
	struct media_device mdev;
	struct v4l2_subdev sd;
	struct media_pad sd_pads[MTK_MFB_NODE_NUM];
	struct mtk_mfb_node node[MTK_MFB_NODE_NUM];

	spinlock_t lock; /* streaming map, ready lists, interrupt status */
	unsigned int streaming;
	bool params_link;
	u32 sequence;
	struct workqueue_struct *wq;
	struct work_struct work;
	struct completion done;
	u32 irq_status;

	struct mutex run_lock; /* job runner and its buffers */
	struct mtk_mfb_aux tdr;
	struct mtk_mfb_layout layout;
	struct mtk_mfb_geom geom;
	bool layout_ok;
	struct mtk_mfb_tuning tuning;
};

/* mtk_mfb_hw.c */
void mtk_mfb_kick(struct mtk_mfb *mfb);
void mtk_mfb_aux_free_all(struct mtk_mfb *mfb);

/* mtk_mfb_v4l2.c */
struct mtk_mfb *mtk_mfb_alloc(struct device *dev);
int mtk_mfb_v4l2_register(struct mtk_mfb *mfb);
void mtk_mfb_v4l2_unregister(struct mtk_mfb *mfb);
int mtk_mfb_job_geom(struct mtk_mfb *mfb, bool wt_in, struct mtk_mfb_geom *g);

/* mtk_mfb_params.c */
int mtk_mfb_params_check(struct mtk_mfb *mfb, struct vb2_buffer *vb,
			 struct mtk_mfb_buffer *buf);
void mtk_mfb_params_apply(struct mtk_mfb_tuning *t,
			  const struct mtk_mfb_buffer *buf);

#endif /* _MTK_MFB_H */
