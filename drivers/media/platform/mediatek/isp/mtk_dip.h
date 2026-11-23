/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * MediaTek MT6785 ISP pass 2 (DIP): driver state.
 */

#ifndef _MTK_DIP_H
#define _MTK_DIP_H

#include <linux/clk.h>
#include <linux/completion.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/notifier.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <media/media-device.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>
#include <media/videobuf2-v4l2.h>

#include <linux/media/mediatek/mtk-dip-mt6785.h>

#include "mtk_dip_tile.h"

/* Video nodes */
enum mtk_dip_node_id {
	MTK_DIP_NODE_RAW,
	MTK_DIP_NODE_PARAMS,
	MTK_DIP_NODE_MAIN,
	MTK_DIP_NODE_SCALED,
	MTK_DIP_NODE_BASE,
	MTK_DIP_NODE_WEIGHT,
	MTK_DIP_NODE_NUM,
};

/* command queue thread that starts the tiles */
#define MTK_DIP_CQ_THREAD		1

/* sizes the driver accepts */
#define MTK_DIP_MIN_W			128
#define MTK_DIP_MIN_H			64
#define MTK_DIP_MAX_W			4624
#define MTK_DIP_MAX_H			3472
/* scaled output: 1/2 to 1/16 of the processed area on each axis */
#define MTK_DIP_SCALED_MIN_W		32
#define MTK_DIP_SCALED_MIN_H		16
#define MTK_DIP_SCALED_MAX_W		(MTK_DIP_MAX_W / 2)
#define MTK_DIP_SCALED_MAX_H		(MTK_DIP_MAX_H / 2)

/**
 * struct mtk_dip_buffer - buffer of a video node
 * @vb: vb2 buffer
 * @list: entry in the ready list of its node
 * @params: kernel copy of a params buffer (params node only), checked
 */
struct mtk_dip_buffer {
	struct vb2_v4l2_buffer vb;
	struct list_head list;
	u8 params[MTK_DIP_PARAMS_MAX_SIZE] __aligned(8);
};

/**
 * struct mtk_dip_node - video node
 * @dip: the device
 * @id: enum mtk_dip_node_id
 * @vdev: video device
 * @pad: pad of the video device entity
 * @queue: vb2 queue
 * @lock: serializes the ioctls of the node, vb2 queue lock
 * @ready: queued buffers not yet in a job, protected by mtk_dip.lock
 * @fmt: format
 * @crop: processed area of the input frame (raw node only)
 */
struct mtk_dip_node {
	struct mtk_dip *dip;
	enum mtk_dip_node_id id;
	struct video_device vdev;
	struct media_pad pad;
	struct vb2_queue queue;
	struct mutex lock; /* ioctls of the node */
	struct list_head ready;
	struct v4l2_format fmt;
	struct v4l2_rect crop;
};

/**
 * struct mtk_dip_aux - coherent buffer of the driver with a guard behind it
 * @cpu: CPU address
 * @dma: bus address
 * @size: usable size, the guard follows
 */
struct mtk_dip_aux {
	void *cpu;
	dma_addr_t dma;
	size_t size;
};

/**
 * struct mtk_dip_addrs - bus addresses of a job
 * @in: first processed input line (Y plane of a YUV input)
 * @in_uv: offset of the UV plane of a YUV input behind @in
 * @out: Y plane of the main output (if on), a driver buffer when a job with
 *	temporal noise reduction has no main output
 * @out_uv: offset of the UV plane behind @out
 * @scaled: scaled output (if on)
 * @ref: Y plane of the reference of the temporal noise reduction (if on)
 * @ref_uv: offset of its UV plane behind @ref
 * @wt: weight map of the blend (if on); @ref and @ref_uv are the base image
 */
struct mtk_dip_addrs {
	dma_addr_t in;
	u32 in_uv;
	dma_addr_t out;
	u32 out_uv;
	dma_addr_t scaled;
	dma_addr_t ref;
	u32 ref_uv;
	dma_addr_t wt;
};

/* register words of the image quality modules */
#define MTK_DIP_EE_WORDS		30
#define MTK_DIP_YCNR_WORDS		48
#define MTK_DIP_DEMOSAIC_WORDS		20
#define MTK_DIP_NBC2_WORDS		18
#define MTK_DIP_NR3D_WORDS		22
#define MTK_DIP_RADIAL_WORDS		12
#define MTK_DIP_RNR_WORDS		16
/* cycle lengths of the four counters of the pattern part: word, fields */
#define MTK_DIP_NR3D_CYCLE_WORD		20
#define MTK_DIP_NR3D_CYCLE_SHIFT(i)	(20 - 6 * (i))
#define MTK_DIP_NR3D_CYCLE_MASK		0x3f

/**
 * struct mtk_dip_param_state - translated parameters of the last job
 * @blc: black level offsets and gains, register order, for BGGR: Bayer
 *	position k of the params buffer as colour k; mtk_dip_params_bayer()
 *	gives the words for the phase of a job
 * @wb: white balance gain words, for BGGR like @blc
 * @ccm: colour matrix words
 * @gamma: gamma points
 * @gamma_ctl: gamma control (bypass or curve)
 * @lsc: lens shading enabled
 * @lsc_new: @lsc_gain changed since the shading table was last computed
 * @lsc_gain: gains at the corners of the shading grid
 * @ee: edge enhancement enabled
 * @ycnr: own settings of the first noise reduction stage, else neutral
 * @demosaic: own settings of the demosaicing, else neutral
 * @nbc2: own settings of the second noise reduction stage, else neutral
 * @radial: own radial weights, else neutral
 * @rnr: own settings of the raw noise reduction, else neutral
 * @ee_w: register words of the edge enhancement
 * @ycnr_w: register words of the first noise reduction stage
 * @demosaic_w: register words of the demosaicing
 * @nbc2_w: register words of the second noise reduction stage
 * @radial_w: register words of each radial weight module
 * @rnr_w: register words of the raw noise reduction
 * @nr3d: temporal noise reduction enabled
 * @nr3d_restart: the job of the last params buffer starts a new sequence
 * @nr3d_dx: global motion, horizontal
 * @nr3d_dy: global motion, vertical
 * @nr3d_w: register words of the temporal noise reduction
 * @mix: the blend is on (jobs with a base image and a weight map)
 * @mix_w: the two control words of the blend
 */
struct mtk_dip_param_state {
	u32 blc[8];
	u32 wb[2];
	u32 ccm[6];
	u32 gamma[MTK_DIP_GAMMA_POINTS];
	u32 gamma_ctl;
	bool lsc;
	bool lsc_new;
	u16 lsc_gain[MTK_DIP_BAYER_CHANNELS][MTK_DIP_LSC_POINTS]
		    [MTK_DIP_LSC_POINTS];
	bool ee;
	bool ycnr;
	bool demosaic;
	bool nbc2;
	bool radial;
	bool rnr;
	u32 ee_w[MTK_DIP_EE_WORDS];
	u32 ycnr_w[MTK_DIP_YCNR_WORDS];
	u32 demosaic_w[MTK_DIP_DEMOSAIC_WORDS];
	u32 nbc2_w[MTK_DIP_NBC2_WORDS];
	u32 radial_w[MTK_DIP_RADIAL_WORDS];
	u32 rnr_w[MTK_DIP_RNR_WORDS];
	bool nr3d;
	bool nr3d_restart;
	s16 nr3d_dx;
	s16 nr3d_dy;
	u32 nr3d_w[MTK_DIP_NR3D_WORDS];
	bool mix;
	u32 mix_w[2];
};

/* reference of the temporal noise reduction */
enum mtk_dip_ref_src {
	MTK_DIP_REF_NONE,
	MTK_DIP_REF_OWN,	/* a buffer of the driver */
	MTK_DIP_REF_MAIN,	/* a buffer of the main node, held back */
};

/**
 * struct mtk_dip_nr3d - state of the temporal noise reduction
 * @buf: the two output buffers of the driver for jobs without main output
 * @src: where the reference is
 * @own: index into @buf of the reference (MTK_DIP_REF_OWN)
 * @held: main buffer that is the reference (MTK_DIP_REF_MAIN), not returned
 *	to user space yet
 * @ref: bus address of the Y plane of the reference
 * @ref_uv: offset of its UV plane
 * @w: width of the reference
 * @h: height of the reference
 * @stride: bytes per line of the reference
 * @size: bytes of the buffer of the reference
 * @sleeps: value of mtk_dip.sleeps when the reference was written
 * @hold_until: @held goes back to user space at this time without a job
 * @count: four counters stepped once per job, written into the module
 * @starved: the memory for the module ran out; it stays off until the next
 *	raw stream or until it is switched off and on again
 */
struct mtk_dip_nr3d {
	struct mtk_dip_aux buf[2];
	enum mtk_dip_ref_src src;
	unsigned int own;
	struct mtk_dip_buffer *held;
	dma_addr_t ref;
	u32 ref_uv;
	u32 w;
	u32 h;
	u32 stride;
	size_t size;
	u32 sleeps;
	unsigned long hold_until;
	u8 count[4];
	bool starved;
};

/**
 * struct mtk_dip - DIP device
 * @dev: the platform device
 * @regs: DIP register window
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
 * @tile_done: the command queue thread finished a tile
 * @irq_cq: CQ_INT_STATUS bits since the last tile start
 * @irq_int: INT_STATUS bits since the last tile start
 * @run_lock: held by the job runner for a job, and for freeing its buffers
 * @reset_pending: a job failed, and the ISP domain has not been off since
 * @reset_wait: woken when the ISP domain goes off
 * @pd_nb: power notifier of the ISP domain
 * @ctrl: command queue descriptors, register image and tile descriptions
 * @stripe: stripe store between the tiles; with the scaled output the
 *	second stripe store behind the first, at @stripe2_off
 * @stripe2_off: offset of the second stripe store in @stripe
 * @lsc: shading table, while lens shading is in use
 * @lsc_ok: @lsc holds the table of @par and @frame
 * @layout: tiles of the current geometry
 * @geom: geometry of @layout
 * @frame: frame grids of @geom
 * @layout_ok: @layout, @geom and the register image match
 * @par: parameters of the last job, neutral after the probe
 * @nr3d: temporal noise reduction, protected by @run_lock
 * @nr3d_hold: returns a held main buffer when no job follows in time
 * @sleeps: system sleeps since the probe
 *
 * Freed with the last open file of its video nodes, see mtk_dip_alloc().
 */
struct mtk_dip {
	struct device *dev;
	void __iomem *regs;
	struct clk *clk;
	int irq;

	struct v4l2_device v4l2_dev;
	struct media_device mdev;
	struct v4l2_subdev sd;
	struct media_pad sd_pads[MTK_DIP_NODE_NUM];
	struct mtk_dip_node node[MTK_DIP_NODE_NUM];

	spinlock_t lock; /* streaming map, ready lists, interrupt status */
	unsigned int streaming;
	bool params_link;
	u32 sequence;
	struct workqueue_struct *wq;
	struct work_struct work;
	struct completion tile_done;
	u32 irq_cq;
	u32 irq_int;

	struct mutex run_lock; /* job runner and its buffers */
	bool reset_pending;
	wait_queue_head_t reset_wait;
	struct notifier_block pd_nb;
	struct mtk_dip_aux ctrl;
	struct mtk_dip_aux stripe;
	u32 stripe2_off;
	struct mtk_dip_aux lsc;
	bool lsc_ok;
	struct mtk_dip_tile_layout *layout;
	struct mtk_dip_tile_geom geom;
	struct mtk_dip_frame_cfg frame;
	bool layout_ok;
	struct mtk_dip_param_state par;
	struct mtk_dip_nr3d nr3d;
	struct delayed_work nr3d_hold;
	u32 sleeps;
};

/* mtk_dip_hw.c */
void mtk_dip_kick(struct mtk_dip *dip);
void mtk_dip_aux_free_all(struct mtk_dip *dip);
void mtk_dip_nr3d_drop(struct mtk_dip *dip);
void mtk_dip_nr3d_restart(struct mtk_dip *dip);
int mtk_dip_job_geometry(const struct v4l2_pix_format_mplane *in,
			 const struct v4l2_rect *crop,
			 const struct v4l2_pix_format_mplane *out,
			 const struct v4l2_pix_format_mplane *sc,
			 const struct v4l2_pix_format_mplane *base,
			 const struct v4l2_pix_format_mplane *wt,
			 struct mtk_dip_tile_geom *g);

/* mtk_dip_v4l2.c */
struct mtk_dip *mtk_dip_alloc(struct device *dev);
int mtk_dip_v4l2_register(struct mtk_dip *dip);
void mtk_dip_v4l2_unregister(struct mtk_dip *dip);
u32 mtk_dip_fit_width(u32 w, u32 max);
bool mtk_dip_raw_is_10bit(const struct v4l2_pix_format_mplane *pix);
u32 mtk_dip_raw_pix_id(const struct v4l2_pix_format_mplane *pix);
bool mtk_dip_raw_is_yuv(const struct v4l2_pix_format_mplane *pix);

/* mtk_dip_params.c */
void mtk_dip_params_neutral(struct mtk_dip_param_state *par);
int mtk_dip_params_check(struct mtk_dip *dip, struct vb2_buffer *vb,
			 struct mtk_dip_buffer *buf);
void mtk_dip_params_apply(struct mtk_dip_param_state *par,
			  const struct mtk_dip_buffer *buf);
void mtk_dip_params_bayer(const struct mtk_dip_param_state *par, u32 pix_id,
			  u32 *blc, u32 *wb);

/* mtk_dip_bq.c */
int mtk_dip_bq_check(const struct v4l2_isp_params_block_header *h);
void mtk_dip_bq_ee_words(const struct mtk_dip_params_ee *ee, u32 *words);
void mtk_dip_bq_ycnr_words(const struct mtk_dip_params_ycnr *nr, u32 *words);
void mtk_dip_bq_demosaic_words(const struct mtk_dip_params_demosaic *dm,
			       u32 *words);
void mtk_dip_bq_nbc2_words(const struct mtk_dip_params_nbc2 *nr, u32 *words);
void mtk_dip_bq_nr3d_words(const struct mtk_dip_params_nr3d *nr, u32 *words);
void mtk_dip_bq_radial_words(const struct mtk_dip_params_radial *rw,
			     u32 *words);
void mtk_dip_bq_rnr_words(const struct mtk_dip_params_rnr *nr, u32 *words);
void mtk_dip_bq_neutral_words(unsigned int type, u32 *words);

/* mtk_dip_profile.c */
#define MTK_DIP_CTRL_DESC		0x0000
#define MTK_DIP_CTRL_IMAGE		0x1000
#define MTK_DIP_CTRL_TDR		0x7000
#define MTK_DIP_CTRL_TDR_WORDS		(MTK_DIP_TILE_MAX * MTK_DIP_TILE_WORDS_MAX)
#define MTK_DIP_CTRL_SIZE		(MTK_DIP_CTRL_TDR + \
					 4 * MTK_DIP_CTRL_TDR_WORDS)

void mtk_dip_profile_build(struct mtk_dip *dip);
void mtk_dip_profile_params(struct mtk_dip *dip);
void mtk_dip_profile_start(struct mtk_dip *dip, const struct mtk_dip_addrs *a);

#endif /* _MTK_DIP_H */
