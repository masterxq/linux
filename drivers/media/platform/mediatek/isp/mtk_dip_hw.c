// SPDX-License-Identifier: GPL-2.0-only
/*
 * MediaTek MT6785 ISP pass 2 (DIP): device, power and job runner.
 *
 * A job is one raw frame through the block. The runner takes one buffer of
 * every streaming video node (raw input, at least one of the main and the
 * scaled output, parameters if queued), computes the tile layout when the geometry
 * changed, powers the block, programs the frame registers, and starts the
 * tiles one after the other with the CPU: tile description address, start
 * bit of the command queue thread, wait for its interrupt. The thread loads
 * the register image before each tile, and the CPU writes the frame
 * registers before the first one, so a job does not depend on the one
 * before. The module set of a job follows its parameters: with lens shading
 * the driver adds the shading module and computes its table. With the
 * temporal noise reduction the output of a job is the reference of the
 * next one: a main buffer stays with the driver until the next job has
 * read it, a job without the main output writes a buffer of the driver. The
 * block stays powered for a short while after a job (runtime PM
 * autosuspend) and the next frame runs without a power cycle. Powering the
 * ISP domain off is the only reset of the block: it goes off at once after
 * a job that failed or ran into the time limit, and after every job with
 * the job_reset parameter set. The MFB shares the domain; after a failed
 * job the next one waits until the domain was off.
 *
 * The DMA ports of the block (SMI larb5 ports 0-6) go through the IOMMU.
 * The SMI larb driver configures the larb, and the device link of the IOMMU
 * powers and clocks it before the block; the MFB shares the larb.
 *
 * The initial register values are those of the MediaTek vendor driver for
 * this block (camera_dip.c, isp_50, Copyright (C) 2016 MediaTek Inc.,
 * GPL-2.0).
 */

#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/notifier.h>
#include <linux/platform_device.h>
#include <linux/pm_domain.h>
#include <linux/pm_runtime.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/wait.h>
#include <media/videobuf2-dma-contig.h>

#include "mtk_dip.h"
#include "mtk_dip_regs.h"

static unsigned int tile_timeout_ms = 200;
module_param(tile_timeout_ms, uint, 0644);
MODULE_PARM_DESC(tile_timeout_ms, "time limit of one tile (default 200)");

static bool job_reset;
module_param(job_reset, bool, 0644);
MODULE_PARM_DESC(job_reset,
		 "power off (reset) the block after every job instead of keeping it on for the next one (default 0)");

static unsigned int nr3d_hold_ms = 100;
module_param(nr3d_hold_ms, uint, 0644);
MODULE_PARM_DESC(nr3d_hold_ms,
		 "temporal noise reduction: a main buffer that is the reference goes back after this time without a next job (default 100)");

static bool hang_dump;
module_param(hang_dump, bool, 0644);
MODULE_PARM_DESC(hang_dump,
		 "debug: print registers at a tile that does not end (default 0)");

/* the block stays on this long after a job; sysfs power/autosuspend_delay_ms */
#define MTK_DIP_AUTOSUSPEND_MS		150

/* guard behind each buffer of the driver; the block never writes there */
#define MTK_DIP_GUARD			SZ_4K
#define MTK_DIP_GUARD_BYTE		0xa5

/* longest wait for the reset after a failed job, see mtk_dip_reset_wait() */
#define MTK_DIP_RESET_WAIT_MS		1000

/* FIFO settings of the DMA ports: CON register, CON, CON2, CON3 */
static const struct {
	u16 reg;
	u32 con;
	u32 con2;
	u32 con3;
} mtk_dip_dma_cons[DIP_DMA_PORT_NUM] = {
	{ 0x034c, 0x80000040, 0x00400040, 0x00150015 },	/* IMG2O */
	{ 0x037c, 0x80000040, 0x00400040, 0x00150015 },	/* IMG2BO */
	{ 0x03ac, 0x80000040, 0x00400040, 0x00150015 },	/* IMG3O */
	{ 0x03dc, 0x80000020, 0x00200020, 0x000a000a },	/* IMG3BO */
	{ 0x040c, 0x80000020, 0x00200020, 0x000a000a },	/* IMG3CO */
	{ 0x043c, 0x80000040, 0x00400040, 0x00150015 },	/* FEO */
	{ 0x046c, 0x80000040, 0x00400040, 0x00150015 },	/* PAK2O */
	{ 0x051c, 0x80000080, 0x00800080, 0x002a002a },	/* IMGI */
	{ 0x054c, 0x80000040, 0x00400040, 0x00150015 },	/* IMGBI */
	{ 0x057c, 0x80000040, 0x00400040, 0x00150015 },	/* IMGCI */
	{ 0x05ac, 0x80000080, 0x00800080, 0x002a002a },	/* VIPI */
	{ 0x05dc, 0x80000040, 0x00400040, 0x00150015 },	/* VIP2I */
	{ 0x060c, 0x80000040, 0x00400040, 0x00150015 },	/* VIP3I */
	{ 0x063c, 0x80000020, 0x00200020, 0x000a000a },	/* DMGI */
	{ 0x066c, 0x80000020, 0x00200020, 0x000a000a },	/* DEPI */
	{ 0x069c, 0x80000020, 0x00200020, 0x000a000a },	/* LCEI */
	{ 0x06cc, 0x80000020, 0x00200020, 0x000a000a },	/* UFDI */
	{ 0x07ec, 0x80000040, 0x00400040, 0x00400040 },	/* SMX1O */
	{ 0x081c, 0x80000040, 0x00400040, 0x00400040 },	/* SMX2O */
	{ 0x084c, 0x80000040, 0x00400040, 0x00400040 },	/* SMX3O */
	{ 0x087c, 0x80000040, 0x00400040, 0x00400040 },	/* SMX4O */
	{ 0x08ac, 0x80000040, 0x00400040, 0x00400040 },	/* SMX1I */
	{ 0x08dc, 0x80000040, 0x00400040, 0x00400040 },	/* SMX2I */
	{ 0x090c, 0x80000040, 0x00400040, 0x00400040 },	/* SMX3I */
	{ 0x093c, 0x80000040, 0x00400040, 0x00400040 },	/* SMX4I */
};

/**
 * struct mtk_dip_set - one job
 * @buf: its buffers, one per streaming node
 * @nr3d: the temporal noise reduction runs in this job
 * @chain: the output of this job becomes the reference of the next one
 * @own: driver buffer the job writes its full output into, -1 for none (a
 *	job without main output while the temporal noise reduction is on)
 * @hold: the driver keeps the main buffer of the job as the reference
 */
struct mtk_dip_set {
	struct mtk_dip_buffer *buf[MTK_DIP_NODE_NUM];
	bool nr3d;
	bool chain;
	int own;
	bool hold;
};

/*
 * The initial values of the vendor driver: clock gating of the block off,
 * interrupt sources off, command queue threads, DMA FIFO settings.
 */
static void mtk_dip_hw_init(struct mtk_dip *dip)
{
	unsigned int i;

	writel(DIP_CTL_TOP_DCM_DIS_ALL, dip->regs + DIP_CTL_TOP_DCM_DIS);
	writel(0, dip->regs + DIP_CTL_INT_EN);
	writel(0, dip->regs + DIP_CTL_CQ_INT_EN);
	writel(0, dip->regs + DIP_CTL_CQ_INT2_EN);
	writel(0, dip->regs + DIP_CTL_CQ_INT3_EN);

	for (i = 0; i < DIP_CQ_THR_NUM; i++)
		writel(DIP_CQ_THR_CTL_INIT, dip->regs + DIP_CQ_THR_CTL(i));
	for (i = 0; i < DIP_CQ_THR_NUM; i++)
		writel(DIP_CQ_THR_DESC_SIZE_INIT,
		       dip->regs + DIP_CQ_THR_DESC_SIZE(i));

	for (i = 0; i < ARRAY_SIZE(mtk_dip_dma_cons); i++) {
		writel(mtk_dip_dma_cons[i].con, dip->regs + mtk_dip_dma_cons[i].reg);
		writel(mtk_dip_dma_cons[i].con2,
		       dip->regs + mtk_dip_dma_cons[i].reg + DIP_DMA_CON2);
		writel(mtk_dip_dma_cons[i].con3,
		       dip->regs + mtk_dip_dma_cons[i].reg + DIP_DMA_CON3);
	}
}

static irqreturn_t mtk_dip_irq(int irq, void *data)
{
	struct mtk_dip *dip = data;
	u32 status, cq, cq2;

	/* enabled only while the block is powered; reading clears */
	status = readl(dip->regs + DIP_CTL_INT_STATUS);
	cq = readl(dip->regs + DIP_CTL_CQ_INT_STATUS);
	cq2 = readl(dip->regs + DIP_CTL_CQ_INT2_STATUS);
	if (!status && !cq && !cq2)
		return IRQ_NONE;

	spin_lock(&dip->lock);
	dip->irq_int |= status;
	dip->irq_cq |= cq;
	spin_unlock(&dip->lock);

	if (cq & BIT(MTK_DIP_CQ_THREAD))
		complete(&dip->tile_done);

	return IRQ_HANDLED;
}

static int mtk_dip_runtime_resume(struct device *dev)
{
	struct mtk_dip *dip = dev_get_drvdata(dev);
	int ret;

	ret = clk_prepare_enable(dip->clk);
	if (ret) {
		dev_err(dev, "failed to enable the clock: %d\n", ret);
		return ret;
	}

	mtk_dip_hw_init(dip);

	/* an unreachable block reads 0 everywhere */
	if (readl(dip->regs + DIP_CQ_THR_CTL(0)) != DIP_CQ_THR_CTL_INIT)
		dev_warn(dev, "block does not keep its initial values\n");

	enable_irq(dip->irq);

	return 0;
}

static int mtk_dip_runtime_suspend(struct device *dev)
{
	struct mtk_dip *dip = dev_get_drvdata(dev);

	/* waits for a running handler: no register access after this */
	disable_irq(dip->irq);

	clk_disable_unprepare(dip->clk);

	return 0;
}

/* Driver buffers: coherent, with a guard behind them */
static void mtk_dip_aux_free(struct mtk_dip *dip, struct mtk_dip_aux *a)
{
	if (a->cpu)
		dma_free_coherent(dip->dev, a->size + MTK_DIP_GUARD, a->cpu,
				  a->dma);
	memset(a, 0, sizeof(*a));
}

static int mtk_dip_aux_alloc(struct mtk_dip *dip, struct mtk_dip_aux *a,
			     size_t size, gfp_t gfp)
{
	size = PAGE_ALIGN(size);
	if (a->cpu && a->size >= size)
		return 0;

	mtk_dip_aux_free(dip, a);
	a->cpu = dma_alloc_coherent(dip->dev, size + MTK_DIP_GUARD, &a->dma,
				    gfp);
	if (!a->cpu)
		return -ENOMEM;
	a->size = size;
	if (upper_32_bits(a->dma + size + MTK_DIP_GUARD - 1)) {
		mtk_dip_aux_free(dip, a);
		return -ENOMEM;
	}
	memset(a->cpu + size, MTK_DIP_GUARD_BYTE, MTK_DIP_GUARD);

	return 0;
}

static bool mtk_dip_aux_guard_ok(struct mtk_dip *dip, struct mtk_dip_aux *a,
				 const char *name)
{
	if (!memchr_inv(a->cpu + a->size, MTK_DIP_GUARD_BYTE, MTK_DIP_GUARD))
		return true;

	dev_err_ratelimited(dip->dev, "write behind the %s buffer\n", name);
	memset(a->cpu + a->size, MTK_DIP_GUARD_BYTE, MTK_DIP_GUARD);
	return false;
}

/**
 * mtk_dip_aux_free_all - release the buffers of the driver
 * @dip: the device, with run_lock held or the runner stopped
 */
void mtk_dip_aux_free_all(struct mtk_dip *dip)
{
	mtk_dip_nr3d_drop(dip);
	mtk_dip_aux_free(dip, &dip->nr3d.buf[0]);
	mtk_dip_aux_free(dip, &dip->nr3d.buf[1]);
	mtk_dip_aux_free(dip, &dip->ctrl);
	mtk_dip_aux_free(dip, &dip->stripe);
	mtk_dip_aux_free(dip, &dip->lsc);
	dip->layout_ok = false;
	dip->lsc_ok = false;
}

/**
 * mtk_dip_nr3d_drop - forget the reference of the temporal noise reduction
 * @dip: the device, with run_lock held
 *
 * A main buffer that is the reference goes back to user space now; the
 * next job starts a new sequence.
 */
void mtk_dip_nr3d_drop(struct mtk_dip *dip)
{
	struct mtk_dip_nr3d *n = &dip->nr3d;

	lockdep_assert_held(&dip->run_lock);
	if (n->held)
		vb2_buffer_done(&n->held->vb.vb2_buf, VB2_BUF_STATE_DONE);
	n->held = NULL;
	n->src = MTK_DIP_REF_NONE;
}

/**
 * mtk_dip_nr3d_restart - a new stream: no reference, counters from 0
 * @dip: the device, with run_lock held
 */
void mtk_dip_nr3d_restart(struct mtk_dip *dip)
{
	mtk_dip_nr3d_drop(dip);
	memset(dip->nr3d.count, 0, sizeof(dip->nr3d.count));
	dip->nr3d.starved = false;
}

/* No reference and no buffers of the driver */
static void mtk_dip_nr3d_release(struct mtk_dip *dip)
{
	mtk_dip_nr3d_drop(dip);
	mtk_dip_aux_free(dip, &dip->nr3d.buf[0]);
	mtk_dip_aux_free(dip, &dip->nr3d.buf[1]);
}

/*
 * The memory for the module ran out: it stays off until the next raw stream
 * or until it is switched off and on again, and its buffers go back to the
 * memory region, so the jobs run as without it. Trying again at every job
 * would cost a failed allocation per frame.
 */
static void mtk_dip_nr3d_starve(struct mtk_dip *dip)
{
	mtk_dip_nr3d_release(dip);
	dip->nr3d.starved = true;
	dev_warn(dip->dev,
		 "no memory for the temporal noise reduction, off until the next stream\n");
}

/* A held main buffer goes back when no job followed in time */
static void mtk_dip_nr3d_hold_work(struct work_struct *work)
{
	struct mtk_dip *dip = container_of(to_delayed_work(work), struct mtk_dip,
					   nr3d_hold);

	mutex_lock(&dip->run_lock);
	if (dip->nr3d.held && time_after_eq(jiffies, dip->nr3d.hold_until)) {
		dev_dbg(dip->dev, "temporal noise reduction: no job in time\n");
		mtk_dip_nr3d_drop(dip);
	}
	mutex_unlock(&dip->run_lock);
}

/*
 * One buffer of the raw node and of every capture node that streams (at
 * least one of the main and the scaled node), or nothing; with a YUV input
 * one of the base image and of the weight map too when both stream (none
 * while only one of them streams). The first buffer of the params node
 * joins if the node streams with its link enabled. A job never waits for
 * parameters.
 */
static bool mtk_dip_take_set(struct mtk_dip *dip, struct mtk_dip_set *set)
{
	static const enum mtk_dip_node_id nodes[] = {
		MTK_DIP_NODE_RAW, MTK_DIP_NODE_MAIN, MTK_DIP_NODE_SCALED,
		MTK_DIP_NODE_BASE, MTK_DIP_NODE_WEIGHT,
	};
	const unsigned int cap = BIT(MTK_DIP_NODE_MAIN) |
				 BIT(MTK_DIP_NODE_SCALED);
	const unsigned int ins = BIT(MTK_DIP_NODE_BASE) |
				 BIT(MTK_DIP_NODE_WEIGHT);
	struct mtk_dip_node *par = &dip->node[MTK_DIP_NODE_PARAMS];
	unsigned int i, need;
	unsigned long flags;
	bool ok = false;

	spin_lock_irqsave(&dip->lock, flags);
	need = BIT(MTK_DIP_NODE_RAW) | (dip->streaming & cap);
	if (!(dip->streaming & BIT(MTK_DIP_NODE_RAW)) ||
	    !(dip->streaming & cap))
		goto out;
	if (dip->streaming & ins) {
		/* the format of a streaming raw node does not change */
		if ((dip->streaming & ins) != ins ||
		    !mtk_dip_raw_is_yuv(&dip->node[MTK_DIP_NODE_RAW].fmt.fmt.pix_mp))
			goto out;
		need |= ins;
	}
	for (i = 0; i < ARRAY_SIZE(nodes); i++)
		if (need & BIT(nodes[i]) && list_empty(&dip->node[nodes[i]].ready))
			goto out;

	memset(set, 0, sizeof(*set));
	set->own = -1;
	for (i = 0; i < ARRAY_SIZE(nodes); i++) {
		if (!(need & BIT(nodes[i])))
			continue;
		set->buf[nodes[i]] = list_first_entry(&dip->node[nodes[i]].ready,
						      struct mtk_dip_buffer, list);
		list_del(&set->buf[nodes[i]]->list);
	}
	if (dip->streaming & BIT(MTK_DIP_NODE_PARAMS) && dip->params_link &&
	    !list_empty(&par->ready)) {
		set->buf[MTK_DIP_NODE_PARAMS] =
			list_first_entry(&par->ready, struct mtk_dip_buffer, list);
		list_del(&set->buf[MTK_DIP_NODE_PARAMS]->list);
	}
	ok = true;
out:
	spin_unlock_irqrestore(&dip->lock, flags);
	return ok;
}

/*
 * Return the buffers; capture buffers take time stamp and number of the raw.
 * A main buffer the driver keeps as the reference goes back later.
 */
static void mtk_dip_finish(struct mtk_dip *dip, struct mtk_dip_set *set,
			   enum vb2_buffer_state state)
{
	struct vb2_v4l2_buffer *raw = &set->buf[MTK_DIP_NODE_RAW]->vb;
	u32 seq = dip->sequence++;
	unsigned int i;

	for (i = 0; i < MTK_DIP_NODE_NUM; i++) {
		struct vb2_v4l2_buffer *vb;

		if (!set->buf[i])
			continue;
		vb = &set->buf[i]->vb;
		vb->sequence = seq;
		vb->field = V4L2_FIELD_NONE;
		if (vb != raw) {
			vb->vb2_buf.timestamp = raw->vb2_buf.timestamp;
			vb->flags &= ~V4L2_BUF_FLAG_TSTAMP_SRC_MASK;
			vb->flags |= raw->flags & V4L2_BUF_FLAG_TSTAMP_SRC_MASK;
		}
		if (i == MTK_DIP_NODE_MAIN && set->hold)
			continue;
		vb2_buffer_done(&vb->vb2_buf, state);
	}
}

/**
 * mtk_dip_job_geometry - geometry of a job from the formats of its nodes
 * @in: format of the raw node
 * @crop: processed area of the raw node
 * @out: format of the main node, NULL for a job without the main output
 * @sc: format of the scaled node, NULL for a job without the scaled output
 * @base: format of the base image node, NULL for a job without the blend
 * @wt: format of the weight map node, NULL for a job without the blend
 * @g: result, without the optional modules the parameters choose
 *
 * A job without the main output takes nothing from the main node, whatever
 * format that node holds while it does not stream.
 *
 * Return: 0, -EPIPE if the main output does not have the size of @crop, or
 * for a YUV input without the main output or with the scaled output.
 */
int mtk_dip_job_geometry(const struct v4l2_pix_format_mplane *in,
			 const struct v4l2_rect *crop,
			 const struct v4l2_pix_format_mplane *out,
			 const struct v4l2_pix_format_mplane *sc,
			 const struct v4l2_pix_format_mplane *base,
			 const struct v4l2_pix_format_mplane *wt,
			 struct mtk_dip_tile_geom *g)
{
	memset(g, 0, sizeof(*g));
	if (mtk_dip_raw_is_yuv(in)) {
		if (sc || !out)
			return -EPIPE;
		g->profile |= MTK_DIP_PROFILE_YUV;
		if (base && wt) {
			g->profile |= MTK_DIP_PROFILE_MIX;
			g->ref_stride = base->plane_fmt[0].bytesperline;
			g->wt_stride = wt->plane_fmt[0].bytesperline;
		}
	} else if (base || wt) {
		return -EPIPE;
	}
	g->frame_w = in->width;
	g->frame_h = in->height;
	g->x0 = crop->left;
	g->y0 = crop->top;
	g->w = crop->width;
	g->h = crop->height;
	g->bpp = mtk_dip_raw_is_10bit(in) ? 10 : 8;
	g->pix_id = mtk_dip_raw_pix_id(in);
	g->in_stride = in->plane_fmt[0].bytesperline;
	if (sc) {
		g->profile |= MTK_DIP_PROFILE_SCALE;
		g->sw = sc->width;
		g->sh = sc->height;
		g->s_stride = sc->plane_fmt[0].bytesperline;
	}
	if (!out) {
		g->profile |= MTK_DIP_PROFILE_NO_MAIN;
		return 0;
	}
	if (out->width != crop->width || out->height != crop->height)
		return -EPIPE;
	g->out_stride = out->plane_fmt[0].bytesperline;

	return 0;
}

/* Geometry of the next job from the formats of the nodes in @set */
static int mtk_dip_geometry(struct mtk_dip *dip, const struct mtk_dip_set *set,
			    struct mtk_dip_tile_geom *g)
{
	const struct mtk_dip_node *raw = &dip->node[MTK_DIP_NODE_RAW];
	int ret;

	ret = mtk_dip_job_geometry(&raw->fmt.fmt.pix_mp, &raw->crop,
				   set->buf[MTK_DIP_NODE_MAIN] ?
				   &dip->node[MTK_DIP_NODE_MAIN].fmt.fmt.pix_mp :
				   NULL,
				   set->buf[MTK_DIP_NODE_SCALED] ?
				   &dip->node[MTK_DIP_NODE_SCALED].fmt.fmt.pix_mp :
				   NULL,
				   set->buf[MTK_DIP_NODE_BASE] ?
				   &dip->node[MTK_DIP_NODE_BASE].fmt.fmt.pix_mp :
				   NULL,
				   set->buf[MTK_DIP_NODE_WEIGHT] ?
				   &dip->node[MTK_DIP_NODE_WEIGHT].fmt.fmt.pix_mp :
				   NULL, g);
	if (ret)
		return ret;
	/*
	 * The module set follows the lens shading, edge and blend parameters;
	 * a YUV input has no shading
	 */
	if (!dip->par.mix)
		g->profile &= ~MTK_DIP_PROFILE_MIX;
	if (dip->par.lsc && !(g->profile & MTK_DIP_PROFILE_YUV))
		g->profile |= MTK_DIP_PROFILE_LSC;
	if (dip->par.ee)
		g->profile |= MTK_DIP_PROFILE_EE;

	return 0;
}

/*
 * Shading table for the parameters and the frame grid of the job; the block
 * is idle, nothing reads the table now.
 */
static int mtk_dip_prepare_lsc(struct mtk_dip *dip)
{
	int ret;

	if (dip->lsc_ok && !dip->par.lsc_new)
		return 0;

	dip->lsc_ok = false;
	ret = mtk_dip_aux_alloc(dip, &dip->lsc, MTK_DIP_LSC_TABLE_SIZE,
				GFP_KERNEL);
	if (ret)
		return ret;
	ret = mtk_dip_lsc_table(&dip->par.lsc_gain[0][0][0], dip->geom.pix_id,
				&dip->frame, dip->lsc.cpu);
	if (ret < 0)
		return ret;
	if (ret)
		dev_dbg(dip->dev, "shading: %d coefficients clamped\n", ret);
	dip->par.lsc_new = false;
	dip->lsc_ok = true;

	return 0;
}

/*
 * Temporal noise reduction of a job: with a reference that fits, the module
 * runs; in any case the output becomes the next reference, written into a
 * buffer of the driver when the job has no main output. Without the module
 * the reference goes.
 */
static void mtk_dip_nr3d_plan(struct mtk_dip *dip, struct mtk_dip_set *set,
			      struct mtk_dip_tile_geom *g)
{
	const struct mtk_dip_param_state *p = &dip->par;
	struct mtk_dip_nr3d *n = &dip->nr3d;
	u32 stride = ALIGN(g->w, 16), cycle;
	unsigned int i;
	int own;

	set->nr3d = false;
	set->chain = false;
	set->own = -1;
	if (!p->nr3d)
		n->starved = false;
	if (!p->nr3d || n->starved) {
		mtk_dip_nr3d_release(dip);
		return;
	}
	if (n->src != MTK_DIP_REF_NONE &&
	    (n->w != g->w || n->h != g->h || p->nr3d_restart ||
	     n->sleeps != READ_ONCE(dip->sleeps)))
		mtk_dip_nr3d_drop(dip);

	if (!set->buf[MTK_DIP_NODE_MAIN]) {
		/* the driver buffer that does not hold the reference */
		own = n->src == MTK_DIP_REF_OWN ? !n->own : 0;
		if (mtk_dip_aux_alloc(dip, &n->buf[own], stride * g->h * 3 / 2,
				      GFP_KERNEL | __GFP_NOWARN)) {
			mtk_dip_nr3d_starve(dip);
			return;
		}
		g->profile &= ~MTK_DIP_PROFILE_NO_MAIN;
		g->out_stride = stride;
		set->own = own;
	}
	set->chain = true;
	if (n->src == MTK_DIP_REF_NONE)
		return;

	set->nr3d = true;
	g->profile |= MTK_DIP_PROFILE_NR3D;
	g->ref_stride = n->stride;
	g->ref_dx = p->nr3d_dx;
	g->ref_dy = p->nr3d_dy;
	/* the counters of the pattern part, each up to its cycle length */
	for (i = 0; i < ARRAY_SIZE(n->count); i++) {
		cycle = (p->nr3d_w[MTK_DIP_NR3D_CYCLE_WORD] >>
			 MTK_DIP_NR3D_CYCLE_SHIFT(i)) & MTK_DIP_NR3D_CYCLE_MASK;
		n->count[i] = n->count[i] < cycle ? n->count[i] + 1 : 0;
	}
}

/* The output of a job that ran: the next reference */
static void mtk_dip_nr3d_done(struct mtk_dip *dip, struct mtk_dip_set *set,
			      int ret)
{
	struct mtk_dip_buffer *ob = set->buf[MTK_DIP_NODE_MAIN];
	const struct mtk_dip_tile_geom *g = &dip->geom;
	struct mtk_dip_nr3d *n = &dip->nr3d;
	unsigned long flags;
	bool main_on;

	if (!set->chain)
		return;
	/* the reference of this job is used up */
	mtk_dip_nr3d_drop(dip);
	if (ret)
		return;

	n->w = g->w;
	n->h = g->h;
	n->stride = g->out_stride;
	n->ref_uv = g->out_stride * g->h;
	n->sleeps = READ_ONCE(dip->sleeps);
	if (set->own >= 0) {
		n->src = MTK_DIP_REF_OWN;
		n->own = set->own;
		n->ref = n->buf[set->own].dma;
		n->size = n->buf[set->own].size;
		return;
	}

	/* a main node that stops returns the buffer itself */
	spin_lock_irqsave(&dip->lock, flags);
	main_on = dip->streaming & BIT(MTK_DIP_NODE_MAIN);
	spin_unlock_irqrestore(&dip->lock, flags);
	if (!main_on)
		return;
	n->src = MTK_DIP_REF_MAIN;
	n->held = ob;
	n->ref = vb2_dma_contig_plane_dma_addr(&ob->vb.vb2_buf, 0);
	n->size = vb2_plane_size(&ob->vb.vb2_buf, 0);
	set->hold = true;
	n->hold_until = jiffies + msecs_to_jiffies(READ_ONCE(nr3d_hold_ms));
	mod_delayed_work(dip->wq, &dip->nr3d_hold,
			 msecs_to_jiffies(READ_ONCE(nr3d_hold_ms)));
}

/* Tile layout, driver buffers and register image for a geometry */
static int mtk_dip_layout(struct mtk_dip *dip,
			  const struct mtk_dip_tile_geom *gp)
{
	struct mtk_dip_tile_geom g = *gp;
	u32 size;
	int ret;

	if (dip->layout_ok && !memcmp(&g, &dip->geom, sizeof(g)))
		return 0;

	dip->layout_ok = false;
	/* the frame grid may change: compute the shading table again */
	dip->lsc_ok = false;
	ret = mtk_dip_aux_alloc(dip, &dip->ctrl, MTK_DIP_CTRL_SIZE, GFP_KERNEL);
	if (ret)
		return ret;

	mtk_dip_frame_cfg_init(&dip->frame, g.frame_w, g.frame_h);
	ret = mtk_dip_tile_compute(&g, &dip->frame, dip->layout,
				   dip->ctrl.cpu + MTK_DIP_CTRL_TDR,
				   MTK_DIP_CTRL_TDR_WORDS);
	if (ret) {
		dev_dbg(dip->dev, "no tile layout for %ux%u: %d\n", g.w, g.h,
			ret);
		return ret;
	}

	/*
	 * A single tile neither reads nor writes the stripe store; the second
	 * one (scaled output, temporal noise reduction) follows the first
	 */
	size = max_t(u32, dip->layout->need_stripe, PAGE_SIZE);
	dip->stripe2_off = 0;
	if (g.profile & (MTK_DIP_PROFILE_SCALE | MTK_DIP_PROFILE_NR3D)) {
		dip->stripe2_off = PAGE_ALIGN(size);
		size = dip->stripe2_off +
		       max_t(u32, dip->layout->need_stripe2, PAGE_SIZE);
	}
	ret = mtk_dip_aux_alloc(dip, &dip->stripe, size, GFP_KERNEL);
	if (ret)
		return ret;

	dip->geom = g;
	mtk_dip_profile_build(dip);
	dip->layout_ok = true;

	return 0;
}

/* Geometry, tile layout and tables of a job */
static int mtk_dip_prepare(struct mtk_dip *dip, struct mtk_dip_set *set)
{
	struct mtk_dip_tile_geom g;
	int ret;

	ret = mtk_dip_geometry(dip, set, &g);
	if (ret)
		return ret;
	/* no temporal noise reduction on a YUV input */
	if (g.profile & MTK_DIP_PROFILE_YUV) {
		set->nr3d = false;
		set->chain = false;
		set->own = -1;
	} else {
		mtk_dip_nr3d_plan(dip, set, &g);
	}
	ret = mtk_dip_layout(dip, &g);
	if (!ret && (g.profile & MTK_DIP_PROFILE_LSC))
		ret = mtk_dip_prepare_lsc(dip);
	if ((ret == -ERANGE && set->nr3d) || (ret == -ENOMEM && set->chain)) {
		if (ret == -ENOMEM) {
			/* the buffers of the module took the memory the job needs */
			mtk_dip_nr3d_starve(dip);
		} else {
			/* a geometry the tiles serve only without the module */
			dev_dbg_ratelimited(dip->dev,
					    "no tile layout with the temporal noise reduction for %ux%u, motion %d,%d\n",
					    g.w, g.h, g.ref_dx, g.ref_dy);
			mtk_dip_nr3d_drop(dip);
		}
		set->nr3d = false;
		set->chain = false;
		set->own = -1;
		ret = mtk_dip_geometry(dip, set, &g);
		if (!ret)
			ret = mtk_dip_layout(dip, &g);
		if (!ret && (g.profile & MTK_DIP_PROFILE_LSC))
			ret = mtk_dip_prepare_lsc(dip);
	}

	return ret;
}

/* Bus addresses of the job and the check that every port stays inside */
static int mtk_dip_addrs(struct mtk_dip *dip, struct mtk_dip_set *set,
			 struct mtk_dip_addrs *a)
{
	const struct mtk_dip_nr3d *n = &dip->nr3d;
	struct mtk_dip_buffer *ob = set->buf[MTK_DIP_NODE_MAIN];
	struct mtk_dip_buffer *sb = set->buf[MTK_DIP_NODE_SCALED];
	struct vb2_buffer *rb = &set->buf[MTK_DIP_NODE_RAW]->vb.vb2_buf;
	const struct mtk_dip_tile_layout *l = dip->layout;
	const struct mtk_dip_tile_geom *g = &dip->geom;
	struct mtk_dip_buffer *bb = set->buf[MTK_DIP_NODE_BASE];
	struct mtk_dip_buffer *wb = set->buf[MTK_DIP_NODE_WEIGHT];
	bool mix = g->profile & MTK_DIP_PROFILE_MIX;
	u64 in_need, y_size, ref_y, ref_uv, in_uv = 0, base_uv = 0;
	bool fit;

	/*
	 * The input port reads whole 4-byte words: at the end of the last line
	 * up to 3 bytes behind a line of 2-byte alignment, which stay inside
	 * the page-aligned allocation.
	 */
	in_need = (u64)g->y0 * g->in_stride + l->need_in;
	y_size = (u64)g->out_stride * g->h;
	fit = in_need <= PAGE_ALIGN(vb2_plane_size(rb, 0)) &&
	      l->need_stripe <= dip->stripe.size &&
	      l->need_lsc <= dip->lsc.size &&
	      l->tdr_bytes <= 4 * MTK_DIP_CTRL_TDR_WORDS;
	if (ob)
		fit = fit && l->need_y <= y_size &&
		      y_size + l->need_uv <= vb2_plane_size(&ob->vb.vb2_buf, 0);
	if (sb)
		fit = fit && l->need_s <= vb2_plane_size(&sb->vb.vb2_buf, 0);
	if (g->profile & (MTK_DIP_PROFILE_SCALE | MTK_DIP_PROFILE_NR3D))
		fit = fit &&
		      (u64)dip->stripe2_off + l->need_stripe2 <= dip->stripe.size;
	if (set->own >= 0)
		fit = fit && l->need_y <= y_size &&
		      y_size + l->need_uv <= n->buf[set->own].size;
	if (set->nr3d) {
		/* the read ports start where the motion puts the frame */
		ref_y = (u64)max(g->ref_dy, 0) * g->ref_stride +
			max(g->ref_dx, 0);
		ref_uv = (u64)max(g->ref_dy, 0) / 2 * g->ref_stride +
			 max(g->ref_dx, 0);
		fit = fit && ref_y + l->need_ref <= n->ref_uv &&
		      n->ref_uv + ref_uv + l->need_ref_uv <= n->size;
	}
	if (g->profile & MTK_DIP_PROFILE_YUV) {
		/* NV12 input: the UV plane follows the Y plane of the frame */
		in_uv = (u64)g->in_stride * g->frame_h;
		fit = fit && in_need <= in_uv &&
		      in_uv + l->need_in_uv <= vb2_plane_size(rb, 0);
	}
	if (mix) {
		base_uv = (u64)g->ref_stride * g->h;
		fit = fit && bb && wb && l->need_ref <= base_uv &&
		      base_uv + l->need_ref_uv <= vb2_plane_size(&bb->vb.vb2_buf, 0) &&
		      l->need_wt <= vb2_plane_size(&wb->vb.vb2_buf, 0);
	}
	if (!fit) {
		dev_err(dip->dev, "job does not fit its buffers\n");
		return -EINVAL;
	}

	memset(a, 0, sizeof(*a));
	a->in = vb2_dma_contig_plane_dma_addr(rb, 0) + (u64)g->y0 * g->in_stride;
	if (ob) {
		a->out = vb2_dma_contig_plane_dma_addr(&ob->vb.vb2_buf, 0);
		a->out_uv = y_size;
	} else if (set->own >= 0) {
		a->out = n->buf[set->own].dma;
		a->out_uv = y_size;
	}
	if (sb)
		a->scaled = vb2_dma_contig_plane_dma_addr(&sb->vb.vb2_buf, 0);
	if (set->nr3d) {
		a->ref = n->ref;
		a->ref_uv = n->ref_uv;
	}
	a->in_uv = in_uv;
	if (mix) {
		a->ref = vb2_dma_contig_plane_dma_addr(&bb->vb.vb2_buf, 0);
		a->ref_uv = base_uv;
		a->wt = vb2_dma_contig_plane_dma_addr(&wb->vb.vb2_buf, 0);
	}

	return 0;
}

/* Register ranges printed at a tile that does not end (debug) */
static const u16 mtk_dip_dump_ranges[][2] = {
	{ 0x0000, 0x0100 }, { 0x0300, 0x0360 }, { 0x0390, 0x03c0 },
	{ 0x0500, 0x0520 }, { 0x0740, 0x07b0 }, { 0x0800, 0x0820 },
	{ 0x0830, 0x0850 }, { 0x08c0, 0x08e0 }, { 0x08f0, 0x0910 },
	{ 0x5300, 0x5340 },
};

static void mtk_dip_hang_dump(struct mtk_dip *dip)
{
	unsigned int i, o;

	dev_info(dip->dev, "hang dump: profile %08x\n", dip->geom.profile);
	for (i = 0; i < ARRAY_SIZE(mtk_dip_dump_ranges); i++)
		for (o = mtk_dip_dump_ranges[i][0]; o < mtk_dip_dump_ranges[i][1];
		     o += 16)
			dev_info(dip->dev, "hang %04x: %08x %08x %08x %08x\n", o,
				 readl(dip->regs + o), readl(dip->regs + o + 4),
				 readl(dip->regs + o + 8),
				 readl(dip->regs + o + 12));
}

/* The CPU starts the tiles one after the other */
static int mtk_dip_tiles(struct mtk_dip *dip)
{
	const struct mtk_dip_tile_layout *l = dip->layout;
	u32 tdr = lower_32_bits(dip->ctrl.dma) + MTK_DIP_CTRL_TDR;
	u32 start = DIP_CTL_START_CQ_THR(MTK_DIP_CQ_THREAD);
	unsigned int ms = READ_ONCE(tile_timeout_ms) ?: 1;
	unsigned long timeout = msecs_to_jiffies(ms);
	unsigned int k;

	for (k = 0; k < l->n_tiles; k++) {
		reinit_completion(&dip->tile_done);
		writel(tdr + 4 * k * l->words, dip->regs + DIP_TDRI_BASE_ADDR);
		writel(start, dip->regs + DIP_CTL_START);
		if (wait_for_completion_timeout(&dip->tile_done, timeout))
			continue;
		/* the thread clears its start bit at the end of the tile */
		if (!(readl(dip->regs + DIP_CTL_START) & start)) {
			dev_warn_once(dip->dev, "tile end without interrupt\n");
			continue;
		}
		dev_err(dip->dev,
			"tile %u of %u: no end after %u ms, status %08x %08x, modules done %08x %08x, handshake %08x\n",
			k, l->n_tiles, ms,
			readl(dip->regs + DIP_CTL_INT_STATUSX),
			readl(dip->regs + DIP_CTL_CQ_INT_STATUSX),
			readl(dip->regs + DIP_CTL_DONE0),
			readl(dip->regs + DIP_CTL_DONE1),
			readl(dip->regs + DIP_CTL_HANDSHAKE));
		if (READ_ONCE(hang_dump))
			mtk_dip_hang_dump(dip);
		return -ETIMEDOUT;
	}

	return 0;
}

/*
 * Powering the ISP domain off is the only reset of the block, and the MFB
 * shares the domain: after a failed job the domain stays on as long as the
 * MFB is on. The next job waits for the domain to go off, for a bounded
 * time.
 */
static int mtk_dip_pd_notify(struct notifier_block *nb, unsigned long action,
			     void *data)
{
	struct mtk_dip *dip = container_of(nb, struct mtk_dip, pd_nb);

	if (action == GENPD_NOTIFY_OFF) {
		WRITE_ONCE(dip->reset_pending, false);
		wake_up(&dip->reset_wait);
	}

	return NOTIFY_OK;
}

static void mtk_dip_reset_wait(struct mtk_dip *dip)
{
	if (!READ_ONCE(dip->reset_pending))
		return;

	if (!wait_event_timeout(dip->reset_wait, !READ_ONCE(dip->reset_pending),
				msecs_to_jiffies(MTK_DIP_RESET_WAIT_MS))) {
		dev_warn(dip->dev,
			 "no reset after the failed job: the ISP domain stays on\n");
		WRITE_ONCE(dip->reset_pending, false);
	}
}

/*
 * One job: power on unless the block is still on from the job before,
 * frame registers, tiles. The block goes off after the autosuspend delay,
 * or at once to reset it: after a failed job, a write behind a driver
 * buffer, or with job_reset.
 */
static int mtk_dip_run(struct mtk_dip *dip, const struct mtk_dip_addrs *a)
{
	int ret;

	mtk_dip_reset_wait(dip);

	ret = pm_runtime_resume_and_get(dip->dev);
	if (ret < 0)
		return ret;

	mtk_dip_profile_start(dip, a);
	writel(BIT(MTK_DIP_CQ_THREAD), dip->regs + DIP_CTL_CQ_INT_EN);
	ret = mtk_dip_tiles(dip);

	writel(0, dip->regs + DIP_CTL_START);
	writel(0, dip->regs + DIP_CTL_CQ_INT_EN);

	if (!mtk_dip_aux_guard_ok(dip, &dip->ctrl, "control") ||
	    !mtk_dip_aux_guard_ok(dip, &dip->stripe, "stripe") ||
	    (dip->lsc.cpu && !mtk_dip_aux_guard_ok(dip, &dip->lsc, "shading")) ||
	    (dip->nr3d.buf[0].cpu &&
	     !mtk_dip_aux_guard_ok(dip, &dip->nr3d.buf[0], "reference")) ||
	    (dip->nr3d.buf[1].cpu &&
	     !mtk_dip_aux_guard_ok(dip, &dip->nr3d.buf[1], "reference")))
		ret = -EIO;

	if (ret)
		WRITE_ONCE(dip->reset_pending, true);
	if (ret || READ_ONCE(job_reset))
		pm_runtime_put_sync_suspend(dip->dev);
	else
		pm_runtime_put_autosuspend(dip->dev);

	return ret;
}

static void mtk_dip_work(struct work_struct *work)
{
	struct mtk_dip *dip = container_of(work, struct mtk_dip, work);
	struct mtk_dip_addrs addrs;
	struct mtk_dip_set set;
	int ret;

	while (mtk_dip_take_set(dip, &set)) {
		mutex_lock(&dip->run_lock);
		if (set.buf[MTK_DIP_NODE_PARAMS])
			mtk_dip_params_apply(&dip->par,
					     set.buf[MTK_DIP_NODE_PARAMS]);

		ret = mtk_dip_prepare(dip, &set);
		dip->par.nr3d_restart = false;
		if (!ret)
			ret = mtk_dip_addrs(dip, &set, &addrs);
		if (!ret) {
			mtk_dip_profile_params(dip);
			ret = mtk_dip_run(dip, &addrs);
		}
		if (ret)
			dev_dbg(dip->dev, "job %u failed: %d\n", dip->sequence,
				ret);
		/* the reference before goes back ahead of the buffers of the job */
		mtk_dip_nr3d_done(dip, &set, ret);
		mtk_dip_finish(dip, &set, ret ? VB2_BUF_STATE_ERROR :
				VB2_BUF_STATE_DONE);
		mutex_unlock(&dip->run_lock);
	}
}

/**
 * mtk_dip_kick - run the jobs that are complete
 * @dip: the device
 */
void mtk_dip_kick(struct mtk_dip *dip)
{
	queue_work(dip->wq, &dip->work);
}

static void mtk_dip_wq_destroy(void *wq)
{
	destroy_workqueue(wq);
}

static void mtk_dip_pd_remove(void *dev)
{
	dev_pm_genpd_remove_notifier(dev);
}

static int mtk_dip_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mtk_dip *dip;
	int ret;

	dip = mtk_dip_alloc(dev);
	if (IS_ERR(dip))
		return PTR_ERR(dip);
	dip->layout = devm_kzalloc(dev, sizeof(*dip->layout), GFP_KERNEL);
	if (!dip->layout)
		return -ENOMEM;

	spin_lock_init(&dip->lock);
	ret = devm_mutex_init(dev, &dip->run_lock);
	if (ret)
		return ret;
	init_completion(&dip->tile_done);
	INIT_WORK(&dip->work, mtk_dip_work);
	INIT_DELAYED_WORK(&dip->nr3d_hold, mtk_dip_nr3d_hold_work);
	mtk_dip_params_neutral(&dip->par);
	platform_set_drvdata(pdev, dip);

	if (!dev->pm_domain)
		return dev_err_probe(dev, -ENODEV, "no power domain\n");
	init_waitqueue_head(&dip->reset_wait);
	dip->pd_nb.notifier_call = mtk_dip_pd_notify;
	ret = dev_pm_genpd_add_notifier(dev, &dip->pd_nb);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to watch the power domain\n");
	ret = devm_add_action_or_reset(dev, mtk_dip_pd_remove, dev);
	if (ret)
		return ret;

	/* all addresses of the block are 32 bit */
	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return dev_err_probe(dev, ret, "failed to set the DMA mask\n");
	/*
	 * Each buffer is one contiguous segment, however large: an imported
	 * DMABUF (a CAMSV frame) is mapped for this device as a single entry.
	 */
	dma_set_max_seg_size(dev, UINT_MAX);

	dip->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(dip->regs))
		return dev_err_probe(dev, PTR_ERR(dip->regs),
				     "failed to map the registers\n");

	dip->clk = devm_clk_get(dev, "dip");
	if (IS_ERR(dip->clk))
		return dev_err_probe(dev, PTR_ERR(dip->clk),
				     "failed to get the clock\n");

	dip->irq = platform_get_irq(pdev, 0);
	if (dip->irq < 0)
		return dip->irq;
	ret = devm_request_irq(dev, dip->irq, mtk_dip_irq, IRQF_NO_AUTOEN,
			       dev_name(dev), dip);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to request the interrupt\n");

	dip->wq = alloc_ordered_workqueue("mtk-dip", WQ_FREEZABLE);
	if (!dip->wq)
		return -ENOMEM;
	ret = devm_add_action_or_reset(dev, mtk_dip_wq_destroy, dip->wq);
	if (ret)
		return ret;

	pm_runtime_set_autosuspend_delay(dev, MTK_DIP_AUTOSUSPEND_MS);
	pm_runtime_use_autosuspend(dev);
	ret = devm_pm_runtime_enable(dev);
	if (ret)
		return ret;

	ret = mtk_dip_v4l2_register(dip);
	if (ret)
		return dev_err_probe(dev, ret, "failed to register the nodes\n");

	return 0;
}

static void mtk_dip_remove(struct platform_device *pdev)
{
	struct mtk_dip *dip = platform_get_drvdata(pdev);
	int ret;

	/* open files keep the nodes, but not the hardware or the runner */
	mtk_dip_v4l2_unregister(dip);
	flush_workqueue(dip->wq);
	cancel_delayed_work_sync(&dip->nr3d_hold);

	/*
	 * Off now: the block can still be on from the last job, for the
	 * autosuspend delay. Disabling runtime PM alone would leave it on, and
	 * the next probe would find it on with its interrupt disabled.
	 */
	ret = pm_runtime_suspend(dip->dev);
	if (ret < 0)
		dev_warn(dip->dev, "block not off at the unbind: %d\n", ret);

	mutex_lock(&dip->run_lock);
	mtk_dip_aux_free_all(dip);
	mutex_unlock(&dip->run_lock);
}

/*
 * The frames after a system sleep do not continue the ones before: the
 * temporal noise reduction starts a new sequence (the job runner is frozen
 * here and reads the count at its next job)
 */
static int mtk_dip_suspend(struct device *dev)
{
	struct mtk_dip *dip = dev_get_drvdata(dev);

	WRITE_ONCE(dip->sleeps, dip->sleeps + 1);

	return pm_runtime_force_suspend(dev);
}

static const struct dev_pm_ops mtk_dip_pm_ops = {
	SYSTEM_SLEEP_PM_OPS(mtk_dip_suspend, pm_runtime_force_resume)
	RUNTIME_PM_OPS(mtk_dip_runtime_suspend, mtk_dip_runtime_resume, NULL)
};

static const struct of_device_id mtk_dip_of_ids[] = {
	{ .compatible = "mediatek,mt6785-dip" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mtk_dip_of_ids);

static struct platform_driver mtk_dip_driver = {
	.probe = mtk_dip_probe,
	.remove = mtk_dip_remove,
	.driver = {
		.name = "mtk-dip",
		.of_match_table = mtk_dip_of_ids,
		.pm = pm_ptr(&mtk_dip_pm_ops),
	},
};
module_platform_driver(mtk_dip_driver);

MODULE_DESCRIPTION("MediaTek MT6785 ISP pass 2 (DIP) driver");
MODULE_LICENSE("GPL");
