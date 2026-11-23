// SPDX-License-Identifier: GPL-2.0-only
/*
 * MediaTek MT6785 multi-frame blending block (MFB): device, power and job
 * runner.
 *
 * A job is one blend: base and reference image in, blended image and weight
 * map out, a weight map in from the second blend of a series on. The runner
 * takes one buffer of every node the job needs, computes the tiles when the
 * geometry changed, powers the block, writes the frame registers with the
 * CPU and sets the start bit; the block walks through the tiles on its own
 * and raises its interrupt at the end of the frame. The block stays powered
 * for a short while after a job (runtime PM autosuspend). A job that runs
 * into the time limit is stopped with the reset sequence of the block; the
 * ISP power domain, shared with the DIP, goes off as soon as neither block
 * is busy.
 *
 * The three DMA ports of the block (SMI larb5 ports 18-20) go through the
 * IOMMU. The SMI larb driver configures the larb, and the device link of
 * the IOMMU powers and clocks it before the block; the DIP shares the larb.
 *
 * Reset sequence and interrupt sources follow the MediaTek vendor driver of
 * this block (camera_mfb.c, isp_50, Copyright (C) 2015 MediaTek Inc.,
 * GPL-2.0).
 */

#include <linux/array_size.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <media/videobuf2-dma-contig.h>

#include "mtk_mfb.h"
#include "mtk_mfb_regs.h"

static unsigned int job_timeout_ms = 500;
module_param(job_timeout_ms, uint, 0644);
MODULE_PARM_DESC(job_timeout_ms, "time limit of one blend (default 500)");

static bool job_reset;
module_param(job_reset, bool, 0644);
MODULE_PARM_DESC(job_reset,
		 "power off the block after every job instead of keeping it on for the next one (default 0)");

static bool hang_dump;
module_param(hang_dump, bool, 0644);
MODULE_PARM_DESC(hang_dump,
		 "debug: print the registers of a blend that does not end (default 0)");

static bool reg_trace;
module_param(reg_trace, bool, 0644);
MODULE_PARM_DESC(reg_trace,
		 "debug: print the frame registers and the tile descriptions of each job (default 0)");

/* the block stays on this long after a job; sysfs power/autosuspend_delay_ms */
#define MTK_MFB_AUTOSUSPEND_MS		150

/* guard behind the driver buffer; the block never writes there */
#define MTK_MFB_GUARD			SZ_4K
#define MTK_MFB_GUARD_BYTE		0xa5

/* reset: request, wait for the DMA to stop, release */
#define MFB_SW_RST_REQ			0x001
#define MFB_SW_RST_DMA_IDLE		BIT(1)
#define MFB_SW_RST_HW			0x101
#define MFB_SW_RST_HW_HOLD		0x100
#define MTK_MFB_RST_US			1000

/* the buffers of one job, one per node that takes part */
struct mtk_mfb_set {
	struct mtk_mfb_buffer *buf[MTK_MFB_NODE_NUM];
};

/*
 * Reset sequence of the vendor driver. Its wait for the DMA is a loop whose
 * condition is always false; here it waits a bounded time for the bit the
 * loop names.
 */
static void mtk_mfb_reset(struct mtk_mfb *mfb)
{
	u32 val;
	int ret;

	writel(MFB_SW_RST_REQ, mfb->regs + MFB_SW_RST);
	ret = readl_poll_timeout_atomic(mfb->regs + MFB_SW_RST, val,
					val & MFB_SW_RST_DMA_IDLE, 1,
					MTK_MFB_RST_US);
	if (ret)
		dev_warn(mfb->dev, "reset: DMA does not stop, SW_RST %08x\n",
			 val);
	writel(MFB_SW_RST_HW, mfb->regs + MFB_SW_RST);
	writel(MFB_SW_RST_HW_HOLD, mfb->regs + MFB_SW_RST);
	writel(0, mfb->regs + MFB_SW_RST);
}

static irqreturn_t mtk_mfb_irq(int irq, void *data)
{
	struct mtk_mfb *mfb = data;
	u32 status;

	/* enabled only while the block is powered; reading clears */
	status = readl(mfb->regs + MFB_INT_STATUS);
	if (!status)
		return IRQ_NONE;

	spin_lock(&mfb->lock);
	mfb->irq_status |= status;
	spin_unlock(&mfb->lock);

	if (status & MFB_INT_DONE) {
		/* nothing more from this job; the next one enables again */
		writel(0, mfb->regs + MFB_INT_CTL);
		complete(&mfb->done);
	}

	return IRQ_HANDLED;
}

static int mtk_mfb_runtime_resume(struct device *dev)
{
	struct mtk_mfb *mfb = dev_get_drvdata(dev);
	int ret;

	ret = clk_prepare_enable(mfb->clk);
	if (ret) {
		dev_err(dev, "failed to enable the clock: %d\n", ret);
		return ret;
	}

	writel(0, mfb->regs + MFB_INT_CTL);
	readl(mfb->regs + MFB_INT_STATUS);
	enable_irq(mfb->irq);

	return 0;
}

static int mtk_mfb_runtime_suspend(struct device *dev)
{
	struct mtk_mfb *mfb = dev_get_drvdata(dev);

	/* waits for a running handler: no register access after this */
	disable_irq(mfb->irq);

	clk_disable_unprepare(mfb->clk);

	return 0;
}

/* Driver buffer: coherent, with a guard behind it */
static void mtk_mfb_aux_free(struct mtk_mfb *mfb, struct mtk_mfb_aux *a)
{
	if (a->cpu)
		dma_free_coherent(mfb->dev, a->size + MTK_MFB_GUARD, a->cpu,
				  a->dma);
	memset(a, 0, sizeof(*a));
}

static int mtk_mfb_aux_alloc(struct mtk_mfb *mfb, struct mtk_mfb_aux *a,
			     size_t size)
{
	size = PAGE_ALIGN(size);
	if (a->cpu && a->size >= size)
		return 0;

	mtk_mfb_aux_free(mfb, a);
	a->cpu = dma_alloc_coherent(mfb->dev, size + MTK_MFB_GUARD, &a->dma,
				    GFP_KERNEL);
	if (!a->cpu)
		return -ENOMEM;
	a->size = size;
	if (upper_32_bits(a->dma + size + MTK_MFB_GUARD - 1)) {
		mtk_mfb_aux_free(mfb, a);
		return -ENOMEM;
	}
	memset(a->cpu + size, MTK_MFB_GUARD_BYTE, MTK_MFB_GUARD);

	return 0;
}

static bool mtk_mfb_aux_guard_ok(struct mtk_mfb *mfb, struct mtk_mfb_aux *a)
{
	if (!memchr_inv(a->cpu + a->size, MTK_MFB_GUARD_BYTE, MTK_MFB_GUARD))
		return true;

	dev_err_ratelimited(mfb->dev, "write behind the tile buffer\n");
	memset(a->cpu + a->size, MTK_MFB_GUARD_BYTE, MTK_MFB_GUARD);
	return false;
}

/**
 * mtk_mfb_aux_free_all - release the buffers of the driver
 * @mfb: the device, with run_lock held or the runner stopped
 */
void mtk_mfb_aux_free_all(struct mtk_mfb *mfb)
{
	mtk_mfb_aux_free(mfb, &mfb->tdr);
	mfb->layout_ok = false;
}

/*
 * One buffer of each required node and of the weight map input while it
 * streams, or nothing; the first buffer of the params node joins if the node
 * streams with its link enabled. A job never waits for parameters.
 */
static bool mtk_mfb_take_set(struct mtk_mfb *mfb, struct mtk_mfb_set *set)
{
	static const enum mtk_mfb_node_id nodes[] = {
		MTK_MFB_NODE_BASE, MTK_MFB_NODE_REF, MTK_MFB_NODE_WT_IN,
		MTK_MFB_NODE_OUT, MTK_MFB_NODE_WT_OUT,
	};
	const unsigned int req = BIT(MTK_MFB_NODE_BASE) | BIT(MTK_MFB_NODE_REF) |
				 BIT(MTK_MFB_NODE_OUT) |
				 BIT(MTK_MFB_NODE_WT_OUT);
	struct mtk_mfb_node *par = &mfb->node[MTK_MFB_NODE_PARAMS];
	unsigned int i, need;
	unsigned long flags;
	bool ok = false;

	spin_lock_irqsave(&mfb->lock, flags);
	if ((mfb->streaming & req) != req)
		goto out;
	need = req | (mfb->streaming & BIT(MTK_MFB_NODE_WT_IN));
	for (i = 0; i < ARRAY_SIZE(nodes); i++)
		if (need & BIT(nodes[i]) && list_empty(&mfb->node[nodes[i]].ready))
			goto out;

	memset(set, 0, sizeof(*set));
	for (i = 0; i < ARRAY_SIZE(nodes); i++) {
		if (!(need & BIT(nodes[i])))
			continue;
		set->buf[nodes[i]] = list_first_entry(&mfb->node[nodes[i]].ready,
						      struct mtk_mfb_buffer, list);
		list_del(&set->buf[nodes[i]]->list);
	}
	if (mfb->streaming & BIT(MTK_MFB_NODE_PARAMS) && mfb->params_link &&
	    !list_empty(&par->ready)) {
		set->buf[MTK_MFB_NODE_PARAMS] =
			list_first_entry(&par->ready, struct mtk_mfb_buffer, list);
		list_del(&set->buf[MTK_MFB_NODE_PARAMS]->list);
	}
	ok = true;
out:
	spin_unlock_irqrestore(&mfb->lock, flags);
	return ok;
}

/* Return the buffers; capture buffers take time stamp and number of the base */
static void mtk_mfb_finish(struct mtk_mfb *mfb, struct mtk_mfb_set *set,
			   enum vb2_buffer_state state)
{
	struct vb2_v4l2_buffer *base = &set->buf[MTK_MFB_NODE_BASE]->vb;
	u32 seq = mfb->sequence++;
	unsigned int i;

	for (i = 0; i < MTK_MFB_NODE_NUM; i++) {
		struct vb2_v4l2_buffer *vb;

		if (!set->buf[i])
			continue;
		vb = &set->buf[i]->vb;
		vb->sequence = seq;
		vb->field = V4L2_FIELD_NONE;
		if (vb != base) {
			vb->vb2_buf.timestamp = base->vb2_buf.timestamp;
			vb->flags &= ~V4L2_BUF_FLAG_TSTAMP_SRC_MASK;
			vb->flags |= base->flags & V4L2_BUF_FLAG_TSTAMP_SRC_MASK;
		}
		vb2_buffer_done(&vb->vb2_buf, state);
	}
}

/* Tiles and their descriptions for the geometry of the job */
static int mtk_mfb_prepare(struct mtk_mfb *mfb, const struct mtk_mfb_set *set)
{
	struct mtk_mfb_geom g;
	int ret;

	ret = mtk_mfb_job_geom(mfb, !!set->buf[MTK_MFB_NODE_WT_IN], &g);
	if (ret)
		return ret;
	if (mfb->layout_ok && !memcmp(&g, &mfb->geom, sizeof(g)))
		return 0;

	mfb->layout_ok = false;
	ret = mtk_mfb_aux_alloc(mfb, &mfb->tdr, MTK_MFB_TDR_BYTES);
	if (ret)
		return ret;
	ret = mtk_mfb_tiles(&g, &mfb->layout);
	if (ret) {
		dev_dbg(mfb->dev, "no tiles for %ux%u: %d\n", g.w, g.h, ret);
		return ret;
	}
	mtk_mfb_tdr(&g, &mfb->layout, mfb->tdr.cpu);
	mfb->geom = g;
	mfb->layout_ok = true;

	return 0;
}

static dma_addr_t mtk_mfb_addr(const struct mtk_mfb_set *set,
			       enum mtk_mfb_node_id id)
{
	return vb2_dma_contig_plane_dma_addr(&set->buf[id]->vb.vb2_buf, 0);
}

static size_t mtk_mfb_bufsize(const struct mtk_mfb_set *set,
			      enum mtk_mfb_node_id id)
{
	return vb2_plane_size(&set->buf[id]->vb.vb2_buf, 0);
}

static bool mtk_mfb_overlap(const struct mtk_mfb_set *set,
			    enum mtk_mfb_node_id a, enum mtk_mfb_node_id b)
{
	dma_addr_t sa, sb;

	if (!set->buf[a] || !set->buf[b])
		return false;
	sa = mtk_mfb_addr(set, a);
	sb = mtk_mfb_addr(set, b);

	return sa < sb + mtk_mfb_bufsize(set, b) &&
	       sb < sa + mtk_mfb_bufsize(set, a);
}

/* Bus addresses of the job; inputs and outputs must not share memory */
static int mtk_mfb_addrs(struct mtk_mfb *mfb, const struct mtk_mfb_set *set,
			 struct mtk_mfb_addrs *a)
{
	static const enum mtk_mfb_node_id ins[] = {
		MTK_MFB_NODE_BASE, MTK_MFB_NODE_REF, MTK_MFB_NODE_WT_IN,
	};
	static const enum mtk_mfb_node_id outs[] = {
		MTK_MFB_NODE_OUT, MTK_MFB_NODE_WT_OUT,
	};
	const struct mtk_mfb_geom *g = &mfb->geom;
	unsigned int i, j;

	for (i = 0; i < ARRAY_SIZE(outs); i++) {
		if (mtk_mfb_overlap(set, outs[i], outs[i ^ 1]))
			goto overlap;
		for (j = 0; j < ARRAY_SIZE(ins); j++)
			if (mtk_mfb_overlap(set, outs[i], ins[j]))
				goto overlap;
	}

	memset(a, 0, sizeof(*a));
	a->base = mtk_mfb_addr(set, MTK_MFB_NODE_BASE);
	a->base_uv = a->base + g->base_bpl * g->h;
	a->ref = mtk_mfb_addr(set, MTK_MFB_NODE_REF);
	a->ref_uv = a->ref + g->ref_bpl * g->h;
	if (set->buf[MTK_MFB_NODE_WT_IN])
		a->wt_in = mtk_mfb_addr(set, MTK_MFB_NODE_WT_IN);
	a->out = mtk_mfb_addr(set, MTK_MFB_NODE_OUT);
	a->out_uv = a->out + g->out_bpl * g->h;
	a->wt_out = mtk_mfb_addr(set, MTK_MFB_NODE_WT_OUT);
	a->tdr = mfb->tdr.dma;

	return 0;

overlap:
	dev_err(mfb->dev, "job writes into one of its inputs or outputs\n");
	return -EINVAL;
}

/* Register ranges printed at a blend that does not end (debug) */
static const u16 mtk_mfb_dump_ranges[][2] = {
	{ 0x0000, 0x00c0 }, { 0x0100, 0x0330 }, { 0x0340, 0x0370 },
	{ 0x0380, 0x03b0 }, { 0x03c0, 0x03e0 },
};

static void mtk_mfb_hang_dump(struct mtk_mfb *mfb)
{
	unsigned int i, o;

	for (i = 0; i < ARRAY_SIZE(mtk_mfb_dump_ranges); i++)
		for (o = mtk_mfb_dump_ranges[i][0]; o < mtk_mfb_dump_ranges[i][1];
		     o += 16)
			dev_info(mfb->dev, "hang %04x: %08x %08x %08x %08x\n", o,
				 readl(mfb->regs + o), readl(mfb->regs + o + 4),
				 readl(mfb->regs + o + 8),
				 readl(mfb->regs + o + 12));
}

static void mtk_mfb_trace(struct mtk_mfb *mfb, const struct mtk_mfb_reg *regs,
			  unsigned int n)
{
	const u32 *w = mfb->tdr.cpu;
	unsigned int i, k;

	for (i = 0; i < n; i++)
		dev_info(mfb->dev, "reg %04x = %08x\n", regs[i].off,
			 regs[i].val);
	for (k = 0; k < mfb->layout.n_tiles; k++)
		for (i = 0; i < MTK_MFB_TDR_WORDS; i += 8, w += 8)
			dev_info(mfb->dev,
				 "tdr %2u.%02u: %08x %08x %08x %08x %08x %08x %08x %08x\n",
				 k, i, w[0], w[1], w[2], w[3], w[4], w[5], w[6],
				 w[7]);
}

/*
 * One blend: power on unless the block is still on from the job before,
 * frame registers, start, wait for the end of the frame. The block goes off
 * after the autosuspend delay, or at once after a failed job, a write behind
 * the tile buffer, or with job_reset.
 */
static int mtk_mfb_run(struct mtk_mfb *mfb, const struct mtk_mfb_addrs *a)
{
	struct mtk_mfb_reg regs[MTK_MFB_REG_MAX];
	unsigned int ms = READ_ONCE(job_timeout_ms) ?: 1;
	unsigned long flags;
	unsigned int n, i;
	u32 irq;
	int ret;

	n = mtk_mfb_frame_regs(&mfb->geom, a, &mfb->tuning, regs);
	if (READ_ONCE(reg_trace))
		mtk_mfb_trace(mfb, regs, n);

	ret = pm_runtime_resume_and_get(mfb->dev);
	if (ret < 0)
		return ret;

	spin_lock_irqsave(&mfb->lock, flags);
	mfb->irq_status = 0;
	spin_unlock_irqrestore(&mfb->lock, flags);
	reinit_completion(&mfb->done);

	for (i = 0; i < n; i++)
		writel(regs[i].val, mfb->regs + regs[i].off);
	writel(MFB_TOP_CFG1_START, mfb->regs + MFB_TOP_CFG1);

	ret = 0;
	if (!wait_for_completion_timeout(&mfb->done, msecs_to_jiffies(ms))) {
		dev_err(mfb->dev,
			"blend %ux%u, %u tiles: no end after %u ms, status %08x, debug %08x, DMA errors %08x %08x\n",
			mfb->geom.w, mfb->geom.h, mfb->layout.n_tiles, ms,
			readl(mfb->regs + MFB_INT_STATUS),
			readl(mfb->regs + MFB_DBG_OUT0),
			readl(mfb->regs + MFB_MFBO_ERR_STAT),
			readl(mfb->regs + MFB_MFB2I_B_ERR_STAT));
		if (READ_ONCE(hang_dump))
			mtk_mfb_hang_dump(mfb);
		writel(0, mfb->regs + MFB_INT_CTL);
		mtk_mfb_reset(mfb);
		ret = -ETIMEDOUT;
	}
	writel(0, mfb->regs + MFB_TOP_CFG1);

	spin_lock_irqsave(&mfb->lock, flags);
	irq = mfb->irq_status;
	spin_unlock_irqrestore(&mfb->lock, flags);
	dev_dbg(mfb->dev, "blend %u: interrupt status %08x\n", mfb->sequence,
		irq);

	if (!mtk_mfb_aux_guard_ok(mfb, &mfb->tdr))
		ret = -EIO;

	if (ret || READ_ONCE(job_reset))
		pm_runtime_put_sync_suspend(mfb->dev);
	else
		pm_runtime_put_autosuspend(mfb->dev);

	return ret;
}

static void mtk_mfb_work(struct work_struct *work)
{
	struct mtk_mfb *mfb = container_of(work, struct mtk_mfb, work);
	struct mtk_mfb_addrs addrs;
	struct mtk_mfb_set set;
	int ret;

	while (mtk_mfb_take_set(mfb, &set)) {
		mutex_lock(&mfb->run_lock);
		if (set.buf[MTK_MFB_NODE_PARAMS])
			mtk_mfb_params_apply(&mfb->tuning,
					     set.buf[MTK_MFB_NODE_PARAMS]);

		ret = mtk_mfb_prepare(mfb, &set);
		if (!ret)
			ret = mtk_mfb_addrs(mfb, &set, &addrs);
		if (!ret)
			ret = mtk_mfb_run(mfb, &addrs);
		if (ret)
			dev_dbg(mfb->dev, "job %u failed: %d\n", mfb->sequence,
				ret);
		mtk_mfb_finish(mfb, &set, ret ? VB2_BUF_STATE_ERROR :
			       VB2_BUF_STATE_DONE);
		mutex_unlock(&mfb->run_lock);
	}
}

/**
 * mtk_mfb_kick - run the jobs that are complete
 * @mfb: the device
 */
void mtk_mfb_kick(struct mtk_mfb *mfb)
{
	queue_work(mfb->wq, &mfb->work);
}

static void mtk_mfb_wq_destroy(void *wq)
{
	destroy_workqueue(wq);
}

static int mtk_mfb_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mtk_mfb *mfb;
	int ret;

	mfb = mtk_mfb_alloc(dev);
	if (IS_ERR(mfb))
		return PTR_ERR(mfb);

	spin_lock_init(&mfb->lock);
	ret = devm_mutex_init(dev, &mfb->run_lock);
	if (ret)
		return ret;
	init_completion(&mfb->done);
	INIT_WORK(&mfb->work, mtk_mfb_work);
	platform_set_drvdata(pdev, mfb);

	if (!dev->pm_domain)
		return dev_err_probe(dev, -ENODEV, "no power domain\n");

	/* all addresses of the block are 32 bit */
	ret = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (ret)
		return dev_err_probe(dev, ret, "failed to set the DMA mask\n");
	/* each buffer is one contiguous segment, however large */
	dma_set_max_seg_size(dev, UINT_MAX);

	mfb->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(mfb->regs))
		return dev_err_probe(dev, PTR_ERR(mfb->regs),
				     "failed to map the registers\n");

	mfb->clk = devm_clk_get(dev, "mfb");
	if (IS_ERR(mfb->clk))
		return dev_err_probe(dev, PTR_ERR(mfb->clk),
				     "failed to get the clock\n");

	mfb->irq = platform_get_irq(pdev, 0);
	if (mfb->irq < 0)
		return mfb->irq;
	ret = devm_request_irq(dev, mfb->irq, mtk_mfb_irq, IRQF_NO_AUTOEN,
			       dev_name(dev), mfb);
	if (ret)
		return dev_err_probe(dev, ret,
				     "failed to request the interrupt\n");

	mfb->wq = alloc_ordered_workqueue("mtk-mfb", WQ_FREEZABLE);
	if (!mfb->wq)
		return -ENOMEM;
	ret = devm_add_action_or_reset(dev, mtk_mfb_wq_destroy, mfb->wq);
	if (ret)
		return ret;

	pm_runtime_set_autosuspend_delay(dev, MTK_MFB_AUTOSUSPEND_MS);
	pm_runtime_use_autosuspend(dev);
	ret = devm_pm_runtime_enable(dev);
	if (ret)
		return ret;

	ret = mtk_mfb_v4l2_register(mfb);
	if (ret)
		return dev_err_probe(dev, ret, "failed to register the nodes\n");

	return 0;
}

static void mtk_mfb_remove(struct platform_device *pdev)
{
	struct mtk_mfb *mfb = platform_get_drvdata(pdev);
	int ret;

	/* open files keep the nodes, but not the hardware or the runner */
	mtk_mfb_v4l2_unregister(mfb);
	flush_workqueue(mfb->wq);

	/* off now: the block can still be on from the last job */
	ret = pm_runtime_suspend(mfb->dev);
	if (ret < 0)
		dev_warn(mfb->dev, "block not off at the unbind: %d\n", ret);

	mtk_mfb_aux_free_all(mfb);
}

/*
 * System sleep: the job runner is a freezable workqueue, so no job runs
 * across the sleep; the block is off after it like after a long pause.
 */
static const struct dev_pm_ops mtk_mfb_pm_ops = {
	SYSTEM_SLEEP_PM_OPS(pm_runtime_force_suspend, pm_runtime_force_resume)
	RUNTIME_PM_OPS(mtk_mfb_runtime_suspend, mtk_mfb_runtime_resume, NULL)
};

static const struct of_device_id mtk_mfb_of_ids[] = {
	{ .compatible = "mediatek,mt6785-mfb" },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mtk_mfb_of_ids);

static struct platform_driver mtk_mfb_driver = {
	.probe = mtk_mfb_probe,
	.remove = mtk_mfb_remove,
	.driver = {
		.name = "mtk-mfb",
		.of_match_table = mtk_mfb_of_ids,
		.pm = pm_ptr(&mtk_mfb_pm_ops),
	},
};
module_platform_driver(mtk_mfb_driver);

MODULE_DESCRIPTION("MediaTek MT6785 multi-frame blending (MFB) driver");
MODULE_LICENSE("GPL");
