// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2015 MediaTek Inc.
 */

#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/kernel_stat.h>
#include <linux/mailbox_controller.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/pm_runtime.h>
#include <linux/seq_file.h>
#include <linux/soc/mediatek/mtk-cmdq.h>
#include <linux/soc/mediatek/mtk-mmsys.h>
#include <linux/soc/mediatek/mtk-mutex.h>

#include <asm/barrier.h>

#include <drm/drm_atomic.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_print.h>
#include <drm/drm_probe_helper.h>
#include <drm/drm_vblank.h>

#include "mtk_crtc.h"
#include "mtk_ddp_comp.h"
#include "mtk_disp_drv.h"
#include "mtk_drm_drv.h"
#include "mtk_plane.h"

/*
 * struct mtk_crtc - MediaTek specific crtc structure.
 * @base: crtc object.
 * @enabled: records whether crtc_enable succeeded
 * @planes: array of 4 drm_plane structures, one for each overlay plane
 * @pending_planes: whether any plane has pending changes to be applied
 * @mmsys_dev: pointer to the mmsys device for configuration registers
 * @mutex: handle to one of the ten disp_mutex streams
 * @ddp_comp_nr: number of components in ddp_comp
 * @ddp_comp: array of pointers the mtk_ddp_comp structures used by this crtc
 *
 * TODO: Needs update: this header is missing a bunch of member descriptions.
 */
#define MTK_VBL_SRC_TIMER	(-1)

/* register block the lisbon debugfs may read or write (component, mmsys, mutex) */
struct mtk_crtc_dbg_blk {
	const char		*name;
	phys_addr_t		base;
	resource_size_t		size;
	void __iomem		*regs;
};

struct mtk_crtc {
	struct drm_crtc			base;
	bool				enabled;

	bool				pending_needs_vblank;
	struct drm_pending_vblank_event	*event;

	struct drm_plane		*planes;
	unsigned int			layer_nr;
	bool				pending_planes;
	bool				pending_async_planes;

#if IS_REACHABLE(CONFIG_MTK_CMDQ)
	struct cmdq_client		cmdq_client;
	struct cmdq_pkt			cmdq_handle;
	u32				cmdq_event;
	u32				cmdq_vblank_cnt;
	wait_queue_head_t		cb_blocking_queue;
#endif

	struct device			*mmsys_dev;
	struct device			*dma_dev;
	struct mtk_mutex		*mutex;
	unsigned int			ddp_comp_nr;
	unsigned int			num_conn_routes;
	const struct mtk_drm_route	*conn_routes;

	/* lock for display hardware access */
	struct mutex			hw_lock;
	bool				config_updating;
	/* lock for config_updating to cmd buffer */
	spinlock_t			config_lock;

	/*
	 * vblank source (index into ddp_comp or MTK_VBL_SRC_TIMER), changed
	 * under dev->vbl_lock only; diagnostics counters and the fallback to
	 * the timer when the selected source stays silent.
	 */
	int				vbl_src;
	bool				vbl_on;
	atomic_t			vbl_hits;
	atomic_t			vbl_timer_hits;
	int				vbl_hits_enable;
	struct delayed_work		vbl_watch;

	/* lisbon diagnostics (debugfs): hw_ready = clocks on, path set up */
	struct mutex			dbg_lock;
	bool				dbg_hw_ready;
	unsigned int			dbg_nblk;
	struct mtk_crtc_dbg_blk		*dbg_blk;
	u32				dbg_rd_addr;
	u32				dbg_rd_val;
	int				dbg_rd_ret;

	struct mtk_ddp_comp		*ddp_comp[];
};

struct mtk_crtc_state {
	struct drm_crtc_state		base;

	bool				pending_config;
	unsigned int			pending_width;
	unsigned int			pending_height;
	unsigned int			pending_vrefresh;
};

static inline struct mtk_crtc *to_mtk_crtc(struct drm_crtc *c)
{
	return container_of(c, struct mtk_crtc, base);
}

static inline struct mtk_crtc_state *to_mtk_crtc_state(struct drm_crtc_state *s)
{
	return container_of(s, struct mtk_crtc_state, base);
}

static void mtk_crtc_finish_page_flip(struct mtk_crtc *mtk_crtc)
{
	struct drm_crtc *crtc = &mtk_crtc->base;
	unsigned long flags;

	if (mtk_crtc->event) {
		spin_lock_irqsave(&crtc->dev->event_lock, flags);
		drm_crtc_send_vblank_event(crtc, mtk_crtc->event);
		drm_crtc_vblank_put(crtc);
		mtk_crtc->event = NULL;
		spin_unlock_irqrestore(&crtc->dev->event_lock, flags);
	}
}

static void mtk_drm_finish_page_flip(struct mtk_crtc *mtk_crtc)
{
	unsigned long flags;

	drm_crtc_handle_vblank(&mtk_crtc->base);

#if IS_REACHABLE(CONFIG_MTK_CMDQ)
	if (mtk_crtc->cmdq_client.chan)
		return;
#endif

	spin_lock_irqsave(&mtk_crtc->config_lock, flags);
	if (!mtk_crtc->config_updating && mtk_crtc->pending_needs_vblank) {
		mtk_crtc_finish_page_flip(mtk_crtc);
		mtk_crtc->pending_needs_vblank = false;
	}
	spin_unlock_irqrestore(&mtk_crtc->config_lock, flags);
}

static void mtk_crtc_destroy(struct drm_crtc *crtc)
{
	struct mtk_crtc *mtk_crtc = to_mtk_crtc(crtc);
	int i;

	mtk_mutex_put(mtk_crtc->mutex);
#if IS_REACHABLE(CONFIG_MTK_CMDQ)
	if (mtk_crtc->cmdq_client.chan) {
		cmdq_pkt_destroy(&mtk_crtc->cmdq_client, &mtk_crtc->cmdq_handle);
		mbox_free_channel(mtk_crtc->cmdq_client.chan);
		mtk_crtc->cmdq_client.chan = NULL;
	}
#endif

	for (i = 0; i < mtk_crtc->ddp_comp_nr; i++) {
		struct mtk_ddp_comp *comp;

		comp = mtk_crtc->ddp_comp[i];
		mtk_ddp_comp_unregister_vblank_cb(comp);
	}

	drm_crtc_cleanup(crtc);
}

static void mtk_crtc_reset(struct drm_crtc *crtc)
{
	struct mtk_crtc_state *state;

	if (crtc->state) {
		__drm_atomic_helper_crtc_destroy_state(crtc->state);
		kfree(to_mtk_crtc_state(crtc->state));
	}
	crtc->state = NULL;

	state = kzalloc_obj(*state);
	if (state)
		__drm_atomic_helper_crtc_reset(crtc, &state->base);
}

static struct drm_crtc_state *mtk_crtc_duplicate_state(struct drm_crtc *crtc)
{
	struct mtk_crtc_state *state;

	state = kmalloc_obj(*state);
	if (!state)
		return NULL;

	__drm_atomic_helper_crtc_duplicate_state(crtc, &state->base);

	WARN_ON(state->base.crtc != crtc);
	state->base.crtc = crtc;
	state->pending_config = false;

	return &state->base;
}

static void mtk_crtc_destroy_state(struct drm_crtc *crtc,
				   struct drm_crtc_state *state)
{
	__drm_atomic_helper_crtc_destroy_state(state);
	kfree(to_mtk_crtc_state(state));
}

static enum drm_mode_status
mtk_crtc_mode_valid(struct drm_crtc *crtc, const struct drm_display_mode *mode)
{
	struct mtk_crtc *mtk_crtc = to_mtk_crtc(crtc);
	enum drm_mode_status status = MODE_OK;
	int i;

	for (i = 0; i < mtk_crtc->ddp_comp_nr; i++) {
		status = mtk_ddp_comp_mode_valid(mtk_crtc->ddp_comp[i], mode);
		if (status != MODE_OK)
			break;
	}
	return status;
}

static bool mtk_crtc_mode_fixup(struct drm_crtc *crtc,
				const struct drm_display_mode *mode,
				struct drm_display_mode *adjusted_mode)
{
	/* Nothing to do here, but this callback is mandatory. */
	return true;
}

static void mtk_crtc_mode_set_nofb(struct drm_crtc *crtc)
{
	struct mtk_crtc_state *state = to_mtk_crtc_state(crtc->state);

	state->pending_width = crtc->mode.hdisplay;
	state->pending_height = crtc->mode.vdisplay;
	state->pending_vrefresh = drm_mode_vrefresh(&crtc->mode);
	wmb();	/* Make sure the above parameters are set before update */
	state->pending_config = true;
}

static int mtk_crtc_ddp_clk_enable(struct mtk_crtc *mtk_crtc)
{
	struct drm_device *dev = mtk_crtc->base.dev;
	int ret;
	int i;

	for (i = 0; i < mtk_crtc->ddp_comp_nr; i++) {
		ret = mtk_ddp_comp_clk_enable(mtk_crtc->ddp_comp[i]);
		if (ret) {
			drm_err(dev, "Failed to enable clock %d: %d\n", i, ret);
			goto err;
		}
	}

	return 0;
err:
	while (--i >= 0)
		mtk_ddp_comp_clk_disable(mtk_crtc->ddp_comp[i]);
	return ret;
}

static void mtk_crtc_ddp_clk_disable(struct mtk_crtc *mtk_crtc)
{
	int i;

	for (i = 0; i < mtk_crtc->ddp_comp_nr; i++)
		mtk_ddp_comp_clk_disable(mtk_crtc->ddp_comp[i]);
}

static
struct mtk_ddp_comp *mtk_ddp_comp_for_plane(struct drm_crtc *crtc,
					    struct drm_plane *plane,
					    unsigned int *local_layer)
{
	struct mtk_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_ddp_comp *comp;
	int i, count = 0;
	unsigned int local_index = plane - mtk_crtc->planes;

	for (i = 0; i < mtk_crtc->ddp_comp_nr; i++) {
		comp = mtk_crtc->ddp_comp[i];
		if (local_index < (count + mtk_ddp_comp_layer_nr(comp))) {
			*local_layer = local_index - count;
			return comp;
		}
		count += mtk_ddp_comp_layer_nr(comp);
	}

	WARN(1, "Failed to find component for plane %d\n", plane->index);
	return NULL;
}

#if IS_REACHABLE(CONFIG_MTK_CMDQ)
static void ddp_cmdq_cb(struct mbox_client *cl, void *mssg)
{
	struct cmdq_cb_data *data = mssg;
	struct cmdq_client *cmdq_cl = container_of(cl, struct cmdq_client, client);
	struct mtk_crtc *mtk_crtc = container_of(cmdq_cl, struct mtk_crtc, cmdq_client);
	struct mtk_crtc_state *state;
	unsigned int i;
	unsigned long flags;

	/* release GCE HW usage and start autosuspend */
	pm_runtime_mark_last_busy(cmdq_cl->chan->mbox->dev);
	pm_runtime_put_autosuspend(cmdq_cl->chan->mbox->dev);

	if (data->sta < 0)
		return;

	state = to_mtk_crtc_state(mtk_crtc->base.state);

	spin_lock_irqsave(&mtk_crtc->config_lock, flags);
	if (mtk_crtc->config_updating)
		goto ddp_cmdq_cb_out;

	state->pending_config = false;

	if (mtk_crtc->pending_planes) {
		for (i = 0; i < mtk_crtc->layer_nr; i++) {
			struct drm_plane *plane = &mtk_crtc->planes[i];
			struct mtk_plane_state *plane_state;

			plane_state = to_mtk_plane_state(plane->state);

			plane_state->pending.config = false;
		}
		mtk_crtc->pending_planes = false;
	}

	if (mtk_crtc->pending_async_planes) {
		for (i = 0; i < mtk_crtc->layer_nr; i++) {
			struct drm_plane *plane = &mtk_crtc->planes[i];
			struct mtk_plane_state *plane_state;

			plane_state = to_mtk_plane_state(plane->state);

			plane_state->pending.async_config = false;
		}
		mtk_crtc->pending_async_planes = false;
	}

ddp_cmdq_cb_out:

	if (mtk_crtc->pending_needs_vblank) {
		mtk_crtc_finish_page_flip(mtk_crtc);
		mtk_crtc->pending_needs_vblank = false;
	}

	spin_unlock_irqrestore(&mtk_crtc->config_lock, flags);

	mtk_crtc->cmdq_vblank_cnt = 0;
	wake_up(&mtk_crtc->cb_blocking_queue);
}
#endif

/*
 * Takeover of a path a bootloader left scanning out (video mode, no CMDQ):
 * mutex, routing and engines are re-programmed by the CPU. Done in the middle
 * of a frame, the running frame breaks (OVL underflow, RDMA abnormal EOF,
 * DSI input unfinished) and the path may not start again. With takeover_sync
 * the re-programming waits for the vertical blanking of the output first.
 */
static bool takeover_sync = true;
module_param(takeover_sync, bool, 0644);
MODULE_PARM_DESC(takeover_sync,
		 "Re-program a path taken over from the bootloader in the vertical blanking");

static int mtk_crtc_ddp_hw_init(struct mtk_crtc *mtk_crtc)
{
	struct drm_crtc *crtc = &mtk_crtc->base;
	struct drm_connector *connector;
	struct drm_encoder *encoder;
	struct drm_connector_list_iter conn_iter;
	struct drm_device *dev = mtk_crtc->base.dev;
	unsigned int width, height, vrefresh, bpc = MTK_MAX_BPC;
	ktime_t sync_start = 0;
	int ret;
	int i;

	if (WARN_ON(!crtc->state))
		return -EINVAL;

	width = crtc->state->adjusted_mode.hdisplay;
	height = crtc->state->adjusted_mode.vdisplay;
	vrefresh = drm_mode_vrefresh(&crtc->state->adjusted_mode);

	drm_for_each_encoder(encoder, crtc->dev) {
		if (encoder->crtc != crtc)
			continue;

		drm_connector_list_iter_begin(crtc->dev, &conn_iter);
		drm_for_each_connector_iter(connector, &conn_iter) {
			if (connector->encoder != encoder)
				continue;
			if (connector->display_info.bpc != 0 &&
			    bpc > connector->display_info.bpc)
				bpc = connector->display_info.bpc;
		}
		drm_connector_list_iter_end(&conn_iter);
	}

	ret = pm_runtime_resume_and_get(crtc->dev->dev);
	if (ret < 0) {
		drm_err(dev, "Failed to enable power domain: %d\n", ret);
		return ret;
	}

	ret = mtk_mutex_prepare(mtk_crtc->mutex);
	if (ret < 0) {
		drm_err(dev, "Failed to enable mutex clock: %d\n", ret);
		goto err_pm_runtime_put;
	}

	ret = mtk_crtc_ddp_clk_enable(mtk_crtc);
	if (ret < 0) {
		drm_err(dev, "Failed to enable component clocks: %d\n", ret);
		goto err_mutex_unprepare;
	}

	if (takeover_sync &&
	    !mtk_ddp_comp_wait_blanking(mtk_crtc->ddp_comp[mtk_crtc->ddp_comp_nr - 1]))
		sync_start = ktime_get();

	for (i = 0; i < mtk_crtc->ddp_comp_nr - 1; i++) {
		if (!mtk_ddp_comp_connect(mtk_crtc->ddp_comp[i], mtk_crtc->mmsys_dev,
					  mtk_crtc->ddp_comp[i + 1]->id))
			mtk_mmsys_ddp_connect(mtk_crtc->mmsys_dev,
					      mtk_crtc->ddp_comp[i]->id,
					      mtk_crtc->ddp_comp[i + 1]->id);
		if (!mtk_ddp_comp_add(mtk_crtc->ddp_comp[i], mtk_crtc->mutex))
			mtk_mutex_add_comp(mtk_crtc->mutex,
					   mtk_crtc->ddp_comp[i]->id);
	}
	if (!mtk_ddp_comp_add(mtk_crtc->ddp_comp[i], mtk_crtc->mutex))
		mtk_mutex_add_comp(mtk_crtc->mutex, mtk_crtc->ddp_comp[i]->id);
	mtk_mutex_enable(mtk_crtc->mutex);

	for (i = 0; i < mtk_crtc->ddp_comp_nr; i++) {
		struct mtk_ddp_comp *comp = mtk_crtc->ddp_comp[i];

		if (i == 1)
			mtk_ddp_comp_bgclr_in_on(comp);

		mtk_ddp_comp_config(comp, width, height, vrefresh, bpc, NULL);
		mtk_ddp_comp_start(comp);
	}

	if (sync_start)
		drm_info(dev, "takeover: path re-programmed %lld us after the start of the blanking\n",
			 ktime_us_delta(ktime_get(), sync_start));

	/* Initially configure all planes */
	for (i = 0; i < mtk_crtc->layer_nr; i++) {
		struct drm_plane *plane = &mtk_crtc->planes[i];
		struct mtk_plane_state *plane_state;
		struct mtk_ddp_comp *comp;
		unsigned int local_layer;

		plane_state = to_mtk_plane_state(plane->state);

		/* should not enable layer before crtc enabled */
		plane_state->pending.enable = false;
		comp = mtk_ddp_comp_for_plane(crtc, plane, &local_layer);
		if (comp)
			mtk_ddp_comp_layer_config(comp, local_layer,
						  plane_state, NULL);
	}

	return 0;

err_mutex_unprepare:
	mtk_mutex_unprepare(mtk_crtc->mutex);
err_pm_runtime_put:
	pm_runtime_put(crtc->dev->dev);
	return ret;
}

static void mtk_crtc_ddp_hw_fini(struct mtk_crtc *mtk_crtc)
{
	struct drm_device *drm = mtk_crtc->base.dev;
	struct drm_crtc *crtc = &mtk_crtc->base;
	unsigned long flags;
	int i;

	for (i = 0; i < mtk_crtc->ddp_comp_nr; i++) {
		mtk_ddp_comp_stop(mtk_crtc->ddp_comp[i]);
		if (i == 1)
			mtk_ddp_comp_bgclr_in_off(mtk_crtc->ddp_comp[i]);
	}

	for (i = 0; i < mtk_crtc->ddp_comp_nr; i++)
		if (!mtk_ddp_comp_remove(mtk_crtc->ddp_comp[i], mtk_crtc->mutex))
			mtk_mutex_remove_comp(mtk_crtc->mutex,
					      mtk_crtc->ddp_comp[i]->id);
	mtk_mutex_disable(mtk_crtc->mutex);
	for (i = 0; i < mtk_crtc->ddp_comp_nr - 1; i++) {
		if (!mtk_ddp_comp_disconnect(mtk_crtc->ddp_comp[i], mtk_crtc->mmsys_dev,
					     mtk_crtc->ddp_comp[i + 1]->id))
			mtk_mmsys_ddp_disconnect(mtk_crtc->mmsys_dev,
						 mtk_crtc->ddp_comp[i]->id,
						 mtk_crtc->ddp_comp[i + 1]->id);
		if (!mtk_ddp_comp_remove(mtk_crtc->ddp_comp[i], mtk_crtc->mutex))
			mtk_mutex_remove_comp(mtk_crtc->mutex,
					      mtk_crtc->ddp_comp[i]->id);
	}
	if (!mtk_ddp_comp_remove(mtk_crtc->ddp_comp[i], mtk_crtc->mutex))
		mtk_mutex_remove_comp(mtk_crtc->mutex, mtk_crtc->ddp_comp[i]->id);
	mtk_crtc_ddp_clk_disable(mtk_crtc);
	mtk_mutex_unprepare(mtk_crtc->mutex);

	pm_runtime_put(drm->dev);

	if (crtc->state->event && !crtc->state->active) {
		spin_lock_irqsave(&crtc->dev->event_lock, flags);
		drm_crtc_send_vblank_event(crtc, crtc->state->event);
		crtc->state->event = NULL;
		spin_unlock_irqrestore(&crtc->dev->event_lock, flags);
	}
}

static void mtk_crtc_ddp_config(struct drm_crtc *crtc,
				struct cmdq_pkt *cmdq_handle)
{
	struct mtk_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_crtc_state *state = to_mtk_crtc_state(mtk_crtc->base.state);
	struct mtk_ddp_comp *comp = mtk_crtc->ddp_comp[0];
	unsigned int i;
	unsigned int local_layer;

	/*
	 * TODO: instead of updating the registers here, we should prepare
	 * working registers in atomic_commit and let the hardware command
	 * queue update module registers on vblank.
	 */
	if (state->pending_config) {
		mtk_ddp_comp_config(comp, state->pending_width,
				    state->pending_height,
				    state->pending_vrefresh, 0,
				    cmdq_handle);

		if (!cmdq_handle)
			state->pending_config = false;
	}

	if (mtk_crtc->pending_planes) {
		for (i = 0; i < mtk_crtc->layer_nr; i++) {
			struct drm_plane *plane = &mtk_crtc->planes[i];
			struct mtk_plane_state *plane_state;

			plane_state = to_mtk_plane_state(plane->state);

			if (!plane_state->pending.config)
				continue;

			comp = mtk_ddp_comp_for_plane(crtc, plane, &local_layer);

			if (comp)
				mtk_ddp_comp_layer_config(comp, local_layer,
							  plane_state,
							  cmdq_handle);
			if (!cmdq_handle)
				plane_state->pending.config = false;
		}

		if (!cmdq_handle)
			mtk_crtc->pending_planes = false;
	}

	if (mtk_crtc->pending_async_planes) {
		for (i = 0; i < mtk_crtc->layer_nr; i++) {
			struct drm_plane *plane = &mtk_crtc->planes[i];
			struct mtk_plane_state *plane_state;

			plane_state = to_mtk_plane_state(plane->state);

			if (!plane_state->pending.async_config)
				continue;

			comp = mtk_ddp_comp_for_plane(crtc, plane, &local_layer);

			if (comp)
				mtk_ddp_comp_layer_config(comp, local_layer,
							  plane_state,
							  cmdq_handle);
			if (!cmdq_handle)
				plane_state->pending.async_config = false;
		}

		if (!cmdq_handle)
			mtk_crtc->pending_async_planes = false;
	}
}

static void mtk_crtc_update_config(struct mtk_crtc *mtk_crtc, bool needs_vblank)
{
#if IS_REACHABLE(CONFIG_MTK_CMDQ)
	struct cmdq_pkt *cmdq_handle = &mtk_crtc->cmdq_handle;
#endif
	struct drm_crtc *crtc = &mtk_crtc->base;
	struct mtk_drm_private *priv = crtc->dev->dev_private;
	unsigned int pending_planes = 0, pending_async_planes = 0;
	int i;
	unsigned long flags;

	mutex_lock(&mtk_crtc->hw_lock);

	spin_lock_irqsave(&mtk_crtc->config_lock, flags);
	mtk_crtc->config_updating = true;
	spin_unlock_irqrestore(&mtk_crtc->config_lock, flags);

	if (needs_vblank)
		mtk_crtc->pending_needs_vblank = true;

	for (i = 0; i < mtk_crtc->layer_nr; i++) {
		struct drm_plane *plane = &mtk_crtc->planes[i];
		struct mtk_plane_state *plane_state;

		plane_state = to_mtk_plane_state(plane->state);
		if (plane_state->pending.dirty) {
			plane_state->pending.config = true;
			plane_state->pending.dirty = false;
			pending_planes |= BIT(i);
		} else if (plane_state->pending.async_dirty) {
			plane_state->pending.async_config = true;
			plane_state->pending.async_dirty = false;
			pending_async_planes |= BIT(i);
		}
	}
	if (pending_planes)
		mtk_crtc->pending_planes = true;
	if (pending_async_planes)
		mtk_crtc->pending_async_planes = true;

	if (priv->data->shadow_register) {
		mtk_mutex_acquire(mtk_crtc->mutex);
		mtk_crtc_ddp_config(crtc, NULL);
		mtk_mutex_release(mtk_crtc->mutex);
	}
#if IS_REACHABLE(CONFIG_MTK_CMDQ)
	if (mtk_crtc->cmdq_client.chan) {
		mbox_flush(mtk_crtc->cmdq_client.chan, 2000);
		cmdq_handle->cmd_buf_size = 0;
		cmdq_pkt_clear_event(cmdq_handle, mtk_crtc->cmdq_event);
		cmdq_pkt_wfe(cmdq_handle, mtk_crtc->cmdq_event, false);
		mtk_crtc_ddp_config(crtc, cmdq_handle);
		cmdq_pkt_eoc(cmdq_handle);
		dma_sync_single_for_device(mtk_crtc->cmdq_client.chan->mbox->dev,
					   cmdq_handle->pa_base,
					   cmdq_handle->cmd_buf_size,
					   DMA_TO_DEVICE);
		/*
		 * CMDQ command should execute in next 3 vblank.
		 * One vblank interrupt before send message (occasionally)
		 * and one vblank interrupt after cmdq done,
		 * so it's timeout after 3 vblank interrupt.
		 * If it fail to execute in next 3 vblank, timeout happen.
		 */
		mtk_crtc->cmdq_vblank_cnt = 3;

		spin_lock_irqsave(&mtk_crtc->config_lock, flags);
		mtk_crtc->config_updating = false;
		spin_unlock_irqrestore(&mtk_crtc->config_lock, flags);

		if (pm_runtime_resume_and_get(mtk_crtc->cmdq_client.chan->mbox->dev) < 0)
			goto update_config_out;

		mbox_send_message(mtk_crtc->cmdq_client.chan, cmdq_handle);
		mbox_client_txdone(mtk_crtc->cmdq_client.chan, 0);
		goto update_config_out;
	}
#endif
	spin_lock_irqsave(&mtk_crtc->config_lock, flags);
	mtk_crtc->config_updating = false;
	spin_unlock_irqrestore(&mtk_crtc->config_lock, flags);

#if IS_REACHABLE(CONFIG_MTK_CMDQ)
update_config_out:
#endif
	mutex_unlock(&mtk_crtc->hw_lock);
}

static void mtk_crtc_ddp_irq(void *data)
{
	struct drm_crtc *crtc = data;
	struct mtk_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_drm_private *priv = crtc->dev->dev_private;

	atomic_inc(&mtk_crtc->vbl_hits);

#if IS_REACHABLE(CONFIG_MTK_CMDQ)
	struct drm_device *dev = mtk_crtc->base.dev;
	if (!priv->data->shadow_register && !mtk_crtc->cmdq_client.chan)
		mtk_crtc_ddp_config(crtc, NULL);
	else if (mtk_crtc->cmdq_vblank_cnt > 0 && --mtk_crtc->cmdq_vblank_cnt == 0)
		drm_err(dev, "mtk_crtc %d CMDQ execute command timeout!\n",
			drm_crtc_index(&mtk_crtc->base));
#else
	if (!priv->data->shadow_register)
		mtk_crtc_ddp_config(crtc, NULL);
#endif
	mtk_drm_finish_page_flip(mtk_crtc);
}

/*
 * vblank source. Upstream: the first component (OVL0 frame done). For a
 * path taken over from a bootloader the frame interrupt may never come;
 * vblank_src selects another component (index in the path, e.g. RDMA or the
 * DSI front porch) or a timer (-1), vblank_fallback_ms switches to the timer
 * when the selected source stays silent after an enable, so that commits and
 * DPMS do not hang in their 10 s timeouts.
 */
static int vblank_src;
module_param(vblank_src, int, 0644);
MODULE_PARM_DESC(vblank_src, "vblank source: index of the path component (0 = first), -1 = timer");

static unsigned int vblank_fallback_ms = 1000;
module_param(vblank_fallback_ms, uint, 0644);
MODULE_PARM_DESC(vblank_fallback_ms, "Switch to the vblank timer if the source stays silent this long after enable, 0 = never");

static bool mtk_crtc_handle_vblank_timeout(struct drm_crtc *crtc)
{
	struct mtk_crtc *mtk_crtc = to_mtk_crtc(crtc);

	atomic_inc(&mtk_crtc->vbl_timer_hits);
	mtk_crtc_ddp_irq(crtc);
	return true;
}

static void mtk_crtc_vbl_src_on(struct mtk_crtc *mtk_crtc, int src)
{
	if (src == MTK_VBL_SRC_TIMER)
		drm_crtc_vblank_start_timer(&mtk_crtc->base);
	else
		mtk_ddp_comp_enable_vblank(mtk_crtc->ddp_comp[src]);
}

static void mtk_crtc_vbl_src_off(struct mtk_crtc *mtk_crtc, int src)
{
	if (src == MTK_VBL_SRC_TIMER)
		drm_crtc_vblank_cancel_timer(&mtk_crtc->base);
	else
		mtk_ddp_comp_disable_vblank(mtk_crtc->ddp_comp[src]);
}

static const char *mtk_crtc_vbl_src_name(struct mtk_crtc *mtk_crtc, int src)
{
	if (src == MTK_VBL_SRC_TIMER)
		return "timer";
	return dev_name(mtk_crtc->ddp_comp[src]->dev);
}

/* needs the path powered (hw_ready or within enable/disable) */
static void mtk_crtc_vbl_switch(struct mtk_crtc *mtk_crtc, int src)
{
	struct drm_device *dev = mtk_crtc->base.dev;
	unsigned long flags;

	spin_lock_irqsave(&dev->vbl_lock, flags);
	if (src != mtk_crtc->vbl_src) {
		if (mtk_crtc->vbl_on) {
			mtk_crtc_vbl_src_off(mtk_crtc, mtk_crtc->vbl_src);
			mtk_crtc_vbl_src_on(mtk_crtc, src);
		}
		mtk_crtc->vbl_src = src;
	}
	spin_unlock_irqrestore(&dev->vbl_lock, flags);
}

static void mtk_crtc_vbl_watch(struct work_struct *work)
{
	struct mtk_crtc *mtk_crtc = container_of(to_delayed_work(work),
						 struct mtk_crtc, vbl_watch);
	int src;

	mutex_lock(&mtk_crtc->dbg_lock);
	src = mtk_crtc->vbl_src;
	if (mtk_crtc->dbg_hw_ready && src != MTK_VBL_SRC_TIMER &&
	    READ_ONCE(mtk_crtc->vbl_on) &&
	    atomic_read(&mtk_crtc->vbl_hits) == mtk_crtc->vbl_hits_enable) {
		drm_warn(mtk_crtc->base.dev,
			 "vblank: no interrupt from %s within %u ms after enable, using the timer\n",
			 mtk_crtc_vbl_src_name(mtk_crtc, src), vblank_fallback_ms);
		mtk_crtc_vbl_switch(mtk_crtc, MTK_VBL_SRC_TIMER);
	}
	mutex_unlock(&mtk_crtc->dbg_lock);
}

static int mtk_crtc_enable_vblank(struct drm_crtc *crtc)
{
	struct mtk_crtc *mtk_crtc = to_mtk_crtc(crtc);

	/* called with dev->vbl_lock held */
	mtk_crtc->vbl_on = true;
	mtk_crtc_vbl_src_on(mtk_crtc, mtk_crtc->vbl_src);

	return 0;
}

static void mtk_crtc_disable_vblank(struct drm_crtc *crtc)
{
	struct mtk_crtc *mtk_crtc = to_mtk_crtc(crtc);

	/* called with dev->vbl_lock held */
	mtk_crtc_vbl_src_off(mtk_crtc, mtk_crtc->vbl_src);
	mtk_crtc->vbl_on = false;
}

static void mtk_crtc_update_output(struct drm_crtc *crtc,
				   struct drm_atomic_commit *state)
{
	int crtc_index = drm_crtc_index(crtc);
	int i;
	struct device *dev;
	struct drm_crtc_state *crtc_state = state->crtcs[crtc_index].new_state;
	struct mtk_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_drm_private *priv;
	unsigned int encoder_mask = crtc_state->encoder_mask;

	if (!crtc_state->connectors_changed)
		return;

	if (!mtk_crtc->num_conn_routes)
		return;

	priv = ((struct mtk_drm_private *)crtc->dev->dev_private)->all_drm_private[crtc_index];
	dev = priv->dev;

	dev_dbg(dev, "connector change:%d, encoder mask:0x%x for crtc:%d\n",
		crtc_state->connectors_changed, encoder_mask, crtc_index);

	for (i = 0; i < mtk_crtc->num_conn_routes; i++) {
		unsigned int comp_id = mtk_crtc->conn_routes[i].route_ddp;
		struct mtk_ddp_comp *comp = &priv->ddp_comp[comp_id];

		if (comp->encoder_index >= 0 &&
		    (encoder_mask & BIT(comp->encoder_index))) {
			mtk_crtc->ddp_comp[mtk_crtc->ddp_comp_nr - 1] = comp;
			dev_dbg(dev, "Add comp_id: %d at path index %d\n",
				comp->id, mtk_crtc->ddp_comp_nr - 1);
			break;
		}
	}
}

int mtk_crtc_plane_check(struct drm_crtc *crtc, struct drm_plane *plane,
			 struct mtk_plane_state *state)
{
	unsigned int local_layer;
	struct mtk_ddp_comp *comp;

	comp = mtk_ddp_comp_for_plane(crtc, plane, &local_layer);
	if (comp)
		return mtk_ddp_comp_layer_check(comp, local_layer, state);
	return 0;
}

void mtk_crtc_plane_disable(struct drm_crtc *crtc, struct drm_plane *plane)
{
#if IS_REACHABLE(CONFIG_MTK_CMDQ)
	struct mtk_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_plane_state *plane_state = to_mtk_plane_state(plane->state);
	int i;

	/* no need to wait for disabling the plane by CPU */
	if (!mtk_crtc->cmdq_client.chan)
		return;

	if (!mtk_crtc->enabled)
		return;

	/* set pending plane state to disabled */
	for (i = 0; i < mtk_crtc->layer_nr; i++) {
		struct drm_plane *mtk_plane = &mtk_crtc->planes[i];
		struct mtk_plane_state *mtk_plane_state = to_mtk_plane_state(mtk_plane->state);

		if (mtk_plane->index == plane->index) {
			memcpy(mtk_plane_state, plane_state, sizeof(*plane_state));
			break;
		}
	}
	mtk_crtc_update_config(mtk_crtc, false);

	/* wait for planes to be disabled by CMDQ */
	wait_event_timeout(mtk_crtc->cb_blocking_queue,
			   mtk_crtc->cmdq_vblank_cnt == 0,
			   msecs_to_jiffies(500));
#endif
}

void mtk_crtc_async_update(struct drm_crtc *crtc, struct drm_plane *plane,
			   struct drm_atomic_commit *state)
{
	struct mtk_crtc *mtk_crtc = to_mtk_crtc(crtc);

	if (!mtk_crtc->enabled)
		return;

	mtk_crtc_update_config(mtk_crtc, false);
}

static void mtk_crtc_atomic_enable(struct drm_crtc *crtc,
				   struct drm_atomic_commit *state)
{
	struct mtk_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_ddp_comp *comp = mtk_crtc->ddp_comp[0];
	struct drm_device *dev = mtk_crtc->base.dev;
	unsigned int fallback_ms;
	int ret, src;

	drm_dbg_driver(dev, "%s %d\n", __func__, crtc->base.id);

	ret = mtk_ddp_comp_power_on(comp);
	if (ret < 0) {
		DRM_DEV_ERROR(comp->dev, "Failed to enable power domain: %d\n", ret);
		return;
	}

	mtk_crtc_update_output(crtc, state);

	ret = mtk_crtc_ddp_hw_init(mtk_crtc);
	if (ret) {
		mtk_ddp_comp_power_off(comp);
		return;
	}

	/* vblank is off here: no vbl_lock needed to pick the source */
	src = READ_ONCE(vblank_src);
	if (src < MTK_VBL_SRC_TIMER || src >= (int)mtk_crtc->ddp_comp_nr)
		src = 0;
	mtk_crtc->vbl_src = src;
	mtk_crtc->vbl_hits_enable = atomic_read(&mtk_crtc->vbl_hits);

	drm_crtc_vblank_on(crtc);
	mtk_crtc->enabled = true;

	mutex_lock(&mtk_crtc->dbg_lock);
	mtk_crtc->dbg_hw_ready = true;
	mutex_unlock(&mtk_crtc->dbg_lock);

	fallback_ms = READ_ONCE(vblank_fallback_ms);
	if (fallback_ms && src != MTK_VBL_SRC_TIMER)
		schedule_delayed_work(&mtk_crtc->vbl_watch, msecs_to_jiffies(fallback_ms));
}

static void mtk_crtc_atomic_disable(struct drm_crtc *crtc,
				    struct drm_atomic_commit *state)
{
	struct mtk_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_ddp_comp *comp = mtk_crtc->ddp_comp[0];
	struct drm_device *dev = mtk_crtc->base.dev;
	int i;

	drm_dbg_driver(dev, "%s %d\n", __func__, crtc->base.id);
	if (!mtk_crtc->enabled)
		return;

	/* no diagnostics on the path from here on (clocks go off below) */
	cancel_delayed_work_sync(&mtk_crtc->vbl_watch);
	mutex_lock(&mtk_crtc->dbg_lock);
	mtk_crtc->dbg_hw_ready = false;
	mutex_unlock(&mtk_crtc->dbg_lock);

	/* Set all pending plane state to disabled */
	for (i = 0; i < mtk_crtc->layer_nr; i++) {
		struct drm_plane *plane = &mtk_crtc->planes[i];
		struct mtk_plane_state *plane_state;

		plane_state = to_mtk_plane_state(plane->state);
		plane_state->pending.enable = false;
		plane_state->pending.config = true;
	}
	mtk_crtc->pending_planes = true;

	mtk_crtc_update_config(mtk_crtc, false);
#if IS_REACHABLE(CONFIG_MTK_CMDQ)
	/* Wait for planes to be disabled by cmdq */
	if (mtk_crtc->cmdq_client.chan)
		wait_event_timeout(mtk_crtc->cb_blocking_queue,
				   mtk_crtc->cmdq_vblank_cnt == 0,
				   msecs_to_jiffies(500));
#endif
	/* Wait for planes to be disabled */
	drm_crtc_wait_one_vblank(crtc);

	drm_crtc_vblank_off(crtc);
	/* a timer callback may still run on another CPU: wait for it */
	if (drm_crtc_vblank_crtc(crtc)->vblank_timer.crtc)
		hrtimer_cancel(&drm_crtc_vblank_crtc(crtc)->vblank_timer.timer);
	mtk_crtc_ddp_hw_fini(mtk_crtc);
	mtk_ddp_comp_power_off(comp);

	mtk_crtc->enabled = false;
}

static void mtk_crtc_atomic_begin(struct drm_crtc *crtc,
				  struct drm_atomic_commit *state)
{
	struct drm_crtc_state *crtc_state = drm_atomic_get_new_crtc_state(state,
									  crtc);
	struct mtk_crtc_state *mtk_crtc_state = to_mtk_crtc_state(crtc_state);
	struct mtk_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct drm_device *dev = mtk_crtc->base.dev;
	unsigned long flags;

	if (mtk_crtc->event && mtk_crtc_state->base.event)
		drm_err(dev, "new event while there is still a pending event\n");

	if (mtk_crtc_state->base.event) {
		mtk_crtc_state->base.event->pipe = drm_crtc_index(crtc);
		WARN_ON(drm_crtc_vblank_get(crtc) != 0);

		spin_lock_irqsave(&crtc->dev->event_lock, flags);
		mtk_crtc->event = mtk_crtc_state->base.event;
		spin_unlock_irqrestore(&crtc->dev->event_lock, flags);

		mtk_crtc_state->base.event = NULL;
	}
}

static void mtk_crtc_atomic_flush(struct drm_crtc *crtc,
				  struct drm_atomic_commit *state)
{
	struct mtk_crtc *mtk_crtc = to_mtk_crtc(crtc);
	int i;

	if (crtc->state->color_mgmt_changed)
		for (i = 0; i < mtk_crtc->ddp_comp_nr; i++) {
			mtk_ddp_gamma_set(mtk_crtc->ddp_comp[i], crtc->state);
			mtk_ddp_ctm_set(mtk_crtc->ddp_comp[i], crtc->state);
		}
	mtk_crtc_update_config(mtk_crtc, !!mtk_crtc->event);
}

#ifdef CONFIG_DEBUG_FS
/*
 * lisbon diagnostics: <debugfs>/dri/N/crtc-0/lisbon_status (read) and
 * lisbon_action (write one command). Hardware is only touched while the path
 * is set up (dbg_hw_ready); register writes additionally need reg_writes=1.
 * No modeset lock is taken, so the actions also work while a commit waits
 * for a vblank that never comes.
 */
static bool reg_writes;
module_param(reg_writes, bool, 0644);
MODULE_PARM_DESC(reg_writes, "Allow register writes through the lisbon debugfs (diagnostics)");

static void mtk_crtc_dbg_add_blk(struct mtk_crtc *mtk_crtc, struct device_node *np,
				 const char *name)
{
	struct mtk_crtc_dbg_blk *b;
	struct resource res;

	if (!np || of_address_to_resource(np, 0, &res))
		return;
	b = &mtk_crtc->dbg_blk[mtk_crtc->dbg_nblk];
	b->regs = of_iomap(np, 0);
	if (!b->regs)
		return;
	b->name = name;
	b->base = res.start;
	b->size = resource_size(&res);
	mtk_crtc->dbg_nblk++;
}

static void __iomem *mtk_crtc_dbg_reg(struct mtk_crtc *mtk_crtc, u32 addr)
{
	unsigned int i;

	if (addr & 3)
		return NULL;
	for (i = 0; i < mtk_crtc->dbg_nblk; i++) {
		struct mtk_crtc_dbg_blk *b = &mtk_crtc->dbg_blk[i];

		if (addr >= b->base && addr - b->base < b->size)
			return b->regs + (addr - b->base);
	}
	return NULL;
}

static void mtk_crtc_dbg_irq(struct seq_file *m, const char *name, struct device_node *np)
{
	bool pending = false, active = false, masked = false;
	struct irq_data *d;
	int irq, rp, ra, rm;

	irq = np ? of_irq_get(np, 0) : -ENOENT;
	if (irq <= 0) {
		seq_printf(m, "  irq %-16s none\n", name);
		return;
	}
	d = irq_get_irq_data(irq);
	rp = irq_get_irqchip_state(irq, IRQCHIP_STATE_PENDING, &pending);
	ra = irq_get_irqchip_state(irq, IRQCHIP_STATE_ACTIVE, &active);
	rm = irq_get_irqchip_state(irq, IRQCHIP_STATE_MASKED, &masked);
	seq_printf(m, "  irq %-16s virq %d hwirq %lu count %u type 0x%x disabled %d masked %d chip: pending %c active %c masked %c\n",
		   name, irq, d ? irqd_to_hwirq(d) : 0, kstat_irqs_usr(irq),
		   d ? irqd_get_trigger_type(d) : 0,
		   d ? irqd_irq_disabled(d) : -1, d ? irqd_irq_masked(d) : -1,
		   rp ? '?' : '0' + pending, ra ? '?' : '0' + active,
		   rm ? '?' : '0' + masked);
}

static int mtk_crtc_dbg_status_show(struct seq_file *m, void *unused)
{
	struct mtk_crtc *mtk_crtc = m->private;
	struct drm_crtc *crtc = &mtk_crtc->base;
	struct drm_vblank_crtc *vblank = drm_crtc_vblank_crtc(crtc);
	unsigned int i;

	mutex_lock(&mtk_crtc->dbg_lock);
	seq_printf(m, "enabled %d hw_ready %d vbl_src %d (%s) vbl_on %d hits %d (at enable %d) timer_hits %d\n",
		   mtk_crtc->enabled, mtk_crtc->dbg_hw_ready, mtk_crtc->vbl_src,
		   mtk_crtc_vbl_src_name(mtk_crtc, mtk_crtc->vbl_src),
		   READ_ONCE(mtk_crtc->vbl_on), atomic_read(&mtk_crtc->vbl_hits),
		   mtk_crtc->vbl_hits_enable, atomic_read(&mtk_crtc->vbl_timer_hits));
	seq_printf(m, "drm vblank: count %llu refcount %d enabled %d\n",
		   drm_crtc_vblank_count(crtc), atomic_read(&vblank->refcount),
		   READ_ONCE(vblank->enabled));
	seq_printf(m, "pending: planes %d async %d needs_vblank %d event %d config_updating %d\n",
		   mtk_crtc->pending_planes, mtk_crtc->pending_async_planes,
		   mtk_crtc->pending_needs_vblank, !!mtk_crtc->event,
		   mtk_crtc->config_updating);
	seq_printf(m, "params: vblank_src %d vblank_fallback_ms %u reg_writes %d\n",
		   READ_ONCE(vblank_src), READ_ONCE(vblank_fallback_ms), READ_ONCE(reg_writes));
	for (i = 0; i < mtk_crtc->ddp_comp_nr; i++) {
		struct mtk_ddp_comp *comp = mtk_crtc->ddp_comp[i];

		seq_printf(m, "comp %u %s\n", i, comp->dev ? dev_name(comp->dev) : "-");
		if (comp->dev)
			mtk_crtc_dbg_irq(m, dev_name(comp->dev), comp->dev->of_node);
	}
	for (i = 0; i < mtk_crtc->layer_nr; i++) {
		struct drm_plane *plane = &mtk_crtc->planes[i];
		struct mtk_plane_state *ps;

		if (!plane->state)
			continue;
		ps = to_mtk_plane_state(plane->state);
		seq_printf(m, "plane %u: fb %d enable %d config %d dirty %d addr %pad pitch %u fmt %p4cc %ux%u+%u+%u\n",
			   i, !!plane->state->fb, ps->pending.enable, ps->pending.config,
			   ps->pending.dirty, &ps->pending.addr, ps->pending.pitch,
			   &ps->pending.format, ps->pending.width, ps->pending.height,
			   ps->pending.x, ps->pending.y);
	}
	for (i = 0; i < mtk_crtc->dbg_nblk; i++)
		seq_printf(m, "block %s %pa size 0x%llx\n", mtk_crtc->dbg_blk[i].name,
			   &mtk_crtc->dbg_blk[i].base, (u64)mtk_crtc->dbg_blk[i].size);
	if (mtk_crtc->dbg_rd_addr)
		seq_printf(m, "last rd 0x%08x = 0x%08x (%d)\n", mtk_crtc->dbg_rd_addr,
			   mtk_crtc->dbg_rd_val, mtk_crtc->dbg_rd_ret);
	mutex_unlock(&mtk_crtc->dbg_lock);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(mtk_crtc_dbg_status);

static int mtk_crtc_dbg_parse_src(struct mtk_crtc *mtk_crtc, const char *arg, int *src)
{
	unsigned int i;

	if (!strcmp(arg, "timer")) {
		*src = MTK_VBL_SRC_TIMER;
		return 0;
	}
	if (!strcmp(arg, "dsi")) {
		*src = mtk_crtc->ddp_comp_nr - 1;
		return 0;
	}
	if (!strcmp(arg, "rdma")) {
		for (i = 0; i < mtk_crtc->ddp_comp_nr; i++)
			if (mtk_crtc->ddp_comp[i]->dev &&
			    strstr(dev_name(mtk_crtc->ddp_comp[i]->dev), ".rdma")) {
				*src = i;
				return 0;
			}
		return -ENOENT;
	}
	if (kstrtoint(arg, 0, src) || *src < 0 || *src >= (int)mtk_crtc->ddp_comp_nr)
		return -EINVAL;
	return 0;
}

static int mtk_crtc_dbg_action(struct mtk_crtc *mtk_crtc, char *cmd)
{
	struct drm_crtc *crtc = &mtk_crtc->base;
	char *arg1, *arg2, *p = strim(cmd);
	unsigned long flags;
	void __iomem *reg;
	unsigned int i;
	u32 addr = 0, val = 0;
	int ret = 0, src;

	arg1 = strsep(&p, " ");
	arg2 = p ? strsep(&p, " ") : NULL;

	if (!mtk_crtc->dbg_hw_ready)
		return -EAGAIN;

	if (!strcmp(arg1, "apply")) {
		/* one software vblank: pending plane config + events */
		local_irq_save(flags);
		mtk_crtc_ddp_irq(crtc);
		local_irq_restore(flags);
	} else if (!strcmp(arg1, "config")) {
		local_irq_save(flags);
		mtk_crtc_ddp_config(crtc, NULL);
		local_irq_restore(flags);
	} else if (!strcmp(arg1, "flip")) {
		mtk_drm_finish_page_flip(mtk_crtc);
	} else if (!strcmp(arg1, "src") && arg2) {
		ret = mtk_crtc_dbg_parse_src(mtk_crtc, arg2, &src);
		if (!ret) {
			mtk_crtc_vbl_switch(mtk_crtc, src);
			drm_info(crtc->dev, "vblank source: %s (debugfs)\n",
				 mtk_crtc_vbl_src_name(mtk_crtc, src));
		}
	} else if (!strcmp(arg1, "lc_off")) {
		ret = -ENODEV;
		for (i = 0; i < mtk_crtc->ddp_comp_nr; i++) {
			struct device *cdev = mtk_crtc->ddp_comp[i]->dev;

			if (cdev && (of_device_is_compatible(cdev->of_node, "mediatek,mt6785-disp-ovl") ||
				     of_device_is_compatible(cdev->of_node, "mediatek,mt6785-disp-ovl-2l")))
				ret = mtk_ovl_lc_off(cdev);
		}
	} else if (!strcmp(arg1, "rd") && arg2) {
		ret = kstrtou32(arg2, 0, &addr);
		reg = ret ? NULL : mtk_crtc_dbg_reg(mtk_crtc, addr);
		if (!ret && !reg)
			ret = -ERANGE;
		mtk_crtc->dbg_rd_addr = addr;
		mtk_crtc->dbg_rd_val = reg ? readl(reg) : 0;
		mtk_crtc->dbg_rd_ret = ret;
	} else if (!strcmp(arg1, "wr") && arg2 && p) {
		if (!READ_ONCE(reg_writes))
			return -EPERM;
		ret = kstrtou32(arg2, 0, &addr);
		if (!ret)
			ret = kstrtou32(strim(p), 0, &val);
		reg = ret ? NULL : mtk_crtc_dbg_reg(mtk_crtc, addr);
		if (!ret && !reg)
			ret = -ERANGE;
		if (!ret) {
			drm_info(crtc->dev, "debugfs: write 0x%08x = 0x%08x (was 0x%08x)\n",
				 addr, val, readl(reg));
			writel(val, reg);
		}
	} else {
		ret = -EINVAL;
	}
	return ret;
}

static ssize_t mtk_crtc_dbg_action_write(struct file *file, const char __user *ubuf,
					 size_t len, loff_t *ppos)
{
	struct mtk_crtc *mtk_crtc = file_inode(file)->i_private;
	char buf[64];
	int ret;

	if (len >= sizeof(buf))
		return -EINVAL;
	if (copy_from_user(buf, ubuf, len))
		return -EFAULT;
	buf[len] = 0;

	mutex_lock(&mtk_crtc->dbg_lock);
	ret = mtk_crtc_dbg_action(mtk_crtc, buf);
	mutex_unlock(&mtk_crtc->dbg_lock);

	return ret ? ret : len;
}

static const struct file_operations mtk_crtc_dbg_action_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.write = mtk_crtc_dbg_action_write,
	.llseek = noop_llseek,
};

static int mtk_crtc_late_register(struct drm_crtc *crtc)
{
	struct mtk_crtc *mtk_crtc = to_mtk_crtc(crtc);
	struct mtk_drm_private *priv = crtc->dev->dev_private;
	unsigned int i;

	if (!crtc->debugfs_entry)
		return 0;

	mtk_crtc->dbg_blk = devm_kcalloc(crtc->dev->dev, mtk_crtc->ddp_comp_nr + 2,
					 sizeof(*mtk_crtc->dbg_blk), GFP_KERNEL);
	if (mtk_crtc->dbg_blk) {
		for (i = 0; i < mtk_crtc->ddp_comp_nr; i++)
			if (mtk_crtc->ddp_comp[i]->dev)
				mtk_crtc_dbg_add_blk(mtk_crtc, mtk_crtc->ddp_comp[i]->dev->of_node,
						     dev_name(mtk_crtc->ddp_comp[i]->dev));
		mtk_crtc_dbg_add_blk(mtk_crtc, mtk_crtc->mmsys_dev->of_node, "mmsys");
		mtk_crtc_dbg_add_blk(mtk_crtc, priv->mutex_node, "mutex");
	}

	debugfs_create_file("lisbon_status", 0444, crtc->debugfs_entry, mtk_crtc,
			    &mtk_crtc_dbg_status_fops);
	debugfs_create_file("lisbon_action", 0200, crtc->debugfs_entry, mtk_crtc,
			    &mtk_crtc_dbg_action_fops);
	return 0;
}
#else
static int mtk_crtc_late_register(struct drm_crtc *crtc)
{
	return 0;
}
#endif

static const struct drm_crtc_funcs mtk_crtc_funcs = {
	.set_config		= drm_atomic_helper_set_config,
	.page_flip		= drm_atomic_helper_page_flip,
	.destroy		= mtk_crtc_destroy,
	.reset			= mtk_crtc_reset,
	.atomic_duplicate_state	= mtk_crtc_duplicate_state,
	.atomic_destroy_state	= mtk_crtc_destroy_state,
	.enable_vblank		= mtk_crtc_enable_vblank,
	.disable_vblank		= mtk_crtc_disable_vblank,
	.late_register		= mtk_crtc_late_register,
};

static const struct drm_crtc_helper_funcs mtk_crtc_helper_funcs = {
	.mode_fixup	= mtk_crtc_mode_fixup,
	.mode_set_nofb	= mtk_crtc_mode_set_nofb,
	.mode_valid	= mtk_crtc_mode_valid,
	.atomic_begin	= mtk_crtc_atomic_begin,
	.atomic_flush	= mtk_crtc_atomic_flush,
	.atomic_enable	= mtk_crtc_atomic_enable,
	.atomic_disable	= mtk_crtc_atomic_disable,
	.handle_vblank_timeout = mtk_crtc_handle_vblank_timeout,
};

static int mtk_crtc_init(struct drm_device *drm, struct mtk_crtc *mtk_crtc,
			 unsigned int pipe)
{
	struct drm_plane *primary = NULL;
	struct drm_plane *cursor = NULL;
	int i, ret;

	for (i = 0; i < mtk_crtc->layer_nr; i++) {
		if (mtk_crtc->planes[i].type == DRM_PLANE_TYPE_PRIMARY)
			primary = &mtk_crtc->planes[i];
		else if (mtk_crtc->planes[i].type == DRM_PLANE_TYPE_CURSOR)
			cursor = &mtk_crtc->planes[i];
	}

	ret = drm_crtc_init_with_planes(drm, &mtk_crtc->base, primary, cursor,
					&mtk_crtc_funcs, NULL);
	if (ret)
		goto err_cleanup_crtc;

	drm_crtc_helper_add(&mtk_crtc->base, &mtk_crtc_helper_funcs);

	return 0;

err_cleanup_crtc:
	drm_crtc_cleanup(&mtk_crtc->base);
	return ret;
}

static int mtk_crtc_num_comp_planes(struct mtk_crtc *mtk_crtc, int comp_idx)
{
	struct mtk_ddp_comp *comp;

	if (comp_idx > 1)
		return 0;

	comp = mtk_crtc->ddp_comp[comp_idx];
	if (!comp->funcs)
		return 0;

	if (comp_idx == 1 && !comp->funcs->bgclr_in_on)
		return 0;

	return mtk_ddp_comp_layer_nr(comp);
}

static inline
enum drm_plane_type mtk_crtc_plane_type(unsigned int plane_idx,
					unsigned int num_planes)
{
	if (plane_idx == 0)
		return DRM_PLANE_TYPE_PRIMARY;
	else if (plane_idx == (num_planes - 1))
		return DRM_PLANE_TYPE_CURSOR;
	else
		return DRM_PLANE_TYPE_OVERLAY;

}

/*
 * Planes are numbered over all components of the path (OVL, then a cascaded
 * OVL_2L blending on top of it): the zpos is that global index and the
 * cursor is the topmost plane, not the last layer of the first OVL, which a
 * cascaded OVL_2L would cover.
 */
static int mtk_crtc_init_comp_planes(struct drm_device *drm_dev,
				     struct mtk_crtc *mtk_crtc,
				     int comp_idx, int pipe,
				     unsigned int total_planes)
{
	int num_planes = mtk_crtc_num_comp_planes(mtk_crtc, comp_idx);
	struct mtk_ddp_comp *comp = mtk_crtc->ddp_comp[comp_idx];
	int i, ret;

	for (i = 0; i < num_planes; i++) {
		ret = mtk_plane_init(drm_dev,
				&mtk_crtc->planes[mtk_crtc->layer_nr],
				BIT(pipe),
				mtk_crtc_plane_type(mtk_crtc->layer_nr, total_planes),
				mtk_ddp_comp_supported_rotations(comp),
				mtk_ddp_comp_get_blend_modes(comp),
				mtk_ddp_comp_get_formats(comp),
				mtk_ddp_comp_get_num_formats(comp),
				mtk_ddp_comp_is_afbc_supported(comp),
				mtk_crtc->layer_nr);
		if (ret)
			return ret;

		mtk_crtc->layer_nr++;
	}
	return 0;
}

struct device *mtk_crtc_dma_dev_get(struct drm_crtc *crtc)
{
	struct mtk_crtc *mtk_crtc = NULL;

	if (!crtc)
		return NULL;

	mtk_crtc = to_mtk_crtc(crtc);
	if (!mtk_crtc)
		return NULL;

	return mtk_crtc->dma_dev;
}

int mtk_crtc_create(struct drm_device *drm_dev, const unsigned int *path,
		    unsigned int path_len, int priv_data_index,
		    const struct mtk_drm_route *conn_routes,
		    unsigned int num_conn_routes)
{
	struct mtk_drm_private *priv = drm_dev->dev_private;
	struct device *dev = drm_dev->dev;
	struct mtk_crtc *mtk_crtc;
	unsigned int num_comp_planes = 0;
	int ret;
	int i;
	bool has_ctm = false;
	uint gamma_lut_size = 0;
	struct drm_crtc *tmp;
	int crtc_i = 0;

	if (!path)
		return 0;

	priv = priv->all_drm_private[priv_data_index];

	drm_for_each_crtc(tmp, drm_dev)
		crtc_i++;

	for (i = 0; i < path_len; i++) {
		enum mtk_ddp_comp_id comp_id = path[i];
		struct device_node *node;
		struct mtk_ddp_comp *comp;

		node = priv->comp_node[comp_id];
		comp = &priv->ddp_comp[comp_id];

		/* Not all drm components have a DTS device node, such as ovl_adaptor,
		 * which is the drm bring up sub driver
		 */
		if (!node && comp_id != DDP_COMPONENT_DRM_OVL_ADAPTOR) {
			dev_info(dev,
				"Not creating crtc %d because component %d is disabled or missing\n",
				crtc_i, comp_id);
			return 0;
		}

		if (!comp->dev) {
			dev_err(dev, "Component %pOF not initialized\n", node);
			return -ENODEV;
		}
	}

	mtk_crtc = devm_kzalloc(dev,
				struct_size(mtk_crtc, ddp_comp, path_len + (conn_routes ? 1 : 0)),
				GFP_KERNEL);
	if (!mtk_crtc)
		return -ENOMEM;

	mtk_crtc->ddp_comp_nr = path_len;
	mtk_crtc->mmsys_dev = priv->mmsys_dev;

	mtk_crtc->mutex = mtk_mutex_get(priv->mutex_dev);
	if (IS_ERR(mtk_crtc->mutex)) {
		ret = PTR_ERR(mtk_crtc->mutex);
		dev_err(dev, "Failed to get mutex: %d\n", ret);
		return ret;
	}

	for (i = 0; i < mtk_crtc->ddp_comp_nr; i++) {
		unsigned int comp_id = path[i];
		struct mtk_ddp_comp *comp;

		comp = &priv->ddp_comp[comp_id];
		mtk_crtc->ddp_comp[i] = comp;

		if (comp->funcs) {
			if (comp->funcs->gamma_set && comp->funcs->gamma_get_lut_size) {
				unsigned int lut_sz = mtk_ddp_gamma_get_lut_size(comp);

				if (lut_sz)
					gamma_lut_size = lut_sz;
			}

			if (comp->funcs->ctm_set)
				has_ctm = true;
		}

		mtk_ddp_comp_register_vblank_cb(comp, mtk_crtc_ddp_irq,
						&mtk_crtc->base);
	}

	for (i = 0; i < mtk_crtc->ddp_comp_nr; i++)
		num_comp_planes += mtk_crtc_num_comp_planes(mtk_crtc, i);

	mtk_crtc->planes = devm_kcalloc(dev, num_comp_planes,
					sizeof(struct drm_plane), GFP_KERNEL);
	if (!mtk_crtc->planes)
		return -ENOMEM;

	for (i = 0; i < mtk_crtc->ddp_comp_nr; i++) {
		ret = mtk_crtc_init_comp_planes(drm_dev, mtk_crtc, i, crtc_i,
						num_comp_planes);
		if (ret)
			return ret;
	}

	/*
	 * Default to use the first component as the dma dev.
	 * In the case of ovl_adaptor sub driver, it needs to use the
	 * dma_dev_get function to get representative dma dev.
	 */
	mtk_crtc->dma_dev = mtk_ddp_comp_dma_dev_get(&priv->ddp_comp[path[0]]);

	ret = mtk_crtc_init(drm_dev, mtk_crtc, crtc_i);
	if (ret < 0)
		return ret;

	if (gamma_lut_size)
		drm_mode_crtc_set_gamma_size(&mtk_crtc->base, gamma_lut_size);
	drm_crtc_enable_color_mgmt(&mtk_crtc->base, 0, has_ctm, gamma_lut_size);
	mutex_init(&mtk_crtc->hw_lock);
	spin_lock_init(&mtk_crtc->config_lock);
	mutex_init(&mtk_crtc->dbg_lock);
	INIT_DELAYED_WORK(&mtk_crtc->vbl_watch, mtk_crtc_vbl_watch);

#if IS_REACHABLE(CONFIG_MTK_CMDQ)
	i = priv->mbox_index++;
	mtk_crtc->cmdq_client.client.dev = mtk_crtc->mmsys_dev;
	mtk_crtc->cmdq_client.client.tx_block = false;
	mtk_crtc->cmdq_client.client.knows_txdone = true;
	mtk_crtc->cmdq_client.client.rx_callback = ddp_cmdq_cb;
	mtk_crtc->cmdq_client.chan =
			mbox_request_channel(&mtk_crtc->cmdq_client.client, i);
	if (IS_ERR(mtk_crtc->cmdq_client.chan)) {
		dev_dbg(dev, "mtk_crtc %d failed to create mailbox client, writing register by CPU now\n",
			drm_crtc_index(&mtk_crtc->base));
		mtk_crtc->cmdq_client.chan = NULL;
	}

	if (mtk_crtc->cmdq_client.chan) {
		ret = of_property_read_u32_index(priv->mutex_node,
						 "mediatek,gce-events",
						 i,
						 &mtk_crtc->cmdq_event);
		if (ret) {
			dev_dbg(dev, "mtk_crtc %d failed to get mediatek,gce-events property\n",
				drm_crtc_index(&mtk_crtc->base));
			mbox_free_channel(mtk_crtc->cmdq_client.chan);
			mtk_crtc->cmdq_client.chan = NULL;
		} else {
			ret = cmdq_pkt_create(&mtk_crtc->cmdq_client,
					      &mtk_crtc->cmdq_handle,
					      PAGE_SIZE);
			if (ret) {
				dev_dbg(dev, "mtk_crtc %d failed to create cmdq packet\n",
					drm_crtc_index(&mtk_crtc->base));
				mbox_free_channel(mtk_crtc->cmdq_client.chan);
				mtk_crtc->cmdq_client.chan = NULL;
			}
		}

		/* for sending blocking cmd in crtc disable */
		init_waitqueue_head(&mtk_crtc->cb_blocking_queue);
	}
#endif

	if (conn_routes) {
		for (i = 0; i < num_conn_routes; i++) {
			unsigned int comp_id = conn_routes[i].route_ddp;
			struct device_node *node = priv->comp_node[comp_id];
			struct mtk_ddp_comp *comp = &priv->ddp_comp[comp_id];

			if (!comp->dev) {
				dev_dbg(dev, "comp_id:%d, Component %pOF not initialized\n",
					comp_id, node);
				/* mark encoder_index to -1, if route comp device is not enabled */
				comp->encoder_index = -1;
				continue;
			}

			mtk_ddp_comp_encoder_index_set(&priv->ddp_comp[comp_id]);
		}

		mtk_crtc->num_conn_routes = num_conn_routes;
		mtk_crtc->conn_routes = conn_routes;

		/* increase ddp_comp_nr at the end of mtk_crtc_create */
		mtk_crtc->ddp_comp_nr++;
	}

	return 0;
}
