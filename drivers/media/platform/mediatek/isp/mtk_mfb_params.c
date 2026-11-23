// SPDX-License-Identifier: GPL-2.0-only
/*
 * MediaTek MT6785 multi-frame blending block (MFB): parameter buffers.
 *
 * Checks the params buffers of user space (struct v4l2_isp_params_buffer
 * with the blocks of <linux/media/mediatek/mtk-mfb-mt6785.h>) and keeps the
 * tuning words for the next job. The words go to the hardware as they are.
 */

#include <linux/array_size.h>
#include <linux/build_bug.h>
#include <linux/string.h>
#include <media/v4l2-isp.h>

#include "mtk_mfb.h"
#include "mtk_mfb_regs.h"

static const struct v4l2_isp_params_block_type_info mtk_mfb_block_info[] = {
	[MTK_MFB_PARAMS_BLOCK_BLEND] = { sizeof(struct mtk_mfb_params_blend) },
	[MTK_MFB_PARAMS_BLOCK_EXTRA] = { sizeof(struct mtk_mfb_params_extra) },
};

static int mtk_mfb_block_check(const struct v4l2_isp_params_block_header *h)
{
	const struct mtk_mfb_params_blend *b = (const void *)h;
	const struct mtk_mfb_params_extra *e = (const void *)h;

	if (h->flags & V4L2_ISP_PARAMS_FL_BLOCK_DISABLE)
		return 0;

	switch (h->type) {
	case MTK_MFB_PARAMS_BLOCK_BLEND:
		return b->reserved ? -EINVAL : 0;
	case MTK_MFB_PARAMS_BLOCK_EXTRA:
		return e->con & ~MFB_CON_TUNING ? -EINVAL : 0;
	default:
		return -EINVAL;
	}
}

/**
 * mtk_mfb_params_check - copy and check a queued params buffer
 * @mfb: the device
 * @vb: the buffer
 * @buf: its driver part; gets the checked copy
 *
 * Return: 0 or -EINVAL.
 */
int mtk_mfb_params_check(struct mtk_mfb *mfb, struct vb2_buffer *vb,
			 struct mtk_mfb_buffer *buf)
{
	const struct v4l2_isp_params_buffer *p = (const void *)buf->params;
	const struct v4l2_isp_params_block_header *h;
	size_t len = vb2_get_plane_payload(vb, 0);
	size_t off;
	int ret;

	ret = v4l2_isp_params_validate_buffer_size(mfb->dev, vb,
						   MTK_MFB_PARAMS_MAX_SIZE);
	if (ret)
		return ret;

	/* user space cannot change the copy behind the checks */
	memcpy(buf->params, vb2_plane_vaddr(vb, 0), len);

	ret = v4l2_isp_params_validate_buffer(mfb->dev, vb, p,
					      mtk_mfb_block_info,
					      ARRAY_SIZE(mtk_mfb_block_info));
	if (ret)
		return ret;
	if (p->version != MTK_MFB_PARAMS_VERSION_V1)
		return -EINVAL;

	for (off = 0; off < p->data_size; off += h->size) {
		h = (const void *)(p->data + off);
		ret = mtk_mfb_block_check(h);
		if (ret) {
			dev_dbg(mfb->dev, "params: block %u at %zu refused\n",
				h->type, off);
			return ret;
		}
	}

	return 0;
}

static void mtk_mfb_apply_blend(struct mtk_mfb_tuning *t,
				const struct mtk_mfb_params_blend *b)
{
	t->ll[0] = b->ll_con1;
	t->ll[1] = b->ll_con2;
	memcpy(t->mbd, b->mbd_con, sizeof(t->mbd));
}

static void mtk_mfb_apply_extra(struct mtk_mfb_tuning *t,
				const struct mtk_mfb_params_extra *e)
{
	t->con = e->con;
	memcpy(&t->ll[2], e->ll_con, sizeof(e->ll_con));
	t->edge = e->edge;
}

/**
 * mtk_mfb_params_apply - take the blocks of a params buffer
 * @t: tuning words, changed for the blocks in the buffer
 * @buf: buffer checked by mtk_mfb_params_check()
 */
void mtk_mfb_params_apply(struct mtk_mfb_tuning *t,
			  const struct mtk_mfb_buffer *buf)
{
	static const struct mtk_mfb_params_blend zero_blend;
	static const struct mtk_mfb_params_extra zero_extra;
	const struct v4l2_isp_params_buffer *p = (const void *)buf->params;
	const struct v4l2_isp_params_block_header *h;
	bool off;
	size_t o;

	BUILD_BUG_ON(sizeof(zero_blend.mbd_con) != sizeof(t->mbd));
	BUILD_BUG_ON(sizeof(zero_extra.ll_con) + 2 * sizeof(u32) !=
		     sizeof(t->ll));

	for (o = 0; o < p->data_size; o += h->size) {
		h = (const void *)(p->data + o);
		off = h->flags & V4L2_ISP_PARAMS_FL_BLOCK_DISABLE;
		switch (h->type) {
		case MTK_MFB_PARAMS_BLOCK_BLEND:
			mtk_mfb_apply_blend(t, off ? &zero_blend :
					    (const void *)h);
			break;
		case MTK_MFB_PARAMS_BLOCK_EXTRA:
			mtk_mfb_apply_extra(t, off ? &zero_extra :
					    (const void *)h);
			break;
		}
	}
}
