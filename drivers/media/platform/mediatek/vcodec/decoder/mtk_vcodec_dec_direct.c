// SPDX-License-Identifier: GPL-2.0
/*
 * Decoder programmed by the host through its registers, without firmware:
 * register window and error recovery.
 */

#include <linux/iopoll.h>
#include <linux/mfd/syscon.h>
#include <linux/platform_device.h>

#include "mtk_vcodec_dec_direct.h"
#include "mtk_vcodec_dec_drv.h"

/* time for the decoder to stop at a hardware break */
#define MTK_VDEC_HW_BREAK_US		20000

/**
 * mtk_vdec_direct_init - get the resources of a decoder without firmware
 * @dev: decoder device
 *
 * Maps the register window of the node, points the register bases of the
 * blocks into it and looks up the vdecsys syscon.
 *
 * Return: 0 or a negative error code.
 */
int mtk_vdec_direct_init(struct mtk_vcodec_dec_dev *dev)
{
	const struct mtk_vdec_direct_data *data = dev->vdec_pdata->direct;
	struct platform_device *pdev = dev->plat_dev;
	struct device *d = &pdev->dev;
	struct resource *res;
	void __iomem *base;
	int i;

	base = devm_platform_get_and_ioremap_resource(pdev, 0, &res);
	if (IS_ERR(base))
		return PTR_ERR(base);

	for (i = VDEC_MISC; i <= VDEC_PP; i++) {
		if (data->reg_offset[i] >= resource_size(res))
			return dev_err_probe(d, -EINVAL,
					     "register block %d outside the window\n", i);
		dev->reg_base[i] = base + data->reg_offset[i];
	}
	if (data->hevc_vld >= resource_size(res) ||
	    data->vp9_vld >= resource_size(res) ||
	    data->vp8_vld >= resource_size(res) ||
	    data->vld2 >= resource_size(res))
		return dev_err_probe(d, -EINVAL, "codec block outside the window\n");

	dev->vdecsys_regmap = syscon_regmap_lookup_by_phandle(d->of_node,
							      "mediatek,vdecsys");
	if (IS_ERR(dev->vdecsys_regmap))
		return dev_err_probe(d, PTR_ERR(dev->vdecsys_regmap),
				     "missing mediatek,vdecsys\n");

	return 0;
}

/**
 * mtk_vdec_direct_block - registers of a block that only one codec uses
 * @dev: decoder device
 * @offset: offset of the block in the register window
 *
 * Return: the mapped address of the block.
 */
void __iomem *mtk_vdec_direct_block(struct mtk_vcodec_dec_dev *dev, u32 offset)
{
	const struct mtk_vdec_direct_data *data = dev->vdec_pdata->direct;

	return dev->reg_base[VDEC_LD] - data->reg_offset[VDEC_LD] + offset;
}

/**
 * mtk_vdec_direct_hw_break - stop the decoder and reset its bitstream reader
 * @dev: decoder device
 *
 * For a picture that did not finish in time: request a break, wait until
 * the decoder has stopped its memory transfers and reset the reader. The
 * next picture starts with a full reset of the blocks.
 */
void mtk_vdec_direct_hw_break(struct mtk_vcodec_dec_dev *dev)
{
	void __iomem *misc = dev->reg_base[VDEC_MISC];
	void __iomem *vld = dev->reg_base[VDEC_LD];
	u32 val;

	val = readl(misc + VDEC_MISC_HW_BREAK);
	writel(val | VDEC_MISC_HW_BREAK_REQ, misc + VDEC_MISC_HW_BREAK);
	if (readl_poll_timeout(misc + VDEC_MISC_HW_BREAK_STA, val,
			       (val & VDEC_MISC_HW_BREAK_DONE) ==
			       VDEC_MISC_HW_BREAK_DONE, 10, MTK_VDEC_HW_BREAK_US))
		dev_err(&dev->plat_dev->dev, "hardware break timeout: %#x\n", val);

	writel(VDEC_VLD_SRST_RESET, vld + VDEC_VLD_SRST);
	writel(0, vld + VDEC_VLD_SRST);

	val = readl(misc + VDEC_MISC_HW_BREAK);
	writel(val & ~VDEC_MISC_HW_BREAK_REQ, misc + VDEC_MISC_HW_BREAK);
}

const struct mtk_vdec_direct_data mtk_vdec_mt6785_direct_data = {
	.reg_offset = {
		[VDEC_MISC]	= 0x5000,
		[VDEC_LD]	= 0x0000,
		[VDEC_TOP]	= 0x0800,
		[VDEC_CM]	= 0x1000,
		[VDEC_AD]	= 0x2000,
		[VDEC_AV]	= 0x3000,
		[VDEC_PP]	= 0x4000,
	},
	.hevc_vld = 0x8000,
	.vp9_vld = 0x8400,
	.vp8_vld = 0x7800,
	.vld2 = 0x6800,
	.capability = MTK_VDEC_FORMAT_MM21 | MTK_VDEC_IS_SUPPORT_10BIT |
		      MTK_VDEC_FORMAT_H264_SLICE | MTK_VDEC_FORMAT_HEVC_FRAME |
		      MTK_VDEC_FORMAT_MPEG2_SLICE | MTK_VDEC_FORMAT_VP9_FRAME |
		      MTK_VDEC_FORMAT_VP8_FRAME,
	.max_width = 3840,
	.max_height = 2176,
	.stateful = &mtk_vdec_direct_stateful_pdata,
};
