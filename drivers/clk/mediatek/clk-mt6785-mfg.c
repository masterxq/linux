// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2025 Dinolek <git@dinolek.me>
 */

#include <dt-bindings/clock/mediatek,mt6785-clk.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>

#include "clk-gate.h"
#include "clk-mtk.h"

static const struct mtk_gate_regs mfg_cg_regs = {
	.sta_ofs = 0x0,
	.clr_ofs = 0x8,
	.set_ofs = 0x4,
};

#define GATE_MFG(_id, _name, _parent, _shift)                     \
	GATE_MTK_FLAGS(_id, _name, _parent, &mfg_cg_regs, _shift, \
		       &mtk_clk_gate_ops_setclr, CLK_SET_RATE_PARENT)

static const struct mtk_gate mfg_clks[] = {
	GATE_MFG(CLK_MFG_BG3D, "mfg_bg3d", "top_mfg", 0),
};

static const struct mtk_clk_desc mfg_desc = {
	.clks = mfg_clks,
	.num_clks = ARRAY_SIZE(mfg_clks),
	.need_runtime_pm = true,
};

static const struct of_device_id of_match_clk_mt6785_mfg[] = {
	{ .compatible = "mediatek,mt6785-mfgcfg", .data = &mfg_desc },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, of_match_clk_mt6785_mfg);

/*
 * The controller sits in the MFG power domain. If the clock core resumes it
 * on demand, clk_prepare() of the GPU clock takes the genpd lock of MFG under
 * the clock prepare lock, while powering on MFG (on behalf of a GPU core
 * domain) holds that genpd lock and prepares the clock of MFG_ASYNC: an ABBA
 * between prepare_lock and genpd->mlock. Keep the controller resumed while it
 * is bound, so the clock core only counts usage on an active device and never
 * calls into genpd under the prepare lock. MFG stayed on before as well while
 * panfrost holds the GPU clock prepared.
 */
static int clk_mt6785_mfg_probe(struct platform_device *pdev)
{
	int ret;

	ret = mtk_clk_simple_probe(pdev);
	if (ret)
		return ret;

	ret = pm_runtime_resume_and_get(&pdev->dev);
	if (ret) {
		mtk_clk_simple_remove(pdev);
		return dev_err_probe(&pdev->dev, ret, "failed to resume\n");
	}

	return 0;
}

static void clk_mt6785_mfg_remove(struct platform_device *pdev)
{
	pm_runtime_put(&pdev->dev);
	mtk_clk_simple_remove(pdev);
}

static struct platform_driver clk_mt6785_mfg_drv = {
	.probe = clk_mt6785_mfg_probe,
	.remove = clk_mt6785_mfg_remove,
	.driver = {
		.name = "clk-mt6785-mfg",
		.of_match_table = of_match_clk_mt6785_mfg,
	},
};
module_platform_driver(clk_mt6785_mfg_drv);

MODULE_DESCRIPTION("MediaTek MT6785 GPU MFlexGraphics driver");
MODULE_LICENSE("GPL");
