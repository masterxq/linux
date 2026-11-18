// SPDX-License-Identifier: GPL-2.0
//
// mt6785-afe-mtcmos.c  --  MT6785 audio power domain without a genpd provider
//
// Some boards keep the SPM power controller (mtk-pm-domains) disabled, e.g. to
// leave the multimedia domain exactly as the bootloader set it up. The AFE can
// then switch the audio MTCMOS itself through the SPM and INFRACFG syscons.
//
// Power sequence, bus protection bit and status bits follow the MT6785 vendor
// kernel, spm_mtcmos_ctrl_audio() in clk-mt6785-pg.c and its
// register definitions (Copyright (c) 2018 MediaTek Inc., GPL-2.0). The vendor
// code waits without a limit; here every wait is bounded and a failed power-on
// switches the domain off again before any AFE register is touched.
//

#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/debugfs.h>
#include <linux/mfd/syscon.h>
#include <linux/of.h>
#include <linux/regmap.h>
#include <linux/seq_file.h>

#include "mt6785-afe-clk.h"
#include "mt6785-afe-common.h"
#include "mt6785-afe-mtcmos.h"

/* SPM (scpsys syscon, 0x10006000) */
#define SPM_POWERON_CONFIG_EN		0x000
#define SPM_PWR_STATUS			0x160
#define SPM_PWR_STATUS_2ND		0x164
#define SPM_AUDIO_PWR_CON		0x31c

#define PWR_RST_B			BIT(0)
#define PWR_ISO				BIT(1)
#define PWR_ON				BIT(2)
#define PWR_ON_2ND			BIT(3)
#define PWR_CLK_DIS			BIT(4)
#define AUDIO_SRAM_PDN			BIT(8)
#define AUDIO_SRAM_PDN_ACK		BIT(12)
#define AUDIO_PWR_STA			BIT(24)
#define SPM_REGISTER_CONTROL		BIT(0)	/* POWERON_CONFIG_EN, set by the bootloader */

/* INFRACFG_AO (0x10001000) */
#define INFRA_TOPAXI_PROTECTEN		0x220
#define INFRA_TOPAXI_PROTECTEN_STA1	0x228
#define INFRA_TOPAXI_PROTECTEN_SET	0x2a0
#define INFRA_TOPAXI_PROTECTEN_CLR	0x2a4
#define AUDIO_PROT_STEP1_0		BIT(31)

#define MTCMOS_POLL_US			10
#define MTCMOS_TIMEOUT_US		10000

static int spm_rmw(struct mt6785_afe_private *p, u32 clr, u32 set)
{
	return regmap_update_bits(p->scpsys, SPM_AUDIO_PWR_CON, clr | set, set);
}

static int mtcmos_wait_status(struct mtk_base_afe *afe, bool on)
{
	struct mt6785_afe_private *p = afe->platform_priv;
	u32 sta = 0, sta2 = 0;
	int ret;

	ret = regmap_read_poll_timeout(p->scpsys, SPM_PWR_STATUS, sta,
				       !!(sta & AUDIO_PWR_STA) == on,
				       MTCMOS_POLL_US, MTCMOS_TIMEOUT_US);
	if (!ret)
		ret = regmap_read_poll_timeout(p->scpsys, SPM_PWR_STATUS_2ND, sta2,
					       !!(sta2 & AUDIO_PWR_STA) == on,
					       MTCMOS_POLL_US, MTCMOS_TIMEOUT_US);
	if (ret) {
		regmap_read(p->scpsys, SPM_PWR_STATUS, &sta);
		regmap_read(p->scpsys, SPM_PWR_STATUS_2ND, &sta2);
		dev_err(afe->dev, "audio MTCMOS: PWR_STATUS 0x%08x / 2ND 0x%08x, bit 24 not %s\n",
			sta, sta2, on ? "set" : "clear");
	}
	return ret;
}

static int mtcmos_wait_sram_ack(struct mtk_base_afe *afe, bool pdn)
{
	struct mt6785_afe_private *p = afe->platform_priv;
	u32 v = 0;
	int ret;

	ret = regmap_read_poll_timeout(p->scpsys, SPM_AUDIO_PWR_CON, v,
				       !!(v & AUDIO_SRAM_PDN_ACK) == pdn,
				       MTCMOS_POLL_US, MTCMOS_TIMEOUT_US);
	if (ret)
		dev_err(afe->dev, "audio MTCMOS: AUDIO_PWR_CON 0x%08x, SRAM_PDN_ACK not %s\n",
			v, pdn ? "set" : "clear");
	return ret;
}

/* vendor: "Set bus protect - step1 : 0", wait for the ack in PROTECTEN_STA1 */
static int mtcmos_bus_protect(struct mtk_base_afe *afe)
{
	struct mt6785_afe_private *p = afe->platform_priv;
	u32 v = 0;
	int ret;

	ret = regmap_write(p->infracfg, INFRA_TOPAXI_PROTECTEN_SET, AUDIO_PROT_STEP1_0);
	if (ret)
		return ret;
	ret = regmap_read_poll_timeout(p->infracfg, INFRA_TOPAXI_PROTECTEN_STA1, v,
				       (v & AUDIO_PROT_STEP1_0) == AUDIO_PROT_STEP1_0,
				       MTCMOS_POLL_US, MTCMOS_TIMEOUT_US);
	if (ret)
		dev_err(afe->dev, "audio MTCMOS: bus protection not acknowledged (STA1 0x%08x)\n", v);
	return ret;
}

/*
 * Vendor power-down sequence after the bus protection is set: SRAM down (with
 * ack), isolation, clock off, reset, power off, then wait for the status bits.
 * Also used to undo a partial power-on. A missing SRAM ack is logged and the
 * domain is switched off anyway.
 */
static int mtcmos_power_down(struct mtk_base_afe *afe)
{
	struct mt6785_afe_private *p = afe->platform_priv;

	spm_rmw(p, 0, AUDIO_SRAM_PDN);
	mtcmos_wait_sram_ack(afe, true);
	spm_rmw(p, 0, PWR_ISO);
	spm_rmw(p, 0, PWR_CLK_DIS);
	spm_rmw(p, PWR_RST_B, 0);
	spm_rmw(p, PWR_ON, 0);
	spm_rmw(p, PWR_ON_2ND, 0);
	return mtcmos_wait_status(afe, false);
}

int mt6785_afe_mtcmos_on(struct mtk_base_afe *afe)
{
	struct mt6785_afe_private *p = afe->platform_priv;
	u32 v = 0;
	int ret;

	if (!p->self_mtcmos || p->mtcmos_on)
		return 0;

	/* vendor: parent clock infracfg_ao_audio_26m ("needed for the SRAM PDN delay IP") */
	ret = clk_prepare_enable(p->clk[CLK_INFRA_AUDIO_26M]);
	if (ret) {
		dev_err(afe->dev, "audio MTCMOS: infra audio 26M clock: %d\n", ret);
		return ret;
	}

	spm_rmw(p, 0, PWR_ON);
	spm_rmw(p, 0, PWR_ON_2ND);
	ret = mtcmos_wait_status(afe, true);
	if (ret)
		goto err_down;

	spm_rmw(p, PWR_CLK_DIS, 0);
	spm_rmw(p, PWR_ISO, 0);
	spm_rmw(p, 0, PWR_RST_B);
	spm_rmw(p, AUDIO_SRAM_PDN, 0);
	ret = mtcmos_wait_sram_ack(afe, false);
	if (ret)
		goto err_down;

	/* vendor: "Release bus protect - step1 : 0" */
	ret = regmap_write(p->infracfg, INFRA_TOPAXI_PROTECTEN_CLR, AUDIO_PROT_STEP1_0);
	if (ret)
		goto err_down;
	/*
	 * Not in the vendor sequence (it does not wait here): make sure the
	 * protection is really gone before the first AFE register access.
	 */
	ret = regmap_read_poll_timeout(p->infracfg, INFRA_TOPAXI_PROTECTEN_STA1, v,
				       !(v & AUDIO_PROT_STEP1_0),
				       MTCMOS_POLL_US, MTCMOS_TIMEOUT_US);
	if (ret) {
		dev_err(afe->dev, "audio MTCMOS: bus protection still active (STA1 0x%08x)\n", v);
		if (mtcmos_bus_protect(afe))
			goto err_keep;
		goto err_down;
	}

	p->mtcmos_on = true;
	p->mtcmos_cycles++;
	return 0;

err_down:
	p->mtcmos_err = ret;
	if (mtcmos_power_down(afe))
		goto err_keep;
	clk_disable_unprepare(p->clk[CLK_INFRA_AUDIO_26M]);
	dev_err(afe->dev, "audio MTCMOS power-on failed (%d), domain switched off again\n", ret);
	return ret;

err_keep:
	/* state unknown: keep the 26M clock, never claim the domain is usable */
	p->mtcmos_err = ret;
	dev_err(afe->dev, "audio MTCMOS power-on failed (%d), domain state unknown\n", ret);
	return ret;
}

int mt6785_afe_mtcmos_off(struct mtk_base_afe *afe)
{
	struct mt6785_afe_private *p = afe->platform_priv;
	int ret;

	if (!p->self_mtcmos || !p->mtcmos_on)
		return 0;

	ret = mtcmos_bus_protect(afe);
	if (ret) {
		/*
		 * The vendor code would spin here. Cutting the power under a busy
		 * bus is worse than leaving the domain on: release the request and
		 * keep the domain powered (the next power-on is idempotent).
		 */
		regmap_write(p->infracfg, INFRA_TOPAXI_PROTECTEN_CLR, AUDIO_PROT_STEP1_0);
		p->mtcmos_err = ret;
		dev_err(afe->dev, "audio MTCMOS left on: bus protection failed (%d)\n", ret);
		return ret;
	}

	ret = mtcmos_power_down(afe);
	p->mtcmos_on = false;
	if (ret) {
		/* keep the 26M clock: the domain may still be partly powered */
		p->mtcmos_err = ret;
		dev_err(afe->dev, "audio MTCMOS power-down incomplete (%d)\n", ret);
		return ret;
	}

	clk_disable_unprepare(p->clk[CLK_INFRA_AUDIO_26M]);
	return 0;
}

static int mtcmos_show(struct seq_file *s, void *data)
{
	struct mtk_base_afe *afe = s->private;
	struct mt6785_afe_private *p = afe->platform_priv;
	static const struct { bool spm; u32 reg; const char *name; } regs[] = {
		{ true, SPM_POWERON_CONFIG_EN, "0x10006000 POWERON_CONFIG_EN" },
		{ true, SPM_PWR_STATUS, "0x10006160 PWR_STATUS" },
		{ true, SPM_PWR_STATUS_2ND, "0x10006164 PWR_STATUS_2ND" },
		{ true, SPM_AUDIO_PWR_CON, "0x1000631c AUDIO_PWR_CON" },
		{ false, INFRA_TOPAXI_PROTECTEN, "0x10001220 TOPAXI_PROTECTEN" },
		{ false, INFRA_TOPAXI_PROTECTEN_STA1, "0x10001228 TOPAXI_PROTECTEN_STA1" },
	};
	unsigned int i;
	u32 v;

	seq_printf(s, "mode: %s\n", p->self_mtcmos ? "self (syscon)" : "genpd");
	seq_printf(s, "driver state: %s, power-on count %u, last error %d\n",
		   p->mtcmos_on ? "on" : "off", p->mtcmos_cycles, p->mtcmos_err);
	if (!p->self_mtcmos)
		return 0;
	/* SPM and INFRACFG_AO are always on: reading them is safe in any state */
	for (i = 0; i < ARRAY_SIZE(regs); i++) {
		if (regmap_read(regs[i].spm ? p->scpsys : p->infracfg, regs[i].reg, &v))
			seq_printf(s, "%-36s = (read error)\n", regs[i].name);
		else
			seq_printf(s, "%-36s = 0x%08x\n", regs[i].name, v);
	}
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(mtcmos);

static void mtcmos_debugfs_remove(void *data)
{
	debugfs_remove(data);
}

static bool spm_genpd_active(struct device_node *spm)
{
	struct device_node *child;

	for_each_available_child_of_node(spm, child) {
		if (of_device_is_compatible(child, "mediatek,mt6785-power-controller")) {
			of_node_put(child);
			return true;
		}
	}
	return false;
}

int mt6785_afe_mtcmos_init(struct mtk_base_afe *afe)
{
	struct mt6785_afe_private *p = afe->platform_priv;
	struct device *dev = afe->dev;
	struct device_node *np = dev->of_node, *spm;
	struct dentry *d;
	bool genpd_active;
	u32 con = 0, sta = 0, sta2 = 0, prot = 0, cfg = 0;
	int ret;

	if (of_property_present(np, "power-domains")) {
		p->self_mtcmos = false;
		dev_info(dev, "audio power domain: genpd\n");
		goto debugfs;
	}

	/*
	 * Without a power domain and without the syscons there is no way to
	 * know that the block is powered: refuse instead of risking a bus hang.
	 */
	spm = of_parse_phandle(np, "mediatek,scpsys", 0);
	if (!spm)
		return dev_err_probe(dev, -EINVAL,
				     "neither power-domains nor mediatek,scpsys given\n");
	genpd_active = spm_genpd_active(spm);
	of_node_put(spm);
	if (genpd_active)
		return dev_err_probe(dev, -EBUSY,
				     "SPM power controller is enabled: use power-domains instead of mediatek,scpsys\n");

	p->scpsys = syscon_regmap_lookup_by_phandle(np, "mediatek,scpsys");
	if (IS_ERR(p->scpsys))
		return dev_err_probe(dev, PTR_ERR(p->scpsys), "no scpsys syscon\n");
	p->infracfg = syscon_regmap_lookup_by_phandle(np, "mediatek,infracfg");
	if (IS_ERR(p->infracfg))
		return dev_err_probe(dev, PTR_ERR(p->infracfg), "no infracfg syscon\n");

	ret = regmap_read(p->scpsys, SPM_POWERON_CONFIG_EN, &cfg);
	if (ret)
		return ret;
	regmap_read(p->scpsys, SPM_AUDIO_PWR_CON, &con);
	regmap_read(p->scpsys, SPM_PWR_STATUS, &sta);
	regmap_read(p->scpsys, SPM_PWR_STATUS_2ND, &sta2);
	regmap_read(p->infracfg, INFRA_TOPAXI_PROTECTEN, &prot);
	dev_info(dev, "audio power domain: self (syscon); POWERON_CONFIG_EN 0x%08x AUDIO_PWR_CON 0x%08x PWR_STATUS 0x%08x/0x%08x PROTECTEN 0x%08x\n",
		 cfg, con, sta, sta2, prot);
	/* the SPM ignores register writes unless the bootloader enabled them */
	if (!(cfg & SPM_REGISTER_CONTROL))
		return dev_err_probe(dev, -ENODEV, "SPM register control not enabled\n");

	p->self_mtcmos = true;

debugfs:
	d = debugfs_create_file("mt6785-afe-mtcmos", 0444, NULL, afe, &mtcmos_fops);
	return devm_add_action_or_reset(dev, mtcmos_debugfs_remove, d);
}
