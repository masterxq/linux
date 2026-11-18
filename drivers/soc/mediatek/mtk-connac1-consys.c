// SPDX-License-Identifier: GPL-2.0-only
/*
 * MediaTek CONNAC1 connectivity subsystem (consys, "soc1_0") power control for MT6785.
 *
 * Powers the connectivity subsystem (CONN MTCMOS, CONN MCU, A-die supplies) and places the
 * BT/WIFI/MCU RAM patches into the reserved consys EMI region, up to the point where the CONN MCU
 * reports "ready" (0x1D1E). BTIF/STP/WMT are in mtk-connac1-wmt.c; the WLAN driver is separate.
 *
 * The sequence, register values and delays follow MediaTek's GPL-2.0 vendor connectivity driver
 * (wmt_drv, mtk_wcn_consys_hw.c and mt6785.c). This is an independent implementation; no vendor
 * code was copied verbatim.
 *
 * Power-on is triggered explicitly through sysfs ("power_on") or by a function driver, never at
 * probe.
 *
 * The CONN MTCMOS is switched through the CONN power domain (genpd, runtime PM) when the node has
 * "power-domains"; the domain driver runs the same sequence incl. bus protection. Without it the
 * driver switches the MTCMOS itself through the SCPSYS syscon (vendor sequence). Either way the
 * step stays where the vendor has it: after the supplies, the MCU reset and the CCIF clock. The
 * platform bus powers an attached domain on before probe; probe hands it back at once (runtime PM
 * active -> suspended), so it is on only for that moment, without supplies.
 */

#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/crc32.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/ktime.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_reserved_mem.h>
#include <linux/pinctrl/consumer.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/pm_wakeup.h>
#include <linux/iopoll.h>
#include <linux/regmap.h>
#include <linux/of_platform.h>
#include <linux/regulator/consumer.h>
#include <linux/reset.h>
#include <linux/soc/mediatek/mtk-connac1.h>
#include <linux/seq_file.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <linux/uaccess.h>

#include "mtk-connac1.h"

/* INFRACFG_AO (syscon) */
#define INFRA_CONN_EMI_REMAP		0x380	/* [19:0] = consys EMI base >> 16 */
#define INFRA_CONN_PERI_REMAP		0x388	/* [19:0] = 0x10003000 >> 16 */
#define INFRA_CONN_MD_REMAP		0x39c	/* modem direct path, only read */
#define INFRA_REMAP_MASK		GENMASK(19, 0)
#define CONN_PERI_REMAP_VAL		(0x10003000 >> 16)

/* PERICFG (syscon) */
#define PERI_CON_STA			0x438	/* PCCIF4_AP_PERI_AP_CCU_CONFIG, [1:0] */
#define PERI_CON_STA_MASK		GENMASK(1, 0)
#define PERI_CON_STA_PWR_ON		0x1

/*
 * CONN MTCMOS without genpd, for boards that keep the SPM power controller (mtk-pm-domains)
 * disabled. Sequence of the vendor spm_mtcmos_ctrl_conn(), with bounded waits.
 */
#define SPM_POWERON_CONFIG_EN		0x000	/* only read: set by the boot loader (0x0b16xxx1) */
#define SPM_PWR_STATUS			0x160
#define SPM_PWR_STATUS_2ND		0x164
#define SPM_CONN_PWR_CON		0x320
#define SPM_CONN_STA			BIT(1)
#define PWR_RST_B			BIT(0)
#define PWR_ISO				BIT(1)
#define PWR_ON				BIT(2)
#define PWR_ON_2ND			BIT(3)
#define PWR_CLK_DIS			BIT(4)
#define INFRA_TOPAXI_PROT_SET		0x2a0
#define INFRA_TOPAXI_PROT_CLR		0x2a4
#define INFRA_TOPAXI_PROT_STA1		0x228
#define INFRA_TOPAXI_PROT_1_SET		0x2a8
#define INFRA_TOPAXI_PROT_1_CLR		0x2ac
#define INFRA_TOPAXI_PROT_STA1_1	0x258
#define CONN_PROT_STEP1			(BIT(13) | BIT(18))	/* TOPAXI */
#define CONN_PROT_STEP2_0		BIT(14)			/* TOPAXI */
#define CONN_PROT_STEP2_1		BIT(10)			/* TOPAXI_1 */
#define MTCMOS_TIMEOUT_US		10000

/* CONN_MCU_CFG, 0x18002000 */
#define MCU_HW_VER			0x000
#define MCU_FW_VER			0x004
#define MCU_STA_16C			0x16c
#define MCU_BOOT_STA			0x600
#define MCU_BOOT_READY			0x1d1e

/* CONN_HIF_PDMA, 0x18004000 */
#define PDMA_SLPPROT_CTRL		0x160
#define PDMA_SLPPROT_CONN2EMI		BIT(4)

/* CONN_HIF_ON, 0x18007000 */
#define HIF_ON_CPUPCR			0x104

/* CONN_MCU_TOP_MISC_OFF, 0x180b1000 */
#define TOP_MISC_OFF_IP_VER		0x010
#define TOP_MISC_OFF_CONF_ID		0x01c
#define CONSYS_IP_VER_MT6785		0x10020600

/* CONN_AFE, 0x180b3000 */
#define AFE_WBG_PLL_03			0x034
#define AFE_WPLL_MASK			GENMASK(20, 19)
#define AFE_WPLL_VAL			BIT(19)

/* CONN_TOP_MISC_ON, 0x180c1000 */
#define TOP_MISC_ON_COCLK_STABLE	0x200
#define COCLK_STABLE_TIME_MASK		GENMASK(15, 0)
#define COCLK_STABLE_TIME		0x708
#define TOP_MISC_ON_COCLK_ACK		0x204
#define COCLK_ACK_CLR_MASK		(GENMASK(15, 8) | BIT(0))
#define COCLK_ACK_VAL			0x600
#define TOP_MISC_ON_TOPPOSE		0x600
#define TOPPOSE_RESTORE_DONE		BIT(23)

/* AP_PCCIF4, 0x1024c000 */
#define PCCIF4_ACK			0x14

/* RAM patch files */
#define PATCH_HDR_LEN			48
#define CONN_EMI_VIEW_BYTE3		0xf0	/* CONN sees the EMI at 0xf0000000 */

/* power-on polls: same counts and sleeps as the vendor driver, but bounded and fatal */
#define CHIPID_POLL_TRIES		10
#define READY_POLL_TRIES		12
#define POLL_SLEEP_MS			20

/* supplies, exactly as the vendor driver programs them */
#define VCN18_UV			1800000
#define VCN33_1_UV			3300000
#define VCN33_2_UV			2800000

static const char * const consys_reg_names[IO_NUM] = {
	[IO_MCU] = "mcu-cfg",
	[IO_HIF_PDMA] = "hif-pdma",
	[IO_HIF_ON] = "hif-on",
	[IO_TOP_MISC_OFF] = "top-misc-off",
	[IO_AFE] = "afe",
	[IO_TOP_MISC_ON] = "top-misc-on",
	[IO_PCCIF4] = "pccif4",
};

/* CONN registers shown in the status file and captured on failure (only read when powered) */
static const struct consys_dump_reg {
	enum consys_iomem io;
	u32 off;
	const char *name;
} consys_dump_regs[] = {
	{ IO_TOP_MISC_OFF, TOP_MISC_OFF_IP_VER, "CONN_IP_VER" },
	{ IO_TOP_MISC_OFF, TOP_MISC_OFF_CONF_ID, "CONN_CONF_ID" },
	{ IO_MCU, MCU_HW_VER, "CONN_HW_VER" },
	{ IO_MCU, MCU_FW_VER, "CONN_FW_VER" },
	{ IO_MCU, MCU_BOOT_STA, "CONN_MCU_600 (ready 0x1d1e)" },
	{ IO_MCU, MCU_STA_16C, "CONN_MCU_16C" },
	{ IO_HIF_ON, HIF_ON_CPUPCR, "CONN_HIF_ON_CPUPCR" },
	{ IO_TOP_MISC_ON, TOP_MISC_ON_COCLK_STABLE, "CONN_COCLOCK_STABLE_TIME" },
	{ IO_TOP_MISC_ON, TOP_MISC_ON_COCLK_ACK, "CONN_COCLOCK_ACK_EN" },
	{ IO_HIF_PDMA, PDMA_SLPPROT_CTRL, "CONN_HIF_PDMA_SLPPROT" },
	{ IO_TOP_MISC_ON, TOP_MISC_ON_TOPPOSE, "CONN_TOPPOSE_RESTORE" },
	{ IO_AFE, AFE_WBG_PLL_03, "CONN_AFE_WBG_PLL_03" },
};

static_assert(ARRAY_SIZE(consys_dump_regs) == CONSYS_DUMP_REGS);

static const phys_addr_t consys_io_phys[IO_NUM] = {
	[IO_MCU] = 0x18002000, [IO_HIF_PDMA] = 0x18004000, [IO_HIF_ON] = 0x18007000,
	[IO_TOP_MISC_OFF] = 0x180b1000, [IO_AFE] = 0x180b3000, [IO_TOP_MISC_ON] = 0x180c1000,
	[IO_PCCIF4] = 0x1024c000,
};

static char *fw_prefix = "soc1_0";
module_param(fw_prefix, charp, 0444);
MODULE_PARM_DESC(fw_prefix, "chip prefix of the RAM patch files (default soc1_0)");
static char *fw_variant = "2a_1";
module_param(fw_variant, charp, 0444);
MODULE_PARM_DESC(fw_variant, "variant of the RAM patch files (default 2a_1)");
static bool fw_vercheck = true;
module_param(fw_vercheck, bool, 0644);
MODULE_PARM_DESC(fw_vercheck, "refuse patches whose SW version differs from the chip FW version");

const char *consys_fw_prefix(void)
{
	return fw_prefix;
}

const char *consys_fw_variant(void)
{
	return fw_variant;
}

bool consys_fw_vercheck(void)
{
	return fw_vercheck;
}

/* CONN MCU state for failure analysis; only with the CONN MTCMOS on */
void consys_conn_sample(struct consys *cs, u32 *boot, u32 *sta16c, u32 *coclk, u32 *pc,
			unsigned int npc)
{
	unsigned int i;

	if (!(cs->on & ON_MTCMOS))
		return;
	*boot = readl(cs->io[IO_MCU] + MCU_BOOT_STA);
	*sta16c = readl(cs->io[IO_MCU] + MCU_STA_16C);
	*coclk = readl(cs->io[IO_TOP_MISC_ON] + TOP_MISC_ON_COCLK_ACK);
	for (i = 0; i < npc; i++) {
		pc[i] = readl(cs->io[IO_HIF_ON] + HIF_ON_CPUPCR);
		if (i + 1 < npc)
			udelay(500);
	}
}

static void consys_step(struct consys *cs, const char *name)
{
	s64 us = ktime_us_delta(ktime_get(), cs->t_start);

	if (cs->nsteps < MAX_STEPS) {
		cs->steps[cs->nsteps].name = name;
		cs->steps[cs->nsteps].us = us;
		cs->nsteps++;
	}
	dev_dbg(cs->dev, "+%lld us: %s\n", us, name);
}

static void consys_log_write(struct consys *cs, const char *name, u32 addr, u32 before, u32 after)
{
	if (cs->nwrites < MAX_WRITES) {
		cs->writes[cs->nwrites].name = name;
		cs->writes[cs->nwrites].addr = addr;
		cs->writes[cs->nwrites].before = before;
		cs->writes[cs->nwrites].after = after;
		cs->nwrites++;
	}
	dev_info(cs->dev, "%s 0x%08x: 0x%08x -> 0x%08x\n", name, addr, before, after);
}

/* read-modify-write of a CONN register, logged with the value before and after */
static void consys_rmw(struct consys *cs, enum consys_iomem io, u32 off, u32 clr, u32 set,
		       const char *name)
{
	void __iomem *a = cs->io[io] + off;
	u32 before = readl(a);

	writel((before & ~clr) | set, a);
	consys_log_write(cs, name, consys_io_phys[io] + off, before, readl(a));
}

static int consys_regmap_rmw(struct consys *cs, struct regmap *map, phys_addr_t base, u32 off,
			     u32 mask, u32 val, const char *name)
{
	u32 before, after;
	int ret;

	ret = regmap_read(map, off, &before);
	if (!ret)
		ret = regmap_update_bits(map, off, mask, val);
	if (!ret)
		ret = regmap_read(map, off, &after);
	if (ret) {
		dev_err(cs->dev, "%s: regmap access failed: %d\n", name, ret);
		return ret;
	}
	consys_log_write(cs, name, base + off, before, after);
	return 0;
}

static void consys_snapshot(struct consys *cs)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(consys_dump_regs); i++)
		cs->snap[i] = readl(cs->io[consys_dump_regs[i].io] + consys_dump_regs[i].off);
	cs->snap_valid = true;
}

/* ---------------------------------------------------------------- RAM patches */

static void consys_release_patches(struct consys *cs)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(cs->patch); i++) {
		release_firmware(cs->patch[i].fw);
		cs->patch[i].fw = NULL;
	}
}

/*
 * Load all three patch files before any hardware is touched, so that a missing file costs
 * nothing and the time between MTCMOS on and patch copy matches the vendor driver.
 *
 * Header (48 bytes, struct wmt_rom_patch of the vendor driver):
 *   0x00 build time (16)  0x10 platform (4)  0x14 HW version (BE16)  0x16 SW version (BE16)
 *   0x18 count/sequence   0x19..0x1b address: EMI offset = (b[0x1a] << 16) | (b[0x19] << 8),
 *        b[0x1b] = 0xf0 (CONN view of the EMI)   0x1c..0x1f type (WMT driver type in 0x1f)
 */
static int consys_request_patches(struct consys *cs)
{
	unsigned int i;
	int ret;

	for (i = 0; i < ARRAY_SIZE(cs->patch); i++) {
		struct consys_patch *p = &cs->patch[i];
		const u8 *h;

		snprintf(p->name, sizeof(p->name), "%s_%s_%s_hdr.bin", fw_prefix, p->part, fw_variant);
		p->verified = false;
		ret = request_firmware(&p->fw, p->name, cs->dev);
		if (ret) {
			dev_err(cs->dev, "patch %s: request_firmware failed: %d\n", p->name, ret);
			goto err;
		}
		if (p->fw->size <= PATCH_HDR_LEN) {
			dev_err(cs->dev, "patch %s: too short (%zu)\n", p->name, p->fw->size);
			ret = -EINVAL;
			goto err;
		}
		h = p->fw->data;
		memcpy(p->date, h, PATCH_BUILDTIME_LEN);
		p->date[PATCH_BUILDTIME_LEN] = '\0';
		p->hw_ver = (h[0x14] << 8) | h[0x15];
		p->sw_ver = (h[0x16] << 8) | h[0x17];
		p->emi_off = (h[0x1a] << 16) | (h[0x19] << 8);
		p->size = p->fw->size - PATCH_HDR_LEN;
		if (h[0x1f] != p->type || h[0x1b] != CONN_EMI_VIEW_BYTE3) {
			dev_err(cs->dev, "patch %s: unexpected header (type %u, addr byte3 0x%02x)\n",
				p->name, h[0x1f], h[0x1b]);
			ret = -EINVAL;
			goto err;
		}
		/* same bound as the vendor driver: offset + size < EMI size */
		if (p->emi_off + p->size >= cs->emi_size ||
		    p->buildtime_off + PATCH_BUILDTIME_LEN > cs->emi_size) {
			dev_err(cs->dev, "patch %s: does not fit into the EMI region\n", p->name);
			ret = -EFBIG;
			goto err;
		}
		p->crc = crc32_le(~0, h + PATCH_HDR_LEN, p->size);
		dev_info(cs->dev, "patch %s: date %.15s hw 0x%04x sw 0x%04x type %u emi_off 0x%06x size 0x%x\n",
			 p->name, p->date, p->hw_ver, p->sw_ver, p->type, p->emi_off, p->size);
	}
	return 0;
err:
	consys_release_patches(cs);
	return ret;
}

static u32 consys_emi_crc(struct consys *cs, u32 off, u32 len)
{
	u8 buf[256];
	u32 crc = ~0;

	while (len) {
		u32 n = min_t(u32, len, sizeof(buf));

		memcpy_fromio(buf, cs->emi + off, n);
		crc = crc32_le(crc, buf, n);
		off += n;
		len -= n;
	}
	return crc;
}

/* step 12: copy the patch bodies into the EMI (vendor order: BT, WIFI, MCU) */
static int consys_copy_patches(struct consys *cs, u32 chip_fw_ver)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(cs->patch); i++) {
		struct consys_patch *p = &cs->patch[i];
		char bt[PATCH_BUILDTIME_LEN];

		if (fw_vercheck && p->sw_ver != chip_fw_ver) {
			dev_err(cs->dev, "patch %s: SW version 0x%04x != chip FW version 0x%04x\n",
				p->name, p->sw_ver, chip_fw_ver);
			return -EINVAL;
		}
		memcpy_toio(cs->emi + p->emi_off, p->fw->data + PATCH_HDR_LEN, p->size);
		/* debug copy of the build time, byte 15 cleared like the vendor driver does */
		memcpy(bt, p->date, PATCH_BUILDTIME_LEN);
		bt[PATCH_BUILDTIME_LEN - 1] = '\0';
		memcpy_toio(cs->emi + p->buildtime_off, bt, PATCH_BUILDTIME_LEN);
		wmb();
		p->verified = consys_emi_crc(cs, p->emi_off, p->size) == p->crc;
		dev_info(cs->dev, "patch %s -> EMI 0x%pa + 0x%06x (0x%x bytes), read-back %s\n",
			 p->name, &cs->emi_phys, p->emi_off, p->size, p->verified ? "ok" : "MISMATCH");
		if (!p->verified)
			return -EIO;
	}
	return 0;
}

/* ---------------------------------------------------------------- power sequence */

static int consys_mtcmos_wait_on(struct consys *cs, bool on)
{
	u32 sta, sta2;
	int ret;

	ret = regmap_read_poll_timeout(cs->scpsys, SPM_PWR_STATUS, sta,
				       !!(sta & SPM_CONN_STA) == on, 10, MTCMOS_TIMEOUT_US);
	if (!ret)
		ret = regmap_read_poll_timeout(cs->scpsys, SPM_PWR_STATUS_2ND, sta2,
					       !!(sta2 & SPM_CONN_STA) == on, 10, MTCMOS_TIMEOUT_US);
	if (ret) {
		regmap_read(cs->scpsys, SPM_PWR_STATUS, &sta);
		regmap_read(cs->scpsys, SPM_PWR_STATUS_2ND, &sta2);
		dev_err(cs->dev, "CONN MTCMOS: status 0x%08x/0x%08x, expected bit 1 = %d\n",
			sta, sta2, on);
	}
	return ret;
}

/* CONN power domain on/off through runtime PM; the status registers are only read */
static int consys_genpd_on(struct consys *cs)
{
	u32 before = 0, after = 0;
	int ret;

	if (cs->scpsys)
		regmap_read(cs->scpsys, SPM_CONN_PWR_CON, &before);
	ret = pm_runtime_resume_and_get(cs->dev);
	if (ret) {
		dev_err(cs->dev, "CONN power domain on failed: %d\n", ret);
		return ret;
	}
	cs->rpm_held = true;
	if (cs->scpsys) {
		regmap_read(cs->scpsys, SPM_CONN_PWR_CON, &after);
		consys_log_write(cs, "CONN_PWR_CON via genpd", 0x10006000 + SPM_CONN_PWR_CON,
				 before, after);
		return consys_mtcmos_wait_on(cs, true);
	}
	return 0;
}

static int consys_genpd_off(struct consys *cs)
{
	int ret;

	if (cs->rpm_held) {
		cs->rpm_held = false;
		ret = pm_runtime_put_sync(cs->dev);
	} else {
		/* retry after a failed power-off */
		ret = pm_runtime_suspend(cs->dev);
	}
	if (ret < 0)
		dev_err(cs->dev, "CONN power domain off: %d\n", ret);
	if (cs->scpsys)
		return consys_mtcmos_wait_on(cs, false);
	return ret < 0 ? ret : 0;
}

/* vendor spm_mtcmos_ctrl_conn(STA_POWER_ON) */
static int consys_mtcmos_on(struct consys *cs)
{
	static const struct { u32 clr, set; const char *name; } steps[] = {
		{ 0, PWR_ON, "CONN_PWR_CON PWR_ON" },
		{ 0, PWR_ON_2ND, "CONN_PWR_CON PWR_ON_2ND" },
		{ PWR_CLK_DIS, 0, "CONN_PWR_CON CLK_DIS=0" },
		{ PWR_ISO, 0, "CONN_PWR_CON ISO=0" },
		{ 0, PWR_RST_B, "CONN_PWR_CON RST_B=1" },
	};
	unsigned int i;
	int ret;

	if (cs->genpd)
		return consys_genpd_on(cs);
	for (i = 0; i < ARRAY_SIZE(steps); i++) {
		ret = consys_regmap_rmw(cs, cs->scpsys, 0x10006000, SPM_CONN_PWR_CON,
					steps[i].clr | steps[i].set, steps[i].set, steps[i].name);
		if (ret)
			return ret;
		if (i == 1) {
			ret = consys_mtcmos_wait_on(cs, true);
			if (ret)
				return ret;
		}
	}
	/* release bus protection: step 2 (TOPAXI 14, TOPAXI_1 10), then step 1 (TOPAXI 13|18) */
	ret = regmap_write(cs->infracfg, INFRA_TOPAXI_PROT_CLR, CONN_PROT_STEP2_0);
	if (!ret)
		ret = regmap_write(cs->infracfg, INFRA_TOPAXI_PROT_1_CLR, CONN_PROT_STEP2_1);
	if (!ret)
		ret = regmap_write(cs->infracfg, INFRA_TOPAXI_PROT_CLR, CONN_PROT_STEP1);
	return ret;
}

static int consys_prot_set(struct consys *cs, u32 set, u32 sta, u32 mask, const char *name)
{
	u32 v;
	int ret;

	ret = regmap_write(cs->infracfg, set, mask);
	if (ret)
		return ret;
	ret = regmap_read_poll_timeout(cs->infracfg, sta, v, (v & mask) == mask, 10,
				       MTCMOS_TIMEOUT_US);
	if (ret)
		dev_err(cs->dev, "bus protection %s: no ack (0x%08x)\n", name, v);
	return ret;
}

/* vendor spm_mtcmos_ctrl_conn(STA_POWER_DOWN) */
static int consys_mtcmos_off(struct consys *cs)
{
	int ret;

	if (cs->genpd)
		return consys_genpd_off(cs);
	ret = consys_prot_set(cs, INFRA_TOPAXI_PROT_SET, INFRA_TOPAXI_PROT_STA1, CONN_PROT_STEP1,
			      "step 1");
	if (!ret)
		ret = consys_prot_set(cs, INFRA_TOPAXI_PROT_SET, INFRA_TOPAXI_PROT_STA1,
				      CONN_PROT_STEP2_0, "step 2.0");
	if (!ret)
		ret = consys_prot_set(cs, INFRA_TOPAXI_PROT_1_SET, INFRA_TOPAXI_PROT_STA1_1,
				      CONN_PROT_STEP2_1, "step 2.1");
	if (ret)
		return ret;	/* no ack: leave the domain powered rather than risk a bus hang */
	regmap_update_bits(cs->scpsys, SPM_CONN_PWR_CON, PWR_ISO, PWR_ISO);
	regmap_update_bits(cs->scpsys, SPM_CONN_PWR_CON, PWR_CLK_DIS, PWR_CLK_DIS);
	regmap_update_bits(cs->scpsys, SPM_CONN_PWR_CON, PWR_RST_B, 0);
	regmap_update_bits(cs->scpsys, SPM_CONN_PWR_CON, PWR_ON, 0);
	regmap_update_bits(cs->scpsys, SPM_CONN_PWR_CON, PWR_ON_2ND, 0);
	return consys_mtcmos_wait_on(cs, false);
}

/* the vendor WMT core switches VCN33_2 (efuse) off after the RF calibration */
void consys_vcn33_2_off(struct consys *cs)
{
	if (cs->on & ON_VCN33_2) {
		regulator_disable(cs->vcn33_2);
		cs->on &= ~ON_VCN33_2;
		dev_info(cs->dev, "VCN33_2 off (after calibration)\n");
	}
}

static int consys_enable_supply(struct consys *cs, struct regulator *reg, int uv, const char *name)
{
	int ret;

	ret = regulator_set_voltage(reg, uv, uv);
	if (ret) {
		dev_err(cs->dev, "%s: set %d uV failed: %d\n", name, uv, ret);
		return ret;
	}
	ret = regulator_enable(reg);
	if (ret)
		dev_err(cs->dev, "%s: enable failed: %d\n", name, ret);
	return ret;
}

/*
 * Vendor wmt_func_gps_on/off() with co-clock and the host LNA pin (WMT_SOC.cfg co_clock_flag=1,
 * wmt_gps_lna_enable=0): GPS_PALDO = consys_hw_vcn28_ctrl(), which is VCN33_2 at 2.8 V on MT6785,
 * on while GNSS runs. The vendor additionally hands the LDO to SRCLKEN0 in low-power mode; like
 * the power-on here it stays under software control.
 */
static int consys_gps_vcn33_2(struct consys *cs, bool on)
{
	int ret;

	if (on == cs->gps_vcn33_2 || (on && !cs->co_clock))
		return 0;
	if (!on) {
		regulator_disable(cs->vcn33_2);
		cs->gps_vcn33_2 = false;
		return 0;
	}
	ret = consys_enable_supply(cs, cs->vcn33_2, VCN33_2_UV, "vcn33-2 (GNSS)");
	if (!ret)
		cs->gps_vcn33_2 = true;
	return ret;
}

/*
 * Power-off in the order of the vendor driver (mtk_wcn_consys_hw_reg_ctrl(0) and
 * consys_hw_power_ctrl(0)); only the steps recorded in cs->on are undone, so this is also the
 * error path of a partial power-on. VCN33_2 is switched off first: in the vendor flow it is
 * already off at this point (the WMT core turns it off after calibration).
 */
static int consys_power_off_hw(struct consys *cs)
{
	int ret;

	if (cs->on & ON_VCN33_2) {
		regulator_disable(cs->vcn33_2);
		cs->on &= ~ON_VCN33_2;
	}
	consys_gps_vcn33_2(cs, false);
	if (cs->on & ON_CON_STA) {
		regmap_write(cs->pericfg, PERI_CON_STA, 0);
		cs->on &= ~ON_CON_STA;
	}
	if (cs->on & ON_MTCMOS) {
		ret = consys_mtcmos_off(cs);
		if (ret) {
			/* keep CCIF clock, pins and supplies on as well: never cut a hung bus */
			dev_err(cs->dev, "CONN MTCMOS off failed: %d, consys left powered\n", ret);
			return ret;
		}
		cs->on &= ~ON_MTCMOS;
	}
	if (cs->on & ON_CCIF) {
		/* let ongoing CCIF tx interrupts settle, then clear the CCIF4 ACK status */
		udelay(100);
		writel(0xff, cs->io[IO_PCCIF4] + PCCIF4_ACK);
		clk_disable_unprepare(cs->ccif_clk);
		cs->on &= ~ON_CCIF;
	}
	/* clock buffer XO_WCN: no register access on MT6785 in the vendor driver either */
	if (cs->on & ON_PINS) {
		ret = pinctrl_select_state(cs->pinctrl, cs->pins_off);
		if (ret)
			dev_err(cs->dev, "pinctrl off state failed: %d\n", ret);
		cs->on &= ~ON_PINS;
	}
	if (cs->on & ON_VCN33_1) {
		regulator_disable(cs->vcn33_1);
		cs->on &= ~ON_VCN33_1;
	}
	udelay(300);
	if (cs->on & ON_VCN18) {
		regulator_disable(cs->vcn18);
		cs->on &= ~ON_VCN18;
	}
	return 0;
}

static int consys_power_on(struct consys *cs)
{
	struct device *dev = cs->dev;
	const char *step;
	u32 v;
	int ret, i;

	cs->t_start = ktime_get();
	cs->nsteps = 0;
	cs->nwrites = 0;
	cs->snap_valid = false;
	cs->ip_ver = cs->hw_ver = cs->fw_ver = cs->conf_id = cs->ready = 0;
	cs->chipid_polls = cs->ready_polls = 0;
	cs->fail_step = NULL;

	step = "request patches";
	ret = consys_request_patches(cs);
	if (ret)
		goto fail_nohw;
	consys_step(cs, step);

	/* P4: clear the consys EMI (vendor: at probe, before the first power-on) */
	step = "clear EMI";
	memset_io(cs->emi, 0, cs->emi_size);
	wmb();
	consys_step(cs, step);

	/* P6: EMI and PERI remap (vendor: at probe); MD remap needs CCCI and stays untouched */
	step = "EMI remap";
	ret = consys_regmap_rmw(cs, cs->infracfg, 0x10001000, INFRA_CONN_EMI_REMAP,
				INFRA_REMAP_MASK, (cs->emi_phys >> 16) & INFRA_REMAP_MASK,
				"CONN_EMI_REMAP");
	if (!ret)
		ret = consys_regmap_rmw(cs, cs->infracfg, 0x10001000, INFRA_CONN_PERI_REMAP,
					INFRA_REMAP_MASK, CONN_PERI_REMAP_VAL, "CONN_PERI_REMAP");
	if (ret)
		goto fail;
	consys_step(cs, step);

	/* 1: VCN18 1.8 V (vendor LP/SRCLKEN0 settings are not applied, see binding notes) */
	step = "VCN18 on";
	ret = consys_enable_supply(cs, cs->vcn18, VCN18_UV, "vcn18");
	if (ret)
		goto fail;
	cs->on |= ON_VCN18;
	consys_step(cs, step);

	/* 2: VCN33_1 3.3 V */
	step = "VCN33_1 on";
	ret = consys_enable_supply(cs, cs->vcn33_1, VCN33_1_UV, "vcn33-1");
	if (ret)
		goto fail;
	cs->on |= ON_VCN33_1;
	consys_step(cs, step);

	/* 3: A-die interface pins: CONN function, 2 mA */
	step = "pinmux on";
	ret = pinctrl_select_state(cs->pinctrl, cs->pins_on);
	if (ret)
		goto fail;
	cs->on |= ON_PINS;
	consys_step(cs, step);

	/* 4 */
	udelay(150);

	/* 5: clock buffer XO_WCN: the vendor call has no register effect on MT6785 */

	/* 6: VCN33_2 2.8 V (co-clock: needed for the efuse read; also on without co-clock) */
	step = "VCN33_2 on";
	ret = consys_enable_supply(cs, cs->vcn33_2, VCN33_2_UV, "vcn33-2");
	if (ret)
		goto fail;
	cs->on |= ON_VCN33_2;
	consys_step(cs, step);

	/* 7: hold the CONN MCU in reset (TOPRGU SWSYSRST bit 12, key 0x88) */
	step = "CONN MCU reset assert";
	ret = reset_control_assert(cs->mcu_rst);
	if (ret)
		goto fail;
	consys_step(cs, step);

	/* 8: CCIF4 clock, then CONN MTCMOS (incl. bus protection release) */
	step = "CCIF4_AP clock on";
	ret = clk_prepare_enable(cs->ccif_clk);
	if (ret)
		goto fail;
	cs->on |= ON_CCIF;
	consys_step(cs, step);

	step = "CONN MTCMOS on";
	cs->on |= ON_MTCMOS;	/* also undo a partial power-on */
	ret = consys_mtcmos_on(cs);
	if (ret)
		goto fail;
	consys_step(cs, step);

	/* 9: CON_STA_REG[1:0] = 1 (CON_PWR_ON) */
	step = "CON_STA";
	ret = consys_regmap_rmw(cs, cs->pericfg, 0x10003000, PERI_CON_STA, PERI_CON_STA_MASK,
				PERI_CON_STA_PWR_ON, "CON_STA_REG");
	if (ret)
		goto fail;
	cs->on |= ON_CON_STA;
	consys_step(cs, step);

	/* 10 */
	udelay(10);

	/* 11: chip id */
	step = "chip id poll";
	for (i = 0; i < CHIPID_POLL_TRIES; i++) {
		cs->chipid_polls++;
		v = readl(cs->io[IO_TOP_MISC_OFF] + TOP_MISC_OFF_IP_VER);
		if (v == CONSYS_IP_VER_MT6785)
			break;
		msleep(POLL_SLEEP_MS);
	}
	cs->ip_ver = v;
	if (v != CONSYS_IP_VER_MT6785) {
		dev_err(dev, "chip id: 0x%08x after %u reads (expected 0x%08x)\n", v,
			cs->chipid_polls, CONSYS_IP_VER_MT6785);
		ret = -ETIMEDOUT;
		goto fail;
	}
	cs->hw_ver = readl(cs->io[IO_MCU] + MCU_HW_VER);
	cs->fw_ver = readl(cs->io[IO_MCU] + MCU_FW_VER);
	consys_step(cs, step);
	dev_info(dev, "chip id 0x%08x after %u reads, HW ver 0x%04x, FW ver 0x%04x\n",
		 v, cs->chipid_polls, cs->hw_ver & 0xffff, cs->fw_ver & 0xffff);

	/* 12: RAM patches into the EMI */
	step = "patch copy";
	ret = consys_copy_patches(cs, cs->fw_ver & 0xffff);
	if (ret)
		goto fail;
	consys_step(cs, step);

	/* 13: CONN fine tuning */
	step = "CONN settings";
	cs->conf_id = readl(cs->io[IO_TOP_MISC_OFF] + TOP_MISC_OFF_CONF_ID);
	cs->hw_ver = readl(cs->io[IO_MCU] + MCU_HW_VER);
	cs->fw_ver = readl(cs->io[IO_MCU] + MCU_FW_VER);
	dev_info(dev, "conf id 0x%x, HW ver 0x%04x, FW ver 0x%04x\n", cs->conf_id & 0xf,
		 cs->hw_ver & 0xffff, cs->fw_ver & 0xffff);
	if (cs->co_clock) {
		consys_rmw(cs, IO_TOP_MISC_ON, TOP_MISC_ON_COCLK_STABLE, COCLK_STABLE_TIME_MASK,
			   COCLK_STABLE_TIME, "CONN_COCLOCK_STABLE_TIME");
		consys_rmw(cs, IO_TOP_MISC_ON, TOP_MISC_ON_COCLK_ACK, COCLK_ACK_CLR_MASK,
			   COCLK_ACK_VAL, "CONN_COCLOCK_ACK_EN");
	}
	consys_rmw(cs, IO_HIF_PDMA, PDMA_SLPPROT_CTRL, PDMA_SLPPROT_CONN2EMI,
		   PDMA_SLPPROT_CONN2EMI, "CONN_HIF_PDMA_SLPPROT");
	consys_rmw(cs, IO_TOP_MISC_ON, TOP_MISC_ON_TOPPOSE, TOPPOSE_RESTORE_DONE,
		   TOPPOSE_RESTORE_DONE, "CONN_TOPPOSE_RESTORE");
	consys_rmw(cs, IO_AFE, AFE_WBG_PLL_03, AFE_WPLL_MASK, AFE_WPLL_VAL, "CONN_AFE_WBG_PLL_03");
	consys_step(cs, step);

	/* 14: ACR/AFE: "No need" on MT6785 */

	/* 15: release the CONN MCU and wait for the ROM/RAM code to report ready */
	step = "CONN MCU reset deassert";
	ret = reset_control_deassert(cs->mcu_rst);
	if (ret)
		goto fail;
	consys_step(cs, step);

	step = "ready poll";
	for (i = 0; i < READY_POLL_TRIES; i++) {
		cs->ready_polls++;
		v = readl(cs->io[IO_MCU] + MCU_BOOT_STA);
		if (v == MCU_BOOT_READY)
			break;
		msleep(POLL_SLEEP_MS);
	}
	cs->ready = v;
	if (v != MCU_BOOT_READY) {
		dev_err(dev, "CONN MCU not ready: 0x18002600 = 0x%08x after %u reads, 0x1800216c = 0x%08x, CPUPCR = 0x%08x\n",
			v, cs->ready_polls, readl(cs->io[IO_MCU] + MCU_STA_16C),
			readl(cs->io[IO_HIF_ON] + HIF_ON_CPUPCR));
		ret = -ETIMEDOUT;
		goto fail;
	}
	consys_step(cs, step);
	dev_info(dev, "CONN MCU ready (0x%04x) after %u reads, 0x1800216c = 0x%08x, CPUPCR = 0x%08x\n",
		 v, cs->ready_polls, readl(cs->io[IO_MCU] + MCU_STA_16C),
		 readl(cs->io[IO_HIF_ON] + HIF_ON_CPUPCR));

	/* 16: settle time at the end of the vendor power-on */
	msleep(20);
	consys_step(cs, "done");
	consys_snapshot(cs);
	consys_release_patches(cs);

	cs->state = CONSYS_ON;
	cs->last_err = 0;
	dev_info(dev, "power on done in %lld us\n", ktime_us_delta(ktime_get(), cs->t_start));
	return 0;

fail:
	if (cs->on & ON_MTCMOS)
		consys_snapshot(cs);
	if (consys_power_off_hw(cs))
		dev_err(dev, "power on failed and the consys could NOT be switched off (undo mask 0x%02x)\n",
			cs->on);
	consys_release_patches(cs);
fail_nohw:
	cs->state = CONSYS_FAILED;
	cs->last_err = ret;
	cs->fail_step = step;
	dev_err(dev, "power on failed in step '%s': %d%s\n", step, ret,
		cs->on ? "" : ", consys switched off again");
	return ret;
}

static int consys_power_off(struct consys *cs)
{
	consys_wmt_close(cs);
	int ret = consys_power_off_hw(cs);

	if (ret) {
		cs->state = CONSYS_FAILED;
		cs->last_err = ret;
		cs->fail_step = "power off";
		return ret;
	}
	cs->state = CONSYS_OFF;
	dev_info(cs->dev, "power off done\n");
	return 0;
}

/* ---------------------------------------------------------------- sysfs / debugfs */

static ssize_t power_on_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct consys *cs = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", cs->state == CONSYS_ON);
}

static ssize_t power_on_store(struct device *dev, struct device_attribute *attr, const char *buf,
			   size_t count)
{
	struct consys *cs = dev_get_drvdata(dev);
	bool on;
	int ret;

	ret = kstrtobool(buf, &on);
	if (ret)
		return ret;

	mutex_lock(&cs->lock);
	if (on && cs->state != CONSYS_ON)
		/* after a failed power-off something is still on: do not stack a new power-on */
		ret = cs->on ? -EBUSY : consys_power_on(cs);
	else if (!on && consys_func_busy(cs))
		ret = -EBUSY;
	else if (!on && cs->on)
		ret = consys_power_off(cs);
	mutex_unlock(&cs->lock);

	return ret ? ret : count;
}
static DEVICE_ATTR_RW(power_on);

static ssize_t wmt_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct consys *cs = dev_get_drvdata(dev);

	return sysfs_emit(buf, "%d\n", cs->wmt.ready);
}

/*
 * "1" powers the consys on if needed, opens the BTIF and runs the WMT start-up sequence; on any
 * error everything is switched off again. "0" closes the BTIF and switches the consys off.
 */
static ssize_t wmt_store(struct device *dev, struct device_attribute *attr, const char *buf,
			 size_t count)
{
	struct consys *cs = dev_get_drvdata(dev);
	bool on;
	int ret;

	ret = kstrtobool(buf, &on);
	if (ret)
		return ret;

	mutex_lock(&cs->lock);
	if (on && !cs->wmt.ready) {
		if (cs->state != CONSYS_ON)
			ret = cs->on ? -EBUSY : consys_power_on(cs);
		if (!ret) {
			ret = consys_wmt_start(cs);
			if (ret)
				consys_power_off(cs);
		}
	} else if (!on && consys_func_busy(cs)) {
		ret = -EBUSY;
	} else if (!on && cs->on) {
		ret = consys_power_off(cs);
	}
	mutex_unlock(&cs->lock);

	return ret ? ret : count;
}
static DEVICE_ATTR_RW(wmt);

static struct attribute *consys_attrs[] = {
	&dev_attr_power_on.attr,
	&dev_attr_wmt.attr,
	NULL
};
ATTRIBUTE_GROUPS(consys);

static void consys_show_supply(struct seq_file *s, const char *name, struct regulator *reg)
{
	seq_printf(s, "  %-8s enabled=%d voltage=%d uV\n", name, regulator_is_enabled(reg),
		   regulator_get_voltage(reg));
}

/*
 * Non-zero 32-bit words per 64 KiB block of the EMI region. The power-on clears it and copies
 * the RAM patches (mcu 0x000000, bt 0x080000, wifi 0x140000); everything else was written by
 * the consys firmware (vendor: debug/coredump area from 0x068000).
 */
static void consys_emi_summary(struct consys *cs, struct seq_file *s)
{
	unsigned int blk, nblk = cs->emi_size / SZ_64K, i;

	seq_puts(s, "EMI non-zero words per 64 KiB block (offset>>16:count):\n ");
	for (blk = 0, i = 0; blk < nblk; blk++) {
		const u32 __iomem *p = cs->emi + blk * SZ_64K;
		unsigned int w, cnt = 0;

		for (w = 0; w < SZ_64K / 4; w++)
			cnt += !!readl_relaxed(p + w);
		if (!cnt)
			continue;
		seq_printf(s, " %02x:%u", blk, cnt);
		if (++i % 10 == 0)
			seq_puts(s, "\n ");
	}
	seq_puts(s, i ? "\n" : " none\n");
}

static int consys_status_show(struct seq_file *s, void *unused)
{
	static const char * const states[] = { "off", "on", "failed" };
	struct consys *cs = s->private;
	unsigned int i;
	u32 v;

	mutex_lock(&cs->lock);
	seq_printf(s, "state: %s\n", states[cs->state]);
	if (cs->state == CONSYS_FAILED)
		seq_printf(s, "last error: %d in step '%s'\n", cs->last_err,
			   cs->fail_step ? cs->fail_step : "?");
	seq_printf(s, "co-clock: %d\n", cs->co_clock);
	seq_printf(s, "EMI: %pa size 0x%zx\n", &cs->emi_phys, cs->emi_size);

	seq_puts(s, "AP-side registers (always readable):\n");
	if (!regmap_read(cs->infracfg, INFRA_CONN_EMI_REMAP, &v))
		seq_printf(s, "  0x10001380 CONN_EMI_REMAP   = 0x%08x\n", v);
	if (!regmap_read(cs->infracfg, INFRA_CONN_PERI_REMAP, &v))
		seq_printf(s, "  0x10001388 CONN_PERI_REMAP  = 0x%08x\n", v);
	if (!regmap_read(cs->infracfg, INFRA_CONN_MD_REMAP, &v))
		seq_printf(s, "  0x1000139c CONN_MD_REMAP    = 0x%08x\n", v);
	if (!regmap_read(cs->pericfg, PERI_CON_STA, &v))
		seq_printf(s, "  0x10003438 CON_STA_REG      = 0x%08x\n", v);

	seq_puts(s, "supplies:\n");
	consys_show_supply(s, "vcn18", cs->vcn18);
	consys_show_supply(s, "vcn33-1", cs->vcn33_1);
	consys_show_supply(s, "vcn33-2", cs->vcn33_2);
	seq_printf(s, "  vcn33-2 held for GNSS: %d\n", cs->gps_vcn33_2);
	seq_printf(s, "  CONN MTCMOS via %s, runtime PM %s%s\n",
		   cs->genpd ? "power domain" : "scpsys syscon",
		   cs->genpd ? (pm_runtime_active(cs->dev) ? "active" : "suspended") : "-",
		   cs->rpm_held ? " (held)" : "");
	if (cs->scpsys && !regmap_read(cs->scpsys, SPM_CONN_PWR_CON, &v))
		seq_printf(s, "  0x10006320 CONN_PWR_CON     = 0x%08x\n", v);
	if (cs->scpsys && !regmap_read(cs->scpsys, SPM_PWR_STATUS, &v))
		seq_printf(s, "  0x10006160 PWR_STATUS       = 0x%08x\n", v);
	if (cs->scpsys && !regmap_read(cs->scpsys, SPM_PWR_STATUS_2ND, &v))
		seq_printf(s, "  0x10006164 PWR_STATUS_2ND   = 0x%08x\n", v);
	if (cs->scpsys && !regmap_read(cs->scpsys, SPM_POWERON_CONFIG_EN, &v))
		seq_printf(s, "  0x10006000 POWERON_CONFIG_EN= 0x%08x\n", v);
	if (!regmap_read(cs->infracfg, INFRA_TOPAXI_PROT_STA1, &v))
		seq_printf(s, "  0x10001228 TOPAXI_PROT_STA1 = 0x%08x\n", v);
	if (!regmap_read(cs->infracfg, INFRA_TOPAXI_PROT_STA1_1, &v))
		seq_printf(s, "  0x10001258 TOPAXI_PROT_STA1_1 = 0x%08x\n", v);
	seq_printf(s, "ccif4 clock: %d, CONN MTCMOS: %d, undo mask 0x%02x\n",
		   !!(cs->on & ON_CCIF), !!(cs->on & ON_MTCMOS), cs->on);

	if (cs->state == CONSYS_ON) {
		seq_puts(s, "CONN registers (live):\n");
		for (i = 0; i < ARRAY_SIZE(consys_dump_regs); i++)
			seq_printf(s, "  0x%08llx %-28s = 0x%08x\n",
				   (u64)consys_io_phys[consys_dump_regs[i].io] + consys_dump_regs[i].off,
				   consys_dump_regs[i].name,
				   readl(cs->io[consys_dump_regs[i].io] + consys_dump_regs[i].off));
	}
	if (cs->snap_valid) {
		seq_printf(s, "CONN registers (snapshot at end of last power-on%s):\n",
			   cs->state == CONSYS_FAILED ? ", before switching off" : "");
		for (i = 0; i < ARRAY_SIZE(consys_dump_regs); i++)
			seq_printf(s, "  0x%08llx %-28s = 0x%08x\n",
				   (u64)consys_io_phys[consys_dump_regs[i].io] + consys_dump_regs[i].off,
				   consys_dump_regs[i].name, cs->snap[i]);
	}

	if (cs->nsteps) {
		seq_printf(s, "last power-on: chip id reads %u (ip 0x%08x, hw 0x%04x, fw 0x%04x, conf 0x%x), ready reads %u (0x%08x)\n",
			   cs->chipid_polls, cs->ip_ver, cs->hw_ver & 0xffff, cs->fw_ver & 0xffff,
			   cs->conf_id & 0xf, cs->ready_polls, cs->ready);
		seq_puts(s, "register writes (before -> after):\n");
		for (i = 0; i < cs->nwrites; i++)
			seq_printf(s, "  0x%08x %-26s 0x%08x -> 0x%08x\n", cs->writes[i].addr,
				   cs->writes[i].name, cs->writes[i].before, cs->writes[i].after);
		seq_puts(s, "steps (us since start):\n");
		for (i = 0; i < cs->nsteps; i++)
			seq_printf(s, "  %8lld %s\n", cs->steps[i].us, cs->steps[i].name);
		seq_puts(s, "patches:\n");
		for (i = 0; i < ARRAY_SIZE(cs->patch); i++)
			seq_printf(s, "  %-32s date %.15s hw 0x%04x sw 0x%04x emi_off 0x%06x size 0x%06x crc32 0x%08x %s\n",
				   cs->patch[i].name, cs->patch[i].date, cs->patch[i].hw_ver,
				   cs->patch[i].sw_ver, cs->patch[i].emi_off, cs->patch[i].size,
				   cs->patch[i].crc, cs->patch[i].verified ? "verified" : "-");
	}
	consys_wmt_show(cs, s);
	consys_emi_summary(cs, s);
	mutex_unlock(&cs->lock);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(consys_status);

/* raw copy of the EMI region (plain DRAM, readable with the consys off), for offline analysis */
static ssize_t consys_emi_read(struct file *file, char __user *ubuf, size_t len, loff_t *ppos)
{
	struct consys *cs = file->private_data;
	loff_t pos = *ppos;
	size_t n;
	void *buf;
	ssize_t ret;

	if (pos < 0)
		return -EINVAL;
	if (pos >= cs->emi_size || !len)
		return 0;
	n = min3(len, (size_t)(cs->emi_size - pos), (size_t)PAGE_SIZE);
	buf = kmalloc(n, GFP_KERNEL);
	if (!buf)
		return -ENOMEM;
	memcpy_fromio(buf, cs->emi + pos, n);
	ret = copy_to_user(ubuf, buf, n) ? -EFAULT : n;
	kfree(buf);
	if (ret > 0)
		*ppos = pos + ret;
	return ret;
}

static const struct file_operations consys_emi_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = consys_emi_read,
	.llseek = default_llseek,
};

/* ---------------------------------------------------------------- function driver interface */

/**
 * connac1_get() - look up the consys of a function device
 * @dev: function device whose node has a "mediatek,consys" phandle
 *
 * Return: the consys or an ERR_PTR (-EPROBE_DEFER while the consys has not probed yet).
 * Release with connac1_put().
 */
struct consys *connac1_get(struct device *dev)
{
	struct platform_device *pdev;
	struct device_node *np;
	struct consys *cs;

	np = of_parse_phandle(dev->of_node, "mediatek,consys", 0);
	if (!np)
		return ERR_PTR(-ENODEV);
	pdev = of_find_device_by_node(np);
	of_node_put(np);
	if (!pdev)
		return ERR_PTR(-EPROBE_DEFER);
	cs = platform_get_drvdata(pdev);
	if (!cs || !READ_ONCE(cs->probed)) {
		put_device(&pdev->dev);
		return ERR_PTR(-EPROBE_DEFER);
	}
	if (!device_link_add(dev, &pdev->dev, DL_FLAG_AUTOREMOVE_CONSUMER))
		dev_warn(dev, "no device link to %s\n", dev_name(&pdev->dev));
	return cs;
}
EXPORT_SYMBOL_GPL(connac1_get);

/* EMI region of the consys (needed by the WLAN driver at probe, before anything is on) */
int connac1_get_emi(struct consys *cs, phys_addr_t *base, size_t *size)
{
	*base = cs->emi_phys;
	*size = cs->emi_size;
	return 0;
}
EXPORT_SYMBOL_GPL(connac1_get_emi);

void connac1_put(struct consys *cs)
{
	if (!IS_ERR_OR_NULL(cs))
		put_device(cs->dev);
}
EXPORT_SYMBOL_GPL(connac1_put);

/**
 * connac1_wifi_on() - make sure the consys runs and is awake for the WLAN function
 * @cs: consys from connac1_get()
 * @info: filled with the EMI region, versions and calibration data
 *
 * Powers the consys on and runs the WMT start-up if needed (about 1.4 s, incl. calibration).
 * Calls are counted; connac1_wifi_off() switches the consys off again after the last user
 * if it was switched on here.
 */
int connac1_wifi_on(struct consys *cs, struct connac1_wifi_info *info)
{
	struct consys_wmt *w = &cs->wmt;
	int ret = 0;

	mutex_lock(&cs->lock);
	if (!w->ready) {
		if (cs->state != CONSYS_ON)
			ret = cs->on ? -EBUSY : consys_power_on(cs);
		if (!ret) {
			ret = consys_wmt_start(cs);
			if (ret)
				consys_power_off(cs);
			else
				cs->func_powered = true;
		}
	}
	if (!ret)
		ret = consys_wmt_psm(cs, CONSYS_PSM_WAKEUP);
	if (!ret) {
		w->wifi_users++;
		info->consys_dev = cs->dev;
		info->emi_phys = cs->emi_phys;
		info->emi_size = cs->emi_size;
		info->hw_ver = cs->hw_ver & 0xffff;
		info->fw_ver = cs->fw_ver & 0xffff;
		info->cal[0] = w->cal[0];
		info->cal_len[0] = w->cal_len[0];
		info->cal[1] = w->cal[1];
		info->cal_len[1] = w->cal_len[1];
	}
	mutex_unlock(&cs->lock);
	if (ret)
		dev_err(cs->dev, "wifi on failed: %d\n", ret);
	return ret;
}
EXPORT_SYMBOL_GPL(connac1_wifi_on);

/* switch the consys off again once no function uses it, if a function switched it on; cs->lock held */
static void consys_func_idle(struct consys *cs)
{
	if (cs->func_powered && !consys_func_busy(cs)) {
		cs->func_powered = false;
		if (cs->on)
			consys_power_off(cs);
	}
}

void connac1_wifi_off(struct consys *cs)
{
	struct consys_wmt *w = &cs->wmt;

	mutex_lock(&cs->lock);
	if (w->wifi_users > 0)
		w->wifi_users--;
	consys_func_idle(cs);
	mutex_unlock(&cs->lock);
}
EXPORT_SYMBOL_GPL(connac1_wifi_off);

/**
 * connac1_bt_on() - switch the Bluetooth function on
 * @cs: consys from connac1_get()
 * @ops: receive callback for STP frames of type BT
 * @priv: passed to @ops
 *
 * Powers the consys on and runs the WMT start-up if needed (as connac1_wifi_on()), wakes it
 * and sends the WMT function control "BT on" (vendor wmt_func_bt_on(); the BT RAM patch is
 * already in the EMI since the power-on). One Bluetooth user at a time.
 */
int connac1_bt_on(struct consys *cs, const struct connac1_bt_ops *ops, void *priv)
{
	struct consys_wmt *w = &cs->wmt;
	int ret = 0;

	mutex_lock(&cs->lock);
	if (w->bt_on || w->bt_ops) {
		ret = -EBUSY;
		goto out;
	}
	if (!w->ready) {
		if (cs->state != CONSYS_ON)
			ret = cs->on ? -EBUSY : consys_power_on(cs);
		if (!ret) {
			ret = consys_wmt_start(cs);
			if (ret)
				consys_power_off(cs);
			else
				cs->func_powered = true;
		}
	}
	if (!ret)
		ret = consys_wmt_psm(cs, CONSYS_PSM_WAKEUP);
	if (!ret)
		ret = consys_wmt_bt(cs, true, ops, priv);
	if (ret) {
		dev_err(cs->dev, "bt on failed: %d\n", ret);
		consys_func_idle(cs);
	}
out:
	mutex_unlock(&cs->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(connac1_bt_on);

/**
 * connac1_bt_off() - switch the Bluetooth function off (WMT "BT off")
 * @cs: consys from connac1_get()
 *
 * No more frames reach the receive callback after this. The consys goes off if a function
 * switched it on and WLAN does not use it.
 */
void connac1_bt_off(struct consys *cs)
{
	mutex_lock(&cs->lock);
	if (cs->wmt.bt_on || cs->wmt.bt_ops)
		consys_wmt_bt(cs, false, NULL, NULL);
	consys_func_idle(cs);
	mutex_unlock(&cs->lock);
}
EXPORT_SYMBOL_GPL(connac1_bt_off);

/**
 * connac1_gps_on() - switch the GNSS function on
 * @cs: consys from connac1_get()
 * @ops: receive callback for STP frames of type GPS
 * @priv: passed to @ops
 * @info: filled with versions and the clock type (vendor ioctls of /dev/stpgps)
 *
 * Powers the consys on and runs the WMT start-up if needed (as connac1_wifi_on()), wakes it,
 * switches VCN33_2 on for the co-clock PALDO and sends the WMT function control "GPS on"
 * (vendor wmt_func_gps_on()). The external LNA is the caller's (the vendor switches it on
 * before this call). One GNSS user at a time.
 */
int connac1_gps_on(struct consys *cs, const struct connac1_gps_ops *ops, void *priv,
		   struct connac1_gps_info *info)
{
	struct consys_wmt *w = &cs->wmt;
	int ret = 0;

	mutex_lock(&cs->lock);
	if (w->gps_on || w->gps_ops) {
		ret = -EBUSY;
		goto out;
	}
	if (!w->ready) {
		if (cs->state != CONSYS_ON)
			ret = cs->on ? -EBUSY : consys_power_on(cs);
		if (!ret) {
			ret = consys_wmt_start(cs);
			if (ret)
				consys_power_off(cs);
			else
				cs->func_powered = true;
		}
	}
	if (!ret)
		ret = consys_wmt_psm(cs, CONSYS_PSM_WAKEUP);
	if (!ret)
		ret = consys_gps_vcn33_2(cs, true);
	if (!ret)
		ret = consys_wmt_gps(cs, true, ops, priv);
	if (ret) {
		dev_err(cs->dev, "gps on failed: %d\n", ret);
		consys_gps_vcn33_2(cs, false);
		consys_func_idle(cs);
		goto out;
	}
	info->hw_ver = cs->hw_ver & 0xffff;
	info->fw_ver = cs->fw_ver & 0xffff;
	info->co_clock = cs->co_clock;
out:
	mutex_unlock(&cs->lock);
	return ret;
}
EXPORT_SYMBOL_GPL(connac1_gps_on);

/**
 * connac1_gps_off() - switch the GNSS function off (WMT "GPS off", then VCN33_2)
 * @cs: consys from connac1_get()
 *
 * No more frames reach the receive callback after this. The consys goes off if a function
 * switched it on and no other function uses it.
 */
void connac1_gps_off(struct consys *cs)
{
	mutex_lock(&cs->lock);
	if (cs->wmt.gps_on || cs->wmt.gps_ops)
		consys_wmt_gps(cs, false, NULL, NULL);
	consys_gps_vcn33_2(cs, false);
	consys_func_idle(cs);
	mutex_unlock(&cs->lock);
}
EXPORT_SYMBOL_GPL(connac1_gps_off);

/* ---------------------------------------------------------------- probe */

static int consys_get_emi(struct consys *cs)
{
	struct resource res;
	int ret;

	ret = of_reserved_mem_region_to_resource(cs->dev->of_node, 0, &res);
	if (ret)
		return dev_err_probe(cs->dev, ret, "no reserved consys EMI region\n");
	cs->emi_phys = res.start;
	cs->emi_size = resource_size(&res);
	/* the remap register holds bits [35:16]; the vendor DT asks for 16 MiB alignment */
	if (cs->emi_phys & 0xffff || cs->emi_size < SZ_2M)
		return dev_err_probe(cs->dev, -EINVAL, "unusable EMI region %pR\n", &res);
	cs->emi = devm_ioremap(cs->dev, cs->emi_phys, cs->emi_size);
	if (!cs->emi)
		return -ENOMEM;
	return 0;
}

static int consys_probe(struct platform_device *pdev)
{
	static const struct consys_patch patches[] = {
		{ .part = "ram_bt", .type = 0, .buildtime_off = 0x68300 },
		{ .part = "ram_wifi", .type = 3, .buildtime_off = 0x68310 },
		{ .part = "ram_mcu", .type = 4, .buildtime_off = 0x68320 },
	};
	struct device *dev = &pdev->dev;
	struct consys *cs;
	unsigned int i;
	int ret;

	/*
	 * The platform bus has just powered the CONN domain on (if there is one), with the
	 * supplies off and the CONN MCU not held in reset. Hand it back first thing.
	 */
	if (dev->pm_domain) {
		pm_runtime_set_active(dev);
		ret = devm_pm_runtime_enable(dev);
		if (ret)
			return ret;
		ret = pm_runtime_suspend(dev);
		if (ret < 0)
			return dev_err_probe(dev, ret, "CONN power domain off at probe\n");
	}

	cs = devm_kzalloc(dev, sizeof(*cs), GFP_KERNEL);
	if (!cs)
		return -ENOMEM;
	cs->dev = dev;
	cs->genpd = !!dev->pm_domain;
	mutex_init(&cs->lock);
	memcpy(cs->patch, patches, sizeof(patches));
	platform_set_drvdata(pdev, cs);

	/* mapping only; no CONN register is read before power-on */
	for (i = 0; i < IO_NUM; i++) {
		cs->io[i] = devm_platform_ioremap_resource_byname(pdev, consys_reg_names[i]);
		if (IS_ERR(cs->io[i]))
			return PTR_ERR(cs->io[i]);
	}

	ret = consys_get_emi(cs);
	if (ret)
		return ret;

	cs->infracfg = syscon_regmap_lookup_by_phandle(dev->of_node, "mediatek,infracfg");
	if (IS_ERR(cs->infracfg))
		return dev_err_probe(dev, PTR_ERR(cs->infracfg), "no infracfg\n");
	cs->pericfg = syscon_regmap_lookup_by_phandle(dev->of_node, "mediatek,pericfg");
	if (IS_ERR(cs->pericfg))
		return dev_err_probe(dev, PTR_ERR(cs->pericfg), "no pericfg\n");
	/* with the power domain only read (status); without it the MTCMOS is switched here */
	cs->scpsys = syscon_regmap_lookup_by_phandle_optional(dev->of_node, "mediatek,scpsys");
	if (IS_ERR(cs->scpsys))
		return dev_err_probe(dev, PTR_ERR(cs->scpsys), "scpsys syscon\n");
	if (!cs->scpsys && !cs->genpd)
		return dev_err_probe(dev, -ENODEV, "neither power-domains nor mediatek,scpsys\n");

	cs->vcn18 = devm_regulator_get(dev, "vcn18");
	if (IS_ERR(cs->vcn18))
		return dev_err_probe(dev, PTR_ERR(cs->vcn18), "no vcn18 supply\n");
	cs->vcn33_1 = devm_regulator_get(dev, "vcn33-1");
	if (IS_ERR(cs->vcn33_1))
		return dev_err_probe(dev, PTR_ERR(cs->vcn33_1), "no vcn33-1 supply\n");
	cs->vcn33_2 = devm_regulator_get(dev, "vcn33-2");
	if (IS_ERR(cs->vcn33_2))
		return dev_err_probe(dev, PTR_ERR(cs->vcn33_2), "no vcn33-2 supply\n");

	cs->ccif_clk = devm_clk_get(dev, "ccif");
	if (IS_ERR(cs->ccif_clk))
		return dev_err_probe(dev, PTR_ERR(cs->ccif_clk), "no ccif clock\n");

	cs->mcu_rst = devm_reset_control_get_exclusive(dev, "mcu");
	if (IS_ERR(cs->mcu_rst))
		return dev_err_probe(dev, PTR_ERR(cs->mcu_rst), "no mcu reset\n");

	cs->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(cs->pinctrl))
		return dev_err_probe(dev, PTR_ERR(cs->pinctrl), "no pinctrl\n");
	cs->pins_on = pinctrl_lookup_state(cs->pinctrl, "on");
	if (IS_ERR(cs->pins_on))
		return dev_err_probe(dev, PTR_ERR(cs->pins_on), "no pinctrl state 'on'\n");
	cs->pins_off = pinctrl_lookup_state(cs->pinctrl, "off");
	if (IS_ERR(cs->pins_off))
		return dev_err_probe(dev, PTR_ERR(cs->pins_off), "no pinctrl state 'off'\n");

	cs->co_clock = of_property_read_bool(dev->of_node, "mediatek,co-clock");

	ret = consys_wmt_init(cs, pdev);
	if (ret)
		return ret;

	cs->debugfs = debugfs_create_dir(dev_name(dev), NULL);
	debugfs_create_file("status", 0444, cs->debugfs, cs, &consys_status_fops);
	debugfs_create_file("emi", 0400, cs->debugfs, cs, &consys_emi_fops);
	consys_wmt_debugfs(cs);

	WRITE_ONCE(cs->probed, true);
	dev_info(dev, "consys EMI %pa size 0x%zx, co-clock %d, CONN MTCMOS %s; power on with: echo 1 > /sys/bus/platform/devices/%s/power_on\n",
		 &cs->emi_phys, cs->emi_size, cs->co_clock,
		 cs->genpd ? "via power domain" : "via scpsys syscon", dev_name(dev));
	return 0;
}

static void consys_remove(struct platform_device *pdev)
{
	struct consys *cs = platform_get_drvdata(pdev);

	debugfs_remove_recursive(cs->debugfs);
	mutex_lock(&cs->lock);
	if (cs->on)
		consys_power_off(cs);
	mutex_unlock(&cs->lock);
}

static void consys_shutdown(struct platform_device *pdev)
{
	struct consys *cs = platform_get_drvdata(pdev);

	/* leave the subsystem off for the next kernel (e.g. a warm reboot into the vendor kernel) */
	mutex_lock(&cs->lock);
	if (cs->on)
		consys_power_off(cs);
	mutex_unlock(&cs->lock);
}

/*
 * The consys runs its own firmware and stays up while the AP sleeps, as with the vendor
 * kernel: Bluetooth, WLAN and GNSS keep their state, and the WMT link continues after the
 * resume. genpd would switch the CONN domain off in the noirq phase of a system suspend no
 * matter whether the device is runtime active, and the functions would then talk to a consys
 * without firmware (no STP acknowledge after the resume). Put the device on the awake path
 * while the consys is on; the CONN domain is GENPD_FLAG_ACTIVE_WAKEUP and stays on then.
 */
static int consys_suspend(struct device *dev)
{
	struct consys *cs = dev_get_drvdata(dev);

	mutex_lock(&cs->lock);
	if (cs->genpd && cs->state == CONSYS_ON)
		device_set_awake_path(dev);
	mutex_unlock(&cs->lock);
	return 0;
}

static DEFINE_SIMPLE_DEV_PM_OPS(consys_pm_ops, consys_suspend, NULL);

static const struct of_device_id consys_of_match[] = {
	{ .compatible = "mediatek,mt6785-consys" },
	{ }
};
MODULE_DEVICE_TABLE(of, consys_of_match);

static struct platform_driver consys_driver = {
	.probe = consys_probe,
	.remove = consys_remove,
	.shutdown = consys_shutdown,
	.driver = {
		.name = "mtk-connac1-consys",
		.of_match_table = consys_of_match,
		.pm = pm_sleep_ptr(&consys_pm_ops),
		.dev_groups = consys_groups,
	},
};
module_platform_driver(consys_driver);

MODULE_FIRMWARE("soc1_0_ram_bt_2a_1_hdr.bin");
MODULE_FIRMWARE("soc1_0_ram_wifi_2a_1_hdr.bin");
MODULE_FIRMWARE("soc1_0_ram_mcu_2a_1_hdr.bin");
MODULE_DESCRIPTION("MediaTek CONNAC1 connectivity subsystem power control (MT6785)");
MODULE_LICENSE("GPL");
