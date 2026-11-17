// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (C) 2022 MediaTek Inc.
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/devfreq.h>
#include <linux/io.h>
#include <linux/math64.h>
#include <linux/minmax.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/pm_opp.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/sched.h>
#include <linux/sched/clock.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/trace_printk.h>

/* MT6785: last changes of the shared VPROC2 rail, see mtk_lisbon_cci_rail_event() */
#define LISBON_HIST		8
/* read-back budget after a transition */
#define LISBON_VERIFY_US	20000

struct mtk_lisbon_cci_ev {
	u64 ns;
	int old_uv;
	int new_uv;
	pid_t pid;
	char type;	/* p = before the change, d = done (ramp delay over), a = aborted */
	bool self;	/* changed by this driver */
	char comm[TASK_COMM_LEN];
};

/* one read-back: Vproc2 RG, DA, Vsram2 RG, DA, Vproc2 DA again (after Vsram2) */
enum { LISBON_RB_RG, LISBON_RB_DA, LISBON_RB_SRG, LISBON_RB_SDA, LISBON_RB_DA2, LISBON_RB_N };

struct mtk_lisbon_cci_probe {
	u64 t0, t1;		/* local_clock() around the register reads */
	unsigned int gen0, gen1;	/* rail_gen before and after */
	bool busy0;		/* a rail change was in flight at the start */
	bool stable;
	int ret;
	int uv[LISBON_RB_N];
};

struct mtk_ccifreq_platform_data {
	int min_volt_shift;
	int max_volt_shift;
	int proc_max_volt;
	int sram_min_volt;
	int sram_max_volt;
};

struct mtk_ccifreq_drv {
	struct device *dev;
	struct devfreq *devfreq;
	struct regulator *proc_reg;
	struct regulator *sram_reg;
	struct clk *cci_clk;
	struct clk *inter_clk;
	int inter_voltage;
	unsigned long pre_freq;
	/* Avoid race condition for regulators between notify and policy */
	struct mutex reg_lock;
	struct notifier_block opp_nb;
	const struct mtk_ccifreq_platform_data *soc_data;
	int vtrack_max;

	/* MT6785 voltage checks, see mtk_lisbon_cci_init() */
	bool lisbon;
	void __iomem *apmixed;
	void __iomem *mcusys;
	struct regmap *pmic;
	int max_volt;
	unsigned long boot_khz;
	unsigned int ntrans;
	struct notifier_block rail_nb;
	bool rail_nb_on;
	struct task_struct *rail_owner;	/* in mtk_ccifreq_target() */
	spinlock_t rail_lock;		/* rail_gen, rail_busy, rail_ev, rail_head */
	unsigned int rail_gen;		/* moves on every rail event */
	bool rail_busy;			/* PRE seen, VOLTAGE_CHANGE/ABORT not yet */
	unsigned int rail_head;
	struct mtk_lisbon_cci_ev rail_ev[LISBON_HIST];
	unsigned int nretry, maxretry, maxverify_us;
};

/*
 * MT6785 + MT6359: the CCI runs from CCIPLL (APMIXED CCIPLL_CON1 0x2a4) through the mcusys bus
 * divider (BUS_PLL_DIVIDER 0x2e0, mux [10:9], CKDIV1 [21:17]) and shares VPROC2/VSRAM_PROC2 with
 * cpu0-5; VSRAM follows by PMIC hardware tracking. PMIC registers from the MT6359 register map, the
 * same ones mediatek-cpufreq reads back.
 *
 * vcheck:  require readable registers and plausible boot values at probe;
 *          before Vproc goes up require VSRAM tracking (or Vsram already at
 *          the target); after every transition compare the clock read back
 *          from CCIPLL/mux with the target and require DA Vproc >= the OPP
 *          voltage and DA Vsram >= DA Vproc.
 * vcheck_seq: take the read-back only while no consumer of VPROC2 is
 *          changing it (cpu0-5 share the rail). The registers are read one
 *          after the other over the PMIC wrapper; without this a change by
 *          mediatek-cpufreq between the Vproc2 and the Vsram2 read compares
 *          values from different moments. Default on; 0 = the original
 *          read-back (for comparison only).
 * tripped: set by a failed check. The CCI voltage request is pinned to the
 *          highest OPP voltage, the frequency is no longer changed and the
 *          module can no longer be unloaded. Read only. The trip logs the
 *          failing read-back, fresh read-backs and the last rail changes.
 * trip_trace: on a trip also stop the ftrace ring buffer (tracing_off()).
 * verbose: log every transition with the read-back values.
 * status:  devfreq target, clock and voltages read back from the hardware.
 */
#define LISBON_CCIPLL_CON1	0x2a4
#define LISBON_BUS_DIV		0x2e0
#define LISBON_PMIC_HWCID	0x8
#define LISBON_VPROC2_RG	0x17b2	/* RG_BUCK_VPROC2_VOSEL [6:0] */
#define LISBON_VPROC2_DA	0x17a6	/* DA_VPROC2_VOSEL [6:0] */
#define LISBON_VSRAM2_RG	0x1b42	/* RG_LDO_VSRAM_PROC2_VOSEL [6:0] */
#define LISBON_VSRAM2_DA	0x1eac	/* DA_VSRAM_PROC2_VOSEL [14:8] */
#define LISBON_VSRAM2_TRACK0	0x1ebe	/* TRACK_EN [0] */
#define LISBON_VPROC_UV(sel)	(400000 + (sel) * 6250)
#define LISBON_VSRAM_UV(sel)	(500000 + (sel) * 6250)
#define LISBON_MUX_SEL		GENMASK(10, 9)
#define LISBON_MUX_PLL		1
#define LISBON_CKDIV1		GENMASK(21, 17)
#define LISBON_CKDIV1_DIV1	8
#define LISBON_PLL_PCW		GENMASK(21, 0)
#define LISBON_PLL_POSDIV	GENMASK(26, 24)

static bool vcheck = true;
module_param(vcheck, bool, 0644);
MODULE_PARM_DESC(vcheck, "check clock and PMIC read-back around CCI changes (MT6785)");
static bool vcheck_seq = true;
module_param(vcheck_seq, bool, 0644);
MODULE_PARM_DESC(vcheck_seq, "read back only while the shared rail is not changing (MT6785)");
static bool trip_trace;
module_param(trip_trace, bool, 0644);
MODULE_PARM_DESC(trip_trace, "stop ftrace on a trip (MT6785)");
static bool tripped;
module_param(tripped, bool, 0444);
MODULE_PARM_DESC(tripped, "set by a failed check: CCI voltage pinned to the highest OPP, no more changes");
static bool verbose;
module_param(verbose, bool, 0644);
MODULE_PARM_DESC(verbose, "log each CCI transition with read-back values");
static DEFINE_MUTEX(mtk_lisbon_cci_lock);
static struct mtk_ccifreq_drv *mtk_lisbon_cci;

/* CCI clock in kHz read back from CCIPLL and the bus mux, 0 if not on the PLL at /1 */
static unsigned long mtk_lisbon_cci_hw_khz(struct mtk_ccifreq_drv *drv, u32 *con1, u32 *mux)
{
	u64 vco;

	*con1 = readl(drv->apmixed + LISBON_CCIPLL_CON1);
	*mux = readl(drv->mcusys + LISBON_BUS_DIV);
	if (FIELD_GET(LISBON_MUX_SEL, *mux) != LISBON_MUX_PLL ||
	    FIELD_GET(LISBON_CKDIV1, *mux) != LISBON_CKDIV1_DIV1)
		return 0;
	vco = (u64)FIELD_GET(LISBON_PLL_PCW, *con1) * 26000;
	return (unsigned long)((vco >> 14) >> FIELD_GET(LISBON_PLL_POSDIV, *con1));
}

/* Vproc2 RG, DA, Vsram2 RG, DA in uV and TRACK0; non-zero on a read error */
static int mtk_lisbon_cci_volts(struct mtk_ccifreq_drv *drv, int uv[4], u32 *track0)
{
	u32 s[4] = { };
	int ret;

	*track0 = 0;
	ret = regmap_read(drv->pmic, LISBON_VPROC2_RG, &s[0]);
	ret |= regmap_read(drv->pmic, LISBON_VPROC2_DA, &s[1]);
	ret |= regmap_read(drv->pmic, LISBON_VSRAM2_RG, &s[2]);
	ret |= regmap_read(drv->pmic, LISBON_VSRAM2_DA, &s[3]);
	ret |= regmap_read(drv->pmic, LISBON_VSRAM2_TRACK0, track0);
	uv[0] = LISBON_VPROC_UV(s[0] & 0x7f);
	uv[1] = LISBON_VPROC_UV(s[1] & 0x7f);
	uv[2] = LISBON_VSRAM_UV(s[2] & 0x7f);
	uv[3] = LISBON_VSRAM_UV((s[3] >> 8) & 0x7f);
	return ret;
}

/* before raising the CCI voltage request to new uV */
static int mtk_lisbon_cci_precheck(struct mtk_ccifreq_drv *drv, int new)
{
	u32 track0;
	int uv[4];

	if (!drv->lisbon || !READ_ONCE(vcheck))
		return 0;
	if (mtk_lisbon_cci_volts(drv, uv, &track0)) {
		dev_err(drv->dev, "lisbon: PMIC read failed, not raising Vproc2\n");
		return -EIO;
	}
	if (new > uv[1] && !(track0 & 1) && uv[3] < new) {
		dev_err(drv->dev, "lisbon: VSRAM_PROC2 tracking off (0x%04x), Vsram %d < %d uV, not raising\n",
			track0, uv[3], new);
		return -EPERM;
	}
	return 0;
}

static void mtk_lisbon_cci_trip(struct mtk_ccifreq_drv *drv, const char *why)
{
	int ret;

	if (!READ_ONCE(tripped)) {
		WRITE_ONCE(tripped, true);
		__module_get(THIS_MODULE);	/* keep the pinned vote until reboot */
	}
	ret = regulator_set_voltage(drv->proc_reg, drv->max_volt,
				    drv->soc_data->proc_max_volt);
	dev_err(drv->dev, "lisbon: tripped (%s): Vproc2 request pinned to %d uV (%d), CCI frequency frozen\n",
		why, drv->max_volt, ret);
}

/*
 * Regulator notifier on VPROC2, called by the core with the rail locked in the
 * task that changes the voltage, for a change by any consumer: PRE before
 * the selector write, VOLTAGE_CHANGE after the ramp delay, ABORT if the write
 * failed. rail_gen moves on every event, rail_busy is set from PRE to
 * VOLTAGE_CHANGE/ABORT.
 */
static int mtk_lisbon_cci_rail_event(struct notifier_block *nb,
				     unsigned long event, void *data)
{
	struct mtk_ccifreq_drv *drv = container_of(nb, struct mtk_ccifreq_drv, rail_nb);
	struct pre_voltage_change_data *pre = data;
	struct mtk_lisbon_cci_ev *ev;
	char type;

	switch (event) {
	case REGULATOR_EVENT_PRE_VOLTAGE_CHANGE:
		type = 'p';
		break;
	case REGULATOR_EVENT_VOLTAGE_CHANGE:
		type = 'd';
		break;
	case REGULATOR_EVENT_ABORT_VOLTAGE_CHANGE:
		type = 'a';
		break;
	default:
		return NOTIFY_DONE;
	}

	spin_lock(&drv->rail_lock);
	drv->rail_gen++;
	drv->rail_busy = type == 'p';
	ev = &drv->rail_ev[drv->rail_head++ % LISBON_HIST];
	ev->ns = local_clock();
	ev->type = type;
	ev->self = READ_ONCE(drv->rail_owner) == current;
	ev->pid = task_pid_nr(current);
	strscpy(ev->comm, current->comm);
	ev->old_uv = type == 'p' ? pre->old_uV : type == 'a' ? (unsigned long)data : 0;
	ev->new_uv = type == 'p' ? pre->min_uV : type == 'd' ? (unsigned long)data : 0;
	spin_unlock(&drv->rail_lock);

	return NOTIFY_OK;
}

/*
 * Vproc2 RG, DA, Vsram2 RG, DA and, with da2, Vproc2 DA once more. stable: no
 * rail change was in flight or started meanwhile (a change by the other
 * consumer always passes PRE before its selector write).
 */
static void mtk_lisbon_cci_sample(struct mtk_ccifreq_drv *drv,
				  struct mtk_lisbon_cci_probe *p, bool da2)
{
	u32 s[LISBON_RB_N] = { };
	int ret;

	spin_lock(&drv->rail_lock);
	p->gen0 = drv->rail_gen;
	p->busy0 = drv->rail_busy;
	spin_unlock(&drv->rail_lock);

	p->t0 = local_clock();
	ret = regmap_read(drv->pmic, LISBON_VPROC2_RG, &s[LISBON_RB_RG]);
	ret |= regmap_read(drv->pmic, LISBON_VPROC2_DA, &s[LISBON_RB_DA]);
	ret |= regmap_read(drv->pmic, LISBON_VSRAM2_RG, &s[LISBON_RB_SRG]);
	ret |= regmap_read(drv->pmic, LISBON_VSRAM2_DA, &s[LISBON_RB_SDA]);
	if (da2)
		ret |= regmap_read(drv->pmic, LISBON_VPROC2_DA, &s[LISBON_RB_DA2]);
	else
		s[LISBON_RB_DA2] = s[LISBON_RB_DA];
	p->t1 = local_clock();

	spin_lock(&drv->rail_lock);
	p->gen1 = drv->rail_gen;
	spin_unlock(&drv->rail_lock);

	p->ret = ret;
	p->stable = !p->busy0 && p->gen0 == p->gen1;
	p->uv[LISBON_RB_RG] = LISBON_VPROC_UV(s[LISBON_RB_RG] & 0x7f);
	p->uv[LISBON_RB_DA] = LISBON_VPROC_UV(s[LISBON_RB_DA] & 0x7f);
	p->uv[LISBON_RB_SRG] = LISBON_VSRAM_UV(s[LISBON_RB_SRG] & 0x7f);
	p->uv[LISBON_RB_SDA] = LISBON_VSRAM_UV((s[LISBON_RB_SDA] >> 8) & 0x7f);
	p->uv[LISBON_RB_DA2] = LISBON_VPROC_UV(s[LISBON_RB_DA2] & 0x7f);
}

static void mtk_lisbon_cci_print_probe(struct mtk_ccifreq_drv *drv, const char *what,
				       const struct mtk_lisbon_cci_probe *p, u64 ref)
{
	dev_err(drv->dev,
		"lisbon: %s %+lld..%+lld us: rg %d da %d srg %d sda %d da2 %d gen %u..%u%s%s%s\n",
		what, (s64)(p->t0 - ref) / 1000, (s64)(p->t1 - ref) / 1000,
		p->uv[LISBON_RB_RG], p->uv[LISBON_RB_DA], p->uv[LISBON_RB_SRG],
		p->uv[LISBON_RB_SDA], p->uv[LISBON_RB_DA2], p->gen0, p->gen1,
		p->busy0 ? " busy" : "", p->stable ? " stable" : " unstable",
		p->ret ? " read-error" : "");
}

/* the one trip report: failing read-back, four fresh ones, the last rail changes */
static void mtk_lisbon_cci_report(struct mtk_ccifreq_drv *drv, const char *why,
				  const struct mtk_lisbon_cci_probe *p, bool seq,
				  unsigned int retry, unsigned long khz, unsigned long hw,
				  int volt)
{
	struct mtk_lisbon_cci_ev ev[LISBON_HIST];
	struct mtk_lisbon_cci_probe q;
	unsigned int head, i;

	if (READ_ONCE(trip_trace))
		tracing_off();
	dev_err(drv->dev,
		"lisbon: TRIP: %s: %lu kHz (hw %lu), OPP %d uV, read-back %s, %u retries%s\n",
		why, khz, hw, volt, seq ? "on a stable rail" : "unsynchronised (vcheck_seq=0)",
		retry, READ_ONCE(trip_trace) ? ", ftrace stopped" : "");
	mtk_lisbon_cci_print_probe(drv, "failed read-back", p, p->t0);
	for (i = 0; i < 4; i++) {
		mtk_lisbon_cci_sample(drv, &q, true);
		mtk_lisbon_cci_print_probe(drv, "re-read", &q, p->t0);
	}

	spin_lock(&drv->rail_lock);
	head = drv->rail_head;
	memcpy(ev, drv->rail_ev, sizeof(ev));
	spin_unlock(&drv->rail_lock);
	for (i = 0; i < LISBON_HIST; i++) {
		const struct mtk_lisbon_cci_ev *e = &ev[(head + i) % LISBON_HIST];

		if (!e->type)
			continue;
		dev_err(drv->dev, "lisbon: rail VPROC2 %+lld us %c %d -> %d uV %s %d %s\n",
			(s64)(e->ns - p->t0) / 1000, e->type, e->old_uv, e->new_uv,
			e->self ? "self" : "other", e->pid, e->comm);
	}
}

/* the read-back before vcheck_seq, kept for comparison */
static const char *mtk_lisbon_cci_check_legacy(struct mtk_ccifreq_drv *drv, int volt,
					       struct mtk_lisbon_cci_probe *p)
{
	int i;

	/* DA follows the soft ramp; the regulator core already waited the ramp delay */
	for (i = 0; i < 20; i++) {
		mtk_lisbon_cci_sample(drv, p, false);
		if (p->ret || p->uv[LISBON_RB_DA] >= volt)
			break;
		udelay(10);
	}
	if (p->ret)
		return "PMIC read failed";
	if (p->uv[LISBON_RB_DA] < volt)
		return "Vproc2 DA below the OPP voltage";
	if (p->uv[LISBON_RB_SDA] < p->uv[LISBON_RB_DA])
		return "Vsram2 DA below Vproc2 DA";
	return NULL;
}

/*
 * Read back until no rail change is in flight (budget LISBON_VERIFY_US). On a
 * stable sample: RG and both Vproc2 DA reads >= the OPP voltage (the core
 * keeps the maximum of all requests), Vsram2 DA >= both Vproc2 DA reads.
 * No stable sample in time counts as a failure.
 */
static const char *mtk_lisbon_cci_check(struct mtk_ccifreq_drv *drv, int volt,
					struct mtk_lisbon_cci_probe *p, unsigned int *retry)
{
	u64 start = local_clock(), us;

	for (;;) {
		mtk_lisbon_cci_sample(drv, p, true);
		if (p->ret)
			return "PMIC read failed";
		if (p->stable && p->uv[LISBON_RB_DA] == p->uv[LISBON_RB_RG] &&
		    p->uv[LISBON_RB_DA2] == p->uv[LISBON_RB_RG])
			break;
		if (local_clock() - start > LISBON_VERIFY_US * NSEC_PER_USEC) {
			if (!p->stable)
				return "no read-back on a stable rail";
			break;
		}
		(*retry)++;
		usleep_range(20, 50);
	}

	us = div_u64(local_clock() - start, NSEC_PER_USEC);
	drv->nretry += *retry;
	drv->maxretry = max(drv->maxretry, *retry);
	drv->maxverify_us = max_t(unsigned int, drv->maxverify_us, us);

	if (p->uv[LISBON_RB_RG] < volt)
		return "Vproc2 RG below the OPP voltage";
	if (min(p->uv[LISBON_RB_DA], p->uv[LISBON_RB_DA2]) < volt)
		return "Vproc2 DA below the OPP voltage";
	if (p->uv[LISBON_RB_SDA] < max(p->uv[LISBON_RB_DA], p->uv[LISBON_RB_DA2]))
		return "Vsram2 DA below Vproc2 DA";
	return NULL;
}

/* after a transition to freq Hz at volt uV (called with reg_lock held) */
static int mtk_lisbon_cci_verify(struct mtk_ccifreq_drv *drv, unsigned long freq, int volt)
{
	bool seq = READ_ONCE(vcheck_seq) && drv->rail_nb_on;
	struct mtk_lisbon_cci_probe p = { };
	unsigned long khz = freq / 1000, hw;
	unsigned int retry = 0;
	const char *bad;
	u32 con1, mux;

	if (!drv->lisbon || !READ_ONCE(vcheck))
		return 0;

	if (seq)
		bad = mtk_lisbon_cci_check(drv, volt, &p, &retry);
	else
		bad = mtk_lisbon_cci_check_legacy(drv, volt, &p);
	hw = mtk_lisbon_cci_hw_khz(drv, &con1, &mux);
	if (!bad && (!hw || hw + khz / 200 < khz || hw > khz + khz / 200))
		bad = "CCI clock read-back differs from the target";

	drv->ntrans++;
	if (!bad && READ_ONCE(verbose))
		dev_info(drv->dev,
			 "lisbon: %lu kHz (hw %lu, con1 0x%08x, mux 0x%08x), OPP %d uV: Vproc2 rg %d da %d da2 %d, Vsram2 rg %d da %d, %u retries\n",
			 khz, hw, con1, mux, volt, p.uv[LISBON_RB_RG], p.uv[LISBON_RB_DA],
			 p.uv[LISBON_RB_DA2], p.uv[LISBON_RB_SRG], p.uv[LISBON_RB_SDA], retry);
	if (!bad)
		return 0;
	if (!READ_ONCE(tripped))
		mtk_lisbon_cci_report(drv, bad, &p, seq, retry, khz, hw, volt);
	mtk_lisbon_cci_trip(drv, bad);
	return -EIO;
}

static int mtk_lisbon_cci_status_get(char *buf, const struct kernel_param *kp)
{
	struct mtk_ccifreq_drv *drv;
	u32 con1 = 0, mux = 0, track0 = 0;
	unsigned long hw = 0, cur = 0;
	int uv[4] = { }, len = 0;

	mutex_lock(&mtk_lisbon_cci_lock);
	drv = mtk_lisbon_cci;
	if (drv) {
		mutex_lock(&drv->reg_lock);
		hw = mtk_lisbon_cci_hw_khz(drv, &con1, &mux);
		mtk_lisbon_cci_volts(drv, uv, &track0);
		cur = drv->pre_freq / 1000;
		len = scnprintf(buf, PAGE_SIZE,
				"cci target=%lu boot=%lu hw=%lu con1=0x%08x mux=0x%08x vproc2_rg=%d vproc2_da=%d vsram2_rg=%d vsram2_da=%d track_en=%u max_volt=%d transitions=%u rail_nb=%d rail_gen=%u retry=%u maxretry=%u maxverify_us=%u\n",
				cur, drv->boot_khz, hw, con1, mux, uv[0], uv[1], uv[2], uv[3],
				track0 & 1, drv->max_volt, drv->ntrans, drv->rail_nb_on,
				READ_ONCE(drv->rail_gen), drv->nretry, drv->maxretry,
				drv->maxverify_us);
		mutex_unlock(&drv->reg_lock);
	} else {
		len = scnprintf(buf, PAGE_SIZE, "cci not bound\n");
	}
	mutex_unlock(&mtk_lisbon_cci_lock);
	len += scnprintf(buf + len, PAGE_SIZE - len,
			 "global vcheck=%d vcheck_seq=%d tripped=%d trip_trace=%d verbose=%d\n",
			 READ_ONCE(vcheck), READ_ONCE(vcheck_seq), READ_ONCE(tripped),
			 READ_ONCE(trip_trace), READ_ONCE(verbose));
	return len;
}

static const struct kernel_param_ops mtk_lisbon_cci_status_ops = {
	.get = mtk_lisbon_cci_status_get,
};
module_param_cb(status, &mtk_lisbon_cci_status_ops, NULL, 0444);
MODULE_PARM_DESC(status, "MT6785 read-back of the CCI clock and Vproc2/Vsram2");

/*
 * Map the registers and check that the boot state is plausible before the
 * driver touches anything: PMIC id of the MT6359 map, proc-supply = VPROC2,
 * VSRAM_PROC2 tracking on, Vproc2 DA = RG inside the OPP voltage range,
 * Vsram2 >= Vproc2, the CCI on CCIPLL at /1 with the rate the clock framework
 * reports, inside the OPP frequency range. Any failure fails the probe.
 */
static int mtk_lisbon_cci_init(struct mtk_ccifreq_drv *drv)
{
	struct device *dev = drv->dev;
	unsigned int vsel_reg, vsel_mask;
	unsigned long khz, fw_khz, lo = 0, hi = ULONG_MAX;
	struct dev_pm_opp *opp;
	struct device_node *np;
	struct resource res;
	u32 id = 0, con1, mux, track0;
	int uv[4], vlo, i, ret;

	if (!of_machine_is_compatible("mediatek,mt6785"))
		return 0;
	drv->lisbon = true;

	/* read only; the registers belong to the clock drivers, so no region request */
	np = of_find_compatible_node(NULL, NULL, "mediatek,mt6785-apmixedsys");
	if (np && !of_address_to_resource(np, 0, &res))
		drv->apmixed = devm_ioremap(dev, res.start, resource_size(&res));
	of_node_put(np);
	np = of_find_compatible_node(NULL, NULL, "mediatek,mt6785-mcusys");
	if (np && !of_address_to_resource(np, 0, &res))
		drv->mcusys = devm_ioremap(dev, res.start, resource_size(&res));
	of_node_put(np);
	if (!drv->apmixed || !drv->mcusys)
		return dev_err_probe(dev, -ENODEV, "lisbon: apmixedsys/mcusys not mapped\n");

	if (regulator_get_hardware_vsel_register(drv->proc_reg, &vsel_reg, &vsel_mask) ||
	    vsel_reg != LISBON_VPROC2_RG)
		return dev_err_probe(dev, -ENODEV, "lisbon: proc-supply is not VPROC2\n");
	drv->pmic = regulator_get_regmap(drv->proc_reg);
	if (IS_ERR_OR_NULL(drv->pmic) || regmap_read(drv->pmic, LISBON_PMIC_HWCID, &id) ||
	    id < 0x5900 || id >= 0x5930)
		return dev_err_probe(dev, -ENODEV, "lisbon: PMIC id 0x%x not the MT6359 map\n", id);

	spin_lock_init(&drv->rail_lock);
	drv->rail_nb.notifier_call = mtk_lisbon_cci_rail_event;
	ret = devm_regulator_register_notifier(drv->proc_reg, &drv->rail_nb);
	if (ret)
		dev_warn(dev, "lisbon: no rail notifier (%d), unsynchronised read-back\n", ret);
	drv->rail_nb_on = !ret;

	opp = dev_pm_opp_find_freq_ceil(dev, &lo);
	if (IS_ERR(opp))
		return dev_err_probe(dev, PTR_ERR(opp), "lisbon: no OPP\n");
	vlo = dev_pm_opp_get_voltage(opp);
	dev_pm_opp_put(opp);
	opp = dev_pm_opp_find_freq_floor(dev, &hi);
	if (IS_ERR(opp))
		return dev_err_probe(dev, PTR_ERR(opp), "lisbon: no OPP\n");
	drv->max_volt = dev_pm_opp_get_voltage(opp);
	dev_pm_opp_put(opp);

	khz = mtk_lisbon_cci_hw_khz(drv, &con1, &mux);
	fw_khz = clk_get_rate(drv->cci_clk) / 1000;
	drv->boot_khz = khz;
	/* mediatek-cpufreq may be ramping Vproc2 right now: give DA time to reach RG */
	for (i = 0; i < 20; i++) {
		if (mtk_lisbon_cci_volts(drv, uv, &track0))
			return dev_err_probe(dev, -EIO, "lisbon: PMIC read failed\n");
		if (uv[1] == uv[0])
			break;
		udelay(10);
	}
	dev_info(dev,
		 "lisbon: boot CCI hw %lu kHz (clk %lu, con1 0x%08x, mux 0x%08x), Vproc2 rg %d da %d, Vsram2 rg %d da %d, track0 0x%04x, OPP %lu..%lu kHz %d..%d uV\n",
		 khz, fw_khz, con1, mux, uv[0], uv[1], uv[2], uv[3], track0,
		 lo / 1000, hi / 1000, vlo, drv->max_volt);
	if (!READ_ONCE(vcheck))
		return 0;
	if (!khz || khz + khz / 200 < fw_khz || khz > fw_khz + fw_khz / 200 ||
	    khz + khz / 200 < lo / 1000 || khz > hi / 1000 + hi / 200000)
		return dev_err_probe(dev, -ENODEV, "lisbon: CCI clock read-back not plausible\n");
	if (!(track0 & 1) || uv[1] != uv[0] || uv[1] < vlo ||
	    uv[1] > drv->soc_data->proc_max_volt || uv[3] < uv[1])
		return dev_err_probe(dev, -ENODEV,
				     "lisbon: Vproc2/Vsram2 read-back not plausible\n");
	return 0;
}

static int mtk_ccifreq_set_voltage(struct mtk_ccifreq_drv *drv, int new_voltage)
{
	const struct mtk_ccifreq_platform_data *soc_data = drv->soc_data;
	struct device *dev = drv->dev;
	int pre_voltage, pre_vsram, new_vsram, vsram, voltage, ret;
	int retry_max = drv->vtrack_max;

	if (!drv->sram_reg) {
		ret = regulator_set_voltage(drv->proc_reg, new_voltage,
					    drv->soc_data->proc_max_volt);
		return ret;
	}

	pre_voltage = regulator_get_voltage(drv->proc_reg);
	if (pre_voltage < 0) {
		dev_err(dev, "invalid vproc value: %d\n", pre_voltage);
		return pre_voltage;
	}

	pre_vsram = regulator_get_voltage(drv->sram_reg);
	if (pre_vsram < 0) {
		dev_err(dev, "invalid vsram value: %d\n", pre_vsram);
		return pre_vsram;
	}

	new_vsram = clamp(new_voltage + soc_data->min_volt_shift,
			  soc_data->sram_min_volt, soc_data->sram_max_volt);

	do {
		if (pre_voltage <= new_voltage) {
			vsram = clamp(pre_voltage + soc_data->max_volt_shift,
				      soc_data->sram_min_volt, new_vsram);
			ret = regulator_set_voltage(drv->sram_reg, vsram,
						    soc_data->sram_max_volt);
			if (ret)
				return ret;

			if (vsram == soc_data->sram_max_volt ||
			    new_vsram == soc_data->sram_min_volt)
				voltage = new_voltage;
			else
				voltage = vsram - soc_data->min_volt_shift;

			ret = regulator_set_voltage(drv->proc_reg, voltage,
						    soc_data->proc_max_volt);
			if (ret) {
				regulator_set_voltage(drv->sram_reg, pre_vsram,
						      soc_data->sram_max_volt);
				return ret;
			}
		} else {
			voltage = max(new_voltage,
				      pre_vsram - soc_data->max_volt_shift);
			ret = regulator_set_voltage(drv->proc_reg, voltage,
						    soc_data->proc_max_volt);
			if (ret)
				return ret;

			if (voltage == new_voltage)
				vsram = new_vsram;
			else
				vsram = max(new_vsram,
					    voltage + soc_data->min_volt_shift);

			ret = regulator_set_voltage(drv->sram_reg, vsram,
						    soc_data->sram_max_volt);
			if (ret) {
				regulator_set_voltage(drv->proc_reg, pre_voltage,
						      soc_data->proc_max_volt);
				return ret;
			}
		}

		pre_voltage = voltage;
		pre_vsram = vsram;

		if (--retry_max < 0) {
			dev_err(dev,
				"over loop count, failed to set voltage\n");
			return -EINVAL;
		}
	} while (voltage != new_voltage || vsram != new_vsram);

	return 0;
}

static int mtk_ccifreq_target(struct device *dev, unsigned long *freq,
			      u32 flags)
{
	struct mtk_ccifreq_drv *drv = dev_get_drvdata(dev);
	struct clk *cci_pll;
	struct dev_pm_opp *opp;
	unsigned long opp_rate;
	int voltage, pre_voltage, inter_voltage, target_voltage, ret;

	if (!drv)
		return -EINVAL;

	if (drv->pre_freq == *freq)
		return 0;

	mutex_lock(&drv->reg_lock);

	if (drv->lisbon && READ_ONCE(tripped)) {
		*freq = drv->pre_freq ? drv->pre_freq : clk_get_rate(drv->cci_clk);
		mutex_unlock(&drv->reg_lock);
		return 0;
	}
	WRITE_ONCE(drv->rail_owner, current);

	inter_voltage = drv->inter_voltage;
	cci_pll = clk_get_parent(drv->cci_clk);

	opp_rate = *freq;
	opp = devfreq_recommended_opp(dev, &opp_rate, 1);
	if (IS_ERR(opp)) {
		dev_err(dev, "failed to find opp for freq: %ld\n", opp_rate);
		ret = PTR_ERR(opp);
		goto out_unlock;
	}

	voltage = dev_pm_opp_get_voltage(opp);
	dev_pm_opp_put(opp);

	pre_voltage = regulator_get_voltage(drv->proc_reg);
	if (pre_voltage < 0) {
		dev_err(dev, "invalid vproc value: %d\n", pre_voltage);
		ret = pre_voltage;
		goto out_unlock;
	}

	/* scale up: set voltage first then freq. */
	target_voltage = max(inter_voltage, voltage);
	if (pre_voltage <= target_voltage) {
		ret = mtk_lisbon_cci_precheck(drv, target_voltage);
		if (ret)
			goto out_unlock;
		ret = mtk_ccifreq_set_voltage(drv, target_voltage);
		if (ret) {
			dev_err_ratelimited(dev, "failed to scale up voltage\n");
			goto out_restore_voltage;
		}
	}

	/* switch the cci clock to intermediate clock source. */
	ret = clk_set_parent(drv->cci_clk, drv->inter_clk);
	if (ret) {
		dev_err(dev, "failed to re-parent cci clock\n");
		goto out_restore_voltage;
	}

	/* set the original clock to target rate. */
	ret = clk_set_rate(cci_pll, *freq);
	if (ret) {
		dev_err(dev, "failed to set cci pll rate: %d\n", ret);
		clk_set_parent(drv->cci_clk, cci_pll);
		goto out_restore_voltage;
	}

	/* switch the cci clock back to the original clock source. */
	ret = clk_set_parent(drv->cci_clk, cci_pll);
	if (ret) {
		dev_err(dev, "failed to re-parent cci clock\n");
		mtk_ccifreq_set_voltage(drv, inter_voltage);
		goto out_unlock;
	}

	/*
	 * If the new voltage is lower than the intermediate voltage or the
	 * original voltage, scale down to the new voltage.
	 */
	if (voltage < inter_voltage || voltage < pre_voltage) {
		ret = mtk_ccifreq_set_voltage(drv, voltage);
		if (ret) {
			dev_err(dev, "failed to scale down voltage\n");
			goto out_unlock;
		}
	}

	drv->pre_freq = *freq;
	mtk_lisbon_cci_verify(drv, *freq, voltage);
	WRITE_ONCE(drv->rail_owner, NULL);
	mutex_unlock(&drv->reg_lock);

	return 0;

out_restore_voltage:
	mtk_ccifreq_set_voltage(drv, pre_voltage);

out_unlock:
	WRITE_ONCE(drv->rail_owner, NULL);
	mutex_unlock(&drv->reg_lock);
	return ret;
}

static int mtk_ccifreq_opp_notifier(struct notifier_block *nb,
				    unsigned long event, void *data)
{
	struct dev_pm_opp *opp = data;
	struct mtk_ccifreq_drv *drv;
	unsigned long freq, volt;

	drv = container_of(nb, struct mtk_ccifreq_drv, opp_nb);

	if (event == OPP_EVENT_ADJUST_VOLTAGE) {
		mutex_lock(&drv->reg_lock);
		freq = dev_pm_opp_get_freq(opp);

		/* current opp item is changed */
		if (freq == drv->pre_freq) {
			volt = dev_pm_opp_get_voltage(opp);
			mtk_ccifreq_set_voltage(drv, volt);
		}
		mutex_unlock(&drv->reg_lock);
	}

	return 0;
}

static struct devfreq_dev_profile mtk_ccifreq_profile = {
	.target = mtk_ccifreq_target,
};

/*
 * MT6785: before the voltage request goes away on unload, stop the governor
 * and park the CCI at the lowest OPP. Its voltage (600 mV) is the lowest
 * voltage any cpu0-5 OPP requests on VPROC2, so the CCI stays covered by
 * mediatek-cpufreq alone (which no longer clamps to the boot voltage once
 * it has seen this driver bound). If parking fails, keep a request for the
 * highest OPP voltage through an extra consumer that is never released.
 */
static void mtk_lisbon_cci_park(struct mtk_ccifreq_drv *drv)
{
	struct device *dev = drv->dev;
	struct regulator *keep;
	unsigned long rate = 0;
	struct dev_pm_opp *opp;
	int ret = -EINVAL;

	devm_devfreq_remove_device(dev, drv->devfreq);

	opp = dev_pm_opp_find_freq_ceil(dev, &rate);
	if (!IS_ERR(opp)) {
		dev_pm_opp_put(opp);
		if (!READ_ONCE(tripped))
			ret = mtk_ccifreq_target(dev, &rate, 0);
	}
	if (!ret && !READ_ONCE(tripped)) {
		dev_info(dev, "lisbon: parked at %lu kHz\n", rate / 1000);
		return;
	}

	keep = regulator_get(dev, "proc");
	if (!IS_ERR(keep))
		ret = regulator_set_voltage(keep, drv->max_volt,
					    drv->soc_data->proc_max_volt);
	dev_crit(dev, "lisbon: parking failed, Vproc2 request %d uV kept until reboot (%d)\n",
		 drv->max_volt, IS_ERR(keep) ? (int)PTR_ERR(keep) : ret);
}

static int mtk_ccifreq_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mtk_ccifreq_drv *drv;
	struct devfreq_passive_data *passive_data;
	struct dev_pm_opp *opp;
	unsigned long rate, opp_volt;
	int ret;

	drv = devm_kzalloc(dev, sizeof(*drv), GFP_KERNEL);
	if (!drv)
		return -ENOMEM;

	drv->dev = dev;
	drv->soc_data = (const struct mtk_ccifreq_platform_data *)
				of_device_get_match_data(&pdev->dev);
	mutex_init(&drv->reg_lock);
	platform_set_drvdata(pdev, drv);

	drv->cci_clk = devm_clk_get(dev, "cci");
	if (IS_ERR(drv->cci_clk)) {
		ret = PTR_ERR(drv->cci_clk);
		return dev_err_probe(dev, ret, "failed to get cci clk\n");
	}

	drv->inter_clk = devm_clk_get(dev, "intermediate");
	if (IS_ERR(drv->inter_clk)) {
		ret = PTR_ERR(drv->inter_clk);
		return dev_err_probe(dev, ret,
				     "failed to get intermediate clk\n");
	}

	drv->proc_reg = devm_regulator_get_optional(dev, "proc");
	if (IS_ERR(drv->proc_reg)) {
		ret = PTR_ERR(drv->proc_reg);
		return dev_err_probe(dev, ret,
				     "failed to get proc regulator\n");
	}

	ret = regulator_enable(drv->proc_reg);
	if (ret) {
		dev_err(dev, "failed to enable proc regulator\n");
		return ret;
	}

	drv->sram_reg = devm_regulator_get_optional(dev, "sram");
	if (IS_ERR(drv->sram_reg)) {
		ret = PTR_ERR(drv->sram_reg);
		if (ret == -EPROBE_DEFER)
			goto out_free_resources;

		drv->sram_reg = NULL;
	} else {
		ret = regulator_enable(drv->sram_reg);
		if (ret) {
			dev_err(dev, "failed to enable sram regulator\n");
			goto out_free_resources;
		}
	}

	/*
	 * We assume min voltage is 0 and tracking target voltage using
	 * min_volt_shift for each iteration.
	 * The retry_max is 3 times of expected iteration count.
	 */
	drv->vtrack_max = 3 * DIV_ROUND_UP(max(drv->soc_data->sram_max_volt,
					       drv->soc_data->proc_max_volt),
					   drv->soc_data->min_volt_shift);

	ret = clk_prepare_enable(drv->cci_clk);
	if (ret)
		goto out_free_resources;

	ret = dev_pm_opp_of_add_table(dev);
	if (ret) {
		dev_err(dev, "failed to add opp table: %d\n", ret);
		goto out_disable_cci_clk;
	}

	rate = clk_get_rate(drv->inter_clk);
	opp = dev_pm_opp_find_freq_ceil(dev, &rate);
	if (IS_ERR(opp)) {
		ret = PTR_ERR(opp);
		dev_err(dev, "failed to get intermediate opp: %d\n", ret);
		goto out_remove_opp_table;
	}
	drv->inter_voltage = dev_pm_opp_get_voltage(opp);
	dev_pm_opp_put(opp);

	ret = mtk_lisbon_cci_init(drv);
	if (ret)
		goto out_remove_opp_table;

	rate = U32_MAX;
	opp = dev_pm_opp_find_freq_floor(drv->dev, &rate);
	if (IS_ERR(opp)) {
		dev_err(dev, "failed to get opp\n");
		ret = PTR_ERR(opp);
		goto out_remove_opp_table;
	}

	opp_volt = dev_pm_opp_get_voltage(opp);
	dev_pm_opp_put(opp);
	ret = mtk_lisbon_cci_precheck(drv, opp_volt);
	if (ret)
		goto out_remove_opp_table;
	WRITE_ONCE(drv->rail_owner, current);
	ret = mtk_ccifreq_set_voltage(drv, opp_volt);
	WRITE_ONCE(drv->rail_owner, NULL);
	if (ret) {
		dev_err(dev, "failed to scale to highest voltage %lu in proc_reg\n",
			opp_volt);
		goto out_remove_opp_table;
	}

	passive_data = devm_kzalloc(dev, sizeof(*passive_data), GFP_KERNEL);
	if (!passive_data) {
		ret = -ENOMEM;
		goto out_remove_opp_table;
	}

	/*
	 * Register the OPP notifier before the governor starts: once the
	 * devfreq device exists the CCI clock may have been changed, and a
	 * later probe failure would drop the voltage request behind it.
	 */
	drv->opp_nb.notifier_call = mtk_ccifreq_opp_notifier;
	ret = dev_pm_opp_register_notifier(dev, &drv->opp_nb);
	if (ret) {
		dev_err(dev, "failed to register opp notifier: %d\n", ret);
		goto out_remove_opp_table;
	}

	passive_data->parent_type = CPUFREQ_PARENT_DEV;
	drv->devfreq = devm_devfreq_add_device(dev, &mtk_ccifreq_profile,
					       DEVFREQ_GOV_PASSIVE,
					       passive_data);
	if (IS_ERR(drv->devfreq)) {
		ret = -EPROBE_DEFER;
		dev_err(dev, "failed to add devfreq device: %ld\n",
			PTR_ERR(drv->devfreq));
		dev_pm_opp_unregister_notifier(dev, &drv->opp_nb);
		goto out_remove_opp_table;
	}

	if (drv->lisbon) {
		mutex_lock(&mtk_lisbon_cci_lock);
		mtk_lisbon_cci = drv;
		mutex_unlock(&mtk_lisbon_cci_lock);
	}
	return 0;

out_remove_opp_table:
	dev_pm_opp_of_remove_table(dev);

out_disable_cci_clk:
	clk_disable_unprepare(drv->cci_clk);

out_free_resources:
	if (regulator_is_enabled(drv->proc_reg))
		regulator_disable(drv->proc_reg);
	if (!IS_ERR_OR_NULL(drv->sram_reg) &&
	    regulator_is_enabled(drv->sram_reg))
		regulator_disable(drv->sram_reg);

	return ret;
}

static void mtk_ccifreq_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mtk_ccifreq_drv *drv;

	drv = platform_get_drvdata(pdev);

	if (drv->lisbon) {
		mutex_lock(&mtk_lisbon_cci_lock);
		mtk_lisbon_cci = NULL;
		mutex_unlock(&mtk_lisbon_cci_lock);
		mtk_lisbon_cci_park(drv);
	}

	dev_pm_opp_unregister_notifier(dev, &drv->opp_nb);
	dev_pm_opp_of_remove_table(dev);
	clk_disable_unprepare(drv->cci_clk);
	regulator_disable(drv->proc_reg);
	if (drv->sram_reg)
		regulator_disable(drv->sram_reg);
}

static const struct mtk_ccifreq_platform_data mt8183_platform_data = {
	.min_volt_shift = 100000,
	.max_volt_shift = 200000,
	.proc_max_volt = 1150000,
};

static const struct mtk_ccifreq_platform_data mt8186_platform_data = {
	.min_volt_shift = 100000,
	.max_volt_shift = 250000,
	.proc_max_volt = 1118750,
	.sram_min_volt = 850000,
	.sram_max_volt = 1118750,
};

static const struct of_device_id mtk_ccifreq_machines[] = {
	{ .compatible = "mediatek,mt8183-cci", .data = &mt8183_platform_data },
	{ .compatible = "mediatek,mt8186-cci", .data = &mt8186_platform_data },
	{ },
};
MODULE_DEVICE_TABLE(of, mtk_ccifreq_machines);

static struct platform_driver mtk_ccifreq_platdrv = {
	.probe	= mtk_ccifreq_probe,
	.remove = mtk_ccifreq_remove,
	.driver = {
		.name = "mtk-ccifreq",
		.of_match_table = mtk_ccifreq_machines,
		/* unbinding only through remove() on unload (MT6785 parks the CCI there) */
		.suppress_bind_attrs = true,
	},
};
module_platform_driver(mtk_ccifreq_platdrv);

MODULE_DESCRIPTION("MediaTek CCI devfreq driver");
MODULE_AUTHOR("Jia-Wei Chang <jia-wei.chang@mediatek.com>");
MODULE_LICENSE("GPL v2");
