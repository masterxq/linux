// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2015 Linaro Ltd.
 * Author: Pi-Cheng Chen <pi-cheng.chen@linaro.org>
 */

#include <linux/clk.h>
#include <linux/cpu.h>
#include <linux/cpufreq.h>
#include <linux/cpumask.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/math64.h>
#include <linux/minmax.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_platform.h>
#include <linux/platform_device.h>
#include <linux/pm_opp.h>
#include <linux/regmap.h>
#include <linux/regulator/consumer.h>
#include <linux/sched.h>
#include <linux/sched/clock.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/trace_printk.h>
#include <linux/workqueue.h>

struct mtk_cpufreq_platform_data {
	int min_volt_shift;
	int max_volt_shift;
	int proc_max_volt;
	int sram_min_volt;
	int sram_max_volt;
	bool ccifreq_supported;
	/* schedutil rate limit for the policies, 0 = cpufreq core default */
	unsigned int transition_delay_us;
};

/*
 * MT6785 + MT6359: the PMIC registers behind one CPU supply. The vendor kernel
 * puts cpu0-5 and the CCI on VPROC2/VSRAM_PROC2 and cpu6-7 on
 * VPROC1/VSRAM_PROC1 and leaves VSRAM to the PMIC hardware tracking. Addresses
 * from the MT6359 register map (chip id 0x59x0 below MT6359P 0x5930, as used by
 * the mainline regulator driver for this chip):
 *   VOSEL fields are 7 bit; Vproc = 400 mV + sel * 6.25 mV,
 *   Vsram = 500 mV + sel * 6.25 mV (mt6359-regulator.c ranges).
 *   DA_* = value the regulator actually outputs, RG_* = programmed value.
 */
struct mtk_lisbon_rail {
	const char *vproc;
	const char *vsram;
	u32 vproc_rg;	/* RG_BUCK_VPROCx_VOSEL [6:0], = vsel_reg of the regulator */
	u32 vproc_da;	/* DA_VPROCx_VOSEL [6:0] */
	u32 vsram_rg;	/* RG_LDO_VSRAM_PROCx_VOSEL [6:0] */
	u32 vsram_da;	/* DA_VSRAM_PROCx_VOSEL [14:8] */
	u32 track0;	/* TRACK_EN [0], TRACK_MODE [1]; TRACK1 = +2 (DELTA [3:0],
			 * OFFSET [14:8]), TRACK2 = +4 (LB [6:0], HB [14:8]) */
};

static const struct mtk_lisbon_rail mtk_lisbon_rails[] = {
	{ "VPROC1", "VSRAM_PROC1", 0x172e, 0x1726, 0x1b40, 0x1e8e, 0x1ea0 },
	{ "VPROC2", "VSRAM_PROC2", 0x17b2, 0x17a6, 0x1b42, 0x1eac, 0x1ebe },
};

#define MTK_LISBON_HWCID	0x8
#define MTK_LISBON_VPROC_UV(sel)	(400000 + (sel) * 6250)
#define MTK_LISBON_VSRAM_UV(sel)	(500000 + (sel) * 6250)

/*
 * Rail history: the last voltage changes of the rail (by any consumer, e.g.
 * the CCI devfreq on VPROC2), recorded from the regulator notifier, which
 * runs under the rail lock in the task that changes the voltage.
 */
#define MTK_LISBON_HIST		8
/* read-back budget after a voltage change */
#define MTK_LISBON_VERIFY_US	20000

struct mtk_lisbon_rail_ev {
	u64 ns;
	int old_uv;
	int new_uv;
	pid_t pid;
	char type;	/* p = before the change, d = done (ramp delay over), a = aborted */
	bool self;	/* changed by this driver (mtk_cpufreq_set_voltage) */
	char comm[TASK_COMM_LEN];
};

/* one read-back: Vproc RG, DA, Vsram RG, DA, Vproc DA again (after Vsram) */
enum { MTK_RB_RG, MTK_RB_DA, MTK_RB_SRG, MTK_RB_SDA, MTK_RB_DA2, MTK_RB_N };

struct mtk_lisbon_probe {
	u64 t0, t1;		/* local_clock() around the register reads */
	unsigned int gen0, gen1;	/* rail_gen before and after */
	bool busy0;		/* a rail change was in flight at the start */
	bool stable;
	int ret;
	int uv[MTK_RB_N];
};

/*
 * The struct mtk_cpu_dvfs_info holds necessary information for doing CPU DVFS
 * on each CPU power/clock domain of Mediatek SoCs. Each CPU cluster in
 * Mediatek SoCs has two voltage inputs, Vproc and Vsram. In some cases the two
 * voltage inputs need to be controlled under a hardware limitation:
 * 100mV < Vsram - Vproc < 200mV
 *
 * When scaling the clock frequency of a CPU clock domain, the clock source
 * needs to be switched to another stable PLL clock temporarily until
 * the original PLL becomes stable at target frequency.
 */
struct mtk_cpu_dvfs_info {
	struct cpumask cpus;
	struct device *cpu_dev;
	struct device *cci_dev;
	struct regulator *proc_reg;
	struct regulator *sram_reg;
	struct clk *cpu_clk;
	struct clk *inter_clk;
	struct list_head list_head;
	int intermediate_voltage;
	bool need_voltage_tracking;
	int vproc_on_boot;
	int pre_vproc;
	/* Avoid race condition for regulators between notify and policy */
	struct mutex reg_lock;
	struct notifier_block opp_nb;
	unsigned int opp_cpu;
	unsigned long current_freq;
	const struct mtk_cpufreq_platform_data *soc_data;
	int vtrack_max;
	bool ccifreq_bound;

	/* MT6785 voltage checks, see mtk_lisbon_init() */
	bool lisbon;
	unsigned int boot_khz;
	struct freq_qos_request cap_req;
	struct regmap *pmic;
	const struct mtk_lisbon_rail *rail;
	u32 pll_con1;
	u32 mux_reg;
	struct notifier_block rail_nb;
	bool rail_nb_on;
	struct task_struct *rail_owner;	/* in mtk_cpufreq_set_voltage() */
	spinlock_t rail_lock;		/* rail_gen, rail_busy, rail_ev, rail_head */
	unsigned int rail_gen;		/* moves on every rail event */
	bool rail_busy;			/* PRE seen, VOLTAGE_CHANGE/ABORT not yet */
	unsigned int rail_head;
	struct mtk_lisbon_rail_ev rail_ev[MTK_LISBON_HIST];
	unsigned int nverify, nretry, maxretry, maxverify_us;
};

static struct platform_device *cpufreq_pdev;

/*
 * Debug switches for boards where CPU DVFS is not yet validated with this
 * driver (both default off, i.e. normal behaviour):
 * hold:    do not probe at boot; clear it and probe through
 *          /sys/bus/platform/drivers_probe after boot.
 * dry_run: never change a clock or a voltage; log the change that would be
 *          made and report success (scaling_cur_freq then shows the request,
 *          cpuinfo_cur_freq the real clock). Clear it to switch for real.
 */
static bool hold;
module_param(hold, bool, 0644);
MODULE_PARM_DESC(hold, "skip probing until cleared, then probe via /sys/bus/platform/drivers_probe");

static bool dry_run;
module_param(dry_run, bool, 0644);
MODULE_PARM_DESC(dry_run, "log frequency/voltage changes instead of making them");

static LIST_HEAD(dvfs_info_list);

static struct mtk_cpu_dvfs_info *mtk_cpu_dvfs_info_lookup(int cpu)
{
	struct mtk_cpu_dvfs_info *info;

	list_for_each_entry(info, &dvfs_info_list, list_head) {
		if (cpumask_test_cpu(cpu, &info->cpus))
			return info;
	}

	return NULL;
}

/*
 * MT6785 debug switches, only active on "mediatek,mt6785":
 * cap0/cap6: upper limit in kHz for the policy of cpu0 (cpu0-5) / cpu6
 *         (cpu6-7), applied as a frequency QoS request (scaling_max_freq
 *         follows). 0 (default) = the OPP the boot loader left the cluster
 *         at, so binding the driver never speeds a cluster up. Writable at
 *         run time; a target above the cap is refused in addition.
 * vcheck: before Vproc goes above both its current and its boot value,
 *         require readable PMIC registers and either VSRAM hardware
 *         tracking or a VSRAM already at the target; after every voltage
 *         change compare DA (actual) with RG (programmed) Vproc and require
 *         DA Vsram >= DA Vproc. A failed check restores the old voltage,
 *         sets "tripped" and fails the transition (frequency unchanged).
 * vcheck_seq: take the read-back only while no consumer of the rail is
 *         changing it (the rail is shared: VPROC2 also feeds the CCI). The
 *         four registers are read one after the other over the PMIC wrapper;
 *         without this a change by the other consumer between the Vproc and
 *         the Vsram read compares values from different moments. Default on;
 *         0 = the original read-back (for comparison only).
 * tripped: set by a failed check; while set Vproc is never raised above the
 *         boot voltage and both policies are capped at their boot OPP. The
 *         first trip logs the read-back, fresh samples and the rail history
 *         once. Write 0 to clear (after reading the log).
 * trip_trace: on a trip also stop the ftrace ring buffer (tracing_off()), so
 *         that events recorded before the trip are kept.
 * verbose: log every real transition with the read-back values.
 * status (read only): caps, clock and PLL read back from the hardware
 *         registers, Vproc/Vsram RG and DA, tracking registers, read-back
 *         statistics.
 */
static unsigned int cap0, cap6;
static bool vcheck = true;
module_param(vcheck, bool, 0644);
MODULE_PARM_DESC(vcheck, "check PMIC read-back around Vproc changes (MT6785)");
static bool vcheck_seq = true;
module_param(vcheck_seq, bool, 0644);
MODULE_PARM_DESC(vcheck_seq, "read back only while the shared rail is not changing (MT6785)");
static bool trip_trace;
module_param(trip_trace, bool, 0644);
MODULE_PARM_DESC(trip_trace, "stop ftrace on a trip (MT6785)");
static bool verbose;
module_param(verbose, bool, 0644);
MODULE_PARM_DESC(verbose, "log each transition with read-back values");
static DEFINE_MUTEX(mtk_lisbon_lock);
static void __iomem *mtk_lisbon_apmixed;	/* 0x1000c000 */
static void __iomem *mtk_lisbon_mcusys;		/* 0x0c53a000 */

static bool tripped;
static atomic_t mtk_lisbon_reported = ATOMIC_INIT(0);
static void mtk_lisbon_cap_work_fn(struct work_struct *work);
static DECLARE_WORK(mtk_lisbon_cap_work, mtk_lisbon_cap_work_fn);

static int mtk_tripped_set(const char *val, const struct kernel_param *kp)
{
	int ret = param_set_bool(val, kp);

	if (ret)
		return ret;
	if (!READ_ONCE(tripped))
		atomic_set(&mtk_lisbon_reported, 0);	/* next trip reports again */
	schedule_work(&mtk_lisbon_cap_work);
	return 0;
}

static const struct kernel_param_ops mtk_tripped_ops = {
	.set = mtk_tripped_set,
	.get = param_get_bool,
};
module_param_cb(tripped, &mtk_tripped_ops, &tripped, 0644);
MODULE_PARM_DESC(tripped, "set by a failed Vproc/Vsram check; blocks raising Vproc above boot");

static unsigned int *mtk_cap_param(struct mtk_cpu_dvfs_info *info)
{
	if (!info->lisbon)
		return NULL;
	if (cpumask_test_cpu(0, &info->cpus))
		return &cap0;
	if (cpumask_test_cpu(6, &info->cpus))
		return &cap6;
	return NULL;
}

static unsigned int mtk_cap_khz(struct mtk_cpu_dvfs_info *info)
{
	unsigned int *cap = mtk_cap_param(info);
	unsigned int v;

	if (!cap)
		return FREQ_QOS_MAX_DEFAULT_VALUE;
	v = READ_ONCE(*cap);
	if (!v || (READ_ONCE(tripped) && info->boot_khz && v > info->boot_khz))
		v = info->boot_khz;
	return v;
}

/* re-apply the caps (tripped changed): the boot OPP needs no more than the boot voltage */
static void mtk_lisbon_cap_work_fn(struct work_struct *work)
{
	struct mtk_cpu_dvfs_info *info;

	mutex_lock(&mtk_lisbon_lock);
	list_for_each_entry(info, &dvfs_info_list, list_head) {
		if (!mtk_cap_param(info) || !freq_qos_request_active(&info->cap_req))
			continue;
		if (freq_qos_update_request(&info->cap_req, mtk_cap_khz(info)) > 0)
			dev_info(info->cpu_dev, "cpu%d: cap %u kHz%s\n", info->opp_cpu,
				 mtk_cap_khz(info), READ_ONCE(tripped) ? " (tripped)" : "");
	}
	mutex_unlock(&mtk_lisbon_lock);
}

static int mtk_cap_set(const char *val, const struct kernel_param *kp)
{
	struct mtk_cpu_dvfs_info *info;
	unsigned int v;
	int ret;

	ret = kstrtouint(val, 0, &v);
	if (ret)
		return ret;

	mutex_lock(&mtk_lisbon_lock);
	*(unsigned int *)kp->arg = v;
	list_for_each_entry(info, &dvfs_info_list, list_head) {
		if (mtk_cap_param(info) != kp->arg ||
		    !freq_qos_request_active(&info->cap_req))
			continue;
		ret = freq_qos_update_request(&info->cap_req, mtk_cap_khz(info));
		if (ret >= 0) {
			ret = 0;
			dev_info(info->cpu_dev, "cpu%d: cap %u kHz\n",
				 info->opp_cpu, mtk_cap_khz(info));
		}
	}
	mutex_unlock(&mtk_lisbon_lock);

	return ret;
}

static const struct kernel_param_ops mtk_cap_ops = {
	.set = mtk_cap_set,
	.get = param_get_uint,
};
module_param_cb(cap0, &mtk_cap_ops, &cap0, 0644);
MODULE_PARM_DESC(cap0, "max kHz for cpu0-5 (MT6785), 0 = boot OPP");
module_param_cb(cap6, &mtk_cap_ops, &cap6, 0644);
MODULE_PARM_DESC(cap6, "max kHz for cpu6-7 (MT6785), 0 = boot OPP");

static int mtk_lisbon_vsel(struct mtk_cpu_dvfs_info *info, u32 reg, int shift,
			   u32 *sel)
{
	int ret = regmap_read(info->pmic, reg, sel);

	*sel = (*sel >> shift) & 0x7f;
	return ret;
}

/* Vproc (RG, DA) and Vsram (RG, DA) in uV; non-zero on a read error */
static int mtk_lisbon_volts(struct mtk_cpu_dvfs_info *info, int uv[4])
{
	const struct mtk_lisbon_rail *r = info->rail;
	u32 s[4];
	int ret;

	ret = mtk_lisbon_vsel(info, r->vproc_rg, 0, &s[0]);
	ret |= mtk_lisbon_vsel(info, r->vproc_da, 0, &s[1]);
	ret |= mtk_lisbon_vsel(info, r->vsram_rg, 0, &s[2]);
	ret |= mtk_lisbon_vsel(info, r->vsram_da, 8, &s[3]);
	uv[0] = MTK_LISBON_VPROC_UV(s[0]);
	uv[1] = MTK_LISBON_VPROC_UV(s[1]);
	uv[2] = MTK_LISBON_VSRAM_UV(s[2]);
	uv[3] = MTK_LISBON_VSRAM_UV(s[3]);
	return ret;
}

/* Called before Vproc changes from old to new uV. */
static int mtk_lisbon_precheck(struct mtk_cpu_dvfs_info *info, int old, int new)
{
	u32 track0 = 0;
	int uv[4];

	if (!info->lisbon || new <= old || new <= info->vproc_on_boot)
		return 0;
	if (READ_ONCE(tripped)) {
		dev_err_ratelimited(info->cpu_dev,
				    "cpu%d: tripped, not raising Vproc to %d uV (boot %d)\n",
				    info->opp_cpu, new, info->vproc_on_boot);
		return -EPERM;
	}
	if (!READ_ONCE(vcheck))
		return 0;
	if (!info->pmic) {
		dev_err_ratelimited(info->cpu_dev,
				    "cpu%d: no PMIC read-back, not raising Vproc above boot\n",
				    info->opp_cpu);
		return -EPERM;
	}
	if (mtk_lisbon_volts(info, uv) ||
	    regmap_read(info->pmic, info->rail->track0, &track0)) {
		dev_err_ratelimited(info->cpu_dev, "cpu%d: PMIC read failed\n",
				    info->opp_cpu);
		return -EIO;
	}
	if (!(track0 & 1) && uv[3] < new) {
		dev_err_ratelimited(info->cpu_dev,
				    "cpu%d: %s tracking off (0x%04x) and Vsram %d < %d uV, not raising\n",
				    info->opp_cpu, info->rail->vsram, track0, uv[3], new);
		return -EPERM;
	}
	return 0;
}

/* PLL output in kHz from CON1: PCW [21:0] (8 integer bits), POSDIV [26:24] */
static unsigned long mtk_lisbon_pll_khz(u32 con1)
{
	u64 vco = (u64)(con1 & GENMASK(21, 0)) * 26000;

	return (unsigned long)((vco >> 14) >> ((con1 >> 24) & 7));
}

/* mcusys mux register: SEL [10:9] (0 26M, 1 PLL, 2/3 intermediate), CKDIV1 [21:17] */
static unsigned long mtk_lisbon_out_khz(unsigned long pll, u32 mux, unsigned long inter)
{
	static const u8 num[32] = { [8] = 4, [9] = 3, [10] = 2, [11] = 1, [16] = 5,
				    [17] = 4, [18] = 3, [19] = 2, [20] = 1, [24] = 6,
				    [25] = 5, [26] = 4, [27] = 3, [28] = 2, [29] = 1 };
	static const u8 den[32] = { [8] = 4, [9] = 4, [10] = 4, [11] = 4, [16] = 5,
				    [17] = 5, [18] = 5, [19] = 5, [20] = 5, [24] = 6,
				    [25] = 6, [26] = 6, [27] = 6, [28] = 6, [29] = 6 };
	unsigned int d = (mux >> 17) & 0x1f;
	unsigned long f;

	switch ((mux >> 9) & 3) {
	case 0:
		f = 26000;
		break;
	case 1:
		f = pll;
		break;
	default:
		f = inter;
		break;
	}
	return den[d] ? f * num[d] / den[d] : 0;
}

/*
 * Regulator notifier on the cluster supply. The core calls it with the rail
 * locked, in the task that changes the voltage, for a change by any consumer
 * (on VPROC2 also the CCI devfreq): PRE before the selector is written,
 * VOLTAGE_CHANGE after the ramp delay, ABORT if the write failed. rail_gen
 * moves on every event, rail_busy is set from PRE to VOLTAGE_CHANGE/ABORT.
 */
static int mtk_lisbon_rail_event(struct notifier_block *nb,
				 unsigned long event, void *data)
{
	struct mtk_cpu_dvfs_info *info = container_of(nb, struct mtk_cpu_dvfs_info, rail_nb);
	struct pre_voltage_change_data *pre = data;
	struct mtk_lisbon_rail_ev *ev;
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

	spin_lock(&info->rail_lock);
	info->rail_gen++;
	info->rail_busy = type == 'p';
	ev = &info->rail_ev[info->rail_head++ % MTK_LISBON_HIST];
	ev->ns = local_clock();
	ev->type = type;
	ev->self = READ_ONCE(info->rail_owner) == current;
	ev->pid = task_pid_nr(current);
	strscpy(ev->comm, current->comm);
	ev->old_uv = type == 'p' ? pre->old_uV : type == 'a' ? (unsigned long)data : 0;
	ev->new_uv = type == 'p' ? pre->min_uV : type == 'd' ? (unsigned long)data : 0;
	spin_unlock(&info->rail_lock);

	return NOTIFY_OK;
}

/*
 * One read-back: Vproc RG, DA, Vsram RG, DA and, with da2, Vproc DA once more
 * after Vsram. stable: no rail change was in flight or started meanwhile, so
 * all values describe the same rail state (a change by the other consumer
 * always passes PRE before its selector write).
 */
static void mtk_lisbon_sample(struct mtk_cpu_dvfs_info *info,
			      struct mtk_lisbon_probe *p, bool da2)
{
	const struct mtk_lisbon_rail *r = info->rail;
	u32 s[MTK_RB_N] = { };
	int ret;

	spin_lock(&info->rail_lock);
	p->gen0 = info->rail_gen;
	p->busy0 = info->rail_busy;
	spin_unlock(&info->rail_lock);

	p->t0 = local_clock();
	ret = mtk_lisbon_vsel(info, r->vproc_rg, 0, &s[MTK_RB_RG]);
	ret |= mtk_lisbon_vsel(info, r->vproc_da, 0, &s[MTK_RB_DA]);
	ret |= mtk_lisbon_vsel(info, r->vsram_rg, 0, &s[MTK_RB_SRG]);
	ret |= mtk_lisbon_vsel(info, r->vsram_da, 8, &s[MTK_RB_SDA]);
	if (da2)
		ret |= mtk_lisbon_vsel(info, r->vproc_da, 0, &s[MTK_RB_DA2]);
	else
		s[MTK_RB_DA2] = s[MTK_RB_DA];
	p->t1 = local_clock();

	spin_lock(&info->rail_lock);
	p->gen1 = info->rail_gen;
	spin_unlock(&info->rail_lock);

	p->ret = ret;
	p->stable = !p->busy0 && p->gen0 == p->gen1;
	p->uv[MTK_RB_RG] = MTK_LISBON_VPROC_UV(s[MTK_RB_RG]);
	p->uv[MTK_RB_DA] = MTK_LISBON_VPROC_UV(s[MTK_RB_DA]);
	p->uv[MTK_RB_SRG] = MTK_LISBON_VSRAM_UV(s[MTK_RB_SRG]);
	p->uv[MTK_RB_SDA] = MTK_LISBON_VSRAM_UV(s[MTK_RB_SDA]);
	p->uv[MTK_RB_DA2] = MTK_LISBON_VPROC_UV(s[MTK_RB_DA2]);
}

static unsigned long mtk_lisbon_hw_khz(struct mtk_cpu_dvfs_info *info)
{
	if (!mtk_lisbon_apmixed || !mtk_lisbon_mcusys)
		return 0;
	return mtk_lisbon_out_khz(mtk_lisbon_pll_khz(readl(mtk_lisbon_apmixed + info->pll_con1)),
				  readl(mtk_lisbon_mcusys + info->mux_reg),
				  clk_get_rate(info->inter_clk) / 1000);
}

static void mtk_lisbon_print_probe(struct mtk_cpu_dvfs_info *info, const char *what,
				   const struct mtk_lisbon_probe *p, u64 ref)
{
	dev_err(info->cpu_dev,
		"cpu%d: %s %+lld..%+lld us: rg %d da %d srg %d sda %d da2 %d gen %u..%u%s%s%s\n",
		info->opp_cpu, what, (s64)(p->t0 - ref) / 1000, (s64)(p->t1 - ref) / 1000,
		p->uv[MTK_RB_RG], p->uv[MTK_RB_DA], p->uv[MTK_RB_SRG],
		p->uv[MTK_RB_SDA], p->uv[MTK_RB_DA2], p->gen0, p->gen1,
		p->busy0 ? " busy" : "", p->stable ? " stable" : " unstable",
		p->ret ? " read-error" : "");
}

/*
 * A failed check. The first one after "tripped" was clear writes one report:
 * the failing read-back, four fresh read-backs, the last rail changes (times
 * relative to the start of the failing read-back; self = this driver) and the
 * clocks read from the hardware. Later ones log one rate-limited line.
 */
static void mtk_lisbon_trip(struct mtk_cpu_dvfs_info *info, int old, int new,
			    const char *why, const struct mtk_lisbon_probe *p,
			    bool seq, unsigned int retry)
{
	struct mtk_lisbon_rail_ev ev[MTK_LISBON_HIST];
	struct mtk_lisbon_probe q;
	unsigned int head, i;
	u32 con1 = 0, mux = 0;

	WRITE_ONCE(tripped, true);
	if (atomic_xchg(&mtk_lisbon_reported, 1)) {
		dev_err_ratelimited(info->cpu_dev, "cpu%d: check failed again (%s), Vproc %d -> %d uV\n",
				    info->opp_cpu, why, old, new);
		return;
	}
	if (READ_ONCE(trip_trace))
		tracing_off();
	schedule_work(&mtk_lisbon_cap_work);

	dev_err(info->cpu_dev,
		"cpu%d: TRIP: %s: %s Vproc %d -> %d uV, read-back %s, %u retries; caps set to the boot OPP\n",
		info->opp_cpu, why, info->rail->vproc, old, new,
		seq ? "on a stable rail" : "unsynchronised (vcheck_seq=0)", retry);
	mtk_lisbon_print_probe(info, "failed read-back", p, p->t0);
	for (i = 0; i < 4; i++) {
		mtk_lisbon_sample(info, &q, true);
		mtk_lisbon_print_probe(info, "re-read", &q, p->t0);
	}

	spin_lock(&info->rail_lock);
	head = info->rail_head;
	memcpy(ev, info->rail_ev, sizeof(ev));
	spin_unlock(&info->rail_lock);
	for (i = 0; i < MTK_LISBON_HIST; i++) {
		const struct mtk_lisbon_rail_ev *e = &ev[(head + i) % MTK_LISBON_HIST];

		if (!e->type)
			continue;
		dev_err(info->cpu_dev, "cpu%d: rail %s %+lld us %c %d -> %d uV %s %d %s\n",
			info->opp_cpu, info->rail->vproc, (s64)(e->ns - p->t0) / 1000,
			e->type, e->old_uv, e->new_uv, e->self ? "self" : "other",
			e->pid, e->comm);
	}

	if (mtk_lisbon_apmixed && mtk_lisbon_mcusys) {
		con1 = readl(mtk_lisbon_apmixed + 0x2a4);
		mux = readl(mtk_lisbon_mcusys + 0x2e0);
	}
	dev_err(info->cpu_dev,
		"cpu%d: clocks: cluster hw %lu kHz (clk %lu), cci con1 0x%08x mux 0x%08x%s\n",
		info->opp_cpu, mtk_lisbon_hw_khz(info), clk_get_rate(info->cpu_clk) / 1000,
		con1, mux, READ_ONCE(trip_trace) ? ", ftrace stopped" : "");
}

/*
 * The read-back before vcheck_seq: four registers one after the other, no
 * matter who changes the rail meanwhile. Kept for comparison.
 */
static const char *mtk_lisbon_check_legacy(struct mtk_cpu_dvfs_info *info, int new,
					   struct mtk_lisbon_probe *p)
{
	int i;

	/* the DA value follows the soft ramp; the core waited the ramp delay */
	for (i = 0; i < 20; i++) {
		mtk_lisbon_sample(info, p, false);
		if (p->ret || p->uv[MTK_RB_DA] == p->uv[MTK_RB_RG])
			break;
		udelay(10);
	}

	if (p->ret)
		return "PMIC read failed";
	if (p->uv[MTK_RB_DA] < p->uv[MTK_RB_RG] && p->uv[MTK_RB_DA] < new)
		return "Vproc DA below the programmed value";
	if (p->uv[MTK_RB_SDA] < p->uv[MTK_RB_DA])
		return "Vsram DA below Vproc DA";
	return NULL;
}

/*
 * Read back until no rail change is in flight (budget MTK_LISBON_VERIFY_US).
 * On a stable sample: RG and both Vproc DA reads >= the request (the core
 * keeps the maximum of all consumer requests, so the other consumer can only
 * add to it), Vsram DA >= both Vproc DA reads. No stable sample in time
 * counts as a failure.
 */
static const char *mtk_lisbon_check(struct mtk_cpu_dvfs_info *info, int new,
				    struct mtk_lisbon_probe *p, unsigned int *retry)
{
	u64 start = local_clock(), us;
	int da_lo, da_hi;

	for (;;) {
		mtk_lisbon_sample(info, p, true);
		if (p->ret)
			return "PMIC read failed";
		/* DA = RG: the ramp is over (DA follows the soft ramp) */
		if (p->stable && p->uv[MTK_RB_DA] == p->uv[MTK_RB_RG] &&
		    p->uv[MTK_RB_DA2] == p->uv[MTK_RB_RG])
			break;
		if (local_clock() - start > MTK_LISBON_VERIFY_US * NSEC_PER_USEC) {
			if (!p->stable)
				return "no read-back on a stable rail";
			break;
		}
		(*retry)++;
		usleep_range(20, 50);
	}

	us = div_u64(local_clock() - start, NSEC_PER_USEC);
	info->nverify++;
	info->nretry += *retry;
	info->maxretry = max(info->maxretry, *retry);
	info->maxverify_us = max_t(unsigned int, info->maxverify_us, us);

	da_lo = min(p->uv[MTK_RB_DA], p->uv[MTK_RB_DA2]);
	da_hi = max(p->uv[MTK_RB_DA], p->uv[MTK_RB_DA2]);
	if (p->uv[MTK_RB_RG] < new)
		return "Vproc RG below the request";
	if (da_lo < new)
		return "Vproc DA below the request";
	if (p->uv[MTK_RB_SDA] < da_hi)
		return "Vsram DA below Vproc DA";
	return NULL;
}

/* Called after Vproc was set from old to new uV (regulator call succeeded). */
static int mtk_lisbon_verify(struct mtk_cpu_dvfs_info *info, int old, int new)
{
	bool seq = READ_ONCE(vcheck_seq) && info->rail_nb_on;
	struct mtk_lisbon_probe p = { };
	unsigned int retry = 0;
	const char *bad;
	int uv[4];

	if (!info->lisbon || !info->pmic || !READ_ONCE(vcheck))
		return 0;

	if (seq)
		bad = mtk_lisbon_check(info, new, &p, &retry);
	else
		bad = mtk_lisbon_check_legacy(info, new, &p);

	if (!bad && READ_ONCE(verbose))
		dev_info(info->cpu_dev,
			 "cpu%d: Vproc %d -> %d uV: %s rg %d da %d da2 %d, %s rg %d da %d, %u retries\n",
			 info->opp_cpu, old, new, info->rail->vproc, p.uv[MTK_RB_RG],
			 p.uv[MTK_RB_DA], p.uv[MTK_RB_DA2], info->rail->vsram,
			 p.uv[MTK_RB_SRG], p.uv[MTK_RB_SDA], retry);
	if (!bad)
		return 0;

	mtk_lisbon_trip(info, old, new, bad, &p, seq, retry);
	if (new > old && old > 0) {
		regulator_set_voltage(info->proc_reg, old,
				      info->soc_data->proc_max_volt);
		mtk_lisbon_volts(info, uv);
		dev_err_ratelimited(info->cpu_dev,
				    "cpu%d: tripped, Vproc back to %d uV (da %d, Vsram da %d)\n",
				    info->opp_cpu, old, uv[1], uv[3]);
	}
	return -EIO;
}

static int mtk_status_get(char *buf, const struct kernel_param *kp)
{
	struct mtk_cpu_dvfs_info *info;
	int len = 0;

	mutex_lock(&mtk_lisbon_lock);
	list_for_each_entry(info, &dvfs_info_list, list_head) {
		u32 con1 = 0, mux = 0, t[3] = { };
		unsigned long pll = 0;
		int uv[4] = { };

		if (!info->lisbon)
			continue;
		if (mtk_lisbon_apmixed && mtk_lisbon_mcusys) {
			con1 = readl(mtk_lisbon_apmixed + info->pll_con1);
			mux = readl(mtk_lisbon_mcusys + info->mux_reg);
			pll = mtk_lisbon_pll_khz(con1);
		}
		len += scnprintf(buf + len, PAGE_SIZE - len,
				 "policy%d cpus=%*pbl cap=%u boot=%u cur=%lu pll=%lu con1=0x%08x mux=0x%08x hw=%lu",
				 cpumask_first(&info->cpus), cpumask_pr_args(&info->cpus),
				 mtk_cap_khz(info), info->boot_khz,
				 clk_get_rate(info->cpu_clk) / 1000, pll, con1, mux,
				 mtk_lisbon_out_khz(pll, mux, clk_get_rate(info->inter_clk) / 1000));
		if (info->pmic) {
			mtk_lisbon_volts(info, uv);
			regmap_read(info->pmic, info->rail->track0, &t[0]);
			regmap_read(info->pmic, info->rail->track0 + 2, &t[1]);
			regmap_read(info->pmic, info->rail->track0 + 4, &t[2]);
			len += scnprintf(buf + len, PAGE_SIZE - len,
					 " rail=%s vproc_rg=%d vproc_da=%d vsram_rg=%d vsram_da=%d track_en=%u track0=0x%04x track1=0x%04x track2=0x%04x rail_nb=%d rail_gen=%u verify=%u retry=%u maxretry=%u maxverify_us=%u",
					 info->rail->vproc, uv[0], uv[1], uv[2], uv[3],
					 t[0] & 1, t[0], t[1], t[2], info->rail_nb_on,
					 READ_ONCE(info->rail_gen), info->nverify, info->nretry,
					 info->maxretry, info->maxverify_us);
		} else {
			len += scnprintf(buf + len, PAGE_SIZE - len, " pmic=none");
		}
		len += scnprintf(buf + len, PAGE_SIZE - len, "\n");
	}
	mutex_unlock(&mtk_lisbon_lock);

	if (mtk_lisbon_apmixed && mtk_lisbon_mcusys) {
		u32 con1 = readl(mtk_lisbon_apmixed + 0x2a4);
		u32 mux = readl(mtk_lisbon_mcusys + 0x2e0);
		unsigned long pll = mtk_lisbon_pll_khz(con1);

		/* CCI: only the PLL path is decoded (hw=0 on 26M/intermediate) */
		unsigned long hw = ((mux >> 9) & 3) == 1 ? mtk_lisbon_out_khz(pll, mux, 0) : 0;

		len += scnprintf(buf + len, PAGE_SIZE - len,
				 "cci pll=%lu con1=0x%08x mux=0x%08x hw=%lu\n",
				 pll, con1, mux, hw);
	}
	len += scnprintf(buf + len, PAGE_SIZE - len,
			 "global vcheck=%d vcheck_seq=%d tripped=%d trip_trace=%d verbose=%d\n",
			 READ_ONCE(vcheck), READ_ONCE(vcheck_seq), READ_ONCE(tripped),
			 READ_ONCE(trip_trace), READ_ONCE(verbose));
	return len;
}

static const struct kernel_param_ops mtk_status_ops = {
	.get = mtk_status_get,
};
module_param_cb(status, &mtk_status_ops, NULL, 0444);
MODULE_PARM_DESC(status, "MT6785 read-back (caps, PLL, Vproc/Vsram)");

static void mtk_lisbon_init(struct mtk_cpu_dvfs_info *info)
{
	unsigned int vsel_reg, vsel_mask, i;
	struct device_node *np;
	struct regmap *map;
	unsigned long rate;
	struct dev_pm_opp *opp;
	u32 id = 0, t0 = 0;
	int uv[4], ret;

	if (!of_machine_is_compatible("mediatek,mt6785"))
		return;
	info->lisbon = true;

	/* cap 0 = the OPP the cluster runs at (lk leaves 1174999/1084999 kHz) */
	rate = clk_get_rate(info->cpu_clk);
	opp = dev_pm_opp_find_freq_ceil(info->cpu_dev, &rate);
	if (IS_ERR(opp)) {
		rate = clk_get_rate(info->cpu_clk);
		opp = dev_pm_opp_find_freq_floor(info->cpu_dev, &rate);
	}
	if (!IS_ERR(opp))
		dev_pm_opp_put(opp);
	info->boot_khz = IS_ERR(opp) ? 0 : rate / 1000;

	if (cpumask_test_cpu(6, &info->cpus)) {
		info->pll_con1 = 0x214;	/* ARMPLL_BL (vendor name ARMPLL_L_CON1) */
		info->mux_reg = 0x2a4;	/* MP1_PLL_DIVIDER */
	} else {
		info->pll_con1 = 0x204;	/* ARMPLL_LL */
		info->mux_reg = 0x2a0;	/* MP0_PLL_DIVIDER */
	}
	if (!mtk_lisbon_apmixed) {
		np = of_find_compatible_node(NULL, NULL, "mediatek,mt6785-apmixedsys");
		mtk_lisbon_apmixed = np ? of_iomap(np, 0) : NULL;
		of_node_put(np);
	}
	if (!mtk_lisbon_mcusys) {
		np = of_find_compatible_node(NULL, NULL, "mediatek,mt6785-mcusys");
		mtk_lisbon_mcusys = np ? of_iomap(np, 0) : NULL;
		of_node_put(np);
	}

	if (regulator_get_hardware_vsel_register(info->proc_reg, &vsel_reg, &vsel_mask)) {
		dev_warn(info->cpu_dev, "cpu%d: no vsel register, no PMIC read-back\n", info->opp_cpu);
		return;
	}
	for (i = 0; i < ARRAY_SIZE(mtk_lisbon_rails); i++)
		if (mtk_lisbon_rails[i].vproc_rg == vsel_reg)
			info->rail = &mtk_lisbon_rails[i];
	map = regulator_get_regmap(info->proc_reg);
	if (!info->rail || IS_ERR_OR_NULL(map) || regmap_read(map, MTK_LISBON_HWCID, &id) ||
	    id < 0x5900 || id >= 0x5930) {
		dev_warn(info->cpu_dev, "cpu%d: vsel 0x%x, PMIC id 0x%x not the MT6359 map, no read-back\n",
			 info->opp_cpu, vsel_reg, id);
		info->rail = NULL;
		return;
	}
	info->pmic = map;

	spin_lock_init(&info->rail_lock);
	info->rail_nb.notifier_call = mtk_lisbon_rail_event;
	ret = regulator_register_notifier(info->proc_reg, &info->rail_nb);
	if (ret)
		dev_warn(info->cpu_dev, "cpu%d: no rail notifier (%d), unsynchronised read-back\n",
			 info->opp_cpu, ret);
	info->rail_nb_on = !ret;

	mtk_lisbon_volts(info, uv);
	regmap_read(map, info->rail->track0, &t0);
	dev_info(info->cpu_dev,
		 "cpu%d: boot %u kHz, %s rg %d da %d uV, %s rg %d da %d uV, track0 0x%04x, cap %u kHz\n",
		 info->opp_cpu, info->boot_khz, info->rail->vproc, uv[0], uv[1],
		 info->rail->vsram, uv[2], uv[3], t0, mtk_cap_khz(info));
}

static int mtk_cpufreq_voltage_tracking(struct mtk_cpu_dvfs_info *info,
					int new_vproc)
{
	const struct mtk_cpufreq_platform_data *soc_data = info->soc_data;
	struct regulator *proc_reg = info->proc_reg;
	struct regulator *sram_reg = info->sram_reg;
	int pre_vproc, pre_vsram, new_vsram, vsram, vproc, ret;
	int retry = info->vtrack_max;

	pre_vproc = regulator_get_voltage(proc_reg);
	if (pre_vproc < 0) {
		dev_err(info->cpu_dev,
			"invalid Vproc value: %d\n", pre_vproc);
		return pre_vproc;
	}

	pre_vsram = regulator_get_voltage(sram_reg);
	if (pre_vsram < 0) {
		dev_err(info->cpu_dev, "invalid Vsram value: %d\n", pre_vsram);
		return pre_vsram;
	}

	new_vsram = clamp(new_vproc + soc_data->min_volt_shift,
			  soc_data->sram_min_volt, soc_data->sram_max_volt);

	do {
		if (pre_vproc <= new_vproc) {
			vsram = clamp(pre_vproc + soc_data->max_volt_shift,
				      soc_data->sram_min_volt, new_vsram);
			ret = regulator_set_voltage(sram_reg, vsram,
						    soc_data->sram_max_volt);

			if (ret)
				return ret;

			if (vsram == soc_data->sram_max_volt ||
			    new_vsram == soc_data->sram_min_volt)
				vproc = new_vproc;
			else
				vproc = vsram - soc_data->min_volt_shift;

			ret = regulator_set_voltage(proc_reg, vproc,
						    soc_data->proc_max_volt);
			if (ret) {
				regulator_set_voltage(sram_reg, pre_vsram,
						      soc_data->sram_max_volt);
				return ret;
			}
		} else {
			vproc = max(new_vproc,
				    pre_vsram - soc_data->max_volt_shift);
			ret = regulator_set_voltage(proc_reg, vproc,
						    soc_data->proc_max_volt);
			if (ret)
				return ret;

			if (vproc == new_vproc)
				vsram = new_vsram;
			else
				vsram = max(new_vsram,
					    vproc + soc_data->min_volt_shift);

			ret = regulator_set_voltage(sram_reg, vsram,
						    soc_data->sram_max_volt);
			if (ret) {
				regulator_set_voltage(proc_reg, pre_vproc,
						      soc_data->proc_max_volt);
				return ret;
			}
		}

		pre_vproc = vproc;
		pre_vsram = vsram;

		if (--retry < 0) {
			dev_err(info->cpu_dev,
				"over loop count, failed to set voltage\n");
			return -EINVAL;
		}
	} while (vproc != new_vproc || vsram != new_vsram);

	return 0;
}

static int mtk_cpufreq_set_voltage(struct mtk_cpu_dvfs_info *info, int vproc)
{
	const struct mtk_cpufreq_platform_data *soc_data = info->soc_data;
	int old = 0, ret;

	if (info->lisbon) {
		old = regulator_get_voltage(info->proc_reg);
		ret = mtk_lisbon_precheck(info, old, vproc);
		if (ret)
			return ret;
	}

	WRITE_ONCE(info->rail_owner, current);
	if (info->need_voltage_tracking)
		ret = mtk_cpufreq_voltage_tracking(info, vproc);
	else
		ret = regulator_set_voltage(info->proc_reg, vproc,
					    soc_data->proc_max_volt);
	WRITE_ONCE(info->rail_owner, NULL);
	if (!ret && info->lisbon)
		ret = mtk_lisbon_verify(info, old, vproc);
	if (!ret)
		info->pre_vproc = vproc;

	return ret;
}

static bool is_ccifreq_ready(struct mtk_cpu_dvfs_info *info)
{
	struct device_link *sup_link;

	if (info->ccifreq_bound)
		return true;

	sup_link = device_link_add(info->cpu_dev, info->cci_dev,
				   DL_FLAG_AUTOREMOVE_CONSUMER);
	if (!sup_link) {
		dev_err(info->cpu_dev, "cpu%d: sup_link is NULL\n", info->opp_cpu);
		return false;
	}

	if (sup_link->supplier->links.status != DL_DEV_DRIVER_BOUND)
		return false;

	info->ccifreq_bound = true;

	return true;
}

static int mtk_cpufreq_set_target(struct cpufreq_policy *policy,
				  unsigned int index)
{
	struct cpufreq_frequency_table *freq_table = policy->freq_table;
	struct clk *cpu_clk = policy->clk;
	struct clk *armpll = clk_get_parent(cpu_clk);
	struct mtk_cpu_dvfs_info *info = policy->driver_data;
	struct device *cpu_dev = info->cpu_dev;
	struct dev_pm_opp *opp;
	long freq_hz, pre_freq_hz;
	int vproc, pre_vproc, inter_vproc, target_vproc, ret;

	inter_vproc = info->intermediate_voltage;

	pre_freq_hz = clk_get_rate(cpu_clk);

	if (mtk_cap_param(info) && freq_table[index].frequency > mtk_cap_khz(info)) {
		dev_warn_ratelimited(cpu_dev, "cpu%d: %u kHz above cap %u kHz, refused\n",
				     policy->cpu, freq_table[index].frequency,
				     mtk_cap_khz(info));
		return -EPERM;
	}

	if (READ_ONCE(dry_run)) {
		unsigned long f = freq_table[index].frequency * 1000UL;

		opp = dev_pm_opp_find_freq_ceil(cpu_dev, &f);
		vproc = IS_ERR(opp) ? PTR_ERR(opp) : dev_pm_opp_get_voltage(opp);
		if (!IS_ERR(opp))
			dev_pm_opp_put(opp);
		dev_info(cpu_dev, "cpu%d: dry run: %ld -> %u kHz, Vproc %d -> %d uV (OPP), boot %d uV, intermediate %d uV, cci %s; nothing changed\n",
			 policy->cpu, pre_freq_hz / 1000, freq_table[index].frequency,
			 regulator_get_voltage(info->proc_reg), vproc, info->vproc_on_boot,
			 inter_vproc, info->ccifreq_bound ? "bound" : "not bound");
		return 0;
	}

	mutex_lock(&info->reg_lock);

	if (unlikely(info->pre_vproc <= 0))
		pre_vproc = regulator_get_voltage(info->proc_reg);
	else
		pre_vproc = info->pre_vproc;

	if (pre_vproc < 0) {
		dev_err(cpu_dev, "invalid Vproc value: %d\n", pre_vproc);
		ret = pre_vproc;
		goto out;
	}

	freq_hz = freq_table[index].frequency * 1000;

	opp = dev_pm_opp_find_freq_ceil(cpu_dev, &freq_hz);
	if (IS_ERR(opp)) {
		dev_err(cpu_dev, "cpu%d: failed to find OPP for %ld\n",
			policy->cpu, freq_hz);
		ret = PTR_ERR(opp);
		goto out;
	}
	vproc = dev_pm_opp_get_voltage(opp);
	dev_pm_opp_put(opp);

	/*
	 * If MediaTek cci is supported but is not ready, we will use the value
	 * of max(target cpu voltage, booting voltage) to prevent high freqeuncy
	 * low voltage crash.
	 */
	if (info->soc_data->ccifreq_supported && !is_ccifreq_ready(info))
		vproc = max(vproc, info->vproc_on_boot);

	/*
	 * If the new voltage or the intermediate voltage is higher than the
	 * current voltage, scale up voltage first.
	 */
	target_vproc = max(inter_vproc, vproc);
	if (pre_vproc <= target_vproc) {
		ret = mtk_cpufreq_set_voltage(info, target_vproc);
		if (ret) {
			dev_err_ratelimited(cpu_dev, "cpu%d: failed to scale up voltage!\n",
					    policy->cpu);
			mtk_cpufreq_set_voltage(info, pre_vproc);
			goto out;
		}
	}

	/* Reparent the CPU clock to intermediate clock. */
	ret = clk_set_parent(cpu_clk, info->inter_clk);
	if (ret) {
		dev_err(cpu_dev,
			"cpu%d: failed to re-parent cpu clock!\n", policy->cpu);
		mtk_cpufreq_set_voltage(info, pre_vproc);
		goto out;
	}

	/* Set the original PLL to target rate. */
	ret = clk_set_rate(armpll, freq_hz);
	if (ret) {
		dev_err(cpu_dev,
			"cpu%d: failed to scale cpu clock rate!\n", policy->cpu);
		clk_set_parent(cpu_clk, armpll);
		mtk_cpufreq_set_voltage(info, pre_vproc);
		goto out;
	}

	/* Set parent of CPU clock back to the original PLL. */
	ret = clk_set_parent(cpu_clk, armpll);
	if (ret) {
		dev_err(cpu_dev,
			"cpu%d: failed to re-parent cpu clock!\n", policy->cpu);
		mtk_cpufreq_set_voltage(info, inter_vproc);
		goto out;
	}

	/*
	 * If the new voltage is lower than the intermediate voltage or the
	 * original voltage, scale down to the new voltage.
	 */
	if (vproc < inter_vproc || vproc < pre_vproc) {
		ret = mtk_cpufreq_set_voltage(info, vproc);
		if (ret) {
			dev_err_ratelimited(cpu_dev, "cpu%d: failed to scale down voltage!\n",
					    policy->cpu);
			clk_set_parent(cpu_clk, info->inter_clk);
			clk_set_rate(armpll, pre_freq_hz);
			clk_set_parent(cpu_clk, armpll);
			goto out;
		}
	}

	info->current_freq = freq_hz;
	if (info->lisbon && READ_ONCE(verbose))
		dev_info(cpu_dev, "cpu%d: %ld -> %ld kHz, Vproc %d -> %d uV (OPP %d)\n",
			 policy->cpu, pre_freq_hz / 1000, freq_hz / 1000, pre_vproc,
			 info->pre_vproc, vproc);

out:
	mutex_unlock(&info->reg_lock);

	return ret;
}

static int mtk_cpufreq_opp_notifier(struct notifier_block *nb,
				    unsigned long event, void *data)
{
	struct dev_pm_opp *opp = data;
	struct dev_pm_opp *new_opp;
	struct mtk_cpu_dvfs_info *info;
	unsigned long freq, volt;
	int ret = 0;

	info = container_of(nb, struct mtk_cpu_dvfs_info, opp_nb);

	if (event == OPP_EVENT_ADJUST_VOLTAGE) {
		freq = dev_pm_opp_get_freq(opp);

		mutex_lock(&info->reg_lock);
		if (info->current_freq == freq) {
			volt = dev_pm_opp_get_voltage(opp);
			ret = mtk_cpufreq_set_voltage(info, volt);
			if (ret)
				dev_err(info->cpu_dev,
					"failed to scale voltage: %d\n", ret);
		}
		mutex_unlock(&info->reg_lock);
	} else if (event == OPP_EVENT_DISABLE) {
		freq = dev_pm_opp_get_freq(opp);

		/* case of current opp item is disabled */
		if (info->current_freq == freq) {
			freq = 1;
			new_opp = dev_pm_opp_find_freq_ceil(info->cpu_dev,
							    &freq);
			if (IS_ERR(new_opp)) {
				dev_err(info->cpu_dev,
					"all opp items are disabled\n");
				ret = PTR_ERR(new_opp);
				return notifier_from_errno(ret);
			}

			dev_pm_opp_put(new_opp);

			struct cpufreq_policy *policy __free(put_cpufreq_policy)
				= cpufreq_cpu_get(info->opp_cpu);
			if (policy)
				cpufreq_driver_target(policy, freq / 1000,
						      CPUFREQ_RELATION_L);
		}
	}

	return notifier_from_errno(ret);
}

static struct device *of_get_cci(struct device *cpu_dev)
{
	struct device_node *np;
	struct platform_device *pdev;

	np = of_parse_phandle(cpu_dev->of_node, "mediatek,cci", 0);
	if (!np)
		return ERR_PTR(-ENODEV);

	pdev = of_find_device_by_node(np);
	of_node_put(np);
	if (!pdev)
		return ERR_PTR(-ENODEV);

	return &pdev->dev;
}

static int mtk_cpu_dvfs_info_init(struct mtk_cpu_dvfs_info *info, int cpu)
{
	struct device *cpu_dev;
	struct dev_pm_opp *opp;
	unsigned long rate;
	int ret;

	cpu_dev = get_cpu_device(cpu);
	if (!cpu_dev)
		return dev_err_probe(cpu_dev, -ENODEV, "failed to get cpu%d device\n", cpu);
	info->cpu_dev = cpu_dev;

	info->ccifreq_bound = false;
	if (info->soc_data->ccifreq_supported) {
		info->cci_dev = of_get_cci(info->cpu_dev);
		if (IS_ERR(info->cci_dev))
			return dev_err_probe(cpu_dev, PTR_ERR(info->cci_dev),
					     "cpu%d: failed to get cci device\n",
					     cpu);
	}

	info->cpu_clk = clk_get(cpu_dev, "cpu");
	if (IS_ERR(info->cpu_clk)) {
		ret = PTR_ERR(info->cpu_clk);
		dev_err_probe(cpu_dev, ret, "cpu%d: failed to get cpu clk\n", cpu);
		goto out_put_cci_dev;
	}

	info->inter_clk = clk_get(cpu_dev, "intermediate");
	if (IS_ERR(info->inter_clk)) {
		ret = PTR_ERR(info->inter_clk);
		dev_err_probe(cpu_dev, ret,
			      "cpu%d: failed to get intermediate clk\n", cpu);
		goto out_free_mux_clock;
	}

	info->proc_reg = regulator_get_optional(cpu_dev, "proc");
	if (IS_ERR(info->proc_reg)) {
		ret = PTR_ERR(info->proc_reg);
		dev_err_probe(cpu_dev, ret,
			      "cpu%d: failed to get proc regulator\n", cpu);
		goto out_free_inter_clock;
	}

	ret = regulator_enable(info->proc_reg);
	if (ret) {
		dev_err_probe(cpu_dev, ret, "cpu%d: failed to enable vproc\n", cpu);
		goto out_free_proc_reg;
	}

	/* Both presence and absence of sram regulator are valid cases. */
	info->sram_reg = regulator_get_optional(cpu_dev, "sram");
	if (IS_ERR(info->sram_reg)) {
		ret = PTR_ERR(info->sram_reg);
		if (ret == -EPROBE_DEFER) {
			dev_err_probe(cpu_dev, ret,
				      "cpu%d: Failed to get sram regulator\n", cpu);
			goto out_disable_proc_reg;
		}

		info->sram_reg = NULL;
	} else {
		ret = regulator_enable(info->sram_reg);
		if (ret) {
			dev_err_probe(cpu_dev, ret, "cpu%d: failed to enable vsram\n", cpu);
			goto out_free_sram_reg;
		}
	}

	/* Get OPP-sharing information from "operating-points-v2" bindings */
	ret = dev_pm_opp_of_get_sharing_cpus(cpu_dev, &info->cpus);
	if (ret) {
		dev_err_probe(cpu_dev, ret,
			"cpu%d: failed to get OPP-sharing information\n", cpu);
		goto out_disable_sram_reg;
	}

	ret = dev_pm_opp_of_cpumask_add_table(&info->cpus);
	if (ret) {
		dev_err_probe(cpu_dev, ret, "cpu%d: no OPP table\n", cpu);
		goto out_disable_sram_reg;
	}

	ret = clk_prepare_enable(info->cpu_clk);
	if (ret) {
		dev_err_probe(cpu_dev, ret, "cpu%d: failed to enable cpu clk\n", cpu);
		goto out_free_opp_table;
	}

	ret = clk_prepare_enable(info->inter_clk);
	if (ret) {
		dev_err_probe(cpu_dev, ret, "cpu%d: failed to enable inter clk\n", cpu);
		goto out_disable_mux_clock;
	}

	if (info->soc_data->ccifreq_supported) {
		info->vproc_on_boot = regulator_get_voltage(info->proc_reg);
		if (info->vproc_on_boot < 0) {
			ret = dev_err_probe(info->cpu_dev, info->vproc_on_boot,
					    "invalid Vproc value\n");
			goto out_disable_inter_clock;
		}
	}

	/* Search a safe voltage for intermediate frequency. */
	rate = clk_get_rate(info->inter_clk);
	opp = dev_pm_opp_find_freq_ceil(cpu_dev, &rate);
	if (IS_ERR(opp)) {
		ret = dev_err_probe(cpu_dev, PTR_ERR(opp),
				    "cpu%d: failed to get intermediate opp\n", cpu);
		goto out_disable_inter_clock;
	}
	info->intermediate_voltage = dev_pm_opp_get_voltage(opp);
	dev_pm_opp_put(opp);

	mutex_init(&info->reg_lock);
	info->current_freq = clk_get_rate(info->cpu_clk);

	info->opp_cpu = cpu;
	info->opp_nb.notifier_call = mtk_cpufreq_opp_notifier;
	ret = dev_pm_opp_register_notifier(cpu_dev, &info->opp_nb);
	if (ret) {
		dev_err_probe(cpu_dev, ret, "cpu%d: failed to register opp notifier\n", cpu);
		goto out_disable_inter_clock;
	}

	/*
	 * If SRAM regulator is present, software "voltage tracking" is needed
	 * for this CPU power domain.
	 */
	info->need_voltage_tracking = (info->sram_reg != NULL);

	/*
	 * We assume min voltage is 0 and tracking target voltage using
	 * min_volt_shift for each iteration.
	 * The vtrack_max is 3 times of expeted iteration count.
	 */
	info->vtrack_max = 3 * DIV_ROUND_UP(max(info->soc_data->sram_max_volt,
						info->soc_data->proc_max_volt),
					    info->soc_data->min_volt_shift);

	mtk_lisbon_init(info);

	return 0;

out_disable_inter_clock:
	clk_disable_unprepare(info->inter_clk);

out_disable_mux_clock:
	clk_disable_unprepare(info->cpu_clk);

out_free_opp_table:
	dev_pm_opp_of_cpumask_remove_table(&info->cpus);

out_disable_sram_reg:
	if (info->sram_reg)
		regulator_disable(info->sram_reg);

out_free_sram_reg:
	if (info->sram_reg)
		regulator_put(info->sram_reg);

out_disable_proc_reg:
	regulator_disable(info->proc_reg);

out_free_proc_reg:
	regulator_put(info->proc_reg);

out_free_inter_clock:
	clk_put(info->inter_clk);

out_free_mux_clock:
	clk_put(info->cpu_clk);

out_put_cci_dev:
	if (info->soc_data->ccifreq_supported)
		put_device(info->cci_dev);

	return ret;
}

static void mtk_cpu_dvfs_info_release(struct mtk_cpu_dvfs_info *info)
{
	if (info->rail_nb_on)
		regulator_unregister_notifier(info->proc_reg, &info->rail_nb);
	info->rail_nb_on = false;
	regulator_disable(info->proc_reg);
	regulator_put(info->proc_reg);
	if (info->sram_reg) {
		regulator_disable(info->sram_reg);
		regulator_put(info->sram_reg);
	}
	clk_disable_unprepare(info->cpu_clk);
	clk_put(info->cpu_clk);
	clk_disable_unprepare(info->inter_clk);
	clk_put(info->inter_clk);
	dev_pm_opp_of_cpumask_remove_table(&info->cpus);
	dev_pm_opp_unregister_notifier(info->cpu_dev, &info->opp_nb);
	if (info->soc_data->ccifreq_supported)
		put_device(info->cci_dev);
}

static int mtk_cpufreq_init(struct cpufreq_policy *policy)
{
	struct mtk_cpu_dvfs_info *info;
	struct cpufreq_frequency_table *freq_table;
	int ret;

	info = mtk_cpu_dvfs_info_lookup(policy->cpu);
	if (!info) {
		pr_err("dvfs info for cpu%d is not initialized.\n",
			policy->cpu);
		return -EINVAL;
	}

	ret = dev_pm_opp_init_cpufreq_table(info->cpu_dev, &freq_table);
	if (ret) {
		dev_err(info->cpu_dev,
			"failed to init cpufreq table for cpu%d: %d\n",
			policy->cpu, ret);
		return ret;
	}

	cpumask_copy(policy->cpus, &info->cpus);
	policy->freq_table = freq_table;
	policy->driver_data = info;
	policy->clk = info->cpu_clk;
	policy->transition_delay_us = info->soc_data->transition_delay_us;

	if (mtk_cap_param(info)) {
		mutex_lock(&mtk_lisbon_lock);
		ret = freq_qos_add_request(&policy->constraints, &info->cap_req,
					   FREQ_QOS_MAX, mtk_cap_khz(info));
		mutex_unlock(&mtk_lisbon_lock);
		if (ret < 0) {
			dev_err(info->cpu_dev, "cpu%d: cap request failed: %d\n",
				policy->cpu, ret);
			dev_pm_opp_free_cpufreq_table(info->cpu_dev, &policy->freq_table);
			return ret;
		}
	}

	return 0;
}

static void mtk_cpufreq_exit(struct cpufreq_policy *policy)
{
	struct mtk_cpu_dvfs_info *info = policy->driver_data;

	mutex_lock(&mtk_lisbon_lock);
	if (freq_qos_request_active(&info->cap_req))
		freq_qos_remove_request(&info->cap_req);
	mutex_unlock(&mtk_lisbon_lock);
	dev_pm_opp_free_cpufreq_table(info->cpu_dev, &policy->freq_table);
}

static struct cpufreq_driver mtk_cpufreq_driver = {
	.flags = CPUFREQ_NEED_INITIAL_FREQ_CHECK |
		 CPUFREQ_HAVE_GOVERNOR_PER_POLICY |
		 CPUFREQ_IS_COOLING_DEV,
	.verify = cpufreq_generic_frequency_table_verify,
	.target_index = mtk_cpufreq_set_target,
	.get = cpufreq_generic_get,
	.init = mtk_cpufreq_init,
	.exit = mtk_cpufreq_exit,
	.register_em = cpufreq_register_em_with_opp,
	.name = "mtk-cpufreq",
};

static int mtk_cpufreq_probe(struct platform_device *pdev)
{
	const struct mtk_cpufreq_platform_data *data;
	struct mtk_cpu_dvfs_info *info, *tmp;
	int cpu, ret;

	if (READ_ONCE(hold)) {
		dev_info(&pdev->dev, "held (mediatek_cpufreq.hold): echo 0 > /sys/module/mediatek_cpufreq/parameters/hold; echo %s > /sys/bus/platform/drivers_probe\n",
			 dev_name(&pdev->dev));
		return -ENODEV;
	}

	data = dev_get_platdata(&pdev->dev);
	if (!data)
		return dev_err_probe(&pdev->dev, -ENODEV,
				     "failed to get mtk cpufreq platform data\n");

	for_each_present_cpu(cpu) {
		info = mtk_cpu_dvfs_info_lookup(cpu);
		if (info)
			continue;

		info = devm_kzalloc(&pdev->dev, sizeof(*info), GFP_KERNEL);
		if (!info) {
			ret = dev_err_probe(&pdev->dev, -ENOMEM,
					    "Failed to allocate dvfs_info\n");
			goto release_dvfs_info_list;
		}

		info->soc_data = data;
		ret = mtk_cpu_dvfs_info_init(info, cpu);
		if (ret)
			goto release_dvfs_info_list;

		mutex_lock(&mtk_lisbon_lock);
		list_add(&info->list_head, &dvfs_info_list);
		mutex_unlock(&mtk_lisbon_lock);
	}

	ret = cpufreq_register_driver(&mtk_cpufreq_driver);
	if (ret) {
		dev_err_probe(&pdev->dev, ret, "failed to register mtk cpufreq driver\n");
		goto release_dvfs_info_list;
	}

	return 0;

release_dvfs_info_list:
	list_for_each_entry_safe(info, tmp, &dvfs_info_list, list_head) {
		mtk_cpu_dvfs_info_release(info);
		mutex_lock(&mtk_lisbon_lock);
		list_del(&info->list_head);
		mutex_unlock(&mtk_lisbon_lock);
	}

	return ret;
}

static struct platform_driver mtk_cpufreq_platdrv = {
	.driver = {
		.name	= "mtk-cpufreq",
	},
	.probe		= mtk_cpufreq_probe,
};

static const struct mtk_cpufreq_platform_data mt2701_platform_data = {
	.min_volt_shift = 100000,
	.max_volt_shift = 200000,
	.proc_max_volt = 1150000,
	.sram_min_volt = 0,
	.sram_max_volt = 1150000,
	.ccifreq_supported = false,
};

/*
 * MT6785: a transition is a synchronous sequence of MT6359 voltage changes
 * over the PMIC wrapper, two mux switches and a PLL relock, and on cpu0-5
 * the CCI devfreq (passive governor) usually follows with the same sequence
 * on the shared VPROC2 rail. Together this takes about 1.2 ms in the sugov
 * kthread (up to 3.6 ms), longer than the 1 ms core default. Re-evaluate
 * at most every 4 ms.
 */
static const struct mtk_cpufreq_platform_data mt6785_platform_data = {
	.min_volt_shift = 100000,
	.max_volt_shift = 250000,
	.proc_max_volt = 1118750,
	.sram_min_volt = 850000,
	.sram_max_volt = 1118750,
	.ccifreq_supported = true,
	.transition_delay_us = 4000,
};

static const struct mtk_cpufreq_platform_data mt7622_platform_data = {
	.min_volt_shift = 100000,
	.max_volt_shift = 200000,
	.proc_max_volt = 1350000,
	.sram_min_volt = 0,
	.sram_max_volt = 1350000,
	.ccifreq_supported = false,
};

static const struct mtk_cpufreq_platform_data mt7623_platform_data = {
	.min_volt_shift = 100000,
	.max_volt_shift = 200000,
	.proc_max_volt = 1300000,
	.ccifreq_supported = false,
};

static const struct mtk_cpufreq_platform_data mt7988_platform_data = {
	.min_volt_shift = 100000,
	.max_volt_shift = 200000,
	.proc_max_volt = 900000,
	.sram_min_volt = 0,
	.sram_max_volt = 1150000,
	.ccifreq_supported = true,
};

static const struct mtk_cpufreq_platform_data mt8183_platform_data = {
	.min_volt_shift = 100000,
	.max_volt_shift = 200000,
	.proc_max_volt = 1150000,
	.sram_min_volt = 0,
	.sram_max_volt = 1150000,
	.ccifreq_supported = true,
};

static const struct mtk_cpufreq_platform_data mt8186_platform_data = {
	.min_volt_shift = 100000,
	.max_volt_shift = 250000,
	.proc_max_volt = 1118750,
	.sram_min_volt = 850000,
	.sram_max_volt = 1118750,
	.ccifreq_supported = true,
};

static const struct mtk_cpufreq_platform_data mt8516_platform_data = {
	.min_volt_shift = 100000,
	.max_volt_shift = 200000,
	.proc_max_volt = 1310000,
	.sram_min_volt = 0,
	.sram_max_volt = 1310000,
	.ccifreq_supported = false,
};

/* List of machines supported by this driver */
static const struct of_device_id mtk_cpufreq_machines[] __initconst __maybe_unused = {
	{ .compatible = "mediatek,mt2701", .data = &mt2701_platform_data },
	{ .compatible = "mediatek,mt2712", .data = &mt2701_platform_data },
	{ .compatible = "mediatek,mt6785", .data = &mt6785_platform_data },
	{ .compatible = "mediatek,mt7622", .data = &mt7622_platform_data },
	{ .compatible = "mediatek,mt7623", .data = &mt7623_platform_data },
	{ .compatible = "mediatek,mt7988a", .data = &mt7988_platform_data },
	{ .compatible = "mediatek,mt8167", .data = &mt8516_platform_data },
	{ .compatible = "mediatek,mt817x", .data = &mt2701_platform_data },
	{ .compatible = "mediatek,mt8173", .data = &mt2701_platform_data },
	{ .compatible = "mediatek,mt8176", .data = &mt2701_platform_data },
	{ .compatible = "mediatek,mt8183", .data = &mt8183_platform_data },
	{ .compatible = "mediatek,mt8186", .data = &mt8186_platform_data },
	{ .compatible = "mediatek,mt8365", .data = &mt2701_platform_data },
	{ .compatible = "mediatek,mt8516", .data = &mt8516_platform_data },
	{ }
};
MODULE_DEVICE_TABLE(of, mtk_cpufreq_machines);

static int __init mtk_cpufreq_driver_init(void)
{
	const struct mtk_cpufreq_platform_data *data;
	int err;

	data = of_machine_get_match_data(mtk_cpufreq_machines);
	if (!data) {
		pr_debug("Machine is not compatible with mtk-cpufreq\n");
		return -ENODEV;
	}

	err = platform_driver_register(&mtk_cpufreq_platdrv);
	if (err)
		return err;

	/*
	 * Since there's no place to hold device registration code and no
	 * device tree based way to match cpufreq driver yet, both the driver
	 * and the device registration codes are put here to handle defer
	 * probing.
	 */
	cpufreq_pdev = platform_device_register_data(NULL, "mtk-cpufreq", -1,
						     data, sizeof(*data));
	if (IS_ERR(cpufreq_pdev)) {
		pr_err("failed to register mtk-cpufreq platform device\n");
		platform_driver_unregister(&mtk_cpufreq_platdrv);
		return PTR_ERR(cpufreq_pdev);
	}

	return 0;
}
module_init(mtk_cpufreq_driver_init)

static void __exit mtk_cpufreq_driver_exit(void)
{
	platform_device_unregister(cpufreq_pdev);
	platform_driver_unregister(&mtk_cpufreq_platdrv);
}
module_exit(mtk_cpufreq_driver_exit)

MODULE_DESCRIPTION("MediaTek CPUFreq driver");
MODULE_AUTHOR("Pi-Cheng Chen <pi-cheng.chen@linaro.org>");
MODULE_LICENSE("GPL v2");
