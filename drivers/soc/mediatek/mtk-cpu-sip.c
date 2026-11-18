// SPDX-License-Identifier: GPL-2.0-only
/*
 * MediaTek legacy TF-A: power down an offlined CPU core through the SiP call
 * MTK_SIP_POWER_DOWN_CORE, like the vendor kernel does after PSCI CPU_OFF.
 *
 * The MT6785 vendor kernel registers a CPUHP_BP_PREPARE_DYN teardown that issues SiP 0xc2000212
 * (POWER_DOWN_CORE, core number) for the dead core and SiP 0xc2000210 (POWER_DOWN_CLUSTER) when it
 * was the last core of its cluster. Mainline only uses PSCI CPU_OFF; with the vendor TF-A a later
 * PSCI CPU_ON then hangs the system. With the core call, CPU_ON after CPU_OFF works for small and
 * big cores.
 * The call is on by default on boards that list "motorola,lisbon" and off elsewhere
 * (power_down_core=-1: decide by compatible; 0/1 force it; the parameter stays writable at run
 * time).
 *
 * The cluster call (power_down_cluster, default off) is only made when no other online CPU shares
 * the cluster id; MT6785 DTs describe all eight cores in one cpu-map cluster (DynamIQ), so it is
 * never made there unless cluster_split selects the vendor grouping (cpu0-5 / cpu6-7).
 */
#include <linux/arm-smccc.h>
#include <linux/cpu.h>
#include <linux/cpuhotplug.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/topology.h>

#define MTK_SIP_POWER_DOWN_CLUSTER	0xc2000210
#define MTK_SIP_POWER_DOWN_CORE		0xc2000212

static int power_down_core = -1;
module_param(power_down_core, int, 0644);
MODULE_PARM_DESC(power_down_core, "issue MTK_SIP_POWER_DOWN_CORE after a CPU went offline (-1: on for motorola,lisbon)");

static bool power_down_cluster;
module_param(power_down_cluster, bool, 0644);
MODULE_PARM_DESC(power_down_cluster, "also issue MTK_SIP_POWER_DOWN_CLUSTER for the last core of a cluster");

/*
 * cluster_split: first CPU of the second cluster as the vendor kernel counts
 * it (0 = use the DT topology). The MT6785 vendor kernel treats cpu6/7 (A76)
 * as cluster 1 and cpu0-5 (A55) as cluster 0, while the mainline DT has one
 * cpu-map cluster, so with 0 the cluster call is never made. Writing 6 makes
 * the driver issue POWER_DOWN_CLUSTER with cluster id 1 when cpu6 and cpu7
 * are both offline (only if power_down_cluster is also set).
 */
static unsigned int cluster_split;
module_param(cluster_split, uint, 0644);
MODULE_PARM_DESC(cluster_split, "first CPU of the second cluster for the cluster call (0: DT topology; vendor grouping: 6)");

static int mtk_cpu_sip_cluster(unsigned int cpu)
{
	unsigned int split = READ_ONCE(cluster_split);

	if (split && split < nr_cpu_ids)
		return cpu >= split;
	return topology_cluster_id(cpu);
}

static int mtk_cpu_sip_dead(unsigned int cpu)
{
	struct arm_smccc_res res;
	unsigned int other;
	bool alone = true;

	if (READ_ONCE(power_down_core) <= 0)
		return 0;

	arm_smccc_smc(MTK_SIP_POWER_DOWN_CORE, cpu, 0, 0, 0, 0, 0, 0, &res);
	pr_info("mtk-cpu-sip: cpu%u POWER_DOWN_CORE -> 0x%lx\n", cpu, res.a0);

	if (!READ_ONCE(power_down_cluster))
		return 0;

	for_each_online_cpu(other)
		if (other != cpu && mtk_cpu_sip_cluster(other) == mtk_cpu_sip_cluster(cpu))
			alone = false;
	if (alone) {
		arm_smccc_smc(MTK_SIP_POWER_DOWN_CLUSTER, mtk_cpu_sip_cluster(cpu),
			      0, 0, 0, 0, 0, 0, &res);
		pr_info("mtk-cpu-sip: cluster %d POWER_DOWN_CLUSTER -> 0x%lx\n",
			mtk_cpu_sip_cluster(cpu), res.a0);
	}

	return 0;
}

static int __init mtk_cpu_sip_init(void)
{
	int ret;

	if (!of_machine_is_compatible("mediatek,mt6785"))
		return 0;

	if (power_down_core < 0)
		power_down_core = of_machine_is_compatible("motorola,lisbon");

	ret = cpuhp_setup_state_nocalls(CPUHP_BP_PREPARE_DYN, "soc/mediatek/cpu-sip:dead",
					NULL, mtk_cpu_sip_dead);
	if (ret < 0)
		pr_err("mtk-cpu-sip: cpuhp state not registered: %d\n", ret);
	else
		pr_info("mtk-cpu-sip: ready (power_down_core=%d)\n", power_down_core);
	return 0;
}
device_initcall(mtk_cpu_sip_init);
