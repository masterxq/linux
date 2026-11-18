// SPDX-License-Identifier: GPL-2.0-only
/*
 * MT6785 SPM firmware diagnostics through the vendor TF-A SiP interface.
 *
 * The vendor TF-A loads the SPM PCM firmware from the spmfw partition, but the vendor kernel starts
 * it ("kick") from its SPM driver with the firmware index and the DRAM data rate. Mainline never
 * does, so system suspend through the TF-A and the SPM idle states have no running SPM firmware.
 * This driver only makes that state visible and, for debugging, lets the kick be issued by hand:
 *
 *   /sys/kernel/debug/lisbon-spm/fw_status  -1 not initialised, 0 not loaded,
 *                                            1 loaded, 2 loaded and kicked
 *   /sys/kernel/debug/lisbon-spm/kick       write 1: kick with spmfw_idx and
 *                                            dram_rate (needs allow_kick=1)
 *
 * The SiP function IDs and the argument layout follow the vendor kernel (GPL-2.0); the DRAM data
 * rate default (4266) is the one the vendor kernel uses on this board.
 */

#include <linux/arm-smccc.h>
#include <linux/debugfs.h>
#include <linux/init.h>
#include <linux/kstrtox.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/of.h>

#define MTK_SIP_SPM_FIRMWARE_STATUS	0xc2000221
#define MTK_SIP_SPM_ARGS		0xc200022a
#define SPM_ARGS_SPMFW_IDX_KICK		0

static bool allow_kick;
module_param(allow_kick, bool, 0644);
MODULE_PARM_DESC(allow_kick, "allow the SPM firmware kick through debugfs");

static unsigned int spmfw_idx;
module_param(spmfw_idx, uint, 0644);
MODULE_PARM_DESC(spmfw_idx, "SPM firmware index for the kick");

static unsigned int dram_rate = 4266;
module_param(dram_rate, uint, 0644);
MODULE_PARM_DESC(dram_rate, "DRAM data rate passed with the kick");

static struct dentry *lisbon_spm_dir;

static int fw_status_show(struct seq_file *s, void *unused)
{
	struct arm_smccc_res res;

	arm_smccc_smc(MTK_SIP_SPM_FIRMWARE_STATUS, 0, 0, 0, 0, 0, 0, 0, &res);
	seq_printf(s, "%ld\n", (long)res.a0);

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(fw_status);

static ssize_t kick_write(struct file *file, const char __user *buf,
			  size_t count, loff_t *ppos)
{
	struct arm_smccc_res res;
	bool kick;
	int ret;

	ret = kstrtobool_from_user(buf, count, &kick);
	if (ret)
		return ret;
	if (!kick)
		return count;
	if (!READ_ONCE(allow_kick))
		return -EPERM;

	arm_smccc_smc(MTK_SIP_SPM_ARGS, SPM_ARGS_SPMFW_IDX_KICK, spmfw_idx,
		      dram_rate, 0, 0, 0, 0, &res);
	pr_info("lisbon-spm: firmware kick idx %u rate %u: %ld\n",
		spmfw_idx, dram_rate, (long)res.a0);

	return count;
}

static const struct file_operations kick_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.write = kick_write,
	.llseek = noop_llseek,
};

static int __init lisbon_spm_init(void)
{
	if (!of_machine_is_compatible("mediatek,mt6785"))
		return -ENODEV;

	lisbon_spm_dir = debugfs_create_dir("lisbon-spm", NULL);
	debugfs_create_file("fw_status", 0444, lisbon_spm_dir, NULL,
			    &fw_status_fops);
	debugfs_create_file("kick", 0200, lisbon_spm_dir, NULL, &kick_fops);

	return 0;
}
module_init(lisbon_spm_init);

static void __exit lisbon_spm_exit(void)
{
	debugfs_remove_recursive(lisbon_spm_dir);
}
module_exit(lisbon_spm_exit);

MODULE_DESCRIPTION("MT6785 SPM firmware diagnostics (lisbon)");
MODULE_LICENSE("GPL");
