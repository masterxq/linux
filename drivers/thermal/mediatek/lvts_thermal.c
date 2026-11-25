// SPDX-License-Identifier: GPL-2.0-only
/*
 * Copyright (c) 2023 MediaTek Inc.
 * Author: Balsam CHIHI <bchihi@baylibre.com>
 */

#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/delay.h>
#include <linux/debugfs.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <linux/mutex.h>
#include <linux/nvmem-consumer.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/reset.h>
#include <linux/thermal.h>
#include <dt-bindings/thermal/mediatek,lvts-thermal.h>

#include "../thermal_hwmon.h"

#define LVTS_MONCTL0(__base)	(__base + 0x0000)
#define LVTS_MONCTL1(__base)	(__base + 0x0004)
#define LVTS_MONCTL2(__base)	(__base + 0x0008)
#define LVTS_MONINT(__base)		(__base + 0x000C)
#define LVTS_MONINTSTS(__base)	(__base + 0x0010)
#define LVTS_MONIDET0(__base)	(__base + 0x0014)
#define LVTS_MONIDET1(__base)	(__base + 0x0018)
#define LVTS_MONIDET2(__base)	(__base + 0x001C)
#define LVTS_MONIDET3(__base)	(__base + 0x0020)
#define LVTS_H2NTHRE(__base)	(__base + 0x0024)
#define LVTS_HTHRE(__base)		(__base + 0x0028)
#define LVTS_OFFSETH(__base)	(__base + 0x0030)
#define LVTS_OFFSETL(__base)	(__base + 0x0034)
#define LVTS_MSRCTL0(__base)	(__base + 0x0038)
#define LVTS_MSRCTL1(__base)	(__base + 0x003C)
#define LVTS_TSSEL(__base)		(__base + 0x0040)
#define LVTS_CALSCALE(__base)	(__base + 0x0048)
#define LVTS_ID(__base)			(__base + 0x004C)
#define LVTS_CONFIG(__base)		(__base + 0x0050)
#define LVTS_EDATA00(__base)	(__base + 0x0054)
#define LVTS_EDATA01(__base)	(__base + 0x0058)
#define LVTS_EDATA02(__base)	(__base + 0x005C)
#define LVTS_EDATA03(__base)	(__base + 0x0060)
#define LVTS_MSROFT(__base)		(__base + 0x006C)
#define LVTS_ATP0(__base)		(__base + 0x0070)
#define LVTS_ATP1(__base)		(__base + 0x0074)
#define LVTS_ATP2(__base)		(__base + 0x0078)
#define LVTS_ATP3(__base)		(__base + 0x007C)
#define LVTS_MSR0(__base)		(__base + 0x0090)
#define LVTS_MSR1(__base)		(__base + 0x0094)
#define LVTS_MSR2(__base)		(__base + 0x0098)
#define LVTS_MSR3(__base)		(__base + 0x009C)
#define LVTS_IMMD0(__base)		(__base + 0x00A0)
#define LVTS_IMMD1(__base)		(__base + 0x00A4)
#define LVTS_IMMD2(__base)		(__base + 0x00A8)
#define LVTS_IMMD3(__base)		(__base + 0x00AC)
#define LVTS_PROTCTL(__base)	(__base + 0x00C0)
#define LVTS_PROTTA(__base)		(__base + 0x00C4)
#define LVTS_PROTTB(__base)		(__base + 0x00C8)
#define LVTS_PROTTC(__base)		(__base + 0x00CC)
#define LVTS_CLKEN(__base)		(__base + 0x00E4)

#define LVTS_PERIOD_UNIT			0
#define LVTS_GROUP_INTERVAL			0
#define LVTS_FILTER_INTERVAL		0
#define LVTS_SENSOR_INTERVAL		0
#define LVTS_HW_FILTER				0x0
#define LVTS_TSSEL_CONF				0x13121110
#define LVTS_CALSCALE_CONF			0x300

#define LVTS_MONINT_OFFSET_HIGH_INTEN_SENSOR0		BIT(3)
#define LVTS_MONINT_OFFSET_HIGH_INTEN_SENSOR1		BIT(8)
#define LVTS_MONINT_OFFSET_HIGH_INTEN_SENSOR2		BIT(13)
#define LVTS_MONINT_OFFSET_HIGH_INTEN_SENSOR3		BIT(25)
#define LVTS_MONINT_OFFSET_LOW_INTEN_SENSOR0		BIT(2)
#define LVTS_MONINT_OFFSET_LOW_INTEN_SENSOR1		BIT(7)
#define LVTS_MONINT_OFFSET_LOW_INTEN_SENSOR2		BIT(12)
#define LVTS_MONINT_OFFSET_LOW_INTEN_SENSOR3		BIT(24)

#define LVTS_INT_SENSOR0			0x0009001F
#define LVTS_INT_SENSOR1			0x001203E0
#define LVTS_INT_SENSOR2			0x00247C00
#define LVTS_INT_SENSOR3			0x1FC00000

#define LVTS_SENSOR_MAX				4
#define LVTS_GOLDEN_TEMP_MAX		62
#define LVTS_GOLDEN_TEMP_DEFAULT	50
#define LVTS_COEFF_A_MT8195			-250460
#define LVTS_COEFF_B_MT8195			250460
#define LVTS_COEFF_A_MT7987			-204650
#define LVTS_COEFF_B_MT7987			204650
#define LVTS_COEFF_A_MT7988			-204650
#define LVTS_COEFF_B_MT7988			204650
#define LVTS_COEFF_A_MT8196			391460
#define LVTS_COEFF_B_MT8196			-391460

#define LVTS_MSR_OFFSET_MT8196		-984

#define LVTS_MSR_READ_TIMEOUT_US	400
#define LVTS_MSR_READ_WAIT_US		(LVTS_MSR_READ_TIMEOUT_US / 2)

#define LVTS_MINIMUM_THRESHOLD		20000

#define LVTS_MAX_CAL_OFFSETS		3
#define LVTS_NUM_CAL_OFFSETS_MT7988	3
#define LVTS_NUM_CAL_OFFSETS_MT8196	2

static int golden_temp = LVTS_GOLDEN_TEMP_DEFAULT;
static int golden_temp_offset;

/*
 * Safety gate for platforms whose LVTS support is not yet validated
 * (lvts_data.safety_gate, MT6785). Module parameter "mode":
 *   read  (default) map the block and the efuse, log calibration and controller registers,
 *         write nothing to the thermal controller, register no thermal zone
 *   on    initialise the controllers, check every sensor once (calibration and first reading
 *         plausible), register thermal zones for the plausible ones only, polling only
 *         (no interrupt requested, MONINT stays 0)
 *   full  as "on", but with trip interrupts like on every other SoC
 * In every mode the hardware protection reset (MONINT bit 31, PROTTC) stays off.
 */
enum lvts_gate_mode {
	LVTS_GATE_READ,
	LVTS_GATE_ON,
	LVTS_GATE_FULL,
};

static char *lvts_mode = "read";
module_param_named(mode, lvts_mode, charp, 0444);
MODULE_PARM_DESC(mode, "safety gated SoCs (MT6785): read (default), on (polling only) or full");

#define LVTS_GATE_CAL_MIN	10000		/* count_r, vendor default 19000 */
#define LVTS_GATE_CAL_MAX	30000
#define LVTS_GATE_BOOT_MIN	10000		/* first reading, mC */
#define LVTS_GATE_BOOT_MAX	85000
#define LVTS_GATE_RANGE_MIN	(-25000)	/* any reading outside is ignored */
#define LVTS_GATE_RANGE_MAX	150000
#define LVTS_GATE_SPIKE		30000		/* jump that needs a second reading */
#define LVTS_GATE_SPIKE_CONFIRM	2

enum lvts_msr_mode {
	LVTS_MSR_IMMEDIATE_MODE,
	LVTS_MSR_FILTERED_MODE,
	LVTS_MSR_ATP_MODE,
};

struct lvts_sensor_data {
	int dt_id;
	u8 cal_offsets[LVTS_MAX_CAL_OFFSETS];
};

struct lvts_ctrl_data {
	struct lvts_sensor_data lvts_sensor[LVTS_SENSOR_MAX];
	u8 valid_sensor_mask;
	int offset;
	enum lvts_msr_mode mode;
	/* Optional SoC specific sampling setup, 0 = driver defaults */
	u32 monctl1;
	u32 monctl2;
	u32 msrctl0;
};

#define VALID_SENSOR_MAP(s0, s1, s2, s3) \
	.valid_sensor_mask = (((s0) ? BIT(0) : 0) | \
			      ((s1) ? BIT(1) : 0) | \
			      ((s2) ? BIT(2) : 0) | \
			      ((s3) ? BIT(3) : 0))

#define lvts_for_each_valid_sensor(i, lvts_ctrl) \
	for ((i) = 0; (i) < LVTS_SENSOR_MAX; (i)++) \
		if (!((lvts_ctrl)->valid_sensor_mask & BIT(i))) \
			continue; \
		else

struct lvts_platform_ops {
	int (*lvts_raw_to_temp)(u32 raw_temp, int temp_factor);
	u32 (*lvts_temp_to_raw)(int temperature, int temp_factor);
};

struct lvts_data {
	const struct lvts_ctrl_data *lvts_ctrl;
	const struct lvts_platform_ops *ops;
	const u32 *conn_cmd;
	const u32 *init_cmd;
	int num_cal_offsets;
	int num_lvts_ctrl;
	int num_conn_cmd;
	int num_init_cmd;
	int temp_factor;
	int temp_offset;
	int gt_calib_bit_offset;
	unsigned int def_calibration;
	u16 msr_offset;
	bool write_calscale;
	bool safety_gate;
};

struct lvts_sensor {
	struct thermal_zone_device *tz;
	void __iomem *msr;
	void __iomem *base;
	int id;
	int dt_id;
	int low_thresh;
	int high_thresh;
	bool gate;
	bool have_last;
	int last_temp;
	int spike_cnt;
};

struct lvts_ctrl {
	struct lvts_sensor sensors[LVTS_SENSOR_MAX];
	const struct lvts_data *lvts_data;
	u32 calibration[LVTS_SENSOR_MAX];
	u8 valid_sensor_mask;
	int mode;
	void __iomem *base;
	int low_thresh;
	int high_thresh;
	struct mutex trip_lock;		/* shared thresholds and interrupt registers */
	u32 monctl1;
	u32 monctl2;
	u32 msrctl0;
};

struct lvts_domain {
	struct lvts_ctrl *lvts_ctrl;
	struct reset_control *reset;
	struct clk *clk;
	int num_lvts_ctrl;
	void __iomem *base;
	size_t calib_len;
	u8 *calib;
	enum lvts_gate_mode gate_mode;
	bool read_only;
#ifdef CONFIG_DEBUG_FS
	struct dentry *dom_dentry;
#endif
};

#ifdef CONFIG_MTK_LVTS_THERMAL_DEBUGFS

#define LVTS_DEBUG_FS_REGS(__reg)		\
{						\
	.name = __stringify(__reg),		\
	.offset = __reg(0),			\
}

static const struct debugfs_reg32 lvts_regs[] = {
	LVTS_DEBUG_FS_REGS(LVTS_MONCTL0),
	LVTS_DEBUG_FS_REGS(LVTS_MONCTL1),
	LVTS_DEBUG_FS_REGS(LVTS_MONCTL2),
	LVTS_DEBUG_FS_REGS(LVTS_MONINT),
	LVTS_DEBUG_FS_REGS(LVTS_MONINTSTS),
	LVTS_DEBUG_FS_REGS(LVTS_MONIDET0),
	LVTS_DEBUG_FS_REGS(LVTS_MONIDET1),
	LVTS_DEBUG_FS_REGS(LVTS_MONIDET2),
	LVTS_DEBUG_FS_REGS(LVTS_MONIDET3),
	LVTS_DEBUG_FS_REGS(LVTS_H2NTHRE),
	LVTS_DEBUG_FS_REGS(LVTS_HTHRE),
	LVTS_DEBUG_FS_REGS(LVTS_OFFSETH),
	LVTS_DEBUG_FS_REGS(LVTS_OFFSETL),
	LVTS_DEBUG_FS_REGS(LVTS_MSRCTL0),
	LVTS_DEBUG_FS_REGS(LVTS_MSRCTL1),
	LVTS_DEBUG_FS_REGS(LVTS_TSSEL),
	LVTS_DEBUG_FS_REGS(LVTS_CALSCALE),
	LVTS_DEBUG_FS_REGS(LVTS_ID),
	LVTS_DEBUG_FS_REGS(LVTS_CONFIG),
	LVTS_DEBUG_FS_REGS(LVTS_EDATA00),
	LVTS_DEBUG_FS_REGS(LVTS_EDATA01),
	LVTS_DEBUG_FS_REGS(LVTS_EDATA02),
	LVTS_DEBUG_FS_REGS(LVTS_EDATA03),
	LVTS_DEBUG_FS_REGS(LVTS_MSROFT),
	LVTS_DEBUG_FS_REGS(LVTS_ATP0),
	LVTS_DEBUG_FS_REGS(LVTS_ATP1),
	LVTS_DEBUG_FS_REGS(LVTS_ATP2),
	LVTS_DEBUG_FS_REGS(LVTS_ATP3),
	LVTS_DEBUG_FS_REGS(LVTS_MSR0),
	LVTS_DEBUG_FS_REGS(LVTS_MSR1),
	LVTS_DEBUG_FS_REGS(LVTS_MSR2),
	LVTS_DEBUG_FS_REGS(LVTS_MSR3),
	LVTS_DEBUG_FS_REGS(LVTS_IMMD0),
	LVTS_DEBUG_FS_REGS(LVTS_IMMD1),
	LVTS_DEBUG_FS_REGS(LVTS_IMMD2),
	LVTS_DEBUG_FS_REGS(LVTS_IMMD3),
	LVTS_DEBUG_FS_REGS(LVTS_PROTCTL),
	LVTS_DEBUG_FS_REGS(LVTS_PROTTA),
	LVTS_DEBUG_FS_REGS(LVTS_PROTTB),
	LVTS_DEBUG_FS_REGS(LVTS_PROTTC),
	LVTS_DEBUG_FS_REGS(LVTS_CLKEN),
};

static void lvts_debugfs_exit(void *data)
{
	struct lvts_domain *lvts_td = data;

	debugfs_remove_recursive(lvts_td->dom_dentry);
}

static int lvts_debugfs_init(struct device *dev, struct lvts_domain *lvts_td)
{
	struct debugfs_regset32 *regset;
	struct lvts_ctrl *lvts_ctrl;
	struct dentry *dentry;
	char name[64];
	int i;

	lvts_td->dom_dentry = debugfs_create_dir(dev_name(dev), NULL);
	if (IS_ERR(lvts_td->dom_dentry))
		return 0;

	for (i = 0; i < lvts_td->num_lvts_ctrl; i++) {

		lvts_ctrl = &lvts_td->lvts_ctrl[i];

		sprintf(name, "controller%d", i);
		dentry = debugfs_create_dir(name, lvts_td->dom_dentry);
		if (IS_ERR(dentry))
			continue;

		regset = devm_kzalloc(dev, sizeof(*regset), GFP_KERNEL);
		if (!regset)
			continue;

		regset->base = lvts_ctrl->base;
		regset->regs = lvts_regs;
		regset->nregs = ARRAY_SIZE(lvts_regs);

		debugfs_create_regset32("registers", 0400, dentry, regset);
	}

	return devm_add_action_or_reset(dev, lvts_debugfs_exit, lvts_td);
}

#else

static inline int lvts_debugfs_init(struct device *dev,
				    struct lvts_domain *lvts_td)
{
	return 0;
}

#endif

static int lvts_raw_to_temp(u32 raw_temp, const struct lvts_data *lvts_data)
{
	return lvts_data->ops->lvts_raw_to_temp(raw_temp & 0xFFFF, lvts_data->temp_factor);
}

static u32 lvts_temp_to_raw(int temperature, const struct lvts_data *lvts_data)
{
	return lvts_data->ops->lvts_temp_to_raw(temperature, lvts_data->temp_factor);
}

static int lvts_raw_to_temp_mt7988(u32 raw_temp, int temp_factor)
{
	int temperature;

	temperature = ((s64)(raw_temp & 0xFFFF) * temp_factor) >> 14;
	temperature += golden_temp_offset;

	return temperature;
}

static u32 lvts_temp_to_raw_mt7988(int temperature, int temp_factor)
{
	u32 raw_temp = ((s64)(golden_temp_offset - temperature)) << 14;

	raw_temp = div_s64(raw_temp, -temp_factor);

	return raw_temp;
}

static u32 lvts_temp_to_raw_mt8196(int temperature, int temp_factor)
{
	u32 raw_temp;

	raw_temp = temperature - golden_temp_offset;

	return div_s64((s64)temp_factor << 14, raw_temp);
}

static int lvts_get_temp(struct thermal_zone_device *tz, int *temp)
{
	struct lvts_sensor *lvts_sensor = thermal_zone_device_priv(tz);
	struct lvts_ctrl *lvts_ctrl = container_of(lvts_sensor, struct lvts_ctrl,
						   sensors[lvts_sensor->id]);
	const struct lvts_data *lvts_data = lvts_ctrl->lvts_data;
	void __iomem *msr = lvts_sensor->msr;
	u32 value;
	int rc;

	/*
	 * Measurement registers:
	 *
	 * LVTS_MSR[0-3] / LVTS_IMMD[0-3]
	 *
	 * Bits:
	 *
	 * 32-17: Unused
	 * 16	: Valid temperature
	 * 15-0	: Raw temperature
	 */
	rc = readl_poll_timeout(msr, value, value & BIT(16),
				LVTS_MSR_READ_WAIT_US, LVTS_MSR_READ_TIMEOUT_US);

	/*
	 * As the thermal zone temperature will read before the
	 * hardware sensor is fully initialized, we have to check the
	 * validity of the temperature returned when reading the
	 * measurement register. The thermal controller will set the
	 * valid bit temperature only when it is totally initialized.
	 *
	 * Otherwise, we may end up with garbage values out of the
	 * functionning temperature and directly jump to a system
	 * shutdown.
	 */
	if (rc)
		return -EAGAIN;

	*temp = lvts_raw_to_temp(value, lvts_data);

	if (!lvts_sensor->gate)
		return 0;

	/*
	 * Safety gate: a reading outside of what the die can reach is a read error, not a
	 * temperature (raw 0 would be about 230 C and end in a critical shutdown). A jump of
	 * more than LVTS_GATE_SPIKE against the last accepted value has to be confirmed by
	 * further readings before it is reported.
	 */
	if (*temp < LVTS_GATE_RANGE_MIN || *temp > LVTS_GATE_RANGE_MAX) {
		pr_warn_ratelimited("%s: lvts gate: reading %d mC (raw %#x) ignored\n",
				    thermal_zone_device_type(tz), *temp, value);
		return -EAGAIN;
	}

	if (lvts_sensor->have_last && abs(*temp - lvts_sensor->last_temp) > LVTS_GATE_SPIKE &&
	    lvts_sensor->spike_cnt < LVTS_GATE_SPIKE_CONFIRM) {
		lvts_sensor->spike_cnt++;
		pr_warn_ratelimited("%s: lvts gate: jump %d -> %d mC held back\n",
				    thermal_zone_device_type(tz), lvts_sensor->last_temp, *temp);
		*temp = lvts_sensor->last_temp;
		return 0;
	}

	lvts_sensor->spike_cnt = 0;
	lvts_sensor->last_temp = *temp;
	lvts_sensor->have_last = true;

	return 0;
}

static void lvts_update_irq_mask(struct lvts_ctrl *lvts_ctrl)
{
	static const u32 high_offset_inten_masks[] = {
		LVTS_MONINT_OFFSET_HIGH_INTEN_SENSOR0,
		LVTS_MONINT_OFFSET_HIGH_INTEN_SENSOR1,
		LVTS_MONINT_OFFSET_HIGH_INTEN_SENSOR2,
		LVTS_MONINT_OFFSET_HIGH_INTEN_SENSOR3,
	};
	static const u32 low_offset_inten_masks[] = {
		LVTS_MONINT_OFFSET_LOW_INTEN_SENSOR0,
		LVTS_MONINT_OFFSET_LOW_INTEN_SENSOR1,
		LVTS_MONINT_OFFSET_LOW_INTEN_SENSOR2,
		LVTS_MONINT_OFFSET_LOW_INTEN_SENSOR3,
	};
	u32 value = 0;
	int i;

	value = readl(LVTS_MONINT(lvts_ctrl->base));

	lvts_for_each_valid_sensor(i, lvts_ctrl) {
		if (lvts_ctrl->sensors[i].high_thresh == lvts_ctrl->high_thresh
		    && lvts_ctrl->sensors[i].low_thresh == lvts_ctrl->low_thresh) {
			/*
			 * The minimum threshold needs to be configured in the
			 * OFFSETL register to get working interrupts, but we
			 * don't actually want to generate interrupts when
			 * crossing it.
			 */
			value &= ~(low_offset_inten_masks[i] | high_offset_inten_masks[i]);
			if (lvts_ctrl->high_thresh != INT_MAX)
				value |= high_offset_inten_masks[i];
			if (lvts_ctrl->low_thresh != -INT_MAX)
				value |= low_offset_inten_masks[i];
		} else {
			value &= ~(low_offset_inten_masks[i] | high_offset_inten_masks[i]);
		}
	}

	writel(value, LVTS_MONINT(lvts_ctrl->base));
}

static bool lvts_should_update_thresh(struct lvts_ctrl *lvts_ctrl, int high)
{
	int i;

	if (high != INT_MAX &&
	    (lvts_ctrl->high_thresh == INT_MAX || high > lvts_ctrl->high_thresh))
		return true;

	lvts_for_each_valid_sensor(i, lvts_ctrl)
		if (lvts_ctrl->sensors[i].high_thresh == lvts_ctrl->high_thresh
		    && lvts_ctrl->sensors[i].low_thresh == lvts_ctrl->low_thresh)
			return false;

	return true;
}

static int lvts_set_trips(struct thermal_zone_device *tz, int low, int high)
{
	struct lvts_sensor *lvts_sensor = thermal_zone_device_priv(tz);
	struct lvts_ctrl *lvts_ctrl = container_of(lvts_sensor, struct lvts_ctrl,
						   sensors[lvts_sensor->id]);
	const struct lvts_data *lvts_data = lvts_ctrl->lvts_data;
	void __iomem *base = lvts_sensor->base;
	u32 raw_low = lvts_temp_to_raw(low != -INT_MAX ? low : LVTS_MINIMUM_THRESHOLD,
				       lvts_data);
	bool should_update_thresh;

	mutex_lock(&lvts_ctrl->trip_lock);
	lvts_sensor->low_thresh = low;
	lvts_sensor->high_thresh = high;

	should_update_thresh = lvts_should_update_thresh(lvts_ctrl, high);
	if (should_update_thresh) {
		lvts_ctrl->high_thresh = high;
		lvts_ctrl->low_thresh = low;
	}
	if (!should_update_thresh)
		goto update_mask;

	/*
	 * Low offset temperature threshold
	 *
	 * LVTS_OFFSETL
	 *
	 * Bits:
	 *
	 * 14-0 : Raw temperature for threshold
	 */
	pr_debug("%s: Setting low limit temperature interrupt: %d\n",
		 thermal_zone_device_type(tz), low);
	writel(raw_low, LVTS_OFFSETL(base));

	/*
	 * High offset temperature threshold
	 *
	 * LVTS_OFFSETH
	 *
	 * Bits:
	 *
	 * 14-0 : Raw temperature for threshold
	 */
	if (high != INT_MAX) {
		pr_debug("%s: Setting high limit temperature interrupt: %d\n",
			 thermal_zone_device_type(tz), high);
		writel(lvts_temp_to_raw(high, lvts_data), LVTS_OFFSETH(base));
	}

update_mask:
	lvts_update_irq_mask(lvts_ctrl);
	mutex_unlock(&lvts_ctrl->trip_lock);
	return 0;
}

static irqreturn_t lvts_ctrl_irq_handler(struct lvts_ctrl *lvts_ctrl)
{
	u32 value, enabled;
	static const u32 masks[] = {
		LVTS_INT_SENSOR0,
		LVTS_INT_SENSOR1,
		LVTS_INT_SENSOR2,
		LVTS_INT_SENSOR3
	};
	int i;

	/*
	 * Interrupt monitoring status
	 *
	 * LVTS_MONINTST
	 *
	 * Bits:
	 *
	 * 31 : Interrupt for stage 3
	 * 30 : Interrupt for stage 2
	 * 29 : Interrupt for state 1
	 * 28 : Interrupt using filter on sensor 3
	 *
	 * 27 : Interrupt using immediate on sensor 3
	 * 26 : Interrupt normal to hot on sensor 3
	 * 25 : Interrupt high offset on sensor 3
	 * 24 : Interrupt low offset on sensor 3
	 *
	 * 23 : Interrupt hot threshold on sensor 3
	 * 22 : Interrupt cold threshold on sensor 3
	 * 21 : Interrupt using filter on sensor 2
	 * 20 : Interrupt using filter on sensor 1
	 *
	 * 19 : Interrupt using filter on sensor 0
	 * 18 : Interrupt using immediate on sensor 2
	 * 17 : Interrupt using immediate on sensor 1
	 * 16 : Interrupt using immediate on sensor 0
	 *
	 * 15 : Interrupt device access timeout interrupt
	 * 14 : Interrupt normal to hot on sensor 2
	 * 13 : Interrupt high offset interrupt on sensor 2
	 * 12 : Interrupt low offset interrupt on sensor 2
	 *
	 * 11 : Interrupt hot threshold on sensor 2
	 * 10 : Interrupt cold threshold on sensor 2
	 *  9 : Interrupt normal to hot on sensor 1
	 *  8 : Interrupt high offset interrupt on sensor 1
	 *
	 *  7 : Interrupt low offset interrupt on sensor 1
	 *  6 : Interrupt hot threshold on sensor 1
	 *  5 : Interrupt cold threshold on sensor 1
	 *  4 : Interrupt normal to hot on sensor 0
	 *
	 *  3 : Interrupt high offset interrupt on sensor 0
	 *  2 : Interrupt low offset interrupt on sensor 0
	 *  1 : Interrupt hot threshold on sensor 0
	 *  0 : Interrupt cold threshold on sensor 0
	 *
	 * We are interested in the sensor(s) responsible of the
	 * interrupt event. We update the thermal framework with the
	 * thermal zone associated with the sensor. The framework will
	 * take care of the rest whatever the kind of interrupt, we
	 * are only interested in which sensor raised the interrupt.
	 *
	 * sensor 3 interrupt: 0001 1111 1100 0000 0000 0000 0000 0000
	 *                  => 0x1FC00000
	 * sensor 2 interrupt: 0000 0000 0010 0100 0111 1100 0000 0000
	 *                  => 0x00247C00
	 * sensor 1 interrupt: 0000 0000 0001 0010 0000 0011 1110 0000
	 *                  => 0X001203E0
	 * sensor 0 interrupt: 0000 0000 0000 1001 0000 0000 0001 1111
	 *                  => 0x0009001F
	 */
	mutex_lock(&lvts_ctrl->trip_lock);
	enabled = readl(LVTS_MONINT(lvts_ctrl->base));
	value = readl(LVTS_MONINTSTS(lvts_ctrl->base));
	/* Acknowledge before the thermal core can program a new window. */
	if (value)
		writel(value, LVTS_MONINTSTS(lvts_ctrl->base));
	mutex_unlock(&lvts_ctrl->trip_lock);
	value &= enabled;
	if (!value)
		return IRQ_NONE;

	/*
	 * Let's figure out which sensors raised the interrupt
	 *
	 * NOTE: the masks array must be ordered with the index
	 * corresponding to the sensor id eg. index=0, mask for
	 * sensor0.
	 */
	for (i = 0; i < ARRAY_SIZE(masks); i++) {
		struct thermal_zone_device *tz = READ_ONCE(lvts_ctrl->sensors[i].tz);

		if (tz && (value & masks[i]))
			thermal_zone_device_update(tz, THERMAL_TRIP_VIOLATED);
	}

	return IRQ_HANDLED;
}

/*
 * Temperature interrupt handler. Even if the driver supports more
 * interrupt modes, we use the interrupt when the temperature crosses
 * the hot threshold the way up and the way down (modulo the
 * hysteresis).
 *
 * Each thermal domain has a couple of interrupts, one for hardware
 * reset and another one for all the thermal events happening on the
 * different sensors.
 *
 * The interrupt is configured for thermal events when crossing the
 * hot temperature limit. At each interrupt, we check in every
 * controller if there is an interrupt pending.
 */
static irqreturn_t lvts_irq_handler(int irq, void *data)
{
	struct lvts_domain *lvts_td = data;
	irqreturn_t aux, iret = IRQ_NONE;
	int i;

	for (i = 0; i < lvts_td->num_lvts_ctrl; i++) {

		aux = lvts_ctrl_irq_handler(&lvts_td->lvts_ctrl[i]);
		if (aux != IRQ_HANDLED)
			continue;

		iret = IRQ_HANDLED;
	}

	return iret;
}

static const struct thermal_zone_device_ops lvts_ops = {
	.get_temp = lvts_get_temp,
	.set_trips = lvts_set_trips,
};

/* Safety gate "on": no set_trips, the thermal core polls (polling-delay from DT) */
static const struct thermal_zone_device_ops lvts_ops_poll = {
	.get_temp = lvts_get_temp,
};

static int lvts_sensor_init(struct device *dev, struct lvts_ctrl *lvts_ctrl,
					const struct lvts_ctrl_data *lvts_ctrl_data)
{
	struct lvts_sensor *lvts_sensor = lvts_ctrl->sensors;

	void __iomem *msr_regs[] = {
		LVTS_MSR0(lvts_ctrl->base),
		LVTS_MSR1(lvts_ctrl->base),
		LVTS_MSR2(lvts_ctrl->base),
		LVTS_MSR3(lvts_ctrl->base)
	};

	void __iomem *imm_regs[] = {
		LVTS_IMMD0(lvts_ctrl->base),
		LVTS_IMMD1(lvts_ctrl->base),
		LVTS_IMMD2(lvts_ctrl->base),
		LVTS_IMMD3(lvts_ctrl->base)
	};

	void __iomem *atp_regs[] = {
		LVTS_ATP0(lvts_ctrl->base),
		LVTS_ATP1(lvts_ctrl->base),
		LVTS_ATP2(lvts_ctrl->base),
		LVTS_ATP3(lvts_ctrl->base)
	};

	int i;

	lvts_for_each_valid_sensor(i, lvts_ctrl_data) {

		int dt_id = lvts_ctrl_data->lvts_sensor[i].dt_id;

		/*
		 * At this point, we don't know which id matches which
		 * sensor. Let's set arbitrally the id from the index.
		 */
		lvts_sensor[i].id = i;

		/*
		 * The thermal zone registration will set the trip
		 * point interrupt in the thermal controller
		 * register. But this one will be reset in the
		 * initialization after. So we need to post pone the
		 * thermal zone creation after the controller is
		 * setup. For this reason, we store the device tree
		 * node id from the data in the sensor structure
		 */
		lvts_sensor[i].dt_id = dt_id;

		/*
		 * We assign the base address of the thermal
		 * controller as a back pointer. So it will be
		 * accessible from the different thermal framework ops
		 * as we pass the lvts_sensor pointer as thermal zone
		 * private data.
		 */
		lvts_sensor[i].base = lvts_ctrl->base;

		/*
		 * Each sensor has its own register address to read from.
		 */
		switch (lvts_ctrl_data->mode) {
		case LVTS_MSR_IMMEDIATE_MODE:
			lvts_sensor[i].msr = imm_regs[i];
			break;
		case LVTS_MSR_FILTERED_MODE:
			lvts_sensor[i].msr = msr_regs[i];
			break;
		case LVTS_MSR_ATP_MODE:
			lvts_sensor[i].msr = atp_regs[i];
			break;
		default:
			lvts_sensor[i].msr = imm_regs[i];
			break;
		}

		lvts_sensor[i].low_thresh = INT_MIN;
		lvts_sensor[i].high_thresh = INT_MIN;
		lvts_sensor[i].gate = lvts_ctrl->lvts_data->safety_gate;
	}

	lvts_ctrl->valid_sensor_mask = lvts_ctrl_data->valid_sensor_mask;

	return 0;
}

static int lvts_decode_sensor_calibration(const struct lvts_sensor_data *sensor,
					const u8 *efuse_calibration, u32 calib_len,
					u8 num_offsets, u32 *calib)
{
	int i;
	u32 calib_val = 0;

	for (i = 0; i < num_offsets; i++) {
		u8 offset = sensor->cal_offsets[i];

		if (offset >= calib_len)
			return -EINVAL;
		// Pack each calibration byte into the correct position
		calib_val |= efuse_calibration[offset] << (8 * i);
	}

	*calib = calib_val;
	return 0;
}

/*
 * The efuse blob values follows the sensor enumeration per thermal
 * controller. The decoding of the stream is as follow:
 *
 * MT8192 :
 * Stream index map for MCU Domain mt8192 :
 *
 * <-----mcu-tc#0-----> <-----sensor#0----->        <-----sensor#1----->
 *  0x01 | 0x02 | 0x03 | 0x04 | 0x05 | 0x06 | 0x07 | 0x08 | 0x09 | 0x0A | 0x0B
 *
 * <-----sensor#2----->        <-----sensor#3----->
 *  0x0C | 0x0D | 0x0E | 0x0F | 0x10 | 0x11 | 0x12 | 0x13
 *
 * <-----sensor#4----->        <-----sensor#5----->        <-----sensor#6----->        <-----sensor#7----->
 *  0x14 | 0x15 | 0x16 | 0x17 | 0x18 | 0x19 | 0x1A | 0x1B | 0x1C | 0x1D | 0x1E | 0x1F | 0x20 | 0x21 | 0x22 | 0x23
 *
 * Stream index map for AP Domain mt8192 :
 *
 * <-----sensor#0----->        <-----sensor#1----->
 *  0x24 | 0x25 | 0x26 | 0x27 | 0x28 | 0x29 | 0x2A | 0x2B
 *
 * <-----sensor#2----->        <-----sensor#3----->
 *  0x2C | 0x2D | 0x2E | 0x2F | 0x30 | 0x31 | 0x32 | 0x33
 *
 * <-----sensor#4----->        <-----sensor#5----->
 *  0x34 | 0x35 | 0x36 | 0x37 | 0x38 | 0x39 | 0x3A | 0x3B
 *
 * <-----sensor#6----->        <-----sensor#7----->        <-----sensor#8----->
 *  0x3C | 0x3D | 0x3E | 0x3F | 0x40 | 0x41 | 0x42 | 0x43 | 0x44 | 0x45 | 0x46 | 0x47
 *
 * MT8195 :
 * Stream index map for MCU Domain mt8195 :
 *
 * <-----mcu-tc#0-----> <-----sensor#0-----> <-----sensor#1----->
 *  0x01 | 0x02 | 0x03 | 0x04 | 0x05 | 0x06 | 0x07 | 0x08 | 0x09
 *
 * <-----mcu-tc#1-----> <-----sensor#2-----> <-----sensor#3----->
 *  0x0A | 0x0B | 0x0C | 0x0D | 0x0E | 0x0F | 0x10 | 0x11 | 0x12
 *
 * <-----mcu-tc#2-----> <-----sensor#4-----> <-----sensor#5-----> <-----sensor#6-----> <-----sensor#7----->
 *  0x13 | 0x14 | 0x15 | 0x16 | 0x17 | 0x18 | 0x19 | 0x1A | 0x1B | 0x1C | 0x1D | 0x1E | 0x1F | 0x20 | 0x21
 *
 * Stream index map for AP Domain mt8195 :
 *
 * <-----ap--tc#0-----> <-----sensor#0-----> <-----sensor#1----->
 *  0x22 | 0x23 | 0x24 | 0x25 | 0x26 | 0x27 | 0x28 | 0x29 | 0x2A
 *
 * <-----ap--tc#1-----> <-----sensor#2-----> <-----sensor#3----->
 *  0x2B | 0x2C | 0x2D | 0x2E | 0x2F | 0x30 | 0x31 | 0x32 | 0x33
 *
 * <-----ap--tc#2-----> <-----sensor#4-----> <-----sensor#5-----> <-----sensor#6----->
 *  0x34 | 0x35 | 0x36 | 0x37 | 0x38 | 0x39 | 0x3A | 0x3B | 0x3C | 0x3D | 0x3E | 0x3F
 *
 * <-----ap--tc#3-----> <-----sensor#7-----> <-----sensor#8----->
 *  0x40 | 0x41 | 0x42 | 0x43 | 0x44 | 0x45 | 0x46 | 0x47 | 0x48
 *
 * MT8196 :
 * Stream index map for MCU Domain mt8196 :
 *
 * <-sensor#1--> <-sensor#0--> <-sensor#3--> <-sensor#2-->
 *  0x04 | 0x05 | 0x06 | 0x07 | 0x08 | 0x09 | 0x0A | 0x0B
 *
 * <-sensor#5--> <-sensor#4--> <-sensor#7--> <-sensor#6-->
 *  0x0C | 0x0D | 0x0E | 0x0F | 0x10 | 0x11 | 0x12 | 0x13
 *
 * <-sensor#9--> <-sensor#8--> <-sensor#11-> <-sensor#10->
 *  0x14 | 0x15 | 0x16 | 0x17 | 0x18 | 0x19 | 0x1A | 0X1B
 *
 * <-sensor#13-> <-sensor#12-> <-sensor#15-> <-sensor#14->
 *  0x1C | 0x1D | 0x1E | 0x1F | 0x20 | 0x21 | 0x22 | 0x23
 *
 * Stream index map for APU Domain mt8196 :
 *
 * <-sensor#1--> <-sensor#0--> <-sensor#3--> <-sensor#2-->
 *  0x24 | 0x25 | 0x26 | 0x27 | 0x28 | 0x29 | 0x2A | 0x2B
 *
 * Stream index map for GPU Domain mt8196 :
 *
 * <-sensor#1--> <-sensor#0-->
 *  0x2C | 0x2D | 0x2E | 0x2F
 *
 * Stream index map for AP Domain mt8196 :
 *
 * <-sensor#1--> <-sensor#0--> <-sensor#3--> <-sensor#2-->
 *  0x30 | 0x31 | 0x32 | 0x33 | 0x34 | 0x35 | 0x36 | 0x37
 *
 * <-sensor#5--> <-sensor#4--> <-sensor#6--> <-sensor#7-->
 *  0x38 | 0x39 | 0x3A | 0x3B | 0x3C | 0x3D | 0x3E | 0x3F
 *
 * Note: In some cases, values don't strictly follow a little endian ordering.
 * The data description gives byte offsets constituting each calibration value
 * for each sensor.
 */
static int lvts_calibration_init(struct device *dev, struct lvts_ctrl *lvts_ctrl,
					const struct lvts_ctrl_data *lvts_ctrl_data,
					u8 *efuse_calibration,
					size_t calib_len)
{
	const struct lvts_data *lvts_data = lvts_ctrl->lvts_data;
	int i, ret;
	u32 gt;

	/* A zero value for gt means that device has invalid efuse data */
	gt = (((u32 *)efuse_calibration)[0] >> lvts_data->gt_calib_bit_offset) & 0xff;

	lvts_for_each_valid_sensor(i, lvts_ctrl_data) {
		const struct lvts_sensor_data *sensor =
					&lvts_ctrl_data->lvts_sensor[i];
		u32 calib = 0;

		ret = lvts_decode_sensor_calibration(sensor, efuse_calibration,
						     calib_len,
						     lvts_data->num_cal_offsets,
						     &calib);
		if (ret)
			return ret;

		if (lvts_data->safety_gate &&
		    (calib < LVTS_GATE_CAL_MIN || calib > LVTS_GATE_CAL_MAX)) {
			dev_warn(dev, "lvts gate: sensor dt %d calibration %u implausible, dropped\n",
				 sensor->dt_id, calib);
			lvts_ctrl->valid_sensor_mask &= ~BIT(i);
			lvts_ctrl->calibration[i] = 0;
			continue;
		}

		if (lvts_data->safety_gate)
			dev_info(dev, "lvts gate: sensor dt %d calibration %u\n",
				 sensor->dt_id, calib);

		if (gt) {
			lvts_ctrl->calibration[i] = calib;
			if (lvts_ctrl->lvts_data->msr_offset)
				lvts_ctrl->calibration[i] += lvts_ctrl->lvts_data->msr_offset;
		} else if (lvts_ctrl->lvts_data->def_calibration) {
			lvts_ctrl->calibration[i] = lvts_ctrl->lvts_data->def_calibration;
		} else {
			dev_err(dev, "efuse contains invalid calibration data and no default given.\n");
			return -ENODATA;
		}
	}

	return 0;
}

/*
 * The efuse bytes stream can be split into different chunk of
 * nvmems. This function reads and concatenate those into a single
 * buffer so it can be read sequentially when initializing the
 * calibration data.
 */
static int lvts_calibration_read(struct device *dev, struct lvts_domain *lvts_td,
					const struct lvts_data *lvts_data)
{
	struct device_node *np = dev_of_node(dev);
	struct nvmem_cell *cell;
	struct property *prop;
	const char *cell_name;

	of_property_for_each_string(np, "nvmem-cell-names", prop, cell_name) {
		size_t len;
		u8 *efuse;

		cell = of_nvmem_cell_get(np, cell_name);
		if (IS_ERR(cell)) {
			dev_err(dev, "Failed to get cell '%s'\n", cell_name);
			return PTR_ERR(cell);
		}

		efuse = nvmem_cell_read(cell, &len);

		nvmem_cell_put(cell);

		if (IS_ERR(efuse)) {
			dev_err(dev, "Failed to read cell '%s'\n", cell_name);
			return PTR_ERR(efuse);
		}

		lvts_td->calib = devm_krealloc(dev, lvts_td->calib,
					       lvts_td->calib_len + len, GFP_KERNEL);
		if (!lvts_td->calib) {
			kfree(efuse);
			return -ENOMEM;
		}

		memcpy(lvts_td->calib + lvts_td->calib_len, efuse, len);

		lvts_td->calib_len += len;

		kfree(efuse);
	}

	return 0;
}

static int lvts_golden_temp_init(struct device *dev, u8 *calib,
				 const struct lvts_data *lvts_data)
{
	u32 gt;

	/*
	 * The golden temp information is contained in the first 32-bit
	 * word  of efuse data at a specific bit offset.
	 */
	gt = (((u32 *)calib)[0] >> lvts_data->gt_calib_bit_offset) & 0xff;

	/* Safety gate: no fake golden temperature, no sensor at all */
	if (lvts_data->safety_gate && (!gt || gt >= LVTS_GOLDEN_TEMP_MAX)) {
		dev_err(dev, "lvts gate: golden temperature %u implausible, no sensor used\n", gt);
		return -ENODATA;
	}

	/* A zero value for gt means that device has invalid efuse data */
	if (gt && gt < LVTS_GOLDEN_TEMP_MAX)
		golden_temp = gt;

	golden_temp_offset = golden_temp * 500 + lvts_data->temp_offset;

	dev_info(dev, "%sgolden temp=%d\n", gt ? "" : "fake ", golden_temp);

	return 0;
}

static int lvts_ctrl_init(struct device *dev, struct lvts_domain *lvts_td,
					const struct lvts_data *lvts_data)
{
	size_t size = sizeof(*lvts_td->lvts_ctrl) * lvts_data->num_lvts_ctrl;
	struct lvts_ctrl *lvts_ctrl;
	int i, ret;

	/*
	 * Create the calibration bytes stream from efuse data
	 */
	ret = lvts_calibration_read(dev, lvts_td, lvts_data);
	if (ret)
		return ret;

	ret = lvts_golden_temp_init(dev, lvts_td->calib, lvts_data);
	if (ret)
		return ret;

	lvts_ctrl = devm_kzalloc(dev, size, GFP_KERNEL);
	if (!lvts_ctrl)
		return -ENOMEM;

	for (i = 0; i < lvts_data->num_lvts_ctrl; i++) {

		lvts_ctrl[i].base = lvts_td->base + lvts_data->lvts_ctrl[i].offset;
		lvts_ctrl[i].lvts_data = lvts_data;
		mutex_init(&lvts_ctrl[i].trip_lock);

		ret = lvts_sensor_init(dev, &lvts_ctrl[i],
				       &lvts_data->lvts_ctrl[i]);
		if (ret)
			return ret;

		ret = lvts_calibration_init(dev, &lvts_ctrl[i],
					    &lvts_data->lvts_ctrl[i],
					    lvts_td->calib,
					    lvts_td->calib_len);
		if (ret)
			return ret;

		/*
		 * The mode the ctrl will use to read the temperature
		 * (filtered or immediate)
		 */
		lvts_ctrl[i].mode = lvts_data->lvts_ctrl[i].mode;

		lvts_ctrl[i].monctl1 = lvts_data->lvts_ctrl[i].monctl1;
		lvts_ctrl[i].monctl2 = lvts_data->lvts_ctrl[i].monctl2;
		lvts_ctrl[i].msrctl0 = lvts_data->lvts_ctrl[i].msrctl0;

		lvts_ctrl[i].low_thresh = INT_MIN;
		lvts_ctrl[i].high_thresh = INT_MIN;
	}

	/*
	 * We no longer need the efuse bytes stream, let's free it
	 */
	devm_kfree(dev, lvts_td->calib);

	lvts_td->lvts_ctrl = lvts_ctrl;
	lvts_td->num_lvts_ctrl = lvts_data->num_lvts_ctrl;

	return 0;
}

static void lvts_ctrl_monitor_enable(struct device *dev, struct lvts_ctrl *lvts_ctrl, bool enable)
{
	/*
	 * Bitmaps to enable each sensor on filtered mode in the MONCTL0
	 * register.
	 */
	static const u8 sensor_filt_bitmap[] = { BIT(0), BIT(1), BIT(2), BIT(3) };
	u32 sensor_map = 0;
	int i;

	if (lvts_ctrl->mode == LVTS_MSR_IMMEDIATE_MODE)
		return;

	if (enable) {
		lvts_for_each_valid_sensor(i, lvts_ctrl)
			sensor_map |= sensor_filt_bitmap[i];
	}

	/*
	 * Bits:
	 *      9: Single point access flow
	 *    0-3: Enable sensing point 0-3
	 */
	writel(sensor_map | BIT(9), LVTS_MONCTL0(lvts_ctrl->base));
}

/*
 * At this point the configuration register is the only place in the
 * driver where we write multiple values. Per hardware constraint,
 * each write in the configuration register must be separated by a
 * delay of 2 us.
 */
static void lvts_write_config(struct lvts_ctrl *lvts_ctrl, const u32 *cmds, int nr_cmds)
{
	int i;

	/*
	 * Configuration register
	 */
	for (i = 0; i < nr_cmds; i++) {
		writel(cmds[i], LVTS_CONFIG(lvts_ctrl->base));
		usleep_range(2, 4);
	}
}

static int lvts_irq_init(struct lvts_ctrl *lvts_ctrl)
{
	/*
	 * LVTS_PROTCTL : Thermal Protection Sensor Selection
	 *
	 * Bits:
	 *
	 * 19-18 : Sensor to base the protection on
	 * 17-16 : Strategy:
	 *         00 : Average of 4 sensors
	 *         01 : Max of 4 sensors
	 *         10 : Selected sensor with bits 19-18
	 *         11 : Reserved
	 */

	/*
	 * LVTS_PROTTA : Stage 1 temperature threshold
	 * LVTS_PROTTB : Stage 2 temperature threshold
	 * LVTS_PROTTC : Stage 3 temperature threshold
	 *
	 * Bits:
	 *
	 * 14-0: Raw temperature threshold
	 *
	 * writel(0x0, LVTS_PROTTA(lvts_ctrl->base));
	 * writel(0x0, LVTS_PROTTB(lvts_ctrl->base));
	 * writel(0x0, LVTS_PROTTC(lvts_ctrl->base));
	 */

	/*
	 * LVTS_MONINT : Interrupt configuration register
	 *
	 * The LVTS_MONINT register layout is the same as the LVTS_MONINTSTS
	 * register, except we set the bits to enable the interrupt.
	 */
	writel(0, LVTS_MONINT(lvts_ctrl->base));

	return 0;
}

static int lvts_domain_reset(struct device *dev, struct reset_control *reset)
{
	int ret;

	ret = reset_control_assert(reset);
	if (ret)
		return ret;

	return reset_control_deassert(reset);
}

/*
 * Enable or disable the clocks of a specified thermal controller
 */
static int lvts_ctrl_set_enable(struct lvts_ctrl *lvts_ctrl, int enable)
{
	/*
	 * LVTS_CLKEN : Internal LVTS clock
	 *
	 * Bits:
	 *
	 * 0 : enable / disable clock
	 */
	writel(enable, LVTS_CLKEN(lvts_ctrl->base));

	return 0;
}

static int lvts_ctrl_connect(struct device *dev, struct lvts_ctrl *lvts_ctrl)
{
	const struct lvts_data *lvts_data = lvts_ctrl->lvts_data;
	u32 id;

	lvts_write_config(lvts_ctrl, lvts_data->conn_cmd, lvts_data->num_conn_cmd);

	/*
	 * LVTS_ID : Get ID and status of the thermal controller
	 *
	 * Bits:
	 *
	 * 0-5	: thermal controller id
	 *   7	: thermal controller connection is valid
	 */
	id = readl(LVTS_ID(lvts_ctrl->base));
	if (!(id & BIT(7)))
		return -EIO;

	return 0;
}

static int lvts_ctrl_initialize(struct device *dev, struct lvts_ctrl *lvts_ctrl)
{
	const struct lvts_data *lvts_data = lvts_ctrl->lvts_data;

	lvts_write_config(lvts_ctrl, lvts_data->init_cmd, lvts_data->num_init_cmd);

	return 0;
}

static int lvts_ctrl_calibrate(struct device *dev, struct lvts_ctrl *lvts_ctrl)
{
	int i;
	void __iomem *lvts_edata[] = {
		LVTS_EDATA00(lvts_ctrl->base),
		LVTS_EDATA01(lvts_ctrl->base),
		LVTS_EDATA02(lvts_ctrl->base),
		LVTS_EDATA03(lvts_ctrl->base)
	};

	/*
	 * LVTS_EDATA0X : Efuse calibration reference value for sensor X
	 *
	 * Bits:
	 *
	 * 20-0 : Efuse value for normalization data
	 */
	for (i = 0; i < LVTS_SENSOR_MAX; i++)
		writel(lvts_ctrl->calibration[i], lvts_edata[i]);

	/* LVTS_MSROFT : Constant offset applied to MSR values
	 * for post-processing
	 *
	 * Bits:
	 *
	 * 20-0 : Constant data added to MSR values
	 */
	if (lvts_ctrl->lvts_data->msr_offset)
		writel(lvts_ctrl->lvts_data->msr_offset,
		       LVTS_MSROFT(lvts_ctrl->base));

	return 0;
}

static int lvts_ctrl_configure(struct device *dev, struct lvts_ctrl *lvts_ctrl)
{
	u32 value;

	/*
	 * LVTS_TSSEL : Sensing point index numbering
	 *
	 * Bits:
	 *
	 * 31-24: ADC Sense 3
	 * 23-16: ADC Sense 2
	 * 15-8	: ADC Sense 1
	 * 7-0	: ADC Sense 0
	 */
	value = LVTS_TSSEL_CONF;
	writel(value, LVTS_TSSEL(lvts_ctrl->base));

	/*
	 * LVTS_CALSCALE : ADC voltage round
	 */
	value = 0x300;
	value = LVTS_CALSCALE_CONF;
	if (lvts_ctrl->lvts_data->write_calscale)
		writel(value, LVTS_CALSCALE(lvts_ctrl->base));

	/*
	 * LVTS_MSRCTL0 : Sensor filtering strategy
	 *
	 * Filters:
	 *
	 * 000 : One sample
	 * 001 : Avg 2 samples
	 * 010 : 4 samples, drop min and max, avg 2 samples
	 * 011 : 6 samples, drop min and max, avg 4 samples
	 * 100 : 10 samples, drop min and max, avg 8 samples
	 * 101 : 18 samples, drop min and max, avg 16 samples
	 *
	 * Bits:
	 *
	 * 0-2  : Sensor0 filter
	 * 3-5  : Sensor1 filter
	 * 6-8  : Sensor2 filter
	 * 9-11 : Sensor3 filter
	 */
	value = LVTS_HW_FILTER << 9 |  LVTS_HW_FILTER << 6 |
			LVTS_HW_FILTER << 3 | LVTS_HW_FILTER;
	if (lvts_ctrl->msrctl0)
		value = lvts_ctrl->msrctl0;
	writel(value, LVTS_MSRCTL0(lvts_ctrl->base));

	/*
	 * LVTS_MONCTL1 : Period unit and group interval configuration
	 *
	 * The clock source of LVTS thermal controller is 26MHz.
	 *
	 * The period unit is a time base for all the interval delays
	 * specified in the registers. By default we use 12. The time
	 * conversion is done by multiplying by 256 and 1/26.10^6
	 *
	 * An interval delay multiplied by the period unit gives the
	 * duration in seconds.
	 *
	 * - Filter interval delay is a delay between two samples of
	 * the same sensor.
	 *
	 * - Sensor interval delay is a delay between two samples of
	 * different sensors.
	 *
	 * - Group interval delay is a delay between different rounds.
	 *
	 * For example:
	 *     If Period unit = C, filter delay = 1, sensor delay = 2, group delay = 1,
	 *     and two sensors, TS1 and TS2, are in a LVTS thermal controller
	 *     and then
	 *     Period unit time = C * 1/26M * 256 = 12 * 38.46ns * 256 = 118.149us
	 *     Filter interval delay = 1 * Period unit = 118.149us
	 *     Sensor interval delay = 2 * Period unit = 236.298us
	 *     Group interval delay = 1 * Period unit = 118.149us
	 *
	 *     TS1    TS1 ... TS1    TS2    TS2 ... TS2    TS1...
	 *        <--> Filter interval delay
	 *                       <--> Sensor interval delay
	 *                                             <--> Group interval delay
	 * Bits:
	 *      29 - 20 : Group interval
	 *      16 - 13 : Send a single interrupt when crossing the hot threshold (1)
	 *                or an interrupt everytime the hot threshold is crossed (0)
	 *       9 - 0  : Period unit
	 *
	 */
	value = LVTS_GROUP_INTERVAL << 20 | LVTS_PERIOD_UNIT;
	if (lvts_ctrl->monctl1)
		value = lvts_ctrl->monctl1;
	writel(value, LVTS_MONCTL1(lvts_ctrl->base));

	/*
	 * LVTS_MONCTL2 : Filtering and sensor interval
	 *
	 * Bits:
	 *
	 *      25-16 : Interval unit in PERIOD_UNIT between sample on
	 *              the same sensor, filter interval
	 *       9-0  : Interval unit in PERIOD_UNIT between each sensor
	 *
	 */
	value = LVTS_FILTER_INTERVAL << 16 | LVTS_SENSOR_INTERVAL;
	if (lvts_ctrl->monctl2)
		value = lvts_ctrl->monctl2;
	writel(value, LVTS_MONCTL2(lvts_ctrl->base));

	return lvts_irq_init(lvts_ctrl);
}

/*
 * Safety gate: start the measurement on all calibrated sensors, wait for one valid reading
 * each and keep only sensors whose first reading is plausible for a device that just booted.
 */
static void lvts_gate_check_sensors(struct device *dev, struct lvts_ctrl *lvts_ctrl)
{
	const struct lvts_data *lvts_data = lvts_ctrl->lvts_data;
	u32 value;
	int i, rc, temp;

	writel(lvts_ctrl->valid_sensor_mask | BIT(9), LVTS_MONCTL0(lvts_ctrl->base));

	lvts_for_each_valid_sensor(i, lvts_ctrl) {
		struct lvts_sensor *lvts_sensor = &lvts_ctrl->sensors[i];

		rc = readl_poll_timeout(lvts_sensor->msr, value, value & BIT(16), 1000, 100000);
		if (rc) {
			dev_warn(dev, "lvts gate: sensor dt %d no valid reading (%#x), dropped\n",
				 lvts_sensor->dt_id, value);
			lvts_ctrl->valid_sensor_mask &= ~BIT(i);
			continue;
		}

		temp = lvts_raw_to_temp(value, lvts_data);
		if (temp < LVTS_GATE_BOOT_MIN || temp > LVTS_GATE_BOOT_MAX) {
			dev_warn(dev, "lvts gate: sensor dt %d first reading %d mC (raw %#x) implausible, dropped\n",
				 lvts_sensor->dt_id, temp, value);
			lvts_ctrl->valid_sensor_mask &= ~BIT(i);
			continue;
		}

		lvts_sensor->last_temp = temp;
		lvts_sensor->have_last = true;
		dev_info(dev, "lvts gate: sensor dt %d first reading %d mC\n",
			 lvts_sensor->dt_id, temp);
	}

	writel(lvts_ctrl->valid_sensor_mask | BIT(9), LVTS_MONCTL0(lvts_ctrl->base));
}

static int lvts_ctrl_start(struct device *dev, struct lvts_ctrl *lvts_ctrl,
			   enum lvts_gate_mode gate_mode)
{
	const struct thermal_zone_device_ops *ops = &lvts_ops;
	struct lvts_sensor *lvts_sensors = lvts_ctrl->sensors;
	struct thermal_zone_device *tz;
	u32 sensor_map = 0;
	int i;
	/*
	 * Bitmaps to enable each sensor on immediate and filtered modes, as
	 * described in MSRCTL1 and MONCTL0 registers below, respectively.
	 */
	u32 sensor_imm_bitmap[] = { BIT(4), BIT(5), BIT(6), BIT(9) };
	u32 sensor_filt_bitmap[] = { BIT(0), BIT(1), BIT(2), BIT(3) };

	u32 *sensor_bitmap = lvts_ctrl->mode == LVTS_MSR_IMMEDIATE_MODE ?
			     sensor_imm_bitmap : sensor_filt_bitmap;

	if (lvts_ctrl->lvts_data->safety_gate) {
		if (lvts_ctrl->mode != LVTS_MSR_FILTERED_MODE)
			return -EINVAL;
		lvts_gate_check_sensors(dev, lvts_ctrl);
		if (gate_mode == LVTS_GATE_ON)
			ops = &lvts_ops_poll;
	}

	lvts_for_each_valid_sensor(i, lvts_ctrl) {

		int dt_id = lvts_sensors[i].dt_id;

		tz = devm_thermal_of_zone_register(dev, dt_id, &lvts_sensors[i],
						   ops);
		if (IS_ERR(tz)) {
			/*
			 * This thermal zone is not described in the
			 * device tree. It is not an error from the
			 * thermal OF code POV, we just continue.
			 */
			if (PTR_ERR(tz) == -ENODEV)
				continue;

			return PTR_ERR(tz);
		}

		devm_thermal_add_hwmon_sysfs(dev, tz);

		/*
		 * The thermal zone pointer will be needed in the
		 * interrupt handler, we store it in the sensor
		 * structure. The thermal domain structure will be
		 * passed to the interrupt handler private data as the
		 * interrupt is shared for all the controller
		 * belonging to the thermal domain.
		 */
		WRITE_ONCE(lvts_sensors[i].tz, tz);

		/*
		 * This sensor was correctly associated with a thermal
		 * zone, let's set the corresponding bit in the sensor
		 * map, so we can enable the temperature monitoring in
		 * the hardware thermal controller.
		 */
		sensor_map |= sensor_bitmap[i];
	}

	/*
	 * The initialization of the thermal zones give us
	 * which sensor point to enable. If any thermal zone
	 * was not described in the device tree, it won't be
	 * enabled here in the sensor map.
	 */
	if (lvts_ctrl->mode == LVTS_MSR_IMMEDIATE_MODE) {
		/*
		 * LVTS_MSRCTL1 : Measurement control
		 *
		 * Bits:
		 *
		 * 9: Ignore MSRCTL0 config and do immediate measurement on sensor3
		 * 6: Ignore MSRCTL0 config and do immediate measurement on sensor2
		 * 5: Ignore MSRCTL0 config and do immediate measurement on sensor1
		 * 4: Ignore MSRCTL0 config and do immediate measurement on sensor0
		 *
		 * That configuration will ignore the filtering and the delays
		 * introduced in MONCTL1 and MONCTL2
		 */
		writel(sensor_map, LVTS_MSRCTL1(lvts_ctrl->base));
	} else {
		/*
		 * Bits:
		 *      9: Single point access flow
		 *    0-3: Enable sensing point 0-3
		 */
		writel(sensor_map | BIT(9), LVTS_MONCTL0(lvts_ctrl->base));
	}

	return 0;
}

/*
 * Safety gate "read": only readl() on the thermal controller. Logs what the boot loader left
 * in the controllers, the efuse calibration and, if a controller happens to measure, the
 * converted temperatures. No reset, no ASIF command, no thermal zone.
 */
static int lvts_gate_read(struct device *dev, struct lvts_domain *lvts_td,
			  const struct lvts_data *lvts_data)
{
	int i, j, ret;

	for (i = 0; i < lvts_data->num_lvts_ctrl; i++) {
		void __iomem *base = lvts_td->base + lvts_data->lvts_ctrl[i].offset;

		dev_info(dev, "lvts gate: read ctrl%d id %#x clken %#x monctl0 %#x monint %#x protctl %#x prottc %#x edata %#x %#x %#x msr %#x %#x %#x\n",
			 i, readl(LVTS_ID(base)), readl(LVTS_CLKEN(base)),
			 readl(LVTS_MONCTL0(base)), readl(LVTS_MONINT(base)),
			 readl(LVTS_PROTCTL(base)), readl(LVTS_PROTTC(base)),
			 readl(LVTS_EDATA00(base)), readl(LVTS_EDATA01(base)),
			 readl(LVTS_EDATA02(base)), readl(LVTS_MSR0(base)),
			 readl(LVTS_MSR1(base)), readl(LVTS_MSR2(base)));
	}

	lvts_td->read_only = true;

	ret = lvts_ctrl_init(dev, lvts_td, lvts_data);
	if (ret) {
		dev_warn(dev, "lvts gate: mode=read, calibration rejected (%d)\n", ret);
		return 0;
	}

	for (i = 0; i < lvts_td->num_lvts_ctrl; i++) {
		struct lvts_ctrl *lvts_ctrl = &lvts_td->lvts_ctrl[i];

		lvts_for_each_valid_sensor(j, lvts_ctrl) {
			u32 value = readl(lvts_ctrl->sensors[j].msr);

			if (value & BIT(16))
				dev_info(dev, "lvts gate: read sensor dt %d %d mC (boot loader setup)\n",
					 lvts_ctrl->sensors[j].dt_id,
					 lvts_raw_to_temp(value, lvts_data));
		}
	}

	dev_info(dev, "lvts gate: mode=read, nothing written, no thermal zone\n");

	return lvts_debugfs_init(dev, lvts_td);
}

static int lvts_domain_init(struct device *dev, struct lvts_domain *lvts_td,
					const struct lvts_data *lvts_data)
{
	struct lvts_ctrl *lvts_ctrl;
	int i, ret, active = 0, total = 0;

	if (lvts_data->safety_gate && lvts_td->gate_mode == LVTS_GATE_READ)
		return lvts_gate_read(dev, lvts_td, lvts_data);

	ret = lvts_ctrl_init(dev, lvts_td, lvts_data);
	if (ret)
		return ret;

	ret = lvts_domain_reset(dev, lvts_td->reset);
	if (ret) {
		dev_dbg(dev, "Failed to reset domain");
		return ret;
	}

	for (i = 0; i < lvts_td->num_lvts_ctrl; i++) {

		lvts_ctrl = &lvts_td->lvts_ctrl[i];

		/*
		 * Initialization steps:
		 *
		 * - Enable the clock
		 * - Connect to the LVTS
		 * - Initialize the LVTS
		 * - Prepare the calibration data
		 * - Select monitored sensors
		 * [ Configure sampling ]
		 * [ Configure the interrupt ]
		 * - Start measurement
		 */
		ret = lvts_ctrl_set_enable(lvts_ctrl, true);
		if (ret) {
			dev_dbg(dev, "Failed to enable LVTS clock");
			return ret;
		}

		ret = lvts_ctrl_connect(dev, lvts_ctrl);
		if (ret) {
			dev_dbg(dev, "Failed to connect to LVTS controller");
			return ret;
		}

		ret = lvts_ctrl_initialize(dev, lvts_ctrl);
		if (ret) {
			dev_dbg(dev, "Failed to initialize controller");
			return ret;
		}

		ret = lvts_ctrl_calibrate(dev, lvts_ctrl);
		if (ret) {
			dev_dbg(dev, "Failed to calibrate controller");
			return ret;
		}

		ret = lvts_ctrl_configure(dev, lvts_ctrl);
		if (ret) {
			dev_dbg(dev, "Failed to configure controller");
			return ret;
		}

		if (lvts_data->safety_gate) {
			/* hardware protection must stay off: no stage 3 interrupt */
			if (readl(LVTS_MONINT(lvts_ctrl->base))) {
				dev_warn(dev, "lvts gate: ctrl%d MONINT %#x, cleared\n", i,
					 readl(LVTS_MONINT(lvts_ctrl->base)));
				writel(0, LVTS_MONINT(lvts_ctrl->base));
			}
			dev_info(dev, "lvts gate: ctrl%d id %#x protctl %#x prottc %#x\n", i,
				 readl(LVTS_ID(lvts_ctrl->base)),
				 readl(LVTS_PROTCTL(lvts_ctrl->base)),
				 readl(LVTS_PROTTC(lvts_ctrl->base)));
		}

		ret = lvts_ctrl_start(dev, lvts_ctrl, lvts_td->gate_mode);
		if (ret) {
			dev_dbg(dev, "Failed to start controller");
			return ret;
		}

		total += hweight8(lvts_data->lvts_ctrl[i].valid_sensor_mask);
		active += hweight8(lvts_ctrl->valid_sensor_mask);
	}

	if (lvts_data->safety_gate)
		dev_info(dev, "lvts gate: mode=%s, %d of %d sensors active, hardware protection off\n",
			 lvts_td->gate_mode == LVTS_GATE_FULL ? "full" : "on", active, total);

	return lvts_debugfs_init(dev, lvts_td);
}

static int lvts_probe(struct platform_device *pdev)
{
	const struct lvts_data *lvts_data;
	struct lvts_domain *lvts_td;
	struct device *dev = &pdev->dev;
	struct resource *res;
	int irq = 0, ret;

	lvts_td = devm_kzalloc(dev, sizeof(*lvts_td), GFP_KERNEL);
	if (!lvts_td)
		return -ENOMEM;

	lvts_data = of_device_get_match_data(dev);
	if (!lvts_data)
		return -ENODEV;

	lvts_td->clk = devm_clk_get_enabled(dev, NULL);
	if (IS_ERR(lvts_td->clk))
		return dev_err_probe(dev, PTR_ERR(lvts_td->clk), "Failed to retrieve clock\n");

	res = platform_get_mem_or_io(pdev, 0);
	if (!res)
		return dev_err_probe(dev, (-ENXIO), "No IO resource\n");

	lvts_td->base = devm_platform_get_and_ioremap_resource(pdev, 0, &res);
	if (IS_ERR(lvts_td->base))
		return dev_err_probe(dev, PTR_ERR(lvts_td->base), "Failed to map io resource\n");

	/*
	 * Depending on the SoC+Firmware combination, the LVTS hardware may be
	 * may be actively used by one or even multiple concurrent MCUs!
	 * In this case, resetting it may produce either a severe slowdown of
	 * the entire system, or even a thermal protection AP reset, as some
	 * MCU(s) may be reading a very high or very low temperature while the
	 * LVTS is being reset.
	 *
	 * On those, don't fail if no reset is found as that may be omitted on
	 * purpose, but still check if there's one, because some board(s) may
	 * be running on a different bootchain with reduced firmwares or using
	 * firmwares with reduced functionality.
	 */
	lvts_td->reset = devm_reset_control_get_optional_exclusive(dev, NULL);
	if (IS_ERR(lvts_td->reset))
		return dev_err_probe(dev, PTR_ERR(lvts_td->reset), "Failed to get reset control\n");

	golden_temp_offset = lvts_data->temp_offset;

	lvts_td->gate_mode = LVTS_GATE_FULL;
	if (lvts_data->safety_gate) {
		if (sysfs_streq(lvts_mode, "on"))
			lvts_td->gate_mode = LVTS_GATE_ON;
		else if (sysfs_streq(lvts_mode, "full"))
			lvts_td->gate_mode = LVTS_GATE_FULL;
		else
			lvts_td->gate_mode = LVTS_GATE_READ;
	}
	if (lvts_td->gate_mode == LVTS_GATE_FULL) {
		irq = platform_get_irq(pdev, 0);
		if (irq < 0)
			return irq;
	}

	ret = lvts_domain_init(dev, lvts_td, lvts_data);
	if (ret)
		return dev_err_probe(dev, ret, "Failed to initialize the lvts domain\n");

	platform_set_drvdata(pdev, lvts_td);

	if (lvts_td->read_only || lvts_td->gate_mode != LVTS_GATE_FULL)
		return 0;

	/*
	 * At this point the LVTS is initialized and enabled. We can
	 * safely enable the interrupt.
	 */
	ret = devm_request_threaded_irq(dev, irq, NULL, lvts_irq_handler,
					IRQF_ONESHOT, dev_name(dev), lvts_td);
	if (ret)
		return ret;

	return 0;
}

static void lvts_remove(struct platform_device *pdev)
{
	struct lvts_domain *lvts_td;
	int i;

	lvts_td = platform_get_drvdata(pdev);
	if (lvts_td->read_only)
		return;

	for (i = 0; i < lvts_td->num_lvts_ctrl; i++)
		lvts_ctrl_set_enable(&lvts_td->lvts_ctrl[i], false);
}

static const struct lvts_ctrl_data mt7987_lvts_ap_data_ctrl[] = {
	{
		.lvts_sensor = {
			{ .dt_id = MT7987_CPU,
			  .cal_offsets = { 0x04, 0x05, 0x06 } },
			{ .dt_id = MT7987_ETH2P5G,
			  .cal_offsets = { 0x08, 0x09, 0x0a } },
		},
		VALID_SENSOR_MAP(1, 1, 0, 0),
		.offset = 0x0,
		.mode = LVTS_MSR_FILTERED_MODE,
	},
};

static const struct lvts_ctrl_data mt7988_lvts_ap_data_ctrl[] = {
	{
		.lvts_sensor = {
			{ .dt_id = MT7988_CPU_0,
			  .cal_offsets = { 0x00, 0x01, 0x02 } },
			{ .dt_id = MT7988_CPU_1,
			  .cal_offsets = { 0x04, 0x05, 0x06 } },
			{ .dt_id = MT7988_ETH2P5G_0,
			  .cal_offsets = { 0x08, 0x09, 0x0a } },
			{ .dt_id = MT7988_ETH2P5G_1,
			  .cal_offsets = { 0x0c, 0x0d, 0x0e } }
		},
		VALID_SENSOR_MAP(1, 1, 1, 1),
		.offset = 0x0,
	},
	{
		.lvts_sensor = {
			{ .dt_id = MT7988_TOPS_0,
			   .cal_offsets = { 0x14, 0x15, 0x16 } },
			{ .dt_id = MT7988_TOPS_1,
			   .cal_offsets = { 0x18, 0x19, 0x1a } },
			{ .dt_id = MT7988_ETHWARP_0,
			   .cal_offsets = { 0x1c, 0x1d, 0x1e } },
			{ .dt_id = MT7988_ETHWARP_1,
			   .cal_offsets = { 0x20, 0x21, 0x22 } }
		},
		VALID_SENSOR_MAP(1, 1, 1, 1),
		.offset = 0x100,
	}
};

static int lvts_suspend(struct device *dev)
{
	struct lvts_domain *lvts_td;
	int i;

	lvts_td = dev_get_drvdata(dev);
	if (lvts_td->read_only)
		return 0;

	for (i = 0; i < lvts_td->num_lvts_ctrl; i++) {
		lvts_ctrl_monitor_enable(dev, &lvts_td->lvts_ctrl[i], false);
		usleep_range(100, 200);
		lvts_ctrl_set_enable(&lvts_td->lvts_ctrl[i], false);
	}

	clk_disable_unprepare(lvts_td->clk);

	return 0;
}

static int lvts_resume(struct device *dev)
{
	struct lvts_domain *lvts_td;
	int i, ret;

	lvts_td = dev_get_drvdata(dev);
	if (lvts_td->read_only)
		return 0;

	ret = clk_prepare_enable(lvts_td->clk);
	if (ret)
		return ret;

	for (i = 0; i < lvts_td->num_lvts_ctrl; i++) {
		lvts_ctrl_set_enable(&lvts_td->lvts_ctrl[i], true);
		usleep_range(100, 200);
		lvts_ctrl_monitor_enable(dev, &lvts_td->lvts_ctrl[i], true);
	}

	return 0;
}

static const u32 default_conn_cmds[] = { 0xC103FFFF, 0xC502FF55 };
static const u32 mt7988_conn_cmds[] = { 0xC103FFFF, 0xC502FC55 };

/*
 * Write device mask: 0xC1030000
 */
static const u32 default_init_cmds[] = {
	0xC1030E01, 0xC1030CFC, 0xC1030A8C, 0xC103098D, 0xC10308F1,
	0xC10307A6, 0xC10306B8, 0xC1030500, 0xC1030420, 0xC1030300,
	0xC1030030, 0xC10300F6, 0xC1030050, 0xC1030060, 0xC10300AC,
	0xC10300FC, 0xC103009D, 0xC10300F1, 0xC10300E1
};

/*
 * MT6785 uses the vendor ASIF command prefix 0x81 (write) / 0x85 (read back), auto
 * RCK (0x0E = 0x01) and single mode (0x06 = 0xB8) in the vendor order
 * (lvts_device_identification(), lvts_Device_Enable_Init_all_Devices()).
 */
static const u32 mt6785_conn_cmds[] = { 0x8103FFFF, 0x8502FF55 };

static const u32 mt6785_init_cmds[] = {
	0x81030420, 0x81030500, 0x81030300, 0x810308C3, 0x8103098D,
	0x81030C7C, 0x81030AA8, 0x81030B04, 0x81030E01, 0x810306B8
};

static const u32 mt7987_init_cmds[] = {
	0xC1030300, 0xC1030420, 0xC1030500, 0xC10307A6, 0xC10308C7,
	0xC103098D, 0xC1030C7C, 0xC1030AA8, 0xC10308CE, 0xC10308C7,
	0xC1030B04, 0xC1030E01, 0xC10306B8
};

static const u32 mt7988_init_cmds[] = {
	0xC1030300, 0xC1030420, 0xC1030500, 0xC10307A6, 0xC1030CFC,
	0xC1030A8C, 0xC103098D, 0xC10308F1, 0xC1030B04, 0xC1030E01,
	0xC10306B8
};

/*
 * MT6785: four controllers, as in the vendor kernel: 0x000 big cluster (LVTS1_0/1), 0x100
 * LITTLE cluster (LVTS2_0/1/2), 0x200 GPU (LVTS3_0/1), 0x300 VPU (LVTS4_0). The sampling
 * setup is the vendor one: MONCTL1 = group interval << 20 | period unit, MONCTL2 = filter
 * interval << 16 | sensor interval (tc_speed), MSRCTL0 0x492 = 2 out of 4 samples on every
 * sensor.
 */
#define MT6785_LVTS_MONCTL2	0x00010001
#define MT6785_LVTS_MSRCTL0	0x00000492

static const struct lvts_ctrl_data mt6785_lvts_data_ctrl[] = {
	{
		.lvts_sensor = {
			{ .dt_id = MT6785_BIG_CPU0,
			  .cal_offsets = { 4, 5, 6 } },
			{ .dt_id = MT6785_BIG_CPU1,
			  .cal_offsets = { 8, 9, 10 } },
		},
		VALID_SENSOR_MAP(1, 1, 0, 0),
		.offset = 0x0,
		.mode = LVTS_MSR_FILTERED_MODE,
		.monctl1 = 0x0010000c,
		.monctl2 = MT6785_LVTS_MONCTL2,
		.msrctl0 = MT6785_LVTS_MSRCTL0,
	},
	{
		.lvts_sensor = {
			{ .dt_id = MT6785_LITTLE_CPU0,
			  .cal_offsets = { 12, 13, 14 } },
			{ .dt_id = MT6785_LITTLE_CPU1,
			  .cal_offsets = { 16, 17, 18 } },
			{ .dt_id = MT6785_LITTLE_CPU2,
			  .cal_offsets = { 20, 21, 22 } }
		},
		VALID_SENSOR_MAP(1, 1, 1, 0),
		.offset = 0x100,
		.mode = LVTS_MSR_FILTERED_MODE,
		.monctl1 = 0x00100007,
		.monctl2 = MT6785_LVTS_MONCTL2,
		.msrctl0 = MT6785_LVTS_MSRCTL0,
	},
	{
		.lvts_sensor = {
			{ .dt_id = MT6785_GPU0,
			  .cal_offsets = { 24, 25, 26 } },
			{ .dt_id = MT6785_GPU1,
			  .cal_offsets = { 28, 29, 30 } }
		},
		VALID_SENSOR_MAP(1, 1, 0, 0),
		.offset = 0x200,
		.mode = LVTS_MSR_FILTERED_MODE,
		.monctl1 = 0x0010000c,
		.monctl2 = MT6785_LVTS_MONCTL2,
		.msrctl0 = MT6785_LVTS_MSRCTL0,
	},
	{
		.lvts_sensor = {
			{ .dt_id = MT6785_VPU,
			  .cal_offsets = { 32, 33, 34 } }
		},
		VALID_SENSOR_MAP(1, 0, 0, 0),
		.offset = 0x300,
		.mode = LVTS_MSR_FILTERED_MODE,
		.monctl1 = 0x0050000c,
		.monctl2 = MT6785_LVTS_MONCTL2,
		.msrctl0 = MT6785_LVTS_MSRCTL0,
	}
};

/*
 * The MT8186 calibration data is stored as packed 3-byte little-endian
 * values using a weird layout that makes sense only when viewed as a 32-bit
 * hexadecimal word dump. Let's suppose SxBy where x = sensor number and
 * y = byte number where the LSB is y=0. We then have:
 *
 *   [S0B2-S0B1-S0B0-S1B2] [S1B1-S1B0-S2B2-S2B1] [S2B0-S3B2-S3B1-S3B0]
 *
 * However, when considering a byte stream, those appear as follows:
 *
 *   [S1B2] [S0B0[ [S0B1] [S0B2] [S2B1] [S2B2] [S1B0] [S1B1] [S3B0] [S3B1] [S3B2] [S2B0]
 *
 * Hence the rather confusing offsets provided below.
 */
static const struct lvts_ctrl_data mt8186_lvts_data_ctrl[] = {
	{
		.lvts_sensor = {
			{ .dt_id = MT8186_LITTLE_CPU0,
			  .cal_offsets = { 5, 6, 7 } },
			{ .dt_id = MT8186_LITTLE_CPU1,
			  .cal_offsets = { 10, 11, 4 } },
			{ .dt_id = MT8186_LITTLE_CPU2,
			  .cal_offsets = { 15, 8, 9 } },
			{ .dt_id = MT8186_CAM,
			  .cal_offsets = { 12, 13, 14 } }
		},
		VALID_SENSOR_MAP(1, 1, 1, 1),
		.offset = 0x0,
	},
	{
		.lvts_sensor = {
			{ .dt_id = MT8186_BIG_CPU0,
			  .cal_offsets = { 22, 23, 16 } },
			{ .dt_id = MT8186_BIG_CPU1,
			  .cal_offsets = { 27, 20, 21 } }
		},
		VALID_SENSOR_MAP(1, 1, 0, 0),
		.offset = 0x100,
	},
	{
		.lvts_sensor = {
			{ .dt_id = MT8186_NNA,
			  .cal_offsets = { 29, 30, 31 } },
			{ .dt_id = MT8186_ADSP,
			  .cal_offsets = { 34, 35, 28 } },
			{ .dt_id = MT8186_GPU,
			  .cal_offsets = { 39, 32, 33 } }
		},
		VALID_SENSOR_MAP(1, 1, 1, 0),
		.offset = 0x200,
	}
};

static const struct lvts_ctrl_data mt8188_lvts_mcu_data_ctrl[] = {
	{
		.lvts_sensor = {
			{ .dt_id = MT8188_MCU_LITTLE_CPU0,
			  .cal_offsets = { 22, 23, 24 } },
			{ .dt_id = MT8188_MCU_LITTLE_CPU1,
			  .cal_offsets = { 25, 26, 27 } },
			{ .dt_id = MT8188_MCU_LITTLE_CPU2,
			  .cal_offsets = { 28, 29, 30 } },
			{ .dt_id = MT8188_MCU_LITTLE_CPU3,
			  .cal_offsets = { 31, 32, 33 } },
		},
		VALID_SENSOR_MAP(1, 1, 1, 1),
		.offset = 0x0,
	},
	{
		.lvts_sensor = {
			{ .dt_id = MT8188_MCU_BIG_CPU0,
			  .cal_offsets = { 34, 35, 36 } },
			{ .dt_id = MT8188_MCU_BIG_CPU1,
			  .cal_offsets = { 37, 38, 39 } },
		},
		VALID_SENSOR_MAP(1, 1, 0, 0),
		.offset = 0x100,
	}
};

static const struct lvts_ctrl_data mt8188_lvts_ap_data_ctrl[] = {
	{
		.lvts_sensor = {

			{ /* unused */ },
			{ .dt_id = MT8188_AP_APU,
			  .cal_offsets = { 40, 41, 42 } },
		},
		VALID_SENSOR_MAP(0, 1, 0, 0),
		.offset = 0x0,
	},
	{
		.lvts_sensor = {
			{ .dt_id = MT8188_AP_GPU0,
			  .cal_offsets = { 43, 44, 45 } },
			{ .dt_id = MT8188_AP_GPU1,
			  .cal_offsets = { 46, 47, 48 } },
			{ .dt_id = MT8188_AP_ADSP,
			  .cal_offsets = { 49, 50, 51 } },
		},
		VALID_SENSOR_MAP(1, 1, 1, 0),
		.offset = 0x100,
	},
	{
		.lvts_sensor = {
			{ .dt_id = MT8188_AP_VDO,
			  .cal_offsets = { 52, 53, 54 } },
			{ .dt_id = MT8188_AP_INFRA,
			  .cal_offsets = { 55, 56, 57 } },
		},
		VALID_SENSOR_MAP(1, 1, 0, 0),
		.offset = 0x200,
	},
	{
		.lvts_sensor = {
			{ .dt_id = MT8188_AP_CAM1,
			  .cal_offsets = { 58, 59, 60 } },
			{ .dt_id = MT8188_AP_CAM2,
			  .cal_offsets = { 61, 62, 63 } },
		},
		VALID_SENSOR_MAP(1, 1, 0, 0),
		.offset = 0x300,
	}
};

static const struct lvts_ctrl_data mt8192_lvts_mcu_data_ctrl[] = {
	{
		.lvts_sensor = {
			{ .dt_id = MT8192_MCU_BIG_CPU0,
			  .cal_offsets = { 0x04, 0x05, 0x06 } },
			{ .dt_id = MT8192_MCU_BIG_CPU1,
			  .cal_offsets = { 0x08, 0x09, 0x0a } }
		},
		VALID_SENSOR_MAP(1, 1, 0, 0),
		.offset = 0x0,
		.mode = LVTS_MSR_FILTERED_MODE,
	},
	{
		.lvts_sensor = {
			{ .dt_id = MT8192_MCU_BIG_CPU2,
			  .cal_offsets = { 0x0c, 0x0d, 0x0e } },
			{ .dt_id = MT8192_MCU_BIG_CPU3,
			  .cal_offsets = { 0x10, 0x11, 0x12 } }
		},
		VALID_SENSOR_MAP(1, 1, 0, 0),
		.offset = 0x100,
		.mode = LVTS_MSR_FILTERED_MODE,
	},
	{
		.lvts_sensor = {
			{ .dt_id = MT8192_MCU_LITTLE_CPU0,
			  .cal_offsets = { 0x14, 0x15, 0x16 } },
			{ .dt_id = MT8192_MCU_LITTLE_CPU1,
			  .cal_offsets = { 0x18, 0x19, 0x1a } },
			{ .dt_id = MT8192_MCU_LITTLE_CPU2,
			  .cal_offsets = { 0x1c, 0x1d, 0x1e } },
			{ .dt_id = MT8192_MCU_LITTLE_CPU3,
			  .cal_offsets = { 0x20, 0x21, 0x22 } }
		},
		VALID_SENSOR_MAP(1, 1, 1, 1),
		.offset = 0x200,
		.mode = LVTS_MSR_FILTERED_MODE,
	}
};

static const struct lvts_ctrl_data mt8192_lvts_ap_data_ctrl[] = {
	{
		.lvts_sensor = {
			{ .dt_id = MT8192_AP_VPU0,
			  .cal_offsets = { 0x24, 0x25, 0x26 } },
			{ .dt_id = MT8192_AP_VPU1,
			  .cal_offsets = { 0x28, 0x29, 0x2a } }
		},
		VALID_SENSOR_MAP(1, 1, 0, 0),
		.offset = 0x0,
	},
	{
		.lvts_sensor = {
			{ .dt_id = MT8192_AP_GPU0,
			  .cal_offsets = { 0x2c, 0x2d, 0x2e } },
			{ .dt_id = MT8192_AP_GPU1,
			  .cal_offsets = { 0x30, 0x31, 0x32 } }
		},
		VALID_SENSOR_MAP(1, 1, 0, 0),
		.offset = 0x100,
	},
	{
		.lvts_sensor = {
			{ .dt_id = MT8192_AP_INFRA,
			  .cal_offsets = { 0x34, 0x35, 0x36 } },
			{ .dt_id = MT8192_AP_CAM,
			  .cal_offsets = { 0x38, 0x39, 0x3a } },
		},
		VALID_SENSOR_MAP(1, 1, 0, 0),
		.offset = 0x200,
	},
	{
		.lvts_sensor = {
			{ .dt_id = MT8192_AP_MD0,
			  .cal_offsets = { 0x3c, 0x3d, 0x3e } },
			{ .dt_id = MT8192_AP_MD1,
			  .cal_offsets = { 0x40, 0x41, 0x42 } },
			{ .dt_id = MT8192_AP_MD2,
			  .cal_offsets = { 0x44, 0x45, 0x46 } }
		},
		VALID_SENSOR_MAP(1, 1, 1, 0),
		.offset = 0x300,
	}
};

static const struct lvts_ctrl_data mt8195_lvts_mcu_data_ctrl[] = {
	{
		.lvts_sensor = {
			{ .dt_id = MT8195_MCU_BIG_CPU0,
			  .cal_offsets = { 0x04, 0x05, 0x06 } },
			{ .dt_id = MT8195_MCU_BIG_CPU1,
			  .cal_offsets = { 0x07, 0x08, 0x09 } }
		},
		VALID_SENSOR_MAP(1, 1, 0, 0),
		.offset = 0x0,
	},
	{
		.lvts_sensor = {
			{ .dt_id = MT8195_MCU_BIG_CPU2,
			  .cal_offsets = { 0x0d, 0x0e, 0x0f } },
			{ .dt_id = MT8195_MCU_BIG_CPU3,
			  .cal_offsets = { 0x10, 0x11, 0x12 } }
		},
		VALID_SENSOR_MAP(1, 1, 0, 0),
		.offset = 0x100,
	},
	{
		.lvts_sensor = {
			{ .dt_id = MT8195_MCU_LITTLE_CPU0,
			  .cal_offsets = { 0x16, 0x17, 0x18 } },
			{ .dt_id = MT8195_MCU_LITTLE_CPU1,
			  .cal_offsets = { 0x19, 0x1a, 0x1b } },
			{ .dt_id = MT8195_MCU_LITTLE_CPU2,
			  .cal_offsets = { 0x1c, 0x1d, 0x1e } },
			{ .dt_id = MT8195_MCU_LITTLE_CPU3,
			  .cal_offsets = { 0x1f, 0x20, 0x21 } }
		},
		VALID_SENSOR_MAP(1, 1, 1, 1),
		.offset = 0x200,
	}
};

static const struct lvts_ctrl_data mt8195_lvts_ap_data_ctrl[] = {
	{
		.lvts_sensor = {
			{ .dt_id = MT8195_AP_VPU0,
			  .cal_offsets = { 0x25, 0x26, 0x27 } },
			{ .dt_id = MT8195_AP_VPU1,
			  .cal_offsets = { 0x28, 0x29, 0x2a } }
		},
		VALID_SENSOR_MAP(1, 1, 0, 0),
		.offset = 0x0,
	},
	{
		.lvts_sensor = {
			{ .dt_id = MT8195_AP_GPU0,
			  .cal_offsets = { 0x2e, 0x2f, 0x30 } },
			{ .dt_id = MT8195_AP_GPU1,
			  .cal_offsets = { 0x31, 0x32, 0x33 } }
		},
		VALID_SENSOR_MAP(1, 1, 0, 0),
		.offset = 0x100,
	},
	{
		.lvts_sensor = {
			{ .dt_id = MT8195_AP_VDEC,
			  .cal_offsets = { 0x37, 0x38, 0x39 } },
			{ .dt_id = MT8195_AP_IMG,
			  .cal_offsets = { 0x3a, 0x3b, 0x3c } },
			{ .dt_id = MT8195_AP_INFRA,
			  .cal_offsets = { 0x3d, 0x3e, 0x3f } }
		},
		VALID_SENSOR_MAP(1, 1, 1, 0),
		.offset = 0x200,
	},
	{
		.lvts_sensor = {
			{ .dt_id = MT8195_AP_CAM0,
			  .cal_offsets = { 0x43, 0x44, 0x45 } },
			{ .dt_id = MT8195_AP_CAM1,
			  .cal_offsets = { 0x46, 0x47, 0x48 } }
		},
		VALID_SENSOR_MAP(1, 1, 0, 0),
		.offset = 0x300,
	}
};

static const struct lvts_ctrl_data mt8196_lvts_mcu_data_ctrl[] = {
	{
		.lvts_sensor = {
			{ .dt_id = MT8196_MCU_MEDIUM_CPU6_0,
			  .cal_offsets = { 0x06, 0x07 } },
			{ .dt_id = MT8196_MCU_MEDIUM_CPU6_1,
			  .cal_offsets = { 0x04, 0x05 } },
			{ .dt_id = MT8196_MCU_DSU2,
			  .cal_offsets = { 0x0A, 0x0B } },
			{ .dt_id = MT8196_MCU_DSU3,
			  .cal_offsets = { 0x08, 0x09 } }
		},
		VALID_SENSOR_MAP(1, 1, 1, 1),
		.offset = 0x0,
		.mode = LVTS_MSR_ATP_MODE,
	},
	{
		.lvts_sensor = {
			{ .dt_id = MT8196_MCU_LITTLE_CPU3,
			  .cal_offsets = { 0x0E, 0x0F } },
			{ .dt_id = MT8196_MCU_LITTLE_CPU0,
			  .cal_offsets = { 0x0C, 0x0D } },
			{ .dt_id = MT8196_MCU_LITTLE_CPU1,
			  .cal_offsets = { 0x12, 0x13 } },
			{ .dt_id = MT8196_MCU_LITTLE_CPU2,
			  .cal_offsets = { 0x10, 0x11 } }
		},
		VALID_SENSOR_MAP(1, 1, 1, 1),
		.offset = 0x100,
		.mode = LVTS_MSR_ATP_MODE,
	},
	{
		.lvts_sensor = {
			{ .dt_id = MT8196_MCU_MEDIUM_CPU4_0,
			  .cal_offsets = { 0x16, 0x17 } },
			{ .dt_id = MT8196_MCU_MEDIUM_CPU4_1,
			  .cal_offsets = { 0x14, 0x15 } },
			{ .dt_id = MT8196_MCU_MEDIUM_CPU5_0,
			  .cal_offsets = { 0x1A, 0x1B } },
			{ .dt_id = MT8196_MCU_MEDIUM_CPU5_1,
			  .cal_offsets = { 0x18, 0x19 } }
		},
		VALID_SENSOR_MAP(1, 1, 1, 1),
		.offset = 0x200,
		.mode = LVTS_MSR_ATP_MODE,
	},
	{
		.lvts_sensor = {
			{ .dt_id = MT8196_MCU_DSU0,
			  .cal_offsets = { 0x1E, 0x1F } },
			{ .dt_id = MT8196_MCU_DSU1,
			  .cal_offsets = { 0x1C, 0x1D } },
			{ .dt_id = MT8196_MCU_BIG_CPU7_0,
			  .cal_offsets = { 0x22, 0x23 } },
			{ .dt_id = MT8196_MCU_BIG_CPU7_1,
			  .cal_offsets = { 0x20, 0x21 } }
		},
		VALID_SENSOR_MAP(1, 1, 1, 1),
		.offset = 0x300,
		.mode = LVTS_MSR_ATP_MODE,
	}
};

static const struct lvts_ctrl_data mt8196_lvts_ap_data_ctrl[] = {
	{
		.lvts_sensor = {
			{ .dt_id = MT8196_AP_TOP0,
			  .cal_offsets = { 0x32, 0x33 } },
			{ .dt_id = MT8196_AP_TOP1,
			  .cal_offsets = { 0x30, 0x31 } },
			{ .dt_id = MT8196_AP_TOP2,
			  .cal_offsets = { 0x36, 0x37 } },
			{ .dt_id = MT8196_AP_TOP3,
			  .cal_offsets = { 0x34, 0x35 } }
		},
		VALID_SENSOR_MAP(1, 1, 1, 1),
		.offset = 0x0,
		.mode = LVTS_MSR_ATP_MODE,
	},
	{
		.lvts_sensor = {
			{ .dt_id = MT8196_AP_BOT0,
			  .cal_offsets = { 0x3A, 0x3B } },
			{ .dt_id = MT8196_AP_BOT1,
			  .cal_offsets = { 0x38, 0x39 } },
			{ .dt_id = MT8196_AP_BOT2,
			  .cal_offsets = { 0x3E, 0x3F } },
			{ .dt_id = MT8196_AP_BOT3,
			  .cal_offsets = { 0x3C, 0x3D } }
		},
		VALID_SENSOR_MAP(1, 1, 1, 1),
		.offset = 0x100,
		.mode = LVTS_MSR_ATP_MODE,
	}
};

static const struct lvts_platform_ops lvts_platform_ops_mt7988 = {
	.lvts_raw_to_temp = lvts_raw_to_temp_mt7988,
	.lvts_temp_to_raw = lvts_temp_to_raw_mt7988,
};

static const struct lvts_platform_ops lvts_platform_ops_mt8196 = {
	.lvts_raw_to_temp = lvts_raw_to_temp_mt7988,
	.lvts_temp_to_raw = lvts_temp_to_raw_mt8196,
};

/*
 * Calibration: golden temperature in bits 7:0 of the first word, then one 24 bit count_r per
 * sensor (vendor lvts_thermal_cal_prepare()); the nvmem cells concatenate to the cal_offsets
 * above. Conversion as in the vendor kernel: T = -204650 * raw / 16384 + golden * 500 + 204650.
 */
static const struct lvts_data mt6785_lvts_data = {
	.lvts_ctrl	= mt6785_lvts_data_ctrl,
	.num_lvts_ctrl	= ARRAY_SIZE(mt6785_lvts_data_ctrl),
	.conn_cmd	= mt6785_conn_cmds,
	.init_cmd	= mt6785_init_cmds,
	.num_conn_cmd	= ARRAY_SIZE(mt6785_conn_cmds),
	.num_init_cmd	= ARRAY_SIZE(mt6785_init_cmds),
	.num_cal_offsets = LVTS_NUM_CAL_OFFSETS_MT7988,
	.temp_factor	= LVTS_COEFF_A_MT7988,
	.temp_offset	= LVTS_COEFF_B_MT7988,
	.gt_calib_bit_offset = 0,
	.def_calibration = 19000,
	.write_calscale	= true,
	.safety_gate	= true,
	.ops = &lvts_platform_ops_mt7988,
};

static const struct lvts_data mt7987_lvts_ap_data = {
	.lvts_ctrl	= mt7987_lvts_ap_data_ctrl,
	.num_lvts_ctrl	= ARRAY_SIZE(mt7987_lvts_ap_data_ctrl),
	.conn_cmd	= mt7988_conn_cmds,
	.init_cmd	= mt7987_init_cmds,
	.num_conn_cmd	= ARRAY_SIZE(mt7988_conn_cmds),
	.num_init_cmd	= ARRAY_SIZE(mt7987_init_cmds),
	.temp_factor	= LVTS_COEFF_A_MT7987,
	.temp_offset	= LVTS_COEFF_B_MT7987,
	.gt_calib_bit_offset = 32,
	.def_calibration = 19380,
};

static const struct lvts_data mt7988_lvts_ap_data = {
	.lvts_ctrl	= mt7988_lvts_ap_data_ctrl,
	.conn_cmd	= mt7988_conn_cmds,
	.init_cmd	= mt7988_init_cmds,
	.num_lvts_ctrl	= ARRAY_SIZE(mt7988_lvts_ap_data_ctrl),
	.num_conn_cmd	= ARRAY_SIZE(mt7988_conn_cmds),
	.num_init_cmd	= ARRAY_SIZE(mt7988_init_cmds),
	.temp_factor	= LVTS_COEFF_A_MT7988,
	.temp_offset	= LVTS_COEFF_B_MT7988,
	.gt_calib_bit_offset = 24,
	.num_cal_offsets = LVTS_NUM_CAL_OFFSETS_MT7988,
	.ops = &lvts_platform_ops_mt7988,
};

static const struct lvts_data mt8186_lvts_data = {
	.lvts_ctrl	= mt8186_lvts_data_ctrl,
	.conn_cmd	= default_conn_cmds,
	.init_cmd	= default_init_cmds,
	.num_lvts_ctrl	= ARRAY_SIZE(mt8186_lvts_data_ctrl),
	.num_conn_cmd	= ARRAY_SIZE(default_conn_cmds),
	.num_init_cmd	= ARRAY_SIZE(default_init_cmds),
	.temp_factor	= LVTS_COEFF_A_MT7988,
	.temp_offset	= LVTS_COEFF_B_MT7988,
	.gt_calib_bit_offset = 24,
	.def_calibration = 19000,
	.num_cal_offsets = LVTS_NUM_CAL_OFFSETS_MT7988,
	.ops = &lvts_platform_ops_mt7988,
};

static const struct lvts_data mt8188_lvts_mcu_data = {
	.lvts_ctrl	= mt8188_lvts_mcu_data_ctrl,
	.conn_cmd	= default_conn_cmds,
	.init_cmd	= default_init_cmds,
	.num_lvts_ctrl	= ARRAY_SIZE(mt8188_lvts_mcu_data_ctrl),
	.num_conn_cmd	= ARRAY_SIZE(default_conn_cmds),
	.num_init_cmd	= ARRAY_SIZE(default_init_cmds),
	.temp_factor	= LVTS_COEFF_A_MT8195,
	.temp_offset	= LVTS_COEFF_B_MT8195,
	.gt_calib_bit_offset = 20,
	.def_calibration = 35000,
	.num_cal_offsets = LVTS_NUM_CAL_OFFSETS_MT7988,
	.ops = &lvts_platform_ops_mt7988,
};

static const struct lvts_data mt8188_lvts_ap_data = {
	.lvts_ctrl	= mt8188_lvts_ap_data_ctrl,
	.conn_cmd	= default_conn_cmds,
	.init_cmd	= default_init_cmds,
	.num_lvts_ctrl	= ARRAY_SIZE(mt8188_lvts_ap_data_ctrl),
	.num_conn_cmd	= ARRAY_SIZE(default_conn_cmds),
	.num_init_cmd	= ARRAY_SIZE(default_init_cmds),
	.temp_factor	= LVTS_COEFF_A_MT8195,
	.temp_offset	= LVTS_COEFF_B_MT8195,
	.gt_calib_bit_offset = 20,
	.def_calibration = 35000,
	.num_cal_offsets = LVTS_NUM_CAL_OFFSETS_MT7988,
	.ops = &lvts_platform_ops_mt7988,
};

static const struct lvts_data mt8192_lvts_mcu_data = {
	.lvts_ctrl	= mt8192_lvts_mcu_data_ctrl,
	.conn_cmd	= default_conn_cmds,
	.init_cmd	= default_init_cmds,
	.num_lvts_ctrl	= ARRAY_SIZE(mt8192_lvts_mcu_data_ctrl),
	.num_conn_cmd	= ARRAY_SIZE(default_conn_cmds),
	.num_init_cmd	= ARRAY_SIZE(default_init_cmds),
	.temp_factor	= LVTS_COEFF_A_MT8195,
	.temp_offset	= LVTS_COEFF_B_MT8195,
	.gt_calib_bit_offset = 24,
	.def_calibration = 35000,
	.num_cal_offsets = LVTS_NUM_CAL_OFFSETS_MT7988,
	.ops = &lvts_platform_ops_mt7988,
};

static const struct lvts_data mt8192_lvts_ap_data = {
	.lvts_ctrl	= mt8192_lvts_ap_data_ctrl,
	.conn_cmd	= default_conn_cmds,
	.init_cmd	= default_init_cmds,
	.num_lvts_ctrl	= ARRAY_SIZE(mt8192_lvts_ap_data_ctrl),
	.num_conn_cmd	= ARRAY_SIZE(default_conn_cmds),
	.num_init_cmd	= ARRAY_SIZE(default_init_cmds),
	.temp_factor	= LVTS_COEFF_A_MT8195,
	.temp_offset	= LVTS_COEFF_B_MT8195,
	.gt_calib_bit_offset = 24,
	.def_calibration = 35000,
	.num_cal_offsets = LVTS_NUM_CAL_OFFSETS_MT7988,
	.ops = &lvts_platform_ops_mt7988,
};

static const struct lvts_data mt8195_lvts_mcu_data = {
	.lvts_ctrl	= mt8195_lvts_mcu_data_ctrl,
	.conn_cmd	= default_conn_cmds,
	.init_cmd	= default_init_cmds,
	.num_lvts_ctrl	= ARRAY_SIZE(mt8195_lvts_mcu_data_ctrl),
	.num_conn_cmd	= ARRAY_SIZE(default_conn_cmds),
	.num_init_cmd	= ARRAY_SIZE(default_init_cmds),
	.temp_factor	= LVTS_COEFF_A_MT8195,
	.temp_offset	= LVTS_COEFF_B_MT8195,
	.gt_calib_bit_offset = 24,
	.def_calibration = 35000,
	.num_cal_offsets = LVTS_NUM_CAL_OFFSETS_MT7988,
	.ops = &lvts_platform_ops_mt7988,
};

static const struct lvts_data mt8195_lvts_ap_data = {
	.lvts_ctrl	= mt8195_lvts_ap_data_ctrl,
	.conn_cmd	= default_conn_cmds,
	.init_cmd	= default_init_cmds,
	.num_lvts_ctrl	= ARRAY_SIZE(mt8195_lvts_ap_data_ctrl),
	.num_conn_cmd	= ARRAY_SIZE(default_conn_cmds),
	.num_init_cmd	= ARRAY_SIZE(default_init_cmds),
	.temp_factor	= LVTS_COEFF_A_MT8195,
	.temp_offset	= LVTS_COEFF_B_MT8195,
	.gt_calib_bit_offset = 24,
	.def_calibration = 35000,
	.num_cal_offsets = LVTS_NUM_CAL_OFFSETS_MT7988,
	.ops = &lvts_platform_ops_mt7988,
};

static const struct lvts_data mt8196_lvts_mcu_data = {
	.lvts_ctrl	= mt8196_lvts_mcu_data_ctrl,
	.num_lvts_ctrl	= ARRAY_SIZE(mt8196_lvts_mcu_data_ctrl),
	.temp_factor	= LVTS_COEFF_A_MT8196,
	.temp_offset	= LVTS_COEFF_B_MT8196,
	.gt_calib_bit_offset = 0,
	.def_calibration = 14437,
	.num_cal_offsets = LVTS_NUM_CAL_OFFSETS_MT8196,
	.msr_offset = LVTS_MSR_OFFSET_MT8196,
	.ops = &lvts_platform_ops_mt8196,
};

static const struct lvts_data mt8196_lvts_ap_data = {
	.lvts_ctrl	= mt8196_lvts_ap_data_ctrl,
	.num_lvts_ctrl	= ARRAY_SIZE(mt8196_lvts_ap_data_ctrl),
	.temp_factor	= LVTS_COEFF_A_MT8196,
	.temp_offset	= LVTS_COEFF_B_MT8196,
	.gt_calib_bit_offset = 0,
	.def_calibration = 14437,
	.num_cal_offsets = LVTS_NUM_CAL_OFFSETS_MT8196,
	.msr_offset = LVTS_MSR_OFFSET_MT8196,
	.ops = &lvts_platform_ops_mt8196,
};

static const struct of_device_id lvts_of_match[] = {
	{ .compatible = "mediatek,mt6785-lvts", .data = &mt6785_lvts_data },
	{ .compatible = "mediatek,mt7987-lvts-ap", .data = &mt7987_lvts_ap_data },
	{ .compatible = "mediatek,mt7988-lvts-ap", .data = &mt7988_lvts_ap_data },
	{ .compatible = "mediatek,mt8186-lvts", .data = &mt8186_lvts_data },
	{ .compatible = "mediatek,mt8188-lvts-mcu", .data = &mt8188_lvts_mcu_data },
	{ .compatible = "mediatek,mt8188-lvts-ap", .data = &mt8188_lvts_ap_data },
	{ .compatible = "mediatek,mt8192-lvts-mcu", .data = &mt8192_lvts_mcu_data },
	{ .compatible = "mediatek,mt8192-lvts-ap", .data = &mt8192_lvts_ap_data },
	{ .compatible = "mediatek,mt8195-lvts-mcu", .data = &mt8195_lvts_mcu_data },
	{ .compatible = "mediatek,mt8195-lvts-ap", .data = &mt8195_lvts_ap_data },
	{ .compatible = "mediatek,mt8196-lvts-mcu", .data = &mt8196_lvts_mcu_data },
	{ .compatible = "mediatek,mt8196-lvts-ap", .data = &mt8196_lvts_ap_data },
	{},
};
MODULE_DEVICE_TABLE(of, lvts_of_match);

static const struct dev_pm_ops lvts_pm_ops = {
	NOIRQ_SYSTEM_SLEEP_PM_OPS(lvts_suspend, lvts_resume)
};

static struct platform_driver lvts_driver = {
	.probe = lvts_probe,
	.remove = lvts_remove,
	.driver = {
		.name = "mtk-lvts-thermal",
		.of_match_table = lvts_of_match,
		.pm = &lvts_pm_ops,
	},
};
module_platform_driver(lvts_driver);

MODULE_AUTHOR("Balsam CHIHI <bchihi@baylibre.com>");
MODULE_DESCRIPTION("MediaTek LVTS Thermal Driver");
MODULE_LICENSE("GPL");
