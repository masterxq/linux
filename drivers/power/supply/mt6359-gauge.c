// SPDX-License-Identifier: GPL-2.0-only
/*
 * MediaTek MT6359 fuel gauge (FGADC): battery current, coulomb counter and
 * state of charge.
 *
 * The FGADC of the MT6359 PMIC measures the battery current over the sense
 * resistor and integrates it in hardware: every 0.5 s (one tick of the time
 * counter NTER) the mean current of that half second is added to the 32 bit
 * coulomb counter CAR. The bootloader enables the gauge (FG_ON) and resets
 * the counters only when the battery was removed or the device was off for a
 * long time; the counters keep running across reboots and system sleep. This
 * driver never enables, resets or recalibrates the gauge.
 *
 * Readings are taken with the software latch handshake in FGADC_CON1, which
 * copies current and counters into the readable registers:
 *   FG_SW_READ_PRE (bit 0) = 1, wait for FG_LATCHDATA_ST (bit 15) = 1,
 *   read, FG_SW_CLEAR (bit 3) = 1 and FG_SW_READ_PRE = 0, wait for
 *   FG_LATCHDATA_ST = 0, FG_SW_CLEAR = 0.
 * Only bits 0 and 3 are touched; the reset bits of the same register are
 * written back as read.
 *
 * Register layout, handshake and scaling follow the vendor kernel
 * (mt6359_gauge.c: fgauge_read_current(), fgauge_get_coulomb() and
 * fgauge_get_time(), GPL-2.0): current LSB 610.352 uA at a 5 mOhm sense resistor, time LSB
 * 0.5 s; one CAR LSB is one current LSB for one tick (84.77 nAh, rounded to
 * 85 nAh by the vendor). The vendor driver scales current and charge by
 * 5 mOhm / R_FG_VALUE and by the board CAR_TUNE_VALUE; here both are one
 * effective "shunt-resistor-micro-ohms" from the device tree.
 *
 * The current reported is FG_CIC2, the mean current of the last tick that was
 * added to CAR, so current and charge agree by construction. FG_CURRENT_OUT
 * (the vendor's current) is a much shorter sample: on battery it includes the
 * CPU burst of the very read that latches it and reads 80..90 mA more
 * discharge than the counter.
 *
 * This is the battery power supply of the board: voltage and temperature come
 * from IIO channels (MT6359 AUXADC BATADC and the battery NTC), the design
 * data and the open circuit voltage table from the monitored battery.
 *
 * State of charge: the driver keeps the counter value at which the battery
 * would be empty ("CAR at 0 %"); the capacity is the counter distance from it
 * relative to the full charge. That reference is set
 *   - at probe from the open circuit voltage (terminal voltage corrected by
 *     current times internal resistance), unless a stored reference is valid,
 *   - to "full" when the charger reports a full battery and the voltage
 *     estimate shows at least 90 %,
 *   - downwards when the battery voltage shows less than 10 % while the
 *     counter still shows more (two polls in a row).
 * The reference is kept in the two spare registers RG_SYSTEM_INFO_CON1/2 of
 * the gauge, which keep their contents as long as the PMIC is powered, like
 * the counter itself. As CAR keeps counting while the kernel does not run
 * (reboot, charging in the bootloader, power off), the capacity stays right
 * across all of these. The stored reference is ignored when the counters were
 * reset during this boot or when it differs from the voltage estimate by more
 * than 20 %.
 *
 * Across system suspend the driver logs the charge drawn between its noirq
 * suspend and noirq resume, i.e. the average battery current while asleep.
 * The debugfs file mt6359-gauge/raw shows one latched set of raw registers.
 */
#include <linux/bitfield.h>
#include <linux/debugfs.h>
#include <linux/devm-helpers.h>
#include <linux/iio/consumer.h>
#include <linux/ktime.h>
#include <linux/math64.h>
#include <linux/mfd/mt6397/core.h>
#include <linux/module.h>
#include <linux/mod_devicetable.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/pm.h>
#include <linux/power_supply.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/seq_file.h>
#include <linux/workqueue.h>

#define MT6359_FGADC_CON0		0xd08
#define MT6359_FGADC_CON1		0xd0a
#define MT6359_FGADC_CAR_CON0		0xd16
#define MT6359_FGADC_CAR_CON1		0xd18
#define MT6359_FGADC_NTER_CON0		0xd38
#define MT6359_FGADC_NTER_CON1		0xd3a
#define MT6359_FGADC_CUR_CON0		0xd8a
#define MT6359_FGADC_CUR_CON3		0xd90
#define MT6359_FGADC_OFFSET_CON0	0xd92
#define MT6359_SYSTEM_INFO_CON1		0xd9c
#define MT6359_SYSTEM_INFO_CON2		0xd9e

#define FG_ON				BIT(0)
#define FG_SW_READ_PRE			BIT(0)
#define FG_SW_CLEAR			BIT(3)
#define FG_LATCHDATA_ST			BIT(15)
#define FG_NTER_29_16			GENMASK(13, 0)

/* stored reference: CON1 = bits 15:0, CON2 = magic and bits 27:16 */
#define FG_REF_MAGIC_MASK		GENMASK(15, 12)
#define FG_REF_MAGIC			0xa
#define FG_REF_HI_MASK			GENMASK(11, 0)
#define FG_REF_BITS			28
#define FG_REF_LIMIT			((s64)BIT_ULL(FG_REF_BITS - 1))

/* LSBs at the reference sense resistor */
#define FG_REF_SHUNT_UOHM		5000
#define FG_CUR_LSB_NA			610352LL	/* 610.352 uA */
#define FG_TICK_MS			500		/* NTER */
#define FG_CAR_LSB_NAS			(FG_CUR_LSB_NA * FG_TICK_MS / 1000)

#define FG_POLL_MS			30000
/* counters younger than the uptime plus this were reset during this boot */
#define FG_RESET_MARGIN_S		120
/* largest accepted difference of a stored reference to the OCV estimate */
#define FG_REF_MAX_DIFF			2000		/* 0.01 % */
/* low battery correction: voltage estimate below, counter above it by */
#define FG_LOW_CAP			1000		/* 0.01 % */
#define FG_LOW_DIFF			500		/* 0.01 % */
/* "full" from the charger needs at least this from the voltage estimate */
#define FG_FULL_CAP			9000		/* 0.01 % */

struct mt6359_gauge_sample {
	s16 cur;		/* FG_CURRENT_OUT, short term sample */
	s16 cic2;		/* FG_CIC2, mean of the last tick, added to CAR */
	s32 car;
	u32 nter;
};

struct mt6359_gauge {
	struct device *dev;
	struct regmap *regmap;
	struct power_supply *psy;
	struct power_supply_battery_info *info;
	struct iio_channel *voltage;
	struct iio_channel *temperature;
	struct delayed_work work;
	struct mutex lock;		/* latch handshake and the state below */
	u32 shunt_uohm;
	bool fg_on;
	int status;
	s64 full_lsb;			/* full charge in CAR LSB */
	s64 car_zero;			/* CAR at 0 % */
	int last_cap;			/* last capacity reported, % */
	int low_count;
	s64 win_car;			/* CAR and time at the last poll */
	ktime_t win_t;
	int avg_ua;			/* mean current between the last two polls */
	s64 sleep_car;
	ktime_t sleep_t;
	bool sleep_valid;
	struct dentry *debugfs;
};

static int mt6359_gauge_latch(struct mt6359_gauge *g,
			      struct mt6359_gauge_sample *s, int *offset)
{
	unsigned int st, cur, cic2, lo, hi, nlo, nhi, off;
	int ret, ret2;

	ret = regmap_update_bits(g->regmap, MT6359_FGADC_CON1, FG_SW_READ_PRE,
				 FG_SW_READ_PRE);
	if (ret)
		return ret;
	ret = regmap_read_poll_timeout(g->regmap, MT6359_FGADC_CON1, st,
				       st & FG_LATCHDATA_ST, 20, 20000);
	if (!ret)
		ret = regmap_read(g->regmap, MT6359_FGADC_CUR_CON0, &cur);
	if (!ret)
		ret = regmap_read(g->regmap, MT6359_FGADC_CUR_CON3, &cic2);
	if (!ret)
		ret = regmap_read(g->regmap, MT6359_FGADC_CAR_CON0, &lo);
	if (!ret)
		ret = regmap_read(g->regmap, MT6359_FGADC_CAR_CON1, &hi);
	if (!ret)
		ret = regmap_read(g->regmap, MT6359_FGADC_NTER_CON0, &nlo);
	if (!ret)
		ret = regmap_read(g->regmap, MT6359_FGADC_NTER_CON1, &nhi);
	if (!ret && offset)
		ret = regmap_read(g->regmap, MT6359_FGADC_OFFSET_CON0, &off);

	/* release the latch in any case, like the vendor code */
	ret2 = regmap_update_bits(g->regmap, MT6359_FGADC_CON1,
				  FG_SW_READ_PRE | FG_SW_CLEAR, FG_SW_CLEAR);
	if (!ret2)
		ret2 = regmap_read_poll_timeout(g->regmap, MT6359_FGADC_CON1, st,
						!(st & FG_LATCHDATA_ST), 20, 20000);
	if (!ret2)
		ret2 = regmap_update_bits(g->regmap, MT6359_FGADC_CON1,
					  FG_SW_CLEAR, 0);
	if (ret || ret2)
		return ret ?: ret2;

	s->cur = (s16)cur;
	s->cic2 = (s16)cic2;
	s->car = (s32)((hi & 0xffff) << 16 | (lo & 0xffff));
	s->nter = (nhi & FG_NTER_29_16) << 16 | (nlo & 0xffff);
	if (offset)
		*offset = (s16)off;
	return 0;
}

/*
 * The products below stay far inside s64 (|raw| < 2^31, constants < 2^31),
 * the divisors do not fit into the s32 divisor of div_s64(): use div64_s64().
 */
static int mt6359_gauge_ua(struct mt6359_gauge *g, s64 raw)
{
	return div64_s64(raw * FG_CUR_LSB_NA * FG_REF_SHUNT_UOHM,
			 (s64)g->shunt_uohm * 1000);
}

static s64 mt6359_gauge_uah(struct mt6359_gauge *g, s64 car)
{
	return div64_s64(car * FG_CAR_LSB_NAS * FG_REF_SHUNT_UOHM,
			 (s64)g->shunt_uohm * 3600 * 1000);
}

static s64 mt6359_gauge_uah_to_lsb(struct mt6359_gauge *g, s64 uah)
{
	return div64_s64(uah * g->shunt_uohm * 3600 * 1000,
			 FG_CAR_LSB_NAS * FG_REF_SHUNT_UOHM);
}

static int mt6359_gauge_voltage(struct mt6359_gauge *g, int *uv)
{
	int ret = iio_read_channel_processed(g->voltage, uv);

	if (ret < 0)
		return ret;
	*uv *= 1000;
	return 0;
}

/* temperature in 0.1 degrees Celsius */
static int mt6359_gauge_temp(struct mt6359_gauge *g, int *temp)
{
	int ret;

	if (!g->temperature)
		return -ENODATA;
	ret = iio_read_channel_processed(g->temperature, temp);
	if (ret < 0)
		return ret;
	*temp /= 100;
	return 0;
}

/* capacity in 0.01 % at counter value car, not clamped; lock held */
static s64 mt6359_gauge_cap(struct mt6359_gauge *g, s64 car)
{
	return div64_s64((car - g->car_zero) * 10000, g->full_lsb);
}

static int mt6359_gauge_cap_clamped(struct mt6359_gauge *g, s64 car)
{
	return clamp_t(s64, mt6359_gauge_cap(g, car), 0, 10000);
}

/* capacity in 0.01 % from the open circuit voltage estimate now */
static int mt6359_gauge_ocv_cap(struct mt6359_gauge *g,
				const struct mt6359_gauge_sample *s, bool log)
{
	int ua, uv, temp = 250, cap, ret;

	ret = mt6359_gauge_voltage(g, &uv);
	if (ret)
		return ret;
	mt6359_gauge_temp(g, &temp);
	ua = mt6359_gauge_ua(g, s->cic2);
	/* terminal voltage = OCV + I * R, I positive while charging */
	uv -= div_s64((s64)ua * g->info->factory_internal_resistance_uohm,
		      1000000);
	cap = power_supply_batinfo_ocv2cap(g->info, uv, temp / 10);
	if (cap < 0)
		return cap;
	if (log)
		dev_info(g->dev, "OCV estimate %d mV at %d mA, %d.%d C: %d %%\n",
			 uv / 1000, ua / 1000, temp / 10, abs(temp % 10), cap);
	return cap * 100;
}

static void mt6359_gauge_store_ref(struct mt6359_gauge *g)
{
	u32 v = (u32)g->car_zero;
	int ret;

	if (g->car_zero >= FG_REF_LIMIT || g->car_zero < -FG_REF_LIMIT) {
		dev_warn(g->dev, "reference %lld out of range, not stored\n",
			 g->car_zero);
		return;
	}
	ret = regmap_write(g->regmap, MT6359_SYSTEM_INFO_CON1, v & 0xffff);
	if (!ret)
		ret = regmap_write(g->regmap, MT6359_SYSTEM_INFO_CON2,
				   FIELD_PREP(FG_REF_MAGIC_MASK, FG_REF_MAGIC) |
				   FIELD_PREP(FG_REF_HI_MASK, v >> 16));
	if (ret)
		dev_warn(g->dev, "cannot store the reference: %d\n", ret);
}

static int mt6359_gauge_load_ref(struct mt6359_gauge *g, s64 *car_zero)
{
	unsigned int lo, hi;
	int ret;

	ret = regmap_read(g->regmap, MT6359_SYSTEM_INFO_CON1, &lo);
	if (!ret)
		ret = regmap_read(g->regmap, MT6359_SYSTEM_INFO_CON2, &hi);
	if (ret)
		return ret;
	if (FIELD_GET(FG_REF_MAGIC_MASK, hi) != FG_REF_MAGIC)
		return -ENOENT;
	*car_zero = sign_extend32(FIELD_GET(FG_REF_HI_MASK, hi) << 16 |
				  (lo & 0xffff), FG_REF_BITS - 1);
	return 0;
}

/* set the reference so that the capacity at counter value car is cap; lock held */
static void mt6359_gauge_anchor(struct mt6359_gauge *g, s64 car, int cap)
{
	g->car_zero = car - div_s64(g->full_lsb * cap, 10000);
	mt6359_gauge_store_ref(g);
}

/* initial reference at probe; lock held */
static int mt6359_gauge_init_ref(struct mt6359_gauge *g)
{
	struct mt6359_gauge_sample s;
	s64 car_zero, uptime_s;
	int ocv_cap, ref_cap, ret;
	const char *why;

	ret = mt6359_gauge_latch(g, &s, NULL);
	if (ret)
		return ret;
	g->win_car = s.car;
	g->win_t = ktime_get_boottime();
	ocv_cap = mt6359_gauge_ocv_cap(g, &s, true);

	uptime_s = ktime_divns(g->win_t, NSEC_PER_SEC);
	ret = mt6359_gauge_load_ref(g, &car_zero);
	if (ret) {
		why = "no stored reference";
	} else if ((s64)s.nter * FG_TICK_MS / 1000 < uptime_s + FG_RESET_MARGIN_S) {
		why = "counters were reset during this boot";
	} else {
		g->car_zero = car_zero;
		ref_cap = mt6359_gauge_cap(g, s.car);
		if (ocv_cap >= 0 && abs(ref_cap - ocv_cap) > FG_REF_MAX_DIFF) {
			dev_info(g->dev, "stored reference gives %d %%, OCV %d %%\n",
				 ref_cap / 100, ocv_cap / 100);
			why = "stored reference implausible";
		} else {
			dev_info(g->dev, "capacity %d.%02d %% from the stored reference (counters running for %u s)\n",
				 ref_cap / 100, abs(ref_cap % 100),
				 s.nter * FG_TICK_MS / 1000);
			return 0;
		}
	}

	if (ocv_cap < 0)
		return ocv_cap;
	dev_info(g->dev, "%s: capacity from the OCV estimate\n", why);
	mt6359_gauge_anchor(g, s.car, ocv_cap);
	return 0;
}

static int mt6359_gauge_get_status(struct mt6359_gauge *g)
{
	union power_supply_propval val;

	if (!power_supply_am_i_supplied(g->psy))
		return POWER_SUPPLY_STATUS_DISCHARGING;
	if (power_supply_get_property_from_supplier(g->psy, POWER_SUPPLY_PROP_STATUS,
						    &val))
		return POWER_SUPPLY_STATUS_CHARGING;
	switch (val.intval) {
	case POWER_SUPPLY_STATUS_FULL:
	case POWER_SUPPLY_STATUS_NOT_CHARGING:
	case POWER_SUPPLY_STATUS_CHARGING:
		return val.intval;
	default:
		return POWER_SUPPLY_STATUS_DISCHARGING;
	}
}

/* capacity in % as reported to user space; lock held */
static int mt6359_gauge_ui_cap(struct mt6359_gauge *g, s64 car)
{
	int cap = DIV_ROUND_CLOSEST(mt6359_gauge_cap_clamped(g, car), 100);

	/* 100 % only once the charger has finished */
	if (g->status == POWER_SUPPLY_STATUS_CHARGING)
		return min(cap, 99);
	return cap;
}

/* low battery: follow the voltage when it shows clearly less; lock held */
static void mt6359_gauge_check_low(struct mt6359_gauge *g,
				   const struct mt6359_gauge_sample *s)
{
	int cap = mt6359_gauge_cap_clamped(g, s->car);
	int vcap;

	/* also "not charging" with the input switched off or too weak */
	if (g->status == POWER_SUPPLY_STATUS_CHARGING ||
	    g->status == POWER_SUPPLY_STATUS_FULL || s->cic2 >= 0) {
		g->low_count = 0;
		return;
	}
	vcap = mt6359_gauge_ocv_cap(g, s, false);
	if (vcap < 0 || vcap >= FG_LOW_CAP || cap - vcap <= FG_LOW_DIFF) {
		g->low_count = 0;
		return;
	}
	if (++g->low_count < 2)
		return;
	dev_info(g->dev, "battery voltage shows %d %%, counter %d %%: following the voltage\n",
		 vcap / 100, cap / 100);
	mt6359_gauge_anchor(g, s->car, vcap);
	g->low_count = 0;
}

/*
 * The charger reports full: that is 100 %, as for the vendor gauge, unless the
 * voltage shows much less. The MT6360 ends charging at about 4.40 V while
 * 300..450 mA still flow into the battery (CV 4.45 V);
 * relaxed that is 4.37 V or 95 % in the OCV table. A full report from the warm
 * zone (CV 4.2 V) ends at about 80 % and is not taken.
 */
static void mt6359_gauge_check_full(struct mt6359_gauge *g,
				    const struct mt6359_gauge_sample *s)
{
	int vcap = mt6359_gauge_ocv_cap(g, s, false);

	if (vcap >= FG_FULL_CAP) {
		mt6359_gauge_anchor(g, s->car, 10000);
		dev_info(g->dev, "charger reports full: capacity 100 %%\n");
	} else {
		dev_info(g->dev, "charger reports full, but the voltage shows %d %%: capacity stays %d %%\n",
			 vcap / 100, mt6359_gauge_cap_clamped(g, s->car) / 100);
	}
}

static void mt6359_gauge_work(struct work_struct *work)
{
	struct mt6359_gauge *g = container_of(to_delayed_work(work),
					      struct mt6359_gauge, work);
	struct mt6359_gauge_sample s;
	bool changed = false, full = false;
	int status, cap;
	ktime_t t;
	s64 ms;

	status = mt6359_gauge_get_status(g);

	mutex_lock(&g->lock);
	if (status != g->status) {
		full = status == POWER_SUPPLY_STATUS_FULL;
		g->status = status;
		changed = true;
	}
	if (g->fg_on && !mt6359_gauge_latch(g, &s, NULL)) {
		if (full)
			mt6359_gauge_check_full(g, &s);
		mt6359_gauge_check_low(g, &s);

		t = ktime_get_boottime();
		ms = ktime_ms_delta(t, g->win_t);
		if (ms >= 1000) {
			g->avg_ua = div64_s64(mt6359_gauge_uah(g, s.car - g->win_car) *
					      3600 * 1000, ms);
			g->win_car = s.car;
			g->win_t = t;
		}

		cap = mt6359_gauge_ui_cap(g, s.car);
		if (cap != g->last_cap) {
			g->last_cap = cap;
			changed = true;
		}
	}
	mutex_unlock(&g->lock);

	if (changed)
		power_supply_changed(g->psy);
	queue_delayed_work(system_freezable_power_efficient_wq, &g->work,
			   msecs_to_jiffies(FG_POLL_MS));
}

static void mt6359_gauge_external_power_changed(struct power_supply *psy)
{
	struct mt6359_gauge *g = power_supply_get_drvdata(psy);

	mod_delayed_work(system_freezable_power_efficient_wq, &g->work, 0);
}

static const enum power_supply_property mt6359_gauge_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CHARGE_COUNTER,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX,
};

static int mt6359_gauge_get_property(struct power_supply *psy,
				     enum power_supply_property psp,
				     union power_supply_propval *val)
{
	struct mt6359_gauge *g = power_supply_get_drvdata(psy);
	struct mt6359_gauge_sample s;
	int ret = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		val->intval = g->status;
		return 0;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = 1;
		return 0;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		return mt6359_gauge_voltage(g, &val->intval);
	case POWER_SUPPLY_PROP_TEMP:
		return mt6359_gauge_temp(g, &val->intval);
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		val->intval = g->info->charge_full_design_uah;
		return 0;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
		return power_supply_battery_info_get_prop(g->info, psp, val);
	default:
		break;
	}

	if (!g->fg_on)
		return -ENODATA;

	mutex_lock(&g->lock);
	if (psp == POWER_SUPPLY_PROP_CURRENT_AVG) {
		val->intval = g->avg_ua;
		goto out;
	}
	ret = mt6359_gauge_latch(g, &s, NULL);
	if (ret)
		goto out;
	switch (psp) {
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		val->intval = mt6359_gauge_ua(g, s.cic2);
		break;
	case POWER_SUPPLY_PROP_CHARGE_COUNTER:
		/* the hardware counter, continuous across reboots and sleep */
		val->intval = mt6359_gauge_uah(g, s.car);
		break;
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		val->intval = div_s64((s64)mt6359_gauge_cap_clamped(g, s.car) *
				      g->info->charge_full_design_uah, 10000);
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		val->intval = mt6359_gauge_ui_cap(g, s.car);
		break;
	default:
		ret = -EINVAL;
	}
out:
	mutex_unlock(&g->lock);
	return ret;
}

static const struct power_supply_desc mt6359_gauge_desc = {
	.name = "battery",
	.type = POWER_SUPPLY_TYPE_BATTERY,
	.properties = mt6359_gauge_props,
	.num_properties = ARRAY_SIZE(mt6359_gauge_props),
	.get_property = mt6359_gauge_get_property,
	.external_power_changed = mt6359_gauge_external_power_changed,
};

static int mt6359_gauge_raw_show(struct seq_file *sf, void *data)
{
	struct mt6359_gauge *g = dev_get_drvdata(sf->private);
	struct mt6359_gauge_sample s;
	unsigned int con0;
	int offset, ret;

	mutex_lock(&g->lock);
	ret = regmap_read(g->regmap, MT6359_FGADC_CON0, &con0);
	if (!ret)
		ret = mt6359_gauge_latch(g, &s, &offset);
	mutex_unlock(&g->lock);
	if (ret)
		return ret;
	seq_printf(sf, "time %lld con0 0x%04x car %d nter %u cur %d cic2 %d offset %d car_zero %lld\n",
		   ktime_to_ms(ktime_get_boottime()), con0, s.car, s.nter, s.cur,
		   s.cic2, offset, g->car_zero);
	return 0;
}

static void mt6359_gauge_debugfs_remove(void *dir)
{
	debugfs_remove_recursive(dir);
}

static int mt6359_gauge_probe(struct platform_device *pdev)
{
	struct mt6397_chip *pmic = dev_get_drvdata(pdev->dev.parent);
	struct power_supply_config cfg = {};
	struct device *dev = &pdev->dev;
	struct mt6359_gauge *g;
	unsigned int con0;
	int ret;

	g = devm_kzalloc(dev, sizeof(*g), GFP_KERNEL);
	if (!g)
		return -ENOMEM;
	g->dev = dev;
	g->regmap = pmic ? pmic->regmap : NULL;
	if (!g->regmap)
		return dev_err_probe(dev, -ENODEV, "no PMIC regmap\n");
	g->status = POWER_SUPPLY_STATUS_UNKNOWN;
	g->last_cap = -1;
	ret = devm_mutex_init(dev, &g->lock);
	if (ret)
		return ret;
	platform_set_drvdata(pdev, g);

	g->shunt_uohm = FG_REF_SHUNT_UOHM;
	device_property_read_u32(dev, "shunt-resistor-micro-ohms", &g->shunt_uohm);
	if (!g->shunt_uohm)
		return dev_err_probe(dev, -EINVAL, "invalid shunt-resistor-micro-ohms\n");

	g->voltage = devm_iio_channel_get(dev, "voltage");
	if (IS_ERR(g->voltage))
		return dev_err_probe(dev, PTR_ERR(g->voltage), "no voltage channel\n");
	g->temperature = devm_iio_channel_get(dev, "temperature");
	if (IS_ERR(g->temperature)) {
		ret = PTR_ERR(g->temperature);
		if (ret != -ENODEV)
			return dev_err_probe(dev, ret, "no temperature channel\n");
		g->temperature = NULL;
	}

	ret = regmap_read(g->regmap, MT6359_FGADC_CON0, &con0);
	if (ret)
		return dev_err_probe(dev, ret, "cannot read FGADC_CON0\n");
	g->fg_on = con0 & FG_ON;
	if (!g->fg_on)
		dev_warn(dev, "gauge not enabled by the bootloader (FGADC_CON0 0x%04x): no current, charge or capacity\n",
			 con0);

	ret = devm_delayed_work_autocancel(dev, &g->work, mt6359_gauge_work);
	if (ret)
		return ret;

	cfg.drv_data = g;
	cfg.fwnode = dev_fwnode(dev);
	g->psy = devm_power_supply_register(dev, &mt6359_gauge_desc, &cfg);
	if (IS_ERR(g->psy))
		return dev_err_probe(dev, PTR_ERR(g->psy), "cannot register the power supply\n");

	ret = power_supply_get_battery_info(g->psy, &g->info);
	if (ret)
		return dev_err_probe(dev, ret, "no monitored battery\n");
	if (!power_supply_battery_info_has_prop(g->info,
						POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN) ||
	    !g->info->ocv_table_size[0])
		return dev_err_probe(dev, -EINVAL,
				     "battery needs a design capacity and an OCV table\n");
	if (g->info->factory_internal_resistance_uohm < 0)
		g->info->factory_internal_resistance_uohm = 0;
	g->full_lsb = mt6359_gauge_uah_to_lsb(g, g->info->charge_full_design_uah);

	if (g->fg_on) {
		mutex_lock(&g->lock);
		ret = mt6359_gauge_init_ref(g);
		mutex_unlock(&g->lock);
		if (ret)
			return dev_err_probe(dev, ret, "cannot read the gauge\n");
		g->debugfs = debugfs_create_dir("mt6359-gauge", NULL);
		debugfs_create_devm_seqfile(dev, "raw", g->debugfs,
					    mt6359_gauge_raw_show);
		/* registered last: the directory goes before the file data */
		ret = devm_add_action_or_reset(dev, mt6359_gauge_debugfs_remove,
					       g->debugfs);
		if (ret)
			return ret;
	}

	queue_delayed_work(system_freezable_power_efficient_wq, &g->work, 0);
	dev_info(dev, "sense resistor %u uOhm, design %d uAh\n", g->shunt_uohm,
		 g->info->charge_full_design_uah);
	return 0;
}

/* charge drawn while the system sleeps, between the noirq phases */
static int mt6359_gauge_suspend_noirq(struct device *dev)
{
	struct mt6359_gauge *g = dev_get_drvdata(dev);
	struct mt6359_gauge_sample s;

	if (!g->fg_on)
		return 0;
	mutex_lock(&g->lock);
	g->sleep_valid = !mt6359_gauge_latch(g, &s, NULL);
	if (g->sleep_valid) {
		g->sleep_car = s.car;
		g->sleep_t = ktime_get_boottime();
	}
	mutex_unlock(&g->lock);
	return 0;
}

static int mt6359_gauge_resume_noirq(struct device *dev)
{
	struct mt6359_gauge *g = dev_get_drvdata(dev);
	struct mt6359_gauge_sample s;
	s64 uah, ms;

	if (!g->fg_on || !g->sleep_valid)
		return 0;
	mutex_lock(&g->lock);
	g->sleep_valid = false;
	if (!mt6359_gauge_latch(g, &s, NULL)) {
		ms = ktime_ms_delta(ktime_get_boottime(), g->sleep_t);
		uah = mt6359_gauge_uah(g, (s64)s.car - g->sleep_car);
		if (ms > 0)
			dev_info(dev, "asleep %lld ms, %lld uAh, average %lld uA\n",
				 ms, uah, div64_s64(uah * 3600 * 1000, ms));
	}
	mutex_unlock(&g->lock);
	return 0;
}

static const struct dev_pm_ops mt6359_gauge_pm_ops = {
	NOIRQ_SYSTEM_SLEEP_PM_OPS(mt6359_gauge_suspend_noirq,
				  mt6359_gauge_resume_noirq)
};

static const struct of_device_id mt6359_gauge_of_match[] = {
	{ .compatible = "mediatek,mt6359-gauge" },
	{ }
};
MODULE_DEVICE_TABLE(of, mt6359_gauge_of_match);

static struct platform_driver mt6359_gauge_driver = {
	.driver = {
		.name = "mt6359-gauge",
		.of_match_table = mt6359_gauge_of_match,
		.pm = pm_sleep_ptr(&mt6359_gauge_pm_ops),
	},
	.probe = mt6359_gauge_probe,
};
module_platform_driver(mt6359_gauge_driver);

MODULE_DESCRIPTION("MediaTek MT6359 fuel gauge");
MODULE_LICENSE("GPL");
MODULE_IMPORT_NS("IIO_CONSUMER");
