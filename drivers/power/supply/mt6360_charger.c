// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2021 MediaTek Inc.
 */

#include <linux/devm-helpers.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/linear_range.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/property.h>
#include <linux/regmap.h>
#include <linux/regulator/driver.h>
#include <linux/thermal.h>

#define MT6360_PMU_CHG_CTRL1	0x311
#define MT6360_PMU_CHG_CTRL2	0x312
#define MT6360_PMU_CHG_CTRL3	0x313
#define MT6360_PMU_CHG_CTRL4	0x314
#define MT6360_PMU_CHG_CTRL5	0x315
#define MT6360_PMU_CHG_CTRL6	0x316
#define MT6360_PMU_CHG_CTRL7	0x317
#define MT6360_PMU_CHG_CTRL8	0x318
#define MT6360_PMU_CHG_CTRL9	0x319
#define MT6360_PMU_CHG_CTRL10	0x31A
#define MT6360_PMU_CHG_CTRL12	0x31C
#define MT6360_PMU_CHG_CTRL13	0x31D
#define MT6360_PMU_DEVICE_TYPE	0x322
#define MT6360_PMU_USB_STATUS1	0x327
#define MT6360_PMU_CHG_STAT	0x34A
#define MT6360_PMU_CHG_CTRL19	0x361
#define MT6360_PMU_FOD_STAT	0x3E7

/* MT6360_PMU_CHG_CTRL1 */
#define MT6360_FSLP_SHFT	(3)
#define MT6360_FSLP_MASK	BIT(MT6360_FSLP_SHFT)
#define MT6360_OPA_MODE_SHFT	(0)
#define MT6360_OPA_MODE_MASK	BIT(MT6360_OPA_MODE_SHFT)
/* MT6360_PMU_CHG_CTRL2 */
#define MT6360_IINLMTSEL_SHFT	(2)
#define MT6360_IINLMTSEL_MASK	GENMASK(3, 2)
#define MT6360_TE_EN_MASK	BIT(4)
#define MT6360_CHG_EN_MASK	BIT(0)
/* MT6360_PMU_CHG_CTRL3 */
#define MT6360_IAICR_SHFT	(2)
#define MT6360_IAICR_MASK	GENMASK(7, 2)
#define MT6360_ILIM_EN_MASK	BIT(0)
/* MT6360_PMU_CHG_CTRL4 */
#define MT6360_VOREG_SHFT	(1)
#define MT6360_VOREG_MASK	GENMASK(7, 1)
/* MT6360_PMU_CHG_CTRL5 */
#define MT6360_VOBST_MASK	GENMASK(7, 2)
/* MT6360_PMU_CHG_CTRL6 */
#define MT6360_VMIVR_SHFT      (1)
#define MT6360_VMIVR_MASK      GENMASK(7, 1)
/* MT6360_PMU_CHG_CTRL7 */
#define MT6360_ICHG_SHFT	(2)
#define MT6360_ICHG_MASK	GENMASK(7, 2)
/* MT6360_PMU_CHG_CTRL8 */
#define MT6360_IPREC_SHFT	(0)
#define MT6360_IPREC_MASK	GENMASK(3, 0)
/* MT6360_PMU_CHG_CTRL9 */
#define MT6360_IEOC_SHFT	(4)
#define MT6360_IEOC_MASK	GENMASK(7, 4)
/* MT6360_PMU_CHG_CTRL10 */
#define MT6360_OTG_OC_MASK	GENMASK(3, 0)
#define MT6360_OTG_OC_SEL_MASK	GENMASK(2, 0)
/* MT6360_PMU_CHG_CTRL12 */
#define MT6360_WT_FC_SHFT	(5)
#define MT6360_WT_FC_MASK	GENMASK(7, 5)
#define MT6360_TMR_EN_MASK	BIT(1)
/* MT6360_PMU_CHG_CTRL13 */
#define MT6360_CHG_WDT_EN_MASK	BIT(7)
/* MT6360_PMU_DEVICE_TYPE */
#define MT6360_USBCHGEN_MASK	BIT(7)
/* MT6360_PMU_USB_STATUS1 */
#define MT6360_USB_STATUS_SHFT	(4)
#define MT6360_USB_STATUS_MASK	GENMASK(6, 4)
/* MT6360_PMU_CHG_STAT */
#define MT6360_CHG_STAT_SHFT	(6)
#define MT6360_CHG_STAT_MASK	GENMASK(7, 6)
#define MT6360_VBAT_LVL_MASK	BIT(5)
/* MT6360_PMU_CHG_CTRL19 */
#define MT6360_VINOVP_SHFT	(5)
#define MT6360_VINOVP_MASK	GENMASK(6, 5)
/* MT6360_PMU_FOD_STAT */
#define MT6360_CHRDET_EXT_MASK	BIT(4)

/* uV */
#define MT6360_VMIVR_MIN	3900000
#define MT6360_VMIVR_MAX	13400000
#define MT6360_VMIVR_STEP	100000
/* uA */
#define MT6360_ICHG_MIN		100000
#define MT6360_ICHG_MAX		5000000
#define MT6360_ICHG_STEP	100000
/* uV */
#define MT6360_VOREG_MIN	3900000
#define MT6360_VOREG_MAX	4710000
#define MT6360_VOREG_STEP	10000
/* uA */
#define MT6360_AICR_MIN		100000
#define MT6360_AICR_MAX		3250000
#define MT6360_AICR_STEP	50000
/* uA */
#define MT6360_IPREC_MIN	100000
#define MT6360_IPREC_MAX	850000
#define MT6360_IPREC_STEP	50000
/* uA */
#define MT6360_IEOC_MIN		100000
#define MT6360_IEOC_MAX		850000
#define MT6360_IEOC_STEP	50000

enum {
	MT6360_RANGE_VMIVR,
	MT6360_RANGE_ICHG,
	MT6360_RANGE_VOREG,
	MT6360_RANGE_AICR,
	MT6360_RANGE_IPREC,
	MT6360_RANGE_IEOC,
	MT6360_RANGE_MAX,
};

static const struct linear_range mt6360_chg_range[MT6360_RANGE_MAX] = {
	LINEAR_RANGE_IDX(MT6360_RANGE_VMIVR, 3900000, 0, 0x5F, 100000),
	LINEAR_RANGE_IDX(MT6360_RANGE_ICHG, 100000, 0, 0x31, 100000),
	LINEAR_RANGE_IDX(MT6360_RANGE_VOREG, 3900000, 0, 0x51, 10000),
	LINEAR_RANGE_IDX(MT6360_RANGE_AICR, 100000, 0, 0x3F, 50000),
	LINEAR_RANGE_IDX(MT6360_RANGE_IPREC, 100000, 0, 0x0F, 50000),
	LINEAR_RANGE_IDX(MT6360_RANGE_IEOC, 100000, 0, 0x0F, 50000),
};

/*
 * Battery temperature zones of the guard. COOL and WARM are the alert ranges
 * of the battery (between temp_min and temp_alert_min, between temp_alert_max
 * and temp_max), where charging continues with reduced limits. UNKNOWN is the
 * state before the first and after a failed temperature reading.
 */
enum mt6360_chg_zone {
	MT6360_ZONE_COLD,
	MT6360_ZONE_COOL,
	MT6360_ZONE_NORMAL,
	MT6360_ZONE_WARM,
	MT6360_ZONE_HOT,
	MT6360_ZONE_UNKNOWN,
	MT6360_ZONE_CNT,
};

static const char * const mt6360_chg_zone_name[MT6360_ZONE_CNT] = {
	"cold", "cool", "normal", "warm", "hot", "unknown",
};

/* Charge limits of a zone; ichg 0 means charging is disabled (CHG_EN off) */
struct mt6360_chg_limit {
	u32 ichg;
	u32 cv;
};

struct mt6360_chg_info {
	struct device *dev;
	struct regmap *regmap;
	struct power_supply_desc psy_desc;
	struct power_supply *psy;
	struct regulator_dev *otg_rdev;
	struct mutex chgdet_lock;
	u32 vinovp;
	bool pwr_rdy;
	bool bc12_en;
	bool otg_on;
	int psy_usb_type;
	struct work_struct chrdet_work;
	/* optional battery temperature guard */
	struct power_supply_battery_info *bat_info;
	struct thermal_zone_device *bat_tz;
	struct delayed_work guard_work;
	bool chg_inhibit;
	bool tz_failed;
	/* zone boundaries in mC */
	int tmin_mc;
	int tmax_mc;
	int cool_max_mc;
	int warm_min_mc;
	/* charge limits: battery maximum, request (sysfs) and zone limits */
	struct mutex lim_lock;
	u32 ichg_max;
	u32 cv_max;
	u32 ichg_req;
	u32 cv_req;
	struct mt6360_chg_limit zone_lim[MT6360_ZONE_CNT];
	enum mt6360_chg_zone zone;
	/* optional input source (USB Type-C port) */
	struct fwnode_handle *src_fwnode;
	struct notifier_block psy_nb;
	struct work_struct src_work;
	u32 aicr_def;
	u32 mivr_def;
	u32 src_ua;
	u32 src_uv;
};

enum mt6360_iinlmtsel {
	MT6360_IINLMTSEL_AICR_3250 = 0,
	MT6360_IINLMTSEL_CHG_TYPE,
	MT6360_IINLMTSEL_AICR,
	MT6360_IINLMTSEL_LOWER_LEVEL,
};

enum mt6360_pmu_chg_type {
	MT6360_CHG_TYPE_NOVBUS = 0,
	MT6360_CHG_TYPE_UNDER_GOING,
	MT6360_CHG_TYPE_SDP,
	MT6360_CHG_TYPE_SDPNSTD,
	MT6360_CHG_TYPE_DCP,
	MT6360_CHG_TYPE_CDP,
	MT6360_CHG_TYPE_DISABLE_BC12,
	MT6360_CHG_TYPE_MAX,
};

static int mt6360_get_chrdet_ext_stat(struct mt6360_chg_info *mci,
					     bool *pwr_rdy)
{
	int ret;
	unsigned int regval;

	ret = regmap_read(mci->regmap, MT6360_PMU_FOD_STAT, &regval);
	if (ret < 0)
		return ret;
	*pwr_rdy = (regval & MT6360_CHRDET_EXT_MASK) ? true : false;
	return 0;
}

static int mt6360_charger_get_online(struct mt6360_chg_info *mci,
				     union power_supply_propval *val)
{
	unsigned int ctrl1;
	int ret;
	bool pwr_rdy;

	ret = mt6360_get_chrdet_ext_stat(mci, &pwr_rdy);
	if (ret < 0)
		return ret;
	/* VBUS from the own OTG boost is not a charger */
	ret = regmap_read(mci->regmap, MT6360_PMU_CHG_CTRL1, &ctrl1);
	if (ret < 0)
		return ret;
	val->intval = pwr_rdy && !(ctrl1 & MT6360_OPA_MODE_MASK);
	return 0;
}

static int mt6360_charger_get_status(struct mt6360_chg_info *mci,
				     union power_supply_propval *val)
{
	int status, ret;
	unsigned int regval;
	bool pwr_rdy;

	ret = mt6360_get_chrdet_ext_stat(mci, &pwr_rdy);
	if (ret < 0)
		return ret;
	if (!pwr_rdy) {
		status = POWER_SUPPLY_STATUS_DISCHARGING;
		goto out;
	}

	if (mci->chg_inhibit) {
		status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		goto out;
	}

	ret = regmap_read(mci->regmap, MT6360_PMU_CHG_STAT, &regval);
	if (ret < 0)
		return ret;
	regval &= MT6360_CHG_STAT_MASK;
	regval >>= MT6360_CHG_STAT_SHFT;
	switch (regval) {
	case 0x0:
		status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		break;
	case 0x1:
		status = POWER_SUPPLY_STATUS_CHARGING;
		break;
	case 0x2:
		status = POWER_SUPPLY_STATUS_FULL;
		break;
	default:
		ret = -EIO;
	}
out:
	if (!ret)
		val->intval = status;
	return ret;
}

static int mt6360_charger_get_charge_type(struct mt6360_chg_info *mci,
					  union power_supply_propval *val)
{
	int type, ret;
	unsigned int regval;
	u8 chg_stat;

	ret = regmap_read(mci->regmap, MT6360_PMU_CHG_STAT, &regval);
	if (ret < 0)
		return ret;

	chg_stat = (regval & MT6360_CHG_STAT_MASK) >> MT6360_CHG_STAT_SHFT;
	switch (chg_stat) {
	case 0x01: /* Charge in Progress */
		if (regval & MT6360_VBAT_LVL_MASK)
			type = POWER_SUPPLY_CHARGE_TYPE_FAST;
		else
			type = POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
		break;
	case 0x00: /* Not Charging */
	case 0x02: /* Charge Done */
	case 0x03: /* Charge Fault */
	default:
		type = POWER_SUPPLY_CHARGE_TYPE_NONE;
		break;
	}

	val->intval = type;
	return 0;
}

static int mt6360_charger_get_ichg(struct mt6360_chg_info *mci,
				   union power_supply_propval *val)
{
	int ret;
	u32 sel, value;

	ret = regmap_read(mci->regmap, MT6360_PMU_CHG_CTRL7, &sel);
	if (ret < 0)
		return ret;
	sel = (sel & MT6360_ICHG_MASK) >> MT6360_ICHG_SHFT;
	ret = linear_range_get_value(&mt6360_chg_range[MT6360_RANGE_ICHG], sel, &value);
	if (!ret)
		val->intval = value;
	return ret;
}

static int mt6360_charger_get_max_ichg(struct mt6360_chg_info *mci,
				       union power_supply_propval *val)
{
	val->intval = mci->ichg_max;
	return 0;
}

static int mt6360_charger_get_cv(struct mt6360_chg_info *mci,
				 union power_supply_propval *val)
{
	int ret;
	u32 sel, value;

	ret = regmap_read(mci->regmap, MT6360_PMU_CHG_CTRL4, &sel);
	if (ret < 0)
		return ret;
	sel = (sel & MT6360_VOREG_MASK) >> MT6360_VOREG_SHFT;
	ret = linear_range_get_value(&mt6360_chg_range[MT6360_RANGE_VOREG], sel, &value);
	if (!ret)
		val->intval = value;
	return ret;
}

static int mt6360_charger_get_max_cv(struct mt6360_chg_info *mci,
				     union power_supply_propval *val)
{
	val->intval = mci->cv_max;
	return 0;
}

static int mt6360_charger_get_aicr(struct mt6360_chg_info *mci,
				   union power_supply_propval *val)
{
	int ret;
	u32 sel, value;

	ret = regmap_read(mci->regmap, MT6360_PMU_CHG_CTRL3, &sel);
	if (ret < 0)
		return ret;
	sel = (sel & MT6360_IAICR_MASK) >> MT6360_IAICR_SHFT;
	ret = linear_range_get_value(&mt6360_chg_range[MT6360_RANGE_AICR], sel, &value);
	if (!ret)
		val->intval = value;
	return ret;
}

static int mt6360_charger_get_mivr(struct mt6360_chg_info *mci,
				   union power_supply_propval *val)
{
	int ret;
	u32 sel, value;

	ret = regmap_read(mci->regmap, MT6360_PMU_CHG_CTRL6, &sel);
	if (ret < 0)
		return ret;
	sel = (sel & MT6360_VMIVR_MASK) >> MT6360_VMIVR_SHFT;
	ret = linear_range_get_value(&mt6360_chg_range[MT6360_RANGE_VMIVR], sel, &value);
	if (!ret)
		val->intval = value;
	return ret;
}

static int mt6360_charger_get_iprechg(struct mt6360_chg_info *mci,
				      union power_supply_propval *val)
{
	int ret;
	u32 sel, value;

	ret = regmap_read(mci->regmap, MT6360_PMU_CHG_CTRL8, &sel);
	if (ret < 0)
		return ret;
	sel = (sel & MT6360_IPREC_MASK) >> MT6360_IPREC_SHFT;
	ret = linear_range_get_value(&mt6360_chg_range[MT6360_RANGE_IPREC], sel, &value);
	if (!ret)
		val->intval = value;
	return ret;
}

static int mt6360_charger_get_ieoc(struct mt6360_chg_info *mci,
				   union power_supply_propval *val)
{
	int ret;
	u32 sel, value;

	ret = regmap_read(mci->regmap, MT6360_PMU_CHG_CTRL9, &sel);
	if (ret < 0)
		return ret;
	sel = (sel & MT6360_IEOC_MASK) >> MT6360_IEOC_SHFT;
	ret = linear_range_get_value(&mt6360_chg_range[MT6360_RANGE_IEOC], sel, &value);
	if (!ret)
		val->intval = value;
	return ret;
}

static int mt6360_charger_set_online(struct mt6360_chg_info *mci,
				     const union power_supply_propval *val)
{
	u8 force_sleep = val->intval ? 0 : 1;

	return regmap_update_bits(mci->regmap,
				  MT6360_PMU_CHG_CTRL1,
				  MT6360_FSLP_MASK,
				  force_sleep << MT6360_FSLP_SHFT);
}

static int mt6360_charger_set_ichg(struct mt6360_chg_info *mci,
				   const union power_supply_propval *val)
{
	u32 sel;

	linear_range_get_selector_within(&mt6360_chg_range[MT6360_RANGE_ICHG], val->intval, &sel);
	return regmap_update_bits(mci->regmap,
				  MT6360_PMU_CHG_CTRL7,
				  MT6360_ICHG_MASK,
				  sel << MT6360_ICHG_SHFT);
}

static int mt6360_charger_set_cv(struct mt6360_chg_info *mci,
				 const union power_supply_propval *val)
{
	u32 sel;

	linear_range_get_selector_within(&mt6360_chg_range[MT6360_RANGE_VOREG], val->intval, &sel);
	return regmap_update_bits(mci->regmap,
				  MT6360_PMU_CHG_CTRL4,
				  MT6360_VOREG_MASK,
				  sel << MT6360_VOREG_SHFT);
}

static int mt6360_charger_set_aicr(struct mt6360_chg_info *mci,
				   const union power_supply_propval *val)
{
	u32 sel;

	linear_range_get_selector_within(&mt6360_chg_range[MT6360_RANGE_AICR], val->intval, &sel);
	return regmap_update_bits(mci->regmap,
				  MT6360_PMU_CHG_CTRL3,
				  MT6360_IAICR_MASK,
				  sel << MT6360_IAICR_SHFT);
}

static int mt6360_charger_set_mivr(struct mt6360_chg_info *mci,
				   const union power_supply_propval *val)
{
	u32 sel;

	linear_range_get_selector_within(&mt6360_chg_range[MT6360_RANGE_VMIVR], val->intval, &sel);
	return regmap_update_bits(mci->regmap,
				  MT6360_PMU_CHG_CTRL6,
				  MT6360_VMIVR_MASK,
				  sel << MT6360_VMIVR_SHFT);
}

static int mt6360_charger_set_iprechg(struct mt6360_chg_info *mci,
				      const union power_supply_propval *val)
{
	u32 sel;

	linear_range_get_selector_within(&mt6360_chg_range[MT6360_RANGE_IPREC], val->intval, &sel);
	return regmap_update_bits(mci->regmap,
				  MT6360_PMU_CHG_CTRL8,
				  MT6360_IPREC_MASK,
				  sel << MT6360_IPREC_SHFT);
}

static int mt6360_charger_set_ieoc(struct mt6360_chg_info *mci,
				   const union power_supply_propval *val)
{
	u32 sel;

	linear_range_get_selector_within(&mt6360_chg_range[MT6360_RANGE_IEOC], val->intval, &sel);
	return regmap_update_bits(mci->regmap,
				  MT6360_PMU_CHG_CTRL9,
				  MT6360_IEOC_MASK,
				  sel << MT6360_IEOC_SHFT);
}

/* ICHG and CV in effect: the request, capped by the zone (off zones keep it) */
static u32 mt6360_chg_eff_ichg(struct mt6360_chg_info *mci)
{
	u32 lim = mci->zone_lim[mci->zone].ichg;

	return lim ? min(mci->ichg_req, lim) : mci->ichg_req;
}

static u32 mt6360_chg_eff_cv(struct mt6360_chg_info *mci)
{
	u32 lim = mci->zone_lim[mci->zone].cv;

	return lim ? min(mci->cv_req, lim) : mci->cv_req;
}

/* Program ICHG and CV in effect; caller holds lim_lock */
static int mt6360_chg_write_limits(struct mt6360_chg_info *mci)
{
	union power_supply_propval val;
	int ret;

	lockdep_assert_held(&mci->lim_lock);

	val.intval = mt6360_chg_eff_cv(mci);
	ret = mt6360_charger_set_cv(mci, &val);
	if (ret)
		return ret;
	val.intval = mt6360_chg_eff_ichg(mci);
	return mt6360_charger_set_ichg(mci, &val);
}

/*
 * Charge current and voltage requests from user space are kept within the
 * battery limits and, through mt6360_chg_write_limits(), within the limits
 * of the current temperature zone.
 */
static int mt6360_chg_request_ichg(struct mt6360_chg_info *mci,
				   const union power_supply_propval *val)
{
	int ret;

	mutex_lock(&mci->lim_lock);
	mci->ichg_req = clamp_val(val->intval, MT6360_ICHG_MIN, mci->ichg_max);
	ret = mt6360_chg_write_limits(mci);
	mutex_unlock(&mci->lim_lock);
	return ret;
}

static int mt6360_chg_request_cv(struct mt6360_chg_info *mci,
				 const union power_supply_propval *val)
{
	int ret;

	mutex_lock(&mci->lim_lock);
	mci->cv_req = clamp_val(val->intval, MT6360_VOREG_MIN, mci->cv_max);
	ret = mt6360_chg_write_limits(mci);
	mutex_unlock(&mci->lim_lock);
	return ret;
}

/* Input current requests stay within the current limit of the source, if known */
static int mt6360_chg_request_aicr(struct mt6360_chg_info *mci,
				   const union power_supply_propval *val)
{
	union power_supply_propval aicr = *val;
	int ret;

	mutex_lock(&mci->lim_lock);
	if (mci->src_ua && aicr.intval > mci->src_ua)
		aicr.intval = mci->src_ua;
	ret = mt6360_charger_set_aicr(mci, &aicr);
	mutex_unlock(&mci->lim_lock);
	return ret;
}

static int mt6360_charger_get_property(struct power_supply *psy,
				       enum power_supply_property psp,
				       union power_supply_propval *val)
{
	struct mt6360_chg_info *mci = power_supply_get_drvdata(psy);
	int ret = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		ret = mt6360_charger_get_online(mci, val);
		break;
	case POWER_SUPPLY_PROP_STATUS:
		ret = mt6360_charger_get_status(mci, val);
		break;
	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		ret = mt6360_charger_get_charge_type(mci, val);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		ret = mt6360_charger_get_ichg(mci, val);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX:
		ret = mt6360_charger_get_max_ichg(mci, val);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		ret = mt6360_charger_get_cv(mci, val);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX:
		ret = mt6360_charger_get_max_cv(mci, val);
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		ret = mt6360_charger_get_aicr(mci, val);
		break;
	case POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT:
		ret = mt6360_charger_get_mivr(mci, val);
		break;
	case POWER_SUPPLY_PROP_PRECHARGE_CURRENT:
		ret = mt6360_charger_get_iprechg(mci, val);
		break;
	case POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT:
		ret = mt6360_charger_get_ieoc(mci, val);
		break;
	case POWER_SUPPLY_PROP_USB_TYPE:
		val->intval = mci->psy_usb_type;
		break;
	default:
		ret = -ENODATA;
	}
	return ret;
}

static int mt6360_charger_set_property(struct power_supply *psy,
				       enum power_supply_property psp,
				       const union power_supply_propval *val)
{
	struct mt6360_chg_info *mci = power_supply_get_drvdata(psy);
	int ret;

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
		ret = mt6360_charger_set_online(mci, val);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
		ret = mt6360_chg_request_ichg(mci, val);
		break;
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
		ret = mt6360_chg_request_cv(mci, val);
		break;
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
		ret = mt6360_chg_request_aicr(mci, val);
		break;
	case POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT:
		ret = mt6360_charger_set_mivr(mci, val);
		break;
	case POWER_SUPPLY_PROP_PRECHARGE_CURRENT:
		ret = mt6360_charger_set_iprechg(mci, val);
		break;
	case POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT:
		ret = mt6360_charger_set_ieoc(mci, val);
		break;
	default:
		ret = -EINVAL;
	}
	return ret;
}

static int mt6360_charger_property_is_writeable(struct power_supply *psy,
					       enum power_supply_property psp)
{
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT:
	case POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE:
	case POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT:
	case POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT:
	case POWER_SUPPLY_PROP_PRECHARGE_CURRENT:
	case POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT:
		return 1;
	default:
		return 0;
	}
}

static enum power_supply_property mt6360_charger_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_CURRENT_MAX,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE,
	POWER_SUPPLY_PROP_CONSTANT_CHARGE_VOLTAGE_MAX,
	POWER_SUPPLY_PROP_INPUT_CURRENT_LIMIT,
	POWER_SUPPLY_PROP_INPUT_VOLTAGE_LIMIT,
	POWER_SUPPLY_PROP_PRECHARGE_CURRENT,
	POWER_SUPPLY_PROP_CHARGE_TERM_CURRENT,
	POWER_SUPPLY_PROP_USB_TYPE,
};

static const struct power_supply_desc mt6360_charger_desc = {
	.type			= POWER_SUPPLY_TYPE_USB,
	.properties		= mt6360_charger_properties,
	.num_properties		= ARRAY_SIZE(mt6360_charger_properties),
	.get_property		= mt6360_charger_get_property,
	.set_property		= mt6360_charger_set_property,
	.property_is_writeable	= mt6360_charger_property_is_writeable,
	.usb_types		= BIT(POWER_SUPPLY_USB_TYPE_SDP) |
				  BIT(POWER_SUPPLY_USB_TYPE_CDP) |
				  BIT(POWER_SUPPLY_USB_TYPE_DCP) |
				  BIT(POWER_SUPPLY_USB_TYPE_UNKNOWN),
};

/*
 * OTG boost: no BC1.2 detection on the own output, the D+/D- lines belong to
 * the USB host then. The VBUS interrupt arms the detection again once the
 * boost is off and an external source appears.
 */
static int mt6360_chg_otg_enable(struct regulator_dev *rdev)
{
	struct mt6360_chg_info *mci = rdev_get_drvdata(rdev);
	int ret;

	mutex_lock(&mci->chgdet_lock);
	ret = regmap_update_bits(mci->regmap, MT6360_PMU_DEVICE_TYPE, MT6360_USBCHGEN_MASK, 0);
	if (!ret) {
		mci->bc12_en = false;
		mci->otg_on = true;
		ret = regulator_enable_regmap(rdev);
		if (ret)
			mci->otg_on = false;
	}
	mutex_unlock(&mci->chgdet_lock);
	return ret;
}

static int mt6360_chg_otg_disable(struct regulator_dev *rdev)
{
	struct mt6360_chg_info *mci = rdev_get_drvdata(rdev);
	int ret;

	mutex_lock(&mci->chgdet_lock);
	ret = regulator_disable_regmap(rdev);
	if (!ret)
		mci->otg_on = false;
	mutex_unlock(&mci->chgdet_lock);
	return ret;
}

/* OTG over-current thresholds (CHG_CTRL10 bits 2:0), vendor driver table */
static const unsigned int mt6360_otg_oc_table[] = {
	500000, 700000, 1100000, 1300000, 1800000, 2100000, 2400000, 3000000,
};

static const struct regulator_ops mt6360_chg_otg_ops = {
	.list_voltage = regulator_list_voltage_linear,
	.enable = mt6360_chg_otg_enable,
	.disable = mt6360_chg_otg_disable,
	.is_enabled = regulator_is_enabled_regmap,
	.set_voltage_sel = regulator_set_voltage_sel_regmap,
	.get_voltage_sel = regulator_get_voltage_sel_regmap,
	.set_current_limit = regulator_set_current_limit_regmap,
	.get_current_limit = regulator_get_current_limit_regmap,
};

static const struct regulator_desc mt6360_otg_rdesc = {
	.of_match = "usb-otg-vbus-regulator",
	.name = "usb-otg-vbus",
	.ops = &mt6360_chg_otg_ops,
	.owner = THIS_MODULE,
	.type = REGULATOR_VOLTAGE,
	.min_uV = 4425000,
	.uV_step = 25000,
	.n_voltages = 57,
	.vsel_reg = MT6360_PMU_CHG_CTRL5,
	.vsel_mask = MT6360_VOBST_MASK,
	.enable_reg = MT6360_PMU_CHG_CTRL1,
	.enable_mask = MT6360_OPA_MODE_MASK,
	.curr_table = mt6360_otg_oc_table,
	.n_current_limits = ARRAY_SIZE(mt6360_otg_oc_table),
	.csel_reg = MT6360_PMU_CHG_CTRL10,
	.csel_mask = MT6360_OTG_OC_SEL_MASK,
};

static void mt6360_chg_src_set(struct mt6360_chg_info *mci, u32 src_ua, u32 src_uv, bool force);
static u32 mt6360_chg_bc12_aicr(struct mt6360_chg_info *mci);

static irqreturn_t mt6360_pmu_attach_i_handler(int irq, void *data)
{
	struct mt6360_chg_info *mci = data;
	int ret;
	unsigned int usb_status;
	int last_usb_type;

	mutex_lock(&mci->chgdet_lock);
	if (!mci->bc12_en) {
		dev_warn(mci->dev, "Received attach interrupt, bc12 disabled, ignore irq\n");
		goto out;
	}
	last_usb_type = mci->psy_usb_type;
	/* Plug in */
	ret = regmap_read(mci->regmap, MT6360_PMU_USB_STATUS1, &usb_status);
	if (ret < 0)
		goto out;
	usb_status &= MT6360_USB_STATUS_MASK;
	usb_status >>= MT6360_USB_STATUS_SHFT;
	switch (usb_status) {
	case MT6360_CHG_TYPE_NOVBUS:
		dev_dbg(mci->dev, "Received attach interrupt, no vbus\n");
		goto out;
	case MT6360_CHG_TYPE_UNDER_GOING:
		dev_dbg(mci->dev, "Received attach interrupt, under going...\n");
		goto out;
	case MT6360_CHG_TYPE_SDP:
		mci->psy_usb_type = POWER_SUPPLY_USB_TYPE_SDP;
		break;
	case MT6360_CHG_TYPE_SDPNSTD:
		mci->psy_usb_type = POWER_SUPPLY_USB_TYPE_SDP;
		break;
	case MT6360_CHG_TYPE_CDP:
		mci->psy_usb_type = POWER_SUPPLY_USB_TYPE_CDP;
		break;
	case MT6360_CHG_TYPE_DCP:
		mci->psy_usb_type = POWER_SUPPLY_USB_TYPE_DCP;
		break;
	case MT6360_CHG_TYPE_DISABLE_BC12:
		dev_dbg(mci->dev, "Received attach interrupt, bc12 detect not enable\n");
		goto out;
	default:
		mci->psy_usb_type = POWER_SUPPLY_USB_TYPE_UNKNOWN;
		dev_dbg(mci->dev, "Received attach interrupt, reserved address\n");
		goto out;
	}

	dev_dbg(mci->dev, "Received attach interrupt, chg_type = %d\n", mci->psy_usb_type);
	if (last_usb_type != mci->psy_usb_type) {
		/* without a Type-C current the input limit follows the BC1.2 result */
		mutex_lock(&mci->lim_lock);
		mt6360_chg_src_set(mci, mci->src_ua, mci->src_uv, true);
		if (!mci->src_ua)
			dev_info(mci->dev, "BC1.2 type %d: input current limit %u uA\n",
				 mci->psy_usb_type, mt6360_chg_bc12_aicr(mci));
		mutex_unlock(&mci->lim_lock);
		power_supply_changed(mci->psy);
	}
out:
	mutex_unlock(&mci->chgdet_lock);
	return IRQ_HANDLED;
}

static void mt6360_handle_chrdet_ext_evt(struct mt6360_chg_info *mci)
{
	int ret;
	bool pwr_rdy, bc12;

	mutex_lock(&mci->chgdet_lock);
	ret = mt6360_get_chrdet_ext_stat(mci, &pwr_rdy);
	if (ret < 0)
		goto out;
	if (mci->pwr_rdy == pwr_rdy) {
		dev_dbg(mci->dev, "Received vbus interrupt, pwr_rdy is same(%d)\n", pwr_rdy);
		goto out;
	}
	mci->pwr_rdy = pwr_rdy;
	dev_dbg(mci->dev, "Received vbus interrupt, pwr_rdy = %d\n", pwr_rdy);
	if (!pwr_rdy) {
		mci->psy_usb_type = POWER_SUPPLY_USB_TYPE_UNKNOWN;
		/* source gone: back to the default input current and MIVR */
		mutex_lock(&mci->lim_lock);
		mt6360_chg_src_set(mci, 0, 0, true);
		mutex_unlock(&mci->lim_lock);
		power_supply_changed(mci->psy);

	}
	bc12 = pwr_rdy && !mci->otg_on;
	ret = regmap_update_bits(mci->regmap,
				 MT6360_PMU_DEVICE_TYPE,
				 MT6360_USBCHGEN_MASK,
				 bc12 ? MT6360_USBCHGEN_MASK : 0);
	if (ret < 0)
		goto out;
	mci->bc12_en = bc12;
out:
	mutex_unlock(&mci->chgdet_lock);
}

static void mt6360_chrdet_work(struct work_struct *work)
{
	struct mt6360_chg_info *mci = (struct mt6360_chg_info *)container_of(
				     work, struct mt6360_chg_info, chrdet_work);

	mt6360_handle_chrdet_ext_evt(mci);
}

static irqreturn_t mt6360_pmu_chrdet_ext_evt_handler(int irq, void *data)
{
	struct mt6360_chg_info *mci = data;

	mt6360_handle_chrdet_ext_evt(mci);
	return IRQ_HANDLED;
}

static int mt6360_chg_irq_register(struct platform_device *pdev)
{
	const struct {
		const char *name;
		irq_handler_t handler;
	} irq_descs[] = {
		{ "attach_i", mt6360_pmu_attach_i_handler },
		{ "chrdet_ext_evt", mt6360_pmu_chrdet_ext_evt_handler }
	};
	int i, ret;

	for (i = 0; i < ARRAY_SIZE(irq_descs); i++) {
		ret = platform_get_irq_byname(pdev, irq_descs[i].name);
		if (ret < 0)
			return ret;

		ret = devm_request_threaded_irq(&pdev->dev, ret, NULL,
						irq_descs[i].handler,
						IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
						irq_descs[i].name,
						platform_get_drvdata(pdev));
		if (ret < 0)
			return ret;
	}

	return 0;
}

static u32 mt6360_vinovp_trans_to_sel(u32 val)
{
	u32 vinovp_tbl[] = { 5500000, 6500000, 11000000, 14500000 };
	int i;

	/* Select the smaller and equal supported value */
	for (i = 0; i < ARRAY_SIZE(vinovp_tbl)-1; i++) {
		if (val < vinovp_tbl[i+1])
			break;
	}
	return i;
}

static int mt6360_chg_init_setting(struct mt6360_chg_info *mci)
{
	int ret;
	u32 sel;

	sel = mt6360_vinovp_trans_to_sel(mci->vinovp);
	ret = regmap_update_bits(mci->regmap, MT6360_PMU_CHG_CTRL19,
				  MT6360_VINOVP_MASK, sel << MT6360_VINOVP_SHFT);
	if (ret)
		return dev_err_probe(mci->dev, ret, "%s: Failed to apply vinovp\n", __func__);
	ret = regmap_update_bits(mci->regmap, MT6360_PMU_DEVICE_TYPE,
				 MT6360_USBCHGEN_MASK, 0);
	if (ret)
		return dev_err_probe(mci->dev, ret, "%s: Failed to disable bc12\n", __func__);
	ret = regmap_update_bits(mci->regmap, MT6360_PMU_CHG_CTRL2,
				 MT6360_IINLMTSEL_MASK,
				 MT6360_IINLMTSEL_AICR <<
					MT6360_IINLMTSEL_SHFT);
	if (ret)
		return dev_err_probe(mci->dev, ret,
				     "%s: Failed to switch iinlmtsel to aicr\n", __func__);
	usleep_range(5000, 6000);
	ret = regmap_update_bits(mci->regmap, MT6360_PMU_CHG_CTRL3,
				 MT6360_ILIM_EN_MASK, 0);
	if (ret)
		return dev_err_probe(mci->dev, ret,
				     "%s: Failed to disable ilim\n", __func__);
	ret = regmap_update_bits(mci->regmap, MT6360_PMU_CHG_CTRL10,
				 MT6360_OTG_OC_MASK, MT6360_OTG_OC_MASK);
	if (ret)
		return dev_err_probe(mci->dev, ret,
				     "%s: Failed to config otg oc to 3A\n", __func__);
	return 0;
}

/* Charge limits from the monitored battery and the charger node */
static int mt6360_chg_apply_limits(struct mt6360_chg_info *mci)
{
	struct power_supply_battery_info *info = mci->bat_info;
	union power_supply_propval val;
	static const u32 timer_hours[] = { 4, 6, 8, 10, 12, 14, 16, 20 };
	u32 hours, i;
	int ret;

	val.intval = mci->aicr_def;
	ret = mt6360_charger_set_aicr(mci, &val);
	if (ret)
		return ret;

	mutex_lock(&mci->lim_lock);
	ret = mt6360_chg_write_limits(mci);
	mutex_unlock(&mci->lim_lock);
	if (ret)
		return ret;

	if (info) {
		if (info->precharge_current_ua > 0) {
			val.intval = info->precharge_current_ua;
			ret = mt6360_charger_set_iprechg(mci, &val);
			if (ret)
				return ret;
		}
		if (info->charge_term_current_ua > 0) {
			val.intval = info->charge_term_current_ua;
			ret = mt6360_charger_set_ieoc(mci, &val);
			if (!ret)
				ret = regmap_update_bits(mci->regmap, MT6360_PMU_CHG_CTRL2,
							 MT6360_TE_EN_MASK, MT6360_TE_EN_MASK);
			if (ret)
				return ret;
		}
	}

	if (!device_property_read_u32(mci->dev, "richtek,safety-timer-hours", &hours)) {
		for (i = 0; i < ARRAY_SIZE(timer_hours) - 1; i++)
			if (hours < timer_hours[i + 1])
				break;
		ret = regmap_update_bits(mci->regmap, MT6360_PMU_CHG_CTRL12,
					 MT6360_WT_FC_MASK | MT6360_TMR_EN_MASK,
					 (i << MT6360_WT_FC_SHFT) | MT6360_TMR_EN_MASK);
		if (ret)
			return ret;
	}

	if (device_property_read_bool(mci->dev, "richtek,disable-charger-watchdog")) {
		ret = regmap_update_bits(mci->regmap, MT6360_PMU_CHG_CTRL13,
					 MT6360_CHG_WDT_EN_MASK, 0);
		if (ret)
			return ret;
	}

	return 0;
}

static void mt6360_chg_dump(struct mt6360_chg_info *mci, const char *when)
{
	union power_supply_propval cv = {}, ichg = {}, aicr = {}, ieoc = {}, mivr = {};
	unsigned int c1 = 0, c2 = 0, c10 = 0, c12 = 0, c13 = 0, c19 = 0, st = 0;

	mt6360_charger_get_cv(mci, &cv);
	mt6360_charger_get_ichg(mci, &ichg);
	mt6360_charger_get_aicr(mci, &aicr);
	mt6360_charger_get_ieoc(mci, &ieoc);
	mt6360_charger_get_mivr(mci, &mivr);
	regmap_read(mci->regmap, MT6360_PMU_CHG_CTRL1, &c1);
	regmap_read(mci->regmap, MT6360_PMU_CHG_CTRL2, &c2);
	regmap_read(mci->regmap, MT6360_PMU_CHG_CTRL10, &c10);
	regmap_read(mci->regmap, MT6360_PMU_CHG_CTRL12, &c12);
	regmap_read(mci->regmap, MT6360_PMU_CHG_CTRL13, &c13);
	regmap_read(mci->regmap, MT6360_PMU_CHG_CTRL19, &c19);
	regmap_read(mci->regmap, MT6360_PMU_CHG_STAT, &st);
	dev_info(mci->dev,
		 "%s: CV %d uV, ICHG %d uA, AICR %d uA, IEOC %d uA, MIVR %d uV, CTRL1 %02x CTRL2 %02x CTRL10 %02x CTRL12 %02x CTRL13 %02x CTRL19 %02x STAT %02x\n",
		 when, cv.intval, ichg.intval, aicr.intval, ieoc.intval, mivr.intval,
		 c1, c2, c10, c12, c13, c19, st);
}

#define MT6360_GUARD_PERIOD_MS	10000
#define MT6360_GUARD_HYST_MC	2000

/* Zone of a battery temperature (mC); COOL and WARM include their outer boundary */
static enum mt6360_chg_zone mt6360_chg_temp_zone(struct mt6360_chg_info *mci, int temp)
{
	if (temp < mci->tmin_mc)
		return MT6360_ZONE_COLD;
	if (temp > mci->tmax_mc)
		return MT6360_ZONE_HOT;
	if (temp <= mci->cool_max_mc)
		return MT6360_ZONE_COOL;
	if (temp > mci->warm_min_mc)
		return MT6360_ZONE_WARM;
	return MT6360_ZONE_NORMAL;
}

/*
 * A zone that allows less (or the same) is taken at once. A zone that allows
 * more only once the temperature is MT6360_GUARD_HYST_MC inside it, so that
 * the limits do not toggle at a boundary.
 */
static bool mt6360_chg_zone_ok(struct mt6360_chg_info *mci, enum mt6360_chg_zone zone, int temp)
{
	const struct mt6360_chg_limit *o = &mci->zone_lim[mci->zone];
	const struct mt6360_chg_limit *n = &mci->zone_lim[zone];

	if (mci->zone == MT6360_ZONE_UNKNOWN || (n->ichg <= o->ichg && n->cv <= o->cv))
		return true;
	if (zone < mci->zone)
		return mt6360_chg_temp_zone(mci, temp + MT6360_GUARD_HYST_MC) <= zone;
	return mt6360_chg_temp_zone(mci, temp - MT6360_GUARD_HYST_MC) >= zone;
}

static u32 mt6360_chg_sel(int range, u32 val)
{
	u32 sel;

	linear_range_get_selector_within(&mt6360_chg_range[range], val, &sel);
	return sel;
}

/*
 * Bring CHG_EN, CV and ICHG in line with the zone. This runs every guard
 * period, so a register that changed behind the driver (another writer, a
 * charger reset) is restored within one period. Charging is switched off
 * before and on after the limits are written. Caller holds lim_lock.
 */
static int mt6360_chg_enforce(struct mt6360_chg_info *mci, bool zone_changed)
{
	bool en = mci->zone_lim[mci->zone].ichg;
	unsigned int ctrl2, ctrl4, ctrl7;
	int ret;

	lockdep_assert_held(&mci->lim_lock);

	ret = regmap_read(mci->regmap, MT6360_PMU_CHG_CTRL2, &ctrl2);
	if (ret)
		goto fail;

	if (!en) {
		if (ctrl2 & MT6360_CHG_EN_MASK) {
			ret = regmap_update_bits(mci->regmap, MT6360_PMU_CHG_CTRL2,
						 MT6360_CHG_EN_MASK, 0);
			if (ret)
				goto fail;
			if (!zone_changed)
				dev_warn(mci->dev, "CHG_EN was set outside the driver, cleared\n");
		}
		mci->chg_inhibit = true;
		return 0;
	}

	ret = regmap_read(mci->regmap, MT6360_PMU_CHG_CTRL4, &ctrl4);
	if (!ret)
		ret = regmap_read(mci->regmap, MT6360_PMU_CHG_CTRL7, &ctrl7);
	if (ret)
		goto fail;
	if ((ctrl4 & MT6360_VOREG_MASK) >> MT6360_VOREG_SHFT !=
	    mt6360_chg_sel(MT6360_RANGE_VOREG, mt6360_chg_eff_cv(mci)) ||
	    (ctrl7 & MT6360_ICHG_MASK) >> MT6360_ICHG_SHFT !=
	    mt6360_chg_sel(MT6360_RANGE_ICHG, mt6360_chg_eff_ichg(mci))) {
		ret = mt6360_chg_write_limits(mci);
		if (ret)
			goto fail;
		if (!zone_changed)
			dev_warn(mci->dev, "CV/ICHG changed outside the driver (%02x/%02x), restored\n",
				 ctrl4, ctrl7);
	}

	if (!(ctrl2 & MT6360_CHG_EN_MASK)) {
		ret = regmap_update_bits(mci->regmap, MT6360_PMU_CHG_CTRL2,
					 MT6360_CHG_EN_MASK, MT6360_CHG_EN_MASK);
		if (ret)
			goto fail;
	}
	mci->chg_inhibit = false;
	return 0;

fail:
	dev_err(mci->dev, "cannot apply the %s zone limits: %d, charging stopped\n",
		mt6360_chg_zone_name[mci->zone], ret);
	regmap_update_bits(mci->regmap, MT6360_PMU_CHG_CTRL2, MT6360_CHG_EN_MASK, 0);
	mci->chg_inhibit = true;
	return ret;
}

/*
 * Battery temperature guard: charging (CHG_EN) only within the
 * operating-range-celsius of the monitored battery, with the reduced limits
 * of the charger node inside its alert-celsius margins. The temperature comes
 * from the thermal zone named by mediatek,battery-thermal-zone; a failed read
 * stops charging.
 */
static void mt6360_chg_guard_work(struct work_struct *work)
{
	struct mt6360_chg_info *mci = container_of(to_delayed_work(work),
						   struct mt6360_chg_info, guard_work);
	enum mt6360_chg_zone zone;
	bool changed = false;
	int temp, ret;

	ret = thermal_zone_get_temp(mci->bat_tz, &temp);

	mutex_lock(&mci->lim_lock);
	if (ret) {
		if (!mci->tz_failed)
			dev_warn(mci->dev, "battery temperature unreadable (%d), charging stopped\n",
				 ret);
		mci->tz_failed = true;
		zone = MT6360_ZONE_UNKNOWN;
	} else {
		mci->tz_failed = false;
		zone = mt6360_chg_temp_zone(mci, temp);
		if (zone != mci->zone && !mt6360_chg_zone_ok(mci, zone, temp))
			zone = mci->zone;
	}

	if (zone != mci->zone) {
		mci->zone = zone;
		changed = true;
	}
	ret = mt6360_chg_enforce(mci, changed);
	if (changed && !ret) {
		if (mci->zone == MT6360_ZONE_UNKNOWN)
			dev_warn(mci->dev, "battery temperature unknown: charging disabled\n");
		else if (mci->chg_inhibit)
			dev_warn(mci->dev, "battery %d mC: %s zone, charging disabled\n",
				 temp, mt6360_chg_zone_name[zone]);
		else
			dev_info(mci->dev, "battery %d mC: %s zone, CV %u uV, ICHG %u uA\n",
				 temp, mt6360_chg_zone_name[zone], mt6360_chg_eff_cv(mci),
				 mt6360_chg_eff_ichg(mci));
	}
	mutex_unlock(&mci->lim_lock);

	if (changed)
		power_supply_changed(mci->psy);

	schedule_delayed_work(&mci->guard_work, msecs_to_jiffies(MT6360_GUARD_PERIOD_MS));
}

/*
 * Look up the battery thermal zone before the probe touches any register or
 * registers the power supply: the zone usually appears later than the
 * charger (it needs the PMIC AUXADC and generic-adc-thermal), and a deferral
 * after that point repeated the initial setting and the register dump.
 */
static int mt6360_chg_guard_get_zone(struct mt6360_chg_info *mci)
{
	struct thermal_zone_device *tz;
	struct device_node *tz_np;
	int ret;

	tz_np = of_parse_phandle(mci->dev->of_node, "mediatek,battery-thermal-zone", 0);
	if (!tz_np)
		return 0;
	tz = thermal_zone_get_zone_by_name(tz_np->name);
	of_node_put(tz_np);
	if (IS_ERR(tz)) {
		ret = PTR_ERR(tz);
		return dev_err_probe(mci->dev, ret == -ENODEV ? -EPROBE_DEFER : ret,
				     "battery thermal zone\n");
	}
	mci->bat_tz = tz;
	return 0;
}

/* Battery maximum and zone limits; without battery data the hardware range */
static void mt6360_chg_init_limits(struct mt6360_chg_info *mci)
{
	struct power_supply_battery_info *info = mci->bat_info;
	struct mt6360_chg_limit *z = mci->zone_lim;
	u32 v;

	if (info && info->constant_charge_current_max_ua > 0) {
		mci->ichg_max = clamp_val(info->constant_charge_current_max_ua,
					  MT6360_ICHG_MIN, MT6360_ICHG_MAX);
		mci->ichg_req = mci->ichg_max;
	}
	if (info && info->constant_charge_voltage_max_uv > 0) {
		mci->cv_max = clamp_val(info->constant_charge_voltage_max_uv,
					MT6360_VOREG_MIN, MT6360_VOREG_MAX);
		mci->cv_req = mci->cv_max;
	}

	z[MT6360_ZONE_NORMAL].ichg = mci->ichg_max;
	z[MT6360_ZONE_NORMAL].cv = mci->cv_max;
	z[MT6360_ZONE_COOL] = z[MT6360_ZONE_NORMAL];
	z[MT6360_ZONE_WARM] = z[MT6360_ZONE_NORMAL];
	if (!device_property_read_u32(mci->dev, "mediatek,cool-charge-current-microamp", &v))
		z[MT6360_ZONE_COOL].ichg = clamp_val(v, MT6360_ICHG_MIN, mci->ichg_max);
	if (!device_property_read_u32(mci->dev, "mediatek,cool-charge-voltage-microvolt", &v))
		z[MT6360_ZONE_COOL].cv = clamp_val(v, MT6360_VOREG_MIN, mci->cv_max);
	if (!device_property_read_u32(mci->dev, "mediatek,warm-charge-current-microamp", &v))
		z[MT6360_ZONE_WARM].ichg = clamp_val(v, MT6360_ICHG_MIN, mci->ichg_max);
	if (!device_property_read_u32(mci->dev, "mediatek,warm-charge-voltage-microvolt", &v))
		z[MT6360_ZONE_WARM].cv = clamp_val(v, MT6360_VOREG_MIN, mci->cv_max);
	/* COLD, HOT and UNKNOWN stay { 0, 0 }: charging disabled */
}

static int mt6360_chg_guard_init(struct mt6360_chg_info *mci)
{
	struct power_supply_battery_info *info = mci->bat_info;
	int ret;

	if (!info || info->temp_min == INT_MIN || info->temp_max == INT_MAX) {
		mci->bat_tz = NULL;	/* no range, no guard */
		return 0;
	}

	if (!mci->bat_tz)
		return dev_err_probe(mci->dev, -EINVAL,
				     "operating-range-celsius set but no battery thermal zone\n");

	mci->tmin_mc = info->temp_min * 1000;
	mci->tmax_mc = info->temp_max * 1000;
	mci->cool_max_mc = INT_MIN;
	mci->warm_min_mc = INT_MAX;
	if (info->temp_alert_min > info->temp_min && info->temp_alert_min < info->temp_max)
		mci->cool_max_mc = info->temp_alert_min * 1000;
	if (info->temp_alert_max < info->temp_max && info->temp_alert_max > info->temp_min)
		mci->warm_min_mc = info->temp_alert_max * 1000;

	/* Stop charging until the first successful temperature check */
	mci->zone = MT6360_ZONE_UNKNOWN;
	mci->chg_inhibit = true;
	ret = regmap_update_bits(mci->regmap, MT6360_PMU_CHG_CTRL2, MT6360_CHG_EN_MASK, 0);
	if (ret)
		return ret;

	ret = devm_delayed_work_autocancel(mci->dev, &mci->guard_work, mt6360_chg_guard_work);
	if (ret)
		return ret;
	dev_info(mci->dev, "battery temperature guard: charging only within [%d, %d] C\n",
		 info->temp_min, info->temp_max);
	if (mci->cool_max_mc != INT_MIN)
		dev_info(mci->dev, "up to %d C: CV %u uV, ICHG %u uA\n", info->temp_alert_min,
			 mci->zone_lim[MT6360_ZONE_COOL].cv, mci->zone_lim[MT6360_ZONE_COOL].ichg);
	if (mci->warm_min_mc != INT_MAX)
		dev_info(mci->dev, "above %d C: CV %u uV, ICHG %u uA\n", info->temp_alert_max,
			 mci->zone_lim[MT6360_ZONE_WARM].cv, mci->zone_lim[MT6360_ZONE_WARM].ichg);
	return 0;
}

/* above this the source has a high voltage (PD) contract */
#define MT6360_SRC_HV_MIN_UV	5500000
/* MIVR below the contract voltage, vendor PD_VBUS_IR_DROP_THRESHOLD */
#define MT6360_SRC_IR_DROP_UV	1200000
/* highest MIVR taken over from the boot loader, vendor min_charger_voltage */
#define MT6360_MIVR_DEF_MAX	4600000

/*
 * Input current limit for a dedicated or charging downstream port found by
 * BC1.2 when the Type-C port reports no current: below the 1.5 A that BC1.2
 * allows for both (vendor: 2 A for an AC adapter, 1.5 A for a charging host).
 * A standard downstream port or an unknown source keeps the default.
 */
#define MT6360_AICR_BC12_UA	1200000

static u32 mt6360_chg_bc12_aicr(struct mt6360_chg_info *mci)
{
	switch (mci->psy_usb_type) {
	case POWER_SUPPLY_USB_TYPE_DCP:
	case POWER_SUPPLY_USB_TYPE_CDP:
		return max_t(u32, mci->aicr_def, MT6360_AICR_BC12_UA);
	default:
		return mci->aicr_def;
	}
}

/*
 * Input current limit and MIVR for the current limit and voltage of the input
 * source (0 = not known): AICR follows the source, MIVR sits 1.2 V below a
 * high voltage contract. Without a known source AICR follows the BC1.2
 * result and MIVR keeps its default. User space may set the input current.
 * Caller holds lim_lock.
 */
static void mt6360_chg_src_set(struct mt6360_chg_info *mci, u32 src_ua, u32 src_uv, bool force)
{
	union power_supply_propval aicr, mivr;
	int ret;

	lockdep_assert_held(&mci->lim_lock);

	if (!force && src_ua == mci->src_ua && src_uv == mci->src_uv)
		return;

	aicr.intval = src_ua ? min_t(u32, src_ua, MT6360_AICR_MAX) : mt6360_chg_bc12_aicr(mci);
	mivr.intval = mci->mivr_def;
	if (src_ua && src_uv > MT6360_SRC_HV_MIN_UV)
		mivr.intval = max_t(u32, mci->mivr_def, src_uv - MT6360_SRC_IR_DROP_UV);

	ret = mt6360_charger_set_aicr(mci, &aicr);
	if (!ret)
		ret = mt6360_charger_set_mivr(mci, &mivr);
	if (ret) {
		dev_err(mci->dev, "cannot set the input limits: %d\n", ret);
		return;
	}

	if (src_ua != mci->src_ua || src_uv != mci->src_uv)
		dev_info(mci->dev, "input source %u uV, %u uA: AICR %d uA, MIVR %d uV\n",
			 src_uv, src_ua, aicr.intval, mivr.intval);
	mci->src_ua = src_ua;
	mci->src_uv = src_uv;
}

static void mt6360_chg_src_work(struct work_struct *work)
{
	struct mt6360_chg_info *mci = container_of(work, struct mt6360_chg_info, src_work);
	union power_supply_propval val;
	struct power_supply *src;
	u32 src_ua = 0, src_uv = 0;

	src = power_supply_get_by_reference(dev_fwnode(mci->dev), "mediatek,input-source");
	if (!IS_ERR_OR_NULL(src)) {
		if (!power_supply_get_property(src, POWER_SUPPLY_PROP_ONLINE, &val) &&
		    val.intval > 0 &&
		    !power_supply_get_property(src, POWER_SUPPLY_PROP_CURRENT_MAX, &val) &&
		    val.intval > 0) {
			src_ua = val.intval;
			if (!power_supply_get_property(src, POWER_SUPPLY_PROP_VOLTAGE_NOW, &val) &&
			    val.intval > 0)
				src_uv = val.intval;
		}
		power_supply_put(src);
	}

	mutex_lock(&mci->lim_lock);
	mt6360_chg_src_set(mci, src_ua, src_uv, false);
	mutex_unlock(&mci->lim_lock);
}

/* follow every change of the input source power supply */
static int mt6360_chg_psy_notifier(struct notifier_block *nb, unsigned long event, void *data)
{
	struct mt6360_chg_info *mci = container_of(nb, struct mt6360_chg_info, psy_nb);
	struct power_supply *psy = data;

	if (event == PSY_EVENT_PROP_CHANGED && psy->dev.parent &&
	    dev_fwnode(psy->dev.parent) == mci->src_fwnode)
		schedule_work(&mci->src_work);
	return NOTIFY_OK;
}

static void mt6360_chg_put_fwnode(void *data)
{
	fwnode_handle_put(data);
}

static void mt6360_chg_unreg_notifier(void *data)
{
	power_supply_unreg_notifier(data);
}

static int mt6360_chg_src_init(struct mt6360_chg_info *mci)
{
	struct fwnode_handle *fwnode;
	int ret;

	fwnode = fwnode_find_reference(dev_fwnode(mci->dev), "mediatek,input-source", 0);
	if (IS_ERR(fwnode)) {
		if (PTR_ERR(fwnode) != -ENOENT)
			dev_warn(mci->dev, "mediatek,input-source: %pe\n", fwnode);
		return 0;
	}
	mci->src_fwnode = fwnode;
	ret = devm_add_action_or_reset(mci->dev, mt6360_chg_put_fwnode, fwnode);
	if (ret)
		return ret;

	ret = devm_work_autocancel(mci->dev, &mci->src_work, mt6360_chg_src_work);
	if (ret)
		return ret;
	mci->psy_nb.notifier_call = mt6360_chg_psy_notifier;
	ret = power_supply_reg_notifier(&mci->psy_nb);
	if (ret)
		return ret;
	ret = devm_add_action_or_reset(mci->dev, mt6360_chg_unreg_notifier, &mci->psy_nb);
	if (ret)
		return ret;

	schedule_work(&mci->src_work);
	return 0;
}

static int mt6360_charger_probe(struct platform_device *pdev)
{
	struct mt6360_chg_info *mci;
	struct power_supply_config charger_cfg = {};
	struct regulator_config config = { };
	union power_supply_propval val;
	int ret;

	mci = devm_kzalloc(&pdev->dev, sizeof(*mci), GFP_KERNEL);
	if (!mci)
		return -ENOMEM;

	mci->dev = &pdev->dev;
	mci->vinovp = 6500000;
	mutex_init(&mci->chgdet_lock);
	mutex_init(&mci->lim_lock);
	platform_set_drvdata(pdev, mci);
	ret = devm_work_autocancel(&pdev->dev, &mci->chrdet_work, mt6360_chrdet_work);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "Failed to set delayed work\n");

	ret = device_property_read_u32(&pdev->dev, "richtek,vinovp-microvolt", &mci->vinovp);
	if (ret)
		dev_warn(&pdev->dev, "Failed to parse vinovp in DT, keep default 6.5v\n");

	mci->regmap = dev_get_regmap(pdev->dev.parent, NULL);
	if (!mci->regmap)
		return dev_err_probe(&pdev->dev, -ENODEV, "Failed to get parent regmap\n");

	ret = mt6360_chg_guard_get_zone(mci);
	if (ret)
		return ret;

	ret = mt6360_chg_init_setting(mci);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "Failed to initial setting\n");

	/*
	 * Defaults: the input current from the charger node (else as found),
	 * MIVR as set by the boot loader (at most the vendor value), charge
	 * current and voltage as found until the battery data is known.
	 */
	ret = mt6360_charger_get_aicr(mci, &val);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "Failed to read AICR\n");
	mci->aicr_def = val.intval;
	device_property_read_u32(&pdev->dev, "input-current-limit-microamp", &mci->aicr_def);
	ret = mt6360_charger_get_mivr(mci, &val);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "Failed to read MIVR\n");
	mci->mivr_def = min_t(u32, val.intval, MT6360_MIVR_DEF_MAX);
	ret = mt6360_charger_get_ichg(mci, &val);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "Failed to read ICHG\n");
	mci->ichg_req = val.intval;
	ret = mt6360_charger_get_cv(mci, &val);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "Failed to read CV\n");
	mci->cv_req = val.intval;
	mci->ichg_max = MT6360_ICHG_MAX;
	mci->cv_max = MT6360_VOREG_MAX;
	mci->zone = MT6360_ZONE_NORMAL;

	memcpy(&mci->psy_desc, &mt6360_charger_desc, sizeof(mci->psy_desc));
	mci->psy_desc.name = dev_name(&pdev->dev);
	charger_cfg.drv_data = mci;
	charger_cfg.fwnode = dev_fwnode(&pdev->dev);
	mci->psy = devm_power_supply_register(&pdev->dev,
					      &mci->psy_desc, &charger_cfg);
	if (IS_ERR(mci->psy))
		return dev_err_probe(&pdev->dev, PTR_ERR(mci->psy),
				     "Failed to register power supply dev\n");

	mt6360_chg_dump(mci, "initial");

	ret = power_supply_get_battery_info(mci->psy, &mci->bat_info);
	if (ret)
		mci->bat_info = NULL;

	mutex_lock(&mci->lim_lock);
	mt6360_chg_init_limits(mci);
	mutex_unlock(&mci->lim_lock);

	ret = mt6360_chg_guard_init(mci);
	if (ret)
		return ret;

	ret = mt6360_chg_apply_limits(mci);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "Failed to apply charge limits\n");

	ret = mt6360_chg_irq_register(pdev);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "Failed to register irqs\n");

	config.dev = &pdev->dev;
	config.regmap = mci->regmap;
	config.driver_data = mci;
	mci->otg_rdev = devm_regulator_register(&pdev->dev, &mt6360_otg_rdesc,
						&config);
	if (IS_ERR(mci->otg_rdev))
		return PTR_ERR(mci->otg_rdev);

	ret = mt6360_chg_src_init(mci);
	if (ret)
		return dev_err_probe(&pdev->dev, ret, "Failed to set up the input source\n");

	schedule_work(&mci->chrdet_work);
	if (mci->bat_tz)
		schedule_delayed_work(&mci->guard_work, 0);

	mt6360_chg_dump(mci, "configured");
	return 0;
}

static const struct of_device_id __maybe_unused mt6360_charger_of_id[] = {
	{ .compatible = "mediatek,mt6360-chg", },
	{},
};
MODULE_DEVICE_TABLE(of, mt6360_charger_of_id);

static const struct platform_device_id mt6360_charger_id[] = {
	{ .name = "mt6360-chg" },
	{ }
};
MODULE_DEVICE_TABLE(platform, mt6360_charger_id);

static struct platform_driver mt6360_charger_driver = {
	.driver = {
		.name = "mt6360-chg",
		.of_match_table = mt6360_charger_of_id,
	},
	.probe = mt6360_charger_probe,
	.id_table = mt6360_charger_id,
};
module_platform_driver(mt6360_charger_driver);

MODULE_AUTHOR("Gene Chen <gene_chen@richtek.com>");
MODULE_DESCRIPTION("MT6360 Charger Driver");
MODULE_LICENSE("GPL");
