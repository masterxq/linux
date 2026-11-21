// SPDX-License-Identifier: GPL-2.0-only
//
// aw88258.c  --  Awinic AW88258 (chip ID 0x1852) smart power amplifier, without DSP
//
// Register map, power sequence and the register profile format follow the Awinic aw882xx
// vendor driver v0.2.6 (Copyright (c) 2019 AWINIC Technology CO., LTD, Nick Li, GPL-2.0 or
// later), aw882xx.[ch] and aw882xx_reg.h. The chip itself only reports the ID 0x1852.
//
// The vendor design protects the loudspeaker with a DSP algorithm (excursion and temperature
// via the I/V sense feedback). There is no such DSP here, so there is no loudspeaker
// protection; the driver only keeps the hardware within the vendor settings:
//  - the amplifier starts muted and powered down, and is only powered up with "Amp Switch" on
//  - "Amp Volume" covers the full attenuation range down to 0 dB, as the vendor driver runs it
//    during a speakerphone call; it starts at the mute end. A lower limit for media playback is
//    a policy of the user of the control (UCM, sound server), not of this driver
//  - the profile cannot raise the speaker gain or the boost peak current above the caps below
//  - while playing, the status flags (over-temperature, over-current, boost over-current,
//    over/under voltage) are polled; any of them powers the amplifier down until the next start
// The vendor register profile (aw882xx_spk_reg.bin) is loaded with request_firmware() and is
// not part of this driver.

#include <linux/bitfield.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/regmap.h>
#include <linux/seq_file.h>
#include <linux/workqueue.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include <sound/tlv.h>

#define AW_REG_ID		0x00
#define AW_REG_SYSST		0x01
#define AW_REG_SYSINT		0x02
#define AW_REG_SYSINTM		0x03
#define AW_REG_SYSCTRL		0x04
#define AW_REG_SYSCTRL2		0x05
#define AW_REG_I2SCTRL		0x06
#define AW_REG_I2SCFG1		0x07
#define AW_REG_HAGCCFG4		0x0c
#define AW_REG_VBAT		0x12
#define AW_REG_TEMP		0x13
#define AW_REG_PLLCTRL1		0x66
#define AW_REG_TESTCTRL2	0x71
#define AW_REG_EFCTRL1		0x72
#define AW_REG_EFRL		0x7b
#define AW_REG_MAX		0x7d

#define AW_CHIP_ID		0x1852

/* SYSST */
#define AW_SYSST_PLLS		BIT(0)
#define AW_SYSST_OTHS		BIT(1)
#define AW_SYSST_OCDS		BIT(3)
#define AW_SYSST_CLKS		BIT(4)
#define AW_SYSST_BSTOCS		BIT(11)
#define AW_SYSST_UVLS		BIT(14)
#define AW_SYSST_OVP2S		BIT(15)
#define AW_SYSST_FAULTS		(AW_SYSST_OTHS | AW_SYSST_OCDS | AW_SYSST_BSTOCS | \
				 AW_SYSST_UVLS | AW_SYSST_OVP2S)
/* SYSCTRL */
#define AW_SPK_GAIN_MASK	GENMASK(14, 12)
#define AW_AMPPD		BIT(1)
#define AW_PWDN			BIT(0)
/* SYSCTRL2 */
#define AW_HMUTE		BIT(4)
#define AW_BST_IPEAK_MASK	GENMASK(3, 0)
/* I2SCTRL */
#define AW_CHSEL_MASK		GENMASK(11, 10)
#define AW_I2SFS_MASK		GENMASK(7, 6)
#define AW_I2SBCK_MASK		GENMASK(5, 4)
#define AW_I2SSR_MASK		GENMASK(3, 0)
/* I2SCFG1 */
#define AW_DOHZ			BIT(4)
#define AW_I2STXEN		BIT(0)
/* HAGCCFG4 */
#define AW_VOL_MASK		GENMASK(15, 8)
#define AW_VOL_MUTE		0xf0	/* 90 dB attenuation */
/* PLLCTRL1 */
#define AW_CCO_MUX		BIT(14)
/* TESTCTRL2 */
#define AW_DITHER_EN		BIT(7)

#define AW_SYSINTM_ALL		0xefbf	/* vendor: all interrupts masked */

/*
 * I2SCTRL CHSEL: which slot of the stereo I2S frame the amplifier plays. The vendor profile
 * selects the left slot and relies on the Android HAL to downmix to mono for the single
 * loudspeaker. Without that HAL the amplifier mixes the slots itself ((L + R) / 2), so a
 * stereo stream is played completely; left or right only are kept for stereo pairs.
 */
enum {
	AW_CHSEL_LEFT = 1,
	AW_CHSEL_RIGHT = 2,
	AW_CHSEL_MONO = 3,
};

/*
 * Caps on the register profile. The vendor profile uses SPK_GAIN 4, BST_IPEAK 9 (3.75 A) and
 * 0 dB attenuation; the vendor monitor lowers the peak current to 2.5 A in the cold.
 */
#define AW_SPK_GAIN_MAX		4
#define AW_BST_IPEAK_MAX	8	/* 3.5 A */

/* "Amp Volume": 0.5 dB steps from 90 dB attenuation (step 0, treated as mute) to 0 dB */
#define AW_MAX_ATTEN_HALF_DB	180	/* 90 dB */
#define AW_VOL_STEPS		AW_MAX_ATTEN_HALF_DB

#define AW_FAULT_POLL_MS	1000
#define AW_PROFILE_MAX_LEN	512

struct aw88258 {
	struct device *dev;
	struct regmap *regmap;
	struct gpio_desc *reset;
	const char *fw_name;
	struct delayed_work fault_work;
	struct mutex lock;		/* state, controls, register sequences */
	struct dentry *debugfs;
	bool profile_loaded;
	bool stream_on;			/* playback stream unmuted by ASoC */
	bool running;
	bool fault;
	bool spk_switch;		/* "Amp Switch", default off */
	unsigned int chsel;		/* "Amp Channel": I2S slot(s) played, AW_CHSEL_* */
	unsigned int vol;		/* "Amp Volume" steps, starts at 0 = 90 dB attenuation */
	unsigned int last_sysst;
};

static const struct regmap_config aw88258_regmap = {
	.reg_bits = 8,
	.val_bits = 16,
	.max_register = AW_REG_MAX,
	.cache_type = REGCACHE_NONE,
};

/* attenuation in 0.5 dB steps -> VOL field: upper nibble 6 dB, lower nibble 0.5 dB (0..11) */
static unsigned int aw88258_vol_reg(unsigned int steps)
{
	unsigned int att = AW_MAX_ATTEN_HALF_DB - min(steps, AW_VOL_STEPS);

	return ((att / 12) << 4) | (att % 12);
}

static int aw88258_apply_volume(struct aw88258 *aw)
{
	unsigned int vol = aw->spk_switch ? aw88258_vol_reg(aw->vol) : AW_VOL_MUTE;
	int ret;

	ret = regmap_update_bits(aw->regmap, AW_REG_HAGCCFG4, AW_VOL_MASK, vol << 8);
	if (ret)
		return ret;
	return regmap_update_bits(aw->regmap, AW_REG_SYSCTRL2, AW_HMUTE,
				  aw->spk_switch ? 0 : AW_HMUTE);
}

static void aw88258_power_down(struct aw88258 *aw)
{
	regmap_update_bits(aw->regmap, AW_REG_HAGCCFG4, AW_VOL_MASK, AW_VOL_MUTE << 8);
	regmap_update_bits(aw->regmap, AW_REG_SYSCTRL2, AW_HMUTE, AW_HMUTE);
	regmap_update_bits(aw->regmap, AW_REG_I2SCFG1, AW_I2STXEN, 0);
	regmap_update_bits(aw->regmap, AW_REG_SYSCTRL, AW_PWDN, AW_PWDN);
}

/*
 * Vendor profile: records of 4 bytes, register address and value as little-endian u16, no
 * header and no checksum. Written in file order with the caps applied; ID/status/interrupt
 * registers and the efuse block are never written (0x00 = soft reset, 0x72.. = efuse program).
 */
static int aw88258_load_profile(struct aw88258 *aw)
{
	const struct firmware *fw;
	unsigned int i, written = 0;
	int ret;

	ret = request_firmware(&fw, aw->fw_name, aw->dev);
	if (ret) {
		dev_err(aw->dev, "profile %s: %d\n", aw->fw_name, ret);
		return ret;
	}
	if (!fw->size || fw->size % 4 || fw->size > AW_PROFILE_MAX_LEN) {
		dev_err(aw->dev, "profile %s: bad size %zu\n", aw->fw_name, fw->size);
		ret = -EINVAL;
		goto out;
	}

	for (i = 0; i < fw->size; i += 4) {
		unsigned int reg = fw->data[i] | fw->data[i + 1] << 8;
		unsigned int val = fw->data[i + 2] | fw->data[i + 3] << 8;

		if (reg <= AW_REG_SYSINT || reg > AW_REG_MAX ||
		    (reg >= AW_REG_EFCTRL1 && reg <= AW_REG_EFRL)) {
			dev_warn(aw->dev, "profile: register 0x%02x skipped\n", reg);
			continue;
		}

		switch (reg) {
		case AW_REG_SYSINTM:
			val = AW_SYSINTM_ALL;
			break;
		case AW_REG_SYSCTRL:
			if (FIELD_GET(AW_SPK_GAIN_MASK, val) > AW_SPK_GAIN_MAX)
				val = (val & ~AW_SPK_GAIN_MASK) |
				      FIELD_PREP(AW_SPK_GAIN_MASK, AW_SPK_GAIN_MAX);
			val |= AW_PWDN;
			break;
		case AW_REG_SYSCTRL2:
			if (FIELD_GET(AW_BST_IPEAK_MASK, val) > AW_BST_IPEAK_MAX)
				val = (val & ~AW_BST_IPEAK_MASK) | AW_BST_IPEAK_MAX;
			val |= AW_HMUTE;
			break;
		case AW_REG_I2SCTRL:
			val = (val & ~AW_CHSEL_MASK) | FIELD_PREP(AW_CHSEL_MASK, aw->chsel);
			break;
		case AW_REG_I2SCFG1:
			/* no I/V sense feedback without DSP: transmitter off, output Hi-Z */
			val = (val & ~AW_I2STXEN) | AW_DOHZ;
			break;
		case AW_REG_HAGCCFG4:
			val = (val & ~AW_VOL_MASK) | (AW_VOL_MUTE << 8);
			break;
		case AW_REG_TESTCTRL2:
			ret = regmap_update_bits(aw->regmap, reg, AW_DITHER_EN, val & AW_DITHER_EN);
			if (ret)
				goto out;
			written++;
			continue;
		}

		ret = regmap_write(aw->regmap, reg, val);
		if (ret)
			goto out;
		written++;
	}
	aw->profile_loaded = true;
	dev_info(aw->dev, "profile %s: %u registers written (capped, muted)\n",
		 aw->fw_name, written);
out:
	release_firmware(fw);
	return ret;
}

static int aw88258_start(struct aw88258 *aw)
{
	unsigned int sysst = 0;
	int ret, i;

	if (aw->running)
		return 0;
	if (!aw->profile_loaded)
		return -ENODEV;

	regmap_update_bits(aw->regmap, AW_REG_SYSCTRL2, AW_HMUTE, AW_HMUTE);
	regmap_update_bits(aw->regmap, AW_REG_I2SCFG1, AW_I2STXEN, 0);
	ret = regmap_update_bits(aw->regmap, AW_REG_SYSCTRL, AW_PWDN, 0);
	if (ret)
		return ret;
	usleep_range(2000, 2500);

	/* vendor: up to 10 x 2 ms for PLL lock and clock OK (needs BCK/WS from the AFE) */
	for (i = 0; i < 10; i++) {
		ret = regmap_read(aw->regmap, AW_REG_SYSST, &sysst);
		if (!ret && (sysst & (AW_SYSST_PLLS | AW_SYSST_CLKS)) ==
			    (AW_SYSST_PLLS | AW_SYSST_CLKS) && !(sysst & AW_SYSST_FAULTS))
			break;
		usleep_range(2000, 2500);
	}
	aw->last_sysst = sysst;
	if (i == 10) {
		dev_err(aw->dev, "start: SYSST 0x%04x (PLL/clock not ok or fault), staying off\n",
			sysst);
		aw88258_power_down(aw);
		return -EIO;
	}

	/* read-to-clear interrupt status, then volume/unmute as the controls say */
	regmap_read(aw->regmap, AW_REG_SYSINT, &sysst);
	regmap_read(aw->regmap, AW_REG_SYSINT, &sysst);
	ret = aw88258_apply_volume(aw);
	if (ret) {
		aw88258_power_down(aw);
		return ret;
	}
	aw->running = true;
	aw->fault = false;
	schedule_delayed_work(&aw->fault_work, msecs_to_jiffies(AW_FAULT_POLL_MS));
	return 0;
}

static void aw88258_stop(struct aw88258 *aw)
{
	aw->running = false;
	aw88258_power_down(aw);
}

static void aw88258_fault_work(struct work_struct *work)
{
	struct aw88258 *aw = container_of(work, struct aw88258, fault_work.work);
	unsigned int sysst;

	mutex_lock(&aw->lock);
	if (!aw->running)
		goto out;
	if (regmap_read(aw->regmap, AW_REG_SYSST, &sysst))
		sysst = AW_SYSST_FAULTS;	/* I2C failure: treat as fault */
	aw->last_sysst = sysst;
	if (sysst & AW_SYSST_FAULTS) {
		dev_err(aw->dev, "fault, SYSST 0x%04x: amplifier powered down\n", sysst);
		aw->fault = true;
		aw88258_stop(aw);
		goto out;
	}
	schedule_delayed_work(&aw->fault_work, msecs_to_jiffies(AW_FAULT_POLL_MS));
out:
	mutex_unlock(&aw->lock);
}

static int aw88258_hw_params(struct snd_pcm_substream *substream,
			     struct snd_pcm_hw_params *params, struct snd_soc_dai *dai)
{
	struct aw88258 *aw = snd_soc_component_get_drvdata(dai->component);
	unsigned int rate = params_rate(params), sr, fs, bck, frame;
	int ret = 0;

	switch (rate) {
	case 8000:
		sr = 0;
		break;
	case 16000:
		sr = 3;
		break;
	case 32000:
		sr = 6;
		break;
	case 44100:
		sr = 7;
		break;
	case 48000:
		sr = 8;
		break;
	default:
		return -EINVAL;
	}
	switch (params_width(params)) {
	case 16:
		fs = 0;
		break;
	case 24:
		fs = 2;
		break;
	case 32:
		fs = 3;
		break;
	default:
		return -EINVAL;
	}
	frame = params_physical_width(params) * 2;
	switch (frame) {
	case 32:
		bck = 0;
		break;
	case 48:
		bck = 1;
		break;
	case 64:
		bck = 2;
		break;
	default:
		return -EINVAL;
	}

	mutex_lock(&aw->lock);
	/*
	 * Without the profile the amplifier is never powered up, but the I2S
	 * link keeps working (e.g. for an AFE loopback test).
	 */
	if (!aw->profile_loaded && aw88258_load_profile(aw))
		dev_warn(aw->dev, "no profile: amplifier stays powered down\n");
	ret = regmap_update_bits(aw->regmap, AW_REG_I2SCTRL,
				 AW_CHSEL_MASK | AW_I2SFS_MASK | AW_I2SBCK_MASK | AW_I2SSR_MASK,
				 FIELD_PREP(AW_CHSEL_MASK, aw->chsel) |
				 FIELD_PREP(AW_I2SFS_MASK, fs) | FIELD_PREP(AW_I2SBCK_MASK, bck) |
				 FIELD_PREP(AW_I2SSR_MASK, sr));
	if (!ret)
		/* vendor: CCO_MUX 0 for 8/16/32 kHz, 1 otherwise */
		ret = regmap_update_bits(aw->regmap, AW_REG_PLLCTRL1, AW_CCO_MUX,
					 (rate == 8000 || rate == 16000 || rate == 32000) ?
					 0 : AW_CCO_MUX);
	mutex_unlock(&aw->lock);
	return ret;
}

static int aw88258_mute_stream(struct snd_soc_dai *dai, int mute, int stream)
{
	struct aw88258 *aw = snd_soc_component_get_drvdata(dai->component);
	int ret = 0;

	if (stream != SNDRV_PCM_STREAM_PLAYBACK)
		return 0;

	mutex_lock(&aw->lock);
	aw->stream_on = !mute;
	if (mute) {
		cancel_delayed_work(&aw->fault_work);
		aw88258_stop(aw);
	} else if (aw->spk_switch) {
		/* "Amp Switch" off: the amplifier stays powered down */
		ret = aw88258_start(aw);
		/*
		 * Without profile or clock the amplifier stays powered down;
		 * that is the safe state, so do not fail the whole stream.
		 */
		if (ret) {
			dev_warn_ratelimited(aw->dev, "amplifier stays off: %d\n", ret);
			ret = 0;
		}
	}
	mutex_unlock(&aw->lock);
	return ret;
}

static int aw88258_set_fmt(struct snd_soc_dai *dai, unsigned int fmt)
{
	if ((fmt & SND_SOC_DAIFMT_FORMAT_MASK) != SND_SOC_DAIFMT_I2S ||
	    (fmt & SND_SOC_DAIFMT_CLOCK_PROVIDER_MASK) != SND_SOC_DAIFMT_CBC_CFC)
		return -EINVAL;
	return 0;
}

static const struct snd_soc_dai_ops aw88258_dai_ops = {
	.hw_params = aw88258_hw_params,
	.set_fmt = aw88258_set_fmt,
	.mute_stream = aw88258_mute_stream,
	.no_capture_mute = 1,
};

static struct snd_soc_dai_driver aw88258_dai = {
	.name = "aw88258-aif",
	.playback = {
		.stream_name = "Speaker_Playback",
		.channels_min = 1,
		.channels_max = 2,
		.rates = SNDRV_PCM_RATE_8000 | SNDRV_PCM_RATE_16000 | SNDRV_PCM_RATE_32000 |
			 SNDRV_PCM_RATE_44100 | SNDRV_PCM_RATE_48000,
		.formats = SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_S24_LE |
			   SNDRV_PCM_FMTBIT_S32_LE,
	},
	.ops = &aw88258_dai_ops,
};

static int aw88258_vol_get(struct snd_kcontrol *kc, struct snd_ctl_elem_value *uc)
{
	struct aw88258 *aw = snd_soc_component_get_drvdata(snd_kcontrol_chip(kc));

	uc->value.integer.value[0] = aw->vol;
	return 0;
}

static int aw88258_vol_put(struct snd_kcontrol *kc, struct snd_ctl_elem_value *uc)
{
	struct aw88258 *aw = snd_soc_component_get_drvdata(snd_kcontrol_chip(kc));
	long v = uc->value.integer.value[0];
	int ret = 0;

	if (v < 0 || v > AW_VOL_STEPS)
		return -EINVAL;
	mutex_lock(&aw->lock);
	if (aw->vol != v) {
		aw->vol = v;
		ret = 1;
		if (aw->running)
			aw88258_apply_volume(aw);
	}
	mutex_unlock(&aw->lock);
	return ret;
}

static int aw88258_switch_get(struct snd_kcontrol *kc, struct snd_ctl_elem_value *uc)
{
	struct aw88258 *aw = snd_soc_component_get_drvdata(snd_kcontrol_chip(kc));

	uc->value.integer.value[0] = aw->spk_switch;
	return 0;
}

static int aw88258_switch_put(struct snd_kcontrol *kc, struct snd_ctl_elem_value *uc)
{
	struct aw88258 *aw = snd_soc_component_get_drvdata(snd_kcontrol_chip(kc));
	bool on = !!uc->value.integer.value[0];
	int ret = 0;

	mutex_lock(&aw->lock);
	if (aw->spk_switch != on) {
		aw->spk_switch = on;
		ret = 1;
		if (!on && aw->running) {
			cancel_delayed_work(&aw->fault_work);
			aw88258_stop(aw);
		} else if (on && aw->stream_on && !aw->running && aw88258_start(aw)) {
			dev_err(aw->dev, "amplifier start failed\n");
		}
	}
	mutex_unlock(&aw->lock);
	return ret;
}

static const char * const aw88258_chsel_texts[] = { "Left", "Right", "Mono" };

static int aw88258_chsel_get(struct snd_kcontrol *kc, struct snd_ctl_elem_value *uc)
{
	struct aw88258 *aw = snd_soc_component_get_drvdata(snd_kcontrol_chip(kc));

	uc->value.enumerated.item[0] = aw->chsel - AW_CHSEL_LEFT;
	return 0;
}

static int aw88258_chsel_put(struct snd_kcontrol *kc, struct snd_ctl_elem_value *uc)
{
	struct aw88258 *aw = snd_soc_component_get_drvdata(snd_kcontrol_chip(kc));
	unsigned int item = uc->value.enumerated.item[0];
	int ret = 0;

	if (item >= ARRAY_SIZE(aw88258_chsel_texts))
		return -EINVAL;
	mutex_lock(&aw->lock);
	if (aw->chsel != item + AW_CHSEL_LEFT) {
		aw->chsel = item + AW_CHSEL_LEFT;
		regmap_update_bits(aw->regmap, AW_REG_I2SCTRL, AW_CHSEL_MASK,
				   FIELD_PREP(AW_CHSEL_MASK, aw->chsel));
		ret = 1;
	}
	mutex_unlock(&aw->lock);
	return ret;
}

static SOC_ENUM_SINGLE_EXT_DECL(aw88258_chsel_enum, aw88258_chsel_texts);

/* 0 = -90 dB (muted), each step +0.5 dB, top = 0 dB */
static const DECLARE_TLV_DB_SCALE(aw88258_vol_tlv, -9000, 50, 1);

static const struct snd_kcontrol_new aw88258_controls[] = {
	SOC_SINGLE_EXT_TLV("Amp Volume", SND_SOC_NOPM, 0, AW_VOL_STEPS, 0,
			   aw88258_vol_get, aw88258_vol_put, aw88258_vol_tlv),
	SOC_SINGLE_BOOL_EXT("Amp Switch", 0, aw88258_switch_get, aw88258_switch_put),
	SOC_ENUM_EXT("Amp Channel", aw88258_chsel_enum, aw88258_chsel_get, aw88258_chsel_put),
};

static const struct snd_soc_dapm_widget aw88258_widgets[] = {
	SND_SOC_DAPM_AIF_IN("AMP_AIF_RX", "Speaker_Playback", 0, SND_SOC_NOPM, 0, 0),
	SND_SOC_DAPM_OUTPUT("SPK"),
};

static const struct snd_soc_dapm_route aw88258_routes[] = {
	{ "SPK", NULL, "AMP_AIF_RX" },
};

static const struct snd_soc_component_driver aw88258_component = {
	.controls = aw88258_controls,
	.num_controls = ARRAY_SIZE(aw88258_controls),
	.dapm_widgets = aw88258_widgets,
	.num_dapm_widgets = ARRAY_SIZE(aw88258_widgets),
	.dapm_routes = aw88258_routes,
	.num_dapm_routes = ARRAY_SIZE(aw88258_routes),
	.endianness = 1,
};

static int aw88258_status_show(struct seq_file *s, void *data)
{
	struct aw88258 *aw = s->private;
	static const struct { u8 reg; const char *name; } regs[] = {
		{ AW_REG_SYSST, "SYSST" }, { AW_REG_SYSCTRL, "SYSCTRL" },
		{ AW_REG_SYSCTRL2, "SYSCTRL2" }, { AW_REG_I2SCTRL, "I2SCTRL" },
		{ AW_REG_I2SCFG1, "I2SCFG1" }, { AW_REG_HAGCCFG4, "HAGCCFG4" },
		{ AW_REG_PLLCTRL1, "PLLCTRL1" },
	};
	unsigned int i, v;

	mutex_lock(&aw->lock);
	seq_printf(s, "profile %s, running %d, fault %d, switch %d, volume step %u (-%u.%u dB)\n",
		   aw->profile_loaded ? "loaded" : "not loaded", aw->running, aw->fault,
		   aw->spk_switch, aw->vol, (AW_MAX_ATTEN_HALF_DB - aw->vol) / 2,
		   (AW_MAX_ATTEN_HALF_DB - aw->vol) % 2 ? 5 : 0);
	for (i = 0; i < ARRAY_SIZE(regs); i++) {
		if (regmap_read(aw->regmap, regs[i].reg, &v))
			seq_printf(s, "0x%02x %-9s read error\n", regs[i].reg, regs[i].name);
		else
			seq_printf(s, "0x%02x %-9s 0x%04x\n", regs[i].reg, regs[i].name, v);
	}
	/* vendor monitor conversions: VBAT mV = raw * 6025 / 1023, TEMP = signed 10 bit in C */
	if (!regmap_read(aw->regmap, AW_REG_VBAT, &v))
		seq_printf(s, "VBAT %u mV\n", (v & 0x3ff) * 6025 / 1023);
	if (!regmap_read(aw->regmap, AW_REG_TEMP, &v))
		seq_printf(s, "TEMP %d C\n", sign_extend32(v & 0x3ff, 9));
	mutex_unlock(&aw->lock);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(aw88258_status);

static int aw88258_probe(struct i2c_client *i2c)
{
	struct device *dev = &i2c->dev;
	struct aw88258 *aw;
	unsigned int id = 0;
	int ret, i;

	aw = devm_kzalloc(dev, sizeof(*aw), GFP_KERNEL);
	if (!aw)
		return -ENOMEM;
	aw->dev = dev;
	aw->chsel = AW_CHSEL_MONO;
	mutex_init(&aw->lock);
	INIT_DELAYED_WORK(&aw->fault_work, aw88258_fault_work);
	i2c_set_clientdata(i2c, aw);

	aw->fw_name = "aw882xx_spk_reg.bin";
	device_property_read_string(dev, "firmware-name", &aw->fw_name);

	aw->regmap = devm_regmap_init_i2c(i2c, &aw88258_regmap);
	if (IS_ERR(aw->regmap))
		return dev_err_probe(dev, PTR_ERR(aw->regmap), "regmap\n");

	/* vendor: reset low 1 ms, high, 2 ms */
	aw->reset = devm_gpiod_get_optional(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(aw->reset))
		return dev_err_probe(dev, PTR_ERR(aw->reset), "reset gpio\n");
	if (aw->reset) {
		usleep_range(1000, 1500);
		gpiod_set_value_cansleep(aw->reset, 0);
		usleep_range(2000, 2500);
	}

	for (i = 0; i < 5; i++) {
		ret = regmap_read(aw->regmap, AW_REG_ID, &id);
		if (!ret && id == AW_CHIP_ID)
			break;
		usleep_range(5000, 6000);
	}
	if (ret || id != AW_CHIP_ID) {
		ret = ret ?: -ENODEV;
		dev_err(dev, "chip ID 0x%04x (expected 0x%04x): %d\n", id, AW_CHIP_ID, ret);
		goto err_reset;
	}

	/* after reset the chip is powered down and muted; make sure of it */
	aw88258_power_down(aw);

	ret = devm_snd_soc_register_component(dev, &aw88258_component, &aw88258_dai, 1);
	if (ret)
		goto err_reset;

	aw->debugfs = debugfs_create_file(dev_name(dev), 0444, NULL, aw, &aw88258_status_fops);
	dev_info(dev, "AW88258 (ID 0x%04x), muted; no DSP speaker protection\n", id);
	return 0;

err_reset:
	if (aw->reset)
		gpiod_set_value_cansleep(aw->reset, 1);
	return ret;
}

static void aw88258_remove(struct i2c_client *i2c)
{
	struct aw88258 *aw = i2c_get_clientdata(i2c);

	debugfs_remove(aw->debugfs);
	cancel_delayed_work_sync(&aw->fault_work);
	mutex_lock(&aw->lock);
	aw88258_stop(aw);
	mutex_unlock(&aw->lock);
	if (aw->reset)
		gpiod_set_value_cansleep(aw->reset, 1);
}

static const struct of_device_id aw88258_of_match[] = {
	{ .compatible = "awinic,aw88258" },
	{ }
};
MODULE_DEVICE_TABLE(of, aw88258_of_match);

static const struct i2c_device_id aw88258_i2c_id[] = {
	{ "aw88258" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, aw88258_i2c_id);

static struct i2c_driver aw88258_driver = {
	.driver = {
		.name = "aw88258",
		.of_match_table = aw88258_of_match,
	},
	.probe = aw88258_probe,
	.remove = aw88258_remove,
	.id_table = aw88258_i2c_id,
};
module_i2c_driver(aw88258_driver);

MODULE_DESCRIPTION("Awinic AW88258 (ID 0x1852) amplifier without DSP");
MODULE_LICENSE("GPL");
