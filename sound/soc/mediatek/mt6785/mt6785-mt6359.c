// SPDX-License-Identifier: GPL-2.0
//
// mt6785-mt6359.c  --  MT6785 + MT6359 ALSA SoC machine driver
//
// Front end / back end layout and the MTKAIF calibration follow
// mt8192-mt6359-rt1015-rt5682.c (Copyright (c) 2020 MediaTek Inc.,
// Jiaxin Yu <jiaxin.yu@mediatek.com>), which matches the MT6785 vendor
// machine driver mt6785-mt6359.c (Copyright (c) 2018 MediaTek Inc.) for the
// MT6359 part. The speaker amplifier hangs on I2S3 (codec from the device
// tree), I2S0 carries its I/V sense return; the headset jack is reported by
// the MT6359 ACCDET (mediatek,accdet).
//

#include <linux/input.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/pm_runtime.h>
#include <linux/regmap.h>
#include <sound/jack.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>

#include "../../codecs/mt6359.h"
#include "../../codecs/mt6359-accdet.h"
#include "../common/mtk-afe-platform-driver.h"
#include "../common/mtk-soc-card.h"
#include "../common/mtk-soundcard-driver.h"
#include "mt6785-afe-clk.h"
#include "mt6785-afe-common.h"
#include "mt6785-afe-gpio.h"

#define MT6785_MT6359_CARD_NAME "mt6785_mt6359"

/* MT6359 supports calibration phases 0 .. 42 */
#define MT6785_MTKAIF_NUM_PHASE 42
/* upper bound for one test-done poll, the vendor driver uses 10000 reads */
#define MT6785_MTKAIF_POLL_MAX 10000

static int mt6785_mt6359_mtkaif_calibration(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_component *cmpnt_afe =
		snd_soc_rtdcom_lookup(rtd, AFE_PCM_NAME);
	struct snd_soc_component *cmpnt_codec =
		snd_soc_rtd_to_codec(rtd, 0)->component;
	struct mtk_base_afe *afe = snd_soc_component_get_drvdata(cmpnt_afe);
	struct mt6785_afe_private *afe_priv = afe->platform_priv;
	int test_done[3], cycle[3], prev_cycle[3] = { 0 }, chosen[3];
	unsigned int monitor = 0;
	bool calib_ok = true;
	int phase, counter, i, ret;

	/* never touch AFE registers without power and clocks */
	ret = pm_runtime_resume_and_get(afe->dev);
	if (ret) {
		dev_err(afe->dev, "%s(), AFE power on failed %d, skipped\n",
			__func__, ret);
		return ret;
	}

	mt6785_afe_gpio_request(afe->dev, true, MT6785_DAI_ADDA, 1);
	mt6785_afe_gpio_request(afe->dev, true, MT6785_DAI_ADDA, 0);
	mt6785_afe_gpio_request(afe->dev, true, MT6785_DAI_ADDA_CH34, 1);
	mt6785_afe_gpio_request(afe->dev, true, MT6785_DAI_ADDA_CH34, 0);

	mt6359_mtkaif_calibration_enable(cmpnt_codec);

	/* set clock protocol 2 */
	regmap_update_bits(afe->regmap, AFE_AUD_PAD_TOP, 0xff, 0x38);
	regmap_update_bits(afe->regmap, AFE_AUD_PAD_TOP, 0xff, 0x39);

	/* set test type to synchronizer pulse */
	regmap_update_bits(afe_priv->topckgen, CKSYS_AUD_TOP_CFG, 0xffff, 0x4);

	afe_priv->mtkaif_calibration_num_phase = MT6785_MTKAIF_NUM_PHASE;
	for (i = 0; i < 3; i++)
		afe_priv->mtkaif_chosen_phase[i] = -1;

	for (phase = 0;
	     phase <= afe_priv->mtkaif_calibration_num_phase && calib_ok;
	     phase++) {
		mt6359_set_mtkaif_calibration_phase(cmpnt_codec,
						    phase, phase, phase);

		regmap_update_bits(afe_priv->topckgen, CKSYS_AUD_TOP_CFG,
				   0x1, 0x1);

		for (i = 0; i < 3; i++) {
			test_done[i] = 0;
			cycle[i] = -1;
		}
		counter = 0;
		while (!test_done[0] || !test_done[1] || !test_done[2]) {
			regmap_read(afe_priv->topckgen, CKSYS_AUD_TOP_MON,
				    &monitor);

			for (i = 0; i < 3; i++) {
				test_done[i] = (monitor >> (28 + i)) & 0x1;
				if (test_done[i])
					cycle[i] = (monitor >> (4 * i)) & 0xf;
			}

			/* handle if never test done */
			if (++counter > MT6785_MTKAIF_POLL_MAX) {
				dev_err(afe->dev,
					"%s(), test fail, cycle %d/%d/%d, monitor 0x%x\n",
					__func__, cycle[0], cycle[1], cycle[2],
					monitor);
				calib_ok = false;
				break;
			}
		}

		if (phase == 0)
			for (i = 0; i < 3; i++)
				prev_cycle[i] = cycle[i];

		for (i = 0; i < 3; i++) {
			if (cycle[i] != prev_cycle[i] &&
			    afe_priv->mtkaif_chosen_phase[i] < 0) {
				afe_priv->mtkaif_chosen_phase[i] = phase - 1;
				afe_priv->mtkaif_phase_cycle[i] = prev_cycle[i];
			}
		}

		regmap_update_bits(afe_priv->topckgen, CKSYS_AUD_TOP_CFG,
				   0x1, 0x0);

		if (afe_priv->mtkaif_chosen_phase[0] >= 0 &&
		    afe_priv->mtkaif_chosen_phase[1] >= 0 &&
		    afe_priv->mtkaif_chosen_phase[2] >= 0)
			break;
	}

	for (i = 0; i < 3; i++)
		chosen[i] = afe_priv->mtkaif_chosen_phase[i] < 0 ?
			    0 : afe_priv->mtkaif_chosen_phase[i];

	mt6359_set_mtkaif_calibration_phase(cmpnt_codec,
					    chosen[0], chosen[1], chosen[2]);

	/* disable rx fifo */
	regmap_update_bits(afe->regmap, AFE_AUD_PAD_TOP, 0xff, 0x38);

	mt6359_mtkaif_calibration_disable(cmpnt_codec);

	mt6785_afe_gpio_request(afe->dev, false, MT6785_DAI_ADDA, 1);
	mt6785_afe_gpio_request(afe->dev, false, MT6785_DAI_ADDA, 0);
	mt6785_afe_gpio_request(afe->dev, false, MT6785_DAI_ADDA_CH34, 1);
	mt6785_afe_gpio_request(afe->dev, false, MT6785_DAI_ADDA_CH34, 0);
	pm_runtime_put(afe->dev);

	/* typical result on lisbon: 3/3/3 or 4/3/3 */
	dev_info(afe->dev, "%s(), %s, mtkaif_chosen_phase[0/1/2]:%d/%d/%d\n",
		 __func__, calib_ok ? "done" : "FAILED",
		 afe_priv->mtkaif_chosen_phase[0],
		 afe_priv->mtkaif_chosen_phase[1],
		 afe_priv->mtkaif_chosen_phase[2]);

	return 0;
}

static struct snd_soc_jack mt6785_mt6359_jacks[1];

/*
 * Report the jack (input device and "Headphone Jack"/"Headset Mic Jack"
 * controls for UCM) without coupling it to the DAPM pins: the headphone path
 * stays usable without a plugged headset, e.g. for register tests.
 */
static int mt6785_mt6359_accdet_init(struct snd_soc_pcm_runtime *rtd)
{
	struct mtk_soc_card_data *soc_card_data = snd_soc_card_get_drvdata(rtd->card);
	struct snd_soc_jack *jack = &mt6785_mt6359_jacks[0];
	int ret;

	if (!soc_card_data->accdet)
		return 0;

	ret = snd_soc_card_jack_new(rtd->card, "Headset Jack",
				    SND_JACK_HEADSET | SND_JACK_BTN_0 |
				    SND_JACK_BTN_1 | SND_JACK_BTN_2, jack);
	if (ret)
		return dev_err_probe(rtd->dev, ret, "headset jack\n");

	snd_jack_add_new_kctl(jack->jack, "Headphone", SND_JACK_HEADPHONE);
	snd_jack_add_new_kctl(jack->jack, "Headset Mic", SND_JACK_MICROPHONE);

	/*
	 * The key codes come from the ACCDET driver (BTN_0 play/pause, BTN_1
	 * volume down, BTN_2 volume up), like the vendor kernel: middle
	 * band (mid..up) = up, upper band (up..down) = down.
	 */
	ret = mt6359_accdet_enable_jack_detect(soc_card_data->accdet, jack);
	if (ret)
		return dev_err_probe(rtd->dev, ret, "jack detection\n");
	return 0;
}

/* the jack belongs to the card: detach it before the card goes away */
static void mt6785_mt6359_exit(struct snd_soc_pcm_runtime *rtd)
{
	struct mtk_soc_card_data *soc_card_data = snd_soc_card_get_drvdata(rtd->card);

	if (soc_card_data->accdet)
		mt6359_accdet_enable_jack_detect(soc_card_data->accdet, NULL);
}

static int mt6785_mt6359_init(struct snd_soc_pcm_runtime *rtd)
{
	struct snd_soc_component *cmpnt_afe =
		snd_soc_rtdcom_lookup(rtd, AFE_PCM_NAME);
	struct snd_soc_component *cmpnt_codec =
		snd_soc_rtd_to_codec(rtd, 0)->component;
	struct mtk_base_afe *afe = snd_soc_component_get_drvdata(cmpnt_afe);
	struct mt6785_afe_private *afe_priv = afe->platform_priv;

	/* set mtkaif protocol */
	mt6359_set_mtkaif_protocol(cmpnt_codec,
				   MT6359_MTKAIF_PROTOCOL_2_CLK_P2);
	afe_priv->mtkaif_protocol = MTKAIF_PROTOCOL_2_CLK_P2;

	/*
	 * A failed calibration leaves phase 0 in place: capture may be
	 * distorted, playback is not affected. Do not fail the card for it.
	 */
	mt6785_mt6359_mtkaif_calibration(rtd);

	/* headset jack, optional: a missing ACCDET only costs the detection */
	mt6785_mt6359_accdet_init(rtd);

	return 0;
}

/* FE */
SND_SOC_DAILINK_DEFS(playback1,
		     DAILINK_COMP_ARRAY(COMP_CPU("DL1")),
		     DAILINK_COMP_ARRAY(COMP_DUMMY()),
		     DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(playback12,
		     DAILINK_COMP_ARRAY(COMP_CPU("DL12")),
		     DAILINK_COMP_ARRAY(COMP_DUMMY()),
		     DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(playback2,
		     DAILINK_COMP_ARRAY(COMP_CPU("DL2")),
		     DAILINK_COMP_ARRAY(COMP_DUMMY()),
		     DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(playback3,
		     DAILINK_COMP_ARRAY(COMP_CPU("DL3")),
		     DAILINK_COMP_ARRAY(COMP_DUMMY()),
		     DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(capture1,
		     DAILINK_COMP_ARRAY(COMP_CPU("UL1")),
		     DAILINK_COMP_ARRAY(COMP_DUMMY()),
		     DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(capture2,
		     DAILINK_COMP_ARRAY(COMP_CPU("UL2")),
		     DAILINK_COMP_ARRAY(COMP_DUMMY()),
		     DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(capture3,
		     DAILINK_COMP_ARRAY(COMP_CPU("UL3")),
		     DAILINK_COMP_ARRAY(COMP_DUMMY()),
		     DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(capture6,
		     DAILINK_COMP_ARRAY(COMP_CPU("UL6")),
		     DAILINK_COMP_ARRAY(COMP_DUMMY()),
		     DAILINK_COMP_ARRAY(COMP_EMPTY()));

/* BE, codecs are filled in from the device tree (dai-link subnodes) */
SND_SOC_DAILINK_DEFS(primary_codec,
		     DAILINK_COMP_ARRAY(COMP_CPU("ADDA")),
		     DAILINK_COMP_ARRAY(COMP_EMPTY()),
		     DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(primary_codec_ch34,
		     DAILINK_COMP_ARRAY(COMP_CPU("ADDA_CH34")),
		     DAILINK_COMP_ARRAY(COMP_EMPTY()),
		     DAILINK_COMP_ARRAY(COMP_EMPTY()));

/* modem speech: hostless front end, PCM 2 is the interface of MD1 */
SND_SOC_DAILINK_DEFS(hostless_speech,
		     DAILINK_COMP_ARRAY(COMP_CPU("Hostless Speech DAI")),
		     DAILINK_COMP_ARRAY(COMP_DUMMY()),
		     DAILINK_COMP_ARRAY(COMP_EMPTY()));

SND_SOC_DAILINK_DEFS(pcm2,
		     DAILINK_COMP_ARRAY(COMP_CPU("PCM 2")),
		     DAILINK_COMP_ARRAY(COMP_DUMMY()),
		     DAILINK_COMP_ARRAY(COMP_EMPTY()));

/* speaker amplifier (codec from the device tree, dummy without dai-link node) */
SND_SOC_DAILINK_DEFS(i2s3,
		     DAILINK_COMP_ARRAY(COMP_CPU("I2S3")),
		     DAILINK_COMP_ARRAY(COMP_DUMMY()),
		     DAILINK_COMP_ARRAY(COMP_EMPTY()));

/* I/V sense return of the amplifier; also the I2S3 -> I2S0 loopback */
SND_SOC_DAILINK_DEFS(i2s0,
		     DAILINK_COMP_ARRAY(COMP_CPU("I2S0")),
		     DAILINK_COMP_ARRAY(COMP_DUMMY()),
		     DAILINK_COMP_ARRAY(COMP_EMPTY()));

/*
 * I2S BEs run like the vendor kernel: 48 kHz, 16 bit (BCK = 32 fs), stereo,
 * the AFE provides BCK/LRCK.
 */
static int mt6785_i2s_hw_params_fixup(struct snd_soc_pcm_runtime *rtd,
				      struct snd_pcm_hw_params *params)
{
	hw_param_interval(params, SNDRV_PCM_HW_PARAM_RATE)->min = 48000;
	hw_param_interval(params, SNDRV_PCM_HW_PARAM_RATE)->max = 48000;
	hw_param_interval(params, SNDRV_PCM_HW_PARAM_CHANNELS)->min = 2;
	hw_param_interval(params, SNDRV_PCM_HW_PARAM_CHANNELS)->max = 2;
	snd_mask_none(hw_param_mask(params, SNDRV_PCM_HW_PARAM_FORMAT));
	snd_mask_set_format(hw_param_mask(params, SNDRV_PCM_HW_PARAM_FORMAT),
			    SNDRV_PCM_FORMAT_S16_LE);
	return 0;
}

#define MT6785_FE_LINK(_name, _link, _playback)			\
	{								\
		.name = _name,						\
		.stream_name = _name,					\
		.trigger = { SND_SOC_DPCM_TRIGGER_PRE,			\
			     SND_SOC_DPCM_TRIGGER_PRE },		\
		.dynamic = 1,						\
		.playback_only = _playback,				\
		.capture_only = !(_playback),				\
		.ops = (_playback) ?					\
		       &mtk_soundcard_common_playback_ops :		\
		       &mtk_soundcard_common_capture_ops,		\
		SND_SOC_DAILINK_REG(_link),				\
	}

/*
 * PCM device numbers follow this order:
 *  0 Playback_1 (DL1)   1 Playback_12 (DL12)  2 Playback_2 (DL2)
 *  3 Playback_3 (DL3)   4 Capture_1 (VUL12)   5 Capture_2 (AWB)
 *  6 Capture_3 (VUL2)   7 Capture_6 (VUL4)    8 Hostless_Speech (no buffer)
 */
static struct snd_soc_dai_link mt6785_mt6359_dai_links[] = {
	/* Front End DAI links */
	MT6785_FE_LINK("Playback_1", playback1, 1),
	MT6785_FE_LINK("Playback_12", playback12, 1),
	MT6785_FE_LINK("Playback_2", playback2, 1),
	MT6785_FE_LINK("Playback_3", playback3, 1),
	MT6785_FE_LINK("Capture_1", capture1, 0),
	MT6785_FE_LINK("Capture_2", capture2, 0),
	MT6785_FE_LINK("Capture_3", capture3, 0),
	MT6785_FE_LINK("Capture_6", capture6, 0),
	{
		/* opened during a call: routes the modem PCM to the codec */
		.name = "Hostless_Speech",
		.stream_name = "Hostless_Speech",
		.trigger = { SND_SOC_DPCM_TRIGGER_PRE,
			     SND_SOC_DPCM_TRIGGER_PRE },
		.dynamic = 1,
		.ignore_suspend = 1,
		SND_SOC_DAILINK_REG(hostless_speech),
	},
	/* Back End DAI links */
	{
		.name = "Primary Codec",
		.no_pcm = 1,
		.ignore_suspend = 1,
		.init = mt6785_mt6359_init,
		.exit = mt6785_mt6359_exit,
		SND_SOC_DAILINK_REG(primary_codec),
	},
	{
		.name = "Primary Codec CH34",
		.no_pcm = 1,
		.ignore_suspend = 1,
		SND_SOC_DAILINK_REG(primary_codec_ch34),
	},
	{
		.name = "PCM 2",
		.no_pcm = 1,
		.ignore_suspend = 1,
		SND_SOC_DAILINK_REG(pcm2),
	},
	{
		.name = "I2S3",
		.no_pcm = 1,
		.playback_only = 1,
		.ignore_suspend = 1,
		.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
			   SND_SOC_DAIFMT_CBC_CFC,
		.be_hw_params_fixup = mt6785_i2s_hw_params_fixup,
		SND_SOC_DAILINK_REG(i2s3),
	},
	{
		.name = "I2S0",
		.no_pcm = 1,
		.capture_only = 1,
		.ignore_suspend = 1,
		.dai_fmt = SND_SOC_DAIFMT_I2S | SND_SOC_DAIFMT_NB_NF |
			   SND_SOC_DAIFMT_CBC_CFC,
		.be_hw_params_fixup = mt6785_i2s_hw_params_fixup,
		SND_SOC_DAILINK_REG(i2s0),
	},
};

static const struct snd_soc_dapm_widget mt6785_mt6359_widgets[] = {
	SND_SOC_DAPM_HP("Headphone", NULL),
	SND_SOC_DAPM_SPK("Earpiece", NULL),
	SND_SOC_DAPM_SPK("Speaker", NULL),
	SND_SOC_DAPM_MIC("Main Mic", NULL),
	SND_SOC_DAPM_MIC("Sub Mic", NULL),
	SND_SOC_DAPM_MIC("Headset Mic", NULL),
};

static const struct snd_kcontrol_new mt6785_mt6359_controls[] = {
	SOC_DAPM_PIN_SWITCH("Headphone"),
	SOC_DAPM_PIN_SWITCH("Earpiece"),
	SOC_DAPM_PIN_SWITCH("Speaker"),
	SOC_DAPM_PIN_SWITCH("Main Mic"),
	SOC_DAPM_PIN_SWITCH("Sub Mic"),
	SOC_DAPM_PIN_SWITCH("Headset Mic"),
};

static struct snd_soc_card mt6785_mt6359_card = {
	.name = MT6785_MT6359_CARD_NAME,
	.owner = THIS_MODULE,
	.dai_link = mt6785_mt6359_dai_links,
	.num_links = ARRAY_SIZE(mt6785_mt6359_dai_links),
	.controls = mt6785_mt6359_controls,
	.num_controls = ARRAY_SIZE(mt6785_mt6359_controls),
	.dapm_widgets = mt6785_mt6359_widgets,
	.num_dapm_widgets = ARRAY_SIZE(mt6785_mt6359_widgets),
};

static int mt6785_mt6359_soc_card_probe(struct mtk_soc_card_data *soc_card_data,
					bool legacy)
{
	struct snd_soc_card *card = soc_card_data->card_data->card;
	int ret;

	/* only the device tree link description is supported */
	if (legacy)
		return dev_err_probe(card->dev, -EINVAL,
				     "audio-routing and dai-link nodes required\n");

	ret = mt6785_afe_gpio_init(card->dev);
	if (ret)
		return dev_err_probe(card->dev, ret, "%s init gpio error\n",
				     __func__);

	return 0;
}

static const unsigned int mt6785_pcm_playback_channels[] = { 1, 2 };
static const unsigned int mt6785_pcm_playback_rates[] = { 48000 };

static const unsigned int mt6785_pcm_capture_channels[] = { 1, 2, 4 };
static const unsigned int mt6785_pcm_capture_rates[] = {
	8000, 16000, 32000, 48000
};

static const struct mtk_pcm_constraints_data mt6785_pcm_constraints[MTK_CONSTRAINT_CAPTURE + 1] = {
	[MTK_CONSTRAINT_PLAYBACK] = {
		.channels = &(const struct snd_pcm_hw_constraint_list) {
			.list = mt6785_pcm_playback_channels,
			.count = ARRAY_SIZE(mt6785_pcm_playback_channels)
		},
		.rates = &(const struct snd_pcm_hw_constraint_list) {
			.list = mt6785_pcm_playback_rates,
			.count = ARRAY_SIZE(mt6785_pcm_playback_rates)
		}
	},
	[MTK_CONSTRAINT_CAPTURE] = {
		.channels = &(const struct snd_pcm_hw_constraint_list) {
			.list = mt6785_pcm_capture_channels,
			.count = ARRAY_SIZE(mt6785_pcm_capture_channels)
		},
		.rates = &(const struct snd_pcm_hw_constraint_list) {
			.list = mt6785_pcm_capture_rates,
			.count = ARRAY_SIZE(mt6785_pcm_capture_rates)
		}
	}
};

static const struct mtk_soundcard_pdata mt6785_mt6359_pdata = {
	.card_name = MT6785_MT6359_CARD_NAME,
	.card_data = &(struct mtk_platform_card_data) {
		.card = &mt6785_mt6359_card,
		.jacks = mt6785_mt6359_jacks,
		.num_jacks = ARRAY_SIZE(mt6785_mt6359_jacks),
		.pcm_constraints = mt6785_pcm_constraints,
		.num_pcm_constraints = ARRAY_SIZE(mt6785_pcm_constraints),
	},
	.soc_probe = mt6785_mt6359_soc_card_probe,
};

static const struct of_device_id mt6785_mt6359_dt_match[] = {
	{
		.compatible = "mediatek,mt6785-mt6359-sound",
		.data = &mt6785_mt6359_pdata,
	},
	{}
};
MODULE_DEVICE_TABLE(of, mt6785_mt6359_dt_match);

static const struct dev_pm_ops mt6785_mt6359_pm_ops = {
	.poweroff = snd_soc_poweroff,
	.restore = snd_soc_resume,
};

static struct platform_driver mt6785_mt6359_driver = {
	.driver = {
		.name = "mt6785_mt6359",
		.of_match_table = mt6785_mt6359_dt_match,
		.pm = &mt6785_mt6359_pm_ops,
	},
	.probe = mtk_soundcard_common_probe,
};

module_platform_driver(mt6785_mt6359_driver);

MODULE_DESCRIPTION("MT6785-MT6359 ALSA SoC machine driver");
MODULE_LICENSE("GPL");
