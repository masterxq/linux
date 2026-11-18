/* SPDX-License-Identifier: GPL-2.0 */
/*
 * mt6785-afe-common.h  --  Mediatek 6785 audio driver definitions
 *
 * Derived from mt8192-afe-common.h (Copyright (c) 2020 MediaTek Inc.,
 * Shane Chien) and the MT6785 vendor driver (Copyright (c) 2017 MediaTek Inc.,
 * Eason Yen). The MT6785 AFE register map matches MT8192 except for the
 * blocks MT8192 added later (DL9, I2S6..9, tiny connections).
 */

#ifndef _MT_6785_AFE_COMMON_H_
#define _MT_6785_AFE_COMMON_H_

#include <linux/list.h>
#include <linux/regmap.h>
#include <sound/soc.h>

#include "../common/mtk-base-afe.h"
#include "mt6785-reg.h"

enum {
	MT6785_MEMIF_DL1,
	MT6785_MEMIF_DL12,
	MT6785_MEMIF_DL2,
	MT6785_MEMIF_DL3,
	MT6785_MEMIF_DL4,
	MT6785_MEMIF_DL5,
	MT6785_MEMIF_DL6,
	MT6785_MEMIF_DL7,
	MT6785_MEMIF_DL8,
	MT6785_MEMIF_DAI,
	MT6785_MEMIF_DAI2,
	MT6785_MEMIF_MOD_DAI,
	MT6785_MEMIF_VUL12,
	MT6785_MEMIF_VUL2,
	MT6785_MEMIF_VUL3,
	MT6785_MEMIF_VUL4,
	MT6785_MEMIF_VUL5,
	MT6785_MEMIF_VUL6,
	MT6785_MEMIF_AWB,
	MT6785_MEMIF_AWB2,
	MT6785_MEMIF_HDMI,
	MT6785_MEMIF_NUM,
	MT6785_DAI_ADDA = MT6785_MEMIF_NUM,
	MT6785_DAI_ADDA_CH34,
	MT6785_DAI_AP_DMIC,
	MT6785_DAI_AP_DMIC_CH34,
	MT6785_DAI_I2S_0,
	MT6785_DAI_I2S_1,
	MT6785_DAI_I2S_2,
	MT6785_DAI_I2S_3,
	MT6785_DAI_I2S_5,
	MT6785_DAI_PCM_1,
	MT6785_DAI_PCM_2,
	MT6785_DAI_HOSTLESS_LPBK,
	MT6785_DAI_HOSTLESS_SPEECH,
	MT6785_DAI_NUM,
};

enum {
	MT6785_IRQ_0,
	MT6785_IRQ_1,
	MT6785_IRQ_2,
	MT6785_IRQ_3,
	MT6785_IRQ_4,
	MT6785_IRQ_5,
	MT6785_IRQ_6,
	MT6785_IRQ_7,
	MT6785_IRQ_8,
	MT6785_IRQ_9,
	MT6785_IRQ_10,
	MT6785_IRQ_11,
	MT6785_IRQ_12,
	MT6785_IRQ_13,
	MT6785_IRQ_14,
	MT6785_IRQ_15,
	MT6785_IRQ_16,
	MT6785_IRQ_17,
	MT6785_IRQ_18,
	MT6785_IRQ_19,
	MT6785_IRQ_20,
	MT6785_IRQ_21,
	MT6785_IRQ_22,
	MT6785_IRQ_23,
	MT6785_IRQ_24,
	MT6785_IRQ_25,
	MT6785_IRQ_26,
	MT6785_IRQ_31,	/* used only for TDM (HDMI memif) */
	MT6785_IRQ_NUM,
};

enum {
	MTKAIF_PROTOCOL_1 = 0,
	MTKAIF_PROTOCOL_2,
	MTKAIF_PROTOCOL_2_CLK_P2,
};

enum {
	MTK_AFE_ADDA_DL_GAIN_MUTE = 0,
	MTK_AFE_ADDA_DL_GAIN_NORMAL = 0xf74f,
	/* SA suggest apply -0.3db to audio/speech path */
};

/* MCLK */
enum {
	MT6785_I2S0_MCK = 0,
	MT6785_I2S1_MCK,
	MT6785_I2S2_MCK,
	MT6785_I2S3_MCK,
	MT6785_I2S4_MCK,
	MT6785_I2S4_BCK,
	MT6785_I2S5_MCK,
	MT6785_MCK_NUM,
};

struct clk;

struct mt6785_afe_private {
	struct clk **clk;
	struct regmap *topckgen;
	struct regmap *apmixedsys;
	struct regmap *infracfg;
	struct regmap *scpsys;
	int stf_positive_gain_db;

	/* audio MTCMOS switched by the driver (no genpd provider) */
	bool self_mtcmos;
	bool mtcmos_on;
	unsigned int mtcmos_cycles;
	int mtcmos_err;
	int pm_runtime_bypass_reg_ctl;

	/* dai */
	bool dai_on[MT6785_DAI_NUM];
	void *dai_priv[MT6785_DAI_NUM];

	/* adda */
	int mtkaif_protocol;
	int mtkaif_chosen_phase[4];
	int mtkaif_phase_cycle[4];
	int mtkaif_calibration_num_phase;
	int mtkaif_dmic;
	int mtkaif_dmic_ch34;
	int mtkaif_adda6_only;

	/* mck */
	int mck_rate[MT6785_MCK_NUM];
};

int mt6785_dai_adda_register(struct mtk_base_afe *afe);
int mt6785_dai_i2s_register(struct mtk_base_afe *afe);
int mt6785_dai_pcm_register(struct mtk_base_afe *afe);
int mt6785_dai_hostless_register(struct mtk_base_afe *afe);

int mt6785_dai_i2s_set_share(struct mtk_base_afe *afe, const char *main_i2s_name,
			     const char *secondary_i2s_name);

unsigned int mt6785_general_rate_transform(struct device *dev,
					   unsigned int rate);
unsigned int mt6785_rate_transform(struct device *dev,
				   unsigned int rate, int aud_blk);

int mt6785_dai_set_priv(struct mtk_base_afe *afe, int id,
			int priv_size, const void *priv_data);

#endif
