/* SPDX-License-Identifier: GPL-2.0 */
/*
 * mt6785-afe-clk.h  --  Mediatek 6785 afe clock ctrl definition
 *
 * Derived from mt8192-afe-clk.h (Copyright (c) 2020 MediaTek Inc.,
 * Shane Chien); clock set and topckgen offsets as in the MT6785 vendor
 * driver mt6785-afe-clk.[ch] (Copyright (c) 2018 MediaTek Inc.).
 */

#ifndef _MT6785_AFE_CLOCK_CTRL_H_
#define _MT6785_AFE_CLOCK_CTRL_H_

/* topckgen, used for the MTKAIF calibration (same offsets as MT8192) */
#define CKSYS_AUD_TOP_CFG 0x032c
#define CKSYS_AUD_TOP_MON 0x0330

#define APLL1_W_NAME "APLL1"
#define APLL2_W_NAME "APLL2"

enum {
	MT6785_APLL1 = 0,
	MT6785_APLL2,
};

enum {
	CLK_AFE = 0,
	CLK_TML,
	CLK_APLL22M,
	CLK_APLL24M,
	CLK_APLL1_TUNER,
	CLK_APLL2_TUNER,
	CLK_NLE,
	CLK_INFRA_SYS_AUDIO,
	CLK_INFRA_AUDIO_26M,
	CLK_MUX_AUDIO,
	CLK_MUX_AUDIOINTBUS,
	CLK_TOP_MAINPLL_D2_D4,
	/* apll related mux */
	CLK_TOP_MUX_AUD_1,
	CLK_TOP_APLL1_CK,
	CLK_TOP_MUX_AUD_2,
	CLK_TOP_APLL2_CK,
	CLK_TOP_MUX_AUD_ENG1,
	CLK_TOP_APLL1_D8,
	CLK_TOP_MUX_AUD_ENG2,
	CLK_TOP_APLL2_D8,
	CLK_TOP_MUX_AUDIO_H,
	CLK_TOP_I2S0_M_SEL,
	CLK_TOP_I2S1_M_SEL,
	CLK_TOP_I2S2_M_SEL,
	CLK_TOP_I2S3_M_SEL,
	CLK_TOP_I2S4_M_SEL,
	CLK_TOP_I2S5_M_SEL,
	CLK_TOP_APLL12_DIV0,
	CLK_TOP_APLL12_DIV1,
	CLK_TOP_APLL12_DIV2,
	CLK_TOP_APLL12_DIV3,
	CLK_TOP_APLL12_DIV4,
	CLK_TOP_APLL12_DIVB,
	CLK_TOP_APLL12_DIV5,
	CLK_CLK26M,
	CLK_NUM
};

struct mtk_base_afe;

int mt6785_init_clock(struct mtk_base_afe *afe);
int mt6785_afe_enable_clock(struct mtk_base_afe *afe);
void mt6785_afe_disable_clock(struct mtk_base_afe *afe);

int mt6785_apll1_enable(struct mtk_base_afe *afe);
void mt6785_apll1_disable(struct mtk_base_afe *afe);

int mt6785_apll2_enable(struct mtk_base_afe *afe);
void mt6785_apll2_disable(struct mtk_base_afe *afe);

int mt6785_get_apll_rate(struct mtk_base_afe *afe, int apll);
int mt6785_get_apll_by_rate(struct mtk_base_afe *afe, int rate);
int mt6785_get_apll_by_name(struct mtk_base_afe *afe, const char *name);

int mt6785_mck_enable(struct mtk_base_afe *afe, int mck_id, int rate);
void mt6785_mck_disable(struct mtk_base_afe *afe, int mck_id);

int mt6785_set_audio_int_bus_parent(struct mtk_base_afe *afe,
				    int clk_id);

#endif
