/* SPDX-License-Identifier: GPL-2.0 */
/*
 * mt6785-afe-mtcmos.h  --  MT6785 audio power domain without a genpd provider
 */

#ifndef _MT6785_AFE_MTCMOS_H_
#define _MT6785_AFE_MTCMOS_H_

struct mtk_base_afe;

int mt6785_afe_mtcmos_init(struct mtk_base_afe *afe);
int mt6785_afe_mtcmos_on(struct mtk_base_afe *afe);
int mt6785_afe_mtcmos_off(struct mtk_base_afe *afe);

#endif
