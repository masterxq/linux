/* SPDX-License-Identifier: GPL-2.0 */
/*
 * mt6785-afe-gpio.h  --  Mediatek 6785 afe gpio ctrl definition
 *
 * Derived from mt8192-afe-gpio.h (Copyright (c) 2020 MediaTek Inc.,
 * Shane Chien <shane.chien@mediatek.com>)
 */

#ifndef _MT6785_AFE_GPIO_H_
#define _MT6785_AFE_GPIO_H_

struct device;

int mt6785_afe_gpio_init(struct device *dev);

int mt6785_afe_gpio_request(struct device *dev, bool enable,
			    int dai, int uplink);

#endif
