# SPDX-License-Identifier: GPL-2.0

mtk-camsv-objs += mtk_camsv_hw.o
mtk-camsv-objs += mtk_camsv_video.o
obj-$(CONFIG_MTK_CAMSV) += mtk-camsv.o

mtk-seninf-objs += mtk_seninf.o
obj-$(CONFIG_MTK_SENINF30) += mtk-seninf.o

mtk-dip-mt6785-y += mtk_dip_bq.o
mtk-dip-mt6785-y += mtk_dip_hw.o
mtk-dip-mt6785-y += mtk_dip_lsc.o
mtk-dip-mt6785-y += mtk_dip_v4l2.o
mtk-dip-mt6785-y += mtk_dip_params.o
mtk-dip-mt6785-y += mtk_dip_profile.o
mtk-dip-mt6785-y += mtk_dip_tile.o
mtk-dip-mt6785-$(CONFIG_MTK_DIP_MT6785_KUNIT_TEST) += mtk_dip_tile_test.o
mtk-dip-mt6785-$(CONFIG_MTK_DIP_MT6785_KUNIT_TEST) += mtk_dip_params_test.o
mtk-dip-mt6785-$(CONFIG_MTK_DIP_MT6785_KUNIT_TEST) += mtk_dip_lsc_test.o
mtk-dip-mt6785-$(CONFIG_MTK_DIP_MT6785_KUNIT_TEST) += mtk_dip_bq_test.o
mtk-dip-mt6785-$(CONFIG_MTK_DIP_MT6785_KUNIT_TEST) += mtk_dip_bayer_test.o
obj-$(CONFIG_MTK_DIP_MT6785) += mtk-dip-mt6785.o
