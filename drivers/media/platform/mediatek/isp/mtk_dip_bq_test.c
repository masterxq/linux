// SPDX-License-Identifier: GPL-2.0-only
/*
 * KUnit test of the image quality blocks of the MT6785 DIP: translation of
 * the fields into register words, the derived slopes of the edge enhancement
 * curve, the fixed bits of the second noise reduction stage, range checks
 * and the module state of the parameter buffer.
 *
 * The blocks hold synthetic values (element k of a block: 17 k + 5, modulo
 * the range of its field); the expected words were computed with an
 * independent implementation of the field layout and the slope rule.
 */

#include <kunit/test.h>
#include <linux/bits.h>
#include <linux/stddef.h>
#include <linux/string.h>

#include "mtk_dip.h"

static const struct mtk_dip_params_ee mtk_dip_bq_test_ee0 = {
	.header = { .type = MTK_DIP_PARAMS_BLOCK_EE,
		    .size = sizeof(struct mtk_dip_params_ee) },
	.band_offset = { 5, 22, 39 },
	.band_slope = { 56, 73, 90 },
	.band_ratio = 11,
	.band_core_th = { 124, 141, 158 },
	.core_th = 175,
	.band_gain = { 0, 17, 2 },
	.luma_gain = { 243, 260, 277, 294, 311, 328, 345 },
	.radial_gain = { 106, 123 },
	.radial_response_ratio = 12,
	.gain_curve_radial = 1,
	.gain_curve_x = { 10, 60, 120, 200 },
	.gain_curve_y = { 498, 515, 532, 549 },
	.gain_curve_y_end = -300,
	.gain_curve_aux = 566,
	.gain_curve_th_over = 71,
	.gain_curve_th_under = 88,
	.gain_curve_th_min = 105,
	.response_smooth = 2,
	.overshoot_clip = 3,
	.dot_reduction = 156,
	.dot_th = 173,
	.response_clip = 190,
	.response_clip_luma_th = 207,
	.response_clip_luma_lower = 224,
	.response_clip_luma_upper = 241,
	.master_gain_neg = 2,
	.master_gain_pos = 19,
	.strength_upper = 36,
	.strength_lower = 53,
	.strength_edge_slope = 70,
	.strength_edge_offset = 87,
	.strength_response_slope = 104,
	.strength_response_offset = 121,
	.luma_limit_diff = 138,
	.luma_contrast_level = 3,
	.luma_min_sel = 4,
	.luma_max_sel = 5,
	.chroma_contrast_level = 6,
	.chroma_min_sel = 3,
	.chroma_max_sel = 0,
	.chroma_boost_limit_upper = 1,
	.chroma_boost_limit_lower = 18,
	.chroma_boost_gain = 35,
	.chroma_boost_enable = 0,
	.chroma_boost_y_const = 69,
	.chroma_boost_y_offset = 86,
	.chroma_boost_y_offset_sel = 3,
	.regions_enable = 0,
	.region_enable = { 1, 0, 1 },
	.region_radius_slope2 = { 188, 205, 222 },
	.region_radius_slope = { 239, 256, 273 },
	.region_radius_range = { 34, 51, 4 },
	.region_angle_slope = { 341, 358, 375 },
	.region_angle_range = { 8, 25, 42 },
	.region_angle_center = { 187, 204, 221 },
	.region_radius_center = { 238, 255, 16 },
	.region_lowpass_gain = { 33, 50, 3 },
	.region_lowpass_enable = { 0, 1, 0 },
	.region_edge_enable = { 1, 0, 1 },
	.region_edge_slope = { 58, 11, 28 },
	.region_edge_th = { 45, 62, 15 },
	.region_confidence_gain = { 0, 1, 2 },
	.region_gain = { 83, 100, 117 },
};

static const u32 mtk_dip_bq_test_ee0_words[] = {
	0x00271605, 0x0b5a4938, 0xaf9e8d7c, 0x00021100, 0x115410f3, 0x1484dd26,
	0x566c7b6a, 0x01f20a7f, 0x02033c05, 0x02147805, 0x0225c803, 0x02360080,
	0x00695847, 0x00ad9c32, 0xf1e0cfbe, 0x00001302, 0x00003524, 0x79685746,
	0x0365438a, 0x00231201, 0x00035645, 0x4bc0ef22, 0xeebb2155, 0x530b7aa1,
	0x0cd10033, 0xffcc6566, 0x641f8b72, 0x4de11104, 0x10dda977, 0x7523dc83,
};

static const struct mtk_dip_params_ee mtk_dip_bq_test_ee1 = {
	.header = { .type = MTK_DIP_PARAMS_BLOCK_EE,
		    .size = sizeof(struct mtk_dip_params_ee) },
	.band_offset = { 8, 25, 42 },
	.band_slope = { 59, 76, 93 },
	.band_ratio = 14,
	.band_core_th = { 127, 144, 161 },
	.core_th = 178,
	.band_gain = { 3, 20, 5 },
	.luma_gain = { 246, 263, 280, 297, 314, 331, 348 },
	.radial_gain = { 109, 126 },
	.radial_response_ratio = 15,
	.gain_curve_radial = 0,
	.gain_curve_x = { 1, 2, 3, 254 },
	.gain_curve_y = { 501, 518, 535, 552 },
	.gain_curve_y_end = 3071,
	.gain_curve_aux = 569,
	.gain_curve_th_over = 74,
	.gain_curve_th_under = 91,
	.gain_curve_th_min = 108,
	.response_smooth = 5,
	.overshoot_clip = 6,
	.dot_reduction = 159,
	.dot_th = 176,
	.response_clip = 193,
	.response_clip_luma_th = 210,
	.response_clip_luma_lower = 227,
	.response_clip_luma_upper = 244,
	.master_gain_neg = 5,
	.master_gain_pos = 22,
	.strength_upper = 39,
	.strength_lower = 56,
	.strength_edge_slope = 73,
	.strength_edge_offset = 90,
	.strength_response_slope = 107,
	.strength_response_offset = 124,
	.luma_limit_diff = 141,
	.luma_contrast_level = 6,
	.luma_min_sel = 7,
	.luma_max_sel = 0,
	.chroma_contrast_level = 1,
	.chroma_min_sel = 2,
	.chroma_max_sel = 3,
	.chroma_boost_limit_upper = 4,
	.chroma_boost_limit_lower = 21,
	.chroma_boost_gain = 38,
	.chroma_boost_enable = 1,
	.chroma_boost_y_const = 72,
	.chroma_boost_y_offset = 89,
	.chroma_boost_y_offset_sel = 2,
	.regions_enable = 1,
	.region_enable = { 0, 1, 0 },
	.region_radius_slope2 = { 191, 208, 225 },
	.region_radius_slope = { 242, 259, 276 },
	.region_radius_range = { 37, 54, 7 },
	.region_angle_slope = { 344, 361, 378 },
	.region_angle_range = { 11, 28, 45 },
	.region_angle_center = { 190, 207, 224 },
	.region_radius_center = { 241, 2, 19 },
	.region_lowpass_gain = { 36, 53, 6 },
	.region_lowpass_enable = { 1, 0, 1 },
	.region_edge_enable = { 0, 1, 0 },
	.region_edge_slope = { 61, 14, 31 },
	.region_edge_th = { 48, 1, 18 },
	.region_confidence_gain = { 3, 4, 5 },
	.region_gain = { 86, 103, 120 },
};

static const u32 mtk_dip_bq_test_ee1_words[] = {
	0x002a1908, 0x0e5d4c3b, 0xb2a1907f, 0x00051403, 0x11841cf6, 0x14b4e929,
	0x570f7e6d, 0x01f5017f, 0x0206027f, 0x0217037f, 0x0228fe01, 0x0239007f,
	0x006c5b4a, 0x00b09f65, 0xf4e3d2c1, 0x00001605, 0x00003827, 0x7c6b5a49,
	0x3210768d, 0x01261504, 0x00025948, 0x8bf0f225, 0xf1be2d58, 0x563c3d64,
	0x4d010336, 0x02cf7169, 0x67404eb5, 0x0e111407, 0x13e0b57a, 0x78549f46,
};

static const struct mtk_dip_params_ycnr mtk_dip_bq_test_ycnr = {
	.header = { .type = MTK_DIP_PARAMS_BLOCK_YCNR,
		    .size = sizeof(struct mtk_dip_params_ycnr) },
	.chroma_enable = 1,
	.luma_enable = 1,
	.luma_filter_sel = { 1, 0, 1, 2 },
	.radial_link = 1,
	.filter_mode = 0,
	.chroma_smooth_edge = 1,
	.chroma_filter = 0,
	.chroma_smooth_edge_th = 7,
	.center_gain_lo_th = 0,
	.center_gain_hi_th = 17,
	.luma_slope_v_th = 226,
	.luma_slope_h_th = 243,
	.luma_vert_act_th = 4,
	.luma_aux_gain_th = 21,
	.luma_vert_sigma = 38,
	.luma_scale_x = { 55, 72, 89, 106, 123, 140, 157, 174 },
	.luma_scale_y = { 31, 16, 1, 18, 3, 20, 5, 22, 7 },
	.luma_scale_slope = { 24, 9, 26, 11, 28, 13, 30, 15, 0 },
	.chroma_scale_x = { 241, 2, 19 },
	.chroma_scale_y = { 4, 21, 6, 23 },
	.chroma_scale_slope = { 8, 25, 10, 27 },
	.luma_act_y = { 172, 189, 206, 223 },
	.luma_act_slope = { 48, 1, 18, 35 },
	.luma_act_x = { 52, 69, 86 },
	.radial_x = { 103, 120, 137 },
	.radial_y = { 26, 43, 60, 13 },
	.radial_slope = { 30, 47, 0, 17 },
	.radial_chroma_gain = 2,
	.radial_scale_gain = 3,
	.luma_range_v = {
		{ 68, 85, 102, 119 },
		{ 136, 153, 170, 187 },
		{ 204, 221, 238, 255 },
		{ 16, 33, 50, 67 },
	},
	.chroma_aux_gain_th = 20,
	.chroma_luma_diff_th = 101,
	.luma_range_h = {
		{ 118, 135, 152, 169 },
		{ 186, 203, 220, 237 },
		{ 254, 15, 32, 49 },
		{ 66, 83, 100, 117 },
	},
	.act_radial_gain = 6,
	.act_diff_hi_th = 23,
	.act_diff_gain = 8,
	.act_diff_lo_th = 25,
	.chroma_range_v = { 202, 219, 236, 253 },
	.luma_act_center_offset = 14,
	.luma_act_center_gain = 31,
	.luma_act_center_th = 48,
	.chroma_range_h = { 65, 82, 99, 116 },
	.luma_range_ratio_th = { 5, 6, 7, 8 },
	.luma_range_ratio_slope = { 1, 2, 3, 0 },
	.luma_hf_weight = { 13, 30, 15, 0 },
	.luma_hf_core_th = 81,
	.luma_hf_core_slope = 2,
	.luma_hf_clip = 115,
	.hf_balance_mode = 0,
	.luma_hf_act_x = { 149, 166, 183, 200 },
	.luma_hf_act_y = { 89, 106, 123, 12, 29 },
	.luma_hf_act_slope = { 46, 63, 16, 33, 50 },
	.act_blend_base = 3,
	.chroma_dither = { 20, 5 },
	.center_gain_lo_th_lpf = 22,
	.center_gain_hi_th_lpf = 7,
	.luma_hf_luma_x = { 216, 233 },
	.luma_hf_luma_y = { 122, 11, 28 },
	.luma_hf_luma_slope = { 13, 30, 15 },
};

static const u32 mtk_dip_bq_test_ycnr_words[] = {
	0x01000913, 0x07000100, 0xf3e21100, 0x00261504, 0x6a594837, 0x1201101f,
	0x0b1a0918, 0x001302f1, 0x17061504, 0x1b0a1908, 0xdfcebdac, 0x23120130,
	0x00897867, 0x0d3c2b1a, 0x11002f1e, 0x77665544, 0x00651400, 0xbbaa9988,
	0x00000032, 0xffeeddcc, 0x43322110, 0xa9988776, 0xeddccbba, 0x31200ffe,
	0x00000000, 0x00000000, 0x00000000, 0x19801706, 0x75645342, 0xfdecdbca,
	0x00301f0e, 0x74635241, 0x03218765, 0x000f1e0d, 0x00730251, 0xc8b7a695,
	0x0c7b6a59, 0x05140300, 0x00000716, 0x0000321d, 0x21103f2e, 0x0b7ae9d8,
	0x0f1e0d1c, 0xae9d8c7b, 0x16051403, 0x0f1e0d1c, 0x00000007, 0x00564534,
};

static const struct mtk_dip_params_demosaic mtk_dip_bq_test_demosaic = {
	.header = { .type = MTK_DIP_PARAMS_BLOCK_DEMOSAIC,
		    .size = sizeof(struct mtk_dip_params_demosaic) },
	.color_diff_gain_slope = 5,
	.color_diff_gain_offset = 22,
	.color_diff_gain_ratio = 7,
	.color_diff_kernel = 0,
	.offset_a = 73,
	.level_slope = { 10, 11, 12 },
	.level_offset = { 141, 158, 175 },
	.color_diff_slope_luma = 0,
	.color_diff_slope_chroma = 17,
	.offset_b = 226,
	.luma_weight = { 243, 260, 277, 294, 311, 328 },
	.radial_enable = 1,
	.radial_ratio2 = 10,
	.radial_gain = { 123, 140 },
	.core_th = { 157, 174 },
	.gain_a = { 31, 16, 33 },
	.gain_b = { 18, 3 },
	.noise_strength = { 20, 5, 22 },
	.crosstalk_slope = 7,
	.crosstalk_offset = 88,
	.crosstalk_ratio = 9,
	.noise_gain = 10,
	.noise_slope = 11,
	.noise_offset = { 156, 173, 190 },
	.ratio_a = 15,
	.band_gain = { 0, 17, 2 },
	.band_strength = 19,
	.band_upper = { 36, 53, 70 },
	.band_lower = { 87, 104, 121 },
	.band_slope_left = 10,
	.band_slope_right = 11,
	.clip_th = 172,
	.undershoot_th = 189,
	.overshoot_th = 206,
	.ratio_b = 31,
	.radial_ratio = 16,
	.mode_a = 1,
	.bound_upper = 18,
	.bound_lower = 35,
	.ratio_c = 4,
	.gain_c = 5,
	.clip_flag = 0,
	.bound_blend = 7,
	.threshold_a = 8,
	.threshold_b = 9,
	.gain_pos = 26,
	.gain_neg = 11,
};

static const u32 mtk_dip_bq_test_demosaic_words[] = {
	0x00007165, 0x2208da49, 0x9ebafce2, 0x03ce0915, 0x049a6f48, 0x001ee315,
	0x090ff09d, 0x4ac3d0b6, 0x9cadbeba, 0x98222fae, 0x57246835, 0x007946ba,
	0x00cebdac, 0x0000061f, 0x00002312, 0x00000004, 0x00000161, 0x0000000e,
	0x00000098, 0x0000017a,
};

static const struct mtk_dip_params_nbc2 mtk_dip_bq_test_nbc2 = {
	.header = { .type = MTK_DIP_PARAMS_BLOCK_NBC2,
		    .size = sizeof(struct mtk_dip_params_nbc2) },
	.luma_enable = 1,
	.scale_mode = 2,
	.chroma_ref_luma = 1,
	.radial_link = 0,
	.chroma_smooth_edge = 1,
	.chroma_filter = 0,
	.chroma_threshold = 11,
	.luma_scale_x = { 124, 141, 158 },
	.luma_scale_y = { 15, 0, 17, 2 },
	.luma_scale_slope = { 19, 4, 21, 6 },
	.radial_x = { 55, 72, 89 },
	.radial_y = { 42, 59, 12, 29 },
	.radial_slope = { 46, 63, 16, 33 },
	.aux_v_gain = 2,
	.aux_gain_th = 3,
	.chroma_luma_diff_th = 20,
	.chroma_mode = 1,
	.aux_th = { 54, 71, 88, 105 },
	.radial_chroma_gain = 10,
	.luma_weight = 11,
	.core_th = 28,
	.core_slope = 5,
	.diff_th = 30,
	.diff_slope = 7,
	.class_th = { 224, 49, 2 },
	.class_slope = { 3, 4, 5 },
	.class_level = { 6, 23, 8 },
	.class_offset = { 121, 10, 155 },
	.variance = 4,
	.strength = { 61, 14, 31, 48, 1 },
	.act_blend_base = 18,
	.chroma_dither = { 3, 20 },
};

static const u32 mtk_dip_bq_test_nbc2_words[] = {
	0x000f0123, 0x00000100, 0xb0000000, 0x009e8d7c, 0x0211000f, 0x06150413,
	0x00594837, 0x1d0c3b2a, 0x21103f2e, 0x01140382, 0x69584736, 0x000b000a,
	0xe071e51c, 0x17431063, 0x3d408502, 0x01301f0e, 0x009b0a79, 0x14031200,
};

static void mtk_dip_bq_test_words(struct kunit *test)
{
	struct mtk_dip_params_ee *ee;
	u32 w[MTK_DIP_YCNR_WORDS];

	KUNIT_EXPECT_EQ(test, mtk_dip_bq_check(&mtk_dip_bq_test_ee0.header), 0);
	mtk_dip_bq_ee_words(&mtk_dip_bq_test_ee0, w);
	KUNIT_EXPECT_MEMEQ(test, w, mtk_dip_bq_test_ee0_words,
			   sizeof(mtk_dip_bq_test_ee0_words));

	/* slopes limited to 127 */
	KUNIT_EXPECT_EQ(test, mtk_dip_bq_check(&mtk_dip_bq_test_ee1.header), 0);
	mtk_dip_bq_ee_words(&mtk_dip_bq_test_ee1, w);
	KUNIT_EXPECT_MEMEQ(test, w, mtk_dip_bq_test_ee1_words,
			   sizeof(mtk_dip_bq_test_ee1_words));

	KUNIT_EXPECT_EQ(test, mtk_dip_bq_check(&mtk_dip_bq_test_ycnr.header), 0);
	mtk_dip_bq_ycnr_words(&mtk_dip_bq_test_ycnr, w);
	KUNIT_EXPECT_MEMEQ(test, w, mtk_dip_bq_test_ycnr_words,
			   sizeof(mtk_dip_bq_test_ycnr_words));

	KUNIT_EXPECT_EQ(test,
			mtk_dip_bq_check(&mtk_dip_bq_test_demosaic.header), 0);
	mtk_dip_bq_demosaic_words(&mtk_dip_bq_test_demosaic, w);
	KUNIT_EXPECT_MEMEQ(test, w, mtk_dip_bq_test_demosaic_words,
			   sizeof(mtk_dip_bq_test_demosaic_words));

	/* empty first segment: steepest slope towards its step */
	ee = kunit_kmalloc(test, sizeof(*ee), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, ee);
	*ee = mtk_dip_bq_test_ee0;
	ee->gain_curve_x[0] = 0;
	KUNIT_EXPECT_EQ(test, mtk_dip_bq_check(&ee->header), 0);
	mtk_dip_bq_ee_words(ee, w);
	KUNIT_EXPECT_EQ(test, w[7] & GENMASK(7, 0), 127);
	/* 16 * (515 - 498) / 60 */
	KUNIT_EXPECT_EQ(test, w[8] & GENMASK(7, 0), 5);
	ee->gain_curve_y[0] = 0;
	mtk_dip_bq_ee_words(ee, w);
	KUNIT_EXPECT_EQ(test, w[7] & GENMASK(7, 0), 0);

	/* with the fixed bits of the driver */
	KUNIT_EXPECT_EQ(test, mtk_dip_bq_check(&mtk_dip_bq_test_nbc2.header), 0);
	mtk_dip_bq_nbc2_words(&mtk_dip_bq_test_nbc2, w);
	KUNIT_EXPECT_MEMEQ(test, w, mtk_dip_bq_test_nbc2_words,
			   sizeof(mtk_dip_bq_test_nbc2_words));
}

/* Values out of range, reserved bytes, broken relations */
/*
 * Temporal noise reduction: every field at its largest value fills exactly
 * the bits of the module words that hold parameters, plus the fixed bits of
 * the driver; radial link off sets its switch
 */
static void mtk_dip_bq_test_nr3d(struct kunit *test)
{
	static const u32 full[MTK_DIP_NR3D_WORDS] = {
		0x00000f10, 0xc0000000, 0xfffffff0, 0xfff00000, 0x3ffffc00,
		0xfffffffc, 0xfffffffc, 0xfffffffc, 0xfffffffc, 0xfffffffc,
		0xfffffffc, 0xfffffffc, 0xfffffffc, 0xfffffffc, 0xfffffffc,
		0xfffffffc, 0xfffffffc, 0xffffff00, 0xfffffffc, 0xffffff00,
		0x03ffffff, 0xfffffffc,
	};
	struct mtk_dip_params_nr3d nr = {
		.header = { .type = MTK_DIP_PARAMS_BLOCK_NR3D,
			    .size = sizeof(struct mtk_dip_params_nr3d) },
	};
	u32 w[MTK_DIP_NR3D_WORDS];
	unsigned int i;
	u8 *p = (u8 *)&nr;

	/* all u8 fields from strength_max on at their maximum: 6 bits */
	for (i = offsetof(struct mtk_dip_params_nr3d, strength_max);
	     i < offsetof(struct mtk_dip_params_nr3d, reserved); i++)
		p[i] = 0x3f;
	memset(nr.blend_ratio, 0x1f, sizeof(nr.blend_ratio));
	nr.motion_penalty_weight = 15;
	nr.block_diff_th = 15;
	nr.pattern_mode = 3;
	nr.luma_enable = 1;
	nr.chroma_enable = 1;
	nr.radial_link = 1;
	nr.motion_x = -MTK_DIP_NR3D_MOTION_MAX;
	nr.motion_y = MTK_DIP_NR3D_MOTION_MAX;
	KUNIT_ASSERT_EQ(test, mtk_dip_bq_check(&nr.header), 0);
	mtk_dip_bq_nr3d_words(&nr, w);
	for (i = 0; i < MTK_DIP_NR3D_WORDS; i++)
		KUNIT_EXPECT_EQ_MSG(test, w[i], full[i], "word %u", i);

	nr.radial_link = 0;
	mtk_dip_bq_nr3d_words(&nr, w);
	KUNIT_EXPECT_EQ(test, w[0], full[0] | BIT(20));

	/* refused: values over their field, odd motion, both parts off */
	nr.radial_link = 2;
	KUNIT_EXPECT_EQ(test, mtk_dip_bq_check(&nr.header), -EINVAL);
	nr.radial_link = 1;
	nr.strength_max = 64;
	KUNIT_EXPECT_EQ(test, mtk_dip_bq_check(&nr.header), -EINVAL);
	nr.strength_max = 63;
	nr.motion_x = 1;
	KUNIT_EXPECT_EQ(test, mtk_dip_bq_check(&nr.header), -EINVAL);
	nr.motion_x = MTK_DIP_NR3D_MOTION_MAX + 2;
	KUNIT_EXPECT_EQ(test, mtk_dip_bq_check(&nr.header), -EINVAL);
	nr.motion_x = 0;
	nr.luma_enable = 0;
	nr.chroma_enable = 0;
	KUNIT_EXPECT_EQ(test, mtk_dip_bq_check(&nr.header), -EINVAL);
	nr.chroma_enable = 1;
	nr.flags = 2;
	KUNIT_EXPECT_EQ(test, mtk_dip_bq_check(&nr.header), -EINVAL);
	nr.flags = MTK_DIP_NR3D_FL_RESTART;
	nr.reserved[1] = 1;
	KUNIT_EXPECT_EQ(test, mtk_dip_bq_check(&nr.header), -EINVAL);
	nr.reserved[1] = 0;
	KUNIT_EXPECT_EQ(test, mtk_dip_bq_check(&nr.header), 0);
}

static void mtk_dip_bq_test_check(struct kunit *test)
{
	struct mtk_dip_params_demosaic *dm;
	struct mtk_dip_params_nbc2 *n2;
	struct mtk_dip_params_ycnr *nr;
	struct mtk_dip_params_ee *ee;

	ee = kunit_kmalloc(test, sizeof(*ee), GFP_KERNEL);
	nr = kunit_kmalloc(test, sizeof(*nr), GFP_KERNEL);
	dm = kunit_kmalloc(test, sizeof(*dm), GFP_KERNEL);
	n2 = kunit_kmalloc(test, sizeof(*n2), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, ee);
	KUNIT_ASSERT_NOT_NULL(test, nr);
	KUNIT_ASSERT_NOT_NULL(test, dm);
	KUNIT_ASSERT_NOT_NULL(test, n2);

	*ee = mtk_dip_bq_test_ee0;
	ee->luma_gain[6] = 512;
	KUNIT_EXPECT_EQ(test, mtk_dip_bq_check(&ee->header), -EINVAL);
	*ee = mtk_dip_bq_test_ee0;
	ee->luma_gain[6] = 511;
	KUNIT_EXPECT_EQ(test, mtk_dip_bq_check(&ee->header), 0);
	ee->band_gain[1] = 32;
	KUNIT_EXPECT_EQ(test, mtk_dip_bq_check(&ee->header), -EINVAL);
	*ee = mtk_dip_bq_test_ee0;
	ee->reserved2[5] = 1;
	KUNIT_EXPECT_EQ(test, mtk_dip_bq_check(&ee->header), -EINVAL);
	*ee = mtk_dip_bq_test_ee0;
	ee->gain_curve_x[2] = ee->gain_curve_x[1];
	KUNIT_EXPECT_EQ(test, mtk_dip_bq_check(&ee->header), -EINVAL);
	*ee = mtk_dip_bq_test_ee0;
	ee->gain_curve_x[0] = 0;
	KUNIT_EXPECT_EQ(test, mtk_dip_bq_check(&ee->header), 0);
	ee->gain_curve_x[1] = 0;
	KUNIT_EXPECT_EQ(test, mtk_dip_bq_check(&ee->header), -EINVAL);
	*ee = mtk_dip_bq_test_ee0;
	ee->gain_curve_x[3] = 255;
	KUNIT_EXPECT_EQ(test, mtk_dip_bq_check(&ee->header), -EINVAL);
	*ee = mtk_dip_bq_test_ee0;
	ee->gain_curve_y_end = 3072;
	KUNIT_EXPECT_EQ(test, mtk_dip_bq_check(&ee->header), -EINVAL);
	ee->gain_curve_y_end = -2049;
	KUNIT_EXPECT_EQ(test, mtk_dip_bq_check(&ee->header), -EINVAL);

	*nr = mtk_dip_bq_test_ycnr;
	nr->luma_enable = 0;
	nr->chroma_enable = 0;
	KUNIT_EXPECT_EQ(test, mtk_dip_bq_check(&nr->header), -EINVAL);
	nr->chroma_enable = 1;
	KUNIT_EXPECT_EQ(test, mtk_dip_bq_check(&nr->header), 0);
	nr->luma_filter_sel[1] = 2;
	KUNIT_EXPECT_EQ(test, mtk_dip_bq_check(&nr->header), -EINVAL);
	*nr = mtk_dip_bq_test_ycnr;
	nr->luma_range_h[3][3] = 255;
	KUNIT_EXPECT_EQ(test, mtk_dip_bq_check(&nr->header), 0);
	nr->reserved[0] = 1;
	KUNIT_EXPECT_EQ(test, mtk_dip_bq_check(&nr->header), -EINVAL);

	*dm = mtk_dip_bq_test_demosaic;
	dm->gain_a[2] = 63;
	KUNIT_EXPECT_EQ(test, mtk_dip_bq_check(&dm->header), 0);
	dm->gain_a[2] = 64;
	KUNIT_EXPECT_EQ(test, mtk_dip_bq_check(&dm->header), -EINVAL);
	*dm = mtk_dip_bq_test_demosaic;
	dm->gain_a[1] = 32;
	KUNIT_EXPECT_EQ(test, mtk_dip_bq_check(&dm->header), -EINVAL);

	/* scales with other tile margins */
	*n2 = mtk_dip_bq_test_nbc2;
	n2->scale_mode = 3;
	KUNIT_EXPECT_EQ(test, mtk_dip_bq_check(&n2->header), 0);
	n2->scale_mode = 1;
	KUNIT_EXPECT_EQ(test, mtk_dip_bq_check(&n2->header), -EINVAL);
	n2->scale_mode = 0;
	KUNIT_EXPECT_EQ(test, mtk_dip_bq_check(&n2->header), -EINVAL);
	n2->scale_mode = 4;
	KUNIT_EXPECT_EQ(test, mtk_dip_bq_check(&n2->header), -EINVAL);
	*n2 = mtk_dip_bq_test_nbc2;
	n2->class_th[0] = 255;
	n2->class_th[1] = 63;
	KUNIT_EXPECT_EQ(test, mtk_dip_bq_check(&n2->header), 0);
	n2->class_th[1] = 64;
	KUNIT_EXPECT_EQ(test, mtk_dip_bq_check(&n2->header), -EINVAL);
	*n2 = mtk_dip_bq_test_nbc2;
	n2->radial_y[3] = 64;
	KUNIT_EXPECT_EQ(test, mtk_dip_bq_check(&n2->header), -EINVAL);
}

/* The blocks switch the modules on and back to the defaults */
static void mtk_dip_bq_test_state(struct kunit *test)
{
	struct v4l2_isp_params_block_header *h;
	struct v4l2_isp_params_buffer *pb;
	struct mtk_dip_param_state *p;
	struct mtk_dip_buffer *buf;
	size_t n;

	p = kunit_kzalloc(test, sizeof(*p), GFP_KERNEL);
	buf = kunit_kzalloc(test, sizeof(*buf), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, p);
	KUNIT_ASSERT_NOT_NULL(test, buf);
	pb = (void *)buf->params;
	pb->version = MTK_DIP_PARAMS_VERSION_V1;

	mtk_dip_params_neutral(p);
	KUNIT_EXPECT_FALSE(test, p->ee);
	KUNIT_EXPECT_FALSE(test, p->ycnr);
	KUNIT_EXPECT_FALSE(test, p->demosaic);
	KUNIT_EXPECT_FALSE(test, p->nbc2);

	n = 0;
	memcpy(pb->data + n, &mtk_dip_bq_test_ee0, sizeof(mtk_dip_bq_test_ee0));
	n += sizeof(mtk_dip_bq_test_ee0);
	memcpy(pb->data + n, &mtk_dip_bq_test_ycnr,
	       sizeof(mtk_dip_bq_test_ycnr));
	n += sizeof(mtk_dip_bq_test_ycnr);
	memcpy(pb->data + n, &mtk_dip_bq_test_demosaic,
	       sizeof(mtk_dip_bq_test_demosaic));
	n += sizeof(mtk_dip_bq_test_demosaic);
	memcpy(pb->data + n, &mtk_dip_bq_test_nbc2,
	       sizeof(mtk_dip_bq_test_nbc2));
	n += sizeof(mtk_dip_bq_test_nbc2);
	pb->data_size = n;
	mtk_dip_params_apply(p, buf);
	KUNIT_EXPECT_TRUE(test, p->ee);
	KUNIT_EXPECT_TRUE(test, p->ycnr);
	KUNIT_EXPECT_TRUE(test, p->demosaic);
	KUNIT_EXPECT_TRUE(test, p->nbc2);
	KUNIT_EXPECT_MEMEQ(test, p->nbc2_w, mtk_dip_bq_test_nbc2_words,
			   sizeof(p->nbc2_w));
	KUNIT_EXPECT_MEMEQ(test, p->ee_w, mtk_dip_bq_test_ee0_words,
			   sizeof(p->ee_w));
	KUNIT_EXPECT_MEMEQ(test, p->ycnr_w, mtk_dip_bq_test_ycnr_words,
			   sizeof(p->ycnr_w));
	KUNIT_EXPECT_MEMEQ(test, p->demosaic_w, mtk_dip_bq_test_demosaic_words,
			   sizeof(p->demosaic_w));

	/* a buffer without the blocks keeps them */
	pb->data_size = 0;
	mtk_dip_params_apply(p, buf);
	KUNIT_EXPECT_TRUE(test, p->ee);
	KUNIT_EXPECT_TRUE(test, p->ycnr);
	KUNIT_EXPECT_TRUE(test, p->nbc2);

	/* disabling headers: edge enhancement off, the others to the defaults */
	h = (void *)pb->data;
	h[0] = (struct v4l2_isp_params_block_header) {
		MTK_DIP_PARAMS_BLOCK_EE, MTK_DIP_PARAMS_BLOCK_FL_DISABLE,
		sizeof(*h) };
	h[1] = (struct v4l2_isp_params_block_header) {
		MTK_DIP_PARAMS_BLOCK_YCNR, MTK_DIP_PARAMS_BLOCK_FL_DISABLE,
		sizeof(*h) };
	h[2] = (struct v4l2_isp_params_block_header) {
		MTK_DIP_PARAMS_BLOCK_DEMOSAIC, MTK_DIP_PARAMS_BLOCK_FL_DISABLE,
		sizeof(*h) };
	h[3] = (struct v4l2_isp_params_block_header) {
		MTK_DIP_PARAMS_BLOCK_NBC2, MTK_DIP_PARAMS_BLOCK_FL_DISABLE,
		sizeof(*h) };
	pb->data_size = 4 * sizeof(*h);
	mtk_dip_params_apply(p, buf);
	KUNIT_EXPECT_FALSE(test, p->ee);
	KUNIT_EXPECT_FALSE(test, p->ycnr);
	KUNIT_EXPECT_FALSE(test, p->demosaic);
	KUNIT_EXPECT_FALSE(test, p->nbc2);
}

/*
 * Neutral settings in the register image: the first noise reduction stage
 * with its chroma part only and no tone map, demosaicing 0, the second
 * stage with scale 3, flat radial weights, raw noise reduction with unity
 * normalisation
 */
static void mtk_dip_bq_test_neutral_image(struct kunit *test, const u32 *img)
{
	unsigned int i;

	KUNIT_EXPECT_EQ(test, img[0x46c4 / 4], 0);
	KUNIT_EXPECT_EQ(test, img[0x3700 / 4], 0x000f0001);
	KUNIT_EXPECT_EQ(test, img[0x3704 / 4], 0);
	KUNIT_EXPECT_EQ(test, img[0x3300 / 4], 0);
	KUNIT_EXPECT_EQ(test, img[0x36fc / 4], 0);
	KUNIT_EXPECT_EQ(test, img[0x37b4 / 4], 0x9b7b5800);
	for (i = 0x1540; i <= 0x1598; i += 4)
		KUNIT_EXPECT_EQ(test, img[i / 4], 0);
	KUNIT_EXPECT_EQ(test, img[0x4800 / 4], 0x000f0031);
	KUNIT_EXPECT_EQ(test, img[0x4840 / 4], 0x00000080);
	KUNIT_EXPECT_EQ(test, img[0x4874 / 4], 0);
	KUNIT_EXPECT_EQ(test, img[0x15c0 / 4], 0);
	KUNIT_EXPECT_EQ(test, img[0x15d0 / 4], 0x08000800);
	KUNIT_EXPECT_EQ(test, img[0x52d0 / 4], 0x08000800);
	KUNIT_EXPECT_EQ(test, img[0x15dc / 4], 0);
	KUNIT_EXPECT_EQ(test, img[0x14c0 / 4], 0);
	KUNIT_EXPECT_EQ(test, img[0x14cc / 4], 0x01000000);
	KUNIT_EXPECT_EQ(test, img[0x14d8 / 4], 512);
	KUNIT_EXPECT_EQ(test, img[0x14dc / 4], 0x01188c46);
	KUNIT_EXPECT_EQ(test, img[0x14f4 / 4], 0);
}

/* Register image: own settings, and the neutral settings without them */
static void mtk_dip_bq_test_image(struct kunit *test)
{
	struct v4l2_isp_params_block_header *h;
	struct v4l2_isp_params_buffer *pb;
	struct mtk_dip_buffer *buf;
	struct mtk_dip *dip;
	size_t n = 0;
	u32 *img;

	dip = kunit_kzalloc(test, sizeof(*dip), GFP_KERNEL);
	buf = kunit_kzalloc(test, sizeof(*buf), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, dip);
	KUNIT_ASSERT_NOT_NULL(test, buf);
	dip->ctrl.cpu = kunit_kzalloc(test, MTK_DIP_CTRL_SIZE, GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, dip->ctrl.cpu);
	img = dip->ctrl.cpu + MTK_DIP_CTRL_IMAGE;
	pb = (void *)buf->params;
	pb->version = MTK_DIP_PARAMS_VERSION_V1;

	/* neutral settings, edge mode 0 */
	mtk_dip_params_neutral(&dip->par);
	mtk_dip_profile_params(dip);
	mtk_dip_bq_test_neutral_image(test, img);

	memcpy(pb->data + n, &mtk_dip_bq_test_ee0, sizeof(mtk_dip_bq_test_ee0));
	n += sizeof(mtk_dip_bq_test_ee0);
	memcpy(pb->data + n, &mtk_dip_bq_test_ycnr,
	       sizeof(mtk_dip_bq_test_ycnr));
	n += sizeof(mtk_dip_bq_test_ycnr);
	memcpy(pb->data + n, &mtk_dip_bq_test_demosaic,
	       sizeof(mtk_dip_bq_test_demosaic));
	n += sizeof(mtk_dip_bq_test_demosaic);
	memcpy(pb->data + n, &mtk_dip_bq_test_nbc2,
	       sizeof(mtk_dip_bq_test_nbc2));
	n += sizeof(mtk_dip_bq_test_nbc2);
	pb->data_size = n;
	mtk_dip_params_apply(&dip->par, buf);
	mtk_dip_profile_params(dip);

	KUNIT_EXPECT_EQ(test, img[0x46c0 / 4], 0);
	KUNIT_EXPECT_EQ(test, img[0x46c4 / 4], 1);
	KUNIT_EXPECT_EQ(test, img[0x46c8 / 4], mtk_dip_bq_test_ee0_words[0]);
	KUNIT_EXPECT_EQ(test, img[0x473c / 4], mtk_dip_bq_test_ee0_words[29]);
	KUNIT_EXPECT_EQ(test, img[0x4740 / 4], 0);
	KUNIT_EXPECT_EQ(test, img[0x4750 / 4], 0);

	KUNIT_EXPECT_EQ(test, img[0x3700 / 4],
			mtk_dip_bq_test_ycnr_words[0] | 0x000f0000);
	KUNIT_EXPECT_EQ(test, img[0x37a8 / 4], mtk_dip_bq_test_ycnr_words[42]);
	KUNIT_EXPECT_EQ(test, img[0x37d4 / 4], mtk_dip_bq_test_ycnr_words[43]);
	KUNIT_EXPECT_EQ(test, img[0x37e4 / 4], mtk_dip_bq_test_ycnr_words[47]);
	KUNIT_EXPECT_EQ(test, img[0x37ac / 4], 0);
	KUNIT_EXPECT_EQ(test, img[0x37b4 / 4], 0x9b7b5800);
	KUNIT_EXPECT_EQ(test, img[0x37d0 / 4], 0x01000000);
	KUNIT_EXPECT_EQ(test, img[0x3300 / 4], 0);
	KUNIT_EXPECT_EQ(test, img[0x36fc / 4], 0);

	KUNIT_EXPECT_EQ(test, img[0x1540 / 4],
			mtk_dip_bq_test_demosaic_words[0]);
	KUNIT_EXPECT_EQ(test, img[0x1574 / 4],
			mtk_dip_bq_test_demosaic_words[13]);
	KUNIT_EXPECT_EQ(test, img[0x1578 / 4], 0);
	KUNIT_EXPECT_EQ(test, img[0x157c / 4],
			mtk_dip_bq_test_demosaic_words[14]);
	KUNIT_EXPECT_EQ(test, img[0x1594 / 4],
			mtk_dip_bq_test_demosaic_words[18]);
	KUNIT_EXPECT_EQ(test, img[0x1598 / 4],
			mtk_dip_bq_test_demosaic_words[19]);

	KUNIT_EXPECT_EQ(test, img[0x4800 / 4], mtk_dip_bq_test_nbc2_words[0]);
	KUNIT_EXPECT_EQ(test, img[0x4808 / 4], mtk_dip_bq_test_nbc2_words[2]);
	KUNIT_EXPECT_EQ(test, img[0x480c / 4], 0);
	KUNIT_EXPECT_EQ(test, img[0x4810 / 4], mtk_dip_bq_test_nbc2_words[3]);
	KUNIT_EXPECT_EQ(test, img[0x485c / 4], mtk_dip_bq_test_nbc2_words[16]);
	KUNIT_EXPECT_EQ(test, img[0x4874 / 4], mtk_dip_bq_test_nbc2_words[17]);

	/* disabled: back to the defaults */
	h = (void *)pb->data;
	h[0] = (struct v4l2_isp_params_block_header) {
		MTK_DIP_PARAMS_BLOCK_EE, MTK_DIP_PARAMS_BLOCK_FL_DISABLE,
		sizeof(*h) };
	h[1] = (struct v4l2_isp_params_block_header) {
		MTK_DIP_PARAMS_BLOCK_YCNR, MTK_DIP_PARAMS_BLOCK_FL_DISABLE,
		sizeof(*h) };
	h[2] = (struct v4l2_isp_params_block_header) {
		MTK_DIP_PARAMS_BLOCK_DEMOSAIC, MTK_DIP_PARAMS_BLOCK_FL_DISABLE,
		sizeof(*h) };
	h[3] = (struct v4l2_isp_params_block_header) {
		MTK_DIP_PARAMS_BLOCK_NBC2, MTK_DIP_PARAMS_BLOCK_FL_DISABLE,
		sizeof(*h) };
	pb->data_size = 4 * sizeof(*h);
	mtk_dip_params_apply(&dip->par, buf);
	mtk_dip_profile_params(dip);
	KUNIT_EXPECT_EQ(test, img[0x46c8 / 4], 0);
	mtk_dip_bq_test_neutral_image(test, img);
}

/* Radial weights and raw noise reduction: field positions and limits */
static const struct mtk_dip_params_radial mtk_dip_bq_test_radial = {
	.header = { MTK_DIP_PARAMS_BLOCK_RADIAL, MTK_DIP_PARAMS_BLOCK_FL_ENABLE,
		    sizeof(struct mtk_dip_params_radial) },
	.center_x = 1001, .center_y = 702,
	.radius = { 300, 600, 16383 },
	.weight = { 1, 2, 3, 4, 255 },
	.slope = { 0x000123, 0x004567, 0x089abc, 0xffffff },
};

static const u32 mtk_dip_bq_test_radial_words[MTK_DIP_RADIAL_WORDS] = {
	0x02be03e9, 0x0258012c, 0x02013fff, 0x00ff0403, 0x08000800, 0, 0,
	0x00000123, 0x00004567, 0x00089abc, 0x00ffffff, 0,
};

static const struct mtk_dip_params_rnr mtk_dip_bq_test_rnr = {
	.header = { MTK_DIP_PARAMS_BLOCK_RNR, MTK_DIP_PARAMS_BLOCK_FL_ENABLE,
		    sizeof(struct mtk_dip_params_rnr) },
	.ratio = 300, .eps = 7,
	.noise_offset = { 1, 2, 511 }, .noise_slope = { 3, 4, 5 },
	.radial_gain = { 1023, 2 },
	.norm_gain = { 1024, 512, 8191 }, .norm_offset = { -1, 2047, -2048 },
	.black_gain = 600,
	.range = { 1, 2, 3, 63 }, .median_ratio = 127, .profile_ratio = 1,
	.radius = 5, .th_right = 6, .th_left = 63,
	.flat_mode = 1, .gradient_ratio = 63, .lowpass_ratio = 255,
	.intensity_offset = 9, .radial_enable = 1,
	.level_th = { 10, 11 }, .level_slope = { 12, 13 },
};

static const u32 mtk_dip_bq_test_rnr_words[MTK_DIP_RNR_WORDS] = {
	0x00000f2c, 0x07fc0401, 0x00140803, 0x02007fff, 0x010007ff,
	0x0ffff800, 0x00000258, 0x01188c46, 0x004c2613, 0x02241489,
	0x00fc3081, 0x1f8d40ff, 0x0004ffff, 0x001ff805, 0x0a0c0b0d, 0,
};

static void mtk_dip_bq_test_radial_rnr(struct kunit *test)
{
	struct mtk_dip_params_radial rw = mtk_dip_bq_test_radial;
	struct mtk_dip_params_rnr nr = mtk_dip_bq_test_rnr;
	u32 w[MTK_DIP_RNR_WORDS];

	KUNIT_EXPECT_EQ(test, mtk_dip_bq_check(&rw.header), 0);
	mtk_dip_bq_radial_words(&rw, w);
	KUNIT_EXPECT_MEMEQ(test, w, mtk_dip_bq_test_radial_words,
			   sizeof(mtk_dip_bq_test_radial_words));
	rw.radius[2] = 16384;
	KUNIT_EXPECT_EQ(test, mtk_dip_bq_check(&rw.header), -EINVAL);
	rw = mtk_dip_bq_test_radial;
	rw.slope[3] = 0x1000000;
	KUNIT_EXPECT_EQ(test, mtk_dip_bq_check(&rw.header), -EINVAL);
	rw = mtk_dip_bq_test_radial;
	rw.reserved = 1;
	KUNIT_EXPECT_EQ(test, mtk_dip_bq_check(&rw.header), -EINVAL);

	KUNIT_EXPECT_EQ(test, mtk_dip_bq_check(&nr.header), 0);
	mtk_dip_bq_rnr_words(&nr, w);
	KUNIT_EXPECT_MEMEQ(test, w, mtk_dip_bq_test_rnr_words,
			   sizeof(mtk_dip_bq_test_rnr_words));
	nr.norm_offset[1] = 2048;
	KUNIT_EXPECT_EQ(test, mtk_dip_bq_check(&nr.header), -EINVAL);
	nr = mtk_dip_bq_test_rnr;
	nr.norm_gain[0] = 8192;
	KUNIT_EXPECT_EQ(test, mtk_dip_bq_check(&nr.header), -EINVAL);
	nr = mtk_dip_bq_test_rnr;
	nr.radius = 8;
	KUNIT_EXPECT_EQ(test, mtk_dip_bq_check(&nr.header), -EINVAL);
	nr = mtk_dip_bq_test_rnr;
	nr.reserved[4] = 1;
	KUNIT_EXPECT_EQ(test, mtk_dip_bq_check(&nr.header), -EINVAL);
}

static struct kunit_case mtk_dip_bq_test_cases[] = {
	KUNIT_CASE(mtk_dip_bq_test_words),
	KUNIT_CASE(mtk_dip_bq_test_nr3d),
	KUNIT_CASE(mtk_dip_bq_test_check),
	KUNIT_CASE(mtk_dip_bq_test_state),
	KUNIT_CASE(mtk_dip_bq_test_image),
	KUNIT_CASE(mtk_dip_bq_test_radial_rnr),
	{}
};

static struct kunit_suite mtk_dip_bq_test_suite = {
	.name = "mtk-dip-mt6785-bq",
	.test_cases = mtk_dip_bq_test_cases,
};

kunit_test_suite(mtk_dip_bq_test_suite);
