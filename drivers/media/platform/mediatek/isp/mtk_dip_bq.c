// SPDX-License-Identifier: GPL-2.0-only
/*
 * MediaTek MT6785 ISP pass 2 (DIP): the image quality blocks (edge
 * enhancement, both luma/chroma noise reduction stages, demosaicing,
 * temporal noise reduction, radial weights, raw noise reduction).
 *
 * Each parameter of these blocks is one field of a register word of its
 * module. The tables below give, per struct member (or array element), the
 * word, the first bit and the width of its field; the checks and the
 * translation into register words are generic.
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/stddef.h>
#include <linux/string.h>
#include <media/v4l2-isp.h>

#include "mtk_dip.h"

/**
 * struct mtk_dip_bq_field - a parameter of an image quality block
 * @off: offset of the member in the block
 * @size: size of the member: 1 (u8), 2 (u16) or 4 (u32)
 * @word: register word of the module
 * @lsb: first bit of the field in the word
 * @width: width of the field in bits
 */
struct mtk_dip_bq_field {
	u16 off;
	u8 size;
	u8 word;
	u8 lsb;
	u8 width;
};

/**
 * struct mtk_dip_bq_reserved - reserved bytes of a block, must be 0
 * @off: offset in the block
 * @len: number of bytes
 */
struct mtk_dip_bq_reserved {
	u16 off;
	u16 len;
};

#define BQ(s, m, w, l, n) \
	{ offsetof(struct s, m), sizeof_field(struct s, m), w, l, n }
#define BQR(s, m) { offsetof(struct s, m), sizeof_field(struct s, m) }

static const struct mtk_dip_bq_field mtk_dip_ee_fields[] = {
	BQ(mtk_dip_params_ee, band_offset[0], 0, 0, 8),
	BQ(mtk_dip_params_ee, band_offset[1], 0, 8, 8),
	BQ(mtk_dip_params_ee, band_offset[2], 0, 16, 8),
	BQ(mtk_dip_params_ee, band_slope[0], 1, 0, 8),
	BQ(mtk_dip_params_ee, band_slope[1], 1, 8, 8),
	BQ(mtk_dip_params_ee, band_slope[2], 1, 16, 8),
	BQ(mtk_dip_params_ee, band_ratio, 1, 24, 5),
	BQ(mtk_dip_params_ee, band_core_th[0], 2, 0, 8),
	BQ(mtk_dip_params_ee, band_core_th[1], 2, 8, 8),
	BQ(mtk_dip_params_ee, band_core_th[2], 2, 16, 8),
	BQ(mtk_dip_params_ee, core_th, 2, 24, 8),
	BQ(mtk_dip_params_ee, band_gain[0], 3, 0, 5),
	BQ(mtk_dip_params_ee, band_gain[1], 3, 8, 5),
	BQ(mtk_dip_params_ee, band_gain[2], 3, 16, 5),
	BQ(mtk_dip_params_ee, luma_gain[0], 4, 0, 9),
	BQ(mtk_dip_params_ee, luma_gain[1], 4, 10, 9),
	BQ(mtk_dip_params_ee, luma_gain[2], 4, 20, 9),
	BQ(mtk_dip_params_ee, luma_gain[3], 5, 0, 9),
	BQ(mtk_dip_params_ee, luma_gain[4], 5, 10, 9),
	BQ(mtk_dip_params_ee, luma_gain[5], 5, 20, 9),
	BQ(mtk_dip_params_ee, luma_gain[6], 6, 22, 9),
	BQ(mtk_dip_params_ee, radial_gain[0], 6, 0, 8),
	BQ(mtk_dip_params_ee, radial_gain[1], 6, 8, 8),
	BQ(mtk_dip_params_ee, radial_response_ratio, 6, 16, 5),
	BQ(mtk_dip_params_ee, gain_curve_radial, 6, 21, 1),
	BQ(mtk_dip_params_ee, gain_curve_x[0], 7, 8, 8),
	BQ(mtk_dip_params_ee, gain_curve_x[1], 8, 8, 8),
	BQ(mtk_dip_params_ee, gain_curve_x[2], 9, 8, 8),
	BQ(mtk_dip_params_ee, gain_curve_x[3], 10, 8, 8),
	BQ(mtk_dip_params_ee, gain_curve_y[0], 7, 16, 10),
	BQ(mtk_dip_params_ee, gain_curve_y[1], 8, 16, 10),
	BQ(mtk_dip_params_ee, gain_curve_y[2], 9, 16, 10),
	BQ(mtk_dip_params_ee, gain_curve_y[3], 10, 16, 10),
	BQ(mtk_dip_params_ee, gain_curve_aux, 11, 16, 10),
	BQ(mtk_dip_params_ee, gain_curve_th_over, 12, 0, 8),
	BQ(mtk_dip_params_ee, gain_curve_th_under, 12, 8, 8),
	BQ(mtk_dip_params_ee, gain_curve_th_min, 12, 16, 8),
	BQ(mtk_dip_params_ee, response_smooth, 13, 0, 3),
	BQ(mtk_dip_params_ee, overshoot_clip, 13, 4, 3),
	BQ(mtk_dip_params_ee, dot_reduction, 13, 8, 8),
	BQ(mtk_dip_params_ee, dot_th, 13, 16, 8),
	BQ(mtk_dip_params_ee, response_clip, 14, 0, 8),
	BQ(mtk_dip_params_ee, response_clip_luma_th, 14, 8, 8),
	BQ(mtk_dip_params_ee, response_clip_luma_lower, 14, 16, 8),
	BQ(mtk_dip_params_ee, response_clip_luma_upper, 14, 24, 8),
	BQ(mtk_dip_params_ee, master_gain_neg, 15, 0, 8),
	BQ(mtk_dip_params_ee, master_gain_pos, 15, 8, 8),
	BQ(mtk_dip_params_ee, strength_upper, 16, 0, 8),
	BQ(mtk_dip_params_ee, strength_lower, 16, 8, 8),
	BQ(mtk_dip_params_ee, strength_edge_slope, 17, 0, 8),
	BQ(mtk_dip_params_ee, strength_edge_offset, 17, 8, 8),
	BQ(mtk_dip_params_ee, strength_response_slope, 17, 16, 8),
	BQ(mtk_dip_params_ee, strength_response_offset, 17, 24, 8),
	BQ(mtk_dip_params_ee, luma_limit_diff, 18, 0, 8),
	BQ(mtk_dip_params_ee, luma_contrast_level, 18, 8, 3),
	BQ(mtk_dip_params_ee, luma_min_sel, 18, 12, 3),
	BQ(mtk_dip_params_ee, luma_max_sel, 18, 16, 3),
	BQ(mtk_dip_params_ee, chroma_contrast_level, 18, 20, 3),
	BQ(mtk_dip_params_ee, chroma_min_sel, 18, 24, 2),
	BQ(mtk_dip_params_ee, chroma_max_sel, 18, 28, 2),
	BQ(mtk_dip_params_ee, chroma_boost_limit_upper, 19, 0, 8),
	BQ(mtk_dip_params_ee, chroma_boost_limit_lower, 19, 8, 8),
	BQ(mtk_dip_params_ee, chroma_boost_gain, 19, 16, 8),
	BQ(mtk_dip_params_ee, chroma_boost_enable, 19, 24, 1),
	BQ(mtk_dip_params_ee, chroma_boost_y_const, 20, 0, 8),
	BQ(mtk_dip_params_ee, chroma_boost_y_offset, 20, 8, 8),
	BQ(mtk_dip_params_ee, chroma_boost_y_offset_sel, 20, 16, 2),
	BQ(mtk_dip_params_ee, regions_enable, 21, 31, 1),
	BQ(mtk_dip_params_ee, region_enable[0], 21, 30, 1),
	BQ(mtk_dip_params_ee, region_enable[1], 24, 30, 1),
	BQ(mtk_dip_params_ee, region_enable[2], 27, 30, 1),
	BQ(mtk_dip_params_ee, region_radius_slope2[0], 21, 20, 10),
	BQ(mtk_dip_params_ee, region_radius_slope2[1], 24, 20, 10),
	BQ(mtk_dip_params_ee, region_radius_slope2[2], 27, 20, 10),
	BQ(mtk_dip_params_ee, region_radius_slope[0], 21, 8, 10),
	BQ(mtk_dip_params_ee, region_radius_slope[1], 24, 8, 10),
	BQ(mtk_dip_params_ee, region_radius_slope[2], 27, 8, 10),
	BQ(mtk_dip_params_ee, region_radius_range[0], 21, 0, 6),
	BQ(mtk_dip_params_ee, region_radius_range[1], 24, 0, 6),
	BQ(mtk_dip_params_ee, region_radius_range[2], 27, 0, 6),
	BQ(mtk_dip_params_ee, region_angle_slope[0], 22, 0, 10),
	BQ(mtk_dip_params_ee, region_angle_slope[1], 25, 0, 10),
	BQ(mtk_dip_params_ee, region_angle_slope[2], 28, 0, 10),
	BQ(mtk_dip_params_ee, region_angle_range[0], 22, 10, 6),
	BQ(mtk_dip_params_ee, region_angle_range[1], 25, 10, 6),
	BQ(mtk_dip_params_ee, region_angle_range[2], 28, 10, 6),
	BQ(mtk_dip_params_ee, region_angle_center[0], 22, 16, 8),
	BQ(mtk_dip_params_ee, region_angle_center[1], 25, 16, 8),
	BQ(mtk_dip_params_ee, region_angle_center[2], 28, 16, 8),
	BQ(mtk_dip_params_ee, region_radius_center[0], 22, 24, 8),
	BQ(mtk_dip_params_ee, region_radius_center[1], 25, 24, 8),
	BQ(mtk_dip_params_ee, region_radius_center[2], 28, 24, 8),
	BQ(mtk_dip_params_ee, region_lowpass_gain[0], 23, 0, 6),
	BQ(mtk_dip_params_ee, region_lowpass_gain[1], 26, 0, 6),
	BQ(mtk_dip_params_ee, region_lowpass_gain[2], 29, 0, 6),
	BQ(mtk_dip_params_ee, region_lowpass_enable[0], 23, 6, 1),
	BQ(mtk_dip_params_ee, region_lowpass_enable[1], 26, 6, 1),
	BQ(mtk_dip_params_ee, region_lowpass_enable[2], 29, 6, 1),
	BQ(mtk_dip_params_ee, region_edge_enable[0], 23, 7, 1),
	BQ(mtk_dip_params_ee, region_edge_enable[1], 26, 7, 1),
	BQ(mtk_dip_params_ee, region_edge_enable[2], 29, 7, 1),
	BQ(mtk_dip_params_ee, region_edge_slope[0], 23, 8, 6),
	BQ(mtk_dip_params_ee, region_edge_slope[1], 26, 8, 6),
	BQ(mtk_dip_params_ee, region_edge_slope[2], 29, 8, 6),
	BQ(mtk_dip_params_ee, region_edge_th[0], 23, 14, 6),
	BQ(mtk_dip_params_ee, region_edge_th[1], 26, 14, 6),
	BQ(mtk_dip_params_ee, region_edge_th[2], 29, 14, 6),
	BQ(mtk_dip_params_ee, region_confidence_gain[0], 23, 20, 4),
	BQ(mtk_dip_params_ee, region_confidence_gain[1], 26, 20, 4),
	BQ(mtk_dip_params_ee, region_confidence_gain[2], 29, 20, 4),
	BQ(mtk_dip_params_ee, region_gain[0], 23, 24, 8),
	BQ(mtk_dip_params_ee, region_gain[1], 26, 24, 8),
	BQ(mtk_dip_params_ee, region_gain[2], 29, 24, 8),
};

static const struct mtk_dip_bq_reserved mtk_dip_ee_reserved[] = {
	BQR(mtk_dip_params_ee, reserved0),
	BQR(mtk_dip_params_ee, reserved1),
	BQR(mtk_dip_params_ee, reserved2),
};

static const struct mtk_dip_bq_field mtk_dip_ycnr_fields[] = {
	BQ(mtk_dip_params_ycnr, chroma_enable, 0, 0, 1),
	BQ(mtk_dip_params_ycnr, luma_enable, 0, 1, 1),
	BQ(mtk_dip_params_ycnr, luma_filter_sel[0], 0, 4, 1),
	BQ(mtk_dip_params_ycnr, luma_filter_sel[1], 0, 6, 1),
	BQ(mtk_dip_params_ycnr, luma_filter_sel[2], 0, 8, 2),
	BQ(mtk_dip_params_ycnr, luma_filter_sel[3], 0, 10, 2),
	BQ(mtk_dip_params_ycnr, radial_link, 0, 24, 1),
	BQ(mtk_dip_params_ycnr, filter_mode, 1, 0, 2),
	BQ(mtk_dip_params_ycnr, chroma_smooth_edge, 1, 8, 2),
	BQ(mtk_dip_params_ycnr, chroma_filter, 1, 12, 1),
	BQ(mtk_dip_params_ycnr, chroma_smooth_edge_th, 1, 24, 3),
	BQ(mtk_dip_params_ycnr, center_gain_lo_th, 2, 0, 5),
	BQ(mtk_dip_params_ycnr, center_gain_hi_th, 2, 8, 5),
	BQ(mtk_dip_params_ycnr, luma_slope_v_th, 2, 16, 8),
	BQ(mtk_dip_params_ycnr, luma_slope_h_th, 2, 24, 8),
	BQ(mtk_dip_params_ycnr, luma_vert_act_th, 3, 0, 5),
	BQ(mtk_dip_params_ycnr, luma_aux_gain_th, 3, 8, 5),
	BQ(mtk_dip_params_ycnr, luma_vert_sigma, 3, 16, 8),
	BQ(mtk_dip_params_ycnr, luma_scale_x[0], 4, 0, 8),
	BQ(mtk_dip_params_ycnr, luma_scale_x[1], 4, 8, 8),
	BQ(mtk_dip_params_ycnr, luma_scale_x[2], 4, 16, 8),
	BQ(mtk_dip_params_ycnr, luma_scale_x[3], 4, 24, 8),
	BQ(mtk_dip_params_ycnr, luma_scale_x[4], 43, 0, 8),
	BQ(mtk_dip_params_ycnr, luma_scale_x[5], 43, 8, 8),
	BQ(mtk_dip_params_ycnr, luma_scale_x[6], 43, 16, 8),
	BQ(mtk_dip_params_ycnr, luma_scale_x[7], 43, 24, 8),
	BQ(mtk_dip_params_ycnr, luma_scale_y[0], 5, 0, 5),
	BQ(mtk_dip_params_ycnr, luma_scale_y[1], 5, 8, 5),
	BQ(mtk_dip_params_ycnr, luma_scale_y[2], 5, 16, 5),
	BQ(mtk_dip_params_ycnr, luma_scale_y[3], 5, 24, 5),
	BQ(mtk_dip_params_ycnr, luma_scale_y[4], 44, 0, 5),
	BQ(mtk_dip_params_ycnr, luma_scale_y[5], 44, 8, 5),
	BQ(mtk_dip_params_ycnr, luma_scale_y[6], 44, 16, 5),
	BQ(mtk_dip_params_ycnr, luma_scale_y[7], 44, 24, 5),
	BQ(mtk_dip_params_ycnr, luma_scale_y[8], 46, 0, 5),
	BQ(mtk_dip_params_ycnr, luma_scale_slope[0], 6, 0, 5),
	BQ(mtk_dip_params_ycnr, luma_scale_slope[1], 6, 8, 5),
	BQ(mtk_dip_params_ycnr, luma_scale_slope[2], 6, 16, 5),
	BQ(mtk_dip_params_ycnr, luma_scale_slope[3], 6, 24, 5),
	BQ(mtk_dip_params_ycnr, luma_scale_slope[4], 45, 0, 5),
	BQ(mtk_dip_params_ycnr, luma_scale_slope[5], 45, 8, 5),
	BQ(mtk_dip_params_ycnr, luma_scale_slope[6], 45, 16, 5),
	BQ(mtk_dip_params_ycnr, luma_scale_slope[7], 45, 24, 5),
	BQ(mtk_dip_params_ycnr, luma_scale_slope[8], 46, 8, 5),
	BQ(mtk_dip_params_ycnr, chroma_scale_x[0], 7, 0, 8),
	BQ(mtk_dip_params_ycnr, chroma_scale_x[1], 7, 8, 8),
	BQ(mtk_dip_params_ycnr, chroma_scale_x[2], 7, 16, 8),
	BQ(mtk_dip_params_ycnr, chroma_scale_y[0], 8, 0, 5),
	BQ(mtk_dip_params_ycnr, chroma_scale_y[1], 8, 8, 5),
	BQ(mtk_dip_params_ycnr, chroma_scale_y[2], 8, 16, 5),
	BQ(mtk_dip_params_ycnr, chroma_scale_y[3], 8, 24, 5),
	BQ(mtk_dip_params_ycnr, chroma_scale_slope[0], 9, 0, 5),
	BQ(mtk_dip_params_ycnr, chroma_scale_slope[1], 9, 8, 5),
	BQ(mtk_dip_params_ycnr, chroma_scale_slope[2], 9, 16, 5),
	BQ(mtk_dip_params_ycnr, chroma_scale_slope[3], 9, 24, 5),
	BQ(mtk_dip_params_ycnr, luma_act_y[0], 10, 0, 8),
	BQ(mtk_dip_params_ycnr, luma_act_y[1], 10, 8, 8),
	BQ(mtk_dip_params_ycnr, luma_act_y[2], 10, 16, 8),
	BQ(mtk_dip_params_ycnr, luma_act_y[3], 10, 24, 8),
	BQ(mtk_dip_params_ycnr, luma_act_slope[0], 11, 0, 6),
	BQ(mtk_dip_params_ycnr, luma_act_slope[1], 11, 8, 6),
	BQ(mtk_dip_params_ycnr, luma_act_slope[2], 11, 16, 6),
	BQ(mtk_dip_params_ycnr, luma_act_slope[3], 11, 24, 6),
	BQ(mtk_dip_params_ycnr, luma_act_x[0], 47, 0, 8),
	BQ(mtk_dip_params_ycnr, luma_act_x[1], 47, 8, 8),
	BQ(mtk_dip_params_ycnr, luma_act_x[2], 47, 16, 8),
	BQ(mtk_dip_params_ycnr, radial_x[0], 12, 0, 8),
	BQ(mtk_dip_params_ycnr, radial_x[1], 12, 8, 8),
	BQ(mtk_dip_params_ycnr, radial_x[2], 12, 16, 8),
	BQ(mtk_dip_params_ycnr, radial_y[0], 13, 0, 6),
	BQ(mtk_dip_params_ycnr, radial_y[1], 13, 8, 6),
	BQ(mtk_dip_params_ycnr, radial_y[2], 13, 16, 6),
	BQ(mtk_dip_params_ycnr, radial_y[3], 13, 24, 6),
	BQ(mtk_dip_params_ycnr, radial_slope[0], 14, 0, 6),
	BQ(mtk_dip_params_ycnr, radial_slope[1], 14, 8, 6),
	BQ(mtk_dip_params_ycnr, radial_slope[2], 14, 16, 6),
	BQ(mtk_dip_params_ycnr, radial_slope[3], 14, 24, 6),
	BQ(mtk_dip_params_ycnr, radial_chroma_gain, 18, 0, 4),
	BQ(mtk_dip_params_ycnr, radial_scale_gain, 18, 4, 3),
	BQ(mtk_dip_params_ycnr, luma_range_v[0][0], 15, 0, 8),
	BQ(mtk_dip_params_ycnr, luma_range_v[0][1], 15, 8, 8),
	BQ(mtk_dip_params_ycnr, luma_range_v[0][2], 15, 16, 8),
	BQ(mtk_dip_params_ycnr, luma_range_v[0][3], 15, 24, 8),
	BQ(mtk_dip_params_ycnr, luma_range_v[1][0], 17, 0, 8),
	BQ(mtk_dip_params_ycnr, luma_range_v[1][1], 17, 8, 8),
	BQ(mtk_dip_params_ycnr, luma_range_v[1][2], 17, 16, 8),
	BQ(mtk_dip_params_ycnr, luma_range_v[1][3], 17, 24, 8),
	BQ(mtk_dip_params_ycnr, luma_range_v[2][0], 19, 0, 8),
	BQ(mtk_dip_params_ycnr, luma_range_v[2][1], 19, 8, 8),
	BQ(mtk_dip_params_ycnr, luma_range_v[2][2], 19, 16, 8),
	BQ(mtk_dip_params_ycnr, luma_range_v[2][3], 19, 24, 8),
	BQ(mtk_dip_params_ycnr, luma_range_v[3][0], 20, 0, 8),
	BQ(mtk_dip_params_ycnr, luma_range_v[3][1], 20, 8, 8),
	BQ(mtk_dip_params_ycnr, luma_range_v[3][2], 20, 16, 8),
	BQ(mtk_dip_params_ycnr, luma_range_v[3][3], 20, 24, 8),
	BQ(mtk_dip_params_ycnr, chroma_aux_gain_th, 16, 8, 5),
	BQ(mtk_dip_params_ycnr, chroma_luma_diff_th, 16, 16, 8),
	BQ(mtk_dip_params_ycnr, luma_range_h[0][0], 21, 0, 8),
	BQ(mtk_dip_params_ycnr, luma_range_h[0][1], 21, 8, 8),
	BQ(mtk_dip_params_ycnr, luma_range_h[0][2], 21, 16, 8),
	BQ(mtk_dip_params_ycnr, luma_range_h[0][3], 21, 24, 8),
	BQ(mtk_dip_params_ycnr, luma_range_h[1][0], 22, 0, 8),
	BQ(mtk_dip_params_ycnr, luma_range_h[1][1], 22, 8, 8),
	BQ(mtk_dip_params_ycnr, luma_range_h[1][2], 22, 16, 8),
	BQ(mtk_dip_params_ycnr, luma_range_h[1][3], 22, 24, 8),
	BQ(mtk_dip_params_ycnr, luma_range_h[2][0], 23, 0, 8),
	BQ(mtk_dip_params_ycnr, luma_range_h[2][1], 23, 8, 8),
	BQ(mtk_dip_params_ycnr, luma_range_h[2][2], 23, 16, 8),
	BQ(mtk_dip_params_ycnr, luma_range_h[2][3], 23, 24, 8),
	BQ(mtk_dip_params_ycnr, luma_range_h[3][0], 28, 0, 8),
	BQ(mtk_dip_params_ycnr, luma_range_h[3][1], 28, 8, 8),
	BQ(mtk_dip_params_ycnr, luma_range_h[3][2], 28, 16, 8),
	BQ(mtk_dip_params_ycnr, luma_range_h[3][3], 28, 24, 8),
	BQ(mtk_dip_params_ycnr, act_radial_gain, 27, 0, 4),
	BQ(mtk_dip_params_ycnr, act_diff_hi_th, 27, 8, 5),
	BQ(mtk_dip_params_ycnr, act_diff_gain, 27, 20, 4),
	BQ(mtk_dip_params_ycnr, act_diff_lo_th, 27, 24, 5),
	BQ(mtk_dip_params_ycnr, chroma_range_v[0], 29, 0, 8),
	BQ(mtk_dip_params_ycnr, chroma_range_v[1], 29, 8, 8),
	BQ(mtk_dip_params_ycnr, chroma_range_v[2], 29, 16, 8),
	BQ(mtk_dip_params_ycnr, chroma_range_v[3], 29, 24, 8),
	BQ(mtk_dip_params_ycnr, luma_act_center_offset, 30, 0, 6),
	BQ(mtk_dip_params_ycnr, luma_act_center_gain, 30, 8, 6),
	BQ(mtk_dip_params_ycnr, luma_act_center_th, 30, 16, 6),
	BQ(mtk_dip_params_ycnr, chroma_range_h[0], 31, 0, 8),
	BQ(mtk_dip_params_ycnr, chroma_range_h[1], 31, 8, 8),
	BQ(mtk_dip_params_ycnr, chroma_range_h[2], 31, 16, 8),
	BQ(mtk_dip_params_ycnr, chroma_range_h[3], 31, 24, 8),
	BQ(mtk_dip_params_ycnr, luma_range_ratio_th[0], 32, 0, 4),
	BQ(mtk_dip_params_ycnr, luma_range_ratio_th[1], 32, 4, 4),
	BQ(mtk_dip_params_ycnr, luma_range_ratio_th[2], 32, 8, 4),
	BQ(mtk_dip_params_ycnr, luma_range_ratio_th[3], 32, 12, 4),
	BQ(mtk_dip_params_ycnr, luma_range_ratio_slope[0], 32, 16, 2),
	BQ(mtk_dip_params_ycnr, luma_range_ratio_slope[1], 32, 20, 2),
	BQ(mtk_dip_params_ycnr, luma_range_ratio_slope[2], 32, 24, 2),
	BQ(mtk_dip_params_ycnr, luma_range_ratio_slope[3], 32, 28, 2),
	BQ(mtk_dip_params_ycnr, luma_hf_weight[0], 33, 0, 5),
	BQ(mtk_dip_params_ycnr, luma_hf_weight[1], 33, 8, 5),
	BQ(mtk_dip_params_ycnr, luma_hf_weight[2], 33, 16, 5),
	BQ(mtk_dip_params_ycnr, luma_hf_weight[3], 33, 24, 5),
	BQ(mtk_dip_params_ycnr, luma_hf_core_th, 34, 0, 8),
	BQ(mtk_dip_params_ycnr, luma_hf_core_slope, 34, 8, 2),
	BQ(mtk_dip_params_ycnr, luma_hf_clip, 34, 16, 8),
	BQ(mtk_dip_params_ycnr, hf_balance_mode, 34, 24, 2),
	BQ(mtk_dip_params_ycnr, luma_hf_act_x[0], 35, 0, 8),
	BQ(mtk_dip_params_ycnr, luma_hf_act_x[1], 35, 8, 8),
	BQ(mtk_dip_params_ycnr, luma_hf_act_x[2], 35, 16, 8),
	BQ(mtk_dip_params_ycnr, luma_hf_act_x[3], 35, 24, 8),
	BQ(mtk_dip_params_ycnr, luma_hf_act_y[0], 36, 0, 7),
	BQ(mtk_dip_params_ycnr, luma_hf_act_y[1], 36, 8, 7),
	BQ(mtk_dip_params_ycnr, luma_hf_act_y[2], 36, 16, 7),
	BQ(mtk_dip_params_ycnr, luma_hf_act_y[3], 36, 24, 7),
	BQ(mtk_dip_params_ycnr, luma_hf_act_y[4], 39, 0, 7),
	BQ(mtk_dip_params_ycnr, luma_hf_act_slope[0], 40, 0, 6),
	BQ(mtk_dip_params_ycnr, luma_hf_act_slope[1], 40, 8, 6),
	BQ(mtk_dip_params_ycnr, luma_hf_act_slope[2], 40, 16, 6),
	BQ(mtk_dip_params_ycnr, luma_hf_act_slope[3], 40, 24, 6),
	BQ(mtk_dip_params_ycnr, luma_hf_act_slope[4], 39, 8, 6),
	BQ(mtk_dip_params_ycnr, act_blend_base, 37, 8, 7),
	BQ(mtk_dip_params_ycnr, chroma_dither[0], 37, 16, 5),
	BQ(mtk_dip_params_ycnr, chroma_dither[1], 37, 24, 5),
	BQ(mtk_dip_params_ycnr, center_gain_lo_th_lpf, 38, 0, 5),
	BQ(mtk_dip_params_ycnr, center_gain_hi_th_lpf, 38, 8, 5),
	BQ(mtk_dip_params_ycnr, luma_hf_luma_x[0], 41, 0, 8),
	BQ(mtk_dip_params_ycnr, luma_hf_luma_x[1], 41, 8, 8),
	BQ(mtk_dip_params_ycnr, luma_hf_luma_y[0], 41, 16, 7),
	BQ(mtk_dip_params_ycnr, luma_hf_luma_y[1], 41, 24, 7),
	BQ(mtk_dip_params_ycnr, luma_hf_luma_y[2], 42, 0, 7),
	BQ(mtk_dip_params_ycnr, luma_hf_luma_slope[0], 42, 8, 5),
	BQ(mtk_dip_params_ycnr, luma_hf_luma_slope[1], 42, 16, 5),
	BQ(mtk_dip_params_ycnr, luma_hf_luma_slope[2], 42, 24, 5),
};

static const struct mtk_dip_bq_reserved mtk_dip_ycnr_reserved[] = {
	BQR(mtk_dip_params_ycnr, reserved),
};

static const struct mtk_dip_bq_field mtk_dip_demosaic_fields[] = {
	BQ(mtk_dip_params_demosaic, color_diff_gain_slope, 0, 0, 4),
	BQ(mtk_dip_params_demosaic, color_diff_gain_offset, 0, 4, 8),
	BQ(mtk_dip_params_demosaic, color_diff_gain_ratio, 0, 12, 5),
	BQ(mtk_dip_params_demosaic, color_diff_kernel, 0, 17, 1),
	BQ(mtk_dip_params_demosaic, offset_a, 1, 0, 8),
	BQ(mtk_dip_params_demosaic, level_slope[0], 1, 8, 4),
	BQ(mtk_dip_params_demosaic, level_slope[1], 2, 20, 4),
	BQ(mtk_dip_params_demosaic, level_slope[2], 2, 8, 4),
	BQ(mtk_dip_params_demosaic, level_offset[0], 1, 12, 8),
	BQ(mtk_dip_params_demosaic, level_offset[1], 2, 24, 8),
	BQ(mtk_dip_params_demosaic, level_offset[2], 2, 12, 8),
	BQ(mtk_dip_params_demosaic, color_diff_slope_luma, 1, 20, 5),
	BQ(mtk_dip_params_demosaic, color_diff_slope_chroma, 1, 25, 5),
	BQ(mtk_dip_params_demosaic, offset_b, 2, 0, 8),
	BQ(mtk_dip_params_demosaic, luma_weight[0], 3, 18, 9),
	BQ(mtk_dip_params_demosaic, luma_weight[1], 3, 9, 9),
	BQ(mtk_dip_params_demosaic, luma_weight[2], 3, 0, 9),
	BQ(mtk_dip_params_demosaic, luma_weight[3], 4, 18, 9),
	BQ(mtk_dip_params_demosaic, luma_weight[4], 4, 9, 9),
	BQ(mtk_dip_params_demosaic, luma_weight[5], 4, 0, 9),
	BQ(mtk_dip_params_demosaic, radial_enable, 5, 0, 1),
	BQ(mtk_dip_params_demosaic, radial_ratio2, 5, 1, 5),
	BQ(mtk_dip_params_demosaic, radial_gain[0], 5, 14, 8),
	BQ(mtk_dip_params_demosaic, radial_gain[1], 5, 6, 8),
	BQ(mtk_dip_params_demosaic, core_th[0], 6, 0, 8),
	BQ(mtk_dip_params_demosaic, core_th[1], 9, 0, 8),
	BQ(mtk_dip_params_demosaic, gain_a[0], 6, 13, 5),
	BQ(mtk_dip_params_demosaic, gain_a[1], 6, 8, 5),
	BQ(mtk_dip_params_demosaic, gain_a[2], 16, 0, 6),
	BQ(mtk_dip_params_demosaic, gain_b[0], 6, 23, 5),
	BQ(mtk_dip_params_demosaic, gain_b[1], 6, 18, 5),
	BQ(mtk_dip_params_demosaic, noise_strength[0], 7, 10, 5),
	BQ(mtk_dip_params_demosaic, noise_strength[1], 7, 5, 5),
	BQ(mtk_dip_params_demosaic, noise_strength[2], 7, 0, 5),
	BQ(mtk_dip_params_demosaic, crosstalk_slope, 7, 15, 4),
	BQ(mtk_dip_params_demosaic, crosstalk_offset, 7, 19, 8),
	BQ(mtk_dip_params_demosaic, crosstalk_ratio, 7, 27, 5),
	BQ(mtk_dip_params_demosaic, noise_gain, 8, 0, 4),
	BQ(mtk_dip_params_demosaic, noise_slope, 8, 4, 4),
	BQ(mtk_dip_params_demosaic, noise_offset[0], 8, 24, 8),
	BQ(mtk_dip_params_demosaic, noise_offset[1], 8, 16, 8),
	BQ(mtk_dip_params_demosaic, noise_offset[2], 8, 8, 8),
	BQ(mtk_dip_params_demosaic, ratio_a, 9, 8, 4),
	BQ(mtk_dip_params_demosaic, band_gain[0], 9, 22, 5),
	BQ(mtk_dip_params_demosaic, band_gain[1], 9, 17, 5),
	BQ(mtk_dip_params_demosaic, band_gain[2], 9, 12, 5),
	BQ(mtk_dip_params_demosaic, band_strength, 9, 27, 5),
	BQ(mtk_dip_params_demosaic, band_upper[0], 10, 16, 8),
	BQ(mtk_dip_params_demosaic, band_upper[1], 10, 0, 8),
	BQ(mtk_dip_params_demosaic, band_upper[2], 11, 8, 8),
	BQ(mtk_dip_params_demosaic, band_lower[0], 10, 24, 8),
	BQ(mtk_dip_params_demosaic, band_lower[1], 10, 8, 8),
	BQ(mtk_dip_params_demosaic, band_lower[2], 11, 16, 8),
	BQ(mtk_dip_params_demosaic, band_slope_left, 11, 0, 4),
	BQ(mtk_dip_params_demosaic, band_slope_right, 11, 4, 4),
	BQ(mtk_dip_params_demosaic, clip_th, 12, 0, 8),
	BQ(mtk_dip_params_demosaic, undershoot_th, 12, 8, 8),
	BQ(mtk_dip_params_demosaic, overshoot_th, 12, 16, 8),
	BQ(mtk_dip_params_demosaic, ratio_b, 13, 0, 5),
	BQ(mtk_dip_params_demosaic, radial_ratio, 13, 5, 5),
	BQ(mtk_dip_params_demosaic, mode_a, 13, 10, 1),
	BQ(mtk_dip_params_demosaic, bound_upper, 14, 0, 8),
	BQ(mtk_dip_params_demosaic, bound_lower, 14, 8, 8),
	BQ(mtk_dip_params_demosaic, ratio_c, 15, 0, 4),
	BQ(mtk_dip_params_demosaic, gain_c, 16, 6, 6),
	BQ(mtk_dip_params_demosaic, clip_flag, 17, 0, 1),
	BQ(mtk_dip_params_demosaic, bound_blend, 17, 1, 5),
	BQ(mtk_dip_params_demosaic, threshold_a, 18, 0, 4),
	BQ(mtk_dip_params_demosaic, threshold_b, 18, 4, 4),
	BQ(mtk_dip_params_demosaic, gain_pos, 19, 0, 5),
	BQ(mtk_dip_params_demosaic, gain_neg, 19, 5, 5),
};

static const struct mtk_dip_bq_reserved mtk_dip_demosaic_reserved[] = {
	BQR(mtk_dip_params_demosaic, reserved),
};

static const struct mtk_dip_bq_field mtk_dip_nbc2_fields[] = {
	BQ(mtk_dip_params_nbc2, luma_enable, 0, 1, 1),
	BQ(mtk_dip_params_nbc2, scale_mode, 0, 4, 2),
	BQ(mtk_dip_params_nbc2, chroma_ref_luma, 0, 8, 1),
	BQ(mtk_dip_params_nbc2, radial_link, 0, 24, 1),
	BQ(mtk_dip_params_nbc2, chroma_smooth_edge, 1, 8, 2),
	BQ(mtk_dip_params_nbc2, chroma_filter, 1, 12, 1),
	BQ(mtk_dip_params_nbc2, chroma_threshold, 2, 28, 4),
	BQ(mtk_dip_params_nbc2, luma_scale_x[0], 3, 0, 8),
	BQ(mtk_dip_params_nbc2, luma_scale_x[1], 3, 8, 8),
	BQ(mtk_dip_params_nbc2, luma_scale_x[2], 3, 16, 8),
	BQ(mtk_dip_params_nbc2, luma_scale_y[0], 4, 0, 5),
	BQ(mtk_dip_params_nbc2, luma_scale_y[1], 4, 8, 5),
	BQ(mtk_dip_params_nbc2, luma_scale_y[2], 4, 16, 5),
	BQ(mtk_dip_params_nbc2, luma_scale_y[3], 4, 24, 5),
	BQ(mtk_dip_params_nbc2, luma_scale_slope[0], 5, 0, 5),
	BQ(mtk_dip_params_nbc2, luma_scale_slope[1], 5, 8, 5),
	BQ(mtk_dip_params_nbc2, luma_scale_slope[2], 5, 16, 5),
	BQ(mtk_dip_params_nbc2, luma_scale_slope[3], 5, 24, 5),
	BQ(mtk_dip_params_nbc2, radial_x[0], 6, 0, 8),
	BQ(mtk_dip_params_nbc2, radial_x[1], 6, 8, 8),
	BQ(mtk_dip_params_nbc2, radial_x[2], 6, 16, 8),
	BQ(mtk_dip_params_nbc2, radial_y[0], 7, 0, 6),
	BQ(mtk_dip_params_nbc2, radial_y[1], 7, 8, 6),
	BQ(mtk_dip_params_nbc2, radial_y[2], 7, 16, 6),
	BQ(mtk_dip_params_nbc2, radial_y[3], 7, 24, 6),
	BQ(mtk_dip_params_nbc2, radial_slope[0], 8, 0, 6),
	BQ(mtk_dip_params_nbc2, radial_slope[1], 8, 8, 6),
	BQ(mtk_dip_params_nbc2, radial_slope[2], 8, 16, 6),
	BQ(mtk_dip_params_nbc2, radial_slope[3], 8, 24, 6),
	BQ(mtk_dip_params_nbc2, aux_v_gain, 9, 0, 4),
	BQ(mtk_dip_params_nbc2, aux_gain_th, 9, 8, 5),
	BQ(mtk_dip_params_nbc2, chroma_luma_diff_th, 9, 16, 8),
	BQ(mtk_dip_params_nbc2, chroma_mode, 9, 24, 1),
	BQ(mtk_dip_params_nbc2, aux_th[0], 10, 0, 8),
	BQ(mtk_dip_params_nbc2, aux_th[1], 10, 8, 8),
	BQ(mtk_dip_params_nbc2, aux_th[2], 10, 16, 8),
	BQ(mtk_dip_params_nbc2, aux_th[3], 10, 24, 8),
	BQ(mtk_dip_params_nbc2, radial_chroma_gain, 11, 0, 4),
	BQ(mtk_dip_params_nbc2, luma_weight, 11, 16, 4),
	BQ(mtk_dip_params_nbc2, core_th, 12, 0, 5),
	BQ(mtk_dip_params_nbc2, core_slope, 12, 8, 3),
	BQ(mtk_dip_params_nbc2, diff_th, 12, 12, 5),
	BQ(mtk_dip_params_nbc2, diff_slope, 12, 20, 3),
	BQ(mtk_dip_params_nbc2, class_th[0], 12, 24, 8),
	BQ(mtk_dip_params_nbc2, class_th[1], 13, 12, 6),
	BQ(mtk_dip_params_nbc2, class_th[2], 14, 0, 8),
	BQ(mtk_dip_params_nbc2, class_slope[0], 13, 0, 3),
	BQ(mtk_dip_params_nbc2, class_slope[1], 13, 20, 3),
	BQ(mtk_dip_params_nbc2, class_slope[2], 14, 8, 3),
	BQ(mtk_dip_params_nbc2, class_level[0], 13, 4, 5),
	BQ(mtk_dip_params_nbc2, class_level[1], 13, 24, 5),
	BQ(mtk_dip_params_nbc2, class_level[2], 14, 12, 5),
	BQ(mtk_dip_params_nbc2, class_offset[0], 16, 0, 8),
	BQ(mtk_dip_params_nbc2, class_offset[1], 16, 8, 6),
	BQ(mtk_dip_params_nbc2, class_offset[2], 16, 16, 8),
	BQ(mtk_dip_params_nbc2, variance, 14, 20, 3),
	BQ(mtk_dip_params_nbc2, strength[0], 14, 24, 6),
	BQ(mtk_dip_params_nbc2, strength[1], 15, 0, 6),
	BQ(mtk_dip_params_nbc2, strength[2], 15, 8, 6),
	BQ(mtk_dip_params_nbc2, strength[3], 15, 16, 6),
	BQ(mtk_dip_params_nbc2, strength[4], 15, 24, 6),
	BQ(mtk_dip_params_nbc2, act_blend_base, 17, 8, 7),
	BQ(mtk_dip_params_nbc2, chroma_dither[0], 17, 16, 5),
	BQ(mtk_dip_params_nbc2, chroma_dither[1], 17, 24, 5),
};

/* temporal noise reduction: word 0 is the register at 0x538c, word n 0x5394 + 4 n */
static const struct mtk_dip_bq_field mtk_dip_nr3d_fields[] = {
	BQ(mtk_dip_params_nr3d, luma_enable, 1, 31, 1),
	BQ(mtk_dip_params_nr3d, chroma_enable, 1, 30, 1),
	BQ(mtk_dip_params_nr3d, strength_max, 2, 6, 6),
	BQ(mtk_dip_params_nr3d, blend_ratio[0], 2, 27, 5),
	BQ(mtk_dip_params_nr3d, blend_ratio[1], 2, 22, 5),
	BQ(mtk_dip_params_nr3d, blend_ratio[2], 2, 17, 5),
	BQ(mtk_dip_params_nr3d, blend_ratio[3], 2, 12, 5),
	BQ(mtk_dip_params_nr3d, noise_level, 3, 26, 6),
	BQ(mtk_dip_params_nr3d, noise_slope, 3, 20, 6),
	BQ(mtk_dip_params_nr3d, small_diff_th, 4, 10, 6),
	BQ(mtk_dip_params_nr3d, motion_penalty_th, 4, 16, 6),
	BQ(mtk_dip_params_nr3d, motion_penalty_weight, 4, 22, 4),
	BQ(mtk_dip_params_nr3d, block_diff_th, 4, 26, 4),
	BQ(mtk_dip_params_nr3d, block_luma[0], 5, 26, 6),
	BQ(mtk_dip_params_nr3d, block_luma[1], 5, 20, 6),
	BQ(mtk_dip_params_nr3d, block_luma[2], 5, 14, 6),
	BQ(mtk_dip_params_nr3d, block_luma[3], 5, 8, 6),
	BQ(mtk_dip_params_nr3d, block_luma[4], 5, 2, 6),
	BQ(mtk_dip_params_nr3d, block_luma[5], 6, 26, 6),
	BQ(mtk_dip_params_nr3d, block_luma[6], 6, 20, 6),
	BQ(mtk_dip_params_nr3d, block_luma[7], 6, 14, 6),
	BQ(mtk_dip_params_nr3d, block_chroma[0], 6, 8, 6),
	BQ(mtk_dip_params_nr3d, block_chroma[1], 6, 2, 6),
	BQ(mtk_dip_params_nr3d, block_chroma[2], 7, 26, 6),
	BQ(mtk_dip_params_nr3d, block_chroma[3], 7, 20, 6),
	BQ(mtk_dip_params_nr3d, block_chroma[4], 7, 14, 6),
	BQ(mtk_dip_params_nr3d, block_chroma[5], 7, 8, 6),
	BQ(mtk_dip_params_nr3d, block_chroma[6], 7, 2, 6),
	BQ(mtk_dip_params_nr3d, block_chroma[7], 8, 26, 6),
	BQ(mtk_dip_params_nr3d, texture_level[0], 8, 20, 6),
	BQ(mtk_dip_params_nr3d, texture_level[1], 8, 14, 6),
	BQ(mtk_dip_params_nr3d, texture_level[2], 8, 8, 6),
	BQ(mtk_dip_params_nr3d, texture_level[3], 8, 2, 6),
	BQ(mtk_dip_params_nr3d, texture_level[4], 9, 26, 6),
	BQ(mtk_dip_params_nr3d, texture_level[5], 9, 20, 6),
	BQ(mtk_dip_params_nr3d, texture_level[6], 9, 14, 6),
	BQ(mtk_dip_params_nr3d, texture_level[7], 9, 8, 6),
	BQ(mtk_dip_params_nr3d, detail_base[0], 9, 2, 6),
	BQ(mtk_dip_params_nr3d, detail_base[1], 10, 26, 6),
	BQ(mtk_dip_params_nr3d, detail_base[2], 10, 20, 6),
	BQ(mtk_dip_params_nr3d, detail_base[3], 10, 14, 6),
	BQ(mtk_dip_params_nr3d, detail_base[4], 10, 8, 6),
	BQ(mtk_dip_params_nr3d, detail_base[5], 10, 2, 6),
	BQ(mtk_dip_params_nr3d, detail_base[6], 11, 26, 6),
	BQ(mtk_dip_params_nr3d, detail_base[7], 11, 20, 6),
	BQ(mtk_dip_params_nr3d, texture_base[0], 11, 14, 6),
	BQ(mtk_dip_params_nr3d, texture_base[1], 11, 8, 6),
	BQ(mtk_dip_params_nr3d, texture_base[2], 11, 2, 6),
	BQ(mtk_dip_params_nr3d, texture_base[3], 12, 26, 6),
	BQ(mtk_dip_params_nr3d, texture_base[4], 12, 20, 6),
	BQ(mtk_dip_params_nr3d, texture_base[5], 12, 14, 6),
	BQ(mtk_dip_params_nr3d, texture_base[6], 12, 8, 6),
	BQ(mtk_dip_params_nr3d, texture_base[7], 12, 2, 6),
	BQ(mtk_dip_params_nr3d, motion_curve[0], 13, 26, 6),
	BQ(mtk_dip_params_nr3d, motion_curve[1], 13, 20, 6),
	BQ(mtk_dip_params_nr3d, motion_curve[2], 13, 14, 6),
	BQ(mtk_dip_params_nr3d, motion_curve[3], 13, 8, 6),
	BQ(mtk_dip_params_nr3d, motion_curve[4], 13, 2, 6),
	BQ(mtk_dip_params_nr3d, motion_curve[5], 14, 26, 6),
	BQ(mtk_dip_params_nr3d, motion_curve[6], 14, 20, 6),
	BQ(mtk_dip_params_nr3d, motion_curve[7], 14, 14, 6),
	BQ(mtk_dip_params_nr3d, variance_weight[0], 14, 8, 6),
	BQ(mtk_dip_params_nr3d, variance_weight[1], 14, 2, 6),
	BQ(mtk_dip_params_nr3d, variance_weight[2], 15, 26, 6),
	BQ(mtk_dip_params_nr3d, variance_weight[3], 15, 20, 6),
	BQ(mtk_dip_params_nr3d, variance_weight[4], 15, 14, 6),
	BQ(mtk_dip_params_nr3d, variance_weight[5], 15, 8, 6),
	BQ(mtk_dip_params_nr3d, variance_weight[6], 15, 2, 6),
	BQ(mtk_dip_params_nr3d, variance_weight[7], 16, 26, 6),
	BQ(mtk_dip_params_nr3d, smooth_weight[0], 16, 20, 6),
	BQ(mtk_dip_params_nr3d, smooth_weight[1], 16, 14, 6),
	BQ(mtk_dip_params_nr3d, smooth_weight[2], 16, 8, 6),
	BQ(mtk_dip_params_nr3d, smooth_weight[3], 16, 2, 6),
	BQ(mtk_dip_params_nr3d, smooth_weight[4], 17, 26, 6),
	BQ(mtk_dip_params_nr3d, smooth_weight[5], 17, 20, 6),
	BQ(mtk_dip_params_nr3d, smooth_weight[6], 17, 14, 6),
	BQ(mtk_dip_params_nr3d, smooth_weight[7], 17, 8, 6),
	BQ(mtk_dip_params_nr3d, diff_level[0], 18, 26, 6),
	BQ(mtk_dip_params_nr3d, diff_level[1], 18, 20, 6),
	BQ(mtk_dip_params_nr3d, diff_level[2], 18, 14, 6),
	BQ(mtk_dip_params_nr3d, diff_level[3], 18, 8, 6),
	BQ(mtk_dip_params_nr3d, diff_level[4], 18, 2, 6),
	BQ(mtk_dip_params_nr3d, diff_level[5], 19, 26, 6),
	BQ(mtk_dip_params_nr3d, diff_level[6], 19, 20, 6),
	BQ(mtk_dip_params_nr3d, diff_level[7], 19, 14, 6),
	BQ(mtk_dip_params_nr3d, diff_level[8], 19, 8, 6),
	BQ(mtk_dip_params_nr3d, pattern_mode, 20, 0, 2),
	BQ(mtk_dip_params_nr3d, pattern_cycle[0], 20, 20, 6),
	BQ(mtk_dip_params_nr3d, pattern_cycle[1], 20, 14, 6),
	BQ(mtk_dip_params_nr3d, pattern_cycle[2], 20, 8, 6),
	BQ(mtk_dip_params_nr3d, pattern_cycle[3], 20, 2, 6),
	BQ(mtk_dip_params_nr3d, pattern_texture_th[0], 21, 26, 6),
	BQ(mtk_dip_params_nr3d, pattern_texture_th[1], 21, 20, 6),
	BQ(mtk_dip_params_nr3d, pattern_texture_th[2], 21, 14, 6),
	BQ(mtk_dip_params_nr3d, pattern_texture_th[3], 21, 8, 6),
	BQ(mtk_dip_params_nr3d, pattern_texture_offset, 21, 2, 6),
};

static const struct mtk_dip_bq_reserved mtk_dip_nr3d_reserved[] = {
	BQR(mtk_dip_params_nr3d, reserved),
};

/*
 * Radial weights: words 0 to 3 and 7 to 10 of each weight module; word 4
 * holds the scale of the frame coordinates (driver), words 5, 6 and 11 are 0
 */
static const struct mtk_dip_bq_field mtk_dip_radial_fields[] = {
	BQ(mtk_dip_params_radial, center_x, 0, 0, 14),
	BQ(mtk_dip_params_radial, center_y, 0, 16, 14),
	BQ(mtk_dip_params_radial, radius[0], 1, 0, 14),
	BQ(mtk_dip_params_radial, radius[1], 1, 16, 14),
	BQ(mtk_dip_params_radial, radius[2], 2, 0, 14),
	BQ(mtk_dip_params_radial, weight[0], 2, 16, 8),
	BQ(mtk_dip_params_radial, weight[1], 2, 24, 8),
	BQ(mtk_dip_params_radial, weight[2], 3, 0, 8),
	BQ(mtk_dip_params_radial, weight[3], 3, 8, 8),
	BQ(mtk_dip_params_radial, weight[4], 3, 16, 8),
	BQ(mtk_dip_params_radial, slope[0], 7, 0, 24),
	BQ(mtk_dip_params_radial, slope[1], 8, 0, 24),
	BQ(mtk_dip_params_radial, slope[2], 9, 0, 24),
	BQ(mtk_dip_params_radial, slope[3], 10, 0, 24),
};

static const struct mtk_dip_bq_reserved mtk_dip_radial_reserved[] = {
	BQR(mtk_dip_params_radial, reserved),
};

/*
 * Raw noise reduction: word n is the register at 0x14c0 + 4 n; words 3 to 5
 * hold the normalisation gains in bits 27:15 and its offsets (below)
 */
static const struct mtk_dip_bq_field mtk_dip_rnr_fields[] = {
	BQ(mtk_dip_params_rnr, ratio, 0, 0, 9),
	BQ(mtk_dip_params_rnr, eps, 0, 9, 8),
	BQ(mtk_dip_params_rnr, noise_offset[0], 1, 0, 9),
	BQ(mtk_dip_params_rnr, noise_offset[1], 1, 9, 9),
	BQ(mtk_dip_params_rnr, noise_offset[2], 1, 18, 9),
	BQ(mtk_dip_params_rnr, noise_slope[0], 2, 0, 9),
	BQ(mtk_dip_params_rnr, noise_slope[1], 2, 9, 9),
	BQ(mtk_dip_params_rnr, noise_slope[2], 2, 18, 9),
	BQ(mtk_dip_params_rnr, norm_gain[0], 3, 15, 13),
	BQ(mtk_dip_params_rnr, norm_gain[1], 4, 15, 13),
	BQ(mtk_dip_params_rnr, norm_gain[2], 5, 15, 13),
	BQ(mtk_dip_params_rnr, black_gain, 6, 0, 13),
	BQ(mtk_dip_params_rnr, range[0], 10, 0, 6),
	BQ(mtk_dip_params_rnr, range[1], 10, 6, 6),
	BQ(mtk_dip_params_rnr, range[2], 10, 12, 6),
	BQ(mtk_dip_params_rnr, range[3], 10, 18, 6),
	BQ(mtk_dip_params_rnr, median_ratio, 11, 0, 7),
	BQ(mtk_dip_params_rnr, profile_ratio, 11, 7, 7),
	BQ(mtk_dip_params_rnr, radius, 11, 14, 3),
	BQ(mtk_dip_params_rnr, th_right, 11, 17, 6),
	BQ(mtk_dip_params_rnr, th_left, 11, 23, 6),
	BQ(mtk_dip_params_rnr, flat_mode, 12, 0, 1),
	BQ(mtk_dip_params_rnr, gradient_ratio, 12, 1, 6),
	BQ(mtk_dip_params_rnr, lowpass_ratio, 12, 7, 8),
	BQ(mtk_dip_params_rnr, intensity_offset, 12, 15, 8),
	BQ(mtk_dip_params_rnr, radial_enable, 13, 0, 1),
	BQ(mtk_dip_params_rnr, radial_gain[1], 13, 1, 10),
	BQ(mtk_dip_params_rnr, radial_gain[0], 13, 11, 10),
	BQ(mtk_dip_params_rnr, level_slope[1], 14, 0, 8),
	BQ(mtk_dip_params_rnr, level_th[1], 14, 8, 8),
	BQ(mtk_dip_params_rnr, level_slope[0], 14, 16, 8),
	BQ(mtk_dip_params_rnr, level_th[0], 14, 24, 8),
};

static const struct mtk_dip_bq_reserved mtk_dip_rnr_reserved[] = {
	BQR(mtk_dip_params_rnr, reserved),
};

/**
 * struct mtk_dip_bq_block - an image quality block
 * @fields: its parameters
 * @n_fields: number of @fields
 * @reserved: its reserved bytes
 * @n_reserved: number of @reserved
 * @words: register words of the module
 */
struct mtk_dip_bq_block {
	const struct mtk_dip_bq_field *fields;
	unsigned int n_fields;
	const struct mtk_dip_bq_reserved *reserved;
	unsigned int n_reserved;
	unsigned int words;
};

static const struct mtk_dip_bq_block mtk_dip_bq_ee = {
	mtk_dip_ee_fields, ARRAY_SIZE(mtk_dip_ee_fields),
	mtk_dip_ee_reserved, ARRAY_SIZE(mtk_dip_ee_reserved),
	MTK_DIP_EE_WORDS,
};

static const struct mtk_dip_bq_block mtk_dip_bq_ycnr = {
	mtk_dip_ycnr_fields, ARRAY_SIZE(mtk_dip_ycnr_fields),
	mtk_dip_ycnr_reserved, ARRAY_SIZE(mtk_dip_ycnr_reserved),
	MTK_DIP_YCNR_WORDS,
};

static const struct mtk_dip_bq_block mtk_dip_bq_demosaic = {
	mtk_dip_demosaic_fields, ARRAY_SIZE(mtk_dip_demosaic_fields),
	mtk_dip_demosaic_reserved, ARRAY_SIZE(mtk_dip_demosaic_reserved),
	MTK_DIP_DEMOSAIC_WORDS,
};

static const struct mtk_dip_bq_block mtk_dip_bq_nbc2 = {
	mtk_dip_nbc2_fields, ARRAY_SIZE(mtk_dip_nbc2_fields),
	NULL, 0,
	MTK_DIP_NBC2_WORDS,
};

static const struct mtk_dip_bq_block mtk_dip_bq_nr3d = {
	mtk_dip_nr3d_fields, ARRAY_SIZE(mtk_dip_nr3d_fields),
	mtk_dip_nr3d_reserved, ARRAY_SIZE(mtk_dip_nr3d_reserved),
	MTK_DIP_NR3D_WORDS,
};

static const struct mtk_dip_bq_block mtk_dip_bq_radial = {
	mtk_dip_radial_fields, ARRAY_SIZE(mtk_dip_radial_fields),
	mtk_dip_radial_reserved, ARRAY_SIZE(mtk_dip_radial_reserved),
	MTK_DIP_RADIAL_WORDS,
};

static const struct mtk_dip_bq_block mtk_dip_bq_rnr = {
	mtk_dip_rnr_fields, ARRAY_SIZE(mtk_dip_rnr_fields),
	mtk_dip_rnr_reserved, ARRAY_SIZE(mtk_dip_rnr_reserved),
	MTK_DIP_RNR_WORDS,
};

/* radial weights: unity scale of the frame coordinates (2048) on both axes */
#define RADIAL_SCALE_WORD	4
#define RADIAL_SCALE_ONE	0x08000800

/*
 * Raw noise reduction: the offsets of the normalisation are signed 12 bit,
 * held sign extended in bits 14:0 of words 3 to 5; words 7 to 9 have fixed
 * values
 */
#define RNR_NORM_WORD		3
#define RNR_NORM_OFFSET		GENMASK(14, 0)
#define RNR_NORM_OFFSET_MIN	(-2048)
#define RNR_NORM_OFFSET_MAX	2047
#define RNR_FIXED_WORD		7
static const u32 mtk_dip_rnr_fixed[] = { 0x01188c46, 0x004c2613, 0x02241489 };

/*
 * Neutral settings of the modules that stay in the pipeline: the least
 * filtering their fields allow. The first noise reduction stage needs one of
 * its parts on for its tile margins, the second its chroma part with a scale
 * of at least 2 (fixed in the words).
 */
static const struct mtk_dip_params_ycnr mtk_dip_ycnr_neutral = {
	.chroma_enable = 1,
};

static const struct mtk_dip_params_demosaic mtk_dip_demosaic_neutral;

static const struct mtk_dip_params_nbc2 mtk_dip_nbc2_neutral = {
	.scale_mode = 3,
};

static const struct mtk_dip_params_radial mtk_dip_radial_neutral;

static const struct mtk_dip_params_rnr mtk_dip_rnr_neutral = {
	.norm_gain = { 512, 512, 512 },
	.black_gain = 512,
};

/*
 * Second noise reduction stage: the chroma part (word 0, bit 0) is always on
 * and the other filter mode (word 0, bit 12) always off, both because of the
 * tile margins; bits 19:16 of word 0 and bit 7 of word 9 have fixed values.
 * Scales below 2 have smaller tile margins than the ones of the tile
 * computation.
 */
#define NBC2_W0_FIXED		(BIT(0) | GENMASK(19, 16))
#define NBC2_W9_FIXED		BIT(7)
#define NBC2_SCALE_MIN		2

/*
 * Temporal noise reduction: word 0 holds the module mode (fixed 0xf00), the
 * enable of the blend window (bit 4) and the switch that makes the module
 * ignore its radial weights (bit 20); bits 5:0 of word 2 are fixed at 0x30.
 */
#define NR3D_W0_FIXED		(GENMASK(11, 8) | BIT(4))
#define NR3D_W0_RADIAL_OFF	BIT(20)
#define NR3D_W2_FIXED		0x30
#define NR3D_FLAGS		MTK_DIP_NR3D_FL_RESTART

/* the response gain curve of the edge enhancement: words of its segments */
#define EE_CURVE_WORD		7
#define EE_CURVE_SEGS		5
#define EE_CURVE_SLOPE		GENMASK(7, 0)
#define EE_CURVE_X_END		255
#define EE_CURVE_Y_END_MIN	(-2048)
#define EE_CURVE_Y_END_MAX	3071
#define EE_SLOPE_MIN		(-128)
#define EE_SLOPE_MAX		127

static u32 mtk_dip_bq_value(const void *blk, const struct mtk_dip_bq_field *f)
{
	const u8 *p = (const u8 *)blk + f->off;
	u32 v32;
	u16 v;

	if (f->size == 1)
		return *p;
	if (f->size == 4) {
		memcpy(&v32, p, sizeof(v32));
		return v32;
	}
	memcpy(&v, p, sizeof(v));

	return v;
}

static int mtk_dip_bq_check_fields(const void *blk,
				   const struct mtk_dip_bq_block *b)
{
	const u8 *p = blk;
	unsigned int i, k;

	for (i = 0; i < b->n_fields; i++)
		if (mtk_dip_bq_value(blk, &b->fields[i]) >
		    GENMASK(b->fields[i].width - 1, 0))
			return -EINVAL;
	for (i = 0; i < b->n_reserved; i++)
		for (k = 0; k < b->reserved[i].len; k++)
			if (p[b->reserved[i].off + k])
				return -EINVAL;

	return 0;
}

static void mtk_dip_bq_pack(const void *blk, const struct mtk_dip_bq_block *b,
			    u32 *words)
{
	const struct mtk_dip_bq_field *f;
	unsigned int i;

	memset(words, 0, b->words * sizeof(*words));
	for (i = 0; i < b->n_fields; i++) {
		f = &b->fields[i];
		words[f->word] |= mtk_dip_bq_value(blk, f) << f->lsb;
	}
}

static int mtk_dip_bq_check_ee(const struct mtk_dip_params_ee *ee)
{
	unsigned int i;
	u8 x = ee->gain_curve_x[0];

	/*
	 * Inner points strictly rising inside the input range; the first one
	 * may sit at input 0, which leaves the first segment empty
	 */
	for (i = 1; i < ARRAY_SIZE(ee->gain_curve_x); i++) {
		if (ee->gain_curve_x[i] <= x)
			return -EINVAL;
		x = ee->gain_curve_x[i];
	}
	if (x >= EE_CURVE_X_END)
		return -EINVAL;
	if (ee->gain_curve_y_end < EE_CURVE_Y_END_MIN ||
	    ee->gain_curve_y_end > EE_CURVE_Y_END_MAX)
		return -EINVAL;

	return mtk_dip_bq_check_fields(ee, &mtk_dip_bq_ee);
}

static int mtk_dip_bq_check_ycnr(const struct mtk_dip_params_ycnr *nr)
{
	if (!nr->luma_enable && !nr->chroma_enable)
		return -EINVAL;

	return mtk_dip_bq_check_fields(nr, &mtk_dip_bq_ycnr);
}

static int mtk_dip_bq_check_nbc2(const struct mtk_dip_params_nbc2 *nr)
{
	if (nr->scale_mode < NBC2_SCALE_MIN)
		return -EINVAL;

	return mtk_dip_bq_check_fields(nr, &mtk_dip_bq_nbc2);
}

static int mtk_dip_bq_check_rnr(const struct mtk_dip_params_rnr *nr)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(nr->norm_offset); i++)
		if (nr->norm_offset[i] < RNR_NORM_OFFSET_MIN ||
		    nr->norm_offset[i] > RNR_NORM_OFFSET_MAX)
			return -EINVAL;

	return mtk_dip_bq_check_fields(nr, &mtk_dip_bq_rnr);
}

static int mtk_dip_bq_check_nr3d(const struct mtk_dip_params_nr3d *nr)
{
	if (nr->flags & ~NR3D_FLAGS || nr->radial_link > 1 ||
	    (!nr->luma_enable && !nr->chroma_enable))
		return -EINVAL;
	if (nr->motion_x & 1 || nr->motion_y & 1 ||
	    abs(nr->motion_x) > MTK_DIP_NR3D_MOTION_MAX ||
	    abs(nr->motion_y) > MTK_DIP_NR3D_MOTION_MAX)
		return -EINVAL;

	return mtk_dip_bq_check_fields(nr, &mtk_dip_bq_nr3d);
}

/**
 * mtk_dip_bq_check - check an enabled image quality block
 * @h: the block
 *
 * Return: 0, -EINVAL for a value out of range, -ENOENT for a block of another
 * type.
 */
int mtk_dip_bq_check(const struct v4l2_isp_params_block_header *h)
{
	switch (h->type) {
	case MTK_DIP_PARAMS_BLOCK_EE:
		return mtk_dip_bq_check_ee(container_of(h, struct mtk_dip_params_ee,
							header));
	case MTK_DIP_PARAMS_BLOCK_YCNR:
		return mtk_dip_bq_check_ycnr(container_of(h,
							  struct mtk_dip_params_ycnr,
							  header));
	case MTK_DIP_PARAMS_BLOCK_DEMOSAIC:
		return mtk_dip_bq_check_fields(h, &mtk_dip_bq_demosaic);
	case MTK_DIP_PARAMS_BLOCK_NBC2:
		return mtk_dip_bq_check_nbc2(container_of(h,
							  struct mtk_dip_params_nbc2,
							  header));
	case MTK_DIP_PARAMS_BLOCK_NR3D:
		return mtk_dip_bq_check_nr3d(container_of(h,
							  struct mtk_dip_params_nr3d,
							  header));
	case MTK_DIP_PARAMS_BLOCK_RADIAL:
		return mtk_dip_bq_check_fields(h, &mtk_dip_bq_radial);
	case MTK_DIP_PARAMS_BLOCK_RNR:
		return mtk_dip_bq_check_rnr(container_of(h,
							 struct mtk_dip_params_rnr,
							 header));
	}

	return -ENOENT;
}

/*
 * 16 * dy / dx, rounded half away from zero, signed 8 bit; an empty segment
 * (dx 0, only the first one can be) gets the steepest slope towards its step,
 * which no input reaches
 */
static s32 mtk_dip_ee_slope(s32 dx, s32 dy)
{
	s32 q = 16 * dy;

	if (!dx)
		return dy > 0 ? EE_SLOPE_MAX : dy < 0 ? EE_SLOPE_MIN : 0;

	return clamp(DIV_ROUND_CLOSEST(q, dx), EE_SLOPE_MIN, EE_SLOPE_MAX);
}

/**
 * mtk_dip_bq_ee_words - register words of the edge enhancement
 * @ee: checked block
 * @words: MTK_DIP_EE_WORDS words
 */
void mtk_dip_bq_ee_words(const struct mtk_dip_params_ee *ee, u32 *words)
{
	s32 x[EE_CURVE_SEGS + 1], y[EE_CURVE_SEGS + 1];
	unsigned int i;

	mtk_dip_bq_pack(ee, &mtk_dip_bq_ee, words);

	x[0] = 0;
	y[0] = 0;
	for (i = 0; i < ARRAY_SIZE(ee->gain_curve_x); i++) {
		x[i + 1] = ee->gain_curve_x[i];
		y[i + 1] = ee->gain_curve_y[i];
	}
	x[EE_CURVE_SEGS] = EE_CURVE_X_END;
	y[EE_CURVE_SEGS] = ee->gain_curve_y_end;
	for (i = 0; i < EE_CURVE_SEGS; i++)
		words[EE_CURVE_WORD + i] |=
			FIELD_PREP(EE_CURVE_SLOPE,
				   (u32)mtk_dip_ee_slope(x[i + 1] - x[i],
							 y[i + 1] - y[i]) & 0xff);
}

/**
 * mtk_dip_bq_ycnr_words - register words of the first noise reduction stage
 * @nr: checked block
 * @words: MTK_DIP_YCNR_WORDS words, without the fixed bits of the driver
 */
void mtk_dip_bq_ycnr_words(const struct mtk_dip_params_ycnr *nr, u32 *words)
{
	mtk_dip_bq_pack(nr, &mtk_dip_bq_ycnr, words);
}

/**
 * mtk_dip_bq_demosaic_words - register words of the demosaicing
 * @dm: checked block
 * @words: MTK_DIP_DEMOSAIC_WORDS words
 */
void mtk_dip_bq_demosaic_words(const struct mtk_dip_params_demosaic *dm,
			       u32 *words)
{
	mtk_dip_bq_pack(dm, &mtk_dip_bq_demosaic, words);
}

/**
 * mtk_dip_bq_nbc2_words - register words of the second noise reduction stage
 * @nr: checked block
 * @words: MTK_DIP_NBC2_WORDS words, with the fixed bits of the driver
 */
void mtk_dip_bq_nbc2_words(const struct mtk_dip_params_nbc2 *nr, u32 *words)
{
	mtk_dip_bq_pack(nr, &mtk_dip_bq_nbc2, words);
	words[0] |= NBC2_W0_FIXED;
	words[9] |= NBC2_W9_FIXED;
}

/**
 * mtk_dip_bq_nr3d_words - register words of the temporal noise reduction
 * @nr: checked block
 * @words: MTK_DIP_NR3D_WORDS words, with the fixed bits of the driver
 */
void mtk_dip_bq_nr3d_words(const struct mtk_dip_params_nr3d *nr, u32 *words)
{
	mtk_dip_bq_pack(nr, &mtk_dip_bq_nr3d, words);
	words[0] |= NR3D_W0_FIXED | (nr->radial_link ? 0 : NR3D_W0_RADIAL_OFF);
	words[2] |= NR3D_W2_FIXED;
}

/**
 * mtk_dip_bq_radial_words - register words of a radial weight module
 * @rw: checked block
 * @words: MTK_DIP_RADIAL_WORDS words, with the scale word of the driver
 */
void mtk_dip_bq_radial_words(const struct mtk_dip_params_radial *rw,
			     u32 *words)
{
	mtk_dip_bq_pack(rw, &mtk_dip_bq_radial, words);
	words[RADIAL_SCALE_WORD] = RADIAL_SCALE_ONE;
}

/**
 * mtk_dip_bq_rnr_words - register words of the raw noise reduction
 * @nr: checked block
 * @words: MTK_DIP_RNR_WORDS words, with the fixed words of the driver
 */
void mtk_dip_bq_rnr_words(const struct mtk_dip_params_rnr *nr, u32 *words)
{
	unsigned int i;
	u32 v;

	mtk_dip_bq_pack(nr, &mtk_dip_bq_rnr, words);
	for (i = 0; i < ARRAY_SIZE(nr->norm_offset); i++) {
		v = (u32)(s32)nr->norm_offset[i];
		words[RNR_NORM_WORD + i] |= FIELD_PREP(RNR_NORM_OFFSET,
						       v & RNR_NORM_OFFSET);
	}
	memcpy(&words[RNR_FIXED_WORD], mtk_dip_rnr_fixed,
	       sizeof(mtk_dip_rnr_fixed));
}

/**
 * mtk_dip_bq_neutral_words - register words of the neutral settings
 * @type: MTK_DIP_PARAMS_BLOCK_YCNR, _DEMOSAIC, _NBC2, _RADIAL or _RNR
 * @words: the words of the module, as the function for the block gives them
 */
void mtk_dip_bq_neutral_words(unsigned int type, u32 *words)
{
	switch (type) {
	case MTK_DIP_PARAMS_BLOCK_YCNR:
		mtk_dip_bq_ycnr_words(&mtk_dip_ycnr_neutral, words);
		break;
	case MTK_DIP_PARAMS_BLOCK_DEMOSAIC:
		mtk_dip_bq_demosaic_words(&mtk_dip_demosaic_neutral, words);
		break;
	case MTK_DIP_PARAMS_BLOCK_NBC2:
		mtk_dip_bq_nbc2_words(&mtk_dip_nbc2_neutral, words);
		break;
	case MTK_DIP_PARAMS_BLOCK_RADIAL:
		mtk_dip_bq_radial_words(&mtk_dip_radial_neutral, words);
		break;
	case MTK_DIP_PARAMS_BLOCK_RNR:
		mtk_dip_bq_rnr_words(&mtk_dip_rnr_neutral, words);
		break;
	}
}
