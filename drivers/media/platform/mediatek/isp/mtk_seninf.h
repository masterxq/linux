/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Interface of the SENINF driver to the CAMSV driver.
 */
#ifndef __MTK_SENINF_H__
#define __MTK_SENINF_H__

struct v4l2_subdev;

/*
 * mtk_seninf_pixel_mode - pixels per clock of a SENINF source pad
 * @sd: the SENINF subdev
 * @pad: the source pad
 *
 * Returns 0 (one pixel per clock), 1 (two) or 2 (four) for the stream that
 * the active routing feeds to @pad, -EOPNOTSUPP when the SENINF model has a
 * fixed pixel mode, or another negative error code.
 */
int mtk_seninf_pixel_mode(struct v4l2_subdev *sd, unsigned int pad);

#endif
