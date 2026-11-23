/* SPDX-License-Identifier: ((GPL-2.0+ WITH Linux-syscall-note) OR MIT) */
/*
 * Userspace interface of the MT6785 ISP pass 2 ("DIP"): a memory-to-memory
 * image processor that turns one raw Bayer frame, or one NV12 frame, into
 * YUV frames.
 *
 * Device model (media controller, driver and model "mtk-dip", one processing
 * entity "mtk-dip", no subdev controls):
 *
 *   video node            direction      queue type              format
 *   mtk-dip-raw           to the ISP     VIDEO_OUTPUT_MPLANE     raw Bayer (below)
 *   mtk-dip-params        to the ISP     META_OUTPUT             V4L2_META_FMT_MTK_DIP_PARAMS
 *   mtk-dip-main          from the ISP   VIDEO_CAPTURE_MPLANE    NV12
 *   mtk-dip-scaled        from the ISP   VIDEO_CAPTURE_MPLANE    YUYV
 *   mtk-dip-base          to the ISP     VIDEO_OUTPUT_MPLANE     NV12
 *   mtk-dip-weight        to the ISP     VIDEO_OUTPUT_MPLANE     GREY
 *
 * The links of mtk-dip-raw, mtk-dip-main and mtk-dip-scaled are enabled and
 * immutable, the link of mtk-dip-params is enabled by default and can be
 * disabled. A job runs when mtk-dip-raw and at least one of mtk-dip-main and
 * mtk-dip-scaled stream, and mtk-dip-raw and every streaming output hold a
 * queued buffer: each job writes all streaming outputs from the same raw
 * frame. Buffers are taken in queue order. An output that starts or stops
 * streaming while the others stream joins or leaves from the next job on.
 * mtk-dip-params never holds a job
 * back: if it streams with its link enabled and a buffer is queued, the first
 * one belongs to the next job; without a params buffer, or with one that
 * omits a block, the values of the previous job stay in effect. Before the
 * first job the driver uses neutral values (unity gains, identity matrix,
 * gamma bypass, no lens shading, no edge enhancement, flat radial weights)
 * and the least filtering the noise reduction and demosaicing modules allow
 * (see their blocks). Sequence numbers
 * and timestamps of the capture buffers are copied from the raw buffer. A
 * job that fails returns its raw, params and output buffers with
 * V4L2_BUF_FLAG_ERROR, in queue order. STREAMOFF on any
 * node waits for the running job and returns the buffers of that node.
 *
 * Opening a node does not power the block; it is powered for the jobs and
 * goes off shortly after the last one.
 *
 * Geometry is fixed between streamon and streamoff. The processed area is
 * set with VIDIOC_S_SELECTION(V4L2_SEL_TGT_CROP) on mtk-dip-raw; the format
 * of mtk-dip-main has to have the size of that area (no scaling on the main
 * output), streamon fails with -EPIPE otherwise. Crop left and top are even
 * (the Bayer phase stays), width and height even, at least 128x64, at most
 * the raw frame; the tile computation cannot serve a few bands of widths
 * with or without the edge enhancement (690-728, 1322-1366, 1960-2004,
 * 2598-2642, 3236-3280, 3874-3918, 4512-4556), which S_SELECTION and
 * TRY_FMT round down. ENUM_FRAMESIZES on
 * mtk-dip-raw and mtk-dip-main reports the steps (2x2) and limits. The
 * bytesperline of mtk-dip-main is a multiple of 16, at least the width and
 * at most 9248.
 *
 * mtk-dip-scaled scales the same area, from 1/2 down to 1/16 of its width
 * and of its height (each axis on its own, so the aspect ratio may change),
 * at least 32x16, width and height even. The scaler makes each output pixel
 * from the input pixels it covers (an area average). TRY_FMT and S_FMT on
 * mtk-dip-scaled take the crop rectangle of mtk-dip-raw at that time: set
 * the raw format and the crop first. They round the size into these limits
 * and the width down to one the tile computation serves with this area
 * (with and without edge enhancement); streamon fails with -EPIPE if the
 * crop changed since to an area the scaled size does not fit. Enlarging or
 * a factor between 1/2 and 1 is not supported. ENUM_FRAMESIZES reports the
 * steps (2x2) and the limits for the largest area (32x16 to 2312x1736). The
 * bytesperline of mtk-dip-scaled is a multiple of 16, at least twice the
 * width and at most 9248. Zoom is a crop on mtk-dip-raw with a scaled output
 * of a fixed size, set between streamoff and streamon of mtk-dip-raw.
 *
 * Raw formats on mtk-dip-raw, each in the four Bayer orders:
 *  - V4L2_PIX_FMT_SBGGR8, _SGBRG8, _SGRBG8, _SRGGB8: one byte per pixel.
 *  - V4L2_PIX_FMT_MTISP_SBGGR10, _SGBRG10, _SGRBG10, _SRGGB10: 10 bits per
 *    pixel as one bit stream per line, least significant bit first (4 pixels
 *    in 5 bytes), the layout of the MediaTek CAMSV and pass 1 raw outputs.
 * The order is that of the top left 2x2 cell of the frame; the crop keeps it
 * (even left and top). The per-position arrays of the params blocks follow
 * the order of the format (see MTK_DIP_BAYER_CHANNELS); the driver places
 * them in the colour registers of the hardware.
 * bytesperline is at least DIV_ROUND_UP(width * bpp, 8), at most 9248 and a
 * multiple of 2; a longer line of the caller is kept (the CAMSV writes 5780
 * bytes for 4624 pixels, pass 1 5784). The frame starts at the start of the
 * buffer: a data_offset on mtk-dip-raw is refused with -EINVAL at QBUF.
 *
 * Buffers must be physically contiguous and below 4 GiB (the DIP works
 * without IOMMU); the nodes allocate such buffers and export them
 * (VIDIOC_EXPBUF), and import DMABUFs (for instance CAMSV buffers on
 * mtk-dip-raw); an imported DMABUF that does not qualify is refused with
 * -EINVAL at QBUF. Allocated buffers come from the camera memory region of
 * the board; REQBUFS returns fewer buffers than asked for, or fails with
 * -ENOMEM, once the region is full.
 *
 * YUV input and blend: with V4L2_PIX_FMT_NV12 on mtk-dip-raw (Y plane, then
 * the interleaved UV plane, both with the bytesperline of the format, a
 * multiple of 16) the job runs only the YUV part of the block: both noise
 * reduction stages (MTK_DIP_PARAMS_BLOCK_YCNR and _NBC2), the edge
 * enhancement (MTK_DIP_PARAMS_BLOCK_EE) and the main output; the blocks of
 * the raw stages are accepted and kept but have no effect. The processed
 * area is the whole frame (S_SELECTION returns it), mtk-dip-scaled does not
 * take part (streamon fails with -EPIPE), and widths the tile computation
 * cannot serve round down in TRY_FMT. When mtk-dip-base and mtk-dip-weight
 * stream as well, each job also takes one buffer from each of them and
 * blends the input with the base image (NV12) by the weight map (8 bits per
 * pixel), pixel by pixel, in front of the noise reduction stages, with the
 * settings of MTK_DIP_PARAMS_BLOCK_MIX: the last step of a multi-frame
 * photo, the input being the blended result of several frames and the base
 * image the reference frame. Both nodes take the size of the raw frame
 * (streamon fails with -EPIPE otherwise) and only stream with an NV12 raw
 * format; a job waits until both have a buffer. With a Bayer format on
 * mtk-dip-raw nothing changes: mtk-dip-base and mtk-dip-weight refuse to
 * start (-EPIPE).
 *
 * Nothing in this interface carries register values, tile data or physical
 * addresses. The driver validates every block and translates it.
 */
#ifndef _UAPI_MTK_DIP_MT6785_H
#define _UAPI_MTK_DIP_MT6785_H

#include <linux/types.h>
#include <linux/videodev2.h>
#include <linux/media/v4l2-isp.h>

/*
 * The packed raw formats are defined in videodev2.h, the definitions here
 * serve user space built against older headers. The params format exists
 * only here.
 */
#ifndef V4L2_PIX_FMT_MTISP_SBGGR10
/* MediaTek packed 10-bit Bayer, LSB first */
#define V4L2_PIX_FMT_MTISP_SBGGR10	v4l2_fourcc('M', 'B', 'B', 'A')
#define V4L2_PIX_FMT_MTISP_SGBRG10	v4l2_fourcc('M', 'B', 'G', 'A')
#define V4L2_PIX_FMT_MTISP_SGRBG10	v4l2_fourcc('M', 'B', 'g', 'A')
#define V4L2_PIX_FMT_MTISP_SRGGB10	v4l2_fourcc('M', 'B', 'R', 'A')
#endif
#ifndef V4L2_META_FMT_MTK_DIP_PARAMS
/* Parameters of mtk-dip-params */
#define V4L2_META_FMT_MTK_DIP_PARAMS	v4l2_fourcc('M', 'D', 'P', 'P')
#endif

#define MTK_DIP_PARAMS_VERSION_V1	1

/*
 * Size of a params buffer: the header and one block of each type fit (4296
 * bytes with the blocks up to MTK_DIP_PARAMS_BLOCK_RNR). New block types do
 * not change the meaning of the old ones; the version stays. The size grows
 * with them: 6144 with the blocks up to MTK_DIP_PARAMS_BLOCK_MIX, 6656 with
 * MTK_DIP_PARAMS_BLOCK_RADIAL and MTK_DIP_PARAMS_BLOCK_RNR, so that user space
 * can tell from the buffer size which types a driver takes.
 */
#define MTK_DIP_PARAMS_MAX_SIZE		6656

/**
 * enum mtk_dip_params_block_type - type of a block in the params buffer
 * @MTK_DIP_PARAMS_BLOCK_BLC: black level and digital gain
 * @MTK_DIP_PARAMS_BLOCK_WB: white balance gains
 * @MTK_DIP_PARAMS_BLOCK_CCM: colour matrix
 * @MTK_DIP_PARAMS_BLOCK_GAMMA: tone curve
 * @MTK_DIP_PARAMS_BLOCK_LSC: lens shading correction
 * @MTK_DIP_PARAMS_BLOCK_EE: edge enhancement
 * @MTK_DIP_PARAMS_BLOCK_YCNR: luma/chroma noise reduction, first stage
 * @MTK_DIP_PARAMS_BLOCK_DEMOSAIC: demosaicing
 * @MTK_DIP_PARAMS_BLOCK_NBC2: luma/chroma noise reduction, second stage
 * @MTK_DIP_PARAMS_BLOCK_NR3D: temporal noise reduction (type 9 is not used)
 * @MTK_DIP_PARAMS_BLOCK_MIX: blend with the base image (YUV input only)
 * @MTK_DIP_PARAMS_BLOCK_RADIAL: radial weights of the filter modules
 * @MTK_DIP_PARAMS_BLOCK_RNR: raw noise reduction
 *
 * The buffer is a struct v4l2_isp_params_buffer (version
 * MTK_DIP_PARAMS_VERSION_V1) followed by blocks, each starting with a
 * struct v4l2_isp_params_block_header. Unknown types, bad sizes and values
 * out of range fail the buffer with -EINVAL at QBUF; a job never runs with
 * half a buffer.
 */
enum mtk_dip_params_block_type {
	MTK_DIP_PARAMS_BLOCK_BLC = 0,
	MTK_DIP_PARAMS_BLOCK_WB = 1,
	MTK_DIP_PARAMS_BLOCK_CCM = 2,
	MTK_DIP_PARAMS_BLOCK_GAMMA = 3,
	MTK_DIP_PARAMS_BLOCK_LSC = 4,
	MTK_DIP_PARAMS_BLOCK_EE = 5,
	MTK_DIP_PARAMS_BLOCK_YCNR = 6,
	MTK_DIP_PARAMS_BLOCK_DEMOSAIC = 7,
	MTK_DIP_PARAMS_BLOCK_NBC2 = 8,
	MTK_DIP_PARAMS_BLOCK_NR3D = 10,
	MTK_DIP_PARAMS_BLOCK_MIX = 11,
	MTK_DIP_PARAMS_BLOCK_RADIAL = 12,
	MTK_DIP_PARAMS_BLOCK_RNR = 13,
};

/*
 * Block flags (struct v4l2_isp_params_block_header.flags):
 * V4L2_ISP_PARAMS_FL_BLOCK_DISABLE: module neutral, the payload may be
 *	omitted (header only) and is ignored.
 * V4L2_ISP_PARAMS_FL_BLOCK_ENABLE, or no flag: the payload is applied.
 */
#define MTK_DIP_PARAMS_BLOCK_FL_DISABLE	V4L2_ISP_PARAMS_FL_BLOCK_DISABLE
#define MTK_DIP_PARAMS_BLOCK_FL_ENABLE	V4L2_ISP_PARAMS_FL_BLOCK_ENABLE

/*
 * Channel order of all per-channel arrays: the four positions of the Bayer
 * cell in raster order (top left, top right, bottom left, bottom right) of
 * the format set on mtk-dip-raw. For BGGR that is B, Gb, Gr, R; for GRBG
 * Gr, R, B, Gb; for GBRG Gb, B, R, Gr; for RGGB R, Gr, Gb, B.
 */
#define MTK_DIP_BAYER_CHANNELS	4

/**
 * struct mtk_dip_params_blc - black level and digital gain, first stage
 * @header: block header, type MTK_DIP_PARAMS_BLOCK_BLC
 * @level: black level per Bayer position, unsigned Q0.16 of full scale
 *	(for 10 bit data a level of 64 is 64 << 6). Subtracted before @gain.
 *	The hardware resolves 1/4096 of full scale.
 * @gain: digital gain per Bayer position, unsigned Q4.12 (1.0 = 0x1000),
 *	range 0 to just below 16.0. The hardware resolves 1/512.
 *
 * Output is clipped to 12 bits. Neutral: level 0, gain 1.0.
 */
struct mtk_dip_params_blc {
	struct v4l2_isp_params_block_header header;
	__u16 level[MTK_DIP_BAYER_CHANNELS];
	__u16 gain[MTK_DIP_BAYER_CHANNELS];
} __attribute__((aligned(8)));

/**
 * struct mtk_dip_params_wb - white balance gains, before demosaicing
 * @header: block header, type MTK_DIP_PARAMS_BLOCK_WB
 * @gain: gain per Bayer position, unsigned Q4.12, range 0 to just below 16.0,
 *	hardware resolution 1/512.
 *
 * Neutral: 1.0 everywhere.
 */
struct mtk_dip_params_wb {
	struct v4l2_isp_params_block_header header;
	__u16 gain[MTK_DIP_BAYER_CHANNELS];
} __attribute__((aligned(8)));

/**
 * struct mtk_dip_params_ccm - colour matrix, after demosaicing, before gamma
 * @header: block header, type MTK_DIP_PARAMS_BLOCK_CCM
 * @coeff: row-major 3x3 matrix, signed Q4.12 (1.0 = 0x1000), range -8.0 to
 *	just below 8.0, hardware resolution 1/512. Row = output channel
 *	R, G, B; column = input channel R, G, B.
 * @reserved: must be 0
 *
 * Works on linear data. Results below zero are raised to a small positive
 * floor by the hardware. Neutral: identity.
 */
struct mtk_dip_params_ccm {
	struct v4l2_isp_params_block_header header;
	__s16 coeff[3][3];
	__u16 reserved[3];
} __attribute__((aligned(8)));

/*
 * Gamma curve as output values at fixed, non-uniform input positions, in
 * units of 1/1024 of full scale: MTK_DIP_GAMMA_SEG0 points with a step of 2
 * (inputs 0 to 126), then MTK_DIP_GAMMA_SEG1 points with a step of 4 (128
 * to 252), then MTK_DIP_GAMMA_SEG2 points with a step of 8 (256 to 1016).
 * Inputs above 1016 take the value of the last point. Between points the
 * hardware interpolates linearly.
 */
#define MTK_DIP_GAMMA_INPUT_SCALE	1024
#define MTK_DIP_GAMMA_SEG0	64
#define MTK_DIP_GAMMA_STEP0	2
#define MTK_DIP_GAMMA_SEG1	32
#define MTK_DIP_GAMMA_STEP1	4
#define MTK_DIP_GAMMA_SEG2	96
#define MTK_DIP_GAMMA_STEP2	8
#define MTK_DIP_GAMMA_POINTS	(MTK_DIP_GAMMA_SEG0 + MTK_DIP_GAMMA_SEG1 + \
				 MTK_DIP_GAMMA_SEG2)

/**
 * struct mtk_dip_params_gamma - tone curve, after the colour matrix
 * @header: block header, type MTK_DIP_PARAMS_BLOCK_GAMMA
 * @lut: output value per point and channel (R, G, B), unsigned 10 bit
 *	(0 to 1023).
 *
 * With MTK_DIP_PARAMS_BLOCK_FL_DISABLE the module is bypassed (linear).
 */
struct mtk_dip_params_gamma {
	struct v4l2_isp_params_block_header header;
	__u16 lut[3][MTK_DIP_GAMMA_POINTS];
} __attribute__((aligned(8)));

/*
 * Lens shading grid: gains at the corners of MTK_DIP_LSC_BLOCKS x
 * MTK_DIP_LSC_BLOCKS blocks over the whole input frame (the format of
 * mtk-dip-raw, independent of the crop). With a frame of W x H pixels,
 * bw = W / 32 and bh = H / 32 (integer divisions), corner column i lies at
 * pixel column 2 * bw * i for i < 16 and at W (behind the last pixel) for
 * i = 16, so the last block of a row takes the rest; rows likewise with bh
 * and H. For 4624x3472: columns 0, 288, ..., 4320, 4624 and rows 0, 216, ...,
 * 3240, 3472.
 *
 * Between the corners the driver interpolates with a natural cubic spline
 * along every row and every column of corners and blends the four edges of
 * each block (Coons patch). The gain of a 2x2 cell is the value at its top
 * left pixel and applies to all four pixels with their own Bayer position.
 * Like any cubic spline, the interpolation overshoots next to steep steps
 * between corners.
 */
#define MTK_DIP_LSC_BLOCKS	16
#define MTK_DIP_LSC_POINTS	(MTK_DIP_LSC_BLOCKS + 1)

/**
 * struct mtk_dip_params_lsc - lens shading correction, on the raw data
 * @header: block header, type MTK_DIP_PARAMS_BLOCK_LSC
 * @gain: gain per Bayer position and grid corner, unsigned Q4.12
 *	(1.0 = 0x1000, 0 to just below 16.0), indexed [position][row][column],
 *	rows from the top, columns from the left.
 *
 * Applied after the black level stage and before the white balance; the
 * result is clipped to the range of the raw data. Neutral:
 * MTK_DIP_PARAMS_BLOCK_FL_DISABLE.
 *
 * The block selects the module set of the jobs: while lens shading is
 * enabled (from a job whose params buffer carries an enabled LSC block up to
 * one that carries a disabled one) the jobs run with the shading module, else
 * without it. The two sets serve the same sizes and crops, so the selection
 * may change from one job to the next while streaming; nothing has to be
 * decided at VIDIOC_STREAMON. The grid is relative to the input
 * frame, so it stays valid when the crop changes, and applies to a new frame
 * size until the next LSC block.
 */
struct mtk_dip_params_lsc {
	struct v4l2_isp_params_block_header header;
	__u16 gain[MTK_DIP_BAYER_CHANNELS][MTK_DIP_LSC_POINTS][MTK_DIP_LSC_POINTS];
} __attribute__((aligned(8)));

/*
 * Blocks of the image quality modules: EE, YCNR, DEMOSAIC and NBC2.
 *
 * Their fields are the parameters of the hardware modules, one struct member
 * per parameter (arrays for the points of a curve or the elements of a
 * group), each with the range of its register field. The meaning given for
 * a field is a short description of its role; the tuning of these modules
 * comes from a calibration, not from the descriptions. Fields whose role is
 * not documented have neutral names after their kind and order (offset_a,
 * gain_b, ...) and say so in their description. A value out of range,
 * a reserved byte other than 0 or a violated relation fails the buffer with
 * -EINVAL at QBUF.
 *
 * Curves are given as points and slopes in the units of the registers:
 * points x[] on the input axis (0 to 255 for the full input range), values
 * y[] at the start and at the points, and a slope per segment. The driver
 * takes the slopes as they are; the caller keeps them consistent with the
 * points. The response gain curve of the edge enhancement is the exception,
 * see struct mtk_dip_params_ee.
 */

/**
 * struct mtk_dip_params_ee - edge enhancement, on the luma after the first
 *	noise reduction stage
 * @header: block header, type MTK_DIP_PARAMS_BLOCK_EE
 * @band_offset: offset per frequency band, one of a slope/offset pair with
 *	@band_slope, 0 to 255; role not documented
 * @band_slope: slope per frequency band, belongs to @band_offset, 0 to 255;
 *	role not documented
 * @band_ratio: ratio common to the three bands, belongs to @band_offset and
 *	@band_slope, 0 to 31; role not documented
 * @band_core_th: coring threshold per band, 0 to 255
 * @core_th: coring threshold of the combined response, 0 to 255
 * @band_gain: gain per band, 0 to 31
 * @luma_gain: gain over the luma range, seven points, 0 to 511
 * @radial_gain: gains of the radial weight link, 0 to 255
 * @radial_response_ratio: share of the radial weight in the response, 0 to 31
 * @gain_curve_radial: the gain curve follows the radial weights (0 or 1)
 * @gain_curve_x: inner points of the response gain curve, input; 0 <= x[0] <
 *	x[1] < x[2] < x[3] < 255 (x[0] 0: the curve starts at the first point)
 * @gain_curve_y: output at the inner points, 0 to 1023
 * @gain_curve_y_end: output at input 255, -2048 to 3071; the driver derives the
 *	five segment slopes from the points
 * @gain_curve_aux: a further value of the response gain curve in its output
 *	units, 0 to 1023; role not documented
 * @gain_curve_th_over: overshoot threshold of the gain curve, 0 to 255
 * @gain_curve_th_under: undershoot threshold of the gain curve, 0 to 255
 * @gain_curve_th_min: lower threshold of the gain curve, 0 to 255
 * @response_smooth: smoothing of the response, 0 to 7
 * @overshoot_clip: overshoot clipping strength, 0 to 7
 * @dot_reduction: reduction of isolated dots, 0 to 255
 * @dot_th: threshold of isolated dots, 0 to 255
 * @response_clip: clip of the response, 0 to 255
 * @response_clip_luma_th: luma threshold of the response clip, 0 to 255
 * @response_clip_luma_lower: lower luma bound of the response clip, 0 to 255
 * @response_clip_luma_upper: upper luma bound of the response clip, 0 to 255
 * @master_gain_neg: overall gain of negative responses (darkening), 0 to 255
 * @master_gain_pos: overall gain of positive responses (brightening), 0 to 255
 * @strength_upper: upper bound of the edge strength, 0 to 255
 * @strength_lower: lower bound of the edge strength, 0 to 255
 * @strength_edge_slope: slope of the edge strength over the edge measure, 0 to
 *	255
 * @strength_edge_offset: offset of the edge strength over the edge measure, 0
 *	to 255
 * @strength_response_slope: slope of the edge strength over the response, 0 to
 *	255
 * @strength_response_offset: offset of the edge strength over the response, 0
 *	to 255
 * @luma_limit_diff: luma limit difference, 0 to 255
 * @luma_contrast_level: luma contrast level, 0 to 7
 * @luma_min_sel: luma minimum selection, 0 to 7
 * @luma_max_sel: luma maximum selection, 0 to 7
 * @chroma_contrast_level: chroma contrast level, 0 to 7
 * @chroma_min_sel: chroma minimum selection, 0 to 3
 * @chroma_max_sel: chroma maximum selection, 0 to 3
 * @chroma_boost_limit_upper: upper limit of the chroma boost, 0 to 255
 * @chroma_boost_limit_lower: lower limit of the chroma boost, 0 to 255
 * @chroma_boost_gain: gain of the chroma boost, 0 to 255
 * @chroma_boost_enable: chroma boost on (0 or 1)
 * @chroma_boost_y_const: luma constant of the chroma boost, 0 to 255
 * @chroma_boost_y_offset: luma offset of the chroma boost, 0 to 255
 * @chroma_boost_y_offset_sel: luma offset selection of the chroma boost, 0 to 3
 * @regions_enable: colour regions on (0 or 1)
 * @region_enable: colour region on (0 or 1)
 * @reserved0: must be 0
 * @region_radius_slope2: second slope over the chroma radius, 0 to 1023
 * @region_radius_slope: slope over the chroma radius, 0 to 1023
 * @region_radius_range: width of the region in chroma radius, 0 to 63
 * @reserved1: must be 0
 * @region_angle_slope: slope over the hue angle, 0 to 1023
 * @region_angle_range: width of the region in hue angle, 0 to 63
 * @region_angle_center: centre of the region in hue angle, 0 to 255
 * @region_radius_center: centre of the region in chroma radius, 0 to 255
 * @region_lowpass_gain: low-pass gain in the region, 0 to 63
 * @region_lowpass_enable: low-pass in the region on (0 or 1)
 * @region_edge_enable: edge measure in the region on (0 or 1)
 * @region_edge_slope: slope over the edge measure in the region, 0 to 63
 * @region_edge_th: edge threshold in the region, 0 to 63
 * @region_confidence_gain: confidence gain of the region, 0 to 15
 * @region_gain: sharpening gain in the region, 0 to 255
 * @reserved2: must be 0
 *
 * The response of three high-pass bands is weighted, cored and limited, and
 * added to the luma: positive responses with @master_gain_pos, negative ones
 * with @master_gain_neg. The response gain curve runs through (0, 0), the
 * four inner points and (255, @gain_curve_y_end); the driver programs its
 * five segment slopes as 16 * dy / dx, rounded half away from zero and
 * limited to -128 to 127. The radial weights the module links to are those
 * of MTK_DIP_PARAMS_BLOCK_RADIAL.
 *
 * The block selects the module set of the jobs: while edge enhancement is
 * enabled (from a job whose params buffer carries an enabled EE block up to
 * one that carries a disabled one) the jobs run with the module and its
 * radial weights, else without them. The module costs columns at the tile
 * edges; the widths that VIDIOC_S_SELECTION accepts are those that the tiles
 * serve with and without it, so the selection may change from one job to the
 * next while streaming. With MTK_DIP_PARAMS_BLOCK_FL_DISABLE the module is
 * off.
 */
struct mtk_dip_params_ee {
	struct v4l2_isp_params_block_header header;
	__u8 band_offset[3];
	__u8 band_slope[3];
	__u8 band_ratio;
	__u8 band_core_th[3];
	__u8 core_th;
	__u8 band_gain[3];
	__u16 luma_gain[7];
	__u8 radial_gain[2];
	__u8 radial_response_ratio;
	__u8 gain_curve_radial;
	__u8 gain_curve_x[4];
	__u16 gain_curve_y[4];
	__s16 gain_curve_y_end;
	__u16 gain_curve_aux;
	__u8 gain_curve_th_over;
	__u8 gain_curve_th_under;
	__u8 gain_curve_th_min;
	__u8 response_smooth;
	__u8 overshoot_clip;
	__u8 dot_reduction;
	__u8 dot_th;
	__u8 response_clip;
	__u8 response_clip_luma_th;
	__u8 response_clip_luma_lower;
	__u8 response_clip_luma_upper;
	__u8 master_gain_neg;
	__u8 master_gain_pos;
	__u8 strength_upper;
	__u8 strength_lower;
	__u8 strength_edge_slope;
	__u8 strength_edge_offset;
	__u8 strength_response_slope;
	__u8 strength_response_offset;
	__u8 luma_limit_diff;
	__u8 luma_contrast_level;
	__u8 luma_min_sel;
	__u8 luma_max_sel;
	__u8 chroma_contrast_level;
	__u8 chroma_min_sel;
	__u8 chroma_max_sel;
	__u8 chroma_boost_limit_upper;
	__u8 chroma_boost_limit_lower;
	__u8 chroma_boost_gain;
	__u8 chroma_boost_enable;
	__u8 chroma_boost_y_const;
	__u8 chroma_boost_y_offset;
	__u8 chroma_boost_y_offset_sel;
	__u8 regions_enable;
	__u8 region_enable[3];
	__u8 reserved0[1];
	__u16 region_radius_slope2[3];
	__u16 region_radius_slope[3];
	__u8 region_radius_range[3];
	__u8 reserved1[1];
	__u16 region_angle_slope[3];
	__u8 region_angle_range[3];
	__u8 region_angle_center[3];
	__u8 region_radius_center[3];
	__u8 region_lowpass_gain[3];
	__u8 region_lowpass_enable[3];
	__u8 region_edge_enable[3];
	__u8 region_edge_slope[3];
	__u8 region_edge_th[3];
	__u8 region_confidence_gain[3];
	__u8 region_gain[3];
	__u8 reserved2[6];
} __attribute__((aligned(8)));

/**
 * struct mtk_dip_params_ycnr - luma/chroma noise reduction, first stage, on
 *	the YUV data after demosaicing
 * @header: block header, type MTK_DIP_PARAMS_BLOCK_YCNR
 * @chroma_enable: chroma noise reduction on (0 or 1)
 * @luma_enable: luma noise reduction on (0 or 1); one of the two must be on
 * @luma_filter_sel: filter selection per luma level, 0-1, 0-1, 0-3, 0-3
 * @radial_link: strength follows the radial weights (0 or 1)
 * @filter_mode: filter mode, 0 to 3
 * @chroma_smooth_edge: chroma smoothing at edges, 0 to 3
 * @chroma_filter: chroma filter selection (0 or 1)
 * @chroma_smooth_edge_th: edge threshold of the chroma smoothing, 0 to 7
 * @center_gain_lo_th: lower threshold of the centre pixel gain, 0 to 31
 * @center_gain_hi_th: upper threshold of the centre pixel gain, 0 to 31
 * @luma_slope_v_th: vertical luma slope threshold, 0 to 255
 * @luma_slope_h_th: horizontal luma slope threshold, 0 to 255
 * @luma_vert_act_th: vertical luma activity threshold, 0 to 31
 * @luma_aux_gain_th: gain threshold of the luma part, 0 to 31; role not
 *	documented
 * @luma_vert_sigma: vertical luma sigma, 0 to 255
 * @luma_scale_x: luma strength curve over the luma level: inner points, 0 to
 *	255
 * @luma_scale_y: luma strength curve: values at 0, the inner points, 0 to 31
 * @luma_scale_slope: luma strength curve: slope per segment, 0 to 31
 * @chroma_scale_x: chroma strength curve: inner points, 0 to 255
 * @chroma_scale_y: chroma strength curve: values, 0 to 31
 * @chroma_scale_slope: chroma strength curve: slopes, 0 to 31
 * @luma_act_y: luma activity curve: values, 0 to 255
 * @luma_act_slope: luma activity curve: slopes, 0 to 63
 * @luma_act_x: luma activity curve: inner points, 0 to 255
 * @radial_x: radial weight curve: inner points, 0 to 255
 * @radial_y: radial weight curve: values, 0 to 63
 * @radial_slope: radial weight curve: slopes, 0 to 63
 * @radial_chroma_gain: radial gain of the chroma part, 0 to 15
 * @radial_scale_gain: radial gain of the strength, 0 to 7
 * @luma_range_v: vertical luma range per level and step, 0 to 255
 * @chroma_aux_gain_th: gain threshold of the chroma part, 0 to 31; role not
 *	documented
 * @chroma_luma_diff_th: luma difference threshold of the chroma filter, 0 to
 *	255
 * @luma_range_h: horizontal luma range per level and step, 0 to 255
 * @act_radial_gain: radial gain of the activity, 0 to 15
 * @act_diff_hi_th: upper threshold of the activity difference, 0 to 31
 * @act_diff_gain: gain of the activity difference, 0 to 15
 * @act_diff_lo_th: lower threshold of the activity difference, 0 to 31
 * @chroma_range_v: vertical chroma range per step, 0 to 255
 * @luma_act_center_offset: offset of the luma activity of the centre, 0 to 63
 * @luma_act_center_gain: gain of the luma activity of the centre, 0 to 63
 * @luma_act_center_th: threshold of the luma activity of the centre, 0 to 63
 * @chroma_range_h: horizontal chroma range per step, 0 to 255
 * @luma_range_ratio_th: range ratio threshold per luma level, 0 to 15
 * @luma_range_ratio_slope: range ratio slope per luma level, 0 to 3
 * @luma_hf_weight: high frequency weight per luma level, 0 to 31
 * @luma_hf_core_th: coring threshold of the luma high frequencies, 0 to 255
 * @luma_hf_core_slope: coring slope of the luma high frequencies, 0 to 3
 * @luma_hf_clip: clip of the luma high frequencies, 0 to 255
 * @hf_balance_mode: balance mode of the high frequencies, 0 to 3
 * @luma_hf_act_x: high frequency activity curve: inner points, 0 to 255
 * @luma_hf_act_y: high frequency activity curve: values, 0 to 127
 * @luma_hf_act_slope: high frequency activity curve: slopes, 0 to 63
 * @act_blend_base: base of the chroma activity blend, 0 to 127
 * @chroma_dither: dither of U and V, 0 to 31
 * @center_gain_lo_th_lpf: lower centre gain threshold, low-pass part, 0 to 31
 * @center_gain_hi_th_lpf: upper centre gain threshold, low-pass part, 0 to 31
 * @luma_hf_luma_x: high frequency luma curve: inner points, 0 to 255
 * @luma_hf_luma_y: high frequency luma curve: values, 0 to 127
 * @luma_hf_luma_slope: high frequency luma curve: slopes, 0 to 31
 * @reserved: must be 0
 *
 * The module filters luma and chroma within ranges per luma level and step,
 * with strength curves over the luma level and the local activity and the
 * radial weights of MTK_DIP_PARAMS_BLOCK_RADIAL. It works without a local
 * tone map:
 * the stage does not take a gain map from the statistics, and its lookup
 * table is off. At least one of @luma_enable and @chroma_enable has to be
 * set; the module stays in the pipeline with its tile margins in any case.
 * With MTK_DIP_PARAMS_BLOCK_FL_DISABLE, and before the first YCNR block, the
 * driver uses its neutral settings: the chroma part on (for the tile
 * margins), all other fields 0.
 */
struct mtk_dip_params_ycnr {
	struct v4l2_isp_params_block_header header;
	__u8 chroma_enable;
	__u8 luma_enable;
	__u8 luma_filter_sel[4];
	__u8 radial_link;
	__u8 filter_mode;
	__u8 chroma_smooth_edge;
	__u8 chroma_filter;
	__u8 chroma_smooth_edge_th;
	__u8 center_gain_lo_th;
	__u8 center_gain_hi_th;
	__u8 luma_slope_v_th;
	__u8 luma_slope_h_th;
	__u8 luma_vert_act_th;
	__u8 luma_aux_gain_th;
	__u8 luma_vert_sigma;
	__u8 luma_scale_x[8];
	__u8 luma_scale_y[9];
	__u8 luma_scale_slope[9];
	__u8 chroma_scale_x[3];
	__u8 chroma_scale_y[4];
	__u8 chroma_scale_slope[4];
	__u8 luma_act_y[4];
	__u8 luma_act_slope[4];
	__u8 luma_act_x[3];
	__u8 radial_x[3];
	__u8 radial_y[4];
	__u8 radial_slope[4];
	__u8 radial_chroma_gain;
	__u8 radial_scale_gain;
	__u8 luma_range_v[4][4];
	__u8 chroma_aux_gain_th;
	__u8 chroma_luma_diff_th;
	__u8 luma_range_h[4][4];
	__u8 act_radial_gain;
	__u8 act_diff_hi_th;
	__u8 act_diff_gain;
	__u8 act_diff_lo_th;
	__u8 chroma_range_v[4];
	__u8 luma_act_center_offset;
	__u8 luma_act_center_gain;
	__u8 luma_act_center_th;
	__u8 chroma_range_h[4];
	__u8 luma_range_ratio_th[4];
	__u8 luma_range_ratio_slope[4];
	__u8 luma_hf_weight[4];
	__u8 luma_hf_core_th;
	__u8 luma_hf_core_slope;
	__u8 luma_hf_clip;
	__u8 hf_balance_mode;
	__u8 luma_hf_act_x[4];
	__u8 luma_hf_act_y[5];
	__u8 luma_hf_act_slope[5];
	__u8 act_blend_base;
	__u8 chroma_dither[2];
	__u8 center_gain_lo_th_lpf;
	__u8 center_gain_hi_th_lpf;
	__u8 luma_hf_luma_x[2];
	__u8 luma_hf_luma_y[3];
	__u8 luma_hf_luma_slope[3];
	__u8 reserved[5];
} __attribute__((aligned(8)));

/**
 * struct mtk_dip_params_demosaic - demosaicing, raw Bayer to RGB
 * @header: block header, type MTK_DIP_PARAMS_BLOCK_DEMOSAIC
 * @color_diff_gain_slope: colour difference gain slope, 0 to 15
 * @color_diff_gain_offset: colour difference gain offset, 0 to 255
 * @color_diff_gain_ratio: colour difference gain ratio, 0 to 31
 * @color_diff_kernel: colour difference kernel selection (0 or 1)
 * @offset_a: an offset, 0 to 255; role not documented
 * @level_slope: slope per detail level, 0 to 15
 * @level_offset: offset per detail level, 0 to 255
 * @color_diff_slope_luma: colour difference slope, luma part, 0 to 31
 * @color_diff_slope_chroma: colour difference slope, chroma part, 0 to 31
 * @offset_b: a second offset, 0 to 255; role not documented
 * @luma_weight: weight over the luma range, six points, 0 to 511
 * @radial_enable: radial weights on (0 or 1)
 * @radial_ratio2: a second radial ratio, 0 to 31; role not documented
 * @radial_gain: radial gains, 0 to 255
 * @core_th: coring thresholds, 0 to 255
 * @gain_a: a group of gains, 0-31, 0-31, 0-63; role not documented
 * @gain_b: a second group of gains, 0 to 31; role not documented
 * @noise_strength: noise strength per level, 0 to 31
 * @crosstalk_slope: crosstalk slope, 0 to 15
 * @crosstalk_offset: crosstalk offset, 0 to 255
 * @crosstalk_ratio: crosstalk ratio, 0 to 31
 * @noise_gain: noise gain, 0 to 15
 * @noise_slope: noise slope, 0 to 15
 * @noise_offset: noise offset per level, 0 to 255
 * @ratio_a: a ratio, 0 to 15; role not documented
 * @band_gain: gain per frequency band, 0 to 31
 * @band_strength: strength of the frequency bands, 0 to 31
 * @band_upper: upper bound per band, 0 to 255
 * @band_lower: lower bound per band, 0 to 255
 * @band_slope_left: left slope of the bands, 0 to 15
 * @band_slope_right: right slope of the bands, 0 to 15
 * @clip_th: clip threshold, 0 to 255
 * @undershoot_th: undershoot threshold, 0 to 255
 * @overshoot_th: overshoot threshold, 0 to 255
 * @ratio_b: a second ratio, 0 to 31; role not documented
 * @radial_ratio: radial ratio, 0 to 31
 * @mode_a: a mode bit (0 or 1); role not documented
 * @bound_upper: upper bound of a group with @bound_lower and @bound_blend, 0 to
 *	255; role not documented
 * @bound_lower: lower bound of the group of @bound_upper, 0 to 255
 * @ratio_c: a third ratio, 0 to 15; role not documented
 * @gain_c: a further gain, 0 to 63; role not documented
 * @clip_flag: a clipping bit (0 or 1); role not documented
 * @bound_blend: blend weight of the group of @bound_upper, 0 to 31
 * @threshold_a: a threshold, 0 to 15; role not documented
 * @threshold_b: a second threshold, 0 to 15; role not documented
 * @gain_pos: gain of positive values, the counterpart of @gain_neg, 0 to 31;
 *	role not documented
 * @gain_neg: gain of negative values, 0 to 31
 * @reserved: must be 0
 *
 * The module interpolates the missing colours along the detected direction,
 * with crosstalk and noise handling and a detail gain per band. It cannot be
 * bypassed: with MTK_DIP_PARAMS_BLOCK_FL_DISABLE, and before the first
 * DEMOSAIC block, the driver sets all fields to 0.
 */
struct mtk_dip_params_demosaic {
	struct v4l2_isp_params_block_header header;
	__u8 color_diff_gain_slope;
	__u8 color_diff_gain_offset;
	__u8 color_diff_gain_ratio;
	__u8 color_diff_kernel;
	__u8 offset_a;
	__u8 level_slope[3];
	__u8 level_offset[3];
	__u8 color_diff_slope_luma;
	__u8 color_diff_slope_chroma;
	__u8 offset_b;
	__u16 luma_weight[6];
	__u8 radial_enable;
	__u8 radial_ratio2;
	__u8 radial_gain[2];
	__u8 core_th[2];
	__u8 gain_a[3];
	__u8 gain_b[2];
	__u8 noise_strength[3];
	__u8 crosstalk_slope;
	__u8 crosstalk_offset;
	__u8 crosstalk_ratio;
	__u8 noise_gain;
	__u8 noise_slope;
	__u8 noise_offset[3];
	__u8 ratio_a;
	__u8 band_gain[3];
	__u8 band_strength;
	__u8 band_upper[3];
	__u8 band_lower[3];
	__u8 band_slope_left;
	__u8 band_slope_right;
	__u8 clip_th;
	__u8 undershoot_th;
	__u8 overshoot_th;
	__u8 ratio_b;
	__u8 radial_ratio;
	__u8 mode_a;
	__u8 bound_upper;
	__u8 bound_lower;
	__u8 ratio_c;
	__u8 gain_c;
	__u8 clip_flag;
	__u8 bound_blend;
	__u8 threshold_a;
	__u8 threshold_b;
	__u8 gain_pos;
	__u8 gain_neg;
	__u8 reserved[3];
} __attribute__((aligned(8)));

/**
 * struct mtk_dip_params_nbc2 - luma/chroma noise reduction, second stage, on
 *	the YUV data behind the first stage (and the edge enhancement)
 * @header: block header, type MTK_DIP_PARAMS_BLOCK_NBC2
 * @luma_enable: luma part on (0 or 1); the chroma part is always on
 * @scale_mode: filter scale, 2 or 3 (0 and 1 are refused)
 * @chroma_ref_luma: vertical chroma filtering follows the luma (0 or 1)
 * @radial_link: strength follows the radial weights (0 or 1)
 * @chroma_smooth_edge: chroma smoothing at edges, 0 to 3
 * @chroma_filter: chroma filter selection (0 or 1)
 * @chroma_threshold: threshold of the chroma part, 0 to 15; role not documented
 * @luma_scale_x: strength curve over the luma level: inner points, 0 to 255
 * @luma_scale_y: strength curve: values at 0 and at the inner points, 0 to 31
 * @luma_scale_slope: strength curve: slope per segment, 0 to 31
 * @radial_x: radial weight curve: inner points, 0 to 255
 * @radial_y: radial weight curve: values, 0 to 63
 * @radial_slope: radial weight curve: slopes, 0 to 63
 * @aux_v_gain: vertical gain of a group with @aux_gain_th and @aux_th, 0 to 15;
 *	role not documented
 * @aux_gain_th: gain threshold of the group of @aux_v_gain, 0 to 31
 * @chroma_luma_diff_th: luma difference threshold of the chroma filter, 0 to
 *	255
 * @chroma_mode: chroma mode (0 or 1)
 * @aux_th: thresholds of the group of @aux_v_gain, 0 to 255
 * @radial_chroma_gain: radial gain of the chroma part, 0 to 15
 * @luma_weight: luma weight, 0 to 15
 * @core_th: coring threshold, 0 to 31
 * @core_slope: coring slope, 0 to 7
 * @diff_th: difference threshold, 0 to 31
 * @diff_slope: difference slope, 0 to 7
 * @class_th: threshold per classification stage, 0-255, 0-63, 0-255
 * @class_slope: slope per classification stage, 0 to 7
 * @class_level: level per classification stage, 0 to 31
 * @class_offset: offset per classification stage, 0-255, 0-63, 0-255
 * @variance: variance setting, 0 to 7
 * @strength: strength per level, 0 to 63
 * @act_blend_base: base of the chroma activity blend, 0 to 127
 * @chroma_dither: dither of U and V, 0 to 31
 *
 * The second stage filters mainly the chroma (colour noise in flat areas),
 * with a strength curve over the luma level and the radial weights of
 * MTK_DIP_PARAMS_BLOCK_RADIAL. It stays in the pipeline with its tile margins
 * in any case; the
 * margins the driver uses are those of the chroma part with scale 2 or 3,
 * which is why the chroma part cannot be switched off and smaller scales are
 * refused. With MTK_DIP_PARAMS_BLOCK_FL_DISABLE, and before the first NBC2
 * block, the driver uses its neutral settings: scale_mode 3, all other
 * fields 0.
 */
struct mtk_dip_params_nbc2 {
	struct v4l2_isp_params_block_header header;
	__u8 luma_enable;
	__u8 scale_mode;
	__u8 chroma_ref_luma;
	__u8 radial_link;
	__u8 chroma_smooth_edge;
	__u8 chroma_filter;
	__u8 chroma_threshold;
	__u8 luma_scale_x[3];
	__u8 luma_scale_y[4];
	__u8 luma_scale_slope[4];
	__u8 radial_x[3];
	__u8 radial_y[4];
	__u8 radial_slope[4];
	__u8 aux_v_gain;
	__u8 aux_gain_th;
	__u8 chroma_luma_diff_th;
	__u8 chroma_mode;
	__u8 aux_th[4];
	__u8 radial_chroma_gain;
	__u8 luma_weight;
	__u8 core_th;
	__u8 core_slope;
	__u8 diff_th;
	__u8 diff_slope;
	__u8 class_th[3];
	__u8 class_slope[3];
	__u8 class_level[3];
	__u8 class_offset[3];
	__u8 variance;
	__u8 strength[5];
	__u8 act_blend_base;
	__u8 chroma_dither[2];
} __attribute__((aligned(8)));

/*
 * Temporal noise reduction: each job blends the image of its frame with the
 * output of the job before (the reference), pixel by pixel, more where the
 * two agree and less where they differ (motion, scene change). The driver
 * keeps the reference itself, user space never feeds a frame back:
 *  - a job with a buffer on mtk-dip-main writes it, and that buffer becomes
 *    the reference of the next job: the driver returns it to user space only
 *    when the next job has read it, or after a short time without a next job
 *    (module parameter nr3d_hold_ms, then the chain starts again). The
 *    buffers of mtk-dip-main come back one job later while the module is on.
 *  - a job without the main output writes its full image into a buffer of
 *    the driver (two of them, each the size of an NV12 frame of the
 *    processed area, taken from the camera memory region while the module is
 *    on).
 * The scaled output is made from the blended image as well.
 *
 * When the memory region cannot provide these buffers, or a job does not get
 * its other buffers next to them, the driver frees them and the module stays
 * off until the next VIDIOC_STREAMON of mtk-dip-raw or until a block with
 * MTK_DIP_PARAMS_BLOCK_FL_DISABLE and a new block switch it on again. The
 * jobs run and write their outputs as without the block; the kernel log
 * reports it once each time.
 *
 * A job runs without the module, and its output starts a new reference
 * chain, when there is no valid reference: the first job after the module
 * was switched on, after VIDIOC_STREAMON of mtk-dip-raw, after
 * VIDIOC_STREAMOFF of any node, after a job that failed, after a system
 * sleep, after a pause longer than nr3d_hold_ms, and with
 * MTK_DIP_NR3D_FL_RESTART (scene cut). The module also stays off in a job
 * whose geometry the tile computation serves only without it; the job runs
 * and writes its outputs as without the block. A params buffer without
 * this block leaves the module as it is, with the values of the last block;
 * MTK_DIP_PARAMS_BLOCK_FL_DISABLE switches it off and drops the reference.
 *
 * Global motion: motion_x and motion_y give the position of a pixel of this
 * frame in the reference minus its position in this frame (a scene that
 * moved 10 pixels to the right between the frames gives motion_x = -10).
 * The module blends where both frames overlap and passes the frame through
 * elsewhere. 0 for a camera at rest.
 *
 * The fields below are the parameters of the module, one struct member per
 * register field; the meaning given for a field is a short description of
 * its role, the values come from a calibration. Curves have
 * MTK_DIP_NR3D_CURVE_POINTS values over an input range the hardware fixes.
 */
#define MTK_DIP_NR3D_CURVE_POINTS	8
/* largest global motion in pixels, either direction; even values only */
#define MTK_DIP_NR3D_MOTION_MAX		64

/* this job starts a new sequence: no reference (scene cut) */
#define MTK_DIP_NR3D_FL_RESTART		(1U << 0)

/**
 * struct mtk_dip_params_nr3d - temporal noise reduction, on the YUV image
 *	behind the second noise reduction stage
 * @header: block header, type MTK_DIP_PARAMS_BLOCK_NR3D
 * @flags: MTK_DIP_NR3D_FL_*; they apply to the job of this buffer only
 * @luma_enable: blend the luma (0 or 1)
 * @chroma_enable: blend the chroma (0 or 1); one of the two must be on
 * @radial_link: the strength follows the radial weights (0 or 1)
 * @motion_x: global motion, horizontal, even, -MTK_DIP_NR3D_MOTION_MAX to
 *	MTK_DIP_NR3D_MOTION_MAX
 * @motion_y: global motion, vertical, likewise
 * @strength_max: upper bound of the blend strength, 0 to 63
 * @blend_ratio: ratios of four measures in the blend decision (block luma,
 *	detail, texture, motion), 0 to 31; roles not documented beyond this
 * @noise_level: expected noise level, 0 to 63; usually raised with the
 *	sensitivity
 * @noise_slope: a second noise setting, 0 to 63; role not documented
 * @small_diff_th: threshold of small frame differences, 0 to 63
 * @motion_penalty_th: threshold of the motion penalty, 0 to 63
 * @motion_penalty_weight: weight of the motion penalty, 0 to 15
 * @block_diff_th: a threshold of block differences, 0 to 15; role not
 *	documented
 * @block_luma: block luma curve, 0 to 63 per point; role not documented
 * @block_chroma: block chroma curve, 0 to 63; role not documented
 * @texture_level: texture level curve, 0 to 63
 * @detail_base: detail base curve, 0 to 63; role not documented
 * @texture_base: texture base curve, 0 to 63; role not documented
 * @motion_curve: motion curve, 0 to 63
 * @variance_weight: variance weight curve, 0 to 63
 * @smooth_weight: smoothing weight curve, 0 to 63
 * @diff_level: difference level curve, nine points, 0 to 63; role not
 *	documented
 * @pattern_mode: mode of the pattern counters, 0 to 3; role not documented
 * @pattern_cycle: cycle lengths of four counters the driver steps once per
 *	job, 0 to 63 (a counter runs from 0 to its value)
 * @pattern_texture_th: four texture thresholds of the pattern part, 0 to 63
 * @pattern_texture_offset: offset of @pattern_texture_th, 0 to 63
 * @reserved: must be 0
 */
struct mtk_dip_params_nr3d {
	struct v4l2_isp_params_block_header header;
	__u8 flags;
	__u8 luma_enable;
	__u8 chroma_enable;
	__u8 radial_link;
	__s16 motion_x;
	__s16 motion_y;
	__u8 strength_max;
	__u8 blend_ratio[4];
	__u8 noise_level;
	__u8 noise_slope;
	__u8 small_diff_th;
	__u8 motion_penalty_th;
	__u8 motion_penalty_weight;
	__u8 block_diff_th;
	__u8 block_luma[MTK_DIP_NR3D_CURVE_POINTS];
	__u8 block_chroma[MTK_DIP_NR3D_CURVE_POINTS];
	__u8 texture_level[MTK_DIP_NR3D_CURVE_POINTS];
	__u8 detail_base[MTK_DIP_NR3D_CURVE_POINTS];
	__u8 texture_base[MTK_DIP_NR3D_CURVE_POINTS];
	__u8 motion_curve[MTK_DIP_NR3D_CURVE_POINTS];
	__u8 variance_weight[MTK_DIP_NR3D_CURVE_POINTS];
	__u8 smooth_weight[MTK_DIP_NR3D_CURVE_POINTS];
	__u8 diff_level[MTK_DIP_NR3D_CURVE_POINTS + 1];
	__u8 pattern_mode;
	__u8 pattern_cycle[4];
	__u8 pattern_texture_th[4];
	__u8 pattern_texture_offset;
	__u8 reserved[2];
} __attribute__((aligned(8)));

/*
 * Blend (MTK_DIP_PARAMS_BLOCK_MIX), jobs with mtk-dip-base and
 * mtk-dip-weight only; other jobs ignore it. Each output pixel is
 *
 *	out = (in * (256 - w) + base * w) / 256
 *
 * with in the pixel of mtk-dip-raw, base the pixel of mtk-dip-base and w the
 * weight of the base image, taken from the weight map value m of the pixel:
 *
 *	w = weight_max                                   if m < map_start
 *	w = clamp(slope * (map_end - m), weight_min, weight_max)  otherwise
 *
 * The weight falls with the map value: from map_end on the input wins (up to
 * weight_min), below map_start the base image (weight_max). Chroma uses the
 * map at the chroma sample. With weight_to_base 0 the roles of input and base
 * swap (w is the weight of the input). A slope of 0 gives weight_min
 * everywhere at or above map_start.
 *
 * With luma_enable and chroma_enable 0 nothing is blended and the output
 * takes the input, or the base image where luma_pass_base and
 * chroma_pass_base are set.
 *
 * A params buffer without this block leaves the last values; before the
 * first block the driver uses weight_to_base 1, weight 0 to 255, slope 1,
 * map_start 0, map_end 255: the map value is the weight of the input
 * (0 base image, 255 input). MTK_DIP_PARAMS_BLOCK_FL_DISABLE switches the
 * blend off: the job filters the input alone; it still takes and returns a
 * buffer of mtk-dip-base and of mtk-dip-weight, unread.
 */

/**
 * struct mtk_dip_params_mix - blend of the input with the base image
 * @header: block header, type MTK_DIP_PARAMS_BLOCK_MIX
 * @luma_enable: blend the luma (0 or 1)
 * @chroma_enable: blend the chroma (0 or 1); both on or both off
 * @weight_to_base: the ramp gives the weight of the base image (1) or of the
 *	input (0)
 * @luma_pass_base: luma blend off: output the base luma (1) or the input
 *	luma (0)
 * @chroma_pass_base: chroma blend off: output the base chroma (1) or the
 *	input chroma (0)
 * @weight_min: lowest weight, 0 to 255, at most @weight_max
 * @weight_max: highest weight, 0 to 255; also the weight below @map_start
 * @slope: weight steps per map step, 0 to 255
 * @map_start: map values below it give @weight_max, 0 to 255
 * @map_end: map value at which the ramp reaches weight 0, 0 to 255
 * @reserved: must be 0
 */
struct mtk_dip_params_mix {
	struct v4l2_isp_params_block_header header;
	__u8 luma_enable;
	__u8 chroma_enable;
	__u8 weight_to_base;
	__u8 luma_pass_base;
	__u8 chroma_pass_base;
	__u8 weight_min;
	__u8 weight_max;
	__u8 slope;
	__u8 map_start;
	__u8 map_end;
	__u8 reserved[6];
} __attribute__((aligned(8)));

/**
 * struct mtk_dip_params_radial - radial weights of the filter modules
 * @header: block header, type MTK_DIP_PARAMS_BLOCK_RADIAL
 * @center_x: horizontal position of the centre, in pixels of the raw frame,
 *	0 to 16383
 * @center_y: vertical position of the centre, 0 to 16383
 * @radius: distances from the centre of the inner points of the weight
 *	curve, in pixels, 0 to 16383
 * @weight: weights at the centre, at the three radii and at the farthest
 *	corner, 0 to 255
 * @reserved: must be 0
 * @slope: slopes of the four segments of the weight curve, weight steps per
 *	pixel in units of 1/65536, 0 to 16777215
 *
 * The edge enhancement, both luma/chroma noise reduction stages, the
 * demosaicing, the raw noise reduction and the temporal noise reduction can
 * scale their strength with the distance of a pixel from a centre: each of
 * them has a radial link switch and a curve of its own over the weight that
 * this block defines. The weight runs from @weight[0] at the centre along
 * straight segments through @weight[1] to @weight[3] at @radius[0] to
 * @radius[2], to @weight[4] at the farthest corner of the frame. The module
 * does not compute the slopes from the points: @slope[0] to @slope[3] are
 * those of the four segments ((w[i + 1] - w[i]) * 65536 / (r[i + 1] - r[i])
 * with r = 0, @radius and the distance of the farthest corner), and the
 * distance of the corner follows the octagon approximation of the hardware;
 * the caller provides them. The same weights serve all modules and all jobs
 * until the next block.
 *
 * With MTK_DIP_PARAMS_BLOCK_FL_DISABLE, and before the first RADIAL block,
 * all fields are 0: the weight is 0 everywhere, as at the centre.
 */
struct mtk_dip_params_radial {
	struct v4l2_isp_params_block_header header;
	__u16 center_x;
	__u16 center_y;
	__u16 radius[3];
	__u8 weight[5];
	__u8 reserved;
	__u32 slope[4];
} __attribute__((aligned(8)));

/**
 * struct mtk_dip_params_rnr - raw noise reduction
 * @header: block header, type MTK_DIP_PARAMS_BLOCK_RNR
 * @ratio: filter ratio, 0 to 511
 * @noise_offset: constant part of the noise model per colour (red, green,
 *	blue), 0 to 511
 * @noise_slope: part of the noise model that rises with the level, per
 *	colour (red, green, blue), 0 to 511
 * @radial_gain: gains of the radial link, 0 to 1023
 * @norm_gain: colour gains the module normalises the raw values with, for
 *	blue, green and red of the colour order of the hardware (a BGGR frame;
 *	the driver does not move them with the Bayer order), unsigned 13 bit,
 *	512 = 1.0
 * @norm_offset: offsets of the normalisation, same order, -2048 to 2047
 * @black_gain: gain of the black level stage the normalisation assumes, 0 to
 *	8191, 512 = 1.0
 * @eps: small constant of the filter weights, 0 to 255
 * @range: range limits of the four filter steps, 0 to 63
 * @median_ratio: share of the median filter, 0 to 127
 * @profile_ratio: share of the profile filter, 0 to 127
 * @radius: filter radius, 0 to 7
 * @th_right: a threshold, 0 to 63; role not documented
 * @th_left: the counterpart of @th_right, 0 to 63
 * @flat_mode: mode for flat areas (0 or 1)
 * @gradient_ratio: share of the gradient term, 0 to 63
 * @lowpass_ratio: share of the low-pass term, 0 to 255
 * @intensity_offset: offset of the intensity term, 0 to 255
 * @radial_enable: strength follows the radial weights (0 or 1)
 * @level_th: thresholds of two level stages, 0 to 255; role not documented
 * @level_slope: slopes of the two level stages, 0 to 255
 * @reserved: must be 0
 *
 * The module filters the raw Bayer values before the demosaicing, with a
 * noise model per colour. It stays in the pipeline in any case. With
 * MTK_DIP_PARAMS_BLOCK_FL_DISABLE, and before the first RNR block, the driver
 * sets the normalisation gains and @black_gain to 512 and all other fields
 * to 0.
 */
struct mtk_dip_params_rnr {
	struct v4l2_isp_params_block_header header;
	__u16 ratio;
	__u16 noise_offset[3];
	__u16 noise_slope[3];
	__u16 radial_gain[2];
	__u16 norm_gain[3];
	__s16 norm_offset[3];
	__u16 black_gain;
	__u8 eps;
	__u8 range[4];
	__u8 median_ratio;
	__u8 profile_ratio;
	__u8 radius;
	__u8 th_right;
	__u8 th_left;
	__u8 flat_mode;
	__u8 gradient_ratio;
	__u8 lowpass_ratio;
	__u8 intensity_offset;
	__u8 radial_enable;
	__u8 level_th[2];
	__u8 level_slope[2];
	__u8 reserved[5];
} __attribute__((aligned(8)));

#endif /* _UAPI_MTK_DIP_MT6785_H */
