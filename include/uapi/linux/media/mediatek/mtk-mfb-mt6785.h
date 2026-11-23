/* SPDX-License-Identifier: ((GPL-2.0+ WITH Linux-syscall-note) OR MIT) */
/*
 * Userspace interface of the MT6785 multi-frame blending block ("MFB"): a
 * memory-to-memory block that blends a reference image into a base image
 * under control of a per-pixel weight map, and writes the blended image and
 * the updated weight map. A multi-frame noise reduction runs it once per
 * additional frame: the first blend starts the weight map, each further
 * blend reads the map of the blend before.
 *
 * Device model (media controller, driver and model "mtk-mfb", one processing
 * entity "mtk-mfb", no subdev controls):
 *
 *   video node           direction      queue type              format
 *   mtk-mfb-base         to the block   VIDEO_OUTPUT_MPLANE     NV12
 *   mtk-mfb-ref          to the block   VIDEO_OUTPUT_MPLANE     NV12
 *   mtk-mfb-weight-in    to the block   VIDEO_OUTPUT_MPLANE     GREY
 *   mtk-mfb-params       to the block   META_OUTPUT             V4L2_META_FMT_MTK_MFB_PARAMS
 *   mtk-mfb-out          from the block VIDEO_CAPTURE_MPLANE    NV12
 *   mtk-mfb-weight-out   from the block VIDEO_CAPTURE_MPLANE    GREY
 *
 * NV12 is one buffer per image (num_planes 1): the Y plane, then the UV
 * plane at bytesperline * height. GREY is the weight map, one byte per
 * pixel. All images of a blend have the same width and height; each node
 * keeps its own bytesperline (a multiple of 16, at least the width, at most
 * 9248). The width is a multiple of 4 from 64 to 4624, the height even from
 * 32 to 3472. ENUM_FRAMESIZES reports these steps and limits. STREAMON of a
 * node whose size differs from mtk-mfb-base fails with -EPIPE.
 *
 * A blend (job) runs when mtk-mfb-base, mtk-mfb-ref, mtk-mfb-out and
 * mtk-mfb-weight-out stream and hold a queued buffer. mtk-mfb-weight-in
 * takes part while it streams: a job then also needs a buffer there, and
 * the block continues that weight map; while it does not stream, the block
 * starts a new map. Start the series with mtk-mfb-weight-in stopped, and
 * stream it from the second blend on. Buffers are taken in queue order. A
 * node that starts or stops streaming while the others stream joins or
 * leaves from the next job on.
 *
 * mtk-mfb-params never holds a job back: if it streams with its link
 * enabled and a buffer is queued, the first one belongs to the next job;
 * without one, or with one that omits a block, the values of the previous
 * job stay in effect. Before the first params buffer all tuning words are
 * 0. Sequence numbers and timestamps of the capture buffers are copied from
 * the base buffer. A job that fails returns all its buffers with
 * V4L2_BUF_FLAG_ERROR. STREAMOFF on any node waits for the running job and
 * returns the buffers of that node.
 *
 * Chaining: export the buffers of mtk-mfb-out and mtk-mfb-weight-out
 * (VIDEO_EXPBUF) and queue them as DMABUFs on mtk-mfb-base and
 * mtk-mfb-weight-in of the next blend. A blend must not read and write the
 * same buffer: the tiles of a frame read columns next to the ones the tile
 * before has written. Two output buffers per kind, used in turn, serve a
 * series of any length.
 *
 * Buffers must be physically contiguous and below 4 GiB (the block works
 * without IOMMU); the nodes allocate such buffers from the camera memory
 * region of the board and export them, and import DMABUFs that qualify. A
 * data_offset is refused with -EINVAL at QBUF.
 *
 * Opening a node does not power the block; it is powered for the jobs and
 * goes off shortly after the last one.
 */
#ifndef _UAPI_MTK_MFB_MT6785_H
#define _UAPI_MTK_MFB_MT6785_H

#include <linux/types.h>
#include <linux/videodev2.h>
#include <linux/media/v4l2-isp.h>

#ifndef V4L2_META_FMT_MTK_MFB_PARAMS
/* Parameters of mtk-mfb-params */
#define V4L2_META_FMT_MTK_MFB_PARAMS	v4l2_fourcc('M', 'F', 'B', 'P')
#endif

#define MTK_MFB_PARAMS_VERSION_V1	1

/* Size of a params buffer: the header and one block of each type fit */
#define MTK_MFB_PARAMS_MAX_SIZE		256

/**
 * enum mtk_mfb_params_block_type - type of a block in the params buffer
 * @MTK_MFB_PARAMS_BLOCK_BLEND: the tuning words a camera sets per
 *	sensitivity
 * @MTK_MFB_PARAMS_BLOCK_EXTRA: the other tuning words of the block, which
 *	camera software usually leaves at 0
 *
 * The buffer is a struct v4l2_isp_params_buffer (version
 * MTK_MFB_PARAMS_VERSION_V1) followed by blocks, each starting with a
 * struct v4l2_isp_params_block_header. Unknown types, bad sizes and
 * reserved bits that are set fail the buffer with -EINVAL at QBUF.
 * V4L2_ISP_PARAMS_FL_BLOCK_DISABLE sets the words of the block to 0 (the
 * payload may then be omitted).
 *
 * The fields of the tuning words are not documented; the blocks carry them
 * as whole 32-bit words under the names of the vendor register map.
 */
enum mtk_mfb_params_block_type {
	MTK_MFB_PARAMS_BLOCK_BLEND = 0,
	MTK_MFB_PARAMS_BLOCK_EXTRA = 1,
};

#define MTK_MFB_MBD_CON_WORDS		11
#define MTK_MFB_LL_CON_EXTRA_WORDS	8

/**
 * struct mtk_mfb_params_blend - per-sensitivity tuning words
 * @header: block header, type MTK_MFB_PARAMS_BLOCK_BLEND
 * @ll_con1: word LL_CON1
 * @ll_con2: word LL_CON2
 * @mbd_con: words MBD_CON0 to MBD_CON10
 * @reserved: must be 0
 */
struct mtk_mfb_params_blend {
	struct v4l2_isp_params_block_header header;
	__u32 ll_con1;
	__u32 ll_con2;
	__u32 mbd_con[MTK_MFB_MBD_CON_WORDS];
	__u32 reserved;
} __attribute__((aligned(8)));

/**
 * struct mtk_mfb_params_extra - other tuning words
 * @header: block header, type MTK_MFB_PARAMS_BLOCK_EXTRA
 * @con: bits 15:4 of the control word; bits 3:0 (the blend mode, set by
 *	the driver from mtk-mfb-weight-in) and 31:16 must be 0
 * @ll_con: words LL_CON3 to LL_CON10
 * @edge: word EDGE
 */
struct mtk_mfb_params_extra {
	struct v4l2_isp_params_block_header header;
	__u32 con;
	__u32 ll_con[MTK_MFB_LL_CON_EXTRA_WORDS];
	__u32 edge;
} __attribute__((aligned(8)));

#endif /* _UAPI_MTK_MFB_MT6785_H */
