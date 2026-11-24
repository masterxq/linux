/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2020 BayLibre
 * Copyright (c) 2022 MediaTek Inc.
 */

#ifndef __MTK_CAMSV_H__
#define __MTK_CAMSV_H__

#include <linux/clk.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/kernel.h>
#include <linux/kref.h>
#include <linux/pm_runtime.h>
#include <linux/videodev2.h>
#include <linux/workqueue.h>
#include <media/media-entity.h>
#include <media/v4l2-subdev.h>
#include <media/videobuf2-core.h>
#include <media/videobuf2-dma-contig.h>
#include <media/videobuf2-v4l2.h>
#include <soc/mediatek/smi.h>

#define IMG_MAX_WIDTH			5376U
#define IMG_MAX_HEIGHT			4032U
#define IMG_DEF_WIDTH			1920U
#define IMG_DEF_HEIGHT			1080U
#define IMG_MIN_WIDTH			80U
#define IMG_MIN_HEIGHT			60U
#define MAX_SUPPORTED_NUM_PLANES	1U
/* largest frame: 16 bits per pixel */
#define IMG_MAX_SIZEIMAGE		(IMG_MAX_WIDTH * 2 * IMG_MAX_HEIGHT)

#define MTK_CAM_CIO_PAD_SENINF		0U
#define MTK_CAM_CIO_PAD_VIDEO		1U
#define MTK_CAM_CIO_NUM_PADS		2U

struct mtk_cam_format_info {
	u32 code;
	u32 fourcc;
	u8 bpp;
	u32 w_factor;
	u32 module_en_pak;
	u32 fmt_sel;
	u32 pak;
	u32 imgo_stride;
};

const struct mtk_cam_format_info *mtk_cam_format_info_find(u32 code,
							   u32 fourcc);


struct mtk_cam_dev_buffer {
	struct vb2_v4l2_buffer v4l2_buf;
	struct list_head list;
	dma_addr_t daddr;
};

/**
 * struct mtk_cam_vdev_desc - MTK camera device descriptor
 * @num_fmts: the number of supported node formats
 * @fmts: supported format
 * @frmsizes: supported V4L2 frame size number
 */
struct mtk_cam_vdev_desc {
	u8 num_fmts;
	const u32 *fmts;
	const struct v4l2_frmsizeenum *frmsizes;
};

/**
 * struct mtk_cam_video_device - MediaTek video device structure
 * @cam: The CAMSV, held until the node is released
 * @desc: The node description of video device
 * @vdev_pad: The media pad graph object of video device
 * @vdev: The video device instance
 * @vbq: A videobuf queue of video device
 * @vdev_lock: Serializes vb2 queue and video device operations
 * @format: The V4L2 format of video device
 * @fmtinfo: Information about the current format
 *
 * Allocated each time the seninf binds the CAMSV and freed in the release
 * of @vdev, after the last open file: an unbind does not free a node that
 * user space still holds open, and a node registered again is a new one.
 */
struct mtk_cam_video_device {
	struct mtk_cam_dev *cam;
	const struct mtk_cam_vdev_desc *desc;

	struct media_pad vdev_pad;
	struct video_device vdev;
	struct vb2_queue vbq;

	/* Serializes vb2 queue and video device operations */
	struct mutex vdev_lock;

	struct v4l2_pix_format_mplane format;
	const struct mtk_cam_format_info *fmtinfo;
};

/**
 * struct mtk_cam_dev - MediaTek camera device structure.
 * @dev: Pointer to device.
 * @regs: Base address of CAMSV.
 * @regs_img0: Base address of CAMSV IMG0.
 * @regs_tg: Base address of CAMSV TG.
 * @num_clks: Number of clocks.
 * @clks: The clocks.
 * @irq: Irq fired when buffer is ready.
 * @conf: soc specific driver data.
 * @subdev: The V4L2 sub-device instance.
 * @subdev_pads: Media pads of this sub-device.
 * @vdev: The video device node while the seninf has bound the CAMSV, or NULL.
 * @seninf: Pointer to the seninf pad.
 * @default_fmt: Pixel format of the video node after registration.
 * @streaming: The stream is running in the hardware (start_streaming
 *	succeeded and stop_streaming has not run yet). The system sleep
 *	callbacks stop and restart only such a stream.
 * @stream_fmtinfo: Packer and DMA setup of the running stream.
 * @stream_width: Width of the running stream in pixels.
 * @stream_height: Height of the running stream in lines.
 * @stream_bpl: Bytes per line of the running stream.
 * @sequence: Buffer sequence number.
 * @buf_list_lock: Protects the buffer list.
 * @buf_list: List head for the buffer list.
 * @kref: References of the driver (until the unbind), of the registered
 *	sub-device (until its device node is released) and of each video node
 *	(until its release). The structure is freed with the last of them.
 * @put_work: Drops the references of released sub-device registrations.
 * @deferred_puts: Number of references @put_work has to drop.
 */
struct mtk_cam_dev {
	struct device *dev;
	void __iomem *regs;
	void __iomem *regs_img0;
	void __iomem *regs_tg;

	unsigned int num_clks;
	struct clk_bulk_data *clks;
	unsigned int irq;
	const struct mtk_cam_conf *conf;

	struct v4l2_subdev subdev;
	struct media_pad subdev_pads[MTK_CAM_CIO_NUM_PADS];
	struct mtk_cam_video_device *vdev;
	struct media_pad *seninf;
	u32 default_fmt;
	bool streaming;
	const struct mtk_cam_format_info *stream_fmtinfo;
	u32 stream_width;
	u32 stream_height;
	u32 stream_bpl;
	unsigned int sequence;
	u32 tg_sen_mode;	/* TG_SEN_MODE of the current stream */

	spinlock_t buf_list_lock;
	struct list_head buf_list;

	/* no-FBC (MT8183) buffer tracking, protected by buf_list_lock */
	struct mtk_cam_dev_buffer *cur;
	void *scratch_va;
	dma_addr_t scratch_dma;
	size_t scratch_size;

	/* debug counters */
	u32 irq_cnt;
	u32 done_cnt;
	u32 err_cnt;
	u32 err_frames;
	u32 frame_errs;
	u32 frame_status;
	u32 scratch_cnt;
	u32 last_status;
	struct dentry *dbg;
	struct kref kref;
	struct work_struct put_work;
	atomic_t deferred_puts;
};

/**
 * struct mtk_cam_conf - MediaTek camera configuration structure
 * @tg_sen_mode: TG sensor mode
 * @module_en: module enable
 * @imgo_con: dma control register value
 * @imgo_con2: dma control register 2 value
 * @imgo_xsize: IMGO XSIZE register offset (from the IMGO window)
 * @imgo_ysize: IMGO YSIZE register offset
 * @imgo_stride: IMGO STRIDE register offset
 * @imgo_con_reg: IMGO CON register offset
 * @imgo_con2_reg: IMGO CON2 register offset
 * @imgo_con3: dma control register 3 value
 * @imgo_con3_reg: IMGO CON3 register offset, 0 if not written
 * @has_fbc: single-register IMGO FBC at top+0x1c (MT8365); without it the
 *	driver re-programs IMGO_BASE_ADDR per frame and parks the DMA on a
 *	scratch buffer when no user buffer is queued (MT8183)
 */
struct mtk_cam_conf {
	u32 tg_sen_mode;
	u32 module_en;
	u32 imgo_con;
	u32 imgo_con2;
	u32 imgo_xsize;
	u32 imgo_ysize;
	u32 imgo_stride;
	u32 imgo_con_reg;
	u32 imgo_con2_reg;
	u32 imgo_con3;
	u32 imgo_con3_reg;
	bool has_fbc;
	bool pak_reversed;	/* MT8183: PAK 0=RAW8 .. 3=RAW14 (MT8365: 0=RAW14 .. 3=RAW8) */
	bool pak_pix_mode;	/* MT6785: PAK and IMGO bus follow the pixel mode */
	bool all_clks;		/* take every clock of the node (MT6785: cam, camtg, sv0, sv1) */
	bool tg_idle_stop;	/* stop: wait for the TG to go idle, then reset the IMGO DMA */
};

int mtk_cam_dev_init(struct mtk_cam_dev *cam_dev);
void mtk_cam_dev_cleanup(struct mtk_cam_dev *cam_dev);
void mtk_cam_dev_get(struct mtk_cam_dev *cam_dev);
void mtk_cam_dev_put(struct mtk_cam_dev *cam_dev);
void mtk_cam_dev_put_deferred(struct mtk_cam_dev *cam_dev);
int mtk_cam_video_register(struct mtk_cam_dev *cam_dev);
void mtk_cam_video_unregister(struct mtk_cam_dev *cam_dev);
void mtk_cam_vb2_return_all_buffers(struct mtk_cam_dev *cam,
				    enum vb2_buffer_state state);


#endif /* __MTK_CAMSV_H__ */
