/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Decoder programmed by the host through its registers, without firmware.
 */

#ifndef _MTK_VCODEC_DEC_DIRECT_H_
#define _MTK_VCODEC_DEC_DIRECT_H_

#include <linux/bits.h>
#include <linux/types.h>

#include "../common/mtk_vcodec_cmn_drv.h"

struct mtk_vcodec_dec_pdata;

struct mtk_vcodec_dec_dev;

/* registers shared by all codecs of the host-programmed decoder */
#define VDEC_SYS_DCM_CON		0x018

#define VDEC_VLD_SRST			0x108
#define VDEC_VLD_SRST_RESET		BIT(0)

#define VDEC_MISC_HW_BREAK		0x100
#define VDEC_MISC_HW_BREAK_REQ		BIT(0)
#define VDEC_MISC_HW_BREAK_STA		0x104
#define VDEC_MISC_HW_BREAK_DONE		(BIT(0) | BIT(4))

/**
 * struct mtk_vdec_direct_data - decoder without firmware
 * @reg_offset:		offset of the register blocks VDEC_MISC to VDEC_PP in
 *			the register window of the node; VDEC_SYS is reached
 *			through the vdecsys syscon
 * @hevc_vld:		offset of the HEVC syntax block in the register
 *			window, 0 without HEVC
 * @vp9_vld:		offset of the VP9 syntax block, 0 without VP9
 * @vp8_vld:		offset of the VP8 syntax block, 0 without VP8
 * @vld2:		offset of the second bitstream reader that VP8 and VP9
 *			use next to VDEC_LD, 0 without them
 * @capability:		MTK_VDEC_FORMAT_* flags of the codecs and formats
 * @max_width:		largest coded width
 * @max_height:		largest coded height
 *
 * The host writes the registers of the decoder for each picture itself;
 * no firmware is loaded and no coprocessor is involved.
 */
struct mtk_vdec_direct_data {
	u32 reg_offset[NUM_MAX_VDEC_REG_BASE];
	u32 hevc_vld;
	u32 vp9_vld;
	u32 vp8_vld;
	u32 vld2;
	unsigned int capability;
	unsigned int max_width;
	unsigned int max_height;
	const struct mtk_vcodec_dec_pdata *stateful;
};

extern const struct mtk_vdec_direct_data mtk_vdec_mt6785_direct_data;
extern const struct mtk_vcodec_dec_pdata mtk_vdec_direct_stateful_pdata;

int mtk_vdec_direct_init(struct mtk_vcodec_dec_dev *dev);
void __iomem *mtk_vdec_direct_block(struct mtk_vcodec_dec_dev *dev, u32 offset);
void mtk_vdec_direct_hw_break(struct mtk_vcodec_dec_dev *dev);

#endif /* _MTK_VCODEC_DEC_DIRECT_H_ */
