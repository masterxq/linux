/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * MediaTek MT6785 ISP pass 2 (DIP): registers.
 *
 * Names and offsets of the control, command queue, DMA port and interrupt
 * registers are those of the MediaTek vendor driver for this block
 * (camera_dip.c, isp_50, Copyright (C) 2016 MediaTek Inc., GPL-2.0). The
 * module enable bits, the tile walker registers and the registers of the
 * processing modules are named after their function.
 */

#ifndef _MTK_DIP_REGS_H
#define _MTK_DIP_REGS_H

#include <linux/bits.h>

#define DIP_REG_SIZE			0x6000

#define DIP_CTL_START			0x0000
/* bit n starts command queue thread n; the hardware clears it when done */
#define DIP_CTL_START_CQ_THR(n)		BIT(n)

/* module enables: processing in YUV, second YUV group, raw/RGB, DMA ports */
#define DIP_CTL_YUV_EN			0x0004
#define DIP_CTL_YUV2_EN			0x0008
#define DIP_CTL_RGB_EN			0x000c
#define DIP_CTL_DMA_EN			0x0010
#define DIP_CTL_FMT_SEL			0x0014
#define DIP_CTL_PATH_SEL		0x0018
#define DIP_CTL_MISC_SEL		0x001c
#define DIP_CTL_RGB2_EN			0x00e8

#define DIP_FMT_IN			GENMASK(4, 0)
#define DIP_FMT_IN_RAW8			8
#define DIP_FMT_IN_RAW10		9
/* format of the reference read ports: 3 = NV12, as the main output */
#define DIP_FMT_REF			GENMASK(12, 8)
#define DIP_FMT_REF_NV12		3
#define DIP_FMT_OUT			GENMASK(22, 20)
#define DIP_FMT_OUT_NV12		3
#define DIP_FMT_PIX_ID			GENMASK(29, 28)
/* scaled output: 0 = YUYV in one plane */
#define DIP_FMT_OUT_S			BIT(24)

#define DIP_CTL_INT_EN			0x0020
#define DIP_CTL_CQ_INT_EN		0x0024
#define DIP_CTL_CQ_INT2_EN		0x0028
#define DIP_CTL_CQ_INT3_EN		0x002c
#define DIP_CTL_INT_STATUS		0x0030
#define DIP_CTL_CQ_INT_STATUS		0x0034
#define DIP_CTL_CQ_INT2_STATUS		0x0038
#define DIP_CTL_INT_STATUSX		0x0040
#define DIP_CTL_CQ_INT_STATUSX		0x0044
/* debug state: module done flags of the tile, request/ready handshake */
#define DIP_CTL_DONE0			0x0060
#define DIP_CTL_DONE1			0x0064
#define DIP_CTL_HANDSHAKE		0x00b8

/* tile walker: control, tile index and the three module enable registers */
#define DIP_TDR_CTL			0x0050
#define DIP_TDR_CTL_VAL			0x0b
#define DIP_TDR_TILE			0x0054
#define DIP_TDR_TCM0_EN			0x0058
#define DIP_TDR_TCM1_EN			0x005c
#define DIP_TDR_TCM2_EN			0x00e0

#define DIP_CTL_TOP_DCM_DIS		0x00a0
#define DIP_CTL_TOP_DCM_DIS_ALL		BIT(0)

#define DIP_CQ_THR_NUM			19
#define DIP_CQ_THR_CTL(n)		(0x0204 + 12 * (n))
#define DIP_CQ_THR_BASEADDR(n)		(0x0208 + 12 * (n))
#define DIP_CQ_THR_DESC_SIZE(n)		(0x020c + 12 * (n))
/* the vendor initial value: thread enabled, immediate mode */
#define DIP_CQ_THR_CTL_INIT		0x11
#define DIP_CQ_THR_DESC_SIZE_INIT	0x308

/* Tile description (TDR) input base address */
#define DIP_TDRI_BASE_ADDR		0x0304

/* DMA ports: base address, then XSIZE, YSIZE, STRIDE at +0x10/+0x14/+0x18 */
#define DIP_IMG2O_BASE			0x0330
#define DIP_IMG2BO_BASE			0x0360
#define DIP_IMG3O_BASE			0x0390
#define DIP_IMG3BO_BASE			0x03c0
#define DIP_IMG3CO_BASE			0x03f0
#define DIP_FEO_BASE			0x0420
#define DIP_PAK2O_BASE			0x0450
#define DIP_IMGI_BASE			0x0500
#define DIP_IMGBI_BASE			0x0530
#define DIP_IMGCI_BASE			0x0560
#define DIP_VIPI_BASE			0x0590
#define DIP_VIP2I_BASE			0x05c0
#define DIP_VIP3I_BASE			0x05f0
#define DIP_DMGI_BASE			0x0620
#define DIP_DEPI_BASE			0x0650
#define DIP_LCEI_BASE			0x0680
#define DIP_UFDI_BASE			0x06b0
#define DIP_SMX1O_BASE			0x07d0
#define DIP_SMX2O_BASE			0x0800
#define DIP_SMX3O_BASE			0x0830
#define DIP_SMX4O_BASE			0x0860
#define DIP_SMX1I_BASE			0x0890
#define DIP_SMX2I_BASE			0x08c0
#define DIP_SMX3I_BASE			0x08f0
#define DIP_SMX4I_BASE			0x0920

#define DIP_DMA_XSIZE			0x10
#define DIP_DMA_YSIZE			0x14
#define DIP_DMA_STRIDE			0x18
#define DIP_DMA_STRIDE_BUS1		BIT(16)
/*
 * Upper bits of the stride of the scaled output port for YUYV (vendor
 * value)
 */
#define DIP_DMA_STRIDE_OUT_S_YUYV	0x01910000
/*
 * Upper bits of the stride of the reference Y port for a planar YUV frame
 * (vendor value)
 */
#define DIP_DMA_STRIDE_REF_Y		0x40000000
/* FIFO setting CON at +0x1c of a port, CON2 and CON3 behind it */
#define DIP_DMA_CON2			0x4
#define DIP_DMA_CON3			0x8
#define DIP_DMA_PORT_NUM		25

/* Processing modules: registers the driver computes */
#define DIP_BLC_OFFSET(c)		(0x11c0 + 4 * (c))	/* B, Gr, Gb, R */
#define DIP_BLC_GAIN(c)			(0x11d0 + 4 * (c))
#define DIP_LSC_CTL			0x1340
#define DIP_LSC_GRID_X			0x1344
#define DIP_LSC_GRID_Y			0x1348
#define DIP_LSC_LAST			0x134c
#define DIP_LSC_STRENGTH0		0x1350
#define DIP_LSC_STRENGTH1		0x1360
#define DIP_LSC_GRID_SIZE		GENMASK(12, 0)
#define DIP_LSC_GRID_NUM		GENMASK(17, 13)
#define DIP_LSC_LAST_W			GENMASK(28, 16)
#define DIP_LSC_LAST_H			GENMASK(12, 0)
#define DIP_WB_SAT0			0x1440
#define DIP_WB_SAT1			0x1444
#define DIP_WB_GAIN_BG			0x1448	/* B [12:0], G of the blue row [28:16] */
#define DIP_WB_GAIN_GR			0x144c	/* G of the red row [12:0], R [28:16] */
#define DIP_WB_FLARE0			0x1450
#define DIP_WB_FLARE1			0x1454
#define DIP_WB_SAT_FULL			0x0fff0fff
#define DIP_CCM(n)			(0x2080 + 4 * (n))
#define DIP_CCM_CTL0			0x2098
#define DIP_CCM_CTL1			0x209c
#define DIP_GAMMA_LUT(i)		(0x2180 + 4 * (i))
#define DIP_GAMMA_CTL			0x2480
#define DIP_RGB2YUV(n)			(0x3240 + 4 * (n))
#define DIP_STAT_SCALER_CTL		0x3880
#define DIP_STAT_SCALER_IN		0x3884
#define DIP_STAT_SCALER_OUT		0x3888
#define DIP_STAT_SCALER_STEP_X		0x388c
#define DIP_STAT_SCALER_STEP_Y		0x3890
/*
 * Scaler of the scaled output: control, input and output size (width in the
 * low half), horizontal and vertical step (output pixels per input pixel,
 * Q20), then the frame offsets, which stay 0
 */
#define DIP_SCALER_CTL			0x5300
#define DIP_SCALER_CTL_H_EN		BIT(0)
#define DIP_SCALER_CTL_V_EN		BIT(1)
#define DIP_SCALER_CTL_H_MODE		GENMASK(6, 5)
#define DIP_SCALER_CTL_V_MODE		GENMASK(8, 7)
/* source accumulation: down-scaling to 1/2 and below */
#define DIP_SCALER_MODE_ACC		1
#define DIP_SCALER_IN			0x5304
#define DIP_SCALER_OUT			0x5308
#define DIP_SCALER_STEP_X		0x530c
#define DIP_SCALER_STEP_Y		0x5310
#define DIP_SCALER_LAST			0x5334

/*
 * Temporal noise reduction: control (vendor value with the module on), mode
 * word, blend window (offset and size, width in the low half), the words of the
 * module behind it, the four counters of the pattern part (6 bits each from bit
 * 26 down in steps of 6, from bit 8 up)
 */
#define DIP_NR3D_CTL			0x5380
#define DIP_NR3D_CTL_ON			0x00001003
#define DIP_NR3D_MODE			0x538c
#define DIP_NR3D_ON_OFST		0x5390
#define DIP_NR3D_ON_SIZE		0x5394
#define DIP_NR3D_WORD(i)		(0x5394 + 4 * (i))	/* i >= 1 */
#define DIP_NR3D_COUNT			0x53ec
#define DIP_NR3D_COUNT_SHIFT(i)		(26 - 6 * (i))
/* radial weights of the temporal noise reduction */
#define DIP_NR3D_RADIAL			0x52c0
/* 4:2:0 to 4:2:2 converter of the reference: edges, columns, lines */
#define DIP_REF422_EDGE			0x3080
#define DIP_REF422_X			0x3084
#define DIP_REF422_Y			0x3088
#define DIP_REF422_EDGE_ALL		0xf

/*
 * Blend of the multi-frame jobs: two control words. Word 0: ramp weight
 * belongs to the base image, luma and chroma blend on, source of each
 * component with its blend off, then the weight limits and the slope;
 * word 1: the two map points of the ramp. The uapi header describes the
 * fields.
 */
#define DIP_MIX_CTL0			0x32c0
#define DIP_MIX_CTL1			0x32c4
#define DIP_MIX_WT_BASE			BIT(0)
#define DIP_MIX_Y_EN			BIT(4)
#define DIP_MIX_Y_PASS_BASE		BIT(5)
#define DIP_MIX_UV_EN			BIT(6)
#define DIP_MIX_UV_PASS_BASE		BIT(7)
#define DIP_MIX_WT_MIN			GENMASK(15, 8)
#define DIP_MIX_WT_MAX			GENMASK(23, 16)
#define DIP_MIX_SLOPE			GENMASK(31, 24)
#define DIP_MIX_MAP_START		GENMASK(7, 0)
#define DIP_MIX_MAP_END			GENMASK(23, 16)

/* 13-bit fields of the black level, white balance and colour matrix */
#define DIP_FIELD13_LO			GENMASK(12, 0)
#define DIP_FIELD13_HI			GENMASK(28, 16)
/* 11-bit fields of the RGB to YUV matrix */
#define DIP_FIELD11_LO			GENMASK(10, 0)
#define DIP_FIELD11_HI			GENMASK(26, 16)
/* gamma: three 10-bit outputs per point */
#define DIP_GAMMA_R			GENMASK(9, 0)
#define DIP_GAMMA_G			GENMASK(19, 10)
#define DIP_GAMMA_B			GENMASK(29, 20)

#endif /* _MTK_DIP_REGS_H */
