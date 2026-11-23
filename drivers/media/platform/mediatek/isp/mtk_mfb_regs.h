/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * MediaTek MT6785 multi-frame blending block (MFB): registers.
 *
 * Register map from the MediaTek vendor driver of this block (camera_mfb.c,
 * isp_50, Copyright (C) 2015 MediaTek Inc., GPL-2.0).
 */

#ifndef _MTK_MFB_REGS_H
#define _MTK_MFB_REGS_H

#include <linux/bits.h>

/* blending control and tuning words */
#define MFB_CON				0x0000
#define MFB_CON_MODE			GENMASK(3, 0)
#define MFB_CON_TUNING			GENMASK(15, 4)
#define MFB_LL_CON1			0x0004
#define MFB_LL_CON2			0x0008
#define MFB_LL_CON3			0x000c
#define MFB_LL_CON4			0x0010
#define MFB_EDGE			0x0014
#define MFB_LL_CON5			0x0018
/* LL_CON5 .. LL_CON10 follow at 4-byte steps */
#define MFB_MBD_CON0			0x0030
/* MBD_CON0 .. MBD_CON10 follow at 4-byte steps */
#define MFB_TOP_CFG0			0x005c
#define MFB_TOP_CFG1			0x0060
#define MFB_TOP_CFG1_START		BIT(0)
#define MFB_TOP_CFG2			0x0064
#define MFB_INT_CTL			0x0080
#define MFB_INT_STATUS			0x0084
#define MFB_INT_DONE			BIT(0)
#define MFB_SW_RST			0x0088
#define MFB_MAIN_DCM_ST			0x008c
#define MFB_DMA_DCM_ST			0x0090
#define MFB_DBG_CTL0			0x009c
#define MFB_DBG_OUT0			0x00a0

/* tile descriptions */
#define MFB_DMA_SOFT_RSTSTAT		0x0100
#define MFB_TDRI_BASE_ADDR		0x0104
#define MFB_TDRI_OFST_ADDR		0x0108
#define MFB_TDRI_XSIZE			0x010c

/* DMA ports: base of the register group of each port */
#define MFB_MFBO			0x0130
#define MFB_MFB2O			0x0160
#define MFB_MFBI			0x0190
#define MFB_MFB2I			0x01c0
#define MFB_MFB3I			0x01f0
#define MFB_MFB4I			0x0220
#define MFB_MFBO_B			0x02a0
#define MFB_MFBI_B			0x02d0
#define MFB_MFB2I_B			0x0300
/* offsets inside the group of a port */
#define MFB_PORT_BASE_ADDR		0x00
#define MFB_PORT_OFST_ADDR		0x08
#define MFB_PORT_XSIZE			0x10
#define MFB_PORT_YSIZE			0x14
#define MFB_PORT_STRIDE			0x18
#define MFB_PORT_CON			0x1c

/* DMA error status, one word per port */
#define MFB_DMA_ERR_CTRL		0x0250
#define MFB_MFBO_ERR_STAT		0x0254
#define MFB_MFB2I_B_ERR_STAT		0x0274

/* resizer of the MFB4I port, chroma converters, 4:2:0 converter */
#define MFB_SRZ_CONTROL			0x0340
#define MFB_SRZ_IN_IMG			0x0344
#define MFB_SRZ_OUT_IMG			0x0348
#define MFB_SRZ_HORI_STEP		0x034c
#define MFB_SRZ_VERT_STEP		0x0350
#define MFB_SRZ_HORI_INT_OFST		0x0354
#define MFB_SRZ_HORI_SUB_OFST		0x0358
#define MFB_SRZ_VERT_INT_OFST		0x035c
#define MFB_SRZ_VERT_SUB_OFST		0x0360
#define MFB_C02A_CON			0x0380
#define MFB_C02B_CON			0x03a0
#define MFB_CRSP_CTRL			0x03c0
#define MFB_CRSP_OUT_IMG		0x03c8
#define MFB_CRSP_STEP_OFST		0x03cc

#endif /* _MTK_MFB_REGS_H */
