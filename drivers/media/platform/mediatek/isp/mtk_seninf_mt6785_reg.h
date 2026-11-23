/* SPDX-License-Identifier: GPL-2.0 */
/*
 * MT6785 SENINF registers ("csi_phy_2_0" generation, as on MT6833/MT8188).
 *
 * Field names follow the MT8188 ISP7 seninf headers where the layout is the
 * same.
 */

#ifndef __MTK_SENINF_MT6785_REG_H__
#define __MTK_SENINF_MT6785_REG_H__

#include <linux/bits.h>

/* Top block (seninf base) */
#define MT6785_SENINF_TOP_CTRL			0x0000
/* MUX0..3 source SENINF, 4 bits each at 0/8/16/24; MUX4..5 in CTRL_1 at 0/8 */
#define MT6785_SENINF_TOP_MUX_CTRL_0		0x0010
#define MT6785_SENINF_TOP_MUX_CTRL_1		0x0014
#define MT6785_SENINF_TOP_PHY_CTRL_CSI(n)	(0x0060 + 4 * (n))
#define MT6785_PHY_CTRL_DPHY_EN			BIT(0)
#define MT6785_PHY_CTRL_CPHY_EN			BIT(1)

/* CAM mux block (seninf base + 0x400): 11 CAM muxes towards CAM A/B and CAMSV1..6 */
#define MT6785_SENINF_CAM_MUX_BASE		0x0400
/* source MUX of CAM mux k, 4 bits */
#define MT6785_CAM_MUX_CTRL(k)			(0x0000 + 4 * ((k) / 4))
#define MT6785_CAM_MUX_CTRL_SHIFT(k)		(8 * ((k) % 4))
#define MT6785_CAM_MUX_EN			0x0010
#define MT6785_CAM_MUX_OPT(k)			(0x0020 + 4 * (k))
/* virtual channel and data type filter of CAM mux k */
#define MT6785_CAM_MUX_OPT_VC_EN		BIT(0)
#define MT6785_CAM_MUX_OPT_DT_EN		BIT(1)
#define MT6785_CAM_MUX_OPT_VC_SEL		GENMASK(12, 8)
#define MT6785_CAM_MUX_OPT_DT_SEL		GENMASK(21, 16)
#define MT6785_CAM_MUX_IRQ_EN			0x00a0
#define MT6785_CAM_MUX_IRQ_STATUS		0x00a8
#define MT6785_CAM_MUX_CHK_CTL_0(k)		(0x0100 + 0x10 * (k))
#define MT6785_CAM_MUX_CHK_PIX_MODE		GENMASK(1, 0)
#define MT6785_CAM_MUX_CHK_CTL_1(k)		(0x0104 + 0x10 * (k))	/* expected size */
#define MT6785_CAM_MUX_CHK_RES(k)		(0x0108 + 0x10 * (k))	/* measured size */
#define MT6785_CAM_MUX_CHK_ERR_RES(k)		(0x010c + 0x10 * (k))
#define MT6785_NUM_CAM_MUXES			11

/* Per input (seninf n at n * 0x1000): control block */
#define MT6785_SENINF_CTRL			0x0200
#define MT6785_SENINF_EN			BIT(0)
#define MT6785_SENINF_CSI_CTRL			0x0210
#define MT6785_SENINF_CSI_CSI2_EN		BIT(0)
#define MT6785_SENINF_CSI2_SW_RST		BIT(4)
#define MT6785_SENINF_TESTMDL_CTRL		0x0220
#define MT6785_SENINF_TESTMDL_EN		BIT(0)

/* Per input: test model (pattern generator) */
#define MT6785_SENINF_TM_CTL			0x0608
#define MT6785_SENINF_TM_SIZE			0x060c
#define MT6785_SENINF_TM_CLK			0x0610

/* Per input: CSI-2 receiver */
#define MT6785_SENINF_CSI2_EN			0x0a00	/* data lane mask */
#define MT6785_SENINF_CSI2_OPT			0x0a04
#define MT6785_CSI2_CPHY_SEL			BIT(0)
#define MT6785_SENINF_CSI2_HDR_MODE_0		0x0a08
#define MT6785_CSI2_HEADER_MODE			GENMASK(7, 0)
#define MT6785_CSI2_HEADER_LEN			GENMASK(10, 8)
#define MT6785_SENINF_CSI2_RESYNC_MERGE_CTRL	0x0a10
#define MT6785_CSI2_RESYNC_CYCLE_CNT_OPT	BIT(8)
#define MT6785_SENINF_CSI2_DI_CTRL(s)		(0x0a20 + 4 * (s))
#define MT6785_CSI2_VC_INTERLEAVE_EN		BIT(0)
#define MT6785_CSI2_DT_INTERLEAVE_MODE		GENMASK(5, 4)
#define MT6785_CSI2_VC_SEL			GENMASK(9, 8)
#define MT6785_CSI2_DT_SEL			GENMASK(21, 16)
/*
 * Streams of the CSI-2 receiver: DI_CTRL(s) selects the virtual channel and
 * data type of stream s, CH_CTRL(c) the streams of output channel c. The MUX
 * fed by channel c selects it as source MT6785_MUX_SRC_MIPI + c.
 */
#define MT6785_SENINF_CSI2_NUM_STREAMS		4
#define MT6785_SENINF_CSI2_CH_CTRL(c)		(0x0a60 + 4 * (c))
#define MT6785_CSI2_CH_STREAM(s)		BIT(8 + (s))
#define MT6785_SENINF_CSI2_IRQ_EN		0x0ac0
#define MT6785_CSI2_IRQ_CLR_MODE		BIT(31)
#define MT6785_SENINF_CSI2_IRQ_STATUS		0x0ac8
#define MT6785_SENINF_CSI2_PACKET_STATUS	0x0ad4
#define MT6785_SENINF_CSI2_PACKET_CNT_STATUS	0x0adc

/* Per MUX (mux m at m * 0x1000) */
#define MT6785_SENINF_MUX_CTRL_0		0x0d00
#define MT6785_SENINF_MUX_EN			BIT(0)
#define MT6785_SENINF_MUX_IRQ_SW_RST		BIT(1)
#define MT6785_SENINF_MUX_SW_RST		BIT(2)
#define MT6785_SENINF_MUX_CTRL_1		0x0d04
#define MT6785_SENINF_MUX_SRC_SEL		GENMASK(3, 0)
#define MT6785_SENINF_MUX_PIX_MODE		GENMASK(9, 8)
#define MT6785_SENINF_MUX_FIFO_PUSH_EN		GENMASK(21, 16)
#define MT6785_SENINF_MUX_OPT			0x0d08
#define MT6785_SENINF_MUX_HSYNC_POL		BIT(16)
#define MT6785_SENINF_MUX_VSYNC_POL		BIT(17)
#define MT6785_SENINF_MUX_IRQ_STATUS		0x0d18
#define MT6785_SENINF_MUX_SIZE			0x0d30

/* MUX source selection */
#define MT6785_MUX_SRC_TEST_MODEL		1
#define MT6785_MUX_SRC_MIPI			8

#endif /* __MTK_SENINF_MT6785_REG_H__ */
