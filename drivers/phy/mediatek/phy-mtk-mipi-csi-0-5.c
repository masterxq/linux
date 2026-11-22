// SPDX-License-Identifier: GPL-2.0
/*
 * MediaTek MIPI CSI v0.5 driver
 *
 * Copyright (c) 2023, MediaTek Inc.
 * Copyright (c) 2023, BayLibre Inc.
 */

#include <dt-bindings/phy/phy.h>
#include <linux/bitfield.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/nvmem-consumer.h>
#include <linux/mutex.h>
#include <linux/phy/phy.h>
#include <linux/platform_device.h>
#include <linux/slab.h>

#include "phy-mtk-io.h"
#include "phy-mtk-mipi-csi-0-5-rx-reg.h"

#define CSIXB_OFFSET		0x1000

/*
 * MT6785 port window (0x4000 per CSI port): analog A at +0x0, analog B at
 * +0x1000 (same layout as above), D-PHY top at +0x2000, C-PHY top at +0x3000.
 * Fields of the D-PHY top as in the MT8188 ISP7 csi0-dphy header.
 */
#define MT6785_DPHY_TOP_OFFSET			0x2000
#define MT6785_ANA04				0x0004
#define MT6785_ANA04_LPRX_VTL_VTH		(GENMASK(2, 0) | GENMASK(6, 4) | \
						 GENMASK(10, 8) | GENMASK(14, 12))
#define MT6785_ANA04_BG_VREF_SEL		GENMASK(19, 16)
#define MT6785_ANA08				0x0008	/* HSRT codes L0P/L0N/L1P/L1N */
#define MT6785_ANA0C				0x000c	/* HSRT codes L2P/L2N */
#define MT6785_HSRT_CODE_4			(GENMASK(4, 0) | GENMASK(12, 8) | \
						 GENMASK(20, 16) | GENMASK(28, 24))
#define MT6785_HSRT_CODE_2			(GENMASK(4, 0) | GENMASK(12, 8))
#define MT6785_ANA_EQ(n)			(0x0018 + 4 * (n))	/* lanes 0..2 */
#define MT6785_EQ_OS_CAL_EN			(BIT(0) | BIT(16))
#define MT6785_EQ_EN				(BIT(2) | BIT(18))
#define MT6785_EQ_IS				(GENMASK(5, 4) | GENMASK(21, 20))
#define MT6785_EQ_BW				(GENMASK(7, 6) | GENMASK(23, 22))
#define MT6785_EQ_DG				(GENMASK(11, 8) | GENMASK(27, 24))
#define MT6785_ANA28				0x0028
#define MT6785_ANA30				0x0030
#define MT6785_ANA34				0x0034
#define MT6785_ANA3C				0x003c
#define MT6785_CDR_CODE				(GENMASK(4, 0) | GENMASK(12, 8) | \
						 GENMASK(20, 16) | GENMASK(28, 24))
#define MT6785_ANA48				0x0048
#define MT6785_ANA48_OS_CAL_DONE		(BIT(0) | BIT(3) | BIT(5))

#define MT6785_DPHY_RX_LANE_EN			0x0000
#define MT6785_DPHY_RX_LC0_EN			BIT(0)
#define MT6785_DPHY_RX_LC1_EN			BIT(1)
#define MT6785_DPHY_RX_LD_EN			GENMASK(11, 8)
#define MT6785_DPHY_RX_LD0_EN			BIT(8)
#define MT6785_DPHY_RX_LD1_EN			BIT(9)
#define MT6785_DPHY_RX_LANE_SELECT		0x0004
#define MT6785_DPHY_RX_LC0_SEL			GENMASK(2, 0)
#define MT6785_DPHY_RX_LC1_SEL			GENMASK(6, 4)
#define MT6785_DPHY_RX_LD0_SEL			GENMASK(10, 8)
#define MT6785_DPHY_RX_LD1_SEL			GENMASK(14, 12)
#define MT6785_DPHY_RX_LD2_SEL			GENMASK(18, 16)
#define MT6785_DPHY_RX_LD3_SEL			GENMASK(22, 20)
#define MT6785_DPHY_RX_CK_DATA_MUX_EN		BIT(31)
#define MT6785_DPHY_RX_DATA_LANE_HS(n)		(0x0020 + 4 * (n))
#define MT6785_DPHY_RX_HS_TRAIL			GENMASK(15, 8)
#define MT6785_DPHY_RX_HS_SETTLE		GENMASK(23, 16)
#define MT6785_DPHY_RX_HS_TRAIL_EN		BIT(29)
#define MT6785_DPHY_RX_DESKEW_STATUS		0x0084
#define MT6785_DPHY_RX_DESKEW_DONE		GENMASK(15, 0)

/*
 * Above this lane rate the D-PHY hardware deskew check runs after the sensor
 * starts streaming (D-PHY v1.2 requires deskew calibration from 1.5 Gbit/s
 * on).
 */
#define MT6785_DESKEW_MIN_RATE			1427000000ULL

/* default position of the three 5-bit HSRT codes in an efuse word: 31:17 */
#define MT6785_EFUSE_HSRT_SHIFT			17

struct mtk_mipi_cdphy_port {
	struct device *dev;
	void __iomem *base;
	struct phy *phy;
	u32 type;
	u32 mode;
	u32 num_lanes;
	u32 efuse[2];	/* MT6785: HSRT trimming words of the A and B half, 0 = none */
	u32 efuse_shift[2];	/* MT6785: bit position of the codes in each word */
	u64 lane_rate;	/* MT6785: bit/s per lane from phy_configure(), 0 = unknown */
	u32 lanes;	/* MT6785: data lanes from phy_configure(), 0 = num_lanes */
};

enum PHY_TYPE {
	DPHY = 0,
	CPHY,
	CDPHY,
};

static void mtk_phy_csi_cdphy_ana_eq_tune(void __iomem *base)
{
	mtk_phy_update_field(base + MIPI_RX_ANA18_CSIXA, RG_CSI0A_L0_T0AB_EQ_IS, 1);
	mtk_phy_update_field(base + MIPI_RX_ANA18_CSIXA, RG_CSI0A_L0_T0AB_EQ_BW, 1);
	mtk_phy_update_field(base + MIPI_RX_ANA1C_CSIXA, RG_CSI0A_L1_T1AB_EQ_IS, 1);
	mtk_phy_update_field(base + MIPI_RX_ANA1C_CSIXA, RG_CSI0A_L1_T1AB_EQ_BW, 1);
	mtk_phy_update_field(base + MIPI_RX_ANA20_CSI0A, RG_CSI0A_L2_T1BC_EQ_IS, 1);
	mtk_phy_update_field(base + MIPI_RX_ANA20_CSI0A, RG_CSI0A_L2_T1BC_EQ_BW, 1);

	mtk_phy_update_field(base + CSIXB_OFFSET + MIPI_RX_ANA18_CSIXA, RG_CSI0A_L0_T0AB_EQ_IS, 1);
	mtk_phy_update_field(base + CSIXB_OFFSET + MIPI_RX_ANA18_CSIXA, RG_CSI0A_L0_T0AB_EQ_BW, 1);
	mtk_phy_update_field(base + CSIXB_OFFSET + MIPI_RX_ANA1C_CSIXA, RG_CSI0A_L1_T1AB_EQ_IS, 1);
	mtk_phy_update_field(base + CSIXB_OFFSET + MIPI_RX_ANA1C_CSIXA, RG_CSI0A_L1_T1AB_EQ_BW, 1);
	mtk_phy_update_field(base + CSIXB_OFFSET + MIPI_RX_ANA20_CSI0A, RG_CSI0A_L2_T1BC_EQ_IS, 1);
	mtk_phy_update_field(base + CSIXB_OFFSET + MIPI_RX_ANA20_CSI0A, RG_CSI0A_L2_T1BC_EQ_BW, 1);
}

static void mtk_phy_csi_dphy_ana_eq_tune(void __iomem *base)
{
	mtk_phy_update_field(base + MIPI_RX_ANA18_CSIXA, RG_CSI1A_L0_EQ_IS, 1);
	mtk_phy_update_field(base + MIPI_RX_ANA18_CSIXA, RG_CSI1A_L0_EQ_BW, 1);
	mtk_phy_update_field(base + MIPI_RX_ANA18_CSIXA, RG_CSI1A_L1_EQ_IS, 1);
	mtk_phy_update_field(base + MIPI_RX_ANA18_CSIXA, RG_CSI1A_L1_EQ_BW, 1);
	mtk_phy_update_field(base + MIPI_RX_ANA1C_CSIXA, RG_CSI1A_L2_EQ_IS, 1);
	mtk_phy_update_field(base + MIPI_RX_ANA1C_CSIXA, RG_CSI1A_L2_EQ_BW, 1);

	mtk_phy_update_field(base + CSIXB_OFFSET + MIPI_RX_ANA18_CSIXA, RG_CSI1A_L0_EQ_IS, 1);
	mtk_phy_update_field(base + CSIXB_OFFSET + MIPI_RX_ANA18_CSIXA, RG_CSI1A_L0_EQ_BW, 1);
	mtk_phy_update_field(base + CSIXB_OFFSET + MIPI_RX_ANA18_CSIXA, RG_CSI1A_L1_EQ_IS, 1);
	mtk_phy_update_field(base + CSIXB_OFFSET + MIPI_RX_ANA18_CSIXA, RG_CSI1A_L1_EQ_BW, 1);
	mtk_phy_update_field(base + CSIXB_OFFSET + MIPI_RX_ANA1C_CSIXA, RG_CSI1A_L2_EQ_IS, 1);
	mtk_phy_update_field(base + CSIXB_OFFSET + MIPI_RX_ANA1C_CSIXA, RG_CSI1A_L2_EQ_BW, 1);
}

static int mtk_mipi_phy_power_on(struct phy *phy)
{
	struct mtk_mipi_cdphy_port *port = phy_get_drvdata(phy);
	void __iomem *base = port->base;

	/*
	 * The driver currently supports DPHY and CD-PHY phys,
	 * but the only mode supported is DPHY,
	 * so CD-PHY capable phys must be configured in DPHY mode
	 */
	if (port->type == CDPHY) {
		mtk_phy_update_field(base + MIPI_RX_ANA00_CSIXA, RG_CSI0A_CPHY_EN, 0);
		mtk_phy_update_field(base + CSIXB_OFFSET + MIPI_RX_ANA00_CSIXA,
				     RG_CSI0A_CPHY_EN, 0);
	}

	/*
	 * Lane configuration:
	 *
	 * Only 4 data + 1 clock is supported for now with the following mapping:
	 *
	 * CSIXA_LNR0 --> D2
	 * CSIXA_LNR1 --> D0
	 * CSIXA_LNR2 --> C
	 * CSIXB_LNR0 --> D1
	 * CSIXB_LNR1 --> D3
	 */
	mtk_phy_update_field(base + MIPI_RX_ANA00_CSIXA, RG_CSIXA_DPHY_L0_CKMODE_EN, 0);
	mtk_phy_update_field(base + MIPI_RX_ANA00_CSIXA, RG_CSIXA_DPHY_L0_CKSEL, 1);
	mtk_phy_update_field(base + MIPI_RX_ANA00_CSIXA, RG_CSIXA_DPHY_L1_CKMODE_EN, 0);
	mtk_phy_update_field(base + MIPI_RX_ANA00_CSIXA, RG_CSIXA_DPHY_L1_CKSEL, 1);
	mtk_phy_update_field(base + MIPI_RX_ANA00_CSIXA, RG_CSIXA_DPHY_L2_CKMODE_EN, 1);
	mtk_phy_update_field(base + MIPI_RX_ANA00_CSIXA, RG_CSIXA_DPHY_L2_CKSEL, 1);

	mtk_phy_update_field(base + CSIXB_OFFSET + MIPI_RX_ANA00_CSIXA,
			     RG_CSIXA_DPHY_L0_CKMODE_EN, 0);
	mtk_phy_update_field(base + CSIXB_OFFSET + MIPI_RX_ANA00_CSIXA, RG_CSIXA_DPHY_L0_CKSEL, 1);
	mtk_phy_update_field(base + CSIXB_OFFSET + MIPI_RX_ANA00_CSIXA,
			     RG_CSIXA_DPHY_L1_CKMODE_EN, 0);
	mtk_phy_update_field(base + CSIXB_OFFSET + MIPI_RX_ANA00_CSIXA, RG_CSIXA_DPHY_L1_CKSEL, 1);
	mtk_phy_update_field(base + CSIXB_OFFSET + MIPI_RX_ANA00_CSIXA,
			     RG_CSIXA_DPHY_L2_CKMODE_EN, 0);
	mtk_phy_update_field(base + CSIXB_OFFSET + MIPI_RX_ANA00_CSIXA, RG_CSIXA_DPHY_L2_CKSEL, 1);

	/* Byte clock invert */
	mtk_phy_update_field(base + MIPI_RX_ANAA8_CSIXA, RG_CSIXA_CDPHY_L0_T0_BYTECK_INVERT, 1);
	mtk_phy_update_field(base + MIPI_RX_ANAA8_CSIXA, RG_CSIXA_DPHY_L1_BYTECK_INVERT, 1);
	mtk_phy_update_field(base + MIPI_RX_ANAA8_CSIXA, RG_CSIXA_CDPHY_L2_T1_BYTECK_INVERT, 1);

	mtk_phy_update_field(base + CSIXB_OFFSET + MIPI_RX_ANAA8_CSIXA,
			     RG_CSIXA_CDPHY_L0_T0_BYTECK_INVERT, 1);
	mtk_phy_update_field(base + CSIXB_OFFSET + MIPI_RX_ANAA8_CSIXA,
			     RG_CSIXA_DPHY_L1_BYTECK_INVERT, 1);
	mtk_phy_update_field(base + CSIXB_OFFSET + MIPI_RX_ANAA8_CSIXA,
			     RG_CSIXA_CDPHY_L2_T1_BYTECK_INVERT, 1);

	/* Start ANA EQ tuning */
	if (port->type == CDPHY)
		mtk_phy_csi_cdphy_ana_eq_tune(base);
	else
		mtk_phy_csi_dphy_ana_eq_tune(base);

	/* End ANA EQ tuning */
	mtk_phy_set_bits(base + MIPI_RX_ANA40_CSIXA, 0x90);

	mtk_phy_update_field(base + MIPI_RX_ANA24_CSIXA, RG_CSIXA_RESERVE, 0x40);
	mtk_phy_update_field(base + CSIXB_OFFSET + MIPI_RX_ANA24_CSIXA, RG_CSIXA_RESERVE, 0x40);
	mtk_phy_update_field(base + MIPI_RX_WRAPPER80_CSIXA, CSR_CSI_RST_MODE, 0);
	mtk_phy_update_field(base + CSIXB_OFFSET + MIPI_RX_WRAPPER80_CSIXA, CSR_CSI_RST_MODE, 0);
	/* ANA power on */
	mtk_phy_update_field(base + MIPI_RX_ANA00_CSIXA, RG_CSIXA_BG_CORE_EN, 1);
	mtk_phy_update_field(base + CSIXB_OFFSET + MIPI_RX_ANA00_CSIXA, RG_CSIXA_BG_CORE_EN, 1);
	usleep_range(20, 40);
	mtk_phy_update_field(base + MIPI_RX_ANA00_CSIXA, RG_CSIXA_BG_LPF_EN, 1);
	mtk_phy_update_field(base + CSIXB_OFFSET + MIPI_RX_ANA00_CSIXA, RG_CSIXA_BG_LPF_EN, 1);

	return 0;
}

static int mtk_mipi_phy_power_off(struct phy *phy)
{
	struct mtk_mipi_cdphy_port *port = phy_get_drvdata(phy);
	void __iomem *base = port->base;

	/* Disable MIPI BG. */
	mtk_phy_update_field(base + MIPI_RX_ANA00_CSIXA, RG_CSIXA_BG_CORE_EN, 0);
	mtk_phy_update_field(base + MIPI_RX_ANA00_CSIXA, RG_CSIXA_BG_LPF_EN, 0);

	mtk_phy_update_field(base + CSIXB_OFFSET + MIPI_RX_ANA00_CSIXA, RG_CSIXA_BG_CORE_EN, 0);
	mtk_phy_update_field(base + CSIXB_OFFSET + MIPI_RX_ANA00_CSIXA, RG_CSIXA_BG_LPF_EN, 0);

	return 0;
}

/*
 * MT6785 D-PHY, 4D1C over both halves of a port or 2D1C on the A half.
 */
/*
 * Efuse trimming of the HSRT codes: one word per half with three 5-bit codes,
 * lane 0 pair on top, then lane 1 and lane 2. For CSI1 the codes are in bits
 * 31:17 of both words; other ports have codes in bits 16:2 of a word (CSI0: one
 * word for both halves, A in 31:17, B in 16:2), so the position comes from the
 * optional mediatek,efuse-hsrt-shift property (lane 2 code position, default
 * 17). Applied only when both words are programmed; 2D1C trims only the A half.
 */
static void mtk_mipi_phy_mt6785_efuse(struct mtk_mipi_cdphy_port *port,
				      void __iomem *ana, unsigned int half)
{
	u32 w = port->efuse[half] >> port->efuse_shift[half];
	u32 l0 = (w >> 10) & 0x1f, l1 = (w >> 5) & 0x1f, l2 = w & 0x1f;

	if (!port->efuse[0] || !port->efuse[1])
		return;

	mtk_phy_update_bits(ana + MT6785_ANA08, MT6785_HSRT_CODE_4,
			    l0 | l0 << 8 | l1 << 16 | l1 << 24);
	mtk_phy_update_bits(ana + MT6785_ANA0C, MT6785_HSRT_CODE_2, l2 | l2 << 8);
}

static int mtk_mipi_phy_mt6785_read_efuse(struct mtk_mipi_cdphy_port *port)
{
	static const char *const names[] = { "csi-a", "csi-b" };
	unsigned int i;
	int ret;

	port->efuse_shift[0] = MT6785_EFUSE_HSRT_SHIFT;
	port->efuse_shift[1] = MT6785_EFUSE_HSRT_SHIFT;
	ret = device_property_read_u32_array(port->dev,
					     "mediatek,efuse-hsrt-shift",
					     port->efuse_shift, 2);
	if (ret && ret != -EINVAL)
		return dev_err_probe(port->dev, ret,
				     "mediatek,efuse-hsrt-shift\n");
	for (i = 0; i < ARRAY_SIZE(names); i++)
		if (port->efuse_shift[i] > MT6785_EFUSE_HSRT_SHIFT)
			return dev_err_probe(port->dev, -EINVAL,
					     "efuse shift %u out of range\n",
					     port->efuse_shift[i]);

	for (i = 0; i < ARRAY_SIZE(names); i++) {
		ret = nvmem_cell_read_u32(port->dev, names[i], &port->efuse[i]);
		if (ret == -ENOENT || ret == -EOPNOTSUPP)
			port->efuse[i] = 0;
		else if (ret)
			return dev_err_probe(port->dev, ret, "efuse %s\n", names[i]);
	}
	dev_dbg(port->dev, "efuse %08x %08x\n", port->efuse[0], port->efuse[1]);

	return 0;
}

static void mtk_mipi_phy_mt6785_ana_init(void __iomem *ana)
{
	unsigned int i;

	mtk_phy_update_bits(ana + MT6785_ANA04, MT6785_ANA04_LPRX_VTL_VTH,
			    0x4444);
	mtk_phy_update_field(ana + MT6785_ANA04, MT6785_ANA04_BG_VREF_SEL, 8);

	for (i = 0; i < 3; i++) {
		void __iomem *eq = ana + MT6785_ANA_EQ(i);

		mtk_phy_update_bits(eq, MT6785_EQ_IS, 0x00100010);
		mtk_phy_update_bits(eq, MT6785_EQ_BW, 0x00400040);
		mtk_phy_set_bits(eq, MT6785_EQ_EN);
		mtk_phy_update_bits(eq, MT6785_EQ_DG, 0x03000300);
	}

	mtk_phy_update_bits(ana + MT6785_ANA08, MT6785_HSRT_CODE_4, 0x10101010);
	mtk_phy_update_bits(ana + MT6785_ANA0C, MT6785_HSRT_CODE_2, 0x00001010);
	mtk_phy_update_bits(ana + MT6785_ANA30, MT6785_CDR_CODE, 0x01080808);
	mtk_phy_update_bits(ana + MT6785_ANA3C, MT6785_CDR_CODE, 0x01080808);
	mtk_phy_set_bits(ana + MT6785_ANA28, BIT(0));
	mtk_phy_set_bits(ana + MT6785_ANA34, BIT(0));
}

static void mtk_mipi_phy_mt6785_ana_off(void __iomem *ana)
{
	unsigned int i;

	for (i = 0; i < 3; i++)
		mtk_phy_clear_bits(ana + MT6785_ANA_EQ(i), MT6785_EQ_OS_CAL_EN);
	mtk_phy_clear_bits(ana + MIPI_RX_ANA00_CSIXA, RG_CSIXA_BG_LPF_EN);
	mtk_phy_clear_bits(ana + MIPI_RX_ANA00_CSIXA, RG_CSIXA_BG_CORE_EN);
}

static int mtk_mipi_phy_mt6785_ana_on(struct mtk_mipi_cdphy_port *port,
				      void __iomem *ana)
{
	unsigned int i;
	u32 val;
	int ret;

	mtk_mipi_phy_mt6785_ana_off(ana);
	usleep_range(200, 300);

	mtk_phy_set_bits(ana + MIPI_RX_ANA00_CSIXA, RG_CSIXA_BG_CORE_EN);
	usleep_range(30, 40);
	mtk_phy_set_bits(ana + MIPI_RX_ANA00_CSIXA, RG_CSIXA_BG_LPF_EN);
	udelay(1);
	for (i = 0; i < 3; i++)
		mtk_phy_set_bits(ana + MT6785_ANA_EQ(i), MT6785_EQ_OS_CAL_EN);
	udelay(1);

	/* offset calibration, the vendor waits up to 101 x 30 us */
	ret = readl_poll_timeout(ana + MT6785_ANA48, val,
				 (val & MT6785_ANA48_OS_CAL_DONE) ==
				 MT6785_ANA48_OS_CAL_DONE, 30, 3030);
	if (ret)
		dev_warn(port->dev, "offset calibration timeout, ANA48 %08x\n", val);

	return 0;
}

/*
 * 2D1C on the A half: one or two data lanes, the other half stays off.
 */
static bool mtk_mipi_phy_mt6785_half_a(struct mtk_mipi_cdphy_port *port)
{
	return port->lanes && port->lanes < port->num_lanes;
}

/* 4D1C: D-PHY, clock on A lane 2, all lanes clock-selected */
static void mtk_mipi_phy_mt6785_4d1c_clock(void __iomem *a, void __iomem *b)
{
	mtk_phy_clear_bits(a + MIPI_RX_ANA00_CSIXA, RG_CSI0A_CPHY_EN);
	mtk_phy_clear_bits(b + MIPI_RX_ANA00_CSIXA, RG_CSI0A_CPHY_EN);
	mtk_phy_clear_bits(a + MIPI_RX_ANA00_CSIXA,
			   RG_CSIXA_DPHY_L0_CKMODE_EN | RG_CSIXA_DPHY_L1_CKMODE_EN);
	mtk_phy_set_bits(a + MIPI_RX_ANA00_CSIXA, RG_CSIXA_DPHY_L2_CKMODE_EN);
	mtk_phy_clear_bits(b + MIPI_RX_ANA00_CSIXA,
			   RG_CSIXA_DPHY_L0_CKMODE_EN | RG_CSIXA_DPHY_L1_CKMODE_EN |
			   RG_CSIXA_DPHY_L2_CKMODE_EN);
	mtk_phy_set_bits(a + MIPI_RX_ANA00_CSIXA,
			 RG_CSIXA_DPHY_L0_CKSEL | RG_CSIXA_DPHY_L1_CKSEL |
			 RG_CSIXA_DPHY_L2_CKSEL);
	mtk_phy_set_bits(b + MIPI_RX_ANA00_CSIXA,
			 RG_CSIXA_DPHY_L0_CKSEL | RG_CSIXA_DPHY_L1_CKSEL |
			 RG_CSIXA_DPHY_L2_CKSEL);
}

/* 2D1C on the A half: D-PHY, clock on A lane 1, no clock selection */
static void mtk_mipi_phy_mt6785_2d1c_clock(void __iomem *a)
{
	mtk_phy_clear_bits(a + MIPI_RX_ANA00_CSIXA, RG_CSI0A_CPHY_EN);
	mtk_phy_clear_bits(a + MIPI_RX_ANA00_CSIXA,
			   RG_CSIXA_DPHY_L0_CKMODE_EN | RG_CSIXA_DPHY_L2_CKMODE_EN);
	mtk_phy_set_bits(a + MIPI_RX_ANA00_CSIXA, RG_CSIXA_DPHY_L1_CKMODE_EN);
	mtk_phy_clear_bits(a + MIPI_RX_ANA00_CSIXA,
			   RG_CSIXA_DPHY_L0_CKSEL | RG_CSIXA_DPHY_L1_CKSEL |
			   RG_CSIXA_DPHY_L2_CKSEL);
}

/*
 * Lane mapping (A_L0 = 0, A_L1 = 1, A_L2 = 2, B_L0 = 3, B_L1 = 4):
 * 4D1C: D0 <- A_L1, D1 <- B_L0, D2 <- A_L0, D3 <- B_L1, clock <- A_L2.
 * 2D1C on the A half: D0 <- A_L0, D1 <- A_L2, clock <- A_L1.
 */
static void mtk_mipi_phy_mt6785_lanes(struct mtk_mipi_cdphy_port *port,
				      void __iomem *top)
{
	bool half_a = mtk_mipi_phy_mt6785_half_a(port);
	u32 val;

	val = readl(top + MT6785_DPHY_RX_LANE_SELECT);
	val &= ~(MT6785_DPHY_RX_LC0_SEL | MT6785_DPHY_RX_LD0_SEL |
		 MT6785_DPHY_RX_LD1_SEL | MT6785_DPHY_RX_LD2_SEL |
		 MT6785_DPHY_RX_LD3_SEL);
	if (half_a)
		val |= FIELD_PREP(MT6785_DPHY_RX_LC0_SEL, 1) |
		       FIELD_PREP(MT6785_DPHY_RX_LD0_SEL, 0) |
		       FIELD_PREP(MT6785_DPHY_RX_LD1_SEL, 2);
	else
		val |= FIELD_PREP(MT6785_DPHY_RX_LC0_SEL, 2) |
		       FIELD_PREP(MT6785_DPHY_RX_LD0_SEL, 1) |
		       FIELD_PREP(MT6785_DPHY_RX_LD1_SEL, 3) |
		       FIELD_PREP(MT6785_DPHY_RX_LD2_SEL, 0) |
		       FIELD_PREP(MT6785_DPHY_RX_LD3_SEL, 4);
	writel(val, top + MT6785_DPHY_RX_LANE_SELECT);

	val = readl(top + MT6785_DPHY_RX_LANE_EN);
	val &= ~(MT6785_DPHY_RX_LD_EN | MT6785_DPHY_RX_LC1_EN);
	val |= MT6785_DPHY_RX_LC0_EN;
	if (!half_a)
		val |= MT6785_DPHY_RX_LD_EN;
	else if (port->lanes > 1)
		val |= MT6785_DPHY_RX_LD0_EN | MT6785_DPHY_RX_LD1_EN;
	else
		val |= MT6785_DPHY_RX_LD0_EN;
	writel(val, top + MT6785_DPHY_RX_LANE_EN);
	mtk_phy_set_bits(top + MT6785_DPHY_RX_LANE_SELECT,
			 MT6785_DPHY_RX_CK_DATA_MUX_EN);
}

static int mtk_mipi_phy_mt6785_power_on(struct phy *phy)
{
	struct mtk_mipi_cdphy_port *port = phy_get_drvdata(phy);
	bool half_a = mtk_mipi_phy_mt6785_half_a(port);
	void __iomem *a = port->base;
	void __iomem *b = port->base + CSIXB_OFFSET;
	void __iomem *top = port->base + MT6785_DPHY_TOP_OFFSET;
	unsigned int i;

	mtk_mipi_phy_mt6785_ana_init(a);
	if (!half_a)
		mtk_mipi_phy_mt6785_ana_init(b);
	mtk_mipi_phy_mt6785_efuse(port, a, 0);
	if (!half_a)
		mtk_mipi_phy_mt6785_efuse(port, b, 1);

	/* D-PHY timing: settle 0x20, trail 4, trail enabled on all data lanes */
	for (i = 0; i < 4; i++) {
		void __iomem *hs = top + MT6785_DPHY_RX_DATA_LANE_HS(i);

		mtk_phy_update_field(hs, MT6785_DPHY_RX_HS_SETTLE, 0x20);
		mtk_phy_update_field(hs, MT6785_DPHY_RX_HS_TRAIL, 4);
		mtk_phy_set_bits(hs, MT6785_DPHY_RX_HS_TRAIL_EN);
	}

	if (half_a) {
		mtk_mipi_phy_mt6785_2d1c_clock(a);
		mtk_mipi_phy_mt6785_ana_on(port, a);
	} else {
		mtk_mipi_phy_mt6785_4d1c_clock(a, b);
		mtk_mipi_phy_mt6785_ana_on(port, a);
		mtk_mipi_phy_mt6785_ana_on(port, b);
	}

	mtk_mipi_phy_mt6785_lanes(port, top);

	/* deskew result of this stream only (write one to clear) */
	if (port->lane_rate > MT6785_DESKEW_MIN_RATE)
		mtk_phy_set_bits(top + MT6785_DPHY_RX_DESKEW_STATUS,
				 MT6785_DPHY_RX_DESKEW_DONE);

	dev_dbg(port->dev, "power on %s: ANA00 %08x/%08x ANA48 %08x/%08x LANE_EN %08x LANE_SELECT %08x\n",
		half_a ? "2D1C A" : "4D1C",
		readl(a + MIPI_RX_ANA00_CSIXA), readl(b + MIPI_RX_ANA00_CSIXA),
		readl(a + MT6785_ANA48), readl(b + MT6785_ANA48),
		readl(top + MT6785_DPHY_RX_LANE_EN),
		readl(top + MT6785_DPHY_RX_LANE_SELECT));

	return 0;
}

static int mtk_mipi_phy_mt6785_power_off(struct phy *phy)
{
	struct mtk_mipi_cdphy_port *port = phy_get_drvdata(phy);
	void __iomem *top = port->base + MT6785_DPHY_TOP_OFFSET;

	mtk_phy_clear_bits(top + MT6785_DPHY_RX_LANE_EN,
			   MT6785_DPHY_RX_LD_EN | MT6785_DPHY_RX_LC0_EN |
			   MT6785_DPHY_RX_LC1_EN);
	mtk_mipi_phy_mt6785_ana_off(port->base);
	mtk_mipi_phy_mt6785_ana_off(port->base + CSIXB_OFFSET);

	return 0;
}

/*
 * The receiver passes the lane rate (hs_clk_rate, bit/s per lane, as the
 * Cadence and other CSI-2 receivers do) before powering the PHY on.
 */
static int mtk_mipi_phy_mt6785_configure(struct phy *phy,
					 union phy_configure_opts *opts)
{
	struct mtk_mipi_cdphy_port *port = phy_get_drvdata(phy);
	struct phy_configure_opts_mipi_dphy *dphy = &opts->mipi_dphy;

	/* all lanes of the port (4D1C), or one or two on the A half (2D1C) */
	if (dphy->lanes != port->num_lanes &&
	    (port->num_lanes != 4 || !dphy->lanes || dphy->lanes > 2)) {
		dev_err(port->dev, "%u data lanes not supported (%u, or 1..2 on the A half of 4)\n",
			dphy->lanes, port->num_lanes);
		return -EINVAL;
	}

	port->lanes = dphy->lanes;
	port->lane_rate = dphy->hs_clk_rate;
	dev_dbg(port->dev, "%u lanes (%s), lane rate %llu bit/s%s\n",
		port->lanes, mtk_mipi_phy_mt6785_half_a(port) ? "2D1C A" : "4D1C",
		port->lane_rate,
		port->lane_rate > MT6785_DESKEW_MIN_RATE ? ", deskew check" : "");

	return 0;
}

/*
 * Called by the receiver after the sensor started streaming. Above the
 * deskew rate the D-PHY reports the hardware deskew result per data lane
 * (one bit per lane in each nibble of the low half word). The driver waits
 * up to about 3 ms for a result and continues without one: the result is
 * reported, a missing one is not an error.
 */
static int mtk_mipi_phy_mt6785_calibrate(struct phy *phy)
{
	struct mtk_mipi_cdphy_port *port = phy_get_drvdata(phy);
	void __iomem *top = port->base + MT6785_DPHY_TOP_OFFSET;
	u32 val;
	int ret;

	if (port->lane_rate <= MT6785_DESKEW_MIN_RATE)
		return 0;

	ret = readl_poll_timeout(top + MT6785_DPHY_RX_DESKEW_STATUS, val,
				 val & MT6785_DPHY_RX_DESKEW_DONE, 30, 3030);
	if (ret)
		dev_info(port->dev, "lane rate %llu bit/s: no deskew result (%08x)\n",
			 port->lane_rate, val);
	else
		dev_info(port->dev, "lane rate %llu bit/s: deskew status %04lx\n",
			 port->lane_rate, val & MT6785_DPHY_RX_DESKEW_DONE);

	return 0;
}

static const struct phy_ops mtk_cdphy_mt6785_ops = {
	.power_on	= mtk_mipi_phy_mt6785_power_on,
	.power_off	= mtk_mipi_phy_mt6785_power_off,
	.configure	= mtk_mipi_phy_mt6785_configure,
	.calibrate	= mtk_mipi_phy_mt6785_calibrate,
	.owner		= THIS_MODULE,
};

static struct phy *mtk_mipi_cdphy_xlate(struct device *dev,
					const struct of_phandle_args *args)
{
	struct mtk_mipi_cdphy_port *priv = dev_get_drvdata(dev);

	/*
	 * If PHY is CD-PHY then we need to get the operating mode
	 * For now only D-PHY mode is supported
	 */
	if (priv->type == CDPHY) {
		if (args->args_count != 1) {
			dev_err(dev, "invalid number of arguments\n");
			return ERR_PTR(-EINVAL);
		}
		switch (args->args[0]) {
		case PHY_TYPE_DPHY:
			priv->mode = DPHY;
			if (priv->num_lanes != 4) {
				dev_err(dev, "Only 4D1C mode is supported for now!\n");
				return ERR_PTR(-EINVAL);
			}
			break;
		default:
			dev_err(dev, "Unsupported PHY type: %i\n", args->args[0]);
			return ERR_PTR(-EINVAL);
		}
	} else {
		if (args->args_count) {
			dev_err(dev, "invalid number of arguments\n");
			return ERR_PTR(-EINVAL);
		}
		priv->mode = DPHY;
	}

	return priv->phy;
}

static const struct phy_ops mtk_cdphy_ops = {
	.power_on	= mtk_mipi_phy_power_on,
	.power_off	= mtk_mipi_phy_power_off,
	.owner		= THIS_MODULE,
};

static int mtk_mipi_cdphy_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct phy_provider *phy_provider;
	struct mtk_mipi_cdphy_port *port;
	struct phy *phy;
	int ret;
	u32 phy_type;

	port = devm_kzalloc(dev, sizeof(*port), GFP_KERNEL);
	if (!port)
		return -ENOMEM;

	dev_set_drvdata(dev, port);

	port->dev = dev;

	port->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(port->base))
		return PTR_ERR(port->base);

	ret = of_property_read_u32(dev->of_node, "num-lanes", &port->num_lanes);
	if (ret) {
		dev_err(dev, "Failed to read num-lanes property: %i\n", ret);
		return ret;
	}

	/*
	 * phy-type is optional, if not present, PHY is considered to be CD-PHY
	 */
	if (device_property_present(dev, "phy-type")) {
		ret = of_property_read_u32(dev->of_node, "phy-type", &phy_type);
		if (ret) {
			dev_err(dev, "Failed to read phy-type property: %i\n", ret);
			return ret;
		}
		switch (phy_type) {
		case PHY_TYPE_DPHY:
			port->type = DPHY;
			break;
		default:
			dev_err(dev, "Unsupported PHY type: %i\n", phy_type);
			return -EINVAL;
		}
	} else {
		port->type = CDPHY;
	}

	if (device_get_match_data(dev) == &mtk_cdphy_mt6785_ops) {
		ret = mtk_mipi_phy_mt6785_read_efuse(port);
		if (ret)
			return ret;
	}

	phy = devm_phy_create(dev, NULL, device_get_match_data(dev) ?: &mtk_cdphy_ops);
	if (IS_ERR(phy)) {
		dev_err(dev, "Failed to create PHY: %ld\n", PTR_ERR(phy));
		return PTR_ERR(phy);
	}

	port->phy = phy;
	phy_set_drvdata(phy, port);

	phy_provider = devm_of_phy_provider_register(dev, mtk_mipi_cdphy_xlate);
	if (IS_ERR(phy_provider)) {
		dev_err(dev, "Failed to register PHY provider: %ld\n",
			PTR_ERR(phy_provider));
		return PTR_ERR(phy_provider);
	}

	return 0;
}

static const struct of_device_id mtk_mipi_cdphy_of_match[] = {
	{ .compatible = "mediatek,mt6785-csi-rx", .data = &mtk_cdphy_mt6785_ops },
	{ .compatible = "mediatek,mt8365-csi-rx" },
	{ /* sentinel */},
};
MODULE_DEVICE_TABLE(of, mtk_mipi_cdphy_of_match);

static struct platform_driver mipi_cdphy_pdrv = {
	.probe = mtk_mipi_cdphy_probe,
	.driver	= {
		.name	= "mtk-mipi-csi-0-5",
		.of_match_table = mtk_mipi_cdphy_of_match,
	},
};
module_platform_driver(mipi_cdphy_pdrv);

MODULE_DESCRIPTION("MediaTek MIPI CSI CD-PHY v0.5 Driver");
MODULE_AUTHOR("Louis Kuo <louis.kuo@mediatek.com>");
MODULE_LICENSE("GPL");
