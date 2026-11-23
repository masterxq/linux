// SPDX-License-Identifier: GPL-2.0-only
/*
 * KUnit test of the Bayer orders of the MT6785 DIP: the Bayer phase of each
 * raw format, BGGR jobs against reference hashes of the register image, tile
 * descriptions and shading table, and for the other phases the colour
 * registers, the shading table and the tile headers.
 */

#include <kunit/test.h>
#include <linux/align.h>
#include <linux/array_size.h>
#include <linux/bitfield.h>
#include <linux/string.h>
#include <linux/unaligned.h>

#include "mtk_dip.h"
#include "mtk_dip_regs.h"
#include "mtk_dip_tile.h"

/* FNV-1a, 32 bit */
static u32 mtk_dip_bayer_test_hash(const void *p, size_t n)
{
	const u8 *b = p;
	u32 h = 0x811c9dc5;

	while (n--) {
		h ^= *b++;
		h *= 0x01000193;
	}

	return h;
}

/* params buffer with a different value in every Bayer position */
static void mtk_dip_bayer_test_params(struct kunit *test, struct mtk_dip *dip)
{
	struct v4l2_isp_params_buffer *pb;
	struct mtk_dip_params_blc *blc;
	struct mtk_dip_params_lsc *lsc;
	struct mtk_dip_params_wb *wb;
	struct mtk_dip_buffer *buf;
	unsigned int k, j, i;
	u8 *d;

	buf = kunit_kzalloc(test, sizeof(*buf), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, buf);
	pb = (void *)buf->params;
	pb->version = MTK_DIP_PARAMS_VERSION_V1;
	d = pb->data;

	blc = (void *)d;
	blc->header.type = MTK_DIP_PARAMS_BLOCK_BLC;
	blc->header.size = sizeof(*blc);
	for (k = 0; k < MTK_DIP_BAYER_CHANNELS; k++) {
		blc->level[k] = (60 + 3 * k) << 6;
		blc->gain[k] = 0x1100 + 0x80 * k;
	}
	d += sizeof(*blc);

	wb = (void *)d;
	wb->header.type = MTK_DIP_PARAMS_BLOCK_WB;
	wb->header.size = sizeof(*wb);
	for (k = 0; k < MTK_DIP_BAYER_CHANNELS; k++)
		wb->gain[k] = 0x1000 + 0x300 * k;
	d += sizeof(*wb);

	lsc = (void *)d;
	lsc->header.type = MTK_DIP_PARAMS_BLOCK_LSC;
	lsc->header.size = sizeof(*lsc);
	for (k = 0; k < MTK_DIP_BAYER_CHANNELS; k++)
		for (j = 0; j < MTK_DIP_LSC_POINTS; j++)
			for (i = 0; i < MTK_DIP_LSC_POINTS; i++)
				lsc->gain[k][j][i] = 0x1000 + 0x40 * k +
					24 * ((i - 8) * (i - 8) +
					      (j - 8) * (j - 8)) + 5 * i * k;
	d += sizeof(*lsc);
	pb->data_size = d - pb->data;

	mtk_dip_params_neutral(&dip->par);
	mtk_dip_params_apply(&dip->par, buf);
}

static struct mtk_dip *mtk_dip_bayer_test_dip(struct kunit *test)
{
	struct mtk_dip *dip;

	dip = kunit_kzalloc(test, sizeof(*dip), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, dip);
	dip->ctrl.cpu = kunit_kzalloc(test, MTK_DIP_CTRL_SIZE, GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, dip->ctrl.cpu);
	dip->layout = kunit_kzalloc(test, sizeof(*dip->layout), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, dip->layout);
	mtk_dip_bayer_test_params(test, dip);

	return dip;
}

struct mtk_dip_bayer_test_job {
	u32 fourcc;
	u32 bpp;
	struct v4l2_rect crop;
	u32 profile;
	u32 ctrl_hash;
	u32 lsc_hash;
};

/*
 * Reference hashes of the control buffer (descriptors, register image, tile
 * descriptions) and of the shading table of BGGR jobs.
 */
static const struct mtk_dip_bayer_test_job mtk_dip_bayer_test_bggr[] = {
	{ V4L2_PIX_FMT_MTISP_SBGGR10, 10, { 0, 0, 4624, 3472 },
	  MTK_DIP_PROFILE_LSC | MTK_DIP_PROFILE_EE, 0xfb98ae38, 0x3b7a4e55 },
	{ V4L2_PIX_FMT_SBGGR8, 8, { 1352, 1196, 1920, 1080 }, 0,
	  0x06f59da4, 0x3b7a4e55 },
};

/* Control buffer and shading table of a job on a 4624x3472 frame */
static void mtk_dip_bayer_test_run(struct kunit *test, struct mtk_dip *dip,
				   u32 fourcc, u32 bpp,
				   const struct v4l2_rect *r, u32 profile,
				   u8 *table)
{
	struct v4l2_pix_format_mplane in = {
		.width = 4624, .height = 3472, .pixelformat = fourcc,
		.plane_fmt[0].bytesperline =
			ALIGN(DIV_ROUND_UP(4624 * bpp, 8), 2),
	};
	struct v4l2_pix_format_mplane out = {};
	struct v4l2_rect crop = *r;
	struct mtk_dip_tile_geom g;

	crop.width = mtk_dip_fit_width(crop.width, in.width - crop.left);
	out.width = crop.width;
	out.height = crop.height;
	out.plane_fmt[0].bytesperline = ALIGN(crop.width, 16);
	KUNIT_ASSERT_EQ(test, mtk_dip_job_geometry(&in, &crop, &out, NULL,
						   NULL, NULL, &g), 0);
	KUNIT_EXPECT_EQ(test, g.bpp, bpp);
	g.profile |= profile;

	memset(dip->ctrl.cpu, 0, MTK_DIP_CTRL_SIZE);
	mtk_dip_frame_cfg_init(&dip->frame, g.frame_w, g.frame_h);
	KUNIT_ASSERT_EQ(test, mtk_dip_tile_compute(&g, &dip->frame, dip->layout,
						   dip->ctrl.cpu +
						   MTK_DIP_CTRL_TDR,
						   MTK_DIP_CTRL_TDR_WORDS), 0);
	dip->geom = g;
	mtk_dip_profile_build(dip);
	mtk_dip_profile_params(dip);
	KUNIT_ASSERT_GE(test, mtk_dip_lsc_table(&dip->par.lsc_gain[0][0][0],
						g.pix_id, &dip->frame, table),
			0);
}

/* Bayer phase and bits per pixel of each raw format */
static void mtk_dip_bayer_test_formats(struct kunit *test)
{
	static const struct {
		u32 fourcc;
		u32 bpp;
		u32 pix_id;
	} fmts[] = {
		{ V4L2_PIX_FMT_SBGGR8, 8, MTK_DIP_PIX_ID_BGGR },
		{ V4L2_PIX_FMT_SGBRG8, 8, MTK_DIP_PIX_ID_GBRG },
		{ V4L2_PIX_FMT_SGRBG8, 8, MTK_DIP_PIX_ID_GRBG },
		{ V4L2_PIX_FMT_SRGGB8, 8, MTK_DIP_PIX_ID_RGGB },
		{ V4L2_PIX_FMT_MTISP_SBGGR10, 10, MTK_DIP_PIX_ID_BGGR },
		{ V4L2_PIX_FMT_MTISP_SGBRG10, 10, MTK_DIP_PIX_ID_GBRG },
		{ V4L2_PIX_FMT_MTISP_SGRBG10, 10, MTK_DIP_PIX_ID_GRBG },
		{ V4L2_PIX_FMT_MTISP_SRGGB10, 10, MTK_DIP_PIX_ID_RGGB },
		{ V4L2_PIX_FMT_NV12, 8, 0 },
	};
	struct v4l2_pix_format_mplane in = {
		.width = 1920, .height = 1080,
	};
	struct v4l2_pix_format_mplane out = {
		.width = 1920, .height = 1080,
		.plane_fmt[0].bytesperline = 1920,
	};
	struct v4l2_rect crop = { 0, 0, 1920, 1080 };
	struct mtk_dip_tile_geom g;
	unsigned int i;

	/* the codes the vendor sensor drivers report for the first pixel */
	KUNIT_EXPECT_EQ(test, MTK_DIP_PIX_ID_BGGR, 0);
	KUNIT_EXPECT_EQ(test, MTK_DIP_PIX_ID_GBRG, 1);
	KUNIT_EXPECT_EQ(test, MTK_DIP_PIX_ID_GRBG, 2);
	KUNIT_EXPECT_EQ(test, MTK_DIP_PIX_ID_RGGB, 3);

	for (i = 0; i < ARRAY_SIZE(fmts); i++) {
		in.pixelformat = fmts[i].fourcc;
		in.plane_fmt[0].bytesperline =
			ALIGN(DIV_ROUND_UP(in.width * fmts[i].bpp, 8), 16);
		KUNIT_ASSERT_EQ(test, mtk_dip_job_geometry(&in, &crop, &out,
							   NULL, NULL, NULL,
							   &g), 0);
		KUNIT_EXPECT_EQ_MSG(test, g.pix_id, fmts[i].pix_id,
				    "%p4cc", &fmts[i].fourcc);
		KUNIT_EXPECT_EQ_MSG(test, g.bpp, fmts[i].bpp,
				    "%p4cc", &fmts[i].fourcc);
		KUNIT_EXPECT_EQ(test, mtk_dip_raw_is_10bit(&in),
				fmts[i].bpp == 10);
		if (fmts[i].fourcc != V4L2_PIX_FMT_NV12)
			KUNIT_EXPECT_EQ(test, mtk_dip_tile_check(&g), 0);
	}
}

/* BGGR: the reference control buffer and shading table */
static void mtk_dip_bayer_test_bggr_kept(struct kunit *test)
{
	const struct mtk_dip_bayer_test_job *j;
	struct mtk_dip *dip;
	unsigned int n;
	u8 *table;

	dip = mtk_dip_bayer_test_dip(test);
	table = kunit_kzalloc(test, MTK_DIP_LSC_TABLE_SIZE, GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, table);

	for (n = 0; n < ARRAY_SIZE(mtk_dip_bayer_test_bggr); n++) {
		j = &mtk_dip_bayer_test_bggr[n];
		mtk_dip_bayer_test_run(test, dip, j->fourcc, j->bpp, &j->crop,
				       j->profile, table);
		KUNIT_EXPECT_EQ(test, dip->geom.pix_id, MTK_DIP_PIX_ID_BGGR);
		KUNIT_EXPECT_EQ_MSG(test,
				    mtk_dip_bayer_test_hash(dip->ctrl.cpu,
							    MTK_DIP_CTRL_SIZE),
				    j->ctrl_hash, "job %u", n);
		KUNIT_EXPECT_EQ_MSG(test,
				    mtk_dip_bayer_test_hash(table,
							    MTK_DIP_LSC_TABLE_SIZE),
				    j->lsc_hash, "job %u", n);
	}
}

/* black level register of colour c (B, Gb, Gr, R): B, Gr, Gb, R */
static u32 mtk_dip_bayer_test_blc_reg(unsigned int c)
{
	static const u8 reg[MTK_DIP_BAYER_CHANNELS] = { 0, 2, 1, 3 };

	return reg[c];
}

/* white balance gain of colour c (B, Gb; Gr, R) in the register image */
static u32 mtk_dip_bayer_test_wb(const u32 *img, unsigned int c)
{
	u32 w = img[(c < 2 ? DIP_WB_GAIN_BG : DIP_WB_GAIN_GR) / 4];

	return c & 1 ? FIELD_GET(DIP_FIELD13_HI, w) :
		       FIELD_GET(DIP_FIELD13_LO, w);
}

/*
 * The other phases: colour c of the black level and white balance registers
 * and of the shading table takes the values of the Bayer position c ^ pix_id
 * of the params buffer; the values of that position for BGGR are those of
 * colour c ^ pix_id. The tile descriptions differ in the phase field of the
 * tile headers only.
 */
static void mtk_dip_bayer_test_phases(struct kunit *test)
{
	static const u32 fourcc[] = {
		V4L2_PIX_FMT_MTISP_SBGGR10, V4L2_PIX_FMT_MTISP_SGBRG10,
		V4L2_PIX_FMT_MTISP_SGRBG10, V4L2_PIX_FMT_MTISP_SRGGB10,
	};
	const struct v4l2_rect crop = { 0, 0, 4624, 3472 };
	const u32 profile = MTK_DIP_PROFILE_LSC | MTK_DIP_PROFILE_EE;
	const size_t ch = MTK_DIP_LSC_BLOCK_BYTES / MTK_DIP_BAYER_CHANNELS;
	u32 *img, *ref_img, *tdr, *ref_tdr, d;
	unsigned int p, c, k, b, n;
	u8 *table, *ref_table;
	struct mtk_dip *dip;
	u32 words, tiles;

	dip = mtk_dip_bayer_test_dip(test);
	table = kunit_kzalloc(test, MTK_DIP_LSC_TABLE_SIZE, GFP_KERNEL);
	ref_table = kunit_kzalloc(test, MTK_DIP_LSC_TABLE_SIZE, GFP_KERNEL);
	ref_img = kunit_kzalloc(test, MTK_DIP_CTRL_SIZE, GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, table);
	KUNIT_ASSERT_NOT_NULL(test, ref_table);
	KUNIT_ASSERT_NOT_NULL(test, ref_img);
	img = dip->ctrl.cpu + MTK_DIP_CTRL_IMAGE;
	tdr = dip->ctrl.cpu + MTK_DIP_CTRL_TDR;
	ref_tdr = (void *)ref_img + MTK_DIP_CTRL_TDR;

	mtk_dip_bayer_test_run(test, dip, fourcc[0], 10, &crop, profile,
			       ref_table);
	memcpy(ref_img, dip->ctrl.cpu, MTK_DIP_CTRL_SIZE);
	words = dip->layout->words;
	tiles = dip->layout->n_tiles;
	KUNIT_ASSERT_GT(test, tiles, 1);
	ref_img = (void *)ref_img + MTK_DIP_CTRL_IMAGE;

	for (p = 1; p < ARRAY_SIZE(fourcc); p++) {
		mtk_dip_bayer_test_run(test, dip, fourcc[p], 10, &crop, profile,
				       table);
		KUNIT_ASSERT_EQ(test, dip->geom.pix_id, p);

		for (c = 0; c < MTK_DIP_BAYER_CHANNELS; c++) {
			u32 r = mtk_dip_bayer_test_blc_reg(c);
			u32 q = mtk_dip_bayer_test_blc_reg(c ^ p);

			KUNIT_EXPECT_EQ_MSG(test, img[DIP_BLC_OFFSET(r) / 4],
					    ref_img[DIP_BLC_OFFSET(q) / 4],
					    "phase %u colour %u", p, c);
			KUNIT_EXPECT_EQ_MSG(test, img[DIP_BLC_GAIN(r) / 4],
					    ref_img[DIP_BLC_GAIN(q) / 4],
					    "phase %u colour %u", p, c);
			KUNIT_EXPECT_EQ_MSG(test,
					    mtk_dip_bayer_test_wb(img, c),
					    mtk_dip_bayer_test_wb(ref_img,
								  c ^ p),
					    "phase %u colour %u", p, c);
		}
		/* the rest of the register image is the same */
		for (k = 0; k < (MTK_DIP_CTRL_TDR - MTK_DIP_CTRL_IMAGE) / 4;
		     k++) {
			if (k * 4 >= DIP_BLC_OFFSET(0) &&
			    k * 4 <= DIP_BLC_GAIN(3))
				continue;
			if (k * 4 == DIP_WB_GAIN_BG || k * 4 == DIP_WB_GAIN_GR)
				continue;
			KUNIT_EXPECT_EQ_MSG(test, img[k], ref_img[k],
					    "phase %u register 0x%x", p, 4 * k);
		}

		/* shading: colour c of a block holds position c ^ p */
		for (b = 0; b < MTK_DIP_LSC_TABLE_SIZE /
		     MTK_DIP_LSC_BLOCK_BYTES; b++) {
			const u8 *blk = table + b * MTK_DIP_LSC_BLOCK_BYTES;
			const u8 *ref = ref_table + b * MTK_DIP_LSC_BLOCK_BYTES;

			for (c = 0; c < MTK_DIP_BAYER_CHANNELS; c++)
				KUNIT_EXPECT_MEMEQ_MSG(test, blk + c * ch,
						       ref + (c ^ p) * ch, ch,
						       "phase %u block %u colour %u",
						       p, b, c);
		}
		/*
		 * the gain at the top left corner is c00 of the first block,
		 * the last of its 12 half-words
		 */
		for (c = 0; c < MTK_DIP_BAYER_CHANNELS; c++)
			KUNIT_EXPECT_EQ(test,
					get_unaligned_le16(table + c * ch +
							   ch - 2),
					dip->par.lsc_gain[c ^ p][0][0]);

		/* tiles: the phase in each header, nothing else changes */
		KUNIT_ASSERT_EQ(test, dip->layout->words, words);
		KUNIT_ASSERT_EQ(test, dip->layout->n_tiles, tiles);
		n = 0;
		for (k = 0; k < words * tiles; k++) {
			d = tdr[k] ^ ref_tdr[k];
			if (!d)
				continue;
			n++;
			KUNIT_EXPECT_EQ_MSG(test, k % words, 0,
					    "phase %u word %u", p, k);
			KUNIT_EXPECT_EQ_MSG(test, d, p << 6,
					    "phase %u word %u", p, k);
		}
		KUNIT_EXPECT_EQ(test, n, tiles);
	}
}

static struct kunit_case mtk_dip_bayer_test_cases[] = {
	KUNIT_CASE(mtk_dip_bayer_test_formats),
	KUNIT_CASE(mtk_dip_bayer_test_bggr_kept),
	KUNIT_CASE(mtk_dip_bayer_test_phases),
	{}
};

static struct kunit_suite mtk_dip_bayer_test_suite = {
	.name = "mtk-dip-mt6785-bayer",
	.test_cases = mtk_dip_bayer_test_cases,
};

kunit_test_suite(mtk_dip_bayer_test_suite);
