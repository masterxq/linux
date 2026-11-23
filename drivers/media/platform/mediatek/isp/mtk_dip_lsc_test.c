// SPDX-License-Identifier: GPL-2.0-only
/*
 * KUnit test of the MT6785 DIP lens shading table.
 *
 * The references were computed by an independent implementation of the
 * same integer spline and patch rules; they are a checksum of the whole
 * table and the words of a few blocks. The corner gains are synthetic: a
 * smooth surface with a different shape per Bayer position, and alternating
 * 0 / 65535 corners whose coefficients exceed the stored range.
 */

#include <kunit/test.h>
#include <linux/array_size.h>
#include <linux/slab.h>
#include <linux/unaligned.h>

#include "mtk_dip_tile.h"

#define MTK_DIP_LSC_GAINS	(4 * MTK_DIP_LSC_CORNERS * MTK_DIP_LSC_CORNERS)

struct mtk_dip_lsc_sample {
	u8 row;
	u8 col;
	u8 pos;
	u16 hw[12];
};

struct mtk_dip_lsc_ref {
	const char *name;
	u32 frame_w;
	u32 frame_h;
	bool steps;
	int clamped;
	u32 fnv;
	struct mtk_dip_lsc_sample s[4];
};

static const struct mtk_dip_lsc_ref mtk_dip_lsc_refs[] = {
	{ "full", 4624, 3472, 0, 0, 0x10fc90dd,
	  { { 0, 0, 0, { 0x0000, 0x0000, 0x0008, 0x0000, 0x0000, 0x0005,
			 0x0000, 0x0009, 0x0000, 0x811e, 0x80b2, 0x1f00 } },
	    { 7, 8, 1, { 0x81c3, 0x81b6, 0x00f7, 0x02c9, 0x02bb, 0x00ec,
			 0x815f, 0x8104, 0x815f, 0x002f, 0x8025, 0x133d } },
	    { 15, 15, 3, { 0x0001, 0x0001, 0x805d, 0x8002, 0x8003, 0x8061,
			 0x0115, 0x0025, 0x0121, 0x0106, 0x805f, 0x2295 } },
	    { 3, 12, 2, { 0x001d, 0x001f, 0x00ac, 0x8011, 0x00b3, 0x8073,
			 0x80e9, 0x80c4, 0x0055, 0x009f, 0x804b, 0x1dfe } } },
	},
	{ "binned", 2304, 1728, 0, 0, 0x293021e5,
	  { { 0, 0, 0, { 0x0000, 0x0000, 0x0008, 0x0000, 0x0000, 0x0005,
			 0x0000, 0x0009, 0x0000, 0x811e, 0x80b2, 0x1f00 } },
	    { 7, 8, 1, { 0x81c3, 0x81b6, 0x00f7, 0x02c9, 0x02bb, 0x00ec,
			 0x815f, 0x8104, 0x815f, 0x002e, 0x8025, 0x133d } },
	    { 15, 15, 3, { 0x0000, 0x0000, 0x805c, 0x0000, 0x8001, 0x8059,
			 0x0113, 0x0022, 0x010a, 0x0107, 0x804f, 0x2295 } },
	    { 3, 12, 2, { 0x001e, 0x001f, 0x00ad, 0x8012, 0x00b3, 0x8073,
			 0x80e9, 0x80c4, 0x0055, 0x009e, 0x804b, 0x1dfe } } },
	},
	{ "smallest", 128, 64, 0, 0, 0x235ff0e6,
	  { { 0, 0, 0, { 0x0000, 0x0000, 0x0008, 0x0000, 0x0000, 0x0005,
			 0x0000, 0x0009, 0x0000, 0x811e, 0x80b2, 0x1f00 } },
	    { 7, 8, 1, { 0x81c3, 0x81b6, 0x00f7, 0x02c9, 0x02bb, 0x00ec,
			 0x815f, 0x8104, 0x815f, 0x002e, 0x8025, 0x133d } },
	    { 15, 15, 3, { 0x0000, 0x0000, 0x805c, 0x0000, 0x8001, 0x8059,
			 0x0113, 0x0022, 0x010a, 0x0107, 0x804f, 0x2295 } },
	    { 3, 12, 2, { 0x001e, 0x001f, 0x00ad, 0x8012, 0x00b3, 0x8073,
			 0x80e9, 0x80c4, 0x0055, 0x009e, 0x804b, 0x1dfe } } },
	},
	{ "steps", 4624, 3472, 1, 8812, 0x29f3e005,
	  { { 0, 0, 0, { 0x7fff, 0x7fff, 0xddb4, 0x0000, 0x0000, 0xddb4,
			 0x0000, 0xffff, 0x0000, 0x7fff, 0x7fff, 0x0000 } },
	    { 7, 8, 1, { 0x7fff, 0x7fff, 0xffff, 0xffff, 0xffff, 0xffff,
			 0x7fff, 0x7fff, 0x7fff, 0x0000, 0x0006, 0x0000 } },
	    { 15, 15, 3, { 0xffff, 0xffff, 0x636e, 0x7fff, 0x7fff, 0x6559,
			 0xffff, 0xffff, 0xffff, 0x46dd, 0x4ab1, 0xffff } },
	    { 3, 12, 2, { 0xffff, 0xffff, 0x7fff, 0x7fff, 0x7fff, 0x7fff,
			 0xffff, 0xffff, 0xffff, 0x014b, 0x8444, 0xffff } } },
	},
};

/* corner k, j, i: Bayer position, row, column */
static void mtk_dip_lsc_test_gains(u16 *gain, bool steps)
{
	int k, j, i, dx, dy;

	for (k = 0; k < 4; k++) {
		for (j = 0; j < MTK_DIP_LSC_CORNERS; j++) {
			for (i = 0; i < MTK_DIP_LSC_CORNERS; i++) {
				dx = i - 8;
				dy = j - 8;
				*gain++ = steps ? ((i + j + k) & 1 ? 0xffff : 0) :
					  4096 + 37 * dx * dx + 23 * dy * dy +
					  101 * k * i - 61 * k * j +
					  17 * ((i * j * (k + 1)) % 29);
			}
		}
	}
}

static u32 mtk_dip_lsc_test_fnv(const u8 *b, size_t n)
{
	u32 h = 0x811c9dc5;

	while (n--)
		h = (h ^ *b++) * 0x01000193;

	return h;
}

static void mtk_dip_lsc_test_refs(struct kunit *test)
{
	struct mtk_dip_frame_cfg f;
	unsigned int i, s, w;
	u16 *gain;
	u8 *tab;
	int ret;

	gain = kunit_kcalloc(test, MTK_DIP_LSC_GAINS, sizeof(*gain),
			     GFP_KERNEL);
	tab = kunit_kzalloc(test, MTK_DIP_LSC_TABLE_SIZE, GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, gain);
	KUNIT_ASSERT_NOT_NULL(test, tab);

	for (i = 0; i < ARRAY_SIZE(mtk_dip_lsc_refs); i++) {
		const struct mtk_dip_lsc_ref *r = &mtk_dip_lsc_refs[i];

		mtk_dip_lsc_test_gains(gain, r->steps);
		mtk_dip_frame_cfg_init(&f, r->frame_w, r->frame_h);
		ret = mtk_dip_lsc_table(gain, MTK_DIP_PIX_ID_BGGR, &f, tab);
		KUNIT_EXPECT_EQ_MSG(test, ret, r->clamped, "%s", r->name);
		KUNIT_EXPECT_EQ_MSG(test, mtk_dip_lsc_test_fnv(tab, MTK_DIP_LSC_TABLE_SIZE),
				    r->fnv, "%s", r->name);
		for (s = 0; s < ARRAY_SIZE(r->s); s++) {
			const struct mtk_dip_lsc_sample *p = &r->s[s];
			const u8 *b = tab + p->row * f.lsc_stride +
				      p->col * MTK_DIP_LSC_BLOCK_BYTES + p->pos * 24;

			for (w = 0; w < ARRAY_SIZE(p->hw); w++)
				KUNIT_EXPECT_EQ_MSG(test, get_unaligned_le16(b + 2 * w),
						    p->hw[w], "%s block %u,%u pos %u word %u",
						    r->name, p->row, p->col, p->pos, w);
		}
	}
}

/* Constant gains give constant blocks: only the constant term is set */
static void mtk_dip_lsc_test_flat(struct kunit *test)
{
	struct mtk_dip_frame_cfg f;
	unsigned int i;
	u16 *gain;
	u8 *tab;

	gain = kunit_kcalloc(test, MTK_DIP_LSC_GAINS, sizeof(*gain),
			     GFP_KERNEL);
	tab = kunit_kzalloc(test, MTK_DIP_LSC_TABLE_SIZE, GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, gain);
	KUNIT_ASSERT_NOT_NULL(test, tab);

	for (i = 0; i < MTK_DIP_LSC_GAINS; i++)
		gain[i] = 0x1000;
	mtk_dip_frame_cfg_init(&f, 4624, 3472);
	KUNIT_EXPECT_EQ(test, mtk_dip_lsc_table(gain, MTK_DIP_PIX_ID_BGGR, &f, tab), 0);
	for (i = 0; i < MTK_DIP_LSC_TABLE_SIZE / 2; i++)
		KUNIT_EXPECT_EQ_MSG(test, get_unaligned_le16(tab + 2 * i),
				    i % 12 == 11 ? 0x1000 : 0, "half-word %u", i);

	/* a grid the frame is too small for */
	mtk_dip_frame_cfg_init(&f, 30, 30);
	KUNIT_EXPECT_EQ(test, mtk_dip_lsc_table(gain, MTK_DIP_PIX_ID_BGGR, &f, tab), -EINVAL);
}

static struct kunit_case mtk_dip_lsc_test_cases[] = {
	KUNIT_CASE(mtk_dip_lsc_test_refs),
	KUNIT_CASE(mtk_dip_lsc_test_flat),
	{}
};

static struct kunit_suite mtk_dip_lsc_test_suite = {
	.name = "mtk-dip-mt6785-lsc",
	.test_cases = mtk_dip_lsc_test_cases,
};

kunit_test_suite(mtk_dip_lsc_test_suite);
