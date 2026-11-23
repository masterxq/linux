// SPDX-License-Identifier: GPL-2.0-only
/*
 * KUnit tests of the MT6785 MFB driver: tile layout, tile descriptions and
 * frame registers of fixed examples, written out by hand from the rules in
 * mtk_mfb_job.c, and the translation of the parameter blocks.
 */

#include <kunit/test.h>
#include <linux/array_size.h>
#include <linux/slab.h>

#include "mtk_mfb.h"
#include "mtk_mfb_job.h"
#include "mtk_mfb_regs.h"

static const struct mtk_mfb_geom mtk_mfb_full = {
	.w = 4624, .h = 3472, .base_bpl = 4624, .ref_bpl = 4624,
	.out_bpl = 4624, .wt_in_bpl = 4624, .wt_out_bpl = 4624,
};

static const struct mtk_mfb_geom mtk_mfb_small = {
	.w = 256, .h = 64, .base_bpl = 256, .ref_bpl = 256, .out_bpl = 256,
	.wt_out_bpl = 256,
};

static void mtk_mfb_test_layout_full(struct kunit *test)
{
	struct mtk_mfb_layout *l = kunit_kzalloc(test, sizeof(*l), GFP_KERNEL);
	unsigned int k;

	KUNIT_ASSERT_NOT_NULL(test, l);
	KUNIT_ASSERT_EQ(test, mtk_mfb_geom_check(&mtk_mfb_full), 0);
	KUNIT_ASSERT_EQ(test, mtk_mfb_tiles(&mtk_mfb_full, l), 0);
	KUNIT_EXPECT_EQ(test, l->n_tiles, 19);

	KUNIT_EXPECT_EQ(test, l->tile[0].in_xs, 0);
	KUNIT_EXPECT_EQ(test, l->tile[0].in_xe, 255);
	KUNIT_EXPECT_EQ(test, l->tile[0].out_xe, 251);
	KUNIT_EXPECT_EQ(test, l->tile[1].in_xs, 248);
	KUNIT_EXPECT_EQ(test, l->tile[1].in_xe, 503);
	KUNIT_EXPECT_EQ(test, l->tile[1].out_xs, 252);
	KUNIT_EXPECT_EQ(test, l->tile[1].out_xe, 499);
	KUNIT_EXPECT_EQ(test, l->tile[18].in_xs, 4464);
	KUNIT_EXPECT_EQ(test, l->tile[18].in_xe, 4623);
	KUNIT_EXPECT_EQ(test, l->tile[18].out_xs, 4468);
	KUNIT_EXPECT_EQ(test, l->tile[18].out_xe, 4623);

	/* outputs follow each other, core inputs stay on the grid and limit */
	for (k = 0; k < l->n_tiles; k++) {
		const struct mtk_mfb_tile *t = &l->tile[k];

		KUNIT_EXPECT_EQ(test, t->in_xs % 4, 0);
		KUNIT_EXPECT_EQ(test, (t->in_xe + 1) % 4, 0);
		KUNIT_EXPECT_LE(test, t->in_xe - t->in_xs + 1, 256);
		KUNIT_EXPECT_LE(test, t->in_xs, t->out_xs);
		KUNIT_EXPECT_GE(test, t->in_xe, t->out_xe);
		if (k)
			KUNIT_EXPECT_EQ(test, t->out_xs, l->tile[k - 1].out_xe + 1);
	}
}

static void mtk_mfb_test_geom(struct kunit *test)
{
	struct mtk_mfb_geom g = mtk_mfb_small;

	KUNIT_EXPECT_EQ(test, mtk_mfb_geom_check(&g), 0);
	g.w = 258;
	KUNIT_EXPECT_EQ(test, mtk_mfb_geom_check(&g), -EINVAL);
	g = mtk_mfb_small;
	g.out_bpl = 252;
	KUNIT_EXPECT_EQ(test, mtk_mfb_geom_check(&g), -EINVAL);
	g = mtk_mfb_small;
	g.h = 3474;
	KUNIT_EXPECT_EQ(test, mtk_mfb_geom_check(&g), -EINVAL);
}

/* one tile of 256x64 without weight map input */
static const u32 mtk_mfb_small_tdr[MTK_MFB_TDR_WORDS] = {
	0x0027002f, 0x00000000, 0x003f00ff, 0x00000000, 0x003f00ff, 0x00000000,
	0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x003f00ff, 0x00000000,
	0x00000000, 0x003f00ff, 0x00000000, 0x00000000, 0x01000000, 0x00000040,
	0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x00000000, 0x001f00ff,
	0x00000000, 0x001f00ff, 0x00000000, 0x001f00ff, 0x00000000, 0x0000000f,
	0x00ff0000, 0x003f0000, 0x0000000f, 0x00ff0000, 0x003f0000, 0x00400100,
	0x00000000, 0x00000000, 0x00ff0000, 0x003f0000,
};

static void mtk_mfb_test_tdr_small(struct kunit *test)
{
	struct mtk_mfb_layout *l = kunit_kzalloc(test, sizeof(*l), GFP_KERNEL);
	u32 *w = kunit_kzalloc(test, MTK_MFB_TDR_BYTES, GFP_KERNEL);
	unsigned int i;

	KUNIT_ASSERT_NOT_NULL(test, l);
	KUNIT_ASSERT_NOT_NULL(test, w);
	KUNIT_ASSERT_EQ(test, mtk_mfb_tiles(&mtk_mfb_small, l), 0);
	KUNIT_ASSERT_EQ(test, l->n_tiles, 1);
	mtk_mfb_tdr(&mtk_mfb_small, l, w);
	for (i = 0; i < MTK_MFB_TDR_WORDS; i++)
		KUNIT_EXPECT_EQ_MSG(test, w[i], mtk_mfb_small_tdr[i], "word %u", i);
}

static void mtk_mfb_test_tdr_full(struct kunit *test)
{
	struct mtk_mfb_layout *l = kunit_kzalloc(test, sizeof(*l), GFP_KERNEL);
	u32 *w = kunit_kzalloc(test, MTK_MFB_TDR_BYTES, GFP_KERNEL);
	u32 *t;

	KUNIT_ASSERT_NOT_NULL(test, l);
	KUNIT_ASSERT_NOT_NULL(test, w);
	KUNIT_ASSERT_EQ(test, mtk_mfb_tiles(&mtk_mfb_full, l), 0);
	mtk_mfb_tdr(&mtk_mfb_full, l, w);

	/* first tile: left edge, no end interrupt */
	KUNIT_EXPECT_EQ(test, w[0], 0x0027000d);
	/* second tile: inner, core input 248..503 */
	t = w + MTK_MFB_TDR_WORDS;
	KUNIT_EXPECT_EQ(test, t[0], 0x0027000c);
	KUNIT_EXPECT_EQ(test, t[1], 248);
	KUNIT_EXPECT_EQ(test, t[2], 0x0d8f00ff);
	/* weight map input: the output window 252..499 */
	KUNIT_EXPECT_EQ(test, t[5], 252);
	KUNIT_EXPECT_EQ(test, t[6], 0x0d8f00f7);
	KUNIT_EXPECT_EQ(test, t[9], 252);
	KUNIT_EXPECT_EQ(test, t[10], 0x0d8f00f7);
	KUNIT_EXPECT_EQ(test, t[15], 248 & 15);
	KUNIT_EXPECT_EQ(test, t[23], 0x06c700ff);
	KUNIT_EXPECT_EQ(test, t[27], 0x06c700f7);
	KUNIT_EXPECT_EQ(test, t[35], 0x0d9000f8);
	/* last tile: right edge and end interrupt */
	t = w + 18 * MTK_MFB_TDR_WORDS;
	KUNIT_EXPECT_EQ(test, t[0], 0x0027002e);
	KUNIT_EXPECT_EQ(test, t[16], 160 << 16);
}

static void mtk_mfb_test_regs(struct kunit *test)
{
	static const struct mtk_mfb_addrs a = {
		.base = 0x50000000, .base_uv = 0x50010000, .ref = 0x51000000,
		.ref_uv = 0x51010000, .out = 0x52000000, .out_uv = 0x52010000,
		.wt_out = 0x53000000, .tdr = 0x54000000,
	};
	struct mtk_mfb_reg *r = kunit_kcalloc(test, MTK_MFB_REG_MAX, sizeof(*r),
					      GFP_KERNEL);
	struct mtk_mfb_tuning t = { .ll = { 0x11 }, .mbd = { [10] = 0x22 } };
	unsigned int n;

	KUNIT_ASSERT_NOT_NULL(test, r);
	n = mtk_mfb_frame_regs(&mtk_mfb_small, &a, &t, r);
	KUNIT_ASSERT_EQ(test, n, 87);

	KUNIT_EXPECT_EQ(test, r[0].off, MFB_INT_CTL);
	KUNIT_EXPECT_EQ(test, r[1].val, 0xf);
	KUNIT_EXPECT_EQ(test, r[2].val, 0x1f3);
	/* MFBI: address, stride with format bits, x size, y size, FIFO */
	KUNIT_EXPECT_EQ(test, r[3].off, MFB_MFBI + MFB_PORT_BASE_ADDR);
	KUNIT_EXPECT_EQ(test, r[4].val, 0x00300100);
	KUNIT_EXPECT_EQ(test, r[5].val, 0xff);
	KUNIT_EXPECT_EQ(test, r[6].val, 63);
	KUNIT_EXPECT_EQ(test, r[7].val, 0x80000040);
	/* MFB3I unused: x size 0 - 1 as the vendor driver writes it */
	KUNIT_EXPECT_EQ(test, r[23].off, MFB_MFB3I + MFB_PORT_BASE_ADDR);
	KUNIT_EXPECT_EQ(test, r[25].val, 0xffffffff);
	/* MFBO_B: half height, small FIFO */
	KUNIT_EXPECT_EQ(test, r[41].off, MFB_MFBO_B + MFB_PORT_YSIZE);
	KUNIT_EXPECT_EQ(test, r[41].val, 31);
	KUNIT_EXPECT_EQ(test, r[42].val, 0x80000020);
	KUNIT_EXPECT_EQ(test, r[48].off, MFB_TDRI_BASE_ADDR);
	KUNIT_EXPECT_EQ(test, r[49].val, 159);
	KUNIT_EXPECT_EQ(test, r[63].off, MFB_CRSP_STEP_OFST);
	KUNIT_EXPECT_EQ(test, r[64].off, MFB_CON);
	KUNIT_EXPECT_EQ(test, r[64].val, 0);
	KUNIT_EXPECT_EQ(test, r[65].val, 0x11);
	KUNIT_EXPECT_EQ(test, r[86].off, MFB_MBD_CON0 + 40);
	KUNIT_EXPECT_EQ(test, r[86].val, 0x22);
}

static void mtk_mfb_test_regs_wt_in(struct kunit *test)
{
	struct mtk_mfb_addrs a = { .wt_in = 0x55000000 };
	struct mtk_mfb_geom g = mtk_mfb_small;
	struct mtk_mfb_reg *r = kunit_kcalloc(test, MTK_MFB_REG_MAX, sizeof(*r),
					      GFP_KERNEL);
	struct mtk_mfb_tuning t = {};

	KUNIT_ASSERT_NOT_NULL(test, r);
	g.wt_in_bpl = 256;
	mtk_mfb_frame_regs(&g, &a, &t, r);
	KUNIT_EXPECT_EQ(test, r[2].val, 0x1f7);
	KUNIT_EXPECT_EQ(test, r[23].val, 0x55000000);
	KUNIT_EXPECT_EQ(test, r[24].val, 256);
	KUNIT_EXPECT_EQ(test, r[64].val, 1);
}

static void mtk_mfb_test_params(struct kunit *test)
{
	struct mtk_mfb_buffer *buf = kunit_kzalloc(test, sizeof(*buf), GFP_KERNEL);
	struct v4l2_isp_params_buffer *p;
	struct mtk_mfb_params_blend *b;
	struct mtk_mfb_params_extra *e;
	struct mtk_mfb_tuning t = {};
	unsigned int i;

	KUNIT_ASSERT_NOT_NULL(test, buf);
	p = (void *)buf->params;
	p->version = MTK_MFB_PARAMS_VERSION_V1;
	p->data_size = sizeof(*b) + sizeof(*e);
	b = (void *)p->data;
	e = (void *)(p->data + sizeof(*b));
	b->header.type = MTK_MFB_PARAMS_BLOCK_BLEND;
	b->header.size = sizeof(*b);
	b->ll_con1 = 1;
	b->ll_con2 = 2;
	for (i = 0; i < MTK_MFB_MBD_CON_WORDS; i++)
		b->mbd_con[i] = 0x30 + i;
	e->header.type = MTK_MFB_PARAMS_BLOCK_EXTRA;
	e->header.size = sizeof(*e);
	e->con = 0x70;
	for (i = 0; i < MTK_MFB_LL_CON_EXTRA_WORDS; i++)
		e->ll_con[i] = 3 + i;
	e->edge = 0x14;

	KUNIT_EXPECT_EQ(test, sizeof(*b), 64);
	KUNIT_EXPECT_EQ(test, sizeof(*e), 48);
	mtk_mfb_params_apply(&t, buf);
	KUNIT_EXPECT_EQ(test, t.con, 0x70);
	for (i = 0; i < MTK_MFB_LL_WORDS; i++)
		KUNIT_EXPECT_EQ(test, t.ll[i], i + 1);
	KUNIT_EXPECT_EQ(test, t.edge, 0x14);
	KUNIT_EXPECT_EQ(test, t.mbd[10], 0x3a);

	/* a disabled block sets its words to 0, the other block stays */
	b->header.flags = V4L2_ISP_PARAMS_FL_BLOCK_DISABLE;
	p->data_size = sizeof(*b);
	mtk_mfb_params_apply(&t, buf);
	KUNIT_EXPECT_EQ(test, t.ll[0], 0);
	KUNIT_EXPECT_EQ(test, t.mbd[10], 0);
	KUNIT_EXPECT_EQ(test, t.ll[2], 3);
}

static struct kunit_case mtk_mfb_job_test_cases[] = {
	KUNIT_CASE(mtk_mfb_test_layout_full),
	KUNIT_CASE(mtk_mfb_test_geom),
	KUNIT_CASE(mtk_mfb_test_tdr_small),
	KUNIT_CASE(mtk_mfb_test_tdr_full),
	KUNIT_CASE(mtk_mfb_test_regs),
	KUNIT_CASE(mtk_mfb_test_regs_wt_in),
	KUNIT_CASE(mtk_mfb_test_params),
	{}
};

static struct kunit_suite mtk_mfb_job_test_suite = {
	.name = "mtk-mfb-mt6785-job",
	.test_cases = mtk_mfb_job_test_cases,
};

kunit_test_suite(mtk_mfb_job_test_suite);
