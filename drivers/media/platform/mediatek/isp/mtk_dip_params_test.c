// SPDX-License-Identifier: GPL-2.0-only
/*
 * KUnit test of the MT6785 DIP parameter translation: neutral values and the
 * encoding of each block, with synthetic values.
 */

#include <kunit/test.h>
#include <linux/string.h>

#include "mtk_dip.h"

/* the parameter state is too large for the stack */
static void mtk_dip_params_test_neutral(struct kunit *test)
{
	struct mtk_dip_param_state *p;
	unsigned int i;

	p = kunit_kzalloc(test, sizeof(*p), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, p);
	mtk_dip_params_neutral(p);
	for (i = 0; i < 4; i++) {
		KUNIT_EXPECT_EQ(test, p->blc[i], 0);
		KUNIT_EXPECT_EQ(test, p->blc[4 + i], 512);
	}
	KUNIT_EXPECT_EQ(test, p->wb[0], 0x02000200);
	KUNIT_EXPECT_EQ(test, p->wb[1], 0x02000200);
	KUNIT_EXPECT_EQ(test, p->ccm[0], 0x200);
	KUNIT_EXPECT_EQ(test, p->ccm[1], 0);
	KUNIT_EXPECT_EQ(test, p->ccm[2], 0x02000000);
	KUNIT_EXPECT_EQ(test, p->ccm[3], 0);
	KUNIT_EXPECT_EQ(test, p->ccm[4], 0);
	KUNIT_EXPECT_EQ(test, p->ccm[5], 0x200);
	/* bypass, identity points: 0, step 2, 4 and 8 in 10-bit units */
	KUNIT_EXPECT_EQ(test, p->gamma_ctl, 0x100307ff);
	KUNIT_EXPECT_EQ(test, p->gamma[0], 0);
	KUNIT_EXPECT_EQ(test, p->gamma[1], 2 | 2 << 10 | 2 << 20);
	KUNIT_EXPECT_EQ(test, p->gamma[64], 128 | 128 << 10 | 128 << 20);
	KUNIT_EXPECT_EQ(test, p->gamma[96], 256 | 256 << 10 | 256 << 20);
	KUNIT_EXPECT_EQ(test, p->gamma[191], 1016 | 1016 << 10 | 1016 << 20);
}

static void *mtk_dip_params_test_buf(struct kunit *test,
				     struct mtk_dip_buffer **buf)
{
	struct v4l2_isp_params_buffer *pb;

	*buf = kunit_kzalloc(test, sizeof(**buf), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, *buf);
	pb = (void *)(*buf)->params;
	pb->version = MTK_DIP_PARAMS_VERSION_V1;

	return pb->data;
}

static void mtk_dip_params_test_blocks(struct kunit *test)
{
	struct mtk_dip_params_gamma *gm;
	struct mtk_dip_params_blc *blc;
	struct mtk_dip_params_ccm *ccm;
	struct mtk_dip_params_wb *wb;
	struct v4l2_isp_params_buffer *pb;
	struct mtk_dip_param_state *p;
	struct mtk_dip_buffer *buf;
	unsigned int i;
	u8 *d;

	p = kunit_kzalloc(test, sizeof(*p), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, p);
	d = mtk_dip_params_test_buf(test, &buf);
	pb = (void *)buf->params;

	blc = (void *)d;
	blc->header.type = MTK_DIP_PARAMS_BLOCK_BLC;
	blc->header.size = sizeof(*blc);
	/* black 64 of 10 bit on B, 0 elsewhere; gains 1.0, 2.0, 0.5, max */
	blc->level[0] = 64 << 6;
	blc->gain[0] = 0x1000;
	blc->gain[1] = 0x2000;
	blc->gain[2] = 0x0800;
	blc->gain[3] = 0xffff;
	d += sizeof(*blc);

	wb = (void *)d;
	wb->header.type = MTK_DIP_PARAMS_BLOCK_WB;
	wb->header.size = sizeof(*wb);
	wb->gain[0] = 0x2000;	/* B 2.0 */
	wb->gain[1] = 0x1000;	/* Gb */
	wb->gain[2] = 0x1000;	/* Gr */
	wb->gain[3] = 0x1800;	/* R 1.5 */
	d += sizeof(*wb);

	ccm = (void *)d;
	ccm->header.type = MTK_DIP_PARAMS_BLOCK_CCM;
	ccm->header.size = sizeof(*ccm);
	ccm->coeff[0][0] = 0x1800;	/* 1.5 */
	ccm->coeff[0][1] = -0x0800;	/* -0.5 */
	ccm->coeff[1][1] = 0x1000;
	ccm->coeff[2][2] = 0x1000;
	ccm->coeff[2][0] = -0x8000;	/* -8.0 */
	d += sizeof(*ccm);

	gm = (void *)d;
	gm->header.type = MTK_DIP_PARAMS_BLOCK_GAMMA;
	gm->header.size = sizeof(*gm);
	for (i = 0; i < MTK_DIP_GAMMA_POINTS; i++) {
		gm->lut[0][i] = i;
		gm->lut[1][i] = 2 * i;
		gm->lut[2][i] = 1023 - i;
	}
	d += sizeof(*gm);
	pb->data_size = d - pb->data;

	mtk_dip_params_neutral(p);
	mtk_dip_params_apply(p, buf);

	/* offset -256 in 12-bit units: 13-bit two's complement 0x1f00 */
	KUNIT_EXPECT_EQ(test, p->blc[0], 0x1f00);
	KUNIT_EXPECT_EQ(test, p->blc[1], 0);
	KUNIT_EXPECT_EQ(test, p->blc[4 + 0], 512);	/* B */
	KUNIT_EXPECT_EQ(test, p->blc[4 + 2], 1024);	/* Gb: register 2 */
	KUNIT_EXPECT_EQ(test, p->blc[4 + 1], 256);	/* Gr: register 1 */
	KUNIT_EXPECT_EQ(test, p->blc[4 + 3], 0x1fff);	/* clamped */
	KUNIT_EXPECT_EQ(test, p->wb[0], 1024 | 512 << 16);
	KUNIT_EXPECT_EQ(test, p->wb[1], 512 | 768 << 16);
	KUNIT_EXPECT_EQ(test, p->ccm[0], 768 | (0x2000 - 256) << 16);
	KUNIT_EXPECT_EQ(test, p->ccm[1], 0);
	KUNIT_EXPECT_EQ(test, p->ccm[2], 512 << 16);
	KUNIT_EXPECT_EQ(test, p->ccm[4], 0x1000);	/* -4096 */
	KUNIT_EXPECT_EQ(test, p->ccm[5], 512);
	KUNIT_EXPECT_EQ(test, p->gamma_ctl, 0x100307fe);
	KUNIT_EXPECT_EQ(test, p->gamma[5], 5 | 10 << 10 | 1018 << 20);

	/* a disabling header returns the block to neutral, others stay */
	d = pb->data;
	memset(d, 0, MTK_DIP_PARAMS_MAX_SIZE - sizeof(*pb));
	((struct v4l2_isp_params_block_header *)d)->type =
		MTK_DIP_PARAMS_BLOCK_GAMMA;
	((struct v4l2_isp_params_block_header *)d)->flags =
		MTK_DIP_PARAMS_BLOCK_FL_DISABLE;
	((struct v4l2_isp_params_block_header *)d)->size =
		sizeof(struct v4l2_isp_params_block_header);
	pb->data_size = sizeof(struct v4l2_isp_params_block_header);
	mtk_dip_params_apply(p, buf);
	KUNIT_EXPECT_EQ(test, p->gamma_ctl, 0x100307ff);
	KUNIT_EXPECT_EQ(test, p->blc[0], 0x1f00);
}

/* The lens shading block keeps the grid; a disabling header turns it off */
static void mtk_dip_params_test_lsc(struct kunit *test)
{
	struct v4l2_isp_params_block_header *h;
	struct v4l2_isp_params_buffer *pb;
	struct mtk_dip_param_state *p;
	struct mtk_dip_params_lsc *lsc;
	struct mtk_dip_buffer *buf;

	p = kunit_kzalloc(test, sizeof(*p), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, p);
	lsc = mtk_dip_params_test_buf(test, &buf);
	pb = (void *)buf->params;

	mtk_dip_params_neutral(p);
	KUNIT_EXPECT_FALSE(test, p->lsc);

	lsc->header.type = MTK_DIP_PARAMS_BLOCK_LSC;
	lsc->header.size = sizeof(*lsc);
	lsc->gain[0][0][0] = 0x3000;
	lsc->gain[3][16][16] = 0x2800;
	lsc->gain[1][8][8] = 0x1000;
	pb->data_size = sizeof(*lsc);
	mtk_dip_params_apply(p, buf);
	KUNIT_EXPECT_TRUE(test, p->lsc);
	KUNIT_EXPECT_TRUE(test, p->lsc_new);
	KUNIT_EXPECT_EQ(test, p->lsc_gain[0][0][0], 0x3000);
	KUNIT_EXPECT_EQ(test, p->lsc_gain[3][16][16], 0x2800);
	KUNIT_EXPECT_EQ(test, p->lsc_gain[1][8][8], 0x1000);
	KUNIT_EXPECT_EQ(test, p->lsc_gain[2][8][8], 0);

	/* a disabling header switches shading off */
	p->lsc_new = false;
	h = &lsc->header;
	h->flags = MTK_DIP_PARAMS_BLOCK_FL_DISABLE;
	h->size = sizeof(*h);
	pb->data_size = sizeof(*h);
	mtk_dip_params_apply(p, buf);
	KUNIT_EXPECT_FALSE(test, p->lsc);
	KUNIT_EXPECT_FALSE(test, p->lsc_new);
}

static struct kunit_case mtk_dip_params_test_cases[] = {
	KUNIT_CASE(mtk_dip_params_test_neutral),
	KUNIT_CASE(mtk_dip_params_test_blocks),
	KUNIT_CASE(mtk_dip_params_test_lsc),
	{}
};

static struct kunit_suite mtk_dip_params_test_suite = {
	.name = "mtk-dip-mt6785-params",
	.test_cases = mtk_dip_params_test_cases,
};

kunit_test_suite(mtk_dip_params_test_suite);
