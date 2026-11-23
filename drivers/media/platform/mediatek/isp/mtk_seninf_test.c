// SPDX-License-Identifier: GPL-2.0
/*
 * KUnit tests of the SENINF stream state: routing and formats stay with a
 * running stream, an input with two sources links one at a time. Included
 * from mtk_seninf.c (static functions).
 */

#include <kunit/device.h>
#include <kunit/test.h>

/*
 * MT6785: sink pads 0..3, source pads 4..7. The default routing is the
 * fixed table of the SoC, in the order of the sink pads: 0 -> 7, 1 -> 4,
 * 2 -> 5. The running stream of the tests is 2 -> 5, the other one 1 -> 4.
 */
#define SENINF_TEST_SINK_RUN	2U
#define SENINF_TEST_SRC_RUN	5U
#define SENINF_TEST_SINK_OTHER	1U
#define SENINF_TEST_SRC_OTHER	4U
#define SENINF_TEST_ROUTE_RUN	2U
#define SENINF_TEST_ROUTE_OTHER	1U

static void mtk_seninf_test_cleanup(void *data)
{
	v4l2_subdev_cleanup(data);
}

static struct mtk_seninf *mtk_seninf_test_alloc(struct kunit *test)
{
	struct mtk_seninf *priv;
	struct v4l2_subdev *sd;
	unsigned int i, num_pads;

	priv = kunit_kzalloc(test, sizeof(*priv), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, priv);
	priv->dev = kunit_device_register(test, "mtk-seninf-test");
	KUNIT_ASSERT_FALSE(test, IS_ERR(priv->dev));
	priv->conf = &seninf_6785_conf;

	num_pads = priv->conf->nb_inputs + priv->conf->nb_outputs;
	for (i = 0; i < num_pads; i++)
		priv->pads[i].flags = i < priv->conf->nb_inputs ?
				      MEDIA_PAD_FL_SINK : MEDIA_PAD_FL_SOURCE;

	sd = &priv->subdev;
	KUNIT_ASSERT_EQ(test, media_entity_pads_init(&sd->entity, num_pads,
						     priv->pads), 0);
	v4l2_subdev_init(sd, &seninf_subdev_ops);
	sd->internal_ops = &seninf_subdev_internal_ops;
	sd->dev = priv->dev;
	sd->flags |= V4L2_SUBDEV_FL_STREAMS;
	strscpy(sd->name, "mtk-seninf-test", sizeof(sd->name));
	KUNIT_ASSERT_EQ(test, v4l2_subdev_init_finalize(sd), 0);
	KUNIT_ASSERT_EQ(test, kunit_add_action_or_reset(test,
							mtk_seninf_test_cleanup,
							sd), 0);

	return priv;
}

/* A copy of the routes of @state, as user space reads and sets them */
static void mtk_seninf_test_copy(struct kunit *test,
				 struct v4l2_subdev_state *state,
				 struct v4l2_subdev_krouting *routing)
{
	size_t size = state->routing.num_routes * sizeof(*routing->routes);

	routing->routes = kunit_kzalloc(test, size, GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, routing->routes);
	memcpy(routing->routes, state->routing.routes, size);
	routing->num_routes = state->routing.num_routes;
	routing->len_routes = state->routing.num_routes;
}

static void mtk_seninf_test_routing(struct kunit *test)
{
	struct mtk_seninf *priv = mtk_seninf_test_alloc(test);
	struct v4l2_subdev *sd = &priv->subdev;
	struct v4l2_subdev_krouting same, other;
	struct v4l2_mbus_framefmt *fmt, *fmt_run;
	struct v4l2_subdev_state *state;
	u32 tmp;

	state = v4l2_subdev_lock_and_get_active_state(sd);
	KUNIT_ASSERT_EQ(test, state->routing.num_routes, 3U);
	KUNIT_ASSERT_EQ(test, state->routing.routes[SENINF_TEST_ROUTE_RUN].sink_pad,
			SENINF_TEST_SINK_RUN);
	KUNIT_ASSERT_EQ(test, state->routing.routes[SENINF_TEST_ROUTE_OTHER].sink_pad,
			SENINF_TEST_SINK_OTHER);
	KUNIT_EXPECT_EQ(test, v4l2_subdev_routing_find_opposite_end(&state->routing,
								    SENINF_TEST_SRC_RUN, 0,
								    &tmp, NULL), 0);
	KUNIT_EXPECT_EQ(test, tmp, SENINF_TEST_SINK_RUN);

	mtk_seninf_test_copy(test, state, &same);
	mtk_seninf_test_copy(test, state, &other);
	/* routes 1 -> 4 and 2 -> 5 become 2 -> 4 and 1 -> 5 */
	swap(other.routes[SENINF_TEST_ROUTE_OTHER].sink_pad,
	     other.routes[SENINF_TEST_ROUTE_RUN].sink_pad);
	/* the same table in another order is the same routing */
	swap(same.routes[0], same.routes[2]);

	fmt_run = v4l2_subdev_state_get_format(state, SENINF_TEST_SRC_RUN, 0);
	KUNIT_ASSERT_NOT_NULL(test, fmt_run);
	fmt_run->width = 4624;
	fmt_run->height = 3472;
	/* what a successful seninf_enable_streams() leaves */
	priv->streaming = BIT(SENINF_TEST_SRC_RUN);

	/*
	 * Running: the same table keeps the stream configurations (with the
	 * enabled flags of the core) as they are, another one is refused.
	 */
	KUNIT_EXPECT_EQ(test, seninf_set_routing(sd, state, V4L2_SUBDEV_FORMAT_ACTIVE,
						 &same), 0);
	fmt = v4l2_subdev_state_get_format(state, SENINF_TEST_SRC_RUN, 0);
	KUNIT_EXPECT_PTR_EQ(test, fmt, fmt_run);
	KUNIT_EXPECT_EQ(test, fmt_run->width, 4624U);
	KUNIT_EXPECT_EQ(test, fmt_run->height, 3472U);

	KUNIT_EXPECT_EQ(test, seninf_set_routing(sd, state, V4L2_SUBDEV_FORMAT_ACTIVE,
						 &other), -EBUSY);
	fmt = v4l2_subdev_state_get_format(state, SENINF_TEST_SRC_RUN, 0);
	KUNIT_EXPECT_PTR_EQ(test, fmt, fmt_run);
	KUNIT_EXPECT_EQ(test, v4l2_subdev_routing_find_opposite_end(&state->routing,
								    SENINF_TEST_SRC_OTHER, 0,
								    &tmp, NULL), 0);
	KUNIT_EXPECT_EQ(test, tmp, SENINF_TEST_SINK_OTHER);

	/* stopped: a new table applies and resets the formats */
	priv->streaming = 0;
	KUNIT_EXPECT_EQ(test, seninf_set_routing(sd, state, V4L2_SUBDEV_FORMAT_ACTIVE,
						 &other), 0);
	KUNIT_EXPECT_EQ(test, v4l2_subdev_routing_find_opposite_end(&state->routing,
								    SENINF_TEST_SRC_OTHER, 0,
								    &tmp, NULL), 0);
	KUNIT_EXPECT_EQ(test, tmp, SENINF_TEST_SINK_RUN);
	fmt = v4l2_subdev_state_get_format(state, SENINF_TEST_SRC_RUN, 0);
	KUNIT_ASSERT_NOT_NULL(test, fmt);
	KUNIT_EXPECT_EQ(test, fmt->width, (u32)SENINF_DEFAULT_WIDTH);

	v4l2_subdev_unlock_state(state);
}

static void mtk_seninf_test_format(struct kunit *test)
{
	struct mtk_seninf *priv = mtk_seninf_test_alloc(test);
	struct v4l2_subdev *sd = &priv->subdev;
	struct v4l2_subdev_format fmt = {
		.which = V4L2_SUBDEV_FORMAT_ACTIVE,
		.format = {
			.code = MEDIA_BUS_FMT_SBGGR10_1X10,
			.width = 1920,
			.height = 1080,
		},
	};
	struct v4l2_subdev_state *state;

	state = v4l2_subdev_lock_and_get_active_state(sd);
	priv->streaming = BIT(SENINF_TEST_SRC_RUN);

	/* both ends of the running stream keep their format */
	fmt.pad = SENINF_TEST_SINK_RUN;
	KUNIT_EXPECT_EQ(test, seninf_set_fmt(sd, state, &fmt), -EBUSY);
	fmt.pad = SENINF_TEST_SRC_RUN;
	KUNIT_EXPECT_EQ(test, seninf_set_fmt(sd, state, &fmt), -EBUSY);
	KUNIT_EXPECT_EQ(test, v4l2_subdev_state_get_format(state, SENINF_TEST_SINK_RUN,
							   0)->width, (u32)SENINF_DEFAULT_WIDTH);

	/* the other stream can be configured, the format goes to its source */
	fmt.pad = SENINF_TEST_SINK_OTHER;
	KUNIT_EXPECT_EQ(test, seninf_set_fmt(sd, state, &fmt), 0);
	KUNIT_EXPECT_EQ(test, v4l2_subdev_state_get_format(state, SENINF_TEST_SRC_OTHER,
							   0)->width, 1920U);

	priv->streaming = 0;
	fmt.pad = SENINF_TEST_SINK_RUN;
	KUNIT_EXPECT_EQ(test, seninf_set_fmt(sd, state, &fmt), 0);
	KUNIT_EXPECT_EQ(test, v4l2_subdev_state_get_format(state,
							   SENINF_TEST_SRC_RUN,
							   0)->width, 1920U);

	v4l2_subdev_unlock_state(state);
}

/*
 * The default routes of the MT6785 do not depend on the connected ports: CSI1
 * and CSI2 keep their outputs when a sensor appears on CSI0, which has an
 * output of its own; port 6 stays free for the phase detection stream, CSI0B
 * has no route.
 */
static void mtk_seninf_test_default_routes(struct kunit *test)
{
	static const struct {
		u32 sink;
		u32 source;
	} expected[] = {
		{ CSI_PORT_0, 7 },
		{ CSI_PORT_1, 4 },
		{ CSI_PORT_2, 5 },
	};
	struct mtk_seninf *priv = mtk_seninf_test_alloc(test);
	struct v4l2_subdev *sd = &priv->subdev;
	struct v4l2_subdev_state *state;
	unsigned int i;
	u32 tmp;

	state = v4l2_subdev_lock_and_get_active_state(sd);
	KUNIT_EXPECT_EQ(test, state->routing.num_routes, (u32)ARRAY_SIZE(expected));
	for (i = 0; i < ARRAY_SIZE(expected); i++) {
		KUNIT_EXPECT_EQ(test,
				v4l2_subdev_routing_find_opposite_end(&state->routing,
								      expected[i].source, 0,
								      &tmp, NULL), 0);
		KUNIT_EXPECT_EQ(test, tmp, expected[i].sink);
	}
	KUNIT_EXPECT_NE(test, v4l2_subdev_routing_find_opposite_end(&state->routing, 6, 0,
								    &tmp, NULL), 0);
	KUNIT_EXPECT_NE(test, v4l2_subdev_routing_find_opposite_end(&state->routing,
								    CSI_PORT_0B, 0,
								    &tmp, NULL), 0);
	/* each routed sink pad has a format to configure */
	KUNIT_EXPECT_NOT_NULL(test, v4l2_subdev_state_get_format(state, CSI_PORT_0, 0));
	KUNIT_EXPECT_NOT_NULL(test, v4l2_subdev_state_get_format(state, 7, 0));
	v4l2_subdev_unlock_state(state);
}

/*
 * Two sources on CSI0 behind a board switch (2 and 4 data lanes): one link at
 * a time, the lane configuration follows the linked source, no change while
 * the input runs.
 */
static void mtk_seninf_test_two_sources(struct kunit *test)
{
	struct mtk_seninf *priv = mtk_seninf_test_alloc(test);
	struct mtk_seninf_input *input = &priv->inputs[CSI_PORT_0];
	struct media_entity *entity = &priv->subdev.entity;
	struct media_pad *sink = &priv->pads[CSI_PORT_0];
	struct media_entity *sensor;
	struct v4l2_subdev *sd;
	struct media_pad *pad;
	unsigned int i;

	sensor = kunit_kcalloc(test, 3, sizeof(*sensor), GFP_KERNEL);
	sd = kunit_kcalloc(test, 2, sizeof(*sd), GFP_KERNEL);
	pad = kunit_kcalloc(test, 3, sizeof(*pad), GFP_KERNEL);
	KUNIT_ASSERT_NOT_NULL(test, sensor);
	KUNIT_ASSERT_NOT_NULL(test, sd);
	KUNIT_ASSERT_NOT_NULL(test, pad);

	/* pad 2 belongs to no source of the input */
	for (i = 0; i < 3; i++) {
		sensor[i].name = "mtk-seninf-test-sensor";
		pad[i].entity = &sensor[i];
		pad[i].flags = MEDIA_PAD_FL_SOURCE;
	}

	input->pad = CSI_PORT_0;
	input->num_sources = 2;
	for (i = 0; i < 2; i++) {
		input->sources[i].sd = &sd[i];
		input->sources[i].pad = &pad[i];
		input->sources[i].bus.num_data_lanes = i ? 4 : 2;
	}

	/* the first source, linked as after binding */
	KUNIT_EXPECT_EQ(test, seninf_link_setup(entity, sink, &pad[0],
						MEDIA_LNK_FL_ENABLED), 0);
	KUNIT_EXPECT_PTR_EQ(test, input->source_sd, &sd[0]);
	KUNIT_EXPECT_EQ(test, input->bus.num_data_lanes, 2U);

	/* the second one only once the first is unlinked */
	KUNIT_EXPECT_EQ(test, seninf_link_setup(entity, sink, &pad[1],
						MEDIA_LNK_FL_ENABLED), -EBUSY);
	KUNIT_EXPECT_PTR_EQ(test, input->source_sd, &sd[0]);

	/* not while the input runs */
	input->users = 1;
	KUNIT_EXPECT_EQ(test, seninf_link_setup(entity, sink, &pad[0], 0), -EBUSY);
	KUNIT_EXPECT_PTR_EQ(test, input->active, &input->sources[0]);
	input->users = 0;

	KUNIT_EXPECT_EQ(test, seninf_link_setup(entity, sink, &pad[0], 0), 0);
	KUNIT_EXPECT_NULL(test, input->source_sd);
	KUNIT_EXPECT_NULL(test, input->source_pad);
	KUNIT_EXPECT_EQ(test, seninf_link_setup(entity, sink, &pad[1],
						MEDIA_LNK_FL_ENABLED), 0);
	KUNIT_EXPECT_PTR_EQ(test, input->source_sd, &sd[1]);
	KUNIT_EXPECT_PTR_EQ(test, input->source_pad, &pad[1]);
	KUNIT_EXPECT_EQ(test, input->bus.num_data_lanes, 4U);

	/* unlinking the other source changes nothing */
	KUNIT_EXPECT_EQ(test, seninf_link_setup(entity, sink, &pad[0], 0), 0);
	KUNIT_EXPECT_PTR_EQ(test, input->source_sd, &sd[1]);

	/* no source of the input; the output links are left to the core */
	KUNIT_EXPECT_EQ(test, seninf_link_setup(entity, sink, &pad[2],
						MEDIA_LNK_FL_ENABLED), -EINVAL);
	KUNIT_EXPECT_EQ(test, seninf_link_setup(entity, &priv->pads[7], &pad[2],
						MEDIA_LNK_FL_ENABLED), 0);
	KUNIT_EXPECT_PTR_EQ(test, input->source_sd, &sd[1]);
}

static struct kunit_case mtk_seninf_test_cases[] = {
	KUNIT_CASE(mtk_seninf_test_default_routes),
	KUNIT_CASE(mtk_seninf_test_routing),
	KUNIT_CASE(mtk_seninf_test_format),
	KUNIT_CASE(mtk_seninf_test_two_sources),
	{}
};

static struct kunit_suite mtk_seninf_test_suite = {
	.name = "mtk-seninf-stream-state",
	.test_cases = mtk_seninf_test_cases,
};

kunit_test_suite(mtk_seninf_test_suite);
