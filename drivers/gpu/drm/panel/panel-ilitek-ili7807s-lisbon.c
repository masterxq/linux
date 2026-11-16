// SPDX-License-Identifier: GPL-2.0-only
/*
 * Tianma ILI7807S 1080x2460 DSC video-mode panel of the Motorola Moto G60s (lisbon), behind a
 * MediaTek MT6382 DSI bridge which does the DSC compression (DSC 1.1, 8 bpp, 2 slices of 540x10).
 *
 * Timing, DSC parameters, the init table and the power sequence come from the vendor panel driver
 * panel_mot_mipi_vid_tianma_ili7807s_678_fhd (GPL-2.0, MediaTek/Motorola); re-implemented on
 * drm_panel.
 *
 * lk initialises the panel. With the "takeover" parameter (default on) the driver keeps a panel
 * that is running at probe (reset released, bias on) and does not touch it on the first prepare;
 * GPIOs are requested "as is". The backlight (AW99703) is a separate device referenced from the DT.
 */

#include <linux/delay.h>
#include <linux/gpio/consumer.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/of.h>

#include <drm/display/drm_dsc.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_panel.h>
#include <drm/drm_probe_helper.h>

#include <video/mipi_display.h>

static bool takeover = true;
module_param(takeover, bool, 0444);
MODULE_PARM_DESC(takeover, "Keep a panel that lk left running (no reset, no init on the first prepare). Default: on");

struct ili7807s_lisbon {
	struct drm_panel panel;
	struct mipi_dsi_device *dsi;
	struct gpio_desc *reset;
	struct gpio_desc *vddi;
	struct gpio_desc *enp;
	struct gpio_desc *enn;
	struct drm_dsc_config dsc;
	bool boot_on;
};

struct ili7807s_cmd {
	u8 cmd;
	u8 len;
	u8 data[3];
	u16 delay_ms;
};

/* init_setting_cmd of the vendor driver (120 Hz DSC settings, page 0 sleep out/display on) */
static const struct ili7807s_cmd ili7807s_init[] = {
	{ 0xff, 3, { 0x78, 0x07, 0x06 } },
	{ 0x5f, 1, { 0x24 } },
	{ 0x4e, 1, { 0xf0 } },
	{ 0x4d, 1, { 0x00 } },
	{ 0x48, 1, { 0x09 } },
	{ 0xc7, 1, { 0x05 } },
	{ 0xff, 3, { 0x78, 0x07, 0x00 } },
	{ MIPI_DCS_EXIT_SLEEP_MODE, 1, { 0x00 }, 60 },
	{ MIPI_DCS_SET_DISPLAY_ON, 1, { 0x00 }, 20 },
	{ 0xff, 3, { 0x78, 0x07, 0x03 } },	/* PWM 12 bit, 28.32 kHz */
	{ 0x83, 1, { 0x20 } },
	{ 0x84, 1, { 0x00 } },
	{ 0xff, 3, { 0x78, 0x07, 0x06 } },
	{ 0x08, 1, { 0x20 } },
	{ 0xff, 3, { 0x78, 0x07, 0x00 } },
	{ 0x51, 2, { 0x0c, 0xcc } },
	{ 0x53, 1, { 0x2c } },
	{ 0x55, 1, { 0x01 } },			/* CABC UI */
	{ 0x35, 1, { 0x00 } },
	{ 0x55, 1, { 0x01 } },
	{ 0xff, 3, { 0x78, 0x07, 0x06 } },
	{ 0x3e, 1, { 0xe3 } },
	{ 0x80, 1, { 0x00 } },
	{ 0xff, 3, { 0x78, 0x07, 0x08 } },
	{ 0xfd, 2, { 0x00, 0x9f } },
	{ 0xe1, 1, { 0xd9 } },
	{ 0xfd, 2, { 0x00, 0x00 } },
	{ 0xff, 3, { 0x78, 0x07, 0x00 } },
	{ 0x55, 1, { 0x01 } },
};

static inline struct ili7807s_lisbon *to_ili(struct drm_panel *panel)
{
	return container_of(panel, struct ili7807s_lisbon, panel);
}

/* like the vendor push_table: DCS below 0xb0, generic (manufacturer) above */
static int ili7807s_send(struct ili7807s_lisbon *ctx, const struct ili7807s_cmd *c)
{
	u8 buf[4];
	ssize_t ret;

	buf[0] = c->cmd;
	memcpy(buf + 1, c->data, c->len);
	if (c->cmd < 0xb0)
		ret = mipi_dsi_dcs_write_buffer(ctx->dsi, buf, c->len + 1);
	else
		ret = mipi_dsi_generic_write(ctx->dsi, buf, c->len + 1);
	if (ret < 0)
		return ret;
	if (c->delay_ms)
		msleep(c->delay_ms);
	return 0;
}

static int ili7807s_prepare(struct drm_panel *panel)
{
	struct ili7807s_lisbon *ctx = to_ili(panel);
	struct device *dev = &ctx->dsi->dev;
	unsigned int i;
	int ret;

	if (ctx->boot_on) {
		dev_info(dev, "panel kept as lk left it\n");
		return 0;
	}

	/* lcm_init_power: LCD LDO, then bias +/- (bias chip keeps the lk voltages) */
	gpiod_set_value_cansleep(ctx->vddi, 1);
	usleep_range(2000, 3000);
	gpiod_set_value_cansleep(ctx->enp, 1);
	usleep_range(2000, 3000);
	gpiod_set_value_cansleep(ctx->enn, 1);
	usleep_range(5000, 6000);

	/* lcm_init: reset 1-0-1 (logical: released, asserted, released) */
	gpiod_set_value_cansleep(ctx->reset, 0);
	msleep(10);
	gpiod_set_value_cansleep(ctx->reset, 1);
	msleep(10);
	gpiod_set_value_cansleep(ctx->reset, 0);
	msleep(20);

	for (i = 0; i < ARRAY_SIZE(ili7807s_init); i++) {
		ret = ili7807s_send(ctx, &ili7807s_init[i]);
		if (ret) {
			dev_err(dev, "init command %u (0x%02x): %d\n", i,
				ili7807s_init[i].cmd, ret);
			return ret;
		}
	}
	return 0;
}

static int ili7807s_unprepare(struct drm_panel *panel)
{
	struct ili7807s_lisbon *ctx = to_ili(panel);
	struct device *dev = &ctx->dsi->dev;
	int ret;

	/* lcm_suspend_setting, then bias off; reset stays released like the vendor */
	ret = mipi_dsi_dcs_set_display_off(ctx->dsi);
	if (ret < 0)
		dev_warn(dev, "display off: %d\n", ret);
	msleep(20);
	ret = mipi_dsi_dcs_enter_sleep_mode(ctx->dsi);
	if (ret < 0)
		dev_warn(dev, "sleep in: %d\n", ret);
	msleep(120);

	gpiod_set_value_cansleep(ctx->enn, 0);
	usleep_range(2000, 3000);
	gpiod_set_value_cansleep(ctx->enp, 0);
	ctx->boot_on = false;
	return 0;
}

/*
 * Panel timing of the vendor driver (60 Hz: VFP 2580); the clock is what
 * this timing gives at 60 Hz, the bridge derives its own link timing.
 */
static const struct drm_display_mode ili7807s_mode = {
	.clock = 385117,
	.hdisplay = 1080,
	.hsync_start = 1080 + 102,
	.hsync_end = 1080 + 102 + 4,
	.htotal = 1080 + 102 + 4 + 82,
	.vdisplay = 2460,
	.vsync_start = 2460 + 2580,
	.vsync_end = 2460 + 2580 + 2,
	.vtotal = 2460 + 2580 + 2 + 20,
	.width_mm = 70,
	.height_mm = 161,
	.type = DRM_MODE_TYPE_DRIVER | DRM_MODE_TYPE_PREFERRED,
};

static int ili7807s_get_modes(struct drm_panel *panel,
			      struct drm_connector *connector)
{
	return drm_connector_helper_get_modes_fixed(connector, &ili7807s_mode);
}

static const struct drm_panel_funcs ili7807s_funcs = {
	.prepare = ili7807s_prepare,
	.unprepare = ili7807s_unprepare,
	.get_modes = ili7807s_get_modes,
};

/* dsc_params of the vendor lcm_get_params */
static void ili7807s_init_dsc(struct drm_dsc_config *dsc)
{
	dsc->dsc_version_major = 1;
	dsc->dsc_version_minor = 1;
	dsc->slice_height = 10;
	dsc->slice_width = 540;
	dsc->slice_count = 2;
	dsc->bits_per_component = 8;
	dsc->bits_per_pixel = 8 << 4;
	dsc->block_pred_enable = true;
	dsc->convert_rgb = true;
	dsc->pic_width = 1080;
	dsc->pic_height = 2460;
	dsc->line_buf_depth = 9;
	dsc->slice_chunk_size = 540;
	dsc->initial_xmit_delay = 512;
	dsc->initial_dec_delay = 526;
	dsc->initial_scale_value = 32;
	dsc->scale_increment_interval = 237;
	dsc->scale_decrement_interval = 7;
	dsc->first_line_bpg_offset = 12;
	dsc->nfl_bpg_offset = 2731;
	dsc->slice_bpg_offset = 2604;
	dsc->initial_offset = 6144;
	dsc->final_offset = 4366;
	dsc->flatness_min_qp = 3;
	dsc->flatness_max_qp = 12;
	dsc->rc_model_size = 8192;
	dsc->rc_edge_factor = 6;
	dsc->rc_quant_incr_limit0 = 11;
	dsc->rc_quant_incr_limit1 = 11;
	dsc->rc_tgt_offset_high = 3;
	dsc->rc_tgt_offset_low = 3;
}

static int ili7807s_probe(struct mipi_dsi_device *dsi)
{
	struct device *dev = &dsi->dev;
	struct ili7807s_lisbon *ctx;
	int ret;

	ctx = devm_drm_panel_alloc(dev, struct ili7807s_lisbon, panel,
				   &ili7807s_funcs, DRM_MODE_CONNECTOR_DSI);
	if (IS_ERR(ctx))
		return PTR_ERR(ctx);
	ctx->dsi = dsi;
	mipi_dsi_set_drvdata(dsi, ctx);

	/* "as is": never change a level lk set up */
	ctx->reset = devm_gpiod_get(dev, "reset", GPIOD_ASIS);
	if (IS_ERR(ctx->reset))
		return dev_err_probe(dev, PTR_ERR(ctx->reset), "reset gpio\n");
	ctx->vddi = devm_gpiod_get_optional(dev, "vddi-enable", GPIOD_ASIS);
	if (IS_ERR(ctx->vddi))
		return dev_err_probe(dev, PTR_ERR(ctx->vddi), "vddi gpio\n");
	ctx->enp = devm_gpiod_get_optional(dev, "enp", GPIOD_ASIS);
	if (IS_ERR(ctx->enp))
		return dev_err_probe(dev, PTR_ERR(ctx->enp), "enp gpio\n");
	ctx->enn = devm_gpiod_get_optional(dev, "enn", GPIOD_ASIS);
	if (IS_ERR(ctx->enn))
		return dev_err_probe(dev, PTR_ERR(ctx->enn), "enn gpio\n");

	/* running: reset released and positive bias on */
	ctx->boot_on = takeover && gpiod_get_value_cansleep(ctx->reset) == 0 &&
		       (!ctx->enp || gpiod_get_value_cansleep(ctx->enp) == 1);
	dev_info(dev, "lk state: reset %d, enp %d, enn %d -> %s\n",
		 gpiod_get_value_cansleep(ctx->reset),
		 ctx->enp ? gpiod_get_value_cansleep(ctx->enp) : -1,
		 ctx->enn ? gpiod_get_value_cansleep(ctx->enn) : -1,
		 ctx->boot_on ? "takeover" : "full init on prepare");

	ret = drm_panel_of_backlight(&ctx->panel);
	if (ret)
		return dev_err_probe(dev, ret, "backlight\n");

	ili7807s_init_dsc(&ctx->dsc);
	dsi->dsc = &ctx->dsc;
	dsi->lanes = 4;
	dsi->format = MIPI_DSI_FMT_RGB888;
	dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_SYNC_PULSE |
			  MIPI_DSI_MODE_LPM;

	drm_panel_add(&ctx->panel);

	ret = mipi_dsi_attach(dsi);
	if (ret) {
		drm_panel_remove(&ctx->panel);
		return dev_err_probe(dev, ret, "attach to the bridge DSI host\n");
	}
	return 0;
}

static void ili7807s_remove(struct mipi_dsi_device *dsi)
{
	struct ili7807s_lisbon *ctx = mipi_dsi_get_drvdata(dsi);

	mipi_dsi_detach(dsi);
	drm_panel_remove(&ctx->panel);
}

static const struct of_device_id ili7807s_of_match[] = {
	{ .compatible = "tianma,ili7807s-lisbon" },
	{ }
};
MODULE_DEVICE_TABLE(of, ili7807s_of_match);

static struct mipi_dsi_driver ili7807s_driver = {
	.probe = ili7807s_probe,
	.remove = ili7807s_remove,
	.driver = {
		.name = "panel-ilitek-ili7807s-lisbon",
		.of_match_table = ili7807s_of_match,
	},
};
module_mipi_dsi_driver(ili7807s_driver);

MODULE_DESCRIPTION("Tianma ILI7807S DSC panel of the Motorola Moto G60s (behind MT6382)");
MODULE_LICENSE("GPL");
