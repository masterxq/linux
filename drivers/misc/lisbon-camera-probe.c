// SPDX-License-Identifier: GPL-2.0-only
/*
 * Gated power-up and chip ID probe for the five cameras of the Motorola Moto G60s (lisbon, MT6785).
 *
 * The lisbon-kamera device tree describes the camera buses (I2C2, I2C4, I2C7), the GPIO switched
 * camera LDOs (regulator-fixed), the master clocks (topckgen camtg1..5, CMMCLK0..4 pins), the reset
 * and power-down lines, the MIPI switch (GPIO72) and the sensor, VCM and EEPROM nodes, but keeps
 * everything disabled, so the system behaves as if the cameras were not described.
 *
 * On load the module enables the camera LDO nodes, the three buses and the sensor nodes with one
 * device tree changeset and binds to the five sensors. Then, one sensor at a time and in the order
 * of the vendor sensor index, it runs the vendor power-up sequence, reads the chip ID (main camera:
 * also the VCM status and the EEPROM header; front camera: the EEPROM header), runs the vendor
 * power-down sequence and prints one RESULT line with the state of all camera supplies at the end.
 * Unloading reverts the changeset.
 *
 * With stream=1 the module only switches the front camera data path on: the cam1 LDOs, I2C4, the
 * CSI-RX PHY, seninf and CAMSV1 found through the seninf graph, and the OV16A1Q node, in that order
 * and with one changeset. It does not bind to the sensor and runs no sequence; the V4L2 drivers
 * (ov16a1q, phy-mtk-mipi-csi-0-5, mtk-seninf, mtk-camsv) take the nodes. Unloading reverts the
 * changeset, after the V4L2 drivers are gone.
 *
 * Writes to the cameras: none, except the page select 0xfd = 0x00 that the vendor OV02B1B driver
 * sends before its ID read. The VCM and the EEPROMs are only read.
 *
 * Values and sequences follow the vendor kernel:
 *  - power sequences and waits; off = the on steps in reverse order with the same waits and every
 *    line low; MIPI switch: S5K4H7 1, OV02B1B and S5K5E9 0, off 0
 *  - DOVDD 1.8 V is set before it is enabled; the other supplies are GPIO switched LDOs
 *  - MCLK: 24 MHz from TOP_MUX_CAMTG<n+1> for CMMCLK<n>; the MCLK step only switches the pin
 *    between GPIO and CMMCLK mode, the clock runs before the sequence starts
 *  - I2C addresses, ID registers and IDs; EEPROM 0xa0/0xa2 with 16-bit offsets and the header
 *    layout (table version, calibration HW/SW version, 8 byte part number); DW9800V at 0x18 (8 bit)
 *    with single byte register reads
 */

#define pr_fmt(fmt) "lisbon-camera-probe: " fmt

#include <linux/array_size.h>
#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/gpio/consumer.h>
#include <linux/i2c.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/pinctrl/consumer.h>
#include <linux/property.h>
#include <linux/regulator/consumer.h>
#include <linux/sprintf.h>
#include <linux/time64.h>

#define LCP_MCLK_HZ		24000000
#define LCP_DOVDD_UV		1800000
#define LCP_BIND_WAIT_MS	3000
#define LCP_POLL_MS		20
#define LCP_ID_TRIES		2
#define LCP_MIPI_SWITCH_PATH	"/mux-controller-cam-mipi"
#define LCP_PINS_MCLK		"mclk"

/* OV02B1B page select register, written with 0 before the ID read like the vendor driver */
#define LCP_OV02B1B_PAGE	0xfd

/* DW9800V registers read for the status (all reads) */
static const u8 lcp_vcm_regs[] = { 0x00, 0x02, 0x05, 0x06, 0x07 };

/* EEPROM header: table version, calibration HW and SW version, part number (8 bytes) */
#define LCP_EEPROM_HDR		11
#define LCP_EEPROM_MPN		3

enum lcp_pin {
	LCP_MCLK,
	LCP_LINE,	/* the reset (RST) or power-down (PDN) line, whichever the sequence uses */
	LCP_AVDD,
	LCP_DVDD,
	LCP_DOVDD,
	LCP_AFVDD,
	LCP_NPINS,
};

static const char *const lcp_pin_names[LCP_NPINS] = {
	"MCLK", "LINE", "AVDD", "DVDD", "DOVDD", "AFVDD",
};

struct lcp_step {
	u8 pin;
	u8 on;		/* line: level while powered; others: 1 */
	u8 delay_ms;	/* wait after the step, on and off */
};

#define LCP_MAX_STEPS	8

struct lcp_desc {
	const char *name;
	unsigned int idx;	/* vendor sensor index = camN in the vendor DT */
	const char *line_name;	/* vendor name of the line: RST or PDN */
	unsigned int addr_bytes;
	u16 id_reg;		/* high byte; low byte at id_reg + 1 */
	u16 id;
	int mipi_sel;		/* MIPI switch level from the vendor platform sequence, -1 none */
	bool page0;
	bool vcm;
	u16 eeprom;		/* 7-bit EEPROM address on the same bus, 0 none */
	unsigned int nsteps;
	struct lcp_step steps[LCP_MAX_STEPS];
};

static const struct lcp_desc lcp_ov64b40 = {
	.name = "ov64b40", .idx = 0, .line_name = "RST",
	.addr_bytes = 2, .id_reg = 0x300b, .id = 0x6442, .mipi_sel = -1,
	.vcm = true, .eeprom = 0x50,
	.nsteps = 7,
	.steps = {
		{ LCP_MCLK, 1, 1 }, { LCP_LINE, 0, 1 }, { LCP_AVDD, 1, 0 }, { LCP_DOVDD, 1, 0 },
		{ LCP_DVDD, 1, 1 }, { LCP_AFVDD, 1, 0 }, { LCP_LINE, 1, 4 },
	},
};

static const struct lcp_desc lcp_ov16a1q = {
	.name = "ov16a1q", .idx = 1, .line_name = "RST",
	.addr_bytes = 2, .id_reg = 0x300b, .id = 0x1641, .mipi_sel = -1,
	.eeprom = 0x51,
	.nsteps = 6,
	.steps = {
		{ LCP_LINE, 0, 0 }, { LCP_MCLK, 1, 2 }, { LCP_DOVDD, 1, 0 }, { LCP_AVDD, 1, 0 },
		{ LCP_DVDD, 1, 0 }, { LCP_LINE, 1, 5 },
	},
};

static const struct lcp_desc lcp_ov02b1b = {
	.name = "ov02b1b", .idx = 2, .line_name = "PDN",
	.addr_bytes = 1, .id_reg = 0x02, .id = 0x002b, .mipi_sel = 0, .page0 = true,
	.nsteps = 5,
	.steps = {
		{ LCP_LINE, 0, 1 }, { LCP_DOVDD, 1, 1 }, { LCP_AVDD, 1, 11 }, { LCP_MCLK, 1, 1 },
		{ LCP_LINE, 1, 5 },
	},
};

static const struct lcp_desc lcp_s5k4h7 = {
	.name = "s5k4h7", .idx = 4, .line_name = "PDN",
	.addr_bytes = 2, .id_reg = 0x0000, .id = 0x487b, .mipi_sel = 1,
	.nsteps = 6,
	.steps = {
		{ LCP_LINE, 0, 1 }, { LCP_MCLK, 1, 1 }, { LCP_AVDD, 1, 1 }, { LCP_DVDD, 1, 1 },
		{ LCP_DOVDD, 1, 1 }, { LCP_LINE, 1, 5 },
	},
};

static const struct lcp_desc lcp_s5k5e9 = {
	.name = "s5k5e9", .idx = 6, .line_name = "RST",
	.addr_bytes = 2, .id_reg = 0x0000, .id = 0x559b, .mipi_sel = 0,
	.nsteps = 6,
	.steps = {
		{ LCP_LINE, 0, 1 }, { LCP_AVDD, 1, 1 }, { LCP_DVDD, 1, 1 }, { LCP_DOVDD, 1, 1 },
		{ LCP_MCLK, 1, 1 }, { LCP_LINE, 1, 18 },
	},
};

/* in the order of the vendor sensor index, the order the vendor HAL probes them */
static const struct lcp_desc *const lcp_descs[] = {
	&lcp_ov64b40, &lcp_ov16a1q, &lcp_ov02b1b, &lcp_s5k4h7, &lcp_s5k5e9,
};

#define LCP_NCAMS	ARRAY_SIZE(lcp_descs)

static const char *const lcp_compatibles[LCP_NCAMS] = {
	"ovti,ov64b40", "ovti,ov16a1q", "ovti,ov02b1b", "samsung,s5k4h7", "samsung,s5k5e9",
};

static unsigned int cams = BIT(0) | BIT(1) | BIT(2) | BIT(4) | BIT(6);
module_param(cams, uint, 0444);
MODULE_PARM_DESC(cams, "bit mask of the vendor sensor indexes to probe (default 0x57: 0,1,2,4,6)");

static bool stream;
module_param(stream, bool, 0444);
MODULE_PARM_DESC(stream,
		 "only enable the front camera data path (cam1 LDOs, I2C4, CSI-RX PHY, seninf, CAMSV1, OV16A1Q) for the V4L2 drivers");

static bool mclk_all;
module_param(mclk_all, bool, 0444);
MODULE_PARM_DESC(mclk_all,
		 "run all five camtg clocks during each sequence (test of the CAMTG/CMMCLK mapping)");

struct lcp_cam {
	const struct lcp_desc *desc;
	struct i2c_client *client;
	struct clk *mclk;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_off;
	struct pinctrl_state *pins_on;
	struct gpio_desc *line;
	struct regulator *supply[LCP_NPINS];
	bool done[LCP_MAX_STEPS];
	bool mclk_on;
	int id_err;
	u16 id;
	int vcm_err;
	u8 vcm[ARRAY_SIZE(lcp_vcm_regs)];
	int eeprom_err;
	u8 eeprom[LCP_EEPROM_HDR];
	bool ran;
	bool powered;
	bool seq_err;
};

static struct lcp_cam lcp_cams[LCP_NCAMS];
static DEFINE_MUTEX(lcp_lock);
static struct gpio_desc *lcp_mipi_sel;
static struct of_changeset lcp_cs;
static bool lcp_cs_applied;
static bool lcp_driver_added;

static const char *lcp_onoff(struct regulator *reg)
{
	int ret;

	if (!reg)
		return "-";
	ret = regulator_is_enabled(reg);
	if (ret < 0)
		return "err";

	return ret ? "ON" : "off";
}

static void lcp_print_state(struct lcp_cam *cam, const char *when)
{
	struct device *dev = &cam->client->dev;
	int dir = cam->line ? gpiod_get_direction(cam->line) : -ENODEV;
	int raw = cam->line ? gpiod_get_raw_value_cansleep(cam->line) : -ENODEV;

	dev_info(dev, "%s %s: %s %s/%d avdd=%s dvdd=%s dovdd=%s afvdd=%s mipi_sel=%d\n",
		 cam->desc->name, when, cam->desc->line_name,
		 dir < 0 ? "?" : dir ? "in" : "out", raw,
		 lcp_onoff(cam->supply[LCP_AVDD]), lcp_onoff(cam->supply[LCP_DVDD]),
		 lcp_onoff(cam->supply[LCP_DOVDD]), lcp_onoff(cam->supply[LCP_AFVDD]),
		 lcp_mipi_sel ? gpiod_get_raw_value_cansleep(lcp_mipi_sel) : -1);
}

static int lcp_mclk_all(struct lcp_cam *self, bool on)
{
	unsigned int i;
	int ret;

	for (i = 0; i < LCP_NCAMS; i++) {
		struct lcp_cam *c = &lcp_cams[i];

		if (c == self || !c->client || !c->mclk || clk_get_rate(c->mclk) != LCP_MCLK_HZ)
			continue;
		if (on) {
			ret = clk_prepare_enable(c->mclk);
			if (ret)
				return ret;
			c->mclk_on = true;
		} else if (c->mclk_on) {
			clk_disable_unprepare(c->mclk);
			c->mclk_on = false;
		}
	}

	return 0;
}

static int lcp_set(struct lcp_cam *cam, const struct lcp_step *s, bool power_on)
{
	struct device *dev = &cam->client->dev;
	struct regulator *dovdd = cam->supply[LCP_DOVDD];
	struct regulator *reg;
	const char *state;
	int ret = 0;

	switch (s->pin) {
	case LCP_MCLK:
		ret = pinctrl_select_state(cam->pinctrl, power_on ? cam->pins_on : cam->pins_off);
		break;
	case LCP_LINE:
		ret = gpiod_direction_output_raw(cam->line, power_on ? s->on : 0);
		break;
	case LCP_DOVDD:
		if (power_on) {
			ret = regulator_set_voltage(dovdd, LCP_DOVDD_UV, LCP_DOVDD_UV);
			if (!ret && regulator_get_voltage(dovdd) != LCP_DOVDD_UV)
				ret = -ERANGE;
			if (ret) {
				dev_err(dev, "DOVDD not at %d uV: %pe\n", LCP_DOVDD_UV,
					ERR_PTR(ret));
				break;
			}
		}
		fallthrough;
	default:
		reg = cam->supply[s->pin];
		ret = power_on ? regulator_enable(reg) : regulator_disable(reg);
		break;
	}

	if (s->pin == LCP_LINE)
		state = power_on && s->on ? "high" : "low";
	else
		state = power_on ? "on" : "off";
	dev_info(dev, "%s %s %s, wait %u ms: %d\n", cam->desc->name, lcp_pin_names[s->pin],
		 state, s->delay_ms, ret);

	return ret;
}

/* The vendor power-down: the steps done so far in reverse order, each line low, same waits. */
static void lcp_power_off(struct lcp_cam *cam)
{
	const struct lcp_desc *d = cam->desc;
	int i, ret;

	if (!cam->powered)
		return;
	cam->powered = false;
	if (d->mipi_sel >= 0 && lcp_mipi_sel)
		gpiod_direction_output_raw(lcp_mipi_sel, 0);

	for (i = d->nsteps - 1; i >= 0; i--) {
		if (!cam->done[i])
			continue;
		ret = lcp_set(cam, &d->steps[i], false);
		if (ret) {
			dev_err(&cam->client->dev, "%s off: %s failed: %pe\n", d->name,
				lcp_pin_names[d->steps[i].pin], ERR_PTR(ret));
			cam->seq_err = true;
		}
		cam->done[i] = false;
		fsleep(d->steps[i].delay_ms * USEC_PER_MSEC);
	}

	if (cam->mclk_on) {
		clk_disable_unprepare(cam->mclk);
		cam->mclk_on = false;
	}
	if (mclk_all)
		lcp_mclk_all(cam, false);
}

static int lcp_power_on(struct lcp_cam *cam)
{
	const struct lcp_desc *d = cam->desc;
	unsigned int i;
	int ret;

	cam->powered = true;
	ret = clk_prepare_enable(cam->mclk);
	if (ret)
		return ret;
	cam->mclk_on = true;
	if (mclk_all) {
		ret = lcp_mclk_all(cam, true);
		if (ret)
			return ret;
	}

	if (d->mipi_sel >= 0) {
		ret = gpiod_direction_output_raw(lcp_mipi_sel, d->mipi_sel);
		if (ret)
			return ret;
	}

	for (i = 0; i < d->nsteps; i++) {
		ret = lcp_set(cam, &d->steps[i], true);
		if (ret) {
			dev_err(&cam->client->dev, "%s on: %s failed: %pe\n", d->name,
				lcp_pin_names[d->steps[i].pin], ERR_PTR(ret));
			return ret;
		}
		cam->done[i] = true;
		fsleep(d->steps[i].delay_ms * USEC_PER_MSEC);
	}

	return 0;
}

static int lcp_read_reg(struct i2c_client *client, unsigned int addr_bytes, u16 reg, u8 *val)
{
	u8 buf[2] = { reg >> 8, reg & 0xff };
	struct i2c_msg msgs[2] = {
		{ .addr = client->addr, .len = addr_bytes, .buf = addr_bytes == 2 ? buf : &buf[1] },
		{ .addr = client->addr, .flags = I2C_M_RD, .len = 1, .buf = val },
	};
	int ret;

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	if (ret == ARRAY_SIZE(msgs))
		return 0;

	return ret < 0 ? ret : -EIO;
}

static void lcp_read_id(struct lcp_cam *cam)
{
	const struct lcp_desc *d = cam->desc;
	struct i2c_client *client = cam->client;
	u8 hi = 0, lo = 0;
	int t, ret = 0;

	for (t = 0; t < LCP_ID_TRIES; t++) {
		ret = 0;
		if (d->page0) {
			u8 page[2] = { LCP_OV02B1B_PAGE, 0x00 };

			ret = i2c_master_send(client, page, sizeof(page));
			ret = ret == (int)sizeof(page) ? 0 : ret < 0 ? ret : -EIO;
		}
		if (!ret)
			ret = lcp_read_reg(client, d->addr_bytes, d->id_reg, &hi);
		if (!ret)
			ret = lcp_read_reg(client, d->addr_bytes, d->id_reg + 1, &lo);
		if (!ret && (hi << 8 | lo) == d->id)
			break;
	}

	cam->id_err = ret;
	cam->id = hi << 8 | lo;
	if (ret)
		dev_info(&client->dev, "%s ID read at 0x%02x failed: %pe\n", d->name, client->addr,
			 ERR_PTR(ret));
	else
		dev_info(&client->dev, "%s ID [0x%0*x..] = 0x%04x (expected 0x%04x): %s\n", d->name,
			 d->addr_bytes * 2, d->id_reg, cam->id, d->id,
			 cam->id == d->id ? "match" : "MISMATCH");
}

/* VCM status, single byte reads with a stop in between like the vendor lens driver */
static void lcp_read_vcm(struct lcp_cam *cam, struct device_node *lens)
{
	struct i2c_adapter *adap = cam->client->adapter;
	u32 addr;
	unsigned int i;
	int ret;

	ret = of_property_read_u32(lens, "reg", &addr);
	if (ret) {
		cam->vcm_err = ret;
		return;
	}

	for (i = 0; i < ARRAY_SIZE(lcp_vcm_regs); i++) {
		u8 reg = lcp_vcm_regs[i];
		struct i2c_msg w = { .addr = addr, .len = 1, .buf = &reg };
		struct i2c_msg r = {
			.addr = addr, .flags = I2C_M_RD, .len = 1, .buf = &cam->vcm[i],
		};

		ret = i2c_transfer(adap, &w, 1);
		if (ret == 1)
			ret = i2c_transfer(adap, &r, 1);
		if (ret != 1) {
			ret = ret < 0 ? ret : -EIO;
			break;
		}
		ret = 0;
	}

	cam->vcm_err = ret;
	if (ret)
		dev_info(&cam->client->dev, "vcm 0x%02x: read failed: %pe\n", addr, ERR_PTR(ret));
	else
		dev_info(&cam->client->dev,
			 "vcm 0x%02x: [00]=0x%02x [02]=0x%02x [05]=0x%02x [06]=0x%02x [07]=0x%02x\n",
			 addr, cam->vcm[0], cam->vcm[1], cam->vcm[2], cam->vcm[3], cam->vcm[4]);
}

/* EEPROM header only (no calibration data): 16-bit offset 0, sequential read */
static void lcp_read_eeprom(struct lcp_cam *cam)
{
	struct i2c_client *client = cam->client;
	u8 off[2] = { 0, 0 };
	struct i2c_msg msgs[2] = {
		{ .addr = cam->desc->eeprom, .len = sizeof(off), .buf = off },
		{ .addr = cam->desc->eeprom, .flags = I2C_M_RD, .len = LCP_EEPROM_HDR,
		  .buf = cam->eeprom },
	};
	int ret;

	ret = i2c_transfer(client->adapter, msgs, ARRAY_SIZE(msgs));
	cam->eeprom_err = ret == ARRAY_SIZE(msgs) ? 0 : ret < 0 ? ret : -EIO;
	if (cam->eeprom_err)
		dev_info(&client->dev, "eeprom 0x%02x: read failed: %pe\n", cam->desc->eeprom,
			 ERR_PTR(cam->eeprom_err));
	else
		dev_info(&client->dev,
			 "eeprom 0x%02x: table 0x%02x cal-hw 0x%02x cal-sw 0x%02x part \"%*pE\"\n",
			 cam->desc->eeprom, cam->eeprom[0], cam->eeprom[1], cam->eeprom[2],
			 LCP_EEPROM_HDR - LCP_EEPROM_MPN, &cam->eeprom[LCP_EEPROM_MPN]);
}

static void lcp_run(struct lcp_cam *cam)
{
	const struct lcp_desc *d = cam->desc;
	struct device *dev = &cam->client->dev;
	struct device_node *lens;
	unsigned long rate = clk_get_rate(cam->mclk);
	int ret;

	cam->ran = true;
	cam->id_err = -ENODATA;
	cam->vcm_err = -ENODATA;
	cam->eeprom_err = -ENODATA;

	dev_info(dev, "%s (cam%u) on %s addr 0x%02x, mclk %lu Hz%s\n", d->name, d->idx,
		 dev_name(&cam->client->adapter->dev), cam->client->addr, rate,
		 mclk_all ? " (all camtg running)" : "");
	if (rate != LCP_MCLK_HZ) {
		dev_err(dev, "%s: mclk is not 24 MHz, not powering the sensor\n", d->name);
		cam->id_err = -EINVAL;
		cam->seq_err = true;
		return;
	}

	lcp_print_state(cam, "before");
	ret = lcp_power_on(cam);
	if (!ret) {
		lcp_print_state(cam, "on");
		lcp_read_id(cam);
	}
	if (ret) {
		cam->id_err = ret;
		cam->seq_err = true;
	} else {
		lens = of_parse_phandle(dev->of_node, "lens-focus", 0);
		if (d->vcm && lens)
			lcp_read_vcm(cam, lens);
		of_node_put(lens);
		if (d->eeprom)
			lcp_read_eeprom(cam);
	}
	lcp_power_off(cam);
	lcp_print_state(cam, "after");
}

/*
 * Before the first sequence every reset/power-down line goes to its vendor off level (low), the
 * level each sensor has after its vendor power-down. The bootloader leaves the lines of cam2,
 * cam4 and cam6 high; with the shared AVDD and DOVDD switched on for cam2 and the cam4/cam6 DVDD
 * still off, the half supplied S5K4H7 and S5K5E9 come out of reset and the OV02B1B on the same
 * bus does not answer.
 */
static void lcp_park_lines(void)
{
	unsigned int i;
	int ret;

	for (i = 0; i < LCP_NCAMS; i++) {
		struct lcp_cam *c = &lcp_cams[i];

		if (!c->client || !c->line)
			continue;
		if (gpiod_get_direction(c->line) == 0 &&
		    gpiod_get_raw_value_cansleep(c->line) == 0)
			continue;
		ret = gpiod_direction_output_raw(c->line, 0);
		dev_info(&c->client->dev, "%s %s parked low (vendor off level): %d\n",
			 c->desc->name, c->desc->line_name, ret);
	}
}

static bool lcp_supplies_off(void)
{
	bool off = true;
	unsigned int i, p;

	for (i = 0; i < LCP_NCAMS; i++) {
		struct lcp_cam *c = &lcp_cams[i];

		if (!c->client)
			continue;
		for (p = LCP_AVDD; p < LCP_NPINS; p++)
			if (c->supply[p] && regulator_is_enabled(c->supply[p]) != 0)
				off = false;
	}

	return off;
}

static void lcp_summary(void)
{
	char line[256];
	unsigned int i;
	int n = 0;

	for (i = 0; i < LCP_NCAMS; i++) {
		const struct lcp_cam *c = &lcp_cams[i];
		const struct lcp_desc *d = lcp_descs[i];

		if (!(cams & BIT(d->idx)))
			continue;
		if (!c->client)
			n += scnprintf(line + n, sizeof(line) - n, " cam%u=%s:unbound", d->idx,
				       d->name);
		else if (!c->ran)
			n += scnprintf(line + n, sizeof(line) - n, " cam%u=%s:skipped", d->idx,
				       d->name);
		else if (c->id_err)
			n += scnprintf(line + n, sizeof(line) - n, " cam%u=%s:%pe", d->idx, d->name,
				       ERR_PTR(c->id_err));
		else
			n += scnprintf(line + n, sizeof(line) - n, " cam%u=%s:0x%04x%s", d->idx,
				       d->name, c->id, c->id == d->id ? "" : "(mismatch)");
		if (c->ran && d->vcm)
			n += scnprintf(line + n, sizeof(line) - n, " vcm=%s",
				       c->vcm_err ? "error" : "ok");
		if (c->ran && d->eeprom)
			n += scnprintf(line + n, sizeof(line) - n, " eeprom%u=%s", d->idx,
				       c->eeprom_err ? "error" : "ok");
		if (c->seq_err)
			n += scnprintf(line + n, sizeof(line) - n, " seq%u=error", d->idx);
	}
	n += scnprintf(line + n, sizeof(line) - n, " supplies=%s",
		       lcp_supplies_off() ? "all-off" : "STILL-ON");
	pr_info("RESULT%s\n", line);
}

static int lcp_probe(struct i2c_client *client)
{
	const struct lcp_desc *d = i2c_get_match_data(client);
	struct device *dev = &client->dev;
	struct lcp_cam *cam = NULL;
	struct device_node *lens;
	unsigned int i;

	for (i = 0; i < LCP_NCAMS; i++)
		if (lcp_descs[i] == d)
			cam = &lcp_cams[i];
	if (!cam)
		return -ENODEV;

	cam->mclk = devm_clk_get(dev, NULL);
	if (IS_ERR(cam->mclk))
		return dev_err_probe(dev, PTR_ERR(cam->mclk), "no mclk\n");

	cam->pinctrl = devm_pinctrl_get(dev);
	if (IS_ERR(cam->pinctrl))
		return dev_err_probe(dev, PTR_ERR(cam->pinctrl), "no pinctrl\n");
	cam->pins_off = pinctrl_lookup_state(cam->pinctrl, PINCTRL_STATE_DEFAULT);
	cam->pins_on = pinctrl_lookup_state(cam->pinctrl, LCP_PINS_MCLK);
	if (IS_ERR(cam->pins_off) || IS_ERR(cam->pins_on))
		return dev_err_probe(dev, -EINVAL, "pinctrl states default/%s missing\n",
				     LCP_PINS_MCLK);

	/* as is: the line is driven only by the sequence */
	cam->line = devm_gpiod_get_optional(dev, "reset", GPIOD_ASIS);
	if (!cam->line)
		cam->line = devm_gpiod_get_optional(dev, "powerdown", GPIOD_ASIS);
	if (IS_ERR_OR_NULL(cam->line))
		return dev_err_probe(dev, cam->line ? PTR_ERR(cam->line) : -EINVAL,
				     "no reset/powerdown line\n");

	cam->supply[LCP_AVDD] = devm_regulator_get_optional(dev, "avdd");
	cam->supply[LCP_DVDD] = devm_regulator_get_optional(dev, "dvdd");
	cam->supply[LCP_DOVDD] = devm_regulator_get_optional(dev, "dovdd");
	lens = of_parse_phandle(dev->of_node, "lens-focus", 0);
	cam->supply[LCP_AFVDD] = lens ? devm_of_regulator_get_optional(dev, lens, "vdd") :
					ERR_PTR(-ENODEV);
	of_node_put(lens);

	for (i = LCP_AVDD; i < LCP_NPINS; i++) {
		struct regulator *reg = cam->supply[i];
		unsigned int s;
		bool used = false;

		for (s = 0; s < d->nsteps; s++)
			used |= d->steps[s].pin == i;
		if (IS_ERR(reg)) {
			if (PTR_ERR(reg) == -EPROBE_DEFER || used)
				return dev_err_probe(dev, PTR_ERR(reg), "supply %s\n",
						     lcp_pin_names[i]);
			cam->supply[i] = NULL;
		}
	}

	mutex_lock(&lcp_lock);
	cam->desc = d;
	cam->client = client;
	mutex_unlock(&lcp_lock);
	lcp_print_state(cam, "bound");

	return 0;
}

static void lcp_remove(struct i2c_client *client)
{
	unsigned int i;

	mutex_lock(&lcp_lock);
	for (i = 0; i < LCP_NCAMS; i++) {
		struct lcp_cam *cam = &lcp_cams[i];

		if (cam->client != client)
			continue;
		/* nothing stays on after the run, this only covers an aborted run */
		lcp_power_off(cam);
		cam->client = NULL;
	}
	mutex_unlock(&lcp_lock);
}

static const struct of_device_id lcp_of_match[] = {
	{ .compatible = "ovti,ov64b40", .data = &lcp_ov64b40 },
	{ .compatible = "ovti,ov16a1q", .data = &lcp_ov16a1q },
	{ .compatible = "ovti,ov02b1b", .data = &lcp_ov02b1b },
	{ .compatible = "samsung,s5k4h7", .data = &lcp_s5k4h7 },
	{ .compatible = "samsung,s5k5e9", .data = &lcp_s5k5e9 },
	{ }
};

/* no MODULE_DEVICE_TABLE: the module must never load automatically */

static struct i2c_driver lcp_driver = {
	.driver = {
		.name = "lisbon-camera-probe",
		.of_match_table = lcp_of_match,
		.suppress_bind_attrs = true,
	},
	.probe = lcp_probe,
	.remove = lcp_remove,
};

static void lcp_cleanup(void);

/* status = "okay" for a node and, once, for each node its supply or bus phandles point to */
static int lcp_enable_node(struct device_node *np)
{
	if (of_device_is_available(np))
		return 0;

	return of_changeset_update_prop_string(&lcp_cs, np, "status", "okay");
}

static int lcp_enable_supply(struct device_node *np, const char *prop)
{
	struct device_node *reg = of_parse_phandle(np, prop, 0);
	struct of_changeset_entry *ce;
	int ret = 0;

	if (!reg)
		return 0;
	/* skip nodes already in the changeset (shared LDOs) */
	list_for_each_entry(ce, &lcp_cs.entries, node)
		if (ce->np == reg)
			goto out;
	if (of_device_is_compatible(reg, "regulator-fixed"))
		ret = lcp_enable_node(reg);
out:
	of_node_put(reg);

	return ret;
}

static int lcp_build_changeset(struct device_node **nodes)
{
	static const char *const supplies[] = { "avdd-supply", "dvdd-supply", "dovdd-supply" };
	struct of_changeset_entry *ce;
	struct device_node *bus, *lens;
	unsigned int i, s;
	bool seen;
	int ret = 0;

	/* LDOs first, then the buses, then the sensors: the notifications run in this order */
	for (i = 0; i < LCP_NCAMS && !ret; i++) {
		for (s = 0; s < ARRAY_SIZE(supplies) && !ret; s++)
			ret = lcp_enable_supply(nodes[i], supplies[s]);
		lens = of_parse_phandle(nodes[i], "lens-focus", 0);
		if (lens && !ret)
			ret = lcp_enable_supply(lens, "vdd-supply");
		of_node_put(lens);
	}
	for (i = 0; i < LCP_NCAMS && !ret; i++) {
		bus = of_get_parent(nodes[i]);
		seen = false;
		list_for_each_entry(ce, &lcp_cs.entries, node)
			seen |= ce->np == bus;
		if (!seen)
			ret = lcp_enable_node(bus);
		of_node_put(bus);
	}
	/* all sensors, also those not selected by cams: their lines are parked below */
	for (i = 0; i < LCP_NCAMS && !ret; i++)
		ret = lcp_enable_node(nodes[i]);

	return ret;
}

/* stream=1: the nodes of the front camera data path, sensor last */
static int lcp_build_stream_changeset(struct device_node *sensor)
{
	static const char *const supplies[] = { "avdd-supply", "dvdd-supply", "dovdd-supply" };
	struct device_node *seninf, *bus, *np, *ep, *remote;
	unsigned int s;
	int i, ret = 0;

	seninf = of_find_compatible_node(NULL, NULL, "mediatek,mt6785-seninf");
	if (!seninf) {
		pr_err("no mediatek,mt6785-seninf node\n");
		return -ENODEV;
	}

	for (s = 0; s < ARRAY_SIZE(supplies) && !ret; s++)
		ret = lcp_enable_supply(sensor, supplies[s]);
	bus = of_get_parent(sensor);
	if (!ret)
		ret = lcp_enable_node(bus);
	of_node_put(bus);

	for (i = 0; !ret && (np = of_parse_phandle(seninf, "phys", i)); i++) {
		ret = lcp_enable_node(np);
		of_node_put(np);
	}
	if (!ret)
		ret = lcp_enable_node(seninf);

	/* the outputs: every remote device of the seninf graph except the sensor */
	for_each_endpoint_of_node(seninf, ep) {
		if (ret)
			break;
		remote = of_graph_get_remote_port_parent(ep);
		if (remote && remote != sensor)
			ret = lcp_enable_node(remote);
		of_node_put(remote);
	}
	of_node_put(ep);
	of_node_put(seninf);

	if (!ret)
		ret = lcp_enable_node(sensor);

	return ret;
}

static int lcp_stream_init(void)
{
	struct device_node *sensor;
	struct of_changeset_entry *ce;
	int ret;

	sensor = of_find_compatible_node(NULL, NULL, "ovti,ov16a1q");
	if (!sensor) {
		pr_err("no ovti,ov16a1q node (boot the lisbon-kamera DTB)\n");
		return -ENODEV;
	}

	ret = lcp_build_stream_changeset(sensor);
	of_node_put(sensor);
	if (!ret && !list_empty(&lcp_cs.entries)) {
		ret = of_changeset_apply(&lcp_cs);
		lcp_cs_applied = !ret;
	}
	if (ret) {
		pr_err("enabling the front camera data path failed: %pe\n", ERR_PTR(ret));
		lcp_cleanup();
		return ret;
	}

	list_for_each_entry(ce, &lcp_cs.entries, node)
		pr_info("stream: %pOF enabled\n", ce->np);
	pr_info("stream: front camera data path on, load the V4L2 drivers\n");

	return 0;
}

static void lcp_wait_bound(void)
{
	unsigned int i, t;
	bool all;

	for (t = 0; t < LCP_BIND_WAIT_MS; t += LCP_POLL_MS) {
		all = true;
		mutex_lock(&lcp_lock);
		for (i = 0; i < LCP_NCAMS; i++)
			if (!lcp_cams[i].client)
				all = false;
		mutex_unlock(&lcp_lock);
		if (all)
			return;
		msleep(LCP_POLL_MS);
	}
	pr_warn("not all sensors bound after %u ms\n", LCP_BIND_WAIT_MS);
}

static void lcp_cleanup(void)
{
	if (lcp_driver_added)
		i2c_del_driver(&lcp_driver);
	lcp_driver_added = false;
	if (lcp_mipi_sel)
		gpiod_put(lcp_mipi_sel);
	lcp_mipi_sel = NULL;
	if (lcp_cs_applied)
		of_changeset_revert(&lcp_cs);
	lcp_cs_applied = false;
	of_changeset_destroy(&lcp_cs);
}

static int __init lcp_init(void)
{
	struct device_node *nodes[LCP_NCAMS] = { };
	struct device_node *mux;
	unsigned int i;
	int ret;

	pr_info("start (cams=0x%02x mclk_all=%d stream=%d)\n", cams, mclk_all, stream);
	of_changeset_init(&lcp_cs);

	if (stream)
		return lcp_stream_init();

	for (i = 0; i < LCP_NCAMS; i++) {
		nodes[i] = of_find_compatible_node(NULL, NULL, lcp_compatibles[i]);
		if (!nodes[i]) {
			pr_err("no %s node (boot the lisbon-kamera DTB)\n", lcp_compatibles[i]);
			ret = -ENODEV;
			goto out_put;
		}
	}

	mux = of_find_node_by_path(LCP_MIPI_SWITCH_PATH);
	if (!mux) {
		pr_err("%s missing\n", LCP_MIPI_SWITCH_PATH);
		ret = -ENODEV;
		goto out_put;
	}
	/* the switch node stays disabled; the line keeps its level until the first sequence */
	lcp_mipi_sel = fwnode_gpiod_get_index(of_fwnode_handle(mux), "mux", 0, GPIOD_ASIS,
					      "cam-mipi-switch");
	of_node_put(mux);
	if (IS_ERR(lcp_mipi_sel)) {
		ret = PTR_ERR(lcp_mipi_sel);
		lcp_mipi_sel = NULL;
		pr_err("MIPI switch GPIO: %pe\n", ERR_PTR(ret));
		goto out_put;
	}
	pr_info("MIPI switch GPIO before: %d\n", gpiod_get_raw_value_cansleep(lcp_mipi_sel));

	ret = i2c_add_driver(&lcp_driver);
	if (ret)
		goto out_put;
	lcp_driver_added = true;

	ret = lcp_build_changeset(nodes);
	if (!ret && !list_empty(&lcp_cs.entries)) {
		ret = of_changeset_apply(&lcp_cs);
		lcp_cs_applied = !ret;
	}
	if (ret) {
		pr_err("enabling the camera nodes failed: %pe\n", ERR_PTR(ret));
		goto out_put;
	}

	lcp_wait_bound();

	mutex_lock(&lcp_lock);
	lcp_park_lines();
	for (i = 0; i < LCP_NCAMS; i++)
		if ((cams & BIT(lcp_descs[i]->idx)) && lcp_cams[i].client)
			lcp_run(&lcp_cams[i]);
	lcp_summary();
	mutex_unlock(&lcp_lock);

	for (i = 0; i < LCP_NCAMS; i++)
		of_node_put(nodes[i]);

	return 0;

out_put:
	lcp_cleanup();
	for (i = 0; i < LCP_NCAMS; i++)
		of_node_put(nodes[i]);

	return ret;
}
module_init(lcp_init);

static void __exit lcp_exit(void)
{
	lcp_cleanup();
	pr_info("camera nodes disabled again\n");
}
module_exit(lcp_exit);

MODULE_DESCRIPTION("Motorola Moto G60s (lisbon) camera power and chip ID probe");
MODULE_LICENSE("GPL");
