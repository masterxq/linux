// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * ILITEK TDDI (ILI7807S and related ILI9881/ILI9882) SPI touchscreen driver
 *
 * The touch part of these touch-and-display-driver-integration chips has no
 * flash: after every touch reset the host has to download the firmware into
 * the chip's instruction/data RAM over SPI ("host download") using the
 * so-called ICE debug mode, then start the code and read touch reports.
 *
 * Protocol, register addresses and firmware (Intel HEX with 0xAF/0xB0 block
 * records) handling are derived from the ILITEK "v3" vendor driver
 * (ilitek_v3_*.c, Copyright (C) 2011 ILI Technology Corporation, author
 * Dicky Chiang, GPL-2.0-or-later) as shipped in Motorola's kernel modules
 * (ilitek_mtk_mmi).
 * This is a reduced re-implementation: SPI transport, firmware download,
 * multitouch protocol B. No MP test, gestures, proximity, proc/sysfs nodes,
 * ESD worker or panel notifiers.
 *
 * Display coupling: only the touch reset line is driven here. The display
 * reset is a separate line and is left alone, so the bootloader-initialised
 * panel keeps running. With a "panel" phandle the driver follows the panel
 * (drm_panel_follower): the display reset and bias of a panel power cycle
 * also reset the touch part of the chip and wipe its firmware RAM, so touch
 * stops before the panel is unprepared and downloads the firmware again once
 * the panel is enabled. A panel that only appears later (display stack
 * started at run time) is looked up again for a while.
 */

#include <linux/bitfield.h>
#include <linux/crc32.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/gpio/consumer.h>
#include <linux/hex.h>
#include <linux/input.h>
#include <linux/input/mt.h>
#include <linux/input/touchscreen.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/regulator/consumer.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/unaligned.h>
#include <linux/vmalloc.h>

#include <drm/drm_panel.h>

#define ILI_SPI_WRITE		0x82
#define ILI_SPI_READ		0x83
#define ILI_SPI_ACK		0xa3

/* ICE mode (debug port) commands, sent after ILI_SPI_WRITE */
#define ILI_ICE_RW		0x25	/* followed by 24-bit LE address */
static const u8 ili_ice_open[] = { 0x25, 0x62, 0x10, 0x18 };
static const u8 ili_ice_close[] = { 0x1b, 0x62, 0x10, 0x18 };

/* Registers reachable in ICE mode */
#define ILI_REG_PID		0x4009c
#define ILI_REG_OTP_ID		0x400a0
#define ILI_REG_ANA_ID		0x400a4
#define ILI_REG_CODE_RESET	0x40040
#define ILI_CODE_RESET_KEY	0xae
#define ILI_REG_WDT9_DUMMY2	0x51024
#define ILI_DUMMY_PATTERN	0xa55a5aa5

/* DMA engine used to CRC the downloaded RAM content */
#define ILI_REG_DMA1_SRC1	0x072104
#define ILI_REG_DMA1_SRC1_FMT	0x072108
#define ILI_REG_DMA1_DEST	0x072114
#define ILI_REG_DMA1_DEST_FMT	0x072118
#define ILI_REG_DMA1_SIZE	0x07211c
#define ILI_REG_DMA1_CTRL	0x072100
#define ILI_REG_CRC_EN		0x041016
#define ILI_REG_CRC_SEL		0x041017
#define ILI_REG_CRC_RESULT	0x04101c
#define ILI_REG_INT_FLAG	0x048006

/* Firmware commands (normal mode) */
#define ILI_CMD_MODE_CTRL	0xf0
#define ILI_DATA_FORMAT_DEMO	0x00

/* Report packets */
#define ILI_PID_DEMO		0x5a
#define ILI_PID_DEMO_HIRES	0x5b
#define ILI_PID_INFO_HEADER	0xb7
#define ILI_DEMO_LEN		43	/* low resolution, no customer data */
#define ILI_DEMO_LEN_5B_LOW	62
#define ILI_DEMO_LEN_HIRES	72
#define ILI_CUSTOMER_LEN	50
#define ILI_DEMO_INFO_LEN	3
#define ILI_MAX_FINGERS		10
#define ILI_LOWRES_RANGE	2048
#define ILI_HIRES_RANGE		8192
#define ILI_CUSTOMER_TYPE_OFF	0x1f

/* Firmware image layout */
#define ILI_FW_MAX_SIZE		(256 * 1024)
#define ILI_FW_BLOCKS		17
#define ILI_BLK_AP		1
#define ILI_BLK_DATA		2
#define ILI_BLK_TUNING		3
#define ILI_DLM_START		0x20610
#define ILI_INFO_OFFSET		0x4f	/* fw info lies at AP end - 0x4f */
#define ILI_INFO_LEN		75
#define ILI_CORE_VER_1430	0x01040300
#define ILI_CORE_VER_1470	0x01040700

#define ILI_XFER_CHUNK		2048	/* host download chunk, as vendor "split" mode */
#define ILI_BUF_SIZE		(ILI_XFER_CHUNK + 16)

struct ili_block {
	u32 start, end, len;	/* in firmware image */
	u32 mem;		/* destination in chip RAM */
	bool fixed;
};

struct ili_tddi {
	struct spi_device *spi;
	struct input_dev *input;
	struct gpio_desc *reset_gpio;
	struct touchscreen_properties prop;
	struct mutex lock;	/* serialises SPI sequences */
	struct work_struct recover_work;
	const char *fw_name;

	/* panel follower */
	struct drm_panel_follower follower;
	struct delayed_work follow_work;	/* panel not registered yet */
	struct work_struct resume_work;		/* firmware download after a panel power cycle */
	unsigned int follow_tries;
	bool following;
	bool panel_off;				/* the panel was unprepared since the last run */

	u8 *tx;
	u8 *rx;
	bool ice;

	u32 pid;
	u32 core_ver;
	u32 fw_ver;
	u16 panel_w, panel_h;
	bool trans_xy;
	bool hires;
	bool customer;
	unsigned int report_len;
	unsigned int ack_errors;
	unsigned long touched;
	bool running;
};

static bool probe_only;
module_param(probe_only, bool, 0444);
MODULE_PARM_DESC(probe_only, "Only reset the controller and read its chip id, no firmware download");

/* ------------------------------------------------------------------ SPI */

static int ili_spi_xfer(struct ili_tddi *ts, size_t len, bool read)
{
	struct spi_transfer t = {
		.tx_buf = ts->tx,
		.rx_buf = read ? ts->rx : NULL,
		.len = len,
	};

	if (len > ILI_BUF_SIZE)
		return -EINVAL;
	return spi_sync_transfer(ts->spi, &t, 1);
}

/* Plain write: ICE framing is 0x82 + payload, firmware framing adds a length */
static int ili_write(struct ili_tddi *ts, const u8 *data, size_t len)
{
	size_t hdr = ts->ice ? 1 : 3;

	if (len + hdr > ILI_BUF_SIZE)
		return -EINVAL;
	ts->tx[0] = ILI_SPI_WRITE;
	if (!ts->ice) {
		ts->tx[1] = len >> 8;
		ts->tx[2] = len & 0xff;
	}
	memcpy(ts->tx + hdr, data, len);
	return ili_spi_xfer(ts, len + hdr, false);
}

/* Read: 0x83 then clock out data; outside ICE mode two extra bytes, 2nd is ACK */
static int ili_read(struct ili_tddi *ts, u8 *data, size_t len)
{
	size_t off = ts->ice ? 0 : 2;
	int ret;

	if (len + off + 1 > ILI_BUF_SIZE)
		return -EINVAL;
	memset(ts->tx, 0, len + off + 1);
	ts->tx[0] = ILI_SPI_READ;
	ret = ili_spi_xfer(ts, len + off + 1, true);
	if (ret)
		return ret;
	if (!ts->ice && ts->rx[1] != ILI_SPI_ACK)
		return -EPROTO;
	memcpy(data, ts->rx + off + 1, len);
	return 0;
}

static int ili_ice_enable(struct ili_tddi *ts)
{
	int ret;

	ts->ice = true;
	ret = ili_write(ts, ili_ice_open, sizeof(ili_ice_open));
	if (ret)
		ts->ice = false;
	return ret;
}

static int ili_ice_disable(struct ili_tddi *ts)
{
	int ret;

	ret = ili_write(ts, ili_ice_close, sizeof(ili_ice_close));
	if (!ret)
		ts->ice = false;
	return ret;
}

static int ili_ice_write(struct ili_tddi *ts, u32 addr, u32 val, int len)
{
	u8 buf[8];
	int i;

	buf[0] = ILI_ICE_RW;
	buf[1] = addr;
	buf[2] = addr >> 8;
	buf[3] = addr >> 16;
	for (i = 0; i < len; i++)
		buf[4 + i] = val >> (8 * i);
	return ili_write(ts, buf, 4 + len);
}

static int ili_ice_read(struct ili_tddi *ts, u32 addr, u32 *val, int len)
{
	u8 buf[4] = { ILI_ICE_RW, addr, addr >> 8, addr >> 16 };
	int ret, i;

	ret = ili_write(ts, buf, sizeof(buf));
	if (ret)
		return ret;
	ret = ili_read(ts, buf, len);
	if (ret)
		return ret;
	*val = 0;
	for (i = 0; i < len; i++)
		*val |= buf[i] << (8 * i);
	return 0;
}

/* -------------------------------------------------------------- control */

static void ili_hw_reset(struct ili_tddi *ts)
{
	/* Same timing as the vendor driver (ili_tp_reset, rst_edge_delay 10 ms) */
	gpiod_set_value_cansleep(ts->reset_gpio, 0);
	usleep_range(1000, 1500);
	gpiod_set_value_cansleep(ts->reset_gpio, 1);
	usleep_range(5000, 6000);
	gpiod_set_value_cansleep(ts->reset_gpio, 0);
	msleep(10);
	ts->ice = false;
}

static int ili_identify(struct ili_tddi *ts)
{
	u32 val = 0, otp = 0, ana = 0;
	int ret, i;

	ret = ili_ice_enable(ts);
	if (ret)
		return ret;

	for (i = 0; i < 3; i++) {
		ret = ili_ice_write(ts, ILI_REG_WDT9_DUMMY2, ILI_DUMMY_PATTERN, 4);
		if (!ret)
			ret = ili_ice_read(ts, ILI_REG_WDT9_DUMMY2, &val, 4);
		if (!ret && (val == ILI_DUMMY_PATTERN || val == (u32)-ILI_DUMMY_PATTERN))
			break;
		msleep(30);
	}
	if (i == 3) {
		dev_err(&ts->spi->dev, "dummy register check failed (read %08x, ret %d)\n",
			val, ret);
		return ret ?: -ENODEV;
	}

	ret = ili_ice_read(ts, ILI_REG_PID, &ts->pid, 4);
	if (!ret)
		ret = ili_ice_read(ts, ILI_REG_OTP_ID, &otp, 4);
	if (!ret)
		ret = ili_ice_read(ts, ILI_REG_ANA_ID, &ana, 4);
	if (ret)
		return ret;

	dev_info(&ts->spi->dev, "chip pid %08x (id %04x type %02x ver %02x), otp %02x, ana %02x\n",
		 ts->pid, ts->pid >> 16, (ts->pid >> 8) & 0xff, ts->pid & 0xff,
		 otp & 0xff, ana & 0xff);

	switch (ts->pid >> 16) {
	case 0x7807:
	case 0x9881:
	case 0x9882:
		break;
	default:
		dev_warn(&ts->spi->dev, "unexpected chip id %04x\n", ts->pid >> 16);
	}
	return 0;
}

/* -------------------------------------------------------------- firmware */

static int ili_hex(const u8 *p, int digits, u32 *out)
{
	u32 v = 0;
	int i;

	for (i = 0; i < digits; i++) {
		int d = hex_to_bin(p[i]);

		if (d < 0)
			return -EINVAL;
		v = (v << 4) | d;
	}
	*out = v;
	return 0;
}

/*
 * Parse the ILITEK Intel-HEX file. Besides data records (type 0x00) and
 * extended address records (0x02, 0x04) it carries vendor records:
 * 0xAF = block descriptor (start[24] end[24] number[8]),
 * 0xB0 = fixed RAM destination for a block (addr[24] number[8]).
 */
static int ili_parse_hex(struct device *dev, const u8 *hex, size_t size, u8 *img,
			 struct ili_block *blk)
{
	size_t i = 0;
	u32 ext = 0;
	bool have_af = false;

	memset(img, 0xff, ILI_FW_MAX_SIZE);
	memset(blk, 0, sizeof(*blk) * ILI_FW_BLOCKS);

	while (i < size) {
		u32 len, addr, type, num, v1, v2, k;

		if (hex[i] == '\r' || hex[i] == '\n') {
			i++;
			continue;
		}
		if (hex[i] != ':' || i + 11 > size)
			return -EINVAL;
		if (ili_hex(&hex[i + 1], 2, &len) || ili_hex(&hex[i + 3], 4, &addr) ||
		    ili_hex(&hex[i + 7], 2, &type))
			return -EINVAL;
		if (i + 11 + len * 2 > size)
			return -EINVAL;

		switch (type) {
		case 0x00:
			addr += ext << 16;
			if (addr + len > ILI_FW_MAX_SIZE)
				return -EINVAL;
			for (k = 0; k < len; k++) {
				if (ili_hex(&hex[i + 9 + 2 * k], 2, &v1))
					return -EINVAL;
				img[addr + k] = v1;
			}
			break;
		case 0x02:
			if (ili_hex(&hex[i + 9], 4, &v1))
				return -EINVAL;
			ext = v1 >> 12;
			break;
		case 0x04:
			if (ili_hex(&hex[i + 9], 4, &ext))
				return -EINVAL;
			break;
		case 0xaf:
			if (len < 7 || ili_hex(&hex[i + 9], 6, &v1) ||
			    ili_hex(&hex[i + 15], 6, &v2) || ili_hex(&hex[i + 21], 2, &num))
				return -EINVAL;
			if (num >= ILI_FW_BLOCKS || v2 < v1 || v2 >= ILI_FW_MAX_SIZE)
				return -EINVAL;
			blk[num].start = v1;
			blk[num].end = v2;
			blk[num].len = v2 - v1 + 1;
			blk[num].fixed = false;
			have_af = true;
			break;
		case 0xb0:
			if (!have_af)
				break;
			if (len < 4 || ili_hex(&hex[i + 9], 6, &v1) || ili_hex(&hex[i + 15], 2, &num))
				return -EINVAL;
			if (num >= ILI_FW_BLOCKS)
				return -EINVAL;
			blk[num].mem = v1;
			blk[num].fixed = true;
			break;
		default:	/* 0x01 EOF, 0x03 start address */
			break;
		}
		i += 11 + len * 2;
	}

	if (!have_af || !blk[ILI_BLK_AP].len) {
		dev_err(dev, "firmware has no block table\n");
		return -EINVAL;
	}

	/* RAM destinations like ilitek_tddi_fw_update_block_info() */
	if (!blk[ILI_BLK_AP].fixed)
		blk[ILI_BLK_AP].mem = 0;
	if (!blk[ILI_BLK_DATA].fixed)
		blk[ILI_BLK_DATA].mem = ILI_DLM_START;
	if (!blk[ILI_BLK_TUNING].fixed)
		blk[ILI_BLK_TUNING].mem = blk[ILI_BLK_DATA].mem + blk[ILI_BLK_DATA].len;

	/* Each block ends with its big-endian CRC32 (MSB first, init ~0, no xorout) */
	for (i = 0; i < ILI_FW_BLOCKS; i++) {
		u32 crc, stored;

		if (!blk[i].len)
			continue;
		if (blk[i].len < 8)
			return -EINVAL;
		crc = crc32_be(~0U, img + blk[i].start, blk[i].len - 4);
		stored = get_unaligned_be32(img + blk[i].end - 3);
		if (crc != stored) {
			dev_err(dev, "firmware block %zu corrupt (crc %08x, stored %08x)\n",
				i, crc, stored);
			return -EINVAL;
		}
	}
	return 0;
}

static void ili_parse_info(struct ili_tddi *ts, const u8 *img, const struct ili_block *blk)
{
	const u8 *info = img + blk[ILI_BLK_AP].end - ILI_INFO_OFFSET;
	u8 rib0 = info[0], rib1 = info[1];
	unsigned int res_mode, cust;

	ts->core_ver = get_unaligned_be32(&info[68]);
	ts->fw_ver = get_unaligned_be32(&info[48]);
	ts->panel_w = get_unaligned_le16(&info[16]);
	ts->panel_h = get_unaligned_le16(&info[18]);

	/* Report info block: byte0 bit0 = report by pixel; byte1 [2:0] resolution, [7:3] customer */
	if (ts->core_ver >= ILI_CORE_VER_1470) {
		res_mode = rib1 & 0x07;
		cust = rib1 >> 3;
	} else {
		res_mode = 0;
		cust = ILI_CUSTOMER_TYPE_OFF;
	}
	ts->hires = res_mode == 1;
	ts->customer = cust != ILI_CUSTOMER_TYPE_OFF;
	ts->trans_xy = ts->core_ver >= ILI_CORE_VER_1430 && (rib0 & BIT(0));

	if (!ts->hires && !ts->customer)
		ts->report_len = ILI_DEMO_LEN;
	else if (!ts->hires)
		ts->report_len = ILI_DEMO_LEN_5B_LOW + ILI_CUSTOMER_LEN;
	else
		ts->report_len = ILI_DEMO_LEN_HIRES + (ts->customer ? ILI_CUSTOMER_LEN : 0);

	if (!ts->panel_w || !ts->panel_h || ts->panel_w > 8192 || ts->panel_h > 8192) {
		ts->panel_w = 1080;
		ts->panel_h = 2460;
	}

	dev_info(&ts->spi->dev,
		 "firmware %08x core %08x, panel %ux%u, %s res%s, pixel coords %s, report %u bytes\n",
		 ts->fw_ver, ts->core_ver, ts->panel_w, ts->panel_h,
		 ts->hires ? "high" : "low", ts->customer ? " +customer" : "",
		 ts->trans_xy ? "yes" : "no", ts->report_len);
}

static int ili_dma_crc(struct ili_tddi *ts, u32 start, u32 size, u32 *crc)
{
	static const struct { u32 reg, val; int len; } seq[] = {
		{ ILI_REG_DMA1_SRC1, 0, 4 },		/* patched below */
		{ ILI_REG_DMA1_SRC1_FMT, 0x80000001, 4 },
		{ ILI_REG_DMA1_DEST, 0x0002725c, 4 },
		{ ILI_REG_DMA1_DEST_FMT, 0x80000000, 4 },
		{ ILI_REG_DMA1_SIZE, 0, 4 },		/* patched below */
		{ ILI_REG_CRC_EN, 0x00, 1 },
		{ ILI_REG_CRC_SEL, 0x03, 1 },
		{ ILI_REG_CRC_EN, 0x01, 1 },
		{ ILI_REG_DMA1_CTRL, 0x02000000, 4 },	/* stop */
		{ ILI_REG_INT_FLAG, 0x02, 1 },		/* clear */
		{ ILI_REG_DMA1_CTRL, 0x01000000, 4 },	/* start */
	};
	u32 val = 0;
	int i, ret;

	for (i = 0; i < ARRAY_SIZE(seq); i++) {
		u32 v = seq[i].val;

		if (seq[i].reg == ILI_REG_DMA1_SRC1)
			v = start;
		else if (seq[i].reg == ILI_REG_DMA1_SIZE)
			v = size;
		ret = ili_ice_write(ts, seq[i].reg, v, seq[i].len);
		if (ret)
			return ret;
	}

	for (i = 0; i < 50; i++) {
		usleep_range(1000, 1200);
		ret = ili_ice_read(ts, ILI_REG_INT_FLAG, &val, 1);
		if (ret)
			return ret;
		if (val & 0x02)
			break;
	}
	if (i == 50)
		return -ETIMEDOUT;

	return ili_ice_read(ts, ILI_REG_CRC_RESULT, crc, 4);
}

static int ili_program_block(struct ili_tddi *ts, const u8 *img, const struct ili_block *b,
			     const char *name)
{
	u32 off, crc_fw, crc_hw = 0;
	int ret;

	for (off = 0; off < b->len; off += ILI_XFER_CHUNK) {
		u32 n = min_t(u32, ILI_XFER_CHUNK, b->len - off);
		u32 addr = b->mem + off;
		u32 padded = ALIGN(n, 4);

		/* 0x82 0x25 addr[0..2] data..., direct (vendor iram_program) */
		ts->tx[0] = ILI_SPI_WRITE;
		ts->tx[1] = ILI_ICE_RW;
		ts->tx[2] = addr;
		ts->tx[3] = addr >> 8;
		ts->tx[4] = addr >> 16;
		memcpy(ts->tx + 5, img + b->start + off, n);
		memset(ts->tx + 5 + n, 0xff, padded - n);
		ret = ili_spi_xfer(ts, padded + 5, false);
		if (ret)
			return ret;
	}

	crc_fw = crc32_be(~0U, img + b->start, b->len - 4);
	ret = ili_dma_crc(ts, b->mem, b->len - 4, &crc_hw);
	if (ret)
		return ret;
	if (crc_fw != crc_hw) {
		dev_err(&ts->spi->dev, "%s: RAM crc %08x != image crc %08x\n",
			name, crc_hw, crc_fw);
		return -EIO;
	}
	dev_dbg(&ts->spi->dev, "%s: %u bytes at 0x%05x, crc %08x ok\n",
		name, b->len, b->mem, crc_hw);
	return 0;
}

static int ili_download(struct ili_tddi *ts, const u8 *img, const struct ili_block *blk)
{
	static const struct { int idx; const char *name; } order[] = {
		{ ILI_BLK_AP, "AP" }, { ILI_BLK_DATA, "DATA" }, { ILI_BLK_TUNING, "TUNING" },
	};
	static const u8 demo[] = { ILI_CMD_MODE_CTRL, ILI_DATA_FORMAT_DEMO };
	int i, ret;

	ili_hw_reset(ts);
	ret = ili_ice_enable(ts);
	if (ret)
		return ret;

	for (i = 0; i < ARRAY_SIZE(order); i++) {
		const struct ili_block *b = &blk[order[i].idx];

		if (!b->len)
			continue;
		ret = ili_program_block(ts, img, b, order[i].name);
		if (ret)
			return ret;
	}

	/* Restart the MCU on the new code, leave ICE mode */
	ret = ili_ice_write(ts, ILI_REG_CODE_RESET, ILI_CODE_RESET_KEY, 1);
	if (ret)
		return ret;
	ret = ili_ice_disable(ts);
	if (ret)
		return ret;
	msleep(50);

	/* Select the demo (finger report) data format */
	return ili_write(ts, demo, sizeof(demo));
}

static int ili_load_firmware(struct ili_tddi *ts)
{
	struct device *dev = &ts->spi->dev;
	const struct firmware *fw;
	struct ili_block blk[ILI_FW_BLOCKS];
	u8 *img;
	int ret, tries;

	ret = request_firmware(&fw, ts->fw_name, dev);
	if (ret) {
		dev_err(dev, "cannot load firmware %s: %d\n", ts->fw_name, ret);
		return ret;
	}

	img = vmalloc(ILI_FW_MAX_SIZE);
	if (!img) {
		ret = -ENOMEM;
		goto out_release;
	}

	ret = ili_parse_hex(dev, fw->data, fw->size, img, blk);
	if (ret) {
		dev_err(dev, "firmware %s: invalid ILITEK hex (%d)\n", ts->fw_name, ret);
		goto out_free;
	}
	ili_parse_info(ts, img, blk);

	for (tries = 0; tries < 3; tries++) {
		ret = ili_download(ts, img, blk);
		if (!ret)
			break;
		dev_warn(dev, "firmware download attempt %d failed: %d\n", tries + 1, ret);
	}
	if (ret) {
		ili_hw_reset(ts);
		goto out_free;
	}
	dev_info(dev, "firmware %s downloaded\n", ts->fw_name);

out_free:
	vfree(img);
out_release:
	release_firmware(fw);
	return ret;
}

/* ---------------------------------------------------------------- input */

static void ili_report(struct ili_tddi *ts, const u8 *buf)
{
	unsigned long now = 0;
	int i, stride = ts->hires ? 5 : 4;
	int base = (ts->hires || ts->customer) ? ILI_DEMO_INFO_LEN : 0;

	for (i = 0; i < ILI_MAX_FINGERS; i++) {
		const u8 *p = buf + base + stride * i + 1;
		u32 x, y;

		if (ts->hires) {
			if (p[0] == 0xff && p[1] == 0xff && p[2] == 0xff && p[3] == 0xff)
				continue;
			x = get_unaligned_be16(p);
			y = get_unaligned_be16(p + 2);
		} else {
			if (p[0] == 0xff && p[1] == 0xff && p[2] == 0xff)
				continue;
			x = ((p[0] & 0xf0) << 4) | p[1];
			y = ((p[0] & 0x0f) << 8) | p[2];
		}

		if (!ts->trans_xy) {
			u32 range = ts->hires ? ILI_HIRES_RANGE : ILI_LOWRES_RANGE;

			x = x * ts->panel_w / range;
			y = y * ts->panel_h / range;
		}

		input_mt_slot(ts->input, i);
		input_mt_report_slot_state(ts->input, MT_TOOL_FINGER, true);
		touchscreen_report_pos(ts->input, &ts->prop, x, y, true);
		now |= BIT(i);
	}

	for_each_set_bit(i, &ts->touched, ILI_MAX_FINGERS) {
		if (now & BIT(i))
			continue;
		input_mt_slot(ts->input, i);
		input_mt_report_slot_inactive(ts->input);
	}
	ts->touched = now;

	input_mt_sync_frame(ts->input);
	input_sync(ts->input);
}

static irqreturn_t ili_irq(int irq, void *data)
{
	struct ili_tddi *ts = data;
	u8 buf[ILI_DEMO_LEN_HIRES + ILI_CUSTOMER_LEN + ILI_DEMO_INFO_LEN + 1];
	const u8 *pkt = buf;
	unsigned int len = ts->report_len;
	u8 sum = 0;
	int ret, i;

	if (!READ_ONCE(ts->running) || len > sizeof(buf))
		return IRQ_HANDLED;

	mutex_lock(&ts->lock);
	ret = ili_read(ts, buf, len);
	mutex_unlock(&ts->lock);

	if (ret == -EPROTO) {
		dev_warn_ratelimited(&ts->spi->dev, "no SPI ACK (got %02x)\n", ts->rx[1]);
		if (++ts->ack_errors >= 5) {
			ts->ack_errors = 0;
			WRITE_ONCE(ts->running, false);
			schedule_work(&ts->recover_work);
		}
		return IRQ_HANDLED;
	} else if (ret) {
		dev_warn_ratelimited(&ts->spi->dev, "report read failed: %d\n", ret);
		return IRQ_HANDLED;
	}
	ts->ack_errors = 0;

	for (i = 0; i < len - 1; i++)
		sum += buf[i];
	if ((u8)-sum != buf[len - 1]) {
		dev_dbg_ratelimited(&ts->spi->dev, "checksum mismatch, packet %*ph\n", 8, buf);
		return IRQ_HANDLED;
	}

	if (pkt[0] == ILI_PID_INFO_HEADER)
		pkt += ILI_DEMO_INFO_LEN;

	switch (pkt[0]) {
	case ILI_PID_DEMO:
	case ILI_PID_DEMO_HIRES:
		ili_report(ts, pkt);
		break;
	default:
		dev_dbg_ratelimited(&ts->spi->dev, "ignoring packet id %02x\n", pkt[0]);
		break;
	}
	return IRQ_HANDLED;
}

static void ili_release_all(struct ili_tddi *ts)
{
	int i;

	for_each_set_bit(i, &ts->touched, ILI_MAX_FINGERS) {
		input_mt_slot(ts->input, i);
		input_mt_report_slot_inactive(ts->input);
	}
	ts->touched = 0;
	input_mt_sync_frame(ts->input);
	input_sync(ts->input);
}

static void ili_recover_work(struct work_struct *work)
{
	struct ili_tddi *ts = container_of(work, struct ili_tddi, recover_work);
	int ret;

	dev_warn(&ts->spi->dev, "controller lost, reloading firmware\n");
	disable_irq(ts->spi->irq);
	ili_release_all(ts);
	mutex_lock(&ts->lock);
	ret = ili_load_firmware(ts);
	mutex_unlock(&ts->lock);
	if (!ret)
		WRITE_ONCE(ts->running, true);
	enable_irq(ts->spi->irq);
}

/* ------------------------------------------------------- panel follower */

#define ILI_FOLLOW_RETRY_MS	1000
#define ILI_FOLLOW_TRIES	300

static int ili_panel_unpreparing(struct drm_panel_follower *follower)
{
	struct ili_tddi *ts = container_of(follower, struct ili_tddi, follower);

	/* a reload still queued from the last enable is pointless now; IRQ stays off then */
	cancel_work_sync(&ts->resume_work);
	if (ts->panel_off)
		return 0;
	/* the panel is about to lose bias and get reset: stop before the chip vanishes */
	WRITE_ONCE(ts->running, false);
	cancel_work_sync(&ts->recover_work);
	disable_irq(ts->spi->irq);
	ili_release_all(ts);
	ts->ack_errors = 0;
	ts->panel_off = true;
	dev_dbg(&ts->spi->dev, "panel off, touch stopped\n");
	return 0;
}

static int ili_panel_enabled(struct drm_panel_follower *follower)
{
	struct ili_tddi *ts = container_of(follower, struct ili_tddi, follower);

	/* nothing to do if the panel kept running (takeover, backlight-only off) */
	if (ts->panel_off)
		schedule_work(&ts->resume_work);
	return 0;
}

static const struct drm_panel_follower_funcs ili_follower_funcs = {
	.panel_unpreparing = ili_panel_unpreparing,
	.panel_enabled = ili_panel_enabled,
};

static void ili_resume_work(struct work_struct *work)
{
	struct ili_tddi *ts = container_of(work, struct ili_tddi, resume_work);
	int ret;

	mutex_lock(&ts->lock);
	ret = ili_load_firmware(ts);	/* includes the touch reset */
	mutex_unlock(&ts->lock);
	ts->panel_off = false;
	if (ret)
		dev_err(&ts->spi->dev, "firmware reload after the panel power cycle: %d\n", ret);
	else
		WRITE_ONCE(ts->running, true);
	/* with the IRQ on, a failure is retried through the ACK error recovery */
	enable_irq(ts->spi->irq);
}

static void ili_follow_work(struct work_struct *work)
{
	struct ili_tddi *ts = container_of(to_delayed_work(work), struct ili_tddi, follow_work);
	int ret;

	ret = drm_panel_add_follower(&ts->spi->dev, &ts->follower);
	if (!ret) {
		ts->following = true;
		dev_info(&ts->spi->dev, "following the panel\n");
	} else if (ret == -EPROBE_DEFER && ++ts->follow_tries < ILI_FOLLOW_TRIES) {
		schedule_delayed_work(&ts->follow_work, msecs_to_jiffies(ILI_FOLLOW_RETRY_MS));
	} else {
		dev_info(&ts->spi->dev, "not following the panel (%d), only the missing-ACK recovery remains\n",
			 ret);
	}
}

static int ili_input_setup(struct ili_tddi *ts)
{
	struct input_dev *input;
	int ret;

	input = devm_input_allocate_device(&ts->spi->dev);
	if (!input)
		return -ENOMEM;

	input->name = "ILITEK TDDI touchscreen";
	input->phys = "spi/input0";
	input->id.bustype = BUS_SPI;
	input->id.vendor = 0x222a;	/* ILITEK USB vendor id */
	input->id.product = ts->pid >> 16;

	input_set_abs_params(input, ABS_MT_POSITION_X, 0, ts->panel_w - 1, 0, 0);
	input_set_abs_params(input, ABS_MT_POSITION_Y, 0, ts->panel_h - 1, 0, 0);
	touchscreen_parse_properties(input, true, &ts->prop);

	ret = input_mt_init_slots(input, ILI_MAX_FINGERS, INPUT_MT_DIRECT | INPUT_MT_DROP_UNUSED);
	if (ret)
		return ret;

	ts->input = input;
	return input_register_device(input);
}

static int ili_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct ili_tddi *ts;
	int ret;

	if (spi->controller->flags & SPI_CONTROLLER_HALF_DUPLEX)
		return dev_err_probe(dev, -EINVAL, "full duplex SPI required\n");

	ts = devm_kzalloc(dev, sizeof(*ts), GFP_KERNEL);
	if (!ts)
		return -ENOMEM;
	ts->spi = spi;
	mutex_init(&ts->lock);
	INIT_WORK(&ts->recover_work, ili_recover_work);
	INIT_WORK(&ts->resume_work, ili_resume_work);
	INIT_DELAYED_WORK(&ts->follow_work, ili_follow_work);
	ts->follower.funcs = &ili_follower_funcs;
	spi_set_drvdata(spi, ts);

	ts->tx = devm_kzalloc(dev, ILI_BUF_SIZE, GFP_KERNEL);
	ts->rx = devm_kzalloc(dev, ILI_BUF_SIZE, GFP_KERNEL);
	if (!ts->tx || !ts->rx)
		return -ENOMEM;

	ts->fw_name = "ilitek/ili7807s.hex";
	device_property_read_string(dev, "firmware-name", &ts->fw_name);

	spi->mode = SPI_MODE_0;
	spi->bits_per_word = 8;
	ret = spi_setup(spi);
	if (ret)
		return dev_err_probe(dev, ret, "spi_setup failed\n");

	ret = devm_regulator_get_enable_optional(dev, "vdd");
	if (ret && ret != -ENODEV)
		return dev_err_probe(dev, ret, "vdd supply\n");
	ret = devm_regulator_get_enable_optional(dev, "vio");
	if (ret && ret != -ENODEV)
		return dev_err_probe(dev, ret, "vio supply\n");

	/* Only the touch reset line, never the display reset */
	ts->reset_gpio = devm_gpiod_get(dev, "reset", GPIOD_OUT_LOW);
	if (IS_ERR(ts->reset_gpio))
		return dev_err_probe(dev, PTR_ERR(ts->reset_gpio), "reset gpio\n");

	mutex_lock(&ts->lock);
	ili_hw_reset(ts);
	ret = ili_identify(ts);
	if (!ret && probe_only) {
		ili_ice_disable(ts);
		mutex_unlock(&ts->lock);
		dev_info(dev, "probe_only set, stopping after chip id\n");
		return 0;
	}
	if (!ret)
		ret = ili_load_firmware(ts);
	mutex_unlock(&ts->lock);
	if (ret)
		return dev_err_probe(dev, ret, "controller init failed\n");

	ret = ili_input_setup(ts);
	if (ret)
		return dev_err_probe(dev, ret, "input setup\n");

	WRITE_ONCE(ts->running, true);
	ret = devm_request_threaded_irq(dev, spi->irq, NULL, ili_irq, IRQF_ONESHOT,
					dev_name(dev), ts);
	if (ret) {
		WRITE_ONCE(ts->running, false);
		return dev_err_probe(dev, ret, "irq %d\n", spi->irq);
	}

	/* the panel may register later than us (or never): never defer the probe on it */
	if (drm_is_panel_follower(dev))
		schedule_delayed_work(&ts->follow_work, 0);
	return 0;
}

static void ili_remove(struct spi_device *spi)
{
	struct ili_tddi *ts = spi_get_drvdata(spi);

	cancel_delayed_work_sync(&ts->follow_work);
	if (ts->following)
		drm_panel_remove_follower(&ts->follower);
	WRITE_ONCE(ts->running, false);
	cancel_work_sync(&ts->resume_work);
	cancel_work_sync(&ts->recover_work);
}

static const struct of_device_id ili_of_match[] = {
	{ .compatible = "ilitek,ili7807s" },
	{ }
};
MODULE_DEVICE_TABLE(of, ili_of_match);

static const struct spi_device_id ili_spi_ids[] = {
	{ "ili7807s" },
	{ }
};
MODULE_DEVICE_TABLE(spi, ili_spi_ids);

static struct spi_driver ili_driver = {
	.driver = {
		.name = "ilitek-tddi-spi",
		.of_match_table = ili_of_match,
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
	},
	.id_table = ili_spi_ids,
	.probe = ili_probe,
	.remove = ili_remove,
};
module_spi_driver(ili_driver);

MODULE_DESCRIPTION("ILITEK TDDI SPI touchscreen driver");
MODULE_LICENSE("GPL");
