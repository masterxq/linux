// SPDX-License-Identifier: GPL-2.0-only
/*
 * MediaTek MT6382 MIPI-DSI bridge (DSI RX -> RDMA -> DSC -> DSI TX): SPI control interface,
 * takeover of a bootloader-initialised chip.
 *
 * The MT6382 is controlled through a vendor SPI slave protocol with 32-bit little-endian register
 * addresses, sent LSB first on the wire:
 *
 *   read:  [CR, addr(4), len-1(4)] [RS] -> status & 0x06 == 0x06
 *          [RD, dummy(len)]        [RS] -> no error bits, poll RDWR_FINISH
 *   write: [CW, addr(4), len-1(4)] [RS] -> status & 0x06 == 0x06
 *          [WD, data(len)]         [RS] -> no error bits, poll RDWR_FINISH
 *   error: [WS, 0xff] clears the slave error status
 *   init:  [CT, 0x04] selects 4-byte single-wire transfers
 *
 * By default the driver does not reset, power or clock the chip and normally only issues read
 * commands (CR/RD/RS). Everything that changes the state of the chip - register writes (CW/WD), the
 * "write status" and "config type" commands - is refused unless the module parameter "allow_writes"
 * is set. The one exception is "init_on_cfg_error": if the read-only chip ID read fails in the
 * config phase, the driver runs exactly the vendor spislv_init() sequence once (CT, SPISLV_CTRL,
 * DRV_CFG0) and reads again.
 *
 * The driver stays bound even when the bridge does not answer, so that the SPI parameters can be
 * varied through debugfs.
 *
 * DRM ("drm" module parameter, default off): the driver becomes the drm_bridge between the SoC DSI
 * host and the panel. It registers itself as DSI device on the SoC host (4 lanes RGB888, sync-pulse
 * video, clock lane LP in blanking, as lk programs the link) and provides a DSI host for the panel
 * that sends commands through the TX command queue over SPI. The first enable takes the bridge over
 * as lk left it (TX started in video mode, RX in IPI mode); nothing of the bridge is reprogrammed.
 * With "drm" off no DSI device is registered, the SoC DSI host never gets a downstream device and
 * mediatek-drm never binds a card.
 *
 * The protocol, timing and limits follow MediaTek's GPL-2.0 vendor driver spi_slave.c ("SPI driver
 * for mt6382", Copyright (C) 2018 MediaTek Inc.) as shipped in the Motorola Moto G60s vendor
 * kernel, together with the SPI controller settings it hands to the vendor spi-mt65xx driver (LSB
 * first, tick delay, CS timing). The code here is a re-implementation on top of regmap.
 */

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/kernel.h>
#include <linux/minmax.h>
#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_graph.h>
#include <linux/platform_data/spi-mt65xx.h>
#include <linux/regmap.h>
#include <linux/seq_file.h>
#include <linux/sizes.h>
#include <linux/slab.h>
#include <linux/spi/spi.h>
#include <linux/unaligned.h>
#include <linux/workqueue.h>

#include <drm/drm_atomic_state_helper.h>
#include <drm/drm_bridge.h>
#include <drm/drm_mipi_dsi.h>
#include <drm/drm_modes.h>
#include <drm/drm_of.h>
#include <drm/drm_panel.h>
#include <video/mipi_display.h>

/* SPI slave commands */
#define MT6382_CMD_RS			0x06	/* read status */
#define MT6382_CMD_WS			0x08	/* write status */
#define MT6382_CMD_CR			0x0a	/* config read */
#define MT6382_CMD_CW			0x0c	/* config write */
#define MT6382_CMD_WD			0x0e	/* write data */
#define MT6382_CMD_CT			0x10	/* config type */
#define MT6382_CMD_RD			0x81	/* read data */

#define MT6382_CT_4BYTE_SINGLE		0x04
#define MT6382_WS_CLEAR_ALL		0xff

/* SPI slave status byte (second byte of the RS answer) */
#define MT6382_SR_SLV_ON		BIT(0)
#define MT6382_SR_CFG_SUCCESS		BIT(1)
#define MT6382_SR_TXRX_FIFO_RDY		BIT(2)
#define MT6382_SR_RD_ERR		BIT(3)
#define MT6382_SR_WR_ERR		BIT(4)
#define MT6382_SR_RDWR_FINISH		BIT(5)
#define MT6382_SR_TIMEOUT_ERR		BIT(6)
#define MT6382_SR_CMD_ERR		BIT(7)
#define MT6382_SR_CFG_READY		(MT6382_SR_CFG_SUCCESS | MT6382_SR_TXRX_FIFO_RDY)
#define MT6382_SR_ERRORS		(MT6382_SR_RD_ERR | MT6382_SR_WR_ERR | \
					 MT6382_SR_TIMEOUT_ERR)

/* stop reading the register list after this many failures in a row */
#define MT6382_PROBE_MAX_FAILS		3

/* limits of the vendor driver */
#define MT6382_XFER_MAX			(SZ_64K - 1)
#define MT6382_FINISH_POLLS		10
#define MT6382_RETRIES			10

/*
 * Bus speed. The vendor driver starts with 6.5 MHz for both directions and only switches to 55 MHz
 * (TX) / 28 MHz (RX) after re-initialising the bridge, which this driver never does.
 */
#define MT6382_SPI_LOW_HZ		6500000
#define MT6382_SPI_MIN_HZ		100000
#define MT6382_SPI_TX_MAX_HZ		55000000
#define MT6382_SPI_RX_MAX_HZ		28000000

/* "low-speed-tick-delay" of the vendor DT */
#define MT6382_MTK_TICK_DELAY_LOW	4
/*
 * Vendor spi-mt65xx: CS setup, hold and idle time = 2 * sck_time ticks when
 * the chip config leaves them 0, i.e. 18 ticks of 109.2 MHz at 6.5 MHz.
 */
#define MT6382_CS_TIME_NS		165

/* registers */
#define MT6382_REG_CHIP_ID		0x00000
#define MT6382_CHIP_ID_MASK		GENMASK(31, 16)
#define MT6382_CHIP_ID_VAL		0x63820000
#define MT6382_REG_LDO_STATUS		0x000ac
#define MT6382_REG_SPISLV_CTRL		0x02000
#define MT6382_SPISLV_CTRL_INIT		0x00000040	/* early-trans 0 (vendor DT) */
#define MT6382_REG_IOCFG_DRV_CFG0	0x05000
#define MT6382_DRV_CFG0_SPIS_SLVO	GENMASK(23, 21)
#define MT6382_SLAVE_DRIVE_STRENGTH	3		/* vendor DT slave-drive-strength */
#define MT6382_REG_DSI_TX_START		0x21000
#define MT6382_REG_DSI_TX_MODE		0x21014
#define MT6382_REG_RX_MODE		0x23170
#define MT6382_MAX_REGISTER		0x4fffc

#define MT6382_SMALL_BUF		16

static bool allow_writes;
module_param(allow_writes, bool, 0644);
MODULE_PARM_DESC(allow_writes,
		 "Allow commands that change the bridge state (register writes, WS, CT). Default: off");

static int mt6382_drm_set(const char *val, const struct kernel_param *kp);
static const struct kernel_param_ops mt6382_drm_ops = {
	.set = mt6382_drm_set,
	.get = param_get_bool,
};
static bool drm;
module_param_cb(drm, &mt6382_drm_ops, &drm, 0644);
MODULE_PARM_DESC(drm,
		 "Build the DRM pipeline (register the DSI device on the SoC host) after drm_delay_ms. Default: off");

static uint drm_delay_ms = 20000;
module_param(drm_delay_ms, uint, 0644);
MODULE_PARM_DESC(drm_delay_ms, "Delay between switching drm on and registering the DSI device (ms)");

static bool init_on_cfg_error;
module_param(init_on_cfg_error, bool, 0444);
MODULE_PARM_DESC(init_on_cfg_error,
		 "If the read-only chip ID read fails in the config phase, run the vendor spislv_init sequence once (CT, SPISLV_CTRL, DRV_CFG0) and read again. Default: off");

/*
 * Registers read in takeover mode. The vendor driver reads all of them while the display is
 * running, so reading them does not disturb the bridge. Everything else is not readable through
 * regmap on purpose.
 */
static const u32 mt6382_known_regs[] = {
	0x00000, 0x00008, 0x00010, 0x00064, 0x00080, 0x00084,	/* SYSREG */
	0x000a0, 0x000a4, 0x000a8, 0x000ac,
	0x05000,						/* IOCFG DRV_CFG0 */
	0x14010, 0x14020, 0x1412c,				/* OCLA */
	0x20000,						/* DSC */
	0x21000, 0x2100c, 0x21010, 0x21014, 0x21018, 0x2101c,	/* DSI TX */
	0x21020, 0x21024, 0x21028, 0x2102c, 0x21050, 0x21054,
	0x21058, 0x21104, 0x21110, 0x21114, 0x21118, 0x2111c,
	0x2200c, 0x22028, 0x2202c, 0x22030, 0x2203c,		/* MIPI TX */
	0x23170, 0x23174,					/* MMSYS_CONFIG */
	0x24014,						/* RDMA */
	0x25020, 0x2502c, 0x25030,				/* MUTEX */
};

/*
 * Wider read-only snapshot (debugfs "dump"): the register blocks the vendor
 * init sequence programs (ddp_disp_bdg.c bdg_common_init/bdg_tx_init/
 * bdg_dsc_init/mipi_dsi_rx_mac_init), word by word. Only blocks that are
 * powered while the display runs; left out on purpose: EFUSE (reads may
 * trigger the fuse logic), GCE, the RX D-PHY at 0x40000 and the interrupt
 * status registers of the DSI2 device (0xd00c, 0xd200-0xd2ff), which may
 * clear on read.
 */
static const struct {
	u32 start, end;		/* end inclusive */
} mt6382_dump_ranges[] = {
	{ 0x00000, 0x000ac },	/* SYSREG */
	{ 0x02000, 0x02000 },	/* SPISLV_CTRL */
	{ 0x03000, 0x03018 },	/* TOPCKGEN */
	{ 0x04000, 0x0400c },	/* APMIXEDSYS */
	{ 0x04200, 0x04224 },
	{ 0x05000, 0x05000 },	/* IOCFG DRV_CFG0 */
	{ 0x0d000, 0x0d008 },	/* DSI2 device (RX MAC) */
	{ 0x0d010, 0x0d1fc },
	{ 0x0d300, 0x0d4ec },
	{ 0x14000, 0x14500 },	/* OCLA (RX lanes) */
	{ 0x20000, 0x20200 },	/* DSC */
	{ 0x21000, 0x211fc },	/* DSI TX */
	{ 0x21400, 0x2141c },
	{ 0x22000, 0x22554 },	/* MIPI TX */
	{ 0x23000, 0x231a8 },	/* MMSYS_CONFIG */
	{ 0x24000, 0x240b4 },	/* RDMA */
	{ 0x25000, 0x25030 },	/* MUTEX */
};

struct mt6382_bufs {
	u8 cmd[9];
	u8 rs_tx[2];
	u8 rs_rx[2];
	u8 ws[2];
	u8 ct[2];
	u8 data_tx[MT6382_SMALL_BUF];
	u8 data_rx[MT6382_SMALL_BUF];
};

struct mt6382 {
	struct device *dev;
	struct spi_device *spi;
	struct regmap *regmap;
	struct dentry *debugfs;

	/* serialises the SPI command sequences, the buffers and SPI settings */
	struct mutex lock;
	struct mt6382_bufs *bufs;	/* separate allocation, DMA-safe */

	struct mtk_chip_config chip_cfg;	/* handed to spi-mt65xx */
	void *orig_controller_data;
	bool mtk_controller;
	u32 tx_hz;
	u32 rx_hz;
	u32 cs_time_ns;

	/* the vendor init sequence may write exactly these, nothing else */
	bool init_running;
	/* DRM: one register write on behalf of the panel host (m->lock held) */
	bool scoped_write;

	bool connected;
	u32 chip_id;
	u8 last_status;
	unsigned int err_cfg;
	unsigned int err_status;
	unsigned int err_finish;
	unsigned int err_bus;
	unsigned int refused;

	u32 dbg_addr;

	/* DRM */
	struct drm_bridge bridge;
	struct mipi_dsi_host panel_host;	/* for the panel behind the TX */
	struct mipi_dsi_device *dsi;		/* us, on the SoC DSI host */
	struct mipi_dsi_device *panel_dsi;
	struct drm_panel *panel;
	struct device_node *soc_host_node;
	struct device_node *panel_node;
	struct delayed_work drm_work;
	unsigned int drm_tries;
	bool drm_host_registered;
	bool drm_bridge_added;
	bool drm_done;
	bool tx_running;			/* TX video running at DRM start */
	bool takeover_pending;			/* first enable keeps the bridge as it is */
	bool tx_writes;				/* TX command queue may be written */
	bool bridge_off;			/* reinit: bridge powered down, next enable inits */

	/* reinit: RESETB (GPIO32), requested at the first re-initialisation only */
	struct gpio_desc *reset_gpio;
	bool reset_gpio_tried;
	struct mt6382_trace *trace;		/* last re-initialisation, debugfs reinit_trace */
};

static struct mt6382 *mt6382_dev;		/* single instance, for the "drm" parameter */

static void mt6382_decode_status(u8 s, char *buf, size_t len)
{
	snprintf(buf, len, "0x%02x [%s%s%s%s%s%s%s%s]", s,
		 s & MT6382_SR_SLV_ON ? " SLV_ON" : "",
		 s & MT6382_SR_CFG_SUCCESS ? " CFG_SUCCESS" : "",
		 s & MT6382_SR_TXRX_FIFO_RDY ? " FIFO_RDY" : "",
		 s & MT6382_SR_RD_ERR ? " RD_ERR" : "",
		 s & MT6382_SR_WR_ERR ? " WR_ERR" : "",
		 s & MT6382_SR_RDWR_FINISH ? " RDWR_FINISH" : "",
		 s & MT6382_SR_TIMEOUT_ERR ? " TIMEOUT_ERR" : "",
		 s & MT6382_SR_CMD_ERR ? " CMD_ERR" : "");
}

/*
 * Commands that change the slave state are allowed with allow_writes, or
 * while the vendor init sequence runs (CT, SPISLV_CTRL and DRV_CFG0 only).
 */
static bool mt6382_may_write(struct mt6382 *m, const char *what, u32 addr,
			     bool init_ok)
{
	if (READ_ONCE(allow_writes) || (init_ok && m->init_running) || m->scoped_write)
		return true;

	m->refused++;
	dev_warn_ratelimited(m->dev, "refusing %s (addr 0x%05x): allow_writes=0\n",
			     what, addr);
	return false;
}

/* apply bit order, CS timing and sample settings; m->lock held */
static int mt6382_apply_spi_settings(struct mt6382 *m)
{
	struct spi_device *spi = m->spi;

	spi->cs_setup.value = m->cs_time_ns;
	spi->cs_setup.unit = SPI_DELAY_UNIT_NSECS;
	spi->cs_hold.value = m->cs_time_ns;
	spi->cs_hold.unit = SPI_DELAY_UNIT_NSECS;
	spi->cs_inactive.value = m->cs_time_ns;
	spi->cs_inactive.unit = SPI_DELAY_UNIT_NSECS;
	spi->bits_per_word = 8;

	return spi_setup(spi);
}

/*
 * SPI controller errors must not look like slave errors (-EIO), which are
 * retried; controller errors are not.
 */
static int mt6382_bus_err(int ret)
{
	return ret == -EIO ? -ECOMM : ret;
}

/* one [RS] transfer, appended to an existing message or sent alone */
static void mt6382_prep_rs(struct mt6382 *m, struct spi_transfer *x)
{
	struct mt6382_bufs *b = m->bufs;

	b->rs_tx[0] = MT6382_CMD_RS;
	b->rs_tx[1] = 0;
	b->rs_rx[0] = 0;
	b->rs_rx[1] = 0;
	x->tx_buf = b->rs_tx;
	x->rx_buf = b->rs_rx;
	x->len = sizeof(b->rs_tx);
	x->speed_hz = m->rx_hz;
}

/* RS alone; m->lock held */
static int mt6382_read_status(struct mt6382 *m, u8 *status)
{
	struct spi_transfer x = { };
	int ret;

	mt6382_prep_rs(m, &x);
	ret = spi_sync_transfer(m->spi, &x, 1);
	if (ret)
		return ret;

	*status = m->bufs->rs_rx[1];
	m->last_status = *status;
	return 0;
}

/* [WS, 0xff]: clear the slave error status; m->lock held */
static int mt6382_clear_status(struct mt6382 *m, u32 addr)
{
	struct mt6382_bufs *b = m->bufs;
	struct spi_transfer x = { };
	int ret;

	/* the vendor init sequence does not cover WS: allow_writes only */
	if (!mt6382_may_write(m, "WS (clear error status)", addr, false))
		return -EPERM;

	b->ws[0] = MT6382_CMD_WS;
	b->ws[1] = MT6382_WS_CLEAR_ALL;
	x.tx_buf = b->ws;
	x.len = sizeof(b->ws);
	x.speed_hz = m->tx_hz;
	ret = spi_sync_transfer(m->spi, &x, 1);
	if (ret)
		m->err_bus++;
	return ret;
}

/* [CT, 0x04]: select 4-byte single-wire transfers; m->lock held */
static int mt6382_config_type(struct mt6382 *m)
{
	struct mt6382_bufs *b = m->bufs;
	struct spi_transfer x = { };

	if (!mt6382_may_write(m, "CT (config type)", 0, true))
		return -EPERM;

	b->ct[0] = MT6382_CMD_CT;
	b->ct[1] = MT6382_CT_4BYTE_SINGLE;
	x.tx_buf = b->ct;
	x.len = sizeof(b->ct);
	x.speed_hz = m->tx_hz;
	return spi_sync_transfer(m->spi, &x, 1);
}

/* one access of at most MT6382_XFER_MAX bytes, no retries; m->lock held */
static int mt6382_xfer_once(struct mt6382 *m, u32 addr, void *val, u32 len,
			    bool read)
{
	struct mt6382_bufs *b = m->bufs;
	struct spi_transfer x[2] = { };
	u8 *tx = b->data_tx, *rx = b->data_rx;
	bool spislv_ctrl_write = !read && addr == MT6382_REG_SPISLV_CTRL;
	unsigned int polls = 0;
	u8 status;
	int ret;

	/* config phase: [CR|CW, addr, len - 1] [RS] */
	b->cmd[0] = read ? MT6382_CMD_CR : MT6382_CMD_CW;
	put_unaligned_le32(addr, &b->cmd[1]);
	put_unaligned_le32(len - 1, &b->cmd[5]);
	x[0].tx_buf = b->cmd;
	x[0].len = sizeof(b->cmd);
	x[0].speed_hz = m->tx_hz;
	x[0].cs_change = 1;
	mt6382_prep_rs(m, &x[1]);

	ret = spi_sync_transfer(m->spi, x, ARRAY_SIZE(x));
	if (ret) {
		m->err_bus++;
		return mt6382_bus_err(ret);
	}

	status = b->rs_rx[1];
	m->last_status = status;
	/* vendor: the status is not valid while the early-transfer bit changes */
	if (spislv_ctrl_write)
		status = MT6382_SR_CFG_READY;
	if ((status & MT6382_SR_CFG_READY) != MT6382_SR_CFG_READY) {
		m->err_cfg++;
		dev_dbg(m->dev, "config %s 0x%05x: status 0x%02x\n",
			read ? "read" : "write", addr, status);
		return -EIO;
	}

	/* data phase: [RD|WD, data] [RS] */
	if (len + 1 > MT6382_SMALL_BUF) {
		tx = kzalloc(2 * (len + 1), GFP_KERNEL);
		if (!tx)
			return -ENOMEM;
		rx = tx + len + 1;
	} else {
		memset(tx, 0, MT6382_SMALL_BUF);
		memset(rx, 0, MT6382_SMALL_BUF);
	}

	memset(x, 0, sizeof(x));
	if (read) {
		tx[0] = MT6382_CMD_RD;
		x[0].rx_buf = rx;
		x[0].speed_hz = m->rx_hz;
	} else {
		tx[0] = MT6382_CMD_WD;
		memcpy(tx + 1, val, len);
		x[0].speed_hz = m->tx_hz;
	}
	x[0].tx_buf = tx;
	x[0].len = len + 1;
	x[0].cs_change = 1;
	mt6382_prep_rs(m, &x[1]);

	ret = spi_sync_transfer(m->spi, x, ARRAY_SIZE(x));
	if (ret) {
		m->err_bus++;
		ret = mt6382_bus_err(ret);
		goto out;
	}

	status = b->rs_rx[1];
	m->last_status = status;
	if (spislv_ctrl_write)
		status = MT6382_SR_CFG_READY | MT6382_SR_RDWR_FINISH;
	if (status & MT6382_SR_ERRORS) {
		m->err_status++;
		dev_dbg(m->dev, "%s 0x%05x: error status 0x%02x\n",
			read ? "read" : "write", addr, status);
		mt6382_clear_status(m, addr);
		ret = -EIO;
		goto out;
	}

	while (!(status & MT6382_SR_RDWR_FINISH)) {
		if (polls++ >= MT6382_FINISH_POLLS) {
			m->err_finish++;
			ret = -EBUSY;
			goto out;
		}
		fsleep(1000);
		ret = mt6382_read_status(m, &status);
		if (ret) {
			m->err_bus++;
			ret = mt6382_bus_err(ret);
			goto out;
		}
	}

	if (read)
		memcpy(val, rx + 1, len);
out:
	if (tx != b->data_tx)
		kfree(tx);
	return ret;
}

/* m->lock held */
static int mt6382_xfer_locked(struct mt6382 *m, u32 addr, void *val,
			      size_t len, bool read)
{
	u8 *p = val;
	unsigned int try;
	int ret = 0;

	if (!read) {
		bool init_reg = addr == MT6382_REG_SPISLV_CTRL ||
				addr == MT6382_REG_IOCFG_DRV_CFG0;

		if (!mt6382_may_write(m, "register write", addr, init_reg && len == 4))
			return -EPERM;
	}

	while (len) {
		u32 chunk = min_t(size_t, len, MT6382_XFER_MAX);

		for (try = 0; ; try++) {
			ret = mt6382_xfer_once(m, addr, p, chunk, read);
			/*
			 * Like the vendor driver, repeat accesses the slave
			 * rejected (-EIO) or did not finish (-EBUSY). Do not
			 * repeat SPI controller errors (e.g. the transfer
			 * timeout of the SPI core), so a dead bus costs one
			 * timeout per access and never stalls for long.
			 */
			if (!ret)
				break;
			if (ret != -EIO && ret != -EBUSY)
				return ret;
			if (try >= MT6382_RETRIES) {
				dev_warn_ratelimited(m->dev,
					"%s 0x%05x failed after %u tries: %d (status 0x%02x)\n",
					read ? "read" : "write", addr, try + 1,
					ret, m->last_status);
				return ret;
			}
		}
		addr += chunk;
		p += chunk;
		len -= chunk;
	}
	return 0;
}

static int mt6382_xfer(struct mt6382 *m, u32 addr, void *val, size_t len,
		       bool read)
{
	int ret;

	if (!len)
		return 0;

	mutex_lock(&m->lock);
	ret = mt6382_xfer_locked(m, addr, val, len, read);
	mutex_unlock(&m->lock);
	return ret;
}

static int mt6382_read32_locked(struct mt6382 *m, u32 addr, u32 *val)
{
	u8 buf[4];
	int ret;

	ret = mt6382_xfer_locked(m, addr, buf, sizeof(buf), true);
	if (!ret)
		*val = get_unaligned_le32(buf);
	return ret;
}

static int mt6382_write32_locked(struct mt6382 *m, u32 addr, u32 val)
{
	u8 buf[4];

	put_unaligned_le32(val, buf);
	return mt6382_xfer_locked(m, addr, buf, sizeof(buf), false);
}

static int mt6382_reg_read(void *context, unsigned int reg, unsigned int *val)
{
	struct mt6382 *m = context;
	u8 buf[4];
	int ret;

	ret = mt6382_xfer(m, reg, buf, sizeof(buf), true);
	if (!ret)
		*val = get_unaligned_le32(buf);
	return ret;
}

static int mt6382_reg_write(void *context, unsigned int reg, unsigned int val)
{
	struct mt6382 *m = context;
	u8 buf[4];

	put_unaligned_le32(val, buf);
	return mt6382_xfer(m, reg, buf, sizeof(buf), false);
}

static bool mt6382_readable_reg(struct device *dev, unsigned int reg)
{
	unsigned int i;

	for (i = 0; i < ARRAY_SIZE(mt6382_known_regs); i++)
		if (mt6382_known_regs[i] == reg)
			return true;
	return false;
}

static bool mt6382_volatile_reg(struct device *dev, unsigned int reg)
{
	return true;
}

static const struct regmap_config mt6382_regmap_config = {
	.name = "mt6382",
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4,
	.max_register = MT6382_MAX_REGISTER,
	.reg_read = mt6382_reg_read,
	.reg_write = mt6382_reg_write,
	.readable_reg = mt6382_readable_reg,
	.volatile_reg = mt6382_volatile_reg,
	.cache_type = REGCACHE_NONE,
};

/* log the raw slave status (RS only, changes nothing); m->lock held */
static void mt6382_log_status_locked(struct mt6382 *m, const char *step)
{
	char sbuf[96];
	u8 status;
	int ret;

	ret = mt6382_read_status(m, &status);
	if (ret) {
		dev_info(m->dev, "%s: RS failed: %d\n", step, ret);
		return;
	}
	mt6382_decode_status(status, sbuf, sizeof(sbuf));
	dev_info(m->dev, "%s: status %s\n", step, sbuf);
}

/*
 * Exactly what the vendor spislv_init() does at every boot: CT {0x10, 0x04}, then SPISLV_CTRL =
 * 0x40 | (low_speed_early_trans << 16) with the status ignored, then DRV_CFG0[23:21] =
 * slave_drive_strength by read-modify-write. The vendor DT of the Moto G60s has
 * low-speed-early-trans = 0, slave-drive-strength = 3. m->lock held.
 */
static int mt6382_vendor_init_locked(struct mt6382 *m)
{
	u32 val, nval;
	int ret;

	m->init_running = true;

	ret = mt6382_config_type(m);
	dev_info(m->dev, "init 1/3: CT {0x10, 0x04}: %d\n", ret);
	mt6382_log_status_locked(m, "init 1/3 after CT");
	if (ret)
		goto out;

	ret = mt6382_write32_locked(m, MT6382_REG_SPISLV_CTRL, MT6382_SPISLV_CTRL_INIT);
	dev_info(m->dev, "init 2/3: SPISLV_CTRL 0x%05x = 0x%08x: %d (raw status 0x%02x, ignored as in the vendor driver)\n",
		 MT6382_REG_SPISLV_CTRL, MT6382_SPISLV_CTRL_INIT, ret, m->last_status);
	mt6382_log_status_locked(m, "init 2/3 after SPISLV_CTRL");
	if (ret)
		goto out;

	ret = mt6382_read32_locked(m, MT6382_REG_IOCFG_DRV_CFG0, &val);
	dev_info(m->dev, "init 3/3: DRV_CFG0 0x%05x read: %d, val 0x%08x (status 0x%02x)\n",
		 MT6382_REG_IOCFG_DRV_CFG0, ret, ret ? 0 : val, m->last_status);
	if (ret)
		goto out;

	nval = (val & ~MT6382_DRV_CFG0_SPIS_SLVO) |
	       FIELD_PREP(MT6382_DRV_CFG0_SPIS_SLVO, MT6382_SLAVE_DRIVE_STRENGTH);
	ret = mt6382_write32_locked(m, MT6382_REG_IOCFG_DRV_CFG0, nval);
	dev_info(m->dev, "init 3/3: DRV_CFG0 0x%05x = 0x%08x: %d (status 0x%02x)\n",
		 MT6382_REG_IOCFG_DRV_CFG0, nval, ret, m->last_status);
	mt6382_log_status_locked(m, "init 3/3 after DRV_CFG0");
out:
	m->init_running = false;
	return ret;
}

static int mt6382_read_chip_id_locked(struct mt6382 *m)
{
	u32 id;
	int ret;

	ret = mt6382_read32_locked(m, MT6382_REG_CHIP_ID, &id);
	if (ret)
		return ret;
	m->chip_id = id;
	if ((id & MT6382_CHIP_ID_MASK) != MT6382_CHIP_ID_VAL)
		return -ENODEV;
	return 0;
}

/* chip ID check, optionally with the vendor init on a config error */
static int mt6382_connect(struct mt6382 *m, bool may_init)
{
	unsigned int cfg_before;
	int ret;

	mutex_lock(&m->lock);
	mt6382_log_status_locked(m, "connect: before first access");

	cfg_before = m->err_cfg;
	ret = mt6382_read_chip_id_locked(m);
	dev_info(m->dev, "connect: chip id read (read-only): %d, id 0x%08x, status 0x%02x, cfg/status/finish errors %u/%u/%u\n",
		 ret, m->chip_id, m->last_status, m->err_cfg, m->err_status, m->err_finish);

	if (ret == -EIO && m->err_cfg > cfg_before && may_init) {
		dev_info(m->dev, "connect: config error, running the vendor spislv_init sequence\n");
		ret = mt6382_vendor_init_locked(m);
		if (!ret) {
			ret = mt6382_read_chip_id_locked(m);
			dev_info(m->dev, "connect: chip id read after init: %d, id 0x%08x, status 0x%02x\n",
				 ret, m->chip_id, m->last_status);
		}
	}

	m->connected = !ret;
	mutex_unlock(&m->lock);
	return ret;
}

/* ------------------------------------------------------------------ debugfs */

/* same format as the register dump of the vendor driver */
static int mt6382_regs_show(struct seq_file *s, void *data)
{
	struct mt6382 *m = s->private;
	unsigned int i, val;
	int ret;

	for (i = 0; i < ARRAY_SIZE(mt6382_known_regs); i++) {
		u32 reg = mt6382_known_regs[i];

		ret = regmap_read(m->regmap, reg, &val);
		if (ret)
			seq_printf(s, "addr:0x%08x, err:%d\n", reg, ret);
		else
			seq_printf(s, "addr:0x%08x, val:0x%08x\n", reg, val);
	}
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(mt6382_regs);

static int mt6382_dump_show(struct seq_file *s, void *data)
{
	struct mt6382 *m = s->private;
	unsigned int i, fails = 0;
	u32 addr, val;
	int ret;

	for (i = 0; i < ARRAY_SIZE(mt6382_dump_ranges); i++) {
		for (addr = mt6382_dump_ranges[i].start;
		     addr <= mt6382_dump_ranges[i].end; addr += 4) {
			mutex_lock(&m->lock);
			ret = mt6382_read32_locked(m, addr, &val);
			mutex_unlock(&m->lock);
			if (ret) {
				seq_printf(s, "addr:0x%08x, err:%d\n", addr, ret);
				if (++fails >= MT6382_PROBE_MAX_FAILS)
					return 0;
				continue;
			}
			fails = 0;
			seq_printf(s, "addr:0x%08x, val:0x%08x\n", addr, val);
		}
	}
	return 0;
}
/* one pass over the hardware: a buffer large enough that seq_file never retries */
static int mt6382_dump_open(struct inode *inode, struct file *file)
{
	return single_open_size(file, mt6382_dump_show, inode->i_private, SZ_128K);
}

static const struct file_operations mt6382_dump_fops = {
	.owner = THIS_MODULE,
	.open = mt6382_dump_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int mt6382_status_show(struct seq_file *s, void *data)
{
	struct mt6382 *m = s->private;
	struct spi_device *spi = m->spi;
	char sbuf[96];

	mutex_lock(&m->lock);
	mt6382_decode_status(m->last_status, sbuf, sizeof(sbuf));
	seq_printf(s, "connected: %d\n", m->connected);
	seq_printf(s, "chip_id: 0x%08x\n", m->chip_id);
	seq_printf(s, "spi_tx_hz: %u\nspi_rx_hz: %u\n", m->tx_hz, m->rx_hz);
	seq_printf(s, "spi_mode: %u\nlsb_first: %d\n", (unsigned int)(spi->mode & SPI_MODE_X_MASK),
		   !!(spi->mode & SPI_LSB_FIRST));
	seq_printf(s, "cs_time_ns: %u\n", m->cs_time_ns);
	seq_printf(s, "mtk_controller: %d\ntick_delay: %u\nsample_sel: %u\n",
		   m->mtk_controller, m->chip_cfg.tick_delay, m->chip_cfg.sample_sel);
	seq_printf(s, "allow_writes: %d\ninit_on_cfg_error: %d\n",
		   READ_ONCE(allow_writes), init_on_cfg_error);
	seq_printf(s, "last_status: %s\n", sbuf);
	seq_printf(s, "err_config: %u\nerr_status: %u\nerr_finish_timeout: %u\n",
		   m->err_cfg, m->err_status, m->err_finish);
	seq_printf(s, "err_bus: %u\nrefused_writes: %u\n", m->err_bus, m->refused);
	mutex_unlock(&m->lock);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(mt6382_status);

/* rs: one RS command, nothing else */
static int mt6382_rs_show(struct seq_file *s, void *data)
{
	struct mt6382 *m = s->private;
	char sbuf[96];
	u8 status;
	int ret;

	mutex_lock(&m->lock);
	ret = mt6382_read_status(m, &status);
	mutex_unlock(&m->lock);
	if (ret) {
		seq_printf(s, "error %d\n", ret);
		return 0;
	}
	mt6382_decode_status(status, sbuf, sizeof(sbuf));
	seq_printf(s, "%s\n", sbuf);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(mt6382_rs);

/* reg: read the register at reg_addr (any 4-byte aligned address) */
static int mt6382_reg_show(struct seq_file *s, void *data)
{
	struct mt6382 *m = s->private;
	u32 addr = READ_ONCE(m->dbg_addr);
	u32 val = 0;
	int ret;

	if (addr & 3 || addr > MT6382_MAX_REGISTER) {
		seq_printf(s, "addr:0x%08x, err:%d (unaligned or out of range)\n", addr, -EINVAL);
		return 0;
	}

	mutex_lock(&m->lock);
	ret = mt6382_read32_locked(m, addr, &val);
	mutex_unlock(&m->lock);
	if (ret)
		seq_printf(s, "addr:0x%08x, err:%d, status:0x%02x\n", addr, ret, m->last_status);
	else
		seq_printf(s, "addr:0x%08x, val:0x%08x, status:0x%02x\n",
			   addr, val, m->last_status);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(mt6382_reg);

static int mt6382_set_param(struct mt6382 *m, u32 *field, u64 val, u64 min, u64 max)
{
	int ret;

	if (val < min || val > max)
		return -EINVAL;

	mutex_lock(&m->lock);
	*field = val;
	ret = mt6382_apply_spi_settings(m);
	mutex_unlock(&m->lock);
	return ret;
}

static int mt6382_get_tx_hz(void *data, u64 *val)
{
	*val = ((struct mt6382 *)data)->tx_hz;
	return 0;
}

static int mt6382_set_tx_hz(void *data, u64 val)
{
	struct mt6382 *m = data;

	return mt6382_set_param(m, &m->tx_hz, val, MT6382_SPI_MIN_HZ,
				min_t(u32, MT6382_SPI_TX_MAX_HZ, m->spi->max_speed_hz));
}
DEFINE_DEBUGFS_ATTRIBUTE(mt6382_tx_hz_fops, mt6382_get_tx_hz, mt6382_set_tx_hz, "%llu\n");

static int mt6382_get_rx_hz(void *data, u64 *val)
{
	*val = ((struct mt6382 *)data)->rx_hz;
	return 0;
}

static int mt6382_set_rx_hz(void *data, u64 val)
{
	struct mt6382 *m = data;

	return mt6382_set_param(m, &m->rx_hz, val, MT6382_SPI_MIN_HZ,
				min_t(u32, MT6382_SPI_RX_MAX_HZ, m->spi->max_speed_hz));
}
DEFINE_DEBUGFS_ATTRIBUTE(mt6382_rx_hz_fops, mt6382_get_rx_hz, mt6382_set_rx_hz, "%llu\n");

static int mt6382_get_tick(void *data, u64 *val)
{
	*val = ((struct mt6382 *)data)->chip_cfg.tick_delay;
	return 0;
}

static int mt6382_set_tick(void *data, u64 val)
{
	struct mt6382 *m = data;

	return mt6382_set_param(m, &m->chip_cfg.tick_delay, val, 0, 7);
}
DEFINE_DEBUGFS_ATTRIBUTE(mt6382_tick_fops, mt6382_get_tick, mt6382_set_tick, "%llu\n");

static int mt6382_get_sample(void *data, u64 *val)
{
	*val = ((struct mt6382 *)data)->chip_cfg.sample_sel;
	return 0;
}

static int mt6382_set_sample(void *data, u64 val)
{
	struct mt6382 *m = data;

	return mt6382_set_param(m, &m->chip_cfg.sample_sel, val, 0, 1);
}
DEFINE_DEBUGFS_ATTRIBUTE(mt6382_sample_fops, mt6382_get_sample, mt6382_set_sample, "%llu\n");

static int mt6382_get_cs_time(void *data, u64 *val)
{
	*val = ((struct mt6382 *)data)->cs_time_ns;
	return 0;
}

static int mt6382_set_cs_time(void *data, u64 val)
{
	struct mt6382 *m = data;

	return mt6382_set_param(m, &m->cs_time_ns, val, 0, 10000);
}
DEFINE_DEBUGFS_ATTRIBUTE(mt6382_cs_time_fops, mt6382_get_cs_time, mt6382_set_cs_time, "%llu\n");

static int mt6382_get_mode(void *data, u64 *val)
{
	*val = ((struct mt6382 *)data)->spi->mode & SPI_MODE_X_MASK;
	return 0;
}

static int mt6382_set_mode(void *data, u64 val)
{
	struct mt6382 *m = data;
	int ret;

	if (val > 3)
		return -EINVAL;
	mutex_lock(&m->lock);
	m->spi->mode = (m->spi->mode & ~SPI_MODE_X_MASK) | val;
	ret = mt6382_apply_spi_settings(m);
	mutex_unlock(&m->lock);
	return ret;
}
DEFINE_DEBUGFS_ATTRIBUTE(mt6382_mode_fops, mt6382_get_mode, mt6382_set_mode, "%llu\n");

static int mt6382_get_lsb(void *data, u64 *val)
{
	*val = !!(((struct mt6382 *)data)->spi->mode & SPI_LSB_FIRST);
	return 0;
}

static int mt6382_set_lsb(void *data, u64 val)
{
	struct mt6382 *m = data;
	int ret;

	if (val > 1)
		return -EINVAL;
	mutex_lock(&m->lock);
	if (val)
		m->spi->mode |= SPI_LSB_FIRST;
	else
		m->spi->mode &= ~SPI_LSB_FIRST;
	ret = mt6382_apply_spi_settings(m);
	mutex_unlock(&m->lock);
	return ret;
}
DEFINE_DEBUGFS_ATTRIBUTE(mt6382_lsb_fops, mt6382_get_lsb, mt6382_set_lsb, "%llu\n");

/* retry: chip ID check again, read-only (writing any value triggers it) */
static int mt6382_set_retry(void *data, u64 val)
{
	struct mt6382 *m = data;

	mt6382_connect(m, false);
	return 0;
}
DEFINE_DEBUGFS_ATTRIBUTE(mt6382_retry_fops, NULL, mt6382_set_retry, "%llu\n");

/* ct / ws / init: state-changing actions, allow_writes only */
static int mt6382_set_ct(void *data, u64 val)
{
	struct mt6382 *m = data;
	int ret;

	if (!READ_ONCE(allow_writes))
		return -EPERM;
	mutex_lock(&m->lock);
	ret = mt6382_config_type(m);
	dev_info(m->dev, "debugfs: CT: %d\n", ret);
	mt6382_log_status_locked(m, "debugfs: after CT");
	mutex_unlock(&m->lock);
	return ret;
}
DEFINE_DEBUGFS_ATTRIBUTE(mt6382_ct_fops, NULL, mt6382_set_ct, "%llu\n");

static int mt6382_set_ws(void *data, u64 val)
{
	struct mt6382 *m = data;
	int ret;

	if (!READ_ONCE(allow_writes))
		return -EPERM;
	mutex_lock(&m->lock);
	ret = mt6382_clear_status(m, 0);
	dev_info(m->dev, "debugfs: WS 0xff: %d\n", ret);
	mt6382_log_status_locked(m, "debugfs: after WS");
	mutex_unlock(&m->lock);
	return ret;
}
DEFINE_DEBUGFS_ATTRIBUTE(mt6382_ws_fops, NULL, mt6382_set_ws, "%llu\n");

static int mt6382_set_init(void *data, u64 val)
{
	struct mt6382 *m = data;
	int ret;

	if (!READ_ONCE(allow_writes))
		return -EPERM;
	mutex_lock(&m->lock);
	ret = mt6382_vendor_init_locked(m);
	mutex_unlock(&m->lock);
	return ret;
}
DEFINE_DEBUGFS_ATTRIBUTE(mt6382_init_fops, NULL, mt6382_set_init, "%llu\n");

static void mt6382_debugfs_init(struct mt6382 *m)
{
	struct dentry *d = debugfs_create_dir("mt6382", NULL);

	m->debugfs = d;
	debugfs_create_file("status", 0444, d, m, &mt6382_status_fops);
	debugfs_create_file("regs", 0444, d, m, &mt6382_regs_fops);
	debugfs_create_file("dump", 0400, d, m, &mt6382_dump_fops);
	debugfs_create_file("rs", 0444, d, m, &mt6382_rs_fops);
	debugfs_create_x32("reg_addr", 0644, d, &m->dbg_addr);
	debugfs_create_file("reg", 0444, d, m, &mt6382_reg_fops);
	debugfs_create_file_unsafe("spi_tx_hz", 0644, d, m, &mt6382_tx_hz_fops);
	debugfs_create_file_unsafe("spi_rx_hz", 0644, d, m, &mt6382_rx_hz_fops);
	debugfs_create_file_unsafe("spi_mode", 0644, d, m, &mt6382_mode_fops);
	debugfs_create_file_unsafe("lsb_first", 0644, d, m, &mt6382_lsb_fops);
	debugfs_create_file_unsafe("cs_time_ns", 0644, d, m, &mt6382_cs_time_fops);
	debugfs_create_file_unsafe("retry", 0200, d, m, &mt6382_retry_fops);
	if (m->mtk_controller) {
		debugfs_create_file_unsafe("tick_delay", 0644, d, m, &mt6382_tick_fops);
		debugfs_create_file_unsafe("sample_sel", 0644, d, m, &mt6382_sample_fops);
	}
	debugfs_create_file_unsafe("ct", 0200, d, m, &mt6382_ct_fops);
	debugfs_create_file_unsafe("ws", 0200, d, m, &mt6382_ws_fops);
	debugfs_create_file_unsafe("init", 0200, d, m, &mt6382_init_fops);
}

/* ---------------------------------------------------------------------- DRM */

#define MT6382_TX_INTSTA		0x2100c
#define MT6382_TX_CMD_DONE		BIT(1)
#define MT6382_TX_BUSY			BIT(31)
#define MT6382_TX_CMDQ_CON		0x21060
#define MT6382_TX_CMDQ_SIZE_SEL		BIT(15)
#define MT6382_TX_CMDQ			0x21d00
#define MT6382_TX_CMDQ_WORDS		32
#define MT6382_RX_IPI_MODE		BIT(0)

/*
 * Host-side timing of the lisbon link. lk drives the SoC DSI at 2.3 Gbit/s per lane (PCW
 * 0x58762762), RGB888 on 4 lanes, i.e. a pixel clock of 2300 * 4 / 24 MHz; the vertical timing
 * equals the panel's, the line time of 3288 ns gives htotal 1260. The DSI driver takes the exact
 * porch and D-PHY registers from the running link ("mediatek,inherit-boot-state"), these numbers
 * only fix size, rate and refresh.
 */
#define MT6382_HOST_CLOCK_KHZ		383333
#define MT6382_HOST_HSYNC		2
#define MT6382_HOST_HBP			30
#define MT6382_HOST_HFP			148

/*
 * Second operating mode "reinit" (allow_writes and reinit both on): display off/on (DPMS) switches
 * bridge and panel off completely and brings them up again with the vendor sequence: host LP-11
 * (CRTC enable) -> bridge power, TX/DSC/RDMA/MUTEX/MIPI-TX, RX PHY and MAC -> RX stop state ->
 * panel power and init through the TX command queue -> TX video -> host video with deskew (mtk_dsi
 * enable). The first enable after the takeover always keeps lk's state. XO_EXTBUF3 stays on as lk
 * left it (nothing in this kernel switches it off).
 *
 * Like the vendor bdg_common_init, the bring-up starts with a reset pulse on RESETB (GPIO32: high,
 * 10 us low, high), then the SPI slave set-up (spislv_init: CT, SPISLV_CTRL, DRV_CFG0) and the NFC
 * clock buffer (0xa0 = 0x02, off as lk leaves it). The vendor power-off leaves the PHY LDOs in
 * low-power mode (set_LDO_off: PHYLDO1_LP_EN = 1, PHYLDO2_EN = 0) and set_LDO_on polls LDO_STATUS &
 * 0x300 before it undoes that; only the reset brings LDO_CTRL1 back to its default. Without the
 * pulse the poll times out (0x000ac reads 0xf24a, bit 8 missing).
 */
static bool reinit;
module_param(reinit, bool, 0644);
MODULE_PARM_DESC(reinit,
		 "Display off/on re-initialises bridge and panel completely (needs allow_writes; default: backlight only)");

static bool reinit_reset = true;
module_param(reinit_reset, bool, 0644);
MODULE_PARM_DESC(reinit_reset,
		 "Re-initialisation starts with a RESETB pulse and the SPI slave set-up, as the vendor bdg_common_init (default: on)");

static bool reinit_ldo_kick = true;
module_param(reinit_ldo_kick, bool, 0644);
MODULE_PARM_DESC(reinit_ldo_kick,
		 "Diagnostic: if LDO_STATUS stays without 0x300, clear PHYLDO1_LP_EN and poll again (the vendor driver clears it right after the poll; default: on)");

static uint reinit_retries = 1;
module_param(reinit_retries, uint, 0644);
MODULE_PARM_DESC(reinit_retries,
		 "Further complete attempts (power-off, reset, bring-up) if the bridge part fails before the panel init (default: 1)");

enum mt6382_seq_type {
	MT6382_SEQ_W,
	MT6382_SEQ_M,
	MT6382_SEQ_DELAY,
	MT6382_SEQ_POLL,
	MT6382_SEQ_EFUSE,
};

struct mt6382_seq_op {
	u8 op;
	u8 stable;		/* POLL: matches in a row */
	u16 delay_us;		/* DELAY, POLL interval */
	u16 tries;		/* POLL */
	u32 addr;
	u32 mask;
	u32 val;
};

#define SEQ_W(a, v)	{ .op = MT6382_SEQ_W, .addr = (a), .mask = 0xffffffff, .val = (v) }
#define SEQ_M(a, m, v)	{ .op = MT6382_SEQ_M, .addr = (a), .mask = (m), .val = (v) }
#define SEQ_DELAY(us)	{ .op = MT6382_SEQ_DELAY, .delay_us = (us) }
#define SEQ_POLL(a, m, v, t, d, s) \
	{ .op = MT6382_SEQ_POLL, .addr = (a), .mask = (m), .val = (v), \
	  .tries = (t), .delay_us = (d), .stable = (s) }
#define SEQ_EFUSE()	{ .op = MT6382_SEQ_EFUSE }

#include "mediatek-mt6382-seq.h"

/* eFuse of the LDO trim (set_LDO_on) */
#define MT6382_EFUSE_STATUS		0x09128
#define MT6382_EFUSE_TRIM1		0x098e0
#define MT6382_EFUSE_TRIM2		0x098e4
#define MT6382_LDO_CTRL0		0x000a4
#define MT6382_LDO_TRIM_BY_EFUSE	BIT(30)
#define MT6382_LDO_CTRL1		0x000a8
#define MT6382_PHYLDO1_LP_EN		BIT(29)
#define MT6382_LDO_READY		0x300	/* LDO_STATUS bits set_LDO_on waits for */
#define MT6382_REG_CKBUF_CTRL		0x000a0
#define MT6382_CKBUF_NFC_OFF		0x02	/* bdg_clk_buf_nfc(0), lk value */
#define MT6382_TX_INTSTA_ULPS_POLL	0x2100c	/* power-off: timeout only warns, like the vendor */

/*
 * Diagnostics of the last re-initialisation (debugfs mt6382/reinit_trace):
 * every sequence step with the value read and written, and snapshots of the
 * SYSREG block (0x00-0xac: power, reset, clock buffer, LDOs) plus RESETB.
 */
#define MT6382_TRACE_OPS		1024
#define MT6382_TRACE_SNAPS		10
#define MT6382_SNAP_REGS		(0xac / 4 + 1)

struct mt6382_trace_op {
	const char *seq;
	u16 idx;
	u8 op;
	s16 ret;
	u32 addr;
	u32 rd;		/* value read (M, POLL: last value) */
	u32 wr;		/* value written (W, M), tries used (POLL), us (DELAY) */
};

struct mt6382_trace_snap {
	const char *label;
	int resetb;	/* raw level of RESETB, -1: not requested */
	int ret;
	u32 val[MT6382_SNAP_REGS];
};

struct mt6382_trace {
	unsigned int run;
	unsigned int n_ops, lost_ops, n_snaps;
	int result;
	struct mt6382_trace_op ops[MT6382_TRACE_OPS];
	struct mt6382_trace_snap snaps[MT6382_TRACE_SNAPS];
};

static void mt6382_trace_op(struct mt6382 *m, const char *seq, unsigned int idx,
			    const struct mt6382_seq_op *o, u32 rd, u32 wr, int ret)
{
	struct mt6382_trace *t = m->trace;
	struct mt6382_trace_op *e;

	if (!t)
		return;
	if (t->n_ops >= MT6382_TRACE_OPS) {
		t->lost_ops++;
		return;
	}
	e = &t->ops[t->n_ops++];
	e->seq = seq;
	e->idx = idx;
	e->op = o->op;
	e->ret = ret;
	e->addr = o->addr;
	e->rd = rd;
	e->wr = wr;
}

/* SYSREG snapshot into the trace and the key registers into the log */
static void mt6382_snapshot(struct mt6382 *m, const char *label)
{
	struct mt6382_trace *t = m->trace;
	struct mt6382_trace_snap tmp, *sn = &tmp;
	unsigned int i;
	int ret = 0;

	if (t && t->n_snaps < MT6382_TRACE_SNAPS)
		sn = &t->snaps[t->n_snaps++];
	memset(sn, 0, sizeof(*sn));
	sn->label = label;
	sn->resetb = m->reset_gpio ? gpiod_get_raw_value_cansleep(m->reset_gpio) : -1;

	mutex_lock(&m->lock);
	for (i = 0; i < MT6382_SNAP_REGS && !ret; i++)
		ret = mt6382_read32_locked(m, 4 * i, &sn->val[i]);
	mutex_unlock(&m->lock);
	sn->ret = ret;

	dev_info(m->dev,
		 "reinit [%s]: %s PWR 0x%08x RST 0x%08x CKBUF 0x%02x LDO0 0x%08x LDO1 0x%08x LDO_STATUS 0x%04x RESETB %d\n",
		 label, ret ? "READ ERROR," : "", sn->val[0x08 / 4], sn->val[0x10 / 4],
		 sn->val[0xa0 / 4], sn->val[0xa4 / 4], sn->val[0xa8 / 4], sn->val[0xac / 4],
		 sn->resetb);
}

static int mt6382_trace_show(struct seq_file *sf, void *data)
{
	static const char * const opn[] = { "W", "M", "DELAY", "POLL", "EFUSE" };
	struct mt6382 *m = sf->private;
	struct mt6382_trace *t = m->trace;
	unsigned int i, j;

	if (!t || !t->run) {
		seq_puts(sf, "no re-initialisation yet\n");
		return 0;
	}
	seq_printf(sf, "run %u result %d, %u steps (%u not recorded), %u snapshots\n",
		   t->run, t->result, t->n_ops, t->lost_ops, t->n_snaps);
	for (i = 0; i < t->n_snaps; i++) {
		const struct mt6382_trace_snap *sn = &t->snaps[i];

		seq_printf(sf, "snapshot %u [%s] RESETB %d read %d\n", i, sn->label, sn->resetb, sn->ret);
		for (j = 0; j < MT6382_SNAP_REGS; j++)
			seq_printf(sf, "%s0x%03x:%08x%s", j % 4 ? " " : "  ", 4 * j, sn->val[j],
				   j % 4 == 3 || j == MT6382_SNAP_REGS - 1 ? "\n" : "");
	}
	seq_puts(sf, "seq idx op addr read written|tries|us ret\n");
	for (i = 0; i < t->n_ops; i++) {
		const struct mt6382_trace_op *e = &t->ops[i];

		seq_printf(sf, "%s %u %s 0x%05x 0x%08x 0x%08x %d\n", e->seq, e->idx,
			   e->op < ARRAY_SIZE(opn) ? opn[e->op] : "?", e->addr, e->rd, e->wr, e->ret);
	}
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(mt6382_trace);

/* m->lock held */
static int mt6382_seq_efuse_locked(struct mt6382 *m)
{
	u32 st, t1, t2, v;
	int ret;

	ret = mt6382_read32_locked(m, MT6382_EFUSE_STATUS, &st);
	if (ret || !(st & 0x7fffff))
		return ret;
	ret = mt6382_read32_locked(m, MT6382_EFUSE_TRIM1, &t1);
	if (!ret)
		ret = mt6382_read32_locked(m, MT6382_EFUSE_TRIM2, &t2);
	if (ret)
		return ret;
	/* the vendor driver tests TRIM1 and TRIM2 (TRIM3 is read but its test repeats TRIM2) */
	if (!(t1 & 0x3f) && !(t2 & 0xf))
		return 0;
	ret = mt6382_read32_locked(m, MT6382_LDO_CTRL0, &v);
	if (!ret)
		ret = mt6382_write32_locked(m, MT6382_LDO_CTRL0, v | MT6382_LDO_TRIM_BY_EFUSE);
	dev_info(m->dev, "reinit: LDO trim by eFuse (0x%06x, %02x %x)\n", st & 0x7fffff,
		 t1 & 0x3f, t2 & 0xf);
	return ret;
}

/*
 * Diagnostic for a missing LDO ready: clear PHYLDO1_LP_EN, which the vendor power-off set and
 * set_LDO_on clears only after the poll, and poll again. Without a reset this is the only bit of
 * LDO_CTRL1 that differs from lk's running state besides PHYLDO2_EN. m->lock held.
 */
static int mt6382_ldo_kick_locked(struct mt6382 *m, const struct mt6382_seq_op *o, u32 *v)
{
	u32 c0 = 0, c1 = 0;
	unsigned int t;
	int ret;

	ret = mt6382_read32_locked(m, MT6382_LDO_CTRL0, &c0);
	if (!ret)
		ret = mt6382_read32_locked(m, MT6382_LDO_CTRL1, &c1);
	if (ret)
		return ret;
	dev_warn(m->dev,
		 "reinit: LDO_STATUS 0x%04x without 0x300 (LDO_CTRL0 0x%08x LDO_CTRL1 0x%08x); diagnostic: PHYLDO1_LP_EN = 0, poll again\n",
		 *v, c0, c1);
	ret = mt6382_write32_locked(m, MT6382_LDO_CTRL1, c1 & ~MT6382_PHYLDO1_LP_EN);
	for (t = 0; t < o->tries && !ret; t++) {
		ret = mt6382_read32_locked(m, o->addr, v);
		if (ret || (*v & o->mask) == o->val)
			break;
		fsleep(o->delay_us);
	}
	if (ret)
		return ret;
	if ((*v & o->mask) != o->val) {
		dev_warn(m->dev, "reinit: LDO_STATUS 0x%04x after PHYLDO1_LP_EN = 0: still not ready\n", *v);
		return -ETIMEDOUT;
	}
	dev_warn(m->dev, "reinit: LDO_STATUS 0x%04x ready after PHYLDO1_LP_EN = 0 (%u polls); sequence continues (deviation from the vendor sequence)\n",
		 *v, t + 1);
	return 0;
}

static int mt6382_seq_run(struct mt6382 *m, const struct mt6382_seq_op *ops,
			  unsigned int n, const char *name)
{
	unsigned int i, t, hit = 0;
	u32 v = 0, wr = 0;
	int ret = 0;

	mutex_lock(&m->lock);
	for (i = 0; i < n && !ret; i++) {
		const struct mt6382_seq_op *o = &ops[i];

		v = 0;
		wr = 0;
		switch (o->op) {
		case MT6382_SEQ_W:
			wr = o->val;
			ret = mt6382_write32_locked(m, o->addr, o->val);
			break;
		case MT6382_SEQ_M:
			ret = mt6382_read32_locked(m, o->addr, &v);
			wr = (v & ~o->mask) | o->val;
			if (!ret)
				ret = mt6382_write32_locked(m, o->addr, wr);
			break;
		case MT6382_SEQ_DELAY:
			wr = o->delay_us;
			fsleep(o->delay_us);
			break;
		case MT6382_SEQ_POLL:
			for (t = 0, hit = 0; t < o->tries; t++) {
				ret = mt6382_read32_locked(m, o->addr, &v);
				if (ret)
					break;
				if ((v & o->mask) == o->val && ++hit >= o->stable)
					break;
				fsleep(o->delay_us);
			}
			wr = t;
			if (!ret && hit < o->stable) {
				dev_warn(m->dev, "seq %s: 0x%05x = 0x%08x, waited for 0x%08x/0x%08x\n",
					 name, o->addr, v, o->val, o->mask);
				if (o->addr == MT6382_REG_LDO_STATUS && READ_ONCE(reinit_ldo_kick))
					ret = mt6382_ldo_kick_locked(m, o, &v);
				else if (o->addr != MT6382_TX_INTSTA_ULPS_POLL)
					ret = -ETIMEDOUT;
			}
			break;
		case MT6382_SEQ_EFUSE:
			ret = mt6382_seq_efuse_locked(m);
			break;
		}
		mt6382_trace_op(m, name, i, o, v, wr, ret);
	}
	mutex_unlock(&m->lock);

	if (ret)
		dev_err(m->dev, "seq %s: step %u of %u failed: %d\n", name, i, n, ret);
	else
		dev_dbg(m->dev, "seq %s: %u steps\n", name, n);
	return ret;
}

#define MT6382_SEQ(m, s)	mt6382_seq_run(m, mt6382_seq_##s, ARRAY_SIZE(mt6382_seq_##s), #s)

static inline struct mt6382 *bridge_to_mt6382(struct drm_bridge *b)
{
	return container_of(b, struct mt6382, bridge);
}

static inline struct mt6382 *panel_host_to_mt6382(struct mipi_dsi_host *h)
{
	return container_of(h, struct mt6382, panel_host);
}

static int mt6382_read32(struct mt6382 *m, u32 addr, u32 *val)
{
	int ret;

	mutex_lock(&m->lock);
	ret = mt6382_read32_locked(m, addr, val);
	mutex_unlock(&m->lock);
	return ret;
}

/* TX command queue writes, only while the TX is in command mode */
static int mt6382_tx_write32(struct mt6382 *m, u32 addr, u32 val)
{
	int ret;

	if (!m->tx_writes)
		return -EPERM;
	mutex_lock(&m->lock);
	m->scoped_write = true;
	ret = mt6382_write32_locked(m, addr, val);
	m->scoped_write = false;
	mutex_unlock(&m->lock);
	return ret;
}

static bool mt6382_tx_video_running(struct mt6382 *m)
{
	u32 start, mode, rx;

	if (mt6382_read32(m, MT6382_REG_DSI_TX_START, &start) ||
	    mt6382_read32(m, MT6382_REG_DSI_TX_MODE, &mode) ||
	    mt6382_read32(m, MT6382_REG_RX_MODE, &rx))
		return false;
	return (start & 1) && (mode & 3) && (rx & MT6382_RX_IPI_MODE);
}

/* panel DSI host: one packet through the TX command queue (format of mtk_dsi_cmdq) */
static ssize_t mt6382_panel_host_transfer(struct mipi_dsi_host *host,
					  const struct mipi_dsi_msg *msg)
{
	struct mt6382 *m = panel_host_to_mt6382(host);
	struct mipi_dsi_packet pkt;
	u32 words[MT6382_TX_CMDQ_WORDS] = { };
	unsigned int i, n;
	u32 sta, mode;
	int ret, t;

	if (msg->rx_len)
		return -EOPNOTSUPP;
	ret = mipi_dsi_create_packet(&pkt, msg);
	if (ret)
		return ret;
	if (pkt.payload_length > (MT6382_TX_CMDQ_WORDS - 1) * 4)
		return -EINVAL;

	/* the TX accepts queued commands only in command mode */
	ret = mt6382_read32(m, MT6382_REG_DSI_TX_MODE, &mode);
	if (ret)
		return ret;
	if (mode & 3) {
		dev_warn_ratelimited(m->dev, "panel command 0x%02x refused: TX in video mode\n",
				     pkt.header[0]);
		return -EBUSY;
	}

	if (mipi_dsi_packet_format_is_long(msg->type)) {
		words[0] = 2 | pkt.header[0] << 8 | pkt.header[1] << 16 | pkt.header[2] << 24;
		for (i = 0; i < pkt.payload_length; i++)
			words[1 + i / 4] |= (u32)pkt.payload[i] << (8 * (i % 4));
		n = 1 + DIV_ROUND_UP(pkt.payload_length, 4);
	} else {
		words[0] = pkt.header[0] << 8 | pkt.header[1] << 16 | pkt.header[2] << 24;
		n = 1;
	}

	for (i = 0; i < n && !ret; i++)
		ret = mt6382_tx_write32(m, MT6382_TX_CMDQ + 4 * i, words[i]);
	if (!ret)
		ret = mt6382_tx_write32(m, MT6382_TX_CMDQ_CON,
					n > 1 ? MT6382_TX_CMDQ_SIZE_SEL | n : 1);
	if (!ret)
		ret = mt6382_tx_write32(m, MT6382_REG_DSI_TX_START, 0);
	if (!ret)
		ret = mt6382_tx_write32(m, MT6382_REG_DSI_TX_START, 1);
	if (ret)
		return ret;

	for (t = 0; t < 100; t++) {
		ret = mt6382_read32(m, MT6382_TX_INTSTA, &sta);
		if (ret)
			return ret;
		if (sta & MT6382_TX_CMD_DONE)
			break;
		usleep_range(500, 1000);
	}
	/* vendor: write 0 to clear CMD_DONE */
	mt6382_tx_write32(m, MT6382_TX_INTSTA, ~MT6382_TX_CMD_DONE & 0xffff);
	if (!(sta & MT6382_TX_CMD_DONE))
		return -ETIMEDOUT;
	return pkt.payload_length;
}

static int mt6382_panel_host_attach(struct mipi_dsi_host *host,
				    struct mipi_dsi_device *dsi)
{
	struct mt6382 *m = panel_host_to_mt6382(host);

	if (dsi->lanes != 4 || dsi->format != MIPI_DSI_FMT_RGB888)
		dev_warn(m->dev, "panel: %u lanes, format %d (expected 4, RGB888)\n",
			 dsi->lanes, dsi->format);
	m->panel_dsi = dsi;
	dev_info(m->dev, "panel %s attached (DSC %s)\n", dev_name(&dsi->dev),
		 dsi->dsc ? "yes" : "no");
	return 0;
}

static int mt6382_panel_host_detach(struct mipi_dsi_host *host,
				    struct mipi_dsi_device *dsi)
{
	struct mt6382 *m = panel_host_to_mt6382(host);

	m->panel_dsi = NULL;
	return 0;
}

static const struct mipi_dsi_host_ops mt6382_panel_host_ops = {
	.attach = mt6382_panel_host_attach,
	.detach = mt6382_panel_host_detach,
	.transfer = mt6382_panel_host_transfer,
};

static int mt6382_bridge_attach(struct drm_bridge *bridge,
				struct drm_encoder *encoder,
				enum drm_bridge_attach_flags flags)
{
	struct mt6382 *m = bridge_to_mt6382(bridge);

	if (!(flags & DRM_BRIDGE_ATTACH_NO_CONNECTOR))
		return -EINVAL;
	if (!m->panel)
		return -EPROBE_DEFER;
	return 0;
}

static int mt6382_bridge_get_modes(struct drm_bridge *bridge,
				   struct drm_connector *connector)
{
	struct mt6382 *m = bridge_to_mt6382(bridge);

	return drm_panel_get_modes(m->panel, connector);
}

/* the panel mode in, the RX (SoC host) timing out */
static bool mt6382_bridge_mode_fixup(struct drm_bridge *bridge,
				     const struct drm_display_mode *mode,
				     struct drm_display_mode *adj)
{
	adj->clock = MT6382_HOST_CLOCK_KHZ;
	adj->hsync_start = mode->hdisplay + MT6382_HOST_HFP;
	adj->hsync_end = adj->hsync_start + MT6382_HOST_HSYNC;
	adj->htotal = adj->hsync_end + MT6382_HOST_HBP;
	/* vertical timing as the panel (TX and RX run line-locked) */
	return true;
}

static enum drm_mode_status
mt6382_bridge_mode_valid(struct drm_bridge *bridge,
			 const struct drm_display_info *info,
			 const struct drm_display_mode *mode)
{
	/* the RX side is set up for one picture size only */
	if (mode->hdisplay != 1080 || mode->vdisplay != 2460)
		return MODE_BAD;
	return MODE_OK;
}

/*
 * Prologue of the vendor bdg_common_init: reset pulse, spislv_init at low speed, NFC clock buffer.
 * XO_EXTBUF3 is on (left on by lk).
 */
static int mt6382_reinit_reset(struct mt6382 *m)
{
	int ret, dir;
	u32 id = 0;

	if (!READ_ONCE(reinit_reset)) {
		dev_info(m->dev, "reinit: no reset pulse (reinit_reset=0)\n");
		return 0;
	}
	if (!m->reset_gpio_tried) {
		struct gpio_desc *g;

		m->reset_gpio_tried = true;
		/* "as is": requesting switches the pin to GPIO mode (vendor pinmux 0x2000: mode 0) */
		g = devm_gpiod_get_optional(m->dev, "reset", GPIOD_ASIS);
		if (IS_ERR(g))
			return dev_err_probe(m->dev, PTR_ERR(g), "reinit: reset gpio\n");
		m->reset_gpio = g;
	}
	if (!m->reset_gpio) {
		dev_warn(m->dev, "reinit: no reset-gpios in DT, bring-up without reset\n");
		return 0;
	}

	dir = gpiod_get_direction(m->reset_gpio);
	dev_info(m->dev, "reinit: RESETB before the pulse: %s, raw level %d (1 = released)\n",
		 dir == GPIO_LINE_DIRECTION_OUT ? "output" :
		 dir == GPIO_LINE_DIRECTION_IN ? "input" : "direction unknown",
		 gpiod_get_raw_value_cansleep(m->reset_gpio));

	/* bdg_tx_pull_6382_reset_pin: high, 10 us, low, 10 us, high (active low line) */
	ret = gpiod_direction_output(m->reset_gpio, 0);
	if (ret)
		return ret;
	udelay(10);
	gpiod_set_value_cansleep(m->reset_gpio, 1);
	udelay(10);
	gpiod_set_value_cansleep(m->reset_gpio, 0);
	/* the vendor goes straight on to spislv_init; 1 ms margin (assumption, no datasheet) */
	fsleep(1000);

	mutex_lock(&m->lock);
	ret = mt6382_vendor_init_locked(m);
	if (!ret)
		ret = mt6382_read_chip_id_locked(m);
	id = m->chip_id;
	if (!ret)
		ret = mt6382_write32_locked(m, MT6382_REG_CKBUF_CTRL, MT6382_CKBUF_NFC_OFF);
	mutex_unlock(&m->lock);

	if (ret)
		dev_err(m->dev, "reinit: after the reset pulse: SPI slave set-up/chip id/NFC clock: %d (id 0x%08x, status 0x%02x)\n",
			ret, id, m->last_status);
	else
		dev_info(m->dev, "reinit: reset pulse, SPI slave set up, chip id 0x%08x, NFC clock buffer off\n", id);
	return ret;
}

/*
 * One complete bring-up. *panel_touched tells whether the panel was powered
 * (a retry would have to switch it off first, so there is none then).
 */
static int mt6382_reinit_once(struct mt6382 *m, bool *panel_touched)
{
	int ret;

	*panel_touched = false;
	mt6382_snapshot(m, "before power-off");
	/* host lanes are in LP-11 now: CRTC enable powered the SoC DSI on */
	if (MT6382_SEQ(m, power_off))
		dev_warn(m->dev, "enable: bridge power-off before the init incomplete\n");
	mt6382_snapshot(m, "after power-off");
	ret = mt6382_reinit_reset(m);
	if (!ret) {
		mt6382_snapshot(m, "after reset");
		ret = MT6382_SEQ(m, power_on);
		mt6382_snapshot(m, "after power_on");
	}
	if (!ret)
		ret = MT6382_SEQ(m, config);
	if (!ret)
		ret = MT6382_SEQ(m, rx_mac);
	if (!ret)
		ret = MT6382_SEQ(m, stopstate);
	if (!ret) {
		*panel_touched = true;
		m->tx_writes = true;
		drm_panel_prepare(m->panel);
		m->tx_writes = false;
		if (!m->panel->prepared)
			ret = -EIO;
	}
	if (!ret)
		ret = MT6382_SEQ(m, tx_video);
	if (ret)
		mt6382_snapshot(m, "after the failure");
	return ret;
}

static void mt6382_bridge_pre_enable(struct drm_bridge *bridge,
				     struct drm_atomic_state *state)
{
	struct mt6382 *m = bridge_to_mt6382(bridge);

	if (m->takeover_pending) {
		/*
		 * Takeover: lk left RX, DSC, TX and the panel running; the
		 * SoC host keeps its link (inherit-boot-state). Nothing to do.
		 */
		dev_info(m->dev, "enable: takeover, bridge left as lk set it up\n");
		return;
	}

	if (m->bridge_off) {
		unsigned int attempt, retries = READ_ONCE(reinit_retries);
		bool panel_touched = false;
		int ret = -EINVAL;

		if (m->trace) {
			unsigned int run = m->trace->run + 1;

			memset(m->trace, 0, sizeof(*m->trace));
			m->trace->run = run;
		}
		for (attempt = 0; attempt <= retries; attempt++) {
			if (attempt)
				dev_warn(m->dev, "enable: re-initialisation attempt %u of %u\n",
					 attempt + 1, retries + 1);
			ret = mt6382_reinit_once(m, &panel_touched);
			if (!ret || panel_touched)
				break;
		}
		if (m->trace)
			m->trace->result = ret;
		if (ret) {
			dev_err(m->dev,
				"enable: re-initialisation failed (%d), the panel stays dark; blank/unblank tries again, details in debugfs mt6382/reinit_trace\n",
				ret);
			return;
		}
		m->bridge_off = false;
		dev_info(m->dev, "enable: bridge and panel re-initialised, TX in video mode\n");
		return;
	}

	if (!mt6382_tx_video_running(m))
		dev_err(m->dev, "enable: bridge not running (reinit=0 or allow_writes=0 when it was switched off), the panel stays dark\n");
	else
		dev_info(m->dev, "enable: bridge still running, link restarted by the host\n");
}

static void mt6382_bridge_enable(struct drm_bridge *bridge,
				 struct drm_atomic_state *state)
{
	struct mt6382 *m = bridge_to_mt6382(bridge);

	m->takeover_pending = false;
	if (m->bridge_off) {
		/*
		 * The re-initialisation failed: do not power and reset the panel
		 * half-way (that also resets the touch part of the TDDI chip)
		 * and keep the backlight off.
		 */
		dev_warn(m->dev, "enable: bridge not initialised, panel and backlight stay off\n");
		return;
	}
	/*
	 * Takeover: the panel driver keeps the running panel in prepare.
	 * After a backlight-only off the panel stays prepared; enable switches
	 * the backlight and tells the panel followers (touch).
	 */
	if (!m->panel->prepared)
		drm_panel_prepare(m->panel);
	drm_panel_enable(m->panel);
}

static void mt6382_bridge_disable(struct drm_bridge *bridge,
				  struct drm_atomic_state *state)
{
	struct mt6382 *m = bridge_to_mt6382(bridge);

	/*
	 * Without reinit, "off" is the backlight only: panel and bridge stay
	 * powered, the host link stops. With reinit (and allow_writes) the TX
	 * stops and goes to command mode while the host still sends video,
	 * then the panel gets display off / sleep in through the TX command
	 * queue and loses its supplies. The bridge itself is powered down at
	 * the next enable, right before it is brought up again: only then the
	 * host lanes are known to be in LP-11 (the CRTC is disabled after the
	 * bridge chain, there is no later bridge hook).
	 */
	drm_panel_disable(m->panel);
	if (!READ_ONCE(reinit) || !READ_ONCE(allow_writes))
		return;

	if (MT6382_SEQ(m, tx_command))
		dev_warn(m->dev, "disable: TX stop failed, switching the panel off anyway\n");
	m->tx_writes = true;
	drm_panel_unprepare(m->panel);
	m->tx_writes = false;
	m->bridge_off = true;
	dev_info(m->dev, "disable: TX stopped, panel off (reinit)\n");
}

static const struct drm_bridge_funcs mt6382_bridge_funcs = {
	.attach = mt6382_bridge_attach,
	.get_modes = mt6382_bridge_get_modes,
	.mode_fixup = mt6382_bridge_mode_fixup,
	.mode_valid = mt6382_bridge_mode_valid,
	.atomic_pre_enable = mt6382_bridge_pre_enable,
	.atomic_enable = mt6382_bridge_enable,
	.atomic_disable = mt6382_bridge_disable,
	.atomic_duplicate_state = drm_atomic_helper_bridge_duplicate_state,
	.atomic_destroy_state = drm_atomic_helper_bridge_destroy_state,
	.atomic_reset = drm_atomic_helper_bridge_reset,
};

static void mt6382_drm_work(struct work_struct *work)
{
	struct mt6382 *m = container_of(to_delayed_work(work), struct mt6382, drm_work);
	struct device *dev = m->dev;
	const struct mipi_dsi_device_info info = {
		.type = "mt6382",
		.channel = 0,
		.node = NULL,
	};
	struct mipi_dsi_host *host;
	struct drm_panel *panel;
	int ret;

	if (m->drm_done || !READ_ONCE(drm))
		return;

	if (!m->connected) {
		dev_err(dev, "drm: bridge not connected over SPI, not starting\n");
		return;
	}

	if (!m->drm_host_registered) {
		m->tx_running = mt6382_tx_video_running(m);
		if (m->tx_running) {
			m->takeover_pending = true;
		} else if (READ_ONCE(reinit) && READ_ONCE(allow_writes)) {
			/* e.g. the MM power domain was cycled: full bring-up at the first enable */
			dev_info(dev, "drm: bridge not running as lk leaves it, first enable re-initialises it (reinit)\n");
			m->bridge_off = true;
		} else {
			dev_err(dev, "drm: bridge TX/RX not running as lk leaves it; takeover impossible, not starting\n");
			return;
		}

		m->panel_host.dev = dev;
		m->panel_host.ops = &mt6382_panel_host_ops;
		ret = mipi_dsi_host_register(&m->panel_host);
		if (ret) {
			dev_err(dev, "drm: panel DSI host: %d\n", ret);
			return;
		}
		m->drm_host_registered = true;
	}

	panel = of_drm_find_panel(m->panel_node);
	if (IS_ERR(panel)) {
		if (PTR_ERR(panel) == -EPROBE_DEFER && ++m->drm_tries < 20) {
			schedule_delayed_work(&m->drm_work, msecs_to_jiffies(500));
			return;
		}
		dev_err(dev, "drm: panel not found: %ld\n", PTR_ERR(panel));
		return;
	}
	m->panel = panel;

	host = of_find_mipi_dsi_host_by_node(m->soc_host_node);
	if (!host) {
		if (++m->drm_tries < 20) {
			schedule_delayed_work(&m->drm_work, msecs_to_jiffies(500));
			return;
		}
		dev_err(dev, "drm: SoC DSI host not found\n");
		return;
	}

	if (!m->drm_bridge_added) {
		m->bridge.of_node = dev->of_node;
		m->bridge.type = DRM_MODE_CONNECTOR_DSI;
		m->bridge.ops = DRM_BRIDGE_OP_MODES;
		m->bridge.pre_enable_prev_first = true;
		drm_bridge_add(&m->bridge);
		m->drm_bridge_added = true;
	}

	m->dsi = mipi_dsi_device_register_full(host, &info);
	if (IS_ERR(m->dsi)) {
		dev_err(dev, "drm: DSI device: %ld\n", PTR_ERR(m->dsi));
		m->dsi = NULL;
		return;
	}
	m->dsi->lanes = 4;
	m->dsi->format = MIPI_DSI_FMT_RGB888;
	m->dsi->mode_flags = MIPI_DSI_MODE_VIDEO | MIPI_DSI_MODE_VIDEO_SYNC_PULSE |
			     MIPI_DSI_CLOCK_NON_CONTINUOUS;
	ret = mipi_dsi_attach(m->dsi);
	if (ret) {
		dev_err(dev, "drm: attach to the SoC DSI host: %d\n", ret);
		mipi_dsi_device_unregister(m->dsi);
		m->dsi = NULL;
		return;
	}
	m->drm_done = true;
	dev_info(dev, "drm: DSI device attached to %pOF, takeover of the running bridge\n",
		 m->soc_host_node);
}

static int mt6382_drm_set(const char *val, const struct kernel_param *kp)
{
	int ret = param_set_bool(val, kp);

	if (!ret && READ_ONCE(drm) && READ_ONCE(mt6382_dev))
		schedule_delayed_work(&mt6382_dev->drm_work,
				      msecs_to_jiffies(READ_ONCE(drm_delay_ms)));
	return ret;
}

static int mt6382_drm_init(struct mt6382 *m)
{
	struct device_node *np = m->dev->of_node;

	INIT_DELAYED_WORK(&m->drm_work, mt6382_drm_work);

	m->soc_host_node = of_graph_get_remote_node(np, 0, -1);
	m->panel_node = of_get_child_by_name(np, "panel");
	if (!m->soc_host_node || !m->panel_node) {
		dev_info(m->dev, "drm: no DSI graph/panel in DT, DRM support off\n");
		of_node_put(m->soc_host_node);
		of_node_put(m->panel_node);
		m->soc_host_node = NULL;
		m->panel_node = NULL;
		return 0;
	}

	m->trace = devm_kzalloc(m->dev, sizeof(*m->trace), GFP_KERNEL);
	if (m->trace && m->debugfs)
		debugfs_create_file("reinit_trace", 0400, m->debugfs, m, &mt6382_trace_fops);

	WRITE_ONCE(mt6382_dev, m);
	if (READ_ONCE(drm))
		schedule_delayed_work(&m->drm_work, msecs_to_jiffies(READ_ONCE(drm_delay_ms)));
	dev_info(m->dev, "drm: %s (switch: /sys/module/mediatek_mt6382/parameters/drm, delay %u ms)\n",
		 READ_ONCE(drm) ? "scheduled" : "off", READ_ONCE(drm_delay_ms));
	return 0;
}

static void mt6382_drm_remove(void *data)
{
	struct mt6382 *m = data;

	WRITE_ONCE(mt6382_dev, NULL);
	if (!m->soc_host_node)
		return;
	/* the trace buffer is devm memory freed right after this action */
	debugfs_lookup_and_remove("reinit_trace", m->debugfs);
	cancel_delayed_work_sync(&m->drm_work);
	if (m->dsi) {
		mipi_dsi_detach(m->dsi);
		mipi_dsi_device_unregister(m->dsi);
	}
	if (m->drm_bridge_added)
		drm_bridge_remove(&m->bridge);
	if (m->drm_host_registered)
		mipi_dsi_host_unregister(&m->panel_host);
	of_node_put(m->soc_host_node);
	of_node_put(m->panel_node);
}

/* -------------------------------------------------------------------- probe */

static void mt6382_restore_controller_data(void *data)
{
	struct mt6382 *m = data;

	m->spi->controller_data = m->orig_controller_data;
}

static void mt6382_debugfs_remove(void *data)
{
	struct mt6382 *m = data;

	debugfs_remove_recursive(m->debugfs);
}

static void mt6382_dump_regs(struct mt6382 *m)
{
	unsigned int i, val, ldo = 0, tx_start = 0, tx_mode = 0, rx_mode = 0;
	unsigned int fails = 0;
	int ret;

	for (i = 0; i < ARRAY_SIZE(mt6382_known_regs); i++) {
		u32 reg = mt6382_known_regs[i];

		ret = regmap_read(m->regmap, reg, &val);
		if (ret) {
			dev_warn(m->dev, "addr:0x%08x, err:%d\n", reg, ret);
			if (++fails >= MT6382_PROBE_MAX_FAILS) {
				dev_warn(m->dev, "%u read errors in a row, register list skipped\n",
					 fails);
				return;
			}
			continue;
		}
		fails = 0;
		dev_info(m->dev, "addr:0x%08x, val:0x%08x\n", reg, val);

		switch (reg) {
		case MT6382_REG_LDO_STATUS:
			ldo = val;
			break;
		case MT6382_REG_DSI_TX_START:
			tx_start = val;
			break;
		case MT6382_REG_DSI_TX_MODE:
			tx_mode = val;
			break;
		case MT6382_REG_RX_MODE:
			rx_mode = val;
			break;
		}
	}

	dev_info(m->dev,
		 "MT6382 chip id 0x%08x, LDO_STATUS 0x%04x, DSI-TX start %u mode %u, RX mode %u; taken over (allow_writes=%d, SPI %u Hz)\n",
		 m->chip_id, ldo, tx_start, tx_mode, rx_mode,
		 READ_ONCE(allow_writes), m->rx_hz);
}

static int mt6382_probe(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct mt6382 *m;
	int ret;

	m = devm_drm_bridge_alloc(dev, struct mt6382, bridge, &mt6382_bridge_funcs);
	if (IS_ERR(m))
		return PTR_ERR(m);
	m->bufs = devm_kzalloc(dev, sizeof(*m->bufs), GFP_KERNEL);
	if (!m->bufs)
		return -ENOMEM;

	m->dev = dev;
	m->spi = spi;
	mutex_init(&m->lock);
	spi_set_drvdata(spi, m);

	/* stay at the vendor's power-on speed; never above the DT limit */
	m->tx_hz = min_t(u32, MT6382_SPI_LOW_HZ, spi->max_speed_hz);
	m->rx_hz = min_t(u32, MT6382_SPI_LOW_HZ, spi->max_speed_hz);
	m->cs_time_ns = MT6382_CS_TIME_NS;

	/*
	 * The vendor driver hands its settings to spi-mt65xx through
	 * controller_data (spi_slave.c spislv_chip_info): rx_mlsb = tx_mlsb = 0,
	 * i.e. LSB first, sample_sel 0, low-speed tick delay. Do the same, but
	 * only on that controller.
	 */
	m->orig_controller_data = spi->controller_data;
	m->chip_cfg.sample_sel = 0;
	m->chip_cfg.tick_delay = MT6382_MTK_TICK_DELAY_LOW;
	if (of_device_is_compatible(spi->controller->dev.of_node, "mediatek,mt6765-spi")) {
		spi->controller_data = &m->chip_cfg;
		m->mtk_controller = true;
		ret = devm_add_action_or_reset(dev, mt6382_restore_controller_data, m);
		if (ret)
			return ret;
	}

	spi->mode = (spi->mode & ~SPI_MODE_X_MASK) | SPI_MODE_0 | SPI_LSB_FIRST;
	ret = mt6382_apply_spi_settings(m);
	if (ret)
		return dev_err_probe(dev, ret, "spi_setup failed\n");

	m->regmap = devm_regmap_init(dev, NULL, m, &mt6382_regmap_config);
	if (IS_ERR(m->regmap))
		return dev_err_probe(dev, PTR_ERR(m->regmap), "regmap init failed\n");

	mt6382_debugfs_init(m);
	ret = devm_add_action_or_reset(dev, mt6382_debugfs_remove, m);
	if (ret)
		return ret;

	dev_info(dev, "SPI mode %u, %s first, tx/rx %u/%u Hz, tick delay %u%s, CS time %u ns, allow_writes=%d, init_on_cfg_error=%d\n",
		 (unsigned int)(spi->mode & SPI_MODE_X_MASK),
		 spi->mode & SPI_LSB_FIRST ? "LSB" : "MSB",
		 m->tx_hz, m->rx_hz, m->chip_cfg.tick_delay,
		 m->mtk_controller ? "" : " (not applied)", m->cs_time_ns,
		 READ_ONCE(allow_writes), init_on_cfg_error);

	ret = mt6382_connect(m, init_on_cfg_error);
	if (ret) {
		/* stay bound: debugfs remains available */
		dev_warn(dev, "bridge not connected (%d, id 0x%08x, last status 0x%02x); debugfs stays available\n",
			 ret, m->chip_id, m->last_status);
		return 0;
	}

	mt6382_dump_regs(m);

	ret = mt6382_drm_init(m);
	if (ret)
		return ret;
	return devm_add_action_or_reset(dev, mt6382_drm_remove, m);
}

static const struct of_device_id mt6382_of_match[] = {
	{ .compatible = "mediatek,mt6382" },
	{ }
};
MODULE_DEVICE_TABLE(of, mt6382_of_match);

static const struct spi_device_id mt6382_spi_ids[] = {
	{ "mt6382" },
	{ }
};
MODULE_DEVICE_TABLE(spi, mt6382_spi_ids);

static struct spi_driver mt6382_driver = {
	.driver = {
		.name = "mediatek-mt6382",
		.of_match_table = mt6382_of_match,
		/* never hold up the boot on the bridge */
		.probe_type = PROBE_PREFER_ASYNCHRONOUS,
	},
	.probe = mt6382_probe,
	.id_table = mt6382_spi_ids,
};
module_spi_driver(mt6382_driver);

MODULE_DESCRIPTION("MediaTek MT6382 MIPI-DSI bridge, SPI control (takeover)");
MODULE_LICENSE("GPL");
