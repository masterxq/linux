// SPDX-License-Identifier: GPL-2.0-only
/*
 * Sensor hub of the SCP of the Motorola Moto G60s (lisbon, MT6785).
 *
 * The SCP firmware (tinysys/nanohub, loaded by lk from the scp partition) runs the sensor
 * drivers itself. This driver binds to the sensor-hub node below the SCP node (mtk_scp creates
 * the device), starts the SCP (remoteproc attach to the image of lk; autostart=0 leaves that to
 * the remoteproc sysfs) and talks to the firmware over the IPI channel of mtk_scp:
 *  - once the hub reports "init done" it asks for the chip behind each sensor type, hands
 *    the sensor FIFO in the shared DRAM region to the hub (SET_CONFIG) and syncs the time
 *    base (SET_TIMESTAMP, every 10 s),
 *  - it enables and disables sensors with the nanohub config command, sent as a nanohub
 *    comms packet over the CHRE IPI pair (24 to the SCP, 25 back as "bytes taken"),
 *  - it reads the events the hub writes to the FIFO (sensor IPI, NOTIFY DIRECT_PUSH or
 *    FIFO_FULL with the new write pointer) and keeps the newest sample of each sensor for
 *    lisbon-scp-iio, which enables a sensor on its first read and disables it again after
 *    idle_ms without reads,
 *  - it sets up the SCP logger (ring buffer in the shared DRAM region) and exposes the log of
 *    the SCP, both the SRAM ring and the DRAM buffer,
 *  - it shows the SCP state (reset, clocks, registers, loader parameters, IPI counters),
 *  - before system suspend it detaches the SCP (the PMIC, SPM and DVFS setup the vendor
 *    makes for a hub that runs while the AP sleeps is missing) and attaches it again
 *    afterwards; with suspend_detach=0 it refuses the suspend instead,
 *  - after each hub setup it hands the factory calibration of the light/proximity sensor
 *    to the hub (config data, see ps_cali/als_cali) and switches the proximity driver of
 *    the hub to the mode the Android HAL uses while an app listens (config data 6),
 *  - then it has the hub calibrate the crosstalk offset of the proximity sensor, unless the
 *    sensor is covered at that moment (ps_xtalk),
 *  - after the first hub setup it adds the auxiliary device "lisbon_scp_hub.iio" for the
 *    IIO devices of lisbon-scp-iio.
 *
 * debugfs: /sys/kernel/debug/lisbon-scp/{status,sensors,hub,log,dramlog,eint}
 *   echo query > sensors          ask the hub again
 *   echo "on accel" > hub         enable a sensor until "off accel" (also gyro, mag, light,
 *                                 prox), "sync" sends the time base again
 *   echo "cfg prox 6 1" > hub     send config data (up to 8 integers) to a sensor
 *   echo "cali prox" > hub        start the calibration of a sensor; the result comes back
 *                                 as calibration event (hub: "cali" values)
 *   echo "get prox" > hub         ask the hub for the current data of a sensor (GET_DATA),
 *                                 answer packet shown as "get:" in hub
 *
 * Protocol values (IPI ids, sensor IPI packet of 48 bytes, actions and notify events, the
 * "get sensor info" request, SET_CONFIG/SET_TIMESTAMP layout, sensor FIFO header and the
 * 44 byte event, nanohub comms packet with sync byte, sequence, reason, CRC and the config
 * command, logger control block, layout of the shared DRAM region) follow the MediaTek
 * vendor kernel for MT6785 (SCP logger and IPI, nanohub sensor hub and comms drivers); only
 * values and layouts are taken over.
 */

#include <linux/bits.h>
#include <linux/clk.h>
#include <linux/crc32.h>
#include <linux/debugfs.h>
#include <linux/auxiliary_bus.h>
#include <linux/ctype.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/firmware.h>
#include <linux/io.h>
#include <linux/jiffies.h>
#include <linux/kstrtox.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/remoteproc.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/suspend.h>
#include <linux/timekeeping.h>
#include <linux/uaccess.h>
#include <linux/unaligned.h>
#include <linux/wait.h>
#include <linux/workqueue.h>

#include <clocksource/arm_arch_timer.h>

#include "mtk_common.h"
#include "lisbon-scp-hub.h"

/* IPI ids of the MT6785 SCP firmware */
#define LSH_IPI_LOGGER_WAKEUP	3
#define LSH_IPI_LOGGER_INIT	4
#define LSH_IPI_SENSOR		8
#define LSH_IPI_CHRE		24
#define LSH_IPI_CHREX		25
#define LSH_IPI_PLL_CTRL	26
#define LSH_IPI_ERROR_INFO	31

/* sensor IPI packet: type, action, errCode/event, reserved, 44 bytes of data */
#define LSH_PKT_SIZE		48
#define LSH_ACT_GET_DATA	2
#define LSH_ACT_SET_CONFIG	4
#define LSH_ACT_SET_CUST	5
#define LSH_ACT_NOTIFY		6
#define LSH_ACT_SET_TIMESTAMP	8
#define LSH_CUST_GET_SENSOR_INFO 12
#define LSH_NOTIFY_INIT_DONE	0
#define LSH_NOTIFY_FIFO_FULL	1
#define LSH_NOTIFY_DIRECT_PUSH	4
#define LSH_NOTIFY_EVENTS	5
#define LSH_RSP_TIMEOUT_MS	500

/* SET_CONFIG/SET_TIMESTAMP request and NOTIFY answer (64 bit fields are 8 byte aligned) */
#define LSH_CFG_BUF_BASE	4
#define LSH_CFG_BUF_SIZE	8
#define LSH_CFG_AP_TIME		16
#define LSH_CFG_ARCH_COUNTER	24
#define LSH_NOTIFY_WP		8

/* sensorInfo_t in the "get sensor info" answer, after type/action/err/rsv and the action word */
#define LSH_INFO_NAME		8
#define LSH_INFO_LIB		24
#define LSH_INFO_LAYOUT		40
#define LSH_INFO_DEVID		41
#define LSH_NAME_LEN		16

/* shared DRAM region (memory-region of the SCP): six blocks in this order */
#define LSH_SHARE_SENSOR_OFFSET	0x49200
#define LSH_SHARE_SENSOR_SIZE	0x100000
#define LSH_SHARE_LOGGER_OFFSET	(0x49200 + 0x100000 + 0x1000)
#define LSH_SHARE_LOGGER_SIZE	0x180000

/* sensor FIFO: header rp, wp, size, reserved, then 44 byte events (data_unit_t) */
#define LSH_FIFO_HDR		16
#define LSH_EVT_SIZE		44
#define LSH_EVT_TYPE		0
#define LSH_EVT_ACTION		1
#define LSH_EVT_TIME		4
#define LSH_EVT_DATA		12
#define LSH_EVT_ACTION_DATA	0
#define LSH_EVT_ACTIONS		6

/* nanohub comms packet: sync, seq, reason, len, data, CRC over all before */
#define LSH_COMMS_SYNC		0x31
#define LSH_COMMS_HDR		10
#define LSH_COMMS_CRC		4
#define LSH_COMMS_MAX_DATA	255
#define LSH_COMMS_ACK		0x0000
#define LSH_COMMS_NACK		0x0001
#define LSH_COMMS_BUSY		0x0002
#define LSH_COMMS_WRITE		0x1091
#define LSH_CHRE_TRIES		5
#define LSH_CHRE_ACK_MS		100
#define LSH_CHRE_REPLY_MS	300
#define LSH_CHRE_BUSY_MS	100

/* nanohub config command (struct ConfigCmd, packed, 20 bytes) */
#define LSH_EVT_NO_SENSOR_CONFIG 0x300
#define LSH_CONFIG_DISABLE	0
#define LSH_CONFIG_ENABLE	1
#define LSH_CONFIG_CFG_DATA	3
#define LSH_CONFIG_CALIBRATE	4
#define LSH_CONFIG_SIZE		20
#define LSH_CFG_MAX		8

/*
 * Config data of the TMD2755 driver in the hub (first word = type): proximity 0 = factory
 * thresholds (high, low, cover, uncover), 3 = calibration type for "cali" (0 crosstalk, 1 factory
 * thresholds, 2 interrupt line), 6 = mode (1: thresholds of the factory calibration, as while an
 * Android app listens; 0: thresholds of the hub's own offset calibration); light 0 = scale * 1000.
 */
#define LSH_PS_CFG_FACTORY	0
#define LSH_PS_CFG_CALI_TYPE	3
#define LSH_PS_CALI_XTALK	0
#define LSH_PS_CFG_MODE		6
#define LSH_ALS_CFG_SCALE	0
#define LSH_EVT_ACTION_CALI	3
#define LSH_EVT_ACTION_TEST	5
/* GET_DATA answer of the proximity sensor: data type, flag, raw count with the offset added */
#define LSH_GET_PS_TYPE		16
#define LSH_GET_PS_RAW		24
#define LSH_XTALK_SETTLE_MS	300
#define LSH_XTALK_WAIT_MS	2000

/*
 * SCP external interrupt block ("scp_cirq_eint" at 0x105ca000 in the vendor mt6785.dts);
 * the hub takes the proximity interrupt of the TMD2755 there.
 * Read only, for debugfs, while the SCP runs.
 */
#define LSH_EINT_BASE		0x105ca000
#define LSH_EINT_SIZE		0x1000
#define LSH_EINT_SHOW_END	0x440

#define LSH_SYNC_PERIOD_MS	10000
#define LSH_ATTACH_WAIT_MS	5000

/* logger control block at the start of the logger block */
#define LSH_LOG_MAGIC		0x504c5402
#define LSH_LOG_INIT_MAGIC	0x5a5a5a5a
#define LSH_LOG_CTL_SIZE	24
#define LSH_LOG_BUF_OFS		32
#define LSH_LOG_BUF_SIZE	0x100000
#define LSH_LOG_SRAM_MAX	0x4000

/* register views for the status file */
#define LSH_INFRA_CG0_STA	0x90
#define LSH_INFRA_CG_SCP	BIT(4)
#define LSH_INFRA_RST_STA	0x148
#define LSH_INFRA_IRQ_SET	0xb14
#define LSH_CLK_SW_SEL		0x4000
#define LSH_CLK_ENABLE		0x4004
#define LSH_CLK_SRAM_PDN	0x402c
#define LSH_CLK_L1_SRAM_PD	0x4080
#define LSH_CLK_TCM_TAIL_PD	0x4094

struct lsh_log_ctl {
	u32 base;
	u32 size;
	u32 enable;
	u32 info_ofs;
	u32 buff_ofs;
	u32 buff_size;
	u32 r_pos;
	u32 w_pos;
};

/* SRAM offsets of the SCP log ring, sent by the firmware with LOGGER_INIT */
struct lsh_log_info {
	u32 dram_addr;
	u32 buf_addr;
	u32 start_addr;
	u32 end_addr;
	u32 buf_maxlen;
};

struct lsh_fifo {
	u32 rp;
	u32 wp;
	u32 size;
	u32 reserved;
	u8 data[];
};

struct lsh;

/* one sensor type of the hub */
struct lsh_sensor {
	struct lsh *h;
	u8 handle;		/* Android sensor type - 1, as in the IPI packets and events */
	const char *label;
	int err;
	char name[LSH_NAME_LEN + 1];
	char lib[LSH_NAME_LEN + 1];
	s8 layout;
	s8 devid;

	/* enable state, only for the first LISBON_SCP_NUM_SENSORS entries */
	struct mutex lock;
	bool enabled;
	bool manual;
	unsigned int gen;
	unsigned long last_read;
	int config_result;
	struct delayed_work idle_work;

	/* newest sample, protected by lsh.data_lock */
	struct lisbon_scp_sample sample;
	bool valid;
	unsigned int events[LSH_EVT_ACTIONS];
	/* values of the last calibration or self test event, protected by lsh.data_lock */
	s32 cali[6];
};

/* sensor handles of the hub; the first entries follow enum lisbon_scp_sensor */
static const struct {
	u8 handle;
	const char *label;
} lsh_types[] = {
	{ 0, "accel" }, { 3, "gyro" }, { 1, "mag" },
	{ 4, "light" }, { 7, "prox" }, { 5, "baro" },
};

/*
 * Firmware rate per sensor: Hz * 1024 (12800 = 12.5 Hz). Light and proximity are on-change sensors,
 * the rate is a request only.
 */
static unsigned int rates[LISBON_SCP_NUM_SENSORS] = { 12800, 12800, 12800, 5120, 5120 };
module_param_array(rates, uint, NULL, 0644);
MODULE_PARM_DESC(rates, "rate per sensor (accel,gyro,mag,light,prox) in Hz * 1024");

static unsigned int idle_ms = 5000;
module_param(idle_ms, uint, 0644);
MODULE_PARM_DESC(idle_ms, "disable a sensor this long after its last read");

static unsigned int first_ms = 1000;
module_param(first_ms, uint, 0644);
MODULE_PARM_DESC(first_ms, "wait this long for the first sample after enabling a sensor");

static bool suspend_detach = true;
module_param(suspend_detach, bool, 0644);
MODULE_PARM_DESC(suspend_detach, "detach the SCP before system suspend (0: refuse the suspend)");

static bool dramlog = true;
module_param(dramlog, bool, 0444);
MODULE_PARM_DESC(dramlog, "hand the DRAM log buffer to the SCP logger");

/*
 * Factory calibration of the TMD2755, as the vendor HAL sends it to the hub: each unit has
 * it in its nvcfg partition, sensor/ps_cali.json (high and low threshold, cover, uncover)
 * and sensor/als_cali.json (light scale * 1000), e.g. {"ps_cali": [198, 143, 750, 509]}.
 * The driver reads copies of these files as firmware (LSH_PS_CALI_FW, LSH_ALS_CALI_FW) when
 * it binds; the module parameters, when set, take precedence. Without both the hub uses its
 * defaults.
 */
#define LSH_PS_CALI_FW		"lisbon/sensor/ps_cali.json"
#define LSH_ALS_CALI_FW		"lisbon/sensor/als_cali.json"

static int ps_cali[4];
module_param_array(ps_cali, int, NULL, 0644);
MODULE_PARM_DESC(ps_cali, "proximity factory calibration high,low,cover,uncover (0,0,0,0: from "
		 LSH_PS_CALI_FW ")");

static int als_cali;
module_param(als_cali, int, 0644);
MODULE_PARM_DESC(als_cali, "light factory scale * 1000 (0: from " LSH_ALS_CALI_FW ")");

static bool autostart = true;
module_param(autostart, bool, 0444);
MODULE_PARM_DESC(autostart, "start the SCP when the driver binds (0: remoteproc sysfs only)");

static bool prox_mode = true;
module_param(prox_mode, bool, 0644);
MODULE_PARM_DESC(prox_mode, "switch the hub proximity driver to the mode of the Android HAL on enable");

static bool ps_xtalk = true;
module_param(ps_xtalk, bool, 0644);
MODULE_PARM_DESC(ps_xtalk, "proximity crosstalk calibration after each hub setup");

struct lsh {
	struct mtk_scp *scp;
	struct rproc *rproc;
	struct device *dev;
	struct dentry *dir;
	struct rproc_subdev subdev;
	struct notifier_block pm_nb;

	/* one sensor IPI request at a time */
	struct mutex xfer_lock;
	/* protects the sensor IPI rx state, the CHRE rx slot, wp_pending and the counters */
	spinlock_t rx_lock;
	struct completion rx_done;
	u8 tx[LSH_PKT_SIZE];
	u8 rx[LSH_PKT_SIZE];
	bool waiting;
	unsigned int rx_count;
	unsigned int rx_unmatched;
	unsigned int notify[LSH_NOTIFY_EVENTS];
	u8 last_notify[LSH_PKT_SIZE];
	bool init_done;

	struct lsh_sensor sensors[ARRAY_SIZE(lsh_types)];
	unsigned int queries;

	/* hub state: set up once per SCP run */
	struct work_struct setup_work;
	bool hub_ready;
	unsigned int gen;
	int setup_err;
	wait_queue_head_t data_wq;
	struct delayed_work sync_work;
	unsigned int syncs;
	int sync_err;

	/* sensor FIFO, protected by data_lock */
	struct lsh_fifo *fifo;
	phys_addr_t fifo_phys;
	u32 fifo_size;
	u32 wp_pending;
	struct work_struct fifo_work;
	/* protects the FIFO read side and the newest samples of the sensors */
	spinlock_t data_lock;
	unsigned int fifo_bad;
	unsigned int fifo_events;
	unsigned int fifo_unknown;

	/* CHRE comms, one transaction at a time */
	struct mutex chre_lock;
	struct completion chre_done;
	bool chre_wait;
	u8 chre_buf[LSH_COMMS_HDR + LSH_COMMS_MAX_DATA + LSH_COMMS_CRC];
	unsigned int chre_len;
	u32 chre_seq;
	unsigned int chre_rx;
	unsigned int chre_tx;
	unsigned int chre_err;
	unsigned int chre_unsolicited;
	unsigned int chre_ack_len;
	struct work_struct chre_ack_work;
	int chre_last;
	u8 chre_last_pkt[2][32];
	unsigned int chre_last_len[2];

	/* start of the SCP when the driver binds; booted: this driver holds the rproc */
	struct work_struct boot_work;
	bool booted;
	int boot_err;

	/* IIO devices, added after the first hub setup */
	struct auxiliary_device *iio;

	/* factory calibration from the firmware files, 0 if there is none */
	int fw_ps_cali[4];
	int fw_als_cali;

	/* system suspend */
	struct work_struct resume_work;
	bool pm_detached;
	int pm_err;

	struct work_struct logger_work;
	/* proximity crosstalk calibration after the hub setup */
	struct work_struct xtalk_work;
	int xtalk_err;
	s32 xtalk_raw;
	struct lsh_log_info log_info;
	bool log_info_valid;
	int logger_reply;
	struct lsh_log_ctl *log_ctl;
	phys_addr_t log_phys;
	/* protects log_info and the read position of the DRAM log */
	struct mutex log_lock;

	unsigned int logger_wakeups;
	unsigned int pll_requests;
	u32 pll_last[2];
	unsigned int err_infos;
	char last_err[64];

	void __iomem *eint;
	int cali_err;

	/* last GET_DATA answer, for debugfs */
	u8 get_rsp[LSH_PKT_SIZE];
	int get_err;
	unsigned int gets;
};

static struct lsh *lsh_dev;

static bool lsh_running(struct lsh *h)
{
	return h->scp->running && h->rproc->state == RPROC_ATTACHED;
}

/* ---- sensor IPI 8: requests and notifications ---- */

static void lsh_sensor_ipi(void *data, unsigned int len, void *priv)
{
	struct lsh *h = priv;
	u8 *p = data;
	unsigned long flags;
	bool setup = false, fifo = false;

	if (len < 4 || len > LSH_PKT_SIZE)
		return;

	spin_lock_irqsave(&h->rx_lock, flags);
	h->rx_count++;
	if (p[1] == LSH_ACT_NOTIFY) {
		if (p[2] < LSH_NOTIFY_EVENTS)
			h->notify[p[2]]++;
		memset(h->last_notify, 0, sizeof(h->last_notify));
		memcpy(h->last_notify, p, len);
		if (p[2] == LSH_NOTIFY_INIT_DONE) {
			setup = !h->init_done;
			h->init_done = true;
		} else if ((p[2] == LSH_NOTIFY_DIRECT_PUSH || p[2] == LSH_NOTIFY_FIFO_FULL) &&
			   len >= LSH_NOTIFY_WP + 4) {
			h->wp_pending = get_unaligned_le32(&p[LSH_NOTIFY_WP]);
			fifo = true;
		}
	} else if (h->waiting && p[0] == h->tx[0] && p[1] == h->tx[1]) {
		memset(h->rx, 0, sizeof(h->rx));
		memcpy(h->rx, p, len);
		h->waiting = false;
		complete(&h->rx_done);
	} else {
		h->rx_unmatched++;
	}
	spin_unlock_irqrestore(&h->rx_lock, flags);

	if (setup)
		schedule_work(&h->setup_work);
	if (fifo)
		schedule_work(&h->fifo_work);
}

/* send one sensor IPI request and wait for the answer of the same type and action */
static int lsh_request(struct lsh *h, const u8 *req, u8 *rsp)
{
	unsigned long flags;
	int ret;

	lockdep_assert_held(&h->xfer_lock);

	spin_lock_irqsave(&h->rx_lock, flags);
	memcpy(h->tx, req, LSH_PKT_SIZE);
	reinit_completion(&h->rx_done);
	h->waiting = true;
	spin_unlock_irqrestore(&h->rx_lock, flags);

	ret = scp_ipi_send(h->scp, LSH_IPI_SENSOR, req, LSH_PKT_SIZE, 0);
	if (!ret && !wait_for_completion_timeout(&h->rx_done,
						 msecs_to_jiffies(LSH_RSP_TIMEOUT_MS)))
		ret = -ETIMEDOUT;

	spin_lock_irqsave(&h->rx_lock, flags);
	h->waiting = false;
	if (!ret) {
		memcpy(rsp, h->rx, LSH_PKT_SIZE);
		if ((s8)rsp[2])
			ret = -EREMOTEIO;
	}
	spin_unlock_irqrestore(&h->rx_lock, flags);

	return ret;
}

static int lsh_query_one(struct lsh *h, struct lsh_sensor *s)
{
	u8 req[LSH_PKT_SIZE] = { s->handle, LSH_ACT_SET_CUST };
	u8 rsp[LSH_PKT_SIZE];
	unsigned long flags;
	int ret;

	put_unaligned_le32(LSH_CUST_GET_SENSOR_INFO, &req[4]);
	ret = lsh_request(h, req, rsp);
	if (!ret && get_unaligned_le32(&rsp[4]) != LSH_CUST_GET_SENSOR_INFO)
		ret = -EPROTO;

	spin_lock_irqsave(&h->rx_lock, flags);
	if (!ret) {
		memcpy(s->name, &rsp[LSH_INFO_NAME], LSH_NAME_LEN);
		s->name[LSH_NAME_LEN] = '\0';
		memcpy(s->lib, &rsp[LSH_INFO_LIB], LSH_NAME_LEN);
		s->lib[LSH_NAME_LEN] = '\0';
		s->layout = rsp[LSH_INFO_LAYOUT];
		s->devid = rsp[LSH_INFO_DEVID];
	}
	s->err = ret;
	spin_unlock_irqrestore(&h->rx_lock, flags);

	return ret;
}

static int lsh_query(struct lsh *h)
{
	char line[256];
	int i, n = 0, found = 0;

	if (!lsh_running(h))
		return -ENODEV;

	mutex_lock(&h->xfer_lock);
	for (i = 0; i < ARRAY_SIZE(h->sensors); i++) {
		struct lsh_sensor *s = &h->sensors[i];

		if (!lsh_query_one(h, s)) {
			found++;
			n += scnprintf(line + n, sizeof(line) - n, " %s=%s", s->label,
				       s->name);
		} else {
			n += scnprintf(line + n, sizeof(line) - n, " %s=-(%d)", s->label,
				       s->err);
		}
	}
	h->queries++;
	mutex_unlock(&h->xfer_lock);

	dev_info(h->scp->dev, "hub sensors:%s\n", line);

	return found ? 0 : -ENODEV;
}

/* AP boot time and arch counter read together, as the hub expects for its time base */
static void lsh_put_time(u8 *req)
{
	unsigned long flags;
	u64 now, cnt;

	local_irq_save(flags);
	now = ktime_get_boottime_ns();
	cnt = arch_timer_read_counter();
	local_irq_restore(flags);
	put_unaligned_le64(now, &req[LSH_CFG_AP_TIME]);
	put_unaligned_le64(cnt, &req[LSH_CFG_ARCH_COUNTER]);
}

static int lsh_send_timestamp(struct lsh *h)
{
	u8 req[LSH_PKT_SIZE] = { 0, LSH_ACT_SET_TIMESTAMP };
	u8 rsp[LSH_PKT_SIZE];
	int ret;

	mutex_lock(&h->xfer_lock);
	lsh_put_time(req);
	ret = lsh_request(h, req, rsp);
	h->syncs++;
	h->sync_err = ret;
	mutex_unlock(&h->xfer_lock);

	return ret;
}

static void lsh_sync_work(struct work_struct *work)
{
	struct lsh *h = container_of(to_delayed_work(work), struct lsh, sync_work);

	if (!READ_ONCE(h->hub_ready))
		return;
	lsh_send_timestamp(h);
	schedule_delayed_work(&h->sync_work, msecs_to_jiffies(LSH_SYNC_PERIOD_MS));
}

/* ---- sensor FIFO in the shared DRAM region ---- */

static void lsh_fifo_init(struct lsh *h)
{
	struct lsh_fifo *f = h->fifo;
	unsigned long flags;

	spin_lock_irqsave(&h->data_lock, flags);
	h->wp_pending = 0;
	WRITE_ONCE(f->rp, 0);
	WRITE_ONCE(f->wp, 0);
	WRITE_ONCE(f->size, h->fifo_size);
	WRITE_ONCE(f->reserved, 0);
	/* the header must be visible before its address goes to the SCP */
	wmb();
	spin_unlock_irqrestore(&h->data_lock, flags);
}

static int lsh_send_fifo(struct lsh *h)
{
	u8 req[LSH_PKT_SIZE] = { 0, LSH_ACT_SET_CONFIG };
	u8 rsp[LSH_PKT_SIZE];
	int ret;

	put_unaligned_le32(lower_32_bits(h->fifo_phys), &req[LSH_CFG_BUF_BASE]);
	put_unaligned_le32(h->fifo_size, &req[LSH_CFG_BUF_SIZE]);
	mutex_lock(&h->xfer_lock);
	lsh_put_time(req);
	ret = lsh_request(h, req, rsp);
	mutex_unlock(&h->xfer_lock);

	return ret;
}

static struct lsh_sensor *lsh_by_handle(struct lsh *h, u8 handle)
{
	int i;

	for (i = 0; i < LISBON_SCP_NUM_SENSORS; i++)
		if (h->sensors[i].handle == handle)
			return &h->sensors[i];
	return NULL;
}

static void lsh_fifo_event(struct lsh *h, const u8 *e)
{
	struct lsh_sensor *s = lsh_by_handle(h, e[LSH_EVT_TYPE]);
	u8 action = e[LSH_EVT_ACTION];
	int i;

	h->fifo_events++;
	if (!s) {
		h->fifo_unknown++;
		return;
	}
	if (action < LSH_EVT_ACTIONS)
		s->events[action]++;
	if (action == LSH_EVT_ACTION_CALI || action == LSH_EVT_ACTION_TEST) {
		for (i = 0; i < ARRAY_SIZE(s->cali); i++)
			s->cali[i] = get_unaligned_le32(&e[LSH_EVT_DATA + 4 * i]);
		return;
	}
	if (action != LSH_EVT_ACTION_DATA)
		return;

	for (i = 0; i < ARRAY_SIZE(s->sample.v); i++)
		s->sample.v[i] = get_unaligned_le32(&e[LSH_EVT_DATA + 4 * i]);
	s->sample.timestamp = get_unaligned_le64(&e[LSH_EVT_TIME]);
	s->sample.seq++;
	s->valid = true;
}

static bool lsh_fifo_pos_ok(struct lsh *h, u32 pos)
{
	return pos < h->fifo_size && !(pos % LSH_EVT_SIZE);
}

static void lsh_fifo_work(struct work_struct *work)
{
	struct lsh *h = container_of(work, struct lsh, fifo_work);
	struct lsh_fifo *f = h->fifo;
	u8 e[LSH_EVT_SIZE];
	unsigned long flags;
	u32 rp, wp;
	bool woke = false;

	if (!READ_ONCE(h->hub_ready))
		return;

	spin_lock_irqsave(&h->rx_lock, flags);
	wp = h->wp_pending;
	spin_unlock_irqrestore(&h->rx_lock, flags);

	spin_lock_irqsave(&h->data_lock, flags);
	if (!lsh_fifo_pos_ok(h, wp)) {
		/* fall back to the write pointer the hub keeps in the header */
		h->fifo_bad++;
		wp = READ_ONCE(f->wp);
		if (!lsh_fifo_pos_ok(h, wp))
			goto out;
	}
	rp = READ_ONCE(f->rp);
	if (!lsh_fifo_pos_ok(h, rp)) {
		h->fifo_bad++;
		rp = wp;
	}
	/* read the events before the read pointer goes back */
	rmb();
	while (rp != wp) {
		memcpy(e, &f->data[rp], LSH_EVT_SIZE);
		lsh_fifo_event(h, e);
		rp += LSH_EVT_SIZE;
		if (rp >= h->fifo_size)
			rp = 0;
		woke = true;
	}
	/* all events copied before the hub may overwrite them */
	mb();
	WRITE_ONCE(f->rp, wp);
out:
	spin_unlock_irqrestore(&h->data_lock, flags);

	if (woke)
		wake_up_all(&h->data_wq);
}

/* ---- CHRE comms: nanohub packets over IPI 24, "bytes taken" over IPI 25 ---- */

/* CRC-32 (MSB first, polynomial 0x04c11db7, start ~0) over little endian words */
static u32 lsh_comms_crc(const u8 *buf, size_t len)
{
	u32 crc = ~0U;
	u8 be[4];
	size_t i, k;

	for (i = 0; i < len; i += 4) {
		u32 word = 0;

		for (k = 0; k < 4 && i + k < len; k++)
			word |= (u32)buf[i + k] << (8 * k);
		put_unaligned_be32(word, be);
		crc = crc32_be(crc, be, sizeof(be));
	}

	return crc;
}

static void lsh_chre_ipi(void *data, unsigned int len, void *priv)
{
	struct lsh *h = priv;
	unsigned long flags;
	bool ack = false;

	spin_lock_irqsave(&h->rx_lock, flags);
	h->chre_rx++;
	if (h->chre_wait && !h->chre_len && len <= sizeof(h->chre_buf)) {
		memcpy(h->chre_buf, data, len);
		h->chre_len = len;
		complete(&h->chre_done);
	} else {
		/* nobody waits: the SCP still wants to know the bytes were taken */
		h->chre_unsolicited++;
		h->chre_ack_len = len;
		ack = true;
	}
	spin_unlock_irqrestore(&h->rx_lock, flags);

	if (ack)
		schedule_work(&h->chre_ack_work);
}

static void lsh_chre_ack(struct lsh *h, u32 len)
{
	u8 tok[4];

	put_unaligned_le32(len, tok);
	if (scp_ipi_send(h->scp, LSH_IPI_CHREX, tok, sizeof(tok), 0))
		h->chre_err++;
}

static void lsh_chre_ack_work(struct work_struct *work)
{
	struct lsh *h = container_of(work, struct lsh, chre_ack_work);

	lsh_chre_ack(h, READ_ONCE(h->chre_ack_len));
}

/* wait for one packet from the hub, take it and check it; Return: packet length */
static int lsh_chre_recv(struct lsh *h, u8 *buf, unsigned int timeout_ms)
{
	unsigned long flags;
	unsigned int len;
	u32 crc;

	wait_for_completion_timeout(&h->chre_done, msecs_to_jiffies(timeout_ms));

	spin_lock_irqsave(&h->rx_lock, flags);
	len = h->chre_len;
	memcpy(buf, h->chre_buf, len);
	h->chre_len = 0;
	reinit_completion(&h->chre_done);
	spin_unlock_irqrestore(&h->rx_lock, flags);

	if (!len)
		return -ETIMEDOUT;

	lsh_chre_ack(h, len);

	/* keep the last two packets for debugfs */
	memcpy(h->chre_last_pkt[0], h->chre_last_pkt[1], sizeof(h->chre_last_pkt[0]));
	h->chre_last_len[0] = h->chre_last_len[1];
	memcpy(h->chre_last_pkt[1], buf, min_t(unsigned int, len, sizeof(h->chre_last_pkt[1])));
	h->chre_last_len[1] = len;

	if (len < LSH_COMMS_HDR + LSH_COMMS_CRC || buf[0] != LSH_COMMS_SYNC ||
	    len != LSH_COMMS_HDR + buf[LSH_COMMS_HDR - 1] + LSH_COMMS_CRC)
		return -EBADMSG;
	crc = lsh_comms_crc(buf, len - LSH_COMMS_CRC);
	if (crc != get_unaligned_le32(&buf[len - LSH_COMMS_CRC]))
		return -EBADMSG;

	return len;
}

/*
 * One nanohub "write" (CMD_COMMS_WRITE): the hub answers with one byte (non-zero: taken),
 * either after an ACK packet or directly; like the vendor comms code, a packet with the
 * right sequence and a reason other than ACK/NACK/BUSY is the answer.
 * Return: 0 with *result set, or a negative error.
 */
static int lsh_chre_write(struct lsh *h, const void *data, u8 len, u8 *result)
{
	u8 pkt[LSH_COMMS_HDR + LSH_COMMS_MAX_DATA + LSH_COMMS_CRC];
	u8 rsp[sizeof(pkt)];
	unsigned int size = LSH_COMMS_HDR + len + LSH_COMMS_CRC;
	unsigned long flags;
	int try, ret = -EIO;
	u32 seq;

	mutex_lock(&h->chre_lock);
	seq = h->chre_seq++;
	pkt[0] = LSH_COMMS_SYNC;
	put_unaligned_le32(seq, &pkt[1]);
	put_unaligned_le32(LSH_COMMS_WRITE, &pkt[5]);
	pkt[9] = len;
	memcpy(&pkt[LSH_COMMS_HDR], data, len);
	put_unaligned_le32(lsh_comms_crc(pkt, LSH_COMMS_HDR + len), &pkt[LSH_COMMS_HDR + len]);

	for (try = 0; try < LSH_CHRE_TRIES; try++) {
		u32 reason;

		spin_lock_irqsave(&h->rx_lock, flags);
		h->chre_wait = true;
		h->chre_len = 0;
		reinit_completion(&h->chre_done);
		spin_unlock_irqrestore(&h->rx_lock, flags);

		h->chre_tx++;
		ret = scp_ipi_send(h->scp, LSH_IPI_CHRE, pkt, size, 0);
		if (ret)
			break;

		ret = lsh_chre_recv(h, rsp, LSH_CHRE_ACK_MS);
		if (ret < 0)
			continue;
		reason = get_unaligned_le32(&rsp[5]);
		if (get_unaligned_le32(&rsp[1]) != seq) {
			ret = -EPROTO;
			continue;
		}
		if (reason == LSH_COMMS_BUSY) {
			ret = -EBUSY;
			msleep(LSH_CHRE_BUSY_MS);
			continue;
		}
		if (reason == LSH_COMMS_NACK) {
			ret = -EAGAIN;
			continue;
		}

		if (reason == LSH_COMMS_ACK) {
			ret = lsh_chre_recv(h, rsp, LSH_CHRE_REPLY_MS);
			if (ret < 0)
				continue;
			if (get_unaligned_le32(&rsp[1]) != seq) {
				ret = -EPROTO;
				continue;
			}
		}
		*result = rsp[LSH_COMMS_HDR - 1] ? rsp[LSH_COMMS_HDR] : 0;
		ret = 0;
		break;
	}
	if (ret)
		h->chre_err++;
	h->chre_last = ret;

	spin_lock_irqsave(&h->rx_lock, flags);
	h->chre_wait = false;
	spin_unlock_irqrestore(&h->rx_lock, flags);
	mutex_unlock(&h->chre_lock);

	return ret;
}

/*
 * nanohub config command for one sensor: event, latency, rate (Hz * 1024), Android sensor
 * type, command, flags, then the config data of CFG_DATA as little endian words.
 * Return: answer byte of the hub (>= 0) or a negative error
 */
static int lsh_sensor_cmd(struct lsh *h, struct lsh_sensor *s, u8 command,
			  const s32 *data, unsigned int n)
{
	u8 cmd[LSH_CONFIG_SIZE + 4 * LSH_CFG_MAX] = {};
	u8 result = 0;
	unsigned int i;
	int ret;

	if (n > LSH_CFG_MAX)
		return -EINVAL;

	put_unaligned_le32(LSH_EVT_NO_SENSOR_CONFIG, &cmd[0]);
	put_unaligned_le64(0, &cmd[4]);			/* latency: report every sample */
	put_unaligned_le32(rates[s - h->sensors], &cmd[12]);
	cmd[16] = s->handle + 1;			/* Android sensor type */
	cmd[17] = command;
	for (i = 0; i < n; i++)
		put_unaligned_le32(data[i], &cmd[LSH_CONFIG_SIZE + 4 * i]);

	ret = lsh_chre_write(h, cmd, LSH_CONFIG_SIZE + 4 * n, &result);

	return ret ? ret : result;
}

/*
 * GET_DATA (sensor IPI action 2): the hub answers with its current data of the sensor
 * (vendor: data_unit_t after a 4 byte header; the layout of this firmware's answer is not
 * clear yet, debugfs shows the packet as is).
 */
static int lsh_get_data(struct lsh *h, struct lsh_sensor *s)
{
	u8 req[LSH_PKT_SIZE] = { s->handle, LSH_ACT_GET_DATA };
	u8 rsp[LSH_PKT_SIZE] = {};
	int ret;

	mutex_lock(&h->xfer_lock);
	ret = lsh_request(h, req, rsp);
	memcpy(h->get_rsp, rsp, sizeof(rsp));
	h->get_err = ret;
	h->gets++;
	mutex_unlock(&h->xfer_lock);

	return ret;
}

/* proximity mode of the hub driver, sent after enable and before disable like the HAL */
static void lsh_prox_mode(struct lsh *h, struct lsh_sensor *s, bool on)
{
	s32 cfg[2] = { LSH_PS_CFG_MODE, on };
	int ret;

	if (s != &h->sensors[LISBON_SCP_PROX] || !READ_ONCE(prox_mode))
		return;
	ret = lsh_sensor_cmd(h, s, LSH_CONFIG_CFG_DATA, cfg, ARRAY_SIZE(cfg));
	if (ret < 0)
		dev_warn_ratelimited(h->scp->dev, "prox mode %d failed: %d\n", on, ret);
}

/* nanohub enable/disable for one sensor */
static int lsh_sensor_config(struct lsh *h, struct lsh_sensor *s, bool on)
{
	int ret;

	lockdep_assert_held(&s->lock);

	if (!on)
		lsh_prox_mode(h, s, false);
	ret = lsh_sensor_cmd(h, s, on ? LSH_CONFIG_ENABLE : LSH_CONFIG_DISABLE, NULL, 0);
	s->config_result = ret;
	if (ret < 0) {
		dev_warn_ratelimited(h->scp->dev, "%s %s failed: %d\n",
				     on ? "enable" : "disable", s->label, ret);
		return ret;
	}
	if (on)
		lsh_prox_mode(h, s, true);

	s->enabled = on;
	s->gen = h->gen;
	if (!on) {
		unsigned long flags;

		spin_lock_irqsave(&h->data_lock, flags);
		s->valid = false;
		spin_unlock_irqrestore(&h->data_lock, flags);
	}
	dev_dbg(h->scp->dev, "%s %s (rate %u, answer %d)\n", s->label, on ? "on" : "off",
		rates[s - h->sensors], ret);

	return 0;
}

/* forget the enable state of an earlier SCP run */
static void lsh_sensor_sync_gen(struct lsh *h, struct lsh_sensor *s)
{
	lockdep_assert_held(&s->lock);
	if (s->gen != READ_ONCE(h->gen)) {
		s->enabled = false;
		s->manual = false;
		s->gen = READ_ONCE(h->gen);
	}
}

static void lsh_idle_work(struct work_struct *work)
{
	struct lsh_sensor *s = container_of(to_delayed_work(work), struct lsh_sensor,
					    idle_work);
	struct lsh *h = s->h;
	unsigned long due;

	mutex_lock(&s->lock);
	lsh_sensor_sync_gen(h, s);
	if (s->enabled && !s->manual) {
		due = s->last_read + msecs_to_jiffies(idle_ms);
		if (time_before(jiffies, due))
			mod_delayed_work(system_wq, &s->idle_work, due - jiffies);
		else if (READ_ONCE(h->hub_ready) && lsh_sensor_config(h, s, false))
			mod_delayed_work(system_wq, &s->idle_work, msecs_to_jiffies(idle_ms));
	}
	mutex_unlock(&s->lock);
}

/* ---- hub setup per SCP run ---- */

/* factory calibration in use: module parameters if set, else the firmware files */
static void lsh_ps_cali(struct lsh *h, int *out)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(ps_cali); i++)
		out[i] = READ_ONCE(ps_cali[i]);
	if (!out[0] && !out[1])
		memcpy(out, h->fw_ps_cali, sizeof(h->fw_ps_cali));
}

static int lsh_als_cali(struct lsh *h)
{
	int v = READ_ONCE(als_cali);

	return v > 0 ? v : h->fw_als_cali;
}

/*
 * Numbers of the array in a vendor calibration file ({"name": [n, ...]}).
 * Return: count of numbers read, or a negative error
 */
static int lsh_cali_parse(const u8 *p, size_t size, int *out, int n)
{
	const u8 *end = p + size;
	int i = 0;

	p = memchr(p, '[', size);
	if (!p)
		return -EINVAL;
	for (p++; p < end && *p != ']'; ) {
		bool neg = *p == '-';
		long v = 0;

		if (!neg && !isdigit(*p)) {
			p++;
			continue;
		}
		if (neg)
			p++;
		if (p >= end || !isdigit(*p) || i >= n)
			return -EINVAL;
		for (; p < end && isdigit(*p); p++) {
			v = v * 10 + (*p - '0');
			if (v > INT_MAX)
				return -ERANGE;
		}
		out[i++] = neg ? -v : v;
	}

	return p < end ? i : -EINVAL;
}

static void lsh_cali_load(struct lsh *h, const char *name, int *out, int n)
{
	const struct firmware *fw;
	int vals[4];
	int ret;

	ret = firmware_request_nowarn(&fw, name, h->dev);
	if (ret) {
		dev_info(h->dev, "no %s (%d), hub default\n", name, ret);
		return;
	}
	ret = lsh_cali_parse(fw->data, fw->size, vals, ARRAY_SIZE(vals));
	release_firmware(fw);
	if (ret != n) {
		dev_warn(h->dev, "%s: %d values instead of %d, not used\n", name, ret, n);
		return;
	}
	memcpy(out, vals, n * sizeof(*out));
}

/* factory calibration of light and proximity, after every setup of the hub (vendor: SCP ready) */
static void lsh_send_cali(struct lsh *h)
{
	struct lsh_sensor *ps = &h->sensors[LISBON_SCP_PROX];
	struct lsh_sensor *als = &h->sensors[LISBON_SCP_LIGHT];
	int als_scale = lsh_als_cali(h);
	s32 cfg[5];
	int ret = 0;

	lsh_ps_cali(h, &cfg[1]);
	if (!ps->err && (cfg[1] || cfg[2])) {
		cfg[0] = LSH_PS_CFG_FACTORY;
		mutex_lock(&ps->lock);
		ret = lsh_sensor_cmd(h, ps, LSH_CONFIG_CFG_DATA, cfg, ARRAY_SIZE(cfg));
		mutex_unlock(&ps->lock);
	}
	if (ret >= 0 && !als->err && als_scale > 0) {
		cfg[0] = LSH_ALS_CFG_SCALE;
		cfg[1] = als_scale;
		mutex_lock(&als->lock);
		ret = lsh_sensor_cmd(h, als, LSH_CONFIG_CFG_DATA, cfg, 2);
		mutex_unlock(&als->lock);
	}
	h->cali_err = ret < 0 ? ret : 0;
	if (ret < 0)
		dev_warn(h->scp->dev, "sending the light/proximity calibration failed: %d\n", ret);
}

/*
 * Crosstalk calibration of the proximity sensor, once per hub setup: the light the cover glass
 * reflects back alone gives a raw count of about 500 on lisbon, above the factory "near"
 * threshold (198), so without an offset the hub reports "near" all the time. The hub firmware
 * measures and sets the offset of the TMD2755 on request (config data type 3 = 0, then
 * CALIBRATE) and answers with a calibration event; the chip keeps the offset while it is
 * powered. Skipped while something covers the sensor (raw count, read with GET_DATA, nearer to
 * the factory "cover" value than to "uncover"), so a device that starts covered keeps the
 * offset it had.
 */
static void lsh_xtalk_work(struct work_struct *work)
{
	struct lsh *h = container_of(work, struct lsh, xtalk_work);
	struct lsh_sensor *s = &h->sensors[LISBON_SCP_PROX];
	s32 cfg[2] = { LSH_PS_CFG_CALI_TYPE, LSH_PS_CALI_XTALK };
	unsigned int gen = READ_ONCE(h->gen), before;
	int cali[ARRAY_SIZE(ps_cali)];
	bool was_on;
	s32 raw = 0;
	int ret;

	lsh_ps_cali(h, cali);

	mutex_lock(&s->lock);
	lsh_sensor_sync_gen(h, s);
	if (!READ_ONCE(h->hub_ready) || READ_ONCE(h->gen) != gen) {
		ret = -EAGAIN;
		goto out;
	}
	was_on = s->enabled;
	if (!was_on) {
		ret = lsh_sensor_config(h, s, true);
		if (ret)
			goto out;
	}
	msleep(LSH_XTALK_SETTLE_MS);

	ret = lsh_get_data(h, s);
	if (!ret && !get_unaligned_le32(&h->get_rsp[LSH_GET_PS_TYPE])) {
		raw = get_unaligned_le32(&h->get_rsp[LSH_GET_PS_RAW]);
		if (cali[2] > cali[3] && raw > (cali[2] + cali[3]) / 2) {
			ret = -EBUSY;
			goto off;
		}
	}

	before = READ_ONCE(s->events[LSH_EVT_ACTION_CALI]);
	ret = lsh_sensor_cmd(h, s, LSH_CONFIG_CFG_DATA, cfg, ARRAY_SIZE(cfg));
	if (ret >= 0)
		ret = lsh_sensor_cmd(h, s, LSH_CONFIG_CALIBRATE, NULL, 0);
	if (ret >= 0 && !wait_event_timeout(h->data_wq,
					    READ_ONCE(s->events[LSH_EVT_ACTION_CALI]) != before ||
					    !READ_ONCE(h->hub_ready),
					    msecs_to_jiffies(LSH_XTALK_WAIT_MS)))
		ret = -ETIMEDOUT;
	if (ret > 0)
		ret = 0;
	if (!ret && !READ_ONCE(h->hub_ready))
		ret = -EAGAIN;
off:
	if (!was_on && READ_ONCE(h->hub_ready))
		lsh_sensor_config(h, s, false);
out:
	h->xtalk_err = ret;
	h->xtalk_raw = raw;
	mutex_unlock(&s->lock);
	if (ret == -EBUSY)
		dev_info(h->scp->dev, "proximity covered (raw %d), no crosstalk calibration\n",
			 raw);
	else if (ret)
		dev_warn(h->scp->dev, "proximity crosstalk calibration failed: %d\n", ret);
	else
		dev_info(h->scp->dev, "proximity crosstalk calibrated (raw before %d)\n", raw);
}

static void lsh_setup_work(struct work_struct *work)
{
	struct lsh *h = container_of(work, struct lsh, setup_work);
	unsigned long flags;
	int i, t, ret;

	/* "init done" can come before remoteproc has finished attaching */
	for (t = 0; t < LSH_ATTACH_WAIT_MS && !lsh_running(h); t += 100)
		msleep(100);
	if (!lsh_running(h))
		return;

	lsh_query(h);

	if (!h->fifo) {
		h->setup_err = -ENOMEM;
		return;
	}
	lsh_fifo_init(h);
	ret = lsh_send_fifo(h);
	if (!ret)
		ret = lsh_send_timestamp(h);
	h->setup_err = ret;
	if (ret) {
		dev_err(h->scp->dev, "sensor hub setup failed: %d\n", ret);
		return;
	}

	spin_lock_irqsave(&h->data_lock, flags);
	for (i = 0; i < LISBON_SCP_NUM_SENSORS; i++)
		h->sensors[i].valid = false;
	spin_unlock_irqrestore(&h->data_lock, flags);

	lsh_send_cali(h);

	WRITE_ONCE(h->gen, h->gen + 1);
	WRITE_ONCE(h->hub_ready, true);
	wake_up_all(&h->data_wq);
	schedule_delayed_work(&h->sync_work, msecs_to_jiffies(LSH_SYNC_PERIOD_MS));
	dev_info(h->scp->dev, "sensor hub ready: fifo %pa+%#x, calibration %d\n", &h->fifo_phys,
		 h->fifo_size, h->cali_err);
	if (READ_ONCE(ps_xtalk) && !h->sensors[LISBON_SCP_PROX].err)
		schedule_work(&h->xtalk_work);

	/* the sensor list stays the same for later runs of the SCP */
	if (!h->iio) {
		struct auxiliary_device *adev;

		adev = auxiliary_device_create(h->dev, KBUILD_MODNAME, "iio", NULL, 0);
		if (adev)
			h->iio = adev;
		else
			dev_err(h->dev, "cannot add the IIO device\n");
	}
}

/* rproc subdevice: told when the SCP stops (detach, crash) */
static void lsh_subdev_stop(struct rproc_subdev *subdev, bool crashed)
{
	struct lsh *h = container_of(subdev, struct lsh, subdev);
	unsigned long flags;

	WRITE_ONCE(h->hub_ready, false);
	WRITE_ONCE(h->gen, h->gen + 1);
	spin_lock_irqsave(&h->rx_lock, flags);
	h->init_done = false;
	spin_unlock_irqrestore(&h->rx_lock, flags);
	wake_up_all(&h->data_wq);
	dev_info(h->scp->dev, "sensor hub stopped%s\n", crashed ? " (crash)" : "");
}

/* ---- interface for lisbon-scp-iio ---- */

struct device *lisbon_scp_hub_device(void)
{
	return lsh_dev ? lsh_dev->scp->dev : NULL;
}
EXPORT_SYMBOL_GPL(lisbon_scp_hub_device);

bool lisbon_scp_hub_present(enum lisbon_scp_sensor id)
{
	struct lsh *h = lsh_dev;
	unsigned long flags;
	bool ret;

	if (!h || id >= LISBON_SCP_NUM_SENSORS)
		return false;
	spin_lock_irqsave(&h->rx_lock, flags);
	ret = h->queries && !h->sensors[id].err;
	spin_unlock_irqrestore(&h->rx_lock, flags);

	return ret;
}
EXPORT_SYMBOL_GPL(lisbon_scp_hub_present);

/* name of the chip as the hub reports it; stable once the hub is set up */
const char *lisbon_scp_hub_chip(enum lisbon_scp_sensor id)
{
	if (!lisbon_scp_hub_present(id))
		return NULL;
	return lsh_dev->sensors[id].name;
}
EXPORT_SYMBOL_GPL(lisbon_scp_hub_chip);

/**
 * lisbon_scp_hub_read() - newest sample of a sensor
 * @id:	sensor
 * @out: sample
 *
 * Enables the sensor in the hub if it is off and waits up to first_ms for its first
 * sample. The sensor stays on until idle_ms after the last call. Sleeps.
 *
 * Return: 0, -EAGAIN if the hub is not set up or no sample came, other negative errors
 */
int lisbon_scp_hub_read(enum lisbon_scp_sensor id, struct lisbon_scp_sample *out)
{
	struct lsh *h = lsh_dev;
	struct lsh_sensor *s;
	unsigned long flags;
	long w;
	int ret = 0;

	if (!h || id >= LISBON_SCP_NUM_SENSORS)
		return -EINVAL;
	s = &h->sensors[id];

	mutex_lock(&s->lock);
	lsh_sensor_sync_gen(h, s);
	if (!READ_ONCE(h->hub_ready)) {
		ret = -EAGAIN;
		goto out;
	}
	s->last_read = jiffies;
	if (!s->enabled) {
		ret = lsh_sensor_config(h, s, true);
		if (ret)
			goto out;
	}
	mod_delayed_work(system_wq, &s->idle_work, msecs_to_jiffies(idle_ms));

	w = wait_event_interruptible_timeout(h->data_wq, READ_ONCE(s->valid) ||
					     !READ_ONCE(h->hub_ready),
					     msecs_to_jiffies(first_ms));
	if (w < 0) {
		ret = w;
		goto out;
	}

	spin_lock_irqsave(&h->data_lock, flags);
	if (s->valid)
		*out = s->sample;
	else
		ret = -EAGAIN;
	spin_unlock_irqrestore(&h->data_lock, flags);
out:
	mutex_unlock(&s->lock);

	return ret;
}
EXPORT_SYMBOL_GPL(lisbon_scp_hub_read);

/* ---- start and system suspend ---- */

/* attach takes up to 2 s (READY of the firmware): not in probe */
static void lsh_boot_work(struct work_struct *work)
{
	struct lsh *h = container_of(work, struct lsh, boot_work);
	int ret;

	ret = rproc_boot(h->rproc);
	h->boot_err = ret;
	if (ret) {
		dev_err(h->scp->dev, "SCP start failed: %d\n", ret);
		return;
	}
	h->booted = true;
}

static void lsh_resume_work(struct work_struct *work)
{
	struct lsh *h = container_of(work, struct lsh, resume_work);
	int ret;

	ret = rproc_boot(h->rproc);
	h->pm_err = ret;
	if (ret) {
		dev_err(h->scp->dev, "SCP restart after suspend failed: %d\n", ret);
		return;
	}
	h->booted = true;
	dev_info(h->scp->dev, "SCP attached again after suspend\n");
}

static int lsh_pm_notify(struct notifier_block *nb, unsigned long event, void *unused)
{
	struct lsh *h = container_of(nb, struct lsh, pm_nb);
	int ret;

	switch (event) {
	case PM_SUSPEND_PREPARE:
	case PM_HIBERNATION_PREPARE:
		/* a start still running would leave the SCP on during the suspend */
		flush_work(&h->boot_work);
		if (h->rproc->state != RPROC_ATTACHED)
			return NOTIFY_DONE;
		if (!suspend_detach) {
			dev_warn(h->scp->dev, "SCP running, suspend refused (detach it first)\n");
			return notifier_from_errno(-EBUSY);
		}
		/* without PMIC/SPM/DVFS setup the hub must not run while the AP sleeps */
		ret = rproc_detach(h->rproc);
		if (!ret && h->rproc->state == RPROC_ATTACHED)
			ret = -EBUSY;	/* someone else still holds the rproc */
		h->pm_err = ret;
		if (ret) {
			dev_err(h->scp->dev, "SCP detach before suspend failed: %d\n", ret);
			return notifier_from_errno(ret);
		}
		h->pm_detached = true;
		h->booted = false;
		dev_info(h->scp->dev, "SCP detached for suspend\n");
		return NOTIFY_OK;
	case PM_POST_SUSPEND:
	case PM_POST_HIBERNATION:
	case PM_POST_RESTORE:
		if (h->pm_detached) {
			h->pm_detached = false;
			schedule_work(&h->resume_work);
		}
		return NOTIFY_OK;
	}

	return NOTIFY_DONE;
}

/* ---- logger and diagnostics ---- */

static void lsh_logger_init_ipi(void *data, unsigned int len, void *priv)
{
	struct lsh *h = priv;
	struct lsh_log_info *li = &h->log_info;
	size_t sram = h->scp->sram_size;

	if (len < sizeof(*li))
		return;

	mutex_lock(&h->log_lock);
	memcpy(li, data, sizeof(*li));
	h->log_info_valid = li->buf_maxlen && li->buf_maxlen <= LSH_LOG_SRAM_MAX &&
			    li->buf_addr < sram && li->buf_maxlen <= sram - li->buf_addr &&
			    li->start_addr <= sram - 4 && li->end_addr <= sram - 4 &&
			    li->dram_addr <= sram - 4;
	/* the SCP is awake while it waits for this answer */
	if (h->log_info_valid && h->log_ctl)
		writel(h->log_phys, h->scp->sram_base + li->dram_addr);
	mutex_unlock(&h->log_lock);

	if (h->log_info_valid && h->log_ctl)
		schedule_work(&h->logger_work);
}

static void lsh_logger_work(struct work_struct *work)
{
	struct lsh *h = container_of(work, struct lsh, logger_work);
	u32 magic = LSH_LOG_INIT_MAGIC;

	h->logger_reply = scp_ipi_send(h->scp, LSH_IPI_LOGGER_INIT, &magic, sizeof(magic), 0);
	/* the vendor kernel enables the buffer even if the answer failed */
	WRITE_ONCE(h->log_ctl->enable, 1);
}

static void lsh_logger_wakeup_ipi(void *data, unsigned int len, void *priv)
{
	struct lsh *h = priv;

	h->logger_wakeups++;
}

static void lsh_pll_ipi(void *data, unsigned int len, void *priv)
{
	struct lsh *h = priv;

	h->pll_requests++;
	if (len >= 8) {
		h->pll_last[0] = get_unaligned_le32(data);
		h->pll_last[1] = get_unaligned_le32(data + 4);
	}
	dev_info_ratelimited(h->scp->dev, "SCP asks for PLL control %u/%u (not handled)\n",
			     h->pll_last[0], h->pll_last[1]);
}

static void lsh_error_ipi(void *data, unsigned int len, void *priv)
{
	struct lsh *h = priv;
	char ctx[33];

	h->err_infos++;
	if (len < 8)
		return;
	memset(ctx, 0, sizeof(ctx));
	if (len > 8)
		memcpy(ctx, data + 8, min_t(size_t, len - 8, sizeof(ctx) - 1));
	snprintf(h->last_err, sizeof(h->last_err), "case %u sensor %u: %s",
		 get_unaligned_le32(data), get_unaligned_le32(data + 4), ctx);
	dev_warn_ratelimited(h->scp->dev, "SCP error info: %s\n", h->last_err);
}

static const struct {
	u32 id;
	scp_ipi_handler_t handler;
} lsh_ipis[] = {
	{ LSH_IPI_LOGGER_WAKEUP, lsh_logger_wakeup_ipi },
	{ LSH_IPI_LOGGER_INIT, lsh_logger_init_ipi },
	{ LSH_IPI_SENSOR, lsh_sensor_ipi },
	{ LSH_IPI_CHRE, lsh_chre_ipi },
	{ LSH_IPI_PLL_CTRL, lsh_pll_ipi },
	{ LSH_IPI_ERROR_INFO, lsh_error_ipi },
};

static const char * const lsh_states[] = {
	[RPROC_OFFLINE] = "offline", [RPROC_SUSPENDED] = "suspended",
	[RPROC_RUNNING] = "running", [RPROC_CRASHED] = "crashed",
	[RPROC_ATTACHED] = "attached", [RPROC_DETACHED] = "detached",
};

static void lsh_show_regs(struct seq_file *m, struct lsh *h)
{
	struct mtk_scp *scp = h->scp;
	void __iomem *b = scp->cluster->reg_base;
	u32 cg = 0, rst = 0, irq = 0;
	int i;

	regmap_read(scp->infracfg, LSH_INFRA_CG0_STA, &cg);
	regmap_read(scp->infracfg, LSH_INFRA_RST_STA, &rst);
	regmap_read(scp->infracfg, LSH_INFRA_IRQ_SET, &irq);
	seq_printf(m, "infra: cg0_sta=%#010x (scp clock %s) module_reset_sta=%#010x irq_set=%#010x\n",
		   cg, cg & LSH_INFRA_CG_SCP ? "GATED" : "on", rst, irq);
	/* reading the SCP registers with the bus clock gated would hang the AP */
	if (cg & LSH_INFRA_CG_SCP)
		return;

	seq_printf(m, "cfg: sw_rstn=%#x to_host=%#x host_to_scp=%#x to_spm=%#x wdt=%#x sleep=%#x\n",
		   readl(b + MT8183_SW_RSTN), readl(b + MT8183_SCP_TO_HOST),
		   readl(b + MT8183_HOST_TO_SCP), readl(b + MT6785_SCP_TO_SPM),
		   readl(b + MT8183_WDT_CFG), readl(b + MT6785_SCP_SLEEP_STATUS));
	seq_puts(m, "gpr:");
	for (i = 0; i < 8; i++)
		seq_printf(m, " %#x", readl(b + MT6785_SCP_GPR(i)));
	seq_printf(m, "\ndebug: sp=%#x lr=%#x psp=%#x pc=%#x\n",
		   readl(b + 0xa8), readl(b + 0xac), readl(b + 0xb0), readl(b + 0xb4));
	seq_printf(m, "remap: %#010x %#010x %#010x\n",
		   readl(b + 0x120), readl(b + 0x124), readl(b + 0x128));
	seq_printf(m, "clkctrl: sw_sel=%#x enable=%#x sram_pdn=%#x l1_sram_pd=%#x tcm_tail_pd=%#x\n",
		   readl(b + LSH_CLK_SW_SEL), readl(b + LSH_CLK_ENABLE),
		   readl(b + LSH_CLK_SRAM_PDN), readl(b + LSH_CLK_L1_SRAM_PD),
		   readl(b + LSH_CLK_TCM_TAIL_PD));
}

static int lsh_status_show(struct seq_file *m, void *v)
{
	struct lsh *h = m->private;
	struct mtk_scp *scp = h->scp;
	struct mtk_scp_region_info *ri = &scp->region_info;
	unsigned int st = h->rproc->state;
	unsigned long flags;
	int i;

	seq_printf(m, "rproc: %s state=%s running=%d ready_info=%#x awake=%u awake_fail=%u recovery=%s\n",
		   h->rproc->name, st < ARRAY_SIZE(lsh_states) && lsh_states[st] ?
		   lsh_states[st] : "?", scp->running, scp->ready_info, scp->awake_count,
		   scp->awake_fail, h->rproc->recovery_disabled ? "disabled" : "enabled");
	seq_printf(m, "clk: %lu Hz\n", clk_get_rate(scp->clk));
	lsh_show_regs(m, h);
	seq_printf(m, "region: %s loader=%#x+%#x fw=%#x+%#x dram=%#x+%#x backup=%#x l1c=%#x/%#x\n",
		   ri->platform, ri->ap_loader_start, ri->ap_loader_size,
		   ri->ap_firmware_start, ri->ap_firmware_size, ri->ap_dram_start,
		   ri->ap_dram_size, ri->ap_dram_backup_start, ri->il1c_con, ri->dl1c_con);
	seq_printf(m, "share: phys=%pad size=%#zx\n", &scp->dma_addr,
		   scp->cpu_addr ? scp->data->scp_sizes->max_dram_size : 0);
	seq_printf(m, "ipi: rx=%u rx_unhandled=%u last_rx=%u tx=%u tx_timeout=%u last_tx=%u wdt=%u\n",
		   scp->ipi_rx_count, scp->ipi_rx_unhandled, scp->ipi_last_rx_id,
		   scp->ipi_tx_count, scp->ipi_tx_timeout, scp->ipi_last_tx_id, scp->wdt_count);

	spin_lock_irqsave(&h->rx_lock, flags);
	seq_printf(m, "hub: rx=%u unmatched=%u init_done=%d notify(init/fifo/notify/batch/push)=%u/%u/%u/%u/%u queries=%u\n",
		   h->rx_count, h->rx_unmatched, h->init_done, h->notify[0], h->notify[1],
		   h->notify[2], h->notify[3], h->notify[4], h->queries);
	for (i = 0; i < ARRAY_SIZE(h->sensors); i++) {
		struct lsh_sensor *s = &h->sensors[i];

		if (s->err)
			seq_printf(m, "  %-5s: error %d\n", s->label, s->err);
		else if (s->name[0])
			seq_printf(m, "  %-5s: %s lib=%s layout=%d devid=%d\n", s->label,
				   s->name, s->lib, s->layout, s->devid);
	}
	if (h->queries)
		seq_printf(m, "last response: %*ph\n", LSH_PKT_SIZE, h->rx);
	spin_unlock_irqrestore(&h->rx_lock, flags);

	seq_printf(m, "pll requests=%u last=%u/%u, logger wakeups=%u, error infos=%u %s\n",
		   h->pll_requests, h->pll_last[0], h->pll_last[1], h->logger_wakeups,
		   h->err_infos, h->last_err);
	if (h->log_info_valid)
		seq_printf(m, "log: sram buf=%#x len=%u start@%#x end@%#x dram@%#x reply=%d\n",
			   h->log_info.buf_addr, h->log_info.buf_maxlen, h->log_info.start_addr,
			   h->log_info.end_addr, h->log_info.dram_addr, h->logger_reply);
	if (h->log_ctl)
		seq_printf(m, "dramlog: phys=%pa enable=%u r_pos=%u w_pos=%u\n", &h->log_phys,
			   READ_ONCE(h->log_ctl->enable), READ_ONCE(h->log_ctl->r_pos),
			   READ_ONCE(h->log_ctl->w_pos));

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(lsh_status);

static int lsh_sensors_show(struct seq_file *m, void *v)
{
	struct lsh *h = m->private;
	unsigned long flags;
	int i;

	spin_lock_irqsave(&h->rx_lock, flags);
	for (i = 0; i < ARRAY_SIZE(h->sensors); i++) {
		struct lsh_sensor *s = &h->sensors[i];

		if (!h->queries)
			seq_printf(m, "%s -\n", s->label);
		else if (s->err)
			seq_printf(m, "%s - %d\n", s->label, s->err);
		else
			seq_printf(m, "%s %s\n", s->label, s->name);
	}
	spin_unlock_irqrestore(&h->rx_lock, flags);

	return 0;
}

static ssize_t lsh_sensors_write(struct file *file, const char __user *ubuf, size_t count,
				 loff_t *ppos)
{
	struct lsh *h = ((struct seq_file *)file->private_data)->private;
	char buf[8];
	int ret;

	if (count >= sizeof(buf))
		return -EINVAL;
	if (copy_from_user(buf, ubuf, count))
		return -EFAULT;
	buf[count] = '\0';
	if (!sysfs_streq(buf, "query"))
		return -EINVAL;

	ret = lsh_query(h);

	return ret ? ret : count;
}
DEFINE_SHOW_STORE_ATTRIBUTE(lsh_sensors);

/* hub state, FIFO, CHRE comms and the newest sample of each sensor */
static int lsh_hub_show(struct seq_file *m, void *v)
{
	struct lsh *h = m->private;
	int cali[ARRAY_SIZE(ps_cali)];
	unsigned long flags;
	int i;

	seq_printf(m, "hub: ready=%d gen=%u setup=%d syncs=%u sync=%d pm_detached=%d pm=%d\n",
		   READ_ONCE(h->hub_ready), READ_ONCE(h->gen), h->setup_err, h->syncs,
		   h->sync_err, h->pm_detached, h->pm_err);
	spin_lock_irqsave(&h->rx_lock, flags);
	seq_printf(m, "chre: tx=%u rx=%u err=%u unsolicited=%u last=%d seq=%u\n",
		   h->chre_tx, h->chre_rx, h->chre_err, h->chre_unsolicited, h->chre_last,
		   h->chre_seq);
	seq_printf(m, "last notify: %*ph\n", 32, h->last_notify);
	spin_unlock_irqrestore(&h->rx_lock, flags);
	for (i = 0; i < ARRAY_SIZE(h->chre_last_pkt); i++) {
		unsigned int n = min_t(unsigned int, h->chre_last_len[i],
				       sizeof(h->chre_last_pkt[i]));

		seq_printf(m, "chre rx[%d] len=%u: %*ph\n", i, h->chre_last_len[i], n,
			   h->chre_last_pkt[i]);
	}

	spin_lock_irqsave(&h->data_lock, flags);
	if (h->fifo)
		seq_printf(m, "fifo: phys=%pa size=%#x rp=%#x wp=%#x pending=%#x events=%u unknown=%u bad=%u\n",
			   &h->fifo_phys, READ_ONCE(h->fifo->size), READ_ONCE(h->fifo->rp),
			   READ_ONCE(h->fifo->wp), h->wp_pending, h->fifo_events,
			   h->fifo_unknown, h->fifo_bad);
	for (i = 0; i < LISBON_SCP_NUM_SENSORS; i++) {
		struct lsh_sensor *s = &h->sensors[i];
		struct lisbon_scp_sample *p = &s->sample;

		seq_printf(m, "%-5s on=%d manual=%d rate=%u answer=%d events(data/flush/bias/cali/temp/test)=%u/%u/%u/%u/%u/%u",
			   s->label, s->enabled, s->manual, rates[i], s->config_result,
			   s->events[0], s->events[1], s->events[2], s->events[3],
			   s->events[4], s->events[5]);
		if (s->valid)
			seq_printf(m, " seq=%u t=%llu v=%d %d %d %d %d %d %d %d",
				   p->seq, p->timestamp, p->v[0], p->v[1], p->v[2], p->v[3],
				   p->v[4], p->v[5], p->v[6], p->v[7]);
		if (s->events[LSH_EVT_ACTION_CALI] || s->events[LSH_EVT_ACTION_TEST])
			seq_printf(m, " cali=%d %d %d %d %d %d", s->cali[0], s->cali[1],
				   s->cali[2], s->cali[3], s->cali[4], s->cali[5]);
		seq_putc(m, '\n');
	}
	spin_unlock_irqrestore(&h->data_lock, flags);
	lsh_ps_cali(h, cali);
	seq_printf(m, "calibration: ps_cali=%d,%d,%d,%d als_cali=%d (file %d,%d,%d,%d %d) prox_mode=%d sent=%d\n",
		   cali[0], cali[1], cali[2], cali[3], lsh_als_cali(h),
		   h->fw_ps_cali[0], h->fw_ps_cali[1], h->fw_ps_cali[2], h->fw_ps_cali[3],
		   h->fw_als_cali, READ_ONCE(prox_mode), h->cali_err);
	seq_printf(m, "start: autostart=%d booted=%d result=%d iio=%s\n", autostart, h->booted,
		   h->boot_err, h->iio ? dev_name(&h->iio->dev) : "-");
	seq_printf(m, "crosstalk: ps_xtalk=%d result=%d raw_before=%d\n", READ_ONCE(ps_xtalk),
		   h->xtalk_err, h->xtalk_raw);
	if (h->gets)
		seq_printf(m, "get: n=%u err=%d %*ph\n", h->gets, h->get_err, LSH_PKT_SIZE,
			   h->get_rsp);

	return 0;
}

static ssize_t lsh_hub_write(struct file *file, const char __user *ubuf, size_t count,
			     loff_t *ppos)
{
	struct lsh *h = ((struct seq_file *)file->private_data)->private;
	enum { LSH_W_ON, LSH_W_OFF, LSH_W_CFG, LSH_W_CALI, LSH_W_GET } op;
	struct lsh_sensor *s = NULL;
	char buf[128], *cmd, *name, *tok;
	s32 cfg[LSH_CFG_MAX];
	unsigned int n = 0;
	int i, ret;

	if (count >= sizeof(buf))
		return -EINVAL;
	if (copy_from_user(buf, ubuf, count))
		return -EFAULT;
	buf[count] = '\0';
	cmd = strim(buf);

	if (!strcmp(cmd, "sync")) {
		ret = READ_ONCE(h->hub_ready) ? lsh_send_timestamp(h) : -EAGAIN;
		return ret ? ret : count;
	}
	tok = strsep(&cmd, " ");
	if (!strcmp(tok, "on"))
		op = LSH_W_ON;
	else if (!strcmp(tok, "off"))
		op = LSH_W_OFF;
	else if (!strcmp(tok, "cfg"))
		op = LSH_W_CFG;
	else if (!strcmp(tok, "cali"))
		op = LSH_W_CALI;
	else if (!strcmp(tok, "get"))
		op = LSH_W_GET;
	else
		return -EINVAL;
	if (!cmd)
		return -EINVAL;
	cmd = skip_spaces(cmd);
	name = strsep(&cmd, " ");
	for (i = 0; i < LISBON_SCP_NUM_SENSORS; i++)
		if (!strcmp(name, h->sensors[i].label))
			s = &h->sensors[i];
	if (!s)
		return -EINVAL;
	while (op == LSH_W_CFG && cmd && *(cmd = skip_spaces(cmd))) {
		tok = strsep(&cmd, " ");
		if (n >= ARRAY_SIZE(cfg) || kstrtos32(tok, 0, &cfg[n]))
			return -EINVAL;
		n++;
	}
	if (op == LSH_W_CFG && !n)
		return -EINVAL;

	mutex_lock(&s->lock);
	lsh_sensor_sync_gen(h, s);
	if (!READ_ONCE(h->hub_ready))
		ret = -EAGAIN;
	else if (op == LSH_W_CFG)
		ret = lsh_sensor_cmd(h, s, LSH_CONFIG_CFG_DATA, cfg, n);
	else if (op == LSH_W_CALI)
		ret = lsh_sensor_cmd(h, s, LSH_CONFIG_CALIBRATE, NULL, 0);
	else if (op == LSH_W_GET)
		ret = lsh_get_data(h, s);
	else
		ret = lsh_sensor_config(h, s, op == LSH_W_ON);
	if (ret >= 0 && (op == LSH_W_ON || op == LSH_W_OFF))
		s->manual = op == LSH_W_ON;
	mutex_unlock(&s->lock);

	return ret < 0 ? ret : count;
}
DEFINE_SHOW_STORE_ATTRIBUTE(lsh_hub);

/* registers of the SCP interrupt block (EINT), only while the SCP runs with its bus clock on */
static int lsh_eint_show(struct seq_file *m, void *v)
{
	struct lsh *h = m->private;
	u32 cg = 0, off;

	regmap_read(h->scp->infracfg, LSH_INFRA_CG0_STA, &cg);
	if (!h->eint || (cg & LSH_INFRA_CG_SCP) || !lsh_running(h)) {
		seq_puts(m, "(SCP not running)\n");
		return 0;
	}
	for (off = 0; off < LSH_EINT_SHOW_END; off += 0x40)
		seq_printf(m, "%#05x: %#010x\n", off, readl(h->eint + off));

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(lsh_eint);

/* SRAM ring of the SCP log, from start to end index */
static int lsh_log_show(struct seq_file *m, void *v)
{
	struct lsh *h = m->private;
	struct mtk_scp *scp = h->scp;
	struct lsh_log_info li;
	u32 start, end, n, i;
	bool awake = false;
	char *buf;

	mutex_lock(&h->log_lock);
	li = h->log_info;
	if (!h->log_info_valid || !lsh_running(h)) {
		mutex_unlock(&h->log_lock);
		seq_puts(m, "(no SCP log: SCP not running or no logger info yet)\n");
		return 0;
	}

	buf = kmalloc(li.buf_maxlen + 1, GFP_KERNEL);
	if (!buf) {
		mutex_unlock(&h->log_lock);
		return -ENOMEM;
	}
	awake = !scp_awake_get(scp);
	start = readl(scp->sram_base + li.start_addr);
	end = readl(scp->sram_base + li.end_addr);
	if (start >= li.buf_maxlen || end >= li.buf_maxlen) {
		start = 0;
		end = 0;
	}
	for (n = 0, i = start; i != end; i = (i + 1) % li.buf_maxlen)
		buf[n++] = readb(scp->sram_base + li.buf_addr + i);
	if (awake)
		scp_awake_put(scp);
	mutex_unlock(&h->log_lock);

	buf[n] = '\0';
	seq_printf(m, "%s\n", buf);
	kfree(buf);

	return 0;
}
DEFINE_SHOW_ATTRIBUTE(lsh_log);

/* DRAM buffer of the SCP logger: everything since the last read */
static ssize_t lsh_dramlog_read(struct file *file, char __user *ubuf, size_t count,
				loff_t *ppos)
{
	struct lsh *h = file->private_data;
	struct lsh_log_ctl *c = h->log_ctl;
	const char *data = (const char *)c + LSH_LOG_BUF_OFS;
	u32 r, w, n;
	ssize_t ret;

	mutex_lock(&h->log_lock);
	r = READ_ONCE(c->r_pos);
	w = READ_ONCE(c->w_pos);
	if (r >= LSH_LOG_BUF_SIZE || w >= LSH_LOG_BUF_SIZE) {
		ret = -EIO;
		goto out;
	}
	/* the producer wraps at the end of the buffer */
	n = w >= r ? w - r : LSH_LOG_BUF_SIZE - r;
	n = min_t(size_t, n, count);
	ret = n;
	if (n && copy_to_user(ubuf, data + r, n))
		ret = -EFAULT;
	else
		WRITE_ONCE(c->r_pos, (r + n) % LSH_LOG_BUF_SIZE);
out:
	mutex_unlock(&h->log_lock);

	return ret;
}

static const struct file_operations lsh_dramlog_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.read = lsh_dramlog_read,
};

static void lsh_dramlog_init(struct lsh *h)
{
	struct mtk_scp *scp = h->scp;
	struct lsh_log_ctl *c;

	if (!dramlog || !scp->cpu_addr || scp->data->scp_sizes->max_dram_size <
	    LSH_SHARE_LOGGER_OFFSET + LSH_SHARE_LOGGER_SIZE)
		return;

	c = scp->cpu_addr + LSH_SHARE_LOGGER_OFFSET;
	c->base = LSH_LOG_MAGIC;
	c->size = LSH_LOG_CTL_SIZE;
	c->enable = 0;
	c->info_ofs = LSH_LOG_CTL_SIZE;
	c->buff_ofs = LSH_LOG_BUF_OFS;
	c->buff_size = LSH_LOG_BUF_SIZE;
	c->r_pos = 0;
	c->w_pos = 0;
	/* make the block visible before its address goes to the SCP */
	wmb();
	h->log_ctl = c;
	h->log_phys = scp->dma_addr + LSH_SHARE_LOGGER_OFFSET;
}

static void lsh_fifo_setup(struct lsh *h)
{
	struct mtk_scp *scp = h->scp;

	if (!scp->cpu_addr || scp->data->scp_sizes->max_dram_size <
	    LSH_SHARE_SENSOR_OFFSET + LSH_SHARE_SENSOR_SIZE)
		return;

	h->fifo = scp->cpu_addr + LSH_SHARE_SENSOR_OFFSET;
	h->fifo_phys = scp->dma_addr + LSH_SHARE_SENSOR_OFFSET;
	/* whole events only, as the vendor kernel sizes it */
	h->fifo_size = (LSH_SHARE_SENSOR_SIZE - LSH_FIFO_HDR) / LSH_EVT_SIZE * LSH_EVT_SIZE;
}

static void lsh_cancel_works(struct lsh *h)
{
	int i;

	cancel_work_sync(&h->setup_work);
	cancel_delayed_work_sync(&h->sync_work);
	cancel_work_sync(&h->fifo_work);
	cancel_work_sync(&h->chre_ack_work);
	cancel_work_sync(&h->resume_work);
	cancel_work_sync(&h->logger_work);
	cancel_work_sync(&h->xtalk_work);
	for (i = 0; i < LISBON_SCP_NUM_SENSORS; i++)
		cancel_delayed_work_sync(&h->sensors[i].idle_work);
}

static int lsh_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mtk_scp *scp;
	struct lsh *h;
	int i, ret;

	/* the rproc of mtk_scp exists once mtk_scp is bound to the parent */
	if (!dev->parent || !device_is_bound(dev->parent))
		return -EPROBE_DEFER;
	scp = dev_get_drvdata(dev->parent);
	if (!scp || !scp->rproc || !scp->infracfg)
		return dev_err_probe(dev, -ENODEV, "mtk_scp is not bound to the SCP\n");
	if (lsh_dev)
		return -EBUSY;

	h = kzalloc_obj(*h);
	if (!h)
		return -ENOMEM;
	h->scp = scp;
	h->rproc = scp_get_rproc(scp);
	h->dev = dev;
	mutex_init(&h->xfer_lock);
	mutex_init(&h->log_lock);
	mutex_init(&h->chre_lock);
	spin_lock_init(&h->rx_lock);
	spin_lock_init(&h->data_lock);
	init_completion(&h->rx_done);
	init_completion(&h->chre_done);
	init_waitqueue_head(&h->data_wq);
	INIT_WORK(&h->setup_work, lsh_setup_work);
	INIT_DELAYED_WORK(&h->sync_work, lsh_sync_work);
	INIT_WORK(&h->fifo_work, lsh_fifo_work);
	INIT_WORK(&h->chre_ack_work, lsh_chre_ack_work);
	INIT_WORK(&h->boot_work, lsh_boot_work);
	INIT_WORK(&h->resume_work, lsh_resume_work);
	INIT_WORK(&h->logger_work, lsh_logger_work);
	INIT_WORK(&h->xtalk_work, lsh_xtalk_work);
	h->chre_seq = 1;
	for (i = 0; i < ARRAY_SIZE(h->sensors); i++) {
		struct lsh_sensor *s = &h->sensors[i];

		s->h = h;
		s->handle = lsh_types[i].handle;
		s->label = lsh_types[i].label;
		mutex_init(&s->lock);
		INIT_DELAYED_WORK(&s->idle_work, lsh_idle_work);
	}

	lsh_cali_load(h, LSH_PS_CALI_FW, h->fw_ps_cali, ARRAY_SIZE(h->fw_ps_cali));
	lsh_cali_load(h, LSH_ALS_CALI_FW, &h->fw_als_cali, 1);

	if (h->rproc->state == RPROC_ATTACHED)
		dev_warn(scp->dev, "SCP already running: logger info and init done are missed\n");
	else
		lsh_dramlog_init(h);
	lsh_fifo_setup(h);
	h->eint = ioremap(LSH_EINT_BASE, LSH_EINT_SIZE);

	for (i = 0; i < ARRAY_SIZE(lsh_ipis); i++) {
		ret = scp_ipi_register(scp, lsh_ipis[i].id, lsh_ipis[i].handler, h);
		if (ret)
			goto unregister;
	}

	h->subdev.stop = lsh_subdev_stop;
	rproc_add_subdev(h->rproc, &h->subdev);

	h->pm_nb.notifier_call = lsh_pm_notify;
	ret = register_pm_notifier(&h->pm_nb);
	if (ret)
		goto remove_subdev;

	h->dir = debugfs_create_dir("lisbon-scp", NULL);
	debugfs_create_file("status", 0444, h->dir, h, &lsh_status_fops);
	debugfs_create_file("sensors", 0644, h->dir, h, &lsh_sensors_fops);
	debugfs_create_file("hub", 0644, h->dir, h, &lsh_hub_fops);
	debugfs_create_file("log", 0444, h->dir, h, &lsh_log_fops);
	if (h->eint)
		debugfs_create_file("eint", 0400, h->dir, h, &lsh_eint_fops);
	if (h->log_ctl)
		debugfs_create_file("dramlog", 0400, h->dir, h, &lsh_dramlog_fops);

	lsh_dev = h;
	platform_set_drvdata(pdev, h);
	dev_info(scp->dev, "sensor hub ready for start (rproc %s, dram log %s, fifo %s, autostart %d)\n",
		 dev_name(&h->rproc->dev), h->log_ctl ? "on" : "off", h->fifo ? "on" : "off",
		 autostart);
	if (autostart)
		schedule_work(&h->boot_work);

	return 0;

remove_subdev:
	rproc_remove_subdev(h->rproc, &h->subdev);
	i = ARRAY_SIZE(lsh_ipis);
unregister:
	while (--i >= 0)
		scp_ipi_unregister(scp, lsh_ipis[i].id);
	lsh_cancel_works(h);
	if (h->eint)
		iounmap(h->eint);
	kfree(h);
	return ret;
}

static void lsh_remove(struct platform_device *pdev)
{
	struct lsh *h = platform_get_drvdata(pdev);
	int i;

	/* no start or suspend handling from here on */
	cancel_work_sync(&h->boot_work);
	unregister_pm_notifier(&h->pm_nb);
	cancel_work_sync(&h->resume_work);
	/* detach, not stop: a stopped SCP would need a firmware file for the next start */
	if (h->booted && h->rproc->state == RPROC_ATTACHED)
		rproc_detach(h->rproc);
	h->booted = false;
	cancel_work_sync(&h->setup_work);
	/* IIO readers go before the hub state they read */
	if (h->iio)
		auxiliary_device_destroy(h->iio);
	/* waits for readers and writers of the debugfs files */
	debugfs_remove_recursive(h->dir);
	rproc_remove_subdev(h->rproc, &h->subdev);
	WRITE_ONCE(h->hub_ready, false);
	for (i = 0; i < ARRAY_SIZE(lsh_ipis); i++)
		scp_ipi_unregister(h->scp, lsh_ipis[i].id);
	lsh_cancel_works(h);
	lsh_dev = NULL;
	if (h->eint)
		iounmap(h->eint);
	for (i = 0; i < ARRAY_SIZE(h->sensors); i++)
		mutex_destroy(&h->sensors[i].lock);
	mutex_destroy(&h->chre_lock);
	mutex_destroy(&h->log_lock);
	mutex_destroy(&h->xfer_lock);
	kfree(h);
}

static const struct of_device_id lsh_of_match[] = {
	{ .compatible = "mediatek,mt6785-scp-sensor-hub" },
	{}
};
MODULE_DEVICE_TABLE(of, lsh_of_match);

static struct platform_driver lsh_driver = {
	.probe = lsh_probe,
	.remove = lsh_remove,
	.driver = {
		.name = "lisbon-scp-hub",
		.of_match_table = lsh_of_match,
	},
};
module_platform_driver(lsh_driver);

MODULE_DESCRIPTION("lisbon: SCP sensor hub protocol and diagnostics (MT6785)");
MODULE_LICENSE("GPL");
