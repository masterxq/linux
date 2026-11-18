/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * MediaTek CONNAC1 consys (MT6785 "soc1_0"): shared definitions of the power-on part
 * (mtk-connac1-consys.c) and the BTIF/STP/WMT part (mtk-connac1-wmt.c).
 * Function users: WLAN (gen4m), Bluetooth (btmtkstp), GNSS (mtk-connac1-gps).
 */
#ifndef __MTK_CONNAC1_H
#define __MTK_CONNAC1_H

#include <linux/clk.h>
#include <linux/firmware.h>
#include <linux/ktime.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>
#include <linux/unaligned.h>
#include <linux/wait.h>
#include <linux/workqueue.h>

struct dentry;
struct pinctrl;
struct pinctrl_state;
struct regmap;
struct regulator;
struct reset_control;
struct seq_file;

#define CONSYS_DUMP_REGS	12
#define PATCH_BUILDTIME_LEN	16
#define CONSYS_RX_RING		16384	/* power of two; a full STP window of BT frames fits */
#define CONSYS_WMT_STEPS	48
#define CONSYS_TRC_N		512	/* BTIF trace records, power of two */
#define CONSYS_TRC_DATA		24	/* bytes kept per trace record */
#define CONSYS_BTIF_REGS	28	/* BTIF registers 0x00..0x6c */
#define CONSYS_PC_SAMPLES	8
#define CONSYS_STP_SEQS		8	/* STP sequence numbers 0..7 */
/*
 * largest own STP payload: a WMT command, an HCI packet with its H4 type byte or one write of
 * the GNSS engine (one STP frame per write like the vendor stp_chrdev_gps.c; the engine writes
 * up to about 1.4 KiB at once)
 */
#define CONSYS_STP_PAYLOAD	2048
#define CONSYS_STP_FRAME	(4 + CONSYS_STP_PAYLOAD + 2)	/* header, payload, CRC */
#define CONSYS_STP_RXMAX	4096	/* the STP length field has 12 bits */

/* one BTIF trace record: a TX write, an RX interrupt batch, a polled RX batch or a marker */
struct consys_trc {
	s64 ns;				/* since the start of the WMT sequence */
	const char *name;		/* marker text (dir 'M') */
	u16 len;			/* bytes in this transfer (data[] keeps the first ones) */
	u8 dir;				/* 'T' PIO TX, 'D' DMA TX, 'R' RX irq, 'P' RX polled, 'M' marker */
	u8 lsr;				/* LSR seen with this record (RX) */
	u8 data[CONSYS_TRC_DATA];
};

struct consys_wmt_step {
	const char *name;
	s64 us;
	u32 tx, rx, irqs;
};

struct connac1_bt_ops;
struct connac1_gps_ops;

struct consys_wmt {
	void __iomem *btif;
	struct clk *btif_clk;
	int irq;
	bool clk_on, irq_on, irq_disabled, ready;

	/* optional AP DMA TX channel of the BTIF (vendor default) */
	void __iomem *txdma;
	struct clk *dma_clk;
	u8 *vff;
	dma_addr_t vff_dma;
	bool dma_on, dma_clk_on;
	u32 vff_wpt;			/* write pointer incl. wrap bit */
	int txdma_irq;			/* known, not requested: TX is drained by polling */

	/* optional AP DMA RX channel of the BTIF (vendor default) */
	void __iomem *rxdma;
	u8 *rx_vff;
	dma_addr_t rx_vff_dma;
	int rxdma_irq;
	bool rxdma_on, rxdma_irq_on;
	u32 rxdma_irqs, rxdma_spins;

	spinlock_t rx_lock;		/* RX ring between IRQ and reader */
	wait_queue_head_t rx_wq;
	u8 rx_ring[CONSYS_RX_RING];
	unsigned int rx_head, rx_tail;

	spinlock_t trc_lock;
	struct consys_trc *trc;
	unsigned int trc_head, trc_count;

	/* STP state; stp_lock serialises command/event exchanges and the idle RX handler */
	struct mutex stp_lock;
	struct work_struct rx_work;
	bool async;			/* start-up done: frames outside a command go to rx_work */
	bool asleep;			/* STP power saving: consys told to sleep */
	bool full;
	/*
	 * Full mode sequence state as the vendor stp_core: txseq = next own sequence number,
	 * txack = last consys frame taken (sent back as ack), rxack = last own frame the consys
	 * acknowledged. Own frames rxack+1 .. txseq-1 are kept for retransmission.
	 */
	u8 txseq, txack, rxack;
	u8 retries;			/* retransmission rounds without ack progress */
	unsigned long retx_at;		/* jiffies: retransmit the unacknowledged frames */
	struct {
		u16 len;
		u8 buf[CONSYS_STP_FRAME];
	} txq[CONSYS_STP_SEQS];
	u32 rx_frames;			/* data frames received in full mode */
	u32 rx_dup, unsol;
	u32 rx_oos;			/* data frames out of sequence (dropped, ack repeated) */
	u32 rx_stale_ack;		/* acks without progress while own frames were pending */
	u32 retx_rounds, retx_frames, ack_waits;
	u8 unsol_evt[32];
	size_t unsol_len;
	u8 abuf[1024];			/* frames taken by rx_work */
	u32 psm_sleeps, psm_wakeups;
	int wifi_users;

	/*
	 * Bluetooth function (connac1_bt_on): STP frames of type BT go to bt_ops->recv (stp_lock
	 * held); retx_work retransmits own frames that stay unacknowledged while nobody waits.
	 */
	const struct connac1_bt_ops *bt_ops;
	void *bt_priv;
	bool bt_on;
	struct delayed_work retx_work;
	u8 xbuf[CONSYS_STP_RXMAX];	/* payload of frames other than WMT */
	u32 bt_rx_frames, bt_rx_bytes, bt_tx_frames, bt_tx_bytes, bt_tx_errors;

	/* GNSS function (connac1_gps_on): frames of type GPS go to gps_ops->recv (stp_lock held) */
	const struct connac1_gps_ops *gps_ops;
	void *gps_priv;
	bool gps_on;
	u32 gps_rx_frames, gps_rx_bytes, gps_tx_frames, gps_tx_bytes, gps_tx_errors;

	/* statistics and results */
	u32 tx_bytes, rx_bytes, rx_overflow, irq_empty, irq_total, rx_polled;
	u32 lsr_seen;			/* OR of all LSR values read in the RX path */
	u32 rx_junk, rx_bad_hdr, rx_bad_crc, rx_acks, rx_other;
	u32 tx_waits, tx_wait_max_us, tx_lag_timeouts;
	u32 pda_frags, pda_frag_evts, pda_us, pda_drain_us;
	int err;
	const char *err_step;
	u32 chip_reg[3];
	char patch_name[64];
	u32 patch_size;
	u8 evt[1024];
	size_t evt_len;
	u8 cal[2][512];
	size_t cal_len[2];
	ktime_t t_start;
	unsigned int nsteps;
	struct consys_wmt_step steps[CONSYS_WMT_STEPS];

	/* state captured on a failure, before everything is switched off */
	bool fail_valid;
	u8 fail_btif[CONSYS_BTIF_REGS];
	u32 fail_dma[6];		/* EN, WPT, RPT, VALID, LEFT, DEBUG */
	u32 fail_rxdma[7];		/* EN, INT_FLAG, WPT, RPT, VALID, LEFT, DEBUG */
	u32 fail_boot, fail_16c, fail_coclk, fail_pc[CONSYS_PC_SAMPLES];
	unsigned int fail_rx_pending;
};

enum consys_iomem {
	IO_MCU, IO_HIF_PDMA, IO_HIF_ON, IO_TOP_MISC_OFF, IO_AFE, IO_TOP_MISC_ON, IO_PCCIF4,
	IO_NUM
};

/* bits in consys->on: what has to be undone on power-off */
#define ON_VCN18	BIT(0)
#define ON_VCN33_1	BIT(1)
#define ON_PINS		BIT(2)
#define ON_VCN33_2	BIT(3)
#define ON_CCIF		BIT(4)
#define ON_MTCMOS	BIT(5)
#define ON_CON_STA	BIT(6)

enum consys_state { CONSYS_OFF, CONSYS_ON, CONSYS_FAILED };

struct consys_patch {
	const char *part;		/* file name part, e.g. "ram_bt" */
	u8 type;			/* WMT driver type expected in header byte 0x1f */
	u32 buildtime_off;		/* EMI offset of the debug copy of the build time */
	const struct firmware *fw;
	char name[64];
	char date[PATCH_BUILDTIME_LEN + 1];
	u16 hw_ver, sw_ver;
	u32 emi_off, size, crc;
	bool verified;
};

#define MAX_WRITES	16
#define MAX_STEPS	24

struct consys {
	struct device *dev;
	void __iomem *io[IO_NUM];
	void __iomem *emi;
	phys_addr_t emi_phys;
	size_t emi_size;
	struct regmap *infracfg;
	struct regmap *pericfg;
	struct regmap *scpsys;
	struct regulator *vcn18, *vcn33_1, *vcn33_2;
	struct clk *ccif_clk;
	struct reset_control *mcu_rst;
	struct pinctrl *pinctrl;
	struct pinctrl_state *pins_on, *pins_off;
	bool co_clock;
	bool genpd;			/* CONN MTCMOS through the CONN power domain (runtime PM) */
	bool rpm_held;			/* runtime PM reference taken for a power-on */
	struct dentry *debugfs;

	struct mutex lock;		/* serialises power on/off and status */
	enum consys_state state;
	u32 on;
	int last_err;
	const char *fail_step;

	/* results of the last power-on, for the status file */
	u32 ip_ver, hw_ver, fw_ver, conf_id, ready, chipid_polls, ready_polls;
	ktime_t t_start;
	unsigned int nsteps;
	struct { const char *name; s64 us; } steps[MAX_STEPS];
	unsigned int nwrites;
	struct { const char *name; u32 addr, before, after; } writes[MAX_WRITES];
	bool snap_valid;
	u32 snap[CONSYS_DUMP_REGS];

	struct consys_patch patch[3];
	bool probed;
	bool func_powered;		/* switched on by connac1_wifi/bt/gps_on() */
	bool gps_vcn33_2;		/* VCN33_2 held for GNSS (vendor GPS_PALDO, co-clock) */
	struct consys_wmt wmt;
};

/* mtk-connac1-consys.c */
const char *consys_fw_prefix(void);
const char *consys_fw_variant(void);
bool consys_fw_vercheck(void);
void consys_vcn33_2_off(struct consys *cs);
void consys_conn_sample(struct consys *cs, u32 *boot, u32 *sta16c, u32 *coclk, u32 *pc,
			unsigned int npc);

enum consys_psm { CONSYS_PSM_SLEEP, CONSYS_PSM_WAKEUP, CONSYS_PSM_HOST_AWAKE };

/* mtk-connac1-wmt.c */
int consys_wmt_psm(struct consys *cs, enum consys_psm op);
int consys_wmt_bt(struct consys *cs, bool on, const struct connac1_bt_ops *ops, void *priv);
int consys_wmt_gps(struct consys *cs, bool on, const struct connac1_gps_ops *ops, void *priv);
int consys_wmt_init(struct consys *cs, struct platform_device *pdev);
int consys_wmt_start(struct consys *cs);
void consys_wmt_close(struct consys *cs);
void consys_wmt_step(struct consys *cs, const char *name);
void consys_wmt_show(struct consys *cs, struct seq_file *s);
void consys_wmt_debugfs(struct consys *cs);

/* a function driver (WLAN, BT or GNSS) uses the consys: no power-off through sysfs */
static inline bool consys_func_busy(struct consys *cs)
{
	return cs->wmt.wifi_users || cs->wmt.bt_on || cs->wmt.gps_on;
}

#endif
