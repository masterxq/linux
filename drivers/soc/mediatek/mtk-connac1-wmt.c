// SPDX-License-Identifier: GPL-2.0-only
/*
 * MediaTek CONNAC1 consys: BTIF, STP and the WMT start-up sequence up to the point where the vendor
 * driver reports "LPBK func on" (chip info, MCU patch download, reset, STP full mode, chip id,
 * coex, RF calibration, FM comm, time sync).
 *
 * Protocol details follow MediaTek's GPL-2.0 vendor drivers (btif: btif_plat.c, btif_dma_plat.c;
 * wmt_drv: stp_core.c, wmt_ic_soc.c). The command bytes and their order are those the vendor driver
 * sends. Independent implementation, no vendor code copied verbatim.
 *
 * Like the vendor driver, both BTIF directions use the AP DMA by default (tx_dma, rx_dma): TX
 * through a virtual FIFO drained by polling, RX through a virtual FIFO and the RX DMA interrupt.
 * PIO stays as fallback (byte-wise TX, RX in the BTIF interrupt). Either way received bytes end up
 * in one ring, with a polled fallback while waiting. Everything else runs in process context: the
 * sysfs write or function driver call, rx_work for frames nobody waits for, retx_work.
 *
 * Bluetooth (connac1_bt_on/off/send): WMT function control "BT on/off" (vendor wmt_func_bt_on(),
 * wmt_core_func_ctrl_cmd()) and HCI packets in H4 format inside STP frames of type BT (vendor
 * stp_chrdev_bt.c: mtk_wcn_stp_send_data/receive_data with BT_TASK_INDX). The TX DMA channel keeps
 * being drained by polling in the sending thread (no TX interrupt): every caller sleeps anyway
 * until its frame has left, and BT frames are at most about 1 KiB (0.7 ms on the link).
 *
 * GNSS (connac1_gps_on/off/send): WMT function control "GPS on/off" (vendor wmt_func_gps_on(),
 * function type 2) and the data stream of the GNSS DSP in STP frames of type GPS (vendor
 * stp_chrdev_gps.c with GPS_TASK_INDX 2: one frame per write, received frames appended to a byte
 * stream). The payload is opaque here; the positioning engine runs in user space.
 *
 * Diagnostics (debugfs <dev>/btif_trace, <dev>/status): a ring of the last BTIF transfers with time
 * stamps and their first bytes, per-step byte counters, and on a failure the BTIF/DMA registers and
 * the CONN MCU state, captured before the consys is switched off.
 */

#include <linux/crc16.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/export.h>
#include <linux/firmware.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/kernel.h>
#include <linux/moduleparam.h>
#include <linux/regulator/consumer.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/timekeeping.h>
#include <linux/soc/mediatek/mtk-connac1.h>
#include <linux/uaccess.h>

#include "mtk-connac1.h"

/* BTIF registers (btif_priv.h) */
#define BTIF_RBR		0x00
#define BTIF_THR		0x00
#define BTIF_IER		0x04
#define  BTIF_IER_RXFEN		BIT(0)
#define  BTIF_IER_TXEEN		BIT(1)
#define BTIF_IIR		0x08	/* read */
#define BTIF_FIFOCTRL		0x08	/* write */
#define  BTIF_FIFOCTRL_CLR_RX	BIT(1)
#define  BTIF_FIFOCTRL_CLR_TX	BIT(2)
#define BTIF_FAKELCR		0x0c
#define BTIF_LSR		0x14
#define  BTIF_LSR_DR		BIT(0)
#define  BTIF_LSR_THRE		BIT(5)
#define  BTIF_LSR_TEMT		BIT(6)
#define BTIF_SLEEP_EN		0x48
#define BTIF_DMA_EN		0x4c
#define  BTIF_DMA_EN_RX		BIT(0)
#define  BTIF_DMA_EN_TX		BIT(1)
#define  BTIF_DMA_EN_AUTORST	BIT(2)
#define BTIF_RTOCNT		0x54
#define BTIF_TRI_LVL		0x60
#define  BTIF_TRI_LVL_TX(x)	((x) & 0xf)
#define  BTIF_TRI_LVL_RX(x)	(((x) & 0x7) << 4)
#define  BTIF_TRI_LOOP_EN	BIT(7)
#define BTIF_WAK		0x64
#define  BTIF_WAK_BIT		BIT(0)
#define BTIF_WAT_TIME		0x68
#define BTIF_HANDSHAKE		0x6c
#define  BTIF_HANDSHAKE_EN	BIT(0)

#define BTIF_TX_FIFO_SIZE	16
#define BTIF_TX_FIFO_THRE	(BTIF_TX_FIFO_SIZE / 2)
#define BTIF_RX_FIFO_THRE	1
#define BTIF_TX_TIMEOUT_US	100000	/* per FIFO refill */
#define BTIF_DRAIN_TIMEOUT_US	500000	/* until everything queued has left */
#define BTIF_IRQ_STORM		100000	/* RX interrupts without data before giving up */
#define BTIF_RX_POLL_MS		10	/* RX fallback poll while waiting for data */

/* AP DMA BTIF TX channel (btif_dma_priv.h), virtual FIFO mode */
#define TXDMA_INT_FLAG		0x00
#define TXDMA_INT_EN		0x04
#define TXDMA_EN		0x08
#define TXDMA_RST		0x0c
#define  TXDMA_RST_WARM		BIT(0)
#define  TXDMA_RST_HARD		BIT(1)
#define TXDMA_STOP		0x10
#define TXDMA_FLUSH		0x14
#define TXDMA_VFF_ADDR		0x1c
#define TXDMA_VFF_LEN		0x24
#define TXDMA_VFF_THRE		0x28
#define TXDMA_VFF_WPT		0x2c
#define TXDMA_VFF_RPT		0x30
#define TXDMA_VFF_VALID		0x3c
#define TXDMA_VFF_LEFT		0x40
#define TXDMA_DEBUG		0x50
#define TXDMA_VFF_ADDR_H	0x54
#define TXDMA_VFF_SIZE		8192
#define TXDMA_WPT_MASK		0xffff
#define TXDMA_WPT_WRAP		BIT(16)

/* AP DMA BTIF RX channel (same layout, btif_dma_priv.h) */
#define RXDMA_INT_FLAG		0x00
#define  RXDMA_INT_THRE		BIT(0)
#define  RXDMA_INT_DONE		BIT(1)
#define RXDMA_INT_EN		0x04
#define RXDMA_EN		0x08
#define RXDMA_RST		0x0c
#define RXDMA_STOP		0x10
#define RXDMA_VFF_ADDR		0x1c
#define RXDMA_VFF_LEN		0x24
#define RXDMA_VFF_THRE		0x28
#define RXDMA_VFF_WPT		0x2c
#define RXDMA_VFF_RPT		0x30
#define RXDMA_VFF_VALID		0x3c
#define RXDMA_VFF_LEFT		0x40
#define RXDMA_DEBUG		0x50
#define RXDMA_VFF_ADDR_H	0x54
#define RXDMA_VFF_SIZE		8192	/* vendor RX_DMA_VFF_SIZE */
#define RXDMA_THRE		1	/* vendor DMA_RX_THRE(): interrupt from the first byte */
#define DMA_PTR_MASK		0xffff
#define DMA_PTR_WRAP		BIT(16)

/* STP */
#define STP_TYPE_BT		0
#define STP_TYPE_GPS		2
#define STP_TYPE_WMT		4
#define STP_HDR_LEN		4
#define STP_CRC_LEN		2
#define STP_MAX_CMD		256	/* largest frame sent in mandatory mode (WMT only) */
#define STP_SEQ_MASK		(CONSYS_STP_SEQS - 1)
#define STP_WINSIZE		7	/* vendor MTKSTP_WINSIZE: unacknowledged frames at most */
#define STP_RETRY_LIMIT		10	/* vendor MTKSTP_RETRY_LIMIT */
#define STP_FRAME_MS		50	/* a frame that has started is complete within this */
static_assert(STP_MAX_CMD <= CONSYS_STP_PAYLOAD);
static_assert(CONSYS_STP_PAYLOAD < CONSYS_STP_RXMAX);

/* what __stp_recv() waits for besides a WMT frame */
enum stp_until { STP_UNTIL_FRAME, STP_UNTIL_ACKED, STP_UNTIL_ROOM };

/* WMT */
#define WMT_CAL_TIMEOUT_MS	6000
#define WMT_PDA_FRAG		1000
#define WMT_PDA_HDR_LEN		12
#define WMT_PATCH_HDR_LEN	30	/* struct WMT_PATCH (28) + 2 byte checksum */
#define WMT_PATCH_BUILDTIME_OFF	0x68330

/*
 * Run-time switches for debugging, read at every start of the sequence
 * (/sys/module/mtk_connac1/parameters/).
 */
/*
 * PIO TX: one byte per empty-FIFO check. The vendor refill policy (up to 16 resp. 8 bytes per
 * check, tx_mode 0/1) loses bytes during the MCU patch download; byte by byte works and is not
 * slower (the link takes about 0.66 us per byte, 22 KiB in 15 ms either way). The vendor driver
 * itself always uses TX DMA.
 */
static unsigned int tx_mode = 2;
module_param(tx_mode, uint, 0644);
MODULE_PARM_DESC(tx_mode, "BTIF PIO TX refill: 2 one byte at a time (default), 0 vendor refill (empty: 16, below threshold: 8 bytes) with relaxed 32 bit writes (loses bytes), 1 16 bytes only when empty (loses bytes), 3 vendor refill with vendor writes (8 bit write + full barrier per byte, loses bytes), 4 vendor refill, after each refill wait until LSR shows the FIFO non-empty (test)");
/*
 * The vendor driver runs both BTIF directions through the AP DMA (ENABLE_BTIF_TX_DMA and
 * ENABLE_BTIF_RX_DMA in mtk_btif.h); the default here is the same, PIO stays as fallback.
 * Both are read at every start of the sequence; without the DT ranges PIO is used.
 */
static bool tx_dma = true;
module_param(tx_dma, bool, 0644);
MODULE_PARM_DESC(tx_dma, "send through the AP DMA TX channel like the vendor driver (default; 0 = PIO, see tx_mode)");
static bool rx_dma = true;
module_param(rx_dma, bool, 0644);
MODULE_PARM_DESC(rx_dma, "receive through the AP DMA RX channel like the vendor driver (default; 0 = PIO with the BTIF RX interrupt)");
static unsigned int pda_frag = WMT_PDA_FRAG;
module_param(pda_frag, uint, 0644);
MODULE_PARM_DESC(pda_frag, "MCU patch fragment size in bytes (16..4096, vendor 1000)");
static unsigned int pda_frag_delay_us;
module_param(pda_frag_delay_us, uint, 0644);
MODULE_PARM_DESC(pda_frag_delay_us, "pause after every patch fragment (max 50000)");
static bool pda_frag_drain;
module_param(pda_frag_drain, bool, 0644);
MODULE_PARM_DESC(pda_frag_drain, "wait until each patch fragment has left the BTIF before the next");
static unsigned int pda_frag_wait_ms;
module_param(pda_frag_wait_ms, uint, 0644);
MODULE_PARM_DESC(pda_frag_wait_ms, "after every fragment wait up to this long for an STP event (max 200, 0 = do not wait)");
static unsigned int pda_reset_delay_ms;
module_param(pda_reset_delay_ms, uint, 0644);
MODULE_PARM_DESC(pda_reset_delay_ms, "pause between the last patch byte and the WMT reset (max 1000)");
static unsigned int evt_timeout_ms = 2000;
module_param(evt_timeout_ms, uint, 0644);
MODULE_PARM_DESC(evt_timeout_ms, "timeout for a WMT event (100..10000)");
static unsigned int wak_pulse;
module_param(wak_pulse, uint, 0644);
MODULE_PARM_DESC(wak_pulse, "BTIF wake-up pulse to the consys: bit 0 after BTIF open, bit 1 before the WMT reset");
static unsigned int reset_pad;
module_param(reset_pad, uint, 0644);
MODULE_PARM_DESC(reset_pad, "if the WMT reset gets no event: send this many zero bytes and retry once (max 4096, 0 = off)");
/*
 * Pause after "set stp" before the first full mode frame (vendor: 10 ms; the chip needs at least 2
 * ms for the mode switch). Without it the consys still takes "query stp" in mandatory mode, answers
 * it without acknowledging sequence 0 and then drops every later frame as out of sequence.
 */
static unsigned int stp_switch_ms = 10;
module_param(stp_switch_ms, uint, 0644);
MODULE_PARM_DESC(stp_switch_ms, "pause after switching STP to full mode, ms (vendor 10, max 100; 0 only to test the retransmission)");
static unsigned int stp_retx_ms = 180;
module_param(stp_retx_ms, uint, 0644);
MODULE_PARM_DESC(stp_retx_ms, "STP retransmission timeout, ms (vendor MTKSTP_TX_TIMEOUT 180, 20..2000)");

static inline u32 btif_rd(struct consys_wmt *w, u32 off)
{
	return readl(w->btif + off);
}

static inline void btif_wr(struct consys_wmt *w, u32 off, u32 val)
{
	writel(val, w->btif + off);
}

/* read-modify-write exactly like the vendor BTIF_SET_BIT/BTIF_CLR_BIT */
static inline void btif_set(struct consys_wmt *w, u32 off, u32 bits)
{
	btif_wr(w, off, btif_rd(w, off) | bits);
}

static inline void btif_clr(struct consys_wmt *w, u32 off, u32 bits)
{
	btif_wr(w, off, btif_rd(w, off) & ~bits);
}

/* ---------------------------------------------------------------- trace */

static s64 wmt_ns(struct consys_wmt *w)
{
	return ktime_to_ns(ktime_sub(ktime_get(), w->t_start));
}

static void wmt_trc(struct consys_wmt *w, u8 dir, const char *name, const u8 *data, size_t len,
		    u8 lsr)
{
	struct consys_trc *t;
	unsigned long flags;

	if (!w->trc)
		return;
	spin_lock_irqsave(&w->trc_lock, flags);
	t = &w->trc[w->trc_head];
	w->trc_head = (w->trc_head + 1) & (CONSYS_TRC_N - 1);
	if (w->trc_count < CONSYS_TRC_N)
		w->trc_count++;
	t->ns = wmt_ns(w);
	t->name = name;
	t->dir = dir;
	t->lsr = lsr;
	t->len = min_t(size_t, len, U16_MAX);
	memset(t->data, 0, sizeof(t->data));
	if (data)
		memcpy(t->data, data, min_t(size_t, len, CONSYS_TRC_DATA));
	spin_unlock_irqrestore(&w->trc_lock, flags);
}

static int btif_trace_show(struct seq_file *s, void *unused)
{
	struct consys *cs = s->private;
	struct consys_wmt *w = &cs->wmt;
	unsigned int i, idx, count;
	unsigned long flags;
	struct consys_trc *copy;

	copy = kmalloc_array(CONSYS_TRC_N, sizeof(*copy), GFP_KERNEL);
	if (!copy)
		return -ENOMEM;
	spin_lock_irqsave(&w->trc_lock, flags);
	count = w->trc_count;
	idx = (w->trc_head - count) & (CONSYS_TRC_N - 1);
	for (i = 0; i < count; i++)
		copy[i] = w->trc[(idx + i) & (CONSYS_TRC_N - 1)];
	spin_unlock_irqrestore(&w->trc_lock, flags);

	seq_puts(s, "# us since start, T=PIO tx D=DMA tx R=rx irq P=rx polled M=marker, len, lsr, first bytes\n");
	for (i = 0; i < count; i++) {
		struct consys_trc *t = &copy[i];

		if (t->dir == 'M') {
			seq_printf(s, "%12lld.%03lld M %s\n", t->ns / 1000, t->ns % 1000, t->name);
			continue;
		}
		seq_printf(s, "%12lld.%03lld %c %5u lsr %02x %*ph%s\n", t->ns / 1000, t->ns % 1000,
			   t->dir, t->len, t->lsr, min_t(int, t->len, CONSYS_TRC_DATA), t->data,
			   t->len > CONSYS_TRC_DATA ? " ..." : "");
	}
	kfree(copy);
	return 0;
}
DEFINE_SHOW_ATTRIBUTE(btif_trace);

void consys_wmt_step(struct consys *cs, const char *name)
{
	struct consys_wmt *w = &cs->wmt;

	if (w->nsteps < CONSYS_WMT_STEPS) {
		struct consys_wmt_step *st = &w->steps[w->nsteps++];

		st->name = name;
		st->us = ktime_us_delta(ktime_get(), w->t_start);
		st->tx = w->tx_bytes;
		st->rx = w->rx_bytes;
		st->irqs = w->irq_total;
	}
	wmt_trc(w, 'M', name, NULL, 0, 0);
}

/* ---------------------------------------------------------------- BTIF RX */

static unsigned int rx_count(struct consys_wmt *w)
{
	return (w->rx_head - w->rx_tail) & (CONSYS_RX_RING - 1);
}

/* one received byte into the ring; rx_lock held */
static void rx_put(struct consys_wmt *w, u8 c)
{
	if (rx_count(w) == CONSYS_RX_RING - 1) {
		w->rx_overflow++;
	} else {
		w->rx_ring[w->rx_head] = c;
		w->rx_head = (w->rx_head + 1) & (CONSYS_RX_RING - 1);
	}
	w->rx_bytes++;
}

static inline u32 rxdma_rd(struct consys_wmt *w, u32 off)
{
	return readl(w->rxdma + off);
}

static inline void rxdma_wr(struct consys_wmt *w, u32 off, u32 val)
{
	writel(val, w->rxdma + off);
}

/* take everything the RX FIFO holds; rx_lock held */
static unsigned int btif_drain_rx(struct consys_wmt *w, u8 dir)
{
	u8 first[CONSYS_TRC_DATA];
	unsigned int n = 0;
	u32 lsr, lsr0;

	lsr0 = lsr = btif_rd(w, BTIF_LSR);
	w->lsr_seen |= lsr;
	while (lsr & BTIF_LSR_DR) {
		u8 c = btif_rd(w, BTIF_RBR) & 0xff;

		if (n < sizeof(first))
			first[n] = c;
		rx_put(w, c);
		if (++n > 4096)
			break;
		lsr = btif_rd(w, BTIF_LSR);
		w->lsr_seen |= lsr;
	}
	if (n)
		wmt_trc(w, dir, NULL, first, n, lsr0);
	return n;
}

/*
 * AP DMA RX channel (vendor hal_rx_dma_irq_handler()): the hardware writes into the virtual FIFO
 * and advances WPT; the driver copies from RPT to WPT (wrap bits in bit 16) and writes RPT back.
 * VALID counts bytes still inside the channel; the vendor loops until it is 0 as well, here
 * that wait is bounded. rx_lock held.
 */
static unsigned int rxdma_drain(struct consys_wmt *w, u8 dir)
{
	u8 first[CONSYS_TRC_DATA];
	unsigned int n = 0, spins = 0;
	u32 rpt, wpt;

	rpt = rxdma_rd(w, RXDMA_VFF_RPT);
	wpt = rxdma_rd(w, RXDMA_VFF_WPT);
	for (;;) {
		u32 r = rpt & DMA_PTR_MASK, wp = wpt & DMA_PTR_MASK;
		unsigned int avail;

		if (rpt == wpt) {
			/* bytes in flight inside the channel: they land within microseconds */
			if (!(rxdma_rd(w, RXDMA_VFF_VALID) & 0xffff) || ++spins > 100)
				break;
			w->rxdma_spins++;
			udelay(1);
			wpt = rxdma_rd(w, RXDMA_VFF_WPT);
			continue;
		}
		avail = ((rpt ^ wpt) & DMA_PTR_WRAP) ? wp + RXDMA_VFF_SIZE - r : wp - r;
		if (avail > RXDMA_VFF_SIZE)
			break;			/* inconsistent pointers; status shows them */
		dma_rmb();			/* data behind WPT is visible after reading WPT */
		while (avail--) {
			u8 c = w->rx_vff[r];

			if (n < sizeof(first))
				first[n] = c;
			n++;
			rx_put(w, c);
			if (++r == RXDMA_VFF_SIZE) {
				r = 0;
				rpt ^= DMA_PTR_WRAP;
			}
		}
		rpt = (rpt & DMA_PTR_WRAP) | r;
		rxdma_wr(w, RXDMA_VFF_RPT, rpt);
		wpt = rxdma_rd(w, RXDMA_VFF_WPT);
	}
	if (n)
		wmt_trc(w, dir, NULL, first, n, 0);
	return n;
}

static irqreturn_t rxdma_irq(int irq, void *data)
{
	struct consys_wmt *w = data;
	unsigned int n;

	spin_lock(&w->rx_lock);
	w->rxdma_irqs++;
	/* vendor order: interrupt enable off, flags cleared (written as ones), drain, enable on */
	rxdma_wr(w, RXDMA_INT_EN, 0);
	rxdma_wr(w, RXDMA_INT_FLAG, rxdma_rd(w, RXDMA_INT_FLAG) | RXDMA_INT_THRE | RXDMA_INT_DONE);
	n = rxdma_drain(w, 'X');
	rxdma_wr(w, RXDMA_INT_EN, RXDMA_INT_THRE | RXDMA_INT_DONE);
	spin_unlock(&w->rx_lock);

	if (n) {
		wake_up(&w->rx_wq);
		if (READ_ONCE(w->async))
			schedule_work(&w->rx_work);
	}
	return IRQ_HANDLED;
}

static irqreturn_t btif_irq(int irq, void *data)
{
	struct consys_wmt *w = data;
	unsigned int n;

	spin_lock(&w->rx_lock);
	w->irq_total++;
	/* with RX DMA the RX FIFO belongs to the DMA channel: never read RBR then */
	n = w->rxdma_on ? rxdma_drain(w, 'R') : btif_drain_rx(w, 'R');
	spin_unlock(&w->rx_lock);

	if (!n) {
		/* not ours or nothing pending: guard against a stuck level interrupt */
		if (++w->irq_empty > BTIF_IRQ_STORM) {
			disable_irq_nosync(irq);
			w->irq_disabled = true;
		}
		return IRQ_NONE;
	}
	wake_up(&w->rx_wq);
	if (READ_ONCE(w->async))
		schedule_work(&w->rx_work);
	return IRQ_HANDLED;
}

/* fallback if the interrupt does not come: look at the FIFO (or the RX DMA buffer) directly */
static void btif_poll_rx(struct consys_wmt *w)
{
	unsigned long flags;
	unsigned int n;

	spin_lock_irqsave(&w->rx_lock, flags);
	n = w->rxdma_on ? rxdma_drain(w, 'P') : btif_drain_rx(w, 'P');
	w->rx_polled += n;
	spin_unlock_irqrestore(&w->rx_lock, flags);
}

/* take exactly len bytes from the RX ring, waiting at most until the deadline */
static int btif_rx(struct consys_wmt *w, u8 *buf, size_t len, unsigned long deadline)
{
	while (len) {
		unsigned long flags;
		long left = (long)(deadline - jiffies);

		if (w->irq_disabled)
			return -EIO;
		if (!rx_count(w)) {
			if (left <= 0)
				return -ETIMEDOUT;
			wait_event_timeout(w->rx_wq, rx_count(w) || w->irq_disabled,
					   min_t(long, left, msecs_to_jiffies(BTIF_RX_POLL_MS)));
			if (!rx_count(w))
				btif_poll_rx(w);
			continue;
		}
		spin_lock_irqsave(&w->rx_lock, flags);
		while (len && rx_count(w)) {
			*buf++ = w->rx_ring[w->rx_tail];
			w->rx_tail = (w->rx_tail + 1) & (CONSYS_RX_RING - 1);
			len--;
		}
		spin_unlock_irqrestore(&w->rx_lock, flags);
	}
	return 0;
}

/* ---------------------------------------------------------------- BTIF TX, PIO */

static int btif_wait_lsr(struct consys_wmt *w, u32 mask, u32 *lsr, bool atomic)
{
	ktime_t t0 = ktime_get();
	u32 us;
	int ret;

	*lsr = btif_rd(w, BTIF_LSR);
	if (*lsr & mask)
		return 0;
	w->tx_waits++;
	if (atomic)
		ret = readl_poll_timeout_atomic(w->btif + BTIF_LSR, *lsr, *lsr & mask, 1,
						BTIF_TX_TIMEOUT_US);
	else
		ret = readl_poll_timeout(w->btif + BTIF_LSR, *lsr, *lsr & mask, 10,
					 BTIF_TX_TIMEOUT_US);
	us = ktime_us_delta(ktime_get(), t0);
	if (us > w->tx_wait_max_us)
		w->tx_wait_max_us = us;
	return ret;
}

static int btif_tx_pio(struct consys_wmt *w, const u8 *buf, size_t len, unsigned int mode)
{
	wmt_trc(w, 'T', NULL, buf, len, 0);
	while (len) {
		unsigned int room, n;
		u32 lsr;
		int ret;

		switch (mode) {
		case 1:
			ret = btif_wait_lsr(w, BTIF_LSR_TEMT, &lsr, true);
			room = BTIF_TX_FIFO_SIZE;
			break;
		case 2:
			ret = btif_wait_lsr(w, BTIF_LSR_TEMT, &lsr, true);
			room = 1;
			break;
		default:
			/* vendor hal_btif_send_data(): empty -> 16, below threshold -> 8 */
			ret = btif_wait_lsr(w, BTIF_LSR_TEMT | BTIF_LSR_THRE, &lsr, false);
			room = (lsr & BTIF_LSR_TEMT) ? BTIF_TX_FIFO_SIZE :
						       BTIF_TX_FIFO_SIZE - BTIF_TX_FIFO_THRE;
			break;
		}
		if (ret)
			return ret;
		n = min_t(size_t, room, len);
		len -= n;
		w->tx_bytes += n;
		if (mode == 3) {
			/* vendor btif_reg_sync_writeb(): byte write, then mb() before the next */
			while (n--) {
				writeb(*buf++, w->btif + BTIF_THR);
				mb();
			}
		} else {
			while (n--)
				btif_wr(w, BTIF_THR, *buf++);
		}
		if (mode == 4) {
			/*
			 * Wait until LSR shows the bytes just written (TEMT low) before the next
			 * look at THRE/TEMT. If LSR lags behind the writes, the refill policy
			 * otherwise sees a stale "empty" and overfills the FIFO.
			 */
			u32 v;

			if (readl_poll_timeout_atomic(w->btif + BTIF_LSR, v, !(v & BTIF_LSR_TEMT), 0, 20))
				w->tx_lag_timeouts++;
		}
	}
	return 0;
}

/* ---------------------------------------------------------------- BTIF TX, AP DMA */

static inline u32 txdma_rd(struct consys_wmt *w, u32 off)
{
	return readl(w->txdma + off);
}

static inline void txdma_wr(struct consys_wmt *w, u32 off, u32 val)
{
	writel(val, w->txdma + off);
}

/*
 * vendor hal_btif_dma_hw_init() for the TX direction, interrupts off: commands and their events
 * alternate, so every write is drained right away by polling (txdma_drain() does the flush the
 * vendor TX DMA interrupt does). AP DMA clock on (btif_open).
 */
static int txdma_start(struct consys_wmt *w)
{
	u32 v;
	int ret;

	txdma_wr(w, TXDMA_RST, txdma_rd(w, TXDMA_RST) | TXDMA_RST_WARM);
	ret = readl_poll_timeout(w->txdma + TXDMA_EN, v, !(v & 1), 100, 1000);
	if (ret) {
		txdma_wr(w, TXDMA_RST, txdma_rd(w, TXDMA_RST) | TXDMA_RST_HARD);
		txdma_wr(w, TXDMA_RST, txdma_rd(w, TXDMA_RST) & ~TXDMA_RST_HARD);
	}
	txdma_wr(w, TXDMA_VFF_ADDR, lower_32_bits(w->vff_dma));
	txdma_wr(w, TXDMA_VFF_ADDR_H, upper_32_bits(w->vff_dma));
	txdma_wr(w, TXDMA_VFF_LEN, TXDMA_VFF_SIZE);
	txdma_wr(w, TXDMA_VFF_WPT, 0);
	txdma_wr(w, TXDMA_VFF_RPT, 0);
	txdma_wr(w, TXDMA_VFF_THRE, TXDMA_VFF_SIZE - 7);
	txdma_wr(w, TXDMA_INT_FLAG, txdma_rd(w, TXDMA_INT_FLAG) & ~1);
	txdma_wr(w, TXDMA_INT_EN, 0);
	w->vff_wpt = 0;

	/* vendor _btif_tx_dma_setup(): channel enable, then the BTIF into TX DMA mode */
	txdma_wr(w, TXDMA_EN, txdma_rd(w, TXDMA_EN) | 1);
	btif_set(w, BTIF_DMA_EN, BTIF_DMA_EN_TX);
	w->dma_on = true;
	return 0;
}

/* wait until the virtual FIFO is empty; residues below 8 bytes need a flush (vendor) */
static int txdma_drain(struct consys_wmt *w, unsigned int timeout_us)
{
	ktime_t end = ktime_add_us(ktime_get(), timeout_us);

	for (;;) {
		u32 left = txdma_rd(w, TXDMA_VFF_LEFT) & 0xffff;
		u32 valid = txdma_rd(w, TXDMA_VFF_VALID) & 0xffff;

		if (left == TXDMA_VFF_SIZE && !valid)
			return 0;
		if (valid && valid < 8 && !(txdma_rd(w, TXDMA_FLUSH) & 1) &&
		    !(txdma_rd(w, TXDMA_STOP) & 1))
			txdma_wr(w, TXDMA_FLUSH, txdma_rd(w, TXDMA_FLUSH) | 1);
		if (ktime_after(ktime_get(), end))
			return -ETIMEDOUT;
		usleep_range(20, 50);
	}
}

static void txdma_stop(struct consys_wmt *w)
{
	u32 v;

	if (w->dma_on) {
		txdma_drain(w, 20000);
		txdma_wr(w, TXDMA_STOP, txdma_rd(w, TXDMA_STOP) | 1);
		readl_poll_timeout_atomic(w->txdma + TXDMA_STOP, v, !(v & 1), 1, 10000);
		txdma_wr(w, TXDMA_EN, 0);
		btif_clr(w, BTIF_DMA_EN, BTIF_DMA_EN_TX);
		w->dma_on = false;
	}
}

/*
 * vendor _btif_rx_dma_setup() / hal_btif_dma_hw_init() for the RX direction: warm reset (hard
 * reset if the channel does not stop), virtual FIFO, threshold 1, flags cleared, interrupts on,
 * channel enable, BTIF into RX DMA mode. The BTIF RX interrupt stays off then (the vendor keeps
 * it enabled, but reading RBR would take bytes past the DMA; with DMA it has nothing to do).
 */
static int rxdma_start(struct consys_wmt *w)
{
	u32 v;
	int ret;

	rxdma_wr(w, RXDMA_RST, rxdma_rd(w, RXDMA_RST) | TXDMA_RST_WARM);
	ret = readl_poll_timeout(w->rxdma + RXDMA_EN, v, !(v & 1), 100, 1000);
	if (ret) {
		rxdma_wr(w, RXDMA_RST, rxdma_rd(w, RXDMA_RST) | TXDMA_RST_HARD);
		rxdma_wr(w, RXDMA_RST, rxdma_rd(w, RXDMA_RST) & ~TXDMA_RST_HARD);
	}
	rxdma_wr(w, RXDMA_VFF_ADDR, lower_32_bits(w->rx_vff_dma));
	rxdma_wr(w, RXDMA_VFF_ADDR_H, upper_32_bits(w->rx_vff_dma));
	rxdma_wr(w, RXDMA_VFF_LEN, RXDMA_VFF_SIZE);
	rxdma_wr(w, RXDMA_VFF_WPT, 0);
	rxdma_wr(w, RXDMA_VFF_RPT, 0);
	rxdma_wr(w, RXDMA_VFF_THRE, RXDMA_THRE);
	rxdma_wr(w, RXDMA_INT_FLAG, rxdma_rd(w, RXDMA_INT_FLAG) | RXDMA_INT_THRE | RXDMA_INT_DONE);
	rxdma_wr(w, RXDMA_INT_EN, RXDMA_INT_THRE | RXDMA_INT_DONE);

	ret = request_irq(w->rxdma_irq, rxdma_irq, 0, "mtk-connac1-btif-rxdma", w);
	if (ret)
		return ret;
	w->rxdma_irq_on = true;
	w->rxdma_irqs = w->rxdma_spins = 0;
	btif_clr(w, BTIF_IER, BTIF_IER_RXFEN);
	w->rxdma_on = true;
	rxdma_wr(w, RXDMA_EN, rxdma_rd(w, RXDMA_EN) | 1);
	btif_set(w, BTIF_DMA_EN, BTIF_DMA_EN_RX);
	return 0;
}

/* vendor btif_rx_dma_ctrl(DISABLE): STOP, wait until it is taken; then back to PIO */
static void rxdma_stop(struct consys_wmt *w)
{
	u32 v;

	if (w->rxdma_on) {
		btif_clr(w, BTIF_DMA_EN, BTIF_DMA_EN_RX);
		rxdma_wr(w, RXDMA_STOP, rxdma_rd(w, RXDMA_STOP) | 1);
		readl_poll_timeout_atomic(w->rxdma + RXDMA_STOP, v, !(v & 1), 1, 10000);
		rxdma_wr(w, RXDMA_INT_EN, 0);
		rxdma_wr(w, RXDMA_EN, 0);
	}
	if (w->rxdma_irq_on) {
		free_irq(w->rxdma_irq, w);
		w->rxdma_irq_on = false;
	}
	w->rxdma_on = false;
}

static int btif_tx_dma(struct consys_wmt *w, const u8 *buf, size_t len)
{
	wmt_trc(w, 'D', NULL, buf, len, 0);
	while (len) {
		u32 left, wpt, n, tail, v;
		int ret;

		ret = readl_poll_timeout(w->txdma + TXDMA_VFF_LEFT, left, left & 0xffff, 20,
					 BTIF_TX_TIMEOUT_US);
		if (ret)
			return ret;
		left &= 0xffff;
		n = min_t(size_t, left, len);
		wpt = w->vff_wpt & TXDMA_WPT_MASK;
		tail = min_t(u32, n, TXDMA_VFF_SIZE - wpt);
		memcpy(w->vff + wpt, buf, tail);
		memcpy(w->vff, buf + tail, n - tail);
		wpt += n;
		if (wpt >= TXDMA_VFF_SIZE)
			w->vff_wpt = (wpt - TXDMA_VFF_SIZE) | ((w->vff_wpt & TXDMA_WPT_WRAP) ^ TXDMA_WPT_WRAP);
		else
			w->vff_wpt = wpt | (w->vff_wpt & TXDMA_WPT_WRAP);
		/* a running flush stops the channel; wait for it, then (re)enable */
		readl_poll_timeout_atomic(w->txdma + TXDMA_FLUSH, v, !(v & 1), 1, 1000);
		dma_wmb();
		txdma_wr(w, TXDMA_EN, txdma_rd(w, TXDMA_EN) | 1);
		txdma_wr(w, TXDMA_VFF_WPT, w->vff_wpt);
		v = txdma_rd(w, TXDMA_VFF_VALID) & 0xffff;
		if (v && v < 8)
			txdma_wr(w, TXDMA_FLUSH, txdma_rd(w, TXDMA_FLUSH) | 1);
		w->tx_bytes += n;
		buf += n;
		len -= n;
	}
	/* the channel moves 8 byte units; get the rest out now (vendor: flush in the TX irq) */
	return txdma_drain(w, BTIF_DRAIN_TIMEOUT_US);
}

static int btif_tx(struct consys_wmt *w, const u8 *buf, size_t len)
{
	if (w->dma_on)
		return btif_tx_dma(w, buf, len);
	return btif_tx_pio(w, buf, len, min(tx_mode, 4U));
}

/* wait until everything queued has left the AP side */
static int btif_drain(struct consys_wmt *w, unsigned int timeout_us)
{
	u32 lsr;
	int ret;

	if (w->dma_on) {
		ret = txdma_drain(w, timeout_us);
		if (ret)
			return ret;
	}
	return readl_poll_timeout(w->btif + BTIF_LSR, lsr, lsr & BTIF_LSR_TEMT, 10, timeout_us);
}

/* vendor hal_btif_raise_wak_sig(): ap_wakeup_consys low for > 1/32 kHz, then high */
static void btif_wak_pulse(struct consys_wmt *w)
{
	btif_clr(w, BTIF_WAK, BTIF_WAK_BIT);
	usleep_range(128, 160);
	btif_set(w, BTIF_WAK, BTIF_WAK_BIT);
	wmt_trc(w, 'M', "BTIF_WAK pulse", NULL, 0, 0);
}

/* vendor hal_btif_hw_init(): normal mode, new handshake, FIFO clear, trigger levels, PIO, RX irq */
static void btif_hw_init(struct consys_wmt *w)
{
	btif_wr(w, BTIF_FAKELCR, 0);
	btif_set(w, BTIF_HANDSHAKE, BTIF_HANDSHAKE_EN);
	btif_set(w, BTIF_FIFOCTRL, BTIF_FIFOCTRL_CLR_RX);
	btif_clr(w, BTIF_FIFOCTRL, BTIF_FIFOCTRL_CLR_RX);
	btif_set(w, BTIF_FIFOCTRL, BTIF_FIFOCTRL_CLR_TX);
	btif_clr(w, BTIF_FIFOCTRL, BTIF_FIFOCTRL_CLR_TX);
	btif_wr(w, BTIF_TRI_LVL, BTIF_TRI_LVL_TX(BTIF_TX_FIFO_THRE) | BTIF_TRI_LVL_RX(BTIF_RX_FIFO_THRE));
	btif_clr(w, BTIF_TRI_LVL, BTIF_TRI_LOOP_EN);
	btif_clr(w, BTIF_DMA_EN, BTIF_DMA_EN_TX);
	btif_clr(w, BTIF_DMA_EN, BTIF_DMA_EN_RX);
	btif_set(w, BTIF_DMA_EN, BTIF_DMA_EN_AUTORST);
	btif_clr(w, BTIF_IER, BTIF_IER_TXEEN);
	btif_set(w, BTIF_IER, BTIF_IER_RXFEN);
}

static int btif_open(struct consys *cs)
{
	struct consys_wmt *w = &cs->wmt;
	int ret;

	ret = clk_prepare_enable(w->btif_clk);
	if (ret)
		return ret;
	w->clk_on = true;
	w->rx_head = w->rx_tail = 0;
	w->rx_bytes = w->rx_overflow = w->irq_empty = w->irq_total = w->rx_polled = 0;
	w->lsr_seen = 0;
	w->irq_disabled = false;
	btif_hw_init(w);
	ret = request_irq(w->irq, btif_irq, 0, "mtk-connac1-btif", w);
	if (ret)
		goto err;
	w->irq_on = true;

	if ((tx_dma && !w->txdma) || (rx_dma && !w->rxdma))
		dev_warn_once(cs->dev, "BTIF DMA requested but not described in DT, using PIO\n");
	if ((tx_dma && w->txdma) || (rx_dma && w->rxdma)) {
		ret = clk_prepare_enable(w->dma_clk);
		if (ret)
			goto err;
		w->dma_clk_on = true;
	}
	if (rx_dma && w->rxdma) {
		ret = rxdma_start(w);
		if (ret)
			goto err;
	}
	if (tx_dma && w->txdma) {
		ret = txdma_start(w);
		if (ret)
			goto err;
	}
	return 0;
err:
	consys_wmt_close(cs);
	return ret;
}

void consys_wmt_close(struct consys *cs)
{
	struct consys_wmt *w = &cs->wmt;

	WRITE_ONCE(w->async, false);
	if (w->irq_on) {
		btif_clr(w, BTIF_IER, BTIF_IER_RXFEN);
		free_irq(w->irq, w);
		w->irq_on = false;
	}
	/* no new work after free_irq; a running one sees async == false and stops */
	cancel_work_sync(&w->rx_work);
	cancel_delayed_work_sync(&w->retx_work);
	/* a BT or GNSS sender inside the STP lock finishes its frame before the BTIF goes down */
	mutex_lock(&w->stp_lock);
	w->bt_ops = NULL;
	w->bt_priv = NULL;
	w->bt_on = false;
	w->gps_ops = NULL;
	w->gps_priv = NULL;
	w->gps_on = false;
	mutex_unlock(&w->stp_lock);
	if (w->clk_on) {
		txdma_stop(w);
		rxdma_stop(w);
	}
	if (w->dma_clk_on) {
		clk_disable_unprepare(w->dma_clk);
		w->dma_clk_on = false;
	}
	if (w->clk_on) {
		clk_disable_unprepare(w->btif_clk);
		w->clk_on = false;
	}
	w->ready = false;
	w->asleep = false;
	w->wifi_users = 0;
}

/* ---------------------------------------------------------------- STP */

/*
 * Mandatory mode (until "set stp"): 80 | 40+len_hi | len_lo | 00, payload, 00 00.
 * Full mode: 80+(seq<<3)+ack | (type<<4)+len_hi | len_lo | sum(hdr[0..2]), payload, CRC-16
 * (ARC, little endian). "ack" is the sequence number of the last received packet (7 at start).
 * ACK-only packets: 80+ack | 00 | 00 | sum.
 *
 * Full mode reliability follows the vendor stp_core: every own data frame stays queued until
 * the consys acknowledges it; without ack progress for stp_retx_ms the host sends the resync
 * pattern 7f 7f 7f 7f and all unacknowledged frames again (ack field refreshed), at most
 * STP_RETRY_LIMIT rounds. Received data frames are taken only in sequence; anything else is
 * dropped and the previous ack is repeated, so the consys retransmits. Unlike the vendor there
 * is no timer: retransmissions happen while a caller waits for frames (stp_recv), which is
 * always the case while a command is outstanding.
 */
static unsigned long stp_retx_timeout(void)
{
	return msecs_to_jiffies(clamp(stp_retx_ms, 20U, 2000U));
}

/* own full mode frames sent but not yet acknowledged */
static unsigned int stp_unacked(struct consys_wmt *w)
{
	return (w->txseq - w->rxack - 1) & STP_SEQ_MASK;
}

static void stp_full_mode(struct consys_wmt *w)
{
	w->full = true;
	w->txseq = 0;
	w->txack = STP_SEQ_MASK;
	w->rxack = STP_SEQ_MASK;
	w->retries = 0;
}

/*
 * One STP frame of the given type; "prefix" >= 0 is put in front of the payload (the H4 type
 * byte of a BT packet). In full mode the frame is built in its retransmission slot.
 */
static int stp_send_frame(struct consys_wmt *w, u8 type, int prefix, const u8 *payload, size_t len)
{
	u8 small[STP_HDR_LEN + STP_MAX_CMD + STP_CRC_LEN];
	size_t plen = len + (prefix >= 0);
	size_t n = STP_HDR_LEN + plen + STP_CRC_LEN;
	u8 *frame, *data, *tail;
	int ret;

	/* one contiguous write per frame (one DMA transfer, one trace record) */
	if (w->full) {
		if (plen > CONSYS_STP_PAYLOAD)
			return -EINVAL;
		if (stp_unacked(w) >= STP_WINSIZE)
			return -EBUSY;
		frame = w->txq[w->txseq].buf;
	} else {
		if (plen > STP_MAX_CMD)
			return -EINVAL;
		frame = small;
	}
	data = frame + STP_HDR_LEN;
	if (prefix >= 0)
		*data++ = prefix;
	memcpy(data, payload, len);
	tail = frame + STP_HDR_LEN + plen;
	tail[0] = tail[1] = 0;
	if (w->full) {
		u16 crc = crc16(0, frame + STP_HDR_LEN, plen);

		frame[0] = 0x80 | (w->txseq << 3) | w->txack;
		tail[0] = crc & 0xff;
		tail[1] = crc >> 8;
		w->txq[w->txseq].len = n;
	} else {
		frame[0] = 0x80;
	}
	frame[1] = (type << 4) | ((plen >> 8) & 0x0f);
	frame[2] = plen & 0xff;
	frame[3] = w->full ? (frame[0] + frame[1] + frame[2]) & 0xff : 0;
	ret = btif_tx(w, frame, n);
	if (!ret && w->full) {
		w->txseq = (w->txseq + 1) & STP_SEQ_MASK;
		/* vendor: the retry timer restarts with every new frame */
		w->retx_at = jiffies + stp_retx_timeout();
	}
	return ret;
}

static int stp_send(struct consys_wmt *w, const u8 *payload, size_t len)
{
	return stp_send_frame(w, STP_TYPE_WMT, -1, payload, len);
}

static int stp_send_ack(struct consys_wmt *w)
{
	u8 hdr[STP_HDR_LEN];

	hdr[0] = 0x80 | w->txack;
	hdr[1] = 0;
	hdr[2] = 0;
	hdr[3] = hdr[0];
	return btif_tx(w, hdr, sizeof(hdr));
}

/* vendor stp_process_rxack(): accept an ack that covers frames actually outstanding */
static void stp_rx_ack(struct consys_wmt *w, u8 ack)
{
	unsigned int n = (ack - w->rxack) & STP_SEQ_MASK;

	if (!n || n > stp_unacked(w)) {
		if (stp_unacked(w))
			w->rx_stale_ack++;
		return;
	}
	w->rxack = ack;
	w->retries = 0;
	w->retx_at = jiffies + stp_retx_timeout();
}

/* vendor stp_do_tx_timeout(): resync pattern, then every unacknowledged frame again */
static int stp_retransmit(struct consys_wmt *w)
{
	static const u8 resync[STP_HDR_LEN] = { 0x7f, 0x7f, 0x7f, 0x7f };
	struct consys *cs = container_of(w, struct consys, wmt);
	unsigned int seq;
	int ret;

	if (w->retries >= STP_RETRY_LIMIT)
		return -ETIMEDOUT;
	w->retries++;
	w->retx_rounds++;
	dev_warn_ratelimited(cs->dev, "STP: no ack for %u frame(s), retransmission %u (rxack %u txack %u txseq %u)\n",
			     stp_unacked(w), w->retries, w->rxack, w->txack, w->txseq);
	wmt_trc(w, 'M', "STP retransmit", NULL, 0, 0);
	ret = btif_tx(w, resync, sizeof(resync));
	for (seq = (w->rxack + 1) & STP_SEQ_MASK; !ret && seq != w->txseq;
	     seq = (seq + 1) & STP_SEQ_MASK) {
		u8 *f = w->txq[seq].buf;

		/* vendor stp_update_tx_queue(): current ack, new header checksum */
		f[0] = (f[0] & ~STP_SEQ_MASK) | w->txack;
		f[3] = (f[0] + f[1] + f[2]) & 0xff;
		ret = btif_tx(w, f, w->txq[seq].len);
		w->retx_frames++;
	}
	w->retx_at = jiffies + stp_retx_timeout();
	return ret;
}

/* wait for the first byte of a frame; retransmit on the way while own frames are unacknowledged */
static int stp_rx_byte(struct consys_wmt *w, u8 *c, unsigned long deadline)
{
	for (;;) {
		bool retx = w->full && stp_unacked(w) && time_before(w->retx_at, deadline);
		int ret;

		ret = btif_rx(w, c, 1, retx ? w->retx_at : deadline);
		if (ret != -ETIMEDOUT || !retx)
			return ret;
		ret = stp_retransmit(w);
		if (ret)
			return ret;
	}
}

/* a frame of another STP type, taken in sequence (and acknowledged in full mode) */
static void stp_deliver(struct consys_wmt *w, u8 type, const u8 *data, size_t len)
{
	if (type == STP_TYPE_BT && w->bt_ops) {
		w->bt_rx_frames++;
		w->bt_rx_bytes += len;
		w->bt_ops->recv(w->bt_priv, data, len);
		return;
	}
	if (type == STP_TYPE_GPS && w->gps_ops) {
		w->gps_rx_frames++;
		w->gps_rx_bytes += len;
		w->gps_ops->recv(w->gps_priv, data, len);
		return;
	}
	w->rx_other++;
}

static bool stp_until_met(struct consys_wmt *w, enum stp_until until)
{
	switch (until) {
	case STP_UNTIL_ACKED:
		return !stp_unacked(w);
	case STP_UNTIL_ROOM:
		return stp_unacked(w) < STP_WINSIZE;
	default:
		return false;
	}
}

/*
 * Receive one STP data packet. ACK-only packets are consumed on the way; in full mode every
 * data packet taken is acknowledged, out-of-sequence packets (repeats included) are dropped
 * with the previous ack repeated. Returns 0 with a WMT packet in "payload", 2 after a packet of
 * another type was handed on (BT, GPS) or dropped (FM), 1 as soon as "until" is met (own frames
 * acknowledged / room in the window), or a negative error.
 */
static int __stp_recv(struct consys_wmt *w, u8 *payload, size_t max, size_t *len,
		      unsigned long deadline, enum stp_until until)
{
	u8 hdr[STP_HDR_LEN], crc[STP_CRC_LEN];
	size_t n, lim;
	u8 *dst;
	int ret;

	for (;;) {
		unsigned long fdl;
		u8 seq, type;

		if (stp_until_met(w, until))
			return 1;
		/* resynchronise on the start bit */
		do {
			ret = stp_rx_byte(w, hdr, deadline);
			if (ret)
				return ret;
			if (!(hdr[0] & 0x80))
				w->rx_junk++;
		} while (!(hdr[0] & 0x80));
		/* the rest of a started frame is not cut off by the caller's deadline */
		fdl = jiffies + msecs_to_jiffies(STP_FRAME_MS);
		if (time_after(deadline, fdl))
			fdl = deadline;
		ret = btif_rx(w, hdr + 1, 3, fdl);
		if (ret)
			return ret;
		n = ((hdr[1] & 0x0f) << 8) | hdr[2];
		type = hdr[1] >> 4;

		if (w->full && hdr[3] != ((hdr[0] + hdr[1] + hdr[2]) & 0xff)) {
			w->rx_bad_hdr++;
			continue;
		}
		if (n == 0) {
			/* ACK-only packet */
			w->rx_acks++;
			if (w->full)
				stp_rx_ack(w, hdr[0] & STP_SEQ_MASK);
			continue;
		}
		if (type == STP_TYPE_WMT) {
			dst = payload;
			lim = max;
		} else {
			dst = w->xbuf;
			lim = sizeof(w->xbuf);
		}
		if (n > lim) {
			w->rx_bad_hdr++;
			return -EMSGSIZE;
		}
		ret = btif_rx(w, dst, n, fdl);
		if (!ret)
			ret = btif_rx(w, crc, sizeof(crc), fdl);
		if (ret)
			return ret;
		if (w->full) {
			if (crc16(0, dst, n) != (crc[0] | (crc[1] << 8))) {
				/* no ack: the consys sends it again (vendor behaviour) */
				w->rx_bad_crc++;
				continue;
			}
			seq = (hdr[0] >> 3) & STP_SEQ_MASK;
			if (seq != ((w->txack + 1) & STP_SEQ_MASK)) {
				if (seq == w->txack)
					w->rx_dup++;
				else
					w->rx_oos++;
				ret = stp_send_ack(w);
				if (ret)
					return ret;
				continue;
			}
			stp_rx_ack(w, hdr[0] & STP_SEQ_MASK);
			w->rx_frames++;
			w->txack = seq;
			ret = stp_send_ack(w);
			if (ret)
				return ret;
		}
		if (type != STP_TYPE_WMT) {
			stp_deliver(w, type, dst, n);
			return 2;
		}
		*len = n;
		return 0;
	}
}

/*
 * Receive one WMT data packet; packets of other types are handed on meanwhile. With
 * "until_acked", return 1 as soon as all own frames are acknowledged.
 */
static int stp_recv(struct consys_wmt *w, u8 *payload, size_t max, size_t *len,
		    unsigned long deadline, bool until_acked)
{
	int ret;

	do {
		ret = __stp_recv(w, payload, max, len, deadline,
				 until_acked ? STP_UNTIL_ACKED : STP_UNTIL_FRAME);
	} while (ret == 2);
	return ret;
}

/* a WMT event nobody asked for (or one that arrived after its command gave up) */
static void wmt_unsolicited(struct consys *cs, const u8 *evt, size_t len)
{
	struct consys_wmt *w = &cs->wmt;

	w->unsol++;
	w->unsol_len = min(len, sizeof(w->unsol_evt));
	memcpy(w->unsol_evt, evt, w->unsol_len);
	wmt_trc(w, 'M', "unsolicited WMT event", NULL, 0, 0);
	dev_info_ratelimited(cs->dev, "WMT unsolicited event (%zu bytes) %*ph\n", len,
			     (int)min_t(size_t, len, 16), evt);
}

/* frames that arrive while no command waits: acknowledge and record them */
static void wmt_rx_work(struct work_struct *work)
{
	struct consys_wmt *w = container_of(work, struct consys_wmt, rx_work);
	struct consys *cs = container_of(w, struct consys, wmt);
	size_t len;

	mutex_lock(&w->stp_lock);
	/* a frame that has started arrives completely within microseconds */
	while (w->async && rx_count(w)) {
		int ret = __stp_recv(w, w->abuf, sizeof(w->abuf), &len,
				     jiffies + msecs_to_jiffies(50), STP_UNTIL_FRAME);

		if (ret == 0)
			wmt_unsolicited(cs, w->abuf, len);
		else if (ret != 2)
			break;
	}
	mutex_unlock(&w->stp_lock);
}

static unsigned int wmt_evt_timeout(void);

/*
 * Own frames that stay unacknowledged while nobody waits for frames (BT/GNSS data sent, consys
 * quiet): wait for the acks here, retransmitting like stp_core (stp_rx_byte()).
 */
static void stp_retx_work(struct work_struct *work)
{
	struct consys_wmt *w = container_of(to_delayed_work(work), struct consys_wmt, retx_work);
	struct consys *cs = container_of(w, struct consys, wmt);
	unsigned long deadline = jiffies + msecs_to_jiffies(wmt_evt_timeout());
	size_t len;
	int ret = 1;

	mutex_lock(&w->stp_lock);
	while (READ_ONCE(w->async) && w->full && stp_unacked(w)) {
		ret = __stp_recv(w, w->abuf, sizeof(w->abuf), &len, deadline, STP_UNTIL_ACKED);
		if (ret == 0)
			wmt_unsolicited(cs, w->abuf, len);
		else if (ret != 2)
			break;
	}
	if (ret < 0)
		dev_warn_ratelimited(cs->dev, "STP: own frames still unacknowledged: %d (rxack %u txseq %u)\n",
				     ret, w->rxack, w->txseq);
	mutex_unlock(&w->stp_lock);
}

/* wait until the STP window has room for one more own frame; stp_lock held */
static int stp_wait_room(struct consys *cs, unsigned long deadline)
{
	struct consys_wmt *w = &cs->wmt;
	size_t len;
	int ret;

	while (w->full && stp_unacked(w) >= STP_WINSIZE) {
		ret = __stp_recv(w, w->abuf, sizeof(w->abuf), &len, deadline, STP_UNTIL_ROOM);
		if (ret == 0)
			wmt_unsolicited(cs, w->abuf, len);
		else if (ret < 0)
			return ret;
	}
	return 0;
}

/* ---------------------------------------------------------------- WMT */

static unsigned int wmt_evt_timeout(void)
{
	return clamp(evt_timeout_ms, 100U, 10000U);
}

/*
 * A command is complete when its event is there and the consys has also acknowledged the command
 * frame. Normally the ack comes first (ack-only frame, then the event with the same ack). If the
 * event arrives without it, the consys did not take the frame into its sequence (e.g. "query stp"
 * during the mode switch); wait here, retransmitting, so that the next command does not start
 * behind a gap. Frames arriving meanwhile are unsolicited.
 */
static int stp_wait_acked(struct consys *cs, const char *name)
{
	struct consys_wmt *w = &cs->wmt;
	unsigned long deadline = jiffies + msecs_to_jiffies(wmt_evt_timeout());
	size_t len;
	int ret;

	if (!w->full || !stp_unacked(w))
		return 0;
	w->ack_waits++;
	for (;;) {
		ret = stp_recv(w, w->abuf, sizeof(w->abuf), &len, deadline, true);
		if (ret == 1)
			return 0;
		if (ret) {
			dev_err(cs->dev, "WMT %s: event but no STP ack: %d (rxack %u txseq %u, retransmissions %u)\n",
				name, ret, w->rxack, w->txseq, w->retx_rounds);
			return ret;
		}
		wmt_unsolicited(cs, w->abuf, len);
	}
}

/*
 * Send one WMT command and wait for its event. The event must match "evt" over "evt_len"
 * bytes, except where "skip" has a bit set (bit i = byte i ignored).
 */
static int wmt_wait_evt(struct consys *cs, const char *name, u8 opcode, const u8 *evt,
			size_t evt_len, u32 skip, unsigned long deadline)
{
	struct consys_wmt *w = &cs->wmt;
	size_t len, i;
	int ret;

	for (;;) {
		ret = stp_recv(w, w->evt, sizeof(w->evt), &len, deadline, false);
		if (ret) {
			dev_err(cs->dev, "WMT %s: no event: %d (rx %u bytes, %u polled, acks %u, irqs %u)\n",
				name, ret, w->rx_bytes, w->rx_polled, w->rx_acks, w->irq_total);
			return ret;
		}
		/* events are 02 <opcode> <len16> <payload>; anything else is not ours */
		if (len >= 2 && w->evt[0] == 0x02 && w->evt[1] == opcode)
			break;
		wmt_unsolicited(cs, w->evt, len);
	}
	w->evt_len = len;
	consys_wmt_step(cs, name);
	if (len < evt_len)
		goto mismatch;
	for (i = 0; i < evt_len && i < 32; i++)
		if (!(skip & BIT(i)) && w->evt[i] != evt[i])
			goto mismatch;
	ret = stp_wait_acked(cs, name);
	if (ret)
		return ret;
	dev_info(cs->dev, "WMT %s: ok, event %*ph\n", name, (int)min_t(size_t, len, 16), w->evt);
	return 0;
mismatch:
	dev_err(cs->dev, "WMT %s: unexpected event (%zu bytes) %*ph\n", name, len,
		(int)min_t(size_t, len, 16), w->evt);
	return -EPROTO;
}

static int wmt_cmd(struct consys *cs, const char *name, const u8 *cmd, size_t cmd_len,
		   const u8 *evt, size_t evt_len, u32 skip, unsigned int timeout_ms)
{
	struct consys_wmt *w = &cs->wmt;
	int ret;

	mutex_lock(&w->stp_lock);
	w->err_step = name;
	wmt_trc(w, 'M', name, NULL, 0, 0);
	/* BT or GNSS data may fill the STP window */
	ret = stp_wait_room(cs, jiffies + msecs_to_jiffies(timeout_ms));
	if (!ret)
		ret = stp_send(w, cmd, cmd_len);
	if (ret)
		dev_err(cs->dev, "WMT %s: send failed: %d\n", name, ret);
	else
		ret = wmt_wait_evt(cs, name, cmd[1], evt, evt_len, skip,
				   jiffies + msecs_to_jiffies(timeout_ms));
	mutex_unlock(&w->stp_lock);
	return ret;
}

#define WMT(name, cmd, evt, skip, tmo) \
	wmt_cmd(cs, name, cmd, sizeof(cmd), evt, sizeof(evt), skip, tmo)

static int wmt_reg_read(struct consys *cs, const char *name, u32 addr, u32 *val)
{
	u8 cmd[] = { 0x01, 0x08, 0x10, 0x00, 0x02, 0x01, 0x00, 0x01,
		     addr & 0xff, (addr >> 8) & 0xff, (addr >> 16) & 0xff, addr >> 24,
		     0x00, 0x00, 0x00, 0x00, 0xff, 0xff, 0x00, 0x00 };
	const u8 evt[] = { 0x02, 0x08, 0x0c, 0x00, 0x00, 0x00, 0x00, 0x01,
			   cmd[8], cmd[9], cmd[10], cmd[11] };
	int ret;

	ret = WMT(name, cmd, evt, 0, wmt_evt_timeout());
	if (!ret)
		*val = get_unaligned_le32(&cs->wmt.evt[12]);
	return ret;
}

/* MCU patch (soc1_0_patch_mcu_*_hdr.bin) by PDA download, as mtk_wcn_soc_pda_patch_dwn() */
static int wmt_pda_patch(struct consys *cs)
{
	static const u8 evt[] = { 0x02, 0x01, 0x01, 0x00, 0x00 };
	struct consys_wmt *w = &cs->wmt;
	unsigned int frag = clamp(pda_frag, 16U, 4096U);
	unsigned int delay_us = min(pda_frag_delay_us, 50000U);
	unsigned int wait_ms = min(pda_frag_wait_ms, 200U);
	const struct firmware *fw;
	u8 cmd[21] = { 0x01, 0x01, 0x11, 0x00, 0x00,
		       0, 0, 0, 0,		/* target address */
		       0, 0, 0, 0,		/* size */
		       0, 0, 0, 0,		/* scramble key */
		       0x2f, 0x02, 0x02, 0x00 };	/* configuration */
	u8 pda[WMT_PDA_HDR_LEN] = { 0 };
	char bt[16];
	const u8 *h, *body;
	u32 size, off, total;
	ktime_t t0;
	u16 sw;
	int ret;

	snprintf(w->patch_name, sizeof(w->patch_name), "%s_patch_mcu_%s_hdr.bin",
		 consys_fw_prefix(), consys_fw_variant());
	ret = request_firmware(&fw, w->patch_name, cs->dev);
	if (ret) {
		dev_err(cs->dev, "patch %s: request_firmware failed: %d\n", w->patch_name, ret);
		return ret;
	}
	ret = -EINVAL;
	if (fw->size <= WMT_PATCH_HDR_LEN + 0x1c)
		goto out;
	h = fw->data;
	sw = (h[0x16] << 8) | h[0x17];
	/* byte 0x18: (count << 4) | sequence, only single patch files */
	if (h[0x18] != 0x11) {
		dev_err(cs->dev, "patch %s: unsupported count/sequence 0x%02x\n", w->patch_name, h[0x18]);
		goto out;
	}
	if (consys_fw_vercheck() && sw != (cs->fw_ver & 0xffff)) {
		dev_err(cs->dev, "patch %s: SW version 0x%04x != chip FW version 0x%04x\n",
			w->patch_name, sw, cs->fw_ver & 0xffff);
		goto out;
	}
	body = h + WMT_PATCH_HDR_LEN;
	size = fw->size - WMT_PATCH_HDR_LEN;
	/* address bytes like the vendor launcher: { 0, hdr[0x19], hdr[0x1a], hdr[0x1b] } */
	cmd[5] = 0;
	cmd[6] = h[0x19];
	cmd[7] = h[0x1a];
	cmd[8] = h[0x1b];
	put_unaligned_le32(size, &cmd[9]);
	w->patch_size = size;
	dev_info(cs->dev, "patch %s: date %.15s sw 0x%04x addr %02x %02x %02x %02x size 0x%x\n",
		 w->patch_name, h, sw, cmd[5], cmd[6], cmd[7], cmd[8], size);

	ret = WMT("PDA config", cmd, evt, 0, wmt_evt_timeout());
	if (ret)
		goto out;

	/* raw (not STP framed): 12 byte length header, then the body in fragments */
	mutex_lock(&w->stp_lock);
	w->err_step = "PDA data";
	t0 = ktime_get();
	total = size + WMT_PDA_HDR_LEN;
	put_unaligned_le32(total, pda);
	ret = btif_tx(w, pda, sizeof(pda));
	for (off = 0; !ret && off < size; off += frag) {
		ret = btif_tx(w, body + off, min_t(u32, frag, size - off));
		if (ret)
			break;
		w->pda_frags++;
		if (pda_frag_drain || wait_ms) {
			ret = btif_drain(w, BTIF_DRAIN_TIMEOUT_US);
			if (ret)
				break;
		}
		if (wait_ms) {
			size_t len;
			int r = stp_recv(w, w->evt, sizeof(w->evt), &len,
					 jiffies + msecs_to_jiffies(wait_ms), false);

			if (!r) {
				w->pda_frag_evts++;
				dev_info(cs->dev, "patch fragment %u: event %*ph\n", w->pda_frags,
					 (int)min_t(size_t, len, 16), w->evt);
			}
		}
		if (delay_us)
			fsleep(delay_us);
	}
	if (ret) {
		mutex_unlock(&w->stp_lock);
		dev_err(cs->dev, "patch %s: raw download failed at 0x%x: %d\n", w->patch_name, off, ret);
		goto out;
	}
	w->pda_us = ktime_us_delta(ktime_get(), t0);
	consys_wmt_step(cs, "PDA data queued");
	/* measure when the last byte has left; if the consys stops reading this times out */
	ret = btif_drain(w, BTIF_DRAIN_TIMEOUT_US);
	w->pda_drain_us = ktime_us_delta(ktime_get(), t0);
	mutex_unlock(&w->stp_lock);
	if (ret) {
		dev_err(cs->dev, "patch %s: data not drained after %u us (LSR 0x%02x)\n",
			w->patch_name, w->pda_drain_us, btif_rd(w, BTIF_LSR));
		goto out;
	}
	consys_wmt_step(cs, "PDA data drained");
	dev_info(cs->dev, "patch %s: %u fragments of %u bytes sent in %u us (drained after %u us), %u fragment events\n",
		 w->patch_name, w->pda_frags, frag, w->pda_us, w->pda_drain_us, w->pda_frag_evts);

	/* debug copy of the build time in the EMI, byte 15 cleared (vendor: 0x68330) */
	memcpy(bt, h, sizeof(bt));
	bt[15] = '\0';
	memcpy_toio(cs->emi + WMT_PATCH_BUILDTIME_OFF, bt, sizeof(bt));
	ret = 0;
out:
	release_firmware(fw);
	return ret;
}

static void wmt_time_cmd(u8 *cmd)
{
	struct timespec64 ts;

	ktime_get_real_ts64(&ts);
	cmd[0] = 0x01;
	cmd[1] = 0xf0;
	cmd[2] = 0x09;
	cmd[3] = 0x00;
	cmd[4] = 0x02;
	put_unaligned_le32((u32)ts.tv_sec, &cmd[5]);
	put_unaligned_le32((u32)(ts.tv_nsec / NSEC_PER_USEC), &cmd[9]);
}

static int wmt_reset(struct consys *cs)
{
	static const u8 c_reset[] = { 0x01, 0x07, 0x01, 0x00, 0x04 };
	static const u8 e_reset[] = { 0x02, 0x07, 0x01, 0x00, 0x00 };
	struct consys_wmt *w = &cs->wmt;
	unsigned int pad = min(reset_pad, 4096U);
	int ret;

	if (pda_reset_delay_ms)
		msleep(min(pda_reset_delay_ms, 1000U));
	if (wak_pulse & 2)
		btif_wak_pulse(w);
	ret = WMT("reset", c_reset, e_reset, 0, wmt_evt_timeout());
	if (ret != -ETIMEDOUT || !pad)
		return ret;

	/*
	 * Diagnosis only: if the consys lost patch bytes it still waits for data. Fill up with
	 * zeros and ask again; an answer now means bytes were missing (the patch is then broken).
	 */
	dev_warn(cs->dev, "WMT reset: sending %u padding bytes and retrying once\n", pad);
	memset(w->evt, 0, min_t(size_t, pad, sizeof(w->evt)));
	while (pad) {
		unsigned int n = min_t(unsigned int, pad, sizeof(w->evt));

		mutex_lock(&w->stp_lock);
		ret = btif_tx(w, w->evt, n);
		mutex_unlock(&w->stp_lock);
		if (ret)
			return ret;
		pad -= n;
	}
	ret = WMT("reset after padding", c_reset, e_reset, 0, wmt_evt_timeout());
	if (!ret) {
		dev_err(cs->dev, "WMT reset answered only after padding: patch bytes were lost\n");
		ret = -EIO;
	}
	return ret;
}

/* the start-up sequence of the vendor driver ("LPBK func on"), in trace order */
static int wmt_startup(struct consys *cs)
{
	static const u8 c_stp_q[] = { 0x01, 0x04, 0x01, 0x00, 0x04 };
	static const u8 e_stp_q[] = { 0x02, 0x04, 0x06, 0x00, 0x00, 0x04 };
	static const u8 c_stp_set[] = { 0x01, 0x04, 0x05, 0x00, 0x03, 0xdb, 0x0e, 0x68, 0x01 };
	static const u8 e_stp_set[] = { 0x02, 0x04, 0x02, 0x00, 0x00, 0x03 };
	static const u8 e_stp_q2[] = { 0x02, 0x04, 0x06, 0x00, 0x00, 0x04, 0xdb, 0x0e, 0x68, 0x01 };
	static const u8 c_chipid[] = { 0x01, 0x02, 0x05, 0x00, 0x11, 0x85, 0x67, 0x00, 0x00 };
	static const u8 e_ok02[] = { 0x02, 0x02, 0x01, 0x00, 0x00 };
	static const u8 c_lte[] = { 0x01, 0x10, 0x02, 0x00, 0x15, 0x01 };
	static const u8 e_ok10[] = { 0x02, 0x10, 0x01, 0x00, 0x00 };
	static const u8 c_tssi[] = { 0x01, 0x02, 0x04, 0x00, 0x10, 0x03, 0xc0, 0x0e };
	static const u8 c_ant[] = { 0x01, 0x14, 0x03, 0x00, 0x06, 0x02, 0x00 };
	static const u8 e_ant[] = { 0x02, 0x14, 0x02, 0x00, 0x00, 0x06 };
	static const u8 c_cal1[] = { 0x01, 0x14, 0x01, 0x00, 0x01 };
	static const u8 e_cal1[] = { 0x02, 0x14, 0x00, 0x00, 0x00, 0x01 };
	static const u8 c_cal3[] = { 0x01, 0x14, 0x01, 0x00, 0x03 };
	static const u8 e_cal3[] = { 0x02, 0x14, 0x00, 0x00, 0x00, 0x03 };
	static const u8 c_coex1e[] = { 0x01, 0x10, 0x04, 0x00, 0x1e, 0x00, 0x00, 0x00 };
	static const u8 c_coex1f[] = { 0x01, 0x10, 0x04, 0x00, 0x1f, 0x00, 0x00, 0x00 };
	static const u8 c_coex20[] = { 0x01, 0x10, 0x04, 0x00, 0x20, 0x00, 0x00, 0x00 };
	static const u8 c_fm[] = { 0x01, 0x05, 0x02, 0x00, 0x02, 0x02 };
	static const u8 e_fm[] = { 0x02, 0x05, 0x02, 0x00, 0x00, 0x02 };
	static const u8 e_time[] = { 0x02, 0xf0, 0x02, 0x00, 0x02, 0x00 };
	static const u8 c_f0_03[] = { 0x01, 0xf0, 0x02, 0x00, 0x03, 0x00 };
	static const u8 e_f0_03[] = { 0x02, 0xf0, 0x02, 0x00, 0x03, 0x00 };
	struct consys_wmt *w = &cs->wmt;
	unsigned int tmo = wmt_evt_timeout();
	u8 c_time[13];
	int ret;

	/* mandatory STP mode */
	w->full = false;
	ret = wmt_reg_read(cs, "read 0x80000008", 0x80000008, &w->chip_reg[0]);
	if (!ret)
		ret = wmt_reg_read(cs, "read 0x80000000", 0x80000000, &w->chip_reg[1]);
	if (!ret)
		ret = wmt_reg_read(cs, "read 0x80000004", 0x80000004, &w->chip_reg[2]);
	if (ret)
		return ret;
	dev_info(cs->dev, "WMT chip regs: 0x80000008=0x%08x 0x80000000=0x%08x 0x80000004=0x%08x\n",
		 w->chip_reg[0], w->chip_reg[1], w->chip_reg[2]);

	ret = wmt_pda_patch(cs);
	if (!ret)
		ret = wmt_reset(cs);
	/* bytes 6..9 of the default STP config are only logged */
	if (!ret)
		ret = WMT("query stp default", c_stp_q, e_stp_q, 0, tmo);
	if (!ret)
		ret = WMT("set stp", c_stp_set, e_stp_set, 0, tmo);
	if (ret)
		return ret;

	/* full STP mode from here on; give the consys time to switch as well (see stp_switch_ms) */
	stp_full_mode(w);
	if (stp_switch_ms)
		msleep(min(stp_switch_ms, 100U));
	consys_wmt_step(cs, "STP full mode");
	ret = WMT("query stp", c_stp_q, e_stp_q2, 0, tmo);
	if (!ret)
		ret = WMT("set chip id", c_chipid, e_ok02, 0, tmo);
	if (!ret)
		ret = WMT("set LTE project", c_lte, e_ok10, 0, tmo);
	if (!ret)
		ret = WMT("bt tssi from wifi", c_tssi, e_ok02, 0, tmo);
	if (!ret)
		ret = WMT("ant swap", c_ant, e_ant, 0, tmo);
	/* RF calibration: bytes 2/3 (length) vary, the data is kept for the WLAN driver */
	if (!ret)
		ret = WMT("RF calibration start", c_cal1, e_cal1, BIT(2) | BIT(3), WMT_CAL_TIMEOUT_MS);
	if (!ret) {
		w->cal_len[0] = min_t(size_t, w->evt_len, sizeof(w->cal[0]));
		memcpy(w->cal[0], w->evt, w->cal_len[0]);
		ret = WMT("RF calibration data", c_cal3, e_cal3, BIT(2) | BIT(3), WMT_CAL_TIMEOUT_MS);
	}
	if (ret)
		return ret;
	w->cal_len[1] = min_t(size_t, w->evt_len, sizeof(w->cal[1]));
	memcpy(w->cal[1], w->evt, w->cal_len[1]);

	/* efuse no longer needed: VCN33_2 off like the vendor driver after calibration */
	consys_vcn33_2_off(cs);
	consys_wmt_step(cs, "VCN33_2 off");

	ret = WMT("coex bt ctrl", c_coex1e, e_ok10, 0, tmo);
	if (!ret)
		ret = WMT("coex opp time ratio", c_coex1f, e_ok10, 0, tmo);
	if (!ret)
		ret = WMT("coex ble scan ratio", c_coex20, e_ok10, 0, tmo);
	if (!ret)
		ret = WMT("FM comm", c_fm, e_fm, 0, tmo);
	if (!ret) {
		wmt_time_cmd(c_time);
		ret = WMT("time sync", c_time, e_time, 0, tmo);
	}
	if (!ret)
		ret = WMT("f0 03", c_f0_03, e_f0_03, 0, tmo);
	if (!ret) {
		wmt_time_cmd(c_time);
		ret = WMT("time sync 2", c_time, e_time, 0, tmo);
	}
	return ret;
}

/* capture what can explain a failure while the BTIF clock and the consys are still on */
static void wmt_fail_snapshot(struct consys *cs)
{
	struct consys_wmt *w = &cs->wmt;
	unsigned int i;

	if (!w->clk_on)
		return;
	btif_poll_rx(w);
	w->fail_rx_pending = rx_count(w);
	/* like the vendor dump: one byte per register, RBR (offset 0) is not read */
	w->fail_btif[0] = 0;
	for (i = 1; i < CONSYS_BTIF_REGS; i++)
		w->fail_btif[i] = readb(w->btif + 4 * i);
	if (w->rxdma_on) {
		w->fail_rxdma[0] = rxdma_rd(w, RXDMA_EN);
		w->fail_rxdma[1] = rxdma_rd(w, RXDMA_INT_FLAG);
		w->fail_rxdma[2] = rxdma_rd(w, RXDMA_VFF_WPT);
		w->fail_rxdma[3] = rxdma_rd(w, RXDMA_VFF_RPT);
		w->fail_rxdma[4] = rxdma_rd(w, RXDMA_VFF_VALID);
		w->fail_rxdma[5] = rxdma_rd(w, RXDMA_VFF_LEFT);
		w->fail_rxdma[6] = rxdma_rd(w, RXDMA_DEBUG);
	}
	if (w->dma_on) {
		w->fail_dma[0] = txdma_rd(w, TXDMA_EN);
		w->fail_dma[1] = txdma_rd(w, TXDMA_VFF_WPT);
		w->fail_dma[2] = txdma_rd(w, TXDMA_VFF_RPT);
		w->fail_dma[3] = txdma_rd(w, TXDMA_VFF_VALID);
		w->fail_dma[4] = txdma_rd(w, TXDMA_VFF_LEFT);
		w->fail_dma[5] = txdma_rd(w, TXDMA_DEBUG);
	}
	consys_conn_sample(cs, &w->fail_boot, &w->fail_16c, &w->fail_coclk, w->fail_pc,
			   CONSYS_PC_SAMPLES);
	w->fail_valid = true;
	dev_err(cs->dev, "WMT failure in '%s': LSR 0x%02x HANDSHAKE 0x%02x WAK 0x%02x, rx pending %u, CONN 0x18002600 0x%08x 0x1800216c 0x%08x CPUPCR 0x%08x..0x%08x\n",
		w->err_step ? w->err_step : "?", w->fail_btif[5], w->fail_btif[27], w->fail_btif[25],
		w->fail_rx_pending, w->fail_boot, w->fail_16c, w->fail_pc[0],
		w->fail_pc[CONSYS_PC_SAMPLES - 1]);
}

int consys_wmt_start(struct consys *cs)
{
	struct consys_wmt *w = &cs->wmt;
	unsigned long flags;
	int ret;

	w->nsteps = 0;
	w->t_start = ktime_get();
	spin_lock_irqsave(&w->trc_lock, flags);
	w->trc_head = w->trc_count = 0;
	spin_unlock_irqrestore(&w->trc_lock, flags);
	w->rx_junk = w->rx_bad_hdr = w->rx_bad_crc = w->rx_acks = w->rx_other = 0;
	w->tx_bytes = w->tx_waits = w->tx_wait_max_us = w->tx_lag_timeouts = 0;
	w->pda_frags = w->pda_frag_evts = w->pda_us = w->pda_drain_us = 0;
	w->rx_frames = w->rx_dup = w->unsol = 0;
	w->rx_oos = w->rx_stale_ack = w->retx_rounds = w->retx_frames = w->ack_waits = 0;
	w->full = false;
	w->unsol_len = 0;
	w->psm_sleeps = w->psm_wakeups = 0;
	w->bt_rx_frames = w->bt_rx_bytes = w->bt_tx_frames = w->bt_tx_bytes = w->bt_tx_errors = 0;
	w->gps_rx_frames = 0;
	w->gps_rx_bytes = 0;
	w->gps_tx_frames = 0;
	w->gps_tx_bytes = 0;
	w->gps_tx_errors = 0;
	w->asleep = false;
	w->cal_len[0] = w->cal_len[1] = 0;
	w->evt_len = 0;
	w->err = 0;
	w->err_step = "BTIF open";
	w->fail_valid = false;

	ret = btif_open(cs);
	if (ret) {
		dev_err(cs->dev, "BTIF open failed: %d\n", ret);
		w->err = ret;
		return ret;
	}
	consys_wmt_step(cs, w->dma_on ? "BTIF open (TX DMA)" : "BTIF open (PIO)");
	if (wak_pulse & 1)
		btif_wak_pulse(w);
	/* vendor: about 7 ms between BTIF open and the first WMT command */
	usleep_range(5000, 7000);
	ret = wmt_startup(cs);
	if (ret) {
		w->err = ret;
		wmt_fail_snapshot(cs);
		consys_wmt_close(cs);
		return ret;
	}
	w->ready = true;
	w->err_step = NULL;
	WRITE_ONCE(w->async, true);
	/* anything that came in after the last event */
	schedule_work(&w->rx_work);
	dev_info(cs->dev, "WMT start-up done in %lld us (tx %u, rx %u bytes)\n",
		 ktime_us_delta(ktime_get(), w->t_start), w->tx_bytes, w->rx_bytes);
	return 0;
}

/* ---------------------------------------------------------------- STP power saving */

static const u8 c_sleep[] = { 0x01, 0x03, 0x01, 0x00, 0x01 };
static const u8 e_sleep[] = { 0x02, 0x03, 0x02, 0x00, 0x00, 0x01 };
static const u8 c_host_awake[] = { 0x01, 0x03, 0x01, 0x00, 0x02 };
static const u8 e_host_awake[] = { 0x02, 0x03, 0x02, 0x00, 0x00, 0x02 };
static const u8 e_wakeup[] = { 0x02, 0x03, 0x02, 0x00, 0x00, 0x03 };

/*
 * Vendor opfunc_pwr_sv(): SLEEP = WMT 03/01; WAKEUP over BTIF (fullset mode) = BTIF_WAK pulse,
 * then the consys reports 03/03 without a command; HOST_AWAKE = WMT 03/02 (answer to a wake-up
 * the consys started). The driver itself never puts the consys to sleep (no idle timer and no
 * BGF wake-up interrupt yet); these are for tests and for callers that need it awake.
 */
int consys_wmt_psm(struct consys *cs, enum consys_psm op)
{
	struct consys_wmt *w = &cs->wmt;
	int ret;

	if (!w->ready)
		return -ENODEV;
	switch (op) {
	case CONSYS_PSM_SLEEP:
		if (w->asleep)
			return 0;
		/* nothing wakes the consys for BT/GNSS traffic yet (no BGF wake-up interrupt) */
		if (w->bt_ops || w->gps_ops)
			return -EBUSY;
		ret = WMT("sleep", c_sleep, e_sleep, 0, wmt_evt_timeout());
		if (!ret) {
			w->asleep = true;
			w->psm_sleeps++;
		}
		return ret;
	case CONSYS_PSM_WAKEUP:
		if (!w->asleep)
			return 0;
		mutex_lock(&w->stp_lock);
		w->err_step = "wakeup";
		btif_wak_pulse(w);
		ret = wmt_wait_evt(cs, "wakeup", 0x03, e_wakeup, sizeof(e_wakeup), 0,
				   jiffies + msecs_to_jiffies(wmt_evt_timeout()));
		mutex_unlock(&w->stp_lock);
		if (!ret) {
			w->asleep = false;
			w->psm_wakeups++;
		}
		return ret;
	case CONSYS_PSM_HOST_AWAKE:
		ret = WMT("host awake", c_host_awake, e_host_awake, 0, wmt_evt_timeout());
		if (!ret)
			w->asleep = false;
		return ret;
	}
	return -EINVAL;
}

/* ---------------------------------------------------------------- function control */

/*
 * Receiver of the STP frames of a function (BT or GPS); installed before "on" so that nothing
 * the function sends afterwards is lost, removed before "off".
 */
static void wmt_func_receiver(struct consys_wmt *w, u8 type, const void *ops, void *priv)
{
	mutex_lock(&w->stp_lock);
	if (type == STP_TYPE_BT) {
		w->bt_ops = ops;
		w->bt_priv = priv;
	} else {
		w->gps_ops = ops;
		w->gps_priv = priv;
	}
	mutex_unlock(&w->stp_lock);
}

struct wmt_func {
	u8 type;
	const char *on, *off, *sync, *name;
};

static const struct wmt_func wmt_func_bt = {
	STP_TYPE_BT, "BT func on", "BT func off", "time sync (BT on)", "BT"
};

static const struct wmt_func wmt_func_gps = {
	STP_TYPE_GPS, "GPS func on", "GPS func off", "time sync (GPS on)", "GPS"
};

/*
 * WMT function control (vendor wmt_core_func_ctrl_cmd()): 01 06 02 00 <type> <on>, event
 * 02 06 01 00 <status>; after a function is on the vendor sends a UTC time sync.
 * cs->lock held, consys started and awake.
 */
static int wmt_func_ctrl(struct consys *cs, const struct wmt_func *f, bool on, const void *ops,
			 void *priv, bool *state)
{
	static const u8 e_func[] = { 0x02, 0x06, 0x01, 0x00, 0x00 };
	static const u8 e_time[] = { 0x02, 0xf0, 0x02, 0x00, 0x02, 0x00 };
	const u8 c_func[] = { 0x01, 0x06, 0x02, 0x00, f->type, on };
	struct consys_wmt *w = &cs->wmt;
	u8 c_time[13];
	int ret;

	if (!w->ready)
		return -ENODEV;
	if (on) {
		wmt_func_receiver(w, f->type, ops, priv);
		ret = WMT(f->on, c_func, e_func, 0, wmt_evt_timeout());
		if (ret) {
			wmt_func_receiver(w, f->type, NULL, NULL);
			return ret;
		}
		*state = true;
		wmt_time_cmd(c_time);
		if (WMT(f->sync, c_time, e_time, 0, wmt_evt_timeout()))
			dev_warn(cs->dev, "time sync after %s on failed, %s stays on\n", f->name,
				 f->name);
		return 0;
	}

	/* no more frames to the function driver from here on */
	wmt_func_receiver(w, f->type, NULL, NULL);
	ret = 0;
	if (*state)
		ret = WMT(f->off, c_func, e_func, 0, wmt_evt_timeout());
	*state = false;
	if (ret)
		dev_err(cs->dev, "%s func off failed: %d\n", f->name, ret);
	return ret;
}

/*
 * Bluetooth (vendor wmt_func_bt_on(), function type 0). The vendor PALDO step around it
 * (consys_hw_bt_vcn33_ctrl) is empty on MT6785.
 */
int consys_wmt_bt(struct consys *cs, bool on, const struct connac1_bt_ops *ops, void *priv)
{
	return wmt_func_ctrl(cs, &wmt_func_bt, on, ops, priv, &cs->wmt.bt_on);
}

/*
 * GNSS (vendor wmt_func_gps_on(), function type 2). The steps around it (VCN33_2 as the
 * co-clock PALDO, the external LNA pin) are done by the callers; the GPS sync pin step of the
 * vendor is a no-op on MT6785 (no OPT_GPS_SYNC, no host sync pin).
 */
int consys_wmt_gps(struct consys *cs, bool on, const struct connac1_gps_ops *ops, void *priv)
{
	return wmt_func_ctrl(cs, &wmt_func_gps, on, ops, priv, &cs->wmt.gps_on);
}

/**
 * connac1_bt_send() - send one HCI packet to the BT part of the consys
 * @cs: consys with the BT function on
 * @pkt_type: H4 packet type (HCI_COMMAND_PKT, HCI_ACLDATA_PKT, ...), sent as first byte
 * @data: packet without the type byte
 * @len: length of @data
 *
 * Sleeps until the STP window has room and the frame has left the BTIF; the consys
 * acknowledges it later (retransmissions as stp_core). Process context only.
 */
int connac1_bt_send(struct consys *cs, u8 pkt_type, const u8 *data, size_t len)
{
	struct consys_wmt *w = &cs->wmt;
	int ret;

	mutex_lock(&w->stp_lock);
	if (!READ_ONCE(w->async) || !w->bt_on || !w->full) {
		ret = -ENODEV;
		goto out;
	}
	ret = stp_wait_room(cs, jiffies + msecs_to_jiffies(wmt_evt_timeout()));
	if (!ret)
		ret = stp_send_frame(w, STP_TYPE_BT, pkt_type, data, len);
	if (ret) {
		w->bt_tx_errors++;
		goto out;
	}
	w->bt_tx_frames++;
	w->bt_tx_bytes += len + 1;
	/* retransmit if the ack does not come and nobody waits for frames by then */
	mod_delayed_work(system_wq, &w->retx_work, stp_retx_timeout() + 1);
out:
	mutex_unlock(&w->stp_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(connac1_bt_send);

/**
 * connac1_gps_send() - send one block of the GNSS data stream as one STP frame
 * @cs: consys with the GNSS function on
 * @data: payload
 * @len: length of @data, at most connac1_gps_max_payload()
 *
 * Sleeps until the STP window has room and the frame has left the BTIF (vendor GPS_write()
 * returns -ENOSPC instead when the window is full). Process context only.
 */
int connac1_gps_send(struct consys *cs, const u8 *data, size_t len)
{
	struct consys_wmt *w = &cs->wmt;
	int ret;

	if (!len || len > CONSYS_STP_PAYLOAD)
		return -EINVAL;
	mutex_lock(&w->stp_lock);
	if (!READ_ONCE(w->async) || !w->gps_on || !w->full) {
		ret = -ENODEV;
		goto out;
	}
	ret = stp_wait_room(cs, jiffies + msecs_to_jiffies(wmt_evt_timeout()));
	if (!ret)
		ret = stp_send_frame(w, STP_TYPE_GPS, -1, data, len);
	if (ret) {
		w->gps_tx_errors++;
		goto out;
	}
	w->gps_tx_frames++;
	w->gps_tx_bytes += len;
	mod_delayed_work(system_wq, &w->retx_work, stp_retx_timeout() + 1);
out:
	mutex_unlock(&w->stp_lock);
	return ret;
}
EXPORT_SYMBOL_GPL(connac1_gps_send);

/* largest @len of connac1_gps_send() */
size_t connac1_gps_max_payload(void)
{
	return CONSYS_STP_PAYLOAD;
}
EXPORT_SYMBOL_GPL(connac1_gps_max_payload);

static ssize_t psm_write(struct file *file, const char __user *ubuf, size_t len, loff_t *ppos)
{
	struct consys *cs = file->private_data;
	char buf[16];
	enum consys_psm op;
	int ret;

	if (len >= sizeof(buf))
		return -EINVAL;
	if (copy_from_user(buf, ubuf, len))
		return -EFAULT;
	buf[len] = '\0';
	if (sysfs_streq(buf, "sleep"))
		op = CONSYS_PSM_SLEEP;
	else if (sysfs_streq(buf, "wakeup"))
		op = CONSYS_PSM_WAKEUP;
	else if (sysfs_streq(buf, "awake"))
		op = CONSYS_PSM_HOST_AWAKE;
	else
		return -EINVAL;
	mutex_lock(&cs->lock);
	ret = consys_wmt_psm(cs, op);
	mutex_unlock(&cs->lock);
	return ret ? ret : len;
}

static const struct file_operations psm_fops = {
	.owner = THIS_MODULE,
	.open = simple_open,
	.write = psm_write,
	.llseek = noop_llseek,
};

void consys_wmt_show(struct consys *cs, struct seq_file *s)
{
	struct consys_wmt *w = &cs->wmt;
	unsigned int i;

	seq_printf(s, "WMT: %s, last error %d%s%s, STP %s mode, txseq %u txack %u rxack %u unacked %u\n",
		   w->ready ? "ready" : "not ready", w->err, w->err ? " in " : "",
		   w->err && w->err_step ? w->err_step : "", w->full ? "full" : "mandatory",
		   w->txseq, w->txack, w->rxack, w->full ? stp_unacked(w) : 0);
	seq_printf(s, "  BTIF path: TX %s, RX %s (DT: tx-dma %s irq %d, rx-dma %s irq %d); RX DMA irqs %u, in-flight waits %u\n",
		   w->dma_on ? "AP DMA" : "PIO", w->rxdma_on ? "AP DMA" : "PIO",
		   w->txdma ? "yes" : "no", w->txdma_irq, w->rxdma ? "yes" : "no", w->rxdma_irq,
		   w->rxdma_irqs, w->rxdma_spins);
	if (w->rxdma_on)
		seq_printf(s, "  RX DMA EN 0x%x INT_EN 0x%x FLAG 0x%x WPT 0x%x RPT 0x%x VALID 0x%x LEFT 0x%x\n",
			   rxdma_rd(w, RXDMA_EN), rxdma_rd(w, RXDMA_INT_EN), rxdma_rd(w, RXDMA_INT_FLAG),
			   rxdma_rd(w, RXDMA_VFF_WPT), rxdma_rd(w, RXDMA_VFF_RPT),
			   rxdma_rd(w, RXDMA_VFF_VALID), rxdma_rd(w, RXDMA_VFF_LEFT));
	seq_printf(s, "  switches: rx_dma %d tx_mode %u tx_dma %d pda_frag %u pda_frag_delay_us %u pda_frag_drain %d pda_frag_wait_ms %u pda_reset_delay_ms %u evt_timeout_ms %u wak_pulse %u reset_pad %u stp_switch_ms %u stp_retx_ms %u\n",
		   rx_dma, tx_mode, tx_dma, pda_frag, pda_frag_delay_us, pda_frag_drain, pda_frag_wait_ms,
		   pda_reset_delay_ms, evt_timeout_ms, wak_pulse, reset_pad, stp_switch_ms, stp_retx_ms);
	seq_printf(s, "  STP reliability: retransmission rounds %u (%u frames), acks without progress %u, out of sequence %u, waits for a late ack %u\n",
		   w->retx_rounds, w->retx_frames, w->rx_stale_ack, w->rx_oos, w->ack_waits);
	seq_printf(s, "  BTIF tx %u rx %u bytes (%u polled), rx overflow %u junk %u bad hdr %u bad crc %u acks %u other %u\n",
		   w->tx_bytes, w->rx_bytes, w->rx_polled, w->rx_overflow, w->rx_junk, w->rx_bad_hdr,
		   w->rx_bad_crc, w->rx_acks, w->rx_other);
	seq_printf(s, "  STP frames %u, repeated %u; unsolicited WMT events %u; PSM %s, sleeps %u wakeups %u; wifi users %d\n",
		   w->rx_frames, w->rx_dup, w->unsol, w->asleep ? "asleep" : "awake", w->psm_sleeps,
		   w->psm_wakeups, w->wifi_users);
	seq_printf(s, "  BT: %s, rx %u frames %u bytes, tx %u frames %u bytes, tx errors %u\n",
		   w->bt_on ? "on" : (w->bt_ops ? "switching" : "off"), w->bt_rx_frames, w->bt_rx_bytes,
		   w->bt_tx_frames, w->bt_tx_bytes, w->bt_tx_errors);
	seq_printf(s, "  GPS: %s, rx %u frames %u bytes, tx %u frames %u bytes, tx errors %u\n",
		   w->gps_on ? "on" : (w->gps_ops ? "switching" : "off"), w->gps_rx_frames,
		   w->gps_rx_bytes, w->gps_tx_frames, w->gps_tx_bytes, w->gps_tx_errors);
	if (w->unsol_len)
		seq_printf(s, "  last unsolicited event: %*ph\n", (int)w->unsol_len, w->unsol_evt);
	seq_printf(s, "  irq %s, %u interrupts (%u without data), LSR bits seen 0x%02x, tx waits %u (max %u us), LSR lag timeouts %u\n",
		   w->irq_disabled ? "DISABLED (storm)" : "ok", w->irq_total, w->irq_empty,
		   w->lsr_seen, w->tx_waits, w->tx_wait_max_us, w->tx_lag_timeouts);
	seq_printf(s, "  patch download: %u fragments, %u fragment events, queued in %u us, drained after %u us\n",
		   w->pda_frags, w->pda_frag_evts, w->pda_us, w->pda_drain_us);
	if (w->clk_on)
		seq_printf(s, "  BTIF IER 0x%02x IIR 0x%02x LSR 0x%02x DMA_EN 0x%02x TRI_LVL 0x%02x WAK 0x%02x HANDSHAKE 0x%02x SLEEP_EN 0x%02x\n",
			   btif_rd(w, BTIF_IER), btif_rd(w, BTIF_IIR), btif_rd(w, BTIF_LSR),
			   btif_rd(w, BTIF_DMA_EN), btif_rd(w, BTIF_TRI_LVL), btif_rd(w, BTIF_WAK),
			   btif_rd(w, BTIF_HANDSHAKE), btif_rd(w, BTIF_SLEEP_EN));
	if (w->fail_valid) {
		seq_printf(s, "  at the failure ('%s'), before switching off:\n",
			   w->err_step ? w->err_step : "?");
		seq_printf(s, "    BTIF regs (byte per register from 0x00, like the vendor dump): %*ph\n",
			   CONSYS_BTIF_REGS, w->fail_btif);
		if (tx_dma && w->txdma)
			seq_printf(s, "    TX DMA EN 0x%x WPT 0x%x RPT 0x%x VALID 0x%x LEFT 0x%x DEBUG 0x%x\n",
				   w->fail_dma[0], w->fail_dma[1], w->fail_dma[2], w->fail_dma[3],
				   w->fail_dma[4], w->fail_dma[5]);
		if (rx_dma && w->rxdma)
			seq_printf(s, "    RX DMA EN 0x%x FLAG 0x%x WPT 0x%x RPT 0x%x VALID 0x%x LEFT 0x%x DEBUG 0x%x\n",
				   w->fail_rxdma[0], w->fail_rxdma[1], w->fail_rxdma[2], w->fail_rxdma[3],
				   w->fail_rxdma[4], w->fail_rxdma[5], w->fail_rxdma[6]);
		seq_printf(s, "    rx bytes pending %u; CONN 0x18002600 0x%08x 0x1800216c 0x%08x 0x180c1204 0x%08x\n",
			   w->fail_rx_pending, w->fail_boot, w->fail_16c, w->fail_coclk);
		seq_puts(s, "    CPUPCR samples (0.5 ms apart):");
		for (i = 0; i < CONSYS_PC_SAMPLES; i++)
			seq_printf(s, " %08x", w->fail_pc[i]);
		seq_putc(s, '\n');
	}
	seq_printf(s, "  chip regs via WMT: 0x80000008=0x%08x 0x80000000=0x%08x 0x80000004=0x%08x\n",
		   w->chip_reg[0], w->chip_reg[1], w->chip_reg[2]);
	seq_printf(s, "  MCU patch %s size 0x%x, calibration events %zu/%zu bytes\n",
		   w->patch_name, w->patch_size, w->cal_len[0], w->cal_len[1]);
	if (w->evt_len)
		seq_printf(s, "  last event (%zu bytes): %*ph\n", w->evt_len,
			   (int)min_t(size_t, w->evt_len, 32), w->evt);
	seq_puts(s, "  steps (us since BTIF open, tx/rx bytes, interrupts):\n");
	for (i = 0; i < w->nsteps; i++)
		seq_printf(s, "    %8lld %6u %5u %5u %s\n", w->steps[i].us, w->steps[i].tx,
			   w->steps[i].rx, w->steps[i].irqs, w->steps[i].name);
}

void consys_wmt_debugfs(struct consys *cs)
{
	debugfs_create_file("btif_trace", 0444, cs->debugfs, cs, &btif_trace_fops);
	debugfs_create_file("psm", 0200, cs->debugfs, cs, &psm_fops);
}

int consys_wmt_init(struct consys *cs, struct platform_device *pdev)
{
	struct consys_wmt *w = &cs->wmt;
	struct device *dev = &pdev->dev;
	struct resource *res;

	spin_lock_init(&w->rx_lock);
	spin_lock_init(&w->trc_lock);
	mutex_init(&w->stp_lock);
	INIT_WORK(&w->rx_work, wmt_rx_work);
	INIT_DELAYED_WORK(&w->retx_work, stp_retx_work);
	init_waitqueue_head(&w->rx_wq);
	w->btif = devm_platform_ioremap_resource_byname(pdev, "btif");
	if (IS_ERR(w->btif))
		return PTR_ERR(w->btif);
	w->btif_clk = devm_clk_get(dev, "btif");
	if (IS_ERR(w->btif_clk))
		return dev_err_probe(dev, PTR_ERR(w->btif_clk), "no btif clock\n");
	w->irq = platform_get_irq_byname(pdev, "btif");
	if (w->irq < 0)
		return w->irq;
	w->trc = devm_kcalloc(dev, CONSYS_TRC_N, sizeof(*w->trc), GFP_KERNEL);
	if (!w->trc)
		return -ENOMEM;

	/* optional: AP DMA TX and RX channels (tx_dma, rx_dma); without them only PIO */
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "btif-tx-dma");
	if (!res)
		return 0;
	w->dma_clk = devm_clk_get(dev, "apdma");
	if (IS_ERR(w->dma_clk))
		return dev_err_probe(dev, PTR_ERR(w->dma_clk), "no apdma clock\n");
	if (dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32)))
		return dev_err_probe(dev, -EIO, "no 32 bit DMA\n");
	w->txdma = devm_ioremap_resource(dev, res);
	if (IS_ERR(w->txdma))
		return PTR_ERR(w->txdma);
	w->vff = dmam_alloc_coherent(dev, TXDMA_VFF_SIZE, &w->vff_dma, GFP_KERNEL);
	if (!w->vff)
		return -ENOMEM;
	/* the TX channel is drained by polling; its interrupt is only recorded */
	w->txdma_irq = platform_get_irq_byname_optional(pdev, "btif-tx-dma");

	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "btif-rx-dma");
	if (!res)
		return 0;
	w->rxdma_irq = platform_get_irq_byname_optional(pdev, "btif-rx-dma");
	if (w->rxdma_irq < 0) {
		dev_warn(dev, "btif-rx-dma without interrupt, RX stays PIO\n");
		return 0;
	}
	w->rxdma = devm_ioremap_resource(dev, res);
	if (IS_ERR(w->rxdma))
		return PTR_ERR(w->rxdma);
	w->rx_vff = dmam_alloc_coherent(dev, RXDMA_VFF_SIZE, &w->rx_vff_dma, GFP_KERNEL);
	if (!w->rx_vff)
		return -ENOMEM;
	return 0;
}
