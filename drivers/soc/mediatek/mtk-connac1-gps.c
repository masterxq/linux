// SPDX-License-Identifier: GPL-2.0-only
/*
 * MediaTek CONNAC1 consys (MT6785): GNSS function, character device /dev/stpgps.
 *
 * The GNSS receiver of the consys has no NMEA output of its own: the positioning engine runs on the
 * application processor and exchanges an opaque data stream with the GNSS DSP. That stream travels
 * in STP frames of type GPS over the BTIF link owned by the consys driver. This driver only
 * switches the function on and off and passes the stream through, with the user space interface of
 * the vendor driver (stp_chrdev_gps.c of MediaTek's GPL-2.0 connectivity modules), so that the
 * vendor positioning engine can be used unchanged:
 *  - open: external LNA on, WMT function "GPS on" (one user at a time)
 *  - read: received frames as one byte stream (blocking, O_NONBLOCK gives -EAGAIN)
 *  - write: one STP frame per call (at most connac1_gps_max_payload() bytes are taken)
 *  - ioctl: the vendor command numbers (plain integers, no _IO encoding), see gps_ioctl()
 *  - release: LNA off, WMT function "GPS off"
 * An rfkill switch of type GPS switches the receiver off: a block powers an open device down at
 * once (reads and writes fail with -EIO, poll reports an error, the user closes and opens again),
 * and open() fails with -ERFKILL while blocked.
 * Independent implementation; ioctl numbers, their return conventions and the LNA sequence follow
 * the vendor driver, no vendor code copied.
 */

#include <linux/compat.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/gpio/consumer.h>
#include <linux/kfifo.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/rfkill.h>
#include <linux/slab.h>
#include <linux/soc/mediatek/mtk-connac1.h>
#include <linux/uaccess.h>
#include <linux/wait.h>

/* vendor stp_chrdev_gps.c COMBO_IOC_* */
#define GPS_IOC_STP_CTRL	0	/* refused by the vendor as well */
#define GPS_IOC_RAW_DATA	1	/* refused by the vendor as well */
#define GPS_IOC_IC_HW_VERSION	7	/* u32 to user */
#define GPS_IOC_IC_FW_VERSION	8	/* u32 to user */
#define GPS_IOC_D1_EFUSE_GET	9	/* returns 0 on MT6785 */
#define GPS_IOC_RTC_FLAG	10	/* returns the RTC low power flag (0) */
#define GPS_IOC_CO_CLOCK_FLAG	11	/* returns the co-clock flag */
#define GPS_IOC_TRIGGER_ASSERT	12
#define GPS_IOC_WMT_STATUS	13	/* 0: no chip reset */
#define GPS_IOC_TAKE_WAKELOCK	14
#define GPS_IOC_GIVE_WAKELOCK	15
#define GPS_IOC_GET_LNA_PIN	16	/* u32 to user */
#define GPS_IOC_HW_SUSPEND	18
#define GPS_IOC_HW_RESUME	19
#define GPS_IOC_LISTEN_RST_EVT	20	/* blocks until close */
#define GPS_IOC_GET_MD_STATUS	21

/* about 3 s of the DSP stream at the data rates of the vendor engine */
#define GPS_RX_FIFO		SZ_64K

struct cgps {
	struct device *dev;
	struct consys *cs;
	struct gpio_desc *lna;
	u32 lna_pin;
	struct miscdevice misc;
	struct connac1_gps_info info;

	struct mutex lock;		/* open/release/rfkill: open, powered, blocked */
	bool open;
	bool powered;			/* LNA and the WMT function on for the open file */
	bool blocked;			/* rfkill soft or hard block */
	bool dead;			/* powered down by rfkill under the open file */
	bool closing;			/* close() called on the open file: wake up waiters */
	struct rfkill *rfkill;
	wait_queue_head_t close_wq;

	struct mutex rd_lock;		/* the single consumer of rx */
	struct kfifo rx;		/* producer: cgps_recv() under the STP lock */
	wait_queue_head_t rx_wq;

	struct mutex wr_lock;		/* one writer at a time: txbuf */
	u8 *txbuf;
	size_t txmax;

	/* statistics (sysfs "stats") */
	u32 opens, rx_frames, rx_bytes, rx_dropped, tx_frames, tx_bytes, tx_errors;
};

static void cgps_recv(void *priv, const u8 *data, size_t len)
{
	struct cgps *g = priv;
	unsigned int n;

	n = kfifo_in(&g->rx, data, len);
	g->rx_frames++;
	g->rx_bytes += n;
	if (n < len) {
		g->rx_dropped += len - n;
		dev_warn_ratelimited(g->dev, "rx fifo full, %zu bytes dropped\n", len - n);
	}
	wake_up_interruptible(&g->rx_wq);
}

static const struct connac1_gps_ops cgps_ops = {
	.recv = cgps_recv,
};

static struct cgps *cgps_from_file(struct file *file)
{
	return container_of(file->private_data, struct cgps, misc);
}

static int cgps_open(struct inode *inode, struct file *file)
{
	struct cgps *g = cgps_from_file(file);
	int ret;

	mutex_lock(&g->lock);
	if (g->open) {
		ret = -EBUSY;
		goto out;
	}
	if (g->blocked) {
		ret = -ERFKILL;
		goto out;
	}
	/* consumer side is idle and the function is off: no producer either */
	kfifo_reset(&g->rx);
	/* vendor order: LNA on (gps_lna_state_oh), then the WMT function */
	gpiod_set_value_cansleep(g->lna, 1);
	ret = connac1_gps_on(g->cs, &cgps_ops, g, &g->info);
	if (ret) {
		gpiod_set_value_cansleep(g->lna, 0);
		goto out;
	}
	g->open = true;
	g->powered = true;
	g->dead = false;
	g->closing = false;
	g->opens++;
	ret = stream_open(inode, file);
	dev_info(g->dev, "on (hw 0x%04x fw 0x%04x, co-clock %d)\n", g->info.hw_ver,
		 g->info.fw_ver, g->info.co_clock);
out:
	mutex_unlock(&g->lock);
	return ret;
}

/* g->lock held: vendor order, LNA to its init state (low), then the WMT function off */
static void cgps_power_off(struct cgps *g)
{
	if (!g->powered)
		return;
	gpiod_set_value_cansleep(g->lna, 0);
	connac1_gps_off(g->cs);
	g->powered = false;
}

/* every close(): release LISTEN_RST_EVT and readers before the last reference goes */
static int cgps_flush(struct file *file, fl_owner_t id)
{
	struct cgps *g = cgps_from_file(file);

	WRITE_ONCE(g->closing, true);
	wake_up_interruptible(&g->close_wq);
	wake_up_interruptible(&g->rx_wq);
	return 0;
}

static int cgps_release(struct inode *inode, struct file *file)
{
	struct cgps *g = cgps_from_file(file);

	mutex_lock(&g->lock);
	cgps_power_off(g);
	g->open = false;
	g->dead = false;
	mutex_unlock(&g->lock);
	dev_info(g->dev, "off (rx %u bytes, %u dropped, tx %u bytes)\n", g->rx_bytes,
		 g->rx_dropped, g->tx_bytes);
	return 0;
}

static ssize_t cgps_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
{
	struct cgps *g = cgps_from_file(file);
	unsigned int copied;
	int ret;

	if (!count)
		return 0;
	if (mutex_lock_interruptible(&g->rd_lock))
		return -ERESTARTSYS;
	while (kfifo_is_empty(&g->rx)) {
		if (READ_ONCE(g->dead)) {
			ret = -EIO;
			goto out;
		}
		if (READ_ONCE(g->closing)) {
			ret = 0;
			goto out;
		}
		if (file->f_flags & O_NONBLOCK) {
			ret = -EAGAIN;
			goto out;
		}
		ret = wait_event_interruptible(g->rx_wq, !kfifo_is_empty(&g->rx) ||
					       READ_ONCE(g->closing) || READ_ONCE(g->dead));
		if (ret)
			goto out;
	}
	ret = kfifo_to_user(&g->rx, buf, count, &copied);
	if (!ret)
		ret = copied;
out:
	mutex_unlock(&g->rd_lock);
	return ret;
}

static ssize_t cgps_write(struct file *file, const char __user *buf, size_t count,
			  loff_t *ppos)
{
	struct cgps *g = cgps_from_file(file);
	size_t n = min(count, g->txmax);
	int ret;

	if (!count)
		return -EINVAL;
	if (mutex_lock_interruptible(&g->wr_lock))
		return -ERESTARTSYS;
	if (READ_ONCE(g->dead)) {
		ret = -EIO;
		goto out;
	}
	if (copy_from_user(g->txbuf, buf, n)) {
		ret = -EFAULT;
		goto out;
	}
	ret = connac1_gps_send(g->cs, g->txbuf, n);
	if (ret) {
		g->tx_errors++;
		/* vendor: no room in the STP window is -ENOSPC, the caller retries */
		if (ret == -EBUSY || ret == -ETIMEDOUT)
			ret = -ENOSPC;
		goto out;
	}
	g->tx_frames++;
	g->tx_bytes += n;
	ret = n;
out:
	mutex_unlock(&g->wr_lock);
	return ret;
}

static __poll_t cgps_poll(struct file *file, poll_table *wait)
{
	struct cgps *g = cgps_from_file(file);
	__poll_t mask = EPOLLOUT | EPOLLWRNORM;

	poll_wait(file, &g->rx_wq, wait);
	if (!kfifo_is_empty(&g->rx))
		mask |= EPOLLIN | EPOLLRDNORM;
	if (READ_ONCE(g->closing))
		mask |= EPOLLHUP;
	if (READ_ONCE(g->dead))
		mask |= EPOLLERR | EPOLLHUP;
	return mask;
}

static long cgps_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct cgps *g = cgps_from_file(file);
	u32 __user *up = (u32 __user *)arg;

	switch (cmd) {
	case GPS_IOC_STP_CTRL:
	case GPS_IOC_RAW_DATA:
		return -EINVAL;
	case GPS_IOC_IC_HW_VERSION:
		return put_user(g->info.hw_ver, up);
	case GPS_IOC_IC_FW_VERSION:
		return put_user(g->info.fw_ver, up);
	case GPS_IOC_D1_EFUSE_GET:
	case GPS_IOC_RTC_FLAG:
	case GPS_IOC_WMT_STATUS:
		return 0;
	case GPS_IOC_CO_CLOCK_FLAG:
		return g->info.co_clock;
	case GPS_IOC_TRIGGER_ASSERT:
		/* no consys assert/coredump path: the vendor's failure code */
		return -EBUSY;
	case GPS_IOC_TAKE_WAKELOCK:
	case GPS_IOC_GIVE_WAKELOCK:
		/* nothing suspends while the consys is on (no consys PSM yet) */
		return 0;
	case GPS_IOC_GET_LNA_PIN:
		return put_user(g->lna_pin, up);
	case GPS_IOC_HW_SUSPEND:
	case GPS_IOC_HW_RESUME:
		/* vendor answer when the firmware does not support it */
		return -EINVAL;
	case GPS_IOC_LISTEN_RST_EVT:
		/* no chip reset path: wake up on close, an rfkill block (or a signal) */
		return wait_event_interruptible(g->close_wq, READ_ONCE(g->closing) ||
						READ_ONCE(g->dead));
	case GPS_IOC_GET_MD_STATUS:
		/* vendor without b13b14-status-addr in the DT, as on MT6785 */
		return -EFAULT;
	default:
		return -ENOTTY;
	}
}

static const struct file_operations cgps_fops = {
	.owner = THIS_MODULE,
	.open = cgps_open,
	.flush = cgps_flush,
	.release = cgps_release,
	.read = cgps_read,
	.write = cgps_write,
	.poll = cgps_poll,
	.unlocked_ioctl = cgps_ioctl,
	.compat_ioctl = compat_ptr_ioctl,
};

static ssize_t stats_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct cgps *g = dev_get_drvdata(dev);
	int n;

	n = sysfs_emit(buf, "open %d powered %d rfkill %d opens %u\n", g->open, g->powered,
		       g->blocked, g->opens);
	n += sysfs_emit_at(buf, n, "rx_frames %u rx_bytes %u rx_dropped %u rx_queued %u\n",
			   g->rx_frames, g->rx_bytes, g->rx_dropped, kfifo_len(&g->rx));
	n += sysfs_emit_at(buf, n, "tx_frames %u tx_bytes %u tx_errors %u\n",
			   g->tx_frames, g->tx_bytes, g->tx_errors);
	return n;
}
static DEVICE_ATTR_RO(stats);

static struct attribute *cgps_attrs[] = {
	&dev_attr_stats.attr,
	NULL
};
ATTRIBUTE_GROUPS(cgps);

/* rfkill core: rfkill_global_mutex held, then g->lock (open() never takes rfkill locks) */
static int cgps_rfkill_set_block(void *data, bool blocked)
{
	struct cgps *g = data;

	mutex_lock(&g->lock);
	g->blocked = blocked;
	if (blocked && g->powered) {
		cgps_power_off(g);
		WRITE_ONCE(g->dead, true);
		wake_up_interruptible(&g->rx_wq);
		wake_up_interruptible(&g->close_wq);
		dev_info(g->dev, "off (rfkill)\n");
	}
	mutex_unlock(&g->lock);
	return 0;
}

static const struct rfkill_ops cgps_rfkill_ops = {
	.set_block = cgps_rfkill_set_block,
};

static void cgps_rfkill_free(void *data)
{
	struct rfkill *rf = data;

	rfkill_unregister(rf);
	rfkill_destroy(rf);
}

static void cgps_put_consys(void *cs)
{
	connac1_put(cs);
}

static void cgps_free_fifo(void *fifo)
{
	kfifo_free((struct kfifo *)fifo);
}

static int cgps_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct of_phandle_args args;
	struct cgps *g;
	int ret;

	g = devm_kzalloc(dev, sizeof(*g), GFP_KERNEL);
	if (!g)
		return -ENOMEM;
	g->dev = dev;
	mutex_init(&g->lock);
	mutex_init(&g->rd_lock);
	mutex_init(&g->wr_lock);
	init_waitqueue_head(&g->rx_wq);
	init_waitqueue_head(&g->close_wq);
	platform_set_drvdata(pdev, g);

	g->cs = connac1_get(dev);
	if (IS_ERR(g->cs))
		return dev_err_probe(dev, PTR_ERR(g->cs), "no consys\n");
	ret = devm_add_action_or_reset(dev, cgps_put_consys, g->cs);
	if (ret)
		return ret;

	/* LNA low until the device is opened (vendor gps_lna_state_init) */
	g->lna = devm_gpiod_get(dev, "lna", GPIOD_OUT_LOW);
	if (IS_ERR(g->lna))
		return dev_err_probe(dev, PTR_ERR(g->lna), "no lna gpio\n");
	/* the engine asks for the pin number (vendor: from the pinmux of the LNA state) */
	ret = of_parse_phandle_with_args(dev->of_node, "lna-gpios", "#gpio-cells", 0, &args);
	if (ret)
		return dev_err_probe(dev, ret, "lna-gpios\n");
	of_node_put(args.np);
	g->lna_pin = args.args_count ? args.args[0] : 0;

	g->txmax = connac1_gps_max_payload();
	g->txbuf = devm_kmalloc(dev, g->txmax, GFP_KERNEL);
	if (!g->txbuf)
		return -ENOMEM;
	ret = kfifo_alloc(&g->rx, GPS_RX_FIFO, GFP_KERNEL);
	if (ret)
		return ret;
	ret = devm_add_action_or_reset(dev, cgps_free_fifo, &g->rx);
	if (ret)
		return ret;

	g->rfkill = rfkill_alloc(dev_name(dev), dev, RFKILL_TYPE_GPS, &cgps_rfkill_ops, g);
	if (!g->rfkill)
		return -ENOMEM;
	ret = rfkill_register(g->rfkill);
	if (ret) {
		rfkill_destroy(g->rfkill);
		return dev_err_probe(dev, ret, "rfkill\n");
	}
	ret = devm_add_action_or_reset(dev, cgps_rfkill_free, g->rfkill);
	if (ret)
		return ret;

	g->misc.minor = MISC_DYNAMIC_MINOR;
	g->misc.name = "stpgps";
	g->misc.fops = &cgps_fops;
	g->misc.parent = dev;
	g->misc.mode = 0600;
	ret = misc_register(&g->misc);
	if (ret)
		return dev_err_probe(dev, ret, "misc device\n");
	dev_info(dev, "/dev/stpgps ready (LNA GPIO %u, max write %zu bytes)\n", g->lna_pin,
		 g->txmax);
	return 0;
}

static void cgps_remove(struct platform_device *pdev)
{
	struct cgps *g = platform_get_drvdata(pdev);

	/* the open file holds the module: nothing is open here */
	misc_deregister(&g->misc);
}

static const struct of_device_id cgps_of_match[] = {
	{ .compatible = "mediatek,mt6785-gnss" },
	{ }
};
MODULE_DEVICE_TABLE(of, cgps_of_match);

static struct platform_driver cgps_driver = {
	.probe = cgps_probe,
	.remove = cgps_remove,
	.driver = {
		.name = "mtk-connac1-gps",
		.of_match_table = cgps_of_match,
		.dev_groups = cgps_groups,
		.suppress_bind_attrs = true,
	},
};
module_platform_driver(cgps_driver);

MODULE_DESCRIPTION("MediaTek CONNAC1 consys GNSS function (/dev/stpgps)");
MODULE_LICENSE("GPL");
