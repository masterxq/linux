// SPDX-License-Identifier: GPL-2.0-only
/*
 * Bluetooth HCI driver for the MediaTek CONNAC1 connectivity subsystem (MT6785 "soc1_0").
 *
 * The consys driver (drivers/soc/mediatek/mtk-connac1-*.c) owns the BTIF, the STP link and
 * the WMT control channel. This driver switches the Bluetooth function on and off through it
 * (WMT function control) and exchanges HCI packets in H4 format (type byte first) inside STP
 * frames of type BT, which is how MediaTek's GPL-2.0 vendor character device stp_chrdev_bt.c
 * (bt-mt66xx, wmt/) hands them to the Android stack. The BT RAM patch is placed in the consys
 * memory at power-on, so there is no firmware download here. Independent implementation, no
 * vendor code copied.
 *
 * The controller address: if the controller reports none (zero or all ones), a stable
 * locally administered one derived from the serial number of the bootloader command line is
 * set (bdaddr_mode=1, default) or the device stays unconfigured until user space sets one
 * (bdaddr_mode=0, "btmgmt public-addr"). The address is never logged.
 */

#include <linux/jhash.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/skbuff.h>
#include <linux/soc/mediatek/mtk-connac1.h>
#include <linux/string.h>
#include <linux/workqueue.h>

#include <net/bluetooth/bluetooth.h>
#include <net/bluetooth/hci_core.h>

#define VERSION "0.2"

static int bdaddr_mode = 1;
module_param(bdaddr_mode, int, 0644);
MODULE_PARM_DESC(bdaddr_mode, "controller without an address: 1 = set a stable locally administered one (default), 0 = stay unconfigured (btmgmt public-addr); 2 = always set the locally administered one");

/* diagnosis: further HCI_QUIRK_* numbers to set at probe, without rebuilding the driver */
static int quirks[8];
static int nquirks;
module_param_array(quirks, int, &nquirks, 0444);
MODULE_PARM_DESC(quirks, "additional HCI quirk numbers (enum in include/net/bluetooth/hci.h) set at probe, comma separated");

struct btmtkstp_dev {
	struct device *dev;
	struct hci_dev *hdev;
	struct consys *cs;
	bool on;

	struct sk_buff_head txq;
	struct work_struct tx_work;

	/* H4 reassembly; only touched from the receive callback (STP lock) and with BT off */
	struct sk_buff *rx_skb;
	bool rx_hdr_done;
};

/* ---------------------------------------------------------------- receive */

static void btmtkstp_rx_drop(struct btmtkstp_dev *bdev)
{
	kfree_skb(bdev->rx_skb);
	bdev->rx_skb = NULL;
	bdev->hdev->stat.err_rx++;
}

/* one STP frame of type BT: an H4 stream, normally exactly one packet */
static void btmtkstp_recv(void *priv, const u8 *data, size_t len)
{
	struct btmtkstp_dev *bdev = priv;
	struct hci_dev *hdev = bdev->hdev;

	hdev->stat.byte_rx += len;
	while (len) {
		struct sk_buff *skb = bdev->rx_skb;
		unsigned int dlen;
		size_t n;

		if (!skb) {
			unsigned int hlen;

			switch (data[0]) {
			case HCI_EVENT_PKT:
				hlen = HCI_EVENT_HDR_SIZE;
				break;
			case HCI_ACLDATA_PKT:
				hlen = HCI_ACL_HDR_SIZE;
				break;
			case HCI_SCODATA_PKT:
				hlen = HCI_SCO_HDR_SIZE;
				break;
			case HCI_ISODATA_PKT:
				hlen = HCI_ISO_HDR_SIZE;
				break;
			default:
				hdev->stat.err_rx++;
				bt_dev_err_ratelimited(hdev, "unknown packet type 0x%02x, %zu bytes dropped",
						       data[0], len);
				return;
			}
			skb = bt_skb_alloc(HCI_MAX_FRAME_SIZE, GFP_ATOMIC);
			if (!skb) {
				hdev->stat.err_rx++;
				return;
			}
			hci_skb_pkt_type(skb) = data[0];
			hci_skb_expect(skb) = hlen;
			bdev->rx_skb = skb;
			bdev->rx_hdr_done = false;
			data++;
			len--;
			continue;
		}

		n = min_t(size_t, hci_skb_expect(skb) - skb->len, len);
		skb_put_data(skb, data, n);
		data += n;
		len -= n;
		if (skb->len < hci_skb_expect(skb))
			continue;

		if (!bdev->rx_hdr_done) {
			switch (hci_skb_pkt_type(skb)) {
			case HCI_EVENT_PKT:
				dlen = hci_event_hdr(skb)->plen;
				break;
			case HCI_ACLDATA_PKT:
				dlen = __le16_to_cpu(hci_acl_hdr(skb)->dlen);
				break;
			case HCI_SCODATA_PKT:
				dlen = hci_sco_hdr(skb)->dlen;
				break;
			default:
				dlen = hci_iso_data_len(__le16_to_cpu(hci_iso_hdr(skb)->dlen));
				break;
			}
			if (dlen > skb_tailroom(skb)) {
				bt_dev_err_ratelimited(hdev, "packet type 0x%02x too long (%u)",
						       hci_skb_pkt_type(skb), dlen);
				btmtkstp_rx_drop(bdev);
				return;
			}
			bdev->rx_hdr_done = true;
			hci_skb_expect(skb) += dlen;
			if (dlen)
				continue;
		}

		bdev->rx_skb = NULL;
		hci_recv_frame(hdev, skb);
	}
}

static const struct connac1_bt_ops btmtkstp_ops = {
	.recv = btmtkstp_recv,
};

/* ---------------------------------------------------------------- send */

static void btmtkstp_tx_work(struct work_struct *work)
{
	struct btmtkstp_dev *bdev = container_of(work, struct btmtkstp_dev, tx_work);
	struct hci_dev *hdev = bdev->hdev;
	struct sk_buff *skb;
	int ret;

	while ((skb = skb_dequeue(&bdev->txq))) {
		ret = connac1_bt_send(bdev->cs, hci_skb_pkt_type(skb), skb->data, skb->len);
		if (ret) {
			hdev->stat.err_tx++;
			bt_dev_err_ratelimited(hdev, "send failed: %d", ret);
		} else {
			hdev->stat.byte_tx += skb->len + 1;
		}
		kfree_skb(skb);
	}
}

static int btmtkstp_send_frame(struct hci_dev *hdev, struct sk_buff *skb)
{
	struct btmtkstp_dev *bdev = hci_get_drvdata(hdev);

	switch (hci_skb_pkt_type(skb)) {
	case HCI_COMMAND_PKT:
		hdev->stat.cmd_tx++;
		break;
	case HCI_ACLDATA_PKT:
		hdev->stat.acl_tx++;
		break;
	case HCI_SCODATA_PKT:
		hdev->stat.sco_tx++;
		break;
	case HCI_ISODATA_PKT:
		break;
	default:
		return -EILSEQ;
	}

	skb_queue_tail(&bdev->txq, skb);
	schedule_work(&bdev->tx_work);
	return 0;
}

/* ---------------------------------------------------------------- open, close, setup */

static int btmtkstp_open(struct hci_dev *hdev)
{
	struct btmtkstp_dev *bdev = hci_get_drvdata(hdev);
	int ret;

	ret = connac1_bt_on(bdev->cs, &btmtkstp_ops, bdev);
	if (ret) {
		bt_dev_err(hdev, "BT function on failed: %d", ret);
		return ret;
	}
	bdev->on = true;
	return 0;
}

static int btmtkstp_close(struct hci_dev *hdev)
{
	struct btmtkstp_dev *bdev = hci_get_drvdata(hdev);

	cancel_work_sync(&bdev->tx_work);
	skb_queue_purge(&bdev->txq);
	if (bdev->on) {
		connac1_bt_off(bdev->cs);
		bdev->on = false;
	}
	/* the receive callback is gone now */
	kfree_skb(bdev->rx_skb);
	bdev->rx_skb = NULL;
	return 0;
}

static int btmtkstp_flush(struct hci_dev *hdev)
{
	struct btmtkstp_dev *bdev = hci_get_drvdata(hdev);

	skb_queue_purge(&bdev->txq);
	cancel_work_sync(&bdev->tx_work);
	return 0;
}

/* vendor command 0xfc1a, as btmtk_set_bdaddr() */
static int btmtkstp_set_bdaddr(struct hci_dev *hdev, const bdaddr_t *bdaddr)
{
	struct sk_buff *skb;

	skb = __hci_cmd_sync(hdev, 0xfc1a, sizeof(*bdaddr), bdaddr, HCI_INIT_TIMEOUT);
	if (IS_ERR(skb)) {
		bt_dev_err(hdev, "setting the address failed: %ld", PTR_ERR(skb));
		return PTR_ERR(skb);
	}
	kfree_skb(skb);
	return 0;
}

/* value of "key" in the command line the bootloader put in /chosen (as the WLAN glue) */
static bool btmtkstp_bootarg(const char *key, char *val, size_t len)
{
	const char *args = NULL, *p, *e;
	size_t klen = strlen(key), n;

	if (!of_chosen || of_property_read_string(of_chosen, "bootargs", &args) || !args)
		return false;
	for (p = args; (p = strstr(p, key)); p += klen) {
		if (p != args && p[-1] != ' ')
			continue;
		p += klen;
		e = strchrnul(p, ' ');
		n = min_t(size_t, e - p, len - 1);
		memcpy(val, p, n);
		val[n] = 0;
		return n > 0;
	}
	return false;
}

/* stable locally administered address from the serial number (other salts than WLAN) */
static bool btmtkstp_laa(bdaddr_t *ba)
{
	char buf[64];
	u32 h1, h2;

	if (!btmtkstp_bootarg("androidboot.serialno=", buf, sizeof(buf)))
		return false;
	h1 = jhash(buf, strlen(buf), 0x62746164);
	h2 = jhash(buf, strlen(buf), 0x6c697362);
	/* b[5] is the first octet of the written form: 02:xx:xx:xx:xx:xx */
	ba->b[5] = 0x02;
	ba->b[4] = h1 >> 24;
	ba->b[3] = h1 >> 16;
	ba->b[2] = h1 >> 8;
	ba->b[1] = h1;
	ba->b[0] = h2;
	return true;
}

static int btmtkstp_setup(struct hci_dev *hdev)
{
	static const bdaddr_t bdaddr_ones = {{ 0xff, 0xff, 0xff, 0xff, 0xff, 0xff }};
	struct hci_rp_read_local_version *ver;
	struct hci_rp_read_bd_addr *rp;
	struct sk_buff *skb;
	bool invalid;
	bdaddr_t ba;

	/* the BT part was just switched on: start from a reset controller */
	skb = __hci_cmd_sync(hdev, HCI_OP_RESET, 0, NULL, HCI_INIT_TIMEOUT);
	if (IS_ERR(skb)) {
		bt_dev_err(hdev, "HCI reset failed: %ld", PTR_ERR(skb));
		return PTR_ERR(skb);
	}
	kfree_skb(skb);

	skb = __hci_cmd_sync(hdev, HCI_OP_READ_LOCAL_VERSION, 0, NULL, HCI_INIT_TIMEOUT);
	if (IS_ERR(skb)) {
		bt_dev_err(hdev, "reading the version failed: %ld", PTR_ERR(skb));
		return PTR_ERR(skb);
	}
	if (skb->len >= sizeof(*ver)) {
		ver = (void *)skb->data;
		bt_dev_info(hdev, "HCI %u rev 0x%04x, LMP %u subver 0x%04x, manufacturer %u",
			    ver->hci_ver, __le16_to_cpu(ver->hci_rev), ver->lmp_ver,
			    __le16_to_cpu(ver->lmp_subver), __le16_to_cpu(ver->manufacturer));
	}
	kfree_skb(skb);

	skb = __hci_cmd_sync(hdev, HCI_OP_READ_BD_ADDR, 0, NULL, HCI_INIT_TIMEOUT);
	if (IS_ERR(skb)) {
		bt_dev_err(hdev, "reading the address failed: %ld", PTR_ERR(skb));
		return PTR_ERR(skb);
	}
	if (skb->len < sizeof(*rp)) {
		kfree_skb(skb);
		return -EIO;
	}
	rp = (void *)skb->data;
	bacpy(&ba, &rp->bdaddr);
	kfree_skb(skb);
	invalid = !bacmp(&ba, BDADDR_ANY) || !bacmp(&ba, &bdaddr_ones);

	if (!invalid && bdaddr_mode != 2) {
		bt_dev_info(hdev, "address from the controller");
		return 0;
	}
	if (bdaddr_mode == 0) {
		bt_dev_warn(hdev, "controller has no address, unconfigured (set one with btmgmt public-addr)");
		hci_set_quirk(hdev, HCI_QUIRK_INVALID_BDADDR);
		return 0;
	}
	if (!btmtkstp_laa(&ba)) {
		bt_dev_warn(hdev, "no serial number for a locally administered address, unconfigured");
		hci_set_quirk(hdev, HCI_QUIRK_INVALID_BDADDR);
		return 0;
	}
	bt_dev_info(hdev, "%s: stable locally administered address",
		    invalid ? "controller has no address" : "bdaddr_mode=2");
	return btmtkstp_set_bdaddr(hdev, &ba);
}

/* ---------------------------------------------------------------- probe */

static int btmtkstp_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct btmtkstp_dev *bdev;
	struct hci_dev *hdev;
	int i, ret;

	bdev = devm_kzalloc(dev, sizeof(*bdev), GFP_KERNEL);
	if (!bdev)
		return -ENOMEM;
	bdev->dev = dev;
	skb_queue_head_init(&bdev->txq);
	INIT_WORK(&bdev->tx_work, btmtkstp_tx_work);

	bdev->cs = connac1_get(dev);
	if (IS_ERR(bdev->cs))
		return dev_err_probe(dev, PTR_ERR(bdev->cs), "no consys\n");

	hdev = hci_alloc_dev();
	if (!hdev) {
		ret = -ENOMEM;
		goto err_put;
	}
	bdev->hdev = hdev;
	hdev->bus = HCI_UART;
	hci_set_drvdata(hdev, bdev);
	hdev->open = btmtkstp_open;
	hdev->close = btmtkstp_close;
	hdev->flush = btmtkstp_flush;
	hdev->setup = btmtkstp_setup;
	hdev->send = btmtkstp_send_frame;
	hdev->set_bdaddr = btmtkstp_set_bdaddr;
	hdev->manufacturer = 70;	/* MediaTek */
	/* BT off at close loses the controller state: setup again at every open */
	hci_set_quirk(hdev, HCI_QUIRK_NON_PERSISTENT_SETUP);
	/*
	 * Seen on the MT6785 consys (HCI 11, LMP subver 0xffd7): extended features page 2
	 * announces the synchronization train, but Read Synchronization Train Parameters
	 * (0x0c77) fails with Unknown Command and aborts the init. Do not read page 2.
	 */
	hci_set_quirk(hdev, HCI_QUIRK_BROKEN_LOCAL_EXT_FEATURES_PAGE_2);
	for (i = 0; i < nquirks; i++) {
		if (quirks[i] < 0 || quirks[i] >= __HCI_NUM_QUIRKS) {
			dev_warn(dev, "quirk %d out of range, ignored\n", quirks[i]);
			continue;
		}
		hci_set_quirk(hdev, quirks[i]);
		dev_info(dev, "quirk %d set (module parameter)\n", quirks[i]);
	}
	SET_HCIDEV_DEV(hdev, dev);
	platform_set_drvdata(pdev, bdev);

	ret = hci_register_dev(hdev);
	if (ret < 0) {
		dev_err(dev, "registering the HCI device failed: %d\n", ret);
		goto err_free;
	}
	dev_info(dev, "MediaTek CONNAC1 Bluetooth over STP, %s\n", hdev->name);
	return 0;

err_free:
	hci_free_dev(hdev);
err_put:
	connac1_put(bdev->cs);
	return ret;
}

static void btmtkstp_remove(struct platform_device *pdev)
{
	struct btmtkstp_dev *bdev = platform_get_drvdata(pdev);

	/* closes the device (BT off) if it is open */
	hci_unregister_dev(bdev->hdev);
	hci_free_dev(bdev->hdev);
	connac1_put(bdev->cs);
}

static const struct of_device_id btmtkstp_of_match[] = {
	{ .compatible = "mediatek,mt6785-bluetooth" },
	{ }
};
MODULE_DEVICE_TABLE(of, btmtkstp_of_match);

static struct platform_driver btmtkstp_driver = {
	.probe = btmtkstp_probe,
	.remove = btmtkstp_remove,
	.driver = {
		.name = "btmtkstp",
		.of_match_table = btmtkstp_of_match,
	},
};
module_platform_driver(btmtkstp_driver);

MODULE_DESCRIPTION("MediaTek CONNAC1 consys Bluetooth driver (STP over BTIF) ver " VERSION);
MODULE_VERSION(VERSION);
MODULE_LICENSE("GPL");
