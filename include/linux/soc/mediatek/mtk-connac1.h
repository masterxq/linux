/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * MediaTek CONNAC1 consys (MT6785 "soc1_0"): interface for the function drivers (WLAN, BT).
 *
 * The consys driver (drivers/soc/mediatek/mtk-connac1-*.c) powers the subsystem, downloads
 * the MCU patch over BTIF/STP and runs the WMT start-up including the RF calibration. A WLAN
 * driver attached to the CONN bus (wifi@18000000) references the consys node with the
 * "mediatek,consys" phandle and switches its function on and off through this interface.
 * The Bluetooth driver does the same; its HCI packets (H4 format, type byte first) travel in
 * STP frames of type BT over the BTIF owned by the consys driver. The GNSS driver exchanges the
 * opaque data stream of the GNSS DSP in STP frames of type GPS.
 */
#ifndef __LINUX_SOC_MEDIATEK_MTK_CONNAC1_H
#define __LINUX_SOC_MEDIATEK_MTK_CONNAC1_H

#include <linux/err.h>
#include <linux/types.h>

struct consys;
struct device;

struct connac1_wifi_info {
	struct device *consys_dev;
	phys_addr_t emi_phys;		/* EMI region of the consys (CONN view: remapped) */
	size_t emi_size;
	u32 hw_ver, fw_ver;		/* CONN_HW_VER / CONN_FW_VER, e.g. 0x8a00 */
	/* WMT RF calibration events (RF_CAL 01 and 03), valid while the function is on */
	const u8 *cal[2];
	size_t cal_len[2];
};

/**
 * struct connac1_bt_ops - callbacks of the Bluetooth function driver
 * @recv: payload of one STP frame of type BT (H4 stream: type byte, header, data; a frame
 *	normally holds one complete packet). Called in process context with the STP lock held:
 *	must not sleep for long and must not call back into the consys.
 */
struct connac1_bt_ops {
	void (*recv)(void *priv, const u8 *data, size_t len);
};

/**
 * struct connac1_gps_ops - callbacks of the GNSS function driver
 * @recv: payload of one STP frame of type GPS (part of the byte stream of the GNSS DSP).
 *	Called in process context with the STP lock held: must not sleep for long and must
 *	not call back into the consys.
 */
struct connac1_gps_ops {
	void (*recv)(void *priv, const u8 *data, size_t len);
};

struct connac1_gps_info {
	u32 hw_ver, fw_ver;		/* CONN_HW_VER / CONN_FW_VER (low 16 bits) */
	bool co_clock;			/* the consys shares the PMIC 26 MHz clock (co-clock) */
};

#if IS_ENABLED(CONFIG_MTK_CONNAC1_CONSYS)
struct consys *connac1_get(struct device *dev);
void connac1_put(struct consys *cs);
int connac1_get_emi(struct consys *cs, phys_addr_t *base, size_t *size);
int connac1_wifi_on(struct consys *cs, struct connac1_wifi_info *info);
void connac1_wifi_off(struct consys *cs);
int connac1_bt_on(struct consys *cs, const struct connac1_bt_ops *ops, void *priv);
void connac1_bt_off(struct consys *cs);
int connac1_bt_send(struct consys *cs, u8 pkt_type, const u8 *data, size_t len);
int connac1_gps_on(struct consys *cs, const struct connac1_gps_ops *ops, void *priv,
		   struct connac1_gps_info *info);
void connac1_gps_off(struct consys *cs);
int connac1_gps_send(struct consys *cs, const u8 *data, size_t len);
size_t connac1_gps_max_payload(void);
#else
static inline struct consys *connac1_get(struct device *dev)
{
	return ERR_PTR(-ENODEV);
}
static inline void connac1_put(struct consys *cs) { }
static inline int connac1_get_emi(struct consys *cs, phys_addr_t *base, size_t *size)
{
	return -ENODEV;
}
static inline int connac1_wifi_on(struct consys *cs, struct connac1_wifi_info *info)
{
	return -ENODEV;
}
static inline void connac1_wifi_off(struct consys *cs) { }
static inline int connac1_bt_on(struct consys *cs, const struct connac1_bt_ops *ops, void *priv)
{
	return -ENODEV;
}
static inline void connac1_bt_off(struct consys *cs) { }
static inline int connac1_bt_send(struct consys *cs, u8 pkt_type, const u8 *data, size_t len)
{
	return -ENODEV;
}
static inline int connac1_gps_on(struct consys *cs, const struct connac1_gps_ops *ops, void *priv,
				 struct connac1_gps_info *info)
{
	return -ENODEV;
}
static inline void connac1_gps_off(struct consys *cs) { }
static inline int connac1_gps_send(struct consys *cs, const u8 *data, size_t len)
{
	return -ENODEV;
}
static inline size_t connac1_gps_max_payload(void)
{
	return 0;
}
#endif

#endif
