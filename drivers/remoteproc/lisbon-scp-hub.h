/* SPDX-License-Identifier: GPL-2.0-only */
/*
 * Sensor hub of the MT6785 SCP on the Motorola Moto G60s (lisbon): interface of
 * lisbon-scp-hub for the IIO module.
 */
#ifndef __LISBON_SCP_HUB_H
#define __LISBON_SCP_HUB_H

#include <linux/types.h>

struct device;

enum lisbon_scp_sensor {
	LISBON_SCP_ACCEL,
	LISBON_SCP_GYRO,
	LISBON_SCP_MAG,
	LISBON_SCP_LIGHT,
	LISBON_SCP_PROX,
	LISBON_SCP_NUM_SENSORS,
};

/*
 * One event of the hub as it sits in the shared FIFO (data_unit_t, 44 bytes):
 * values after the 12 byte header. Units as sent by the firmware:
 * accelerometer m/s^2 * 1000, magnetometer uT * 100, gyroscope
 * rad/s * 7505747 (deg/s * 131000), light lux (v[0]), proximity distance
 * in cm (v[1], "oneshot"). Axes in the device frame (Android/IIO).
 */
struct lisbon_scp_sample {
	s32 v[8];
	u64 timestamp;	/* SCP time in ns */
	u32 seq;	/* counts the samples of this sensor */
};

struct device *lisbon_scp_hub_device(void);
bool lisbon_scp_hub_present(enum lisbon_scp_sensor id);
const char *lisbon_scp_hub_chip(enum lisbon_scp_sensor id);
int lisbon_scp_hub_read(enum lisbon_scp_sensor id, struct lisbon_scp_sample *s);

#endif
