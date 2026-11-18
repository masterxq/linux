// SPDX-License-Identifier: GPL-2.0-only
/*
 * IIO devices for the sensors behind the SCP sensor hub of the Motorola Moto G60s
 * (lisbon, MT6785).
 *
 * Binds to the auxiliary device "lisbon_scp_hub.iio" that lisbon-scp-hub adds once the hub is set
 * up, so udev loads this module through its alias. One IIO device per hub sensor that
 * lisbon-scp-hub found: accelerometer and gyroscope
 * (InvenSense ICM-4N607), magnetometer (Memsic MMC5603), light and proximity (ams TMD2755).
 * Reading a channel enables the sensor in the hub (lisbon-scp-hub disables it again after
 * idle_ms without reads) and returns its newest sample; no buffer, no trigger, which is
 * what iio-sensor-proxy needs for its polling drivers.
 *
 * Units of the hub (scale constants of the SCP firmware, see lisbon-scp-hub.h): the
 * firmware converts its float samples with * 1000 (accelerometer, m/s^2), * 100
 * (magnetometer, uT) and * 57.29578 * 131000 (gyroscope, rad/s), so the scales are
 * 0.001 m/s^2, 1/7505747 rad/s and 0.0001 gauss. Light is in lux (vendor HAL takes the
 * value as is). The hub delivers the axes already turned into the device frame (Android
 * convention, which is also the one of Documentation/devicetree/bindings/iio/mount-matrix.txt:
 * flat, display up: z = +9.8 m/s^2; upright: y = +9.8 m/s^2).
 *
 * Mount matrix: "mount-matrix" of the child nodes accelerometer, gyroscope and magnetometer
 * of the "sensor-hub" node (the node of the hub device), identity without them.
 * iio-sensor-proxy, the user of the accelerometer, takes in_mount_matrix and wants the
 * gravity readings with the opposite sign (upright: y < 0), so the lisbon DT sets the
 * accelerometer matrix to minus identity, as other phone DTs do for it.
 */

#include <linux/auxiliary_bus.h>
#include <linux/iio/iio.h>
#include <linux/kernel.h>
#include <linux/limits.h>
#include <linux/module.h>
#include <linux/property.h>

#include "lisbon-scp-hub.h"

/*
 * The hub reports the distance of the proximity sensor in cm as an integer (event value 1,
 * "oneshot"); far is the maximum range (5 on lisbon). IIO wants larger = closer, so by
 * default in_proximity_raw is 0 at prox_far and 10 otherwise, with nearlevel 5.
 */
static unsigned int prox_index = 1;
module_param(prox_index, uint, 0644);
MODULE_PARM_DESC(prox_index, "value of the proximity event shown as in_proximity_raw (0..7)");

static int prox_far = 5;
module_param(prox_far, int, 0644);
MODULE_PARM_DESC(prox_far, "in_proximity_raw is 0 when the value equals this, else 10 (INT_MIN: value as is)");

static unsigned int prox_nearlevel = 5;
module_param(prox_nearlevel, uint, 0644);
MODULE_PARM_DESC(prox_nearlevel, "in_proximity_nearlevel (raw value from which on an object is near)");

#define LSCP_PROX_NEAR_RAW	10

struct lscp_iio {
	enum lisbon_scp_sensor id;
	struct iio_mount_matrix orientation;
};

static const struct iio_mount_matrix *lscp_mount_matrix(const struct iio_dev *indio_dev,
							const struct iio_chan_spec *chan)
{
	const struct lscp_iio *d = iio_priv(indio_dev);

	return &d->orientation;
}

static const struct iio_chan_spec_ext_info lscp_vec_ext_info[] = {
	IIO_MOUNT_MATRIX(IIO_SHARED_BY_DIR, lscp_mount_matrix),
	{ }
};

static ssize_t lscp_nearlevel(struct iio_dev *indio_dev, uintptr_t priv,
			      const struct iio_chan_spec *chan, char *buf)
{
	return sysfs_emit(buf, "%u\n", READ_ONCE(prox_nearlevel));
}

static const struct iio_chan_spec_ext_info lscp_prox_ext_info[] = {
	{
		.name = "nearlevel",
		.shared = IIO_SEPARATE,
		.read = lscp_nearlevel,
	},
	{ }
};

#define LSCP_AXIS(_type, _axis, _idx) {					\
	.type = _type,							\
	.modified = 1,							\
	.channel2 = IIO_MOD_##_axis,					\
	.address = _idx,						\
	.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),			\
	.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),		\
	.ext_info = lscp_vec_ext_info,					\
	.scan_index = -1,						\
}

static const struct iio_chan_spec lscp_accel_channels[] = {
	LSCP_AXIS(IIO_ACCEL, X, 0),
	LSCP_AXIS(IIO_ACCEL, Y, 1),
	LSCP_AXIS(IIO_ACCEL, Z, 2),
};

static const struct iio_chan_spec lscp_gyro_channels[] = {
	LSCP_AXIS(IIO_ANGL_VEL, X, 0),
	LSCP_AXIS(IIO_ANGL_VEL, Y, 1),
	LSCP_AXIS(IIO_ANGL_VEL, Z, 2),
};

static const struct iio_chan_spec lscp_magn_channels[] = {
	LSCP_AXIS(IIO_MAGN, X, 0),
	LSCP_AXIS(IIO_MAGN, Y, 1),
	LSCP_AXIS(IIO_MAGN, Z, 2),
};

static const struct iio_chan_spec lscp_light_channels[] = {
	{
		.type = IIO_LIGHT,
		.info_mask_separate = BIT(IIO_CHAN_INFO_PROCESSED),
		.scan_index = -1,
	},
};

static const struct iio_chan_spec lscp_prox_channels[] = {
	{
		.type = IIO_PROXIMITY,
		.info_mask_separate = BIT(IIO_CHAN_INFO_RAW),
		.ext_info = lscp_prox_ext_info,
		.scan_index = -1,
	},
};

static const struct {
	const char *name;
	const struct iio_chan_spec *channels;
	unsigned int num_channels;
	/* scale numerator/denominator, 0 = no scale */
	int scale[2];
	/* child of the sensor-hub DT node with the mount matrix */
	const char *node;
} lscp_desc[LISBON_SCP_NUM_SENSORS] = {
	[LISBON_SCP_ACCEL] = { "lisbon-scp-accel", lscp_accel_channels,
			       ARRAY_SIZE(lscp_accel_channels), { 1, 1000 }, "accelerometer" },
	[LISBON_SCP_GYRO] = { "lisbon-scp-gyro", lscp_gyro_channels,
			      ARRAY_SIZE(lscp_gyro_channels), { 1, 7505747 }, "gyroscope" },
	[LISBON_SCP_MAG] = { "lisbon-scp-magn", lscp_magn_channels,
			     ARRAY_SIZE(lscp_magn_channels), { 1, 10000 }, "magnetometer" },
	[LISBON_SCP_LIGHT] = { "lisbon-scp-light", lscp_light_channels,
			       ARRAY_SIZE(lscp_light_channels) },
	[LISBON_SCP_PROX] = { "lisbon-scp-prox", lscp_prox_channels,
			      ARRAY_SIZE(lscp_prox_channels) },
};

static int lscp_read_raw(struct iio_dev *indio_dev, struct iio_chan_spec const *chan,
			 int *val, int *val2, long mask)
{
	struct lscp_iio *d = iio_priv(indio_dev);
	struct lisbon_scp_sample s;
	unsigned int idx;
	int ret;

	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		if (!lscp_desc[d->id].scale[1])
			return -EINVAL;
		*val = lscp_desc[d->id].scale[0];
		*val2 = lscp_desc[d->id].scale[1];
		return IIO_VAL_FRACTIONAL;
	case IIO_CHAN_INFO_RAW:
	case IIO_CHAN_INFO_PROCESSED:
		ret = lisbon_scp_hub_read(d->id, &s);
		if (ret)
			return ret;
		if (d->id == LISBON_SCP_PROX) {
			idx = min_t(unsigned int, READ_ONCE(prox_index), ARRAY_SIZE(s.v) - 1);
			*val = s.v[idx];
			if (READ_ONCE(prox_far) != INT_MIN)
				*val = *val == READ_ONCE(prox_far) ? 0 : LSCP_PROX_NEAR_RAW;
		} else {
			*val = s.v[chan->address];
		}
		return IIO_VAL_INT;
	}

	return -EINVAL;
}

static const struct iio_info lscp_info = {
	.read_raw = lscp_read_raw,
};

static struct iio_dev *lscp_devs[LISBON_SCP_NUM_SENSORS];
static struct fwnode_handle *lscp_nodes[LISBON_SCP_NUM_SENSORS];

static void lscp_remove_all(void)
{
	int i;

	for (i = LISBON_SCP_NUM_SENSORS - 1; i >= 0; i--) {
		if (lscp_devs[i]) {
			iio_device_unregister(lscp_devs[i]);
			iio_device_free(lscp_devs[i]);
			lscp_devs[i] = NULL;
		}
		fwnode_handle_put(lscp_nodes[i]);
		lscp_nodes[i] = NULL;
	}
}

/* mount matrix from the DT node of the sensor (identity without node or property) */
static int lscp_orientation(struct device *parent, struct fwnode_handle *hub,
			    struct iio_dev *indio_dev, int i)
{
	struct lscp_iio *d = iio_priv(indio_dev);

	if (hub && lscp_desc[i].node)
		lscp_nodes[i] = fwnode_get_named_child_node(hub, lscp_desc[i].node);
	if (lscp_nodes[i])
		device_set_node(&indio_dev->dev, lscp_nodes[i]);
	else
		device_set_node(&indio_dev->dev, dev_fwnode(parent));

	return iio_read_mount_matrix(&indio_dev->dev, &d->orientation);
}

static int lscp_probe(struct auxiliary_device *adev, const struct auxiliary_device_id *id)
{
	struct device *parent = &adev->dev;
	struct fwnode_handle *hub;
	int i, ret, n = 0;

	/* one hub, one auxiliary device: the IIO devices sit in module-wide tables */
	if (!lisbon_scp_hub_device())
		return -ENODEV;
	hub = fwnode_handle_get(dev_fwnode(adev->dev.parent));

	for (i = 0; i < LISBON_SCP_NUM_SENSORS; i++) {
		struct iio_dev *indio_dev;
		struct lscp_iio *d;

		if (!lisbon_scp_hub_present(i)) {
			dev_info(parent, "%s: no sensor in the hub\n", lscp_desc[i].name);
			continue;
		}

		indio_dev = iio_device_alloc(parent, sizeof(*d));
		if (!indio_dev) {
			ret = -ENOMEM;
			goto err;
		}
		d = iio_priv(indio_dev);
		d->id = i;
		ret = lscp_orientation(parent, hub, indio_dev, i);
		if (ret) {
			dev_err(parent, "%s: bad mount-matrix: %d\n", lscp_desc[i].name, ret);
			iio_device_free(indio_dev);
			goto err;
		}
		indio_dev->name = lscp_desc[i].name;
		/* the accelerometer sits in the display half (iio-sensor-proxy reads the label) */
		if (i == LISBON_SCP_ACCEL)
			indio_dev->label = "accel-display";
		indio_dev->info = &lscp_info;
		indio_dev->modes = INDIO_DIRECT_MODE;
		indio_dev->channels = lscp_desc[i].channels;
		indio_dev->num_channels = lscp_desc[i].num_channels;

		ret = iio_device_register(indio_dev);
		if (ret) {
			iio_device_free(indio_dev);
			goto err;
		}
		lscp_devs[i] = indio_dev;
		n++;
	}

	fwnode_handle_put(hub);
	if (!n) {
		dev_err(parent, "the sensor hub reports no sensor (not set up yet?)\n");
		return -ENODEV;
	}
	dev_info(parent, "%d IIO devices for the sensor hub (accel %s, mag %s, light %s)\n", n,
		 lisbon_scp_hub_chip(LISBON_SCP_ACCEL) ?: "-",
		 lisbon_scp_hub_chip(LISBON_SCP_MAG) ?: "-",
		 lisbon_scp_hub_chip(LISBON_SCP_LIGHT) ?: "-");

	return 0;

err:
	fwnode_handle_put(hub);
	lscp_remove_all();
	return ret;
}

static void lscp_remove(struct auxiliary_device *adev)
{
	lscp_remove_all();
}

static const struct auxiliary_device_id lscp_id_table[] = {
	{ .name = "lisbon_scp_hub.iio" },
	{}
};
MODULE_DEVICE_TABLE(auxiliary, lscp_id_table);

static struct auxiliary_driver lscp_driver = {
	.probe = lscp_probe,
	.remove = lscp_remove,
	.id_table = lscp_id_table,
};
module_auxiliary_driver(lscp_driver);

MODULE_DESCRIPTION("lisbon: IIO devices for the MT6785 SCP sensor hub");
MODULE_LICENSE("GPL");
