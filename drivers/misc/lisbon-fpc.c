// SPDX-License-Identifier: GPL-2.0-only
/*
 * Normal world controls of the FPC fingerprint sensor of the Motorola Moto G60s (lisbon, MT6785).
 *
 * The sensor hangs on SPI0, which only the secure world drives: a trusted application of the
 * vendor TEE talks to the chip. Its user space client needs the normal world for the rest and
 * expects these files below /sys/class/fingerprint/fpc_spi:
 *
 *  hw_reset    write "reset": pulse the reset line
 *  clk_enable  write a boolean: switch the SPI0 clock gate
 *  irq         read: level of the interrupt line; poll() returns on a rising edge;
 *              writes are accepted and ignored (the client acknowledges that way)
 *  modalias    read: "fpc_spi" without a newline
 *
 * The driver switches the sensor supply on at probe time, selects the parent of the SPI clock
 * mux and resets the chip once. It does not touch SPI0 itself.
 */
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/regulator/consumer.h>

struct lisbon_fpc {
	struct gpio_desc *reset;
	struct gpio_desc *enable;
	struct gpio_desc *irq_gpio;
	struct clk *spi_clk;
	struct device *class_dev;
	struct mutex lock;	/* serializes reset pulses and clock switching */
	bool clock_on;
};

static void lisbon_fpc_reset(struct lisbon_fpc *fpc)
{
	gpiod_set_value_cansleep(fpc->reset, 0);
	usleep_range(100, 200);
	gpiod_set_value_cansleep(fpc->reset, 1);
	fsleep(5000);
	gpiod_set_value_cansleep(fpc->reset, 0);
	fsleep(5000);
}

static ssize_t hw_reset_store(struct device *dev, struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct lisbon_fpc *fpc = dev_get_drvdata(dev);

	if (!sysfs_streq(buf, "reset"))
		return -EINVAL;

	mutex_lock(&fpc->lock);
	lisbon_fpc_reset(fpc);
	mutex_unlock(&fpc->lock);
	return count;
}
static DEVICE_ATTR_WO(hw_reset);

static ssize_t clk_enable_store(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t count)
{
	struct lisbon_fpc *fpc = dev_get_drvdata(dev);
	bool enable;
	int ret;

	ret = kstrtobool(buf, &enable);
	if (ret)
		return ret;

	mutex_lock(&fpc->lock);
	if (enable != fpc->clock_on) {
		if (enable)
			ret = clk_prepare_enable(fpc->spi_clk);
		else
			clk_disable_unprepare(fpc->spi_clk);
		if (!ret)
			fpc->clock_on = enable;
	}
	mutex_unlock(&fpc->lock);
	return ret ? ret : count;
}
static DEVICE_ATTR_WO(clk_enable);

static ssize_t irq_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct lisbon_fpc *fpc = dev_get_drvdata(dev);
	int value;

	value = gpiod_get_value_cansleep(fpc->irq_gpio);
	if (value < 0)
		return value;

	return sysfs_emit(buf, "%d\n", value);
}

static ssize_t irq_store(struct device *dev, struct device_attribute *attr,
			 const char *buf, size_t count)
{
	/* Motorola's client acknowledges poll events by writing to this file. */
	return count;
}
static DEVICE_ATTR(irq, 0600, irq_show, irq_store);

static ssize_t modalias_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	/* The vendor client expects the value without a trailing newline. */
	return sysfs_emit(buf, "fpc_spi");
}
static DEVICE_ATTR_RO(modalias);

static struct attribute *lisbon_fpc_attrs[] = {
	&dev_attr_hw_reset.attr,
	&dev_attr_clk_enable.attr,
	&dev_attr_irq.attr,
	&dev_attr_modalias.attr,
	NULL,
};
ATTRIBUTE_GROUPS(lisbon_fpc);

static const struct class lisbon_fpc_class = {
	.name = "fingerprint",
};

static irqreturn_t lisbon_fpc_irq(int irq, void *data)
{
	struct lisbon_fpc *fpc = data;

	sysfs_notify(&fpc->class_dev->kobj, NULL, "irq");
	return IRQ_HANDLED;
}

static void lisbon_fpc_power_off(void *data)
{
	struct lisbon_fpc *fpc = data;

	gpiod_set_value_cansleep(fpc->reset, 1);
	gpiod_set_value_cansleep(fpc->enable, 0);
}

static void lisbon_fpc_release(void *data)
{
	struct lisbon_fpc *fpc = data;

	device_unregister(fpc->class_dev);
	if (fpc->clock_on)
		clk_disable_unprepare(fpc->spi_clk);
}

static int lisbon_fpc_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct lisbon_fpc *fpc;
	struct clk *parent, *selector;
	int ret, irq;

	fpc = devm_kzalloc(dev, sizeof(*fpc), GFP_KERNEL);
	if (!fpc)
		return -ENOMEM;

	mutex_init(&fpc->lock);
	fpc->reset = devm_gpiod_get(dev, "reset", GPIOD_OUT_HIGH);
	if (IS_ERR(fpc->reset))
		return dev_err_probe(dev, PTR_ERR(fpc->reset), "failed to get reset GPIO\n");

	fpc->irq_gpio = devm_gpiod_get(dev, "irq", GPIOD_IN);
	if (IS_ERR(fpc->irq_gpio))
		return dev_err_probe(dev, PTR_ERR(fpc->irq_gpio), "failed to get IRQ GPIO\n");

	irq = gpiod_to_irq(fpc->irq_gpio);
	if (irq < 0)
		return dev_err_probe(dev, irq, "failed to map IRQ GPIO\n");

	parent = devm_clk_get(dev, "parent");
	if (IS_ERR(parent))
		return dev_err_probe(dev, PTR_ERR(parent), "failed to get SPI parent clock\n");
	selector = devm_clk_get(dev, "selector");
	if (IS_ERR(selector))
		return dev_err_probe(dev, PTR_ERR(selector), "failed to get SPI clock mux\n");
	fpc->spi_clk = devm_clk_get(dev, "spi");
	if (IS_ERR(fpc->spi_clk))
		return dev_err_probe(dev, PTR_ERR(fpc->spi_clk), "failed to get SPI clock gate\n");

	ret = clk_set_parent(selector, parent);
	if (ret)
		return dev_err_probe(dev, ret, "failed to select SPI parent clock\n");

	ret = devm_regulator_get_enable(dev, "vdd");
	if (ret)
		return dev_err_probe(dev, ret, "failed to enable sensor supply\n");

	fpc->enable = devm_gpiod_get(dev, "enable", GPIOD_OUT_HIGH);
	if (IS_ERR(fpc->enable))
		return dev_err_probe(dev, PTR_ERR(fpc->enable), "failed to get supply GPIO\n");

	ret = devm_add_action_or_reset(dev, lisbon_fpc_power_off, fpc);
	if (ret)
		return ret;

	lisbon_fpc_reset(fpc);
	fpc->class_dev = device_create_with_groups(&lisbon_fpc_class, dev, 0, fpc,
						   lisbon_fpc_groups, "fpc_spi");
	if (IS_ERR(fpc->class_dev))
		return PTR_ERR(fpc->class_dev);

	ret = devm_add_action_or_reset(dev, lisbon_fpc_release, fpc);
	if (ret)
		return ret;

	return devm_request_threaded_irq(dev, irq, NULL, lisbon_fpc_irq,
					 IRQF_ONESHOT | IRQF_TRIGGER_RISING, dev_name(dev), fpc);
}

static const struct of_device_id lisbon_fpc_of_match[] = {
	{ .compatible = "motorola,lisbon-fpc-tee" },
	{ }
};
MODULE_DEVICE_TABLE(of, lisbon_fpc_of_match);

static struct platform_driver lisbon_fpc_driver = {
	.probe = lisbon_fpc_probe,
	.driver = {
		.name = "lisbon-fpc",
		.of_match_table = lisbon_fpc_of_match,
	},
};

static int __init lisbon_fpc_init(void)
{
	int ret;

	ret = class_register(&lisbon_fpc_class);
	if (ret)
		return ret;

	ret = platform_driver_register(&lisbon_fpc_driver);
	if (ret)
		class_unregister(&lisbon_fpc_class);
	return ret;
}
module_init(lisbon_fpc_init);

static void __exit lisbon_fpc_exit(void)
{
	platform_driver_unregister(&lisbon_fpc_driver);
	class_unregister(&lisbon_fpc_class);
}
module_exit(lisbon_fpc_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Moto G60s FPC sensor controls for the Trustonic client");
