// SPDX-License-Identifier: GPL-2.0+
/*
 * Mediatek Watchdog Driver
 *
 * Copyright (C) 2014 Matthias Brugger
 *
 * Matthias Brugger <matthias.bgg@gmail.com>
 *
 * Based on sunxi_wdt.c
 */

#include <dt-bindings/reset/mt2712-resets.h>
#include <dt-bindings/reset/mediatek,mt6735-wdt.h>
#include <dt-bindings/reset/mediatek,mt6785-resets.h>
#include <dt-bindings/reset/mediatek,mt6795-resets.h>
#include <dt-bindings/reset/mt7986-resets.h>
#include <dt-bindings/reset/mt8183-resets.h>
#include <dt-bindings/reset/mt8186-resets.h>
#include <dt-bindings/reset/mt8188-resets.h>
#include <dt-bindings/reset/mt8192-resets.h>
#include <dt-bindings/reset/mt8195-resets.h>
#include <linux/bitfield.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/platform_device.h>
#include <linux/reset-controller.h>
#include <linux/suspend.h>
#include <linux/types.h>
#include <linux/watchdog.h>
#include <linux/interrupt.h>

#define WDT_MAX_TIMEOUT		31
#define WDT_MIN_TIMEOUT		2
#define WDT_LENGTH_TIMEOUT(n)	((n) << 5)
/* the counter counts units of 512 ticks of the 32 kHz clock */
#define WDT_LENGTH_UNITS_PER_SEC	64

#define WDT_LENGTH		0x04
#define WDT_LENGTH_KEY		0x8

#define WDT_RST			0x08
#define WDT_RST_RELOAD		0x1971

#define WDT_MODE		0x00
#define WDT_MODE_EN		(1 << 0)
#define WDT_MODE_EXT_POL_LOW	(0 << 1)
#define WDT_MODE_EXT_POL_HIGH	(1 << 1)
#define WDT_MODE_EXRST_EN	(1 << 2)
#define WDT_MODE_IRQ_EN		(1 << 3)
#define WDT_MODE_AUTO_START	(1 << 4)
#define WDT_MODE_DUAL_EN	(1 << 6)
#define WDT_MODE_CNT_SEL	(1 << 8)
#define WDT_MODE_KEY		0x22000000

#define WDT_SWRST		0x14
#define WDT_SWRST_KEY		0x1209

#define WDT_SWSYSRST		0x18U
#define WDT_SWSYS_RST_KEY	0x88000000

#define WDT_SWSYSRST_EN		0xfc

#define DRV_NAME		"mtk-wdt"
#define DRV_VERSION		"1.0"

#define MT7988_TOPRGU_SW_RST_NUM	24

/* general purpose timer (GPT) block, see mediatek,sleep-timer */
#define GPT_IRQ_EN		0x00
#define GPT_IRQ_STA		0x04
#define GPT_IRQ_ACK		0x08
#define GPT_IRQ_BIT(ch)		BIT((ch) - 1)
#define GPT_CON(ch)		(0x10 * (ch))
#define GPT_CON_ENABLE		BIT(0)
#define GPT_CON_CLEAR		BIT(1)
#define GPT_CON_MODE		GENMASK(5, 4)
#define GPT_CON_MODE_REPEAT	1
#define GPT_CLK(ch)		(0x10 * (ch) + 0x04)
#define GPT_CLK_SRC_RTC32K	BIT(4)
#define GPT_CMP(ch)		(0x10 * (ch) + 0x0c)
#define GPT_CHANNELS		5	/* 32-bit channels 1..5 */
#define GPT_RTC_HZ		32768

static bool nowayout = WATCHDOG_NOWAYOUT;
static unsigned int timeout;
static bool suspend_keep;
static unsigned int sleep_feeds;

struct mtk_wdt_dev {
	struct watchdog_device wdt_dev;
	void __iomem *wdt_base;
	spinlock_t lock; /* protects WDT_SWSYSRST reg */
	struct reset_controller_dev rcdev;
	bool disable_wdt_extrst;
	bool reset_by_toprgu;
	bool has_swsysrst_en;
	void __iomem *gpt_base;		/* sleep feed, NULL without one */
	unsigned int gpt_ch;
	bool gpt_running;
};

struct mtk_wdt_data {
	int toprgu_sw_rst_num;
	bool has_swsysrst_en;
};

static const struct mtk_wdt_data mt2712_data = {
	.toprgu_sw_rst_num = MT2712_TOPRGU_SW_RST_NUM,
};

static const struct mtk_wdt_data mt6735_data = {
	.toprgu_sw_rst_num = MT6735_TOPRGU_RST_NUM,
};

static const struct mtk_wdt_data mt6785_data = {
	.toprgu_sw_rst_num = MT6785_TOPRGU_RST_NUM,
};

static const struct mtk_wdt_data mt6795_data = {
	.toprgu_sw_rst_num = MT6795_TOPRGU_SW_RST_NUM,
};

static const struct mtk_wdt_data mt7986_data = {
	.toprgu_sw_rst_num = MT7986_TOPRGU_SW_RST_NUM,
};

static const struct mtk_wdt_data mt7988_data = {
	.toprgu_sw_rst_num = MT7988_TOPRGU_SW_RST_NUM,
	.has_swsysrst_en = true,
};

static const struct mtk_wdt_data mt8183_data = {
	.toprgu_sw_rst_num = MT8183_TOPRGU_SW_RST_NUM,
};

static const struct mtk_wdt_data mt8186_data = {
	.toprgu_sw_rst_num = MT8186_TOPRGU_SW_RST_NUM,
};

static const struct mtk_wdt_data mt8188_data = {
	.toprgu_sw_rst_num = MT8188_TOPRGU_SW_RST_NUM,
};

static const struct mtk_wdt_data mt8192_data = {
	.toprgu_sw_rst_num = MT8192_TOPRGU_SW_RST_NUM,
};

static const struct mtk_wdt_data mt8195_data = {
	.toprgu_sw_rst_num = MT8195_TOPRGU_SW_RST_NUM,
};

/**
 * toprgu_reset_sw_en_unlocked() - enable/disable software control for reset bit
 * @data: Pointer to instance of driver data.
 * @id: Bit number identifying the reset to be enabled or disabled.
 * @enable: If true, enable software control for that bit, disable otherwise.
 *
 * Context: The caller must hold lock of struct mtk_wdt_dev.
 */
static void toprgu_reset_sw_en_unlocked(struct mtk_wdt_dev *data,
					unsigned long id, bool enable)
{
	u32 tmp;

	tmp = readl(data->wdt_base + WDT_SWSYSRST_EN);
	if (enable)
		tmp |= BIT(id);
	else
		tmp &= ~BIT(id);

	writel(tmp, data->wdt_base + WDT_SWSYSRST_EN);
}

static int toprgu_reset_update(struct reset_controller_dev *rcdev,
			       unsigned long id, bool assert)
{
	unsigned int tmp;
	unsigned long flags;
	struct mtk_wdt_dev *data =
		 container_of(rcdev, struct mtk_wdt_dev, rcdev);

	spin_lock_irqsave(&data->lock, flags);

	if (assert && data->has_swsysrst_en)
		toprgu_reset_sw_en_unlocked(data, id, true);

	tmp = readl(data->wdt_base + WDT_SWSYSRST);
	if (assert)
		tmp |= BIT(id);
	else
		tmp &= ~BIT(id);
	tmp |= WDT_SWSYS_RST_KEY;
	writel(tmp, data->wdt_base + WDT_SWSYSRST);

	if (!assert && data->has_swsysrst_en)
		toprgu_reset_sw_en_unlocked(data, id, false);

	spin_unlock_irqrestore(&data->lock, flags);

	return 0;
}

static int toprgu_reset_assert(struct reset_controller_dev *rcdev,
			       unsigned long id)
{
	return toprgu_reset_update(rcdev, id, true);
}

static int toprgu_reset_deassert(struct reset_controller_dev *rcdev,
				 unsigned long id)
{
	return toprgu_reset_update(rcdev, id, false);
}

static int toprgu_reset(struct reset_controller_dev *rcdev,
			unsigned long id)
{
	int ret;

	ret = toprgu_reset_assert(rcdev, id);
	if (ret)
		return ret;

	return toprgu_reset_deassert(rcdev, id);
}

static const struct reset_control_ops toprgu_reset_ops = {
	.assert = toprgu_reset_assert,
	.deassert = toprgu_reset_deassert,
	.reset = toprgu_reset,
};

static int toprgu_register_reset_controller(struct platform_device *pdev,
					    int rst_num)
{
	int ret;
	struct mtk_wdt_dev *mtk_wdt = platform_get_drvdata(pdev);

	spin_lock_init(&mtk_wdt->lock);

	mtk_wdt->rcdev.owner = THIS_MODULE;
	mtk_wdt->rcdev.nr_resets = rst_num;
	mtk_wdt->rcdev.ops = &toprgu_reset_ops;
	mtk_wdt->rcdev.of_node = pdev->dev.of_node;
	ret = devm_reset_controller_register(&pdev->dev, &mtk_wdt->rcdev);
	if (ret != 0)
		dev_err(&pdev->dev,
			"couldn't register wdt reset controller: %d\n", ret);
	return ret;
}

static int mtk_wdt_restart(struct watchdog_device *wdt_dev,
			   unsigned long action, void *data)
{
	struct mtk_wdt_dev *mtk_wdt = watchdog_get_drvdata(wdt_dev);
	void __iomem *wdt_base;
	u32 reg;

	wdt_base = mtk_wdt->wdt_base;

	/* Enable reset in order to issue a system reset instead of an IRQ */
	reg = readl(wdt_base + WDT_MODE);
	reg &= ~WDT_MODE_IRQ_EN;
	writel(reg | WDT_MODE_KEY, wdt_base + WDT_MODE);

	while (1) {
		writel(WDT_SWRST_KEY, wdt_base + WDT_SWRST);
		mdelay(5);
	}

	return 0;
}

static int mtk_wdt_ping(struct watchdog_device *wdt_dev)
{
	struct mtk_wdt_dev *mtk_wdt = watchdog_get_drvdata(wdt_dev);
	void __iomem *wdt_base = mtk_wdt->wdt_base;

	iowrite32(WDT_RST_RELOAD, wdt_base + WDT_RST);

	return 0;
}

/*
 * The counter runs for at most WDT_MAX_TIMEOUT seconds; the watchdog core
 * keeps longer heartbeats alive (max_hw_heartbeat_ms). In dual mode it runs
 * twice per heartbeat: the pretimeout interrupt comes after the first run,
 * the reset after the second one.
 */
static void mtk_wdt_set_length(struct mtk_wdt_dev *mtk_wdt,
			       unsigned int timeout, bool dual)
{
	u32 units = min(timeout, WDT_MAX_TIMEOUT) * WDT_LENGTH_UNITS_PER_SEC;

	if (dual)
		units /= 2;

	iowrite32(WDT_LENGTH_TIMEOUT(units) | WDT_LENGTH_KEY,
		  mtk_wdt->wdt_base + WDT_LENGTH);
}

static void mtk_wdt_set_dual(struct mtk_wdt_dev *mtk_wdt, bool dual)
{
	void __iomem *wdt_base = mtk_wdt->wdt_base;
	u32 reg = ioread32(wdt_base + WDT_MODE);

	if (dual)
		reg |= WDT_MODE_IRQ_EN | WDT_MODE_DUAL_EN;
	else
		reg &= ~(WDT_MODE_IRQ_EN | WDT_MODE_DUAL_EN);

	iowrite32(reg | WDT_MODE_KEY, wdt_base + WDT_MODE);
}

/*
 * The only pretimeout the hardware has: half the heartbeat, rounded up so
 * that the interrupt never comes before timeout - pretimeout. None for a
 * heartbeat the counter cannot hold.
 */
static unsigned int mtk_wdt_hw_pretimeout(unsigned int timeout)
{
	return timeout > WDT_MAX_TIMEOUT ? 0 : DIV_ROUND_UP(timeout, 2);
}

static int mtk_wdt_set_timeout(struct watchdog_device *wdt_dev,
			       unsigned int timeout)
{
	struct mtk_wdt_dev *mtk_wdt = watchdog_get_drvdata(wdt_dev);

	wdt_dev->timeout = timeout;

	/* like the core, drop a pretimeout that no longer fits */
	if (wdt_dev->pretimeout) {
		wdt_dev->pretimeout = mtk_wdt_hw_pretimeout(timeout);
		if (!wdt_dev->pretimeout)
			mtk_wdt_set_dual(mtk_wdt, false);
	}

	mtk_wdt_set_length(mtk_wdt, timeout, wdt_dev->pretimeout);
	mtk_wdt_ping(wdt_dev);

	return 0;
}

static int mtk_wdt_stop(struct watchdog_device *wdt_dev)
{
	struct mtk_wdt_dev *mtk_wdt = watchdog_get_drvdata(wdt_dev);
	void __iomem *wdt_base = mtk_wdt->wdt_base;
	u32 reg;

	reg = readl(wdt_base + WDT_MODE);
	reg &= ~WDT_MODE_EN;
	reg |= WDT_MODE_KEY;
	iowrite32(reg, wdt_base + WDT_MODE);

	return 0;
}

static int mtk_wdt_start(struct watchdog_device *wdt_dev)
{
	u32 reg;
	struct mtk_wdt_dev *mtk_wdt = watchdog_get_drvdata(wdt_dev);
	void __iomem *wdt_base = mtk_wdt->wdt_base;
	int ret;

	ret = mtk_wdt_set_timeout(wdt_dev, wdt_dev->timeout);
	if (ret < 0)
		return ret;

	reg = ioread32(wdt_base + WDT_MODE);
	if (wdt_dev->pretimeout)
		reg |= (WDT_MODE_IRQ_EN | WDT_MODE_DUAL_EN);
	else
		reg &= ~(WDT_MODE_IRQ_EN | WDT_MODE_DUAL_EN);
	if (mtk_wdt->disable_wdt_extrst)
		reg &= ~WDT_MODE_EXRST_EN;
	if (mtk_wdt->reset_by_toprgu)
		reg |= WDT_MODE_CNT_SEL;
	reg |= (WDT_MODE_EN | WDT_MODE_KEY);
	iowrite32(reg, wdt_base + WDT_MODE);

	return 0;
}

static void mtk_wdt_init(struct watchdog_device *wdt_dev)
{
	struct mtk_wdt_dev *mtk_wdt = watchdog_get_drvdata(wdt_dev);

	/*
	 * Left running by the bootloader: program length and mode as for a
	 * start, so that the hardware resets after the heartbeat the core
	 * keeps alive and not after whatever the bootloader chose.
	 */
	if (readl(mtk_wdt->wdt_base + WDT_MODE) & WDT_MODE_EN) {
		set_bit(WDOG_HW_RUNNING, &wdt_dev->status);
		mtk_wdt_start(wdt_dev);
	}
}

static int mtk_wdt_set_pretimeout(struct watchdog_device *wdd,
				  unsigned int timeout)
{
	struct mtk_wdt_dev *mtk_wdt = watchdog_get_drvdata(wdd);
	unsigned int pretimeout = 0;

	if (timeout) {
		pretimeout = mtk_wdt_hw_pretimeout(wdd->timeout);
		if (!pretimeout)
			return -EINVAL;
	}

	/* shorten the run before entering dual mode, lengthen it after leaving */
	wdd->pretimeout = pretimeout;
	if (pretimeout)
		mtk_wdt_set_length(mtk_wdt, wdd->timeout, true);
	mtk_wdt_set_dual(mtk_wdt, pretimeout);
	if (!pretimeout)
		mtk_wdt_set_length(mtk_wdt, wdd->timeout, false);
	mtk_wdt_ping(wdd);

	return 0;
}

static irqreturn_t mtk_wdt_isr(int irq, void *arg)
{
	struct watchdog_device *wdd = arg;

	watchdog_notify_pretimeout(wdd);

	return IRQ_HANDLED;
}

/*
 * Feed the watchdog while the system is suspended to idle.
 *
 * In suspend-to-idle the CPUs sit in their idle states with the tick frozen,
 * so neither userspace nor the watchdog core can ping, and the counter runs
 * for at most WDT_MAX_TIMEOUT seconds: an unfed watchdog would reset every
 * sleep longer than that. A channel of the general purpose timer, clocked
 * from the 32 kHz clock, raises a non-suspendable interrupt three times per
 * heartbeat instead. Its handler pings the watchdog and the CPU goes back to
 * sleep; no device resumes and the system does not wake up. If the system
 * hangs in the sleep, so that no CPU takes the interrupt any more, the
 * watchdog resets it after the heartbeat; a sleep of any length stays fed.
 * The timer runs only from the noirq suspend of the watchdog to its noirq
 * resume, the other devices suspend and resume under the normal watchdog.
 *
 * The vendor kernel covers the same window with the watchdog of the SPM
 * firmware (SPM_ARGS_PCM_WDT), which mainline does not start.
 */
static void mtk_wdt_hang_test(unsigned int where);

static irqreturn_t mtk_wdt_sleep_feed_isr(int irq, void *arg)
{
	struct mtk_wdt_dev *mtk_wdt = arg;
	void __iomem *gpt = mtk_wdt->gpt_base;
	u32 bit = GPT_IRQ_BIT(mtk_wdt->gpt_ch);

	if (!(readl(gpt + GPT_IRQ_STA) & bit))
		return IRQ_NONE;

	writel(bit, gpt + GPT_IRQ_ACK);
	mtk_wdt_ping(&mtk_wdt->wdt_dev);
	WRITE_ONCE(sleep_feeds, sleep_feeds + 1);
	mtk_wdt_hang_test(1);

	return IRQ_HANDLED;
}

static void mtk_wdt_sleep_feed_start(struct mtk_wdt_dev *mtk_wdt)
{
	void __iomem *gpt = mtk_wdt->gpt_base;
	unsigned int ch = mtk_wdt->gpt_ch;
	unsigned int period;

	period = max(min(mtk_wdt->wdt_dev.timeout, WDT_MAX_TIMEOUT) / 3, 1U);

	writel(0, gpt + GPT_CON(ch));
	writel(GPT_CLK_SRC_RTC32K, gpt + GPT_CLK(ch));
	writel(period * GPT_RTC_HZ, gpt + GPT_CMP(ch));
	writel(GPT_IRQ_BIT(ch), gpt + GPT_IRQ_ACK);
	writel(readl(gpt + GPT_IRQ_EN) | GPT_IRQ_BIT(ch), gpt + GPT_IRQ_EN);
	writel(FIELD_PREP(GPT_CON_MODE, GPT_CON_MODE_REPEAT) | GPT_CON_CLEAR |
	       GPT_CON_ENABLE, gpt + GPT_CON(ch));
	mtk_wdt->gpt_running = true;
}

static void mtk_wdt_sleep_feed_stop(struct mtk_wdt_dev *mtk_wdt)
{
	void __iomem *gpt = mtk_wdt->gpt_base;
	unsigned int ch = mtk_wdt->gpt_ch;

	writel(0, gpt + GPT_CON(ch));
	writel(readl(gpt + GPT_IRQ_EN) & ~GPT_IRQ_BIT(ch), gpt + GPT_IRQ_EN);
	writel(GPT_IRQ_BIT(ch), gpt + GPT_IRQ_ACK);
	mtk_wdt->gpt_running = false;
}

static void mtk_wdt_put_clk(void *clk)
{
	clk_disable_unprepare(clk);
	clk_put(clk);
}

/* mediatek,sleep-timer = <&gpt channel>; no property: no sleep feed */
static int mtk_wdt_sleep_feed_init(struct device *dev,
				   struct mtk_wdt_dev *mtk_wdt)
{
	struct of_phandle_args args;
	void __iomem *gpt;
	unsigned int ch;
	struct clk *clk;
	int irq, ret;

	ret = of_parse_phandle_with_fixed_args(dev->of_node,
					       "mediatek,sleep-timer", 1, 0,
					       &args);
	if (ret)
		return ret == -ENOENT ? 0 : ret;

	ch = args.args[0];
	if (ch < 1 || ch > GPT_CHANNELS) {
		ret = -EINVAL;
		goto out;
	}

	gpt = devm_of_iomap(dev, args.np, 0, NULL);
	if (IS_ERR(gpt)) {
		ret = PTR_ERR(gpt);
		goto out;
	}

	irq = of_irq_get(args.np, 0);
	if (irq <= 0) {
		ret = irq ? irq : -ENXIO;
		goto out;
	}

	clk = of_clk_get_by_name(args.np, "bus");
	if (IS_ERR(clk)) {
		ret = PTR_ERR(clk);
		goto out;
	}
	ret = clk_prepare_enable(clk);
	if (ret) {
		clk_put(clk);
		goto out;
	}
	ret = devm_add_action_or_reset(dev, mtk_wdt_put_clk, clk);
	if (ret)
		goto out;

	/* leave a channel alone that someone else (firmware) runs */
	if (readl(gpt + GPT_CON(ch)) & GPT_CON_ENABLE) {
		ret = -EBUSY;
		goto out;
	}

	mtk_wdt->gpt_base = gpt;
	mtk_wdt->gpt_ch = ch;
	mtk_wdt_sleep_feed_stop(mtk_wdt);

	ret = devm_request_irq(dev, irq, mtk_wdt_sleep_feed_isr,
			       IRQF_NO_SUSPEND | IRQF_NO_THREAD,
			       "wdt_sleep_feed", mtk_wdt);
	if (ret)
		mtk_wdt->gpt_base = NULL;
out:
	of_node_put(args.np);
	return ret;
}

static const struct watchdog_info mtk_wdt_info = {
	.identity	= DRV_NAME,
	.options	= WDIOF_SETTIMEOUT |
			  WDIOF_KEEPALIVEPING |
			  WDIOF_MAGICCLOSE,
};

static const struct watchdog_info mtk_wdt_pt_info = {
	.identity	= DRV_NAME,
	.options	= WDIOF_SETTIMEOUT |
			  WDIOF_PRETIMEOUT |
			  WDIOF_KEEPALIVEPING |
			  WDIOF_MAGICCLOSE,
};

static const struct watchdog_ops mtk_wdt_ops = {
	.owner		= THIS_MODULE,
	.start		= mtk_wdt_start,
	.stop		= mtk_wdt_stop,
	.ping		= mtk_wdt_ping,
	.set_timeout	= mtk_wdt_set_timeout,
	.set_pretimeout	= mtk_wdt_set_pretimeout,
	.restart	= mtk_wdt_restart,
};

static int mtk_wdt_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct mtk_wdt_dev *mtk_wdt;
	const struct mtk_wdt_data *wdt_data;
	int err, irq;

	mtk_wdt = devm_kzalloc(dev, sizeof(*mtk_wdt), GFP_KERNEL);
	if (!mtk_wdt)
		return -ENOMEM;

	platform_set_drvdata(pdev, mtk_wdt);

	mtk_wdt->wdt_base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(mtk_wdt->wdt_base))
		return PTR_ERR(mtk_wdt->wdt_base);

	irq = platform_get_irq_optional(pdev, 0);
	if (irq > 0) {
		err = devm_request_irq(&pdev->dev, irq, mtk_wdt_isr, 0, "wdt_bark",
				       &mtk_wdt->wdt_dev);
		if (err)
			return err;

		mtk_wdt->wdt_dev.info = &mtk_wdt_pt_info;
		mtk_wdt->wdt_dev.pretimeout =
			mtk_wdt_hw_pretimeout(WDT_MAX_TIMEOUT);
	} else {
		if (irq == -EPROBE_DEFER)
			return -EPROBE_DEFER;

		mtk_wdt->wdt_dev.info = &mtk_wdt_info;
	}

	mtk_wdt->wdt_dev.ops = &mtk_wdt_ops;
	mtk_wdt->wdt_dev.timeout = WDT_MAX_TIMEOUT;
	mtk_wdt->wdt_dev.max_hw_heartbeat_ms = WDT_MAX_TIMEOUT * 1000;
	mtk_wdt->wdt_dev.min_timeout = WDT_MIN_TIMEOUT;
	mtk_wdt->wdt_dev.parent = dev;

	watchdog_init_timeout(&mtk_wdt->wdt_dev, timeout, dev);
	watchdog_set_nowayout(&mtk_wdt->wdt_dev, nowayout);
	watchdog_set_restart_priority(&mtk_wdt->wdt_dev, 128);

	watchdog_set_drvdata(&mtk_wdt->wdt_dev, mtk_wdt);

	mtk_wdt->disable_wdt_extrst =
		of_property_read_bool(dev->of_node, "mediatek,disable-extrst");

	mtk_wdt->reset_by_toprgu =
		of_property_read_bool(dev->of_node, "mediatek,reset-by-toprgu");

	mtk_wdt_init(&mtk_wdt->wdt_dev);

	err = mtk_wdt_sleep_feed_init(dev, mtk_wdt);
	if (err == -EPROBE_DEFER)
		return err;
	if (err)
		dev_warn(dev, "no feed during suspend-to-idle: %d\n", err);
	else if (mtk_wdt->gpt_base)
		dev_info(dev, "fed by GPT channel %u during suspend-to-idle\n",
			 mtk_wdt->gpt_ch);

	watchdog_stop_on_reboot(&mtk_wdt->wdt_dev);
	err = devm_watchdog_register_device(dev, &mtk_wdt->wdt_dev);
	if (unlikely(err))
		return err;

	dev_info(dev, "Watchdog enabled (timeout=%d sec, nowayout=%d)\n",
		 mtk_wdt->wdt_dev.timeout, nowayout);

	wdt_data = of_device_get_match_data(dev);
	if (wdt_data) {
		err = toprgu_register_reset_controller(pdev,
						       wdt_data->toprgu_sw_rst_num);
		if (err)
			return err;

		mtk_wdt->has_swsysrst_en = wdt_data->has_swsysrst_en;
	}

	return 0;
}

#ifdef CONFIG_PM_SLEEP_DEBUG
/*
 * Test aid: 1 hangs the CPU that takes the second feed of a sleep, 2
 * hangs the noirq resume after the watchdog restarted. Both with interrupts
 * off and only while the watchdog runs, so the watchdog has to reset.
 */
static unsigned int sleep_hang_test;
module_param(sleep_hang_test, uint, 0644);
MODULE_PARM_DESC(sleep_hang_test,
		 "test aid: hang 1 in suspend-to-idle, 2 in the noirq resume (default=0)");

static void mtk_wdt_hang_test(unsigned int where)
{
	static unsigned int feeds;

	if (READ_ONCE(sleep_hang_test) != where)
		return;
	if (where == 1 && ++feeds < 2)
		return;

	pr_emerg("mtk-wdt: sleep_hang_test %u: hanging, the watchdog must reset\n",
		 where);
	local_irq_disable();
	for (;;)
		cpu_relax();
}
#else
static void mtk_wdt_hang_test(unsigned int where)
{
}
#endif

/*
 * Stop and restart the watchdog in the noirq phases: it then keeps counting
 * while the other devices suspend and resume, so a driver that hangs there
 * ends in a reset instead of a dead system. In suspend-to-idle with a sleep
 * timer it keeps running in between too, fed by mtk_wdt_sleep_feed_isr().
 */
static int mtk_wdt_suspend(struct device *dev)
{
	struct mtk_wdt_dev *mtk_wdt = dev_get_drvdata(dev);
	struct watchdog_device *wdd = &mtk_wdt->wdt_dev;

	if (!watchdog_active(wdd))
		return 0;

	/*
	 * Test aid: with suspend_keep the watchdog keeps counting
	 * unfed while the system sleeps, so a sleep that does not end within
	 * the heartbeat (at most WDT_MAX_TIMEOUT) ends in a reset.
	 *
	 * With a pretimeout the hardware runs in dual mode and raises the
	 * pretimeout interrupt after half the heartbeat; nothing handles it
	 * while the system sleeps, and on MT6785 the system then never wakes
	 * up. Count the whole heartbeat in single mode while asleep;
	 * mtk_wdt_resume() restores the mode through mtk_wdt_start().
	 */
	if (READ_ONCE(suspend_keep)) {
		if (wdd->pretimeout) {
			mtk_wdt_set_dual(mtk_wdt, false);
			mtk_wdt_set_length(mtk_wdt, wdd->timeout, false);
		}
		mtk_wdt_ping(wdd);
		dev_info(dev, "kept running unfed during suspend (%u s)\n",
			 min(wdd->timeout, WDT_MAX_TIMEOUT));
		return 0;
	}

	if (mtk_wdt->gpt_base && pm_suspend_target_state == PM_SUSPEND_TO_IDLE) {
		mtk_wdt_ping(wdd);
		mtk_wdt_sleep_feed_start(mtk_wdt);
		return 0;
	}

	mtk_wdt_stop(wdd);

	return 0;
}

static int mtk_wdt_resume(struct device *dev)
{
	struct mtk_wdt_dev *mtk_wdt = dev_get_drvdata(dev);

	if (mtk_wdt->gpt_running)
		mtk_wdt_sleep_feed_stop(mtk_wdt);

	if (watchdog_active(&mtk_wdt->wdt_dev)) {
		mtk_wdt_start(&mtk_wdt->wdt_dev);
		mtk_wdt_ping(&mtk_wdt->wdt_dev);
		mtk_wdt_hang_test(2);
	}

	return 0;
}

static const struct of_device_id mtk_wdt_dt_ids[] = {
	{ .compatible = "mediatek,mt2712-wdt", .data = &mt2712_data },
	{ .compatible = "mediatek,mt6589-wdt" },
	{ .compatible = "mediatek,mt6735-wdt", .data = &mt6735_data },
	{ .compatible = "mediatek,mt6785-wdt", .data = &mt6785_data },
	{ .compatible = "mediatek,mt6795-wdt", .data = &mt6795_data },
	{ .compatible = "mediatek,mt7986-wdt", .data = &mt7986_data },
	{ .compatible = "mediatek,mt7988-wdt", .data = &mt7988_data },
	{ .compatible = "mediatek,mt8183-wdt", .data = &mt8183_data },
	{ .compatible = "mediatek,mt8186-wdt", .data = &mt8186_data },
	{ .compatible = "mediatek,mt8188-wdt", .data = &mt8188_data },
	{ .compatible = "mediatek,mt8192-wdt", .data = &mt8192_data },
	{ .compatible = "mediatek,mt8195-wdt", .data = &mt8195_data },
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, mtk_wdt_dt_ids);

static DEFINE_NOIRQ_DEV_PM_OPS(mtk_wdt_pm_ops,
			       mtk_wdt_suspend, mtk_wdt_resume);

static struct platform_driver mtk_wdt_driver = {
	.probe		= mtk_wdt_probe,
	.driver		= {
		.name		= DRV_NAME,
		.pm		= pm_sleep_ptr(&mtk_wdt_pm_ops),
		.of_match_table	= mtk_wdt_dt_ids,
	},
};

module_platform_driver(mtk_wdt_driver);

module_param(timeout, uint, 0);
MODULE_PARM_DESC(timeout, "Watchdog heartbeat in seconds");

module_param(nowayout, bool, 0);
MODULE_PARM_DESC(nowayout, "Watchdog cannot be stopped once started (default="
			__MODULE_STRING(WATCHDOG_NOWAYOUT) ")");

module_param(suspend_keep, bool, 0644);
MODULE_PARM_DESC(suspend_keep,
		 "test aid: keep an active watchdog running unfed during system suspend (default=0)");

module_param(sleep_feeds, uint, 0444);
MODULE_PARM_DESC(sleep_feeds,
		 "lisbon: pings by the sleep timer during suspend-to-idle so far");

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Matthias Brugger <matthias.bgg@gmail.com>");
MODULE_DESCRIPTION("Mediatek WatchDog Timer Driver");
MODULE_VERSION(DRV_VERSION);
