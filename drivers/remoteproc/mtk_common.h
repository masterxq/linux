/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Copyright (c) 2019 MediaTek Inc.
 */

#ifndef __RPROC_MTK_COMMON_H
#define __RPROC_MTK_COMMON_H

#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/remoteproc.h>
#include <linux/remoteproc/mtk_scp.h>

#define MT8183_SW_RSTN			0x0
#define MT8183_SW_RSTN_BIT		BIT(0)
#define MT8183_SCP_TO_HOST		0x1C
#define MT8183_SCP_IPC_INT_BIT		BIT(0)
#define MT8183_SCP_WDT_INT_BIT		BIT(8)
#define MT8183_HOST_TO_SCP		0x28
#define MT8183_HOST_IPC_INT_BIT		BIT(0)
#define MT6785_SCP_TO_SPM		0x20
#define MT6785_HOST_IPC_AWAKE_BIT	BIT(2)
#define MT6785_SCP_GPR(x)		(0x50 + (x) * 4)
#define MT6785_SCP_DEBUG_PC		0xb4
#define MT6785_SCP_SLEEP_STATUS		0x114
#define MT6785_SCP_A_IS_SLEEP		BIT(0)
#define MT8183_WDT_CFG			0x84
#define MT8183_SCP_CLK_SW_SEL		0x4000
#define MT8183_SCP_CLK_DIV_SEL		0x4024
#define MT8183_SCP_SRAM_PDN		0x402C
#define MT8183_SCP_L1_SRAM_PD		0x4080
#define MT8183_SCP_TCM_TAIL_SRAM_PD	0x4094

#define MT8183_SCP_CACHE_SEL(x)		(0x14000 + (x) * 0x3000)
#define MT8183_SCP_CACHE_CON		MT8183_SCP_CACHE_SEL(0)
#define MT8183_SCP_DCACHE_CON		MT8183_SCP_CACHE_SEL(1)
#define MT8183_SCP_CACHESIZE_8KB	BIT(8)
#define MT8183_SCP_CACHE_CON_WAYEN	BIT(10)

#define MT8186_SCP_L1_SRAM_PD_P1	0x40B0
#define MT8186_SCP_L1_SRAM_PD_p2	0x40B4

#define MT8192_L2TCM_SRAM_PD_0		0x10C0
#define MT8192_L2TCM_SRAM_PD_1		0x10C4
#define MT8192_L2TCM_SRAM_PD_2		0x10C8
#define MT8192_L1TCM_SRAM_PDN		0x102C
#define MT8192_CPU0_SRAM_PD		0x1080

#define MT8192_SCP2APMCU_IPC_SET	0x4080
#define MT8192_SCP2APMCU_IPC_CLR	0x4084
#define MT8192_SCP_IPC_INT_BIT		BIT(0)
#define MT8192_SCP2SPM_IPC_CLR		0x4094
#define MT8192_GIPC_IN_SET		0x4098
#define MT8192_HOST_IPC_INT_BIT		BIT(0)
#define MT8195_CORE1_HOST_IPC_INT_BIT	BIT(4)

#define MT8192_CORE0_SW_RSTN_CLR	0x10000
#define MT8192_CORE0_SW_RSTN_SET	0x10004
#define MT8192_CORE0_MEM_ATT_PREDEF	0x10008
#define MT8192_CORE0_WDT_IRQ		0x10030
#define MT8192_CORE0_WDT_CFG		0x10034

#define MT8195_SYS_STATUS		0x4004
#define MT8195_CORE0_WDT		BIT(16)
#define MT8195_CORE1_WDT		BIT(17)

#define MT8195_L1TCM_SRAM_PDN_RESERVED_RSI_BITS		GENMASK(7, 4)

#define MT8195_CPU1_SRAM_PD			0x1084
#define MT8195_SSHUB2APMCU_IPC_SET		0x4088
#define MT8195_SSHUB2APMCU_IPC_CLR		0x408C
#define MT8195_CORE1_SW_RSTN_CLR		0x20000
#define MT8195_CORE1_SW_RSTN_SET		0x20004
#define MT8195_CORE1_MEM_ATT_PREDEF		0x20008
#define MT8195_CORE1_WDT_IRQ			0x20030
#define MT8195_CORE1_WDT_CFG			0x20034

#define MT8195_SEC_CTRL				0x85000
#define MT8195_CORE_OFFSET_ENABLE_D		BIT(13)
#define MT8195_CORE_OFFSET_ENABLE_I		BIT(12)
#define MT8195_L2TCM_OFFSET_RANGE_0_LOW		0x850b0
#define MT8195_L2TCM_OFFSET_RANGE_0_HIGH	0x850b4
#define MT8195_L2TCM_OFFSET			0x850d0

#define SCP_FW_VER_LEN			32

/* tinysys loader layout in SCP SRAM (MT6785 and other "v01" SCPs) */
#define MTK_SCP_REGION_INFO_OFFSET	0x400
#define MTK_SCP_RTOS_START		0x800
#define MTK_SCP_PLATFORM_LEN		12

struct scp_run {
	u32 signaled;
	s8 fw_ver[SCP_FW_VER_LEN];
	u32 dec_capability;
	u32 enc_capability;
	wait_queue_head_t wq;
};

struct scp_ipi_desc {
	/* For protecting handler. */
	struct mutex lock;
	scp_ipi_handler_t handler;
	void *priv;
};

struct mtk_scp;
struct regmap;

struct mtk_scp_sizes_data {
	size_t max_dram_size;
	size_t ipi_share_buffer_size;
	/* bytes between the len field and the payload of a share object */
	size_t ipi_share_buffer_reserved;
};

/**
 * struct mtk_scp_region_info - loader parameters in SCP SRAM
 *
 * Written by the bootloader at %MTK_SCP_REGION_INFO_OFFSET together with the
 * tinysys loader at SRAM offset 0. The loader copies the firmware from
 * @ap_firmware_start to %MTK_SCP_RTOS_START and jumps to it. All addresses
 * are AP physical addresses.
 */
struct mtk_scp_region_info {
	u32 ap_loader_start;
	u32 ap_loader_size;
	u32 ap_firmware_start;
	u32 ap_firmware_size;
	u32 ap_dram_start;
	u32 ap_dram_size;
	u32 ap_dram_backup_start;
	u32 struct_size;
	u32 scp_log_thru_ap_uart;
	u32 task_context_ptr;
	u32 il1c_con;
	u32 dl1c_con;
	u32 scpctl;
	u32 ap_params_start;
	char platform[MTK_SCP_PLATFORM_LEN];
};

struct mtk_scp_of_data {
	int (*scp_clk_get)(struct mtk_scp *scp);
	int (*scp_before_load)(struct mtk_scp *scp);
	void (*scp_irq_handler)(struct mtk_scp *scp);
	void (*scp_reset_assert)(struct mtk_scp *scp);
	void (*scp_reset_deassert)(struct mtk_scp *scp);
	void (*scp_stop)(struct mtk_scp *scp);
	void *(*scp_da_to_va)(struct mtk_scp *scp, u64 da, size_t len);
	/* firmware placed in SRAM/DRAM by the bootloader instead of an ELF file */
	int (*scp_preload_init)(struct mtk_scp *scp);
	/* keep the SCP out of deep sleep while the AP accesses its SRAM */
	int (*scp_awake)(struct mtk_scp *scp, bool awake);
	const struct rproc_ops *rproc_ops;

	u32 host_to_scp_reg;
	u32 host_to_scp_int_bit;
	/* bits of host_to_scp_reg that mean "previous IPI not taken yet", 0: all */
	u32 host_to_scp_busy_mask;

	/* IPI the firmware sends once it runs, SCP_IPI_INIT if not set */
	u32 init_ipi_id;
	scp_ipi_handler_t init_ipi_handler;

	size_t ipi_buf_offset;
	const struct mtk_scp_sizes_data *scp_sizes;
};

struct mtk_scp_of_cluster {
	void __iomem *reg_base;
	void __iomem *l1tcm_base;
	size_t l1tcm_size;
	phys_addr_t l1tcm_phys;
	struct list_head mtk_scp_list;
	/* Prevent concurrent operations of this structure and L2TCM power control. */
	struct mutex cluster_lock;
	u32 l2tcm_refcnt;
};

struct mtk_scp {
	struct device *dev;
	struct rproc *rproc;
	struct clk *clk;
	void __iomem *sram_base;
	size_t sram_size;
	phys_addr_t sram_phys;

	const struct mtk_scp_of_data *data;

	struct mtk_share_obj __iomem *recv_buf;
	struct mtk_share_obj __iomem *send_buf;
	struct scp_run run;
	/* To prevent multiple ipi_send run concurrently. */
	struct mutex send_lock;
	struct scp_ipi_desc ipi_desc[SCP_IPI_MAX];
	bool ipi_id_ack[SCP_IPI_MAX];
	wait_queue_head_t ack_wq;
	u8 *share_buf;

	void *cpu_addr;
	dma_addr_t dma_addr;

	struct rproc_subdev *rpmsg_subdev;

	struct list_head elem;
	struct mtk_scp_of_cluster *cluster;

	/* bootloader-loaded firmware (scp_preload_init) */
	struct mtk_scp_region_info region_info;
	void *loader;
	struct regmap *infracfg;
	u32 ready_info;
	bool running;

	/* scp_awake_get()/scp_awake_put() */
	struct mutex awake_lock;
	unsigned int awake_count;

	/* statistics */
	unsigned int ipi_rx_count;
	unsigned int ipi_rx_unhandled;
	unsigned int ipi_tx_count;
	unsigned int ipi_tx_timeout;
	unsigned int awake_fail;
	unsigned int wdt_count;
	u32 ipi_last_rx_id;
	u32 ipi_last_tx_id;
};

/**
 * struct mtk_share_obj - SRAM buffer shared with AP and SCP
 *
 * @id:		IPI id
 * @len:	share buffer length
 * @share_buf:	share buffer data
 */
struct mtk_share_obj {
	u32 id;
	u32 len;
	u8 *share_buf;
};

void scp_memcpy_aligned(void __iomem *dst, const void *src, unsigned int len);
void __iomem *scp_ipi_payload(struct mtk_scp *scp, struct mtk_share_obj __iomem *obj);
int scp_awake_get(struct mtk_scp *scp);
void scp_awake_put(struct mtk_scp *scp);
void scp_ipi_lock(struct mtk_scp *scp, u32 id);
void scp_ipi_unlock(struct mtk_scp *scp, u32 id);

#endif
