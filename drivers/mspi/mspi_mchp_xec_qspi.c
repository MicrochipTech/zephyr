/*
 * Copyright (c) 2026 Microchip Technology Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/*
 * MSPI flash controller driver for Microchip XEC series QMSPI periperhal.
 * QMSPI current implementation supports two hardware controlled chip selects, full-duplex,
 * dual, and quad I/O. Single-wire half-duplex requires the IO[0] and IO[1] signals to be connected
 * together externally. This driver does not supporting single-wire half-duplex.
 * QMSPI use hardware descriptor registers to describe each phase of the
 * transfer. QMSPI implements DMA support to either channels in the central DMA
 * controller or using the 6 six channels of local-DMA in the QMSPI peripheral.
 */

#include "zephyr/pm/state.h"
#include <sys/errno.h>
#define DT_DRV_COMPAT microchip_xec_mspi_controller

#include <soc.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/drivers/dma.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/drivers/mspi.h>
#include <zephyr/drivers/pinctrl.h>
#include <zephyr/dt-bindings/interrupt-controller/mchp-xec-ecia.h>
#include <zephyr/irq.h>
#include <zephyr/kernel.h>
#include <zephyr/pm/device.h>
#include <zephyr/pm/device_runtime.h>
#include <zephyr/pm/policy.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(mspi_mchp_xec, CONFIG_MSPI_LOG_LEVEL);

#define MSPI_XEC_QSPI_MAX_FREQ MHZ(96)
#define MSPI_XEC_QSPI_MIN_FREQ 1500U

struct mspi_xec_qspi_drvcfg {
	mm_reg_t qbase;
	uint32_t clock_freq;
	uint16_t enc_pcr;
	uint8_t girq;
	uint8_t girq_pos;
	enum mspi_cpp_mode cpp_mode;
	struct mspi_cfg mspicfg;
	const struct pinctrl_dev_config *pcfg;
	void (*irq_cfg_fp)(void);
#if defined(CONFIG_MSPI_DMA)
#if defined(CONFIG_MSPI_MCHP_XEC_USE_DMA_DRIVER)
	const struct device *tx_dma_dev;
	const struct device *rx_dma_dev;
	uint8_t tx_dma_chan;
	uint8_t tx_dma_trigsrc;
	uint8_t rx_dma_chan;
	uint8_t rx_dma_trigsrc;
#else

#endif
#endif
};

struct mspi_xec_qspi_drv_data {
	const struct device *dev; /* pointer to this device */
	volatile uint32_t hwsts;
	struct k_mutex lock;
	struct k_sem dev_lock;
	struct k_sem sync;
	const struct mspi_dev_id *dev_id;
	struct mspi_dev_cfg dev_cfg;
	struct mspi_xfr *mxfr;
	void *tx_rem;
	uint32_t tx_rem_len;
	void *rx_rem;
	uint32_t rx_rem_len;
	struct mspi_callback_context *cb_ctx[MSPI_BUS_EVENT_MAX];
	mspi_callback_handler_t cb[MSPI_BUS_EVENT_MAX];
#if defined(CONFIG_MSPI_DMA)
#if defined(CONFIG_MSPI_MCHP_XEC_USE_DMA_DRIVER)
	int dma_status;
	bool dma_done;
	struct dma_config dma_cfg;
	struct dma_block_config dma_blk_cfg;
#else
	/* TODO - do we need anything? */
#endif
#endif
};

static bool mspi_xec_qspi_is_busy(const struct device *controller)
{
	struct mspi_xec_qspi_drv_data *drvdat = controller->data;

	return k_sem_count_get(&drvdat->dev_lock) == 0;
}

/*
 * DMA Support
 */
#if defined(CONFIG_MSPI_DMA)
/**
 * @brief Initialize DMA for QSPI
 *
 * Configures both Zephyr DMA driver and HAL DMA driver for QSPI transfers.
 * Due to use of QSPI HAL API, both drivers need to be configured.
 *
 * @param hdma Pointer to HAL DMA handle
 * @param dma_stream Pointer to Zephyr DMA stream structure
 * @return 0 on success, negative errno on failure
 */
#if 0
static int mspi_stm32_qspi_dma_init(DMA_HandleTypeDef *hdma, struct stm32_stream *dma_stream)
{
	int ret;

	/* Check if DMA device is ready */
	if (!device_is_ready(dma_stream->dev)) {
		LOG_ERR("DMA %s device not ready", dma_stream->dev->name);
		return -ENODEV;
	}

	/* Configure Zephyr DMA driver */
	dma_stream->cfg.user_data = hdma;
	/* This field is used to inform driver that it is overridden */
	dma_stream->cfg.linked_channel = STM32_DMA_HAL_OVERRIDE;

	ret = dma_config(dma_stream->dev, dma_stream->channel, &dma_stream->cfg);
	if (ret != 0) {
		LOG_ERR("Failed to configure DMA channel %d", dma_stream->channel);
		return ret;
	}

	/* Validate data size alignment */
	if (dma_stream->cfg.source_data_size != dma_stream->cfg.dest_data_size) {
		LOG_ERR("DMA Source and destination data sizes not aligned");
		return -EINVAL;
	}

	/* Configure HAL DMA driver for QSPI */
	int index = find_lsb_set(dma_stream->cfg.source_data_size) - 1;

	hdma->Init.PeriphDataAlignment = mspi_stm32_table_dest_size[index];
	hdma->Init.MemDataAlignment = mspi_stm32_table_src_size[index];
	hdma->Init.PeriphInc = DMA_PINC_DISABLE;
	hdma->Init.MemInc = DMA_MINC_ENABLE;
	hdma->Init.Mode = DMA_NORMAL;
	hdma->Init.Priority = mspi_stm32_table_priority[dma_stream->cfg.channel_priority];
	hdma->Init.Direction = mspi_stm32_table_direction[dma_stream->cfg.channel_direction];
#ifdef CONFIG_DMA_STM32_V1
	/* TODO: Not tested in this configuration */
	hdma->Init.Channel = dma_stream->cfg.dma_slot;
#else
	hdma->Init.Request = dma_stream->cfg.dma_slot;
#endif /* CONFIG_DMA_STM32_V1 */

	/* Get DMA channel instance */
	hdma->Instance = STM32_DMA_GET_CHANNEL_INSTANCE(dma_stream->reg, dma_stream->channel);

	/* Initialize HAL DMA */
	if (HAL_DMA_Init(hdma) != HAL_OK) {
		LOG_ERR("QSPI DMA Init failed");
		return -EIO;
	}

	LOG_DBG("QSPI DMA initialized");
	return 0;
}

/**
 * @brief Setup DMA for QSPI controller
 *
 * @param dev_cfg Device configuration
 * @param dev_data Device data
 * @return 0 on success, negative errno on failure
 */
static int mspi_stm32_qspi_dma_setup(const struct mspi_stm32_conf *dev_cfg,
				     struct mspi_stm32_data *dev_data)
{
	int ret;

	if (!dev_cfg->dma_specified) {
		LOG_ERR("DMA configuration is missing from the device tree");
		return -EIO;
	}

	/* Initialize DMA */
	ret = mspi_stm32_qspi_dma_init(&dev_data->hdma, &dev_data->dma);
	if (ret != 0) {
		LOG_ERR("QSPI DMA init failed");
		return ret;
	}

	/* Link DMA to QSPI HAL handle */
	__HAL_LINKDMA(&dev_data->hmspi.qspi, hdma, dev_data->hdma);

	LOG_DBG("QSPI with DMA Transfer configured");
	return 0;
}

/**
 * @brief DMA callback for QSPI transfers
 *
 * Routes DMA interrupts to HAL DMA IRQ handler
 */
static void mspi_stm32_qspi_dma_callback(const struct device *dev, void *arg,
					 uint32_t channel, int status)
{
	DMA_HandleTypeDef *hdma = arg;

	ARG_UNUSED(dev);
	ARG_UNUSED(channel);

	if (status < 0) {
		LOG_ERR("DMA callback error with channel %d", channel);
	}

	HAL_DMA_IRQHandler(hdma);
}
#endif /* 0 */
#endif /* CONFIG_MSPI_DMA */

#if 0
/* Check if device is in memory-mapped mode */
static bool mspi_stm32_qspi_is_memmap(const struct device *controller)
{
	struct mspi_stm32_data *dev_data = controller->data;

	/* Check the FMODE bits in CCR register to see if in memory-mapped mode */
	return stm32_reg_read_bits(&dev_data->hmspi.qspi.Instance->CCR, QUADSPI_CCR_FMODE) ==
	       QUADSPI_CCR_FMODE;
}

/* Set the device back in command mode */
static int mspi_stm32_qspi_memmap_off(const struct device *controller)
{
	struct mspi_stm32_data *dev_data = controller->data;

	if (!mspi_stm32_qspi_is_memmap(controller)) {
		/* Already in command mode */
		return 0;
	}

	if (HAL_QSPI_Abort(&dev_data->hmspi.qspi) != HAL_OK) {
		LOG_ERR("QSPI MemMapped abort failed");
		return -EIO;
	}

	LOG_DBG("QSPI memory mapped mode disabled");
	return 0;
}

/* Set the device in MemMapped mode */
static int mspi_stm32_qspi_memmap_on(const struct device *controller)
{
	struct mspi_stm32_data *dev_data = controller->data;
	QSPI_CommandTypeDef s_command;
	QSPI_MemoryMappedTypeDef s_MemMappedCfg;
	HAL_StatusTypeDef hal_ret;

	if (mspi_stm32_qspi_is_memmap(controller)) {
		/* Already in memory-mapped mode */
		return 0;
	}

	s_command = mspi_stm32_qspi_prepare_cmd(dev_data->dev_cfg.io_mode,
						dev_data->dev_cfg.data_rate);

	/* Set read command - use the configured read command if available */
	if (dev_data->dev_cfg.read_cmd != 0) {
		s_command.Instruction = dev_data->dev_cfg.read_cmd;
	} else {
		/* Fallback to standard fast read commands */
		if (dev_data->dev_cfg.addr_length == 4) {
			s_command.Instruction = MSPI_NOR_CMD_READ_FAST_4B;
		} else {
			s_command.Instruction = MSPI_NOR_CMD_READ_FAST;
		}
	}

	s_command.AddressSize = mspi_stm32_qspi_hal_address_size(dev_data->dev_cfg.addr_length);
	s_command.DummyCycles = dev_data->dev_cfg.rx_dummy;
	s_command.Address = 0;

	/* Enable the memory-mapping */
	s_MemMappedCfg.TimeOutActivation = QSPI_TIMEOUT_COUNTER_DISABLE;
	s_MemMappedCfg.TimeOutPeriod = 0;

	hal_ret = HAL_QSPI_MemoryMapped(&dev_data->hmspi.qspi, &s_command, &s_MemMappedCfg);
	if (hal_ret != HAL_OK) {
		LOG_ERR("Failed to enable QSPI memory mapped mode: %d", hal_ret);
		return -EIO;
	}

	LOG_DBG("QSPI memory mapped mode enabled");
	return 0;
}

/**
 * @brief Check if command requires indirect mode in XIP configuration
 *
 * @param packet Pointer to transfer packet
 * @return true if command needs indirect mode.
 * @return false if command can use memory-mapped mode.
 */
static bool mspi_stm32_qspi_needs_indirect_mode(const struct mspi_xfer_packet *packet)
{
	return (packet->cmd == MSPI_NOR_CMD_WREN) || (packet->cmd == MSPI_NOR_CMD_SE) ||
	       (packet->cmd == MSPI_NOR_CMD_SE_4B) || (packet->cmd == MSPI_NOR_CMD_RDSR) ||
	       (packet->dir == MSPI_TX);
}
#endif /* 0 */

#if 0
/**
 * @brief Execute data transfer (TX or RX) in indirect mode
 *
 * @param dev Pointer to device structure
 * @param packet Pointer to transfer packet
 * @param access_mode Access mode (SYNC, ASYNC, or DMA)
 * @return 0 on success
 * @return A negative errno value upon failure.
 */
static int mspi_stm32_qspi_execute_transfer(const struct device *dev,
					    const struct mspi_xfer_packet *packet,
					    uint8_t access_mode)
{
	HAL_StatusTypeDef hal_ret;
	struct mspi_stm32_data *dev_data = dev->data;

	if (packet->dir == MSPI_RX) {
		/* Receive the data */
		switch (access_mode) {
		case MSPI_ACCESS_SYNC:
			hal_ret = HAL_QSPI_Receive(&dev_data->hmspi.qspi, packet->data_buf,
						   HAL_QSPI_TIMEOUT_DEFAULT_VALUE);
			break;
		case MSPI_ACCESS_ASYNC:
			hal_ret = HAL_QSPI_Receive_IT(&dev_data->hmspi.qspi, packet->data_buf);
			break;
		case MSPI_ACCESS_DMA:
#if defined(CONFIG_MSPI_DMA)
			hal_ret = HAL_QSPI_Receive_DMA(&dev_data->hmspi.qspi, packet->data_buf);
#else
			LOG_ERR("DMA mode not enabled (CONFIG_MSPI_DMA not set)");
			return -ENOTSUP;
#endif
			break;
		default:
			LOG_ERR("Invalid access mode: %d", access_mode);
			return -EINVAL;
		}
	} else {
		/* Transmit the data */
		switch (access_mode) {
		case MSPI_ACCESS_SYNC:
			hal_ret = HAL_QSPI_Transmit(&dev_data->hmspi.qspi, packet->data_buf,
						    HAL_QSPI_TIMEOUT_DEFAULT_VALUE);
			break;
		case MSPI_ACCESS_ASYNC:
			hal_ret = HAL_QSPI_Transmit_IT(&dev_data->hmspi.qspi, packet->data_buf);
			break;
		case MSPI_ACCESS_DMA:
#if defined(CONFIG_MSPI_DMA)
			hal_ret = HAL_QSPI_Transmit_DMA(&dev_data->hmspi.qspi, packet->data_buf);
#else
			LOG_ERR("DMA mode not enabled (CONFIG_MSPI_DMA not set)");
			return -ENOTSUP;
#endif
			break;
		default:
			LOG_ERR("Invalid access mode: %d", access_mode);
			return -EINVAL;
		}
	}

	if ((hal_ret != HAL_OK) || (access_mode == MSPI_ACCESS_SYNC)) {
		pm_policy_state_lock_put(PM_STATE_SUSPEND_TO_IDLE, PM_ALL_SUBSTATES);
		(void)pm_device_runtime_put(dev);

		if (hal_ret != HAL_OK) {
			LOG_ERR("Failed to start %s transfer: %d",
				packet->dir == MSPI_RX ? "receive" : "transmit", hal_ret);
			return -EIO;
		}

		return 0;
	}

	/* For ASYNC mode, wait for IRQ completion (PM locks released in ISR) */
	if (k_sem_take(&dev_data->sync, K_FOREVER) < 0) {
		LOG_ERR("Failed to complete async transfer");
		/* If semaphore wait fails, ISR never completed, so release PM locks */
		pm_policy_state_lock_put(PM_STATE_SUSPEND_TO_IDLE, PM_ALL_SUBSTATES);
		(void)pm_device_runtime_put(dev);
		return -EIO;
	}

	return 0;
}

/**
 * @brief Read data in memory-mapped mode (XIP)
 *
 * Note: Write operations are NOT supported in memory-mapped mode for QSPI.
 * Writes must use indirect mode .
 *
 * @param dev Pointer to the device structure
 * @param packet Pointer to transfer packet (must be RX direction)
 * @return 0 on success.
 * @return A negative errno value upon failure.
 */
static int mspi_stm32_qspi_memory_mapped_read(const struct device *dev,
					      const struct mspi_xfer_packet *packet)
{
	struct mspi_stm32_data *dev_data = dev->data;
	int ret;

	if (!mspi_stm32_qspi_is_memmap(dev)) {
		ret = mspi_stm32_qspi_memmap_on(dev);
		if (ret != 0) {
			LOG_ERR("Failed to enable memory mapped mode");
			return ret;
		}
	}

	uintptr_t mmap_addr = dev_data->memmap_base_addr + packet->address;

	/* Memory-mapped mode is READ-ONLY for QSPI */
	LOG_DBG("Memory-mapped read from 0x%08lx, len %zu", mmap_addr, packet->num_bytes);
	memcpy(packet->data_buf, (void *)mmap_addr, packet->num_bytes);

	return 0;
}

/**
 * @brief Send a Command to the NOR and Receive/Transceive data if relevant in IT or DMA mode.
 *
 * @param dev Pointer to device structure
 * @param packet Pointer to transfer packet
 * @param access_mode Access mode (SYNC or ASYNC)
 * @return 0 on success.
 * @return A negative errno value upon failure.
 */
static int mspi_stm32_qspi_access(const struct device *dev, const struct mspi_xfer_packet *packet,
				  uint8_t access_mode)
{
	struct mspi_stm32_data *dev_data = dev->data;
	HAL_StatusTypeDef hal_ret;
	int ret;

	/* === XIP Mode: Handle memory-mapped or indirect mode switching === */
	if (dev_data->xip_cfg.enable) {
		/* Read operations can use memory-mapped mode */
		if (!mspi_stm32_qspi_needs_indirect_mode(packet)) {
			return mspi_stm32_qspi_memory_mapped_read(dev, packet);
		}

		/* Commands that need indirect mode*/
		ret = mspi_stm32_qspi_memmap_off(dev);
		if (ret != 0) {
			LOG_ERR("Failed to abort memory-mapped mode");
			return ret;
		}
	}

	/* === Indirect Mode: Standard command + data transfer === */

	/* Acquire PM locks for indirect mode operations */
	(void)pm_device_runtime_get(dev);
	pm_policy_state_lock_get(PM_STATE_SUSPEND_TO_IDLE, PM_ALL_SUBSTATES);

	/* Prepare QSPI command structure */
	QSPI_CommandTypeDef cmd = mspi_stm32_qspi_prepare_cmd(dev_data->dev_cfg.io_mode,
							      dev_data->dev_cfg.data_rate);

	cmd.NbData = packet->num_bytes;
	cmd.Instruction = packet->cmd;
	cmd.DummyCycles = packet->dir == MSPI_TX ? dev_data->ctx.xfer.tx_dummy
					  : dev_data->ctx.xfer.rx_dummy;
	cmd.Address = packet->address;
	cmd.AddressSize = mspi_stm32_qspi_hal_address_size(dev_data->ctx.xfer.addr_length);

	if (cmd.NbData == 0) {
		cmd.DataMode = QSPI_DATA_NONE;
	}

	if (cmd.Instruction == MSPI_NOR_CMD_WREN) {
		cmd.AddressMode = QSPI_ADDRESS_NONE;
	}

	hal_ret = HAL_QSPI_Command(&dev_data->hmspi.qspi, &cmd, HAL_QSPI_TIMEOUT_DEFAULT_VALUE);
	if (hal_ret != HAL_OK) {
		LOG_ERR("HAL_QSPI_Command failed: %d", hal_ret);
		pm_policy_state_lock_put(PM_STATE_SUSPEND_TO_IDLE, PM_ALL_SUBSTATES);
		(void)pm_device_runtime_put(dev);
		return -EIO;
	}

	/* If no data phase, we're done */
	if (packet->num_bytes == 0) {
		pm_policy_state_lock_put(PM_STATE_SUSPEND_TO_IDLE, PM_ALL_SUBSTATES);
		(void)pm_device_runtime_put(dev);
		return 0;
	}

	/* Execute the data transfer (TX or RX) */
	return mspi_stm32_qspi_execute_transfer(dev, packet, access_mode);
}

/**
 * @brief Validate MSPI configuration parameters
 *
 * @param config Pointer to MSPI configuration
 * @return 0 on success, negative errno on failure
 */
static int mspi_stm32_qspi_conf_validate(const struct mspi_cfg *config, uint32_t max_frequency)
{
	/* Only Controller mode is supported */
	if (config->op_mode != MSPI_OP_MODE_CONTROLLER) {
		LOG_ERR("Only support MSPI controller mode.");
		return -ENOTSUP;
	}

	/* Check the max possible freq. */
	if (config->max_freq > max_frequency) {
		LOG_ERR("Max_freq %d too large.", config->max_freq);
		return -ENOTSUP;
	}

	if (config->duplex != MSPI_HALF_DUPLEX) {
		LOG_ERR("Only support half duplex mode.");
		return -ENOTSUP;
	}

	if (config->num_periph > MSPI_MAX_DEVICE) {
		LOG_ERR("Invalid MSPI peripheral number.");
		return -ENOTSUP;
	}

	return 0;
}

/**
 * @brief Configure QSPI clocks and calculate prescaler
 *
 * @param cfg Device configuration
 * @param data Device data
 * @return 0 on success, negative errno on failure
 */
static int mspi_stm32_qspi_clock_config(const struct mspi_stm32_conf *cfg,
					struct mspi_stm32_data *data)
{
	uint32_t ahb_clock_freq;
	uint32_t prescaler;

	/* Clock configuration */
	if (clock_control_on(DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE),
			     (clock_control_subsys_t)&cfg->pclken[0]) != 0) {
		LOG_ERR("Could not enable MSPI clock");
		return -EIO;
	}
	if (clock_control_get_rate(DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE),
				   (clock_control_subsys_t)&cfg->pclken[0], &ahb_clock_freq) < 0) {
		LOG_ERR("Failed call clock_control_get_rate(pclken)");
		return -EIO;
	}

	/* Calculate prescaler based on desired frequency */
	for (prescaler = MSPI_STM32_CLOCK_PRESCALER_MIN;
	     prescaler <= MSPI_STM32_CLOCK_PRESCALER_MAX; prescaler++) {
		data->dev_cfg.freq = MSPI_STM32_CLOCK_COMPUTE(ahb_clock_freq, prescaler);

		if (data->dev_cfg.freq <= cfg->mspicfg.max_freq) {
			break;
		}
	}

	__ASSERT_NO_MSG(prescaler <= MSPI_STM32_CLOCK_PRESCALER_MAX);

	/* Set prescaler in QSPI HAL handle */
	data->hmspi.qspi.Init.ClockPrescaler = prescaler;

	return 0;
}

/**
 * @brief Initialize QSPI HAL
 *
 * @param hmspi QSPI HAL handle
 * @return 0 on success, negative errno on failure
 */
static int mspi_stm32_qspi_hal_init(QSPI_HandleTypeDef *hmspi)
{
	HAL_StatusTypeDef hal_ret;

	/* Initialize remaining QSPI handle parameters */
	hmspi->Init.FifoThreshold = MSPI_STM32_FIFO_THRESHOLD;
	hmspi->Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_1_CYCLE;
	hmspi->Init.ClockMode = QSPI_CLOCK_MODE_0;
	hmspi->Init.FlashID = QSPI_FLASH_ID_1;
	hmspi->Init.DualFlash = QSPI_DUALFLASH_DISABLE;

	/* Initialize HAL QSPI */
	hal_ret = HAL_QSPI_Init(hmspi);
	if (hal_ret != HAL_OK) {
		LOG_ERR("HAL_QSPI_Init failed: %d", hal_ret);
		return -EIO;
	}

	return 0;
}
#endif /* 0 */

static void qspi_soft_reset(mm_reg_t qbase)
{
	uint32_t reg_save_restore[4] = {0};

	reg_save_restore[0] = sys_read32(qbase + XEC_QSPI_CSTM_OFS);
	reg_save_restore[1] = sys_read32(qbase + XEC_QSPI_TAPS_OFS);
	reg_save_restore[2] = sys_read32(qbase + XEC_QSPI_TAPS_ADJ_OFS);
	reg_save_restore[3] = sys_read32(qbase + XEC_QSPI_TAPS_CR2_OFS);

	sys_set_bit(qbase + XEC_QSPI_MODE_OFS, XEC_QSPI_MODE_SRST_POS);
	while (sys_test_bit(qbase + XEC_QSPI_MODE_OFS, XEC_QSPI_MODE_SRST_POS) != 0) {
	}

	sys_write32(reg_save_restore[0], qbase + XEC_QSPI_CSTM_OFS);
	sys_write32(reg_save_restore[1], qbase + XEC_QSPI_TAPS_OFS);
	sys_write32(reg_save_restore[2], qbase + XEC_QSPI_TAPS_ADJ_OFS);
	sys_write32(reg_save_restore[3], qbase + XEC_QSPI_TAPS_CR2_OFS);
}

static uint32_t xec_qspi_calc_fdiv(uint32_t freqhz)
{
	uint32_t fdiv = 0;

	if (freqhz >= MSPI_XEC_QSPI_MAX_FREQ) {
		fdiv = 1u; /* divide by 1 */
	} else if (freqhz <= MSPI_XEC_QSPI_MIN_FREQ) {
		fdiv = 0u; /* divide by 65536 */
	} else {
		fdiv = MSPI_XEC_QSPI_MAX_FREQ / freqhz;
		if ((MSPI_XEC_QSPI_MAX_FREQ % freqhz) >= (freqhz / 2u)) {
			fdiv++;
		}
		if (fdiv > UINT16_MAX) {
			fdiv = UINT16_MAX;
		}
	}

	return fdiv;
}

/* Mode SPI_MODE_CPOL : SPI_MODE_CPHA
 *  0      0               0           clock idle lo, data sampled on RE, shifted on FE
 *  1      0               1           clock idle lo, data sampled on FE, shifted on RE
 *  2      1               0           clock idle hi, data sampled on FE, shifted on RE
 *  3      1               1           clock idle hi, data sampled on RE, shifted on FE
 *       CPOL CPHA_SDI CPHA_SDO
 *  0     0     0         0
 *  1     0     1         1
 *  2     1     0         0
 *  3     1     1         1
 *
 * QSPI SPI frequency >= 48 MHz require:
 * Mode CPOL CPHA_SDI CPHA_SDO
 * 0      0     0         1
 * 3      1     1         0
 */
static const uint8_t qspi_signal_mode[4] = {0, 0x6, 0x1, 0x3};

static void mspi_xec_qspi_ctrl_init(const struct device *controller, uint32_t freqhz,
                                    enum mspi_cpp_mode cpp_mode)
{
	const struct mspi_xec_qspi_drvcfg *drvcfg = controller->config;
	mm_reg_t qb = drvcfg->qbase;
	uint32_t v = 0, mode = 0;

	if (drvcfg->mspicfg.re_init == true) {
		qspi_soft_reset(qb);
	}

	sys_clear_bit(qb + XEC_QSPI_MODE_OFS, XEC_QSPI_MODE_ACTV_POS);

	v = xec_qspi_calc_fdiv(freqhz);
	mode = XEC_QSPI_MODE_CK_DIV_SET(v);

	v = qspi_signal_mode[cpp_mode];
	if (freqhz >= MHZ(48)) {
		v ^= BIT(2); /* flip CPHA_SDO bit[2] of 3-bit value */
	}

	mode |= XEC_QSPI_MODE_CP_SET(v);
	mode |= BIT(XEC_QSPI_MODE_ACTV_POS);

	sys_write32(mode, qb + XEC_QSPI_MODE_OFS);
}

#if defined(CONFIG_MSPI_DMA) && defined(CONFIG_MSPI_MCHP_XEC_USE_DMA_DRIVER)
/* Configure DMA channels in DMA driver.
 * Default callbacks are list completion and error.
 * Default handshaking is hardware.
 * We are not using callbacks.
 * This called from MSPI controller config. We do not know transfer properties such
 * as data size, data buffer alignments, etc. We can't configure XEC DMA channels here.
 * We can check if DMA devices are ready.
 */
static int mspi_xec_qspi_dma_is_ready(const struct device *controller)
{
	const struct mspi_xec_qspi_drvcfg *drvcfg = controller->config;

	if (device_is_ready(drvcfg->tx_dma_dev) == false) {
		return -EIO;
	}

	if (device_is_ready(drvcfg->rx_dma_dev) == false) {
		return -EIO;
	}

	return 0;
}

/* DMA driver callback
 * dev = pointer to DMA drive device struture
 * user_data = pointer to MSPI device structure
 * channel = DMA channel that invoked the callback
 * status = DMA_STATUS_COMPLETE all blocks in list complete,
 *          DMA_STATUS_BLOCK callback completion of each block in block list
 *          DMA_STATUS_HALF_COMPLETE callback at completion of half a block
 *          <0 error
 */
static void mspi_xec_qspi_dma_cb(const struct device *dev, void *user_data, uint32_t channel,
                                 int status)
{
	const struct device *mspi_dev = (struct device *)user_data;
	const struct mspi_xec_qspi_drvcfg *drvcfg = mspi_dev->config;
	struct mspi_xec_qspi_drv_data *drvdat = mspi_dev->data;

	drvdat->dma_status = status;
	drvdat->dma_done = true;

	if (status < 0) {
		LOG_DBG("MSPI XEC-QSPI DMA cb error (%s)", status);
	}
}

/* TODO Setup of DMA driver channels requires information:
 * TX buffer and length, RX buffer and length, and direction.
 * SPI protocol transmits a command with optional parameters.
 * Read SPI commands read data after the command.
 * TX DMA channel source_data_size is transfer width 1, 2, or 4 bytes.
 * RX DMA channel dest_data_size is transfer width 1, 2, or 4 bytes.
 * The transfer widths are calculated from buffer alignment and size.
 * We get this information from:
 * struct mspi_xfer_packet {
 *	enum mspi_xfer_direction    dir;
 *	enum mspi_bus_event_cb_mask cb_mask;
 *	uint32_t                    cmd;
 *	uint32_t                    address;
 *	uint32_t                    num_bytes;
 *	uint8_t                     *data_buf;
 * };
 * MSPI API takes a pointer to struct mspi_xfer
 * This structure contains a pointer to struct mspi_xfer_packet and
 * a number of packets member.
 */
static int mspi_xec_qspi_dma_setup(const struct device *controller)
{
	const struct mspi_xec_qspi_drvcfg *drvcfg = controller->config;
	struct mspi_xec_qspi_drv_data *drvdat = controller->data;
	struct mspi_xfr *mxfr = drvdat->mxfr;
	struct dma_config *dcfg = &drvdat->dma_cfg;
	struct dma_block_config *dbcfg = &drvdat->dma_blk_cfg;
	const struct device *dma_dev = NULL;
	const struct mspi_xfr_packet *mpkt = NULL;
	int rc = 0;
	uint8_t dma_chan = 0;

	if ((mxfr == NULL) || || (mxfr->packets == NULL) || (mxfr->packets->num_bytes == 0)) {
		return -EINVAL;
	}

	memset(dcfg, 0, sizeof(struct dma_config));
	memset(dbcfg, 0, sizeof(struct dma_block_config));

	mpkt = mxfr->packets;

	if (mpkt->dir == MSPI_TX) {
		dma_dev = drvcfg->tx_dma_dev;
		dma_chan = drvcfg->tx_dma_chan;
		dcfg->dma_slot = drvcfg->tx_dma_trigsrc;
		dcfg->channel_direction = MEMORY_TO_PERIPHERAL;
		dcfg->source_data_size = 0; /* memory */
		dcfg->dest_data_size = 0; /* QSPI.TXB register */
	} else {
		dma_dev = drvcfg->rx_dma_dev;
		dma_chan = drvcfg->rx_dma_chan;
		dcfg->dma_slot = drvcfg->rx_dma_trigsrc;
		dcfg->channel_direction = PERIPHERAL_TO_MEMORY;
		dcfg->source_data_size = 0; /* QSPI TXB register */
		dcfg->dest_data_size = 0; /* memory */
	}

	dcfg->block_count = 1u;
	dcfg->head_block = dbcfg;
	dcfg->user_data = (void *)controller;
	dcfg->dma_callback = mspi_xec_qspi_dma_cb;

	rc = dma_config(dma_dev, dma_chan, dcfg);
	if (rc != 0) {
		return rc;
	}

	return rc;
}
#endif /* CONFIG_MSPI_DMA && CONFIG_MSPI_MCHP_XEC_USE_DMA_DRIVER */

/* XEC QMSPI controller does not support acting as a target.
 * SPI flash protocols are inherently half-duplex.
 * dual-data-rate on both clock edges is not supported.
 * QMSPI supports two partially HW controlled chip selects. The driver must
 * select which of the two chip selects QMSPI can directly control.
 * GPIOs may also be used as chip selects if they are asserted and de-asserted with
 * a delay before the first SPI clock and after the last SPI clock.
 */
static int xec_mspi_check_ctrl_config(const struct mspi_cfg *mcfg, uint32_t max_freq)
{
	if (mcfg == NULL) {
		return -EINVAL;
	}

	if (mcfg->op_mode != MSPI_OP_MODE_CONTROLLER) {
		LOG_ERR("XEC QSPI Only supports MSPI controller mode.");
		return -ENOTSUP;
	}

	if (mcfg->max_freq > max_freq) {
		LOG_ERR("Requested MSPI freq (%u) > max (%u)", mcfg->max_freq, max_freq);
		return -ENOTSUP;
	}

	if (mcfg->duplex != MSPI_HALF_DUPLEX) {
		LOG_ERR("Only support half duplex mode.");
		return -ENOTSUP;
	}

	if (mcfg->sw_multi_periph == false) {
		LOG_ERR("SW-Mult-Periph must be true!");
		return -ENOTSUP;
	}

	if (mcfg->dqs_support == true) {
		LOG_ERR("HW does not support DDR");
		return -ENOTSUP;
	}

	if (mcfg->num_periph == 0) {
		LOG_ERR("No peripherals specified!");
		return -ENOTSUP;
	}

	return 0;
}

/* API mspi_config */
static int mspi_xec_qspi_config(const struct mspi_dt_spec *spec)
{
	const struct device *controller = spec->bus;
	const struct mspi_cfg *mspicfg = &spec->config;
	const struct mspi_xec_qspi_drvcfg *drvcfg = controller->config;
	struct mspi_xec_qspi_drv_data *drvdat = controller->data;
	int rc = 0;

	LOG_DBG("MSPI XEC-QSPI ctrl config");

	rc = xec_mspi_check_ctrl_config(mspicfg, drvcfg->mspicfg.max_freq);
	if (rc != 0) {
		return rc;
	}

	/* inform PM subsystem this device must be active and operational */
	(void)pm_device_runtime_get(controller);
	pm_policy_state_lock_get(PM_STATE_SUSPEND_TO_IDLE, PM_ALL_SUBSTATES);

	rc = pinctrl_apply_state_direct(drvcfg->pcfg, PINCTRL_STATE_DEFAULT);
	if (rc != 0) {
		LOG_ERR("init: pinctrl error (%d)", rc);
		goto xec_spi_config_exit;
	}


	if (drvdat->dev_cfg.dqs_enable == true) {
		LOG_ERR("SPI target device has DQS enabled but XEC-QSPI does not support DQS");
		rc = -ENOTSUP;
		goto xec_spi_config_exit;
	}

	/* Confgure QSPI controller */
	mspi_xec_qspi_ctrl_init(controller, drvcfg->clock_freq, drvcfg->cpp_mode);

	/* Driver build with MSPI_DMA enabled and use the DMA driver for XEC central DMA
	 * If MSPI_MCHP_XEC_USE_DMA_DRIVER is not enabled we will use QSPI local-DMA channels.
	 */
#if defined(CONFIG_MSPI_DMA) && defined(CONFIG_MSPI_MCHP_XEC_USE_DMA_DRIVER)
	rc = mspi_xec_qspi_dma_is_ready(controller);
	if (rc != 0) {
		LOG_ERR("XEC-QSPI DMA driver is not ready (%d)", rc);
		goto xec_api_config_exit;
	}
#endif

	/* enable GIRQ to routine QSPI interrupt to NVIC.
	 * ISR and NVIC were configured in driver init.
	 */
	soc_ecia_girq_ctrl(drvcfg->girq, drvcfg->girq_pos, MCHP_XEC_ECIA_GIRQ_EN);

	/* Release lock on device configuration */
	if (k_sem_count_get(&drvdat->dev_lock) == 0) {
		k_sem_give(&drvdat->dev_lock);
	}

	LOG_INF("XEC QSPI controller configured OK");

xec_spi_config_exit:
	pm_policy_state_lock_put(PM_STATE_SUSPEND_TO_IDLE, PM_ALL_SUBSTATES);
	(void)pm_device_runtime_put(controller);

	return rc;
}

/**
 * Validate and set frequency configuration.
 */
#if 0 /* TODO */
static int mspi_stm32_qspi_validate_and_set_freq(struct mspi_stm32_data *data, uint32_t freq,
						 uint32_t max_frequency)
{
	if (freq > max_frequency) {
		LOG_ERR("%u, freq is too large", __LINE__);
		return -ENOTSUP;
	}
	data->dev_cfg.freq = freq;
	return 0;
}
#endif /* 0 */

/**
 * Validate and set QSPI IO mode.
 * QSPI hardware doesn't support octal mode.
 */
#if 0 /* TODO */
static int mspi_stm32_qspi_validate_and_set_io_mode(struct mspi_stm32_data *data, uint32_t io_mode)
{
	if (io_mode == MSPI_IO_MODE_OCTAL) {
		LOG_ERR("%u, QSPI doesn't support octal mode", __LINE__);
		return -ENOTSUP;
	}

	if (io_mode >= MSPI_IO_MODE_MAX) {
		LOG_ERR("%u, Invalid io_mode", __LINE__);
		return -EINVAL;
	}

	data->dev_cfg.io_mode = io_mode;
	return 0;
}
#endif /* 0 */

#if 0 /* TODO */
/**
 * Validate and set QSPI data rate.
 * Only single data rate (SDR) is currently supported.
 */
static int mspi_stm32_qspi_validate_and_set_data_rate(struct mspi_stm32_data *data,
						      uint32_t data_rate)
{
	if (data_rate != MSPI_DATA_RATE_SINGLE) {
		LOG_ERR("%u, only single data rate supported", __LINE__);
		return -ENOTSUP;
	}

	if (data_rate >= MSPI_DATA_RATE_MAX) {
		LOG_ERR("%u, Invalid data_rate", __LINE__);
		return -EINVAL;
	}

	data->dev_cfg.data_rate = data_rate;
	return 0;
}
#endif /* 0 */

/**
 * Validate and set CPP (Clock Polarity/Phase).
 */
#if 0 /* TODO */
static int mspi_stm32_qspi_validate_and_set_cpp(struct mspi_stm32_data *data, uint32_t cpp)
{
	if (cpp > MSPI_CPP_MODE_3) {
		LOG_ERR("%u, Invalid cpp", __LINE__);
		return -EINVAL;
	}
	data->dev_cfg.cpp = cpp;
	return 0;
}
#endif

/**
 * Validate and set endianness.
 */
#if 0 /* XEC QMSPI only support MSBF */
static int mspi_stm32_qspi_validate_and_set_endian(struct mspi_stm32_data *data, uint32_t endian)
{
	if (endian > MSPI_XFER_BIG_ENDIAN) {
		LOG_ERR("%u, Invalid endian", __LINE__);
		return -EINVAL;
	}
	data->dev_cfg.endian = endian;
	return 0;
}
#endif /* 0 */

/**
 * Validate and set chip select polarity.
 */
#if 0 /* TODO - XEC QMSPI only supports MSBF */
static int mspi_stm32_qspi_validate_and_set_ce_polarity(struct mspi_stm32_data *data,
							uint32_t ce_polarity)
{
	if (ce_polarity > MSPI_CE_ACTIVE_HIGH) {
		LOG_ERR("%u, Invalid ce_polarity", __LINE__);
		return -EINVAL;
	}
	data->dev_cfg.ce_polarity = ce_polarity;
	return 0;
}
#endif /* 0 */

#if 0 /* XEC QMSPI does not support DDR */
/**
 * Validate and set DQS (Data Strobe) configuration.
 */
static int mspi_stm32_qspi_validate_and_set_dqs(struct mspi_stm32_data *data, bool dqs_enable,
						bool dqs_support)
{
	if (dqs_enable && !dqs_support) {
		LOG_ERR("%u, DQS mode not supported", __LINE__);
		return -ENOTSUP;
	}
	data->dev_cfg.dqs_enable = dqs_enable;
	return 0;
}
#endif /* 0 */

#if 0 /* TODO */
/**
 * @brief Set transfer-related configuration parameters
 *
 * @param data Pointer to device data
 * @param param_mask Configuration mask
 * @param dev_cfg Device configuration to apply
 */
static void mspi_stm32_qspi_set_transfer_params(struct mspi_stm32_data *data,
						const enum mspi_dev_cfg_mask param_mask,
						const struct mspi_dev_cfg *dev_cfg)
{
	if ((param_mask & MSPI_DEVICE_CONFIG_RX_DUMMY) != 0) {
		data->dev_cfg.rx_dummy = dev_cfg->rx_dummy;
	}
	if ((param_mask & MSPI_DEVICE_CONFIG_TX_DUMMY) != 0) {
		data->dev_cfg.tx_dummy = dev_cfg->tx_dummy;
	}
	if ((param_mask & MSPI_DEVICE_CONFIG_READ_CMD) != 0) {
		data->dev_cfg.read_cmd = dev_cfg->read_cmd;
	}
	if ((param_mask & MSPI_DEVICE_CONFIG_WRITE_CMD) != 0) {
		data->dev_cfg.write_cmd = dev_cfg->write_cmd;
	}
	if ((param_mask & MSPI_DEVICE_CONFIG_CMD_LEN) != 0) {
		data->dev_cfg.cmd_length = dev_cfg->cmd_length;
	}
	if ((param_mask & MSPI_DEVICE_CONFIG_ADDR_LEN) != 0) {
		data->dev_cfg.addr_length = dev_cfg->addr_length;
	}
	if ((param_mask & MSPI_DEVICE_CONFIG_MEM_BOUND) != 0) {
		data->dev_cfg.mem_boundary = dev_cfg->mem_boundary;
	}
	if ((param_mask & MSPI_DEVICE_CONFIG_BREAK_TIME) != 0) {
		data->dev_cfg.time_to_break = dev_cfg->time_to_break;
	}
}
#endif /* 0 */

#if 0 /* TODO */
/**
 * Check and save dev_cfg to controller data->dev_cfg.
 *
 * @param controller Pointer to the device structure for the driver instance.
 * @param param_mask Macro definition of what to be configured in cfg.
 * @param dev_cfg The device runtime configuration for the MSPI controller.
 * @return 0 MSPI device configuration successful.
 * @return A negative errno value upon failure.
 */
static int mspi_stm32_qspi_dev_cfg_save(const struct device *controller,
					const enum mspi_dev_cfg_mask param_mask,
					const struct mspi_dev_cfg *dev_cfg)
{
	const struct mspi_stm32_conf *cfg = controller->config;
	struct mspi_stm32_data *data = controller->data;
	int ret = 0;

	if ((param_mask & MSPI_DEVICE_CONFIG_CE_NUM) != 0) {
		data->dev_cfg.ce_num = dev_cfg->ce_num;
	}

	if ((param_mask & MSPI_DEVICE_CONFIG_FREQUENCY) != 0) {
		ret = mspi_stm32_qspi_validate_and_set_freq(data, dev_cfg->freq,
							    cfg->mspicfg.max_freq);
		if (ret != 0) {
			return ret;
		}
	}

	if ((param_mask & MSPI_DEVICE_CONFIG_IO_MODE) != 0) {
		ret = mspi_stm32_qspi_validate_and_set_io_mode(data, dev_cfg->io_mode);
		if (ret != 0) {
			return ret;
		}
	}

	if ((param_mask & MSPI_DEVICE_CONFIG_DATA_RATE) != 0) {
		ret = mspi_stm32_qspi_validate_and_set_data_rate(data, dev_cfg->data_rate);
		if (ret != 0) {
			return ret;
		}
	}

	if ((param_mask & MSPI_DEVICE_CONFIG_CPP) != 0) {
		ret = mspi_stm32_qspi_validate_and_set_cpp(data, dev_cfg->cpp);
		if (ret != 0) {
			return ret;
		}
	}

	if ((param_mask & MSPI_DEVICE_CONFIG_ENDIAN) != 0) {
		ret = mspi_stm32_qspi_validate_and_set_endian(data, dev_cfg->endian);
		if (ret != 0) {
			return ret;
		}
	}

	if ((param_mask & MSPI_DEVICE_CONFIG_CE_POL) != 0) {
		ret = mspi_stm32_qspi_validate_and_set_ce_polarity(data, dev_cfg->ce_polarity);
		if (ret != 0) {
			return ret;
		}
	}

	if ((param_mask & MSPI_DEVICE_CONFIG_DQS) != 0) {
		ret = mspi_stm32_qspi_validate_and_set_dqs(data, dev_cfg->dqs_enable,
							   cfg->mspicfg.dqs_support);
		if (ret != 0) {
			return ret;
		}
	}

	/* Set transfer-related configuration parameters */
	mspi_stm32_qspi_set_transfer_params(data, param_mask, dev_cfg);

	return 0;
}
#endif /* 0 */

/**
 * API implementation of mspi_dev_config : controller device specific configuration
 *
 * @param controller Pointer to the device structure for the driver instance.
 * @param dev_id Pointer to the device ID structure from a device.
 * @param param_mask Macro definition of what to be configured in cfg.
 * @param dev_cfg The device runtime configuration for the MSPI controller.
 *
 * @retval 0 if successful.
 * @retval -EINVAL invalid capabilities, failed to configure device.
 * @retval -ENOTSUP capability not supported by MSPI peripheral.
 */
#if 0
static int mspi_stm32_qspi_dev_config(const struct device *controller,
				      const struct mspi_dev_id *dev_id,
				      const enum mspi_dev_cfg_mask param_mask,
				      const struct mspi_dev_cfg *dev_cfg)
{
	const struct mspi_stm32_conf *cfg = controller->config;
	struct mspi_stm32_data *data = controller->data;
	bool locked = false;
	int ret = 0;

	/* Check if device ID has changed and lock accordingly */
	if (data->dev_id != dev_id) {
		if (k_mutex_lock(&data->lock, K_MSEC(CONFIG_MSPI_COMPLETION_TIMEOUT_TOLERANCE))) {
			LOG_ERR("Failed to acquire lock for device config");
			return -EBUSY;
		}

		locked = true;
	}

	if (mspi_is_inp(controller)) {
		ret = -EBUSY;
		goto e_return;
	}

	if (param_mask == MSPI_DEVICE_CONFIG_NONE && !cfg->mspicfg.sw_multi_periph) {
		/* Nothing to do but saving the device ID */
		data->dev_id = dev_id;
		goto e_return;
	}

	(void)pm_device_runtime_get(controller);
	/* Prevent the clocks to be stopped during the request */
	pm_policy_state_lock_get(PM_STATE_SUSPEND_TO_IDLE, PM_ALL_SUBSTATES);

	data->dev_id = dev_id;
	/* Validate and save device configuration */
	ret = mspi_stm32_qspi_dev_cfg_save(controller, param_mask, dev_cfg);
	if (ret != 0) {
		LOG_ERR("failed to change device cfg");
	}

	/* Release PM resources */
	pm_policy_state_lock_put(PM_STATE_SUSPEND_TO_IDLE, PM_ALL_SUBSTATES);
	(void)pm_device_runtime_put(controller);

e_return:
	if (locked) {
		k_mutex_unlock(&data->lock);
	}

	return ret;
}
#else

#if 0
enum mspi_dev_cfg_mask {
	MSPI_DEVICE_CONFIG_NONE         = 0,
	MSPI_DEVICE_CONFIG_CE_NUM       = BIT(0),
	MSPI_DEVICE_CONFIG_FREQUENCY    = BIT(1),
	MSPI_DEVICE_CONFIG_IO_MODE      = BIT(2),
	MSPI_DEVICE_CONFIG_DATA_RATE    = BIT(3),
	MSPI_DEVICE_CONFIG_CPP          = BIT(4),
	MSPI_DEVICE_CONFIG_ENDIAN       = BIT(5),
	MSPI_DEVICE_CONFIG_CE_POL       = BIT(6),
	MSPI_DEVICE_CONFIG_DQS          = BIT(7),
	MSPI_DEVICE_CONFIG_RX_DUMMY     = BIT(8),
	MSPI_DEVICE_CONFIG_TX_DUMMY     = BIT(9),
	MSPI_DEVICE_CONFIG_READ_CMD     = BIT(10),
	MSPI_DEVICE_CONFIG_WRITE_CMD    = BIT(11),
	MSPI_DEVICE_CONFIG_CMD_LEN      = BIT(12),
	MSPI_DEVICE_CONFIG_ADDR_LEN     = BIT(13),
	MSPI_DEVICE_CONFIG_MEM_BOUND    = BIT(14),
	MSPI_DEVICE_CONFIG_BREAK_TIME   = BIT(15),
	MSPI_DEVICE_CONFIG_ALL          = BIT_MASK(16),
};
#endif /* 0 */

static int mspi_xec_qspi_dev_cfg_update(const struct device *controller,
                                        const enum mspi_dev_cfg_mask param_mask,
                                        const struct mspi_dev_cfg *dev_cfg)
{
	const struct mspi_xec_qspi_drvcfg *drvcfg = controller->config;
	struct mspi_xec_qspi_drv_data *drvdat = controller->data;
	int rc = 0;

	return rc;
}

/* TODO Configure QSPI for the specific device configuration based on param_mask */
static int mspi_xec_qspi_dev_config(const struct device *controller,
				    const struct mspi_dev_id *dev_id,
				    const enum mspi_dev_cfg_mask param_mask,
				    const struct mspi_dev_cfg *dev_cfg)
{
	struct mspi_xec_qspi_drv_data *drvdat = controller->data;
	int rc = 0;
	bool is_locked = false;

	/* Check if device ID has changed and lock accordingly */
	if (drvdat->dev_id != dev_id) {
		if (k_mutex_lock(&drvdat->lock, K_MSEC(CONFIG_MSPI_COMPLETION_TIMEOUT_TOLERANCE))) {
			LOG_ERR("MSPI XEC-QSPI device config is locked");
			return -EBUSY;
		}

		is_locked = true;
	}

	if (mspi_xec_qspi_is_busy(controller) == true) {
		rc = -EBUSY;
		goto mspi_xec_qspi_dev_cfg_exit;
	}

	if (param_mask == MSPI_DEVICE_CONFIG_NONE) {
		drvdat->dev_id = dev_id; /* No params to change. Update device ID */
		goto mspi_xec_qspi_dev_cfg_exit;
	}

	(void)pm_device_runtime_get(controller);
	/* Prevent the clocks to be stopped during the request */
	pm_policy_state_lock_get(PM_STATE_SUSPEND_TO_IDLE, PM_ALL_SUBSTATES);

	drvdat->dev_id = dev_id;

	rc = mspi_xec_qspi_dev_cfg_update(controller, param_mask, dev_cfg);
	if (rc != 0) {
		LOG_ERR("MSPI XEC-QSPI dev_cfg error (%d)", rc);
	}

#if 0 /* TODO */
	ret = mspi_stm32_qspi_dev_cfg_save(controller, param_mask, dev_cfg);
	if (ret != 0) {
		LOG_ERR("failed to change device cfg");
	}
#endif
	pm_policy_state_lock_put(PM_STATE_SUSPEND_TO_IDLE, PM_ALL_SUBSTATES);
	(void)pm_device_runtime_put(controller);

mspi_xec_qspi_dev_cfg_exit:
	if (is_locked == true) {
		k_mutex_unlock(&drvdat->lock);
	}

	return rc;
}
#endif

/* API mspi_get_channel_status */
static int mspi_xec_qspi_get_channel_status(const struct device *controller, uint8_t ch)
{
	int rc = 0;

	if (mspi_xec_qspi_is_busy(controller) == true) {
		rc = -EBUSY;
	}

	return rc;
}

/* TODO
 * Vendor defined timing config.
 * XEC QSPI we support
 * chip select timing all in units of 48MHz or 96MHz clocks?
 *    delay CS off to CS on
 *    delay CS de-assertion to HW switching IO[2:3] to nWP & nHOLD.
 *    delay from last clock edge to CS de-assertion
 *    delay from CS assertion to first clock edge
 *
 * internal sample taps
 *   Clock TAP select: 8 bits
 *   Control TAP select: 8 bits
 *   Clock TAP adjust value: signed 8-bits
 *   Control TAPI adjust value: signed 8-bits
 *
 * internal taps control
 *   auto-mode: 0=Off, 1=ON, 2=ON_Periodic (100kHz)
 *   auto-multiplier: 3-bits. 0=Max(8). 1-7 are multipliers on 50% of the system clock.
 */
static int mspi_xec_qspi_timing_config(const struct device *controller,
				       const struct mspi_dev_id *dev_id,
				       const uint32_t param_mask, void *cfg)
{
	return -ENOTSUP;
}

/* Support callbacks for events:
 * MSPI_BUS_RESET = 0 TODO can we support this?
 * MSPI_BUS_ERROR = 1
 * MSPI_BUS_XFER_COMPLETE = 2
 * MSPI_BUS_TIMEROUT = 3
 * Do we need an array or can we use a bitmap?
 */
static int mspi_xec_qspi_register_cb(const struct device *controller,
				     const struct mspi_dev_id *dev_id,
				     const enum mspi_bus_event evt_type,
				     mspi_callback_handler_t cb,
				     struct mspi_callback_context *ctx)
{
	struct mspi_xec_qspi_drv_data *const drvdat = controller->data;

	if (mspi_xec_qspi_is_busy(controller) == true) {
		return -EBUSY;
	}

	if (dev_id != drvdat->dev_id) {
		return -ESTALE;
	}

	if (evt_type >= MSPI_BUS_EVENT_MAX) {
		return -EINVAL;
	}

	drvdat->cb_ctx[evt_type] = ctx;
	drvdat->cb[evt_type] = cb;

	return 0;
}

/* TODO */
#if 0
static int mspi_stm32_qspi_pio_transceive(const struct device *controller,
					  const struct mspi_xfer *xfer)
{
	struct mspi_stm32_data *dev_data = controller->data;
	struct mspi_stm32_context *ctx = &dev_data->ctx;
	const struct mspi_xfer_packet *packet;
	uint32_t packet_idx;
	int ret = 0;

	if (xfer->num_packet == 0 || xfer->packets == NULL ||
	    xfer->timeout > CONFIG_MSPI_COMPLETION_TIMEOUT_TOLERANCE) {
		LOG_ERR("Transfer: wrong parameters");
		return -EFAULT;
	}

	/* Acquire the context lock (semaphore) */
	if (k_sem_take(&ctx->lock, K_MSEC(xfer->timeout)) < 0) {
		return -EBUSY;
	}

	ctx->xfer = *xfer;
	ctx->packets_left = ctx->xfer.num_packet;

	while (ctx->packets_left > 0) {
		packet_idx = ctx->xfer.num_packet - ctx->packets_left;
		packet = &ctx->xfer.packets[packet_idx];

		/*
		 * Always starts with a command,
		 * then payload is given by the xfer->num_packet
		 */
		ret = mspi_stm32_qspi_access(controller, packet, ctx->xfer.async ?
					     MSPI_ACCESS_ASYNC : MSPI_ACCESS_SYNC);

		if (ret != 0) {
			LOG_ERR("QSPI access failed for packet %d: %d", packet_idx, ret);
			ret = -EIO;
			goto out;
		}

		ctx->packets_left--;
	}

out:
	k_sem_give(&ctx->lock);
	return ret;
}
#endif /* 0 */

/**
 * API implementation of mspi_transceive.
 *
 * @param controller Pointer to the device structure for the driver instance.
 * @param dev_id Pointer to the device ID structure from a device.
 * @param xfer Pointer to the MSPI transfer started by dev_id.
 *
 * @retval 0 if successful.
 * @retval -ESTALE device ID don't match, need to call mspi_dev_config first.
 * @retval A negative errno value upon failure.
 */
#if 0
static int mspi_stm32_qspi_transceive(const struct device *controller,
				      const struct mspi_dev_id *dev_id,
				      const struct mspi_xfer *xfer)
{
	struct mspi_stm32_data *data = controller->data;
	int ret = 0;

	/* Verify device ID matches */
	if (dev_id != data->dev_id) {
		LOG_ERR("transceive : dev_id don't match");
		return -ESTALE;
	}

	/* Need to map the xfer to the data context */
	data->ctx.xfer = *xfer;

	if (xfer->xfer_mode == MSPI_PIO) {
		ret = mspi_stm32_qspi_pio_transceive(controller, xfer);
	} else {
		ret = -EIO;
	}

	return ret;
}
#else
/* TODO */
static int mspi_xec_qspi_transceive(const struct device *controller,
				    const struct mspi_dev_id *dev_id, const struct mspi_xfer *xfer)
{
	return -ENOTSUP;
}
#endif

/**
 * @brief QSPI ISR function
 */
#if 0
static void mspi_stm32_qspi_isr(const struct device *dev)
{
	struct mspi_stm32_data *dev_data = dev->data;

	HAL_QSPI_IRQHandler(&dev_data->hmspi.qspi);

	k_sem_give(&dev_data->sync);
	pm_policy_state_lock_put(PM_STATE_SUSPEND_TO_IDLE, PM_ALL_SUBSTATES);
	(void)pm_device_runtime_put(dev);
}
#else
/* TODO */
static void mspi_xec_qspi_isr(const struct device *dev)
{
	const struct mspi_xec_qspi_drvcfg *drvcfg = dev->config;
	struct mspi_xec_qspi_drv_data *const drvdat = dev->data;
	mm_reg_t qb = drvcfg->qbase;

	drvdat->hwsts = sys_read32(qb + XEC_QSPI_SR_OFS);

	k_sem_give(&drvdat->sync);
	pm_policy_state_lock_put(PM_STATE_SUSPEND_TO_IDLE, PM_ALL_SUBSTATES);
	(void)pm_device_runtime_put(dev);
}
#endif

#ifdef CONFIG_PM_DEVICE
/**
 * @brief Power management action callback
 */
#if 0
static int mspi_stm32_qspi_pm_action(const struct device *dev, enum pm_device_action action)
{
	const struct mspi_stm32_conf *cfg = dev->config;
	struct mspi_stm32_data *dev_data = dev->data;
	int ret;

	switch (action) {
	case PM_DEVICE_ACTION_RESUME:
		ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
		if (ret < 0) {
			LOG_ERR("Cannot apply default pins state (%d)", ret);
			return ret;
		}

		/* Re-enable clock */
		if (clock_control_on(DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE),
				     (clock_control_subsys_t)&cfg->pclken[0]) != 0) {
			LOG_ERR("Could not enable MSPI clock on resume");
			return -EIO;
		}

		LOG_DBG("QSPI resumed");
		return 0;

	case PM_DEVICE_ACTION_SUSPEND:
		ret = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_SLEEP);
		if (ret < 0 && ret != -ENOENT) {
			LOG_ERR("Cannot apply sleep pins state (%d)", ret);
			return ret;
		}

		/* Check if XIP is enabled or if controller is in use */
		if (dev_data->xip_cfg.enable || k_mutex_lock(&dev_data->lock, K_NO_WAIT) != 0) {
			LOG_ERR("Controller in use, cannot be suspended");
			return -EBUSY;
		}

		/* Disable QSPI peripheral */
		if (HAL_QSPI_DeInit(&dev_data->hmspi.qspi) != HAL_OK) {
			LOG_WRN("HAL_QSPI_DeInit failed during suspend");
		}

		/* Disable clock */
		if (clock_control_off(DEVICE_DT_GET(STM32_CLOCK_CONTROL_NODE),
				      (clock_control_subsys_t)&cfg->pclken[0]) != 0) {
			LOG_WRN("Could not disable MSPI clock on suspend");
		}

		k_mutex_unlock(&dev_data->lock);

		LOG_DBG("QSPI suspended");
		return 0;

	default:
		return -ENOTSUP;
	}
}
#else
/* XEC QMSPI obeys PCR block's sleep enable signal. Once it reaches a unit size boundary
 * of 1, 2, or 4 bytes it will pause and clear its CLK_REQ signal back to PCR.
 * We can't disable the pins until we know QMSPI is idle. The driver must make sure QMSPI
 * is idle before informing the kernel this driver can enter a low power mode.
 */
static int mspi_xec_qspi_pm_action(const struct device *dev, enum pm_device_action action)
{
	int rc = 0;

	switch (action) {
	case PM_DEVICE_ACTION_RESUME:
		rc = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_DEFAULT);
		if (rc < 0) {
			LOG_ERR("PM device resume pinctrl error (%d)", rc);
			break;
		}
		break;
	case PM_DEVICE_ACTION_SUSPEND:
		rc = pinctrl_apply_state(cfg->pcfg, PINCTRL_STATE_SLEEP);
		if (rc < 0 && rc != -ENOENT) {
			LOG_ERR("PM device suspend pinctrl error (%d)", rc);
			break;
		}
		break;
	default:
		return -ENOTSUP;
	}

	return rc;
}
#endif /* 0 */
#endif /* CONFIG_PM_DEVICE */

static int mspi_xec_qspi_init(const struct device *controller)
{
	const struct mspi_xec_qspi_drvcfg *drvcfg = controller->config;
	const struct mspi_dt_spec spec = {
		.bus = controller,
		.config = drvcfg->mspicfg,
	};

	/* attach ISR and enable NVIC. We keep GIRQ disabled until controller config */
	if (drvcfg->irq_cfg_fp != NULL) {
		drvcfg->irq_cfg_fp();
	}

	return mspi_xec_qspi_config(&spec);
}

static DEVICE_API(mspi, mspi_xec_qspi_driver_api) = {
	.config = mspi_xec_qspi_config,
	.dev_config = mspi_xec_qspi_dev_config,
	.get_channel_status = mspi_xec_qspi_get_channel_status,
	.transceive = mspi_xec_qspi_transceive,
	.register_callback = mspi_xec_qspi_register_cb,
	.timing_config = mspi_xec_qspi_timing_config,
};

#if defined(CONFIG_MSPI_DMA)
#if defined(CONFIG_MSPI_MCHP_XEC_USE_DMA_DRIVER)

#define MSPI_XEC_QSPI_DMA_NODE(i, name)    DT_INST_DMAS_CTLR_BY_NAME(i, name)
#define MSPI_XEC_QSPI_DMA_DEVICE(i, name)  DEVICE_DT_GET(MSPI_XEC_QSPI_DMA_NODE(i, name))
#define MSPI_XEC_QSPI_DMA_CHAN(i, name)    DT_INST_DMAS_CELL_BY_NAME(i, name, channel)
#define MSPI_XEC_QSPI_DMA_TRIGSRC(i, name) DT_INST_DMAS_CELL_BY_NAME(i, name, trigsrc)

#define MSPI_XEC_QSPI_DMA_DEVCFG(inst) \
	.tx_dma_dev = MSPI_XEC_QSPI_DMA_DEVICE(inst, tx), \
	.rx_dma_dev = MSPI_XEC_QSPI_DMA_DEVICE(inst, rx), \
	.tx_dma_chan = MSPI_XEC_QSPI_DMA_CHAN(inst, tx), \
	.tx_dma_trigsrc = MSPI_XEC_QSPI_DMA_TRIGSRC(inst, tx), \
	.rx_dma_chan = MSPI_XEC_QSPI_DMA_CHAN(inst, rx), \
	.rx_dma_trigsrc = MSPI_XEC_QSPI_DMA_TRIGSRC(inst, rx),
#else /* Local-DMA */
#define MSPI_XEC_QSPI_DMA_DEVCFG(inst)
#endif
#else
#define MSPI_XEC_QSPI_DMA_DEVCFG(inst)
#endif /* CONFIG_MSPI_DMA */

#define MSPI_XEC_QSPI_GIRQ(inst)     MCHP_XEC_ECIA_GIRQ(DT_INST_PROP_BY_IDX(inst, girqs, 0))
#define MSPI_XEC_QSPI_GIRQ_POS(inst) MCHP_XEC_ECIA_GIRQ_POS(DT_INST_PROP_BY_IDX(inst, girqs, 0))

/* MCHP XEC QSPI controller properities:
 * SPI data width: full-duplex, dual-io, and quad-io. Should we set MSPI op_mode to full or
 * half duplex?
 * Hardware does not support DDR set dqs_support to false.
 * QSPI supports two hardare controlled chip selects selectable by software and can
 * support GPIO chip selects. We set sw_mult_periph to true indicating the driver must
 * configure the hardware or GPIO chip select.
 */
#define XEC_MSPI_QSPI_CONFIG(inst) \
	{ \
		.channel_num = 0, \
		.op_mode = DT_INST_ENUM_IDX_OR(inst, op_mode, MSPI_OP_MODE_CONTROLLER), \
		.duplex = DT_INST_ENUM_IDX_OR(inst, duplex, MSPI_HALF_DUPLEX), \
		.max_freq = DT_INST_PROP(inst, clock_frequency), \
		.dqs_support = false, \
		.num_periph = DT_INST_CHILD_NUM(inst), \
		.sw_multi_periph = true, \
		.num_ce_gpios = ARRAY_SIZE(ce_gpios##inst), \
		.ce_group = ce_gpios##inst, \
	}

#define MSPI_XEC_QSPI_INIT(inst) \
	PM_DEVICE_DT_INST_DEFINE(inst, mspi_xec_qspi_pm_action); \
	static struct mspi_xec_qspi_drv_data mspi_xec_qspi_drv_dat##inst = { \
		.lock = Z_MUTEX_INITIALIZER(mspi_xec_qspi_drv_dat##inst.lock), \
		.dev_lock = Z_SEM_INITIALIZER(mspi_xec_qspi_drv_dat##inst.dev_lock, 0, 1), \
		.sync = Z_SEM_INITIALIZER(mspi_xec_qspi_drv_dat##inst.sync, 0, 1), \
	}; \
	PINCTRL_DT_INST_DEFINE(inst); \
	static struct gpio_dt_spec ce_gpios##inst[] = MSPI_CE_GPIOS_DT_SPEC_INST_GET(inst); \
	static void mspi_xec_qspi_irq_cfg##inst(void) { \
		IRQ_CONNECT(DT_INST_IRQN(inst), DT_INST_IRQ(inst, priority), mspi_xec_qspi_isr, \
			    DEVICE_DT_INST_GET(inst), 0); \
		irq_enable(DT_INST_IRQN(inst)); \
	} \
	static const struct mspi_xec_qspi_drvcfg mspi_xec_qspi_drv_cfg##inst = { \
		.qbase = (mm_reg_t)DT_INST_REG_ADDR(inst), \
		.clock_freq = DT_INST_PROP_OR(inst, clock_frequency, MHZ(12)), \
		.enc_pcr = (uint16_t)DT_INST_PROP(inst, pcr), \
		.girq = MSPI_XEC_QSPI_GIRQ(inst), \
		.girq_pos = MSPI_XEC_QSPI_GIRQ_POS(inst), \
		.cpp_mode = DT_INST_PROP_OR(inst, cpp_mode, MSPI_CPP_MODE_0), \
		.mspicfg = XEC_MSPI_QSPI_CONFIG(inst), \
		.pcfg = PINCTRL_DT_INST_DEV_CONFIG_GET(inst), \
		.irq_cfg_fp = mspi_xec_qspi_irq_cfg##inst, \
		MSPI_XEC_QSPI_DMA_DEVCFG(inst) \
	}; \
	DEVICE_DT_INST_DEFINE(inst, &mspi_xec_qspi_init, \
			      PM_DEVICE_DT_INST_GET(inst), \
			      &mspi_xec_qspi_drv_dat##inst, \
			      &mspi_xec_qspi_drv_cfg##inst, POST_KERNEL, \
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, &mspi_xec_qspi_driver_api);

DT_INST_FOREACH_STATUS_OKAY(MSPI_XEC_QSPI_INIT)
