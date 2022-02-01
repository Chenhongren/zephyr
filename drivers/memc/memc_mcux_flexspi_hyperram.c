/*
 * Copyright 2021 Basalte bv
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT   nxp_imx_flexspi_hyperram

#include <logging/log.h>
#include <sys/util.h>

#include "memc_mcux_flexspi.h"


/*
 * NOTE: If CONFIG_FLASH_MCUX_FLEXSPI_XIP is selected, Any external functions
 * called while interacting with the flexspi MUST be relocated to SRAM or ITCM
 * at runtime, so that the chip does not access the flexspi to read program
 * instructions while it is being written to
 */
#if defined(CONFIG_FLASH_MCUX_FLEXSPI_XIP) && (CONFIG_MEMC_LOG_LEVEL > 0)
#warning "Enabling memc driver logging and XIP mode simultaneously can cause \
	read-while-write hazards. This configuration is not recommended."
#endif

LOG_MODULE_REGISTER(memc_flexspi, CONFIG_MEMC_LOG_LEVEL);

enum {
	READ_DATA,
	WRITE_DATA,
	READ_REG,
	WRITE_REG,
};

struct memc_flexspi_hyperram_config {
	flexspi_port_t port;
	flexspi_device_config_t config;
};

static const uint32_t memc_flexspi_hyperram_lut[][4] = {
	/* Read Data */
	[READ_DATA] = {
		FLEXSPI_LUT_SEQ(kFLEXSPI_Command_DDR,             kFLEXSPI_8PAD, 0xA0,
				kFLEXSPI_Command_RADDR_DDR,       kFLEXSPI_8PAD, 0x18),
		FLEXSPI_LUT_SEQ(kFLEXSPI_Command_CADDR_DDR,       kFLEXSPI_8PAD, 0x10,
				kFLEXSPI_Command_DUMMY_RWDS_DDR,  kFLEXSPI_8PAD, 0x06),
		FLEXSPI_LUT_SEQ(kFLEXSPI_Command_READ_DDR,        kFLEXSPI_8PAD, 0x04,
				kFLEXSPI_Command_STOP,            kFLEXSPI_1PAD, 0x00),
	},

	/* Write Data */
	[WRITE_DATA] = {
		FLEXSPI_LUT_SEQ(kFLEXSPI_Command_DDR,             kFLEXSPI_8PAD, 0x20,
				kFLEXSPI_Command_RADDR_DDR,       kFLEXSPI_8PAD, 0x18),
		FLEXSPI_LUT_SEQ(kFLEXSPI_Command_CADDR_DDR,       kFLEXSPI_8PAD, 0x10,
				kFLEXSPI_Command_DUMMY_RWDS_DDR,  kFLEXSPI_8PAD, 0x06),
		FLEXSPI_LUT_SEQ(kFLEXSPI_Command_WRITE_DDR,       kFLEXSPI_8PAD, 0x04,
				kFLEXSPI_Command_STOP,            kFLEXSPI_1PAD, 0x00),
	},

	/* Read Register */
	[READ_REG] = {
		FLEXSPI_LUT_SEQ(kFLEXSPI_Command_DDR,             kFLEXSPI_8PAD, 0xE0,
				kFLEXSPI_Command_RADDR_DDR,       kFLEXSPI_8PAD, 0x18),
		FLEXSPI_LUT_SEQ(kFLEXSPI_Command_CADDR_DDR,       kFLEXSPI_8PAD, 0x10,
				kFLEXSPI_Command_DUMMY_RWDS_DDR,  kFLEXSPI_8PAD, 0x06),
		FLEXSPI_LUT_SEQ(kFLEXSPI_Command_READ_DDR,        kFLEXSPI_8PAD, 0x04,
				kFLEXSPI_Command_STOP,            kFLEXSPI_1PAD, 0x00),
	},

	/* Write Register */
	[WRITE_REG] = {
		FLEXSPI_LUT_SEQ(kFLEXSPI_Command_DDR,             kFLEXSPI_8PAD, 0x60,
				kFLEXSPI_Command_RADDR_DDR,       kFLEXSPI_8PAD, 0x18),
		FLEXSPI_LUT_SEQ(kFLEXSPI_Command_CADDR_DDR,       kFLEXSPI_8PAD, 0x10,
				kFLEXSPI_Command_DUMMY_RWDS_DDR,  kFLEXSPI_8PAD, 0x06),
		FLEXSPI_LUT_SEQ(kFLEXSPI_Command_WRITE_DDR,       kFLEXSPI_8PAD, 0x04,
				kFLEXSPI_Command_STOP,            kFLEXSPI_1PAD, 0x00),
	},
};

static int memc_flexspi_hyperram_get_vendor_id(const struct device *dev,
						uint16_t *vendor_id)
{
	const struct memc_flexspi_hyperram_config *config = dev->config;
	uint32_t buffer = 0;
	int ret;

	flexspi_transfer_t transfer = {
		.deviceAddress = 0,
		.port = config->port,
		.cmdType = kFLEXSPI_Read,
		.SeqNumber = 1,
		.seqIndex = READ_REG,
		.data = &buffer,
		.dataSize = 4,
	};

	LOG_DBG("Reading id");

	ret = memc_flexspi_transfer(config->controller, &transfer);
	*vendor_id = buffer & 0xffff;

	return ret;
}

static int memc_flexspi_hyperram_init(const struct device *dev)
{
	const struct memc_flexspi_hyperram_config *config = dev->config;
	uint16_t vendor_id;

	if (!device_is_ready(config->controller)) {
		LOG_ERR("Controller device not ready");
		return -ENODEV;
	}

	if (memc_flexspi_set_device_config(config->controller, &config->config,
					   config->port)) {
		LOG_ERR("Could not set device configuration");
		return -EINVAL;
	}

	if (memc_flexspi_update_lut(config->controller, 0,
				    (const uint32_t *) memc_flexspi_hyperram_lut,
				    sizeof(memc_flexspi_hyperram_lut) / 4)) {
		LOG_ERR("Could not update lut");
		return -EINVAL;
	}

	memc_flexspi_reset(config->controller);

	if (memc_flexspi_hyperram_get_vendor_id(dev, &vendor_id)) {
		LOG_ERR("Could not read vendor id");
		return -EIO;
	}
	LOG_DBG("Vendor id: 0x%0x", vendor_id);

	return 0;
}

#define CONCAT3(x, y, z) x ## y ## z

#define CS_INTERVAL_UNIT(unit) \
	CONCAT3(kFLEXSPI_CsIntervalUnit, unit, SckCycle)

#define AHB_WRITE_WAIT_UNIT(unit) \
	CONCAT3(kFLEXSPI_AhbWriteWaitUnit, unit, AhbCycle)

#define MEMC_FLEXSPI_DEVICE_CONFIG(n)					\
	{								\
		.flexspiRootClk = MHZ(332),				\
		.isSck2Enabled = false,					\
		.flashSize = DT_INST_PROP(n, size) / 8 / KB(1),		\
		.CSIntervalUnit =					\
			CS_INTERVAL_UNIT(				\
				DT_INST_PROP(n, cs_interval_unit)),	\
		.CSInterval = DT_INST_PROP(n, cs_interval),		\
		.CSHoldTime = DT_INST_PROP(n, cs_hold_time),		\
		.CSSetupTime = DT_INST_PROP(n, cs_setup_time),		\
		.dataValidTime = DT_INST_PROP(n, data_valid_time),	\
		.columnspace = DT_INST_PROP(n, column_space),		\
		.enableWordAddress = DT_INST_PROP(n, word_addressable),	\
		.AWRSeqIndex = WRITE_DATA,				\
		.AWRSeqNumber = 1,					\
		.ARDSeqIndex = READ_DATA,				\
		.ARDSeqNumber = 1,					\
		.AHBWriteWaitUnit =					\
			AHB_WRITE_WAIT_UNIT(				\
				DT_INST_PROP(n, ahb_write_wait_unit)),	\
		.AHBWriteWaitInterval =					\
			DT_INST_PROP(n, ahb_write_wait_interval),	\
		.enableWriteMask = true,				\
	}								\

#define MEMC_FLEXSPI_HYPERRAM(n)				  \
	static const struct memc_flexspi_hyperram_config	  \
		memc_flexspi_hyperram_config_##n = {		  \
		.controller = DEVICE_DT_GET(DT_INST_BUS(n)),	  \
		.port = DT_INST_REG_ADDR(n),			  \
		.config = MEMC_FLEXSPI_DEVICE_CONFIG(n),	  \
	};							  \
								  \
	DEVICE_DT_INST_DEFINE(n,				  \
			      memc_flexspi_hyperram_init,	  \
			      NULL,				  \
			      NULL,				  \
			      &memc_flexspi_hyperram_config_##n,  \
			      POST_KERNEL,			  \
			      CONFIG_KERNEL_INIT_PRIORITY_DEVICE, \
			      NULL);

DT_INST_FOREACH_STATUS_OKAY(MEMC_FLEXSPI_HYPERRAM)
