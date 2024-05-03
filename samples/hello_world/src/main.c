/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>

#include <zephyr/drivers/spi.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(flash_test, LOG_LEVEL_ERR);

static struct spi_config spi_cfg = {
	.operation = SPI_WORD_SET(8),
	.frequency = 1500000,
	.slave = 0,
};

void read_jedec_id()
{
	struct device *spi_device = (struct device *)DEVICE_DT_GET(DT_NODELABEL(spi0));

	uint8_t buf[1] = {0x9F};
	uint8_t data[3] = { 0 };
	struct spi_buf spi_buf[2] = {
		{
			.buf = buf,
			.len = 1,
		},
		{
			.buf = data,
			.len = 3
		}
	};

	const struct spi_buf_set tx_set = {.buffers = spi_buf, .count = 2};
	const struct spi_buf_set rx_set = {.buffers = spi_buf, .count = 2};
	int ret;

	ret = spi_transceive(spi_device, &spi_cfg, &tx_set, &rx_set);

	if (ret) {
		LOG_ERR("Failed to send SPI data: %d\n", ret);
		return;
	}

	LOG_HEXDUMP_ERR(spi_buf[1].buf, spi_buf[1].len, "JEDEC ID:");
}

int main(void)
{
	// uint32_t sspi_clk = KHZ(24000);
	// spi_cfg.frequency = sspi_clk / 16;
	// LOG_ERR("frequency= %d", spi_cfg.frequency);

	// LOG_ERR("Chip select 0");
	// spi_cfg.slave = 0;
	// read_jedec_id();
	// k_sleep(K_MSEC(3000));

	// LOG_ERR("Chip select 1");
	// spi_cfg.slave = 1;
	// read_jedec_id();
	// k_sleep(K_MSEC(3000));

	return 0;
}
