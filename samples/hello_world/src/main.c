/*
 * Copyright (c) 2012-2014 Wind River Systems, Inc.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>

#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/sys/byteorder.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(flash_test, LOG_LEVEL_INF);

#define TEST_JEDEC_ID 1
#define TEST_FREQUENCY 1
#define TEST_ASYNC_TRANSFER 1
#define TEST_QUAD_MODE 1
#define TEST_DUAL_MODE 1
#define TEST_LINE_MODE 1
#define TEST_FLASH_ACCESS 1
#define TEST_RX_ONLY 1
#define TEST_TX_ONLY 1

#define SPI_FLASH_TEST_REGION_OFFSET 0x0
#define SPI_FLASH_SECTOR_SIZE 0x1000 * 1 /* base 0x1000 = 4096 bytes */

#define SPI_NOR_MAX_ID_LEN 3
#define SPI_NOR_CMD_WRSR        0x01    /* Write status register */
#define SPI_NOR_WIP_BIT         BIT(0)  /* Write in progress */
#define SPI_NOR_WEL_BIT         BIT(1)  /* Write enable latch */
#define SPI_NOR_QUAD_BIT        BIT(6)  /* Write enable latch */
#define SPI_NOR_CMD_WRDI        0x04    /* Write disable */
#define SPI_NOR_CMD_RDSR        0x05    /* Read status register */
#define SPI_NOR_CMD_WREN        0x06    /* Write enable */
#define SPI_NOR_CMD_RDID        0x9F    /* Read JEDEC ID */
#define SPI_NOR_CMD_2READ       0xBB    /* Read data (1-2-2) */
#define SPI_NOR_CMD_DREAD       0x3B    /* Read data (1-1-2) */
#define SPI_NOR_CMD_2READ       0xBB    /* Read data (1-2-2) */

#define SPI_NOR_CMD_QREAD       0x6B    /* Read data (1-1-4) */
#define SPI_NOR_CMD_4READ       0xEB    /* Read data (1-4-4) */


/* Indicates that an access command includes bytes for the address.
 * If not provided the opcode is not followed by address bytes.
 */
#define NOR_ACCESS_ADDRESSED BIT(0)

/* Indicates that addressed access uses a 24-bit address regardless of
 * spi_nor_data::flag_32bit_addr.
 */
#define NOR_ACCESS_24BIT_ADDR BIT(1)

/* Indicates that addressed access uses a 32-bit address regardless of
 * spi_nor_data::flag_32bit_addr.
 */
#define NOR_ACCESS_32BIT_ADDR BIT(2)

/* Indicates that an access command is performing a write.  If not
 * provided access is a read.
 */
#define NOR_ACCESS_WRITE BIT(7)

static struct spi_config spi_cfg = {
	.operation = SPI_WORD_SET(8),
	.frequency = 1500000,
	.slave = 0,
};

/*
 * @brief Send an SPI command
 *
 * @param dev Device struct
 * @param opcode The command to send
 * @param access flags that determine how the command is constructed.
 *        See NOR_ACCESS_*.
 * @param addr The address to send
 * @param data The buffer to store or read the value
 * @param length The size of the buffer
 * @return 0 on success, negative errno code otherwise
 */
static int spi_nor_access(const struct device *const dev,
			  uint8_t opcode, unsigned int access,
			  off_t addr, void *data, size_t length)
{
	bool is_write = (access & NOR_ACCESS_WRITE) != 0U;
	uint8_t buf[5] = { 0 };
	struct spi_buf spi_buf[2] = {
		{
			.buf = buf,
			.len = 1,
		},
		{
			.buf = data,
			.len = length
		}
	};

	buf[0] = opcode;
	const struct spi_buf_set tx_set = {
		.buffers = spi_buf,
		.count = (length != 0) ? 2 : 1,
	};

	const struct spi_buf_set rx_set = {
		.buffers = spi_buf,
		.count = 2,
	};

	if (is_write) {
		return spi_write(dev, &spi_cfg, &tx_set);
	}

	return spi_transceive(dev, &spi_cfg, &tx_set, &rx_set);
}

#define spi_nor_cmd_read(dev, opcode, dest, length) \
	spi_nor_access(dev, opcode, 0, 0, dest, length)
#define spi_nor_cmd_addr_read(dev, opcode, addr, dest, length) \
	spi_nor_access(dev, opcode, NOR_ACCESS_ADDRESSED, addr, dest, length)
#define spi_nor_cmd_write(dev, opcode) \
	spi_nor_access(dev, opcode, NOR_ACCESS_WRITE, 0, NULL, 0)
#define spi_nor_cmd_addr_write(dev, opcode, addr, src, length) \
	spi_nor_access(dev, opcode, NOR_ACCESS_WRITE | NOR_ACCESS_ADDRESSED, \
		       addr, (void *)src, length)

#define ASYNC_STACK_SIZE 512
static struct k_poll_signal async_sig = K_POLL_SIGNAL_INITIALIZER(async_sig);
__maybe_unused static struct k_poll_event async_evt =
	K_POLL_EVENT_INITIALIZER(K_POLL_TYPE_SIGNAL,
				 K_POLL_MODE_NOTIFY_ONLY,
				 &async_sig);
static K_SEM_DEFINE(caller, 0, 1);
K_THREAD_STACK_DEFINE(spi_async_stack, ASYNC_STACK_SIZE);
static int result = 1;

__maybe_unused static void spi_async_call_cb(void *p1,
			      void *p2,
			      void *p3)
{
	ARG_UNUSED(p1);
	ARG_UNUSED(p2);
	ARG_UNUSED(p3);

	struct k_poll_event *evt = p1;
	struct k_sem *caller_sem = p2;
	int ret;

	LOG_INF("Polling...");

	while (1) {
		ret = k_poll(evt, 1, K_MSEC(2000));

		if (!ret) {
			result = evt->signal->result;
			k_sem_give(caller_sem);

			/* Reinitializing for next call */
			evt->signal->signaled = 0U;
			evt->state = K_POLL_STATE_NOT_READY;
		}
	}
}

__maybe_unused static int read_jedec_id_async(const struct device *dev)
{
	int ret;
	uint8_t tx_data[1] = { SPI_NOR_CMD_RDID };
	uint8_t rx_data[3] = { 0 };
	struct spi_buf spi_buf[2] = {
		{
			.buf = tx_data,
			.len = sizeof(tx_data),
		},
		{
			.buf = rx_data,
			.len = sizeof(rx_data),
		}
	};

	const struct spi_buf_set tx_set = {.buffers = spi_buf, .count = 2};
	const struct spi_buf_set rx_set = {.buffers = spi_buf, .count = 2};

	ret = spi_transceive_signal(dev, &spi_cfg, &tx_set, &rx_set, &async_sig);
	if (ret) {
		LOG_ERR("Failed to send spi data");
		return ret;
	}
	k_sem_take(&caller, K_FOREVER);

	LOG_HEXDUMP_INF(spi_buf[1].buf, spi_buf[1].len, "JEDEC ID:");
	return 0;
}

__maybe_unused static int read_jedec_id(const struct device *dev)
{
	uint8_t jedec_id[SPI_NOR_MAX_ID_LEN];
	int ret;

	ret = spi_nor_cmd_read(dev, SPI_NOR_CMD_RDID, jedec_id, SPI_NOR_MAX_ID_LEN);
	if (ret) {
		LOG_ERR("Failed to send spi data");
		return ret;
	}

	LOG_HEXDUMP_INF(jedec_id, sizeof(jedec_id), "JEDEC ID:");
	return 0;
}

__maybe_unused static int flash_access_test() {
	const struct device *flash_dev = DEVICE_DT_GET(DT_ALIAS(spi_flash0));
	uint8_t expected[128] = { 0 };
	size_t len = sizeof(expected);
	uint8_t buffer[sizeof(expected)];
	int ret;

	if (!device_is_ready(flash_dev)) {
		LOG_ERR("%s: device not ready", flash_dev->name);
		return -ENODEV;
	}

	ret = flash_erase(flash_dev, SPI_FLASH_TEST_REGION_OFFSET,
			 SPI_FLASH_SECTOR_SIZE);
	if (ret != 0) {
		LOG_ERR("Flash erase failed! %d", ret);
		return ret;
	} else {
		LOG_INF("Flash erase succeeded!");
	}

	memset(buffer, 0, len);
	ret = flash_read(flash_dev, SPI_FLASH_TEST_REGION_OFFSET, buffer, len);
	if (ret != 0) {
		LOG_ERR("Flash read failed! %d", ret);
		return -1;
	}
	LOG_HEXDUMP_DBG(buffer, len, "RX: ");

	for(int i = 0; i < sizeof(expected); i ++) {
		expected[i] = (i / 10) * 0x10 + (i % 10);
		// expected[i] = i;
	}
	LOG_HEXDUMP_DBG(expected, sizeof(expected), "TX: ");

	LOG_INF("Attempting to write %zu bytes", sizeof(expected));
	ret = flash_write(flash_dev, SPI_FLASH_TEST_REGION_OFFSET, expected, sizeof(expected));
	if (ret != 0) {
		LOG_ERR("Flash write failed! %d", ret);
		return -1;
	}

	memset(buffer, 0, len);
	ret = flash_read(flash_dev, SPI_FLASH_TEST_REGION_OFFSET, buffer, len);
	if (ret != 0) {
		LOG_ERR("Flash read failed! %d", ret);
		return -1;
	}

	LOG_HEXDUMP_DBG(buffer, len, "RX: ");
	if (memcmp(expected, buffer, len) == 0) {
		LOG_INF("Data read matches data written. Good!!");
	} else {
		LOG_ERR("Data read does not match data written!!");
		return -1;
	}
	return 0;
}

__maybe_unused static int dual_mode_test(const struct device *dev)
{
	uint8_t tx_data[] = {SPI_NOR_CMD_2READ, 0x0, 0x0, 0x0, 0xFF};
	uint8_t rx_data[16] = { 0 };
	struct spi_buf spi_buf[2] = {
		{
			.buf = tx_data,
			.len = sizeof(tx_data),
		},
		{
			.buf = rx_data,
			.len = sizeof(rx_data),
		}
	};

	const struct spi_buf_set tx_set = {.buffers = spi_buf, .count = 1};
	const struct spi_buf_set rx_set = {.buffers = spi_buf + 1, .count = 1};
	int ret;

	ret = spi_transceive(dev, &spi_cfg, &tx_set, &rx_set);

	if (ret) {
		LOG_ERR("Failed to send spi data: %d\n", ret);
		return ret;
	}

	LOG_HEXDUMP_INF(spi_buf[1].buf, spi_buf[1].len, "Dual read:");
	return 0;
}

__maybe_unused static int quad_enable(const struct device *dev, const bool enable)
{
	int ret;
	uint8_t reg;

	// Write enable
	ret = spi_nor_cmd_write(dev, SPI_NOR_CMD_WREN);
	do {
		ret = spi_nor_cmd_read(dev, SPI_NOR_CMD_RDSR, &reg, sizeof(reg));
		k_sleep(K_MSEC(10));
	} while ((reg & SPI_NOR_WEL_BIT) != SPI_NOR_WEL_BIT);

	// Quad enable or disable
	uint8_t tx_data[] = { SPI_NOR_CMD_WRSR, SPI_NOR_QUAD_BIT };
	struct spi_buf spi_buf[1] = {
		{
			.buf = tx_data,
			.len = sizeof(tx_data),
		},
	};

	if (enable) {
		tx_data[1] = SPI_NOR_QUAD_BIT;
	} else {
		tx_data[1] = 0;
	}

	const struct spi_buf_set tx_set = {.buffers = spi_buf, .count = 1};
	ret = spi_transceive(dev, &spi_cfg, &tx_set, NULL);
	if (ret) {
		LOG_ERR("Failed to send spi data: %d\n", ret);
		return ret;
	}
	do {
		ret = spi_nor_cmd_read(dev, SPI_NOR_CMD_RDSR, &reg, sizeof(reg));
		k_sleep(K_MSEC(10));
	} while ((reg & SPI_NOR_QUAD_BIT) != SPI_NOR_QUAD_BIT);

	// Write disable
	ret = spi_nor_cmd_write(dev, SPI_NOR_CMD_WRDI);
	do {
		ret = spi_nor_cmd_read(dev, SPI_NOR_CMD_RDSR, &reg, sizeof(reg));
		k_sleep(K_MSEC(10));
	} while ((reg & SPI_NOR_WEL_BIT) == SPI_NOR_WEL_BIT);

	return 0;
}

__maybe_unused static int quad_mode_test(const struct device *dev)
{
	uint8_t tx_data[] = {SPI_NOR_CMD_4READ, 0x0, 0x0, 0x0, 0x0, 0xFF, 0xFF};
	uint8_t rx_data[16] = { 0 };
	struct spi_buf spi_buf[2] = {
		{
			.buf = tx_data,
			.len = sizeof(tx_data),
		},
		{
			.buf = rx_data,
			.len = sizeof(rx_data),
		}
	};

	const struct spi_buf_set tx_set = {.buffers = spi_buf, .count = 1};
	const struct spi_buf_set rx_set = {.buffers = spi_buf + 1, .count = 1};
	int ret;

	ret = spi_transceive(dev, &spi_cfg, &tx_set, &rx_set);

	if (ret) {
		LOG_ERR("Failed to send spi data: %d", ret);
		return ret;
	}

	LOG_HEXDUMP_INF(spi_buf[1].buf, spi_buf[1].len, "Quad read:");
	return 0;
}

__maybe_unused static int test_rx_only(const struct device *dev)
{
	int ret;
	uint8_t rx_data[16] = { 0 };
	struct spi_buf spi_buf[1] = {
		{
			.buf = rx_data,
			.len = sizeof(rx_data),
		}
	};

	const struct spi_buf_set rx_set = {.buffers = spi_buf, .count = 1};

	ret = spi_read(dev, &spi_cfg, &rx_set);
	if (ret) {
		LOG_ERR("Failed to read spi data");
		return ret;
	}

	LOG_HEXDUMP_INF(spi_buf[0].buf, spi_buf[0].len, "RX:");
	return 0;
}

__maybe_unused static int test_tx_only(const struct device *dev)
{
	int ret;
	uint8_t tx_data[17] = { 0 };
	uint8_t rx_data[10] = { 0 };
	struct spi_buf spi_buf[2] = {
		{
			.buf = tx_data,
			.len = sizeof(tx_data),
		},
		{
			.buf = rx_data,
			.len = sizeof(rx_data),
		}
	};

	const struct spi_buf_set tx_set = {.buffers = spi_buf, .count = 2};
	const struct spi_buf_set rx_set = {.buffers = spi_buf, .count = 2};

	for (int i = 0; i < sizeof(tx_data); i++) {
		tx_data[i] = i;
	}

	ret = spi_transceive(dev, &spi_cfg, &tx_set, &rx_set);
	if (ret) {
		LOG_ERR("Failed to send spi data");
		return ret;
	}

	LOG_HEXDUMP_INF(spi_buf[1].buf, spi_buf[1].len, "RX:");
	return 0;
}

int main(void)
{
	const struct device *spi_device = (struct device *)DEVICE_DT_GET(DT_NODELABEL(spi0));
	uint32_t sspi_clk = MHZ(24);
	int ret;

	spi_cfg.operation |= SPI_LINES_SINGLE;
	spi_cfg.frequency = sspi_clk / 16;
	LOG_INF("Default SPI Frequency = %dHz", spi_cfg.frequency);

#if TEST_JEDEC_ID
	spi_cfg.slave = 0;
	LOG_INF("============Read JEDEC ID(CS=%d)==============", spi_cfg.slave);
	ret = read_jedec_id(spi_device);
	if (ret) {
		return ret;
	}

	spi_cfg.slave = 1;
	LOG_INF("============Read JEDEC ID(CS=%d)==============", spi_cfg.slave);
	ret = read_jedec_id(spi_device);
	if (ret) {
		return ret;
	}
	spi_cfg.slave = 0;
#endif

#if TEST_FREQUENCY
	for (int i = 0; i <= 8; i++) {
		if (i == 0) {
			spi_cfg.frequency = sspi_clk;
		} else {
			spi_cfg.frequency = sspi_clk / (i * 2);
		}
		LOG_INF("============Frequency(%dHz)==============", spi_cfg.frequency);
		ret = read_jedec_id(spi_device);
		if (ret) {
			return ret;
		}
		k_sleep(K_MSEC(10));
	}
	spi_cfg.frequency = sspi_clk / 16;
#endif

#if TEST_ASYNC_TRANSFER
	spi_cfg.slave = 1;
	LOG_INF("============(ASYNC) Read JEDEC ID(CS=%d)==============", spi_cfg.slave);
	if (!IS_ENABLED(CONFIG_SPI_ASYNC)) {
		LOG_ERR("CONFIG_SPI_ASYNC is disabled");
		return -1;
	}
	struct k_thread async_thread;
	k_tid_t async_thread_id;
	async_thread_id = k_thread_create(&async_thread,
					  spi_async_stack, ASYNC_STACK_SIZE,
					  spi_async_call_cb,
					  &async_evt, &caller, NULL,
					  K_PRIO_COOP(7), 0, K_NO_WAIT);
	ret = read_jedec_id_async(spi_device);
	if (ret) {
		return ret;
	}
	spi_cfg.slave = 0;
#endif /* TEST_ASYNC_TRANSFER */

#if TEST_FLASH_ACCESS
	spi_cfg.slave = 1;
	LOG_INF("============Flash Access(CS=%d)==============", spi_cfg.slave);
	ret = flash_access_test();
	if (ret) {
		return ret;
	}
	spi_cfg.slave = 0;
#endif /* TEST_FLASH_ACCESS */

#if TEST_DUAL_MODE
	spi_cfg.slave = 1;
	LOG_INF("============Dual mode test(CS=%d)==============", spi_cfg.slave);
	spi_cfg.operation &= ~(SPI_LINES_SINGLE | SPI_LINES_QUAD);
	spi_cfg.operation |= SPI_LINES_DUAL;
	ret = dual_mode_test(spi_device);
	if (ret) {
		return ret;
	}
	spi_cfg.slave = 0;
#endif

#if TEST_QUAD_MODE
	spi_cfg.slave = 1;
	LOG_INF("============Quad mode test(CS=%d)==============", spi_cfg.slave);
	spi_cfg.operation |= SPI_LINES_SINGLE;
	spi_cfg.operation &= ~(SPI_LINES_DUAL | SPI_LINES_QUAD);
	ret = quad_enable(spi_device, true);
	if (ret) {
		return ret;
	}

	spi_cfg.operation |= SPI_LINES_QUAD;
	spi_cfg.operation &= ~(SPI_LINES_SINGLE | SPI_LINES_DUAL);
	ret = quad_mode_test(spi_device);
	if (ret) {
		return ret;
	}

	spi_cfg.operation |= SPI_LINES_SINGLE;
	spi_cfg.operation &= ~(SPI_LINES_DUAL | SPI_LINES_QUAD);
	ret = quad_enable(spi_device, false);
	if (ret) {
		return ret;
	}
	spi_cfg.slave = 0;
#endif

#if TEST_LINE_MODE
	spi_cfg.slave = 0;
	LOG_INF("============SPI line test(CS=%d)==============", spi_cfg.slave);
	spi_cfg.operation |= (SPI_MODE_CPHA | SPI_MODE_CPOL);
	/* Should be returned as -1 */
	ret = read_jedec_id(spi_device);
	if (ret == 0) {
		return ret;
	}
	spi_cfg.operation &= ~(SPI_MODE_CPHA | SPI_MODE_CPOL);
#endif

#if TEST_RX_ONLY
	spi_cfg.slave = 0;
	LOG_INF("============SPI rx only(CS=%d)==============", spi_cfg.slave);
	ret = test_rx_only(spi_device);
	if (ret) {
		return ret;
	}
#endif

#if TEST_TX_ONLY
	spi_cfg.slave = 0;
	LOG_INF("============SPI tx only(CS=%d)==============", spi_cfg.slave);
	ret = test_tx_only(spi_device);
	if (ret) {
		return ret;
	}
#endif

	return 0;
}
