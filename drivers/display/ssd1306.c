/*
 * Copyright (c) 2018 PHYTEC Messtechnik GmbH
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#if defined(CONFIG_DT_HAS_SOLOMON_SSD1306FB_ENABLED)
#define DT_DRV_COMPAT solomon_ssd1306fb
#elif defined(CONFIG_DT_HAS_SINOWEALTH_SH1106_ENABLED)
#define DT_DRV_COMPAT sinowealth_sh1106
#endif

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(ssd1306, CONFIG_DISPLAY_LOG_LEVEL);

#include <string.h>
#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/drivers/display.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>

#include "ssd1306_regs.h"

#define SSD1306_CLOCK_DIV_RATIO		0x0
#define SSD1306_CLOCK_FREQUENCY		0x8
#define SSD1306_PANEL_VCOM_DESEL_LEVEL	0x20
#define SSD1306_PANEL_PUMP_VOLTAGE	SSD1306_SET_PUMP_VOLTAGE_90

#ifndef SSD1306_ADDRESSING_MODE
#define SSD1306_ADDRESSING_MODE		(SSD1306_SET_MEM_ADDRESSING_HORIZONTAL)
#endif

struct ssd1306_config {
#if DT_INST_ON_BUS(0, i2c)
	struct i2c_dt_spec bus;
#elif DT_INST_ON_BUS(0, spi)
	struct spi_dt_spec bus;
	struct gpio_dt_spec data_cmd;
#endif
	struct gpio_dt_spec reset;
	uint16_t height;
	uint16_t width;
	uint8_t segment_offset;
	uint8_t page_offset;
	uint8_t display_offset;
	uint8_t multiplex_ratio;
	uint8_t prechargep;
	bool segment_remap;
	bool com_invdir;
	bool com_sequential;
	bool color_inversion;
	bool sh1106_compatible;
	int ready_time_ms;
};

struct ssd1306_data {
	uint8_t contrast;
	uint8_t scan_mode;
};

#if DT_INST_ON_BUS(0, i2c)

static inline bool ssd1306_bus_ready(const struct device *dev)
{
	const struct ssd1306_config *config = dev->config;

	return device_is_ready(config->bus.bus);
}

static inline int ssd1306_write_bus(const struct device *dev,
				    uint8_t *buf, size_t len, bool command)
{
	const struct ssd1306_config *config = dev->config;

	return i2c_burst_write_dt(&config->bus,
				  command ? SSD1306_CONTROL_ALL_BYTES_CMD :
				  SSD1306_CONTROL_ALL_BYTES_DATA,
				  buf, len);
}

#elif DT_INST_ON_BUS(0, spi)

static inline bool ssd1306_bus_ready(const struct device *dev)
{
	const struct ssd1306_config *config = dev->config;

	if (gpio_pin_configure_dt(&config->data_cmd, GPIO_OUTPUT_INACTIVE) < 0) {
		return false;
	}

	return spi_is_ready_dt(&config->bus);
}

static inline int ssd1306_write_bus(const struct device *dev,
				    uint8_t *buf, size_t len, bool command)
{
	const struct ssd1306_config *config = dev->config;
	int errno;

	gpio_pin_set_dt(&config->data_cmd, command ? 0 : 1);
	struct spi_buf tx_buf = {
		.buf = buf,
		.len = len
	};

	struct spi_buf_set tx_bufs = {
		.buffers = &tx_buf,
		.count = 1
	};

	errno = spi_write_dt(&config->bus, &tx_bufs);

	return errno;
}
#endif

static inline int ssd1306_set_panel_orientation(const struct device *dev)
{
	const struct ssd1306_config *config = dev->config;
	uint8_t cmd_buf[] = {(config->segment_remap ? SSD1306_SET_SEGMENT_MAP_REMAPED
						    : SSD1306_SET_SEGMENT_MAP_NORMAL),
			     (config->com_invdir ? SSD1306_SET_COM_OUTPUT_SCAN_FLIPPED
						 : SSD1306_SET_COM_OUTPUT_SCAN_NORMAL)};

	return ssd1306_write_bus(dev, cmd_buf, sizeof(cmd_buf), true);
}

static inline int ssd1306_set_timing_setting(const struct device *dev)
{
	const struct ssd1306_config *config = dev->config;
	uint8_t cmd_buf[] = {SSD1306_SET_CLOCK_DIV_RATIO,
			     (SSD1306_CLOCK_FREQUENCY << 4) | SSD1306_CLOCK_DIV_RATIO,
			     SSD1306_SET_CHARGE_PERIOD,
			     config->prechargep,
			     SSD1306_SET_VCOM_DESELECT_LEVEL,
			     SSD1306_PANEL_VCOM_DESEL_LEVEL};

	return ssd1306_write_bus(dev, cmd_buf, sizeof(cmd_buf), true);
}

static inline int ssd1306_set_hardware_config(const struct device *dev)
{
	const struct ssd1306_config *config = dev->config;
	uint8_t cmd_buf[] = {
		SSD1306_SET_START_LINE,
		SSD1306_SET_DISPLAY_OFFSET,
		config->display_offset,
		SSD1306_SET_PADS_HW_CONFIG,
		(config->com_sequential ? SSD1306_SET_PADS_HW_SEQUENTIAL
					: SSD1306_SET_PADS_HW_ALTERNATIVE),
		SSD1306_SET_MULTIPLEX_RATIO,
		config->multiplex_ratio,
	};

	return ssd1306_write_bus(dev, cmd_buf, sizeof(cmd_buf), true);
}

static inline int ssd1306_set_charge_pump(const struct device *dev)
{
	const struct ssd1306_config *config = dev->config;
	uint8_t cmd_buf[] = {
		(config->sh1106_compatible ? SH1106_SET_DCDC_MODE : SSD1306_SET_CHARGE_PUMP_ON),
		(config->sh1106_compatible ? SH1106_SET_DCDC_ENABLED
					   : SSD1306_SET_CHARGE_PUMP_ON_ENABLED),
		SSD1306_PANEL_PUMP_VOLTAGE,
	};

	return ssd1306_write_bus(dev, cmd_buf, sizeof(cmd_buf), true);
}

static int ssd1306_resume(const struct device *dev)
{
	uint8_t cmd_buf[] = {
		SSD1306_DISPLAY_ON,
	};

	return ssd1306_write_bus(dev, cmd_buf, sizeof(cmd_buf), true);
}

static int ssd1306_suspend(const struct device *dev)
{
	uint8_t cmd_buf[] = {
		SSD1306_DISPLAY_OFF,
	};

	return ssd1306_write_bus(dev, cmd_buf, sizeof(cmd_buf), true);
}

static int ssd1306_write_default(const struct device *dev, const uint16_t x, const uint16_t y,
				 const struct display_buffer_descriptor *desc, const void *buf,
				 const size_t buf_len)
{
	uint8_t cmd_buf[] = {
		SSD1306_SET_MEM_ADDRESSING_MODE,
		SSD1306_ADDRESSING_MODE,
		SSD1306_SET_COLUMN_ADDRESS,
		x,
		(x + desc->width - 1),
		SSD1306_SET_PAGE_ADDRESS,
		y/8,
		((y + desc->height)/8 - 1)
	};

	if (ssd1306_write_bus(dev, cmd_buf, sizeof(cmd_buf), true)) {
		LOG_ERR("Failed to write command");
		return -1;
	}

	return ssd1306_write_bus(dev, (uint8_t *)buf, buf_len, false);
}

static int ssd1306_write_sh1106(const struct device *dev, const uint16_t x, const uint16_t y,
				const struct display_buffer_descriptor *desc, const void *buf,
				const size_t buf_len)
{
	const struct ssd1306_config *config = dev->config;
	uint8_t x_offset = x + config->segment_offset;
	uint8_t cmd_buf[] = {
		SSD1306_SET_LOWER_COL_ADDRESS |
			(x_offset & SSD1306_SET_LOWER_COL_ADDRESS_MASK),
		SSD1306_SET_HIGHER_COL_ADDRESS |
			((x_offset >> 4) & SSD1306_SET_LOWER_COL_ADDRESS_MASK),
		SSD1306_SET_PAGE_START_ADDRESS | (y / 8)
	};
	uint8_t *buf_ptr = (uint8_t *)buf;

	for (uint8_t n = 0; n < desc->height / 8; n++) {
		cmd_buf[sizeof(cmd_buf) - 1] =
			SSD1306_SET_PAGE_START_ADDRESS | (n + (y / 8));
		LOG_HEXDUMP_DBG(cmd_buf, sizeof(cmd_buf), "cmd_buf");

		if (ssd1306_write_bus(dev, cmd_buf, sizeof(cmd_buf), true)) {
			return -1;
		}

		if (ssd1306_write_bus(dev, buf_ptr, desc->width, false)) {
			return -1;
		}

		buf_ptr = buf_ptr + desc->width;
		if (buf_ptr > ((uint8_t *)buf + buf_len)) {
			LOG_ERR("Exceeded buffer length");
			return -1;
		}
	}

	return 0;
}

static int ssd1306_write(const struct device *dev, const uint16_t x, const uint16_t y,
			 const struct display_buffer_descriptor *desc, const void *buf)
{
	const struct ssd1306_config *config = dev->config;
	size_t buf_len;

	if (desc->pitch < desc->width) {
		LOG_ERR("Pitch is smaller then width");
		return -1;
	}

	buf_len = MIN(desc->buf_size, desc->height * desc->width / 8);
	if (buf == NULL || buf_len == 0U) {
		LOG_ERR("Display buffer is not available");
		return -1;
	}

	if (desc->pitch > desc->width) {
		LOG_ERR("Unsupported mode");
		return -1;
	}

	if ((y & 0x7) != 0U) {
		LOG_ERR("Unsupported origin");
		return -1;
	}

	LOG_DBG("x %u, y %u, pitch %u, width %u, height %u, buf_len %u", x, y, desc->pitch,
		desc->width, desc->height, buf_len);

	if (config->sh1106_compatible) {
		return ssd1306_write_sh1106(dev, x, y, desc, buf, buf_len);
	}

	return ssd1306_write_default(dev, x, y, desc, buf, buf_len);
}

static int ssd1306_read(const struct device *dev, const uint16_t x,
			const uint16_t y,
			const struct display_buffer_descriptor *desc,
			void *buf)
{
	LOG_ERR("Unsupported");
	return -ENOTSUP;
}

static void *ssd1306_get_framebuffer(const struct device *dev)
{
	LOG_ERR("Unsupported");
	return NULL;
}

static int ssd1306_set_brightness(const struct device *dev,
				  const uint8_t brightness)
{
	LOG_WRN("Unsupported");
	return -ENOTSUP;
}

static int ssd1306_set_contrast(const struct device *dev, const uint8_t contrast)
{
	uint8_t cmd_buf[] = {
		SSD1306_SET_CONTRAST_CTRL,
		contrast,
	};

	return ssd1306_write_bus(dev, cmd_buf, sizeof(cmd_buf), true);
}

static void ssd1306_get_capabilities(const struct device *dev,
				     struct display_capabilities *caps)
{
	const struct ssd1306_config *config = dev->config;
	memset(caps, 0, sizeof(struct display_capabilities));
	caps->x_resolution = config->width;
	caps->y_resolution = config->height;
	caps->supported_pixel_formats = PIXEL_FORMAT_MONO10;
	caps->current_pixel_format = PIXEL_FORMAT_MONO10;
	caps->screen_info = SCREEN_INFO_MONO_VTILED;
}

static int ssd1306_set_orientation(const struct device *dev,
				   const enum display_orientation
				   orientation)
{
	LOG_ERR("Unsupported");
	return -ENOTSUP;
}

static int ssd1306_set_pixel_format(const struct device *dev,
				    const enum display_pixel_format pf)
{
	if (pf == PIXEL_FORMAT_MONO10) {
		return 0;
	}
	LOG_ERR("Unsupported");
	return -ENOTSUP;
}

static int ssd1306_init_device(const struct device *dev)
{
	const struct ssd1306_config *config = dev->config;

	uint8_t cmd_buf[] = {
		SSD1306_SET_ENTIRE_DISPLAY_OFF,
		(config->color_inversion ? SSD1306_SET_REVERSE_DISPLAY
					 : SSD1306_SET_NORMAL_DISPLAY),
	};

	/* Reset if pin connected */
	if (config->reset.port) {
		k_sleep(K_MSEC(SSD1306_RESET_DELAY));
		gpio_pin_set_dt(&config->reset, 1);
		k_sleep(K_MSEC(SSD1306_RESET_DELAY));
		gpio_pin_set_dt(&config->reset, 0);
	}

	/* Turn display off */
	if (ssd1306_suspend(dev)) {
		return -EIO;
	}

	if (ssd1306_set_timing_setting(dev)) {
		return -EIO;
	}

	if (ssd1306_set_hardware_config(dev)) {
		return -EIO;
	}

	if (ssd1306_set_panel_orientation(dev)) {
		return -EIO;
	}

	if (ssd1306_set_charge_pump(dev)) {
		return -EIO;
	}

	if (ssd1306_write_bus(dev, cmd_buf, sizeof(cmd_buf), true)) {
		return -EIO;
	}

	if (ssd1306_set_contrast(dev, CONFIG_SSD1306_DEFAULT_CONTRAST)) {
		return -EIO;
	}

	ssd1306_resume(dev);

	return 0;
}

static int ssd1306_init(const struct device *dev)
{
	const struct ssd1306_config *config = dev->config;

	LOG_DBG("");

	k_sleep(K_TIMEOUT_ABS_MS(config->ready_time_ms));

	if (!ssd1306_bus_ready(dev)) {
		LOG_ERR("Bus device %s not ready!", config->bus.bus->name);
		return -EINVAL;
	}

	if (config->reset.port) {
		int ret;

		ret = gpio_pin_configure_dt(&config->reset,
					    GPIO_OUTPUT_INACTIVE);
		if (ret < 0) {
			return ret;
		}
	}

	if (ssd1306_init_device(dev)) {
		LOG_ERR("Failed to initialize device!");
		return -EIO;
	}

	return 0;
}

static const struct ssd1306_config ssd1306_config = {
#if DT_INST_ON_BUS(0, i2c)
	.bus = I2C_DT_SPEC_INST_GET(0),
#elif DT_INST_ON_BUS(0, spi)
	.bus = SPI_DT_SPEC_INST_GET(
		0, SPI_OP_MODE_MASTER | SPI_TRANSFER_MSB | SPI_WORD_SET(8), 0),
	.data_cmd = GPIO_DT_SPEC_INST_GET(0, data_cmd_gpios),
#endif
	.reset = GPIO_DT_SPEC_INST_GET_OR(0, reset_gpios, { 0 }),
	.height = DT_INST_PROP(0, height),
	.width = DT_INST_PROP(0, width),
	.segment_offset = DT_INST_PROP(0, segment_offset),
	.page_offset = DT_INST_PROP(0, page_offset),
	.display_offset = DT_INST_PROP(0, display_offset),
	.multiplex_ratio = DT_INST_PROP(0, multiplex_ratio),
	.segment_remap = DT_INST_PROP(0, segment_remap),
	.com_invdir = DT_INST_PROP(0, com_invdir),
	.com_sequential = DT_INST_PROP(0, com_sequential),
	.prechargep = DT_INST_PROP(0, prechargep),
	.color_inversion = DT_INST_PROP(0, inversion_on),
	.sh1106_compatible = DT_NODE_HAS_COMPAT(0, sinowealth_sh1106),
	.ready_time_ms = DT_INST_PROP(0, ready_time_ms),
};

static struct ssd1306_data ssd1306_driver;

static struct display_driver_api ssd1306_driver_api = {
	.blanking_on = ssd1306_suspend,
	.blanking_off = ssd1306_resume,
	.write = ssd1306_write,
	.read = ssd1306_read,
	.get_framebuffer = ssd1306_get_framebuffer,
	.set_brightness = ssd1306_set_brightness,
	.set_contrast = ssd1306_set_contrast,
	.get_capabilities = ssd1306_get_capabilities,
	.set_pixel_format = ssd1306_set_pixel_format,
	.set_orientation = ssd1306_set_orientation,
};

DEVICE_DT_INST_DEFINE(0, ssd1306_init, NULL,
		      &ssd1306_driver, &ssd1306_config,
		      POST_KERNEL, CONFIG_DISPLAY_INIT_PRIORITY,
		      &ssd1306_driver_api);
