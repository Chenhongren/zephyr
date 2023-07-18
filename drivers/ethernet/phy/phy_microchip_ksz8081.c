/*
 * Copyright 2023 NXP
 *
 * Inspiration from phy_mii.c, which is:
 * Copyright (c) 2021 IP-Logix Inc.
 * Copyright 2022 NXP
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT microchip_ksz8081

#include <zephyr/kernel.h>
#include <zephyr/net/phy.h>
#include <zephyr/net/mii.h>
#include <zephyr/drivers/mdio.h>
#include <string.h>
#include <zephyr/sys/util_macro.h>
#include <zephyr/drivers/gpio.h>

#define LOG_MODULE_NAME phy_mc_ksz8081
#define LOG_LEVEL CONFIG_PHY_LOG_LEVEL
#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(LOG_MODULE_NAME);

#define PHY_MC_KSZ8081_OMSO_REG			0x16
#define PHY_MC_KSZ8081_OMSO_FACTORY_MODE_MASK	BIT(15)
#define PHY_MC_KSZ8081_OMSO_NAND_TREE_MASK	BIT(5)

#define PHY_MC_KSZ8081_CTRL2_REG		0x1F
#define PHY_MC_KSZ8081_CTRL2_REF_CLK_SEL	BIT(7)

#define PHY_MC_KSZ8081_RESET_HOLD_TIME

enum ksz8081_interface {
	KSZ8081_MII,
	KSZ8081_RMII,
};

struct mc_ksz8081_config {
	uint8_t addr;
	const struct device *mdio_dev;
	enum ksz8081_interface phy_iface;
	const struct gpio_dt_spec reset_gpio;
	const struct gpio_dt_spec interrupt_gpio;
};

struct mc_ksz8081_data {
	const struct device *dev;
	struct phy_link_state state;
	phy_callback_t cb;
	void *cb_data;
	struct k_mutex mutex;
	struct k_work_delayable phy_monitor_work;
};

static int phy_mc_ksz8081_read(const struct device *dev,
				uint16_t reg_addr, uint32_t *data)
{
	const struct mc_ksz8081_config *config = dev->config;
	int ret;

	ret = mdio_read(config->mdio_dev, config->addr, reg_addr, (uint16_t *)data);
	if (ret) {
		return ret;
	}

	return 0;
}

static int phy_mc_ksz8081_write(const struct device *dev,
				uint16_t reg_addr, uint32_t data)
{
	const struct mc_ksz8081_config *config = dev->config;
	int ret;

	ret = mdio_write(config->mdio_dev, config->addr, reg_addr, (uint16_t)data);
	if (ret) {
		return ret;
	}

	return 0;
}

static int phy_mc_ksz8081_autonegotiate(const struct device *dev)
{
	const struct mc_ksz8081_config *config = dev->config;
	int ret;
	uint32_t bmcr = 0;
	uint32_t bmsr = 0;
	uint16_t timeout = CONFIG_PHY_AUTONEG_TIMEOUT_MS / 100;

	/* Read control register to write back with autonegotiation bit */
	ret = phy_mc_ksz8081_read(dev, MII_BMCR, &bmcr);
	if (ret) {
		LOG_ERR("Error reading phy (%d) basic control register", config->addr);
		return ret;
	}

	/* (re)start autonegotiation */
	LOG_DBG("PHY (%d) is entering autonegotiation sequence", config->addr);
	bmcr |= MII_BMCR_AUTONEG_ENABLE | MII_BMCR_AUTONEG_RESTART;
	bmcr &= ~MII_BMCR_ISOLATE;

	ret = phy_mc_ksz8081_write(dev, MII_BMCR, bmcr);
	if (ret) {
		LOG_ERR("Error writing phy (%d) basic control register", config->addr);
		return ret;
	}

	/* TODO change this to GPIO interrupt driven */
	do {
		if (timeout-- == 0) {
			LOG_DBG("PHY (%d) autonegotiation timed out", config->addr);
			return -ETIMEDOUT;
		}
		k_msleep(100);

		ret = phy_mc_ksz8081_read(dev, MII_BMSR, &bmsr);
		if (ret) {
			LOG_ERR("Error reading phy (%d) basic status register", config->addr);
			return ret;
		}
	} while (!(bmsr & MII_BMSR_AUTONEG_COMPLETE));

	LOG_DBG("PHY (%d) autonegotiation completed", config->addr);

	return 0;
}

static int phy_mc_ksz8081_get_link(const struct device *dev,
					struct phy_link_state *state)
{
	const struct mc_ksz8081_config *config = dev->config;
	struct mc_ksz8081_data *data = dev->data;
	int ret;
	uint32_t bmsr = 0;
	uint32_t anar = 0;
	uint32_t anlpar = 0;

	/* Lock mutex */
	ret = k_mutex_lock(&data->mutex, K_FOREVER);
	if (ret) {
		LOG_ERR("PHY mutex lock error");
		return ret;
	}

	/* Read link state */
	ret = phy_mc_ksz8081_read(dev, MII_BMSR, &bmsr);
	if (ret) {
		LOG_ERR("Error reading phy (%d) basic status register", config->addr);
		k_mutex_unlock(&data->mutex);
		return ret;
	}
	state->is_up = bmsr & MII_BMSR_LINK_STATUS;

	LOG_DBG("PHY %d is %s", config->addr, data->state.is_up ? "up" : "down");

	/* Read currently configured advertising options */
	ret = phy_mc_ksz8081_read(dev, MII_ANAR, &anar);
	if (ret) {
		LOG_ERR("Error reading phy (%d) advertising register", config->addr);
		k_mutex_unlock(&data->mutex);
		return ret;
	}

	/* Read link partner capability */
	ret = phy_mc_ksz8081_read(dev, MII_ANLPAR, &anlpar);
	if (ret) {
		LOG_ERR("Error reading phy (%d) link partner register", config->addr);
		k_mutex_unlock(&data->mutex);
		return ret;
	}

	/* Unlock mutex */
	k_mutex_unlock(&data->mutex);

	uint32_t mutual_capabilities = anar & anlpar;

	if (mutual_capabilities & MII_ADVERTISE_100_FULL) {
		state->speed = LINK_FULL_100BASE_T;
	} else if (mutual_capabilities & MII_ADVERTISE_100_HALF) {
		state->speed = LINK_HALF_100BASE_T;
	} else if (mutual_capabilities & MII_ADVERTISE_10_FULL) {
		state->speed = LINK_FULL_10BASE_T;
	} else if (mutual_capabilities & MII_ADVERTISE_10_HALF) {
		state->speed = LINK_HALF_10BASE_T;
	} else {
		LOG_ERR("No valid PHY %d capabilities", config->addr);
		return -EIO;
	}

	LOG_DBG("PHY (%d) Link speed %s Mb, %s duplex\n", config->addr,
		(PHY_LINK_IS_SPEED_100M(state->speed) ? "100" : "10"),
		PHY_LINK_IS_FULL_DUPLEX(state->speed) ? "full" : "half");

	return 0;
}

/*
 * Configuration set statically (DT) that should never change
 * This function is needed in case the PHY is reset then the next call
 * to configure the phy will ensure this configuration will be redone
 */
static int phy_mc_ksz8081_static_cfg(const struct device *dev)
{
	const struct mc_ksz8081_config *config = dev->config;
	uint32_t omso = 0;
	uint32_t ctrl2 = 0;
	int ret = 0;

	/* Force normal operation in the case of factory mode */
	ret = phy_mc_ksz8081_read(dev, PHY_MC_KSZ8081_OMSO_REG, (uint32_t *)&omso);
	if (ret) {
		return ret;
	}

	omso &= ~PHY_MC_KSZ8081_OMSO_FACTORY_MODE_MASK &
		~PHY_MC_KSZ8081_OMSO_NAND_TREE_MASK;

	ret = phy_mc_ksz8081_write(dev, PHY_MC_KSZ8081_OMSO_REG, (uint32_t)omso);
	if (ret) {
		return ret;
	}

	/* Select correct reference clock mode depending on interface setup */
	ret = phy_mc_ksz8081_read(dev, PHY_MC_KSZ8081_CTRL2_REG, (uint32_t *)&ctrl2);
	if (ret) {
		return ret;
	}

	if (config->phy_iface == KSZ8081_RMII) {
		ctrl2 |= PHY_MC_KSZ8081_CTRL2_REF_CLK_SEL;
	} else {
		ctrl2 &= ~PHY_MC_KSZ8081_CTRL2_REF_CLK_SEL;
	}

	ret = phy_mc_ksz8081_write(dev, PHY_MC_KSZ8081_CTRL2_REG, (uint32_t)ctrl2);
	if (ret) {
		return ret;
	}

	return 0;
}

static int phy_mc_ksz8081_cfg_link(const struct device *dev,
					enum phy_link_speed speeds)
{
	const struct mc_ksz8081_config *config = dev->config;
	struct mc_ksz8081_data *data = dev->data;
	int ret;
	uint32_t anar;

	/* Lock mutex */
	ret = k_mutex_lock(&data->mutex, K_FOREVER);
	if (ret) {
		LOG_ERR("PHY mutex lock error");
		return ret;
	}

	/* We are going to reconfigure the phy, don't need to monitor until done */
	k_work_cancel_delayable(&data->phy_monitor_work);

	/* DT configurations */
	ret = phy_mc_ksz8081_static_cfg(dev);
	if (ret) {
		k_mutex_unlock(&data->mutex);
		return ret;
	}

	/* Read ANAR register to write back */
	ret = phy_mc_ksz8081_read(dev, MII_ANAR, &anar);
	if (ret) {
		LOG_ERR("Error reading phy (%d) advertising register", config->addr);
		k_mutex_unlock(&data->mutex);
		return -EIO;
	}

	/* Setup advertising register */
	if (speeds & LINK_FULL_100BASE_T) {
		anar |= MII_ADVERTISE_100_FULL;
	} else {
		anar &= ~MII_ADVERTISE_100_FULL;
	}
	if (speeds & LINK_HALF_100BASE_T) {
		anar |= MII_ADVERTISE_100_HALF;
	} else {
		anar &= ~MII_ADVERTISE_100_HALF;
	}
	if (speeds & LINK_FULL_10BASE_T) {
		anar |= MII_ADVERTISE_10_FULL;
	} else {
		anar &= ~MII_ADVERTISE_10_FULL;
	}
	if (speeds & LINK_HALF_10BASE_T) {
		anar |= MII_ADVERTISE_10_HALF;
	} else {
		anar &= ~MII_ADVERTISE_10_HALF;
	}

	/* Write capabilities to advertising register */
	ret = phy_mc_ksz8081_write(dev, MII_ANAR, anar);
	if (ret) {
		LOG_ERR("Error writing phy (%d) advertising register", config->addr);
		k_mutex_unlock(&data->mutex);
		return ret;
	}

	/* (re)do autonegotiation */
	ret = phy_mc_ksz8081_autonegotiate(dev);
	if (ret) {
		LOG_ERR("Error in autonegotiation");
		k_mutex_unlock(&data->mutex);
		return ret;
	}

	/* Unlock mutex */
	k_mutex_unlock(&data->mutex);


	/* Get link status */
	ret = phy_mc_ksz8081_get_link(dev, &data->state);
	if (ret) {
		return ret;
	}

	/* Start monitoring */
	k_work_schedule(&data->phy_monitor_work,
				K_MSEC(CONFIG_PHY_MONITOR_PERIOD));

	/* Log the results of the configuration */
	LOG_INF("PHY %d is %s", config->addr, data->state.is_up ? "up" : "down");
	LOG_INF("PHY (%d) Link speed %s Mb, %s duplex\n", config->addr,
		(PHY_LINK_IS_SPEED_100M(data->state.speed) ? "100" : "10"),
		PHY_LINK_IS_FULL_DUPLEX(data->state.speed) ? "full" : "half");

	return 0;
}

static int phy_mc_ksz8081_link_cb_set(const struct device *dev,
					phy_callback_t cb, void *user_data)
{
	struct mc_ksz8081_data *data = dev->data;

	data->cb = cb;
	data->cb_data = user_data;

	phy_mc_ksz8081_get_link(dev, &data->state);

	data->cb(dev, &data->state, data->cb_data);

	return 0;
}

static void phy_mc_ksz8081_monitor_work_handler(struct k_work *work)
{
	struct k_work_delayable *dwork = k_work_delayable_from_work(work);
	struct mc_ksz8081_data *data =
		CONTAINER_OF(dwork, struct mc_ksz8081_data, phy_monitor_work);
	const struct device *dev = data->dev;
	struct phy_link_state state;
	int rc;

	rc = phy_mc_ksz8081_get_link(dev, &state);

	if (rc == 0 && memcmp(&state, &data->state, sizeof(struct phy_link_state)) != 0) {
		memcpy(&data->state, &state, sizeof(struct phy_link_state));
		if (data->cb) {
			data->cb(dev, &data->state, data->cb_data);
		}
	}

	/* TODO change this to GPIO interrupt driven */
	k_work_reschedule(&data->phy_monitor_work, K_MSEC(CONFIG_PHY_MONITOR_PERIOD));
}

static int phy_mc_ksz8081_init(const struct device *dev)
{
	const struct mc_ksz8081_config *config = dev->config;
	struct mc_ksz8081_data *data = dev->data;
	int ret;

	data->dev = dev;

	ret = k_mutex_init(&data->mutex);
	if (ret) {
		return ret;
	}

	mdio_bus_enable(config->mdio_dev);

	/* Prevent NAND TREE mode */
	ret = gpio_pin_configure_dt(&config->interrupt_gpio, GPIO_OUTPUT_ACTIVE);
	if (ret) {
		return ret;
	}

	/* Start reset */
	ret = gpio_pin_configure_dt(&config->reset_gpio, GPIO_OUTPUT_INACTIVE);
	if (ret) {
		return ret;
	}

	/* Wait for 500 ms as specified by datasheet */
	k_busy_wait(USEC_PER_MSEC * 500);

	/* Reset over */
	ret = gpio_pin_set_dt(&config->reset_gpio, 1);
	if (ret) {
		return ret;
	}

	k_work_init_delayable(&data->phy_monitor_work,
				phy_mc_ksz8081_monitor_work_handler);

	return 0;
}

static const struct ethphy_driver_api mc_ksz8081_phy_api = {
	.get_link = phy_mc_ksz8081_get_link,
	.cfg_link = phy_mc_ksz8081_cfg_link,
	.link_cb_set = phy_mc_ksz8081_link_cb_set,
	.read = phy_mc_ksz8081_read,
	.write = phy_mc_ksz8081_write,
};

#define MICROCHIP_KSZ8081_INIT(n)						\
	static const struct mc_ksz8081_config mc_ksz8081_##n##_config = {	\
		.addr = DT_INST_REG_ADDR(n),					\
		.mdio_dev = DEVICE_DT_GET(DT_INST_PARENT(n)),			\
		.phy_iface = DT_INST_ENUM_IDX(n, mc_interface_type),		\
		.reset_gpio = GPIO_DT_SPEC_INST_GET(n, mc_reset_gpio),		\
		.interrupt_gpio = GPIO_DT_SPEC_INST_GET(n, mc_interrupt_gpio),	\
	};									\
										\
	static struct mc_ksz8081_data mc_ksz8081_##n##_data;			\
										\
	DEVICE_DT_INST_DEFINE(n, &phy_mc_ksz8081_init, NULL,			\
			&mc_ksz8081_##n##_data, &mc_ksz8081_##n##_config,	\
			POST_KERNEL, CONFIG_PHY_INIT_PRIORITY,			\
			&mc_ksz8081_phy_api);

DT_INST_FOREACH_STATUS_OKAY(MICROCHIP_KSZ8081_INIT)
