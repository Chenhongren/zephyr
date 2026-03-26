/*
 * Copyright (c) 2026 ITE Technology Corporation.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/pwm.h>
#include <zephyr/drivers/sensor.h>

#include <zephyr/logging/log.h>
#define LOG_LEVEL CONFIG_LOG_LEVEL_DBG
LOG_MODULE_REGISTER(main);

#define RPM_TO_HZ(rpm, p) (rpm * p / 60)

#define WAIT_PWM_READY_MS K_MSEC(50)

int main(void)
{
	const struct device *pwm_dev = DEVICE_DT_GET(DT_ALIAS(pwm_test));
	const struct device *fan_dev = DEVICE_DT_GET(DT_ALIAS(tach_test));
	const int pulses_per_round = DT_PROP(DT_ALIAS(tach_test), pulses_per_round);
	struct sensor_value val;
	double rpm;
	int ret;

	if (!device_is_ready(pwm_dev) || !device_is_ready(fan_dev)) {
		LOG_ERR("pwm or fan device is not ready");
		return -ENODEV;
	}

	for (int i = 1; i < 5; i++) {
		uint32_t period_ns = i * NSEC_PER_MSEC, pulse_ns = period_ns / 2;
		uint32_t expected = NSEC_PER_SEC / period_ns;

		ret = pwm_set(pwm_dev, 0, period_ns, pulse_ns, 0);
		if (ret) {
			LOG_ERR("failed to set pwm");
			return ret;
		}

		/* wait for pwm stable */
		k_sleep(WAIT_PWM_READY_MS);

		ret = sensor_sample_fetch_chan(fan_dev, SENSOR_CHAN_RPM);
		if (ret) {
			LOG_ERR("failed to fetch sample");
			return ret;
		}

		ret = sensor_channel_get(fan_dev, SENSOR_CHAN_RPM, &val);
		if (ret) {
			LOG_ERR("failed to get sensor value");
			return ret;
		}

		rpm = sensor_value_to_double(&val);
		if (rpm == 0) {
			LOG_ERR("tach pin not wired to pwm pin? (tach rpm 0)");
			return -EINVAL;
		}

		LOG_INF("rpm %.2f, freq %.2f (exp %d)", rpm, RPM_TO_HZ(rpm, pulses_per_round),
			expected);
	}

	return 0;
}
