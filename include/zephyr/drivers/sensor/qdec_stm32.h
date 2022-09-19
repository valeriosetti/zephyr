/*
 * Copyright (c) 2021 G-Technologies Sdn. Bhd.
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @file
 * @brief Extended public API for STM32's quadrature decoder
 */

#ifndef ZEPHYR_INCLUDE_DRIVERS_SENSOR_QDEC_STM32_H_
#define ZEPHYR_INCLUDE_DRIVERS_SENSOR_QDEC_STM32_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <zephyr/drivers/sensor.h>

enum sensor_channel_qdec_stm32 {
	/** Raw timer's counter value. */
	SENSOR_CHAN_RAW_COUNTER = SENSOR_CHAN_PRIV_START,
};

enum sensor_attribute_qdec_stm32 {
	/** Value at which timer's counter wrap to 0 */
	SENSOR_ATTR_MAX_VALUE = SENSOR_ATTR_PRIV_START,
};

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_DRIVERS_SENSOR_QDEC_STM32_H_ */
