/*
 * Copyright (c) 2025 Arduino SA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#ifndef MICROPY_INCLUDED_DRIVERS_ESP_HOSTED_HAL_H
#define MICROPY_INCLUDED_DRIVERS_ESP_HOSTED_HAL_H
// HAL functions.
int esp_hosted_hal_init(const struct device *dev);
bool esp_hosted_hal_data_ready(const struct device *dev);
int esp_hosted_hal_spi_transfer(const struct device *dev, uint8_t *tx, uint8_t *rx, uint32_t size);
#endif // MICROPY_INCLUDED_DRIVERS_ESP_HOSTED_HAL_H
