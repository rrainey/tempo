/*
 * SPDX-License-Identifier: Apache-2.0
 * 
 * Tempo-BT V1 - Settings Management Header
 */

#ifndef CONFIG_SETTINGS_H
#define CONFIG_SETTINGS_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Initialize settings subsystem
 * 
 * Registers handlers and loads saved settings from NVS
 * 
 * @return 0 on success, negative error code on failure
 */
int app_settings_init(void);

/**
 * @brief Get BLE device name
 * 
 * @return Current BLE name string
 */
const char *app_settings_get_ble_name(void);

/**
 * @brief Get IMU output data rate
 * 
 * @return IMU ODR in Hz
 */
uint16_t app_settings_get_imu_odr(void);

/**
 * @brief Get barometer sampling rate
 * 
 * @return Baro rate in Hz
 */
uint16_t app_settings_get_baro_rate(void);

/**
 * @brief Get GNSS update rate
 * 
 * @return GNSS rate in Hz
 */
uint8_t app_settings_get_gnss_rate(void);

/**
 * @brief Set BLE device name
 * 
 * @param name New BLE name (max 31 chars)
 * @return 0 on success, negative error code on failure
 */
int app_settings_set_ble_name(const char *name);

/**
 * @brief Set IMU output data rate
 * 
 * @param odr_hz New ODR in Hz
 * @return 0 on success, negative error code on failure
 */
int app_settings_set_imu_odr(uint16_t odr_hz);

/**
 * @brief Test function to demonstrate settings
 */
void app_settings_test(void);

#endif /* CONFIG_SETTINGS_H */