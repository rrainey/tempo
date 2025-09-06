/*
 * SPDX-License-Identifier: Apache-2.0
 * 
 * Tempo-BT V1 - Settings Management Header
 */

#ifndef CONFIG_SETTINGS_H
#define CONFIG_SETTINGS_H

#include <stdint.h>
#include <stdbool.h>
#include <zephyr/bluetooth/uuid.h>

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
 * @brief Get user UUID
 * 
 * @return Pointer to user UUID
 */
const struct bt_uuid_128 *app_settings_get_user_uuid(void);

/**
 * @brief Get device UUID
 * 
 * @return Pointer to device UUID
 */
const struct bt_uuid_128 *app_settings_get_device_uuid(void);

/**
 * @brief Get log backend type
 * 
 * @return Log backend string ("littlefs" or "fatfs")
 */
const char *app_settings_get_log_backend(void);

/**
 * @brief Get PPS enabled state
 * 
 * @return true if PPS is enabled (always false for V1)
 */
bool app_settings_get_pps_enabled(void);

/**
 * @brief Set BLE device name
 * 
 * @param name New BLE name (max 31 chars)
 * @return 0 on success, negative error code on failure
 */
int app_settings_set_ble_name(const char *name);

/**
 * @brief Set user UUID
 * 
 * @param uuid New user UUID
 * @return 0 on success, negative error code on failure
 */
int app_settings_set_user_uuid(const struct bt_uuid_128 *uuid);

/**
 * @brief Set device UUID
 * 
 * @param uuid New device UUID
 * @return 0 on success, negative error code on failure
 */
int app_settings_set_device_uuid(const struct bt_uuid_128 *uuid);

/**
 * @brief Set log backend type
 * 
 * @param backend Backend type ("littlefs" or "fatfs")
 * @return 0 on success, negative error code on failure
 */
int app_settings_set_log_backend(const char *backend);

/**
 * @brief Generate a new random UUID
 * 
 * @param uuid Output UUID structure
 * @return 0 on success, negative error code on failure
 */
int app_settings_generate_uuid(struct bt_uuid_128 *uuid);

/**
 * @brief Test function to demonstrate settings
 */
void app_settings_test(void);

#endif /* CONFIG_SETTINGS_H */