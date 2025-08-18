/*
 * SPDX-License-Identifier: Apache-2.0
 * 
 * Tempo-BT V1 - Application Initialization Header
 */

#ifndef APP_INIT_H
#define APP_INIT_H

/**
 * @brief Initialize the storage subsystem
 * 
 * Mounts LittleFS and creates necessary directories
 * 
 * @return 0 on success, negative error code on failure
 */
int app_storage_init(void);

#endif /* APP_INIT_H */