/*
 * SPDX-License-Identifier: Apache-2.0
 * 
 * Tempo-BT V1 - Application State Management
 */

#ifndef APP_STATE_H
#define APP_STATE_H

#include <stdint.h>
#include <stdbool.h>
#include <zephyr/kernel.h>

/* System operating modes */
typedef enum {
    APP_MODE_IDLE,       /* System idle, waiting for user input */
    APP_MODE_ARMED,      /* Ready to start logging on exit detection */
    APP_MODE_LOGGING,    /* Actively logging flight data */
    APP_MODE_POSTFLIGHT, /* Flight complete, finalizing logs */
    APP_MODE_ERROR       /* Error state */
} app_mode_t;

/* System health information */
typedef struct {
    uint8_t storage_free_percent;  /* 0-100% free space */
    uint8_t battery_percent;       /* 0-100% battery (placeholder for V1) */
    bool ble_connected;            /* BLE connection status */
    bool gnss_fix_valid;           /* GNSS has valid fix */
    bool sensors_ok;               /* All sensors operational */
} app_health_t;

/* System state container */
typedef struct {
    app_mode_t mode;               /* Current operating mode */
    app_health_t health;           /* System health metrics */
    
    /* State flags */
    struct {
        bool pps_locked : 1;       /* PPS time sync (always false on V1) */
        bool upload_active : 1;    /* File upload in progress */
        bool dfu_in_progress : 1;  /* DFU update in progress */
        bool logging_enabled : 1;  /* Logging is enabled */
    } flags;
    
    /* Timestamps */
    int64_t boot_time_ms;          /* System boot time */
    int64_t mode_entry_time_ms;    /* When current mode was entered */
    
    /* Mutex for thread-safe access */
    struct k_mutex mutex;
} system_state_t;

/* Global system state instance */
extern system_state_t g_system_state;

/**
 * @brief Initialize the application state
 * 
 * Sets up initial state values and mutex
 * 
 * @return 0 on success, negative error code on failure
 */
int app_state_init(void);

/**
 * @brief Get current operating mode (thread-safe)
 * 
 * @return Current app_mode_t
 */
app_mode_t app_state_get_mode(void);

/**
 * @brief Set operating mode (thread-safe)
 * 
 * @param new_mode Mode to transition to
 * @return 0 on success, -EINVAL if invalid transition
 */
int app_state_set_mode(app_mode_t new_mode);

/**
 * @brief Get system health snapshot (thread-safe)
 * 
 * @param health Pointer to health struct to fill
 */
void app_state_get_health(app_health_t *health);

/**
 * @brief Update storage free percentage
 * 
 * @param percent Free space percentage (0-100)
 */
void app_state_update_storage_free(uint8_t percent);

/**
 * @brief Check if logging is currently active
 * 
 * @return true if in LOGGING mode, false otherwise
 */
bool app_state_is_logging(void);

#endif /* APP_STATE_H */