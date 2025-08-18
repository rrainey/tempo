/*
 * SPDX-License-Identifier: Apache-2.0
 * 
 * Tempo-BT V1 - Application State Management Implementation
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <string.h>

#include "app/app_state.h"

LOG_MODULE_REGISTER(app_state, LOG_LEVEL_INF);

/* Global system state instance */
system_state_t g_system_state;

/* Mode names for logging */
static const char *mode_names[] = {
    [APP_MODE_IDLE] = "IDLE",
    [APP_MODE_ARMED] = "ARMED",
    [APP_MODE_LOGGING] = "LOGGING",
    [APP_MODE_POSTFLIGHT] = "POSTFLIGHT",
    [APP_MODE_ERROR] = "ERROR"
};

int app_state_init(void)
{
    /* Initialize mutex */
    k_mutex_init(&g_system_state.mutex);
    
    /* Lock for initialization */
    k_mutex_lock(&g_system_state.mutex, K_FOREVER);
    
    /* Set initial state */
    g_system_state.mode = APP_MODE_IDLE;
    g_system_state.boot_time_ms = k_uptime_get();
    g_system_state.mode_entry_time_ms = g_system_state.boot_time_ms;
    
    /* Initialize health metrics */
    g_system_state.health.storage_free_percent = 100;  /* Will be updated after mount */
    g_system_state.health.battery_percent = 100;       /* Placeholder for V1 */
    g_system_state.health.ble_connected = false;
    g_system_state.health.gnss_fix_valid = false;
    g_system_state.health.sensors_ok = true;           /* Assume OK until proven otherwise */
    
    /* Clear flags */
    memset(&g_system_state.flags, 0, sizeof(g_system_state.flags));
    g_system_state.flags.logging_enabled = true;       /* Default enabled */
    
    k_mutex_unlock(&g_system_state.mutex);
    
    LOG_INF("App state initialized, mode: %s", mode_names[APP_MODE_IDLE]);
    
    return 0;
}

app_mode_t app_state_get_mode(void)
{
    app_mode_t mode;
    
    k_mutex_lock(&g_system_state.mutex, K_FOREVER);
    mode = g_system_state.mode;
    k_mutex_unlock(&g_system_state.mutex);
    
    return mode;
}

int app_state_set_mode(app_mode_t new_mode)
{
    int ret = 0;
    
    if (new_mode >= ARRAY_SIZE(mode_names)) {
        return -EINVAL;
    }
    
    k_mutex_lock(&g_system_state.mutex, K_FOREVER);
    
    app_mode_t old_mode = g_system_state.mode;
    
    /* Validate state transitions */
    bool valid_transition = false;
    
    switch (old_mode) {
    case APP_MODE_IDLE:
        /* Can go to ARMED or ERROR */
        valid_transition = (new_mode == APP_MODE_ARMED || new_mode == APP_MODE_ERROR);
        break;
        
    case APP_MODE_ARMED:
        /* Can go to LOGGING, IDLE, or ERROR */
        valid_transition = (new_mode == APP_MODE_LOGGING || 
                           new_mode == APP_MODE_IDLE || 
                           new_mode == APP_MODE_ERROR);
        break;
        
    case APP_MODE_LOGGING:
        /* Can go to POSTFLIGHT or ERROR */
        valid_transition = (new_mode == APP_MODE_POSTFLIGHT || new_mode == APP_MODE_ERROR);
        break;
        
    case APP_MODE_POSTFLIGHT:
        /* Can only go to IDLE or ERROR */
        valid_transition = (new_mode == APP_MODE_IDLE || new_mode == APP_MODE_ERROR);
        break;
        
    case APP_MODE_ERROR:
        /* Can go to IDLE after recovery */
        valid_transition = (new_mode == APP_MODE_IDLE);
        break;
        
    default:
        valid_transition = false;
        break;
    }
    
    if (valid_transition) {
        g_system_state.mode = new_mode;
        g_system_state.mode_entry_time_ms = k_uptime_get();
        LOG_INF("Mode transition: %s -> %s", mode_names[old_mode], mode_names[new_mode]);
    } else {
        LOG_WRN("Invalid mode transition: %s -> %s", mode_names[old_mode], mode_names[new_mode]);
        ret = -EINVAL;
    }
    
    k_mutex_unlock(&g_system_state.mutex);
    
    return ret;
}

void app_state_get_health(app_health_t *health)
{
    if (!health) {
        return;
    }
    
    k_mutex_lock(&g_system_state.mutex, K_FOREVER);
    memcpy(health, &g_system_state.health, sizeof(app_health_t));
    k_mutex_unlock(&g_system_state.mutex);
}

void app_state_update_storage_free(uint8_t percent)
{
    k_mutex_lock(&g_system_state.mutex, K_FOREVER);
    g_system_state.health.storage_free_percent = percent;
    k_mutex_unlock(&g_system_state.mutex);
    
    LOG_DBG("Storage free: %d%%", percent);
}

bool app_state_is_logging(void)
{
    return (app_state_get_mode() == APP_MODE_LOGGING);
}