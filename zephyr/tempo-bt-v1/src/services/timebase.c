/*
 * SPDX-License-Identifier: Apache-2.0
 * 
 * Tempo-BT V1 - Time Base Service Implementation
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/sys/time_units.h>

#include "services/timebase.h"

LOG_MODULE_REGISTER(timebase, LOG_LEVEL_INF);

/* Time correlation data */
static time_correlation_t time_corr = {
    .mono_us = 0,
    .utc_ms = 0,
    .valid = false,
    .accuracy_ms = 0
};

/* Mutex for correlation updates */
static struct k_mutex corr_mutex;

/* Boot time reference */
static uint64_t boot_time_us;

int timebase_init(void)
{
    /* Initialize mutex */
    k_mutex_init(&corr_mutex);
    
    /* Capture boot time */
    boot_time_us = k_ticks_to_us_floor64(k_uptime_ticks());
    
    LOG_INF("Timebase initialized, boot time: %llu us", boot_time_us);
    
    /* Test the timer resolution */
    uint64_t t1 = time_now_us();
    k_busy_wait(100);  /* Wait 100 microseconds */
    uint64_t t2 = time_now_us();
    uint64_t delta = t2 - t1;
    
    LOG_INF("Timer test: 100us busy wait measured as %llu us", delta);
    
    return 0;
}

uint64_t time_now_us(void)
{
    /* Get current uptime in ticks and convert to microseconds */
    return k_ticks_to_us_floor64(k_uptime_ticks());
}

void timebase_update_correlation(uint64_t utc_ms, uint32_t accuracy_ms)
{
    k_mutex_lock(&corr_mutex, K_FOREVER);
    
    /* Capture current monotonic time */
    time_corr.mono_us = time_now_us();
    time_corr.utc_ms = utc_ms;
    time_corr.accuracy_ms = accuracy_ms;
    time_corr.valid = true;
    
    k_mutex_unlock(&corr_mutex);
    
    LOG_INF("Time correlation updated: mono=%llu us, UTC=%llu ms, accuracy=%u ms",
            time_corr.mono_us, time_corr.utc_ms, time_corr.accuracy_ms);
}

bool timebase_mono_to_utc(uint64_t mono_us, uint64_t *utc_ms)
{
    if (!utc_ms) {
        return false;
    }
    
    k_mutex_lock(&corr_mutex, K_FOREVER);
    
    if (!time_corr.valid) {
        k_mutex_unlock(&corr_mutex);
        LOG_DBG("No valid time correlation available");
        return false;
    }
    
    /* Calculate time offset from correlation point */
    int64_t offset_us = (int64_t)(mono_us - time_corr.mono_us);
    int64_t offset_ms = offset_us / 1000;
    
    /* Apply offset to UTC time */
    *utc_ms = time_corr.utc_ms + offset_ms;
    
    k_mutex_unlock(&corr_mutex);
    
    return true;
}

bool timebase_get_correlation(time_correlation_t *corr)
{
    if (!corr) {
        return false;
    }
    
    k_mutex_lock(&corr_mutex, K_FOREVER);
    *corr = time_corr;
    k_mutex_unlock(&corr_mutex);
    
    return corr->valid;
}

/* UTC placeholder function for when GNSS is not available */
const char *timebase_utc_string_placeholder(void)
{
    return "unknown";
}