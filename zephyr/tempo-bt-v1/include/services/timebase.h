/*
 * SPDX-License-Identifier: Apache-2.0
 * 
 * Tempo-BT V1 - Time Base Service
 */

#ifndef SERVICES_TIMEBASE_H
#define SERVICES_TIMEBASE_H

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief Time correlation structure
 * 
 * Maps monotonic time to UTC time when GNSS fix is available
 */
typedef struct {
    uint64_t mono_us;      /* Monotonic timestamp in microseconds */
    uint64_t utc_ms;       /* UTC timestamp in milliseconds since epoch */
    bool valid;            /* True if correlation is valid */
    uint32_t accuracy_ms;  /* Time accuracy estimate in milliseconds */
} time_correlation_t;

/**
 * @brief Initialize the timebase service
 * 
 * Sets up the high-resolution timer for monotonic timestamps
 * 
 * @return 0 on success, negative error code on failure
 */
int timebase_init(void);

/**
 * @brief Get current monotonic time in microseconds
 * 
 * This is the primary timestamp source for all sensor data.
 * Time is monotonic since boot and not affected by time adjustments.
 * 
 * @return Current time in microseconds since boot
 */
uint64_t time_now_us(void);

/**
 * @brief Get current monotonic time in milliseconds
 * 
 * Convenience function for millisecond timestamps
 * 
 * @return Current time in milliseconds since boot
 */
static inline uint32_t time_now_ms(void)
{
    return (uint32_t)(time_now_us() / 1000ULL);
}

/**
 * @brief Update time correlation from GNSS
 * 
 * Called when GNSS provides accurate time information
 * 
 * @param utc_ms UTC time in milliseconds since epoch
 * @param accuracy_ms Estimated accuracy in milliseconds
 */
void timebase_update_correlation(uint64_t utc_ms, uint32_t accuracy_ms);

/**
 * @brief Convert monotonic time to UTC
 * 
 * @param mono_us Monotonic timestamp in microseconds
 * @param utc_ms Output: UTC time in milliseconds (if available)
 * @return true if conversion successful, false if no valid correlation
 */
bool timebase_mono_to_utc(uint64_t mono_us, uint64_t *utc_ms);

/**
 * @brief Get current time correlation
 * 
 * @param corr Output: Current correlation data
 * @return true if correlation is valid, false otherwise
 */
bool timebase_get_correlation(time_correlation_t *corr);

/**
 * @brief Check if PPS is locked (V2 feature)
 * 
 * @return Always false on V1 hardware
 */
static inline bool timebase_pps_locked(void)
{
    return false;  /* No PPS on V1 */
}

/**
 * @brief Get UTC time string placeholder
 * 
 * Returns "unknown" until GNSS provides valid time
 * Used by aggregator for $PTH sentences when UTC is not available
 * 
 * @return "unknown" string
 */
const char *timebase_utc_string_placeholder(void);

#endif /* SERVICES_TIMEBASE_H */