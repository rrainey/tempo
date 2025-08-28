/*
 * SPDX-License-Identifier: Apache-2.0
 * 
 * Tempo-BT V1 - Aggregator Service Implementation
 */
#pragma GCC diagnostic ignored "-Wdouble-promotion"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

#include "generated/app_version.h"

#include "services/aggregator.h"
#include "services/timebase.h"
#include "services/imu.h"
#include "services/baro.h"
#include "services/gnss.h"
#include "services/file_writer.h"
#include "app/log_format.h"

LOG_MODULE_REGISTER(aggregator, LOG_LEVEL_INF);

/* Ring buffer sizes */
#define IMU_RING_SIZE       128  /* 128 samples @ 400Hz = 320ms buffer */
#define BARO_RING_SIZE      32   /* 32 samples @ 50Hz = 640ms buffer */
#define GNSS_RING_SIZE      32   /* 32 fixes @ 10Hz = 3.2s buffer (increased for 10Hz) */

/* Output timing */
#define IMU_OUTPUT_PERIOD_MS    25   /* 40 Hz */
#define ENV_OUTPUT_PERIOD_MS    250  /* 4 Hz */

char gps_date_string[14];

/* Ring buffer structures */
typedef struct {
    imu_sample_t samples[IMU_RING_SIZE];
    uint32_t head;
    uint32_t tail;
    struct k_spinlock lock;
} imu_ring_t;

typedef struct {
    baro_sample_t samples[BARO_RING_SIZE];
    uint32_t head;
    uint32_t tail;
    struct k_spinlock lock;
} baro_ring_t;

typedef struct {
    gnss_fix_t samples[GNSS_RING_SIZE];
    uint32_t head;
    uint32_t tail;
    struct k_spinlock lock;
} gnss_ring_t;

/* Static data */
static imu_ring_t imu_ring;
static baro_ring_t baro_ring;
static gnss_ring_t gnss_ring;

static aggregator_config_t config = {
    .imu_output_rate = 40,
    .env_output_rate = 4,
    .gnss_output_rate = 1,
    .mag_output_rate = 0,
    .enable_quaternion = true,
    .enable_magnetometer = false,
    .session_id = 0,
    .session_start_us = 0
};

static aggregator_output_callback_t output_callback = NULL;

/* Statistics */
static struct {
    uint32_t imu_count;
    uint32_t env_count;
    uint32_t gnss_count;
    uint32_t dropped_count;
} stats;

/* Output thread */
K_THREAD_STACK_DEFINE(aggregator_stack, 4096);
static struct k_thread aggregator_thread;
static k_tid_t aggregator_tid = NULL;
static volatile bool running = false;

/* Work items for timed output */
static struct k_work_delayable imu_output_work;
static struct k_work_delayable env_output_work;

/* Latest samples for interpolation */
static imu_sample_t last_imu_sample;
static baro_sample_t last_baro_sample;
static bool have_imu_sample = false;
static bool have_baro_sample = false;

/* Helper: Ring buffer push */
#define RING_PUSH(ring, item) do { \
    k_spinlock_key_t key = k_spin_lock(&(ring).lock); \
    uint32_t next_head = ((ring).head + 1) % ARRAY_SIZE((ring).samples); \
    if (next_head != (ring).tail) { \
        (ring).samples[(ring).head] = (item); \
        (ring).head = next_head; \
    } else { \
        stats.dropped_count++; \
    } \
    k_spin_unlock(&(ring).lock, key); \
} while(0)

/* Helper: Ring buffer pop */
#define RING_POP(ring, item, result) do { \
    k_spinlock_key_t key = k_spin_lock(&(ring).lock); \
    if ((ring).tail != (ring).head) { \
        *(item) = (ring).samples[(ring).tail]; \
        (ring).tail = ((ring).tail + 1) % ARRAY_SIZE((ring).samples); \
        result = true; \
    } else { \
        result = false; \
    } \
    k_spin_unlock(&(ring).lock, key); \
} while(0)

/* NMEA checksum calculation */
uint8_t log_calc_checksum(const char *sentence, size_t len)
{
    uint8_t checksum = 0;
    
    for (size_t i = 0; i < len; i++) {
        checksum ^= (uint8_t)sentence[i];
    }
    
    return checksum;
}

/* Format sentence with checksum */
int log_format_sentence(char *buf, size_t buf_size, const char *fmt, ...)
{
    va_list args;
    int len;
    uint8_t checksum;
    
    /* Format the main sentence */
    va_start(args, fmt);
    len = vsnprintf(buf, buf_size - 5, fmt, args);  /* Leave room for *HH\r\n */
    va_end(args);
    
    if (len < 0 || len >= buf_size - 5) {
        return -1;
    }
    
    /* Calculate checksum (skip the $) */
    checksum = log_calc_checksum(buf + 1, len - 1);
    
    /* Append checksum and CRLF */
    len += snprintf(buf + len, buf_size - len, "*%02X\r\n", checksum);
    
    return len;
}

/* Get relative timestamp */
uint32_t log_get_timestamp_ms(uint64_t session_start_us)
{
    uint64_t now_us = time_now_us();
    uint64_t elapsed_us = now_us - session_start_us;
    return (uint32_t)(elapsed_us / 1000);
}

/* Sensor callbacks */
static void imu_data_callback(const imu_sample_t *samples, size_t count)
{
    for (size_t i = 0; i < count; i++) {
        RING_PUSH(imu_ring, samples[i]);
        last_imu_sample = samples[i];
        have_imu_sample = true;
        stats.imu_count++;
    }
}

static void baro_data_callback(const baro_sample_t *sample)
{
    RING_PUSH(baro_ring, *sample);
    last_baro_sample = *sample;
    have_baro_sample = true;
    stats.env_count++;
}

static void gnss_fix_callback(const gnss_fix_t *fix)
{
    RING_PUSH(gnss_ring, *fix);
    stats.gnss_count++;
    
    /* Update global date string if we have valid date */
    if (fix->date_valid) {
        snprintf(gps_date_string, sizeof(gps_date_string), 
                 "%04d-%02d-%02d", fix->year, fix->month, fix->day);
    }
    
    /* Output GPS fix sentence immediately */
    if (output_callback && fix->position_valid && fix->time_valid) {
        char line[LOG_MAX_SENTENCE_LEN];
        char time_str[32];
        
        /* Format ISO8601 time with milliseconds */
        snprintf(time_str, sizeof(time_str), 
                 "%04d-%02d-%02dT%02d:%02d:%02d.%03dZ",
                 fix->year, fix->month, fix->day,
                 fix->hours, fix->minutes, fix->seconds, fix->milliseconds);
        
        /* Output standard NMEA GGA sentence (if needed) */
        /* ... GGA output code ... */
        
        /* Output $PTH sentence immediately after
         * $PTH contains the millis() timestamp when the GNSS sentence arrived
         */
        uint32_t timestamp_ms = log_get_timestamp_ms(config.session_start_us);
        int len = log_format_sentence(line, sizeof(line),
                                      "$PTH,%u",
                                      timestamp_ms);
        
        if (len > 0) {
            output_callback(line, len);
        }
    }
}

void aggregator_write_version(void)
{
    if (!output_callback) {
        return;
    }
    
    char line[LOG_MAX_SENTENCE_LEN];
    char version_string[64];
    
    /* Format: $PVER,<description>,<numeric_version> */
    /* For Tempo V1, we use the new format with extended info */
    snprintf(version_string, sizeof(version_string),
             "Tempo %s %s (%s)",
             APP_DEVICE_TYPE,
             APP_VERSION_STRING,
             APP_GIT_COMMIT);
    
    int len = log_format_sentence(line, sizeof(line),
                                  "$PVER,\"%s\",%d",
                                  version_string,
                                  APP_VERSION_NUMERIC);
    
    if (len > 0) {
        output_callback(line, len);
    }
}

/* Output work handlers */
static void imu_output_handler(struct k_work *work)
{
    ARG_UNUSED(work);
    
    if (!running || !output_callback || !have_imu_sample) {
        goto reschedule;
    }
    
    char line[LOG_MAX_SENTENCE_LEN];
    uint32_t timestamp_ms = log_get_timestamp_ms(config.session_start_us);
    
    /* Output IMU sentence using latest sample */
    int len = log_format_sentence(line, sizeof(line),
                                  "$PIMU,%u,%.3f,%.3f,%.3f,%.4f,%.4f,%.4f",
                                  timestamp_ms,
                                  last_imu_sample.accel_x,
                                  last_imu_sample.accel_y,
                                  last_imu_sample.accel_z,
                                  last_imu_sample.gyro_x,
                                  last_imu_sample.gyro_y,
                                  last_imu_sample.gyro_z);
    
    if (len > 0) {
        output_callback(line, len);
    }
    
    /* Output quaternion if enabled (placeholder for now) */
    if (config.enable_quaternion) {
        len = log_format_sentence(line, sizeof(line),
                                  "$PIM2,%u,1.0000,0.0000,0.0000,0.0000",
                                  timestamp_ms);
        if (len > 0) {
            output_callback(line, len);
        }
    }
    
reschedule:
    if (running) {
        k_work_reschedule(&imu_output_work, K_MSEC(IMU_OUTPUT_PERIOD_MS));
    }
}

static void env_output_handler(struct k_work *work)
{
    ARG_UNUSED(work);
    
    if (!running || !output_callback || !have_baro_sample) {
        goto reschedule;
    }
    
    char line[LOG_MAX_SENTENCE_LEN];
    uint32_t timestamp_ms = log_get_timestamp_ms(config.session_start_us);
    
    /* Convert pressure from Pa to hPa for compatibility */
    float pressure_hpa = last_baro_sample.pressure_pa / 100.0f;
    
    /* Convert altitude from meters to feet for compatibility */
    float altitude_ft = last_baro_sample.altitude_m * 3.28084f;
    
    /* Battery voltage is -1 for V1 (no battery monitoring) */
    float battery_v = -1.0f;
    
    /* Output environmental sentence using latest sample
     * Format: $PENV,timestamp_ms,pressure_hPa,altitude_ft,battery_v
     */
    int len = log_format_sentence(line, sizeof(line),
                                  "$PENV,%u,%.2f,%.2f,%.2f",
                                  timestamp_ms,
                                  pressure_hpa,
                                  altitude_ft,
                                  battery_v);
    
    if (len > 0) {
        output_callback(line, len);
    }
    
reschedule:
    if (running) {
        k_work_reschedule(&env_output_work, K_MSEC(ENV_OUTPUT_PERIOD_MS));
    }
}

/* Aggregator thread (future use for more complex processing) */
static void aggregator_thread_fn(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);
    
    LOG_INF("Aggregator thread started");
    
    while (running) {
        /* Currently all output is handled by work items */
        /* This thread reserved for future quaternion calculations, etc */
        k_sleep(K_MSEC(100));
    }
    
    LOG_INF("Aggregator thread stopped");
}

/* Public API */
int aggregator_init(void)
{
    LOG_INF("Initializing aggregator service");
    
    /* Initialize ring buffers */
    memset(&imu_ring, 0, sizeof(imu_ring));
    memset(&baro_ring, 0, sizeof(baro_ring));
    memset(&gnss_ring, 0, sizeof(gnss_ring));
    
    /* Initialize work items */
    k_work_init_delayable(&imu_output_work, imu_output_handler);
    k_work_init_delayable(&env_output_work, env_output_handler);
    
    /* Register sensor callbacks */
    imu_register_callback(imu_data_callback);
    baro_register_callback(baro_data_callback);
    gnss_register_fix_callback(gnss_fix_callback);
    
    /* Reset statistics */
    memset(&stats, 0, sizeof(stats));
    
    LOG_INF("Aggregator initialized");
    
    return 0;
}

int aggregator_configure(const aggregator_config_t *cfg)
{
    if (!cfg) {
        return -EINVAL;
    }
    
    config = *cfg;
    
    LOG_INF("Aggregator configured: IMU=%dHz, ENV=%dHz, GNSS=%dHz",
            config.imu_output_rate, config.env_output_rate, config.gnss_output_rate);
    
    return 0;
}

void aggregator_register_output_callback(aggregator_output_callback_t callback)
{
    output_callback = callback;
}

int aggregator_start(void)
{
    if (running) {
        return -EALREADY;
    }
    
    LOG_INF("Starting aggregator");
    
    /* Set session start time if not already set */
    if (config.session_start_us == 0) {
        config.session_start_us = time_now_us();
    }
    
    running = true;
    
    /* Start output work items */
    k_work_schedule(&imu_output_work, K_MSEC(IMU_OUTPUT_PERIOD_MS));
    k_work_schedule(&env_output_work, K_MSEC(ENV_OUTPUT_PERIOD_MS));
    
    /* Create aggregator thread */
    aggregator_tid = k_thread_create(&aggregator_thread,
                                     aggregator_stack,
                                     K_THREAD_STACK_SIZEOF(aggregator_stack),
                                     aggregator_thread_fn,
                                     NULL, NULL, NULL,
                                     K_PRIO_PREEMPT(5), 0, K_NO_WAIT);
    k_thread_name_set(aggregator_tid, "aggregator");
    
    return 0;
}

int aggregator_stop(void)
{
    if (!running) {
        return -EALREADY;
    }
    
    LOG_INF("Stopping aggregator");
    
    running = false;
    
    /* Cancel work items */
    k_work_cancel_delayable(&imu_output_work);
    k_work_cancel_delayable(&env_output_work);
    
    /* Wait for thread to exit */
    if (aggregator_tid) {
        k_thread_join(aggregator_tid, K_FOREVER);
        aggregator_tid = NULL;
    }
    
    return 0;
}

void aggregator_write_session_config(void)
{
    if (!output_callback) {
        return;
    }
    
    char line[LOG_MAX_SENTENCE_LEN];
    char time_str[32];
    
    /* Get current time for session start */
    uint64_t now_us = time_now_us();
    uint32_t now_ms = (uint32_t)(now_us / 1000);
    
    /* If we have GPS time, use ISO format, otherwise use milliseconds */
    if (gps_date_string[0] != '\0') {
        /* We have GPS date but might not have full time yet */
        snprintf(time_str, sizeof(time_str), "%sT00:00:00Z", gps_date_string);
    } else {
        /* No GPS yet, use relative timestamp */
        snprintf(time_str, sizeof(time_str), "T+%u", now_ms);
    }
    
    /* Format: $PSFC,session_id,time,rates,axes
     * rates format: imu:env:gnss:mag
     * axes: always NED for V1
     */
    int len = log_format_sentence(line, sizeof(line),
                                  "$PSFC,%u,%s,%d:%d:%d:%d,NED",
                                  config.session_id,
                                  time_str,
                                  config.imu_output_rate,
                                  config.env_output_rate,
                                  config.gnss_output_rate,
                                  config.mag_output_rate);
    
    if (len > 0) {
        output_callback(line, len);
    }
}

void aggregator_write_state_change(const char *old_state, const char *new_state,
                                   const char *trigger)
{
    if (!output_callback) {
        return;
    }
    
    char line[LOG_MAX_SENTENCE_LEN];
    uint32_t timestamp_ms = log_get_timestamp_ms(config.session_start_us);
    
    /* Format: $PST,timestamp_ms,old_state,new_state,trigger */
    int len = log_format_sentence(line, sizeof(line),
                                  "$PST,%u,%s,%s,%s",
                                  timestamp_ms,
                                  old_state,
                                  new_state,
                                  trigger);
    
    if (len > 0) {
        output_callback(line, len);
    }
}

int aggregator_set_gnss_rate(uint16_t rate_hz)
{
    if (rate_hz != 1 && rate_hz != 10) {
        LOG_ERR("Invalid GNSS rate %d Hz (must be 1 or 10)", rate_hz);
        return -EINVAL;
    }
    
    config.gnss_output_rate = rate_hz;
    
    LOG_INF("GNSS output rate changed to %d Hz", rate_hz);
    
    /* Note: The actual GNSS module configuration should be done by the GNSS service */
    /* This just updates our expected rate for statistics and logging */
    
    return 0;
}

void aggregator_get_stats(uint32_t *imu_count, uint32_t *env_count,
                          uint32_t *gnss_count, uint32_t *dropped_count)
{
    if (imu_count) *imu_count = stats.imu_count;
    if (env_count) *env_count = stats.env_count;
    if (gnss_count) *gnss_count = stats.gnss_count;
    if (dropped_count) *dropped_count = stats.dropped_count;
}

void aggregator_set_session_start_time(uint64_t start_us)
{
    config.session_start_us = start_us;
}

void aggregator_set_session_id(uint32_t id)
{
    config.session_id = id;
}

void aggregator_write_session_header(void)
{
    /* First write version */
    aggregator_write_version();
    
    /* Then write session file config */
    aggregator_write_session_config();
}
