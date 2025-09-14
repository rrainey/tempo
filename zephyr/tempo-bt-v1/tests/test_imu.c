/*
 * SPDX-License-Identifier: Apache-2.0
 * 
 * Tempo-BT V1 - IMU Test Program
 */
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/shell/shell.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

#include "services/imu.h"
#include "services/timebase.h"

LOG_MODULE_REGISTER(test_imu, LOG_LEVEL_INF);

#ifdef CONFIG_SHELL

/* Test state */
static struct {
    bool running;
    uint32_t sample_count;
    uint32_t error_count;
    uint64_t start_time;
    uint64_t last_sample_time;
    
    /* Statistics */
    struct {
        float accel_min[3];
        float accel_max[3];
        float accel_sum[3];
        float gyro_min[3];
        float gyro_max[3];
        float gyro_sum[3];
        float temp_min;
        float temp_max;
        float temp_sum;
    } stats;
} test_state;

/* Ring buffer for samples */
#define SAMPLE_BUFFER_SIZE 1000
static imu_sample_t sample_buffer[SAMPLE_BUFFER_SIZE];
static uint32_t sample_write_idx = 0;
static uint32_t sample_read_idx = 0;

/* Test configurations */
static const imu_config_t test_configs[] = {
    /* Basic 200Hz test */
    {
        .accel_odr_hz = 200,
        .gyro_odr_hz = 200,
        .accel_range_g = 16,
        .gyro_range_dps = 2000,
    },
    /* High rate 400Hz test */
    {
        .accel_odr_hz = 400,
        .gyro_odr_hz = 400,
        .accel_range_g = 16,
        .gyro_range_dps = 2000,
    },
    /* Low rate test */
    {
        .accel_odr_hz = 100,
        .gyro_odr_hz = 100,
        .accel_range_g = 8,
        .gyro_range_dps = 1000,
    },
    /* Low range test */
    {
        .accel_odr_hz = 100,
        .gyro_odr_hz = 100,
        .accel_range_g = 2,
        .gyro_range_dps = 250,
    }
};

/* Helper functions */
static void reset_stats(void)
{
    test_state.sample_count = 0;
    test_state.error_count = 0;
    
    for (int i = 0; i < 3; i++) {
        test_state.stats.accel_min[i] = INFINITY;
        test_state.stats.accel_max[i] = -INFINITY;
        test_state.stats.accel_sum[i] = 0.0f;
        test_state.stats.gyro_min[i] = INFINITY;
        test_state.stats.gyro_max[i] = -INFINITY;
        test_state.stats.gyro_sum[i] = 0.0f;
    }
    
    test_state.stats.temp_min = INFINITY;
    test_state.stats.temp_max = -INFINITY;
    test_state.stats.temp_sum = 0.0f;
}

static void update_stats(const imu_sample_t *sample)
{
    /* Update accelerometer stats */
    test_state.stats.accel_sum[0] += sample->accel_x;
    test_state.stats.accel_sum[1] += sample->accel_y;
    test_state.stats.accel_sum[2] += sample->accel_z;
    
    if (sample->accel_x < test_state.stats.accel_min[0]) 
        test_state.stats.accel_min[0] = sample->accel_x;
    if (sample->accel_x > test_state.stats.accel_max[0]) 
        test_state.stats.accel_max[0] = sample->accel_x;
    
    if (sample->accel_y < test_state.stats.accel_min[1]) 
        test_state.stats.accel_min[1] = sample->accel_y;
    if (sample->accel_y > test_state.stats.accel_max[1]) 
        test_state.stats.accel_max[1] = sample->accel_y;
    
    if (sample->accel_z < test_state.stats.accel_min[2]) 
        test_state.stats.accel_min[2] = sample->accel_z;
    if (sample->accel_z > test_state.stats.accel_max[2]) 
        test_state.stats.accel_max[2] = sample->accel_z;
    
    /* Update gyroscope stats */
    test_state.stats.gyro_sum[0] += sample->gyro_x;
    test_state.stats.gyro_sum[1] += sample->gyro_y;
    test_state.stats.gyro_sum[2] += sample->gyro_z;
    
    if (sample->gyro_x < test_state.stats.gyro_min[0]) 
        test_state.stats.gyro_min[0] = sample->gyro_x;
    if (sample->gyro_x > test_state.stats.gyro_max[0]) 
        test_state.stats.gyro_max[0] = sample->gyro_x;
    
    if (sample->gyro_y < test_state.stats.gyro_min[1]) 
        test_state.stats.gyro_min[1] = sample->gyro_y;
    if (sample->gyro_y > test_state.stats.gyro_max[1]) 
        test_state.stats.gyro_max[1] = sample->gyro_y;
    
    if (sample->gyro_z < test_state.stats.gyro_min[2]) 
        test_state.stats.gyro_min[2] = sample->gyro_z;
    if (sample->gyro_z > test_state.stats.gyro_max[2]) 
        test_state.stats.gyro_max[2] = sample->gyro_z;
    
    /* Update temperature stats */
    test_state.stats.temp_sum += sample->temperature;
    if (sample->temperature < test_state.stats.temp_min) 
        test_state.stats.temp_min = sample->temperature;
    if (sample->temperature > test_state.stats.temp_max) 
        test_state.stats.temp_max = sample->temperature;
}

static void print_stats(void)
{
    if (test_state.sample_count == 0) {
        LOG_INF("No samples collected");
        return;
    }
    
    uint64_t duration_us = test_state.last_sample_time - test_state.start_time;
    float duration_s = duration_us / 1000000.0f;
    float actual_rate = test_state.sample_count / duration_s;
    
    LOG_INF("=== IMU Test Results ===");
    LOG_INF("Duration: %.2f seconds", duration_s);
    LOG_INF("Samples: %u", test_state.sample_count);
    LOG_INF("Errors: %u", test_state.error_count);
    LOG_INF("Actual rate: %.1f Hz", actual_rate);
    
    LOG_INF("Accelerometer (m/s²):");
    LOG_INF("  X: min=%.3f, max=%.3f, avg=%.3f",
            test_state.stats.accel_min[0],
            test_state.stats.accel_max[0],
            test_state.stats.accel_sum[0] / test_state.sample_count);
    LOG_INF("  Y: min=%.3f, max=%.3f, avg=%.3f",
            test_state.stats.accel_min[1],
            test_state.stats.accel_max[1],
            test_state.stats.accel_sum[1] / test_state.sample_count);
    LOG_INF("  Z: min=%.3f, max=%.3f, avg=%.3f",
            test_state.stats.accel_min[2],
            test_state.stats.accel_max[2],
            test_state.stats.accel_sum[2] / test_state.sample_count);
    
    /* Calculate magnitude of average acceleration */
    float avg_x = test_state.stats.accel_sum[0] / test_state.sample_count;
    float avg_y = test_state.stats.accel_sum[1] / test_state.sample_count;
    float avg_z = test_state.stats.accel_sum[2] / test_state.sample_count;
    float magnitude = sqrtf(avg_x*avg_x + avg_y*avg_y + avg_z*avg_z);
    LOG_INF("  Magnitude: %.3f m/s² (should be ~9.81 if stationary)", magnitude);
    
    LOG_INF("Gyroscope (rad/s):");
    LOG_INF("  X: min=%.3f, max=%.3f, avg=%.3f",
            test_state.stats.gyro_min[0],
            test_state.stats.gyro_max[0],
            test_state.stats.gyro_sum[0] / test_state.sample_count);
    LOG_INF("  Y: min=%.3f, max=%.3f, avg=%.3f",
            test_state.stats.gyro_min[1],
            test_state.stats.gyro_max[1],
            test_state.stats.gyro_sum[1] / test_state.sample_count);
    LOG_INF("  Z: min=%.3f, max=%.3f, avg=%.3f",
            test_state.stats.gyro_min[2],
            test_state.stats.gyro_max[2],
            test_state.stats.gyro_sum[2] / test_state.sample_count);
    
    LOG_INF("Temperature (°C):");
    LOG_INF("  min=%.1f, max=%.1f, avg=%.1f",
            test_state.stats.temp_min,
            test_state.stats.temp_max,
            test_state.stats.temp_sum / test_state.sample_count);
}

/* IMU data callback */
static void imu_test_callback(const imu_sample_t *samples, size_t count)
{
    for (size_t i = 0; i < count; i++) {
        if (test_state.running) {
            /* Store sample */
            uint32_t next_idx = (sample_write_idx + 1) % SAMPLE_BUFFER_SIZE;
            if (next_idx != sample_read_idx) {
                sample_buffer[sample_write_idx] = samples[i];
                sample_write_idx = next_idx;
            }
            
            /* Update stats */
            test_state.sample_count++;
            test_state.last_sample_time = samples[i].timestamp_us;
            update_stats(&samples[i]);
            
            /* Print every 100th sample */
            if (test_state.sample_count % 100 == 0) {
                LOG_INF("Sample %u: Ax=%.2f, Ay=%.2f, Az=%.2f, Gx=%.2f, Gy=%.2f, Gz=%.2f, T=%.1f",
                        test_state.sample_count,
                        samples[i].accel_x, samples[i].accel_y, samples[i].accel_z,
                        samples[i].gyro_x, samples[i].gyro_y, samples[i].gyro_z,
                        samples[i].temperature);
            }
        }
    }
}

/* Test functions */
static int test_basic_operation(void)
{
    int ret;
    
    LOG_INF("=== Basic Operation Test ===");
    
    /* Configure for basic operation */
    ret = imu_configure(&test_configs[0]);
    if (ret < 0) {
        LOG_ERR("Failed to configure IMU: %d", ret);
        return ret;
    }
    
    /* Start data collection */
    reset_stats();
    test_state.running = true;
    test_state.start_time = time_now_us();
    
    ret = imu_start();
    if (ret < 0) {
        LOG_ERR("Failed to start IMU: %d", ret);
        return ret;
    }
    
    /* Collect data for 5 seconds */
    LOG_INF("Collecting data for 5 seconds...");
    k_sleep(K_SECONDS(5));
    
    /* Stop collection */
    test_state.running = false;
    ret = imu_stop();
    if (ret < 0) {
        LOG_ERR("Failed to stop IMU: %d", ret);
        return ret;
    }
    
    /* Print results */
    print_stats();
    
    return 0;
}

static int test_configuration_changes(void)
{
    int ret;
    imu_config_t config;
    
    LOG_INF("=== Configuration Change Test ===");
    
    /* Test different configurations */
    for (int i = 0; i < ARRAY_SIZE(test_configs); i++) {
        LOG_INF("Testing config %d: %dHz, %dg, %ddps",
                i, test_configs[i].accel_odr_hz, test_configs[i].accel_range_g,
                test_configs[i].gyro_range_dps);
        
        ret = imu_configure(&test_configs[i]);
        if (ret < 0) {
            LOG_ERR("Failed to configure: %d", ret);
            continue;
        }
        
        /* Verify configuration */
        ret = imu_get_config(&config);
        if (ret < 0) {
            LOG_ERR("Failed to get config: %d", ret);
            continue;
        }
        
        if (config.accel_odr_hz != test_configs[i].accel_odr_hz ||
            config.gyro_odr_hz != test_configs[i].gyro_odr_hz ||
            config.accel_range_g != test_configs[i].accel_range_g ||
            config.gyro_range_dps != test_configs[i].gyro_range_dps) {
            LOG_ERR("Configuration mismatch!");
        } else {
            LOG_INF("Configuration verified OK");
        }
        
        /* Brief data collection */
        reset_stats();
        test_state.running = true;
        test_state.start_time = time_now_us();
        
        ret = imu_start();
        if (ret == 0) {
            k_sleep(K_SECONDS(1));
            test_state.running = false;
            imu_stop();
            
            LOG_INF("Collected %u samples in 1 second", test_state.sample_count);
        }
    }
    
    return 0;
}

/* Direct register test using Zephyr sensor API */
static int test_direct_read(void)
{
    const struct device *dev = DEVICE_DT_GET(DT_ALIAS(imu0));
    struct sensor_value accel[3], gyro[3], temp;
    int ret;
    
    LOG_INF("=== Direct Read Test ===");
    
    if (!device_is_ready(dev)) {
        LOG_ERR("Device not ready");
        return -ENODEV;
    }
    
    for (int i = 0; i < 10; i++) {
        ret = sensor_sample_fetch(dev);
        if (ret < 0) {
            LOG_ERR("Sample fetch failed: %d", ret);
            continue;
        }
        
        sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, accel);
        sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ, gyro);
        sensor_channel_get(dev, SENSOR_CHAN_DIE_TEMP, &temp);
        
        LOG_INF("Sample %d: Ax=%.2f, Ay=%.2f, Az=%.2f, Gx=%.2f, Gy=%.2f, Gz=%.2f, T=%.1f",
                i,
                sensor_value_to_float(&accel[0]),
                sensor_value_to_float(&accel[1]),
                sensor_value_to_float(&accel[2]),
                sensor_value_to_float(&gyro[0]),
                sensor_value_to_float(&gyro[1]),
                sensor_value_to_float(&gyro[2]),
                sensor_value_to_float(&temp));
        
        k_msleep(100);
    }
    
    return 0;
}

/* Shell commands */
static int cmd_imu_test_start(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);
    
    shell_print(sh, "Starting IMU tests...");
    
    /* Direct read test first */
    test_direct_read();
    k_sleep(K_SECONDS(1));
    
    /* Run all tests */
    test_basic_operation();
    k_sleep(K_SECONDS(1));
    
    test_configuration_changes();
    
    shell_print(sh, "IMU tests complete");
    
    return 0;
}

static int cmd_imu_test_stream(const struct shell *sh, size_t argc, char **argv)
{
    if (argc < 2) {
        shell_print(sh, "Usage: imu test stream <seconds>");
        return -EINVAL;
    }
    
    int seconds = atoi(argv[1]);
    if (seconds <= 0 || seconds > 60) {
        shell_print(sh, "Duration must be 1-60 seconds");
        return -EINVAL;
    }
    
    shell_print(sh, "Streaming IMU data for %d seconds...", seconds);
    
    reset_stats();
    test_state.running = true;
    test_state.start_time = time_now_us();
    
    int ret = imu_start();
    if (ret < 0) {
        shell_print(sh, "Failed to start IMU: %d", ret);
        return ret;
    }
    
    /* Print header */
    shell_print(sh, "Time(ms), Ax(m/s²), Ay(m/s²), Az(m/s²), Gx(rad/s), Gy(rad/s), Gz(rad/s), T(°C)");
    
    /* Stream data */
    uint64_t end_time = k_uptime_get() + (seconds * 1000);
    
    while (k_uptime_get() < end_time) {
        /* Print any new samples */
        while (sample_read_idx != sample_write_idx) {
            imu_sample_t *s = &sample_buffer[sample_read_idx];
            uint32_t time_ms = (s->timestamp_us - test_state.start_time) / 1000;
            
            shell_print(sh, "%u, %.3f, %.3f, %.3f, %.3f, %.3f, %.3f, %.1f",
                        time_ms,
                        s->accel_x, s->accel_y, s->accel_z,
                        s->gyro_x, s->gyro_y, s->gyro_z,
                        s->temperature);
            
            sample_read_idx = (sample_read_idx + 1) % SAMPLE_BUFFER_SIZE;
        }
        
        k_sleep(K_MSEC(10));
    }
    
    test_state.running = false;
    imu_stop();
    
    print_stats();
    
    return 0;
}

static int cmd_imu_test_motion(const struct shell *sh, size_t argc, char **argv)
{
    ARG_UNUSED(argc);
    ARG_UNUSED(argv);
    
    shell_print(sh, "Motion detection test - move the device!");
    
    reset_stats();
    test_state.running = true;
    test_state.start_time = time_now_us();
    
    int ret = imu_start();
    if (ret < 0) {
        shell_print(sh, "Failed to start IMU: %d", ret);
        return ret;
    }
    
    float motion_threshold = 2.0f;  /* m/s² */
    float rotation_threshold = 0.5f; /* rad/s */
    uint32_t motion_events = 0;
    uint32_t rotation_events = 0;
    
    shell_print(sh, "Monitoring for 10 seconds...");
    
    for (int i = 0; i < 100; i++) {  /* 10 seconds at 100ms intervals */
        k_sleep(K_MSEC(100));
        
        /* Check latest sample for motion */
        if (sample_write_idx != sample_read_idx) {
            uint32_t idx = (sample_write_idx + SAMPLE_BUFFER_SIZE - 1) % SAMPLE_BUFFER_SIZE;
            imu_sample_t *s = &sample_buffer[idx];
            
            /* Check for linear motion (deviation from gravity) */
            float accel_magnitude = sqrtf(s->accel_x*s->accel_x + 
                                         s->accel_y*s->accel_y + 
                                         s->accel_z*s->accel_z);
            if (fabsf(accel_magnitude - 9.81f) > motion_threshold) {
                motion_events++;
                shell_print(sh, "Motion detected! Magnitude: %.2f m/s²", accel_magnitude);
            }
            
            /* Check for rotation */
            float gyro_magnitude = sqrtf(s->gyro_x*s->gyro_x + 
                                        s->gyro_y*s->gyro_y + 
                                        s->gyro_z*s->gyro_z);
            if (gyro_magnitude > rotation_threshold) {
                rotation_events++;
                shell_print(sh, "Rotation detected! Magnitude: %.2f rad/s", gyro_magnitude);
            }
        }
    }
    
    test_state.running = false;
    imu_stop();
    
    shell_print(sh, "Motion events: %u", motion_events);
    shell_print(sh, "Rotation events: %u", rotation_events);
    
    return 0;
}

/* Shell command registration */
SHELL_STATIC_SUBCMD_SET_CREATE(imu_test_cmds,
    SHELL_CMD(start, NULL, "Run all IMU tests", cmd_imu_test_start),
    SHELL_CMD(stream, NULL, "Stream IMU data for N seconds", cmd_imu_test_stream),
    SHELL_CMD(motion, NULL, "Detect motion events", cmd_imu_test_motion),
    SHELL_SUBCMD_SET_END
);

SHELL_CMD_REGISTER(imu_test, &imu_test_cmds, "IMU test commands", NULL);

/* Initialization */
int imu_test_init(void)
{
    int ret;
    
    LOG_INF("Initializing IMU test module");
    
    /* Initialize IMU */
    ret = imu_init();
    if (ret < 0) {
        LOG_ERR("Failed to initialize IMU: %d", ret);
        return ret;
    }
    
    /* Register callback */
    imu_register_callback(imu_test_callback);
    
    /* Configure for default operation */
    ret = imu_configure(&test_configs[0]);
    if (ret < 0) {
        LOG_ERR("Failed to configure IMU: %d", ret);
        return ret;
    }
    
    LOG_INF("IMU test module ready");
    LOG_INF("Use 'imu_test start' to run all tests");
    LOG_INF("Use 'imu_test stream <seconds>' to stream data");
    LOG_INF("Use 'imu_test motion' to detect motion");
    
    return 0;
}

/* Call this from main() after timebase_init() */
// imu_test_init();

#endif