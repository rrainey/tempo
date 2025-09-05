/*
 * SPDX-License-Identifier: Apache-2.0
 * 
 * Tempo-BT V1 - IMU Service Implementation
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include "services/imu.h"

LOG_MODULE_REGISTER(imu_service, LOG_LEVEL_INF);

static const struct device *imu_dev;
static imu_data_callback_t data_callback;
static imu_config_t current_config = {
    .accel_odr_hz = 200,
    .gyro_odr_hz = 200,
    .accel_range_g = 16,
    .gyro_range_dps = 2000
};

K_THREAD_STACK_DEFINE(imu_thread_stack, 2048);
static struct k_thread imu_thread_data;
static k_tid_t imu_thread_id;
static volatile bool running;

/* Optional trigger support */
#ifdef CONFIG_ICM42688_TRIGGER
static struct sensor_trigger data_ready_trigger = {
    .type = SENSOR_TRIG_DATA_READY,
    .chan = SENSOR_CHAN_ACCEL_XYZ,
};

static void imu_trigger_handler(const struct device *dev,
                               const struct sensor_trigger *trig)
{
    struct sensor_value accel[3], gyro[3], temp;
    imu_sample_t sample;
    
    if (sensor_sample_fetch(dev) < 0) {
        LOG_ERR("Sample fetch failed");
        return;
    }
    
    sensor_channel_get(dev, SENSOR_CHAN_ACCEL_XYZ, accel);
    sensor_channel_get(dev, SENSOR_CHAN_GYRO_XYZ, gyro);
    sensor_channel_get(dev, SENSOR_CHAN_DIE_TEMP, &temp);
    
    sample.timestamp_us = time_now_us();
    sample.accel_x = sensor_value_to_float(&accel[0]);
    sample.accel_y = sensor_value_to_float(&accel[1]);
    sample.accel_z = sensor_value_to_float(&accel[2]);
    sample.gyro_x = sensor_value_to_float(&gyro[0]);
    sample.gyro_y = sensor_value_to_float(&gyro[1]);
    sample.gyro_z = sensor_value_to_float(&gyro[2]);
    sample.temperature = sensor_value_to_float(&temp);
    
    if (data_callback) {
        data_callback(&sample, 1);
    }
}
#endif

static void imu_thread_fn(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);
    
    struct sensor_value accel[3], gyro[3], temp;
    imu_sample_t sample;
    uint32_t sleep_ms = 1000 / current_config.accel_odr_hz;
    
    LOG_INF("IMU polling thread started");
    
    while (running) {
        if (sensor_sample_fetch(imu_dev) < 0) {
            LOG_ERR("Sample fetch failed");
            k_sleep(K_MSEC(sleep_ms));
            continue;
        }
        
        sensor_channel_get(imu_dev, SENSOR_CHAN_ACCEL_XYZ, accel);
        sensor_channel_get(imu_dev, SENSOR_CHAN_GYRO_XYZ, gyro);
        sensor_channel_get(imu_dev, SENSOR_CHAN_DIE_TEMP, &temp);
        
        sample.timestamp_us = time_now_us();
        sample.accel_x = sensor_value_to_float(&accel[0]);
        sample.accel_y = sensor_value_to_float(&accel[1]);
        sample.accel_z = sensor_value_to_float(&accel[2]);
        sample.gyro_x = sensor_value_to_float(&gyro[0]);
        sample.gyro_y = sensor_value_to_float(&gyro[1]);
        sample.gyro_z = sensor_value_to_float(&gyro[2]);
        sample.temperature = sensor_value_to_float(&temp);
        
        if (data_callback) {
            data_callback(&sample, 1);
        }
        
        k_sleep(K_MSEC(sleep_ms));
    }
    
    LOG_INF("IMU polling thread stopped");
}

int imu_init(void)
{
    LOG_INF("Initializing IMU service");
    
    //imu_dev = DEVICE_DT_GET(DT_ALIAS(imu0));
    imu_dev = DEVICE_DT_GET_ONE(invensense_icm42688);

    extern int icm42688_init(const struct device *dev);

    icm42688_init(imu_dev);

    if (!device_is_ready(imu_dev)) {
        LOG_ERR("IMU device not ready");
        return -ENODEV;
    }
    
    LOG_INF("IMU device ready: %s", imu_dev->name);
    
#ifdef CONFIG_ICM42688_TRIGGER
    /* Use interrupt-driven mode if available */
    if (sensor_trigger_set(imu_dev, &data_ready_trigger, 
                          imu_trigger_handler) < 0) {
        LOG_WRN("Failed to set trigger, falling back to polling");
    } else {
        LOG_INF("Configured for interrupt mode");
    }
#endif
    
    return 0;
}

int imu_configure(const imu_config_t *config)
{
    struct sensor_value val;
    int ret;
    
    current_config = *config;
    
    /* Set accelerometer range */
    val.val1 = config->accel_range_g;
    val.val2 = 0;
    ret = sensor_attr_set(imu_dev, SENSOR_CHAN_ACCEL_XYZ,
                         SENSOR_ATTR_FULL_SCALE, &val);
    if (ret < 0) {
        LOG_ERR("Failed to set accel range: %d", ret);
        return ret;
    }
    
    /* Set gyroscope range */
    val.val1 = config->gyro_range_dps;
    val.val2 = 0;
    ret = sensor_attr_set(imu_dev, SENSOR_CHAN_GYRO_XYZ,
                         SENSOR_ATTR_FULL_SCALE, &val);
    if (ret < 0) {
        LOG_ERR("Failed to set gyro range: %d", ret);
        return ret;
    }
    
    /* Set sampling frequency */
    val.val1 = config->accel_odr_hz;
    val.val2 = 0;
    ret = sensor_attr_set(imu_dev, SENSOR_CHAN_ACCEL_XYZ,
                         SENSOR_ATTR_SAMPLING_FREQUENCY, &val);
    if (ret < 0) {
        LOG_ERR("Failed to set accel ODR: %d", ret);
        return ret;
    }
    
    ret = sensor_attr_set(imu_dev, SENSOR_CHAN_GYRO_XYZ,
                         SENSOR_ATTR_SAMPLING_FREQUENCY, &val);
    if (ret < 0) {
        LOG_ERR("Failed to set gyro ODR: %d", ret);
        return ret;
    }
    
    LOG_INF("IMU configured: %dHz, %dg, %ddps",
            config->accel_odr_hz, config->accel_range_g, 
            config->gyro_range_dps);
    
    return 0;
}

void imu_register_callback(imu_data_callback_t callback)
{
    data_callback = callback;
}

int imu_start(void)
{
    if (running) {
        return -EALREADY;
    }
    
    LOG_INF("Starting IMU data collection");
    running = true;
    
#ifndef CONFIG_ICM42688_TRIGGER
    /* Start polling thread if not using interrupts */
    imu_thread_id = k_thread_create(&imu_thread_data,
                                   imu_thread_stack,
                                   K_THREAD_STACK_SIZEOF(imu_thread_stack),
                                   imu_thread_fn,
                                   NULL, NULL, NULL,
                                   K_PRIO_PREEMPT(5), 0, K_NO_WAIT);
    k_thread_name_set(imu_thread_id, "imu_poll");
#endif
    
    return 0;
}

int imu_stop(void)
{
    if (!running) {
        return -EALREADY;
    }
    
    LOG_INF("Stopping IMU data collection");
    running = false;
    
#ifndef CONFIG_ICM42688_TRIGGER
    if (imu_thread_id) {
        k_thread_join(imu_thread_id, K_FOREVER);
        imu_thread_id = NULL;
    }
#endif
    
    return 0;
}

int imu_get_config(imu_config_t *config)
{
    *config = current_config;
    return 0;
}