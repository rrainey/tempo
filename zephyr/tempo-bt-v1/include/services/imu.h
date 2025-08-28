/*
 * SPDX-License-Identifier: Apache-2.0
 * 
 * Tempo-BT V1 - IMU Service
 */

#ifndef SERVICES_IMU_H
#define SERVICES_IMU_H

#include <stdint.h>
#include <stdbool.h>

/* IMU sample data structure */
typedef struct {
    /* Timestamp in microseconds */
    uint64_t timestamp_us;
    
    /* Accelerometer data in m/s^2 */
    float accel_x;
    float accel_y;
    float accel_z;
    
    /* Gyroscope data in rad/s */
    float gyro_x;
    float gyro_y;
    float gyro_z;
    
    /* Temperature in Celsius */
    float temperature;
} imu_sample_t;

/* IMU configuration */
typedef struct {
    uint16_t accel_odr_hz;      /* Accelerometer output data rate */
    uint16_t gyro_odr_hz;       /* Gyroscope output data rate */
    uint8_t accel_range_g;      /* Accelerometer range (2, 4, 8, 16 g) */
    uint16_t gyro_range_dps;    /* Gyroscope range (250, 500, 1000, 2000 dps) */
    bool fifo_enabled;          /* Enable FIFO buffering */
} imu_config_t;

/* Callback type for IMU data */
typedef void (*imu_data_callback_t)(const imu_sample_t *samples, size_t count);

/**
 * @brief Initialize the IMU
 * 
 * @return 0 on success, negative error code on failure
 */
int imu_init(void);

/**
 * @brief Configure IMU settings
 * 
 * @param config Configuration parameters
 * @return 0 on success, negative error code on failure
 */
int imu_configure(const imu_config_t *config);

/**
 * @brief Register callback for IMU data
 * 
 * @param callback Function to call when new data is available
 */
void imu_register_callback(imu_data_callback_t callback);

/**
 * @brief Start IMU data collection
 * 
 * @return 0 on success, negative error code on failure
 */
int imu_start(void);

/**
 * @brief Stop IMU data collection
 * 
 * @return 0 on success, negative error code on failure
 */
int imu_stop(void);

/**
 * @brief Get current IMU configuration
 * 
 * @param config Output configuration structure
 * @return 0 on success, negative error code on failure
 */
int imu_get_config(imu_config_t *config);

#endif /* SERVICES_IMU_H */