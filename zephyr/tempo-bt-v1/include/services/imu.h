/*
 * SPDX-License-Identifier: Apache-2.0
 * 
 * Tempo-BT V1 - IMU Service Interface
 */

#ifndef SERVICES_IMU_H
#define SERVICES_IMU_H

#include <stdint.h>
#include <stdbool.h>

typedef struct {
    uint64_t timestamp_us;
    float accel_x;          /* m/s^2 */
    float accel_y;          /* m/s^2 */
    float accel_z;          /* m/s^2 */
    float gyro_x;           /* rad/s */
    float gyro_y;           /* rad/s */
    float gyro_z;           /* rad/s */
    float temperature;      /* Celsius */
} imu_sample_t;

typedef struct {
    uint16_t accel_odr_hz;
    uint16_t gyro_odr_hz;
    uint8_t accel_range_g;
    uint16_t gyro_range_dps;
} imu_config_t;

typedef void (*imu_data_callback_t)(const imu_sample_t *samples, size_t count);

int imu_init(void);
int imu_configure(const imu_config_t *config);
void imu_register_callback(imu_data_callback_t callback);
int imu_start(void);
int imu_stop(void);
int imu_get_config(imu_config_t *config);

uint64_t time_now_us(void);

#endif /* SERVICES_IMU_H */