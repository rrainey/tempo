/*
 * SPDX-License-Identifier: Apache-2.0
 * 
 * Tempo-BT V1 - Barometer Service (BMP390)
 */

#ifndef SERVICES_BARO_H
#define SERVICES_BARO_H

#include <stdint.h>
#include <stdbool.h>

/* Barometer sample data structure */
typedef struct {
    /* Timestamp in microseconds */
    uint64_t timestamp_us;
    
    /* Pressure in Pascals */
    float pressure_pa;
    
    /* Temperature in Celsius */
    float temperature_c;
    
    /* Calculated altitude in meters (optional) */
    float altitude_m;
    
    /* Data validity flags */
    bool pressure_valid;
    bool temperature_valid;
} baro_sample_t;

/* Barometer configuration */
typedef struct {
    /* Output data rate in Hz (1, 2, 4, 8, 16, 32, 64, 128, 200) */
    uint16_t odr_hz;
    
    /* Oversampling settings */
    uint8_t pressure_oversampling;    /* 1, 2, 4, 8, 16, 32 */
    uint8_t temperature_oversampling; /* 1, 2, 4, 8, 16, 32 */
    
    /* IIR filter coefficient (0 = off, 1, 3, 7, 15, 31, 63, 127) */
    uint8_t iir_filter_coeff;
    
    /* Enable interrupt on data ready */
    bool enable_data_ready_int;
} baro_config_t;

/* Callback type for barometer data */
typedef void (*baro_data_callback_t)(const baro_sample_t *sample);

/**
 * @brief Initialize the barometer
 * 
 * @return 0 on success, negative error code on failure
 */
int baro_init(void);

/**
 * @brief Configure barometer settings
 * 
 * @param config Configuration parameters
 * @return 0 on success, negative error code on failure
 */
int baro_configure(const baro_config_t *config);

/**
 * @brief Register callback for barometer data
 * 
 * @param callback Function to call when new data is available
 */
int baro_register_callback(baro_data_callback_t callback);

/**
 * @brief Start barometer data collection
 * 
 * @return 0 on success, negative error code on failure
 */
int baro_start(void);

/**
 * @brief Stop barometer data collection
 * 
 * @return 0 on success, negative error code on failure
 */
int baro_stop(void);

/**
 * @brief Get current barometer configuration
 * 
 * @param config Output configuration structure
 * @return 0 on success, negative error code on failure
 */
int baro_get_config(baro_config_t *config);

/**
 * @brief Set sea level pressure for altitude calculation
 * 
 * @param pressure_pa Sea level pressure in Pascals
 */
void baro_set_sea_level_pressure(float pressure_pa);

/**
 * @brief Unregister a barometer data callback
 * 
 * @param callback Callback function to unregister
 * @return 0 on success, negative error code on failure
 */
int baro_unregister_callback(baro_data_callback_t callback);

/**
 * @brief Set ground reference from current pressure reading
 * 
 * This should be called when the system is on the ground to establish
 * a reference for AGL (Above Ground Level) calculations.
 * 
 * @return 0 on success, negative error code on failure
 */
int baro_set_ground_reference(void);

/**
 * @brief Get altitude above ground level (AGL) from a sample
 * 
 * @param sample Barometer sample with pressure data
 * @return AGL in meters, or 0 if ground reference not set
 */
float baro_get_agl(const baro_sample_t *sample);

/**
 * @brief Get ground reference values
 * 
 * @param pressure_pa Output: Ground pressure in Pascals (can be NULL)
 * @param altitude_m Output: Ground altitude in meters MSL (can be NULL)
 * @return 0 on success, -ENODATA if ground reference not set
 */
int baro_get_ground_reference(float *pressure_pa, float *altitude_m);

/**
 * @brief Clear ground reference
 * 
 * Resets to standard atmosphere
 */
void baro_clear_ground_reference(void);

#endif /* SERVICES_BARO_H */