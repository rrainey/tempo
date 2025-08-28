/*
 * SPDX-License-Identifier: Apache-2.0
 * 
 * Test BMP390 Barometer
 */
#pragma GCC diagnostic ignored "-Wdouble-promotion"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include "services/baro.h"

LOG_MODULE_REGISTER(test_baro, LOG_LEVEL_DBG);

static int sample_count = 0;

static void baro_test_callback(const baro_sample_t *sample)
{
    sample_count++;
    
    LOG_INF("Baro sample #%d:", sample_count);
    LOG_INF("  Timestamp: %llu us", sample->timestamp_us);
    LOG_INF("  Pressure: %.2f Pa (%.2f hPa)", sample->pressure_pa, sample->pressure_pa / 100.0f);
    LOG_INF("  Temperature: %.2f °C", sample->temperature_c);
    LOG_INF("  Altitude: %.2f m", sample->altitude_m);
}

int test_baro(void)
{
    int ret;
    baro_config_t config;
    
    LOG_INF("=== BMP390 Barometer Test ===");
    
    /* Initialize barometer */
    ret = baro_init();
    if (ret < 0) {
        LOG_ERR("Failed to initialize barometer: %d", ret);
        return ret;
    }
    
    /* Register callback */
    baro_register_callback(baro_test_callback);
    
    /* Get current configuration */
    ret = baro_get_config(&config);
    if (ret == 0) {
        LOG_INF("Current configuration:");
        LOG_INF("  ODR: %d Hz", config.odr_hz);
        LOG_INF("  Pressure oversampling: %dx", config.pressure_oversampling);
        LOG_INF("  Temperature oversampling: %dx", config.temperature_oversampling);
        LOG_INF("  IIR filter coefficient: %d", config.iir_filter_coeff);
    }
    
    /* Start measurements */
    ret = baro_start();
    if (ret < 0) {
        LOG_ERR("Failed to start barometer: %d", ret);
        return ret;
    }
    
    LOG_INF("Barometer started, collecting samples for 10 seconds...");
    
    /* Collect samples for 10 seconds */
    k_sleep(K_SECONDS(10));
    
    /* Stop measurements */
    ret = baro_stop();
    if (ret < 0) {
        LOG_ERR("Failed to stop barometer: %d", ret);
    }
    
    LOG_INF("Barometer test completed. Collected %d samples", sample_count);
    
    return 0;
}