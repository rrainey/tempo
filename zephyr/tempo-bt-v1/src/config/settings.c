/*
 * SPDX-License-Identifier: Apache-2.0
 * 
 * Tempo-BT V1 - Settings Management
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>
#include <string.h>

LOG_MODULE_REGISTER(app_settings, LOG_LEVEL_INF);

/* Settings values with defaults */
static struct {
    /* BLE settings */
    char ble_name[32];
    
    /* Unit system */
    uint8_t unit_system;  /* 0 = metric, 1 = imperial */
    
    /* Sensor rates */
    uint16_t imu_odr;     /* IMU output data rate in Hz */
    uint16_t baro_rate;   /* Barometer rate in Hz */
    uint16_t mag_rate;    /* Magnetometer rate in Hz */
    uint8_t gnss_rate_hz; /* GNSS rate in Hz */
    
    /* Features */
    bool pps_enabled;     /* PPS sync enabled (always false for V1) */
    
    /* Storage */
    char log_backend[16]; /* "littlefs" or "fatfs" */
} app_settings = {
    /* Default values */
    .ble_name = "TempoBT",
    .unit_system = 0,     /* Metric by default */
    .imu_odr = 400,       /* 400 Hz */
    .baro_rate = 50,      /* 50 Hz */
    .mag_rate = 50,       /* 50 Hz */
    .gnss_rate_hz = 5,    /* 5 Hz */
    .pps_enabled = false, /* No PPS on V1 */
    .log_backend = "littlefs"
};

/* Settings handler */
static int settings_set_handler(const char *name, size_t len,
                               settings_read_cb read_cb, void *cb_arg)
{
    const char *next;
    int rc;

    if (settings_name_steq(name, "ble_name", &next) && !next) {
        if (len >= sizeof(app_settings.ble_name)) {
            return -EINVAL;
        }
        rc = read_cb(cb_arg, app_settings.ble_name, len);
        if (rc >= 0) {
            app_settings.ble_name[len] = '\0';
            LOG_INF("Set ble_name: %s", app_settings.ble_name);
        }
        return rc;
    }

    if (settings_name_steq(name, "unit_system", &next) && !next) {
        if (len != sizeof(app_settings.unit_system)) {
            return -EINVAL;
        }
        rc = read_cb(cb_arg, &app_settings.unit_system, len);
        if (rc >= 0) {
            LOG_INF("Set unit_system: %d", app_settings.unit_system);
        }
        return rc;
    }

    if (settings_name_steq(name, "imu_odr", &next) && !next) {
        if (len != sizeof(app_settings.imu_odr)) {
            return -EINVAL;
        }
        rc = read_cb(cb_arg, &app_settings.imu_odr, len);
        if (rc >= 0) {
            LOG_INF("Set imu_odr: %d Hz", app_settings.imu_odr);
        }
        return rc;
    }

    if (settings_name_steq(name, "baro_rate", &next) && !next) {
        if (len != sizeof(app_settings.baro_rate)) {
            return -EINVAL;
        }
        rc = read_cb(cb_arg, &app_settings.baro_rate, len);
        if (rc >= 0) {
            LOG_INF("Set baro_rate: %d Hz", app_settings.baro_rate);
        }
        return rc;
    }

    if (settings_name_steq(name, "gnss_rate_hz", &next) && !next) {
        if (len != sizeof(app_settings.gnss_rate_hz)) {
            return -EINVAL;
        }
        rc = read_cb(cb_arg, &app_settings.gnss_rate_hz, len);
        if (rc >= 0) {
            LOG_INF("Set gnss_rate_hz: %d Hz", app_settings.gnss_rate_hz);
        }
        return rc;
    }

    /* Unknown setting */
    return -ENOENT;
}

static struct settings_handler app_settings_handler = {
    .name = "app",
    .h_set = settings_set_handler
};

int app_settings_init(void)
{
    int rc;

    /* Register settings handler */
    rc = settings_register(&app_settings_handler);
    if (rc) {
        LOG_ERR("Failed to register settings handler: %d", rc);
        return rc;
    }

    /* Initialize settings subsystem */
    rc = settings_subsys_init();
    if (rc) {
        LOG_ERR("Failed to initialize settings subsystem: %d", rc);
        return rc;
    }

    /* Load settings from NVS */
    rc = settings_load();
    if (rc) {
        LOG_ERR("Failed to load settings: %d", rc);
        return rc;
    }

    LOG_INF("Settings loaded successfully");
    return 0;
}

/* Getter functions */
const char *app_settings_get_ble_name(void)
{
    return app_settings.ble_name;
}

uint16_t app_settings_get_imu_odr(void)
{
    return app_settings.imu_odr;
}

uint16_t app_settings_get_baro_rate(void)
{
    return app_settings.baro_rate;
}

uint8_t app_settings_get_gnss_rate(void)
{
    return app_settings.gnss_rate_hz;
}

/* Setter functions with persistence */
int app_settings_set_ble_name(const char *name)
{
    if (!name || strlen(name) >= sizeof(app_settings.ble_name)) {
        return -EINVAL;
    }
    
    strcpy(app_settings.ble_name, name);
    return settings_save_one("app/ble_name", name, strlen(name));
}

int app_settings_set_imu_odr(uint16_t odr_hz)
{
    app_settings.imu_odr = odr_hz;
    return settings_save_one("app/imu_odr", &odr_hz, sizeof(odr_hz));
}

/* Test function to demonstrate settings */
void app_settings_test(void)
{
    LOG_INF("Current settings:");
    LOG_INF("  BLE name: %s", app_settings.ble_name);
    LOG_INF("  Unit system: %s", app_settings.unit_system ? "Imperial" : "Metric");
    LOG_INF("  IMU ODR: %d Hz", app_settings.imu_odr);
    LOG_INF("  Baro rate: %d Hz", app_settings.baro_rate);
    LOG_INF("  GNSS rate: %d Hz", app_settings.gnss_rate_hz);
    
    /* Skip the save test for now to isolate the issue */
    LOG_INF("Settings test complete");
}