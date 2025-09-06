/*
 * SPDX-License-Identifier: Apache-2.0
 * 
 * Tempo-BT V1 - Settings Management
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/settings/settings.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/random/random.h>
#include <string.h>

LOG_MODULE_REGISTER(app_settings, LOG_LEVEL_INF);

/* Default UUIDs - these will be replaced with generated values on first boot */
static const struct bt_uuid_128 default_user_uuid = BT_UUID_INIT_128(
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
);

static const struct bt_uuid_128 default_device_uuid = BT_UUID_INIT_128(
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
);

/* Settings values with defaults */
static struct {
    /* BLE settings */
    char ble_name[32];
    
    /* User and device UUIDs */
    struct bt_uuid_128 user_uuid;
    struct bt_uuid_128 device_uuid;
    
    /* Features */
    bool pps_enabled;     /* PPS sync enabled (always false for V1) */
    
    /* Storage */
    char log_backend[16]; /* "littlefs" or "fatfs" */
} app_settings = {
    /* Default values */
    .ble_name = "TempoBT",
    .user_uuid = BT_UUID_INIT_128(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
    .device_uuid = BT_UUID_INIT_128(0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0),
    .pps_enabled = false, /* No PPS on V1 */
    .log_backend = "littlefs"
};

/* Generate a new random UUID */
int app_settings_generate_uuid(struct bt_uuid_128 *uuid)
{
    if (!uuid) {
        return -EINVAL;
    }

    /* Generate 16 random bytes */
    sys_rand_get(uuid->val, sizeof(uuid->val));
    
    /* Set UUID version to 4 (random) and variant bits */
    uuid->val[6] = (uuid->val[6] & 0x0F) | 0x40;  /* Version 4 */
    uuid->val[8] = (uuid->val[8] & 0x3F) | 0x80;  /* Variant bits */
    
    return 0;
}

/* Check if UUID is all zeros (uninitialized) */
static bool is_uuid_empty(const struct bt_uuid_128 *uuid)
{
    for (int i = 0; i < 16; i++) {
        if (uuid->val[i] != 0) {
            return false;
        }
    }
    return true;
}

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

    if (settings_name_steq(name, "user_uuid", &next) && !next) {
        if (len != sizeof(app_settings.user_uuid.val)) {
            return -EINVAL;
        }
        rc = read_cb(cb_arg, app_settings.user_uuid.val, len);
        if (rc >= 0) {
            char uuid_str[BT_UUID_STR_LEN];
            bt_uuid_to_str((const struct bt_uuid *)&app_settings.user_uuid, 
                          uuid_str, sizeof(uuid_str));
            LOG_INF("Set user_uuid: %s", uuid_str);
        }
        return rc;
    }

    if (settings_name_steq(name, "device_uuid", &next) && !next) {
        if (len != sizeof(app_settings.device_uuid.val)) {
            return -EINVAL;
        }
        rc = read_cb(cb_arg, app_settings.device_uuid.val, len);
        if (rc >= 0) {
            char uuid_str[BT_UUID_STR_LEN];
            bt_uuid_to_str((const struct bt_uuid *)&app_settings.device_uuid, 
                          uuid_str, sizeof(uuid_str));
            LOG_INF("Set device_uuid: %s", uuid_str);
        }
        return rc;
    }

    if (settings_name_steq(name, "log_backend", &next) && !next) {
        if (len >= sizeof(app_settings.log_backend)) {
            return -EINVAL;
        }
        rc = read_cb(cb_arg, app_settings.log_backend, len);
        if (rc >= 0) {
            app_settings.log_backend[len] = '\0';
            LOG_INF("Set log_backend: %s", app_settings.log_backend);
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

    /* Generate UUIDs if they haven't been set */
    bool need_save = false;
    
    if (is_uuid_empty(&app_settings.user_uuid)) {
        LOG_INF("Generating new user UUID");
        rc = app_settings_generate_uuid(&app_settings.user_uuid);
        if (rc == 0) {
            need_save = true;
        } else {
            LOG_ERR("Failed to generate user UUID: %d", rc);
        }
    }

    if (is_uuid_empty(&app_settings.device_uuid)) {
        LOG_INF("Generating new device UUID");
        rc = app_settings_generate_uuid(&app_settings.device_uuid);
        if (rc == 0) {
            need_save = true;
        } else {
            LOG_ERR("Failed to generate device UUID: %d", rc);
        }
    }

    /* Save if we generated new UUIDs */
    if (need_save) {
        if (app_settings_set_user_uuid(&app_settings.user_uuid) < 0) {
            LOG_WRN("Failed to persist user UUID");
        }
        if (app_settings_set_device_uuid(&app_settings.device_uuid) < 0) {
            LOG_WRN("Failed to persist device UUID");
        }
    }

    LOG_INF("Settings loaded successfully");
    return 0;
}

/* Getter functions */
const char *app_settings_get_ble_name(void)
{
    return app_settings.ble_name;
}

const struct bt_uuid_128 *app_settings_get_user_uuid(void)
{
    return &app_settings.user_uuid;
}

const struct bt_uuid_128 *app_settings_get_device_uuid(void)
{
    return &app_settings.device_uuid;
}

const char *app_settings_get_log_backend(void)
{
    return app_settings.log_backend;
}

bool app_settings_get_pps_enabled(void)
{
    return app_settings.pps_enabled;
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

int app_settings_set_user_uuid(const struct bt_uuid_128 *uuid)
{
    if (!uuid) {
        return -EINVAL;
    }
    
    memcpy(&app_settings.user_uuid, uuid, sizeof(struct bt_uuid_128));
    return settings_save_one("app/user_uuid", uuid->val, sizeof(uuid->val));
}

int app_settings_set_device_uuid(const struct bt_uuid_128 *uuid)
{
    if (!uuid) {
        return -EINVAL;
    }
    
    memcpy(&app_settings.device_uuid, uuid, sizeof(struct bt_uuid_128));
    return settings_save_one("app/device_uuid", uuid->val, sizeof(uuid->val));
}

int app_settings_set_log_backend(const char *backend)
{
    if (!backend || strlen(backend) >= sizeof(app_settings.log_backend)) {
        return -EINVAL;
    }
    
    strcpy(app_settings.log_backend, backend);
    return settings_save_one("app/log_backend", backend, strlen(backend));
}

/* Test function to demonstrate settings */
void app_settings_test(void)
{
    char uuid_str[BT_UUID_STR_LEN];
    
    LOG_INF("Current settings:");
    LOG_INF("  BLE name: %s", app_settings.ble_name);
    LOG_INF("  Log backend: %s", app_settings.log_backend);
    LOG_INF("  PPS enabled: %s", app_settings.pps_enabled ? "Yes" : "No");
    
    /* Display user UUID */
    bt_uuid_to_str((const struct bt_uuid *)&app_settings.user_uuid, 
                   uuid_str, sizeof(uuid_str));
    LOG_INF("  User UUID: %s", uuid_str);
    
    /* Display device UUID */
    bt_uuid_to_str((const struct bt_uuid *)&app_settings.device_uuid, 
                   uuid_str, sizeof(uuid_str));
    LOG_INF("  Device UUID: %s", uuid_str);
    
    /* Test UUID generation */
    struct bt_uuid_128 test_uuid;
    int rc = app_settings_generate_uuid(&test_uuid);
    if (rc == 0) {
        bt_uuid_to_str((const struct bt_uuid *)&test_uuid, 
                       uuid_str, sizeof(uuid_str));
        LOG_INF("  Generated test UUID: %s", uuid_str);
    } else {
        LOG_ERR("  Failed to generate test UUID: %d", rc);
    }
    
    LOG_INF("Settings test complete");
}