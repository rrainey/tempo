/*
 * SPDX-License-Identifier: Apache-2.0
 * 
 * Tempo-BT V1 - BLE and mcumgr Initialization
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
//#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>
#include <zephyr/mgmt/mcumgr/mgmt/mgmt.h>
#include <zephyr/mgmt/mcumgr/mgmt/callbacks.h>
#include <zephyr/mgmt/mcumgr/transport/smp_bt.h>
#include <zephyr/mgmt/mcumgr/grp/fs_mgmt/fs_mgmt.h>
//#include <zephyr/mgmt/mcumgr/grp/img_mgmt/img_mgmt.h>
#include <zephyr/mgmt/mcumgr/grp/os_mgmt/os_mgmt.h>
#include <zephyr/mgmt/mcumgr/grp/stat_mgmt/stat_mgmt.h>
#include <zephyr/mgmt/mcumgr/grp/shell_mgmt/shell_mgmt.h>

#include "app/app_state.h"
#include "app/events.h"

LOG_MODULE_REGISTER(ble_mcumgr, LOG_LEVEL_INF);

/* BLE advertising data */
static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA_BYTES(BT_DATA_UUID128_ALL,
                  0x84, 0xaa, 0x60, 0x74, 0x52, 0x8a, 0x8b, 0x86,
                  0xd3, 0x4c, 0xb7, 0x1d, 0x1d, 0xdc, 0x53, 0x8d), /* SMP/mcumgr UUID */
};

/* Scan response data */
static const struct bt_data sd[] = {
    BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME, sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

/* Connection reference */
static struct bt_conn *current_conn;

/* Connection callbacks */
static void connected(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        LOG_ERR("Connection failed (err 0x%02x)", err);
        return;
    }

    current_conn = bt_conn_ref(conn);
    
    LOG_INF("Connected");

    /* Update connection parameters for better throughput */
    struct bt_le_conn_param param = {
        .interval_min = 6,    /* 7.5 ms */
        .interval_max = 12,   /* 15 ms */
        .latency = 0,
        .timeout = 400,       /* 4 s */
    };

    int ret = bt_conn_le_param_update(conn, &param);
    if (ret) {
        LOG_WRN("Failed to request connection parameter update: %d", ret);
    }

    /* Emit connection event */
    app_event_t evt = {
        .type = EVT_BLE_CONNECTED
    };
    event_bus_publish(&evt);
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    ARG_UNUSED(conn);

    LOG_INF("Disconnected (reason 0x%02x)", reason);

    /* Stop advertising first */
    bt_le_adv_stop();

    if (current_conn) {
        bt_conn_unref(current_conn);
        current_conn = NULL;
    }

    /* Emit disconnection event */
    app_event_t evt = {
        .type = EVT_BLE_DISCONNECTED
    };
    event_bus_publish(&evt);

    /* Restart advertising */
    int ret = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad),
                               sd, ARRAY_SIZE(sd));
    if (ret) {
        LOG_ERR("Failed to restart advertising: %d", ret);
    }
}


static void param_updated(struct bt_conn *conn, uint16_t interval,
                          uint16_t latency, uint16_t timeout)
{
    ARG_UNUSED(conn);
    
    LOG_INF("Connection parameters updated: interval=%d, latency=%d, timeout=%d",
            interval, latency, timeout);
}

static struct bt_conn_cb conn_callbacks = {
    .connected = connected,
    .disconnected = disconnected,
    .le_param_updated = param_updated,
};

/* File system access hooks for mcumgr */
static enum mgmt_cb_return fs_mgmt_upload_action(uint32_t event, enum mgmt_cb_return prev_status,
				       int32_t *rc, uint16_t *group, bool *abort_more, void *data,
				       size_t data_size)
{
    const char *path;
    
    if (event == MGMT_EVT_OP_FS_MGMT_FILE_ACCESS) {
        struct fs_mgmt_file_access *access = (struct fs_mgmt_file_access *)data;
        path = access->filename;
        
        /* Check write permissions */
        if (access->access == FS_MGMT_FILE_ACCESS_WRITE) {
            /* Allow write access to upload directory */
            if (strstr(path, "/lfs/upload/") == path) {
                return MGMT_CB_OK;
            }
            
            /* Deny all other writes */
            *abort_more = true;
            return MGMT_CB_ERROR_RC;
        } else {
            /* Read access checks */
            /* Deny access to system files */
            if (strstr(path, "/lfs/system/") == path) {
                *abort_more = true;
                return MGMT_CB_ERROR_RC;
            }
            
            /* Allow read access to everything else */
            return MGMT_CB_OK;
        }
    }
    
    return MGMT_CB_OK;
}

static struct mgmt_callback fs_mgmt_cb = {
    .callback = fs_mgmt_upload_action,
    .event_id = MGMT_EVT_OP_FS_MGMT_FILE_ACCESS,
};

/* Initialize BLE and mcumgr */
int ble_mcumgr_init(void)
{
    int ret;

    LOG_INF("Initializing BLE and mcumgr");

    /* Initialize Bluetooth */
    ret = bt_enable(NULL);
    if (ret) {
        LOG_ERR("Failed to enable Bluetooth: %d", ret);
        return ret;
    }

    LOG_INF("Bluetooth initialized");

    /* Register connection callbacks */
    bt_conn_cb_register(&conn_callbacks);

    /* Initialize mcumgr transports - no need to call smp_bt_register() in newer versions */
    /* It's done automatically when CONFIG_MCUMGR_TRANSPORT_BT is enabled */

    /* Register mcumgr command groups - these are now done automatically via Kconfig */
    /* The groups are registered when their respective CONFIG options are enabled:
     * - CONFIG_MCUMGR_GRP_FS for filesystem
     * - CONFIG_MCUMGR_GRP_IMG for image management
     * - CONFIG_MCUMGR_GRP_OS for OS management
     * - CONFIG_MCUMGR_GRP_STAT for statistics
     * - CONFIG_MCUMGR_GRP_SHELL for shell
     */

    /* Register file system access callback */
    mgmt_callback_register(&fs_mgmt_cb);

    /* Start advertising */
    ret = bt_le_adv_start(BT_LE_ADV_CONN_FAST_1, ad, ARRAY_SIZE(ad),
                          sd, ARRAY_SIZE(sd));
    if (ret) {
        LOG_ERR("Failed to start advertising: %d", ret);
        return ret;
    }

    LOG_INF("BLE advertising started");

    return 0;
}

/* Get current connection status */
bool ble_is_connected(void)
{
    return current_conn != NULL;
}

/* Get connection info */
int ble_get_conn_info(struct bt_conn_info *info)
{
    if (!current_conn || !info) {
        return -EINVAL;
    }

    return bt_conn_get_info(current_conn, info);
}