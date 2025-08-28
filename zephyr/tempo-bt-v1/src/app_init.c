/*
 * SPDX-License-Identifier: Apache-2.0
 * 
 * Tempo-BT V1 - Application Initialization
 */

#define PM_lfs_storage_ID PM_littlefs_storage_ID

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/fs/fs.h>
#include <zephyr/fs/littlefs.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/dfu/mcuboot.h>
#include <zephyr/mgmt/mcumgr/mgmt/mgmt.h>
#include <zephyr/mgmt/mcumgr/mgmt/callbacks.h>
//#include <zephyr/mgmt/mcumgr/grp/img_mgmt/img_mgmt.h>

LOG_MODULE_REGISTER(app_init, LOG_LEVEL_INF);

#include "ble_mcumgr.h"
#include "services/logger.h"

extern void tempo_mgmt_register(void);

/* DFU safety callback */
static enum mgmt_cb_return dfu_mgmt_action(uint32_t event, enum mgmt_cb_return prev_status,
				       int32_t *rc, uint16_t *group, bool *abort_more, void *data,
				       size_t data_size)
{
    if (event == MGMT_EVT_OP_IMG_MGMT_DFU_CHUNK) {
        /* Check if logging is active */
        logger_state_t state = logger_get_state();
        if (state == LOGGER_STATE_LOGGING) {
            LOG_WRN("DFU blocked: logging in progress");
            *abort_more = true;
            return MGMT_CB_ERROR_RC;
        }
    }
    
    return MGMT_CB_OK;
}

static struct mgmt_callback dfu_mgmt_cb = {
    .callback = dfu_mgmt_action,
    .event_id = MGMT_EVT_OP_IMG_MGMT_DFU_CHUNK,
};

/* Register DFU safety callback */
void dfu_safety_init(void)
{
    mgmt_callback_register(&dfu_mgmt_cb);
    LOG_INF("DFU safety callback registered");
}

/* Get the partition ID directly */
#define STORAGE_PARTITION_ID FIXED_PARTITION_ID(lfs_storage)

/* LittleFS work buffer and cache configuration */
FS_LITTLEFS_DECLARE_DEFAULT_CONFIG(littlefs_storage);

/* Mount point - renamed to avoid conflict with lfs_mount function */
static struct fs_mount_t storage_mount_point = {
    .type = FS_LITTLEFS,
    .fs_data = &littlefs_storage,
    .storage_dev = (void *)STORAGE_PARTITION_ID,
    .mnt_point = "/lfs",
};

int app_storage_init(void)
{
    int ret;
    struct fs_statvfs stats;

    /* Mount the filesystem */
    ret = fs_mount(&storage_mount_point);
    if (ret < 0) {
        LOG_ERR("Error mounting littlefs: %d", ret);
        return ret;
    }
    
    LOG_INF("LittleFS mounted at %s", storage_mount_point.mnt_point);

    /* Get filesystem statistics */
    ret = fs_statvfs(storage_mount_point.mnt_point, &stats);
    if (ret < 0) {
        LOG_ERR("Error getting filesystem stats: %d", ret);
        return ret;
    }

    LOG_INF("FS: %lu blocks of %lu bytes", 
            stats.f_blocks, stats.f_frsize);
    LOG_INF("FS: %lu free blocks (%lu%% free)",
            stats.f_bfree, 
            (stats.f_bfree * 100) / stats.f_blocks);

    /* Create logs directory if it doesn't exist */
    ret = fs_mkdir("/lfs/logs");
    if (ret < 0 && ret != -EEXIST) {
        LOG_ERR("Error creating logs directory: %d", ret);
        return ret;
    } else if (ret == -EEXIST) {
        LOG_INF("Logs directory already exists");
    } else {
        LOG_INF("Created logs directory");
    }

    return 0;
}

int app_init(void)
{
    int ret;

    LOG_INF("Initializing Tempo-BT application");

    /* Initialize BLE and mcumgr */
    ret = ble_mcumgr_init();
    if (ret != 0) {
        LOG_ERR("Failed to initialize BLE/mcumgr: %d", ret);
        return ret;
    }

    /* Register custom mcumgr handlers */
    tempo_mgmt_register();

    /* Initialize DFU safety */
    dfu_safety_init();

    return 0;
}