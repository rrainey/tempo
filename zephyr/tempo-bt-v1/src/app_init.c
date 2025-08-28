/*
 * SPDX-License-Identifier: Apache-2.0
 * 
 * Tempo-BT V1 - Application Initialization
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/fs/fs.h>
#include <zephyr/fs/littlefs.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>

LOG_MODULE_REGISTER(app_init, LOG_LEVEL_INF);

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