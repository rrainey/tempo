/*
 * SPDX-License-Identifier: Apache-2.0
 * 
 * Tempo-BT V1 - LittleFS Storage Backend
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/fs/fs.h>
#include <zephyr/fs/littlefs.h>
#include <zephyr/storage/flash_map.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <string.h>
#include <stdio.h>

#include "services/storage.h"

LOG_MODULE_REGISTER(storage_littlefs, LOG_LEVEL_INF);

/* LittleFS mount point */
#define LFS_MOUNT_POINT "/lfs"

/* Use external flash partition for LittleFS */
#define LFS_PARTITION_ID    FIXED_PARTITION_ID(external_flash)
#define LFS_PARTITION_NODE  FIXED_PARTITION_NODE(external_flash)

/* LittleFS configuration for external flash */
static struct fs_littlefs lfs_storage;
static struct fs_mount_t lfs_mount_spec = {
    .type = FS_LITTLEFS,
    .fs_data = &lfs_storage,
    .storage_dev = (void *)LFS_PARTITION_ID,
    .mnt_point = LFS_MOUNT_POINT,
};

/* Storage state */
static struct {
    bool mounted;
} storage_state = {
    .mounted = false
};

/* Forward declarations */
static int ensure_mounted(void);
static int create_directory_path(const char *path);

/* ILogSink implementation for LittleFS */
static int lfs_open(void *ctx, const char *path, storage_file_t *file);
static int lfs_write(void *ctx, storage_file_t *file, const void *data, size_t len);
static int lfs_sync(void *ctx, storage_file_t *file);
static int lfs_close(void *ctx, storage_file_t *file);
static int lfs_get_free_space(void *ctx, uint64_t *free_bytes, uint64_t *total_bytes);
static int lfs_delete(void *ctx, const char *path);
static int lfs_exists(void *ctx, const char *path);
static int lfs_list_dir(void *ctx, const char *path, storage_list_cb_t cb, void *cb_ctx);

/* Storage interface */
static const storage_interface_t lfs_interface = {
    .open = lfs_open,
    .write = lfs_write,
    .sync = lfs_sync,
    .close = lfs_close,
    .get_free_space = lfs_get_free_space,
    .delete = lfs_delete,
    .exists = lfs_exists,
    .list_dir = lfs_list_dir,
};

/* Public API */
int storage_littlefs_init(void)
{
    int ret;
    struct fs_dirent entry;
    const struct flash_area *fa;

    LOG_INF("Initializing LittleFS storage backend on external flash");

    /* Get flash area info */
    ret = flash_area_open(LFS_PARTITION_ID, &fa);
    if (ret != 0) {
        LOG_ERR("Failed to open flash area: %d", ret);
        return ret;
    }
    
    LOG_INF("External flash partition: offset 0x%x, size 0x%x", 
            (unsigned int)fa->fa_off, (unsigned int)fa->fa_size);
    flash_area_close(fa);

    /* Check if already mounted */
    ret = fs_stat(LFS_MOUNT_POINT, &entry);
    if (ret == 0) {
        LOG_INF("LittleFS already mounted");
        storage_state.mounted = true;
        return 0;
    }

    /* Mount filesystem */
    ret = fs_mount(&lfs_mount_spec);
    if (ret == 0) {
        /* Mount succeeded */
        storage_state.mounted = true;
        LOG_INF("LittleFS mounted at %s on external flash partition", LFS_MOUNT_POINT);

        /* Create base directories */
        char path[256];
        snprintf(path, sizeof(path), "%s/logs", LFS_MOUNT_POINT);
        fs_mkdir(path);  /* Ignore error if already exists */
        
        return 0;
    } else {
        /* Mount failed - this is normal on first boot with unformatted flash */
        LOG_WRN("LittleFS mount failed with %d - this is normal on first boot", ret);
        LOG_INF("The filesystem will be automatically formatted on first write");
        /* Note: LittleFS in Zephyr often auto-formats on first file operation */
        /* We return success here to allow the system to continue */
        /* The first write operation will trigger the format */
        return 0;
    }
}

int storage_littlefs_deinit(void)
{
    int ret = 0;

    if (storage_state.mounted) {
        ret = fs_unmount(&lfs_mount_spec);
        if (ret == 0) {
            storage_state.mounted = false;
            LOG_INF("LittleFS unmounted");
        } else {
            LOG_ERR("Failed to unmount: %d", ret);
        }
    }

    return ret;
}

const storage_interface_t *storage_littlefs_get_interface(void)
{
    return &lfs_interface;
}

/* Internal functions */
static int ensure_mounted(void)
{
    if (storage_state.mounted) {
        return 0;
    }

    return storage_littlefs_init();
}

static int create_directory_path(const char *full_path)
{
    char path[256];
    char *p;
    int ret;

    /* Make a working copy */
    strncpy(path, full_path, sizeof(path) - 1);
    path[sizeof(path) - 1] = '\0';

    /* Find the last slash to separate directory from filename */
    p = strrchr(path, '/');
    if (p == NULL) {
        return 0;  /* No directory part */
    }

    /* Terminate at the last directory */
    *p = '\0';

    /* Create each directory in the path */
    p = path;
    while ((p = strchr(p + 1, '/')) != NULL) {
        *p = '\0';
        
        struct fs_dirent entry;
        ret = fs_stat(path, &entry);
        if (ret != 0) {
            /* Directory doesn't exist, create it */
            ret = fs_mkdir(path);
            if (ret != 0 && ret != -EEXIST) {
                LOG_ERR("Failed to create directory %s: %d", path, ret);
                return ret;
            }
        }
        
        *p = '/';
    }

    /* Create the final directory */
    struct fs_dirent entry;
    ret = fs_stat(path, &entry);
    if (ret != 0) {
        ret = fs_mkdir(path);
        if (ret != 0 && ret != -EEXIST) {
            LOG_ERR("Failed to create directory %s: %d", path, ret);
            return ret;
        }
    }

    return 0;
}

/* ILogSink implementation */
static int lfs_open(void *ctx, const char *path, storage_file_t *file)
{
    int ret;

    ARG_UNUSED(ctx);

    /* First ensure we're mounted */
    if (!storage_state.mounted) {
        /* Try to mount again - this might trigger auto-format */
        ret = fs_mount(&lfs_mount_spec);
        if (ret == 0) {
            storage_state.mounted = true;
            LOG_INF("LittleFS mounted successfully on file open");
        } else {
            LOG_ERR("Failed to mount LittleFS on file open: %d", ret);
            return ret;
        }
    }

    /* Create directory structure if needed */
    ret = create_directory_path(path);
    if (ret != 0) {
        return ret;
    }

    /* Open file for append/create */
    fs_file_t_init(&file->zfp);
    ret = fs_open(&file->zfp, path, FS_O_CREATE | FS_O_APPEND | FS_O_WRITE);
    if (ret != 0) {
        LOG_ERR("Failed to open %s: %d", path, ret);
        return ret;
    }

    file->is_open = true;
    LOG_DBG("Opened file: %s", path);

    return 0;
}

static int lfs_write(void *ctx, storage_file_t *file, const void *data, size_t len)
{
    ssize_t written;

    ARG_UNUSED(ctx);

    if (!file || !file->is_open) {
        return -EINVAL;
    }

    written = fs_write(&file->zfp, data, len);
    if (written < 0) {
        LOG_ERR("Write failed: %zd", written);
        return (int)written;
    }

    if (written != len) {
        LOG_WRN("Partial write: %zd/%zu bytes", written, len);
        return -EIO;
    }

    return 0;
}

static int lfs_sync(void *ctx, storage_file_t *file)
{
    int ret;

    ARG_UNUSED(ctx);

    if (!file || !file->is_open) {
        return -EINVAL;
    }

    ret = fs_sync(&file->zfp);
    if (ret != 0) {
        LOG_ERR("Sync failed: %d", ret);
    }

    return ret;
}

static int lfs_close(void *ctx, storage_file_t *file)
{
    int ret;

    ARG_UNUSED(ctx);

    if (!file || !file->is_open) {
        return -EINVAL;
    }

    ret = fs_close(&file->zfp);
    if (ret != 0) {
        LOG_ERR("Close failed: %d", ret);
    } else {
        file->is_open = false;
        LOG_DBG("File closed");
    }

    return ret;
}

static int lfs_get_free_space(void *ctx, uint64_t *free_bytes, uint64_t *total_bytes)
{
    struct fs_statvfs stats;
    int ret;

    ARG_UNUSED(ctx);

    /* Make sure we're mounted */
    if (!storage_state.mounted) {
        return -EINVAL;
    }

    ret = fs_statvfs(LFS_MOUNT_POINT, &stats);
    if (ret != 0) {
        LOG_ERR("Failed to get filesystem stats: %d", ret);
        return ret;
    }

    if (free_bytes) {
        *free_bytes = (uint64_t)stats.f_bfree * stats.f_frsize;
    }

    if (total_bytes) {
        *total_bytes = (uint64_t)stats.f_blocks * stats.f_frsize;
    }

    return 0;
}

static int lfs_delete(void *ctx, const char *path)
{
    int ret;

    ARG_UNUSED(ctx);

    if (!storage_state.mounted) {
        return -EINVAL;
    }

    ret = fs_unlink(path);
    
    if (ret != 0 && ret != -ENOENT) {
        LOG_ERR("Failed to delete %s: %d", path, ret);
    }

    return ret;
}

static int lfs_exists(void *ctx, const char *path)
{
    struct fs_dirent entry;
    int ret;

    ARG_UNUSED(ctx);

    if (!storage_state.mounted) {
        return 0;
    }

    ret = fs_stat(path, &entry);
    
    return (ret == 0) ? 1 : 0;
}

static int lfs_list_dir(void *ctx, const char *path, storage_list_cb_t cb, void *cb_ctx)
{
    struct fs_dir_t dir;
    struct fs_dirent entry;
    int ret;

    ARG_UNUSED(ctx);

    if (!cb) {
        return -EINVAL;
    }

    if (!storage_state.mounted) {
        return -EINVAL;
    }

    fs_dir_t_init(&dir);

    ret = fs_opendir(&dir, path);
    if (ret != 0) {
        LOG_ERR("Failed to open directory %s: %d", path, ret);
        return ret;
    }

    while (fs_readdir(&dir, &entry) == 0) {
        if (entry.name[0] == '\0') {
            break;  /* End of directory */
        }

        /* Skip . and .. */
        if (strcmp(entry.name, ".") == 0 || strcmp(entry.name, "..") == 0) {
            continue;
        }

        /* Build full path for callback */
        char full_path[256];
        snprintf(full_path, sizeof(full_path), "%s/%s", path, entry.name);

        /* Call the callback */
        ret = cb(full_path, entry.type == FS_DIR_ENTRY_DIR, entry.size, cb_ctx);
        if (ret != 0) {
            break;  /* Callback requested stop */
        }
    }

    fs_closedir(&dir);
    return ret;
}