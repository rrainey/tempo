/*
 * SPDX-License-Identifier: Apache-2.0
 * 
 * Tempo-BT V1 - Storage Service Implementation
 * 
 * Manages storage backends and provides unified interface
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <string.h>
#include <stdio.h>

#include "services/storage.h"

LOG_MODULE_REGISTER(storage, LOG_LEVEL_INF);

/* Current storage state */
static struct {
    storage_backend_t backend;
    const storage_interface_t *interface;
    void *backend_ctx;
    bool initialized;
    struct k_mutex lock;
} storage_state = {
    .backend = STORAGE_BACKEND_LITTLEFS,
    .interface = NULL,
    .backend_ctx = NULL,
    .initialized = false,
};

/* Initialize mutex at runtime */
static int storage_init_mutex(void)
{
    k_mutex_init(&storage_state.lock);
    return 0;
}

SYS_INIT(storage_init_mutex, POST_KERNEL, CONFIG_KERNEL_INIT_PRIORITY_DEFAULT);

/* Public API */
int storage_init(storage_backend_t backend)
{
    int ret;

    k_mutex_lock(&storage_state.lock, K_FOREVER);

    if (storage_state.initialized) {
        LOG_WRN("Storage already initialized");
        k_mutex_unlock(&storage_state.lock);
        return -EALREADY;
    }

    if (backend >= STORAGE_BACKEND_COUNT) {
        LOG_ERR("Invalid backend type: %d", backend);
        k_mutex_unlock(&storage_state.lock);
        return -EINVAL;
    }

    LOG_INF("Initializing storage backend: %s",
            backend == STORAGE_BACKEND_LITTLEFS ? "littlefs" : "FAT");

    /* Initialize the selected backend */
    switch (backend) {
    case STORAGE_BACKEND_LITTLEFS:
        ret = storage_littlefs_init();
        if (ret == 0) {
            storage_state.interface = storage_littlefs_get_interface();
        }
        break;

    case STORAGE_BACKEND_FATFS:
        ret = storage_fatfs_init();
        if (ret == 0) {
            storage_state.interface = storage_fatfs_get_interface();
        }
        break;

    default:
        ret = -EINVAL;
        break;
    }

    if (ret != 0) {
        LOG_ERR("Failed to initialize backend: %d", ret);
        k_mutex_unlock(&storage_state.lock);
        return ret;
    }

    storage_state.backend = backend;
    storage_state.initialized = true;

    k_mutex_unlock(&storage_state.lock);

    LOG_INF("Storage initialized successfully");
    return 0;
}

int storage_deinit(void)
{
    int ret = 0;

    k_mutex_lock(&storage_state.lock, K_FOREVER);

    if (!storage_state.initialized) {
        k_mutex_unlock(&storage_state.lock);
        return -EINVAL;
    }

    LOG_INF("Deinitializing storage");

    /* Deinitialize the current backend */
    switch (storage_state.backend) {
    case STORAGE_BACKEND_LITTLEFS:
        ret = storage_littlefs_deinit();
        break;

    case STORAGE_BACKEND_FATFS:
        ret = storage_fatfs_deinit();
        break;

    default:
        break;
    }

    storage_state.initialized = false;
    storage_state.interface = NULL;

    k_mutex_unlock(&storage_state.lock);

    return ret;
}

storage_backend_t storage_get_backend(void)
{
    return storage_state.backend;
}

int storage_open(const char *path, storage_file_t *file)
{
    int ret;

    if (!path || !file) {
        return -EINVAL;
    }

    k_mutex_lock(&storage_state.lock, K_FOREVER);

    if (!storage_state.initialized || !storage_state.interface) {
        k_mutex_unlock(&storage_state.lock);
        return -EINVAL;
    }

    memset(file, 0, sizeof(*file));
    ret = storage_state.interface->open(storage_state.backend_ctx, path, file);

    k_mutex_unlock(&storage_state.lock);

    return ret;
}

int storage_write(storage_file_t *file, const void *data, size_t len)
{
    int ret;

    if (!file || !data || len == 0) {
        return -EINVAL;
    }

    k_mutex_lock(&storage_state.lock, K_FOREVER);

    if (!storage_state.initialized || !storage_state.interface) {
        k_mutex_unlock(&storage_state.lock);
        return -EINVAL;
    }

    ret = storage_state.interface->write(storage_state.backend_ctx, file, data, len);

    k_mutex_unlock(&storage_state.lock);

    return ret;
}

int storage_sync(storage_file_t *file)
{
    int ret;

    if (!file) {
        return -EINVAL;
    }

    k_mutex_lock(&storage_state.lock, K_FOREVER);

    if (!storage_state.initialized || !storage_state.interface) {
        k_mutex_unlock(&storage_state.lock);
        return -EINVAL;
    }

    ret = storage_state.interface->sync(storage_state.backend_ctx, file);

    k_mutex_unlock(&storage_state.lock);

    return ret;
}

int storage_close(storage_file_t *file)
{
    int ret;

    if (!file) {
        return -EINVAL;
    }

    k_mutex_lock(&storage_state.lock, K_FOREVER);

    if (!storage_state.initialized || !storage_state.interface) {
        k_mutex_unlock(&storage_state.lock);
        return -EINVAL;
    }

    ret = storage_state.interface->close(storage_state.backend_ctx, file);

    k_mutex_unlock(&storage_state.lock);

    return ret;
}

int storage_get_free_space(uint64_t *free_bytes, uint64_t *total_bytes)
{
    int ret;

    k_mutex_lock(&storage_state.lock, K_FOREVER);

    if (!storage_state.initialized || !storage_state.interface) {
        k_mutex_unlock(&storage_state.lock);
        return -EINVAL;
    }

    ret = storage_state.interface->get_free_space(storage_state.backend_ctx, 
                                                  free_bytes, total_bytes);

    k_mutex_unlock(&storage_state.lock);

    return ret;
}

int storage_delete(const char *path)
{
    int ret;

    if (!path) {
        return -EINVAL;
    }

    k_mutex_lock(&storage_state.lock, K_FOREVER);

    if (!storage_state.initialized || !storage_state.interface) {
        k_mutex_unlock(&storage_state.lock);
        return -EINVAL;
    }

    ret = storage_state.interface->delete(storage_state.backend_ctx, path);

    k_mutex_unlock(&storage_state.lock);

    return ret;
}

int storage_exists(const char *path)
{
    int ret;

    if (!path) {
        return -EINVAL;
    }

    k_mutex_lock(&storage_state.lock, K_FOREVER);

    if (!storage_state.initialized || !storage_state.interface) {
        k_mutex_unlock(&storage_state.lock);
        return -EINVAL;
    }

    ret = storage_state.interface->exists(storage_state.backend_ctx, path);

    k_mutex_unlock(&storage_state.lock);

    return ret;
}

int storage_list_dir(const char *path, storage_list_cb_t cb, void *ctx)
{
    int ret;

    if (!path || !cb) {
        return -EINVAL;
    }

    k_mutex_lock(&storage_state.lock, K_FOREVER);

    if (!storage_state.initialized || !storage_state.interface) {
        k_mutex_unlock(&storage_state.lock);
        return -EINVAL;
    }

    ret = storage_state.interface->list_dir(storage_state.backend_ctx, path, cb, ctx);

    k_mutex_unlock(&storage_state.lock);

    return ret;
}

int storage_read_file(const char *path, void *buffer, size_t max_len, size_t *bytes_read)
{
    storage_file_t file;
    ssize_t read_result;
    int ret;

    if (!path || !buffer || max_len == 0) {
        return -EINVAL;
    }

    /* For reading, we need to use fs_open directly with read flags */
    k_mutex_lock(&storage_state.lock, K_FOREVER);

    if (!storage_state.initialized) {
        k_mutex_unlock(&storage_state.lock);
        return -EINVAL;
    }

    k_mutex_unlock(&storage_state.lock);

    /* Initialize file structure */
    fs_file_t_init(&file.zfp);

    /* Translate path based on backend */
    char full_path[256];
    if (storage_state.backend == STORAGE_BACKEND_FATFS) {
        if (strncmp(path, "/lfs/", 5) == 0) {
            snprintf(full_path, sizeof(full_path), "/SD:%s", path + 4);
        } else {
            snprintf(full_path, sizeof(full_path), "/SD:%s", path);
        }
    } else {
        strncpy(full_path, path, sizeof(full_path) - 1);
        full_path[sizeof(full_path) - 1] = '\0';
    }

    /* Open for reading */
    ret = fs_open(&file.zfp, full_path, FS_O_READ);
    if (ret != 0) {
        LOG_ERR("Failed to open %s for reading: %d", full_path, ret);
        return ret;
    }

    /* Read data */
    read_result = fs_read(&file.zfp, buffer, max_len);
    if (read_result < 0) {
        LOG_ERR("Failed to read from %s: %zd", full_path, read_result);
        fs_close(&file.zfp);
        return (int)read_result;
    }

    if (bytes_read) {
        *bytes_read = (size_t)read_result;
    }

    /* Close file */
    ret = fs_close(&file.zfp);
    if (ret != 0) {
        LOG_ERR("Failed to close file: %d", ret);
    }

    return 0;
}

int storage_write_file(const char *path, const void *data, size_t len)
{
    storage_file_t file;
    //ssize_t written;
    int ret;

    if (!path || !data || len == 0) {
        return -EINVAL;
    }

    /* Delete existing file first */
    storage_delete(path);

    /* Open file for writing */
    ret = storage_open(path, &file);
    if (ret != 0) {
        LOG_ERR("Failed to open %s for writing: %d", path, ret);
        return ret;
    }

    /* Write all data */
    ret = storage_write(&file, data, len);
    if (ret != 0) {
        LOG_ERR("Failed to write to %s: %d", path, ret);
        storage_close(&file);
        return ret;
    }

    /* Sync and close */
    storage_sync(&file);
    ret = storage_close(&file);
    if (ret != 0) {
        LOG_ERR("Failed to close file: %d", ret);
    }

    return ret;
}

/* Utility function to detect and auto-select backend */
int storage_auto_init(void)
{
    /* Try SD card first if present */
    if (storage_fatfs_card_present()) {
        LOG_INF("SD card detected, using FAT filesystem");
        int ret = storage_init(STORAGE_BACKEND_FATFS);
        if (ret == 0) {
            return 0;
        }
        LOG_WRN("Failed to init FAT, falling back to littlefs");
    }

    /* Fall back to littlefs */
    LOG_INF("Using internal littlefs storage");
    return storage_init(STORAGE_BACKEND_LITTLEFS);
}