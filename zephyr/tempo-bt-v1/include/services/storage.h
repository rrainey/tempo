/*
 * SPDX-License-Identifier: Apache-2.0
 * 
 * Tempo-BT V1 - Storage Service Interface
 * 
 * Provides abstraction layer for different storage backends (littlefs, FAT)
 */

#ifndef SERVICES_STORAGE_H
#define SERVICES_STORAGE_H

#include <stdint.h>
#include <stdbool.h>
#include <zephyr/fs/fs.h>

/* Storage file handle */
typedef struct {
    struct fs_file_t zfp;      /* Zephyr file pointer */
    bool is_open;              /* File open flag */
    void *backend_data;        /* Backend-specific data */
} storage_file_t;

/* Directory listing callback
 * Return 0 to continue, non-zero to stop listing */
typedef int (*storage_list_cb_t)(const char *path, bool is_dir, size_t size, void *ctx);

/* Storage interface - implemented by each backend */
typedef struct {
    /* File operations */
    int (*open)(void *ctx, const char *path, storage_file_t *file);
    int (*write)(void *ctx, storage_file_t *file, const void *data, size_t len);
    int (*sync)(void *ctx, storage_file_t *file);
    int (*close)(void *ctx, storage_file_t *file);
    
    /* Filesystem operations */
    int (*get_free_space)(void *ctx, uint64_t *free_bytes, uint64_t *total_bytes);
    int (*delete)(void *ctx, const char *path);
    int (*exists)(void *ctx, const char *path);  /* Returns 1 if exists, 0 if not */
    int (*list_dir)(void *ctx, const char *path, storage_list_cb_t cb, void *cb_ctx);
} storage_interface_t;

/* Storage backend types */
typedef enum {
    STORAGE_BACKEND_LITTLEFS,
    STORAGE_BACKEND_FATFS,
    STORAGE_BACKEND_COUNT
} storage_backend_t;

/* Global storage API */

/**
 * @brief Initialize storage subsystem
 * 
 * @param backend Backend type to use
 * @return 0 on success, negative error code on failure
 */
int storage_init(storage_backend_t backend);

/**
 * @brief Deinitialize storage subsystem
 * 
 * @return 0 on success, negative error code on failure
 */
int storage_deinit(void);

/**
 * @brief Get current storage backend type
 * 
 * @return Current backend type
 */
storage_backend_t storage_get_backend(void);

/**
 * @brief Open file for writing/appending
 * 
 * @param path File path (relative to storage root)
 * @param file File handle to initialize
 * @return 0 on success, negative error code on failure
 */
int storage_open(const char *path, storage_file_t *file);

/**
 * @brief Write data to file
 * 
 * @param file File handle
 * @param data Data to write
 * @param len Length of data
 * @return 0 on success, negative error code on failure
 */
int storage_write(storage_file_t *file, const void *data, size_t len);

/**
 * @brief Sync file to storage
 * 
 * @param file File handle
 * @return 0 on success, negative error code on failure
 */
int storage_sync(storage_file_t *file);

/**
 * @brief Close file
 * 
 * @param file File handle
 * @return 0 on success, negative error code on failure
 */
int storage_close(storage_file_t *file);

/**
 * @brief Get storage free space
 * 
 * @param free_bytes Output: Free space in bytes
 * @param total_bytes Output: Total space in bytes
 * @return 0 on success, negative error code on failure
 */
int storage_get_free_space(uint64_t *free_bytes, uint64_t *total_bytes);

/**
 * @brief Delete file or directory
 * 
 * @param path Path to delete
 * @return 0 on success, negative error code on failure
 */
int storage_delete(const char *path);

/**
 * @brief Check if file/directory exists
 * 
 * @param path Path to check
 * @return 1 if exists, 0 if not, negative on error
 */
int storage_exists(const char *path);

/**
 * @brief List directory contents
 * 
 * @param path Directory path
 * @param cb Callback function for each entry
 * @param ctx Context passed to callback
 * @return 0 on success, negative error code on failure
 */
int storage_list_dir(const char *path, storage_list_cb_t cb, void *ctx);

/**
 * @brief Read file contents (for config files)
 * 
 * @param path File path
 * @param buffer Buffer to read into
 * @param max_len Maximum bytes to read
 * @param bytes_read Output: Actual bytes read
 * @return 0 on success, negative error code on failure
 */
int storage_read_file(const char *path, void *buffer, size_t max_len, size_t *bytes_read);

/**
 * @brief Write complete file (for config files)
 * 
 * @param path File path
 * @param data Data to write
 * @param len Length of data
 * @return 0 on success, negative error code on failure
 */
int storage_write_file(const char *path, const void *data, size_t len);

/* Backend-specific initialization functions */
int storage_littlefs_init(void);
int storage_littlefs_deinit(void);
const storage_interface_t *storage_littlefs_get_interface(void);

int storage_fatfs_init(void);
int storage_fatfs_deinit(void);
const storage_interface_t *storage_fatfs_get_interface(void);
bool storage_fatfs_card_present(void);

/**
 * @brief Translate logical path to physical path based on backend
 * 
 * @param logical_path Logical path (e.g., "/logs/file.csv")
 * @param physical_path Buffer for physical path
 * @param path_size Size of physical_path buffer
 * @return 0 on success, negative error code on failure
 */
int storage_translate_path(const char *logical_path, char *physical_path, size_t path_size);


#endif /* SERVICES_STORAGE_H */