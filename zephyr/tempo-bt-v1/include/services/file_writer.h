/*
 * SPDX-License-Identifier: Apache-2.0
 * 
 * Tempo-BT V1 - File Writer Service
 * 
 * Asynchronous file writing with buffering and backpressure handling
 */

#ifndef SERVICES_FILE_WRITER_H
#define SERVICES_FILE_WRITER_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/* File writer configuration */
typedef struct {
    size_t buffer_size;         /* Write buffer size (1-4KB recommended) */
    uint32_t flush_interval_ms; /* Time-based flush interval */
    size_t queue_depth;         /* Max pending write requests */
} file_writer_config_t;

/* Write request priority levels */
typedef enum {
    WRITE_PRIORITY_CRITICAL = 0,  /* IMU data - never drop */
    WRITE_PRIORITY_HIGH,          /* Baro data */
    WRITE_PRIORITY_MEDIUM,        /* Mag data */
    WRITE_PRIORITY_LOW,           /* GNSS data - drop first */
    WRITE_PRIORITY_COUNT
} write_priority_t;

/* File writer statistics */
typedef struct {
    uint32_t lines_written;
    uint64_t bytes_written;
    uint32_t flushes;
    uint32_t dropped[WRITE_PRIORITY_COUNT];
    uint32_t write_errors;
    uint32_t queue_highwater;
} file_writer_stats_t;

/**
 * @brief Initialize file writer service
 * 
 * @param config Configuration parameters
 * @return 0 on success, negative error code on failure
 */
int file_writer_init(const file_writer_config_t *config);

/**
 * @brief Deinitialize file writer service
 * 
 * @return 0 on success, negative error code on failure
 */
int file_writer_deinit(void);

/**
 * @brief Open file for writing
 * 
 * @param path File path
 * @return 0 on success, negative error code on failure
 */
int file_writer_open(const char *path);

/**
 * @brief Queue data for writing
 * 
 * @param data Data to write
 * @param len Length of data
 * @param priority Priority level for backpressure handling
 * @return 0 on success, -ENOSPC if queue full, negative error on failure
 */
int file_writer_write(const void *data, size_t len, write_priority_t priority);

/**
 * @brief Force flush of pending writes
 * 
 * @return 0 on success, negative error code on failure
 */
int file_writer_flush(void);

/**
 * @brief Close current file
 * 
 * Flushes all pending data and closes the file
 * 
 * @return 0 on success, negative error code on failure
 */
int file_writer_close(void);

/**
 * @brief Get writer statistics
 * 
 * @param stats Output statistics structure
 */
void file_writer_get_stats(file_writer_stats_t *stats);

/**
 * @brief Get current queue usage
 * 
 * @return Percentage of queue used (0-100)
 */
int file_writer_get_queue_usage(void);

#endif /* SERVICES_FILE_WRITER_H */