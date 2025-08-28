/*
 * SPDX-License-Identifier: Apache-2.0
 * 
 * Tempo-BT V1 - File Writer Service Implementation
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <string.h>

#include "services/file_writer.h"
#include "services/storage.h"

LOG_MODULE_REGISTER(file_writer, LOG_LEVEL_INF);

/* Default configuration */
#define DEFAULT_BUFFER_SIZE     2048
#define DEFAULT_FLUSH_INTERVAL  250
#define DEFAULT_QUEUE_DEPTH     128

/* Write request structure */
typedef struct {
    void *data;
    size_t len;
    write_priority_t priority;
} write_request_t;

/* File writer state */
static struct {
    /* Configuration */
    file_writer_config_t config;
    
    /* File state */
    storage_file_t file;
    bool file_open;
    char current_path[256];
    
    /* Write buffer */
    uint8_t *buffer;
    size_t buffer_used;
    
    /* Message queue for requests */
    struct k_msgq msgq;
    char *msgq_buffer;
    
    /* Thread */
    struct k_thread thread;
    k_thread_stack_t *stack;
    k_tid_t tid;
    volatile bool running;
    
    /* Flush work */
    struct k_work_delayable flush_work;
    
    /* Statistics */
    file_writer_stats_t stats;
    
    /* Synchronization */
    struct k_sem close_sem;
    struct k_mutex stats_mutex;
} writer_state;

/* Memory pool for write data */
//K_MEM_POOL_DEFINE(writer_pool, 64, 512, 16, 4);
K_HEAP_DEFINE(writer_pool, 512);

/* Forward declarations */
static void writer_thread_fn(void *p1, void *p2, void *p3);
static void flush_work_handler(struct k_work *work);
static int do_write(const void *data, size_t len);
static int do_flush(void);

/* Public API */
int file_writer_init(const file_writer_config_t *config)
{
    //int ret;
    
    LOG_INF("Initializing file writer");
    
    /* Initialize state */
    memset(&writer_state, 0, sizeof(writer_state));
    
    /* Use provided config or defaults */
    if (config) {
        writer_state.config = *config;
    } else {
        writer_state.config.buffer_size = DEFAULT_BUFFER_SIZE;
        writer_state.config.flush_interval_ms = DEFAULT_FLUSH_INTERVAL;
        writer_state.config.queue_depth = DEFAULT_QUEUE_DEPTH;
    }
    
    /* Allocate write buffer */
    writer_state.buffer = k_malloc(writer_state.config.buffer_size);
    if (!writer_state.buffer) {
        LOG_ERR("Failed to allocate write buffer");
        return -ENOMEM;
    }
    
    /* Allocate message queue buffer */
    size_t msgq_size = writer_state.config.queue_depth * sizeof(write_request_t);
    writer_state.msgq_buffer = k_malloc(msgq_size);
    if (!writer_state.msgq_buffer) {
        LOG_ERR("Failed to allocate msgq buffer");
        k_free(writer_state.buffer);
        return -ENOMEM;
    }
    
    /* Initialize message queue */
    k_msgq_init(&writer_state.msgq, writer_state.msgq_buffer, 
                sizeof(write_request_t), writer_state.config.queue_depth);
    
    /* Allocate thread stack */
    writer_state.stack = k_thread_stack_alloc(2048, 0);
    if (!writer_state.stack) {
        LOG_ERR("Failed to allocate thread stack");
        k_free(writer_state.buffer);
        k_free(writer_state.msgq_buffer);
        return -ENOMEM;
    }
    
    /* Initialize synchronization */
    k_sem_init(&writer_state.close_sem, 0, 1);
    k_mutex_init(&writer_state.stats_mutex);
    
    /* Initialize flush work */
    k_work_init_delayable(&writer_state.flush_work, flush_work_handler);
    
    /* Start writer thread */
    writer_state.running = true;
    writer_state.tid = k_thread_create(&writer_state.thread,
                                       writer_state.stack, 2048,
                                       writer_thread_fn,
                                       NULL, NULL, NULL,
                                       K_PRIO_PREEMPT(10), 0, K_NO_WAIT);
    k_thread_name_set(writer_state.tid, "file_writer");
    
    LOG_INF("File writer initialized");
    return 0;
}

int file_writer_deinit(void)
{
    LOG_INF("Deinitializing file writer");
    
    /* Stop thread */
    writer_state.running = false;
    
    /* Wake up thread if it's waiting */
    write_request_t dummy = {0};
    k_msgq_put(&writer_state.msgq, &dummy, K_NO_WAIT);
    
    /* Wait for thread to exit */
    if (writer_state.tid) {
        k_thread_join(writer_state.tid, K_FOREVER);
    }
    
    /* Close file if open */
    if (writer_state.file_open) {
        file_writer_close();
    }
    
    /* Cancel flush work */
    k_work_cancel_delayable(&writer_state.flush_work);
    
    /* Free resources */
    if (writer_state.buffer) {
        k_free(writer_state.buffer);
    }
    if (writer_state.msgq_buffer) {
        k_free(writer_state.msgq_buffer);
    }
    if (writer_state.stack) {
        k_thread_stack_free(writer_state.stack);
    }
    
    /* Clean up message queue */
    k_msgq_cleanup(&writer_state.msgq);
    
    LOG_INF("File writer deinitialized");
    return 0;
}

int file_writer_open(const char *path)
{
    int ret;
    
    if (!path) {
        return -EINVAL;
    }
    
    if (writer_state.file_open) {
        LOG_WRN("File already open, closing first");
        file_writer_close();
    }
    
    LOG_INF("Opening file: %s", path);
    
    /* Open file through storage layer */
    ret = storage_open(path, &writer_state.file);
    if (ret != 0) {
        LOG_ERR("Failed to open file: %d", ret);
        return ret;
    }
    
    writer_state.file_open = true;
    strncpy(writer_state.current_path, path, sizeof(writer_state.current_path) - 1);
    
    /* Reset statistics for new file */
    k_mutex_lock(&writer_state.stats_mutex, K_FOREVER);
    memset(&writer_state.stats, 0, sizeof(writer_state.stats));
    k_mutex_unlock(&writer_state.stats_mutex);
    
    /* Schedule first flush */
    k_work_schedule(&writer_state.flush_work, 
                    K_MSEC(writer_state.config.flush_interval_ms));
    
    return 0;
}

int file_writer_write(const void *data, size_t len, write_priority_t priority)
{
    write_request_t req;
    int ret;
    
    if (!data || len == 0) {
        return -EINVAL;
    }
    
    if (!writer_state.file_open) {
        return -EINVAL;
    }
    
    if (priority >= WRITE_PRIORITY_COUNT) {
        priority = WRITE_PRIORITY_LOW;
    }
    
    /* Allocate memory for data copy */
    //req.data = k_mem_pool_malloc(&writer_pool, len);
    req.data = k_heap_alloc(&writer_pool, len, K_FOREVER);
    if (!req.data) {
        /* Handle backpressure - drop based on priority */
        k_mutex_lock(&writer_state.stats_mutex, K_FOREVER);
        writer_state.stats.dropped[priority]++;
        k_mutex_unlock(&writer_state.stats_mutex);
        
        LOG_WRN("Dropped write request, priority=%d, len=%zu", priority, len);
        return -ENOSPC;
    }
    
    /* Copy data */
    memcpy(req.data, data, len);
    req.len = len;
    req.priority = priority;
    
    /* Try to queue request */
    ret = k_msgq_put(&writer_state.msgq, &req, K_NO_WAIT);
    if (ret != 0) {
        /* Queue full - apply backpressure policy */
        k_free(req.data);
        
        if (priority == WRITE_PRIORITY_CRITICAL) {
            /* Critical data - try to make room by dropping lower priority */
            write_request_t old_req;
            if (k_msgq_get(&writer_state.msgq, &old_req, K_NO_WAIT) == 0) {
                k_free(old_req.data);
                k_mutex_lock(&writer_state.stats_mutex, K_FOREVER);
                writer_state.stats.dropped[old_req.priority]++;
                k_mutex_unlock(&writer_state.stats_mutex);
                
                /* Retry */
                ret = k_msgq_put(&writer_state.msgq, &req, K_NO_WAIT);
                if (ret == 0) {
                    return 0;
                }
            }
        }
        
        k_mutex_lock(&writer_state.stats_mutex, K_FOREVER);
        writer_state.stats.dropped[priority]++;
        k_mutex_unlock(&writer_state.stats_mutex);
        
        return -ENOSPC;
    }
    
    /* Update queue highwater mark */
    uint32_t used = k_msgq_num_used_get(&writer_state.msgq);
    k_mutex_lock(&writer_state.stats_mutex, K_FOREVER);
    if (used > writer_state.stats.queue_highwater) {
        writer_state.stats.queue_highwater = used;
    }
    k_mutex_unlock(&writer_state.stats_mutex);
    
    return 0;
}

int file_writer_flush(void)
{
    if (!writer_state.file_open) {
        return -EINVAL;
    }
    
    /* Cancel scheduled flush and do it now */
    k_work_cancel_delayable(&writer_state.flush_work);
    
    return do_flush();
}

int file_writer_close(void)
{
    int ret;
    
    if (!writer_state.file_open) {
        return -EINVAL;
    }
    
    LOG_INF("Closing file: %s", writer_state.current_path);
    
    /* Cancel flush work */
    k_work_cancel_delayable(&writer_state.flush_work);
    
    /* Signal close to writer thread */
    writer_state.file_open = false;
    
    /* Wait for writer thread to process remaining queue */
    k_sem_take(&writer_state.close_sem, K_SECONDS(5));
    
    /* Final flush */
    do_flush();
    
    /* Close file */
    ret = storage_close(&writer_state.file);
    if (ret != 0) {
        LOG_ERR("Failed to close file: %d", ret);
    }
    
    /* Log final statistics */
    LOG_INF("File closed: %llu bytes, %u lines, %u flushes",
            writer_state.stats.bytes_written,
            writer_state.stats.lines_written,
            writer_state.stats.flushes);
    
    if (writer_state.stats.write_errors > 0) {
        LOG_WRN("Write errors: %u", writer_state.stats.write_errors);
    }
    
    for (int i = 0; i < WRITE_PRIORITY_COUNT; i++) {
        if (writer_state.stats.dropped[i] > 0) {
            LOG_WRN("Dropped priority %d: %u", i, writer_state.stats.dropped[i]);
        }
    }
    
    return ret;
}

void file_writer_get_stats(file_writer_stats_t *stats)
{
    if (!stats) {
        return;
    }
    
    k_mutex_lock(&writer_state.stats_mutex, K_FOREVER);
    *stats = writer_state.stats;
    k_mutex_unlock(&writer_state.stats_mutex);
}

int file_writer_get_queue_usage(void)
{
    uint32_t used = k_msgq_num_used_get(&writer_state.msgq);
    uint32_t total = writer_state.config.queue_depth;
    
    return (used * 100) / total;
}

/* Internal functions */
static void writer_thread_fn(void *p1, void *p2, void *p3)
{
    write_request_t req;
    int ret;
    
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);
    
    LOG_INF("File writer thread started");
    
    while (writer_state.running) {
        /* Wait for write request */
        ret = k_msgq_get(&writer_state.msgq, &req, K_FOREVER);
        if (ret != 0) {
            continue;
        }
        
        /* Check if this is a dummy request (for shutdown) */
        if (!req.data) {
            continue;
        }
        
        /* Process write request if file is open */
        if (writer_state.file_open) {
            ret = do_write(req.data, req.len);
            if (ret != 0) {
                k_mutex_lock(&writer_state.stats_mutex, K_FOREVER);
                writer_state.stats.write_errors++;
                k_mutex_unlock(&writer_state.stats_mutex);
            }
        }
        
        /* Free request data */
        k_free(req.data);
        
        /* Signal if we're closing and queue is empty */
        if (!writer_state.file_open && k_msgq_num_used_get(&writer_state.msgq) == 0) {
            k_sem_give(&writer_state.close_sem);
        }
    }
    
    LOG_INF("File writer thread exiting");
}

static int do_write(const void *data, size_t len)
{
    size_t space_left = writer_state.config.buffer_size - writer_state.buffer_used;
    
    /* If data fits in buffer, just copy it */
    if (len <= space_left) {
        memcpy(writer_state.buffer + writer_state.buffer_used, data, len);
        writer_state.buffer_used += len;
        
        /* Count lines (look for \n) */
        const char *p = data;
        for (size_t i = 0; i < len; i++) {
            if (p[i] == '\n') {
                k_mutex_lock(&writer_state.stats_mutex, K_FOREVER);
                writer_state.stats.lines_written++;
                k_mutex_unlock(&writer_state.stats_mutex);
            }
        }
        
        /* Flush if buffer is getting full */
        if (writer_state.buffer_used >= writer_state.config.buffer_size * 3 / 4) {
            return do_flush();
        }
        
        return 0;
    }
    
    /* Data doesn't fit - flush buffer first */
    int ret = do_flush();
    if (ret != 0) {
        return ret;
    }
    
    /* If data is larger than buffer, write directly */
    if (len > writer_state.config.buffer_size) {
        ret = storage_write(&writer_state.file, data, len);
        if (ret == 0) {
            k_mutex_lock(&writer_state.stats_mutex, K_FOREVER);
            writer_state.stats.bytes_written += len;
            
            /* Count lines */
            const char *p = data;
            for (size_t i = 0; i < len; i++) {
                if (p[i] == '\n') {
                    writer_state.stats.lines_written++;
                }
            }
            k_mutex_unlock(&writer_state.stats_mutex);
        }
        return ret;
    }
    
    /* Copy to now-empty buffer */
    memcpy(writer_state.buffer, data, len);
    writer_state.buffer_used = len;
    
    return 0;
}

static int do_flush(void)
{
    int ret = 0;
    
    if (writer_state.buffer_used == 0) {
        return 0;
    }
    
    /* Write buffer to storage */
    ret = storage_write(&writer_state.file, writer_state.buffer, 
                        writer_state.buffer_used);
    if (ret == 0) {
        /* Sync to ensure data is on storage */
        storage_sync(&writer_state.file);
        
        k_mutex_lock(&writer_state.stats_mutex, K_FOREVER);
        writer_state.stats.bytes_written += writer_state.buffer_used;
        writer_state.stats.flushes++;
        k_mutex_unlock(&writer_state.stats_mutex);
        
        writer_state.buffer_used = 0;
    } else {
        LOG_ERR("Flush failed: %d", ret);
    }
    
    return ret;
}

static void flush_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);
    
    if (writer_state.file_open) {
        do_flush();
        
        /* Reschedule */
        k_work_reschedule(&writer_state.flush_work,
                          K_MSEC(writer_state.config.flush_interval_ms));
    }
}