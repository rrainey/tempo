/*
 * SPDX-License-Identifier: Apache-2.0
 * 
 * Tempo-BT V1 - Main Application Entry
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/fs/fs.h>
#include <string.h>

#include "app_init.h"
#include "app/app_state.h"
#include "app/events.h"
#include "config/settings.h"
#include "services/timebase.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

/* Test event handler */
static void test_event_handler(const app_event_t *event, void *user_data)
{
    LOG_INF("Test handler received event type %d at %u ms", 
            event->type, event->timestamp_ms);
}

/* Test event subscriber */
static event_subscriber_t test_subscriber;

static int smoke_test_file_write(void)
{
    struct fs_file_t file;
    int ret;
    const char *test_str = "test\n";
    char read_buf[32];
    ssize_t bytes_written, bytes_read;

    fs_file_t_init(&file);

    /* Open file for writing */
    ret = fs_open(&file, "/lfs/logs/smoke.txt", FS_O_CREATE | FS_O_WRITE);
    if (ret < 0) {
        LOG_ERR("Failed to open file for writing: %d", ret);
        return ret;
    }

    /* Write test string */
    bytes_written = fs_write(&file, test_str, strlen(test_str));
    if (bytes_written < 0) {
        LOG_ERR("Failed to write to file: %d", bytes_written);
        fs_close(&file);
        return bytes_written;
    }
    LOG_INF("Wrote %d bytes to smoke.txt", bytes_written);

    /* Close file */
    ret = fs_close(&file);
    if (ret < 0) {
        LOG_ERR("Failed to close file: %d", ret);
        return ret;
    }

    /* Open file for reading to verify */
    ret = fs_open(&file, "/lfs/logs/smoke.txt", FS_O_READ);
    if (ret < 0) {
        LOG_ERR("Failed to open file for reading: %d", ret);
        return ret;
    }

    /* Read back the content */
    bytes_read = fs_read(&file, read_buf, sizeof(read_buf) - 1);
    if (bytes_read < 0) {
        LOG_ERR("Failed to read from file: %d", bytes_read);
        fs_close(&file);
        return bytes_read;
    }
    read_buf[bytes_read] = '\0';
    LOG_INF("Read %d bytes: '%s'", bytes_read, read_buf);

    /* Close file */
    fs_close(&file);

    /* Verify content matches */
    if (bytes_read == bytes_written && memcmp(test_str, read_buf, bytes_read) == 0) {
        LOG_INF("File write/read smoke test PASSED");
        return 0;
    } else {
        LOG_ERR("File content mismatch!");
        return -1;
    }
}

int main(void)
{
    int ret;
    
    printk("boot\n");
    LOG_INF("hello");
    LOG_INF("Tempo-BT V1 started successfully");
    
    /* Initialize app state first */
    ret = app_state_init();
    if (ret < 0) {
        LOG_ERR("Failed to initialize app state: %d", ret);
    }
    
    /* Initialize event bus */
    ret = event_bus_init();
    if (ret < 0) {
        LOG_ERR("Failed to initialize event bus: %d", ret);
    }
    
    /* Subscribe to test events */
    ret = event_bus_subscribe(&test_subscriber, 
                             test_event_handler,
                             (1U << EVT_TEST_DUMMY) | (1U << EVT_MODE_CHANGE),
                             NULL);
    if (ret < 0) {
        LOG_ERR("Failed to subscribe to events: %d", ret);
    }
    
    /* Publish a test event */
    LOG_INF("Publishing test event");
    event_bus_publish_simple(EVT_TEST_DUMMY);
    
    /* Give event thread time to process */
    k_msleep(10);
    
    /* Initialize settings */
    ret = app_settings_init();
    if (ret < 0) {
        LOG_ERR("Failed to initialize settings: %d", ret);
    }
    
    /* Test settings */
    LOG_INF("About to test settings...");
    app_settings_test();
    LOG_INF("Settings test done");
    
    /* Initialize timebase */
    LOG_INF("About to init timebase...");
    ret = timebase_init();
    if (ret < 0) {
        LOG_ERR("Failed to initialize timebase: %d", ret);
    }
    LOG_INF("Timebase init done");
    
    /* Test monotonic timer */
    LOG_INF("Testing monotonic timer...");
    uint64_t t1 = time_now_us();
    k_msleep(10);
    uint64_t t2 = time_now_us();
    LOG_INF("10ms sleep measured as %llu us", t2 - t1);
    
    /* Test UTC correlation (should fail initially) */
    uint64_t utc_ms;
    if (timebase_mono_to_utc(time_now_us(), &utc_ms)) {
        LOG_INF("UTC time available: %llu ms", utc_ms);
    } else {
        LOG_INF("UTC correlation not available yet: %s", 
                timebase_utc_string_placeholder());
    }
    
    /* Print current mode on boot */
    LOG_INF("Current mode: %s", 
            app_state_get_mode() == APP_MODE_IDLE ? "IDLE" : "UNKNOWN");
    
    LOG_INF("About to check QSPI flash...");
    
    /* Check if QSPI flash device is ready */
    const struct device *flash_dev = DEVICE_DT_GET(DT_NODELABEL(mx25r64));
    if (!device_is_ready(flash_dev)) {
        LOG_ERR("QSPI flash device not ready");
    } else {
        LOG_INF("QSPI flash device is ready");
        
        /* Get flash parameters */
        const struct flash_parameters *flash_params = flash_get_parameters(flash_dev);
        LOG_INF("Flash write block size: %d", flash_params->write_block_size);
        
        /* Get flash size */
        uint64_t flash_size;
        int res = flash_get_size(flash_dev, &flash_size);
        if (res == 0) {
            LOG_INF("Flash size: %llu bytes", flash_size);
        }
        
        /* Try to get page info */
        struct flash_pages_info info;
        ret = flash_get_page_info_by_offs(flash_dev, 0, &info);
        if (ret == 0) {
            LOG_INF("Flash page size: %d, start offset: 0x%x", 
                    info.size, (unsigned int)info.start_offset);
        }
    }
    
    /* Initialize storage */
    ret = app_storage_init();
    if (ret < 0) {
        LOG_ERR("Failed to initialize storage: %d", ret);
        /* Continue anyway for now */
    }
    
    /* Run smoke test */
    ret = smoke_test_file_write();
    if (ret < 0) {
        LOG_ERR("Smoke test failed: %d", ret);
    }
    
    /* Log periodic heartbeat */
    while (1) {
        k_sleep(K_SECONDS(5));
        LOG_DBG("Heartbeat - system running");
    }
    
    return 0;
}