/*
 * SPDX-License-Identifier: Apache-2.0
 * 
 * Tempo-BT V1 - Main Application Entry
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/device.h>
#include <zephyr/drivers/flash.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/fs/fs.h>
#include <string.h>

#include "app_init.h"
#include "app/app_state.h"
#include "app/events.h"
#include "config/settings.h"
#include "services/timebase.h"
#include "services/storage.h"
#include "services/file_writer.h"
#include "services/gnss.h"
#include "services/imu.h"
#include "services/baro.h"
#include "services/logger.h"

#include "services/aggregator.h"
#include "util/nmea_checksum.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

/*
 * Button handling for logger control
 */

#define BUTTON_DEBOUNCE_DELAY_MS 50
#define BUTTON_LONG_PRESS_MS     2000

/* Button GPIO specs from device tree */
#define SW0_NODE DT_ALIAS(sw0)
#define SW1_NODE DT_ALIAS(sw1)

#define LED0_NODE DT_ALIAS(led1)
static const struct gpio_dt_spec led = GPIO_DT_SPEC_GET(LED0_NODE, gpios);

#if DT_NODE_HAS_STATUS(SW0_NODE, okay)
static const struct gpio_dt_spec button0 = GPIO_DT_SPEC_GET(SW0_NODE, gpios);
static struct gpio_callback button0_cb_data;
static struct k_work_delayable button0_work;
static int64_t button0_press_time;
#endif

#if DT_NODE_HAS_STATUS(SW1_NODE, okay)
static const struct gpio_dt_spec button1 = GPIO_DT_SPEC_GET(SW1_NODE, gpios);
static struct gpio_callback button1_cb_data;
#endif

/* Button work handler for long press detection */
static void button0_work_handler(struct k_work *work)
{
    ARG_UNUSED(work);
    
    /* Check if button is still pressed */
    int val = gpio_pin_get_dt(&button0);
    if (val == 1) { /* Active low, so 1 means pressed */
        int64_t press_duration = k_uptime_get() - button0_press_time;
        
        if (press_duration >= BUTTON_LONG_PRESS_MS) {
            LOG_INF("Button 0 long press detected");
            
            /* Toggle between idle and armed states */
            logger_state_t state = logger_get_state();
            if (state == LOGGER_STATE_IDLE) {
                logger_arm();
            } else if (state == LOGGER_STATE_ARMED) {
                logger_disarm();
            }
        }
    }
}

/* Button interrupt callbacks */
static void button0_pressed(const struct device *dev, struct gpio_callback *cb,
                            uint32_t pins)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(cb);
    ARG_UNUSED(pins);
    
    button0_press_time = k_uptime_get();
    
    /* Schedule work to check for long press */
    k_work_reschedule(&button0_work, K_MSEC(BUTTON_LONG_PRESS_MS));
}

static void button0_released(const struct device *dev, struct gpio_callback *cb,
                             uint32_t pins)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(cb);
    ARG_UNUSED(pins);
    
    /* Cancel long press detection */
    k_work_cancel_delayable(&button0_work);
    
    int64_t press_duration = k_uptime_get() - button0_press_time;
    
    if (press_duration < BUTTON_LONG_PRESS_MS && 
        press_duration > BUTTON_DEBOUNCE_DELAY_MS) {
        LOG_INF("Button 0 short press detected");
        
        /* Short press: start/stop logging */
        logger_state_t state = logger_get_state();
        if (state == LOGGER_STATE_ARMED) {
            logger_start();
        } else if (state == LOGGER_STATE_LOGGING) {
            logger_stop();
        }
    }
}

#if DT_NODE_HAS_STATUS(SW1_NODE, okay)
static void button1_pressed(const struct device *dev, struct gpio_callback *cb,
                            uint32_t pins)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(cb);
    ARG_UNUSED(pins);
    
    LOG_INF("Button 1 pressed");
    
    /* Button 1 could be used for other functions like:
     * - Force flush
     * - Mark waypoint
     * - Switch logging mode
     */
}
#endif

static void button0_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    ARG_UNUSED(dev);
    ARG_UNUSED(cb);
    ARG_UNUSED(pins);

    int val = gpio_pin_get_dt(&button0);
    if (val == 1) { /* Pressed (active low) */
        button0_pressed(dev, cb, pins);
    } else {
        button0_released(dev, cb, pins);
    }
}

/* Initialize button handling */
int buttons_init(void)
{
    int ret;
    
#if DT_NODE_HAS_STATUS(SW0_NODE, okay)
    if (!device_is_ready(button0.port)) {
        LOG_ERR("Button 0 device not ready");
        return -ENODEV;
    }
    
    ret = gpio_pin_configure_dt(&button0, GPIO_INPUT);
    if (ret != 0) {
        LOG_ERR("Failed to configure button 0: %d", ret);
        return ret;
    }
    
    ret = gpio_pin_interrupt_configure_dt(&button0, GPIO_INT_EDGE_BOTH);
    if (ret != 0) {
        LOG_ERR("Failed to configure button 0 interrupt: %d", ret);
        return ret;
    }
    
    k_work_init_delayable(&button0_work, button0_work_handler);
    
    gpio_init_callback(&button0_cb_data, 
                       button0_isr,
                       BIT(button0.pin));
    
    gpio_add_callback(button0.port, &button0_cb_data);
    
    LOG_INF("Button 0 initialized");
#endif

#if DT_NODE_HAS_STATUS(SW1_NODE, okay)
    if (!device_is_ready(button1.port)) {
        LOG_ERR("Button 1 device not ready");
        return -ENODEV;
    }
    
    ret = gpio_pin_configure_dt(&button1, GPIO_INPUT);
    if (ret != 0) {
        LOG_ERR("Failed to configure button 1: %d", ret);
        return ret;
    }
    
    ret = gpio_pin_interrupt_configure_dt(&button1, GPIO_INT_EDGE_TO_ACTIVE);
    if (ret != 0) {
        LOG_ERR("Failed to configure button 1 interrupt: %d", ret);
        return ret;
    }
    
    gpio_init_callback(&button1_cb_data, button1_pressed, BIT(button1.pin));
    gpio_add_callback(button1.port, &button1_cb_data);
    
    LOG_INF("Button 1 initialized");
#endif

    return 0;
}

static void test_version_output(const char *line, size_t len)
{
    /* Print to console for verification */
    printk("Generated: %.*s", len, line);
    
    /* Verify checksum */
    if (nmea_verify_checksum(line, len)) {
        printk("Checksum verified!\n");
    } else {
        printk("Checksum FAILED!\n");
    }
}

#if 0

/* Test event handler */
static void test_event_handler(const app_event_t *event, void *user_data)
{
    LOG_INF("Test handler received event type %d at %u ms", 
            event->type, event->timestamp_ms);
}

/* Test event subscriber */
static event_subscriber_t test_subscriber;

/* Test NMEA callback */
static int nmea_count = 0;
static void test_nmea_callback(const char *sentence, size_t len)
{
    nmea_count++;
    /* Only log every 100th sentence to reduce output */
    if (nmea_count % 100 == 1) {
        LOG_INF("NMEA #%d (%d bytes): %s", nmea_count, len, sentence);
    }
}

static void test_sentence_generation(void)
{
    LOG_INF("Testing sentence generation...");
    
    /* Set up aggregator */
    aggregator_config_t config = {
        .imu_output_rate = 40,
        .env_output_rate = 4,
        .gnss_output_rate = 1,
        .mag_output_rate = 0,
        .enable_quaternion = true,
        .enable_magnetometer = false,
        .session_id = 12345,
        .session_start_us = time_now_us()
    };
    
    aggregator_configure(&config);
    aggregator_register_output_callback(test_version_output);
    
    /* Test session header */
    LOG_INF("=== Session Header ===");
    aggregator_write_session_header();
    
    /* Test state change */
    LOG_INF("=== State Change ===");
    aggregator_write_state_change("IDLE", "ARMED", "button_press");
    
    /* Simulate some sensor data and test output */
    /* This would normally come from actual sensors */
}

/* Test fix callback */
static void test_fix_callback(const gnss_fix_t *fix)
{
    /* Only log every 10th fix to reduce output */
    static int fix_count = 0;
    fix_count++;
    
    if (fix_count % 10 == 1) {
        LOG_INF("Fix update #%d: Lat=%.6f, Lon=%.6f, Alt=%.1f m, Speed=%.1f m/s",
                fix_count, fix->latitude, fix->longitude, fix->altitude, (double) fix->speed_mps);
        LOG_INF("  Time: %02d:%02d:%02d.%03d UTC, Sats=%d, HDOP=%.1f",
                fix->hours, fix->minutes, fix->seconds, fix->milliseconds,
                fix->num_satellites, (double) fix->hdop);
    }
    
    /* Update time correlation when we have a valid fix */
    if (fix->time_valid && fix->position_valid && (fix_count % 10 == 1)) {
        /* Calculate UTC time in milliseconds since epoch */
        /* Note: This is simplified - doesn't handle date rollover */
        uint64_t utc_ms = (fix->hours * 3600 + fix->minutes * 60 + fix->seconds) * 1000 
                        + fix->milliseconds;
        
        /* Update timebase correlation */
        timebase_update_correlation(utc_ms, 100);  /* 100ms accuracy estimate */
        LOG_INF("Updated time correlation");
    }
}

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
#endif

int main(void)
{
    int ret;

    const struct device *uart = DEVICE_DT_GET(DT_CHOSEN(zephyr_console));
    if (device_is_ready(uart)) {
        for (const char *p = "UART TEST\r\n"; *p; p++) {
            uart_poll_out(uart, *p);
        }
    }
    
    printk("boot\n");
    LOG_INF("hello");
    LOG_INF("Tempo-BT V1 started successfully");
    
    /* Initialize app state first */
    ret = app_state_init();
    if (ret < 0) {
        LOG_ERR("Failed to initialize app state: %d", ret);
    }

    // Initialize storage first
    ret = app_storage_init();
    if (ret < 0) {
        LOG_ERR("Failed to initialize storage: %d", ret);
        return ret;
    }

    // Initialize storage service layer - prefer SD card for flight logs
    if (storage_fatfs_card_present()) {
        LOG_INF("SD card detected, using FAT filesystem for flight logs");
        ret = storage_init(STORAGE_BACKEND_FATFS);
        if (ret < 0) {
            LOG_ERR("Failed to initialize FAT storage: %d", ret);
            LOG_WRN("Falling back to internal littlefs storage");
            ret = storage_init(STORAGE_BACKEND_LITTLEFS);
        }
    } else {
        LOG_INF("No SD card detected, using internal littlefs storage");
        ret = storage_init(STORAGE_BACKEND_LITTLEFS);
    }
    
    if (ret < 0) {
        LOG_ERR("Failed to initialize storage service: %d", ret);
        return ret;
    }

    // Now initialize app (which includes BLE/mcumgr)
    ret = app_init();
    if (ret < 0) {
        LOG_ERR("Failed to initialize app: %d", ret);
    }

    if (!gpio_is_ready_dt(&led)) {
		//return 0;
	}
	ret = gpio_pin_configure_dt(&led, GPIO_OUTPUT_ACTIVE);

    ret = buttons_init();
    if (ret < 0) {
        LOG_ERR("Failed to initialize buttons: %d", ret);
    }   

    /* Initialize event bus */
    ret = event_bus_init();
    if (ret < 0) {
        LOG_ERR("Failed to initialize event bus: %d", ret);
    }
    
#if 0
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
#endif
    
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

    ret = aggregator_init();
    if (ret < 0) {
        LOG_ERR("Failed to initialize aggregator: %d", ret);
    }
    LOG_INF("Aggregator init done");    

    aggregator_register_output_callback(test_version_output);
    aggregator_write_version();
    
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
    
    #if 0
    
    /* Run smoke test */
    ret = smoke_test_file_write();
    if (ret < 0) {
        LOG_ERR("Smoke test failed: %d", ret);
    }
    #endif
    
    /* Initialize GNSS */
    #if 0  /* Temporarily disabled to focus on IMU */
    LOG_INF("About to init GNSS...");
    ret = gnss_init();
    if (ret < 0) {
        LOG_ERR("Failed to initialize GNSS: %d", ret);
    } else {
        /* Register test callback */
        gnss_register_nmea_callback(test_nmea_callback);
        gnss_register_fix_callback(test_fix_callback);
        LOG_INF("GNSS initialized, waiting for NMEA data...");
    }
   

    test_sentence_generation();

     #endif
    
    /* Initialize IMU - but skip it for now due to hardware issues */
    #if 1
    LOG_INF("About to init IMU...");
    ret = imu_init();
    if (ret < 0) {
        LOG_ERR("Failed to initialize IMU: %d", ret);
    } else {
        LOG_INF("IMU initialized successfully");
    }
    #endif
    
    #if 0
    /* Test Barometer (BMP390) */
    LOG_INF("Testing BMP390 barometer...");
    extern int test_baro(void);
    ret = test_baro();
    if (ret < 0) {
        LOG_ERR("Barometer test failed: %d", ret);
    } else {
        LOG_INF("Barometer test completed successfully");
    }
    #endif

    ret = file_writer_init(NULL);  // Use default config
    if (ret < 0) {
        LOG_ERR("Failed to initialize file writer: %d", ret);
    }

     logger_config_t logger_cfg = {
        .base_path = "/logs",
        .use_date_folders = true,
        .use_uuid_names = true,
        .imu_rate_hz = 40,
        .env_rate_hz = 4,
        .gnss_rate_hz = 1,
        .enable_quaternion = true,
        .enable_magnetometer = false,
        .auto_start_on_takeoff = false,
        .auto_stop_on_landing = false
    };

    ret = logger_init(&logger_cfg);
    if (ret < 0) {
        LOG_ERR("Failed to initialize logger: %d", ret);
    }
    
    LOG_INF("System initialization complete");
    //LOG_INF("Connect via: mcumgr --conntype ble --connstring peer_name='Tempo-BT' echo hello");

    // Main thread idle loop
    while (1) {
        /* Log health status periodically */
        k_sleep(K_SECONDS(30));

        ret = gpio_pin_toggle_dt(&led);
        
        /* Get storage stats */
        uint64_t free_bytes, total_bytes;
        if (storage_get_free_space(&free_bytes, &total_bytes) == 0) {
            LOG_INF("Storage: %llu MB free of %llu MB", 
                    free_bytes / (1024*1024), total_bytes / (1024*1024));
        }
    }
    
    return 0;
}