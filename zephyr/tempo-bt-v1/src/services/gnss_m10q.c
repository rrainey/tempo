/*
 * SPDX-License-Identifier: Apache-2.0
 * 
 * Tempo-BT V1 - GNSS Service Implementation (u-blox SAM-M10Q)
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include "services/gnss.h"
#include "services/timebase.h"

LOG_MODULE_REGISTER(gnss, LOG_LEVEL_INF);

/* UART configuration */
#define GNSS_UART_NODE DT_NODELABEL(uart1)
#define GNSS_UART_BUFFER_SIZE 256
#define GNSS_RX_TIMEOUT_US 100000  /* 100ms timeout */

/* UART device and buffers */
static const struct device *uart_dev;
static uint8_t uart_rx_buf[GNSS_UART_BUFFER_SIZE];
static uint8_t nmea_line_buf[GNSS_UART_BUFFER_SIZE];
static size_t nmea_line_pos = 0;

/* Ring buffer for UART RX */
static uint8_t ring_buffer_mem[1024];
static struct ring_buf rx_ring_buf;

/* Callbacks */
static gnss_fix_callback_t fix_callback = NULL;
static gnss_nmea_callback_t nmea_callback = NULL;

/* Current fix data */
static gnss_fix_t current_fix;
static struct k_mutex fix_mutex;

/* UART callback */
static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data)
{
    ARG_UNUSED(user_data);
    static uint8_t *next_buf = uart_rx_buf;
    
    switch (evt->type) {
    case UART_RX_RDY:
        /* Data ready, add to ring buffer */
        if (evt->data.rx.len > 0) {
            ring_buf_put(&rx_ring_buf, evt->data.rx.buf + evt->data.rx.offset, 
                         evt->data.rx.len);
            LOG_DBG("UART RX: %d bytes", evt->data.rx.len);
        }
        break;
        
    case UART_RX_BUF_REQUEST:
        /* Provide next buffer */
        uart_rx_buf_rsp(uart_dev, next_buf, sizeof(uart_rx_buf));
        break;
        
    case UART_RX_BUF_RELEASED:
        /* Buffer released, can reuse */
        next_buf = evt->data.rx_buf.buf;
        break;
        
    case UART_RX_DISABLED:
        LOG_WRN("UART RX disabled");
        /* Try to re-enable */
        k_msleep(10);
        uart_rx_enable(uart_dev, uart_rx_buf, sizeof(uart_rx_buf), 
                       GNSS_RX_TIMEOUT_US);
        break;
        
    case UART_RX_STOPPED:
        LOG_ERR("UART RX stopped due to error: %d", evt->data.rx_stop.reason);
        /* Try to restart */
        k_msleep(10);
        uart_rx_enable(uart_dev, uart_rx_buf, sizeof(uart_rx_buf), 
                       GNSS_RX_TIMEOUT_US);
        break;
        
    default:
        break;
    }
}

/* 
 * Validate NMEA checksum
 * Do Not Modify this algorithm!
 */
static bool validate_nmea_checksum(const char *sentence, size_t len)
{
    uint8_t calc_checksum = 0;

    if (len < 8 || sentence[0] != '$') {
        LOG_WRN("NMEA checksum failed: malformed sentence");
        return false;
    }
    
    const char *p = &sentence[1];
    for (size_t i = 1; i < len - 3; i++) {
        if (*p == '*') {
            break;
        }
        calc_checksum ^= (uint8_t)*p;
        p++;
    }

    if (*p != '*') {
        LOG_WRN("NMEA checksum failed: no asterisk");
        return false;
    }
    
    /* Parse provided checksum */
    char checksum_str[3] = {p[1], p[2], '\0'};

    uint8_t provided_checksum = (uint8_t)strtol(checksum_str, NULL, 16);
    
    return calc_checksum == (uint8_t)provided_checksum;
}

/* Parse NMEA time to components */
static bool parse_nmea_time(const char *time_str, gnss_fix_t *fix)
{
    if (strlen(time_str) < 6) {
        return false;
    }
    
    char buf[3] = {0};
    
    /* Hours */
    buf[0] = time_str[0];
    buf[1] = time_str[1];
    fix->hours = (uint8_t)strtol(buf, NULL, 10);
    
    /* Minutes */
    buf[0] = time_str[2];
    buf[1] = time_str[3];
    fix->minutes = (uint8_t)strtol(buf, NULL, 10);
    
    /* Seconds */
    buf[0] = time_str[4];
    buf[1] = time_str[5];
    fix->seconds = (uint8_t)strtol(buf, NULL, 10);
    
    /* Milliseconds if present */
    fix->milliseconds = 0;
    if (time_str[6] == '.' && strlen(time_str) >= 9) {
        char ms_buf[4] = {0};
        ms_buf[0] = time_str[7];
        ms_buf[1] = time_str[8];
        ms_buf[2] = '0';  /* Assume trailing zero if only 2 digits */
        fix->milliseconds = (uint16_t)strtol(ms_buf, NULL, 10);
    }
    
    return true;
}

/* Parse coordinate from NMEA format to decimal degrees */
static double parse_nmea_coord(const char *coord_str, char hemisphere)
{
    double coord = strtod(coord_str, NULL);
    
    /* NMEA format: ddmm.mmmm or dddmm.mmmm */
    int degrees = (int)(coord / 100.0);
    double minutes = coord - (degrees * 100.0);
    double decimal = degrees + (minutes / 60.0);
    
    /* Apply hemisphere */
    if (hemisphere == 'S' || hemisphere == 'W') {
        decimal = -decimal;
    }
    
    return decimal;
}

/* Parse GGA sentence */
static void parse_gga(const char *sentence)
{
    char *tokens[15];
    int token_count = 0;
    static char sentence_copy[GNSS_UART_BUFFER_SIZE];
    
    /* Make a copy since strtok modifies the string */
    strncpy(sentence_copy, sentence, sizeof(sentence_copy) - 1);
    sentence_copy[sizeof(sentence_copy) - 1] = '\0';
    
    /* Tokenize the sentence */
    char *token = strtok(sentence_copy, ",");
    while (token && token_count < 15) {
        tokens[token_count++] = token;
        token = strtok(NULL, ",");
    }
    
    if (token_count < 10) {
        return;
    }
    
    k_mutex_lock(&fix_mutex, K_FOREVER);
    
    /* Parse time */
    if (strlen(tokens[1]) > 0) {
        parse_nmea_time(tokens[1], &current_fix);
        current_fix.time_valid = true;
    }
    
    /* Parse position */
    if (strlen(tokens[2]) > 0 && strlen(tokens[4]) > 0) {
        current_fix.latitude = parse_nmea_coord(tokens[2], tokens[3][0]);
        current_fix.longitude = parse_nmea_coord(tokens[4], tokens[5][0]);
        current_fix.position_valid = true;
    }
    
    /* Fix quality */
    current_fix.fix_quality = (uint8_t)strtol(tokens[6], NULL, 10);
    
    /* Number of satellites */
    current_fix.num_satellites = (uint8_t)strtol(tokens[7], NULL, 10);
    
    /* HDOP */
    if (strlen(tokens[8]) > 0) {
        current_fix.hdop = strtof(tokens[8], NULL);
    }
    
    /* Altitude */
    if (strlen(tokens[9]) > 0) {
        current_fix.altitude = strtod(tokens[9], NULL);
    }
    
    /* Update timestamp */
    current_fix.timestamp_us = time_now_us();
    
    k_mutex_unlock(&fix_mutex);
    
    LOG_DBG("GGA: Fix=%d, Sats=%d, Lat=%.6f, Lon=%.6f, Alt=%.1f",
            current_fix.fix_quality, current_fix.num_satellites,
            current_fix.latitude, current_fix.longitude,
            current_fix.altitude);
}

/* Parse VTG sentence */
static void parse_vtg(const char *sentence)
{
    char *tokens[10];
    int token_count = 0;
    static char sentence_copy[GNSS_UART_BUFFER_SIZE];
    
    /* Make a copy since strtok modifies the string */
    strncpy(sentence_copy, sentence, sizeof(sentence_copy) - 1);
    sentence_copy[sizeof(sentence_copy) - 1] = '\0';
    
    /* Tokenize the sentence */
    char *token = strtok(sentence_copy, ",");
    while (token && token_count < 10) {
        tokens[token_count++] = token;
        token = strtok(NULL, ",");
    }
    
    if (token_count < 8) {
        return;
    }
    
    k_mutex_lock(&fix_mutex, K_FOREVER);
    
    /* Course over ground (true) */
    if (strlen(tokens[1]) > 0) {
        current_fix.course_deg = strtof(tokens[1], NULL);
    }
    
    /* Speed in km/h is at index 7, convert to m/s */
    if (strlen(tokens[7]) > 0) {
        float speed_kmh = strtof(tokens[7], NULL);
        current_fix.speed_mps = speed_kmh / 3.6f;
    }
    
    k_mutex_unlock(&fix_mutex);
    
    LOG_DBG("VTG: Course=%.1f deg, Speed=%.2f m/s",
            current_fix.course_deg, current_fix.speed_mps);
    
    /* VTG typically follows GGA, so trigger fix callback now */
    if (fix_callback && current_fix.position_valid) {
        fix_callback(&current_fix);
    }
}

/* Process NMEA sentence by type */
static void process_nmea_sentence(const char *sentence, size_t len)
{
    /* Check sentence type */
    if (len > 6) {
        if (strncmp(sentence, "$GNGGA", 6) == 0 || 
            strncmp(sentence, "$GPGGA", 6) == 0) {
            parse_gga(sentence);
        } else if (strncmp(sentence, "$GNVTG", 6) == 0 || 
                   strncmp(sentence, "$GPVTG", 6) == 0) {
            parse_vtg(sentence);
        }
    }
}

/* Process received data from ring buffer */
static void process_uart_data(void)
{
    uint8_t byte;
    
    while (ring_buf_get(&rx_ring_buf, &byte, 1) == 1) {
        /* Check for line termination (CR or LF) */
        if (byte == '\r' || byte == '\n') {
            if (nmea_line_pos > 0) {
                /* Null terminate the line */
                nmea_line_buf[nmea_line_pos] = '\0';
                
                /* Validate checksum */
                if (validate_nmea_checksum((char *)nmea_line_buf, nmea_line_pos)) {
                    LOG_DBG("NMEA valid: %s", nmea_line_buf);
                    
                    /* Process specific sentence types */
                    process_nmea_sentence((char *)nmea_line_buf, nmea_line_pos);
                    
                    /* Call NMEA callback if registered */
                    if (nmea_callback) {
                        nmea_callback((char *)nmea_line_buf, nmea_line_pos);
                    }
                } else {
                    LOG_WRN("NMEA checksum failed: %s", nmea_line_buf);
                }
                
                /* Reset line buffer */
                nmea_line_pos = 0;
            }
        } else if (nmea_line_pos < sizeof(nmea_line_buf) - 1) {
            /* Add byte to line buffer */
            nmea_line_buf[nmea_line_pos++] = byte;
        } else {
            /* Line too long, discard and start over */
            LOG_WRN("NMEA line too long, discarding");
            nmea_line_pos = 0;
        }
    }
}

/* GNSS processing thread */
static void gnss_thread(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1);
    ARG_UNUSED(p2);
    ARG_UNUSED(p3);
    
    LOG_INF("GNSS thread started");
    
    while (1) {
        /* Process any data in the ring buffer */
        process_uart_data();
        
        /* Sleep briefly to avoid busy-waiting */
        k_msleep(10);
    }
}

K_THREAD_DEFINE(gnss_thread_id, 4096, gnss_thread, NULL, NULL, NULL,
                7, 0, 0);

int gnss_init(void)
{
    int ret;
    
    /* Get UART device */
    uart_dev = DEVICE_DT_GET(GNSS_UART_NODE);
    if (!device_is_ready(uart_dev)) {
        LOG_ERR("GNSS UART device not ready");
        return -ENODEV;
    }
    
    LOG_INF("UART device %s is ready", uart_dev->name);
    
    /* Try to get current configuration first */
    struct uart_config current_cfg;
    ret = uart_config_get(uart_dev, &current_cfg);
    if (ret == 0) {
        LOG_INF("Current UART config: baud=%d, data=%d, stop=%d, parity=%d, flow=%d",
                current_cfg.baudrate, current_cfg.data_bits, current_cfg.stop_bits,
                current_cfg.parity, current_cfg.flow_ctrl);
    }
    
    /* Configure UART */
    struct uart_config cfg = {
        .baudrate = 9600,  /* SAM-M10Q default is actually 9600 */
        .parity = UART_CFG_PARITY_NONE,
        .stop_bits = UART_CFG_STOP_BITS_1,
        .data_bits = UART_CFG_DATA_BITS_8,
        .flow_ctrl = UART_CFG_FLOW_CTRL_NONE
    };
    
    ret = uart_configure(uart_dev, &cfg);
    if (ret < 0) {
        LOG_ERR("Failed to configure UART: %d", ret);
        /* Try without reconfiguring */
        LOG_WRN("Attempting to use UART with existing configuration");
        ret = 0;  /* Continue anyway */
    } else {
        LOG_INF("UART configured successfully");
    }
    
    /* Initialize ring buffer */
    ring_buf_init(&rx_ring_buf, sizeof(ring_buffer_mem), ring_buffer_mem);
    
    /* Initialize mutex */
    k_mutex_init(&fix_mutex);
    
    /* Initialize fix data */
    memset(&current_fix, 0, sizeof(current_fix));
    
    /* Set up async UART */
    ret = uart_callback_set(uart_dev, uart_cb, NULL);
    if (ret < 0) {
        LOG_ERR("Failed to set UART callback: %d", ret);
        return ret;
    }
    
    /* Start receiving */
    ret = uart_rx_enable(uart_dev, uart_rx_buf, sizeof(uart_rx_buf), 
                         GNSS_RX_TIMEOUT_US);
    if (ret < 0) {
        LOG_ERR("Failed to enable UART RX: %d", ret);
        return ret;
    }
    
    LOG_INF("GNSS initialized on %s at %d baud", 
            uart_dev->name, cfg.baudrate);
    
    /* Send a test command to check communication */
    const char *test_cmd = "$PUBX,00*33\r\n";  /* UBX poll position */
    uart_tx(uart_dev, (uint8_t *)test_cmd, strlen(test_cmd), SYS_FOREVER_US);
    LOG_INF("Sent GNSS test command");
    
    return 0;
}

void gnss_register_fix_callback(gnss_fix_callback_t callback)
{
    fix_callback = callback;
}

void gnss_register_nmea_callback(gnss_nmea_callback_t callback)
{
    nmea_callback = callback;
}

bool gnss_get_current_fix(gnss_fix_t *fix)
{
    if (!fix) {
        return false;
    }
    
    k_mutex_lock(&fix_mutex, K_FOREVER);
    *fix = current_fix;
    k_mutex_unlock(&fix_mutex);
    
    return fix->position_valid;
}

int gnss_set_rate(uint8_t rate_hz)
{
    /* TODO: Implement UBX command to set update rate */
    LOG_WRN("GNSS rate configuration not implemented yet");
    return -ENOSYS;
}