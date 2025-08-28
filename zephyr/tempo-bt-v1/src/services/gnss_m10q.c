/*
 * SPDX-License-Identifier: Apache-2.0
 * 
 * Tempo-BT V1 - GNSS Service Implementation (u-blox SAM-M10Q)
 */
#pragma GCC diagnostic ignored "-Wdouble-promotion"
#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <time.h>
#include <zephyr/posix/time.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>

#include "services/gnss.h"
#include "services/timebase.h"

LOG_MODULE_REGISTER(gnss, LOG_LEVEL_INF);

// forward declaration
static void update_system_time_from_fix(void);

/* UART configuration */
#define GNSS_UART_NODE DT_NODELABEL(uart2)
#define GNSS_UART_BUFFER_SIZE 256
#define GNSS_RX_TIMEOUT_US 100000  /* 100ms timeout */

/* UART device and buffers */
static const struct device *uart_dev;
static uint8_t uart_rx_buf[GNSS_UART_BUFFER_SIZE];
static uint8_t uart_rx_buf2[GNSS_UART_BUFFER_SIZE];  /* Second buffer for double buffering */
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
    static uint8_t *next_buf = uart_rx_buf2;
    
    switch (evt->type) {
    case UART_RX_RDY:
        /* Data ready, add to ring buffer */
        if (evt->data.rx.len > 0) {
            int written = ring_buf_put(&rx_ring_buf, 
                                      evt->data.rx.buf + evt->data.rx.offset, 
                                      evt->data.rx.len);
            if (written < evt->data.rx.len) {
                LOG_WRN("Ring buffer full, dropped %d bytes", 
                        evt->data.rx.len - written);
            }
        }
        break;
        
    case UART_RX_BUF_REQUEST:
        /* Provide next buffer */
        uart_rx_buf_rsp(uart_dev, next_buf, GNSS_UART_BUFFER_SIZE);
        /* Swap buffers */
        next_buf = (next_buf == uart_rx_buf) ? uart_rx_buf2 : uart_rx_buf;
        break;
        
    case UART_RX_BUF_RELEASED:
        /* Buffer released, nothing to do */
        break;
        
    case UART_RX_DISABLED:
        LOG_WRN("UART RX disabled");
        /* Try to re-enable */
        k_msleep(10);
        uart_rx_enable(uart_dev, uart_rx_buf, GNSS_UART_BUFFER_SIZE, 
                       GNSS_RX_TIMEOUT_US);
        break;
        
    case UART_RX_STOPPED:
        LOG_ERR("UART RX stopped due to error: %d", evt->data.rx_stop.reason);
        /* Try to restart */
        k_msleep(10);
        uart_rx_enable(uart_dev, uart_rx_buf, GNSS_UART_BUFFER_SIZE, 
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

    // set date/time (one time event after boot and GPS signal acquisition)
    update_system_time_from_fix();
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

/* Parse GSA sentence for DOP values */
static void parse_gsa(const char *sentence)
{
    char *tokens[20];
    int token_count = 0;
    static char sentence_copy[GNSS_UART_BUFFER_SIZE];
    
    /* Make a copy since strtok modifies the string */
    strncpy(sentence_copy, sentence, sizeof(sentence_copy) - 1);
    sentence_copy[sizeof(sentence_copy) - 1] = '\0';
    
    /* Tokenize the sentence */
    char *token = strtok(sentence_copy, ",");
    while (token && token_count < 20) {
        tokens[token_count++] = token;
        token = strtok(NULL, ",");
    }
    
    /* GSA format:
     * $GPGSA,A,3,04,05,,09,12,,,24,,,,,2.5,1.3,2.1*39
     * Field 0: $GPGSA
     * Field 1: Mode (A=Auto, M=Manual)
     * Field 2: Fix type (1=no fix, 2=2D, 3=3D)
     * Fields 3-14: Satellite PRNs used in solution
     * Field 15: PDOP
     * Field 16: HDOP
     * Field 17: VDOP
     */
    
    if (token_count < 18) {
        LOG_WRN("GSA sentence too short: %d tokens", token_count);
        return;
    }
    
    k_mutex_lock(&fix_mutex, K_FOREVER);
    
    /* Parse PDOP (field 15) - not currently stored but available */
    /* float pdop = 0.0;
    if (strlen(tokens[15]) > 0) {
        pdop = strtof(tokens[15], NULL);
    } */
    
    /* Parse HDOP (field 16) */
    if (strlen(tokens[16]) > 0) {
        current_fix.hdop = strtof(tokens[16], NULL);
    }
    
    /* Parse VDOP (field 17) */
    if (strlen(tokens[17]) > 0) {
        current_fix.vdop = strtof(tokens[17], NULL);
    }
    
    k_mutex_unlock(&fix_mutex);
    
    LOG_DBG("GSA: HDOP=%.1f, VDOP=%.1f", current_fix.hdop, current_fix.vdop);
}

/* Parse RMC sentence for date */
static void parse_rmc(const char *sentence)
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
    
    /* RMC format:
     * $GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A
     * Field 0: $GPRMC
     * Field 1: Time (HHMMSS.sss)
     * Field 2: Status (A=active, V=void)
     * Field 3: Latitude
     * Field 4: N/S
     * Field 5: Longitude
     * Field 6: E/W
     * Field 7: Speed over ground (knots)
     * Field 8: Course over ground (degrees)
     * Field 9: Date (DDMMYY)
     * Field 10: Magnetic variation
     * Field 11: E/W
     */
    
    if (token_count < 10) {
        LOG_WRN("RMC sentence too short: %d tokens", token_count);
        return;
    }
    
    k_mutex_lock(&fix_mutex, K_FOREVER);
    
    /* Check if data is valid */
    if (strlen(tokens[2]) > 0 && tokens[2][0] == 'A') {
        /* Parse time if present */
        if (strlen(tokens[1]) > 0) {
            parse_nmea_time(tokens[1], &current_fix);
            current_fix.time_valid = true;
        }
        
        /* Parse date (DDMMYY format) */
        if (strlen(tokens[9]) >= 6) {
            char buf[3] = {0};
            
            /* Day */
            buf[0] = tokens[9][0];
            buf[1] = tokens[9][1];
            current_fix.day = (uint8_t)strtol(buf, NULL, 10);
            
            /* Month */
            buf[0] = tokens[9][2];
            buf[1] = tokens[9][3];
            current_fix.month = (uint8_t)strtol(buf, NULL, 10);
            
            /* Year (YY format - assume 20YY for now) */
            buf[0] = tokens[9][4];
            buf[1] = tokens[9][5];
            uint8_t year_yy = (uint8_t)strtol(buf, NULL, 10);
            
            /* Convert 2-digit year to 4-digit year */
            /* Assume 2000-2099 range */
            current_fix.year = 2000 + year_yy;
            
            current_fix.date_valid = true;
            
            LOG_DBG("RMC: Date=%04d-%02d-%02d", 
                    current_fix.year, current_fix.month, current_fix.day);
        }
        
        /* Parse position if present (redundant with GGA but can be useful) */
        if (strlen(tokens[3]) > 0 && strlen(tokens[5]) > 0) {
            current_fix.latitude = parse_nmea_coord(tokens[3], tokens[4][0]);
            current_fix.longitude = parse_nmea_coord(tokens[5], tokens[6][0]);
            /* Don't set position_valid here - let GGA handle that */
        }
        
        /* Parse speed over ground (knots to m/s) */
        if (strlen(tokens[7]) > 0) {
            float speed_knots = strtof(tokens[7], NULL);
            current_fix.speed_mps = speed_knots * 0.514444f; /* 1 knot = 0.514444 m/s */
        }
        
        /* Parse course over ground */
        if (strlen(tokens[8]) > 0) {
            current_fix.course_deg = strtof(tokens[8], NULL);
        }
    } else {
        LOG_DBG("RMC: Status void");
    }
    
    k_mutex_unlock(&fix_mutex);

    // set date/time (one time event after boot and GPS signal acquisition)
    update_system_time_from_fix();
}


/* Process NMEA sentence by type */
static void process_nmea_sentence(const char *sentence, size_t len)
{
    if (len > 6) {
        if (strncmp(sentence, "$GNGGA", 6) == 0 || 
            strncmp(sentence, "$GPGGA", 6) == 0) {
            parse_gga(sentence);
        } else if (strncmp(sentence, "$GNVTG", 6) == 0 || 
                   strncmp(sentence, "$GPVTG", 6) == 0) {
            parse_vtg(sentence);
        } else if (strncmp(sentence, "$GNGSA", 6) == 0 || 
                   strncmp(sentence, "$GPGSA", 6) == 0) {
            parse_gsa(sentence);  
        } else if (strncmp(sentence, "$GNRMC", 6) == 0 || 
                   strncmp(sentence, "$GPRMC", 6) == 0) {
            parse_rmc(sentence);  
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
        k_msleep(5);  /* Reduced from 10ms to process faster */
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
    int ret;
    uint8_t ubx_cfg_rate[14];
    uint16_t period_ms;
    
    /* SAM-M10Q supports 1Hz to 10Hz */
    if (rate_hz < 1 || rate_hz > 10) {
        LOG_ERR("Invalid rate %d Hz (SAM-M10Q supports 1-10 Hz)", rate_hz);
        return -EINVAL;
    }
    
    /* Calculate period in milliseconds */
    period_ms = 1000 / rate_hz;
    
    LOG_INF("Setting GNSS rate to %d Hz (period %d ms)", rate_hz, period_ms);
    
    /* Build UBX-CFG-RATE message */
    ubx_cfg_rate[0] = 0xB5;  /* Sync char 1 */
    ubx_cfg_rate[1] = 0x62;  /* Sync char 2 */
    ubx_cfg_rate[2] = 0x06;  /* Class: CFG */
    ubx_cfg_rate[3] = 0x08;  /* ID: RATE */
    ubx_cfg_rate[4] = 0x06;  /* Length LSB */
    ubx_cfg_rate[5] = 0x00;  /* Length MSB */
    ubx_cfg_rate[6] = period_ms & 0xFF;  /* measRate LSB */
    ubx_cfg_rate[7] = (period_ms >> 8) & 0xFF;  /* measRate MSB */
    ubx_cfg_rate[8] = 0x01;  /* navRate LSB (1 = every measurement) */
    ubx_cfg_rate[9] = 0x00;  /* navRate MSB */
    ubx_cfg_rate[10] = 0x01; /* timeRef LSB (1 = GPS time) */
    ubx_cfg_rate[11] = 0x00; /* timeRef MSB */
    
    /* Calculate and add checksum */
    uint8_t ck_a = 0, ck_b = 0;
    for (int i = 2; i < 12; i++) {
        ck_a += ubx_cfg_rate[i];
        ck_b += ck_a;
    }
    ubx_cfg_rate[12] = ck_a;
    ubx_cfg_rate[13] = ck_b;
    
    /* Send command */
    ret = uart_tx(uart_dev, ubx_cfg_rate, sizeof(ubx_cfg_rate), SYS_FOREVER_MS);
    if (ret < 0) {
        LOG_ERR("Failed to send rate config: %d", ret);
        return ret;
    }
    
    /* TODO: Wait for UBX-ACK-ACK response to confirm */
    k_msleep(100);
    
    LOG_INF("GNSS rate configuration sent");
    
    return 0;
}

/* Set system time from GNSS fix data */
static void update_system_time_from_fix(void)
{
    struct timespec ts;
    time_t unix_time;
    struct tm time_tm;
    
    /* Check if we have valid date AND time */
    if (!current_fix.date_valid || !current_fix.time_valid) {
        return;
    }
    
    /* Check if we've already set the time this session */
    static bool time_set = false;
    if (time_set) {
        return;  /* Only set once per boot */
    }
    
    /* Build tm structure */
    memset(&time_tm, 0, sizeof(time_tm));
    time_tm.tm_year = current_fix.year - 1900;  /* tm_year is years since 1900 */
    time_tm.tm_mon = current_fix.month - 1;     /* tm_mon is 0-11 */
    time_tm.tm_mday = current_fix.day;
    time_tm.tm_hour = current_fix.hours;
    time_tm.tm_min = current_fix.minutes;
    time_tm.tm_sec = current_fix.seconds;
    time_tm.tm_isdst = 0;  /* UTC has no DST */
    
    /* Convert to Unix timestamp */
    unix_time = mktime(&time_tm);
    if (unix_time == (time_t)-1) {
        LOG_ERR("Failed to convert GNSS time to Unix timestamp");
        return;
    }
    
    /* Set system time */
    ts.tv_sec = unix_time;
    ts.tv_nsec = current_fix.milliseconds * 1000000;  /* Convert ms to ns */
    
    int ret = clock_settime(CLOCK_REALTIME, &ts);
    if (ret == 0) {
        time_set = true;
        LOG_INF("System time set from GNSS: %04d-%02d-%02d %02d:%02d:%02d.%03d UTC",
                current_fix.year, current_fix.month, current_fix.day,
                current_fix.hours, current_fix.minutes, current_fix.seconds,
                current_fix.milliseconds);
                
        /* Also update the global date string used by aggregator */
        extern char gps_date_string[14];
        snprintf(gps_date_string, sizeof(gps_date_string), 
                 "%04d-%02d-%02d", current_fix.year, current_fix.month, current_fix.day);
    } else {
        LOG_ERR("Failed to set system time: %d", ret);
    }
}

#ifdef CONFIG_RTC
#include <zephyr/drivers/rtc.h>

static void update_rtc_from_fix(void)
{
    const struct device *rtc = DEVICE_DT_GET(DT_ALIAS(rtc0));
    struct rtc_time rtc_tm;
    
    if (!device_is_ready(rtc)) {
        return;
    }
    
    memset(&rtc_tm, 0, sizeof(rtc_tm));
    rtc_tm.tm_year = current_fix.year - 1900;
    rtc_tm.tm_mon = current_fix.month - 1;
    rtc_tm.tm_mday = current_fix.day;
    rtc_tm.tm_hour = current_fix.hours;
    rtc_tm.tm_min = current_fix.minutes;
    rtc_tm.tm_sec = current_fix.seconds;
    
    int ret = rtc_set_time(rtc, &rtc_tm);
    if (ret == 0) {
        LOG_INF("RTC updated from GNSS");
    }
}
#endif