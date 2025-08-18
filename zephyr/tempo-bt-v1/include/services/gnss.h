/*
 * SPDX-License-Identifier: Apache-2.0
 * 
 * Tempo-BT V1 - GNSS Service
 */

#ifndef SERVICES_GNSS_H
#define SERVICES_GNSS_H

#include <stdint.h>
#include <stdbool.h>

/* GNSS fix data structure */
typedef struct {
    /* Position */
    double latitude;        /* Degrees, -90 to +90 */
    double longitude;       /* Degrees, -180 to +180 */
    double altitude;        /* Meters above MSL */
    
    /* Velocity */
    float speed_mps;        /* Speed in m/s */
    float course_deg;       /* Course over ground in degrees */
    
    /* Quality */
    uint8_t fix_quality;    /* 0=invalid, 1=GPS, 2=DGPS, etc */
    uint8_t num_satellites; /* Number of satellites used */
    float hdop;             /* Horizontal dilution of precision */
    
    /* Time */
    uint8_t hours;          /* UTC hours (0-23) */
    uint8_t minutes;        /* UTC minutes (0-59) */
    uint8_t seconds;        /* UTC seconds (0-59) */
    uint16_t milliseconds;  /* UTC milliseconds (0-999) */
    
    /* Validity */
    bool position_valid;
    bool time_valid;
    
    /* Timestamp */
    uint64_t timestamp_us;  /* Monotonic timestamp when fix was received */
} gnss_fix_t;

/* Callback type for GNSS fix updates */
typedef void (*gnss_fix_callback_t)(const gnss_fix_t *fix);

/* Callback type for raw NMEA sentences */
typedef void (*gnss_nmea_callback_t)(const char *sentence, size_t len);

/**
 * @brief Initialize the GNSS service
 * 
 * Sets up UART for GNSS communication
 * 
 * @return 0 on success, negative error code on failure
 */
int gnss_init(void);

/**
 * @brief Register callback for fix updates
 * 
 * @param callback Function to call when new fix is available
 */
void gnss_register_fix_callback(gnss_fix_callback_t callback);

/**
 * @brief Register callback for raw NMEA sentences
 * 
 * @param callback Function to call for each NMEA sentence
 */
void gnss_register_nmea_callback(gnss_nmea_callback_t callback);

/**
 * @brief Get current fix data
 * 
 * @param fix Output structure to fill
 * @return true if fix is valid, false otherwise
 */
bool gnss_get_current_fix(gnss_fix_t *fix);

/**
 * @brief Configure GNSS update rate
 * 
 * @param rate_hz Update rate in Hz (1-10)
 * @return 0 on success, negative error code on failure
 */
int gnss_set_rate(uint8_t rate_hz);

#endif /* SERVICES_GNSS_H */