/*
 * SPDX-License-Identifier: Apache-2.0
 * 
 * Tempo-BT V1 - Log Format Definitions
 * 
 * Defines the CSV logging format with NMEA-style $Pxxx sentences
 */

#ifndef APP_LOG_FORMAT_H
#define APP_LOG_FORMAT_H

#include <stdint.h>
#include <stdbool.h>

/* Maximum sentence length including CRLF and null terminator */
#define LOG_MAX_SENTENCE_LEN    256

/* Sentence type identifiers */
#define LOG_SENTENCE_SFC    "$PSFC"   /* Session file config */
#define LOG_SENTENCE_IMU    "$PIMU"   /* IMU data (accel + gyro) */
#define LOG_SENTENCE_IM2    "$PIM2"   /* IMU quaternion */
#define LOG_SENTENCE_ENV    "$PENV"   /* Environmental (baro + temp) */
#define LOG_SENTENCE_ST     "$PST"    /* State change */
#define LOG_SENTENCE_MAG    "$PMAG"   /* Magnetometer (optional) */
#define LOG_SENTENCE_VER    "$PVER"   /* Version info with date */
#define LOG_SENTENCE_FIX    "$PFIX"   /* GPS fix data (renamed from $PTH) */

/* Update version sentence to include date:
 * $PVER,<version>,<hw_rev>,<fw_rev>,<date>*HH
 * Example: $PVER,1.0,V1,0.1.0,2025-01-15*AB
 */
typedef struct {
    const char *version;      /* Protocol version */
    const char *hw_rev;       /* Hardware revision */
    const char *fw_rev;       /* Firmware revision */
    const char *date;         /* Current UTC date from GPS (YYYY-MM-DD) */
} log_ver_t;

/* GPS Fix sentence format (renamed from PTH):
 * $PFIX,<timestamp_ms>,<utc_time>,<lat>,<lon>,<alt>,<fix_quality>,<hdop>,<vdop>*HH
 * When printed, Lat/Lon should be include seven decimal digits of precision. Altitude to 1/100 meters.
 * Example: $PFIX,123456,12:34:56.789Z,37.7749000,-122.4194000,10.50,3,1.2,1.5*56
 */
typedef struct {
    uint32_t timestamp_ms;    /* Milliseconds since session start */
    const char *utc_time;     /* ISO8601 UTC time with ms (UTC, time-only) */
    double latitude;          /* Decimal degrees */
    double longitude;         /* Decimal degrees */
    float altitude;           /* Meters above MSL */
    uint8_t fix_quality;      /* 0=no fix, 1=2D, 2=3D, 3=3D+DGPS */
    float hdop;               /* Horizontal DOP */
    float vdop;               /* Vertical DOP */
    float speed;              /* Groundspeed (m/s) */
    float course;             /* Course over ground (i.e.,track made good) (degrees, true) */
} log_fix_t;


/* Session file config sentence format:
 * $PSFC,<session_id>,<start_time>,<rates>,<axes>*HH
 * Example: $PSFC,12345678,2025-01-15T12:34:56Z,40:4:1:1,NED*CD
 */
typedef struct {
    uint32_t session_id;      /* Unique session identifier */
    const char *start_time;   /* ISO8601 UTC time or "unknown" */
    uint16_t imu_rate_hz;     /* IMU output rate */
    uint16_t env_rate_hz;     /* Environmental data rate */
    uint16_t gnss_rate_hz;    /* GNSS rate */
    uint16_t mag_rate_hz;     /* Magnetometer rate (0 if disabled) */
    const char *axes;         /* Coordinate system (NED/ENU/etc) */
} log_sfc_t;

/* IMU sentence format:
 * $PIMU,<timestamp_ms>,<ax>,<ay>,<az>,<gx>,<gy>,<gz>*HH
 * Example: $PIMU,123456,1.23,-0.45,9.81,0.01,-0.02,0.03*EF
 * Units: accel in m/s^2, gyro in rad/s
 */
typedef struct {
    uint32_t timestamp_ms;    /* Milliseconds since session start */
    float ax, ay, az;         /* Accelerometer X, Y, Z */
    float gx, gy, gz;         /* Gyroscope X, Y, Z */
} log_imu_t;

/* IMU quaternion sentence format:
 * $PIM2,<timestamp_ms>,<qw>,<qx>,<qy>,<qz>*HH
 * Example: $PIM2,123456,1.0000,0.0000,0.0000,0.0000*12
 */
typedef struct {
    uint32_t timestamp_ms;    /* Milliseconds since session start */
    float qw, qx, qy, qz;     /* Quaternion components */
} log_im2_t;

/* Environmental sentence format:
 * $PENV,<timestamp_ms>,<pressure_pa>,<temp_c>,<altitude_m>*HH
 * Example: $PENV,123456,101325,25.3,0.0*34
 */
typedef struct {
    uint32_t timestamp_ms;    /* Milliseconds since session start */
    float pressure_pa;        /* Pressure in Pascals */
    float temperature_c;      /* Temperature in Celsius */
    float altitude_m;         /* Calculated altitude in meters */
} log_env_t;

/* Time hack sentence format:
 * $PTH,<timestamp_ms>,<utc_time>,<lat>,<lon>,<alt>,<fix_quality>*HH
 * Example: $PTH,123456,2025-01-15T12:34:56.789Z,37.7749,-122.4194,10.5,3*56
 */
typedef struct {
    uint32_t timestamp_ms;    /* Milliseconds since session start */
    const char *utc_time;     /* ISO8601 UTC time with ms */
    double latitude;          /* Decimal degrees */
    double longitude;         /* Decimal degrees */
    float altitude;           /* Meters above MSL */
    uint8_t fix_quality;      /* 0=no fix, 1=2D, 2=3D, 3=3D+DGPS */
} log_th_t;

/* State change sentence format:
 * $PST,<timestamp_ms>,<old_state>,<new_state>,<trigger>*HH
 * Example: $PST,123456,IDLE,ARMED,USER*78
 */
typedef struct {
    uint32_t timestamp_ms;    /* Milliseconds since session start */
    const char *old_state;    /* Previous state name */
    const char *new_state;    /* New state name */
    const char *trigger;      /* What triggered the change */
} log_st_t;

/* Magnetometer sentence format (optional):
 * $PMAG,<timestamp_ms>,<mx>,<my>,<mz>*HH
 * Example: $PMAG,123456,23.5,-12.3,45.6*9A
 * Units: microTesla
 */
typedef struct {
    uint32_t timestamp_ms;    /* Milliseconds since session start */
    float mx, my, mz;         /* Magnetic field X, Y, Z in uT */
} log_mag_t;

/* Helper functions */

/**
 * @brief Calculate NMEA checksum for a sentence
 * 
 * @param sentence Sentence string (without $ and *HH)
 * @param len Length of sentence
 * @return 8-bit XOR checksum
 */
uint8_t log_calc_checksum(const char *sentence, size_t len);

/**
 * @brief Format a complete sentence with checksum
 * 
 * @param buf Output buffer
 * @param buf_size Size of output buffer
 * @param fmt Printf-style format string (should start with $Pxxx,)
 * @param ... Format arguments
 * @return Number of characters written (excluding null terminator)
 */
int log_format_sentence(char *buf, size_t buf_size, const char *fmt, ...);

/**
 * @brief Get relative timestamp in milliseconds
 * 
 * @param session_start_us Session start time in microseconds
 * @return Milliseconds since session start
 */
uint32_t log_get_timestamp_ms(uint64_t session_start_us);

#endif /* APP_LOG_FORMAT_H */