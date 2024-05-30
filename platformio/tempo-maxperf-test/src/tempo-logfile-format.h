/* 
 * This file is part of the Tempo distribution (https://github.com/rrainey/tempo)
 * Copyright (c) 2024 Riley Rainey
 * 
 * This program is free software: you can redistribute it and/or modify  
 * it under the terms of the GNU General Public License as published by  
 * the Free Software Foundation, version 3.
 *
 * This program is distributed in the hope that it will be useful, but 
 * WITHOUT ANY WARRANTY; without even the implied warranty of 
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU 
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License 
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 */
#ifndef tempo_logfile_format_h
#define tempo_logfile_format_h

enum class TempoRawRecordType { Version = 0, NMEA = 1, IMU = 2, Magnetometer = 3, Barometer = 4 };

#define TEMPO_RAW_FILE_VERSION 1
#define TEMPO_RAW_FILE_HEADER_SIZE 10

/**
 * Tempo Raw Log Record definition
 * 
 * This is defined as a simple C structure to retain control over byte alignment and field sizing
 */
typedef struct {
    // storage for a LongerTimestamp
    unsigned long long timestamp;
    // stores TempoRawRecordType, but we need to ensure it is an 8-bit enum
    unsigned char type; 
    // count of bytes that follow (based on type)
    unsigned char variablePartLength;
    union {
        unsigned char versionNumber;
        // NMEA sentence; shortened to remove <CR><LF><NULL> bytes
        char nmeaRecord[256];
        struct {
            short gyro[3];
            short acc[3];
            unsigned short timestamp;
            unsigned char temp;
            unsigned char header;
        } imuSample;
        struct {
            unsigned long mag[3];
        } magSample;
        struct {
            double pressure_Pa;
            double temp_degC;
        } baroSample;
    } variablePart;

} tempoRawLogRecord;

#endif