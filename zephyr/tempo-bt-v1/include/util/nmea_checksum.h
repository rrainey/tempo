/*
 * SPDX-License-Identifier: Apache-2.0
 * 
 * Tempo-BT V1 - NMEA Checksum Utilities
 */

#ifndef UTIL_NMEA_CHECKSUM_H
#define UTIL_NMEA_CHECKSUM_H

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

/**
 * @brief Calculate NMEA checksum for a payload
 * 
 * Calculates the XOR checksum of all bytes in the payload.
 * If the payload starts with '$', it is skipped.
 * If a '*' is encountered, checksum calculation stops there.
 * 
 * @param payload Payload string  
 * @param n Length of payload
 * @return Calculated checksum byte
 */
uint8_t nmea_checksum(const char *payload, size_t n);

/**
 * @brief Verify checksum of a complete NMEA sentence
 * 
 * Checks if the sentence has a valid format ($...*HH) and
 * if the checksum matches.
 * 
 * @param sentence Complete NMEA sentence including $ and *HH
 * @param len Length of sentence
 * @return 1 if checksum is valid, 0 otherwise
 */
int nmea_verify_checksum(const char *sentence, size_t len);

/**
 * @brief Append checksum and CRLF to NMEA sentence
 * 
 * Takes a sentence without checksum and appends *HH\r\n
 * 
 * @param sentence Buffer containing sentence (modified in place)
 * @param buf_size Size of buffer
 * @return 0 on success, -1 on error (buffer too small)
 */
int nmea_append_checksum(char *sentence, size_t buf_size);

#endif /* UTIL_NMEA_CHECKSUM_H */