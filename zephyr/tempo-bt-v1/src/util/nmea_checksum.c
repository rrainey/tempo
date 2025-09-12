/*
 * SPDX-License-Identifier: Apache-2.0
 * 
 * Tempo-BT V1 - NMEA Checksum Utilities
 */

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include "util/nmea_checksum.h"

uint8_t nmea_checksum(const char *payload, size_t n)
{
    uint8_t checksum = 0;
    
    /* Skip the $ if present */
    if (n > 0 && payload[0] == '$') {
        payload++;
        n--;
    }
    
    /* Calculate XOR checksum up to * or end of string */
    for (size_t i = 0; i < n; i++) {
        if (payload[i] == '*') {
            break;
        }
        checksum ^= (uint8_t)payload[i];
    }
    
    return checksum;
}

int nmea_verify_checksum(const char *sentence, size_t len)
{
    uint8_t calc_checksum = 0;

    if (len < 8 || sentence[0] != '$') {
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
        return false;
    }
    
    /* Parse provided checksum */
    char checksum_str[3] = {p[1], p[2], '\0'};

    uint8_t provided_checksum = (uint8_t)strtol(checksum_str, NULL, 16);
    
    return calc_checksum == (uint8_t)provided_checksum;
}

int nmea_append_checksum(char *sentence, size_t buf_size)
{
    size_t len;
    uint8_t checksum;
    
    if (!sentence || buf_size < 6) {  /* Need room for at least "*HH\r\n" */
        return -1;
    }
    
    len = strlen(sentence);
    if (len + 5 > buf_size) {  /* No room for checksum + CRLF */
        return -1;
    }
    
    /* Calculate checksum */
    checksum = nmea_checksum(sentence, len);
    
    /* Append checksum and CRLF */
    snprintf(sentence + len, buf_size - len, "*%02X\r\n", checksum);
    
    return 0;
}