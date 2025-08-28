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
    const char *star;
    uint8_t calc_checksum;
    uint8_t sent_checksum;
    
    if (!sentence || len < 8) {  /* Minimum: $X*HH\r\n */
        return 0;
    }
    
    /* Find the * character */
    star = memchr(sentence, '*', len);
    if (!star || (star - sentence) < 2 || (len - (star - sentence)) < 3) {
        return 0;
    }
    
    /* Calculate checksum of payload between $ and * */
    size_t payload_len = star - sentence;
    if (sentence[0] == '$') {
        payload_len--;
    }
    calc_checksum = nmea_checksum(sentence, payload_len);
    
    /* Parse the sent checksum */
    char hex[3] = {star[1], star[2], '\0'};
    char *endptr;
    sent_checksum = (uint8_t)strtol(hex, &endptr, 16);
    if (endptr != &hex[2]) {
        return 0;  /* Invalid hex digits */
    }
    
    return (calc_checksum == sent_checksum) ? 1 : 0;
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