/*
 * SPDX-License-Identifier: Apache-2.0
 * 
 * Tempo-BT V1 - NMEA Checksum Unit Tests
 */

#include <zephyr/ztest.h>
#include <string.h>
#include "util/nmea_checksum.h"

ZTEST_SUITE(nmea_checksum_tests, NULL, NULL, NULL, NULL, NULL);

ZTEST(nmea_checksum_tests, test_basic_checksum)
{
    /* Test vectors with known checksums */
    struct {
        const char *payload;
        uint8_t expected;
    } vectors[] = {
        /* Standard NMEA sentences */
        {"GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,", 0x47},
        {"GPVTG,054.7,T,034.4,M,005.5,N,010.2,K", 0x48},
        {"GPRMC,081836,A,3751.65,S,14507.36,E,000.0,360.0,130998,011.3,E", 0x62},
        
        /* Tempo custom sentences */
        {"PVER,1.0,V1,0.1.0,2025-01-17", 0x1C},
        {"PIMU,1000,1.23,4.56,7.89,0.123,0.456,0.789", 0x36},
        {"PENV,1000,1013.25,20.5,100.0", 0x7C},
        {"PTH,1234567", 0x33},
        {"PST,1000,IDLE,ARMED,button", 0x5E},
        
        /* Edge cases */
        {"", 0x00},  /* Empty payload */
        {"A", 0x41}, /* Single character */
        {"AB", 0x03}, /* Two characters XOR */
    };
    
    for (size_t i = 0; i < ARRAY_SIZE(vectors); i++) {
        uint8_t result = nmea_checksum(vectors[i].payload, strlen(vectors[i].payload));
        zassert_equal(result, vectors[i].expected,
                      "Checksum mismatch for '%s': got 0x%02X, expected 0x%02X",
                      vectors[i].payload, result, vectors[i].expected);
    }
}

ZTEST(nmea_checksum_tests, test_with_dollar_prefix)
{
    const char *sentence1 = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,";
    const char *sentence2 = "GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,";
    
    /* Both should produce same checksum */
    uint8_t cs1 = nmea_checksum(sentence1, strlen(sentence1));
    uint8_t cs2 = nmea_checksum(sentence2, strlen(sentence2));
    
    zassert_equal(cs1, cs2, "Dollar prefix handling failed");
    zassert_equal(cs1, 0x47, "Incorrect checksum value");
}

ZTEST(nmea_checksum_tests, test_with_star_terminator)
{
    const char *sentence = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47";
    
    /* Should stop at * */
    uint8_t cs = nmea_checksum(sentence, strlen(sentence));
    zassert_equal(cs, 0x47, "Star terminator not handled correctly");
}

ZTEST(nmea_checksum_tests, test_verify_checksum_valid)
{
    /* Valid sentences */
    const char *valid[] = {
        "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47\r\n",
        "$GPVTG,054.7,T,034.4,M,005.5,N,010.2,K*48\r\n",
        "$PVER,1.0,V1,0.1.0,2025-01-17*1C\r\n",
        "$PIMU,1000,1.23,4.56,7.89,0.123,0.456,0.789*36\r\n",
    };
    
    for (size_t i = 0; i < ARRAY_SIZE(valid); i++) {
        bool result = nmea_verify_checksum(valid[i], strlen(valid[i]));
        zassert_true(result, "Valid sentence '%s' failed verification", valid[i]);
    }
}

ZTEST(nmea_checksum_tests, test_verify_checksum_invalid)
{
    /* Invalid sentences */
    const char *invalid[] = {
        "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*48\r\n",  /* Wrong checksum */
        "$GPVTG,054.7,T,034.4,M,005.5,N,010.2,K*4G\r\n",  /* Invalid hex digit */
        "$PVER,1.0,V1,0.1.0,2025-01-17\r\n",  /* Missing checksum */
        "PIMU,1000,1.23,4.56,7.89*36\r\n",  /* Missing $ */
        "$PENV*\r\n",  /* Too short */
        "",  /* Empty */
    };
    
    for (size_t i = 0; i < ARRAY_SIZE(invalid); i++) {
        bool result = nmea_verify_checksum(invalid[i], strlen(invalid[i]));
        zassert_false(result, "Invalid sentence '%s' passed verification", invalid[i]);
    }
}

ZTEST(nmea_checksum_tests, test_append_checksum)
{
    char buffer[128];
    int ret;
    
    /* Test successful append */
    strcpy(buffer, "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,");
    ret = nmea_append_checksum(buffer, sizeof(buffer));
    zassert_equal(ret, 0, "Failed to append checksum");
    zassert_mem_equal(buffer + strlen(buffer) - 5, "*47\r\n", 5,
                      "Incorrect checksum appended");
    
    /* Test with Tempo sentence */
    strcpy(buffer, "$PIMU,1000,1.23,4.56,7.89,0.123,0.456,0.789");
    ret = nmea_append_checksum(buffer, sizeof(buffer));
    zassert_equal(ret, 0, "Failed to append checksum");
    zassert_true(nmea_verify_checksum(buffer, strlen(buffer)),
                 "Appended checksum failed verification");
    
    /* Test buffer too small */
    strcpy(buffer, "$PVER,1.0,V1,0.1.0,2025-01-17");
    ret = nmea_append_checksum(buffer, strlen(buffer) + 1);  /* No room for checksum */
    zassert_equal(ret, -1, "Should fail with small buffer");
}

ZTEST(nmea_checksum_tests, test_checksum_special_chars)
{
    /* Test with special characters and high-bit values */
    const char *special = "Test,\x80\xFF\x01\x7F";
    uint8_t cs = nmea_checksum(special, strlen(special));
    
    /* Manually calculate: T^e^s^t^,^0x80^0xFF^0x01^0x7F */
    uint8_t expected = 'T' ^ 'e' ^ 's' ^ 't' ^ ',' ^ 0x80 ^ 0xFF ^ 0x01 ^ 0x7F;
    zassert_equal(cs, expected, "Special character handling failed");
}

ZTEST(nmea_checksum_tests, test_tempo_sentences)
{
    /* Test all Tempo-specific sentence types */
    struct {
        char sentence[128];
        const char *expected_end;
    } tempo_tests[] = {
        {"$PVER,1.0,V1,0.1.0,2025-01-17", "*1C\r\n"},
        {"$PSFC,12345,2025-01-17T12:00:00Z,40:4:10:0,NED", "*6E\r\n"},
        {"$PIMU,1000,9.81,0.00,0.00,0.0000,0.0000,0.0000", "*71\r\n"},
        {"$PIM2,1000,1.0000,0.0000,0.0000,0.0000", "*7C\r\n"},
        {"$PENV,1000,1013.25,20.5,100.0", "*7C\r\n"},
        {"$PTH,1234567890", "*08\r\n"},
        {"$PST,1000,WAIT,FLIGHT,takeoff_detected", "*47\r\n"},
        {"$PMAG,1000,23.45,-12.34,45.67", "*43\r\n"},
    };
    
    for (size_t i = 0; i < ARRAY_SIZE(tempo_tests); i++) {
        int ret = nmea_append_checksum(tempo_tests[i].sentence, 
                                       sizeof(tempo_tests[i].sentence));
        zassert_equal(ret, 0, "Failed to append checksum to %s", 
                      tempo_tests[i].sentence);
        
        /* Verify the checksum matches expected */
        size_t len = strlen(tempo_tests[i].sentence);
        zassert_mem_equal(tempo_tests[i].sentence + len - 5,
                          tempo_tests[i].expected_end, 5,
                          "Wrong checksum for sentence %zu", i);
        
        /* Double-check with verify function */
        zassert_true(nmea_verify_checksum(tempo_tests[i].sentence, len),
                     "Verification failed for sentence %zu", i);
    }
}