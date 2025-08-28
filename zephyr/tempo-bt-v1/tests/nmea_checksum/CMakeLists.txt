cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(nmea_checksum_test)

target_sources(app PRIVATE
    ../../src/util/nmea_checksum.c
    src/test_nmea_checksum.c
)

target_include_directories(app PRIVATE 
    ../../include
)