# tempo-logger
### Log jumps in both Dropkick and raw sensor data files

The tempo board has two dedicated SPI interfaces.  The first is used to connect the
ICM42688V IMU IC.  The second connects to the SD Card interface.

The SAM-M10Q GNSS receiver, MMC5983MA Magenetometer, and BMP390 Barometer/Temp sensors are connected on a shared I2C interface which is driven at 100Kbps.

## Program execution

This application configures the sensor peripherals then looks for conditions indicating that a skydive is starting. The application then begins logging the jump.  Two log files area created.

The application will stop logging after it detects that the jumper has landed. It closes the log files and then returns to a state where it looks for the next jump.

It the program detects that it cannot keep up with logging the data to the SD card at any time, it will close the log file and stop.

## Dropkick log file format

This log file is an extension of the NMEA 0183 GNSS log file format.  It is described in the Dropkick project.

## Binary Log File Format

A custom binary log file format is created by the application. It is composed of a sequence of variable-length records, each composed of an 18-byte fixed section and a variable section of up to 255 bytes.

| Field  | data type | Description |
|------|-------------|-------------|
|  timestamp   | 128-bit unsigned integer   | microseconds since start of log   |
|  record type    | 8-bit unsigned integer | see below    |
|  variable section length    | 8-bit unsigned integer | byte length of the variable data section (0-255) |
|  variable data section  | dependent on record type | |


### Record types

| Code  | Description | Length field |
|------|-------------|-------------|
|  1    | File version    | 1 (constant); this version of the application inserts 0x00 as the version     |
|  2    | IMU FIFO Record | 27 (constant)    |
|  3    | GNSS NMEA sentence | variable (max 100) |
|  4    | Magnetometer FIFO record | ?|
|  5    | Barometer/Temp Sample | ? |

## SD Card bandwidth

The SD Card interface is driven at 16Mbps, making the maximum theoretical throughput something like 2 megabytes per second. The practical limit is probably half or less of that value.