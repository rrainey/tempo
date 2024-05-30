This aplication tests the throughput of the SD Card interface by continoursly logging
samples from all periperal ICs.

It is based on an example from the Adafruit SdFat library fork, version 2.2.3.

The tempo board has two dedicated SPI interfaces.  The first is used to connect the
ICM42688V IMU IC.  The second connects to the SD Card interface.

The SAM-M10Q GNSS receiver, MMC5983MA Magenetometer, and BMP390 Barometer/Temp sensors are connected on a shared I2C interface which is driven at 100Kbps.

## Program execution

This application simply initializes and configures the peripherals. It then begins logging.  The logged data goes to the log file basically in the same form as it was delivered by the peripheral.  Interpretation of the data would require postprocessing software, which is yet to be written.

The application will stop logging and close the file if a character is received on the USB serial interface, or if a fixed timer expires (this timer is normally set to expire after 60 seconds).

It the program detects that it cannot keep up with logging the data to the SD card at any time, it will close the log file and stop.

## Log File Format

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

The SD Card interface is driven at 16Mbps, making the maximum theoritical throughput something like 2.0 megabytes per second. The practical limit is probably half or less of that value.