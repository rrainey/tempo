This aplication tests the SD Card interface and reformats the card -- if present.

It is based on an example from the Adafruit SdFat library fork, version 2.2.3.

The SD Card is connected via a second SPI interface which is separate from the
SPI interface used for the ICM42688 IMU.

The application interactions will look something like this:

```
---- Opened the serial port COM17 ----
SdFat version: 2.2.0

Assuming the SD is the only SPI device.
Edit DISABLE_CS_PIN to disable an SPI device.

Assuming the SD chip select pin is: 34
Edit SD_CS_PIN to change the SD chip select pin.

type any character to start
---- Sent utf8 encoded message: "x" ----
init time: 155 ms

Card type: SDXC
sdSpecVer: 6.00
HighSpeedMode: true

Manufacturer ID: 0X9F
OEM ID: TI
Product: ASTC
Revision: 0.0
Serial number: 0X3F1
Manufacturing date: 9/2022

cardSize: 62534.98 MB (MB = 1,000,000 bytes)
flashEraseSize: 128 blocks
eraseSingleBlock: true
dataAfterErase: ones

OCR: 0XC0FF8000

SD Partition Table
part,boot,bgnCHS[3],type,endCHS[3],start,length
1,0X80,0XFE,0XFF,0XFF,0XB,0XFE,0XFF,0XFF,2048,122136576
2,0X0,0X0,0X0,0X0,0X0,0X0,0X0,0X0,0,0
3,0X0,0X0,0X0,0X0,0X0,0X0,0X0,0X0,0,0
4,0X0,0X0,0X0,0X0,0X0,0X0,0X0,0X0,0,0

Scanning FAT, please wait.

Volume is FAT32
sectorsPerCluster: 64
fatStartSector:    2080
dataStartSector:   31892
clusterCount:      1907917
freeClusterCount:  1907825

type any character to start
```