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
#define PIN_SPI1_MISO         (35u)
#define PIN_SPI1_MOSI         (36u)
#define PIN_SPI1_SCK          (33u)
#define PERIPH_SPI1           sercom0
#define PAD_SPI1_TX           SPI_PAD_0_SCK_1
#define PAD_SPI1_RX           SERCOM_RX_PAD_3

/*
 * This represents the SPI1 interface on the Tempo board.  SPI1 is dedicated to the SD Card peripheral.
 *
 * We run the test at a 16MHz clock speed.
 */
#define SDCARD_MOSI_PIN         PIN_SPI1_MOSI
#define SDCARD_MISO_PIN         PIN_SPI1_MISO
#define SDCARD_SCK_PIN          PIN_SPI1_SCK
#define SDCARD_SS_PIN           (34u)

/**
 * tempo board connections
 * 
 * BMP390    INT       D11
 * MMC5983MA INT       D9  
 * ICM42688  INT/INT1  D6  
 * ICM42688  INT2      D5  
 * 
 */
#define RED_LED           (13u)
#define GREEN_LED         (12u)

#define ICM42688_intPin1 digitalPinToInterrupt(6)   // INT/INT1
#define MMC5983MA_intPin digitalPinToInterrupt(9)
#define BMP390_intPin    digitalPinToInterrupt(11)  // 11 for tempo, 10 for Peakick
#define ICM42688_intPin2 digitalPinToInterrupt(10)  // INT2/CLKIN as an interrupt line
#define CLKOUT            (10u)                     // as a SAMD51 output line
#define ICM42688_SPI_CS   (5u)                      // Chip Select for IMU

#define GPS_I2C_ADDR      0x42  // SAM-M10Q
#define BMP390_I2C_ADDR   0x76