/* 
 * This file is part of the Tempo distribution (https://github.com/rrainey/tempo)
 * Copyright (c) 2024 Riley Rainey
 * 
 * Portions of this file are based on example code that is part of the Adafruit 
 * SD FAT Arduino library
 * (adafruit/SdFat - Adafruit Fork@^2.2.3)
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

#include <Arduino.h>
#include <CombinedLogger.h>
#include <ICM42688.h>
#include <LogfileManager.h>
#include <LongerTimestamp.h>
#include <MMC5983MA.h>
#include <SPI.h>
#include <SdFat.h>
#include <bmp3.h>
#include <sdios.h>
#include <avr/dtostrf.h>

#include "tempo-arduino-pins.h"
#include "MorseBlinker.h"

/**
 * Tempo V1 board has a dedicated SPI interface for the SD card running at 16 MHz
 */
SPIClass spiflash(&PERIPH_SPI1, SDCARD_MISO_PIN, SDCARD_SCK_PIN,
                  SDCARD_MOSI_PIN, PAD_SPI1_TX, PAD_SPI1_RX);

/*
  Set DISABLE_CS_PIN to disable a second SPI device on a common SPI interface.
  For example, with the Ethernet shield, set DISABLE_CS_PIN
  to 10 to disable the Ethernet controller.
*/
const int8_t DISABLE_CS_PIN = -1;

// SDCARD_SS_PIN is defined for the built-in SD on the Tempo V1 board.

#ifndef SDCARD_SS_PIN
const uint8_t SD_CS_PIN = SS;
#else   // SDCARD_SS_PIN
const uint8_t SD_CS_PIN = SDCARD_SS_PIN;
#endif  // SDCARD_SS_PIN

// Automatically select the SD card configuration settings (SD_CONFIG)
#if HAS_SDIO_CLASS
#define SD_CONFIG SdioConfig(FIFO_SDIO)
#elif ENABLE_DEDICATED_SPI
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, DEDICATED_SPI, SD_SCK_MHZ(16), &spiflash)
#else  // HAS_SDIO_CLASS
#define SD_CONFIG SdSpiConfig(SD_CS_PIN, SHARED_SPI, SD_SCK_MHZ(16))
#endif  // HAS_SDIO_CLASS

//------------------------------------------------------------------------------
SdFs sd;
cid_t cid;
csd_t csd;
scr_t scr;
uint8_t cmd6Data[64];
uint32_t eraseSize;
uint32_t ocr;
static ArduinoOutStream cout(Serial);

/**
 * The CombinedLogger class drives the Tempo V1 logger application.
 */
CombinedLogger logger(sd);

//------------------------------------------------------------------------------
void printSdCardInfo() {
    cout << F("\nManufacturer ID: ");
    cout << uppercase << showbase << hex << int(cid.mid) << dec << endl;
    cout << F("OEM ID: ") << cid.oid[0] << cid.oid[1] << endl;
    cout << F("Product: ");
    for (uint8_t i = 0; i < 5; i++) {
        cout << cid.pnm[i];
    }
    cout << F("\nRevision: ") << cid.prvN() << '.' << cid.prvM() << endl;
    cout << F("Serial number: ") << hex << cid.psn() << dec << endl;
    cout << F("Manufacture date: ");
    cout << cid.mdtMonth() << '/' << cid.mdtYear() << endl;
    cout << endl;

    cout << F("\nCard type: ");

    switch (sd.card()->type()) {
        case SD_CARD_TYPE_SD1:
            cout << F("SD1\n");
            break;

        case SD_CARD_TYPE_SD2:
            cout << F("SD2\n");
            break;

        case SD_CARD_TYPE_SDHC:
            if (csd.capacity() < 70000000) {
                cout << F("SDHC\n");
            } else {
                cout << F("SDXC\n");
            }
            break;

        default:
            cout << F("Unknown\n");
    }
}
//------------------------------------------------------------------------------
void clearSerialInput() {
    uint32_t m = micros();
    do {
        if (Serial.read() >= 0) {
            m = micros();
        }
    } while (micros() - m < 10000);
}
//------------------------------------------------------------------------------
void errorPrint() {
    if (sd.sdErrorCode()) {
        cout << F("SD errorCode: ") << hex << showbase;
        printSdErrorSymbol(&Serial, sd.sdErrorCode());
        cout << F(" = ") << int(sd.sdErrorCode()) << endl;
        cout << F("SD errorData = ") << int(sd.sdErrorData()) << dec << endl;
    }
}
//------------------------------------------------------------------------------
void printSdVolumeInfo() {
    cout << F("\nScanning filesystem\n");
    int32_t freeClusterCount = sd.freeClusterCount();
    if (sd.fatType() <= 32) {
        cout << F("\nVolume is FAT") << int(sd.fatType()) << endl;
    } else {
        cout << F("\nVolume is exFAT\n");
    }
    cout << F("sectorsPerCluster: ") << sd.sectorsPerCluster() << endl;
    cout << F("fatStartSector:    ") << sd.fatStartSector() << endl;
    cout << F("dataStartSector:   ") << sd.dataStartSector() << endl;
    cout << F("clusterCount:      ") << sd.clusterCount() << endl;
    cout << F("freeClusterCount:  ");
    if (freeClusterCount >= 0) {
        cout << freeClusterCount << endl;
    } else {
        cout << F("failed\n");
        errorPrint();
    }
}

uint32_t tStart_ms = 0;

void setup() {

    delay(1000);

    tStart_ms = millis();

    Serial.begin(115200);

    // wait for Serial to connect.
    // Press on if that doesn't happen within ten seconds.
    while ( !Serial && (millis() - tStart_ms) < 10000 ) {
        delay(100);
    }

    if (OPS_MODE == OPS_STATIC_TEST) {
        Serial.println("Welcome. The device has booted in OPS_STATIC_TEST mode.");
        Serial.println("");
        Serial.println("This test will take about 20 minutes to complete. You will see");
        Serial.println("state change messages for STATE_WAIT, STATE_IN_FLIGHT, and STATE_LANDED_1. ");
        Serial.println("The test is complete when the device returns to STATE_WAIT.");
        Serial.println("You may then inspect the information in the last log file generated.");
        Serial.println("---");
    }

    tStart_ms = millis();

    // Configure LEDs and other GPIO pins
    pinMode(RED_LED, OUTPUT);
    pinMode(GREEN_LED, OUTPUT);
    pinMode(ICM42688_intPin1, INPUT);
    pinMode(ICM42688_intPin2, INPUT);
    pinMode(MMC5983MA_intPin, INPUT);
    pinMode(BMP390_intPin, INPUT);

    // Signal start of configuration
    digitalWrite(RED_LED, HIGH);

    if (!sd.cardBegin(SD_CONFIG)) {
        cout << F(
            "\nSD initialization failed.\n"
            "Do not reformat the card!\n"
            "Is the card correctly inserted?\n"
            "Is there a wiring/soldering problem?\n");
        if (isSpi(SD_CONFIG)) {
            cout << F(
                "Is SD_CS_PIN set to the correct value?\n"
                "Does another SPI device need to be disabled?\n");
        }

        // BlinkState::BLINK_STATE_NO_SDCARD
        logger.setBlinkState(BlinkState::BLINK_STATE_NO_SDCARD);

        // loop forever
        while (true) {
            delay(10);
            logger.getBlinker().loop();
        }
    }

    if (!sd.card()->readCID(&cid) || !sd.card()->readCSD(&csd) ||
        !sd.card()->readOCR(&ocr) || !sd.card()->readSCR(&scr)) {
        cout << F("readInfo failed\n");
        errorPrint();

        logger.setBlinkState(BlinkState::BLINK_STATE_NO_SDCARD);

        // loop forever
        while (true) {
            delay(10);
            logger.getBlinker().loop();
        }
    }

    printSdCardInfo();

    cout << F("sdSpecVer: ") << 0.01 * scr.sdSpecVer() << endl;
    cout << F("HighSpeedMode: ");
    if (scr.sdSpecVer() && sd.card()->cardCMD6(0X00FFFFFF, cmd6Data) &&
        (2 & cmd6Data[13])) {
        cout << F("true\n");
    } else {
        cout << F("false\n");
    }

    if (!sd.volumeBegin()) {
        cout << F("\nvolumeBegin failed. Is the card formatted?\n");
        errorPrint();

        logger.setBlinkState(BlinkState::BLINK_STATE_BAD_FILESYSTEM);

        while (true) {
            delay(10);
            logger.getBlinker().loop();
        }
    }

    printSdVolumeInfo();

    /*
     * Configure sensor peripherals for operation and begin looking for
     * conditions that would indicate that a jump has started (i.e., that the aircraft has
     * lifted off from the runway)
     */

    if (logger.configureSensors() != BinaryLogger::APIResult::Success) {
        Serial.println("Could not configure sensors; halting");

        logger.setBlinkState(BlinkState::BLINK_STATE_INIT_FAILED);

        while (true) {
            delay(10);
            logger.getBlinker().loop();
        }
    }

    logger.setBlinkState(BlinkState::BLINK_STATE_IDLE);
}

void loop() {

    /*
     * Check sensors and update the logger's
     * state machine
     */
    logger.loop();

}