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
#include <Arduino.h>
#include <SdFat.h>
#include <sdios.h>
#include <bmp3.h>
#include <ICM42688.h>
#include <MMC5983MA.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>

#include "LongerTimestamp.h"
#include "LongerTimestampQueue.h"
#include "BinaryLogger.h"
#include "tempo-arduino-pins.h"
#include "tempo-logfile-format.h"

SFE_UBLOX_GNSS gnss;
ICM42688 imu(SPI, ICM42688_SPI_CS);
MMC5983MA mmc(MMC5983MA_ADDRESS, &Wire); 

#include <MicroNMEA.h>
char nmeaBuffer[100];
MicroNMEA nmea(nmeaBuffer, sizeof(nmeaBuffer));

#define CPU_SUPPLIES_IMU_CLKIN false

/**
 * Interrupt service routines, flags, and supporting data
 */

static volatile unsigned short imuUnservicedISRCount = 0;
static LongerTimestampQueue imuTimestamps;
static LongerTimestamp last;

void imuInterruptHandler() { 
    noInterrupts();
    imuTimestamps.push( last.getLongMicros() );
    interrupts();
    ++imuUnservicedISRCount; 
}

static volatile bool newMMC5983MAData = false;
static volatile uint32_t mmcIntCount = 0;

void magInterruptHandler() { 
    newMMC5983MAData = true; 
    ++ mmcIntCount;
}

static volatile bool pressureSampleAvailable = false;

void BMPSampleReadyHandler() { 
    pressureSampleAvailable = true; 
}

BinaryLogger *pSingleton;

/**
 * End of Interrupt service routines and flags
 */

BinaryLogger::BinaryLogger(SdFs &Sd) : logManager(&Sd) {
    imuInvalidSamples = 0;
    imuFifoEmpty = 0;
    imuISROverflow = 0;
    imuIntCount = 0;
    imuFIFOProcessed = 0;
    loopCount = 0;
    loopTotal = 0;

    state = OperatingState::Initialized;

    usGPSDate = 0;
    usGPSTime = 0;

    pSingleton = this;

    logfileDateSet = false;
}

char incomingNMEA[256];
char *pNMEA = incomingNMEA;
bool bStartOfNMEA = true;

void SFE_UBLOX_GNSS::processNMEA(char incoming) {
    pSingleton->processNMEAx(incoming);
}

BinaryLogger::APIResult BinaryLogger::configureSensors() {

    uint8_t Ascale = AFS_4G;
    uint8_t Gscale = GFS_250DPS;
    uint8_t AODR = AODR_200Hz;
    uint8_t GODR = GODR_200Hz;

    icm42688AccelPowerMode aMode = aMode_LN;
    icm42688GyroPowerMode gMode = gMode_LN;

    mmc5983ma_modr_t MODR = MODR_100Hz;
    mmc5983ma_bandwidth_t MBW = MBW_100Hz;
    mmc5983ma_sr_Interval_t MSET = MSET_2000;


    Wire.begin();          
    Wire.setClock(100000);  // I2C frequency at 100 kHz

    SPI.begin();
    imu.begin();

    gnss.begin();

    gnss.setUART1Output(0);
    gnss.setUART2Output(0);

    gnss.setI2COutput(COM_TYPE_NMEA);
    gnss.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Save (only) the communications port settings to flash and BBR

    // Idle reporting will be at 0.5 Hz
    gnss.setMeasurementRate(2000);
    gnss.setNavigationRate(1);

    if (gnss.setDynamicModel(DYN_MODEL_AIRBORNE2g) == false) 
    {
        Serial.println(F("Warning: GNSS setDynamicModel() failed"));
        return APIResult::GenericError;
    }
    else
    {
        Serial.println(F("GNSS Dynamic Platform Model set to AIRBORNE2g"));
    }

    //This will pipe all NMEA sentences to the serial port so we can see them
    //myGNSS.setNMEAOutputPort(Serial);

    gnss.setNavigationFrequency(1);

    imu.reset();
    mmc.reset();

    Serial.println("ICM42688");
    byte ICM42688ID = imu.getChipID(); 

    Serial.println("MMC5983MA");
    byte MMC5983ID = mmc.getChipID();

    Serial.println("BMP390");
    bmp3_begin(&dev, BMP390_I2C_ADDR, &Wire);
    uint8_t BMP390ID = dev.chip_id;

    bool allSensorsAcknowledged =
        (ICM42688ID == 0x47 || ICM42688ID == 0xDB) && MMC5983ID == 0x30 && BMP390ID == 0x60;

    if (allSensorsAcknowledged) {

        logfileDateSet = false;

        Serial.println(" ");
        Serial.println("All peripherals are operational");
        Serial.println(" ");

        mmc.reset();

        mmc.performSetOperation();

        attachInterrupt(MMC5983MA_intPin, magInterruptHandler, RISING);

        //mmc.setSoftIronCalibration(softIronOffset, softIronScale);

        mmc.startSampleCollection(MODR, MBW, MSET);

        // Configure IMU

        attachInterrupt(ICM42688_intPin1, imuInterruptHandler, RISING); 

        imu.reset();

        imu.init( Ascale, Gscale, AODR, GODR, aMode, gMode, CPU_SUPPLIES_IMU_CLKIN );

        // Receive IMU samples via FIFO
        if (ICM42688_RETURN_OK != imu.startFifoSampling( 3 )) {
            Serial.println("enableFifoMode() failed");
            return APIResult::SensorFault;
        }
        delay (10);

        // configure Barometer

        bmp3_settings settings;

        // initialize BMP390 per datasheet, section 3.4.1 for "Drone"
        rslt = bmp3_init(&dev);
        if (rslt != BMP3_OK) {                             
            Serial.println("Error returned from " "bmp3_init");
            return APIResult::SensorFault;
        }

        settings.int_settings.drdy_en = BMP3_ENABLE;
        settings.press_en = BMP3_ENABLE;
        settings.temp_en = BMP3_ENABLE;

        settings.odr_filter.press_os = BMP3_OVERSAMPLING_8X;
        settings.odr_filter.temp_os = BMP3_NO_OVERSAMPLING;
        settings.odr_filter.odr = BMP3_ODR_12_5_HZ;
        settings.odr_filter.iir_filter = BMP3_IIR_FILTER_COEFF_3;
      
        uint32_t settings_sel = BMP3_SEL_PRESS_EN | BMP3_SEL_TEMP_EN |
                       BMP3_SEL_PRESS_OS | BMP3_SEL_TEMP_OS | BMP3_SEL_ODR |
                       BMP3_SEL_DRDY_EN | BMP3_SEL_IIR_FILTER;

        rslt = bmp3_set_sensor_settings(settings_sel, &settings, &dev);
        if (rslt != BMP3_OK) {                             
            Serial.println("Error returned from " "bmp3_set_sensor_settings");
            return APIResult::SensorFault;
        }

        attachInterrupt(BMP390_intPin, BMPSampleReadyHandler, RISING);

        settings.op_mode = BMP3_MODE_NORMAL;
        rslt = bmp3_set_op_mode(&settings, &dev);
        if (rslt != BMP3_OK) {                             
            Serial.println("Error returned from " "bmp3_set_op_mode");
            return APIResult::SensorFault;
        }

        setOperatingState( OperatingState::Idle );

    }
    else {
        return APIResult::SensorFault;
    }

    return APIResult::Success; 
}

BinaryLogger::APIResult BinaryLogger::startLogging(LogfileSlotID slot) {

    // create and open the binary log file
    if (logManager.openLogfile(slot, "TBS", tbsLogFile) != LogfileManager::APIResult::Success) {
        return APIResult::CannotCreateLogfile;
    }

    setOperatingState( OperatingState::Running );

    return APIResult::Success;
}

void BinaryLogger::stopLogging() {

    if (getOperatingState() == OperatingState::Running) {

        imu.reset();
        mmc.reset();
        bmp3_soft_reset(&dev);

        tbsLogFile.truncate();
        tbsLogFile.sync();
        tbsLogFile.close();
        setOperatingState( OperatingState::Idle );
        
    }
}

void BinaryLogger::loop() {

    ++loopCount;
    ++loopTotal;

    pollGNSS();

    pollMagnetometer();

    processIMUInterrupts();

    processBarometerInterrupts();
}

void BinaryLogger::processBarometerInterrupts() {

    // Pressure sample available?

    if (pressureSampleAvailable) {
        pressureSampleAvailable = false;
        bmp3_status status;

        rslt = bmp3_get_status(&status, &dev);
        if (rslt != 0) {
            // log an error
        }

        /* Read temperature and pressure data iteratively based on data ready
         * interrupt */
        if ((rslt == BMP3_OK) && (status.intr.drdy == BMP3_ENABLE)) {
            
            rslt = bmp3_get_sensor_data(BMP3_PRESS_TEMP, &data, &dev);
            if (rslt != 0) {
                // log an error
            }

            /* NOTE : Read status register again to clear data ready interrupt
             * status */
            rslt = bmp3_get_status(&status, &dev);
            if (rslt != 0) {
                // log an error
            }

            // allow derived class to process the barometer sample
            handleBaroSample(&data);

            // write record to binary log file
            writeBaroRecord(data.pressure, data.temperature);

        }
    }
}

void BinaryLogger::processIMUInterrupts() {
    // IMU Sample available?

    if (imuUnservicedISRCount > 0) {
        longTime_t itime_us = 0;

        noInterrupts();
        int res = imuTimestamps.pop(&itime_us);
        interrupts();

        if (res == 0) {
            Serial.println("assertion error: no timestamp in queue");
        }

        if (imuUnservicedISRCount > 1) {
            ++imuISROverflow;
        }
        imuUnservicedISRCount = 0;
        ++imuIntCount;

        // Read status bits to clear interrupt (see definition of INT_CONFIG0)
        uint8_t intStatus = imu.readIntStatus();
        (void)intStatus;

        uint8_t status;
        icm42688::fifo_packet3 sampleBuf[140], *pBuf;
        uint16_t packetCount;

        status = imu.readFiFo(sampleBuf, &packetCount);
        if (status != ICM42688_RETURN_OK) {
            Serial.println("readFifo encountered an error");
        }

        imuFIFOProcessed += packetCount;

        for (int i = 0; i < packetCount; ++i) {
            pBuf = &sampleBuf[i];

            // valid sample?

            if ((pBuf->header & ICM42688_FIFO_HEADER_MSG) == 0) {
                if ((pBuf->header & (ICM42688_FIFO_HEADER_ACCEL |
                                     ICM42688_FIFO_HEADER_GYRO)) ==
                    (ICM42688_FIFO_HEADER_ACCEL | ICM42688_FIFO_HEADER_GYRO)) {
                    if (!(PACKET3_SAMPLE_MARKED_INVALID(pBuf->gx) ||
                          PACKET3_SAMPLE_MARKED_INVALID(pBuf->gy) ||
                          PACKET3_SAMPLE_MARKED_INVALID(pBuf->gz) ||
                          PACKET3_SAMPLE_MARKED_INVALID(pBuf->ax) ||
                          PACKET3_SAMPLE_MARKED_INVALID(pBuf->ay) ||
                          PACKET3_SAMPLE_MARKED_INVALID(pBuf->az))) {
                        ++imuFIFOProcessed;

                        // Allow any derived class to process the IMU sample
                        handleIMUSample(itime_us, pBuf);

                        unsigned long gyro[3];
                        unsigned long acc[3];
                        gyro[0] = pBuf->gx;
                        gyro[1] = pBuf->gy;
                        gyro[2] = pBuf->gz;
                        acc[0] = pBuf->ax;
                        acc[1] = pBuf->ay;
                        acc[2] = pBuf->az;

                        writeImuRecord(gyro, acc, pBuf->header, pBuf->timestamp,
                                       pBuf->temp);

                    } else {
                        imuInvalidSamples++;
                    }
                } else {
                    imuInvalidSamples++;
                }
            } else {
                imuFifoEmpty++;
            }
        }
    }
}

void BinaryLogger::pollMagnetometer() {
    // MMC5983MA magnetometer has has sample available?

    // if (newMMC5983MAData == true) {
    // newMMC5983MAData = false;

    // if (mmcIntCount % 1000 == 0) {
    //     Serial.println("interrupts");
    // }

    uint8_t MMC5983MAstatus = mmc.getStatus();
    if (MMC5983MAstatus & STATUS_MEAS_M_DONE) {
        uint32_t sample[3];
        if (mmc.readData(sample) != -1) {
            // Allow any derived class to process the magnetometer sample
            handleMagSample(sample);

            // write a record to the binary log
            writeMagRecord(sample);
        }
        mmc.clearInterrupt();
    }
    // }
}

void BinaryLogger::pollGNSS() {
    // Poll GNSS receiver
    // Where a full sentence is received, extra processing ensues which may
    // trigger a change in operating state
    // @see override of SFE_UBLOX_GNSS::processNMEA
    gnss.checkUblox();
}

void BinaryLogger::processNMEAx(char incoming) {
    /*
     * New sentence arriving? record the time
     */
    if (bStartOfNMEA) {
        //lastNMEATime_ms = millis() - ulLogfileOriginMillis;
        bStartOfNMEA = false;
    }
    *pNMEA++ = incoming;

    nmea.process(incoming);

    if (incoming == '\n') {
        *pNMEA = '\0';

        if ( getOperatingState() == OperatingState::Running ) {

            // Allow any derived class to process the NMEA sentence
            handleNMEASentence( incomingNMEA );

            writeNmeaSentence( incomingNMEA );

            /*
             * Received GPS date and time? Update logfile metadata
             */
            if (strncmp(incomingNMEA + 3, "RMC", 3) == 0) {
                updateGPSDateTime(incomingNMEA);
            }

        }

        if (false) {
            Serial.print(incomingNMEA);
        }

        // Reset for next sentence
        pNMEA = incomingNMEA;
        bStartOfNMEA = true;
    }
}

void BinaryLogger::setOperatingState(OperatingState newState) {
    state = newState;
}

void BinaryLogger::writeVersionRecord(int version) {
    if (getOperatingState() == OperatingState::Running) {
        tempoRawLogRecord record;
        record.type = (int)TempoRawRecordType::Version;
        record.timestamp = (unsigned long long)timestamp.getLongMicros();
        record.variablePartLength = sizeof(record.variablePart.versionNumber);
        record.variablePart.versionNumber = TEMPO_RAW_FILE_VERSION;

        tbsLogFile.write((uint8_t *)&record, record.variablePartLength + TEMPO_RAW_FILE_HEADER_SIZE);
    }
}

void BinaryLogger::writeNmeaSentence(const char *pSentence) {
    if (getOperatingState() == OperatingState::Running) {
        // omit <CR><LF><NUL>
        size_t len = strlen(pSentence) - 2;
        if (len > 0) {
            tempoRawLogRecord record;
            record.type = (int)TempoRawRecordType::NMEA;
            record.timestamp = (unsigned long long)timestamp.getLongMicros();
            record.variablePartLength = sizeof(record.variablePart.versionNumber);
            strncpy( record.variablePart.nmeaRecord, pSentence, len );
            tbsLogFile.write((uint8_t *)&record, len + TEMPO_RAW_FILE_HEADER_SIZE);
        }
    }
}

void BinaryLogger::writeImuRecord(unsigned long gyro[3], unsigned long acc[3],
                                  unsigned char header,
                                  unsigned short timestamp,
                                  unsigned char temp) {
    if (getOperatingState() == OperatingState::Running) {
    }
}

void BinaryLogger::writeMagRecord(unsigned long sample[3]) {
    if (getOperatingState() == OperatingState::Running) {
        tempoRawLogRecord record;
        record.type = (int)TempoRawRecordType::Magnetometer;
        record.timestamp = (unsigned long long)timestamp.getLongMicros();
        record.variablePartLength = sizeof(record.variablePart.magSample);
        record.variablePart.magSample.mag[0] = sample[0];
        record.variablePart.magSample.mag[1] = sample[1];
        record.variablePart.magSample.mag[2] = sample[2];

        tbsLogFile.write((uint8_t *)&record, record.variablePartLength + TEMPO_RAW_FILE_HEADER_SIZE);
    }
}

void BinaryLogger::writeBaroRecord(double pressure_Pa, double temp_degC) {
    if (getOperatingState() == OperatingState::Running) {
        tempoRawLogRecord record;
        record.type = (int)TempoRawRecordType::Barometer;
        record.timestamp = (unsigned long long)timestamp.getLongMicros();
        record.variablePartLength = sizeof(record.variablePart.baroSample);
        record.variablePart.baroSample.pressure_Pa = pressure_Pa;
        record.variablePart.baroSample.temp_degC = temp_degC;

        tbsLogFile.write((uint8_t *)&record, record.variablePartLength + TEMPO_RAW_FILE_HEADER_SIZE);
    }
}

void BinaryLogger::dateTime(uint16_t *date, uint16_t *time) {
    *date = usGPSDate;
    *time = usGPSTime;
}

void BinaryLogger::updateGPSDateTime(char *pIncomingNMEA) {
    // sentence has already been processed prior to this call
    // data stored in nmea object
    if (nmea.isValid()) {
        usGPSDate = FAT_DATE(nmea.getYear(), nmea.getMonth(), nmea.getDay());
        usGPSTime =
            FAT_TIME(nmea.getHour(), nmea.getMinute(), nmea.getSecond());

        if (getOperatingState() == OperatingState::Running &&
            logfileDateSet == false) {
            tbsLogFile.timestamp(T_CREATE, nmea.getYear(), nmea.getMonth(),
                              nmea.getDay(), nmea.getHour(), nmea.getMinute(),
                              nmea.getSecond());
            tbsLogFile.timestamp(T_ACCESS, nmea.getYear(), nmea.getMonth(),
                              nmea.getDay(), nmea.getHour(), nmea.getMinute(),
                              nmea.getSecond());
            tbsLogFile.timestamp(T_WRITE, nmea.getYear(), nmea.getMonth(),
                              nmea.getDay(), nmea.getHour(), nmea.getMinute(),
                              nmea.getSecond());
            logfileDateSet = true;
        }
    }
}
