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
#ifndef BINARY_LOGGER_H_
#define BINARY_LOGGER_H_

#include <Arduino.h>
#include <SdFat.h>
#include <sdios.h>
#include <bmp3.h>
#include <ICM42688.h>

#include "LongerTimestamp.h"
#include "LogfileManager.h"
#include "MorseBlinker.h"

/**
 * @class BinaryLogger
 * @brief The BinaryLogger class is responsible for logging raw peripheral data to an SD card.
 *
 * The BinaryLogger class provides methods to start and stop logging, as well as process various interrupts and poll sensor data.
 * It also allows getting and setting the operating state of the logger.
 */
class BinaryLogger {
    public:
        enum class OperatingState { Initialized, Logging, Idle };

        enum class APIResult {
            Success = 0,
            GenericError = -1,
            CannotCreateLogfile = -2,
            SensorFault = -3
        };

        /**
         * @brief Constructs a BinaryLogger object.
         *
         * This constructor initializes the BinaryLogger object with the provided SdFs object.
         * It also initializes various member variables to their default values.
         *
         * @param Sd A reference to the SD Card used for logging.
         */
        BinaryLogger(SdFs &sd);

        // Configure all sensors for operation and begin receiving data
        APIResult configureSensors();

        // Create a new logfile on the SD Card and begin logging raw peripheral data to the file
        virtual APIResult startLogging(LogfileSlotID slot);

        // Stop logging and close the log file on the SD Card
        virtual void stopLogging();

        // call this exactly once from the main Arduino application loop() function
        virtual void loop();

        void processBarometerInterrupts();

        /// @brief process all buffered samples in IMU FIFO
        void processIMUInterrupts();

        /// @brief poll and process a Magnetometer sample
        void pollMagnetometer();

        /// @brief poll GNSS's NMEA data stream
        void pollGNSS();

        OperatingState getOperatingState() const {
            return state;
        };

        void setOperatingState(OperatingState newState);

        void processNMEAx(char incoming);

    protected:

        /**
         * @brief A stub to allow a derived class access to each IMU sample.
         *
         * @param pSample Pointer to the IMU sample.
         */
        virtual void handleIMUSample( longTime_t itime_us, icm42688::fifo_packet3 * pSample ) {};

        /**
         * @brief A stub to allow a derived class access to each magnetometer sample.
         *
         * @param sample The magnetometer sample.
         */
        virtual void handleMagSample( uint32_t sample[3] ) {};

        /**
         * @brief A stub to allow a derived class access to each barometer sample.
         *
         * @param pData The barometer sample.
         */
        virtual void handleBaroSample(  bmp3_data * pData ) {};

        /**
         * @brief A stub to allow a derived class access to each NMEA sentence.
         *
         * @param sentenceStartTime_ms millis() time at start of the sentence reception
         * @param pSentence The NMEA sentence.
         */
        virtual void handleNMEASentence( uint32_t sentenceStartTime_ms, const char * pSentence ) {};

        LongerTimestamp timestamp;
        FsFile  tbsLogFile;
        bool    logfileDateSet;
        LogfileManager  logManager;
        unsigned long imuInvalidSamples;
        unsigned long imuFifoEmpty;
        unsigned long imuISROverflow;
        unsigned long imuIntCount;
        unsigned long imuFIFOProcessed;
        unsigned long loopCount;
        unsigned long loopTotal;

        // millis() time at start of logging
        uint32_t ulLogfileOriginMillis;

        // millis() time at start of NMEA sentence, relative to ulLogfileOriginMillis
        uint32_t lastNMEATime_ms;

        int8_t rslt;
        bmp3_dev dev;
        bmp3_data data;

        OperatingState state;

        unsigned short usGPSDate;
        unsigned short usGPSTime;

        void writeVersionRecord(int version);
        void writeNmeaSentence(const char *pSentence);
        void writeImuRecord(unsigned long gyro[3], unsigned long acc[3], unsigned char header, unsigned short timestamp, unsigned char temp);
        void writeMagRecord(unsigned long sample[3]);
        void writeBaroRecord(double pressure_Pa, double temp_degC);

        void dateTime(uint16_t* date, uint16_t* time);

        // This shall only be called for G*RMC sentences
        // If the RMC status is marked as "Valid" update last GPS time
        // This allows us to use GPS time to set a close-to-correct file
        // creation time for each log file.
        void updateGPSDateTime(char* pIncomingNMEA);
};

#endif