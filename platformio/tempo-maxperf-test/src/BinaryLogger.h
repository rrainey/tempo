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

#include "LongerTimestamp.h"
#include "LogfileManager.h"

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

        // Create a new logfile on the SD Card and begin logging raw peripheral data to the file
        APIResult startLogging();

        // Stop logging and close the log file on the SD Card
        void stopLogging();

        // call this exactly once from the main Arduino application loop() function
        void loop();

        void processBarometerInterrupts();

        void processIMUInterrupts();

        void pollMagnetometer();

        void pollGNSS();

        OperatingState getOperatingState() const {
            return state;
        };

        void setOperatingState(OperatingState newState);

        void processNMEAx(char incoming);

    protected:
        LongerTimestamp timestamp;
        FsFile  logfile;
        bool    logfileDateSet;
        LogfileManager  logManager;
        unsigned long imuInvalidSamples;
        unsigned long imuFifoEmpty;
        unsigned long imuISROverflow;
        unsigned long imuIntCount;
        unsigned long imuFIFOProcessed;
        unsigned long loopCount;
        unsigned long loopTotal;

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