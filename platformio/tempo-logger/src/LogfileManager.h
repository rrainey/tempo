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
#pragma once
#include <Arduino.h>
#include <SdFat.h>
#include <sdios.h>

typedef unsigned short LogfileSlotID;

/**
 * We'll use the DCF standard for image storage as the basis for our
 * log file directory organization.
 */

#define LOGDIR_DCIM "DCIM"
#define LOGDIR_100TEMPO "100TEMPO"

class Logfile : public FsFile {
public:
    Logfile(FsFile &file) {
        this->pFile = &file;
        actualLength = 0;
    }

    /**
     * Writes the specified bytes to the file.
     *
     * @param pBuf Pointer to the buffer containing the bytes to be written.
     * @param size The number of bytes to write.
     * @return The number of bytes written.
     */
    int writeBytes(uint8_t *pBuf, size_t size) {
        actualLength += size;
        return pFile->write(pBuf, size);

    }

    /**
     * @brief Closes the file, syncs the data to disk, truncates the file, and sets the file pointer to NULL.
     */
    void close() {
        pFile->sync();
        pFile->truncate();
        pFile->close();
        pFile = NULL;
    }

    public:
        unsigned long actualLength;
        FsFile * pFile;
};

class LogfileManager {
    public:
    /**
     * @brief Enum class representing the result of an API operation.
     */
    enum class APIResult {
        Success = 0,                    /**< The operation was successful. */
        CannotOpenFile = -1,            /**< Failed to open the file. */
        SpacePreallocationFailed = -2,  /**< Failed to preallocate space on the media. */
        CannotCreateDirectory = -3,     /**< Failed to create a directory. */
        FileExists = -4                 /**< Trying to create a file that already exists. */
    };

    LogfileManager(SdFs *pSd);

    /**
     * Locate the next available logfile slot on the SD card.
     * 
     * Examine the \DCIM\100TEMPO\ directory and find the next available logfile slot.
     * 
     */
    APIResult findNextLogfileSlot(LogfileSlotID * pSlot);

    /**
     * Create and open a logfile for writing.
     *
     * This function creates a new logfile in the logging directory and opens it for writing.
     * If the directory does not exist, it will be created. If the logfile already exists, an error is returned.
     * The logfile will have 2GB or space preallocated to it. This space should be truncated just before the
     * file is closed.
     *
     * @param slot the slot ID to be used to generate the filename
     * @param pSuffix a pointer to a three character suffix to be assigned to the filename
     * @param file The reference to the `FsFile` object representing the logfile.
     * @return The result of the operation, indicating success or failure.
     */
    APIResult openLogfile(LogfileSlotID slot, const char * pSuffix, FsFile &file);

    protected:
        char dirname[32];
        char path[128];
        unsigned short nextIndex;
        SdFs *pSd;
};