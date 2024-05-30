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

#define DEFAULT_LOGDIR "\\DCIM\\100TEMPO"

class Logfile : public FsFile {
    Logfile(FsFile &file) {
        this->pFile = &file;
        actualLength = 0;
    }

    int writeBytes(uint8_t *pBuf, size_t size) {
        actualLength += size;
        return pFile->write(pBuf, size);

    }

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
        LogfileManager(SdFs *pSd);

        int openLogfile(FsFile &file);

    protected:
        char dirname[32];
        unsigned short nextIndex;
        SdFs *pSd;
};