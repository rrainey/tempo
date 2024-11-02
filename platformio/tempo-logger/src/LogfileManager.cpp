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
#include "LogfileManager.h"

// preallocate 2GB log file; truncate unused space on close
unsigned long LOGFILE_PREALLOCATE = 2147483648;

LogfileManager::LogfileManager(SdFs *pSd) {
    this->pSd = pSd;
    nextIndex = 0;
}

LogfileManager::APIResult LogfileManager::findNextLogfileSlot(LogfileSlotID * pSlot) { 
    bool found = true;
    pSd->chdir("\\");


    if (pSd->exists(LOGDIR_DCIM) == false) {

        Serial.println("LOGDIR_DCIM does not exist");

        if (pSd->mkdir(LOGDIR_DCIM, true) == false) {
            return APIResult::CannotCreateDirectory;
        }
    }

    pSd->chdir(LOGDIR_DCIM);

    if (pSd->exists(LOGDIR_100TEMPO) == false) {

        Serial.println("LOGDIR_100TEMP does not exist");

        if (pSd->mkdir(LOGDIR_100TEMPO, true) == false) {
            return APIResult::CannotCreateDirectory;
        }
    }

    pSd->chdir(LOGDIR_100TEMPO);

    // both TBS and TXT filenames must be unused within the candidate numeric slot.

    while(found) {
        sprintf (path, "LOG%05d.TBS", nextIndex);
        //Serial.println(path);
        found = pSd->exists(path);

        if (!found) {

            sprintf (path, "LOG%05d.TXT", nextIndex);
            //Serial.println(path);
            found = pSd->exists(path);

            if (found) {
                nextIndex++;
            }
        }
    }

    pSd->chdir("\\");

    *pSlot = nextIndex++;

    //Serial.print("Next slot: ");
    //Serial.println(*pSlot);

    return APIResult::Success;
}

LogfileManager::APIResult LogfileManager::openLogfile(LogfileSlotID slot, const char * pSuffix, FsFile& file) { 
    APIResult result = APIResult::Success;
    bool found = true;

    pSd->chdir("\\");

    if (!pSd->exists(LOGDIR_DCIM)) {

        if (pSd->mkdir(LOGDIR_DCIM, true)) {
            return APIResult::CannotCreateDirectory;
        }
    }

    pSd->chdir(LOGDIR_DCIM);

    if (!pSd->exists(LOGDIR_100TEMPO)) {

        if (pSd->mkdir(LOGDIR_100TEMPO, true)) {
            return APIResult::CannotCreateDirectory;
        }
    }

    pSd->chdir(LOGDIR_100TEMPO);
    
    sprintf (path, "LOG%05d.%s", slot, pSuffix);
    found = pSd->exists(path);

    if (!found) {
        if (!file.open(path, O_RDWR | O_CREAT | O_TRUNC)) {
            Serial.print("open failed: " );
            Serial.println(path);
            return APIResult::CannotOpenFile;
        }
        if (LOGFILE_PREALLOCATE > 0 && !file.preAllocate(LOGFILE_PREALLOCATE)) {
            Serial.println("preallocate failed");
            return APIResult::SpacePreallocationFailed;
        }

        file.seek( 0 );
    }
    else {
        result = APIResult::FileExists;
    }

    pSd->chdir("\\");
    
    return result;
}

