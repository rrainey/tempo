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
}

int LogfileManager::openLogfile(FsFile& file) { 
    char path[128];
    bool found = false;
    if (!pSd->exists(DEFAULT_LOGDIR)) {
        if (pSd->mkdir(DEFAULT_LOGDIR, true)) {
            return -3;
        }
    }
    strcpy(dirname, DEFAULT_LOGDIR);
    strcat (dirname, "\\");
    while(!found) {
        strcpy(path, dirname);
        sprintf (path+strlen(dirname), "log%05d.tbs", nextIndex++);
        found = !pSd->exists(path);
    }
    if (!file.open(path, O_RDWR | O_CREAT | O_TRUNC)) {
        return -1;
    }
    if (LOGFILE_PREALLOCATE > 0 && !file.preAllocate(LOGFILE_PREALLOCATE)) {
        return -2;
    }

    file.seek( 0 );
    
    return 0;
}

