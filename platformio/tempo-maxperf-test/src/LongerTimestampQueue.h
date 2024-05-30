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
#ifndef LONGERTIMESTAMPQUEUE_HPP_
#define LONGERTIMESTAMPQUEUE_HPP_

#include <Arduino.h>
#include "LongerTimestamp.h"

#define IMU_TIME_RING_SIZE 16

class LongerTimestampQueue {
    public:
        LongerTimestampQueue() {
            timestampRingFront = 0;
            timestampRingBack = 0;
        }

        int count() {
            int res;
            if (timestampRingFront >= timestampRingBack) {
                res = timestampRingFront - timestampRingBack;
            }
            else {
                res = IMU_TIME_RING_SIZE + timestampRingFront - timestampRingBack;
            }
            return res;
        }

        int push(longTime_t t_us) {
            int ret = 0;
            if (count() == IMU_TIME_RING_SIZE-1) {
                return -1;
            }
            timestamps_us[timestampRingFront++] = LongerTimestamp(t_us);
            if (timestampRingFront == IMU_TIME_RING_SIZE) {
                timestampRingFront = 0;
            }
            return ret;
        }

        int pop(longTime_t *pt_us) {
            int ret = 0;
            if (timestampRingFront != timestampRingBack) {
                *pt_us = timestamps_us[timestampRingBack++];
                if (timestampRingBack == IMU_TIME_RING_SIZE) {
                    timestampRingBack = 0;
                }
                ret = 1;
            }
            return ret;
        }

    protected:
        unsigned short timestampRingFront;
        unsigned short timestampRingBack;
        LongerTimestamp timestamps_us[IMU_TIME_RING_SIZE];
};

#endif