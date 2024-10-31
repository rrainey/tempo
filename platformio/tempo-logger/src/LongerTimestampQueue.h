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

// Maximum number of elements in the timestamp ring buffer.
//
// I am choosing a relatively large number here to account for time delays in some
// SPI-based SD Card operations, especially thinking about delays in opening and closing a log file.
//
// The practical upper bound of this number corresponds to the number of IMU samples storable in the
// ICM42688's FIFO buffer - for our application configuration, this number is 128.
#define IMU_TIME_RING_SIZE 128

/**
 * @class LongerTimestampQueue
 * @brief A queue for storing and retrieving timestamps.
 * 
 * The LongerTimestampQueue class provides a queue data structure for storing and retrieving timestamps.
 * It supports operations such as pushing a timestamp into the queue, popping and retrieving the next timestamp from the queue,
 * and counting the number of elements in the queue.
 */
class LongerTimestampQueue {
    public:
        /**
         * @brief Default constructor.
         * 
         * Initializes the timestamp queue with default values.
         */
        LongerTimestampQueue() {
            timestampRingFront = 0;
            timestampRingBack = 0;
        }

        /**
         * @brief Returns the number of elements in the timestamp queue.
         *
         * @return The number of elements in the timestamp queue.
         */
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

        /**
         * @brief Pushes a timestamp into the queue.
         * 
         * @param t_us The timestamp to be pushed, in microseconds.
         * @return 0 if the timestamp was successfully pushed, -1 if the queue is full.
         */
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

        /**
         * @brief Pops and retrieves the next timestamp from the queue.
         * 
         * This function removes the next timestamp from the queue and stores it in the provided pointer.
         * If the queue is empty, the function returns 0.
         * 
         * @param pt_us Pointer to a variable where the retrieved timestamp will be stored.
         * @return 1 if a timestamp was successfully retrieved, 0 if the queue is empty.
         */
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
        unsigned short timestampRingFront; /**< The front index of the timestamp ring buffer. */
        unsigned short timestampRingBack; /**< The back index of the timestamp ring buffer. */
        LongerTimestamp timestamps_us[IMU_TIME_RING_SIZE]; /**< The array of timestamps. */
};

#endif