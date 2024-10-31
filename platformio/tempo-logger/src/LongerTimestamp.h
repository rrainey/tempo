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
#ifndef LOGGER_HPP_
#define LOGGER_HPP_

#include <Arduino.h>

/*
 * Save a higher resolution timestamp corresponding to each IMU interrupt
 */
typedef unsigned long long longTime_t;

class LongerTimestamp {
    public:
        LongerTimestamp() {
            longTime_us = 0;
        }

        LongerTimestamp(longTime_t longTime_us) {
            this->longTime_us = longTime_us;
        }

        /// @brief get current microsecond-resolution time
        /// @return returns a long long version of the time so as not to wrap so frequently (wraps every 1^25 years)
        LongerTimestamp getLongMicros() {
            uint32_t m_us = micros();
            // catch a wrapped long microsecond counter
            if ((longTime_us & 0xffffffff) > m_us) {
                longTime_us = ((longTime_us & 0xffffffff00000000) + 0x0000000100000000) | m_us;
            }
            else {
                longTime_us = (longTime_us & 0xffffffff00000000) | m_us;
            }
            return LongerTimestamp(longTime_us);
        }

        operator longTime_t() {
            return longTime_us;
        }

    protected:
        longTime_t longTime_us;
};






#endif