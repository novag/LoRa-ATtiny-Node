/*
 * Copyright (c) 2018 Hendrik Hagendorn
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this program.  If not, see <http:// www.gnu.org/licenses/>.
 */
#ifndef DHT22_H
#define DHT22_H

#include <stdint.h>

class DHT22 {
  public:
    int8_t ReadData();
    uint16_t GetHumidity();
    uint16_t GetTemperature();
  private:
    uint8_t mData[5];

    int8_t AwaitBit(uint8_t state, int8_t timeout);
    int8_t Reset();
};

#endif
