/******************************************************************************************
* Copyright 2018 Hendrik Hagendorn
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
******************************************************************************************/
#ifndef SI7021_H
#define SI7021_H

#include <stdint.h>

#define SI7021_DEFAULT_ADDRESS 0x40

#define SI7021_READ_HEATER_CONTROL_REGISTER                     0x11
#define SI7021_WRITE_HEATER_CONTROL_REGISTER                    0x51
#define SI7021_READ_TEMPERATURE_FROM_PREVIOUS_RH_MEASUREMENT    0xE0
#define SI7021_READ_USER_REGISTER                               0xE7
#define SI7021_MEASURE_TEMPERATURE_NO_HOLD_MASTER_MODE          0xF3
#define SI7021_MEASURE_HUMIDIY_NO_HOLD_MASTER_MODE              0xF5
#define SI7021_RESET                                            0xFE

#define SI7021_CHECKSUM_ERROR   0x2
#define SI7021_TIMEOUT_ERROR    0x3

class Si7021 {
    int8_t mI2CAddr;

    void Reset();
    uint8_t Crc8(uint8_t h, uint8_t l);
    uint8_t ReadRegister8(uint8_t reg);

  public:
    Si7021(uint8_t i2c_addr);
    bool Init();
    uint16_t ReadTemperature();
    float ReadTemperatureFloat();
    uint16_t MeasureTemperature();
    float MeasureTemperatureFloat();
    uint16_t MeasureHumidity();
    float MeasureHumidityFloat();
};

#endif
