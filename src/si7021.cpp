/*
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
 */
#include "config.h"
#if ENABLE_SI7021

#include <avr/wdt.h>
#include <stdio.h>
#include <util/delay.h>

#include "si7021.h"
#include "utils.h"
#include "tinyi2cmaster.h"


Si7021::Si7021(uint8_t i2c_addr) {
    mI2CAddr = i2c_addr;
}

bool Si7021::Init() {
    TinyI2C.Init();

    Si7021::Reset();
    uint8_t res = Si7021::ReadRegister8(SI7021_READ_TEMPERATURE_FROM_PREVIOUS_RH_MEASUREMENT);

    return res == 0x3A;
}

void Si7021::Reset(void) {
    TinyI2C.Start(mI2CAddr, 0);
    TinyI2C.Write(SI7021_RESET);
    TinyI2C.Stop();

    _delay_ms(50);
}

uint8_t Si7021::Crc8(uint8_t h, uint8_t l) {
    uint8_t crc = 0;
    uint8_t data[] = {h, l};

    for (uint8_t i = 0; i < 2; i++) {
        crc ^= data[i];

        for (uint8_t j = 8; j > 0; j--) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ 0x131;
            } else {
                crc = (crc << 1);
            }
        }
    }

    return crc;
}

uint16_t Si7021::ReadTemperature() {
    TinyI2C.Start(mI2CAddr, 0);
    TinyI2C.Write(SI7021_READ_TEMPERATURE_FROM_PREVIOUS_RH_MEASUREMENT);
    TinyI2C.Stop();

    _delay_ms(25);

    for (uint8_t i = 0; i < 16; i++) {
        if (TinyI2C.Start(mI2CAddr, 2)) {
            uint8_t h = TinyI2C.Read();
            uint8_t l = TinyI2C.Read();
            TinyI2C.Stop();

            // No CRC available for this command
            // SI7021 always returns XXXXXX00 in the LSB field
            if (l & 0x3) {
                return SI7021_CHECKSUM_ERROR;
            }

            return h << 8 | l;
        }

        _delay_ms(6);
    }

    return SI7021_TIMEOUT_ERROR;
}

float Si7021::ReadTemperatureFloat() {
    float temperature = Si7021::ReadTemperature();
    if (temperature == SI7021_CHECKSUM_ERROR || temperature == SI7021_TIMEOUT_ERROR) {
        return NAN;
    }

    temperature *= 175.72f;
    temperature /= 65536;
    temperature -= 46.85f;

    return temperature;
}

uint16_t Si7021::MeasureTemperature() {
    TinyI2C.Start(mI2CAddr, 0);
    TinyI2C.Write(SI7021_MEASURE_TEMPERATURE_NO_HOLD_MASTER_MODE);
    TinyI2C.Stop();

    _delay_ms(25);

    for (uint8_t i = 0; i < 16; i++) {
        if (TinyI2C.Start(mI2CAddr, 3)) {
            uint8_t h = TinyI2C.Read();
            uint8_t l = TinyI2C.Read();
            uint8_t chksum = TinyI2C.Read();
            TinyI2C.Stop();

            if (chksum != Si7021::Crc8(h, l)) {
                return SI7021_CHECKSUM_ERROR;
            }

            return h << 8 | l;
        }

        _delay_ms(6);
    }

    return SI7021_TIMEOUT_ERROR;
}

float Si7021::MeasureTemperatureFloat() {
    float temperature = Si7021::MeasureTemperature();
    if (temperature == SI7021_CHECKSUM_ERROR || temperature == SI7021_TIMEOUT_ERROR) {
        return NAN;
    }

    temperature *= 175.72f;
    temperature /= 65536;
    temperature -= 46.85f;

    return temperature;
}

uint16_t Si7021::MeasureHumidity() {
    TinyI2C.Start(mI2CAddr, 0);
    TinyI2C.Write(SI7021_MEASURE_HUMIDIY_NO_HOLD_MASTER_MODE);
    TinyI2C.Stop();

    _delay_ms(25);

    for (uint8_t i = 0; i < 16; i++) {
        if (TinyI2C.Start(mI2CAddr, 3)) {
            uint8_t h = TinyI2C.Read();
            uint8_t l = TinyI2C.Read();
            uint8_t chksum = TinyI2C.Read();
            TinyI2C.Stop();

            if (chksum != Si7021::Crc8(h, l)) {
                return SI7021_CHECKSUM_ERROR;
            }

            return h << 8 | (l & 0xFC);
        }

        _delay_ms(6);
    }

    return SI7021_TIMEOUT_ERROR;
}

float Si7021::MeasureHumidityFloat() {
    float humidity = Si7021::MeasureHumidity();
    if (humidity == SI7021_CHECKSUM_ERROR || humidity == SI7021_TIMEOUT_ERROR) {
        return NAN;
    }

    humidity *= 125;
    humidity /= 65536;
    humidity -= 6;

    return humidity;
}

uint8_t Si7021::ReadRegister8(uint8_t reg) {
    uint8_t value;

    TinyI2C.Start(mI2CAddr, 0);
    TinyI2C.Write(reg);
    TinyI2C.Stop();

    for (uint8_t i = 0; i < 50; i++) {
        if (TinyI2C.Start(mI2CAddr, 1)) {
            value = TinyI2C.Read();
            TinyI2C.Stop();

            return value;
        }
        _delay_ms(2);
    }

    return 0;
}

#endif // ENABLE_SI7021
