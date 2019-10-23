/*
 * Copyright 2018 Hendrik Hagendorn
 * Copyright 2017 Arjen Lentz - DOW-CRC table
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
#if ENABLE_DS18B20

#include <avr/pgmspace.h>
#include <util/delay.h>

#include "ds18b20.h"
#include "pins.h"
#include "utils.h"


// DOW-CRC / Polynomial X^8 + X^5 + X^4 + X^0
// https://lentz.com.au/blog/calculating-crc-with-a-tiny-32-entry-lookup-table
static const uint8_t PROGMEM DowCrcTable[] = {
    0x00, 0x5E, 0xBC, 0xE2, 0x61, 0x3F, 0xDD, 0x83,
    0xC2, 0x9C, 0x7E, 0x20, 0xA3, 0xFD, 0x1F, 0x41,
    0x00, 0x9D, 0x23, 0xBE, 0x46, 0xDB, 0x65, 0xF8,
    0x8C, 0x11, 0xAF, 0x32, 0xCA, 0x57, 0xE9, 0x74
};


bool DS18B20::Reset() {
    bool is_present = false;

    SETBIT(DDR_DS18B20, PB_DS18B20);
    CLEARBIT(PRT_DS18B20, PB_DS18B20);
    _delay_us(500);

    CLEARBIT(DDR_DS18B20, PB_DS18B20);
    SETBIT(PRT_DS18B20, PB_DS18B20);
    _delay_us(90);

    if (BITVAL(PIN_DS18B20, PB_DS18B20) == 0) {
        is_present = true;
    }
    _delay_us(300);

    return is_present;
}

void DS18B20::WriteBit(bool bit) {
    SETBIT(DDR_DS18B20, PB_DS18B20);
    CLEARBIT(PRT_DS18B20, PB_DS18B20);
    _delay_us(5);

    if (bit) {
        SETBIT(PRT_DS18B20, PB_DS18B20);
        _delay_us(55);
    } else {
        _delay_us(55);
        SETBIT(PRT_DS18B20, PB_DS18B20);
        _delay_us(5);
    }
}

void DS18B20::WriteByte(uint8_t data) {
    for (uint8_t i = 0; i < 8; i++) {
        DS18B20::WriteBit(data & 1);
        data >>= 1;
    }
}

bool DS18B20::ReadBit() {
    bool bit;

    SETBIT(DDR_DS18B20, PB_DS18B20);
    CLEARBIT(PRT_DS18B20, PB_DS18B20);
    _delay_us(5);

    CLEARBIT(DDR_DS18B20, PB_DS18B20);
    SETBIT(PRT_DS18B20, PB_DS18B20);
    _delay_us(10);

    bit = BITVAL(PIN_DS18B20, PB_DS18B20);

    _delay_us(50);

    return bit;
}

uint8_t DS18B20::ReadByte() {
    uint8_t data = 0;

    for (uint8_t i = 0; i < 8; i++) {
        data >>= 1;
        if (DS18B20::ReadBit()) {
            data |= 0x80;
        }
    }

    return data;
}

void DS18B20::SkipROM() {
    DS18B20::WriteByte(CMD_DS18B20_SKIP_ROM);
}

void DS18B20::CopyScratchpad() {
    DS18B20::Reset();
    DS18B20::SkipROM();

    DS18B20::WriteByte(CMD_DS18B20_COPY_SCRATCHPAD);
    _delay_ms(10);
}

void DS18B20::WriteScratchpad() {
    DS18B20::Reset();
    DS18B20::SkipROM();

    DS18B20::WriteByte(CMD_DS18B20_WRITE_SCRATCHPAD);

    DS18B20::WriteByte(125); // High alarm temperature
    DS18B20::WriteByte(-55); // Low alarm temperature

    // Resolution
#if CFG_DS18B20_RES == 9
    DS18B20::WriteByte(RES_DS18B20_9_BIT);
#elif CFG_DS18B20_RES == 10
    DS18B20::WriteByte(RES_DS18B20_10_BIT);
#elif CFG_DS18B20_RES == 11
    DS18B20::WriteByte(RES_DS18B20_11_BIT);
#else
    DS18B20::WriteByte(RES_DS18B20_12_BIT);
#endif
}

void DS18B20::ReadScratchpad(uint8_t *data) {
    DS18B20::Reset();
    DS18B20::SkipROM();

    DS18B20::WriteByte(CMD_DS18B20_READ_SCRATCHPAD);
    for (uint8_t i = 0; i < 9; i++) {
        data[i] = DS18B20::ReadByte();
    }
}

uint16_t DS18B20::MeasureTemperature() {
    uint8_t data[9], cfg;
    uint16_t temp;

    if (!mConfigurationValid) {
        DS18B20::ReadScratchpad(data);
        if (DS18B20::Crc8(data, 8) != data[8]) {
            return DS18B20_CHECKSUM_ERROR;
        }

#if CFG_DS18B20_RES == 9
        if (data[4] != RES_DS18B20_9_BIT) {
#elif CFG_DS18B20_RES == 10
        if (data[4] != RES_DS18B20_10_BIT) {
#elif CFG_DS18B20_RES == 11
        if (data[4] != RES_DS18B20_11_BIT) {
#else
        if (data[4] != RES_DS18B20_12_BIT) {
#endif
            DS18B20::WriteScratchpad();
            DS18B20::CopyScratchpad();
        }

        mConfigurationValid = true;
    }

    DS18B20::Reset();
    DS18B20::SkipROM();

    DS18B20::WriteByte(CMD_DS18B20_CONVERT_T);

#if CFG_DS18B20_RES == 9
    _delay_ms(94);
#elif CFG_DS18B20_RES == 10
    _delay_ms(188);
#elif CFG_DS18B20_RES == 11
    _delay_ms(375);
#else
    _delay_ms(750);
#endif

    DS18B20::ReadScratchpad(data);
    if (DS18B20::Crc8(data, 8) != data[8]) {
        return DS18B20_CHECKSUM_ERROR;
    }

    temp = (data[1] << 8) | data[0];

#if CFG_DS18B20_RES == 9
    temp &= 0xFFF8;
#elif CFG_DS18B20_RES == 10
    temp &= 0xFFFC;
#elif CFG_DS18B20_RES == 11
    temp &= 0xFFFE;
#endif

    return temp;
}

uint8_t DS18B20::Crc8(const uint8_t *data, uint8_t length) {
    uint8_t crc = 0;

    while (length--) {
        crc = *data++ ^ crc;
        crc = pgm_read_byte(DowCrcTable + (crc & 0x0f)) ^
              pgm_read_byte(DowCrcTable + 16 + ((crc >> 4) & 0x0f));
    }

    return crc;
}

#endif // ENABLE_DS18B20
