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
#include <avr/wdt.h>
#include <stdio.h>
#include <util/delay.h>

#include "dht22.h"
#include "pins.h"
#include "utils.h"

int8_t DHT22::AwaitBit(uint8_t state, int8_t timeout) {
    int8_t count = 0;

    for (; BITVAL(PIN_DHT22, PB_DHT22) == state; count++) {
        if (count >= timeout) {
            return -1;
        }

        // Above instructions already delay by more than 1 us
        //_delay_us(1);
    }

    return count;
}

int8_t DHT22::Reset() {
    mData[0] = 0;
    mData[1] = 0;
    mData[2] = 0;
    mData[3] = 0;
    mData[4] = 1;

    SETBIT(DDR_DHT22, PB_DHT22);
    CLEARBIT(PRT_DHT22, PB_DHT22);
    _delay_ms(5);

    CLEARBIT(DDR_DHT22, PB_DHT22);
    SETBIT(PRT_DHT22, PB_DHT22);
    _delay_us(5);

    // 20-40us HIGH
    if (DHT22::AwaitBit(1, 50) < 0) {
        return -11;
    }

    // 80us LOW
    if (DHT22::AwaitBit(0, 90) < 0) {
        return -12;
    }

    // 80us HIGH
    if (DHT22::AwaitBit(1, 90) < 0) {
        return -13;
    }

    return 0;
}

int8_t DHT22::ReadData() {
    int8_t result, time;

    if ((result = DHT22::Reset()) < 0) {
        return result;
    }

    for (uint8_t i = 0; i < 40; i++) {
        if (DHT22::AwaitBit(0, 60) < 0) {
            return -14;
        }

        time = DHT22::AwaitBit(1, 80);
        if (time < 0) {
            return -15;
        }

        mData[i / 8] <<= 1;

        if (time > 30) { // Bit 1
            mData[i / 8] |= 1;
        }
    }

    if (mData[4] != ((mData[0] + mData[1] + mData[2] + mData[3]) & 0xFF)) {
        return -1;
    }

    return 0;
}

// Divide return value by 10
uint16_t DHT22::GetHumidity() {
    return (mData[0] << 8) + mData[1];
}

// Divide return value by 10
uint16_t DHT22::GetTemperature() {
    return (mData[2] << 8) + mData[3];
}
