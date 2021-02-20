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
#ifdef DEBUG

#include <avr/pgmspace.h>
#include <stdlib.h>
#include "basicserial.h"
#include "debug.h"


// common
const char STR_1[] PROGMEM = "1";
const char STR_2[] PROGMEM = "2";
// main.cpp
const char STR_BOOTED[] PROGMEM = "Booted";
const char STR_JOINING[] PROGMEM = "Joining";
const char STR_JOINED[] PROGMEM = "Joined";
const char STR_SENDING[] PROGMEM = "Sending";
// slimlora.cpp
const char STR_TXDONE[] PROGMEM = "RSP: TxDone";
const char STR_RESULT[] PROGMEM = "Result: ";
const char STR_PJA_PLEN[] PROGMEM = "PJA: packet_length = ";
const char STR_PD_PLEN[] PROGMEM = "PD: packet_length = ";
const char STR_PORT[] PROGMEM = "Port: ";
const char STR_FCNT_L[] PROGMEM = "FCnt L: ";
const char STR_FCNT_R[] PROGMEM = "FCnt R: ";

PGM_P const DEBUG_STRINGS[] PROGMEM = {
    // common
    STR_1,
    STR_2,
    // main.cpp
    STR_BOOTED,
    STR_JOINING,
    STR_JOINED,
    STR_SENDING,
    // slimlora.cpp
    STR_TXDONE,
    STR_RESULT,
    STR_PJA_PLEN,
    STR_PD_PLEN,
    STR_PORT,
    STR_FCNT_L,
    STR_FCNT_R
};

void tx(const char *str) {
    while (*str) TxByte(*str++);
}

void tx_dstr(DStr dstr) {
    PGM_P str = (PGM_P) pgm_read_word(&DEBUG_STRINGS[dstr]);
    char c;

    while ((c = pgm_read_byte(str++))) TxByte(c);
}


void debug_fast() {
    TxByte('+');
    TxByte('\n');
}

void debug(DStr dstr) {
    tx_dstr(dstr);

    TxByte('.');
    TxByte('\n');
}

void debug_int16(DStr dstr, int16_t value) {
    char buffer[7] = {0};
    itoa(value, buffer, 10);

    tx_dstr(dstr);
    tx(buffer);
    TxByte('\n');
}

void debug_uint16(DStr dstr, uint16_t value) {
    char buffer[6] = {0};
    utoa(value, buffer, 10);

    tx_dstr(dstr);
    tx(buffer);
    TxByte('\n');
}

void debug_int32(DStr dstr, int32_t value) {
    char buffer[12] = {0};
    ltoa(value, buffer, 10);

    tx_dstr(dstr);
    tx(buffer);
    TxByte('\n');
}

void debug_uint32(DStr dstr, uint32_t value) {
    char buffer[11] = {0};
    ultoa(value, buffer, 10);

    tx_dstr(dstr);
    tx(buffer);
    TxByte('\n');
}

void debug_binrep(DStr dstr, uint8_t value) {
    tx_dstr(dstr);
    for (int8_t i = 7; i >= 0; i--) {
        (value & (1 << i)) ? TxByte('1') : TxByte('0');
    }
    TxByte('\n');
}

void debug_bytes(DStr dstr, uint8_t *bytes, uint8_t length) {
    char buffer[4];

    tx_dstr(dstr);

    for (uint8_t i = 0; i < length; i++) {
        buffer[0] = buffer[1] = buffer[2] = buffer[3] = 0;

        utoa(bytes[i], buffer, 16);

        for (uint8_t i = 0; i < sizeof(buffer); i++) {
            if (buffer[i] == 0) {
                buffer[i] = ' ';
                buffer[i + 1] = 0;
                break;
            }
        }

        tx(buffer);
    }
    TxByte('\n');
}

#endif // DEBUG
