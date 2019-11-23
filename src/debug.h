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
#ifndef DEBUG_H
#define DEBUG_H

#include <stdint.h>


enum DStr {
    // common
    DSTR_1,
    DSTR_2,
    // main.cpp
    DSTR_BOOTED,
    DSTR_JOINING,
    DSTR_JOINED,
    DSTR_SENDING,
    // tinylora.cpp
    DSTR_RSP_TXDONE,
    DSTR_RESULT,
    DSTR_PJA_PLEN,
    DSTR_PD_PLEN,
    DSTR_PORT,
    DSTR_FCNT_L,
    DSTR_FCNT_R
};

void debug(DStr dstr);
void debug_int16(DStr dstr, int16_t value);
void debug_uint16(DStr dstr, uint16_t value);
void debug_int32(DStr dstr, int32_t value);
void debug_uint32(DStr dstr, uint32_t value);
void debug_binrep(DStr dstr, uint8_t value);
void debug_bytes(DStr dstr, uint8_t *bytes, uint8_t length);

#endif // DEBUG_H
