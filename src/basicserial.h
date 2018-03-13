/* Optimized AVR305 half-duplex serial uart implementation
 * timing for 81N, 115.2kbps @8Mhz = 69.4 cycles/bit
 * and @16Mhz = 138.9 cycles/bit
 * @author: Ralph Doncaster
 * @author: Hendrik Hagendorn
 * @version: $Id$
 */

#ifndef BASICSERIAL_H
#define BASICSERIAL_H

#include <stdint.h>
#include <stdlib.h>

#define BAUD_RATE 115200

#ifdef F_CPU
#define TXDELAY (((F_CPU / BAUD_RATE) - 9) / 3)
#else
#error CPU frequency F_CPU undefined
#endif

#if TXDELAY > 255
#error low baud rates unsupported - use higher BAUD_RATE
#endif

extern "C" {
    void TxTimedByte(char, char);
}

#define TxByte(c) TxTimedByte(c, TXDELAY)

inline void debug(const char *msg) {
    while (*msg) TxByte(*msg++);
}

inline void debug_int(const char *prefix, int value) {
    char buffer[6] = {0};
    itoa(value, buffer, 10);

    debug(prefix);
    debug(buffer);
    TxByte('\n');
}

inline void debug_uint(const char *prefix, unsigned int value) {
    char buffer[6] = {0};
    utoa(value, buffer, 10);

    debug(prefix);
    debug(buffer);
    TxByte('\n');
}

inline void debug_binrep(const char *prefix, uint8_t value) {
    debug(prefix);
    for (int8_t i = 7; i >= 0; i--) {
        (value & (1 << i)) ? TxByte('1') : TxByte('0');
    }
    TxByte('\n');
}

#endif