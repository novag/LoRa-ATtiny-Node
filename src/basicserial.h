/* Optimized AVR305 half-duplex serial uart implementation
 * timing for 81N, 115.2kbps @8Mhz = 69.4 cycles/bit
 * and @16Mhz = 138.9 cycles/bit
 * @author: Ralph Doncaster
 * @author: Hendrik Hagendorn
 * @version: $Id$
 */

#ifndef BASICSERIAL_H
#define BASICSERIAL_H

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

#endif // BASICSERIAL_H
