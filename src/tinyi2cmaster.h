/** Tiny I2C
 *
 * 14th April 2018 - David Johnson-Davies - www.technoblogy.com
 * 22nd September 2018 - Hendrik Hagendorn - Removed arduino dependencies (supports only attiny85)
 *
 * CC BY 4.0
 * Licensed under a Creative Commons Attribution 4.0 International license:
 * http://creativecommons.org/licenses/by/4.0/
 */

#ifndef TinyI2CMaster_H
#define TinyI2CMaster_H

#include <avr/io.h>
#include <stdint.h>
#include <util/delay.h>

#include "pins.h"

// Defines
#define TWI_FAST_MODE

#ifdef TWI_FAST_MODE                 // TWI FAST mode timing limits. SCL = 100-400kHz
#define DELAY_T2TWI (_delay_us(2))   // >1.3us
#define DELAY_T4TWI (_delay_us(1))   // >0.6us
#else                                // TWI STANDARD mode timing limits. SCL <= 100kHz
#define DELAY_T2TWI (_delay_us(5))   // >4.7us
#define DELAY_T4TWI (_delay_us(4))   // >4.0us
#endif

#define TWI_NACK_BIT 0 // Bit position for (N)ACK bit.

// Constants
// Prepare register value to: Clear flags, and set USI to shift 8 bits i.e. count 16 clock edges.
const unsigned char USISR_8bit = 1 << USISIF | 1 << USIOIF | 1 << USIPF | 1 << USIDC |
#if PB_I2C_SCL == PINB2
                                 0x0 << USICNT0;
#else
                                 0x8 << USICNT0;
#endif
// Prepare register value to: Clear flags, and set USI to shift 1 bit i.e. count 2 clock edges.
const unsigned char USISR_1bit = 1 << USISIF | 1 << USIOIF | 1 << USIPF | 1 << USIDC |
#if PB_I2C_SCL == PINB2
                                 0xE << USICNT0;
#else
                                 0xF << USICNT0;
#endif

class TinyI2CMaster {
  public:
    TinyI2CMaster();
    void Init(void);
    uint8_t Read(void);
    uint8_t ReadLast(void);
    bool Write(uint8_t data);
    bool Start(uint8_t address, int readcount);
    bool Restart(uint8_t address, int readcount);
    void Stop(void);
    void End(void);

  private:
    int I2Ccount;
    uint8_t Transfer(uint8_t data, bool write);
};

extern TinyI2CMaster TinyI2C;

#endif
