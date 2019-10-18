/** Tiny I2C
 *
 * 14th April 2018 - David Johnson-Davies - www.technoblogy.com
 * 22nd September 2018 - Hendrik Hagendorn - Removed arduino dependencies (supports only attiny85)
 *
 * CC BY 4.0
 * Licensed under a Creative Commons Attribution 4.0 International license:
 * http://creativecommons.org/licenses/by/4.0/
 */
#include "config.h"
#if ENABLE_SI7021

#include "tinyi2cmaster.h"

TinyI2CMaster::TinyI2CMaster() { }

uint8_t TinyI2CMaster::Transfer(uint8_t data, bool write) {
    USISR = data;                               // Set USISR according to data.
    /* Prepare clocking. */
    data = 0 << USISIE | 0 << USIOIE |          // Interrupts disabled
           1 << USIWM1 | 0 << USIWM0 |          // Set USI in Two-wire mode.
#if PB_I2C_SCL == PINB2
           1 << USICS1 | 0 << USICS0 | 1 << USICLK | // Software clock strobe as source (USITC).
           1 << USITC;                          // Toggle Clock Port.
#else
           0 << USICS1 | 0 << USICS0 | 1 << USICLK | // Software clock strobe as source (USICLK).
           0 << USITC;                          // Ignore Clock Port.
#endif

    do {
        DELAY_T2TWI;
#if PB_I2C_SCL == PINB2
        USICR = data;                           // Generate positive SCL edge.
#else
        PRT_I2C_SCL |= 1 << PB_I2C_SCL;         // Release SCL.
#endif
        while (!(PIN_I2C_SCL & 1 << PB_I2C_SCL)); // Wait for SCL to go high.
        DELAY_T4TWI;
#if PB_I2C_SCL == PINB2
        USICR = data;                           // Generate negative SCL edge.
#else
        if (!write) USICR = data;
        PRT_I2C_SCL &= ~(1 << PB_I2C_SCL);      // Pull SCL LOW.
        if (write) USICR = data;
#endif
    } while (!(USISR & 1 << USIOIF));           // Check for transfer complete.

    DELAY_T2TWI;
    data = USIDR;                               // Read out data.
    USIDR = 0xFF;                               // Release SDA.
    DDRB |= (1 << PINB0);                       // Enable SDA as output.

    return data;                                // Return the data from the USIDR
}

void TinyI2CMaster::Init() {
    PORTB |= 1 << PINB0;                        // Enable pullup on SDA.
    PRT_I2C_SCL |= 1 << PB_I2C_SCL;             // Enable pullup on SCL.

    DDR_I2C_SCL |= 1 << PB_I2C_SCL;             // Enable SCL as output.
    DDRB |= 1 << PINB0;                         // Enable SDA as output.

    USIDR = 0xFF;                               // Preload data register with "released level" data.
    USICR = 0 << USISIE | 0 << USIOIE |         // Disable Interrupts.
            1 << USIWM1 | 0 << USIWM0 |         // Set USI in Two-wire mode.
#if PB_I2C_SCL == PINB2
            1 << USICS1 | 0 << USICS0 | 1 << USICLK | // Software stobe as counter clock source (USITC)
#else
            0 << USICS1 | 0 << USICS0 | 1 << USICLK | // Software stobe as counter clock source (USICLK)
#endif
            0 << USITC;
    USISR = 1 << USISIF | 1 << USIOIF | 1 << USIPF | 1 << USIDC | // Clear flags,
            0x0 << USICNT0;                     // and reset counter.
}

uint8_t TinyI2CMaster::Read(void) {
    if ((I2Ccount != 0) && (I2Ccount != -1)) {
        I2Ccount--;
    }

    /* Read a byte */
    DDRB &= ~(1 << PINB0);                      // Enable SDA as input.
    uint8_t data = TinyI2CMaster::Transfer(USISR_8bit, false);

    /* Prepare to generate ACK (or NACK in case of End Of Transmission) */
    if (I2Ccount == 0) {
        USIDR = 0xFF;
    } else {
        USIDR = 0x00;
    }
    TinyI2CMaster::Transfer(USISR_1bit, true);  // Generate ACK/NACK.

    return data;                                // Read successfully completed
}

uint8_t TinyI2CMaster::ReadLast(void) {
    I2Ccount = 0;

    return TinyI2CMaster::Read();
}

bool TinyI2CMaster::Write(uint8_t data) {
    /* Write a byte */
    PRT_I2C_SCL &= ~(1 << PB_I2C_SCL);          // Pull SCL LOW.
    USIDR = data;                               // Setup data.
    TinyI2CMaster::Transfer(USISR_8bit, true);  // Send 8 bits on bus.

    /* Clock and verify (N)ACK from slave */
    DDRB &= ~(1 << PINB0);                      // Enable SDA as input.
    if (TinyI2CMaster::Transfer(USISR_1bit, false) & 1 << TWI_NACK_BIT) {
        return false;
    }

    return true;                                // Write successfully completed
}

bool TinyI2CMaster::Start(uint8_t address, int readcount) {
    if (readcount != 0) {
        I2Ccount = readcount;
        readcount = 1;
    }
    uint8_t addressRW = address << 1 | readcount;

    /* Release SCL to ensure that (repeated) Start can be performed */
    PRT_I2C_SCL |= 1 << PB_I2C_SCL;             // Release SCL.
    while (!(PIN_I2C_SCL & 1 << PB_I2C_SCL));   // Verify that SCL becomes high.
#ifdef TWI_FAST_MODE
    DELAY_T4TWI;
#else
    DELAY_T2TWI;
#endif

    /* Generate Start Condition */
    PORTB &= ~(1 << PINB0);                     // Force SDA LOW.
    DELAY_T4TWI;
    PRT_I2C_SCL &= ~(1 << PB_I2C_SCL);          // Pull SCL LOW.
    PORTB |= 1 << PINB0;                        // Release SDA.

#if PB_I2C_SCL == PINB2
    if (!(USISR & 1 << USISIF)) {
        return false;
    }
#else
    DELAY_T4TWI;
#endif

    /* Write address */
    PRT_I2C_SCL &= ~(1 << PB_I2C_SCL);          // Pull SCL LOW.
    USIDR = addressRW;                          // Setup data.
    TinyI2CMaster::Transfer(USISR_8bit, true);  // Send 8 bits on bus.

    /* Clock and verify (N)ACK from slave */
    DDRB &= ~(1 << PINB0);                      // Enable SDA as input.
    if (TinyI2CMaster::Transfer(USISR_1bit, false) & 1 << TWI_NACK_BIT) {
        return false;                           // No ACK
    }

    return true;                                // Start successfully completed
}

bool TinyI2CMaster::Restart(uint8_t address, int readcount) {
    return TinyI2CMaster::Start(address, readcount);
}

void TinyI2CMaster::Stop(void) {
    PORTB &= ~(1 << PINB0);                     // Pull SDA low.
    PRT_I2C_SCL |= 1 << PB_I2C_SCL;             // Release SCL.
    while (!(PIN_I2C_SCL & 1 << PB_I2C_SCL));   // Wait for SCL to go high.
    DELAY_T4TWI;
    PORTB |= 1 << PINB0;                        // Release SDA.
    DELAY_T2TWI;
}

void TinyI2CMaster::End() {
    PRT_I2C_SCL &= ~(1 << PB_I2C_SCL);          // Pull SCL LOW.
}

TinyI2CMaster TinyI2C = TinyI2CMaster();        // Instantiate a TinyI2C object

#endif // ENABLE_SI7021
