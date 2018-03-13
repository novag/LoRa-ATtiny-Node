// Arduino tinySPI Library Copyright (C) 2018 by Jack Christensen and
// licensed under GNU GPL v3.0, https://www.gnu.org/licenses/gpl.html
//
// Arduino hardware SPI master library for
// ATtiny24/44/84, ATtiny25/45/85, ATtiny261/461/861, ATtiny2313/4313.
//
// https://github.com/JChristensen/tinySPI
// Jack Christensen 24Oct2013

#include "tinyspi.h"

void TinySPI::Init() {
    USICR &= ~(_BV(USISIE) | _BV(USIOIE) | _BV(USIWM1));
    USICR |= _BV(USIWM0) | _BV(USICS1) | _BV(USICLK);
    SPI_DDR_PORT |= _BV(USCK_DD_PIN);   // set the USCK pin as output
    SPI_DDR_PORT |= _BV(DO_DD_PIN);     // set the DO pin as output
    SPI_DDR_PORT &= ~_BV(DI_DD_PIN);    // set the DI pin as input
}

void TinySPI::SetDataMode(uint8_t spi_data_mode) {
    if (spi_data_mode == SPI_MODE1) {
        USICR |= _BV(USICS0);
    } else {
        USICR &= ~_BV(USICS0);
    }
}

uint8_t TinySPI::Transfer(uint8_t spi_data) {
    USIDR = spi_data;
    USISR = _BV(USIOIF);                // clear counter and counter overflow interrupt flag
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { // ensure a consistent clock period
        while (!(USISR & _BV(USIOIF))) USICR |= _BV(USITC);
    }

    return USIDR;
}

void TinySPI::End() {
    USICR &= ~(_BV(USIWM1) | _BV(USIWM0));
}

TinySPI SPI;
