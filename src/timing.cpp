/*
 * Copyright 2019 Hendrik Hagendorn
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
#include <avr/interrupt.h>
#include <util/atomic.h>

#include "timing.h"


volatile uint32_t t0_ticks = 0;

ISR(TIMER0_COMPA_vect) {
    t0_ticks++;
}

void init_timer0_16us() {
    cli();

    t0_ticks = 0;

    TCCR0A = TCCR0B = TCNT0 = 0;
    OCR0A = 127; // = 8000000 / (1 * 62500) - 1 (must be <256)
    TCCR0A |= (1 << WGM01); // CTC
    TCCR0B |= (0 << CS02) | (0 << CS01) | (1 << CS00); // 1 prescaler
    TIMSK |= (1 << OCIE0A); // CTC interrut

    sei();
}

void stop_timer0() {
    TCCR0B = 0;
}

void wait_until(uint32_t tickstamp) {
    int32_t delta;

    while (1) {
        ATOMIC_BLOCK(ATOMIC_FORCEON) {
            delta = tickstamp - t0_ticks;
        }

        if (delta <= 0) {
            break;
        }
    }
}
