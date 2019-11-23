#ifndef TIMING_H
#define TIMING_H

#include <stdint.h>

#define TICKS_PER_SECOND 62500 // 1s / 16us

extern volatile uint32_t t0_ticks;

void init_timer0_16us();
void stop_timer0();
void wait_until(uint32_t tickstamp);

#endif // TIMING_H
