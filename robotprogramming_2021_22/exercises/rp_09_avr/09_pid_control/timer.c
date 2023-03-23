/**
 * timer.c
 */

#include "timer.h"
#include <avr/io.h>
#include <avr/interrupt.h>

void TIMER5_init(uint16_t interval) {
    cli();
    // Configure timer according to example 03
    TCCR5A = 0;
    TCCR5B = (1 << WGM52) | (1 << CS50) | (1 << CS52);
    uint16_t ocrval = (uint16_t)(15.62 * interval);
    OCR5A = ocrval;
    TIMSK5 |= (1 << OCIE5A);
    sei();
}