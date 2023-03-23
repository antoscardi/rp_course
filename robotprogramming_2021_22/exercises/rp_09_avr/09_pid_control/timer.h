/**
 * timer.h
 */

#pragma once

#include <stdint.h>
#include <avr/io.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initializes Timer5 to periodically trigger the 
 * TIMER5_COMPA_vect interrupt routine.
 * Fill ISR(TIMER5_COMPA_vect) {<code>} to run code during these interrupts and
 * remember not to use delays or slow code as ISR are designed to be fast.
 * 
 * @param interval period of the trigger in milliseconds [ms]
 */
void TIMER5_init(uint16_t interval);

#ifdef __cplusplus
}
#endif
