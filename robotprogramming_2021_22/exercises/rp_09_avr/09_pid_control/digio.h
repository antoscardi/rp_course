/**
 * digio.h
 */

#pragma once

#include <stdint.h>
#include <avr/io.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the Digital I/O module acting on 
 * Register E.
 * The pins on Reg.E are:
 *  PE0 : Pin 0
 *  PE1 : Pin 1
 *  PE2 : -
 *  PE3 : Pin 5 (PWM)
 *  PE4 : Pin 2 (PWM)
 *  PE5 : Pin 3 (PWM)
 *  PE6 : -
 *  PE7 : -
 */
void DigIO_init(void);

/**
 * @brief Set the direction of a pin on Register E
 * 
 * @param bit PEx where x is the bit of the target pin.
 * @param dir Direction of the target pin: 0 for input, 1 for output
 */
void DigIO_REGE_setDirection(uint8_t bit, uint8_t dir);

/**
 * @brief Set the value of a pin on Register E
 * 
 * @param bit PEx where x is the bit of the target pin.
 * @param val Value of the target pin:
 *  if pin's direction is 0 (INPUT): 
 *      0 for direct connection, 1 for pull-up resistor enable 
 *  if pin's direction is 1 (OUTPUT): 
 *      0 for LOW, 1 for HIGH
 */
void DigIO_REGE_setValue(uint8_t bit, uint8_t val);

#ifdef __cplusplus
}
#endif
