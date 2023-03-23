/**
 * pwm.h
 */

#pragma once

#include <stdint.h>
#include <avr/io.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Enable the PWM module.
 * For commodity, only the Pin 13 is enabled as PWM.
 */
void PWM_init(void);

/**
 * @brief Set the DutyCycle for the Pin 13 to change the PWM signal
 * 
 * @param value DutyCycle value from 0 to 255
 */
void PWM_setDutyCycle(uint8_t value);

#ifdef __cplusplus
}
#endif
