/**
 * encoder.h
 */

#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize Encoder module.
 * The encoder relies on the first two pins of Register B
 *  PB0 : Pin 53
 *  PB1 : Pin 52
 */
void encoder_init(void);

/**
 * @brief Returns the encoder counter at the current instant.
 * 
 * @return uint16_t encoder counter
 */
uint16_t encoder_read(void);

#ifdef __cplusplus
}
#endif