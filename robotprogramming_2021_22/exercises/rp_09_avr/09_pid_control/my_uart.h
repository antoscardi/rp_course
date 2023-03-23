/**
 * my_uart.h
 */

#pragma once
#include <util/delay.h>
#include <stdio.h>
#include <stdint.h>
#include <avr/io.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initializes the UART module at baudrate 19200
 * 
 */
void UART_init(void);

/**
 * @brief Send a byte (uint8_t) on the UART interface
 * The call is blocking. 
 * 
 * @param c the byte to be sent
 */
void UART_putChar(uint8_t c);

/**
 * @brief Receives a byte (uint8_t) from the UART interface
 * The call is blocking.
 * 
 * @return uint8_t The received byte
 */
uint8_t UART_getChar(void);

/**
 * @brief Sends a stream of bytes (uint8_t*) on the UART interface
 * The call is blocking.
 * usage example:
 *  UART_putString((uint8_t*)"Hello World!\n");
 * 
 * @param buf pointer to a byte array to be sent
 */
void UART_putString(uint8_t *buf);

/**
 * @brief Receives a stream of bytes (uint8_t*) from the UART interface
 * The call is blocking.
 * usage example:
 *  uint8_t recv_buf[128];
 *  UART_getString(recv_buf);
 * 
 * @param buf pointer to a byte array in which input stream is stored
 * @return uint8_t number of received bytes
 */
uint8_t UART_getString(uint8_t *buf);

/**
 * @brief Returns true if there is data to be read from serial, false otherwise
 * 
 * @return uint8_t 
 */
uint8_t UART_rxAvailable(void);



#ifdef __cplusplus
}
#endif