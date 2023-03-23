/**
 * @file pid_motor_example.cpp
 */

#include "digio.h"
#include "pwm.h"
#include "timer.h"
#include "encoder.h"
#include "my_uart.h"
#include "motor.h"
#include <avr/interrupt.h>
#include <stdio.h>
#include <string.h>


// Can be changed, but do you really need to ? 
static const float DT_MS = 10;          // ms
static const float DT_S = DT_MS / 1000; // s

static const uint8_t BIT_REGE_DIRA = 4;
static const uint8_t BIT_REGE_DIRB = 5;
static const float KP = 15.;
static const float KD = 3.;
static const float KI = 1.;

// Made for you who use arduino often :)
Motor motor(BIT_REGE_DIRA, BIT_REGE_DIRB, KP, KI, KD);

// Don't touch this stuff please
volatile uint8_t timer_int_occurred = 0;
ISR(TIMER5_COMPA_vect) {
    timer_int_occurred = 1;
    // Spin the motor routine
    motor.spinOnce();
}
// 


void setup(void) {
    DigIO_init();
    UART_init();
    TIMER5_init((uint16_t)DT_MS);
    PWM_init();
    encoder_init();
    motor.usePid(true);
}

uint8_t rx_buf[64];
uint8_t tx_buf[64];

static int16_t desired_speed = 0;

void sync_loop(void) {
    // if available, read command from serial
    
    if (UART_rxAvailable()) {   
        memset(rx_buf, 0, 64);
        UART_getString(rx_buf);
        desired_speed = atoi((char*)rx_buf);
        // Issue command to motor
        motor.setSpeed(desired_speed);
    }
    sprintf((char *)tx_buf, "des_speed: %d\tmes_speed: %d\terror: %d\n",
        motor.desired_speed(), motor.measured_speed(), (int16_t)motor.error());
    UART_putString(tx_buf);
}

int main(void) {
    // Arduino like :)
    setup();
    while(1) {
        if (timer_int_occurred) {
            sync_loop();
            timer_int_occurred = 0;
        }
    }
}