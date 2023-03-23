/**
 * digio.c
 */

#include "digio.h"


void DigIO_init(void) {
    // Nothing really required here :)
}

void DigIO_REGE_setDirection(uint8_t bit, uint8_t dir) {
    uint8_t mask = 0x01 << bit;
    if (dir)
        DDRE |= mask;
    else
        DDRE &= ~mask;
}

void DigIO_REGE_setValue(uint8_t bit, uint8_t dir) {
    uint8_t mask = 0x01 << bit;
    if (dir)
        PORTE |= mask;
    else
        PORTE &= ~mask;
}