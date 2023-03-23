#include "counter.h"
#include <avr/io.h>

// initialize counter 4 in 16 bit mode, to count at 0.5 us
void Counter_init(void){
  // set normal mode of operation
  TCCR1A=0;
  TCCR1B=(1 << CS12);
  OCR1A=0xFFFF;
}

uint16_t Counter_get(void){
  return TCNT1;
}
