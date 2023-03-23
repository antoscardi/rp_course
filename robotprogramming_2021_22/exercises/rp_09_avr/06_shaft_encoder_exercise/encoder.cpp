#include <util/delay.h>
#include <stdio.h>
#include <stdint.h>
#include <avr/io.h>
#include "../avr_common/uart.h" // this includes the printf and initializes it

static const uint8_t mask = 0x03;

//! @brief this represents a transition table
//!        each entry represents the output of that transition
//!        the index of the table is [prev_state, next_state]
static const int8_t _transition_table []=
  {
      0,  //0000
      1,  //0001
     -1,  //0010
      0,  //0011
     -1,  //0100
      0,  //0101
      0,  //0110
      1,  //0111
      1,  //1000
      0,  //1001
      0,  //1010
     -1,  //1011
      0,  //1100
     -1,  //1101
      1,  //1110
      0   //1111
    };


int main(void) {
  // this initializes the printf/uart things
  printf_init();

  DDRA &= ~mask;  // initialize 2 pins of portA as input
  PORTA = mask;  // pull up on input bits

  // initialize the state as pull up
  int prev_value = mask;
  int curr_value = mask;

  int counter = 0;
  
  while(1) {
    
    curr_value = PINA & mask ;  // read from portA

    // compute the index of the transition table
    uint8_t idx = prev_value | (curr_value<<2);
    counter += _transition_table[idx];

    // print multiple things
    printf("PIN<A> value     = %02x\n", curr_value);
    printf("index            = %d\n", idx);
    printf("counter value    = %d\n", counter);
    printf("***********************\n");

    // going to next state
    prev_value = curr_value;    
    _delay_ms(1000); // from delay.h, wait 1.0s
  }

  return 0;
}
