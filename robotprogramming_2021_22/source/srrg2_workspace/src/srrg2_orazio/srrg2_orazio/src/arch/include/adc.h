#pragma once
#include <stdint.h>

// initializes the adc of the chip, and starts one round of conversion
// interrupt driven on all ADC pins
// values are stored
void ADC_start(void);

void ADC_stop(void);

// returns the number of digital io pins on the chip
uint8_t  ADC_numChannels(void);

typedef enum {
  ADCSuccess=0,
  ADCOutOfBound=-1,
  ADCNotInitialized=-2
} ADCError;


// returns the value sampled by the adc
// <0 on error

int16_t ADC_getValue(uint8_t channel_num);
