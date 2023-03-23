#include "adc.h"
#include <avr/io.h>
#include <avr/interrupt.h>

#define ADC_NUM_CHANNELS 4

static volatile uint16_t _adc_channels[ADC_NUM_CHANNELS];
static const uint8_t _adc_resamples=2; // number of times a channel is sampled
static volatile uint8_t _adc_current_resamples=0;

static uint8_t admux_default=  (1 << REFS0)|0x2;

static volatile uint8_t current_channel;
// initializes the adc of the chip
void ADC_start(void) {
  current_channel=0;
  ADCSRA |= (1 << ADPS2);// | (1 << ADPS1) | (1 << ADPS0);  
  ADCSRA |= (1 << ADEN);  
  ADCSRA |= (1 << ADIE);  
  
   // port F as input
  DDRF  = 0x03;
  PORTF = 0xFF;
  // ADC Enable and prescaler of 128
  // 16000000/128 = 125000
  DIDR0 = 0xFF;
  
  // start from first channel, no gain
  ADMUX  = admux_default + current_channel; 
  _adc_current_resamples=0;
  // start first conversion
  ADCSRA |= (1<<ADSC);
}

void ADC_stop(void) {
  ADMUX =0;
  ADCSRA &=~((1<<ADIE)); //|(1<<ADEN)
}

// returns the number of digital io pins on the chip
uint8_t  ADC_numChannels(void) {
  return ADC_NUM_CHANNELS;
}

//blocking conversion
int16_t ADC_getValue(uint8_t channel_num) {
  if (channel_num>=ADC_NUM_CHANNELS)
    return ADCOutOfBound;
  return _adc_channels[channel_num];
}

ISR(ADC_vect) {
  if (_adc_current_resamples<_adc_resamples) {
    ++_adc_current_resamples;
    _adc_channels[current_channel]=ADC;
  } else {
    _adc_current_resamples=0;
    _adc_channels[current_channel]=ADC;
    ++current_channel;
    if(current_channel>=ADC_NUM_CHANNELS) {
      ADC_stop();
      return;
    }
    ADMUX = admux_default + current_channel;
  }
  ADCSRA |= (1<<ADSC);
}
