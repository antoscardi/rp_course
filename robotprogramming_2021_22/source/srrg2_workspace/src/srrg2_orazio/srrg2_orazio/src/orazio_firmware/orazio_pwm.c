#include "orazio_pwm.h"
#include "pwm.h"
void Orazio_servoInit(void) {
  for (uint8_t i=0; i<SERVO_MAX; ++i) {
    int pin = servo_params.servo_pins[i];
    if (pin == -1)
      continue;
    PWMError result=PWM_enable(pin, 1);
    if (result!=PWMSuccess) {
      servo_params.servo_pins[i]=-1;
    }
  }
}

void Orazio_servoControl(void) {
  
}

void Orazio_servoHandle(void) {
  if (! servo_params.header.system_enabled)
    return;
  for (uint8_t i=0; i<SERVO_MAX; ++i) {
    int pin = servo_params.servo_pins[i];
    int value = servo_control.servo_value[i];
    if ( pin <0){
      servo_status.servo_value[i]=0;
      continue;
    }
    if (PWM_isEnabled(pin) != PWMEnabled) {
      servo_status.servo_value[i]=0;
      continue;
    }
    PWM_setDutyCycle(pin, servo_control.servo_value[i]);
    servo_status.servo_value[i]=value;
  }
}
