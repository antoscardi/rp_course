#include "orazio_hbridge_internal.h"
#include "digio.h"
#include "pwm.h"
#include "orazio_packets.h"



void HBridgePWMDir_fini(HBridge* bridge);
void HBridgePWMDir_setSpeed(HBridge* bridge, int16_t speed);

void HBridgeDualPWM_fini(HBridge* bridge);
void HBridgeDualPWM_setSpeed(HBridge* bridge, int16_t speed);

void HBridgeHalfPWM_fini(HBridge* bridge);
void HBridgeHalfPWM_setSpeed(HBridge* bridge, int16_t speed);

void HBridgePWMUpDown_fini(HBridge* bridge);
void HBridgePWMUpDown_setSpeed(HBridge* bridge, int16_t speed);


static HBridgeOps h_bridge_ops[]=
  {
    // PWM and dir
    {
      .fini_fn=HBridgePWMDir_fini,
      .setSpeed_fn=HBridgePWMDir_setSpeed
    },
    // dual pwm
    {
      .fini_fn=HBridgeDualPWM_fini,
      .setSpeed_fn=HBridgeDualPWM_setSpeed
    },
    // half pwm
    {
      .fini_fn=HBridgeHalfPWM_fini,
      .setSpeed_fn=HBridgeHalfPWM_setSpeed
    },
    // pwmupdown
    {
      .fini_fn=HBridgePWMUpDown_fini,
      .setSpeed_fn=HBridgePWMUpDown_setSpeed
    },
    
  };


/* PWM+Dir Mode */
PacketStatus HBridgePWMDir_init(HBridge* bridge, int8_t* pins){
  bridge->ops=h_bridge_ops+HBridgeTypePWMDir;

  int8_t  *pwm_pin=&pins[0];
  int8_t  *dir_pin=&pins[1];
  int8_t  *brake_pin=&pins[2];
  PacketStatus status =Success;

  if (DigIO_setDirection(*dir_pin,Output)<0
      || DigIO_setValue(*dir_pin,0) < 0) {
    *dir_pin=-1;
    status = WrongPins;
  }
  bridge->params.pwmdir.dir_pin=*dir_pin;

  if (PWM_enable(*pwm_pin,1)<0
      || PWM_setDutyCycle(*pwm_pin, 0)<0) {
    *pwm_pin=-1;
    status = WrongPins;
  }
  bridge->params.pwmdir.pwm_pin=*pwm_pin;

  // brake_pin optional
  if (*brake_pin>=0
      && (DigIO_setDirection(*brake_pin,Output)<0
          || DigIO_setValue(*brake_pin,0) < 0 )){
    *brake_pin=-1;
    status = WrongPins;
  }
  DigIO_setValue(*brake_pin,0); 
  bridge->params.pwmdir.brake_pin=*brake_pin;

  return status;
}

void HBridgePWMDir_fini(HBridge* bridge){
  PWM_setDutyCycle(bridge->params.pwmdir.pwm_pin,0);
  PWM_enable(bridge->params.pwmdir.pwm_pin,0);
  DigIO_setValue(bridge->params.pwmdir.dir_pin,0);
  DigIO_setDirection(bridge->params.pwmdir.dir_pin,Input);
  DigIO_setValue(bridge->params.pwmdir.brake_pin, 0);
  DigIO_setDirection(bridge->params.pwmdir.brake_pin, Input);
}

void HBridgePWMDir_setSpeed(HBridge* bridge, int16_t speed){
  const int8_t dir_pin = bridge->params.pwmdir.dir_pin;
  const int8_t pwm_pin = bridge->params.pwmdir.pwm_pin;
  uint16_t pwm=0;
  uint8_t dir=0;
  if(speed>=0){
    pwm=speed;
    dir=0;
  } else {
    pwm=-speed;
    dir=1;
  }
  if (pwm>255)
    pwm=255;
  DigIO_setValue(dir_pin, dir);
  PWM_setDutyCycle(pwm_pin, pwm);
}

/* Dual PWM */
PacketStatus HBridgeDualPWM_init(HBridge* bridge, int8_t* pins){
  bridge->ops=h_bridge_ops+HBridgeTypeDualPWM;

  int8_t* pwm_forward_pin=&pins[0];
  int8_t* pwm_backward_pin=&pins[1];
  PacketStatus status =Success;

  if (PWM_enable(*pwm_forward_pin,1)<0
      || PWM_setDutyCycle(*pwm_forward_pin,0)<0) {
    *pwm_forward_pin = -1;
    status=WrongPins;
  }
  bridge->params.dualpwm.pwm_forward_pin=*pwm_forward_pin;

  if (PWM_enable(*pwm_backward_pin,1)<0
      || PWM_setDutyCycle(*pwm_backward_pin,0)<0) {
    *pwm_backward_pin = -1;
    status=WrongPins;
  }
  bridge->params.dualpwm.pwm_backward_pin = *pwm_backward_pin;
  return status;
}

void HBridgeDualPWM_fini(HBridge* bridge){
  PWM_setDutyCycle(bridge->params.dualpwm.pwm_forward_pin,0);
  PWM_enable(bridge->params.dualpwm.pwm_forward_pin,0);
  PWM_setDutyCycle(bridge->params.dualpwm.pwm_backward_pin,0);
  PWM_enable(bridge->params.dualpwm.pwm_backward_pin,0);
}

void HBridgeDualPWM_setSpeed(HBridge* bridge, int16_t speed){
  const uint8_t fpwm_pin = bridge->params.dualpwm.pwm_forward_pin;
  const uint8_t bpwm_pin = bridge->params.dualpwm.pwm_backward_pin;
  if (speed>255)
    speed=255;
  if (speed<-255)
    speed=-255;
  if(speed>=0){
    PWM_setDutyCycle(fpwm_pin, speed);
    PWM_setDutyCycle(bpwm_pin, 0);
  } else {
    PWM_setDutyCycle(fpwm_pin, 0);
    PWM_setDutyCycle(bpwm_pin, -speed);
  }
}

/* Half PWM */
PacketStatus HBridgeHalfPWM_init(HBridge* bridge, int8_t* pins){
  bridge->ops=h_bridge_ops+HBridgeTypeHalfCyclePWM;

  int8_t *pwm_pin=&pins[0];
  int8_t *enable_pin=&pins[1];
  PacketStatus status=Success;

  if (PWM_setDutyCycle(*pwm_pin,0)<0
      || PWM_enable(*pwm_pin,1)<0) {
    *pwm_pin=bridge->params.halfpwm.pwm_pin;
    status =  WrongPins;
  }
  bridge->params.halfpwm.pwm_pin=*pwm_pin;

  if ( *enable_pin>=0 &&
       ( DigIO_setDirection(*enable_pin,Output)<0
         || DigIO_setValue(*enable_pin, 1) < 0)) {
    *enable_pin = bridge->params.halfpwm.enable_pin;
    status= WrongPins;
  }
  bridge->params.halfpwm.enable_pin = *enable_pin;
  return status;
}

void HBridgeHalfPWM_fini(HBridge* bridge){
  uint8_t pwm_pin=bridge->params.halfpwm.pwm_pin;
  uint8_t enable_pin=bridge->params.halfpwm.enable_pin;
  PWM_setDutyCycle(pwm_pin,0);
  PWM_enable(pwm_pin,0);
  DigIO_setDirection(enable_pin,Input);
  DigIO_setValue(enable_pin, 0);
}

void HBridgeHalfPWM_setSpeed(HBridge* bridge, int16_t speed){
  uint8_t pwm_pin=bridge->params.halfpwm.pwm_pin;
  if (speed>255)
    speed=255;
  if (speed<-255)
    speed=-255;
  speed=127+speed/2;
  PWM_setDutyCycle(pwm_pin, speed);
}

/* PWMUpDown */
PacketStatus HBridgePWMUpDown_init(HBridge* bridge, int8_t* pins){
  bridge->ops=h_bridge_ops+HBridgeTypePWMUpDown;

  int8_t *pwm_pin=&pins[0];
  int8_t *fwd_pin=&pins[1];
  int8_t *bwd_pin=&pins[2];
  PacketStatus status=Success;
  if (PWM_setDutyCycle(*pwm_pin,0)<0
      || PWM_enable(*pwm_pin,1)<0) {
    *pwm_pin=bridge->params.pwmupdown.pwm_pin;
    status= WrongPins;
  }
  bridge->params.pwmupdown.pwm_pin=*pwm_pin;

  if( DigIO_setDirection(*fwd_pin,Output)<0
      || DigIO_setValue(*fwd_pin, 1) < 0) {
    *fwd_pin = bridge->params.pwmupdown.fwd_pin;
    status = WrongPins;
  }
  bridge->params.pwmupdown.fwd_pin = *fwd_pin;

  if( DigIO_setDirection(*bwd_pin,Output)<0
      || DigIO_setValue(*bwd_pin, 1) < 0) {
    *bwd_pin = bridge->params.pwmupdown.bwd_pin;
    status= WrongPins;
  }
  bridge->params.pwmupdown.bwd_pin = *bwd_pin;

  return status;
}

void HBridgePWMUpDown_fini(HBridge* bridge){
  uint8_t pwm_pin=bridge->params.pwmupdown.pwm_pin;
  uint8_t fwd_pin=bridge->params.pwmupdown.fwd_pin;
  uint8_t bwd_pin=bridge->params.pwmupdown.bwd_pin;
  PWM_setDutyCycle(pwm_pin,0);
  DigIO_setDirection(fwd_pin, 0);
  DigIO_setDirection(bwd_pin, 0);
}

void HBridgePWMUpDown_setSpeed(HBridge* bridge, int16_t speed){
  uint8_t pwm_pin=bridge->params.pwmupdown.pwm_pin;
  uint8_t fwd_pin=bridge->params.pwmupdown.fwd_pin;
  uint8_t bwd_pin=bridge->params.pwmupdown.bwd_pin;
  if (speed>255)
    speed=255;
  if (speed<-255)
    speed=-255;
  if (speed>0){
    DigIO_setValue(fwd_pin,1);
    DigIO_setValue(bwd_pin,0);
  } else {
    DigIO_setValue(fwd_pin,0);
    DigIO_setValue(bwd_pin,1);
    speed=-speed;
  }
  PWM_setDutyCycle(pwm_pin, speed);
}

PacketStatus HBridge_init(struct HBridge* bridge, const HBridgeType type, int8_t* pins){
  PacketStatus status;
  switch(type){
  case HBridgeTypePWMDir:
    status = HBridgePWMDir_init(bridge, pins);
    break;
  case HBridgeTypeDualPWM:
    status = HBridgeDualPWM_init(bridge, pins);
    break;
  case HBridgeTypeHalfCyclePWM:
    status = HBridgeHalfPWM_init(bridge, pins);
    break;
  case HBridgeTypePWMUpDown:
    status = HBridgePWMUpDown_init(bridge, pins);
    break;
  default:
    status = GenericError;
  }
  bridge->enabled=(status==Success);
  return status;
}

PacketStatus HBridge_setSpeed(HBridge* bridge, int16_t speed) {
  if (! bridge->enabled)
    return GenericError;
  if (! bridge->ops)
    return GenericError;
  if (! bridge->ops->setSpeed_fn)
    return GenericError;
  (*bridge->ops->setSpeed_fn)(bridge, speed);
  return 0;
}
