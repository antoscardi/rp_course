#pragma once
#include <stdint.h>
#include "orazio_packets.h"
#include "packet_operations.h"

#ifdef __cplusplus
extern "C" {
#endif

  struct HBridgeOps;

  typedef struct HBridge{
    struct HBridgeOps* ops;
    uint8_t enabled;
    union {
      struct {
        int8_t pwm_pin;
        int8_t dir_pin;
        int8_t brake_pin; // -1: disable
      } pwmdir;
      struct {
        int8_t pwm_forward_pin;
        int8_t pwm_backward_pin;
      } dualpwm;
      struct {
        int8_t pwm_pin;
        int8_t enable_pin;
      } halfpwm;
      struct {
        int8_t pwm_pin;
        int8_t fwd_pin;
        int8_t bwd_pin;
      } pwmupdown;
    } params;
  } HBridge;

  typedef void (*HBridgeFiniFn)(HBridge* bridge);
  typedef void (*HBridgeSetSpeedFn)(HBridge* bridge, int16_t speed);

  typedef struct HBridgeOps{
    HBridgeFiniFn fini_fn;
    HBridgeSetSpeedFn setSpeed_fn;
  }  HBridgeOps;



  PacketStatus HBridge_setSpeed(struct HBridge* bridge, int16_t speed);

  /**
     type=PWMDir   :  pins[0]=pwm, pins[1]=dir, pins[2]=brake;
     type=DualPWM  :  pins[0]=pwm_fwd, pins[1]=pwm_bwd;
     type=half_pwm :  pins[0]=pwm, pins[1]=enable;
     type=PwmUpDown :  pins[0]=pwm, pins[1]=fwd, pins[2]=bwd ;
     @returns 0 on success, <0 on failure
  */
  PacketStatus HBridge_init(struct HBridge* bridge, const HBridgeType type, int8_t* pins);
  

#ifdef __cplusplus
}
#endif
