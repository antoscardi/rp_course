/**
 * motor.cpp
 */

#include "motor.h"
extern "C" {
#include "encoder.h"
#include "pwm.h"
#include <math.h>
}

template <typename T>
static T clamp(T v, T value){
  if (v>value)
    return value;
  if (v<-value)
    return -value;
  return v;
}

Motor::Motor(uint8_t bit_rege_dira_, uint8_t bit_rege_dirb_, float kp_, float ki_, float kd_) : 
  _dira(bit_rege_dira_), _dirb(bit_rege_dirb_), _kp(kp_), _ki(ki_), _kd(kd_) {
    // TODO
    // Set _dira and _dirb as DIGIO Output pins
}

/**
 * @brief Enable or disable use of PID internal controller
 * 
 * @param pid_en_ 0 for PWM open loop control, 1 for PID controller
 */
void Motor::usePid(uint8_t pid_en_) {
  // TODO
  // Save PID enable flag
}

/**
 * @brief Set the desired velocity of the motor
 * If PID controller is enabled, issue desired ticks/interval, else
 * issue desired PWM
 * 
 * @param desired_speed_ 
 */
void Motor::setSpeed(int16_t desired_speed_) {
  // TODO
  // Store desired speed
}

/**
 * @brief Both compute the correction control and interface with
 * the DIGIO/PWM modules to control the joint.
 * Should be used in a time-invariant loop (TIMER5 is handy)
 */
void Motor::spinOnce(void) {
  // TODO
  // Compute previous and current encoder measurements (both uint16_t)
  // Compute measured speed (int16_t)
  // If using PID, then:
  //  Compute the previous and current state error (float)
  //  Clamp error with ramp_step value
  //  Compute integral component by accumulating ki * error
  //  Clamp the accumulator between +- 255
  //  Compute the derivative term
  //  Output the control term by summing the P,I,D components and by 
  //    clamping it to +-255
  // If not using PID, then:
  //  Output the control term equal to the desired speed

  // Issue control command to hardware
  // Careful to set the direction accordingly and 
  // clamp the absolute value of the output term
  return;
}