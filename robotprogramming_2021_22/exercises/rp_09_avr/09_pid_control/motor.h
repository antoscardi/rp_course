/**
 * motor.h
 */

#pragma once

#include <stdint.h>
#include <stdlib.h>
extern "C" {
#include "digio.h"
}

class Motor {
public:
    // ctor
    Motor(uint8_t bit_rege_dira_, uint8_t bit_rege_dirb_, float kp_, float ki_, float kd_);

    /**
     * @brief Enable or disable use of PID internal controller
     * 
     * @param pid_en_ 0 for PWM open loop control, 1 for PID controller
     */
    void usePid(uint8_t pid_en_);
    
    /**
     * @brief Set the desired velocity of the motor
     * If PID controller is enabled, issue desired ticks/interval, else
     * issue desired PWM
     * 
     * @param desired_speed_ 
     */
    void setSpeed(int16_t desired_speed_);

    /**
     * @brief Both compute the correction control and interface with
     * the DIGIO/PWM modules to control the joint.
     * Should be used in a time-invariant loop (TIMER5 is handy)
     */
    void spinOnce(void);
protected:
    // Given variables (to let the compiler and you happy)
    // Interface
    uint8_t _dira, _dirb;
    // PID parameters
    float _kp, _ki, _kd;
    // Internal data
    float _error = 0.0f;
    int16_t _des_speed = 0;
    int16_t _mes_speed = 0;
    // TODO:
    // We need some stuff:
    //  accumulator for integral term (keep in float)
    //  an enable flag for PID control
    //  encoder measurement (keep uint16_t)
    //  a ramp_step variable to clamp the error at every iteration (a value of 5. should be fine)    
public:
    // Internal methods setters/getters
    inline float &kp(void) {return _kp;}
    inline float &ki(void) {return _ki;}
    inline float &kd(void) {return _kd;}
    inline float error(void) const {return _error;}
    inline int16_t desired_speed(void) const {return _des_speed;}
    inline int16_t measured_speed(void) const {return _mes_speed;}
};