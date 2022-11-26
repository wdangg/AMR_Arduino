#ifndef __MOTOR_CALC_H__
#define __MOTOR_CALC_H__


#include "motor.h"

/* Calculate Actual Robot Velocity */
uint32_t calc_wheel_vel(int32_t *ticks);

/* Compute PWM */
// uint32_t calc_pwm_out(int32_t *des_pwm, const );


#endif /* __MOTOR_CALC_H__ */

