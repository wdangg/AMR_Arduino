#ifndef __HEAD_H__
#define __HEAD_H__

#include "middleware.h"
#include "MPU6050_tockn.h"


/**
 * @brief Definition
 * 
 */

#define PRESSURES   							false
#define RUMBLE      							false

#define PS2_DAT        							30    
#define PS2_CMD        							28
#define PS2_SEL        							26
#define PS2_CLK        							24

#define ENC_LEFT_A           					19
#define ENC_LEFT_B           					18

#define ENC_RIGHT_A          					21
#define ENC_RIGHT_B          					20

#define WHEEL_RADIUS							0.032 //d = 64mm
#define WHEEL_BASE								0.227

#define ENA										9
#define IN1										5
#define IN2										6

#define ENB										10
#define IN3										13
#define IN4										8

#define CATHODE_LED                             A1
#define ANODE_LED                               A3

/**
 * @brief Global Variables
 * 
 */

PS2X ps2x;
int error = 0;
byte type = 0;
byte vibrate = 0;

double left_PID_Setpoint, left_PID_Input, left_PID_Output;
double right_PID_Setpoint, right_PID_Input, right_PID_Output;

double l_Kp=2, l_Ki=5, l_Kd=1;
double r_Kp=2, r_Ki=5, r_Kd=1;

PID left_PID(&left_PID_Input, &left_PID_Output, &left_PID_Setpoint, l_Kp, l_Ki, l_Kd, DIRECT);
PID right_PID(&right_PID_Input, &right_PID_Output, &right_PID_Setpoint, r_Kp, r_Ki, r_Kd, DIRECT);


Motor left_motor(ENA, IN1, IN2, ENC_LEFT_A, ENC_LEFT_B, LEFT_ID);
Motor right_motor(ENB, IN3, IN4, ENC_RIGHT_A, ENC_RIGHT_B, RIGHT_ID);


/**
 * @brief Function Prototype 
 * 
 */



#endif /* __HEAD_H__ */