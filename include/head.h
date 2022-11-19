#ifndef __HEAD_H__
#define __HEAD_H__

#include "middleware.h"
#include "MPU6050_tockn.h"
#include "PS2X_lib.h"

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

#define ENC_LEFT_A           					18
#define ENC_LEFT_B           					19

#define ENC_RIGHT_A          					20
#define ENC_RIGHT_B          					21

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
int8_t error = 0;
int8_t type = 0;
int8_t vibrate = 0;
volatile bool manual_mode = false;
volatile int cb_pwma = 0, cb_pwmb;
volatile int l_pwm_out = 0, r_pwm_out = 0;
volatile uint32_t pre_millis = 0, cur_millis = 0;
volatile uint32_t last_cmd_receive = 0;

double left_PID_Setpoint, left_PID_Input, left_PID_Output;
double right_PID_Setpoint, right_PID_Input, right_PID_Output;

double l_Kp=2, l_Ki=5, l_Kd=1;
double r_Kp=2, r_Ki=5, r_Kd=1;

PID left_PID(&left_PID_Input, &left_PID_Output, &left_PID_Setpoint, l_Kp, l_Ki, l_Kd, DIRECT);
PID right_PID(&right_PID_Input, &right_PID_Output, &right_PID_Setpoint, r_Kp, r_Ki, r_Kd, DIRECT);


Motor left_motor(ENA, IN1, IN2, ENC_LEFT_A, ENC_LEFT_B, LEFT_ID);
Motor right_motor(ENB, IN3, IN4, ENC_RIGHT_A, ENC_RIGHT_B, RIGHT_ID);




void system_setup()
{
	cli(); /* Disable Global Interrupts */
	error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, PRESSURES, RUMBLE);
	
	TCCR1B = TCCR1B & 0b11111000 | 1;      /* set 31KHz PWM to prevent motor noise */ 
	
	Serial3.begin(9600);
	Serial3.print("\n\n\n\n\n\nStart Application\n\n\n\n");

	/* Initialize Motor */
	left_motor.init();
	right_motor.init();

	left_motor.stop();
	right_motor.stop();

	pinMode(CATHODE_LED, OUTPUT); 
	pinMode(ANODE_LED, OUTPUT);
	digitalWrite(CATHODE_LED, LOW); 
	digitalWrite(ANODE_LED, 0);

	sei(); /* Enable Global Interrupts */
}


#endif /* __HEAD_H__ */