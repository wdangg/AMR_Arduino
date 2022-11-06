#include "head.h"

#define MAX_PWM								200
#define MIN_PWM								40

volatile bool manual_mode = false;

volatile int16_t l_pwm_out = 0;
volatile int16_t r_pwm_out = 0;

void control_motor(uint8_t *l_val, uint8_t *r_val);
void calc_cmd_vel(const geometry_msgs::Twist& cmdVel);


ros::NodeHandle nh;

ros::Subscriber<geometry_msgs::Twist> sub_cmd_vel("cmd_vel", &calc_cmd_vel);

void setup()
{
	nh.getHardware()->setBaud(57600);

	nh.initNode();
	// nh.advertise(rightPub);
	// nh.advertise(leftPub);
	nh.subscribe(sub_cmd_vel);

	error = ps2x.config_gamepad(PS2_CLK, PS2_CMD, PS2_SEL, PS2_DAT, PRESSURES, RUMBLE);
	
	TCCR1B = TCCR1B & 0b11111000 | 1;      /* set 31KHz PWM to prevent motor noise */ 
	
	Serial3.begin(9600);

	left_motor.init();
	right_motor.init();

	left_PID.SetMode(AUTOMATIC);
	left_PID.SetSampleTime(1);
	left_PID.SetOutputLimits(0, 100);
	
	right_PID.SetMode(AUTOMATIC);
	right_PID.SetSampleTime(1);
	right_PID.SetOutputLimits(0, 100);


} /* END SET_UP */ 

void loop()
{
	nh.spinOnce();

	ps2x.read_gamepad(false, vibrate);

	if (ps2x.ButtonReleased(PSB_R1))
	{
		manual_mode = !manual_mode;
	}
	
	if (manual_mode == true)
	{
		if (ps2x.ButtonPressed(PSB_TRIANGLE))
		{
			left_motor.rotate(50);
			right_motor.rotate(50);
		}
		else if (ps2x.ButtonPressed(PSB_CIRCLE))
		{
			left_motor.rotate(50);
			right_motor.rotate(-50);
		}
		else if (ps2x.ButtonPressed(PSB_SQUARE))
		{
			left_motor.rotate(-50);
			right_motor.rotate(50);
		}
		else if (ps2x.ButtonPressed(PSB_CROSS))
		{
			left_motor.rotate(-50);
			right_motor.rotate(-50);
		}
		else if (ps2x.ButtonReleased(PSB_TRIANGLE) || ps2x.ButtonReleased(PSB_CIRCLE)
		      || ps2x.ButtonReleased(PSB_SQUARE) || ps2x.ButtonReleased(PSB_CROSS))
		{
			left_motor.stop();
			right_motor.stop();
		}
	}

	
		
} /* END LOOP */

void calc_cmd_vel(const geometry_msgs::Twist& cmdVel)
{
	if (cmdVel.angular.z != 0)
	{
		if (cmdVel.angular.z < 0)
		{
			// left_motor.rotate(50);
			// right_motor.rotate(-50);
			l_pwm_out = 50;
			r_pwm_out = -50;
		}
		else
		{
			// left_motor.rotate(-50);
			// right_motor.rotate(50);
			l_pwm_out = -50;
			r_pwm_out = 50;
		}
	}
	else
	{
		if (cmdVel.linear.x > 0)
		{
			// left_motor.rotate(50);
			// right_motor.rotate(50);
			l_pwm_out = 50;
			r_pwm_out = 50;
		}
		else if (cmdVel.linear.x < 0)
		{
			// left_motor.rotate(-50);
			// right_motor.rotate(-50);
			l_pwm_out = -50;
			r_pwm_out = -50;
		}
		else
		{
			// left_motor.stop();
			// right_motor.stop();
			l_pwm_out = 0;
			r_pwm_out = 0;
		}
		
	}
} /* CALC_CMD_VEL */

void control_motor(int16_t *l_val, int16_t *r_val)
{
	left_motor.rotate(l_val);
	right_motor.rotate(r_val);

} /* CONTROL_MOTOR */


