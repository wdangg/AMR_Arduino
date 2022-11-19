#include "head.h"

#define MAX_PWM											200
#define MIN_PWM											40
#define INTERVAL_TIME									100
#define CONTROL_TIMEOUT									2


void PID_Init();
void ps2_control();
void control_motor();

void calc_cmd_vel(const geometry_msgs::Twist& cmdVel);
void ISR_Right_Ticks();
void ISR_Left_Ticks();

volatile bool right_dir = true;
volatile bool left_dir = true;
volatile int32_t right_ticks = 0;
volatile int32_t left_ticks = 0;

volatile bool reset_board = false;

ros::NodeHandle nh;

ros::Subscriber<geometry_msgs::Twist> sub_cmd_vel("cmd_vel", &calc_cmd_vel);

void resetBoard();

void setup()
{
	// nh.getHardware()->setBaud(57600);

	// nh.initNode();
	// nh.advertise(rightPub);
	// nh.advertise(leftPub);
	// nh.subscribe(sub_cmd_vel);
	reset_board = false;
	right_dir = true;
	left_dir = true;

	system_setup(); 

	attachInterrupt(digitalPinToInterrupt(ENC_LEFT_A), ISR_Left_Ticks, RISING);
	attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_A), ISR_Right_Ticks, RISING);

	delay(1000);
} /* END SET_UP */ 

void loop()
{
	// nh.spinOnce();
	if (reset_board == true)
	{
		resetBoard();
	}

	ps2x.read_gamepad(false, vibrate);

	if (ps2x.ButtonReleased(PSB_R1))
	{
		manual_mode = !manual_mode;
		digitalWrite(ANODE_LED, !digitalRead(ANODE_LED));
	}
	else if (ps2x.ButtonReleased(PSB_R2))
	{
		reset_board = true;
	}


	if (manual_mode == true)
	{
		ps2_control();
	}
	
	/* Publish Data */
	if (millis() - pre_millis > INTERVAL_TIME)
	{
		/* Publishing Data */

		/* Update Previous Mills Variable */
		pre_millis = millis();
	}

	/* Timeout Control Robot */
	if ( ((millis()/1000)-last_cmd_receive) > CONTROL_TIMEOUT )
	{
		if (manual_mode == false)
		{
			l_pwm_out = 0;
			r_pwm_out = 0;
		}
	}

	/* Control Motor After Calculate These Parameters */
	control_motor();

	Serial3.print("left_ticks: "); Serial3.print(left_ticks);
	Serial3.print("\t\tright_ticks: "); Serial3.println(right_ticks);

	// Serial3.print("A: "); Serial3.print(digitalRead(ENC_LEFT_A));
	// Serial3.print("\t\tB: "); Serial3.println(digitalRead(ENC_LEFT_B));

} /* END LOOP */

void ISR_Right_Ticks()
{
	// Encoder Right A - attachInterrupts - RISING Mode
	int8_t val = digitalRead(ENC_RIGHT_B);
	if (val == 0)
	{
		right_ticks--;
	}
	else
	{
		right_ticks++;
	}
}

void ISR_Left_Ticks()
{
	int8_t val = digitalRead(ENC_LEFT_B);
	if (val == 0)
	{
		left_ticks++;
	}
	else
	{
		left_ticks--;
	}
}

void calc_cmd_vel(const geometry_msgs::Twist& cmdVel)
{
	last_cmd_receive = millis()/1000;

	if (cmdVel.angular.z != 0)
	{
		if (cmdVel.angular.z < 0)
		{
			l_pwm_out = 50;
			r_pwm_out = -50;
		}
		else
		{
			l_pwm_out = -50;
			r_pwm_out = 50;
		}
	}
	else
	{
		if (cmdVel.linear.x > 0)
		{
			l_pwm_out = 50;
			r_pwm_out = 50;
		}
		else if (cmdVel.linear.x < 0)
		{
			l_pwm_out = -50;
			r_pwm_out = -50;
		}
		else
		{
			l_pwm_out = 0;
			r_pwm_out = 0;
		}
	}
} /* CALC_CMD_VEL */


void ps2_control()
{
	digitalWrite(ANODE_LED, HIGH);
	if (ps2x.ButtonPressed(PSB_TRIANGLE))
	{
		l_pwm_out = 50;
		r_pwm_out = 50;
	}
	else if (ps2x.ButtonPressed(PSB_CIRCLE))
	{
		l_pwm_out = 50;
		r_pwm_out = -50;
	}
	else if (ps2x.ButtonPressed(PSB_SQUARE))
	{
		l_pwm_out = -50;
		r_pwm_out = 50;
	}
	else if (ps2x.ButtonPressed(PSB_CROSS))
	{
		l_pwm_out = -50;
		r_pwm_out = -50;
	}
	else if (ps2x.ButtonReleased(PSB_TRIANGLE) || ps2x.ButtonReleased(PSB_CIRCLE)
			|| ps2x.ButtonReleased(PSB_SQUARE) || ps2x.ButtonReleased(PSB_CROSS))
	{
		l_pwm_out = 0;
		r_pwm_out = 0;
	}
} /* PS2_CONTROL */

void control_motor()
{
	left_motor.rotate(l_pwm_out);
	right_motor.rotate(r_pwm_out);
} /* CONTROL_MOTOR */

void PID_Init()
{
	left_PID.SetMode(AUTOMATIC);
	left_PID.SetSampleTime(1);
	left_PID.SetOutputLimits(0, 100);
	
	right_PID.SetMode(AUTOMATIC);
	right_PID.SetSampleTime(1);
	right_PID.SetOutputLimits(0, 100);
}

void resetBoard()
{
  asm volatile ( "jmp 0");  
}
