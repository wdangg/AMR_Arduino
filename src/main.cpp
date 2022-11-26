#include "head.h"

#define MAX_PWM											(200)
#define MIN_PWM											(40)
#define CONTROL_TIMEOUT									(1)

#define INTERVAL_TIME									(50)
#define PPR												(2000)
#define PI												(3.14)
#define DIAMETER										(0.064)
#define PERIMETER										((PI)*(DIAMETER))
#define WHEEL_BASE										(0.25)
#define TICKS_PER_METER									(9000)

#define PWM_TURN										(50)

void PID_Init();
void ps2_control();
void control_motor();

void ISR_Right_Ticks();
void ISR_Left_Ticks();

volatile bool right_dir = true;
volatile bool left_dir = true;
volatile int32_t right_ticks = 0; 
volatile int32_t left_ticks = 0;

volatile bool reset_board = false;

volatile int l_pwm_out = 0, r_pwm_out = 0;
volatile float l_vel_request = 0, r_vel_request = 0;

volatile geometry_msgs::Twist receive_twist;
void resetBoard();
void calc_cmd_vel(const geometry_msgs::Twist& cmdVel);


ros::NodeHandle nh;

ros::Subscriber<geometry_msgs::Twist> sub_cmd_vel("cmd_vel", &calc_cmd_vel);



void setup()
{
	nh.getHardware()->setBaud(115200);
	nh.initNode();
	// nh.advertise(rightPub);
	// nh.advertise(leftPub);
	nh.subscribe(sub_cmd_vel);

	Serial3.begin(115200);
	Serial3.print("\n\n\n\n\n\n[INFO] Start Application\n\n");

	reset_board = false;
	right_dir = true;
	left_dir = true;
	PID_Init();
	system_setup(); 

	// attachInterrupt(digitalPinToInterrupt(ENC_LEFT_A), ISR_Left_Ticks, RISING);
	// attachInterrupt(digitalPinToInterrupt(ENC_RIGHT_A), ISR_Right_Ticks, RISING);

	delay(600);
} /* END SET_UP */ 

void loop()
{
	nh.spinOnce();

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

	if (reset_board == true)
	{
		resetBoard();
	}

	if (manual_mode == true)
	{
		ps2_control();
	}
	
	/* Publish Data */
	if (millis() - pre_millis > INTERVAL_TIME)
	{
		/* Calculate Velocity */
		// static int32_t left_vel = calc_wheel_vel(&left_ticks);
		// static int32_t right_vel = calc_wheel_vel(&right_ticks);

		/* Publishing Data */
		// calc_pwm_out();

		/* Update Previous Mills Variable */
		pre_millis = millis();
	}

	/* Control Motor After Calculate These Parameters */
	control_motor();

	/* Timeout Control Robot */
	if ( ((millis()/1000)-last_cmd_receive) > CONTROL_TIMEOUT)
	{
		if (manual_mode == false)
		{
			l_pwm_out = 0;
			r_pwm_out = 0;
		}
	}

	// Serial3.print("left_ticks: "); Serial3.print(left_ticks);
	// Serial3.print("\t\tright_ticks: "); Serial3.println(right_ticks);

	// Serial3.print("l_vel_request: "); Serial3.print(receive_twist.linear.x);
	// Serial3.print("\t\tr_vel_request: "); Serial3.println(receive_twist.angular.z);
} /* END LOOP */

void ISR_Right_Ticks()
{
	/* Encoder_Right_A -  RISING Mode */
	int8_t val = digitalRead(ENC_RIGHT_B);

	if (1 == val)
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
	/* Encoder_Right_A -  RISING Mode */
	int8_t val = digitalRead(ENC_LEFT_B);

	if (0 == val)
	{
		left_ticks--;
	} 
	else 
	{
		left_ticks++;
	}
}

void calc_cmd_vel(const geometry_msgs::Twist& cmdVel)
{
	last_cmd_receive = millis()/1000;

	if (cmdVel.angular.z != 0)
	{
		if (cmdVel.angular.z < 0)
		{
			l_pwm_out = PWM_TURN;
			r_pwm_out = -PWM_TURN;
		}
		else
		{
			l_pwm_out = -PWM_TURN;
			r_pwm_out = PWM_TURN;
		}
	}
	else
	{
		if (cmdVel.linear.x > 0)
		{
			l_pwm_out = PWM_TURN;
			r_pwm_out = PWM_TURN;
		}
		else if (cmdVel.linear.x < 0)
		{
			l_pwm_out = -PWM_TURN;
			r_pwm_out = -PWM_TURN;
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
		l_pwm_out = PWM_TURN;
		r_pwm_out = PWM_TURN;
	}
	else if (ps2x.ButtonPressed(PSB_CIRCLE))
	{
		l_pwm_out = PWM_TURN;
		r_pwm_out = -PWM_TURN;
	}
	else if (ps2x.ButtonPressed(PSB_SQUARE))
	{
		l_pwm_out = -PWM_TURN;
		r_pwm_out = PWM_TURN;
	}
	else if (ps2x.ButtonPressed(PSB_CROSS))
	{
		l_pwm_out = -PWM_TURN;
		r_pwm_out = -PWM_TURN;
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
	left_PID.SetOutputLimits(-200, 200);
	
	right_PID.SetMode(AUTOMATIC);
	right_PID.SetSampleTime(1);
	right_PID.SetOutputLimits(-200, 200);
}

void resetBoard()
{
  asm volatile ( "jmp 0");  
}
