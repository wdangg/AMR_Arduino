#include "head.h"

#define MAX_PWM											200
#define MIN_PWM											40
#define INTERVAL_TIME									20
#define CONTROL_TIMEOUT									2

volatile bool manual_mode = false;

volatile int cb_pwma = 0, cb_pwmb;
volatile int l_pwm_out = 0, r_pwm_out = 0;

volatile unsigned long pre_millis = 0, cur_millis = 0;

volatile unsigned long last_cmd_receive = 0;

void PID_Init();
void ps2_control();
void control_motor();

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
	Serial3.println("Serial 3");

	/* Initialize Motor */
	left_motor.init();
	right_motor.init();

	left_motor.stop();
	right_motor.stop();

	pinMode(CATHODE_LED, OUTPUT); 
	pinMode(ANODE_LED, OUTPUT);
	digitalWrite(CATHODE_LED, LOW); 
	digitalWrite(ANODE_LED, 0);

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
		
} /* END LOOP */

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
	Serial3.print(l_pwm_out); Serial3.print("\t"); Serial3.println(r_pwm_out);
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

