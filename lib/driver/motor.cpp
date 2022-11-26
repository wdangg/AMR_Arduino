#include "motor.h"


Motor::Motor(int enable_pin, int cw_pin, int ccw_pin, int encoder_A, int encoder_B, ID_Type id, Dir_Type dir)
{
    Motor::enable_pin = enable_pin;
    Motor::cw_pin = cw_pin;
    Motor::ccw_pin = ccw_pin;
    Motor::encoder_A = encoder_A;
    Motor::encoder_B = encoder_B;    
    Motor::id = id;
    Motor::dir = e_STOP;
};

void Motor::init()
{
    pinMode(enable_pin, OUTPUT);
    pinMode(cw_pin, OUTPUT);
    pinMode(ccw_pin, OUTPUT);
    pinMode(encoder_A, INPUT_PULLUP);
    pinMode(encoder_B, INPUT_PULLUP);
};

void Motor::rotate(int pwm)
{
    pwm = constrain(pwm, -255, 255);
    if (pwm >= 0)
    {
        digitalWrite(cw_pin, HIGH);
        digitalWrite(ccw_pin, LOW);
        analogWrite(enable_pin, pwm);
        Motor::current_pwm = pwm;
        Motor::dir = e_CW;
    }
    else
    {
        digitalWrite(cw_pin, LOW);
        digitalWrite(ccw_pin, HIGH);
        analogWrite(enable_pin, (-pwm));
        Motor::current_pwm = (-pwm);   
        Motor::dir = e_CCW;
    }  
};

void Motor::set_dir(bool dir)
{
    if (dir == 1)
    {
        digitalWrite(cw_pin, HIGH);
        digitalWrite(ccw_pin, LOW);
    }
    else 
    {
        digitalWrite(cw_pin, LOW);
        digitalWrite(ccw_pin, HIGH);
    }
}

Dir_Type Motor::get_dir()
{
    return Motor::dir;
}

void Motor::stop()
{
    analogWrite(enable_pin, LOW);
    Motor::dir = e_STOP;
};

void Motor::get_info(HardwareSerial &p_serial)
{
        p_serial.println();
        p_serial.print("[INFO] enable_pin: ");   p_serial.print(Motor::enable_pin);
        p_serial.print("\tcw_pin: ");     p_serial.print(Motor::cw_pin);
        p_serial.print("\tccw_pin: ");    p_serial.print(Motor::ccw_pin);
        p_serial.print("\tencoder_A: ");  p_serial.print(Motor::encoder_A);
        p_serial.print("\tencoder_B: ");  p_serial.println(Motor::encoder_B);    
}

void Motor::get_current_pwm(HardwareSerial &p_serial)
{
    p_serial.println();
    p_serial.print("[INFO] ID: "); 
    p_serial.print(Motor::id);
    p_serial.print("\tpwm: "); p_serial.println(Motor::current_pwm);
};





