#include "driver.h"


Motor::Motor(int enable_pin, int cw_pin, int ccw_pin, int encoder_A, int encoder_B, ID_Type id)
{
    Motor::enable_pin = enable_pin;
    Motor::cw_pin = cw_pin;
    Motor::ccw_pin = ccw_pin;
    Motor::encoder_A = encoder_A;
    Motor::encoder_B = encoder_B;    
    Motor::id = id;
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
    if (pwm >= 0)
    {
        digitalWrite(cw_pin, HIGH);
        digitalWrite(ccw_pin, LOW);
        analogWrite(enable_pin, pwm);
        Motor::current_pwm = pwm;
    }
    else
    {
        digitalWrite(cw_pin, LOW);
        digitalWrite(ccw_pin, HIGH);
        analogWrite(enable_pin, (-pwm));  
        Motor::current_pwm = (-pwm);     
    }  
};

void Motor::stop()
{
    analogWrite(enable_pin, 0);
};

void Motor::print_info(Serial_Type serial_type)
{
    switch (serial_type)
    {
    case e_SERIAL3:
        Serial3.println();
        Serial3.print("enable_pin: ");   Serial3.print(Motor::enable_pin);
        Serial3.print("\tcw_pin: ");     Serial3.print(Motor::cw_pin);
        Serial3.print("\tccw_pin: ");    Serial3.print(Motor::ccw_pin);
        Serial3.print("\tencoder_A: ");  Serial3.print(Motor::encoder_A);
        Serial3.print("\tencoder_B: ");  Serial3.println(Motor::encoder_B);       
        break;
    case e_SERIAL2:
        Serial2.println();
        Serial2.print("enable_pin: ");   Serial2.println(Motor::enable_pin);
        Serial2.print("\tcw_pin: ");     Serial2.print(Motor::cw_pin);
        Serial2.print("\tccw_pin: ");    Serial2.print(Motor::ccw_pin);
        Serial2.print("\tencoder_A: ");  Serial2.print(Motor::encoder_A);
        Serial2.print("\tencoder_B: ");  Serial2.println(Motor::encoder_B); 
    case e_SERIAL1:
        Serial1.println();
        Serial1.print("enable_pin: ");   Serial1.println(Motor::enable_pin);
        Serial1.print("\tcw_pin: ");     Serial1.print(Motor::cw_pin);
        Serial1.print("\tccw_pin: ");    Serial1.print(Motor::ccw_pin);
        Serial1.print("\tencoder_A: ");  Serial1.print(Motor::encoder_A);
        Serial1.print("\tencoder_B: ");  Serial1.println(Motor::encoder_B); 
    case e_SERIAL:
        Serial3.println();
        Serial3.print("enable_pin: ");   Serial.println(Motor::enable_pin);
        Serial3.print("\tcw_pin: ");     Serial.print(Motor::cw_pin);
        Serial3.print("\tccw_pin: ");    Serial.print(Motor::ccw_pin);
        Serial3.print("\tencoder_A: ");  Serial.print(Motor::encoder_A);
        Serial3.print("\tencoder_B: ");  Serial.println(Motor::encoder_B);    
    default:
        break;
    }

};

void Motor::get_current_pwm(Serial_Type serial_port)
{
    switch (serial_port)
    {
    case e_SERIAL:
        Serial.println();
        Serial.print("ID: "); 
        Serial.print(Motor::id);
        Serial.print("\tpwm: "); Serial.println(Motor::current_pwm);
        break;
    case e_SERIAL1:
        Serial1.println();
        Serial1.print("ID: "); 
        Serial1.print(Motor::id);
        Serial1.print("\tpwm: "); Serial1.println(Motor::current_pwm);
        break;
    case e_SERIAL2:
        Serial2.println();
        Serial2.print("ID: "); 
        Serial2.print(Motor::id);
        Serial2.print("\tpwm: "); Serial2.println(Motor::current_pwm);
        break;
    case e_SERIAL3:
        Serial3.println();
        Serial3.print("ID: "); 
        Serial3.print(Motor::id);
        Serial3.print("\tpwm: "); Serial3.println(Motor::current_pwm);
        break;  
    default:
        break;
    }
};





