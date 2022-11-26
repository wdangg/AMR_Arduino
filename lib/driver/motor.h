#ifndef __MOTOR_H__
#define __MOTOR_H__

#include <Arduino.h>

typedef enum motor_id
{
    LEFT_ID,
    RIGHT_ID
} ID_Type;

typedef enum current_dir
{
    e_CCW,
    e_CW,
    e_STOP
} Dir_Type;

class Motor 
{
    private:
        int enable_pin;
        int cw_pin;
        int ccw_pin;
        int encoder_A;
        int encoder_B;
        int current_pwm;
        ID_Type id;
        Dir_Type dir;

    public:
        Motor(int enable_pin, int cw_pin, int ccw_pin, int encoder_A, int encoder_B, ID_Type id, Dir_Type dir);

        void init();

        void set_dir(bool dir);

        void rotate(int pwm);

        void stop();

        void get_current_pwm(HardwareSerial &p_serial);

        void get_info(HardwareSerial &p_serial);

        Dir_Type get_dir();
};


#endif /*  __MOTOR_H__ */
