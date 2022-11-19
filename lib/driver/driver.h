#ifndef __DRIVER_H__
#define __DRIVER_H__

#include <Arduino.h>
#include <ros.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int64.h>
#include <geometry_msgs/Twist.h>
#include <PID_v1.h>
#include <PS2X_lib.h>



typedef enum serial
{
    e_SERIAL,
    e_SERIAL1,
    e_SERIAL2,
    e_SERIAL3
} Serial_Type;

typedef enum motor_id
{
    LEFT_ID,
    RIGHT_ID
} ID_Type;


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

    public:
        Motor(int enable_pin, int cw_pin, int ccw_pin, int encoder_A, int encoder_B, ID_Type id);

        void init();

        void rotate(int pwm);

        void stop();

        void get_current_pwm(Serial_Type serial_port);

        void print_info(Serial_Type serial_type);
};


#endif /*  __DRIVER_H__ */
