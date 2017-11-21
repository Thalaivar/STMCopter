#pragma once

#include "mbed.h"
#include "MPU9250.h"

#define ESC_MOTOR1 PA_0
#define ESC_MOTOR2 PA_1
#define ESC_MOTOR3 PB_10
#define ESC_MOTOR4 PB_4

Serial pc(PA_2, PA_3);  //d10, d2
Timer t, t1;
MPU9250 imu;
InterruptIn ppmPin(PB_5);
PwmOut esc1(ESC_MOTOR1);
PwmOut esc2(ESC_MOTOR2);
PwmOut esc3(ESC_MOTOR3);
PwmOut esc4(ESC_MOTOR4);
DigitalOut myled(LED1);
DigitalOut power(PC_1);
DigitalOut gnd(PC_0);


