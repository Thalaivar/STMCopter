#pragma once

#include "mbed.h"
#include "MPU9250.h"

Serial pc(USBTX, USBRX);
Timer t, t1;
MPU9250 imu(PB_9, PB_8);
InterruptIn ppmPin(PB_5);
PwmOut esc1(PA_0);
PwmOut esc2(PA_1);
PwmOut esc3(PB_10);
PwmOut esc4(PB_4);
DigitalOut myled(LED1);


