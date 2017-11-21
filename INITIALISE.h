#pragma once

#ifndef INITIALISE_H
#define INITIALISE_H

#include "PERIPHERALS.h"
#include "PPM.h"
#include "PID.h"

#define KP_ROLL     1.5
#define KP_PITCH    1.4
#define KP_YAW      -0.9
#define KD_ROLL     0.0775
#define KD_PITCH    0.0755
#define KD_YAW      -0.04
#define KI_ROLL     0.02
#define KI_PITCH    0.02
#define KI_YAW      0

#define MAGBIAS_X 98.89
#define MAGBIAS_Y 181.041
#define MAGBIAS_Z -20.38


void initialiseIMU();
void initialisePeripherals(); //call first
void initialiseErrInt();
void initialiseTimers(); //call before main loop
void initialiseGains();
void initialiseMagBias();

void initialiseIMU(){
            
            uint8_t address = imu.readByte(MPU9250_ADDRESS, WHO_AM_I_MPU9250);
            pc.printf("%x\n",address);
            if(address == 0x73){
                pc.printf("Resetting IMU....");
                imu.resetMPU9250();
                myled=!myled;
                wait(1);
                myled=!myled;
                pc.printf("Calibrating IMU...");
                imu.calibrateMPU9250(gyroBias, accelBias);
                myled=!myled;
                wait(1);
                myled=!myled;
                imu.initMPU9250();
                pc.printf("IMU initialised!\n");
                myled=!myled;
                wait(2);
                myled=!myled;
                imu.initAK8963(magCalibration);
                myled=!myled;
                wait(1);
                myled=!myled;
                pc.printf("AK8963 initialized for active data mode....\n\r");
                imu.getAres();
                imu.getGres();
                imu.getMres();
            }
            
            else{
                while(1){
                    myled = !myled;
                    wait(0.5);
                   }
                }                            
    }

void initialisePeripherals() {
        pc.baud(57600);
        myled=!myled;
        wait(1);
        myled=!myled;
        wait(1);
        myled=!myled;
        ppmPin.rise(&measureChannel);
    }

void initialiseErrInt(){
        roll_e.integ = 0;
        pitch_e.integ = 0;
        yaw_e.integ = 0;
    }

void initialiseGains(){
        roll_e.kp  = KP_ROLL;
        pitch_e.kp = KP_PITCH;
        yaw_e.kp   = KP_YAW;
        roll_e.Kd  = KD_ROLL;
        pitch_e.Kd = KD_PITCH;
        yaw_e.Kd   = KD_YAW;
        roll_e.ki  = KI_ROLL;
        pitch_e.ki = KI_PITCH;
        yaw_e.ki   = KI_YAW;  
}

void initialiseTimers(){
        t.reset();
        //t1.stop();
        t.start();
        t1.start();
    }

void initialiseMagBias(){
        magbias[0]  = MAGBIAS_X;
        magbias[1]  = MAGBIAS_Y;
        magbias[2]  = MAGBIAS_Z;
    }
//add optional magnetometer calibration func.
#endif
