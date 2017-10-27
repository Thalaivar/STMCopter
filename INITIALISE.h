#pragma once
#ifndef INITIALISE_H
#define INITIALISE_H

#include "PERIPHERALS.h"
#include "PPM.h"
#include "PID.h"

void initialiseIMU();
void initialisePeripherals(); //call first
void initialiseErrInt();
void initialiseTimers(); //call before main loop

void initialiseIMU(){
            
            
            pc.printf("Resetting IMU....");
            imu.resetMPU9250();
            wait(1);
            pc.printf("Calibrating IMU...");
            imu.calibrateMPU9250(imu.gyroBias, imu.accelBias);
            wait(2);
            imu.initMPU9250();
            pc.printf("IMU initialised!\n");
            wait(2);
            imu.initAK8963(imu.magCalibration);
            wait(1);
            pc.printf("AK8963 initialized for active data mode....\n\r");
            imu.getAres();
            imu.getGres();
            imu.getMres();
    }

void initialisePeripherals() {
        pc.baud(57600);
        //i2c.frequency(400000);
        wait(2);
        ppmPin.rise(&measureChannel);
    }

void initialiseErrInt(){
        roll_e.integ = 0;
        pitch_e.integ = 0;
        yaw_e.integ = 0;
    }

void initialiseTimers(){
        t.reset();
        t1.reset();
    }
#endif
