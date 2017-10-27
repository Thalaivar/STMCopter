#pragma once
#ifndef ATTITUDE_H
#define ATTITUDE_H

#include "PERIPHERALS.h"
#include "math.h"

void getAngles();
void printAngles(bool option);

void getAngles() {
      
     imu.readAccelData(imu.accelCount); 
     imu.ax = (float)imu.accelCount[0]*imu.aRes - imu.accelBias[0];  
     imu.ay = (float)imu.accelCount[1]*imu.aRes - imu.accelBias[1];   
     imu.az = (float)imu.accelCount[2]*imu.aRes - imu.accelBias[2];  
   
     imu.readGyroData(imu.gyroCount);  
     imu.gx = (float)imu.gyroCount[0]*imu.gRes - imu.gyroBias[0];  
     imu.gy = (float)imu.gyroCount[1]*imu.gRes - imu.gyroBias[1];  
     imu.gz = (float)imu.gyroCount[2]*imu.gRes - imu.gyroBias[2];   
  
     imu.readMagData(imu.magCount); 
     imu.mx = (float)imu.magCount[0]*imu.mRes*imu.magCalibration[0] - imu.magbias[0];  
     imu.my = (float)imu.magCount[1]*imu.mRes*imu.magCalibration[1] - imu.magbias[1];  
     imu.mz = (float)imu.magCount[2]*imu.mRes*imu.magCalibration[2] - imu.magbias[2];
                            
     float G = sqrt(imu.ax*imu.ax + imu.ay*imu.ay + imu.az*imu.az);
     imu.pitch = asin(-imu.ax/G)*180.0f/PI;
     imu.roll = asin(imu.ay/(G*cos(imu.pitch*PI/180.0f)))*180.0f/PI;
     imu.yaw = atan2(imu.my, imu.mx);
     imu.yaw*=180.0f/PI;     
}

void printAngles(bool option){
    if(option) pc.printf("%f, %f, %f\n", imu.roll, imu.pitch, imu.yaw);
    }
    
#endif
