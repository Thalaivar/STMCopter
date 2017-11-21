#pragma once
#ifndef ATTITUDE_H
#define ATTITUDE_H

#include "PERIPHERALS.h"
#include "math.h"

void getAngles(uint8_t print);

void getAngles(uint8_t print) {
      
     imu.readAccelData(accelCount); 
     ax = (float)accelCount[0]*aRes - accelBias[0];  
     ay = (float)accelCount[1]*aRes - accelBias[1];   
     az = (float)accelCount[2]*aRes - accelBias[2];  
   
     imu.readGyroData(gyroCount);  
     gx = (float)gyroCount[0]*gRes - gyroBias[0];  
     gy = (float)gyroCount[1]*gRes - gyroBias[1];  
     gz = (float)gyroCount[2]*gRes - gyroBias[2];   
  
     imu.readMagData(magCount); 
     mx = (float)magCount[0]*mRes*magCalibration[0] - magbias[0];  
     my = (float)magCount[1]*mRes*magCalibration[1] - magbias[1];  
     mz = (float)magCount[2]*mRes*magCalibration[2] - magbias[2];
                            
     float G = sqrt(ax*ax + ay*ay + az*az);
     pitch = asin(-ax/G)*180.0f/PI;
     roll = asin(ay/(G*cos(pitch*PI/180.0f)))*180.0f/PI;
     yaw = atan2(my, mx);
     yaw*=180.0f/PI;     
     
     if(print == 1) pc.printf("%f, %f, %f\n", roll, pitch, yaw);
}

    
#endif
