#pragma once
#ifndef PID_H
#define PID_H

#include "mbed.h"
#include "PPM.h" 
#include "PERIPHERALS.h"

void controlQuad(uint8_t print);       //set print to 1: thrust vals  2: PID vals
float PID_pitch, PID_roll, PID_yaw;
float setpoints[6];
float calculateErrInt(float err_int, float err, float errPrev, int deltat);
void convertToSetpoints();

class error{
        public:
            float integ, prev, Kp, Ki, Kd;
    }roll_e,pitch_e,yaw_e;

void controlQuad(uint8_t print){
        //calculate errors
        //first proportional error
        int deltat = t1.read();

        convertToSetpoints();
        float rollErr = setpoints[0] - imu.roll;
        float pitchErr = setpoints[1] - imu.pitch;
        float yawErr = (channelVal[3] - 1497)*0.6;
        
        float rollErrDot = -imu.gx;
        float pitchErrDot = -imu.gy;
        float yawErrDot = -imu.gz;
        
        //calculate integral error
        roll_e.integ = calculateErrInt(roll_e.integ, rollErr, roll_e.prev, deltat);
        pitch_e.integ = calculateErrInt(pitch_e.integ, pitchErr, pitch_e.prev, deltat);
        yaw_e.integ = calculateErrInt(yaw_e.integ, yawErr, yaw_e.prev, deltat);
        
        PID_roll = roll_e.Kp*rollErr + roll_e.Kd*rollErrDot + roll_e.Ki*roll_e.integ;
        PID_pitch = pitch_e.Kp*pitchErr + pitch_e.Kd*pitchErrDot + pitch_e.Ki*pitch_e.integ;
        PID_yaw = yaw_e.Kp*yawErr + yaw_e.Kd*yawErrDot + yaw_e.Ki*yaw_e.integ;
        
        //convertToMicroseconds(); need to convert PID outputs to esc microseconds
        
        roll_e.prev = rollErr;
        pitch_e.prev = pitchErr;
        yaw_e.prev = yawErr;
        
        int thrust1 = channelVal[2] - PID_pitch - PID_roll + PID_yaw;
        int thrust2 = channelVal[2] + PID_pitch - PID_roll - PID_yaw;
        int thrust3 = channelVal[2] + PID_pitch + PID_roll + PID_yaw;
        int thrust4 = channelVal[2] - PID_pitch + PID_roll - PID_yaw;
        
        if(thrust1 < 1000) thrust1 = 1000;
        if(thrust1 > 2000) thrust1 = 2000;
        if(thrust2 < 1000) thrust2 = 1000;
        if(thrust2 > 2000) thrust2 = 2000;
        if(thrust3 < 1000) thrust3 = 1000;
        if(thrust3 > 2000) thrust3 = 2000;
        if(thrust4 < 1000) thrust4 = 1000;
        if(thrust4 > 2000) thrust4 = 2000;
         
        esc1.pulsewidth_us(thrust1);
        esc2.pulsewidth_us(thrust2);
        esc3.pulsewidth_us(thrust3);
        esc4.pulsewidth_us(thrust4);
        
        if(print == 1) pc.printf("%d %d %d %d\n", thrust1, thrust2, thrust3, thrust4);
        else if(print == 2) pc.printf("%f %f %f\n", PID_roll, PID_pitch, PID_yaw);
        t1.reset();
    }

float calculateErrInt(float err_int, float err, float errPrev, int deltat){ //calculate integral of error for all three at omce else timer will fuck up
        
        //calculate addition to err_int
        float deltae = (deltat/2)*(errPrev + err);//reset t1
        
        //add to err_int
        return err_int + deltae;
    }

void convertToSetpoints(){
        for(int i = 0; i < 5; i++){
                if(i != 2){ //height setpoint will come from channel 3
                    if(channelVal[i] >= 1497 && channelVal[i] <= 1503){
                         setpoints[i] = 0;
                    }
                    
                    else{
                        setpoints[i] = (channelVal[i] - 1495)/16.67; //to get max setpoint of 10 degrees
                    }
                }
                
                else if(i == 2) setpoints[i] = (channelVal[i] - 1003)/200; //to get max height of 5 meters
        }
    }

#endif
