#pragma once

#ifndef PID_H
#define PID_H

#include "mbed.h"
#include "PPM.h" 
#include "PERIPHERALS.h"

#define setpointScaler       12.575
#define yawSetpointScaler    12.575
#define heightSetpointScaler 200
#define MAX_THRUST_VAL       2000
#define MIN_THRUST_VAL       1000

void   controlQuad(uint8_t print);       //set print to 1: thrust vals  2: PID vals 3: proportional errors 4: derivative errors 5: integral errors 0: no print
float  PID_pitch, PID_roll, PID_yaw;
float  setpoints[6];
float  calculateErrInt(float err_int, float err, float errPrev, float deltat);
void   convertToSetpoints();

class error{
        public:
            float integ;
            float prev;
            float kp;
            float ki;
            float Kd;
        }roll_e, pitch_e, yaw_e;

void controlQuad(uint8_t print){
        //calculate errors
        //first proportional error
        
        float deltat = t1.read_us()/1000000.0f;

        convertToSetpoints();
        float rollErr   =   setpoints[0] - roll;
        float pitchErr  =   setpoints[1] - pitch;
        float yawErr    =   setpoints[3] - gz;  
        
        float rollErrDot     =     -gx;
        float pitchErrDot    =     -gy;
        float yawErrDot      =     -(yawErr - yaw_e.prev)/deltat; //low pass filter, also check actual noise
        
        //calculate integral error
        roll_e.integ    = calculateErrInt(roll_e.integ, rollErr, roll_e.prev, deltat);
        pitch_e.integ   = calculateErrInt(pitch_e.integ, pitchErr, pitch_e.prev, deltat);
        yaw_e.integ     = calculateErrInt(yaw_e.integ, yawErr, yaw_e.prev, deltat);
        
        PID_roll =  roll_e.kp*rollErr   +   roll_e.Kd*rollErrDot       +    roll_e.ki*roll_e.integ;
        PID_pitch = pitch_e.kp*pitchErr +   pitch_e.Kd*pitchErrDot     +    pitch_e.ki*pitch_e.integ;
        PID_yaw =   yaw_e.kp*yawErr     +   yaw_e.Kd*yawErrDot         +    yaw_e.ki*yaw_e.integ;
        
        //convertToMicroseconds(); need to convert PID outputs to esc microseconds
        
        roll_e.prev = rollErr;
        pitch_e.prev = pitchErr;
        yaw_e.prev = yawErr;
        
        int thrust1 = channelVal[2] - PID_pitch - PID_roll + PID_yaw;
        int thrust2 = channelVal[2] + PID_pitch - PID_roll - PID_yaw;
        int thrust3 = channelVal[2] + PID_pitch + PID_roll + PID_yaw;
        int thrust4 = channelVal[2] - PID_pitch + PID_roll - PID_yaw;
        
        if(thrust1 < MIN_THRUST_VAL) thrust1 = MIN_THRUST_VAL;
        if(thrust1 > MAX_THRUST_VAL) thrust1 = MAX_THRUST_VAL;
        if(thrust2 < MIN_THRUST_VAL) thrust2 = MIN_THRUST_VAL;
        if(thrust2 > MAX_THRUST_VAL) thrust2 = MAX_THRUST_VAL;
        if(thrust3 < MIN_THRUST_VAL) thrust3 = MIN_THRUST_VAL;
        if(thrust3 > MAX_THRUST_VAL) thrust3 = MAX_THRUST_VAL;
        if(thrust4 < MIN_THRUST_VAL) thrust4 = MIN_THRUST_VAL;
        if(thrust4 > MAX_THRUST_VAL) thrust4 = MAX_THRUST_VAL;
         
        esc1.pulsewidth_us(thrust1);
        esc2.pulsewidth_us(thrust2);
        esc3.pulsewidth_us(thrust3);
        esc4.pulsewidth_us(thrust4);
        
        if(print == 1) 
                            pc.printf("%d,%d,%d,%d\n", thrust1,thrust2,thrust3, thrust4);
        else if(print==2)   pc.printf("%f,%f,%f\n",    PID_roll, PID_pitch, PID_yaw);
        else if(print==3)   pc.printf("%f,%f,%f\n",    rollErr, pitchErr,yawErr);
        else if(print==4)   pc.printf("%f,%f,%f\n",    rollErrDot, pitchErrDot, yawErrDot);
        else if(print==5)   pc.printf("%f,%f,%f\n",    roll_e.integ, pitch_e.integ, yaw_e.integ);
        
        t1.reset();
    }

float calculateErrInt(float err_int, float err, float errPrev, float deltat){ //calculate integral of error for all three at omce else timer will fuck up
        
        //calculate addition to err_int
        float deltae = (deltat/2)*(errPrev + err);//reset t1
        
        //add to err_int
        return err_int + deltae;
    }

void convertToSetpoints(){
        for(int i = 0; i < 5; i++){
                if(i != 2 && i != 3){ //height setpoint will come from channel 3 and yaw setpoint to 40 dps
                    if(channelVal[i] >= 1497 && channelVal[i] <= 1503){
                         setpoints[i] = 0;
                    }
                    
                    else{
                        setpoints[i] = (channelVal[i] - 1495)/setpointScaler; //to get max setpoint of 20 degrees
                    }
                }
                
                else if(i == 2) setpoints[i] = (channelVal[i] - 1003)/heightSetpointScaler; //to get max height of 5 meters
                
                else if(i == 3) setpoints[i] = (channelVal[i] - 1495)/yawSetpointScaler; // to get max yaw rate setpoint of 40 dps   
        }
    }


//introduce dead band for yaw
#endif

//need to view data for yawErrDot and yaw_e.integ
