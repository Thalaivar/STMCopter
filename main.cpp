
#include "ARMING.h"                                                             //Header for SafeGaurds
#include "ATTITUDE.h"                                                           //Header for Calculating Attitude             
#include "PID.h"                                                                //Headere for computing and implementing Corrections
#include "INITIALISE.h"                                                         //Definition for Gains and setup functions
#include "MPU9250.h"                                                            //IMU Header file
#include "PERIPHERALS.h"                                                        //Declaration for ESC output and Initialisation        
#include "PPM.h"                                                                //Radio Signal setup and reading 
#include "TELEMETRY.h"

int main(){
        initialisePeripherals();
        //pc.printf("HI I AM ALIVE!");
        //wait(3);
        initialiseIMU();
        initialiseErrInt();
        initialiseGains();
        initialiseMagBias();
        
        armCheck=false;                                                             
        if(!quadTestMode(true)) armQuad();                                                    //Set this true only when testing in room else arm Quad,pull down channel 2 and 3 for 2 seconds            
        
        initialiseTimers();                                                     //setup for timer
        
        while(armCheck){                                                        //functional loop of quad when running
            if(imu.readByte(MPU9250_ADDRESS, INT_STATUS) & 0x01) getAngles(1);  //Getting attitude Feedback               
            controlQuad(0);                                                     //Implementing control, input of function is serial monitor control  
            //disarmCheck();                                                      //disarm guard to powerdown the aerial vehicle as precaution
            sendData();
        }
}
