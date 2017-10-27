#include "ARMING.h"
#include "ATTITUDE.h"
#include "PID.h"
#include "INITIALISE.h"
#include "MPU9250.h"
#include "PERIPHERALS.h"
#include "PPM.h"

int main(){
        initialisePeripherals();
        initialiseIMU();
        initialiseErrInt();
        
        quadTestMode(true);
        
        initialiseTimers();
        
        while(armCheck){
                getAngles();
                controlQuad(2);
            }
    }
