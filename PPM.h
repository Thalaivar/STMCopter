#pragma once
#ifndef PPM_H
#define PPM_H

#include "mbed.h"
#include "PERIPHERALS.h"


uint16_t channelVal[6];
void measureChannel();          //gets all channel values
bool printChannel;              //set true to print channel values
void printChannelVals();              
uint8_t currentChannel;     //to keep track of which channel is being read

void measureChannel(){
    int elapsedTime = t.read_us();
              //if any pulse shorter than 1000us, discard as garbage
       if(elapsedTime < 900){
            t.reset();
            return;
        }
      
      else{ 
            //if start/stop sequence, then set currentChannel = 1 and begin to read
            if(elapsedTime > 2100) {
                    currentChannel = 1;
                    t.reset();
                    return;
                }
            //read channel value
            else if(elapsedTime >= 1000 && elapsedTime <= 2000){
                    if(currentChannel == 1){
                            channelVal[0] = elapsedTime;
                            currentChannel = 2;
                            t.reset();
                            return;
                        }
                    
                    else if(currentChannel == 2){
                            channelVal[1] = elapsedTime;
                            currentChannel = 3;
                            t.reset();
                            return;
                        }
                    
                    else if(currentChannel == 3){
                            channelVal[2] = elapsedTime;
                            currentChannel = 4;
                            t.reset();
                            return;
                        }
                    
                    else if(currentChannel == 4){
                            channelVal[3] = elapsedTime;
                            currentChannel = 5;
                            t.reset();
                            return;
                        }
                    
                    else if(currentChannel == 5){
                            channelVal[4] = elapsedTime;
                            currentChannel = 6;
                            t.reset();
                            return;
                        }
                    
                    else if(currentChannel == 6){
                            channelVal[5] = elapsedTime;
                            currentChannel = 0;
                            t.reset();
                            return;
                        }
                }
          } 
    }

void printChannelVals(){
        if(printChannel) pc.printf("1: %d  ;  2: %d  ;  3: %d  ;  4: %d\n", channelVal[0], channelVal[1], channelVal[2], channelVal[3]);
    }
#endif
