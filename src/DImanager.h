#include <AccelStepper.h> 
#include "CommsPacket.h"
#include <Wire.h>
#include "I2C_Anything.h"


enum STA {DO, DI};

class DImanager{
private:
AccelStepper* Motor;

STA ThisDevice;
int locAddy; 
float StartMillis;
int index;

int prevTime;
CommsPacket RunComm;


public: 
DImanager();
DImanager(STA WhatIs, int Addy, int indexVal){
    ThisDevice = WhatIs;
    locAddy = Addy;
    Wire.begin(Addy);
    StartMillis = millis();
    Motor = new AccelStepper(4,3);
    index = indexVal;
    //prevTime = millis()-StartMillis;
    prevTime = 0;
  
}


void DOUpdate(CommsPacket IN){
    RunComm = IN;
}

void DeviceIn(){
    
    if(ThisDevice == DI){
        if(Wire.available()){
            I2C_readAnything (RunComm);
            prevTime = 0;
        }
    }
    if(nextTime()){
        prevTime++;
        Motor->setSpeed(RunComm.U[index][prevTime]);
    }
    Motor->run();

    

}

bool nextTime(){
    ((millis()-StartMillis)>(RunComm.U[index][prevTime+1]));
}



};