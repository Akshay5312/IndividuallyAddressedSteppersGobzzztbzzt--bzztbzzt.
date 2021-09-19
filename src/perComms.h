#include <Wire.h>
#include "DImanager.h"

class perComms{
private: 
    int actuatorSize;
    CommsPacket* CurrentSend; 
    int StartIndex = 1; 
    float* pos;   
public:
DImanager *localStepperManager;
int startMillis;
perComms();
perComms(int ActuatorSize, CommsPacket *CurrSen){
    Wire.begin();
    actuatorSize = ActuatorSize;
    CurrentSend = CurrSen;
    pos = new float[actuatorSize];
    startMillis = millis();
    localStepperManager = new DImanager(DO, 0, 0);
}
float* pullPos();
void sendU();
};



float* perComms::pullPos(){
    float* sum = new float[actuatorSize];
    for(int j = 0; j < actuatorSize; j++){
        sum[j] = 0;
    }
    int N = 0;
    for(int i = 0; CurrentSend->t[i]<=(millis()-startMillis); i++){
        for(int j = 0; j < actuatorSize; j++){
            sum[j] += CurrentSend->U[i][j];
        }
        N++;
    } 
    float* deltaPos = sum;
    for(int j = 0; j < actuatorSize; j++){
        deltaPos[j]=(sum[j]/N)*((millis()-startMillis)-CurrentSend->t[0]);        
    }
    return deltaPos;
}


/* This is the code that would run, but alas, at the moment I do not have enough devices to do this, so I am implementing a similar, yet different function
void perComms::sendU(){
    for(int i = StartIndex; i < actuatorSize+StartIndex; i++){
        Wire.beginTransmission(i);
        Wire.write(&CurrentSend);
        Wire.endTransmission(i);
    }
}

*/
void perComms::sendU(){
     for(int i = StartIndex; i < actuatorSize+StartIndex-1; i++){
        Wire.beginTransmission(i);
        I2C_writeAnything(*CurrentSend);
        Wire.endTransmission(i);
    }
    localStepperManager->DOUpdate(*CurrentSend);
    

}