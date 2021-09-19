
#include <BasicLinearAlgebra.h>
#include<AccelStepper.h>
#include "perComms.h"


class StateManager{
private:

int stateSize;
float* state; 
float* deltaState;
//BLA::Matrix* XLinear;
//BLA::Matrix* ULinear;
int actuatorCount;
float* posU;
CommsPacket CurrentSend;


float* currintU;


//float dt;


enum constructorEnum {InitialState, InitActuatorValues, X0, intU0};

public:

perComms *CommsManager;
static int CommsSteps;
static float CommsTimeEnd;
//Constructor creates state and delta state 0 vectors of size Size
StateManager(int Size, int ActuatorSize);

//Constructor creates state and delta state vectors depending on input given.
// initializingEnum declares whether initVal is initial state or initial actuator values
StateManager(int Size, int ActuatorSize, float* initVal, constructorEnum initializingEnum);

//returns the state from the forward kinematics given input intU, or the actuator values
float* localFK(float* intU);


void pullPos();       

float* getState();

float* localIK(float* X, float* target, float* currentMotorPos);

float* IKSupplement(float* STIN);

CommsPacket IK(float* target){
    CommsPacket Temp = CurrentSend;

    state = localFK(currintU);
    Temp.U[0] = localIK(state, target, currintU);
    Temp.t[0] = millis()-CommsManager->startMillis;
    float deltaT = CommsTimeEnd/CommsSteps;
    float* tempState = state;
    float* tempintU = currintU;
    for( int i = 1; i < CommsSteps; i++){
        for(int j = 0; j < actuatorCount; j++){
            tempintU[j] += Temp.U[i-1][j]*deltaT;
        }
        Temp.U[i] = localIK(localFK(tempintU), target, tempintU);
        Temp.t[i] = millis()-CommsManager->startMillis+i*deltaT;
    }
    CurrentSend = Temp;
    return CurrentSend;
}

};


float* StateManager::getState(){
    return state;
}

void StateManager::pullPos(){
    float* deltaintU = CommsManager->pullPos();
    for(int i = 0; i<stateSize; i++){
        currintU[i] += deltaintU[i];
        }
}
StateManager:: StateManager(int Size, int ActuatorSize, float* initVal, constructorEnum initializingEnum){
    
    actuatorCount = ActuatorSize;
   // A = Identity<Size,Size>;
   // B = Identity<Size,actuatorCount>;
    stateSize = Size;
    if((initializingEnum == X0)||(initializingEnum == InitialState)) state = initVal;
    if ((initializingEnum == intU0)||(initializingEnum == InitActuatorValues)) state = localFK(initVal);
    deltaState = new float[stateSize];

    currintU = new float[actuatorCount];
    CommsManager = new perComms(actuatorCount, &CurrentSend);
    for(int i = 0; i<actuatorCount; i++){
        currintU[i] = 0;
    }

    CurrentSend.Size = CommsSteps;
}


StateManager::StateManager(int Size, int ActuatorSize){
    actuatorCount = ActuatorSize;
    state = new float[Size];
    deltaState = new float[Size];
    
    CommsManager = new perComms(actuatorCount, &CurrentSend);

    currintU = new float[actuatorCount];
    for(int i = 0; i < Size; i++){
        deltaState[i] = 0;
        state[i] = 0;
    }

    for(int i = 0; i<actuatorCount; i++){
        currintU[i] = 0;
    }
    CurrentSend.Size = CommsSteps;
}