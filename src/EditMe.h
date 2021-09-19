
#include "StateManager.h"
#include <math.h>
////////////////////////////////////////////////////////////////////////
//This section is the custom include section. This is the code you edit.

    //The amount of discrete velocity steps sent to the controllers 
    int StateManager::CommsSteps = 500;
    //The time interval that packet covers in milliseconds
    float StateManager:: CommsTimeEnd = 5000;



    float* StateManager:: localFK(float* currentMotorpos){
        float d = 2.853;
        float startTheta = 0.174533;
        float T0 = currentMotorpos[0]*0.0314159+startTheta;
        float T1 = 3.1415- (currentMotorpos[1]*0.0314159+startTheta);

         
        float L = d*cos((T1-T0)/2)*2;
        float Tt = (T1+T0)/2;

        float STATE[2] = {L*cos(Tt), L*sin(Tt)};
        return STATE;
    }

    float* StateManager:: localIK( float* currentState, float* target, float* currentMotorPos){
        //The "target" is target change in State. Set your IK accordingly

        //values in state can be returned by stateSize


        float* tempState = currentState;
        float delta_t_test = 0.001;
        for(int i = 0; i<stateSize; i++){
            tempState[i] = currentState[i]+target[i]*delta_t_test;
        }
        float* NewVals = IKSupplement(tempState);
        for(int i = 0; i<stateSize; i++){
            NewVals[i] = (NewVals[i]-currentMotorPos[i])/delta_t_test;
        }

        return NewVals;
    }

    //The "true" IK, this returns a position value of motors given a state. If you have solved IK of velocity, you don't have to use this. 
    float* StateManager:: IKSupplement(float* STIN){  float d = 2.853;
        float T1mT2 = 2*acos(sqrt(pow(STIN[0],2)+pow(STIN[1],2))/2)/d ;
        float T1pT2;
        if(STIN[0]<0){ 
            T1pT2= 3.1415-atan(STIN[1]/STIN[0]);
        }else { T1pT2= atan(STIN[1]/STIN[0]);}
        T1pT2 = 2*T1pT2;

        float OUT[2] = {(T1pT2+T1mT2)/2 ,(T1pT2-T1mT2)/2};
        return OUT;
    }




///////////////////////////////////////////////////////////////////////