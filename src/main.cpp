#include <Arduino.h>
#include <EditMe.h>

StateManager Base(2,2);
void setup() {
  // put your setup code here, to run once:

}


void loop() {
  // put your main code here, to run repeatedly:
    float Target[2] = {0,1};
    Base.IK(Target);
    

}