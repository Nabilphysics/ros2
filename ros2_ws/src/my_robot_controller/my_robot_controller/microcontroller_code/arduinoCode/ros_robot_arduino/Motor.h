#ifndef Motor_h
#define Motor_h

#include "Arduino.h"

class Motor {
  public:
    Motor(char pinNo1, char pinNo2, uint8_t pwmPin); // Motor Driver's Two Direction Control pin and Enable Pin to control speed 
    void Drive(char direction, int pwmValue);
    void Stop();

  private:
    char _pinNo1;
    char _pinNo2;
    uint8_t _pwmPin; 
};
#endif

/*
Drive Function -> Drive(direction, pwmValue)
              Direction -> F=Forward, R=Reverse, S=Stop
Stop Function -> Stop()
              All Motor Stop               
SRH              
*/
