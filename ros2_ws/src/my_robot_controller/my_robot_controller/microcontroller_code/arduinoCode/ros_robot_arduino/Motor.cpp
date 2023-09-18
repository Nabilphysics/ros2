/*
Constructor -> Motor(char Motor_Driver_Pin_1, char Motor_Driver_Pin_2, PWM Value)
Drive Function -> Drive(char direction, int pwmValue)
              Direction -> F=Forward, R=Reverse, S=Stop
Stop Function -> Stop()
              All Motor Stop  
https://www.github.com/Nabilphysics                           
SRH              
*/

#include "Arduino.h"
#include "Motor.h"

Motor ::Motor(char pinNo1, char pinNo2, uint8_t pwmPin){
  pinMode(pwmPin, OUTPUT);
  pinMode(pinNo1, OUTPUT);
  pinMode(pinNo2, OUTPUT);
  _pinNo1 = pinNo1;
  _pinNo2 = pinNo2;
  _pwmPin = pwmPin;

}

void Motor::Drive(char direction, int pwmValue){

 
  if(direction == 'F'){
    digitalWrite(_pinNo1, HIGH);
    digitalWrite(_pinNo2, LOW);
  }

  if(direction == 'R'){
    digitalWrite(_pinNo1, LOW);
    digitalWrite(_pinNo2, HIGH);
  }
  if(direction == 'S'){
    digitalWrite(_pinNo1, LOW);
    digitalWrite(_pinNo2, LOW);
  }
  analogWrite(_pwmPin, pwmValue);
  
}

void Motor::Stop(){

    digitalWrite(_pinNo1, LOW);
    digitalWrite(_pinNo2, LOW);
    analogWrite(_pwmPin, 0);
  
}

//SRH