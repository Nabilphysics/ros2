/*
Magnetic Encoder Analog Value to Tick Converter
What this Class Does:
This class Takes analog value from Magnetic Encoder(e.g. AS5600) and converts it to tick counter.
Analog output depends on the angle of the magnetic encoder. For 8 bit ADC 360 Degree means analog value 1023.
So value is increasing with angle and after highest analog value(e.g. 1023) it will go to 0. 
This code check and do roation roundup if necessar. Encoder value go to highest to next cycle it can detect it. It checks previous and current value.
Lets say encoder rotated 1000 to 5 analog value clockwise. How we can check this? If current analog value is lower than 30% of encoder highest value and previous value is greater than
70% of encoder highest value then we assume roation is clockwise and we also set necessary parameter to achieve currect tick count.

How to Set Analog Pin : 
EncoderCount encoderForwardRight(A0, '+'); // Give - or + if you want to change the counting direction
How to Get Value:
encoderForwardRight.getValue()

Written By: Syed Razwanul haque(Nabil)
https://www.github.com/Nabilphysics                           
SRH              
*/
#ifndef Encoder_count_h
#define Encoder_count_h

#include "Arduino.h"

class EncoderCount {
  public:
    EncoderCount(char analog_pin, char encoder_direction); 
    int getValue();
    

  private:
    char _analog_pin;
    char _encoder_direction;
    int encoder_highest = 1023;
    int encoder_lowest = 0;
    int previous_encoder = 0.0;
    uint8_t encoder_tick_resolution = 1;
    int current_encoder_analog_value = 0;
    int tick_count = 0;
    int tick_count_resolution = 1;
    int highest_tick = 32767;
    int lowest_tick = -32768;
    bool first_cycle_flag = true;
};
#endif