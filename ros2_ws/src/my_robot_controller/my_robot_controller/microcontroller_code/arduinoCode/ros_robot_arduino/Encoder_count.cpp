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

#include "Arduino.h"
#include "Encoder_count.h"

EncoderCount ::EncoderCount(char analog_pin, char encoder_direction){
  _analog_pin = analog_pin;
  _encoder_direction = encoder_direction;
}

int EncoderCount::getValue(){
  current_encoder_analog_value = analogRead(_analog_pin);
  
  //Rotatin Check if Negative Direction (Anti Clockwise)
    if((current_encoder_analog_value > encoder_highest * 0.7 && current_encoder_analog_value <= encoder_highest) && (previous_encoder >= encoder_lowest && previous_encoder < encoder_highest * 0.3)){
      previous_encoder = previous_encoder + encoder_highest;
     
    }
   
    //Rotation Positive Direction (Clockwise)  
    else if((previous_encoder > encoder_highest * 0.7 && previous_encoder <= encoder_highest) && (current_encoder_analog_value >= encoder_lowest && current_encoder_analog_value < encoder_highest * 0.3)){
      previous_encoder = previous_encoder - encoder_highest;
    }
      
    if((current_encoder_analog_value >= previous_encoder + encoder_tick_resolution)){
      if(_encoder_direction == '+'){
        tick_count = tick_count + (abs(current_encoder_analog_value - previous_encoder)/encoder_tick_resolution);
      }
      else if(_encoder_direction == '-'){
        tick_count = tick_count - (abs(current_encoder_analog_value - previous_encoder)/encoder_tick_resolution);
      }
      
       
        // We want Encoder Tick count 0 at Robot Startup. So it will check startup and will reset tick counter
        if(first_cycle_flag == true){
          tick_count = 0;
          first_cycle_flag = false;
        }
        previous_encoder = current_encoder_analog_value; 
    }
    
    else if(current_encoder_analog_value <= previous_encoder- encoder_tick_resolution){
      if(_encoder_direction == '+'){
        tick_count = tick_count - (abs(current_encoder_analog_value - previous_encoder)/encoder_tick_resolution);
      }
      else if(_encoder_direction == '-'){
        tick_count = tick_count + (abs(current_encoder_analog_value - previous_encoder)/encoder_tick_resolution);
      }
      
       // We want Encoder Tick count 0 at Robot Startup. So it will check startup and will reset tick counter
        if(first_cycle_flag == true){
          tick_count = 0;
          first_cycle_flag = false;
        }
         previous_encoder = current_encoder_analog_value ;
    }
      
    if(tick_count > highest_tick){
      tick_count = lowest_tick;
    }
          
    if(tick_count < lowest_tick){
      tick_count = highest_tick ;
    }

    return tick_count;
 
  
}


//SRH