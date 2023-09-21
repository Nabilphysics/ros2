/*
4Wheel Differential Drive
Syed Razwanul Haque(Nabil), https://www.github.com/Nabilphysics, https://www.nabilbd.com
************** Motor Driver & Arduino Pin **************
----------------------------------------------------
Motor                 | In1 | In2 | PWM Pin
----------------------------------------------------
Forward Right Motor   | 2   | 4   | 3        
Forward Left Motor    | 7   | 8   | 5 
Aft Right Motor       | 13  | 12  | 6
Aft Left Motor        | A4  | A5  | 10
----------------------------------------------------
----------------------------------------------------
 ************* Magnetic Encoder & Arduino Pin(Analog Read Mode) *************
 ---------------------------------------------------
       Magnetic Encoder No        | Arduino Pin
 ---------------------------------------------------
        Forward Right Mot Encoder |   A0
        Forward Left Mot Encoder  |   A1
        Aft Right Mot Encoder     |   A2  (Not Used Right Now)
        Aft Left Motor Encoder    |   A3  (Not Used Right Now)          
 ---------------------------------------------------
 ************* Arduino Uno Pin Left *************
 0,1,9,11
 ---------------------------------------------------
 ---------- Incoming ROS Node Serial Data Format -----
 Incoming Data Format -> Start_Char:Left_Forward_Motor_Direction : PWM : Right_Forward_Motor_Direction : PWM : Left_Aft_Motor_Direction : PWM : Right_Aft_Motor_Direction : PWM : End_Char
 e.g: 'KF200F200F200F200G'
 Outgoing Data Format -> encoderTicks_Forward_Right_Motor,encoder_Forward_Left_Motor,encoder_Aft_Right_Motor,encoder_Aft_Left_Motor
e.g: -1000,2,350,-145
 e.g.  S000F100  means: Direction=Stop,PWM=000 
 ---------- Algorithm -----------
1- Read All Encoder Analog Data
2- Read Valid Data From ROS Node with Start and End marker 
3- Extract Right and Left Motor Command Data from ROS
4- Convert Magnetic Encoder Analog Data to Encoder ticks and send to ROS Node
5 - Check Safety and Write to motor


 Arduino: Arduino Uno
 Motor Driver: L298N Red
 Encoder : Magnetic Encoder with Analog Output
*/

#include <SoftwareSerial.h>

SoftwareSerial softwareSerial(9, 11); // RX, TX

#include "Motor.h"
#include "Encoder_count.h"
//********* Motor Object - Start**********
Motor motorForwardRight(2, 4, 3);  //Change Pin Position if Direction Reverse - motorForwardRight(In1, In2, PWM pin)
Motor motorForwardLeft(8, 7, 5);
Motor motorAftRight(13, 12, 6);
Motor motorAftLeft(A5, A4, 10);
//********* Motor Object - End**********
//********* Encoder Object - Start**********
EncoderCount encoderForwardRight(A0, '+'); // Give - or + if you want to change the counting direction
EncoderCount encoderForwardLeft(A1, '-'); 
EncoderCount encoderAftRight(A2, '+'); 
EncoderCount encoderAftLeft(A3, '+'); 
//********* Encoder Object - End**********

int leftForwardMotorPWM = 0;
int leftAftMotorPWM = 0;
int rightForwardMotorPWM = 0;
int rightAftMotorPWM = 0;
// We only need only left motor pair direction and right pair motor direction, I am extracting four anyway 
char leftForwardMotorDirection;
char leftAftMotorDirection;
char rightForwardMotorDirection;
char rightAftMotorDirection;

bool safetyStop = true;
int safetyStopTimeout = 1000; //If no data received within this time robot will be stopped
String incomingRosData;
String dataToRos;

int startTime,endTime;

String dataFromRos = "";
int i = 0;
int k = 0;



void setup() {
  Serial.begin(115200);
  softwareSerial.begin(115200);
  softwareSerial.println("Experimental Code ...... ");
}

void loop() {
    // Read Encoder Analog Data and send it to ROS Node for Processing
  
  startTime = millis();
  //motorForwardRight.Drive('F', 110); 
  //current_encoder_value = encoderForwardRight.getValue();
  //encoder_difference = current_encoder_value - previous_encoder_value;

  dataToRos = String(encoderForwardRight.getValue()) + "," + String(encoderForwardLeft.getValue()) + "," + String(encoderAftRight.getValue()) + "," +  String(encoderAftLeft.getValue());

  //Serial.println(dataToRos);
  
    if(Serial.available() > 0){
      
      char inputData = Serial.read();
      String inputString = String(inputData);
    

     if(inputString.startsWith("K")){ // Match The Sensor Header Data and start reading to avoid corrupted data
        i = 0; 
      }

    dataFromRos = dataFromRos + inputString;
    i = i + 1;
    
    if(inputString.startsWith("G")){ //Check if the data has ended
      k = i;
    }
    
    if(i == k){  //If String is received completely 
      //softwareSerial.print("Total Data: ");softwareSerial.println(dataFromRos);
      leftForwardMotorDirection = dataFromRos.substring(1,2).charAt(0);
      leftForwardMotorPWM = dataFromRos.substring(2,5).toInt();
    
      rightForwardMotorDirection = dataFromRos.substring(5,6).charAt(0);
      rightForwardMotorPWM = dataFromRos.substring(6,9).toInt();

      leftAftMotorDirection = dataFromRos.substring(9,10).charAt(0);
      leftAftMotorPWM = dataFromRos.substring(10,13).toInt();

      rightAftMotorDirection = dataFromRos.substring(13,14).charAt(0);
      rightAftMotorPWM = dataFromRos.substring(14,17).toInt();

      //softwareSerial.print("Ld:");softwareSerial.print(leftMotorDirection);softwareSerial.print(" LPWM:");softwareSerial.print(leftMotorAppliedPwm);softwareSerial.print(" Rd:");softwareSerial.print(rightMotorDirection);softwareSerial.print(" RPWM:");softwareSerial.println(rightMotorAppliedPwm);
      dataFromRos = "";
      //softwareSerial.println(endTime);
      if(endTime > 3){ //if more than 3 ms
         Serial.println(dataToRos);
         //softwareSerial.println(dataFromRos);
        // softwareSerial.print("  Endtime: ");softwareSerial.println(endTime);
         //softwareSerial.println(dataToRos);
         endTime = 0;
      }
     
      Serial.flush();
      safetyStop = false;
      //startTime = millis();
    }
  } 
  

  if(safetyStop == false){
    //softwareSerial.print("Safety Stop: "); softwareSerial.println(safetyStop);
    //softwareSerial.println("Driving");
    //softwareSerial.print("Ld:");softwareSerial.print(leftMotorDirection);softwareSerial.print(" LPWM:");softwareSerial.print(leftMotorAppliedPwm);softwareSerial.print(" Rd:");softwareSerial.print(rightMotorDirection);softwareSerial.print(" RPWM:");softwareSerial.println(rightMotorAppliedPwm);
    motorForwardLeft.Drive(leftForwardMotorDirection, leftForwardMotorPWM);
    motorAftLeft.Drive(leftAftMotorDirection, leftAftMotorPWM);
    motorForwardRight.Drive(rightForwardMotorDirection, rightForwardMotorPWM); 
    motorAftRight.Drive(rightAftMotorDirection, rightAftMotorPWM);

  }
  if(safetyStop == true){
    //softwareSerial.print("Safety Stop: "); softwareSerial.println(safetyStop);
    motorForwardLeft.Stop();
    motorForwardRight.Stop(); 
    motorAftLeft.Stop();
    motorAftRight.Stop();
  }

  
  //softwareSerial.print("Endtime: ");softwareSerial.println(endTime);
  //softwareSerial.print("Safety: ");softwareSerial.println(safetyStop);
  
  endTime =  (millis() - startTime)+ endTime;
 
  //softwareSerial.print("Endtime: ");softwareSerial.println(endTime);
  // If this time elapsed robot will stop if no data is available (communication error)
  if(endTime > safetyStopTimeout){
    safetyStop = true;
  }
  
}
