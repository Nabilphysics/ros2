
## Microcontroller Code
Go to the arduinoCode folder for the microcontroller code. Right now microcontroller is reading encoder data, converting it to encoder ticks, and sending it to ROS Node and Receiving data from ROS Node and driving the robot accordingly. 

### Incoming Data Format(from ROS)
Incoming Data Format -> Left_Forward_Motor_Direction : PWM : Right_Forward_Motor_Direction : PWM : Left_Aft_Motor_Direction : PWM : Right_Aft_Motor_Direction : PWM : End_Char
e.g: 'KF200F200F200F200G'

### Outgoing Data Format
encoderTicks_Forward_Right_Motor,encoder_Forward_Left_Motor,encoder_Aft_Right_Motor,encoder_Aft_Left_Motor
e.g: -1000,2,350,-145

