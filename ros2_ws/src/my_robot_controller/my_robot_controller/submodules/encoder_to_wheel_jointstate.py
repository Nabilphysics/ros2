'''
This class takes wheel encoder ticks and convert it to wheel joint state value.

Written by Syed Razwanul Haque (Nabil)
https://www.github.com/Nabilphysics
'''
class WheelState():
    def __init__(self,radian_per_rotate,enc_tick_rotate, float_round):
        self.wheel_joint_state_value = 0.0
        self.full_roation_angle = radian_per_rotate # Angle in Radian for full rotation, 2*pi
        self.float_round = float_round #Digit after float point
        self.encoder_ticks_per_rotaion = enc_tick_rotate

    def getState(self, current_encoder,previous_encoder):
        self.wheel_joint_state_value = self.wheel_joint_state_value + (((current_encoder - previous_encoder) /self.encoder_ticks_per_rotaion ))* self.full_roation_angle #2*pi
        if(self.wheel_joint_state_value > 6.2832):
            self.wheel_joint_state_value = 0.0
        if(self.wheel_joint_state_value < -6.2832):
            self.wheel_joint_state_value= 0.0
        #print(self.wheel_joint_state_value)
        return round(self.wheel_joint_state_value, self.float_round)