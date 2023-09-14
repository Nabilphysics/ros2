class PID():
    def __init__(self, Kp, Ki, Kd, highest_pwm, lowest_pwm):
        self.wheel_error = 0.0 
        self.integral = 0.0
        self.derivative = 0.0
        self.previous_error = 0.0
        self.applied_wheel_pwm = 0
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.highest_pwm = highest_pwm
        self.lowest_pwm = lowest_pwm
    
    def getPidOutput(self, time_elapsed, target_velocity, current_velocity):
            
        self.wheel_error = abs(target_velocity) - abs(current_velocity)
        
        self.integral = self.integral + (self.wheel_error * self.Ki* time_elapsed)
        
        if(self.integral > self.highest_pwm):
            self.integral = self.highest_pwm
        elif(self.integral < self.lowest_pwm):
            self.integral = self.lowest_pwm
        
        self.derivative = (self.wheel_error - self.previous_error)/time_elapsed
        self.applied_wheel_pwm = (self.Kp * self.wheel_error) + (self.integral) + (self.Kd * self.derivative)
        
        if self.applied_wheel_pwm > self.highest_pwm:
            self.applied_wheel_pwm = self.highest_pwm
        
        elif self.applied_wheel_pwm < self.lowest_pwm:
            self.applied_wheel_pwm = self.lowest_pwm
        
        if target_velocity == 0.0:
            self.applied_wheel_pwm = 0
            self.integral = 0.0
        self.previous_error = self.wheel_error 
        return self.applied_wheel_pwm 