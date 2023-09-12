#!/usr/bin/env python
#https://github.com/jfstepha/differential-drive/blob/master/scripts/diff_tf.py#L192
import rclpy.time
import rclpy
import rclpy.duration
from rclpy.clock import Clock, ROSClock
from math import sin, cos, pi

from rclpy.node import Node
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped
from std_msgs.msg import Int16
from geometry_msgs.msg import Twist
import serial
ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

class PID():
    def __init__(self, Kp, Ki, Kd, highest_pwm, lowest_pwm):
        self.wheel_error = 0.0 
        self.integral = 0.0
        self.derivative = 0.0
        self.previous_error = 0.0
        self.applied_wheel_pwm = 0.0
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


class DiffTf(Node):
    def __init__(self):
        super().__init__('diff_tf_pid_v4')
        qos_profile = QoSProfile(depth=10)
        self.nodename = self.get_name()
        self.get_logger().info("-I- %s started" % self.nodename)
        #### parameters #######
        #self.rate = self.get_parameter("~rate",10.0)  # the rate at which to publish the transform
        self.rate = 10.0
        #self.ticks_meter = float(self.get_parameter('ticks_meter', 250))  # The number of wheel encoder ticks per meter of travel
        self.ticks_meter = 27190 #Experiment: 26850 Calculated: 27190
        #self.base_width = float(self.get_parameter('~base_width', 1.3)) # The wheel base width in meters
        self.base_width = 1.3
        #self.base_frame_id = self.get_parameter('~base_frame_id','base_link') # the name of the base frame of the robot
        self.base_frame_id = "base_link"
        #self.odom_frame_id = self.get_parameter('~odom_frame_id', 'odom') # the name of the odometry reference frame
        self.odom_frame_id = "odom"
        #self.encoder_min = self.get_parameter('encoder_min', -32768)
        self.encoder_min = -32768
        #self.encoder_max = self.get_parameter('encoder_max', 32768)
        self.encoder_max = 32767
        #self.encoder_low_wrap = self.get_parameter('wheel_low_wrap', (self.encoder_max - self.encoder_min) * 0.3 + self.encoder_min )
        self.encoder_low_wrap = ((self.encoder_max - self.encoder_min) * 0.3 + self.encoder_min) 
        #self.encoder_high_wrap = self.get_parameter('wheel_high_wrap', (self.encoder_max - self.encoder_min) * 0.7 + self.encoder_min )
        self.encoder_high_wrap = ((self.encoder_max - self.encoder_min) * 0.7 + self.encoder_min )
        self.t_delta = 0.01
        self.t_next =  ((ROSClock().now().to_msg().sec)+((ROSClock().now().to_msg().nanosec)/1e9)) + self.t_delta
        
        # internal data
        self.enc_left = None        # wheel encoder readings
        self.enc_right = None
        self.left = 0               # actual values coming back from robot
        self.right = 0
        self.lmult = 0
        self.rmult = 0
        self.prev_lencoder = 0
        self.prev_rencoder = 0
        self.x = 0.0                  # position in xy plane 
        self.y = 0.0
        self.th = 0.0
        self.dx = 0                 # speeds in x/rotation
        self.dr = 0
        self.current_left_wheel_velocity = 0.0
        self.current_right_wheel_velocity = 0.0
        #Simulated
        self.simulated_velocity = 0
        '''
        # PID Related Left Wheel Variable
        self.left_wheel_error = 0.0 
        self.left_integral = 0.0
        self.left_derivative = 0.0
        self.previous_left_error = 0.0
        self.applied_left_wheel_pwm = 0.0
        self.left_wheel_Kp = 160.0
        self.left_wheel_Ki = 60.0
        self.left_wheel_Kd = 1.5
        self.left_highest_pwm = 255.0
        self.left_lowest_pwm = 20.0
        '''
        self.forward_left_wheel_direction = ''
        '''
        # PID Related Right Wheel Variable
        self.right_wheel_error = 0.0        
        self.right_integral = 0.0        
        self.right_derivative = 0.0        
        self.previous_right_error = 0.0       
        self.applied_right_wheel_pwm = 0.0
        self.right_wheel_Kp = 160.0
        self.right_wheel_Ki = 60.0
        self.right_wheel_Kd = 1.5
        self.right_highest_pwm = 255
        self.right_lowest_pwm = 20
        '''
        self.forward_right_wheel_direction = ''
       

        self.send_data = ''
        self.serial_raw = ''
        # --- Encoder Related Varibale ---
        self.forward_right_motor_tick = 0
        self.forward_left_motor_tick = 0
        # ----- Encoder Related Varibale -- END
        self.then = ((ROSClock().now().to_msg().sec)+((ROSClock().now().to_msg().nanosec)/1e9))
        # subscriptions
        #self.create_subscription(Int16,'lwheel',self.lwheelCallback, 10)
        #self.create_subscription(Int16,'rwheel',self.rwheelCallback, 10)
        self.create_subscription(Twist,'/cmd_vel',self.twistCallback, 10)
    
        self.odomPub = self.create_publisher(Odometry, 'odom', qos_profile)
        self.get_logger().info("publisher creater")
        self.odomBroadcaster = TransformBroadcaster(self, qos= qos_profile)
        
        self.odom_trans = TransformStamped()
        #self.odom_trans.header.frame_id = 'odom'
        self.odom_trans.header.frame_id = self.base_frame_id
        self.odom_trans.child_frame_id = self.odom_frame_id

        self.target_left_wheel_velocity = 0
        self.target_right_wheel_velocity = 0
        self.robot_base = 0.38
        self.commanded_linear_velocity = 0.0
        self.commanded_angular_velocity = 0.0
        
        self.create_timer(0.001, self.serialDataProcess)
        self.create_timer(0.01, self.update)
        self.create_timer(0.05, self.targetWheelVelocity)
        self.create_timer(0.001,self.sendReceiveData)
        self.create_timer(1.0, self.showData)
        
    def showData(self):
        #print('Left-Right Velocity: ', self.current_left_wheel_velocity, self.current_right_wheel_velocity)
        #print('Left-Right RPM: ', ((self.current_left_wheel_velocity*9.55)/0.06), ((self.current_right_wheel_velocity*9.55)/0.06)) 
        print('------')
        print('Right_Error:',self.right_wheel_error,' Right_Apl_PWM: ',self.applied_right_wheel_pwm, ' Target_R_Vel:',self.target_right_wheel_velocity, ' Curr_R_Vel:',self.current_right_wheel_velocity,' Integral_R:',self.right_integral, ' Deriv:', self.right_derivative)   
        print('Left_Error:',self.left_wheel_error,' Left_Apl_PWM: ',self.applied_left_wheel_pwm, ' Target_L_Vel:',self.target_left_wheel_velocity, ' Curr_L_Vel:',self.current_left_wheel_velocity,' Integral_L:',self.left_integral,' Deriv:', self.left_derivative)
        print('######')
    def sendReceiveData(self):
        #self.send_data = 'KF110F110G'
        #self.applied_right_wheel_pwm = 110
        #self.applied_left_wheel_pwm = 110
        
        #Send to Arduino Serial - Data Format
        #StartChar:Left_Forward_Motor_Direction:PWM:Rigit_Forward_Motor_Direction:PWM:EndChar
        self.send_data = 'K'+ self.forward_left_wheel_direction + str(int(self.applied_left_wheel_pwm)).zfill(3) + self.forward_right_wheel_direction + str(int(self.applied_right_wheel_pwm)).zfill(3) + 'G'
        ser.write(bytes(self.send_data, 'utf-8'))
        self.serial_raw = ser.readline()
        #print(self.serial_raw)

    def serialDataProcess(self):
        try:
            serial_decode = self.serial_raw.decode("utf-8","ignore")
            #print(serial_decode) position 0 = right encoder raw value, 1 = left Encoder Raw Value
            serial_split = serial_decode.split(",")
        
            self.forward_right_motor_tick = float(serial_split[0])
            self.forward_left_motor_tick = float(serial_split[1])
        except:
            pass
        self.lwheelCallback(self.forward_left_motor_tick) # Send to Another Function. 
        self.rwheelCallback(self.forward_right_motor_tick) # Send to Another Function. 

    def targetWheelVelocity(self):
        
        self.target_left_wheel_velocity = self.commanded_linear_velocity * 1.0 - ((self.commanded_angular_velocity * self.robot_base)/2)
        self.target_right_wheel_velocity = self.commanded_linear_velocity * 1.0 + ((self.commanded_angular_velocity * self.robot_base)/2)
        
    def update(self):
            now = ((ROSClock().now().to_msg().sec)+((ROSClock().now().to_msg().nanosec)/1e9))
            if now > self.t_next:
                elapsed = now - self.then
                self.then = now
                # calculate odometry
                if self.enc_left == None:
                    d_left = 0
                    d_right = 0
                else:
                    d_left = (self.left - self.enc_left) / self.ticks_meter
                    d_right = (self.right - self.enc_right) / self.ticks_meter
                    '''
                    d_left = (self.forward_left_motor_tick - self.enc_left) / self.ticks_meter
                    d_right = (self.forward_right_motor_tick - self.enc_right) / self.ticks_meter
                    '''
                self.enc_left = self.left
                self.enc_right = self.right
                '''
                self.enc_left = self.forward_left_motor_tick
                self.enc_right = self.forward_right_motor_tick
                '''
               
                # distance traveled is the average of the two wheels 
                d = ( d_left + d_right ) / 2
                # this approximation works (in radians) for small angles
                th = ( d_right - d_left ) / self.base_width
                # calculate velocities
                self.current_left_wheel_velocity = d_left/elapsed
                self.current_right_wheel_velocity = d_right/elapsed
                #print('d_left - d_right: ', d_left, d_right)
                
                
                self.dx = d / elapsed
                self.dr = th / elapsed

                #Left Wheel PID : TODO: Make a Separate Class for PID
                
                self.left_wheel_error = abs(self.target_left_wheel_velocity) - abs(self.current_left_wheel_velocity)
                #self.left_wheel_error = abs(self.target_current_left_wheel_velocity) - abs(self.simulated_velocity)
                self.left_integral = self.left_integral + (self.left_wheel_error * self.left_wheel_Ki* elapsed)
                
                if(self.left_integral > self.left_highest_pwm):
                    self.left_integral = self.left_highest_pwm
                elif(self.left_integral < self.left_lowest_pwm):
                    self.left_integral = self.left_lowest_pwm
               
                self.left_derivative = (self.left_wheel_error - self.previous_left_error)/elapsed
                self.applied_left_wheel_pwm = (self.left_wheel_Kp * self.left_wheel_error) + (self.left_integral) + (self.left_wheel_Kd * self.left_derivative)
               
                if self.applied_left_wheel_pwm > self.left_highest_pwm:
                    self.applied_left_wheel_pwm = self.left_highest_pwm
                
                elif self.applied_left_wheel_pwm < self.left_lowest_pwm:
                    self.applied_left_wheel_pwm = self.left_lowest_pwm
               
                if self.target_left_wheel_velocity == 0.0:
                    self.applied_left_wheel_pwm = 0
                    self.left_integral = 0.0
                self.previous_left_error = self.left_wheel_error  
                
                if(self.target_left_wheel_velocity > 0.0):
                    self.forward_left_wheel_direction = 'F'
                if(self.target_left_wheel_velocity < 0.0):
                    self.forward_left_wheel_direction = 'R'
                if(self.target_left_wheel_velocity == 0.0):
                    self.forward_left_wheel_direction = 'S'   
                #print('Left_Error:',self.left_wheel_error,' Left_Apl_PWM: ',self.applied_left_wheel_pwm, ' Target_L_Vel:',self.target_left_wheel_velocity, ' Curr_L_Vel:',self.current_left_wheel_velocity,' Integral_L:',self.left_integral)

                # Right Wheel PID
                self.right_wheel_error = abs(self.target_right_wheel_velocity) - abs(self.current_right_wheel_velocity)
                self.right_integral = self.right_integral + (self.right_wheel_error * self.right_wheel_Ki * elapsed)
                if(self.right_integral > self.right_highest_pwm):
                    self.right_integral = self.right_highest_pwm
                elif(self.right_integral < self.right_lowest_pwm):
                    self.right_integral = self.right_lowest_pwm

                self.right_derivative = (self.right_wheel_error - self.previous_right_error)/elapsed
                
                self.applied_right_wheel_pwm = (self.right_wheel_Kp * self.right_wheel_error) + (self.right_integral) + (self.right_wheel_Kd * self.right_derivative)
                if self.applied_right_wheel_pwm > self.right_highest_pwm:
                    self.applied_right_wheel_pwm = self.right_highest_pwm
                if self.applied_right_wheel_pwm < self.right_lowest_pwm:
                    self.applied_right_wheel_pwm = self.right_lowest_pwm
                if self.target_right_wheel_velocity == 0.0:
                    self.applied_right_wheel_pwm = 0
                    self.right_integral = 0.0
                self.previous_right_error = self.right_wheel_error
                if(self.target_right_wheel_velocity > 0.0):
                    self.forward_right_wheel_direction = 'F'
                    #self.applied_right_wheel_pwm = 255
                if(self.target_right_wheel_velocity < 0.0):
                    self.forward_right_wheel_direction = 'R'
                    #self.applied_right_wheel_pwm = 255
                if(self.target_right_wheel_velocity == 0.0):
                    self.forward_right_wheel_direction = 'S'  
                #print('Right_Error:',self.right_wheel_error,' Right_Apl_PWM: ',self.applied_right_wheel_pwm, ' Target_R_Vel:',self.target_right_wheel_velocity, ' Curr_R_Vel:',self.current_right_wheel_velocity,' Integral_R:',self.right_integral)
                
                #Send to Arduino Serial - Data Format
                #Left_Forward_Motor_Direction:PWM:Rigit_Forward_Motor_Direction:PWM
                #self.send_data = 'K'+'S' + str(int(self.applied_left_wheel_pwm)).zfill(3) + 'F' + str(int(self.applied_right_wheel_pwm)).zfill(3) + 'G'
                #send_data = forward_left_wheel_direction + str(int(self.applied_left_wheel_pwm)).zfill(3) + forward_right_wheel_direction + str(int(self.applied_right_wheel_pwm)).zfill(3) + '\n'

                #print(send_data)
                #ser.write(bytes(send_data, 'utf-8'))
                #ser.write(send_data.encode())
               
                
                if (d != 0):
                    # calculate distance traveled in x and y
                    x = cos( th ) * d
                    y = -sin( th ) * d
                    # calculate the final position of the robot
                    # Equation for converting a point in local reference frame to global reference frame
                    #VxG = (X_local * cos(γ)) – (Y_local * sin(γ)) 
                    #VyG = (X_local * sin(γ)) + (Y_local * cos(γ))
                    self.x = self.x + ( cos( self.th ) * x - sin( self.th ) * y )
                    self.y = self.y + ( sin( self.th ) * x + cos( self.th ) * y )
                if( th != 0):
                    self.th = self.th + th
                

                #publish odom transform information
                self.odom_trans.header.stamp = self.get_clock().now().to_msg()
                self.odom_trans.transform.translation.x = self.x
                self.odom_trans.transform.translation.y = self.y
                self.odom_trans.transform.translation.z = 0.0
                self.odom_trans.transform.rotation = euler_to_quaternion(0.0, 0.0, self.th)

                # send the joint state and transform
                self.odomBroadcaster.sendTransform(self.odom_trans)
              
                # publish the odom information
                quaternion = Quaternion()
                quaternion.x = 0.0
                quaternion.y = 0.0
                quaternion.z = sin(self.th/2)
                quaternion.w = cos(self.th/2)
                '''
                self.odomBroadcaster.sendTransform(
                    (self.x, self.y, 0),
                    (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
                    self.get_clock().now().to_msg(),
                    self.base_frame_id,
                    self.odom_frame_id
                    )
                '''
                odom = Odometry()
                odom.header.stamp = self.get_clock().now().to_msg()
                odom.header.frame_id = self.odom_frame_id
                odom.pose.pose.position.x = self.x
                odom.pose.pose.position.y = self.y
                odom.pose.pose.position.z = 0.0
                odom.pose.pose.orientation = quaternion
                odom.child_frame_id = self.base_frame_id
                odom.twist.twist.linear.x = self.dx
        
                odom.twist.twist.linear.y = 0.0
                odom.twist.twist.angular.z = self.dr
                
                self.odomPub.publish(odom)
                    
             
    def lwheelCallback(self, msg):
        enc = msg
        #print("ENC lwheelCallback",enc)
        if (enc < self.encoder_low_wrap and self.prev_lencoder > self.encoder_high_wrap):
            self.lmult = self.lmult + 1
            #print("self.mult.Lwheel+ = ", self.lmult ) 
            
        if (enc > self.encoder_high_wrap and self.prev_lencoder < self.encoder_low_wrap):
            self.lmult = self.lmult - 1
            #print("self.mult.Lwheel- = ", self.lmult ) 
            
        self.left = 1.0 * (enc + self.lmult * (self.encoder_max - self.encoder_min)) 
        #print("ENC lwheelCallback : self.left= ", self.left) 
        self.prev_lencoder = enc
        
    def rwheelCallback(self, msg):

        enc = msg
        #print("ENC RwheelCallback",enc)
        if(enc < self.encoder_low_wrap and self.prev_rencoder > self.encoder_high_wrap):
            self.rmult = self.rmult + 1
        
        if(enc > self.encoder_high_wrap and self.prev_rencoder < self.encoder_low_wrap):
            self.rmult = self.rmult - 1
            
        self.right = 1.0 * (enc + self.rmult * (self.encoder_max - self.encoder_min))
        #print("ENC RwheelCallback : self.right= ", self.right) 
        #print('Left-Right Enc Tick: ',self.left,' -- ',self.right)
        self.prev_rencoder = enc

    def twistCallback(self, msg = Twist):
        self.commanded_linear_velocity = msg.linear.x
        self.commanded_angular_velocity = msg.angular.z
        self.simulated_velocity = msg.linear.y
        #print('Com_linear_velocity: ', self.commanded_linear_velocity )
        #print('Com_angular_velocity: ',self.commanded_angular_velocity)

def euler_to_quaternion(roll, pitch, yaw):
    qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
    qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
    qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
    qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)

def main(args = None):
    rclpy.init(args = args)
    node = DiffTf()
    rclpy.spin(node)
    rclpy.shutdown(node)


if __name__ == '__main__':
    main()
