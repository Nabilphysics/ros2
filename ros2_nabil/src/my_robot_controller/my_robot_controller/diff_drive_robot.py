#!/usr/bin/env python
'''
https://www.github.com/Nabilphysics
Syed Razwanul Haque(Nabil)
.
Heavily Inspired from below repo. Some of the code from this repo used as it is.
https://github.com/jfstepha/differential-drive/blob/master/scripts/diff_tf.py#L192
'''
import rclpy.time
import rclpy
import rclpy.duration
from rclpy.clock import Clock, ROSClock
from math import sin, cos, pi
from my_robot_controller.submodules.pid import PID 
from my_robot_controller.submodules.encoder_to_wheel_jointstate import WheelState
from my_robot_controller.submodules.encoder_wrap import EncoderWrap

from rclpy.node import Node
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped
from std_msgs.msg import Int16
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import serial

ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)

class DiffTf(Node):
    def __init__(self):
        super().__init__('Differential_Drive_Robot_Started')
        qos_profile = QoSProfile(depth=10)
        self.nodename = self.get_name()
        self.get_logger().info("-I- %s started" % self.nodename)
        self.ticks_meter = 27190 #Experiment: 26850 Calculated: 27190
        self.wheel_one_rev_ticks = 10230  # Total Encoder count per one full roation of robot wheel
        self.base_width = 0.38 # in Meter
        self.base_frame_id = "base_footprint" # Odometry Reference Frame
        self.odom_frame_id = "odom"
        self.encoder_min = -32768
        self.encoder_max = 32767
        self.encoder_low_wrap = ((self.encoder_max - self.encoder_min) * 0.3 + self.encoder_min) 
        self.encoder_high_wrap = ((self.encoder_max - self.encoder_min) * 0.7 + self.encoder_min )
        
        # Encoder Object
        self.left_forward_enc = EncoderWrap(encoder_min= -32768, encoder_max= 32767)
        self.right_forward_enc = EncoderWrap(encoder_min= -32768, encoder_max= 32767)
        self.left_aft_enc = EncoderWrap(encoder_min= -32768, encoder_max= 32767)
        self.right_aft_enc = EncoderWrap(encoder_min= -32768, encoder_max= 32767)
        
        # previous encoder value
        self.prv_enc_left_forward = None        # wheel encoder readings
        self.prv_enc_left_aft = None
        self.prv_enc_right_forward = None
        self.prv_enc_right_aft = None
        # Current Encoder value
        self.curr_enc_left_forward = 0               
        self.curr_enc_left_aft = 0
        self.curr_enc_right_forward = 0
        self.curr_enc_right_aft = 0
        
        self.x = 0.0                  # position in xy plane 
        self.y = 0.0
        self.th = 0.0
        self.dx = 0                 # speeds in x/rotation
        self.dr = 0

        self.current_left_forward_velocity = 0.0
        self.current_left_aft_velocity = 0.0
        self.current_right_forward_velocity = 0.0
        self.current_right_aft_velocity = 0.0
        
        #Direction Variable. Left Two motors will same and Right Two Motors will be same
        self.left_wheel_direction = ''
        self.right_wheel_direction = ''
        # Applied PWM to Motors. Four Separate value will be computed using PID class
        self.applied_left_forward_pwm = 0.0
        self.applied_right_forward_pwm = 0.0
        self.applied_left_aft_pwm = 0
        self.applied_right_aft_Pwm = 0       

        # PID Object for four Motors, but for direction and target velocity left two motors and right two motors will be same and thus grouped.
        self.left_forward_pid = PID(Kp=120.0, Ki=110.0, Kd= 1.0 , highest_pwm=255, lowest_pwm=90) #KF080F055F060F085G
        self.right_forward_pid = PID(Kp=120.0, Ki=110.0, Kd= 1.0 , highest_pwm=255, lowest_pwm=60)
        self.left_aft_pid = PID(Kp=120.0, Ki=110.0, Kd= 1.0 , highest_pwm=255, lowest_pwm=60)
        self.right_aft_pid = PID(Kp=120.0, Ki=110.0, Kd= 1.0 , highest_pwm=255, lowest_pwm=90)
        
        #Wheel Joint State Calculation Object
        self.left_forward_wheel = WheelState(radian_per_rotate= 6.2832, enc_tick_rotate= self.wheel_one_rev_ticks, float_round= 2)
        self.left_aft_wheel = WheelState(radian_per_rotate= 6.2832, enc_tick_rotate= self.wheel_one_rev_ticks, float_round= 2)
        self.right_forward_wheel = WheelState(radian_per_rotate= 6.2832, enc_tick_rotate= self.wheel_one_rev_ticks, float_round= 2)
        self.right_aft_wheel = WheelState(radian_per_rotate= 6.2832, enc_tick_rotate= self.wheel_one_rev_ticks, float_round= 2)
        
        self.send_data = ''
        self.serial_raw = ''
        # --- Encoder Related Varibale ---
        self.left_forward_motor_tick = 0
        self.left_aft_motor_tick = 0
        self.right_forward_motor_tick = 0
        self.right_aft_motor_tick = 0
        # ----- Encoder Related Varibale -- END
        self.left_forward_wheel_state = 0.0
        self.left_aft_wheel_state = 0.0
        self.right_forward_wheel_state = 0.0
        self.right_aft_wheel_state = 0.0

        self.previous_time = ((ROSClock().now().to_msg().sec)+((ROSClock().now().to_msg().nanosec)/1e9))
        # subscriptions
        self.create_subscription(Twist,'/cmd_vel',self.twistCallback, 10)
    
        self.odomPub = self.create_publisher(Odometry, 'odom', qos_profile)
        self.get_logger().info("publisher created")
        self.odomBroadcaster = TransformBroadcaster(self, qos= qos_profile)
        
        
        self.odom_trans = TransformStamped()
        self.odom_trans.header.frame_id = self.odom_frame_id
        self.odom_trans.child_frame_id = self.base_frame_id
        
        self.target_left_wheel_velocity = 0.0
        self.target_right_wheel_velocity = 0.0
        
        self.commanded_linear_velocity = 0.0
        self.commanded_angular_velocity = 0.0
        
        self.create_timer(0.001, self.serialDataProcess)
        self.create_timer(0.01, self.update)
        self.create_timer(0.05, self.targetWheelVelocity)
        self.create_timer(0.001,self.sendReceiveData)
        self.create_timer(1.0, self.showData)
        
        self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
        self.motorPwmPub = self.create_publisher(Int16, 'motor_pwm', qos_profile)
        self.motorTargetVel = self.create_publisher(Float32, 'TargetVel', qos_profile)
        self.motorCurrentVel = self.create_publisher(Float32, 'CurrentVel', qos_profile)
  
        self.joint_state = JointState()
    
    def showData(self):
        #print('Left-Right Velocity: ', self.current_left_wheel_velocity, self.current_right_wheel_velocity)
        #print('Left-Right RPM: ', ((self.current_left_wheel_velocity*9.55)/0.06), ((self.current_right_wheel_velocity*9.55)/0.06)) 
        print('------')
        print('Right_Error:',self.right_forward_pid.wheel_error,' Right_Apl_PWM: ',self.applied_right_forward_pwm, ' Target_R_Vel:',self.target_right_wheel_velocity, ' Curr_R_Vel:',self.current_right_forward_velocity,' Integral_R:',self.right_forward_pid.integral, ' Deriv:', self.right_forward_pid.derivative)   
        print('Left_Error:',self.left_forward_pid.wheel_error,' Left_Apl_PWM: ',self.applied_left_forward_pwm, ' Target_L_Vel:',self.target_left_wheel_velocity, ' Curr_L_Vel:',self.current_left_forward_velocity,' Integral_L:',self.left_forward_pid.integral,' Deriv:', self.left_forward_pid.derivative)
        print('######')
   
    def sendReceiveData(self):
        # Send to Arduino Serial - Data Format
        # Start_Char : Left_Forward_Motor_Direction : PWM : Right_Forward_Motor_Direction : PWM : Left_Aft_Motor_Direction : PWM : Right_Aft_Motor_Direction : PWM : End_Char
        self.send_data = 'K'+ self.left_wheel_direction + str(int(self.applied_left_forward_pwm)).zfill(3) + self.right_wheel_direction + str(int(self.applied_right_forward_pwm)).zfill(3) + \
        self.left_wheel_direction + str(int(self.applied_left_aft_pwm)).zfill(3) + self.right_wheel_direction + str(int(self.applied_right_aft_Pwm)).zfill(3) + 'G'
        #self.send_data = 'K'+ 'F' + str(int(self.applied_left_forward_pwm)).zfill(3) + 'F' + str(int(self.applied_right_forward_pwm)).zfill(3) + 'G'
        #self.send_data = 'KF200F200F200F200G'
        ser.write(bytes(self.send_data, 'utf-8'))
        self.serial_raw = ser.readline()

    def serialDataProcess(self):
        try:
            serial_decode = self.serial_raw.decode("utf-8","ignore")
            #print(serial_decode) position 0 = right encoder raw value, 1 = left Encoder Raw Value
            serial_split = serial_decode.split(",")
           
            self.right_forward_motor_tick = int(serial_split[0])
            self.left_forward_motor_tick = int(serial_split[1])
            self.right_aft_motor_tick = int(serial_split[2])
            self.left_aft_motor_tick = int(serial_split[3])
        except:
            pass
        
        self.curr_enc_left_forward = self.left_forward_enc.getEncTick(self.left_forward_motor_tick)
        self.curr_enc_right_forward = self.right_forward_enc.getEncTick(self.right_forward_motor_tick)
        self.curr_enc_left_aft = self.left_aft_enc.getEncTick(self.left_aft_motor_tick)
        self.curr_enc_right_aft = self.right_aft_enc.getEncTick(self.right_aft_motor_tick)
       
    def targetWheelVelocity(self):
        self.target_left_wheel_velocity = self.commanded_linear_velocity * 1.0 - ((self.commanded_angular_velocity * self.base_width)/2)
        self.target_right_wheel_velocity = self.commanded_linear_velocity * 1.0 + ((self.commanded_angular_velocity * self.base_width)/2)
        
    def update(self):
                now = ((ROSClock().now().to_msg().sec)+((ROSClock().now().to_msg().nanosec)/1e9))
                elapsed = now - self.previous_time
                self.previous_time = now
                # Calculate Odometry
                if (self.prv_enc_left_forward or self.prv_enc_right_forward) == None:
                    d_left_forward = 0
                    d_left_aft = 0
                    d_right_forward = 0
                    d_right_aft = 0
            
                else:
                    d_left_forward = (self.curr_enc_left_forward - self.prv_enc_left_forward) / self.ticks_meter
                    d_left_aft = (self.curr_enc_left_aft - self.prv_enc_left_aft) / self.ticks_meter
                    d_right_forward = (self.curr_enc_right_forward - self.prv_enc_right_forward) / self.ticks_meter
                    d_right_aft = (self.curr_enc_right_aft - self.prv_enc_right_aft)/self.ticks_meter
                    
                    self.right_forward_wheel_state = self.right_forward_wheel.getState(self.curr_enc_right_forward, self.prv_enc_right_forward)
                    self.left_aft_wheel_state = self.left_aft_wheel.getState(self.curr_enc_left_aft, self.prv_enc_left_aft)
                    self.left_forward_wheel_state = self.left_forward_wheel.getState(self.curr_enc_left_forward, self.prv_enc_left_forward)
                    self.right_aft_wheel_state = self.right_aft_wheel.getState(self.curr_enc_right_aft, self.prv_enc_right_aft)
                    

                self.prv_enc_left_forward = self.curr_enc_left_forward
                self.prv_enc_left_aft = self.curr_enc_left_aft
                self.prv_enc_right_forward = self.curr_enc_right_forward
                self.prv_enc_right_aft = self.curr_enc_right_aft
             
                d_left = (d_left_forward + d_left_aft)/2
                d_right = (d_right_forward + d_right_aft)/2

                # distance traveled is the average of the two wheels 
                d = ( d_left + d_right ) / 2
                
                # this approximation works (in radians) for small angles
                th = ( d_right - d_left ) / self.base_width
                # calculate velocities
                self.current_left_forward_velocity = d_left_forward/elapsed
                self.current_left_aft_velocity = d_left_aft/elapsed
                self.current_right_forward_velocity = d_right_forward/elapsed
                self.current_right_aft_velocity = d_right_aft/elapsed
                
                self.dx = d / elapsed
                self.dr = th / elapsed

                self.applied_left_forward_pwm = self.left_forward_pid.getPidOutput(time_elapsed=elapsed, target_velocity=self.target_left_wheel_velocity, current_velocity=self.current_left_forward_velocity)
                self.applied_left_aft_pwm = self.left_aft_pid.getPidOutput(time_elapsed=elapsed, target_velocity=self.target_left_wheel_velocity, current_velocity=self.current_left_aft_velocity)

                self.applied_right_forward_pwm = self.right_forward_pid.getPidOutput(time_elapsed=elapsed, target_velocity=self.target_right_wheel_velocity, current_velocity=self.current_right_forward_velocity)
                self.applied_right_aft_Pwm = self.right_aft_pid.getPidOutput(time_elapsed=elapsed, target_velocity=self.target_right_wheel_velocity, current_velocity=self.current_right_aft_velocity)
                
                # Publishing Data for Viewing in Floxglove Studio for Debugging
                motor_pwm_msg = Int16()
                motor_pwm_msg.data = int(self.applied_right_forward_pwm)
                self.motorPwmPub.publish(motor_pwm_msg)

                motor_target_velocity = Float32()
                motor_target_velocity.data = self.target_right_wheel_velocity
                self.motorTargetVel.publish(motor_target_velocity)

                motor_current_velocity = Float32()
                motor_current_velocity.data = self.current_right_forward_velocity
                self.motorCurrentVel.publish(motor_current_velocity)
                
                # Direction of the Robot 
                if(self.target_left_wheel_velocity > 0.0):
                    self.left_wheel_direction = 'F'
                if(self.target_left_wheel_velocity < 0.0):
                    self.left_wheel_direction = 'R'
                if(self.target_left_wheel_velocity == 0.0):
                    self.left_wheel_direction = 'S'   
    
                
                if(self.target_right_wheel_velocity > 0.0):
                    self.right_wheel_direction = 'F'
                if(self.target_right_wheel_velocity < 0.0):
                    self.right_wheel_direction = 'R'
                if(self.target_right_wheel_velocity == 0.0):
                    self.right_wheel_direction = 'S'  
               
               
                if (d != 0):
                    # Calculate the Distance Traveled in x and y
                    x = cos( th ) * d
                    y = -sin( th ) * d
                    # Calculate the final position of the Robot
                    # Equation for converting a point in local reference frame to global reference frame
                    #VxG = (X_local * cos(γ)) – (Y_local * sin(γ)) 
                    #VyG = (X_local * sin(γ)) + (Y_local * cos(γ))
                    self.x = self.x + ( cos( self.th ) * x - sin( self.th ) * y )
                    self.y = self.y + ( sin( self.th ) * x + cos( self.th ) * y )
                if( th != 0):
                    self.th = self.th + th
                
                time_now = self.get_clock().now()
                #publish odom transform information
                self.odom_trans.header.stamp = time_now.to_msg()
                self.odom_trans.transform.translation.x = self.x
                self.odom_trans.transform.translation.y = self.y
                self.odom_trans.transform.translation.z = 0.0
                self.odom_trans.transform.rotation = euler_to_quaternion(0.0, 0.0, self.th)
                
                
                self.joint_state.header.stamp = time_now.to_msg()
                self.joint_state.name = ['base_right_forward_wheel_joint', 'base_right_aft_wheel_joint','base_left_forward_wheel_joint','base_left_aft_wheel_joint']
                self.joint_state.position = [self.right_forward_wheel_state, self.right_aft_wheel_state, self.left_forward_wheel_state, self.left_aft_wheel_state]
                
                # send the joint state and transform
                self.joint_pub.publish(self.joint_state)
                self.odomBroadcaster.sendTransform(self.odom_trans)
              
                odom = Odometry()
                odom.header.stamp = self.get_clock().now().to_msg()
                odom.header.frame_id = self.odom_frame_id
                odom.pose.pose.position.x = self.x
                odom.pose.pose.position.y = self.y
                odom.pose.pose.position.z = 0.0
                
                odom.pose.pose.orientation = euler_to_quaternion(0.0, 0.0, self.th)
                odom.child_frame_id = self.base_frame_id
                odom.twist.twist.linear.x = self.dx
        
                odom.twist.twist.linear.y = 0.0
                odom.twist.twist.angular.z = self.dr
                
                self.odomPub.publish(odom)

    def twistCallback(self, msg = Twist):
        self.commanded_linear_velocity = msg.linear.x
        self.commanded_angular_velocity = msg.angular.z
        

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
