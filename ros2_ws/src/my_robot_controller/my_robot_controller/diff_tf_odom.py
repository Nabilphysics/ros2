#!/usr/bin/env python



#from rclpy import *
import rclpy.time
import rclpy
import rclpy.duration
from rclpy.clock import Clock, ROSClock
#import roslib
from math import sin, cos, pi

from rclpy.node import Node
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from rclpy.qos import QoSProfile
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped
from std_msgs.msg import Int16

#############################################################################
class DiffTf(Node):
#############################################################################

    #############################################################################
    def __init__(self):
    #############################################################################
        rclpy.init()
        super().__init__('diff_tf')
        #rclpy.init_node("diff_tf")
        qos_profile = QoSProfile(depth=10)
        
        self.nodename = self.get_name()
        self.get_logger().info("-I- %s started" % self.nodename)
        
        #### parameters #######
        #self.rate = self.get_parameter("~rate",10.0)  # the rate at which to publish the transform
        self.rate = 10.0
        #self.ticks_meter = float(self.get_parameter('ticks_meter', 250))  # The number of wheel encoder ticks per meter of travel
        self.ticks_meter = 250
        #self.base_width = float(self.get_parameter('~base_width', 1.3)) # The wheel base width in meters
        self.base_width = 1.3
        #self.base_frame_id = self.get_parameter('~base_frame_id','base_link') # the name of the base frame of the robot
        self.base_frame_id = "base_link"
        #self.odom_frame_id = self.get_parameter('~odom_frame_id', 'odom') # the name of the odometry reference frame
        self.odom_frame_id = "odom"
        #self.encoder_min = self.get_parameter('encoder_min', -32768)
        self.encoder_min = -32768
        #self.encoder_max = self.get_parameter('encoder_max', 32768)
        self.encoder_max = 32768
        #self.encoder_low_wrap = self.get_parameter('wheel_low_wrap', (self.encoder_max - self.encoder_min) * 0.3 + self.encoder_min )
        self.encoder_low_wrap = ((self.encoder_max - self.encoder_min) * 0.3 + self.encoder_min) 
        #self.encoder_high_wrap = self.get_parameter('wheel_high_wrap', (self.encoder_max - self.encoder_min) * 0.7 + self.encoder_min )
        self.encoder_high_wrap = ((self.encoder_max - self.encoder_min) * 0.7 + self.encoder_min )
        
        self.t_delta = 0.1
       
        self.t_next =  ((ROSClock().now().to_msg().sec)+(ROSClock().now().to_msg().nanosec)) + self.t_delta

        self.get_logger().info("time: " + str(self.t_next))
        
       
        
        
        # internal data
        self.enc_left = None        # wheel encoder readings
        self.enc_right = None
        self.left = 0               # actual values coming back from robot
        self.right = 0
        self.lmult = 0
        self.rmult = 0
        self.prev_lencoder = 0
        self.prev_rencoder = 0
        self.x = 0                  # position in xy plane 
        self.y = 0
        self.th = 0
        self.dx = 0                 # speeds in x/rotation
        self.dr = 0
        self.then = ((ROSClock().now().to_msg().sec)+(ROSClock().now().to_msg().nanosec))
        
        # subscriptions
        self.create_subscription(Int16,'lwheel',self.lwheelCallback, 10)
        self.create_subscription(Int16,'rwheel',self.rwheelCallback, 10)
    
        self.odomPub = self.create_publisher(Odometry, 'odom', qos_profile)
        self.get_logger().info("publisher creater")
        self.odomBroadcaster = TransformBroadcaster(self, qos= qos_profile)
        loop_rate = self.create_rate(10)

        try:
            while rclpy.ok():
                rclpy.spin_once(self)
                
                self.get_logger().info("updating")
                now = ((ROSClock().now().to_msg().sec)+(ROSClock().now().to_msg().nanosec))
                
                if now > self.t_next:
                    elapsed = now - self.then
                    self.get_logger().info(" elapsed " + str(elapsed))
                    self.then = now
                    #elapsed = elapsed.to_sec()
                    #elapsed = elapsed
                    
                    # calculate odometry
                    if self.enc_left == None:
                        d_left = 0
                        d_right = 0
                    else:
                        d_left = (self.left - self.enc_left) / self.ticks_meter
                        d_right = (self.right - self.enc_right) / self.ticks_meter
                    self.enc_left = self.left
                    self.enc_right = self.right
                
                    # distance traveled is the average of the two wheels 
                    d = ( d_left + d_right ) / 2
                    # this approximation works (in radians) for small angles
                    th = ( d_right - d_left ) / self.base_width
                    # calculate velocities
                    self.dx = d / elapsed
                    self.dr = th / elapsed
                
                    
                    if (d != 0):
                        # calculate distance traveled in x and y
                        x = cos( th ) * d
                        y = -sin( th ) * d
                        # calculate the final position of the robot
                        self.x = self.x + ( cos( self.th ) * x - sin( self.th ) * y )
                        self.y = self.y + ( sin( self.th ) * x + cos( self.th ) * y )
                    if( th != 0):
                        self.th = self.th + th
                        
                    # publish the odom information
                    quaternion = Quaternion()
                    quaternion.x = 0.0
                    quaternion.y = 0.0
                    quaternion.z = sin(self.th/2)
                    quaternion.w = cos(self.th/2)
                    self.odomBroadcaster.sendTransform(
                        (self.x, self.y, 0),
                        (quaternion.x, quaternion.y, quaternion.z, quaternion.w),
                        self.get_clock().now(),
                        self.base_frame_id,
                        self.odom_frame_id
                        )
                    
                    odom = Odometry()
                    odom.header.stamp = self.get_clock().now().to_msg()
                    odom.header.frame_id = self.odom_frame_id
                    odom.pose.pose.position.x = self.x
                    odom.pose.pose.position.y = self.y
                    odom.pose.pose.position.z = 0
                    odom.pose.pose.orientation = quaternion
                    odom.child_frame_id = self.base_frame_id
                    odom.twist.twist.linear.x = self.dx
                    odom.twist.twist.linear.y = 0
                    odom.twist.twist.angular.z = self.dr
                    self.odomPub.publish(odom)
                    loop_rate.sleep()

        except KeyboardInterrupt:
            pass

    #############################################################################
    def lwheelCallback(self, msg):
    #############################################################################
        enc = msg.data
        if (enc < self.encoder_low_wrap and self.prev_lencoder > self.encoder_high_wrap):
            self.lmult = self.lmult + 1
            
        if (enc > self.encoder_high_wrap and self.prev_lencoder < self.encoder_low_wrap):
            self.lmult = self.lmult - 1
            
        self.left = 1.0 * (enc + self.lmult * (self.encoder_max - self.encoder_min)) 
        self.prev_lencoder = enc
        
    #############################################################################
    def rwheelCallback(self, msg):
    #############################################################################
        enc = msg.data
        if(enc < self.encoder_low_wrap and self.prev_rencoder > self.encoder_high_wrap):
            self.rmult = self.rmult + 1
        
        if(enc > self.encoder_high_wrap and self.prev_rencoder < self.encoder_low_wrap):
            self.rmult = self.rmult - 1
            
        self.right = 1.0 * (enc + self.rmult * (self.encoder_max - self.encoder_min))
        self.prev_rencoder = enc

#############################################################################
#############################################################################

def main():
    #rclpy.init()
    node = DiffTf()
    
    #rclpy.spin(node)
    
    #rclpy.shutdown(node)


if __name__ == '__main__':
    main()

   
    #try:
   #     diffTf = DiffTf()
   #     diffTf.spin()
  #  except rclpy.ROSInterruptException:
   #     pass