#! /usr/bin/env python3
import serial
import time

ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)

#time.sleep(0.1)

user_input = "L"

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from std_msgs.msg import String

class PoseSubscriberNode(Node):

    def __init__(self):
        super().__init__("led_control_simple")
        self.get_logger().info("Pose Subscriber Node has been started")  
        #self.create_subscription(Twist,"/turtle1/cmd_vel", self.pose_callback, 10)
        self.publisher_ = self.create_publisher(String, 'led_state_publisher', 10)
        self.create_subscription(Twist,"/cmd_vel", self.pose_callback, 10)


    def pose_callback(self, msg: Twist):
        # ros2 run turtlebot3_teleop teleop_keyboard
        # press W to ON Led and Press S to OFF Led 
        led_state = "L"
        if msg.linear.x == 0.0:
            led_state = "L"
            ser.write(bytes(led_state, 'utf-8'))
            #ser.write(b'L')
        if msg.linear.x > 0.0:
            led_state = "H"
            ser.write(bytes(led_state, 'utf-8'))
            #ser.write(b'H')     
        led_topic_msg = String()
        led_topic_msg.data = led_state   
        self.publisher_.publish(led_topic_msg)    

        self.get_logger().info(str(msg))



def main(args = None):
    rclpy.init(args=args)
    node = PoseSubscriberNode()
    rclpy.spin(node)
    rclpy.shutdown() 
