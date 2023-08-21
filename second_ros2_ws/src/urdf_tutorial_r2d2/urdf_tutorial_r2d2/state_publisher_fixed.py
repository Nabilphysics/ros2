#https://docs.ros.org/en/foxy/Tutorials/Intermediate/URDF/Using-URDF-with-Robot-State-Publisher.html#
from math import sin, cos, pi
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from geometry_msgs.msg import Quaternion
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster, TransformStamped
from std_msgs.msg import Int16
from std_msgs.msg import Float32

class StatePublisher(Node):
    degree = pi / 180.0
    # robot state
    
    enc = 0.5

    def __init__(self):
       
        super().__init__('state_publisher_fixed')

        qos_profile = QoSProfile(depth=10)
        self.joint_pub = self.create_publisher(JointState, 'joint_states', qos_profile)
        # subscriptions
        self.create_subscription(Float32,'swivel_position',self.swivel_position, 10)

        self.broadcaster = TransformBroadcaster(self, qos=qos_profile)
        self.nodeName = self.get_name()
        self.get_logger().info("{0} started".format(self.nodeName))

        # message declarations
        self.odom_trans = TransformStamped()
        self.odom_trans.header.frame_id = 'odom'
        self.odom_trans.child_frame_id = 'axis'

        self.joint_state = JointState()
        self.joint_state.name = ['swivel', 'tilt', 'periscope']

        self.swivel_position = 1.57
        self.tile_position = 1.0
        self.periscope_position = 0.5
        
        self.create_timer(0.1, self.update)

    def update(self):
        self.swivel_position = self.enc
        
        # update joint_state
        now = self.get_clock().now()
        self.joint_state.header.stamp = now.to_msg()
       
        self.joint_state.position = [self.swivel_position, self.tile_position, self.periscope_position]
        # update transform
        # (moving in a circle with radius=2)
        self.odom_trans.header.stamp = now.to_msg()
        self.odom_trans.transform.translation.x = 2.0
        self.odom_trans.transform.translation.y = 1.0
        self.odom_trans.transform.translation.z = 0.0
        self.odom_trans.transform.rotation = euler_to_quaternion(0.0, 1.57, 0.0) # roll,pitch,yaw
        
        # send the joint state and transform
        self.joint_pub.publish(self.joint_state)
        self.broadcaster.sendTransform(self.odom_trans)
        # Create new robot state
       

    def swivel_position(self, msg):
            self.enc = msg.data  
            #print(self.enc)    
            self.get_logger().info(" enc: " + str(self.enc))    

def euler_to_quaternion(roll, pitch, yaw):
    qx = sin(roll/2) * cos(pitch/2) * cos(yaw/2) - cos(roll/2) * sin(pitch/2) * sin(yaw/2)
    qy = cos(roll/2) * sin(pitch/2) * cos(yaw/2) + sin(roll/2) * cos(pitch/2) * sin(yaw/2)
    qz = cos(roll/2) * cos(pitch/2) * sin(yaw/2) - sin(roll/2) * sin(pitch/2) * cos(yaw/2)
    qw = cos(roll/2) * cos(pitch/2) * cos(yaw/2) + sin(roll/2) * sin(pitch/2) * sin(yaw/2)
    return Quaternion(x=qx, y=qy, z=qz, w=qw)

def main(args = None):
    rclpy.init(args = args)
    node = StatePublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
