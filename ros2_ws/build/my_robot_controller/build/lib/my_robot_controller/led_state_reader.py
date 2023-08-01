import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class PoseSubscriberNode(Node):

    def __init__(self):
        super().__init__("led_state_reader_node")
        self.get_logger().info("ed_state_reader_node Node has been started")  
        self.create_subscription(String,"/led_state_publisher", self.pose_callback, 10)
      


    def pose_callback(self, msg: String):
         

        self.get_logger().info(str(msg))



def main(args = None):
    rclpy.init(args=args)
    node = PoseSubscriberNode()
    rclpy.spin(node)
    rclpy.shutdown() 