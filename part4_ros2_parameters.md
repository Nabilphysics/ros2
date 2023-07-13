## What is the ROS parameter
ROS2 Parameters are explained briefly here https://docs.ros.org/en/rolling/Concepts/Basic/About-Parameters.html

Let's imagine a camera is connected to a robot. Now, you want to change the camera FPS or other settings. We can use ROS Parameter to set the camera settings in this scenario. So a parameter is a configuration value of a node. We can think of parameters as node settings. 
A node can store parameters as integers, floats, booleans, strings, and lists. In ROS 2, each node maintains its own parameters.

Now get a brief idea about Parameters from here https://docs.ros.org/en/rolling/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters.html

## Note of Parameters related day to day use commands:
```
ros2 param list
```
get parameter
```
ros2 param get <node name> <parameter>
```
Set parameter
```
ros2 run my_robot_controller <node name> --ros-args -p <<parameter name>:=<value>> 
```

```python
#! /usr/bin/env python3
import rclpy
from rclpy.node import Node


class MyNode(Node):

   def __init__(self):
       super().__init__("nabil_first_node")
       # node name : first_node but this file name my_first_node
       # remember executable name may be different in setup.py file
       self.declare_parameter("FPS")
       self.declare_parameter("Shutter")
       self.counter_ = 1
       # This node first print the following line
       self.get_logger().info("Hello from nabil_first_node")
       # This timer will call timer_callback function in every 0.5 Second
       self.create_timer(0.5, self.timer_callbac)

   def timer_callbac(self):
       self.get_logger().info("www.nabilbd.com " + "Counter: " + str(self.counter_))
       self.counter_ = self.counter_ + 1


def main(args = None):
   # Start the node
   rclpy.init(args = args)
   
   node = MyNode()
   # spin will keep the node running
   rclpy.spin(node) 

   # Finally Shutdown the node
   rclpy.shutdown()


if __name__ == '__main__':
   main()
```
