### Source ros2 :
For ROS2 Version: humble
```
source /opt/ros/humble/setup.bash
```
For ROS2 Version: Jazzy
```
source /opt/ros/jazzy/setup.bash
```

### Add source in bashrc
Add ```su``` for root privilege
```
nano ~/.bashrc
```
or if you are already in the home directory
```
nano .bashrc
```
then add the below command in the last line and save 
For ROS2 Version: humble
```
source /opt/ros/humble/setup.bash
```
For ROS2 Version: Jazzy
```
source /opt/ros/jazzy/setup.bash
```
### Demo Node to Listen:
```
ros2 run demo_nodes_py listener
```
Demo Node to Publish:
```
ros2 run demo_nodes_cpp talker
```
## Info
### ROS2 Node List
```
ros2 node list
```
### ros2 node info
```
ros2 node info <node_name>
```
### ros2 topic list
```
ros2 topic list
```
```
ros2 topic list -t
```
### ROS2 Topic info:
```
ros2 topic info /chatter
```
Example Output:</br>
Type: std_msgs/msg/String</br>
Publisher count: 1 </br>
Subscription count: 0 </br>


### ROS2 Message Info
```
ros2 interface show std_msgs/msg/String
```
Example Output: </br>
 #This was originally provided as an example message.</br>
 #It is deprecated as of Foxy</br>
 #It is recommended to create your own semantically meaningful message.</br>
 #However if you would like to continue using this please use the equivalent in example_msgs.</br>
string data</br>

### Subscribe a Topic
```
ros2 topic echo /chatter
```
### Publish to a topic
```
ros2 topic pub -r 9 /chatter std_msgs/msg/String "{data: 'Nabil'}"
```
where, -r is the rate, </chatter> is a topic publising by <talker> node 

### Topic Publishing Frequency
```
ros2 topic hz /chatter
```
### Bandwidth
```
ros2 topic bw /chatter
```
### See what is going on Graphically
 ```
 rqt_graph
 ```
 ### Play with Turtlesim Simulator
 Start Turtlesim Graphical Tool
 ```
 ros2 run turtlesim turtlesim_node
 ```
 start turtle teleop keyboard to be able to control the turtle
 ```
 ros2 run turtlesim turtle_teleop_key
 ```
 here turtlesim is a package and turtlesim_node and turtle_teleop_key are the node.
 
 ## ROS2 Workspace creation
 ### Install Colcon Build Tool
```
 sudo apt install python3-colcon-common-extensions
 ```
 enable colcon autocompletion
 ```
 cd /usr/share/colcon_argcomplete/hook/
 ```
use command ls and you should see colcon-argcomplete.bash  colcon-argcomplete.zsh
Now we have to add this in bashrc file
```
 nano ~/.bashrc
 ```
 add below line and save
 ```
 source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
 ```
 ### Create Workspace
 go to home directory using command
 ```
 cd
 ```
 then
```
 mkdir ros2_ws
```
 enter ros2_ws folder/directory using 
 ```
 cd ros2_ws
 ```
 create a folder/directory src using
 ```
 mkdir src
 ```
 then from ros2_ws directory type
 ```
 colcon build
 ```
 ## Source our newly created workspace to be able to use its node
 ```
 source ~/ros2_ws/install/setup.bash
 ```
 ## ROS2 Python Package creation
 Useful Video: https://youtu.be/iBGZ8LEvkCY
 
 go to src directory using this command
 ```cd ~/ros2_ws/src/```
 then create package using this command
 ```
 ros2 pkg create my_robot_controller --build-type ament_python --dependencies rclpy
 ```
 here, 
 Package Name: my_robot_controller
 Build Tool : Colcon
 Build System : ament_python for Python package or ament_cmake for C++ package
 rclpy : python client library for ROS2
 rclcpp : C++ client library for ROS2
 
 download visual studio code using the following command
 ``` 
 sudo snap install code --classic 
 ```
 Open Visual Studio in the same directory just type
 ``` 
 code . 
 ```
 visual studio will be open and click trust

```
colcon build
```
if you find any error you may need to downgrade python package setuptools to 58.2.0 </br>
If pip3 is not installed
```
sudo apt install python3-pip
```
then
```
pip3 install setuptools==58.2.0
```
Now build using 
```
colcon build
```


 
 ### Creating a new node in our workspace and package
 Go to the directory ros2_ws > src > my_robot_controller > my_robot_controller 
 ```
 cd ~/ros2_ws/src/my_robot_controller/my_robot_controller
 ```
 create my_first_node.py file
 ```
touch my_first_node.py
 ```
 Now change permission to make it executable
 ```
 chmod +x my_first_node.py
 ```
Now open visual studio in the same directory (i.e. ros2_ws > src > my_robot_controller > my_robot_controller) 
 ```
 code .
 ```
 and add following code and save the ```my_first_node.py``` file

 ```python
#! /usr/bin/env python3
import rclpy
from rclpy.node import Node


class MyNode(Node):

    def __init__(self):
        super().__init__("nabil_first_node")
        # node name : first_node but this file name my_first_node
        # remember executable name may be different in setup.py file
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
Now open setup.py file and add code in console_scripts. Actually it's the instruction to create executables.</br>
We are going to add ```"test_node = my_robot_controller.my_first_node:main"```</br>
Where,</br>
Executable Name = test_node</br>
Package Name = my_robot_controller</br>
Python File Name = my_first_node</br>
main function = main</br>

Full code ```setup.py```

```python
from setuptools import setup

package_name = 'my_robot_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nabil',
    maintainer_email='www.nabilbd.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "test_node = my_robot_controller.my_first_node:main"
            # Executable Name = test_node
            # Package Name = my_robot_controller
            # Python File Name = my_first_node
            # main function = main
        ],
    },
)

```
From ```~/ros2_ws``` workspace build the update that we have made
```
colcon build
```
Now run the node
```
ros2 run my_robot_controller test_node
```
Everytime we made a change in our code we have to build. But Python is an interpreted lanugae. An interpreted language is a programming language which are generally interpreted, without compiling a program into machine instructions. </br>

</bn>to avoid build every time we can do following thing
in ```ros2_ws``` workspace
```
colcon build --symlink-install
```
then source bashrc
```
source ~/.bashrc
```
Warning: If package not found error popup then
```
cd ~/ros2_ws
colcon build
source install/setup.bash
```
From now you can just change the python code and after saving change will be reflected without building. 
Now run the node and see visually using ```rqt_graph```

![alt text](https://github.com/Nabilphysics/ros2/blob/main/images/rqt_graph_first_node.JPG)

You can see node info while node is running 
```
ros2 node list
```
```
ros2 node info /nabil_first_node
```
### Package Related Command
List of All the Packages
```
ros2 pkg list
```
Search or Filter package
```
ros2 pkg list | grep PACKAGE_NAME
```
See executables under certain package
```
ros2 pkg executables turtlesim
```
See package location ```ros2 pkg prefix --share PACKAGE_NAME```
```
ros2 pkg prefix --share turtlesim
```
Runs executables provided by a package 
```
ros2 run <package_name> <executable_file>
```
### Turtlesim Draw Circle
lets create another node name ```draw_circle``` as like ```test_node```</br>
follow previous node creation process and also update ```setup.py``` and ```package.xml```

draw_circle.py Code:
```python
#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class DrawCircleNode(Node):
    def __init__(self):
        super().__init__("draw_circle")
        self.cmd_vel_publisher_ = self.create_publisher(Twist, "/turtle1/cmd_vel",10)
        self.timer_ = self.create_timer(0.5, self.send_velocity_command)
        self.get_logger().info("Draw Circle Node has been Started")

    def send_velocity_command(self):
        msg = Twist()
        msg.linear.x = 2.0
        msg.angular.z = 1.0
        self.cmd_vel_publisher_.publish(msg)

def main(args = None):
    rclpy.init(args = args)
    node = DrawCircleNode()
    rclpy.spin(node)
    rclpy.shutdown()
```
setup.py code:
```python
from setuptools import setup

package_name = 'my_robot_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nabil',
    maintainer_email='www.nabilbd.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "test_node = my_robot_controller.my_first_node:main",
            # Executable Name = test_node
            # Package Name = my_robot_controller
            # Python File Name = my_first_node
            # main function = main
            "draw_circle = my_robot_controller.draw_circle:main"
        ],
    },
)
```
package.xml file:
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_robot_controller</name>
  <version>0.0.0</version>
  <description>TODO: Package description</description>
  <maintainer email="nabil@todo.todo">nabil</maintainer>
  <license>TODO: License declaration</license>

  <depend>rclpy</depend>
  <depend>geometry_msgs</depend>
  <depend>turtlesim</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

## Subscriber Node Example Code
```python
#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose

class PoseSubscriberNode(Node):

    def __init__(self):
        super().__init__("pose_subscriber")
        self.get_logger().info("Pose Subscriber Node has been started")  
        self.create_subscription(Pose,"/turtle1/pose", self.pose_callback, 10)


    def pose_callback(self, msg: Pose):
        self.get_logger().info(str(msg))    





def main(args = None):
    rclpy.init(args=args)
    node = PoseSubscriberNode()
    rclpy.spin(node)
    rclpy.shutdown() 
```
## Run TurtleSim from Commandline

Step 1: Run TurtleSim Node
```
ros2 run turtlesim turtlesim_node
```
Step 2: See all ros2 topic 
```
ros2 topic list
```
Step 3: We are interested in ```/turtle1/cmd_vel``` topic. So we need more info
```
ros2 topic info /turtle1/cmd_vel
```
Step 4: Now we will see the Type:
```
ros2 interface show geometry_msgs/msg/Twist
```
Step 5: We have to Publish Twist message in ```/turtle1/cmd_vel``` topic command format is </br>
```
ros2 topic pub -t 1 <topic> <message_type> <message>
```
Where, -t is how many times we want to send the message</br>
Tip: Type ```ros2 topic pub -t 1 /turtle1/cmd_vel geometry_msgs/msg/Twist "lin``` and press ```tab``` and then fill the message
So we are sending
```
ros2 topic pub -t 1 /turtle1/cmd_vel geometry_msgs/msg/Twist "linear:
  x: 2.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"
  ```
  You will see that Turtle is moving towards x direction.  

## ROS2 Service
Services are another method of communication for nodes in the ROS graph. Services are based on a call-and-response model, versus topics’ publisher-subscriber model. While topics allow nodes to subscribe to data streams and get continual updates, services only provide data when they are specifically called by a client.</br>
https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services.html
</br>

### Service Related Command & Play with service
Run a demo service
```
ros2 run demo_nodes_cpp add_two_ints_server
```
Our goal is to send two number and this service will give us back the summation result. </br> 
See service list
```
ros2 service list
```
To see in details 
```
ros2 service list -t
```
```
ros2 service type /add_two_ints
```
```
ros2 interface show example_interfaces/srv/AddTwoInts
```
Call a Service
```
ros2 service call <service_name> <service_type> <arguments>
```
Now we will send two number to the service
```
ros2 service call /add_two_ints example_interfaces/srv/AddTwoInts "{'a':2,'b':3}"
```
## TurtleSim related service </br>
https://docs.ros.org/en/foxy/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters.html </br>
First, start TurtleSim Node
```
ros2 run turtlesim turtle_node
```
More Turtle:
```
ros2 service call /spawn turtlesim/srv/Spawn "{'x': 5.0, 'y':5.0, 'theta':90.0, 'name':'commando'}"
```
Clear Trace:
```
ros2 service call /clear std_srvs/srv/Empty 
```
## Turtle field color change based on turtle position
Credit:  https://youtu.be/vCTbUgw6k8U
```python
#!/usr/bin/env python3
# Code Credit: https://youtu.be/vCTbUgw6k8U

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from turtlesim.srv import SetPen
from functools import partial

class TurtleControllerNode(Node):
    
    def __init__(self):
        super().__init__("turtle_controller")
        self.previous_x_ = 0
        self.cmd_vel_publisher_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.pose_subscriber_ = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 10)
        self.get_logger().info("Turtle Controller with Servcie")

    def pose_callback(self, pose: Pose):
        cmd = Twist()
        if pose.x > 9.0 or pose.x < 2.0 or pose.y > 9.0 or pose.y < 2.0:
            cmd.linear.x = 1.0
            cmd.angular.z = 0.9
        else:
            cmd.linear.x = 5.0
            cmd.angular.z = 0.0
        self.cmd_vel_publisher_.publish(cmd)

        if pose.x > 5.5 and self.previous_x_ <= 5.5:
            self.previous_x_ = pose.x
            self.get_logger().info("Set color to red")
            self.call_set_pen_service(255, 0, 0, 3, 0)
        elif pose.x <= 5.5 and self.previous_x_ > 5.5:
            self.previous_x_ = pose.x
            self.get_logger().info("Set color to red")
            self.call_set_pen_service(0, 255 ,0, 3, 0)

    def call_set_pen_service(self, r, g, b, width, off):
        client = self.create_client(SetPen, "/turtle1/set_pen")
        while not client.wait_for_service(1.0): # 1.0 is timeout in second
            self.get_logger().warn("Waiting for service ...")

        request = SetPen.Request()
        request.r = r
        request.g = g
        request.b = b
        request.width = width
        request.off = off

        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_set_pen))

    def callback_set_pen(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error("Service call failed: %r" % (e,))

def main(args = None):
    rclpy.init(args = args)
    node = TurtleControllerNode()
    rclpy.spin(node)
    rclpy.shutdown()
```
```setup.py``` file:
```python
from setuptools import setup

package_name = 'my_robot_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nabil',
    maintainer_email='www.nabilbd.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "test_node = my_robot_controller.my_first_node:main",
            # Executable Name = test_node
            # Package Name = my_robot_controller
            # Python File Name = my_first_node
            # main function = main
            "draw_circle = my_robot_controller.draw_circle:main",
            "pose_subscriber = my_robot_controller.pose_subscriber:main",
            "turtle_controller = my_robot_controller.turtle_controller:main"
        ],
    },
)
```
## ROS2 Parameter
https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Parameters/Understanding-ROS2-Parameters.html

## ROS2 Action 
https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html

## ROS2 Launch File
The launch system in ROS 2 is responsible for helping the user describe the configuration of their system and then execute it as described. The configuration of the system includes what programs to run, where to run them, what arguments to pass them, and ROS-specific conventions which make it easy to reuse components throughout the system by giving them each a different configuration. It is also responsible for monitoring the state of the processes launched, and reporting and/or reacting to changes in the state of those processes.

Launch files written in Python, XML, or YAML can start and stop different nodes as well as trigger and act on various events. See Using Python, XML, and YAML for ROS 2 Launch Files for a description of the different formats. The package providing this framework is launch_ros, which uses the non-ROS-specific launch framework underneath.

Official Tutorial: https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Creating-Launch-Files.html

Step 1: Create a new directory to store your launch files:
```
mkdir launch
```
Step 2: Write the launch file
```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='turtlesim',
            namespace='turtlesim1',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='turtlesim',
            namespace='turtlesim2',
            executable='turtlesim_node',
            name='sim'
        ),
        Node(
            package='turtlesim',
            executable='mimic',
            name='mimic',
            remappings=[
                ('/input/pose', '/turtlesim1/turtle1/pose'),
                ('/output/cmd_vel', '/turtlesim2/turtle1/cmd_vel'),
            ]
        )
    ])
   ```
    Step 3: ROS2 Launch
   ```
   cd launch
   ```
   ```
   ros2 launch turtlesim_mimic_launch.py
   ```
## Recording and Playback Data
https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Recording-And-Playing-Back-Data/Recording-And-Playing-Back-Data.html
