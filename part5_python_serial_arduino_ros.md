### To See USB port in Ubuntu
```
lsusb
```
## If serial port does not show in Arduino IDE on Ubuntu
```
sudo apt remove brltty 
```
Plug the arduino or dongle off and on

## Arduino and Python Serial Communication Example
### Python Code
```python
import serial
import time

ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1)
time.sleep(0.1)

user_input = "L"

while user_input != 'q':
    user_input = input("H = LED ON, L = LED_OFF, q = Quit : ")

    byte_data = bytes(user_input, 'utf-8')
    byte_data_capitalize = byte_data.capitalize()
   
    ser.write(byte_data_capitalize) 
    time.sleep(0.5) 

for i in range(5):
    ser.write(b'H')   
    time.sleep(0.2)        
    ser.write(b'L')  
    time.sleep(0.2)

print('You entered q so, I am Exiting the program')

ser.close()

```

### Arduino Code
```c++
const int ledPin = 13; 
int incomingByte;      // variable stores incoming serial data

void setup() {
  Serial.begin(115200);
  pinMode(ledPin, OUTPUT);
}

void loop() {
  if (Serial.available() > 0) {
    // read the oldest byte in the serial buffer:
    incomingByte = Serial.read();
    // if it's a capital H (ASCII 72), turn on the LED:
    if (incomingByte == 'H') {
      digitalWrite(ledPin, HIGH);
    }
    // if it's an L (ASCII 76) turn off the LED:
    if (incomingByte == 'L') {
      digitalWrite(ledPin, LOW);
    }
  }
}
```
### ROS2 Humble node to control LED from turtlebot3 teleop_keyboard
#### Step 1: Make a node using the code. Arduino code will be same as before.
```python
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

class PoseSubscriberNode(Node):

    def __init__(self):
        super().__init__("led_control_simple")
        self.get_logger().info("Pose Subscriber Node has been started")  
        #self.create_subscription(Twist,"/turtle1/cmd_vel", self.pose_callback, 10)
        self.create_subscription(Twist,"/cmd_vel", self.pose_callback, 10)


    def pose_callback(self, msg: Twist):
        # ros2 run turtlebot3_teleop teleop_keyboard
        # press W to ON Led and Press S to OFF Led 
        if msg.linear.x == 0.0:
            ser.write(b'L')
        if msg.linear.x > 0.0:
            ser.write(b'H')        

        self.get_logger().info(str(msg))



def main(args = None):
    rclpy.init(args=args)
    node = PoseSubscriberNode()
    rclpy.spin(node)
    rclpy.shutdown() 
```
#### Step 2: Run teleop keyboard from the turtlebot3 package
```
ros2 run turtlebot3_teleop teleop_keyboard
```

## Control a LED with Arduino: 1 ROS Node to Control and Publish LED State to a Topic and 1 ROS Node to Read LED State
### Step 1: 
First, we will create a Node name "led_control_simple" which will control Arduino LED and will make a topic name "led_state_publisher". This node will take keyboard input from Turtlebot3 teleop_keybaord. If we press 'W', the led will turn ON, and if we press 'S' it will turn OFF the led. To start the teleop keyboard 
```
ros2 run turtlebot3_teleop teleop_keyboard
```
if you need to download turtlebot3 related package follow https://github.com/Nabilphysics/ros2/blob/main/part5_ros2_nav2.md
</br>
Code for our "led_control_simple" node is:
```python
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

````
### Step 2: Create State Reader Node ""led_state_reader_node
```python
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
```
### Step 3: Upload Arduino Code
```c++
const int ledPin = 13; 
int incomingByte;      // variable stores incoming serial data

void setup() {
  Serial.begin(115200);
  pinMode(ledPin, OUTPUT);
}

void loop() {
  if (Serial.available() > 0) {
    // read the oldest byte in the serial buffer:
    incomingByte = Serial.read();
    // if it's a capital H (ASCII 72), turn on the LED:
    if (incomingByte == 'H') {
      digitalWrite(ledPin, HIGH);
    }
    // if it's an L (ASCII 76) turn off the LED:
    if (incomingByte == 'L') {
      digitalWrite(ledPin, LOW);
    }
  }
}
```
### Step 4: Edit setup.py file
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
    maintainer_email='nabil@cruxbd.com',
    description='www.nabilbd.com',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "led_control_simple = my_robot_controller.led_control_simple:main",
            "led_state_reader = my_robot_controller.led_state_reader:main"
           
        ],
    },
)

```
### Step 5: Build using colon build
```
cd ~/ros2_ws
colcon build --symlink-install
```
### Step 6: Run these two nodes and turtlebot3 teleop keyboard
```
ros2 run my_robot_controller led_control_simple
```
```
ros2 run my_robot_controller led_state_reader
```
```
ros2 run turtlebot3_teleop teleop_keyboard
```
Now, if you press 'W' the led should be turned ON, and if you press 'S' the led should be turned OFF

## How to monitor various topic
We can use a tool name rqt to visualize the data
```
rqt
```
then, Plugins Menu > Topics > Topics Monitor

![alt text](https://github.com/Nabilphysics/ros2/blob/main/images/rqt_topic_monitor.png)

## How to publish data from rqt
```
rqt
```
then, Plugins Menu > Topics > Message Publisher
![alt text](https://github.com/Nabilphysics/ros2/blob/main/images/rqt_topic_publisher.png)

