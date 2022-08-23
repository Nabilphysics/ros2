### Source ros2 :
```
source /opt/ros/humble/setup.bash
```
### Add source in bashrc
```
nano ~./bashrc
```
then add below command in the last line and save 
```
source /opt/ros/humble/setup.bash
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
