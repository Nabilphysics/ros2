### Install SLAM toolbox
```
sudo apt install ros-humble-slam-toolbox
```
Launch a robot in gazebo. For example, turtlebot3
```
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```
See all available ros2 topic
```
ros2 topic list
```
To see scan topic
```
ros2 topic info /scan
```
OUTPUT:
```
Type: sensor_msgs/msg/LaserScan
Publisher count: 1
Subscription count: 0
```
Now start navigation stack
```
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True
```
other terminal we will now start navigation toolbox
```
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=Tru
```
Start RVIZ2
```
ros2 run rviz2 rviz2
```
First, click Add button from left bottom side. Add TF and Map. You will not see map. Inorder to solve that we need to add /map in the Topic. Now map whould be loaded.</br>


