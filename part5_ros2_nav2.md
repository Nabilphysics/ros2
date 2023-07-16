Learning Source:</br>
https://youtu.be/idQb2pB-h2Q

```
sudo apt update
```
```
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-turtlebot3*
```
```
gedit ~/.bashrc
```
Add
```
export TURTLEBOT3_MODEL=waffle
```
before the line source /opt/ros/humble/setup.bash

To see
```
printenv | grep TURTLE
```
Test gazebo 
```
gazebo
```
```
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```
To see in rviz
```
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
```
To drive the robot start a keyboard
```
ros2 run turtlebot3_teleop teleop_keyboard
```
To save the map
```
ros2 run nav2_map_server map_saver_cli -f maps/my_map
```



