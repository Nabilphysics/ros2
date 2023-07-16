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


