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
To start a new world that is different from previous
```
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
```
## Install Cyclone DDS
```
sudo apt install ros-humble-rmw-cyclonedds-cpp
```
add to bashrc
```
gedit ~/.bashrc
```
add the below line just before "source /opt/ros/humble/setup.bash" 
```
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```
So the bottom part of our bashrc file now looks like
```
export TURTLEBOT3_MODEL=waffle
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/humble/setup.bash
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
source ~/ros2_ws/install/setup.bash
```
Now we will edit some parameters,
```
cd /opt/ros/humble/share/turtlebot3_navigation2/param
sudo gedit waffle.yaml
```
Now change robot_model_type: "differential" to
```
robot_model_type: "nav2_amcl::DifferentialMotionModel"
```

## Starting navigation
```
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=maps/my_map.yaml
```

