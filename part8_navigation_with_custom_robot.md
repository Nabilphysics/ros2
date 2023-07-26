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
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=True
```
Start RVIZ2
```
ros2 run rviz2 rviz2
```
First, click Add button from the left bottom side. Add TF and Map. You will not see a map. In order to solve that we need to add /map in the Topic. Now the map would be loaded.</br>
Run the robot to generate the map using teleop keyboard 
```
ros2 run turtlebot3_teleop teleop_keyboard
```
then stop the teleop keyboard and save the map
```

ros2 run nav2_map_server map_saver_cli -f maps/my_world
```
we can also save(File>Save As) the configuration file from RViz<br>
Now, launch the robot,
```
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```
nav2 bringup
```
ros2 launch nav2_bringup bringup_launch.py use_sim_time:=True map:=maps/my_world.yaml
```
rviz2
```
ros2 run rviz2 rviz2
```
from rviz2, Add Map, TF, Laser Scan, Robot Model.</br>
In order to load the map, from the map dropdown menu at left > Select Topic as /map, Durability Policy as Transient Local</br>
Also, add map> rename as Global Costmap. Add Topic> Global Costmap, Color Scheme> Costmap
Now, add another map > rename it as Local Costmap. Add Topic > Local Costmap, Color Scheme > Costmap
We can save the rviz configation in File> Save As
My Rviz File link is, https://github.com/Nabilphysics/ros2/blob/main/files/my_world.rviz

## Launch File and Parameters

