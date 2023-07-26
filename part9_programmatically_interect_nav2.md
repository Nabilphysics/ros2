## Interect with Nav2 Programmatically
### Start the Robot(Turtlebot3_gazebo), Gazebo and RVIZ with previously saved configuration
```
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
ros2 launch nav2_bringup bringup_launch.py use_sim_time:=True map:=maps/my_world.yaml
ros2 run rviz2 rviz2
```
See current ros topics
```
ros2 topic list
```
```
ros2 topic info /initialpose
```
We should get,
```
Type: geometry_msgs/msg/PoseWithCovarianceStamped
Publisher count: 1
Subscription count: 1
```

