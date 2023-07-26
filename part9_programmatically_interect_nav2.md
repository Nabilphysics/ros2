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
We can see the topic
```
ros2 topic echo /initialpose
```
Now if we give 2D Pose Estimate in RVIZ we should get like this
```
header:
  stamp:
    sec: 1690364032
    nanosec: 557806155
  frame_id: map
pose:
  pose:
    position:
      x: -2.0083489418029785
      y: -0.5036280155181885
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: -0.0006784694107932677
      w: 0.9999997698396028
  covariance:
  - 0.25
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.25
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.0
  - 0.06853891909122467
---
```

