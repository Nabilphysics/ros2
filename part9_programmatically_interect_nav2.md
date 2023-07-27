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
Here, orientation is in Quaternion Angle</br>
To see ros2 action and info
```
ros2 action list
ros2 action info /navigate_to_pose
```
We should get
```
Action: /navigate_to_pose
Action clients: 2
    /bt_navigator
    /waypoint_follower
Action servers: 1
    /bt_navigator
```
### Simple Python Commandar API
To install
```
sudo apt install ros-humble-nav2-simple-commander
```
create a python file
```
touch nav2_test.py
chmod +x nav2_test.py
```
Install euler to quaternion transformation package
```
sudo apt install ros-humble-tf-transformations
sudo apt install python3-transforms3d
```
Open in Visual Studio Code
```
code nav2_test.py
```
Code:
```python
#!/usr/bin/env python3
# Code from Edouard Renard
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import tf_transformations

def main():
    # --- Init
    rclpy.init()
    nav = BasicNavigator()

    # Set initial pose
    q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0,0.0,0.0)
    initial_pose = PoseStamped()
    initial_pose.header.frame_id = 'map'
    initial_pose.header.stamp = nav.get_clock().now().to_msg()
    initial_pose.pose.position.x = 0.0
    initial_pose.pose.position.y = 0.0
    initial_pose.pose.position.z = 0.0

    initial_pose.pose.orientation.x = q_x
    initial_pose.pose.orientation.y = q_y
    initial_pose.pose.orientation.z = q_z
    initial_pose.pose.orientation.w = q_w
    
    nav.setInitialPose(initial_pose)
    
    # Wait of Nav2
    nav.waitUntilNav2Active()
    
    # Send Nav2 Goal
    # PI == 3.14 == 180
    # PI/2 == 1.57 == 90
    q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0,0.0,1.57)
    goal_pose = PoseStamped()
    goal_pose.header.frame_id = 'map'
    goal_pose.header.stamp = nav.get_clock().now().to_msg()
    goal_pose.pose.position.x = 3.5
    goal_pose.pose.position.y = 1.0
    goal_pose.pose.position.z = 0.0
    goal_pose.pose.orientation.x = q_x
    goal_pose.pose.orientation.y = q_y
    goal_pose.pose.orientation.z = q_z
    goal_pose.pose.orientation.w = q_w
    
    nav.goToPose(goal_pose) 

    while not nav.isTaskComplete():
        feedback = nav.getFeedback()
        print(feedback)
    
    print(nav.getResult())
    #shutdown
    rclpy.shutdown()

if __name__== '__main__':
    main()
```
Now start turtlebot3, gazebo and rviz again
```
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
ros2 launch nav2_bringup bringup_launch.py use_sim_time:=True map:=maps/my_map.yaml
ros2 run rviz2 rviz2
```
or
```
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=maps/my_map.yaml
```
Now run the python code.

## Waypoint Follower Mode
```python
#!/usr/bin/env python3
# Code from Edouard Renard
import rclpy
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
import tf_transformations

def create_pose_stamped(navigator: BasicNavigator, position_x, position_y, orientation_z):
    q_x, q_y, q_z, q_w = tf_transformations.quaternion_from_euler(0.0, 0.0, orientation_z)
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = position_x
    pose.pose.position.y = position_y
    pose.pose.position.z = 0.0
    pose.pose.orientation.x = q_x
    pose.pose.orientation.y = q_y
    pose.pose.orientation.z = q_z
    pose.pose.orientation.w = q_w
    return pose


def main():
    # --- Init
    rclpy.init()
    nav = BasicNavigator()

    # Set initial pose
    initial_pose = create_pose_stamped(nav, 0.0,0.0, 0.0)
    # ------- Uncomment if Intial Pose is not set already --------
    #nav.setInitialPose(initial_pose)
    
    # Wait of Nav2
    nav.waitUntilNav2Active()
    
    # Send Nav2 Goal
    # PI == 3.14 == 180
    # PI/2 == 1.57 == 90
    goal_pose1 = create_pose_stamped(nav, 3.5, 1.0, 1.57)
    goal_pose2 = create_pose_stamped(nav, 2.0, 2.5, 3.14)
    goal_pose3 = create_pose_stamped(nav, 0.5, 1.0, -1.57)
    goal_pose_return_home = create_pose_stamped(nav, 0.0, 0.0, 0.0)
    
    # Go to one pose
    # nav.goToPose(goal_pose_return_home) 
    # while not nav.isTaskComplete():
    #    feedback = nav.getFeedback()
    #    print(feedback)
   
    
     # --- Follow Waypoints ---
    waypoints = [goal_pose1, goal_pose2, goal_pose3, goal_pose_return_home]
    
   
    while(1):
        nav.followWaypoints(waypoints)
        while not nav.isTaskComplete():
            feedback = nav.getFeedback()
            print(feedback)

    print(nav.getResult())
    #shutdown
    rclpy.shutdown()

if __name__== '__main__':
    main()
```
One of the important things is to wait until a task has been completed.
```python
while not nav.isTaskComplete():
            feedback = nav.getFeedback()
            print(feedback)
```




