Useful GitHub Link: https://github.com/ros-simulation/gazebo_ros_pkgs/tree/ros2 </br>
In this repo if we go to https://github.com/ros-simulation/gazebo_ros_pkgs/blob/ros2/gazebo_plugins/include/gazebo_plugins/gazebo_ros_diff_drive.hpp link we can see example usage in comment section. </br>

Step 1: Add Gazibo Diff_drive plugin to mobile_base_gazebo.xacro File
```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

    <gazebo reference="base_link">
        <material>Gazebo/Blue</material>
    </gazebo>

    <gazebo reference="right_wheel_link">
        <material>Gazebo/Grey</material>
    </gazebo>

    <gazebo reference="left_wheel_link">
        <material>Gazebo/Grey</material>
    </gazebo>

    <gazebo reference="caster_wheel_link">
        <material>Gazebo/Grey</material>
        <!-- mu1 and mu2 to make friction less -->
        <mu1 value = "0.1" />
        <mu2 value = "0.1" />
    </gazebo>


    <gazebo>
        <plugin name="diff_drive_controller" filename="libgazebo_ros_diff_drive.so">

            <!-- Update rate in Hz -->
            <update_rate>50</update_rate>

            <!-- wheels -->
            <left_joint>base_left_wheel_joint</left_joint>
            <right_joint>base_right_wheel_joint</right_joint>

            <!-- kinematics -->
            <wheel_separation>0.45</wheel_separation>
            <wheel_diameter>0.2</wheel_diameter>

                <!-- limits 
                <max_wheel_torque>20</max_wheel_torque>
                <max_wheel_acceleration>1.0</max_wheel_acceleration>
                -->
            <!-- input 
            <command_topic>cmd_vel</command_topic>
            -->

            <!-- output -->
            <publish_odom>true</publish_odom>
            <publish_odom_tf>true</publish_odom_tf>
            <publish_wheel_tf>true</publish_wheel_tf>

            <odometry_topic>odom</odometry_topic>
            <odometry_frame>odom</odometry_frame>
            <robot_base_frame>base_footprint</robot_base_frame>

        </plugin>
    </gazebo>

</robot>

```
Step 2: build </br>
Step 3: Publish velocity command to cmd_vel topic
```
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3},angular:{z: 0.0}}"
```
Note: if we see rqt_graph we can see diff_drive_controller node publishing to odom topic and tf directly. Usually we expect it publish to joint_states and joint_state publishes to tf. </br>
![alt text](https://github.com/Nabilphysics/ros2/blob/main/images/gazibo_diff_drive_rqt_graph.png)

