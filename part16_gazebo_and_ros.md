## Gazebo and ROS
Gazebo is an independent Simulation Tool. gazebo_ros package can bridge Gazebo and ROS environment. </br>
![alt text](https://github.com/Nabilphysics/ros2/blob/main/images/gazebo_ros.png)
Credit: Robotics Backend (https://www.youtube.com/@RoboticsBackEnd)
In gazebo, there are many plugins to control the robot. Those plugins simulate real physical robot's encoders, wheels, etc. <br>

## Adding Inertia and Collision Tags in the URDF
Resources:</br>
http://wiki.ros.org/urdf/Tutorials/Adding%20Physical%20and%20Collision%20Properties%20to%20a%20URDF%20Model </br>
List of moments of Inertia: https://en.wikipedia.org/wiki/List_of_moments_of_inertia#List_of_3D_inertia_tensors <br>
Step 1: Add inertia for box, cylinder, and sphere as xacro:maco(like creating a function) in common_properties.xacro file like </br>
```xml
<xacro:macro name="box_inertia" params="m l w h xyz rpy ">
        <inertial>
            <origin xyz="${xyz}" rpy="${rpy}"/>
            <mass value="${m}"/>
            <inertia ixx="${(m/12) * (h*h + l*l)}" ixy="0.0" ixz="0.0" 
                     iyy="${(m/12) * (w*w + l*l)}"  iyz="0.0" izz="${(m/12) * (w*w + h*h)}" />
        </inertial>
    </xacro:macro>
```
Step 2: Provide inertia as a parameter for each link in mobile_base.xacro file like
```xml
<xacro:box_inertia m="5.0" l="${base_length}" w="${base_width}" h="${base_height}"
                            xyz="0 0 ${base_height/2.0}" rpy="0 0 0" />
```
Step 3: Add Collision(now copy of geometry and origin) like
```xml
<collision>
            <geometry>
                <box size="${base_length} ${base_width} ${base_height}"/>
            </geometry>
            <origin xyz="0 0 ${base_height/2.0}" rpy="0 0 0"/>
        </collision>
```

so the full code becomes </br>
Code : my_robot.urdf.xacro</br>
Credit: https://www.youtube.com/@RoboticsBackEnd
```xml
<?xml version="1.0"?>
<robot name="my_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">

    <xacro:include filename="common_properties.xacro"/>
    <xacro:include filename="mobile_base.xacro"/>
    
</robot>
```
Code : common_properties.xacro</br>
Credit: https://www.youtube.com/@RoboticsBackEnd
```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">
    
    <material name="blue">
        <color rgba="0 0 0.5 1" />
    </material>

    <material name="grey">
        <color rgba="0.5 0.5 0.5 1" />
    </material>

    <xacro:macro name="box_inertia" params="m l w h xyz rpy">
        <inertial>
            <origin xyz="${xyz}" rpy="${rpy}" />
            <mass value="${m}" />
            <inertia ixx="${(m/12) * (h*h + l*l)}" ixy="0" ixz="0"
                     iyy="${(m/12) * (w*w + l*l)}" iyz="0"
                     izz="${(m/12) * (w*w + h*h)}" />
        </inertial>
    </xacro:macro>

    <xacro:macro name="cylinder_inertia" params="m r h xyz rpy">
        <inertial>
            <origin xyz="${xyz}" rpy="${rpy}" />
            <mass value="${m}" />
            <inertia ixx="${(m/12) * (3*r*r + h*h)}" ixy="0" ixz="0"
                     iyy="${(m/12) * (3*r*r + h*h)}" iyz="0"
                     izz="${(m/2) * (r*r)}" />
        </inertial>
    </xacro:macro>

    <xacro:macro name="sphere_inertia" params="m r xyz rpy">
        <inertial>
            <origin xyz="${xyz}" rpy="${rpy}" />
            <mass value="${m}" />
            <inertia ixx="${(2/5) * m * r * r}" ixy="0" ixz="0"
                     iyy="${(2/5) * m * r * r}" iyz="0"
                     izz="${(2/5) * m * r * r}" />
        </inertial>
    </xacro:macro>

</robot>
```
Code : mobile_base.xacro<br>
Credit: https://www.youtube.com/@RoboticsBackEnd
```xml
<?xml version="1.0"?>
<robot xmlns:xacro="http://www.ros.org/wiki/xacro">

    <xacro:property name="base_length" value="0.6" />
    <xacro:property name="base_width" value="0.4" />
    <xacro:property name="base_height" value="0.2" />
    <xacro:property name="wheel_radius" value="0.1" />
    <xacro:property name="wheel_length" value="0.05" />

    <link name="base_footprint" />

    <link name="base_link">
        <visual>
            <geometry>
                <box size="${base_length} ${base_width} ${base_height}" />
            </geometry>
            <origin xyz="0 0 ${base_height / 2.0}" rpy="0 0 0" />
            <material name="blue" />
        </visual>
        <collision>
            <geometry>
                <box size="${base_length} ${base_width} ${base_height}"/>
            </geometry>
            <origin xyz="0 0 ${base_height/2.0}" rpy="0 0 0"/>
        </collision>
        <xacro:box_inertia m="5.0" l="${base_length}" w="${base_width}" h="${base_height}"
                           xyz="0 0 ${base_height / 2.0}" rpy="0 0 0" />
    </link>

    <xacro:macro name="wheel_link" params="prefix">
        <link name="${prefix}_wheel_link">
            <visual>
                <geometry>
                    <cylinder radius="${wheel_radius}" length="${wheel_length}" />
                </geometry>
                <origin xyz="0 0 0" rpy="${pi / 2.0} 0 0" />
                <material name="grey" />
            </visual>
            <collision>
                <geometry>
                    <cylinder radius= "${wheel_radius}" length="${wheel_length}"/>
                 </geometry>
                <origin xyz="0.0 0.0 0.0" rpy="${pi/2.0} 0.0 0.0"/>
            </collision>
            <xacro:cylinder_inertia m="1.0" r="${wheel_radius}" h="${wheel_length}" 
                                    xyz="0 0 0" rpy="${pi / 2.0} 0 0" />
        </link>
    </xacro:macro>

    <xacro:wheel_link prefix="right" />
    <xacro:wheel_link prefix="left" />

    <link name="caster_wheel_link">
        <visual>
            <geometry>
                <sphere radius="${wheel_radius / 2.0}" />
            </geometry>
            <origin xyz="0 0 0" rpy="0 0 0" />
            <material name="grey" />
        </visual>
        <collision>
            <geometry>
                <sphere radius="${wheel_radius/2.0}"/>
            </geometry>
            <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
        </collision>

        <xacro:sphere_inertia m="0.5" r="${wheel_radius / 2.0}"
                              xyz="0 0 0" rpy="0 0 0" />
    </link>

    <joint name="base_joint" type="fixed">
        <parent link="base_footprint" />
        <child link="base_link" />
        <origin xyz="0 0 ${wheel_radius}" rpy="0 0 0"/>
    </joint>

    <joint name="base_right_wheel_joint" type="continuous">
        <parent link="base_link" />
        <child link="right_wheel_link" />
        <origin xyz="${-base_length / 4.0} ${-(base_width + wheel_length) / 2.0} 0" rpy="0 0 0" />
        <axis xyz="0 1 0" />
    </joint>

    <joint name="base_left_wheel_joint" type="continuous">
        <parent link="base_link" />
        <child link="left_wheel_link" />
        <origin xyz="${-base_length / 4.0} ${(base_width + wheel_length) / 2.0} 0" rpy="0 0 0" />
        <axis xyz="0 1 0" />
    </joint>

    <joint name="base_caster_wheel_joint" type="fixed">
        <parent link="base_link" />
        <child link="caster_wheel_link" />
        <origin xyz="${base_length / 3.0} 0 ${-wheel_radius / 2.0}" rpy="0 0 0" />
    </joint>

</robot>
```
## Launch our Robot in Gazebo

Step 1:
```
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro ~/ros2_ws/src/my_robot_description/urdf/my_robot.urdf.xacro)"
```
Step 2: (in another terminal)
```
ros2 launch gazebo_ros gazebo.launch.py
```
Step 3: (in another terminal)
```
ros2 run gazebo_ros spawn_entity.py -topic robot_description -entity my_robot_1
```








