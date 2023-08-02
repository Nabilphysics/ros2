## Install xacro
```
sudo apt install ros-<distro>-xacro
```
## Rewrite our URDF file with XACRO
Step 1: rename "my_robot.urdf" to "my_robot.urdf.xacro" </br>
Step 2: rename like this inside "display.launch.py" and "display.launch.xml"</br>
Step 3: modify main code of my_robot.urdf </br>
Code:
```xml
<?xml version="1.0"?>
<robot name="my_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">

    <xacro:property name="base_length" value="0.6" />
    <xacro:property name="base_width" value="0.4" />
    <xacro:property name="base_height" value="0.2" />
    <xacro:property name="wheel_radius" value="0.1" />
    <xacro:property name="wheel_length" value="0.05" />

    <material name="green">
        <color rgba="0.0 0.5 0.0 1.0"/>    
    </material>
    <material name="blue">
        <color rgba="0.0 0.0 0.5 1.0"/>    
    </material>
    <material name="grey">
        <color rgba="0.0 0.5 0.5 1.0"/>    
    </material>
    
    <link name="base_footprint">
       
    </link>

    <link name="base_link">
        <visual>
            <geometry>
                <box size="${base_length} ${base_width} ${base_height}"/>
            </geometry>
            <origin xyz="0 0 ${base_height/2.0}" rpy="0 0 0"/>
            <material name="blue"/>
        </visual>
    </link>

    <link name="right_wheel_link">
        <visual>
            <geometry>
                <cylinder radius= "${wheel_radius}" length="${wheel_length}"/>
            </geometry>
            <origin xyz="0.0 0.0 0.0" rpy="${pi/2.0} 0.0 0.0"/>
            <material name="grey"/>
        </visual>
    </link>
    <link name="left_wheel_link">
        <visual>
            <geometry>
                <cylinder radius="${wheel_radius}" length="${wheel_length}"/>
            </geometry>
            <origin xyz="0.0 0.0 0.0" rpy="${pi/2.0} 0.0 0.0"/>
            <material name="grey"/>
        </visual>
    </link>
     <link name="caster_wheel_link">
        <visual>
            <geometry>
                <sphere radius="${wheel_radius/2.0}"/>
            </geometry>
            <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
            <material name="grey"/>
        </visual>
    </link>


    <joint name="base_joint" type="fixed">
        <origin xyz="0.0 0.0 ${wheel_radius}" rpy="0 0 0"/>
        <parent link="base_footprint"/>
        <child link="base_link"/>
       
    </joint>

    <joint name="base_right_wheel_joint" type="continuous">
        <origin xyz="${-base_length / 4.0} ${-(base_width + wheel_length)/2.0} 0.0" rpy="0 0 0"/>
        <parent link="base_link"/>
        <child link="right_wheel_link"/>
        <axis xyz="0.0 1.0 0.0"/>  
    </joint>
    <joint name="base_left_wheel_joint" type="continuous">
        <origin xyz="${-base_length / 4.0} ${(base_width + wheel_length)/2.0} 0.0" rpy="0 0 0"/>
        <parent link="base_link"/>
        <child link="left_wheel_link"/>
        <axis xyz="0.0 1.0 0.0"/>  
    </joint>
    <joint name="base_caster_wheel_joint" type="fixed">
        <origin xyz="${base_length/3} 0.0 ${-wheel_radius/2.0}" rpy="0.0 0.0 0.0"/>
        <parent link="base_link"/>
        <child link="caster_wheel_link"/>  
    </joint>

</robot>
```
## Introducing XACRO MACRO in our code
```xml
<?xml version="1.0"?>
<robot name="my_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">

    <xacro:property name="base_length" value="0.6" />
    <xacro:property name="base_width" value="0.4" />
    <xacro:property name="base_height" value="0.2" />
    <xacro:property name="wheel_radius" value="0.1" />
    <xacro:property name="wheel_length" value="0.05" />


   

    <material name="green">
        <color rgba="0.0 0.5 0.0 1.0"/>    
    </material>
    <material name="blue">
        <color rgba="0.0 0.0 0.5 1.0"/>    
    </material>
    <material name="grey">
        <color rgba="0.0 0.5 0.5 1.0"/>    
    </material>
    
    <link name="base_footprint">
       
    </link>

    <link name="base_link">
        <visual>
            <geometry>
                <box size="${base_length} ${base_width} ${base_height}"/>
            </geometry>
            <origin xyz="0 0 ${base_height/2.0}" rpy="0 0 0"/>
            <material name="blue"/>
        </visual>
    </link>

    <xacro:macro name="wheel_link" params="prefix">
        <link name="${prefix}_wheel_link">
            <visual>
                <geometry>
                  <cylinder radius= "${wheel_radius}" length="${wheel_length}"/>
                 </geometry>
                <origin xyz="0.0 0.0 0.0" rpy="${pi/2.0} 0.0 0.0"/>
                <material name="grey"/>
            </visual>
        </link>
    </xacro:macro>

    <xacro:wheel_link prefix="right"/>
    <xacro:wheel_link prefix="left"/>
    
    
     <link name="caster_wheel_link">
        <visual>
            <geometry>
                <sphere radius="${wheel_radius/2.0}"/>
            </geometry>
            <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
            <material name="grey"/>
        </visual>
    </link>


    <joint name="base_joint" type="fixed">
        <origin xyz="0.0 0.0 ${wheel_radius}" rpy="0 0 0"/>
        <parent link="base_footprint"/>
        <child link="base_link"/>
       
    </joint>

    <joint name="base_right_wheel_joint" type="continuous">
        <origin xyz="${-base_length / 4.0} ${-(base_width + wheel_length)/2.0} 0.0" rpy="0 0 0"/>
        <parent link="base_link"/>
        <child link="right_wheel_link"/>
        <axis xyz="0.0 1.0 0.0"/>  
    </joint>
    <joint name="base_left_wheel_joint" type="continuous">
        <origin xyz="${-base_length / 4.0} ${(base_width + wheel_length)/2.0} 0.0" rpy="0 0 0"/>
        <parent link="base_link"/>
        <child link="left_wheel_link"/>
        <axis xyz="0.0 1.0 0.0"/>  
    </joint>
    <joint name="base_caster_wheel_joint" type="fixed">
        <origin xyz="${base_length/3} 0.0 ${-wheel_radius/2.0}" rpy="0.0 0.0 0.0"/>
        <parent link="base_link"/>
        <child link="caster_wheel_link"/>  
    </joint>

</robot>
```
