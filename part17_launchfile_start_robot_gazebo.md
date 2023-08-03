## Make a Launch file to start robot in gazebo
```
cd ros2_ws/src
ros2 pkg create my_robot_bringup
```
### Delete the include and src folder, and create a launch and rviz folder inside my_robot_bringup folder</br>
Now edit the CMakeLists.txt file. So the file will look like this,
```cmake
cmake_minimum_required(VERSION 3.8)
project(my_robot_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
install(
  DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
```
### Launch Folder
Create my_robot_gazebo.launch.xml file
```xml
<launch>
    <let name="urdf_path" 
         value="$(find-pkg-share my_robot_description)/urdf/my_robot.urdf.xacro" />

    <let name="rviz_config_path"
         value="$(find-pkg-share my_robot_bringup)/rviz/urdf_config.rviz" />     

    <node pkg="robot_state_publisher" exec="robot_state_publisher">
        <param name="robot_description" value="$(command 'xacro $(var urdf_path)')" />
    </node>

    <include file="$(find-pkg-share gazebo_ros)/launch/gazebo.launch.py"/>

    <node pkg="gazebo_ros" exec="spawn_entity.py"
          args="-topic robot_description -entity my_robot"/>


    <node pkg="rviz2" exec="rviz2" output="screen" args="-d $(var rviz_config_path)" />

</launch>
```
### rviz folder
Copy urdf_config.rviz file from my_robot_description folder to this folder

### package.xml file in my_robot_bringup folder should be 
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>my_robot_bringup</name>
  <version>0.0.0</version>
  <description>TODO: Package description</description>
  <maintainer email="nabilsust@gmail.com">nabil</maintainer>
  <license>TODO: License declaration</license>

  <buildtool_depend>ament_cmake</buildtool_depend>

  <exec_depend>my_robot_description</exec_depend>
  <exec_depend>robot_state_publisher</exec_depend>
  <exec_depend>gazebo_ros</exec_depend>

  <test_depend>ament_lint_auto</test_depend>
  <test_depend>ament_lint_common</test_depend>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```
## Edit URDF to bring color to Gazebo and fix the inertia
Step 1: Create a file name mobile_base_gazebo.xacro in urdf folder of my_robot_description folder and add gazebo color code</br>
Code: mobile_base_gazebo.xacro
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
    </gazebo>

</robot>

```
Step 2: Edit my_robot.urdf.xacro file and add include "mobile_base_gazebo.xacro"</br>
Code: my_robot.urdf.xacro
```xml
<?xml version="1.0"?>
<robot name="my_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">

    <xacro:include filename="common_properties.xacro"/>
    <xacro:include filename="mobile_base.xacro"/>
    <xacro:include filename="mobile_base_gazebo.xacro"/>
</robot>
```
Step 3: Edit mobile_base.xacro file's inertia property for box_inertia, cylinder_inertia and sphere_inertia by multiplying 2
Code: mobile_base.xacro
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
        <xacro:box_inertia m="5.0" l="${2*base_length}" w="${2*base_width}" h="${2*base_height}"
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
            <xacro:cylinder_inertia m="1.0" r="${2*wheel_radius}" h="${2*wheel_length}" 
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

        <xacro:sphere_inertia m="0.5" r="${2*wheel_radius / 2.0}" 
                              xyz="0 0 0" rpy="0 0 0" />
                              <!-- Multiplied by 2 to increase inertia --> 
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

Step 4: Colcon build and launch
```
colcon build --symlink-install
source install/setup.bash
ros2 launch my_robot_bringup my_robot_gazebo.launch.xml
```














