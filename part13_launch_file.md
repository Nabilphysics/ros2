## Write a launch file to start the Robot State Publisher with URDF
create a launch folder/directory inside my_robot_description. Also, add rviz and urdf folder. Save rviz config file inside rviz directory as "urdf_config.rviz" </br>
add launch in our CMakeLists.txt
```cmake
cmake_minimum_required(VERSION 3.8)
project(my_robot_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf launch rviz
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()

```
### XML Launch File
Create a file inside the launch folder name "display.launch.xml"
```xml
<launch>
    <let name="urdf_path" 
         value="$(find-pkg-share my_robot_description)/urdf/my_robot.urdf" />
    <let name="rviz_config_path"
         value="$(find-pkg-share my_robot_description)/rviz/urdf_config.rviz" />

   
    <node pkg="robot_state_publisher" exec="robot_state_publisher">
        <param name="robot_description" value="$(command 'xacro $(var urdf_path)')" />
    </node>
    
    <node pkg="joint_state_publisher_gui" exec="joint_state_publisher_gui">
    </node>

    <node pkg="rviz2" exec="rviz2" output="screen" args="-d $(var rviz_config_path)" />
  
</launch>
 
 <?ignore
    Reference for understanding:
    ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro my_robot.urdf)"
    ros2 run joint_state_publisher_gui joint_state_publisher_gui
    ros2 run rviz2 rviz2
?>
```
build using colcon build</br>
run
```
ros2 launch my_robot_description display.launch.xml
```
### Python Launch File
inside launch folder create a file name display.launch.py
```python
from launch import LaunchDescription
import os
from ament_index_python.packages import get_package_share_path
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from launch_ros.actions import Node

def generate_launch_description():
    
    
    # find the path of my_robot.urdf where it has installed
    urdf_path = os.path.join(get_package_share_path('my_robot_description'), 'urdf', 'my_robot.urdf')
    # rviz config path
    rviz_config_path = os.path.join(get_package_share_path('my_robot_description'), 'rviz', 'urdf_config.rviz')

    #parameter value (will be used in node)
    robot_description = ParameterValue(Command(['xacro ', urdf_path]), value_type=str)

    #Start Node robot_state_publisher
    #Reference: # Reference: ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro my_robot.urdf)"
    robot_state_publisher_node = Node(
        package="robot_state_publisher", 
        executable="robot_state_publisher",
        parameters=[{'robot_description': robot_description}]
        )

    #Start Node robot_state_publisher
    #Reference: ros2 run joint_state_publisher_gui joint_state_publisher_gui
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui"
    )
    
    #Start rviz node
    #Reference: ros2 run rviz2 rviz2
    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', rviz_config_path]
        
    )
    
    
    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz2_node
    ])
```





