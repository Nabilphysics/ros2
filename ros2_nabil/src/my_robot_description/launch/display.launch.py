from launch import LaunchDescription
import os
from ament_index_python.packages import get_package_share_path
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from launch_ros.actions import Node

def generate_launch_description():
    
    
    # find the path of my_robot.urdf where it has installed
    urdf_path = os.path.join(get_package_share_path('my_robot_description'), 'urdf', 'my_robot.urdf.xacro')
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