## ROS2 C++ Package Creation
Go to directory 
```
cd ~/ros2_ws/src
```
```
ros2 pkg create my_cpp_pkg --build-type ament_cmake --dependencies rclcpp
```
to build
```
~/ros2_ws/
```
colcon build
```
or if you want to build a specific package
```
colcon build --packages-select my_cpp_pkg
```

