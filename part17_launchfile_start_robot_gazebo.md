## Make a Launch file to start robot in gazebo
```
cd ros2_ws/src
ros2 pkg create my_robot_bringup
```
Delete include and src folder and create launch folder inside my_robot_bringup folder</br>
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
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)


ament_package()
```





