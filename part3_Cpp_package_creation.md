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
```
colcon build
```
or if you want to build a specific package

```
colcon build --packages-select my_cpp_pkg
```
Now, enter this directory
```
/ros2_ws/src/my_cpp_pkg/src
```
make a file name cpp a cpp file
```
touch odometry_publisher.cpp
```
go to /ros2_ws/src/ folder and open vs code here
```
code .
```
paste an example code
```c++
#include "rclcpp/rclcpp.hpp"

class MyNode : public rclcpp::Node
{
public:
    MyNode() : Node("cpp_test"), counter_(0)
    {
        RCLCPP_INFO(this->get_logger(), "Hello Odometry Node");

        timer_ = this->create_wall_timer(std::chrono::seconds(1),
                                         std::bind(&MyNode::timerCallback, this));
    }

private:
    void timerCallback()
    {
        counter_++;
        RCLCPP_INFO(this->get_logger(), "Hello %d", counter_);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    int counter_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MyNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
```
## VS Code Related Fix
You might need to fix the include path</br>
Press Ctrl + Shift + P
Find, C/C++: Edit Configuration(JSON)<>
My file look like this</br>
```json
{
  "configurations": [
    {
      "browse": {
        "databaseFilename": "${default}",
        "limitSymbolsToIncludedHeaders": false
      },
      "includePath": [
        "/opt/ros/humble/include/**",
        "/usr/include/**"
      ],
      "name": "ROS",
      "intelliSenseMode": "gcc-x64",
      "compilerPath": "/usr/bin/gcc",
      "cStandard": "gnu11",
      "cppStandard": "c++14"
    }
  ],
  "version": 4
}
```
Edit the CMakeText.txt File
```cmake
cmake_minimum_required(VERSION 3.8)
project(my_cpp_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_executable(cpp_node src/odometry_publisher.cpp)
ament_target_dependencies(cpp_node rclcpp)

install(TARGETS
  cpp_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

```
Now go to /ros2_ws/ folder and build.
```
colcon build
```
or
```
colcon build colcon build --packages-select my_cpp_pkg
```
then source,
```
source install/setup.bash
```

