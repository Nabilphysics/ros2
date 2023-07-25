Download an example urdf file from https://github.com/Nabilphysics/ros2/blob/main/files/my_robot.urdf
You may make a file name my_robot.urdf and copy-paste the content from the above link. This file is from a youtube channel "robotics backend"</br>
To easily view the urdf file we can download a package. 
```
sudo apt install ros-humble-urdf-tutorial
```
Now, go to the my_robot.urdf directory and apply the below command
```
ros2 launch urdf_tutorial display.launch.py model:=my_robot.urdf
```
To see turtlebot3 urdf file like this go to the turtlebot3 folder
```
/opt/ros/humble/share/turtlebot3_description/urdf
```
then,
```
ros2 launch urdf_tutorial display.launch.py model:=turtlebot3_waffle.urdf
```


