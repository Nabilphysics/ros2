Install a ROS package for URDF Visualization
```
sudo apt install ros-humble-urdf-tutorial
```
To see example URDF, go to this directory to see installed packages
```
cd /opt/ros/humble/share/urdf_tutorial/urdf
```
To see,
```
ros2 launch urdf_tutorial display.launch.py model:=08-macroed.urdf.xacro
```
Red = x axis, Green = Y axis and Blue = z-axis</br>
To see TF Tree,
```
sudo apt install ros-humble-tf2-tools
```
then,
```
ros2 run tf2_tools view_frames
```
it will output a pdf.




