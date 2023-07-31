### How Robot State Publisher and URDF Work Together
Launch our previous URDF File using 
```
ros2 launch urdf_tutorial display.launch.py model:=my_robot.urdf
```
We already know that, to output tf tree as pdf,
```
ros2 run tf2_tools view_frames -o my_robot_frames
```
my_robot_frames = output file name
Output Tree is,
![alt text](https://github.com/Nabilphysics/ros2/blob/main/images/my_robot_frame_tf.png)
