Learning Source:</br>
https://youtu.be/idQb2pB-h2Q

```
sudo apt update
```
```
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup ros-humble-turtlebot3*
```
```
gedit ~/.bashrc
```
Add
```
export TURTLEBOT3_MODEL=waffle
```
before the line source /opt/ros/humble/setup.bash

To see
```
printenv | grep TURTLE
```
Test gazebo 
```
gazebo
```
```
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```
To see in rviz
```
ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True
```
To drive the robot start a keyboard
```
ros2 run turtlebot3_teleop teleop_keyboard
```
To save the map
```
ros2 run nav2_map_server map_saver_cli -f maps/my_map
```
To start a new world that is different from previous
```
ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
```
## Install Cyclone DDS
```
sudo apt install ros-humble-rmw-cyclonedds-cpp
```
add to bashrc
```
gedit ~/.bashrc
```
add the below line just before "source /opt/ros/humble/setup.bash" 
```
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
```
So the bottom part of our bashrc file now looks like
```
export TURTLEBOT3_MODEL=waffle
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
source /opt/ros/humble/setup.bash
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
source ~/ros2_ws/install/setup.bash
```
Now we will edit some parameters,
```
cd /opt/ros/humble/share/turtlebot3_navigation2/param
sudo gedit waffle.yaml
```
Now change robot_model_type: "differential" to
```
robot_model_type: "nav2_amcl::DifferentialMotionModel"
```


## Starting navigation
```
ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True map:=maps/my_map.yaml
```
## Global Planner, Local Planner, Costmaps
will be added
## Recovery Behavior
will be added
## TF
To see TF topic
```
ros2 topic list
```
```
ros2 topic echo /tf
```
to export TF tree as pdf(it will listen to tf topics for 5 second and then it will export as pdf)
```
ros2 run tf2_tools view_frames 
```

## Building World in Gazebo, Make Turtlebot Navigate in World
We will start navigating Turtlebot in our own world. After making a world in gazebo we will clone turtlebot repository in our machine and we will include our world in this repository</br>
### Step 1: Make a workspace name turtlebot3_ws
```
mkdir turtlebot3_ws
```
```
cd turtlebot3_ws
mkdir src
cd src
```
### Step 2: clone the repository
```
git clone https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git .

```
. means clone here without making any directory. Now switch to humble-devel branch(according to your ros version)
```
git checkout humble-devel
```
### Step 3: Build the repository using colcon and source to the bashrc 
```
colcon build
```
source this workspace to bashrc
```
gedit ~/.bashrc 
```
and add
```
source ~/turtlebot3_ws/install/setup.bash
```
this line at the end of the file.</br>

### Step 4: Add our world to our cloned repository and edit the launch file to drive the turtblebot to our own world.
Cut and paste our newely created "my_world.world" file to cd ~/turtlebot3_ws/src/turtlebot3_gazebo/worlds

### Step 5: Edit the launch file with our world
go to 
```
cd ~/turtlebot3_ws/src/turtlebot3_gazebo/worlds
```
```
cp turtlebot3_house.launch.py turtlebot3_my_world.launch.py
```
Now open turtlebot3_my_world.launch.py and fine the below line and replace with our own world file
```
  world = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'worlds',
        'my_world.world'
    )
```













