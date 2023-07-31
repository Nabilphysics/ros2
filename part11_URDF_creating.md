## Create a URDF file
```
touch my_robot.urdf
```
Open with Visual Studio Code
```
code my_robot.urdf
```
Install a Visual Studio Extension named "URDF" by smileroboitcs. It will help us to create URDF.</br>
Write this code,
```xml
<?xml version="1.0"?>
<robot name="my_robot">
    <link name="base_link">
        <visual>
            <geometry>
                <box size="0.6 0.4 0.2" />
            </geometry>
            <origin xyz="0 0 0" rpy="0 0 0" />
        </visual>
    </link>
</robot>
```
To test this ,
```
ros2 launch urdf_tutorial display.launch.py model:=my_robot.urdf
```
```xml
<?xml version="1.0"?>
<robot name="my_robot">

    <material name="green">
        <color rgba="0.0 0.5 0.0 1.0"/>    
    </material>
    <material name="blue">
        <color rgba="0.0 0.0 0.5 1.0"/>    
    </material>
    <material name="grey">
        <color rgba="0.0 0.5 0.5 1.0"/>    
    </material>

    <link name="base_link">
        <visual>
            <geometry>
                <box size="0.6 0.4 0.2" />
            </geometry>
            <origin xyz="0 0 0.1" rpy="0 0 0" />
            <material name="blue" />
        </visual>
    </link>

    <link name="second_link">
        <visual>
          <geometry>
            <cylinder radius="0.1" length="0.2"/>
          </geometry> 
          <origin xyz="0.0 0.0 0.1" rpy="0.0 0.0 0.0"/> 
          <material name="grey"/>    
        </visual>
    </link>

    <joint name="base_second_joint" type="fixed">
        <parent link="base_link"/>
        <child link="second_link"/>
        <origin xyz="0.0 0.0 0.2" rpy="0.0 0.0 0.0" />
        
    </joint>
    

</robot>
```
Good practice is first provide 0,0,0 for everything except baselink and then start adjusting. </br>
First fixed the TF then visual.<br>
Lets add another link,
```xml
<?xml version="1.0"?>
<robot name="my_robot">

    <material name="green">
        <color rgba="0.0 0.5 0.0 1.0"/>    
    </material>
    <material name="blue">
        <color rgba="0.0 0.0 0.5 1.0"/>    
    </material>
    <material name="grey">
        <color rgba="0.0 0.5 0.5 1.0"/>    
    </material>

    <link name="base_link">
        <visual>
            <geometry>
                <box size="0.6 0.4 0.2" />
            </geometry>
            <origin xyz="0 0 0.1" rpy="0 0 0" />
            <material name="blue" />
        </visual>
    </link>

    <link name="second_link">
        <visual>
          <geometry>
            <cylinder radius="0.1" length="0.2"/>
          </geometry> 
          <origin xyz="0.0 0.0 0.1" rpy="0.0 0.0 0.0"/> 
          <material name="grey"/>    
        </visual>
    </link>

     <link name="third_link">
        <visual>
          <geometry>
             <box size="0.1 0.1 0.1" />
          </geometry> 
          <origin xyz="0.0 0.0 0.05" rpy="0.0 0.0 0.0"/> 
          <material name="green"/>    
        </visual>
    </link>

    <joint name="base_second_joint" type="fixed">
        <parent link="base_link"/>
        <child link="second_link"/>
        <origin xyz="0.0 0.0 0.2" rpy="0.0 0.0 0.0" />
        
    </joint>

    <joint name="second_third_joint" type="fixed">
        <parent link="second_link"/>
        <child link="third_link"/>
        <origin xyz="0.0 0.0 0.2" rpy="0.0 0.0 0.0" />
        
    </joint>
    

</robot>
```
Good Resource : http://wiki.ros.org/urdf/XML<br>
To add revolution in joint
```xml
<joint name="base_second_joint" type="revolute">
        <parent link="base_link"/>
        <child link="second_link"/>
        <origin xyz="0.0 0.0 0.2" rpy="0.0 0.0 0.0" />
        <axis xyz="0.0 0.0 1.0"/>
        <limit lower="-1.57" upper="1.57" effort="100.0" velocity="100.0"/>
        
    </joint>
```
Full Code
```xml
<?xml version="1.0"?>
<robot name="my_robot">

    <material name="green">
        <color rgba="0.0 0.5 0.0 1.0"/>    
    </material>
    <material name="blue">
        <color rgba="0.0 0.0 0.5 1.0"/>    
    </material>
    <material name="grey">
        <color rgba="0.0 0.5 0.5 1.0"/>    
    </material>

    <link name="base_link">
        <visual>
            <geometry>
                <box size="0.6 0.4 0.2" />
            </geometry>
            <origin xyz="0 0 0.1" rpy="0 0 0" />
            <material name="blue" />
        </visual>
    </link>

    <link name="second_link">
        <visual>
          <geometry>
            <cylinder radius="0.1" length="0.2"/>
          </geometry> 
          <origin xyz="0.0 0.0 0.1" rpy="0.0 0.0 0.0"/> 
          <material name="grey"/>    
        </visual>
    </link>

 

    <joint name="base_second_joint" type="revolute">
        <parent link="base_link"/>
        <child link="second_link"/>
        <origin xyz="0.0 0.0 0.2" rpy="0.0 0.0 0.0" />
        <axis xyz="0.0 0.0 1.0"/>
        <limit lower="-1.57" upper="1.57" effort="100.0" velocity="100.0"/>
        
    </joint> 

</robot>
```
For continuous rotation
```xml
 <joint name="base_second_joint" type="continuous">
        <parent link="base_link"/>
        <child link="second_link"/>
        <origin xyz="0.0 0.0 0.2" rpy="0.0 0.0 0.0" />
        <axis xyz="0.0 0.0 1.0"/>
 </joint>
```
For sliding,
```xml
<joint name="base_second_joint" type="prismatic">
        <parent link="base_link"/>
        <child link="second_link"/>
        <origin xyz="0.0 0.0 0.2" rpy="0.0 0.0 0.0" />
        <axis xyz="1.0 0.0 0.0"/>
        <limit lower="0.0" upper="0.3" effort="100.0" velocity="100.0"/>
</joint>
```
To mode in 3D
```xml
  <joint name="base_second_joint" type="prismatic">
        <parent link="base_link"/>
        <child link="second_link"/>
        <origin xyz="0.0 0.0 0.2" rpy="0.0 0.0 0.0" />
        <axis xyz="1.0 1.0 1.0"/>
        <limit lower="0.0" upper="0.3" effort="100.0" velocity="100.0"/>
        
    </joint>
```
To move it diagonally,
```xml
  <joint name="base_second_joint" type="prismatic">
        <parent link="base_link"/>
        <child link="second_link"/>
        <origin xyz="0.0 0.0 0.2" rpy="0.0 0.0 0.0" />
        <axis xyz="1.0 1.0 0.0"/>
        <limit lower="0.0" upper="0.3" effort="100.0" velocity="100.0"/>
        
    </joint>
```
## Complete URDF for a two wheel Robot with Ball Caster
```xml
<?xml version="1.0"?>
<robot name="my_robot">

    <material name="green">
        <color rgba="0.0 0.5 0.0 1.0"/>    
    </material>
    <material name="blue">
        <color rgba="0.0 0.0 0.5 1.0"/>    
    </material>
    <material name="grey">
        <color rgba="0.0 0.5 0.5 1.0"/>    
    </material>
    
    <link name="base_footprint">
       
    </link>

    <link name="base_link">
        <visual>
            <geometry>
                <box size="0.6 0.4 0.2" />
            </geometry>
            <origin xyz="0 0 0.1" rpy="0 0 0" />
            <material name="blue" />
        </visual>
    </link>

    <link name="right_wheel_link">
        <visual>
            <geometry>
                <cylinder radius="0.1" length="0.05"/>
            </geometry>
            <origin xyz="0.0 0.0 0.0" rpy="1.57 0.0 0.0"/>
            <material name="grey"/>
        </visual>
    </link>
    <link name="left_wheel_link">
        <visual>
            <geometry>
                <cylinder radius="0.1" length="0.05"/>
            </geometry>
            <origin xyz="0.0 0.0 0.0" rpy="1.57 0.0 0.0"/>
            <material name="grey"/>
        </visual>
    </link>
     <link name="caster_wheel_link">
        <visual>
            <geometry>
                <sphere radius="0.05"/>
            </geometry>
            <origin xyz="0.0 0.0 0.0" rpy="0.0 0.0 0.0"/>
            <material name="grey"/>
        </visual>
    </link>


    <joint name="base_joint" type="fixed">
        <origin xyz="0.0 0.0 0.1" rpy="0 0 0"/>
        <parent link="base_footprint"/>
        <child link="base_link"/>
       
    </joint>

    <joint name="base_right_wheel_joint" type="continuous">
        <origin xyz="-0.15 -0.225 0.0" rpy="0 0 0"/>
        <parent link="base_link"/>
        <child link="right_wheel_link"/>
        <axis xyz="0.0 1.0 0.0"/>  
    </joint>
    <joint name="base_left_wheel_joint" type="continuous">
        <origin xyz="-0.15 0.225 0.0" rpy="0 0 0"/>
        <parent link="base_link"/>
        <child link="left_wheel_link"/>
        <axis xyz="0.0 1.0 0.0"/>  
    </joint>
    <joint name="base_caster_wheel_joint" type="fixed">
        <origin xyz="0.2 0.0 -0.05" rpy="0.0 0.0 0.0"/>
        <parent link="base_link"/>
        <child link="caster_wheel_link"/>  
    </joint>

</robot>
```
Run it,
```
ros2 launch urdf_tutorial display.launch.py model:=my_robot.urdf
```
Final Result,
![alt text](https://github.com/Nabilphysics/ros2/blob/main/images/urdf_two_wheel_robot.png)
In RViz select Global Options > Fixed Frame > base_footprint

