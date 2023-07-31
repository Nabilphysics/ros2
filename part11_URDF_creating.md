Create a urdf file
```
touch my_robot.urdf
```
Open with Visual Studio Code
```
code my_robot.urdf
```
Install a Visual Studio Extension named "URDF" by smileroboitcs. It will help us to create URDF.</br>
Write this code,
```
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
```
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


