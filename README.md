# Creating a world with a custom robot inside

## Step 1. Create empty world and write a launch file
1. Go to catkin source directory and create a new package
```
cd ~/catkin_ws/src && catkin_create_pkg my_package
```
This creates `my_package` folder under `catkin_ws/src`. In `my_package`, you will see `CMakeLists.txt` and `package.xml`.

2. Create new package and necessary directories in `my_package`.
```
cd my_package
```
```
mkdir launch && mkdir worlds && cd worlds
```
3. Create a world file and open it.
```
touch empty.world && gedit empty.world
```
4. Fill the following in 'empty.world'
```
<sdf version="1.4">

  <world name="default">

    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- Light source -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- World camera -->
    <gui fullscreen='0'>
      <camera name='world_camera'>
        <pose>4.927360 -4.376610 3.740080 0.000000 0.275643 2.356190</pose>
        <view_controller>orbit</view_controller>
      </camera>
    </gui>

  </world>
</sdf>
```
5. Go to `launch` folder, create `world.launch` file and open it.
```
cd ../launch && touch world.launch && gedit world.launch
```
6. Fill the following in 'world.launch'
```
<launch>

  <!-- World File -->
  <arg name="world_file" default="$(find my_package)/worlds/empty.world"/>

  <!-- Launch Gazebo World -->
  <include file="$(find gazebo_ros)/launch/empty_world.launch">
    <arg name="use_sim_time" value="true"/>
    <arg name="debug" value="false"/>
    <arg name="gui" value="true" />
    <arg name="world_name" value="$(arg world_file)"/>
  </include>

</launch>
```
7. Go to `catkin_ws` and do `catkin_make`
```
cd ../../.. && catkin_make
```
8. Source `setup.bash`
```
source devel/setup.bash
```
9. Run the launch file
```
roslaunch my_package world.launch
```
Should open an empty world in Gazebo.

## Step 2. Create a custom made robot and edit the launch file
1. Go to `my_package` folder to create `urdf` folder
```
cd ~/catkin_ws/src/my_package && mkdir urdf
```
2. Create xacro file under `urdf` folder
```
cd urdf && touch my_robot.xacro && gedit my_robot.xacro
```
3. PAste the following into xacro file
```
<robot name="my_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">

  <link name="robot_footprint"></link>

  <joint name="robot_footprint_joint" type="fixed">
    <origin xyz="0 0 0" rpy="0 0 0" />
    <parent link="robot_footprint"/>
    <child link="chassis" />
  </joint>

  <link name='chassis'>
    <pose>0 0 0.1 0 0 0</pose>

    <inertial>
      <mass value="15.0"/>
      <origin xyz="0.0 0 0" rpy=" 0 0 0"/>
      <inertia
          ixx="0.1" ixy="0" ixz="0"
          iyy="0.1" iyz="0"
          izz="0.1"
      />
    </inertial>

    <collision name='collision'>
      <origin xyz="0 0 0" rpy=" 0 0 0"/> 
      <geometry>
        <box size=".4 .2 .1"/>
      </geometry>
    </collision>

    <visual name='chassis_visual'>
      <origin xyz="0 0 0" rpy=" 0 0 0"/>
      <geometry>
        <box size=".4 .2 .1"/>
      </geometry>
    </visual>


    <collision name='back_caster_collision'>
      <origin xyz="-0.15 0 -0.05" rpy=" 0 0 0"/>
      <geometry>
        <sphere radius="0.0499"/>
      </geometry>
    </collision>

    <visual name='back_caster_visual'>
      <origin xyz="-0.15 0 -0.05" rpy=" 0 0 0"/>
      <geometry>
        <sphere radius="0.05"/>
      </geometry>
    </visual>

    <collision name='front_caster_collision'>
      <origin xyz="0.15 0 -0.05" rpy=" 0 0 0"/>
      <geometry>
        <sphere radius="0.0499"/>
      </geometry>
    </collision>

    <visual name='front_caster_visual'>
      <origin xyz="0.15 0 -0.05" rpy=" 0 0 0"/>
      <geometry>
        <sphere radius="0.05"/>
      </geometry>
    </visual>

  </link>

</robot>
```
5.

Download [hokuyo.dae](https://s3-us-west-1.amazonaws.com/udacity-robotics/hokuyo.dae) and put it in 
