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
3. Paste the following into xacro file
```<robot name="my_robot" xmlns:xacro="http://www.ros.org/wiki/xacro">
 <link name="right_wheel">
<collision>
	<origin xyz="0 0 0" rpy="0 1.5707 1.5707" />
	<geometry>
    		 <cylinder radius="0.1" length="0.05"/>
  	</geometry>
</collision>
<inertial>
	<mass value="5"/>
	<origin xyz="0 0 0" rpy="0 1.5707 1.5707" />

	<inertia
          ixx="0.1" ixy="0" ixz="0"
          iyy="0.1" iyz="0"
          izz="0.1"
        />

</inertial>
<visual>
	<origin xyz="0 0 0" rpy="0 1.5707 1.5707" />
 	<geometry>
     		<cylinder radius="0.1" length="0.05"/>
  	</geometry>
</visual>
</link>
 <link name="left_wheel">
<collision>
	<origin xyz="0 0 0" rpy="0 1.5707 1.5707" />
	<geometry>
    		 <cylinder radius="0.1" length="0.05"/>
  	</geometry>
</collision>
<inertial>
	<mass value="5"/>
	<origin xyz="0 0 0" rpy="0 1.5707 1.5707" />
	<inertia
          ixx="0.1" ixy="0" ixz="0"
          iyy="0.1" iyz="0"
          izz="0.1"
        />

</inertial>
<visual>
	<origin xyz="0 0 0" rpy="0 1.5707 1.5707" />
 	<geometry>
     		<cylinder radius="0.1" length="0.05"/>
	</geometry>
</visual>
</link>
  <joint type="continuous" name="left_wheel_hinge">
    <origin xyz="0 0.15 0" rpy="0 0 0"/>
    <child link="left_wheel"/>
    <parent link="chassis"/>
    <axis xyz="0 1 0" rpy="0 0 0"/>
    <limit effort="10000" velocity="1000"/>
    <dynamics damping="1.0" friction="1.0"/>
  </joint>
  <joint type="continuous" name="right_wheel_hinge">
    <origin xyz="0 -0.15 0" rpy="0 0 0"/>
    <child link="right_wheel"/>
    <parent link="chassis"/>
    <axis xyz="0 1 0" rpy="0 0 0"/>
    <limit effort="10000" velocity="1000"/>
    <dynamics damping="1.0" friction="1.0"/>
  </joint>
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
4. Go to `launch` folder, create robot_description.launch and open it.
```
cd ../launch && touch robot_description.launch && gedit robot_description.launch 
```
5.Paste the following into robot_description.launch.
```
<launch>

  <!-- send urdf to param server -->
  <param name="robot_description" command="$(find xacro)/xacro --inorder '$(find my_package)/urdf/my_robot.xacro'" />

</launch>
```
6. Open world.launch and update it to employ the robot.
```
gedit world.launch
```
7. Add the following after the `<launch>`.
```
  <!-- Robot pose -->
  <arg name="x" default="0"/>
  <arg name="y" default="0"/>
  <arg name="z" default="0"/>
  <arg name="roll" default="0"/>
  <arg name="pitch" default="0"/>
  <arg name="yaw" default="0"/>

  <!-- Launch other relevant files-->
  <include file="$(find my_package)/launch/robot_description.launch"/>
```
8. Add the following before the `</launch>`.
```
<!-- Find my robot Description-->
  <param name="robot_description" command="$(find xacro)/xacro --inorder '$(find my_package)/urdf/my_robot.xacro'"/>

  <!-- Spawn My Robot -->
  <node name="urdf_spawner" pkg="gazebo_ros" type="spawn_model" respawn="false" output="screen" 
        args="-urdf -param robot_description -model my_robot 
              -x $(arg x) -y $(arg y) -z $(arg z)
              -R $(arg roll) -P $(arg pitch) -Y $(arg yaw)"/>
```
9. Go to `catkin_ws`, do `catkin_make`, then `source devel/setup.bash`.
```
cd ../../.. && catkin_make && source devel/setup.bash
```
10. Run the launch file.
```
roslaunch my_package world.launch
```
## Step 3. Add right and left wheels

--to be filled later

## Step 4. Add camera and Lidar
We should add both link and a joint for Camera and Lidar.

### Step 4.1. Add Camera
1. Open `my_robot.xacro`
```
cd ~/catkin_ws/src/my_package/urdf && gedit my_robot.xacro
```
2. Add the following link  and joint description to `my_robot.xacro`.
```
    <link name="camera">
    <collision>
      <geometry>
        <box size="0.05 0.05 0.05"/>
      </geometry>
    <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="0.1"/>
      <inertia ixx="1e-6" ixy="0" ixz="0" iyy="1e-6" iyz="0" izz="1e-6"/>
    </collision>
    <inertial>
      <geometry>
        <box size="0.05 0.05 0.05"/>
      </geometry>
    <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="0.1"/>
      <inertia ixx="1e-6" ixy="0" ixz="0" iyy="1e-6" iyz="0" izz="1e-6"/>
    </inertial>
    <visual>
      <geometry>
        <box size="0.05 0.05 0.05"/>
      </geometry>
    <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="0.1"/>
      <inertia ixx="1e-6" ixy="0" ixz="0" iyy="1e-6" iyz="0" izz="1e-6"/>
    </visual>
    <box_inertia sizeX="0.05" sizeY="0.05" sizeZ="0.05" mass="0.1">
      <origin xyz="0 0 0" rpy="0 0 0"/>
    </box_inertia>
  </link>

  <joint type="fixed" name="camera_joint">
    <origin xyz="0.2 0 0" rpy="0 0 0"/>
    <child link="camera"/>
    <parent link="chassis"/>
    <axis xyz="0 1 0" rpy="0 0 0"/>
  </joint>
```
### Step 4.1. Add Lidar
1. Create meshes directory and mesh file in it
```
cd .. && mkdir meshes && cd meshes
```
2. Download [hokuyo.dae](https://s3-us-west-1.amazonaws.com/udacity-robotics/hokuyo.dae) and put it in `meshes` directory.

3. Go to urdf to open `my_robot.xacro`.
```
cd ../urdf && gedit my_robot.xacro
```
4. Add the following into `my_robot.xacro` file.
```
  <link name="hokuyo">
    <collision>
      <geometry>
        <box size="0.1 0.1 0.1"/>
      </geometry>
    <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="1e-5"/>
      <inertia ixx="1e-6" ixy="0" ixz="0" iyy="1e-6" iyz="0" izz="1e-6"/>
    </collision>
    <inertial>
      <geometry>
        <box size="0.6 0.1 0.2"/>
      </geometry>
    <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="1e-5"/>
      <inertia ixx="1e-6" ixy="0" ixz="0" iyy="1e-6" iyz="0" izz="1e-6"/>
    </inertial>
    <visual>
      <geometry>
        <mesh filename="package://my_package/meshes/hokuyo.dae"/>
      </geometry>
    <origin xyz="0 0 0" rpy="0 0 0"/>
      <mass value="1e-5"/>
      <inertia ixx="1e-6" ixy="0" ixz="0" iyy="1e-6" iyz="0" izz="1e-6"/>
    </visual>
  </link>

  <joint type="fixed" name="hokuyo_joint">
    <origin xyz="0.15 0 0.1" rpy="0 0 0"/>
    <child link="hokuyo"/>
    <parent link="chassis"/>
    <axis xyz="0 1 0" rpy="0 0 0"/>
  </joint>
```
5. Go under `catkin_ws` for `catkin_make` and sourcing.
```
cd ../../.. && catkin_make && source devel/setup.bash
```
6. Roslaunch
```
roslaunch my_package world.launch
```
## Step 5. Gazebo Plugins
URDF in itself can't help with takes those images in simulation? How does a lidar sensor take laser measurements in simulation? How exactly does your robot move in a simulated environment?..But Gazebo allows for plugins that implement specific use-cases.
We will cover the use of three such plugins:

A plugin for the camera sensor.
A plugin for the Hokuyo lidar sensor.
A plugin for the wheel joints actuator.


1. Download the [my_robot.gazebo](https://s3-us-west-1.amazonaws.com/udacity-robotics/my_robot.gazebo) file. Check lines 14 49 93 for topic names.

2. Put it under `my_package/urdf` directory. (Replace <full_path_to_my_robot.gazebo> with the actual path)
```
cd ~/catkin_ws/src/my_package/urdf && cp <full_path_to_my_robot.gazebo>/my_robot.gazebo .
```
   
3. Add the following line to `my_robot.xacro` immediately before `robot_footprint` link.
```
 <xacro:include filename="$(find my_package)/urdf/my_robot.gazebo" />
```
## Step 6. Rviz Integration
1. Open `robot_description.launch` file.
```
gedit ~/catkin_ws/src/my_package/launch/robot_description.launch
```
  
2. Add the following lines after the first `param` definition.
```
  <!-- Send fake joint values-->
  <node name="joint_state_publisher" pkg="joint_state_publisher" type="joint_state_publisher">
    <param name="use_gui" value="false"/>
  </node>

  <!-- Send robot states to tf -->
  <node name="robot_state_publisher" pkg="robot_state_publisher" type="robot_state_publisher" respawn="false" output="screen"/>
```
Those elements add two nodes - the joint_state_publisher and the robot_state_publisher.

3. Open `world.launch` file.
```
gedit ~/catkin_ws/src/my_package/launch/world.launch
```
4. Add the following lines after the urdf_spawner node definition.
```
<!--launch rviz-->
<node name="rviz" pkg="rviz" type="rviz" respawn="false"/>
```
5. Source setup file and roslaunch
```
cd ~/catkin_ws/ && source devel/setup.bash && roslaunch my_package world.launch
```
6. On the left side of RViz, under Displays:
  - Select odom for fixed frame
  - Click the Add button and
    - add RobotModel and your robot model should load up in RViz.
    - add Camera and select the Image topic that was defined in the camera Gazebo plugin
    - add LaserScan and select the topic that was defined in the Hokuyo Gazebo plugin.

7. Add a box or sphere to see the effect on RViz.

## Step 7. Move the robot
1. Publish
```
rostopic pub /cmd_vel geometry_msgs/Twist  "linear:
  x: 0.1
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.1" 
```
## Step 8. Follow a ball in world

Create `ball_chaser` package with `drive_bot` node to employ a service to *publish* velocity to wheel joints and `process_image` node to *subscribe* to the camera image and determine position of the ball.

```
cd ~/catkin_ws/src && catkin_create_pkg ball_chaser roscpp std_msgs message_generation
```
This package will contain C++ source code and messages, that`s why we added `roscpp std_msgs message_generation` dependencies.

```
cd ~/catkin_ws/src/ball_chaser/ && mkdir srv launch
```
mkdir takes multiple arguments, simply run mkdir dir1 dir2 dir3...

`srv` is the directory where you store service files and `launch` is the directory where you store launch files. The `src` directory where you will store C++ programs is created by default.

```
cd ~/catkin_ws/ && catkin_make
```

### Step 8.1. drive_bot node
`a ball_chaser/command_robot` service to drive the robot around by setting its linear x and angular z velocities. The service server publishes messages containing the velocities for the wheel joints.

After writing this node, you will be able to request the `ball_chaser/command_robot` service, either from the terminal or from a client node, to drive the robot by controlling its linear x and angular z velocities.

The drive_bot.cpp node is similar to the arm_mover.cpp node that you already wrote. Both nodes contain a ROS publisher and service. But this time, instead of publishing messages to the arm joint angles, you have to publish messages to the wheels joint angles. Please refer to the arm_mover.cpp node before you begin coding the drive_bot.cpp node.

arm_mover.cpp:
```
#include "ros/ros.h"
#include "simple_arm/GoToPosition.h"
#include <std_msgs/Float64.h>

// Global joint publisher variables
ros::Publisher joint1_pub, joint2_pub;

// This function checks and clamps the joint angles to a safe zone
std::vector<float> clamp_at_boundaries(float requested_j1, float requested_j2)
{
    // Define clamped joint angles and assign them to the requested ones
    float clamped_j1 = requested_j1;
    float clamped_j2 = requested_j2;

    // Get min and max joint parameters, and assigning them to their respective variables
    float min_j1, max_j1, min_j2, max_j2;
    // Assign a new node handle since we have no access to the main one
    ros::NodeHandle n2;
    // Get node name
    std::string node_name = ros::this_node::getName();
    // Get joints min and max parameters
    n2.getParam(node_name + "/min_joint_1_angle", min_j1);
    n2.getParam(node_name + "/max_joint_1_angle", max_j1);
    n2.getParam(node_name + "/min_joint_2_angle", min_j2);
    n2.getParam(node_name + "/max_joint_2_angle", max_j2);

    // Check if joint 1 falls in the safe zone, otherwise clamp it
    if (requested_j1 < min_j1 || requested_j1 > max_j1) {
        clamped_j1 = std::min(std::max(requested_j1, min_j1), max_j1);
        ROS_WARN("j1 is out of bounds, valid range (%1.2f,%1.2f), clamping to: %1.2f", min_j1, max_j1, clamped_j1);
    }
    // Check if joint 2 falls in the safe zone, otherwise clamp it
    if (requested_j2 < min_j2 || requested_j2 > max_j2) {
        clamped_j2 = std::min(std::max(requested_j2, min_j2), max_j2);
        ROS_WARN("j2 is out of bounds, valid range (%1.2f,%1.2f), clamping to: %1.2f", min_j2, max_j2, clamped_j2);
    }

    // Store clamped joint angles in a clamped_data vector
    std::vector<float> clamped_data = { clamped_j1, clamped_j2 };

    return clamped_data;
}

// This callback function executes whenever a safe_move service is requested
bool handle_safe_move_request(simple_arm::GoToPosition::Request& req,
    simple_arm::GoToPosition::Response& res)
{

    ROS_INFO("GoToPositionRequest received - j1:%1.2f, j2:%1.2f", (float)req.joint_1, (float)req.joint_2);

    // Check if requested joint angles are in the safe zone, otherwise clamp them
    std::vector<float> joints_angles = clamp_at_boundaries(req.joint_1, req.joint_2);

    // Publish clamped joint angles to the arm
    std_msgs::Float64 joint1_angle, joint2_angle;

    joint1_angle.data = joints_angles[0];
    joint2_angle.data = joints_angles[1];

    joint1_pub.publish(joint1_angle);
    joint2_pub.publish(joint2_angle);

    // Wait 3 seconds for arm to settle
    ros::Duration(3).sleep();

    // Return a response message
    res.msg_feedback = "Joint angles set - j1: " + std::to_string(joints_angles[0]) + " , j2: " + std::to_string(joints_angles[1]);
    ROS_INFO_STREAM(res.msg_feedback);

    return true;
}

int main(int argc, char** argv)
{
    // Initialize the arm_mover node and create a handle to it
    ros::init(argc, argv, "arm_mover");
    ros::NodeHandle n;

    // Define two publishers to publish std_msgs::Float64 messages on joints respective topics
    joint1_pub = n.advertise<std_msgs::Float64>("/simple_arm/joint_1_position_controller/command", 10);
    joint2_pub = n.advertise<std_msgs::Float64>("/simple_arm/joint_2_position_controller/command", 10);

    // Define a safe_move service with a handle_safe_move_request callback function
    ros::ServiceServer service = n.advertiseService("/arm_mover/safe_move", handle_safe_move_request);
    ROS_INFO("Ready to send joint commands");

    // Handle ROS communication events
    ros::spin();

    return 0;
}
```





