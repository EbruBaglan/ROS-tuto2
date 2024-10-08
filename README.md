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

3. Go to `my_robot.xacro`
```
cd ~/catkin_ws/src/my_package/urdf && gedit my_robot.xacro
```

4. Add the following line to `my_robot.xacro` immediately before `robot_footprint` link.
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

Create `DriveToTarget.srv` under `srv`.
```
cd ~/catkin_ws/src/ball_chaser/srv && touch DriveToTarget.srv 
```

Fill in the following into file.
```
float64 linear_x
float64 angular_z
---
string msg_feedback
```
Check if it works:
```
cd ~/catkin_ws/ && source devel/setup.bash && rossrv show DriveToTarget
```

Create the `drive_bot` node.
```
cd ~/catkin_ws/src/my_package/src && touch drive_bot.cpp
```
The code is as follows.
```
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "ball_chaser/DriveToTarget.h"

// ROS::Publisher motor commands;
ros::Publisher motor_command_publisher;

// TODO: Create a handle_drive_request callback function that executes whenever a drive_bot service is requested
// This function should publish the requested linear x and angular velocities to the robot wheel joints
// After publishing the requested velocities, a message feedback should be returned with the requested wheel velocities
  bool handle_drive_request(ball_chaser::DriveToTarget::Request& req, ball_chaser::DriveToTarget::Response& res)
  {

    ROS_INFO("DriveToTarget Request received - linear_x:%1.2f, angular_z:%1.2f", (float)req.linear_x, (float)req.angular_z);

    // Create a motor_command object of type geometry_msgs::Twist
    geometry_msgs::Twist motor_command;
    // Set wheel velocities
    motor_command.linear.x = req.linear_x;
    motor_command.angular.z = req.angular_z;
    // Publish angles to drive the robot
    motor_command_publisher.publish(motor_command);

    // Return a response message
    res.msg_feedback = "cmd_vel command set - linear_x: " + std::to_string(motor_command.linear.x) + " , angular_z: " + std::to_string(motor_command.angular.z);
    ROS_INFO_STREAM(res.msg_feedback);

    return true;
  }

int main(int argc, char** argv)
{
    // Initialize a ROS node
    ros::init(argc, argv, "drive_bot");

    // Create a ROS NodeHandle object
    ros::NodeHandle n;

    motor_command_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    //Topic you want to subscribe
    // TODO: Define a drive /ball_chaser/command_robot service with a handle_drive_request callback function
    ros::ServiceServer service = n.advertiseService("/ball_chaser/command_robot", handle_drive_request);

    ROS_INFO("Ready to send drive requests!");

    ros::spin();

    return 0;
}
```

Edit CMakeLists.txt:
```
cmake_minimum_required(VERSION 3.0.2)
project(ball_chaser)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
)

## Generate services in the 'srv' folder
add_service_files(
   FILES
   DriveToTarget.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES ball_chaser
#  CATKIN_DEPENDS message_generation roscpp std_msgs
#  DEPENDS system_lib
)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(drive_bot src/drive_bot.cpp)
target_link_libraries(drive_bot ${catkin_LIBRARIES})
add_dependencies(drive_bot ball_chaser_generate_messages_cpp)

```
Build package:
```
cd ~/catkin_ws/ && catkin_make && source devel/setup.bash
```
Launch the world
```
roslaunch my_package world.launch
```
Run node
```
cd ~/catkin_ws/ && source devel/setup.bash && rosrun ball_chaser drive_bot
```

Request a ball_chaser/command_robot service. Go forward:
```
rosservice call /ball_chaser/command_robot "linear_x: 0.5 angular_z: 0.0"  
```

Go left:
```
rosservice call /ball_chaser/command_robot "linear_x: 0.0 angular_z: 0.5" 
```
Go right? Stop?
```
rosservice call /ball_chaser/command_robot "linear_x: angular_z: "
```

Create a launch file under `ball chaser` package.
```
gedit ~/catkin_ws/src/ball_chaser/launch/ball_chase.launch
```

Add the following inside.
```
<launch>

 <!-- The drive_bot node -->
  <node name="drive_bot" type="drive_bot" pkg="ball_chaser" output="screen">
  </node>

</launch>
```




### Step 8.2. Model a White Ball.
Open Gazebo.
```
gazebo
```
Go to Edit >>Model Editor.

Double click on the sphere, and change its radius to 0.1 both in Visual and Collision.

To change the ball’s color to white: set its Visual Ambient, Diffuse, Specular, and Emissive RGBA values to 1.

Save the white ball model as `my_ball` under the /home directory. Then exit the Model Editor tool and go back to the Gazebo main world.

Now that you are back in the Gazebo main world, you can click on “Insert” and drop the white ball anywhere in the scene.

Now that you modeled the white ball, relaunch the nodes inside world.launch. Then verify that you can insert a `my_ball` anywhere inside your world.

Place the white ball anywhere outside of your building structure, so that the robot would not see it. Then, save a copy of this new world under `~/catkin_ws/src/my_robot/worlds` by replacing your old `<yourname>.world` file. Whenever you launch this newly saved world you should be able to see your building environment, in addition, the white ball.

process_image: This client node will subscribe to the robot’s camera images and analyze them to determine the position of the white ball. 
Once the ball position is determined, the client node will request a service from the drive_bot server node to drive the robot toward the ball. The robot can drive either left, right or forward, depending on the robot position inside the image.

After you write this node, place the white ball in front of the robot’s camera. If everything works, your node should analyze the image, detect the ball’s position, and then request a ball_chaser/command_robot service to drive the robot towards the ball!

The process_image.cpp client node is similar to the look_away.cpp client node. You’ll have to subscribe to the /camera/rgb/image_raw topic to get instantaneous images from the robot’s camera.


process_image.cpp: This node will analyze the image and request services to drive the robot. Create the source code file within the src directory of your ball_chaser package. It might be a bit challenging to write this program from scratch, thus I am providing you with some hints. Attached below is a piece of the complete code with multiple hints to help you finish the implementation.

Create the `process_image` node.
```
cd ~/catkin_ws/src/my_package/src && touch process_image.cpp
```


The code is as follows.
```
#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // TODO: Request a service and pass the velocities to it to drive the robot
    ROS_INFO_STREAM("Driving the robot to the target.");
    
    // Request service with velocities
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;
    
    // Call the DriveToTarget service and pass the requested velocities
    if (!client.call(srv)) {
	    ROS_ERROR("Failed to call service DriveToTarget.");
	}
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{

    int white_pixel = 255;

    int height = img.height;
    int step = img.step;
    int region = -1;

    for (int i = 0; i < height*step; i++){
		if ((img.data[i]==255) && (img.data[i+1] == 255) && (img.data[i+2] == 255)){
			region = i % step;
			break;
		}
	}
    float lin_x=0.0,ang_z=0.0;
	// alani belirle sol mu sag mi

    drive_robot(lin_x,ang_z);
    if (region == -1){
		ROS_INFO_STREAM("Target not found: Stopping robot...");
	}
	else{
		ROS_INFO_STREAM("Target Detected: Driving to target");
	}

    // TODO: Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}
```



