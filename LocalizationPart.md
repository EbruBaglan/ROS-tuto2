# Localization

```
cd ~/catkin_ws/src && git clone https://github.com/turtlebot/turtlebot_simulator
```

```
cd ~/catkin_ws
source devel/setup.bash
rosdep -i install turtlebot_gazebo
```

```
catkin_make
source devel/setup.bash
```
```
roslaunch turtlebot_gazebo turtlebot_world.launch
```
