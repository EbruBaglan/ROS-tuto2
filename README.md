# ROS-tuto2

## Creating a world with a custom robot inside
1. Go to `catkin_ws/src` and type
```
catkin_create_pkg my_package
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

```
mkdir meshes
```

Download [hokuyo.dae](https://s3-us-west-1.amazonaws.com/udacity-robotics/hokuyo.dae) and put it in 
