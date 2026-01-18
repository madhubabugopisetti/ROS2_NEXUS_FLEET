# ROS2_NEXUS_FLEET

### Kill terminals
```
pkill -f ros2
pkill -f gazebo
pkill -f gz
pkill -f rviz
pkill -f nav2
pkill -f slam_toolbox
```

### BUILD
```
cd ~/ros2_nexus_fleet_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install
source install/setup.bash
clear
```

## GOAL 1: Render model in Gazebo

### STEP 1: Creating a workspace
```
mkdir -p ~/ros2_nexus_fleet_ws/src
cd ~/ros2_nexus_fleet_ws/src
source /opt/ros/jazzy/setup.bash
ros2 pkg create fleet_description --build-type ament_cmake
```

### STEP 2: Creating folders structure
```
fleet_description/
    ├── config/
        ├── 
    ├── launch/
        ├── gazebo_rviz.launch.py
    ├── rviz/
        ├── 
    ├── urdf/
        ├── arm.xacro
        ├── car.xacro
        ├── world.xacro
    ├── worlds/
        ├── world.sdf
```
```
cd fleet_description

mkdir -p config
mkdir -p launch
mkdir -p rviz
mkdir -p urdf
mkdir -p worlds

touch launch/gazebo_rviz.launch.py
touch urdf/car.xacro
touch urdf/arm.xacro
touch urdf/world.xacro
touch worlds/world.sdf

cd ~/ros2_nexus_fleet_ws

colcon build
```

### STEP 3: Create world and render in gazebo
- Add folder config, launch, urdf, worlds, rviz, maps to CMakeLists.txt
- Add world, walls in world.sdf
- Terminal 1: gz sim -r ~/ros2_nexus_fleet_ws/src/fleet_description/worlds/world.sdf<br />
- Add gazebo node in gazebo_rviz.launch.py
- Terminal 1: ros2 launch fleet_description gazebo_rviz.launch.py
![alt text](./src/images/image-1.png)


### STEP 4: Create model
- car.xacro code from **ROS2_NEXUS_ADS**, and add **${prefix}** in link name
- arm.xacro code from **ROS2_NEXUS_AGS**, and add **${prefix}** in link name
- world.xacro add a joint to attach car + arm
- Verify
```
ros2 run xacro xacro ~/ros2_nexus_fleet_ws/src/fleet_description/urdf/world.xacro > /tmp/test.urdf
check_urdf /tmp/test.urdf
```
- Add spawn node in gazebo_rviz.launch.py
- [BUILD](#build)
- Terminal 1: ros2 launch fleet_description gazebo_rviz.launch.py