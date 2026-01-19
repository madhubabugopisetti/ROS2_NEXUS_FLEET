# ROS2_NEXUS_FLEET

### Kill terminals
```
pkill -f ros2
pkill -f gazebo
pkill -f gz
pkill -f rviz
pkill -f nav2
pkill -f slam_toolbox

export GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ros/$ROS_DISTRO/lib:$GZ_SIM_SYSTEM_PLUGIN_PATH
export LD_LIBRARY_PATH=/opt/ros/$ROS_DISTRO/lib:$LD_LIBRARY_PATH
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
ros2 run xacro xacro ~/ros2_nexus_fleet_ws/src/fleet_description/urdf/nexus_fleet.xacro > /tmp/test.urdf
check_urdf /tmp/test.urdf
```
- Add spawn node in gazebo_rviz.launch.py
- [BUILD](#build)
- Terminal 1: ros2 launch fleet_description gazebo_rviz.launch.py

## GOAL 2: Move model in gazebo and rviz via commands

- Remove caster wheels and add front wheels & back wheels
- Add a controller manager and define their parameters
- Remove base_link of arm , connect car_base_link to arm_shoulder_link
- Add **clock**, jsb, **load_controller**(auto activate jps, diff_drive_controller, aarm_controllerc) nodes in gazebo_rviz.launch.py
- [BUILD](#build)
- Terminal 1: ros2 launch fleet_description gazebo_rviz.launch.py
- ![alt text](./src/images/image-2.png)
- Terminal 2: ros2 control list_controllers
- Terminal 3: 
```
ros2 topic pub -r 10 /diff_drive_controller/cmd_vel geometry_msgs/msg/TwistStamped "
header:
    frame_id: car_base_link
twist:
    linear:
        x: 0.3
    angular:
        z: 0.2
"
```
- ![alt text](./src/images/image-4.png)
- Terminal 4: 
```
ros2 topic pub --once /arm_controller/joint_trajectory trajectory_msgs/msg/JointTrajectory "
joint_names:
- arm_shoulder_joint
- arm_elbow_joint
- arm_forearm_joint
- arm_wrist_pitch_joint
- arm_wrist_roll_joint
points:
- positions: [0.5, 1.0, 0.5, 0.3, 0.0]
    time_from_start: {sec: 3}
"
```
- ![alt text](./src/images/image-3.png)
- Add Spawn node with fleet.rviz
- Fixed Frame -> Odom, Add Robot Model, Add TF save config as fleet.rviz
![alt text](./src/images/image-6.png)
![alt text](./src/images/image-5.png)

## GOAL3: Create a map

### Step 1: LiDAR & camer Setup
- Add objects in world.sdf
- Add **gpu_lidar**, **camera** sensors and **gz-sim-sensors-system** plugin in world.xacro
- Add **/scan**, **/camera/image**, bridges in gazebo_rviz.launch.py
- [BUILD](#build)
- Terminal 1: ros2 launch fleet_description gazebo_rviz.launch.py
- Add image, topic as /camera/image and ScanLaser, topic as /scan -> save config
- ![alt text](./src/images/image-7.png)

### Step 2: Create map
- Create slam.launch.py to create map
- Add red cube for pick up
- Add blue cube for docking
- [BUILD](#build)
- Terminal 1: ros2 launch fleet_description gazebo_rviz.launch.py
- ![alt text](./src/images/image-8.png)
- Terminal 2: ros2 launch fleet_description slam.launch.py
- Terminal 3: ros2 lifecycle set /slam_toolbox configure  ros2 lifecycle set /slam_toolbox activate
- Fixed Frame -> map, Add Map, topic as /map
- ![alt text](./src/images/image-9.png)
- Terminal 4: ros2 run teleop_twist_keyboard teleop_twist_keyboard
- ![alt text](./src/images/image-10.png)
- Terminal 5: ros2 run nav2_map_server map_saver_cli -f ~/ros2_nexus_fleet_ws/src/fleet_description/maps/my_map

## GOAL 4: Localization (auto load saved map)
- Create localization.launch.py file in launch and amcl.yaml in config
- Add Map Server → loads your saved map, AMCL → localizes robot in that map, Lifecycle Manager → automatically starts them
- [BUILD](#build)
- Terminal 1: ros2 launch fleet_description gazebo_rviz.launch.py
- Terminal 2: ros2 launch fleet_description localization.launch.py
- Fixed Fram -> map
- ![alt text](./src/images/image-11.png)

## GOAL 5: Navigation - Goal Point click
- Create navigation.launch.py file in launch and nav2.yaml in config
- Add Planner → makes a path, Controller → follows the path, Behavior tree → orchestrates navigation, Recovery behaviors → unstuck logic, Velocity smoother → smooth motion, Lifecycle manager → auto-starts everything
- [BUILD](#build)
- Terminal 1: ros2 launch fleet_description gazebo_rviz.launch.py
- Terminal 2: ros2 launch fleet_description localization.launch.py
- Terminal 3: ros2 launch fleet_description navigation.launch.py
- ![alt text](./src/images/image-12.png)

## GOAL 7: AUTO NAVIGATION - To pickup point
- Create new package
```
cd ~/ros2_nexus_fleet_ws/src
source /opt/ros/jazzy/setup.bash
ros2 pkg create nexus_car_auto_nav --build-type ament_python --dependencies rclpy nav2_msgs geometry_msgs
```
```
cd ~/ros2_nexus_fleet_ws
colcon build --symlink-install
source install/setup.bash
```
- Create a node file
```
cd ~/ros2_nexus_fleet_ws/src/nexus_car_auto_nav/nexus_car_auto_nav
touch auto_nav.py
chmod +x auto_nav.py
```
- Add it to **entry_points** ```'auto_nav = nexus_car_auto_nav.auto_nav:main',```
- [BUILD](#build)
- Terminal 1: ros2 launch fleet_description gazebo_rviz.launch.py
- Terminal 2: ros2 launch fleet_description localization.launch.py
- Terminal 3: ros2 launch fleet_description navigation.launch.py
- Terminal 3: ros2 run nexus_car_auto_nav auto_nav
- ![alt text](./src/images/image-13.png)