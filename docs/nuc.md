# Setup Intel NUC with ZMOAB

ZMOAB is originally made from ROS1, so we need to install ROS1 as well.

## Dependencies

Please make sure to have these following packages install before,

### ROS1

- install ROS1 ([noetic](http://wiki.ros.org/noetic/Installation/Ubuntu), recommended)

- `sudo apt install ros-noetic-rosserial-python`

### ROS2

- install ROS2 [Foxy](https://docs.ros.org/en/foxy/Installation.html).

- `sudo apt install ros-foxy-robot-localization`

- `sudo apt install ros-foxy-navigation2`

- `sudo apt install ros-foxy-nav2-bringup`

- `sudo apt install ros-foxy-slam-toolbox`

Assume that you have installed ROS2 and created the workspace as `dev_ws`. Then clone these packages to your `dev_ws/src` and do `colcon build --symlink-install` .

- https://github.com/YDLIDAR/ydlidar_ros2_driver

- https://github.com/attraclab/zmoab_ros2_utils, check on the installation step of zmoab_ros2_utils as well.

- https://github.com/attraclab/atcart8_nav2_demo

- https://github.com/ros2/ros1_bridge , MUST build `ros1_bridge` as explained [here](https://github.com/ros2/ros1_bridge#building-the-bridge-from-source).

## Pretest

Let's make sure that we can have a fully control of ATCart8 and odometry published properly.

We need to start ZMOAB nodes to port ROS1 topics to ROS2, please check on the [step here](https://github.com/attraclab/zmoab_ros2_utils#run-manually).

```sh
# Testing Odometry and EKF from zmoab_ros2_utils

## Assume that all of ZMOAB nodes are running,

## Terminal1 (on PC)
source /opt/ros/foxy/setup.bash
source ~/dev_ws/install/local_setup.bash
rviz2
### open the config file from ~/dev_ws/src/atcart8_nav2_demo/rviz/ directory
### select odom_ekf_test.rviz
### you should be able to see red arrow which is atcart8/odom (original odom), 
### and also green arrow odometry/filtered (fused odom)
### if moving around it should change the orientation too.

```

```sh
# Testing ROS2 YDLidar laserscan

## Terminal1 (on NUC)
source /opt/ros/foxy/setup.bash
source ~/dev_ws/install/local_setup.bash
ros2 launch atcart8_nav2_demo ydlidar.launch.py

## Terminal2 (on PC)
source /opt/ros/foxy/setup.bash
source ~/dev_ws/install/local_setup.bash
rviz2
### open the config file from ~/dev_ws/src/atcart8_nav2_demo/rviz/ directory
### select laserscan.rviz
### you should be able to see red dots of laserscan, if moving around it should change the orientation too.
```

## Run

When changing new environment, we need to create new map of that environment, save the map, and use it for navigation later.
Here is the workflow.

### Making a map from slam_toolbox

```sh

## Assume that all ZMOAB nodes are running either by manual or auto start

## Terminal1 (on NUC)
source /opt/ros/foxy/setup.bash
source ~/dev_ws/install/local_setup.bash
ros2 launch atcart8_nav2_demo ydlidar.launch.py

## Terminal2 (on PC)
source /opt/ros/foxy/setup.bash
source ~/dev_ws/install/local_setup.bash
rviz2
### open the config file from ~/dev_ws/src/atcart8_nav2_demo/rviz/ directory
### select slam_mapping.rviz

## Terminal3 (on NUC)
source /opt/ros/foxy/setup.bash
source ~/dev_ws/install/local_setup.bash
ros2 run slam_toolbox async_slam_toolbox_node --ros-args --params-file /home/$USER/dev_ws/src/atcart8_nav2_demo/config/mapper_params_online_async.yaml

### on Rviz2 window, you should see the map slowly generate,
### we just need to move the bot to cover the area we want

## Terminal4 (on NUC)
source /opt/ros/foxy/setup.bash
source ~/dev_ws/install/local_setup.bash
cd ~/dev_ws/src/atcart8_nav2_demo/map
ros2 run nav2_map_server map_saver_cli -f <map_name>
### if it shows error of timeout, please run map_saver again until it's complete saving.
```

### Navigation

```sh
## Assume that all ZMOAB nodes are running either by manual or auto start

## Terminal1 (on NUC)
source /opt/ros/foxy/setup.bash
source ~/dev_ws/install/local_setup.bash
ros2 launch atcart8_nav2_demo ydlidar.launch.py

## Terminal2 (on PC)
source /opt/ros/foxy/setup.bash
source ~/dev_ws/install/local_setup.bash
rviz2
### open the config file from ~/dev_ws/src/atcart8_nav2_demo/rviz/ directory
### select nav2_default_view.rviz

## Terminal3 (on Jetson)
source /opt/ros/foxy/setup.bash
source ~/dev_ws/install/local_setup.bash
ros2 launch nav2_bringup bringup_launch.py use_sim_time:=False autostart:=True map:=/home/$USER/dev_ws/src/atcart8_nav2_demo/map/<your-map>.yaml params_file:=/home/$USER/dev_ws/src/atcart8_nav2_demo/config/nav2_params_zmoab.yaml
### please specify the params_file and map correctly

### nav2_params.yaml is for original size atcart8
### nav2_params_smallCart.yaml is for small size atcart8

### Click on and drag on Navigation 2 Goal panel to send goal_pose to controller
```