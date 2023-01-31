#!/bin/bash

source /opt/ros/humble/setup.bash

echo "Start nav2pose_client in docker"

cd /home/user/nav2

ros2 launch nav2_bringup bringup_launch.py use_sim_time:=False autostart:=True map:=/home/user/nav2/map/office.yaml  params_file:=/home/user/nav2/config/nav2_params_humble.yaml