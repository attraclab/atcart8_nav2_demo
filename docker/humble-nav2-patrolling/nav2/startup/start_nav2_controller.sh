#!/bin/bash

source /opt/ros/humble/setup.bash

echo "Start nav2_controller in docker"

cd /home/user/nav2/scripts

python3 nav2_controller.py