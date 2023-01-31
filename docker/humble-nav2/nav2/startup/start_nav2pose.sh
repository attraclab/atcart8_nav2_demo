#!/bin/bash

source /opt/ros/humble/setup.bash

echo "Start nav2pose_client in docker"

cd /home/user/nav2/scripts

python3 nav2pose_client.py