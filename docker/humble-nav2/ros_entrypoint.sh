#!/bin/bash
set -e

# setup ros2 environment
source "/opt/ros/$ROS_DISTRO/setup.bash" --

cd /home/user/nav2/startup

bash start_nav2_bringup.sh & bash start_nav2pose.sh

exec "$@"
