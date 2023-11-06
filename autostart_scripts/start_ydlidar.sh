#!/bin/bash

rosws=dev_ws
rospackage=atcart8_nav2_demo

sleep 20

export DISPLAY=:0.0
export LOGFILE=/home/$USER/$rosws/src/$rospackage/autostart_scripts/ydlidar_launch.log

source /home/$USER/$rosws/src/$rospackage/autostart_scripts/ROS_CONFIG.txt
#source /opt/ros/noetic/setup.bash
source /opt/ros/galactic/setup.bash
source /home/$USER/$rosws/install/local_setup.bash


while true
do
		echo >>$LOGFILE
		echo "----------------------------------------------" >> $LOGFILE
		date >> $LOGFILE

		echo "Starting ydlidar.launch.py" >> $LOGFILE
		
		ros2 launch atcart8_nav2_demo ydlidar.launch.py >> $LOGFILE

		echo "program seems to have stopped" >> $LOGFILE

		date >> $LOGFILE
		sleep 1

done
