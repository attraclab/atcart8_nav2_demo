#!/bin/bash

rosws=dev_ws
rospackage=atcart8_nav2_demo

sleep 80

export DISPLAY=:0.0
export LOGFILE=/home/$USER/$rosws/src/$rospackage/autostart_scripts/nav2.log

source /home/$USER/$rosws/src/$rospackage/autostart_scripts/ROS_CONFIG.txt
#source /opt/ros/noetic/setup.bash
source /opt/ros/galactic/setup.bash
source /home/$USER/$rosws/install/local_setup.bash


while true
do
		echo >>$LOGFILE
		echo "----------------------------------------------" >> $LOGFILE
		date >> $LOGFILE

		echo "Starting nav2_bringup in docker" >> $LOGFILE

		#docker run -d --rm --net=host --env="ROS_DOMAIN_ID=1" --name nav2 humble-nav2 bash  -it -c  "ros2 launch nav2_bringup bringup_launch.py use_sim_time:=False autostart:=True map:=/home/user/nav2/map/office.yaml  params_file:=/home/user/nav2/config/nav2_params_humble.yaml" 
		docker run -d --rm --net=host --env="ROS_DOMAIN_ID=1" --name nav2 humble-nav2

		docker logs -f nav2 >> $LOGFILE

		echo "program seems to have stopped" >> $LOGFILE

		date >> $LOGFILE
		sleep 1

done
