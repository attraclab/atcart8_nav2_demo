FROM ros:humble
RUN apt-get update && \
apt-get install -y \
	ros-humble-navigation2 \
	ros-humble-nav2-bringup \
	ros-humble-nav2-simple-commander \
	ros-humble-slam-toolbox \
	ros-humble-tf-transformations \
	python3-pip \
	vim
	
RUN pip3 install transforms3d
	
RUN mkdir -p /home/user/nav2
COPY ./nav2 /home/user/nav2

RUN chmod +x /home/user/nav2/startup/*

COPY ./ros_entrypoint.sh /

ENTRYPOINT ["/ros_entrypoint.sh"]

CMD ["bash"]
