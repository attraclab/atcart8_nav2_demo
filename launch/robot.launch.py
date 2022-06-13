from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import LifecycleNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os

def generate_launch_description():
	ld = LaunchDescription()

	### JMOAB ATCART8 ###
	atcart8_node = Node(
		package="jmoab_ros2",
		executable="atcart8",
		name="atcart8")

	imu_node = Node(
		package="jmoab_ros2",
		executable="bno055",
		name="imu")

	base_to_imu_node = Node(
		package="tf2_ros",
		executable="static_transform_publisher",
		arguments=["0", "0", "0", "0", "0", "0", "base_link", "imu_link"],
		name="base_to_imu_tf")

	base_to_footprint_node = Node(
		package="tf2_ros",
		executable="static_transform_publisher",
		arguments=["0", "0", "0", "0", "0", "0", "base_link", "base_footprint"],
		name="base_to_footprint_tf")


	### YDLidar ##
	### copied from https://github.com/YDLIDAR/ydlidar_ros2_driver/blob/master/launch/ydlidar_launch.py
	share_dir = get_package_share_directory('ydlidar_ros2_driver')
	parameter_file = LaunchConfiguration('ydlidar_params_file')
	node_name = 'ydlidar_ros2_driver_node'

	ydlidar_params_declare = DeclareLaunchArgument('ydlidar_params_file',
										   default_value=os.path.join(
											   get_package_share_directory('atcart8_nav2_demo'), 'config', 'ydlidar.yaml'),
										   description='FPath to the ROS2 parameters file to use.')

	ydlidar_driver_node = Node(package='ydlidar_ros2_driver',
								executable='ydlidar_ros2_driver_node',
								name='ydlidar_ros2_driver_node',
								output='screen',
								parameters=[parameter_file],
								)
	base_to_laser_node = Node(package='tf2_ros',
					executable='static_transform_publisher',
					name='static_tf_pub_laser',
					arguments=['0.2', '0', '0.24','0', '0', '0', '1','base_link','laser_frame'],
					)

	### robot_localization ##
	ekf_node = Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[os.path.join(get_package_share_directory('atcart8_nav2_demo'),'config','ekf.yaml')],
           )

	ld.add_action(atcart8_node)
	ld.add_action(imu_node)
	ld.add_action(base_to_imu_node)
	ld.add_action(base_to_footprint_node)
	ld.add_action(ydlidar_params_declare)
	ld.add_action(ydlidar_driver_node)
	ld.add_action(base_to_laser_node)
	ld.add_action(ekf_node)

	return ld