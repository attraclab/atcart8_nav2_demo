import time
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from nav2_msgs.action import NavigateToPose, NavigateThroughPoses, FollowWaypoints
from rclpy.action import ActionClient
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, PoseArray, Pose
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from lifecycle_msgs.srv import GetState
import numpy as np

class Nav2Controller(Node):

	def __init__(self):
		super().__init__('nav2_controller_node')
		self.get_logger().info('Start nav2_controller_node')

		self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
		self.initial_pose = PoseWithCovarianceStamped()
		self.initial_pose.header.frame_id = 'map'
		self.initial_pose.header.stamp = self.get_clock().now().to_msg()
		self.initial_pose.pose.pose.position.x = 0.0
		self.initial_pose.pose.pose.position.y = 0.0
		self.initial_pose.pose.pose.orientation.z = 0.0
		self.initial_pose.pose.pose.orientation.w = 1.0

		##############
		## Nav2Pose ##
		##############
		self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
		self.goal_point_sub = self.create_subscription(PoseStamped, 'target_point', self.goal_point_callback, 1)
		self.cancel_sub = self.create_subscription(Bool, 'cancel_task', self.cancel_task_callback, 1)
		self.init_pose_sub = self.create_subscription(Bool, 'set_init_pose', self.set_init_pose_callback, 1)
		self.got_nav2pose_goal = False
		self.nav2pose_done = False
		######################
		### FollowWaypoint ###
		######################
		# self.nav_through_poses_client = ActionClient(self, NavigateThroughPoses, 'navigate_through_poses')
		self.follow_waypoint_client = ActionClient(self, FollowWaypoints, 'follow_waypoints')
		self.goal_poses_sub = self.create_subscription(PoseArray, 'goal_poses', self.goal_poses_callback, 1)
		self.cancel_poses_sub = self.create_subscription(Bool, 'cancel_poses_task', self.cancel_poses_task_callback, 1)
		self.got_followWaypoint_goal = False
		self.followWaypoint_done = False
		self.goal_poses = []
		self.map_goal_poses = []
		self.closest_idx = 0
		self.current_waypoint = 0
		self.map_current_waypoint = 0
		self.goal_poses_len = 0
		self.feedback_last_stamp = time.time()
		self.last_cancel_stamp = time.time()
		self.first_time = True
		# self.start_stamp = time.time()
		# self.last_print_stamp = time.time()
		# self.once = True
		# self.timer = self.create_timer(0.02, self.timer_callback)

		self.robot_pose_sub = self.create_subscription(Pose, 'robot_pose', self.robot_pose_callback, 1)
		self.robot_pose = Pose()

	##############################
	### Nav2Pose action client ###
	##############################
	def nav2pose_send_goal(self, pose):

		self.get_logger().info("Waiting for 'NavigateToPose' action server")

		while not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
			self.get_logger().info("'NavigateToPose' action server not available, waiting...")

		self.got_nav2pose_goal = True
		goal_msg = NavigateToPose.Goal()
		goal_msg.pose = pose
		goal_msg.behavior_tree = ''

		self.nav2pose_done = False
		self.followWaypoint_done = False

		self.get_logger().info("NavToPose Send goal pose")
		send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg, self.nav2pose_feedbackCallback)
		send_goal_future.add_done_callback(self.nav2pose_goal_response_callback)

	def nav2pose_goal_response_callback(self, future):
		goal_handle = future.result()
		if not goal_handle.accepted:
			self.get_logger().info('NavToPose Goal rejected :(')
			return

		self.get_logger().info('NavToPose Goal accepted :)')
		print("nav2pose_done", self.nav2pose_done)
		self.nav2pose_goal_handle = goal_handle

		get_result_future = goal_handle.get_result_async()
		get_result_future.add_done_callback(self.nav2pose_get_result_callback)

	def nav2pose_get_result_callback(self, future):
		result = future.result().result
		self.get_logger().info('NavToPose result {:}'.format(result.result))
		self.nav2pose_done = True
		print("nav2pose_done", self.nav2pose_done)
		# self.get_logger().info('Result: {0}'.format(result.result))

	def nav2pose_feedbackCallback(self, msg):
		# self.get_logger().info('NavToPose feedback {:}'.format(msg.feedback))
		# feedback = msg.feedback
		return

	def nav2pose_cancel_done(self, future):
		cancel_response = future.result()
		
		if len(cancel_response.goals_canceling) > 0:
			self.get_logger().info('NavToPose Goal successfully canceled')
		else:
			self.get_logger().info('NavToPose Goal failed to cancel')

	#####################################
	### FollowWaypoint action client ###
	#####################################

	def followWaypoint_send_goal(self, poses):

		self.get_logger().info("Waiting for 'FollowWaypoint' action server")
		while not self.follow_waypoint_client.wait_for_server(timeout_sec=1.0):
			self.get_logger().info("FollowWaypoint action server not available, waiting...")

		goal_msg =  FollowWaypoints.Goal() #FollowWaypoint.Goal()
		goal_msg.poses = poses
		#goal_msg.behavior_tree = ''

		self.got_followWaypoint_goal = True
		self.followWaypoint_done = False
		self.nav2pose_done = False

		self.get_logger().info(f'FollowWaypoint Navigating with {len(goal_msg.poses)} goals....')
		send_goal_future = self.follow_waypoint_client.send_goal_async(goal_msg, self.followWaypoint_feedbackCallback)
		send_goal_future.add_done_callback(self.followWaypoint_goal_response_callback)

	def followWaypoint_goal_response_callback(self, future):
		goal_handle = future.result()
		if not goal_handle.accepted:
			self.get_logger().info('FollowWaypoint Goal rejected :(')
			return

		self.get_logger().info('FollowWaypoint Goal accepted :)')
		print("followWaypoint_done", self.followWaypoint_done)
		self.followWaypoint_goal_handle = goal_handle

		get_result_future = goal_handle.get_result_async()
		get_result_future.add_done_callback(self.followWaypoint_get_result_callback)

	def followWaypoint_get_result_callback(self, future):
		result = future.result().result
		# self.get_logger().info('Result: {0}'.format(result.result))
		self.get_logger().info('FollowWaypoint result {:}'.format(result))
		if self.current_waypoint == (self.goal_poses_len-1):
			self.followWaypoint_done = True
			print("followWaypoint_done", self.followWaypoint_done)
			# Loop over once finished the last point
			self.followWaypoint_send_goal(self.goal_poses)

	def followWaypoint_feedbackCallback(self, msg):
		# self.get_logger().info('FollowWaypoint feedback {:}'.format(msg.feedback))
		self.current_waypoint = msg.feedback.current_waypoint

		if ((self.closest_idx + self.current_waypoint) >= self.goal_poses_len):
			self.map_current_waypoint = self.closest_idx + self.current_waypoint - self.goal_poses_len 
		else:
			self.map_current_waypoint = self.closest_idx + self.current_waypoint

		if (time.time() - self.feedback_last_stamp) > 1.0:
			print("goal_len {} current_wp {} map_current_wp {}".format(self.goal_poses_len, self.current_waypoint, self.map_current_waypoint))
			self.feedback_last_stamp = time.time()
		# feedback = msg.feedback
		return

	def followWaypoint_cancel_done(self, future):
		cancel_response = future.result()
		self.last_cancel_stamp = time.time()

		if len(cancel_response.goals_canceling) > 0:
			self.get_logger().info('FollowWaypoint Goal successfully canceled')
		else:
			self.get_logger().info('FollowWaypoint Goal failed to cancel')

	###############################
	### ros topics and callback ###
	###############################

	def set_init_pose_callback(self, msg):

		if msg.data == True:
			self.set_initial_pose()

	def set_initial_pose(self):
		# msg = PoseWithCovarianceStamped()
		# msg.pose.pose = self.initial_pose.pose
		# msg.header.frame_id = self.initial_pose.header.frame_id
		# msg.header.stamp = self.initial_pose.header.stamp
		
		self.initial_pose_received = False

		# while not self.initial_pose_received:
		for i in range(1):
			self.get_logger().info('Publishing Initial Pose')
			self.initial_pose_pub.publish(self.initial_pose)

		self.get_logger().info('Set Initial Pose done')

	### Nav2Pose callback ###
	def cancel_task_callback(self, msg):

		if msg.data == True:
			if self.got_nav2pose_goal:
				self.get_logger().info("Nav2Pose Try cancel task")
				future = self.nav2pose_goal_handle.cancel_goal_async()
				future.add_done_callback(self.nav2pose_cancel_done)
				self.got_nav2pose_goal = False

	def goal_point_callback(self, msg):

		self.get_logger().info("Got goal point")
		goal_pose = PoseStamped()
		goal_pose.header.frame_id = 'map'
		goal_pose.header.stamp = self.get_clock().now().to_msg()
		goal_pose.pose.position.x = msg.pose.position.x
		goal_pose.pose.position.y = msg.pose.position.y
		goal_pose.pose.orientation.x = msg.pose.orientation.x
		goal_pose.pose.orientation.y = msg.pose.orientation.y
		goal_pose.pose.orientation.z = msg.pose.orientation.z
		goal_pose.pose.orientation.w = msg.pose.orientation.w

		# self.get_logger().info(goal_pose)

		self.nav2pose_send_goal(goal_pose)

	### FollowWaypoint ###
	def cancel_poses_task_callback(self, msg):
		print(msg.data)
		if msg.data == True:
			if self.got_followWaypoint_goal:
				self.get_logger().info("FollowWaypoint Try cancel task")
				future = self.followWaypoint_goal_handle.cancel_goal_async()
				future.add_done_callback(self.followWaypoint_cancel_done)
				self.got_followWaypoint_goal = False
			# self.get_logger().info("FollowWaypoint Try cancel task")
			# future = self.followWaypoint_goal_handle.cancel_goal_async()
			# future.add_done_callback(self.followWaypoint_cancel_done)

	def goal_poses_callback(self, msg):

		self.goal_poses = []
		self.map_goal_poses = []
		for pose in msg.poses:
			goal_pose = PoseStamped()
			goal_pose.header.frame_id = 'map'
			goal_pose.header.stamp = self.get_clock().now().to_msg()
			goal_pose.pose.position.x = pose.position.x
			goal_pose.pose.position.y = pose.position.y
			goal_pose.pose.orientation.w = pose.orientation.w
			goal_pose.pose.orientation.z = pose.orientation.z

			self.map_goal_poses.append(goal_pose)

			print(goal_pose)

		self.goal_poses_len = len(self.map_goal_poses)

		## for first time, we are going to the closest point within direction of path
		## and in case we cancel it, and after while we will use closest point again
		## but if it's a short pause, we will go to the next map_current_waypoint
		if ((time.time() - self.last_cancel_stamp) > 20.0) or self.first_time:
			if self.first_time:
				print("First time, start at closest point")
				self.first_time = False
			else:
				print("Longer than 10 seconds since last cancel")
			self.goal_poses, self.closest_idx = self.setClosestPointAsFirstIndex(self.map_goal_poses)
			self.followWaypoint_send_goal(self.goal_poses)
		else:
			print("continue next waypoint")
			self.goal_poses = self.reIndexingArray(self.map_goal_poses, self.map_current_waypoint)
			self.closest_idx = self.map_current_waypoint
			self.followWaypoint_send_goal(self.goal_poses)

	### robot pose ###
	def robot_pose_callback(self, msg):

		self.robot_pose = msg

	def setClosestPointAsFirstIndex(self, goal_poses_array):

		robot_x = self.robot_pose.position.x
		robot_y = self.robot_pose.position.y

		print("robot_pose ", robot_x, robot_y)

		dist_array = []
		total_points = len(goal_poses_array)
		closest_idx = 0
		closest_idx_2nd = 1
		closest_dist = 0
		closest_dist_2nd = 0
		for i, goal_pose in enumerate(goal_poses_array):

			goal_x = goal_pose.pose.position.x
			goal_y = goal_pose.pose.position.y

			dist = np.sqrt((goal_y - robot_y)**2 + (goal_x - robot_x)**2)

			if i == 0:
				closest_idx = 0
				closest_dist = dist
				closest_dist_2nd = dist
			else:
				if (dist < closest_dist):
					closest_idx = i
					closest_dist = dist

			dist_array.append(dist)


		print("closest_idx before", closest_idx)
		## in case robot is somewhere in the middle
		if (closest_idx != total_points-1) and (closest_idx != 0):
			
			## we check the heading of closest point to next point
			## and check the heading of robot to next point
			## these two headings should point to the same direction, so the closest point is target point
			## but if difference of these headings are too big meaning target point is next point

			closest_x = goal_poses_array[closest_idx].pose.position.x
			closest_y = goal_poses_array[closest_idx].pose.position.y
			next_closest_x = goal_poses_array[closest_idx+1].pose.position.x
			next_closest_y = goal_poses_array[closest_idx+1].pose.position.y
			closest_hdg = np.arctan2((next_closest_y-closest_y),(next_closest_x-closest_x))

			robot_hdg = np.arctan2((closest_y-robot_y),(closest_x-robot_x))

			## special case when hdg target is near 180/-180 deg, we use 360 range calculation
			if ((170 < np.degrees(closest_hdg) < 180) or (-180 < np.degrees(closest_hdg) < -170)):
				closest_hdg = closest_hdg%360
				robot_hdg = robot_hdg%360


			if (abs(closest_hdg - robot_hdg) > np.radians(120)):
				closest_idx = closest_idx+1


		## in case closest point is 0
		elif closest_idx == 0:

			closest_x = goal_poses_array[0].pose.position.x
			closest_y = goal_poses_array[0].pose.position.y
			next_closest_x = goal_poses_array[1].pose.position.x
			next_closest_y = goal_poses_array[1].pose.position.y
			closest_hdg = np.arctan2((next_closest_y-closest_y),(next_closest_x-closest_x))

			robot_hdg = np.arctan2((closest_y-robot_y),(closest_x-robot_x))

			## special case when hdg target is near 180/-180 deg, we use 360 range calculation
			if ((170 < np.degrees(closest_hdg) < 180) or (-180 < np.degrees(closest_hdg) < -170)):
				closest_hdg = closest_hdg%360
				robot_hdg = robot_hdg%360


			if (abs(closest_hdg - robot_hdg) > np.radians(120)):
				closest_idx = 1

		## in case closest point is last index
		elif closest_idx == total_points-1:

			closest_x = goal_poses_array[closest_idx].pose.position.x
			closest_y = goal_poses_array[closest_idx].pose.position.y
			next_closest_x = goal_poses_array[0].pose.position.x
			next_closest_y = goal_poses_array[0].pose.position.y
			closest_hdg = np.arctan2((next_closest_y-closest_y),(next_closest_x-closest_x))

			robot_hdg = np.arctan2((closest_y-robot_y),(closest_x-robot_x))

			## special case when hdg target is near 180/-180 deg, we use 360 range calculation
			if ((170 < np.degrees(closest_hdg) < 180) or (-180 < np.degrees(closest_hdg) < -170)):
				closest_hdg = closest_hdg%360
				robot_hdg = robot_hdg%360


			if (abs(closest_hdg - robot_hdg) > np.radians(45)):
				closest_idx = 0

		print("closest_idx after", closest_idx)
		print("closest_dist", closest_dist)
		print("dist_array", dist_array)

		if closest_idx == 0:
			return goal_poses_array, closest_idx
		else:
			tmp_goal_poses_array = []
			for j in range(total_points):
				if (closest_idx+j) < total_points:
					tmp_goal_poses_array.append(goal_poses_array[closest_idx+j])
				else:
					tmp_goal_poses_array.append(goal_poses_array[closest_idx+j-total_points])
			return tmp_goal_poses_array, closest_idx


	def reIndexingArray(self, goal_poses_array, want_first_idx):

		tmp_array = []
		total_points = len(goal_poses_array)

		for i in range(total_points):

			if ((i + want_first_idx) < total_points):
				tmp_array.append(goal_poses_array[want_first_idx+i])
			else:
				tmp_array.append(goal_poses_array[want_first_idx+i-total_points])

		return tmp_array



def main(args=None):
	rclpy.init(args=args)
	node = Nav2Controller()
	rclpy.spin(node)
	node.destroy()
	rclpy.shutdown()

if __name__ == '__main__':
	main()