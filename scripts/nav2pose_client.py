
import time
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Duration
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from lifecycle_msgs.srv import GetState

class Nav2Pose(Node):

	def __init__(self):
		super().__init__('nav2pose_client')
		self.get_logger().info('Start nav2pose_client node')

		self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

		amcl_pose_qos = QoSProfile(
									durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
									reliability=QoSReliabilityPolicy.RELIABLE,
									history=QoSHistoryPolicy.KEEP_LAST,
									depth=1)


		# self.localization_pose_sub = self.create_subscription(PoseWithCovarianceStamped, 'amcl_pose', self._amclPoseCallback, amcl_pose_qos)
		# self.initial_pose_received = False

		self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
		self.initial_pose = PoseWithCovarianceStamped()
		self.initial_pose.header.frame_id = 'map'
		self.initial_pose.header.stamp = self.get_clock().now().to_msg()
		self.initial_pose.pose.pose.position.x = 0.0
		self.initial_pose.pose.pose.position.y = 0.0
		self.initial_pose.pose.pose.orientation.z = 0.0
		self.initial_pose.pose.pose.orientation.w = 1.0

		self.goal_point_sub = self.create_subscription(PoseStamped, 'target_point', self.goal_point_callback, 1)
		self.cancel_sub = self.create_subscription(Bool, 'cancel_task', self.cancel_task_callback, 1)
		self.init_pose_sub = self.create_subscription(Bool, 'set_init_pose', self.set_init_pose_callback, 1)

		self.got_goal = False

		# self._waitForNodeToActivate('amcl')
		# self._waitForNodeToActivate('bt_navigator')

		self.start_stamp = time.time()
		self.last_print_stamp = time.time()
		self.once = True
		self.timer = self.create_timer(0.02, self.timer_callback)

	##############################
	### Nav2Pose action client ###
	##############################
	def send_goal(self, pose):

		self.get_logger().info("Waiting for 'NavigateToPose' action server")

		while not self.nav_to_pose_client.wait_for_server(timeout_sec=1.0):
			self.get_logger().info("'NavigateToPose' action server not available, waiting...")

		self.got_goal = True
		goal_msg = NavigateToPose.Goal()
		goal_msg.pose = pose
		goal_msg.behavior_tree = ''

		self.get_logger().info("Send goal pose")
		send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg, self._feedbackCallback)
		
		send_goal_future.add_done_callback(self._goal_response_callback)
		# rclpy.spin_until_future_complete(self, send_goal_future)
		# self.goal_handle = send_goal_future.result()

		# self.get_logger().info("complete")

		# if not self.goal_handle.accepted:
		# 	self.get_logger().error('Goal to ' + str(pose.pose.position.x) + ' ' + mstr(pose.pose.position.y) + ' was rejected!')
		# 	return False

		# self.result_future = self.goal_handle.get_result_async()

	def _goal_response_callback(self, future):
		goal_handle = future.result()
		if not goal_handle.accepted:
			self.get_logger().info('Goal rejected :(')
			return

		self.get_logger().info('Goal accepted :)')

		self._goal_handle = goal_handle

		self.get_result_future = goal_handle.get_result_async()
		self.get_result_future.add_done_callback(self._get_result_callback)

	def _get_result_callback(self, future):
		result = future.result().result
		# self.get_logger().info('Result: {0}'.format(result.result))

	def _feedbackCallback(self, msg):
		# self.get_logger().info('Received action feedback message')
		self.feedback = msg.feedback
		return

	def _cancel_done(self, future):
		cancel_response = future.result()
		
		if len(cancel_response.goals_canceling) > 0:
			self.get_logger().info('Goal successfully canceled')
		else:
			self.get_logger().info('Goal failed to cancel')

	###############################
	### ros topics and callback ###
	###############################
	def _waitForNodeToActivate(self, node_name):
		# Waits for the node within the tester namespace to become active
		self.get_logger().info(f'Waiting for {node_name} to become active..')
		node_service = f'{node_name}/get_state'
		state_client = self.create_client(GetState, node_service)
		while not state_client.wait_for_service(timeout_sec=1.0):
			self.get_logger().info(f'{node_service} service not available, waiting...')

		req = GetState.Request()
		state = 'unknown'
		while state != 'active':
			self.get_logger().info(f'Getting {node_name} state...')
			future = state_client.call_async(req)
			rclpy.spin_until_future_complete(self, future)
			if future.result() is not None:
				state = future.result().current_state.label
				self.get_logger().info(f'Result of get_state: {state}')
			time.sleep(2)
		return

	def set_init_pose_callback(self, msg):

		if msg.data == True:
			self.set_initial_pose()

	def cancel_task_callback(self, msg):

		if msg.data == True:
			if self.got_goal:
				self.get_logger().info("Try cancel task")
				future = self._goal_handle.cancel_goal_async()
				future.add_done_callback(self._cancel_done)
				self.got_goal = False

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

		self.send_goal(goal_pose)

	def _amclPoseCallback(self, msg):
		self.get_logger().info('Received amcl pose')
		self.initial_pose_received = True

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


	############
	### Loop ###
	############
	def timer_callback(self):

		## this loop is for your code to do something else

		
		if (time.time() - self.last_print_stamp) > 1.0:
			## if you want to print somehting in 1Hz put it here

			# self.get_logger().info("period: {:.2f}".format(self.period))
			self.last_print_stamp = time.time()
			


def main(args=None):
	rclpy.init(args=args)
	node = Nav2Pose()
	rclpy.spin(node)
	node.destroy()
	rclpy.shutdown()

if __name__ == '__main__':
	main()