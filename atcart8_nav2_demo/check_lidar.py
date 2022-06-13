
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class Test(Node):

	def __init__(self):

		super().__init__("test_node")
		self.get_logger().info('Check lidar')

		# https://answers.ros.org/question/334649/ros2-turtlebot3-gazebo-scan-topic-subscriber-is-not-calling-the-callback-function/
		qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT, history=rclpy.qos.HistoryPolicy.KEEP_LAST, depth=1)


		self.sub = self.create_subscription(LaserScan, 'scan', self.callback, qos_profile=qos_policy)

		### Loop spin ###
		timer_period = 0.05
		self.timer = self.create_timer(timer_period, self.timer_callback)

	def callback(self, msg):

		print(len(msg.ranges))

	def timer_callback(self):
		pass



def main(args=None):

	rclpy.init(args=args)
	node = Test()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()

if __name__ == "__main__":
	main()
