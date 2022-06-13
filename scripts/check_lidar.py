
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan

class Test(Node):

	def __init__(self):

		super().__init__("test_node")
		self.get_logger().info('Check lidar')
		self.sub = self.create_subscription(LaserScan, 'scan', self.callback, 10)

		### Loop spin ###
		timer_period = 0.05
		self.timer = self.create_timer(timer_period, self.timer_callback)

	def callback(self, msg):

		print(msg)

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
