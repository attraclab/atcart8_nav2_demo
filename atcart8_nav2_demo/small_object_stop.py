
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import UInt8
from geometry_msgs.msg import Twist
import numpy as np
from numpy import pi
import time
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from tf_transformations import quaternion_from_euler

class SmallObjStop(Node):

	def __init__(self):
		super().__init__('small_object_stop_node')
		self.get_logger().info('Start small_object_stop node')

		self.pub_once = True
		self.from_hold_cmd = True
		self.mid_scanline_closest = 100.0
		self.atcart_mode = 2
		self.mode_num = 2
		self.stamp_at_hold = time.time()
		self.stamp_got_closest_last = time.time()
		self.found_obj_period = 0.0
		self.vx = 0.0
		self.wz = 0.0
		self.isSpinning = False
		self.front_closest = 100.0

		qos_policy = rclpy.qos.QoSProfile(reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT, history=rclpy.qos.HistoryPolicy.KEEP_LAST, depth=1)

		self.oakd_mid_scan_sub = self.create_subscription(LaserScan, '/oakd/mid_scan', self.mid_scan_callback, qos_policy)
		self.atcart_mode_sub = self.create_subscription(UInt8, '/jmoab/atcart_mode', self.atcart_mode_callback, 10)
		self.cmd_vel_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

		self.scan_sub = self.create_subscription(LaserScan, 'scan', self.scan_callback, qos_profile=qos_policy)
		self.front_scan_pub = self.create_publisher(LaserScan, '/front_scan', qos_profile=qos_policy)
		self.front_scan_msg = LaserScan()

		self.atcart_mode_cmd_pub = self.create_publisher(UInt8, '/jmoab/atcart_mode_cmd', 10)
		self.atcart_mode_cmd_msg = UInt8()

		self.br = TransformBroadcaster(self)

		### Loop spin ###
		timer_period = 0.01
		self.timer = self.create_timer(timer_period, self.timer_callback)


	def mid_scan_callback(self, msg):

		# ranges_array = np.asarray(msg.ranges[360:920])
		ranges_array = np.asarray(msg.ranges[160:480])
		## remove zero out from array
		ranges_array = ranges_array[ranges_array != 0]
		if len(ranges_array) != 0:
			self.mid_scanline_closest = ranges_array.min()
		else:
			self.mid_scanline_closest = 0.1
		# print(ranges_array)
		# print("closest ", self.mid_scanline_closest)
		


	def atcart_mode_callback(self, msg):

		self.atcart_mode = msg.data
		# print(self.atcart_mode)

	def cmd_vel_callback(self, msg):
		self.vx = msg.linear.x
		self.wz = msg.angular.z

		if self.vx == 0 and self.wz != 0.0:
			self.isSpinning = True
		else:
			self.isSpinning = False

	def scan_callback(self, msg):
		# print("Scan publish")
		self.front_scan_msg.header.stamp = self.get_clock().now().to_msg()
		self.front_scan_msg.header.frame_id = "front_laser"	#"laser_frame"
		self.front_scan_msg.time_increment = msg.time_increment
		self.front_scan_msg.angle_increment = msg.angle_increment

		self.front_scan_msg.scan_time = msg.scan_time
		self.front_scan_msg.range_min = msg.range_min
		self.front_scan_msg.range_max = msg.range_max
		range_len = len(msg.ranges)
		oneDeg_idx = range_len/360
		scan_ang = 10.0 # +5 -5 from middle
		start_idx = int(range_len/2.0 - ((scan_ang/2.0)*oneDeg_idx)) #940
		last_idx = int(range_len/2.0 + ((scan_ang/2.0)*oneDeg_idx)) #1048
		self.front_scan_msg.angle_min = (-(scan_ang/2.0)*pi)/180.0	#((-90-self.right_wall_detect_deg)*pi)/180.0
		self.front_scan_msg.angle_max = ((scan_ang/2.0)*pi)/180.0	#((-90+self.right_wall_detect_deg)*pi)/180.0
		self.front_scan_msg.ranges = msg.ranges[start_idx:last_idx]
		self.front_scan_msg.intensities = msg.intensities[start_idx:last_idx]

		self.front_scan_pub.publish(self.front_scan_msg)

		t = TransformStamped()
		t.header.frame_id = "base_link" 
		t.header.stamp = self.get_clock().now().to_msg()
		t.child_frame_id = "front_laser"	#"base_footprint"	#"base_link"
		t.transform.translation.x = 0.0
		t.transform.translation.y = 0.0
		t.transform.translation.z = 0.22

		# q = quaternion_from_euler(tf[3],tf[4],tf[5])
		t.transform.rotation.x = 0.0
		t.transform.rotation.y = 0.0
		t.transform.rotation.z = 0.0
		t.transform.rotation.w = 1.0
		self.br.sendTransform(t)

		front_array = np.asarray(self.front_scan_msg.ranges)
		front_array = front_array[front_array != 0]

		if len(front_array) != 0:
			self.front_closest = front_array.min()
		else:
			self.front_closest = 1.0
		


	def timer_callback(self):

		self.closest_diff = abs(self.front_closest - self.mid_scanline_closest)

		## A criteria to judge whether there is small object or not
		## if the depth camera (mid_scan) could see something less than 0.8,
		## and the closest difference between main lidar and dept camera is more than 1.5m
		## it could be consider that there might be small object there
		maybe_small_obj = (self.mid_scanline_closest <= 0.8) and (self.closest_diff > 1.5)

		## in manual mode, and spinning from cmd_vel, we don't want to detect small object
		if self.atcart_mode != 1 and (not self.isSpinning):

			if maybe_small_obj:
				self.found_obj_period = time.time() - self.stamp_got_closest_last

				## if there might be small object and it passed 1.0 second
				## we will publish atcart_mode to HOLD one time
				if self.pub_once and (self.found_obj_period > 1.0):
					# print("Switch to hold")
					self.mode_num = 0
					self.atcart_mode_cmd_msg.data = 0
					self.atcart_mode_cmd_pub.publish(self.atcart_mode_cmd_msg)
					self.pub_once = False
					self.from_hold_cmd = True

				self.stamp_at_hold = time.time()

				## if the object still there, the code will just stay in this IF

				
			else:
				self.stamp_got_closest_last = time.time()
				period_of_obj_disappear = (time.time() - self.stamp_at_hold)
				
				## if object disappear for more than 2 seconds, then we continue to move to auto mode
				if self.from_hold_cmd and (period_of_obj_disappear > 2.0):
					print("period_of_obj_disappear", period_of_obj_disappear)
					# print("Switch to auto")
					self.mode_num = 2
					self.atcart_mode_cmd_msg.data = 2
					self.atcart_mode_cmd_pub.publish(self.atcart_mode_cmd_msg)
					self.pub_once = True
					self.from_hold_cmd = False

		print("mode: {:d} mid_closest: {:.2f} front_closest: {:.2f} closest_diff: {:.2f} found_obj_T: {:.2f} maybe_small_obj: {:}".format(\
			self.mode_num, self.mid_scanline_closest, self.front_closest, self.closest_diff, self.found_obj_period, maybe_small_obj))
		


def main(args=None):

	rclpy.init(args=None)
	node = SmallObjStop()
	rclpy.spin(node)
	node.destroy_node()
	rclpy.shutdown()

if __name__ == "__main__":
	main()
