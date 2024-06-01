#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, PoseStamped
from visualization_msgs.msg import Marker
from sensor_msgs.msg import LaserScan #Lidar
from nav_msgs.msg import Odometry
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler


# This class will make the puzzlebot move to a given goal
class Bug0():

	ANGLE_OFFSET = np.pi

	def __init__(self):
		rospy.on_shutdown(self.cleanup)

		node_name = rospy.get_name()

		############ Variables ###############
		self.eps = rospy.get_param(node_name+'/eps', 0.0) #distance to the goal to switch to the next state
		is_sim = rospy.get_param(node_name+"/is_sim", False)

		self.clockwise = False
		self.pose_x = 0.0
		self.pose_y = 0.0

		self.pose_theta = 0.0

		self.x_target = 0.0
		self.y_target = 0.0

		self.goal_received = False #flag to indicate if the goal has been received
		self.lidar_received = 0 #flag to indicate if the laser scan has been received
		self.target_position_tolerance = 0.2 #acceptable distance to the goal to declare the robot has arrived to it [m]
		self.wr = 0.0 # right wheel speed [rad/s]
		self.wl = 0.0 # left wheel speed [rad/s]
		self.odom_received = False
		self.theta_fw = 0

		self.max_v = 0.12
		self.max_w = 0.3

		self.closest_angle = 0.0 #Angle to the closest object
		self.closest_range = 0.0 #Distance to the closest object
		self.ao_distance = 0.25 # distance from closest obstacle to activate the avoid obstacle behavior [m]
		self.stop_distance = 0.1 # distance from closest obstacle to stop the robot [m]

		self.v_msg = Twist() # Robot's desired speed
		self.current_state = 'Stop' # Robot's current state
		self.theta_AO = 0.0
		self.hit_distance = np.inf

		###******* INIT PUBLISHERS *******###
		vel_topic = "/cmd_vel" if not is_sim else "puzzlebot_1/base_controller/cmd_vel"
		scan_topic = "/scan" if not is_sim else "/puzzlebot_1/scan"

		self.pub_cmd_vel = rospy.Publisher(vel_topic, Twist, queue_size=1)
		pub_theta_gtg = rospy.Publisher('theta_gtg', PoseStamped, queue_size=1)
		pub_theta_AO = rospy.Publisher('theta_AO', PoseStamped, queue_size=1)
		pub_closest_object = rospy.Publisher('closest_object', Marker, queue_size=1)
		pub_mode = rospy.Publisher('mode', Marker, queue_size=1)
		pub_theta_fw = rospy.Publisher('theta_fw', PoseStamped, queue_size=1)
		pub_goal = rospy.Publisher('goal_marker', Marker, queue_size=1)

		rospy.Subscriber(scan_topic, LaserScan, self.laser_cb)
		rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_cb)
		rospy.Subscriber("/odom", Odometry, self.odom_cb)

		self.last_closest_object = [0, 0]

		#********** INIT NODE **********###
		rate = rospy.Rate(50) #freq Hz
		rate.sleep()

		self.prev_angle = 0

		while not rospy.is_shutdown() and not self.odom_received:
			rate.sleep()

		self.additional_init()

		################ MAIN LOOP ################
		while not rospy.is_shutdown():
			self.theta_gtg = self.get_theta_gtg(self.x_target, self.y_target, self.pose_x, self.pose_y, self.pose_theta)

			if self.lidar_received:
				self.run_state_machine()

			pose_gtg = PoseStamped()
			pose_gtg.header.stamp = rospy.Time.now()
			pose_gtg.header.frame_id = "base_link"
			pose_gtg.pose.position.x = 0
			pose_gtg.pose.position.y = 0
			pose_gtg.pose.position.z = 0
			quat = quaternion_from_euler(0, 0, self.theta_gtg)
			pose_gtg.pose.orientation.x = quat[0]
			pose_gtg.pose.orientation.y = quat[1]
			pose_gtg.pose.orientation.z = quat[2]
			pose_gtg.pose.orientation.w = quat[3]

			pose_AO = PoseStamped()
			pose_AO.header.stamp = rospy.Time.now()
			pose_AO.header.frame_id = "base_link"
			pose_AO.pose.position.x = 0
			pose_AO.pose.position.y = 0
			pose_AO.pose.position.z = 0
			quat = quaternion_from_euler(0, 0, self.theta_AO)
			pose_AO.pose.orientation.x = quat[0]
			pose_AO.pose.orientation.y = quat[1]
			pose_AO.pose.orientation.z = quat[2]
			pose_AO.pose.orientation.w = quat[3]

			pose_fw = PoseStamped()
			pose_fw.header.stamp = rospy.Time.now()
			pose_fw.header.frame_id = "base_link"
			pose_fw.pose.position.x = 0
			pose_fw.pose.position.y = 0
			pose_fw.pose.position.z = 0
			quat = quaternion_from_euler(0, 0, self.theta_fw)
			pose_fw.pose.orientation.x = quat[0]
			pose_fw.pose.orientation.y = quat[1]
			pose_fw.pose.orientation.z = quat[2]
			pose_fw.pose.orientation.w = quat[3]

			marker_closest = Marker()
			marker_closest.header.frame_id = "base_link"
			marker_closest.header.stamp = rospy.Time.now()
			marker_closest.ns = "closest_object"
			marker_closest.id = 0
			marker_closest.type = Marker.CYLINDER
			marker_closest.action = Marker.ADD
			marker_closest.pose.position.x = self.closest_range * np.cos(self.closest_angle)
			marker_closest.pose.position.y = self.closest_range * np.sin(self.closest_angle)
			marker_closest.pose.position.z = 0
			marker_closest.pose.orientation.x = 0.0
			marker_closest.pose.orientation.y = 0.0
			marker_closest.pose.orientation.z = 0.0
			marker_closest.pose.orientation.w = 1.0
			marker_closest.scale.x = 0.1
			marker_closest.scale.y = 0.1
			marker_closest.scale.z = 0.3
			marker_closest.color.a = 1.0
			marker_closest.color.r = 0.0
			marker_closest.color.g = 1.0
			marker_closest.color.b = 0.0

			marker_mode = Marker()
			marker_mode.header.frame_id = "base_link"
			marker_mode.header.stamp = rospy.Time.now()
			marker_mode.ns = "mode"
			marker_mode.id = 0
			marker_mode.type = Marker.CYLINDER
			marker_mode.action = Marker.ADD
			marker_mode.scale.x = self.ao_distance*2
			marker_mode.scale.y = self.ao_distance*2
			marker_mode.scale.z = 0.01

			marker_mode.pose.position.x = 0
			marker_mode.pose.position.y = 0
			marker_mode.pose.position.z = 0
			marker_mode.pose.orientation.x = 0.0

			marker_mode.pose.orientation.y = 0.0
			marker_mode.pose.orientation.z = 0.0
			marker_mode.pose.orientation.w = 1.0

			if self.current_state == 'Stop':
				marker_mode.color.a = 1.0
				marker_mode.color.r = 0.0
				marker_mode.color.g = 1.0
				marker_mode.color.b = 0.0
			elif self.current_state == 'GoToGoal':
				marker_mode.color.a = 1.0
				marker_mode.color.r = 1.0
				marker_mode.color.g = 1.0
				marker_mode.color.b = 0.0
			elif self.current_state == 'AvoidObstacle':
				marker_mode.color.a = 1.0
				marker_mode.color.r = 0.0
				marker_mode.color.g = 0.0
				marker_mode.color.b = 1.0

			goal_marker = Marker()
			goal_marker.header.frame_id = "odom"
			goal_marker.header.stamp = rospy.Time.now()
			goal_marker.ns = "goal"
			goal_marker.id = 0
			goal_marker.type = Marker.CUBE
			goal_marker.action = Marker.ADD
			goal_marker.pose.position.x = self.x_target
			goal_marker.pose.position.y = self.y_target
			goal_marker.pose.position.z = 0.0
			goal_marker.pose.orientation.x = 0.0
			goal_marker.pose.orientation.y = 0.0
			goal_marker.pose.orientation.z = 0.0
			goal_marker.pose.orientation.w = 1.0
			goal_marker.scale.x = self.target_position_tolerance
			goal_marker.scale.y = self.target_position_tolerance
			goal_marker.scale.z = 0.01
			goal_marker.color.a = 1.0
			goal_marker.color.r = 0.0
			goal_marker.color.g = 1.0
			goal_marker.color.b = 0.0

			pub_goal.publish(goal_marker)
			pub_mode.publish(marker_mode)
			pub_closest_object.publish(marker_closest)
			pub_theta_gtg.publish(pose_gtg)
			pub_theta_AO.publish(pose_AO)
			pub_theta_fw.publish(pose_fw)
			self.pub_cmd_vel.publish(self.v_msg)

			self.additional_publish()

			rate.sleep()

	def ao_condition(self):
		return abs(self.theta_AO - self.theta_gtg) < np.pi/2 and self.progress() < abs(self.hit_distance - self.eps)

	def run_state_machine(self):
		self.closest_range, self.closest_angle = self.get_closest_object(self.lidar_msg)
		self.theta_AO = self.get_theta_AO(self.closest_angle)
		if self.current_state == 'Stop':
			if self.goal_received:
				print("Going to goal")
				self.current_state = "GoToGoal"
			self.v_msg.linear.x = 0
			self.v_msg.angular.z = 0

		elif self.current_state == 'GoToGoal':
			if self.at_goal():
				print("At goal")
				self.current_state = "Stop"
				self.goal_received = 0

			elif self.closest_range < self.stop_distance:
				print("Too close")
				self.current_state = "Stop"
				self.goal_received = 0

			elif self.closest_range < self.ao_distance:
				#current_closest_object = self.get_closest_object_pos()
				#distance = self.get_distance(self.last_closest_object, current_closest_object)

				#if distance > self.eps:
					theta_fwc = self.normalize_angle(self.theta_AO - np.pi/2)
					self.clockwise = abs(theta_fwc - self.theta_gtg)<=np.pi/2
					self.current_state = "AvoidObstacle"
					self.hit_distance = self.progress()
					if self.clockwise:
						print("AvoidObstacleClockwise")
					else:
						print("AvoidObstacleCounter")
			else:
				v_gtg, w_gtg = self.compute_gtg_control(self.x_target, self.y_target, self.pose_x, self.pose_y, self.pose_theta)
				self.v_msg.linear.x = v_gtg
				self.v_msg.angular.z = w_gtg

		elif self.current_state == 'AvoidObstacle':
			if self.ao_condition():
				self.last_closest_object = self.get_closest_object_pos()
				self.current_state = "GoToGoal"
				print("Going to goal")

			if self.at_goal():
				print("At goal")
				self.current_state = "Stop"
				self.goal_received = 0

			elif self.closest_range < self.stop_distance:
				print("Too close")
				self.current_state = "Stop"
				self.goal_received = 0

			else:
				current_closest_object = self.get_closest_object_pos()
				distance = self.get_distance(self.last_closest_object, current_closest_object)
				if distance > 0.35 and abs(self.closest_angle - self.prev_angle) > np.pi / 3 * 2:
					theta_fwc = self.normalize_angle(self.theta_AO - np.pi/2)
					self.clockwise = abs(theta_fwc - self.theta_gtg)<=np.pi/2
					print("Jump")

				self.last_closest_object = self.get_closest_object_pos()
				v_ao, w_ao = self.compute_fw_control(self.closest_angle, self.clockwise)
				self.v_msg.linear.x = v_ao
				self.v_msg.angular.z = w_ao

		self.prev_angle = self.closest_angle

	def at_goal(self):
		return np.sqrt((self.x_target-self.pose_x)**2 + (self.y_target-self.pose_y)**2) < self.target_position_tolerance

	def get_closest_object(self, lidar_msg):
		# This function returns the closest object to the robot
		# This functions receives a ROS LaserScan message and returns the distance and direction to the closest object
		ranges = np.array(lidar_msg.ranges)
		angle_min = lidar_msg.angle_min
		increment = lidar_msg.angle_increment

		# Fill from second to fifth sixths with np.inf
		sixth = len(ranges) // 6
		front_ranges = ranges.copy()
		front_ranges[sixth:sixth*5] = np.inf
		front_closest = np.min(front_ranges)
		if front_closest < self.ao_distance:
			min_idx = np.argmin(front_ranges)
		else:
			min_idx = np.argmin(ranges)

		closest_distance = ranges[min_idx]
		closest_angle = min_idx * increment + angle_min + self.ANGLE_OFFSET
		return closest_distance, closest_angle

	def get_theta_gtg(self, x_target, y_target, x_robot, y_robot, theta_robot):
		theta_target = np.arctan2(y_target - y_robot, x_target - x_robot)
		return self.normalize_angle(theta_target - theta_robot)


	def compute_gtg_control(self, x_target, y_target, x_robot, y_robot, theta_robot):
		# This function returns the linear and angular speed to reach a given goal
		kvmax = 1.0 # linear speed maximum gain
		kwmax = 1.0  # angular angular speed maximum gain
		av = 2.0  # Constant to adjust the exponential's growth rate
		aw = 2.0  # Constant to adjust the exponential's growth rate
		ed = np.sqrt((x_target - x_robot)**2 + (y_target - y_robot)**2)
		# Compute angle to the target position
		e_theta = self.get_theta_gtg(x_target, y_target, x_robot, y_robot, theta_robot)
		# Compute the robot's angular speed
		kw = kwmax * (1 - np.exp(-aw * e_theta**2)) / abs(e_theta)
		w = kw * e_theta
		w = np.clip(w, -self.max_w, self.max_w)

		if abs(e_theta) > np.pi/8:
			# we first turn to the goal
			v = 0  # linear speed
		else:
			# Make the linear speed gain proportional to the distance to the target position
			kv = kvmax * (1 - np.exp(-av * ed**2)) / abs(ed)
			v = kv * ed  # linear speed
			v = np.clip(v, -self.max_v, self.max_v)

		return v, w

	def compute_fw_control(self, closest_angle, clockwise):
		kAO = 2.0  # Proportional constant for the angular speed controller
		closest_angle = self.normalize_angle(closest_angle)

		if clockwise:
			theta_fw = self.get_theta_AO(closest_angle) - np.pi/2
		else:
			theta_fw = self.get_theta_AO(closest_angle) + np.pi/2

		theta_fw = self.normalize_angle(theta_fw)

		# Modify angular speed calculation
		w = kAO * theta_fw
		w = np.clip(w, -self.max_w, self.max_w)

		# Modify linear speed based on the turning angle
		max_v = self.max_v  # Assuming self.max_v is defined as the maximum linear velocity
		angle_threshold = np.pi / 6  # Threshold angle for reducing speed more sharply
		scaling_factor = 10  # Factor to increase the steepness of the scaling

		if abs(theta_fw) > angle_threshold:
			v = max_v / (scaling_factor * (abs(theta_fw) + 1))
		else:
			v = max_v * (1 - (abs(theta_fw) / angle_threshold) ** 2)

		self.theta_fw = theta_fw
		return v, w


	def get_theta_AO(self, closest_angle):
		return self.normalize_angle(closest_angle - np.pi)

	def normalize_angle(self, angle):
		return np.arctan2(np.sin(angle), np.cos(angle))

	def progress(self):
		dx = self.x_target - self.pose_x
		dy = self.y_target - self.pose_y
		distance = np.sqrt(dx**2 + dy**2)
		return distance

	def laser_cb(self, msg):
		self.lidar_msg = msg
		self.lidar_received = 1

	def goal_cb(self, msg):
		self.x_target = msg.pose.position.x
		self.y_target = msg.pose.position.y
		self.goal_received = 1
		self.hit_distance = np.inf


	def cleanup(self):
		# This function is called just before finishing the node
		# You can use it to clean things up before leaving
		# Example: stop the robot before finishing a node.
		vel_msg = Twist()
		self.pub_cmd_vel.publish(vel_msg)

	def odom_cb(self, msg):
		self.pose_x = msg.pose.pose.position.x
		self.pose_y = msg.pose.pose.position.y
		quat = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
		# Convert the orientation to Euler angles
		_, _, yaw = euler_from_quaternion(quat)
		self.odom_received = True
		self.pose_theta = yaw

	def get_distance(self, a, b):
		x1 = a[0]
		y1 = a[1]
		x2 = b[0]
		y2 = b[1]
		return np.sqrt((x1-x2)**2 + (y1-y2)**2)

	def additional_init(self):
		pass

	def additional_publish(self):
		pass

	def get_closest_object_pos(self):
		return [self.pose_x + self.closest_range * np.cos(self.closest_angle), self.pose_y + self.closest_range * np.sin(self.closest_angle)]

if __name__ == "__main__":
	rospy.init_node("bug0", anonymous=True)
	Bug0()