#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, PoseStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import Bool
from sensor_msgs.msg import LaserScan #Lidar
from nav_msgs.msg import Odometry, Path
import numpy as np
from tf.transformations import euler_from_quaternion, quaternion_from_euler


# This class will make the puzzlebot move to a given goal
class AutonomousNav():

	ANGLE_OFFSET = 0

	def __init__(self):
		rospy.on_shutdown(self.cleanup)

		############ Variables ###############
		eps = rospy.get_param('/bug2/eps', 0.0) #distance to the goal to switch to the next state
		clockwise = False

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

		closest_angle = 0.0 #Angle to the closest object
		closest_range = 0.0 #Distance to the closest object
		ao_distance = 0.23 # distance from closest obstacle to activate the avoid obstacle behavior [m]
		stop_distance = 0.1 # distance from closest obstacle to stop the robot [m]

		v_msg = Twist() # Robot's desired speed
		current_state = 'Stop' # Robot's current state
		theta_AO = 0.0
		hit_distance = 0

		###******* INIT PUBLISHERS *******###
		self.pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
		pub_theta_gtg = rospy.Publisher('theta_gtg', PoseStamped, queue_size=1)
		pub_theta_AO = rospy.Publisher('theta_AO', PoseStamped, queue_size=1)
		pub_closest_object = rospy.Publisher('closest_object', Marker, queue_size=1)
		pub_ray_trace = rospy.Publisher('ray_trace', Path, queue_size=1)
		pub_mode = rospy.Publisher('mode', Marker, queue_size=1)

		rospy.Subscriber("/scan", LaserScan, self.laser_cb)
		rospy.Subscriber("/move_base_simple/goal", PoseStamped, self.goal_cb)
		rospy.Subscriber("/odom", Odometry, self.odom_cb)


		#********** INIT NODE **********###
		freq = 50
		rate = rospy.Rate(freq) #freq Hz
		rate.sleep()

		prev_angle = 0

		while not rospy.is_shutdown() and not self.odom_received:
			rate.sleep()

		self.calculate_ray()

		################ MAIN LOOP ################
		while not rospy.is_shutdown():
			theta_gtg = self.get_theta_gtg(self.x_target, self.y_target, self.pose_x, self.pose_y, self.pose_theta)

			if self.lidar_received:
				closest_range, closest_angle = self.get_closest_object(self.lidar_msg)
				theta_AO = self.get_theta_AO(closest_angle)
				if current_state == 'Stop':
					if self.goal_received:
						print("Going to goal")
						current_state = "GoToGoal"
					v_msg.linear.x = 0
					v_msg.angular.z = 0

				elif current_state == 'GoToGoal':
					if self.at_goal():
						print("At goal")
						current_state = "Stop"
						self.goal_received = 0

					elif closest_range < stop_distance:
						print("Too close")
						current_state = "Stop"

					elif closest_range < ao_distance:
						theta_fwc = self.normalize_angle(theta_AO - np.pi/2)
						clockwise = abs(theta_fwc - theta_gtg)<=np.pi/2
						current_state = "AvoidObstacle"
						hit_distance = self.progress()
						if clockwise:
							print("AvoidObstacleClockwise")
						else:
							print("AvoidObstacleCounter")
					else:
						v_gtg, w_gtg = self.compute_gtg_control(self.x_target, self.y_target, self.pose_x, self.pose_y, self.pose_theta)
						v_msg.linear.x = v_gtg
						v_msg.angular.z = w_gtg

				elif current_state == 'AvoidObstacle':
					if self.distance_to_line([self.pose_x, self.pose_y], self.ray_trace) < 0.05 and self.progress() < abs(hit_distance - eps):
						current_state = "GoToGoal"
						print("Going to goal")

					if self.at_goal():
						print("At goal")
						current_state = "Stop"
						self.goal_received = 0

					elif closest_range < stop_distance:
						print("Too close")
						current_state = "Stop"
					else:
						# If the closest object suddenly jumps to the other side of the corridor, recalculate direction
						if abs(prev_angle - closest_angle) > np.pi / 4 * 3:
							theta_fwc = self.normalize_angle(theta_AO - np.pi/2)
							clockwise = abs(theta_fwc - theta_gtg)<=np.pi/2
							print("Jump")

						v_ao, w_ao = self.compute_fw_control(closest_angle, clockwise)
						v_msg.linear.x = v_ao
						v_msg.angular.z = w_ao

				prev_angle = closest_angle

			pose_gtg = PoseStamped()
			pose_gtg.header.stamp = rospy.Time.now()
			pose_gtg.header.frame_id = "base_link"
			pose_gtg.pose.position.x = 0
			pose_gtg.pose.position.y = 0
			pose_gtg.pose.position.z = 0
			quat = quaternion_from_euler(0, 0, theta_gtg)
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
			quat = quaternion_from_euler(0, 0, theta_AO)
			pose_AO.pose.orientation.x = quat[0]
			pose_AO.pose.orientation.y = quat[1]
			pose_AO.pose.orientation.z = quat[2]
			pose_AO.pose.orientation.w = quat[3]

			marker_closest = Marker()
			marker_closest.header.frame_id = "base_link"
			marker_closest.header.stamp = rospy.Time.now()
			marker_closest.ns = "closest_object"
			marker_closest.id = 0
			marker_closest.type = Marker.CYLINDER
			marker_closest.action = Marker.ADD
			marker_closest.pose.position.x = closest_range * np.cos(closest_angle)
			marker_closest.pose.position.y = closest_range * np.sin(closest_angle)
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
			marker_mode.scale.x = ao_distance*2
			marker_mode.scale.y = ao_distance*2
			marker_mode.scale.z = 0.01

			marker_mode.pose.position.x = 0
			marker_mode.pose.position.y = 0
			marker_mode.pose.position.z = 0
			marker_mode.pose.orientation.x = 0.0

			marker_mode.pose.orientation.y = 0.0
			marker_mode.pose.orientation.z = 0.0
			marker_mode.pose.orientation.w = 1.0

			marker_mode.color.a = 1.0
			marker_mode.color.r = 1.0 if current_state == 'GoToGoal' else 0.0
			marker_mode.color.g = 1.0 if current_state == 'GoToGoal' else 0.0
			marker_mode.color.b = 0.0 if current_state == 'GoToGoal' else 1.0

			path = Path()
			path.header.stamp = rospy.Time.now()
			path.header.frame_id = "odom"
			path.poses = []

			pose = PoseStamped()
			pose.header.stamp = rospy.Time.now()
			pose.header.frame_id = "odom"
			pose.pose.position.x = self.x_target
			pose.pose.position.y = self.y_target
			pose.pose.position.z = 0
			pose.pose.orientation.x = 0
			pose.pose.orientation.y = 0
			pose.pose.orientation.z = 0
			pose.pose.orientation.w = 1
			path.poses.append(pose)

			pose = PoseStamped()
			pose.header.stamp = rospy.Time.now()
			pose.header.frame_id = "odom"
			pose.pose.position.x = self.start_x
			pose.pose.position.y = self.start_y
			pose.pose.position.z = 0
			pose.pose.orientation.x = 0
			pose.pose.orientation.y = 0
			pose.pose.orientation.z = 0
			pose.pose.orientation.w = 1
			path.poses.append(pose)

			pub_ray_trace.publish(path)
			pub_mode.publish(marker_mode)
			pub_closest_object.publish(marker_closest)
			pub_theta_gtg.publish(pose_gtg)
			pub_theta_AO.publish(pose_AO)
			self.pub_cmd_vel.publish(v_msg)
			rate.sleep()

	def at_goal(self):
		return np.sqrt((self.x_target-self.pose_x)**2 + (self.y_target-self.pose_y)**2) < self.target_position_tolerance

	def get_closest_object(self, lidar_msg):
		# This function returns the closest object to the robot
		# This functions receives a ROS LaserScan message and returns the distance and direction to the closest object
		ranges = np.array(lidar_msg.ranges)
		angle_min = lidar_msg.angle_min + self.ANGLE_OFFSET
		min_idx = np.argmin(ranges)
		closest_range = ranges[min_idx]
		closest_angle = angle_min + min_idx * lidar_msg.angle_increment
		# limit the angle to [-pi, pi]
		closest_angle = np.arctan2(np.sin(closest_angle), np.cos(closest_angle))
		return closest_range, closest_angle

	def get_theta_gtg(self, x_target, y_target, x_robot, y_robot, theta_robot):
		theta_target = np.arctan2(y_target - y_robot, x_target - x_robot)
		return self.normalize_angle(theta_target - theta_robot)


	def compute_gtg_control(self, x_target, y_target, x_robot, y_robot, theta_robot):
		# This function returns the linear and angular speed to reach a given goal
		kvmax = 0.15  # linear speed maximum gain
		kwmax = 0.5  # angular angular speed maximum gain
		av = 2.0  # Constant to adjust the exponential's growth rate
		aw = 2.0  # Constant to adjust the exponential's growth rate
		ed = np.sqrt((x_target - x_robot)**2 + (y_target - y_robot)**2)
		# Compute angle to the target position
		e_theta = self.get_theta_gtg(x_target, y_target, x_robot, y_robot, theta_robot)
		# Compute the robot's angular speed
		kw = kwmax * (1 - np.exp(-aw * e_theta**2)) / abs(e_theta)
		w = kw * e_theta

		if abs(e_theta) > np.pi/8:
			# we first turn to the goal
			v = 0  # linear speed
		else:
			# Make the linear speed gain proportional to the distance to the target position
			kv = kvmax * (1 - np.exp(-av * ed**2)) / abs(ed)
			v = kv * ed  # linear speed

		return v, w

	def compute_fw_control(self, closest_angle, clockwise):
		kAO = 1.5 # Proportional constant for the angular speed controller
		closest_angle = self.normalize_angle(closest_angle)
		if clockwise:
			theta_fw = self.get_theta_AO(closest_angle) - np.pi/2
		else:
			theta_fw = self.get_theta_AO(closest_angle) + np.pi/2
		theta_fw = self.normalize_angle(theta_fw)

		w = kAO * theta_fw

		if abs(theta_fw) > np.pi / 2:
			v = 0
		else:
			v = 0.17

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
		self.calculate_ray()

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

	def get_ray_trace(self, a, b):
		x1 = a[0]
		y1 = a[1]
		x2 = b[0]
		y2 = b[1]
		# avoid division by zero
		if x1 == x2:
			x2 += 0.0001
		m = (y2-y1)/(x2-x1)
		b = y1 - m*x1
		return m, b

	def distance_to_line(self, point, line):
		m = line[0]
		b = line[1]
		x0 = point[0]
		y0 = point[1]
		return abs(m*x0 - y0 + b)/np.sqrt(m**2 + 1)

	def calculate_ray(self):
		self.start_x = self.pose_x
		self.start_y = self.pose_y
		self.ray_trace = self.get_ray_trace([self.start_x, self.start_y], [self.x_target, self.y_target])


############################### MAIN PROGRAM ####################################
if __name__ == "__main__":
	rospy.init_node("go_to_goal_with_obstacles1", anonymous=True)
	AutonomousNav()
