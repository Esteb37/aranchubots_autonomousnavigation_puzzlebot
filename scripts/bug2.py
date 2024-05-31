#!/usr/bin/env python
from bug import BugBase
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import numpy as np

# This class will make the puzzlebot move to a given goal
class AutonomousNav(BugBase):


	def additional_init(self):
		self.pub_ray_trace = rospy.Publisher('ray_trace', Path, queue_size=1)

		self.calculate_ray()

	def additional_publish(self):
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

		self.pub_ray_trace.publish(path)

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

			elif self.closest_range < self.ao_distance:
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
			if self.distance_to_line([self.pose_x, self.pose_y], self.ray_trace) < 0.05 and self.progress() < abs(self.hit_distance - self.eps):
				self.current_state = "GoToGoal"
				print("Going to goal")

			if self.at_goal():
				print("At goal")
				self.current_state = "Stop"
				self.goal_received = 0

			elif self.closest_range < self.stop_distance:
				print("Too close")
				self.current_state = "Stop"
			else:
				# If the closest object suddenly jumps to the other side of the corridor, recalculate direction
				if abs(self.prev_angle - self.closest_angle) > np.pi / 4 * 3:
					theta_fwc = self.normalize_angle(self.theta_AO - np.pi/2)
					self.clockwise = abs(theta_fwc - self.theta_gtg)<=np.pi/2
					print("Jump")

				v_ao, w_ao = self.compute_fw_control(self.closest_angle, self.clockwise)
				self.v_msg.linear.x = v_ao
				self.v_msg.angular.z = w_ao

		self.prev_angle = self.closest_angle

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