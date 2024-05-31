#!/usr/bin/env python
import rospy
import numpy as np
from bug import BugBase

# This class will make the puzzlebot move to a given goal
class AutonomousNav(BugBase):
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
			if abs(self.theta_AO - self.theta_gtg) < np.pi/2 and self.progress() < abs(self.hit_distance - self.eps):
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


############################### MAIN PROGRAM ####################################
if __name__ == "__main__":
	rospy.init_node("go_to_goal_with_obstacles1", anonymous=True)
	AutonomousNav()
