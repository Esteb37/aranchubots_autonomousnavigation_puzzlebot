#!/usr/bin/env python
from bug0 import Bug0
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import numpy as np

# This class will make the puzzlebot move to a given goal
class Bug2(Bug0):

	RAY_DISTANCE = 0.1

	last_ray_hit = [0, 0]

	def ao_condition(self):
		condition = self.distance_to_line([self.pose_x, self.pose_y], self.ray_trace) < self.RAY_DISTANCE and self.progress() < abs(self.hit_distance - self.eps)
		if condition:
			self.hit_distance = self.progress()
		return condition

	def separation_condition(self):
		return True

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
		pose.pose.position.x = 0
		pose.pose.position.y = self.ray_trace[1]
		pose.pose.position.z = 0
		pose.pose.orientation.x = 0
		pose.pose.orientation.y = 0
		pose.pose.orientation.z = 0
		pose.pose.orientation.w = 1
		path.poses.append(pose)

		pose = PoseStamped()
		pose.header.stamp = rospy.Time.now()
		pose.header.frame_id = "odom"
		pose.pose.position.x = 3.2
		pose.pose.position.y = self.ray_trace[0]*3.2 + self.ray_trace[1]
		pose.pose.position.z = 0
		pose.pose.orientation.x = 0
		pose.pose.orientation.y = 0
		pose.pose.orientation.z = 0
		pose.pose.orientation.w = 1
		path.poses.append(pose)

		self.pub_ray_trace.publish(path)


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

	def goal_cb(self, msg):
		self.x_target = msg.pose.position.x
		self.y_target = msg.pose.position.y
		self.goal_received = 1
		self.current_state = "GoToGoal"
		self.hit_distance = np.inf
		self.calculate_ray()

if __name__ == "__main__":
	rospy.init_node("bug2", anonymous=True)
	Bug2()