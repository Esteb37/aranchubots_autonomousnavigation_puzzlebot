#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import numpy as np
from bug0 import Bug0

class BugRay(Bug0):

	def ao_condition(self):
		return self.ray_clear or (abs(self.theta_AO - self.theta_gtg) < np.pi/2 and self.progress() < abs(self.hit_distance - self.eps))

	def additional_init(self):
		self.ray_step = 0.01
		self.ray_number = int(0.19 / self.ray_step)
		self.pub_ray = rospy.Publisher('ray', Path, queue_size=1)
		self.hit_points = []
		self.ray_origins = []

	def additional_publish(self):
		points = []

		for i in range(len(self.hit_points)):
			points.append(self.ray_origins[i])
			points.append(self.hit_points[i])
			points.append(self.ray_origins[i])

		path = Path()
		path.header.frame_id = "odom"
		path.header.stamp = rospy.Time.now()
		path.poses = []

		for i, point in enumerate(points):
			path.poses.append(PoseStamped())
			path.poses[i].header.frame_id = "odom"
			path.poses[i].header.stamp = rospy.Time.now()
			path.poses[i].pose.position.x = point[0]
			path.poses[i].pose.position.y = point[1]
			path.poses[i].pose.position.z = 0
			path.poses[i].pose.orientation.x = 0
			path.poses[i].pose.orientation.y = 0
			path.poses[i].pose.orientation.z = 0
			path.poses[i].pose.orientation.w = 1

		self.pub_ray.publish(path)

	def additional_state_setup(self):
		self.ray_clear, self.hit_points = self.get_hit_points([self.pose_x, self.pose_y], [self.x_target, self.y_target], self.lidar_msg)

	def project_ray(self, position, goal, lidar):

		if not lidar:
			return True, goal

		ranges = lidar.ranges
		angle_min = lidar.angle_min
		angle_increment = lidar.angle_increment

		original_position = [self.pose_x, self.pose_y]

		goal_dist = np.sqrt((goal[0] - position[0])**2 + (goal[1] - position[1])**2)
		goal_theta = np.arctan2(goal[1] - position[1], goal[0] - position[0])

		for i in range(len(ranges)):
			angle = angle_min + i * angle_increment + self.ANGLE_OFFSET
			x = original_position[0] + ranges[i] * np.cos(angle + self.pose_theta)
			y = original_position[1] + ranges[i] * np.sin(angle + self.pose_theta)

			point_dist = np.sqrt((x - position[0])**2 + (y - position[1])**2)
			point_theta = np.arctan2(y - position[1], x - position[0])

			if point_dist < goal_dist and abs(point_theta - goal_theta) <= angle_increment * 2:
				return False, [x, y]

		return True, goal


	def get_hit_points(self, position, goal, lidar):

		theta = self.normalize_angle(np.arctan2(goal[1] - position[1], goal[0] - position[0]))
		self.ray_origins = self.choose_order(position, theta)
		ray_goals = self.choose_order(goal, theta)
		clear = True

		hit_points = [None] * self.ray_number
		for i in range(self.ray_number):
			origin_point = self.ray_origins[i]
			goal_point = ray_goals[i]
			clear_point, hit_point = self.project_ray(origin_point, goal_point, lidar)
			hit_points[i] = hit_point
			if clear_point == False:
				clear = False

		return clear, hit_points


	def get_side(self, position, distance, theta, right):
		if right:
			angle = np.pi/2
		else:
			angle = -np.pi/2

		return [position[0] + distance * np.cos(theta + angle), position[1] + distance * np.sin(theta + angle)]

	def choose_order(self, point, theta):
		a = []
		b = []
		for i in range(self.ray_number // 2):
			distance = self.ray_step * i * 2
			a.append(self.get_side(point, distance, theta, False))
			b.append(self.get_side(point, distance, theta, True))

		if a[0][0] > b[0][0]:
			right = a
			left = b
		elif a[0][0] < b[0][0]:
			right = b
			left = a
		elif a[0][1] > b[0][1]:
			right = a
			left = b
		else:
			right = b
			left = a

		return left + [point] + right

if __name__ == "__main__":
	rospy.init_node("bugray", anonymous=True)
	BugRay()