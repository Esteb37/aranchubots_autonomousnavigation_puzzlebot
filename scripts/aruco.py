#!/usr/bin/env python
import rospy
from fiducial_msgs.msg import FiducialTransformArray
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import Quaternion, Vector3
import numpy as np
import math
import tf.transformations as tft

class Aruco():
	def __init__(self):
		rospy.init_node('aruco', anonymous=True)

		rospy.Subscriber("/fiducial_transforms", FiducialTransformArray, self.fiducial_callback)

		self.position = rospy.Publisher('/aruco', Float32MultiArray, queue_size=1)

		self.marker_id = 0.0
		self.translation = Vector3()
		self.rotation = Quaternion()
		self.image_error = 0.0
		self.object_error = 0.0
		self.fiducial_area = 0.0

		position_msg = Float32MultiArray()

		rate = rospy.Rate(50)
		while not rospy.is_shutdown():
			if self.marker_id != -1:
				position_robot = self.transform_to_robot_frame()
				distance, angle = self.calculate_distance_and_angle(position_robot)
				position_msg.data = [distance, angle, float(self.marker_id)]
				self.position.publish(position_msg)

			self.marker_id = -1
			rate.sleep()


	def transform_to_robot_frame(self):
		T_cam_to_robot = np.array([[0, 0, 1, 0.07],
								   [1, 0, 0, 0.0],
								   [0, 1, 0, 0.09],
								   [0, 0, 0, 1]])
		position_homogeneous = np.array([self.translation.x, self.translation.y, self.translation.z, 1.0])
		T_final = np.dot(T_cam_to_robot, position_homogeneous)
		return T_final[:3]


	def calculate_distance_and_angle(self, position_robot):
		distance = math.sqrt(position_robot[0]**2 + position_robot[1]**2 + position_robot[2]**2)
		angle = - math.atan2(position_robot[1], position_robot[0])
		angle = self.normalize_angle(angle)
		return distance, angle


	def fiducial_callback(self, data):
		if len(data.transforms) != 0:
			transform = data.transforms[0]

			self.marker_id = transform.fiducial_id
			self.translation = transform.transform.translation
			self.rotation = transform.transform.rotation
			self.image_error = transform.image_error
			self.object_error = transform.object_error
			self.fiducial_area = transform.fiducial_area

	def normalize_angle(self, angle):
		return np.arctan2(np.sin(angle), np.cos(angle))

if __name__ == '__main__':
	Aruco()
