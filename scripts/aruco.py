#!/usr/bin/env python
import rospy
from fiducial_msgs.msg import FiducialTransformArray
import numpy as np
import math
import tf

assigned_marker = 703


def fiducial_callback(data):
	for transform in data.transforms:
		marker_id = transform.fiducial_id

		if (marker_id == assigned_marker):
			translation = transform.transform.translation
			rotation = transform.transform.rotation
			image_error = transform.image_error
			object_error = transform.object_error
			fiducial_area = transform.fiducial_area

			tf_marker_to_camera = tf.TransformerROS()
			tf_marker_to_camera.setTransform(tf.TransformerROS.msgToTransform(transform.transform))
			tf_camera_to_base_link = tf.TransformerROS()
			# get the transfrom from camera to base_link from tf
			tf_camera_to_base_link = tf_camera_to_base_link.lookupTransform("base_link", "camera_link", rospy.Time(0))
			tf_marker_to_base_link = tf_marker_to_camera * tf_camera_to_base_link
			x,y,z = tf_marker_to_base_link.getOrigin()
			print("x: ", x)
			print("y: ", y)
			print("z: ", z)

			break








def listener():
	rospy.init_node('fiducial_listener', anonymous=True)
	rospy.Subscriber("/fiducial_transforms", FiducialTransformArray, fiducial_callback)
	rospy.spin()


if __name__ == '__main__':
	listener()
