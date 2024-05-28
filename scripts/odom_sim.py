#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from tf.transformations import quaternion_from_euler
import tf2_ros
from geometry_msgs.msg import TransformStamped
import numpy as np
from utils.KF import KalmanFilter
from utils.aruco_markers import MarkerLocations

np.set_printoptions(suppress=True)
np.set_printoptions(formatter={'float': '{: 0.4f}'.format})

#This class will do the following:
# subscribe to the /cmd_vel topic
# publish the simulated pose of the robot to /pose_sim topic
# publish to /wr and /wl the simulated wheel speed.
class PuzzlebotLocClass():
	def __init__(self):
		rospy.init_node('localisation')

		# Subscribers
		############################### SUBSCRIBERS #####################################
		rospy.Subscriber("/puzzlebot_1/wl", Float32, self.wl_cb)
		rospy.Subscriber("/puzzlebot_1/wr", Float32, self.wr_cb)
		rospy.Subscriber("/aruco", Float32, self.aruco_cb)

		x_target = rospy.get_param('/odom_node/goal_x', 0)
		y_target = rospy.get_param('/odom_node/goal_y', 0)

		x_init = rospy.get_param('/odom_node/pos_x', 0)
		y_init = rospy.get_param('/odom_node/pos_y', 0)
		theta_init = rospy.get_param('/odom_node/pos_theta', 0)

		# Publishers
		self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=1)
		self.goal_pub = rospy.Publisher('/goal_marker', Marker, queue_size=1)
		self.tf_broadcaster = tf2_ros.TransformBroadcaster()

		############ ROBOT CONSTANTS ################
		R = 0.05 #puzzlebot wheel radius [m]
		L = 0.19 #puzzlebot wheel separation [m]
		DT = 0.02 # Desired time to update the robot's pose [s]
		KR = 20.0
		KL = 30.0

		############ Variables ###############
		mu = np.array([x_init, y_init, theta_init])
		sigma = np.zeros((3,3))

		Qk = np.zeros((3,3))
		Rk = np.zeros((2,2))
		covariance = np.zeros((2,3))
		jacobian = np.zeros((3,2))

		self.wl = 0.0
		self.wr = 0.0
		self.aruco_id = 0
		self.z_i = np.zeros(2)
		self.m_i = np.zeros(2)

		KF = KalmanFilter(DT, mu)

		rate = rospy.Rate(int(1/DT))
		while not rospy.is_shutdown():

			WR = self.wr
			WL = self.wl
			mi = self.m_i
			zi = self.z_i

			V = R * (WR + WL)/2
			W = R * (WR - WL)/L

			theta = mu[2]

			covariance = np.array([[KR * np.abs(WR),			  0],
                          		   [0,				KL * np.abs(WL)]])

			jacobian = (0.5 * R * DT) * np.array([[np.cos(theta), np.cos(theta)],
												  [np.sin(theta), np.sin(theta)],
												  [2/L,			 -2/L		   ]])

			Qk = jacobian.dot(covariance).dot(jacobian.T)

			KF.predict([V, W], Qk)

			if self.aruco_id > 0:
				mu, sigma = KF.correct(mi, zi, Rk)
			else:
				mu, sigma = KF.step()

			self.aruco_id = 0

			self.publish_odom(mu, sigma, [V, W])

			self.publish_transforms(mu)

			self.publish_goal_marker(x_target, y_target)

			rate.sleep()

	def wl_cb(self, msg):
		self.wl = msg.data

	def wr_cb(self, msg):
		self.wr = msg.data


	def aruco_cb(self, msg):
		self.aruco_id = int(msg.data[2])
		self.z_i = msg.data[0:2]
		self.m_i = MarkerLocations[self.aruco_id]

	def publish_odom(self, pose, sigma, speed):
		#This functions receives the robot speed v [m/s] and w [rad/s]
		# and gets the robot's pose (x, y, theta) where (x,y) are in [m] and (theta) in [rad]
		# is the orientation

		odometry = Odometry()

		odometry.header.stamp = rospy.Time.now()
		odometry.header.frame_id = "odom"
		odometry.child_frame_id = "base_link"

		# Fill the pose information
		odometry.pose.pose.position.x = pose[0]
		odometry.pose.pose.position.y = pose[1]
		odometry.pose.pose.position.z = 0.0

		quat = quaternion_from_euler(0,0,pose[2])

		odometry.pose.pose.orientation.x = quat[0]
		odometry.pose.pose.orientation.y = quat[1]
		odometry.pose.pose.orientation.z = quat[2]
		odometry.pose.pose.orientation.w = quat[3]

		odometry.twist.twist.linear.x = speed[0]
		odometry.twist.twist.angular.z = speed[1]

		# Init a 36 elements array
		odometry.pose.covariance = [0.0] * 36

		# Fill the 3D covariance matrix
		odometry.pose.covariance[0] = sigma[0][0]
		odometry.pose.covariance[1] = sigma[0][1]
		odometry.pose.covariance[5] = sigma[0][2]
		odometry.pose.covariance[6] = sigma[1][0]
		odometry.pose.covariance[7] = sigma[1][1]
		odometry.pose.covariance[11] = sigma[1][2]
		odometry.pose.covariance[30] = sigma[2][0]
		odometry.pose.covariance[31] = sigma[2][1]
		odometry.pose.covariance[35] = sigma[2][2]

		self.odom_pub.publish(odometry)

	def publish_transforms(self, pose):

		t = TransformStamped()

		t.header.stamp = rospy.Time.now()
		t.header.frame_id = "odom"
		t.child_frame_id = "base_link"
		t.transform.translation.x = pose[0]
		t.transform.translation.y = pose[1]
		t.transform.translation.z = 0.0

		quat = quaternion_from_euler(0,0,pose[2])

		t.transform.rotation.x = quat[0]
		t.transform.rotation.y = quat[1]
		t.transform.rotation.z = quat[2]
		t.transform.rotation.w = quat[3]

		self.tf_broadcaster.sendTransform(t)

		# map to odom transform at 0,0
		t.header.stamp = rospy.Time.now()
		t.header.frame_id = "map"
		t.child_frame_id = "odom"
		t.transform.translation.x = 0
		t.transform.translation.y = 0
		t.transform.translation.z = 0.0

		t.transform.rotation.x = 0
		t.transform.rotation.y = 0
		t.transform.rotation.z = 0
		t.transform.rotation.w = 1

		self.tf_broadcaster.sendTransform(t)

	def publish_goal_marker(self, x, y):
		marker_goal = Marker()
		marker_goal.header.frame_id = "odom"
		marker_goal.header.stamp = rospy.Time.now()
		marker_goal.ns = "goal_marker"
		marker_goal.id = 0
		marker_goal.type = Marker.CUBE
		marker_goal.action = Marker.ADD
		marker_goal.pose.position.x = x
		marker_goal.pose.position.y = y
		marker_goal.pose.position.z = 0
		marker_goal.pose.orientation.x = 0.0
		marker_goal.pose.orientation.y = 0.0
		marker_goal.pose.orientation.z = 0.0
		marker_goal.pose.orientation.w = 1.0
		marker_goal.scale.x = 0.2
		marker_goal.scale.y = 0.2
		marker_goal.scale.z = 0.5
		marker_goal.color.a = 1.0
		marker_goal.color.r = 0.0
		marker_goal.color.g = 1.0
		marker_goal.color.b = 0.0

		self.goal_pub.publish(marker_goal)

############################### MAIN PROGRAM ####################################
if __name__ == "__main__":
	PuzzlebotLocClass()