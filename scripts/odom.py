#!/usr/bin/env python
import rospy
from visualization_msgs.msg import Marker, MarkerArray
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Float32MultiArray
from tf.transformations import quaternion_from_euler
import tf2_ros
from geometry_msgs.msg import TransformStamped
import numpy as np
from KF import KalmanFilter
from aruco_markers import MarkerLocations

np.set_printoptions(suppress=True)
np.set_printoptions(formatter={'float': '{: 0.4f}'.format})

#This class will do the following:
# subscribe to the /cmd_vel topic
# publish the simulated pose of the robot to /pose_sim topic
# publish to /wr and /wl the simulated wheel speed.
class PuzzlebotLocClass():
	def __init__(self):

		self.LOG = np.array([])

		rospy.init_node('localisation')

		# Subscribers
		############################### SUBSCRIBERS #####################################
		x_init = rospy.get_param('/odom_node/pos_x', 0)
		y_init = rospy.get_param('/odom_node/pos_y', 0)
		theta_init = rospy.get_param('/odom_node/pos_theta', 0)
		is_sim = rospy.get_param("/odom_node/is_sim", False)

		prefix = "" if not is_sim else "/puzzlebot_1"
		rospy.Subscriber(prefix+"/wl", Float32, self.wl_cb)
		rospy.Subscriber(prefix+"/wr", Float32, self.wr_cb)
		rospy.Subscriber("/aruco", Float32MultiArray, self.aruco_cb)


		# Publishers
		self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=1)
		self.goal_pub = rospy.Publisher('/goal_marker', Marker, queue_size=1)
		self.aruco_pub = rospy.Publisher('/aruco_markers', MarkerArray, queue_size=1)
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
		Rk = np.array([[0.1, 0.0],
			  		   [0.0, 0.02]])
		covariance = np.zeros((2,3))
		jacobian = np.zeros((3,2))

		self.wl = 0.0
		self.wr = 0.0
		self.aruco_id = 0
		self.z_i = np.zeros(2)
		self.m_i = np.zeros(2)

		KF = KalmanFilter(DT, mu)

		rate = rospy.Rate(int(1 / DT))

		rospy.on_shutdown(self.on_kill)

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

			self.publish_aruco_markers()

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

		log = np.array([pose[0], pose[1], pose[2], speed[0], speed[1]])
		self.LOG = np.append(self.LOG, log)


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

	def publish_aruco_markers(self):
		marker_array = MarkerArray()
		for idx, marker in MarkerLocations.items():
			m = Marker()
			m.header.frame_id = "map"
			m.header.stamp = rospy.Time.now()
			m.ns = "aruco"
			m.id = idx
			m.type = m.CUBE
			m.action = m.ADD
			m.pose.position.x = marker[0]
			m.pose.position.y = marker[1]
			m.pose.position.z = 0.083 + 0.097 / 2
			m.pose.orientation.x = 0.0
			m.pose.orientation.y = 0.0
			m.pose.orientation.z = 0.0
			m.pose.orientation.w = 1.0
			m.scale.x = 0.097
			m.scale.y = 0.097
			m.scale.z = 0.097
			m.color.a = 1.0
			m.color.r = 1.0
			m.color.g = 1.0
			m.color.b = 1.0
			m.lifetime = rospy.Duration(1)
			marker_array.markers.append(m)

		self.aruco_pub.publish(marker_array)

	def on_kill(self):
		rospy.loginfo("Shutting down node")
		rospy.signal_shutdown("User shutdown")
		np.save('log.npy', self.LOG)


############################### MAIN PROGRAM ####################################
if __name__ == "__main__":
	PuzzlebotLocClass()