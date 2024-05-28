#!/usr/bin/env python

import rospy
import numpy as np
import pandas as pd
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import Float32
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler
from geometry_msgs.msg import Twist

def z(x):
    return 5 if x == 2 else x

class Experiments():

	WHEEL_RADIUS = 0.05
	TRACK_WIDTH = 0.19

	KL = 10
	KR = 10

	X_OFFSET = 10
	Y_OFFSET = 0.3

	def __init__(self):
		rospy.init_node('experiments', anonymous=True)

		while rospy.get_time() == 0:
			print("No simulated time has been received yet")

		# Define the rate of the while loop
		self.dt = 0.01
		rate = rospy.Rate(1/self.dt)

		IS_LINEAR = int(input("Linear or angular? (0/1): "))
		print(IS_LINEAR)
		ITER = int(input("Iteration (0-4): "))

		# get data/Mediciones.xlsx
		data = pd.read_excel('data/Mediciones.xlsx')

		linear = data.iloc[:, 3:7].to_numpy()

		linear_thetas = data.iloc[:, 10].to_numpy()
		linear_speed = data.iloc[:, 1].to_numpy()

		# get second excel sheet in the document
		data = pd.read_excel('data/Mediciones.xlsx', sheet_name='Mediciones vuelta')
		angular = data.iloc[:, 3:5].to_numpy()
		angular_speed = data.iloc[:, 1].to_numpy()

		poses_publisher = rospy.Publisher('poses', PoseArray, queue_size=10)
		odom_publisher = rospy.Publisher('odom', Odometry, queue_size=10)

		self.cmd_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=10)

		rospy.on_shutdown(self.on_shutdown)

		rospy.Subscriber('wr', Float32, self.wr_callback)
		rospy.Subscriber('wl', Float32, self.wl_callback)

		self.wr = 0
		self.wl = 0

		covariance = np.zeros((3,3))

		poses = PoseArray()
		poses.header.stamp = rospy.Time.now()
		poses.header.frame_id = 'map'

		start = ITER*10
		end = start + 10

		expected_x = linear[start][0]
		expected_theta = angular[start][0]


		x_avg = 0
		y_avg = 0
		theta_avg = 0

		if IS_LINEAR == 0:

			linear_speed = linear_speed[start]
			angular_speed = 0

			for i, (_, _, x, y) in enumerate(linear[start:end]):
				pose = Pose()
				pose.position.x = x
				pose.position.y = y
				pose.position.z = 0
				quat = quaternion_from_euler(0,0,np.deg2rad(linear_thetas[i]))
				pose.orientation.x = quat[0]
				pose.orientation.y = quat[1]
				pose.orientation.z = quat[2]
				pose.orientation.w = quat[3]
				poses.poses.append(pose)
				x_avg += x
				y_avg += y
				theta_avg += np.deg2rad(linear_thetas[i])

		else:
			linear_speed = 0
			angular_speed = angular_speed[start]
			for i, (_, theta) in enumerate(angular[start:end]):
				pose = Pose()
				pose.position.x = 0
				pose.position.y = 0
				pose.position.z = 0
				quat = quaternion_from_euler(0,0,np.deg2rad(theta))
				pose.orientation.x = quat[0]
				pose.orientation.y = quat[1]
				pose.orientation.z = quat[2]
				pose.orientation.w = quat[3]
				poses.poses.append(pose)

		pose = np.array([[0.0],
				 [0.0],
				 [0.0]])

		theta = 0.0

		while not rospy.is_shutdown():

			cmd = Twist()
			if IS_LINEAR == 0:
				if pose[0,0] < expected_x:
					cmd.linear.x = linear_speed
					cmd.angular.z = angular_speed
				else:
					cmd.linear.x = 0
					cmd.angular.z = 0
			else:
				if np.rad2deg(pose[2,0]) < expected_theta:
					cmd.linear.x = linear_speed
					cmd.angular.z = angular_speed
				else:
					cmd.linear.x = 0
					cmd.angular.z = 0

			self.cmd_publisher.publish(cmd)

			linear_velocity = (self.wr + self.wl)/2*self.WHEEL_RADIUS
			angular_velocity = self.WHEEL_RADIUS*(self.wr - self.wl)/self.TRACK_WIDTH

			dt_v = self.dt*linear_velocity
			dt_w = self.dt*angular_velocity

			pose += np.array([  [dt_v*np.cos(theta)],
					  			[dt_v*np.sin(theta)],
					  			[dt_w]])

			Hk = np.array([ [1, 0, -dt_v*np.sin(theta)],
							[0, 1, dt_v*np.cos(theta)],
							[0, 0, 1]])

			Q_k = self.QK(theta, self.wr, self.wl)

			covariance = Hk.dot(covariance).dot(Hk.T) + Q_k

			theta = pose[2,0]

			poses_publisher.publish(poses)

			odometry = self.fill_odom(pose[0,0], pose[1,0], pose[2,0], covariance, linear_velocity, angular_velocity)

			odom_publisher.publish(odometry)

			rate.sleep()

	def dwk(self, theta):
		matrix = np.array([[np.cos(theta), np.cos(theta)],
							[np.sin(theta), np.sin(theta)],
							[2/self.TRACK_WIDTH, -2/self.TRACK_WIDTH]])
		return 0.5*self.WHEEL_RADIUS*self.dt*matrix

	def sigma(self, wr, wl):
		return np.array([[self.KR*abs(wr), 0],
						 [0, self.KL*abs(wl)]])

	def QK(self, theta, wr, wl):
		return self.dwk(theta).dot(self.sigma(wr, wl)).dot(self.dwk(theta).T)

	def wr_callback(self, msg):
		self.wr = msg.data

	def wl_callback(self, msg):
		self.wl = msg.data

	def fill_odom(self,Mu_x, Mu_y, Mu_theta, Sigma, Mu_v, Mu_w):
		odom=Odometry()
		odom.header.stamp =rospy.Time.now()
		odom.header.frame_id = "map"
		odom.child_frame_id = "base_link"

		# Fill the pose information
		odom.pose.pose.position.x = Mu_x
		odom.pose.pose.position.y = Mu_y
		odom.pose.pose.position.z = 0.0
		quat=quaternion_from_euler(0,0,Mu_theta)
		odom.pose.pose.orientation.x = quat[0]
		odom.pose.pose.orientation.y = quat[1]
		odom.pose.pose.orientation.z = quat[2]
		odom.pose.pose.orientation.w = quat[3]

		# Init a 36 elements array
		odom.pose.covariance = [0.0]*36
		for i in range(3):
			for j in range(3):
				odom.pose.covariance[z(i)*6+z(j)] = Sigma[i,j]

		odom.pose.covariance[0]*=self.X_OFFSET
		odom.pose.covariance[7]*=self.Y_OFFSET
		# Fill the speed information
		odom.twist.twist.linear.x = Mu_v
		odom.twist.twist.angular.z = Mu_w

		return odom


	def on_shutdown(self):
		rospy.loginfo('Shutting down')
		cmd = Twist()
		cmd.linear.x = 0
		cmd.angular.z = 0
		self.cmd_publisher.publish(cmd)



if __name__ == '__main__':
	node = Experiments()