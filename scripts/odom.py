#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from tf.transformations import quaternion_from_euler
import tf2_ros
from geometry_msgs.msg import TransformStamped
import numpy as np


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
        rospy.Subscriber("wl", Float32, self.wl_sub)
        rospy.Subscriber("wr", Float32, self.wr_sub)


        # Publishers
        self.odom_pub = rospy.Publisher('odom', Odometry, queue_size=1)


        ############ ROBOT CONSTANTS ################
        self.r = 0.05 #puzzlebot wheel radius [m]
        self.L = 0.19 #puzzlebot wheel separation [m]
        self.dt = 0.02 # Desired time to update the robot's pose [s]


        ############ Variables ###############
        self.mu = np.zeros(3)
        self.sigma = np.zeros((3,3))
        self.Qk = np.zeros((3,3))
        self.cov_ruido = np.zeros((2,3))
        self.jacob = np.zeros((3,2))


        self.wl = 0.0
        self.wr = 0.0
        self.theta_ant = 0.0
        self.v = 0.0
        self.w = 0.0
        self.kr = 20.0
        self.kl = 30.0


        self.odometry = Odometry()


        self.tf_broadcaster = tf2_ros.TransformBroadcaster()
        self.t = TransformStamped()


        rate = rospy.Rate(int(0.5/self.dt))

        while not rospy.is_shutdown():

            self.update_robot_pose()


            self.cov_ruido = np.array([[self.kr*np.abs(self.wr), 0], [0, self.kl*np.abs(self.wl)]])
            self.jacob = (0.5*self.r*self.dt) * np.array([[np.cos(self.theta_ant), np.cos(self.theta_ant)], [np.sin(self.theta_ant), np.sin(self.theta_ant)], [2/self.L, -2/self.L]])


            self.Qk = self.jacob.dot(self.cov_ruido).dot(self.jacob.T)


            input_vel = np.array([self.dt*self.v*np.cos(self.mu[-1]), self.dt*self.v*np.sin(self.mu[-1]),self.dt*self.w])
            H = np.array([[1, 0, -input_vel[1]], [0, 1, input_vel[0]], [0, 0, 1]])


            self.mu = self.mu + input_vel
            self.sigma = (H.dot(self.sigma).dot(H.T)) + self.Qk

            ######## Publish the data #################
            self.odom_pub.publish(self.odometry)
            rate.sleep()


    def wl_sub(self, msg):
        self.wl = msg.data


    def wr_sub(self, msg):
        self.wr = msg.data


    def update_robot_pose(self):
        #This functions receives the robot speed v [m/s] and w [rad/s]
        # and gets the robot's pose (x, y, theta) where (x,y) are in [m] and (theta) in [rad]
        # is the orientation,


        self.v = self.r*(self.wr + self.wl)/2
        self.w = -self.r*(self.wr - self.wl)/self.L
        self.mu[0] = self.mu[0] + self.v*np.cos(self.mu[2])*self.dt
        self.mu[1] = self.mu[1] + self.v*np.sin(self.mu[2])*self.dt
        self.theta_ant = self.mu[2]
        self.mu[2] = self.mu[2] + self.w*self.dt


        self.odometry.header.stamp = rospy.Time.now()
        self.odometry.header.frame_id = "odom"
        self.odometry.child_frame_id = "base_link"


        # Fill the pose information
        self.odometry.pose.pose.position.x = self.mu[0]
        self.odometry.pose.pose.position.y = self.mu[1]
        self.odometry.pose.pose.position.z = 0.0


        quat = quaternion_from_euler(0,0,self.mu[2])


        self.odometry.pose.pose.orientation.x = quat[0]
        self.odometry.pose.pose.orientation.y = quat[1]
        self.odometry.pose.pose.orientation.z = quat[2]
        self.odometry.pose.pose.orientation.w = quat[3]


        self.odometry.twist.twist.linear.x = self.v
        self.odometry.twist.twist.angular.z = self.w


        # Init a 36 elements array
        self.odometry.pose.covariance = [0.0] * 36


        # Fill the 3D covariance matrix
        self.odometry.pose.covariance[0] = self.sigma[0][0]
        self.odometry.pose.covariance[1] = self.sigma[0][1]
        self.odometry.pose.covariance[5] = self.sigma[0][2]
        self.odometry.pose.covariance[6] = self.sigma[1][0]
        self.odometry.pose.covariance[7] = self.sigma[1][1]
        self.odometry.pose.covariance[11] = self.sigma[1][2]
        self.odometry.pose.covariance[30] = self.sigma[2][0]
        self.odometry.pose.covariance[31] = self.sigma[2][1]
        self.odometry.pose.covariance[35] = self.sigma[2][2]


        # Fill the transformation information
        self.t.header.stamp = rospy.Time.now()
        self.t.header.frame_id = "odom"
        self.t.child_frame_id = "base_link"
        self.t.transform.translation.x = self.mu[0]
        self.t.transform.translation.y = self.mu[1]
        self.t.transform.translation.z = 0.0


        self.t.transform.rotation.x = quat[0]
        self.t.transform.rotation.y = quat[1]
        self.t.transform.rotation.z = quat[2]
        self.t.transform.rotation.w = quat[3]


        # A transformation is broadcasted instead of published
        self.tf_broadcaster.sendTransform(self.t) #broadcast the transformation



    def get_pose_array(self):
        subset_experiments = self.exp_df.iloc[(self.simulation-1) * 10:((self.simulation-1) * 10) + 10]


        vel = 0.15 + ((self.simulation-1) * 0.05)


        for index, row in subset_experiments.iterrows():
            pose = Pose()


            if self.exp_type == 1:
                pose.position.x = 1.0 + row['X']
                pose.position.y = 0.0 + row['Y']
                pose.position.z = 0.0


                theta = 0.0 + row['Theta']
            else:
                pose.position.x = 0.0
                pose.position.y = 0.0
                pose.position.z = 0.0


                theta = -(np.pi/2 + (row['Theta'] * np.pi/180))


            quat = quaternion_from_euler(0, 0, theta)
            pose.orientation.x = quat[0]
            pose.orientation.y = quat[1]
            pose.orientation.z = quat[2]
            pose.orientation.w = quat[3]


            self.pose_array.poses.append(pose)


        if self.exp_type == 1:
            periodo = 1 / vel
        else:
            periodo = (np.pi/2) / vel
        return vel, periodo



############################### MAIN PROGRAM ####################################
if __name__ == "__main__":
    PuzzlebotLocClass()