#!/usr/bin/env python
import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from tf.transformations import quaternion_from_euler
import numpy as np

np.set_printoptions(suppress=True)
np.set_printoptions(formatter={'float': '{: 0.4f}'.format})

def z(x):
    return 5 if x == 2 else x

class PublishOdomCov():
    def __init__(self):
        # first thing, init a node!
        rospy.init_node('publish_odom_with_covariance')
        self.odom_pub = rospy.Publisher("odom", Odometry, queue_size=1)
        odom = Odometry()

        # Wait for a valid simulated time
        while rospy.get_time() == 0:
            print("no simulated time has been received yet")
        print("Got time")

        rate = rospy.Rate(20) # The rate of the while loop
        while not rospy.is_shutdown():
            # 2D robot pose
            Mu_x=1.0 # Mean value of position x
            Mu_y = 1.0 # Mean value of position y
            Mu_theta = 0.7 # Mean value of orientation (yaw)

            sxx = 1.0 #Variance along x axis
            syy = 1.0 #Variance along y axis
            stheta = 0.7 #Variance of theta

            #Robot velocity
            Mu_v=0.0 #[m/s] Mean value of Robot linear speed
            Mu_w=0.0 #[rad/s] Mean value of Robot angular speed
            # Calculate Covariance Matrix Sigma for (x,y, theta)
            Sigma_2D_pose = np.array([[sxx,0.0,0.0],
                                      [0.0,syy,0.0],
                                      [0.0,0.0,stheta]])
            print("2D Pose: ",(Mu_x,Mu_y,Mu_theta))
            print("Covariance: ")
            print(Sigma_2D_pose)
            odom = self.fill_odom(Mu_x,Mu_y,Mu_theta, Sigma_2D_pose, Mu_v, Mu_w)
            self.odom_pub.publish(odom)
            rate.sleep()

    def fill_odom(self,Mu_x, Mu_y, Mu_theta, Sigma, Mu_v, Mu_w):
        odom=Odometry()
        odom.header.stamp =rospy.Time.now()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        # Fill the pose information
        odom.pose.pose.position.x = Mu_x
        odom.pose.pose.position.y = Mu_y
        odom.pose.pose.position.z = 0.0
        quat=quaternion_from_euler(0.0, 0.0, Mu_theta)
        odom.pose.pose.orientation.x = quat[0]
        odom.pose.pose.orientation.y = quat[1]
        odom.pose.pose.orientation.z = quat[2]
        odom.pose.pose.orientation.w = quat[3]

        # Init a 36 elements array
        odom.pose.covariance = [0.0]*36
        for i in range(3):
            for j in range(3):
                odom.pose.covariance[z(i)*6+z(j)] = Sigma[i,j]

        # Fill the speed information
        odom.twist.twist.linear.x = Mu_v
        odom.twist.twist.angular.z = Mu_w

        return odom


############################### MAIN PROGRAM ####################################
if __name__ == "__main__":
    PublishOdomCov()