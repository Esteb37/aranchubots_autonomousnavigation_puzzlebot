#!/usr/bin/env python3
import rospy
from tf.transformations import quaternion_from_euler
import tf2_ros #ROS package to work with transformations
from geometry_msgs.msg import TransformStamped
import numpy as np


class TfBroadcaster():

    def __init__(self):
        rospy.init_node('tf2_broadcaster')

        #Create a tf broadcaster, it will be in charge of publishing the transformations
        # consider a broadcaster as an special publisher that works only for transformations.
        my_tf_br = tf2_ros.TransformBroadcaster()

        #Create a transformation
        # The type of message that we use to publish a transformation
        t = TransformStamped()

        #Robot pose
        x = 1.0
        y = 0.0
        theta = 0.0
        r=rospy.Rate(50)
        print("Node broadcaster initialized!!")
        print("use rviz to see the rotating transformation")

        while not rospy.is_shutdown():

            # Fill the transformation information
            t.header.stamp = rospy.Time.now()
            t.header.frame_id = "odom"
            t.child_frame_id = "base_link"
            t.transform.translation.x = x
            t.transform.translation.y = y
            t.transform.translation.z = 0.0
            theta += 0.01 #theta will be increasing

            #Clip the value of theta to the interval [-pi to pi]
            theta = np.arctan2(np.sin(theta), np.cos(theta))

            #The transformation requires the orientation as a quaternion
            q = quaternion_from_euler(0, 0, theta)
            t.transform.rotation.x = q[0]
            t.transform.rotation.y = q[1]
            t.transform.rotation.z = q[2]
            t.transform.rotation.w = q[3]

            # A transformation is broadcasted instead of published
            my_tf_br.sendTransform(t) #broadcast the transformation
            r.sleep()

if __name__ == '__main__':
    TfBroadcaster()