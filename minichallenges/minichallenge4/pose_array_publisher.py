#!/usr/bin/env python
# license removed for brevity 
import rospy
from geometry_msgs.msg import PoseArray, Pose
import numpy as np
from tf.transformations import quaternion_from_euler 
import numpy as np
import pandas as pd

class PublishPoseArray():
    def __init__(self):
        # first thing, init a node!
        rospy.init_node('pose_array_publisher')
        self.pose_array_pub = rospy.Publisher("pose_array_topic", PoseArray, queue_size=1)
        # Create a PoseArray message

        # Cargar archivo csv/xlsx con los datos obtenidos
        data = pd.read_csv('/home/naomin/catkin_ws/src/aranchubots_autonomousnavigation_puzzlebot/minichallenges/minichallenge4/mediciones_aranchubots.csv')

        pose_array_msg = PoseArray()
        # Wait for a valid simulated time 
        while rospy.get_time() == 0: 
            print("no simulated time has been received yet")
        print("Got time") 
         # Add some sample poses to the PoseArray 
        for i in range(50): 
            pose = Pose() 
            # The position will be a random number with normal distribution 
            # NOTE: this is just an example  
            # you will have to fill the pose information (x,y,theta) with the  
            # results from your different experiments.  
            """
            EXAMPLE
            pose.position.x = 1+np.random.normal(0,0.3,1) # mean 0 and standard deviation 0.3 
            pose.position.y = np.random.normal(0,0.1,1) # mean 0 and standard deviation 0.1 
            pose.position.z = 0 
            """
            # OUR DATA
            pose.position.x = data['X'].values
            print(pose.position.x)
            pose.position.y = data['Y'].values
            pose.position.z = data['Z'].values

            theta = data['Theta'].values

            quat = quaternion_from_euler(0.0, 0.0, theta) 
            pose.orientation.x = quat[0] 
            pose.orientation.y = quat[1] 
            pose.orientation.z = quat[2] 
            pose.orientation.w = quat[3] 
            pose.orientation.w = 1.0 
            pose_array_msg.poses.append(pose) 

        rate = rospy.Rate(20) # The rate of the while loop 

        # Set the header information (frame ID and timestamp) 
        pose_array_msg.header.frame_id = 'odom' 
        pose_array_msg.header.stamp = rospy.Time.now() 

        # Publish the PoseArray 
        rate = rospy.Rate(10)  # 10 Hz 

        while not rospy.is_shutdown():
            self.pose_array_pub.publish(pose_array_msg) 
            rate.sleep() 

############################### MAIN PROGRAM ####################################  

if __name__ == "__main__":
    PublishPoseArray()  