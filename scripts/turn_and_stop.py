#!/usr/bin/env python3

import rospy

from geometry_msgs.msg import Twist
from std_msgs.msg import String

class TurnAndStop():

    def __init__(self):

        rospy.init_node('move_forward_some_time')

        rospy.on_shutdown(self.cleanup)


        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist,queue_size=1)

        self.command_sub = rospy.Subscriber('command', String, self.command_callback)

        self.command = None

        my_twist = Twist()
        my_twist.angular.z = 0.5

        rate = rospy.Rate(20)

        rospy.loginfo("About to be moving forward!")

        while rospy.get_time() == 0:

            print("no simulated time has been received yet")

        while not rospy.is_shutdown():

            if self.command == "stop":
                my_twist.angular.z = 0

            self.cmd_vel_pub.publish(my_twist)
            rate.sleep()

    def command_callback(self, msg):
        self.command = msg.data


    def cleanup(self):
        self.cmd_vel_pub.publish(Twist())
        print("Stopping the robot")
        print("Bye bye!!!")



############################### MAIN PROGRAM ####################################

if __name__ == "__main__":

    TurnAndStop()