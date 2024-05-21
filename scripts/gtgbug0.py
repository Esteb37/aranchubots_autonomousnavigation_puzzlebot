#!/usr/bin/env python3
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
# This class implements a simple obstacle avoidance algorithm
class WallFollowingControlClass():
    def __init__(self):
        rospy.on_shutdown(self.cleanup)

        ####################### PUBLISEHRS AND SUBSCRIBERS ############################
        rospy.Subscriber("scan", LaserScan, self.laser_cb) #base_scan
        rospy.Subscriber("gtg_vel", Twist, self.gtg_cb)
        rospy.Subscriber("ed", Float32, self.ed_cb)
        rospy.Subscriber("ed_theta", Float32, self.ed_theta_cb)
        self.cmd_vel_pub = rospy.Publisher("cmd_vel", Twist, queue_size=1)

        ######################## CONSTANTS AND VARIABLES ##############################
        self.laser_received             = False
        self.control_received           = False
        self.avoid_obstacle_received    = False
        self.ed_received                = False
        self.ed_theta_received          = False
        self.flag_wf                    = False
        self.flag                       = "go to goal"
        self.flag_current_state         = "gtg"
        #v_desired  = 0.4 #[m/s] desired speed when there are no obstacles
        self.closest_angle = 0.0 #Angle to the closest object
        self.closest_range = np.inf #Distance to the closest object
        vel_msg = Twist()
        ## Speeds from topics
        self.ao_msg     = Twist()
        self.gtg_msg    = Twist()
        self.ed_msg     = Float32()
        self.ed_theta_msg     = Float32()
        epsilon     = 0.1 # 0.2
        self.ed_tau = 0.0
        self.distance_H = 0.0   # Distance to HIT POINT
        self.distance_L = 0.0   # Distance to LEAVE POINT
        freq = 10
        rate = rospy.Rate(freq) #10Hz is the lidar's frequency
        print("Node initialized " + str(freq) + " hz")
        ############################### MAIN LOOP #####################################
        while not rospy.is_shutdown():
            ############################### YOUR CODE HERE ##############################
            if self.laser_received and self.control_received and self.ed_received and self.ed_theta_received:
                self.laser_received = False
                #dfw = min(self.lidar_msg.ranges)
                vel_msg = Twist()
                # Closest range and theta_ao
                l_msg = self.lidar_msg
                # Transform laser data because of <laser> PI rotation w.r.t. <base_link>
                new_ranges = self.tf_laser_data(l_msg.ranges)
                # closest_range = min(l_msg.ranges)                               # closest_range ---> antes
                closest_range = min(new_ranges) # new
                idx = l_msg.ranges.index(closest_range)
                closest_angle = l_msg.angle_min + idx * l_msg.angle_increment   # closest_angle
                theta = closest_angle
                theta       = np.arctan2(np.sin(theta),np.cos(theta))
                theta_ao    = theta - np.pi
                theta_ao    = np.arctan2(np.sin(theta_ao),np.cos(theta_ao))     # theta_ao
                vel_gtg     = self.gtg_msg.linear.x
                w_gtg       = self.gtg_msg.angular.z
                theta_gtg = self.ed_theta_msg.data
                ####
                """
                if (self.ed_msg.data < 0 or self.ed_msg.data > 0.20):
                    if (dfw <= 0.5): # 0.6
                        theta_fwc   = -np.pi/2 + theta_ao
                        theta_fwcc  = np.pi/2 + theta_ao
                        theta_fwc   = np.arctan2(np.sin(theta_fwc), np.cos(theta_fwc))
                        theta_fwcc  = np.arctan2(np.sin(theta_fwcc), np.cos(theta_fwcc))
                        if (self.flag_wf == False):
                            if (abs(theta_fwc - theta_gtg) <= np.pi/2): self.flag = "wall following - clockwise"
                            else: self.flag = "wall following - counter-clockwise"
                            self.flag_wf = True
                            self.ed_tau = self.ed_msg.data      # dGTG(tau)
                        if (self.flag == "wall following - clockwise"): theta_fwf = theta_fwc
                        else: theta_fwf = theta_fwcc
                        if ((abs(theta_ao - theta_gtg) < np.pi/2) and (self.ed_msg.data < abs(self.ed_tau - epsilon))):
                            vel_msg.linear.x, vel_msg.angular.z = vel_gtg, w_gtg
                            self.flag       = "go to goal"
                            self.flag_wf    = False
                        else:
                            Kw = 1.0
                            vel_msg.linear.x, vel_msg.angular.z = 0.1, Kw * theta_fwf
                    else:
                        vel_msg.linear.x, vel_msg.angular.z = vel_gtg, w_gtg
                        self.flag       = "go to goal"
                        self.flag_wf    = False
                else:
                    vel_msg.linear.x, vel_msg.angular.z = 0.0, 0.0
                    print("STOP")
                """
                ### OLDD
                ### NEW
                goal_distance = self.ed_msg.data    # Distance to goal [m]
                fw_distance = 0.3                   # Following wall distance [m] -- # 0.4
                target_position_tolerance = 0.20    # Target position tolerance [m]

                print("================ INIT")
                if (goal_distance < target_position_tolerance):      # At goal - Goal reached
                    print("Goal reached STOP")
                    self.flag_current_state = "stop"
                    #print("Stop")
                    vel_msg.linear.x, vel_msg.angular.z = 0.0, 0.0
                elif self.flag_current_state == "gtg":                  # Go to goal
                    if closest_range <= fw_distance:
                            # Implement the following walls behavior
                        self.distance_H = goal_distance                # Save HIT POINT distance to goal
                        print(" change to following walls")
                        self.flag_current_state = "Clockwise"
                    else:
                        print("Moving to the Goal")
                        vel_msg.linear.x, vel_msg.angular.z = vel_gtg, w_gtg

                elif self.flag_current_state == "Clockwise":
                    self.distance_L = goal_distance                    # Save LEAVE POINT distance to goal EVERY TIME we are in WF behaviour
                    if ((abs(abs(theta_ao) - abs(theta_gtg)) < np.pi/2) and (self.distance_L < abs(self.distance_H - epsilon))): # Output conditions of Wall Following behaviour ---> CLEAR SHOT and FAT GUARDS
                        self.flag_current_state = "gtg"
                        print("Change to Go to goal")
                    else:
                        # Always one way
                        #"""
                        flag_cc_ccc = 1     # 0 --> Clockwise
                                            # 1 --> Counterclockwise
                        if flag_cc_ccc == 0:    theta_fw  = -np.pi/2 + theta_ao  # Clockwise behaviour
                        else:                   theta_fw  = np.pi/2 + theta_ao   # Counterclockwise behaviour
                        theta_fw = np.arctan2(np.sin(theta_fw), np.cos(theta_fw))
                        #"""
                        # Let the algorithm decide
                        """
                        theta_fwc   = -np.pi/2 + theta_ao
                        theta_fwcc  = np.pi/2 + theta_ao
                        theta_fwc   = np.arctan2(np.sin(theta_fwc), np.cos(theta_fwc))
                        theta_fwcc  = np.arctan2(np.sin(theta_fwcc), np.cos(theta_fwcc))
                        if (abs(theta_fwc - theta_gtg) <= np.pi/2): theta_fw = theta_fwc
                        else: theta_fw = theta_fwcc
                        #theta_fw = np.arctan2(np.sin(theta_fw), np.cos(theta_fw))
                        """
                        Kw = 1.6 # 1.3
                        vel_msg.linear.x, vel_msg.angular.z = 0.09, Kw * theta_fw   # 0.12

                elif self.flag_current_state == 'stop':
                    print("STOP")
                    vel_msg.linear.x, vel_msg.angular.z = 0.0, 0.0

                print("Actual State         : " + self.flag_current_state)
                print("vel_x                : " + str(vel_msg.linear.x))
                print("vel_z                : " + str(vel_msg.angular.z))
                print("closest_range <= "+ str(fw_distance) + ": " + str(closest_range))
                print("goal_distance or LP  : " + str(goal_distance))
                print("GD or LP < THIS (FG) : " + str(abs(self.distance_H - epsilon)))
                print("1.5708   > THIS (CS) : " + str(abs(abs(theta_ao) - abs(theta_gtg))))
                print("theta_ao             : " + str(theta_ao))
                print("theta_gtg            : " + str(theta_gtg))
                print("================ END")
                print(" ")
            self.cmd_vel_pub.publish(vel_msg)
            rate.sleep()
    def tf_laser_data(self, lidar_ranges):
        ori     = lidar_ranges
        #len(a) = 1147 impar -- mid point (in front) -- index int(1147/2 - 0.5)
        fin = np.roll(ori, int(len(lidar_ranges)/2 + 1))
        return fin
    def laser_cb(self, msg):
        ## This function receives a message of type LaserScan and computes the closest object direction and range
        self.lidar_msg = msg
        self.laser_received = True
    def gtg_cb(self, msg):
        self.gtg_msg = msg
        self.control_received = True
    def ed_cb(self, msg):
        self.ed_msg = msg
        self.ed_received = True
    def ed_theta_cb(self, msg):
        self.ed_theta_msg = msg
        self.ed_theta_received = True
    def cleanup(self):
        #This function is called just before finishing the node
        # You can use it to clean things up before leaving
        # Example: stop the robot before finishing a node.
        vel_msg = Twist()
        self.cmd_vel_pub.publish(vel_msg)
############################### MAIN PROGRAM ####################################
if __name__ == "__main__":
    rospy.init_node("wall_following", anonymous=True)
    WallFollowingControlClass()
