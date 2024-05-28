#!/usr/bin/env python
import rospy
from fiducial_msgs.msg import FiducialTransformArray
from std_msgs.msg import Float32MultiArray, Int32
import numpy as np
import math
import tf.transformations as tft

class Aruco():
    def __init__(self):
        rospy.init_node('aruco', anonymous=True)

        rospy.Subscriber("/fiducial_transforms", FiducialTransformArray, self.fiducial_callback)

        position = rospy.Publisher('/position', Float32MultiArray, queue_size=1)
        marker = rospy.Publisher('/position', Int32, queue_size=1)

        rospy.spin()

        self.translation = 0.0
        self.rotation = 0.0
        self.image_error = 0.0
        self.object_error = 0.0
        self.fiducial_area = 0.0

        dt = 0.02
        rate = rospy.Rate(int(0.5/dt))
        while not rospy.is_shutdown():

            # Transformar la posicion del marco del robot usando la traslacion
            position_robot = self.transform_to_robot_frame()

            # Calcular la distancia y el angulo
            distance, angle = self.calculate_distance_and_angle(position_robot)

            # Imprimir la informacion de manera estructurada
            rospy.loginfo("fiducial_id: %d", self.marker_id)
            rospy.loginfo("transform:")
            rospy.loginfo("  translation:")
            rospy.loginfo("    x: %f", self.translation.x)
            rospy.loginfo("    y: %f", self.translation.y)
            rospy.loginfo("    z: %f", self.translation.z)
            rospy.loginfo("  rotation:")
            rospy.loginfo("    x: %f", self.rotation.x)
            rospy.loginfo("    y: %f", self.rotation.y)
            rospy.loginfo("    z: %f", self.rotation.z)
            rospy.loginfo("    w: %f", self.rotation.w)
            rospy.loginfo("image_error: %f", self.image_error)
            rospy.loginfo("object_error: %f", self.object_error)
            rospy.loginfo("fiducial_area: %f", self.fiducial_area)
            rospy.loginfo("Position in Robot Frame: x: %f, y: %f, z: %f", position_robot[0], position_robot[1], position_robot[2])
            rospy.loginfo("Distance: %f, Angle: %f", distance, angle)

            position.publish([distance, angle])
            marker.publish(self.marker_id)
            rate.sleep()


    def transform_to_robot_frame(self):
        # Matriz de transformacion adicional para ajustar el marco de la camara al marco del robot
        T_cam_to_robot = np.array([[0, 0, 1, 0.07],
                                   [1, 0, 0, 0.0],
                                   [0, 1, 0, 0.09],
                                   [0, 0, 0, 1]])

        # Covertir la posicion a un vector homogeneo
        position_homogeneous = np.array([self.translation.x, self.translation.y, self.translation.z, 1.0])

        # Transformar la posicion
        T_final = np.dot(T_cam_to_robot, position_homogeneous)
        return T_final[:3] # Devolver solo x, y, z


    def calculate_distance_and_angle(self, position_robot):
        distance = math.sqrt(position_robot[0]**2 + position_robot[1]**2 + position_robot[2]**2)
        angle = math.atan2(position_robot[1], position_robot[0])
        return distance, angle


    def fiducial_callback(self, data):
        for transform in data.transforms:
            self.marker_id = transform.fiducial_id
            self.translation = transform.transform.translation
            self.rotation = transform.transform.rotation
            self.image_error = transform.image_error
            self.object_error = transform.object_error
            self.fiducial_area = transform.fiducial_area


if __name__ == '__main__':
    Aruco()
