#!/usr/bin/env python
import rospy
from fiducial_msgs.msg import FiducialTransformArray
import numpy as np
import math

assigned_marker = 703

def transform_to_robot_frame(position):
    # Matriz de transformación
    T_cam_to_robot = np.array([[0, -1, 0, 0.06],
                               [0, 0, -1, 0.0],
                               [1, 0, 0, 0.06],
                               [0, 0, 0, 1]])

    # Convertir posición a un vector homogéneo
    position_homogeneous = np.array([position.x, position.y, position.z, 1])

    # Transformar la posición
    position_robot = np.dot(T_cam_to_robot, position_homogeneous)
    return position_robot[:3]  # Devolver solo las coordenadas x, y, z


def calculate_distance_and_angle(position_robot):
    distance = math.sqrt(position_robot[0]**2 + position_robot[1]**2 + position_robot[2]**2)
    angle = math.atan2(position_robot[1], position_robot[0])
    return distance, angle


def fiducial_callback(data):
    for transform in data.transforms:
        marker_id = transform.fiducial_id

        if (marker_id == assigned_marker):
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            image_error = transform.image_error
            object_error = transform.object_error
            fiducial_area = transform.fiducial_area

            # Transformar la posición al marco del robot
            position_robot = transform_to_robot_frame(translation)

            # Calcular la distancia y el ángulo
            distance, angle = calculate_distance_and_angle(position_robot)

            # Imprimir la información de manera estructurada
            rospy.loginfo("fiducial_id: %d", marker_id)
            rospy.loginfo("transform:")
            rospy.loginfo("  translation:")
            rospy.loginfo("    x: %f", translation.x)
            rospy.loginfo("    y: %f", translation.y)
            rospy.loginfo("    z: %f", translation.z)
            rospy.loginfo("  rotation:")
            rospy.loginfo("    x: %f", rotation.x)
            rospy.loginfo("    y: %f", rotation.y)
            rospy.loginfo("    z: %f", rotation.z)
            rospy.loginfo("    w: %f", rotation.w)
            rospy.loginfo("image_error: %f", image_error)
            rospy.loginfo("object_error: %f", object_error)
            rospy.loginfo("fiducial_area: %f", fiducial_area)
            rospy.loginfo("Position in Robot Frame: x: %f, y: %f, z: %f", position_robot[0], position_robot[1], position_robot[2])
            rospy.loginfo("Distance: %f, Angle: %f", distance, angle)


def listener():
    rospy.init_node('fiducial_listener', anonymous=True)
    rospy.Subscriber("/fiducial_transforms", FiducialTransformArray, fiducial_callback)
    rospy.spin()


if __name__ == '__main__':
    listener()
