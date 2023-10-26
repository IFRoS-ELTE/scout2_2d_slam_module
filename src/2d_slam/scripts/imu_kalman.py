#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Imu
import numpy as np
from filterpy.kalman import KalmanFilter
from geometry_msgs.msg import Vector3

class IMUFilterNode:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('imu_filter_node')
        
        # Define the Kalman filter for angular velocity
        self.kf_ang_vel = KalmanFilter(dim_x=3, dim_z=3)
        self.kf_ang_vel.x = np.array([0.0, 0.0, 0.0])  # Initial state estimate
        self.kf_ang_vel.P = np.eye(3)  # Initial state covariance
        self.kf_ang_vel.F = np.eye(3)  # Transition matrix
        self.kf_ang_vel.H = np.eye(3)  # Measurement matrix
        self.kf_ang_vel.R = np.eye(3)  # Measurement noise covariance
        self.kf_ang_vel.Q = np.diag([0.05, 0.05, 0.05])  # Process noise covariance

        # Define the Kalman filter for linear acceleration
        self.kf_lin_acc = KalmanFilter(dim_x=3, dim_z=3)
        self.kf_lin_acc.x = np.array([0.0, 0.0, 0.0])  # Initial state estimate
        self.kf_lin_acc.P = np.eye(3)  # Initial state covariance
        self.kf_lin_acc.F = np.eye(3)  # Transition matrix
        self.kf_lin_acc.H = np.eye(3)  # Measurement matrix
        self.kf_lin_acc.R = np.eye(3)  # Measurement noise covariance
        self.kf_lin_acc.Q = np.eye(3)  # Process noise covariance

        # Create ROS subscribers and publishers
        self.imu_sub = rospy.Subscriber("/imu/data", Imu, self.imu_callback)
        self.filtered_ang_vel_pub = rospy.Publisher("/filtered_angular_velocity", Vector3, queue_size=10)
        self.filtered_lin_acc_pub = rospy.Publisher("/filtered_linear_acceleration", Vector3, queue_size=10)

    def imu_callback(self, imu_msg):
        # Extract angular velocity and linear acceleration data from the IMU message
        ang_vel = np.array([imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z])
        lin_acc = np.array([imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z])

        # Apply Kalman filter to angular velocity
        self.kf_ang_vel.predict()
        self.kf_ang_vel.update(ang_vel)
        filtered_ang_vel = self.kf_ang_vel.x

        # Apply Kalman filter to linear acceleration
        self.kf_lin_acc.predict()
        self.kf_lin_acc.update(lin_acc)
        filtered_lin_acc = self.kf_lin_acc.x

        # Publish filtered angular velocity and linear acceleration
        ang_vel_msg = Vector3(*filtered_ang_vel)
        lin_acc_msg = Vector3(*filtered_lin_acc)
        self.filtered_ang_vel_pub.publish(ang_vel_msg)
        self.filtered_lin_acc_pub.publish(lin_acc_msg)

    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        node = IMUFilterNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
