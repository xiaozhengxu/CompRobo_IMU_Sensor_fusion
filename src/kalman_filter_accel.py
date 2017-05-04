#!/usr/bin/env python

"""
    This script implements a Kalman filter to determine linear position from imu accel
    authors: Xiaozheng Xu and Riley Chapman
    Date: April 30th, 2017
    
    x_0 ~ N(0, sigma_sq)
    x_t = f(x_{t-1}) + w_t, w_t ~ N(0, sigma_m_sq)
    z_t = H * x_t + v_t, v_t ~ N(0, sigma_z_sq)
"""

import rospy
import math
from math import e, sqrt, pi, cos, sin
import sys
import time

import numpy as np

import matplotlib.pyplot as plt
from dynamic_reconfigure.server import Server
from simple_filter.cfg import SimpleKalmanConfig

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Twist

from tf.transformations import quaternion_from_euler, rotation_matrix, quaternion_from_matrix, euler_from_quaternion
from tf.broadcaster import TransformBroadcaster
from dynamic_reconfigure.server import Server
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

def convert_pose_to_xy_and_theta(pose):
    """ Convert pose (geometry_msgs.Pose) to a (x,y,yaw) tuple """
    orientation_tuple = (pose.orientation.x,
                         pose.orientation.y,
                         pose.orientation.z,
                         pose.orientation.w)
    angles = euler_from_quaternion(orientation_tuple)
    return (pose.position.x, pose.position.y, angles[2])

class KalmanFilter(object):
    """ A Kalman filter node that estimates neato's position using wheel odometry generated odom and imu data 
    The state variables are x, y, theta, v, w and the sensor measurements are v_odom, w_odom, and w_gyro"""

    def __init__(self):
        """ Sets up the world model and loads initial parameters """
        rospy.init_node('kalman_filter')

        # initial beliefs
        self.x = 0
        self.y = 0
        self.z = 0 #This is equivalent to yaw

        #Initialize state variables (also called xk)
        self.mu = np.array([self.x, self.y, self.z, 0, 0, 0, 0, 0, 0]) #x, y, z, vx, vy,vz, ax,ay,az

        #noise in estimate mu, (also known as pk, the prediction error)
        self.sigma_sq = np.array([[0.1, 0, 0, 0, 0, 0, 0, 0, 0],
                                 [0, 0.1, 0, 0, 0, 0, 0, 0, 0],
                                 [0, 0, 0.1, 0, 0, 0, 0, 0, 0],
                                 [0, 0, 0, 0.05, 0, 0, 0, 0, 0],
                                 [0, 0, 0, 0, 0.05, 0, 0, 0, 0],
                                 [0, 0, 0, 0, 0, 0.05, 0, 0, 0],
                                 [0, 0, 0, 0, 0, 0, 0.01, 0, 0],
                                 [0, 0, 0, 0, 0, 0, 0, 0.01, 0],
                                 [0, 0, 0, 0, 0, 0, 0, 0, 0.01]])

        self.x_accel = 0
        self.y_accel = 0
        self.z_accel = 0

        #Initialize the measurements
        self.z_t = np.array([self.x_accel, self.y_accel , self.z_accel]) # ax_accel, ay_accel, az_accel

        
        # update step noise (also known as Q)
        # self.sigma_m_sq = rospy.get_param('~sigma_m_sq', 0.01)
        self.sigma_m_sq = np.array([[0.1, 0, 0, 0, 0, 0, 0, 0, 0],
                                 [0, 0.1, 0, 0, 0, 0, 0, 0, 0],
                                 [0, 0, 0.1, 0, 0, 0, 0, 0, 0],
                                 [0, 0, 0, 0.05, 0, 0, 0, 0, 0],
                                 [0, 0, 0, 0, 0.05, 0, 0, 0, 0],
                                 [0, 0, 0, 0, 0, 0.05, 0, 0, 0],
                                 [0, 0, 0, 0, 0, 0, 0.01, 0, 0],
                                 [0, 0, 0, 0, 0, 0, 0, 0.01, 0],
                                 [0, 0, 0, 0, 0, 0, 0, 0, 0.01]])

        #sensor noise:
        self.sigma_z_sq = np.array([[0.04, 0, 0], #noise in x_accel
                                   [0, 0.04, 0], #noise in y_accel
                                   [0, 0, 0.04]]) #noise in z_accel

        self.H = np.array([[0, 0, 0, 0, 0, 0, 1, 0, 0],
                           [0, 0, 0, 0, 0, 0, 0, 1, 0],
                           [0, 0, 0, 0, 0, 0, 0, 0, 1]])


        # Set up publisher to publish to combined Odom
        self.odom_pub = rospy.Publisher('combined_odom', Odometry, queue_size=1)
        self.odomBroadcaster = TransformBroadcaster()

        # Set up substriber to imu and odom
        self.imu_sub = rospy.Subscriber('imu', Imu, self.imu_callback)
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_callback)

        self.gyro_offset = None

    def imu_callback(self,msg):
        # imu_pose = convert_pose_to_xy_and_theta(msg)
        #The offset for gyro angular velocity_z is 0.01 (when the robot is stationary), so we subtract it from each measurement below: 
        #this is similar to what the calibration in the arduino does

        self.x_accel = msg.linear_acceleration.x
        self.y_accel = msg.linear_acceleration.y
        self.z_accel = msg.linear_acceleration.z-10

    def odom_callback(self,msg):
        pass

    def run(self):
        r = rospy.Rate(20)
        curr_time = rospy.Time.now()
        last_time = curr_time
        odom = Odometry(header=rospy.Header(frame_id="combined_odom"), child_frame_id='base_link')
        while not rospy.is_shutdown():
            # Do Kalman updates
            curr_time = rospy.Time.now()
            dt = (curr_time - last_time).to_sec()
            last_time = curr_time

            F_k = np.array([[1, 0, 0, dt, 0, 0, 0, 0, 0],
                            [0, 1, 0, 0, dt, 0, 0, 0, 0],
                            [0, 0, 1, 0, 0, dt, 0, 0, 0],
                            [0, 0, 0, 1, 0, 0, dt, 0, 0],
                            [0, 0, 0, 0, 1, 0, 0, dt, 0],
                            [0, 0, 0, 0, 0, 1, 0, 0, dt],
                            [0, 0, 0, 0, 0, 0, 1, 0, 0],
                            [0, 0, 0, 0, 0, 0, 0, 1, 0],
                            [0, 0, 0, 0, 0, 0, 0, 0, 1]])

            # print self.z_t
            self.z_t = np.array([self.x_accel, self.y_accel, self.z_accel]) 

            #Predict step
            self.mu = F_k.dot(self.mu)

            #update error in prediction
            self.sigma_sq = F_k.dot(self.sigma_sq).dot(F_k.T) + self.sigma_m_sq

            #Update step
            print np.shape(self.mu)
            print np.shape(self.H.dot(self.mu))
            measurement_residual = self.z_t - self.H.dot(self.mu)

            residual_covariance = self.H.dot(self.sigma_sq).dot(self.H.T) + self.sigma_z_sq

            K_t = self.sigma_sq.dot(self.H.T).dot(np.linalg.inv(residual_covariance)) #Kalman gain

            self.mu = self.mu + K_t.dot(measurement_residual)

            self.sigma_sq = (np.eye(len(self.mu))-K_t.dot(self.H)).dot(self.sigma_sq)

            #Publish the new odom message based on the integrated odom values
            odom.header.stamp = curr_time
            odom.pose.pose.position.x = self.mu[0]
            odom.pose.pose.position.y = self.mu[1]
            odom.pose.pose.position.z = self.mu[2]

            odom.pose.covariance = [self.sigma_sq[0,0], 0, 0, 0, 0, 0, #uncertainty in x
                                    0, self.sigma_sq[1,1], 0, 0, 0, 0, #uncertainty in y
                                    0, 0, self.sigma_sq[2,2], 0, 0, 0, #uncertainty in z
                                    0, 0, 0, 0, 0, 0, #uncertainty in roll
                                    0, 0, 0, 0, 0, 0, #uncertainty in pitch
                                    0, 0, 0, 0, 0, 0] #uncertainty in yaw

            odom.twist.covariance = [0, 0, 0, 0, 0, 0, #uncertainty in x_dot
                                    0, 0, 0, 0, 0, 0, #uncertainty in y_dot
                                    0, 0, 0, 0, 0, 0, #uncertainty in z_dot
                                    0, 0, 0, 0, 0, 0, #uncertainty in change in roll
                                    0, 0, 0, 0, 0, 0, #uncertainty in change in pitch
                                    0, 0, 0, 0, 0, 0] #uncertainty in change in yaw

            self.odom_pub.publish(odom)

            try:
                r.sleep()
            except rospy.exceptions.ROSTimeMovedBackwardsException:
                print "Time went backwards. Carry on."

if __name__ == '__main__':
    node = KalmanFilter()
    node.run()