#!/usr/bin/env python

"""
    This script implements a Kalman filter to integrate IMU yaw orientation with the neato's odometry:

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
        self.theta = 0 #This is equivalent to yaw

        #Initialize state variables (also called xk)
        self.mu = np.array([self.x, self.y, self.theta, 0, 0]) #x_0, y_0, theta_0, v_0, w_0

        #noise in estimate mu, (also known as pk, the prediction error)
        self.sigma_sq = np.array([[0.1, 0, 0, 0, 0],
                                 [0, 0.1, 0, 0, 0],
                                 [0, 0, 0.1, 0, 0],
                                 [0, 0, 0, 0.1, 0],
                                 [0, 0, 0, 0, 0.1]])
        #Initialize the measurements
        self.z_t = np.array([0, 0 ,0]) # v_odom, w_odom, w_gyro

        #Initializing initial odom/imu values
        self.v_odom = 0
        self.w_odom = 0
        self.w_imu = 0
        
        # update step noise (also known as Q)
        # self.sigma_m_sq = rospy.get_param('~sigma_m_sq', 0.01)
        self.sigma_m_sq = np.array([[0.1, 0, 0, 0, 0],
                                 [0, 0.1, 0, 0, 0],
                                 [0, 0, 0.1, 0, 0],
                                 [0, 0, 0, 0.1, 0],
                                 [0, 0, 0, 0, 0.1]])
        # sensor noise
        # self.sigma_z_sq = rospy.get_param('~sigma_z_sq', .1)
        self.sigma_z_sq = np.array([[0.1, 0, 0], #noise in v_odom
                                    [0, 1, 0], #noise in w_odom
                                    [0, 0, 0.03]]) #noise in w_gyro
        
        self.H = np.array([[0, 0, 0, 1, 0],
                          [0, 0, 0, 0, 1],
                          [0, 0, 0, 0, 1]])

        # Set up publisher to publish to combined Odom
        self.odom_pub = rospy.Publisher('combined_odom', Odometry, queue_size=1)
        self.odomBroadcaster = TransformBroadcaster()

        # Set up substriber to imu and odom
        self.imu_sub = rospy.Subscriber('imu', Imu, self.imu_callback)
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_callback)

    def imu_callback(self,msg):
        # imu_pose = convert_pose_to_xy_and_theta(msg)
        self.w_imu = msg.angular_velocity.z

    def odom_callback(self,msg):
        # cur_pos = convert_pose_to_xy_and_theta(msg.pose.pose)
        # self.odom_roll = cur_pos[0]
     #    self.odom_pitch = cur_pos[1]
        # self.odom_yaw = cur_pos[2]
        print 'getting odom values'
        self.w_odom = msg.twist.twist.angular.z
        self.odom_xdot = msg.twist.twist.linear.x
        self.odom_ydot = msg.twist.twist.linear.y
        self.v_odom = sqrt(self.odom_ydot**2+self.odom_ydot**2)

    def run(self):
        r = rospy.Rate(20)
        curr_time = rospy.Time.now()
        last_time = curr_time
        odom = Odometry(header=rospy.Header(frame_id="odom"), child_frame_id='base_link')
        while not rospy.is_shutdown():
            # Do Kalman updates
            curr_time = rospy.Time.now()
            dt = (curr_time - last_time).to_sec()
            last_time = curr_time

            x_k, y_k, theta_k, v_k, w_k = self.mu
            self.z_t = np.array([self.v_odom, self.w_odom, self.w_imu]) 
            #Predict step
            self.mu = np.array([x_k + v_k*dt*cos(theta_k), #update state variables
                                y_k + v_k*dt*sin(theta_k),
                                   theta_k + w_k*dt,
                                         v_k,
                                         w_k])
            #The Jacobian of update model
            F_k = np.array([[1, 0, -v_k*dt*sin(theta_k), dt*cos(theta_k), 0],
                            [0, 1, -v_k*dt*cos(theta_k), dt*sin(theta_k), 0],
                            [0, 0, 1, 0, dt],
                            [0, 0, 0, 1, 0],
                            [0, 0, 0, 0, 1]])
            #update error in prediction
            self.sigma_sq = F_k.dot(self.sigma_sq).dot(F_k.T) + self.sigma_m_sq

            #Update step
            measurement_residual = self.z_t - self.H.dot(self.mu)

            residual_covariance = self.H.dot(self.sigma_sq).dot(self.H.T) + self.sigma_z_sq

            K_t = self.sigma_sq.dot(self.H.T).dot(np.linalg.inv(residual_covariance)) #Kalman gain

            self.mu = self.mu + K_t.dot(measurement_residual)

            self.sigma_sq = (np.eye(len(self.mu))-K_t.dot(self.H)).dot(self.sigma_sq)

            #Publish the new odom message based on the integrated odom values
            odom.header.stamp = curr_time
            odom.pose.pose.position.x = self.mu[0]
            odom.pose.pose.position.y = self.mu[1]
            odom.pose.pose.position.z = 0

            quaternion = Quaternion()
            quaternion.z = sin(self.mu[2]/2.0)
            quaternion.w = cos(self.mu[2]/2.0)
            odom.pose.pose.orientation = quaternion

            odom.pose.covariance = [self.sigma_sq[0,0], 0, 0, 0, 0, 0, #uncertainty in x
                                    0, self.sigma_sq[1,1], 0, 0, 0, 0, #uncertainty in y
                                    0, 0, 0, 0, 0, 0, #uncertainty in z
                                    0, 0, 0, 0, 0, 0, #uncertainty in roll
                                    0, 0, 0, 0, 0, 0, #uncertainty in pitch
                                    0, 0, 0, 0, 0, self.sigma_sq[2,2]] #uncertainty in yaw

            #The velocities are in child frame base_link
            odom.twist.twist.linear.x = self.mu[3]
            odom.twist.twist.angular.z = self.mu[4] 

            odom.twist.covariance = [self.sigma_sq[3,3], 0, 0, 0, 0, 0, #uncertainty in x_dot
                                    0, 0, 0, 0, 0, 0, #uncertainty in y_dot
                                    0, 0, 0, 0, 0, 0, #uncertainty in z_dot
                                    0, 0, 0, 0, 0, 0, #uncertainty in change in roll
                                    0, 0, 0, 0, 0, 0, #uncertainty in change in pitch
                                    0, 0, 0, 0, 0, self.sigma_sq[4,4]] #uncertainty in change in yaw

            self.odomBroadcaster.sendTransform((self.mu[0], self.mu[1], 0), (quaternion.x, quaternion.y, quaternion.z, quaternion.w), curr_time, "base_link", "odom" )
            self.odom_pub.publish(odom)
            try:
                r.sleep()
            except rospy.exceptions.ROSTimeMovedBackwardsException:
                print "Time went backwards. Carry on."

if __name__ == '__main__':
    node = KalmanFilter()
    node.run()