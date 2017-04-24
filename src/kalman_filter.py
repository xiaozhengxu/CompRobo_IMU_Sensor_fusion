#!/usr/bin/env python

"""
    This script implements a Kalman filter to integrate IMU yaw orientation with the neato's odometry:

    x_0 ~ N(0, sigma_sq)
    x_t = x_{t-1} + w_t, w_t ~ N(0, sigma_m_sq)
    z_t = x_t + v_t, v_t ~ N(0, sigma_z_sq)
"""


import rospy
import math
from math import e, sqrt, pi
import sys
import time

from numpy import arange
from numpy.random import randn

import matplotlib.pyplot as plt
from dynamic_reconfigure.server import Server
from simple_filter.cfg import SimpleKalmanConfig

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry

from tf.transformations import quaternion_from_euler, rotation_matrix, quaternion_from_matrix
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
    """ A Kalman filter node that estimates a single state theta using noisy wheel odometry estimates and imu orientation"""

    def __init__(self):
        """ Sets up the world model and loads initial parameters """
        rospy.init_node('kalman_filter')

        # initial beliefs: TO change
        self.theta = 0
        self.sigma_sq = 1

        #Set initial odometry
        self.odom_x = None
        self.odom_y = None
    	self.odom_yaw = None

        # motor noise
        sigma_m_sq = rospy.get_param('~sigma_m_sq', 0.01)
        # observation noise
        sigma_z_sq = rospy.get_param('~sigma_z_sq', .1)

        # Set up publisher to publish to combined Odom
        self.pub = rospy.Publisher('combined_odom', Odometry, queue_size=1)

        # Set up substriber to imu and odom
        self.imu_sub = rospy.Subscriber('imu',Imu, self.imu_callback)
        self.odom_sub = rospy.Subscriber('odom',Odometry, self.odom_callback)

        srv = Server(SimpleKalmanConfig, self.config_callback)


    def imu_callback(self,msg):
    	imu_pose = convert_pose_to_xy_and_theta(msg)
    	self.imu_pitch = imu_pose[0]
        self.imu_roll = imu_pose[1]
    	self.imu_yaw = imu_pose[2]

    	self.imu_ztheta_dot = msg.angular_velocity.z

    def odom_callback(self,msg):
    	cur_pos = convert_pose_to_xy_and_theta(msg.pose.pose)
    	self.odom_pitch = cur_pos[0]
        self.odom_roll = cur_pos[1]
    	self.odom_yaw = cur_pos[2]

    	self.odom_ztheta_dot = msg.twist.twist.angular.z

    	self.odom_xdot = msg.pose.twist.twist.linear.x
    	self.odom_ydot = msg.pose.twist.twist.linear.y

    def config_callback(self, config, level):
        """ Get the pause_time, movement noise, and measurement noise """
        self.pause_time = config['pause_time']
        self.world.sigma_m_sq = config['sigma_m_sq']
        self.world.sigma_z_sq = config['sigma_z_sq']
        return config

    def run(self):
        while not rospy.is_shutdown():
            # Do Kalman updates
            K_t = (self.sigma_sq + self.world.sigma_m_sq)/(self.sigma_sq + self.world.sigma_m_sq + self.world.sigma_z_sq)
            self.mu = self.mu + K_t*(z_t - self.mu)
            self.sigma_sq = (1-K_t)*(self.sigma_sq+self.world.sigma_m_sq)



if __name__ == '__main__':
    node = KalmanFilter()
    node.run()