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

from tf.transformations import quaternion_from_euler
from dynamic_reconfigure.server import Server
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

class SimpleKalmanFilter(object):
    """ A Kalman filter node that estimates a single state x_t using noisy position measurements """

    def __init__(self):
        """ Sets up the world model and loads initial parameters """
        rospy.init_node('kalman_filter')

        # initial beliefs: TO change
        self.mu = 0
        self.sigma_sq = 1

        # motor noise
        sigma_m_sq = rospy.get_param('~sigma_m_sq', 0.01)
        # observation noise
        sigma_z_sq = rospy.get_param('~sigma_z_sq', .1)

        # Set up publisher to publish to combined Odom
        self.pub = rospy.Publisher('combined_odom', Imu, queue_size=1)

        # Set up substriber to imu and odom
        srv = Server(SimpleKalmanConfig, self.config_callback)

    def config_callback(self, config, level):
        """ Get the pause_time, movement noise, and measurement noise """
        self.pause_time = config['pause_time']
        self.world.sigma_m_sq = config['sigma_m_sq']
        self.world.sigma_z_sq = config['sigma_z_sq']
        return config

    def run(self):
        while not rospy.is_shutdown():
            # Graph new observation from the system
            z_t = self.world.get_z_t()
            self.graphs = self.plot_pdf(z_t)

            # Do Kalman updates
            K_t = (self.sigma_sq + self.world.sigma_m_sq)/(self.sigma_sq + self.world.sigma_m_sq + self.world.sigma_z_sq)
            self.mu = self.mu + K_t*(z_t - self.mu)
            self.sigma_sq = (1-K_t)*(self.sigma_sq+self.world.sigma_m_sq)
            plt.pause(self.pause_time)
            self.graphs = self.plot_pdf(z_t)

            # sample next state
            self.world.get_x_t()
            plt.pause(self.pause_time)

    def plot_pdf(self, z):
        """ Plot the Gaussian PDF with the specified mean (mu) and variance (sigma_sq)
            x_true is the true system state which will be plotted in blue
            z is the current observation which will be plotted in red """
        xs = arange(min(-5,z-2,self.world.x_true-2), max(5,z+2,self.world.x_true+2), .005)
        p_of_x = [1./sqrt(2*pi*self.sigma_sq)*e**(-(x - self.mu)**2/(2*self.sigma_sq)) for x in xs]
        plt.xlim([min(xs), max(xs)])
        if self.graphs:
            self.graphs[0].set_xdata(xs)
            self.graphs[0].set_ydata(p_of_x)
            self.graphs[1].set_xdata(self.world.x_true)
            self.graphs[2].set_xdata(z)
        else:
            self.graphs = []
            self.graphs.append(plt.plot(xs, p_of_x)[0])
            self.graphs.append(plt.plot(self.world.x_true, 0,'b.')[0])
            self.graphs.append(plt.plot(z, 0,'r.')[0])
            self.graphs[1].set_markersize(20)
            self.graphs[2].set_markersize(20)
            plt.ylim([0, 5])
            plt.legend(('probability density','true position','measured position'))
        plt.show(False)
        return self.graphs

if __name__ == '__main__':
    node = SimpleKalmanFilter()
    node.run()