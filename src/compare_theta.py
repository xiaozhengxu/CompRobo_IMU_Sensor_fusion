#!/usr/bin/env python

import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Pose2D
from tf.transformations import quaternion_from_euler, euler_from_quaternion

def convert_pose_to_xy_and_theta(pose):
    """ Convert pose (geometry_msgs.Pose) to a (x,y,yaw) tuple """
    orientation_tuple = (pose.orientation.x,
                         pose.orientation.y,
                         pose.orientation.z,
                         pose.orientation.w)
    angles = euler_from_quaternion(orientation_tuple)
    return_pose = Pose2D(pose.position.x, pose.position.y, angles[2])
    return return_pose

class Compare_Poses(object):
    """ A simple node that subscribes to the published poses of odom, combined_odom, 
        and STAR_pose to compare them 
    """
    def __init__(self):
        rospy.init_node('compare_poses')
        self.odom = Pose2D(x=0,y=0,theta=0)
        self.combined_odom = Pose2D(x=0,y=0,theta=0)
        self.STAR = Pose2D(x=0,y=0,theta=0)

        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_callback)
        self.combined_odom_sub = rospy.Subscriber('combined_odom', Odometry, self.combined_odom_callback)
        self.star_sub = rospy.Subscriber('STAR_pose', PoseStamped, self.star_callback)

        self.odom_pub = rospy.Publisher('odom_2d', Pose2D, queue_size=10)
        self.combined_odom_pub = rospy.Publisher('combined_odom_2d', Pose2D, queue_size=10)
        self.star_pub = rospy.Publisher('STAR_2d', Pose2D, queue_size=10)

    def odom_callback(self, msg):
        self.odom = convert_pose_to_xy_and_theta(msg.pose.pose)

    def combined_odom_callback(self,msg):
        self.combined_odom = convert_pose_to_xy_and_theta(msg.pose.pose)

    def star_callback(self,msg):
        self.STAR = convert_pose_to_xy_and_theta(msg.pose)

    def print_thetas(self):
        print "Thetas -> Odom: " + str(self.odom[2]) + " Combined_Odom: " + str(self.combined_odom[2]) + " STAR: " + str(self.STAR[2])

    def publish_2d_poses(self):
        self.odom_pub.publish(self.odom)
        self.combined_odom_pub.publish(self.combined_odom)
        self.star_pub.publish(self.STAR)


    def run(self):
        r = rospy.Rate(20)
        while not rospy.is_shutdown():
            self.publish_2d_poses()
            try:
                r.sleep()
            except rospy.exceptions.ROSTimeMovedBackwardsException:
                print "Time went backwards. Carry on."


if __name__ == '__main__':
    node = Compare_Poses()
    node.run()