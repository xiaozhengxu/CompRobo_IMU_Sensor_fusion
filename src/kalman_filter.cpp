
#include <string>
#include <vector>

#include "ros/ros.h"
//#include <sstream>
//#include <iostream>

#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>

#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Float32MultiArray.h>

//#include <tf/tf.h>

class KalmanFilter {
	public:
		void imuCallback(const sensor_msgs::Imu msg);
		void odomCallback(const nav_msgs::Odometry msg);
};

void KalmanFilter::imuCallback(const sensor_msgs::Imu msg){
	ROS_INFO("IMU received");
}

void KalmanFilter::odomCallback(const nav_msgs::Odometry msg){
	ROS_INFO("Odom received");
}


int main(int argc, char **argv){
	ros::init(argc, argv,"kalman_filter");
	ros::NodeHandle n;
	
	KalmanFilter kf;
	ros::Subscriber imu_sub = n.subscribe("imu", 1000, &KalmanFilter::imuCallback, &kf);
	ros::Subscriber odom_sub = n.subscribe("odom", 1000, &KalmanFilter::odomCallback, &kf);

	ros::spin();

  	return 0;
}