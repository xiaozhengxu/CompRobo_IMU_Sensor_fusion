
#include <string>
#include <vector>

#include "ros/ros.h"
#include <sstream>
#include <iostream>
#include <Eigen/Dense>

#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>

#include <geometry_msgs/Vector3Stamped.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Float32MultiArray.h>

#include <tf/tf.h>

class KalmanFilter {
	public:
		//initial beliefs
		float x = 0;
		float y = 0;
		float theta = 0;

		//initial odom/imu values
		float v_odom = 0;
		float w_odom = 0;
		float w_imu = 0;

		int m = 5;
		int n = 2;

		Eigen::MatrixXd mu(m,n);
		//mu << x,y,theta,0,0;//x_0, y_0, theta_0, v_0, w_0

	
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
	ros::Publisher pub = n.advertise<nav_msgs::Odometry>("combined_odom", 1000);

	ros::Rate loop_rate(10);

	while (ros::ok()) {
		ROS_INFO("Doing stuff");
		//publish
		ros::spinOnce();
		loop_rate.sleep();
	}
  	return 0;
}