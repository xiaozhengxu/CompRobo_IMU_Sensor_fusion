
#include <string>
#include <vector>

#include <ros/ros.h>
#include <sstream>
#include <iostream>

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
		ros::Publisher odom_pub;
		ros::Subscriber imu_sub;
		ros::Subscriber odom_sub;

		KalmanFilter(ros::NodeHandle n); //This is the constructor

		void imuCallback(const sensor_msgs::Imu msg){
		}

		void odomCallback(const nav_msgs::Odometry msg){

		}

};

KalmanFilter::KalmanFilter(ros::NodeHandle n){
	odom_pub = n.advertise<nav_msgs::Odometry>("combined_odom",1000);
	imu_sub = n.subscribe("imu",1000, &KalmanFilter::imuCallback, this);
	odom_sub = n.subscribe("odom",1000, &KalmanFilter::odomCallback, this);
	std::cout << "initialized kalman filter";
}

int main(int argc, char **argv){
	ros::init(argc, argv,"Kalman Filter");
	ros::NodeHandle n;
	
	KalmanFilter kf(n);
	// KalmanFilter kf(odom_pub);
	return 0;
}