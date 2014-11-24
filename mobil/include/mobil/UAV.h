/*
 * UAV.h
 *
 *  Created on: May 5, 2014
 *      Author: ilan
 */

#ifndef UAV_H_
#define UAV_H_

#include <math.h>
#include <ros/ros.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/String.h>
//#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include "Utils.h"
#include <iostream> // used to log statistics
#include <fstream> // used to log statistics

using namespace std;

class UAV {
private:
	ros::NodeHandle node_handle_;
	ros::Subscriber raw_imu_publisher_;
	ros::Subscriber magnetic_subscriber_;
	ros::Subscriber location_subscriber_;
	ros::Subscriber file_subscriber_;
	tf::TransformListener listener;
	tf::StampedTransform transform;

public:
	static const double X_MAX_SPEED = 2.0;
	static const double Y_MAX_SPEED = 2.0;
	static const double Z_MAX_SPEED = 2.0;
	static const double YAW_MAX_SPEED = 90.0*M_PI/180.0;

	double roll_;
	double pitch_;
	double yaw_;
	double heading_;

	Point UAVLocation;
	Point GPSLocation;
	bool usingRelativeLocation;

	string world_fn, estimated_fn, marker_fn; //fn -> frame name...

	ofstream myfile;
	bool writeStatistics;

	UAV(string str_ns, bool Statistics);
	virtual ~UAV();
	void imuCallback(const sensor_msgs::Imu::ConstPtr& imu);
	void magneticCallback(const geometry_msgs::Vector3Stamped::ConstPtr& magnetic);
	void locationCallback(const geometry_msgs::PoseStamped::ConstPtr& location);
	void fileCallback(const std_msgs::String::ConstPtr& msg);
};

#endif /* UAV_H_ */
