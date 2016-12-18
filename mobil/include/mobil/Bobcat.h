/*
 * Bobcat.h
 *
 *  Created on: Aug 10, 2014
 *      Author: ilanlupu
 */

#ifndef BOBCAT_H_
#define BOBCAT_H_

#include <ros/ros.h>
#include "Utils.h"

using namespace std;

class Bobcat {
private:
	ros::NodeHandle node_handle_;
	ros::Subscriber location_subscriber_;

public:
	static const double X_MAX_SPEED = 5.0;
	static const double Y_MAX_SPEED = 5.0;
	static const double Z_MAX_SPEED = 0.0;
	static const double YAW_MAX_SPEED = 90.0*M_PI/180.0;

	Position BobcatLocation;

	Bobcat(string str_ns);
	virtual ~Bobcat();
	void locationCallback(const nav_msgs::Odometry::Ptr odometry);
};

#endif /* BOBCAT_H_ */
