/*
 * Bobcat.cpp
 *
 *  Created on: Aug 14, 2014
 *      Author: ilanlupu
 */

#include "Bobcat.h"

Bobcat::Bobcat(string str_ns)
{
	location_subscriber_ = node_handle_.subscribe("/" + str_ns + "/odom", 10, &Bobcat::locationCallback, this);
}

Bobcat::~Bobcat() {
	// TODO Auto-generated destructor stub
}

void Bobcat::locationCallback(const nav_msgs::Odometry::Ptr odometry)
{
	BobcatLocation.pose.x = odometry->pose.pose.position.x;
	BobcatLocation.pose.y = odometry->pose.pose.position.y;
	BobcatLocation.pose.z = odometry->pose.pose.position.z;
	BobcatLocation.heading = tf::getYaw(odometry->pose.pose.orientation);
	BobcatLocation.msg = odometry;
}
