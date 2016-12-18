/*
 * UAV.cpp
 *
 *  Created on: May 5, 2014
 *      Author: ilan
 */

#include "UAV.h"

UAV::UAV(string str_ns, bool Statistics) {
	ros::NodeHandle params("~");

	roll_ = 0;
	pitch_ = 0;
	yaw_ = 0;
	heading_ = 0;
	usingRelativeLocation = false;

	raw_imu_publisher_ = node_handle_.subscribe("raw_imu", 1, &UAV::imuCallback, this);
	magnetic_subscriber_ = node_handle_.subscribe("magnetic", 1, &UAV::magneticCallback, this);
	location_subscriber_ = node_handle_.subscribe("ground_truth_to_tf/pose", 1, &UAV::locationCallback, this);
	file_subscriber_ = node_handle_.subscribe("close_file", 10, &UAV::fileCallback, this);

	node_handle_.getParam("/" + str_ns + "/world_fn", world_fn);
	node_handle_.getParam("/" + str_ns + "/estimated_fn", estimated_fn);
	node_handle_.getParam("/" + str_ns + "/marker_fn", marker_fn);

	writeStatistics = Statistics;
	myfile.open("/home/ilanlupu/dmw/statistics.txt");
}

UAV::~UAV() {
	// TODO Auto-generated destructor stub
	cout << "Sucker------------------------------------------------------------------------------------------------->";
	myfile.close();
}

void UAV::imuCallback(const sensor_msgs::Imu::ConstPtr& imu)
{
	tf::Quaternion q(imu->orientation.x,
					 imu->orientation.y,
					 imu->orientation.z,
					 imu->orientation.w);

	tf::Matrix3x3 m(q);

	m.getEulerYPR(yaw_, pitch_, roll_);
}

void UAV::magneticCallback(const geometry_msgs::Vector3Stamped::ConstPtr& magnetic)
{
	float headX;
	float headY;
	float cos_roll;
	float sin_roll;
	float cos_pitch;
	float sin_pitch;

	cos_roll = cos(roll_);
	sin_roll = 1  - (cos_roll * cos_roll);
	cos_pitch = cos(pitch_);
	sin_pitch = 1  - (cos_pitch * cos_pitch);

	// Tilt compensated magnetic field X component:
	headX = magnetic->vector.x*cos_pitch+magnetic->vector.y*sin_roll*sin_pitch+magnetic->vector.z*cos_roll*sin_pitch;
	// Tilt compensated magnetic field Y component:
	headY = magnetic->vector.y*cos_roll-magnetic->vector.z*sin_roll;
	// magnetic heading
	heading_ = atan2(-headY,headX);

//	// Declination correction (if supplied)
//	if( fabs(_declination) > 0.0 )
//	{
//		heading = heading + _declination;
//		if (heading > M_PI)    // Angle normalization (-180 deg, 180 deg)
//			heading -= (2.0 * M_PI);
//		else if (heading < -M_PI)
//			heading += (2.0 * M_PI);
//	}

//	heading_ = (360 + (int)(heading_*(180/M_PI))) % 360; Convert to degrees
}

void UAV::locationCallback(const geometry_msgs::PoseStamped::ConstPtr& location)
{
	double distanceError, hightError, angleError;

	GPSLocation.x = location->pose.position.x;
	GPSLocation.y = location->pose.position.y;
	GPSLocation.z = location->pose.position.z;

	ros::Time now = ros::Time::now();

	try
	{
		if(listener.waitForTransform(world_fn, estimated_fn, now, ros::Duration(0.10)))
		{
			//ROS_INFO("estimateLocation::world->estimated: transformed!");

			listener.lookupTransform(world_fn, estimated_fn, now, transform);

			UAVLocation.x = transform.getOrigin().x();
			UAVLocation.y = transform.getOrigin().y();
			UAVLocation.z = transform.getOrigin().z();

			usingRelativeLocation = true;
		}
		else
		{
			//ROS_INFO("estimateLocation::world->estimated: couldn't transform");

			UAVLocation.x = GPSLocation.x;
			UAVLocation.y = GPSLocation.y;
			UAVLocation.z = GPSLocation.z;

			usingRelativeLocation = false;
		}

		if(writeStatistics)
		{
			if(listener.waitForTransform("base_link", estimated_fn, now, ros::Duration(0.10)))
			{
				listener.lookupTransform("base_link", estimated_fn, now, transform);

				distanceError = sqrt(pow(transform.getOrigin().x(), 2) +
									 pow(transform.getOrigin().y(), 2) +
									 pow(transform.getOrigin().z(), 2));

//				tf::StampedTransform tmp1, tmp2;
//				listener.lookupTransform(world_fn, "base_link", now, tmp1);
//				listener.lookupTransform(world_fn, estimated_fn, now, tmp2);
//
//				angleError = sqrt(pow(atan2(tmp2.getOrigin().y(), tmp2.getOrigin().x()) -
//									  atan2(tmp1.getOrigin().y(), tmp1.getOrigin().x()),2));
			}
			else
			{
			distanceError = sqrt(pow(UAVLocation.x - GPSLocation.x, 2) +
								 pow(UAVLocation.y - GPSLocation.y, 2) +
								 pow(UAVLocation.z - GPSLocation.z, 2));
			}

			hightError = sqrt(pow(UAVLocation.z - GPSLocation.z, 2));

			angleError = sqrt(pow(atan2(UAVLocation.y, UAVLocation.x) -
								  atan2(GPSLocation.y, GPSLocation.x),2));

			if (myfile.is_open())
			{
				myfile << distanceError << "," << hightError << "," <<  angleError * (180/M_PI) << "," << (usingRelativeLocation ? "1" : "0") << "\n";
			}
		}
	}
	catch (const tf::TransformException & ex)
	{
		//ROS_ERROR("estimateLocation:: %s",ex.what());
	}

//	double heightFromGround;
//	tf::StampedTransform transform;
//	ros::Time now = ros::Time::now();
//
//	if(listener.waitForTransform("/world", "/uav_estimated_location", now, ros::Duration(0.05)))
//	{
//		listener.lookupTransform("/world", "/uav_estimated_location", now, transform);
//		heightFromGround = transform.getOrigin().z();
//		ROS_INFO("world->estimated: x: %f, y: %f, z: %f",transform.getOrigin().x(),transform.getOrigin().y(),transform.getOrigin().z());
//	}
//	else
//	{
//		ROS_INFO("world->estimated: couldn't transform");
//	    heightFromGround = height->pose.position.z;
//	}
//
//    if (heightFromGround > flyHeight - HEIGHT_BUFFER &&
//    	heightFromGround < flyHeight + HEIGHT_BUFFER)
//    {
//    	axes_.z.value = 0;
//    }
//
//    else if (heightFromGround < flyHeight)
//    {
//    	axes_.z.value = CLIMB_DESCENT_SPEED;
//	}
//
//    else
//    {
//    	axes_.z.value = -1 * CLIMB_DESCENT_SPEED;
//    }
}

void UAV::fileCallback(const std_msgs::String::ConstPtr& msg)
{
	myfile.close();
}
