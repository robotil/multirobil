#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <math.h>
#include <iostream>
#include <fstream>
#include <time.h>

using namespace std;

int main(int argc, char** argv)
{
	ros::init(argc, argv, "statistics");

	if (argc < 4)
	{
		ROS_ERROR("Please send semple rate, two frame names to compare and world frame... exiting...");
		//ROS::shutdown();
		return 0;
	}
	string sempleRateString = argv[1];
	double sempleRate = atof(sempleRateString.c_str());

	ros::NodeHandle nh;
	tf::TransformListener listener;

	tf::StampedTransform firstTransform;
	tf::StampedTransform secondTransform;
	string first_fn = argv[2]; //fn -> frame name...
	string second_fn = argv[3];
	string world_fn = argv[4];

	ofstream myfile ("statistics.txt");

	ros::Rate rate(sempleRate);
	while (nh.ok())
	{
		double distanceError, hightError, angleError;

		try
		{
			listener.lookupTransform(world_fn, first_fn, ros::Time(0), firstTransform);
			listener.lookupTransform(world_fn, second_fn, ros::Time(0), secondTransform);

			distanceError = sqrt(pow(firstTransform.getOrigin().x() - secondTransform.getOrigin().x(), 2) +
										 pow(firstTransform.getOrigin().y() - secondTransform.getOrigin().y(), 2) +
										 pow(firstTransform.getOrigin().z() - secondTransform.getOrigin().z(), 2));

			hightError = sqrt(pow(firstTransform.getOrigin().z() - secondTransform.getOrigin().z(), 2));

			angleError = sqrt(pow(atan2(firstTransform.getOrigin().y(), firstTransform.getOrigin().x()) -
						 atan2(secondTransform.getOrigin().y(), secondTransform.getOrigin().x()),2));
		}
		catch (const tf::TransformException & ex)
		{
			//ROS_ERROR("%s",ex.what());
		}

		myfile << distanceError << "," << hightError << "," <<  angleError * (180/M_PI) << "\n";

		rate.sleep();
	}

	myfile.close();
	return 0;
};
