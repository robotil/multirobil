#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <math.h>

using namespace std;

double axisRotationAngle;
ros::Subscriber magnetic_subscriber;

string getParam(const string& paramName, ros::NodeHandle& nh);
void magneticCallback(const geometry_msgs::Vector3Stamped::ConstPtr& magnetic);

int main(int argc, char** argv){

  if (argc < 2) {
	  cout << "Usage: ... name space" << endl;
	  return 1;
  }
  string str_ns = argv[1];

  ros::init(argc, argv, "shared_coordinates");

  ros::NodeHandle nh;
  tf::TransformListener listener;
  tf::StampedTransform anchorTransform, robotTransform;
  tf::TransformBroadcaster br;
  tf::Transform uavTransform;
  tf::Quaternion q;

  double uavLocationX ,uavLocationY, uavLocationZ;

  string world_fn, anchor_fn; //fn -> frame name...
  string robot_fn, marker_fn;
  string estimated_fn;
  string magnetic_topic;
  double correctionXY, correctionZ;

  nh.getParam("/" + str_ns + "/shared_coordinates/world_fn", world_fn);
  nh.getParam("/" + str_ns + "/shared_coordinates/marker_fn", marker_fn);
  nh.getParam("/" + str_ns + "/shared_coordinates/estimated_fn", estimated_fn);
  nh.getParam("/" + str_ns + "/shared_coordinates/anchor_fn", anchor_fn);
  nh.getParam("/" + str_ns + "/shared_coordinates/robot_fn", robot_fn);
  nh.getParam("/" + str_ns + "/shared_coordinates/magnetic_topic", magnetic_topic);
  nh.getParam("/" + str_ns + "/shared_coordinates/correctionXY", correctionXY);
  nh.getParam("/" + str_ns + "/shared_coordinates/correctionZ", correctionZ);

  magnetic_subscriber = nh.subscribe(magnetic_topic, 10, &magneticCallback);

  ros::Rate rate(10.0);
  while (nh.ok())
  {
    try
    {
		// Align the robot coordinate system
		string aligned_robot_fn = "aligned_" + robot_fn;
		uavTransform.setOrigin(tf::Vector3(0, 0, 0));
		q.setRPY(0, 0, axisRotationAngle);
		uavTransform.setRotation(q);
		br.sendTransform(tf::StampedTransform(uavTransform, ros::Time::now(), robot_fn, aligned_robot_fn));

		// Fined the transforms between the world -> marker -> robot
		// Using the transform from the robot to the maker in order to avoid problems with rotated markers
    	listener.lookupTransform(aligned_robot_fn, marker_fn, ros::Time(0), robotTransform);
    	listener.lookupTransform(world_fn, anchor_fn, ros::Time(0), anchorTransform);

    	uavLocationX = anchorTransform.getOrigin().x() - robotTransform.getOrigin().x();
    	uavLocationY = anchorTransform.getOrigin().y() - robotTransform.getOrigin().y();
    	uavLocationZ = anchorTransform.getOrigin().z() - robotTransform.getOrigin().z();

    	// Adding correction vector
    	double uavVectorLenght = sqrt(pow(uavLocationX,2) + pow(uavLocationY,2));

    	uavLocationX += correctionXY * (uavLocationX/uavVectorLenght);
    	uavLocationY += correctionXY * (uavLocationY/uavVectorLenght);
    	uavLocationZ += correctionZ;

    	// Publishing a new estimated location for the robot.
		uavTransform.setOrigin( tf::Vector3(uavLocationX, uavLocationY, uavLocationZ));
		q.setRPY(0, 0, 0);
		uavTransform.setRotation(q);
		br.sendTransform(tf::StampedTransform(uavTransform, ros::Time::now(), world_fn, estimated_fn));
    }
    catch (const tf::TransformException & ex)
    {
    	//ROS_ERROR("shared_coordinates:: %s",ex.what());
    }

    ros::spinOnce();
    rate.sleep();
  }
  return 0;
};

string getParam(const string& paramName, ros::NodeHandle& nh)
{
	string paramValue;

    if (!nh.getParam(paramName, paramValue))
    {
    	ROS_ERROR("%s is missing, exiting node", paramName.c_str());
    	ros::shutdown();
    }

	return paramValue;
}

void magneticCallback(const geometry_msgs::Vector3Stamped::ConstPtr& magnetic)
{
	axisRotationAngle = atan2(magnetic->vector.y,magnetic->vector.x);
	//ROS_INFO("axisRotationAngle: %f",axisRotationAngle);
}
