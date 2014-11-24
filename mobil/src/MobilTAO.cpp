#include <iostream>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/foreach.hpp>

#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PointStamped.h>
#include <std_msgs/Bool.h>

#include <decision_making/TAO.h>
#include <decision_making/TAOStdProtocols.h>
#include <decision_making/ROSTask.h>
#include <decision_making/DecisionMaking.h>

#include <math.h>

#include "UAV.h"
#include "Bobcat.h"
#include "Utils.h"

using namespace std;
using namespace decision_making;

#define foreach BOOST_FOREACH

#define POINT_BUFFER 0.1
#define FORWARD_SIDES_SPEED 0.2
#define CLIMB_DESCENT_SPEED 0.2

const static double YAW_SPEED = 20/180.0*M_PI;

char Agents[2][3]={
        "r1","r2"
};

/*************************************************************************************************
*** World model for TAO context
**************************************************************************************************/

struct WorldModel : public CallContextParameters
{
    boost::mutex mtx;
//    Position robotPosition;
    std_msgs::Bool usingRelativeLocation;

    string str() const {
        stringstream s;
//        s << "Position: x = " << robotPosition.pose.x << ", y = " << robotPosition.pose.y << "Position: z = " << robotPosition.pose.z << " , h = " << robotPosition.heading << endl;
        s << "UsingRelativeLocation = " << usingRelativeLocation.data << endl;
        return s.str();
    }
};

#define WM TAO_CONTEXT.parameters<WorldModel>()


/*************************************************************************************************
*** Forward declarations
**************************************************************************************************/

void publisheVelocityMsg();
void flyToPoint(Point destPoint);
void yawControl();
void setCircle(bool CWTurn = false);

decision_making::TaskResult BobcatMission(string name, const FSMCallContext& context, EventQueue& eventQueue);
decision_making::TaskResult BobcatFindTeam(string name, const FSMCallContext& context, EventQueue& eventQueue);
decision_making::TaskResult UavMission(string name, const FSMCallContext& context, EventQueue& eventQueue);
decision_making::TaskResult UavFindTeam(string name, const FSMCallContext& context, EventQueue& eventQueue);
decision_making::TaskResult SendPosition(string name, const CallContext& context, EventQueue& eventQueue);

/*************************************************************************************************
*** Variables
**************************************************************************************************/

boost::shared_ptr<WorldModel> wm(new WorldModel());
int robotNum;
robotTypes robotType;
string robotNameSpace;

ros::Publisher velocity_publisher;
geometry_msgs::Twist velocity;

UAV* uav;
Bobcat* bobcat;
Axes axes_;

tf::TransformListener* listener;

/*************************************************************************************************
*** Decision Making
**************************************************************************************************/

TAO_HEADER(BobcatMission)
TAO_HEADER(UavMission)
TAO_HEADER(BobcatFindTeam)
TAO_HEADER(UavFindTeam)

TAO(Mobil)
{
    TAO_PLANS
    {
        Start,
		PerformMission,
		FindTeam,
    }

    TAO_START_PLAN(Start);

    TAO_BGN
    {
        TAO_PLAN(Start)
        {
			TAO_START_CONDITION(true);
			TAO_ALLOCATE_EMPTY;
 			TAO_STOP_CONDITION(true);
 			TAO_TEAM->define("using_relative_location");
 			TAO_TEAM->define("bobcat_position");
			TAO_NEXT(NextFirstReady)
			{
				TAO_NEXT_PLAN(PerformMission);
				TAO_NEXT_PLAN(FindTeam);
            }
        }

        TAO_PLAN(PerformMission)
        {
			TAO_START_CONDITION(wm->usingRelativeLocation.data); // Start condition -> UAV sees Bobcat and uses relative location
			cout<<"TAO_PLAN(Mobil::PerformMission)"<<endl;
			TAO_ALLOCATE(AllocFirstReady) /* TODO: change to allocation protocol by type, and remove type from start condition */
			{
				TAO_SUBPLAN(BobcatMission);
				TAO_SUBPLAN(UavMission);
			}
            TAO_STOP_CONDITION(!wm->usingRelativeLocation.data); // Start condition -> UAV not seeing Bobcat and not using relative location
            TAO_NEXT(NextFirstReady)
			{
                TAO_NEXT_PLAN(FindTeam);
            }
        }

        TAO_PLAN(FindTeam)
        {
            TAO_START_CONDITION(!wm->usingRelativeLocation.data); // Start condition -> UAV not seeing Bobcat and not using relative location
            cout<<"TAO_PLAN(Mobil::FindTeam)"<<endl;
            TAO_ALLOCATE(AllocFirstReady) /* TODO: change to allocation protocol by type, and remove type from start condition */
			{
				TAO_SUBPLAN(BobcatFindTeam);
				TAO_SUBPLAN(UavFindTeam);
			}
            TAO_STOP_CONDITION(wm->usingRelativeLocation.data); // Stop condition -> UAV sees Bobcat and uses relative location
            TAO_NEXT(NextFirstReady)
			{
                TAO_NEXT_PLAN(PerformMission);
            }
        }
    }
    TAO_END
}

TAO(BobcatMission)
{
    TAO_PLANS
	{
        RunTask,
    }

    TAO_START_PLAN(RunTask);

    TAO_BGN
	{
        TAO_PLAN(RunTask)
		{
            TAO_START_CONDITION(robotType == BobcatRobot);
            cout<<"TAO_PLAN(BobcatMission::RunTask)"<<endl;
            TAO_ALLOCATE_EMPTY
			TAO_CALL_TASK(BobcatMission);
            TAO_CALL_TASK(SendPosition);
            TAO_STOP_CONDITION(false); /* TODO: check that plan dies when father hits stop condition and what happens when task ends */
            TAO_NEXT_EMPTY
        }
    }
    TAO_END
}

TAO(BobcatFindTeam)
{
    TAO_PLANS
	{
        RunTask,
    }

    TAO_START_PLAN(RunTask);

    TAO_BGN
	{
        TAO_PLAN(RunTask)
		{
            TAO_START_CONDITION(robotType == BobcatRobot);
            cout<<"TAO_PLAN(BobcatFindTeam::RunTask)"<<endl;
            TAO_ALLOCATE_EMPTY
			TAO_CALL_TASK(BobcatFindTeam);
            TAO_CALL_TASK(SendPosition);
            TAO_STOP_CONDITION(false); /* TODO: check that plan dies when father hits stop condition and what happens when task ends */
            TAO_NEXT_EMPTY
        }
    }
    TAO_END
}

TAO(UavMission)
{
    TAO_PLANS
	{
        RunTask,
    }

    TAO_START_PLAN(RunTask);

    TAO_BGN
	{
        TAO_PLAN(RunTask)
		{
            TAO_START_CONDITION(robotType == UavRobot);
            cout<<"TAO_PLAN(UavMission::RunTask)"<<endl;
            TAO_ALLOCATE_EMPTY
			TAO_CALL_TASK(UavMission);
            TAO_CALL_TASK(SendPosition);
            TAO_STOP_CONDITION(false); /* TODO: check that plan dies when father hits stop condition and what happens when task ends */
            TAO_NEXT_EMPTY
        }
    }
    TAO_END
}

TAO(UavFindTeam)
{
    TAO_PLANS
	{
        RunTask,
    }

    TAO_START_PLAN(RunTask);

    TAO_BGN
	{
        TAO_PLAN(RunTask)
		{
            TAO_START_CONDITION(robotType == UavRobot);
            cout<<"TAO_PLAN(UavFindTeam::RunTask)"<<endl;
            TAO_ALLOCATE_EMPTY
			TAO_CALL_TASK(UavFindTeam);
            TAO_CALL_TASK(SendPosition);
            TAO_STOP_CONDITION(false); /* TODO: check that plan dies when father hits stop condition and what happens when task ends */
            TAO_NEXT_EMPTY
        }
    }
    TAO_END
}

/*************************************************************************************************
*** Utilities
**************************************************************************************************/

void flyToPoint(Point destPoint)
{
	double curr_x = uav->UAVLocation.x;
	double curr_y = uav->UAVLocation.y;
	double curr_z = uav->UAVLocation.z;

	axes_.x.value = 0;
	axes_.y.value = 0;
	axes_.z.value = 0;
	axes_.yaw.value = 0;

	double angleFromNorth = atan2(destPoint.y - curr_y, destPoint.x - curr_x);
	double flightAngle = angleFromNorth - uav->heading_;

	if (curr_x > destPoint.x + POINT_BUFFER || curr_x < destPoint.x - POINT_BUFFER)
	{
		axes_.x.value = cos(flightAngle) * FORWARD_SIDES_SPEED;
	}

	if (curr_y > destPoint.y + POINT_BUFFER || curr_y < destPoint.y - POINT_BUFFER)
	{
		axes_.y.value = sin(flightAngle) * FORWARD_SIDES_SPEED;
	}

	if (curr_z > destPoint.z + POINT_BUFFER || curr_z < destPoint.z - POINT_BUFFER)
	{
		axes_.z.value = CLIMB_DESCENT_SPEED * atan2(destPoint.z - curr_z,
													sqrt(pow(destPoint.x - curr_x,2) +
														 pow(destPoint.y - curr_y,2)));
	}
}

void yawControl()
{
	//tf::TransformListener listener;
	tf::StampedTransform Transform;

	ros::Time now = ros::Time::now();

	try
	{
		if(listener->waitForTransform("base_link", uav->marker_fn, now, ros::Duration(0.1)))
		{
			listener->lookupTransform("base_link", uav->marker_fn, now, Transform);

			double angelToMarker = atan2(Transform.getOrigin().y(), Transform.getOrigin().x());
			//ROS_INFO("angelToMarker = %f", angelToMarker);

			axes_.yaw.value = YAW_SPEED * (angelToMarker > 0 ? 1.0 : -1.0);
			//ROS_INFO("yawControl:: YAW_SPEED: %f",axes_.yaw.value);
		}
	}
	catch(const tf::TransformException & ex)
	{
		axes_.yaw.value = 0;
		ROS_ERROR("yawControl:: %s",ex.what());
	}
}

void publisheVelocityMsg()
{
	velocity.linear.x  = axes_.x.value * axes_.x.max;
	velocity.linear.y  = axes_.y.value * axes_.y.max;
	velocity.linear.z  = axes_.z.value * axes_.z.max;
	velocity.angular.z = axes_.yaw.value * axes_.yaw.max;

	velocity_publisher.publish(velocity);
}

void setCircle(bool CWTurn /* = false */)
{
	axes_.x.value = 0.0;
	axes_.y.value = 0.5;
	axes_.z.value = 0.0;
	axes_.yaw.value = (CWTurn ? -1 : 1) * 0.3;
}

/*************************************************************************************************
*** Tasks
**************************************************************************************************/

decision_making::TaskResult BobcatMission(string name, const CallContext& context, EventQueue& eventQueue)
{
    cout<<("BobcatMission Task started")<<endl;

    axes_.x.value = 0.05;
	axes_.y.value = 0.05;
	axes_.z.value = 0;
	axes_.yaw.value = 0;

    while(eventQueue.isTerminated() == false)
	{
		publisheVelocityMsg();
    }

    cout<<("BobcatMission Task ended")<<endl;
    return decision_making::TaskResult::SUCCESS();
}

decision_making::TaskResult BobcatFindTeam(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
    cout<<("BobcatFindTeam Task started")<<endl;

    axes_.x.value = 0.0;
	axes_.y.value = 0.0;
	axes_.z.value = 0.0;
	axes_.yaw.value = 0.0;

    while(eventQueue.isTerminated() == false)
	{
		publisheVelocityMsg();
    }

    cout<<("BobcatFindTeam Task ended")<<endl;
    return decision_making::TaskResult::SUCCESS();
}

decision_making::TaskResult UavMission(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
	cout<<("UavMission Task started")<<endl;

	Point points[2];
	points[0] = Point(-3.0, 1.0, 1.5);
	points[1] = Point(-2.0, -1.0, 0.5);

	int curentPoint = 0;

	while(eventQueue.isTerminated() == false)
	{
		if(axes_.x.value + axes_.y.value + axes_.z.value == 0.0)
		{
			curentPoint = (curentPoint + 1) % 2;
			ROS_INFO("UAV Changing Point");
		}

		flyToPoint(points[curentPoint]);
		yawControl();
//		setCircle(true);

		publisheVelocityMsg();
	}

	cout<<("UavMission Task ended")<<endl;
	return decision_making::TaskResult::SUCCESS();
}

decision_making::TaskResult UavFindTeam(string name, const FSMCallContext& context, EventQueue& eventQueue)
{
	cout<<("UavFindTeam Task started")<<endl;

	bool bobcatReached = false;
	bool facingNorth = false;

    while(eventQueue.isTerminated() == false)
	{
    	nav_msgs::Odometry odometry = context.team(TAO_CURRENT_TEAM_NAME)->mem("bobcat_position");
		Point bobcatPos(odometry.pose.pose.position.x - 2.5, odometry.pose.pose.position.y, 1.0);

		if ((!bobcatReached && uav->UAVLocation.distanceTo(bobcatPos) > 0.1) || !facingNorth)
		{
			flyToPoint(bobcatPos);

			if (fabs(uav->heading_ * (180/M_PI)) > 10)
			{
				axes_.yaw.value = 0.2;
			}
			else
			{
				facingNorth = true;
				axes_.yaw.value = 0.0;
			}
		}
		else
		{
			bobcatReached = true;
	    	setCircle(true);
		}

		publisheVelocityMsg();
    }

	cout<<("UavFindTeam Task ended")<<endl;
	return decision_making::TaskResult::SUCCESS();
}

decision_making::TaskResult SendPosition(string name, const CallContext& context, EventQueue& eventQueue)
{ // TODO: can be split in two, bobcatSendPos && uavSendPos...
    while (!eventQueue.isTerminated())
    {
        if (robotType == BobcatRobot && bobcat->BobcatLocation.msg) //context.parameters<WorldModel>().robotPosition.msg)
        {
//        	ROS_INFO("SendPosition == BobcatRobot");

            boost::mutex::scoped_lock locker (context.parameters<WorldModel>().mtx);
            context.team(TAO_CURRENT_TEAM_NAME)->mem("bobcat_position") = *(bobcat->BobcatLocation.msg); //*(context.parameters<WorldModel>().robotPosition.msg);

            wm->usingRelativeLocation = context.team(TAO_CURRENT_TEAM_NAME)->mem("using_relative_location");
//            ROS_INFO("SendPosition == BobcatRobot %d", wm->usingRelativeLocation.data ? 1 : 0);
        }
        else if (robotType == UavRobot)
        {
        	boost::mutex::scoped_lock locker (context.parameters<WorldModel>().mtx);
        	wm->usingRelativeLocation.data = uav->usingRelativeLocation;
        	context.team(TAO_CURRENT_TEAM_NAME)->mem("using_relative_location") = context.parameters<WorldModel>().usingRelativeLocation;
        }

        boost::this_thread::sleep(boost::posix_time::milliseconds(250));
    }

    return TaskResult::SUCCESS();
}

/*************************************************************************************************
*** Topic callbacks
**************************************************************************************************/

/*************************************************************************************************
*** Main
**************************************************************************************************/

int main(int argc, char **argv) {

    if (argc < 3) {
        cout << "Usage: ... robotType robotNum" << endl;
        return 1;
    }
    string str_robotType = argv[1];
    string str_robotNum = argv[2];
    robotNameSpace = str_robotType + "_" + str_robotNum;

    ros::init(argc, argv, "mobil_" + robotNameSpace);
    ros_decision_making_init(argc, argv);
    ROS_INFO("Starting Mobil...");

    ros::NodeHandle node;

    robotNum = atoi(argv[1]);
    robotType = str_robotType.compare("UAV") == 0 ? UavRobot : BobcatRobot;
    wm->usingRelativeLocation.data = false;

	axes_.x.value = 0;
	axes_.y.value = 0;
	axes_.z.value = 0;
	axes_.yaw.value = 0;

	velocity_publisher = node.advertise<geometry_msgs::Twist>("/" + robotNameSpace + "/cmd_vel", 10);

    if(robotType == BobcatRobot)
    {
    	bobcat = new Bobcat(robotNameSpace);

    	axes_.x.max = bobcat->X_MAX_SPEED;
    	axes_.y.max = bobcat->Y_MAX_SPEED;
    	axes_.z.max = bobcat->Z_MAX_SPEED;
    	axes_.yaw.max = bobcat->YAW_MAX_SPEED;
    }
    else if(robotType == UavRobot)
	{
    	uav = new UAV(robotNameSpace, true);
    	listener = new tf::TransformListener();

    	axes_.x.max = uav->X_MAX_SPEED;
    	axes_.y.max = uav->Y_MAX_SPEED;
    	axes_.z.max = uav->Z_MAX_SPEED;
    	axes_.yaw.max = uav->YAW_MAX_SPEED;
	}

    decision_making::LocalTasks::registrate("BobcatMission",BobcatMission);
    decision_making::LocalTasks::registrate("BobcatFindTeam",BobcatFindTeam);
    decision_making::LocalTasks::registrate("UavMission",UavMission);
    decision_making::LocalTasks::registrate("UavFindTeam",UavFindTeam);
    decision_making::LocalTasks::registrate("SendPosition",SendPosition);

    ros::AsyncSpinner spinner(2);
    spinner.start();

    RosEventQueue eventQueue;
    CallContext context;
    context.createParameters(wm);

    teamwork::Teammates teammates;
    teamwork::Agent::Ptr self_agent = teammates.add_self(Agents[robotNum]);
    teammates.add(Agents[(robotNum+1)%2]);

    teamwork::SharedMemory db;
    teamwork::Team main_team = teamwork::createMainTeam(db, "main", teammates);
    context.self(self_agent);
    context.team(main_team.ptr());
    context.team(TAO_CURRENT_TEAM_NAME,main_team.ptr());

    eventQueue.async_spin();

    //str_robotType + "_" + str_robotNum + "Starting TAO";
    ROS_INFO("Starting TAO");
    TaoMobil(&context, &eventQueue);
    ROS_INFO("TAO Done");
    eventQueue.close();
    spinner.stop();

    return 0;
}
