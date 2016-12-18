/*
 * Utils.h
 *
 *  Created on: Aug 10, 2014
 *      Author: ilanlupu
 */

#ifndef UTILS_H_
#define UTILS_H_

#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <math.h>

enum robotTypes { UavRobot, BobcatRobot };

struct Point {
    double x;
    double y;
    double z;

    Point() { x = 0; y = 0; z = 0;}
    Point(double _x, double _y, double _z) { x = _x; y = _y; z = _z;}

    double distanceTo(Point destination)
    {
    	return (double)(sqrt(pow(x-destination.x,2) + pow(y-destination.y,2) + pow(z-destination.z,2)));
    }

    tf::Vector3 toVector3() {
        return tf::Vector3(x, y, z);
    }
};

struct Position {
    Point pose;
    double heading;
    nav_msgs::Odometry::Ptr msg;
};

struct Axis
	{
		double value;
		double max;
	};

struct Axes
	{
		Axis x;
		Axis y;
		Axis z;
	    Axis yaw;
	};

const double r2d=57.2957795;
const double d2r=0.0174532925;

//double getAngle(const tf::Vector3& vec){
//    tf::Vector3 v (1,0,0);
//    return vec.angle(v) * (vec.y()<0?-1:1);
//}
//
//tf::Vector3 getPoliar(double angle, double len){
//    tf::Vector3 v (len, 0, 0);
//    v = v.rotate(tf::Vector3(0, 0, 1), angle);
//    return v;
//}
//double normilizeAngle(double angle) {
//    return getAngle(getPoliar(angle,1));
//}

#endif /* UTILS_H_ */
