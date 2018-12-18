/* See the file "LICENSE" for the full license and copyrights governing this code. */

#pragma once

#include "tf/transform_datatypes.h"
#include "path_primitive_base.h"

namespace path_primitive {
class PathPrimitiveLine : public PathPrimitiveBase
{
public:
	PathPrimitiveLine(tf::Pose start, tf::Pose end, tf::Vector3 startVel, tf::Vector3 endVel);

	virtual bool isInRange(tf::Pose pose);
	virtual PathPrimitiveResult project(tf::Pose pose);
	virtual double distanceError(tf::Pose pose);
	virtual tf::Vector3 tangentVector(tf::Point point);
	virtual tf::Quaternion tangentQuat(tf::Point point);
	virtual double curvatureAt(tf::Pose pose);
	virtual double getRadius();
	virtual double length();
	virtual tf::Pose getPoseAtDistanceAlongSegment(double distance);
	virtual PathPrimitiveType getType();
};

}
