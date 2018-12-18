/* See the file "LICENSE" for the full license and copyrights governing this code. */

#pragma once

#include "tf/transform_datatypes.h"
#include "path_primitive/path_primitive_base.h"

namespace path_primitive {
class PathPrimitiveCircle : public PathPrimitiveBase
{
public:
	PathPrimitiveCircle(tf::Pose start, tf::Pose end, tf::Vector3 startVel, tf::Vector3 endVel);
	PathPrimitiveCircle(tf::Pose start, tf::Pose end, tf::Vector3 startVel, tf::Vector3 endVel, tf::Point centerPoint);

	virtual bool isInRange(tf::Pose pose);
	virtual PathPrimitiveResult project(tf::Pose pose);
	virtual double distanceError(tf::Pose pose);
	virtual tf::Vector3 tangentVector(tf::Point point);
	virtual tf::Quaternion tangentQuat(tf::Point point);
	virtual double curvatureAt(tf::Pose pose);
	virtual double length();
	virtual tf::Pose getPoseAtDistanceAlongSegment(double distance);
	virtual PathPrimitiveType getType();
	virtual double getRadius();
	tf::Vector3 getCenter() { return center; }
	void setCenter(tf::Vector3 cen) { center = cen; }
	void setRadius(double r) { radius = r; if(radius != 0.0) curvature = 1.0/radius; }
private:
	tf::Vector3 center;

	/// Radius can be positive or negative (positive is CCW, negative is CW rotation direction from start to end)
	double radius;
	double curvature;
};

}
