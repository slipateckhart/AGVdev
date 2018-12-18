/* See the file "LICENSE" for the full license and copyrights governing this code. */

#pragma once

#include "tf/transform_datatypes.h"
#include "path_primitive_base.h"
#include "tinyspline/tinysplinecpp.h"

namespace path_primitive {

struct BezierCacheInfo
{
	tf::Vector3 point;
	double accumulatedDistance;

};

class PathPrimitiveBezier : public PathPrimitiveBase
{
public:
	PathPrimitiveBezier(tf::Pose _cp0, tf::Pose _cp1, tf::Pose _cp2, tf::Pose _cp3, tf::Vector3 startVel, tf::Vector3 endVel);

	virtual bool isInRange(tf::Pose pose);
	virtual PathPrimitiveResult project(tf::Pose pose);
	virtual double distanceError(tf::Pose pose);
	virtual tf::Vector3 tangentVector(tf::Point point);
	virtual tf::Quaternion tangentQuat(tf::Point point);
	virtual double curvatureAt(tf::Pose pose);
	virtual double length();
	virtual tf::Pose getPoseAtDistanceAlongSegment(double distance);
	virtual PathPrimitiveType getType();

	virtual ~PathPrimitiveBezier();

	tf::Vector3 evaluate(double t);
	void calculateLength();

	tf::Vector3 tangentVector(double u);
	tf::Quaternion tangentQuat(double u);
	double getLengthAt(double u);
	double getParameterForLength(double s);
	double curvatureAt(double u);

	double distanceToLine(tf::Vector3 point, tf::Vector3 line0, tf::Vector3 line1);

private:
	tf::Pose cp0, cp1, cp2, cp3;
	TsBSpline *spline;
	double calculatedLength;

	int cacheCount;
	std::vector<double> lengthToParameterMap;
	std::vector<double> parameterToLengthMap;
};

}
