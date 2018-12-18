/* See the file "LICENSE" for the full license and copyrights governing this code. */
#include "path_primitive/path_primitive_line.h"

#include <cmath>
#include "tf/transform_datatypes.h"

namespace path_primitive {
inline double limitAngle (double angle)
{
	while (angle > M_PI)
		angle -= (M_PI * 2.0);
	while (angle <= -M_PI)
		angle += (M_PI * 2.0);

	return angle;
}

PathPrimitiveLine::PathPrimitiveLine(tf::Pose start, tf::Pose end, tf::Vector3 startVel, tf::Vector3 endVel) {
	setStartPose(start);
	setEndPose(end);
	setStartVelocity(startVel);
	setEndVelocity(endVel);
}

bool PathPrimitiveLine::isInRange(tf::Pose pose) {
	double deltaY = getEndPose().getOrigin().getY() - pose.getOrigin().getY();
	double deltaX = getEndPose().getOrigin().getX() - pose.getOrigin().getX();
	double currentAngle = std::atan2(deltaY, deltaX);
	double stopAngle = tf::getYaw(getEndPose().getRotation());

//	ROS_ERROR("PathPrimitive Linesegment %d, %1.4f, %1.4f", (int)(std::abs(limitAngle(currentAngle - stopAngle)) < M_PI / 2.0), currentAngle, stopAngle);
	return std::abs(limitAngle(currentAngle - stopAngle)) < M_PI / 2.0;

}

PathPrimitiveResult PathPrimitiveLine::project(tf::Pose pose) {
	PathPrimitiveResult result;
	result.curvature = 0;

	//Reference: https://pantherfile.uwm.edu/ericskey/www/231material/hws19.pdf
	double dy = getEndPose().getOrigin().getY() - getStartPose().getOrigin().getY();
	double dx = getEndPose().getOrigin().getX() - getStartPose().getOrigin().getX();
	double x = 0.0, y = 0.0;

	if (fabs(dx) > 0.0001)
	{
		double lineSlope = dy/dx;
		double lineConstant = getEndPose().getOrigin().getY() - (lineSlope * getEndPose().getOrigin().getX());
		x = (pose.getOrigin().getX() + (lineSlope * pose.getOrigin().getY()) - (lineSlope * lineConstant)) / (pow(lineSlope, 2) + 1);
		y = ((lineSlope * pose.getOrigin().getX()) + (pow(lineSlope, 2) * pose.getOrigin().getY()) + lineConstant) / (pow(lineSlope,2) + 1);
	}
	else
	{
		x = getStartPose().getOrigin().getX();
		y = pose.getOrigin().getY();
		//ROS_WARN("Line along Y axis, robot pose [%1.3f][%1.3f]", pose.getOrigin().getX(), pose.getOrigin().getY());
	}

	result.poseOnSegment.setOrigin(tf::Vector3(x, y, 0.0));
	result.poseOnSegment.setRotation(getEndPose().getRotation());

	double distStart = tf::tfDistance(result.poseOnSegment.getOrigin(), getStartPose().getOrigin());
	double distEnd = tf::tfDistance(result.poseOnSegment.getOrigin(), getEndPose().getOrigin());
	double segLen = tf::tfDistance(getStartPose().getOrigin(), getEndPose().getOrigin());

	if (fabs(distStart + distEnd - segLen) > 0.001)
	{
		if (distStart < distEnd)
			result.poseOnSegment = getStartPose();
		else
			result.poseOnSegment = getEndPose();
	}

	// Signed distance, not simple Euclidean distance
	result.distanceError = distanceError(pose);
	result.distanceFromEnd = tf::tfDistance(result.poseOnSegment.getOrigin(), getEndPose().getOrigin());

	double endAngle = tf::getYaw(getEndPose().getRotation());
	double currentAngle = tf::getYaw(pose.getRotation());

	result.headingError = limitAngle(endAngle - currentAngle);
	result.velocity = getStartVelocity().getX();

//	ROS_ERROR("length[%1.2f], lineSlope[%1.2f], lineConstant[%1.2f], x[%1.2f], y[%1.2f], startPose[%1.2f][%1.2f], endPose[%1.2f][%1.2f]", length(), lineSlope, lineConstant, x, y, getStartPose().getOrigin().getX(), getStartPose().getOrigin().getY(), getEndPose().getOrigin().getX(), getEndPose().getOrigin().getY());

	return result;

	// Note: this used to return angle error, now it will return absolute angle.
//	return limitAngle(startAngle - currentAngle);

}

double PathPrimitiveLine::distanceError(tf::Pose pose) {
	double x0 = pose.getOrigin().getX();
	double y0 = pose.getOrigin().getY();
	double x1 = getStartPose().getOrigin().getX();
	double y1 = getStartPose().getOrigin().getY();
	double x2 = getEndPose().getOrigin().getX();
	double y2 = getEndPose().getOrigin().getY();

	//Source: https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
	double numerator = (y2 - y1) * x0 - (x2 - x1) * y0 + x2 * y1 - y2 * x1;
	double denominator = std::sqrt(std::pow(y2 - y1, 2) + std::pow(x2 - x1, 2));

	if(denominator < 1e-9)
		ROS_WARN("Denominator value low in path_primitive_line. Start and End point may be same for path segment");

	return numerator/ denominator;
}

tf::Vector3 PathPrimitiveLine::tangentVector(tf::Point point) {
	tf::Vector3 line = getEndPose().getOrigin() - getStartPose().getOrigin();
	return line;
}

tf::Quaternion PathPrimitiveLine::tangentQuat(tf::Point point) {
	tf::Vector3 line = tangentVector(point);
	double lineAngle = std::atan2(line.getY(), line.getX());
	tf::Quaternion quat = tf::createQuaternionFromYaw(lineAngle);
	return quat;
}

double PathPrimitiveLine::curvatureAt(tf::Pose pose) {
	return 0;
}

double PathPrimitiveLine::getRadius(){
	return  std::numeric_limits<double>::infinity();
}


double PathPrimitiveLine::length() {
	return tf::tfDistance(getStartPose().getOrigin(), getEndPose().getOrigin());
}

tf::Pose PathPrimitiveLine::getPoseAtDistanceAlongSegment(double distance)
{
	if(distance > length())
		return getEndPose();

	if(distance <= 0)
		return getStartPose();

	float ratio = distance / length();
	tf::Pose pose = getStartPose();
	pose.setOrigin((1.0f - ratio) * getStartPose().getOrigin() + ratio * getEndPose().getOrigin());

	return pose;

}

PathPrimitiveType PathPrimitiveLine::getType() {
	return PathPrimitiveType::LINE;
}
}
