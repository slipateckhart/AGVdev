/* See the file "LICENSE" for the full license and copyrights governing this code. */
#include "path_primitive/path_primitive_circle.h"

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

PathPrimitiveCircle::PathPrimitiveCircle(tf::Pose start, tf::Pose end, tf::Vector3 startVel, tf::Vector3 endVel) {
	setStartPose(start);
	setEndPose(end);
	setStartVelocity(startVel);
	setEndVelocity(endVel);
//	center = cen;
	//radius = tf::tfDistance(cen, start.getOrigin());
	radius = 0;
	curvature = 0;

	// This initialization will need to be redeveloped when we switch to the definition of a circle as center + start + end
	double startAngle = tf::getYaw(getStartPose().getRotation());
	double stopAngle = tf::getYaw(getEndPose().getRotation());
	double cosDiff = std::cos(startAngle) - std::cos(stopAngle);
	double sinDiff = std::sin(startAngle) - std::sin(stopAngle);

	if (sinDiff != 0.0 && cosDiff != 0.0)
	{
		double radius_y = (getEndPose().getOrigin().getY() - getStartPose().getOrigin().getY()) / (cosDiff);
		double radius_x = (getEndPose().getOrigin().getX() - getStartPose().getOrigin().getX()) / (sinDiff);

		if (std::abs(radius_y) <= std::abs(radius_x) + 0.1 && std::abs(radius_y) >= std::abs(radius_x) - 0.1)
		{
			radius = radius_y;
			center.setValue(getStartPose().getOrigin().getX() + radius_x * std::sin(startAngle),
					getStartPose().getOrigin().getY() + radius_y * std::cos(startAngle),
					0.0);

		}
		else
		{
			ROS_ERROR("Circle Primitive Construct Fail %1.5f %1.5f", radius_x, radius_y);
		}
	}
	if(std::fabs(radius) > 0.00000001)
		curvature = 1.0/radius;
}


PathPrimitiveCircle::PathPrimitiveCircle(tf::Pose start, tf::Pose end, tf::Vector3 startVel, tf::Vector3 endVel, tf::Point centerPoint) {
	setStartPose(start);
	setEndPose(end);
	setStartVelocity(startVel);
	setEndVelocity(endVel);
    center = centerPoint;
	//radius = tf::tfDistance(cen, start.getOrigin());
	radius = 0;
	curvature = 0;

	// This initialization will need to be redeveloped when we switch to the definition of a circle as center + start + end
	double startAngle = tf::getYaw(getStartPose().getRotation());
	double stopAngle = tf::getYaw(getEndPose().getRotation());
	double cosDiff = std::cos(startAngle) - std::cos(stopAngle);
	double sinDiff = std::sin(startAngle) - std::sin(stopAngle);

	if (sinDiff != 0.0 && cosDiff != 0.0)
	{
		double radius_y = (getEndPose().getOrigin().getY() - getStartPose().getOrigin().getY()) / (cosDiff);
		double radius_x = (getEndPose().getOrigin().getX() - getStartPose().getOrigin().getX()) / (sinDiff);

		if (std::abs(radius_y) <= std::abs(radius_x) + 0.1 && std::abs(radius_y) >= std::abs(radius_x) - 0.1)
		{
			radius = radius_y;

		}
		else
		{
			ROS_ERROR("Circle Primitive Construct Fail %1.5f %1.5f", radius_x, radius_y);
		}
	}
	if(std::fabs(radius) > 0.00000001)
		curvature = 1.0/radius;
}

bool PathPrimitiveCircle::isInRange(tf::Pose pose) {
	double deltaY = getEndPose().getOrigin().getY() - pose.getOrigin().getY();
	double deltaX = getEndPose().getOrigin().getX() - pose.getOrigin().getX();
	double currentAngle = std::atan2(deltaY, deltaX);
	double stopAngle = tf::getYaw(getEndPose().getRotation());

//	ROS_ERROR("PathPrimitive Circle %d, %1.4f, %1.4f", (int)(std::abs(limitAngle(currentAngle - stopAngle)) < M_PI / 2.0), currentAngle, stopAngle);

	return std::abs(limitAngle(currentAngle - stopAngle)) < M_PI / 2.0;
}

PathPrimitiveResult PathPrimitiveCircle::project(tf::Pose pose) {

	//Calculate x,y of pose on circle
	//Reference: http://stackoverflow.com/questions/300871/best-way-to-find-a-point-on-a-circle-closest-to-a-given-point

	double currentYaw = tf::getYaw(pose.getRotation());
	double tangentYaw;
	PathPrimitiveResult result;
	if (isInRange(pose) == true)
	{
		tf::Vector3 vector = pose.getOrigin() - center;
		vector.setZ(0);
		double vectorMag = tf::tfDistance(vector, tf::Vector3(0, 0, 0));
		double x = center.getX() + (vector.getX() / vectorMag * std::fabs(radius));
		double y = center.getY() + (vector.getY() / vectorMag * std::fabs(radius));

		result.poseOnSegment.setOrigin(tf::Vector3(x, y, 0.0));

		//Calculate heading of pose on circle
		double radiusYaw = std::atan2(pose.getOrigin().getY() - center.getY(), (pose.getOrigin().getX() - center.getX()));
		tangentYaw = radiusYaw + std::copysign(M_PI / 2.0, limitAngle(tf::getYaw(getEndPose().getRotation()) - tf::getYaw(getStartPose().getRotation())));

		result.poseOnSegment.setRotation(tf::createQuaternionFromYaw(tangentYaw));
	}
	else
	{
		tf::Pose nearestPose;

		if (tf::tfDistance(getStartPose().getOrigin(), pose.getOrigin()) < tf::tfDistance(getEndPose().getOrigin(), pose.getOrigin()))
		{
			nearestPose = getStartPose();
		}
		else
		{
			nearestPose = getEndPose();
		}
		double lineSlope = tan(tf::getYaw(nearestPose.getRotation()));
		double lineConstant = nearestPose.getOrigin().getY() - (lineSlope * nearestPose.getOrigin().getX());
		double x = (pose.getOrigin().getX() + (lineSlope * pose.getOrigin().getY()) - (lineSlope * lineConstant)) / (pow(lineSlope, 2) + 1);
		double y = ((lineSlope * pose.getOrigin().getX()) + (pow(lineSlope, 2) * pose.getOrigin().getY()) + lineConstant) / (pow(lineSlope,2) + 1);

		//double x = nearestPose.getOrigin().getX();
		//double y = nearestPose.getOrigin().getY();

		result.poseOnSegment.setOrigin(tf::Vector3(x, y, 0.0));
		result.poseOnSegment.setRotation(nearestPose.getRotation());
		tangentYaw = tf::getYaw(nearestPose.getRotation());
	}

	result.curvature = curvature;

	result.distanceError = distanceError(pose);
	result.headingError = limitAngle(tangentYaw - currentYaw);

	tf::Vector3 v1, v2;
	v1 = center - result.poseOnSegment.getOrigin();
	v2 = center - getEndPose().getOrigin();

	double arcAngle = tf::tfAngle(v1, v2);
	result.distanceFromEnd = std::fabs(radius * arcAngle); //l=r*theta

	double percent = result.distanceFromEnd / length();
	result.velocity = getStartVelocity().getX();

	return result;

}

double PathPrimitiveCircle::distanceError(tf::Pose pose) {


	double distance = 0.0;
	tf::Point startPoint = getStartPose().getOrigin();
	tf::Point currentPoint = pose.getOrigin();

	//	if (checkStartPoseCircle())
//	{

	distance = tf::tfDistance(currentPoint, center) - std::fabs(radius);
			//std::hypot(currentPoint.getX() - center.getX(), currentPoint.getY() - center.getY()) - std::fabs(radius);
	distance *= std::copysign(1.0, limitAngle(tf::getYaw(getEndPose().getRotation()) - tf::getYaw(getStartPose().getRotation())));
//	}
//	else
//	{
//		// Extend line back.
//		// Find distance using ax + by + c = 0
//		// Source: https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
//		double a = startPoint.getX() - origin_circle.getX();
//		double b = startPoint.getY() - origin_circle.getY();
//		double c = -1.0 * (a * startPoint.getX() + b * startPoint.getY());
//
//		distance = (a * currentPoint.getX() + b * currentPoint.getY() + c) / std::hypot(a, b);
//	}
	return distance;

}

tf::Vector3 PathPrimitiveCircle::tangentVector(tf::Point point) {
	tf::Vector3 line(1.0,0,0);
	line = tf::quatRotate(tangentQuat(point), line);
	return line;
}

tf::Quaternion PathPrimitiveCircle::tangentQuat(tf::Point point) {
	double radiusYaw = std::atan2(point.getY() - center.getY(), (point.getX() - center.getX()));
	double tangentYaw = radiusYaw + std::copysign(M_PI / 2.0, limitAngle(tf::getYaw(getEndPose().getRotation()) - tf::getYaw(getStartPose().getRotation())));
	tf::Quaternion quat = tf::Quaternion(tf::Vector3(0,0,1.0), tangentYaw);
	return quat;
}

double PathPrimitiveCircle::curvatureAt(tf::Pose pose) {
	if(radius != 0.0)
		return 1.0/radius;
	return 0;
}

double PathPrimitiveCircle::getRadius(){
	return radius;
}

double PathPrimitiveCircle::length() {
	tf::Vector3 v1, v2;
	v1 = center - getStartPose().getOrigin();
	v2 = center - getEndPose().getOrigin();

	double arcAngle = tf::tfAngle(v1, v2);
//	ROS_INFO("Distance: %1.2f", fabs(radiusCircle * arcAngle));
	return std::fabs(radius* arcAngle); //l=r*theta

}

tf::Pose PathPrimitiveCircle::getPoseAtDistanceAlongSegment(double distance)
{

	if(distance > length())
		return getEndPose();

	if(distance <= 0)
		return getStartPose();

	tf::Vector3 v1, v2;
	v1 = getStartPose().getOrigin() - center;
	v2 = getEndPose().getOrigin() - center;

	float theta = distance/radius;

	tf::Quaternion dq = tf::createQuaternionFromYaw(theta);

	tf::Transform rotate;
	rotate.setRotation(dq);

	v1 = rotate * v1;

	tf::Quaternion q = getStartPose().getRotation() * dq;


	tf::Pose pose;

	tf::Vector3 p = center + v1;
	p.setZ(0);
	pose.setOrigin( p );
	pose.setRotation( q );

	return pose;

}

PathPrimitiveType PathPrimitiveCircle::getType() {
	return PathPrimitiveType::CIRCLE;
}
}
