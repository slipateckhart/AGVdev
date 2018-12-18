/* See the file "LICENSE" for the full license and copyrights governing this code. */
#include "path_primitive/path_primitive_bezier.h"

#include <cmath>
#include <stdio.h>
#include "tf/transform_datatypes.h"
#include "tinyspline/tinysplinecpp.h"

namespace path_primitive {
inline double limitAngle (double angle)
{
	while (angle > M_PI)
		angle -= (M_PI * 2.0);
	while (angle <= -M_PI)
		angle += (M_PI * 2.0);

	return angle;
}

PathPrimitiveBezier::PathPrimitiveBezier(tf::Pose _cp0, tf::Pose _cp1, tf::Pose _cp2, tf::Pose _cp3, tf::Vector3 startVel, tf::Vector3 endVel) {
	spline = nullptr;
	setStartPose(_cp0);
	setEndPose(_cp3);
	cp0 = _cp0;
	cp1 = _cp1;
	cp2 = _cp2;
	cp3 = _cp3;
	setStartVelocity(startVel);
	setEndVelocity(endVel);

	spline = new TsBSpline(3, 3, 4, TS_CLAMPED );
	float *a = spline->ctrlp();
	a[0] = cp0.getOrigin().getX();
	a[1] = cp0.getOrigin().getY();
	a[2] = cp0.getOrigin().getZ();

	a[3] = cp1.getOrigin().getX();
	a[4] = cp1.getOrigin().getY();
	a[5] = cp1.getOrigin().getZ();

	a[6] = cp2.getOrigin().getX();
	a[7] = cp2.getOrigin().getY();
	a[8] = cp2.getOrigin().getZ();

	a[9] = cp3.getOrigin().getX();
	a[10] = cp3.getOrigin().getY();
	a[11] = cp3.getOrigin().getZ();

	calculateLength();

}

bool PathPrimitiveBezier::isInRange(tf::Pose pose) {


	// Check to see if the point is past the plane of the start point and behind the plane of the end point
	// This check will fail in some extreme cases, like exaggerated s-curves.
	tf::Vector3 startTangent = cp1.getOrigin() - cp0.getOrigin();
	tf::Vector3 endTangent = cp2.getOrigin() - cp3.getOrigin();

	tf::Vector3 curStart = pose.getOrigin() - cp0.getOrigin();
	tf::Vector3 curEnd = pose.getOrigin() - cp3.getOrigin();

	float ds = startTangent.dot(curStart);
	float de = endTangent.dot(curEnd);

	return (ds >= 0) && (de >= 0);

}

PathPrimitiveResult PathPrimitiveBezier::project(tf::Pose pose) {
	PathPrimitiveResult result;

	tf::Vector3 p = pose.getOrigin();

	double minParam = -1.0;
	tf::Vector3 minPoint;
	double minDist = std::numeric_limits<double>::max();

	double u;
	double step = 1.0/(double)(1 << 16);
	double distance;
	tf::Vector3 sp;
	for(u=0.0; u <= 1.0; u += step)
	{
		sp = evaluate(u);
		distance = sp.distance2(p);
		if(distance < minDist)
		{
			minDist = distance;
			minPoint = sp;
			minParam = u;
		}
	}

	minDist = sqrt(minDist);

	result.poseOnSegment.setOrigin(minPoint);
	result.poseOnSegment.setRotation(tangentQuat(minParam));

	result.parameter = minParam;

	tf::Vector3 tangent = tangentVector(minParam);
	double side = 1.0;
	double lineError = distanceToLine(p, minPoint, minPoint + tangent);
	if(lineError < 0)
		side = -1.0;

	result.distanceError = minDist*side;
	double endDistance = length() - getLengthAt(minParam);
	result.distanceFromEnd = endDistance;

	tf::Quaternion targetRot = result.poseOnSegment.getRotation();
	tf::Quaternion curRot = pose.getRotation();
	tfScalar roll, pitch, yaw;
	tf::Matrix3x3(targetRot - curRot).getRPY(roll, pitch, yaw);

	result.headingError = yaw;

	result.curvature = curvatureAt(minParam);

	return result;

}

double PathPrimitiveBezier::distanceError(tf::Pose pose) {
	return 0;
}

tf::Vector3 PathPrimitiveBezier::tangentVector(tf::Point point) {
	tf::Vector3 tangent;

	tf::Pose pose;
	pose.setOrigin(point);
	PathPrimitiveResult result = project(pose);
	tangent = tangentVector(result.parameter);

	return tangent;
}

tf::Vector3 PathPrimitiveBezier::tangentVector(double u) {
	tf::Vector3 tangent;

	TsBSpline *splineCopy = new TsBSpline(*spline);

	splineCopy->split(u);

	splineCopy->toBeziers();

	// Debug print statements
/*
	printf("tangentVector beziers nCtrlp: %d, nKnot: %d\n", splineCopy->nCtrlp(), splineCopy->nKnots());
	int i,j;
	for(i = 0; i < splineCopy->nCtrlp(); i++)
	{
		printf("ctrlp %d: ( ", i);
		for(j = 0; j < splineCopy->dim(); j++)
		{
			printf("%1.2f ", splineCopy->ctrlp()[i*splineCopy->dim() + j]);
		}
		printf(")\n");
	}

	printf("knots: ( ");

	for(i = 0; i < splineCopy->nKnots(); i++)
	{
		printf("%1.2f ", splineCopy->knots()[i]);
	}
	printf(")\n");
*/
	float* p = splineCopy->ctrlp();
	int dim = splineCopy->dim();
	int deg = splineCopy->deg();
	// Access the points that are after the split point, meaning p0 and p1 of the second bezier curve

	int offset0 = (deg-1)*dim;
	int offset1 = deg*dim;

	// Special case for the very beginning of the curve
	if(u == 0.0)
	{
		offset0 = 0;
		offset1 = dim;
	}

	tf::Vector3 p0(p[offset0+0], p[offset0+1], p[offset0+2]);
	tf::Vector3 p1(p[offset1+0], p[offset1+1], p[offset1+2]);

	tangent = p1-p0;
	tangent.normalize();
//	printf("tangent: ( %1.2f, %1.2f, %1.2f ) \n", tangent.getX(), tangent.getY(), tangent.getZ());


	return tangent;
}

tf::Quaternion PathPrimitiveBezier::tangentQuat(tf::Point point) {
	tf::Quaternion quat = tf::createQuaternionFromYaw(0);
	return quat;
}

double PathPrimitiveBezier::curvatureAt(tf::Pose pose) {

	PathPrimitiveResult result = project(pose);

	return result.curvature;
}

double PathPrimitiveBezier::length() {

	return calculatedLength;
}

PathPrimitiveType PathPrimitiveBezier::getType() {
	return PathPrimitiveType::BEZIER;
}

PathPrimitiveBezier::~PathPrimitiveBezier() {
	if(spline != nullptr)
		delete spline;
	spline = nullptr;
}

tf::Vector3 PathPrimitiveBezier::evaluate(double t) {
	TsDeBoorNet net = spline->evaluate(t);
	float *r = net.result();
	tf::Vector3 result = tf::Vector3(r[0], r[1], r[2]);
	return result;
}

void PathPrimitiveBezier::calculateLength() {
	tf::Vector3 p0, p1;
	double distance = 0.0;

	cacheCount = 1 << 6;
	double step = 1.0/(double)cacheCount;

	double u;
	int index = 0;

	int counter = 0;

	for(int i = 0; i <= cacheCount; i++)
	{
		lengthToParameterMap.push_back(0.0);
	}

	parameterToLengthMap.push_back(0.0);

	int subStepCount = 1 << 6;
	double subStepSize = 1.0/(double)subStepCount;

	// Advance in small substeps for better precision
	for(u = 0.0; u < 1.0; )
	{
		for(int i = 0; i < subStepCount; i++ )
		{
			u += step * subStepSize;
			p1 = evaluate(u);
			distance += p1.distance(p0);
			p0 = p1;
		}
//		printf("ptol %1.17f:%1.17f\n", u, distance);
		parameterToLengthMap.push_back(distance);
	}

	calculatedLength = distance;
	distance = 0;


	int lastIndex = 0;
	counter = 0;
	// Advance in small substeps
	for(u = 0.0; u <= 1.0; u += step*subStepSize)
	{
		p1 = evaluate(u);
		if(u > 0.0)
		{
			distance += p1.distance(p0);
		}
		// compute index using ceil
		index = ceil((distance/calculatedLength) * cacheCount);
		lengthToParameterMap[index] = u;

		lastIndex = index;

		counter++;
		p0 = p1;
	}
//	printf("ltop %1.17f:%1.17f\n", calculatedLength * (double)lastIndex/(double)cacheCount, lengthToParameterMap[lastIndex]);

}

double PathPrimitiveBezier::getLengthAt(double u) {
	if(u < 0.000000001)
		return 0;
	int index1 = floor(cacheCount*u);
	if(index1 >= cacheCount)
		return calculatedLength;

	double l1 = parameterToLengthMap[index1];
	double l2 = parameterToLengthMap[index1 + 1];
	double alpha = (u - ((double)index1 / (double)cacheCount)) * (double)cacheCount;

	double result = (1.0 - alpha)*l1 + alpha * l2;
	return result;
}

double PathPrimitiveBezier::getParameterForLength(double s) {
	if(s <= 0.0)
		return 0;
	int index = floor(cacheCount * s / calculatedLength);
	if(index >= cacheCount)
		return 1.0;

	double l1 = lengthToParameterMap[index];
	double l2 = lengthToParameterMap[index + 1];

	double alpha = ((s/calculatedLength) - ((double)index / (double)cacheCount)) * (double)cacheCount;

	double result = (1.0 - alpha)*l1 + alpha * l2;
	return result;
}


tf::Quaternion PathPrimitiveBezier::tangentQuat(double u) {
	tf::Vector3 tangent = tangentVector(u);
	tangent.setZ(0);
	double yaw = atan2(tangent.y(), tangent.x());
	return tf::createQuaternionFromYaw(yaw);
}

double PathPrimitiveBezier::curvatureAt(double u) {

	// Curvature calculation from: http://tom.cs.byu.edu/~557/text/cagd.pdf
	TsBSpline *splineCopy = new TsBSpline(*spline);

	splineCopy->split(u);

	splineCopy->toBeziers();

	// Debug print statements
/*
	printf("curvature beziers nCtrlp: %d, nKnot: %d\n", splineCopy->nCtrlp(), splineCopy->nKnots());
	int i,j;
	for(i = 0; i < splineCopy->nCtrlp(); i++)
	{
		printf("ctrlp %d: ( ", i);
		for(j = 0; j < splineCopy->dim(); j++)
		{
			printf("%1.2f ", splineCopy->ctrlp()[i*splineCopy->dim() + j]);
		}
		printf(")\n");
	}

	printf("knots: ( ");

	for(i = 0; i < splineCopy->nKnots(); i++)
	{
		printf("%1.2f ", splineCopy->knots()[i]);
	}
	printf(")\n");
*/

	float* p = splineCopy->ctrlp();
	int dim = splineCopy->dim();
	int deg = splineCopy->deg();
	// Access the points at the end of the first bezier curve, which will be up against the split point
	int offset0 = deg*dim;
	int offset1 = (deg-1)*dim;
	int offset2 = (deg-2)*dim;

	// Special case for the very beginning of the curve
	if(u == 0.0)
	{
		offset0 = 0;
		offset1 = dim;
		offset2 = dim*2;
	}

	tf::Vector3 p0(p[offset0+0], p[offset0+1], p[offset0+2]);
	tf::Vector3 p1(p[offset1+0], p[offset1+1], p[offset1+2]);
	tf::Vector3 p2(p[offset2+0], p[offset2+1], p[offset2+2]);

	double a = (p1 - p0).length();
	double h = fabs(distanceToLine(p2, p0, p1));

	double curvature = ((double)(deg - 1) / (double) deg) * ( h / (a * a));
//	printf("Curvature at %1.9f: %1.9f\n", u, curvature);
	return curvature;

}

double PathPrimitiveBezier::distanceToLine(tf::Vector3 point, tf::Vector3 line0,
		tf::Vector3 line1) {

	double x0 = point.x();
	double y0 = point.y();
	double x1 = line0.x();
	double y1 = line0.y();
	double x2 = line1.x();
	double y2 = line1.y();

	//Source: https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
	double numerator = (y2 - y1) * x0 - (x2 - x1) * y0 + x2 * y1 - y2 * x1;
	double denominator = std::sqrt(std::pow(y2 - y1, 2) + std::pow(x2 - x1, 2));

	return numerator / denominator;
}

tf::Pose PathPrimitiveBezier::getPoseAtDistanceAlongSegment(double distance)
{

	tf::Pose pose;

	return pose;

}

}
