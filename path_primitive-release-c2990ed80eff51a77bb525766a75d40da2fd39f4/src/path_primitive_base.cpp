/* See the file "LICENSE" for the full license and copyrights governing this code. */

#include "path_primitive/path_primitive_base.h"
#include "path_primitive/path_primitive_factory.h"
#include "path_msgs/PathSegment.h"
#include <vector>

namespace path_primitive {
int getSegmentIndexToFollow(tf::Pose robotPose, std::vector<path_msgs::PathSegment>& _list, int currentGoalIndex=0)
{
//		ROS_INFO("[%s] Finding segment to follow, current Index: [%d], list size: [%d]", ros::this_node::getName().c_str(), currentPathIndex, _list.size());

	//First check all forward paths to be valid and return if found
	for (int i = currentGoalIndex; i < _list.size(); i++)
	{
		PathPrimitiveBase* segment = PathPrimitiveFactory::generate(_list.at(i));

		if (segment->isInRange(robotPose) == true)
		{
			delete segment;
			return i;
		}
		delete segment;

	}

	//Then check all backward paths to be valid and return if found
	for (int i = currentGoalIndex; i >= 0; i--)
	{
		PathPrimitiveBase* segment = PathPrimitiveFactory::generate(_list.at(i));

		if (segment->isInRange(robotPose) == true)
		{
			delete segment;
			return i;
		}
		delete segment;
	}

	//Consider cases where no segment was found valid (leaf path segments at start/stop)
	//Chose the closest one available
	//Set first segment to be the closest
	int closestDistanceIndex = 0;
	PathPrimitiveBase* segmentTemp = PathPrimitiveFactory::generate(_list.at(closestDistanceIndex));
	double distenceErrTemp, headingErrTemp;
	PathPrimitiveResult info = segmentTemp->project(robotPose);
	double closestDist = tf::tfDistance(info.poseOnSegment.getOrigin(), robotPose.getOrigin());

	delete segmentTemp;

	//Check for closest segment on remaining n-1 segments
	for (int i = 1; i < _list.size(); i++)
	{
		PathPrimitiveBase* segment = PathPrimitiveFactory::generate(_list.at(i));
		info = segment->project(robotPose);
		double distance = tf::tfDistance(info.poseOnSegment.getOrigin(), robotPose.getOrigin());

		if (closestDist > distance)
		{
			closestDist = distance;
			closestDistanceIndex = i;
		}
		delete segment;
	}
	return closestDistanceIndex;
}

tf::Pose getLookaheadPose(int _segmentIndexToFollow, tf::Pose robotPose, std::vector<path_msgs::PathSegment>& _list, double desiredLookaheadDist, PathPrimitiveResult &_lookAheadResult, tf::Pose& _closestPointToReturn)
{

	int currentGoalIndex = _segmentIndexToFollow;

	//Find the segment ID where the lookahead point will lie on the list of path segments
	double cumulativeLookaheadDist = 0.0;
	//search along the path until cumulative distance < lookahead
	while ((_segmentIndexToFollow < (int)(_list.size())) && cumulativeLookaheadDist < desiredLookaheadDist)
	{
		PathPrimitiveBase* pathPrimitive = PathPrimitiveFactory::generate(_list.at(_segmentIndexToFollow));
		PathPrimitiveResult info = pathPrimitive->project(robotPose);

		//Add remaining length of segment if we are evaluating current path segment
		//else add the the complete length of segment
		if (_segmentIndexToFollow == currentGoalIndex)// this happens if robot is on _segmentIndexToFollow
			cumulativeLookaheadDist += info.distanceFromEnd;
		else
			cumulativeLookaheadDist += pathPrimitive->length();

		//ROS_INFO("cumulativeLookahead:[%1.3f], desiredLookaheadDist:[%1.3f], _segmentToFollow:[%d], distanceFromEnd[%1.2f], distanceCrossTrackErr[%1.2f]", cumulativeLookaheadDist, desiredLookaheadDist, _segmentIndexToFollow, info.distanceFromEnd, info.distanceError);
		//wrap around and come to begining of path, until enough distance is accumulated
		if (_segmentIndexToFollow++ == _list.size())
			_segmentIndexToFollow = 0;

		delete pathPrimitive;
	}
	
	//this is the point ON the path, that is closes to the robot pose
	PathPrimitiveResult res = (PathPrimitiveFactory::generate(_list.at(currentGoalIndex)))->project(robotPose);
	_closestPointToReturn = res.poseOnSegment;

	//Make sure segment index is a valid number after doing calculations in previous loop
	//this is done because, there was an increment in searching for segment index to follow in above loop
	if (_segmentIndexToFollow > 0)
		 _segmentIndexToFollow--;
	else
		_segmentIndexToFollow = _list.size() - 1;

	//Calculate pose for lookahead
	//TODO - To be replaced by static function from path primitive class
	tf::Pose lookaheadPose;

	double distToBeRemoved = 0.0;
	//Get lookahead pose on segment from start pose
	for (int i = currentGoalIndex; (i != _segmentIndexToFollow); i = (i + 1)%_list.size())
	{
		PathPrimitiveBase* path = PathPrimitiveFactory::generate(_list.at(i));
		PathPrimitiveResult info = path->project(robotPose);

		if (path->length() > info.distanceFromEnd) // on path segment
			distToBeRemoved += info.distanceFromEnd;
		else
			distToBeRemoved += path->length(); // if not on path segment

		delete path;
	}
	//remaining dist
	double remainingDist = desiredLookaheadDist - distToBeRemoved;

	tf::Pose closestPoint;
	PathPrimitiveBase* segment = PathPrimitiveFactory::generate(_list.at(_segmentIndexToFollow));
	PathPrimitiveResult info = segment->project(robotPose);
	_lookAheadResult = info;

	if (_segmentIndexToFollow == currentGoalIndex)
		closestPoint = info.poseOnSegment;
	else
		closestPoint = segment->getStartPose();

	if (segment->getType() == PathPrimitiveType::LINE)
	{
		double theta = atan2(segment->getEndPose().getOrigin().getY() - closestPoint.getOrigin().getY(), segment->getEndPose().getOrigin().getX() - closestPoint.getOrigin().getX());
		lookaheadPose.setOrigin(tf::Vector3(closestPoint.getOrigin().getX() + remainingDist * cos(theta), closestPoint.getOrigin().getY() + remainingDist * sin(theta),  0.0));
		lookaheadPose.setRotation(tf::createQuaternionFromYaw(tf::getYaw(segment->getEndPose().getRotation())));

		//ROS_INFO("Calculating lookahead pose LINE - remianingDist[%1.3f], cumulativeLookahead[%1.2f], closestPoint[%1.2f][%1.2f]|[%1.2f], lookaheadpose[%1.2f][%1.2f]|[%1.2f]", remainingDist, cumulativeLookaheadDist, closestPoint.getOrigin().getX(), closestPoint.getOrigin().getY(), tf::getYaw(closestPoint.getRotation()), lookaheadPose.getOrigin().getX(), lookaheadPose.getOrigin().getY(), tf::getYaw(lookaheadPose.getRotation()));
	}

	else if (segment->getType() == PathPrimitiveType::CIRCLE)
	{
		double r = 1.0 / info.curvature;

		//this is in static frame where frame is at center of CIRCLE
		//where x points front 
		double theta = remainingDist / r;
		double dx = r * sin(theta);
		double dy = r * (1.0 - cos(theta));
		double robotYaw = tf::getYaw(closestPoint.getRotation());

		

		//convert dx, dy from static frame to map frame. 
		double dxInMap = dx * cos(robotYaw) - dy * sin(robotYaw);
		double dyInMap = dx *sin(robotYaw) + dy * cos(robotYaw);

		//ROS_WARN(" wb [%1.3f], r [%1.3f] theta [%1.3f] dx [%1.3f] dy [%1.3f] m_dx [%1.3f] m_dy [%1.3f]", 1.0, r, theta, dx, dy, dxInMap, dyInMap);

		tf::Vector3 desiredPoint(closestPoint.getOrigin().getX() + dxInMap, closestPoint.getOrigin().getY() + dyInMap, 0.0);
		double desiredHeading = robotYaw + theta;

		lookaheadPose.setOrigin(desiredPoint);
		lookaheadPose.setRotation(tf::createQuaternionFromYaw(desiredHeading));
		//ROS_INFO("Calculating lookahead pose CIRCLE - remianingDist[%1.3f], cumulativeLookahead[%1.2f], closestPoint[%1.2f][%1.2f]|[%1.2f], lookaheadpose[%1.2f][%1.2f]|[%1.2f]", remainingDist, cumulativeLookaheadDist, closestPoint.getOrigin().getX(), closestPoint.getOrigin().getY(), tf::getYaw(closestPoint.getRotation()), lookaheadPose.getOrigin().getX(), lookaheadPose.getOrigin().getY(), tf::getYaw(lookaheadPose.getRotation()));
	}
	delete segment;

	return lookaheadPose;
}
}
