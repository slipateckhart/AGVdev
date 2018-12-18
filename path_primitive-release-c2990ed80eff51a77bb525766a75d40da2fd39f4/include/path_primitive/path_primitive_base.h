/* See the file "LICENSE" for the full license and copyrights governing this code. */

#pragma once

#include "tf/transform_datatypes.h"
#include "path_msgs/PathSegment.h"
#include <limits>

namespace path_primitive
{
/*!
   A structure to represent the parameters of path segments
 */
struct PathPrimitiveResult
{
	tf::Pose poseOnSegment;	/**< Position and orientation of the robot closer or on the segment. */
	double parameter;		/**< Splits the Bezier curve at this value (0-1) and it is applied recursively to form new control points.*/
	double curvature;		/**< Amount of deviation of a segment from being straight (0 for line type)*/
	double distanceFromEnd;	/**< Distance between the projected pose on the segment and the end pose of the segment*/
	double headingError;	/**< Angular difference between the yaw direction of the given robot orientation and actual orientation of the segment.
							Note : For the headingError the negative/positive sign indicate the given pose is opposite/same direction with respect to the heading of the Start pose of the segment */
	double distanceError;	/**< Difference in distance between the segment and the given robot pose.
							Note : For the distanceError, the negative/positive sign indicate the given pose is on the left/right side with respect to the heading of Start pose of the segment. */
	double velocity; 		/**< Velocity at the start pose*/
};

/*!
   An enum to represent the path primitive type
 */
enum PathPrimitiveType {NONE, TURN, LINE, CIRCLE, ARC, BEZIER};

class PathPrimitiveBase
{
public:
	virtual bool isInRange(tf::Pose pose) = 0;							 /**< Returns true if the given robot pose is within the range of the path segment.
	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 @param pose Pose of the robot. */
	virtual PathPrimitiveResult project(tf::Pose pose) = 0; 			 /**< Returns an object of PathPrimitiveResult with the projected pose on the segment,
																			  distance error, heading error, distance from end pose, curvature and velocity.
																			  @param pose Pose of the robot. */
	virtual tf::Vector3 tangentVector(tf::Point point) = 0;				 /**< Returns a tangent vector at the given point on the segment.
																			  @param point point on the segment. */
	virtual tf::Quaternion tangentQuat(tf::Point point) = 0;			 /**< Returns a quaternion of the tangent vector at the given point on the segment.
																			  @param point point on the segment.*/
	virtual double curvatureAt(tf::Pose pose) = 0;						 /**< Updates the curvature of the segment. It is zero for line and (1/radius) for circle.
																			  @param pose Pose of the robot. */
	virtual double distanceError(tf::Pose pose) = 0;					 /**< Returns the distance between the given pose and the segment.
																			  Note : Calculations vary based on the type of segment. @param pose Pose of the robot. */
	virtual tf::Pose getPoseAtDistanceAlongSegment(double distance) = 0; /**< Returns the pose on the segment at a given distance.
																			  @param distance Distance at which the pose on segment is calculate.*/
	virtual double length()= 0;											 /**< Returns the length of the path segment. */
	virtual PathPrimitiveType getType() = 0; 							 /**< Returns the type of segment. */
	virtual double getRadius() = 0;
	void setStartVelocity(tf::Vector3 vel) { startVelocity = vel; }      /**< Sets start velocity. @param vel velocity at start of the segment.*/
	void setEndVelocity(tf::Vector3 vel) { endVelocity = vel;}			 /**< Sets end velocity. @param vel velocity at end of the segment */
	void setStartPose(tf::Pose pose) { startPose = pose; }				 /**< Sets start pose. @param pose Pose at start of the segment*/
	void setEndPose(tf::Pose pose) { endPose = pose;}					 /**< Sets send pose. @param pose Pose at end of the segment*/
	tf::Vector3 getStartVelocity() { return startVelocity; }			 /**< Returns the velocity at start pose of the segment*/
	tf::Vector3 getEndVelocity() { return endVelocity;}					 /**< Returns the velocity at end pose of the segment*/
	tf::Pose getStartPose() { return startPose; }						 /**< Returns the start pose of the segment */
	tf::Pose getEndPose() { return endPose;}							 /**< Returns the end pose of the segment*/
	virtual ~PathPrimitiveBase() {}

	int seqId;

private:
	tf::Pose startPose;
	tf::Pose endPose;
	tf::Vector3 startVelocity;
	tf::Vector3 endVelocity;
};


tf::Pose getLookaheadPose(int _segmentIndexToFollow, tf::Pose robotPose, std::vector<path_msgs::PathSegment>& _list, double desiredLookaheadDist, PathPrimitiveResult&, tf::Pose&); /**< Returns a look ahead pose on a segment to follow.
																																														@param _segmentIndexToFollow The next segment index
																																														@param _list List of path segments
																																														@param robotPose Current pose of the robot
																																														@param desiredLookaheadDist A distance to look head
																																														@param closestPont The closest point to return*/
int getSegmentIndexToFollow(tf::Pose robotPose, std::vector<path_msgs::PathSegment>& _list, int currentGoalIndex);																	/**< Returns the next index in the list of path segments to follow based on its proximity to the current segment
 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	@param _list List of path segments
																																														@param robotPose Current pose of the robot
																																														@param currentGoalIndex Index of the current path	 	 	 */

}

