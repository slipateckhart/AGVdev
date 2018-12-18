/* See the file "LICENSE" for the full license and copyrights governing this code. */

#include "path_primitive/path_primitive_factory.h"

namespace path_primitive {
inline double limitAngle (double angle)
{
	while (angle > M_PI)
		angle -= (M_PI * 2.0);
	while (angle <= -M_PI)
		angle += (M_PI * 2.0);

	return angle;
}

PathPrimitiveBase* PathPrimitiveFactory::generate(path_msgs::PathSegment _path)
{
	tf::Pose start, end;
	tf::Vector3 startVel, endVel;
	tf::Point centerPoint;
	geometry_msgs::Pose *startPose, *endPose;
	geometry_msgs::Vector3 *startVelocity, *endVelocity;
	geometry_msgs::Point *centerPt;
	//bool *isClockwise;

	switch (_path.type) {
		case _path.TYPE_LINE:
			tf::poseMsgToTF(_path.line.startPose, start);
			tf::poseMsgToTF(_path.line.endPose, end);
			tf::vector3MsgToTF(_path.line.startTwist.linear, startVel);
			tf::vector3MsgToTF(_path.line.endTwist.linear, endVel);
			break;
		case _path.TYPE_CIRCLE:
			tf::poseMsgToTF(_path.circle.startPose,start);
			tf::poseMsgToTF(_path.circle.endPose, end);
			tf::vector3MsgToTF(_path.circle.startTwist.linear, startVel);
			tf::vector3MsgToTF(_path.circle.endTwist.linear, endVel);
			tf::pointMsgToTF(_path.circle.centerPoint, centerPoint);
			//isClockwise = &_path.circle.isClockwise;
			break;
		case _path.TYPE_BEZIER:
			tf::poseMsgToTF(_path.bezier.startPose,start);
			tf::poseMsgToTF(_path.bezier.endPose, end);
			tf::vector3MsgToTF(_path.bezier.startTwist.linear, startVel);
			tf::vector3MsgToTF(_path.bezier.endTwist.linear, endVel);
		break;
	}

	switch(_path.type) {
		case _path.TYPE_LINE:	return new PathPrimitiveLine(start, end, startVel, endVel);	break;
		case _path.TYPE_CIRCLE:	return new PathPrimitiveCircle(start, end, startVel, endVel, centerPoint);	break;
		case _path.TYPE_BEZIER:	/*TODO: Define a createBezier function*/	break;
		default:
			ROS_ERROR("UNKNOWN PATH PRIMITIVE, RETURNING NULL POINTER in PathPrimitiveFactory.  Brace yourself for a crash.");
			return nullptr;
	}
}

path_msgs::PathSegment PathPrimitiveFactory::generate(PathPrimitiveBase &_path)
{
	path_msgs::PathSegment segment;

	switch (_path.getType())
	{
		case PathPrimitiveType::LINE:
			segment.type = segment.TYPE_LINE;
			tf::poseTFToMsg(_path.getStartPose(), segment.line.startPose);
			tf::poseTFToMsg(_path.getEndPose(), segment.line.endPose);
			tf::vector3TFToMsg(_path.getStartVelocity(), segment.line.startTwist.linear);
			tf::vector3TFToMsg(_path.getEndVelocity(), segment.line.endTwist.linear);
			break;
		case PathPrimitiveType::CIRCLE:
			segment.type = segment.TYPE_CIRCLE;
			tf::poseTFToMsg(_path.getStartPose(), segment.circle.startPose);
			tf::poseTFToMsg(_path.getEndPose(), segment.circle.endPose);
			tf::vector3TFToMsg(_path.getStartVelocity(), segment.circle.startTwist.linear);
			tf::vector3TFToMsg(_path.getEndVelocity(), segment.circle.endTwist.linear);
			break;
		default:
			ROS_ERROR("[%s] Unknown segment type [%d]", ros::this_node::getName().c_str(), _path.getType());
			break;
	}

	return segment;
}

}
