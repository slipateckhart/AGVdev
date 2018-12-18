/* See the file "LICENSE" for the full license and copyrights governing this code. */

#pragma once

#include "tf/transform_datatypes.h"
#include "path_primitive/path_primitives.h"
#include "path_msgs/PathSegment.h"
#include "ros/ros.h"
namespace path_primitive {
class PathPrimitiveFactory
{
public:
	static PathPrimitiveBase* generate(path_msgs::PathSegment _path);	/**< Returns a pointer to the object of PathprimitiveBase class with start pose and velocity and end pose and velocity of the path segment.
	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	@param _path path segment of path_msgs*/
	static path_msgs::PathSegment generate(PathPrimitiveBase &);		/**< Returns a path segment of path_msgs type.
	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 @param An object of PathPrimitiveBase class */

};
}
