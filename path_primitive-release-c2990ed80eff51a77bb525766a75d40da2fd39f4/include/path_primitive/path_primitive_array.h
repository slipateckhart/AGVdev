/* See the file "LICENSE" for the full license and copyrights governing this code. */

#pragma once

#include "path_primitive/path_primitive_factory.h"
#include <path_msgs/Path.h>

namespace path_primitive
{
	/*!
	   A structure to represent the path (array of path segments)
	 */
	class PathPrimitiveArray
	{
		public:
			//Default constructor
			PathPrimitiveArray();
			//Constructor which takes a path message
			PathPrimitiveArray(const path_msgs::Path&);
			//Default destructor
			~PathPrimitiveArray();
			//add a segment to path. 
			void addSegment(const path_msgs::PathSegment&);
			//add a primitive to path
			//void addPrimitive(const PathPrimitiveBase*);
			//conversion from path primitive array to path message
			static bool toMsg(const PathPrimitiveArray&, path_msgs::Path&);
			//converstion from path message to path primitive array
			static bool fromMsg(const path_msgs::Path&, PathPrimitiveArray&);
			
			//a vector to hold each path segement as a base class
			std::vector<PathPrimitiveBase*> segments;
			int8_t _version; //path version
			std::string _name; //path name
			bool _is_default; //flag for default status of path
			
		private:
	};
}