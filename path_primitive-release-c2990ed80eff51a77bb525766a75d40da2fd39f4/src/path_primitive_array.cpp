#include "path_primitive/path_primitive_array.h"

namespace path_primitive{

	PathPrimitiveArray::PathPrimitiveArray()
	{
		//default constructor
	}

	PathPrimitiveArray::PathPrimitiveArray(const path_msgs::Path& _path)
	{	
		_name = _path.name;
		_is_default = _path.is_default;
		_version = _path.message_version;
		std::for_each(_path.segments.begin(), _path.segments.end(), 
					  std::bind(&PathPrimitiveArray::addSegment, this, std::placeholders::_1));

	}

	PathPrimitiveArray::~PathPrimitiveArray()
	{
		fflush(stdout);
		for(auto segment : segments)
		{
			if(segment){
				delete segment;
				segment = nullptr;
			}
		}

		segments.clear();
	}

	bool PathPrimitiveArray::toMsg(const PathPrimitiveArray& _path, path_msgs::Path& _msg)
	{
		_msg.name = _path._name;
		_msg.is_default = _path._is_default;
		_msg.message_version = _path._version;
		for(auto segment : _path.segments)
			_msg.segments.push_back(PathPrimitiveFactory::generate(*segment));
	}

	bool PathPrimitiveArray::fromMsg(const path_msgs::Path& _msg, PathPrimitiveArray& _path)
	{
		_path._name = _msg.name;
		_path._is_default = _msg.is_default;
		_path._version = _msg.message_version;
		for(auto segment : _msg.segments)
			_path.segments.push_back(PathPrimitiveFactory::generate(segment));
	}

	void PathPrimitiveArray::addSegment(const path_msgs::PathSegment& segment)
	{
		segments.push_back(PathPrimitiveFactory::generate(segment));
	}

/*	void PathPrimitiveArray::addPrimitive(const PathPrimitiveBase* primitive)
	{
		//TODO: Decide if a deep copy needs to be done
	}*/
}