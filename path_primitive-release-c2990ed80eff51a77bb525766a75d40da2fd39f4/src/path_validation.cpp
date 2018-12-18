#include "path_validation/path_validation.h"

namespace path_primitive
{
	bool path_validation::isPathValid(const path_msgs::Path& _path,const pathOptions& options)
	{
		PathPrimitiveArray path(_path);
		return isPathValid(path, options);
	}

	bool path_validation::isPathValid(const PathPrimitiveArray& _path,const pathOptions& options)
	{
		if(_path.segments.size() <= 0)
		{
			ROS_ERROR("[%s]:Invalid path. Number of segments is:%d", ros::this_node::getName().c_str(), (int)_path.segments.size());
			return false;
		}
		else
			ROS_DEBUG("[%s]:Number of segments in path:[%s] is:[%d].", ros::this_node::getName().c_str(), _path._name.c_str(), (int)_path.segments.size());

		auto pathIt = std::begin(_path.segments);
		uint count  = 0;

		for (; count < _path.segments.size() || pathIt != std::end(_path.segments); pathIt++, count++)
		{
			ROS_DEBUG("[%s]:Checking segment:%d", ros::this_node::getName().c_str(), count);
			PathPrimitiveBase* currSegment;
			PathPrimitiveBase* nextSegment;
			//if current segment is at the end of the segment
			//windup to start of path
			currSegment = _path.segments.at(count);
			nextSegment = _path.segments.at(((count + 1) % (int)_path.segments.size())); 
			double seperation = getL2Norm(currSegment->getEndPose().getOrigin(), nextSegment->getStartPose().getOrigin());
			
			switch(currSegment->getType())
			{
				case PathPrimitiveType::LINE :
				{
					tf::Vector3 startVect = currSegment->tangentVector(currSegment->getStartPose().getOrigin());

					//a line to line junction
					if(nextSegment->getType() == PathPrimitiveType::LINE)
					{
						tf::Vector3 endVect = nextSegment->tangentVector(nextSegment->getStartPose().getOrigin());;
						double slopeDiff = dotProduct(startVect, endVect);
						if(!checkConstraints(options, seperation, slopeDiff, count, (int)_path.segments.size()))
							return false;
					}
					//line to circle junction
					else if(nextSegment->getType() == PathPrimitiveType::CIRCLE)
					{
						tf::Vector3 endVect = nextSegment->tangentVector(nextSegment->getStartPose().getOrigin());
						double slopeDiff = dotProduct(startVect, endVect);
						if(!checkConstraints(options, seperation, slopeDiff, count, (int)_path.segments.size()))
							return false;
					}
					break;
				}
				case PathPrimitiveType::CIRCLE:
				{
					tf::Vector3 startVect = currSegment->tangentVector(currSegment->getEndPose().getOrigin());

					// //true if radius is above threshold else false
					if(fabs(currSegment->getRadius()) <= options.minRadius)
					{
						//ERROR
						ROS_WARN("[%s]:Radius is below threshold{[%f] vs [%f]} for segment:%d", ros::this_node::getName().c_str(), 
																								currSegment->getRadius(),
																								options.minRadius,
																								count);
						return false;
					}		
					//junction between circle and line
					if(nextSegment->getType() == PathPrimitiveType::LINE)
					{
						tf::Vector3 endVect = nextSegment->tangentVector(nextSegment->getStartPose().getOrigin());
						double slopeDiff = dotProduct(startVect, endVect);
						if(!checkConstraints(options, seperation, slopeDiff, count, (int)_path.segments.size()))
							return false;
					}
					//junction between circle and circle
					else if(nextSegment->getType() == PathPrimitiveType::CIRCLE)
					{
						tf::Vector3 endVect = nextSegment->tangentVector(nextSegment->getStartPose().getOrigin());
						double slopeDiff = dotProduct(startVect, endVect);
						if(!checkConstraints(options, seperation, slopeDiff, count, (int)_path.segments.size()))
							return false;
					}
					break;
				}
				//unknown type of path segment	
				default:
				{
					ROS_ERROR("[%s]: Unknown type of segment encountered in path:[%s].", ros::this_node::getName().c_str(), _path._name.c_str());
					return false;
					break;
				}
			}

		}
		return true;	
	}

	//TODO: Replace last two params by a bool (count == size) will do the job
	bool path_validation::checkConstraints(const pathOptions& options, double sep, double slopeDiff, int count, int size)
	{
		if(options.loopPath && (sep > options.maxSegSep))
		{
			ROS_WARN("[%s]: Segment:[%d]<->[%d] seperation is:%f. Threshold:%f", ros::this_node::getName().c_str(), 
																  count,(count + 1) % size, sep, options.maxSegSep);
			return false;
		}
		//if looping is disabled and you are at the last segment, don't do a slope check
		//this can happen when paths are discontinuous
		else if(!options.loopPath && (count + 1 == size))
		{
			ROS_DEBUG("[%s]: No slope check for last segment in open loop path", ros::this_node::getName().c_str());
			//DEBUG
		}
		else if(slopeDiff < std::sin(options.maxSlopeSep)) 
		{
			ROS_WARN("[%s]: Segment:[%d]<->[%d] slope difference is:%f. Threshold:%f", ros::this_node::getName().c_str(), 
													count,(count + 1) % size, slopeDiff, std::sin(options.maxSlopeSep));
			return false;
		}
		else
		{
			ROS_DEBUG("[%s]:Something wierd happened. Debug this", ros::this_node::getName().c_str());
			//this happens when all checks pass
		}

		return true;
	}

	double path_validation::getL2Norm(const tf::Vector3& p1, const tf::Vector3& p2)
	{
		return sqrt(pow(p1.getX() - p2.getX(),2) + pow(p1.getY() - p2.getY(),2)); //currently only 2D is supported
	}

	double path_validation::dotProduct(tf::Vector3& v1, tf::Vector3& v2)
	{
		v1.normalize();
		v2.normalize();
		return (v1.getX() * v2.getX() + v1.getY() * v2.getY() + v1.getZ() * v2.getZ()); 
	}
}