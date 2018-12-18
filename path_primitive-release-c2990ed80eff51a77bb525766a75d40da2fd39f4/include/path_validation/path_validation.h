/* See the file "LICENSE" for the full license and copyrights governing this code. */

#ifndef PATH_VALIDATION_H_
#define PATH_VALIDATION_H_

#include <cmath>
#include <memory>
#include <ros/ros.h>
#include <tf/tf.h>
#include <geometry_msgs/PoseStamped.h>
#include <path_primitive/path_primitive_array.h>

namespace path_primitive
{

	struct pathOptions
	{
		public:
			double minRadius;
			double maxSlopeSep;
			double maxSegSep;
			double loopPath;
			std::map<uint, uint> feedback;
	};

	class path_validation
	{
	public:
		enum segmentStatus
			{
				INVALID = 0,
				VALID = 1
			};

		static bool isPathValid(const path_msgs::Path&,const pathOptions&);
		static bool isPathValid(const PathPrimitiveArray&,const pathOptions&);
		static double getL2Norm(const tf::Vector3&, const tf::Vector3&);
		static double dotProduct(tf::Vector3&, tf::Vector3&);
		static bool checkConstraints(const pathOptions&, double, double, int, int);
	};
}

#endif /* PATH_VALIDATION_H_ */
