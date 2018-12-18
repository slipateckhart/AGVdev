/* See the file "LICENSE" for the full license and copyrights governing this code. */

#pragma once

#include "tf/transform_datatypes.h"

namespace rainbow {
class Rainbow
{
public:
	void prune(float d, tf::Vector3 p1, float r1, tf::Vector3 p2, float r2,
			std::vector<tf::Vector3> &inputPoints, std::vector<tf::Vector3> &outputPoints);
};

}
