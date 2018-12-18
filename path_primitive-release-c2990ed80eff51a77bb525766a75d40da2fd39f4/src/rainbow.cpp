/* See the file "LICENSE" for the full license and copyrights governing this code. */
#include "path_primitive/rainbow.h"

#include <cmath>
#include "tf/transform_datatypes.h"

namespace rainbow {

void rainbow::Rainbow::prune(float d, tf::Vector3 p1, float r1, tf::Vector3 p2,
		float r2, std::vector<tf::Vector3> &inputPoints, std::vector<tf::Vector3> &outputPoints) {

	tf::Vector3 v12 = (p2 - p1).normalized();

	tf::Vector3 c1 = p1 + v12*r1;
	tf::Vector3 c2 = p2 + v12*r2;

	float theta1 = d / r1;
	tf::Vector3 ep1 = tf::quatRotate(tf::createQuaternionFromYaw(theta1), c1 - p1);

	float theta2 = d / r2;
	tf::Vector3 ep2 = tf::quatRotate(tf::createQuaternionFromYaw(theta2), c2 - p2);

	tf::Vector3 ep2_c2 = (c2 - ep2).normalized();
	tf::Vector3 ep2_ep1 = ep1 - ep2;
	float ep_len = ep2_ep1.length();
	ep2_ep1.normalize();
	float inv_cos_phi = 1.0f / ep2_ep1.dot(ep2_c2);
	float r3 = 0.5 * ep_len * inv_cos_phi;

	tf::Vector3 c3 = ep2 + ep2_c2 * r3;

}

}
