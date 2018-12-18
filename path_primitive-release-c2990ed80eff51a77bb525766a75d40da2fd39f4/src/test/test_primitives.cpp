#include <gtest/gtest.h>

#include "../include/path_primitive/path_primitives.h"
#include "tf/transform_datatypes.h"



/******************************** Line ***********************************/

TEST(PathPrimitiveTest,Line) {
	/*
	 *  Initialize the start pose and end pose of the segment
	 */
	tf::Pose p1, p2;
	// start pose
	p1.setOrigin(tf::Vector3(0,0,0));
	//end pose
	p2.setOrigin(tf::Vector3(1,0,0));

	path_primitive::PathPrimitiveLine *prim = new path_primitive::PathPrimitiveLine(p1, p2, tf::Vector3(0,0,0), tf::Vector3(1,0,0));

	/*
	 *  Test case 1 : Valid segment (A line of unit length). Robot pose is IN RANGE area and above the segment.
	 *  Functions tested :
	 *  	* bool isInRange(tf::Pose pose) Should return true or the test terminates.
	 *  	* PathPrimitiveResult project(tf::Pose pose)
	 *  	* tf::Vector3 tangentVector(tf::Point point)
	 *  	* tf::Quaternion tangentQuat(tf::Point point
	 *  	* tf::Pose getPoseAtDistanceAlongSegment(double distance) Tested at a distance of 0.6.
	 *  	* double distanceError(tf::Pose pose)
	 *  	* double length()
	 *  	* double curvatureAt(tf::Pose pose)
	 */
	tf::Pose robotPose;

	//robot pose
	robotPose.setOrigin(tf::Vector3(0.5,1,0));
	robotPose.setRotation(tf::createQuaternionFromYaw(-M_PI/2.0));

	// isInRange function
	ASSERT_TRUE(prim->isInRange(robotPose));

	// project function
	path_primitive::PathPrimitiveResult res = prim->project(robotPose);
	EXPECT_FLOAT_EQ(0.0f, res.curvature);
	EXPECT_FLOAT_EQ(0.5f, res.poseOnSegment.getOrigin().getX());
	EXPECT_FLOAT_EQ(0.0f, res.poseOnSegment.getOrigin().getY());
	EXPECT_FLOAT_EQ(-1.0f, res.distanceError);
	EXPECT_FLOAT_EQ(M_PI/2.0,res.headingError);
	EXPECT_FLOAT_EQ(0.5f, res.distanceFromEnd);
	EXPECT_FLOAT_EQ(0.0f, res.velocity);

	// length function
	EXPECT_FLOAT_EQ(1.0f, prim->length());

	// getPoseAtDistanceAlongSegment function
	double distance = 0.6;
	tf::Pose p = prim->getPoseAtDistanceAlongSegment(distance) ;
	EXPECT_FLOAT_EQ(0.6f, p.getOrigin().getX());
	EXPECT_FLOAT_EQ(0.0f, p.getOrigin().getY());

	// tangentQuat function
	tf::Point pt(0,0,0);
	tf::Quaternion q;
	q = prim->tangentQuat(pt);
	EXPECT_FLOAT_EQ(0.0, q.getX());
	EXPECT_FLOAT_EQ(0.0, q.getY());
	EXPECT_FLOAT_EQ(0.0, q.getZ());

	/*
	 *  Test case 2 : Valid segment (A line of unit length). Robot pose is IN RANGE area and to left of the start pose of the segment.
	 *  Functions tested :
	 *  	* bool isInRange(tf::Pose pose) Should return true or the test terminates.
	 *  	* PathPrimitiveResult project(tf::Pose pose)
	 *  	* tf::Pose getPoseAtDistanceAlongSegment(double distance) Tested at a distance of 0. Should return start pose.
	 *  	* double distanceError(tf::Pose pose)
	 */
	//robot pose
	robotPose.setOrigin(tf::Vector3(-1,-1,0));

	// isInRange function
	ASSERT_TRUE(prim->isInRange(robotPose));

	//  project function
	res = prim->project(robotPose);
	EXPECT_FLOAT_EQ(0.0f, res.curvature);
	EXPECT_FLOAT_EQ(0.0f, res.poseOnSegment.getOrigin().getX());
	EXPECT_FLOAT_EQ(0.0f, res.poseOnSegment.getOrigin().getY());
	EXPECT_FLOAT_EQ(1.0f, res.distanceError);
	EXPECT_FLOAT_EQ(M_PI/2.0,res.headingError);
	EXPECT_FLOAT_EQ(1.0f, res.distanceFromEnd);
	EXPECT_FLOAT_EQ(0.0f, res.velocity);

	// getPoseAtDistanceAlongSegment()
	distance = 0;
	p = prim->getPoseAtDistanceAlongSegment(distance) ;
	EXPECT_FLOAT_EQ(0.0f, p.getOrigin().getX());
	EXPECT_FLOAT_EQ(0.0f, p.getOrigin().getY());

	/*
	 * Test case 3 : Valid segment (A line of unit length). Robot pose is OUT OF RANGE and to right of the end pose of the segment.
	 * Functions tested :
	 *  	* bool isInRange(tf::Pose pose) Should return false or the test terminates.
	 *  	* tf::Pose getPoseAtDistanceAlongSegment(double distance) Tested at a distance equal to the length of the segment. Should return end pose.
	 */
	robotPose.setOrigin(tf::Vector3(2,-4,0)); //robot pose

	// isInRange function
	ASSERT_FALSE(prim->isInRange(robotPose));

	// getPoseAtDistanceAlongSegment function
	distance = prim->length();
	p = prim->getPoseAtDistanceAlongSegment(distance) ;
	EXPECT_FLOAT_EQ(1.0f, p.getOrigin().getX());
	EXPECT_FLOAT_EQ(0.0f, p.getOrigin().getY());

	/*
	 * Test case 4 : valid segment.
	 * Function tested :
	 *  	* tf::Pose getPoseAtDistanceAlongSegment(double distance) Tested at a distance greater than length of the segment. Should return end pose.
	 */
	// getPoseAtDistanceAlongSegment function
	distance = 10;
	p = prim->getPoseAtDistanceAlongSegment(distance) ;
	EXPECT_FLOAT_EQ(1.0f, p.getOrigin().getX());
	EXPECT_FLOAT_EQ(0.0f, p.getOrigin().getY());

}

/******************************** Circle / Arcs  ***********************************/

/* Circle1 tests with the constructor which has centerPoint as one of its input argument */

TEST(PathPrimitiveTest,Circle1) {

	/*
	 * Initialize the start pose and end pose of the segment
	 */
	tf::Pose p1, p2;
	tf::Point center(0,0,0); //center point
	p1.setOrigin(tf::Vector3(1,0,0)); // start pose
	p1.setRotation(tf::createQuaternionFromYaw(M_PI/2.0));
	p2.setOrigin(tf::Vector3(0,1,0)); //end pose
	p2.setRotation(tf::createQuaternionFromYaw(-M_PI));
	path_primitive::PathPrimitiveCircle *prim = new path_primitive::PathPrimitiveCircle(p1, p2, tf::Vector3(0,0,0), tf::Vector3(1,0,0),center);

	/*
	 *  Test case 1 : Valid segment(An arc of unit radius and it's a quarter). Robot pose is IN RANGE area and above the segment.
	 *  Functions tested :
	 *  	* bool isInRange(tf::Pose pose) Should return true or the test terminates.
	 *  	* PathPrimitiveResult project(tf::Pose pose)
	 *  	* tf::Vector3 tangentVector(tf::Point point)
	 *  	* tf::Quaternion tangentQuat(tf::Point point
	 *  	* tf::Pose getPoseAtDistanceAlongSegment(double distance). Tested at a distance of 0.6.
	 *  	* double distanceError(tf::Pose pose)
	 *  	* double length()
	 *  	* double curvatureAt(tf::Pose pose)
	 */
	tf::Pose robotPose;
	robotPose.setOrigin(tf::Vector3(0.5,1,0)); //robot pose
	robotPose.setRotation(tf::createQuaternionFromYaw(-M_PI/2));

	// isInRange function
	ASSERT_TRUE(prim->isInRange(robotPose));

	// project function
	path_primitive::PathPrimitiveResult res = prim->project(robotPose);
	EXPECT_FLOAT_EQ(1.0f, res.curvature);
	EXPECT_NEAR(0.447f, res.poseOnSegment.getOrigin().getX(),1e-3);
	EXPECT_NEAR(0.894f, res.poseOnSegment.getOrigin().getY(),1e-3);
	EXPECT_NEAR(0.118f, res.distanceError,1e-3);
	EXPECT_NEAR(-2.034f,res.headingError,1e-3);
	EXPECT_NEAR(0.464f, res.distanceFromEnd,1e-3);
	EXPECT_FLOAT_EQ(0.0f, res.velocity);

	// length function
	EXPECT_NEAR(M_PI/2.0, prim->length(),1e-3);

	// get pose along a distance on the segment
	double distance = 0.6;
	tf::Pose p= prim->getPoseAtDistanceAlongSegment(distance) ;
	EXPECT_NEAR(0.825f, p.getOrigin().getX(),1e-3);
	EXPECT_NEAR(0.564f, p.getOrigin().getY(),1e-3);
	EXPECT_NEAR(0.884f,p.getRotation().getZ(),1e-3);

	// tangentQuat function
	tf::Point pt(res.poseOnSegment.getOrigin().getX(),res.poseOnSegment.getOrigin().getY(),0);
	tf::Quaternion q;
	q = prim->tangentQuat(pt);
	EXPECT_FLOAT_EQ(0.0, q.getX());
	EXPECT_FLOAT_EQ(0.0, q.getY());
	EXPECT_NEAR(0.9733, q.getZ(),1e-3);
	EXPECT_NEAR(0.229f, q.getW(),1e-3);

	// tangentVector function
	tf::Vector3 v =prim->tangentVector(pt);
	EXPECT_NEAR(-0.894f, v.getX(),1e-3);
	EXPECT_NEAR(0.447f, v.getY(),1e-3);
	EXPECT_NEAR(0, v.getZ(),1e-3);

	/*
	 * Test case 2 : Valid segment(An arc of unit radius and it's a quarter). Robot pose is OUT OF RANGE area and to the left of the start pose of the segment.
	 * Functions tested :
	 *  	* bool isInRange(tf::Pose pose) Should return false or the test terminates.
	 *  	* PathPrimitiveResult project(tf::Pose pose)
	 *  	* tf::Vector3 tangentVector(tf::Point point)
	 *  	* tf::Quaternion tangentQuat(tf::Point point
	 *  	* tf::Pose getPoseAtDistanceAlongSegment(double distance) Tested at a distance of 0.
	 *  	* double distanceError(tf::Pose pose)
	 */
	robotPose.setOrigin(tf::Vector3(-1,0,0)); //robot pose
	robotPose.setRotation(tf::createQuaternionFromYaw(-M_PI/2.0));

	// isInRange function
	ASSERT_FALSE(prim->isInRange(robotPose));

	// project function
	res = prim->project(robotPose);
	EXPECT_FLOAT_EQ(1.0f, res.curvature); // checks curvatureAt()
    EXPECT_FLOAT_EQ(-1.0f, res.poseOnSegment.getOrigin().getX());
	EXPECT_FLOAT_EQ(1.0f, res.poseOnSegment.getOrigin().getY());
	EXPECT_NEAR(0.0f, res.distanceError,1e-3); // checks distanceError()
	EXPECT_NEAR(-1.570f, res.headingError,1e-3);
	EXPECT_NEAR(0.785f, res.distanceFromEnd,1e-3);
	EXPECT_FLOAT_EQ(0.0f, res.velocity);

	// getPoseAtDistanceAlongSegment function
	distance = 0;
	p = prim->getPoseAtDistanceAlongSegment(distance) ;
	EXPECT_FLOAT_EQ(1.0f, p.getOrigin().getX());
	EXPECT_FLOAT_EQ(0.0f, p.getOrigin().getY());

	// tangenQuat function
	pt.setX(res.poseOnSegment.getOrigin().getX());
	pt.setY(res.poseOnSegment.getOrigin().getY());
	pt.setZ(0);
	q = prim->tangentQuat(pt);
	EXPECT_FLOAT_EQ(0.0, q.getX());
	EXPECT_FLOAT_EQ(0.0, q.getY());
	EXPECT_NEAR(0.924f, q.getZ(),1e-3);
	EXPECT_NEAR(-0.382f, q.getW(),1e-3);


	// tangentVector function
	v =prim->tangentVector(pt);
	EXPECT_NEAR(-0.707f, v.getX(),1e-3);
	EXPECT_NEAR(-0.707f, v.getY(),1e-3);
	EXPECT_NEAR(0, v.getZ(),1e-3);

	/*
	 * Test case 3 : Valid segment(An arc of unit radius and it's a quarter). Robot pose is IN RANGE area and to the bottom of the start pose of the segment.
	 * Functions tested :
	 *  	* bool isInRange(tf::Pose pose) Should return true or the test terminates.
	 *  	* PathPrimitiveResult project(tf::Pose pose)
	 *  	* tf::Pose getPoseAtDistanceAlongSegment(double distance) Tested at a distance of 1.
	 */

	robotPose.setOrigin(tf::Vector3(2,-1,0));

	// isInRange function
	ASSERT_TRUE(prim->isInRange(robotPose));

	// project function
	res = prim->project(robotPose);
	EXPECT_NEAR(0.894f, res.poseOnSegment.getOrigin().getX(),1e-3);
	EXPECT_NEAR(-0.447f, res.poseOnSegment.getOrigin().getY(),1e-3);

	// getPoseAtDistanceAlongSegment function
	distance = 4;
	p= prim->getPoseAtDistanceAlongSegment(distance) ;
	EXPECT_FLOAT_EQ(0.0f, p.getOrigin().getX());
	EXPECT_FLOAT_EQ(1.0f, p.getOrigin().getY());

}

/* Circle2 tests with the constructor which doesn't have centerPoint as input argument */

TEST(PathPrimitiveTest,Circle2) {

	/*
	 * Initialize the start pose and end pose of the segment
	 */
	tf::Pose p1, p2;
	p1.setOrigin(tf::Vector3(1,0,0)); // start pose
	p1.setRotation(tf::createQuaternionFromYaw(M_PI/2.0));
	p2.setOrigin(tf::Vector3(0,1,0)); //end pose
	p2.setRotation(tf::createQuaternionFromYaw(-M_PI));
	path_primitive::PathPrimitiveCircle *prim = new path_primitive::PathPrimitiveCircle(p1, p2, tf::Vector3(0,0,0), tf::Vector3(1,0,0));

	/*
	 *  Test case 1 : Valid segment(An arc of unit radius and it's a quarter). Robot pose is IN RANGE area and above the segment.
	 *  Functions tested :
	 *  	* bool isInRange(tf::Pose pose) Should return true or the test terminates.
	 *  	* PathPrimitiveResult project(tf::Pose pose)
	 *  	* tf::Vector3 tangentVector(tf::Point point)
	 *  	* tf::Quaternion tangentQuat(tf::Point point
	 *  	* tf::Pose getPoseAtDistanceAlongSegment(double distance). Tested at a distance of 0.6.
	 *  	* double distanceError(tf::Pose pose)
	 *  	* double length()
	 *  	* double curvatureAt(tf::Pose pose)
	 */
	tf::Pose robotPose;
	robotPose.setOrigin(tf::Vector3(0.5,1,0)); //robot pose
	robotPose.setRotation(tf::createQuaternionFromYaw(-M_PI/2));

	// isInRange function
	ASSERT_TRUE(prim->isInRange(robotPose));

	// project function
	path_primitive::PathPrimitiveResult res = prim->project(robotPose);
	EXPECT_FLOAT_EQ(1.0f, res.curvature);
	EXPECT_NEAR(0.447f, res.poseOnSegment.getOrigin().getX(),1e-3);
	EXPECT_NEAR(0.894f, res.poseOnSegment.getOrigin().getY(),1e-3);
	EXPECT_NEAR(0.118f, res.distanceError,1e-3);
	EXPECT_NEAR(-2.034f,res.headingError,1e-3);
	EXPECT_NEAR(0.464f, res.distanceFromEnd,1e-3);
	EXPECT_FLOAT_EQ(0.0f, res.velocity);

	// length function
	EXPECT_NEAR(M_PI/2.0, prim->length(),1e-3);

	// get pose along a distance on the segment
	double distance = 0.6;
	tf::Pose p= prim->getPoseAtDistanceAlongSegment(distance) ;
	EXPECT_NEAR(0.825f, p.getOrigin().getX(),1e-3);
	EXPECT_NEAR(0.564f, p.getOrigin().getY(),1e-3);
	EXPECT_NEAR(0.884f,p.getRotation().getZ(),1e-3);

	// tangentQuat function
	tf::Point pt(res.poseOnSegment.getOrigin().getX(),res.poseOnSegment.getOrigin().getY(),0);
	tf::Quaternion q;
	q = prim->tangentQuat(pt);
	EXPECT_FLOAT_EQ(0.0, q.getX());
	EXPECT_FLOAT_EQ(0.0, q.getY());
	EXPECT_NEAR(0.9733, q.getZ(),1e-3);
	EXPECT_NEAR(0.229f, q.getW(),1e-3);

	// tangentVector function
	tf::Vector3 v =prim->tangentVector(pt);
	EXPECT_NEAR(-0.894f, v.getX(),1e-3);
	EXPECT_NEAR(0.447f, v.getY(),1e-3);
	EXPECT_NEAR(0, v.getZ(),1e-3);

	/*
	 * Test case 2 : Valid segment(An arc of unit radius and it's a quarter). Robot pose is OUT OF RANGE area and to the left of the start pose of the segment.
	 * Functions tested :
	 *  	* bool isInRange(tf::Pose pose) Should return false or the test terminates.
	 *  	* PathPrimitiveResult project(tf::Pose pose)
	 *  	* tf::Vector3 tangentVector(tf::Point point)
	 *  	* tf::Quaternion tangentQuat(tf::Point point
	 *  	* tf::Pose getPoseAtDistanceAlongSegment(double distance) Tested at a distance of 0.
	 *  	* double distanceError(tf::Pose pose)
	 */
	robotPose.setOrigin(tf::Vector3(-1,0,0)); //robot pose
	robotPose.setRotation(tf::createQuaternionFromYaw(-M_PI/2.0));

	// isInRange function
	ASSERT_FALSE(prim->isInRange(robotPose));

	// project function
	res = prim->project(robotPose);
	EXPECT_FLOAT_EQ(1.0f, res.curvature); // checks curvatureAt()
    EXPECT_FLOAT_EQ(-1.0f, res.poseOnSegment.getOrigin().getX());
	EXPECT_FLOAT_EQ(1.0f, res.poseOnSegment.getOrigin().getY());
	EXPECT_NEAR(0.0f, res.distanceError,1e-3); // checks distanceError()
	EXPECT_NEAR(-1.570f, res.headingError,1e-3);
	EXPECT_NEAR(0.785f, res.distanceFromEnd,1e-3);
	EXPECT_FLOAT_EQ(0.0f, res.velocity);

	// getPoseAtDistanceAlongSegment function
	distance = 0;
	p = prim->getPoseAtDistanceAlongSegment(distance) ;
	EXPECT_FLOAT_EQ(1.0f, p.getOrigin().getX());
	EXPECT_FLOAT_EQ(0.0f, p.getOrigin().getY());

	// tangenQuat function
	pt.setX(res.poseOnSegment.getOrigin().getX());
	pt.setY(res.poseOnSegment.getOrigin().getY());
	pt.setZ(0);
	q = prim->tangentQuat(pt);
	EXPECT_FLOAT_EQ(0.0, q.getX());
	EXPECT_FLOAT_EQ(0.0, q.getY());
	EXPECT_NEAR(0.924f, q.getZ(),1e-3);
	EXPECT_NEAR(-0.382f, q.getW(),1e-3);


	// tangentVector function
	v =prim->tangentVector(pt);
	EXPECT_NEAR(-0.707f, v.getX(),1e-3);
	EXPECT_NEAR(-0.707f, v.getY(),1e-3);
	EXPECT_NEAR(0, v.getZ(),1e-3);

	/*
	 * Test case 3 : Valid segment(An arc of unit radius and it's a quarter). Robot pose is IN RANGE area and to the bottom of the start pose of the segment.
	 * Functions tested :
	 *  	* bool isInRange(tf::Pose pose) Should return true or the test terminates.
	 *  	* PathPrimitiveResult project(tf::Pose pose)
	 *  	* tf::Pose getPoseAtDistanceAlongSegment(double distance) Tested at a distance of 1.
	 */

	robotPose.setOrigin(tf::Vector3(2,-1,0));

	// isInRange function
	ASSERT_TRUE(prim->isInRange(robotPose));

	// project function
	res = prim->project(robotPose);
	EXPECT_NEAR(0.894f, res.poseOnSegment.getOrigin().getX(),1e-3);
	EXPECT_NEAR(-0.447f, res.poseOnSegment.getOrigin().getY(),1e-3);

	// getPoseAtDistanceAlongSegment function
	distance = 4;
	p= prim->getPoseAtDistanceAlongSegment(distance) ;
	EXPECT_FLOAT_EQ(0.0f, p.getOrigin().getX());
	EXPECT_FLOAT_EQ(1.0f, p.getOrigin().getY());

}

/**********************************************  Factory  *************************************************/
TEST(PathPrimitiveTest,Factory){
	path_primitive::PathPrimitiveBase *primBase;
	path_msgs::PathSegment _path;
	tf::Pose start, end;
	tf::Vector3 startVel, endVel;
	tf::Point center;

	/*
	 * Test Case 1 : Generate a line given a message object of path_msgs
	 */

	_path.type = path_msgs::PathSegment::TYPE_LINE;

	_path.line.startPose.position.x = 0.0;
	_path.line.startPose.position.y = 0.0;
	_path.line.startPose.position.z = 0.0;

	_path.line.endPose.position.x = 1.0;
	_path.line.endPose.position.y = 0.0;
	_path.line.endPose.position.z = 0.0;

	_path.line.startTwist.linear.x = 0.0;
	_path.line.startTwist.linear.y = 0.0;
	_path.line.startTwist.linear.z = 0.0;

	_path.line.endTwist.linear.x = 1.0;
	_path.line.endTwist.linear.y = 0.0;
	_path.line.endTwist.linear.z = 0.0;

	primBase = path_primitive::PathPrimitiveFactory::generate(_path);
	start.setOrigin(primBase->getStartPose().getOrigin());
	end.setOrigin(primBase->getEndPose().getOrigin());
	startVel = primBase->getStartVelocity();
	endVel = primBase->getEndVelocity();

	EXPECT_EQ(path_primitive::PathPrimitiveType::LINE,primBase->getType());
	EXPECT_FLOAT_EQ(0.0f, start.getOrigin().getX());
	EXPECT_FLOAT_EQ(1.0f, end.getOrigin().getX());
	EXPECT_FLOAT_EQ(0.0f, startVel.getX());
	EXPECT_FLOAT_EQ(1.0f, endVel.getX());

	/*
	 * Test Case 2 : Generate a circle given a message object of path_msgs
	 */

	_path.type = path_msgs::PathSegment::TYPE_CIRCLE;

	_path.circle.startPose.position.x = 1.0;
	_path.circle.startPose.position.y = 0.0;
	_path.circle.startPose.position.z = 0.0;
	_path.circle.startPose.orientation.x = 0;
	_path.circle.startPose.orientation.y = 0;
	_path.circle.startPose.orientation.z = 1;
	_path.circle.startPose.orientation.w = 1;

	_path.circle.endPose.position.x = 0.0;
	_path.circle.endPose.position.y = 1.0;
	_path.circle.endPose.position.z = 0.0;
	_path.circle.startPose.orientation.x = 0;
	_path.circle.endPose.orientation.y = 0;
	_path.circle.endPose.orientation.z = -1;
	_path.circle.endPose.orientation.w = 0;

	_path.circle.startTwist.linear.x = 0.0;
	_path.circle.startTwist.linear.y = 0.0;
	_path.circle.startTwist.linear.z = 0.0;

	_path.circle.endTwist.linear.x = 1.0;
	_path.circle.endTwist.linear.y = 0.0;
	_path.circle.endTwist.linear.z = 0.0;

	_path.circle.centerPoint.x = 0.0;
	_path.circle.centerPoint.y = 0.0;
	_path.circle.centerPoint.z = 0.0;

	primBase = path_primitive::PathPrimitiveFactory::generate(_path);

	start.setOrigin(primBase->getStartPose().getOrigin());
	start.setRotation(primBase->getStartPose().getRotation());
	end.setOrigin(primBase->getEndPose().getOrigin());
	end.setRotation(primBase->getStartPose().getRotation());
	startVel = primBase->getStartVelocity();
	endVel = primBase->getEndVelocity();
	path_primitive::PathPrimitiveCircle *primCircle = new path_primitive::PathPrimitiveCircle(start, end, startVel, endVel);
    center = primCircle->getCenter();

	EXPECT_EQ(path_primitive::PathPrimitiveType::CIRCLE,primBase->getType());
	EXPECT_FLOAT_EQ(1.0f, start.getOrigin().getX());
	EXPECT_FLOAT_EQ(1.0f, end.getOrigin().getY());
	EXPECT_FLOAT_EQ(0.0f, center.getX());
	EXPECT_FLOAT_EQ(0.0f, startVel.getX());
	EXPECT_FLOAT_EQ(1.0f, endVel.getX());

	/*
	 *  Test Case 3: Generate a line given a message object of PathPrimitiveBase class
	 */

	start.setOrigin(tf::Vector3(0,0,0));
	end.setOrigin(tf::Vector3(1,0,0));
	startVel.setValue(0,0,0);
	endVel.setValue(1,0,0);

	path_primitive::PathPrimitiveLine *primLine = new path_primitive::PathPrimitiveLine(start,end,startVel,endVel);

	_path = path_primitive::PathPrimitiveFactory::generate(*primLine);

	EXPECT_EQ(0,_path.type); // 0 - TYPE_LINE
	EXPECT_FLOAT_EQ(0.0f,_path.line.startPose.position.x);
	EXPECT_FLOAT_EQ(1.0f,_path.line.endPose.position.x);
	EXPECT_FLOAT_EQ(0.0f, _path.line.startTwist.linear.x);
	EXPECT_FLOAT_EQ(1.0f, _path.line.endTwist.linear.x);

	/*
	 *  Test Case 4: Generate a circle given a message object of PathPrimitiveBase class
	 */

	start.setOrigin(tf::Vector3(1,0,0));
	end.setOrigin(tf::Vector3(0,1,0));
	start.setRotation(tf::createQuaternionFromYaw(M_PI/2));
	end.setRotation(tf::createQuaternionFromYaw(-M_PI));
	center.setValue(0,0,0);
	startVel.setValue(0,0,0);
	endVel.setValue(1,0,0);

	*primCircle = path_primitive::PathPrimitiveCircle(start,end,startVel,endVel,center);

	_path = path_primitive::PathPrimitiveFactory::generate(*primCircle);

	EXPECT_EQ(1,_path.type); // 1 - TYPE_CIRCLE
	EXPECT_FLOAT_EQ(1.0f,_path.circle.startPose.position.x);
	EXPECT_FLOAT_EQ(1.0f,_path.circle.endPose.position.y);
	EXPECT_FLOAT_EQ(0.0f,_path.circle.centerPoint.x);
	EXPECT_FLOAT_EQ(0.0f, _path.circle.startTwist.linear.x);
	EXPECT_FLOAT_EQ(1.0f, _path.circle.endTwist.linear.x);

}

/******************************** PathprimitveBase Class ***********************************/
/*
 * Test : Checks the following functions,
 * 		* tf::Pose getLookaheadPose(int _segmentIndexToFollow, tf::Pose robotPose, std::vector<path_msgs::PathSegment>& _list, double desiredLookaheadDist, PathPrimitiveResult&, tf::Pose&)
 * 		* int getSegmentIndexToFollow(tf::Pose robotPose, std::vector<path_msgs::PathSegment>& _list, int currentGoalIndex);
 *      Creates a new list of valid path segments with a line, arc and a line.
 */
TEST(PathPrimitiveTest,Base){

	int currentGoalIndex= 2;
	double _lookAheadDist= 0.25;
	tf::Pose robotPose;
	robotPose.setOrigin(tf::Vector3(0.5,1,0));
	robotPose.setRotation(tf::createQuaternionFromYaw(-M_PI/2));
	std::vector<path_msgs::PathSegment> _list;
	path_primitive::PathPrimitiveResult _lookAheadResult;
	tf::Pose  _closestPointToReturn;
	path_msgs::PathSegment _path1, _path2, _path3;

	_path1.type = path_msgs::PathSegment::TYPE_LINE;

	_path1.line.startPose.position.x = 0.0;
	_path1.line.startPose.position.y = 0.0;
	_path1.line.startPose.position.z = 0.0;

	_path1.line.endPose.position.x = 1.0;
	_path1.line.endPose.position.y = 0.0;
	_path1.line.endPose.position.z = 0.0;

	_path1.line.startTwist.linear.x = 0.0;
	_path1.line.startTwist.linear.y = 0.0;
	_path1.line.startTwist.linear.z = 0.0;

	_path1.line.endTwist.linear.x = 1.0;
	_path1.line.endTwist.linear.y = 0.0;
	_path1.line.endTwist.linear.z = 0.0;

	_path2.type = path_msgs::PathSegment::TYPE_CIRCLE;

	_path2.circle.startPose.position.x = 1.0;
	_path2.circle.startPose.position.y = 0.0;
	_path2.circle.startPose.position.z = 0.0;
	_path2.circle.startPose.orientation.x = 0;
	_path2.circle.startPose.orientation.y = 0;
	_path2.circle.startPose.orientation.z = 1;
	_path2.circle.startPose.orientation.w = 1;

	_path2.circle.endPose.position.x = 0.0;
	_path2.circle.endPose.position.y = 1.0;
	_path2.circle.endPose.position.z = 0.0;
	_path2.circle.endPose.orientation.x = 0;
	_path2.circle.endPose.orientation.y = 0;
	_path2.circle.endPose.orientation.z = -1;
	_path2.circle.endPose.orientation.w = 0;

	_path2.circle.startTwist.linear.x = 0.0;
	_path2.circle.startTwist.linear.y = 0.0;
	_path2.circle.startTwist.linear.z = 0.0;

	_path2.circle.endTwist.linear.x = 1.0;
	_path2.circle.endTwist.linear.y = 0.0;
	_path2.circle.endTwist.linear.z = 0.0;

	_path2.circle.centerPoint.x = 0.0;
	_path2.circle.centerPoint.y = 0.0;
	_path2.circle.centerPoint.z = 0.0;

	_path2.type = path_msgs::PathSegment::TYPE_LINE;

	_path3.line.startPose.position.x = 0.0;
	_path3.line.startPose.position.y = 1.0;
	_path3.line.startPose.position.z = 0.0;
	_path3.line.startPose.orientation.x = 0;
	_path3.line.startPose.orientation.y = 0;
	_path3.line.startPose.orientation.z = -1;
	_path3.line.startPose.orientation.w = 1;

	_path3.line.endPose.position.x = 0.0;
	_path3.line.endPose.position.y = 0.0;
	_path3.line.endPose.position.z = 0.0;
	_path3.line.endPose.orientation.x = 0;
	_path3.line.endPose.orientation.y = 0;
	_path3.line.endPose.orientation.z = 0;
	_path3.line.endPose.orientation.w = 0;

	_path3.line.startTwist.linear.x = 0.0;
	_path3.line.startTwist.linear.y = 0.0;
	_path3.line.startTwist.linear.z = 0.0;

	_path3.line.endTwist.linear.x = 1.0;
	_path3.line.endTwist.linear.y = 0.0;
	_path3.line.endTwist.linear.z = 0.0;

	_list = {_path1, _path2, _path3};

	int _segmentIndexToFollow = path_primitive::getSegmentIndexToFollow(robotPose, _list, currentGoalIndex);

	tf::Pose lookAheadPose = path_primitive::getLookaheadPose( _segmentIndexToFollow, robotPose, _list, _lookAheadDist, _lookAheadResult, _closestPointToReturn);

	EXPECT_EQ(2,_segmentIndexToFollow);
	EXPECT_NEAR(0.0f,lookAheadPose.getOrigin().getX(),1e-3);
	EXPECT_FLOAT_EQ(0.75f,lookAheadPose.getOrigin().getY());
	EXPECT_FLOAT_EQ(0.0f,lookAheadPose.getOrigin().getZ());

}

/******************************** Bezier Curves ***********************************/
//TEST(PathPrimitives, bezierLine) {
//	tf::Pose p0,p1,p2,p3;
//	p0.setOrigin(tf::Vector3(0,0,0));
//	p1.setOrigin(tf::Vector3(1,0,0));
//	p2.setOrigin(tf::Vector3(2,0,0));
//	p3.setOrigin(tf::Vector3(3,0,0));
//	path_primitive::PathPrimitiveBezier *prim = new path_primitive::PathPrimitiveBezier(p0,p1,p2,p3,tf::Vector3(0,0,0), tf::Vector3(1,0,0));
//
//	tf::Pose range0;
//	tf::Pose range1;
//	tf::Pose range2;
//	range0.setOrigin(tf::Vector3(1,0.5,0));
//	range1.setOrigin(tf::Vector3(-1,0.5,0));
//	range2.setOrigin(tf::Vector3(5,0.5,0));
//	EXPECT_TRUE(prim->isInRange(range0));
//	EXPECT_FALSE(prim->isInRange(range1));
//	EXPECT_FALSE(prim->isInRange(range2));
//
//	tf::Vector3 p = prim->evaluate(0);
//	EXPECT_FLOAT_EQ(0.0f, p.getX());
//	EXPECT_FLOAT_EQ(0.0f, p.getY());
//	EXPECT_FLOAT_EQ(0.0f, p.getZ());
//
//	p = prim->evaluate(1);
//	EXPECT_FLOAT_EQ(3.0f, p.getX());
//	EXPECT_FLOAT_EQ(0.0f, p.getY());
//	EXPECT_FLOAT_EQ(0.0f, p.getZ());
//
//	p = prim->evaluate(0.5f);
//	EXPECT_NEAR(1.5f, p.getX(), 1e-6);
//	EXPECT_NEAR(0.0f, p.getY(), 1e-6);
//	EXPECT_NEAR(0.0f, p.getZ(), 1e-6);
//
//	EXPECT_NEAR(3.0f, prim->length(), 1e-6);
//
//	path_primitive::PathPrimitiveResult result;
//
//	tf::Pose point;
//	point.setOrigin(tf::Vector3(1.5f,0,0));
//	point.setRotation(tf::createQuaternionFromYaw(0));
//	result = prim->project(point);
//
//	EXPECT_NEAR(1.5f, result.poseOnSegment.getOrigin().x(), 1e-6);
//	EXPECT_NEAR(0.0f, result.poseOnSegment.getOrigin().y(), 1e-6);
//	point.setOrigin(tf::Vector3(1.5f,1.0f,0));
//	result = prim->project(point);
//
//	EXPECT_NEAR(1.5f, result.poseOnSegment.getOrigin().x(), 1e-6);
//	EXPECT_NEAR(0.0f, result.poseOnSegment.getOrigin().y(), 1e-6);
///*
//	double length, u[6] = {0.0, 0.49, 0.5, 0.501, 0.75, 1.0};
//	for(int i = 0; i < 6; i++)
//	{
//		length = prim->getLengthAt(u[i]);
//		printf("u: %1.4f len: %1.17f\n", u[i], length);
//
//	}
//
//	double s[6] = {0.0, 1, 15, 15.5, 17.5, 30.0};
//	for(int i = 0; i < 6; i++)
//	{
//		length = prim->getParameterForLength(s[i]);
//		printf("s: %1.4f param: %1.17f\n", s[i], length);
//	}
//	*/
//
//	delete prim;
//}
//
//TEST(PathPrimitives, bezierHorseshoe)
//{
//	tf::Pose p0,p1,p2,p3;
//	p0.setOrigin(tf::Vector3(0,0,0));
//	p1.setOrigin(tf::Vector3(1,0,0));
//	p2.setOrigin(tf::Vector3(1,1,0));
//	p3.setOrigin(tf::Vector3(0,1,0));
//	path_primitive::PathPrimitiveBezier *prim = new path_primitive::PathPrimitiveBezier(p0,p1,p2,p3,tf::Vector3(0,0,0), tf::Vector3(1,0,0));
//
//	prim->tangentVector(0.0);
//	prim->tangentVector(0.5);
//	prim->tangentVector(0.75);
//	prim->tangentVector(0.85);
//	prim->tangentVector(1.0);
//
//	prim->curvatureAt(0.0);
//	prim->curvatureAt(0.5);
//	prim->curvatureAt(0.75);
//	prim->curvatureAt(0.85);
//	prim->curvatureAt(1.0);
//
//	tf::Pose pose;
//	pose.setOrigin(tf::Vector3(-1.5,0.0,0));
//	pose.setRotation(tf::createQuaternionFromYaw(M_PI*0.5));
//	path_primitive::PathPrimitiveResult result;
//	result = prim->project(pose);
////	printf("Closest u:%1.9f, x: %1.9f, y: %1.9f, remain: %1.5f, dist: %1.6f\n", result.parameter, result.poseOnSegment.getOrigin().x(),
////			result.poseOnSegment.getOrigin().y(), result.distanceFromEnd, result.distanceError);
//
//	delete prim;
//
//}


int main(int argc, char **argv){
testing::InitGoogleTest(&argc, argv);
return RUN_ALL_TESTS();
}
