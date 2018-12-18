# Path Primitive library

A library used by virtual rail controller for processing the path segments and also obtain information about path segments in behavior layer. The framework has functions to 

* generate path segments in the form of lines, arcs or bezier lines
* project points from the path segment
* check if the robot is within the range of path
* calculate the distance of the robot from the path

	The library takes in path through path_msgs mostly and outputs a object of the struct PathPrimitiveResult that gives information about the robot's projection on path segments and errors in distance and heading with respect to the path. The hierarchy of the classes in this library is shown in the image below.

![Hierarchy of classes](/AGVdev/resources/images/hoc.jpg)

## Code example

The following are usages of few methods in the library

* **bool isInRange(tf::Pose pose)** :

	It checks if the given robot pose is within the range of the path. The image below shows how the range areas have been defined incase of line and arc.

![isInRange()](/resources/images/isInRange.jpg)


* **PathPrimitiveResult project(tf::Pose pose)** :

	It projects the given robot pose onto line or arc segment. The image below shows the projection point. <TODO : Add the changess made in project function>

![project()](/resources/images/project.jpg)


* **double curvatureAt(tf::Pose pose)** :

	It returns the reciprocal of radius of an arc or circle. It is the amount of deviation of a segment from being straight. It is zero for line.


* **double distanceError(tf::Pose pose)** :

	It calculates the  distance between the given point and the segment. The image below illustrates various cases of distance error and heading error.

![project()](/resources/images/distError.jpg) 

Note : The distance error and heading error are signed. For the distanceError, the negative/positive sign indicate the given pose is on the left/right side with respect to the heading of Start pose of the segment.For the headingError the negative/positive sign indicate the given pose is opposite/same direction with respect to the heading of the Start pose of the segment. Reference : [Distance formula](https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line)

* **tf::Pose getPoseAtDistanceAlongSegment(double distance)** :

	It finds the pose on the segment at a given distance.


## Tests

The tests/test_primitive.cpp runs units tests on PathPrimitiveLine, PathprimitiveCircle and PathPrimitiveBerziers.To run the file, run the following command in the workspace,

```
$ catkin_make run_test_path_primitive

```
<TODO : Edit the test cases and diagrams>

### Test for Lines

   There are five test cases, four test cases run test on a valid path segment and the last test case runs on an invalid segment. Each test case has a new robot pose in/out of range area. The image below shows three tested robot poses and the path segment.

![project()](/resources/images/Test_Line.jpg)
 
### Test for Circle/Arcs with center

   There are three test cases. Each test case has a new robot pose in/out of range area.This test passes center point as an input argument to the constructor. The image below shows three tested robot poses and the path segment
	
![project()](/resources/images/Test_Arcs.jpg)

### Test for circle/Arcs without center

   Similar to the previous test.The only difference is this test does not input center point for arc and it is calculated inside the constructor. The image below shows three tested robot poses and the path segment

### Test for Factory class

   There are two test cases to check the generation of path segments give the path_msgs/ an object of PathPrimitiveBase class . Each function is tested twice for generating a line and circle, hence there are four test cases in total

### Test for PathPrimitiveBase class
 
   This test checks the two functions gor next segment index to follow and generation of look ahead pose with list of path segments as shown in the image below,

![project()](/resources/images/Test_lookahead.jpeg)


## Version

TODO : Include version of the library and tools used

## License

Copyright (c) 5D Robotics, Inc.

This code is proprietary and confidential. Permission to use it in any way,
including but not limited to viewing, copying, modifying, distributing, or
selling this software and its documentation for any purpose remains at the
sole discretion of 5D Robotics, Inc. and must be approved in writing.
