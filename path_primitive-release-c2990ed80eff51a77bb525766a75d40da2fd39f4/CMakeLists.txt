cmake_minimum_required(VERSION 2.8.3)
project(path_primitive)

## NOTE: We do this because we want C++11.  However, this is forbidden by
##       ROS catkin coding standards
##  http://docs.ros.org/indigo/api/catkin/html/user_guide/standards.html
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  path_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES
    path_primitive
    path_validation
  CATKIN_DEPENDS
    roscpp
    tf
    path_msgs
)

###########
## Build ##
###########
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/path_primitive_line.cpp
  src/path_primitive_circle.cpp
  src/path_primitive_bezier.cpp
  src/path_primitive_factory.cpp
  src/path_primitive_base.cpp
  src/path_primitive_array.cpp
  src/path_validation.cpp
)

add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)

#link_directories( lib )

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${PROJECT_SOURCE_DIR}/lib/libtinyspline.so
  ${PROJECT_SOURCE_DIR}/lib/libtinysplinecpp.so
)

add_library(path_validation
  src/path_validation.cpp
)

add_dependencies(path_validation
  ${catkin_EXPORTED_TARGETS}
)

target_link_libraries(path_validation
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(TARGETS path_validation
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY lib/
	DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

#############
## Testing ##
#############

# Add gtest based cpp test target and link libraries
if (CATKIN_ENABLE_TESTING)
	catkin_add_gtest(${PROJECT_NAME}-test src/test/test_primitives.cpp)
	target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
