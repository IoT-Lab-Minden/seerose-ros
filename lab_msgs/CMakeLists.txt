cmake_minimum_required(VERSION 2.8.3)
project(lab_msgs)

## Add support for C++11, supported in ROS Kinetic and newer
# add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  geometry_msgs 
  message_generation
)


## Generate messages in the 'msg' folder
add_message_files(
	FILES
	Plane.msg
	Line.msg
	PointingGesture.msg
	CameraProbability.msg
	ExperimentResult.msg
	VirtualBorder.msg
	VirtualBorderArray.msg
	RegionOfInterestStamped.msg
)

## Generate services in the 'srv' folder
add_service_files(
    FILES
    MapUpdate.srv
    GetUInt8.srv
    SetUInt8.srv
    GetObjects.srv
    CheckCameraResponsibility.srv
    FindObjectWithMaxLocalProb.srv
    CallCameraSelection.srv
    GetObjForId.srv
    GetLastMaxProbObject.srv
    GraspPosition.srv
    GetPoint.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs 
)


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  CATKIN_DEPENDS message_runtime std_msgs
)

