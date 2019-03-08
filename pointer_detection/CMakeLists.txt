cmake_minimum_required(VERSION 2.8.3)
project(pointer_detection)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
	roscpp
	geometry_msgs
	sensor_msgs
	lab_msgs
	cv_bridge 
	image_transport  
	message_filters   
	tf
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

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
   INCLUDE_DIRS include
#  LIBRARIES pointer_detection
#  CATKIN_DEPENDS cv_bridge geometry_msgs image_transport roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########
include_directories(include
                    ${catkin_INCLUDE_DIRS})

add_executable(pointer_localizer_ie src/PointerLocalizerIE.cpp)
target_link_libraries(pointer_localizer_ie ${catkin_LIBRARIES})
add_dependencies(pointer_localizer_ie pointer_detection_generate_messages_cpp ${catkin_EXPORTED_TARGETS})

add_executable(pointer_localizer_robot src/PointerLocalizerRobot.cpp)
target_link_libraries(pointer_localizer_robot ${catkin_LIBRARIES})
add_dependencies(pointer_localizer_robot pointer_detection_generate_messages_cpp ${catkin_EXPORTED_TARGETS})
