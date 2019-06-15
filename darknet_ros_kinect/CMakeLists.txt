cmake_minimum_required(VERSION 2.8.3)
project(darknet_ros_kinect)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  message_generation
)

## Generate messages in the 'msg' folder
add_message_files(
	FILES
	ObjectPosition.msg
	ObjectPositions.msg
)

## Generate added messages and services with any dependencies listed here
generate_messages(
	DEPENDENCIES
	std_msgs  # Or other packages containing msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES darknet_ros_kinect
#  CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
	CATKIN_DEPENDS message_runtime
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


find_package(PCL 1.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

set( OpenCV_DIR "/usr/local/include/opencv2/" )
find_package( OpenCV REQUIRED )

set(CMAKE_CXX_FLAGS "-pthread")

add_executable(darknet_kinect src/kinect.cpp)
target_link_libraries(darknet_kinect ${OpenCV_LIBS} ${catkin_LIBRARIES} ${PCL_LIBRARIES})
