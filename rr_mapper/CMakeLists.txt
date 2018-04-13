cmake_minimum_required(VERSION 2.8.3)
project(rr_mapper)

find_package(
	catkin REQUIRED COMPONENTS 
	roscpp 
	rospy 
	std_msgs 
	sensor_msgs 
	nav_msgs
	tf
	roslint
)

catkin_package(
	INCLUDE_DIRS include
	CATKIN_DEPENDS 
	roscpp 
	rospy 
	std_msgs 
	sensor_msgs 
	nav_msgs
	tf
)

roslint_cpp()

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
include_directories(include ${roscpp_INCLUDE_DIRS}) 
add_executable(rr_mapper src/main.cpp src/laser_mapper.cpp src/occupancy_grid_utils.cpp)
target_link_libraries(rr_mapper ${roscpp_LIBRARIES}) 