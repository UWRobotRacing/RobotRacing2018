cmake_minimum_required(VERSION 2.8.3)
project(rr_mapper)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

set(CMAKE_BUILD_TYPE Release)

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

catkin_package()
roslint_cpp()

include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(rr_mapper src/main.cpp src/laser_mapper.cpp src/occupancy_grid_utils.cpp)
target_link_libraries(rr_mapper ${catkin_LIBRARIES}) 
