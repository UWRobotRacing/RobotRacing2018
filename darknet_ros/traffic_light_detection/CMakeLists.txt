cmake_minimum_required(VERSION 2.8.3)
project(traffic_light_detection)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  darknet_ros_msgs
  cv_bridge
  image_transport
)

catkin_package(
	INCLUDE_DIRS include
  CATKIN_DEPENDS
    darknet_ros_msgs
    cv_bridge
)

include_directories(include 
	${catkin_INCLUDE_DIRS}
)

add_executable(traffic_light_detector
	src/main.cpp
	src/traffic_light_detector.cpp
)

target_link_libraries(traffic_light_detector
	${catkin_LIBRARIES}
)
