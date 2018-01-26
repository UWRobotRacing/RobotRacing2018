cmake_minimum_required(VERSION 2.8.3)
project(vis)

set(CMAKE_BUILD_TYPE Debug)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs sensor_msgs geometry_msgs image_transport cv_bridge)
find_package(OpenCV)

catkin_package()

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(lineNode src/lineNode.cpp src/vision_fun_2015.cpp src/visionprocessor.cpp include/visionprocessor.hpp include/vision_fun_2015.hpp)
target_link_libraries(lineNode ${catkin_LIBRARIES})
