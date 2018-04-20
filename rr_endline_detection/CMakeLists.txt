cmake_minimum_required(VERSION 2.8.3)
project(rr_endline_detection)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  cv_bridge
  image_transport
)
find_package(OpenCV)

catkin_package()

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_executable(endline_detection 
  src/main.cpp
  include/endline_detection.hpp
  src/endline_detection.cpp
)

target_link_libraries(endline_detection
  ${catkin_LIBRARIES}
)
