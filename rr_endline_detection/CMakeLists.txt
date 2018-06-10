cmake_minimum_required(VERSION 2.8.3)
project(rr_endline_detection)

set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  std_srvs
  cv_bridge
  rr_common
  image_transport
)
find_package(OpenCV REQUIRED)

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
  ${OpenCV_LIBS}
)
