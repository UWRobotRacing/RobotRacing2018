cmake_minimum_required(VERSION 2.8.3)
project(rr_lane_detection)

set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  geometry_msgs
  image_transport
  cv_bridge
)

find_package(OpenCV REQUIRED)

catkin_package()

include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(lane_detection src/lane_detection.cpp src/thresholding.cpp src/lane_detection_processor.cpp include/lane_detection_processor.hpp include/thresholding.hpp)
target_link_libraries(lane_detection ${catkin_LIBRARIES} ${OpenCV_LIBS})