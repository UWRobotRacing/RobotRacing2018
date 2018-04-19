cmake_minimum_required(VERSION 2.8.3)
project(rr_endline_detection)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  cv_bridge
  image_transport
)

## Generate added messages and services with any dependencies listed here
# generate_messages(
#   DEPENDENCIES
#   std_msgs
# )

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rr_endline_detection
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_executable(endline_detection 
src/main.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(endline_detection
  ${catkin_LIBRARIES}
)
