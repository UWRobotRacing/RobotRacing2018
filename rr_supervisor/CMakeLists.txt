cmake_minimum_required(VERSION 2.8.3)
project(rr_supervisor)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
)

catkin_package()

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_executable(supervisor
  src/main.cpp
)

target_link_libraries(supervisor
  ${catkin_LIBRARIES}
)
