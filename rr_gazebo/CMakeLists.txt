cmake_minimum_required(VERSION 2.8.3)
project(rr_gazebo)
set(CMAKE_BUILD_TYPE Release)
add_compile_options(-std=c++11)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  rr_common
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  roslint
  tf
)
catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})
add_executable(simulated_odom src/simulated_odom.cpp 
               src/simulated_odom_node.cpp include/simulated_odom.hpp)
target_link_libraries(simulated_odom ${catkin_LIBRARIES})
roslint_cpp(src/simulated_odom.cpp src/simulated_odom_node.cpp include/simulated_odom.h)

install(PROGRAMS
  scripts/ackermann_controller.py
  scripts/connect_controller.py
  scripts/vehicle_joystick.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
  FILES_MATCHING PATTERN "*.launch" PATTERN "*.machine" PATTERN "*.yaml" PATTERN "*.urdf"
)
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
  FILES_MATCHING PATTERN "*.yaml"
)
