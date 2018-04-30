cmake_minimum_required(VERSION 2.8.3)
project(rr_vehicle_gazebo)
find_package(catkin REQUIRED)
catkin_package()

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