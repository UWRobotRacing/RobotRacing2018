cmake_minimum_required(VERSION 2.8.3)
project(rr_controller)

find_package(catkin REQUIRED COMPONENTS 
			 roscpp 
			 rospy 
			 std_msgs 
			 sensor_msgs 
			 geometry_msgs)


###################################
## catkin specific configuration ##
###################################

catkin_package(
   INCLUDE_DIRS 
   CATKIN_DEPENDS 
			 roscpp 
			 rospy 
			 std_msgs 
			 sensor_msgs 
			 geometry_msgs
   DEPENDS system_lib)

###########
## Build ##
###########

#add_executable(robotracing_pid src/robotracing_pid_control.cpp)
add_executable(rr_controller src/rr_controller.cpp)
#add_executable(robotracing_long_control src/robotracing_long_control.cpp)


#target_link_libraries(robotracing_pid ${catkin_LIBRARIES}) 
target_link_libraries(rr_controller ${catkin_LIBRARIES}) 
#target_link_libraries(robotracing_long_control ${catkin_LIBRARIES}) 
