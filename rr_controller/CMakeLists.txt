cmake_minimum_required(VERSION 2.8.3)
project(rr_controller)

find_package(catkin REQUIRED
             roscpp 
	     std_msgs 
	     geometry_msgs
             roslint
)

###################################
##       catkin custom msgs      ##
###################################


###################################
## catkin specific configuration ##
###################################

catkin_package(
   INCLUDE_DIRS 
   CATKIN_DEPENDS 
			 roscpp 
			 std_msgs 
			 geometry_msgs
   DEPENDS system_lib)

###########
## Build ##
###########


include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
add_executable(rr_controller src/controller.cpp src/main.cpp include/controller.h)
roslint_cpp()

## Specify libraries to link a library or executable target against
target_link_libraries(rr_controller ${catkin_LIBRARIES})
