cmake_minimum_required(VERSION 2.8.3)
project(rr_path_planner)

find_package(catkin REQUIRED
             roscpp 
	     rospy 
	     std_msgs 
	     nav_msgs 
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
			 rospy 
			 std_msgs 
			 nav_msgs 
			 geometry_msgs
			 visualization_msgs
   DEPENDS system_lib)

###########
## Build ##
###########
roslint_cpp(src/main.cpp src/PathPlanner2k16.cpp include/PathPlanner2k16.h)

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
add_executable(rr_path_planner src/main.cpp)
add_library(PathPlanner2k16 src/PathPlanner2k16.cpp include/PathPlanner2k16.h)

## add_executable(rr_path_planner src/main.cpp src/rr_path_planner.cpp include/PathPlanner2k16.h)

## Specify libraries to link a library or executable target against
target_link_libraries(rr_path_planner PathPlanner2k16 ${catkin_LIBRARIES} ${rr_libraries_LIBRARIES})

target_link_libraries(rr_path_planner PathPlanner2k16 ${catkin_LIBRARIES})
