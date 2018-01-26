cmake_minimum_required(VERSION 2.8.3)
project(rr_supervisor)

find_package(catkin REQUIRED COMPONENTS 
             roscpp 
             rospy 
             std_msgs
             roslint)


###################################
## catkin specific configuration ##
###################################

catkin_package(
   INCLUDE_DIRS 
   CATKIN_DEPENDS 
             roscpp 
             rospy 
             std_msgs 
   DEPENDS system_lib rr_libraries)

###########
## Build ##
###########

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
add_executable(rr_supervisor src/rr_supervisor.cpp src/main.cpp include/rr_supervisor.h)
target_link_libraries(rr_supervisor EnableCatcher ${catkin_LIBRARIES}) 

roslint_cpp(src/main.cpp src/rr_supervisor.cpp include/rr_supervisor.h)

