cmake_minimum_required(VERSION 2.8.3)
project(rr_mapper)

find_package(catkin REQUIRED COMPONENTS 
			 roscpp 
			 rospy 
			 std_msgs 
			 sensor_msgs 
			 nav_msgs
			 tf
       roslint)

find_package(rr_libraries REQUIRED)

###################################
## catkin specific configuration ##
###################################

catkin_package(
   INCLUDE_DIRS include
   CATKIN_DEPENDS 
			 roscpp 
			 rospy 
			 std_msgs 
			 sensor_msgs 
			 nav_msgs
			 tf
   DEPENDS system_lib rr_libraries)

###########
## Build ##
###########
roslint_cpp()

# Get workspace directory for catkin
# get_filename_component(PARENT_DIR ${PROJECT_SOURCE_DIR} PATH)
# include_directories(include ${catkin_INCLUDE_DIRS} ${PARENT_DIR}/rr_libraries/include) 

include_directories(include ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})
include_directories(include ${roscpp_INCLUDE_DIRS} ${rr_libraries_INCLUDE_DIRS}) 

add_executable(rr_mapper src/main.cpp src/LaserMapper.cpp)

target_link_libraries(rr_mapper ${roscpp_LIBRARIES} ${rr_libraries_LIBRARIES}) 
