cmake_minimum_required(VERSION 2.8.3)
project(gazebo_ros_vacuum_gripper)

find_package(catkin REQUIRED COMPONENTS
  gazebo_ros
  roscpp
  std_msgs
  gazebo_msgs
)

find_package (gazebo REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS gazebo_ros roscpp
)

###########
## Build ##
###########

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(include)
include_directories( ${catkin_INCLUDE_DIRS} 
                     ${Boost_INCLUDE_DIR} 
                     ${GAZEBO_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/gazebo_ros_vacuum_gripper.cpp)
## Specify libraries to link a library or executable target against
target_link_libraries( ${PROJECT_NAME} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES} )
