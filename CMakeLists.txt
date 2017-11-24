cmake_minimum_required(VERSION 2.8.3)
project(blind_controller)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -std=c++0x")
set(CMAKE_BUILD_TYPE "Release")

add_definitions(-DEIGEN_DONT_ALIGN_STATICALLY)

SET( CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${PROJECT_SOURCE_DIR} )
	  
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  message_generation
)

add_message_files(
  FILES
  ChangeDirection.msg
  Motion.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
CATKIN_DEPENDS message_runtime
)

include_directories(
  src
  ${catkin_INCLUDE_DIRS}
)

add_subdirectory(src)
