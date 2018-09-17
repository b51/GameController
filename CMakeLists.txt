cmake_minimum_required(VERSION 2.8.3)
project(GameController)

## Compile as C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  ikid_msgs
  Config
)

include(${Config_SOURCE_PREFIX}/cmake/GoogleDeps.cmake)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
 ${GOOGLE_INCLUDE_DIRS}

)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp
                 std_msgs
                 ikid_msgs
                 Config
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

add_executable(GameControllerNodeMain
  src/GameControllerNodeMain.cc
  src/GameControllerNode.cc
  src/GameController.cc
  src/UdpComm.cpp
)

target_link_libraries(GameControllerNodeMain
  ${catkin_LIBRARIES}
  ${GOOGLE_LIBS}
)
