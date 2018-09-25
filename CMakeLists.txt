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
  GoogleDeps
  Shm
)

list(APPEND CMAKE_PREFIX_PATH ${GoogleDeps_SOURCE_PREFIX}/share/cartographer)

find_package(cartographer REQUIRED)

include("${CARTOGRAPHER_CMAKE_DIR}/functions.cmake")

set(BUILD_SHARED_LIBS OFF)
option(BUILD_GRPC "build features that require Cartographer gRPC support" false)
google_initialize_cartographer_project()
google_enable_testing()

include(FindPkgConfig)

find_package(LuaGoogle REQUIRED)
find_package(Ceres REQUIRED)

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp
                 std_msgs
)

if (CMAKE_BUILD_TYPE STREQUAL Debug)
  add_definitions(-DDEBUG)
endif()

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

add_executable(${PROJECT_NAME}
  src/${PROJECT_NAME}NodeMain.cc
  src/${PROJECT_NAME}Node.cc
  src/UdpComm.cpp
)

target_link_libraries(${PROJECT_NAME}
  cartographer
  ${catkin_LIBRARIES}
)
