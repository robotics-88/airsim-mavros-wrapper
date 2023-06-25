cmake_minimum_required(VERSION 3.0.2)
project(airsim_launch)

# set this to path to AirSim root folder if you want your catkin workspace in a custom directory
set(AIRSIM_ROOT $ENV{HOME}/src/AirSim/)

add_subdirectory("${AIRSIM_ROOT}/cmake/rpclib_wrapper" rpclib_wrapper)
add_subdirectory("${AIRSIM_ROOT}/cmake/AirLib" AirLib)
add_subdirectory("${AIRSIM_ROOT}/cmake/MavLinkCom" MavLinkCom)

set(CMAKE_CXX_STANDARD 17)

set(RPC_LIB_INCLUDES " ${AIRSIM_ROOT}/external/rpclib/rpclib-2.3.0/include")
set(RPC_LIB rpc) # name of .a file with lib prefix
message(STATUS "found RPC_LIB_INCLUDES=${RPC_LIB_INCLUDES}")

find_package(OpenCV REQUIRED)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  mavros_msgs
  nav_msgs
  octomap_rviz_plugins
  rosbridge_server
  roscpp
  roslib
  std_msgs
  tf2
  tf2_geometry_msgs
  tf2_ros
  tf2_sensor_msgs
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
   INCLUDE_DIRS include
#  LIBRARIES airsim_ros
   CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${AIRSIM_ROOT}/AirLib/deps/eigen3
  ${AIRSIM_ROOT}/AirLib/include
  ${RPC_LIB_INCLUDES}
  ${AIRSIM_ROOT}/MavLinkCom/include
  ${AIRSIM_ROOT}/MavLinkCom/common_utils
  ${OpenCV_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(airsim_node
  src/airsim_ros_node.cpp
)

add_library(airsim_ros 
  src/airsim_ros_wrapper.cpp
  src/airsim_ned_wrapper.cpp
)
target_link_libraries(airsim_ros ${catkin_LIBRARIES} ${OpenCV_LIBS} yaml-cpp AirLib)

add_dependencies(airsim_ros ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(airsim_node airsim_ros ${catkin_LIBRARIES} ${OpenCV_LIBS} AirLib)


## Specify libraries to link a library or executable target against
target_link_libraries(airsim_node
  ${catkin_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS
    #list of nodes
    airsim_node
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS 
    #list of shared libraries
    airsim_ros
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(FILES 
  #list of necessary files (xml...)
  README.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
