cmake_minimum_required(VERSION 3.5)
project(airsim_launch)


find_package(OpenCV REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

## Find macros and libraries
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_sensor_msgs REQUIRED)
find_package(rclpy REQUIRED)
find_package(tf2 REQUIRED)
find_package(image_transport REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_typesupport_cpp REQUIRED)
find_package(rosidl_default_runtime REQUIRED)

# set this to path to AirSim root folder if you want your workspace in a custom directory
set(AIRSIM_DIR $ENV{AIRSIM_DIR})

add_subdirectory("${AIRSIM_DIR}/cmake/rpclib_wrapper" rpclib_wrapper)
add_subdirectory("${AIRSIM_DIR}/cmake/AirLib" AirLib)
add_subdirectory("${AIRSIM_DIR}/cmake/MavLinkCom" MavLinkCom)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-O3 -Wall -Wextra -Wnoexcept -Wstrict-null-sentinel")
set(CXX_EXP_LIB "-nostdinc++ -I/usr/include/c++/8 -I/usr/include/x86_64-linux-gnu/c++/8 -nodefaultlibs 
-l/usr/lib/x86_64-linux-gnu/libc++.so -l/usr/lib/x86_64-linux-gnu/libc++abi.so
-lm -lc -lgcc_s -lgcc
-lstdc++fs -fmax-errors=10")
set(RPC_LIB_INCLUDES " ${AIRSIM_DIR}/external/rpclib/rpclib-2.3.0/include")
set(RPC_LIB rpc)
message(STATUS "found RPC_LIB_INCLUDES=${RPC_LIB_INCLUDES}")

###########
## Build ##
###########


set(INCLUDE_DIRS include 
  ${AIRSIM_DIR}/AirLib/deps/eigen3
  ${AIRSIM_DIR}/AirLib/include
  ${RPC_LIB_INCLUDES}
  ${AIRSIM_DIR}/MavLinkCom/include
  ${AIRSIM_DIR}/MavLinkCom/common_utils
  ${OpenCV_INCLUDE_DIRS}
  )
include_directories(${INCLUDE_DIRS})

## Declare a C++ executable
add_executable(airsim_node
  src/airsim_mavros_node.cpp
)

add_library(airsim_mavros 
  src/airsim_mavros.cpp
)
target_link_libraries(airsim_mavros ${OpenCV_LIBS} yaml-cpp AirLib)

ament_target_dependencies(airsim_mavros 
  rclcpp
  sensor_msgs
  geometry_msgs
  mavros_msgs
  nav_msgs
  image_transport
  tf2_geometry_msgs
  tf2_ros
  tf2_sensor_msgs
  cv_bridge
  std_srvs
)
target_link_libraries(airsim_node airsim_mavros ${OpenCV_LIBS} AirLib)

ament_target_dependencies(airsim_node 
  rclcpp
)

#############
## Install ##
#############

install(TARGETS
    #list of nodes
    airsim_node
    DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS 
    #list of shared libraries
    airsim_mavros
    ARCHIVE
    DESTINATION lib
    LIBRARY
    DESTINATION lib
)

install(FILES 
  #list of necessary files (xml...)
  README.md
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

ament_package()