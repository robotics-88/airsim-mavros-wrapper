cmake_minimum_required(VERSION 3.5)
project(airsim_launch)


find_package(OpenCV REQUIRED)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

## Find catkin macros and libraries
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
find_package(airsim_interfaces REQUIRED)

# set this to path to AirSim root folder if you want your catkin workspace in a custom directory
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
set(RPC_LIB_INCLUDES " ${AIRSIM_ROOT}/external/rpclib/rpclib-2.3.0/include")
set(RPC_LIB rpc)
message(STATUS "found RPC_LIB_INCLUDES=${RPC_LIB_INCLUDES}")

###########
## Build ##
###########


set(INCLUDE_DIRS include 
  ${AIRSIM_ROOT}/AirLib/deps/eigen3
  ${AIRSIM_ROOT}/AirLib/include
  ${RPC_LIB_INCLUDES}
  ${AIRSIM_ROOT}/MavLinkCom/include
  ${AIRSIM_ROOT}/MavLinkCom/common_utils
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
target_link_libraries(airsim_mavros ${catkin_LIBRARIES} ${OpenCV_LIBS} yaml-cpp AirLib)

# add_dependencies(airsim_mavros ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
ament_target_dependencies(airsim_mavros 
  rclcpp
  sensor_msgs
  geometry_msgs
  mavros_msgs
  nav_msgs
  image_transport
  tf2_ros
  cv_bridge
)
target_link_libraries(airsim_node airsim_mavros ${catkin_LIBRARIES} ${OpenCV_LIBS} AirLib)


## Specify libraries to link a library or executable target against
target_link_libraries(airsim_node
  ${catkin_LIBRARIES}
)
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



ament_export_dependencies(rclcpp)
ament_export_dependencies(nav_msgs)
ament_export_dependencies(geographic_msgs)
ament_export_dependencies(std_srvs)
ament_export_dependencies(tf2_ros)
ament_export_dependencies(tf2_sensor_msgs)
ament_export_dependencies(rclpy)
ament_export_dependencies(tf2)
ament_export_dependencies(image_transport)
ament_export_dependencies(geometry_msgs)
ament_export_dependencies(cv_bridge)
ament_export_dependencies(tf2_geometry_msgs)
ament_export_dependencies(sensor_msgs)
ament_export_dependencies(mavros_msgs)
ament_export_dependencies(rosidl_default_generators)
ament_export_dependencies(ament_cmake)
ament_export_dependencies(std_msgs)
ament_export_dependencies(rosidl_default_runtime)
ament_export_dependencies(airsim_interfaces)

ament_export_include_directories(${INCLUDE_DIRS})
ament_export_libraries(airsim_mavros)

ament_package()