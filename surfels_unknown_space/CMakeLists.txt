cmake_minimum_required(VERSION 2.8.3)
project(surfels_unknown_space)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  eigen_conversions
  actionlib
  visualization_msgs
  rosbag
  surfels_unknown_space_msgs
)

find_package(rmonica_multi_timer QUIET)
if (${rmonica_multi_timer_FOUND})
  add_definitions(-DRMONICA_MULTI_TIMER_FOUND=1)
else()
  message(WARNING ${PROJECT_NAME}": rmonica_multi_timer not found, time statistics are disabled.")
endif()

find_package(Boost REQUIRED)
find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCL REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES elastic_unknown_info
#  CATKIN_DEPENDS elastic_bridge roscpp sensor_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

add_definitions(-DCL_HPP_MINIMUM_OPENCL_VERSION=110)
add_definitions(-DCL_HPP_TARGET_OPENCL_VERSION=120)

add_subdirectory("surfels_unknown_space_lib")

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${rmonica_multi_timer_INCLUDE_DIRS}
  ${surfels_unknown_space_lib_INCLUDE_DIRS}
)

add_executable(surfels_unknown_space 
  src/surfels_unknown_space_node.cpp
  src/surfels_unknown_space_visualization.cpp
  )

add_dependencies(surfels_unknown_space ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#message(FATAL_ERROR "surfels_unknown_space_lib_LIBRARIES ${surfels_unknown_space_lib_LIBRARIES}")

target_link_libraries(surfels_unknown_space
  ${catkin_LIBRARIES}
  ${rmonica_multi_timer_LIBRARIES}
  ${surfels_unknown_space_lib_LIBRARIES}
)

#############
## Install ##
#############

#############
## Testing ##
#############

