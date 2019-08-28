cmake_minimum_required(VERSION 2.8.3)
project(surfels_unknown_space_state_image_reader)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  pcl_conversions
  roscpp
  sensor_msgs
  surfels_unknown_space_msgs
  tf
  tf_conversions
  eigen_conversions
)

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES surfels_unknown_space_state_image_reader
#  CATKIN_DEPENDS pcl_conversions roscpp sensor_msgs surfels_unknown_space_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
)

add_executable(state_image_reader src/state_image_reader.cpp)

add_dependencies(state_image_reader ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(state_image_reader
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${Eigen3_LIBRARIES}
)

#############
## Install ##
#############

#############
## Testing ##
#############

