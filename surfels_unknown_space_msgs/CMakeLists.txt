cmake_minimum_required(VERSION 2.8.3)
project(surfels_unknown_space_msgs)

find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  std_msgs
  actionlib_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################

add_message_files(
  DIRECTORY msg
  FILES
  FrameState.msg
)

# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

add_action_files(
  DIRECTORY action
  FILES
  GetSurfelCloud.action
  SetSurfelCloud.action
  GetTimers.action
)

generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
  actionlib_msgs
)

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES elastic_unknown_info_msgs
#  CATKIN_DEPENDS sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

#############
## Install ##
#############

#############
## Testing ##
#############

