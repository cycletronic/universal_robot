cmake_minimum_required(VERSION 3.0.2)
project(ath_msgs)

set(MSG_DEPS
  std_msgs
  moveit_msgs
  actionlib_msgs
  sensor_msgs
  geometry_msgs
  trajectory_msgs
  shape_msgs
  object_recognition_msgs
  octomap_msgs)

find_package(catkin REQUIRED COMPONENTS message_generation ${MSG_DEPS})

set(SRV_FILES
  GetTrajectory.srv
)

add_service_files(DIRECTORY srv FILES ${SRV_FILES})

generate_messages(DEPENDENCIES ${MSG_DEPS})

catkin_package(CATKIN_DEPENDS message_runtime ${MSG_DEPS})
