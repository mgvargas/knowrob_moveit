cmake_minimum_required(VERSION 2.8.3)
project(knowrob_moveit_msgs)

find_package(catkin REQUIRED COMPONENTS
  message_generation 
  message_runtime 
  sensor_msgs 
  moveit_msgs
)

add_service_files(
  FILES
  CollisionCheck.srv
)

generate_messages(
  DEPENDENCIES
  sensor_msgs
  moveit_msgs
)

catkin_package(
  CATKIN_DEPENDS 
    message_runtime 
    sensor_msgs 
    moveit_msgs
)
