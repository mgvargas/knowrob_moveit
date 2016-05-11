cmake_minimum_required(VERSION 2.8.3)
project(knowrob_moveit)
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS
  roscpp 
  moveit_core 
  message_generation
  message_runtime
  moveit_msgs
  sensor_msgs
  std_srvs
)

add_service_files(
  FILES
  CheckCollisions.srv
)

generate_messages(
  DEPENDENCIES
  sensor_msgs
  moveit_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES knowrob_collision_checking
  CATKIN_DEPENDS message_runtime moveit_msgs sensor_msgs std_srvs
#  DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS})

## Declare a C++ library
# add_library(knowrob_collision_checking
#   src/${PROJECT_NAME}/knowrob_collision_checking.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(knowrob_collision_checking ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(planning_scene_server src/${PROJECT_NAME}/planning_scene_server.cpp)
add_dependencies(planning_scene_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(planning_scene_server ${catkin_LIBRARIES})

add_executable(planning_scene_client src/${PROJECT_NAME}/planning_scene_client.cpp)
add_dependencies(planning_scene_client ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(planning_scene_client ${catkin_LIBRARIES})

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_knowrob_collision_checking.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
