cmake_minimum_required(VERSION 2.8.3)
project(knowrob_moveit)

find_package(catkin REQUIRED roscpp moveit_core sensor_msgs)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES knowrob_collision_checking
#  CATKIN_DEPENDS other_catkin_pkg
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

add_executable(planning_scene_node src/${PROJECT_NAME}/collision_checker_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(knowrob_collision_checking_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(planning_scene_node ${catkin_LIBRARIES})

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_knowrob_collision_checking.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
