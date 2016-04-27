cmake_minimum_required(VERSION 2.8.3)
project(knowrob_moveit_collision_checker)

find_package(catkin REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES knowrob_collision_checking
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

include_directories(include)

## Declare a C++ library
# add_library(knowrob_collision_checking
#   src/${PROJECT_NAME}/knowrob_collision_checking.cpp
# )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
# add_dependencies(knowrob_collision_checking ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
# add_executable(knowrob_collision_checking_node src/knowrob_collision_checking_node.cpp)

## Add cmake target dependencies of the executable
## same as for the library above
# add_dependencies(knowrob_collision_checking_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
# target_link_libraries(knowrob_collision_checking_node
#   ${catkin_LIBRARIES}
# )

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_knowrob_collision_checking.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
