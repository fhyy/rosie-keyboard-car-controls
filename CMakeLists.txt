cmake_minimum_required(VERSION 2.8.3)
project(rosie_keyboard_controls)

find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs)

catkin_package(
  DEPENDS
  CATKIN_DEPENDS roscpp geometry_msgs
  LIBRARIES
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(rosie_keyboard_controls src/keyboard_controls_node.cpp)
target_link_libraries(rosie_keyboard_controls ${catkin_LIBRARIES})
add_dependencies(rosie_keyboard_controls std_msgs)
