cmake_minimum_required(VERSION 2.8.3)
project(srrg_joystick_teleop)

find_package(catkin REQUIRED
  srrg_cmake_modules)

include(${srrg_cmake_modules_INCLUDE_DIRS}/CMakeCompileOptions.txt)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
)

catkin_package(
  CATKIN_DEPENDS nav_msgs roscpp
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)


## Declare a cpp executable
add_executable(joy_teleop_node 
  src/joy_teleop_node.cpp)

add_executable(joy_event_scanner 
  src/joy_event_scanner.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(joy_teleop_node
   ${catkin_LIBRARIES}
)
