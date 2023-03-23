cmake_minimum_required(VERSION 2.8.3)
project(srrg2_orazio_ros)

find_package(catkin REQUIRED COMPONENTS
  srrg_cmake_modules
  sensor_msgs
  nav_msgs
  control_msgs
  geometry_msgs
  tf2
  tf2_ros
  roscpp
  srrg2_orazio
  )

# Find Eigen3
find_package(Eigen3 REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAMLCPP yaml-cpp REQUIRED)
if(YAMLCPP_VERSION VERSION_GREATER "0.5.0")
    add_definitions(-DHAVE_YAMLCPP_GT_0_5_0)
endif()
link_directories(${YAMLCPP_LIBRARY_DIRS})

include(${srrg_cmake_modules_INCLUDE_DIRS}/CMakeCompileOptions.txt)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES srrg2_orazio_ros
  CATKIN_DEPENDS nav_msgs roscpp srrg2_orazio
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${YAMLCPP_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(orazio_robot_node src/orazio_robot_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(orazio_robot_node
  ${YAMLCPP_LIBRARIES}
  ${catkin_LIBRARIES}
 )
