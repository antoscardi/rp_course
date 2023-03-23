cmake_minimum_required(VERSION 2.8.3)
project(srrg2_hokuyo)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  srrg_cmake_modules
  roscpp
  sensor_msgs
  std_msgs
  )

# Find Eigen3
find_package(Eigen3 REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAMLCPP yaml-cpp REQUIRED)
if(YAMLCPP_VERSION VERSION_GREATER "0.5.0")
    add_definitions(-DHAVE_YAMLCPP_GT_0_5_0)
endif()
link_directories(${YAMLCPP_LIBRARY_DIRS})

#ds fight sloppyness: extend cmake compile flags to be more restrictive
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror -pedantic")

include(${srrg_cmake_modules_INCLUDE_DIRS}/CMakeCompileOptions.txt)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES srrg2_hokuyo
#  CATKIN_DEPENDS roscpp sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${YAMLCPP_INCLUDE_DIRS}
)

## Declare a cpp executable
add_executable(hokuyo_test src/hokuyo_test.c src/hokuyo.c)

## Declare a cpp executable
add_executable(hokuyo_node src/hokuyo_node.cpp src/hokuyo.c)


## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(srrg2_hokuyo_node srrg2_hokuyo_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(hokuyo_node
  ${YAMLCPP_LIBRARIES}
  ${catkin_LIBRARIES}
)
