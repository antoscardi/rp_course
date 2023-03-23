cmake_minimum_required(VERSION 3.0.2)
project(srrg2_navigation_2d_msgs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  srrg_cmake_modules
  message_generation
)

include(${srrg_cmake_modules_INCLUDE_DIRS}/CMakeCompileOptions.txt)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror -pedantic")
message("${PROJECT_NAME}|compiling with these CXX flags: ${CMAKE_CXX_FLAGS}")

#ia find system stuff
find_package(Eigen3 REQUIRED)

add_message_files(
  FILES
  PathFollowerStatus.msg
  CollisionAvoiderStatus.msg
  LocalPathPlannerStatus.msg
  )

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  )

catkin_package(
  CATKIN_DEPENDS
  message_runtime
)


link_directories(${catkin_LIBRARY_DIRS})
