cmake_minimum_required(VERSION 3.0.2)
project(srrg2_navigation_2d_ros)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  srrg2_core
  srrg2_core_ros
  srrg2_navigation_2d
  srrg2_navigation_2d_msgs
  #srrg2_qgl_viewport
  #srrg2_laser_slam_2d
  srrg_cmake_modules
  tf
  image_transport
)

include(${srrg_cmake_modules_INCLUDE_DIRS}/CMakeCompileOptions.txt)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall")
message("${PROJECT_NAME}|compiling with these CXX flags: ${CMAKE_CXX_FLAGS}")

#ia find system stuff
find_package(Eigen3 REQUIRED)
#find_package(QGLViewer REQUIRED)
#find_package(GLUT REQUIRED)

catkin_package(
  INCLUDE_DIRS
  src
  LIBRARIES
  srrg2_navigation_2d_ros_library
  tf_helpers_library
  CATKIN_DEPENDS
  srrg_cmake_modules
#  srrg2_qgl_viewport
# srrg2_laser_slam_2d
  srrg2_core_ros
  tf
  image_transport
  DEPENDS
  #   QGLViewer
  #   GLUT
)

###########
## Build ##
###########


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${PROJECT_SOURCE_DIR}/src
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

link_directories(${catkin_LIBRARY_DIRS})
add_subdirectory(src)
add_subdirectory(apps)
