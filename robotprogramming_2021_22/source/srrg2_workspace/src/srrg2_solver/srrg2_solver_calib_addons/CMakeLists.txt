cmake_minimum_required(VERSION 3.0.2)
project(srrg2_solver_calib_addons)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  srrg_cmake_modules
  srrg2_core
  srrg2_solver
  )

include(${srrg_cmake_modules_INCLUDE_DIRS}/CMakeCompileOptions.txt)
message("${PROJECT_NAME}|compiling with these CXX flags: ${CMAKE_CXX_FLAGS}")

find_package(Eigen3 REQUIRED)
find_package(SuiteSparse REQUIRED)
find_package(Cholmod REQUIRED)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
  INCLUDE_DIRS
    src
  LIBRARIES
    srrg2_solver_calib_utils_library
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  # include
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

link_directories(${catkin_LIBRARY_DIRS})
add_subdirectory(src/srrg2_solver_calib_utils)

