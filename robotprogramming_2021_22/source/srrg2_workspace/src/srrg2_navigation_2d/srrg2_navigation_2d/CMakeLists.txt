cmake_minimum_required(VERSION 3.0.2)
project(srrg2_navigation_2d)

find_package(catkin REQUIRED COMPONENTS
  srrg_cmake_modules
  srrg2_core
  )

include(${srrg_cmake_modules_INCLUDE_DIRS}/CMakeCompileOptions.txt)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror -pedantic")
message("${PROJECT_NAME}|compiling with these CXX flags: ${CMAKE_CXX_FLAGS}")

#ia find system stuff
find_package(Eigen3 REQUIRED)


#ia create a package
catkin_package(
  INCLUDE_DIRS
    src
  LIBRARIES
    srrg2_navigation_2d_library
  CATKIN_DEPENDS
  	srrg_cmake_modules
  	srrg2_core
)

#ds includes provided by this package (restrictive compile flags will be applied!)
include_directories(src)

#ds specify additional locations of header files
#ds treating them as system includes to suppress a massive wave of warnings originating from those header files (!)
include_directories(SYSTEM
  ${EIGEN3_INCLUDE_DIR}
  ${catkin_INCLUDE_DIRS}
)

link_directories(${catkin_LIBRARY_DIRS})
add_subdirectory(src)


