cmake_minimum_required(VERSION 3.10)
project(exercise_2)

# invokes the routines to find a package called Eigen3
find_package(Eigen3 3.3 REQUIRED)

# TODO specify additional locations of header files
include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${PROJECT_SOURCE_DIR}/src
)

# add project source code
add_subdirectory(src)
# TODO do the same for executables
add_executable(exercise_2 src/linalg/vec_f_test.cpp src/linalg/vec_f.cpp src/linalg/vec_f.cpp src/linalg/vec_f.h)


