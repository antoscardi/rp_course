cmake_minimum_required(VERSION 2.8.3)
project(lesson_6)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)


include_directories(. /usr/include/eigen3)

add_executable(eigen_predicate eigen_predicate.cpp)
add_executable(eigen_transform_sort eigen_transform_sort.cpp)
add_executable(eigen_covariance_test eigen_covariance_test.cpp)
