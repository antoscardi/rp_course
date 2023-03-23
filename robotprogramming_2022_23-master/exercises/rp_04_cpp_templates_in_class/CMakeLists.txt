CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

PROJECT(template_test)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3 -march=native" )

add_executable(mat_template_test src/mat_template_test.cpp)

add_executable(mat_f_test
  src/mat_f_test.cpp
  src/mat_f.cpp
  src/vec_f.cpp
  )

add_executable(binary_tree_template_test src/binary_tree_template_test.cpp)

add_executable(binary_tree_template_mys_test src/binary_tree_template_mys.cpp)
