cmake_minimum_required(VERSION 2.8.3)
project(srrg2_map_server)

find_package(catkin REQUIRED
        COMPONENTS
        srrg_cmake_modules
        srrg2_core
        roscpp
        nav_msgs
        tf2
        )

include(${srrg_cmake_modules_INCLUDE_DIRS}/CMakeCompileOptions.txt)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror=pedantic")
message("${PROJECT_NAME}|compiling with these CXX flags: ${CMAKE_CXX_FLAGS}")

find_package(Bullet REQUIRED)
find_package(Boost REQUIRED COMPONENTS filesystem)

find_package(OpenCV REQUIRED)
message("${PROJECT_NAME}|OpenCV version ${OpenCV_VERSION} found in (${OpenCV_DIR})")
message("${PROJECT_NAME}|OpenCV_INCLUDE_DIRS: ${OpenCV_INCLUDE_DIRS}")

# Find Eigen3
find_package(Eigen3 REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(YAMLCPP yaml-cpp REQUIRED)
if(YAMLCPP_VERSION VERSION_GREATER "0.5.0")
    add_definitions(-DHAVE_YAMLCPP_GT_0_5_0)
endif()
link_directories(${YAMLCPP_LIBRARY_DIRS})

catkin_package(
    INCLUDE_DIRS
        include
    LIBRARIES
        map_server_image_loader
    CATKIN_DEPENDS
        roscpp
        nav_msgs
        tf2
    DEPENDS
        OpenCV
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${YAMLCPP_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)

add_library(map_server_image_loader src/image_loader.cpp)
add_dependencies(map_server_image_loader ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(map_server_image_loader
    ${catkin_LIBRARIES}
    ${OpenCV_LIBS}
)

add_executable(map_server src/main.cpp)
add_dependencies(map_server ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(map_server
    map_server_image_loader
    ${YAMLCPP_LIBRARIES}
    ${OpenCV_LIBS}
    ${catkin_LIBRARIES}
)

add_executable(map_server-map_saver src/map_saver.cpp)
add_dependencies(map_server-map_saver ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
set_target_properties(map_server-map_saver PROPERTIES OUTPUT_NAME map_saver)
target_link_libraries(map_server-map_saver
    ${catkin_LIBRARIES}
)

# copy test data to same place as tests are run
function(copy_test_data)
    cmake_parse_arguments(PROJECT "" "" "FILES" ${ARGN})
    foreach(datafile ${PROJECT_FILES})
        file(COPY ${datafile} DESTINATION ${PROJECT_BINARY_DIR}/test)
    endforeach()
endfunction()

install(TARGETS map_server_image_loader
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

## Install project namespaced headers
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE)

## Install excutable python script
install(
    PROGRAMS
    scripts/crop_map
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
