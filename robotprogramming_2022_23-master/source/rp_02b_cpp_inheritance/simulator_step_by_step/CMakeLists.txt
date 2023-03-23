CMAKE_MINIMUM_REQUIRED(VERSION 2.6)

PROJECT(simple_simulator)



# SET(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -DNDEBUG") 
# SET(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3 -DNDEBUG") 
# SET(CMAKE_BUILD_TYPE Release)

# Find Opencv
FIND_PACKAGE(OpenCV  REQUIRED)
INCLUDE_DIRECTORIES(${OpenCV_INCLUDE_DIRS})


add_executable(coso
  src/simple_geometry.cpp
  src/world.cpp
  src/robot.cpp
  src/lidar.cpp
  src/coso.cpp
  )

target_link_libraries(
  coso
  ${OpenCV_LIBS} 
)



