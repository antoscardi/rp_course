cmake_minimum_required(VERSION 2.8.3)
project(srrg2_orazio)

find_package(catkin REQUIRED COMPONENTS srrg_cmake_modules)
include(${srrg_cmake_modules_INCLUDE_DIRS}/CMakeCompileOptions.txt)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Werror=pedantic")
set(CMAKE_CC_FLAGS "${CMAKE_CC_FLAGS} -Werror=pedantic")
message("WARNING CXX FLAGS SET TO ${CMAKE_CXX_FLAGS}")
message("WARNING CC FLAGS SET TO ${CMAKE_CC_FLAGS}")
  
find_package(LibWEBSOCKETS REQUIRED)
find_package(Threads REQUIRED)

catkin_package(
  INCLUDE_DIRS src/orazio_host src/common
  LIBRARIES srrg2_orazio_library
  DEPENDS LibWEBSOCKETS
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  src/common
  src/orazio_host
)

## Declare a C++ library
 add_library(srrg2_orazio_library
   src/common/packet_handler.c
   src/common/deferred_packet_handler.c
   src/orazio_host/orazio_print_packet.c
   src/orazio_host/orazio_client.c
   src/orazio_host/serial_linux.c
 )

add_executable(orazio
  src/orazio_host/orazio.c
  src/orazio_host/orazio_shell_globals.c
  src/orazio_host/orazio_shell_commands.c
  src/orazio_host/orazio_ws_server.c
  src/orazio_host/ventry.c
  src/orazio_host/linenoise.c
  )

target_link_libraries(orazio
  srrg2_orazio_library
  ${CMAKE_THREAD_LIBS_INIT}
  ${LibWEBSOCKETS_LIBRARIES}
)

add_executable(orazio_client_test
  src/host_test/orazio_client_test.c
  src/host_test/orazio_client_test_getkey.c
)
target_link_libraries(orazio_client_test
  srrg2_orazio_library
  ${CMAKE_THREAD_LIBS_INIT}
)

