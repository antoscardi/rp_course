add_executable(app_benchmark app_benchmark.cpp)
target_link_libraries(app_benchmark
  srrg2_slam_interfaces_library
  srrg2_config_library
  srrg2_messages_library
  ${catkin_LIBRARIES}
)
