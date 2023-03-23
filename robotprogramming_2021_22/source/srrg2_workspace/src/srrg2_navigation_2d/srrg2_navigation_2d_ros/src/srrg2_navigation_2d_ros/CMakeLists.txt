add_library(srrg2_navigation_2d_ros_library SHARED
  #common
  navigation_2d_map_requestor_ros.h
  navigation_2d_map_requestor_ros.cpp
  instances.cpp
  instances.h
  #motion_controller.cpp
  #motion_controller.h
  # scan_handler.cpp
  # scan_handler.h
  # path_follower.cpp
  # path_follower.h
  # target_planner.cpp
  # target_planner.h
  # callback_handler.cpp
  # callback_handler.h
  # controller.cpp
  # controller.h
  )
  
add_library(tf_helpers tf_helpers.cpp tf_helpers.h)

target_link_libraries(srrg2_navigation_2d_ros_library
  srrg2_navigation_2d_library
  srrg2_system_utils_library
  srrg2_converters_library
  srrg2_property_library
  srrg2_config_library
  srrg2_data_structures_library
  srrg2_image_library
  srrg2_viewer_library
  srrg2_messages_library
  tf_helpers
  ${catkin_LIBRARIES}
  )

