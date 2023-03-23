add_library(srrg2_navigation_2d_library SHARED
  #common
  navigation_2d_base.cpp
  navigation_2d_base.h
  localizer_2d.cpp
  localizer_2d.h
  planner_2d_utils.cpp
  planner_2d_dmap.cpp
  planner_2d_search.cpp
  planner_2d_path.cpp
  planner_2d.cpp
  planner_2d.h
  instances.cpp
  instances.h
  obstacle_avoidance_base.cpp
  obstacle_avoidance_base.h
  local_path_planner.cpp
  local_path_planner.h
  )

target_link_libraries(srrg2_navigation_2d_library
  srrg2_system_utils_library
  srrg2_property_library
  srrg2_config_library
  srrg2_data_structures_library
  srrg2_point_cloud_library
  srrg2_image_library
  #ia potential gui stuff
  srrg2_viewer_library
  srrg2_messages_library
  ${catkin_LIBRARIES}
  )
