add_executable(planning_robot
  planning_robot.cpp
  )
  
target_link_libraries(planning_robot 
  tf_helpers
  srrg2_navigation_2d_ros_library
  ${catkin_LIBRARIES}
  ${QGLViewer_LIBRARIES}
  ${SRRG_QT_LIBRARIES}
  ${GLUT_LIBRARIES})
  
add_executable(path_follower_app
  path_follower_app.cpp
  )
  
target_link_libraries(path_follower_app 
  tf_helpers
  srrg2_navigation_2d_ros_library
  ${catkin_LIBRARIES}
  ${QGLViewer_LIBRARIES}
  ${SRRG_QT_LIBRARIES}
  ${GLUT_LIBRARIES})
  
add_executable(collision_avoider
  collision_avoider.cpp
  )
  

target_link_libraries(
  collision_avoider
  tf_helpers
  ${catkin_LIBRARIES}
  srrg2_navigation_2d_ros_library
  )
  
  
add_executable(collision_avoider_class
  collision_avoider_class.cpp
  ${catkin_LIBRARIES}
  )

target_link_libraries(
  collision_avoider_class
  ${catkin_LIBRARIES}
  ${QGLViewer_LIBRARIES}
  ${SRRG_QT_LIBRARIES}
  ${GLUT_LIBRARIES}
  pthread
  srrg2_navigation_2d_ros_library
  )
  
  
add_executable(path_provider path_provider.cpp)
target_link_libraries(path_provider 
  tf_helpers
  ${catkin_LIBRARIES}
  )

add_executable(cinema_dumper 
  cinema_dumper.cpp 
  )
target_link_libraries(cinema_dumper
  tf_helpers   
  ${catkin_LIBRARIES}
  )